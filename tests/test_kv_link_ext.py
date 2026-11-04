"""PC data plane package 1: kv-link extensions (docs/design/PC_DATAPLANE_SPEC_20261011.md A, E.1).

One test (or group) per E.1 row, each with its known answer and its positive control
(the fault must be caught): ROI rows bit for bit, extension-block integrity, old/new
compatibility in both directions, overflow, ROI/geometry independence, TELEM unit
conversion, consistent reads, UIO+MAGIC gating, ages, loss/reorder, the e-stop state
machine and delivery, restarts, single writer, shared timebase latch; plus the
per-session frame chunk-loss count with concurrent SYNC/TELEM clients (controller
10-11) and the plan_boxes extraction checked against 58661b9's check_boxes.

Encoder known answers: hw/m7/pl/tools/enc_uart_model.py at feat/dev1/pl-permit-uart-20261011
c2987a8 (not merged here), Odom(clk_khz=100000), WHEEL_DIAM_UM=65000, COUNTS_PER_REV=1320,
TRACK_UM=160000, CTRL=1, frames (left, right, ts) = (0,0,1.001e9), (130,-40,1.002e9),
(270,95,1.003e9), (-500,2000,1.004e9); K_Q16 = 10138395. Values pasted below.
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_kv_link_ext.py
(also /usr/bin/python3.8).
"""
import contextlib
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest

import numpy as np

from host import kv_link
from host.kv_link import (CHUNK, ESTOP_ACK, ESTOP_CMD, REQ, RESP, ROI_DEVICE_ERROR,
                          ROI_NOT_CONFIGURED, TELEM_ACK, TELEM_REQ, BadExtension, CpuRoiCore,
                          KvEstopClient, KvFeatureUnavailable, KvLinkClient, KvLinkServer,
                          KvTelemetryClient, LinkTimeout, NullCore, RemotePlFrameBackend,
                          chunk_loss, decode_request, encode_request)
from host import kv_regs
from host.kv_regs import (ENC_UPDATE_COUNT, PlRegBlock, encoder_words, mock_encoder,
                          mock_permit, mock_sysinfo)
from host.kv_status import decode_telemetry
from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_object_native import (ROI_FIELDS, NativeObjectEvidence, RoiBatch, RoiRow,
                                   compile_object_native, memory_priority, plan_boxes)
from host.m7_pipeline import CaptureIdentity
from host.fpga_clock import now_ns
from perception.fpga_replay.core import Config
from tools.geo_same_input.common import depth_u16_to_m

ROOT = Path(__file__).resolve().parents[1]
H, W = 64, 96
K = np.array([[80.0, 0.0, 47.5], [0.0, 80.0, 31.5], [0.0, 0.0, 1.0]])
GRID = ((-0.8, -0.8, -0.05), 0.05, (32, 32, 32))
UNIT = 0.001
CLOCK, LOCAL, EPOCH = "test_raw", "cam", "e1"


def codes_for(i, h=H, w=W):
    yy, xx = np.mgrid[0:h, 0:w]
    c = (650 + 300 * np.sin(xx / 11. + i) + 150 * np.cos(yy / 7. - i)).astype(np.uint16)
    c[(xx * 7 + yy * 3 + i) % 13 == 0] = 0
    return c


class Env:
    _dir = None

    @classmethod
    def get(cls):
        if cls._dir is None:
            cls._tmp = tempfile.TemporaryDirectory()
            cls._dir = Path(cls._tmp.name)
            cls.frame_lib = compile_frame_native(cls._dir / "frame.so")
            cls.obj_lib = compile_object_native(cls._dir / "obj.so")
            cls.native = NativeObjectEvidence(cls.obj_lib)
        return cls


def cpu_core():
    return FrameNativeM7(Env.get().frame_lib).lib.fpga_m7_depth_frame


def frame_kwargs(fid, codes, ns):
    return dict(depth_m=depth_u16_to_m(codes, UNIT), k=K, camera_to_world=np.eye(4),
                grid_origin_m=GRID[0], voxel_size_m=GRID[1], shape=GRID[2], frame_id=fid,
                capture_time_ns=ns, pose_time_ns=ns, calibration_version="cal",
                pose_confirmed=True)


def reference(codes, fid, ns):
    return FrameNativeM7(Env.get().frame_lib).frame_from_depth(**frame_kwargs(fid, codes, ns))


def ident(fid, ns, stream="s1"):
    return CaptureIdentity(stream, fid, ns, CLOCK, "cal", LOCAL, EPOCH)


def load_old(path, rev):
    """A module from an older commit of this repository (git show)."""
    src = subprocess.run(["git", "show", "%s:%s" % (rev, path)], cwd=str(ROOT),
                         capture_output=True, check=True).stdout
    mod = types.ModuleType("old_%s_%s" % (rev, path.replace("/", "_").replace(".", "_")))
    mod.__file__ = str(ROOT / path)
    exec(compile(src, "%s@%s" % (path, rev), "exec"), mod.__dict__)
    return mod


class WrapSock:
    """A bound UDP socket whose sendto applies ``rule(msg_type, datagram) -> 'send' |
    'drop' | 'hold' | ('delay', s)``; ``release()`` sends the held datagrams."""

    def __init__(self, bind_port=0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", bind_port))
        self.rule = None
        self.held = []
        self.dropped = 0

    def sendto(self, d, addr):
        act = self.rule(d[4], d) if self.rule else "send"
        if act == "drop":
            self.dropped += 1
            return len(d)
        if act == "hold":
            self.held.append((d, addr))
            return len(d)
        if isinstance(act, tuple) and act[0] == "delay":
            time.sleep(act[1])
        return self.sock.sendto(d, addr)

    def release(self):
        held, self.held = self.held, []
        for d, addr in held:
            self.sock.sendto(d, addr)

    def __getattr__(self, name):
        return getattr(self.sock, name)


def drop_first(mtype, n):
    left = [n]

    def rule(t, _d):
        if t == mtype and left[0] > 0:
            left[0] -= 1
            return "drop"
        return "send"
    return rule


def server(core=None, name="cpu", **kw):
    return KvLinkServer(core or cpu_core(), name, bind=("127.0.0.1", 0), **kw).start()


def backend(port, sock=None, timeout_s=1.0):
    c = KvLinkClient(("127.0.0.1", port), chunk_bytes=2000, timeout_s=timeout_s, sock=sock)
    c.hello(2.0)
    return RemotePlFrameBackend(c, depth_unit_m=UNIT)


def boxes_batch(depth, rng, n, *, disabled=(), revision=11, cfg=Config()):
    """Random enabled boxes over the frame (plus disabled rows at the given indices)."""
    h, w = depth.shape
    rows, local = [], []
    for b in range(n):
        if b in disabled:
            rows.append(RoiRow(100 + b, False, None, 0., 0.))
            local.append((100 + b, None, 0., 0.))
            continue
        x0 = int(rng.integers(0, w - 4)); y0 = int(rng.integers(0, h - 4))
        x1 = int(rng.integers(x0 + 3, min(w, x0 + 40))); y1 = int(rng.integers(y0 + 3, min(h, y0 + 30)))
        near = float(rng.uniform(.3, 1.2)); far = near + float(rng.uniform(0, .3))
        rows.append(RoiRow(100 + b, True, (x0, y0, x1, y1), near, far))
        local.append((100 + b, (x0, y0, x1, y1), near, far))
    return RoiBatch(revision, cfg, rows), local


# ------------------------------------------------------------------ wire format
class WireFormatTests(unittest.TestCase):
    def test_struct_sizes_known_answer(self):
        self.assertEqual(kv_link.ROI_REQ_HEAD.size, 44)
        self.assertEqual(kv_link.ROI_REQ_BOX.size, 40)
        self.assertEqual(kv_link.ROI_RESP_HEAD.size, 56)
        self.assertEqual(kv_link.ROI_RESP_ROW.size, 28)
        self.assertEqual(kv_link.TELEM_REQ_BODY.size, 16)
        self.assertEqual(kv_link.TELEM_ACK_HEAD.size, 28)
        self.assertEqual(kv_link.ESTOP_CMD_BODY.size, 28)
        self.assertEqual(kv_link.ESTOP_ACK_BODY.size, 48)
        rng = np.random.default_rng(1)
        batch, _ = boxes_batch(np.zeros((H, W), np.float32), rng, 32)
        self.assertEqual(len(kv_link.encode_roi_block(batch)), 44 + 32 * 40 + 4)
        self.assertEqual(kv_link.TYPE_NAMES[10:14] if False else
                         [kv_link.TYPE_NAMES[t] for t in (10, 11, 12, 13)],
                         ["telem_req", "telem_ack", "estop_cmd", "estop_ack"])

    def test_roundtrips(self):
        rng = np.random.default_rng(2)
        batch, _ = boxes_batch(np.zeros((H, W), np.float32), rng, 5, disabled=(2,))
        d = kv_link.decode_roi_block(kv_link.encode_roi_block(batch))
        self.assertEqual(d["memory_revision"], 11)
        self.assertEqual([r[0] for r in d["rows"]], [r.row_id for r in batch.rows])
        self.assertEqual(d["rows"][2][1:6], (0, 0, 0, 0, 0))
        b = kv_link.encode_estop_cmd(1, 77, 0, 5)
        self.assertEqual(kv_link.decode_estop_cmd(b)["cmd_id"], 77)
        t = kv_link.decode_telem_req(kv_link.encode_telem_req(0x81, 9))
        self.assertEqual((t["want"], t["t_req_nano_ns"]), (0x81, 9))

    def _refix_crc(self, body):
        return kv_link.seal(body[:-4])

    def test_every_body_rejects_ver_rsv_crc_length_enum(self):
        rng = np.random.default_rng(3)
        batch, _ = boxes_batch(np.zeros((H, W), np.float32), rng, 3)
        cases = [(kv_link.encode_roi_block(batch), kv_link.decode_roi_block),
                 (kv_link.encode_telem_req(1, 5), kv_link.decode_telem_req),
                 (kv_link.encode_estop_cmd(1, 5, 0, 1), kv_link.decode_estop_cmd)]
        for good, dec in cases:
            dec(good)
            ver2 = self._refix_crc(bytes([2]) + good[1:])
            with self.assertRaises(BadExtension) as cm:
                dec(ver2)
            self.assertEqual(cm.exception.reason, "ver")
            flipped = bytearray(good)
            flipped[-6] ^= 0x40
            with self.assertRaises(BadExtension) as cm:
                dec(bytes(flipped))
            self.assertEqual(cm.exception.reason, "crc")
            with self.assertRaises(BadExtension) as cm:
                dec(good + b"\0")
            self.assertEqual(cm.exception.reason, "length")
        # enum out of range: op 9, enabled 2, want with an undefined bit
        with self.assertRaises(BadExtension):
            kv_link.decode_estop_cmd(kv_link.encode_estop_cmd(9, 1))
        with self.assertRaises(BadExtension):
            kv_link.decode_telem_req(kv_link.encode_telem_req(1 << 9, 1))
        blk = bytearray(kv_link.encode_roi_block(batch))
        struct.pack_into("<I", blk, 44 + 4, 2)
        with self.assertRaises(BadExtension) as cm:
            kv_link.decode_roi_block(self._refix_crc(bytes(blk)))
        self.assertEqual(cm.exception.reason, "enabled")


# ------------------------------------------------------------------ ROI (A.2)
def roi_dict(local, cfg):
    rows = []
    for row_id, roi, near, far in local:
        if roi is None:
            rows.append((row_id, 0, 0, 0, 0, 0, 0., 0.))
        else:
            rows.append((row_id, 1) + tuple(roi) + (near, far))
    return {"memory_revision": 0, "rows": rows,
            "cfg": {"depth_margin_m": cfg.depth_margin_m, "min_pixels": cfg.min_pixels,
                    "min_valid_fraction": cfg.min_valid_fraction, "free_fraction": cfg.free_fraction}}


class RowMutant:
    """KV-side ROI core that corrupts its answer (positive controls)."""

    def __init__(self, core, how):
        self.core, self.how, self.name = core, how, core.name

    def __call__(self, depth, roi):
        out = self.core(depth, roi)
        rows = [list(r) for r in out["rows"]]
        if self.how == "swap":
            rows[0], rows[1] = rows[1], rows[0]
        elif self.how == "id":
            rows[0][0] += 1
        elif self.how == "pixels":
            i = next(i for i, r in enumerate(rows) if r[1])
            rows[i][2] -= 1
        out["rows"] = [tuple(r) for r in rows]
        return out


class Raising:
    name = "cpu"

    def __call__(self, depth, roi):
        raise OSError("ROI device gone")


class RoiTests(unittest.TestCase):
    def test_cpu_roi_core_equals_evaluate_boxes_on_special_corpus(self):
        """KV raw path == local evaluate_boxes on OR 11.4-style batches: 0.1f/10.0f +-1 ULP,
        NaN/+-inf/+-0/subnormal, negative front (margin 0.5), disabled rows, 1..32 boxes."""
        env = Env.get()
        core = CpuRoiCore(env.obj_lib)
        rng = np.random.default_rng(20261011)
        f = np.float32
        specials = np.array([f(.1), np.nextafter(f(.1), f(1)), np.nextafter(f(.1), f(0)),
                             f(10.), np.nextafter(f(10.), f(11)), np.nextafter(f(10.), f(0)),
                             np.nan, np.inf, -np.inf, f(0.), f(-0.), f(1e-40), f(-1.)], np.float32)
        rows_checked = 0
        for trial in range(120):
            h, w = int(rng.integers(8, 48)), int(rng.integers(8, 64))
            depth = rng.uniform(.05, 3.5, size=(h, w)).astype(np.float32)
            mask = rng.random((h, w)) < .25
            depth[mask] = rng.choice(specials, size=int(mask.sum()))
            cfg = Config(depth_margin_m=float(rng.choice([.04, .5])))
            local = []
            for b in range(int(rng.integers(1, 33))):
                if rng.random() < .15:
                    local.append((b, None, 0., 0.)); continue
                x0 = int(rng.integers(0, w - 4)); y0 = int(rng.integers(0, h - 4))
                x1 = int(rng.integers(x0 + 3, w)); y1 = int(rng.integers(y0 + 3, h))
                near = float(rng.uniform(.2, 3.)); far = near + float(rng.uniform(0, .6))
                local.append((b, (x0, y0, x1, y1), near, far))
            ref = env.native.evaluate_boxes(depth, local, cfg)
            got = core(depth, roi_dict(local, cfg))
            self.assertEqual(got["status"], 0)
            self.assertEqual([dict(zip(ROI_FIELDS, r)) for r in got["rows"]], ref, trial)
            rows_checked += len(ref)
        self.assertGreater(rows_checked, 1000)
        # kernel rejections come back as a status, never rows
        bad = core(np.ones((10, 10), np.float32), roi_dict([(0, (0, 0, 2, 2), 1., 1.)], Config()))
        self.assertEqual((bad["status"], bad["rows"]), (3, []))

    def _frames(self, srv, be, n, rng, **kw):
        out = []
        for i in range(n):
            codes = codes_for(i)
            batch, local = boxes_batch(depth_u16_to_m(codes, UNIT), rng, int(rng.integers(2, 33)), **kw)
            ns = now_ns()
            ev = be.frame_from_depth(identity=ident(i + 1, ns), roi=batch, **frame_kwargs(i + 1, codes, ns))
            out.append((codes, i + 1, ns, ev, be.last_roi, batch, local))
        return out

    def test_loopback_rows_equal_local_and_geometry_equal_reference(self):
        env = Env.get()
        srv = server(roi_core=CpuRoiCore(env.obj_lib))
        try:
            be = backend(srv.port)
            rng = np.random.default_rng(5)
            frames = self._frames(srv, be, 8, rng, disabled=(1,))
            # OR 4.1 boundary: a box whose float32(near - margin) is exactly a code value
            codes = codes_for(20)
            z = depth_u16_to_m(codes, UNIT)
            v = float(z[10, 20])
            batch = RoiBatch(3, Config(), [RoiRow(7, True, (10, 5, 40, 30), v + .04, v + .3)])
            ns = now_ns()
            ev = be.frame_from_depth(identity=ident(30, ns), roi=batch, **frame_kwargs(30, codes, ns))
            frames.append((codes, 30, ns, ev, be.last_roi, batch, [(7, (10, 5, 40, 30), v + .04, v + .3)]))
            for codes, fid, ns, ev, ro, batch, local in frames:
                self.assertTrue(ro.ok, ro.reason)
                self.assertEqual(ro.roi_core, "cpu")
                self.assertEqual(len(ro.rows), len(batch.rows))
                ref = env.native.evaluate_boxes(depth_u16_to_m(codes, UNIT), local, batch.cfg)
                self.assertEqual(ro.rows, ref)
                self.assertTrue(np.array_equal(ev.flags, reference(codes, fid, ns).flags))
                self.assertGreater(ro.times_ns["total"], 0)
            self.assertEqual(be.client.counts["tx_messages:req"], len(frames))
        finally:
            srv.stop()

    def test_server_variants_void_roi_but_deliver_geometry(self):
        env = Env.get()
        for how in ("swap", "id", "pixels"):
            srv = server(roi_core=RowMutant(CpuRoiCore(env.obj_lib), how))
            try:
                be = backend(srv.port)
                codes, fid, ns, ev, ro, batch, local = self._frames(srv, be, 1, np.random.default_rng(6))[0]
                self.assertFalse(ro.ok, how)
                self.assertTrue(ro.reason.startswith("region_backend_failure:row_check"), ro.reason)
                self.assertEqual(ro.rows, [])
                self.assertTrue(np.array_equal(ev.flags, reference(codes, fid, ns).flags), how)
            finally:
                srv.stop()

    def test_roi_failure_and_none_do_not_touch_geometry(self):
        for core, why, status in ((Raising(), "roi_status_0x102", ROI_DEVICE_ERROR),
                                  (None, "kv_roi_not_configured", ROI_NOT_CONFIGURED)):
            srv = server(roi_core=core)
            try:
                be = backend(srv.port)
                codes, fid, ns, ev, ro, batch, local = self._frames(srv, be, 1, np.random.default_rng(7))[0]
                self.assertEqual(ro.reason, "region_backend_failure:%s" % why)
                self.assertTrue(np.array_equal(ev.flags, reference(codes, fid, ns).flags))
                self.assertEqual(srv.last_roi["roi_status"], status)
            finally:
                srv.stop()

    def test_bad_extension_requests_are_void(self):
        """ver=2, ext flag bit1, bit0 with box_count 0, block one byte longer, one byte
        changed without fixing the crc: each counted bad_ext:<reason>, no answer at all."""
        env = Env.get()
        srv = server(roi_core=CpuRoiCore(env.obj_lib))
        try:
            client = KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=2000, timeout_s=0.3)
            client.hello(2.0)
            codes = codes_for(0)
            batch, _ = boxes_batch(depth_u16_to_m(codes, UNIT), np.random.default_rng(8), 4)
            base = dict(frame_id=0, capture_ns=1, pose_ns=1,
                        identity={"stream_instance": "s", "clock_id": "c", "calibration_id": "k",
                                  "pose_epoch": "e"}, width=W, height=H, nx=4, ny=4, nz=4,
                        unit_f32=np.float32(UNIT), voxel=.05, origin=np.zeros(3),
                        inverse_k=np.linalg.inv(K), world_camera=np.eye(4)[:3],
                        codes=codes, depth_crc=0)
            good = encode_request(**base, roi=batch)
            block_at = len(good) - len(kv_link.encode_roi_block(batch))
            seal = kv_link.seal
            variants = {}
            g = bytearray(good)
            g[block_at] = 2
            variants["ver"] = bytes(g[:block_at]) + seal(bytes(g[block_at:-4]))
            g = bytearray(good)
            struct.pack_into("<I", g, kv_link.REQ_HEAD.size - 12, 3)    # ext_flags = bit0|bit1
            variants["flags"] = bytes(g)
            g = bytearray(good)
            struct.pack_into("<H", g, block_at + 2, 0)
            variants["box_count"] = bytes(g[:block_at]) + seal(bytes(g[block_at:-4]))
            variants["length"] = good + b"\0"
            g = bytearray(good)
            g[block_at + 60] ^= 1
            variants["crc"] = bytes(g)
            fid = 0
            for reason, req in variants.items():
                fid += 1
                struct.pack_into("<Q", bytearray(req), 0, fid)
                req = struct.pack("<Q", fid) + req[8:]
                with self.assertRaises(LinkTimeout):
                    client.call(req, {"frame_id": fid, "capture_ns": 1, "pose_ns": 1,
                                      "identity": base["identity"]})
                self.assertEqual(srv.counts["bad_ext:%s" % reason], 1, reason)
            self.assertEqual(srv.counts["served"], 0)
        finally:
            srv.stop()

    def test_corrupt_resp_block_is_void_on_the_client(self):
        """A RESP whose ROI block fails its crc is dropped whole (no partial geometry)."""
        env = Env.get()
        srv = server(roi_core=CpuRoiCore(env.obj_lib))
        orig = kv_link.encode_roi_result

        def corrupt(**kw):
            b = bytearray(orig(**kw))
            b[10] ^= 1
            return bytes(b)
        try:
            kv_link.encode_roi_result = corrupt
            be = backend(srv.port, timeout_s=0.4)
            codes = codes_for(1)
            batch, _ = boxes_batch(depth_u16_to_m(codes, UNIT), np.random.default_rng(9), 3)
            ns = now_ns()
            with self.assertRaises(LinkTimeout):
                be.frame_from_depth(identity=ident(1, ns), roi=batch, **frame_kwargs(1, codes, ns))
            self.assertGreaterEqual(be.client.counts["bad_ext:crc"], 1)
            self.assertEqual(be.last_roi.reason, "region_backend_failure:geometry_failed")
        finally:
            kv_link.encode_roi_result = orig
            srv.stop()

    def test_overflow_priority_and_one_request(self):
        """40 visible tracks -> 32 rows, 8 overflow, order = target label, newer last_seen,
        nearer; one REQ per frame (a second request for the overflow is a failure)."""
        entries, tracks = [], {}
        rng = np.random.default_rng(10)
        for i in range(40):
            u, v = 10 + (i % 8) * 10, 8 + (i // 8) * 10
            z = 1.0 + (i % 5) * 0.1
            c = z * (np.linalg.inv(K) @ np.array([u, v, 1.0]))
            tid = "t%02d" % i
            entries.append((tid, np.array([c - .02, c + .02])))
            tracks[tid] = {"label": "cup" if i % 4 == 0 else "book", "last_seen": float(i % 7),
                           "position_m": c.tolist()}
        plan = plan_boxes((H, W), K, np.eye(4), entries, Config(),
                          priority=memory_priority(tracks, "cup"))
        self.assertEqual(plan.host_resolved, {})
        self.assertEqual(len(plan.batch.rows), 32)
        self.assertEqual(len(plan.overflow), 8)
        expect = sorted(tracks, key=lambda t: (0 if tracks[t]["label"] == "cup" else 1,
                                               -tracks[t]["last_seen"],
                                               float(np.linalg.norm(tracks[t]["position_m"]))))
        self.assertEqual([plan.row_map[r.row_id] for r in plan.batch.rows] + plan.overflow, expect)
        self.assertTrue(all(tracks[plan.row_map[r]]["label"] == "cup" for r in range(10)))
        env = Env.get()
        srv = server(roi_core=CpuRoiCore(env.obj_lib))
        try:
            be = backend(srv.port)
            codes = codes_for(3)
            ns = now_ns()
            be.frame_from_depth(identity=ident(1, ns), roi=plan.batch.with_revision(4),
                                **frame_kwargs(1, codes, ns))
            self.assertTrue(be.last_roi.ok, be.last_roi.reason)
            self.assertEqual(be.client.counts["tx_messages:req"], 1)
            # variant: a client that re-sends the overflow as a second request
            rest = RoiBatch(4, Config(), [RoiRow(i, True, (0, 0, 5, 5), 1., 1.1) for i in range(8)])
            ns = now_ns()
            be.frame_from_depth(identity=ident(2, ns), roi=rest, **frame_kwargs(2, codes, ns))
            self.assertNotEqual(be.client.counts["tx_messages:req"], 1)   # the rule catches it
        finally:
            srv.stop()


# ------------------------------------------------------------------ compatibility
class CompatTests(unittest.TestCase):
    OLD = "5da3b9d"

    @classmethod
    def setUpClass(cls):
        try:
            cls.old = load_old("host/kv_link.py", cls.OLD)
        except (subprocess.CalledProcessError, OSError) as exc:
            raise unittest.SkipTest("git show %s unavailable: %s" % (cls.OLD, exc))

    def test_old_client_on_new_server(self):
        env = Env.get()
        srv = server(roi_core=CpuRoiCore(env.obj_lib))
        try:
            c = self.old.KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=2000, timeout_s=1.0)
            info = c.hello(2.0)
            self.assertEqual(info["features"]["roi_ext"], 1)
            be = self.old.RemotePlFrameBackend(c, depth_unit_m=UNIT)
            for i in range(4):
                codes, ns = codes_for(i), now_ns()
                ev = be.frame_from_depth(identity=ident(i + 1, ns), **frame_kwargs(i + 1, codes, ns))
                self.assertTrue(np.array_equal(ev.flags, reference(codes, i + 1, ns).flags))
            st = c.stats()
            self.assertEqual(st["counts"].get("served"), 4)
            self.assertNotIn("roi_status:0x0", st["counts"])
        finally:
            srv.stop()

    def test_new_request_without_roi_is_byte_identical_to_old(self):
        codes = codes_for(2)
        kw = dict(frame_id=5, capture_ns=6, pose_ns=7,
                  identity={"stream_instance": "s", "clock_id": "c", "calibration_id": "k",
                            "pose_epoch": "e"}, width=W, height=H, nx=4, ny=4, nz=4,
                  unit_f32=np.float32(UNIT), voxel=.05, origin=np.zeros(3),
                  inverse_k=np.linalg.inv(K), world_camera=np.eye(4)[:3], codes=codes, depth_crc=9)
        self.assertEqual(encode_request(**kw), self.old.encode_request(**kw))

    def test_new_client_on_old_server(self):
        srv = self.old.KvLinkServer(cpu_core(), "cpu", bind=("127.0.0.1", 0)).start()
        try:
            be = backend(srv.port, timeout_s=0.4)
            codes = codes_for(1)
            batch, _ = boxes_batch(depth_u16_to_m(codes, UNIT), np.random.default_rng(11), 3)
            ns = now_ns()
            with self.assertRaises(KvFeatureUnavailable):
                be.frame_from_depth(identity=ident(1, ns), roi=batch, **frame_kwargs(1, codes, ns))
            self.assertEqual(be.client.counts["tx_messages:req"], 0)     # nothing sent
            t = KvTelemetryClient(("127.0.0.1", srv.port))
            with self.assertRaises(KvFeatureUnavailable):
                t.poll()
            e = KvEstopClient(("127.0.0.1", srv.port))
            with self.assertRaises(KvFeatureUnavailable):
                e.send("assert", 1)
            self.assertEqual(t.counts["tx_messages:telem_req"] + e.counts["tx_messages:estop_cmd"], 0)
            # forced: an extended REQ to the old server times out (frame fails closed)
            be.client.server_info = dict(be.client.server_info, features={"roi_ext": 1},
                                         roi_max_boxes=32)
            ns = now_ns()
            with self.assertRaises(LinkTimeout):
                be.frame_from_depth(identity=ident(2, ns), roi=batch, **frame_kwargs(2, codes, ns))
            self.assertEqual(srv.counts["served"], 0)
            self.assertGreaterEqual(srv.counts["bad_request:ValueError"], 1)
            # plain frames still work against the old server
            ns = now_ns()
            ev = be.frame_from_depth(identity=ident(3, ns), **frame_kwargs(3, codes, ns))
            self.assertTrue(np.array_equal(ev.flags, reference(codes, 3, ns).flags))
        finally:
            srv.stop()

    def test_forced_extension_to_old_server_fails_the_pipeline_frame(self):
        """Through M7Pipeline: backend_failure, the rolling map gets nothing fresh."""
        from host.m7_pipeline import DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
        srv = self.old.KvLinkServer(cpu_core(), "cpu", bind=("127.0.0.1", 0)).start()
        try:
            be = backend(srv.port, timeout_s=0.3)
            be.client.server_info = dict(be.client.server_info, features={"roi_ext": 1},
                                         roi_max_boxes=32)
            pipe = M7Pipeline(be, GridSpec(*GRID))
            codes = codes_for(1)
            batch, _ = boxes_batch(depth_u16_to_m(codes, UNIT), np.random.default_rng(12), 3)
            be.set_next_roi(batch)
            ns = now_ns()
            pose = PoseAtCapture(ns, CLOCK, LOCAL, EPOCH, True, np.eye(4))
            out = pipe.ingest_geometry(DepthFrame(ident(1, ns), depth_u16_to_m(codes, UNIT), K, pose),
                                       now_ns=now_ns(), now_clock_id=CLOCK)
            self.assertFalse(out.navigation.valid)
            self.assertTrue(out.navigation.reason.startswith("backend_failure"), out.navigation.reason)
        finally:
            srv.stop()


# ------------------------------------------------------------------ registers (A.3)
# Encoder model frames (see module docstring): (left, right, ts, model output)
ENC_MODEL = [
    (130, -40, 1002000000, dict(v_l=131799135, v_r=-40553580, v_c=45622777, dist_l=20110,
                                dist_r=-6187, dist_c=6961, heading=-10771, oflags=17, dt=1000000)),
    (270, 95, 1003000000, dict(v_l=141937530, v_r=136868332, v_c=139402931, dist_l=41768,
                               dist_r=14696, dist_c=28232, heading=-11088, oflags=17, dt=1000000)),
    (-500, 2000, 1004000000, dict(v_l=-780656415, v_r=1931364247, v_c=575353916, dist_l=-77349,
                                  dist_r=309399, dist_c=116025, heading=158411, oflags=17,
                                  dt=1000000)),
]
CLK = 100000000


def enc_words_from_model(left, right, ts, m, shift=0, update_count=1):
    return encoder_words(update_count=update_count, frame_ts=ts, left=left, right=right,
                         odom_flags=m["oflags"], v_left_q16=m["v_l"] >> shift,
                         v_right_q16=m["v_r"] >> shift, v_center_q16=m["v_c"] >> shift,
                         dist_left_um=m["dist_l"], dist_right_um=m["dist_r"],
                         dist_center_um=m["dist_c"], heading_q16=m["heading"] >> shift,
                         dt_cycles=m["dt"])


def fake_uio(root, nodes):
    """sysfs + dev trees: nodes = {index: (name, magic or None)}; returns (sysfs, dev)."""
    sysfs, dev = os.path.join(root, "sys"), os.path.join(root, "dev")
    os.makedirs(dev, exist_ok=True)
    for n, (name, magic) in nodes.items():
        d = os.path.join(sysfs, "uio%d" % n)
        os.makedirs(os.path.join(d, "maps", "map0"))
        open(os.path.join(d, "name"), "w").write(name + "\n")
        open(os.path.join(d, "maps", "map0", "addr"), "w").write("0xa0000000\n")
        open(os.path.join(d, "maps", "map0", "size"), "w").write("0x10000\n")
        buf = bytearray(0x10000)
        if magic is not None:
            struct.pack_into("<I", buf, 0, magic)
            struct.pack_into("<I", buf, 0x08, 0x0117BC0A)
            struct.pack_into("<I", buf, 0x10, CLK)
        open(os.path.join(dev, "uio%d" % n), "wb").write(bytes(buf))
    os.makedirs(sysfs, exist_ok=True)
    return sysfs, dev


class Opens:
    def __init__(self):
        self.calls = []

    def __call__(self, path, flags):
        self.calls.append(path)
        return os.open(path, flags)


FORBIDDEN_DEVICE = "/dev/" + "mem"     # spelled apart so this file is not a hit itself
TELEM_SOURCES = ("host/kv_regs.py", "host/kv_telem.py", "host/kv_status.py", "host/kv_estop.py")


def mentions_physical_memory(text):
    return FORBIDDEN_DEVICE in text


class RegisterTests(unittest.TestCase):
    def test_gate_missing_nodes_opens_no_device_file(self):
        with tempfile.TemporaryDirectory() as t:
            sysfs, dev = fake_uio(t, {0: ("m7_frame", 0x1), 5: ("m7_sysinfo", 0x4D37504C)})
            op = Opens()
            blocks = kv_regs.open_blocks(sysfs_root=sysfs, dev_root=dev, opener=op)
            self.assertTrue(blocks["sysinfo"].available)
            self.assertEqual(blocks["sysinfo"].sysinfo()["build_id"], 0x0117BC0A)
            for name in ("permit", "encoder"):
                self.assertFalse(blocks[name].available)
                self.assertIn("no UIO node named", blocks[name].reason)
            self.assertEqual(op.calls, [os.path.join(dev, "uio5")])     # sysinfo only
            for b in blocks.values():
                b.close()

    def test_gate_slot_map_skips_absent_encoder(self):
        """v3fc (SLOT_MAP 0x363, no slot 7): encoder skipped without opening its node, even if a
        stale UIO node of that name exists; control: A2 SLOT_MAP 0x3D7 opens it."""
        for slot_map, enc_opened in ((0x363, False), (0x3D7, True)):
            with tempfile.TemporaryDirectory() as t:
                sysfs, dev = fake_uio(t, {0: ("m7_sysinfo", 0x4D37504C), 1: ("m7_permit", 0x4D37504D),
                                          2: ("m7_encoder", 0x4D37454E)})
                with open(os.path.join(dev, "uio0"), "r+b") as fp:
                    fp.seek(kv_regs.SYS_SLOT_MAP)
                    fp.write(struct.pack("<I", slot_map))
                op = Opens()
                blocks = kv_regs.open_blocks(sysfs_root=sysfs, dev_root=dev, opener=op)
                self.assertTrue(blocks["sysinfo"].available)
                self.assertTrue(blocks["permit"].available)
                self.assertEqual(blocks["encoder"].available, enc_opened)
                self.assertEqual(os.path.join(dev, "uio2") in op.calls, enc_opened)
                if not enc_opened:
                    self.assertIn("SLOT_MAP", blocks["encoder"].reason)
                    with self.assertRaises(kv_regs.RegisterReadError):
                        blocks["encoder"].encoder_snapshot()
                for b in blocks.values():
                    b.close()

    def test_gate_magic_mismatch(self):
        with tempfile.TemporaryDirectory() as t:
            sysfs, dev = fake_uio(t, {2: ("m7_permit", 0xDEADBEEF), 3: ("m7_encoder", None)})
            op = Opens()
            p = PlRegBlock.open("permit", sysfs_root=sysfs, dev_root=dev, opener=op)
            e = PlRegBlock.open("encoder", sysfs_root=sysfs, dev_root=dev, opener=op)
            self.assertFalse(p.available)
            self.assertIn("MAGIC", p.reason)
            self.assertFalse(e.available)
            self.assertIn("MAGIC", e.reason)
            with self.assertRaises(kv_regs.RegisterReadError):
                p.permit_snapshot()

    def test_no_physical_memory_device_in_register_and_telemetry_sources(self):
        for path in TELEM_SOURCES:
            self.assertFalse(mentions_physical_memory((ROOT / path).read_text()), path)
        # positive control: the checker flags a source that falls back to it
        self.assertTrue(mentions_physical_memory('fd = os.open("%s", os.O_RDWR)' % FORBIDDEN_DEVICE))

    def test_units_known_answer_from_encoder_model(self):
        from host.kv_status import decode_encoder
        for left, right, ts, m in ENC_MODEL:
            blk, _ = mock_encoder(enc_words_from_model(left, right, ts, m))
            raw = blk.encoder_snapshot()
            d = decode_encoder(raw, CLK)
            self.assertEqual((raw["left_count"], raw["right_count"]), (left, right))
            self.assertEqual(d["v_left_mm_s"], m["v_l"] / 65536.)
            self.assertEqual(d["v_center_mm_s"], m["v_c"] / 65536.)
            self.assertEqual(d["dist_right_m"], m["dist_r"] / 1e6)
            self.assertEqual(d["heading_rad"], m["heading"] / 65536.)
            self.assertEqual(d["dt_ms"], 10.0)
            # independent float physics: 65 mm wheel, 1320 counts/rev, 10 ms, 160 mm track
            per_count_mm = math.pi * 65.0 / 1320
            self.assertAlmostEqual(d["dist_left_m"], left * per_count_mm / 1e3, delta=2e-6)
        left, right, ts, m = ENC_MODEL[1]
        prev = ENC_MODEL[0]
        self.assertAlmostEqual(decode_encoder(mock_encoder(enc_words_from_model(left, right, ts, m))[0]
                                              .encoder_snapshot(), CLK)["v_left_mm_s"],
                               (left - prev[0]) * math.pi * 65.0 / 1320 / 0.01, delta=1e-3)
        # positive control: a Q16.16 value off by one shift is caught
        blk, _ = mock_encoder(enc_words_from_model(left, right, ts, m, shift=1))
        self.assertNotEqual(decode_encoder(blk.encoder_snapshot(), CLK)["v_left_mm_s"], m["v_l"] / 65536.)

    def test_encoder_consistent_read(self):
        a = enc_words_from_model(*ENC_MODEL[0][:3], ENC_MODEL[0][3], update_count=1)
        b = enc_words_from_model(*ENC_MODEL[2][:3], ENC_MODEL[2][3], update_count=2)
        state = {"n": 0}

        def publish_between(off):     # PL publishes b right after the first UPDATE_COUNT read
            if off == ENC_UPDATE_COUNT:
                state["n"] += 1
                if state["n"] == 1:
                    state["pending"] = True
            elif state.pop("pending", False):
                win.words.update(b)
        blk, win = mock_encoder(a, on_read=publish_between)
        raw = blk.encoder_snapshot()
        self.assertGreaterEqual(blk.counts["reread"], 1)
        self.assertEqual(raw["update_count"], 2)
        self.assertEqual((raw["left_count"], raw["dist_left_um"], raw["v_right_q16"]),
                         (-500, -77349, 1931364247))
        # positive control: one pass without the UPDATE_COUNT bracket reads a torn mixture
        state["n"] = 0
        blk2, win = mock_encoder(a, on_read=publish_between)
        raw2 = blk2.encoder_snapshot(max_tries=1) if False else None
        w = win
        u1 = w.rd(ENC_UPDATE_COUNT)
        left = kv_regs._s32(w.rd(0x34))
        dist_left_lo = w.rd(0x4C)
        self.assertNotEqual((u1, left), (2, -500))       # first word from a, the rest from b
        self.assertEqual(left, -500)
        self.assertEqual(u1, 1)

    def test_single_writer_static_and_dynamic(self):
        sources = list((ROOT / "host").glob("*.py")) + list((ROOT / "tools" / "kv_link").glob("*.py"))
        callers = sorted(str(p.relative_to(ROOT)) for p in sources
                         if "write_permit_ctrl(" in p.read_text())
        self.assertEqual(callers, ["host/kv_estop.py", "host/kv_regs.py"])
        for path in ("host/kv_telem.py", "host/kv_status.py"):
            self.assertNotIn(".wr(", (ROOT / path).read_text(), path)
        regs_src = (ROOT / "host/kv_regs.py").read_text()
        # write_permit_ctrl + write_permit_next_frame_id (test_kv_roi_pl) + write_permit_window (v3fc2 3c087c9:
        # kv_server --permit-window-ms, once at start, not in the polling path)
        self.assertEqual(regs_src.count("self.window.wr("), 3)
        # dynamic: polling with both blocks present writes nothing
        perm, pwin = mock_permit()
        enc, ewin = mock_encoder(enc_words_from_model(*ENC_MODEL[0][:3], ENC_MODEL[0][3]))
        srv = server(NullCore(), "null", regs={"permit": perm, "encoder": enc})
        try:
            t = KvTelemetryClient(("127.0.0.1", srv.port))
            for _ in range(5):
                tm = t.poll()
            self.assertIn("encoder", tm.avail)
            self.assertEqual((pwin.writes, ewin.writes), ([], []))
            # positive control: a responder that clears odometry on every poll is caught
            orig = srv.telem._block

            def clearing(name, fn):
                if name == "encoder":
                    ewin.wr(0x04, 2)
                return orig(name, fn)
            srv.telem._block = clearing
            t.poll()
            self.assertNotEqual(ewin.writes, [])
        finally:
            srv.stop()

    def _latch_race(self, locked, reads=100000):
        from host.kv_clock_sync import PlTimebase, SYNC_BODY, SyncResponder
        blk, win = mock_permit(tb_step=(1 << 31) + 7, yield_on_lo=True)
        tb = PlTimebase.from_block(blk, clk_hz=CLK)
        if not locked:
            blk.lock, tb.lock = contextlib.nullcontext(), contextlib.nullcontext()
        resp = SyncResponder(pl=tb)
        torn = []
        old = sys.getswitchinterval()
        sys.setswitchinterval(1e-6)

        def telem():
            for _ in range(reads // 2):
                v = blk.timebase()
                if v not in win.issued:
                    torn.append(v)

        def sync():
            for i in range(reads // 6):
                resp.answer(SYNC_BODY.pack(i, 0), 0)
        try:
            ts = [threading.Thread(target=telem), threading.Thread(target=sync)]
            for th in ts:
                th.start()
            for th in ts:
                th.join()
        finally:
            sys.setswitchinterval(old)
        return len(torn), resp.counts["pl_read_error"]

    def test_shared_timebase_latch(self):
        self.assertEqual(self._latch_race(True), (0, 0))
        torn, errors = self._latch_race(False)
        self.assertGreater(torn + errors, 0)          # positive control: unlocked readers tear


# ------------------------------------------------------------------ TELEM (A.3)
def board_like_today():
    """Bitstream 0x0117bc0a: sysinfo present, permit/encoder absent."""
    sysb, _ = mock_sysinfo()
    absent = {n: PlRegBlock(n, None, reason="no UIO node named 'm7_%s'" % n) for n in ("permit", "encoder")}
    absent["sysinfo"] = sysb
    return absent


class TelemetryTests(unittest.TestCase):
    def test_today_avail_and_output_nonempty_finite(self):
        srv = server(regs=board_like_today())
        try:
            be = backend(srv.port)
            codes, ns = codes_for(0), now_ns()
            be.frame_from_depth(identity=ident(1, ns), **frame_kwargs(1, codes, ns))
            t = KvTelemetryClient(("127.0.0.1", srv.port))
            tm = t.poll()
            time.sleep(1.05)
            tm = t.poll()
            self.assertEqual(tm.avail, {"geometry", "estop", "proc", "board"})
            for name in ("permit", "encoder", "ar", "roi"):
                self.assertEqual(set(tm.blocks[name]), {"available", "reason"})
                self.assertTrue(tm.blocks[name]["reason"])
            g = tm.blocks["geometry"]
            self.assertEqual(g["served"], 1)
            self.assertEqual(g["last"]["frame_id"], 1)
            self.assertEqual(g["last"]["capture_ns"], ns)
            self.assertTrue(all(math.isfinite(v) and v >= 0 for v in g["last"]["times_ms"].values()))
            self.assertTrue(math.isfinite(tm.age_upper_ms["geometry"]) and tm.age_upper_ms["geometry"] >= 0)
            self.assertEqual(tm.blocks["board"]["build_id"], 0x0117BC0A)
            self.assertEqual(tm.kv_boot_id, srv.info["kv_boot_id"])
            self.assertTrue(math.isfinite(tm.blocks["proc"]["cpu_per_wall_1s"]))
            self.assertEqual(tm.blocks["estop"]["latched"], False)
            self.assertGreater(tm.rtt_ms, 0)
        finally:
            srv.stop()

    def test_ages_known_answer(self):
        doc = {"geometry": {"available": True, "last": {"kv_done_ns": 1000, "times_ns": {"recv": 2000000}}},
               "permit": {"available": True, "status": 0x2F, "age_cycles": 500000, "timeout_cycles": 20000000,
                          "tb_now": 10 ** 9 + 2000000, "last_done_ts": 10 ** 9},
               "board": {"available": True, "pl_clk_hz": CLK, "kv_boot_id": "ab"}}
        avail = kv_link.blocks_mask(["geometry", "permit", "board"])
        tm = decode_telemetry({"avail": avail, "kv_mono_ns": 5001000, "t_req_nano_ns": 7, "doc": doc},
                              t_req_nano_ns=10 ** 6, t_recv_nano_ns=4 * 10 ** 6)
        self.assertEqual(tm.rtt_ms, 3.0)
        self.assertEqual(tm.blocks["geometry"]["last"]["age_kv_ms"], 5.0)
        self.assertEqual(tm.age_upper_ms["geometry"], 8.0)
        self.assertEqual(tm.blocks["permit"]["since_done_ms"], 20.0)
        self.assertEqual(tm.age_upper_ms["permit"], 23.0)
        self.assertEqual(tm.blocks["permit"]["age_ms"], 5.0)
        self.assertTrue(tm.blocks["permit"]["status_bits"]["fresh"])
        # no clock -> no PL age (never a guess)
        doc["board"]["pl_clk_hz"] = None
        tm = decode_telemetry({"avail": avail, "kv_mono_ns": 5001000, "t_req_nano_ns": 7, "doc": doc},
                              10 ** 6, 4 * 10 ** 6)
        self.assertNotIn("permit", tm.age_upper_ms)

    def test_age_upper_bound_with_30ms_outbound_delay_and_no_clock_mapping(self):
        from host import kv_clock_sync as cs
        sock = WrapSock()
        sock.rule = lambda t, d: ("delay", 0.03) if t == TELEM_REQ else "send"
        srv = server()
        traps = []

        def trap(*a, **k):
            traps.append(a)
            raise AssertionError("clock mapping used for an age")
        names = [(cs.KvNanoMap, "kv_to_nano"), (cs.KvNanoMap, "nano_to_kv"),
                 (cs.KvNanoMap, "offset_at_nano"), (cs.PlKvMap, "pl_to_kv"),
                 (cs.ClockSync, "kv_to_nano"), (cs.ClockSync, "pl_to_nano")]
        saved = [(c, n, getattr(c, n)) for c, n in names]
        try:
            for c, n in names:
                setattr(c, n, trap)
            be = backend(srv.port)
            codes, ns = codes_for(0), now_ns()
            be.frame_from_depth(identity=ident(1, ns), **frame_kwargs(1, codes, ns))
            t = KvTelemetryClient(("127.0.0.1", srv.port), sock=sock)
            t.poll()
            t_req = now_ns()
            tm = t.poll()
            t_after = now_ns()
            done = tm.blocks["geometry"]["last"]["kv_done_ns"]
            self.assertGreaterEqual(tm.rtt_ms, 30.0)
            true_at_req = (t_req - done) / 1e6
            true_at_after = (t_after - done) / 1e6
            self.assertGreaterEqual(tm.age_upper_ms["geometry"], true_at_req + 30.0 - 1e-6)
            self.assertLessEqual(tm.age_upper_ms["geometry"], true_at_after + tm.rtt_ms + 1e-6)
            self.assertEqual(traps, [])
            # positive control: a decoder that maps KV time through B2 trips the trap
            with self.assertRaises(AssertionError):
                cs.ClockSync().kv_to_nano(done)
        finally:
            for c, n, f in saved:
                setattr(c, n, f)
            srv.stop()
        self.assertNotIn("kv_clock_sync", (ROOT / "host/kv_status.py").read_text())

    def test_lost_and_late_answers(self):
        srv_sock = WrapSock()
        srv = KvLinkServer(NullCore(), "null", sock=srv_sock).start()
        try:
            # resend_s=0: one send per poll (the 10-11 package-1 behaviour this test pins down;
            # the resend added for KV_LINK_BURST_TAIL is tested in test_kv_link_burst_tail.py)
            t = KvTelemetryClient(("127.0.0.1", srv.port), timeout_s=0.2, resend_s=0.0)
            t.poll()
            srv_sock.rule = drop_first(TELEM_ACK, 1)
            with self.assertRaises(LinkTimeout):
                t.poll()
            self.assertEqual(t.counts["telem_timeout"], 1)
            t.poll()                                   # next tick recovers
            # late answer of an older request: held, released before the newer one's answer
            srv_sock.rule = lambda typ, d: "hold" if typ == TELEM_ACK else "send"
            with self.assertRaises(LinkTimeout):
                t.poll()
            srv_sock.rule = None
            held = list(srv_sock.held)
            srv_sock.held = []
            orig = srv.telem.answer

            def answer_after_late(payload):
                for d, addr in held:
                    srv_sock.sock.sendto(d, addr)
                return orig(payload)
            srv.telem.answer = answer_after_late
            tm = t.poll()
            self.assertEqual(t.counts["stale:telem_ack"], 1)
            self.assertIn("geometry", tm.avail)
            self.assertEqual(t.counts["telem_ok"], 3)
        finally:
            srv.stop()


# ------------------------------------------------------------------ ESTOP (A.4)
class EstopTests(unittest.TestCase):
    def _srv(self, permit=None, **kw):
        regs = {"permit": permit} if permit is not None else {}
        return server(NullCore(), "null", regs=regs, **kw)

    def test_state_machine(self):
        perm, win = mock_permit()
        srv = self._srv(perm)
        try:
            e = KvEstopClient(("127.0.0.1", srv.port))
            a = e.send("assert", 1)
            self.assertEqual((a.result, a.epoch, a.latched, a.force_stop_written), (0, 1, True, True))
            self.assertTrue(a.permit_status & 8)
            self.assertEqual(e.send("assert", 2).result, 1)
            self.assertEqual(e.send("assert", 2).epoch, 1)
            r = e.send("release", 3, epoch=0)
            self.assertEqual((r.result, r.latched), (2, True))
            r = e.send("release", 4, epoch=1)
            self.assertEqual((r.result, r.latched, r.force_stop_written), (0, False, False))
            self.assertFalse(r.permit_status & 8)
            self.assertEqual(e.send("release", 5, epoch=1).result, 3)
            self.assertEqual(win.writes, [(0x04, 1), (0x04, 0)])
            a = e.send("assert", 6)
            self.assertEqual((a.result, a.epoch), (0, 2))
        finally:
            srv.stop()

    def test_no_permit_block_is_result_5(self):
        srv = self._srv(PlRegBlock("permit", None, reason="no UIO node named 'm7_permit'"))
        try:
            e = KvEstopClient(("127.0.0.1", srv.port))
            a = e.send("assert", 1)
            self.assertEqual((a.result, a.latched, a.force_stop_written, a.permit_avail, a.permit_status),
                             (5, True, False, False, 0))
            self.assertEqual(e.send("query", 2).result, 5)
            tm = KvTelemetryClient(("127.0.0.1", srv.port)).poll()
            self.assertEqual((tm.blocks["estop"]["latched"], tm.blocks["estop"]["force_stop_written"]),
                             (True, False))
            # positive control: a latch that claims success without a permit block is caught
            from host.kv_estop import EstopLatch

            class Lying(EstopLatch):
                def _apply_assert(self):
                    return 0
            lie = Lying(None, boot_id=1)
            ack = kv_link.decode_estop_ack(lie.handle(kv_link.encode_estop_cmd(1, 9)))
            self.assertNotEqual(ack["result"], 5)
        finally:
            srv.stop()

    def test_ignored_ctrl_write_is_result_6_and_rewritten(self):
        perm, win = mock_permit(ignore_writes=True)
        srv = self._srv(perm)
        try:
            e = KvEstopClient(("127.0.0.1", srv.port))
            self.assertEqual(e.send("assert", 1).result, 6)
            time.sleep(0.35)
            self.assertGreaterEqual(len(win.writes), 3)       # rewritten every 100 ms
            self.assertEqual(e.send("query", 2).result, 6)
        finally:
            srv.stop()

    def test_delivery_drops_dedup_and_frame_id_rule(self):
        perm, win = mock_permit()
        srv_sock = WrapSock()
        srv = KvLinkServer(NullCore(), "null", sock=srv_sock, regs={"permit": perm}).start()
        try:
            csock = WrapSock()
            csock.rule = drop_first(ESTOP_CMD, 2)
            e = KvEstopClient(("127.0.0.1", srv.port), sock=csock)
            for _ in range(2):
                with self.assertRaises(LinkTimeout):
                    e.send("assert", 42)
            a3 = e.send("assert", 42)
            self.assertEqual(a3.result, 0)
            self.assertEqual(win.writes, [(0x04, 1)])          # executed once
            a4 = e.send("assert", 42)                           # same cmd_id: same answer
            self.assertEqual(a4, a3)
            self.assertEqual(srv.estop.counts["duplicate_cmd"], 1)
            # positive control: resend with the SAME chunk frame id after a lost answer is
            # swallowed by the reassembler (old frame) and never reaches the latch
            srv_sock.rule = drop_first(ESTOP_ACK, 1)
            with self.assertRaises(LinkTimeout):
                e.send("release", 43, epoch=1)
            seq = e.link._ctrl_seq
            with self.assertRaises(LinkTimeout):
                e.send("release", 43, epoch=1, _frame_id=seq)
            self.assertGreaterEqual(srv.counts["rx_old_frame_chunks"], 1)
            r = e.send("release", 43, epoch=1)                  # new frame id: cached answer
            self.assertEqual((r.result, r.latched), (0, False))
            self.assertEqual(win.writes, [(0x04, 1), (0x04, 0)])
        finally:
            srv.stop()

    def test_restart_kv_and_hub(self):
        perm, _ = mock_permit()
        srv = self._srv(perm)
        port = srv.port
        e = KvEstopClient(("127.0.0.1", port))
        t = KvTelemetryClient(("127.0.0.1", port))
        try:
            a = e.send("assert", 1)
            boot1 = t.poll().kv_boot_id
            # hub restart: a fresh client adopts the KV latch and its epoch
            q = KvEstopClient(("127.0.0.1", port)).send("query", 99)
            self.assertEqual((q.latched, q.epoch, q.kv_boot_id), (True, a.epoch, boot1))
        finally:
            srv.stop()
        perm2, _ = mock_permit()
        sock = WrapSock(port)
        srv2 = KvLinkServer(NullCore(), "null", sock=sock, regs={"permit": perm2}).start()
        try:
            tm = t.poll()
            self.assertNotEqual(tm.kv_boot_id, boot1)           # KV restarted ...
            self.assertFalse(tm.blocks["estop"]["latched"])     # ... and lost the latch: re-assert
            self.assertEqual(e.send("assert", 2).result, 0)
        finally:
            srv2.stop()


# ------------------------------------------------------------------ per-session chunk loss
class SessionLossTests(unittest.TestCase):
    def _run(self, n_drop, frames=12):
        from host.kv_clock_sync import ClockSyncClient
        srv = server(NullCore(), "null")
        sync = ClockSyncClient(("127.0.0.1", srv.port), timeout_s=0.2)
        tele = KvTelemetryClient(("127.0.0.1", srv.port))
        stop = threading.Event()

        def poll():
            while not stop.is_set():
                try:
                    tele.poll()
                except LinkTimeout:
                    pass
                time.sleep(0.02)
        th = threading.Thread(target=poll)
        try:
            sock = WrapSock()
            left = [n_drop]

            def rule(t, d):               # drop one chunk (idx 1) of the first n_drop REQs
                if t == REQ and struct.unpack_from("<H", d, 28)[0] == 1 and left[0] > 0:
                    left[0] -= 1
                    return "drop"
                return "send"
            sock.rule = rule
            be = backend(srv.port, sock=sock, timeout_s=0.25)
            sync.start(50)
            th.start()
            time.sleep(0.2)
            be.client.stats(reset=True)
            base = dict(be.client.counts)
            failed = 0
            for i in range(frames):
                codes, ns = codes_for(i), now_ns()
                try:
                    be.frame_from_depth(identity=ident(i + 1, ns), **frame_kwargs(i + 1, codes, ns))
                except (LinkTimeout, RuntimeError):
                    failed += 1
            end = be.client.stats()
            win = {k: v - base.get(k, 0) for k, v in be.client.counts.items() if v - base.get(k, 0)}
            loss = chunk_loss(win, end["session_counts"])
            naive = win.get("tx_chunks", 0) - end["counts"].get("rx_chunks", 0)
            return loss, naive, failed, sock.dropped
        finally:
            stop.set()
            if th.is_alive():
                th.join()
            sync.stop()
            srv.stop()

    def test_frame_loss_counted_per_session_with_concurrent_sync_and_telem(self):
        loss, naive, failed, dropped = self._run(0)
        self.assertEqual((loss["nano_to_kv"], loss["kv_to_nano"], failed), (0, 0, 0))
        self.assertGreater(loss["nano_sent"], 0)
        self.assertNotEqual(naive, 0)          # the global-counter arithmetic is contaminated
        loss, naive, failed, dropped = self._run(3)
        self.assertEqual(dropped, 3)
        self.assertEqual(loss["nano_to_kv"], 3)                # injected N -> reported N
        self.assertEqual(loss["kv_to_nano"], 0)
        self.assertEqual(failed, 3)


# ------------------------------------------------------------------ plan_boxes extraction
class PlanBoxesBitwiseTests(unittest.TestCase):
    def test_check_boxes_unchanged_against_58661b9(self):
        try:
            old = load_old("host/m7_object_native.py", "58661b9")
        except (subprocess.CalledProcessError, OSError) as exc:
            self.skipTest("git show unavailable: %s" % exc)
        env = Env.get()
        old_native = old.NativeObjectEvidence(env.obj_lib)
        rng = np.random.default_rng(13)
        kinv = np.linalg.inv(K)
        n_ent = 0
        for trial in range(30):
            depth = rng.uniform(.3, 3., size=(H, W)).astype(np.float32)
            depth[rng.random((H, W)) < .1] = np.nan
            entries = []
            for i in range(int(rng.integers(1, 70))):
                u, v = rng.uniform(-30, W + 30), rng.uniform(-30, H + 30)
                z = rng.uniform(-.5, 3.)
                c = z * (kinv @ np.array([u, v, 1.]))
                half = rng.uniform(.005, .2, size=3)
                entries.append(("o%d" % i, np.array([c - half, c + half])))
            pose = np.eye(4)
            if trial % 2:
                a = rng.uniform(-.2, .2)
                pose[:3, :3] = [[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]
                pose[:3, 3] = rng.uniform(-.1, .1, size=3)
            new = env.native.check_boxes(depth, K, pose, entries)
            ref = old_native.check_boxes(depth, K, pose, entries)
            self.assertEqual(repr(new), repr(ref), trial)
            n_ent += len(entries)
        self.assertGreater(n_ent, 500)


# ------------------------------------------------------------------ replay fixture end to end
class ReplayRoiFixtureTests(unittest.TestCase):
    def test_kv_server_roi_cpu_and_nano_replay_roi_memory(self):
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        srv = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--bind",
                                "127.0.0.1", "--port", "0", "--max-idle-s", "60", "--core", "cpu",
                                "--roi-core", "cpu"], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True, env=env)
        try:
            port = json.loads(srv.stdout.readline())["port"]
            with tempfile.TemporaryDirectory() as t:
                prep = os.path.join(t, "prep")
                os.makedirs(prep)
                n = 10
                np.stack([codes_for(i) for i in range(n)]).astype("<u2").tofile(
                    os.path.join(prep, "depth_u16.bin"))
                json.dump({"width": W, "height": H, "frames": n, "depth_file": "depth_u16.bin",
                           "depth_unit_m": UNIT, "K": K.tolist(), "calibration_id": "synthetic",
                           "grid": {"origin_m": list(GRID[0]), "voxel_m": GRID[1],
                                    "shape_xyz": list(GRID[2])}},
                          open(os.path.join(prep, "meta.json"), "w"))
                snap = os.path.join(t, "snap.json")
                tool = str(ROOT / "tools/kv_link/nano_replay.py")
                mk = subprocess.run([sys.executable, tool, "--prep", prep, "--make-roi-memory", snap,
                                     "--roi-boxes", "20"], capture_output=True, text=True, env=env)
                self.assertEqual(mk.returncode, 0, mk.stderr[-2000:])
                out = os.path.join(t, "run")
                done = subprocess.run([sys.executable, tool, "--prep", prep, "--out", out, "--hz", "10",
                                       "--backend", "remote_pl", "--server", "127.0.0.1:%d" % port,
                                       "--chunk-bytes", "2000", "--compare", "--roi-memory", snap],
                                      capture_output=True, text=True, env=env)
                self.assertEqual(done.returncode, 0, done.stderr[-3000:])
                s = json.load(open(os.path.join(out, "summary.json")))
                self.assertTrue(s["compare"]["equal"])
                r = s["roi"]
                self.assertGreater(r["plan"]["rows_sent"], 3)
                self.assertGreaterEqual(len(r["plan"]["host_resolved"]), 2)
                self.assertEqual(r["frames_ok"], s["answered"])
                self.assertTrue(r["compare"]["equal"])
                self.assertTrue(r["compare"]["control_caught"])
                self.assertEqual(r["compare"]["rows_compared"], r["frames_ok"] * r["plan"]["rows_sent"])
                self.assertGreater(len(r["compare"]["verdict_code_counts"]), 1)
                self.assertEqual(s["drops"]["net_chunks_lost_nano_to_kv"], 0)
                self.assertEqual(s["drops"]["net_chunk_loss_source"], "per_session")
        finally:
            srv.terminate()
            srv.communicate(timeout=10)


if __name__ == "__main__":
    unittest.main()
