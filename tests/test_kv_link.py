"""Nano <-> KV260 frame channel (host/kv_link.py, B1 2026-10-11).

Every failure rule is exercised by injecting the fault and checking it is
detected and counted: lost / reordered / duplicated chunks, a late answer, a
stale or forged answer, a request flood, a dead server, a depth conversion that
differs on the far side, a wrong depth unit. The equality test runs the server
as a second process (mock PL behind the register map, interrupt wait) and
compares every voxel with the direct CPU reference. The interrupt wait is shown
to sleep (process CPU << wall) against the spinning poll as positive control.
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_kv_link.py
(also with /usr/bin/python3.8).
"""
import json
import os
from pathlib import Path
import resource
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

from host import kv_link
from host.kv_link import (CHUNK, CHUNK_MTU, NOTICE, REQ, RESP, SUPERSEDED, KvLinkClient,
                          KvLinkServer, LatestSlot, LinkTimeout, NullCore, Reassembler,
                          RemotePlFrameBackend, codes_to_m, decode_request, encode_response,
                          m_to_codes, remote_pl_factory, split)
from host.m7_backend import M7BackendSelector, PlBackendUnavailable
from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
from host.m7_pl_device import (FRAME, IRQ_REGS, DevMemPlDevice, MockPlDevice, PlDeviceUnavailable,
                               UioIrq, find_uio)
from host.m7_rolling import RollingFusion, compile_rolling
from tools.geo_same_input.common import depth_u16_to_m

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import geofx3_ref as fxr  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
GRID = GridSpec((-0.8, -0.8, -0.05), 0.05, (32, 32, 32))
H, W = 64, 96
K = np.array([[80.0, 0.0, 47.5], [0.0, 80.0, 31.5], [0.0, 0.0, 1.0]])
CLOCK, LOCAL, EPOCH = "test_mono", "cam", "e1"
UNIT = 0.001


def codes_for(i, h=H, w=W):
    yy, xx = np.mgrid[0:h, 0:w]
    c = (650 + 300 * np.sin(xx / 11. + i) + 150 * np.cos(yy / 7. - i)).astype(np.uint16)
    c[(xx * 7 + yy * 3 + i) % 13 == 0] = 0
    return c


class Env:
    """Libraries compiled once per test run."""
    _dir = None

    @classmethod
    def get(cls):
        if cls._dir is None:
            cls._tmp = tempfile.TemporaryDirectory()
            cls._dir = Path(cls._tmp.name)
            cls.frame_lib = compile_frame_native(cls._dir / "frame.so")
            cls._n = 0
        return cls

    @classmethod
    def rolling(cls):
        """RollingKernelHost wants a freshly compiled library per instance."""
        cls.get()._n += 1
        return compile_rolling(cls._dir / ("rolling%d.so" % cls._n))


def cpu_backend():
    return FrameNativeM7(Env.get().frame_lib)


def reference(codes, frame_id, ns, unit=UNIT):
    d = depth_u16_to_m(codes, unit)
    return cpu_backend().frame_from_depth(
        depth_m=d, k=K, camera_to_world=np.eye(4), grid_origin_m=GRID.origin_m,
        voxel_size_m=GRID.voxel_m, shape=GRID.shape, frame_id=frame_id, capture_time_ns=ns,
        pose_time_ns=ns, calibration_version="cal", pose_confirmed=True)


def frame(fid, codes, ns, unit=UNIT, stream="s1"):
    ident = CaptureIdentity(stream, fid, ns, CLOCK, "cal", LOCAL, EPOCH)
    pose = PoseAtCapture(ns, CLOCK, LOCAL, EPOCH, True, np.eye(4))
    return DepthFrame(ident, depth_u16_to_m(codes, unit), K, pose)


def ingest(pipe, fid, codes, unit=UNIT):
    ns = time.monotonic_ns()
    out = pipe.ingest_geometry(frame(fid, codes, ns, unit), now_ns=time.monotonic_ns(),
                               now_clock_id=CLOCK)
    return out.navigation, ns


def cpu_core():
    return cpu_backend().lib.fpga_m7_depth_frame


class DelayCore:
    """CPU core that first sleeps the next listed delay (a slow far side)."""

    def __init__(self, delays):
        self.delays, self.core = list(delays), cpu_core()

    def __call__(self, *args):
        if self.delays:
            time.sleep(self.delays.pop(0))
        self.core(*args)


class FaultySocket:
    """Client socket whose sendto can drop / reorder / duplicate chunks of chosen messages."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.rule = None          # callable(list of datagrams) -> list, applied per REQ message
        self._buf = []
        self.sent = 0

    def sendto(self, dgram, addr):
        mtype, idx, n = dgram[4], *struct.unpack_from("<HH", dgram, 28)
        if mtype != REQ or self.rule is None:
            self.sent += 1
            return self.sock.sendto(dgram, addr)
        self._buf.append(dgram)
        if idx == n - 1:
            out, self._buf = self.rule(self._buf), []
            for d in out:
                self.sent += 1
                self.sock.sendto(d, addr)
        return len(dgram)

    def __getattr__(self, name):
        return getattr(self.sock, name)


def start_server(core, name="cpu", **kw):
    return KvLinkServer(core, name, bind=("127.0.0.1", 0), **kw).start()


def remote(server_port, *, sock=None, timeout_s=1.0, chunk_bytes=2000, unit=UNIT):
    client = KvLinkClient(("127.0.0.1", server_port), chunk_bytes=chunk_bytes, timeout_s=timeout_s,
                          sock=sock)
    client.hello(2.0)
    return RemotePlFrameBackend(client, depth_unit_m=unit)


class ConversionTests(unittest.TestCase):
    def test_every_code_round_trips_bitwise_in_both_units(self):
        codes = np.arange(65536, dtype=np.uint16).reshape(256, 256)
        for unit in (0.001, 0.0002):
            d = depth_u16_to_m(codes, unit)
            self.assertEqual(codes_to_m(codes, np.float32(unit)).tobytes(), d.tobytes())
            back = m_to_codes(d, np.float32(unit))
            self.assertEqual(back.tobytes(), codes.tobytes())

    def test_values_off_the_unit_grid_are_refused_not_requantized(self):
        d = depth_u16_to_m(codes_for(0), UNIT)
        i = int(np.flatnonzero(np.isfinite(d.ravel()))[3])
        for bad in (np.nextafter(d.ravel()[i], np.float32(10)), np.float32(0.0),
                    np.float32(70.0), np.float32(-0.5)):
            x = d.copy()
            x.ravel()[i] = bad
            with self.assertRaises(ValueError):
                m_to_codes(x, np.float32(UNIT))
        # a 1/5000 m recording read with the millimetre unit is refused
        tum = depth_u16_to_m(codes_for(1), 0.0002)
        with self.assertRaises(ValueError):
            m_to_codes(tum, np.float32(0.001))


class FramingTests(unittest.TestCase):
    def test_reassembly_counts_every_kind_of_chunk_trouble(self):
        from collections import Counter
        payload = os.urandom(10000)
        c = Counter()
        rx = Reassembler(c)
        d1 = split(REQ, 7, 1, payload, 1000)
        for d in reversed(d1[1:]):                          # out of order
            self.assertIsNone(rx.feed(d))
        self.assertIsNone(rx.feed(d1[3]))                   # duplicate
        got = rx.feed(d1[0])
        self.assertEqual(got[4], payload)
        self.assertEqual(c["rx_duplicate_chunks"], 1)
        self.assertIsNone(rx.feed(d1[2]))                   # chunk of a completed frame
        self.assertEqual(c["rx_old_frame_chunks"], 1)
        d2, d3 = split(REQ, 7, 2, payload, 1000), split(REQ, 7, 3, payload, 1000)
        for d in d2[:-2]:                                   # frame 2 loses two chunks ...
            rx.feed(d)
        for d in d3:                                        # ... and is overtaken by frame 3
            got = rx.feed(d)
        self.assertEqual(got[3], 3)
        self.assertEqual((c["rx_incomplete_superseded"], c["rx_incomplete_missing_chunks"]), (1, 2))
        self.assertIsNone(rx.feed(d2[-1]))
        self.assertEqual(c["rx_old_frame_chunks"], 2)
        self.assertIsNone(rx.feed(b"junk" + bytes(40)))
        self.assertEqual(c["rx_malformed"], 1)
        self.assertEqual(c["rx_completed"], 2)

    def test_latest_slot_keeps_only_the_newest(self):
        slot = LatestSlot()
        for i in range(5):
            slot.put(i)
        self.assertEqual((slot.take(0.1), slot.superseded), (4, 4))
        self.assertIsNone(slot.take(0.05))
        got = []
        t = threading.Thread(target=lambda: got.append(slot.take(2.0)))
        t.start()
        time.sleep(0.05)
        slot.put("x")
        t.join()
        self.assertEqual(got, ["x"])


class IrqTests(unittest.TestCase):
    # GEO_FX3 (2026-11-04): the PL frame core reads codes only; these tests drive the codes call (kv_server's
    # PL path) and compare with the fixed-point core's bit-exact model (tests/geofx3_ref.py), not with the
    # float CPU reference.
    def selector(self, wait, delay, **mock):
        dev = MockPlDevice(frame_library=Env.get().frame_lib, irq=True, delay_s=delay,
                           frame_fx_library=fxr.fx_library(), **mock)
        return M7BackendSelector(pl_device_factory=lambda: dev, pl_wait=wait, pl_timeout_s=3.0,
                                 cpu_frame_library=Env.get().frame_lib,
                                 pl_frame_fx_library=fxr.fx_library()), dev

    @staticmethod
    def expect(codes):
        return fxr.expect(codes, K, np.eye(4), GRID.origin_m, GRID.voxel_m, GRID.shape)

    def run_once(self, backend, codes, fid=1):
        # The calling thread only: that is where poll spins or irq sleeps. (Process
        # CPU also holds OpenBLAS worker threads spinning after np.linalg.inv in
        # FrameNativeM7 -- 0.27 s per 0.4 s under numpy 1.24 here.)
        who = getattr(resource, "RUSAGE_THREAD", resource.RUSAGE_SELF)
        ru0, t0 = resource.getrusage(who), time.monotonic()
        ev = fxr.call_codes(backend, codes, K, np.eye(4), GRID.origin_m, GRID.voxel_m, GRID.shape)
        ru1, t1 = resource.getrusage(who), time.monotonic()
        cpu = (ru1.ru_utime - ru0.ru_utime) + (ru1.ru_stime - ru0.ru_stime)
        return ev, cpu, t1 - t0

    def test_irq_sleeps_poll_spins_both_exact(self):
        codes = codes_for(2)
        ref = self.expect(codes)
        out = {}
        for wait in ("irq", "poll"):
            sel, _ = self.selector(wait, 0.4)
            be = sel.frame_backend("pl")
            self.assertEqual(be.wait_mode, wait)
            ev, cpu, wall = self.run_once(be, codes)
            self.assertTrue(np.array_equal(ev.flags, ref.flags))
            self.assertEqual(ev.accepted_rays, ref.accepted_rays)
            out[wait] = (cpu, wall, be.counts.copy())
        cpu, wall, counts = out["irq"]
        self.assertGreater(wall, 0.38)
        self.assertLess(cpu, 0.25 * wall, out)
        self.assertEqual((counts["irq_wakeups"], counts["irq_spurious"]), (1, 0))
        cpu, wall, _ = out["poll"]
        self.assertGreater(cpu, 0.6 * wall, out)            # positive control: the spin is visible

    def test_stale_isr_would_wake_early_and_is_cleared(self):
        codes = codes_for(3)
        for mutant in (False, True):
            sel, dev = self.selector("irq", 0.2)
            be = sel.frame_backend("pl")
            dev.wr(FRAME, IRQ_REGS["IER"], 1)
            dev._regs.wr(FRAME, IRQ_REGS["ISR"], 1)          # a completion never acknowledged
            dev.wr(FRAME, IRQ_REGS["IER"], 0)
            if mutant:
                be.lib._clear_isr = lambda: None
            ev, _, _ = self.run_once(be, codes)
            self.assertTrue(np.array_equal(ev.flags, self.expect(codes).flags))
            if mutant:
                self.assertGreaterEqual(be.counts["irq_spurious"], 1)
            else:
                self.assertEqual(be.counts["irq_spurious"], 0)

    def test_irq_without_a_line_is_unavailable_never_polls(self):
        dev = MockPlDevice(frame_library=Env.get().frame_lib, irq=False)
        sel = M7BackendSelector(pl_device_factory=lambda: dev, pl_wait="irq")
        with self.assertRaises(PlBackendUnavailable):
            sel.frame_backend("pl")
        self.assertEqual(sel.counts["pl_unavailable:frame"], 1)
        with self.assertRaises(ValueError):
            M7BackendSelector(pl_wait="auto")

    def test_irq_timeout_fails_the_frame(self):
        dev = MockPlDevice(frame_library=Env.get().frame_lib, irq=True, hang=True,
                           frame_fx_library=fxr.fx_library())
        be = M7BackendSelector(pl_device_factory=lambda: dev, pl_wait="irq", pl_timeout_s=0.2,
                               cpu_frame_library=Env.get().frame_lib,
                               pl_frame_fx_library=fxr.fx_library()).frame_backend("pl")
        with self.assertRaises(RuntimeError):
            self.run_once(be, codes_for(1))
        self.assertEqual(be.counts["device_timeout"], 1)

    def test_uio_lookup_by_name(self):
        with tempfile.TemporaryDirectory() as t:
            sysfs, dev = Path(t) / "sys", Path(t) / "dev"
            for i, name in ((0, "m7_object"), (2, "m7_frame"), (3, "axi-pmon")):
                (sysfs / ("uio%d" % i)).mkdir(parents=True)
                (sysfs / ("uio%d" % i) / "name").write_text(name + "\n")
            dev.mkdir()
            self.assertEqual(find_uio("m7_frame", str(sysfs)), 2)
            self.assertIsNone(find_uio("m7_display", str(sysfs)))
            os.mkfifo(str(dev / "uio2"))
            line = UioIrq.open("m7_frame", str(sysfs), str(dev))
            self.assertEqual(line.path, str(dev / "uio2"))
            line.arm()
            self.assertEqual(line.wait(0.5), 1)           # the FIFO echoes the enable word
            line.close()
            with self.assertRaises(PlDeviceUnavailable):
                DevMemPlDevice().irq("frame", sysfs_root=str(sysfs), dev_root=str(dev / "none"))
            with self.assertRaises(PlDeviceUnavailable):
                UioIrq.open("m7_display", str(sysfs), str(dev))
            (sysfs / "uio5").mkdir()
            (sysfs / "uio5" / "name").write_text("m7_frame\n")
            with self.assertRaises(PlDeviceUnavailable):
                find_uio("m7_frame", str(sysfs))


class TwoProcessLoopbackTests(unittest.TestCase):
    """kv_server.py in its own process; the Nano side through M7Pipeline + RollingFusion."""

    def server(self, *args):
        proc = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"),
                                 "--bind", "127.0.0.1", "--port", "0", "--max-idle-s", "60"] + list(args),
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                env=dict(os.environ, PYTHONPATH=str(ROOT)))
        def stop():
            if proc.poll() is None:
                proc.terminate()
            proc.communicate(timeout=10)
        self.addCleanup(stop)
        line = proc.stdout.readline()
        if not line:
            self.fail("server did not start: %s" % proc.stderr.read())
        return proc, json.loads(line)["port"]

    def test_remote_pl_equals_fx_model_voxel_by_voxel(self):
        proc, port = self.server("--core", "pl", "--device", "mock", "--wait", "irq",
                                 "--chunk-bytes", str(CHUNK_MTU))
        for chunk in (60000, CHUNK_MTU):
            sel = M7BackendSelector(remote_pl_factory=remote_pl_factory(
                ("127.0.0.1", port), depth_unit_m=UNIT, chunk_bytes=chunk, timeout_s=3.0))
            be = sel.frame_backend("remote_pl")
            self.assertEqual(be.backend_name, "remote_pl:pl:mock:irq")
            pipe = M7Pipeline(be, GRID)
            fusion = RollingFusion(sel.rolling_backend("cpu", cpu_library=Env.rolling()),
                                   voxel_m=GRID.voxel_m, max_map_age_ns=10 ** 12)
            hits = 0
            for fid in range(1, 6):
                codes = codes_for(fid)
                nav, ns = ingest(pipe, fid, codes)
                # GEO_FX3: the remote PL is the fixed-point core -> its bit-exact model, not the float CPU
                ref = fxr.expect(codes, K, np.eye(4), GRID.origin_m, GRID.voxel_m, GRID.shape)
                self.assertTrue(nav.valid, nav.reason)
                self.assertEqual(nav.backend, "remote_pl:pl:mock:irq")
                self.assertTrue(np.array_equal(nav.payload["voxel_flags"], ref.flags))
                self.assertEqual(nav.payload["valid_grid_samples"], ref.accepted_rays)
                self.assertEqual(nav.payload["outside_grid"], ref.outside_grid)
                hits += int((ref.flags == 2).sum())
                fused = fusion.ingest(nav, pipe, now_ns=time.monotonic_ns(), clock_id=CLOCK,
                                      robot_position_m=(0., 0., 0.))
                self.assertTrue(fused.accepted, fused.reason)
                last = be.lib.last
                self.assertGreater(last["kv_ns"]["core_device"], 0)
                self.assertGreaterEqual(last["rtt_s"], last["kv_service_s"])
            self.assertGreater(hits, 0)
            self.assertEqual(be.client.counts["answered"], 5)
            stats = be.client.stats()
            self.assertEqual(stats["info"]["core_detail"]["wait"], "irq")
            self.assertGreaterEqual(stats["live"]["irq"]["arms"], 5)
            be.client.close()

    def test_null_core_never_looks_like_a_fresh_map(self):
        proc, port = self.server("--core", "null")
        sel = M7BackendSelector(remote_pl_factory=remote_pl_factory(("127.0.0.1", port),
                                                                     depth_unit_m=UNIT))
        be = sel.frame_backend("remote_pl")
        pipe = M7Pipeline(be, GRID)
        for fid in (1, 2, 3):
            nav, _ = ingest(pipe, fid, codes_for(fid))
            self.assertFalse(nav.valid)
            self.assertEqual(nav.reason, "no_valid_grid_samples")
        self.assertEqual(be.client.counts["answered"], 3)

    def test_wrong_unit_fails_the_frame(self):
        proc, port = self.server("--core", "cpu")
        be = remote(port, unit=0.0002)
        pipe = M7Pipeline(be, GRID)
        nav, _ = ingest(pipe, 1, codes_for(1), unit=UNIT)     # millimetre depth, 1/5000 m backend
        self.assertFalse(nav.valid)
        self.assertEqual(nav.reason, "backend_failure:ValueError")
        self.assertEqual(be.counts["depth_not_representable"], 1)
        self.assertEqual(be.client.counts["calls"], 0)        # nothing was sent


class FaultInjectionTests(unittest.TestCase):
    def setUp(self):
        Env.get()

    def test_lost_chunk_fails_that_frame_only_and_is_counted_as_network_loss(self):
        srv = start_server(cpu_core())
        self.addCleanup(srv.stop)
        sock = FaultySocket()
        be = remote(srv.port, sock=sock, timeout_s=0.4)
        pipe = M7Pipeline(be, GRID)
        sock.rule = lambda ds: ds[:3] + ds[4:]               # chunk 3 of frame 1 lost
        nav1, _ = ingest(pipe, 1, codes_for(1))
        self.assertGreater(len(split(REQ, 0, 0, bytes(2 * H * W + 300), 2000)), 4)
        self.assertEqual(nav1.reason, "backend_failure:LinkTimeout")
        self.assertIsNone(pipe._active_geometry_token)
        sock.rule = None
        nav2, ns2 = ingest(pipe, 2, codes_for(2))
        self.assertTrue(nav2.valid, nav2.reason)
        self.assertTrue(np.array_equal(nav2.payload["voxel_flags"], reference(codes_for(2), 2, ns2).flags))
        st = be.client.stats()
        self.assertEqual(st["total_counts"]["rx_incomplete_superseded"], 1)
        self.assertEqual(st["total_counts"]["rx_incomplete_missing_chunks"], 1)
        self.assertEqual(be.client.counts["tx_chunks"] - st["total_counts"]["rx_chunks"], 1)
        self.assertEqual(be.client.counts["timeout"], 1)
        self.assertEqual(be.counts["link_failure:LinkTimeout"], 1)

    def test_reordered_and_duplicated_chunks_still_exact(self):
        srv = start_server(cpu_core())
        self.addCleanup(srv.stop)
        sock = FaultySocket()
        be = remote(srv.port, sock=sock)
        pipe = M7Pipeline(be, GRID)
        sock.rule = lambda ds: ds[2:3] + list(reversed(ds))     # chunk 2 twice, all reversed
        nav, ns = ingest(pipe, 1, codes_for(4))
        self.assertTrue(nav.valid, nav.reason)
        self.assertTrue(np.array_equal(nav.payload["voxel_flags"], reference(codes_for(4), 1, ns).flags))
        self.assertEqual(srv.counts["rx_duplicate_chunks"], 1)

    def test_late_answer_times_out_then_is_counted_stale(self):
        srv = start_server(DelayCore([0.7, 0.0]))
        self.addCleanup(srv.stop)
        be = remote(srv.port, timeout_s=0.4)
        pipe = M7Pipeline(be, GRID)
        nav1, _ = ingest(pipe, 1, codes_for(1))
        self.assertEqual(nav1.reason, "backend_failure:LinkTimeout")
        nav2, _ = ingest(pipe, 2, codes_for(2))
        self.assertTrue(nav2.valid, nav2.reason)
        self.assertEqual(be.client.counts["stale_response"], 1)

    def test_forged_and_stale_answers_are_rejected(self):
        fake = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        fake.bind(("127.0.0.1", 0))
        self.addCleanup(fake.close)
        port = fake.getsockname()[1]
        from collections import Counter
        rx = Reassembler(Counter())
        info = json.dumps({"core": "fake"}).encode()

        def serve():
            fake.settimeout(5)
            while True:
                dgram, addr = fake.recvfrom(65536)
                got = rx.feed(dgram)
                if got is None:
                    continue
                mtype, _, session, fid, payload, _ = got
                if mtype == kv_link.HELLO:
                    for d in split(kv_link.HELLO_ACK, session, fid, info, 2000):
                        fake.sendto(d, addr)
                    continue
                req = decode_request(payload)
                depth = codes_to_m(req["codes"], kv_link.bits_f32(req["unit_bits"]))
                buf = np.zeros(kv_link.MAX_PIXELS, np.float32)
                buf[:depth.size] = depth.ravel()
                flags = np.full(kv_link.GRID_CAPACITY, 99, np.uint8)
                counts = np.zeros(3, np.uint32)
                status = np.zeros(1, np.uint32)
                cpu_core()(buf, req["inverse_k"], req["world_camera"], req["origin"], req["voxel"],
                           req["width"], req["height"], req["nx"], req["ny"], req["nz"],
                           flags, counts, status)
                crc = kv_link.crc32(buf[:depth.size])

                def answer(r, rid):
                    body = encode_response(r, status=status[0], counts=counts, depth_crc=crc,
                                           flags=flags[:32768], times_ns={}, kv_superseded=0,
                                           kv_incomplete=0, core_name="fake")
                    for d in split(RESP, session, rid, body, 2000):
                        fake.sendto(d, addr)
                stale = dict(req, frame_id=fid - 1)
                answer(stale, fid - 1)                                    # stale
                answer(dict(req, capture_ns=req["capture_ns"] + 1), fid)  # same id, wrong time
                answer(dict(req, identity=dict(req["identity"], stream_instance="other")), fid)
                answer(dict(req, frame_id=fid + 1), fid + 1)              # future id
                answer(req, fid)                                          # genuine
                return
        t = threading.Thread(target=serve, daemon=True)
        t.start()
        be = remote(port, timeout_s=2.0)
        pipe = M7Pipeline(be, GRID)
        nav, ns = ingest(pipe, 5, codes_for(5))
        t.join(5)
        self.assertTrue(nav.valid, nav.reason)
        self.assertTrue(np.array_equal(nav.payload["voxel_flags"], reference(codes_for(5), 5, ns).flags))
        self.assertEqual(be.client.counts["stale_response"], 1)
        self.assertEqual(be.client.counts["identity_mismatch"], 3)
        self.assertEqual(be.client.counts["answered"], 1)

    def test_request_flood_keeps_only_the_newest_and_notifies_the_rest(self):
        srv = start_server(DelayCore([0.3] * 8))
        self.addCleanup(srv.stop)
        raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        raw.bind(("127.0.0.1", 0))
        self.addCleanup(raw.close)
        session = 99
        bodies = {}
        for fid in range(1, 6):
            c = codes_for(fid)
            d = depth_u16_to_m(c, UNIT)
            ident = {"stream_instance": "s", "clock_id": CLOCK, "calibration_id": "cal",
                     "pose_epoch": EPOCH}
            bodies[fid] = kv_link.encode_request(
                frame_id=fid, capture_ns=1000 + fid, pose_ns=1000 + fid, identity=ident, width=W,
                height=H, nx=32, ny=32, nz=32, unit_f32=np.float32(UNIT), voxel=0.05,
                origin=GRID.origin_m, inverse_k=np.linalg.inv(K).astype(np.float32),
                world_camera=np.eye(4)[:3].astype(np.float32), codes=c, depth_crc=kv_link.crc32(d))
        for fid in range(1, 6):
            for dg in split(REQ, session, fid, bodies[fid], 60000):
                raw.sendto(dg, ("127.0.0.1", srv.port))
            if fid == 1:
                time.sleep(0.05)
        from collections import Counter
        rx = Reassembler(Counter())
        served, notices = [], []
        raw.settimeout(0.2)
        end = time.monotonic() + 3.0
        while time.monotonic() < end and len(served) + len(notices) < 5:
            try:
                dg, _ = raw.recvfrom(65536)
            except socket.timeout:
                continue
            got = rx.feed(dg)
            if got is None:
                continue
            if got[0] == RESP:
                served.append(got[3])
            elif got[0] == SUPERSEDED:
                notices.append(NOTICE.unpack_from(got[4])[0])
        self.assertEqual(sorted(served + notices), [1, 2, 3, 4, 5])
        self.assertIn(5, served)
        self.assertGreaterEqual(len(notices), 2)
        self.assertEqual(srv.counts["slot_superseded"], len(notices))

    def test_dead_server_times_out_and_the_map_is_not_refreshed(self):
        srv = start_server(cpu_core())
        be = remote(srv.port, timeout_s=0.3)
        sel = M7BackendSelector()
        pipe = M7Pipeline(be, GRID)
        fusion = RollingFusion(sel.rolling_backend("cpu", cpu_library=Env.rolling()),
                               voxel_m=GRID.voxel_m, max_map_age_ns=10 ** 12)
        nav, _ = ingest(pipe, 1, codes_for(1))
        r1 = fusion.ingest(nav, pipe, now_ns=time.monotonic_ns(), clock_id=CLOCK,
                           robot_position_m=(0., 0., 0.))
        self.assertTrue(r1.accepted)
        srv.stop()
        nav2, _ = ingest(pipe, 2, codes_for(2))
        self.assertEqual(nav2.reason, "backend_failure:LinkTimeout")
        r2 = fusion.ingest(nav2, pipe, now_ns=time.monotonic_ns(), clock_id=CLOCK,
                           robot_position_m=(0., 0., 0.))
        self.assertFalse(r2.accepted)
        self.assertEqual(fusion.counts["accepted"], 1)
        self.assertIsNone(pipe._active_geometry_token)

    def test_no_server_means_backend_unavailable_not_a_local_core(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        sel = M7BackendSelector(cpu_frame_library=Env.get().frame_lib,
                                remote_pl_factory=lambda: remote_pl_factory(
                                    ("127.0.0.1", port), depth_unit_m=UNIT, hello_timeout_s=0.3)())
        with self.assertRaises(PlBackendUnavailable):
            sel.frame_backend("remote_pl")
        self.assertEqual(sel.counts["pl_unavailable:frame"], 1)
        with self.assertRaises(PlBackendUnavailable):
            M7BackendSelector().frame_backend("remote_pl")
        with self.assertRaises(PlBackendUnavailable):
            M7BackendSelector().object_backend("remote_pl")
        with self.assertRaises(PlBackendUnavailable):
            M7BackendSelector().rolling_backend("remote_pl")

    def test_far_side_conversion_difference_is_caught_by_crc(self):
        srv = start_server(cpu_core())
        self.addCleanup(srv.stop)
        srv.convert = lambda c, u: codes_to_m(c, np.float32(float(u) * 1.0001))
        be = remote(srv.port)
        pipe = M7Pipeline(be, GRID)
        nav, _ = ingest(pipe, 1, codes_for(1))
        self.assertEqual(nav.reason, "backend_failure:RuntimeError")
        self.assertEqual(be.counts["depth_crc_mismatch"], 1)

    def test_identity_is_required_on_the_remote_path(self):
        srv = start_server(NullCore(), "null")
        self.addCleanup(srv.stop)
        be = remote(srv.port)
        with self.assertRaises(ValueError):
            be.frame_from_depth(depth_m=depth_u16_to_m(codes_for(1), UNIT), k=K,
                                camera_to_world=np.eye(4), grid_origin_m=GRID.origin_m,
                                voxel_size_m=0.05, shape=GRID.shape, frame_id=1,
                                capture_time_ns=1, pose_time_ns=1, calibration_version="cal",
                                pose_confirmed=True)


class NanoReplayFixtureTests(unittest.TestCase):
    """tools/kv_link/nano_replay.py against kv_server.py: paced capture, newest frame only,
    every answered frame equal to the fixed-point core's model (GEO_FX3; the comparator's controls kept)."""

    def prep(self, folder, n=24, unit=UNIT):
        frames = np.stack([codes_for(i) for i in range(n)])
        frames.astype("<u2").tofile(os.path.join(folder, "depth_u16.bin"))
        meta = {"width": W, "height": H, "frames": n, "depth_file": "depth_u16.bin",
                "depth_unit_m": unit, "K": K.tolist(), "calibration_id": "synthetic",
                "grid": {"origin_m": list(GRID.origin_m), "voxel_m": GRID.voxel_m,
                         "shape_xyz": list(GRID.shape)}}
        json.dump(meta, open(os.path.join(folder, "meta.json"), "w"))

    def test_paced_replay_drops_old_frames_and_matches_fx_model(self):
        srv = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--bind",
                                "127.0.0.1", "--port", "0", "--max-idle-s", "60", "--core", "pl",
                                "--device", "mock", "--wait", "irq", "--mock-delay-s", "0.12"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                               env=dict(os.environ, PYTHONPATH=str(ROOT)))
        try:
            port = json.loads(srv.stdout.readline())["port"]
            with tempfile.TemporaryDirectory() as t:
                prep = os.path.join(t, "prep")
                os.makedirs(prep)
                self.prep(prep)
                out = os.path.join(t, "run")
                done = subprocess.run([sys.executable, str(ROOT / "tools/kv_link/nano_replay.py"),
                                       "--prep", prep, "--out", out, "--hz", "30", "--backend",
                                       "remote_pl", "--server", "127.0.0.1:%d" % port,
                                       "--chunk-bytes", "2000", "--compare"],
                                      capture_output=True, text=True, env=dict(os.environ, PYTHONPATH=str(ROOT)))
                s = json.load(open(os.path.join(out, "summary.json")))
                # GEO_FX3 (2026-11-04): the PL is the fixed-point core; --compare uses the float CPU reference,
                # which it need not equal (exit 1 then). The geometry is checked against its bit-exact model.
                self.assertEqual(done.returncode, 0 if s["compare"]["equal"] else 1, done.stderr[-2000:])
                checked, equal = fxr.replay_dump_equals_fx(out, [codes_for(i) for i in range(24)], K,
                                                           GRID.origin_m, GRID.voxel_m, GRID.shape)
                self.assertEqual((checked, equal), (s["answered"], s["answered"]))
                self.assertEqual(s["capture"]["issued"], 24)
                self.assertGreater(s["capture"]["mailbox_superseded_on_nano"], 0)
                self.assertEqual(s["capture"]["consumed"] + s["capture"]["mailbox_superseded_on_nano"], 24)
                self.assertEqual(s["answered"], s["consumed_rows"])
                self.assertEqual(s["dumped_frames"], s["answered"])
                self.assertTrue(s["compare"]["controls_pass"])
                self.assertEqual(s["drops"]["net_chunks_lost_nano_to_kv"], 0)
                self.assertEqual(s["drops"]["net_timeouts"], 0)
                self.assertGreater(s["age_ms_answered"]["median"], 110)
                self.assertGreater(s["stages_ms_median"]["kv_core_device_ms"], 110)
                self.assertLess(s["channel_ms"]["median"], s["age_ms_answered"]["median"])
                # 120 ms > 200 ms budget? no: these frames are fresh enough to fuse
                self.assertGreater(s["fusion_reasons"].get("fused", 0), 0)
                win = s["server_window"]
                self.assertEqual(win["live"]["irq"]["arms"] >= s["answered"], True)
        finally:
            srv.terminate()
            srv.communicate(timeout=10)


if __name__ == "__main__":
    unittest.main()
