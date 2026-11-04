"""v3 L2 (2026-10-26): rolling map on the KV260 PL -- host-side contract tests.

* ROLL codec round trip; a flipped byte is refused (crc).
* Selector: rolling_backend("pl"/"remote_pl") only through factories (no fallback), the
  default stays "cpu"; kv_server without --rolling does not announce "rolling".
* AXI-shell mock (MockRollingWindow + PlRollingCore) replays a recorded synthetic trace
  word for word; positive controls: core mutants SHIFT_SIGN / NO_EVICT and the shell mutant
  drop_last_delta are caught.
* Two processes: kv_server --core pl --device mock --rolling pl; the Nano pipeline's frames go
  through the KV frame core and RollingFusion uses RemotePlRollingBackend. Every delta, the
  final snapshot and a query equal the CPU arm; integrates are zero copy; a block the KV does
  not hold (one cell changed) is answered not_held and resent inline (positive control), and
  a resent command id is not executed twice.
* The generated register header (hw/m7/pl/sw/m7pl_hls_regs.h, section rolling) equals
  host/kv_rolling.ROLLING_REGS entry by entry.
"""
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

from host.kv_rolling import (MODE_INLINE, MODE_REPLAY, OP_CONFIG, RESULT_NOT_EXECUTED, RESULT_OK, ROLLING_REGS, CpuRollingCore,
                             MockRollingWindow, PlRollingCore, RemotePlRollingBackend,
                             RollingService, crc32, decode_roll_ack, decode_roll_req,
                             encode_roll_ack, encode_roll_req, remote_rolling_factory)
from host.m7_backend import M7BackendSelector, PlBackendUnavailable
from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
from host.m7_pl_device import MockPlDevice
from host.m7_rolling import (OP_INTEGRATE, OP_RESET, RollingDeltaMirror, RollingFusion, RollingKernelHost,
                             compile_rolling)
from tools.geo_same_input.common import depth_u16_to_m

ROOT = Path(__file__).resolve().parents[1]
GRID = GridSpec((-0.8, -0.8, -0.05), 0.05, (32, 32, 32))
H, W = 64, 96
K = np.array([[80.0, 0.0, 47.5], [0.0, 80.0, 31.5], [0.0, 0.0, 1.0]])
CLOCK, UNIT = "test_mono", 0.001


def codes_for(i):
    yy, xx = np.mgrid[0:H, 0:W]
    c = (650 + 300 * np.sin(xx / 11. + i) + 150 * np.cos(yy / 7. - i)).astype(np.uint16)
    c[(xx * 7 + yy * 3 + i) % 13 == 0] = 0
    return c


class Env:
    _dir = None
    _n = 0

    @classmethod
    def lib(cls, mutant=None):
        if cls._dir is None:
            cls._tmp = tempfile.TemporaryDirectory()
            cls._dir = Path(cls._tmp.name)
        cls._n += 1
        return compile_rolling(cls._dir / ("r%d.so" % cls._n), mutant=mutant)

    @classmethod
    def frame_lib(cls):
        if getattr(cls, "_frame_lib", None) is None:
            cls.lib()
            cls._frame_lib = compile_frame_native(cls._dir / "frame.so")
        return cls._frame_lib

    @classmethod
    def trace(cls):
        if getattr(cls, "_trace", None) is None:
            cls.lib()
            out = cls._dir / "synthetic.bin"
            subprocess.run([sys.executable, str(ROOT / "tools/m7_rolling_pl/make_trace.py"), "--out", str(out),
                            "--synthetic"], check=True, capture_output=True, text=True,
                           env=dict(os.environ, PYTHONPATH=str(ROOT)))
            cls._trace = out
        return cls._trace


def frame(fid, ns, stream="s1"):
    ident = CaptureIdentity(stream, fid, ns, CLOCK, "cal", "cam", "e1")
    return DepthFrame(ident, depth_u16_to_m(codes_for(fid), UNIT), K,
                      PoseAtCapture(ns, CLOCK, "cam", "e1", True, np.eye(4)))


def replay(trace, core_factory):
    from tools.m7_rolling_pl.replay_trace import compare, read_trace
    core = core_factory()
    bad = 0
    for r in read_trace(trace):
        kw = {}
        if r["obs"] is not None:
            kw["obs"] = r["obs"]
        if r["op"] == 2:
            kw["query"], kw["qc"] = r["query"], r["qc"]
        hdr, delta, qout, dump, _ = core.call(r["op"], r["epoch"], r["frame"], r["wb"], r["ob"], r["dims"], **kw)
        bad += compare(r, hdr, delta, qout, dump) > 0
    return bad


def mock_core(mutant=None, shell_mutant=None):
    dev = MockPlDevice()
    win = MockRollingWindow(dev._memory, Env.lib(mutant), mutant=shell_mutant)
    return PlRollingCore(win, dev, CpuRollingCore(Env.lib()).config_words())


class CodecTests(unittest.TestCase):
    def test_roundtrip_and_crc(self):
        body = encode_roll_req(op=OP_INTEGRATE, cmd_id=7, mode=MODE_INLINE, epoch=3, frame_seq=9,
                               window_base=(-40, 2, -8), obs_base=(1, 2, 3), dims=(2, 2, 1),
                               obs_crc=crc32(b"\x00\x01\x02\x01"), inline=b"\x00\x01\x02\x01")
        req = decode_roll_req(body)
        self.assertEqual((req["op"], req["cmd_id"], req["epoch"], req["window_base"], req["dims"]),
                         (OP_INTEGRATE, 7, 3, (-40, 2, -8), (2, 2, 1)))
        self.assertEqual(req["inline"], b"\x00\x01\x02\x01")
        bad = bytearray(body)
        bad[20] ^= 1
        with self.assertRaises(ValueError):
            decode_roll_req(bytes(bad))
        ack = decode_roll_ack(encode_roll_ack(result=RESULT_OK, cmd_id=7, header=np.arange(24),
                                              delta=np.array([5, 6], np.uint64)))
        self.assertEqual(list(ack["header"]), list(range(24)))
        self.assertEqual(list(ack["delta"]), [5, 6])


class SelectorTests(unittest.TestCase):
    def test_no_fallback_and_default_cpu(self):
        sel = M7BackendSelector()
        for name in ("pl", "remote_pl"):
            with self.assertRaises(PlBackendUnavailable):
                sel.rolling_backend(name)
        self.assertEqual(sel.counts["pl_unavailable:rolling"], 2)
        self.assertIsInstance(sel.rolling_backend("cpu", cpu_library=Env.lib()), RollingKernelHost)

    def test_factories(self):
        from host.kv_rolling import CoreRollingBackend
        sel = M7BackendSelector(pl_rolling_factory=lambda: CoreRollingBackend(mock_core()))
        be = sel.rolling_backend("pl")
        self.assertTrue(be.reset(1, (0, 0, 0)).ok)


class ShellMockReplayTests(unittest.TestCase):
    def test_mock_shell_bit_exact_and_controls_caught(self):
        trace = Env.trace()
        self.assertEqual(replay(trace, mock_core), 0)
        self.assertEqual(replay(trace, lambda: CpuRollingCore(Env.lib())), 0)
        self.assertGreater(replay(trace, lambda: mock_core("SHIFT_SIGN")), 0)
        self.assertGreater(replay(trace, lambda: mock_core("NO_EVICT")), 0)
        self.assertGreater(replay(trace, lambda: mock_core(shell_mutant="drop_last_delta")), 0)

    def test_service_idempotent_and_not_held(self):
        svc = RollingService(mock_core())
        body = encode_roll_req(op=OP_RESET, cmd_id=1, epoch=1, window_base=(0, 0, 0))
        self.assertEqual(decode_roll_ack(svc.handle(body))["result"], RESULT_OK)
        blk = np.full((1, 1, 8), 2, np.uint8)
        svc.hold(5, 0, blk.ravel(), 8)
        req = dict(op=OP_INTEGRATE, epoch=1, frame_seq=5, window_base=(0, 0, 0), obs_base=(0, 0, 0),
                   dims=(8, 1, 1))
        ok = decode_roll_ack(svc.handle(encode_roll_req(cmd_id=2, obs_crc=crc32(blk.tobytes()), **req)))
        self.assertEqual((ok["result"], int(ok["header"][0])), (RESULT_OK, 0))
        again = decode_roll_ack(svc.handle(encode_roll_req(cmd_id=2, obs_crc=crc32(blk.tobytes()), **req)))
        self.assertEqual(svc.counts["duplicate_cmd"], 1)
        self.assertEqual(int(again["header"][0]), 0)           # not STALE_FRAME: not run twice
        other = blk.copy()
        other[0, 0, 3] = 1
        nh = decode_roll_ack(svc.handle(encode_roll_req(cmd_id=3, obs_crc=crc32(other.tobytes()),
                                                        **dict(req, frame_seq=6))))
        self.assertEqual(nh["result"], 1)                        # not_held
        cfg = decode_roll_ack(svc.handle(encode_roll_req(op=OP_CONFIG, cmd_id=4)))
        self.assertEqual(list(cfg["header"][:16]), list(svc.config_words))


class TwoProcessRollingTests(unittest.TestCase):
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

    def test_default_server_has_no_rolling(self):
        _, port = self.server("--core", "cpu")
        sel = M7BackendSelector(remote_rolling_factory=remote_rolling_factory(("127.0.0.1", port)))
        with self.assertRaises(PlBackendUnavailable):
            sel.rolling_backend("remote_pl")

    def test_remote_pl_rolling_equals_local_zero_copy(self):
        from host.kv_link import CHUNK_MTU, remote_pl_factory
        _, port = self.server("--core", "pl", "--device", "mock", "--wait", "poll", "--rolling", "pl",
                              "--chunk-bytes", str(CHUNK_MTU))
        frame_lib = Env.frame_lib()
        sel = M7BackendSelector(
            remote_pl_factory=remote_pl_factory(("127.0.0.1", port), depth_unit_m=UNIT, chunk_bytes=CHUNK_MTU,
                                                timeout_s=3.0),
            remote_rolling_factory=remote_rolling_factory(("127.0.0.1", port), timeout_s=2.0))
        pipe_b = M7Pipeline(sel.frame_backend("remote_pl"), GRID)
        roll_b = sel.rolling_backend("remote_pl")
        self.assertIsInstance(roll_b, RemotePlRollingBackend)
        self.assertEqual(roll_b.kv_core, "pl")              # 10-29 console FPGA panel: asked once (STATS)
        fus_b = RollingFusion(roll_b, voxel_m=0.05, max_map_age_ns=10 ** 12)
        # GEO_FX3 (2026-11-04): the remote frame core is the fixed-point core; the local arm runs its
        # bit-exact model (tests/geofx3_ref.py) instead of the float CPU reference.
        import geofx3_ref
        pipe_a = M7Pipeline(geofx3_ref.FxFrameBackend(UNIT), GRID)
        roll_a = RollingKernelHost(Env.lib())
        fus_a = RollingFusion(roll_a, voxel_m=0.05, max_map_age_ns=10 ** 12)
        changes = 0
        for fid in range(1, 9):
            ns = time.monotonic_ns()
            pos = (0.07 * fid, -0.03 * fid, 0.0)           # the window rolls every frame
            outs = []
            for pipe, fus in ((pipe_a, fus_a), (pipe_b, fus_b)):
                nav = pipe.ingest_geometry(frame(fid, ns), now_ns=ns + 1000, now_clock_id=CLOCK).navigation
                self.assertTrue(nav.valid, nav.reason)
                outs.append(fus.ingest(nav, pipe, now_ns=ns + 2000, clock_id=CLOCK, robot_position_m=pos))
            a, b = outs
            self.assertTrue(a.accepted and b.accepted, (a.reason, b.reason))
            self.assertEqual((a.delta.epoch, a.delta.delta_seq, a.delta.window_base, a.delta.changes),
                             (b.delta.epoch, b.delta.delta_seq, b.delta.window_base, b.delta.changes))
            self.assertTrue(np.array_equal(a.kernel.header, b.kernel.header))
            changes += len(a.delta.changes)
            self.assertGreater(roll_b.last_times["device"], 0)   # this INTEGRATE's PL time (console panel)
        self.assertGreater(changes, 0)
        ra, la = roll_a.snapshot()
        rb, lb = roll_b.snapshot()
        self.assertTrue(np.array_equal(la, lb))
        q = np.array([[x, y, z] for x in range(-50, 60, 7) for y in range(-50, 40, 9) for z in (-8, 0, 3)])
        self.assertTrue(all(np.array_equal(u, v) for u, v in zip(roll_a.query(q)[1], roll_b.query(q)[1])))
        self.assertEqual(roll_b.counts["roll:inline_resend"], 0)
        # positive control: a block the KV does not hold -> not_held -> inline, same result as cpu
        nav = pipe_b.ingest_geometry(frame(20, time.monotonic_ns()), now_ns=time.monotonic_ns(),
                                     now_clock_id=CLOCK).navigation
        flags = np.array(nav.payload["voxel_flags"], copy=True)
        flags.ravel()[100] = (flags.ravel()[100] + 1) % 3
        wb, ob = fus_a.window_base_for((0.6, -0.2, 0.0)), fus_a.lattice_base(nav.payload["grid"])
        ra = roll_a.integrate(fus_a.epoch, 20, wb, ob, flags)
        rb = roll_b.integrate(fus_b.epoch, 20, wb, ob, flags)
        self.assertEqual(roll_b.counts["roll:inline_resend"], 1)
        self.assertEqual(roll_b.counts["roll:not_held"], 1)
        self.assertEqual((ra.status, ra.changes), (rb.status, rb.changes))
        from host.kv_link import KvLinkClient
        cl = KvLinkClient(("127.0.0.1", port))
        st = cl.stats()
        cl.close()
        self.assertEqual(st["rolling"]["counts"]["integrate_zero_copy"], 8)
        self.assertEqual(st["rolling"]["counts"]["integrate_inline"], 1)
        roll_b.close()


class FaultLink:
    """In-process stand-in for KvLinkClient: ``_control_resend`` hands the body to a
    RollingService; ``rules`` [(predicate(req), action, count)] drop the answer before the
    service runs the command ("before") or after ("after")."""

    def __init__(self, svc, replay=True):
        from collections import Counter
        self.svc, self.counts, self.rules, self.waits = svc, Counter(), [], []
        self.feats = {"rolling": 1, "rolling_replay": 1 if replay else 0}

    def require(self, feature, hello_timeout_s=3.0):
        return {"features": dict(self.feats)}

    def _control_resend(self, msg_type, ack_type, body, timeout_s, first_gap_s, flags=0):
        self.waits.append(timeout_s)
        req = decode_roll_req(body)
        for i, (pred, action, n) in enumerate(self.rules):
            if n > 0 and pred(req):
                self.rules[i] = (pred, action, n - 1)
                if action == "before":
                    return None
                self.svc.handle(body)
                return None
        return self.svc.handle(body)

    def close(self):
        pass


def is_integrate(req):
    return req["op"] == OP_INTEGRATE and req["mode"] == MODE_INLINE


class LinkRecoveryTests(unittest.TestCase):
    """Go-live 2026-10-26 fault injection: remote rolling backend survives lost answers, a
    KV that lost its map (bitstream reloaded) and a DUMP timeout; the CPU path keeps raising."""

    AGE = 200_000_000

    @classmethod
    def setUpClass(cls):
        cls.frame_lib = Env.frame_lib()

    def arms(self, replay=True):
        link = FaultLink(RollingService(CpuRollingCore(Env.lib())), replay=replay)
        remote = RemotePlRollingBackend(None, timeout_s=0.3, link=link)
        self.assertTrue(remote.link_recovery)
        link.waits.clear()                                # CONFIG at connect: no frame budget
        fus_b = RollingFusion(remote, voxel_m=0.05, max_map_age_ns=self.AGE)
        fus_a = RollingFusion(RollingKernelHost(Env.lib()), voxel_m=0.05, max_map_age_ns=self.AGE)
        self.assertFalse(fus_a._recovery)
        return link, fus_a, fus_b

    def step(self, fus, fid, ns=None, lag_ns=2000):
        pipes = self.__dict__.setdefault("_pipes", {})
        pipe = pipes.get(id(fus))
        if pipe is None:                                  # one pipeline per fusion arm
            pipe = pipes[id(fus)] = M7Pipeline(FrameNativeM7(self.frame_lib), GRID)
        ns = time.monotonic_ns() if ns is None else ns
        nav = pipe.ingest_geometry(frame(fid, ns), now_ns=ns + 1000, now_clock_id=CLOCK).navigation
        self.assertTrue(nav.valid, nav.reason)
        return fus.ingest(nav, pipe, now_ns=ns + lag_ns, clock_id=CLOCK,
                          robot_position_m=(0.07 * fid, -0.03 * fid, 0.0))

    def snap(self, fus):
        return fus.snapshot(now_ns=time.monotonic_ns(), clock_id=CLOCK)

    def feed_mirror(self, mirror, fus, res):
        if res.delta is not None:
            mirror.apply(res.delta, now_ns=res.delta.acquired_ns + 1)
        if mirror.needs_resync:
            mirror.resync(self.snap(fus))

    def test_lost_ack_executed_late_ack_adopted_mirror_resyncs(self):
        link, fus_a, fus_b = self.arms()
        mirror = RollingDeltaMirror(fus_b.backend.config.dims)
        for fid in range(1, 7):
            if fid == 4:
                link.rules.append((is_integrate, "after", 1))      # KV ran it, answer lost
            a, b = self.step(fus_a, fid), self.step(fus_b, fid)
            self.assertTrue(a.accepted)
            if fid == 4:
                self.assertEqual((b.accepted, b.reason, b.delta), (False, "expired_roll_timeout", None))
                self.assertIsNotNone(fus_b.backend.unanswered)
                # a resync request now must not displace the KV's cached answer
                self.assertEqual(self.snap(fus_b).reason, "roll_pending")
            else:
                self.assertTrue(b.accepted, b.reason)
            self.feed_mirror(mirror, fus_b, b)
        self.assertEqual(fus_b.counts["roll_recovered:late_ack"], 1)
        self.assertEqual(link.svc.counts["replay_hit"], 1)
        self.assertEqual(fus_b._delta_seq, 6)
        self.assertEqual(mirror.counts["resync"], 1)
        sa, sb = self.snap(fus_a), self.snap(fus_b)
        self.assertTrue(sb.valid, sb.reason)
        self.assertTrue(np.array_equal(sa.logodds, sb.logodds))       # every frame is in the map
        self.assertTrue(np.array_equal(mirror.states, sb.states))
        self.assertTrue(all(w <= self.AGE / 1e9 for w in link.waits), max(link.waits))

    def test_lost_ack_not_executed_then_continues(self):
        link, fus_a, fus_b = self.arms()
        mirror = RollingDeltaMirror(fus_b.backend.config.dims)
        for fid in range(1, 7):
            if fid == 4:
                link.rules.append((is_integrate, "before", 1))     # never reached the KV
                self.assertEqual(self.step(fus_b, fid).reason, "expired_roll_timeout")
                continue
            a, b = self.step(fus_a, fid), self.step(fus_b, fid)
            self.assertTrue(a.accepted and b.accepted, b.reason)
            self.feed_mirror(mirror, fus_b, b)
        self.assertEqual(fus_b.counts["roll_recovered:not_executed"], 1)
        self.assertEqual(link.svc.counts["replay_not_executed"], 1)
        self.assertEqual((fus_b.epoch, fus_b._delta_seq), (1, 5))
        self.assertEqual(mirror.counts["resync"], 0)                  # no gap: nothing was lost
        sa, sb = self.snap(fus_a), self.snap(fus_b)
        self.assertTrue(np.array_equal(sa.logodds, sb.logodds))       # frame 4 in neither map
        self.assertTrue(np.array_equal(mirror.states, sb.states))

    def test_unreachable_kv_keeps_pending_and_budget_caps_waits(self):
        link, _, fus_b = self.arms()
        self.step(fus_b, 1)
        link.rules.append((lambda r: True, "before", 3))           # INTEGRATE, then 2 probes lost
        self.assertEqual(self.step(fus_b, 2).reason, "expired_roll_timeout")
        self.assertEqual(self.step(fus_b, 3).reason, "expired_roll_unreachable")
        self.assertEqual(self.step(fus_b, 4).reason, "expired_roll_unreachable")
        self.assertTrue(self.step(fus_b, 5).accepted)              # probe -> not_executed -> fused
        self.assertEqual((fus_b.epoch, fus_b._delta_seq), (1, 2))
        n = len(link.waits)
        ns = time.monotonic_ns()
        res = self.step(fus_b, 6, ns=ns, lag_ns=self.AGE - 1_000_000)   # 1 ms of budget left
        self.assertEqual(res.reason, "expired_roll_budget")
        self.assertEqual(len(link.waits), n)                         # nothing sent
        self.assertTrue(all(w <= self.AGE / 1e9 for w in link.waits))

    def test_kv_reloaded_bitstream_rebuilds_both_sides(self):
        link, _, fus_b = self.arms()
        mirror = RollingDeltaMirror(fus_b.backend.config.dims)
        for fid in (1, 2, 3):
            self.feed_mirror(mirror, fus_b, self.step(fus_b, fid))
        link.svc = RollingService(CpuRollingCore(Env.lib()))        # fresh core: uninitialized
        res = self.step(fus_b, 4)
        self.assertTrue(res.accepted, res.reason)                    # rebuilt within the frame
        self.assertEqual((res.delta.epoch, res.delta.delta_seq), (2, 1))
        self.assertEqual(fus_b.counts["map_lost:kernel_uninitialized"], 1)
        self.assertEqual(fus_b.counts["map_rebuilt:kernel_uninitialized"], 1)
        self.feed_mirror(mirror, fus_b, res)
        self.assertEqual(mirror.counts["epoch_start"], 2)
        self.feed_mirror(mirror, fus_b, self.step(fus_b, 5))
        sb = self.snap(fus_b)
        self.assertEqual((sb.epoch, sb.delta_seq), (2, 2))
        self.assertTrue(np.array_equal(mirror.states, sb.states))
        # same reference: a fresh CPU fusion fed frames 4 and 5 only
        ref = RollingFusion(RollingKernelHost(Env.lib()), voxel_m=0.05, max_map_age_ns=self.AGE)
        self.step(ref, 4)
        self.step(ref, 5)
        self.assertTrue(np.array_equal(self.snap(ref).logodds, sb.logodds))

    def test_kv_restart_lost_cache_diverged_rebuilds(self):
        link, _, fus_b = self.arms()
        for fid in (1, 2):
            self.step(fus_b, fid)
        link.rules.append((is_integrate, "after", 1))
        self.step(fus_b, 3)
        core = link.svc.core
        link.svc = RollingService(core)       # kv_server restarted, map kept in URAM, cache lost
        res = self.step(fus_b, 4)             # probe: not_executed (wrong); delta_seq jumps -> rebuild
        self.assertTrue(res.accepted, res.reason)
        self.assertEqual((res.delta.epoch, res.delta.delta_seq), (2, 1))
        self.assertEqual(fus_b.counts["map_lost:seq_diverged"], 1)

    def test_dump_timeout_is_an_invalid_snapshot(self):
        link, _, fus_b = self.arms()
        self.step(fus_b, 1)
        link.rules.append((lambda r: r["op"] == 3, "before", 1))
        sb = self.snap(fus_b)
        self.assertEqual((sb.valid, sb.reason), (False, "roll_error:LinkTimeout"))
        self.assertTrue(self.snap(fus_b).valid)
        self.assertTrue(self.step(fus_b, 2).accepted)

    def test_no_replay_feature_rebuilds_instead_of_probing(self):
        link, _, fus_b = self.arms(replay=False)
        self.step(fus_b, 1)
        link.rules.append((is_integrate, "after", 1))
        self.step(fus_b, 2)
        res = self.step(fus_b, 3)
        self.assertTrue(res.accepted, res.reason)
        self.assertEqual(res.delta.epoch, 2)
        self.assertEqual(link.svc.counts["replay_not_executed"] + link.svc.counts["replay_hit"], 0)

    def test_service_replay_never_executes(self):
        svc = RollingService(CpuRollingCore(Env.lib()))
        ok = decode_roll_ack(svc.handle(encode_roll_req(op=OP_RESET, cmd_id=7, epoch=1)))
        probe = decode_roll_ack(svc.handle(encode_roll_req(op=OP_INTEGRATE, cmd_id=7, mode=MODE_REPLAY)))
        self.assertEqual((probe["result"], list(probe["header"])), (ok["result"], list(ok["header"])))
        miss = decode_roll_ack(svc.handle(encode_roll_req(op=OP_INTEGRATE, cmd_id=8, mode=MODE_REPLAY)))
        self.assertEqual(miss["result"], RESULT_NOT_EXECUTED)
        self.assertEqual(svc._last_cmd, 7)                            # cache untouched
        self.assertEqual(svc.counts["op:%d" % OP_INTEGRATE], 0)


class RegisterHeaderTests(unittest.TestCase):
    def test_generated_header_matches(self):
        path = ROOT / "hw/m7/pl/sw/m7pl_hls_regs.h"
        text = path.read_text()
        got = {}
        for m in re.finditer(r"#define XFPGA_M7_ROLLING_AXI_CONTROL_ADDR_(\w+)\s+0x([0-9a-fA-F]+)", text):
            got[m.group(1)] = int(m.group(2), 16)
        if not got:
            self.fail("no rolling section in %s" % path)
        for name, off in ROLLING_REGS.items():
            key = {"GIE": "GIE", "IER": "IER", "ISR": "ISR"}.get(name, name)
            if key in got:
                self.assertEqual(got[key], off, name)
            else:
                self.assertIn(name, ("AP_CTRL", "GIE", "IER", "ISR"), "missing %s" % name)


if __name__ == "__main__":
    unittest.main()
