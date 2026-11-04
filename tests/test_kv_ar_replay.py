"""Replay mark on the car AR panel (owner 10-12: replay allowed, always marked; 2026-10-12).

Before: the producer chain replays recorded depth while kv_server runs --ar-source live, so the
panel said LIVE; and with --ar-source replay the core skips both age checks (replay=1), so a
replayed frame never went NOAGE / STALE and showed AGE N/A. Now:
  * the Nano marks a replayed frame (REQ ext_flags bit3, no bytes, feature replay_ext announced
    only with --ar drm): producer --data-source replay, nano_replay always;
  * fill_ar_inputs keeps the age for replay frames (age_valid from the AGE block alone) and sets
    replay = --ar-source replay OR the frame's mark;
  * the helper writes replay=0 to the core when the frame has no age or is over max_age_ms, so
    the core's own rules give NOAGE / STALE exactly as for live; fresh replay -> REPLAY + age;
    the fail-safe age grows from the real Nano basis; events carry replay (data), replay_core
    and the mode (PS-side mirror of the core rule);
  * TELEM ar block: mode / age_ms / age_valid / replay / replay_core / frame_id (the hub reads
    them; before they were all missing).

Checked here (known answers, golden + C reference on the helper's dumps, positive controls):
  W  wire: bit3 round trip; a server without replay_ext voids such a request (BadExtension,
     negative control); without the mark the bytes equal d6735da; the client sets the bit only
     for a server that announces it; HELLO announces replay_ext only with an AR presenter;
  H  helper (host build, mock provider) fed fill_ar_inputs messages: fresh replay -> REPLAY +
     AGE:xxxMS; replay over age -> STALE; replay without age -> NOAGE; live -> LIVE;
     --ar-source replay -> REPLAY; fail-safe after a replay frame: STALE, age from the Nano
     basis; every dump == golden == C reference; event mode == golden mode per dump;
     positive controls: d6735da's helper and d6735da's fill_ar_inputs (git show, built and run
     here) on the same frames do NOT give these pictures (REPLAY / AGE N/A, no STALE);
  C  chain: replay stand-in marker -> SourceMarker -> GeoProducer(KvConnection data_source=
     replay) -> request bit -> KvLinkServer snapshot -> fill_ar_inputs -> helper registers /
     events -> ArPresenter.telem() ar block (REPLAY, then STALE on the fail-safe); the same
     with data_source live gives LIVE; mismatch marker counted; --data-source is required.

Run: PYTHONHASHSEED=0 python3 -m unittest discover -s tests -p 'test_kv_ar_replay.py'
(also /usr/bin/python3.8). Builds both helpers with g++ + the Vitis HLS headers.
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tests"), str(ROOT / "tools" / "ar_display"), str(ROOT / "tools" / "kv_link")):
    if p not in sys.path:
        sys.path.insert(0, p)

from host import kv_ar, kv_link  # noqa: E402
from host.kv_link import (BadExtension, CpuRoiCore, KvLinkClient, KvLinkServer, LinkTimeout,  # noqa: E402
                          RemotePlFrameBackend, decode_request, encode_request)
from host.m7_frame_native import FrameNativeM7  # noqa: E402
from host.fpga_clock import now_ns  # noqa: E402
import ar_dump_check  # noqa: E402
import test_kv_ar as base  # noqa: E402

HLS_INC = os.environ.get("HLS_INC", "/data1/Xilinx/Vitis_HLS/2022.2/include")
CXXFLAGS = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unknown-pragmas", "-ffp-contract=off"]
MAX_AGE = 200


def mono_ns():
    return time.monotonic_ns() if hasattr(time, "monotonic_ns") else int(time.monotonic() * 1e9)


class Bins:
    _old = None

    @classmethod
    def new(cls):
        return base.Env.helper()

    @classmethod
    def old(cls):
        """m7ar_present.cpp of d6735da (git show), built with today's Makefile flags."""
        if cls._old is None:
            env = base.Env.get()
            d = env._dir / "old_helper"
            d.mkdir(exist_ok=True)
            src = subprocess.run(["git", "show", "d6735da:tools/kv_link/ar_helper/m7ar_present.cpp"],
                                 cwd=str(ROOT), stdout=subprocess.PIPE, check=True).stdout
            (d / "m7ar_present_d6735da.cpp").write_bytes(src)
            out = d / "m7ar_present_d6735da"
            r = subprocess.run(["g++"] + CXXFLAGS + ["-I", str(ROOT / "tools/kv_link/ar_helper"),
                                                     "-I", str(ROOT / "hw/m7"), "-isystem", HLS_INC,
                                                     "-o", str(out), str(d / "m7ar_present_d6735da.cpp"),
                                                     str(ROOT / "hw/m7/pl/hls/m7_ar_overlay_axi.cpp"),
                                                     str(ROOT / "hw/m7/ar_overlay.cpp")],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if r.returncode != 0:
                raise RuntimeError("old helper build failed:\n%s" % r.stdout.decode()[-3000:])
            cls._old = str(out)
        return cls._old


# ------------------------------------------------------------------ W: wire
def req_kw(fid=9):
    return dict(frame_id=fid, capture_ns=11, pose_ns=12,
                identity={k: "id_" + k for k in kv_link.IDENTITY_FIELDS}, width=base.WD, height=base.HD,
                nx=32, ny=32, nz=10, unit_f32=base.UNIT, voxel=base.VOXEL, origin=base.ORIGIN,
                inverse_k=np.eye(3), world_camera=np.eye(4)[:3], codes=base.synth_codes(), depth_crc=7)


class WireTests(unittest.TestCase):
    def test_bit3_round_trip_and_void_without_feature(self):
        self.assertEqual(kv_link.EXT_REPLAY, 8)
        buf = encode_request(replay=True, age_block=kv_link.encode_age_block(5), **req_kw())
        out = decode_request(buf, allow_age=True, allow_label=True, allow_replay=True)
        self.assertTrue(out["replay"])
        self.assertEqual(out["nano_elapsed_ns"], 5)
        plain = decode_request(encode_request(**req_kw()), allow_replay=True)
        self.assertFalse(plain["replay"])
        # negative control: a server that does not announce replay_ext voids the request
        with self.assertRaises(BadExtension) as cm:
            decode_request(buf, allow_age=True, allow_label=True)
        self.assertEqual(cm.exception.reason, "flags")
        with self.assertRaises(BadExtension):
            decode_request(encode_request(replay=True, **req_kw()))

    def test_unmarked_bytes_equal_d6735da(self):
        old = base.load_old("host/kv_link.py", "d6735da")
        for extra in ({}, {"roi": base.batch_of()}, {"replay": False}):
            kw = dict(req_kw(), **extra)
            want = old.encode_request(**{k: v for k, v in kw.items() if k != "replay"})
            self.assertEqual(encode_request(**kw), want)
        self.assertEqual(kv_link.FEATURES, old.FEATURES)
        self.assertNotEqual(encode_request(replay=True, **req_kw()), old.encode_request(**req_kw()))

    def _server(self, ar):
        env = base.Env.get()
        core = FrameNativeM7(env.frame_lib).lib.fpga_m7_depth_frame
        return KvLinkServer(core, "cpu", bind=("127.0.0.1", 0), roi_core=CpuRoiCore(env.obj_lib), ar=ar).start()

    def _one_frame(self, srv, data_source, fid=1):
        c = KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=2000, timeout_s=1.0)
        info = c.hello(2.0)
        be = RemotePlFrameBackend(c, depth_unit_m=base.UNIT)
        be.data_source = data_source
        ns = now_ns()
        be.frame_from_depth(identity=base.ident(fid, ns), **base.frame_kwargs(fid, base.synth_codes(fid), ns))
        return info, be, c

    def test_client_marks_only_announced_server(self):
        ar = base.FakeAr()
        srv = self._server(ar)
        try:
            info, be, _c = self._one_frame(srv, "replay")
            self.assertEqual(info["features"]["replay_ext"], 1)
            self.assertEqual(be.counts["replay_ext_sent"], 1)
            for _ in range(100):
                if ar.snaps:
                    break
                time.sleep(0.01)
            self.assertIs(ar.snaps[-1]["replay"], True)
            _i, be2, _c = self._one_frame(srv, "live", fid=2)
            self.assertEqual(be2.counts["replay_ext_sent"], 0)
            for _ in range(100):
                if len(ar.snaps) == 2:
                    break
                time.sleep(0.01)
            self.assertIs(ar.snaps[-1]["replay"], False)
        finally:
            srv.stop()
        off = self._server(None)            # --ar off: no replay_ext, the client sends no bit
        try:
            info, be, c = self._one_frame(off, "replay")
            self.assertNotIn("replay_ext", info["features"])
            self.assertEqual((be.counts["replay_ext_sent"], be.counts["replay_ext_unannounced"]), (0, 1))
            # a request that carries the bit anyway is void there (no answer)
            with self.assertRaises(LinkTimeout):
                c.call(encode_request(replay=True, **req_kw(fid=50)),
                       {"frame_id": 50, "capture_ns": 11, "pose_ns": 11,
                        "identity": {k: "x" for k in kv_link.IDENTITY_FIELDS}})
            self.assertEqual(off.counts["bad_ext:flags"], 1)
        finally:
            off.stop()


    def test_sim_source_sets_the_replay_mark(self):
        # line D1 (2026-10-14): a Gazebo frame (--data-source sim) is never shown as LIVE; the AR
        # core can only say LIVE or REPLAY, so sim requests carry the replay mark like a replay
        ar = base.FakeAr()
        srv = self._server(ar)
        try:
            _i, be, _c = self._one_frame(srv, "sim", fid=3)
            self.assertEqual(be.counts["replay_ext_sent"], 1)
            for _ in range(100):
                if ar.snaps:
                    break
                time.sleep(0.01)
            self.assertIs(ar.snaps[-1]["replay"], True)
        finally:
            srv.stop()


# ------------------------------------------------------------------ fill_ar_inputs
class FillTests(unittest.TestCase):
    def test_age_kept_for_replay_and_replay_is_or(self):
        lay = base.layout()
        regs = lambda inp: dict(zip(kv_ar.REG_NAMES, inp["regs"]))  # noqa: E731
        snap = base.synth_snap()
        for source, mark, want in (("live", False, 0), ("live", True, 1), ("replay", False, 1), ("replay", True, 1)):
            snap["replay"] = mark
            inp = kv_ar.fill_ar_inputs(snap, lay, source=source)
            r = regs(inp)
            self.assertEqual((r["replay"], r["age_valid"], inp["age_basis_ns"]), (want, 1, 4000000), (source, mark))
        snap = base.synth_snap(nano_elapsed_ns=None)
        snap["replay"] = True
        r = regs(kv_ar.fill_ar_inputs(snap, lay, source="live"))
        self.assertEqual((r["replay"], r["age_valid"]), (1, 0))
        # d6735da dropped the age of every replay frame (the decoupling is a real change)
        old = base.load_old("host/kv_ar.py", "d6735da")
        r = regs(old.fill_ar_inputs(base.synth_snap(), lay, source="replay"))
        self.assertEqual((r["replay"], r["age_valid"]), (1, 0))


# ------------------------------------------------------------------ H: helper
def frame_message(fid, *, lay, basis_ms, mark=False, source="live", module=kv_ar):
    snap = base.synth_snap(frame_id=fid, nano_elapsed_ns=None if basis_ms is None else int(basis_ms * 1e6))
    if mark:
        snap["replay"] = True
    inp = module.fill_ar_inputs(snap, lay, source=source, max_age_ms=MAX_AGE)
    inp["first_mono_ns"] = mono_ns()
    return kv_ar.encode_ar_message(inp)


# fid: (basis_ms, replay mark, --ar-source, expected panel[1], expected panel[2] prefix, data replay, core replay)
CASES = [
    (31, 2.0, False, "live", "LIVE", "AGE:0", 0, 0),
    (32, 2.0, True, "live", "REPLAY", "AGE:0", 1, 1),
    (33, 2.0, False, "replay", "REPLAY", "AGE:0", 1, 1),
    (34, 300.0, True, "live", "STALE", "AGE:3", 1, 0),
    (35, None, True, "live", "NOAGE", "AGE N/A", 1, 0),
    (36, 150.0, True, "live", "REPLAY", "AGE:1", 1, 1),     # fresh (150 < 200); then the fail-safe
]
STALE_MS = 1500


def run_helper(exe, d, module=kv_ar, cases=CASES):
    lay = kv_ar.ar_layout(320, 240, base.WD, base.HD)
    seqs = ",".join(str(2 + i) for i in range(len(cases)))
    p = subprocess.Popen([exe, "--provider", "mock", "--display", "none", "--mock-size", "320x240",
                          "--mock-permit", "1", "--stale-ms", str(STALE_MS), "--refresh-ms", "400",
                          "--dump-dir", d, "--dump-seq", seqs, "--dump-failsafe", "1"],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        ev = base.EventReader(p.stdout)
        ev.start()
        ev.wait(lambda e: e["ev"] == "ready")
        for (fid, basis, mark, source, *_rest) in cases:
            os.write(p.stdin.fileno(), frame_message(fid, lay=lay, basis_ms=basis, mark=mark, source=source,
                                                     module=module))
            ev.wait(lambda e, f=fid: e["ev"] == "frame" and e["frame_id"] == f)
        fs = ev.wait(lambda e: e["ev"] == "failsafe", timeout=10)
        p.stdin.close()
        p.send_signal(signal.SIGTERM)
        ev.wait(lambda e: e["ev"] == "exit")
        p.wait(10)
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()
    lib = base.Env.c_ref()
    bases = sorted(os.path.join(d, f[:-5]) for f in os.listdir(d) if f.endswith(".json"))
    rows = [ar_dump_check.check_one(b, lib)[0] for b in bases]
    by_seq = {e["seq"]: e for e in ev.events if e["ev"] in ("frame", "failsafe")}
    return rows, by_seq, fs


def panels(rows):
    return [(r["kind"], r["panel"][1], r["panel"][2]) for r in rows]


def expected(cases=CASES):
    return [("frame", c[4], c[5]) for c in cases] + [("failsafe", "STALE", "AGE>999MS")]


def matches(rows, cases=CASES):
    got, want = panels(rows), expected(cases)
    return len(got) == len(want) and all(g[0] == w[0] and g[1] == w[1] and g[2].startswith(w[2])
                                         for g, w in zip(got, want))


class HelperReplayTests(unittest.TestCase):
    def test_replay_age_rules_and_mode_mirror(self):
        with tempfile.TemporaryDirectory() as d:
            rows, by_seq, fs = run_helper(Bins.new(), d)
        self.assertTrue(matches(rows), panels(rows))
        for r in rows:
            self.assertTrue(r["equal"], r)
            self.assertTrue(r["c_ref_equal_fb"], r)
            e = by_seq[r["seq"]]
            self.assertEqual(e["mode"], r["mode"], r)                   # PS mirror == golden mode
            self.assertEqual(e["mode_name"], r["panel"][1], r)
            self.assertEqual(e["replay_core"], r["replay"], r)          # dump regs = written to the core
        frames = [by_seq[r["seq"]] for r in rows if r["kind"] == "frame"]
        self.assertEqual([(e["frame_id"], e["replay"], e["replay_core"]) for e in frames],
                         [(c[0], c[6], c[7]) for c in CASES])
        self.assertEqual([e["age_valid"] for e in frames], [0 if c[1] is None else 1 for c in CASES])
        # fail-safe after the fresh replay frame 36: STALE, data still replay, core 0, and the age
        # grows from the Nano basis (150 ms) -- d6735da restarted it at 0 for replay frames
        self.assertEqual((fs["frame_id"], fs["replay"], fs["replay_core"], fs["mode_name"]), (36, 1, 0, "STALE"))
        # since_frame_ms is taken after the render, the age before it: 50 ms slack, still far
        # from the 150 ms the old rule loses
        self.assertGreaterEqual(fs["age_ms"], 150 + fs["since_frame_ms"] - 50)

    def test_positive_controls_d6735da(self):
        """The same frames through d6735da's code: never this set of pictures."""
        with tempfile.TemporaryDirectory() as d:          # old helper + old fill (the whole old path)
            rows, by_seq, fs = run_helper(Bins.old(), d, module=base.load_old("host/kv_ar.py", "d6735da"))
        self.assertFalse(matches(rows), "old path not caught")
        got = dict((by_seq[r["seq"]]["frame_id"], r["panel"][1:3]) for r in rows if r["kind"] == "frame")
        self.assertEqual(got[33], ["REPLAY", "AGE N/A"])    # --ar-source replay: the age is dropped
        self.assertEqual(got[32][0], "LIVE")                # the mark is unknown: a replay shown as LIVE
        for r in rows:
            self.assertTrue(r["equal"], r)
        with tempfile.TemporaryDirectory() as d:          # old helper + new fill: no demotion
            rows, by_seq, fs = run_helper(Bins.old(), d)
        self.assertFalse(matches(rows))
        got = dict((by_seq[r["seq"]]["frame_id"], r["panel"][1:3]) for r in rows if r["kind"] == "frame")
        self.assertEqual(got[34][0], "REPLAY")              # 300 ms old replay frame: not STALE
        self.assertEqual(got[35], ["REPLAY", "AGE N/A"])    # replay frame without age: not NOAGE
        self.assertEqual((fs["frame_id"], fs["replay"]), (36, 0))
        self.assertLess(fs["age_ms"], 150 + fs["since_frame_ms"] - 50)  # replay fail-safe age from 0
        with tempfile.TemporaryDirectory() as d:          # new helper + old fill: age dropped
            old = base.load_old("host/kv_ar.py", "d6735da")
            rows, _s, _f = run_helper(Bins.new(), d, module=old,
                                      cases=[(34, 300.0, False, "replay", "STALE", "AGE:3", 1, 0)])
        self.assertFalse(matches(rows, [(34, 300.0, False, "replay", "STALE", "AGE:3", 1, 0)]))
        self.assertEqual(rows[0]["panel"][1], "NOAGE")


# ------------------------------------------------------------------ C: chain
class ChainTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="test_kv_ar_replay_chain_")
        self.d = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def _presenter(self, name):
        dump = os.path.join(self.d, name + "_dump")
        os.makedirs(dump)
        cmd = [Bins.new(), "--provider", "mock", "--display", "none", "--mock-size", "640x480",
               "--mock-permit", "1", "--stale-ms", "400", "--refresh-ms", "300",
               "--dump-dir", dump, "--dump-seq", "2"]
        out = os.path.join(self.d, name)
        os.makedirs(out)
        return kv_ar.ArPresenter(cmd, source="live", max_age_ms=MAX_AGE, out_dir=out), dump

    def _producer(self, srv, data_source, clock=now_ns):
        """(KvConnection, GeoProducer) on the real producer path; ``clock`` = the producer's clock."""
        from host.geo_producer import GeoProducer, KvConnection
        from host.m7_backend import M7BackendSelector
        from host.m7_rolling import compile_rolling
        conn = KvConnection(("127.0.0.1", srv.port), depth_unit_m=base.UNIT, timeout_s=1.0,
                            hello_timeout_s=2.0, data_source=data_source)
        self.assertTrue(conn.connect(), conn.reason)
        roll = M7BackendSelector().rolling_backend("cpu", cpu_library=compile_rolling(
            os.path.join(self.d, "roll_%s_c.so" % data_source)))
        return conn, GeoProducer(conn.backend, roll, depth_unit_m=base.UNIT, roi=False, clock=clock)

    def _producer_frame(self, srv, data_source):
        from host.geo_producer import GeoProducer, KvConnection
        from host.m7_backend import M7BackendSelector
        from host.m7_rolling import compile_rolling
        import test_kv_ar_slice as sl
        conn = KvConnection(("127.0.0.1", srv.port), depth_unit_m=base.UNIT, timeout_s=1.0,
                            hello_timeout_s=2.0, data_source=data_source)
        self.assertTrue(conn.connect(), conn.reason)
        roll = M7BackendSelector().rolling_backend("cpu", cpu_library=compile_rolling(
            os.path.join(self.d, "roll_%s.so" % data_source)))
        core = GeoProducer(conn.backend, roll, depth_unit_m=base.UNIT, roi=False)
        T = sl.rig_camera()
        codes = np.full((base.HD, base.WD), 1500, np.uint16)
        res = core.process(codes, base.K, depth_stamp_ns=1, acquired_ns=now_ns(), T_odom_camera=T,
                           robot_position_m=tuple(T[:3, 3]), calibration_id="c1")
        self.assertIsNotNone(res["grid"], res["nav_reason"])
        return conn

    def wait_telem(self, ar, pred, timeout=10.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            t = ar.telem()
            if pred(t):
                return t
            time.sleep(0.02)
        raise AssertionError("telem never matched: %s" % ar.telem())

    def test_replay_marker_to_telem(self):
        from host.geo_producer import SourceMarker
        from tools.car_geo_producer.replay_camera import source_marker
        # replay stand-in marker -> the producer's cross-check (frame_stats keys)
        sm = SourceMarker("replay")
        # source_warning: integration v5 (2026-10-12), None within the grace frames / with a matching marker
        self.assertEqual(sm.stats_fields(), {"source": "replay", "source_marker": None, "source_mismatch": False,
                                             "source_warning": None})
        sm.on_marker(json.dumps(source_marker("/x/rec30_prep/")))
        self.assertEqual(sm.stats_fields(), {"source": "replay", "source_mismatch": False, "source_warning": None,
                                             "source_marker": {"sensor_source": "replay", "replay_id": "rec30_prep",
                                                               "publisher": "replay_camera"}})
        live = SourceMarker("live")
        live.on_marker(json.dumps(source_marker("/x/rec30_prep")))
        self.assertTrue(live.stats_fields()["source_mismatch"])          # counted, not silent
        self.assertEqual(live.counts["frames_mismatch"], 1)
        live.on_marker("not json")
        self.assertEqual(live.counts["marker_bad"], 1)
        with self.assertRaises(ValueError):
            SourceMarker(None)
        # producer (data_source replay) -> wire bit -> server snapshot -> fill -> helper -> TELEM
        ar, dump = self._presenter("replay")
        env = base.Env.get()
        srv = KvLinkServer(FrameNativeM7(env.frame_lib).lib.fpga_m7_depth_frame, "cpu",
                           bind=("127.0.0.1", 0), ar=ar).start()
        try:
            conn = self._producer_frame(srv, "replay")
            self.assertEqual(conn.backend.counts["replay_ext_sent"], 1)
            self.assertEqual(conn.describe()["server_replay_ext"], 1)
            t = self.wait_telem(ar, lambda t: t["flips"] >= 1)
            self.assertEqual((t["mode"], t["replay"], t["replay_core"], t["age_valid"], t["frame_id"],
                              t["presenter"]), ("REPLAY", 1, 1, 1, 1, "drm"))
            self.assertLess(t["age_ms"], MAX_AGE)
            t = self.wait_telem(ar, lambda t: t["failsafe_active"])
            self.assertEqual((t["mode"], t["replay"], t["replay_core"], t["frame_id"]), ("STALE", 1, 0, 1))
            self.assertGreater(t["age_ms"], MAX_AGE)
            conn.close()
        finally:
            srv.stop()
            ar.close()
        row, _d = ar_dump_check.check_one(os.path.join(dump, "ar_000002"), base.Env.c_ref())
        self.assertTrue(row["equal"] and row["c_ref_equal_fb"], row)
        self.assertEqual(row["panel"][1], "REPLAY")
        regs = json.load(open(os.path.join(dump, "ar_000002.json")))["regs"]
        r = dict(zip(kv_ar.REG_NAMES, regs))
        # the producer's 32x32x10 odom block reaches the mini map (task: slice), layer 7
        self.assertEqual((r["grid_valid"], r["slice_y"], r["replay"], r["age_valid"]), (1, 7, 1, 1))

    def test_live_producer_shows_live(self):
        """A live producer frame -> no replay mark -> the core's LIVE mode while fresh.
        Not wall-clock dependent (10-16; it failed twice on a loaded host after a >200 ms whole-process
        stall): (1) the Nano side runs on a controlled clock (the producer's clock and the AGE block's
        kv_link.now_ns advance 0.1 ms per reading), so the pipeline's own 200 ms expiry and the Nano
        part of the age are fixed; (2) the KV part (first chunk -> the helper's ap_start, the helper's
        CLOCK_MONOTONIC) is judged per event with the unchanged 200 ms rule: the age the helper wrote
        decides which mode is correct (<= 200 ms LIVE, else STALE) and every rendered frame must obey
        it; a frame a host stall pushed past 200 ms is legitimately STALE and the next frame is sent
        (at most 6). At least one frame must be LIVE, with replay 0 / replay_core 0 / age_valid 1."""
        from unittest import mock
        ar, dump = self._presenter("live")
        env = base.Env.get()
        srv = KvLinkServer(FrameNativeM7(env.frame_lib).lib.fpga_m7_depth_frame, "cpu",
                           bind=("127.0.0.1", 0), ar=ar).start()
        start = now_ns()
        ticks = [0]

        def clock():
            ticks[0] += 1
            return start + 100000 * ticks[0]
        try:
            with mock.patch.object(kv_link, "now_ns", clock):
                conn, core = self._producer(srv, "live", clock=clock)
                import test_kv_ar_slice as sl
                T = sl.rig_camera()
                codes = np.full((base.HD, base.WD), 1500, np.uint16)
                seen, live = [], None
                for attempt in range(6):
                    seq0 = ar.telem()["render_seq"] or 0
                    res = core.process(codes, base.K, depth_stamp_ns=1 + attempt, acquired_ns=clock(),
                                       T_odom_camera=T, robot_position_m=tuple(T[:3, 3]), calibration_id="c1")
                    self.assertIsNotNone(res["grid"], res["nav_reason"])     # controlled clock: never expired
                    t = self.wait_telem(ar, lambda t: t["render_kind"] == "frame" and (t["render_seq"] or 0) > seq0)
                    self.assertEqual((t["replay"], t["replay_core"], t["age_valid"]), (0, 0, 1))
                    want = "LIVE" if t["age_ms"] <= MAX_AGE else "STALE"
                    self.assertEqual(t["mode"], want, t)                     # the 200 ms rule, per event
                    seen.append((t["mode"], t["age_ms"]))
                    if want == "LIVE":
                        live = t
                        break
                self.assertIsNotNone(live, "no fresh frame in %d tries: %s" % (len(seen), seen))
                self.assertEqual(conn.backend.counts["replay_ext_sent"], 0)
                conn.close()
        finally:
            srv.stop()
            ar.close()

    def test_data_source_required(self):
        from host.ros_geo_producer import parse_args
        with self.assertRaises(SystemExit):
            parse_args(["--kv-server", "203.0.113.1:1", "--ground-z-m", "0"])
        a, _ = parse_args(["--data-source", "replay", "--kv-server", "203.0.113.1:1", "--ground-z-m", "0"])
        self.assertEqual(a.data_source, "replay")
        rig = (ROOT / "tools/car_geo_producer/rig_inner.sh").read_text()
        self.assertIn('EXP=(--data-source "${DATA_SOURCE:-replay}"', rig)
        src = (ROOT / "host/ros_geo_producer.py").read_text()
        self.assertIn("fs.update(source.stats_fields())", src)
        self.assertIn("SENSOR_SOURCE_TOPIC, lambda m: source.on_marker(m.data), map_qos)", src)
        cam = (ROOT / "tools/car_geo_producer/replay_camera.py").read_text()
        self.assertIn('marker_pub = node.create_publisher(String, "/fpga/sensor_source", latched)', cam)
        self.assertIn("durability=DurabilityPolicy.TRANSIENT_LOCAL", cam)


if __name__ == "__main__":
    unittest.main()
