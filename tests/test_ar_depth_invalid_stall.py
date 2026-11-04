"""AR screen while the depth is invalid (docs/design/AR_DEPTH_INVALID_STALL_20261017.md, 2026-10-17).

Before: a depth frame with no valid pixel is refused by the Nano's M7Pipeline before the KV request
("no_valid_depth"), so the KV got no request, the AR sender had nothing to draw, the helper flipped
STALE after max_age+1 ms and the screen froze until the depth came back (board lnT: 518-702 ms).
Now the KV's AR sender pulls the newest colour itself (no-depth self pull) and shows it with an
all-zero depth and no geometry (core mode 4 NODATA, hatched mini map, no boxes); the helper composes
the colour in mode 4 too. No geometry core runs for such a picture (no permit renewal).

Checked here:
  T  ColourSink: a WANT with its own tolerance is judged with that tolerance (the default WANT keeps
     the sink's 10 ms: key_far as before).
  E  anchor / estimate known answers on ArPresenter internals (fake sink): target key = O + t - lead
     with O = S - (first_mono - nano_elapsed); age basis = t - (K - O); a key below 1e12 (frame index)
     is not pulled; max_s ends a run; after_ms 0 = off.
  G  fill_ar_inputs of a no-geometry snapshot: golden mode 4 NODATA, grid_valid 0, camera cell 32,
     no boxes; the overlay mask stays complete in mode 4 (every depth-rectangle pixel outside the mask
     is the PL's invalid-depth colour).
  H  helper (host build, mock provider): a mode-4 v2 message with colour is composited and the dump
     equals compose(golden) (ar_dump_check, C reference too); a5a6db8's helper (git show) does NOT
     compose it (negative control).
  C  chain, known answer: the real Nano GeoProducer (KvConnection, remote backend, M7Pipeline,
     RollingFusion) -> in-process KvLinkServer (g++ core) with ArPresenter (real helper, mock) + the
     colour bypass (ColourSource over loopback, camera thread at 30 Hz in a foreign sensor clock).
     8 valid frames, N = 20 all-zero frames, 8 valid at 10 Hz:
       new code: ~N NODATA pictures with colour inside the blind window, no STALE, no flip gap
       > 230 ms; the geometry core served all 36 frames (10-26 #2: blank requests; was 16); the rolling map refused the N
       frames (dropped:no_valid_depth = N, no delta).
       negative controls: a5a6db8's ArPresenter + helper, and today's with --ar-nodepth-after-ms 0:
       no picture at all in the blind window and the STALE fail-safe -- the stall the board showed.
Run: cd tests && PYTHONHASHSEED=0 PYTHONPATH=..:. python3 -m unittest test_ar_depth_invalid_stall
(3.13 and 3.8).
"""
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from collections import OrderedDict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tests"), str(ROOT / "tools" / "ar_display"), str(ROOT / "tools" / "kv_link")):
    if p not in sys.path:
        sys.path.insert(0, p)

from host import kv_ar, kv_colour  # noqa: E402
from host.kv_link import KvLinkServer  # noqa: E402
from host.m7_frame_native import FrameNativeM7  # noqa: E402
from host.fpga_clock import now_ns  # noqa: E402
import ar_dump_check  # noqa: E402
import colour_compose  # noqa: E402
import golden  # noqa: E402
import test_kv_ar as base  # noqa: E402
import test_kv_ar_colour as tc  # noqa: E402
import test_kv_ar_replay as rp  # noqa: E402

OLD = "a5a6db8"          # origin/main this branch starts from
WD, HD = base.WD, base.HD
OFFSET = 7 * 10 ** 15    # the camera's sensor clock = KV CLOCK_MONOTONIC + OFFSET (any clock works)


class OldBins:
    _bin = None
    _mod = None

    @classmethod
    def helper(cls):
        """m7ar_present.cpp of a5a6db8 (git show), today's flags (the build of test_kv_ar_replay)."""
        if cls._bin is None:
            env = base.Env.get()
            d = env._dir / "old_helper_a5a6db8"
            d.mkdir(exist_ok=True)
            src = subprocess.run(["git", "show", "%s:tools/kv_link/ar_helper/m7ar_present.cpp" % OLD],
                                 cwd=str(ROOT), stdout=subprocess.PIPE, check=True).stdout
            (d / "m7ar_present_old.cpp").write_bytes(src)
            out = d / "m7ar_present_old"
            r = subprocess.run(["g++"] + rp.CXXFLAGS + ["-I", str(ROOT / "tools/kv_link/ar_helper"),
                                                        "-I", str(ROOT / "hw/m7"), "-isystem", rp.HLS_INC,
                                                        "-o", str(out), str(d / "m7ar_present_old.cpp"),
                                                        str(ROOT / "hw/m7/pl/hls/m7_ar_overlay_axi.cpp"),
                                                        str(ROOT / "hw/m7/ar_overlay.cpp")],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if r.returncode != 0:
                raise RuntimeError("old helper build failed:\n%s" % r.stdout.decode()[-3000:])
            cls._bin = str(out)
        return cls._bin

    @classmethod
    def kv_ar(cls):
        if cls._mod is None:
            cls._mod = base.load_old("host/kv_ar.py", OLD)
        return cls._mod


def nodepth_snap(frame_id=5, sensor=None):
    snap = base.synth_snap(frame_id=frame_id, nano_elapsed_ns=20_000_000)
    snap = dict(snap, codes=np.zeros(WD * HD, dtype="<u2"), flags=np.zeros_like(np.asarray(snap["flags"])),
                roi=None, roi_status=None, roi_rows=[], labels=None, no_geometry=True)
    return snap


# ------------------------------------------------------------------ T: per-WANT tolerance
class ToleranceTests(unittest.TestCase):
    def test_want_tolerance_is_per_want(self):
        ring = kv_colour.ColourRing(4)
        ring.put(10 ** 15, WD, HD, tc.gradient_rgb(seed=1))
        sink = kv_colour.ColourSink(("127.0.0.1", 0), rcvbuf=1 << 20, tol_ns=10_000_000).start()
        src = kv_colour.ColourSource(("127.0.0.1", sink.port), ring.lookup, chunk_bytes=8000,
                                     hello_s=0.05).start()
        try:
            for _ in range(200):
                if sink.has_source():
                    break
                time.sleep(0.01)
            # 40 ms off: the sink's default 10 ms -> the source answers no_match (its tolerance is the WANT's)
            self.assertTrue(sink.want(1, 11, 10 ** 15 + 40_000_000))
            self.assertEqual(sink.take(1, 11, 1.0), (None, "none:no_match"))
            # the same key with a 67 ms WANT: the frame, judged against 67 ms (not refused as key_far)
            self.assertTrue(sink.want(1, 12, 10 ** 15 + 40_000_000, tol_ns=67_000_000))
            res, why = sink.take(1, 12, 1.0)
            self.assertIsNone(why)
            self.assertEqual(res["source_key"], 10 ** 15)
            self.assertEqual(sink.counts["key_far"], 0)
        finally:
            src.stop()
            sink.stop()


# ------------------------------------------------------------------ E: estimate, on presenter internals
class FakeSink:
    def __init__(self, rgb, delta_ns=0, source=True):
        self.rgb, self.delta_ns, self.source = rgb, delta_ns, source
        self.wants = []

    def has_source(self):
        return self.source

    def want(self, frame_id, capture_ns, key, tol_ns=None):
        self.wants.append((frame_id, capture_ns, key, tol_ns))
        return True

    def take(self, frame_id, capture_ns, timeout_s):
        key = self.wants[-1][2]
        return {"rgb": self.rgb, "width": WD, "height": HD, "source_key": key - self.delta_ns,
                "want_key": key}, None

    def describe(self):
        return {}


def bare_presenter(sink, **kw):
    """ArPresenter without a helper process (only the pull internals are exercised)."""
    p = kv_ar.ArPresenter.__new__(kv_ar.ArPresenter)
    p.colour, p.colour_main, p.tune, p.tune_file, p.colour_log = sink, True, {}, None, None
    p.colour_wait_ms = 40.0
    p.nodepth_after_ms = kw.get("after", 150.0)
    p.nodepth_period_ms, p.nodepth_tol_ms, p.nodepth_lead_ms = 100.0, 67.0, 33.0
    p.nodepth_max_s = kw.get("max_s", 60.0)
    p.nd_last_real = p.nd_offset_ns = p.nd_last_pull = p.nd_first_pull = p.nd_last = None
    p.nd_seq = 0
    import collections
    p.counts = collections.Counter()
    p.lock = threading.Lock()
    return p


class EstimateTests(unittest.TestCase):
    def test_anchor_target_and_age_known_answers(self):
        sink = FakeSink(tc.gradient_rgb(), delta_ns=25_000_000)
        p = bare_presenter(sink)
        real = base.synth_snap(frame_id=77, nano_elapsed_ns=30_000_000)
        real.update(sensor_stamp_ns=10 ** 15 + 5_000_000, first_mono_ns=2_000_000_000, capture_ns=123)
        t_take = time.monotonic()
        p._nd_note_real(real, t_take)
        O = 10 ** 15 + 5_000_000 - (2_000_000_000 - 30_000_000)
        self.assertEqual(p.nd_offset_ns, O)
        self.assertAlmostEqual(p._nd_due(t_take, {}), 0.150, places=6)
        snap, col, rec = p._nd_pull({})
        t_w = snap["first_mono_ns"]
        fid, tag, key, tol = sink.wants[-1]
        self.assertEqual((fid, tag, tol), (77, 1, 67_000_000))
        self.assertEqual(key, O + t_w - 33_000_000)
        k = key - 25_000_000
        self.assertEqual(snap["sensor_stamp_ns"], k)
        self.assertEqual(snap["nano_elapsed_ns"], t_w - (k - O))           # = lead + 25 ms
        self.assertEqual(snap["nano_elapsed_ns"], 58_000_000)
        self.assertTrue(snap["no_geometry"])
        self.assertFalse(np.any(snap["codes"]))
        self.assertFalse(np.any(snap["flags"]))
        self.assertIsNone(snap["roi"])
        self.assertEqual(snap["frame_id"], 77)
        self.assertIs(col["rgb"], sink.rgb)
        # next pull one period later; a real frame resets the run
        self.assertAlmostEqual(p._nd_due(p.nd_last_pull, {}), 0.100, places=6)
        p._nd_note_real(real, time.monotonic())
        self.assertIsNone(p.nd_last_pull)

    def test_no_pull_cases(self):
        sink = FakeSink(tc.gradient_rgb())
        p = bare_presenter(sink)
        self.assertIsNone(p._nd_due(time.monotonic(), {}))                 # no real frame yet
        real = base.synth_snap(frame_id=3, nano_elapsed_ns=1)
        real.update(sensor_stamp_ns=4, first_mono_ns=10)                    # replay key: frame index + 1
        p._nd_note_real(real, time.monotonic())
        self.assertIsNone(p._nd_pull({}))
        self.assertEqual(p.counts["nodepth_skip:key_not_time"], 1)
        self.assertEqual(sink.wants, [])
        self.assertIsNone(p._nd_due(time.monotonic(), {"nodepth_after_ms": 0}))   # tune off
        self.assertIsNone(bare_presenter(sink, after=0)._nd_due(time.monotonic(), {}))
        self.assertIsNone(p._nd_due(time.monotonic(), {"colour_main": 0}))        # colour off
        q = bare_presenter(sink, max_s=0.05)
        real.update(sensor_stamp_ns=10 ** 15)
        q._nd_note_real(real, time.monotonic() - 1.0)
        self.assertIsNotNone(q._nd_pull({}))
        q.nd_first_pull -= 0.1                                              # run older than max_s
        self.assertIsNone(q._nd_due(time.monotonic(), {}))


# ------------------------------------------------------------------ G: golden of a no-geometry picture
class GoldenTests(unittest.TestCase):
    def test_mode4_inputs_and_mask(self):
        lay = kv_ar.ar_colour_layout(tc.CANVAS[0], tc.CANVAS[1], WD, HD)
        inp = kv_ar.fill_ar_inputs(nodepth_snap(), lay, source="live", max_age_ms=1000)
        R = kv_ar.R
        self.assertEqual((inp["regs"][R["frame_valid"]], inp["regs"][R["pose_valid"]],
                          inp["regs"][R["grid_valid"]], inp["regs"][R["cam_cell_x"]],
                          inp["regs"][R["cam_cell_z"]], inp["boxes"]), (1, 1, 0, 32, 32, 0))
        self.assertFalse(np.any(inp["depth"]))
        regs = list(inp["regs"])
        regs[R["age_ms"]], regs[R["permit_i"]] = 60, 0
        st, fb, info = golden.render(inp["depth"], inp["grid"], inp["desc"], regs)
        self.assertEqual((st, info["mode"], info["boxes"]), (0, 4, 0))
        dx, dy = lay["depth_dx"], lay["depth_dy"]
        m = colour_compose.overlay_mask(regs, inp["desc"])[dy:dy + HD, dx:dx + WD]
        sub = fb[dy:dy + HD, dx:dx + WD]
        # not fresh: the core paints the whole depth rectangle with its magenta hatch (no depth picture);
        # outside the mask there is nothing but that hatch (the colour replaces exactly it; the PiP shows it)
        yy, xx = np.mgrid[dy:dy + HD, dx:dx + WD]
        hatch = np.where((((xx >> 3) + (yy >> 3)) & 1) == 1, 0xA000A0, 0x180018).astype(np.uint32)
        self.assertEqual(int((sub[~m] != hatch[~m]).sum()), 0)
        self.assertGreater(int((~m).sum()), WD * HD // 2)
        # positive control: the same snapshot WITH geometry (status 0, flags, depth) is mode 0
        snap = base.synth_snap(frame_id=5, nano_elapsed_ns=20_000_000)
        g = kv_ar.fill_ar_inputs(snap, lay, source="live", max_age_ms=1000)
        regs2 = list(g["regs"])
        regs2[R["age_ms"]], regs2[R["permit_i"]] = 60, 0
        self.assertEqual(golden.render(g["depth"], g["grid"], g["desc"], regs2)[2]["mode"], 0)
        self.assertEqual(g["regs"][R["grid_valid"]], 1)


# ------------------------------------------------------------------ H: helper composes mode 4
def nodepth_message(frame_id, lay, rgb):
    inp = kv_ar.fill_ar_inputs(nodepth_snap(frame_id), lay, source="live", max_age_ms=1000)
    inp["first_mono_ns"] = int(time.monotonic() * 1e9)
    inp["colour"] = {"pip": (lay["pip_x"], lay["pip_y"], lay["pip_div"]), "rgb": rgb, "width": WD, "height": HD}
    return kv_ar.encode_ar_message(inp)


class HelperTests(unittest.TestCase):
    def run_helper(self, exe):
        lay = kv_ar.ar_colour_layout(tc.CANVAS[0], tc.CANVAS[1], WD, HD)
        d = tempfile.mkdtemp(prefix="ndhelper_", dir=os.environ.get("TMPDIR"))
        p = subprocess.Popen([exe, "--provider", "mock", "--display", "none", "--mock-size", "%dx%d" % tc.CANVAS,
                              "--mock-permit", "1", "--stale-ms", "1500", "--refresh-ms", "400",
                              "--dump-dir", d, "--dump-seq", "2"],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        ev = base.EventReader(p.stdout)
        ev.start()
        ev.wait(lambda e: e["ev"] == "ready")
        try:
            os.write(p.stdin.fileno(), nodepth_message(61, lay, tc.gradient_rgb(seed=4)))
            e = ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 61)
        finally:
            tc.stop_helper(p)
        return e, d

    def test_new_helper_composes_nodata(self):
        e, d = self.run_helper(base.Env.helper())
        self.assertEqual((e["mode_name"], e["colour_mode"], e["colour"], e["pip"]), ("NODATA", 1, 1, 1))
        r, dd = ar_dump_check.check_one(os.path.join(d, "ar_000002"), base.Env.c_ref())
        self.assertEqual((r["mode_name"], r["colour"]), ("NODATA", "composited"), r)
        self.assertTrue(r["equal"], r)
        self.assertTrue(r["composed_mode_ok"], r)
        self.assertTrue(r["c_ref_equal_fb"], r)
        self.assertEqual(r["pl_differing_pixels"], 0)
        self.assertGreater(r["colour_pixels"], WD * HD // 2)
        self.assertEqual(dd["rgb"], tc.gradient_rgb(seed=4))

    def test_old_helper_does_not_compose_nodata(self):
        e, d = self.run_helper(OldBins.helper())
        self.assertEqual((e["mode_name"], e["colour_mode"], e["colour"]), ("NODATA", 1, 0))
        r, _ = ar_dump_check.check_one(os.path.join(d, "ar_000002"))
        self.assertEqual(r["colour"], "mode_on_not_composited")


# ------------------------------------------------------------------ C: chain, known answer
class Camera(threading.Thread):
    """30 Hz colour into the Nano ring, keyed by the sensor clock (KV mono + OFFSET)."""

    def __init__(self, ring):
        super().__init__(daemon=True)
        self.ring, self.stop_evt, self.latest = ring, threading.Event(), None
        self.frames = OrderedDict()
        self.lock = threading.Lock()

    def run(self):
        i = 0
        while not self.stop_evt.is_set():
            key = time.monotonic_ns() + OFFSET
            rgb = tc.gradient_rgb(seed=i % 50)
            self.ring.put(key, WD, HD, rgb)
            with self.lock:
                self.frames[key] = i % 50
                while len(self.frames) > 200:
                    self.frames.popitem(last=False)
                self.latest = key
            i += 1
            self.stop_evt.wait(1 / 30.0)


class ChainTests(unittest.TestCase):
    PRE, BLIND, POST = 8, 20, 8

    def scenario(self, *, old=False, after_ms=150.0):
        from host.geo_producer import GeoProducer, KvConnection
        from host.m7_backend import M7BackendSelector
        from host.m7_rolling import compile_rolling
        import test_kv_ar_slice as sl
        env = base.Env.get()
        d = tempfile.mkdtemp(prefix="ndchain_", dir=os.environ.get("TMPDIR"))
        out = os.path.join(d, "out")
        os.makedirs(out)
        sink = kv_colour.ColourSink(("127.0.0.1", 0), rcvbuf=4 << 20).start()
        exe = OldBins.helper() if old else base.Env.helper()
        cmd = [exe, "--provider", "mock", "--display", "none", "--mock-size", "640x480", "--mock-permit", "1",
               "--stale-ms", "201", "--refresh-ms", "500"]
        mod = OldBins.kv_ar() if old else kv_ar
        kw = {} if old else {"nodepth_after_ms": after_ms}
        ar = mod.ArPresenter(cmd, source="live", max_age_ms=200, out_dir=out, colour=sink, colour_main=True, **kw)
        srv = KvLinkServer(FrameNativeM7(env.frame_lib).lib.fpga_m7_depth_frame, "cpu",
                           bind=("127.0.0.1", 0), ar=ar).start()
        ring = kv_colour.ColourRing(12)
        src = kv_colour.ColourSource(("127.0.0.1", sink.port), ring.lookup, chunk_bytes=8000, hello_s=0.05).start()
        cam = Camera(ring)
        cam.start()
        rows, marks = [], {}
        try:
            for _ in range(300):
                if sink.has_source() and cam.latest is not None:
                    break
                time.sleep(0.01)
            conn = KvConnection(("127.0.0.1", srv.port), depth_unit_m=base.UNIT, timeout_s=1.0,
                                hello_timeout_s=2.0, data_source="live")
            self.assertTrue(conn.connect(), conn.reason)
            roll = M7BackendSelector().rolling_backend("cpu", cpu_library=compile_rolling(
                os.path.join(d, "roll.so")))
            core = GeoProducer(conn.backend, roll, depth_unit_m=base.UNIT, roi=False)
            T = sl.rig_camera()
            valid = np.full((HD, WD), 1500, np.uint16)
            zero = np.zeros((HD, WD), np.uint16)
            n = self.PRE + self.BLIND + self.POST
            t_next = time.monotonic()
            for i in range(n):
                blind = self.PRE <= i < self.PRE + self.BLIND
                if i == self.PRE + self.BLIND:
                    marks["blind_end"] = time.monotonic()
                with cam.lock:
                    key = cam.latest
                res = core.process(zero if blind else valid, base.K, depth_stamp_ns=key, acquired_ns=now_ns(),
                                   T_odom_camera=T, robot_position_m=tuple(T[:3, 3]), calibration_id="c1")
                rows.append((i, blind, res["result"], res["delta"] is not None))
                if i == self.PRE - 1:
                    marks["blind_start"] = time.monotonic()
                t_next += 0.1
                time.sleep(max(0.0, t_next - time.monotonic()))
            time.sleep(0.4)
            served = srv.counts["served"]
            fusion = dict(core.fusion.counts)
            blank = (core.pipeline.blank_requests, core.pipeline.blank_request_failures)
            telem = ar.telem()
            conn.close() if hasattr(conn, "close") else None
        finally:
            cam.stop_evt.set()
            srv.stop()
            ar.close()
            src.stop()
            sink.stop()
        evs = [json.loads(l) for l in open(os.path.join(out, "ar_events.jsonl"))]
        colour_log = [json.loads(l) for l in open(os.path.join(out, "ar_colour.jsonl"))]
        with cam.lock:
            frames = dict(cam.frames)
        return {"rows": rows, "marks": marks, "served": served, "fusion": fusion, "blank": blank, "evs": evs,
                "colour_log": colour_log, "telem": telem, "frames": frames}

    def window(self, r):
        """Flips strictly between the last real picture before the blind frames (Nano frame id PRE: the
        blind frames' ids reach the KV only since 10-26, as blank requests) and the first real picture after them (id PRE+BLIND+1),
        and the frame-to-frame gaps from the one to the other."""
        flips = [e for e in r["evs"] if e.get("ev") in ("frame", "failsafe")]
        last = [e for e in flips if e["ev"] == "frame" and e["frame_id"] == self.PRE and e["mode_name"] == "LIVE"]
        first = [e for e in flips if e["ev"] == "frame" and e["frame_id"] == self.PRE + self.BLIND + 1]
        self.assertTrue(last and first, "real flips around the window missing")
        a, b = last[-1]["seq"], first[0]["seq"]
        inside = [e for e in flips if a < e["seq"] < b]
        span = [e for e in flips if a <= e["seq"] <= b and e["ev"] == "frame"]
        gaps = [(y["flip_mono_ns"] - x["flip_mono_ns"]) / 1e6 for x, y in zip(span, span[1:])]
        return inside, gaps

    def common_nano_side(self, r):
        """The Nano side is the same code in every arm: blind frames refused, never fused; since 10-26 (#2, owner：重成功率)
        each is still sent as a blank request (all-zero codes) so the PL core runs and renews the motion permit."""
        blind = [row for row in r["rows"] if row[1]]
        self.assertEqual(len(blind), self.BLIND)
        self.assertTrue(all(row[2] == "dropped:no_valid_depth" and not row[3] for row in blind), blind[:3])
        self.assertEqual(r["fusion"].get("dropped:no_valid_depth"), self.BLIND)
        self.assertTrue(all(row[2] == "fused" for row in r["rows"] if not row[1]), r["rows"])
        # 10-26 #2: the core also runs for the blind frames (blank requests; was PRE + POST before 10-26)
        self.assertEqual(r["served"], self.PRE + self.BLIND + self.POST)
        # ... and every blank answer checked out (a depth-crc mismatch or any link error on a blank request counts
        # in blank_request_failures): the KV converted the all-zero codes to exactly the all-NaN depth sent
        self.assertEqual(r["blank"], (self.BLIND, 0))

    def test_new_code_shows_colour_nodata_through_the_blind_window(self):
        r = self.scenario()
        self.common_nano_side(r)
        inside, gaps = self.window(r)
        nod = [e for e in inside if e["ev"] == "frame" and e["mode_name"] == "NODATA"]
        self.assertGreaterEqual(len(nod), self.BLIND - 4, [e["mode_name"] for e in inside])
        self.assertLessEqual(len(nod), self.BLIND + 1)
        self.assertTrue(all(e["colour"] == 1 for e in nod), nod[:2])
        self.assertEqual([e for e in inside if e["ev"] == "failsafe"], [])
        self.assertEqual([e for e in inside if e["ev"] == "frame" and e["mode_name"] != "NODATA"], [])
        self.assertLessEqual(max(gaps), 230.0, gaps)
        pulls = [c for c in r["colour_log"] if c["decision"] == "nodepth"]
        # (the producer stops after the last frame, so the KV pulls on after the TELEM snapshot: >=)
        self.assertGreaterEqual(len(pulls), r["telem"]["colour"]["nodepth"]["sent"])
        # 10-26 #2: the blind pictures now come from the blank requests (the KV gets every frame id), so the
        # no-depth self pull fires only for the few it still beats; was: nodepth sent >= len(nod)
        for c in pulls:                                   # a real camera frame, recent, honestly aged
            self.assertIn(c["source_key"], r["frames"])
            self.assertLess(abs(c["source_key"] - c["want_key"]), 67_000_000)
            self.assertTrue(0 <= c["age_basis_ms"] <= 150, c)
        # the real frames still pair exactly (10 ms), outside the window
        self.assertGreaterEqual(sum(1 for c in r["colour_log"] if c["decision"] == "paired"), self.PRE + self.POST - 1)

    def assert_stall(self, r):
        """10-26 #2 (owner：重成功率): the Nano now sends a blank request for every blind frame, so even the old AR
        code / --ar-nodepth-after-ms 0 keeps flipping (NODATA, no failsafe) -- before 10-26 these arms froze (no
        picture inside the window, STALE, gap > (BLIND-2)*100 ms). No colour self pull in these arms."""
        self.common_nano_side(r)
        inside, gaps = self.window(r)
        self.assertGreaterEqual(len([e for e in inside if e["ev"] == "frame"]), self.BLIND - 4)
        self.assertEqual([e for e in inside if e["ev"] == "failsafe"], [])
        self.assertLessEqual(max(gaps), 230.0, gaps)
        self.assertEqual([c for c in r["colour_log"] if c["decision"] == "nodepth"], [])

    def test_negative_control_old_code_stalls(self):
        self.assert_stall(self.scenario(old=True))

    def test_negative_control_switched_off_stalls(self):
        self.assert_stall(self.scenario(after_ms=0))


if __name__ == "__main__":
    unittest.main()
