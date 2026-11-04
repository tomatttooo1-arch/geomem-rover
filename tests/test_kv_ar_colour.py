"""AR colour main view (owner 10-15; 2026-10-16): colour bypass, PS composition, checker.

What is checked (known answers, an independent numpy reference, positive controls):
  * layout: ar_colour_layout at 1024x600 for 640x360 / 640x480 (known values) and the three areas
    (colour picture incl. label-bar spill, PiP, mini map incl. ring and label) painted into a canvas
    never share a pixel; a tune override that overlaps is rejected (default kept);
  * REQ STAMP block: round trip, crc / version / rsv defects void the request, a server that does not
    announce stamp_ext voids the bit, the client sends it only to a server that announces it;
  * the numpy overlay mask is complete: inside the depth rectangle of a golden render, every pixel
    outside the mask is exactly the pseudo colour of its depth (no PL overlay pixel is missed);
  * helper (host build, mock provider): a v2 message with colour -> composited dump; ar_dump_check:
    .fbpl == golden and .fb == colour_compose.compose(golden); v2 with no colour and v1 -> the PL
    picture unchanged; the fail-safe (STALE) picture is never composited; mutants mask_shift / pip_row
    (helper --colour-mutant) are caught, with the PL part still equal and the differences only where
    the mutant acts; a half-written message does not block the fail-safe timer;
  * bypass: ColourSink <-> ColourSource over loopback (exact pairing, NONE reasons, a source that
    breaks the tolerance is refused);
  * presenter: colour on -> v2 + composited event; colour missing -> depth fallback with the reason in
    TELEM ar.colour; tune colour_main 0 -> version 1 bytes; no bypass -> TELEM keys as before;
  * end to end: kv_server --ar drm (mock) --ar-colour main + nano_replay --colour-port with a prepared
    colour file: every frame paired, dumps composited and equal, the dumped colour == the prepared
    colour of that source frame.
Run: cd tests && PYTHONPATH=.. python3 -m unittest discover -s . -p test_kv_ar_colour.py (3.13 and 3.8).
"""
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "ar_display"))
sys.path.insert(0, str(ROOT / "tools" / "kv_link"))

from host import kv_ar, kv_colour, kv_link  # noqa: E402
from host.kv_link import BadExtension, decode_request, encode_request  # noqa: E402
import ar_dump_check  # noqa: E402
import colour_compose  # noqa: E402
import golden  # noqa: E402
import test_kv_ar as base  # noqa: E402

WD, HD = base.WD, base.HD
CANVAS = (320, 240)


def gradient_rgb(wd=WD, hd=HD, seed=0):
    yy, xx = np.mgrid[0:hd, 0:wd]
    rgb = np.stack([(xx * 2 + yy + seed) % 256, (yy * 3 + 5 * seed) % 256, (xx * 5 + 7 + yy % 3) % 256], -1)
    return rgb.astype(np.uint8).tobytes()


def colour_message(frame_id, lay, rgb=b"", max_age_ms=1000, version2=True):
    snap = base.synth_snap(frame_id=frame_id, nano_elapsed_ns=2000000)
    inp = kv_ar.fill_ar_inputs(snap, lay, source="live", max_age_ms=max_age_ms)
    inp["first_mono_ns"] = int(time.monotonic() * 1e9)
    if version2:
        inp["colour"] = {"pip": (lay["pip_x"], lay["pip_y"], lay["pip_div"]), "rgb": rgb or None,
                         "width": WD, "height": HD}
    return kv_ar.encode_ar_message(inp)


def start_helper(d, *extra, size=CANVAS, stale_ms=1500):
    exe = base.Env.helper()
    p = subprocess.Popen([exe, "--provider", "mock", "--display", "none", "--mock-size", "%dx%d" % size,
                          "--mock-permit", "1", "--stale-ms", str(stale_ms), "--refresh-ms", "400",
                          "--dump-dir", d] + list(extra),
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    import fcntl
    fcntl.fcntl(p.stdin.fileno(), kv_ar.F_SETPIPE_SZ, 1 << 20)
    ev = base.EventReader(p.stdout)
    ev.start()
    ev.wait(lambda e: e["ev"] == "ready")
    return p, ev


def stop_helper(p):
    if p.poll() is None:
        p.send_signal(signal.SIGTERM)
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait()


def rect_inside(rc, xy):
    x0, y0, x1, y1 = rc
    return x0 <= xy[1] < x1 and y0 <= xy[0] < y1


class LayoutTests(unittest.TestCase):
    def paint(self, lay, wd, hd):
        """Count per pixel how many of the three areas cover it (independent of colour_layout_problem)."""
        W, H = lay["width"], lay["height"]
        c = np.zeros((H + 200, W + 200), np.int32)
        dx, dy = lay["depth_dx"], lay["depth_dy"]
        c[max(40, dy - 12):dy + hd, dx:dx + wd + 47] += 1              # picture + label-bar spill
        pw, ph = kv_ar.pip_size(wd, hd, lay["pip_div"])
        c[lay["pip_y"]:lay["pip_y"] + ph, lay["pip_x"]:lay["pip_x"] + pw] += 1
        mx, my = lay["mini_x"], lay["mini_y"]
        c[my - 12:my + 129, mx - 1:mx + 129] += 1
        return c

    def test_known_layouts_disjoint(self):
        for (wd, hd), want in (((640, 480), (0, 80, 688, 48, 784, 309)),
                               ((640, 360), (0, 140, 688, 48, 784, 249))):
            lay = kv_ar.ar_colour_layout(1024, 600, wd, hd)
            self.assertEqual(tuple(lay[k] for k in ("depth_dx", "depth_dy", "pip_x", "pip_y", "mini_x", "mini_y")),
                             want)
            c = self.paint(lay, wd, hd)
            self.assertEqual(int(c.max()), 1, (wd, hd))                  # no pixel in two areas
            self.assertEqual(int(c[:, 1024:].sum() + c[600:, :].sum()), 0)   # all inside the canvas
            self.assertEqual(int(c[:40].sum()), 0)                       # nothing on the panel
            self.assertIsNone(kv_ar.colour_layout_problem(lay, wd, hd))
        self.assertIsNotNone(kv_ar.ar_colour_layout(CANVAS[0], CANVAS[1], WD, HD))
        # the depth-main layout the owner tuned would put the PiP on the picture: refused
        bad = dict(kv_ar.ar_colour_layout(1024, 600, 640, 480), depth_dx=192)
        self.assertEqual(kv_ar.colour_layout_problem(bad, 640, 480), "depth_overlaps_pip")
        bad = dict(kv_ar.ar_colour_layout(1024, 600, 640, 480), mini_y=200)
        self.assertEqual(kv_ar.colour_layout_problem(bad, 640, 480), "pip_overlaps_mini")
        bad = dict(kv_ar.ar_colour_layout(1024, 600, 640, 480), pip_x=720)
        self.assertEqual(kv_ar.colour_layout_problem(bad, 640, 480), "pip_outside")


class StampWireTests(unittest.TestCase):
    def req(self, key):
        return base.request(roi=True, labels=None, age=4000000, fid=9)

    def test_round_trip_defects_and_feature(self):
        args = dict(frame_id=9, capture_ns=1, pose_ns=1, identity={k: "x" for k in kv_link.IDENTITY_FIELDS},
                    width=4, height=2, nx=2, ny=2, nz=2, unit_f32=0.001, voxel=0.05, origin=(0, 0, 0),
                    inverse_k=np.eye(3), world_camera=np.zeros(12), codes=np.arange(8, dtype=np.uint16),
                    depth_crc=1)
        plain = encode_request(**args)
        st = encode_request(sensor_key=123456789012345, **args)
        aged = encode_request(sensor_key=7, age_block=kv_link.encode_age_block(5000), **args)
        self.assertEqual(len(st), len(plain) + kv_link.STAMP_BLOCK_BYTES)
        self.assertEqual(decode_request(plain)["sensor_stamp_ns"], None)
        self.assertEqual(decode_request(st, allow_stamp=True)["sensor_stamp_ns"], 123456789012345)
        r = decode_request(aged, allow_age=True, allow_stamp=True)
        self.assertEqual((r["sensor_stamp_ns"], r["nano_elapsed_ns"]), (7, 5000))
        self.assertTrue(np.array_equal(r["codes"], np.arange(8)))
        with self.assertRaises(BadExtension):
            decode_request(st)                                      # server without stamp_ext
        for off, why in ((len(st) - 1, "crc"), (len(st) - kv_link.STAMP_BLOCK_BYTES, "ver")):
            b = bytearray(st)
            b[off] ^= 0x5A
            with self.assertRaises(BadExtension, msg=why):
                decode_request(bytes(b), allow_stamp=True)

    def test_client_sends_only_when_announced(self):
        env = base.Env.get()
        from host.m7_frame_native import FrameNativeM7
        for announce in (False, True):
            ar = FakeColourAr() if announce else base.FakeAr()
            srv = kv_link.KvLinkServer(FrameNativeM7(env.frame_lib).lib.fpga_m7_depth_frame, "cpu",
                                       bind=("127.0.0.1", 0), ar=ar).start()
            try:
                feats = srv.info["features"]
                self.assertEqual(feats.get("stamp_ext"), 1 if announce else None)
                cli = kv_link.KvLinkClient(("127.0.0.1", srv.port), timeout_s=2.0)
                cli.hello(2.0)
                be = kv_link.RemotePlFrameBackend(cli, depth_unit_m=base.UNIT)
                codes = base.synth_codes(1)
                be.set_next_sensor_key(4242)
                ns = kv_link.now_ns()
                be.frame_from_depth(**base.frame_kwargs(5, codes, ns), identity=base.ident(5, ns), codes=codes)
                self.assertEqual(be.counts["stamp_ext_sent"], 1 if announce else 0)
                deadline = time.monotonic() + 5
                while not ar.snaps and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(ar.snaps[-1]["sensor_stamp_ns"], 4242 if announce else None)
                if announce:
                    self.assertEqual(ar.early_keys, [4242])
                cli.close()
            finally:
                srv.stop()


class FakeColourAr(base.FakeAr):
    colour = object()

    def __init__(self):
        super().__init__()
        self.early_keys = []

    def early(self, req):
        self.early_keys.append(req["sensor_stamp_ns"])


class ReferenceTests(unittest.TestCase):
    def test_mask_complete_on_golden(self):
        lay = kv_ar.ar_colour_layout(CANVAS[0], CANVAS[1], WD, HD)
        snap = base.synth_snap(frame_id=3)
        inp = kv_ar.fill_ar_inputs(snap, lay, source="live", max_age_ms=1000)
        regs = list(inp["regs"])
        regs[8], regs[15] = 5, 1
        st, fb, info = golden.render(inp["depth"], inp["grid"], inp["desc"], regs)
        self.assertEqual((st, info["mode"]), (0, 0))
        self.assertGreater(info["boxes"], 0)
        dx, dy = lay["depth_dx"], lay["depth_dy"]
        z = np.asarray(inp["depth"], np.int64).reshape(HD, WD)
        valid = (z >= 100) & (z <= 9999)
        lutc = np.where(valid, golden.LUT[np.clip(z, 0, 9999) // 32], 0)
        m = colour_compose.overlay_mask(regs, inp["desc"])[dy:dy + HD, dx:dx + WD]
        sub = fb[dy:dy + HD, dx:dx + WD]
        self.assertEqual(int(((sub != lutc) & ~m).sum()), 0)        # no overlay pixel outside the mask
        self.assertGreater(int(m.sum()), 100)
        self.assertGreater(int((~m).sum()), WD * HD // 2)
        out = colour_compose.compose(fb, regs, inp["desc"], gradient_rgb(), (lay["pip_x"], lay["pip_y"], 2))
        col = colour_compose.rgb_to_xrgb(gradient_rgb(), WD, HD)
        self.assertTrue(np.array_equal(out[dy:dy + HD, dx:dx + WD][~m], col[~m]))
        self.assertTrue(np.array_equal(out[dy:dy + HD, dx:dx + WD][m], sub[m]))
        pw, ph = kv_ar.pip_size(WD, HD)
        self.assertTrue(np.array_equal(out[lay["pip_y"]:lay["pip_y"] + ph, lay["pip_x"]:lay["pip_x"] + pw],
                                       sub[::2, ::2]))


class HelperColourTests(unittest.TestCase):
    def test_composite_dumps_and_fallbacks(self):
        lay = kv_ar.ar_colour_layout(CANVAS[0], CANVAS[1], WD, HD)
        with tempfile.TemporaryDirectory() as d:
            p, ev = start_helper(d, "--dump-seq", "2,3,4", "--dump-failsafe", "1", stale_ms=1200)
            try:
                os.write(p.stdin.fileno(), colour_message(21, lay, gradient_rgb()))
                e = ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 21)
                self.assertEqual((e["colour_mode"], e["colour"], e["pip"], e["mode_name"]), (1, 1, 1, "LIVE"))
                self.assertGreater(e["composite_ns"], 0)
                os.write(p.stdin.fileno(), colour_message(22, lay, b""))           # colour on, none paired
                e = ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 22)
                self.assertEqual((e["colour_mode"], e["colour"]), (1, 0))
                os.write(p.stdin.fileno(), colour_message(23, lay, version2=False))   # colour off: v1
                e = ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 23)
                self.assertEqual((e["colour_mode"], e["colour"]), (0, 0))
                os.write(p.stdin.fileno(), colour_message(24, lay, gradient_rgb(seed=3)))
                ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 24)
                fs = ev.wait(lambda e: e["ev"] == "failsafe", timeout=10)        # last frame had colour
                self.assertEqual((fs["colour"], fs["mode_name"]), (0, "STALE"))
            finally:
                stop_helper(p)
            lib = base.Env.c_ref()
            rows = {}
            for f in sorted(os.listdir(d)):
                if f.endswith(".json"):
                    r, _ = ar_dump_check.check_one(os.path.join(d, f[:-5]), lib)
                    rows[r["seq"]] = r
            self.assertEqual(sorted(rows), [2, 3, 4, 6])
            for r in rows.values():
                self.assertTrue(r["equal"], r)
                self.assertTrue(r["c_ref_equal_fb"], r)
            self.assertEqual(rows[2]["colour"], "composited")
            self.assertEqual(rows[2]["pl_differing_pixels"], 0)
            self.assertGreater(rows[2]["colour_pixels"], WD * HD // 2)
            self.assertEqual(rows[3]["colour"], "mode_on_not_composited")
            self.assertNotIn("colour", rows[4])                                  # v1 meta: colour null
            self.assertEqual((rows[6]["kind"], rows[6]["colour"], rows[6]["mode_name"]),
                             ("failsafe", "mode_on_not_composited", "STALE"))
            # the dumped colour is the colour sent, and the composed picture really shows it
            self.assertEqual(open(os.path.join(d, "ar_000002.rgb"), "rb").read(), gradient_rgb())
            fb = np.fromfile(os.path.join(d, "ar_000002.fb"), "<u4").reshape(CANVAS[1], CANVAS[0])
            pl = np.fromfile(os.path.join(d, "ar_000002.fbpl"), "<u4").reshape(CANVAS[1], CANVAS[0])
            self.assertGreater(int((fb != pl).sum()), WD * HD // 2)

    def run_mutant(self, mutant):
        lay = kv_ar.ar_colour_layout(CANVAS[0], CANVAS[1], WD, HD)
        with tempfile.TemporaryDirectory() as d:
            p, ev = start_helper(d, "--dump-seq", "2", "--colour-mutant", mutant)
            try:
                os.write(p.stdin.fileno(), colour_message(31, lay, gradient_rgb()))
                ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 31)
            finally:
                stop_helper(p)
            base_ = os.path.join(d, "ar_000002")
            r, dd = ar_dump_check.check_one(base_)
            want = colour_compose.compose(dd["fbpl"], dd["meta"]["regs"], dd["desc"], dd["rgb"],
                                          dd["meta"]["colour"]["pip"])
            diff = np.argwhere(dd["fb"] != want)
            # the same mutant in the reference reproduces the helper's picture exactly
            mut = colour_compose.compose(dd["fbpl"], dd["meta"]["regs"], dd["desc"], dd["rgb"],
                                         dd["meta"]["colour"]["pip"], mutant=mutant)
            return r, diff, np.array_equal(mut, dd["fb"]), lay

    def test_mutants_caught(self):
        for mutant in ("mask_shift", "pip_row"):
            r, diff, same, lay = self.run_mutant(mutant)
            self.assertFalse(r["equal"], mutant)
            self.assertEqual(r["pl_differing_pixels"], 0, mutant)       # the PL part is still right
            self.assertGreater(r["composite_differing_pixels"], 0, mutant)
            self.assertTrue(same, mutant)
            if mutant == "mask_shift":
                area = (lay["depth_dx"], lay["depth_dy"], lay["depth_dx"] + WD, lay["depth_dy"] + HD)
            else:
                pw, ph = kv_ar.pip_size(WD, HD)
                area = (lay["pip_x"], lay["pip_y"], lay["pip_x"] + pw, lay["pip_y"] + ph)
            self.assertTrue(all(rect_inside(area, xy) for xy in diff), (mutant, diff[:5]))

    def test_half_message_does_not_block_failsafe(self):
        lay = kv_ar.ar_colour_layout(CANVAS[0], CANVAS[1], WD, HD)
        with tempfile.TemporaryDirectory() as d:
            p, ev = start_helper(d, stale_ms=400)
            try:
                os.write(p.stdin.fileno(), colour_message(41, lay, gradient_rgb()))
                ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 41)
                msg = colour_message(42, lay, gradient_rgb(seed=1))
                os.write(p.stdin.fileno(), msg[:len(msg) // 2])
                fs = ev.wait(lambda e: e["ev"] == "failsafe", timeout=5)       # timer runs mid-message
                self.assertEqual(fs["frame_id"], 41)
                os.write(p.stdin.fileno(), msg[len(msg) // 2:])
                e = ev.wait(lambda e: e["ev"] == "frame" and e["frame_id"] == 42)
                self.assertEqual(e["colour"], 1)
            finally:
                stop_helper(p)


class BypassTests(unittest.TestCase):
    def test_pull_pairing_none_and_tolerance(self):
        frames = {1000: gradient_rgb(seed=1), 34000000: gradient_rgb(seed=2)}
        ring = kv_colour.ColourRing(4)
        for k, v in frames.items():
            ring.put(k, WD, HD, v)
        sink = kv_colour.ColourSink(("127.0.0.1", 0), rcvbuf=1 << 20, tol_ns=5000000).start()
        src = kv_colour.ColourSource(("127.0.0.1", sink.port), ring.lookup, chunk_bytes=8000,
                                     hello_s=0.05).start()
        try:
            end = time.monotonic() + 5
            while not sink.has_source() and time.monotonic() < end:
                time.sleep(0.01)
            self.assertTrue(sink.has_source())
            self.assertTrue(sink.want(7, 70, 1000 + 3000000))                 # 3 ms off: the 1000 frame
            self.assertTrue(sink.want(8, 80, 17000000))                       # 17 ms from both: none
            self.assertTrue(sink.want(9, 90, 34000000))
            res, why = sink.take(7, 70, 2.0)
            self.assertIsNone(why)
            self.assertEqual((res["source_key"], res["width"], res["height"], res["rgb"]),
                             (1000, WD, HD, frames[1000]))
            self.assertEqual(sink.take(8, 80, 2.0), (None, "none:no_match"))
            self.assertEqual(sink.take(9, 90, 2.0)[0]["rgb"], frames[34000000])
            self.assertEqual(sink.take(10, 100, 0.05), (None, "not_wanted"))
            self.assertEqual(sink.describe()["counts"]["frames"], 2)
        finally:
            src.stop()
            sink.stop()
        # a source that ignores the tolerance is refused by the KV (independent check)
        sink = kv_colour.ColourSink(("127.0.0.1", 0), rcvbuf=1 << 20, tol_ns=5000000).start()
        src = kv_colour.ColourSource(("127.0.0.1", sink.port), lambda k, t: (k + 9000000, WD, HD, gradient_rgb()),
                                     hello_s=0.05).start()
        try:
            end = time.monotonic() + 5
            while not sink.has_source() and time.monotonic() < end:
                time.sleep(0.01)
            sink.want(1, 1, 5000)
            self.assertEqual(sink.take(1, 1, 2.0), (None, "key_far"))
        finally:
            src.stop()
            sink.stop()


class CameraRatePairingTests(unittest.TestCase):
    """10-29: at 15 fps the Nano says so in its HELLO and the KV pairs with half that period + 5 ms (39 ms, not 17)."""

    def test_pair_tol_follows_hello(self):
        f = kv_colour.pair_tol_ns
        self.assertEqual(f(17000000, None), 17000000)
        self.assertEqual(f(17000000, {"source": "live"}), 17000000)            # older Nano / 30 fps
        self.assertEqual(f(17000000, {"camera_fps": 30}), 17000000)
        self.assertEqual(f(17000000, {"camera_fps": 60}), 17000000)
        self.assertEqual(f(17000000, {"camera_fps": 15}), 39000000)            # >= the runner's 0.038 s slop
        self.assertEqual(f(17000000, {"camera_fps": 10.0}), 55000000)
        self.assertEqual(f(17000000, {"camera_fps": 5}), 105000000)
        self.assertEqual(f(60000000, {"camera_fps": 15}), 60000000)            # never narrower than configured
        for bad in ("x", None, float("nan"), 0, -15, 1e9):
            self.assertEqual(f(17000000, {"camera_fps": bad}), 17000000, bad)

    def run_pair(self, hello_info):
        ring = kv_colour.ColourRing(4)
        ring.put(1000, WD, HD, gradient_rgb(seed=3))
        sink = kv_colour.ColourSink(("127.0.0.1", 0), rcvbuf=1 << 20, tol_ns=17000000).start()
        src = kv_colour.ColourSource(("127.0.0.1", sink.port), ring.lookup, chunk_bytes=8000, hello_s=0.05,
                                     info=hello_info).start()
        try:
            end = time.monotonic() + 5
            while not sink.has_source() and time.monotonic() < end:
                time.sleep(0.01)
            self.assertTrue(sink.want(1, 10, 1000 + 36000000))        # 36 ms off: inside 39, outside 17 (and old 34)
            return sink.take(1, 10, 2.0), sink.describe()["pair_tol_ns"]
        finally:
            src.stop()
            sink.stop()

    def test_sink_uses_the_wider_tolerance_end_to_end(self):
        (res, why), tol = self.run_pair({"source": "live", "camera_fps": 15})
        self.assertIsNone(why)
        self.assertEqual((res["source_key"], tol), (1000, 39000000))
        (res, why), tol = self.run_pair({"source": "live"})
        self.assertEqual((res, why, tol), (None, "none:no_match", 17000000))


class FakeSink:
    def __init__(self, rgb=None, why=None, source=True):
        self.rgb, self.why, self.source = rgb, why, source
        self.port = 0

    def has_source(self):
        return self.source

    def want(self, *a):
        return True

    def take(self, frame_id, capture_ns, timeout_s):
        if self.rgb is None:
            return None, self.why
        return {"rgb": self.rgb, "width": WD, "height": HD, "source_key": 1, "want_key": 1}, None

    def describe(self):
        return {"fake": True}


class PresenterTests(unittest.TestCase):
    def presenter(self, d, colour, tune=None, hold_ms=kv_ar.COLOUR_HOLD_MS, cls=kv_ar.ArPresenter, extra=()):
        cmd = [base.Env.helper(), "--provider", "mock", "--display", "none", "--mock-size", "%dx%d" % CANVAS,
               "--mock-permit", "1", "--stale-ms", "3000", "--refresh-ms", "1000"] + list(extra)
        tf = None
        if tune is not None:
            tf = os.path.join(d, "tune.json")
            json.dump(tune, open(tf, "w"))
        return cls(cmd, source="live", max_age_ms=1000, out_dir=d, colour=colour,
                   colour_main=True, colour_wait_ms=5, tune_file=tf, colour_hold_ms=hold_ms)

    def snap(self, fid, stamp=11):
        s = base.synth_snap(frame_id=fid, nano_elapsed_ns=1000000)
        s["first_mono_ns"] = int(time.monotonic() * 1e9)
        s["capture_ns"], s["sensor_stamp_ns"] = 5, stamp
        return s

    def wait(self, ar, pred, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            t = ar.telem()
            if pred(t):
                return t
            time.sleep(0.02)
        raise AssertionError(ar.telem())

    def test_states_in_telem(self):
        # hold 0 = the rule before owner 10-16: an unpaired frame falls back to depth at once
        with tempfile.TemporaryDirectory() as d:
            ar = self.presenter(d, FakeSink(gradient_rgb()), hold_ms=0)
            try:
                ar.submit(self.snap(1))
                t = self.wait(ar, lambda t: t["flips"] >= 1)
                self.assertEqual((t["colour"]["state"], t["colour"]["composited"], t["colour"]["on"]),
                                 ("main", 1, True))
                ar.colour = FakeSink(None, "timeout")
                ar.submit(self.snap(2))
                t = self.wait(ar, lambda t: t["flips"] >= 2)
                self.assertEqual((t["colour"]["state"], t["colour"]["reason"]), ("depth_fallback", "timeout"))
                ar.submit(self.snap(3, stamp=None))
                t = self.wait(ar, lambda t: t["flips"] >= 3)
                self.assertEqual(t["colour"]["reason"], "no_stamp")
                self.assertEqual(t["colour"]["fallback"], {"timeout": 1, "no_stamp": 1})
                self.assertEqual(t["colour"]["unpaired"], {"timeout": 1, "no_stamp": 1})
                self.assertEqual((t["colour"]["held"], t["colour"]["held_age_ms_max"]), (0, None))
            finally:
                ar.close()
            frames = [json.loads(x) for x in open(os.path.join(d, "ar_events.jsonl")) if '"frame"' in x]
            self.assertEqual([(e["frame_id"], e["colour_mode"], e["colour"]) for e in frames],
                             [(1, 1, 1), (2, 1, 0), (3, 1, 0)])

    def hold_scenario(self, cls=kv_ar.ArPresenter, hold_ms=400):
        """paired (A) -> unpaired at once (held A) -> unpaired after > hold_ms (depth) -> off -> on,
        unpaired (no hold across an off period) -> paired (B). Returns what was observed."""
        A, B = gradient_rgb(seed=1), gradient_rgb(seed=9)
        obs = {}
        with tempfile.TemporaryDirectory() as d:
            ar = self.presenter(d, FakeSink(A), hold_ms=hold_ms, cls=cls,
                                extra=("--dump-dir", d, "--dump-seq", "3"),
                                tune={"colour_main": 1})
            try:
                ar.submit(self.snap(1))
                self.wait(ar, lambda t: t["flips"] >= 1)
                ar.colour = FakeSink(None, "timeout")
                ar.submit(self.snap(2))
                t = self.wait(ar, lambda t: t["flips"] >= 2)
                obs["f2"] = (t["colour"]["state"], t["colour"]["reason"])
                obs["f2_age"] = t["colour"]["held_age_ms_max"]
                time.sleep(hold_ms / 1e3 + 0.15)
                ar.submit(self.snap(3))
                t = self.wait(ar, lambda t: t["flips"] >= 3)
                obs["f3"] = (t["colour"]["state"], t["colour"]["reason"])
                json.dump({"colour_main": 0}, open(ar.tune_file, "w"))
                os.utime(ar.tune_file, ns=(time.time_ns() + 10 ** 9,) * 2)
                ar.colour = FakeSink(B)
                ar.submit(self.snap(4))
                self.wait(ar, lambda t: t["flips"] >= 4)
                json.dump({"colour_main": 1}, open(ar.tune_file, "w"))
                os.utime(ar.tune_file, ns=(time.time_ns() + 2 * 10 ** 9,) * 2)
                ar.colour = FakeSink(None, "none:no_match")
                ar.submit(self.snap(5))
                t = self.wait(ar, lambda t: t["flips"] >= 5)
                obs["f5"] = (t["colour"]["state"], t["colour"]["reason"])
                ar.colour = FakeSink(B)
                ar.submit(self.snap(6))
                t = self.wait(ar, lambda t: t["flips"] >= 6)
                obs["f6"] = t["colour"]["state"]
                c = t["colour"]
                obs["counts"] = (c["paired"], c["unpaired"], c["held"], c["fallback"])
            finally:
                ar.close()
            frames = [json.loads(x) for x in open(os.path.join(d, "ar_events.jsonl")) if '"frame"' in x]
            obs["composited"] = [(e["frame_id"], e["colour"]) for e in frames]
            obs["log"] = [(r["frame_id"], r["decision"], (r["held"] or {}).get("from_frame_id"))
                          for r in map(json.loads, open(os.path.join(d, "ar_colour.jsonl")))]
            r, _ = ar_dump_check.check_one(os.path.join(d, "ar_000003"))
            obs["dump"] = (r["frame_id"], r.get("colour"), r["equal"])
            rgbf = os.path.join(d, "ar_000003.rgb")
            obs["dump_rgb_is_A"] = os.path.exists(rgbf) and open(rgbf, "rb").read() == A
        return obs

    HOLD_EXPECTED = {
        "f2": ("main_held", "timeout"), "f3": ("depth_fallback", "timeout"),
        "f5": ("depth_fallback", "none:no_match"), "f6": "main",
        "counts": (2, {"timeout": 2, "none:no_match": 1}, 1, {"timeout": 1, "none:no_match": 1}),
        "composited": [(1, 1), (2, 1), (3, 0), (4, 0), (5, 0), (6, 1)],
        "log": [(1, "paired", None), (2, "held", 1), (3, "fallback", None), (5, "fallback", None),
                (6, "paired", None)],
        # the held frame's picture is checked against the colour actually reused (frame 1's)
        "dump": (2, "composited", True), "dump_rgb_is_A": True}

    def test_hold_last_paired_colour_then_depth_after_limit(self):
        obs = self.hold_scenario()
        age = obs.pop("f2_age")
        self.assertEqual(obs, self.HOLD_EXPECTED)
        self.assertTrue(0 < age < 400, age)
        self.assertEqual(kv_ar.COLOUR_HOLD_MS, 1000)            # owner 10-16: 1 s

    def test_hold_mutants_caught(self):
        class NoExpiry(kv_ar.ArPresenter):          # mutant: holds whatever the age
            def _colour_for(self, snap, tune, Wd, Hd):
                if self.colour_last is not None:
                    self.colour_last["t"] = time.monotonic()
                return kv_ar.ArPresenter._colour_for(self, snap, tune, Wd, Hd)

        class NoMemory(kv_ar.ArPresenter):          # mutant: the hold never remembers a colour
            def _colour_for(self, snap, tune, Wd, Hd):
                out = kv_ar.ArPresenter._colour_for(self, snap, tune, Wd, Hd)
                self.colour_last = None
                return out
        for cls, key in ((NoExpiry, "f3"), (NoMemory, "f2")):
            obs = self.hold_scenario(cls=cls)
            obs.pop("f2_age")
            self.assertNotEqual(obs[key], self.HOLD_EXPECTED[key], cls.__name__)

    def test_tune_off_sends_version_1_and_no_bypass_keeps_telem(self):
        with tempfile.TemporaryDirectory() as d:
            ar = self.presenter(d, FakeSink(gradient_rgb()), tune={"colour_main": 0})
            try:
                ar.submit(self.snap(1))
                t = self.wait(ar, lambda t: t["flips"] >= 1)
                self.assertEqual((t["colour"]["state"], t["colour"]["on"]), ("off", False))
            finally:
                ar.close()
            e = [json.loads(x) for x in open(os.path.join(d, "ar_events.jsonl")) if '"frame"' in x][0]
            self.assertEqual((e["colour_mode"], e["colour"]), (0, 0))
        with tempfile.TemporaryDirectory() as d:
            cmd = [base.Env.helper(), "--provider", "mock", "--display", "none", "--mock-size", "%dx%d" % CANVAS]
            ar = kv_ar.ArPresenter(cmd, source="live", out_dir=d)
            try:
                self.assertEqual(set(ar.telem()), base.TELEM_AR_FIELDS)
                self.assertFalse(hasattr(ar, "early") and ar._colour_on())
            finally:
                ar.close()
        # v1 bytes with the colour off are those of the code before the colour view
        lay = kv_ar.ar_layout(CANVAS[0], CANVAS[1], WD, HD)
        snap = base.synth_snap(frame_id=4)
        inp = kv_ar.fill_ar_inputs(snap, lay, source="live", max_age_ms=200)
        old = base.load_old("host/kv_ar.py", "56064a8")
        self.assertEqual(kv_ar.encode_ar_message(inp), old.encode_ar_message(inp))


class EndToEndTests(unittest.TestCase):
    def test_kv_server_colour_with_nano_replay(self):
        exe = base.Env.helper()
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        with tempfile.TemporaryDirectory() as t:
            out, dump = os.path.join(t, "kv"), os.path.join(t, "dump")
            srv = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--bind",
                                    "127.0.0.1", "--port", "0", "--max-idle-s", "60", "--core", "cpu",
                                    "--roi-core", "cpu", "--ar-mock", "--ar", "drm", "--ar-helper", exe,
                                    "--ar-mock-size", "%dx%d" % CANVAS, "--ar-max-age-ms", "1500",
                                    "--ar-stale-ms", "1501", "--ar-colour", "main", "--ar-colour-port", "0",
                                    "--ar-colour-wait-ms", "500",
                                    "--ar-dump-dir", dump, "--ar-dump-seq", "3,5,7", "--out", out],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
            try:
                ready = json.loads(srv.stdout.readline())
                port, cport = ready["port"], ready["ar_colour_port"]
                prep = os.path.join(t, "prep")
                os.makedirs(prep)
                n = 8
                np.stack([base.synth_codes(i) for i in range(n)]).astype("<u2").tofile(
                    os.path.join(prep, "depth_u16.bin"))
                json.dump({"width": WD, "height": HD, "frames": n, "depth_file": "depth_u16.bin",
                           "depth_unit_m": base.UNIT, "K": base.K.tolist(), "calibration_id": "synthetic",
                           "grid": {"origin_m": list(base.ORIGIN), "voxel_m": base.VOXEL, "shape_xyz": [32, 32, 32]}},
                          open(os.path.join(prep, "meta.json"), "w"))
                colours = [gradient_rgb(seed=10 + i) for i in range(n)]
                with open(os.path.join(prep, "colour_rgb8.bin"), "wb") as fp:
                    fp.write(b"".join(colours))
                json.dump({"colour_file": "colour_rgb8.bin", "width": WD, "height": HD, "frames": n},
                          open(os.path.join(prep, "colour.json"), "w"))
                snapf = os.path.join(t, "snap.json")
                tool = str(ROOT / "tools/kv_link/nano_replay.py")
                mk = subprocess.run([sys.executable, tool, "--prep", prep, "--make-roi-memory", snapf,
                                     "--roi-boxes", "12"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
                self.assertEqual(mk.returncode, 0, mk.stderr[-2000:])
                nano = os.path.join(t, "nano")
                done = subprocess.run([sys.executable, tool, "--prep", prep, "--out", nano, "--hz", "5",
                                       "--backend", "remote_pl", "--server", "127.0.0.1:%d" % port,
                                       "--chunk-bytes", "8000", "--roi-memory", snapf,
                                       "--colour-port", str(cport)],
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
                self.assertEqual(done.returncode, 0, done.stderr[-3000:])
                s = json.load(open(os.path.join(nano, "summary.json")))
                self.assertEqual(s["answered"], n)
                self.assertEqual(s["backend_counts"]["stamp_ext_sent"], n)
                self.assertEqual(s["colour_source"]["counts"].get("frames_sent"), n)
                tc = kv_link.KvTelemetryClient(("127.0.0.1", port), timeout_s=1.0)
                blk = tc.poll().blocks["ar"]
                self.assertEqual(blk["colour"]["paired"] + blk["colour"]["fallback"].get("timeout", 0), blk["sent"])
                self.assertGreater(blk["colour"]["composited"], 0)
                self.assertEqual(blk["colour"]["fallback"], {})
            finally:
                srv.send_signal(signal.SIGTERM)
                so, se = srv.communicate(timeout=30)
            self.assertEqual(srv.returncode, 0, se[-3000:])
            chk = subprocess.run([sys.executable, str(ROOT / "tools/kv_link/ar_dump_check.py"), "--dump", dump,
                                  "--json", os.path.join(t, "chk.json")], stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, env=env)
            self.assertEqual(chk.returncode, 0, chk.stdout.decode()[-3000:])
            rows = json.load(open(os.path.join(t, "chk.json")))["rows"]
            frames = [r for r in rows if r["kind"] == "frame"]
            self.assertEqual(len(frames), 3)
            for r in frames:
                self.assertEqual((r["colour"], r["pl_differing_pixels"], r["mode_name"]),
                                 ("composited", 0, "LIVE/REPLAY"))
                self.assertGreater(r["boxes"], 0)
                got = open(os.path.join(dump, "ar_%06d.rgb" % r["seq"]), "rb").read()
                self.assertEqual(got, colours[(r["frame_id"] - 1) % n])     # frame id k = source k-1
                # 2026-10-16: the depth the helper got (kv_server default --ar-fill native) is the numpy
                # reference of the source codes, bit for bit
                d = np.fromfile(os.path.join(dump, "ar_%06d.depth" % r["seq"]), dtype="<u2")
                ref = kv_ar.depth_u16_reference(base.synth_codes((r["frame_id"] - 1) % n).astype("<u2"),
                                                kv_link.f32_bits(base.UNIT), WD, HD, 1.0)
                self.assertEqual(d.tobytes(), ref.tobytes())
            ars = json.load(open(os.path.join(out, "summary.json")))["ar"]
            fill = ars["fill"]
            self.assertEqual(fill["impl"], "native")
            self.assertEqual(fill["counts"].get("native"), ars["counts"]["sent"])   # every sent frame went native
            self.assertGreaterEqual(fill["counts"].get("native", 0), len(frames))
            self.assertEqual(len(fill["source_sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
