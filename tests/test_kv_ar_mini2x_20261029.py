"""Screen mini map 2x in the colour main view (owner 10-29).

What is checked (known answers, the numpy reference, positive controls):
  * layout: ar_colour_layout(1024, 600, 640, 480, mini_scale=2) puts the 2x box (260 x 282) at (718, 292)
    (mini_x/y 719/304), disjoint from the colour picture (with the label-bar spill), the PiP and the panel,
    inside 1024 x 600 (independent painting); mini_scale=1 is the layout before; a canvas where 2x does not
    fit falls back to the 1x layout; a tune override that would overlap at 2x is rejected;
  * message: MSG_COLOUR flags b1 is set only for a 2x layout, the bytes are unchanged otherwise;
  * reference: colour_compose.scale_mini on a known box (each source pixel -> a 2 x 2 block, nothing of the
    1x box left outside the copy);
  * helper (host build, mock provider, 1024 x 600): --mini-scale 2 enlarges the mini map on a fresh composited
    frame, a colour-on frame with no colour, a NODATA frame and the fail-safe STALE picture; never on a
    version-1 frame or a frame without the flag; ar_dump_check equal for all (and the C reference equals
    the PL copy); --mini-scale 1 with the same messages: no "mini" in the meta, no enlargement;
  * mutant mini_row is caught by the checker, with the PL part still equal and the differences only
    inside the 2x box.
Run: PYTHONPATH=. python3 tests/test_kv_ar_mini2x_20261029.py
"""
import json
import os
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "tools" / "ar_display"))
sys.path.insert(0, str(ROOT / "tools" / "kv_link"))

from host import kv_ar  # noqa: E402
import ar_dump_check  # noqa: E402
import colour_compose  # noqa: E402
import test_kv_ar as base  # noqa: E402
import test_kv_ar_colour as tc  # noqa: E402

WD, HD = base.WD, base.HD
SCREEN = (1024, 600)


def box2x(lay):
    s = lay.get("mini_scale", 1)
    x0, y0 = lay["mini_x"] - 1, lay["mini_y"] - 12
    return x0, y0, x0 + 130 * s, y0 + 141 * s


def msg(frame_id, lay, rgb=b"", version2=True, zero_depth=False):
    snap = base.synth_snap(frame_id=frame_id, nano_elapsed_ns=2000000)
    inp = kv_ar.fill_ar_inputs(snap, lay, source="live", max_age_ms=1000)
    inp["first_mono_ns"] = int(time.monotonic() * 1e9)
    if zero_depth:
        inp["depth"] = np.zeros_like(np.asarray(inp["depth"]))
    if version2:
        inp["colour"] = {"pip": (lay["pip_x"], lay["pip_y"], lay["pip_div"]), "rgb": rgb or None,
                         "width": WD, "height": HD}
        if lay.get("mini_scale", 1) != 1:
            inp["colour"]["mini_scale"] = lay["mini_scale"]
    return kv_ar.encode_ar_message(inp)


class LayoutTests(unittest.TestCase):
    def test_known_layout_and_disjoint(self):
        one = kv_ar.ar_colour_layout(1024, 600, 640, 480)
        self.assertEqual(one, kv_ar.ar_colour_layout(1024, 600, 640, 480, mini_scale=1))
        self.assertEqual((one["mini_x"], one["mini_y"]), (784, 309))
        self.assertNotIn("mini_scale", one)
        lay = kv_ar.ar_colour_layout(1024, 600, 640, 480, mini_scale=2)
        self.assertEqual((lay["mini_x"], lay["mini_y"], lay["mini_scale"]), (719, 304, 2))
        self.assertEqual(box2x(lay), (718, 292, 978, 574))
        self.assertIsNone(kv_ar.colour_layout_problem(lay, 640, 480))
        # independent painting: colour picture + spill, PiP, panel, 2x box never share a pixel
        W, H = SCREEN
        c = np.zeros((H, W), np.int32)
        c[:40] += 1
        dx, dy = lay["depth_dx"], lay["depth_dy"]
        c[dy - 12:dy + 480, dx:dx + 640 + 48] += 1
        c[lay["pip_y"]:lay["pip_y"] + 240, lay["pip_x"]:lay["pip_x"] + 320] += 1
        x0, y0, x1, y1 = box2x(lay)
        self.assertTrue(0 <= x0 and x1 <= W and 40 <= y0 and y1 <= H)
        c[y0:y1, x0:x1] += 1
        self.assertEqual(int(c.max()), 1)
        # 640x360 too
        lay3 = kv_ar.ar_colour_layout(1024, 600, 640, 360, mini_scale=2)
        self.assertEqual(lay3["mini_scale"], 2)
        self.assertIsNone(kv_ar.colour_layout_problem(lay3, 640, 360))

    def test_fallback_and_override(self):
        small = kv_ar.ar_colour_layout(tc.CANVAS[0], tc.CANVAS[1], WD, HD, mini_scale=2)
        self.assertEqual(small, kv_ar.ar_colour_layout(tc.CANVAS[0], tc.CANVAS[1], WD, HD))
        lay = kv_ar.ar_colour_layout(1024, 600, 640, 480, mini_scale=2)
        # a mini position that is fine at 1x but whose 2x box leaves the screen / hits the PiP
        self.assertEqual(kv_ar.colour_layout_problem(dict(lay, mini_y=340), 640, 480), "mini_outside")
        self.assertEqual(kv_ar.colour_layout_problem(dict(lay, mini_y=290), 640, 480), "pip_overlaps_mini")
        self.assertIsNone(kv_ar.colour_layout_problem(dict(lay, mini_y=340, mini_scale=1), 640, 480))
        with self.assertRaises(ValueError):
            kv_ar.ar_colour_layout(1024, 600, 640, 480, mini_scale=3)

    def test_message_flag(self):
        lay1 = kv_ar.ar_colour_layout(*SCREEN, WD, HD)
        lay2 = kv_ar.ar_colour_layout(*SCREEN, WD, HD, mini_scale=2)
        self.assertEqual(lay2["mini_scale"], 2)
        off = kv_ar.MSG_HEAD.size + kv_ar.MSG_FIXED.size + 96 + 2 * WD * HD + kv_ar.GRID_BYTES + 4 * kv_ar.DESC_U32
        for lay, want in ((lay1, 1), (lay2, 3)):
            m = msg(5, lay, tc.gradient_rgb())
            self.assertEqual(struct.unpack_from("<I", m, off)[0], want)


class ReferenceTests(unittest.TestCase):
    def test_scale_mini_known_box(self):
        W, H = 400, 360
        regs = [0] * 24
        regs[0], regs[1], regs[22], regs[23] = W, H, 11, 22
        fb = np.full((H, W), 7, np.uint32)
        box = (np.arange(141 * 130, dtype=np.uint32) + 100).reshape(141, 130)
        fb[10:151, 10:140] = box
        self.assertTrue(colour_compose.mini_fits(regs, 2))
        out = colour_compose.scale_mini(fb, regs, 2)
        self.assertTrue(np.array_equal(out[10:292, 10:270], np.kron(box, np.ones((2, 2), np.uint32))))
        rest = np.ones((H, W), bool)
        rest[10:292, 10:270] = False
        self.assertTrue((out[rest] == 7).all())
        mut = colour_compose.scale_mini(fb, regs, 2, mutant="mini_row")
        self.assertFalse(np.array_equal(mut, out))
        self.assertTrue(np.array_equal(mut[10:292:2], out[10:292:2]))       # even rows untouched
        regs[23] = H - 100
        self.assertFalse(colour_compose.mini_fits(regs, 2))


class HelperTests(unittest.TestCase):
    def run_helper(self, d, scale, mutant=None, lay=None):
        lay = lay or kv_ar.ar_colour_layout(*SCREEN, WD, HD, mini_scale=2)
        extra = ["--dump-seq", "2,3,4,5,6", "--dump-failsafe", "1"]
        if scale != 1:
            extra += ["--mini-scale", str(scale)]
        if mutant:
            extra += ["--colour-mutant", mutant]
        p, ev = tc.start_helper(d, *extra, size=SCREEN, stale_ms=1200)
        try:
            sends = [(41, dict(rgb=tc.gradient_rgb())),           # fresh, composited
                     (42, dict(rgb=b"")),                          # colour on, none paired
                     (43, dict(version2=False)),                   # v1
                     (44, dict(rgb=tc.gradient_rgb(seed=2), zero_depth=True)),   # NODATA
                     (45, dict(rgb=tc.gradient_rgb(seed=3)))]
            evs = {}
            for fid, kw in sends:
                os.write(p.stdin.fileno(), msg(fid, lay, **kw))
                evs[fid] = ev.wait(lambda e, fid=fid: e["ev"] == "frame" and e["frame_id"] == fid)
            evs["fs"] = ev.wait(lambda e: e["ev"] == "failsafe", timeout=10)
        finally:
            tc.stop_helper(p)
        rows = {}
        for f in sorted(os.listdir(d)):
            if f.endswith(".json"):
                r, dd = ar_dump_check.check_one(os.path.join(d, f[:-5]), base.Env.c_ref())
                rows[r["seq"]] = (r, dd)
        return lay, evs, rows

    def test_scale2_every_flipped_kind(self):
        with tempfile.TemporaryDirectory() as d:
            lay, evs, rows = self.run_helper(d, 2)
        self.assertEqual(sorted(rows), [2, 3, 4, 5, 6, 7])
        self.assertEqual([evs[k]["mini"] for k in (41, 42, 43, 44, 45, "fs")], [1, 1, 0, 1, 1, 1])
        self.assertEqual(evs[44]["mode_name"], "NODATA")
        self.assertEqual(evs["fs"]["mode_name"], "STALE")
        for seq, (r, dd) in rows.items():
            self.assertTrue(r["equal"], r)
            self.assertTrue(r["c_ref_equal_fb"], r)
            self.assertTrue(r["mini_rule_ok"], r)
        self.assertEqual([rows[s][0]["mini_applied"] for s in (2, 3, 4, 5, 6, 7)], [1, 1, 0, 1, 1, 1])
        self.assertEqual(rows[2][0]["colour"], "composited")
        self.assertEqual(rows[5][0]["colour"], "composited")       # NODATA composed too
        self.assertEqual(rows[3][0]["colour"], "mode_on_not_composited")
        self.assertEqual(rows[7][0]["kind"], "failsafe")
        # what the screen shows: the 2x box equals the PL's 1x box enlarged, nothing of 1x outside
        x0, y0, x1, y1 = box2x(lay)
        for s in (2, 3, 5, 6, 7):
            fb, pl = rows[s][1]["fb"], rows[s][1]["fbpl"]
            src = pl[y0:y0 + 141, x0:x0 + 130]
            self.assertTrue(np.array_equal(fb[y0:y1, x0:x1], np.kron(src, np.ones((2, 2), np.uint32))), s)
            self.assertGreater(int((fb[y0:y1, x0:x1] != pl[y0:y1, x0:x1]).sum()), 1000, s)
        # the v1 frame (seq 4): picture as the PL wrote it, no .fbpl
        self.assertNotIn("fbpl", rows[4][1])

    def test_scale1_unchanged(self):
        with tempfile.TemporaryDirectory() as d:
            _, evs, rows = self.run_helper(d, 1)
        self.assertTrue(all("mini" not in e for e in evs.values()))
        for seq, (r, dd) in rows.items():
            self.assertTrue(r["equal"], r)
            self.assertNotIn("mini", dd["meta"])
            self.assertNotIn("mini_rule_ok", r)
        self.assertNotIn("fbpl", rows[3][1])          # colour on, none paired: not touched

    def test_unflagged_frame_not_scaled(self):
        lay1 = kv_ar.ar_colour_layout(*SCREEN, WD, HD)     # 1x layout: no flag in the message
        with tempfile.TemporaryDirectory() as d:
            _, evs, rows = self.run_helper(d, 2, lay=lay1)
        self.assertTrue(all(e["mini"] == 0 for e in evs.values()))
        for seq, (r, dd) in rows.items():
            self.assertTrue(r["equal"], r)
            self.assertEqual((r["mini_applied"], r["mini_expected"]), (0, 0))

    def test_mutant_caught(self):
        with tempfile.TemporaryDirectory() as d:
            lay, evs, rows = self.run_helper(d, 2, mutant="mini_row")
        x0, y0, x1, y1 = box2x(lay)
        for s in (2, 3, 7):
            r, dd = rows[s]
            self.assertFalse(r["equal"], r)
            self.assertEqual(r["pl_differing_pixels"], 0, r)
            want = colour_compose.scale_mini(
                colour_compose.compose(dd["fbpl"], dd["meta"]["regs"], dd["desc"], dd["rgb"],
                                       dd["meta"]["colour"]["pip"]) if "rgb" in dd else dd["fbpl"],
                dd["meta"]["regs"], 2)
            diff = np.argwhere(dd["fb"] != want)
            self.assertGreater(len(diff), 0)
            self.assertTrue(all(y0 <= y < y1 and x0 <= x < x1 for y, x in diff), s)
        self.assertTrue(rows[4][0]["equal"])          # v1 frame untouched by the mutant


if __name__ == "__main__":
    unittest.main()
