"""AR screen bring-up 2026-10-15: display-only box choice (host/kv_ar.select_display_boxes).

Owner 10-15 (via controller, option B of UI_LOOK_DISCUSSION_PACK A4): at most 5 boxes on the AR
screen, same-class 2D IoU > 0.5 hidden, order = the ROI request order (O11 priority). Display only:
the ROI core still checks every row, so the snapshot's ROI rows are not touched here.

Checked with known answers:
  * IoU of closed-interval pixel boxes; NMS hides only same-class boxes strictly above the
    threshold, keeps the earlier (higher priority) box; cap keeps request order;
  * old behaviour switch: box_max=32, box_nms=None gives exactly the pre-change desc (all valid rows);
  * fill_ar_inputs with the owner defaults on 7 rows (two same-class overlaps, one cross-class
    overlap) -> the expected 5 boxes in the expected order, hidden_boxes counted; the golden renders
    exactly those boxes (pixels of a hidden box's own label strip differ from the all-boxes render);
  * the snapshot (ROI rows the core checked) is unchanged by the display choice;
  * positive controls: NMS ignoring the class, and IoU >= instead of >, are both caught.

Run: PYTHONHASHSEED=0 PYTHONPATH=. python3 -m unittest discover -s tests -p test_kv_ar_display_boxes.py
(also /usr/bin/python3.8).
"""
import copy
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from host import kv_ar  # noqa: E402
import test_kv_ar as base  # noqa: E402

# (row_id, en, x0, y0, x1, y1), verdict, label. Depth image is 96x64 (base.WD x base.HD).
ROWS = [
    ((10, 1, 10, 10, 29, 29), 0, "cup"),       # A  kept
    ((11, 1, 12, 12, 31, 31), 2, "cup"),       # B  IoU(A,B)=324/476=0.68 -> hidden (same class)
    ((12, 1, 12, 12, 31, 31), 3, "bottle"),    # C  same box as B but other class -> kept
    ((13, 1, 50, 10, 69, 29), 0, "book"),      # D  kept
    ((14, 1, 60, 10, 79, 29), 0, "book"),      # E  IoU(D,E)=200/600=0.33 -> kept
    ((15, 1, 40, 40, 59, 59), 2, "chair"),     # F  kept (5th)
    ((16, 1, 70, 40, 89, 59), 3, "laptop"),    # G  6th valid -> over the cap
]


def snap_with(rows=ROWS):
    s = base.synth_snap()
    s["roi"] = {"rows": [r for r, _, _ in rows]}
    s["roi_rows"] = [(r[0], r[1], 9, 9, 0, 0, v) for r, v, _ in rows]
    s["labels"] = [n for _, _, n in rows]
    return s


def boxes_of(inp):
    d = inp["desc"]
    return [tuple(int(v) for v in d[2 + 6 * i: 8 + 6 * i]) for i in range(int(d[1]))]


class SelectTests(unittest.TestCase):
    def test_iou_known_answers(self):
        self.assertAlmostEqual(kv_ar._box_iou((10, 10, 29, 29), (12, 12, 31, 31)), 324 / 476)
        self.assertAlmostEqual(kv_ar._box_iou((50, 10, 69, 29), (60, 10, 79, 29)), 200 / 600)
        self.assertEqual(kv_ar._box_iou((0, 0, 9, 9), (10, 0, 19, 9)), 0.0)       # touching, closed
        self.assertAlmostEqual(kv_ar._box_iou((0, 0, 9, 9), (9, 0, 18, 9)), 10 / 190)

    def test_nms_same_class_strictly_above(self):
        a = (0, 0, 9, 9, 0, 1, "cup")
        half = (0, 0, 9, 19, 0, 2, "cup")          # IoU exactly 100/200 = 0.5 -> kept (> only)
        kept, hidden = kv_ar.select_display_boxes([a, half], 5, 0.5)
        self.assertEqual((kept, hidden), ([a, half], 0))
        dup = (1, 0, 10, 9, 0, 3, "cup")           # IoU 90/110 -> hidden
        kept, hidden = kv_ar.select_display_boxes([a, dup], 5, 0.5)
        self.assertEqual((kept, hidden), ([a], 1))
        other = (1, 0, 10, 9, 0, 3, "book")
        kept, hidden = kv_ar.select_display_boxes([a, other], 5, 0.5)
        self.assertEqual((kept, hidden), ([a, other], 0))

    def test_cap_keeps_request_order_and_off_switch(self):
        cands = [(i, 0, i, 0, 0, i, "c%d" % i) for i in range(40)]
        kept, hidden = kv_ar.select_display_boxes(cands, 5, 0.5)
        self.assertEqual((kept, hidden), (cands[:5], 35))
        kept, hidden = kv_ar.select_display_boxes(cands[:32], 32, None)
        self.assertEqual((kept, hidden), (cands[:32], 0))


class FillTests(unittest.TestCase):
    def test_owner_defaults_choose_five_in_o11_order(self):
        snap = snap_with()
        before = copy.deepcopy(snap["roi"]), list(snap["roi_rows"])
        inp = kv_ar.fill_ar_inputs(snap, base.layout(), source="live", box_max=5, box_nms=0.5)
        got = [(b[0], b[1], b[2], b[3], b[4]) for b in boxes_of(inp)]
        want = [(r[2], r[3], r[4], r[5], v) for i, (r, v, _) in enumerate(ROWS) if i in (0, 2, 3, 4, 5)]
        self.assertEqual(got, want)
        self.assertEqual((inp["boxes"], inp["hidden_boxes"], inp["skipped_boxes"]), (5, 2, 0))
        self.assertEqual([kv_ar.ar_label(r[0], n) for (r, _, n) in ROWS[:1]], [boxes_of(inp)[0][5]])
        self.assertEqual((snap["roi"], snap["roi_rows"]), before)     # display only

    def test_old_behaviour_switch_equals_all_rows(self):
        snap = snap_with()
        old = kv_ar.fill_ar_inputs(snap, base.layout(), source="live")       # module defaults = old
        sw = kv_ar.fill_ar_inputs(snap, base.layout(), source="live", box_max=32, box_nms=None)
        self.assertEqual(boxes_of(old), boxes_of(sw))
        self.assertEqual(len(boxes_of(old)), len(ROWS))
        self.assertEqual(old["hidden_boxes"], 0)
        want = [(r[2], r[3], r[4], r[5], v, kv_ar.ar_label(r[0], n)) for r, v, n in ROWS]
        self.assertEqual(boxes_of(old), want)

    def test_golden_draws_only_the_chosen_boxes(self):
        snap = snap_with()
        lay = base.layout()
        new = kv_ar.fill_ar_inputs(snap, lay, source="live", box_max=5, box_nms=0.5)
        old = kv_ar.fill_ar_inputs(snap, lay, source="live")
        st_n, fb_n = base.render(new)[:2]
        st_o, fb_o = base.render(old)[:2]
        self.assertEqual((st_n, st_o), (0, 0))
        diff = np.argwhere(fb_n != fb_o)
        self.assertGreater(len(diff), 0)
        # every differing pixel lies on a hidden box (B or G: frame + its label strip above it)
        dx, dy = lay["depth_dx"], lay["depth_dy"]
        hidden = [ROWS[1][0], ROWS[6][0]]
        for y, x in diff:
            self.assertTrue(any(r[2] + dx - 2 <= x <= max(r[4], r[2] + 48) + dx + 2 and
                                r[3] + dy - 16 <= y <= r[5] + dy + 2 for r in hidden), (y, x))

    def test_positive_controls(self):
        snap = snap_with()
        src = open(kv_ar.__file__).read()
        mutants = {
            "class_blind": ("k[6] == c[6] and ", ""),
            "iou_ge": ("_box_iou(k, c) > nms_iou", "_box_iou(k, c) >= nms_iou"),
        }
        for name, (old, new) in mutants.items():
            with self.subTest(mutant=name):
                self.assertEqual(src.count(old), 1, name)
                mod = type(sys)("kv_ar_" + name)
                mod.__file__ = kv_ar.__file__
                exec(compile(src.replace(old, new), kv_ar.__file__, "exec"), mod.__dict__)
                if name == "class_blind":
                    inp = mod.fill_ar_inputs(snap, base.layout(), source="live", box_max=5, box_nms=0.5)
                    good = kv_ar.fill_ar_inputs(snap, base.layout(), source="live", box_max=5, box_nms=0.5)
                    self.assertNotEqual(boxes_of(inp), boxes_of(good))
                else:
                    a, half = (0, 0, 9, 9, 0, 1, "cup"), (0, 0, 9, 19, 0, 2, "cup")
                    self.assertNotEqual(mod.select_display_boxes([a, half], 5, 0.5),
                                        kv_ar.select_display_boxes([a, half], 5, 0.5))


class StretchTests(unittest.TestCase):
    """Proposed display-only colour stretch (default off): f(z) = zmin + (z - zmin) * k."""

    def test_known_answers(self):
        d = np.array([0, 50, 134, 200, 375, 1000, 2000, 9999, 10000, 65535], np.uint16)
        got = kv_ar.stretch_depth_mm(d, 6.0)
        want = [0, 50, 134, 134 + 66 * 6, 134 + 241 * 6, 134 + 866 * 6, 9999, 9999, 10000, 65535]
        self.assertEqual(got.tolist(), want)
        self.assertEqual(got.dtype, np.uint16)
        self.assertEqual(kv_ar.stretch_depth_mm(d, 1.0).tolist(), d.tolist())
        self.assertEqual(kv_ar.stretch_depth_mm(np.zeros(4, np.uint16), 6.0).tolist(), [0] * 4)

    def test_monotone_and_nearest_kept(self):
        rng = np.random.default_rng(3)
        d = rng.integers(0, 12000, 5000).astype(np.uint16)
        v = (d >= 100) & (d <= 9999)
        got = kv_ar.stretch_depth_mm(d, 4.5)
        order = np.argsort(d[v], kind="stable")
        self.assertTrue(np.all(np.diff(got[v][order].astype(np.int64)) >= 0))
        self.assertEqual(int(got[v].min()), int(d[v].min()))
        self.assertTrue(np.all(got[~v] == d[~v]))

    def test_panel_nearest_z_unchanged_and_colours_change(self):
        snap = snap_with()
        lay = base.layout()
        off = kv_ar.fill_ar_inputs(snap, lay, source="live")
        on = kv_ar.fill_ar_inputs(snap, lay, source="live", depth_stretch=6.0)
        st0, fb0, info0 = base.render(off)
        st1, fb1, info1 = base.render(on)
        self.assertEqual((st0, st1), (0, 0))
        self.assertEqual(info0["panel"], info1["panel"])
        self.assertTrue(np.any(fb0 != fb1))
        # positive control: stretching from 0 instead of zmin (f(z) = z * k) moves the nearest-Z reading
        d = off["depth"].astype(np.int64)
        v = (d >= 100) & (d <= 9999)
        bad = dict(off)
        dd = d.copy()
        dd[v] = np.minimum(d[v] * 6, 9999)
        bad["depth"] = dd.astype("<u2")
        self.assertNotEqual(base.render(bad)[2]["panel"][0], info0["panel"][0])


if __name__ == "__main__":
    unittest.main()
