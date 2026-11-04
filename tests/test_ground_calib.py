"""Line E, live car (2026-10-15): tools/car_run/ground_calib.py -- start-up ground self-calibration of the camera
mount -- on synthetic depth (no ROS): tools/car_geo_producer/mount_sweep.render_floor at a known mount, Gaussian
depth noise sigma0 * (1 + z^2) per frame, quantised to 16UC1 mm like the live topic, optional boxes.

Criteria (written before any run, plan cheeky-prancing-mochi + brief 10-15; not loosened afterwards):
  height 0.33-0.38 m, pitch 20-33 deg, roll +-3 deg: fit error <= 0.3 deg and <= 5 mm;
  configuration off by 3 deg or 2 cm -> corrected back to the truth; off by 6 deg -> refused;
  negative controls refused: all wall, ground mostly covered by a box, all-zero depth.
  (Kernel selection, controller 10-15: also the residual-bend gate <= 1 mm; the unoccluded range must still be
  adopted in full -- no false refusal from the added gates.)
  Wide obstacle (hard acceptance item for the plane-fit kernel; written 2026-10-15 20:56:35 CST by the controller,
  source: measured on the placeholder kernel, which adopted 0.44-0.68 deg / 3-4 mm off with a 2 m wide wall at
  0.9-1.2 m): a box 0.3 m deep, 0.6 m tall (above the camera), width 0.2 / 0.5 / 1.0 / 2.0 m, centred, its front face
  0.5 / 0.7 / 0.9 / 1.2 / 1.6 m ahead of the camera (horizontal), camera 0.36 m / 27 deg: every cell must be either
  refused, or adopted with an error <= 0.3 deg and <= 5 mm. One "adopted but wrong" cell fails the item.
Also: the roll sign convention (common.base_camera) pinned by render -> fit -> the same signed roll; roll 0 = the
transform before, byte for byte. Python 3.8 compatible."""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.car_geo_producer.common import LEVEL, MOUNT_XYZ, base_camera  # noqa: E402
from tools.car_geo_producer.mount_sweep import gemini2_k, render_floor  # noqa: E402
from tools.car_run import ground_calib as gc  # noqa: E402
from tools.car_run.car_mount import mount  # noqa: E402

K, W, H = gemini2_k()
TOL_DEG, TOL_M = 0.3, 0.005          # the fit criterion
MAX_ERR = {"height_m": 0.0, "pitch_deg": 0.0, "roll_deg": 0.0}   # largest error seen (printed at the end)
WIDE_DIST_M = (0.5, 0.7, 0.9, 1.2, 1.6)      # wide-obstacle item (see the header): front face ahead of the camera
WIDE_WIDTH_M = (0.2, 0.5, 1.0, 2.0)
WIDE_CELLS = []                              # (dist, width, verdict, errors) of the last sweep (printed at the end)
BLUR_CELLS = []                              # the same with 9 px blurred edges (additional)


def box_blur(depth_m, k):
    """k x k box blur of a depth image (NaN = 0 = none stays none): a stereo matching window across edges."""
    t = np.nan_to_num(depth_m.astype(np.float64))
    p = np.pad(t, k // 2, mode="edge")
    c = np.cumsum(np.cumsum(np.pad(p, ((1, 0), (1, 0))), 0), 1)
    out = (c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]) / (k * k)
    out[~np.isfinite(depth_m)] = np.nan
    return out.astype(np.float32)


def render(height_m, pitch_deg, roll_deg=0.0, boxes=()):
    """optical-z depth (m, NaN = none) of the floor + boxes for a camera at MOUNT_XYZ x/y, the given mount."""
    T = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], height_m), pitch_deg, roll_deg)
    d = render_floor(K, W, H, T)
    for b in boxes:
        db = render_floor(K, W, H, T, b)
        d = np.where(np.isfinite(db) & ~(db >= d), db, d)
    return d


def frames(depth_m, sigma0=0.002, seed=1, n=gc.FRAMES):
    """n noisy 16UC1 (mm) frames of one depth image: sigma = sigma0 * (1 + z^2)."""
    rng = np.random.RandomState(seed)
    d = depth_m.astype(np.float64)
    sig = sigma0 * (1.0 + np.nan_to_num(d) ** 2)
    out = []
    for _ in range(n):
        z = d + rng.normal(size=d.shape) * sig
        out.append(np.where(np.isfinite(z) & (z > 0), np.round(z * 1000.0), 0).clip(0, 65535).astype(np.uint16))
    return out


def errors(res, h, p, r):
    f = res["fitted"]
    return {"height_m": abs(f["height_m"] - h), "pitch_deg": abs(f["pitch_deg"] - p), "roll_deg": abs(f["roll_deg"] - r)}


def note(e):
    for k in MAX_ERR:
        MAX_ERR[k] = max(MAX_ERR[k], e[k])


def tearDownModule():
    sys.stderr.write("\n[ground_calib synthetic] largest fit error: height %.3f mm pitch %.4f deg roll %.4f deg\n"
                     % (1000 * MAX_ERR["height_m"], MAX_ERR["pitch_deg"], MAX_ERR["roll_deg"]))
    for tag, cells in (("", WIDE_CELLS), (", 9 px blur", BLUR_CELLS)):
        for dist, width, verdict, e in cells:
            sys.stderr.write("[ground_calib wide obstacle%s] %.1f m ahead, %.1f m wide: %s%s\n" % (
                tag, dist, width, verdict, "" if e is None else " (error %.2f mm / %.3f deg / %.3f deg)"
                % (1000 * e["height_m"], e["pitch_deg"], e["roll_deg"])))


class TestRoll(unittest.TestCase):
    """common.base_camera / car_mount.mount roll_deg: 0 = before (bitwise); the sign convention pinned"""

    def test_roll_zero_bitwise_unchanged(self):
        for p in (0.0, 20.0, 27.0, 30.0, 33.0):
            c, s = math.cos(math.radians(p)), math.sin(math.radians(p))
            old = np.eye(4)
            old[:3, :3] = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]]) @ LEVEL   # the expression before 10-15
            old[:3, 3] = (0.1, 0.0, 0.36)
            for T in (base_camera((0.1, 0.0, 0.36), p), base_camera((0.1, 0.0, 0.36), p, 0.0),
                      base_camera((0.1, 0.0, 0.36), p, roll_deg=0)):
                self.assertEqual(T.tobytes(), old.tobytes(), p)
        for h, p in ((0.36, 30.0), (0.33, 27.0), (0.38, 20.0)):
            self.assertEqual(mount(h, p), mount(h, p, roll_deg=0.0))
            self.assertEqual(mount(h, p), mount(h, p, None, 0.0, 0.0))

    def test_roll_sign_convention(self):
        # roll > 0: the camera's right side (optical +x) goes DOWN; the optical axis (pitch) does not move
        for p in (20.0, 27.0):
            R0 = base_camera((0, 0, 0.36), p)[:3, :3]
            R = base_camera((0, 0, 0.36), p, 5.0)[:3, :3]
            self.assertLess(R[2, 0], -0.05)                             # optical +x has a downward base z component
            self.assertGreater(base_camera((0, 0, 0.36), p, -5.0)[2, 0], 0.05)
            np.testing.assert_allclose(R[:, 2], R0[:, 2], atol=1e-12)   # same optical axis
        # render -> fit -> the same signed roll (pins the convention AND the inversion in pose_from_plane)
        for r in (3.0, -3.0, 2.0):
            res = gc.calibrate(frames(render(0.36, 27.0, r), sigma0=0.0), K, 0.36, 27.0)
            self.assertEqual(res["verdict"], "adopt", res["reasons"])
            self.assertAlmostEqual(res["fitted"]["roll_deg"], r, delta=0.05)
            self.assertEqual(math.copysign(1, res["adopted"]["roll_deg"]), math.copysign(1, r))
        # the adopted TF carries the roll: its rotation is base_camera's with that roll
        hc, xyz, q = mount(0.36, 27.0, None, roll_deg=3.0)
        x, y, z, w = q
        Rq = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(Rq, base_camera((0.1, 0.0, 0.36), 27.0, 3.0)[:3, :3], atol=1e-9)
        self.assertRaises(ValueError, mount, 0.36, 27.0, None, 0.0, 50.0)


class TestPure(unittest.TestCase):
    def test_depth_conversion_and_median(self):
        d = gc.depth_mm_to_m(np.array([[0, 1000], [360, 65535]], dtype=np.uint16))
        self.assertTrue(math.isnan(d[0, 0]))
        self.assertEqual(d.dtype, np.float32)
        np.testing.assert_allclose(d[0, 1], 1.0)
        np.testing.assert_allclose(d[1, 0], 0.36)
        fr = [np.full((2, 2), np.nan, np.float32) for _ in range(10)]
        for i in range(10):
            fr[i][0, 0] = 1.0 + i            # always valid: median 5.5
            if i < 4:
                fr[i][0, 1] = 2.0            # valid in 4 of 10: below half -> NaN
            if i < 5:
                fr[i][1, 0] = 3.0            # valid in 5 of 10: kept
        m = gc.median_depth(fr)
        self.assertAlmostEqual(float(m[0, 0]), 5.5)
        self.assertTrue(math.isnan(m[0, 1]))
        self.assertAlmostEqual(float(m[1, 0]), 3.0)
        self.assertTrue(math.isnan(m[1, 1]))

    def test_pose_from_plane_known_answers(self):
        for p, r in ((27.0, 0.0), (20.0, 3.0), (33.0, -3.0), (30.0, 10.0)):
            h, pp, rr = gc.pose_from_plane(gc.up_normal(p, r), 0.36)
            self.assertAlmostEqual(h, 0.36)
            self.assertAlmostEqual(pp, p, places=9)
            self.assertAlmostEqual(rr, r, places=9)

    def test_predicted_ground_counts_have_margin(self):
        # noise-free, unoccluded: the predicted ground samples at the mount extremes, against MIN_INLIERS 3000
        u, v = gc.sample_pixels(W, H)
        for h, p in ((0.33, 20.0), (0.36, 27.0), (0.38, 33.0)):
            n = int(gc.predicted_ground(K, u, v, h, p).sum())
            self.assertGreater(n, 2 * gc.MIN_INLIERS, (h, p, n))

    def test_gate_constants_as_written(self):
        self.assertEqual((gc.FRAMES, gc.STRIDE, gc.RANGE_M, gc.PRIOR_MAX_DEG), (10, 4, (0.3, 2.0), 10.0))
        self.assertEqual((gc.MIN_INLIERS, gc.MIN_INLIER_SHARE, gc.MAX_RMS_M), (3000, 0.5, 0.015))
        self.assertEqual((gc.HALF_MAX_DEG, gc.HALF_MAX_M), (0.3, 0.005))
        self.assertEqual(gc.AUTO_MAX, {"height_m": 0.03})   # owner 10-28: no angle limit against the configuration
        self.assertEqual(gc.CHECK_MAX, {"pitch_deg": 1.0, "height_m": 0.01, "roll_deg": 1.0})
        # upper height 0.39 -> 0.395: owner 10-17 (camera re-tilted, fit 0.3915 m; TestHardHeight395)
        self.assertEqual((gc.HARD_HEIGHT_M, gc.HARD_PITCH_MAX_DEG), ((0.31, 0.395), 35.0))
        # kernel selection (controller 10-15); sigma(z) and the bend limit are estimates (runbook: re-run on the car)
        from tools.car_run import ground_plane_kernel as k
        self.assertEqual(gc.BEND_MAX_M, 0.001)
        self.assertEqual((k.HYP_MAX_DEG, k.HYP_MAX_DH_M, k.MSAC_T, k.SIGMA_K, k.LOCAL_NORMAL_MAX_DEG, k.IRLS_ROUNDS,
                          k.TUKEY_C), (8.0, 0.05, 0.01, 1.5e-3, 10.0, 5, 4.685))
        self.assertIn("Copyright (c) 2018-2026 www.open3d.org", open(k.__file__).read())   # MIT notice kept

    def test_fit_plane_interface(self):
        rng = np.random.RandomState(0)
        P = np.c_[rng.uniform(-1, 1, 500), rng.uniform(-1, 1, 500), np.zeros(500)] + [0, 0, -0.3]
        n, d, inl = gc.fit_plane(P, (0, 0, 1))
        np.testing.assert_allclose(n, (0, 0, 1), atol=1e-9)
        self.assertAlmostEqual(d, 0.3)
        self.assertEqual(inl.dtype, bool)
        self.assertEqual(int(inl.sum()), 500)
        n, d, inl = gc.fit_plane(P, (0, 0, -1))           # oriented to the prior
        np.testing.assert_allclose(n, (0, 0, -1), atol=1e-9)
        n, d, inl = gc.fit_plane(P, (1, 0, 0))            # 90 deg from the prior: no plane
        self.assertIsNone(n)
        self.assertEqual(int(inl.sum()), 0)
        self.assertIsNone(gc.fit_plane(P[:2], (0, 0, 1))[0])


class TestSynthetic(unittest.TestCase):
    """the written criteria on rendered depth (noise, 1 mm quantisation, a box)"""

    def test_mount_range_fit_error(self):
        box = ((0.9, -0.10, 0.0), (1.1, 0.20, 0.15))      # mount_sweep's box, a little off-centre
        k = 0
        for h in (0.33, 0.355, 0.38):
            for p in (20.0, 27.0, 33.0):
                for r in (-3.0, 0.0, 3.0):
                    k += 1
                    sigma0 = (0.001, 0.002, 0.003)[k % 3]
                    d = render(h, p, r, boxes=(box,) if k % 2 else ())
                    res = gc.calibrate(frames(d, sigma0, seed=k), K, h, p)
                    self.assertEqual(res["verdict"], "adopt", (h, p, r, res["reasons"]))
                    e = errors(res, h, p, r)
                    note(e)
                    self.assertLessEqual(e["height_m"], TOL_M, (h, p, r, e))
                    self.assertLessEqual(e["pitch_deg"], TOL_DEG, (h, p, r, e))
                    self.assertLessEqual(e["roll_deg"], TOL_DEG, (h, p, r, e))
                    # the adopted mount is the fit, its TF / height_cell from car_mount
                    a = res["adopted"]
                    hc, xyz, q = mount(a["height_m"], a["pitch_deg"], None, roll_deg=a["roll_deg"])
                    self.assertEqual((res["height_cell"], res["mount_tf"]), (hc, gc.tf_string(xyz, q)))
                    self.assertEqual(a["source"], "fit")

    def test_misconfigured_corrected_or_refused(self):
        h, p, r = 0.36, 27.0, 0.0
        fr = frames(render(h, p, r), 0.002, seed=7)
        for ch, cp in ((h, p + 3), (h, p - 3), (h + 0.02, p), (h - 0.02, p), (h + 0.02, p - 3)):
            res = gc.calibrate(fr, K, ch, cp)
            self.assertEqual(res["verdict"], "adopt", (ch, cp, res["reasons"]))
            e = errors(res, h, p, r)
            note(e)
            self.assertLessEqual(e["pitch_deg"], TOL_DEG)
            self.assertLessEqual(e["height_m"], TOL_M)
            self.assertLessEqual(e["roll_deg"], TOL_DEG)
            self.assertEqual(res["height_cell"], mount(res["adopted"]["height_m"], res["adopted"]["pitch_deg"])[0])
        # owner 10-28: 6 deg off in pitch is no longer refused -- the clean fit is adopted (accurately)
        for ch, cp in ((h, p + 6), (h, p - 6)):
            res = gc.calibrate(fr, K, ch, cp)
            self.assertEqual(res["verdict"], "adopt", (ch, cp, res["reasons"]))
            e = errors(res, h, p, r)
            self.assertLessEqual(e["pitch_deg"], TOL_DEG)
            self.assertLessEqual(e["height_m"], TOL_M)
        # the height limit (+-3 cm) still refuses
        res = gc.calibrate(fr, K, h + 0.04, p)
        self.assertEqual(res["verdict"], "reject")
        self.assertIn("装的位置和配置差太多", res["reasons"][0])
        self.assertIsNone(res["mount_tf"])
        # a real roll of 6 deg (10-28 car: 5.8 deg refused before) is adopted
        res = gc.calibrate(frames(render(h, p, 6.0), 0.002), K, h, p)
        self.assertEqual(res["verdict"], "adopt", res["reasons"])
        self.assertLessEqual(errors(res, h, p, 6.0)["roll_deg"], TOL_DEG)

    def test_check_mode(self):
        h, p = 0.36, 27.0
        fr = frames(render(h, p, 0.0), 0.002, seed=3)
        res = gc.calibrate(fr, K, h, p + 0.5, mode="check")
        self.assertEqual(res["verdict"], "confirm", res["reasons"])
        self.assertEqual(res["adopted"], {"height_m": h, "pitch_deg": p + 0.5, "roll_deg": 0.0, "source": "config"})
        hc, xyz, q = mount(h, p + 0.5)
        self.assertEqual(res["mount_tf"], gc.tf_string(xyz, q))      # the configured TF, as car_mount.py prints it
        for ch, cp in ((h, p + 2.0), (h + 0.015, p)):
            res = gc.calibrate(fr, K, ch, cp, mode="check")
            self.assertEqual(res["verdict"], "reject", (ch, cp))
            self.assertIn("核对不过", res["reasons"][0])
        res = gc.calibrate(frames(render(h, p, 1.5), 0.002), K, h, p, mode="check")
        self.assertEqual(res["verdict"], "reject")
        # check also refuses when the quality gates fail (nothing to check against)
        res = gc.calibrate([np.zeros((H, W), np.uint16)] * 10, K, h, p, mode="check")
        self.assertEqual(res["verdict"], "reject")

    def test_negative_controls_refused(self):
        h, p = 0.36, 27.0
        cases = {
            # a wall 0.30 m ahead of the camera across the whole view: no ground 0.3-2.0 m ahead
            "wall": [((0.40, -5.0, 0.0), (0.50, 5.0, 3.0))],
            # a box 0.4-0.7 m ahead, 1.2 m wide, taller than the camera: most of the predicted ground hidden
            "box_most": [((0.50, -0.6, 0.0), (0.80, 0.6, 0.6))],
        }
        for name, boxes in cases.items():
            res = gc.calibrate(frames(render(h, p, 0.0, boxes), 0.002), K, h, p)
            self.assertEqual(res["verdict"], "reject", name)
            self.assertIn("车前 0.3–2 m", res["reasons"][0], name)
            self.assertLess(res["quality"]["full"]["inlier_share"], gc.MIN_INLIER_SHARE, name)
        res = gc.calibrate([np.zeros((H, W), np.uint16)] * 10, K, h, p)
        self.assertEqual(res["verdict"], "reject")
        self.assertIn("深度图全 0", res["reasons"][0])
        self.assertIsNone(res["mount_tf"])
        # the camera (or the car) moving between the two halves: refused by the halves gate
        fr = frames(render(h, p, 0.0), 0.002)[:5] + frames(render(h, p + 1.0, 0.0), 0.002, seed=2)[:5]
        res = gc.calibrate(fr, K, h, p)
        self.assertEqual(res["verdict"], "reject")
        self.assertTrue(any("两半帧" in x for x in res["reasons"]), res["reasons"])
        # a rough floor (static bumps +-6 cm in depth, the same in every frame: a median does not remove them):
        # refused (inliers, bend, halves -- 10-15: 6 %, 4.9 mm, 0.32 deg / 7.6 mm)
        bumps = np.random.RandomState(5).randint(-60, 61, (H, W))
        rough = [np.where(f > 0, f.astype(np.int64) + bumps, 0).clip(0, 65535).astype(np.uint16)
                 for f in frames(render(h, p, 0.0), 0.001)]
        res = gc.calibrate(rough, K, h, p)
        self.assertEqual(res["verdict"], "reject")
        self.assertTrue(any("系统弯曲" in x for x in res["reasons"]), res["reasons"])
        # the RMS gate's wiring (1.5 cm over the inliers): with the Tukey band and the local-normal exclusion the
        # inlier RMS stays near 1 cm even on that rough floor, so the gate is checked here with its limit lowered
        fr = frames(render(h, p, 0.0), 0.002)
        lim = gc.MAX_RMS_M
        try:
            gc.MAX_RMS_M = 1e-4
            res = gc.calibrate(fr, K, h, p)
        finally:
            gc.MAX_RMS_M = lim
        self.assertEqual(res["verdict"], "reject")
        self.assertTrue(any("拟合残差" in x for x in res["reasons"]), res["reasons"])
        self.assertEqual(gc.calibrate(fr, K, h, p)["verdict"], "adopt")
        # the bend gate's wiring the same way (limit lowered on a clean floor)
        lim = gc.BEND_MAX_M
        try:
            gc.BEND_MAX_M = 1e-7
            res = gc.calibrate(fr, K, h, p)
        finally:
            gc.BEND_MAX_M = lim
        self.assertEqual(res["verdict"], "reject")
        self.assertTrue(any("系统弯曲" in x for x in res["reasons"]), res["reasons"])

    def test_hard_limits_and_given_height_cell(self):
        # fitted 0.40 m (configured 0.38, within 3 cm): above the 0.395 m hard limit (0.39 until 10-17)
        res = gc.calibrate(frames(render(0.40, 27.0), 0.002), K, 0.38, 27.0)
        self.assertEqual(res["verdict"], "reject")
        self.assertIn("硬限", res["reasons"][0])
        # fitted pitch 37 deg (configured 34): above 35
        res = gc.calibrate(frames(render(0.36, 37.0), 0.002), K, 0.36, 34.0)
        self.assertEqual(res["verdict"], "reject")
        self.assertIn("35", res["reasons"][0])
        # CAR_HEIGHT_CELL given: 8 holds a fitted 0.36 m, 7 does not
        fr = frames(render(0.36, 27.0), 0.002)
        self.assertEqual(gc.calibrate(fr, K, 0.36, 27.0, height_cell=8)["height_cell"], 8)
        res = gc.calibrate(fr, K, 0.34, 27.0, height_cell=7)          # configured 0.34 fits 7, the fit 0.36 does not
        self.assertEqual(res["verdict"], "reject")
        self.assertIn("CAR_HEIGHT_CELL=7", res["reasons"][0])
        # derived: the fit moves the cell (configured 0.33 -> 7, fitted 0.36 -> 8)
        self.assertEqual(gc.calibrate(fr, K, 0.33, 27.0)["height_cell"], 8)

    def test_reproducible(self):
        fr = frames(render(0.35, 25.0, 1.0, [((0.9, -0.1, 0.0), (1.1, 0.2, 0.15))]), 0.003)
        a = gc.calibrate(fr, K, 0.36, 27.0)
        b = gc.calibrate([f.copy() for f in fr], K, 0.36, 27.0)
        self.assertEqual(json.dumps(a, sort_keys=True), json.dumps(b, sort_keys=True))

    def test_wide_obstacle_never_adopts_a_wrong_mount(self):
        """Hard acceptance item for the plane-fit kernel (header; controller 2026-10-15 20:56:35 CST): a wide
        wall/box across the floor ahead must give a refusal or a fit within the criterion -- never an adopted wrong
        mount. The first placeholder (RANSAC 2 cm + one least-squares refit) failed it (0.59-0.76 deg / 4-5 mm off
        with a 2 m wide wall at 0.7-0.9 m: the wall foot inside the 2 cm band tilts the refit); the selected kernel
        (ground_plane_kernel.py) passes it.
        Why the research note did not reproduce the bias with the LOWER HALF of the image: our candidates are the
        pixels the configured mount predicts as ground 0.3-2.0 m ahead, a wider area -- at 27 deg the foot of a wall
        0.7 m or more ahead lies in the upper half, which the lower half alone never sees."""
        self.wide_sweep(blur_px=0)

    def test_wide_obstacle_with_blurred_edges(self):
        """additional (not part of the written item): the same sweep with the depth box-blurred over 9 px (a stereo
        matching window rounding the wall foot, as the research note's SMOOTH=9) -- refusal or within the criterion"""
        self.wide_sweep(blur_px=9)

    def wide_sweep(self, blur_px):
        bad, cells = [], []
        for dist in WIDE_DIST_M:
            for width in WIDE_WIDTH_M:
                x0 = MOUNT_XYZ[0] + dist                      # front face, base x (the camera is at MOUNT_XYZ x)
                box = ((x0, -width / 2, 0.0), (x0 + 0.3, width / 2, 0.6))
                d = render(0.36, 27.0, 0.0, [box])
                if blur_px:
                    d = box_blur(d, blur_px)
                res = gc.calibrate(frames(d, 0.002), K, 0.36, 27.0)
                e = errors(res, 0.36, 27.0, 0.0) if res["fitted"] else None
                cells.append((dist, width, res["verdict"], e))
                if res["verdict"] == "adopt" and (e["height_m"] > TOL_M or e["pitch_deg"] > TOL_DEG
                                                  or e["roll_deg"] > TOL_DEG):
                    bad.append((dist, width, e))
        if not blur_px:
            WIDE_CELLS[:] = cells
        else:
            BLUR_CELLS[:] = cells
        self.assertEqual(bad, [])


# --- 10-17 (camera re-tilted to about 7 deg; docs/design/CAM_TILT_10_20261017.md sections 2.1 and 9) ---------------
# The car's live depth camera_info K (640x480 HW D2C, fx 517.17; tools/nav_coldstart_crawl/crawl_lib.K_BOARD): the K
# ground_calib reads on the car (63 x 50 deg; the fixture K above is the 91 x 66 deg datasheet one).
K_CAR = np.array([[517.1719360351562, 0.0, 324.2197265625], [0.0, 517.1170654296875, 240.64755249023438],
                  [0.0, 0.0, 1.0]])
W_CAR, H_CAR = 640, 480


def render_k(Kx, w, h_img, height_m, pitch_deg, roll_deg=0.0, boxes=()):
    """render() for another camera K / image size."""
    T = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], height_m), pitch_deg, roll_deg)
    d = render_floor(Kx, w, h_img, T)
    for b in boxes:
        db = render_floor(Kx, w, h_img, T, b)
        d = np.where(np.isfinite(db) & ~(db >= d), db, d)
    return d


class TestRangeRule(unittest.TestCase):
    """the selection range by configured pitch (section 9.1, controller 10-17): >= 20 deg RANGE_M as before (the
    whole 20-33 deg validation unchanged), < 20 deg the far end cut to 1.4 m. Not a quality gate."""

    def test_constants_and_boundary(self):
        self.assertEqual((gc.SHALLOW_PITCH_DEG, gc.SHALLOW_RANGE_MAX_M), (20.0, 1.4))
        self.assertEqual(gc.RANGE_M, (0.3, 2.0))
        for p in (20.0, 21.0, 27.0, 30.0, 33.0, 35.0):
            self.assertIs(gc.range_for(p), gc.RANGE_M, p)
        for p in (19.999, 17.5, 15.0, 10.0, 7.0, 5.0, 0.0):
            self.assertEqual(gc.range_for(p), (0.3, 1.4), p)
        self.assertEqual((gc.range_text(gc.RANGE_M), gc.range_text((0.3, 1.4))), ("0.3–2", "0.3–1.4"))

    def test_used_by_calibrate_and_recorded(self):
        u, v = gc.sample_pixels(W_CAR, H_CAR)
        fr = frames(render_k(K_CAR, W_CAR, H_CAR, 0.375, 10.0), 0.002, seed=4)
        res = gc.calibrate(fr, K_CAR, 0.375, 10.0)
        self.assertEqual(res["gates"]["range_m"], [0.3, 1.4])
        self.assertEqual(res["gates"]["range_rule"], {"shallow_pitch_deg": 20.0, "shallow_range_max_m": 1.4,
                                                      "range_m_default": [0.3, 2.0]})
        n14 = int(gc.predicted_ground(K_CAR, u, v, 0.375, 10.0, 0.0, (0.3, 1.4)).sum())
        for part in ("full", "first_half", "second_half"):
            self.assertEqual(res["quality"][part]["predicted_ground_samples"], n14, part)
        self.assertLess(n14, int(gc.predicted_ground(K_CAR, u, v, 0.375, 10.0).sum()))
        # >= 20 deg: no new key, the range as before
        res = gc.calibrate(frames(render_k(K_CAR, W_CAR, H_CAR, 0.375, 27.0), 0.002, seed=4), K_CAR, 0.375, 27.0)
        self.assertEqual(res["gates"]["range_m"], [0.3, 2.0])
        self.assertNotIn("range_rule", res["gates"])
        # the refusal text names the range in use
        res = gc.calibrate(frames(render_k(K_CAR, W_CAR, H_CAR, 0.375, 10.0, 0.0,
                                           [((0.40, -5.0, 0.0), (0.50, 5.0, 3.0))]), 0.002), K_CAR, 0.375, 10.0)
        self.assertEqual(res["verdict"], "reject")
        self.assertIn("车前 0.3–1.4 m", res["reasons"][0])
        self.assertIn("前方 0.3–1.4 m", res["reasons"][0])

    def test_predicted_ground_margin_5_15_deg(self):
        # noise-free, unoccluded, range cut to 1.4 m: 5-15 deg x 0.33-0.39 m x roll +-3, both K. The 2 x MIN_INLIERS
        # convention of test_predicted_ground_counts_have_margin does not hold here (lowest 5194 / 5341, measured
        # before the rule was coded, section 9.1): asserted > 1.5 x MIN_INLIERS; at that corner the inlier-count gate
        # binds before the share gate (needs a share >= 58 %).
        for Kx, w, h_img in ((K_CAR, W_CAR, H_CAR), (K, W, H)):
            u, v = gc.sample_pixels(w, h_img)
            for hh in (0.33, 0.355, 0.38, 0.39):
                for p in (5.0, 7.5, 10.0, 12.5, 15.0):
                    for r in (-3.0, 0.0, 3.0):
                        n = int(gc.predicted_ground(Kx, u, v, hh, p, r, gc.range_for(p)).sum())
                        self.assertGreater(n, 1.5 * gc.MIN_INLIERS, (w, hh, p, r, n))


class TestShallowPitch(unittest.TestCase):
    """the written criteria of section 2.1 at 5-15 deg (gates unchanged), a few cells on the car's K; the full sweep
    is tools/cam_tilt10/calib_sweep.py"""

    def run_k(self, h, p, r=0.0, boxes=(), sigma0=0.002, seed=1, cfg=None, mode="auto"):
        fr = frames(render_k(K_CAR, W_CAR, H_CAR, h, p, r, boxes), sigma0, seed=seed)
        ch, cp = cfg if cfg else (h, p)
        return gc.calibrate(fr, K_CAR, ch, cp, mode=mode)

    def assert_fit(self, res, h, p, r):
        self.assertEqual(res["verdict"], "adopt", res["reasons"])
        e = errors(res, h, p, r)
        note(e)
        self.assertLessEqual(e["height_m"], TOL_M, e)
        self.assertLessEqual(e["pitch_deg"], TOL_DEG, e)
        self.assertLessEqual(e["roll_deg"], TOL_DEG, e)
        a = res["adopted"]
        hc, xyz, q = mount(a["height_m"], a["pitch_deg"], None, roll_deg=a["roll_deg"])
        self.assertEqual((res["height_cell"], res["mount_tf"]), (hc, gc.tf_string(xyz, q)))

    def test_mount_cells(self):
        box = ((0.9, -0.10, 0.0), (1.1, 0.20, 0.15))
        for k, (h, p, r, b) in enumerate(((0.375, 10.0, 0.0, False), (0.38, 7.0, 0.6, True), (0.33, 5.0, -3.0, True),
                                          (0.355, 15.0, 3.0, False), (0.38, 12.5, 0.0, True))):
            self.assert_fit(self.run_k(h, p, r, (box,) if b else (), (0.001, 0.002, 0.003)[k % 3], seed=k + 1), h, p, r)

    def test_measured_mount_check_confirms(self):
        # the car's measured 0.39 m / 7.0 deg (board fit 0.3915 / 7.05): check keeps the configuration
        res = self.run_k(0.39, 7.0, 0.5, mode="check")
        self.assertEqual(res["verdict"], "confirm", res["reasons"])
        self.assertEqual(res["adopted"], {"height_m": 0.39, "pitch_deg": 7.0, "roll_deg": 0.0, "source": "config"})

    def test_misconfigured_corrected_or_refused(self):
        h, p = 0.375, 10.0
        fr = frames(render_k(K_CAR, W_CAR, H_CAR, h, p), 0.002, seed=7)
        for ch, cp in ((h, p + 3), (h, p - 3), (h + 0.02, p - 3)):
            res = gc.calibrate(fr, K_CAR, ch, cp)
            self.assert_fit(res, h, p, 0.0)
        for ch, cp in ((h, p + 6), (h, p - 6)):   # owner 10-28: adopted now (no angle limit)
            res = gc.calibrate(fr, K_CAR, ch, cp)
            self.assert_fit(res, h, p, 0.0)

    def test_negative_controls_refused(self):
        for name, boxes in (("wall", [((0.40, -5.0, 0.0), (0.50, 5.0, 3.0))]),
                            ("box_most", [((0.50, -0.6, 0.0), (0.80, 0.6, 0.6))])):
            res = self.run_k(0.375, 10.0, 0.0, boxes)
            self.assertEqual(res["verdict"], "reject", name)
            self.assertIn("车前 0.3–1.4 m", res["reasons"][0], name)


class TestPriorSearch(unittest.TestCase):
    """10-28 (controller): the owner re-tilted the camera to about 25 deg (board: 0.38 m / 25.0 / roll 1.4) with the
    configuration still at 7 deg -- a prior 7 or 15 deg finds no floor. calibrate_search: last good prior, then
    the configuration, then a 0-35 deg sweep; the attempt with the most inliers that passes every gate."""

    def frames(self, h, p, r=0.0):
        return frames(render_k(K_CAR, W_CAR, H_CAR, h, p, r), 0.002, seed=3)

    def test_constants(self):
        self.assertEqual(gc.SWEEP_PITCH_DEG, (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0))

    def test_tilted_to_25_with_config_7(self):
        fr = self.frames(0.38, 25.0, 1.4)
        self.assertEqual(gc.calibrate(fr, K_CAR, 0.39, 7.0)["verdict"], "reject")      # the old search alone
        res = gc.calibrate_search(fr, K_CAR, 0.39, 7.0)
        self.assertEqual((res["verdict"], res["adopted"]["source"], res["adopted"]["prior"]), ("adopt", "fit", "sweep"))
        e = errors(res, 0.38, 25.0, 1.4)
        self.assertLessEqual(e["pitch_deg"], TOL_DEG, e)
        self.assertLessEqual(e["height_m"], TOL_M, e)
        self.assertLessEqual(e["roll_deg"], TOL_DEG, e)
        self.assertEqual([t["prior"] for t in res["search"]][:1], ["config"])
        # the last good 25 deg as the prior: adopted at once (one attempt)
        res2 = gc.calibrate_search(fr, K_CAR, 0.39, 7.0, last_good=(0.38, 25.0, 1.4))
        self.assertEqual((res2["verdict"], res2["adopted"]["prior"], len(res2["search"])), ("adopt", "last_good", 1))
        # a stale last good (7 deg) does not stop the search
        res3 = gc.calibrate_search(fr, K_CAR, 0.39, 7.0, last_good=(0.39, 7.0, 0.0))
        self.assertEqual((res3["verdict"], res3["adopted"]["prior"]), ("adopt", "sweep"))

    def test_config_right_is_the_old_result(self):
        fr = self.frames(0.39, 8.7, 0.6)
        old = gc.calibrate(fr, K_CAR, 0.39, 7.0)
        new = gc.calibrate_search(fr, K_CAR, 0.39, 7.0)
        self.assertEqual(old["adopted"], {k: v for k, v in new["adopted"].items() if k != "prior"})
        self.assertEqual(len(new["search"]), 1)

    def test_no_floor_still_refused(self):
        fr = [np.zeros((H_CAR, W_CAR), np.uint16)] * 10                    # depth all 0: no sweep, refused
        res = gc.calibrate_search(fr, K_CAR, 0.39, 7.0)
        self.assertEqual(res["verdict"], "reject")
        self.assertEqual(len(res["search"]), 1)
        # a wall right ahead: no prior of the sweep finds a floor
        res = gc.calibrate_search(frames(render_k(K_CAR, W_CAR, H_CAR, 0.39, 8.0,
                                                  boxes=[((0.25, -5.0, 0.0), (0.35, 5.0, 3.0))]), 0.002),
                                  K_CAR, 0.39, 7.0)
        self.assertEqual(res["verdict"], "reject")
        self.assertIsNone(res["mount_tf"])


class TestHardHeight395(unittest.TestCase):
    """owner 10-17: the adopted height's upper limit 0.39 -> 0.395 m. Lens in [0.35, 0.40) = the same producer cell
    (height_cell 8, block -0.05..0.45 m); 0.40 m needs height_cell 9, which the block check refuses."""

    def test_fit_0_3915_adopted(self):
        # the car's measured mount (board c10_1: 0.3915 m / 7.05 deg / roll 0.59), configured 0.39 / 7.0, auto
        fr = frames(render_k(K_CAR, W_CAR, H_CAR, 0.3915, 7.05, 0.59), 0.002, seed=9)
        res = gc.calibrate(fr, K_CAR, 0.39, 7.0)
        self.assertEqual(res["verdict"], "adopt", res["reasons"])
        e = errors(res, 0.3915, 7.05, 0.59)
        self.assertLessEqual(e["height_m"], TOL_M)
        self.assertLessEqual(e["pitch_deg"], TOL_DEG)
        self.assertLessEqual(e["roll_deg"], TOL_DEG)
        self.assertEqual(res["height_cell"], 8)

    def test_fit_0_396_refused(self):
        fr = frames(render_k(K_CAR, W_CAR, H_CAR, 0.396, 7.0), 0.002, seed=9)
        res = gc.calibrate(fr, K_CAR, 0.39, 7.0)
        self.assertEqual(res["verdict"], "reject")
        self.assertGreater(res["fitted"]["height_m"], 0.395)
        self.assertEqual(len(res["reasons"]), 1, res["reasons"])
        self.assertIn("超出硬限 0.31–0.395 m", res["reasons"][0])
        self.assertIsNone(res["mount_tf"])

    def test_0_40_refused_by_the_block_check(self):
        self.assertEqual(mount(0.3999, 7.0)[0], 8)
        with self.assertRaises(ValueError):
            mount(0.40, 7.0)
        # behind the hard limit, the block check refuses 0.40 on its own (hard limit lifted for this check only)
        fr = frames(render_k(K_CAR, W_CAR, H_CAR, 0.402, 7.0), 0.002, seed=9)   # a fit at / just above 0.40
        lim = gc.HARD_HEIGHT_M
        try:
            gc.HARD_HEIGHT_M = (0.31, 0.45)
            res = gc.calibrate(fr, K_CAR, 0.39, 7.0)
        finally:
            gc.HARD_HEIGHT_M = lim
        self.assertGreaterEqual(res["fitted"]["height_m"], 0.40)
        self.assertEqual(res["verdict"], "reject")
        self.assertIn("放不进几何块", res["reasons"][0])
        res = gc.calibrate(fr, K_CAR, 0.39, 7.0)
        self.assertEqual(res["verdict"], "reject")
        self.assertIn("硬限", res["reasons"][0])


class TestCli(unittest.TestCase):
    """--from-npz (offline re-run of a saved capture), --out JSON, exit codes, --read for car_inner.sh, --compare"""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="gcalib_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def cli(self, *args):
        return subprocess.run([sys.executable, os.path.join(ROOT, "tools", "car_run", "ground_calib.py")] + list(args),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=120)

    def test_cli_round_trip(self):
        npz = os.path.join(self.tmp, "cap.npz")
        np.savez_compressed(npz, frames=np.stack(frames(render(0.355, 28.0, 1.0), 0.002)), K=K)
        out = os.path.join(self.tmp, "gc.json")
        r = self.cli("--mode", "auto", "--height-m", "0.36", "--pitch-deg", "30", "--from-npz", npz, "--out", out)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        res = json.load(open(out))
        self.assertEqual(res["verdict"], "adopt")
        r = self.cli("--read", out)
        self.assertEqual(r.returncode, 0, r.stderr)
        f = r.stdout.split()
        self.assertEqual(len(f), 12)
        self.assertEqual(f[0], str(res["height_cell"]))
        self.assertEqual(f[4], "fit")
        self.assertEqual(" ".join(f[5:]), res["mount_tf"])
        self.assertAlmostEqual(float(f[2]), 28.0, delta=TOL_DEG)
        # 10-26 owner（重成功率）: an auto refusal falls back to the configured mount -> 0, the refusal text in
        # "warnings", --read gives source config_fallback (before: exit 1 and --read refused it)
        # (10-28: the refusal is now the height limit -- configured 0.31 m against the fitted 0.355 m; an angle
        # difference alone no longer refuses)
        out2 = os.path.join(self.tmp, "gc2.json")
        r = self.cli("--mode", "auto", "--height-m", "0.31", "--pitch-deg", "28", "--from-npz", npz, "--out", out2)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        res2 = json.load(open(out2))
        self.assertEqual((res2["verdict"], res2["adopted"]["source"], res2["reasons"]), ("adopt", "config_fallback", []))
        self.assertIn("装的位置和配置差太多", res2["warnings"][0])
        self.assertEqual((res2["adopted"]["height_m"], res2["adopted"]["pitch_deg"]), (0.31, 28.0))
        self.assertEqual(res2["alert"], gc.ALERT_CONFIG)
        f2 = self.cli("--read", out2).stdout.split()
        self.assertEqual((f2[1], f2[2], f2[4]), ("0.3100", "28.000", "config_fallback"))
        # 10-28 owner: the last adopted fit is kept (--write-last-good, car_up after READY) and a refused start
        # falls back to IT first (source last_good), the configuration only without a usable one
        lg = os.path.join(self.tmp, gc.LAST_GOOD_NAME)
        self.assertEqual(self.cli("--write-last-good", out2, lg).returncode, 0)
        self.assertFalse(os.path.exists(lg))                       # a fallback result is never written
        self.assertEqual(self.cli("--write-last-good", out, lg).returncode, 0)
        rec = json.load(open(lg))
        self.assertEqual((rec["height_m"], rec["pitch_deg"], rec["roll_deg"]),
                         tuple(res["adopted"][k] for k in ("height_m", "pitch_deg", "roll_deg")))
        r = self.cli("--mode", "auto", "--height-m", "0.31", "--pitch-deg", "28", "--from-npz", npz, "--out", out2,
                     "--last-good", lg)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        res3 = json.load(open(out2))
        self.assertEqual((res3["verdict"], res3["adopted"]["source"], res3["alert"]),
                         ("adopt", "last_good", gc.ALERT_LAST_GOOD))
        self.assertAlmostEqual(res3["adopted"]["pitch_deg"], 28.0, delta=TOL_DEG)
        self.assertEqual(res3["mount_tf"], res["mount_tf"])
        self.assertEqual(self.cli("--read", out2).stdout.split()[4], "last_good")
        with open(lg, "w") as fh:
            fh.write("{broken")
        self.cli("--mode", "auto", "--height-m", "0.31", "--pitch-deg", "28", "--from-npz", npz, "--out", out2,
                 "--last-good", lg)
        res4 = json.load(open(out2))
        self.assertEqual(res4["adopted"]["source"], "config_fallback")
        self.assertIn("读不出", res4["last_good"]["why_not"])
        # no depth input + last good: the last good mount too
        self.cli("--write-last-good", out, lg)
        self.cli("--height-m", "0.36", "--pitch-deg", "30", "--from-npz", os.path.join(self.tmp, "none.npz"),
                 "--out", out2, "--last-good", lg)
        self.assertEqual(json.load(open(out2))["adopted"]["source"], "last_good")
        # check mode still refuses (exit 1, reasons in the JSON: car_inner.sh check_reasons reads "reasons")
        r = self.cli("--mode", "check", "--height-m", "0.31", "--pitch-deg", "28", "--from-npz", npz, "--out", out2)
        self.assertEqual(r.returncode, 1, r.stdout + r.stderr)
        self.assertEqual(self.cli("--read", out2).returncode, 1)
        # no input: auto falls back too (0, warning); check -> 2, still a JSON with reasons
        out3 = os.path.join(self.tmp, "gc3.json")
        r = self.cli("--height-m", "0.36", "--pitch-deg", "30", "--from-npz", os.path.join(self.tmp, "none.npz"),
                     "--out", out3)
        self.assertEqual(r.returncode, 0)
        self.assertIn("抓不到深度图", json.load(open(out3))["warnings"][0])
        r = self.cli("--mode", "check", "--height-m", "0.36", "--pitch-deg", "30", "--from-npz",
                     os.path.join(self.tmp, "none.npz"), "--out", out3)
        self.assertEqual(r.returncode, 2)
        self.assertIn("抓不到深度图", json.load(open(out3))["reasons"][0])
        # compare with the mount stored beside a map
        mp = os.path.join(self.tmp, "map.json")
        m = dict(res, adopted=dict(res["adopted"], pitch_deg=res["adopted"]["pitch_deg"] - 1.5))
        json.dump(m, open(mp, "w"))
        r = self.cli("--compare", mp, out)
        self.assertEqual(r.returncode, 0)
        self.assertIn("pitch +1.50 deg", r.stdout)
        self.assertIn("WARNING", r.stdout)
        r = self.cli("--compare", out, out)
        self.assertNotIn("WARNING", r.stdout)
        self.assertIn("not compared", self.cli("--compare", os.path.join(self.tmp, "nope.json"), out).stdout)



class TestCarInnerRead(unittest.TestCase):
    """the --read line through the very `read -r ... <<< "$GC"` line of car_inner.sh, run by bash (the line is
    grepped from the script, not copied, so this follows the source): all six variables, CAM_MOUNT_TF all 7 numbers."""
    READ_RE = "read -r HEIGHT_CELL "

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(ROOT, "tools", "car_run", "car_inner.sh")) as f:
            hits = [ln.strip() for ln in f if cls.READ_RE in ln]
        assert len(hits) == 1, hits
        cls.line = hits[0]
        assert cls.line.endswith('<<< "$GC"'), cls.line

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="gcread_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def shell(self, res):
        out = os.path.join(self.tmp, "gc.json")
        with open(out, "w") as f:
            json.dump(res, f, indent=1, ensure_ascii=False)          # as main() writes --out
        r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "car_run", "ground_calib.py"), "--read", out],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        snippet = ('GC=$(cat "$1")\n' + self.line + '\n'
                   'printf "%s\\n" "$HEIGHT_CELL" "$CAM_HEIGHT" "$CAM_PITCH_DEG" "$GC_ROLL" "$GC_SRC" "$CAM_MOUNT_TF"\n')
        line_file = os.path.join(self.tmp, "gc.line")
        with open(line_file, "w") as f:
            f.write(r.stdout)
        b = subprocess.run(["bash", "-c", snippet, "bash", line_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=30)
        self.assertEqual(b.returncode, 0, b.stderr)
        v = b.stdout.split("\n")
        self.assertEqual(len(v), 7, b.stdout)                         # six values + the trailing newline
        self.assertEqual(v[6], "")
        return v[:6]

    def check(self, res, src):
        a = res["adopted"]
        hc, h, p, r, s, tf = self.shell(res)
        self.assertEqual(hc, str(res["height_cell"]))
        self.assertEqual(h, "%.4f" % a["height_m"])
        self.assertEqual(p, "%.3f" % a["pitch_deg"])
        self.assertEqual(r, "%.3f" % a["roll_deg"])
        self.assertEqual(s, src)
        self.assertEqual(tf, res["mount_tf"])
        self.assertEqual(len(tf.split(" ")), 7)
        [float(x) for x in tf.split(" ")]
        return a

    def test_auto_adopts_fit(self):
        res = gc.calibrate(frames(render(0.355, 28.0), 0.002), K, 0.36, 30.0, mode="auto")
        self.assertEqual(res["verdict"], "adopt", res["reasons"])
        a = self.check(res, "fit")
        self.assertAlmostEqual(a["pitch_deg"], 28.0, delta=TOL_DEG)

    def test_check_keeps_config(self):
        res = gc.calibrate(frames(render(0.36, 27.0), 0.002, seed=3), K, 0.36, 27.5, mode="check")
        self.assertEqual(res["verdict"], "confirm", res["reasons"])
        a = self.check(res, "config")
        self.assertEqual((a["height_m"], a["pitch_deg"], a["roll_deg"]), (0.36, 27.5, 0.0))

    def test_auto_with_roll(self):
        res = gc.calibrate(frames(render(0.36, 27.0, -2.0), 0.002), K, 0.36, 27.0, mode="auto")
        self.assertEqual(res["verdict"], "adopt", res["reasons"])
        a = self.check(res, "fit")
        self.assertAlmostEqual(a["roll_deg"], -2.0, delta=TOL_DEG)
        self.assertTrue(self.shell(res)[3].startswith("-"))

if __name__ == "__main__":
    unittest.main()
