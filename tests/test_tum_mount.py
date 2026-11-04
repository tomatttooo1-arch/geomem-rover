"""tools/nano_ros2/tum_mount.py: the host replay fixture's camera mount comes from the TUM dataset's groundtruth
(2026-10-16 第 6 棒).

The host replay of TUM fr2/pioneer_360 used the real car's mount (0.36 m / 30 deg down; the Kinect looks ~5.7 deg
down), base_link stood ~24 deg off the floor, and the 3-DoF projection of the 6-DoF odometry made fake 8-9 deg yaw
steps in the gate's map->odom corrections (gate_dig/JUMP_DIG.md).

- known answer: a synthetic groundtruth (robot turning about the floor normal and driving, camera on a known mount,
  both signs of roll, two worlds) -> the tool returns that mount (angles <= 0.05 deg, the TF equals base_camera's).
- the check that pins sign and size: the dataset's rotation axis mapped through the mount is base z (< 1 deg) for
  the derived mount; the car's 30 deg mount is refused (~24 deg) -- CLI exit codes too (the launch's fail-closed step).
- positive control of the defect itself, zero free parameters: with the 30 deg mount the ZYX yaw of the projected
  odometry departs from the true heading by >= 8 deg within 40 deg of turning and 25-29 deg over the sequence (as the
  gate saw on the host); with the derived mount < 0.5 deg.
- the real dataset (skipped when /data1/... is absent): pitch 5.0-6.5 deg, |roll| < 1.5 deg, 30 deg -> 24.3 deg tilt.
Python 3.8+.
"""
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.car_geo_producer.common import base_camera  # noqa: E402
from tools.nano_ros2 import tum_mount as tm  # noqa: E402

TOOL = ROOT / "tools/nano_ros2/tum_mount.py"
SEQ = Path("${FPGA_SHARED}/data/20261010-dev1-tum-pioneer/extracted/rgbd_dataset_freiburg2_pioneer_360")
CAR_TF = "0.1 0 0.36 -0.612372436 0.612372436 -0.353553391 0.353553391"   # car_mount.py 0.36 m / 30 deg (host_g4/g5)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def rot_axis(axis, a):
    k = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def synthetic_gt(path, pitch, roll, height, world_tilt=(0., 0., 0.), seconds=40., hz=100., turn_dps=12.):
    """groundtruth.txt of a robot turning (and creeping forward) on a floor whose normal is `world tilt` off the mocap z,
    camera on base_camera((0.1, 0, height), pitch, roll)."""
    T_bc = base_camera((0.1, 0., height), pitch, roll)
    W = rot_axis([1, 0, 0], math.radians(world_tilt[0])) @ rot_axis([0, 1, 0], math.radians(world_tilt[1])) \
        if any(world_tilt) else np.eye(3)
    rows = ["# ground truth trajectory (synthetic)", "# timestamp tx ty tz qx qy qz qw"]
    for k in range(int(seconds * hz)):
        t = k / hz
        yaw = math.radians(turn_dps) * t + 0.3 * math.sin(t / 3.)       # uneven turning, as a driven robot
        B = np.eye(4)
        B[:3, :3] = rot_z(yaw)
        B[:3, 3] = (0.05 * t, 0.2 * math.sin(t / 5.), 0.)
        C = B @ T_bc
        R = W @ C[:3, :3]
        p = W @ C[:3, 3] + np.array([1., -2., 0.])
        q = tm.matrix_quat(R)
        rows.append("%.4f %.6f %.6f %.6f %.7f %.7f %.7f %.7f" % ((1311876799. + t,) + tuple(p) + q))
    Path(path).write_text("\n".join(rows) + "\n")


def run_cli(*args):
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        rc = tm.main([str(a) for a in args])
    return rc, out.getvalue(), err.getvalue()


class KnownAnswer(unittest.TestCase):
    CASES = [(5.73, 0.52, 0.5685, (0., 0.)), (12.0, -2.0, 0.45, (0., 0.)), (30.0, 3.0, 0.36, (0.9, -0.4)),
             (5.73, 0.52, 0.5685, (0.89, 0.3))]

    def test_recovers_mount(self):
        for pitch, roll, h, wt in self.CASES:
            with self.subTest(pitch=pitch, roll=roll, world_tilt=wt), tempfile.TemporaryDirectory() as d:
                synthetic_gt(os.path.join(d, "groundtruth.txt"), pitch, roll, h, wt + (0.,))
                info, _ = tm.derive(d)
                self.assertAlmostEqual(info["pitch_deg"], pitch, delta=0.05)
                self.assertAlmostEqual(info["roll_deg"], roll, delta=0.05)
                self.assertLess(info["residual_tilt_deg"], 0.05)
                if not any(wt):
                    self.assertAlmostEqual(info["height_m"], h, delta=1e-4)
                    self.assertLess(info["axis_vs_world_z_deg"], 0.05)
                else:
                    self.assertAlmostEqual(info["axis_vs_world_z_deg"], math.hypot(*wt), delta=0.05)
                # the printed TF is base_camera of the true mount (quaternion up to sign)
                v = [float(x) for x in info["mount_tf"].split()]
                T = base_camera((0.1, 0., info["height_m"]), pitch, roll)
                self.assertTrue(np.allclose(v[:3], T[:3, 3], atol=1e-6))
                self.assertLess(np.abs(tm.quat_matrix(v[3:]) - T[:3, :3]).max(), 1e-3)

    def test_roll_sign(self):
        for roll in (2.0, -2.0):
            with self.subTest(roll=roll), tempfile.TemporaryDirectory() as d:
                synthetic_gt(os.path.join(d, "groundtruth.txt"), 8.0, roll, 0.5)
                self.assertAlmostEqual(tm.derive(d)[0]["roll_deg"], roll, delta=0.05)

    def test_height_override_and_quat_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            synthetic_gt(os.path.join(d, "groundtruth.txt"), 5.73, 0.52, 0.5685, seconds=20.)
            info, _ = tm.derive(d, height_m=0.605)
            self.assertEqual(info["height_source"], "--height-m")
            self.assertAlmostEqual(float(info["mount_tf"].split()[2]), 0.605, places=6)
        for R in (rot_axis([1, 2, 3], 2.9), rot_axis([0, 0, 1], math.pi), rot_axis([1, 0, 0], 0.)):
            self.assertLess(np.abs(tm.quat_matrix(tm.matrix_quat(R)) - R).max(), 1e-9)

    def test_no_turning_refused(self):
        with tempfile.TemporaryDirectory() as d:
            synthetic_gt(os.path.join(d, "groundtruth.txt"), 5.73, 0.5, 0.57, seconds=10., turn_dps=0.)
            # 0.3 sin(t/3) still turns ~5.7 deg/s at most: with 10 s at a 30 deg window threshold nothing qualifies
            with self.assertRaises(ValueError):
                tm.rotation_axis(*tm.load_groundtruth(os.path.join(d, "groundtruth.txt"))[::2], min_turn_deg=30.)
            rc, out, err = run_cli(os.path.join(d, "missing"))
            self.assertEqual(rc, tm.EXIT_REFUSED)
            self.assertEqual(out, "")


class CheckAndDefect(unittest.TestCase):
    """the fail-closed check and the zero-parameter reproduction of the fake yaw steps on a pioneer-like sequence."""

    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.TemporaryDirectory()
        synthetic_gt(os.path.join(cls.dir.name, "groundtruth.txt"), 5.73, 0.52, 0.5685, seconds=40., turn_dps=20.)   # pioneer_360: GT heading rate p50 12.7, p90 23 deg/s
        cls.info, (cls.t, cls.Rs, cls.w, cls.ac) = tm.derive(cls.dir.name)

    @classmethod
    def tearDownClass(cls):
        cls.dir.cleanup()

    def test_check_tf_cli(self):
        rc, out, err = run_cli(self.dir.name, "--check-tf", self.info["mount_tf"])
        self.assertEqual(rc, tm.EXIT_OK, err)
        self.assertEqual(out, "")            # check mode prints no TF (the launch reads stdout only in derive mode)
        rc, out, err = run_cli(self.dir.name, "--check-tf", CAR_TF)
        self.assertEqual(rc, tm.EXIT_REFUSED)
        self.assertIn("REFUSED", err)
        _, R = tm.tf_rotation(CAR_TF)
        self.assertAlmostEqual(tm.residual_tilt_deg(R, self.ac), 24.27, delta=0.1)
        rc, _, _ = run_cli(self.dir.name, "--check-tf", "1 2 3")
        self.assertEqual(rc, tm.EXIT_REFUSED)

    def test_cli_prints_tf_and_json(self):
        with tempfile.TemporaryDirectory() as d:
            j = os.path.join(d, "m.json")
            r = subprocess.run([sys.executable, str(TOOL), self.dir.name, "--json", j],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertEqual(r.stdout.strip(), self.info["mount_tf"])
            self.assertEqual(len(r.stdout.split()), 7)
            with open(j) as f:
                self.assertAlmostEqual(json.load(f)["pitch_deg"], 5.73, delta=0.05)

    def test_fake_yaw_steps_reproduced_with_car_mount_only(self):
        sub = slice(0, None, 10)

        heading = np.degrees(np.unwrap([tm.rotvec(R @ self.Rs[0].T) @ self.w for R in self.Rs[sub]]))

        def worst_40deg(e):
            # largest error change within 40 deg of heading change (JUMP_DIG 4.1: 14 deg at 24.3 deg tilt)
            return max(float(np.abs(e[(np.abs(heading - heading[i]) <= 40.)] - e[i]).max()) for i in range(len(e)))

        _, R30 = tm.tf_rotation(CAR_TF)
        e30 = tm.yaw_projection_error(self.Rs[sub], R30, self.w)
        _, Rt = tm.tf_rotation(self.info["mount_tf"])
        et = tm.yaw_projection_error(self.Rs[sub], Rt, self.w)
        self.assertGreater(e30.max() - e30.min(), 20.)     # host: M yaw swung 26-29 deg per run
        self.assertGreater(worst_40deg(e30), 8.)           # host: 8-12 deg steps within ~15-40 deg of turning
        self.assertLess(np.abs(et).max(), 0.5)
        self.assertLess(worst_40deg(et), 0.2)


@unittest.skipUnless((SEQ / "groundtruth.txt").is_file(), "TUM fr2/pioneer_360 not on this host")
class RealDataset(unittest.TestCase):
    def test_pioneer_360_mount(self):
        info, (t, Rs, w, ac) = tm.derive(str(SEQ))
        self.assertTrue(5.0 <= info["pitch_deg"] <= 6.5, info["pitch_deg"])      # JUMP_DIG 5.73 (p5-p95 4.6-6.6)
        self.assertLess(abs(info["roll_deg"]), 1.5)
        self.assertLess(info["axis_vs_world_z_deg"], 2.0)
        self.assertTrue(0.5 < info["height_m"] < 0.65)
        _, R30 = tm.tf_rotation(CAR_TF)
        self.assertAlmostEqual(tm.residual_tilt_deg(R30, ac), 24.3, delta=0.5)
        rc, _, _ = run_cli(str(SEQ), "--check-tf", CAR_TF)
        self.assertEqual(rc, tm.EXIT_REFUSED)
        rc, _, _ = run_cli(str(SEQ), "--check-tf", info["mount_tf"])
        self.assertEqual(rc, tm.EXIT_OK)
        sub = slice(0, None, 30)
        e30 = tm.yaw_projection_error(Rs[sub], R30, w)
        self.assertTrue(25. < e30.max() - e30.min() < 29.5)   # the 26-29 deg the gate saw
        _, Rt = tm.tf_rotation(info["mount_tf"])
        self.assertLess(np.abs(tm.yaw_projection_error(Rs[sub], Rt, w)).max(), 0.5)


if __name__ == "__main__":
    unittest.main()
