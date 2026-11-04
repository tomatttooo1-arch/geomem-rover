"""tools/nano_ros2/tum_gt_odom.py + tum_rgbd_pub.py --gt-odom-tf: groundtruth odometry for a distortion-free prior
library (定位门第 11 棒, owner 10-17 12:31: S 夹具用真值重建地图库).

Known answers: hand-built groundtruth (identity mount, the robot driving along its own x axis and turning) gives the
odometry by hand; the fixture's real mount on synthetic rows agrees with an independent matrix implementation of the
S instrument's formula (camera x inverse(mount), 3-DoF, odom = SE2(anchor)^-1 SE2(base)); stamps between rows
interpolate, outside the span are refused; equal neighbouring stamps (fr2/pioneer_360 has 3) are accepted. The
publisher sends the TF before the frame's images and refuses --pingpong."""
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.nano_ros2 import tum_gt_odom as g

ROOT = Path(__file__).resolve().parents[1]
PUB = ROOT / 'tools/nano_ros2/tum_rgbd_pub.py'
MOUNT = '0.1 0 0.5685 -0.52198145 0.52671712 -0.472249615 0.476534094'   # fr2/pioneer_360 (tum_mount.py)
IDENT = '0 0 0 0 0 0 1'


def zq(yaw):
    return [0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]


def row(t, x, y, z, q):
    return [t, x, y, z] + list(q)


def mat_quat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def independent_odom(rows, mount, anchor_i, i):
    """Matrices only: base = cam * inv(mount) -> planar (x, y, yaw) -> inv(SE2 anchor) * SE2 base."""
    v = [float(x) for x in mount.split()]
    M = np.eye(4); M[:3, :3] = mat_quat(*v[3:7]); M[:3, 3] = v[:3]

    def planar(r):
        C = np.eye(4); C[:3, :3] = mat_quat(*r[4:8]); C[:3, 3] = r[1:4]
        B = C @ np.linalg.inv(M)
        th = math.atan2(B[1, 0], B[0, 0])
        return np.array([[math.cos(th), -math.sin(th), B[0, 3]], [math.sin(th), math.cos(th), B[1, 3]], [0, 0, 1]])
    R = np.linalg.inv(planar(rows[anchor_i])) @ planar(rows[i])
    return np.array([R[0, 2], R[1, 2], math.atan2(R[1, 0], R[0, 0])])


class HandAnswerTests(unittest.TestCase):
    def setUp(self):
        # identity mount: base_link = the groundtruth frame. Start at (1, 2) facing +y (yaw 90 deg), drive 1 m forward,
        # then turn in place to face -x (yaw 180 deg)
        self.rows = [row(10., 1., 2., .5, zq(math.pi / 2)), row(11., 1., 3., .5, zq(math.pi / 2)),
                     row(12., 1., 3., .5, zq(math.pi))]
        self.base = g.GroundtruthBase(self.rows, IDENT)
        self.odom = g.GtOdom(self.base, 10.)

    def test_anchor_is_origin(self):
        np.testing.assert_allclose(self.odom.at(10.), [0, 0, 0], atol=1e-12)

    def test_forward_one_metre_is_plus_x(self):
        np.testing.assert_allclose(self.odom.at(11.), [1, 0, 0], atol=1e-12)

    def test_turn_left_90(self):
        np.testing.assert_allclose(self.odom.at(12.), [1, 0, math.pi / 2], atol=1e-12)

    def test_interpolates_between_rows(self):
        np.testing.assert_allclose(self.odom.at(10.5), [.5, 0, 0], atol=1e-12)
        np.testing.assert_allclose(self.odom.at(11.5), [1, 0, math.pi / 4], atol=1e-12)

    def test_outside_span_refused(self):
        self.assertFalse(self.base.covers(9.999))
        self.assertFalse(self.base.covers(12.001))
        self.assertTrue(self.base.covers(10.) and self.base.covers(12.))
        with self.assertRaises(ValueError):
            self.odom.at(12.5)
        self.assertEqual(g.first_covered([9., 9.5, 10.2, 11.], self.base), 10.2)
        self.assertIsNone(g.first_covered([1., 13.], self.base))

    def test_yaw_unwrapped_across_pi(self):
        rows = [row(0., 0, 0, 0, zq(math.radians(170))), row(1., 0, 0, 0, zq(math.radians(-170)))]
        o = g.GtOdom(g.GroundtruthBase(rows, IDENT), 0.)
        self.assertAlmostEqual(math.degrees(o.at(.5)[2]), 10., places=9)    # through 180, not back through 0
        self.assertAlmostEqual(math.degrees(o.at(1.)[2]), 20., places=9)

    def test_yaw_quat(self):
        for yaw in (-3., -1., 0., .7, 3.1):
            qx, qy, qz, qw = g.yaw_quat(yaw)
            R = mat_quat(qx, qy, qz, qw)
            self.assertAlmostEqual(math.atan2(R[1, 0], R[0, 0]), yaw, places=12)
            self.assertAlmostEqual(R[2, 2], 1., places=12)


class FixtureMountTests(unittest.TestCase):
    def test_real_mount_matches_independent_formula(self):
        rng = np.random.RandomState(7)
        rows, t = [], 100.
        for k in range(40):
            # an optical frame looking about 5.7 deg down, the robot at varying places / headings, small tilt noise
            yaw = rng.uniform(-math.pi, math.pi)
            cam = g.T(g.quat_R(*[float(v) for v in MOUNT.split()[3:]]), [0.1, 0., .5685])
            base = g.T(mat_quat(*zq(yaw)), [rng.uniform(-3, 3), rng.uniform(-3, 3), 0.])
            tilt = g.T(mat_quat(*(np.r_[rng.normal(0, .01, 3), 1.] / np.linalg.norm(np.r_[rng.normal(0, .01, 3), 1.]))),
                       [0, 0, 0])
            C = base @ tilt @ cam
            q = np.array(g_matrix_quat(C[:3, :3]))
            rows.append(row(t, *C[:3, 3], q))
            t += .5
        base = g.GroundtruthBase(rows, MOUNT)
        o = g.GtOdom(base, rows[3][0])
        for i in (3, 4, 10, 25, 39):
            got = o.at(rows[i][0])
            ref = independent_odom(rows, MOUNT, 3, i)
            np.testing.assert_allclose(got[:2], ref[:2], atol=1e-9)
            self.assertAlmostEqual(g.wrap(got[2] - ref[2]), 0., places=9)

    def test_equal_neighbour_stamps_accepted_decreasing_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'gt.txt'
            p.write_text('# ground truth\n1.0 0 0 0 0 0 0 1\n1.0 0 0 0 0 0 0 1\n2.0 1 0 0 0 0 0 1\n')
            self.assertEqual(len(g.load_rows(p)), 3)
            p.write_text('2.0 0 0 0 0 0 0 1\n1.0 0 0 0 0 0 0 1\n')
            with self.assertRaises(ValueError):
                g.load_rows(p)

    def test_bad_mount_refused(self):
        with self.assertRaises(ValueError):
            g.parse_tf('0 0 0 0 0 1')
        with self.assertRaises(ValueError):
            g.parse_tf('0 0 0 0 0 0 2')


def g_matrix_quat(R):
    """3x3 -> (x, y, z, w) through tools/nano_ros2/tum_mount.py (an existing, tested conversion)."""
    from tools.nano_ros2.tum_mount import matrix_quat
    return matrix_quat(R)


class PublisherTests(unittest.TestCase):
    def test_tf_sent_before_the_frame_images(self):
        src = PUB.read_text()
        loop = src[src.index('for k, ((rs, rname), (ds, dname)) in enumerate(pairs_sel):'):]
        self.assertLess(loop.index('tf_pub.publish('), loop.index('info_pub.publish(ci)'))
        self.assertLess(loop.index('tf_pub.publish('), loop.index('rgb_pub.publish(rm)'))
        # the TF carries the frame's own stamp (the one rgb / depth / camera_info get)
        self.assertIn('ts.header.stamp.sec, ts.header.stamp.nanosec = sec, nsec', loop)

    def test_off_by_default(self):
        src = PUB.read_text()
        self.assertIn("p.add_argument('--gt-odom-tf', metavar='MOUNT_TF',", src)
        self.assertIn('if gt_odom is not None and gt_odom.covers(rs):', src)
        self.assertIn('tf_pub = None', src)

    def test_pingpong_refused(self):
        with tempfile.TemporaryDirectory() as d:
            r = subprocess.run([sys.executable, str(PUB), d, '--out', os.path.join(d, 'o.jsonl'), '--pingpong', '2',
                                '--gt-odom-tf', MOUNT], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               universal_newlines=True)
            self.assertEqual(r.returncode, 2)
            self.assertIn('--gt-odom-tf does not go with --pingpong', r.stderr)


if __name__ == '__main__':
    unittest.main()
