"""10-28 search blind zone: a cell counts as looked at only when a small object on it is wholly in the image.

Car numbers (10-28 re-mount): camera 0.392 m high, 15.75 deg down, K fx = fy = 517.2, cx = 320, cy = 240.6, 640 x 480,
lens 0.10 m ahead of base_link. Known answers: the floor below the image's bottom edge (~0.46 m ahead of the lens)
is not searched; a view lowers what is 1 m ahead; the side bound is the real 31.7 deg half field, not 43 deg; a
steeper tilt pulls the blind zone in; the next look sits 0.9-1.2 m from the remembered place (not next to it); the
gain the planner chose a look for is what that look removes; the observation ring tries 0.9 / 1.2 m before 0.6 m and
a remembered view taken from close by is demoted with it. Host only, numpy, no ROS.
"""
import math
import os
import sys
import time
import unittest
from types import SimpleNamespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from host.search_belief import BeliefSearch, SearchConfig  # noqa: E402

RES = 0.05
K = np.array([[517.2, 0., 320.], [0., 517.2, 240.6], [0., 0., 1.]])
SIZE = (640, 480)


def mount(tilt_deg=15.75, height_m=0.392, x_m=0.10):
    """T_base_camera: optical (x right, y down, z forward) tilted down by tilt_deg."""
    r0 = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    t = math.radians(tilt_deg)
    ry = np.array([[math.cos(t), 0., math.sin(t)], [0., 1., 0.], [-math.sin(t), 0., math.cos(t)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = ry @ r0, [x_m, 0., height_m]
    return T


def room(w_m=6.0, h_m=5.0, ox=-1.0, oy=-2.5):
    cells = np.full((int(round(h_m / RES)), int(round(w_m / RES))), -1, np.int8)
    cells[2:-2, 2:-2] = 0
    cells[2:4, 2:-2] = cells[-4:-2, 2:-2] = cells[2:-2, 2:4] = cells[2:-2, -4:-2] = 100
    return SimpleNamespace(cells=cells, resolution=RES, origin_x=ox, origin_y=oy, origin_yaw=0.)


def near(b, x, y):
    return int(np.argmin((b.ux - x) ** 2 + (b.uy - y) ** 2))


def with_camera(b, tilt_deg=15.75):
    b.set_camera(K, SIZE, mount(tilt_deg))
    return b


class FrustumTests(unittest.TestCase):
    def setUp(self):
        self.b = with_camera(BeliefSearch(room(), [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]))
        self.b.begin(None)

    def test_mount_numbers(self):
        T = mount()
        self.assertAlmostEqual(math.degrees(math.asin(-T[2, 2])), 15.75, places=6)    # optical z points down 15.75
        # the image's bottom ray: 15.75 + atan(239.4 / 517.2) = 40.65 deg -> floor from 0.457 m ahead of the lens
        self.assertAlmostEqual(0.392 / math.tan(math.radians(15.75) + math.atan(239.4 / 517.2)), 0.457, delta=0.001)

    def test_blind_zone_and_side_bound(self):
        b = self.b
        T = b.camera_pose(0.0, 0.0, 0.0)
        cells = np.array([near(b, x, y) for x, y in ((0.3, 0.), (0.5, 0.), (0.8, 0.), (1.0, 0.), (1.4, 0.),
                                                     (1.1, 0.55), (1.1, 0.8))])
        seen = b.in_frame(T, cells).tolist()
        # lens at x = 0.1: 0.3 m base (0.2 lens) and 0.5 m base (object's near edge 0.35 m lens) below the image;
        # 0.8 / 1.0 / 1.4 in; 1.1 m ahead 0.55 m left (27 deg) in, 0.8 m left (36 deg > 31.7) out
        self.assertEqual(seen, [False, False, True, True, True, True, False])

    def test_steeper_tilt_pulls_the_blind_zone_in(self):
        b = self.b
        c = np.array([near(b, 0.65, 0.)])
        self.assertFalse(b.in_frame(_pose(mount(5.6)), c)[0])                      # old 5.6 deg mount: blind
        self.assertTrue(b.in_frame(_pose(mount(15.75)), c)[0])                     # 15.75 deg: in view

    def test_view_keeps_the_blind_zone_unsearched(self):
        b = self.b
        before = b.miss.copy()
        T = b.camera_pose(0.0, 0.0, 0.0)
        self.assertTrue(b.observe(T, 1.0, base_pose=(0., 0., 0.)))
        blind, ahead, wide = near(b, 0.35, 0.), near(b, 1.0, 0.), near(b, 1.0, 0.85)
        self.assertEqual(b.miss[blind], before[blind])           # used to be x 0.1 ("90 % not there")
        self.assertEqual(b.miss[wide], before[wide])             # 40 deg off axis: outside the real 31.7 deg
        self.assertAlmostEqual(b.miss[ahead], 1 - b.q(np.hypot(1.0 - T[0, 3], 0.))[()], places=6)

    def test_observe_refreshes_the_camera_model(self):
        b = BeliefSearch(room(), [(0.0, 0.0, 0.0)])
        b.begin(None)
        self.assertFalse(b.has_camera())
        T = mount()                                              # base at the origin facing +x
        b.observe(T, 1.0, K=K, image_size=SIZE, T_base_camera=mount())
        self.assertTrue(b.has_camera())
        self.assertEqual(b.miss[near(b, 0.35, 0.)], 1.0)


def _pose(T_base_camera, x=0., y=0., yaw=0.):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1.]]) @ T_base_camera


class SelectionTests(unittest.TestCase):
    NODES = [(0.25 * k - 0.5, 0.0, 0.0) for k in range(17)]          # x -0.5 .. 3.5 along y = 0
    TARGET = (1.5, -0.3)                                             # remembered 0.3 m beside the corridor
    ROBOT = (1.5, 0.0, 0.0)                                          # the car stands next to it (after reobserve)

    def plan(self, camera):
        b = BeliefSearch(room(), self.NODES, robot=self.ROBOT)
        if camera:
            with_camera(b)
        b.begin(self.TARGET)
        pose, info = b.next_goal()
        return b, pose, info

    def first_look_at_target(self, camera):
        """(look number, base pose) of the first look the planner counts as covering the remembered cell."""
        b = BeliefSearch(room(), self.NODES, robot=self.ROBOT)
        if camera:
            with_camera(b)
        b.begin(self.TARGET)
        c = near(b, *self.TARGET)
        for k in range(1, 13):
            pose, _info = b.next_goal()
            self.assertIsNotNone(pose)
            before = b.miss[c]
            b.apply_base_view(pose.x, pose.y, pose.yaw)
            b.robot = (pose.x, pose.y, pose.yaw)
            if b.miss[c] < before:
                return k, pose, b, c
        self.fail("remembered cell never covered")

    def test_remembered_place_is_looked_at_from_afar(self):
        _k, old, _b, _c = self.first_look_at_target(False)
        k, new, b, c = self.first_look_at_target(True)
        d_old = math.hypot(old.x - self.TARGET[0], old.y - self.TARGET[1])
        d_new = math.hypot(new.x - self.TARGET[0], new.y - self.TARGET[1])
        # old: "covered" from 0.3 m (the floor there is below the image); new: from 0.9-1.2 m, really in the image
        self.assertLess(d_old, 0.5, (old, d_old))
        self.assertTrue(0.8 <= d_new <= 1.3, (new, d_new))
        self.assertTrue(b.in_frame(b.camera_pose(new.x, new.y, new.yaw), np.array([c]))[0])

    def test_gain_is_what_the_look_removes(self):
        b, pose, info = self.plan(True)
        n, k = info["node"], b.yaw_set.index(pose.yaw)
        g0 = b.gains()[n, k]
        m0 = b.mass_left()
        b.apply_base_view(pose.x, pose.y, pose.yaw)
        self.assertAlmostEqual(m0 - b.mass_left(), g0, places=6)
        self.assertLess(b.gains()[n, k], 1e-9)

    def test_preview_uses_the_camera(self):
        b = with_camera(BeliefSearch(room(), self.NODES, robot=(-0.5, 0.0, 0.0)))
        b.begin(self.TARGET)
        rows = b.preview(max_goals=3)
        self.assertGreater(rows[0]["gain"], 0)
        self.assertLess(rows[-1].get("mass_after", 1.0), 1.0)

    def test_cost(self):
        """apply_view (1 Hz on the car) on a ~16 x 16 m grid with the frustum; printed, loose bound."""
        m = room(16.0, 16.0, -8.0, -8.0)
        nodes = [(0.5 * i - 6, 0.5 * j - 6, 0.) for i in range(25) for j in range(25) if (i + j) % 3 == 0]
        b = with_camera(BeliefSearch(m, nodes, SearchConfig(max_cells=60000)))
        b.begin(None)
        t0 = time.perf_counter()
        for k in range(20):
            b.apply_base_view(0.1 * k, 0., 0.3 * k)
        ms = (time.perf_counter() - t0) * 1e3 / 20
        t0 = time.perf_counter()
        b.gains()
        g_ms = (time.perf_counter() - t0) * 1e3
        print("\nfrustum apply_view %.2f ms over %d cells; gains %.0f ms over %d nodes" % (ms, b.ux.size, g_ms,
                                                                                            len(b.nodes)))
        self.assertLess(ms, 50)


class ObservationRingTests(unittest.TestCase):
    def setUp(self):
        try:
            from host import ros_runtime
        except Exception as exc:
            raise unittest.SkipTest("host.ros_runtime not importable: %s" % exc)
        self.rt = ros_runtime
        self.map = ros_runtime.FreeMap(np.zeros((160, 160), dtype=int), 0.05, -4.0, -4.0)

    def poses(self, preferred=()):
        pos = np.array([1.0, 0.0, 0.40])          # on a 0.35 m stool: the 0.6 m ring keeps it in the image
        cand = {"position_m": pos.tolist(),
                "bounds_m": [(pos - [0.05, 0.05, 0.05]).tolist(), (pos + [0.05, 0.05, 0.05]).tolist()]}
        return pos, self.rt.observation_poses(cand, self.map, K, SIZE, mount(), (1.5, 0.0),
                                              preferred_poses=preferred)

    def test_floor_object_0_6_ring_is_out_of_frame_already(self):
        pos = np.array([1.0, 0.0, 0.02])
        cand = {"position_m": pos.tolist(),
                "bounds_m": [(pos - [0.05, 0.05, 0.02]).tolist(), (pos + [0.05, 0.05, 0.08]).tolist()]}
        out = self.rt.observation_poses(cand, self.map, K, SIZE, mount(), (1.5, 0.0))
        self.assertTrue(out and all(math.hypot(p.x - 1.0, p.y) > 0.8 for p in out))

    def test_far_rings_first(self):
        pos, out = self.poses()
        d = [math.hypot(p.x - pos[0], p.y - pos[1]) for p in out]
        self.assertTrue(any(x < 0.8 for x in d), d)                     # the 0.6 m ring is still offered ...
        first_near = next(i for i, x in enumerate(d) if x < 0.8)
        self.assertTrue(all(x < 0.8 for x in d[first_near:]), d)         # ... but only after every far pose
        self.assertTrue(all(x >= 0.8 for x in d[:first_near]))
        far = [(p.x - 1.5) ** 2 + p.y ** 2 for p in out[:first_near]]
        self.assertEqual(far, sorted(far))                               # nearest to the car first within a group

    def test_close_remembered_view_is_demoted(self):
        close = self.rt.PoseGoal(1.6, 0.0, math.pi)                     # 0.6 m from the object, facing it
        far = self.rt.PoseGoal(2.0, 0.0, math.pi)                       # 1.0 m
        _pos, out = self.poses((close,))
        self.assertNotEqual(out[0], close)
        self.assertIn(close, out)
        _pos, out = self.poses((far,))
        self.assertEqual(out[0], far)


if __name__ == "__main__":
    unittest.main()
