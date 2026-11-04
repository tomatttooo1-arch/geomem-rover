"""Pure helper calibration; these tests do not claim a running ROS/Nav2 graph."""

import ast
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from host.ros_runtime import (AcquisitionQueue, FreeMap, MotionPermit, image_array, load_waypoints,
                              observation_poses, transform_matrix, use_simulation_clock)


class ROSHelpersTests(unittest.TestCase):
    def test_pending_tf_keeps_original_acquisition_and_only_latest_successor(self):
        pending = AcquisitionQueue(timeout=.5)
        self.assertEqual(pending.push(10., "frame-10", 100.), 0)
        self.assertEqual(pending.push(10.1, "frame-10.1", 100.1), 0)
        self.assertEqual(pending.push(10.2, "frame-10.2", 100.2), 1)
        self.assertEqual(pending.current.payload, "frame-10")
        self.assertEqual(pending.latest.payload, "frame-10.2")
        self.assertFalse(pending.expired(100.49))
        # New frames cannot extend an unresolved old timestamp's deadline.
        self.assertTrue(pending.expired(100.5))
        pending.pop()
        self.assertEqual(pending.current.stamp, 10.2)
        self.assertEqual(pending.current.received_at, 100.2)
        self.assertTrue(pending.expired(100.71))

    def test_clock_tf_arrival_before_deadline_can_release_exact_frame(self):
        pending = AcquisitionQueue()
        pending.push(4., "rgb4", 20.)
        self.assertEqual(pending.push(3., "stale", 20.1), 1)
        self.assertFalse(pending.expired(20.2))
        # Model the external lookup becoming ready, without replacing stamp with latest.
        self.assertEqual(pending.current.stamp, 4.)
        pending.pop()
        self.assertIsNone(pending.current)

    def test_motion_permission_requires_live_navigation_and_all_health_inputs(self):
        permit = MotionPermit()
        healthy = dict(navigating=True, cancelled=False, processed_age=.1,
                       acquisition_age=.1, pose_valid=True, inference_healthy=True)
        self.assertFalse(permit.allowed(**healthy))
        permit.arm()
        self.assertTrue(permit.allowed(**healthy))  # positive calibration
        for changes in ({"navigating": False}, {"cancelled": True}, {"processed_age": 2.1},
                        {"acquisition_age": 1.1}, {"acquisition_age": -.1},
                        {"processed_age": float("nan")}, {"pose_valid": False},
                        {"inference_healthy": False}):
            with self.subTest(changes=changes):
                self.assertFalse(permit.allowed(**{**healthy, **changes}))

    def test_old_result_cannot_resume_motion_after_fault_or_cancel(self):
        permit = MotionPermit()
        healthy = dict(navigating=True, cancelled=False, processed_age=.1,
                       acquisition_age=.1, pose_valid=True, inference_healthy=True)
        permit.arm()
        self.assertTrue(permit.allowed(**healthy))
        permit.stop()
        # Even if a pending worker completes, its fresh frame cannot release the latch.
        self.assertFalse(permit.allowed(**healthy))
        permit.arm()  # an explicit new navigation may release it
        self.assertTrue(permit.allowed(**healthy))

    def test_clock_mode_is_enforced_independently_of_provenance_labels(self):
        self.assertTrue(use_simulation_clock("simulation", False))
        self.assertTrue(use_simulation_clock("simulation", True))
        self.assertFalse(use_simulation_clock("hardware", False))
        with self.assertRaises(ValueError):
            use_simulation_clock("hardware", True)

    def test_padded_color_and_big_endian_depth(self):
        image = SimpleNamespace(encoding="bgr8", is_bigendian=False, width=1, height=2,
                                step=4, data=bytes([1, 2, 3, 99, 4, 5, 6, 98]))
        np.testing.assert_array_equal(image_array(image), [[[3, 2, 1]], [[6, 5, 4]]])
        values = np.array([[1.25, 2.5], [3.75, 4.]], dtype=">f4")
        rows = b"".join(row.tobytes()+b"padding!" for row in values)
        depth = SimpleNamespace(encoding="32FC1", is_bigendian=True, width=2, height=2,
                                step=16, data=rows)
        np.testing.assert_array_equal(image_array(depth), values)

    def test_wrong_encoding_stride_and_payload_rejected(self):
        valid = dict(encoding="rgb8", is_bigendian=False, width=1, height=1, step=3, data=b"123")
        for changes in ({"encoding": "mono8"}, {"step": 2}, {"data": b"12"}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                image_array(SimpleNamespace(**{**valid, **changes}))

    def test_map_free_occupied_unknown_outside_and_rotation(self):
        cells = np.zeros((20, 20), dtype=np.int8)
        grid = FreeMap(cells, .1, -1., -1.)
        self.assertTrue(grid.free(0, 0, .2))
        cells[10, 10] = -1
        self.assertFalse(grid.free(0, 0, .2))
        cells[10, 10] = 100
        self.assertFalse(grid.free(0, 0, .2))
        self.assertFalse(grid.free(1.1, 0, 0))
        self.assertFalse(grid.free(.9, 0, .3))
        rotated = FreeMap(np.zeros((10, 20)), .1, 0, 0, np.pi/2)
        self.assertTrue(rotated.free(-.5, 1., .1))
        self.assertFalse(rotated.free(.5, 1., .1))

    def test_transform_known_translation_rotation_and_invalid_quaternion(self):
        transform = SimpleNamespace(translation=SimpleNamespace(x=1., y=2., z=3.),
                                    rotation=SimpleNamespace(x=0., y=0., z=np.sqrt(.5), w=np.sqrt(.5)))
        result = transform_matrix(transform)
        np.testing.assert_allclose(result @ [1, 0, 0, 1], [1, 3, 3, 1], atol=1e-12)
        transform.rotation.z = transform.rotation.w = 0
        with self.assertRaises(ValueError):
            transform_matrix(transform)

    def test_observation_goals_pass_footprint_and_fov_without_object_coordinates_as_goal(self):
        grid = FreeMap(np.zeros((80, 80)), .1, -4, -4)
        K = np.array([[277., 0, 160], [0, 277., 120], [0, 0, 1.]])
        base_camera = np.array([[0, 0, 1, .18], [-1, 0, 0, 0], [0, -1, 0, .55], [0, 0, 0, 1.]])
        target = {"position_m": [1., 0, .55], "bounds_m": [[.95, -.05, .5], [1.05, .05, .6]]}
        poses = observation_poses(target, grid, K, (320, 240), base_camera, (0, 0))
        self.assertTrue(poses)
        self.assertTrue(all(grid.free(p.x, p.y) for p in poses))
        self.assertTrue(all(np.hypot(p.x-1., p.y) >= .59 for p in poses))
        occupied = FreeMap(np.full((80, 80), 100), .1, -4, -4)
        self.assertEqual(observation_poses(target, occupied, K, (320, 240), base_camera, (0, 0)), [])
        high = {"position_m": [1., 0, 10.], "bounds_m": [[.95, -.05, 9.95], [1.05, .05, 10.05]]}
        self.assertEqual(observation_poses(high, grid, K, (320, 240), base_camera, (0, 0)), [])

    def test_remembered_view_is_preferred_but_cannot_bypass_map_or_visibility(self):
        from host.task_control import PoseGoal
        grid = FreeMap(np.zeros((80, 80)), .1, -4, -4)
        K = np.array([[277., 0, 160], [0, 277., 120], [0, 0, 1.]])
        camera = np.array([[0, 0, 1, .18], [-1, 0, 0, 0], [0, -1, 0, .55], [0, 0, 0, 1.]])
        target = {"position_m": [1., 0, .55], "bounds_m": [[.95, -.05, .5], [1.05, .05, .6]]}
        good, backwards = PoseGoal(-1, 0, 0), PoseGoal(0, 0, np.pi)
        poses = observation_poses(target, grid, K, (320, 240), camera, (0, 0), preferred_poses=(good, backwards))
        self.assertEqual(poses[0], good)
        self.assertNotIn(backwards, poses)
        grid.cells[40, 30] = 100  # the previously visited place is now occupied
        poses = observation_poses(target, grid, K, (320, 240), camera, (0, 0), preferred_poses=(good,))
        self.assertNotIn(good, poses)
        self.assertTrue(poses)  # ring alternatives remain available

    def test_coverage_waypoints_do_not_need_scene_objects_and_reject_invalid_file(self):
        points = load_waypoints()
        self.assertTrue(points)
        self.assertTrue(all(p.frame == "map" for p in points))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "waypoints.json"
            path.write_text('[[1,2,0]]')
            self.assertEqual(load_waypoints(path)[0].x, 1)
            path.write_text('[[1,2]]')
            with self.assertRaises(ValueError):
                load_waypoints(path)

    def test_python38_syntax_for_runtime_and_controller(self):
        root = Path(__file__).resolve().parents[1]
        for path in [root / "host/task_control.py", root / "host/ros_runtime.py"]:
            ast.parse(path.read_text(), feature_version=(3, 8))


if __name__ == "__main__":
    unittest.main()
