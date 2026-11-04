"""Independent line/voxel-box oracle for M7; also checks the C ABI directly."""

import ctypes
from fractions import Fraction
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np

from host.m7_native import NativeM7, compile_native, GRID_CAPACITY, RAY_CAPACITY


ROOT = Path(__file__).resolve().parents[1]


def intersects_open_voxel(start, end, cell):
    """Analytic line versus open voxel interior, not a traversal algorithm.

    When an axis is constant exactly on a voxel plane, the half-open grid
    assigns the line to the positive-side cell (floor(x/256)). Any true
    corner/edge-only contact of a moving ray has zero time interval.
    """
    low_t, high_t = Fraction(0), Fraction(1)
    for axis in range(3):
        position, delta = int(start[axis]), int(end[axis]) - int(start[axis])
        low, high = int(cell[axis]) * 256, (int(cell[axis]) + 1) * 256
        if delta == 0:
            if position // 256 != cell[axis]:
                return False
            continue
        enter = Fraction(low - position, delta)
        leave = Fraction(high - position, delta)
        low_t = max(low_t, min(enter, leave))
        high_t = min(high_t, max(enter, leave))
    return low_t < high_t


def analytic_frame(starts, ends, shape):
    nx, ny, nz = shape
    expected = np.zeros((nz, ny, nx), dtype=np.uint8)
    for start, end in zip(starts, ends):
        origin_cell = tuple(int(x) // 256 for x in start)
        hit_cell = tuple(int(x) // 256 for x in end)
        for z in range(nz):
            for y in range(ny):
                for x in range(nx):
                    cell = (x, y, z)
                    if cell != origin_cell and cell != hit_cell and \
                            intersects_open_voxel(start, end, cell):
                        expected[z, y, x] = max(1, expected[z, y, x])
        expected[hit_cell[2], hit_cell[1], hit_cell[0]] = 2
    return expected


class NativeM7Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="m7-native-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.native = NativeM7(compile_native(Path(cls.tmp.name) / "libm7.so"))
        testbench = Path(cls.tmp.name) / "m7-testbench"
        subprocess.run(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                        str(ROOT / "hw/m7/ray_flags.cpp"),
                        str(ROOT / "hw/m7/testbench.cpp"), "-o", str(testbench)],
                       check=True, capture_output=True, text=True)
        cls.testbench = testbench

    def assert_oracle(self, starts, ends, shape=(4, 4, 4)):
        actual = self.native.trace(np.array(starts, dtype=np.int32),
                                   np.array(ends, dtype=np.int32), shape)
        expected = analytic_frame(starts, ends, shape)
        np.testing.assert_array_equal(actual, expected)
        return actual

    def test_standalone_cpp_testbench(self):
        result = subprocess.run([str(self.testbench)], check=True,
                                capture_output=True, text=True)
        self.assertIn("M7 ray flags C++ testbench PASS", result.stdout)

    def test_axes_reverse_diagonals_and_same_voxel(self):
        cases = [
            ([128, 128, 128], [3*256+128, 128, 128]),
            ([3*256+128, 128, 128], [128, 128, 128]),
            ([128, 128, 128], [3*256+128, 3*256+128, 128]),
            ([128, 128, 128], [3*256+128, 3*256+128, 3*256+128]),
            ([2*256+128, 128, 128], [2*256+129, 128, 128]),
            ([128, 128, 128], [128, 128, 128]),
            ([128, 256, 128], [3*256+128, 256, 128]),
            ([0, 0, 0], [3*256+255, 3*256+255, 3*256+255]),
            ([3*256+255, 3*256+255, 3*256+255], [0, 0, 0]),
        ]
        for start, end in cases:
            with self.subTest(start=start, end=end):
                self.assert_oracle([start], [end])

    def test_crossing_tie_only_marks_diagonal_cells(self):
        flags = self.assert_oracle([[128, 128, 128]],
                                   [[3*256+128, 3*256+128, 128]])
        self.assertEqual(flags[0, 0, 1], 0)
        self.assertEqual(flags[0, 1, 0], 0)
        self.assertEqual(flags[0, 1, 1], 1)
        self.assertEqual(flags[0, 3, 3], 2)

    def test_hit_wins_over_free_order_and_duplicates(self):
        starts = [[128,128,128], [128,128,128], [128,128,128]]
        ends = [[3*256+128,128,128], [256+128,128,128], [256+128,128,128]]
        forward = self.assert_oracle(starts, ends)
        reverse = self.assert_oracle(list(reversed(starts)), list(reversed(ends)))
        np.testing.assert_array_equal(forward, reverse)
        self.assertEqual(forward[0, 0, 1], 2)
        self.assertEqual(forward[0, 0, 2], 1)

    def test_boundary_parallel_and_negative_start_have_defined_half_open_ownership(self):
        self.assert_oracle([[256,128,128]], [[3*256+128,128,128]])
        reverse = self.assert_oracle([[2*256,128,128]], [[128,128,128]])
        self.assertEqual(reverse[0,0,2], 0)
        self.assertEqual(reverse[0,0,1], 1)
        self.assertEqual(reverse[0,0,0], 2)

    def test_independent_random_small_grid_known_analytic_answers(self):
        rng = np.random.default_rng(20260921)
        starts = rng.integers(0, 4*256, size=(24, 3), dtype=np.int32)
        ends = rng.integers(0, 4*256, size=(24, 3), dtype=np.int32)
        self.assert_oracle(starts, ends)
        # A frame union could hide a wrong traversal under other rays' HIT or
        # FREE votes. Check each independently against exact rational boxes.
        for start, end in zip(starts, ends):
            with self.subTest(start=start.tolist(), end=end.tolist()):
                self.assert_oracle([start], [end])
        for _ in range(96):
            start = rng.integers(0, 4*256, size=3, dtype=np.int32)
            end = rng.integers(0, 4*256, size=3, dtype=np.int32)
            self.assert_oracle([start], [end])

    def test_validity_capacity_and_atomic_failure(self):
        start = [[128,128,128], [-1,-1,-1]]
        end = [[3*256+128,128,128], [4*256,0,0]]
        actual = self.native.trace(start, end, (4,4,4), ray_valid=[1,0])
        np.testing.assert_array_equal(actual, analytic_frame(start[:1], end[:1], (4,4,4)))
        empty = self.native.trace(np.empty((0,3), dtype=np.int32),
                                  np.empty((0,3), dtype=np.int32), (4,4,4))
        self.assertFalse(np.any(empty))
        with self.assertRaisesRegex(ValueError, "4096"):
            self.native.trace(np.zeros((RAY_CAPACITY+1,3), np.int32),
                              np.zeros((RAY_CAPACITY+1,3), np.int32), (4,4,4))
        self.assert_atomic(2, starts=start, ends=end, validity=[1,2])
        self.assert_atomic(3, starts=start, ends=end, validity=[1,1])
        self.assert_atomic(1, starts=start, ends=end, validity=[1,0], nx=33)
        self.assert_atomic(1, starts=start, ends=end, validity=[1,0],
                           count=RAY_CAPACITY+1)

    def assert_atomic(self, expected_status, *, starts, ends, validity, nx=4,
                      count=None):
        a = np.zeros((RAY_CAPACITY,3), dtype=np.int32)
        b = np.zeros_like(a)
        v = np.zeros(RAY_CAPACITY, dtype=np.uint32)
        a[:len(starts)] = starts
        b[:len(ends)] = ends
        v[:len(validity)] = validity
        output = np.full(GRID_CAPACITY, 79, dtype=np.uint8)
        status = np.full(1, 99, dtype=np.uint32)
        self.native.lib.fpga_m7_ray_flags(a, b, v,
                                          len(starts) if count is None else count,
                                          nx, 4, 4,
                                          output, status)
        self.assertEqual(int(status[0]), expected_status)
        self.assertTrue(np.all(output == 79))

    def test_host_depth_projection_conservative_outside_and_metadata(self):
        transform = np.eye(4)
        transform[:3, 3] = [.5, .5, .5]
        k = np.diag([100., 100., 1.])
        common = dict(k=k, camera_to_world=transform, grid_origin_m=[0,0,0],
                      voxel_size_m=1., shape=(4,4,4), frame_id=7,
                      capture_time_ns=1000, pose_time_ns=998,
                      calibration_version="intrinsics-a", pose_confirmed=True)
        frame = self.native.frame_from_depth(depth_m=[[1., 9., np.nan]], **common)
        self.assertEqual((frame.accepted_rays, frame.outside_grid,
                          frame.invalid_depth_or_projection), (1, 1, 1))
        self.assertEqual((frame.frame_id, frame.capture_time_ns,
                          frame.pose_time_ns, frame.calibration_version),
                         (7, 1000, 998, "intrinsics-a"))
        self.assertEqual(frame.quantization_axis_error_bound_m, 1/256)
        self.assertEqual(frame.flags[1,0,0], 2)
        self.assertEqual(frame.flags[0,0,0], 0)
        self.assertFalse(np.any(frame.flags == 1))
        with self.assertRaisesRegex(ValueError, "trusted"):
            self.native.frame_from_depth(depth_m=[[1.]], **dict(common, pose_confirmed=False))

    def test_host_quantized_boundary_does_not_pretend_continuous_accuracy(self):
        transform = np.eye(4)
        transform[:3,3] = [.5, .5, .5]
        common = dict(k=np.diag([100.,100.,1.]), camera_to_world=transform,
                      grid_origin_m=[0,0,0], voxel_size_m=1., shape=(4,4,4),
                      frame_id=1, capture_time_ns=1, pose_time_ns=1,
                      calibration_version="a", pose_confirmed=True)
        lower = np.nextafter(np.float32(1.5), np.float32(0))
        before = self.native.frame_from_depth(depth_m=[[lower]], **common)
        at = self.native.frame_from_depth(depth_m=[[np.float32(1.5)]], **common)
        self.assertEqual(before.flags[1,0,0], 2)
        self.assertEqual(at.flags[2,0,0], 2)
        self.assertNotEqual(np.argmax(before.flags), np.argmax(at.flags))

    def test_cross_tile_hit_precedence_and_frame_counts(self):
        transform = np.eye(4)
        transform[:3,3] = [.5,.5,.5]
        common = dict(k=np.diag([1_000_000.,1_000_000.,1.]),
                      camera_to_world=transform, grid_origin_m=[0,0,0],
                      voxel_size_m=1., shape=(4,4,4), frame_id=5,
                      capture_time_ns=5, pose_time_ns=5,
                      calibration_version="a", pose_confirmed=True)
        later_hit = np.full((1, RAY_CAPACITY+1), 3., dtype=np.float32)
        later_hit[0,-1] = 1.
        earlier_hit = np.full((RAY_CAPACITY+1, 1), 3., dtype=np.float32)
        earlier_hit[0,0] = 1.
        for depth in (later_hit, earlier_hit):
            with self.subTest(shape=depth.shape):
                frame = self.native.frame_from_depth(depth_m=depth, **common)
                self.assertEqual(frame.flags[1,0,0], 2)
                self.assertEqual(frame.flags[2,0,0], 1)
                self.assertEqual(frame.flags[3,0,0], 2)
                self.assertEqual(frame.accepted_rays + frame.outside_grid +
                                 frame.invalid_depth_or_projection, depth.size)

    def test_invalid_intrinsics_and_nonrigid_pose_rejected(self):
        base = dict(depth_m=[[1.]], k=np.eye(3), camera_to_world=np.eye(4),
                    grid_origin_m=[-1,-1,-1], voxel_size_m=1., shape=(4,4,4),
                    frame_id=1, capture_time_ns=1, pose_time_ns=1,
                    calibration_version="a", pose_confirmed=True)
        bad_k = np.eye(3)
        bad_k[0,0] = -1
        with self.assertRaisesRegex(ValueError, "nonphysical"):
            self.native.frame_from_depth(**dict(base, k=bad_k))
        bad_pose = np.eye(4)
        bad_pose[0,0] = -1
        with self.assertRaisesRegex(ValueError, "nonphysical"):
            self.native.frame_from_depth(**dict(base, camera_to_world=bad_pose))


if __name__ == "__main__":
    unittest.main()
