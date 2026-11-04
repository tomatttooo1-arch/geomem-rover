"""Full-depth-frame CPU ABI versus the frozen two-stage M7 reference.

Since docs/design/M7_CLIP_SPEC_20261011.md the frame kernel WALKS rays whose
endpoint is outside the grid (clipped) while the frozen two-stage NativeM7
still drops them. Counts are classified identically by both, so counts are
always compared with NativeM7; flags are compared with NativeM7 only when no
ray was clipped, otherwise with the independent golden tools/m7_clip/golden.py.
"""

import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np

from host.m7_frame_native import (FrameNativeM7, MAX_PIXELS,
                                  compile_frame_native)
from host.m7_native import NativeM7, compile_native
from tools.m7_clip.golden import golden_frame


ROOT = Path(__file__).resolve().parents[1]
# Reuse the PRE-EXISTING exact rational line/voxel-box oracle. That test
# module is imported for its function, not for its own test runner.
spec = importlib.util.spec_from_file_location(
    "existing_m7_ray_oracle", ROOT / "tests/test_m7_native.py")
ray_oracle_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ray_oracle_module)
analytic_frame = ray_oracle_module.analytic_frame


class FrameNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="m7-frame-")
        cls.addClassCleanup(cls.tmp.cleanup)
        directory = Path(cls.tmp.name)
        cls.frame = FrameNativeM7(compile_frame_native(directory / "frame.so"))
        cls.old = NativeM7(compile_native(directory / "old.so"))
        cls.testbench = directory / "frame-testbench"
        subprocess.run(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-ffp-contract=off", str(ROOT / "hw/m7/frame_depth.cpp"),
                        str(ROOT / "hw/m7/frame_testbench.cpp"),
                        "-o", str(cls.testbench)], check=True,
                       capture_output=True, text=True)

    @staticmethod
    def common(*, grid_origin=(0., 0., 0.), voxel=1., shape=(4,4,4),
               k=None, transform=None):
        if k is None:
            k = np.diag([1_000_000., 1_000_000., 1.])
        if transform is None:
            transform = np.eye(4)
            transform[:3,3] = [.5,.5,.5]
        return dict(k=k, camera_to_world=transform,
                    grid_origin_m=grid_origin, voxel_size_m=voxel,
                    shape=shape, frame_id=6, capture_time_ns=110,
                    pose_time_ns=109, calibration_version="cal-a",
                    pose_confirmed=True)

    def compare(self, depth, **kwargs):
        params = self.common(**kwargs)
        current = self.frame.frame_from_depth(depth_m=depth, **params)
        former = self.old.frame_from_depth(depth_m=depth, **params)
        if former.outside_grid == 0:
            np.testing.assert_array_equal(current.flags, former.flags)
        else:
            nx, ny, nz = params["shape"]
            inverse_k = np.linalg.inv(np.asarray(params["k"], dtype=np.float64)).astype(np.float32)
            pose = np.asarray(params["camera_to_world"], dtype=np.float64)[:3].astype(np.float32)
            golden, golden_counts = golden_frame(
                np.asarray(depth, dtype=np.float32).reshape(np.shape(depth)), inverse_k, pose,
                params["grid_origin_m"], params["voxel_size_m"], (nx, ny, nz))
            np.testing.assert_array_equal(current.flags.ravel(), golden)
            self.assertEqual(golden_counts, (current.accepted_rays,
                                             current.invalid_depth_or_projection,
                                             current.outside_grid))
        self.assertEqual((current.accepted_rays,
                          current.invalid_depth_or_projection,
                          current.outside_grid),
                         (former.accepted_rays,
                          former.invalid_depth_or_projection,
                          former.outside_grid))
        self.assertEqual(current.accepted_rays + current.invalid_depth_or_projection +
                         current.outside_grid, np.asarray(depth).size)
        self.assertEqual((current.frame_id, current.capture_time_ns,
                          current.pose_time_ns, current.calibration_version),
                         (6, 110, 109, "cal-a"))
        return current

    def test_cpp_testbench(self):
        result = subprocess.run([str(self.testbench)], check=True,
                                capture_output=True, text=True)
        self.assertIn("M7 depth frame C++ testbench PASS", result.stdout)

    def test_known_small_frame_and_existing_analytic_oracle(self):
        frame = self.compare([[3., 1., np.nan]])
        expected = analytic_frame([[128,128,128], [128,128,128]],
                                  [[128,128,896], [128,128,384]], (4,4,4))
        np.testing.assert_array_equal(frame.flags, expected)
        self.assertEqual((frame.accepted_rays, frame.invalid_depth_or_projection,
                          frame.outside_grid), (2, 1, 0))
        self.assertEqual(frame.flags[1,0,0], 2)
        self.assertEqual(frame.flags[2,0,0], 1)

    def test_rotation_translation_skew_and_projection_outside(self):
        angle = .37
        transform = np.eye(4)
        transform[:3,:3] = [[np.cos(angle),0,np.sin(angle)], [0,1,0],
                            [-np.sin(angle),0,np.cos(angle)]]
        transform[:3,3] = [.5,-.25,.8]
        k = np.array([[70.,3.,6.], [0.,73.,5.], [0.,0.,1.]])
        rng = np.random.default_rng(20260921)
        depth = rng.uniform(.11, 9.9, size=(12,16)).astype(np.float32)
        depth.flat[::17] = np.nan
        depth.flat[::31] = 10.
        result = self.compare(depth, k=k, transform=transform,
                              grid_origin=(-2.,-2.,-1.), voxel=.25,
                              shape=(32,32,32))
        self.assertGreater(result.invalid_depth_or_projection, 0)
        self.assertGreater(result.outside_grid, 0)

    def test_quantized_boundary_and_outside_is_clipped(self):
        low = np.nextafter(np.float32(1.5), np.float32(0))
        before = self.compare([[low]])
        at = self.compare([[np.float32(1.5)]])
        self.assertEqual(before.flags[1,0,0], 2)
        self.assertEqual(at.flags[2,0,0], 2)
        # The ray from z=0.5 to z=9.5 leaves the 4-voxel grid: it is counted
        # as clipped, its in-grid voxels z=1..3 are FREE and nothing is HIT
        # (the camera voxel z=0 is never cleared). The 09-21 rule dropped it.
        outside = self.compare([[9.]])
        self.assertEqual(outside.outside_grid, 1)
        self.assertEqual(outside.accepted_rays, 0)
        self.assertEqual(sorted(np.argwhere(outside.flags).tolist()),
                         [[1, 0, 0], [2, 0, 0], [3, 0, 0]])
        self.assertTrue(np.all(outside.flags[outside.flags != 0] == 1))

    def test_highest_grid_address_and_full_grid_reset(self):
        transform = np.eye(4)
        transform[:3,3] = [31.5, 31.5, 30.5]
        occupied = self.compare([[1.]], transform=transform, shape=(32,32,32))
        self.assertEqual(occupied.flags[-1,-1,-1], 2)
        cleared = self.compare(np.empty((0,0), dtype=np.float32),
                               transform=transform, shape=(32,32,32))
        self.assertFalse(np.any(cleared.flags))

    def test_cross_4096_boundary_both_hit_orders_and_reset(self):
        depth = np.full((64,128), np.nan, dtype=np.float32)
        depth.flat[0], depth.flat[4096] = 3., 1.
        later_hit = self.compare(depth)
        self.assertEqual(later_hit.flags[1,0,0], 2)
        self.assertEqual(later_hit.flags[2,0,0], 1)
        depth.flat[0], depth.flat[4096] = 1., 3.
        earlier_hit = self.compare(depth)
        np.testing.assert_array_equal(later_hit.flags, earlier_hit.flags)
        empty = self.compare(np.full((1,1), np.nan, dtype=np.float32))
        self.assertFalse(np.any(empty.flags))
        self.assertEqual(empty.invalid_depth_or_projection, 1)
        raw_empty = self.compare(np.empty((0,0), dtype=np.float32))
        self.assertFalse(np.any(raw_empty.flags))
        self.assertEqual(raw_empty.accepted_rays, 0)

    def test_full_capacity_last_pixel_and_every_pixel_valid(self):
        sparse = np.full((800,1280), np.nan, dtype=np.float32)
        sparse[-1,-1] = 1.
        last = self.compare(sparse)
        self.assertEqual(last.accepted_rays, 1)
        self.assertEqual(last.invalid_depth_or_projection, MAX_PIXELS - 1)
        dense = np.ones((800,1280), dtype=np.float32)
        full = self.compare(dense)
        self.assertEqual(full.accepted_rays, MAX_PIXELS)
        self.assertEqual(full.invalid_depth_or_projection, 0)
        self.assertEqual(full.flags[1,0,0], 2)

    def single_ray_frame(self, origin_q8, end_q8, shape=(4, 4, 4)):
        """Drive the frame top with ONE ray between two exact Q8 points.

        The camera sits at origin_q8/256 metres and the depth is exactly 1 m,
        so inverse K's last column carries the camera-space point directly.
        Every quantity here is a multiple of 2**-8 below 4, hence exact in
        float32 and in the float64 floor: the kernel sees precisely end_q8.
        """
        nx, ny, nz = shape
        depth = np.zeros(MAX_PIXELS, dtype=np.float32)
        depth[0] = 1.
        inverse_k = np.zeros((3, 3), dtype=np.float32)
        inverse_k[:, 2] = [(end - start) / 256.
                           for start, end in zip(origin_q8, end_q8)]
        transform = np.zeros((3, 4), dtype=np.float32)
        transform[:, :3] = np.eye(3)
        transform[:, 3] = [start / 256. for start in origin_q8]
        grid_origin = np.zeros(3, dtype=np.float64)
        flags = np.full(32 ** 3, 99, dtype=np.uint8)
        counts = np.full(3, 99, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        self.frame.lib.fpga_m7_depth_frame(depth, inverse_k, transform,
                                          grid_origin, 1., 1, 1, nx, ny, nz,
                                          flags, counts, status)
        self.assertEqual((int(status[0]), tuple(int(c) for c in counts)),
                         (0, (1, 0, 0)))
        return flags[:nx * ny * nz].reshape(nz, ny, nx).copy()

    def test_simultaneous_axis_crossing_marks_only_the_diagonal_cells(self):
        # Nothing else in this suite puts a ray exactly on a voxel edge: the
        # other cases deliberately keep endpoints at voxel centres reached one
        # axis at a time. This ray runs from the centre of (0,0,0) to the
        # centre of (3,3,3) and meets all three planes at the same instants,
        # so the half-open convention observes only the diagonal cells as free.
        # Replacing the simultaneous-axis step in frame_depth.cpp with a
        # classic single-axis DDA marks six extra face neighbours here.
        depth = np.full((2, 2), np.nan, dtype=np.float32)
        depth[1, 1] = 3.
        frame = self.compare(depth, k=np.eye(3))
        expected = analytic_frame([[128, 128, 128]], [[896, 896, 896]],
                                  (4, 4, 4))
        np.testing.assert_array_equal(frame.flags, expected)
        self.assertEqual(sorted(np.argwhere(frame.flags).tolist()),
                         [[1, 1, 1], [2, 2, 2], [3, 3, 3]])
        self.assertEqual((frame.flags[1, 1, 1], frame.flags[2, 2, 2],
                          frame.flags[3, 3, 3]), (1, 1, 2))
        self.assertEqual((frame.accepted_rays,
                          frame.invalid_depth_or_projection,
                          frame.outside_grid), (1, 3, 0))

    def test_frame_and_ray_kernels_share_one_traversal_rule(self):
        # frame_depth.cpp and ray_flags.cpp carry two independent copies of
        # the same half-open, simultaneous-axis traversal. Nothing else
        # asserts they agree, so one copy could be edited alone and drift.
        cases = [((128, 128, 128), (896, 896, 896)),   # three-axis tie
                 ((128, 128, 128), (896, 896, 128)),   # two-axis tie, z fixed
                 ((128, 128, 128), (128, 896, 896)),   # two-axis tie, x fixed
                 ((896, 896, 896), (128, 128, 128)),   # reversed three-axis tie
                 ((256, 256, 128), (768, 768, 128)),   # tie starting on a plane
                 ((256, 128, 128), (896, 384, 128)),   # plane origin, no tie
                 ((128, 128, 128), (896, 128, 128)),   # single axis
                 ((128, 128, 128), (871, 405, 617))]   # generic, no tie
        for origin_q8, end_q8 in cases:
            with self.subTest(origin=origin_q8, end=end_q8):
                frame_flags = self.single_ray_frame(origin_q8, end_q8)
                ray_flags = self.old.trace([list(origin_q8)], [list(end_q8)],
                                           (4, 4, 4))
                np.testing.assert_array_equal(frame_flags, ray_flags)
                np.testing.assert_array_equal(
                    frame_flags, analytic_frame([list(origin_q8)],
                                                [list(end_q8)], (4, 4, 4)))

    def single_ray_counts(self, origin_q8, end_q8, shape=(4, 4, 4)):
        nx, ny, nz = shape
        depth = np.zeros(MAX_PIXELS, dtype=np.float32)
        depth[0] = 1.
        inverse_k = np.zeros((3, 3), dtype=np.float32)
        inverse_k[:, 2] = [(end - start) / 256. for start, end in zip(origin_q8, end_q8)]
        transform = np.zeros((3, 4), dtype=np.float32)
        transform[:, :3] = np.eye(3)
        transform[:, 3] = [start / 256. for start in origin_q8]
        flags = np.full(32 ** 3, 99, dtype=np.uint8)
        counts = np.full(3, 99, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        self.frame.lib.fpga_m7_depth_frame(depth, inverse_k, transform,
                                          np.zeros(3, dtype=np.float64), 1., 1, 1,
                                          nx, ny, nz, flags, counts, status)
        self.assertEqual(int(status[0]), 0)
        return flags[:nx * ny * nz].reshape(nz, ny, nx).copy(), tuple(int(c) for c in counts)

    def test_clipped_rays_negative_floor_corner_and_far(self):
        # Clipped endpoints: Q8 in (-256, 0) must floor to cell -1 (C '/'
        # would give cell 0 and invent a HIT inside the grid), a ray leaving
        # exactly through a grid corner, and a 200-voxel-long ray.
        cases = [((128, 128 + 512, 128), (-1, 128 + 512, 896)),
                 ((640, 640, 640), (-255, 640, 640)),
                 ((640, 640, 640), (-256, 640, 640)),
                 ((640, 640, 640), (1024, 640, 640)),          # exactly at x = 4 voxels
                 ((640, 640, 640), (1664, 1664, 1664)),        # through the (4,4,4) corner
                 ((128, 128, 128), (51200, 9000, 30000)),      # ~200 voxels away
                 ((896, 384, 640), (-40000, -51200, 7))]
        for origin_q8, end_q8 in cases:
            with self.subTest(origin=origin_q8, end=end_q8):
                flags, counts = self.single_ray_counts(origin_q8, end_q8)
                self.assertEqual(counts, (0, 0, 1))
                self.assertFalse(np.any(flags == 2))
                inverse_k = np.zeros((3, 3), dtype=np.float32)
                inverse_k[:, 2] = [(e - o) / 256. for o, e in zip(origin_q8, end_q8)]
                pose = np.zeros((3, 4), dtype=np.float32)
                pose[:, :3] = np.eye(3)
                pose[:, 3] = [o / 256. for o in origin_q8]
                golden, golden_counts = golden_frame(np.ones((1, 1), np.float32), inverse_k, pose,
                                                     (0., 0., 0.), 1., (4, 4, 4))
                np.testing.assert_array_equal(flags.ravel(), golden)
                self.assertEqual(golden_counts, counts)

    def test_q8_range_gate_counts_invalid(self):
        # A finite endpoint whose Q8 coordinate is >= 2^24 cannot be walked
        # exactly in int32; it is counted invalid (M7_CLIP_SPEC_20261011 §3).
        flags, counts = self.single_ray_counts((128, 128, 128), (128, 128, 1 << 24))
        self.assertEqual(counts, (0, 1, 0))
        self.assertFalse(np.any(flags))
        flags, counts = self.single_ray_counts((128, 128, 128), (128, 128, (1 << 24) - 256))
        self.assertEqual(counts, (0, 0, 1))
        self.assertEqual(sorted(np.argwhere(flags).tolist()), [[1, 0, 0], [2, 0, 0], [3, 0, 0]])

    def test_direct_abi_atomic_errors_and_host_structure(self):
        depth = np.zeros(MAX_PIXELS, dtype=np.float32)
        inverse_k = np.eye(3, dtype=np.float32)
        transform = np.ascontiguousarray(self.common()["camera_to_world"][:3],
                                          dtype=np.float32)
        grid_origin = np.zeros(3, dtype=np.float64)
        for width, height, voxel, pose_x, expected in (
                (1281,1,1.,.5,1), (1,1,0.,.5,2), (1,1,1.,5.,3)):
            with self.subTest(status=expected):
                t = transform.copy()
                t[0,3] = pose_x
                flags = np.full(32**3, 79, dtype=np.uint8)
                counts = np.full(3, 99, dtype=np.uint32)
                status = np.full(1, 99, dtype=np.uint32)
                self.frame.lib.fpga_m7_depth_frame(
                    depth, inverse_k, t, grid_origin, voxel,
                    width, height, 4,4,4, flags, counts, status)
                self.assertEqual(int(status[0]), expected)
                self.assertTrue(np.all(flags == 79))
                self.assertTrue(np.all(counts == 0))
        base = self.common()
        with self.assertRaisesRegex(ValueError, "1280x800"):
            self.frame.frame_from_depth(depth_m=np.ones((1,1281)), **base)
        bad_k = base["k"].copy()
        bad_k[1,0] = 1.
        with self.assertRaisesRegex(ValueError, "nonphysical"):
            self.frame.frame_from_depth(depth_m=[[1.]], **dict(base, k=bad_k))
        improper = base["camera_to_world"].copy()
        improper[0,0] = -1.
        with self.assertRaisesRegex(ValueError, "nonphysical"):
            self.frame.frame_from_depth(depth_m=[[1.]],
                                        **dict(base, camera_to_world=improper))
        with self.assertRaisesRegex(ValueError, "trusted"):
            self.frame.frame_from_depth(depth_m=[[1.]],
                                        **dict(base, pose_confirmed=False))


if __name__ == "__main__":
    unittest.main()
