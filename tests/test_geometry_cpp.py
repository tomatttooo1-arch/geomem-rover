"""Host compiler checks only: these tests do not simulate generated RTL."""
import ctypes
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

from perception.fpga_replay.core import Config, Frame, Memory, Track, backproject, compute_geometry


CAPACITY = 4096
ROOT = Path(__file__).resolve().parents[1]
F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")


class GeometryCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("g++")
        if compiler is None:
            raise RuntimeError("g++ is required for geometry verification; no silent skip")
        cls.tmp = tempfile.TemporaryDirectory(prefix="fpga-geometry-")
        cls.addClassCleanup(cls.tmp.cleanup)
        source = ROOT / "hw/geometry"
        library = Path(cls.tmp.name) / "geometry.so"
        flags = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off"]
        subprocess.run([compiler, *flags, "-fPIC", "-shared", str(source / "geometry.cpp"),
                        "-o", str(library)], check=True, capture_output=True, text=True)
        cls.lib = ctypes.CDLL(str(library))
        cls.lib.fpga_backproject.argtypes = [F32, F32, F32, ctypes.c_uint32, F32, U32, U32]
        cls.lib.fpga_backproject.restype = None
        cls.lib.fpga_visibility_counts.argtypes = [F32, ctypes.c_uint32, ctypes.c_float,
                                                 ctypes.c_float, U32, U32]
        cls.lib.fpga_visibility_counts.restype = None
        cls.testbench = Path(cls.tmp.name) / "testbench"
        subprocess.run([compiler, *flags, str(source / "geometry.cpp"),
                        str(source / "testbench.cpp"), "-o", str(cls.testbench)],
                       check=True, capture_output=True, text=True)

    def points(self, points, k, transform, count=None):
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        data = np.zeros((CAPACITY, 3), dtype=np.float32)
        data[:len(points)] = points
        out = np.full_like(data, 99)
        valid = np.full(CAPACITY, 99, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        self.lib.fpga_backproject(data, np.linalg.inv(k).astype(np.float32),
                                 np.ascontiguousarray(transform[:3], dtype=np.float32),
                                 len(points) if count is None else count, out, valid, status)
        return out, valid, int(status[0])

    def counts(self, depth, front=1.5, back=2.5, count=None):
        depth = np.asarray(depth, dtype=np.float32).ravel()
        data = np.zeros(CAPACITY, dtype=np.float32)
        data[:len(depth)] = depth
        out, status = np.full(4, 99, dtype=np.uint32), np.full(1, 99, dtype=np.uint32)
        self.lib.fpga_visibility_counts(data, len(depth) if count is None else count,
                                       front, back, out, status)
        return out, int(status[0])

    def test_identity_and_rotation_translation_have_known_answers(self):
        k = np.array([[100., 0, 50], [0, 100, 50], [0, 0, 1]])
        points = np.array([[50, 50, 2], [100, 50, 2]])
        t = np.eye(4)
        out, valid, status = self.points(points, k, t)
        self.assertEqual(status, 0)
        np.testing.assert_array_equal(valid[:2], [1, 1])
        np.testing.assert_allclose(out[:2], [[0, 0, 2], [1, 0, 2]], atol=1e-6)
        t[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        t[:3, 3] = [1, 2, 3]
        changed, valid, status = self.points(points, k, t)
        np.testing.assert_allclose(changed[:2], [[1, 2, 5], [1, 3, 5]], atol=1e-6)
        self.assertFalse(np.allclose(changed[:2], out[:2]))

    def test_full_tile_matches_python_with_skew_and_nontrivial_pose(self):
        rng = np.random.default_rng(20260907)
        points = np.c_[rng.uniform(0, 640, CAPACITY), rng.uniform(0, 480, CAPACITY),
                       rng.uniform(.11, 9.9, CAPACITY)].astype(np.float32)
        k = np.array([[525., 7, 319.5], [0, 520., 239.5], [0, 0, 1]])
        angle = .37
        t = np.eye(4)
        t[:3, :3] = [[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]]
        t[:3, 3] = [1.2, -.8, .3]
        out, valid, status = self.points(points, k, t)
        self.assertEqual(status, 0)
        self.assertTrue(np.all(valid == 1))
        np.testing.assert_allclose(out, backproject(points[:, :2], points[:, 2], k, t),
                                   atol=3e-6, rtol=2e-6)

    def test_invalid_depth_and_exact_validity_boundaries(self):
        low, high = np.float32(.1), np.float32(10)
        depths = np.array([np.nan, np.inf, -np.inf, -1, 0, low, high,
                           np.nextafter(low, np.float32(np.inf)),
                           np.nextafter(high, np.float32(0))], dtype=np.float32)
        points = np.c_[np.zeros(len(depths)), np.zeros(len(depths)), depths]
        out, valid, status = self.points(points, np.eye(3), np.eye(4))
        self.assertEqual(status, 0)
        np.testing.assert_array_equal(valid[:9], [0]*7 + [1, 1])
        np.testing.assert_array_equal(out[:7], np.zeros((7, 3)))
        np.testing.assert_array_equal(out[7:9, 2], depths[7:9])
        out, valid, status = self.points([[np.nan, 0, 2], [0, np.inf, 2]],
                                         np.eye(3), np.eye(4))
        np.testing.assert_array_equal(valid[:2], [0, 0])

    def test_count_boundaries_invalid_values_and_opposing_evidence(self):
        depths = np.array([np.nan, np.inf, -np.inf, 0, .1, 10, 1, 1.5, 2.5, 3,
                           np.nextafter(np.float32(1.5), np.float32(0)),
                           np.nextafter(np.float32(2.5), np.float32(np.inf))], dtype=np.float32)
        counts, status = self.counts(depths)
        self.assertEqual(status, 0)
        np.testing.assert_array_equal(counts, [12, 6, 2, 2])
        near, _ = self.counts(np.ones(CAPACITY))
        far, _ = self.counts(np.full(CAPACITY, 3))
        np.testing.assert_array_equal(near, [CAPACITY, CAPACITY, CAPACITY, 0])
        np.testing.assert_array_equal(far, [CAPACITY, CAPACITY, 0, CAPACITY])

    def test_counts_preserve_python_geometry_decisions(self):
        # Known projection: bounds +/- .25 at z=1.5, focal=12 -> pixels 10..14.
        k = np.array([[12., 0, 12], [0, 12., 12], [0, 0, 1]])
        memory = Memory()
        memory.tracks["test"] = Track("test", "cup", "red", np.array([0., 0, 2]),
                                      np.array([-.25, -.25, 1.5]),
                                      np.array([.25, .25, 2.5]), 0)
        cfg = Config()
        cases = [(np.ones(25), "occluded"), (np.full(25, 3), "free"),
                 (np.full(25, 2), "surface_or_uncertain"),
                 (np.r_[np.ones(17), np.full(8, np.nan)], "invalid_depth"),
                 (np.r_[np.full(23, 3), 2, 2], "free"),
                 (np.r_[np.full(22, 3), 2, 2, 2], "surface_or_uncertain")]
        for roi, expected in cases:
            with self.subTest(expected=expected, roi=roi):
                depth = np.full((24, 24), 5., dtype=np.float32)
                depth[10:15, 10:15] = roi.reshape(5, 5)
                frame = Frame("test", 1., np.zeros((24, 24, 3), dtype=np.uint8), depth, k, np.eye(4))
                counts, status = self.counts(roi, 1.5-cfg.depth_margin_m, 2.5+cfg.depth_margin_m)
                total, valid, occluded, free = counts.tolist()
                decision = ("invalid_depth" if valid < cfg.min_pixels or valid/total < cfg.min_valid_fraction
                            else "occluded" if occluded/valid >= .5
                            else "free" if free/valid >= cfg.free_fraction else "surface_or_uncertain")
                self.assertEqual(status, 0)
                self.assertEqual(decision, expected)
                self.assertEqual(compute_geometry(frame, memory)["test"], decision)

    def test_empty_oversized_and_bad_threshold_requests(self):
        counts, status = self.counts([])
        np.testing.assert_array_equal(counts, [0, 0, 0, 0])
        self.assertEqual(status, 0)
        counts, status = self.counts([], count=CAPACITY+1)
        self.assertEqual(status, 1)
        np.testing.assert_array_equal(counts, [0, 0, 0, 0])
        for front, back in [(3, 2), (np.nan, 3), (1, np.inf)]:
            counts, status = self.counts([2], front, back)
            self.assertEqual(status, 2)
            np.testing.assert_array_equal(counts, [0, 0, 0, 0])
        for count, expected in [(0, 0), (CAPACITY+1, 1)]:
            out, valid, status = self.points([], np.eye(3), np.eye(4), count=count)
            self.assertEqual(status, expected)
            self.assertTrue(np.all(out == 99) and np.all(valid == 99))

    def test_standalone_hls_testbench_as_host_cpp(self):
        result = subprocess.run([str(self.testbench)], check=True, capture_output=True, text=True)
        self.assertIn("geometry C++ testbench PASS", result.stdout)


if __name__ == "__main__":
    unittest.main()
