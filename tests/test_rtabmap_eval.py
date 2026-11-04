"""Calibrate ATE against analytically known rigid and nonrigid controls."""
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tools.rtabmap_eval import associate, evaluate, read_tum


class TrajectoryEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        # Planar square has known centered RMS radius sqrt(0.5), no full
        # rotational degeneracy, and a known three-edge path length of 3 m.
        self.points = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]])
        self.truth = self.write("truth.txt", self.points)

    def write(self, name, points, timestamps=None):
        timestamps = np.arange(len(points)) if timestamps is None else timestamps
        rows = [[t, *point, 0., 0., 0., 1.] for t, point in zip(timestamps, points)]
        path = self.root / name
        np.savetxt(path, rows, fmt="%.12g")
        return path

    def test_rigid_transform_has_zero_error(self):
        rotation = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
        estimated = self.write("rigid.txt", self.points @ rotation.T + [8., -3., 2.])
        result = evaluate(estimated, self.truth)
        self.assertLess(result["ate_translation_rmse_m"], 1e-12)
        self.assertEqual(result["matched_samples"], 4)
        self.assertEqual(result["estimated_sample_coverage"], 1.)
        self.assertAlmostEqual(result["estimated_raw_length_m"], 3.)

    def test_scale_error_is_not_fitted_away(self):
        estimated = self.write("scaled.txt", 2 * self.points)
        result = evaluate(estimated, self.truth)
        self.assertAlmostEqual(result["ate_translation_rmse_m"], np.sqrt(.5), places=10)
        self.assertAlmostEqual(result["estimated_raw_length_m"], 6.)
        self.assertAlmostEqual(result["ground_truth_raw_length_m"], 3.)

    def test_nonrigid_deformation_is_detected(self):
        points = self.points.copy()
        points[-1, 2] = 2.
        result = evaluate(self.write("deformed.txt", points), self.truth)
        self.assertGreater(result["ate_translation_rmse_m"], .1)

    def test_time_mismatch_is_rejected(self):
        estimated = self.write("shifted.txt", self.points, np.arange(4) + .1)
        with self.assertRaisesRegex(ValueError, "insufficient timestamp matches"):
            evaluate(estimated, self.truth)

    def test_association_does_not_reuse_truth(self):
        self.assertEqual(associate(np.array([0., .001, 1., 2.]), np.array([0., 1., 2.])),
                         [(0, 0), (2, 1), (3, 2)])
        estimated = self.write("extra.txt", self.points, [0., .001, 1., 2.])
        result = evaluate(estimated, self.truth)
        self.assertEqual(result["matched_samples"], 3)
        self.assertEqual(result["estimated_sample_coverage"], .75)
        self.assertEqual(result["ground_truth_sample_coverage"], .75)

    def test_invalid_inputs_rejected(self):
        for content in ("", "# comment\n", "0 nan 0 0 0 0 0 1", "0 0 0 0 0 0 0 0",
                        "0 0 0", "nonnumeric", "0 0 0 0 0 0 0 1\n0 1 0 0 0 0 0 1"):
            with self.subTest(content=content):
                path = self.root / "bad.txt"
                path.write_text(content)
                with self.assertRaises(ValueError):
                    read_tum(path)
        for threshold in (0, -1, float("nan"), float("inf")):
            with self.subTest(threshold=threshold), self.assertRaises(ValueError):
                evaluate(self.truth, self.truth, threshold)

    def test_unordered_rows_sorted_and_too_few_matches_rejected(self):
        path = self.write("unordered.txt", self.points[::-1], np.arange(4)[::-1])
        self.assertLess(evaluate(path, self.truth)["ate_translation_rmse_m"], 1e-12)
        short = self.write("short.txt", self.points[:2])
        with self.assertRaisesRegex(ValueError, "insufficient timestamp matches"):
            evaluate(short, self.truth)


if __name__ == "__main__":
    unittest.main()
