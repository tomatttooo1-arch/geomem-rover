import unittest
import numpy as np

from tools.rtabmap_match_gap_prepare import pose_matrix, relative_metrics, representatives


class MatchGapPrepareTest(unittest.TestCase):
    def test_representatives_are_first_floor_middle_last(self):
        rows = [{"index": value} for value in (9, 2, 5, 7)]
        self.assertEqual([row["index"] for row in representatives(rows)], [2, 7, 9])

    def test_short_population_deduplicates(self):
        self.assertEqual([row["index"] for row in representatives([{"index": 4}])], [4])

    def test_noncommuting_relative_pose_known_answer(self):
        # left is 90 degrees about Z at x=1; right is 90 degrees about X at y=1.
        s = np.sqrt(.5)
        left = pose_matrix(np.array([0, 1, 0, 0, 0, 0, s, s], float))
        right = pose_matrix(np.array([0, 0, 1, 0, s, 0, 0, s], float))
        distance, angle, relative = relative_metrics(left, right)
        self.assertAlmostEqual(distance, np.sqrt(2))
        self.assertAlmostEqual(angle, 120.0)
        np.testing.assert_allclose(left @ relative, right, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
