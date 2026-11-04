import unittest
import numpy as np

from tools.rtabmap_match_gap_evaluate import optical_relative_to_tracked, transform_error, validate_transform, OPTICAL_IN_TRACKED


class MatchGapEvaluateTest(unittest.TestCase):
    def test_noncommuting_optical_conjugation(self):
        relative = np.array([[0,-1,0,1],[1,0,0,2],[0,0,1,3],[0,0,0,1]], float)
        tracked = optical_relative_to_tracked(relative)
        expected = np.array([[1,0,0,3],[0,0,-1,-1],[0,1,0,-2],[0,0,0,1]], float)
        np.testing.assert_allclose(tracked, expected)
        np.testing.assert_allclose(np.linalg.inv(OPTICAL_IN_TRACKED) @ tracked @ OPTICAL_IN_TRACKED, relative)
        self.assertFalse(np.allclose(tracked, relative))

    def test_transform_error_known_translation_and_rotation(self):
        actual = np.array([[0,-1,0,1],[1,0,0,0],[0,0,1,0],[0,0,0,1]], float)
        translation, rotation = transform_error(actual, np.eye(4))
        self.assertAlmostEqual(translation, 1)
        self.assertAlmostEqual(rotation, 90)

    def test_transform_validation_rejects_nonfinite_and_reflection(self):
        malformed = np.eye(4); malformed[0, 0] = np.nan
        with self.assertRaisesRegex(ValueError, "non-finite"):
            validate_transform(malformed, "nan")
        reflection = np.eye(4); reflection[0, 0] = -1
        with self.assertRaisesRegex(ValueError, "SO3"):
            validate_transform(reflection, "reflection")

    def test_transform_validation_rejects_bad_last_row(self):
        malformed = np.eye(4); malformed[3, 0] = 1
        with self.assertRaisesRegex(ValueError, "last row"):
            validate_transform(malformed, "homogeneous")


if __name__ == "__main__":
    unittest.main()
