"""tum_rgbd_pub --stamp-now: default keeps dataset stamps, enabled restamps near wall time."""

from pathlib import Path
import re
import time
import unittest

from tools.nano_ros2.tum_rgbd_pub import frame_stamp

SRC = Path(__file__).resolve().parents[1] / "tools/nano_ros2/tum_rgbd_pub.py"
# fr1_desk-like rgb stamps, plus a value that carries into the next second
STAMPS = [1305031452.791720, 1305031452.823674, 1305031452.859642, 1305031452.891726,
          1305031453.9999999997]


def original(rs):
    """The stamp split as it was before --stamp-now existed."""
    sec, nsec = int(rs), int(round((rs - int(rs)) * 1e9))
    if nsec >= 1_000_000_000:
        sec, nsec = sec + 1, nsec - 1_000_000_000
    return sec, nsec


def ns(stamp):
    return stamp[0] * 1_000_000_000 + stamp[1]


class StampNowTests(unittest.TestCase):
    def test_default_is_dataset_stamp(self):
        for rs in STAMPS:
            self.assertEqual(frame_stamp(rs), original(rs))
            self.assertEqual(frame_stamp(rs, STAMPS[0], None), original(rs))
        self.assertEqual(frame_stamp(STAMPS[-1]), (1305031454, 0))

    def test_enabled_is_near_now_and_keeps_intervals(self):
        anchor = time.time_ns()
        out = [frame_stamp(rs, STAMPS[0], anchor) for rs in STAMPS]
        self.assertLess(abs(ns(out[0]) - time.time_ns()), 1_000_000_000)
        for k in range(1, len(STAMPS)):
            self.assertEqual(ns(out[k]) - ns(out[k - 1]),
                             round((STAMPS[k] - STAMPS[0]) * 1e9) - round((STAMPS[k - 1] - STAMPS[0]) * 1e9))
            self.assertAlmostEqual((ns(out[k]) - ns(out[k - 1])) * 1e-9, STAMPS[k] - STAMPS[k - 1], places=6)
            self.assertTrue(0 <= out[k][1] < 1_000_000_000)
        self.assertNotEqual(out[0], original(STAMPS[0]))

    def test_same_frame_topics_share_one_stamp(self):
        anchor = time.time_ns()
        self.assertEqual(frame_stamp(STAMPS[2], STAMPS[0], anchor), frame_stamp(STAMPS[2], STAMPS[0], anchor))
        # main(): one frame_stamp call per frame; rgb and depth take it via img(), camera_info copies rgb's header
        main = SRC.read_text().split("def main():", 1)[1]
        self.assertEqual(len(re.findall(r"frame_stamp\(", main)), 1)
        self.assertEqual(len(re.findall(r"m\.header\.stamp\.sec, m\.header\.stamp\.nanosec = sec, nsec", main)), 1)
        self.assertIn("ci.header = rm.header", main)
        self.assertNotRegex(main, r"int\(rs\)|int\(ds\)")


if __name__ == "__main__":
    unittest.main()
