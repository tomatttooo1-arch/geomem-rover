"""The dense-arm sensitivity tool builds exactly the faults it names (2026-09-27)."""

import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location(
    "check_m7_dense_arm_sensitivity", ROOT / "tools/check_m7_dense_arm_sensitivity.py")
TOOL = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(TOOL)


class DenseArmSensitivityToolTests(unittest.TestCase):
    def setUp(self):
        self.v = TOOL.variants()

    def test_arms_are_the_committed_sources_with_local_include(self):
        for arm, rel in TOOL.ARMS.items():
            text = (ROOT / rel).read_text()
            self.assertNotIn('"../frame_depth.hpp"', self.v[arm])
            if arm != "QPIPE":
                self.assertEqual(self.v[arm],
                                 text.replace('"../frame_depth.hpp"', '"frame_depth.hpp"'))
        self.assertIn("#ifdef __SYNTHESIS__\n#pragma HLS PIPELINE II=1\n#endif",
                      self.v["QPIPE"])

    def test_row_stuck_removes_only_the_row_increment(self):
        self.assertNotIn("++row_index;", self.v["UV_ROW_STUCK"])
        self.assertEqual(self.v["UV_ROW_STUCK"].replace("(void)row_index;", "++row_index;"),
                         self.v["UV"])

    def test_skip_invalid_moves_the_advance_after_the_depth_check(self):
        text = self.v["UV_SKIP_INVALID"]
        self.assertEqual(text.count("if (++column == width)"), 1)
        self.assertLess(text.index("++invalid;"), text.index("if (++column == width)"))
        self.assertGreater(self.v["UV"].index("++invalid;"),
                           self.v["UV"].index("if (++column == width)"))

    def test_ambiguous_or_missing_pattern_stops_the_tool(self):
        with self.assertRaises(SystemExit):
            TOOL._replace_once("a a", "a", "b", "twice")
        with self.assertRaises(SystemExit):
            TOOL._replace_once("a", "c", "b", "absent")


if __name__ == "__main__":
    unittest.main()
