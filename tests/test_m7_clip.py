"""Clipped M7 frame (docs/design/M7_CLIP_SPEC_20261011.md): golden and positive controls."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from tools.m7_clip.golden import golden_rays

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("existing_m7_ray_oracle", ROOT / "tests/test_m7_native.py")
oracle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(oracle)


class ClipGoldenTests(unittest.TestCase):
    def test_golden_equals_existing_fraction_oracle_inside_grid(self):
        # For in-grid endpoints the clip golden must reduce to the 09-21 exact
        # Fraction line/voxel oracle (tests/test_m7_native.py), ties included.
        rng = np.random.default_rng(11)
        for i in range(150):
            if i % 2:
                start = rng.integers(0, 8, size=3) * 128
                end = rng.integers(0, 8, size=3) * 128
            else:
                start = rng.integers(0, 1024, size=3)
                end = rng.integers(0, 1024, size=3)
            for method in ("rational", "double"):
                got = golden_rays(start, [end], [True], (4, 4, 4), method)
                want = oracle.analytic_frame([start.tolist()], [end.tolist()], (4, 4, 4)).ravel()
                np.testing.assert_array_equal(got, want, err_msg="%s %s %s" % (method, start, end))

    def test_positive_controls_small(self):
        with tempfile.TemporaryDirectory(prefix="m7-clip-") as tmp:
            out = Path(tmp) / "report.json"
            result = subprocess.run([sys.executable, str(ROOT / "tools/m7_clip/positive_control.py"),
                                     "--build", str(Path(tmp) / "build"), "--out", str(out),
                                     "--random", "120"], capture_output=True, text=True)
            with open(out) as fp:
                summary = json.load(fp)["summary"]
            self.assertEqual(result.returncode, 0, summary["failures"])
            self.assertTrue(summary["pass"])
            self.assertGreater(summary["mut_drop_caught_frames"], 0)
            self.assertGreater(summary["mut_trunc_caught_frames"], 0)
            self.assertEqual(summary["new_eq_golden"], summary["frames"])


if __name__ == "__main__":
    unittest.main()
