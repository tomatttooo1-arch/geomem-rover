"""SW_M1 (nationals 2026-11-03): tools/m7_rolling_pl/replay_trace.py replays op GRID records (BOARD2 grid_replay.py).

Known answer: MERGE1/b1/grid_formal.bin (1,239 records, 750 GRID of which 748 accepted) through the mock PL core
built for GRID = 0 mismatched records (BOARD2 host calibration: 1,239 / 0). Positive control: --corrupt-grid-flags
makes exactly the 748 accepted GRID records differ (BOARD2: 748). A trace without GRID records (formal.bin, 489)
replays as before (0 mismatches, no grid keys in the summary); the cpu backend refuses a GRID trace.
The traces live under runs/ (not in git): skipped when absent.
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_swm1_grid_replay_20261103.py"""
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNS = "${FPGA_SRC}/runs"
GRID_TRACE = os.environ.get("SWM1_GRID_TRACE", RUNS + "/nationals-prelim-20261102/MERGE1/b1/grid_formal.bin")
OLD_TRACE = os.environ.get("B1_TRACE", RUNS + "/fpga-innov-20261025/fuse/traces/formal.bin")
TOOL = str(ROOT / "tools/m7_rolling_pl/replay_trace.py")


def replay(trace, *extra):
    with tempfile.TemporaryDirectory() as t:
        out = os.path.join(t, "s.json")
        r = subprocess.run([sys.executable, TOOL, "--trace", trace, "--out", out] + list(extra),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                           env=dict(os.environ, PYTHONPATH=str(ROOT)), timeout=600)
        s = json.load(open(out)) if os.path.exists(out) else None
    return r, s


@unittest.skipUnless(os.path.exists(GRID_TRACE), "grid trace not on this host")
class GridTrace(unittest.TestCase):
    def test_mock_exact_and_control(self):
        r, s = replay(GRID_TRACE, "--backend", "mock")
        self.assertEqual(r.returncode, 0, r.stderr[-2000:])
        self.assertEqual((s["records"], s["mismatched_records"], s["grid_records"], s["grid_planes_read"]),
                         (1239, 0, 750, 748))
        self.assertTrue(s["grid_probe"]["capable"])
        r, c = replay(GRID_TRACE, "--backend", "mock", "--corrupt-grid-flags", "--expect-fail")
        self.assertEqual(r.returncode, 0, r.stderr[-2000:])
        self.assertEqual(c["mismatched_records"], 748)
        self.assertTrue(all(m["op"] == 4 for m in c["first_mismatches"]))

    def test_cpu_backend_refuses(self):
        r, _ = replay(GRID_TRACE, "--backend", "cpu")
        self.assertEqual(r.returncode, 2)
        self.assertIn("op GRID records", r.stderr)


@unittest.skipUnless(os.path.exists(OLD_TRACE), "formal trace not on this host")
class OldTrace(unittest.TestCase):
    def test_no_grid_records_as_before(self):
        r, s = replay(OLD_TRACE, "--backend", "mock")
        self.assertEqual(r.returncode, 0, r.stderr[-2000:])
        self.assertEqual((s["records"], s["mismatched_records"]), (489, 0))
        self.assertNotIn("grid_records", s)
        self.assertNotIn("grid_probe", r.stdout)
        r, _ = replay(OLD_TRACE, "--backend", "mock", "--corrupt-grid-flags")
        self.assertEqual(r.returncode, 2)


if __name__ == "__main__":
    unittest.main()
