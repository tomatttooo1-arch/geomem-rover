"""End-to-end CPU checks for the fixed M5 Runtime software replay."""

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import tools.m5_software_replay as replay_module
from tools.m5_software_replay import SCENARIOS, main, run_scenarios
from tools.rtabmap_runtime_probe_support import REQUIRED_PYTHON

# The replay loads the production Runtime through a probe that needs 3.9+ (D-028).
# Below that the scenarios cannot run; what must hold instead is that every one of
# them fails and names that requirement, rather than passing, skipping or failing
# with some unrelated error.
SUPPORTED = sys.version_info >= REQUIRED_PYTHON
DECLARED = "requires Python 3.9+ (ast.unparse)"


class M5RuntimeReplayTests(unittest.TestCase):
    def test_all_runtime_scenarios(self):
        results = run_scenarios()
        self.assertEqual(len(results), len(SCENARIOS))
        if SUPPORTED:
            self.assertTrue(all(row["passed"] for row in results), results)
        else:
            for row in results:
                self.assertFalse(row["passed"], row["name"])
                self.assertTrue(row["error"].startswith("RuntimeError: "), row["error"])
                self.assertIn(DECLARED, row["error"])
        self.assertEqual(
            {row["name"] for row in results},
            {function.__name__.replace("scenario_", "") for function in SCENARIOS},
        )

    def test_cli_writes_per_scenario_evidence_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "m5-replay"
            if not SUPPORTED:
                self.assertEqual(main(["--output", str(output)]), 1)
                summary = json.loads((output / "summary.json").read_text())
                self.assertFalse(summary["passed"])
                self.assertTrue(all(DECLARED in row["error"] for row in summary["scenarios"]))
                return
            self.assertEqual(main(["--output", str(output)]), 0)
            summary = json.loads((output / "summary.json").read_text())
            self.assertTrue(summary["passed"])
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(len(manifest["head"]), 40)
            self.assertIn("host/ros_runtime.py", manifest["sources"])
            for row in summary["scenarios"]:
                directory = output / row["name"]
                events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
                self.assertTrue(events)
                self.assertTrue((directory / "summary.json").is_file())
                loaders = [event for event in events if event["event"] == "loader"]
                self.assertEqual(loaders[0]["candidate_branch_insertions"], 0)
                self.assertEqual(loaders[0]["source_class_ast_sha256"],
                                 loaders[0]["loaded_class_ast_sha256"])
                boundaries = [event for event in events
                              if event["event"] == "inference_boundary_delivery"]
                self.assertTrue(boundaries)
                self.assertTrue(all(event["detector"] == "explicit_synthetic_oracle_boxes"
                                    and "not_camera_or_dds" in event["boundary"]
                                    for event in boundaries))
            with self.assertRaises(FileExistsError):
                main(["--output", str(output)])

    def test_a_scenario_that_dies_mid_construction_reports_its_own_cause(self):
        """The recovery path must not overwrite the error it exists to preserve.

        On a scenario failure `run_scenarios` reads the partially built Replay back out
        of `last_created` to keep its raw evidence. That object can be missing anything
        `__init__` had not reached yet, so touching it unconditionally makes this path
        raise its own AttributeError and the run then reports the cleanup, not the cause.
        This is not hypothetical: `tools/rtabmap_runtime_probe_support.py` needs Python
        3.9 or newer, and on 3.8 every scenario died here reporting a missing attribute
        with no hint that `ast.unparse` was the real reason.
        """
        def scenario_dies_while_building():
            return replay_module.Replay("dies_while_building")

        with patch.object(replay_module, "SCENARIOS", (scenario_dies_while_building,)), \
                patch.object(replay_module, "load_runtime",
                             side_effect=RuntimeError("runtime could not be loaded")):
            results = run_scenarios()

        self.assertEqual(len(results), 1)
        row = results[0]
        self.assertEqual(row["name"], "dies_while_building")
        self.assertFalse(row["passed"])
        self.assertEqual(row["error"], "RuntimeError: runtime could not be loaded")
        self.assertIsNone(row["final_task_state"])


if __name__ == "__main__":
    unittest.main()
