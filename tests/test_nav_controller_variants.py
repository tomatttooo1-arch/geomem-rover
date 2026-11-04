"""Single-factor heading candidate must not change translation or safety budgets."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


def leaf_changes(a, b, path=()):
    if isinstance(a, dict) and isinstance(b, dict):
        if a.keys() != b.keys():
            return [path]
        return [change for key in a for change in leaf_changes(a[key], b[key], path+(key,))]
    return [] if a == b else [path]


class ControllerVariantTests(unittest.TestCase):
    def test_heading_candidate_changes_only_yaw_goal_tolerance(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)/'controllers'
            subprocess.run([sys.executable, str(root/'robot_sim/tests/nav_controller_variants.py'),
                            '--output', str(out), '--source-revision', 'unit-control'],
                           check=True, capture_output=True, text=True)
            cases = {name: yaml.safe_load((out/(name+'.yaml')).read_text())
                     for name in ('dwb', 'rpp', 'rpp_damped', 'rpp_heading_precision', 'rpp_dynamic')}
            self.assertEqual(cases['dwb'], yaml.safe_load((root/'robot_sim/fpga_sim/config/nav2_overrides.yaml').read_text()))
            self.assertEqual(leaf_changes(cases['rpp'], cases['rpp_damped']),
                             [('controller_server', 'ros__parameters', 'FollowPath', 'max_angular_accel')])
            self.assertEqual(leaf_changes(cases['rpp_damped'], cases['rpp_heading_precision']),
                             [('controller_server', 'ros__parameters', 'general_goal_checker', 'yaw_goal_tolerance')])
            checker = cases['rpp_heading_precision']['controller_server']['ros__parameters']['general_goal_checker']
            self.assertEqual(checker['yaw_goal_tolerance'], .005)
            self.assertEqual(checker['xy_goal_tolerance'], .03)
            self.assertEqual(checker['plugin'], 'nav2_controller::StoppedGoalChecker')
            self.assertIs(checker['stateful'], False)
            manifest = json.loads((out/'manifest.json').read_text())
            self.assertTrue(manifest['config_only_manifest'])
            self.assertTrue(manifest['current_task_budgets_defined_by_runtime'])
            self.assertNotIn('wall_timeout_seconds', manifest)
            self.assertNotIn('goal', manifest)
            self.assertEqual(manifest['legacy_reference_probe_conditions']['wall_timeout_seconds'], 90)
            self.assertFalse(manifest['same_progress_goal_checkers'])
            self.assertEqual(manifest['goal_checker_exception']['case'], 'rpp_heading_precision')
            dynamic = cases['rpp_dynamic']
            self.assertEqual(dynamic['controller_server']['ros__parameters'].pop('failure_tolerance'), 2.0)
            self.assertEqual(dynamic, cases['rpp_heading_precision'])
            self.assertEqual(manifest['controller_patience_exception']['case'], 'rpp_dynamic')


if __name__ == '__main__':
    unittest.main()
