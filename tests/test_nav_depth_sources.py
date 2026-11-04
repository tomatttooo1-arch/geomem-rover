"""Depth obstacle marking excludes floor; clearing retains measured floor rays."""
from pathlib import Path
import unittest
import yaml


class DepthSourceTests(unittest.TestCase):
    def test_ground_clearing_is_separate_from_obstacle_marking(self):
        config = yaml.safe_load((Path(__file__).resolve().parents[1] /
            'robot_sim/fpga_sim/config/nav2_overrides.yaml').read_text())
        for name in ('local_costmap', 'global_costmap'):
            layer = config[name][name]['ros__parameters']['obstacle_layer']
            mark, clear = layer['depth'], layer['depth_clear']
            self.assertEqual(layer['observation_sources'].split(), ['depth', 'depth_clear'])
            self.assertTrue(mark['marking'])
            self.assertFalse(mark['clearing'])
            self.assertFalse(clear['marking'])
            self.assertTrue(clear['clearing'])
            for z in (0., -0.00001, 0.00001):
                self.assertFalse(mark['min_obstacle_height'] <= z <= mark['max_obstacle_height'])
                self.assertTrue(clear['min_obstacle_height'] <= z <= clear['max_obstacle_height'])
            for z in (.08, .3, 1.5):
                self.assertTrue(mark['min_obstacle_height'] <= z <= mark['max_obstacle_height'])
            self.assertEqual(mark['min_obstacle_height'], .08)
            for key in ('topic', 'data_type', 'max_obstacle_height', 'obstacle_min_range',
                        'obstacle_max_range', 'raytrace_min_range', 'raytrace_max_range',
                        'observation_persistence', 'expected_update_rate'):
                self.assertEqual(mark[key], clear[key])
