"""Geometry checks independent of the wavefront implementation (no ROS or model build)."""
import unittest
import numpy as np
from tools.planner_model.evaluate import Geometry, distribution, goals, length, segment_cells, summarize


class PlannerGeometryTest(unittest.TestCase):
    def test_wall_between_vertices_and_unknown(self):
        m = np.zeros((12, 12), np.uint8)
        m[5, 5] = 254
        m[6, 6] = 255
        g = Geometry(m)
        self.assertEqual(len(g.collisions(np.array([[2., 2.], [9., 9.]]))), 2)
        self.assertFalse(g.collisions(np.array([[2., 2.], [2., 9.]])))

    def test_corner_contact_and_outside(self):
        m = np.zeros((6, 6), np.uint8);m[1, 2] = 253
        self.assertTrue(Geometry(m).collisions(np.array([[1., 1.], [2., 2.]])))
        self.assertTrue(Geometry(m).collisions(np.array([[-1., 1.], [1., 1.]])))
        self.assertEqual(set(map(tuple, segment_cells(np.array([1., 1.]), np.array([3., 1.]), m.shape))),
                         {(1, 1), (2, 1), (3, 1)})

    def test_continuous_clearance_not_vertices_only(self):
        m = np.zeros((12, 12), np.uint8);m[6, 5] = 254
        path = np.array([[2., 3.], [8., 3.]])
        self.assertAlmostEqual(Geometry(m).clearance(path), 3.)
        self.assertAlmostEqual(length(path), .3)

    def test_smoothing_cannot_skip_wall(self):
        m = np.zeros((12, 12), np.uint8);m[2:8, 5] = 254
        path = np.array([[2., 3.], [2., 9.], [8., 9.], [8., 3.]])
        g = Geometry(m); smooth = g.smooth(path)
        self.assertFalse(g.collisions(smooth))
        self.assertLessEqual(length(smooth), length(path))
        self.assertGreaterEqual(len(smooth), 3)

    def test_deterministic_no_duplicate_goals(self):
        m = np.zeros((80, 80), np.uint8)
        a = goals(m, (40, 40), 11)
        self.assertEqual(a, goals(m, (40, 40), 11))
        self.assertEqual(len(a), 25)
        self.assertEqual(len({p for p, _ in a}), 25)
        self.assertNotIn((40, 40), [p for p, _ in a])
        self.assertNotEqual(a, goals(m, (40, 40), 12))

    def test_all_candidate_failures_keep_baseline_and_do_not_crash(self):
        row = dict(navfn_success=True, navfn_exact=False, navfn_hits=[],
                   navfn_length=1., navfn_clearance=1., fixed_equal=True,
                   rounds=0, cells=6400, no_obstacles=False, frame=1, goals_in_frame=1)
        for mode in ['float', 'smooth', 'mutant']:
            row[mode+'_success'] = False
            row[mode+'_hits'] = []
        stats = summarize([row])
        self.assertEqual(stats['success_rate'], 0.)
        self.assertEqual(stats['float']['clearance_navfn_median'], 1.)
        self.assertIsNone(stats['float']['clearance_wave_median'])
        self.assertFalse(stats['float']['gates']['clearance'])
        self.assertFalse(stats['float']['gates']['length'])

    def test_empty_distribution_not_pass(self):
        self.assertIsNone(distribution([])['median'])
        self.assertEqual(distribution([1, 2, 3])['median'], 2.)


if __name__ == '__main__':
    unittest.main()
