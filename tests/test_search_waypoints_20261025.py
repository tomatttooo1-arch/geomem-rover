"""Search waypoints from the real map (2026-10-25, find-object reliability).

Known answer (frame): a synthetic mapping graph whose optimized poses differ from the odometry poses by a known
per-node correction C_i (C_first = identity, as test_map_align_20261025); MappingCore on the export writes search
waypoints at the OPTIMIZED positions (= the exported grid's = Nav2's map frame after map_align), not the odometry ones.
Known answer (thinning): spacing >= 1 m, <= 6, none within 0.5 m of the start, yaw towards the centroid.
Runner side (no ROS): the status document -> PoseGoals, the selection (trajectory goals kept on unknown cells,
dropped on occupied ones, nearest first; car fallback = in-place quarter turns; simulation fallback = the five fixed
points filtered by FreeMap.free, unchanged), FreeMap.free / not_occupied on unknown cells, car_inner's arguments.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tools', 'car_run'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import mapping_ctl  # noqa: E402
from host import map_align  # noqa: E402
from host.ros_runtime import (FreeMap, MAPPING_STATUS_TOPIC, PoseGoal, load_waypoints,  # noqa: E402
                              mapping_search_waypoints, select_search_waypoints, spin_waypoints)
from test_map_align_20261025 import N, correction, odom, write_graph  # noqa: E402


def grid(fill, size_m=10.0, res=0.05, origin=-5.0):
    n = int(round(size_m / res))
    return FreeMap(np.full((n, n), fill, dtype=np.int8), res, origin, origin)


class TestFrameKnownAnswer(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = self.tmp.name
        self.poses, self.db = write_graph(self.d)
        with open(os.path.join(self.d, 'export', 'run.yaml'), 'w') as f:
            f.write('image: run.pgm\n')
        self.out = os.path.join(self.d, 'search_waypoints.json')

    def tearDown(self):
        self.tmp.cleanup()

    def test_waypoints_are_on_the_optimized_map_frame_trajectory(self):
        core = mapping_ctl.MappingCore(self.d, self.d, db=self.db, waypoints_out=self.out)
        core.request_stop()
        core.on_child_exit(0)
        self.assertEqual(core.state, 'exported')
        self.assertTrue(core.align['ok'])
        self.assertFalse(core.align['identity'])
        sw = core.status()['search_waypoints']
        self.assertTrue(sw['ok'], sw)
        self.assertEqual(sw['frame'], 'map')
        with open(self.out) as f:
            self.assertEqual(json.load(f), sw)
        opt = [(correction(k) @ odom(k))[:2, 3] for k in range(N)]   # the map frame: C_k x odom_k
        odo = [odom(k)[:2, 3] for k in range(N)]
        wps = sw['waypoints']
        self.assertGreaterEqual(len(wps), 2)
        for x, y, _yaw in wps:
            # every waypoint IS an optimized node position (1e-6) and is NOT its odometry position
            k = min(range(N), key=lambda i: math.hypot(opt[i][0] - x, opt[i][1] - y))
            self.assertLess(math.hypot(opt[k][0] - x, opt[k][1] - y), 1e-5)
            if k > 0:
                self.assertGreater(math.hypot(odo[k][0] - x, odo[k][1] - y), 0.02)
        # explicit known answer: the thinning of the optimized nodes, computed here by hand
        exp, x0 = [], opt[0]
        for p in opt:
            if math.hypot(*(p - x0)) >= 0.5 and all(math.hypot(*(p - q)) >= 1.0 for q in exp):
                exp.append(p)
        np.testing.assert_allclose([w[:2] for w in wps], np.array(exp), atol=1e-6)
        # the last node's correction is not small: the test separates the two frames
        self.assertGreater(math.hypot(opt[-1][0] - odo[-1][0], opt[-1][1] - odo[-1][1]), 0.25)
        # through the runner's parser: the latched status -> the same goals
        poses = mapping_search_waypoints(json.dumps(core.status()))
        self.assertEqual([(p.x, p.y, p.yaw) for p in poses], [tuple(w) for w in wps])

    def test_no_trajectory_says_why_and_publishes_none(self):
        os.remove(self.poses)
        core = mapping_ctl.MappingCore(self.d, self.d, db=self.db, waypoints_out=self.out)
        core.request_stop()
        core.on_child_exit(0)
        sw = core.status()['search_waypoints']
        self.assertFalse(sw['ok'])
        self.assertEqual(sw['waypoints'], [])
        self.assertIn('poses file', sw['reason'])
        self.assertIsNone(mapping_search_waypoints(json.dumps(core.status())))
        with open(self.out) as f:
            self.assertFalse(json.load(f)['ok'])

    def test_standin_has_none(self):
        core = mapping_ctl.MappingCore(self.d, self.d, standin=True, db=self.db)
        core.request_stop()
        core.on_child_exit(0)
        self.assertIn('stand-in', core.status()['search_waypoints']['reason'])


class TestThinning(unittest.TestCase):
    def test_spacing_start_and_cap(self):
        # a 0.1 m-step square loop 4 m x 4 m around (2, 2), from (0, 0), revisiting its start
        nodes, t = [], 0
        for (ax, ay), (bx, by) in (((0, 0), (4, 0)), ((4, 0), (4, 4)), ((4, 4), (0, 4)), ((0, 4), (0, 0))):
            for i in range(40):
                f = i / 40.0
                nodes.append((t, ax + (bx - ax) * f, ay + (by - ay) * f, 0.0))
                t += 1
        wps = mapping_ctl.search_waypoints(nodes)
        self.assertLessEqual(len(wps), 6)
        self.assertGreaterEqual(len(wps), 5)
        for i, a in enumerate(wps):
            self.assertGreaterEqual(math.hypot(a[0], a[1]), 0.5)
            for b in wps[:i]:
                self.assertGreaterEqual(math.hypot(a[0] - b[0], a[1] - b[1]), 1.0 - 1e-9)
        # spread over the loop: both the first side and the last side are represented
        self.assertTrue(any(w[1] < 0.01 for w in wps) and any(w[0] < 0.01 for w in wps))
        # facing the centroid (2, 2) (within float noise)
        for x, y, yaw in wps:
            want = math.atan2(2 - y, 2 - x)
            self.assertAlmostEqual(math.atan2(math.sin(yaw - want), math.cos(yaw - want)), 0.0, places=1)

    def test_short_trajectory_gives_nothing(self):
        self.assertEqual(mapping_ctl.search_waypoints([(0, 0, 0, 0), (1, 0.3, 0, 0)]), [])
        self.assertEqual(mapping_ctl.search_waypoints([]), [])

    def test_unsorted_input_is_time_ordered(self):
        a = mapping_ctl.search_waypoints([(2, 2.0, 0, 0), (0, 0, 0, 0), (1, 1.0, 0, 0)])
        self.assertEqual([w[0] for w in a], [1.0, 2.0])


class TestRunnerSelection(unittest.TestCase):
    def test_topic_is_mapping_ctl_status(self):
        self.assertEqual(MAPPING_STATUS_TOPIC, mapping_ctl.STATUS_TOPIC)

    def test_free_map_unknown_is_not_free_but_not_occupied(self):
        unknown = grid(-1)
        self.assertFalse(unknown.free(0, 0))
        self.assertTrue(unknown.not_occupied(0, 0))
        m = grid(0)
        m.cells[100, 100] = 100                # occupied cell at (0.0, 0.0)..(0.05, 0.05)
        self.assertFalse(m.not_occupied(0.2, 0.2))   # within the 0.30 m square
        self.assertTrue(m.not_occupied(0.4, 0.4))
        self.assertFalse(m.not_occupied(4.9, 0))     # footprint leaves the grid

    def test_fixed_sim_points_vanish_on_an_unknown_real_map(self):
        # the 10-22 failure: on a real (mostly unknown) map the five simulation points are all filtered out
        goals, info = select_search_waypoints(load_waypoints(), None, 'fixed', grid(-1), (0, 0, 0))
        self.assertEqual(goals, [])
        self.assertEqual(info['source'], 'fixed')

    def test_simulation_fallback_unchanged(self):
        fixed = load_waypoints()
        m = grid(0)
        goals, _ = select_search_waypoints(fixed, None, 'fixed', m, (0, 0, 0))
        self.assertEqual(goals, [p for p in fixed if m.free(p.x, p.y)])
        self.assertEqual(len(goals), 5)

    def test_mapped_goals_on_unknown_cells_kept_nearest_first(self):
        mapped = (PoseGoal(3.0, 0.0, 0.0), PoseGoal(1.0, 0.0, 0.0), PoseGoal(2.0, 2.0, 0.0))
        goals, info = select_search_waypoints(load_waypoints(), mapped, 'spin', grid(-1), (0.0, 0.0, 0.0))
        self.assertEqual(info['source'], 'mapping_trajectory')
        self.assertEqual([(p.x, p.y) for p in goals], [(1.0, 0.0), (3.0, 0.0), (2.0, 2.0)])   # greedy chain: 1 -> 3 (2.0 m) -> (2, 2)

    def test_mapped_goal_on_occupied_dropped(self):
        m = grid(0)
        m.cells[100, 120] = 100               # at (1.0, 0.0)
        mapped = (PoseGoal(1.0, 0.0, 0.0), PoseGoal(2.0, 2.0, 0.0))
        goals, info = select_search_waypoints((), mapped, 'spin', m, (0.0, 0.0, 0.0))
        self.assertEqual([(p.x, p.y) for p in goals], [(2.0, 2.0)])
        self.assertEqual(len(info['mapped_dropped']), 1)

    def test_car_fallback_spins_in_place_never_sim_points(self):
        m = grid(0)                           # even where the five points WOULD be free
        goals, info = select_search_waypoints(load_waypoints(), None, 'spin', m, (1.5, -0.5, 0.2))
        self.assertEqual(info['source'], 'spin_in_place')
        self.assertEqual([(p.x, p.y) for p in goals], [(1.5, -0.5)] * 3)
        for p, k in zip(goals, (1, 2, 3)):
            d = p.yaw - (0.2 + k * math.pi / 2)
            self.assertAlmostEqual(math.atan2(math.sin(d), math.cos(d)), 0.0, places=9)
        # all mapped goals occupied -> the same fallback
        m.cells[:, :] = 100
        goals, info = select_search_waypoints((), (PoseGoal(1.0, 0.0, 0.0),), 'spin', m, (0.0, 0.0, 0.0))
        self.assertEqual(info['source'], 'spin_in_place')
        goals, info = select_search_waypoints((), None, 'none', m, (0.0, 0.0, 0.0))
        self.assertEqual((goals, info['reason']), ([], 'no_search_waypoints'))
        self.assertEqual(len(spin_waypoints(0, 0, 0)), 3)

    def test_status_parser_rejects_bad_documents(self):
        for text in ('not json', '[]', '{}', json.dumps({'search_waypoints': None}),
                     json.dumps({'search_waypoints': {'ok': False, 'waypoints': [[1, 2, 3]]}}),
                     json.dumps({'search_waypoints': {'ok': True, 'waypoints': []}}),
                     json.dumps({'search_waypoints': {'ok': True, 'waypoints': [[1, 2]]}}),
                     json.dumps({'search_waypoints': {'ok': True, 'waypoints': [[1, 2, float('nan')]]}})):
            self.assertIsNone(mapping_search_waypoints(text), text)
        p = mapping_search_waypoints(json.dumps({'search_waypoints': {'ok': True, 'waypoints': [[1, 2, 0.5]]}}))
        self.assertEqual(p, (PoseGoal(1.0, 2.0, 0.5),))


class TestCarInnerArguments(unittest.TestCase):
    def test_flags_in_car_inner(self):
        with open(os.path.join(ROOT, 'tools', 'car_run', 'car_inner.sh')) as f:
            text = f.read()
        self.assertIn('[ "$MAPPING" = 1 ] && RX+=(--search-waypoints-from-mapping)', text)
        self.assertIn('[ "$PLATFORM" = board ] && RX+=(--search-fallback spin)', text)
        self.assertIn('--search-waypoints-out "$O/search_waypoints.json"', text)
        r = subprocess.run(['bash', '-n', os.path.join(ROOT, 'tools', 'car_run', 'car_inner.sh')])
        self.assertEqual(r.returncode, 0)

    def test_runner_parser_accepts_flags(self):
        from host import ros_runtime
        self.assertEqual(ros_runtime.SEARCH_FALLBACKS, ('fixed', 'spin', 'none'))


class TestMapSwap(unittest.TestCase):
    """After mapping_stop -> /map_server/load_map the runner must judge by the NEW map. Transport checked once in the
    fpga-dev1-sim:humble image (map_server old.yaml, a TRANSIENT_LOCAL/RELIABLE depth-1 subscriber as the runner's,
    load_map new.yaml): the subscriber got both grids (20x20 then 40x40), i.e. load_map re-publishes on /map. Here: the
    runner's own on_map (taken from host/ros_runtime.py's source) replaces the map on every message, and its /map
    subscription has that QoS."""

    @staticmethod
    def on_map():
        import ast
        import textwrap
        from types import SimpleNamespace as NS
        from host import ros_runtime
        src = open(ros_runtime.__file__).read()
        fn = [n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == 'on_map']
        assert len(fn) == 1
        text = textwrap.dedent('\n'.join(src.splitlines()[fn[0].lineno - 1:fn[0].end_lineno])) + '\n'
        ns = dict(vars(ros_runtime), args=NS(world_frame='map'))
        exec(compile(text, 'on_map', 'exec'), ns)
        return ns['on_map']

    @staticmethod
    def message(width, value, ox=0.0):
        from types import SimpleNamespace as NS
        q = NS(x=0.0, y=0.0, z=0.0, w=1.0)
        return NS(header=NS(frame_id='map'), data=[value] * (width * width),
                  info=NS(width=width, height=width, resolution=0.05,
                          origin=NS(position=NS(x=ox, y=0.0, z=0.0), orientation=q)))

    def test_new_map_replaces_old(self):
        from types import SimpleNamespace as NS
        on_map = self.on_map()
        faults = []
        node = NS(map=None, diagnostics=None, fault=faults.append, diagnostic_ack=lambda: None)
        on_map(node, self.message(20, 0))              # old: 1 m x 1 m all free
        self.assertTrue(node.map.free(0.5, 0.5))
        self.assertFalse(node.map.free(1.5, 0.5))      # outside the old map
        on_map(node, self.message(40, 0))              # new: 2 m x 2 m all free
        self.assertTrue(node.map.free(1.5, 0.5))
        on_map(node, self.message(40, 100))            # newer: all occupied
        self.assertFalse(node.map.free(0.5, 0.5))
        self.assertEqual(faults, [])

    def test_map_subscription_is_latched_reliable(self):
        from host import ros_runtime
        src = open(ros_runtime.__file__).read()
        i = src.index('self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)')
        q = src[src.rindex('map_qos = QoSProfile(', 0, i):i]
        self.assertIn('DurabilityPolicy.TRANSIENT_LOCAL', q)
        self.assertIn('ReliabilityPolicy.RELIABLE', q)


import test_odom_nav2 as ton  # noqa: E402


class TestBoardPlan(ton.BoardBase):
    """car_up --dry-run on the board: the runner gets the trajectory goals (mapping run) and the spin fallback."""

    def test_board_plans(self):
        r = self.up(*(self.ARGS + ['--dry-run']), CAR_MAPPING='1', CAR_TELEOP='1', CAR_MAP_YAML='')
        self.assertEqual(r.returncode, 0, r.stderr)
        rt = self.line(r.stdout, '    [runtime]')
        self.assertIn(' --search-waypoints-from-mapping ', rt)
        self.assertIn(' --search-fallback spin ', rt)
        self.assertLess(rt.index('--search-fallback'), rt.index('--ros-args') if '--ros-args' in rt else len(rt))
        self.assertIn('/search_waypoints.json ', self.line(r.stdout, '    [mapping]'))
        r = self.up(*(self.ARGS + ['--dry-run']))
        self.assertEqual(r.returncode, 0, r.stderr)
        rt = self.line(r.stdout, '    [runtime]')
        self.assertNotIn('--search-waypoints-from-mapping', rt)
        self.assertIn(' --search-fallback spin ', rt)


if __name__ == '__main__':
    unittest.main()
