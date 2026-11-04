"""10-29 stop sweep (host/search_belief.py, owner: 判搜完前补看没朝向过的方向): when the stop rule fires, the
planner first looks from the reachable (trajectory node, heading) views not chosen yet that would still newly see
(q >= 0.5) at least ``sweep_min_m2`` of not-yet-looked-at searchable floor, however little prior mass it holds;
the search ends only when no such view is left, or none fits in the task's remaining time.

Known answers. 1. The host-sim 'bottle moved 2.4 m south-west' miss (accept-1102 run, the 5 live-track nodes, the
sim room map and camera): after the 5 looks the sim chose, the old rule stops (mass 0.098, unseen 0.019) without
any view toward the bottle; the sweep's first look faces it from the node 0.6 m south of the walk and sees it.
2. Nothing there (living room, car camera): the sweep looks only at views that still see new floor, never the same
view twice, ends by itself with >= 99 % of the reachable floor looked at and no view left that would see new floor.
3. Before the stop rule fires the looks are the same as without the sweep (near objects unchanged).
4. ``time_left_s`` leaves out sweep looks it cannot fit (search_exhausted instead of the task time limit), and the
task controller passes its remaining time; planners without the argument still work. Host only, numpy, no ROS.
"""
from dataclasses import replace
import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools", "car_run"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

import search_preview  # noqa: E402
from host.search_belief import BeliefSearch, SearchConfig, sweep_eta  # noqa: E402
from host.task_control import PoseGoal, TaskConfig, TaskController, _takes_time_left  # noqa: E402
from test_search_prior_floor_20261029 import (K, LIVING, LIVING_NODES, SIZE, START, TARGET, mount,  # noqa: E402
                                              search)

NO_SWEEP = SearchConfig(stop_sweep=False, stop_unseen=0.02)      # the stop of 5b03a62 (prior floor)
SWEEP_02 = SearchConfig(stop_unseen=0.02)       # the sweep alone, at the same stop line as 5b03a62

# the host sim (accept-1102 'moved 2.4 m', run-190328): room map, the runner's 5 live-track nodes, sim camera
SIM_FX = 320 / math.tan(1.58 / 2)
SIM_K = np.array([[SIM_FX, 0, 320], [0, SIM_FX, 200], [0, 0, 1.]])
SIM_NODES = [(0.0, 0.0), (0.083, 0.538), (0.199, -0.535), (-0.416, 0.457), (0.545, -0.164)]
SIM_TARGET, SIM_START, BOTTLE = (1.22, -0.437), (-0.428, 0.455, 3.059), (-0.9, -1.3)
SIM_LOOKS = [(2, -0.7854), (0, 1.5708), (1, 0.0), (4, -0.7854), (3, 2.3562)]     # what the sim chose, in order


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def sim_after_looks(cfg):
    """The sim's 5 looks (turn / drive frames included) on the room map; returns (planner, bottle cell, view)."""
    m = search_preview.load_map(os.path.join(ROOT, "robot_sim", "fpga_sim", "maps", "room.yaml"))
    b = BeliefSearch(m, SIM_NODES, cfg, robot=SIM_START)
    b.set_camera(SIM_K, (640, 400), mount(27, 0.35, 0.13))
    b.begin(SIM_TARGET)
    oc = int(np.argmin((b.ux - BOTTLE[0]) ** 2 + (b.uy - BOTTLE[1]) ** 2))
    seen = [False]
    pose = list(SIM_START)

    def view(x, y, yaw):
        before = b.miss[oc]
        b.apply_base_view(x, y, yaw)
        seen[0] |= bool(b.miss[oc] <= .5 * before + 1e-12)

    def go(x, y, yaw):
        rx, ry, ryaw = pose
        if math.hypot(x - rx, y - ry) > .05:
            head = math.atan2(y - ry, x - rx)
            for a in np.arange(0., abs(wrap(head - ryaw)), .4)[1:]:
                view(rx, ry, ryaw + math.copysign(a, wrap(head - ryaw)))
            n = max(1, int(math.hypot(x - rx, y - ry) / .3))
            for i in range(1, n + 1):
                view(rx + (x - rx) * i / n, ry + (y - ry) * i / n, head)
            ryaw = head
        for a in np.arange(0., abs(wrap(yaw - ryaw)), .4)[1:]:
            view(x, y, ryaw + math.copysign(a, wrap(yaw - ryaw)))
        view(x, y, yaw)
        pose[:] = [x, y, yaw]
        b.robot = (x, y, yaw)

    view(*SIM_START)
    for n, yaw in SIM_LOOKS:
        b.visited.add((n, int(np.argmin([abs(wrap(v - yaw)) for v in b.yaw_set]))))
        go(SIM_NODES[n][0], SIM_NODES[n][1], yaw)
    return b, oc, go, seen


class SimMissTests(unittest.TestCase):
    def test_old_stop_misses_the_bottle_the_sweep_looks_at_it(self):
        b, oc, _go, seen = sim_after_looks(NO_SWEEP)
        self.assertFalse(seen[0])
        self.assertTrue(b.reachable()[oc])                          # a candidate view could see it at q >= 0.5
        self.assertGreater(b.miss[oc], .5)                          # but none looked at it
        pose, info = b.next_goal()
        self.assertIsNone(pose)
        self.assertEqual(info["stop"], "mass_below_stop")
        self.assertLess(info["unseen"], 0.02)

        b, oc, go, seen = sim_after_looks(SWEEP_02)
        pose, info = b.next_goal()
        self.assertEqual(info["sweep"], "mass_below_stop")
        self.assertEqual(info["node"], 2)                           # (0.20, -0.54), facing south-west
        self.assertLess(abs(wrap(pose.yaw - math.atan2(BOTTLE[1] - pose.y, BOTTLE[0] - pose.x))), math.radians(50))
        self.assertGreaterEqual(info["new_m2"], SearchConfig().sweep_min_m2)
        go(pose.x, pose.y, pose.yaw)
        self.assertTrue(seen[0])
        # the defaults (stop line 0.01 + the sweep): seen within the next 3 looks too
        b, oc, go, seen = sim_after_looks(SearchConfig())
        for _ in range(3):
            pose, info = b.next_goal()
            self.assertIsNotNone(pose, info)
            go(pose.x, pose.y, pose.yaw)
            if seen[0]:
                break
        self.assertTrue(seen[0])


class NothingThereTests(unittest.TestCase):
    def test_sweep_ends_by_itself_without_re_looks(self):
        for cfg in (SWEEP_02, SearchConfig()):
            with self.subTest(stop_unseen=cfg.stop_unseen):
                self._ends(cfg)

    def _ends(self, cfg):
        found, looks, stop, b, goals = search(cfg, (99., 99.))
        self.assertFalse(found)
        self.assertIn(stop, ("mass_below_stop", "no_candidate"))
        keys = [(i["node"], i["yaw"]) for _p, i in goals]
        self.assertEqual(len(keys), len(set(keys)))                 # no view chosen twice
        sweep = [i for _p, i in goals if "sweep" in i]
        self.assertGreater(len(sweep), 0)
        self.assertTrue(all(i["new_m2"] >= SearchConfig().sweep_min_m2 for i in sweep))
        self.assertLess(looks, 40)
        reach = b.reachable()
        self.assertGreaterEqual(float(np.mean(b.miss[reach] <= .5)), 0.99)
        _g, new_m2 = b.gains(new_area=True)
        left = [(n, k) for n in range(len(b.nodes)) for k in range(len(b.yaw_set))
                if (n, k) not in b.visited and new_m2[n, k] >= SearchConfig().sweep_min_m2]
        self.assertEqual(left, [])                                   # nothing that would see new floor is left
        # the sweep comes after the old stop: the looks before it are exactly the old ones (same stop line)
        _f, k_old, stop_old, b_old, g_old = search(replace(NO_SWEEP, stop_unseen=cfg.stop_unseen), (99., 99.))
        self.assertEqual([(i["node"], i["yaw"]) for _p, i in g_old], keys[:k_old])
        self.assertEqual(len(sweep), looks - k_old)
        self.assertGreater(float(np.mean(b.miss[reach] <= .5)), float(np.mean(b_old.miss[reach] <= .5)))

    def test_near_objects_unchanged(self):
        n = 0
        for r in (0.5, 1.0):
            for deg in range(0, 360, 45):
                obj = (TARGET[0] + r * math.cos(math.radians(deg)), TARGET[1] + r * math.sin(math.radians(deg)))
                f0, k0, _s, b, g0 = search(NO_SWEEP, obj)
                oc = int(np.argmin((b.ux - obj[0]) ** 2 + (b.uy - obj[1]) ** 2))
                if math.hypot(b.ux[oc] - obj[0], b.uy[oc] - obj[1]) > 0.11:
                    continue
                n += 1
                f1, k1, _s, _b, g1 = search(SWEEP_02, obj)
                with self.subTest(r=r, deg=deg):
                    self.assertTrue(f0 and f1)
                    self.assertEqual([(p.x, p.y, p.yaw) for p, _i in g0], [(p.x, p.y, p.yaw) for p, _i in g1])
        self.assertGreaterEqual(n, 10)


class TimeTests(unittest.TestCase):
    def test_time_left_drops_sweep_looks_that_do_not_fit(self):
        b, _oc, _go, _seen = sim_after_looks(SWEEP_02)
        visited = set(b.visited)
        pose, info = b.next_goal(time_left_s=2.0)                 # less than one look (3 s)
        self.assertIsNone(pose)
        self.assertEqual(info["stop"], "mass_below_stop")
        self.assertGreater(info["sweep_no_time"], 0)
        b.visited, b.stop = set(visited), None
        pose, info = b.next_goal(time_left_s=200.0)
        self.assertIsNotNone(pose)
        self.assertEqual(info["sweep"], "mass_below_stop")
        self.assertLessEqual(sweep_eta(info["path_m"], info["turn_rad"]), 200.0)
        self.assertAlmostEqual(sweep_eta(1.0, 0.0), 1.0 / 0.10 + 3.0)
        # before the stop rule fires the time left changes nothing (only sweep looks are timed)
        picks = []
        for t_left in (None, 1.0):
            b = BeliefSearch(LIVING, LIVING_NODES, SearchConfig(), robot=START)
            b.set_camera(K, SIZE, mount())
            b.begin(TARGET)
            b.apply_base_view(*START)
            pose, info = b.next_goal(time_left_s=t_left)
            self.assertNotIn("sweep", info)
            picks.append(pose)
        self.assertEqual(picks[0], picks[1])

    def test_task_passes_its_remaining_time(self):
        class Planner(object):
            def __init__(self):
                self.calls = []

            def begin(self, xy):
                pass

            def next_goal(self, time_left_s=None):
                self.calls.append(time_left_s)
                return (PoseGoal(1., 0., 0.) if len(self.calls) == 1 else None), {"stop": "mass_below_stop"}

            def mark_failed(self, pose):
                pass

        class Old(Planner):
            def next_goal(self):                                        # noqa: D401 - the pre-10-29 signature
                self.calls.append("none")
                return None, {"stop": "no_candidate"}

        class Mem(object):
            def query_memory(self, goal, after=None):
                return []

        self.assertTrue(_takes_time_left(Planner()))
        self.assertFalse(_takes_time_left(Old()))
        p = Planner()
        task = TaskController(Mem(), lambda row: [], (PoseGoal(5., 0., 0.),),
                              config=replace(TaskConfig(), task_timeout_s=300.0), search_planner=p)
        from test_search_prob_20261026 import Stamp
        cmds = task.start("cup", 10.0, Stamp("s", 10.0))
        self.assertEqual(cmds[0].pose, PoseGoal(1., 0., 0.))
        self.assertAlmostEqual(p.calls[0], 300.0)
        o = Old()
        task = TaskController(Mem(), lambda row: [], (PoseGoal(5., 0., 0.),), config=TaskConfig(), search_planner=o)
        task.start("cup", 10.0, Stamp("s", 10.0))
        self.assertEqual(o.calls, ["none"])
        self.assertEqual((task.state, task.reason), ("not_found", "search_exhausted"))


class ConfigTests(unittest.TestCase):
    def test_defaults_json_and_bad_values(self):
        c = SearchConfig()
        self.assertEqual((c.stop_sweep, c.sweep_min_m2, c.stop_unseen), (True, 0.02, 0.01))
        c = SearchConfig.from_doc({"stop_sweep": False, "sweep_min_m2": 0.05})
        self.assertEqual((c.stop_sweep, c.sweep_min_m2), (False, 0.05))
        self.assertEqual(SearchConfig().record()["stop_sweep"], True)
        for bad in ({"stop_sweep": 1}, {"sweep_min_m2": -0.1}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                SearchConfig.from_doc(bad)


if __name__ == "__main__":
    unittest.main()
