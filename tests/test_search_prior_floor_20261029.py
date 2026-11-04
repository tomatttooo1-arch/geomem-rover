"""10-29 search prior floor (host/search_belief.py): a remembered target's prior keeps ``uniform_floor`` (0.3) spread
over the whole searched area, the mass stop waits until the area is looked over (``stop_unseen``), and a look that
newly sees ``explore_m2`` of unseen area stays a candidate in a large area.

Scene: a 6 x 5 m living room (sofa, table, cabinet), the car's mapping loop as trajectory nodes, the car's 10-28
camera mount. The car starts 1 m from the remembered place facing it; each look applies the views while turning and
driving (as the car's 5 Hz frames do) and the goal view; the object is seen when a view lowers its cell's miss by at
least half (q >= 0.5). Known answers: objects 0.5-1.5 m from the remembered place are found exactly as without the
floor (same looks); objects 2.5-4 m away on the trajectory side, which the old prior stopped without seeing, are
found; with nothing there the search looks over >= 95 % of the area and ends by itself without re-looks; the heat map
keeps the remembered place red. Host only, numpy, no ROS.
"""
import math
import os
import sys
import unittest
from types import SimpleNamespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from host.search_belief import BeliefSearch, SearchConfig  # noqa: E402

RES = 0.05
K = np.array([[517.2, 0., 320.], [0., 517.2, 240.6], [0., 0., 1.]])
SIZE = (640, 480)
# the prior and stop before 10-29 (10-29 stop sweep off too: tests/test_search_stop_sweep_20261029.py)
OLD = SearchConfig(uniform_floor=0.0, stop_unseen=1.0, explore_m2=0.0, stop_sweep=False)


def mount(tilt_deg=15.75, height_m=0.392, x_m=0.10):
    r0 = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    t = math.radians(tilt_deg)
    ry = np.array([[math.cos(t), 0., math.sin(t)], [0., 1., 0.], [-math.sin(t), 0., math.cos(t)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = ry @ r0, [x_m, 0., height_m]
    return T


def room(w_m, h_m, boxes, ox=-0.5, oy=-0.5):
    cells = np.full((int(round(h_m / RES)), int(round(w_m / RES))), -1, np.int8)
    cells[2:-2, 2:-2] = 0
    cells[2:4, 2:-2] = cells[-4:-2, 2:-2] = cells[2:-2, 2:4] = cells[2:-2, -4:-2] = 100
    for x0, y0, x1, y1 in boxes:
        cells[int((y0 - oy) / RES):int((y1 - oy) / RES), int((x0 - ox) / RES):int((x1 - ox) / RES)] = 100
    return SimpleNamespace(cells=cells, resolution=RES, origin_x=ox, origin_y=oy, origin_yaw=0.)


def loop_nodes(corners, step=0.5):
    out = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
        n = max(1, int(round(math.hypot(x1 - x0, y1 - y0) / step)))
        out += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, 0.) for i in range(n)]
    return out


LIVING = room(6.0, 5.0, ((0.0, 3.4, 2.2, 4.3), (2.4, 1.8, 3.4, 2.6), (4.9, 0.0, 5.4, 1.5)))
LIVING_NODES = loop_nodes([(0.6, 0.6), (4.2, 0.6), (4.2, 3.4), (3.6, 3.4), (3.6, 2.9), (0.6, 2.9), (0.6, 0.6)])
TARGET = (4.6, 2.2)            # remembered: by the right wall
START = (3.6, 2.2, 0.)         # the observation pose 1 m away, facing it


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def search(cfg, obj, free_map=LIVING, nodes=LIVING_NODES, target=TARGET, start=START, max_looks=40):
    """Run the search loop; returns (found, looks, stop, planner, goals)."""
    b = BeliefSearch(free_map, nodes, cfg, robot=start)
    b.set_camera(K, SIZE, mount())
    oc = int(np.argmin((b.ux - obj[0]) ** 2 + (b.uy - obj[1]) ** 2))
    inside = math.hypot(b.ux[oc] - obj[0], b.uy[oc] - obj[1]) < 0.11
    b.begin(target)

    def view(x, y, yaw):
        before = b.miss[oc]
        b.apply_base_view(x, y, yaw)
        return inside and b.miss[oc] <= 0.5 * before + 1e-12

    found = view(*start)
    rx, ry, ryaw = start
    goals, stop = [], None
    while not found and len(goals) < max_looks:
        pose, info = b.next_goal()
        if pose is None:
            stop = info["stop"]
            break
        goals.append((pose, info))
        if math.hypot(pose.x - rx, pose.y - ry) > .05:
            head = math.atan2(pose.y - ry, pose.x - rx)
            for a in np.arange(0., abs(wrap(head - ryaw)), 0.4)[1:]:
                found |= view(rx, ry, ryaw + math.copysign(a, wrap(head - ryaw)))
            n = max(1, int(math.hypot(pose.x - rx, pose.y - ry) / 0.3))
            for i in range(1, n + 1):
                found |= view(rx + (pose.x - rx) * i / n, ry + (pose.y - ry) * i / n, head)
            ryaw = head
        for a in np.arange(0., abs(wrap(pose.yaw - ryaw)), 0.4)[1:]:
            found |= view(pose.x, pose.y, ryaw + math.copysign(a, wrap(pose.yaw - ryaw)))
        found |= view(pose.x, pose.y, pose.yaw)
        b.robot = rx, ry, ryaw = pose.x, pose.y, pose.yaw
    return found, len(goals), stop, b, goals


class PriorTests(unittest.TestCase):
    def test_floor_share_and_near_still_highest(self):
        b = BeliefSearch(LIVING, LIVING_NODES)
        b.begin(TARGET)
        new = b.prior.copy()
        self.assertAlmostEqual(float(new.sum()), 1.0, places=9)
        b2 = BeliefSearch(LIVING, LIVING_NODES, OLD)
        b2.begin(TARGET)
        old = b2.prior
        w = np.where(b.edge, SearchConfig().edge_weight, 1.)
        np.testing.assert_allclose(new, 0.7 * old + 0.3 * w / w.sum(), atol=1e-12)
        d = np.hypot(b.ux - TARGET[0], b.uy - TARGET[1])
        self.assertEqual(int(np.argmax(new)), int(np.argmax(old)))                  # the peak stays where it was
        self.assertLess(d[int(np.argmax(new))], 0.6)                                 # (a wall-side cell next to it)
        far = d > 3.0                                                                 # 3 m+ away: was ~0, now real
        self.assertLess(float(old[far].sum()), 0.01)
        self.assertGreater(float(new[far].sum()), 0.1)
        # edge weight still applies to the floor
        self.assertAlmostEqual(new[far & b.edge].max() / new[far & ~b.edge].min(), 3.0, delta=0.6)

    def test_no_memory_unchanged_and_config(self):
        b = BeliefSearch(LIVING, LIVING_NODES)
        b.begin(None)
        w = np.where(b.edge, SearchConfig().edge_weight, 1.)
        np.testing.assert_allclose(b.prior, w / w.sum(), atol=1e-12)
        cfg = SearchConfig.from_doc({"uniform_floor": 0.2, "stop_unseen": 0.05, "explore_m2": 0.5})
        self.assertEqual((cfg.uniform_floor, cfg.stop_unseen, cfg.explore_m2), (0.2, 0.05, 0.5))
        self.assertEqual(SearchConfig().record()["uniform_floor"], 0.3)
        for bad in ({"uniform_floor": 1.0}, {"uniform_floor": -0.1}, {"stop_unseen": 1.5}, {"explore_m2": -1}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                SearchConfig.from_doc(bad)

    def test_heat_map_keeps_the_remembered_place_red(self):
        b = BeliefSearch(LIVING, LIVING_NODES)
        b.begin(TARGET)
        meta, data = b.belief_cells(0.2, relative="prior")
        ii, jj = np.nonzero(data >= 0)
        x = meta["origin"][0] + (jj + .5) * meta["resolution"]
        y = meta["origin"][1] + (ii + .5) * meta["resolution"]
        d = np.hypot(x - TARGET[0], y - TARGET[1])
        v = data[ii, jj]
        self.assertEqual(int(v.max()), 100)
        self.assertGreater(np.median(v[d < 0.5]), 50)
        self.assertLess(np.median(v[d > 3.0]), 10)          # far: faint but present
        self.assertGreater(np.median(v[d > 3.0]), 0)


class BehaviourTests(unittest.TestCase):
    def test_near_objects_found_as_before(self):
        """0.5 / 1.0 / 1.5 m around the remembered place, 8 directions: the same looks as without the floor."""
        n = 0
        for r in (0.5, 1.0, 1.5):
            for deg in range(0, 360, 45):
                obj = (TARGET[0] + r * math.cos(math.radians(deg)), TARGET[1] + r * math.sin(math.radians(deg)))
                f_old, k_old, _s, b, _g = search(OLD, obj)
                oc = int(np.argmin((b.ux - obj[0]) ** 2 + (b.uy - obj[1]) ** 2))
                if math.hypot(b.ux[oc] - obj[0], b.uy[oc] - obj[1]) > 0.11:
                    continue                                       # in a wall / outside the searched area
                n += 1
                f_new, k_new, _s, _b, _g = search(SearchConfig(), obj)
                with self.subTest(r=r, deg=deg):
                    self.assertTrue(f_old and f_new)
                    self.assertLessEqual(k_new, k_old + 1)
        self.assertGreaterEqual(n, 15)

    def test_far_object_on_the_trajectory_side_is_found(self):
        for obj in ((2.1, 2.3), (1.5, 0.2), (0.3, 0.3)):          # 2.5 m, 3.4 m, 4.6 m from the remembered place
            with self.subTest(obj=obj):
                f_old, _k, stop_old, _b, _g = search(OLD, obj)
                self.assertFalse(f_old)                            # the old prior stopped first
                self.assertEqual(stop_old, "mass_below_stop")
                f_new, k_new, _s, _b, goals = search(SearchConfig(), obj)
                self.assertTrue(f_new)
                self.assertLessEqual(k_new, 25)
                # the remembered side first: the first 4 looks within 1.5 m of the remembered place
                self.assertTrue(all(math.hypot(p.x - TARGET[0], p.y - TARGET[1]) < 1.5 for p, _i in goals[:4]))

    def test_nothing_there_looks_over_the_area_and_ends(self):
        found, looks, stop, b, goals = search(SearchConfig(), (99., 99.))
        self.assertFalse(found)
        self.assertIn(stop, ("mass_below_stop", "no_candidate"))
        reach = b.reachable()
        self.assertGreater(float(np.mean(b.miss[reach] <= .5)), 0.95)
        self.assertLess(looks, 40)
        # no re-look round once the area is looked over: at most 2 looks after unseen fell below stop_unseen
        # (10-29: besides the stop sweep's looks, each at a view that still sees new floor -- its own test)
        after = [i for _p, i in goals if i["unseen"] < SearchConfig().stop_unseen and "sweep" not in i]
        self.assertLessEqual(len(after), 2)
        # the old stop left half of it unlooked
        _f, _k, _s, b_old, _g = search(OLD, (99., 99.))
        self.assertLess(float(np.mean(b_old.miss[b_old.reachable()] <= .5)), 0.6)

    def test_mass_stop_waits_for_the_unseen_area(self):
        b = BeliefSearch(LIVING, LIVING_NODES, SearchConfig(stop_mass=0.9), robot=START)
        b.set_camera(K, SIZE, mount())
        b.begin(TARGET)
        b.apply_base_view(*START)
        b.apply_base_view(4.2, 2.0, 0.)
        self.assertLess(b.mass_left(), 0.9)
        pose, info = b.next_goal()
        self.assertIsNotNone(pose)                      # mass below stop_mass, but the area is not looked over yet
        self.assertGreater(info["unseen"], SearchConfig().stop_unseen)
        b = BeliefSearch(LIVING, LIVING_NODES, SearchConfig(stop_mass=0.9, stop_unseen=1.0, stop_sweep=False),
                         robot=START)
        b.set_camera(K, SIZE, mount())
        b.begin(TARGET)
        b.apply_base_view(*START)
        b.apply_base_view(4.2, 2.0, 0.)
        self.assertEqual(b.next_goal()[1]["stop"], "mass_below_stop")    # stop_unseen 1 = the mass alone

    def test_large_area_keeps_exploring_below_min_gain(self):
        """A 9 x 6 m hall: the floor's share per look falls under min_gain before the far side is looked at."""
        hall = room(9.0, 6.0, ((2.0, 1.6, 3.0, 2.4), (5.2, 2.8, 6.2, 3.6), (0.0, 4.6, 1.8, 5.5)))
        nodes = loop_nodes([(0.6, 0.6), (7.4, 0.6), (7.4, 4.4), (0.6, 4.4), (0.6, 0.6)])
        kw = dict(free_map=hall, nodes=nodes, target=(7.9, 2.5), start=(6.9, 2.5, 0.))
        f_new, _k, _s, _b, _g = search(SearchConfig(), (1.5, 2.6), **kw)
        f_off, _k, stop_off, b_off, _g = search(SearchConfig(explore_m2=0.0, stop_sweep=False), (1.5, 2.6), **kw)
        self.assertTrue(f_new)
        self.assertFalse(f_off)
        self.assertEqual(stop_off, "no_candidate")
        self.assertLess(float(np.mean(b_off.miss[b_off.reachable()] <= .5)), 0.9)


if __name__ == "__main__":
    unittest.main()
