"""10-26 probabilistic search (host/search_belief.py) on top of the search MVP.

Known answers on synthetic grids: the prior is higher near the remembered position and along walls; occupied cells
hide what is behind them; a view lowers what it saw and the same look is not counted twice (the next look is not the
same view); a farther look of the same gain scores lower; stop on mass / on no candidate -> not_found/search_exhausted;
no trajectory nodes -> the MVP list; the dense nodes in mapping_ctl's status and the runner's parsing; the preview tool
on a synthetic PGM. Scenario: real core.Memory with rendered planar frames -- the cup moved 1.2 m to the wall, not seen
at the remembered place, found by the probabilistic search within its first three looks (search_single_candidate).
Host only, no ROS, no GPU.
"""
from dataclasses import replace
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools", "car_run"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

import mapping_ctl  # noqa: E402
import search_preview  # noqa: E402
from host.search_belief import BeliefSearch, SearchConfig  # noqa: E402
from host.task_control import PoseGoal, TaskConfig, TaskController  # noqa: E402
from host.ros_runtime import build_search_planner, mapping_search_nodes, SEARCH_MODES  # noqa: E402
from perception.fpga_replay.core import (Config, Detection, Frame, Memory, UNGATED_CREATION,  # noqa: E402
                                        compute_geometry, process_frame)

RES = 0.05


def room(w_m=5.0, h_m=3.6, ox=-0.5, oy=-1.8, walls=()):
    """Free room with a 0.1 m wall frame, unknown outside; ``walls``: extra occupied boxes (x0, y0, x1, y1)."""
    cells = np.full((int(round(h_m / RES)), int(round(w_m / RES))), -1, np.int8)
    cells[2:-2, 2:-2] = 0
    cells[2:4, 2:-2] = cells[-4:-2, 2:-2] = cells[2:-2, 2:4] = cells[2:-2, -4:-2] = 100
    for x0, y0, x1, y1 in walls:
        cells[int((y0 - oy) / RES):int((y1 - oy) / RES), int((x0 - ox) / RES):int((x1 - ox) / RES)] = 100
    return SimpleNamespace(cells=cells, resolution=RES, origin_x=ox, origin_y=oy, origin_yaw=0.)


CORRIDOR = [(0.5 * k, 0.0, 0.0) for k in range(7)]       # y = 0, x 0..3 (room x -0.4..4.4, y -1.7..1.7)


def cell_near(b, x, y):
    return int(np.argmin((b.ux - x) ** 2 + (b.uy - y) ** 2))


class PriorTests(unittest.TestCase):
    def test_near_and_wall_side_are_higher(self):
        b = BeliefSearch(room(), CORRIDOR)
        b.begin((2.0, 0.0))
        p = b.prior
        self.assertAlmostEqual(float(p.sum()), 1.0, places=9)
        self.assertGreater(p[cell_near(b, 2.0, 0.3)], p[cell_near(b, 2.0, 1.0)])     # nearer the remembered place
        # same distance (1.25 m) from the target: the wall-side cell (0.15 m from the wall at y = 1.6) x 3
        wall, inner = cell_near(b, 2.0, 1.45), cell_near(b, 3.25, 0.0)
        self.assertTrue(b.edge[wall])
        self.assertFalse(b.edge[inner])
        self.assertGreater(p[wall], 2.0 * p[inner])
        # the reserved near mass: most of near_mass within 0.6 m of the remembered position
        d = np.hypot(b.ux - 2.0, b.uy)
        self.assertGreater(float(p[d < .6].sum()), SearchConfig().near_mass)

    def test_uniform_without_memory(self):
        b = BeliefSearch(room(), CORRIDOR)
        b.begin(None)
        p = b.prior
        self.assertAlmostEqual(p[~b.edge].max(), p[~b.edge].min())
        self.assertAlmostEqual(p[b.edge].max() / p[~b.edge].max(), SearchConfig().edge_weight)

    def test_universe_is_only_seen_free_space_near_the_trajectory(self):
        b = BeliefSearch(room(), [(0.0, 0.0, 0.0)])
        self.assertLessEqual(float(np.hypot(b.ux, b.uy).max()), SearchConfig().max_range_m + 1e-9)
        cfg = SearchConfig.from_doc({"q_table": {"0.5": 0.8, "1.0": 0.5}, "max_range_m": 1.0})
        self.assertEqual(cfg.q_table, ((0.5, 0.8), (1.0, 0.5)))
        np.testing.assert_allclose(BeliefSearch(room(), CORRIDOR, cfg).q([.2, .75, 1.0, 1.01]), [.8, .65, .5, 0.])
        with self.assertRaises(ValueError):
            SearchConfig.from_doc({"bogus": 1})
        with self.assertRaises(ValueError):
            SearchConfig(q_table=((1.0, 0.5), (0.5, 0.8)))


class ViewTests(unittest.TestCase):
    def test_wall_hides_what_is_behind_it(self):
        # a wall x = 1.0..1.1 across y -0.6..0.6; nodes at x = 0 and x = 2 (each side)
        m = room(walls=[(1.0, -0.6, 1.1, 0.6)])
        b = BeliefSearch(m, [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)])
        b.begin(None)
        before = b.miss.copy()
        b.apply_view(0.0, 0.0, 0.0)                     # look +x from x = 0 at the wall
        behind, front = cell_near(b, 1.6, 0.0), cell_near(b, 0.6, 0.0)
        self.assertEqual(b.miss[behind], before[behind])
        self.assertLess(b.miss[front], before[front])
        # only nodes that see a cell put it in the universe: behind the wall is in (node 2 sees it) but
        # not in node 1's visible set
        idx, _d, _b = b.node_cells[0]
        self.assertNotIn(behind, set(idx.tolist()))
        self.assertIn(behind, set(b.node_cells[1][0].tolist()))

    def test_view_lowers_and_same_look_is_not_counted_twice(self):
        b = BeliefSearch(room(), CORRIDOR, robot=(0.0, 0.0, 0.0))
        b.begin((2.0, 0.0))
        pose, info = b.next_goal()
        n, k = info["node"], b.yaw_set.index(pose.yaw)
        g0 = b.gains()[n, k]
        b.apply_view(pose.x, pose.y, pose.yaw)
        m1 = b.mass_left()
        self.assertAlmostEqual(1 - m1, g0, places=6)    # the view removed exactly the gain it was chosen for
        self.assertLess(b.gains()[n, k], 1e-9)          # nothing left to gain from that view
        b.apply_view(pose.x, pose.y, pose.yaw)          # the same look again (consecutive frames): not counted
        self.assertAlmostEqual(b.mass_left(), m1, places=9)
        b.apply_view(pose.x + .1, pose.y, pose.yaw)     # 0.1 m on (within look_move_m): only the better rates count
        self.assertLess(m1 - b.mass_left(), .2 * (1 - m1))
        nxt, _ = b.next_goal()
        self.assertNotEqual((nxt.x, nxt.y, nxt.yaw), (pose.x, pose.y, pose.yaw))

    def test_observe_rate_limit_and_untrusted_pose(self):
        b = BeliefSearch(room(), CORRIDOR)
        T = cam_T(0.0, 0.0, 0.0)
        self.assertTrue(b.observe(T, 1.0))
        self.assertFalse(b.observe(T, 1.1))             # 5 Hz
        self.assertFalse(b.observe(cam_T(1.5, 0, 0), 1.5, trusted=False))
        self.assertTrue(b.observe(cam_T(1.5, 0, 0), 1.6, base_pose=(1.4, 0, 0)))
        self.assertEqual((b.views, b.views_skipped, b.robot), (2, 1, (1.4, 0.0, 0.0)))

    def test_belief_message_for_the_ui(self):
        b = BeliefSearch(room(), CORRIDOR, robot=(0.0, 0.0, 0.0))
        b.begin((2.0, 0.0))
        b.next_goal()
        msg = json.loads(json.dumps(b.belief_message(), allow_nan=False))
        self.assertEqual(len(msg["data"]), msg["width"] * msg["height"])
        self.assertEqual((msg["frame"], msg["resolution"], len(msg["goals"])), ("map", 0.2, 1))
        self.assertEqual(max(msg["data"]), 100)
        self.assertIn(-1, msg["data"])
        json.dumps(b.summary(compact=True), allow_nan=False)

    def test_miss_floor(self):
        b = BeliefSearch(room(), CORRIDOR)
        b.begin(None)
        for k in range(12):
            b.apply_view(0.6 * (k % 6), 0.0, 0.0)
            b.apply_view(0.6 * (k % 6), 0.0, math.pi)
        self.assertGreaterEqual(float(b.miss.min()), SearchConfig().miss_floor - 1e-12)


class SelectionTests(unittest.TestCase):
    def test_score_falls_with_path_length(self):
        # mirror-symmetric room, uniform prior: the view at x = 0.5 facing +x and the one at x = 3.5 facing -x gain
        # the same; the car at x = 0.5 -> the near one
        m = room(w_m=4.8, ox=-0.4)
        b = BeliefSearch(m, [(0.5, 0.0, 0.0), (3.5, 0.0, 0.0)], robot=(0.5, 0.0, 0.0))
        b.begin(None)
        g = b.gains()
        k0, kpi = b.yaw_set.index(0.0), b.yaw_set.index(math.pi)
        self.assertAlmostEqual(g[0, k0], g[1, kpi], places=6)
        paths = b.path_lengths((0.5, 0.0))
        self.assertAlmostEqual(paths[0], 0.0)
        self.assertAlmostEqual(paths[1], 3.0, delta=.15)
        pose, info = b.next_goal()
        self.assertEqual((pose.x, info["path_m"]), (0.5, 0.0))
        # the mirror start: the far end's view wins
        b.reset()
        b.begin(None)
        b.robot = (3.5, 0.0, math.pi)
        pose, info = b.next_goal()
        self.assertEqual((pose.x, info["path_m"]), (3.5, 0.0))
        # same gain, longer path -> lower score (score = gain / (path + turn + step))
        cfg = b.cfg
        self.assertGreater(g[0, k0] / (0 + cfg.step_cost_m), g[1, kpi] / (paths[1] + cfg.step_cost_m))

    def test_unreachable_and_failed_nodes_are_not_offered(self):
        # node 2 sealed in a closed box: unreachable from the car
        m = room(walls=[(2.6, -1.2, 2.7, 1.2), (3.6, -1.2, 3.7, 1.2), (2.6, -1.2, 3.7, -1.1), (2.6, 1.1, 3.7, 1.2)])
        b = BeliefSearch(m, [(0.5, 0.0, 0.0), (3.15, 0.0, 0.0)], robot=(0.5, 0.0, 0.0))
        self.assertTrue(math.isinf(b.path_lengths((0.5, 0.0))[1]))
        for _ in range(8):
            pose, _info = b.next_goal()
            if pose is None:
                break
            self.assertEqual(pose.x, 0.5)
        b.reset()
        b.mark_failed(PoseGoal(0.5, 0.0, 1.0))
        pose, info = b.next_goal()
        self.assertEqual((pose, info["stop"]), (None, "no_candidate"))

    def test_stop_on_mass_and_controller_word(self):
        # 10-29: stop_unseen 0.99 and no stop sweep isolate the mass rule (the unseen part:
        # test_search_prior_floor_20261029, the sweep: test_search_stop_sweep_20261029)
        b = BeliefSearch(room(), CORRIDOR, SearchConfig(stop_mass=0.8, stop_unseen=0.99, stop_sweep=False),
                         robot=(0.0, 0.0, 0.0))
        b.begin((2.0, 0.0))
        b.apply_view(1.0, 0.0, 0.0)
        b.apply_view(2.5, 0.0, math.pi)
        self.assertLess(b.mass_left(), 0.8)
        pose, info = b.next_goal()
        self.assertEqual((pose, info["stop"]), (None, "mass_below_stop"))
        # through the controller: an unknown object, the planner stops -> not_found/search_exhausted
        b = BeliefSearch(room(), CORRIDOR, SearchConfig(stop_mass=0.99, stop_unseen=0.99, stop_sweep=False),
                         robot=(0.0, 0.0, 0.0))
        b.apply_view(1.0, 0.0, 0.0)                     # seen before the search began (miss kept, prior uniform)
        task = TaskController(Memory("geometry"), lambda row: [], (), search_planner=b)
        out = task.start("cup", 0, Stamp("f0", 0))
        self.assertEqual((task.state, task.reason, out[0].kind), ("not_found", "search_exhausted", "stop"))
        self.assertEqual(task.events[-2]["reason"], "search_planner_stop")


class Stamp(SimpleNamespace):
    def __init__(self, frame_id, timestamp, pose_valid=True):
        super().__init__(frame_id=frame_id, timestamp=timestamp, pose_valid=pose_valid)


class FakePlanner(object):
    def __init__(self, goals):
        self.goals, self.begun, self.failed = list(goals), [], []

    def begin(self, xy):
        self.begun.append(xy)

    def next_goal(self):
        return (self.goals.pop(0), {"n": 1}) if self.goals else (None, {"stop": "no_candidate"})

    def mark_failed(self, pose):
        self.failed.append(pose)


class ControllerTests(unittest.TestCase):
    def test_planner_replaces_the_list_and_failed_goal_is_reported(self):
        p = FakePlanner([PoseGoal(1, 0, 0), PoseGoal(2, 0, 0)])
        task = TaskController(Memory("geometry"), lambda row: [], (PoseGoal(9, 9, 0),),
                              config=TaskConfig(nav_retries_same_goal=0, min_attempt_time_s=0, retry_pause_s=.1,
                                                observation_timeout_s=.5),
                              search_planner=p)
        cmd = task.start("cup", 0, Stamp("f0", 0))[0]
        self.assertEqual((p.begun, cmd.pose), ([None], PoseGoal(1, 0, 0)))
        self.assertEqual(task.events[-2]["reason"], "search_goal_selected")
        out = task.navigation_result(cmd.command_id, False, .5)
        self.assertEqual((out[0].kind, p.failed), ("recover", [PoseGoal(1, 0, 0)]))
        task.observe(Stamp("f1", .6), .6)
        cmd = task.tick(.7)[0]
        self.assertEqual(cmd.pose, PoseGoal(2, 0, 0))
        self.assertTrue(task.navigation_result(cmd.command_id, True, .8) == [])
        task.observe(Stamp("f2", 1.0), 1.0)
        task.tick(1.4)
        self.assertEqual((task.state, task.reason), ("not_found", "search_exhausted"))

    def test_zero_observation_poses_start_the_belief_search_at_the_remembered_place(self):
        """10-26 (owner): no observation pose around a known target -> the real planner, prior at its position."""
        row = {"id": "object-0001", "label": "cup", "state": "historical_memory", "position_m": [2.0, 0.0, 0.1]}
        mem = SimpleNamespace(query_memory=lambda goal, after=None: [dict(row)])
        b = BeliefSearch(room(), CORRIDOR, robot=(0.0, 0.0, 0.0))
        task = TaskController(mem, lambda r: [], (), search_planner=b)
        out = task.start("cup", 0, Stamp("f0", 0))
        self.assertEqual((task.state, task.reason, task.search_trigger), ("navigating", "search", "no_observation_pose"))
        self.assertEqual(out[0].kind, "navigate")
        self.assertEqual(b.summary()["target_xy"], [2.0, 0.0])

    def test_planner_fault_falls_back_to_the_mvp_list(self):
        class Broken(FakePlanner):
            def next_goal(self):
                raise RuntimeError("boom")
        task = TaskController(Memory("geometry"), lambda row: [], (PoseGoal(5, 0, 0),), search_planner=Broken([]))
        cmd = task.start("cup", 0, Stamp("f0", 0))[0]
        self.assertEqual((cmd.kind, cmd.pose, task.search_planner), ("navigate", PoseGoal(5, 0, 0), None))
        self.assertEqual(task.events[-2]["reason"], "search_planner_error")

    def test_mvp_fallback_without_nodes(self):
        self.assertEqual(SEARCH_MODES, ("prob", "mvp"))
        planner, info = build_search_planner("prob", room(), None, None, (0, 0, 0))
        self.assertEqual((planner, info["used"], info["reason"]), (None, "mvp", "no_search_nodes"))
        planner, info = build_search_planner("mvp", room(), CORRIDOR, None, (0, 0, 0))
        self.assertEqual((planner, info["reason"]), (None, "search_mode_mvp"))
        planner, info = build_search_planner("prob", None, CORRIDOR, None, (0, 0, 0))
        self.assertEqual(info["reason"], "no_map")
        bad = SimpleNamespace(cells=np.full((10, 10), 100, np.int8), resolution=.05, origin_x=0, origin_y=0)
        planner, info = build_search_planner("prob", bad, [(0.2, 0.2, 0)], None, (0, 0, 0))
        self.assertIsNone(planner)
        self.assertTrue(info["reason"].startswith("belief_build_failed"))
        cache = {}
        m = room()
        p1, info = build_search_planner("prob", m, CORRIDOR, None, (0, 0, 0), cache)
        p1.apply_view(0, 0, 0)
        p2, info2 = build_search_planner("prob", m, CORRIDOR, None, (1, 0, 0), cache)
        self.assertIs(p1, p2)
        self.assertNotIn("built", info2)
        self.assertEqual((p2.views, p2.robot, p2.began()), (0, (1, 0, 0), False))     # reset per task
        # no planner: the MVP list, exactly as before (the MVP tests pin its order)
        task = TaskController(Memory("geometry"), lambda row: [], (PoseGoal(5, 0, 0),))
        self.assertEqual(task.start("cup", 0, Stamp("f0", 0))[0].pose, PoseGoal(5, 0, 0))

    def test_runtime_wiring(self):
        src = open(os.path.join(ROOT, "host", "ros_runtime.py"), encoding="utf8").read()
        for s in ('"--search-mode", choices=SEARCH_MODES, default="prob"', "search_planner=self.search_planner",
                  "self.search_planner.observe(", 'extra["search"] = planner.summary(compact=True)',
                  'self.record("search_goal_selected"', "mapping_search_nodes(msg.data)"):
            self.assertIn(s, src)
        # the planner sees the frame before the task does (its next goal knows what this frame saw)
        self.assertLess(src.index("self.search_planner.observe("),
                        src.index("self.execute(self.task.observe(frame, self.now(), robot_xy="))
        inner = open(os.path.join(ROOT, "tools", "car_run", "car_inner.sh"), encoding="utf8").read()
        self.assertIn('RX+=(--search-mode "${SEARCH_MODE:-prob}")', inner)
        up = open(os.path.join(ROOT, "tools", "car_run", "car_up.sh"), encoding="utf8").read()
        self.assertIn('INNER_ENV+=(SEARCH_MODE="$CAR_SEARCH_MODE")', up)
        cfg = SearchConfig.from_file(os.path.join(ROOT, "tools", "car_run", "search_config.json"))
        self.assertEqual(cfg.q_table, SearchConfig().q_table)


class MappingNodesTests(unittest.TestCase):
    def test_dense_nodes(self):
        nodes = [(i, 0.1 * i, 0.0, 0.3) for i in range(60)]          # 6 m in 0.1 m steps
        dn = mapping_ctl.dense_nodes(nodes)
        self.assertEqual(len(dn), 12)
        self.assertEqual(dn[0], [0.0, 0.0, 0.3])
        self.assertTrue(all(abs((b[0] - a[0]) - .5) < 1e-9 for a, b in zip(dn, dn[1:])))
        back = nodes + [(60 + i, 5.9 - 0.1 * i, 0.0, 0.) for i in range(60)]   # driven back: kept once
        self.assertEqual(len(mapping_ctl.dense_nodes(back)), 12)
        self.assertEqual(len(mapping_ctl.dense_nodes([(i, 0.5 * i, 0, 0) for i in range(200)])), 80)

    def test_status_field_and_runner_parsing(self):
        from test_map_align_20261025 import write_graph
        with tempfile.TemporaryDirectory() as d:
            _poses, db = write_graph(d)
            with open(os.path.join(d, "export", "run.yaml"), "w") as f:
                f.write("image: run.pgm\n")
            core = mapping_ctl.MappingCore(d, d, db=db)
            core.request_stop()
            core.on_child_exit(0)
            st = core.status()
            self.assertTrue(st["search_nodes"]["ok"], st["search_nodes"])
            self.assertIn("search_waypoints", st)                           # the MVP's list unchanged
            nodes = mapping_search_nodes(json.dumps(st))
            self.assertEqual(len(nodes), len(st["search_nodes"]["nodes"]))
        core = mapping_ctl.MappingCore("/nonexistent", "/nonexistent", standin=True)
        core.request_stop()
        core.on_child_exit(0)
        self.assertFalse((core.status()["search_nodes"] or {"ok": False})["ok"])
        for text in ("x", "{}", json.dumps({"search_nodes": {"ok": False, "nodes": [[1, 2, 3]]}}),
                     json.dumps({"search_nodes": {"ok": True, "nodes": [[1, 2]]}})):
            self.assertIsNone(mapping_search_nodes(text))


class PreviewToolTests(unittest.TestCase):
    def test_preview_on_synthetic_map(self):
        m = room(walls=[(1.5, 0.6, 2.5, 0.8)])
        with tempfile.TemporaryDirectory() as d:
            img = np.where(m.cells[::-1] >= 50, 0, np.where(m.cells[::-1] == 0, 254, 205)).astype(np.uint8)
            with open(os.path.join(d, "map.pgm"), "wb") as f:
                f.write(b"P5\n# synthetic\n%d %d\n255\n" % (img.shape[1], img.shape[0]) + img.tobytes())
            with open(os.path.join(d, "map.yaml"), "w") as f:
                f.write("image: map.pgm\nresolution: %.3f\norigin: [%.2f, %.2f, 0.0]\nnegate: 0\n"
                        "occupied_thresh: 0.65\nfree_thresh: 0.196\n" % (RES, m.origin_x, m.origin_y))
            loaded = search_preview.load_map(os.path.join(d, "map.yaml"))
            np.testing.assert_array_equal(loaded.cells, m.cells)
            with open(os.path.join(d, "status.json"), "w") as f:
                json.dump({"search_nodes": {"ok": True, "nodes": [list(n) for n in CORRIDOR]}}, f)
            png, js = os.path.join(d, "p.png"), os.path.join(d, "p.json")
            r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "car_run", "search_preview.py"),
                                "--map", os.path.join(d, "map.yaml"), "--nodes", os.path.join(d, "status.json"),
                                "--target", "2.0", "0.0", "--png", png, "--json", js,
                                "--config", os.path.join(ROOT, "tools", "car_run", "search_config.json")],
                               capture_output=True, text=True, timeout=60)
            self.assertEqual(r.returncode, 0, r.stderr)
            with open(png, "rb") as f:
                self.assertEqual(f.read(8), b"\x89PNG\r\n\x1a\n")
            order = json.load(open(js))["order"]
            looks = [o for o in order if "x" in o]
            self.assertGreaterEqual(len(looks), 3)
            cum = [o["cum_prob"] for o in looks]
            self.assertEqual(cum, sorted(cum))
            self.assertEqual([o["n"] for o in looks], list(range(1, len(looks) + 1)))
            self.assertIn("cum_prob", r.stdout)


# ---- scenario: real Memory, rendered planar frames --------------------------------------------------------
W, H, FX, HALF = 320, 200, 172., 0.09          # 86 deg horizontal field of view, a cup-sized cylinder
K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1.]])


def cam_T(x, y, yaw):
    """Optical frame (z forward, x right, y down) at camera height = the cup's centre, level, facing yaw."""
    T = np.eye(4)
    T[:3, 0] = [math.sin(yaw), -math.cos(yaw), 0]
    T[:3, 1] = [0, 0, -1]
    T[:3, 2] = [math.cos(yaw), math.sin(yaw), 0]
    T[:3, 3] = [x, y, HALF]
    return T


def render(i, t, pose, cup):
    """Background 3 m away; the cup (a cylinder: the same width from any side) drawn and detected when in front,
    within 2 m and wholly in the image -- what an oracle detector with a 2 m range would report."""
    T = cam_T(*pose)
    rgb = np.full((H, W, 3), 170, np.uint8)
    depth = np.full((H, W), 3.0, np.float32)
    dets = []
    if cup is not None:
        c = (np.linalg.inv(T) @ np.array([cup[0], cup[1], HALF, 1.]))[:3]
        if .1 < c[2] <= 2.0:
            u, v, hw = FX * c[0] / c[2] + W / 2, FX * c[1] / c[2] + H / 2, FX * HALF / c[2]
            l, r, tp, b = int(u - hw), int(u + hw), int(v - hw), int(v + hw)
            if l >= 0 and r < W and tp >= 0 and b < H:
                depth[tp:b, l:r] = c[2]
                rgb[tp:b, l:r] = [230, 20, 20]
                dets = [Detection("cup", 1., (l, tp, r, b), "synthetic_oracle_box", "f%d" % i, t)]
    return Frame("f%d" % i, t, rgb, depth, K, T, True, "synthetic_known_camera", "synthetic_pinhole", t), dets


class MovedCupScenarioTests(unittest.TestCase):
    def test_moved_to_the_wall_found_within_three_looks(self):
        mem = Memory("geometry", replace(Config(), **UNGATED_CREATION))

        def step(i, t, pose, cup):
            frame, dets = render(i, t, pose, cup)
            mem.update_memory(process_frame(frame, dets), compute_geometry(frame, mem), frame)
            return frame

        step(0, 0.0, (0.5, 0.0, 0.0), (2.0, 0.0))                   # mapped/remembered at (2, 0)
        self.assertEqual(mem.snapshot()[0]["state"], "current_confirmed")
        cup = (2.0, 1.2)                                             # moved 1.2 m, 0.3 m from the wall (y = 1.6)
        planner = BeliefSearch(room(), CORRIDOR, robot=(0.5, 0.0, 0.0))
        task = TaskController(mem, lambda row: [PoseGoal(0.8, 0.0, 0.0)], (), config=TaskConfig(),
                              search_planner=planner)
        t, i = 0.1, 1
        cmds = task.start("cup", t, Stamp("s", t))
        looks = []
        for _ in range(12):
            cmd = cmds[0]
            self.assertEqual(cmd.kind, "navigate")
            if task.reason == "search":
                looks.append((cmd.pose.x, cmd.pose.y, round(math.degrees(cmd.pose.yaw))))
            pose = (cmd.pose.x, cmd.pose.y, cmd.pose.yaw)
            t += 1.0
            self.assertEqual(task.navigation_result(cmd.command_id, True, t), [])
            cmds = []
            for _ in range(20):                                       # 0.2 s frames, observation 3 s
                t += 0.2
                i += 1
                frame = step(i, t, pose, cup)
                planner.observe(frame.T_world_camera, t, base_pose=pose)
                cmds = task.observe(frame, t, robot_xy=pose[:2]) or task.tick(t)
                if cmds or task.state in task.TERMINAL:
                    break
            if task.state in task.TERMINAL:
                break
        # 10-29: with memory existence_v3 (10-27) the free views at the remembered place drop the cup to
        # position_invalid within ~1 s, before the not-seen count -- the search starts on that word -- and the cup
        # seen again 1.2 m away keeps its id (an invalidated track takes over the new same-class sighting; 10-28
        # far association is a move), so the task ends found on the remembered object itself
        self.assertEqual(task.search_trigger, "target_position_invalid")
        self.assertEqual((task.state, task.reason, task.target_id), ("found", "fresh_matching_observation",
                                                                     "object-0001"))
        self.assertTrue(1 <= len(looks) <= 3, looks)
        self.assertAlmostEqual(task.found_row["position_m"][1], 1.2, delta=.05)
        s = planner.summary()
        self.assertEqual((s["mode"], s["target_xy"], len(s["goals"])), ("prob", [2.0, 0.009], len(looks)))
        self.assertLess(s["mass_left"], 1.0)


class TimingTests(unittest.TestCase):
    def test_update_and_select_cost(self):
        """Host cost on an 80-node trajectory in a 6 x 5 m room (printed; loose bounds only)."""
        nodes = [(0.2 + 0.5 * (k % 10), 0.3 + 0.5 * (k // 10), 0.) for k in range(80)]
        m = room(w_m=6.0, h_m=5.0, ox=-0.4, oy=-0.4)
        t0 = time.perf_counter()
        b = BeliefSearch(m, nodes, robot=(0.2, 0.3, 0.))
        build = time.perf_counter() - t0
        b.begin((2.5, 2.0))
        t0 = time.perf_counter()
        for k in range(50):
            b.apply_view(0.2 + 0.09 * k, 0.3 + 0.07 * k, 0.4 * k)
        upd = (time.perf_counter() - t0) / 50
        t0 = time.perf_counter()
        for _ in range(5):
            b.next_goal()
        sel = (time.perf_counter() - t0) / 5
        print("\nsearch_belief host: build %.0f ms, update %.2f ms, select %.1f ms, cells %d"
              % (build * 1e3, upd * 1e3, sel * 1e3, b.ux.size))
        self.assertLess(upd, 0.05)
        self.assertLess(sel, 1.0)


if __name__ == "__main__":
    unittest.main()


import test_car_run as tcr  # noqa: E402


class CarUpSearchModeTests(tcr.Base):
    """car_up CAR_SEARCH_MODE -> INNER_ENV SEARCH_MODE (container -e) -> car_inner --search-mode (board dry run)."""
    setUp = tcr.TestLiveBoard.setUp
    line = tcr.TestLiveBoard.line
    make_node = tcr.TestLiveBoard.make_node

    def test_default_prob_mvp_passed_bad_refused(self):
        r = self.up('live', '--dry-run', FAKE_USB='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(' -e SEARCH_MODE=prob ', self.line(r.stdout, '[container stack]'))
        self.assertIn('--search-mode prob --search-config ', r.stdout)      # car_inner's runner line
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_SEARCH_MODE='mvp')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(' -e SEARCH_MODE=mvp ', self.line(r.stdout, '[container stack]'))
        run = os.path.join(self.tmp, 'bad')
        r = self.up('live', '--run-dir', run, FAKE_USB='1', CAR_SEARCH_MODE='fast')
        self.assertEqual(r.returncode, 2, r.stdout)
        self.assertIn('CAR_SEARCH_MODE', r.stderr)
        self.assertFalse(os.path.exists(run))
