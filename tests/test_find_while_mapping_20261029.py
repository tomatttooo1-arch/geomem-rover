"""10-29 find while mapping (owner: the acceptance may never press 结束建图): the search has trajectory nodes before an
export (host/trajectory_nodes.LiveTrack), a live alignment that moves the memory moves the track the same way, and a
running task re-plans / re-targets when its target moved under it (TaskController.map_moved). Host only, no ROS."""

from dataclasses import dataclass
from pathlib import Path
import math
import sys
import unittest

import numpy as np

from host import map_align
from host.task_control import PoseGoal, TaskConfig, TaskController
from host.trajectory_nodes import LiveTrack, dense_nodes, search_waypoints
from host.ros_runtime import FreeMap, build_search_planner, select_search_waypoints

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "car_run"))
import mapping_ctl  # noqa: E402


@dataclass
class Stamp:
    frame_id: str
    timestamp: float
    pose_valid: bool = True


TARGET = {"id": "object-0001", "label": "cup", "color": None, "state": "historical_memory",
          "position_m": [2.0, 0.0, 0.05], "bounds_m": [[1.96, -0.04, 0.0], [2.04, 0.04, 0.10]],
          "last_seen": -1.0, "last_seen_frame": "old"}


class Memory(object):
    def __init__(self, rows):
        self.rows = [dict(r) for r in rows]

    def move(self, dx, dy):
        for r in self.rows:
            r["position_m"] = [r["position_m"][0] + dx, r["position_m"][1] + dy, r["position_m"][2]]

    def query_memory(self, goal, after=None):
        return [dict(r, confirmed_after_query=False) for r in self.rows]


def ring(row):
    """Observation planner stand-in: one pose 1 m in front (-x) of the row, facing it."""
    x, y = row["position_m"][:2]
    return [PoseGoal(x - 1.0, y, 0.0), PoseGoal(x, y - 1.0, math.pi / 2)]


def align_doc(dx, dy, yaw=0.0, stamps=(0.0, 100.0)):
    """A map_align alignment document: the same correction (dx, dy, yaw) at every node stamp."""
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    pose7 = [dx, dy, 0.0, q[0], q[1], q[2], q[3]]
    return {"map_odom": pose7, "nodes": [[float(s)] + pose7 for s in stamps]}


class ThinningMovedTests(unittest.TestCase):
    def test_mapping_ctl_reexports_the_same_functions(self):
        self.assertIs(mapping_ctl.dense_nodes, dense_nodes)
        self.assertIs(mapping_ctl.search_waypoints, search_waypoints)
        self.assertEqual((mapping_ctl.NODES_SPACING_M, mapping_ctl.SEARCH_MAX), (0.5, 6))


class LiveTrackTests(unittest.TestCase):
    def test_keeps_moves_and_turns_only(self):
        t = LiveTrack()
        self.assertTrue(t.add(0.0, 0, 0, 0))
        self.assertFalse(t.add(0.1, 0.05, 0, 0))              # 5 cm, no turn
        self.assertTrue(t.add(0.2, 0.12, 0, 0))               # 12 cm
        self.assertTrue(t.add(0.3, 0.12, 0, math.radians(25)))  # turned on the spot
        self.assertFalse(t.add(0.3, 1.0, 0, 0))               # stamp not newer
        self.assertFalse(t.add(0.4, float("nan"), 0, 0))
        self.assertEqual(len(t), 3)

    def test_nodes_feed_the_probabilistic_search_and_the_mvp_list(self):
        t = LiveTrack()
        for i in range(40):                                   # a 4 m drive along x
            t.add(float(i), 0.1 * i, 0.0, 0.0)
        nodes, wps = t.nodes()
        self.assertGreaterEqual(len(nodes), 7)
        self.assertTrue(all(len(n) == 3 for n in nodes))
        self.assertTrue(wps and all(math.hypot(w[0], w[1]) >= 0.5 for w in wps))
        grid = FreeMap(np.zeros((120, 120), dtype=np.int8), 0.05, -1.0, -3.0)    # 6 x 6 m free, known
        planner, info = build_search_planner("prob", grid, nodes, None, (0.0, 0.0, 0.0))
        self.assertEqual(info["used"], "prob", info)
        planner.begin((2.0, 1.0))
        pose, _ = planner.next_goal()
        self.assertIsNotNone(pose)
        goals, sel = select_search_waypoints((), tuple(PoseGoal(*w) for w in wps), "spin", grid, (0.0, 0.0, 0.0),
                                             outward=True)
        self.assertEqual(sel["source"], "mapping_trajectory")
        self.assertEqual(len(goals), 2 * len(wps))

    def test_move_follows_the_memory_alignment(self):
        t = LiveTrack()
        t.add(10.0, 1.0, 0.0, 0.0)
        t.add(20.0, 2.0, 0.0, 0.0)
        first = align_doc(0.3, -0.1)
        t.move(map_align.Delta(first, None).at)
        self.assertTrue(np.allclose([p[1:3] for p in t.points], [[1.3, -0.1], [2.3, -0.1]]))
        second = align_doc(0.0, 0.0, yaw=math.pi / 2)          # D = C_second x C_first^-1, from the first frame
        t.move(map_align.Delta(second, first).at)
        # odometry-frame (1, 0) under a pure 90 deg turn -> (0, 1)
        self.assertTrue(np.allclose(t.points[0][1:4], [0.0, 1.0, math.pi / 2], atol=1e-9), t.points[0])

    def test_a_bad_delta_keeps_the_points(self):
        t = LiveTrack()
        t.add(0.0, 1.0, 2.0, 0.0)

        def bad(_stamp):
            raise ValueError("no")
        t.move(bad)
        self.assertEqual(t.points, [(0.0, 1.0, 2.0, 0.0)])


class MapMovedTaskTests(unittest.TestCase):
    def make(self, **cfg):
        cfg.setdefault("stream_timeout_s", 60.0)     # no frames in these tests: the stream watchdog stays quiet
        self.memory = Memory([TARGET])
        self.task = TaskController(self.memory, ring, (PoseGoal(0, 0, 0),), config=TaskConfig(**cfg))
        return self.task.start("杯子", 0.0, Stamp("f0", 0.0))

    def test_loop_closure_retargets_the_live_goal_without_a_failure(self):
        nav = self.make()[0]
        self.assertEqual((nav.kind, nav.pose.x, nav.pose.y), ("navigate", 1.0, 0.0))
        self.memory.move(0.4, 0.3)                            # a 0.5 m correction moved the cup
        out = self.task.map_moved(2.0)
        self.assertEqual([c.kind for c in out], ["retarget"])
        self.assertEqual(out[0].command_id, nav.command_id)  # the goal being driven to is retired
        self.assertEqual((self.task.state, self.task.reason), ("recovering", TaskController.RETARGET_REASON))
        self.assertEqual(self.task.attempts[-1]["result"], "interrupted")
        self.assertEqual(self.task.attempts_snapshot()["failures"], 0)
        self.assertEqual(self.task.events[-1]["reason"], "map_moved_retarget")
        nxt = self.task.tick(2.0 + self.task.config.retry_pause_s)
        self.assertEqual([c.kind for c in nxt], ["navigate"])
        self.assertTrue(np.allclose((nxt[0].pose.x, nxt[0].pose.y), (1.4, 0.3)))   # around the NEW place
        self.assertFalse(self.task.attempts[-1]["retry"])
        self.assertEqual(self.task.state, "navigating")

    def test_small_move_replans_the_remaining_poses_only(self):
        self.make()
        self.memory.move(0.18, 0.0)
        self.assertEqual(self.task.map_moved(1.0), [])
        self.assertEqual(self.task.state, "navigating")
        self.assertEqual(self.task.events[-1]["reason"], "map_moved_replanned")
        self.assertTrue(np.allclose((self.task._poses[0].x, self.task._poses[0].y), (1.18, 0.0)))

    def test_noise_changes_nothing(self):
        self.make()
        before = list(self.task._poses)
        self.memory.move(0.05, 0.02)
        self.assertEqual(self.task.map_moved(1.0), [])
        self.assertEqual(self.task._poses, before)

    def test_recovering_retry_goes_to_the_new_place(self):
        nav = self.make()[0]
        out = self.task.navigation_result(nav.command_id, False, 1.0, "navigation_aborted")
        self.assertEqual(self.task.state, "recovering")
        self.memory.move(0.5, 0.0)
        self.assertEqual(self.task.map_moved(1.2), [])
        nxt = self.task.tick(1.2 + self.task.config.retry_pause_s)
        self.assertEqual(nxt[0].kind, "navigate")
        self.assertTrue(np.allclose((nxt[0].pose.x, nxt[0].pose.y), (1.5, 0.0)))
        self.assertEqual([c.kind for c in out], ["recover"])

    def test_no_retarget_when_too_little_time_is_left(self):
        self.make(task_timeout_s=30.0)
        self.memory.move(0.5, 0.0)
        out = self.task.map_moved(9.5)                         # 20.5 s left < 20 + 1 + 1
        self.assertEqual(out, [])
        self.assertEqual(self.task.state, "navigating")
        self.assertEqual(self.task.events[-1]["reason"], "map_moved_replanned")

    def test_search_and_terminal_states_are_left_alone(self):
        self.memory = Memory([])
        self.task = TaskController(self.memory, ring, (PoseGoal(0, 0, 0),))
        self.task.start("杯子", 0.0, Stamp("f0", 0.0))         # unknown object: searching
        self.assertEqual(self.task.map_moved(1.0), [])
        self.task.cancel(1.5)
        self.assertEqual(self.task.map_moved(2.0), [])


class RunnerSourceTests(unittest.TestCase):
    """The runner wiring (ROS node, not importable without rclpy): checked on the source."""

    def setUp(self):
        self.src = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf8")

    def test_planning_map_and_trajectory_feed_query_planner_and_search_goals(self):
        for snippet in ('free_map, map_source = self.planning_map()\n                self.search_planner, planner_info',
                        'observation_poses(candidate, free_map, self.frame.K',
                        'goals, info = select_search_waypoints(waypoints, mapped, fallback, free_map, pose,',
                        'self.live_track.add(frame.timestamp, observed_pose.x, observed_pose.y, observed_pose.yaw)',
                        'elif command.kind == "retarget":',
                        'self.execute(self.task.map_moved(self.now()))'):
            self.assertIn(snippet, self.src)

    def test_every_memory_move_moves_the_followers(self):
        self.assertEqual(self.src.count("self.after_memory_move("), 3)   # live apply, stop apply, frame-tag apply

    def test_live_grid_only_while_mapping_with_live_align(self):
        self.assertIn('if grid is not None and self.mapping_in_progress():', self.src)
        self.assertIn('if self.live_align and getattr(args, "search_waypoints_from_mapping", False):', self.src)


if __name__ == "__main__":
    unittest.main()
