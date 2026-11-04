"""10-26 find-near (owner): A. a fresh confirmation of the target while driving, within 1.2 m, ends the task found
(seen_en_route) with the usual stop; B. after found one final-approach goal (host/ros_runtime.approach_goal), whose
outcome never changes the found verdict. Host only, no ROS."""

from dataclasses import dataclass
from pathlib import Path
import math
import unittest

import numpy as np

from host.task_control import PoseGoal, TaskConfig, TaskController
from host import ros_runtime
from host.ros_runtime import approach_goal, box_in_view, frame_base_xy

ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Stamp:
    frame_id: str
    timestamp: float
    pose_valid: bool = True


TARGET = {"id": "object-0001", "label": "cup", "color": None, "state": "historical_memory",
          "position_m": [2.0, 0.0, 0.05], "bounds_m": [[1.96, -0.04, 0.0], [2.04, 0.04, 0.10]],
          "last_seen": -1.0, "last_seen_frame": "old"}
OTHER = dict(TARGET, id="object-0002", position_m=[2.0, 0.5, 0.05])


class Memory(object):
    """query rows; ``confirm(ids, frame)`` makes those rows freshly confirmed in that frame."""

    def __init__(self, rows):
        self.rows = [dict(r) for r in rows]

    def confirm(self, ids, frame):
        for r in self.rows:
            if r["id"] in ids:
                r.update(state="current_confirmed", last_seen=frame.timestamp, last_seen_frame=frame.frame_id)

    def query_memory(self, goal, after=None):
        out = []
        for r in self.rows:
            item = dict(r)
            item["confirmed_after_query"] = (after is not None and r["last_seen"] > after
                                             and r["state"] == "current_confirmed")
            out.append(item)
        return out


POSES = [PoseGoal(1.1, 0.0, 0.0), PoseGoal(2.0, -0.9, math.pi / 2)]


class EnRouteTests(unittest.TestCase):
    def make(self, rows=(TARGET,), search=(), **cfg):
        self.memory = Memory(rows)
        self.task = TaskController(self.memory, lambda row: list(POSES), search, config=TaskConfig(**cfg))
        return self.task.start("杯子", 0.0, Stamp("f0", 0.0))

    def seen(self, ids, name, t, robot_xy):
        frame = Stamp(name, t)
        self.memory.confirm(ids, frame)
        return self.task.observe(frame, t, robot_xy=robot_xy)

    def test_near_sighting_while_navigating_is_found_with_stop(self):
        nav = self.make()[0]
        self.assertEqual((nav.kind, self.task.state), ("navigate", "navigating"))
        out = self.seen({"object-0001"}, "f1", 0.5, (1.0, 0.3))        # 1.04 m from the target
        self.assertEqual((self.task.state, self.task.reason), ("found", "seen_en_route"))
        self.assertEqual([c.kind for c in out], ["stop", "approach"])
        self.assertEqual(out[0].command_id, nav.command_id)              # the live goal is the one cancelled
        self.assertEqual(self.task.en_route["distance_m"], round(math.hypot(1.0, 0.3), 3))
        self.assertEqual(self.task.events[-1]["distance_m"], self.task.en_route["distance_m"])
        self.assertEqual(self.task.attempts[-1]["result"], "interrupted")
        self.assertEqual(self.task.found_row["id"], "object-0001")

    def test_far_sighting_does_not_decide(self):
        # 10-30: the target's own cap is 1.6 m (was 1.2 m for everything)
        self.make()
        out = self.seen({"object-0001"}, "f1", 0.5, (0.3, 0.0))         # 1.7 m > 1.6
        self.assertEqual((out, self.task.state), ([], "navigating"))

    def test_threshold_is_a_config_field(self):
        self.make(en_route_target_max_distance_m=1.2)
        out = self.seen({"object-0001"}, "f1", 0.5, (0.7, 0.0))         # 1.3 m > 1.2
        self.assertEqual((out, self.task.state), ([], "navigating"))
        self.make(en_route_target_max_distance_m=1.8)
        self.seen({"object-0001"}, "f1", 0.5, (0.3, 0.0))               # 1.7 m <= 1.8
        self.assertEqual(self.task.state, "found")

    def test_target_seen_at_1_4_m_while_navigating_is_found(self):
        """10-30 owner (car 10-29 23:2x: cup1 seen at 1.32 m while turning, dropped by the 1.2 m cap)."""
        self.assertEqual((TaskConfig().en_route_target_max_distance_m, TaskConfig().en_route_max_distance_m),
                         (1.6, 1.2))
        nav = self.make()[0]
        out = self.seen({"object-0001"}, "f1", 0.5, (0.6, 0.0))         # 1.4 m
        self.assertEqual((self.task.state, self.task.reason), ("found", "seen_en_route"))
        self.assertEqual([c.kind for c in out], ["stop", "approach"])
        self.assertEqual(out[0].command_id, nav.command_id)
        self.assertEqual(self.task.en_route["distance_m"], 1.4)

    def test_target_seen_at_1_7_m_while_navigating_is_not_taken(self):
        self.make()
        out = self.seen({"object-0001"}, "f1", 0.5, (0.3, 0.0))         # 1.7 m
        self.assertEqual((out, self.task.state), ([], "navigating"))

    def test_search_candidate_at_1_4_m_keeps_the_1_2_m_cap(self):
        """10-30 owner: a same-class search candidate (not the target itself) still needs <= 1.2 m."""
        self.make(rows=(), search=(PoseGoal(0.0, 0.0, 0.0), PoseGoal(1.0, 0.0, 0.0)))
        self.memory.rows = [dict(TARGET)]
        out = self.seen({"object-0001"}, "f1", 0.5, (0.6, 0.0))         # 1.4 m
        self.assertEqual((out, self.task.state), ([], "navigating"))
        out = self.seen({"object-0001"}, "f2", 0.6, (0.85, 0.0))        # 1.15 m
        self.assertEqual((self.task.state, self.task.reason), ("found", "search_single_candidate"))

    def test_no_position_no_verdict(self):
        self.make()
        out = self.seen({"object-0001"}, "f1", 0.5, None)
        self.assertEqual((out, self.task.state), ([], "navigating"))

    def test_other_object_does_not_decide(self):
        self.make(rows=(TARGET, OTHER))
        # two rows -> ambiguous at start; select the first explicitly
        self.assertEqual(self.task.state, "ambiguous")
        self.task.select_candidate("object-0001", 0.1)
        out = self.seen({"object-0002"}, "f1", 0.5, (2.0, 0.0))
        self.assertEqual((out, self.task.state), ([], "navigating"))

    def test_stale_frame_row_does_not_decide(self):
        self.make()
        self.memory.confirm({"object-0001"}, Stamp("f1", 0.5))
        out = self.task.observe(Stamp("f2", 0.6), 0.6, robot_xy=(1.5, 0.0))   # row confirmed in f1, not f2
        self.assertEqual((out, self.task.state), ([], "navigating"))

    def test_search_mode_single_near_candidate_is_found(self):
        # 10-26 search MVP (owner): one near same-class candidate while searching -> found, its id the target
        self.make(rows=(), search=(PoseGoal(0.0, 0.0, 0.0), PoseGoal(1.0, 0.0, 0.0)))
        self.memory.rows = [dict(TARGET)]
        out = self.seen({"object-0001"}, "f1", 0.5, (1.5, 0.0))
        self.assertEqual((self.task.state, self.task.reason), ("found", "search_single_candidate"))
        self.assertEqual((self.task.target_id, self.task.found_row["id"]), ("object-0001", "object-0001"))
        self.assertEqual([c.kind for c in out], ["stop", "approach"])
        self.assertEqual(self.task.en_route["distance_m"], 0.5)
        self.assertEqual(self.task.attempts[-1]["result"], "interrupted")

    def test_search_mode_two_near_candidates_go_ambiguous_not_found(self):
        self.make(rows=(), search=(PoseGoal(0.0, 0.0, 0.0), PoseGoal(1.0, 0.0, 0.0)))
        self.assertTrue(self.task._searching)
        self.memory.rows = [dict(TARGET), dict(OTHER)]
        out = self.seen({"object-0001", "object-0002"}, "f1", 0.5, (1.5, 0.0))
        self.assertEqual(self.task.state, "ambiguous")
        self.assertEqual(self.task.reason, "search_candidate_selection_required")
        self.assertEqual([c.kind for c in out], ["stop"])
        self.assertEqual([r["id"] for r in self.task.candidates], ["object-0001", "object-0002"])
        self.assertEqual(self.task.attempts[-1]["result"], "interrupted")
        # the explicit selection then navigates as before
        cmd = self.task.select_candidate("object-0001", 0.6)
        self.assertEqual(cmd[0].kind, "navigate")

    def test_search_mode_far_candidate_ignored(self):
        self.make(rows=(), search=(PoseGoal(0.0, 0.0, 0.0),))
        self.memory.rows = [dict(TARGET)]
        out = self.seen({"object-0001"}, "f1", 0.5, (0.0, 0.0))          # 2 m away
        self.assertEqual((out, self.task.state), ([], "navigating"))

    def test_recovering_counts_as_en_route(self):
        nav = self.make()[0]
        out = self.task.navigation_result(nav.command_id, False, 0.3, "navigation_goal_rejected")
        self.assertEqual(self.task.state, "recovering")
        out = self.seen({"object-0001"}, "f1", 0.5, (1.5, 0.0))
        self.assertEqual((self.task.state, self.task.reason), ("found", "seen_en_route"))
        self.assertEqual([c.kind for c in out], ["stop", "approach"])

    def test_arrival_path_unchanged(self):
        nav = self.make()[0]
        self.task.navigation_result(nav.command_id, True, 0.5)
        self.assertEqual(self.task.state, "observing")
        out = self.seen({"object-0001"}, "f1", 0.6, (1.1, 0.0))
        self.assertEqual((self.task.state, self.task.reason), ("found", "fresh_matching_observation"))
        self.assertEqual([c.kind for c in out], ["stop", "approach"])
        self.assertIsNone(self.task.en_route)

    def test_final_approach_off_keeps_old_command_list(self):
        nav = self.make(final_approach=False)[0]
        self.task.navigation_result(nav.command_id, True, 0.5)
        out = self.seen({"object-0001"}, "f1", 0.6, None)
        self.assertEqual([c.kind for c in out], ["stop"])

    def test_approach_outcome_never_changes_found(self):
        nav = self.make()[0]
        self.task.navigation_result(nav.command_id, True, 0.5)
        self.seen({"object-0001"}, "f1", 0.6, None)
        self.task.set_approach({"result": "failed", "reason": "navigation_goal_rejected"})
        self.assertEqual((self.task.state, self.task.reason), ("found", "fresh_matching_observation"))
        self.assertEqual(self.task.tick(5.0), [])
        self.assertEqual(self.task.approach["result"], "failed")

    def test_bad_config_refused(self):
        for kw in ({"en_route_target_max_distance_m": -1.0}, {"en_route_target_max_distance_m": float("nan")},
                   {"observe_min_s": -1.0}, {"observe_min_s": float("inf")},
                   {"en_route_max_distance_m": -1.0}, {"en_route_max_distance_m": float("nan")},
                   {"final_approach": 1}):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                TaskConfig(**kw)


def base_camera(x=0.10, height=0.3915, pitch_deg=7.05):
    """camera optical frame in base_link: x forward, looking forward and pitched down (car mount)."""
    p = math.radians(pitch_deg)
    # optical z = forward pitched down, optical x = -base y, optical y = down-ish
    z = np.array([math.cos(p), 0.0, -math.sin(p)])
    x_axis = np.array([0.0, -1.0, 0.0])
    y_axis = np.cross(z, x_axis)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x_axis, y_axis, z])
    T[:3, 3] = [x, 0.0, height]
    return T


K = np.array([[517.17, 0, 324.22], [0, 517.12, 240.65], [0, 0, 1]])
SIZE = (640, 480)


def world_base(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])


class ApproachGoalTests(unittest.TestCase):
    def test_gap_is_the_footprint_floor_not_20cm(self):
        pose, info = approach_goal(world_base(1.1, 0.0, 0.0), TARGET, base_camera(), K, SIZE)
        # front 0.107 ahead of base; floor 0.30 + 0.05 - 0.107 = 0.243 > 0.20
        self.assertEqual(info["base_to_front_m"], 0.107)
        self.assertEqual(info["gap_floor_m"], 0.243)
        self.assertEqual(info["gap_m"], 0.243)
        self.assertFalse(info["laser_rule"])                          # a 10 cm cup is below the view there
        self.assertAlmostEqual(info["surface_m"], 0.86, places=6)     # box face x 1.96
        self.assertAlmostEqual(pose.x, 1.96 - 0.243 - 0.107, places=6)
        self.assertAlmostEqual((pose.y, pose.yaw), (0.0, 0.0), places=9)
        self.assertAlmostEqual(info["forward_m"], 0.86 - 0.35, places=3)

    def test_target_gap_above_floor_is_kept(self):
        _, info = approach_goal(world_base(1.1, 0.0, 0.0), TARGET, base_camera(), K, SIZE, target_gap_m=0.3)
        self.assertEqual(info["gap_m"], 0.3)

    def test_tall_object_in_view_keeps_laser_distance(self):
        tall = dict(TARGET, bounds_m=[[1.96, -0.04, 0.0], [2.04, 0.04, 0.60]])
        pose, info = approach_goal(world_base(1.1, 0.0, 0.0), tall, base_camera(), K, SIZE)
        self.assertTrue(info["laser_rule"])
        self.assertAlmostEqual(info["gap_m"], 0.42 - 0.007, places=6)
        self.assertAlmostEqual(pose.x, 1.96 - 0.413 - 0.107, places=6)
        # without camera inputs the rule cannot be checked and is not applied
        _, info2 = approach_goal(world_base(1.1, 0.0, 0.0), tall, base_camera())
        self.assertFalse(info2["laser_rule"])

    def test_heads_for_the_object_from_an_angle(self):
        pose, info = approach_goal(world_base(2.0, -0.9, 0.3), TARGET, base_camera(), K, SIZE)
        self.assertAlmostEqual(pose.yaw, math.pi / 2, places=9)       # turned to face it
        self.assertAlmostEqual(pose.x, 2.0, places=9)
        self.assertAlmostEqual(pose.y, -0.04 - 0.35, places=6)

    def test_no_goal_when_already_close_or_inside(self):
        pose, info = approach_goal(world_base(1.62, 0.0, 0.0), TARGET, base_camera(), K, SIZE)
        self.assertIsNone(pose)
        self.assertEqual(info["reason"], "already_within_gap")
        pose, info = approach_goal(world_base(2.0, 0.01, 0.0), TARGET, base_camera())
        self.assertIsNone(pose)
        self.assertIn(info["reason"], ("inside_object_box", "at_object_centre"))
        pose, info = approach_goal(world_base(-1.0, 0.0, 0.0), TARGET, base_camera())
        self.assertEqual(info["reason"], "too_far_for_approach")

    def test_box_in_view_known_answers(self):
        cam = base_camera()
        self.assertTrue(box_in_view(TARGET, PoseGoal(1.1, 0.0, 0.0), K, SIZE, cam))     # 0.76 m from the lens
        self.assertFalse(box_in_view(TARGET, PoseGoal(1.61, 0.0, 0.0), K, SIZE, cam))   # 0.25 m: below the view
        self.assertFalse(box_in_view(TARGET, PoseGoal(1.1, 0.0, math.pi), K, SIZE, cam))  # behind

    def test_frame_base_xy(self):
        cam = base_camera()
        frame = type("F", (), {"pose_valid": True, "T_world_camera": world_base(1.0, 2.0, 0.5) @ cam})()
        self.assertEqual(tuple(round(v, 9) for v in frame_base_xy(frame, cam)), (1.0, 2.0))
        frame.pose_valid = False
        self.assertIsNone(frame_base_xy(frame, cam))
        self.assertIsNone(frame_base_xy(type("F", (), {"pose_valid": True})(), cam))


class E2eToolTests(unittest.TestCase):
    def test_approach_running(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location("e2e_ff", str(ROOT / "tools" / "car_run" / "e2e_find_flow.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        self.assertFalse(mod.approach_running([]))
        self.assertTrue(mod.approach_running([{"event": "approach_started"}]))
        self.assertFalse(mod.approach_running([{"event": "approach_started"}, {"event": "approach_result"}]))
        self.assertTrue(mod.approach_running([{"event": "approach_result"}, {"event": "approach_started"}]))


class RuntimeWiringTests(unittest.TestCase):
    """The node needs ROS; its wiring is pinned in the source."""

    def test_wiring(self):
        src = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        self.assertIn("self.task.observe(frame, self.now(), robot_xy=frame_base_xy(frame, self.base_camera))", src)
        self.assertIn('elif command.kind == "approach":\n                    self.request_approach(command)', src)
        self.assertIn("self.navigate(pose, command.command_id, approach=True)", src)
        self.assertIn('extra["approach"] = self.task.approach', src)
        self.assertIn('extra["en_route"] = self.task.en_route', src)
        self.assertEqual(ros_runtime.APPROACH_GAP_TARGET_M, 0.20)


if __name__ == "__main__":
    unittest.main()
