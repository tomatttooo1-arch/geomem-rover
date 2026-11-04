"""10-26 search MVP (owner-approved): 1. not seen at the remembered place -> search (not_seen_at_last_location), the
observation poses used up -> search too; 2. that search starts at the target's remembered position; 3. each mapping
goal also faces outwards; 4. exactly one fresh same-class candidate while searching -> found/search_single_candidate.
Host only, no ROS, no GPU (real core.Memory on the analytic synthetic scenes where geometry matters)."""

from dataclasses import dataclass, replace
import copy
import math
import os
import unittest

import numpy as np

from host.task_control import PoseGoal, TaskConfig, TaskController, chain_waypoints
from host.ros_runtime import FreeMap, outward_twin, select_search_waypoints
from perception.fpga_replay.core import Config, Memory, UNGATED_CREATION, compute_geometry, process_frame
from perception.fpga_replay.synthetic import scene

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@dataclass
class Stamp:
    frame_id: str
    timestamp: float
    pose_valid: bool = True


def memory():
    return Memory("geometry", replace(Config(), **UNGATED_CREATION))


def update(mem, pair):
    frame, detections = pair
    mem.update_memory(process_frame(frame, detections), compute_geometry(frame, mem), frame)
    return frame


def moved_cup_frame(mem, index, x=0.8):
    """The same cup seen with the camera shifted by x: a new track outside the association radius."""
    frame, detections = scene(index)
    frame.T_world_camera[0, 3] = x
    return update(mem, (frame, detections))


VIEWS = [PoseGoal(0, 0, 0), PoseGoal(.2, 0, 0)]
# given order = from the car (far first on purpose); from the target (0, 0): 0.5 -> 3 -> 5
SEARCH = (PoseGoal(5, 0, 0), PoseGoal(.5, 0, 0), PoseGoal(3, 0, 0))


class Base(unittest.TestCase):
    def setUp(self):
        self.mem = memory()
        update(self.mem, scene(0))                  # the cup is remembered at (0, 0)
        self.t = 0.0

    def make(self, search=SEARCH, views=VIEWS, **cfg):
        # 10-30: these tests time the frame count alone (frames 0.1 s apart); the minimum observation time
        # (observe_min_s, car default 6.0) has its own tests (ObserveMinTimeTests)
        cfg.setdefault("observe_min_s", 0.0)
        self.task = TaskController(self.mem, lambda row: list(views), search, config=TaskConfig(**cfg))
        return self.task.start("cup", 0, Stamp("initial", 0))

    def arrive(self, command, t):
        self.assertEqual(self.task.navigation_result(command.command_id, True, t), [])
        self.assertEqual(self.task.state, "observing")

    def frames(self, kind, n, start, **kw):
        out = []
        for i in range(n):
            t = start + 0.1 * i
            frame, det = scene(int(round(t * 10)), kind, **kw)
            frame.frame_id, frame.timestamp, frame.depth_timestamp = "f%.1f" % t, t, t
            det = [replace(d, frame_id=frame.frame_id, timestamp=t) for d in det]
            update(self.mem, (frame, det))
            out.append(self.task.observe(frame, t))
        return out


class NotSeenTests(Base):
    def test_box_in_view_without_detection_n_frames_starts_search(self):
        cmd = self.make()[0]
        self.arrive(cmd, .05)
        # detector miss: the box is on valid depth at the remembered place (surface_or_uncertain), no detection
        outs = self.frames("present", 4, .1, suppress=True)
        self.assertEqual(self.mem.snapshot()[0]["reason"], "surface_or_uncertain")
        self.assertEqual((outs[-1], self.task.state), ([], "observing"))       # 4 < 5
        out = self.frames("present", 1, .5, suppress=True)[0]
        self.assertEqual((self.task.state, self.task.reason), ("navigating", "search"))
        self.assertEqual(self.task.search_trigger, "not_seen_at_last_location")
        self.assertEqual(self.task.events[-2]["reason"], "not_seen_at_last_location")
        self.assertEqual(out[0].pose, PoseGoal(.5, 0, 0))                    # item 2: from the target, not the car

    def test_free_space_counts_and_occlusion_resets(self):
        cmd = self.make(not_seen_frames=3)[0]
        self.arrive(cmd, .05)
        self.frames("removed", 2, .1)                # free (same pose: not independent, no position_invalid)
        self.assertEqual(self.mem.snapshot()[0]["state"], "historical_memory")
        self.frames("out_of_view", 1, .3)            # resets
        self.frames("removed", 2, .4)
        self.assertEqual(self.task.state, "observing")
        self.frames("removed", 1, .6)
        self.assertEqual(self.task.search_trigger, "not_seen_at_last_location")

    def test_no_geometry_frame_neither_counts_nor_resets(self):
        cmd = self.make(not_seen_frames=2)[0]
        self.arrive(cmd, .05)
        self.frames("removed", 1, .1)
        frame, _ = scene(2, "removed")
        frame.frame_id, frame.timestamp, frame.depth_timestamp = "nogeom", .2, .2
        self.mem.update_memory([], {}, frame)        # empty geometry -> reason "unknown"
        self.assertEqual(self.mem.snapshot()[0]["reason"], "unknown")
        self.task.observe(frame, .2)
        self.assertEqual(self.task.state, "observing")
        self.frames("removed", 1, .3)
        self.assertEqual(self.task.search_trigger, "not_seen_at_last_location")

    def test_geometry_removal_keeps_its_own_word(self):
        # 10-29: memory existence_v3 (on the car since 10-27) votes once per 0.3 s window, so the free views are
        # 0.35 s apart; the remembered cup (score +3) is position_invalid at the third free window (-1.5 each),
        # before the not-seen count (5) -- the removal's own word
        cmd = self.make()[0]
        self.arrive(cmd, .05)
        for i, x in [(1, 0), (2, .12), (3, 0)]:
            frame, det = scene(i, "removed", camera_x=x)
            frame.frame_id, frame.timestamp, frame.depth_timestamp = "f%d" % i, .35 * i, .35 * i
            update(self.mem, (frame, det))
            out = self.task.observe(frame, .35 * i)
        self.assertEqual(self.mem.snapshot()[0]["state"], "position_invalid")
        self.assertEqual(self.task.search_trigger, "target_position_invalid")
        self.assertEqual(out[0].pose, PoseGoal(.5, 0, 0))

    def test_remembered_removed_at_start_searches_from_target(self):
        self.mem.tracks["object-0001"].state = "position_invalid"
        out = self.make()
        self.assertEqual(self.task.search_trigger, "target_position_invalid")
        self.assertEqual(out[0].pose, PoseGoal(.5, 0, 0))

    def test_poses_used_up_search_instead_of_unable_to_confirm(self):
        cmd = self.make(observation_timeout_s=.25)[0]
        self.arrive(cmd, .05)
        out = self.task.tick(.4)                     # pose 1 timed out
        self.assertEqual(out[0].pose, VIEWS[1])
        self.arrive(out[0], .45)
        out = self.task.tick(.8)
        self.assertEqual((self.task.state, out[0].pose), ("navigating", PoseGoal(.5, 0, 0)))
        ev = self.task.events[-2]
        self.assertEqual((ev["reason"], ev["detail"]), ("not_seen_at_last_location", "observation_positions_exhausted"))

    def test_observation_poses_before_search_are_capped(self):
        views = [PoseGoal(0.1 * i, 1, 0) for i in range(8)]
        cmd = self.make(views=views, observation_timeout_s=.25, observation_visits_before_search=2)[0]
        self.arrive(cmd, .05)
        out = self.task.tick(.4)
        self.assertEqual(out[0].pose, views[1])
        self.arrive(out[0], .45)
        out = self.task.tick(.8)
        self.assertEqual((self.task.search_trigger, out[0].pose), ("not_seen_at_last_location", PoseGoal(.5, 0, 0)))
        # no search goals: max_visits alone, as before
        cmd = self.make(search=(), views=views, observation_timeout_s=.25, observation_visits_before_search=2)[0]
        self.arrive(cmd, .05)
        cmd = self.task.tick(.4)[0]
        self.assertEqual(cmd.pose, views[1])
        self.arrive(cmd, .45)
        self.assertEqual(self.task.tick(.8)[0].pose, views[2])

    def test_no_search_goals_keeps_unable_to_confirm(self):
        cmd = self.make(search=(), views=VIEWS[:1], observation_timeout_s=.25)[0]
        self.arrive(cmd, .05)
        self.task.tick(.4)
        self.assertEqual((self.task.state, self.task.reason), ("unable_to_confirm", "observation_positions_exhausted"))

    def test_plain_search_keeps_the_given_order(self):
        cmd = self.task_for_missing()
        self.assertEqual(cmd.pose, SEARCH[0])
        self.assertIsNone(self.task.search_trigger)

    def task_for_missing(self):
        self.task = TaskController(self.mem, lambda row: list(VIEWS), SEARCH)
        return self.task.start("book", 0, Stamp("initial", 0))[0]

    def test_config(self):
        for bad in ({"not_seen_frames": 0}, {"max_search_visits": 0}, {"not_seen_frames": True}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                TaskConfig(**bad)
        self.assertEqual((TaskConfig().not_seen_frames, TaskConfig().max_search_visits), (5, 40))


class SingleCandidateTests(Base):
    def test_seen_elsewhere_while_not_seen_at_place_is_taken_at_once(self):
        # 10-31: the moved cup must become a second track for this path -> one_per_class off (on, it takes the
        # remembered track over and is found under its own id: test_one_per_class_moved_cup_keeps_its_id)
        self.mem.cfg = replace(self.mem.cfg, one_per_class=False)
        cmd = self.make(not_seen_frames=1)[0]
        self.arrive(cmd, .05)
        frame = moved_cup_frame(self.mem, 1)          # the cup is now 0.8 m away; the old place reads free/surface
        self.task.observe(frame, 1)
        self.assertEqual((self.task.state, self.task.reason), ("found", "search_single_candidate"))
        self.assertEqual(self.task.target_id, "object-0002")

    def test_one_per_class_moved_cup_keeps_its_id(self):
        """10-31 one_per_class on (default): the cup seen 0.8 m away is the remembered cup (same id, new place)."""
        cmd = self.make(not_seen_frames=1)[0]
        self.arrive(cmd, .05)
        frame = moved_cup_frame(self.mem, 1)
        self.task.observe(frame, 1)
        self.assertEqual(list(self.mem.tracks), ["object-0001"])
        self.assertEqual((self.task.state, self.task.reason, self.task.target_id),
                         ("found", "fresh_matching_observation", "object-0001"))

    def test_passed_over_candidate_is_auto_found(self):
        twin = copy.deepcopy(self.mem.tracks["object-0001"])
        twin.id = "object-0002"
        twin.position[0] += 3
        self.mem.tracks[twin.id] = twin
        self.make(not_seen_frames=1)
        self.assertEqual(self.task.state, "ambiguous")
        cmd = self.task.select_candidate("object-0001", .01)[0]
        self.arrive(cmd, .05)
        out = self.frames("removed", 1, .1)[0]
        self.assertEqual(self.task.search_trigger, "not_seen_at_last_location")
        self.arrive(out[0], .2)
        # only the row the user did not choose is fresh -> owner 10-29: taken (the chosen one re-identified as it)
        t = self.mem.tracks["object-0002"]
        t.state, t.last_seen, t.last_seen_frame = "current_confirmed", .3, "fx"
        self.task.observe(Stamp("fx", .3), .3)
        self.assertEqual((self.task.state, self.task.reason), ("found", "search_single_candidate"))
        self.assertEqual(self.task.target_id, "object-0002")
        ev = [e for e in self.task.events if e.get("reason") == "search_single_candidate"][-1]
        self.assertEqual((ev["previous_target_id"], ev["passed_over"]), ("object-0001", True))


class ObserveMinTimeTests(Base):
    """10-30 owner (car 10-29 23:2x: a weak cup missed, the not-seen verdict came 2.5 s after arrival): at an
    observation pose the not-seen verdict needs >= not_seen_frames AND >= observe_min_s since arrival (car defaults
    5 / 6.0 s); a fresh match in the meantime is found at once."""

    def make6(self):
        cmd = self.make(observe_min_s=6.0)[0]
        self.assertEqual((TaskConfig().observe_min_s, TaskConfig().not_seen_frames), (6.0, 5))
        self.arrive(cmd, .05)

    def nogeom(self, t):
        frame, _ = scene(int(round(t * 10)), "removed")
        frame.frame_id, frame.timestamp, frame.depth_timestamp = "ng%.1f" % t, t, t
        self.mem.update_memory([], {}, frame)        # empty geometry -> reason "unknown": neither counts nor resets
        return self.task.observe(frame, t)

    def test_five_frames_before_six_seconds_do_not_search(self):
        self.make6()
        self.frames("present", 5, .1, suppress=True)                 # 5 not-seen frames by 0.5 s
        self.assertEqual(self.mem.snapshot()[0]["reason"], "surface_or_uncertain")
        self.assertEqual((self.task.state, self.task.search_trigger), ("observing", None))
        self.frames("present", 50, .6, suppress=True)                # ... up to 5.5 s: still observing
        self.assertEqual((self.task.state, self.task.search_trigger), ("observing", None))

    def test_six_seconds_with_fewer_than_five_frames_do_not_search(self):
        self.make6()
        self.frames("present", 4, .1, suppress=True)                 # 4 valid frames
        for i in range(55):                                           # geometry unavailable 0.5 .. 5.9 s
            self.nogeom(.5 + .1 * i)
        self.assertEqual((self.task.state, self.task.search_trigger), ("observing", None))   # old code: left at 3 s
        out = self.nogeom(6.1)                                        # the observation-pose timeout: next pose
        self.assertEqual((self.task.state, self.task.reason, self.task.search_trigger),
                         ("navigating", "reobserve", None))
        self.assertEqual(out[-1].pose, VIEWS[1])

    def test_both_met_search_on_a_frame(self):
        self.make6()
        self.frames("present", 59, .1, suppress=True)                # 0.1 .. 5.9 s
        self.assertEqual(self.task.state, "observing")
        out = self.frames("present", 1, 6.1, suppress=True)[0]       # 6.05 s since arrival, >= 5 frames
        self.assertEqual((self.task.state, self.task.reason), ("navigating", "search"))
        self.assertEqual(self.task.search_trigger, "not_seen_at_last_location")
        self.assertEqual(out[0].pose, PoseGoal(.5, 0, 0))

    def test_both_met_search_on_the_watchdog(self):
        self.make6()
        self.frames("present", 59, .1, suppress=True)                # last frame 5.9 s (recent)
        out = self.task.tick(6.06)
        self.assertEqual((self.task.state, self.task.reason), ("navigating", "search"))
        self.assertEqual(self.task.search_trigger, "not_seen_at_last_location")
        self.assertEqual(out[0].pose, PoseGoal(.5, 0, 0))

    def test_fresh_match_inside_the_window_is_found_at_once(self):
        self.make6()
        self.frames("present", 29, .1, suppress=True)                # 29 not-seen frames, 0.1 .. 2.9 s
        self.assertEqual(self.task.state, "observing")
        out = self.frames("present", 1, 3.0)[0]                      # detected at 3.0 s
        self.assertEqual((self.task.state, self.task.reason), ("found", "fresh_matching_observation"))
        self.assertEqual(self.task.search_trigger, None)
        self.assertIn("stop", [c.kind for c in out])


class OutwardTwinTests(unittest.TestCase):
    def grid(self):
        n = 200
        return FreeMap(np.full((n, n), 0, dtype=np.int8), 0.05, -5.0, -5.0)

    def test_twins_follow_their_goal_arrival_heading_first(self):
        mapped = (PoseGoal(2.0, 0.0, 0.0), PoseGoal(1.0, 0.0, math.pi))
        goals, info = select_search_waypoints((), mapped, 'spin', self.grid(), (0.0, 0.0, 0.0), outward=True)
        self.assertEqual((info["source"], info["outward_twins"]), ("mapping_trajectory", 2))
        self.assertEqual([(p.x, p.y) for p in goals], [(1.0, 0.0)] * 2 + [(2.0, 0.0)] * 2)
        # arriving at each place driving +x: the yaw-0 goal (the twin at (1, 0), the original at (2, 0)) first
        self.assertAlmostEqual(goals[0].yaw, 0.0)
        self.assertAlmostEqual(abs(goals[1].yaw), math.pi)
        self.assertAlmostEqual(goals[2].yaw, 0.0)
        self.assertAlmostEqual(abs(goals[3].yaw), math.pi)
        # patrol / old callers: no twins, order unchanged
        goals, info = select_search_waypoints((), mapped, 'spin', self.grid(), (0.0, 0.0, 0.0))
        self.assertEqual(len(goals), 2)
        self.assertNotIn("outward_twins", info)

    def test_twin_is_the_opposite_heading(self):
        p = PoseGoal(1.0, 2.0, 0.3)
        q = outward_twin(p)
        self.assertEqual((q.x, q.y), (1.0, 2.0))
        self.assertAlmostEqual(math.cos(q.yaw - p.yaw), -1.0)

    def test_chain_keeps_twins_together_from_any_start(self):
        poses = [PoseGoal(3, 0, 0), PoseGoal(3, 0, math.pi), PoseGoal(1, 0, 0), PoseGoal(1, 0, math.pi)]
        out = chain_waypoints(poses, (4, 0), from_xy=(4, 1))
        self.assertEqual([(p.x, p.y) for p in out], [(3, 0)] * 2 + [(1, 0)] * 2)
        self.assertAlmostEqual(abs(out[2].yaw), math.pi)            # driving -x to (1, 0): face pi first

    def test_six_goals_twice_fit_the_search_visit_cap(self):
        mem = memory()
        search = [PoseGoal(float(i), 0.0, 0.0) for i in range(1, 13)]
        task = TaskController(mem, lambda row: [], search,
                              TaskConfig(observation_timeout_s=.1, task_timeout_s=1e4, min_attempt_time_s=0))
        cmd = task.start("book", 0, Stamp("f0", 0))[0]
        t, poses = 0.0, [cmd.pose]
        while True:
            t += .05
            task.navigation_result(cmd.command_id, True, t)
            t += .2
            out = task.observe(Stamp("f%.2f" % t, t), t)
            if task.state != "navigating":
                break
            cmd = out[0]
            poses.append(cmd.pose)
        self.assertEqual(len(poses), 12)
        self.assertEqual((task.state, task.reason), ("not_found", "search_exhausted"))

    def test_runtime_uses_twins_for_the_find_search_only(self):
        src = open(os.path.join(ROOT, "host", "ros_runtime.py"), encoding="utf8").read()
        self.assertIn('outward=purpose == "search"', src)
        self.assertIn('extra["search_trigger"]', src)


class ScenarioTests(Base):
    """Moved cup, host only: arrive at the remembered place, it is not there (detector sees nothing, place reads
    free), search starts nearest the remembered place, the cup is seen at a search goal -> found under its new id."""

    def test_moved_object_found_by_search(self):
        # 10-31: the moved cup must become a second track for this path -> one_per_class off (on, it takes the
        # remembered track over and is found under its own id: test_one_per_class_moved_cup_keeps_its_id)
        self.mem.cfg = replace(self.mem.cfg, one_per_class=False)
        search = (PoseGoal(4, 0, 0), PoseGoal(1.0, 0, 0), PoseGoal(2.5, 0, 0))
        cmd = self.make(search=search)[0]
        self.arrive(cmd, .05)
        out = self.frames("removed", 5, .1)[-1]
        self.assertEqual(self.task.search_trigger, "not_seen_at_last_location")
        self.assertEqual(out[0].pose, PoseGoal(1.0, 0, 0))
        self.arrive(out[0], 1.0)
        frame = moved_cup_frame(self.mem, 2)
        out = self.task.observe(frame, 2)
        self.assertEqual((self.task.state, self.task.reason, self.task.target_id),
                         ("found", "search_single_candidate", "object-0002"))
        self.assertEqual([c.kind for c in out], ["stop", "approach"])


class BridgeWordsTests(unittest.TestCase):
    def test_bridge_shows_the_trigger_and_screens_know_the_words(self):
        from ui.geomem.bridge import gmb_proto
        g = {"state": "navigating", "reason_task": "search", "search_trigger": "not_seen_at_last_location"}
        self.assertEqual(gmb_proto.compose_task(g, "t1")["message"], gmb_proto.REASON_ZH["not_seen_at_last_location"])
        g["search_trigger"] = "target_position_invalid"
        self.assertEqual(gmb_proto.compose_task(g, "t1")["message"], gmb_proto.REASON_ZH["target_position_invalid"])
        del g["search_trigger"]
        self.assertEqual(gmb_proto.compose_task(g, "t1")["message"], gmb_proto.REASON_ZH["search"])
        for js in (("app", "src", "model.js"), ("operator", "src", "v2", "model.js")):
            text = open(os.path.join(ROOT, "ui", "geomem", *js), encoding="utf-8").read()
            for w in ("not_seen_at_last_location", "target_position_invalid", "search_single_candidate"):
                self.assertIn(w + ":'", text)


class _NoPosePlanner(object):
    """Stands in for host/search_belief.BeliefSearch: records begin(xy), hands out given goals."""
    def __init__(self, goals):
        self.goals, self.begun = list(goals), []

    def begin(self, xy):
        self.begun.append(xy)

    def next_goal(self):
        return (self.goals.pop(0), {"n": 1}) if self.goals else (None, {"stop": "no_candidate"})

    def mark_failed(self, pose):
        pass


class NoObservationPoseTests(Base):
    """10-26 (owner): zero observation poses around a remembered target (e.g. against a wall) -> search from the
    remembered place (no_observation_pose), not unable_to_confirm; without any search the old end stays."""

    def test_zero_poses_with_waypoints_searches_from_the_remembered_place(self):
        out = self.make(views=[])
        self.assertEqual((self.task.state, self.task.reason), ("navigating", "search"))
        self.assertEqual(self.task.search_trigger, "no_observation_pose")
        ev = self.task.events[-2]
        self.assertEqual((ev["reason"], ev["detail"]), ("no_observation_pose", "no_observation_pose_planned"))
        self.assertEqual(self.task.target_id, "object-0001")
        self.assertEqual([c.kind for c in out], ["navigate"])
        self.assertEqual(out[0].pose, PoseGoal(.5, 0, 0))          # nearest to the cup at (0, 0), not the given order
        self.assertEqual(self.task.attempts[0]["kind"], "search")

    def test_zero_poses_without_search_keeps_the_old_end(self):
        out = self.make(search=(), views=[])
        self.assertEqual((self.task.state, self.task.reason), ("unable_to_confirm", "no_observation_pose_planned"))
        self.assertEqual([c.kind for c in out], ["stop"])
        self.assertIsNone(self.task.search_trigger)

    def test_zero_poses_with_planner_uses_the_planner(self):
        planner = _NoPosePlanner([PoseGoal(1, 1, 0)])
        self.task = TaskController(self.mem, lambda row: [], SEARCH, search_planner=planner)
        out = self.task.start("cup", 0, Stamp("initial", 0))
        self.assertEqual(self.task.search_trigger, "no_observation_pose")
        self.assertEqual(len(planner.begun), 1)
        x, y = planner.begun[0]
        self.assertLess(math.hypot(x, y), .05)                     # prior centred on the remembered place
        self.assertEqual(out[0].pose, PoseGoal(1, 1, 0))            # the planner's goal, not the waypoint list
        self.assertEqual(self.task.events[-2]["reason"], "search_goal_selected")

    def test_manual_approach_unchanged(self):
        self.task = TaskController(self.mem, lambda row: [], SEARCH, navigation_available=lambda: False)
        out = self.task.start("cup", 0, Stamp("initial", 0))
        self.assertEqual((self.task.state, self.task.reason), ("needs_manual_approach", "navigation_server_unavailable"))
        self.assertEqual([c.kind for c in out], ["stop"])
        self.assertIsNone(self.task.search_trigger)

    def test_screens_know_the_word(self):
        from ui.geomem.bridge import gmb_proto
        g = {"state": "navigating", "reason_task": "search", "search_trigger": "no_observation_pose"}
        self.assertEqual(gmb_proto.compose_task(g, "t1")["message"], gmb_proto.REASON_ZH["no_observation_pose"])
        for js in (("app", "src", "model.js"), ("operator", "src", "v2", "model.js")):
            text = open(os.path.join(ROOT, "ui", "geomem", *js), encoding="utf-8").read()
            self.assertIn("no_observation_pose:'", text)


if __name__ == "__main__":
    unittest.main()
