"""Task tests add navigation/freshness coverage beyond memory-only replay tests."""

import copy
from dataclasses import dataclass, replace
import unittest

from host.task_control import PoseGoal, TaskConfig, TaskController
from perception.fpga_replay.core import Config as _Config, Memory as _Memory, UNGATED_CREATION, compute_geometry, process_frame
from perception.fpga_replay.synthetic import scene


def Memory(strategy="geometry", cfg=None):
    """core.Memory with the D-044 creation gate off (core.UNGATED_CREATION). The expectations in this file
    were written for single-sighting creation; the gate has its own tests (test_memory_creation_gate.py)."""
    return _Memory(strategy, replace(_Config() if cfg is None else cfg, **UNGATED_CREATION))


@dataclass
class FrameStamp:
    frame_id: str
    timestamp: float
    pose_valid: bool = True


# 10-27: these tests encode the legacy memory rules (two independent free views; a replacement in search gets a
# new id), kept behind Config.existence_v3=False; the v3 counterparts are in tests/test_memory_existence_20261027.py
LEGACY_MEMORY_TESTS = ("test_invalid_position_starts_fixed_search_after_independent_free_views",
                       "test_single_new_identity_in_search_is_found_under_its_own_id",
                       "test_two_new_identities_in_search_still_require_selection")


class TaskControllerTests(unittest.TestCase):
    def setUp(self):
        self.memory = Memory(cfg=_Config(existence_v3=False) if self._testMethodName in LEGACY_MEMORY_TESTS else None)
        self.update(scene(0))
        self.views = [PoseGoal(0, 0, 0), PoseGoal(.2, 0, 0)]
        self.search = [PoseGoal(1, 0, 0), PoseGoal(2, 0, 0)]
        self.task = TaskController(self.memory, lambda row: self.views, self.search)

    def update(self, pair):
        frame, detections = pair
        self.memory.update_memory(process_frame(frame, detections),
                                  compute_geometry(frame, self.memory), frame)
        return frame

    def other_object_frame(self):
        # Translate the whole local scene into another world location. Its cup
        # has the same visual attributes but is outside the association radius.
        frame, detections = scene(1)
        frame.T_world_camera[0, 3] = .8
        return self.update((frame, detections))

    def start(self):
        return self.task.start("找红色杯子", 0, FrameStamp("initial", 0))[0]

    def arrive(self):
        command = self.start()
        self.assertEqual(self.task.navigation_result(command.command_id, True, .5), [])
        return command

    def test_arrival_and_old_memory_do_not_confirm_but_new_same_target_does(self):
        self.arrive()
        self.assertEqual(self.task.state, "observing")
        self.task.observe(FrameStamp("late", .4), .6)
        self.assertEqual(self.task.state, "observing")
        frame = self.update(scene(1))
        self.task.observe(frame, frame.timestamp)
        self.assertEqual(self.task.state, "found")
        self.assertEqual(self.task.reason, "fresh_matching_observation")

    def test_frame_at_arrival_is_not_after_arrival(self):
        command = self.start()
        frame = self.update(scene(1))
        self.task.navigation_result(command.command_id, True, 1)
        self.task.observe(frame, 1)
        self.assertEqual(self.task.state, "observing")

    def test_wrong_target_cannot_confirm(self):
        self.arrive()
        frame = self.other_object_frame()
        self.task.observe(frame, 1)
        self.assertNotEqual(self.task.state, "found")

    def test_wrong_frame_identity_cannot_confirm(self):
        self.arrive()
        self.update(scene(1))
        self.task.observe(FrameStamp("unrelated-frame", 1), 1)
        self.assertEqual(self.task.state, "observing")

    def test_duplicate_frame_cannot_refresh_watchdog(self):
        self.start()
        self.task.observe(FrameStamp("initial", 1), 1)
        commands = self.task.tick(2)
        self.assertEqual(self.task.reason, "stream_lost")
        self.assertEqual(commands[0].kind, "stop")

    def test_old_delivered_frames_cannot_refresh_watchdog(self):
        self.start()
        self.task.observe(FrameStamp("old", .1), 1.5)
        self.task.tick(2)
        self.assertEqual(self.task.reason, "stream_lost")

    def test_fresh_frame_cannot_revive_an_expired_stream(self):
        self.start()
        commands = self.task.observe(FrameStamp("too-late", 2.1), 2.1)
        self.assertEqual(commands[0].kind, "stop")
        self.assertEqual(self.task.reason, "stream_lost")

    def test_expired_observation_waits_for_recent_frame_before_next_goal(self):
        self.task = TaskController(
            self.memory,
            lambda row: self.views,
            config=TaskConfig(observation_timeout_s=.25, observe_min_s=0, stream_timeout_s=2,
                              max_frame_age_s=.5),
        )
        self.arrive()
        self.assertEqual(self.task.tick(.8), [])
        self.assertEqual(self.task.state, "observing")
        commands = self.task.observe(FrameStamp("fresh", .81), .81)
        self.assertEqual(commands[0].kind, "navigate")
        self.assertEqual(commands[0].pose, self.views[1])

    def test_waiting_for_recent_frame_remains_watchdog_bounded(self):
        cases = (
            (TaskConfig(observation_timeout_s=.25, observe_min_s=0, stream_timeout_s=1,
                        max_frame_age_s=.5), 1, "stream_lost"),
            (TaskConfig(observation_timeout_s=.25, observe_min_s=0, stream_timeout_s=2,
                        task_timeout_s=.9, max_frame_age_s=.5), .9, "task_timeout"),
        )
        for config, deadline, reason in cases:
            with self.subTest(reason=reason):
                self.task = TaskController(self.memory, lambda row: self.views, config=config)
                self.arrive()
                self.assertEqual(self.task.tick(.8), [])
                commands = self.task.tick(deadline)
                self.assertEqual(commands[0].kind, "stop")
                self.assertEqual(self.task.reason, reason)

    def test_observe_cannot_revive_expired_task_or_navigation(self):
        cases = (
            (TaskConfig(task_timeout_s=.25), .3, "task_timeout", "stop", "failed"),
            # 10-25: a navigation timeout starts a retry -- still the goal is retired, never revived by the frame
            (TaskConfig(navigation_timeout_s=.25), .3, "navigation_timeout", "recover", "recovering"),
        )
        for config, now, reason, kind, state in cases:
            with self.subTest(reason=reason):
                self.task = TaskController(self.memory, lambda row: self.views, config=config)
                command = self.start()
                commands = self.task.observe(FrameStamp("fresh-" + reason, now), now)
                self.assertEqual((commands[0].kind, self.task.state), (kind, state))
                self.assertEqual(self.task.attempts[0]["reason"], reason)
                self.task.navigation_result(command.command_id, True, now)
                self.assertEqual(self.task.state, state)

    def test_navigation_budget_default_30_car_60(self):
        """10-22 find-nav: a goal still navigating 45 s in times out under the default 30 s budget and carries on
        under the car's 60 s (host/ros_runtime.py --navigation-timeout-s, tools/car_run board), which then ends it
        at 60 s. 10-25: the timed-out goal is retried (recover, then the same goal), not the end of the task."""
        car = TaskConfig(navigation_timeout_s=60.0, task_timeout_s=180.0)
        for config, timed_out in ((TaskConfig(), True), (car, False)):
            with self.subTest(config=config):
                self.task = TaskController(self.memory, lambda row: self.views, config=config)
                self.start()
                commands = []
                for t in range(1, 46):                 # a frame a second: the 2 s stream watchdog stays quiet
                    commands += self.task.observe(FrameStamp("nav-%s-%d" % (timed_out, t), t), t)
                first = self.task.attempts[0]
                self.assertEqual((first["result"], first["reason"]) if timed_out else (first["result"], self.task.state),
                                 ("failed", "navigation_timeout") if timed_out else (None, "navigating"))
                if timed_out:                          # 30: recover; 31: the same goal again
                    self.assertEqual([c.kind for c in commands], ["recover", "navigate"])
                    self.assertEqual((first["duration_s"], self.task.attempts[1]["started"]), (30, 31))
        commands = []
        for t in range(46, 62):
            commands += self.task.observe(FrameStamp("nav-car-%d" % t, t), t)
        self.assertEqual((commands[0].kind, commands[0].reason), ("recover", "navigation_timeout"))
        self.assertEqual(self.task.attempts[0]["duration_s"], 60)

    def test_start_requires_recent_trusted_frame_and_reachable_view(self):
        self.task.start("cup", 2, FrameStamp("old", 0))
        self.assertEqual(self.task.reason, "no_recent_frame")
        self.task = TaskController(self.memory, lambda row: [])
        commands = self.start()
        self.assertEqual(commands.kind, "stop")
        # 10-16: an empty plan is named as such (SELECT_RESULT_DIG_20261016.md); was observation_positions_exhausted
        self.assertEqual(self.task.reason, "no_observation_pose_planned")

    def test_occlusion_uses_alternate_pose_and_exhausts_conservatively(self):
        # no search goals: unable_to_confirm as before (10-26: with search goals the task searches instead)
        self.task = TaskController(self.memory, lambda row: self.views)
        self.arrive()
        frame = self.update(scene(1, "occluded"))
        commands = self.task.observe(frame, 1)
        self.assertEqual(commands[0].pose, self.views[1])
        self.task.navigation_result(commands[0].command_id, True, 1.1)
        frame = self.update(scene(2, "occluded"))
        self.task.observe(frame, 2)
        self.assertEqual(self.task.state, "unable_to_confirm")
        self.assertEqual(self.memory.snapshot()[0]["state"], "temporarily_occluded")

    def test_invalid_position_starts_fixed_search_after_independent_free_views(self):
        self.arrive()
        for i, x in [(1, 0), (2, .12)]:
            frame = self.update(scene(i, "removed", camera_x=x))
            commands = self.task.observe(frame, i)
        self.assertEqual(self.memory.snapshot()[0]["state"], "position_invalid")
        self.assertEqual(commands[0].pose, self.search[0])
        self.assertEqual(commands[0].reason, "search")

    def test_ambiguity_requires_explicit_choice(self):
        twin = copy.deepcopy(self.memory.tracks["object-0001"])
        twin.id = "object-0002"
        twin.position[0] += 1
        self.memory.tracks[twin.id] = twin
        self.assertEqual(self.task.start("cup", 0, FrameStamp("f0", 0)), [])
        self.assertEqual(self.task.state, "ambiguous")
        with self.assertRaises(ValueError):
            self.task.select_candidate("not-offered", .1)
        command = self.task.select_candidate(twin.id, .2)[0]
        self.assertEqual(command.target_id, twin.id)

    def test_failed_and_stale_navigation_results(self):
        command = self.start()
        self.task.navigation_result(999, True, .1)
        self.assertEqual(self.task.state, "navigating")
        self.task.navigation_result(command.command_id, False, .2)
        # 10-25: a failure starts the retry path (was: failed / navigation_failed at once)
        self.assertEqual((self.task.state, self.task.reason), ("recovering", "navigation_retry_pending"))
        self.assertEqual(self.task.attempts[0]["reason"], "navigation_failed")
        self.task.navigation_result(command.command_id, True, .3)   # a late success of the failed goal
        self.assertEqual(self.task.state, "recovering")

    def test_previous_pose_success_cannot_complete_new_navigation(self):
        previous = self.arrive()
        frame = self.update(scene(1, "occluded"))
        following = self.task.observe(frame, 1)[0]
        self.task.navigation_result(previous.command_id, True, 1.1)
        self.assertEqual(self.task.state, "navigating")
        self.task.navigation_result(following.command_id, True, 1.2)
        self.assertEqual(self.task.state, "observing")

    def test_previous_task_success_cannot_complete_another_controller(self):
        previous = self.start()
        self.task.cancel(.1)
        self.task = TaskController(self.memory, lambda row: self.views)
        current = self.start()
        self.assertNotEqual(previous.command_id, current.command_id)
        self.task.navigation_result(previous.command_id, True, .2)
        self.assertEqual(self.task.state, "navigating")
        self.task.navigation_result(current.command_id, True, .3)
        self.assertEqual(self.task.state, "observing")

    def test_cancel_stops_active_action_and_rejects_late_success(self):
        command = self.start()
        stopped = self.task.cancel(.1)[0]
        self.assertEqual(stopped.command_id, command.command_id)
        self.task.navigation_result(command.command_id, True, .2)
        self.assertEqual(self.task.state, "cancelled")

    def test_invalid_pose_and_future_clock_stop_without_confirming(self):
        for stamp, reason in [(FrameStamp("bad", 1, False), "pose_invalid"),
                              (FrameStamp("future", 2), "invalid_frame_clock")]:
            with self.subTest(reason=reason):
                self.task = TaskController(self.memory, lambda row: self.views)
                self.arrive()
                self.task.observe(stamp, 1)
                self.assertEqual(self.task.reason, reason)

    def test_timeouts_and_visit_budget(self):
        self.task = TaskController(self.memory, lambda row: self.views,
                                   config=TaskConfig(navigation_timeout_s=.25))
        self.start()
        self.task.tick(.3)
        self.assertEqual((self.task.reason, self.task.attempts[0]["reason"]),
                         ("navigation_retry_pending", "navigation_timeout"))
        self.task = TaskController(self.memory, lambda row: self.views,
                                   config=TaskConfig(task_timeout_s=.25))
        self.start()
        self.task.tick(.3)
        self.assertEqual(self.task.reason, "task_timeout")
        self.task = TaskController(self.memory, lambda row: self.views,
                                   config=TaskConfig(max_visits=1, observation_timeout_s=.25, observe_min_s=0))
        self.arrive()
        self.task.tick(.8)
        self.assertEqual(self.task.reason, "visit_budget_exhausted")

    def test_missing_target_search_is_bounded_and_requires_selection_on_discovery(self):
        self.task = TaskController(self.memory, lambda row: self.views, self.search,
                                   TaskConfig(observation_timeout_s=.2, observe_min_s=0))
        command = self.task.start("book", 0, FrameStamp("f0", 0))[0]
        self.assertEqual(command.pose, self.search[0])
        self.task.navigation_result(command.command_id, True, .1)
        command = self.task.tick(.4)[0]
        self.assertEqual(command.pose, self.search[1])
        self.task.navigation_result(command.command_id, True, .5)
        self.task.tick(.8)
        self.assertEqual(self.task.state, "not_found")

    def test_single_new_identity_in_search_is_found_under_its_own_id(self):
        # 10-26 (owner): exactly one fresh same-class candidate in search -> found, target_id = that track
        self.memory.tracks["object-0001"].state = "position_invalid"
        command = self.start()
        self.task.navigation_result(command.command_id, True, .5)
        frame = self.other_object_frame()
        commands = self.task.observe(frame, 1)
        self.assertEqual((self.task.state, self.task.reason), ("found", "search_single_candidate"))
        self.assertEqual(self.task.target_id, "object-0002")
        self.assertEqual(self.task.found_row["id"], "object-0002")
        self.assertEqual(self.task.events[-1]["previous_target_id"], "object-0001")
        self.assertEqual([c.kind for c in commands], ["stop", "approach"])

    def test_two_new_identities_in_search_still_require_selection(self):
        self.memory.tracks["object-0001"].state = "position_invalid"
        command = self.start()
        self.task.navigation_result(command.command_id, True, .5)
        frame = self.other_object_frame()
        twin = copy.deepcopy(self.memory.tracks["object-0002"])
        twin.id = "object-0003"
        twin.position[1] += 1
        self.memory.tracks[twin.id] = twin
        self.task.observe(frame, 1)
        self.assertEqual(self.task.state, "ambiguous")
        self.assertEqual(sorted(r["id"] for r in self.task.candidates), ["object-0002", "object-0003"])
        # Selecting a candidate still requires navigation and another frame.
        command = self.task.select_candidate("object-0002", 1.1)[0]
        self.task.navigation_result(command.command_id, True, 1.2)
        self.assertEqual(self.task.state, "observing")
        frame, detections = scene(2)
        frame.T_world_camera[0, 3] = .8
        self.task.observe(self.update((frame, detections)), 2)
        self.assertEqual(self.task.state, "found")
        self.assertEqual(self.task.target_id, "object-0002")

    def test_retry_config_validation(self):
        for bad in ({"nav_retries_same_goal": -1}, {"max_nav_failures": 0}, {"min_attempt_time_s": -1},
                    {"retry_pause_s": float("nan")}, {"fault_nav_fail_first": True}, {"max_nav_failures": 2.0}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                TaskConfig(**bad)
        self.assertEqual((TaskConfig().nav_retries_same_goal, TaskConfig().max_nav_failures,
                          TaskConfig().min_attempt_time_s, TaskConfig().retry_pause_s,
                          TaskConfig().fault_nav_fail_first), (1, 3, 20.0, 1.0, 0))

    def test_contract_validation(self):
        with self.assertRaises(ValueError):
            PoseGoal(float("nan"), 0, 0)
        with self.assertRaises(ValueError):
            TaskConfig(max_visits=0)
        self.start()
        with self.assertRaises(RuntimeError):
            self.start()
        with self.assertRaises(ValueError):
            self.task.tick(-1)


class RetryTests(unittest.TestCase):
    """10-25 find-retry (owner-approved design): known answers for the bounded two-level retry."""

    def setUp(self):
        self.memory = Memory()
        frame, detections = scene(0)
        self.memory.update_memory(process_frame(frame, detections), compute_geometry(frame, self.memory), frame)
        self.views = [PoseGoal(0, 0, 0), PoseGoal(.2, 0, 0), PoseGoal(.4, 0, 0)]
        self.search = [PoseGoal(1, 0, 0), PoseGoal(2, 0, 0)]
        self.frames = 0
        self.t = 0.0

    def make(self, search=(), **cfg):
        self.task = TaskController(self.memory, lambda row: self.views, search, TaskConfig(**cfg))
        return self.task

    def run_to(self, until):
        """Frames every .5 s up to ``until`` (keeps the stream watchdog quiet); returns the commands emitted."""
        out = []
        while self.t + .5 <= until + 1e-9:
            self.t += .5
            self.frames += 1
            out += self.task.observe(FrameStamp("r%d" % self.frames, self.t), self.t)
        return out

    def start(self, goal="找红色杯子"):
        return self.task.start(goal, 0, FrameStamp("r0", 0))

    def nav_fail(self, command, at, reason="navigation_failed"):
        self.run_to(at)
        return self.task.navigation_result(command.command_id, False, self.t, reason)

    def test_first_failure_retries_same_goal_then_succeeds(self):
        self.make()
        first = self.start()[0]
        recover = self.nav_fail(first, 5, "navigation_wall_timeout")
        self.assertEqual([(c.kind, c.pose, c.reason) for c in recover],
                         [("recover", self.views[0], "navigation_wall_timeout")])
        self.assertEqual(self.run_to(5.5), [])                # 1 s pause: nothing yet
        again = self.run_to(6)
        self.assertEqual([(c.kind, c.pose) for c in again], [("navigate", self.views[0])])
        self.assertNotEqual(again[0].command_id, first.command_id)
        self.assertEqual(self.task.navigation_result(again[0].command_id, True, 7), [])
        self.assertEqual(self.task.state, "observing")
        snap = self.task.attempts_snapshot()
        self.assertEqual((snap["count"], snap["positions"], snap["max"], snap["failures"]), (2, 1, 3, 1))
        self.assertEqual([(a["result"], a["reason"], a["retry"], a["duration_s"]) for a in snap["items"]],
                         [("failed", "navigation_wall_timeout", False, 5.0), ("succeeded", None, True, 1.0)])

    def test_two_failures_on_one_goal_move_to_next_pose(self):
        self.make()
        first = self.start()[0]
        self.nav_fail(first, 2)
        retry = self.run_to(3)[0]
        self.assertEqual(retry.pose, self.views[0])
        self.assertEqual(self.nav_fail(retry, 4)[0].kind, "recover")
        following = self.run_to(5)[0]
        self.assertEqual((following.kind, following.pose, following.reason), ("navigate", self.views[1], "reobserve"))
        self.task.navigation_result(following.command_id, True, 6)
        self.assertEqual(self.task.state, "observing")
        self.assertEqual(self.task.attempts_snapshot()["positions"], 2)

    def test_three_attempts_exhausted(self):
        self.make()
        command = self.start()[0]
        for at in (2, 4):
            self.nav_fail(command, at)
            command = self.run_to(at + 1)[0]
        self.assertEqual(command.pose, self.views[1])
        end = self.nav_fail(command, 7)
        self.assertEqual([c.kind for c in end], ["stop"])        # no recover: nothing may follow
        self.assertEqual((self.task.state, self.task.reason), ("failed", "navigation_failed_after_retries"))
        self.assertEqual(self.run_to(10), [])
        self.assertEqual(self.task.attempts_snapshot()["count"], 3)

    def test_arrivals_do_not_spend_the_failure_cap(self):
        """10-26 (host e2e flow sim): goals reached cost nothing from max_nav_failures -- 4 observation poses, all
        reached, none confirming: the 4th is still driven to and the poses' own end word follows."""
        self.views.append(PoseGoal(.6, 0, 0))
        self.make(observation_timeout_s=1, observe_min_s=0)
        command = self.start()[0]
        poses = []
        while command is not None:
            poses.append(command.pose)
            self.task.navigation_result(command.command_id, True, self.t)
            out = self.run_to(self.t + 1)
            command = out[0] if out and out[0].kind == "navigate" else None
        self.assertEqual(poses, self.views)
        self.assertEqual((self.task.state, self.task.reason), ("unable_to_confirm", "observation_positions_exhausted"))
        snap = self.task.attempts_snapshot()
        self.assertEqual((snap["count"], snap["positions"], snap["failures"], snap["max"]), (4, 4, 0, 3))

    def test_five_search_waypoints_all_reached_then_search_exhausted(self):
        """Known answer: 5 search waypoints, every one reached, target never seen -> all 5 driven, search_exhausted."""
        search = [PoseGoal(float(k), 1, 0) for k in range(1, 6)]
        self.make(search=search, observation_timeout_s=1, observe_min_s=0)
        command = self.start("book")[0]
        poses = []
        while command is not None:
            poses.append(command.pose)
            self.task.navigation_result(command.command_id, True, self.t)
            out = self.run_to(self.t + 1)
            command = out[0] if out and out[0].kind == "navigate" else None
        self.assertEqual(poses, search)
        self.assertEqual((self.task.state, self.task.reason), ("not_found", "search_exhausted"))
        self.assertEqual(self.task.attempts_snapshot()["count"], 5)

    def test_mixed_arrivals_and_failures_count_only_failures(self):
        """reached / failed / retry reached / failed / retry failed: the 3rd FAILURE (5th navigate) ends it."""
        self.views = [PoseGoal(k * .2, 0, 0) for k in range(5)]
        self.make(observation_timeout_s=1, observe_min_s=0)
        command = self.start()[0]
        for ok in (True, False, True, False):
            if ok:
                self.task.navigation_result(command.command_id, True, self.t)
                command = self.run_to(self.t + 1)[0]
            else:
                self.nav_fail(command, self.t + .5)
                command = self.run_to(self.t + 1)[0]
            self.assertEqual(command.kind, "navigate")
        self.assertEqual(self.task.attempts_snapshot()["failures"], 2)
        end = self.nav_fail(command, self.t + .5)
        self.assertEqual([c.kind for c in end], ["stop"])
        self.assertEqual((self.task.state, self.task.reason), ("failed", "navigation_failed_after_retries"))
        snap = self.task.attempts_snapshot()
        self.assertEqual((snap["count"], snap["positions"], snap["failures"]), (5, 3, 3))
        self.assertEqual([(round(a["pose"]["x"], 3), a["result"]) for a in snap["items"]],
                         [(0, "succeeded"), (.2, "failed"), (.2, "succeeded"), (.4, "failed"), (.4, "failed")])

    def test_explicit_selection_restarts_the_cap(self):
        """m5 replay empty_old_position_new_id_selection: 3 failures spent before the user picks a new candidate; the
        picked target still gets its navigation (failures counted from the selection), time budget unchanged."""
        self.make(search=self.search, observation_timeout_s=1, observe_min_s=0)
        self.memory.tracks["object-0001"].state = "position_invalid"
        command = self.start()[0]
        self.assertEqual(command.reason, "search")
        self.nav_fail(command, 1)                                              # waypoint 1 fails twice
        command = self.run_to(2)[0]
        self.nav_fail(command, 3)
        command = self.run_to(4)[0]
        self.assertEqual(command.pose, self.search[1])
        self.task.navigation_result(command.command_id, True, self.t)           # waypoint 2 reached
        self.task._state("ambiguous", "search_candidate_selection_required")   # as observe() does on a new identity
        self.task.candidates = [dict(self.memory.query_memory("找红色杯子", after=0)[0], id="object-0001")]
        self.task.attempts.append(dict(self.task.attempts[-1], n=4, result="failed"))   # a 3rd failure already spent
        self.assertEqual(self.task.attempts_snapshot()["failures"], 3)
        self.memory.tracks["object-0001"].state = "current_confirmed"
        command = self.task.select_candidate("object-0001", self.t)[0]
        self.assertEqual((command.kind, self.task.state), ("navigate", "navigating"))
        self.assertEqual((len(self.task.attempts), self.task.attempts_snapshot()["failures"]), (5, 0))
        recover = self.nav_fail(command, self.t + .5)                          # a failure after it: retried, not ended
        self.assertEqual([c.kind for c in recover], ["recover"])

    def test_stream_timeout_is_configurable(self):
        """10-26: a 3 s gap in processed frames ends the task at the default 2.0 s, not at the car's 5.0 s."""
        for stream_timeout_s, expect in ((2.0, ("failed", "stream_lost")), (5.0, ("navigating", "search"))):
            with self.subTest(stream_timeout_s=stream_timeout_s):
                self.make(search=self.search, stream_timeout_s=stream_timeout_s)
                self.start("book")
                out = self.task.tick(3.0)                                      # no frame since t=0
                self.assertEqual((self.task.state, self.task.reason), expect)
                self.assertEqual([c.kind for c in out], ["stop"] if expect[0] == "failed" else [])
        self.assertEqual([c.kind for c in self.task.tick(5.0)], ["stop"])      # the car's 5 s still bounds it
        self.assertEqual(self.task.reason, "stream_lost")

    def test_too_little_time_left_starts_no_new_attempt(self):
        self.make(task_timeout_s=180, navigation_timeout_s=170)   # a long goal that Nav2 aborts late
        command = self.start()[0]
        end = self.nav_fail(command, 165)                        # 15 s left < 20 s
        self.assertEqual([c.kind for c in end], ["stop"])
        self.assertEqual((self.task.state, self.task.reason), ("failed", "attempts_time_exhausted"))

    def test_attempt_budget_is_capped_by_time_left(self):
        self.make(task_timeout_s=180, navigation_timeout_s=170)
        command = self.start()[0]
        self.nav_fail(command, 150)
        retry = self.run_to(151)[0]
        self.assertEqual(self.task.attempt_timeout_s, 29)
        self.assertEqual(self.task.attempts[-1]["budget_s"], 29)
        self.assertEqual(retry.kind, "navigate")

    def test_three_by_sixty_ends_on_task_budget(self):
        """Car numbers (60/180), every goal timing out: 0-60, 61-121, 122-180 (budget 58) -> task_timeout."""
        self.make(task_timeout_s=180, navigation_timeout_s=60)
        self.start()
        out = self.run_to(180)
        self.assertEqual([c.kind for c in out], ["recover", "navigate", "recover", "navigate", "stop"])
        self.assertEqual([c.pose for c in out if c.kind == "navigate"], [self.views[0], self.views[1]])
        self.assertEqual((self.task.state, self.task.reason), ("failed", "task_timeout"))
        self.assertEqual([(a["started"], a["result"], a["budget_s"]) for a in self.task.attempts],
                         [(0, "failed", 60), (61, "failed", 60), (122, "interrupted", 58)])

    def test_cancel_does_not_retry(self):
        self.make()
        first = self.start()[0]
        self.nav_fail(first, 2)
        stop = self.task.cancel(2.2)
        self.assertEqual([c.kind for c in stop], ["stop"])
        self.assertEqual((self.task.state, self.task.reason), ("cancelled", "user_cancelled"))
        self.assertEqual(self.run_to(5), [])
        self.make()
        command = self.start()[0]
        self.task.cancel(.1)
        self.assertEqual(self.task.navigation_result(command.command_id, False, .2), [])
        self.assertEqual((self.task.state, len(self.task.attempts)), ("cancelled", 1))

    def test_adapter_fault_does_not_retry(self):
        self.make()
        self.start()
        self.assertEqual([c.kind for c in self.task.fail("localization_invalid_or_timeout", .1)], ["stop"])
        self.assertEqual(self.task.state, "failed")

    def test_fault_injection_counts(self):
        self.make(fault_nav_fail_first=2, max_nav_failures=5)
        command = self.start()[0]
        out = self.task.navigation_result(command.command_id, True, .5)
        self.assertEqual((out[0].kind, out[0].reason), ("recover", "injected_navigation_failure"))
        command = self.run_to(1.5)[0]
        self.task.navigation_result(command.command_id, True, self.t)          # 2nd: injected again
        command = self.run_to(2.5)[0]
        self.assertEqual(command.pose, self.views[1])
        self.task.navigation_result(command.command_id, True, self.t)          # 3rd: real
        self.assertEqual(self.task.state, "observing")
        self.assertEqual([(a["result"], a["injected"]) for a in self.task.attempts],
                         [("failed", True), ("failed", True), ("succeeded", False)])
        self.make()                                                            # default off
        command = self.start()[0]
        self.task.navigation_result(command.command_id, True, .5)
        self.assertEqual(self.task.state, "observing")

    def test_search_retries_waypoint_then_next(self):
        self.make(search=self.search)
        command = self.start("book")[0]
        self.assertEqual(command.pose, self.search[0])
        self.nav_fail(command, 1)
        command = self.run_to(2)[0]
        self.assertEqual((command.pose, command.reason), (self.search[0], "search"))
        self.nav_fail(command, 3)
        command = self.run_to(4)[0]
        self.assertEqual(command.pose, self.search[1])
        end = self.nav_fail(command, 5)
        self.assertEqual((self.task.state, self.task.reason), ("failed", "navigation_failed_after_retries"))

    def test_search_retry_follows_mapping_waypoint_order(self):
        """10-26 merge with search-waypoints: the runner hands the controller ros_runtime.select_search_waypoints'
        order (nearest-first chain from the robot), and "next waypoint" after the same-goal retry is that order's
        next item, not the mapping file's."""
        import numpy as np
        from host.ros_runtime import FreeMap, select_search_waypoints

        grid = FreeMap(np.zeros((100, 100), dtype=np.int8), .1, -5.0, -5.0)       # 10 m x 10 m, all free
        mapped = [PoseGoal(3, 0, 0), PoseGoal(1, 0, 0), PoseGoal(2, 0, 0)]       # file order: far, near, mid
        ordered, info = select_search_waypoints((), mapped, "spin", grid, (0., 0., 0.))
        self.assertEqual((ordered, info["source"]), ([mapped[1], mapped[2], mapped[0]], "mapping_trajectory"))
        self.make(search=ordered, max_nav_failures=5)
        command = self.start("book")[0]
        self.assertEqual(command.pose, mapped[1])
        self.nav_fail(command, 1)
        command = self.run_to(2)[0]
        self.assertEqual(command.pose, mapped[1])                                 # same goal once
        self.nav_fail(command, 3)
        command = self.run_to(4)[0]
        self.assertEqual(command.pose, mapped[2])                                 # then the next in that order

    def test_last_pose_failing_twice_ends_with_exhausted_word(self):
        self.views = [PoseGoal(0, 0, 0)]
        self.make(max_nav_failures=5)
        command = self.start()[0]
        self.nav_fail(command, 1)
        command = self.run_to(2)[0]
        end = self.nav_fail(command, 3)
        self.assertEqual([c.kind for c in end], ["stop"])
        self.assertEqual((self.task.state, self.task.reason), ("unable_to_confirm", "observation_positions_exhausted"))


if __name__ == "__main__":
    unittest.main()
