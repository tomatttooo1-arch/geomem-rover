"""10-29 owner (边建图边寻物时 Nano CPU 过载): a processed-frame gap no longer ends a task.

Run 20261029-211234-live: every frame expired waiting for clock/TF for ~5 s during a navigation; the task ended
failed:stream_lost and the goal was cancelled. On the car (runner --input-hiccup drop -> TaskConfig
stream_stall_waits=True) such a gap is now a warning: the task keeps its goal, sends no new goal while it lasts,
and carries on when frames come back; task_timeout_s still ends it. A goal the runner could not send because its
input was not ready is held and resent (no failed attempt).

Run: PYTHONPATH=. python3 tests/test_stream_stall_20261029.py
"""
from dataclasses import dataclass, replace
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.task_control import PoseGoal, TaskConfig, TaskController  # noqa: E402
from perception.fpga_replay.core import (Config as _Config, Memory as _Memory, UNGATED_CREATION,  # noqa: E402
                                        compute_geometry, process_frame)
from perception.fpga_replay.synthetic import scene  # noqa: E402


@dataclass
class FrameStamp:
    frame_id: str
    timestamp: float
    pose_valid: bool = True


def Memory():
    return _Memory("geometry", replace(_Config(), **UNGATED_CREATION))


class ControllerStall(unittest.TestCase):
    def setUp(self):
        self.memory = Memory()
        self.update(scene(0))
        self.views = [PoseGoal(0, 0, 0), PoseGoal(.2, 0, 0)]

    def update(self, pair):
        frame, detections = pair
        self.memory.update_memory(process_frame(frame, detections), compute_geometry(frame, self.memory), frame)
        return frame

    def make(self, **kw):
        cfg = dict(stream_stall_waits=True, stream_timeout_s=2.0)
        cfg.update(kw)
        self.task = TaskController(self.memory, lambda row: self.views, config=TaskConfig(**cfg))
        return self.task.start("找红色杯子", 0, FrameStamp("initial", 0))[0]

    def reasons(self):
        return [e["reason"] for e in self.task.events]

    def test_stall_warns_keeps_the_goal_and_the_task_still_finds_the_target(self):
        command = self.make()
        self.assertEqual(command.kind, "navigate")
        self.assertEqual(self.task.tick(1.9), [])
        self.assertIsNone(self.task.stall_snapshot())
        self.assertEqual(self.task.tick(5.0), [])                       # 5 s without a frame: no stop
        self.assertEqual((self.task.state, self.task.reason), ("navigating", "reobserve"))
        self.assertEqual(self.task.stall_snapshot(), {"gap_s": 5.0, "n": 1})
        self.assertEqual(self.reasons().count("stream_stalled"), 1)
        self.assertEqual(self.task.tick(5.5), [])                       # warned once per stall
        self.assertEqual(self.reasons().count("stream_stalled"), 1)
        self.assertEqual(self.task.navigation_result(command.command_id, True, 5.6), [])   # Nav2 arrives anyway
        self.assertEqual(self.task.state, "observing")
        frame = self.update(scene(6))                                   # frames are back
        self.task.observe(frame, 6.0)
        self.assertEqual((self.task.state, self.task.reason), ("found", "fresh_matching_observation"))
        self.assertIsNone(self.task.stream_stall)
        resumed = [e for e in self.task.events if e["reason"] == "stream_resumed"]
        self.assertEqual(len(resumed), 1)
        self.assertEqual(resumed[0]["detail"]["gap_s"], 6.0)
        self.assertEqual(self.task.attempts_snapshot()["failures"], 0)

    def test_default_still_ends_the_task_stream_lost(self):
        """Negative control: the host default (stream_stall_waits False) as before."""
        self.make(stream_stall_waits=False)
        commands = self.task.tick(5.0)
        self.assertEqual([c.kind for c in commands], ["stop"])
        self.assertEqual((self.task.state, self.task.reason), ("failed", "stream_lost"))

    def test_task_timeout_still_ends_a_stalled_task(self):
        self.make(task_timeout_s=30.0)
        self.assertEqual(self.task.tick(29.0), [])
        self.assertEqual(self.task.state, "navigating")
        commands = self.task.tick(30.0)
        self.assertEqual([c.kind for c in commands], ["stop"])
        self.assertEqual((self.task.state, self.task.reason), ("failed", "task_timeout"))

    def test_two_stalls_are_counted_apart(self):
        self.make()
        self.task.tick(3.0)
        self.task.observe(FrameStamp("f3", 3.1), 3.1)
        self.assertIsNone(self.task.stall_snapshot())
        self.task.tick(6.0)
        self.assertEqual(self.task.stall_snapshot()["n"], 2)
        self.assertEqual(self.task.stream_stalls, 2)

    def test_no_new_goal_while_stalled_then_resent_when_frames_return(self):
        command = self.make()
        out = self.task.navigation_result(command.command_id, False, 1.0)     # Nav2 aborted: retry path
        self.assertEqual([c.kind for c in out], ["recover"])
        self.assertEqual(self.task.tick(2.5), [])                              # pause over, but no frame for 2.5 s
        self.assertEqual(self.task.state, "recovering")
        self.assertIsNotNone(self.task.stream_stall)
        self.assertEqual(self.task.tick(4.0), [])
        out = self.task.observe(FrameStamp("back", 4.5), 4.5)                  # frames back: the retry goes out
        self.assertEqual([c.kind for c in out], ["navigate"])
        self.assertEqual(out[0].pose, command.pose)
        self.assertEqual(self.task.attempts_snapshot()["failures"], 1)         # only the Nav2 abort

    def test_held_goal_is_no_failure_and_is_resent(self):
        command = self.make()
        t = 0.5
        for i in range(10):                                                   # more holds than any failure cap
            self.assertEqual(self.task.navigation_held(command.command_id, t), [])
            self.assertEqual((self.task.state, self.task.reason), ("recovering", "navigation_input_wait"))
            self.task.observe(FrameStamp("h%d" % i, t + .1), t + .1)          # frames keep coming
            out = self.task.tick(t + 1.2)
            self.assertEqual([c.kind for c in out], ["navigate"])
            self.assertEqual(out[0].pose, command.pose)
            command, t = out[0], t + 1.3
        snap = self.task.attempts_snapshot()
        self.assertEqual((snap["count"], snap["failures"]), (1, 0))           # only the live attempt is listed
        self.assertEqual(self.task.state, "navigating")

    def test_held_ignores_a_retired_command(self):
        command = self.make()
        self.assertEqual(self.task.navigation_held("other:9", .5), [])
        self.assertEqual(self.task.state, "navigating")
        self.assertEqual(self.task._active_id, command.command_id)


class ConsoleMessage(unittest.TestCase):
    def test_bridge_message_shows_the_stall_in_front(self):
        from ui.geomem.bridge.gmb_proto import compose_task
        group = {"state": "navigating", "reason_task": "reobserve", "target_id": "object-0001", "age_ms": 5,
                 "stream_stall": {"gap_s": 5.1, "n": 1}}
        out = compose_task(group, "boot:1")
        self.assertEqual(out["state"], "navigating")
        self.assertTrue(out["message"].startswith("画面处理中断，等待恢复"), out["message"])
        self.assertIn("正在前往目标附近", out["message"])
        group.pop("stream_stall")
        self.assertEqual(compose_task(group, "boot:1")["message"], "正在前往目标附近")
        held = dict(group, state="recovering", reason_task="navigation_input_wait", stream_stall={"gap_s": 6, "n": 1})
        self.assertEqual(compose_task(held, "boot:1")["message"], "画面处理中断，等待恢复后继续前往")
        failed = dict(group, state="failed", reason_task="task_timeout", stream_stall={"gap_s": 6, "n": 1})
        self.assertFalse(compose_task(failed, "boot:1")["message"].startswith("画面处理中断"))


if __name__ == "__main__":
    unittest.main()
