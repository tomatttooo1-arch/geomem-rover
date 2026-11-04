#!/usr/bin/env python3
"""Deterministic CPU replay of the M5 Runtime task loop with external-I/O fixtures.

This is a software-boundary replay, not a ROS, DDS, camera, Nav2, localization,
or robot test.  The production Runtime class is loaded from its complete source
AST without changing a business method.  Synthetic ``scene`` frames and their
explicit oracle boxes enter at the completed-inference Future boundary; Runtime
``poll`` performs the real geometry, Memory, task, planner and action handling.
"""

import argparse
from concurrent.futures import Future
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from host.fpga_clock import now_ns as raw_now_ns
from host.ros_runtime import FreeMap
from host.task_control import PoseGoal
from perception.fpga_replay.core import UNGATED_CREATION
from perception.fpga_replay.synthetic import scene
from tools.rtabmap_runtime_probe_support import World, gate, healthy, load_runtime


SOURCE_PATHS = (
    "host/ros_runtime.py",
    "host/task_control.py",
    "perception/fpga_replay/core.py",
    "perception/fpga_replay/synthetic.py",
    "tools/rtabmap_runtime_probe_support.py",
    "tools/m5_software_replay.py",
    "tests/test_m5_runtime.py",
)


def _check(condition, message):
    if not condition:
        raise AssertionError(message)


def _old_timing_task_controller(runtime_class):
    """10-30: TaskController in the runner class's globals -> the same with observe_min_s=0 (3 s budget)."""
    g = getattr(runtime_class.__init__, "__globals__", None)
    base = g.get("TaskController") if g is not None else None
    if base is None or getattr(base, "_replay_old_timing", False):
        return
    from host.task_control import TaskConfig

    class TaskController(base):
        _replay_old_timing = True

        def __init__(self, *a, config=None, **k):
            super().__init__(*a, config=replace(config or TaskConfig(), observe_min_s=0.0), **k)

    TaskController.__name__ = TaskController.__qualname__ = base.__name__
    g["TaskController"] = TaskController


class Replay:
    """One isolated Runtime plus deterministic external boundary controls."""

    def __init__(self, name):
        Replay.last_created = self
        self.name = name
        self.world = World("original")
        self.world.waypoints = [PoseGoal(-2., -1., 0.), PoseGoal(-2., 1., 0.)]
        self.node = load_runtime(self.world, Path("unused-runtime-diagnostics.json"))
        # Creation gate off (D-044, 2026-10-21; Config.create_*): the scenario oracles below were written
        # for single-sighting creation (an object seen once is a memory entry), and the regression tests
        # compare current against older runner sources byte for byte; the gate itself is tested in
        # tests/test_memory_creation_gate.py. Applied after construction so every source arm (old
        # runners build a bare Memory()) gets the same memory configuration.
        # 10-27: likewise the legacy existence rule (Config.existence_v3=False): the oracles expect two
        # independent free views and a new id for the replacement found in search; v3 moves the old id to
        # the replacement (owner-approved step 2) -- tests/test_memory_existence_20261027.py covers it.
        self.node.memory.cfg = replace(self.node.memory.cfg, existence_v3=False, **UNGATED_CREATION)
        # 10-30: likewise the 3 s observation budget at an observation pose (TaskConfig.observe_min_s=0): the
        # oracles and the old-source byte comparisons are timed for it; the 6 s minimum observation (the car
        # default) is covered by tests/test_search_mvp_20261026.py ObserveMinTimeTests.
        # Old runner sources build TaskController() with the default config, so the controller factory in the
        # runner's own namespace is wrapped (every source arm gets the same timing).
        _old_timing_task_controller(type(self.node))
        self.gate = gate(self.world, "m5-{}-localization".format(name))
        self.node.map = FreeMap(np.zeros((120, 120), dtype=np.int8), .1, -6., -6.)
        # Identity is intentional for the analytic scene: its optical Z is world Z.
        self.node.base_camera = np.eye(4)
        self.node.robot_xy = (.5, 0.)
        self.next_index = 2
        for wall, stamp in ((.1, 1100000000), (.2, 1200000000), (.3, 1300000000)):
            self._localize(wall, stamp)

    def _localize(self, wall, stamp):
        self.world.clock(wall, stamp)
        row = healthy(self.world, self.gate, stamp)
        self.node.localization.update(row, self.world.wall, self.world.ros_ns)

    def frame(self, kind="present", camera_x=.5, suppress=False, wall=None, world_x_shift=0.):
        """Deliver one synthetic scene at the completed-inference boundary."""
        index = self.next_index
        self.next_index += 1
        stamp = index * 10**9
        self._localize(wall if wall is not None else .3 + index * .1, stamp)
        frame, detections = scene(index, kind, camera_x=camera_x, suppress=suppress)
        frame.T_world_camera[0, 3] += world_x_shift
        token = self.node.localization.token(stamp, self.world.wall, self.world.ros_ns)
        _check(token is not None, "fixture failed to obtain a real localization lease token")
        frame.localization_token = token
        # this boundary stands in for on_images: the RAW arrival the runner's 200 ms geometry
        # age gate ages from (a frame without it is refused as geometry_age_unknown since
        # 2026-10-12, docs/design/RUNNER_AGE_GATE_20261011.md); runtimes without the gate ignore it
        frame.geometry_origin_ns = raw_now_ns()
        future = Future()
        future.set_result(detections)
        self.world.emit(
            "inference_boundary_delivery",
            frame_id=frame.frame_id,
            source="perception.fpga_replay.synthetic.scene",
            detector="explicit_synthetic_oracle_boxes",
            boundary="completed_inference_future_not_camera_or_dds",
            oracle_detection_count=len(detections),
            analytic_world_x_shift=world_x_shift,
        )
        self.node.inference = future
        self.node.results.put((frame, future))
        self.node.poll()
        return frame

    def query(self, text):
        self.world.emit("external_query", text=text)
        self.node.query(text)

    def accept(self):
        request = self.world.action_requests[-1]
        _check(not request["future"].done(), "action request already accepted")
        self.world.emit("nav2_fixture_accept", request_id=request["handle"].request_id)
        request["future"].set_result(request["handle"])
        return request

    def result(self, request, succeeded):
        status = 4 if succeeded else 6
        self.world.emit(
            "nav2_fixture_terminal_receipt",
            request_id=request["handle"].request_id,
            status=status,
            synthetic_external_receipt=True,
        )
        request["handle"].result_future.set_result(NS(status=status))

    def navigate(self, succeeded=True):
        request = self.accept()
        self.result(request, succeeded)
        return request

    def advance(self, wall, ros_seconds, label):
        self.world.clock(wall, round(ros_seconds * 10**9))
        self.world.emit("external_timer", label=label)
        self.node.poll()

    def published(self, event):
        return [row["payload"] for row in self.world.events
                if row["event"] == "publish" and row.get("topic") == "/fpga/status"
                and row["payload"].get("event") == event]

    def last_action_pose(self):
        sends = [row for row in self.world.events if row["event"] == "action_send"]
        _check(sends, "Runtime did not send a navigation action")
        position = sends[-1]["message"]["pose"]["pose"]["position"]
        return position["x"], position["y"]

    def close(self):
        # __init__ publishes last_created before the Runtime exists, so a failure while
        # building one arrives here with nothing to close. Closing must never replace the
        # error that is being reported.
        node = getattr(self, "node", None)
        if node is not None:
            node.close()


def _seed_and_query(name, query="找红色杯子"):
    replay = Replay(name)
    replay.frame("present")
    replay.query(query)
    return replay


def scenario_found_after_fresh_frame():
    r = _seed_and_query("found_after_fresh_frame")
    target = r.node.memory.query_memory("红色杯子")[0]
    _check(r.node.task.target_id == target["id"], "target did not come from Memory query")
    plans = r.published("observation_plan")
    _check(plans and plans[-1]["target_id"] == target["id"], "production planner did not run")
    goal_xy = r.last_action_pose()
    _check(goal_xy != tuple(target["position_m"][:2]),
           "navigation goal was the remembered object coordinate")
    r.navigate(True)
    _check(r.node.task.state == "observing", "navigation success directly became found")
    fresh = r.frame("present")
    _check(r.node.task.state == "found", "fresh same-target frame did not confirm")
    _check(r.node.memory.query_memory("红色杯子")[0]["last_seen_frame"] == fresh.frame_id,
           "fresh confirmation was not written by Runtime poll")
    return r


def scenario_arrival_old_frame_only():
    r = _seed_and_query("arrival_old_frame_only")
    old_id = r.node.frame.frame_id
    r.navigate(True)
    r.node.publish_task()
    _check(r.node.task.state == "observing", "arrival or old Memory incorrectly confirmed target")
    _check(r.node.memory.snapshot()[0]["last_seen_frame"] == old_id, "fixture added a hidden fresh frame")
    return r


def scenario_navigation_unreachable():
    r = _seed_and_query("navigation_unreachable")
    r.navigate(False)
    # 10-25 find-retry: an aborted goal is a failed attempt -- stopped, costmaps cleared, retried after a pause
    _check(r.node.task.state == "recovering" and r.node.task.reason == "navigation_retry_pending",
           "aborted navigation did not enter the bounded retry")
    _check(r.node.task.attempts[-1]["result"] == "failed" and r.node.nav is None,
           "aborted attempt was not recorded / its goal not retired")
    return r


def scenario_missing_bounded():
    r = _seed_and_query("missing_bounded", "找书")
    visits = 0
    while r.node.task.state not in r.node.task.TERMINAL:
        if r.node.nav:
            r.navigate(True)
            visits += 1
        else:
            r.frame("removed")
    _check(r.node.task.state == "not_found" and r.node.task.reason == "search_exhausted",
           "absent target search did not end at its fixed waypoint bound")
    _check(visits == len(r.world.waypoints), "search visit count did not match fixed coverage list")
    return r


def scenario_cancel_late_accept_success():
    r = _seed_and_query("cancel_late_accept_success")
    request = r.world.action_requests[-1]
    r.node.cancel(NS())
    _check(r.node.task.state == "cancelled", "cancel did not terminate task")
    r.accept()
    r.result(request, True)
    _check(r.node.task.state == "cancelled" and r.node.nav is None,
           "late action acceptance/success revived cancelled task")
    _check(r.published("navigation_retired"), "late success was not audited as retired")
    return r


def scenario_no_new_frame_timeout_published():
    r = _seed_and_query("no_new_frame_timeout_published")
    r.advance(2.8, 2.8, "no_new_processed_frame")
    _check(r.node.task.state == "failed", "silent processed stream did not terminate task")
    tasks = r.published("task")
    _check(tasks and tasks[-1]["state"] == "failed",
           "timeout terminal task state was not externally published")
    return r


def scenario_localization_failure():
    r = _seed_and_query("localization_failure")
    r.gate.revoke("lost", "synthetic_external_localization_loss")
    r.world.clock(.6, 2100000000)
    r.node.on_localization(NS(data=json.dumps(r.gate.status(r.world.wall, r.world.ros_ns))))
    _check(r.node.task.state == "failed" and "localization" in r.node.task.reason,
           "localization invalidation did not fail active task")
    return r


def scenario_occlusion_changes_view():
    r = _seed_and_query("occlusion_changes_view")
    first_pose = r.last_action_pose()
    r.navigate(True)
    r.frame("occluded")
    _check(r.node.memory.snapshot()[0]["state"] == "temporarily_occluded",
           "occlusion was not retained as uncertainty")
    _check(r.node.task.state == "navigating", "occlusion did not request another observation pose")
    second_pose = r.last_action_pose()
    _check(first_pose != second_pose,
           "occlusion retried the identical observation pose")
    return r


def scenario_empty_old_position_new_id_selection():
    r = _seed_and_query("empty_old_position_new_id_selection")
    original_id = r.node.task.target_id
    r.navigate(True)
    # Keep supplying recent processed frames while the observation budget expires.
    # Two geometrically independent free views are required before Memory invalidates
    # the old position; the second is taken after Runtime requests another pose.
    for _ in range(4):
        r.frame("removed", camera_x=.5)
    _check(r.node.task.state == "navigating", "bounded observation did not request another view")
    r.navigate(True)
    r.frame("removed", camera_x=.62)
    old = next(row for row in r.node.memory.snapshot() if row["id"] == original_id)
    _check(old["state"] == "position_invalid", "independent empty views did not invalidate old position")
    _check(r.node.task.state == "navigating" and r.last_action_pose() == (-2., -1.),
           "Runtime did not enter the fixed, truth-independent search list")

    r.navigate(True)
    r.frame("present", camera_x=.5, world_x_shift=.8)
    # 10-26 search MVP (already deployed in 0480eb5): one fresh candidate may
    # end the search. Keep the earlier manual-selection arm for old Runtime
    # comparisons; both arms must retain a new ID and fresh visual evidence.
    if r.node.task.reason == "search_single_candidate":
        replacement_id = r.node.task.target_id
        _check(r.node.task.state == "found", "single-candidate verdict did not finish the search")
        _check(replacement_id != original_id, "replacement reused the invalid old track identity")
        replacement = r.node.memory.tracks[replacement_id]
        _check(replacement.state == "current_confirmed" and
               replacement.last_seen > old["last_seen"] and
               replacement.last_seen_frame != old["last_seen_frame"],
               "single-candidate verdict did not use a fresh replacement observation")
        return r
    _check(r.node.task.state == "ambiguous", "new identity was silently accepted during search")
    _check(len(r.node.task.candidates) == 1, "new search identity was not exposed for selection")
    replacement_id = r.node.task.candidates[0]["id"]
    _check(replacement_id != original_id, "replacement reused the invalid old track identity")
    r.node.select(NS(data=replacement_id))
    _check(r.node.task.state == "navigating", "selection did not require a new observation navigation")
    r.navigate(True)
    r.frame("present", camera_x=.5, world_x_shift=.8)
    _check(r.node.task.state == "found" and r.node.task.target_id == replacement_id,
           "selected replacement was not confirmed by a subsequent frame")
    return r


SCENARIOS = (
    scenario_found_after_fresh_frame,
    scenario_arrival_old_frame_only,
    scenario_navigation_unreachable,
    scenario_missing_bounded,
    scenario_cancel_late_accept_success,
    scenario_no_new_frame_timeout_published,
    scenario_localization_failure,
    scenario_occlusion_changes_view,
    scenario_empty_old_position_new_id_selection,
)


def source_manifest():
    return {
        "head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "git_status": subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True),
        "sources": {path: hashlib.sha256((ROOT / path).read_bytes()).hexdigest()
                    for path in SOURCE_PATHS if (ROOT / path).exists()},
    }


def run_scenarios(output=None):
    """Run the same scenario functions used by unittest and optionally persist evidence."""
    results = []
    identity = iter("m5-task-%04d" % i for i in range(1000))

    class Identity:
        def __init__(self):
            self.hex = next(identity)

    manifest = source_manifest()
    for function in SCENARIOS:
        Replay.last_created = None
        replay = None
        error = None
        try:
            with patch("host.task_control.uuid.uuid4", side_effect=Identity):
                replay = function()
        except Exception as exc:  # preserve raw evidence before returning nonzero
            error = "{}: {}".format(type(exc).__name__, exc)
            replay = Replay.last_created
        # A construction failure leaves a partially built Replay in last_created. Read
        # only what it actually reached; assuming a whole object here replaces the real
        # cause with an AttributeError raised by this recovery path itself.
        world = getattr(replay, "world", None)
        events = world.events if world is not None else []
        if replay:
            replay.close()
        node = getattr(replay, "node", None)
        name = function.__name__.replace("scenario_", "")
        result = {
            "name": name,
            "passed": error is None,
            "error": error,
            "final_task_state": node.task.state if node is not None and node.task else None,
            "event_count": len(events),
            "boundary": "CPU software replay with synthetic oracle detections and external Nav2 receipts",
            **manifest,
        }
        results.append(result)
        if output is not None:
            directory = output / name
            directory.mkdir()
            with (directory / "events.jsonl").open("x", encoding="utf8") as stream:
                for row in events:
                    stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            (directory / "summary.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf8")
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path,
                        help="new evidence directory; an existing path is rejected")
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {
        **source_manifest(),
        "command": sys.argv if argv is None else [str(Path(__file__).relative_to(ROOT)), *argv],
        "python": sys.version,
        "platform": platform.platform(),
        "scope": "software boundary only; no ROS graph, DDS, physical camera, detector, Nav2 server, or robot",
        "inference_input": "perception.fpga_replay.synthetic.scene oracle boxes delivered as completed Future",
        "runtime_loading": "complete production Runtime class AST, original policy, zero business-method edits",
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    results = run_scenarios(args.output)
    summary = {"passed": all(row["passed"] for row in results), "scenarios": results}
    (args.output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf8")
    print(json.dumps({row["name"]: row["passed"] for row in results}, ensure_ascii=False, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
