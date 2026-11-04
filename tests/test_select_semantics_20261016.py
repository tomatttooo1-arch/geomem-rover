"""select semantics, three changes approved by the owner on 2026-10-16 (docs/design/SELECT_SEMANTICS_20261016.md).

1. A known target on a car without a NavigateToPose server ends ``needs_manual_approach`` (reason
   navigation_server_unavailable, target summary attached, no planning, the car does not move) instead of
   failed / unable_to_confirm depending on whether the target happened to lie inside the stand-in map.
2. GeoMem keeps the protocol ``state`` in the v2 set and adds an optional ``outcome`` (+ ``targetPosition``)
   so the screen tells "unable to confirm" and "go there by hand" apart from a failure, without old pages
   rejecting frames.
3. ``--pose odom``: a mapping export as /map when given, else a 20 m stand-in (was 8 m).
Each new behaviour is checked against the code before the change (main a47a586) on the same input.
"""

import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from dataclasses import dataclass

import numpy as np

from host.task_control import PoseGoal, TaskConfig, TaskController
from host.hub.hub_core import HubCore, NS, MS
from host.hub.hub_server import RosActionsLog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OLD = "a47a58654382576c5be955edd13f4201a776f0d3"   # main before this change
EXPORT = os.path.join(os.path.dirname(ROOT), "selectdig-1016", "runs", "select-dig-20261016", "rtabmap_export",
                      "20261015_190638.yaml")       # the 10-16 run's rtabmap-export (gitignored run data)
_old_cache = {}


def old_source(path):
    try:
        return subprocess.run(["git", "show", "%s:%s" % (OLD, path)], cwd=ROOT, check=True,
                              capture_output=True, text=True).stdout
    except (OSError, subprocess.CalledProcessError):
        raise unittest.SkipTest("old commit %s not reachable from this checkout" % OLD[:7])


def load_old(path, name):
    if name in _old_cache:
        return _old_cache[name]
    text = old_source(path)
    d = tempfile.mkdtemp(prefix="selsem-old-")
    f = os.path.join(d, name + ".py")
    with open(f, "w") as out:
        out.write(text)
    spec = importlib.util.spec_from_file_location(name, f)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod          # dataclasses resolve the module by name
    spec.loader.exec_module(mod)
    _old_cache[name] = mod
    return mod


@dataclass
class Stamp:
    frame_id: str
    timestamp: float
    pose_valid: bool = True


class Memory(object):
    def __init__(self, rows):
        self.rows = rows

    def query_memory(self, goal, after=None):
        return [dict(r) for r in self.rows]


ROW8 = {"id": "object-0008", "label": "bottle", "color": None, "state": "historical_memory",
        "position_m": [4.97, -0.80, 0.12], "last_seen": 1792091480.8, "last_seen_frame": "rgb-8"}
ROW7 = {"id": "object-0007", "label": "bottle", "color": "green", "state": "historical_memory",
        "position_m": [3.53, 0.89, 0.10], "last_seen": 1792091400.0, "last_seen_frame": "rgb-7"}
POSES22 = [PoseGoal(1.0 + 0.1 * i, 0.0, 0.0) for i in range(22)]


class Planner(object):
    def __init__(self, poses):
        self.poses, self.calls = list(poses), 0

    def __call__(self, row):
        self.calls += 1
        return list(self.poses)


def _as(cls, poses):
    """The same poses as the PoseGoal type of cls's module (the old module has its own class)."""
    goal = sys.modules[cls.__module__].PoseGoal
    return [goal(p.x, p.y, p.yaw) for p in poses]


def _summary(task, commands):
    return (task.state, task.reason, [(c.kind, c.reason, c.target_id) for c in commands])


def _single(cls, row, poses, **kw):
    planner = Planner(_as(cls, poses))
    task = cls(Memory([row]), planner, **kw)
    commands = task.start("瓶子", 0, Stamp("f0", 0))
    return task, commands, planner


NAV_DOWN = {"navigation_available": lambda: False}
NAV_UP = {"navigation_available": lambda: True}


class ManualApproachTests(unittest.TestCase):
    def test_0008_empty_plan_without_navigation(self):
        task, commands, planner = _single(TaskController, ROW8, [], **NAV_DOWN)
        self.assertEqual(_summary(task, commands), ("needs_manual_approach", "navigation_server_unavailable",
                                                    [("stop", "navigation_server_unavailable", "object-0008")]))
        self.assertEqual(planner.calls, 0)
        old = load_old("host/task_control.py", "selsem_old_task_control")
        task, commands, _ = _single(old.TaskController, ROW8, [])
        self.assertEqual((task.state, task.reason), ("unable_to_confirm", "no_observation_pose_planned"))

    def test_0007_planned_poses_without_navigation(self):
        task, commands, planner = _single(TaskController, ROW7, POSES22, **NAV_DOWN)
        self.assertEqual(_summary(task, commands)[:2], ("needs_manual_approach", "navigation_server_unavailable"))
        self.assertEqual([c.kind for c in commands], ["stop"])
        self.assertEqual(planner.calls, 0)
        # old: a navigate command, which the runtime refused -> fault -> fail(...) -> failed
        old = load_old("host/task_control.py", "selsem_old_task_control")
        task, commands, _ = _single(old.TaskController, ROW7, POSES22)
        self.assertEqual(commands[0].kind, "navigate")
        task.fail("navigation_server_unavailable", 0.01)
        self.assertEqual((task.state, task.reason), ("failed", "navigation_server_unavailable"))

    def test_the_select_path_of_the_real_run(self):
        """find 瓶子 -> ambiguous (two candidates) -> select object-0008, as on 10-16."""
        for cls_name in ("new", "old"):
            with self.subTest(code=cls_name):
                if cls_name == "new":
                    task = TaskController(Memory([ROW8, ROW7]), Planner([]), **NAV_DOWN)
                else:
                    old = load_old("host/task_control.py", "selsem_old_task_control")
                    task = old.TaskController(Memory([ROW8, ROW7]), Planner([]))
                self.assertEqual(task.start("瓶子", 0, Stamp("f0", 0)), [])
                self.assertEqual(task.state, "ambiguous")
                commands = task.select_candidate("object-0008", 0.5)
                expect = (("needs_manual_approach", "navigation_server_unavailable") if cls_name == "new"
                          else ("unable_to_confirm", "no_observation_pose_planned"))
                self.assertEqual((task.state, task.reason), expect)
                self.assertEqual([c.kind for c in commands], ["stop"])

    def test_server_lost_between_selection_and_navigate(self):
        task, commands, _ = _single(TaskController, ROW7, POSES22, **NAV_UP)
        self.assertEqual(commands[0].kind, "navigate")
        out = task.fail("navigation_server_unavailable", 0.01)
        self.assertEqual((task.state, task.reason), ("needs_manual_approach", "navigation_server_unavailable"))
        self.assertEqual([c.kind for c in out], ["stop"])
        self.assertEqual(task.target["id"], "object-0007")
        task, _, _ = _single(TaskController, ROW7, POSES22, **NAV_UP)
        task.fail("navigation_input_unhealthy", 0.01)
        self.assertEqual((task.state, task.reason), ("failed", "navigation_input_unhealthy"))
        self.assertIsNone(task.target)

    def test_target_summary(self):
        task, _, _ = _single(TaskController, ROW7, POSES22, **NAV_DOWN)
        self.assertEqual(task.target, {"id": "object-0007", "label": "bottle", "color": "green",
                                       "position_m": [3.53, 0.89, 0.10], "state": "historical_memory",
                                       "last_seen": 1792091400.0})
        self.assertIn("needs_manual_approach", TaskController.TERMINAL)
        self.assertEqual(task.tick(1.0), [])       # terminal: no more commands

    def test_position_invalid_and_search_paths_unchanged(self):
        row = dict(ROW8, state="position_invalid")
        search = [PoseGoal(0.5, 0.5, 0.0)]
        old = load_old("host/task_control.py", "selsem_old_task_control")
        for rows in ([row], []):
            with self.subTest(rows=len(rows)):
                new = TaskController(Memory(rows), Planner([]), search, navigation_available=lambda: False)
                was = old.TaskController(Memory(rows), Planner([]), _as(old.TaskController, search))
                a = new.start("瓶子", 0, Stamp("f0", 0))
                b = was.start("瓶子", 0, Stamp("f0", 0))
                self.assertEqual(_summary(new, a), _summary(was, b))
                self.assertEqual(a[0].kind, "navigate")
                new.fail("navigation_server_unavailable", 0.01)
                was.fail("navigation_server_unavailable", 0.01)
                self.assertEqual((new.state, new.reason), (was.state, was.reason))
                self.assertEqual(new.state, "failed")


def _drive(cls, rows, poses, script, **kw):
    """Run one scripted task; returns (state, reason) after each step and the command summaries."""
    task = cls(Memory(rows), Planner(_as(cls, poses)), **kw)
    out = [_summary(task, task.start("瓶子", 0, Stamp("f0", 0)))]
    for step in script:
        if step[0] == "nav":
            ids = task._active_id
            out.append(_summary(task, task.navigation_result(ids, step[1], step[2])))
        elif step[0] == "observe":
            out.append(_summary(task, task.observe(Stamp(step[1], step[2]), step[2])))
    events = [{k: e[k] for k in ("state", "reason", "target_id")} for e in task.events]
    return out, events


class FoundMemory(Memory):
    """Confirms the target in frame f2 (after arriving)."""

    def query_memory(self, goal, after=None):
        rows = super().query_memory(goal, after)
        for r in rows:
            if after is not None and after > 0:
                r.update(state="current_confirmed", confirmed_after_query=True, last_seen_frame="f2", last_seen=1.6)
        return rows


class NavigationAvailableUnchangedTests(unittest.TestCase):
    SCRIPTS = {
        "empty_plan": ([ROW8], [], []),
        "one_pose_exhausted": ([ROW8], [PoseGoal(4.0, -0.8, 0.0)],
                               [("nav", True, 0.1), ("observe", "f1", 1.5), ("observe", "f2", 3.2)]),
        # "navigation_failed" left this list on 10-25: a failed goal now retries (owner-approved find-retry;
        # known answers in test_task_control.RetryTests), so it no longer matches the old code by design.
    }

    def test_same_as_old_code(self):
        old = load_old("host/task_control.py", "selsem_old_task_control")
        for name, (rows, poses, script) in self.SCRIPTS.items():
            for kw in ({}, NAV_UP):
                with self.subTest(case=name, nav=bool(kw)):
                    # 10-30: old timing (3 s at an observation pose); the 6 s minimum has its own tests
                    self.assertEqual(_drive(TaskController, rows, poses, script,
                                            config=TaskConfig(observe_min_s=0.0), **kw),
                                     _drive(old.TaskController, rows, poses, script))

    def test_navigation_failed_same_start_then_retry(self):
        """10-25 find-retry: the case that left SCRIPTS -- the start (first navigate) is still the old code's to the
        byte; only the failed result differs, by design: old -> failed/navigation_failed, now -> a bounded retry."""
        old = load_old("host/task_control.py", "selsem_old_task_control")
        for kw in ({}, NAV_UP):
            with self.subTest(nav=bool(kw)):
                new_out, new_ev = _drive(TaskController, [ROW7], POSES22[:2], [("nav", False, 0.2)], **kw)
                old_out, old_ev = _drive(old.TaskController, [ROW7], POSES22[:2], [("nav", False, 0.2)])
                self.assertEqual(new_out[0], old_out[0])
                self.assertEqual(new_ev[:-1], old_ev[:-1])
                self.assertEqual((old_ev[-1]["state"], old_ev[-1]["reason"]), ("failed", "navigation_failed"))
                self.assertEqual((new_ev[-1]["state"], new_ev[-1]["reason"]),
                                 ("recovering", "navigation_retry_pending"))

    def test_found_same_as_old_code(self):
        old = load_old("host/task_control.py", "selsem_old_task_control")

        def run(cls, **kw):
            task = cls(FoundMemory([ROW7]), Planner(_as(cls, POSES22[:1])), **kw)
            out = [_summary(task, task.start("瓶子", 0, Stamp("f0", 0)))]
            out.append(_summary(task, task.navigation_result(task._active_id, True, 0.1)))
            out.append(_summary(task, task.observe(Stamp("f2", 1.6), 1.6)))
            return out
        new = run(TaskController, **NAV_UP)
        # 10-26 find-near: found now also emits the final-approach command after the same stop (TaskConfig
        # final_approach, default on); everything before it is the old code's to the byte
        self.assertEqual(new[-1][2][-1], ("approach", "fresh_matching_observation", "object-0007"))
        self.assertEqual(new[:-1] + [new[-1][:2] + (new[-1][2][:-1],)], run(old.TaskController))
        self.assertEqual(new[-1][:2], ("found", "fresh_matching_observation"))


class RuntimeWiringTests(unittest.TestCase):
    """The runtime node needs ROS; its wiring is pinned in the source (as in test_select_result_dig)."""

    def test_query_passes_the_server_check_and_task_row_carries_target_only_then(self):
        src = open(os.path.join(ROOT, "host", "ros_runtime.py")).read()
        # the bound method itself (not its result); 10-26 search_planner may follow it as another keyword
        self.assertRegex(src, r"navigation_available=self\.actions\.server_is_ready[,)]")
        self.assertIn('extra = ({"target": self.task.target} if self.task.state == "needs_manual_approach"', src)
        self.assertIn("processed_frames=self.processed, dropped_frames=self.dropped, **extra)", src)
        # the navigate refusal still goes through fault() (owner: item 4 unchanged)
        self.assertIn("reason = navigation_refusal(self.nav is not None, self.actions.server_is_ready(), self.ready())\n"
                      "            if reason:\n                self.fault(reason)", src)


# ---------------------------------------------------------------- hub

def _status(event, **kw):
    kw["event"] = event
    return json.dumps(kw)


TARGET = {"id": "object-0008", "label": "bottle", "color": None, "position_m": [4.97, -0.8, 0.12],
          "state": "historical_memory", "last_seen": 1792091480.8}


def _hub(core_cls, op, row, op_args=None):
    core = core_cls("live", actions=RosActionsLog())
    core.feed_status(_status("startup", sensor_source="hardware"), 0)
    core.feed_status(_status("task", state="ambiguous", reason="candidate_selection_required",
                             target_id=None, candidates=[]), 1 * NS)
    core.command("pc1", True, {"v": 1, "cmd_id": "pc1:1", "op": op, "args": op_args or {}}, 1 * NS + MS)
    core.feed_status(_status("task", **row), 1 * NS + 2 * MS)
    core.tick(1 * NS + 3 * MS)
    rows = [r[1] for r in core.pop_results() if r[1].get("cmd_id") == "pc1:1"]
    return core, rows


MANUAL_ROW = dict(state="needs_manual_approach", reason="navigation_server_unavailable", target_id="object-0008",
                  candidates=[], target=TARGET)


class HubTests(unittest.TestCase):
    def test_select_result_carries_the_target(self):
        _, rows = _hub(HubCore, "select", MANUAL_ROW, {"candidate_id": "object-0008"})
        r = rows[-1]
        self.assertEqual((r["status"], r["task_state"], r["task_reason"], r["target_id"]),
                         ("accepted", "needs_manual_approach", "navigation_server_unavailable", "object-0008"))
        self.assertEqual(r["target"]["position_m"], [4.97, -0.8, 0.12])

    def test_cancel_window_sees_the_new_end_state(self):
        _, rows = _hub(HubCore, "cancel", MANUAL_ROW)
        self.assertEqual(rows[-1]["status"], "done")
        old = load_old("host/hub/hub_core.py", "selsem_old_hub_core")
        _, rows = _hub(old.HubCore, "cancel", MANUAL_ROW)
        self.assertEqual(rows, [])          # old: not terminal -> waits for the window, no "done"

    def test_rows_without_target_unchanged(self):
        old = load_old("host/hub/hub_core.py", "selsem_old_hub_core")
        row = dict(state="failed", reason="navigation_server_unavailable", target_id="object-0007", candidates=[])
        for op, args in (("select", {"candidate_id": "object-0007"}), ("cancel", {})):
            with self.subTest(op=op):
                _, new = _hub(HubCore, op, row, args)
                _, was = _hub(old.HubCore, op, row, args)
                self.assertEqual(new, was)

    def test_state_task_group(self):
        old = load_old("host/hub/hub_core.py", "selsem_old_hub_core")
        core, _ = _hub(HubCore, "select", MANUAL_ROW, {"candidate_id": "object-0008"})
        g = core._task(2 * NS)
        self.assertEqual(g["target"], TARGET)
        row = dict(state="failed", reason="x", target_id="object-0007", candidates=[])
        a, _ = _hub(HubCore, "select", row, {"candidate_id": "object-0007"})
        b, _ = _hub(old.HubCore, "select", row, {"candidate_id": "object-0007"})
        self.assertEqual(a._task(2 * NS), b._task(2 * NS))


# ---------------------------------------------------------------- GeoMem bridge

class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from ui.geomem.bridge import gmb_proto
        cls.gp = gmb_proto

    def group(self, **kw):
        g = {"state": None, "reason_task": None, "target_id": None, "candidates": [], "age_ms": 7}
        g.update(kw)
        return g

    def test_unable_to_confirm_keeps_state_failed_and_adds_outcome(self):
        t = self.gp.compose_task(self.group(state="unable_to_confirm", reason_task="no_observation_pose_planned",
                                            target_id="object-0008"), "b:1")
        self.assertEqual((t["state"], t["outcome"], t["reasonCode"]),
                         ("failed", "unable_to_confirm", "no_observation_pose_planned"))
        self.assertNotIn("targetPosition", t)

    def test_needs_manual_approach(self):
        t = self.gp.compose_task(self.group(state="needs_manual_approach", reason_task="navigation_server_unavailable",
                                            target_id="object-0008", target=TARGET), "b:1")
        self.assertEqual((t["state"], t["outcome"], t["targetId"]), ("failed", "needs_manual_approach", "object-0008"))
        self.assertEqual(t["targetPosition"], [4.97, -0.8, 0.12])
        self.assertEqual(t["message"], "已在记忆里找到瓶子，记忆位置 (4.97, -0.80, 0.12) m；车上没开导航，不会自己开过去，请人工前往")
        t = self.gp.compose_task(self.group(state="needs_manual_approach", reason_task="navigation_server_unavailable",
                                            target_id="object-0008"), "b:1")
        self.assertIsNone(t["targetPosition"])
        self.assertEqual(t["message"], "已在记忆里找到目标；车上没开导航，不会自己开过去，请人工前往")

    def test_other_states_byte_identical_to_old_bridge(self):
        old = load_old("ui/geomem/bridge/gmb_proto.py", "selsem_old_gmb_proto")
        cases = [dict(state="failed", reason_task="navigation_server_unavailable", target_id="object-0007"),
                 dict(state="not_found", reason_task="search_exhausted"),
                 dict(state="found", reason_task="fresh_matching_observation", target_id="object-0007"),
                 dict(state="cancelled", reason_task="user_cancelled"),
                 dict(state="navigating", reason_task="reobserve", target_id="object-0007"),
                 dict(state="observing", reason_task="fresh_visual_evidence_required"),
                 dict(state="ambiguous", reason_task="candidate_selection_required",
                      candidates=[{"id": "object-0007", "label": "bottle", "position_m": [1, 2, 0]},
                                  {"id": "object-0008", "label": "bottle", "position_m": [3, 2, 0]}]),
                 dict(state=None)]
        for kw in cases:
            with self.subTest(state=kw.get("state")):
                g = self.group(**kw)
                self.assertEqual(json.dumps(self.gp.compose_task(g, "b:1")), json.dumps(old.compose_task(g, "b:1")))

    def test_old_bridge_degrades_without_raising(self):
        old = load_old("ui/geomem/bridge/gmb_proto.py", "selsem_old_gmb_proto")
        counts = {}
        t = old.compose_task(self.group(state="needs_manual_approach", reason_task="navigation_server_unavailable",
                                        target=TARGET), "b:1", counts)
        self.assertEqual((t["state"], counts.get("task_state_unknown")), ("idle", 1))


# ---------------------------------------------------------------- GeoMem page (node)

NODE_CHECK = r"""
const [newPath, oldPath] = process.argv.slice(2);
const n = await import(newPath), o = await import(oldPath);
const base = {type:'snapshot', version:2, seq:1, serverTime:null, source:'live', link:{state:'up'},
  map:{frame:'map', units:'m', placeholder:true}, robot:null, memory:null, objects:[], path:[]};
const task = (extra) => ({id:'b:1', state:'failed', reasonCode:'navigation_server_unavailable', message:'m', targetId:'object-0008', ageMs:1, candidates:[], ...extra});
const out = {};
for (const [k, t] of Object.entries({
  manual: task({outcome:'needs_manual_approach', targetPosition:[4.97,-0.8,0.12]}),
  unable: task({outcome:'unable_to_confirm', reasonCode:'no_observation_pose_planned'}),
  plain: task({}),
  junkOutcome: task({outcome:'warp_drive', targetPosition:'here'}),
  junkPos: task({outcome:'needs_manual_approach', targetPosition:[1, 'x', 2]}),
})) {
  const s = {...base, task:t};
  let newOk = true, oldOk = true;
  try { n.validateSnapshot(s); } catch (e) { newOk = String(e); }
  try { o.validateSnapshot(s); } catch (e) { oldOk = String(e); }
  out[k] = {newOk, oldOk, display:n.taskDisplayState(t), label:n.stateLabel(n.taskDisplayState(t)),
            pos:n.manualTargetPosition(t), terminal:n.TERMINAL.has(t.state), active:n.activeTask(t)};
}
out.labels = {failed:n.stateLabel('failed'), idle:n.stateLabel(n.taskDisplayState(null))};
console.log(JSON.stringify(out));
"""


class PageTests(unittest.TestCase):
    def test_page_model_accepts_outcome_and_old_page_does_not_reject(self):
        node = shutil.which("node")
        if node is None:
            raise unittest.SkipTest("node not on PATH")
        d = tempfile.mkdtemp(prefix="selsem-js-")
        new = os.path.join(d, "model_new.mjs")
        old = os.path.join(d, "model_old.mjs")
        shutil.copy(os.path.join(ROOT, "ui", "geomem", "app", "src", "model.js"), new)
        with open(old, "w", encoding="utf-8") as f:
            f.write(old_source("ui/geomem/app/src/model.js"))
        script = os.path.join(d, "check.mjs")
        with open(script, "w") as f:
            f.write(NODE_CHECK)
        r = subprocess.run([node, script, new, old], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = json.loads(r.stdout)
        for k in ("manual", "unable", "plain", "junkOutcome", "junkPos"):
            with self.subTest(case=k):
                self.assertIs(out[k]["newOk"], True)
                self.assertIs(out[k]["oldOk"], True)        # an old page never rejects these frames
                self.assertTrue(out[k]["terminal"])
                self.assertFalse(out[k]["active"])
        self.assertEqual((out["manual"]["display"], out["manual"]["label"], out["manual"]["pos"]),
                         ("needs_manual_approach", "需人工前往", [4.97, -0.8, 0.12]))
        self.assertEqual((out["unable"]["display"], out["unable"]["label"]), ("unable_to_confirm", "无法确认"))
        self.assertEqual((out["plain"]["display"], out["plain"]["label"]), ("failed", "任务失败"))
        self.assertEqual((out["junkOutcome"]["display"], out["junkOutcome"]["pos"]), ("failed", None))
        self.assertEqual((out["junkPos"]["display"], out["junkPos"]["pos"]), ("needs_manual_approach", None))
        self.assertEqual(out["labels"], {"failed": "任务失败", "idle": "待命"})

    def test_node_check_passes_on_changed_scripts(self):
        node = shutil.which("node")
        if node is None:
            raise unittest.SkipTest("node not on PATH")
        for rel in ("model.js", "app.js"):
            with self.subTest(file=rel):
                r = subprocess.run([node, "--check", os.path.join(ROOT, "ui", "geomem", "app", "src", rel)],
                                   capture_output=True, text=True, timeout=60)
                self.assertEqual(r.returncode, 0, r.stderr)


# ---------------------------------------------------------------- --pose odom /map

K = np.array([[517.1719, 0, 324.2197], [0, 517.1171, 240.6476], [0, 0, 1]])
_q = (-0.600676, 0.606199, -0.366895, 0.370269)          # mount_tf.log base_link -> camera_color_optical_frame


def _base_camera():
    qx, qy, qz, qw = _q
    R = np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                  [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                  [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, [0.1, 0.0, 0.379893]
    return T


# test_select_result_dig_20261016.CASES (the 10-16 run): position, remembered view pose, robot xy, poses on 8 m
CASES = {
    "object-0008": ([4.972826880230857, -0.8017907685967884, 0.12259881929439997],
                    (5.2982, 0.2101, 2 * math.atan2(-0.452305, 0.891864)), (-0.66, 0.34), 0),
    "object-0007": ([3.529854498637705, 0.8895550227752926, 0.09921151867301897],
                    (1.4714, -0.0054, 2 * math.atan2(0.002653, 0.999996)), (4.27, -0.20), 22),
}


def plan_on(rt, free_map, target, rejected=None):
    pos, view, robot, _ = CASES[target]
    pos = np.array(pos)
    cand = {"position_m": pos.tolist(),
            "bounds_m": [(pos - [0.04, 0.04, pos[2]]).tolist(), (pos + [0.04, 0.04, 0.12]).tolist()]}
    kw = {} if rejected is None else {"rejected": rejected}
    return rt.observation_poses(cand, free_map, K, (640, 480), _base_camera(), robot,
                                preferred_poses=(rt.PoseGoal(*view),), **kw)


def free_map_from(rt, bp, map_m):
    w, h, ox, oy, data = bp.free_grid(map_m, 0.05)
    return rt.FreeMap(np.asarray(data).reshape(h, w), 0.05, ox, oy)


def _load_bench_pose():
    spec = importlib.util.spec_from_file_location("selsem_bench_pose",
                                                  os.path.join(ROOT, "tools", "car_run", "bench_pose.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class OdomMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from host import ros_runtime
        except Exception as exc:
            raise unittest.SkipTest("host.ros_runtime not importable: %s" % exc)
        cls.rt = ros_runtime
        cls.bp = _load_bench_pose()

    def test_20m_stand_in_plans_object_0008(self):
        old8 = free_map_from(self.rt, self.bp, 8)
        new20 = free_map_from(self.rt, self.bp, 20)
        self.assertEqual(new20.cells.shape, (400, 400))
        self.assertEqual((new20.origin_x, new20.origin_y), (-10.0, -10.0))
        rejected = {}
        self.assertEqual(plan_on(self.rt, old8, "object-0008", rejected), [])
        self.assertEqual(rejected["outside_map"], 37)
        self.assertGreater(len(plan_on(self.rt, new20, "object-0008")), 0)
        self.assertGreaterEqual(len(plan_on(self.rt, new20, "object-0007")), 22)

    def test_defaults(self):
        up = open(os.path.join(ROOT, "tools", "car_run", "car_up.sh")).read()
        inner = open(os.path.join(ROOT, "tools", "car_run", "car_inner.sh")).read()
        line = 'if [ "$POSE" = odom ]; then : "${BENCH_MAP_M:=20}"; else : "${BENCH_MAP_M:=8}"; fi'
        self.assertIn(line, up)
        self.assertIn(line.replace('"$POSE"', '"${POSE:-}"'), inner)
        # the bench node's own default stays 8 m (the line-4 bench); its stand-in plan is unchanged
        old_text = old_source("tools/car_run/bench_pose.py")
        d = tempfile.mkdtemp(prefix="selsem-bp-")
        f = os.path.join(d, "bench_pose_old.py")
        with open(f, "w") as out:
            out.write(old_text)
        spec = importlib.util.spec_from_file_location("selsem_bench_pose_old", f)
        old = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(old)
        for args in ((0, 8, 0.05, True), (0, 20, 0.05, False), (5, 8, 0.05, True), (0, 0, 0.05, False)):
            with self.subTest(args=args):
                self.assertEqual(json.dumps(self.bp.plan(*args)), json.dumps(old.plan(*args)))

    def write_map(self, d, rows, maxval=255, negate=0, comment=True, name="m"):
        h, w = len(rows), len(rows[0])
        header = b"P5\n" + (b"# rtabmap-like comment\n" if comment else b"") + b"%d %d\n%d\n" % (w, h, maxval)
        with open(os.path.join(d, name + ".pgm"), "wb") as f:
            f.write(header + bytes(v for row in rows for v in row))
        y = os.path.join(d, name + ".yaml")
        with open(y, "w") as f:
            f.write("image: %s.pgm\nresolution: 0.5\norigin: [-1.0, -2.0, 0.0]\nnegate: %d\n"
                    "occupied_thresh: 0.65\nfree_thresh: 0.196\n" % (name, negate))
        return y

    def test_pgm_rows_flipped_and_trinary(self):
        d = tempfile.mkdtemp(prefix="selsem-map-")
        # image top row (largest y): occupied, unknown, free; bottom row: free, free, occupied
        y = self.write_map(d, [[0, 205, 254], [254, 254, 0]])
        w, h, res, origin, data = self.bp.load_map_yaml(y)
        self.assertEqual((w, h, res, origin), (3, 2, 0.5, [-1.0, -2.0, 0.0]))
        self.assertEqual(data, [0, 0, 100, 100, -1, 0])      # OccupancyGrid row 0 = the bottom image row
        y = self.write_map(d, [[0, 205, 254], [254, 254, 0]], negate=1, comment=False, name="n")
        self.assertEqual(self.bp.load_map_yaml(y)[4], [100, 100, 0, 0, 100, 100])
        p = self.bp.plan(0, 8, 0.05, odom_base=False, map_yaml=y)
        self.assertEqual((p["map"]["stand_in"], p["map"]["source"], p["map"]["cells"]),
                         (False, "export", {"free": 2, "occupied": 4, "unknown": 0}))

    def test_bad_inputs_refused(self):
        d = tempfile.mkdtemp(prefix="selsem-map-")
        with open(os.path.join(d, "x.pgm"), "wb") as f:
            f.write(b"P6\n1 1\n255\n\x00\x00\x00")
        y = os.path.join(d, "x.yaml")
        with open(y, "w") as f:
            f.write("image: x.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
        with self.assertRaises(ValueError):
            self.bp.load_map_yaml(y)
        with open(y, "w") as f:
            f.write("image: x.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nmode: scale\n")
        with self.assertRaises(ValueError):
            self.bp.load_map_yaml(y)

    def test_real_export_replays_the_dig_counts(self):
        if not os.path.isfile(EXPORT):
            raise unittest.SkipTest("run data %s not on this host" % EXPORT)
        w, h, res, (ox, oy, yaw), data = self.bp.load_map_yaml(EXPORT)
        self.assertEqual((w, h, res), (278, 178, 0.05))
        m = self.rt.FreeMap(np.asarray(data).reshape(h, w), res, ox, oy, yaw)
        self.assertEqual(len(plan_on(self.rt, m, "object-0008")), 7)     # SELECT_RESULT_DIG_20261016.md section 3
        self.assertEqual(len(plan_on(self.rt, m, "object-0007")), 12)


if __name__ == "__main__":
    unittest.main()
