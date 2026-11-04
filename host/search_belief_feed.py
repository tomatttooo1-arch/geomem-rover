"""10-26 search heat map for the console (/fpga/search/belief): WHEN the runner publishes the belief, and its shape.

No ROS, numpy only; host/ros_runtime.py calls ``BeliefFeed.tick(task, now)`` from publish_task (every processed frame
and every task change). Display only: nothing here changes the planner's belief, the task or its commands.

Phases (``phase``):
  preview  a remembered target, before any search: the prior "if it is not where it was" (the planner's ``begin`` is
           called on a scratch copy of its state and the state restored, so the real search later starts as before).
           Once per task, as soon as the target row is known (after the selection when several candidates).
  search   the planner has begun (the search runs): at once when a new look is chosen, else at most every
           ``min_period_s`` (1 s) and only when the belief changed (views applied while driving count).
  done     the task reached a terminal state (``result`` = the controller's state word, ``reason`` its reason): the
           last belief, once; nothing more for this task.
  none     a new task without a planner (MVP list, no map / no dense nodes) or the planner dropped: clears the page.

Message (std_msgs/String JSON, latched): {"v": 1, "seq", "task", "phase", "frame": "map", "resolution",
"width", "height", "origin": [x, y, yaw] (lower-left corner of cell (row 0, col 0), nav_msgs/OccupancyGrid),
"encoding": "int8_b64", "cells": base64 of width*height int8 row-major (0..100 relative to the prior's max,
-1 outside the searched area), "mass_left", "next": [x, y, yaw] | null (the look being driven to), "seen": [[x, y,
yaw]...] (looks done), "result", "reason", "views", "build_ms"}. phase "none": width = height = 0, cells "".
"""

import base64
import json
import time

STEP_M = 0.2
MIN_PERIOD_S = 1.0
VERSION = 1


def _target_xy(task):
    """The remembered position of the task's target row, or None (no target, invalid, not finite)."""
    row = getattr(task, "_target_row", None)
    try:
        xy = (float(row["position_m"][0]), float(row["position_m"][1]))
    except (KeyError, TypeError, IndexError, ValueError):
        return None
    return xy if all(v == v and abs(v) != float("inf") for v in xy) else None


def preview_cells(planner, target_xy, step_m=STEP_M):
    """(meta, int8 cells) of the prior for ``target_xy`` without touching the planner's state."""
    saved = (planner.prior, planner.target_xy)
    try:
        planner.begin(target_xy)
        return planner.belief_cells(step_m, relative="prior")
    finally:
        planner.prior, planner.target_xy = saved


def message(seq, task_n, phase, meta=None, cells=None, planner=None, result=None, reason=None, build_ms=None):
    goals = [[g["x"], g["y"], g["yaw"]] for g in planner.goals] if planner is not None and phase != "preview" else []
    nxt = goals[-1] if goals and phase == "search" else None
    seen = goals[:-1] if phase == "search" else goals
    out = {"v": VERSION, "seq": seq, "task": task_n, "phase": phase, "frame": "map", "resolution": STEP_M,
           "width": 0, "height": 0, "origin": [0.0, 0.0, 0.0], "encoding": "int8_b64", "cells": "",
           "mass_left": None, "next": nxt, "seen": seen, "result": result, "reason": reason,
           "views": None if planner is None else planner.views, "build_ms": build_ms}
    if meta is not None:
        out.update(meta)
        out["cells"] = base64.b64encode(cells.tobytes()).decode("ascii")
        out["mass_left"] = round(planner.mass_left(), 4) if phase != "preview" else 1.0
    return out


class BeliefFeed(object):
    """Decides when to publish; ``publish(dict)`` sends one message (the runner: json.dumps -> String)."""

    def __init__(self, publish, min_period_s=MIN_PERIOD_S, step_m=STEP_M, clock=time.perf_counter, on_error=None):
        self.publish, self.min_period_s, self.step_m = publish, min_period_s, step_m
        self.clock, self.on_error = clock, on_error
        self.task = None
        self.task_n = 0
        self.seq = 0
        self.stats = {"sent": 0, "bytes_max": 0, "build_ms_max": 0.0}
        self._reset_task_state()

    def _reset_task_state(self):
        self.preview_sent = self.done = False
        self.planner_seen = False
        self.last_t = None
        self.last_key = None

    def _send(self, phase, planner=None, result=None, reason=None, preview_xy=None):
        t0 = self.clock()
        meta = cells = None
        if planner is not None:
            if phase == "preview":
                meta, cells = preview_cells(planner, preview_xy, self.step_m)
            else:
                meta, cells = planner.belief_cells(self.step_m, relative="prior")
            meta = dict(meta)
        build_ms = round((self.clock() - t0) * 1e3, 2)
        self.seq += 1
        msg = message(self.seq, self.task_n, phase, meta, cells, planner, result, reason, build_ms)
        self.publish(msg)
        self.stats["sent"] += 1
        self.stats["build_ms_max"] = max(self.stats["build_ms_max"], build_ms)
        return msg

    def clear(self, reason="map_reset"):
        """10-26「重置地图」: phase none now (the page drops the heat map) and nothing more for the current task
        (its belief was over the old map). The next task starts afresh."""
        self.done = True
        self.planner_seen = False
        return self._send("none", reason=reason)

    def tick(self, task, now):
        """Called after every task update (``now``: seconds, monotonic). Returns the message sent or None."""
        try:
            return self._tick(task, now)
        except Exception as exc:                 # noqa: BLE001 - display only, never the task
            if self.on_error is not None:
                self.on_error(exc)
            return None

    def _tick(self, task, now):
        if task is not self.task:
            prev = self.task
            if prev is not None and self.planner_seen and not self.done:
                self._send("done", None, result=getattr(prev, "state", None), reason="superseded")
            self.task = task
            self.task_n += 1
            self._reset_task_state()
            if task is None:
                return None
            if getattr(task, "search_planner", None) is None:
                return self._send("none")          # a new task: the page drops the old map
        if task is None or self.done:
            return None
        planner = getattr(task, "search_planner", None)
        terminal = task.state in getattr(task, "TERMINAL", ())
        if planner is None:
            if self.planner_seen:                  # dropped mid-task (planner fault -> MVP list)
                self.done = True
                return self._send("none")
            if terminal:
                self.done = True
            return None
        self.planner_seen = True
        if terminal:
            self.done = True
            return self._send("done", planner if planner.began() else None, result=task.state, reason=task.reason)
        if not planner.began():
            if self.preview_sent:
                return None
            xy = _target_xy(task)
            if xy is None:
                return None
            self.preview_sent = True
            return self._send("preview", planner, preview_xy=xy)
        key = (len(planner.goals), planner.views)
        if key == self.last_key:
            return None
        goal_changed = self.last_key is None or key[0] != self.last_key[0]
        if not goal_changed and self.last_t is not None and now - self.last_t < self.min_period_s:
            return None
        self.last_key, self.last_t = key, now
        return self._send("search", planner)


def dumps(msg):
    return json.dumps(msg, separators=(",", ":"), allow_nan=False)
