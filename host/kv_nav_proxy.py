"""Nano end of the navigation messages of the two-board channel, ROS-free part (CTRLPS2 N2, 2026-11-03).

Design: runs/nationals-prelim-20261102/CTRLPS0/DESIGN.md 2.5 (case A) and 3. host/ros_kv_nav_proxy.py wraps this in
the ROS node (NavigateToPose action server of the same name, /cmd_vel, the G1 route); this file holds what does
not need ROS so the host tests run it without a ROS install:

* ``NavLink``: one thread owns one KvLinkClient (one session). Every 1 / odom_hz it sends the newest odometry
  snapshot as NAV_ODOM; queued NAV_GOAL / NAV_PATH go first. Every answer is a NAV_CMD; the newest is kept with
  its local receive time.
* ``command(now)``: the (v, w) to publish -- the newest NAV_CMD only when it is for the active goal, says VALID
  and arrived within ``cmd_fresh_s`` (DESIGN 2.5: 150 ms), else (0, 0).
* ``sparsify``: the G1 route thinned to one point per 0.25 m (first and last point kept).
* ``LinkWatch`` (CTRLPS6, 2026-11-04): what the action server does when NAV_CMD stops coming while a goal runs.
  Old rule (mode "abort"): 1.0 s without NAV_CMD -> ABORTED link_lost. New default (mode "hold"): the published
  command is already zero (``cmd_fresh_s``, the velocity guard's own timeout unchanged); warn, keep the goal
  active, and while the gap lasts every NAV_ODOM carries NAV_ODOM_HOLD (the core holds: PAUSED, no stuck time);
  the first NAV_ODOM after an answer is back comes without it, so the core replans from the current pose (not
  the old path). Only a gap longer than ``abort_s`` (30 s) ends the goal ABORTED link_lost (warned).
"""

import math
import queue
import threading
import time
from collections import Counter

from host.kv_link import (NAV_GOAL, NAV_ODOM, NAV_ODOM_HOLD, NAV_ODOM_MAP_VALID, NAV_ODOM_ODOM_VALID, NAV_ODOM_PERMIT,
                          NAV_ODOM_TELEOP, NAV_OP_CANCEL, NAV_OP_GOAL, NAV_PATH, NAV_STATE, NAV_TERMINAL,
                          BadExtension, LinkError, encode_nav_goal, encode_nav_odom, encode_nav_path)
from host.fpga_clock import now_ns

CMD_FRESH_S = 0.15        # DESIGN 2.5: no new command for 150 ms -> publish zero
ODOM_HZ = 50.0
PATH_STEP_M = 0.25        # DESIGN 2.3 G1: one route point per 0.25 m
LINK_LOST_S = 1.0         # no NAV_CMD for this long while a goal runs: link lost (old rule: ABORTED at once)
LINK_ABORT_S = 30.0       # CTRLPS6 hold mode: a link lost for this long ends the goal ABORTED link_lost
LINK_MODES = ("hold", "abort")


def sparsify(points, step=PATH_STEP_M):
    pts = [(float(x), float(y)) for x, y in points]
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    for p in pts[1:-1]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= step:
            out.append(p)
    out.append(pts[-1])
    return out


class NavLink:
    def __init__(self, client, *, odom_hz=ODOM_HZ, cmd_fresh_s=CMD_FRESH_S, answer_timeout_s=None,
                 clock=time.monotonic, on_cmd=None, hold_after_s=LINK_LOST_S):
        self.client = client
        self.period = 1.0 / float(odom_hz)
        self.cmd_fresh_s = float(cmd_fresh_s)
        self.answer_timeout_s = float(answer_timeout_s if answer_timeout_s is not None else self.period * 2)
        self.clock = clock
        self.on_cmd = on_cmd          # callable(cmd dict, local receive time) or None (recording)
        # CTRLPS6: no NAV_CMD for longer than this while a goal runs -> NAV_ODOM carries NAV_ODOM_HOLD
        # (None = never, the CTRLPS2 behaviour; the "abort" link mode passes None)
        self.hold_after_s = None if hold_after_s is None else float(hold_after_s)
        self.last_gap = None          # (gap s, local receive time) of the newest NAV_CMD gap > hold_after_s
        self.counts = Counter()
        self._lock = threading.Lock()
        self._odom = None             # (dict, version)
        self._odom_version = 0
        self._sent_version = 0
        self._seq = 0
        self._q = queue.Queue()
        self.cmd = None               # newest NAV_CMD dict
        self.cmd_t = None             # its local receive time
        self.active_goal = 0          # goal id this Nano is driving (0 = none)
        self._goal_seq = 0
        self._path_seq = 0
        self._stop = threading.Event()
        self._thread = None

    # -- inputs (any thread)
    def set_odom(self, *, odom_pose, v=0.0, w=0.0, map_odom=(0.0, 0.0, 0.0), odom_stamp_ns=0, permit=False,
                 teleop=False, map_valid=True, odom_valid=True):
        flags = ((NAV_ODOM_PERMIT if permit else 0) | (NAV_ODOM_TELEOP if teleop else 0)
                 | (NAV_ODOM_MAP_VALID if map_valid else 0) | (NAV_ODOM_ODOM_VALID if odom_valid else 0))
        with self._lock:
            self._odom_version += 1
            self._odom = (dict(odom_pose=tuple(odom_pose), v=v, w=w, map_odom=tuple(map_odom),
                               odom_stamp_ns=int(odom_stamp_ns), flags=flags), self._odom_version)

    def new_goal_id(self):
        # unique per Nano process and never 0: high 32 bits = start time, low = counter
        self._goal_seq += 1
        return ((int(time.time()) & 0xFFFFFFFF) << 32) | self._goal_seq

    def send_goal(self, goal_id, x, y, yaw):
        self.active_goal = goal_id
        self._q.put((NAV_GOAL, encode_nav_goal(NAV_OP_GOAL, goal_id, now_ns(), x, y, yaw), 0.3))

    def send_cancel(self, goal_id):
        self._q.put((NAV_GOAL, encode_nav_goal(NAV_OP_CANCEL, goal_id, now_ns()), 0.3))

    def send_path(self, goal_id, points):
        pts = sparsify(points)
        if not pts:
            return 0
        self._path_seq += 1
        self._q.put((NAV_PATH, encode_nav_path(goal_id, self._path_seq, now_ns(),
                                                     pts if len(pts) <= 1024 else pts[:1023] + pts[-1:]), 0.3))
        return len(pts)

    def end_goal(self):
        self.active_goal = 0

    # -- output
    def command(self, now=None):
        """(v, w, why): the newest command when it may be driven, else (0, 0) and the reason."""
        now = self.clock() if now is None else now
        c, t = self.cmd, self.cmd_t
        if not self.active_goal:
            return 0.0, 0.0, "no_goal"
        if c is None or t is None or now - t > self.cmd_fresh_s:
            return 0.0, 0.0, "stale"
        if c["goal_id"] != self.active_goal:
            return 0.0, 0.0, "other_goal"
        if not c["valid"]:
            return 0.0, 0.0, "not_valid"
        return c["v"], c["w"], "ok"

    def goal_status(self, goal_id):
        """None (no answer for it yet) or the newest NAV_CMD for ``goal_id``."""
        c = self.cmd
        return c if c is not None and c["goal_id"] == goal_id else None

    def terminal(self, cmd):
        return cmd is not None and cmd["state"] in NAV_TERMINAL

    # -- the link thread
    def _exchange(self, mtype, body, timeout, resend=0.0):
        try:
            c = self.client.nav_call(mtype, body, timeout, resend_s=resend)
        except (LinkError, BadExtension, OSError) as exc:
            self.counts["error:%s" % type(exc).__name__] += 1
            return None
        if c is None:
            self.counts["timeout"] += 1
            return None
        t = self.clock()
        if (self.hold_after_s is not None and self.active_goal and self.cmd_t is not None
                and t - self.cmd_t > self.hold_after_s):
            self.last_gap = (t - self.cmd_t, t)
            self.counts["link_gap"] += 1
        self.cmd, self.cmd_t = c, t
        self.counts["cmd"] += 1
        if self.on_cmd is not None:
            self.on_cmd(c, t)
        return c

    def _run(self):
        nxt = self.clock()
        while not self._stop.is_set():
            while True:
                try:
                    mtype, body, timeout = self._q.get_nowait()
                except queue.Empty:
                    break
                self._exchange(mtype, body, timeout, resend=0.05)
                self.counts["sent:%d" % mtype] += 1
            with self._lock:
                odom = self._odom
            if odom is not None and odom[1] != self._sent_version:
                o, ver = odom
                self._seq += 1
                flags = o["flags"]
                if self.holding():
                    flags |= NAV_ODOM_HOLD
                    self.counts["odom_hold"] += 1
                body = encode_nav_odom(seq=self._seq, t_nano_ns=now_ns(), odom_stamp_ns=o["odom_stamp_ns"],
                                       odom_pose=o["odom_pose"], v=o["v"], w=o["w"], map_odom=o["map_odom"],
                                       flags=flags)
                self._sent_version = ver
                self._exchange(NAV_ODOM, body, self.answer_timeout_s)
            nxt += self.period
            delay = nxt - self.clock()
            if delay < 0:
                self.counts["late"] += 1
                nxt = self.clock()
                delay = 0
            self._stop.wait(delay)

    def holding(self, now=None):
        """CTRLPS6: a goal runs and its NAV_CMD stopped for > hold_after_s (the NAV_ODOM hold bit)."""
        if self.hold_after_s is None or not self.active_goal or self.cmd_t is None:
            return False
        return (self.clock() if now is None else now) - self.cmd_t > self.hold_after_s

    def start(self):
        self._thread = threading.Thread(target=self._run, name="kv-nav-link", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)


class LinkWatch:
    """CTRLPS6: the link-lost rule of one goal (ROS-free so the host tests run it). ``check(now)`` is called by
    the action server's wait loop (every 50 ms) after the terminal / cancel checks:

    mode "abort" (CTRLPS2 rule, positive control): NAV_CMD older than ``lost_s`` -> ("aborted", "link_lost").
    mode "hold" (default): older than ``lost_s`` -> warn once (event "link" state "lost"), goal stays active;
    older than ``abort_s`` -> ("aborted", "link_lost") (warned); back -> event "link" state "resumed" with the
    gap and ``on_resume()`` (the proxy asks for a new G1 route at once). Gaps the 50 ms loop did not see while
    they lasted (the whole Nano process stalled) are still warned from ``NavLink.last_gap``.
    """

    def __init__(self, link, *, mode="hold", lost_s=LINK_LOST_S, abort_s=LINK_ABORT_S, warn=None, on_resume=None,
                 clock=None):
        if mode not in LINK_MODES:
            raise ValueError("link mode %r not in %r" % (mode, LINK_MODES))
        self.link, self.mode = link, mode
        self.lost_s, self.abort_s = float(lost_s), float(abort_s)
        self.warn = warn or (lambda **kw: None)
        self.on_resume = on_resume or (lambda: None)
        self.clock = clock or link.clock
        self.lost_t = None            # local time the current loss was noticed (None = link fine)
        self.seen_gap = link.last_gap
        self.events = []              # (kind, gap s) for tests / the result line

    def check(self, now=None):
        now = self.clock() if now is None else now
        t = self.link.cmd_t
        if t is None:
            return None
        gap = now - t
        if gap > self.lost_s:
            if self.mode == "abort":
                return "aborted", "link_lost"
            if self.lost_t is None:
                self.lost_t = now
                self.link.counts["link_lost_hold"] += 1
                self.events.append(("lost", round(gap, 3)))
                self.warn(state="lost", gap_s=round(gap, 3), abort_s=self.abort_s)
            if gap > self.abort_s:
                self.link.counts["link_lost_abort"] += 1
                self.events.append(("abort", round(gap, 3)))
                self.warn(state="abort", gap_s=round(gap, 3), abort_s=self.abort_s)
                return "aborted", "link_lost"
            return None
        g = self.link.last_gap
        if self.lost_t is not None or (g is not None and g is not self.seen_gap):
            self.seen_gap = g
            gap_s = round(g[0], 3) if g is not None else round(now - self.lost_t, 3)
            self.lost_t = None
            self.link.counts["link_resumed"] += 1
            self.events.append(("resumed", gap_s))
            self.warn(state="resumed", gap_s=gap_s, abort_s=self.abort_s)
            self.on_resume()
        return None


__all__ = ["NavLink", "LinkWatch", "sparsify", "CMD_FRESH_S", "LINK_LOST_S", "LINK_ABORT_S", "LINK_MODES",
           "NAV_STATE"]
