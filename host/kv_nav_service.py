"""KV260 end of the navigation messages of the two-board channel (CTRLPS2 N2, 2026-11-03).

Design: runs/nationals-prelim-20261102/CTRLPS0/DESIGN.md sections 2.1, 2.5, 3. kv_server --nav soft|pl builds a
``NavService`` around a navigation core and hands it to KvLinkServer(nav=...). The server's reader calls
``handle`` for every NAV_GOAL / NAV_ODOM / NAV_PATH and sends back the NAV_CMD body it returns; a 20 Hz thread
here calls ``core.step`` and keeps the newest command. The KV only answers (DESIGN 3): no message is pushed.

The core (CTRLPS1 N1, host.kv_nav) is used through this duck-typed interface only -- the contract of the two
packages; see CTRLPS2/README.md "接口":

    core.set_goal(goal_id, x, y, yaw)   -> (accepted: bool, reason: str)  map frame, replaces any goal
    core.cancel(goal_id)                -> None   the goal ends CANCELED, (v, w) = 0 at once
    core.on_odom(odom)                  -> None   decode_nav_odom dict (odom->base_link, v, w, map->odom, permit)
    core.on_global_path(goal_id, path_id, points) -> None   G1 route, map frame [(x, y), ...]
    core.step(now_s[, hold=True])       -> dict(v, w, state, reason, goal_id, remaining_m, path_len_m,
                                               recoveries, path_points, need_path)   state / reason: names of
                                               host.kv_link.NAV_STATES / NAV_REASONS
    core.describe()                     -> dict (optional)

CTRLPS6 (2026-11-04) link-lost hold: ``step(now_s, hold=True)`` = hold still like permit off / teleop in hand
(no motion, no stuck time) and replan from the current pose on the next ordinary step. The service asks for it
(a) on every tick while no NAV_ODOM came for > ``odom_hold_s`` (1.0 s, the Nano's link-lost time; 0 = off) and
(b) at once, before answering, for a NAV_ODOM that carries NAV_ODOM_HOLD or that ends such a gap (so the answer
is the held zero, never a command made before the gap, and the core sees the hold even when the next odometry
comes before the next tick).

``MockNavCore`` is a stand-in for host tests only (turn to the next route point, then drive): it is NOT the
planner / tracker of DESIGN 2.2 / 2.4 and is never chosen on the board unless asked for by name.
"""

import math
import threading
import time
from collections import Counter

from host.kv_link import (NAV_CMD_ACCEPTED, NAV_CMD_NEED_PATH, NAV_CMD_VALID, NAV_GOAL, NAV_ODOM,
                          NAV_OP_CANCEL, NAV_OP_GOAL, NAV_PATH, NAV_REASON, NAV_STATE, NAV_TERMINAL,
                          decode_nav_goal, decode_nav_odom, decode_nav_path, encode_nav_cmd)

TICK_HZ = 20.0
ODOM_STALE_S = 0.15        # no NAV_ODOM for this long: the command is not valid (DESIGN 2.5 "150 ms")
ODOM_HOLD_S = 1.0          # CTRLPS6: no NAV_ODOM for this long while a goal runs: the core holds (see above)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    """Pose a (x, y, yaw) then pose b in a's frame."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by, wrap(ath + bth))


class NavService:
    def __init__(self, core, *, tick_hz=TICK_HZ, odom_stale_s=ODOM_STALE_S, odom_hold_s=ODOM_HOLD_S,
                 clock=time.monotonic):
        self.core = core
        self.tick_s = 1.0 / float(tick_hz)
        self.odom_stale_s = float(odom_stale_s)
        self.odom_hold_s = float(odom_hold_s or 0.0)
        self.clock = clock
        self.counts = Counter()
        self._lock = threading.Lock()
        self._goal_id = 0
        self._accepted = False
        self._last_odom_t = None
        self._cmd = None             # (step dict, monotonic time it was made)
        self._cmd_seq = 0
        self._stop = threading.Event()
        self._thread = None

    # -- the reader's side (inline, short)
    def handle(self, mtype, payload):
        """NAV_GOAL / NAV_ODOM / NAV_PATH body -> NAV_CMD body; BadExtension on a malformed body."""
        odom_seq = 0
        with self._lock:
            if mtype == NAV_GOAL:
                g = decode_nav_goal(payload)
                if g["op"] == NAV_OP_GOAL:
                    if g["goal_id"] != self._goal_id:          # a resend of the same goal is idempotent
                        ok, why = self.core.set_goal(g["goal_id"], g["x"], g["y"], g["yaw"])
                        self._goal_id, self._accepted = g["goal_id"], bool(ok)
                        self.counts["goal_accepted" if ok else "goal_rejected:%s" % why] += 1
                        self._step_locked()                   # the answer already carries the new goal
                elif g["op"] == NAV_OP_CANCEL and g["goal_id"] == self._goal_id:
                    self.core.cancel(g["goal_id"])
                    self.counts["goal_canceled"] += 1
                    self._step_locked()
                elif g["op"] == NAV_OP_CANCEL:
                    self.counts["cancel_other_goal"] += 1
            elif mtype == NAV_ODOM:
                o = decode_nav_odom(payload)
                odom_seq = o["seq"]
                gap_end = self._odom_gap_locked()
                self.core.on_odom(o)
                self._last_odom_t = self.clock()
                self.counts["odom"] += 1
                if o.get("hold") or gap_end:
                    self.counts["hold_odom" if o.get("hold") else "hold_gap_end"] += 1
                    self._step_locked(hold=True)
            elif mtype == NAV_PATH:
                p = decode_nav_path(payload)
                if p["goal_id"] == self._goal_id:
                    self.core.on_global_path(p["goal_id"], p["path_id"], p["points"])
                    self.counts["path"] += 1
                else:
                    self.counts["path_other_goal"] += 1
            else:
                raise ValueError("not a NAV request: %r" % (mtype,))
            return self._answer_locked(odom_seq)

    def _answer_locked(self, odom_seq):
        now = self.clock()
        if self._cmd is None:
            st, t = {"state": "idle", "reason": "none", "v": 0.0, "w": 0.0}, now
        else:
            st, t = self._cmd
        state = NAV_STATE[st.get("state", "idle")]
        fresh = self._last_odom_t is not None and now - self._last_odom_t <= self.odom_stale_s
        flags = 0
        if self._accepted:
            flags |= NAV_CMD_ACCEPTED
        if (fresh and self._accepted and state not in NAV_TERMINAL and state != NAV_STATE["idle"]
                and st.get("goal_id", self._goal_id) == self._goal_id):
            flags |= NAV_CMD_VALID
        if st.get("need_path"):
            flags |= NAV_CMD_NEED_PATH
        driven = bool(flags & NAV_CMD_VALID)
        return encode_nav_cmd(state=state, reason=NAV_REASON[st.get("reason", "none")], flags=flags,
                              goal_id=self._goal_id, cmd_seq=self._cmd_seq, odom_seq=odom_seq,
                              kv_mono_ns=int(now * 1e9), cmd_age_us=int(max(0.0, now - t) * 1e6),
                              v=st.get("v", 0.0) if driven else 0.0, w=st.get("w", 0.0) if driven else 0.0,
                              remaining_m=st.get("remaining_m", 0.0), path_len_m=st.get("path_len_m", 0.0),
                              recoveries=st.get("recoveries", 0), path_points=st.get("path_points", 0))

    # -- the 20 Hz control tick
    def _odom_gap_locked(self):
        """CTRLPS6: a goal runs and no NAV_ODOM came for > odom_hold_s (0 = off)."""
        return (self.odom_hold_s > 0 and self._accepted and self._last_odom_t is not None
                and self.clock() - self._last_odom_t > self.odom_hold_s)

    def _step_locked(self, hold=False):
        hold = hold or self._odom_gap_locked()
        if hold:
            self.counts["hold_ticks"] += 1
            st = self.core.step(self.clock(), hold=True)
        else:
            st = self.core.step(self.clock())
        self._cmd = (st, self.clock())
        self._cmd_seq += 1

    def _ticker(self):
        nxt = self.clock()
        while not self._stop.is_set():
            with self._lock:
                try:
                    self._step_locked()
                except Exception as exc:          # counted; the Nano sees no valid command -> zero
                    self.counts["step_error:%s" % type(exc).__name__] += 1
                    self._cmd = ({"state": "aborted", "reason": "plan_failed", "goal_id": self._goal_id}, self.clock())
            nxt += self.tick_s
            delay = nxt - self.clock()
            if delay < 0:
                self.counts["tick_overrun"] += 1
                nxt = self.clock()
                delay = 0
            self._stop.wait(delay)

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._ticker, name="kv-nav", daemon=True)
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def describe(self):
        d = {"tick_hz": round(1.0 / self.tick_s, 3), "odom_stale_s": self.odom_stale_s, "odom_hold_s": self.odom_hold_s,
             "goal_id": self._goal_id, "counts": dict(self.counts)}
        if hasattr(self.core, "describe"):
            d["core"] = self.core.describe()
        return d


class MockNavCore:
    """HOST TEST STAND-IN (not DESIGN 2.2/2.4): follows the G1 route points (or the goal when none came yet),
    turn in place when > 0.785 rad off, else drive; xy <= 0.10 m -> final turn, |yaw| <= 0.10 -> SUCCEEDED.
    Asks for a route (need_path) until one for the goal arrived. Stops (v = w = 0) while the permit is off."""

    name = "mock"

    def __init__(self, v_max=0.15, w_max=0.4, w_min=0.12, xy_tol=0.10, yaw_tol=0.10, lookahead=0.3):
        self.v_max, self.w_max, self.w_min = v_max, w_max, w_min
        self.xy_tol, self.yaw_tol, self.lookahead = xy_tol, yaw_tol, lookahead
        self.goal = None              # (goal_id, x, y, yaw) map frame
        self.state, self.reason = "idle", "none"
        self.route = None
        self.odom = None
        self.turning = False

    def set_goal(self, goal_id, x, y, yaw):
        self.goal = (goal_id, x, y, yaw)
        self.state, self.reason, self.route, self.turning = "plan", "none", None, False
        return True, "none"

    def cancel(self, goal_id):
        if self.goal and self.goal[0] == goal_id and self.state not in ("succeeded", "aborted"):
            self.state, self.reason = "canceled", "none"

    def on_odom(self, odom):
        self.odom = odom

    def on_global_path(self, goal_id, path_id, points):
        if self.goal and goal_id == self.goal[0]:
            self.route = [tuple(p) for p in points]

    def _pose_map(self):
        return compose(self.odom["map_odom"], self.odom["odom_pose"])

    def step(self, now_s, hold=False):
        out = {"v": 0.0, "w": 0.0, "goal_id": self.goal[0] if self.goal else 0, "need_path": False,
               "recoveries": 0, "path_points": len(self.route or ())}
        if self.goal is None or self.odom is None or self.state in ("idle", "succeeded", "aborted", "canceled"):
            out.update(state=self.state, reason=self.reason)
            return out
        gid, gx, gy, gyaw = self.goal
        x, y, th = self._pose_map()
        dist = math.hypot(gx - x, gy - y)
        out["remaining_m"] = dist
        if self.route is None:
            out["need_path"] = True
        if not self.odom["permit"] or self.odom["teleop"] or hold:
            out.update(state=self.state, reason=self.reason)        # held: no motion, no state change
            return out
        if dist <= self.xy_tol or self.state == "final_rotate":
            err = wrap(gyaw - th)
            if abs(err) <= self.yaw_tol:
                self.state = "succeeded"
            else:
                self.state = "final_rotate"
                out["w"] = math.copysign(min(self.w_max, max(self.w_min, abs(err))), err)
            out.update(state=self.state, reason=self.reason)
            return out
        tx, ty = gx, gy
        for px, py in (self.route or ()):                 # first route point beyond the lookahead
            if math.hypot(px - x, py - y) >= self.lookahead and math.hypot(px - gx, py - gy) < dist:
                tx, ty = px, py
                break
        err = wrap(math.atan2(ty - y, tx - x) - th)
        if abs(err) > 0.785 or (self.turning and abs(err) > 0.35):
            self.turning = True
            self.state = "rotate_to_path"
            out["w"] = math.copysign(min(self.w_max, max(self.w_min, abs(err))), err)
        else:
            self.turning = False
            self.state = "track"
            out["v"] = min(self.v_max, max(0.05, dist))
            out["w"] = max(-self.w_max, min(self.w_max, 1.5 * err))
        out.update(state=self.state, reason=self.reason)
        return out

    def describe(self):
        return {"name": self.name, "note": "host test stand-in, not the DESIGN 2.2/2.4 tracker"}


def build_core(nav, core_name, *, rolling=None, sim_world=None, grid_params=None):
    """kv_server --nav soft|pl --nav-core kv_nav|mock -> a core, or SystemExit with the reason (never a
    silent fallback). kv_nav = CTRLPS1's host.kv_nav.make_core(field=soft|pl, grid_source=rolling).
    ``sim_world`` (CTRLPS3, host tests only: kv_server --nav-sim-world) replaces the rolling map by
    host.kv_nav.SimWorldGrid(path). ``grid_params`` (CTRLPS4): the KV-side window layer -- swept_clear_s /
    front_seed_m / front_seed_deg, None entries = the car defaults (host.kv_nav.grid_params_from)."""
    if core_name == "mock":
        return MockNavCore()
    try:
        from host import kv_nav
    except ImportError as exc:
        raise SystemExit("--nav %s: host.kv_nav (CTRLPS1 N1) is not on this tree (%s); "
                         "--nav-core mock is the host test stand-in. Not starting." % (nav, exc))
    make = getattr(kv_nav, "make_core", None)
    if make is None:
        raise SystemExit("--nav %s: host.kv_nav has no make_core(field=, grid_source=) "
                         "(CTRLPS2/README.md interface). Not starting." % nav)
    gp = kv_nav.grid_params_from(grid_params) if hasattr(kv_nav, "grid_params_from") else None
    if sim_world:
        return make(field=nav, grid_source=kv_nav.SimWorldGrid(sim_world, grid_params=gp))
    return make(field=nav, grid_source=rolling, **({"grid_params": gp} if gp is not None else {}))
