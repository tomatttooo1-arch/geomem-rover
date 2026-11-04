"""CTRLPS5 lockstep closed loop: the CTRLPS3/4 container run (tests/kv_nav_real/run_real.sh) rebuilt on SIMULATED time,
so host load cannot touch it, plus controllable timing doses. Host tests only; no ROS, no threads, no wall clock.

What is the SAME product code as the container run: libkv_nav (C++ core) through host/kv_nav.KvNavCore, the costmap
window from host/kv_nav.SimWorldGrid(world.json) (room map + the box that appears within 1 m, KvWindowLayer, C++
inflation), the route thinning host/kv_nav_proxy.sparsify (0.25 m).
What is MODELLED (stand-ins, see CTRLPS5/README.md):
  * chassis      tests/kv_nav_mock/sim_chassis.py: 50 Hz unicycle, FIXED dt 0.02 per tick, 200 ms command watchdog;
  * proxy        NAV_ODOM at 50 Hz (TF = chassis pose), /cmd_vel at 20 Hz = the KV's latest command (zero when not
                 VALID: idle / terminal); route request every 1 s, at once (>= 0.3 s since last) while need_path;
  * KV ticker    host/kv_nav_service.NavService: core.step every 50 ms on its own grid;
  * global route Nav2 Humble NavFn (upstream navfn.cpp, unmodified; sim/navfn_route.cpp = NavfnPlanner::makePlan's
                 steps) on the room map inflated with the car values; the empty obstacle layer of the container
                 adds nothing; planner_server itself (and its costmap update timing) is not run;
  * goal client  goals one after another, next goal gap_s after the previous ended (container: ~1 s + client start).
Timing doses (all default 0 = ideal lockstep):
  --step-jitter-ms D   each core step fires U(0, D) ms late (its grid is kept, as NavService's ticker does)
  --step-phase-ms P    the core ticker's grid is shifted by P ms against the 50 Hz chassis / odom grid
  --odom-delay-ms D    the core sees the pose D ms old (link / TF latency)
  --slip P             each chassis tick is missed with probability P (rclpy timer skip under load: fixed dt, so the
                       car covers less ground per second of time; the container showed 3-9 %)
  --gap-s G            pause between goals (shifts goal 2's start phase)
Output: one JSON line per run (per-goal status, time, detour, the remaining before/after the switch that tripped it,
ticks with the car centre in a >= 253 world cell [box present all run, as tests/kv_nav_real/analyze.py]).
Usage: python3 tools/kv_nav/sim/lockstep_loop.py [--params JSON] [doses] [--seed N] [--rows FILE] [--goals ...]
"""
import argparse
import ctypes
import heapq
import json
import math
import random
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from host.kv_nav import CAR_INFLATION, RES, KvNavCore, SimWorldGrid, load_library, load_world  # noqa: E402
from host.kv_nav_proxy import sparsify  # noqa: E402
from tools.b1_costmap.nav2_ref import InflationParams, inflate_exact, master_from_grid  # noqa: E402

US = 1_000_000
TERMINAL = ("succeeded", "aborted", "canceled")


class Router:
    """Global route stand-in (planner_server): Nav2 Humble NavFn (vendored, unmodified; sim/navfn_route.cpp) on the
    room map inflated with the car values (= the sim global costmap: static + inflation 0.45 / 3.0)."""

    def __init__(self, world_json, navfn_lib=None):
        spec = json.loads(Path(world_json).read_text())
        g, self.base = load_world(ROOT / spec["map_yaml"])
        self.cost = inflate_exact(master_from_grid(g), InflationParams(**CAR_INFLATION))
        boxed = g.copy()
        for b in spec.get("boxes", ()):
            x0, x1 = int(math.floor(b["x0"] / RES)) - self.base[0], int(math.floor(b["x1"] / RES)) - self.base[0]
            y0, y1 = int(math.floor(b["y0"] / RES)) - self.base[1], int(math.floor(b["y1"] / RES)) - self.base[1]
            boxed[y0:y1 + 1, x0:x1 + 1] = 100
        self.boxed = inflate_exact(master_from_grid(boxed), InflationParams(**CAR_INFLATION))
        lib = Path(navfn_lib or ROOT / "tools/kv_nav/build/host/libnavfn_route.so")
        self.nf = ctypes.CDLL(str(lib))
        self.nf.navfn_route.restype = ctypes.c_int

    def cell(self, x, y):
        return int(math.floor(x / RES)) - self.base[0], int(math.floor(y / RES)) - self.base[1]

    def route(self, sx, sy, gx, gy):
        """NavfnPlanner::makePlan: worldToMap = round((w - origin) / res), clear the robot cell, NavFn Dijkstra from
        the goal, path robot -> goal, mapToWorld = origin + m * res (no half cell: the planner's own helpers);
        the goal pose itself last (getPlanFromPotential)."""
        org = (self.base[0] * RES, self.base[1] * RES)
        h, w = self.cost.shape
        s = (int(round((sx - org[0]) / RES)), int(round((sy - org[1]) / RES)))
        g = (int(round((gx - org[0]) / RES)), int(round((gy - org[1]) / RES)))
        if not (0 <= s[0] < w and 0 <= s[1] < h and 0 <= g[0] < w and 0 <= g[1] < h):
            return None
        c = self.cost.astype(np.uint8).copy()
        c[s[1], s[0]] = 0
        cap = 4 * max(w, h) * 4
        X, Y = (ctypes.c_float * cap)(), (ctypes.c_float * cap)()
        n = self.nf.navfn_route(np.ascontiguousarray(c).tobytes(), w, h, s[0], s[1], g[0], g[1], X, Y, cap)
        if n <= 0:
            return None
        pts = [(org[0] + X[i] * RES, org[1] + Y[i] * RES) for i in range(n)]
        pts.append((gx, gy))
        return pts


def run(args, router=None, lib=None):
    rng = random.Random(args.seed)
    lib = lib or load_library(args.lib)
    router = router or Router(args.world)
    t_us = 0
    clock = lambda: t_us / US  # noqa: E731
    params = json.loads(args.params) if args.params else {}
    core = KvNavCore(lib, field="soft", grid_source=SimWorldGrid(args.world), params=params, clock=clock)
    # chassis state
    ch = dict(x=0.0, y=0.0, th=0.0, v=0.0, w=0.0, t_cmd=-1.0)
    hist = []                                     # (t_us, x, y, th, v, w) every chassis tick (odom delay lookup)
    last_cmd = {"v": 0.0, "w": 0.0, "state": "idle", "reason": "none", "remaining_m": 0.0, "need_path": False}
    goals = [tuple(float(v) for v in g.split()) for g in args.goals.split(";")]
    gi = -1
    goal_t0 = None
    next_goal_us = int(args.first_goal_s * US)
    goal_id = 0
    route_req_us = -10 * US
    route_pending = None                          # (deliver_us, goal_id, points)
    results = []
    rows = open(args.rows, "w") if args.rows else None
    per_goal = None
    events = []                                   # heap (t_us, order, kind)
    step_us, odom_us, chassis_us, cmd_us, path_us = 50_000, 20_000, 20_000, 50_000, 250_000
    phase = int(args.step_phase_ms * 1000)
    heapq.heappush(events, (0, 0, "chassis"))
    heapq.heappush(events, (1, 1, "odom"))
    heapq.heappush(events, (phase + 2, 2, "step"))
    heapq.heappush(events, (3, 3, "cmd"))
    heapq.heappush(events, (4, 4, "path"))
    step_grid = phase + 2
    end_us = int(args.max_s * US)
    prev_state = None
    while events:
        t_us, order, kind = heapq.heappop(events)
        if t_us > end_us:
            break
        now = t_us / US
        # goal client
        if gi + 1 < len(goals) and goal_t0 is None and t_us >= next_goal_us:
            gi += 1
            goal_id += 1
            gx, gy, gyaw = goals[gi]
            core.set_goal(goal_id, gx, gy, gyaw)
            goal_t0 = now
            per_goal = {"goal_index": gi + 1, "goal": [gx, gy, gyaw], "detour": False, "recover_ticks": 0,
                        "rem_jumps": [], "unsafe": 0, "samples": 0, "path_len_m": 0.0}
            prev_rem = None
            prev_end = None
        if kind == "chassis":
            if rng.random() >= args.slip:       # a missed tick: no integration this period (fixed-dt integrator)
                if now - ch["t_cmd"] > 0.2:
                    ch["v"] = ch["w"] = 0.0
                dt = 0.02
                ox, oy = ch["x"], ch["y"]
                ch["x"] += ch["v"] * math.cos(ch["th"]) * dt
                ch["y"] += ch["v"] * math.sin(ch["th"]) * dt
                th = ch["th"] + ch["w"] * dt
                ch["th"] = math.atan2(math.sin(th), math.cos(th))
                hist.append((t_us, ch["x"], ch["y"], ch["th"], ch["v"], ch["w"]))
                if len(hist) > 500:
                    del hist[:250]
                if per_goal is not None and goal_t0 is not None:
                    per_goal["samples"] += 1
                    per_goal["path_len_m"] += math.hypot(ch["x"] - ox, ch["y"] - oy)
                    cx, cy = router.cell(ch["x"], ch["y"])
                    if int(router.boxed[cy, cx]) >= 253:
                        per_goal["unsafe"] += 1
            heapq.heappush(events, (t_us + chassis_us, 0, "chassis"))
        elif kind == "odom":
            tq = t_us - int(args.odom_delay_ms * 1000)
            p = None
            for h in reversed(hist):
                if h[0] <= tq:
                    p = h
                    break
            if p is None:
                p = (0, 0.0, 0.0, 0.0, 0.0, 0.0)
            core.on_odom({"odom_pose": (p[1], p[2], p[3]), "v": p[4], "w": p[5], "map_odom": (0.0, 0.0, 0.0),
                          "permit": True, "teleop": False, "map_valid": True, "odom_valid": True})
            heapq.heappush(events, (t_us + odom_us, 1, "odom"))
        elif kind == "step":
            pe = core.path_odom(4096) if goal_t0 is not None else []
            pre_end = pe[-1] if pe else None          # the path in force BEFORE this tick (it may switch in it)
            st = core.step(now)
            last_cmd = st
            step_grid += step_us
            late = int(rng.uniform(0, args.step_jitter_ms) * 1000) if args.step_jitter_ms > 0 else 0
            heapq.heappush(events, (max(step_grid + late, t_us + 1), 2, "step"))
            if goal_t0 is not None and st["goal_id"] == goal_id:
                rem = st["remaining_m"]
                if prev_rem is not None and rem - prev_rem > 0.5:
                    gx_, gy_ = per_goal["goal"][0], per_goal["goal"][1]
                    # the switch happened in the PREVIOUS tick (a tick reports the remaining measured before its
                    # replan), so the old path is the one in force before that tick: prev_end
                    ext = math.hypot(gx_ - prev_end[0], gy_ - prev_end[1]) if prev_end else None
                    per_goal["rem_jumps"].append({"t": round(now - goal_t0, 3), "before": round(prev_rem, 3),
                                                  "after": round(rem, 3), "state": st["state"],
                                                  "reason": st["reason"],
                                                  # old path's end -> goal (straight): what the CTRLPS5 rule adds
                                                  "old_end_to_goal": None if ext is None else round(ext, 3)})
                prev_rem = rem
                prev_end = pre_end
                if st["state"] == "recover":
                    per_goal["recover_ticks"] += 1
                if st["reason"] == "detour":
                    per_goal["detour"] = True
                if rows is not None and (st["state"], st["reason"]) != prev_state:
                    rows.write(json.dumps({"t": round(now, 3), "goal": gi + 1, "state": st["state"],
                                           "reason": st["reason"], "rem": round(rem, 3),
                                           "pose": [round(ch["x"], 3), round(ch["y"], 3)]}) + "\n")
                prev_state = (st["state"], st["reason"])
                if st["state"] in TERMINAL:
                    per_goal.update(status=st["state"].upper(), elapsed_s=round(now - goal_t0, 2),
                                    err_xy=round(math.hypot(ch["x"] - per_goal["goal"][0], ch["y"] - per_goal["goal"][1]), 3),
                                    path_len_m=round(per_goal["path_len_m"], 3))
                    results.append(per_goal)
                    per_goal = None
                    goal_t0 = None
                    next_goal_us = t_us + int(args.gap_s * US)
                    if gi + 1 >= len(goals):
                        break
                elif now - goal_t0 > args.goal_timeout_s:
                    per_goal.update(status="TIMEOUT", elapsed_s=round(now - goal_t0, 2))
                    results.append(per_goal)
                    per_goal = None
                    goal_t0 = None
                    next_goal_us = t_us + int(args.gap_s * US)
        elif kind == "cmd":
            driven = goal_t0 is not None and last_cmd.get("state") not in TERMINAL + ("idle",)
            ch["v"], ch["w"] = (last_cmd["v"], last_cmd["w"]) if driven else (0.0, 0.0)
            ch["t_cmd"] = now
            heapq.heappush(events, (t_us + cmd_us, 3, "cmd"))
        elif kind == "path":
            if route_pending is not None and t_us >= route_pending[0]:
                _, gid, pts = route_pending
                route_pending = None
                if gid == goal_id and goal_t0 is not None and pts:
                    core.on_global_path(gid, 0, sparsify(pts))
            if goal_t0 is not None and route_pending is None:
                due = t_us - route_req_us >= US
                urgent = last_cmd.get("need_path") and t_us - route_req_us >= 300_000
                if due or urgent:
                    route_req_us = t_us
                    gx, gy, _ = goals[gi]
                    pts = router.route(ch["x"], ch["y"], gx, gy)
                    route_pending = (t_us + int(args.route_latency_ms * 1000), goal_id, pts)
            heapq.heappush(events, (t_us + (path_us if route_pending is None else
                                            max(1, min(path_us, route_pending[0] - t_us))), 4, "path"))
    if rows is not None:
        rows.close()
    return {"doses": {k: getattr(args, k) for k in ("step_jitter_ms", "step_phase_ms", "odom_delay_ms", "slip",
                                                     "gap_s", "seed")},
            "params": params, "goals": results, "core_counts": core.describe().get("counts")}


def parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default=str(ROOT / "tests/kv_nav_real/world.json"))
    ap.add_argument("--goals", default="1.0 1.2 1.57;2.2 -1.4 0.0;-1.5 -1.4 3.14")
    ap.add_argument("--params", default="", help="core parameters as JSON (KV_NAV_PARAMS form)")
    ap.add_argument("--lib", default=None)
    ap.add_argument("--step-jitter-ms", type=float, default=0.0)
    ap.add_argument("--step-phase-ms", type=float, default=0.0)
    ap.add_argument("--odom-delay-ms", type=float, default=0.0)
    ap.add_argument("--slip", type=float, default=0.0)
    ap.add_argument("--gap-s", type=float, default=1.5)
    ap.add_argument("--first-goal-s", type=float, default=1.0)
    ap.add_argument("--route-latency-ms", type=float, default=30.0)
    ap.add_argument("--goal-timeout-s", type=float, default=150.0)
    ap.add_argument("--max-s", type=float, default=600.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--rows", default=None, help="state-change rows (JSONL)")
    return ap


if __name__ == "__main__":
    print(json.dumps(run(parser().parse_args())))
