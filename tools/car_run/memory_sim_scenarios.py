#!/usr/bin/env python3
"""Host whole-flow memory scenarios (2026-10-27, memory candidate: existence score, keep-number-on-move, geometry v2).

    python3 tools/car_run/memory_sim_scenarios.py --repo <tree whose car_up runs> --label v1 [--geometry-v2]
        [--scenarios D,A,B,C1,C2,C3,E] [--out-dir DIR]

One stack (car_up.sh sim --yolo-backend ort, CAR_MAPPING=1, CAR_SIM_NAV_PROFILE=car, WALL_X_M 2.5), then the
scenarios in order on the same stack. The bottle (Gazebo model bottle_prop, static) is removed / moved with the
world's UserCommands services (/world/fpga_room/set_pose; "remove" = set_pose 3 m under the floor, so it can be put
back), called inside the Gazebo container. Times are SIMULATION seconds (the runtime's event clock and the frame
acquisition stamps are /clock): the bottle change is stamped with the world's sim_time read right after the
service call. Output OUT/<label>/{flow.log, scenarios.json, run/...}.
World (room_carmount_d2.sdf): bottle (1.25,-0.45), cup (1.25,0), book (1.25,0.45), table x 1.4..2.2 y -0.8..0.8.
Only the bottle is detected reliably by the CPU ort detector in this world; the depth is noiseless.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

WORLD = "fpga_room"
P0 = (1.25, -0.45)          # the bottle's place in the world file
P1 = (1.25, -1.45)          # 1 m away along -y (nothing behind it until the wall at x 3)
HIDE = (-2.5, 2.5, -3.0)    # "removed": under the floor
# viewpoints (x, y, yaw deg), checked with --scenarios PROBE (10-27: the ort detector misses the bottle from some
# views, e.g. 0/62 frames from (0.27,-1.36,-24 deg) with the bottle 1 m ahead at +20 deg; the views below saw it in
# every frame)
V_BOTH = (-0.2, -0.95, 0.0)    # P0 at about +17 deg, P1 at about -17 deg, 1.5 m (PROBE: both seen in every frame; nearer views flip between 0 and 100 % on a few-degree change -- the ort detector is marginal on this render)
V_P0 = (0.6, -0.45, 0.0)       # P0 0.65 m ahead; P1 57 deg off-axis (out of the 90 deg view)
V_P1 = (0.6, -1.45, 0.0)       # P1 0.65 m ahead; P0 57 deg off-axis
V_A = (0.25, -0.45, 0.0)       # P0 1.0 m ahead (scenarios A, B)
V_AWAY = (-0.5, 0.5, 180.0)    # facing the far wall: neither place in view
TERMINAL = ("found", "not_found", "unable_to_confirm", "failed", "cancelled", "needs_manual_approach")
# the mapping walk of e2e_find_flow.py (from (0,0,0)): (forward_m, turn_deg)
WALK = ((1.0, 90.0), (0.0, -90.0), (2.0, -90.0), (0.0, 90.0), (1.0, 90.0), (0.0, -90.0))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Run:
    def __init__(self, a):
        self.a = a
        self.out = os.path.join(a.out_dir, a.label)
        os.makedirs(self.out, exist_ok=True)
        self.run = os.path.join(self.out, "run-%s" % time.strftime("%H%M%S"))
        self.logf = open(os.path.join(self.out, "flow.log"), "a", encoding="utf-8")
        self.res = {"label": a.label, "repo": a.repo, "geometry_v2": a.geometry_v2, "run": self.run,
                    "head": subprocess.run(["git", "-C", a.repo, "rev-parse", "HEAD"], stdout=subprocess.PIPE,
                                           universal_newlines=True).stdout.strip(), "scenarios": {}}
        self.gz = None
        self.bottle_at = P0

    # ---- plumbing
    def say(self, *x):
        line = "%s %s" % (time.strftime("%H:%M:%S"), " ".join(str(v) for v in x))
        print(line, flush=True)
        self.logf.write(line + "\n"); self.logf.flush()

    def sh(self, argv, env=None, timeout=600):
        t = time.time()
        p = subprocess.run(argv, cwd=self.a.repo, env=dict(os.environ, **(env or {})), stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, universal_newlines=True, timeout=timeout)
        self.say("$", " ".join(argv[:5]), "rc=%d" % p.returncode, "%.1fs" % (time.time() - t))
        return p

    def pc(self, *cmd, timeout=120):
        return self.sh(["bash", "tools/car_run/car_pc.sh"] + list(cmd), env={"CAR_RUN": self.run}, timeout=timeout)

    def gz_exec(self, cmd, timeout=60):
        full = "source /opt/ros/humble/setup.bash >/dev/null 2>&1; " + cmd
        p = subprocess.run(["docker", "exec", self.gz, "bash", "-c", full], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, universal_newlines=True, timeout=timeout)
        return p.stdout

    def sim_time(self):
        txt = self.gz_exec("timeout 20 ign topic -e -n 1 -t /world/%s/stats" % WORLD)
        m = re.search(r"sim_time\s*\{\s*sec:\s*(\d+)(?:\s*nsec:\s*(\d+))?", txt)
        return None if not m else int(m.group(1)) + int(m.group(2) or 0) / 1e9

    def robot_pose(self):
        txt = self.gz_exec("timeout 20 ign topic -e -n 1 -t /world/%s/pose/info" % WORLD)
        i = txt.find('name: "fpga_robot"')
        if i < 0:
            return None
        blk = txt[i:i + 600]
        def num(key, s):
            m = re.search(key + r":\s*(-?[\d.e+-]+)", s)
            return float(m.group(1)) if m else 0.0
        pos = blk[blk.find("position"):]
        ori = blk[blk.find("orientation"):]
        x, y = num("x", pos), num("y", pos)
        qz, qw = num("z", ori), num("w", ori)
        if "w:" not in ori:
            qw = 1.0
        return x, y, 2 * math.atan2(qz, qw)

    def set_bottle(self, xyz):
        x, y = xyz[0], xyz[1]
        z = xyz[2] if len(xyz) > 2 else 0.0
        req = 'name: "bottle_prop", position: {x: %g, y: %g, z: %g}, orientation: {w: 1}' % (x, y, z)
        out = self.gz_exec("ign service -s /world/%s/set_pose --reqtype ignition.msgs.Pose "
                           "--reptype ignition.msgs.Boolean --timeout 5000 --req '%s'" % (WORLD, req))
        st = self.sim_time()
        ok = "data: true" in out
        self.say("bottle ->", (x, y, z), "ok" if ok else "FAILED " + out.strip()[-200:], "sim_t=%s" % st)
        self.bottle_at = (x, y) if z > -1 else None
        return st, ok

    def rows(self):
        path = os.path.join(self.run, "runtime.jsonl")
        out = []
        try:
            with open(path, encoding="utf-8") as f:
                for line in f:
                    if line.endswith("\n") and line.strip():
                        try:
                            out.append(json.loads(line))
                        except ValueError:
                            pass
        except OSError:
            pass
        return out

    def last_time(self):
        for r in reversed(self.rows()):
            if r.get("event") == "frame_processed":
                return r.get("time")
        return None

    def goto_rel(self, fwd, turn_deg):
        n0 = len(self.rows())
        self.pc("goto", "--forward-m", "%.3f" % fwd, "--turn-deg", "%.1f" % turn_deg, timeout=60)
        end = time.time() + 180
        while time.time() < end:
            for r in self.rows()[n0:]:
                if r.get("event") in ("goto_result", "goto_rejected"):
                    self.say("goto", round(fwd, 2), round(turn_deg, 1), "->", r.get("event"), r.get("succeeded"),
                             r.get("reason") or "")
                    return r
            time.sleep(1.0)
        self.say("goto", fwd, turn_deg, "-> no result in 180 s")
        return None

    def goto_abs(self, x, y, yaw_deg):
        """Drive to (x, y) then turn to yaw_deg, with relative gotos from the Gazebo pose."""
        for _ in range(3):
            p = self.robot_pose()
            if p is None:
                self.say("no robot pose"); return False
            dx, dy = x - p[0], y - p[1]
            d = math.hypot(dx, dy)
            if d < 0.12:
                break
            h = math.atan2(dy, dx)
            self.goto_rel(min(d, 2.0), math.degrees(wrap(h - p[2])))
        p = self.robot_pose()
        turn = math.degrees(wrap(math.radians(yaw_deg) - p[2]))
        if abs(turn) > 4:
            self.goto_rel(0.0, turn)
        p = self.robot_pose()
        self.say("pose now", tuple(round(v, 3) for v in p), "wanted", (x, y, yaw_deg))
        return p

    def wait_s(self, s):
        time.sleep(s)

    # ---- memory views
    @staticmethod
    def bottles(row):
        return [t for t in row.get("tracks") or [] if t.get("label") == "bottle"]

    def timeline(self, t_from, t_to=None):
        """Per frame (time >= t_from): bottle tracks (id, number, state, reason, position, existence, ev word)."""
        out = []
        for r in self.rows():
            if r.get("event") != "frame_processed":
                continue
            t = r.get("time")
            if t is None or t < t_from or (t_to is not None and t > t_to):
                continue
            ev = r.get("track_ev") or {}
            out.append({"t": round(t, 3), "acq": r.get("acquisition_timestamp"),
                        "det_bottle": sum(1 for d in r.get("detections") or [] if d.get("label") == "bottle"),
                        "bottles": [{"id": b.get("id"), "n": b.get("number"), "s": b.get("state"),
                                     "r": b.get("reason"),
                                     "p": [round(v, 2) for v in (b.get("position_m") or [])[:2]],
                                     "ev": ev.get(b.get("id"))} for b in self.bottles(r)]})
        return out

    @staticmethod
    def compress(tl):
        """Only frames where some bottle track's (id, state, reason, ex) changed."""
        out, last = [], None
        for f in tl:
            key = tuple((b["id"], b["s"], b["r"], (b["ev"] or [None, None, None])[2]) for b in f["bottles"])
            if key != last:
                out.append(f)
                last = key
        return out

    def other_events(self, t_from, kinds=("memory_merged", "geometry_see_through", "geometry_version",
                                           "geometry_version_mismatch")):
        return [r for r in self.rows() if r.get("event") in kinds and (r.get("time") or 0) >= t_from]

    def invalid_count(self, t_from, t_to=None):
        """Transitions into position_invalid per track (any label) in [t_from, t_to]."""
        prev, n, rows = {}, [], self.rows()
        for r in rows:
            if r.get("event") != "frame_processed":
                continue
            t = r.get("time")
            for tr in r.get("tracks") or []:
                s = tr.get("state")
                if t is not None and t >= t_from and (t_to is None or t <= t_to) \
                        and s == "position_invalid" and prev.get(tr.get("id")) != "position_invalid":
                    n.append({"t": t, "id": tr.get("id"), "label": tr.get("label"), "reason": tr.get("reason")})
                prev[tr.get("id")] = s
        return n

    def wait_state(self, tid_pred, timeout_s):
        """Poll until a frame_processed after now satisfies tid_pred(row); returns the row or None."""
        start = len(self.rows())
        end = time.time() + timeout_s
        while time.time() < end:
            for r in self.rows()[start:]:
                if r.get("event") == "frame_processed" and tid_pred(r):
                    return r
            time.sleep(1.0)
        return None

    def bottle_tracks_now(self):
        for r in reversed(self.rows()):
            if r.get("event") == "frame_processed":
                return self.bottles(r)
        return []

    def live_bottle(self):
        bs = [b for b in self.bottle_tracks_now() if b.get("state") != "position_invalid"]
        return bs

    def first_invalid_after(self, ids, t0):
        for r in self.rows():
            if r.get("event") != "frame_processed" or (r.get("time") or 0) < t0:
                continue
            for b in self.bottles(r):
                if b.get("id") in ids and b.get("state") == "position_invalid":
                    return r
        return None

    # ---- stack
    def up(self):
        env = {"CAR_MAPPING": "1", "CAR_SIM_NAV_PROFILE": "car", "WALL_X_M": "2.5",
               "CAR_GEOMETRY_V2": "1" if self.a.geometry_v2 else "0"}
        p = self.sh(["bash", "tools/car_run/car_up.sh", "sim", "--yolo-backend", "ort", "--run-dir", self.run],
                    env=env, timeout=1500)
        self.res["up_rc"] = p.returncode
        if p.returncode != 0:
            self.res["up_tail"] = p.stdout[-1500:]
            return False
        car = json.load(open(os.path.join(self.run, "car.json")))
        self.gz = car["containers"]["gz"]
        self.say("gz container", self.gz)
        return True

    def down(self):
        d = self.sh(["bash", "tools/car_run/car_down.sh", "--run", self.run], timeout=600)
        self.res["down_rc"] = d.returncode

    def save(self):
        with open(os.path.join(self.out, "scenarios.json"), "w", encoding="utf-8") as f:
            json.dump(self.res, f, ensure_ascii=False, indent=1)

    # ---- scenarios
    def sc_D(self):
        """Mapping walk + mapping stop + ~3 min of laps, nothing moved: count position_invalid transitions."""
        t0 = self.last_time() or 0.0
        walk = [self.goto_rel(f, d) for f, d in WALK]
        p = self.pc("mapping-stop", timeout=240)
        time.sleep(10)
        t_map = self.last_time()
        w0 = time.time()
        laps = 0
        while time.time() - w0 < self.a.d_wall_s:
            for f, d in WALK:
                self.goto_rel(f, d)
            laps += 1
        t1 = self.last_time()
        inv = self.invalid_count(t0, t1)
        tl = self.timeline(t0, t1)
        res = {"t0": t0, "t_mapping_stop": t_map, "t1": t1, "sim_s": None if t1 is None else round(t1 - t0, 1),
               "wall_s": round(time.time() - w0, 1), "laps_after_mapping": laps,
               "walk_ok": [bool(w and w.get("succeeded")) for w in walk],
               "mapping_stop_tail": p.stdout[-300:], "invalid_transitions": inv,
               "bottle_ids": sorted({b["id"] for f in tl for b in f["bottles"]}),
               "timeline_changes": self.compress(tl)}
        self.say("D: invalid transitions", len(inv), "bottle ids", res["bottle_ids"])
        return res

    def settle_bottle(self, timeout_s=90):
        r = self.wait_state(lambda row: any(b.get("state") == "current_confirmed" and b.get("last_seen_frame")
                                            == row.get("frame_id") for b in self.bottles(row)), timeout_s)
        return r

    def measure_invalid(self, ids, t_change, timeout_s):
        r = None
        end = time.time() + timeout_s
        while time.time() < end and r is None:
            r = self.first_invalid_after(ids, t_change)
            if r is None:
                time.sleep(1.0)
        return r

    def sc_A(self):
        """Parked ~1 m in front of the bottle, memory built, the bottle removed."""
        self.set_bottle(P0)
        self.goto_abs(*V_A)
        ok = self.settle_bottle()
        time.sleep(self.a.dwell_s)
        before = self.live_bottle()
        ids = [b["id"] for b in before]
        t_rm, _ = self.set_bottle(HIDE)
        r = self.measure_invalid(ids, t_rm, self.a.inv_wait_s)
        t_end = self.last_time()
        res = {"pose": self.robot_pose(), "bottle_seen_before": bool(ok), "tracks_before": before,
               "t_remove_sim": t_rm, "invalid_row_time": r and r.get("time"),
               "invalid_row_acq": r and r.get("acquisition_timestamp"),
               "latency_s": None if r is None else round(r["time"] - t_rm, 2),
               "latency_acq_s": None if r is None or r.get("acquisition_timestamp") is None
               else round(r["acquisition_timestamp"] - t_rm, 2),
               "timeline_changes": self.compress(self.timeline(t_rm - 3, t_end))}
        self.say("A: latency", res["latency_s"], "acq", res["latency_acq_s"])
        return res

    def restore_and_confirm(self, place, view):
        self.set_bottle(place)
        self.goto_abs(*view)
        r = self.settle_bottle()
        time.sleep(self.a.dwell_s)
        return r

    def sc_B(self):
        """Memory built, drive away (bottle out of view), remove it, come back: in-view -> position_invalid."""
        view = V_A
        self.restore_and_confirm(P0, view)
        ids = [b["id"] for b in self.live_bottle()]
        self.goto_abs(*V_AWAY)
        t_rm, _ = self.set_bottle(HIDE)
        time.sleep(5)
        t_back0 = self.last_time()
        self.goto_abs(*view)
        t_arrive = self.last_time()
        r = self.measure_invalid(ids, t_rm, self.a.inv_wait_s)
        t_end = self.last_time()
        tl = self.timeline(t_rm, t_end)
        # first frame (after the return started) where the bottle track had a word other than out_of_view
        t_inview = None
        for f in tl:
            if f["t"] < (t_back0 or 0):
                continue
            for b in f["bottles"]:
                if b["id"] in ids and b["ev"] and b["ev"][0] not in ("v", "-"):
                    t_inview = f["t"]; break
            if t_inview is not None:
                break
        res = {"ids": ids, "t_remove_sim": t_rm, "t_return_start": t_back0, "t_arrive": t_arrive,
               "t_first_in_view_word": t_inview, "invalid_row_time": r and r.get("time"),
               "latency_from_in_view_s": None if r is None or t_inview is None else round(r["time"] - t_inview, 2),
               "latency_from_arrive_s": None if r is None or t_arrive is None else round(r["time"] - t_arrive, 2),
               "timeline_changes": self.compress(tl)}
        self.say("B: from in-view", res["latency_from_in_view_s"], "from arrive", res["latency_from_arrive_s"])
        return res

    def identity_check(self, t_from, ids_before):
        rows_after = self.bottle_tracks_now()
        return {"ids_before": ids_before, "tracks_after": rows_after,
                "n_bottle_tracks_after": len(rows_after),
                "n_live_after": sum(1 for b in rows_after if b.get("state") != "position_invalid"),
                "merged_events": self.other_events(t_from, ("memory_merged",)),
                "kept_id": any(b.get("id") in ids_before and b.get("state") != "position_invalid"
                               and math.hypot(b["position_m"][0] - self.bottle_at[0],
                                              b["position_m"][1] - self.bottle_at[1]) < 0.3
                               for b in rows_after if self.bottle_at)}

    def sc_C1(self):
        """Watching both places: the bottle moved P0 -> P1 in view."""
        view = V_BOTH
        self.restore_and_confirm(P0, view)
        before = self.live_bottle()
        ids = [b["id"] for b in before]
        t_mv, _ = self.set_bottle(P1)
        time.sleep(self.a.move_wait_s)
        t_end = self.last_time()
        res = {"before": before, "t_move_sim": t_mv, **self.identity_check(t_mv, ids),
               "timeline_changes": self.compress(self.timeline(t_mv - 2, t_end))}
        self.say("C1: kept_id", res["kept_id"], "tracks", [(b["id"], b.get("number"), b["state"])
                                                          for b in res["tracks_after"]])
        return res

    def sc_C2(self):
        """Not watching: moved P1 -> P0; then the NEW place (P0) is seen first, the OLD (P1) after.
        The car watches the bottle at P1, turns away (180 deg in place), the bottle moves to P0, the car drives to
        V_P0 (heading +90 en route: P0 comes into view, P1 stays out), then back to V_P1 (heading -90 en route: P1
        in view, P0 out)."""
        self.restore_and_confirm(P1, V_P1)
        ids = [b["id"] for b in self.live_bottle()]
        self.goto_abs(V_P1[0], V_P1[1], 180.0)
        t_mv, _ = self.set_bottle(P0)
        self.goto_abs(*V_P0)
        time.sleep(self.a.move_wait_s)
        t_new = self.last_time()
        mid = self.bottle_tracks_now()
        self.goto_abs(*V_P1)
        time.sleep(self.a.move_wait_s)
        t_end = self.last_time()
        res = {"t_move_sim": t_mv, "t_after_new_view": t_new, "tracks_after_new_view": mid,
               **self.identity_check(t_mv, ids),
               "timeline_changes": self.compress(self.timeline(t_mv - 2, t_end))}
        self.say("C2: kept_id", res["kept_id"], "tracks", [(b["id"], b.get("number"), b["state"])
                                                          for b in res["tracks_after"]])
        return res

    def sc_C3(self):
        """Not watching: moved P0 -> P1; then the OLD place (P0) is seen first (turning back), the NEW (P1) after."""
        self.restore_and_confirm(P0, V_P0)
        ids = [b["id"] for b in self.live_bottle()]
        self.goto_abs(V_P0[0], V_P0[1], 180.0)
        t_mv, _ = self.set_bottle(P1)
        self.goto_abs(*V_P0)
        time.sleep(self.a.move_wait_s)
        t_old = self.last_time()
        mid = self.bottle_tracks_now()
        self.goto_abs(*V_P1)
        time.sleep(self.a.move_wait_s)
        t_end = self.last_time()
        res = {"t_move_sim": t_mv, "t_after_old_view": t_old, "tracks_after_old_view": mid,
               **self.identity_check(t_mv, ids),
               "timeline_changes": self.compress(self.timeline(t_mv - 2, t_end))}
        self.say("C3: kept_id", res["kept_id"], "tracks", [(b["id"], b.get("number"), b["state"])
                                                          for b in res["tracks_after"]])
        return res

    def sc_E(self):
        """Not watching: the bottle moved (P1 -> P0 or P0 -> P1), the car parked elsewhere, then find 瓶子."""
        cur = self.bottle_at
        dest = P0 if cur and math.hypot(cur[0] - P1[0], cur[1] - P1[1]) < 0.2 else P1
        self.goto_abs(*V_AWAY)
        t_mv, _ = self.set_bottle(dest)
        ids = [b["id"] for b in self.bottle_tracks_now()]
        n0 = len(self.rows())
        w0 = time.time()
        t0 = self.last_time()
        self.pc("find", "瓶子", timeout=60)
        term, selected = None, False
        while time.time() - w0 < self.a.task_wait_s and term is None:
            for r in self.rows()[n0:]:
                if r.get("event") == "task" and r.get("state") == "ambiguous" and not selected:
                    c = (r.get("candidates") or [None])[0]
                    cid = c.get("id") if isinstance(c, dict) else c
                    if cid is not None:
                        self.pc("select", str(cid)); selected = True
                if r.get("event") == "task" and r.get("state") in TERMINAL and (r.get("time") or 0) >= (t0 or 0):
                    term = r; break
            time.sleep(1.0)
        t_end = self.last_time()
        tasks = [{k: r.get(k) for k in ("time", "state", "reason", "target_id")} for r in self.rows()[n0:]
                 if r.get("event") == "task"]
        dedup = []
        for x in tasks:
            if not dedup or (x["state"], x["reason"], x["target_id"]) != (dedup[-1]["state"], dedup[-1]["reason"],
                                                                         dedup[-1]["target_id"]):
                dedup.append(x)
        res = {"moved_to": dest, "t_move_sim": t_mv, "ids_before": ids, "t_find": t0,
               "terminal": term and {k: term.get(k) for k in ("time", "state", "reason", "target_id")},
               "sim_s": None if term is None or t0 is None else round(term["time"] - t0, 1),
               "wall_s": round(time.time() - w0, 1), "selected": selected, "task_states": dedup,
               "tracks_after": self.bottle_tracks_now(),
               "merged_events": self.other_events(t_mv, ("memory_merged",)),
               "timeline_changes": self.compress(self.timeline(t_mv - 2, t_end))}
        self.say("E:", res["terminal"], res["sim_s"], "s sim")
        if term is None:
            self.pc("cancel")
        time.sleep(15)       # find-near approach
        return res

    def det_rate(self, s=10.0):
        t0 = self.last_time()
        time.sleep(s)
        fr = [r for r in self.rows() if r.get("event") == "frame_processed" and (r.get("time") or 0) > (t0 or 0)]
        return "%d/%d" % (sum(1 for r in fr if any(d.get("label") == "bottle" for d in r.get("detections") or [])),
                          len(fr))

    def sc_MOVECHECK(self):
        """Is a bottle moved IN VIEW detected at its new place (render / detector check for C1)?"""
        out = {}
        self.set_bottle(P0)
        self.goto_abs(*V_BOTH)
        out["p0_before"] = self.det_rate()
        self.set_bottle(P1)
        out["p1_moved_in_view"] = self.det_rate()
        self.set_bottle(P1)
        out["p1_set_again"] = self.det_rate()
        self.goto_abs(V_BOTH[0], V_BOTH[1], 180.0)
        self.goto_abs(*V_BOTH)
        out["p1_after_turn_away_and_back"] = self.det_rate()
        out["pose"] = self.robot_pose()
        self.say("MOVECHECK", out)
        return out

    def sc_PROBE(self):
        """Detection rate of the bottle at candidate places / viewpoints (picks P1 and the views)."""
        out = []
        for place, view in self.a.probe:
            self.set_bottle(place)
            self.goto_abs(*view)
            time.sleep(3)
            t0 = self.last_time()
            time.sleep(10)
            fr = [r for r in self.rows() if r.get("event") == "frame_processed" and (r.get("time") or 0) > t0]
            det = [r for r in fr if any(d.get("label") == "bottle" for d in r.get("detections") or [])]
            row = {"place": place, "view": view, "pose": self.robot_pose(), "frames": len(fr), "with_bottle": len(det),
                   "boxes": [d["bbox"] for d in (det[-1]["detections"] if det else []) if d.get("label") == "bottle"]}
            self.say("PROBE", place, view, "%d/%d" % (len(det), len(fr)), row["boxes"])
            out.append(row)
        return out

    def main(self):
        try:
            if not self.up():
                return 1
            self.res["startup_events"] = self.other_events(0, ("geometry_version",))
            for name in self.a.scenarios.split(","):
                self.say("=== scenario", name)
                try:
                    self.res["scenarios"][name] = getattr(self, "sc_" + name)()
                except Exception as exc:  # noqa: BLE001 - record and go on with the next scenario
                    self.say("scenario", name, "raised", repr(exc))
                    self.res["scenarios"][name] = {"error": repr(exc)}
                self.save()
            self.res["mismatch_events"] = self.other_events(0, ("geometry_version_mismatch",))
            self.res["see_through_events"] = len(self.other_events(0, ("geometry_see_through",)))
            gv = [r.get("geometry_version") for r in self.rows() if r.get("event") == "frame_processed"]
            self.res["frame_geometry_versions"] = {v: gv.count(v) for v in set(gv)}
            gres = [((r.get("geometry") or {}).get("result")) for r in self.rows()
                    if r.get("event") == "frame_processed"]
            self.res["frame_geometry_results"] = {str(v): gres.count(v) for v in set(gres)}
        finally:
            if not self.a.keep_up:
                self.down()
            self.save()
        return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--label", required=True)
    ap.add_argument("--geometry-v2", action="store_true")
    ap.add_argument("--scenarios", default="D,A,B,C1,C2,C3,E")
    ap.add_argument("--out-dir", default="${FPGA_SRC}/runs/memory-sim-20261027-results")
    ap.add_argument("--dwell-s", type=float, default=8.0, help="wall s parked in view before a change")
    ap.add_argument("--inv-wait-s", type=float, default=90.0, help="wall s to wait for position_invalid")
    ap.add_argument("--move-wait-s", type=float, default=25.0, help="wall s parked after a move / a view")
    ap.add_argument("--d-wall-s", type=float, default=240.0, help="wall s of extra laps in D")
    ap.add_argument("--task-wait-s", type=float, default=300.0)
    ap.add_argument("--keep-up", action="store_true")
    ap.add_argument("--probe", default="", help="PROBE pairs: px,py,vx,vy,vyaw;... ")
    a = ap.parse_args(argv)
    a.probe = [((v[0], v[1]), (v[2], v[3], v[4])) for v in
               ([float(x) for x in s.split(",")] for s in a.probe.split(";") if s)]
    return Run(a).main()


if __name__ == "__main__":
    sys.exit(main())
