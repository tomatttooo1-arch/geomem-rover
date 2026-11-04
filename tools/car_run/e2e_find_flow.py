#!/usr/bin/env python3
"""Host whole-flow simulation (2026-10-26): mapping walk -> mapping stop -> K finds, R rounds, each on a fresh stack.

    python3 tools/car_run/e2e_find_flow.py --rounds 2 --k 5 [--wall-x 2.5] [--sim-nav-profile car|sim]
        [--fault-nav-fail-first N] [--out-dir DIR] [--label NAME] [--targets 瓶子,杯子,书] [--dry-run]

One round:
  car_up.sh sim --yolo-backend ort (CPU inference, no GPU lock) with CAR_MAPPING=1 (the host mapping STAND-IN:
  tools/car_run/mapping_standin.py), WALL_X_M (the stand-in export's wall, default 1.0 = the stand-in's own default),
  CAR_SIM_NAV_PROFILE; a walk of WALK goto steps through the hub (car_pc.sh goto; the host refuses remote control in
  sim); car_pc.sh mapping-stop; RUN/mapping_status.json (state / map_loaded / align: on the host the alignment is NEVER
  computed -- the stand-in has no rtabmap graph -- so only "align has a readable reason, map_loaded true" is checked);
  the object memory (RUN/memory_snapshot.json) against the world's truth (the reference pose is exact: this error is
  detection / geometry error, NOT alignment error); then K x [leave the observation spot (goto turn 180 deg, 1 m),
  find <target>, ambiguous -> select the first candidate, wait for a terminal task state]; car_down.sh.
Output: OUT/<label>/round-<i>/{round.json, events_<j>.jsonl} and OUT/<label>/summary.json + one line
"success a/b (x%)". Failure categories: classify(); "standin_wall" when a failed navigation goal lies within reach of
the stand-in wall (+ inflation) -- a stand-in artefact, not a car defect.
--fault-nav-fail-first N: exported as FPGA_FAULT_NAV_FAIL_FIRST for car_up (whether it reaches the stack container is
car_up's business; not wired here).
"""
import argparse
import collections
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
TERMINAL = ("found", "not_found", "unable_to_confirm", "failed", "cancelled", "needs_manual_approach")
# robot_sim/fpga_sim/worlds/room_carmount_d2.sdf: cup (1.25, 0), book (1.25, 0.45), bottle_prop (1.25, -0.45)
TRUTH = {"bottle": (1.25, -0.45), "cup": (1.25, 0.0), "book": (1.25, 0.45)}
WORD_LABEL = {"瓶子": "bottle", "杯子": "cup", "书": "book"}
# the mapping walk from the start pose (0, 0, yaw 0): (forward_m, turn_deg) -> (0,1,+90) (0,1,0) (0,-1,-90)
# (0,-1,0) (0,0,+90) (0,0,0); the objects at x 1.25 are in view from every stop facing +x
WALK = ((1.0, 90.0), (0.0, -90.0), (2.0, -90.0), (0.0, 90.0), (1.0, 90.0), (0.0, -90.0))
LEAVE = ((1.0, 180.0), (0.8, 90.0), (0.0, 180.0))   # tried in order until one is accepted
WALL_THICK_M = 0.2       # mapping_standin.py: 4 cells x 0.05 m from WALL_X_M
INFLATION_M = 0.45       # the global costmap's inflation radius (handover fact 3)
NAV_LOG_PATTERNS = ("Failed to make progress", "Failed to create plan", "Goal was rejected", "Aborting handle",
                    "failed to get result", "Failed to get result", "occupied", "No valid trajectories",
                    "Planning algorithm", "Reached the goal")
WALL_RE = re.compile(r"\[(\d{9,}\.\d+)\]")


# ---- pure functions (tests/test_e2e_find_flow.py) -------------------------------------------------------------
def approach_running(rows):
    """True when the newest approach_started row has no approach_result after it (10-26 find-near)."""
    running = False
    for r in rows:
        if r.get("event") == "approach_started":
            running = True
        elif r.get("event") == "approach_result":
            running = False
    return running


def read_events(path, start=0):
    """(rows from line index `start` on, next start) of a runtime.jsonl; a torn last line is left for next time."""
    rows, n = [], start
    if not os.path.exists(path):
        return rows, start
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    for i in range(start, len(lines)):
        line = lines[i]
        if not line.endswith("\n"):
            break
        n = i + 1
        if line.strip():
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass
    return rows, n


def terminal(rows):
    """The first terminal task event in rows, or None."""
    for r in rows:
        if r.get("event") == "task" and r.get("state") in TERMINAL:
            return r
    return None


def task_key(row):
    return (row.get("state"), row.get("reason"), row.get("target_id")) if row else None


def last_task(rows):
    for r in reversed(rows):
        if r.get("event") == "task":
            return r
    return None


def find_slice(rows, prev_task):
    """The rows of THIS find: the runner keeps republishing its last task (a finished one too) until the next query,
    so the slice starts at the first task row that differs from the task published before the find (or at the first
    observation_plan / command row). (A new find ending exactly like the previous one at once -- same state, reason
    and target, no navigation -- is not told apart: it reads as no_terminal.)"""
    prev = task_key(prev_task)
    for i, r in enumerate(rows):
        e = r.get("event")
        if e in ("observation_plan", "command") or (e == "task" and task_key(r) != prev):
            return rows[i:]
    # rejections of the query itself are this find's too
    return [r for r in rows if r.get("event") in ("query_rejected", "fault")]


def first_ambiguous(rows):
    for r in rows:
        if r.get("event") == "task" and r.get("state") == "ambiguous":
            return r
    return None


def candidate_id(c):
    return c.get("id") if isinstance(c, dict) else c


def task_summary(rows):
    """What one find did, from its slice of runtime.jsonl."""
    navs = [r["command"] for r in rows if r.get("event") == "command"
            and (r.get("command") or {}).get("kind") == "navigate"]
    results = [r for r in rows if r.get("event") == "navigation_result"]
    started = {r.get("command_id"): r for r in rows if r.get("event") == "navigation_started"}
    failed_goals = []
    for r in results:
        if not r.get("succeeded") and not r.get("cancelled"):
            s = started.get(r.get("command_id"), {})
            dt = (r.get("time", 0) - s.get("time", 0)) if s else None
            failed_goals.append({"command_id": r.get("command_id"), "pose": s.get("pose"), "dt_s": dt})
    term = terminal(rows)
    return {
        "state": term.get("state") if term else None,
        "reason": term.get("reason") if term else None,
        "target_id": term.get("target_id") if term else None,
        "first_navigate_reason": navs[0].get("reason") if navs else None,
        "navigate_reasons": [n.get("reason") for n in navs],
        "n_navigate": len(navs),
        "observation_plan": any(r.get("event") == "observation_plan" for r in rows),
        "ambiguous": first_ambiguous(rows) is not None,
        "query_rejected": [r.get("reason") for r in rows if r.get("event") == "query_rejected"],
        "selection_rejected": [r.get("reason") for r in rows if r.get("event") == "selection_rejected"],
        "navigation_refused": [r.get("reason") for r in rows if r.get("event") == "navigation_refused"],
        "faults": [r.get("reason") for r in rows if r.get("event") == "fault"],
        "nav_succeeded": sum(1 for r in results if r.get("succeeded")),
        "nav_failed_goals": failed_goals,
        "quick_nav_rejects": [g for g in failed_goals if g["dt_s"] is not None and g["dt_s"] < 2.0],
    }


def near_wall(pose, wall_x, room_half=3.0):
    """A goal within the robot-reach of the stand-in wall band [wall_x, wall_x + 0.2) widened by the inflation."""
    if pose is None or wall_x is None or wall_x >= room_half:
        return False
    x = pose.get("x") if isinstance(pose, dict) else pose[0]
    return wall_x - INFLATION_M <= x <= wall_x + WALL_THICK_M + INFLATION_M


def classify(summary, wall_x=None):
    """One category per find: success | standin_wall | <state>:<reason> | no_terminal | query_rejected."""
    if summary.get("state") == "found":
        return "success"
    if summary.get("query_rejected") and summary.get("state") is None:
        return "query_rejected"
    if any(near_wall(g.get("pose"), wall_x) for g in summary.get("nav_failed_goals", [])):
        return "standin_wall"
    if summary.get("state") is None:
        return "no_terminal"
    return "%s:%s" % (summary["state"], summary.get("reason"))


def truth_errors(tracks, truth=TRUTH):
    """Per track of a truth label: xy distance to that label's truth; per label the best (nearest) track."""
    out = {}
    for t in tracks or []:
        label, pos = t.get("label"), t.get("position_m")
        if label not in truth or not pos:
            continue
        tx, ty = truth[label]
        err = math.hypot(pos[0] - tx, pos[1] - ty)
        row = out.setdefault(label, {"tracks": 0, "best_err_m": None, "best_id": None, "states": []})
        row["tracks"] += 1
        row["states"].append(t.get("state"))
        if row["best_err_m"] is None or err < row["best_err_m"]:
            row["best_err_m"], row["best_id"] = round(err, 3), t.get("id")
    return out


def found_error(tracks, target_id, label, truth=TRUTH):
    """xy error of the found track (by id) against the requested label's truth; None when unknown."""
    for t in tracks or []:
        if t.get("id") == target_id and t.get("position_m") and label in truth:
            tx, ty = truth[label]
            return round(math.hypot(t["position_m"][0] - tx, t["position_m"][1] - ty), 3), t.get("label")
    return None, None


def nav_log_issues(lines, t0=None, t1=None):
    """Counts of NAV_LOG_PATTERNS in navigation.log lines whose wall stamp is within [t0, t1] (None = unbounded)."""
    c = collections.Counter()
    for line in lines:
        m = WALL_RE.search(line)
        if m and ((t0 is not None and float(m.group(1)) < t0) or (t1 is not None and float(m.group(1)) > t1)):
            continue
        for p in NAV_LOG_PATTERNS:
            if p in line:
                c[p] += 1
    return dict(c)


def aggregate(tasks):
    """tasks: [{"category": ...}] -> {n, success, rate, categories}."""
    cats = collections.Counter(t["category"] for t in tasks)
    n = len(tasks)
    ok = cats.get("success", 0)
    return {"n": n, "success": ok, "rate": round(ok / n, 3) if n else None, "categories": dict(cats.most_common())}


# ---- the run ----------------------------------------------------------------------------------------------------
class Flow:
    def __init__(self, args, out):
        self.args, self.out = args, out
        self.run = None
        self.offset = 0
        self.log = open(os.path.join(out, "flow.log"), "a", encoding="utf-8")

    def say(self, *a):
        line = "%s %s" % (time.strftime("%H:%M:%S"), " ".join(str(x) for x in a))
        print(line, flush=True)
        self.log.write(line + "\n")
        self.log.flush()

    def sh(self, argv, env=None, timeout=600):
        e = dict(os.environ, **(env or {}))
        t = time.time()
        p = subprocess.run(argv, cwd=REPO, env=e, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True, timeout=timeout)
        self.say("$", " ".join(argv[:4]), "...", "rc=%d" % p.returncode, "%.1fs" % (time.time() - t))
        return p

    def pc(self, *cmd, timeout=240):
        return self.sh(["bash", "tools/car_run/car_pc.sh"] + list(cmd), env={"CAR_RUN": self.run}, timeout=timeout)

    def events(self):
        rows, self.offset = read_events(os.path.join(self.run, "runtime.jsonl"), self.offset)
        return rows

    def mark(self):
        _, self.offset = read_events(os.path.join(self.run, "runtime.jsonl"), self.offset)

    def wait_events(self, pred, timeout_s, acc):
        end = time.time() + timeout_s
        while time.time() < end:
            acc.extend(self.events())
            hit = pred(acc)
            if hit is not None:
                return hit
            time.sleep(1.0)
        return None

    def goto(self, fwd, turn):
        self.mark()
        acc = []
        args = ["goto", "--forward-m", "%g" % fwd, "--turn-deg", "%g" % turn]
        p = self.pc(*args, timeout=60)

        def pred(rows):
            for r in rows:
                if r.get("event") in ("goto_result", "goto_rejected"):
                    return r
            return None
        hit = self.wait_events(pred, 120, acc)
        res = {"forward_m": fwd, "turn_deg": turn, "rc": p.returncode,
               "event": hit.get("event") if hit else None, "succeeded": hit.get("succeeded") if hit else None,
               "reason": hit.get("reason") if hit else None, "pc_tail": p.stdout[-300:]}
        self.say("goto", fwd, turn, "->", res["event"], res["succeeded"], res["reason"] or "")
        return res

    def memory(self):
        try:
            with open(os.path.join(self.run, "memory_snapshot.json"), encoding="utf-8") as f:
                return json.load(f).get("tracks") or []
        except (OSError, ValueError):
            return []

    def map_info(self):
        try:
            stack = json.load(open(os.path.join(self.run, "car.json")))["containers"]["stack"]
        except (OSError, ValueError, KeyError):
            return None
        cmd = ("source /opt/ros/humble/setup.bash; timeout 25 ros2 topic echo --once --qos-durability transient_local "
               "--qos-reliability reliable /map nav_msgs/msg/OccupancyGrid --field info")
        p = subprocess.run(["docker", "exec", stack, "bash", "-c", cmd], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
        return p.stdout[-800:]

    def wait_approach(self, limit_s=30.0):
        """10-26 find-near: a found task drives a final approach goal; goto / find are refused while it runs."""
        end = time.time() + limit_s
        while time.time() < end:
            rows, _ = read_events(os.path.join(self.run, "runtime.jsonl"), 0)
            if not approach_running(rows):
                return
            time.sleep(1.0)
        self.say("approach still running after", limit_s, "s")

    def find(self, j, word, label):
        rec = {"i": j, "word": word, "label": label}
        self.wait_approach()
        leave = None
        for fwd, turn in LEAVE:
            leave = self.goto(fwd, turn)
            if leave["event"] == "goto_result":
                break
        rec["leave"] = leave
        before = {t.get("id"): t.get("state") for t in self.memory() if t.get("label") == label}
        rec["memory_before"] = before
        prev_rows, self.offset = read_events(os.path.join(self.run, "runtime.jsonl"), 0)
        prev_task = last_task(prev_rows)
        rec["prev_task"] = task_key(prev_task)
        start_line = self.offset
        t0 = time.time()
        p = self.pc("find", word, timeout=60)
        rec["find_rc"], rec["find_pc_tail"] = p.returncode, p.stdout[-300:]
        acc = []
        selected = False
        end = t0 + self.args.task_wait_s
        while time.time() < end:
            acc.extend(self.events())
            mine = find_slice(acc, prev_task)
            if terminal(mine) is not None:
                break
            amb = first_ambiguous(mine)
            if amb is not None and not selected:
                cands = amb.get("candidates") or []
                if cands:
                    cid = candidate_id(cands[0])
                    self.say("ambiguous: select", cid, "of", [candidate_id(c) for c in cands])
                    rec["selected"] = cid
                    self.pc("select", str(cid), timeout=60)
                selected = True
            if not any(r.get("event") == "task" for r in mine) \
                    and any(r.get("event") == "query_rejected" for r in acc) and time.time() - t0 > 10:
                break
            time.sleep(1.0)
        t1 = time.time()
        rec["wall_s"] = round(t1 - t0, 1)
        rec["start_line"] = start_line
        with open(os.path.join(self.rdir, "events_%d.jsonl" % j), "w", encoding="utf-8") as f:
            for r in acc:
                if r.get("event") != "frame_processed":
                    f.write(json.dumps(r, ensure_ascii=False) + "\n")
        s = task_summary(find_slice(acc, prev_task))
        rec.update(s)
        rec["category"] = classify(s, self.wall_x)
        tracks = self.memory()
        rec["found_err_m"], rec["found_label"] = found_error(tracks, s.get("target_id"), label)
        if rec["category"] == "success" and rec["found_label"] and rec["found_label"] != label:
            rec["category"] = "found_wrong_object"
        try:
            with open(os.path.join(self.run, "navigation.log"), encoding="utf-8", errors="replace") as f:
                rec["nav_log"] = nav_log_issues(f.readlines(), t0 - 1, t1 + 1)
        except OSError:
            rec["nav_log"] = {}
        if s["state"] is None:   # leave the next find a clean runner
            self.pc("cancel", timeout=60)
            time.sleep(3)
        self.say("find", j, word, "->", rec["category"], "first_nav=%s" % s["first_navigate_reason"],
                 "navs=%d" % s["n_navigate"], "%.0fs" % rec["wall_s"], "err=%s" % rec["found_err_m"])
        return rec

    def round(self, i):
        a = self.args
        tag = "%s-r%d-%s" % (a.label[:10], i, time.strftime("%H%M%S"))
        self.rdir = os.path.join(self.out, "round-%d" % i)
        os.makedirs(self.rdir, exist_ok=True)
        self.run = os.path.join(self.rdir, tag)
        self.offset = 0
        self.wall_x = a.wall_x if a.wall_x is not None else 1.0
        env = {"CAR_MAPPING": "1", "CAR_SIM_NAV_PROFILE": a.sim_nav_profile}
        if a.wall_x is not None:
            env["WALL_X_M"] = "%g" % a.wall_x
        if a.fault_nav_fail_first is not None:
            env["FPGA_FAULT_NAV_FAIL_FIRST"] = str(a.fault_nav_fail_first)
        rec = {"round": i, "run": self.run, "wall_x": self.wall_x, "sim_nav_profile": a.sim_nav_profile,
               "fault_nav_fail_first": a.fault_nav_fail_first,
               "head": subprocess.run(["git", "-C", REPO, "rev-parse", "HEAD"], stdout=subprocess.PIPE,
                                      universal_newlines=True).stdout.strip()}
        up_cmd = ["bash", "tools/car_run/car_up.sh", "sim", "--yolo-backend", "ort", "--run-dir", self.run]
        if a.gpu is not None:
            up_cmd = ["bash", "tools/car_run/car_up.sh", "sim", "--yolo-backend", "torch", "--gpu", str(a.gpu),
                      "--run-dir", self.run]
        t = time.time()
        try:
            p = self.sh(up_cmd, env=env, timeout=1200)
            rec["up_rc"], rec["up_s"] = p.returncode, round(time.time() - t, 1)
            if p.returncode != 0:
                rec["up_tail"] = p.stdout[-1500:]
                return rec
            time.sleep(5)
            rec["map_before"] = self.map_info()
            rec["walk"] = [self.goto(f, d) for f, d in WALK]
            self.mark()
            p = self.pc("mapping-stop", timeout=240)
            rec["mapping_stop_tail"] = p.stdout[-600:]
            status = None
            for _ in range(60):
                try:
                    status = json.load(open(os.path.join(self.run, "mapping_status.json")))
                except (OSError, ValueError):
                    status = None
                if status and status.get("state") in ("exported", "failed") and status.get("map_loaded") is not None:
                    break
                time.sleep(2)
            rec["mapping_status"] = status
            rec["map_after"] = self.map_info()
            rec["memory_align_events"] = [r for r in self.events() if str(r.get("event", "")).startswith("memory_align")]
            time.sleep(max(0, a.settle_s))
            tracks = self.memory()
            rec["memory_after_mapping"] = truth_errors(tracks)
            rec["memory_tracks"] = [{k: t.get(k) for k in ("id", "label", "position_m", "state")} for t in tracks]
            words = [w for w in a.targets.split(",") if w]
            rec["finds"] = [self.find(j, words[j % len(words)], WORD_LABEL.get(words[j % len(words)]))
                            for j in range(a.k)]
            rec["memory_final"] = truth_errors(self.memory())
            for name in ("runtime.jsonl", "navigation.log"):
                rec.setdefault("files", []).append(os.path.join(self.run, name))
        finally:
            d = self.sh(["bash", "tools/car_run/car_down.sh", "--run", self.run], timeout=600)
            rec["down_rc"] = d.returncode
            with open(os.path.join(self.rdir, "round.json"), "w", encoding="utf-8") as f:
                json.dump(rec, f, ensure_ascii=False, indent=1)
        return rec


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rounds", type=int, default=1)
    ap.add_argument("--k", type=int, default=5)
    ap.add_argument("--wall-x", type=float, default=None, help="stand-in export wall x (unset = stand-in default 1.0)")
    ap.add_argument("--sim-nav-profile", choices=("sim", "car"), default="car")
    ap.add_argument("--fault-nav-fail-first", type=int, default=None)
    ap.add_argument("--targets", default="瓶子,杯子,书")
    ap.add_argument("--task-wait-s", type=float, default=220.0)
    ap.add_argument("--settle-s", type=float, default=6.0, help="wait after mapping stop (memory snapshot every 5 s)")
    ap.add_argument("--gpu", type=int, default=None, help="torch on this GPU instead of ort on CPU")
    ap.add_argument("--out-dir", default="${FPGA_SRC}/runs/e2e-flow-sim-20261025")
    ap.add_argument("--label", default="base")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    out = os.path.join(a.out_dir, a.label)
    if a.dry_run:
        print(json.dumps({"out": out, "rounds": a.rounds, "k": a.k, "wall_x": a.wall_x, "walk": WALK, "leave": LEAVE,
                          "targets": a.targets, "sim_nav_profile": a.sim_nav_profile,
                          "fault_nav_fail_first": a.fault_nav_fail_first}, ensure_ascii=False))
        return 0
    os.makedirs(out, exist_ok=True)
    flow = Flow(a, out)
    rounds = []
    for i in range(1, a.rounds + 1):
        rounds.append(flow.round(i))
        tasks = [f for r in rounds for f in r.get("finds", [])]
        summ = {"label": a.label, "rounds": [{k: r.get(k) for k in ("round", "run", "up_rc", "up_s", "down_rc",
                                                                 "mapping_status", "memory_after_mapping")}
                                             for r in rounds], "aggregate": aggregate(tasks)}
        with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summ, f, ensure_ascii=False, indent=1)
    agg = aggregate([f for r in rounds for f in r.get("finds", [])])
    flow.say("success %d/%d (%s) %s" % (agg["success"], agg["n"], agg["rate"], json.dumps(agg["categories"],
                                                                                       ensure_ascii=False)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
