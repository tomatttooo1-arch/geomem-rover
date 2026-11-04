#!/usr/bin/env python3
"""Line 4 readout (2026-10-15): judge a bench session of tools/car_run/bench_line4.sh against the criteria of
PREREG (car-bench-20261015-results/PREREG.md: C1-C5, and C6 the normal stop added 10-15; thresholds are the defaults
below and are printed with the verdict).

  bench_line4_readout.py SESSION [--find-text 瓶子] [--expect "bottle X Y"] [--normal-stops 1] [--json OUT]

SESSION = the session log directory: cycles.txt ("NAME KIND POSE" per line; KIND formal | normal_stop |
rtabmap_try | rtabmap_skipped | jetson_clocks) and the pulled Nano runs under SESSION/nano/runs/<NAME>/ (car_up's
run directory) and SESSION/nano/runs/<NAME>.side/ (what the session wrote beside it). Only the formal and normal-stop cycles are
judged; the rtabmap attempt (or its skip) and the jetson_clocks arm are reported.

Per formal cycle:
  C1 start/stop   car_up rc 0; car_down rc 0 and down.json clean; side/residue.json clean (no wde-car-* container,
                  no car.current pointer, no host process of the run alive)
  C2 ready        READY (up_marks.txt "READY", wall) - side/up_t0 (wall, just before car_up) <= --ready-budget-s
  C3 frame age    producer frames.jsonl, frame rows (no "event") whose acquired_ns (CLOCK_MONOTONIC_RAW) lies in
                  side/window.txt [window_start_raw_ns, window_end_raw_ns]: fused rows (age_ms not null) with
                  age_ms > 200 == 0; VALID only if fused >= --min-fused-share x window_s x --hz and the stack ran
                  through the window (running at both ends, no "EXITED" mark and no FAIL mark inside it)
  C4 fail-stop    side/failstop.json kill_rc 0; stack_fail.txt says "<node> exited after READY" and "fail-stop";
                  marks.txt "FAIL ..." line - t_kill <= --failstop-detect-s; stack container seen exited - t_kill
                  <= --failstop-stop-s; marks.txt has "EXITED <node>"
  C5 find         (a) car_pc.sh find: ACK status forwarded and RESULT status accepted (not rejected); (b) the runner's
                  FIRST answer (runtime.jsonl, time >= find_sent - 0.5 s) is a memory hit with a finite position_m: a
                  "task" in state ambiguous whose candidates hold the label, or a "command" (navigate | stop) / "task"
                  whose target_id is a track of that label in the last frame_processed snapshot; a query_rejected or a
                  search (target_id none) first = fail; (c) that position within --tol-m (x and y) of --expect;
                  (d) the PC side: the last MEMORY frame of pc_mem.fpgarec (car_pc.sh listen) has a track of that label
                  within --tol-m of --expect. Task state "found" needs Nav2 and a moving car: not judged on the bench.
                  The fault "navigation_server_unavailable" after the hit is expected (costmap
                  has no NavigateToPose server; before 2026-10-16 the word was
                  "navigation_busy_server_unavailable_or_input_unhealthy") and is reported, not counted. Since the
                  SELECT_SEMANTICS_20261016 change a single memory hit on a car without a NavigateToPose server ends
                  as task "needs_manual_approach" (first answer: the stop command with that target_id, no fault):
                  also a hit, judged the same way. Without --expect (BENCH_OBJ not given):
                  the query's answer is still recorded, C5 is "not_judged" -- neither pass nor fail (controller 10-15).
Per normal-stop cycle (cycles.txt KIND normal_stop: window + PC side + query, then car_down, no injection):
  C6 normal stop  car_up rc 0 and no FAIL/EXITED/stack_fail before the stop; car_down rc 0, down.json clean, residue
                  clean; every child of pids.txt stopped in reverse start order (rc.txt), each with its final line, none
                  needing KILL; marks.txt "cleanup" then "stopped"; the stack container exited 130 (car_inner.sh's
                  INT trap) and car_down did not KILL it. C1/C2/C3/C5 of that cycle are reported, not judged.
Recorded per cycle, not judged: the runner's detections inside the window (lighting evidence) and --site-note.
Overall: a criterion passes when it passes in every formal cycle and there are --cycles (3) formal cycles (C6: in the
--normal-stops (1) normal-stop cycles; --normal-stops 0 = C6 left out). A verdict is true | false | "not_judged".
--cycles 0 (the find-only rerun): C1-C4 are left out, C5 is judged on the normal-stop cycle(s), C6 as usual.
Exit 0 all pass, 1 any criterion fails, 3 none fails but one is not judged (C5 without --expect), 2 bad input.
Python 3.8 compatible."""
import argparse
import json
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# the runner's own query words (perception/fpga_replay/coco_zh.py, used by core.py query_memory), for the
# default label of --find-text; any COCO class is accepted (D-044, 2026-10-21; D-036's four before)
from perception.fpga_replay import coco_zh  # noqa: E402

WORDS = coco_zh.ZH_TO_CLASS
COCO = coco_zh.COCO_NAMES


def read(path, default=None):
    try:
        with open(path, encoding="utf-8") as f:
            return f.read()
    except OSError:
        return default


def read_json(path):
    t = read(path)
    if t is None:
        return None
    try:
        return json.loads(t)
    except ValueError:
        return None


def jsonl(path):
    rows = []
    t = read(path)
    for line in (t or "").splitlines():
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if isinstance(r, dict):
            rows.append(r)
    return rows


def kv_lines(text):
    out = {}
    for line in (text or "").splitlines():
        k, sep, v = line.partition("=")
        if sep:
            out[k.strip()] = v.strip()
    return out


def marks(path):
    """[(wall float, text)] from a marks/up_marks file ("<epoch.ns> text")."""
    out = []
    for line in (read(path) or "").splitlines():
        ts, _, text = line.partition(" ")
        try:
            out.append((float(ts), text))
        except ValueError:
            continue
    return out


def finite_xy(p):
    return isinstance(p, (list, tuple)) and len(p) >= 2 and all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in p[:2])


def near(p, expect, tol):
    return finite_xy(p) and abs(p[0] - expect[0]) <= tol and abs(p[1] - expect[1]) <= tol


def label_of_text(text):
    """The first COCO class named in ``text`` (Chinese longest-match or English), else None."""
    labels = coco_zh.labels_in_text(text)
    return labels[0] if labels else None


# ------------------------------------------------------------------------------------------ criteria
def c1(run, side):
    up_rc = (read(os.path.join(side, "up_rc")) or "").strip()
    down_rc = (read(os.path.join(side, "down_rc")) or "").strip()
    down = read_json(os.path.join(run, "down.json")) or {}
    res = read_json(os.path.join(side, "residue.json")) or {}
    ok = up_rc == "0" and down_rc == "0" and down.get("clean") is True and res.get("clean") is True
    return {"pass": ok, "up_rc": up_rc or None, "down_rc": down_rc or None, "down_json_clean": down.get("clean"),
            "residue": res or None}


def c2(run, side, budget_s):
    t0 = read(os.path.join(side, "up_t0"))
    ready = [t for t, text in marks(os.path.join(run, "up_marks.txt")) if text.strip() == "READY"]
    tries = [text for _, text in marks(os.path.join(run, "marks.txt")) if text.startswith("camera driver try=")]
    if t0 is None or not ready:
        return {"pass": False, "ready_s": None, "budget_s": budget_s, "reason": "no up_t0 or no READY mark",
                "driver_tries": tries}
    s = ready[0] - float(t0)
    return {"pass": 0 <= s <= budget_s, "ready_s": round(s, 2), "budget_s": budget_s, "driver_tries": tries,
            "ready_marks": [text for _, text in marks(os.path.join(run, "marks.txt")) if text.startswith("ready ")]}


def c3(run, side, a):
    w = kv_lines(read(os.path.join(side, "window.txt")))
    try:
        s_raw, e_raw = int(w["window_start_raw_ns"]), int(w["window_end_raw_ns"])
        s_wall, e_wall = float(w["window_start_wall"]), float(w["window_end_wall"])
    except (KeyError, ValueError):
        return {"pass": False, "valid": False, "reason": "no/bad window.txt"}
    win_s = (e_raw - s_raw) / 1e9
    rows = [r for r in jsonl(os.path.join(run, "frames.jsonl"))
            if "event" not in r and isinstance(r.get("acquired_ns"), int) and s_raw <= r["acquired_ns"] <= e_raw]
    fused = [r for r in rows if r.get("age_ms") is not None]
    ages = sorted(float(r["age_ms"]) for r in fused)
    over = sum(1 for v in ages if v > 200.0)
    inside = [text for t, text in marks(os.path.join(run, "marks.txt")) if s_wall <= t <= e_wall]
    died = [t for t in inside if t.startswith("EXITED") or t.startswith("FAIL")]
    need = a.min_fused_share * a.window_s * a.hz
    running = w.get("stack_running_at_start") == "true" and w.get("stack_running_at_end") == "true"
    valid = len(fused) >= need and running and not died and abs(win_s - a.window_s) <= 5
    rt = [r for r in jsonl(os.path.join(run, "runtime.jsonl"))
          if r.get("event") == "frame_processed" and isinstance(r.get("time"), (int, float)) and s_wall <= r["time"] <= e_wall]
    rt_age = sorted(float((r.get("geometry") or {}).get("frame_age_ms")) for r in rt
                    if isinstance((r.get("geometry") or {}).get("frame_age_ms"), (int, float)))

    def q(v, p):
        return round(v[min(len(v) - 1, int(p * (len(v) - 1) + 0.5))], 2) if v else None
    return {"pass": valid and over == 0, "valid": valid, "over_200": over, "fused": len(fused), "frame_rows": len(rows),
            "fused_needed": round(need, 1), "window_s": round(win_s, 3), "stack_ran_through": running and not died,
            "marks_inside_window": died, "age_ms": {"median": q(ages, 0.5), "p99": q(ages, 0.99), "max": q(ages, 1.0)},
            "runner_secondary": {"frame_processed": len(rt), "geometry_frame_age_ms_max": q(rt_age, 1.0),
                                 "geometry_frame_age_over_200": sum(1 for v in rt_age if v > 200.0)}}


def window_detections(run, side):
    """the runner's detections inside the 120 s window (frame_processed rows, wall time in [window_start_wall,
    window_end_wall]) -- recorded, not judged: evidence of the lighting (owner 10-15: the bench room was dark; the
    colour image is dim, the active-IR depth is not affected)."""
    w = kv_lines(read(os.path.join(side, "window.txt")))
    try:
        s_wall, e_wall = float(w["window_start_wall"]), float(w["window_end_wall"])
    except (KeyError, ValueError):
        return {"frames": None, "reason": "no/bad window.txt"}
    rows = [r for r in jsonl(os.path.join(run, "runtime.jsonl"))
            if r.get("event") == "frame_processed" and isinstance(r.get("time"), (int, float)) and s_wall <= r["time"] <= e_wall]
    by_label = {}
    with_det = 0
    for r in rows:
        dets = r.get("detections") if isinstance(r.get("detections"), list) else []
        with_det += 1 if dets else 0
        for d in dets:
            lab = d.get("label") if isinstance(d, dict) else None
            by_label[lab] = by_label.get(lab, 0) + 1
    return {"frames": len(rows), "frames_with_detection": with_det, "detections": sum(by_label.values()),
            "by_label": by_label}


def c4(run, side, a):
    fs = read_json(os.path.join(side, "failstop.json")) or {}
    node = fs.get("node") or a.failstop_node
    fail = read(os.path.join(run, "stack_fail.txt")) or ""
    mk = marks(os.path.join(run, "marks.txt"))
    t_kill = fs.get("t_kill_wall")
    fail_marks = [t for t, text in mk if text.startswith("FAIL") and t_kill is not None and t >= t_kill]
    exited = any(text.startswith("EXITED %s " % node) or text == "EXITED %s" % node for _, text in mk)
    detect = (fail_marks[0] - t_kill) if fail_marks else None
    stop = (fs["t_stack_exited_seen_wall"] - t_kill) if fs.get("t_stack_exited_seen_wall") and t_kill else None
    text_ok = ("%s exited after READY" % node) in fail and "fail-stop" in fail
    ok = (fs.get("kill_rc") == 0 and text_ok and exited and detect is not None and detect <= a.failstop_detect_s
          and stop is not None and stop <= a.failstop_stop_s)
    return {"pass": ok, "node": node, "signal": fs.get("signal"), "kill_rc": fs.get("kill_rc"),
            "stack_fail": fail.strip() or None, "stack_fail_text_ok": text_ok, "exited_mark": exited,
            "detect_s": None if detect is None else round(detect, 3), "detect_budget_s": a.failstop_detect_s,
            "all_stopped_s": None if stop is None else round(stop, 3), "stop_budget_s": a.failstop_stop_s,
            "rc_txt": (read(os.path.join(run, "rc.txt")) or "").splitlines()}


def pc_memory_tracks(path):
    """tracks of the last MEMORY frame of a .fpgarec (pc/fpgac.py), or None when unreadable."""
    try:
        from pc import fpgac
        _, rows = fpgac.read_fpgarec(path)
    except Exception:  # noqa: BLE001 - missing / truncated file: reported as None
        return None
    last = None
    for _, raw in rows:
        f = fpgac.decode_one(raw)
        if f.kind == fpgac.KIND["MEMORY"]:
            o = f.obj()
            if isinstance(o, dict):
                t = o.get("tracks")
                if t is None and isinstance(o.get("memory"), dict):
                    t = o["memory"].get("tracks")
                if isinstance(t, list):
                    last = t
    return last


NOT_JUDGED = "not_judged"


def track_hit(tracks, tid, label):
    """the memory track TID of the snapshot, if it has that label and a finite position (the memory hit)."""
    tr = [x for x in tracks if x.get("id") == tid] if tid is not None else []
    if tr and tr[0].get("label") == label and finite_xy(tr[0].get("position_m")):
        return tr[0]
    return None


def first_answer(rows, t_q, label):
    """(first answer row summary, hit) -- the runner's first answer to the query sent at t_q (+0.5 s slack).

    Emitters (host/ros_runtime.py, 10-15): query() records "query_rejected" (not ready / busy), else
    TaskController.start() -> execute() records one "command" row per command ({kind, command_id, pose, target_id,
    reason}; a memory hit = navigate/stop with target_id = a remembered track; a search = navigate with target_id
    None) and publish_task() a "task" row (state, target_id, candidates [{id, label, color, position_m, state}]);
    several candidates = task state "ambiguous" with no command. The memory snapshot is the "tracks" list of the
    last "frame_processed" row before the answer."""
    tracks, first, hit = [], None, None
    for r in rows:
        ev = r.get("event")
        if ev == "frame_processed" and isinstance(r.get("tracks"), list):
            tracks = r["tracks"]
        if r["time"] < t_q:
            continue
        cmd = r.get("command") if isinstance(r.get("command"), dict) else {}
        if ev == "query_rejected":
            pass
        elif ev == "command" and cmd.get("kind") in ("navigate", "stop"):
            t = track_hit(tracks, cmd.get("target_id"), label)
            hit = {"via": "command_target", "target_id": cmd.get("target_id"), "candidates": [t]} if t else None
        elif ev == "task":
            if r.get("state") == "ambiguous":
                c = [x for x in r.get("candidates") or [] if x.get("label") == label and finite_xy(x.get("position_m"))]
                hit = {"via": "ambiguous", "candidates": c} if c else None
            else:
                t = track_hit(tracks, r.get("target_id"), label)
                hit = {"via": "task_target", "target_id": r.get("target_id"), "candidates": [t]} if t else None
        else:
            continue
        first = {"event": ev, "time": r["time"], "reason": r.get("reason"), "state": r.get("state"),
                 "target_id": r.get("target_id"), "command": cmd or None}
        break
    return first, hit


def c5(run, side, a, label, expect):
    """found-by-memory (controller 10-15): (a) the hub forwarded and the runner accepted the query, (b) the first
    answer is a memory hit with a finite position, (c) that position within tol of the measured one, (d) the PC got a
    track of that label within tol. No measured position (--expect / BENCH_OBJ missing): the query is still run and
    its answer recorded, but criterion 5 is NOT JUDGED -- neither pass nor fail ("not_judged")."""
    find = read(os.path.join(side, "find.txt")) or ""
    ack = res = None
    for line in find.splitlines():   # pc/fpgapc.py cmd_command: print("ACK", json) / print("RESULT", json)
        for tag in ("ACK", "RESULT"):
            if line.startswith(tag + " "):
                try:
                    v = json.loads(line[len(tag) + 1:])
                except ValueError:
                    v = None
                if tag == "ACK":
                    ack = v
                else:
                    res = v
    a_ok = isinstance(ack, dict) and ack.get("status") == "forwarded" and isinstance(res, dict) \
        and res.get("status") == "accepted"
    sent = read(os.path.join(side, "find_sent"))
    rows = [r for r in jsonl(os.path.join(run, "runtime.jsonl")) if isinstance(r.get("time"), (int, float))]
    first = hit = fault = None
    if sent is not None:
        t_q = float(sent) - 0.5
        first, hit = first_answer(rows, t_q, label)
        fault = next((r.get("reason") for r in rows if r.get("event") == "fault" and r["time"] >= t_q), None)
    b_ok = hit is not None
    positions = [x.get("position_m") for x in (hit or {}).get("candidates", [])]
    pcm = pc_memory_tracks(os.path.join(run, "pc_mem.fpgarec"))
    pc_pos = [x.get("position_m") for x in (pcm or []) if x.get("label") == label]
    if expect is None:
        c_ok = d_ok = NOT_JUDGED
        ok = NOT_JUDGED
    else:
        c_ok = any(near(p, expect, a.tol_m) for p in positions)
        d_ok = any(near(p, expect, a.tol_m) for p in pc_pos)
        ok = bool(a_ok and b_ok and c_ok and d_ok)
    return {"pass": ok, "label": label, "expect_xy": expect, "tol_m": a.tol_m,
            "a_hub_answer": {"pass": a_ok, "ack": ack, "result": res},
            "b_memory_hit": {"pass": b_ok, "first_runner_answer": first, "hit": hit},
            "c_position": {"pass": c_ok, "positions": positions},
            "d_pc_memory": {"pass": d_ok, "pc_positions": pc_pos, "pc_tracks_read": pcm is not None},
            "fault_after_query": fault,
            "note": ("未判（缺量得位置%s）：查询照跑、回答照录，不算过也不算败" % ("；" + a.site_note if a.site_note else "")
                     if expect is None else
                     "found (re-observation after navigation) needs Nav2 + a moving car: not judged on the bench")}


def rc_children(text):
    """rc.txt (car_common.sh car_stop_pid, appended by car_inner.sh's cleanup) -> (names in first-seen order,
    {name: [lines]})."""
    order, lines = [], {}
    for line in (text or "").splitlines():
        m = re.match(r"^(\S+?)(_rc=|_stopped | alive )", line)
        if not m:
            continue
        n = m.group(1)
        if n not in lines:
            order.append(n)
            lines[n] = []
        lines[n].append(line)
    return order, lines


def c6(run, side):
    """normal one-command stop (controller 10-15; no injection: the window, the PC side, the query, then car_down):
    (a) car_up rc 0 and the stack had not failed before the stop (no stack_fail.txt, no FAIL / EXITED mark);
    (b) car_down rc 0, down.json clean, side/residue.json clean;
    (c) all stopped in order: marks.txt "cleanup" then "stopped" (car_inner.sh's own stop ran to its end); rc.txt
        holds every child of pids.txt, first seen in exactly the reverse start order, each with its final
        "<name>_rc=" / "<name>_stopped" line; no child needed KILL;
    (d) rc normal: the stack container exited 130 (car_inner.sh's INT/TERM trap: exit 130) and car_down did not
        have to KILL it (down_marks.txt). Which children needed TERM, and every child rc, are recorded."""
    up_rc = (read(os.path.join(side, "up_rc")) or "").strip()
    down_rc = (read(os.path.join(side, "down_rc")) or "").strip()
    down = read_json(os.path.join(run, "down.json")) or {}
    res = read_json(os.path.join(side, "residue.json")) or {}
    fail = (read(os.path.join(run, "stack_fail.txt")) or "").strip()
    mk = [text for _, text in marks(os.path.join(run, "marks.txt"))]
    died = [t for t in mk if t.startswith("FAIL") or t.startswith("EXITED")]
    a_ok = up_rc == "0" and not fail and not died
    b_ok = down_rc == "0" and down.get("clean") is True and res.get("clean") is True
    started = [line.split()[0] for line in (read(os.path.join(run, "pids.txt")) or "").splitlines() if line.split()]
    order, lines = rc_children(read(os.path.join(run, "rc.txt")))
    final = all(any(("%s_rc=" % n) in l or ("%s_stopped" % n) in l for l in lines.get(n, [])) for n in started)
    killed = [n for n, ls in lines.items() if any("KILL" in l for l in ls)]
    termed = [n for n, ls in lines.items() if any("TERM" in l for l in ls)]
    cleanup_stopped = "cleanup" in mk and "stopped" in mk and mk.index("cleanup") < mk.index("stopped")
    c_ok = bool(started) and order == started[::-1] and final and not killed and cleanup_stopped
    stack = ((read_json(os.path.join(run, "car.json")) or {}).get("containers") or {}).get("stack")
    exit_code = None
    for line in (read(os.path.join(run, "down_rc.txt")) or "").splitlines():
        if stack and line.startswith(stack + " exit="):
            exit_code = line.split("=", 1)[1].strip()
    dmk = read(os.path.join(run, "down_marks.txt")) or ""
    container_killed = "removed with KILL" in dmk
    d_ok = exit_code == "130" and not container_killed
    t0 = read(os.path.join(side, "down_t0"))
    t1 = read(os.path.join(side, "down_t1"))
    return {"pass": bool(a_ok and b_ok and c_ok and d_ok),
            "a_up_and_alive_until_stop": {"pass": a_ok, "up_rc": up_rc or None, "stack_fail": fail or None,
                                          "fail_or_exited_marks": died},
            "b_no_residue": {"pass": b_ok, "down_rc": down_rc or None, "down_json_clean": down.get("clean"),
                             "residue": res or None},
            "c_in_order": {"pass": c_ok, "started": started, "stopped_order": order, "every_child_final_line": final,
                           "killed": killed, "cleanup_then_stopped": cleanup_stopped},
            "d_rc": {"pass": d_ok, "stack_container_exit": exit_code, "container_killed": container_killed},
            "term_used": termed, "rc_txt": (read(os.path.join(run, "rc.txt")) or "").splitlines(),
            "down_s": round(float(t1) - float(t0), 3) if t0 and t1 else None}


def judge_cycle(run, side, a, label, expect, kind="formal"):
    d = {"c1_start_stop": c1(run, side), "c2_ready": c2(run, side, a.ready_budget_s), "c3_frame_age": c3(run, side, a),
         "c5_find": c5(run, side, a, label, expect), "window_detections": window_detections(run, side),
         "car": read_json(os.path.join(run, "car.json")), "camera_check": read_json(os.path.join(run, "camera_check.json")),
         "runtime_info_camera_qos": (read_json(os.path.join(run, "runtime_info.json")) or {}).get("camera_qos")}
    if kind == "normal_stop":
        d["c6_normal_stop"] = c6(run, side)     # no injection in this cycle: C4 does not apply
    else:
        d["c4_failstop"] = c4(run, side, a)
    return d


def combine(values, enough):
    """per-cycle results of one criterion -> True (all pass) | False (any fails, or cycles missing) | not_judged."""
    if not enough or any(v is False for v in values):
        return False
    if any(v == NOT_JUDGED for v in values):
        return NOT_JUDGED
    return all(v is True for v in values)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--find-text", default="瓶子")
    ap.add_argument("--expect", default=None, help='"<label> <x> <y>" metres in base_link = map')
    ap.add_argument("--tol-m", type=float, default=0.30)
    ap.add_argument("--ready-budget-s", type=float, default=240.0)
    ap.add_argument("--window-s", type=float, default=120.0)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--min-fused-share", type=float, default=0.90)
    ap.add_argument("--failstop-node", default="producer")
    ap.add_argument("--failstop-detect-s", type=float, default=3.0)
    ap.add_argument("--failstop-stop-s", type=float, default=60.0)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--normal-stops", type=int, default=1, help="normal-stop cycles expected (0: C6 not judged)")
    ap.add_argument("--site-note", default="", help='what the site lacked, e.g. "现场无光照" (goes into the C5 note)')
    ap.add_argument("--runs", default=None, help="the pulled runs directory (default SESSION/nano/runs)")
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    cyc = read(os.path.join(a.session, "cycles.txt"))
    if cyc is None:
        sys.stderr.write("no cycles.txt in %s\n" % a.session)
        return 2
    runs = a.runs or os.path.join(a.session, "nano", "runs")
    label, expect = label_of_text(a.find_text), None
    if a.expect:
        parts = a.expect.rsplit(None, 2)
        if len(parts) != 3:
            sys.stderr.write("--expect wants '<label> <x> <y>'\n")
            return 2
        try:
            expect = (float(parts[1]), float(parts[2]))
        except ValueError:
            sys.stderr.write("--expect: x, y must be numbers\n")
            return 2
        if parts[0] != label:
            sys.stderr.write("--expect label %r != the label of --find-text %r (%r)\n" % (parts[0], a.find_text, label))
            return 2
    if label not in COCO:
        sys.stderr.write("--find-text %r maps to %r, not a COCO class\n" % (a.find_text, label))
        return 2
    out = {"session": os.path.abspath(a.session), "find_text": a.find_text, "label": label, "expect_xy": expect,
           "thresholds": {k: getattr(a, k) for k in ("tol_m", "ready_budget_s", "window_s", "hz", "min_fused_share",
                                                     "failstop_detect_s", "failstop_stop_s", "cycles", "normal_stops",
                                                     "site_note")},
           "cycles": {}, "normal_stop_cycles": {}, "reported": {}}
    formal, normal = [], []
    for line in cyc.splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        name, kind, pose = parts[:3]
        run, side = os.path.join(runs, name), os.path.join(runs, name + ".side")
        if kind == "formal":
            formal.append(name)
            out["cycles"][name] = dict(judge_cycle(run, side, a, label, expect), pose=pose)
        elif kind == "normal_stop":
            normal.append(name)
            out["normal_stop_cycles"][name] = dict(judge_cycle(run, side, a, label, expect, kind), pose=pose)
        elif kind == "rtabmap_try":
            out["reported"][name] = {"kind": kind, "up_rc": (read(os.path.join(side, "up_rc")) or "").strip() or None,
                                     "up_log_tail": (read(os.path.join(side, "up.log")) or "").splitlines()[-5:],
                                     "stack_fail": (read(os.path.join(run, "stack_fail.txt")) or "").strip() or None,
                                     "expected": "无 RTABMAP_DB 定位库 => car_up 退出码 2（预期）"}
        elif kind == "rtabmap_skipped":
            out["reported"][name] = {"kind": kind, "reason": " ".join(parts[3:]) or "no RTABMAP_DB",
                                     "expected": "无 RTABMAP_DB 定位库 => car_up 退出码 2（预期）；未上板尝试，不占板时"}
        else:
            out["reported"][name] = dict(judge_cycle(run, side, a, label, expect), kind=kind, pose=pose)
    enough = len(formal) == a.cycles
    out["formal_cycles"], out["normal_stop_cycle_names"] = formal, normal
    if a.cycles > 0:
        keys = ["c1_start_stop", "c2_ready", "c3_frame_age", "c4_failstop", "c5_find"]
        out["verdict"] = {k: combine([out["cycles"][n][k]["pass"] for n in formal], enough) for k in keys}
    else:
        # find-only rerun (--cycles 0, README): no fail-stop cycles -- C1-C4 are not part of this session's verdict
        # (never vacuously true); C5 is judged on the normal-stop cycle(s), C6 as usual
        keys = ["c5_find"]
        out["verdict"] = {"c5_find": combine([out["normal_stop_cycles"][n]["c5_find"]["pass"] for n in normal],
                                             len(normal) == a.normal_stops and a.normal_stops > 0)}
        out["find_only"] = True
    if a.normal_stops > 0:
        keys.append("c6_normal_stop")
        out["verdict"]["c6_normal_stop"] = combine([out["normal_stop_cycles"][n]["c6_normal_stop"]["pass"] for n in normal],
                                                   len(normal) == a.normal_stops)
    out["verdict"]["complete"] = enough and (a.normal_stops == 0 or len(normal) == a.normal_stops)
    out["poses"] = sorted({out["cycles"][n]["pose"] for n in formal} | {out["normal_stop_cycles"][n]["pose"] for n in normal})
    if "static" in out["poses"]:
        out["pose_note"] = "台架静态位姿（static），不证明真实建图/定位；rtabmap 单列为下一项"
    # True only: "not_judged" is a non-empty string and must never count as a pass
    out["all_pass"] = all(out["verdict"][k] is True for k in keys)
    out["any_fail"] = any(out["verdict"][k] is False for k in keys)
    out["not_judged"] = [k for k in keys if out["verdict"][k] == NOT_JUDGED]
    text = json.dumps(out, indent=1, ensure_ascii=False, sort_keys=True)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            f.write(text + "\n")
    print(json.dumps({"verdict": out["verdict"], "all_pass": out["all_pass"], "not_judged": out["not_judged"],
                      "formal_cycles": formal, "normal_stop_cycles": normal, "poses": out["poses"]}, ensure_ascii=False))
    if out["all_pass"]:
        return 0
    return 1 if out["any_fail"] else 3


if __name__ == "__main__":
    sys.exit(main())
