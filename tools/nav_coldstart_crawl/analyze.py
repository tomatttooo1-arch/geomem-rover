#!/usr/bin/env python3
"""Judge one closed-loop run against docs/design/NAV_COLDSTART_CRAWL_20261017.md section 4.

  analyze.py RUN_DIR [--box x0,x1,y0,y1,z1] [--no-replica] [--out-dir DIR | --overwrite]
      -> DIR/summary.json + DIR/replica_per_cycle.json (+ prints the summary)
  10-17 (docs/design/CAM_TILT_10_20261017.md section 4): RUN_DIR is read only. With --out-dir the outputs go there
  (created); without it they go into RUN_DIR as before, but only if RUN_DIR holds no summary.json yet, or with
  --overwrite (one rerun overwrote another session's m5/summary.json on 10-17).

Time base = ROS time (system clock, use_sim_time false) everywhere:
  * a DWB cycle is placed at its /evaluation header.stamp; the pose there is interpolated in the scene's truth rows
    (25 Hz, both RAW "t" and ROS "ros"); runs without "ros" in the truth rows (m2) map RAW -> ROS with
    offset = min over evaluations of (RAW receipt - header stamp) (the first evaluation arrives before the recorder
    backlog builds up; checked against the truth-row offset on the runs that have both, "clock" in the summary);
  * C3 time = navigation.log ROS stamps, bt_navigator "Begin navigating" -> "Goal succeeded" (the action server's
    accept -> result); the recorder's receipt times are kept as "driver_receipt_s";
  * the end point is the stopped pose (last truth row, checked stationary for >= 1 s);
  * grids / local costmaps are taken by their header stamps (runs before the header-stamp recorder: RAW receipt
    mapped with the same offset -- later than the stamp by the receipt lag, flagged "approx").
"""
import argparse
import bisect
import json
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crawl_lib as C  # noqa: E402

FAST = 0.15 - 1e-6
LOG_STAMP = re.compile(r"^\[([\w-]+)\] \[\w+\] \[(\d+)\.(\d+)\] \[([\w.]+)\]: (.*)$")


def load_jsonl(p):
    return [json.loads(l) for l in open(p)] if os.path.exists(p) else []


def log_events(path):
    """(ros_ns, node, text) of every stamped line of navigation.log"""
    out = []
    for line in open(path, errors="replace"):
        m = LOG_STAMP.match(line.rstrip("\n"))
        if m:
            out.append((int(m.group(2)) * 10 ** 9 + int(m.group(3).ljust(9, "0")[:9]), m.group(4), m.group(5)))
    return out


class Truth(object):
    """scene truth rows on the ROS clock"""

    def __init__(self, poses, offset_raw_minus_ros):
        if poses and "ros" in poses[0]:
            self.ros = np.array([p["ros"] for p in poses], np.int64)
            self.kind = "ros_field"
        else:
            self.ros = np.array([p["t"] - offset_raw_minus_ros for p in poses], np.int64)
            self.kind = "raw_minus_eval_offset"
        self.p = poses
        self.x = np.array([p["x"] for p in poses])
        self.y = np.array([p["y"] for p in poses])
        self.yaw = np.unwrap(np.array([p["yaw"] for p in poses]))
        x0, y0, th0 = self.x[0], self.y[0], self.yaw[0]
        self.s = (self.x - x0) * math.cos(th0) + (self.y - y0) * math.sin(th0)

    def at(self, t):
        """interpolated pose, held (previous-row) velocity -- the commanded velocity is a step"""
        i = int(np.clip(np.searchsorted(self.ros, t, side="right") - 1, 0, len(self.ros) - 1))
        x, y, yaw, s = (float(np.interp(t, self.ros, a)) for a in (self.x, self.y, self.yaw, self.s))
        return {"x": x, "y": y, "yaw": yaw, "s": s, "v": self.p[i]["v"], "w": self.p[i]["w"],
                "edge": bool(t < self.ros[0] or t > self.ros[-1])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--box", default=None)
    ap.add_argument("--no-replica", action="store_true")
    ap.add_argument("--raw-minus-ros-ns", type=int, default=None,
                    help="RAW - ROS clock offset for runs whose truth rows carry no ROS stamp (m2): measured on the "
                         "runs that have both clocks; the min-receipt-lag estimate is kept only as a cross-check")
    ap.add_argument("--offset-source", default="")
    # 10-17 (docs/design/CAM_TILT_10_20261017.md section 4.1): C2's D (default 0.425 = D_geom 0.325 at the ng3 fit
    # + 0.10) and owner 12:31's C2 revision -- cycles once the car is inside the goal's 0.10 m xy tolerance are left
    # out of the 95 % window (off by default: the literal section 4 verdicts stay as they were)
    ap.add_argument("--d-limit", type=float, default=0.425)
    ap.add_argument("--c2-exclude-goal-tol", action="store_true")
    ap.add_argument("--goal-x", type=float, default=1.0)
    ap.add_argument("--goal-tol", type=float, default=0.10)
    ap.add_argument("--out-dir", default=None, help="write summary.json / replica_per_cycle.json here (RUN_DIR untouched)")
    ap.add_argument("--overwrite", action="store_true", help="without --out-dir: allow replacing RUN_DIR/summary.json")
    a = ap.parse_args()
    R = a.run
    OUT = a.out_dir or R
    if a.out_dir:
        os.makedirs(OUT, exist_ok=True)
    elif os.path.exists(os.path.join(R, "summary.json")) and not a.overwrite:
        sys.exit("analyze.py: %s/summary.json exists; give --out-dir DIR (or --overwrite)" % R)
    S = {"run": os.path.basename(R)}
    nav = json.load(open(os.path.join(R, "nav.json")))
    ev = {e["name"]: e for e in nav["events"]}
    status = ev.get("result", {}).get("status")
    evals = load_jsonl(os.path.join(R, "eval.jsonl"))
    cmds = load_jsonl(os.path.join(R, "cmd.jsonl"))
    scene = load_jsonl(os.path.join(R, "scene.jsonl"))
    frames = [r for r in scene if r["event"] == "frame"]
    poses = [r for r in scene if r["event"] == "pose"]
    g = np.load(os.path.join(R, "grids.npz"))
    header_stamped = "stamp_kind" in g.files

    # ---- clocks
    for e in evals:
        e["sn"] = int(round(e["stamp"] * 1e9))
    lag = np.array([e["t"] - e["sn"] for e in evals], np.int64)
    off_lag = int(lag.min())
    off = a.raw_minus_ros_ns if a.raw_minus_ros_ns is not None else off_lag
    tr = Truth(poses, off)
    S["clock"] = {"truth_time": tr.kind, "offset_raw_minus_ros_from_evals_ns": off_lag, "offset_used_ns": off,
                  "offset_source": a.offset_source or ("truth rows" if poses and "ros" in poses[0] else "min receipt lag"),
                  "eval_receipt_lag_ms_p50_max": [float(np.median(lag - off) / 1e6), float((lag - off).max() / 1e6)]}
    if tr.kind != "ros_field":
        S["clock"]["min_lag_minus_used_ms"] = (off_lag - off) / 1e6
    if tr.kind == "ros_field":
        true_off = np.array([p["t"] - p["ros"] for p in poses], np.int64)
        S["clock"]["offset_from_truth_rows_ns_median"] = int(np.median(true_off))
        S["clock"]["eval_offset_error_ms"] = (off_lag - int(np.median(true_off))) / 1e6
        S["clock"]["truth_offset_first_last_ns"] = [int(true_off[0]), int(true_off[-1])]
        S["clock"]["truth_ros_first_last_ns"] = [int(poses[0]["ros"]), int(poses[-1]["ros"])]
    raw2ros = (lambda t: int(t) - off)

    # ---- goal times on the ROS clock (navigation.log)
    L = log_events(os.path.join(R, "navigation.log"))
    t_begin = next((t for t, n, s in L if n == "bt_navigator" and s.startswith("Begin navigating")), None)
    t_ctrl = next((t for t, n, s in L if n == "controller_server" and s.startswith("Received a goal")), None)
    t_end_log = next(((t, s) for t, n, s in L if n == "bt_navigator" and t_begin and t > t_begin and
                      re.match(r"Goal (succeeded|failed|canceled|aborted)", s)), None)
    t_res = t_end_log[0] if t_end_log else None
    t_acc_raw = ev.get("goal_response", {}).get("t")
    t_res_raw = ev.get("result", {}).get("t")

    # ---- instrument conditions
    nav_ev = [e for e in evals if t_begin and e["sn"] >= t_begin and (t_res is None or e["sn"] <= t_res)]
    dts = np.diff([e["sn"] for e in nav_ev]) / 1e9 if len(nav_ev) > 1 else np.array([np.nan])
    gt = np.array([int(v) if header_stamped else raw2ros(v) for v in g["grid_t"]], np.int64) \
        if "grid_t" in g.files else np.array([], np.int64)
    win_hi = t_res if t_res else (gt.max() if len(gt) else 0)
    gt_nav = gt[(gt >= (t_begin or 0)) & (gt <= win_hi)] if len(gt) else gt
    fused = sum(1 for f in frames if f.get("fusion_reason") == "fused")
    S["instrument"] = {
        "eval_rows_during_nav": len(nav_ev), "controller_hz_median": float(1 / np.median(dts)),
        "controller_period_max_s": float(np.max(dts)),
        "grid_msgs_during_nav": int(len(gt_nav)),
        "grid_hz_during_nav": float((len(gt_nav) - 1) / ((gt_nav[-1] - gt_nav[0]) / 1e9)) if len(gt_nav) > 1 else 0.0,
        "grid_time": "header" if header_stamped else "receipt(approx)",
        "frames": len(frames), "fused": fused, "expired_share": 1 - fused / max(1, len(frames)),
        "frame_hz": float((len(frames) - 1) / ((frames[-1]["t_acq"] - frames[0]["t_acq"]) / 1e9)) if len(frames) > 1 else 0,
        "process_ms_p50_p90": [float(v) for v in np.percentile([f["process_ms"] for f in frames], [50, 90])],
        "render_ms_p50": float(np.median([f["render_ms"] for f in frames])),
        "age_ms_p50_p90": [float(v) for v in np.percentile([f["age_ms"] for f in frames if "age_ms" in f] or [0], [50, 90])]}
    ins = S["instrument"]
    ins["ok"] = bool(ins["controller_hz_median"] >= 8 and ins["grid_hz_during_nav"] >= 8 and ins["expired_share"] <= 0.10)
    try:
        ins["host_load"] = open(os.path.join(R, "host_load.txt")).read().split("load average:")[1].split("\n")[0].strip()
    except (OSError, IndexError):
        pass

    # ---- C1 / C2 curve from DWB itself, pose at the evaluation's header stamp
    rows = []
    for e in nav_ev:
        o = tr.at(e["sn"])
        valid = [(vx, vt) for vx, vt, tot, _ in e["tw"] if tot >= 0]
        st = [vx for vx, vt in valid if abs(vt) < 1e-9 and vx > 0]
        fast = any(vx >= FAST and abs(vt) < 1e-9 for vx, vt in valid)
        rows.append({"t": e["sn"], "s": o["s"], "x": o["x"], "y": o["y"], "yaw": o["yaw"], "v": o["v"], "w": o["w"],
                     "n": len(e["tw"]), "valid": len(valid), "max_straight": max(st) if st else 0.0, "fast": fast,
                     "best": e["tw"][e["best"]][:2] if 0 <= e["best"] < len(e["tw"]) else None,
                     "fails": sorted({f for _, _, tot, f in e["tw"] if tot < 0 and f})})
    bins = {}
    for r in rows:
        b = round(math.floor(max(0.0, r["s"]) / 0.05) * 0.05, 2)
        bins.setdefault(b, []).append(r)
    S["C1_curve"] = [{"s_from": b, "n": len(v), "min": min(r["max_straight"] for r in v),
                      "median": float(np.median([r["max_straight"] for r in v])),
                      "max": max(r["max_straight"] for r in v),
                      "chosen_vx_median": float(np.median([r["best"][0] for r in v if r["best"]] or [0])),
                      "valid_median": float(np.median([r["valid"] for r in v]))}
                     for b, v in sorted(bins.items())]
    S["C1_first_cycle"] = {k: rows[0][k] for k in ("s", "valid", "n", "max_straight", "best", "fails")} if rows else None
    S["C1_pass"] = bool(rows and rows[0]["max_straight"] <= 0.05)
    first = next((i for i, r in enumerate(rows) if r["fast"]), None)
    if first is not None:
        after = rows[first:]
        excluded = 0
        if a.c2_exclude_goal_tol:
            cut = next((i for i, r in enumerate(after) if math.hypot(r["x"] - a.goal_x, r["y"]) <= a.goal_tol), None)
            if cut is not None:
                excluded = len(after) - cut
                after = after[:cut] or after[:1]
        share = sum(r["fast"] for r in after) / len(after)
        sus = next((rows[i]["s"] for i in range(len(rows)) if all(r["fast"] for r in rows[i:])), None)
        S["C2"] = {"s_star": rows[first]["s"], "t_star_s": (rows[first]["t"] - t_begin) / 1e9,
                   "fast_share_after": share, "cycles_after": len(after), "s_sustained": sus,
                   "d_limit": a.d_limit, "exclude_goal_tol": bool(a.c2_exclude_goal_tol), "excluded_cycles": excluded,
                   "pass": bool(rows[first]["s"] <= a.d_limit + 1e-9 and share >= 0.95)}
    else:
        S["C2"] = {"s_star": None, "pass": False}
    S["chosen"] = [{"s": round(r["s"], 3), "t": round((r["t"] - t_begin) / 1e9, 2), "best": r["best"],
                    "max_straight": round(r["max_straight"], 4), "valid": r["valid"]} for r in rows]

    # ---- C3: ROS log times; stopped end point
    last = poses[-1]
    still = [p for p in poses if (tr.ros[-1] - (p["ros"] if "ros" in p else p["t"] - off)) <= 1e9]
    stationary = bool(still and max(abs(p["x"] - last["x"]) + abs(p["y"] - last["y"]) for p in still) < 1e-6
                      and all(p["v"] == 0.0 and p["w"] == 0.0 for p in still))
    at_res = tr.at(t_res) if t_res else None
    S["C3"] = {"status": status, "log_result": t_end_log[1] if t_end_log else None,
               "time_s": None if not (t_begin and t_res) else (t_res - t_begin) / 1e9,
               "time_controller_s": None if not (t_ctrl and t_res) else (t_res - t_ctrl) / 1e9,
               "driver_receipt_s": None if not (t_acc_raw and t_res_raw) else (t_res_raw - t_acc_raw) / 1e9,
               "end_xy": [last["x"], last["y"]], "end_yaw_deg": math.degrees(last["yaw"]),
               "end_stationary_1s": stationary,
               "end_err_m": math.hypot(last["x"] - 1.0, last["y"]),
               "at_result_xy": None if not at_res else [at_res["x"], at_res["y"]]}
    S["C3"]["pass"] = bool(status == 4 and S["C3"]["time_s"] is not None and S["C3"]["time_s"] <= 30.0
                           and S["C3"]["end_err_m"] <= 0.10 and stationary)
    if t_begin:
        def t_at_s(sv):
            k = np.nonzero((tr.ros >= t_begin) & (tr.s >= sv))[0]
            return None if not len(k) else float((tr.ros[k[0]] - t_begin) / 1e9)
        S["C3"]["t_at_s"] = {str(sv): t_at_s(sv) for sv in (0.05, 0.1, 0.2, 0.3, 0.325, 0.4, 0.5, 0.7, 0.9)}

    # ---- C4
    log = open(os.path.join(R, "navigation.log"), errors="replace").read()
    rec = max([f["recoveries"] for f in nav["feedback"]], default=0)
    S["C4"] = {"failed_to_make_progress": len(re.findall(r"Failed to make progress", log)),
               "no_valid_trajectories_lines": len(re.findall(r"\[DWBLocalPlanner\]: No valid trajectories", log)),
               "number_of_recoveries_max": rec,
               "behaviors": sorted(set(re.findall(r"Running (\w+)", log)))}
    S["C4"]["pass"] = bool(S["C4"]["failed_to_make_progress"] == 0 and rec == 0)

    # ---- C5: every message. Runs from m5 on: cmd_all.jsonl (a recorder process of its own) for /cmd_vel_nav,
    # /cmd_vel, /fpga/cmd_vel_safe, and the scene's own row for every command it executed (consumer side, complete
    # by construction); completeness = recorder count of /fpga/cmd_vel_safe == scene count over the recorder's span.
    # m2-m4: only the driver's cmd.jsonl (a subset: receipt backlog, ~10 Hz of the 20 Hz smoother output).
    allc = load_jsonl(os.path.join(R, "cmd_all.jsonl"))
    scmd = [r for r in scene if r["event"] == "cmd"]
    src = allc if allc else cmds
    lim = {}

    def lim_of(cs):
        return {"n": len(cs), "nonzero": sum(1 for c in cs if c["lx"] or c["az"]),
                "max_lx": max([abs(c["lx"]) for c in cs], default=0), "max_ly": max([abs(c["ly"]) for c in cs], default=0),
                "max_az": max([abs(c["az"]) for c in cs], default=0), "min_lx": min([c["lx"] for c in cs], default=0)}

    for topic in ("/cmd_vel", "/fpga/cmd_vel_safe"):
        cs = [c for c in src if c["topic"] == topic]
        lim[topic] = lim_of(cs)
        if allc and t_begin and t_res:
            tt = np.array([raw2ros(c["t"]) for c in cs])
            inn = tt[(tt >= t_begin) & (tt <= t_res)]
            lim[topic]["hz_during_nav"] = float((len(inn) - 1) / ((inn[-1] - inn[0]) / 1e9)) if len(inn) > 1 else 0.0
            lim[topic]["max_gap_ms_during_nav"] = float(np.diff(inn).max() / 1e6) if len(inn) > 1 else None
    if allc:
        nav_cs = [c for c in allc if c["topic"] == "/cmd_vel_nav"]
        lim["/cmd_vel_nav(info)"] = lim_of(nav_cs)
    if scmd:
        lim["scene_executed"] = lim_of(scmd)
        rs = [c["t"] for c in allc if c["topic"] == "/fpga/cmd_vel_safe"]
        if rs:
            lim["completeness"] = {"recorder_safe": len(rs),
                                   "scene_in_recorder_span": sum(1 for c in scmd if rs[0] <= c["t"] <= rs[-1] + 5_000_000)}
    S["C5"] = lim
    S["C5"]["source"] = "cmd_all.jsonl + scene cmd rows" if allc else "driver cmd.jsonl (subset)"
    S["C5"]["pass"] = all(v["max_lx"] <= 0.15 + 1e-6 and v["max_az"] <= 0.4 + 1e-6 and v["max_ly"] <= 1e-6
                          and v["n"] > 0 for k, v in lim.items() if k in ("/cmd_vel", "/fpga/cmd_vel_safe", "scene_executed"))

    # ---- obstacle control (grid by header stamp, truth pose at that stamp)
    if a.box:
        box = tuple(float(v) for v in a.box.split(","))
        blocked = poses[-1]["blocked"] if poses else None
        clears = [p["clear"] for p in poses if p.get("clear") is not None]
        end_row = next((r for r in scene if r["event"] == "end"), {})
        gd, go = g["grid_data"], g["grid_origin"]
        jj, ii = np.mgrid[0:80, 0:80]

        def box_cells(k):
            arr = gd[k].reshape(80, 80).astype(int)
            o = go[k]
            cx, cy = o[0] + (ii + 0.5) * C.VOX, o[1] + (jj + 0.5) * C.VOX
            inbox = (cx >= box[0]) & (cx <= box[1]) & (cy >= box[2]) & (cy <= box[3])
            return arr, inbox

        first_seen = None
        series = []
        for k in range(len(gt)):
            arr, inbox = box_cells(k)
            n_leth = int(((arr == 100) & inbox).sum())
            po = tr.at(int(gt[k]))
            gap = box[0] - (po["x"] + C.ROBOT_RADIUS_M)
            series.append((n_leth, gap))
            if n_leth > 0 and first_seen is None:
                first_seen = {"t_rel_goal_s": None if not t_begin else (int(gt[k]) - t_begin) / 1e9,
                              "lethal_cells": n_leth, "box_cells": int(inbox.sum()), "front_gap_m": gap,
                              "grid_index": k}
        arr, inbox = box_cells(len(gt) - 1)
        nav_gaps = [gp for (nl, gp), t in zip(series, gt) if t_begin and t >= t_begin]
        S["O"] = {"box": box, "blocked": blocked, "end_blocked": end_row.get("blocked"),
                  "min_clear_m": min(clears) if clears else None, "end_min_clear_m": end_row.get("min_clear"),
                  "first_lethal": first_seen,
                  "lethal_cells_at_goal_accept": next((nl for (nl, gp), t in zip(series, gt) if t_begin and t >= t_begin), None),
                  "lethal_cells_min_during_nav": min([nl for (nl, gp), t in zip(series, gt)
                                                      if t_begin and t >= t_begin and (not t_res or t <= t_res)], default=None),
                  "min_front_gap_m": min(nav_gaps) if nav_gaps else None,
                  "last_grid_box_cells": {"lethal": int(((arr == 100) & inbox).sum()),
                                          "free": int(((arr == 0) & inbox).sum()),
                                          "unknown": int(((arr < 0) & inbox).sum())},
                  "O1": bool(first_seen and first_seen["front_gap_m"] >= 0.10),
                  "O2": bool(blocked == 0 and (not clears or min(clears) >= 0.0)),
                  "O3": bool(status in (4, 5, 6) or ev.get("timeout_cancel") is not None)}

    # ---- replica cross-check (not a criterion): costmap by header stamp, pose/velocity at the evaluation stamp.
    # Per sample, obstacle-only: DWB sample legal (total >= 0) vs replica verdict None, over the samples whose DWB
    # outcome came from the obstacle critics or passed (samples thrown out by RotateToGoal / Oscillation first are
    # never obstacle-checked by DWB, critic order RotateToGoal, Oscillation, BaseObstacle, ObstacleFootprint, ...).
    if not a.no_replica and "costmap_t" in g.files:
        ct = np.array([int(v) if header_stamped else raw2ros(v) for v in g["costmap_t"]], np.int64)
        evrows = {id(r): e for r, e in zip(rows, nav_ev)}
        key = (lambda vx, vt: (round(vx, 6), round(vt, 6)))
        per = []
        for r in rows:
            e = evrows[id(r)]
            dwb = {key(vx, vt): (tot >= 0, f) for vx, vt, tot, f in e["tw"]}
            non_obst = {k for k, (ok, f) in dwb.items() if not ok and f not in ("ObstacleFootprint", "BaseObstacle")}
            st_non_obst = any(abs(k[1]) < 1e-9 and k[0] > 0 for k in non_obst)
            item = {"s": round(r["s"], 4), "dwb_max_straight": r["max_straight"], "dwb_valid": r["valid"],
                    "non_obstacle_rejects": len(non_obst), "straight_non_obstacle_reject": st_non_obst}
            for mode in ("prev", "next"):
                k = int(np.searchsorted(ct, r["t"], side="right") - 1) if mode == "prev" else \
                    int(np.searchsorted(ct, r["t"], side="left"))
                if k < 0 or k >= len(ct):
                    item[mode] = None
                    continue
                w, h = g["costmap_wh"][k]
                m = C.master_from_costmap_msg(g["costmap_data"][k], int(w), int(h))
                origin = tuple(float(v) for v in g["costmap_origin"][k])
                v, s = C.evaluate(m, origin, start=(r["x"], r["y"], r["yaw"]), v0=r["v"], w0=r["w"],
                                  cur_for_window=(r["v"], r["w"]))
                rep = {key(*kk): vv is None for kk, vv in v.items()}
                common = [kk for kk in dwb if kk in rep and kk not in non_obst]
                mism = [kk for kk in common if dwb[kk][0] != rep[kk]]
                item[mode] = {"dt_ms": (int(ct[k]) - r["t"]) / 1e6, "replica_max_straight": s["max_straight_vx"],
                              "replica_valid": s["valid"], "sample_set_equal": set(rep) == set(dwb),
                              "mismatch": len(mism), "dwb_only_legal": sum(1 for kk in mism if dwb[kk][0]),
                              "replica_only_legal": sum(1 for kk in mism if rep[kk]),
                              "straight_equal": abs(s["max_straight_vx"] - r["max_straight"]) < 1e-6}
            per.append(item)
        res = {"costmap_time": "header" if header_stamped else "receipt(approx)",
               "costmap_gap_ms_max": float(np.diff(ct).max() / 1e6) if len(ct) > 1 else None,
               "cycles": len(per)}
        for mode in ("prev", "next"):
            ok = [p for p in per if p[mode]]
            res[mode] = {"cycles": len(ok), "max_straight_equal": sum(p[mode]["straight_equal"] for p in ok),
                         "samples_all_equal": sum(p[mode]["mismatch"] == 0 for p in ok),
                         "sample_mismatch_total": sum(p[mode]["mismatch"] for p in ok)}
        both = [p for p in per if p["prev"] and p["next"]]
        res["either"] = {"cycles": len(both),
                         "max_straight_equal": sum(p["prev"]["straight_equal"] or p["next"]["straight_equal"] for p in both),
                         "samples_all_equal": sum(p["prev"]["mismatch"] == 0 or p["next"]["mismatch"] == 0 for p in both)}
        neither = [p for p in both if not (p["prev"]["straight_equal"] or p["next"]["straight_equal"])]
        res["neither_straight"] = [dict(p, prev=p["prev"], next=p["next"]) for p in neither[:12]]
        res["neither_straight_count"] = len(neither)
        res["neither_straight_with_non_obstacle_straight_reject"] = sum(p["straight_non_obstacle_reject"] for p in neither)
        res["neither_samples"] = sum(1 for p in both if p["prev"]["mismatch"] and p["next"]["mismatch"])
        S["replica_crosscheck"] = res
        json.dump(per, open(os.path.join(OUT, "replica_per_cycle.json"), "w"), indent=0)
    S["verdict_main"] = {k: S[k]["pass"] for k in ("C2", "C3", "C4") if isinstance(S.get(k), dict)}
    S["verdict_main"].update(C1=S["C1_pass"], C5=S["C5"]["pass"], instrument_ok=ins["ok"])
    json.dump(S, open(os.path.join(OUT, "summary.json"), "w"), indent=1)
    out = {k: v for k, v in S.items() if k not in ("chosen",)}
    print(json.dumps(out, indent=1)[:9000])


if __name__ == "__main__":
    main()
