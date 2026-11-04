"""10-28 move replay: one class of a car run through the object memory, old vs new rules (Config.move_v2,
create_min_score_by_class, the clipped-box creation rule), plus what a find would drive to.

The car logs carry no depth image and no camera pose, so the observation stream is rebuilt, per detection of
``--label`` after the run's last map reset:
  position   the snapshot history reading of that frame and box (exact), else the logged position of the track
             that box updated in that frame (fused, within ~1 cm), else (no track updated) ``--fallback`` rules:
             a box on the top image edge -> the top-edge track's place, else the nearest-in-time placed reading
             with a similar box row (y within 40 px), else dropped (counted)
  range      fixed ``--range`` (no pose to compute it; only the sd and the 2 m creation range use it)
  truncated  memory_gates.is_truncated on the logged box (640 x 480)
  odometer   camera pose = the wheel odometry (x, y, yaw) at the acquisition time (frames.jsonl gives the
             monotonic -> unix offset): the drift gate grows as on the car
No geometry words are replayed (existence gets only detection hits), so position_invalid never happens here.

  python3 tools/memory_quality/move_log_replay.py RUN_DIR [--label banana] [--json OUT]
RUN_DIR holds runtime.jsonl, memory_snapshot.json, frames.jsonl, wheel_odom.jsonl. Read-only.
"""
import argparse
import bisect
import dataclasses
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np  # noqa: E402

from perception.fpga_replay import core, memory_gates  # noqa: E402
from host.task_control import TaskController  # noqa: E402

W, H = 640, 480
K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])


def old_may_create(obs):
    """memory_gates.may_create before 10-28 (classes not in the table could create from a clipped box)."""
    if not getattr(obs, "truncated", False):
        return True
    row = memory_gates.CLASS_MAX_APPARENT_M.get(obs.label)
    return row is None or row[1]


def mono_offset_ns(frames_path):
    vals = []
    with open(frames_path) as f:
        for line in f:
            if '"depth_stamp_ns"' not in line:
                continue
            r = json.loads(line)
            if r.get("depth_stamp_ns") and r.get("acquired_ns") and r.get("age_ms") is not None:
                vals.append(r["depth_stamp_ns"] - (r["acquired_ns"] - r["age_ms"] * 1e6))
    return float(np.median(vals))


def load_odom(path, offset_ns):
    t, x, y, yaw = [], [], [], []
    with open(path) as f:
        for line in f:
            r = json.loads(line)
            if r.get("x") is None or r.get("t_ns") is None:
                continue
            t.append((r["t_ns"] + offset_ns) * 1e-9); x.append(r["x"]); y.append(r["y"]); yaw.append(r["yaw"])
    return np.array(t), np.array(x), np.array(y), np.array(yaw)


def pose_at(odom, ts):
    t, x, y, yaw = odom
    i = min(max(bisect.bisect_left(t, ts), 0), len(t) - 1)
    c, s = math.cos(yaw[i]), math.sin(yaw[i])
    base = np.array([[c, -s, 0, x[i]], [s, c, 0, y[i]], [0, 0, 1, 0], [0, 0, 0, 1]])
    cam = np.array([[0, 0, 1, 0.1], [-1, 0, 0, 0], [0, -1, 0, 0.39], [0, 0, 0, 1]], dtype=float)
    return base @ cam


def stream(run, label, offset_ns):
    """[(row, [(det, position or None, how)])] per frame_processed after the last map reset."""
    snap = json.load(open(run / "memory_snapshot.json"))
    exact = {}
    for t in snap["tracks"]:
        for h in t["history"]:
            exact[(h["frame_id"], tuple(round(v) for v in h["bbox"]))] = h["position_m"]
    rows, reset = [], None
    with open(run / "runtime.jsonl") as f:
        for line in f:
            if '"map_reset_done"' in line:
                reset = json.loads(line)["time"]; rows = []
                continue
            if '"frame_processed"' not in line:
                continue
            r = json.loads(line)
            if r.get("event") != "frame_processed":
                continue
            rows.append(r)
    out = []
    for r in rows:
        dets = [d for d in r["detections"] if d["label"] == label]
        upd = [x for x in r["tracks"] if x["last_seen_frame"] == r["frame_id"] and x["label"] == label]
        placed = []
        for d in dets:
            key = (r["frame_id"], tuple(round(v) for v in d["bbox"]))
            top = d["bbox"][1] <= 3
            if key in exact:
                placed.append((d, exact[key], "exact"))
                continue
            pick = [x for x in upd if (x["position_m"][2] > 0.3) == top]
            if pick:
                placed.append((d, pick[0]["position_m"], "track"))
            else:
                placed.append((d, None, "unplaced"))
        out.append((r, placed))
    # fallback for unplaced boxes: top edge -> the top-edge track's last place; else nearest-in-time placed reading
    # whose box row is similar
    flat = [(r["acquisition_timestamp"], d, p) for r, pl in out for d, p, how in pl if p is not None]
    for r, pl in out:
        for k, (d, p, how) in enumerate(pl):
            if p is not None:
                continue
            ts = r["acquisition_timestamp"]
            top = d["bbox"][1] <= 3
            near = [(abs(t2 - ts), p2) for t2, d2, p2 in flat
                    if ((d2["bbox"][1] <= 3) == top) and abs(d2["bbox"][1] - d["bbox"][1]) < 40]
            if near:
                pl[k] = (d, min(near, key=lambda v: v[0])[1], "fallback")
    return reset, out


def replay(run, label, cfg, old_rules, odom, events_out, find_windows, rng):
    saved = memory_gates.may_create
    if old_rules:
        memory_gates.may_create = old_may_create
    try:
        m = core.Memory("geometry", cfg)
        reset, frames = stream(run, label, None)
        tc = TaskController(m, lambda row: [], ())
        tc.goal = label
        finds = {w: None for w in find_windows}
        unplaced = 0
        for r, placed in frames:
            ts = float(r["acquisition_timestamp"])
            T = pose_at(odom, ts)
            fr = core.Frame(r["frame_id"], ts, np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan), K, T)
            obs = []
            for d, p, how in placed:
                if p is None:
                    unplaced += 1
                    continue
                p = np.asarray(p, float)
                obs.append(core.Observation(label, float(d["score"]), p, p - [0.06, 0.06, 0.03], p + [0.06, 0.06, 0.03],
                                            "yellow", 1.0, fr.frame_id, ts, tuple(d["bbox"]), "detector",
                                            extent_m=(0.15, 0.05),
                                            truncated=memory_gates.is_truncated(d["bbox"], (H, W), cfg.edge_margin_px),
                                            range_m=rng))
            out = m.update_memory(obs, {}, fr)
            for key in ("relocated", "merged"):
                for row in out.get(key) or ():
                    events_out.append(dict(row, t=round(ts - reset, 1), kind=key))
            for w in find_windows:
                if finds[w] is None and w[0] <= ts - reset <= w[1]:
                    _, fresh = tc._fresh_rows(fr, reset + w[0])
                    if fresh:
                        row = fresh[0]
                        finds[w] = {"t": round(ts - reset, 1), "frame_id": fr.frame_id, "id": row["id"],
                                    "number": row.get("number"),
                                    "memory_position_m": [round(v, 3) for v in row["position_m"]],
                                    "approach_source": "observation" if row.get("approach_position_m") else "memory",
                                    "approach_position_m": [round(v, 3) for v in
                                                            (row.get("approach_position_m") or row["position_m"])],
                                    "n_fresh": len(fresh)}
        tracks = [{"id": t.id, "number": t.number, "position_m": [round(v, 3) for v in t.position],
                   "z": round(float(t.position[2]), 2), "state": t.state, "samples": len(t.samples)}
                  for t in m.tracks.values()]
        return {"tracks": tracks, "finds": {"%s-%s" % w: v for w, v in finds.items()}, "unplaced": unplaced,
                "rejected": dict(m.rejected), "identity_stats": dict(m.identity_stats),
                "candidate_stats": dict(m.candidate_stats)}
    finally:
        memory_gates.may_create = saved


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--label", default="banana")
    ap.add_argument("--range", type=float, default=1.5)
    ap.add_argument("--find", action="append", default=[], help="start-end seconds after the reset (task windows)")
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    odom = load_odom(a.run / "wheel_odom.jsonl", mono_offset_ns(a.run / "frames.jsonl"))
    windows = [tuple(float(v) for v in w.split("-")) for w in a.find] or [(87.0, 1e9), (149.0, 1e9)]
    base = core.Config()
    old_cfg = dataclasses.replace(base, move_v2=False, create_min_score_by_class=())
    res = {}
    for name, cfg, old in (("old_rules", old_cfg, True), ("new_rules", base, False)):
        ev = []
        res[name] = replay(a.run, a.label, cfg, old, odom, ev, windows, a.range)
        res[name]["events"] = ev
    text = json.dumps(res, indent=1, ensure_ascii=False)
    print(text)
    if a.json:
        a.json.write_text(text)


if __name__ == "__main__":
    main()
