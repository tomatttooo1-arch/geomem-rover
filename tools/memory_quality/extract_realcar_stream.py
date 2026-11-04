"""Rebuild the memory's input stream from a real-car run that saved no depth frames.

docs/design/MEMORY_QUALITY_20261016.md section 3. The runner's runtime.jsonl logs, per
processed frame, the detector boxes and the memory tracks after the update. A track
whose ``last_seen_frame`` is this frame was created or moved by exactly one observation
of this frame, and ``position_m`` IS that observation's position (Memory copies it).
Observations the old memory dropped as ambiguous have no logged position and are
counted, not reconstructed.

Camera poses are rebuilt as the runner looked them up: map -> odom identity (pose odom),
odom -> base_link from wheel_odom.jsonl (CLOCK_MONOTONIC_RAW, mapped onto the ROS
realtime stamps through the producer's depth stamp / raw arrival pairs in frames.jsonl),
base_link -> camera from mount_tf.log. Validation printed by this tool: every logged
observation must project inside a same-label detector box of its frame.

Only reads the run directory; writes one new JSON file (refuses to overwrite).
"""
import argparse
import json
import math
from pathlib import Path
import re

import numpy as np

CLOCK_FIT_BIN_S = 100.0
CLOCK_FIT_PERCENTILE = 99.0
PROJECTION_SLACK_PX = 5.0


def quat_matrix(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def mount_matrix(text):
    t = re.search(r"translation: \('([-\d.]+)', '([-\d.]+)', '([-\d.]+)'\)", text)
    q = re.search(r"rotation: \('([-\d.]+)', '([-\d.]+)', '([-\d.]+)', '([-\d.]+)'\)", text)
    if not t or not q:
        raise ValueError("mount_tf.log has no static transform")
    T = np.eye(4)
    T[:3, :3] = quat_matrix(*map(float, q.groups()))
    T[:3, 3] = list(map(float, t.groups()))
    return T


def clock_fit(frames_path):
    """realtime - raw (seconds) as a line in realtime, from the least-delayed producer pairs."""
    t, off = [], []
    with open(frames_path) as stream:
        for line in stream:
            row = json.loads(line)
            if "depth_stamp_ns" in row and "acquired_ns" in row:
                t.append(row["depth_stamp_ns"] / 1e9)
                off.append((row["depth_stamp_ns"] - row["acquired_ns"]) / 1e9)
    t, off = np.array(t), np.array(off)
    base = float(np.floor(off.min()))
    bt, bo = [], []
    for start in np.arange(t[0], t[-1], CLOCK_FIT_BIN_S):
        m = (t >= start) & (t < start + CLOCK_FIT_BIN_S)
        if m.sum() > 50:
            bt.append(np.median(t[m]) - t[0]); bo.append(np.percentile(off[m] - base, CLOCK_FIT_PERCENTILE))
    slope, intercept = np.polyfit(bt, bo, 1)
    return {"t0": float(t[0]), "base": base, "slope": float(slope), "intercept": float(intercept)}


def odom_track(path, fit, extra_s):
    raw, x, y, yaw = [], [], [], []
    with open(path) as stream:
        for line in stream:
            row = json.loads(line)
            raw.append(row["t_ns"] / 1e9); x.append(row["x"]); y.append(row["y"]); yaw.append(row["yaw"])
    raw = np.array(raw)
    # realtime = raw + offset(realtime); offset varies ~30 ppm, so one fixed-point step is exact enough
    real = raw + fit["base"] + fit["intercept"]
    real = raw + fit["base"] + fit["intercept"] + fit["slope"] * (real - fit["t0"]) + extra_s
    return real, np.array(x), np.array(y), np.array(yaw)


def base_pose(odom, ts):
    real, x, y, yaw = odom
    i = int(np.clip(np.searchsorted(real, ts), 1, len(real) - 1))
    a = float(np.clip((ts - real[i-1]) / max(real[i] - real[i-1], 1e-9), 0, 1))
    th = yaw[i-1] + a * (yaw[i] - yaw[i-1])
    T = np.eye(4)
    T[:2, :2] = [[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]]
    T[0, 3] = x[i-1] + a * (x[i] - x[i-1]); T[1, 3] = y[i-1] + a * (y[i] - y[i-1])
    return T


def assign_boxes(obs, detections, T_wc, K):
    """Each logged observation -> the same-label box whose centre is nearest its projection."""
    out, hits = [], 0
    for label in sorted({o["label"] for o in obs}):
        mine = [o for o in obs if o["label"] == label]
        boxes = [d for d in detections if d["label"] == label]
        pairs = []
        for i, o in enumerate(mine):
            c = (np.array(o["position_m"]) - T_wc[:3, 3]) @ T_wc[:3, :3]
            uv = (K @ c)[:2] / c[2]
            o["_uv"] = uv
            for j, d in enumerate(boxes):
                x1, y1, x2, y2 = d["bbox"]
                centre = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
                pairs.append((float(np.linalg.norm(uv - centre)), i, j))
        used_o, used_d = set(), set()
        for _, i, j in sorted(pairs):
            if i in used_o or j in used_d:
                continue
            used_o.add(i); used_d.add(j)
            mine[i]["bbox"] = boxes[j]["bbox"]; mine[i]["score"] = boxes[j]["score"]
        for o in mine:
            uv = o.pop("_uv")
            if "bbox" in o:
                x1, y1, x2, y2 = o["bbox"]
                s = PROJECTION_SLACK_PX
                hits += int(bool(x1 - s <= uv[0] <= x2 + s and y1 - s <= uv[1] <= y2 + s))
            else:
                o["bbox"] = None; o["score"] = None
            out.append(o)
    return out, hits


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("run", type=Path, help="car run directory (runtime.jsonl, wheel_odom.jsonl, frames.jsonl, "
                                          "mount_tf.log, camera_check.json)")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--extra-s", type=float, default=0.06,
                   help="constant added to the wheel-odom clock map (calibrated by projection hits)")
    a = p.parse_args(argv)
    if a.out.exists():
        raise SystemExit("output exists: %s" % a.out)
    run = a.run
    K = np.array(json.loads((run / "camera_check.json").read_text())["got"]["info"]["k"]).reshape(3, 3)
    T_bc = mount_matrix((run / "mount_tf.log").read_text())
    fit = clock_fit(run / "frames.jsonl")
    odom = odom_track(run / "wheel_odom.jsonl", fit, a.extra_s)
    frames, n_obs, n_hits, unapplied, tasks = [], 0, 0, 0, []
    with open(run / "runtime.jsonl") as stream:
        for line in stream:
            r = json.loads(line)
            ev = r.get("event")
            if ev == "task" and r.get("state") == "ambiguous" and r.get("candidates"):
                ids = [c["id"] for c in r["candidates"]]
                if not tasks or tasks[-1]["candidates"] != ids:
                    tasks.append({"time": r["time"], "candidates": ids})
            if ev != "frame_processed":
                continue
            ts = r["acquisition_timestamp"]
            T_wb = base_pose(odom, ts)
            T_wc = T_wb @ T_bc
            obs = [{"label": t["label"], "color": t["color"], "position_m": t["position_m"], "logged_track": t["id"]}
                   for t in r["tracks"] if t["last_seen_frame"] == r["frame_id"]]
            if obs:
                obs, hits = assign_boxes(obs, r["detections"], T_wc, K)
                n_obs += len(obs); n_hits += hits
            unapplied += r["observation_count"] - len(obs)
            frames.append({"frame_id": r["frame_id"], "t": ts,
                           "base": [round(float(T_wb[0, 3]), 6), round(float(T_wb[1, 3]), 6),
                                    round(math.atan2(T_wb[1, 0], T_wb[0, 0]), 7)],
                           "obs": obs,
                           "unapplied": r["observation_count"] - len(obs)})
            last_tracks = r["tracks"]
    doc = {"schema": "fpga.memq.realcar_stream.v1", "source_run": str(run.resolve()),
           "K": K.tolist(), "T_base_camera": T_bc.tolist(), "image_size": [640, 480],
           "clock_fit": fit, "extra_s": a.extra_s,
           "validation": {"logged_observations": n_obs, "projected_inside_box": n_hits,
                          "unapplied_observations_without_position": unapplied},
           "ambiguous_find_events": tasks,
           "logged_final_tracks": [{k: t[k] for k in ("id", "label", "position_m")} for t in last_tracks],
           "frames": frames}
    with a.out.open("x") as stream:
        json.dump(doc, stream, separators=(",", ":"))
    print(json.dumps(doc["validation"]))


if __name__ == "__main__":
    main()
