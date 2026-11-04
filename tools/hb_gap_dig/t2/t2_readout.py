#!/usr/bin/env python3
"""T2 (2026-10-16) readout: one arm per session, judged per runs/hbgap-t2-20261016/PREREG.md.

  t2_readout.py SESSION_DIR --out OUT.json            (SESSION_DIR = runs/hbgap-t2-20261016/log_<ARM>_MMDD_HHMM)
  t2_readout.py --table OUT1.json OUT2.json ...         (side-by-side table of finished readouts)
Same constants as T1 (tools/hb_gap_dig/hb_gap_dig.py: late > 80 ms over the +-50-row floor, 60 s buckets,
"on" = > 5 late/min). The criteria window is [first completion, first completion + WIN_S (1620 s)] = 27 whole
buckets. Python 3.8, standard library only.
"""
import argparse
import bisect
import collections
import csv
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import hb_gap_dig as H  # noqa: E402

WIN_S = 1620                 # 27 x 60 s buckets from the first completion (T1 onsets at 300 and 1260 s)
NEAR_CAM_PER_BUCKET = 30     # > 30 no_valid_depth refusals in a bucket (5 % of 600 frames) = someone near the lens
SEG_BYTES = 524288           # Fast DDS default SHM segment payload (the host repro boundary, 10-14)


def cfg(path):
    d = {}
    for ln in open(path):
        if "=" in ln:
            k, v = ln.rstrip("\n").split("=", 1)
            d[k] = v
    return d


def load_jsonl(path):
    out = []
    if not os.path.exists(path):
        return out
    for ln in open(path):
        ln = ln.strip()
        if ln:
            try:
                out.append(json.loads(ln))
            except ValueError:
                pass
    return out


def gaps_ms(comps):
    return [(y - x) / 1e6 for x, y in zip(comps, comps[1:])]


def shm_segments(stack_shm_text):
    """ls -la lines of the stack container's /dev/shm -> {name: bytes} (fastrtps_* only, no _el / port files)."""
    seg = {}
    for ln in (stack_shm_text or "").splitlines():
        f = ln.split()
        if len(f) >= 9 and f[-1].startswith("fastrtps_") and not f[-1].endswith("_el") and "port" not in f[-1]:
            try:
                seg[f[-1]] = int(f[4])
            except ValueError:
                pass
    return seg


def readout(S):
    c = cfg(os.path.join(S, "config.txt"))
    name, arm = c["name"], c.get("arm")
    RUN = os.path.join(S, "nano", "runs", name)
    SIDE = os.path.join(S, "nano", "runs", "t2side")
    if not os.path.isdir(SIDE):   # known-answer check on a T1 session
        SIDE = os.path.join(S, "nano", "runs", "t1side")
    run = H.Run(name, os.path.join(RUN, "frames.jsonl"))
    t0 = run.t0
    w1 = t0 + WIN_S * 1e9
    res = {"session": S, "name": name, "arm": arm, "arm_env": c.get("arm_env"), "deploy_commit": c.get("deploy_commit"),
           "win_s": WIN_S}
    last_c = run.ran[-1]["C"]
    res["window_complete"] = last_c >= w1
    nb = WIN_S // 60
    # ---- per 60 s bucket
    comps = [r["C"] for r in run.ran]
    B = []
    for k in range(nb):
        a, b = t0 + k * 60e9, t0 + (k + 1) * 60e9
        cs = [x for x in comps if a <= x < b]
        # gaps attributed to the bucket of their END completion; the first one looks back to the previous completion
        i = bisect.bisect_left(comps, a)
        prev = comps[i - 1:i] if i > 0 else []
        g = gaps_ms(prev + cs)
        rows = [r for r in run.rows if a <= r["acquired_ns"] < b]
        gi = [e for e in run.gaps_in if a <= e["raw_ns"] < b]
        nvd = sum(1 for r in rows if H.refuse_kind(r) == "no_valid_depth")
        B.append({"t_s": k * 60, "on": bool(run.on[k]) if k < len(run.on) else None,
                  "late": run.late_per_bucket[k] if k < len(run.late_per_bucket) else 0,
                  "completions": len(cs), "gt200": sum(x > 200 for x in g), "gt337": sum(x > 337 for x in g),
                  "max_gap_ms": round(max(g), 1) if g else None, "input_gap_events": len(gi),
                  "missed_frames": sum(e["missed"] for e in gi), "no_valid_depth": nvd,
                  "over_range": sum(1 for r in rows if H.refuse_kind(r) == "over_range"),
                  "near_camera_flag": nvd > NEAR_CAM_PER_BUCKET})
    res["buckets"] = B
    ok = [b for b in B if not b["near_camera_flag"]]
    mins = len(ok)
    res["near_camera_buckets"] = [b["t_s"] for b in B if b["near_camera_flag"]]
    tot = lambda k: sum(b[k] for b in ok)  # noqa: E731
    res["window"] = {
        "minutes_used": mins, "completions": tot("completions"),
        "completions_per_min": round(tot("completions") / mins, 1) if mins else None,
        "geometry_hz": round(tot("completions") / mins / 60.0, 2) if mins else None,
        "gt200": tot("gt200"), "gt337": tot("gt337"),
        "gt200_per_min": round(tot("gt200") / mins, 2) if mins else None,
        "gt337_per_min": round(tot("gt337") / mins, 3) if mins else None,
        "max_gap_ms": max((b["max_gap_ms"] or 0) for b in ok) if ok else None,
        "max_bucket_gt200_from_180s": max((b["gt200"] for b in ok if b["t_s"] >= 180), default=None),
        "on_buckets": sum(1 for b in ok if b["on"]), "on_fraction": round(sum(1 for b in ok if b["on"]) / mins, 3) if mins else None,
        "first_on_s": next((b["t_s"] for b in B if b["on"]), None),
        "input_gap_events": tot("input_gap_events"), "missed_frames": tot("missed_frames"),
        "late_frames": tot("late")}
    res["timeline"] = "".join("x" if b["near_camera_flag"] else ("#" if b["on"] else ".") for b in B)
    # ---- driver per-frame record: published vs received by the producer, inside the window
    drv = os.path.join(RUN, "drv_frames.csv")
    if os.path.exists(drv):
        rows = list(csv.DictReader(open(drv, newline="")))
        dep = [r for r in rows if r.get("depth_global_ts_sec")]
        pub = [r for r in dep if r.get("depth_arrival_to_publish_steady_us")]
        dev = [float(r["depth_device_ts_sec"]) for r in dep]
        st = [b - x for x, b in zip(dev, dev[1:])]
        per = statistics.median(st) if st else None
        gus = sorted(int(round(float(r["depth_global_ts_sec"]) * 1e6)) for r in pub)
        # the window in the depth-stamp (REALTIME) clock: stamps of the producer's rows in the window
        wrows = [r for r in run.rows if t0 <= r["acquired_ns"] < w1]
        s_lo = min(r["depth_stamp_ns"] for r in wrows) / 1e3
        s_hi = max(r["depth_stamp_ns"] for r in wrows) / 1e3
        pub_w = [g for g in gus if s_lo <= g <= s_hi]
        lost = 0
        for e in run.gaps_in:
            if not (s_lo <= e["stamp_ns"] / 1e3 <= s_hi):
                continue
            lo = bisect.bisect_right(gus, int(round(e["prev_stamp_ns"] / 1e3)) + 2)
            hi = bisect.bisect_left(gus, int(round(e["stamp_ns"] / 1e3)) - 2)
            lost += max(0, hi - lo)
        res["driver"] = {"depth_rows": len(dep), "published": len(pub), "arrived_not_published": len(dep) - len(pub),
                         "period_ms": round(per * 1e3, 2) if per else None,
                         "device_ts_gaps_gt_1p5": sum(1 for x in st if per and x > 1.5 * per),
                         "published_in_window": len(pub_w),
                         "published_not_received_by_producer_in_window": lost,
                         "producer_loss_rate_in_window": round(lost / len(pub_w), 4) if pub_w else None}
    # ---- runner (B3 cost): processed frames
    rt = load_jsonl(os.path.join(RUN, "runtime.jsonl"))
    fp = [d for d in rt if d.get("event") == "frame_processed"]
    if fp:
        ts = [d["time"] for d in fp if isinstance(d.get("time"), (int, float))]
        span = (max(ts) - min(ts)) if len(ts) > 1 else None
        df = [d.get("dropped_frames") for d in fp if isinstance(d.get("dropped_frames"), (int, float))]
        res["runner"] = {"frame_processed": len(fp), "span_s": round(span, 1) if span else None,
                         "processed_per_s": round(len(fp) / span, 2) if span else None,
                         "dropped_frames_first_last": [df[0], df[-1]] if df else None,
                         "geometry_stale": sum(1 for d in rt if d.get("event") == "geometry_stale")}
    # ---- producer summary (QoS in effect)
    ps = os.path.join(RUN, "producer_summary.json")
    if os.path.exists(ps):
        p = json.load(open(ps))
        res["producer_qos"] = {k: p.get(k) for k in ("depth_qos", "depth_qos_switch")}
    ri = os.path.join(RUN, "runtime_info.json")
    if os.path.exists(ri):
        try:
            txt = open(ri).read()
            res["runtime_info_camera_qos"] = [x for x in ("reliable:5", "sensor_data") if x in txt]
        except OSError:
            pass
    # ---- message sizes and SHM segments (post-window probe + the sampler's 60 s records)
    ms = os.path.join(RUN, "t2", "msg_sizes.json")
    if os.path.exists(ms):
        m = json.load(open(ms))
        res["msg_sizes"] = {k: sorted(set(v)) for k, v in m["serialized_bytes"].items()}
        res["msg_decoded"] = m.get("decoded")
        res["msg_exceeds_default_payload"] = {k: (max(v) > SEG_BYTES) if v else None for k, v in m["serialized_bytes"].items()}
    slow = load_jsonl(os.path.join(SIDE, "slow.jsonl"))
    own = {}
    segs = {}
    for s in slow:
        if not s.get("procs"):
            continue
        sz = shm_segments(s.get("stack_shm"))
        if sz:
            segs = sz
        for k, v in s["procs"].items():
            for p in v:
                for f in p.get("shm_fds", []):
                    b = os.path.basename(f)
                    if b.endswith("_el") and "port" not in b:
                        own.setdefault(k, set()).add(b[:-3])
    # the driver node is a child of ros2 launch and inherits its open _el fds: its OWN segments are the ones
    # it holds and the launch process does not (board 10-16)
    if "driver_node" in own and "driver" in own:
        own["driver_node_own"] = own["driver_node"] - own["driver"]
    res["shm"] = {"segment_bytes_last_seen": sorted(set(segs.values())),
                  "own_segment_bytes": {k: sorted(set(segs.get(n) for n in v if n in segs)) for k, v in own.items()},
                  "n_segments": len(segs)}
    ow = os.path.join(RUN, "t2", "shm_owners.txt")
    if os.path.exists(ow):
        res["shm"]["owners_probe"] = open(ow).read().strip().splitlines()
    return res


def table(paths):
    rs = [json.load(open(p)) for p in paths]
    cols = [("arm", lambda r: r["arm"]), ("name", lambda r: r["name"]),
            ("min", lambda r: r["window"]["minutes_used"]),
            (">200/min", lambda r: r["window"]["gt200_per_min"]), (">337/min", lambda r: r["window"]["gt337_per_min"]),
            ("max ms", lambda r: r["window"]["max_gap_ms"]), ("on frac", lambda r: r["window"]["on_fraction"]),
            ("first on", lambda r: r["window"]["first_on_s"]), ("gaps", lambda r: r["window"]["input_gap_events"]),
            ("missed", lambda r: r["window"]["missed_frames"]),
            ("pub-not-rx", lambda r: (r.get("driver") or {}).get("published_not_received_by_producer_in_window")),
            ("loss", lambda r: (r.get("driver") or {}).get("producer_loss_rate_in_window")),
            ("geo Hz", lambda r: r["window"]["geometry_hz"]), ("timeline", lambda r: r["timeline"])]
    print("| " + " | ".join(c for c, _ in cols) + " |")
    print("|" + "---|" * len(cols))
    for r in rs:
        print("| " + " | ".join(str(f(r)) for _, f in cols) + " |")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", nargs="?")
    ap.add_argument("--out")
    ap.add_argument("--table", nargs="+")
    a = ap.parse_args()
    if a.table:
        table(a.table)
        return 0
    res = readout(a.session)
    with open(a.out, "w") as fh:
        json.dump(res, fh, ensure_ascii=False, indent=1)
    brief = {k: res.get(k) for k in ("arm", "window_complete", "window", "timeline", "near_camera_buckets", "driver",
                                     "runner", "producer_qos", "runtime_info_camera_qos", "msg_sizes", "shm")}
    print(json.dumps(brief, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
