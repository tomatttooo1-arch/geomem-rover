#!/usr/bin/env python3
"""Summarise the ROS-runtime board arms of the geometry switch (ros_arm.sh output dirs).

Per arm: processed frames, observations, boxes sent to the KV260, results, roi_core actually run;
CPU of runtime / stub / replay processes and of the whole Nano between the two in-run snapshots
(utime+stime ticks / CLK_TCK; machine busy = /proc/stat total - idle - iowait), as % of one core,
core-equivalents and ms per processed frame (frames whose event time falls in the window).
kv arms: frame-age chain on the Nano's CLOCK_MONOTONIC_RAW, joined to the replay CSV by the
header stamp in frame_id ("rgb-<stamp ns>"):
  publish->rx (ROS transport) | rx->start (inference + queue) | start->send (plan, crc, pack) |
  send->back (UDP + KV260; KV segment durations from kv_times_ms) | back->memory | publish->memory
Cross-arm: frames aligned by replay seq (both replays start at seq 0, same rate); per aligned
frame compare observation_count and the downstream tracks (label, state, reason, position; ids are
numbered in first-seen order, so a different drop pattern renames them).
Usage: ros_report.py --arm cpu=DIR --arm kv=DIR [--arm kvshadow=DIR] [--out JSON]
"""
import argparse
import csv
import json
import sys

TRACK_KEYS = ("label", "state", "reason")     # not id: ids follow which detection came first


def pct(values, q):
    v = sorted(x for x in values if x is not None)
    if not v:
        return None
    k = (len(v) - 1) * q / 100.0
    lo, hi = int(k), min(int(k) + 1, len(v) - 1)
    return round(v[lo] + (v[hi] - v[lo]) * (k - lo), 3)


def dist(values):
    v = [x for x in values if x is not None]
    return {"median": pct(v, 50), "p95": pct(v, 95), "n": len(v)}


def load_replay(path):
    by_stamp = {}
    with open(path) as fp:
        for r in csv.DictReader(fp):
            by_stamp[int(r["stamp_ns"])] = {k: int(v) for k, v in r.items()}
    return by_stamp


def stamp_of(frame_id):
    return int(frame_id.split("-", 1)[1]) if frame_id.startswith("rgb-") else None


def track_key(tracks):
    return sorted((tuple(t.get(k) for k in TRACK_KEYS)
                   + (tuple(round(float(x), 6) for x in (t.get("position_m") or ())),)) for t in tracks)


def cpu_window(snaps, frame_times):
    a, b = snaps[0], snaps[-1]
    hz, dt = float(a["hz"]), b["wall"] - a["wall"]
    frames = sum(1 for t in frame_times if a["wall"] <= t <= b["wall"])
    procs = {}
    for name in a["ticks"]:
        t0, t1 = a["ticks"][name], b["ticks"][name]
        if t0 is None or t1 is None:
            procs[name] = None
            continue
        s = (t1 - t0) / hz
        procs[name] = {"cpu_s": round(s, 3), "pct_one_core": round(100 * s / dt, 2),
                       "ms_per_frame": round(1e3 * s / frames, 3) if frames else None}
    c0, c1 = a["proc_stat_cpu"], b["proc_stat_cpu"]
    d = [y - x for x, y in zip(c0, c1)]
    total, idle = sum(d[:8]), d[3] + d[4]
    busy_s = (total - idle) / hz
    machine = {"busy_pct_of_machine": round(100.0 * (total - idle) / total, 2) if total else None,
               "busy_core_equiv": round(busy_s / dt, 3), "ncpu": a["ncpu"],
               "ms_per_frame": round(1e3 * busy_s / frames, 3) if frames else None,
               "loadavg_start": a.get("loadavg"), "loadavg_end": b.get("loadavg")}
    return {"window_s": round(dt, 3), "frames_in_window": frames, "fps": round(frames / dt, 3) if dt else None,
            "procs": procs, "machine": machine}


def arm_summary(d):
    rows = [json.loads(l) for l in open(d + "/events.jsonl")]
    replay = load_replay(d + "/replay.csv")
    snaps = [json.loads(l) for l in open(d + "/cpu_snaps.jsonl")]
    proc = [r for r in rows if r["event"] == "frame_processed"]
    out = {"dir": d, "events": {}, "frame_processed": len(proc), "published": len(replay),
           "frames_with_observation": sum(1 for r in proc if r.get("observation_count")),
           "observations": sum(r.get("observation_count", 0) for r in proc)}
    for r in rows:
        out["events"][r["event"]] = out["events"].get(r["event"], 0) + 1
    out["cpu"] = cpu_window(snaps, [r["time"] for r in proc])
    # every mode: header stamp -> frame_processed record, both CLOCK_REALTIME (ROS system time);
    # the only frame-age number the cpu mode has (it records no RAW chain)
    out["stamp_to_record_ms"] = dist([1e3 * (r["time"] - r["acquisition_timestamp"]) for r in proc
                                      if r.get("acquisition_timestamp") is not None])
    per_seq = {}
    for r in proc:
        rep = replay.get(stamp_of(r["frame_id"]))
        if rep is not None:
            per_seq[rep["seq"]] = r
    out["matched_to_replay"] = len(per_seq)
    geo = [r["geometry"] for r in proc if "geometry" in r]
    if geo:
        res, cores = {}, {}
        for g in geo:
            res[g["result"]] = res.get(g["result"], 0) + 1
            cores[str(g.get("roi_core"))] = cores.get(str(g.get("roi_core")), 0) + 1
        out["geometry_results"], out["roi_core"] = res, cores
        out["boxes_sent"] = sum(g.get("boxes_sent") or 0 for g in geo)
        out["frames_with_boxes"] = sum(1 for g in geo if g.get("boxes_sent"))
        chain = {"publish_to_rx_ms": [], "rx_to_start_ms": [], "start_to_send_ms": [], "send_to_back_ms": [],
                 "back_to_memory_ms": [], "rx_to_memory_ms": [], "publish_to_memory_ms": []}
        kv = {}
        for r in proc:
            g = r.get("geometry")
            if not g or g.get("result") != "fused":
                continue
            c = g.get("chain_ms") or {}
            for k in ("rx_to_start_ms", "start_to_send_ms", "send_to_back_ms", "back_to_memory_ms"):
                chain[k].append(c.get(k))
            chain["rx_to_memory_ms"].append(g.get("frame_age_ms"))
            rep = replay.get(stamp_of(r["frame_id"]))
            rx = g.get("capture_nano_ns")
            if rep is not None and rx is not None:
                p2r = (rx - rep["publish_raw_ns"]) / 1e6
                chain["publish_to_rx_ms"].append(p2r)
                if g.get("frame_age_ms") is not None:
                    chain["publish_to_memory_ms"].append(p2r + g["frame_age_ms"])
            for k, v in (g.get("kv_times_ms") or {}).items():
                kv.setdefault(k, []).append(v)
            for k, v in (g.get("roi_times_ms") or {}).items():
                kv.setdefault("roi_" + k, []).append(v)
        out["chain_ms"] = {k: dist(v) for k, v in chain.items()}
        out["kv_times_ms"] = {k: dist(v) for k, v in sorted(kv.items())}
    closes = [r for r in rows if r["event"] == "geometry_close"]
    if closes:
        out["geometry_close"] = {"counts": closes[-1].get("counts"), "shadow": closes[-1].get("shadow")}
    out["_per_seq"] = per_seq
    return out


def compare(a, b):
    sa, sb = a["_per_seq"], b["_per_seq"]
    both = sorted(set(sa) & set(sb))
    obs_eq = sum(1 for s in both if sa[s].get("observation_count") == sb[s].get("observation_count"))
    trk_eq = [s for s in both if track_key(sa[s]["tracks"]) == track_key(sb[s]["tracks"])]
    first_diff = next((s for s in both if s not in set(trk_eq)), None)
    only_a, only_b = sorted(set(sa) - set(sb)), sorted(set(sb) - set(sa))
    return {"aligned_frames": len(both), "observation_count_equal": obs_eq, "tracks_equal": len(trk_eq),
            "first_track_diff_seq": first_diff, "only_in_first": len(only_a), "only_in_second": len(only_b),
            "first_drop_difference_seq": min(only_a + only_b) if (only_a or only_b) else None}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", action="append", required=True, help="name=DIR")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    arms = dict(x.split("=", 1) for x in a.arm)
    summ = {k: arm_summary(v) for k, v in arms.items()}
    names = list(summ)
    cmp = {}
    for i, x in enumerate(names):
        for y in names[i + 1:]:
            cmp["%s_vs_%s" % (x, y)] = compare(summ[x], summ[y])
    for s in summ.values():
        s.pop("_per_seq")
    report = {"arms": summ, "compare": cmp}
    text = json.dumps(report, indent=1, sort_keys=True)
    if a.out:
        open(a.out, "x").write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
