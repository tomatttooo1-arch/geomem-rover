#!/usr/bin/env python3
"""TF-gap evidence of one or more whole-chain trials (rig dirs written by rig_inner.sh with PROBE=1).

Per trial (all stamps Nano CLOCK_MONOTONIC_RAW except the ROS header stamps, which are only
compared with each other):
  producer: frames processed, tf_missing (startup/running), the evidence rows of each miss
            (newest odom<-base_link stamp the producer's buffer had, lag to the depth stamp, last
            arrival before the acquisition, arrivals in the last second), gap events, worst
            frame age / delta publish time, first processed frame after the replay's first publish
  probe:    odom<-base_link arrivals seen by an independent subscriber (same QoS as tf2): count,
            largest inter-arrival gap and where it falls; whether the probe saw samples inside
            the producer's miss window
  replay:   largest gap between consecutive depth publishes (publisher side)
  system:   UDP RcvbufErrors/InErrors increase, peak machine busy % over 100 ms samples
Usage: tf_gap_report.py RIG_DIR [RIG_DIR ...]  -> one JSON line per trial
"""
import csv
import json
import os
import sys


def jsonl(path):
    try:
        with open(path) as f:
            return [json.loads(x) for x in f if x.strip()]
    except OSError:
        return []


def rows_csv(path):
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def max_gap(ts):
    ts = sorted(ts)
    best = (0, None)
    for a, b in zip(ts, ts[1:]):
        if b - a > best[0]:
            best = (b - a, a)
    return best


def report(d):
    fr = jsonl(os.path.join(d, "frames.jsonl"))
    frames = [r for r in fr if "event" not in r]
    misses = [r for r in fr if r.get("event") == "tf_missing"]
    events = [r for r in fr if r.get("event") in ("geometry_gap", "geometry_ready", "geometry_resumed")]
    summ = {}
    try:
        summ = json.load(open(os.path.join(d, "producer_summary.json")))
    except (OSError, ValueError):
        pass
    rep = rows_csv(os.path.join(d, "replay.csv"))
    pub = sorted(int(r["publish_raw_ns"]) for r in rep)
    probe = sorted(int(r["arrival_raw_ns"]) for r in rows_csv(os.path.join(d, "tf_probe.csv")))
    sysr = rows_csv(os.path.join(d, "sys.csv"))
    t0 = pub[0] if pub else None
    rel = (lambda t: None if t is None or t0 is None else round((t - t0) / 1e9, 3))
    out = {"dir": d, "processed": len(frames), "tf_missing": (summ.get("counts") or {}).get("tf_missing"),
           "gaps": summ.get("gaps"), "tf_listener_thread": summ.get("tf_listener_thread")}
    if frames:
        out["first_frame_s"] = rel(frames[0]["acquired_ns"])
        worst = max(frames, key=lambda r: r.get("done_age_ms") or 0)
        out["worst_age"] = {"frame_id": worst["frame_id"], "s": rel(worst["acquired_ns"]),
                            "done_age_ms": worst.get("done_age_ms"), "publish_ms": worst.get("publish_ms"),
                            "geometry_ms": worst.get("geometry_ms"), "tf_ms": worst.get("tf_ms"),
                            "mailbox_wait_ms": worst.get("mailbox_wait_ms")}
        pm = [r.get("publish_ms") for r in frames if r.get("publish_ms") is not None]
        out["publish_ms_max"] = max(pm) if pm else None
        out["age_over_200"] = sum(1 for r in frames if (r.get("done_age_ms") or 0) > 200)
    if misses:
        a0, a1 = misses[0]["acquired_ns"], misses[-1]["acquired_ns"]
        out["miss_window_s"] = [rel(a0), rel(a1)]
        out["miss_phases"] = sorted({m.get("phase") for m in misses})
        out["miss_evidence_first_last"] = [{k: m.get(k) for k in ("phase", "tf_latest_stamp_lag_ms",
                                                                  "tf_last_arrival_ms_before_acq",
                                                                  "tf_arrivals_1s", "waited_ms", "error")}
                                           for m in (misses[0], misses[-1])]
        out["probe_arrivals_in_miss_window"] = sum(1 for t in probe if a0 - 1e8 <= t <= a1)
    out["events"] = [dict([(k, e.get(k)) for k in ("event", "phase", "reason", "missed", "gap_ms")],
                          s=rel(e.get("raw_ns"))) for e in events]
    g, at = max_gap(probe)
    out["probe"] = {"n": len(probe), "first_s": rel(probe[0]) if probe else None,
                    "max_gap_ms": round(g / 1e6, 1), "max_gap_at_s": rel(at)}
    g, at = max_gap(pub)
    out["replay_publish_max_gap_ms"] = round(g / 1e6, 1)
    out["replay_publish_max_gap_at_s"] = rel(at)
    if len(sysr) > 1:
        def num(r, k):
            try:
                return float(r[k])
            except (KeyError, TypeError, ValueError):
                return None
        rb = [num(r, "RcvbufErrors") for r in sysr]
        ie = [num(r, "InErrors") for r in sysr]
        busy = []
        for a, b in zip(sysr, sysr[1:]):
            db, dt = num(b, "cpu_busy") - num(a, "cpu_busy"), num(b, "cpu_total") - num(a, "cpu_total")
            if dt:
                busy.append((round(db / dt * 100, 1), rel(int(b["raw_ns"]))))
        out["udp_rcvbuf_errors_delta"] = (rb[-1] - rb[0]) if None not in (rb[0], rb[-1]) else None
        out["udp_in_errors_delta"] = (ie[-1] - ie[0]) if None not in (ie[0], ie[-1]) else None
        jumps = [(rel(int(b["raw_ns"])), num(b, "RcvbufErrors") - num(a, "RcvbufErrors"))
                 for a, b in zip(sysr, sysr[1:]) if None not in (num(a, "RcvbufErrors"), num(b, "RcvbufErrors"))
                 and num(b, "RcvbufErrors") > num(a, "RcvbufErrors")]
        out["udp_rcvbuf_error_jumps_s"] = jumps[:20]
        out["machine_busy_peak"] = max(busy) if busy else None
    return out


def main(argv=None):
    for d in (sys.argv[1:] if argv is None else argv):
        print(json.dumps(report(d), default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
