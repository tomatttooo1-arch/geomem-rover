#!/usr/bin/env python3
"""Summary of one geometry producer rig run (tools/car_geo_producer/rig_inner.sh).

Reads <OUT>/frames.jsonl, probe.json, cpu.json, producer_summary.json, replay.csv (each
optional; a missing file is reported, not guessed) and prints one JSON object:

  frame_age_ms      acquisition (producer's RAW arrival of /fpga/depth) -> delta published,
                    fused frames only: n / median / p95 / p99.9 / max
  done_age_ms       same origin -> frame finished (every processed frame)
  camera_to_delta_ms replay camera RAW publish -> delta published (joined on the depth
                    header stamp; includes DDS transport of the image)
  expired           frames whose nav_reason or fusion_reason starts with "expired",
                    over processed frames (ratio); rate_skipped and mailbox superseded too
  results           per-frame result histogram; nav/fusion reason histograms
  roi               ROI success ratio (body failure None), failures, roi_core seen
  stage_ms          tf / geometry / fusion / rtt / kv core-device medians and p95
  bridge_status     /fpga/rolling_map/status map age (probe): n / median / p95 / p99 / max,
                    over_200, fresh share, reasons
  costmap           received (probe.costmap > 0), count, first arrival, last value histogram
                    (OccupancyGrid values: Nav2 maps 254 lethal -> 100, 0 free -> 0, 255 -> -1)
  cpu               producer / bridge % of one core, whole machine busy %, from cpu.json
Usage: summarize.py <OUT>
"""
import csv
import json
import os
import sys

import numpy as np


def pct(v, q):
    return round(float(np.percentile(v, q)), 3) if len(v) else None


def dist(v):
    v = [float(x) for x in v if x is not None]
    return {"n": len(v), "median": pct(v, 50), "p95": pct(v, 95), "p99_9": pct(v, 99.9),
            "max": round(max(v), 3) if v else None}


def load_json(path, missing):
    try:
        with open(path) as fp:
            return json.load(fp)
    except (OSError, ValueError) as exc:
        missing.append("%s: %s" % (os.path.basename(path), type(exc).__name__))
        return None


def hist(values):
    out = {}
    for v in values:
        k = str(v)
        out[k] = out.get(k, 0) + 1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        sys.stderr.write("usage: summarize.py <OUT>\n")
        return 2
    out = argv[0]
    missing = []
    rows = []
    try:
        with open(os.path.join(out, "frames.jsonl")) as fp:
            rows = [json.loads(line) for line in fp if line.strip()]
    except OSError as exc:
        missing.append("frames.jsonl: %s" % type(exc).__name__)
    # frames.jsonl also carries event rows (tf_missing evidence, geometry_gap/ready/resumed)
    events = [r for r in rows if "event" in r]
    rows = [r for r in rows if "event" not in r]
    summ = load_json(os.path.join(out, "producer_summary.json"), missing) or {}
    probe = load_json(os.path.join(out, "probe.json"), missing) or {}
    cpu = load_json(os.path.join(out, "cpu.json"), missing)
    pub = {}
    try:
        with open(os.path.join(out, "replay.csv"), newline="") as fp:
            for r in csv.DictReader(fp):
                pub[int(r["stamp_ns"])] = int(r["publish_raw_ns"])
    except (OSError, KeyError, ValueError) as exc:
        missing.append("replay.csv: %s" % type(exc).__name__)

    n = len(rows)
    fused = [r for r in rows if r.get("age_ms") is not None]
    expired = [r for r in rows if str(r.get("nav_reason")).startswith("expired")
               or str(r.get("fusion_reason")).startswith("expired")]
    cam = []
    for r in fused:
        p = pub.get(int(r["depth_stamp_ns"])) if r.get("depth_stamp_ns") is not None else None
        if p is not None:
            cam.append((r["acquired_ns"] - p) / 1e6 + r["age_ms"])
    counts = summ.get("counts", {})
    roi_ok = sum(1 for r in rows if r.get("roi") == "ok")
    res = {
        "out": os.path.abspath(out),
        "missing": missing,
        "processed": n,
        "fused": len(fused),
        "frame_age_ms": dist([r["age_ms"] for r in fused]),
        "frame_age_over_200": sum(1 for r in fused if r["age_ms"] > 200),
        "done_age_ms": dist([r.get("done_age_ms") for r in rows]),
        "camera_to_delta_ms": dist(cam),
        "camera_to_delta_joined": len(cam),
        "expired": {"n": len(expired), "ratio": round(len(expired) / n, 5) if n else None,
                    "rate_skipped": counts.get("rate_skipped"),
                    "superseded": (summ.get("slot") or {}).get("superseded"),
                    "depth_rx": counts.get("depth_rx"), "tf_missing": counts.get("tf_missing"),
                    "no_info": counts.get("no_info"), "bad_depth": counts.get("bad_depth"),
                    "worker_errors": {k: v for k, v in counts.items() if k.startswith("worker_error")}},
        "results": hist(r.get("result") for r in rows),
        "nav_reasons": hist(r.get("nav_reason") for r in rows),
        "fusion_reasons": hist(r.get("fusion_reason") for r in rows),
        "roi": {"ok": roi_ok, "ratio": round(roi_ok / n, 5) if n else None,
                "failures": hist(str(r.get("roi")).split(":", 1)[0] for r in rows if r.get("roi") != "ok"),
                "roi_core": sorted({str(r.get("roi_core")) for r in rows if r.get("roi") == "ok"})},
        "stage_ms": {k: {"median": pct([r[k] for r in rows if r.get(k) is not None], 50),
                         "p95": pct([r[k] for r in rows if r.get(k) is not None], 95)}
                     for k in ("mailbox_wait_ms", "tf_ms", "geometry_ms", "fusion_ms", "rtt_ms",
                               "kv_core_device_ms")},
        "producer_counts": counts,
        "gaps": summ.get("gaps"),
        "gap_events": [{k: e.get(k) for k in ("event", "phase", "reason", "missed", "gap_ms", "raw_ns")}
                       for e in events if e.get("event") != "tf_missing"][:50],
        "tf_missing_rows": [{k: e.get(k) for k in ("phase", "depth_stamp_ns", "waited_ms", "error",
                                                   "tf_latest_stamp_lag_ms", "tf_last_arrival_ms_before_acq",
                                                   "tf_arrivals_1s")}
                            for e in events if e.get("event") == "tf_missing"][:60],
        "publish_ms": {"median": pct([r["publish_ms"] for r in rows if r.get("publish_ms") is not None], 50),
                       "max": max([r["publish_ms"] for r in rows if r.get("publish_ms") is not None] or [None]
                                  , key=lambda v: -1 if v is None else v)},
        "labels_sent_frames": max([r.get("labels_sent") or 0 for r in rows] or [0]),
        "block": summ.get("block"),
        "kv": summ.get("kv"),
        "link": summ.get("link"),
        "client_counts": {k: v for k, v in (summ.get("client_counts") or {}).items()
                          if any(s in k for s in ("timeout", "incomplete", "error", "retry", "stale", "drop"))},
    }
    st = probe.get("status_age_ms") or {}
    res["bridge_status"] = {"n": probe.get("status"), "fresh": probe.get("status_fresh"),
                            "age_ms": st, "reasons": probe.get("status_reasons"),
                            "bridge_counts": probe.get("bridge_counts")}
    last = probe.get("last_costmap") or {}
    vals = last.get("values") or {}
    res["costmap"] = {"received": bool(probe.get("costmap", 0) > 0), "count": probe.get("costmap"),
                      "first_s": probe.get("first_costmap_t"), "last_values": vals,
                      "has_lethal_100": "100" in vals, "has_free_0": "0" in vals,
                      "last": {k: last.get(k) for k in ("w", "h", "res", "frame")} if last else None,
                      "grid_msgs": probe.get("grid"), "last_grid_values": (probe.get("last_grid") or {}).get("values"),
                      "geometry_frames_seen": probe.get("frames")}
    res["cpu"] = cpu
    print(json.dumps(res, indent=1, sort_keys=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
