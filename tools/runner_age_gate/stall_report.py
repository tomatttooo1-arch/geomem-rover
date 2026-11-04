#!/usr/bin/env python3
"""Where the runner's slow frames lost their time (reads ros_runtime --stall-diag output).

  stall_report.py <chain dir> [--slow-ms 200] [--frames 40]
    <chain dir>/runtime/stall_diag.jsonl   per-frame timing, waits, gaps (host/runner_stall_diag.py)
    <chain dir>/runtime/runtime.err        "Dropped acquisition after bounded wait" warnings (counted)
  -> one JSON object on stdout.

Slow frame: a processed frame whose gate age (age the 200 ms gate measured) > --slow-ms, or
refused by the gate as stale. Its time is split along the frame timeline
  producer_to_depth_rx | depth_rx_to_rx | acquire | pending | submit_to_infer | infer |
  infer_to_queued | queued_to_picked | picked_to_gate
(they add up to the gate age for kv_topic); the largest one is its class, and when that is
picked_to_gate the largest poll section before the gate names it. For every slow frame the
report also lists diagnostics rows inside [origin - 1 s, gate time]: tf_arrival_gap, poll_gap,
gil_stall, gc_pause, graph_change, acquisition_dropped, frame_superseded, slow_record,
poll_sections (a poll without a frame whose permit / /fpga/memory section took >= 50 ms), inject.
Long sections (>= 100 ms) come from frame rows and frameless poll rows (frame_id None).
Positive-control runs (stall_diag_start.inject set): injected count and inject rows (in a frame /
in a frameless poll) vs. rows (frame or frameless poll) whose injected section is >= 95 % of the
injected ms while every other section stays below it; inject rows overlapped by a GIL watchdog
stall >= ms/2 (hold mode).
Python 3.8 compatible.
"""
import argparse
import collections
import json
import os
import re

SPANS = ("producer_to_depth_rx", "depth_rx_to_rx", "acquire", "pending", "submit_to_infer", "infer",
         "infer_to_queued", "queued_to_picked", "picked_to_gate")
PRE_GATE = ("result", "process", "geometry", "gate")
PUBLISH_SECTIONS = ("record_publish", "permit", "memory_publish")
WINDOW_EVENTS = ("tf_arrival_gap", "poll_gap", "gil_stall", "gc_pause", "graph_change", "acquisition_dropped",
                 "frame_superseded", "slow_record", "poll_sections", "inject")


def load(path):
    with open(path) as stream:
        return [json.loads(l) for l in stream if l.strip()]


def dist(values):
    v = sorted(x for x in values if x is not None)
    if not v:
        return None
    q = lambda p: round(v[min(len(v) - 1, int(p * len(v)))], 3)
    return {"n": len(v), "median": q(0.5), "p95": q(0.95), "p99": q(0.99), "max": round(v[-1], 3)}


def end_raw(row):
    for k in ("end_raw_ns", "raw_ns"):
        if row.get(k) is not None:
            return row[k]
    return None


def classify(row):
    spans = {k: v for k, v in (row.get("spans_ms") or {}).items() if v is not None}
    if not spans:
        return None, None
    top = max(spans, key=lambda k: spans[k])
    sub = None
    if top == "picked_to_gate":
        sec = {k: v for k, v in (row.get("sections_ms") or {}).items() if k in PRE_GATE}
        sub = max(sec, key=lambda k: sec[k]) if sec else None
    return top, sub


def spearman(xs, ys):
    pairs = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
    if len(pairs) < 3:
        return None

    def ranks(v):
        order = sorted(range(len(v)), key=lambda i: v[i])
        r = [0.0] * len(v)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and v[order[j + 1]] == v[order[i]]:
                j += 1
            for k in range(i, j + 1):
                r[order[k]] = (i + j) / 2.0
            i = j + 1
        return r
    rx, ry = ranks([p[0] for p in pairs]), ranks([p[1] for p in pairs])
    n = len(pairs)
    mx, my = sum(rx) / n, sum(ry) / n
    cov = sum((a - mx) * (b - my) for a, b in zip(rx, ry))
    vx = sum((a - mx) ** 2 for a in rx) ** 0.5
    vy = sum((b - my) ** 2 for b in ry) ** 0.5
    return round(cov / (vx * vy), 3) if vx and vy else None


def report(chain, slow_ms=200.0, frames=40):
    rows = load(os.path.join(chain, "runtime", "stall_diag.jsonl"))
    by = collections.defaultdict(list)
    for r in rows:
        by[r.get("event")].append(r)
    timing = by["frame_timing"]
    processed = [r for r in timing if r.get("result") == "processed"]
    err = os.path.join(chain, "runtime", "runtime.err")
    warn = len(re.findall(r"Dropped acquisition after bounded wait", open(err).read())) if os.path.exists(err) else None
    start = (by["stall_diag_start"] or [{}])[0]
    summary = (by["stall_diag_summary"] or [{}])[-1]
    window_rows = sorted((r for k in WINDOW_EVENTS for r in by[k] if end_raw(r) is not None), key=end_raw)

    slow = []
    for r in processed:
        age = r.get("gate_age_ms")
        if (age is not None and age > slow_ms) or r.get("gate_reason") == "geometry_stale":
            top, sub = classify(r)
            origin = r.get("origin_raw_ns")
            gate_t = origin + int(age * 1e6) if origin is not None and age is not None else r.get("end_raw_ns")
            lo = (origin if origin is not None else gate_t) - 10 ** 9
            near = [dict(e, t_rel_gate_ms=round((end_raw(e) - gate_t) / 1e6, 1)) for e in window_rows
                    if lo <= end_raw(e) <= gate_t]
            slow.append({"frame_id": r["frame_id"], "gate_age_ms": age, "verdict": r.get("gate_verdict"),
                         "class": top, "pre_gate_section": sub, "spans_ms": r.get("spans_ms"),
                         "held_ms": r.get("held_ms"), "acq_tries": r.get("acq_tries"),
                         "acq_branches": r.get("acq_branches"), "acq_max_tf_lag_ms": r.get("acq_max_tf_lag_ms"),
                         "acq_prepared_by": r.get("acq_prepared_by"), "acq_tries_by": r.get("acq_tries_by"),
                         "tracks": r.get("tracks"),
                         "big_sections": {k: [v, r["sections_cpu_ms"].get(k)] for k, v in r["sections_ms"].items() if v >= 50},
                         "window_events": collections.Counter(e["event"] for e in near),
                         "window_detail": [{k: e.get(k) for k in ("event", "t_rel_gate_ms", "gap_ms", "late_ms", "pause_ms",
                                                                    "added", "removed", "reason", "since_rx_ms", "tries", "kind",
                                                                    "write_ms", "publish_ms",
                                                                    "max_tf_lag_ms", "tf_last_arrival_ago_ms",
                                                                    "tf_last_stamp_lag_ms", "main_thread_cpu_ms",
                                                                    "process_cpu_ms", "sections_ms", "section")
                                                if e.get(k) is not None}
                                           for e in near][:30]})
    classes = collections.Counter("%s%s" % (s["class"], "/" + s["pre_gate_section"] if s["pre_gate_section"] else "")
                                  for s in slow)
    sections = collections.defaultdict(list)
    cpu = collections.defaultdict(list)
    for r in processed:
        for k, v in r["sections_ms"].items():
            sections[k].append(v)
            cpu[k].append(r["sections_cpu_ms"].get(k))
    long_sections = []
    for r in processed:
        for k, v in r["sections_ms"].items():
            if v >= 100:
                c = r["sections_cpu_ms"].get(k)
                long_sections.append({"frame_id": r["frame_id"], "section": k, "ms": v, "cpu_ms": c,
                                      "blocked": c is not None and c <= 0.1 * v,
                                      "publish_blocked_signature": k in PUBLISH_SECTIONS and c is not None and c <= 10})
    for r in by["poll_sections"]:                  # polls without a frame (permit, /fpga/memory publish)
        for k, v in r["sections_ms"].items():
            if v >= 100:
                c = r["sections_cpu_ms"].get(k)
                long_sections.append({"frame_id": None, "section": k, "ms": v, "cpu_ms": c,
                                      "blocked": c is not None and c <= 0.1 * v,
                                      "publish_blocked_signature": k in PUBLISH_SECTIONS and c is not None and c <= 10,
                                      "end_raw_ns": r.get("end_raw_ns")})
    tracks = [r.get("tracks") for r in processed]
    out = {
        "chain": chain, "slow_ms": slow_ms, "inject": start.get("inject"),
        "frames": {"timing_rows": len(timing), "processed": len(processed),
                   "results": dict(collections.Counter(r.get("result") for r in timing)),
                   "gate": dict(collections.Counter(r.get("gate_verdict") for r in processed))},
        "slow_frames": len(slow), "slow_by_class": dict(classes),
        "spans_ms": {k: dist(r["spans_ms"].get(k) for r in processed) for k in SPANS},
        "gate_age_ms": dist(r.get("gate_age_ms") for r in processed),
        "sections_ms": {k: dist(v) for k, v in sorted(sections.items())},
        "sections_cpu_ms": {k: dist(v) for k, v in sorted(cpu.items())},
        "long_sections_ge_100ms": long_sections[:frames],
        "long_sections_count": len(long_sections),
        "tracks_vs_sections_spearman": {k: spearman(tracks, [r["sections_ms"].get(k) for r in processed])
                                        for k in ("process", "geometry", "memory", "record_dumps", "memory_feed")},
        "tracks": dist(tracks),
        # v6 (--tf-retry): who called the prepare_images that built each processed frame
        "acq_prepared_by": dict(collections.Counter(str(r.get("acq_prepared_by")) for r in processed)),
        "acquisition_dropped": dict(collections.Counter(r.get("reason") for r in by["acquisition_dropped"])),
        "acquisition_dropped_rows": by["acquisition_dropped"][:frames],
        "runtime_err_tf_wait_drops": warn,
        "tf_arrival_gaps": dist(r["gap_ms"] for r in by["tf_arrival_gap"]),
        "poll_sections_rows": len(by["poll_sections"]),
        "poll_gaps": [{k: r.get(k) for k in ("gap_ms", "main_thread_cpu_ms", "process_cpu_ms", "end_raw_ns")}
                      for r in by["poll_gap"]][:frames],
        "gil_stalls": dist(r["late_ms"] for r in by["gil_stall"]),
        "gc_pauses": dist(r["pause_ms"] for r in by["gc_pause"]),
        "slow_records": [{k: r.get(k) for k in ("kind", "write_ms", "publish_ms", "in_frame", "end_raw_ns")}
                         for r in by["slow_record"]][:frames],
        "graph_changes": [{k: r.get(k) for k in ("raw_ns", "added", "removed")} for r in by["graph_change"]][:frames],
        "diag_summary": summary,
        "slow": slow[:frames],
    }
    if start.get("inject"):
        section, every, ms, mode = start["inject"]
        # frame rows and frameless poll rows both carry sections
        hit = [r for r in timing + by["poll_sections"] if r["sections_ms"].get(section, 0) >= 0.95 * ms]
        clean = [r for r in hit if all(v < 0.95 * ms for k, v in r["sections_ms"].items() if k != section)]
        inj = by["inject"]
        stalls = by["gil_stall"]
        # an injection "seen by the GIL watchdog": a stall >= ms/2 overlapping [start, end + 50 ms]
        seen = [e for e in inj if any(g["late_ms"] >= ms / 2 and g["start_raw_ns"] <= e["end_raw_ns"] + 5 * 10 ** 7
                                      and g["end_raw_ns"] >= e["start_raw_ns"] for g in stalls)]
        out["inject_check"] = {"section": section, "every": every, "ms": ms, "mode": mode,
                               "injected": (summary.get("injected") or {}).get(section),
                               "inject_rows": len(inj), "inject_in_frame": sum(1 for e in inj if e.get("in_frame")),
                               "inject_frameless": sum(1 for e in inj if not e.get("in_frame")),
                               "rows_with_section_ge_95pct": len(hit),
                               "rows_frame": sum(1 for r in hit if r.get("event") == "frame_timing"),
                               "rows_frameless_poll": sum(1 for r in hit if r.get("event") == "poll_sections"),
                               "of_which_only_that_section": len(clean),
                               "inject_seen_by_gil_watchdog": len(seen),
                               "slow_attributed_to_section": sum(1 for s in slow if s["pre_gate_section"] == section
                                                                 or s["class"] == section),
                               "gil_stalls_ge_half_ms": sum(1 for r in by["gil_stall"] if r["late_ms"] >= ms / 2)}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("chain")
    ap.add_argument("--slow-ms", type=float, default=200.0)
    ap.add_argument("--frames", type=int, default=40)
    a = ap.parse_args()
    print(json.dumps(report(a.chain, a.slow_ms, a.frames), indent=1, default=str))


if __name__ == "__main__":
    main()
