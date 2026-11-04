#!/usr/bin/env python3
"""Runner geometry age gate: per-run summary from a task runner events.jsonl.

  gate_summary.py <events.jsonl> [<label>]  -> one JSON object on stdout

frame_age_ms   the runner's existing per-frame age (kv_topic: producer acquisition RAW ns
               -> RAW now taken AFTER update_memory), the v2 board figure;
geometry_age_ms the gate's age (same origin, RAW now taken just BEFORE update_memory);
               absent in runs before the gate (then only frame_age_ms is reported and
               ``would_refuse_by_frame_age`` counts frames whose post-use age was > 200 ms).
Python 3.8 compatible, stdlib only.
"""
import collections
import json
import sys


def dist(values):
    v = sorted(float(x) for x in values if x is not None)
    if not v:
        return None

    def q(p):
        return round(v[min(len(v) - 1, int(p * (len(v) - 1) + 0.5))], 3)
    return {"n": len(v), "median": q(0.5), "p95": q(0.95), "p99": q(0.99), "p999": q(0.999),
            "max": round(v[-1], 3), "over_200": sum(1 for x in v if x > 200.0)}


def summarize(path, label=None):
    ev = [json.loads(line) for line in open(path) if line.strip()]
    kinds = collections.Counter(e.get("event") for e in ev)
    fp = [e for e in ev if e.get("event") == "frame_processed"]
    geo = [e.get("geometry") or {} for e in fp]
    results = collections.Counter(g.get("result") for g in geo)
    gates = collections.Counter(g.get("age_gate") for g in geo)
    stale = [e for e in ev if e.get("event") in ("geometry_stale", "geometry_future", "geometry_clock_mismatch",
                                                        "geometry_age_unknown")]
    close = next((e for e in reversed(ev) if e.get("event") == "geometry_close"), None) or {}
    last_tracks = fp[-1].get("tracks") if fp else []
    states = collections.Counter(t.get("state") for t in last_tracks or ())
    used = [g.get("geometry_age_ms") for g in geo if g.get("age_gate") == "pass"]
    out = {"label": label, "events": dict(kinds), "frame_processed": len(fp), "last_event": ev[-1].get("event") if ev else None,
           "geometry_results": dict(results), "age_gate_verdicts": dict(gates),
           "geometry_age_ms_all": dist(g.get("geometry_age_ms") for g in geo),
           "geometry_age_ms_used": dist(used),
           "frame_age_ms": dist(g.get("frame_age_ms") for g in geo),
           "would_refuse_by_frame_age": sum(1 for g in geo if g.get("frame_age_ms") is not None
                                            and g["frame_age_ms"] > 200.0 and g.get("result") == "fused"),
           "refusal_events": len(stale),
           "refusals": [{k: e.get(k) for k in ("frame_id", "reason", "age_ms", "age_basis", "refused_total")}
                        for e in stale[:50]],
           "close_age_gate": close.get("age_gate"), "close_counts": close.get("counts"),
           "final_tracks_by_state": dict(states), "final_tracks": len(last_tracks or ())}
    return out


if __name__ == "__main__":
    print(json.dumps(summarize(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None), indent=1))
