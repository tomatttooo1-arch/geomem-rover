#!/usr/bin/env python3
"""TELEM (type 10/11) polling probe against a running kv_server.py (PC_DATAPLANE A.3, E.1).

Polls at --hz for --seconds on its own socket (own session; the frame channel is not
touched) with --timeout-s per request, and writes:

  <out>.jsonl   one line per poll: t_ms (since start, Nano RAW clock), outcome
                (ok|timeout|stale|error), rtt_ms, avail, ages (age_upper_ms per block),
                and a compact view of permit / estop / roi / geometry / board
  <out>         one JSON report: outcome counts, loss rate (= non-ok / sent), RTT and
                per-block age distributions, the avail sets seen, first/last sample

Usage: telem_probe.py --server HOST:PORT [--hz 10] [--seconds 120] [--timeout-s 0.2]
                      --out report.json
Loss is what the Nano would see: a poll with no answer inside --timeout-s, or an
answer echoing another request (stale), or one that fails to decode.
"""
import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.kv_link import (TELEM_ACK, TELEM_ALL, TELEM_REQ, BadExtension, KvTelemetryClient,  # noqa: E402
                          decode_telem_ack, encode_telem_req)
from host.kv_status import decode_telemetry  # noqa: E402
from host.fpga_clock import now_ns  # noqa: E402


def poll_once(client, timeout_s, late_wait_s):
    """One TELEM exchange like KvTelemetryClient.poll, keeping the KV answer stamp.
    On a timeout keep listening ``late_wait_s`` for THIS request's answer, so a late
    answer is told apart from a lost one, and the KV stamp says on which side the delay
    was: offset = kv_mono - Nano mid-time of ok answers; a late answer stamped at
    t_req + offset + small was delayed after the KV answered (network / Nano side)."""
    link = client.link
    client._ready()
    t_req = now_ns()
    link._ctrl_seq += 1
    seq = link._ctrl_seq
    link._send(TELEM_REQ, seq, encode_telem_req(TELEM_ALL, t_req), 0)
    deadline = time.monotonic() + timeout_s
    late = False
    while True:
        got = link._recv_message(deadline)
        if got is None:
            if late or late_wait_s <= 0:
                return {"outcome": "lost" if late else "timeout", "t_req": t_req}
            late = True
            deadline = time.monotonic() + late_wait_s
            continue
        mtype, _, fid, payload, _ = got
        if mtype == TELEM_ACK and fid == seq:
            break
        link.counts["stale_or_other"] += 1
    t_recv = now_ns()
    try:
        ack = decode_telem_ack(payload)
    except BadExtension as exc:
        return {"outcome": "error", "error": str(exc), "t_req": t_req}
    if ack["t_req_nano_ns"] != t_req:
        return {"outcome": "stale", "t_req": t_req}
    return {"outcome": "late" if late else "ok", "t_req": t_req, "t_recv": t_recv,
            "kv_mono_ns": int(ack["kv_mono_ns"]), "tm": decode_telemetry(ack, t_req, t_recv)}


def dist(values):
    v = sorted(x for x in values if x is not None)
    if not v:
        return None
    n = len(v)

    def q(p):
        return v[min(n - 1, int(p * n))]
    return {"n": n, "min": v[0], "p50": q(0.5), "p90": q(0.9), "p99": q(0.99), "max": v[-1],
            "mean": sum(v) / n}


def compact(tm):
    b = tm.blocks
    out = {"avail": sorted(tm.avail), "rtt_ms": tm.rtt_ms, "ages": dict(tm.age_upper_ms),
           "kv_boot_id": tm.kv_boot_id}
    g = b.get("geometry") or {}
    last = g.get("last") or {}
    out["geometry"] = {"served": g.get("served"), "frame_id": last.get("frame_id"),
                       "status": last.get("status"), "age_kv_ms": last.get("age_kv_ms"),
                       "pl_cycles": last.get("pl_cycles")}
    r = b.get("roi") or {}
    if r.get("available"):
        rl = r.get("last") or {}
        out["roi"] = {"core": r.get("core"), "served": r.get("served"),
                      "frame_id": rl.get("frame_id"), "roi_status": rl.get("roi_status"),
                      "rows": rl.get("rows"), "verdicts": rl.get("verdict_counts"),
                      "times_ms": rl.get("times_ms"), "age_kv_ms": rl.get("age_kv_ms")}
    else:
        out["roi"] = {"available": False, "reason": r.get("reason")}
    p = b.get("permit") or {}
    if p.get("available"):
        out["permit"] = {k: p.get(k) for k in ("status", "status_bits", "last_cause",
                                               "frames_accepted", "frames_rejected",
                                               "revoke_count", "last_frame_id", "age_ms",
                                               "since_done_ms", "timeout_ms")}
    else:
        out["permit"] = {"available": False, "reason": p.get("reason")}
    e = b.get("encoder") or {}
    if e.get("available"):
        out["encoder"] = {k: e.get(k) for k in ("status", "link_ok", "update_count",
                                                "frame_age_ms", "dist_center_m",
                                                "v_center_mm_s", "counters_named")}
    else:
        out["encoder"] = {"available": False, "reason": e.get("reason")}
    a = b.get("ar") or {}
    out["ar"] = {"available": bool(a.get("available")), "reason": a.get("reason")}
    s = b.get("estop") or {}
    out["estop"] = {k: s.get(k) for k in ("latched", "epoch", "force_stop_written",
                                          "pl_applied", "last_result")}
    bd = b.get("board") or {}
    out["board"] = {k: bd.get(k) for k in ("build_id", "pl_clk_hz", "build_id_reason")}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", required=True, help="HOST:PORT of kv_server.py")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--timeout-s", type=float, default=0.2)
    ap.add_argument("--late-wait-s", type=float, default=1.0,
                    help="after a timeout keep listening this long for the late answer")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    host, port = a.server.rsplit(":", 1)
    client = KvTelemetryClient((host, int(port)), timeout_s=a.timeout_s, hello_timeout_s=2.0)
    period = 1e9 / a.hz
    n = int(round(a.seconds * a.hz))
    counts = {"sent": 0, "ok": 0, "late": 0, "timeout": 0, "lost": 0, "stale": 0, "error": 0}
    offsets, lates = [], []
    rtts, ages, avail_sets, first, last = [], {}, {}, None, None
    t0 = now_ns()
    with open(a.out + "l", "w") as log:
        for i in range(n):
            due = t0 + int(i * period)
            wait = (due - now_ns()) / 1e9
            if wait > 0:
                time.sleep(wait)
            counts["sent"] += 1
            rec = {"i": i, "t_ms": (now_ns() - t0) / 1e6}
            r = poll_once(client, a.timeout_s, a.late_wait_s)
            counts[r["outcome"]] += 1
            rec["outcome"] = r["outcome"]
            if r["outcome"] == "ok":
                offsets.append(r["kv_mono_ns"] - (r["t_req"] + r["t_recv"]) // 2)
            if r["outcome"] == "late":
                off = offsets[-1] if offsets else None
                lates.append({"i": i, "arrival_ms": (r["t_recv"] - r["t_req"]) / 1e6,
                              "kv_answer_after_req_ms": None if off is None else
                              (r["kv_mono_ns"] - off - r["t_req"]) / 1e6})
                rec.update(lates[-1])
            if r["outcome"] == "ok":
                tm = r["tm"]
                rec.update(compact(tm))
                rtts.append(tm.rtt_ms)
                for k, v in tm.age_upper_ms.items():
                    ages.setdefault(k, []).append(v)
                key = ",".join(sorted(tm.avail))
                avail_sets[key] = avail_sets.get(key, 0) + 1
                first = first or rec
                last = rec
            log.write(json.dumps(rec, default=str) + "\n")
    lost = counts["sent"] - counts["ok"]     # late answers count as lost for the Nano
    report = {"server": a.server, "hz": a.hz, "seconds": a.seconds, "timeout_s": a.timeout_s,
              "elapsed_s": (now_ns() - t0) / 1e9, "counts": counts,
              "loss_rate": lost / counts["sent"] if counts["sent"] else None,
              "late_answers": lates,
              "offset_spread_ms": (max(offsets) - min(offsets)) / 1e6 if offsets else None,
              "rtt_ms": dist(rtts), "age_upper_ms": {k: dist(v) for k, v in ages.items()},
              "avail_sets": avail_sets, "client_counts": dict(client.counts),
              "first": first, "last": last}
    client.close()
    with open(a.out, "w") as fp:
        json.dump(report, fp, indent=1, sort_keys=True, default=str)
    print(json.dumps({"counts": counts, "loss_rate": report["loss_rate"],
                      "rtt_p50": (report["rtt_ms"] or {}).get("p50"),
                      "rtt_p99": (report["rtt_ms"] or {}).get("p99"),
                      "avail_sets": avail_sets}))


if __name__ == "__main__":
    main()
