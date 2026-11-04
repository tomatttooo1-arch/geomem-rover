#!/usr/bin/env python3
"""One JSON with every number of the whole-chain board run v2 (docs/design/INTEG_CAR_CHAIN_20261011.md §6).

Usage: summarize_chain_v2.py --chain <pulled nano runs>/chain_<T> --kv <pulled kv runs>/srv_ar_<T>

Reuses summarize_chain.py (v1) blocks for the rig, permit/e-stop and PC; adds:
  * start -> first permit=1 (from the hub start, the rig start and the replay's first depth publish;
    all Nano CLOCK_MONOTONIC_RAW)
  * every permit=0 watcher sample from the watcher start to the producer's last accepted frame,
    classified: startup (before the first grant), estop (e-stop sent .. permit back), producer gap
    (inside a producer geometry_gap .. geometry_resumed window, with its phase/reason), other
  * per-process CPU over the rig's measurement window from the system sampler (sys.csv)
  * runtime (kv_topic) and hub SIGINT: rc, last event, hub stats written, abort text in hub.err
  * kv group age as the PC saw it, with the blocks the hub excluded
"""
import argparse
import collections
import csv
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.integ_car_chain.summarize_chain import (dist, jsonl, kv_block, load, pc_block,  # noqa: E402
                                                   permit_block, rig_block)


def marks(d):
    out = {}
    try:
        for line in open(os.path.join(d, "marks.txt")):
            for kv in line.split():
                k, v = kv.split("=", 1)
                out[k] = float(v) if "." in v else int(v)
    except OSError:
        pass
    return out


def rc_lines(d):
    out = {}
    try:
        for line in open(os.path.join(d, "rc.txt")):
            if "=" in line and " " not in line.strip():
                k, v = line.strip().split("=", 1)
                out[k] = v
            elif line.strip():
                out.setdefault("notes", []).append(line.strip())
    except OSError:
        pass
    return out


def first_publish(rig):
    try:
        with open(os.path.join(rig, "replay.csv"), newline="") as f:
            return min(int(r["publish_raw_ns"]) for r in csv.DictReader(f))
    except (OSError, ValueError):
        return None


def gap_windows(rig):
    """[(start_raw, end_raw or None, phase, reason)] from the producer's events."""
    out, cur = [], None
    for r in jsonl(os.path.join(rig, "frames.jsonl")):
        ev = r.get("event")
        if ev == "geometry_gap":
            cur = [r["raw_ns"], None, r.get("phase"), r.get("reason")]
            out.append(cur)
        elif ev in ("geometry_resumed", "geometry_ready") and cur is not None:
            cur[1] = r["raw_ns"]
            cur = None
    return out


def permit_zero_classes(watch, est, gaps, last_accept_ns):
    ok = [r for r in watch if r.get("outcome") == "ok" and r.get("permit") is not None]
    grant = next((r for r in ok if r["permit"]), None)
    est = est or {}
    t_es = est.get("t_estop_send_ns")
    t_rel = est.get("t_release_result_ns")
    restore = next((r for r in ok if t_rel and r["t_ns"] >= t_rel and r["permit"]), None)
    cls = collections.Counter()
    detail = collections.defaultdict(list)
    for r in ok:
        if r["permit"] or (last_accept_ns is not None and r["t_ns"] > last_accept_ns):
            continue
        t = r["t_ns"]
        if grant is None or t < grant["t_ns"]:
            k = "startup_not_ready"
        elif t_es and t >= t_es and (restore is None or t < restore["t_ns"]):
            k = "estop"
        else:
            k = "other"
            for g0, g1, phase, reason in gaps:
                # the permit lapses 200 ms after the last good frame; a gap that started up to
                # 300 ms before the sample and had not ended 300 ms before it explains it
                if g0 - 300e6 <= t and (g1 is None or t <= g1 + 300e6):
                    k = "producer_gap_%s:%s" % (phase, reason)
                    break
        cls[k] += 1
        if len(detail[k]) < 5:
            detail[k].append({"t_ns": t, "cause": r.get("cause"), "last_cause": r.get("last_cause"),
                              "accepted": r.get("accepted")})
    return {"samples_ok": len(ok), "permit0": dict(cls), "examples": dict(detail),
            "first_grant_t_ns": grant["t_ns"] if grant else None}


def cpu_window(rig):
    cpu = load(os.path.join(rig, "cpu.json")) or {}
    t0, t1 = cpu.get("t0_wall"), cpu.get("t1_wall")
    try:
        with open(os.path.join(rig, "sys.csv"), newline="") as f:
            rows = list(csv.DictReader(f))
    except OSError:
        return {"rig_cpu_json": cpu}
    if t0 is None or not rows:
        return {"rig_cpu_json": cpu}
    win = [r for r in rows if t0 <= float(r["wall"]) <= t1]
    if len(win) < 2:
        return {"rig_cpu_json": cpu}
    a, b = win[0], win[-1]
    wall = float(b["wall"]) - float(a["wall"])
    hz = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
    procs = {}
    skip = {"raw_ns", "wall", "cpu_busy", "cpu_total", "load1", "InDatagrams", "NoPorts", "InErrors",
            "RcvbufErrors", "SndbufErrors"}
    for k in a:
        if k in skip or k.startswith("freq_khz_"):     # v7: per-CPU frequency columns, not processes
            continue
        try:
            procs[k] = round((int(b[k]) - int(a[k])) / hz / wall * 100, 2)
        except (TypeError, ValueError):
            procs[k] = None
    busy = (int(b["cpu_busy"]) - int(a["cpu_busy"])) / max(1, int(b["cpu_total"]) - int(a["cpu_total"]))
    # the replay stands in for the camera driver (not a car component): machine CPU also given without it
    rep = procs.get("replay")
    excl = None if rep is None else round(busy * 6 - rep / 100.0, 3)
    return {"window_s": round(wall, 2), "pct_one_core": procs, "machine_busy_pct_all_cores": round(busy * 100, 2),
            "machine_busy_cores_of_6": round(busy * 6, 3),
            "machine_busy_cores_of_6_excl_replay": excl,
            "machine_busy_pct_all_cores_excl_replay": None if excl is None else round(excl / 6 * 100, 2),
            "rig_cpu_json": cpu,
            "udp_rcvbuf_errors_in_window": int(b["RcvbufErrors"]) - int(a["RcvbufErrors"])
            if a.get("RcvbufErrors") and b.get("RcvbufErrors") else None}


def runtime_block(d):
    ev = jsonl(os.path.join(d, "runtime", "events.jsonl"))
    if not ev:
        return None
    c = collections.Counter(e.get("event") for e in ev)
    fp = [e for e in ev if e.get("event") == "frame_processed"]
    res = collections.Counter(((e.get("geometry") or {}).get("result") or "none").split(":")[0] + ":" +
                              (((e.get("geometry") or {}).get("result") or "").split(":") + [""])[1]
                              for e in fp)
    close = next((e for e in reversed(ev) if e.get("event") == "geometry_close"), None)
    sw = jsonl(os.path.join(d, "runtime", "stop_watch.jsonl"))
    return {"events": dict(c), "last_event": ev[-1].get("event"), "frame_processed": len(fp),
            "geometry_results": dict(res), "geometry_close_counts": (close or {}).get("counts"),
            "frame_age_ms": dist((e.get("geometry") or {}).get("frame_age_ms") for e in fp),
            "stop_watch_tail": sw[-3:]}


def kv_age_block(c):
    try:
        from pc import fpgac
        _, rows = fpgac.read_fpgarec(os.path.join(c, "pc.fpgarec"))
    except Exception as exc:
        return {"error": str(exc)}
    excl = collections.Counter()
    basis = collections.Counter()
    blocks = collections.Counter()
    n = 0
    for _, raw in rows:
        f = fpgac.decode_one(raw)
        if f.kind != fpgac.KIND["STATE"]:
            continue
        o = f.obj()
        kv = o.get("kv") or {}
        n += 1
        for k, v in (kv.get("age_excluded") or {}).items():
            excl["%s:%s" % (k, v)] += 1
        basis[",".join(kv.get("age_basis") or [])] += 1
        for k, v in fpgac.kv_blocks(o).items():
            blocks["%s:%s" % (k, v["state"])] += 1
    return {"state_frames": n, "age_excluded": dict(excl), "age_basis": dict(basis), "kv_blocks": dict(blocks)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--chain", required=True)
    ap.add_argument("--kv", required=True)
    a = ap.parse_args()
    c, rig = a.chain, os.path.join(a.chain, "rig")
    out = {"chain": a.chain, "rc": rc_lines(c), "marks": marks(c)}
    rs = load(os.path.join(rig, "summary.json"))
    out["rig"] = rig_block(rs)
    out["rig_extra"] = {k: (rs or {}).get(k) for k in ("gaps", "gap_events", "tf_missing_rows", "publish_ms",
                                                       "labels_sent_frames")}
    watch = jsonl(os.path.join(c, "watch", "watch.jsonl"))
    est = load(os.path.join(c, "watch", "estop.json"))
    out["permit"] = permit_block(watch, est, load(os.path.join(c, "watch", "final.json")))
    run = out["permit"].get("running_window_ns")
    last_acc = run[1] if run else None
    pz = permit_zero_classes(watch, est, gap_windows(rig), last_acc)
    out["permit_zero"] = pz
    m = out["marks"]
    fp = first_publish(rig)
    g = pz.get("first_grant_t_ns")
    out["first_permit"] = {
        "from_hub_start_s": round((g - m["start_raw_ns"]) / 1e9, 3) if g and m.get("start_raw_ns") else None,
        "from_rig_start_s": round((g - m["rig_start_raw_ns"]) / 1e9, 3) if g and m.get("rig_start_raw_ns") else None,
        "from_first_depth_publish_s": round((g - fp) / 1e9, 3) if g and fp else None}
    out["pc"] = pc_block(c, run)
    out["kv_age"] = kv_age_block(c)
    hs = load(os.path.join(c, "hub_stats.json"))
    herr = ""
    try:
        herr = open(os.path.join(c, "hub.err")).read()
    except OSError:
        pass
    out["hub"] = {"stats_written": hs is not None, "counts": (hs or {}).get("counts"),
                  "tick_gap_ms": (hs or {}).get("tick_gap_ms"),
                  "abort_text": "terminate called" in herr, "err_tail": herr[-300:]}
    out["runtime"] = runtime_block(c)
    out["cpu"] = cpu_window(rig)
    out["kv_ar"] = kv_block(a.kv)
    print(json.dumps(out, indent=1, sort_keys=True, default=str))


if __name__ == "__main__":
    main()
