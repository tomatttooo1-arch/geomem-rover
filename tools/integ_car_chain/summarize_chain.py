#!/usr/bin/env python3
"""One JSON with every number of the integration board smoke (docs/design/INTEG_CAR_CHAIN_20261011.md §3).

Usage: summarize_chain.py --nano <pulled nano runs dir> --kv <pulled kv runs dir> --tag HHMM

Reads (all optional, missing parts are reported as missing):
  nano: chain_<T>/{rig/summary.json, rig/frames.jsonl, watch/{watch.jsonl,estop.json,final.json},
        pc_stats.json, pc.fpgarec, hub_stats.json, cpu_hub_pc.json, rc.txt}, integ_sigint_<T>/{rc.txt,
        events.jsonl, stop_watch.jsonl}, aroff60_<T>/summary.json
  kv:   srv_ar_<T>/{summary.json, ar_helper_events.jsonl, server.err}, srv_aroff_<T>/summary.json
Stamps compared with each other all come from the Nano CLOCK_MONOTONIC_RAW (host.fpga_clock): the
permit watcher rows and the e-stop record. The PC recording's own receive stamps (CLOCK_MONOTONIC)
are only used for intervals between its own frames.
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def pct(v, q):
    v = sorted(x for x in v if x is not None)
    if not v:
        return None
    k = (len(v) - 1) * q / 100.0
    lo, hi = int(k), min(int(k) + 1, len(v) - 1)
    return round(v[lo] + (v[hi] - v[lo]) * (k - lo), 3)


def dist(v):
    v = [float(x) for x in v if x is not None]
    if not v:
        return None
    return {"n": len(v), "median": pct(v, 50), "p95": pct(v, 95), "p99_9": pct(v, 99.9),
            "max": round(max(v), 3), "min": round(min(v), 3)}


def load(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def jsonl(path):
    try:
        with open(path) as f:
            return [json.loads(x) for x in f if x.strip()]
    except OSError:
        return []


def rig_block(s):
    if not s:
        return None
    cc = s.get("client_counts") or {}
    bs = s.get("bridge_status") or {}
    ba = bs.get("age_ms") or {}
    cm = s.get("costmap") or {}
    st = s.get("stage_ms") or {}
    return {"processed": s.get("processed"), "fused": s.get("fused"),
            "frame_age_ms": s.get("frame_age_ms"), "frame_age_over_200": s.get("frame_age_over_200"),
            "timeouts": cc.get("timeout", 0), "client_counts": cc, "expired": (s.get("expired") or {}).get("n"),
            "expired_detail": s.get("expired"), "results": s.get("results"),
            "map_status": {"n": bs.get("n"), "over_200": ba.get("over_200"),
                           "over_200_ratio": (round(ba["over_200"] / bs["n"], 5)
                                              if bs.get("n") and ba.get("over_200") is not None else None),
                           "age_ms": ba, "reasons": bs.get("reasons")},
            "costmap": {"received": cm.get("received"), "count": cm.get("count"),
                        "geometry_frames_seen": cm.get("geometry_frames_seen"),
                        "grid_msgs": cm.get("grid_msgs"), "has_lethal_100": cm.get("has_lethal_100")},
            "kv_core_device_ms": st.get("kv_core_device_ms"), "rtt_ms": st.get("rtt_ms"),
            "geometry_ms": st.get("geometry_ms"), "roi": s.get("roi"), "kv": s.get("kv"),
            "cpu": s.get("cpu"), "producer_counts": s.get("producer_counts"), "missing": s.get("missing")}


def first(rows, t, pred):
    for r in rows:
        if r["t_ns"] >= t and r.get("outcome") == "ok" and pred(r):
            return r
    return None


def permit_block(watch, est, fin):
    ok = [r for r in watch if r.get("outcome") == "ok" and r.get("permit") is not None]
    out = {"rows": len(watch), "ok_rows": len(ok),
           "outcomes": {}, "telem_rtt_ms": dist(r.get("rtt_ms") for r in ok)}
    for r in watch:
        out["outcomes"][r.get("outcome")] = out["outcomes"].get(r.get("outcome"), 0) + 1
    if not ok:
        return out
    grant = next((r for r in ok if r["permit"]), None)
    acc_change = [b for a, b in zip(ok, ok[1:]) if b["accepted"] != a["accepted"]]
    last_acc = acc_change[-1] if acc_change else None
    est = est or {}
    t_es, t_rel = est.get("t_estop_send_ns"), est.get("t_release_result_ns")
    restore = first(ok, t_rel, lambda r: r["permit"]) if t_rel else None
    if grant and last_acc:
        run = [r for r in ok if grant["t_ns"] <= r["t_ns"] <= last_acc["t_ns"]]
        zero = [r for r in run if not r["permit"]]
        excl = [r for r in zero if not (t_es and restore and t_es <= r["t_ns"] < restore["t_ns"])]
        out["running_window_ns"] = [grant["t_ns"], last_acc["t_ns"]]
        out["running"] = {
                          "span_s": round((last_acc["t_ns"] - grant["t_ns"]) / 1e9, 3),
                          "samples": len(run), "permit0_samples_all": len(zero),
                          "permit0_samples_outside_estop_window": len(excl),
                          "permit0_outside_detail": excl[:10],
                          "accepted_first_last": [grant["accepted"], last_acc["accepted"]],
                          "rejected_first_last": [grant["rejected"], last_acc["rejected"]],
                          "revokes_first_last": [grant["revokes"], last_acc["revokes"]]}
        drop = first(ok, last_acc["t_ns"], lambda r: not r["permit"])
        out["after_stop_telem"] = {
            "last_accept_change_to_first_permit0_ms_upper": (round((drop["t_ns"] - last_acc["t_ns"]) / 1e6, 2)
                                                             if drop else None),
            "first_permit0_cause": drop.get("cause") if drop else None}
    if t_es:
        z = first(ok, t_es, lambda r: not r["permit"])
        before = [r for r in ok if r["t_ns"] < t_es][-1:] or [None]
        res = est.get("estop_result") or {}
        out["estop"] = {
            "send_to_hub_ack_ms": _ms(est.get("t_estop_ack_ns"), t_es),
            "send_to_result_ms": _ms(est.get("t_estop_result_ns"), t_es),
            "hub_to_kv_latency_ms": res.get("latency_ms"), "result": res,
            "send_to_first_permit0_sample_ms": _ms(z["t_ns"], t_es) if z else None,
            "first_permit0_sample": z, "sample_before_send": before[0],
            "release_send_to_result_ms": _ms(est.get("t_release_result_ns"), est.get("t_release_send_ns")),
            "release_result": est.get("release_result"), "error": est.get("error")}
        if restore:
            prev = [r for r in ok if r["t_ns"] < restore["t_ns"]][-1]
            gap = [r for r in ok if t_rel <= r["t_ns"] < restore["t_ns"]]
            out["estop"]["restore"] = {
                "release_result_to_permit1_ms": _ms(restore["t_ns"], t_rel),
                "permit0_samples_after_release": len(gap),
                "accepted_prev_to_restore": [prev["accepted"], restore["accepted"]],
                "restore_sample": restore,
                "latched_samples_estop_bit": sum(1 for r in ok if t_es <= r["t_ns"] < t_rel and r.get("estop_bit"))}
    p = (fin or {}).get("permit") or {}
    b = (fin or {}).get("board") or {}
    clk = b.get("pl_clk_hz") or 100e6
    if p.get("last_revoke_ts") is not None and p.get("last_done_ts") is not None:
        out["final_regs"] = {"last_cause": p.get("last_cause"),
                             "revoke_after_last_done_ms_pl": round((p["last_revoke_ts"] - p["last_done_ts"]) / clk * 1e3, 4),
                             "revoke_cycles": p["last_revoke_ts"] - p["last_done_ts"],
                             "timeout_cycles": p.get("timeout_cycles"), "status": p.get("status"),
                             "frames_accepted": p.get("frames_accepted"),
                             "frames_rejected": p.get("frames_rejected"), "revoke_count": p.get("revoke_count"),
                             "build_id": b.get("build_id"), "clk_hz": clk}
    return out


def _ms(a, b):
    return None if a is None or b is None else round((a - b) / 1e6, 3)


def pc_block(d, window=None):
    st = load(os.path.join(d, "pc_stats.json"))
    out = {"stats": st}
    if st and st.get("wall_s"):
        out["rates_hz"] = {k: round(v / st["wall_s"], 3) for k, v in (st.get("frames") or {}).items()}
    try:
        from pc import fpgac
        _, rows = fpgac.read_fpgarec(os.path.join(d, "pc.fpgarec"))
    except Exception as exc:
        out["fpgarec_error"] = "%s: %s" % (type(exc).__name__, exc)
        return out
    states = []
    for pc_ns, raw in rows:
        f = fpgac.decode_one(raw)
        if f.kind == fpgac.KIND["STATE"]:
            states.append((pc_ns, f.obj()))
    out["state_frames"] = len(states)
    if window:                                  # Nano RAW: STATE nano_mono_ns vs the permit watcher
        states = [(t, o) for t, o in states if window[0] <= (o.get("nano_mono_ns") or 0) <= window[1]]
        out["state_frames_in_run_window"] = len(states)
        out["run_window_s"] = round((window[1] - window[0]) / 1e9, 3)
    if len(states) > 1:
        span = (states[-1][0] - states[0][0]) / 1e9
        out["state_rate_hz_in_record"] = round((len(states) - 1) / span, 3) if span > 0 else None

    def g(o, *keys):
        for k in keys:
            if not isinstance(o, dict):
                return None
            o = o.get(k)
        return o
    out["state_ages_ms"] = {
        "kv_group_age_ms": dist(g(o, "kv", "age_ms") for _, o in states),
        "kv_geometry_age_ms": dist(g(o, "kv", "geometry", "age_ms") for _, o in states),
        "memory_age_ms": dist(g(o, "memory", "age_ms") for _, o in states),
        "map_age_ms": dist(g(o, "map", "age_ms") for _, o in states)}
    out["state_kv_link_ok"] = sum(1 for _, o in states if g(o, "kv", "link_ok"))
    out["state_srcs"] = {}
    for grp in fpgac.STATE_GROUPS:
        c = {}
        for _, o in states:
            s = g(o, grp, "src")
            c[s] = c.get(s, 0) + 1
        out["state_srcs"][grp] = c
    out["state_permit_seen"] = sorted(set(str(g(o, "kv", "permit", "permit")) for _, o in states))
    out["state_estop_latched_frames"] = sum(1 for _, o in states if g(o, "estop", "latched"))
    out["state_ar"] = [g(states[len(states) // 2][1], "kv", "ar")] if states else None
    return out


def ar_block(evpath):
    ev = jsonl(evpath)
    if not ev:
        return None
    fr = [e for e in ev if e.get("ev") == "frame"]
    fs = [e for e in ev if e.get("ev") == "failsafe"]
    return {"frames": len(fr), "failsafe": len(fs), "not_done": sum(1 for e in fr if not e.get("done")),
            "not_flipped": sum(1 for e in fr if not e.get("flipped")),
            "ar_ms": dist(e["ar_ns"] / 1e6 for e in fr if e.get("done")),
            "interval_ms": dist(e["interval_ms"] for e in fr if e.get("interval_ms", -1) >= 0),
            "age_ms": dist(e.get("age_ms") for e in fr),
            "frame_permit0": sum(1 for e in fr if e.get("permit_before") == 0),
            "ready": next((e for e in ev if e.get("ev") == "ready"), None),
            "exit": [e for e in ev if e.get("ev") in ("exit", "eof", "error")]}


def kv_block(d):
    s = load(os.path.join(d, "summary.json"))
    if not s:
        return None
    tc = s.get("total_counts") or {}
    return {"permit": {k: v for k, v in tc.items() if k.startswith("permit")},
            "bad": {k: v for k, v in tc.items() if k.startswith(("bad_", "core_error", "roi_status"))},
            "served": tc.get("served"), "slot_superseded": tc.get("slot_superseded"),
            "ar": s.get("ar"), "ar_desktop_enable": s.get("ar_desktop_enable"),
            "ar_telem_end": s.get("ar_telem_end")}


def sigint_block(d):
    rc = None
    try:
        rc = open(os.path.join(d, "rc.txt")).read().split()
    except OSError:
        pass
    ev = jsonl(os.path.join(d, "events.jsonl"))
    sw = jsonl(os.path.join(d, "stop_watch.jsonl"))
    return {"rc": rc, "events": len(ev), "frame_processed": sum(1 for e in ev if e.get("event") == "frame_processed"),
            "last_event": ev[-1].get("event") if ev else None, "stop_watch_rows": len(sw),
            "stop_watch_tail": sw[-3:]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nano", required=True)
    ap.add_argument("--kv", required=True)
    ap.add_argument("--tag", required=True)
    a = ap.parse_args()
    T = a.tag
    c = os.path.join(a.nano, "chain_" + T)
    out = {"tag": T}
    try:
        out["chain_rc"] = open(os.path.join(c, "rc.txt")).read().split()
    except OSError:
        out["chain_rc"] = None
    out["chain"] = rig_block(load(os.path.join(c, "rig", "summary.json")))
    out["permit"] = permit_block(jsonl(os.path.join(c, "watch", "watch.jsonl")),
                                 load(os.path.join(c, "watch", "estop.json")),
                                 load(os.path.join(c, "watch", "final.json")))
    run = out["permit"].get("running_window_ns")
    out["pc_whole"] = pc_block(c)
    out["pc"] = pc_block(c, run)
    hs = load(os.path.join(c, "hub_stats.json"))
    out["hub"] = {"stats_counts": (hs or {}).get("counts"), "ticks": (hs or {}).get("ticks"),
                  "tick_gap_ms": (hs or {}).get("tick_gap_ms"), "effective_sndbuf": (hs or {}).get("effective_sndbuf"),
                  "cpu_pct_one_core_whole_life": (round(sum(hs["cpu_times_s"]) / hs["wall_s"] * 100, 2)
                                                  if hs and hs.get("wall_s") else None),
                  "window": load(os.path.join(c, "cpu_hub_pc.json"))}
    out["ar"] = ar_block(os.path.join(a.kv, "srv_ar_" + T, "ar_helper_events.jsonl"))
    out["kv_ar"] = kv_block(os.path.join(a.kv, "srv_ar_" + T))
    out["kv_aroff"] = kv_block(os.path.join(a.kv, "srv_aroff_" + T))
    out["aroff_rig"] = rig_block(load(os.path.join(a.nano, "aroff60_" + T, "summary.json")))
    out["sigint"] = sigint_block(os.path.join(a.nano, "integ_sigint_" + T))
    print(json.dumps(out, indent=1, sort_keys=True, default=str))


if __name__ == "__main__":
    main()
