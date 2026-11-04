#!/usr/bin/env python3
"""T1 (2026-10-16) readout: judges §6 (a)(b)(c) per runs/hbgap-t1-20261016/PREREG.md from the pulled session.

  t1_readout.py SESSION_DIR --out OUT.json        (SESSION_DIR = runs/hbgap-t1-20261016/log_MMDD_HHMM)
Uses tools/hb_gap_dig/hb_gap_dig.py (same constants: late > 80 ms over the +-50-row floor, 60 s buckets, > 5 late/min).
Python 3.8, standard library only.
"""
import argparse
import bisect
import collections
import csv
import glob
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import hb_gap_dig as H  # noqa: E402


def cfg(path):
    d = {}
    for ln in open(path):
        if "=" in ln:
            k, v = ln.strip().split("=", 1)
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


def wall_to_raw(samples):
    pts = [(s["wall"], s["raw_ns"]) for s in samples if "raw_ns" in s and "wall" in s]
    ws = [p[0] for p in pts]

    def f(w):
        i = min(max(1, bisect.bisect_left(ws, w)), len(pts) - 1)
        (w0, r0), (w1, r1) = pts[i - 1], pts[i]
        return r0 + (w - w0) * (r1 - r0) / (w1 - w0)
    return f


def at(samples, raw, key):
    """value of key(sample) at the sample nearest to raw."""
    rs = [s["raw_ns"] for s in samples]
    i = bisect.bisect_left(rs, raw)
    c = [j for j in (i - 1, i) if 0 <= j < len(samples)]
    j = min(c, key=lambda j: abs(rs[j] - raw))
    return key(samples[j]), (rs[j] - raw) / 1e9


def gap_counts(run, a, b):
    comps = [r["C"] for r in run.ran if a <= r["C"] <= b]
    g = [(y - x) / 1e6 for x, y in zip(comps, comps[1:])]
    mins = (b - a) / 6e10
    return {"minutes": round(mins, 2), "completions": len(comps),
            "gt200": sum(x > 200 for x in g), "gt337": sum(x > 337 for x in g),
            "gt200_per_min": round(sum(x > 200 for x in g) / mins, 2) if mins > 0 else None,
            "gt337_per_min": round(sum(x > 337 for x in g) / mins, 2) if mins > 0 else None,
            "max_ms": round(max(g), 1) if g else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    S = a.session
    c = cfg(os.path.join(S, "config.txt"))
    name = c["name"]
    RUN = os.path.join(S, "nano", "runs", name)
    SIDE = os.path.join(S, "nano", "runs", "t1side")
    samples = [s for s in load_jsonl(os.path.join(SIDE, "samples.jsonl")) if "snmp" in s]
    meta = [s for s in load_jsonl(os.path.join(SIDE, "samples.jsonl")) if s.get("event") == "start"][0]
    slow = load_jsonl(os.path.join(SIDE, "slow.jsonl"))
    w2r = wall_to_raw(samples)
    t_ready = w2r(float(c["ready_wall"]))
    t_clean_end = w2r(float(c["clean_end_wall"])) if "clean_end_wall" in c else None
    t_side0 = w2r(float(c["side_start_wall"])) if "side_start_wall" in c else None
    t_side1 = w2r(float(c["side_end_wall"])) if "side_end_wall" in c else None
    frames = os.path.join(RUN, "frames.jsonl")
    run = H.Run("t1", frames)
    summ = H.summarise_run(run, sims=False)
    res = {"session": S, "name": name, "hb_gap_dig_full": summ}
    t0 = run.t0
    res["t0_first_completion_after_ready_s"] = round((t0 - t_ready) / 1e9, 2)
    end_clean = t_clean_end or run.ran[-1]["C"]
    # ---- per-window rates (same口径 as §2)
    res["windows"] = {"clean": gap_counts(run, t0, end_clean)}
    if t_side0 and t_side1:
        res["windows"]["side"] = gap_counts(run, t_side0, t_side1)
    nb = len(run.on)
    bk = [{"k": k, "t_s": k * 60, "late": run.late_per_bucket[k], "on": run.on[k],
           "clean": t0 + (k + 1) * 60e9 <= end_clean} for k in range(nb)]
    res["buckets"] = bk
    clean_b = [b for b in bk if b["clean"]]
    res["clean_on_fraction"] = round(sum(b["on"] for b in clean_b) / max(1, len(clean_b)), 3)
    if t_side0 and t_side1:
        sb = [b for b in bk if t0 + b["k"] * 60e9 >= t_side0 and t0 + (b["k"] + 1) * 60e9 <= t_side1]
        res["side_on_fraction"] = round(sum(b["on"] for b in sb) / max(1, len(sb)), 3)
    # ---- (a)
    hits = [b for b in clean_b if b["t_s"] >= 180 and b["late"] > 5]
    res["a"] = {"rule": "bucket start >= 180 s, inside the clean window, late > 5/min",
                "hit_buckets": [b["t_s"] for b in hits], "verdict": bool(hits),
                "first_on_s": run.first_on_s}
    # ---- (b)
    segs, cur = [], None
    for b in clean_b:
        if b["on"]:
            if cur and cur[1] == b["k"]:
                cur[1] = b["k"] + 1
            else:
                cur = [b["k"], b["k"] + 1]
                segs.append(cur)
    prod_inodes = set()
    for s in slow:
        for p in s.get("procs", {}).get("producer", []):
            prod_inodes.update(p.get("sockets", []))

    def rcv(s):
        return s["snmp"]["Udp"]["RcvbufErrors"]

    def inerr(s):
        return s["snmp"]["Udp"]["InErrors"]

    def indg(s):
        return s["snmp"]["Udp"]["InDatagrams"]

    def pdrops(s):
        return sum(x["drops"] for x in s["udp"] if x["inode"] in prod_inodes)
    bseg = []
    R = G = M = PD = 0
    for k0, k1 in segs:
        ra, rb = t0 + k0 * 60e9, t0 + k1 * 60e9
        (r0, e0), (r1, e1) = at(samples, ra, rcv), at(samples, rb, rcv)
        (d0, _), (d1, _) = at(samples, ra, pdrops), at(samples, rb, pdrops)
        (i0, _), (i1, _) = at(samples, ra, indg), at(samples, rb, indg)
        g = [e for e in run.gaps_in if ra <= e["raw_ns"] < rb]
        seg = {"t_s": [k0 * 60, k1 * 60], "rcvbuf_errors": r1 - r0, "input_gap_events": len(g),
               "missed_frames": sum(e["missed"] for e in g), "producer_socket_drops": d1 - d0,
               "udp_in_datagrams_per_s": round((i1 - i0) / ((rb - ra) / 1e9), 1),
               "sample_offset_s": [round(e0, 2), round(e1, 2)]}
        bseg.append(seg)
        R += seg["rcvbuf_errors"]
        G += seg["input_gap_events"]
        M += seg["missed_frames"]
        PD += seg["producer_socket_drops"]
    # whole clean window and off part, for context
    (rA, _), (rB, _) = at(samples, t0, rcv), at(samples, end_clean, rcv)
    (iA, _), (iB, _) = at(samples, t0, indg), at(samples, end_clean, indg)
    res["b"] = {"segments": bseg, "R_rcvbuf_on": R, "G_input_gap_on": G, "missed_on": M,
                "producer_socket_drops_on": PD,
                "clean_total": {"rcvbuf_errors": rB - rA,
                                "input_gap_events": sum(1 for e in run.gaps_in if t0 <= e["raw_ns"] <= end_clean),
                                "udp_in_datagrams_per_s": round((iB - iA) / ((end_clean - t0) / 1e9), 1)},
                "producer_socket_inodes": sorted(prod_inodes),
                "verdict": bool(G) and abs(R - G) <= 0.2 * G}
    # ---- (c)
    trips = {}
    for zt, tp in meta["trips"].items():
        vals = []
        for k, v in tp.items():
            t, typ = v.split("/")
            if typ in ("passive", "active"):
                vals.append(int(t))
        trips[zt] = sorted(vals)
    ev = []
    for p, q in zip(samples, samples[1:]):
        for zt, lv in trips.items():
            try:
                x, y = int(p["temp"][zt]), int(q["temp"][zt])
            except (ValueError, KeyError, TypeError):
                continue
            for L in lv:
                if (x - L) * (y - L) < 0 or (x < L <= y) or (y < L <= x):
                    ev.append((q["raw_ns"], "trip %s %d" % (zt, L)))
        if p.get("pwm1") != q.get("pwm1"):
            ev.append((q["raw_ns"], "pwm %s->%s" % (p.get("pwm1"), q.get("pwm1"))))
        for k in (q.get("oc") or {}):
            if (p.get("oc") or {}).get(k) != q["oc"][k]:
                ev.append((q["raw_ns"], "oc %s" % k))
    ev.sort()
    evt = [e[0] for e in ev]
    starts = [t0 + k0 * 60e9 for k0, _ in segs]

    def has_ev(a, b):
        i = bisect.bisect_left(evt, a)
        return i < len(evt) and evt[i] <= b
    st_hits = [{"t_s": round((s - t0) / 1e9), "event_within_60s": has_ev(s - 60e9, s + 60e9),
                "events": [e[1] for e in ev if s - 60e9 <= e[0] <= s + 60e9][:8]} for s in starts]
    # specificity: 120 s windows (step 30 s) fully inside clean OFF buckets
    off = [b for b in clean_b if not b["on"]]
    offset = set(b["k"] for b in off)
    wins = []
    for b in off:
        for st in range(0, 60, 30):
            a_ = t0 + b["k"] * 60e9 + st * 1e9
            ks = set(int((x - t0) / 60e9) for x in (a_, a_ + 119.9e9))
            if ks <= offset:
                wins.append(has_ev(a_, a_ + 120e9))
    spec = round(sum(wins) / len(wins), 3) if wins else None
    res["c"] = {"trips_used": trips, "n_events": len(ev), "event_kinds": dict(collections.Counter(e[1].split()[0] for e in ev)),
                "segment_starts": st_hits, "off_window_event_share": spec, "n_off_windows": len(wins),
                "verdict": len(starts) >= 3 and all(x["event_within_60s"] for x in st_hits)
                and (spec is None or spec < 0.5)}
    temps = collections.defaultdict(list)
    for s in samples:
        for zt, v in s["temp"].items():
            try:
                x = int(v) / 1000.0
            except ValueError:
                continue
            temps[zt].append(x)
    res["thermal"] = {zt: [min(v), max(v)] for zt, v in temps.items()}
    res["pwm_values"] = dict(collections.Counter(s.get("pwm1") for s in samples))
    res["rpm_range"] = [min(int(s["rpm"]) for s in samples), max(int(s["rpm"]) for s in samples)]
    # ---- UDP per-minute timeline (clean + side)
    tl = []
    slow_t = [e["start_raw_ns"] for e in run.ev if e.get("event") == "slow_call"]
    for k in range(nb):
        ra, rb = t0 + k * 60e9, t0 + (k + 1) * 60e9
        (r0, _), (r1, _) = at(samples, ra, rcv), at(samples, rb, rcv)
        (i0, _), (i1, _) = at(samples, ra, indg), at(samples, rb, indg)
        (d0, _), (d1, _) = at(samples, ra, pdrops), at(samples, rb, pdrops)
        g = sum(1 for e in run.gaps_in if ra <= e["raw_ns"] < rb)
        sc = sum(1 for x in slow_t if ra <= x < rb)
        (o0, _), (o1, _) = at(samples, ra, lambda s: s["netstat"]["IpExt"]["InOctets"]), at(samples, rb, lambda s: s["netstat"]["IpExt"]["InOctets"])
        tl.append([k * 60, int(run.on[k]), run.late_per_bucket[k], g, r1 - r0, round((i1 - i0) / 60.0), d1 - d0, sc,
                   round((o1 - o0) / 60.0 / 1e6, 3)])
    res["minute_timeline_cols"] = ["t_s", "on", "late", "input_gaps", "rcvbuf_err", "udp_in_per_s", "producer_sock_drops",
                                   "producer_slow_calls", "ip_in_MB_per_s"]
    res["graph_changes"] = [[round((e["raw_ns"] - t0) / 1e9, 1), e.get("added"), e.get("removed")]
                            for e in run.ev if e.get("event") == "graph_change"]
    res["minute_timeline"] = tl
    # ---- driver per-frame record
    drv = os.path.join(RUN, "drv_frames.csv")
    if os.path.exists(drv):
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "integ_car_chain", "cam_drop"))
        rows = list(csv.DictReader(open(drv, newline="")))
        dep = [r for r in rows if r.get("depth_global_ts_sec")]
        dev = [float(r["depth_device_ts_sec"]) for r in dep]
        glb = [float(r["depth_global_ts_sec"]) for r in dep]
        st = [b - x for x, b in zip(dev, dev[1:])]
        per = statistics.median(st)
        hw = [int(r["depth_hardware_frame_number"]) for r in dep if r.get("depth_hardware_frame_number")]
        idx = [int(r["depth_sdk_frame_index"]) for r in dep]
        pub = [r for r in dep if r.get("depth_arrival_to_publish_steady_us")]
        res["driver"] = {"depth_rows": len(dep), "period_ms": round(per * 1e3, 2),
                         "device_ts_gaps_gt_1p5": sum(1 for x in st if x > 1.5 * per),
                         "device_ts_max_step_ms": round(max(st) * 1e3, 1),
                         "hw_frame_missing": sum(b - x - 1 for x, b in zip(hw, hw[1:]) if b - x > 1) if len(hw) > 1 else None,
                         "sdk_index_missing": sum(b - x - 1 for x, b in zip(idx, idx[1:]) if b - x > 1),
                         "published": len(pub), "arrived_not_published": len(dep) - len(pub)}
        # stamps published by the driver inside the producer's input gaps (header stamp = global ts, us)
        gus = sorted(int(round(x * 1e6)) for x in glb)
        pst = set(int(round(r["depth_stamp_ns"] / 1e3)) for r in run.rows)
        gset = set(gus)
        matched = sum(1 for s in pst if s in gset)
        lost_pub = 0
        for e in run.gaps_in:
            lo = bisect.bisect_right(gus, int(round(e["prev_stamp_ns"] / 1e3)) + 2)
            hi = bisect.bisect_left(gus, int(round(e["stamp_ns"] / 1e3)) - 2)
            lost_pub += max(0, hi - lo)
        res["driver"]["producer_rows_matched_to_driver_global_ts"] = [matched, len(pst)]
        res["driver"]["driver_frames_inside_producer_input_gaps"] = lost_pub
        res["driver"]["producer_missed_frames_total"] = sum(e["missed"] for e in run.gaps_in)
    # ---- side receiver
    sd = os.path.join(RUN, "t1")
    if os.path.isdir(sd):
        sf = list(csv.DictReader(open(os.path.join(sd, "side_frames.csv"))))
        so = load_jsonl(os.path.join(sd, "side_over.jsonl"))
        stamps = [int(r["stamp_ns"]) for r in sf]
        rej = [r for r in run.rows if H.refuse_kind(r) == "over_range"]
        rej_side = [r for r in rej if t_side0 and t_side0 <= r["acquired_ns"] <= t_side1]
        sset = set(stamps)
        oset = {d["stamp_ns"]: d for d in so}
        joined = [oset.get(r["depth_stamp_ns"]) for r in rej_side]
        seen = sum(1 for r in rej_side if r["depth_stamp_ns"] in sset)
        jn = [d for d in joined if d]
        codes = collections.Counter()
        blk = [[0] * 4 for _ in range(4)]
        rowh = [0] * 20
        colh = [0] * 32
        for d in so:
            for v, n in d["raw_code_top"]:
                codes[v] += n
            for i in range(4):
                for j in range(4):
                    blk[i][j] += d["block4x4"][i][j]
            rowh = [x + y for x, y in zip(rowh, d["row_hist20"])]
            colh = [x + y for x, y in zip(colh, d["col_hist32"])]

        def qs(xs):
            xs = sorted(xs)
            return [xs[0], xs[len(xs) // 2], xs[int(0.9 * (len(xs) - 1))], xs[-1]] if xs else None
        res["side"] = {"frames_received": len(sf), "over_frames": len(so),
                       "receive_share_of_driver": None,
                       "producer_over_range_refusals_in_side_window": len(rej_side),
                       "of_which_seen_by_side": seen, "of_which_side_n_over_gt0": len(jn),
                       "side_seen_but_n_over_0": seen - len(jn),
                       "n_over_min_p50_p90_max": qs([d["n_over"] for d in so]),
                       "n_over_min_p50_p90_max_joined_refusals": qs([d["n_over"] for d in jn]),
                       "nanmax_m_min_p50_p90_max": qs([d["nanmax_m"] for d in so]),
                       "raw_code_top": codes.most_common(15),
                       "block4x4_sum": blk, "row_hist20_sum": rowh, "col_hist32_sum": colh,
                       "share_with_valid_neighbour_p50": qs([d["share_with_valid_neighbour"] for d in so]),
                       "isolated_share_p50": qs([d["isolated"] / d["n_over"] for d in so]),
                       "ring_depth_m_p50": qs([d["ring_depth_m_p50"] for d in so if d["ring_depth_m_p50"] is not None]),
                       "frame_valid_m_p999": qs([d["frame_valid_m_p999"] for d in so if d["frame_valid_m_p999"] is not None]),
                       "y_range_union": [min(d["y_range"][0] for d in so), max(d["y_range"][1] for d in so)] if so else None,
                       "x_range_union": [min(d["x_range"][0] for d in so), max(d["x_range"][1] for d in so)] if so else None}
        if "driver" in res and t_side0:
            nd = sum(1 for x in glb if True)  # all; restrict below
            g_side = [x for x in glb if stamps and stamps[0] / 1e9 <= x <= stamps[-1] / 1e9]
            res["side"]["receive_share_of_driver"] = round(len(sf) / max(1, len(g_side)), 3)
    rej_all = [r for r in run.rows if H.refuse_kind(r) == "over_range"]
    res["over_range_refusals"] = {"total": len(rej_all),
                                  "clean": sum(1 for r in rej_all if r["acquired_ns"] <= end_clean)}
    with open(a.out, "w") as fh:
        json.dump(res, fh, ensure_ascii=False, indent=1)
    brief = {k: res[k] for k in ("windows", "clean_on_fraction", "a", "over_range_refusals") if k in res}
    brief["b"] = {k: res["b"][k] for k in ("R_rcvbuf_on", "G_input_gap_on", "missed_on", "producer_socket_drops_on",
                                           "clean_total", "verdict")}
    brief["c"] = {k: res["c"][k] for k in ("segment_starts", "off_window_event_share", "event_kinds", "verdict")}
    brief["timeline"] = summ["regime_timeline_60s"]
    print(json.dumps(brief, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
