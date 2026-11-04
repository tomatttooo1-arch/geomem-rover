#!/usr/bin/env python3
"""GOLIVE0 (2026-11-04): judge a pulled bench run against RUN/GOLIVE0/CRITERIA.md (C1-C6) and list the rates.

    python3 tools/golive/judge.py RUN/GOLIVE0/raw/<tag> [min_s] [gmap3]   (expects <tag>/kv/ and <tag>/nano/ from stop_all.sh)
    C8_pln_gmap3 (GOLIVE1 renumbered from C7; PLNCHAIN0, optional): appears when field rows carry "pln" (PLN=gmap3) or with the third arg gmap3.

Writes <tag>/judge.json and prints a short table. Thresholds are the ones in CRITERIA.md (written before the run).

C7 (SRCHCHAIN0, optional; only when the run has kv/srch.jsonl with SRCH=pl requested): the search core is really
called during the run: init on the core (impl pl), >= 2 episodes on the core, the driver's core call count goes up
from episode to episode with 0 core fails, the one-time self-check against the software order says same, and no
fallback. Absent / SRCH=nano -> C7 is not reported and not part of ALL_PASS.
"""

import csv
import json
import os
import re
import sys

SEG_S = 30.0
WARM_S = 10.0


def rows(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path, errors="replace") as fp:
        for line in fp:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return out


def rate(ts, t0, t1):
    ts = [t for t in ts if t0 <= t <= t1]
    return round(len(ts) / (t1 - t0), 2) if t1 > t0 else None


def segs_all_have(ts, t0, t1):
    n, bad = 0, []
    t = t0
    while t + SEG_S <= t1 + 1e-6:
        if not any(t <= x < t + SEG_S for x in ts):
            bad.append(round(t - t0))
        n += 1
        t += SEG_S
    return n, bad


def judge_srch(kv, ph):
    recs = rows(os.path.join(kv, "srch.jsonl"))
    init = [r for r in recs if r.get("ev") == "init"]
    req = init[-1].get("impl_req") if init else None
    if req not in ("pl", "mock"):           # mock = the server stand-in of pl (same driver protocol, C model)
        return None
    ep = [r for r in recs if r.get("ev") == "episode"]
    pl = [r for r in ep if r.get("impl") == req]
    calls = [r.get("core_calls") for r in pl]
    rising = len(calls) >= 2 and all(isinstance(c, int) for c in calls) and all(
        b > a for a, b in zip(calls, calls[1:]))
    sc = [r["selfcheck"] for r in pl if r.get("selfcheck")]
    warns = [r.get("msg") for r in recs if r.get("ev") == "warn"]
    fails = max([r.get("core_fails") or 0 for r in pl] or [0])
    per_ep = [b - a for a, b in zip(calls, calls[1:])] if rising else []
    return {"impl_at_init": init[-1].get("impl"), "init_ms": init[-1].get("init_ms"), "episodes": len(ep),
            "episodes_on_core": len(pl), "core_calls_first_last": [calls[0], calls[-1]] if calls else None,
            "core_calls_per_episode_min": min(per_ep) if per_ep else None, "core_fails": fails,
            "episode_ms_median": sorted(r["ms"] for r in pl)[len(pl) // 2] if pl else None,
            "selfcheck": sc[0] if sc else None, "warnings": warns[:3],
            "first_look_last": pl[-1].get("first") if pl else None,
            "impl_req": req, "pass": bool(init[-1].get("impl") == req and rising and fails == 0 and not warns
                         and sc and sc[0].get("same") is True)}


def judge_mem(kv, nano):
    """GOLIVE1 (MEMCHAIN0's on-board criterion, written before the GOLIVE1 run): present when the uplink carries
    "mem". Pass = every uplink datagram after the first one with mem carries it, src is "pl" on all of them,
    fallbacks stay 0, the ASSOC call count rises in every 30 s segment, and the Nano answers came from src pl."""
    up = rows(os.path.join(kv, "uplink.jsonl"))
    up = [r.get("msg", r) for r in up]
    i0 = next((i for i, m in enumerate(up) if isinstance(m.get("mem"), dict)), None)
    if i0 is None:
        return None
    win = up[i0:]
    mm = [m.get("mem") for m in win if isinstance(m.get("mem"), dict)]
    srcs = sorted({str(m.get("src")) for m in mm})
    fb = max([m.get("fallbacks") or 0 for m in mm] or [0])
    pts = [(m["t_mono_ns"] / 1e9, ((m.get("mem") or {}).get("calls") or {}).get("assoc"))
           for m in win if isinstance(m.get("mem"), dict) and m.get("t_mono_ns")]
    stuck, t = [], pts[0][0] if pts else 0
    while pts and t + SEG_S <= pts[-1][0] + 1e-6:
        seg = [a for tt, a in pts if t <= tt < t + SEG_S and isinstance(a, int)]
        if len(seg) < 2 or seg[-1] <= seg[0]:
            stuck.append(round(t - pts[0][0]))
        t += SEG_S
    nst = [s for s in rows(os.path.join(nano, "console.out")) if "rx" in s and s.get("kv_memory")]
    nsrc = sorted({str((s.get("kv_memory") or {}).get("src")) for s in nst})
    last = mm[-1] if mm else {}
    return {"uplink_after_first_mem": len(win), "with_mem": len(mm), "src": srcs, "fallbacks": fb,
            "assoc_first_last": [pts[0][1], pts[-1][1]] if pts else None, "segments_without_rise_at_s": stuck,
            "nano_status_lines": len(nst), "nano_src": nsrc, "n_objects_last": last.get("n_objects"),
            "pass": bool(len(mm) == len(win) and srcs == ["pl"] and fb == 0 and not stuck and nst
                         and nsrc == ["pl"])}


def pln_check(dep, t0, t1, want=None):
    """PLNCHAIN0 optional criterion: with PLN=gmap3 the field rows carry "pln" (GMAP3 counters). Pass = the core's
    path-fetch count rises over the steady run, in every 30 s segment that has field rows, and no core path ever
    differed from the PS extraction. Present when such rows exist, or when asked for (want="gmap3": absent = fail)."""
    pr = [(r["t_field_ns"] / 1e9, r["pln"]) for r in dep
          if r.get("ev") == "field" and r.get("t_field_ns") and isinstance(r.get("pln"), dict)]
    if not pr and want != "gmap3":
        return None
    st = [(t, p) for t, p in pr if t0 <= t <= t1] or pr
    if not st:
        return {"rows": 0, "pass": False, "why": "no field row carries pln (PLN=gmap3 not active: m5 driver fallback?)"}
    pc = [p.get("path_calls", 0) for _, p in st]
    stuck, t = [], t0
    while t + SEG_S <= t1 + 1e-6:
        seg = [p.get("path_calls", 0) for tt, p in st if t <= tt < t + SEG_S]
        if len(seg) >= 2 and seg[-1] <= seg[0]:
            stuck.append(round(t - t0))
        t += SEG_S
    last = st[-1][1]
    keys = ("path_calls", "path_found", "path_nopath", "path_other", "path_nomark", "path_match", "path_mismatch",
            "field_fail", "gmap_enabled", "merge_ok", "merge_fail", "scan_ok")
    return {"rows": len(st), "path_calls_first": pc[0], "path_calls_last": pc[-1],
            "path_calls_per_s": round((pc[-1] - pc[0]) / (st[-1][0] - st[0][0]), 2) if st[-1][0] > st[0][0] else None,
            "segments_without_rise_at_s": stuck, "last": {k: last.get(k) for k in keys},
            "pass": bool(pc[-1] > pc[0] and not stuck and last.get("path_mismatch", 0) == 0)}


def main(d, min_s=600.0, pln=None):
    kv, nano = os.path.join(d, "kv"), os.path.join(d, "nano")
    prog = open(os.path.join(kv, "progress.txt"), errors="replace").read() if os.path.exists(
        os.path.join(kv, "progress.txt")) else ""
    ph = {m.group(1): float(m.group(2)) for m in re.finditer(r"== PHASE (\S+) ([0-9.]+)", prog)}
    boots = re.findall(r"^boot (.+)$", prog, re.M)
    started = dict(re.findall(r"^(kv_cam|kv_server|dpu_rx|kv_uplink) pid (\d+)", prog, re.M))
    termed = dict(re.findall(r"^TERM (kv_cam|kv_server|dpu_rx|kv_uplink) pid (\d+)", prog, re.M))
    dep = rows(os.path.join(kv, "depth.jsonl"))
    col = rows(os.path.join(kv, "colour.jsonl"))
    box = rows(os.path.join(kv, "boxes.jsonl"))
    t_geo = [r["t_geo_ns"] / 1e9 for r in dep if r.get("chain") == "depth" and r.get("t_geo_ns")]
    t_grid = [r["t_grid_ns"] / 1e9 for r in dep if r.get("chain") == "depth" and r.get("grid_ok") and r.get("t_grid_ns")]
    t_field = [r["t_field_ns"] / 1e9 for r in dep if r.get("ev") == "field" and r.get("t_field_ns")]
    t_col = [r["t_post_ns"] / 1e9 for r in col if r.get("chain") == "colour" and r.get("t_post_ns")]
    t_cam = [r["t_cam_arrive_ns"] / 1e9 for r in dep if r.get("chain") == "depth" and r.get("t_cam_arrive_ns")]
    ndet_frames = sum(1 for r in box if r.get("dets"))
    allt = t_geo + t_col
    res = {"dir": d}
    if allt:
        t0, t1 = min(allt) + WARM_S, max(allt)
    else:
        t0 = t1 = 0.0
    dur = (ph.get("stop", 0) - ph.get("running", 0)) if "running" in ph and "stop" in ph else None
    res["C1_duration_s"] = {"value": round(dur, 1) if dur else None, "min_s": min_s, "pass": bool(dur and dur >= min_s)}
    n, bad = segs_all_have(t_geo, t0, t1)
    fr = rate(t_geo, t0, t1)
    res["C2_depth"] = {"fps_steady": fr, "grid_fps": rate(t_grid, t0, t1), "segments": n, "empty_segments_at_s": bad,
                       "frames": len(t_geo), "pass": bool(fr and fr >= 5 and not bad)}
    n, bad = segs_all_have(t_field, t0, t1)
    res["C3_field"] = {"updates": len(t_field), "fps": rate(t_field, t0, t1), "empty_segments_at_s": bad,
                       "pass": bool(t_field and not bad)}
    n, bad = segs_all_have(t_col, t0, t1)
    fr = rate(t_col, t0, t1)
    res["C4_recog"] = {"fps_steady": fr, "frames": len(t_col), "frames_with_boxes": ndet_frames,
                       "empty_segments_at_s": bad, "pass": bool(fr and fr >= 3 and not bad and ndet_frames > 0)}
    c8 = pln_check(dep, t0, t1, pln)
    if c8 is not None:
        res["C8_pln_gmap3"] = c8
    nrx = rows(os.path.join(nano, "nano_console.jsonl"))
    status = rows(os.path.join(nano, "console.out"))
    status = [s for s in status if "rx" in s]
    need = (dur or 0) * 2 * 0.9
    dcount = [((s.get("map") or {}).get("depth_frames"), (s.get("recog") or {}).get("frames")) for s in status]
    stuck = sum(1 for a, b in zip(dcount, dcount[1:]) if a[0] == b[0] or a[1] == b[1])
    last = status[-1] if status else {}
    res["C5_nano"] = {"datagrams": len(nrx), "needed": round(need), "status_lines": len(status),
                      "status_lines_with_stuck_counter": stuck, "find": last.get("find"),
                      "memory": last.get("memory"), "alarm_lines": last.get("alarm_lines"),
                      "pass": bool(nrx and len(nrx) >= need and stuck == 0 and (last.get("find") or {}).get("found"))}
    dm = open(os.path.join(kv, "dmesg_tail.txt"), errors="replace").read() if os.path.exists(
        os.path.join(kv, "dmesg_tail.txt")) else ""
    same = all(termed.get(k) == v for k, v in started.items()) and len(started) == 4
    res["C6_no_crash"] = {"started": started, "alive_at_stop": termed, "boot": boots,
                          "oops": bool(re.search(r"Oops|panic|Call trace", dm, re.I)),
                          "pass": bool(same and len(set(boots)) == 1 and len(boots) >= 2
                                       and not re.search(r"Oops|panic", dm))}
    res["rates_fps"] = {"camera_depth_arrive": rate(t_cam, t0, t1), "geometry": res["C2_depth"]["fps_steady"],
                        "cost_map": res["C2_depth"]["grid_fps"], "field": res["C3_field"]["fps"],
                        "recognition_out": res["C4_recog"]["fps_steady"],
                        "uplink_datagrams": round(len(nrx) / dur, 2) if dur else None}
    try:
        res["chain_report"] = json.load(open(os.path.join(kv, "chain.json")))
    except (OSError, ValueError):
        pass
    try:
        p = [int(r["power_uW"]) / 1e6 for r in csv.DictReader(open(os.path.join(kv, "pwr.csv")))]
        res["power_W_mean"] = round(sum(p) / len(p), 3) if p else None
    except (OSError, KeyError, ValueError):
        pass
    c7 = judge_srch(kv, ph)
    if c7 is not None:
        res["C7_srch_core"] = c7
    c9 = judge_mem(kv, nano)
    if c9 is not None:
        res["C9_mem_pl"] = c9
    res["all_pass"] = all(res[k]["pass"] for k in res if k.startswith("C"))
    json.dump(res, open(os.path.join(d, "judge.json"), "w"), indent=1)
    for k in sorted(k for k in res if k.startswith("C")):
        v = {kk: vv for kk, vv in res[k].items() if kk not in ("memory", "started", "alive_at_stop")}
        print(k, json.dumps(v, ensure_ascii=False)[:300])
    print("rates_fps", json.dumps(res["rates_fps"]), "power_W_mean", res.get("power_W_mean"), "ALL_PASS", res["all_pass"])


if __name__ == "__main__":
    # 13:1x evolution: 300. Third argument "gmap3" (PLNCHAIN0): C7 required even when no row carries pln
    main(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 600.0, sys.argv[3] if len(sys.argv) > 3 else None)
