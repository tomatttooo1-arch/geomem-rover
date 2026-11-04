#!/usr/bin/env python3
"""Summarise an N4 run directory (docs/design/NANO_FULLLOAD_20260927.md §3-6).

usage: summarize.py <run_dir> [--limit-ms 200] > summary.md ; also writes <run_dir>/summary.json
Percentiles are nearest-rank (sorted, index ceil(q*n)-1), no interpolation (§5).
"""
import argparse
import csv
import json
import math
import re
import statistics
from collections import defaultdict
from datetime import datetime
from pathlib import Path


def pct(v, q):
    if not v:
        return None
    s = sorted(v)
    return s[max(0, math.ceil(q * len(s)) - 1)]


def dist(v, qs=(0.5, 0.95, 0.99, 0.999)):
    if not v:
        return {"n": 0}
    d = {"n": len(v)}
    for q in qs:
        d["p%g" % (q * 100)] = round(pct(v, q), 3)
    d["max"] = round(max(v), 3)
    d["mean"] = round(sum(v) / len(v), 3)
    return d


def load_phases(run):
    marks, win = {}, {}
    for line in open(run / "phases.log"):
        p = line.split()
        if len(p) < 3:
            continue
        wall, up, label = float(p[0]), float(p[1]), " ".join(p[2:])
        marks[label] = (wall, up)
        m = re.match(r"(start|end) (L\S+)$", label)
        if m:
            win.setdefault(m.group(2), {})[m.group(1)] = wall
    return marks, {k: (v["start"], v["end"]) for k, v in win.items() if "start" in v and "end" in v}


def tegra(run):
    rows = []
    for line in open(run / "tegrastats.log"):
        try:
            t = datetime.strptime(line[:19], "%m-%d-%Y %H:%M:%S").timestamp()
        except ValueError:
            continue
        r = {"t": t}
        m = re.search(r"RAM (\d+)/(\d+)MB", line)
        r["ram"], r["ram_total"] = int(m.group(1)), int(m.group(2))
        m = re.search(r"SWAP (\d+)/", line)
        r["swap"] = int(m.group(1)) if m else None
        m = re.search(r"CPU \[([^\]]+)\]", line)
        cores = []
        for c in m.group(1).split(","):
            mm = re.match(r"(\d+)%@(\d+)", c)
            cores.append((int(mm.group(1)), int(mm.group(2))) if mm else (0, 0))
        r["cores"] = cores
        m = re.search(r"GR3D_FREQ (\d+)%(?:@\[?(\d+))?", line)
        r["gpu"] = int(m.group(1)) if m else None
        r["gpu_f"] = int(m.group(2)) if m and m.group(2) else None
        r["temps"] = {k: float(v) for k, v in re.findall(r"(\w+)@([\d.]+)C", line)}
        m = re.search(r"VDD_IN (\d+)mW", line)
        r["vdd_in"] = int(m.group(1))
        rows.append(r)
    return rows


def procs(run):
    out = []
    with open(run / "procs.jsonl") as f:
        head = json.loads(f.readline())
        for line in f:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass
    return head, out


def in_win(t, w):
    return w[0] <= t < w[1]


def summarize(run, limit_ms):
    marks, wins = load_phases(run)
    res = {"windows": {k: [round(a, 3), round(b, 3), round(b - a, 1)] for k, (a, b) in wins.items()}}
    # clock offset wall - mono from the meter (both clocks recorded per image)
    off = None
    imgs = defaultdict(list)
    p = run / "meter" / "images.jsonl"
    if p.exists():
        for line in open(p):
            if line.startswith('["'):
                k, st, rx, rxm = json.loads(line)
                imgs[k].append((st, rx, rxm))
                if off is None:
                    off = rx - rxm
    tg = tegra(run)
    head, ps = procs(run)
    tck = head["clk_tck"]
    det = []
    if (run / "det.jsonl").exists():
        for line in open(run / "det.jsonl"):
            try:
                det.append(json.loads(line))
            except ValueError:
                pass
    b4 = []
    if (run / "b4.csv").exists() and "b4_loop_start" in marks:
        t0 = marks["b4_loop_start"][0]
        with open(run / "b4.csv") as f:
            for r in csv.DictReader(f):
                i = int(r["pass"]) * 310 + int(r["frame"])
                b4.append((t0 + i / 15.0, float(r["e2e_ms"])))
    rt = []
    if (run / "meter" / "rtabmap.jsonl").exists():
        for line in open(run / "meter" / "rtabmap.jsonl"):
            try:
                rt.append(json.loads(line))
            except ValueError:
                pass
    replay = []
    if (run / "rtabmap" / "replay.jsonl").exists():
        for line in open(run / "rtabmap" / "replay.jsonl"):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("event") == "pub":
                replay.append(r["wall_ns"])
    st = []
    if (run / "consumer" / "status.jsonl").exists():
        for line in open(run / "consumer" / "status.jsonl"):
            try:
                st.append(json.loads(line))
            except ValueError:
                pass
    evs = []
    if (run / "consumer" / "events.jsonl").exists():
        evs = [json.loads(line) for line in open(run / "consumer" / "events.jsonl")]
    scene = []
    if (run / "scene" / "scene.jsonl").exists():
        for line in open(run / "scene" / "scene.jsonl"):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("event") == "frame":
                scene.append(r)
    idle_vdd = None
    idle_cores = None
    layers = {}
    for L in sorted(wins, key=lambda k: wins[k][0]):
        w = wins[L]
        d = {"window_s": round(w[1] - w[0], 1)}
        # --- board: tegrastats
        rows = [r for r in tg if in_win(r["t"], w)]
        if rows:
            n = len(rows[0]["cores"])
            d["cpu_pct_per_core"] = [round(statistics.mean(r["cores"][i][0] for r in rows), 1) for i in range(n)]
            d["cpu_mhz_per_core_mean"] = [round(statistics.mean(r["cores"][i][1] for r in rows)) for i in range(n)]
            d["cpu_mhz_min_c0"] = min(r["cores"][0][1] for r in rows)
            d["gpu_pct_mean"] = round(statistics.mean(r["gpu"] for r in rows), 1)
            d["gpu_mhz_mean"] = round(statistics.mean(r["gpu_f"] for r in rows if r["gpu_f"])) if any(r["gpu_f"] for r in rows) else None
            d["ram_used_mb_max"] = max(r["ram"] for r in rows)
            d["ram_total_mb"] = rows[0]["ram_total"]
            d["swap_mb_max"] = max(r["swap"] for r in rows)
            d["vdd_in_mw_mean"] = round(statistics.mean(r["vdd_in"] for r in rows))
            d["vdd_in_mw_max"] = max(r["vdd_in"] for r in rows)
            d["temp_c_max"] = {k: max(r["temps"].get(k, 0) for r in rows) for k in rows[0]["temps"]}
        # --- board: /proc
        pr = [s for s in ps if s.get("event") == "s" and in_win(s["wall"], w)]
        if len(pr) >= 2:
            a, b = pr[0], pr[-1]
            dt = b["mono"] - a["mono"]

            def busy(c):
                return sum(c[:3]) + sum(c[5:8])  # user nice system irq softirq steal
            cores = [k for k in a["cpu"] if k != "cpu"]
            d["board_cores"] = round(sum(busy(b["cpu"][k]) - busy(a["cpu"][k]) for k in cores) / tck / dt, 3)
            d["cluster0_cores"] = round(sum(busy(b["cpu"][k]) - busy(a["cpu"][k]) for k in ["cpu0", "cpu1", "cpu2", "cpu3"]) / tck / dt, 3)
            d["cluster1_cores"] = round(sum(busy(b["cpu"][k]) - busy(a["cpu"][k]) for k in ["cpu4", "cpu5"]) / tck / dt, 3)
            first = {(q["pid"]): q for q in a["procs"]}
            per = defaultdict(float)
            rss = defaultdict(float)
            seen = {}
            for s in pr:
                for q in s["procs"]:
                    seen.setdefault(q["pid"], (s["mono"], q))
                    rss[q["match"]] = max(rss[q["match"]], 0)
            last = {q["pid"]: (s["mono"], q) for s in pr for q in s["procs"]}
            for pid, (t1, q1) in last.items():
                t0, q0 = seen[pid]
                per[q1["match"]] += (q1["ticks"] - q0["ticks"]) / tck / dt
            rs = defaultdict(float)
            for s in pr:
                acc = defaultdict(float)
                for q in s["procs"]:
                    acc[q["match"]] += q["rss_mb"]
                for k, v in acc.items():
                    rs[k] = max(rs[k], v)
            d["proc_cores"] = {k: round(v, 3) for k, v in sorted(per.items(), key=lambda x: -x[1]) if v >= 0.005}
            d["proc_rss_mb_max"] = {k: round(v) for k, v in rs.items() if k in d["proc_cores"]}
            _ = first
        if L == "L0":
            idle_vdd, idle_cores = d.get("vdd_in_mw_mean"), d.get("board_cores")
        elif idle_vdd is not None:
            d["vdd_in_mw_minus_L0"] = d["vdd_in_mw_mean"] - idle_vdd
            d["board_cores_minus_L0"] = round(d["board_cores"] - idle_cores, 3)
        # --- camera
        for k, name in (("c", "color"), ("d", "depth")):
            v = [x for x in imgs.get(k, []) if in_win(x[1] / 1e9, w)]
            if len(v) > 2:
                stamps = [x[0] for x in v]
                iv = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
                med = pct(iv, 0.5)
                d["cam_" + name] = {"frames": len(v), "fps": round(len(v) / (w[1] - w[0]), 2),
                                    "interval_ms": dist(iv), "gaps_gt_1p5x": sum(1 for x in iv if x > 1.5 * med),
                                    "missing_frames_est": int(sum(round(x / med) - 1 for x in iv if x > 1.5 * med)),
                                    "stamp_to_meter_ms": dist([(x[1] - x[0]) / 1e6 for x in v])}
        # --- detector
        dv = [r for r in det if not r.get("warmup") and in_win(r["done_us"] / 1e6, w)]
        if dv:
            n = len(dv)
            sk = sum(r["skipped"] for r in dv)
            d["det"] = {"frames": n, "fps": round(n / (w[1] - w[0]), 2),
                        "e2e_ms": dist([r["e2e_ms"] for r in dv]),
                        "tap_to_done_ms": dist([(r["done_us"] - r["tap_rx_us"]) / 1e3 for r in dv if "tap_rx_us" in r]),
                        "infer_ms": dist([r["infer_ms"] for r in dv]), "proc_ms": dist([r["proc_ms"] for r in dv]),
                        "late": sum(r["late"] for r in dv), "late_ratio": round(sum(r["late"] for r in dv) / n, 4),
                        "late_tap": sum(1 for r in dv if "tap_rx_us" in r and (r["done_us"] - r["tap_rx_us"]) / 1e3 > 66.7),
                        "late_tap_ratio": round(sum(1 for r in dv if "tap_rx_us" in r and (r["done_us"] - r["tap_rx_us"]) / 1e3 > 66.7) / n, 4),
                        "skipped": sk, "skipped_ratio": round(sk / (n + sk), 4)}
        # --- B4
        bv = [e for t, e in b4 if in_win(t, w)]
        if bv:
            d["b4"] = {"frames": len(bv), "e2e_ms": dist(bv), "over_66ms": sum(1 for x in bv if x > 66.7)}
        # --- RTAB-Map
        ov = [r for r in rt if r[0] == "o" and in_win(r[2] / 1e9, w)]
        pubs = [x for x in replay if in_win(x / 1e9, w)]
        if ov:
            d["vo"] = {"replay_pairs": len(pubs), "processed_ratio": round(len(ov) / len(pubs), 4) if pubs else None, "frames": len(ov), "hz": round(len(ov) / (w[1] - w[0]), 2), "lost": sum(1 for r in ov if r[3]),
                       "time_ms": dist([r[6] for r in ov]), "inliers_median": pct([r[4] for r in ov], 0.5)}
        iv_ = [r for r in rt if r[0] == "i" and in_win(r[2] / 1e9, w)]
        if iv_:
            d["rtabmap"] = {"updates": len(iv_), "hz": round(len(iv_) / (w[1] - w[0]), 2),
                            "total_ms": dist([r[5] for r in iv_ if r[5] is not None]),
                            "loop_closures": sum(1 for r in iv_ if r[4]), "wm_last": iv_[-1][6]}
        # --- rolling map status at the consumer
        if off is not None and st:
            sv = [r for r in st if in_win((r[0] + off) / 1e9, w)]
            if len(sv) > 2:
                rx = [r[0] for r in sv]
                ints = [(b - a) / 1e6 for a, b in zip(rx, rx[1:])]
                ages = [r[2] / 1e6 for r in sv if r[2] is not None]
                E = [(sv[i][2] + (sv[i + 1][0] - sv[i][0])) / 1e6 for i in range(len(sv) - 1) if sv[i][2] is not None]
                transit = [(r[0] - r[1]) / 1e6 for r in sv if r[1]]
                over = sum(1 for e in E if e > limit_ms)
                hours = (w[1] - w[0]) / 3600
                d["rolling"] = {"status_rows": len(sv), "status_hz": round(len(sv) / (w[1] - w[0]), 2),
                                "status_interval_ms": dist(ints), "bridge_age_ms": dist(ages),
                                "consumer_age_peak_E_ms": dist(E), "status_transit_ms": dist(transit),
                                "not_fresh_rows": sum(1 for r in sv if not r[3]),
                                "E_over_limit": over, "E_over_limit_per_h": round(over / hours, 1),
                                "limit_ms": limit_ms}
                revs = [e for e in evs if e["ev"] == "permit" and e["ok"] is False and in_win(e["wall"], w)]
                d["rolling"]["permit_revocations"] = len(revs)
                d["rolling"]["revocation_reasons"] = sorted({e["reason"] for e in revs})
            sc = [r for r in scene if in_win((r["t_acq"] + off) / 1e9, w)]
            if len(sc) > 2:
                ta = [r["t_acq"] for r in sc]
                d["producer"] = {"frames": len(sc), "hz": round(len(sc) / (w[1] - w[0]), 2),
                                 "interval_ms": dist([(b - a) / 1e6 for a, b in zip(ta, ta[1:])]),
                                 "acq_to_pub_ms": dist([(r["t_pub"] - r["t_acq"]) / 1e6 for r in sc if "t_pub" in r]),
                                 "rejected": sum(1 for r in sc if not r.get("accepted"))}
        ge = [e for e in evs if e["ev"].startswith("goal") and in_win(e["wall"], w)]
        if ge:
            cnt = defaultdict(int)
            for e in ge:
                cnt[e["ev"] + ("_%s" % e["status"] if e["ev"] == "goal_result" else "")] += 1
            d["goals"] = dict(cnt)
        layers[L] = d
    res["layers"] = layers
    # §5 recommendation from L5
    top = [k for k in ("L5", "L4", "L3") if k in layers and layers[k].get("rolling")]
    L5 = layers[top[0]]["rolling"] if top else None
    if L5 and L5["consumer_age_peak_E_ms"]["n"]:
        p999 = L5["consumer_age_peak_E_ms"]["p99.9"]
        T = max(p999 + 100.0, 200.0)
        T = math.ceil(T / 10.0) * 10
        res["recommendation"] = {"layer": top[0], "E_p99.9_ms": p999, "T_ms": T if p999 <= 300 else None,
                                 "rule": "max(E p99.9 + 100 ms frame period, 200 ms), ceil to 10 ms; none if p99.9 > 300 ms",
                                 "stop_distance_m_at_0.2mps": round(0.2 * T / 1000, 3)}
    return res


def fmt(d, keys=("p50", "p95", "p99", "max")):
    if not d or not d.get("n"):
        return "—"
    return " / ".join("%.1f" % d[k] for k in keys)


def markdown(res):
    L = res["layers"]
    order = [k for k in ["L0", "L1", "L2", "L3", "L4", "L5", "L5nob4", "L4nob4", "L3nob4"] if k in L]
    out = ["| 层 | 整板核当量（−L0） | 簇0 / 簇1 核当量 | 各核平均占用 % | 簇0/簇1 平均 MHz | GPU 忙 % | RAM 峰值 MB | VDD_IN mW（−L0） | 最高温 °C |",
           "|---|---|---|---|---|---|---|---|---|"]
    for k in order:
        d = L[k]
        f = d.get("cpu_mhz_per_core_mean", [0] * 6)
        out.append("| %s | %.2f（%s） | %.2f / %.2f | %s | %d / %d | %.0f | %d | %d（%s） | %.1f |" % (
            k, d["board_cores"], "%+.2f" % d["board_cores_minus_L0"] if "board_cores_minus_L0" in d else "—",
            d["cluster0_cores"], d["cluster1_cores"], " ".join("%.0f" % x for x in d["cpu_pct_per_core"]),
            sum(f[:4]) / 4, sum(f[4:]) / 2, d["gpu_pct_mean"], d["ram_used_mb_max"], d["vdd_in_mw_mean"],
            "%+d" % d["vdd_in_mw_minus_L0"] if "vdd_in_mw_minus_L0" in d else "—", max(d["temp_c_max"].values())))
    out += ["", "| 层 | 彩色 帧/秒（断帧） | 深度 帧/秒（断帧） | 驱动时间戳→测量节点 ms p50/p95/p99/max | 识别 帧/秒 | 识别 收到→出结果 ms p50/p95/p99/max | 识别 时间戳→出结果 ms p50/p95/p99/max | 推理 ms p50 | 迟发 / 跳帧 |",
            "|---|---|---|---|---|---|---|---|---|"]
    for k in order:
        d = L[k]
        c, dp, t = d.get("cam_color"), d.get("cam_depth"), d.get("det")
        out.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            k, "%.2f（%d）" % (c["fps"], c["gaps_gt_1p5x"]) if c else "—",
            "%.2f（%d）" % (dp["fps"], dp["gaps_gt_1p5x"]) if dp else "—",
            fmt(c["stamp_to_meter_ms"]) if c else "—",
            "%.2f" % t["fps"] if t else "—", fmt(t["tap_to_done_ms"]) if t else "—", fmt(t["e2e_ms"]) if t else "—",
            "%.1f" % t["infer_ms"]["p50"] if t else "—", "%d / %d" % (t["late_tap"], t["skipped"]) if t else "—"))
    out += ["", "| 层 | B4 每帧 ms p50/p95/p99/max | 里程计 处理/回放（丢） | 里程计每帧 ms p50/p95/p99/max | RTAB-Map 每次更新 ms p50/p95/p99/max（次数） | 生产端 帧间隔 ms p50/p99/p99.9/max | 采集→发布 ms p50/p99/max | 目标 |",
            "|---|---|---|---|---|---|---|---|"]
    for k in order:
        d = L[k]
        b, v, r, p, g = d.get("b4"), d.get("vo"), d.get("rtabmap"), d.get("producer"), d.get("goals")
        out.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
            k, fmt(b["e2e_ms"]) if b else "—",
            "%d/%d（%d）" % (v["frames"], v["replay_pairs"], v["lost"]) if v else "—",
            fmt(v["time_ms"]) if v else "—", "%s（%d）" % (fmt(r["total_ms"]), r["updates"]) if r else "—",
            fmt(p["interval_ms"], ("p50", "p99", "p99.9", "max")) if p else "—",
            fmt(p["acq_to_pub_ms"], ("p50", "p99", "max")) if p else "—",
            json.dumps(g, ensure_ascii=False) if g else "—"))
    out += ["", "| 层 | status 条数（Hz） | status 间隔 ms p50/p95/p99/p99.9/max | 桥接端地图年龄 ms p50/p95/p99/p99.9/max | 消费端年龄峰值 E ms p50/p95/p99/p99.9/max | E>200 ms 次数（每小时） | 许可撤销次数 |",
            "|---|---|---|---|---|---|---|"]
    q5 = ("p50", "p95", "p99", "p99.9", "max")
    for k in order:
        r = L[k].get("rolling")
        if not r:
            continue
        out.append("| %s | %d（%.1f） | %s | %s | %s | %d（%.1f） | %d |" % (
            k, r["status_rows"], r["status_hz"], fmt(r["status_interval_ms"], q5), fmt(r["bridge_age_ms"], q5),
            fmt(r["consumer_age_peak_E_ms"], q5), r["E_over_limit"], r["E_over_limit_per_h"], r["permit_revocations"]))
    out += ["", "各进程核当量（/proc，窗口平均）与 RSS 峰值 MB：", ""]
    for k in order:
        d = L[k]
        out.append("- %s：%s" % (k, "，".join("%s %.2f（%d MB）" % (n, c, d["proc_rss_mb_max"].get(n, 0))
                                         for n, c in d.get("proc_cores", {}).items())))
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--limit-ms", type=float, default=200.0)
    ap.add_argument("--md", action="store_true", help="print markdown tables instead of JSON")
    a = ap.parse_args()
    run = Path(a.run)
    res = summarize(run, a.limit_ms)
    (run / "summary.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print(markdown(res) if a.md else json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
