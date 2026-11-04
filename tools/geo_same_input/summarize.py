#!/usr/bin/env python3
"""Tabulate a run_board.sh (or tum_recheck.sh) output directory.

Per arm: kernel and end-to-end latency (median / p95 / max over all measured
frames), the spread of per-pass medians, late starts (paced mode), process CPU
from getrusage over the measured passes only (core-equivalents = CPU s / wall
s; CPU ms per frame), whole-board CPU busy and VDD_IN from tegrastats (500 ms
samples over the measured passes only, first 2 s and last 1 s dropped, minus the
idle window at the start), and the mean CPU clock per cluster from cpufreq
time_in_state (whole process: includes start-up, warm-up and, for the bench, its
untimed single-thread reference pre-pass).
Usage: summarize.py <out_dir>   (run_board.sh or tum_recheck.sh layout)
"""
import csv
import glob
import json
import os
import re
import statistics
import sys
from datetime import datetime


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]


def tri(v):
    return "%.2f / %.2f / %.2f" % (statistics.median(v), pct(v, .95), max(v)) if v else "—"


def duo(v):
    return "%.2f / %.2f" % (statistics.median(v), pct(v, .95)) if v else "—"


def phases(d):
    ph = {}
    for line in open(os.path.join(d, "phases.log")):
        t, *rest = line.split()
        m = re.match(r"(start|end) (\S+)", " ".join(rest))
        if m:
            ph.setdefault(m.group(2), {})[m.group(1)] = float(t)
    return ph


def tegra(d):
    ts = []
    for line in open(os.path.join(d, "tegrastats.log")):
        m = re.match(r"(\d\d-\d\d-\d{4} \d\d:\d\d:\d\d) RAM (\d+)/(\d+)MB.*CPU \[([^\]]+)\].*VDD_IN (\d+)mW", line)
        if not m:
            continue
        t = datetime.strptime(m.group(1), "%m-%d-%Y %H:%M:%S").timestamp()
        cores = [c.split("@") for c in m.group(4).split(",")]
        ts.append((t, [float(c[0].rstrip("%")) for c in cores], [int(c[1]) for c in cores], int(m.group(5))))
    return ts


def board(ts, t0, t1):
    s = [x for x in ts if t0 + 2 <= x[0] <= t1 - 1]
    if not s:
        return None
    return {"n": len(s), "cpu_busy_core_equiv": round(statistics.mean(sum(x[1]) / 100 for x in s), 2),
            "per_core_util": [round(statistics.mean(x[1][i] for x in s)) for i in range(len(s[0][1]))],
            "mhz_c0_c4_median": [int(statistics.median(x[2][0] for x in s)), int(statistics.median(x[2][4] for x in s))],
            "vdd_in_mw": round(statistics.mean(x[3] for x in s))}


def tis_mean(d, tag, runs_sub="runs"):
    """Mean clock (MHz) and share of time at the top clock per policy over a run, from time_in_state deltas."""
    out = {}
    paths = [os.path.join(d, runs_sub, "%s.tis_%s" % (tag, k)) for k in ("start", "end")]
    if not all(os.path.exists(p) for p in paths):
        paths = [os.path.join(d, "%s.tis_%s" % (tag, k)) for k in ("start", "end")]
        if not all(os.path.exists(p) for p in paths):
            return None
    snaps = []
    for p in paths:
        cur, tab = None, {}
        for line in open(p):
            if line.startswith("policy"):
                cur = line.strip()
                tab[cur] = {}
            else:
                f, t = line.split()
                tab[cur][int(f)] = int(t)
        snaps.append(tab)
    for pol in snaps[0]:
        delta = {f: snaps[1][pol][f] - snaps[0][pol][f] for f in snaps[0][pol]}
        tot = sum(delta.values())
        if tot <= 0:
            continue
        top = max(delta)
        out[pol] = {"mean_mhz": round(sum(f * t for f, t in delta.items()) / tot / 1000),
                    "share_at_max": round(delta[top] / tot, 3),
                    "top3": sorted(((round(t / tot, 3), f // 1000) for f, t in delta.items()), reverse=True)[:3]}
    return out


def read_rows(path):
    return list(csv.DictReader(open(path)))


def json_lines(path):
    out = []
    for line in open(path):
        line = line.strip()
        if line.startswith("{"):
            out.append(json.loads(line))
    return out


def arm_stats(d, tag):
    if os.path.isdir(os.path.join(d, "runs", tag)):          # python arms A / C
        rows = read_rows(os.path.join(d, "runs", tag, "frames.csv"))
        s = json.load(open(os.path.join(d, "runs", tag, "summary.json")))
        kernel = [float(r["core_ms"]) for r in rows]
        device = [float(r["device_ms"]) for r in rows if r["device_ms"] != ""]
        e2e = [float(r["e2e_ms"]) for r in rows]
        conv = [float(r["convert_ms"]) for r in rows]
        cpu = s["measured_cpu_user_s"] + s["measured_cpu_sys_s"]
        wall, late, frames = s["measured_wall_s"], s["paced_late_starts"], len(rows)
        extra = {"convert_ms": duo(conv), "device_ms": duo(device) if device else None,
                 "backend_name": s["backend_name"], "ik_matches_prep": s["inverse_k_matches_prep"]}
    else:                                                     # C++ bench arms B*
        tum = not os.path.exists(os.path.join(d, "runs", tag + ".csv"))   # tum_recheck.sh layout
        rows = read_rows(os.path.join(d, "csv", tag + ".csv") if tum else os.path.join(d, "runs", tag + ".csv"))
        js = json_lines(os.path.join(d, tag + ".stdout") if tum else os.path.join(d, "runs", tag + ".stdout"))
        run = [j for j in js if "opt_mismatches" in j][0]
        cpuj = ([j for j in js if "measured_cpu_user_s" in j] or [None])[0]
        kernel = [float(r["s1s2_fused_ms"]) for r in rows]
        e2e = [float(r["e2e_ms"]) for r in rows]
        cpu = cpuj["measured_cpu_user_s"] + cpuj["measured_cpu_sys_s"] if cpuj else None
        wall = cpuj["measured_wall_s"] if cpuj else None
        late, frames = run["paced_late_starts"], len(rows)
        extra = {"s0_ms": duo([float(r["s0_decode_ms"]) for r in rows]),
                 "s1_ms": duo([float(r["s1_project_ms"]) for r in rows]),
                 "s2_ms": duo([float(r["s2_rays_ms"]) for r in rows]),
                 "threads": run["threads"], "in_binary_ref_mismatches": run["opt_mismatches"]}
    passes = sorted({r["pass"] for r in rows})
    pass_med = [statistics.median(float(r["e2e_ms"]) for r in rows if r["pass"] == p) for p in passes]
    return {"frames": frames, "kernel": tri(kernel), "e2e": tri(e2e),
            "kernel_median": round(statistics.median(kernel), 3), "kernel_p95": round(pct(kernel, .95), 3),
            "e2e_median": round(statistics.median(e2e), 3), "e2e_p95": round(pct(e2e, .95), 3),
            "pass_e2e_medians": [round(v, 2) for v in pass_med], "late": late,
            "measured_wall_s": wall, "proc_core_equiv": round(cpu / wall, 2) if wall else None,
            "cpu_ms_per_frame": round(cpu / frames * 1e3, 2) if cpu is not None else None, **extra}


def main():
    d = sys.argv[1]
    ph, ts = phases(d), tegra(d)
    idle = board(ts, ph["idle"]["start"], ph["idle"]["end"])
    tags = [t for t in ph if t != "idle"]
    res = {"idle": idle, "arms": {}}
    lines = ["| 臂 | 模式 | 帧数 | 核耗时 中位/p95/max ms | 端到端 中位/p95/max ms | 各趟端到端中位 | 迟发 | 进程 CPU 核当量 | CPU ms/帧 | 整板−空闲 核当量 | VDD_IN mW (−空闲) | 簇0/簇1 平均 MHz |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for tag in tags:
        if "end" not in ph[tag]:
            continue
        st = arm_stats(d, tag)
        # Board window = the measured passes only: they end when the process ends (only the CSV /
        # summary write follows) and last measured_wall_s. Start-up, g++ compile, warm-up pass and
        # the bench's untimed reference pre-pass are excluded.
        t0 = ph[tag]["end"] - st["measured_wall_s"] if st.get("measured_wall_s") else ph[tag]["start"]
        b = board(ts, max(t0, ph[tag]["start"]), ph[tag]["end"])
        tis = tis_mean(d, tag)
        st["board"], st["cpufreq"] = b, tis
        res["arms"][tag] = st
        arm, mode = tag.rsplit("_", 1) if tag.count("_") == 1 else (tag, "")
        bc = "%.2f" % (b["cpu_busy_core_equiv"] - idle["cpu_busy_core_equiv"]) if b else "—"
        pw = "%d (%+d)" % (b["vdd_in_mw"], b["vdd_in_mw"] - idle["vdd_in_mw"]) if b else "—"
        fq = "%s / %s" % (tis["policy0"]["mean_mhz"], tis["policy4"]["mean_mhz"]) if tis else "—"
        lines.append("| %s | %s | %d | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            arm, {"p15": "定速 15 Hz", "free": "连续"}.get(mode, mode or "见名称"), st["frames"], st["kernel"], st["e2e"],
            "–".join("%.2f" % v for v in (min(st["pass_e2e_medians"]), max(st["pass_e2e_medians"]))),
            st["late"], st["proc_core_equiv"], st["cpu_ms_per_frame"], bc, pw, fq))
    lines.append("")
    lines.append("空闲基线：整板 CPU 忙 %s 核当量，VDD_IN %s mW（%d 个 tegrastats 样本）。" % (
        idle["cpu_busy_core_equiv"], idle["vdd_in_mw"], idle["n"]))
    if os.path.exists(os.path.join(d, "compare.json")):
        cmp_ = json.load(open(os.path.join(d, "compare.json")))
        res["compare"] = cmp_
        lines.append("")
        lines.append("逐格对照（参照 = A_p15 第一趟测量；控制：翻 1 格被抓到 %s，错位 1 帧有 %d 帧不同，参照 %d 帧里互不相同的 %d 帧）：" % (
            cmp_["controls"]["one_voxel_flip_detected"], cmp_["controls"]["shift_by_one_frame_frames_differing"],
            cmp_["reference_stats"]["frames"], cmp_["controls"]["distinct_reference_frames"]))
        lines.append("")
        lines.append("| 臂 | 逐格全等 | 有差异的帧 | 差异格总数 | 计数不同的帧 |")
        lines.append("|---|---|---|---|---|")
        for k, v in cmp_["candidates"].items():
            lines.append("| %s | %s | %s | %s | %s |" % (k, "是" if v.get("equal") else "**否**", v.get("frames_with_voxel_diff"),
                                                         v.get("voxels_diff_total"), v.get("frames_with_count_diff")))
    open(os.path.join(d, "summary.md"), "w").write("\n".join(lines) + "\n")
    json.dump(res, open(os.path.join(d, "summary.json"), "w"), indent=1)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
