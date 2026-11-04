"""Summarise a board_bench.sh output directory.

Per arm x grid: for each run (separate process) the per-frame median and p95
of every stage; then the across-run range of those run-level medians (run-level
stability). Whole-board CPU/RAM/power per phase from tegrastats (500 ms samples)
attributed by the phase timestamps; peak RSS / child CPU from the ledger.
"""
import csv, glob, json, os, re, statistics, sys
from datetime import datetime

d = sys.argv[1]
STAGES = ["s0_decode_ms", "s1_project_ms", "s2_rays_ms", "s1s2_fused_ms", "s3_object_ms", "e2e_ms"]


def pct(v, q):
    v = sorted(v); i = min(len(v) - 1, int(round(q * (len(v) - 1)))); return v[i]


runs = {}
for f in sorted(glob.glob(os.path.join(d, "csv", "*.csv"))):
    tag = os.path.basename(f)[:-4]
    m = re.match(r"(.+)_(g\d)_r(\d+)$", tag)
    if not m:
        continue
    arm, grid, rep = m.group(1), m.group(2), int(m.group(3))
    rows = list(csv.DictReader(open(f)))
    st = {}
    for s in STAGES:
        v = [float(r[s]) for r in rows if r[s] not in ("", None)]
        if v:
            st[s] = (statistics.median(v), pct(v, .95), len(v))
    extra = rows[0] if rows else {}
    runs.setdefault((arm, grid), []).append((rep, st, rows))

# tegrastats per phase
ph = {}
for line in open(os.path.join(d, "phases.log")):
    t, *rest = line.split()
    rest = " ".join(rest)
    m = re.match(r"(start|end) (\S+)", rest)
    if m:
        ph.setdefault(m.group(2), {})[m.group(1)] = float(t)
ts = []
tz_off = None
for line in open(os.path.join(d, "tegrastats.log")):
    m = re.match(r"(\d\d-\d\d-\d{4} \d\d:\d\d:\d\d) RAM (\d+)/(\d+)MB.*CPU \[([^\]]+)\].*VDD_IN (\d+)mW", line)
    if not m:
        continue
    t = datetime.strptime(m.group(1), "%m-%d-%Y %H:%M:%S").timestamp()
    cores = [c.split("@") for c in m.group(4).split(",")]
    util = [float(c[0].rstrip("%")) if c[0] != "off" else 0. for c in cores]
    freq = [int(c[1]) if len(c) > 1 else 0 for c in cores]
    ts.append((t, int(m.group(2)), util, freq, int(m.group(5))))


def board(tag):
    p = ph.get(tag, {})
    if "start" not in p or "end" not in p:
        return None
    s = [x for x in ts if p["start"] + 1 <= x[0] <= p["end"] - 0.5]
    if not s:
        return None
    ncore = len(s[0][2])
    return {"samples": len(s), "ram_used_mb_max": max(x[1] for x in s),
            "cpu_util_mean_per_core": [round(statistics.mean(x[2][i] for x in s), 1) for i in range(ncore)],
            "cpu_freq_mhz_median_per_core": [int(statistics.median(x[3][i] for x in s)) for i in range(ncore)],
            "board_cpu_busy_pct_of_6": round(statistics.mean(sum(x[2]) / ncore for x in s), 1),
            "vdd_in_mw_mean": round(statistics.mean(x[4] for x in s))}


ledger = {}
for line in open(os.path.join(d, "ledger.jsonl")):
    j = json.loads(line); ledger[j["tag"]] = j
idle = [x for x in ts if x[0] < min(p.get("start", 1e20) for p in ph.values())]

out = {"idle_before": {"samples": len(idle), "ram_used_mb": idle[-1][1] if idle else None,
                       "vdd_in_mw_mean": round(statistics.mean(x[4] for x in idle)) if idle else None},
       "arms": {}}
lines = []
lines.append("| 臂 | 网格 | 线程 | 趟数×帧样本 | S0 解码 | S1 投影 | S2 射线 | S1+S2 | S3 物体 | 端到端 | 端到端趟间中位数范围 | 峰值 RSS MB | 整板 CPU 忙 %(6核均) | VDD_IN mW |")
lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for (arm, grid), rr in sorted(runs.items(), key=lambda k: (k[0][1], k[0][0])):
    rr.sort()
    pooled = {s: [float(r[s]) for _, _, rows in rr for r in rows if r[s] not in ("", None)] for s in STAGES}
    def cell(s):
        v = pooled[s]
        return f"{statistics.median(v):.2f} / {pct(v, .95):.2f}" if v else "—"
    e2e_meds = [st["e2e_ms"][0] for _, st, _ in rr]
    tags = [f"{arm}_{grid}_r{rep}" for rep, _, _ in rr]
    b = [board(t) for t in tags]; b = [x for x in b if x]
    rss = max(ledger[t]["child_maxrss_kb"] for t in tags if t in ledger) / 1024 if any(t in ledger for t in tags) else float("nan")
    thr = rr[0][2][0]["threads"] if rr[0][2] else "?"
    n = sum(len(rows) for _, _, rows in rr)
    lines.append(f"| {arm} | {grid} | {thr} | {len(rr)}×{n // len(rr)} | {cell('s0_decode_ms')} | {cell('s1_project_ms')} | {cell('s2_rays_ms')} | "
                 f"{cell('s1s2_fused_ms')} | {cell('s3_object_ms')} | {cell('e2e_ms')} | {min(e2e_meds):.2f}–{max(e2e_meds):.2f} | {rss:.0f} | "
                 f"{statistics.median(x['board_cpu_busy_pct_of_6'] for x in b) if b else '—'} | {statistics.median(x['vdd_in_mw_mean'] for x in b) if b else '—'} |")
    out["arms"][f"{arm}_{grid}"] = {"runs": [{"rep": rep, **{s: {"median": st[s][0], "p95": st[s][1], "n": st[s][2]} for s in st}} for rep, st, _ in rr],
                                    "pooled": {s: {"median": statistics.median(pooled[s]), "p95": pct(pooled[s], .95), "n": len(pooled[s])} for s in STAGES if pooled[s]},
                                    "board": b, "peak_rss_mb": rss,
                                    "workload": {k: rr[0][2][0][k] for k in ("accepted", "invalid", "outside", "free_voxels", "hit_voxels", "mask_valid", "roi_valid", "candidate_mask", "candidate_roi")} if rr[0][2] else {}}
json.dump(out, open(os.path.join(d, "summary.json"), "w"), indent=1)
open(os.path.join(d, "summary.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
print(json.dumps(out["idle_before"]))
