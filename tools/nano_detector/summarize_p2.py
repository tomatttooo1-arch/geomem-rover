"""Summarise a run_p2.sh output directory (same table style as nano_baseline/summarize_paced.py).

Geometry phases: per-frame e2e from the bench CSV, late starts from the bench stdout (ledger),
process CPU from getrusage of the child (ledger). Detector phases: per-frame JSON lines of
nano_detector.py (warm-up frames excluded); in a mixed phase only the detector frames whose result
time falls inside the geometry window are counted, so both sides describe the same overlap.
Whole-board CPU / RAM / VDD_IN / GPU busy from tegrastats (500 ms, 1 s time stamps; the first 2 s
and last 1 s of each window are dropped), GPU clock from the devfreq sampler.
Usage: summarize_p2.py <run_dir>
"""
import csv, glob, json, os, re, statistics, sys
from datetime import datetime

d = sys.argv[1]


def pct(v, q):
    v = sorted(v); return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]


def tri(v):
    return f"{statistics.median(v):.2f}/{pct(v, .95):.2f}/{max(v):.2f}" if v else "—"


def duo(v):
    return f"{statistics.median(v):.2f}/{pct(v, .95):.2f}" if v else "—"


ph = {}
for line in open(os.path.join(d, "phases.log")):
    t, *rest = line.split(); rest = " ".join(rest)
    m = re.match(r"(start|end) (\S+)", rest)
    if m:
        ph.setdefault(m.group(2), {})[m.group(1)] = float(t)
ts = []
for line in open(os.path.join(d, "tegrastats.log")):
    m = re.match(r"(\d\d-\d\d-\d{4} \d\d:\d\d:\d\d) RAM (\d+)/(\d+)MB.*CPU \[([^\]]+)\] GR3D_FREQ (\d+)%.*VDD_IN (\d+)mW", line)
    if not m:
        continue
    t = datetime.strptime(m.group(1), "%m-%d-%Y %H:%M:%S").timestamp()
    cores = [c.split("@") for c in m.group(4).split(",")]
    ts.append((t, int(m.group(2)), [float(c[0].rstrip("%")) for c in cores], [int(c[1]) for c in cores],
               int(m.group(6)), int(m.group(5))))
gf = []
if os.path.exists(os.path.join(d, "gpufreq.log")):
    for line in open(os.path.join(d, "gpufreq.log")):
        p = line.split()
        if len(p) == 2 and p[1].isdigit():
            gf.append((float(p[0]), int(p[1])))


def board(t0, t1):
    s = [x for x in ts if t0 + 2 <= x[0] <= t1 - 1]
    if not s:
        return None
    g = [f for t, f in gf if t0 + 2 <= t <= t1 - 1]
    n = len(s[0][2])
    return {"n": len(s), "cpu_busy_core_equiv": round(statistics.mean(sum(x[2]) / 100 for x in s), 2),
            "per_core_util": [round(statistics.mean(x[2][i] for x in s)) for i in range(n)],
            "per_core_mhz": [int(statistics.median(x[3][i] for x in s)) for i in range(n)],
            "ram_used_mb": max(x[1] for x in s), "vdd_in_mw": round(statistics.mean(x[4] for x in s)),
            "gpu_busy_pct": round(statistics.mean(x[5] for x in s), 1),
            "gpu_mhz_median": round(statistics.median(g) / 1e6) if g else None}


led = {}
for line in open(os.path.join(d, "ledger.jsonl")):
    j = json.loads(line); led[j["tag"]] = j
idle = board(ph["idle"]["start"], ph["idle"]["end"])


def bcols(b):
    if not b:
        return "— | — | — | — | — | —"
    return (f"{b['cpu_busy_core_equiv']} | {round(b['cpu_busy_core_equiv'] - idle['cpu_busy_core_equiv'], 2)} | {b['ram_used_mb']} | "
            f"{b['vdd_in_mw']} ({b['vdd_in_mw'] - idle['vdd_in_mw']}) | {b['gpu_busy_pct']} | {b['gpu_mhz_median']}")


out = {"idle": idle, "geometry": {}, "detector": {}}
G = ["| 几何阶段 | 放置 | 帧数 | 端到端 中位/p95/max ms | 迟发帧 | 进程 CPU %单核 | CPU ms/帧 | 整板 CPU 忙(核当量) | 整板−空闲(核当量) | RAM MB | VDD_IN mW (−空闲) | GPU 忙 % | GPU MHz 中位 |",
     "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
for tag in sorted(glob.glob(os.path.join(d, "csv", "*.csv"))):
    tag = os.path.basename(tag)[:-4]
    phase = tag[:-4] if tag.endswith("_geo") else tag
    if phase not in ph or "end" not in ph[phase]:
        continue
    e = [float(r["e2e_ms"]) for r in csv.DictReader(open(os.path.join(d, "csv", tag + ".csv")))]
    j = led.get(tag, {}); cpu_s = j.get("child_user_s", 0) + j.get("child_sys_s", 0)
    mm = re.search(r'"paced_late_starts": (\d+)', " ".join(j.get("stdout", [])))
    thr = re.search(r'"threads": (\d+)', " ".join(j.get("stdout", [])))
    pl = ("核0-3, " if "c0" in tag else "不钉核, ") + (thr.group(1) if thr else "?") + " 线程"
    b = board(ph[phase]["start"], ph[phase]["end"])
    cpu_pct = 100 * cpu_s / j["wall_s"] if j else float("nan")
    G.append(f"| {phase}{' (+识别)' if phase.startswith('mix') else ''} | {pl} | {len(e)} | {tri(e)} | {mm.group(1) if mm else '—'} | "
             f"{cpu_pct:.1f} | {1000 * cpu_s / len(e):.2f} | {bcols(b)} |")
    out["geometry"][phase] = {"n": len(e), "e2e": {"median": statistics.median(e), "p95": pct(e, .95), "max": max(e)},
                              "late": int(mm.group(1)) if mm else None, "proc_cpu_pct_one_core": round(cpu_pct, 1),
                              "cpu_ms_per_frame": round(1000 * cpu_s / len(e), 2), "placement": pl, "board": b}

D = ["| 识别阶段 | 输入 | 模式 | 帧数 | 帧率 Hz | 端到端(SDK收帧→结果) 中位/p95/max ms | 取帧→结果 中位/p95 | 前处理 | 推理 | 后处理 | 迟发(>周期) | 漏掉的计划帧 | 进程 CPU %单核 | CPU ms/帧 | 整板 CPU 忙(核当量) | 整板−空闲(核当量) | RAM MB | VDD_IN mW (−空闲) | GPU 忙 % | GPU MHz 中位 |",
     "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
for f in sorted(glob.glob(os.path.join(d, "det", "*.jsonl"))):
    tag = os.path.basename(f)[:-6]
    sm = json.load(open(os.path.join(d, "det", tag + ".summary.json")))
    recs = [json.loads(l) for l in open(f)]
    recs = [r for r in recs if not r["warmup"]]
    mix = tag.endswith("_det")
    if mix:
        g = ph[tag[:-4]]; w0, w1 = g["start"], g["end"]
        recs = [r for r in recs if w0 * 1e6 <= r["done_us"] <= w1 * 1e6]
    else:
        w0, w1 = ph[tag]["start"], ph[tag]["end"]
    if not recs:
        continue
    b = board(w0, w1)
    wall = (recs[-1]["done_us"] - recs[0]["done_us"]) / 1e6
    col = lambda k: [r[k] for r in recs]
    late = sum(r["late"] for r in recs); skipped = sum(r["skipped"] for r in recs)
    name = tag[:-4] + " (+几何)" if mix else tag
    cam = "" if "cam15" not in tag else " (相机 15 帧/秒)"
    D.append(f"| {name}{cam} | {sm['input']} | {'定速 15 Hz' if sm['mode'] == 'paced' else '尽力'} | {len(recs)} | {(len(recs) - 1) / wall:.2f} | "
             f"{tri(col('e2e_ms'))} | {duo(col('proc_ms'))} | {duo(col('pre_ms'))} | {duo(col('infer_ms'))} | {duo(col('post_ms'))} | "
             f"{late if sm['mode'] == 'paced' else '—'} | {skipped} | {sm['proc_cpu_pct_one_core']}{'*' if mix else ''} | {sm['cpu_ms_per_frame']}{'*' if mix else ''} | {bcols(b)} |")
    out["detector"][tag] = {"n": len(recs), "fps": round((len(recs) - 1) / wall, 2), "mode": sm["mode"], "input": sm["input"],
                            **{k: {"median": statistics.median(col(k)), "p95": pct(col(k), .95), "max": max(col(k))}
                               for k in ("e2e_ms", "proc_ms", "pre_ms", "infer_ms", "post_ms")},
                            "late": late, "skipped": skipped, "proc_cpu_pct_one_core_whole_run": sm["proc_cpu_pct_one_core"],
                            "cpu_ms_per_frame_whole_run": sm["cpu_ms_per_frame"], "board": b, "detector_summary": sm,
                            "dets_per_frame_median": statistics.median(len(r["dets"]) for r in recs)}
json.dump(out, open(os.path.join(d, "summary.json"), "w"), indent=1)
txt = ["## 几何（M7 CPU 链，TUM 5 帧循环，30 Hz 定速）", ""] + G + ["", "## 识别（YOLOv8n-seg TensorRT FP16，Gemini 2 彩色 1280×720 RGB 实时流，钉核 4,5）", ""] + D + [
    "", "\\* 混跑行的识别进程 CPU 是识别整个 70 s 窗口的平均（其中约 60 s 与几何重叠）；其余列只统计重叠窗口。",
    f"", f"空闲基线：整板 CPU 忙 {idle['cpu_busy_core_equiv']} 核当量，VDD_IN {idle['vdd_in_mw']} mW，RAM {idle['ram_used_mb']} MB，GPU 忙 {idle['gpu_busy_pct']} %，GPU {idle['gpu_mhz_median']} MHz。"]
open(os.path.join(d, "summary.md"), "w").write("\n".join(txt) + "\n")
print("\n".join(txt))
