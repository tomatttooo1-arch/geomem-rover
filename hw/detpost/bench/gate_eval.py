"""检测后处理价值闸判读（阈值 2026-11-03 测量前冻结，与 DETPOST0/README.md §4 逐字一致；改阈值只能另开带日期的节）。

时延理由（任一条成立 = 「时延上值得做硬件」）：
  L1  ours 冷缓存单跑中位 T_sw >= 0.10 x DPU 中位 T_dpu
  L2  ours 与几何核并发、按识别帧率调用时的 p99 >= 5.0 ms
  L3  按识别帧率并发 ours 时，几何核 rec30 每帧 call_ms 中位比两次基线均值高 >= 3%，或 p99 高 >= 10%；
      且该增量 >= 3 x 两次基线之间的漂移（否则判「噪声内」）
灰区：L1 比值在 0.05–0.10 之间，或 L3 只在背靠背压力下成立 —— 不自动判，交主控。
「核心链路全在 FPGA」理由与本脚本无关，不在此判。
用法：python3 gate_eval.py --run OUT --dpu-timing dpu_timing.json [--frames-source dpu|model]
"""
import argparse
import csv
import json
import os

import numpy as np

L1_RATIO, L1_GREY = 0.10, 0.05
L2_P99_MS = 5.0
L3_MED, L3_P99, L3_NOISE_X = 0.03, 0.10, 3.0


def geo_stats(d):
    p = os.path.join(d, "geo", "frames.csv")
    rows = list(csv.DictReader(open(p)))
    out = {}
    for k in ("call_ms", "device_ms", "e2e_ms"):
        v = np.array([float(r[k]) for r in rows])
        out[k] = dict(p50=float(np.median(v)), p99=float(np.percentile(v, 99)), n=int(v.size))
    return out


def load(p):
    return json.load(open(p))["summary"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--dpu-timing", required=True)
    ap.add_argument("--frames-source", default="dpu", choices=["dpu", "model"])
    a = ap.parse_args()
    R = a.run
    t_dpu = json.load(open(a.dpu_timing))["ms_p50"]
    solo = {v: [load(os.path.join(R, "solo_%s_%s.json" % (v, s))) for s in "ab"] for v in ("ours", "vai", "numpy")}
    t_sw = float(np.mean([s["ms_p50"] for s in solo["ours"]]))
    ratio = t_sw / t_dpu
    l1 = ratio >= L1_RATIO
    conc_ours = load(os.path.join(R, "conc_ours_rate_post.json"))
    l2 = conc_ours["ms_p99"] >= L2_P99_MS
    b1, b2 = geo_stats(os.path.join(R, "geo_base1")), geo_stats(os.path.join(R, "geo_base2"))
    res_l3 = {}
    l3 = False
    for tag in ("ours_rate", "ours_stress", "vai_rate", "vai_stress"):
        g = geo_stats(os.path.join(R, "geo_" + tag))
        r = {}
        for q, thr in (("p50", L3_MED), ("p99", L3_P99)):
            base = (b1["call_ms"][q] + b2["call_ms"][q]) / 2
            drift = abs(b1["call_ms"][q] - b2["call_ms"][q])
            inc = g["call_ms"][q] - base
            r[q] = dict(base=base, drift=drift, cand=g["call_ms"][q], rel=inc / base,
                        hit=bool(inc / base >= thr and inc >= L3_NOISE_X * drift))
        r["device_ms_p50_rel"] = (g["device_ms"]["p50"] - (b1["device_ms"]["p50"] + b2["device_ms"]["p50"]) / 2) / \
            ((b1["device_ms"]["p50"] + b2["device_ms"]["p50"]) / 2)
        res_l3[tag] = r
        if tag == "ours_rate" and (r["p50"]["hit"] or r["p99"]["hit"]):
            l3 = True
    grey = (not l1 and ratio >= L1_GREY) or (not l3 and (res_l3["ours_stress"]["p50"]["hit"] or res_l3["ours_stress"]["p99"]["hit"]))
    verdict = dict(
        frames_source=a.frames_source,
        T_dpu_ms_p50=t_dpu, T_sw_ours_ms_p50=t_sw, ratio=ratio,
        T_sw_vai_ms_p50=float(np.mean([s["ms_p50"] for s in solo["vai"]])),
        T_sw_numpy_ms_p50=float(np.mean([s["ms_p50"] for s in solo["numpy"]])),
        L1=dict(hit=l1, thr=L1_RATIO), L2=dict(hit=l2, p99=conc_ours["ms_p99"], thr_ms=L2_P99_MS),
        L3=dict(hit=l3, detail=res_l3),
        time_reason_pass=bool(l1 or l2 or l3), grey_zone=bool(grey and not (l1 or l2 or l3)),
        note="frames_source=model 时读数只算线索（仿真 DPU 输出的候选数与真实输出可能不同），需用真实 DPU 输出复测才作数")
    print(json.dumps(verdict, indent=1))
    json.dump(verdict, open(os.path.join(R, "gate_verdict.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
