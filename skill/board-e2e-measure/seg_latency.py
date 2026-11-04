#!/usr/bin/env python3
"""seg_latency.py — 端到端时延分段汇总 + 单调性检查（通用版，带内置自测）。

输入：CSV，每行一帧；必须有 frame 列，以及按流水先后排列的时间戳列（同一单调时钟，纳秒），
例如 t_cam_ns,t_prep_ns,t_dpu_ns,t_post_ns。用 --stages 指定列的先后顺序。

输出：每段（相邻两列之差）和总时延（末列 - 首列）的中位数 / p95（毫秒），以及：
  - 违反单调：同一行里后一级时间戳小于前一级的行数
  - 帧序违反：按 frame 排序后首列时间戳不严格递增的次数
  - 预热剔除：--warmup-s 秒内（相对首帧首列）的行不计
违反行会被列出并剔除后重算；违反比例 > --max-bad（默认 0.1%）时退出码 2，表示这一段不作数。

用法：
  python3 seg_latency.py data.csv --stages t_cam_ns,t_prep_ns,t_dpu_ns,t_post_ns [--warmup-s 10]
  python3 seg_latency.py --selftest      # 在已知答案上校准这个脚本本身
"""
import argparse, csv, random, statistics, sys


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    k = (len(xs) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def summarize(rows, stages, warmup_s=0.0, max_bad=0.001, out=sys.stdout):
    rows = sorted(rows, key=lambda r: int(r["frame"]))
    t0 = min(int(r[stages[0]]) for r in rows)
    rows = [r for r in rows if int(r[stages[0]]) - t0 >= warmup_s * 1e9]
    bad_mono, bad_order = [], []
    prev = None
    for r in rows:
        ts = [int(r[s]) for s in stages]
        if any(b < a for a, b in zip(ts, ts[1:])):
            bad_mono.append(r["frame"])
        if prev is not None and ts[0] <= prev:
            bad_order.append(r["frame"])
        prev = ts[0]
    bad = set(bad_mono) | set(bad_order)
    good = [r for r in rows if r["frame"] not in bad]
    res = {"n": len(rows), "n_used": len(good), "bad_mono": bad_mono, "bad_order": bad_order}
    segs = list(zip(stages, stages[1:])) + [(stages[0], stages[-1])]
    for a, b in segs:
        d = [(int(r[b]) - int(r[a])) / 1e6 for r in good]
        res[f"{a}->{b}"] = (statistics.median(d) if d else float("nan"), pct(d, 0.95))
    print(f"rows={res['n']} used={res['n_used']} bad_monotonic={len(bad_mono)} bad_order={len(bad_order)}", file=out)
    if bad:
        print("  excluded frames: " + ",".join(sorted(bad, key=int)[:20]) + (" ..." if len(bad) > 20 else ""), file=out)
    for a, b in segs:
        m, p = res[f"{a}->{b}"]
        tag = "TOTAL " if (a, b) == (stages[0], stages[-1]) else "      "
        print(f"{tag}{a} -> {b}: median {m:.2f} ms  p95 {p:.2f} ms", file=out)
    res["valid"] = (len(bad) <= max_bad * max(len(rows), 1))
    if not res["valid"]:
        print(f"INVALID: {len(bad)} bad rows > {max_bad*100:.2f}% -> do not report this segment", file=out)
    return res


def _synthetic(n=1000, seed=1):
    rnd = random.Random(seed)
    rows, t = [], 1_000_000_000
    for f in range(n):
        t += 33_000_000
        a = t; b = a + 4_500_000 + rnd.randint(0, 200_000)
        c = b + 64_000_000 + rnd.randint(0, 2_000_000); d = c + 4_000_000 + rnd.randint(0, 500_000)
        rows.append({"frame": str(f), "t0": str(a), "t1": str(b), "t2": str(c), "t3": str(d)})
    return rows


def selftest():
    import io, copy
    st = ["t0", "t1", "t2", "t3"]; ok = True; sink = io.StringIO()
    base = summarize(_synthetic(), st, out=sink)
    # 1) 已知结构：总时延 = 各段之和（中位数近似），且没有违例
    ok &= base["bad_mono"] == [] and base["bad_order"] == [] and base["valid"]
    # 2) 判别力：最后一级时间戳整体 +5 ms，最后一段和总时延的中位数必须正好 +5 ms
    rows = _synthetic()
    for r in rows:
        r["t3"] = str(int(r["t3"]) + 5_000_000)
    sh = summarize(rows, st, out=sink)
    d_seg = sh["t2->t3"][0] - base["t2->t3"][0]; d_tot = sh["t0->t3"][0] - base["t0->t3"][0]
    ok &= abs(d_seg - 5.0) < 1e-6 and abs(d_tot - 5.0) < 1e-6
    # 3) 插一个违反单调的时间戳，必须被查出并剔除
    rows = _synthetic(); rows[500]["t2"] = str(int(rows[500]["t1"]) - 1)
    m = summarize(rows, st, out=sink); ok &= m["bad_mono"] == ["500"] and m["n_used"] == m["n"] - 1
    # 4) 两帧首列时间戳对调（帧序错乱），必须被查出
    rows = _synthetic(); rows[300]["t0"], rows[301]["t0"] = rows[301]["t0"], rows[300]["t0"]
    o = summarize(rows, st, out=sink); ok &= len(o["bad_order"]) >= 1
    # 5) 违例过多 -> INVALID
    rows = _synthetic()
    for i in range(0, 1000, 50):
        rows[i]["t1"] = str(int(rows[i]["t0"]) - 1)
    v = summarize(rows, st, out=sink); ok &= not v["valid"]
    print("SELFTEST " + ("PASS" if ok else "FAIL") + f" (+5ms shift: seg {d_seg:.3f} ms, total {d_tot:.3f} ms)")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?"); ap.add_argument("--stages")
    ap.add_argument("--warmup-s", type=float, default=0.0); ap.add_argument("--max-bad", type=float, default=0.001)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.csv or not a.stages:
        ap.error("need csv and --stages (or --selftest)")
    with open(a.csv, newline="") as f:
        rows = list(csv.DictReader(f))
    r = summarize(rows, a.stages.split(","), a.warmup_s, a.max_bad)
    return 0 if r["valid"] else 2


if __name__ == "__main__":
    sys.exit(main())
