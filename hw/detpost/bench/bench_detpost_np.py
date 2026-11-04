"""A53 上 NumPy 版检测后处理计时（价值闸的第三个读数：Python 产品驱动若直接用 NumPy 做后处理的代价）。
实现就是浮点参考 detpost_float.postprocess（ultralytics 语义，对 8400 格全量向量化解码）。
用法：python3 bench_detpost_np.py --iters 300 --cpu 3 --nc 33 --out res.json f1.bin [f2.bin ...]
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import detpost_float as dfl  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames", nargs="+")
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--warmup", type=int, default=10)
    ap.add_argument("--cpu", type=int, default=-1)
    ap.add_argument("--nc", type=int, default=33)
    ap.add_argument("--period-ms", type=float, default=0.0)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    if a.cpu >= 0:
        os.sched_setaffinity(0, {a.cpu})
    bufs = [open(f, "rb").read() for f in a.frames]
    ts, nb = [], []
    nxt = time.monotonic()
    for it in range(-a.warmup, a.iters):
        buf = bufs[(it + a.warmup) % len(bufs)]
        t0 = time.perf_counter()
        r = dfl.postprocess(buf, a.nc)
        t1 = time.perf_counter()
        if it >= 0:
            ts.append((t1 - t0) * 1e3)
            nb.append(len(r["anchor"]))
        if a.period_ms > 0:
            nxt += a.period_ms / 1e3
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            else:
                nxt = time.monotonic()
    t = np.asarray(ts)
    s = dict(variant="numpy", iters=len(ts), frames=len(bufs), cpu=a.cpu, period_ms=a.period_ms,
             ms_p50=float(np.percentile(t, 50)), ms_p90=float(np.percentile(t, 90)), ms_p99=float(np.percentile(t, 99)),
             ms_max=float(t.max()), ms_mean=float(t.mean()), boxes_mean=float(np.mean(nb)), numpy=np.__version__)
    print(json.dumps(s))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(dict(summary=s, ms=[round(x, 4) for x in ts]), f)


if __name__ == "__main__":
    main()
