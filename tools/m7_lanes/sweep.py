#!/usr/bin/env python3
"""A2 cycle sweep: dispatch policy x lanes x FIFO depth x cost assumptions.

Per-cycle MODEL (tools/m7_lanes/lanes_model.cpp::simulate); no number here is
a measurement. Frame total = FIXED + walk-phase cycles, FIXED = clear 32,768
(two 32Kx1 planes per lane cleared in parallel, II=1) + merge/write-out 32,768
(1 flag byte per cycle) + 200 pipeline fill allowance.
Budget: 33 ms at 100 MHz = 3,300,000 cycles.

Usage: sweep.py --build FRESH --out-dir DIR [--datasets rec30,tum] [--jobs 44] [--quick]
"""
import argparse
import csv
import itertools
import json
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.m7_lanes import lanes as L  # noqa: E402

FIXED = 32768 + 32768 + 200
BUDGET = 3_300_000
LANES = (1, 2, 4, 8, 16)
SINGLE_CONV = ("rr", "fnf", "idle", "lwl_oracle", "tile_col", "interleave")
DEPTHS = (2, 8, 32)
SENS = [(1, 1, 0), (3, 1, 0), (1, 2, 0), (1, 1, 1), (3, 2, 1)]  # (conv_ii, step_cycles, ray_overhead)


def configs():
    out = []
    for pol, n, d in itertools.product(SINGLE_CONV, LANES, DEPTHS):
        out.append(dict(policy=pol, lanes=n, fifo=d, sray=512, conv_ii=1, step_cycles=1, ray_overhead=0))
    for n in LANES:
        out.append(dict(policy="tile_row_mc", lanes=n, fifo=1, sray=512, conv_ii=1, step_cycles=1, ray_overhead=0))
    for pol, n, (ii, sc, ro) in itertools.product(("rr", "idle"), (4, 8, 16), SENS[1:]):
        out.append(dict(policy=pol, lanes=n, fifo=8, sray=512, conv_ii=ii, step_cycles=sc, ray_overhead=ro))
    # FIFO-depth extremes for the two buildable best candidates
    for pol, n, d in itertools.product(("rr", "idle"), (4, 8, 16), (1, 128)):
        out.append(dict(policy=pol, lanes=n, fifo=d, sray=512, conv_ii=1, step_cycles=1, ray_overhead=0))
    # variant B (one process, lanes unrolled, first idle lane takes the s_ray head; no lane FIFOs)
    for n in LANES:
        out.append(dict(policy="direct", lanes=n, fifo=1, sray=512, conv_ii=1, step_cycles=1, ray_overhead=0))
    for n, (ii, sc, ro) in itertools.product((4, 8, 16), SENS[1:]):
        out.append(dict(policy="direct", lanes=n, fifo=1, sray=512, conv_ii=ii, step_cycles=sc, ray_overhead=ro))
    return out


def key(c):
    return "%s|N%d|D%d|Q%d|ii%d|sc%d|ro%d" % (c["policy"], c["lanes"], c["fifo"], c["sray"], c["conv_ii"],
                                             c["step_cycles"], c["ray_overhead"])


_G = {}


def _init(so, dsn, cfgs):
    _G["lm"] = L.Lanes(so)
    _G["ds"] = L.open_dataset(dsn)
    _G["cfgs"] = cfgs


def _frame(f):
    ds, lm = _G["ds"], _G["lm"]
    z = L.depth_frame(ds, f)
    out = []
    with lm.prepare(z, ds["ik"], L.WC, **L.GRID) as p:
        for c in _G["cfgs"]:
            _, _, st = p.run(functional=False, **c)
            lb = st["lane_busy"]
            out.append((f, key(c), st["walk_cycles"], st["conv_done"], st["lower_bound"], st["tail"],
                        st["conv_stall"], st["max_fifo_occ"], st["work"], max(lb), min(lb)))
    return out


def agg(vals):
    a = np.asarray(vals, dtype=np.float64)
    return {"mean": round(float(a.mean())), "p50": round(float(np.percentile(a, 50))),
            "p95": round(float(np.percentile(a, 95))), "p99": round(float(np.percentile(a, 99))),
            "max": int(a.max())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--datasets", default="rec30,tum")
    ap.add_argument("--jobs", type=int, default=44)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--policies", help="comma list: only these policies (output files get a _<tag> suffix)")
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    so = L.build(a.build)
    os.makedirs(a.out_dir, exist_ok=True)
    cfgs = configs()
    if a.policies:
        cfgs = [c for c in cfgs if c["policy"] in a.policies.split(",")]
    sfx = ("_" + a.tag) if a.tag else ""
    for dsn in a.datasets.split(","):
        ds = L.open_dataset(dsn)
        frames = list(range(ds["N"]))
        if a.quick:
            frames = frames[::max(1, len(frames) // 12)]
        t0 = time.time()
        with Pool(a.jobs, initializer=_init, initargs=(so, dsn, cfgs)) as pool:
            res = pool.map(_frame, frames, chunksize=1)
        rows = [r for fr in res for r in fr]
        by = {}
        for r in rows:
            by.setdefault(r[1], []).append(r)
        summary = {}
        for c in cfgs:
            k = key(c)
            rr = by[k]
            walk = [r[2] for r in rr]
            total = [r[2] + FIXED for r in rr]
            ratio = [r[2] / max(1, r[4]) for r in rr]
            imb = [r[9] / max(1, r[10]) for r in rr]
            worst = max(rr, key=lambda r: r[2])
            summary[k] = dict(cfg=c, frames=len(rr), walk_cycles=agg(walk), total_cycles=agg(total),
                              total_ms_max=round((worst[2] + FIXED) / 1e5, 2),
                              frames_over_budget=int(sum(t > BUDGET for t in total)),
                              margin_worst=round(BUDGET / (worst[2] + FIXED), 3),
                              walk_over_lower_bound=agg([x * 1000 for x in ratio]),  # per mille
                              tail=agg([r[5] for r in rr]), conv_stall=agg([r[6] for r in rr]),
                              max_fifo_occ=int(max(r[7] for r in rr)),
                              lane_busy_max_over_min_x1000=agg([x * 1000 for x in imb]),
                              worst_frame=worst[0])
        rep = {"dataset": dsn, "frames": len(frames), "source": "MODEL tools/m7_lanes/sweep.py (per-cycle sim)",
               "fixed_cycles": FIXED, "budget_cycles": BUDGET, "elapsed_s": round(time.time() - t0, 1),
               "configs": summary}
        json.dump(rep, open(os.path.join(a.out_dir, "sweep_%s%s.json" % (dsn, sfx)), "w"), indent=1)
        with open(os.path.join(a.out_dir, "sweep_%s%s.csv" % (dsn, sfx)), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["policy", "lanes", "fifo", "sray", "conv_ii", "step_cycles", "ray_overhead", "frames",
                        "total_mean", "total_p99", "total_max", "ms_max", "frames_over_budget", "margin_worst",
                        "walk_over_lb_permille_mean", "walk_over_lb_permille_max", "tail_max", "worst_frame"])
            for k, s in summary.items():
                c = s["cfg"]
                w.writerow([c["policy"], c["lanes"], c["fifo"], c["sray"], c["conv_ii"], c["step_cycles"],
                            c["ray_overhead"], s["frames"], s["total_cycles"]["mean"], s["total_cycles"]["p99"],
                            s["total_cycles"]["max"], s["total_ms_max"], s["frames_over_budget"], s["margin_worst"],
                            s["walk_over_lower_bound"]["mean"], s["walk_over_lower_bound"]["max"], s["tail"]["max"],
                            s["worst_frame"]])
        print("%s: %d frames x %d configs in %.0f s" % (dsn, len(frames), len(cfgs), time.time() - t0))


if __name__ == "__main__":
    main()
