#!/usr/bin/env python3
"""Turn the sweep JSONs into the tables of M7_RAY_LANES_A2_DESIGN_20261011.md.

All cycle figures are MODEL numbers (tools/m7_lanes/sweep.py). ms = cycles / 1e5 (100 MHz).
Usage: summarize.py --dir runs/m7-ray-lanes-design-20261011 > tables.md
Also writes <dir>/summary.json.
"""
import argparse
import glob
import json
import os

BUDGET = 3_300_000


def load(d):
    out = {}
    for path in sorted(glob.glob(os.path.join(d, "sweep_*.json"))):
        r = json.load(open(path))
        out.setdefault(r["dataset"], {}).update(r["configs"])
        out.setdefault("_meta", {})[os.path.basename(path)] = {k: r[k] for k in ("frames", "fixed_cycles", "budget_cycles")}
    return out


def k(pol, n, d, ii=1, sc=1, ro=0, q=512):
    return "%s|N%d|D%d|Q%d|ii%d|sc%d|ro%d" % (pol, n, d, q, ii, sc, ro)


def ms(c):
    return "%.2f" % (c / 1e5)


def row(S, ds, key):
    s = S[ds].get(key)
    if not s:
        return None
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    a = ap.parse_args()
    S = load(a.dir)
    summary = {"budget_cycles": BUDGET, "min_lanes": {}}
    print("## policies, nominal costs (conv II=1, 1 cycle/step, 0 overhead), D=8 (direct: no lane FIFO)\n")
    print("| policy | N | TUM mean ms | TUM p99 ms | TUM max ms | TUM worst margin | rec30 max ms | walk/LB TUM mean | tail TUM max (cycles) |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for pol in ("rr", "fnf", "idle", "direct", "lwl_oracle", "interleave", "tile_col", "tile_row_mc"):
        for n in (1, 2, 4, 8, 16):
            d = 1 if pol in ("tile_row_mc", "direct") else 8
            t, r = row(S, "tum", k(pol, n, d)), row(S, "rec30", k(pol, n, d))
            if not t:
                continue
            print("| %s | %d | %s | %s | %s | %.2f | %s | %.3f | %d |" % (
                pol, n, ms(t["total_cycles"]["mean"]), ms(t["total_cycles"]["p99"]), ms(t["total_cycles"]["max"]),
                t["margin_worst"], ms(r["total_cycles"]["max"]) if r else "-",
                t["walk_over_lower_bound"]["mean"] / 1000, t["tail"]["max"]))
    print("\n## FIFO depth (nominal costs), TUM max ms / rec30 max ms\n")
    print("| policy | N | D=1 | D=2 | D=8 | D=32 | D=128 |")
    print("|---|---:|---:|---:|---:|---:|---:|")
    for pol in ("rr", "idle", "tile_col", "interleave"):
        for n in (4, 8, 16):
            cells = []
            for d in (1, 2, 8, 32, 128):
                t, r = row(S, "tum", k(pol, n, d)), row(S, "rec30", k(pol, n, d))
                cells.append("%s / %s" % (ms(t["total_cycles"]["max"]), ms(r["total_cycles"]["max"])) if t else "-")
            print("| %s | %d | %s |" % (pol, n, " | ".join(cells)))
    print("\n## cost sensitivity, TUM worst frame (ms, margin = 3.3M / worst total)\n")
    print("| policy | N | nominal | conv II=3 | 2 cycles/step | +1 cycle/ray | all three |")
    print("|---|---:|---:|---:|---:|---:|---:|")
    sens = [(1, 1, 0), (3, 1, 0), (1, 2, 0), (1, 1, 1), (3, 2, 1)]
    for pol in ("rr", "idle", "direct"):
        for n in (4, 8, 16):
            cells = []
            for ii, sc, ro in sens:
                d = 1 if pol == "direct" else 8
                t = row(S, "tum", k(pol, n, d, ii, sc, ro))
                cells.append("%s (%.2f)" % (ms(t["total_cycles"]["max"]), t["margin_worst"]) if t else "-")
            print("| %s | %d | %s |" % (pol, n, " | ".join(cells)))
    print("\n## minimum lanes for TUM worst frame <= 3.3 M cycles\n")
    print("| policy | costs (conv II, cycles/step, +cycles/ray) | min N | worst ms at min N | margin |")
    print("|---|---|---:|---:|---:|")
    for pol in ("rr", "idle", "direct"):
        for ii, sc, ro in sens:
            d = 1 if pol == "direct" else 8
            best = None
            for n in (1, 2, 4, 8, 16):
                t = row(S, "tum", k(pol, n, d, ii, sc, ro))
                if t is None and (ii, sc, ro) == (1, 1, 0):
                    continue
                if t and t["total_cycles"]["max"] <= BUDGET:
                    best = (n, t)
                    break
            if best:
                n, t = best
                summary["min_lanes"]["%s|ii%d|sc%d|ro%d" % (pol, ii, sc, ro)] = {
                    "lanes": n, "worst_cycles": t["total_cycles"]["max"], "margin": t["margin_worst"],
                    "note": "searched N in {4,8,16} only for non-nominal costs" if (ii, sc, ro) != (1, 1, 0) else ""}
                print("| %s | %d, %d, %d | %d | %s | %.2f |" % (pol, ii, sc, ro, n, ms(t["total_cycles"]["max"]),
                                                             t["margin_worst"]))
            else:
                print("| %s | %d, %d, %d | >16 | - | - |" % (pol, ii, sc, ro))
    summary["meta"] = S["_meta"]
    json.dump(summary, open(os.path.join(a.dir, "summary.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
