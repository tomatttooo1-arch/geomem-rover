#!/usr/bin/env python3
"""Hub CPU per run and per arm from host-container chain runs (tools: tf_probe sys.csv + hub_stats).

Per run directory (…/<tag>/chain): two readings of the hub process CPU, in % of one core:
  * window: sys.csv ``hub`` column (cumulative utime+stime ticks) over the rig's measurement
    window [cpu.json t0_wall, t1_wall] (the producer/board_probe window, after warm-up);
  * life: hub_stats.json cpu_times_s / wall_s (whole hub lifetime incl. start-up and teardown).
Plus mean load1 in the window, tick gap p50/p99, and the hub's per-topic callback counts.

usage: hub_cpu_ab.py ARM=TAG,TAG,... [ARM=...] [--root DIR] [--json OUT]
"""

import argparse
import csv
import json
import os
import statistics


def run_stats(d):
    chain = os.path.join(d, "chain")
    with open(os.path.join(chain, "rig", "cpu.json")) as f:
        cpu = json.loads(f.read().strip().splitlines()[-1])
    t0, t1 = cpu["t0_wall"], cpu["t1_wall"]
    rows = list(csv.DictReader(open(os.path.join(chain, "rig", "sys.csv"))))
    win = [r for r in rows if t0 <= float(r["wall"]) <= t1 and r.get("hub") not in (None, "", "NA")]
    a, b = win[0], win[-1]
    wall = float(b["wall"]) - float(a["wall"])
    tck = os.sysconf("SC_CLK_TCK")
    pct = 100.0 * (float(b["hub"]) - float(a["hub"])) / tck / wall
    with open(os.path.join(chain, "hub_stats.json")) as f:
        hs = json.load(f)
    life = 100.0 * sum(hs["cpu_times_s"]) / hs["wall_s"]
    loads = [float(r["load1"]) for r in win]
    cb = dict((k, v) for k, v in hs["counts"].items() if k.startswith("cb:"))
    errs = dict((k, v) for k, v in hs["counts"].items() if "error" in k or "overrun" in k)
    return {"window_s": round(wall, 1), "hub_pct_window": round(pct, 2), "hub_pct_life": round(life, 2),
            "life_wall_s": round(hs["wall_s"], 1), "load1_mean": round(statistics.mean(loads), 1),
            "tick_gap_ms": hs["tick_gap_ms"], "ticks": hs["ticks"], "cb_counts": cb, "errors": errs,
            "producer_pct": cpu.get("producer_pct_one_core")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("arms", nargs="+")
    ap.add_argument("--root", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--json")
    a = ap.parse_args()
    out = {}
    for spec in a.arms:
        arm, tags = spec.split("=", 1)
        runs = {}
        for t in tags.split(","):
            try:
                runs[t] = run_stats(os.path.join(a.root, t))
            except (OSError, KeyError, IndexError, ValueError) as exc:
                runs[t] = {"error": "%s: %s" % (type(exc).__name__, exc)}
        ok = [r for r in runs.values() if "error" not in r]
        agg = {}
        for k in ("hub_pct_window", "hub_pct_life", "load1_mean"):
            v = [r[k] for r in ok]
            agg[k] = {"mean": round(statistics.mean(v), 2) if v else None,
                      "sd": round(statistics.stdev(v), 2) if len(v) > 1 else None, "values": v}
        out[arm] = {"n": len(ok), "agg": agg, "runs": runs}
    text = json.dumps(out, indent=1)
    if a.json:
        with open(a.json, "x") as f:
            f.write(text)
    print(text)


if __name__ == "__main__":
    main()
