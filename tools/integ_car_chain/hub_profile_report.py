#!/usr/bin/env python3
"""Summarize a hub FPGA_HUB_PROFILE directory (host/hub/hub_profile.py) into small text.

  * per Python thread: CPU seconds over its life (thread_time), grouped by thread role;
  * per kernel task (tasks.jsonl snapshots): CPU over a steady window [--from-s, --to-s] after the
    first snapshot, Python threads named via native_id, the rest are native (DDS / middleware);
  * with --top N and .prof files (cprofile mode): per role, the top functions by own time and
    by cumulative time.

usage: hub_profile_report.py DIR [--from-s 20] [--to-s 100] [--top 15] [--json OUT]
"""

import argparse
import collections
import glob
import json
import os
import re


def role(name):
    name = name or "?"
    if name.startswith("hub-sender"):
        return "hub-sender"
    if name.startswith("hub-reader"):
        return "hub-reader"
    return re.sub(r"[-_ ]?\(.*\)$", "", name)


def threads(d):
    recs = []
    for path in glob.glob(os.path.join(d, "*.json")):
        with open(path) as f:
            try:
                recs.append(json.load(f))
            except ValueError:               # a record cut at interpreter exit
                continue
    return recs


def task_window(d, t_from, t_to):
    path = os.path.join(d, "tasks.jsonl")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        snaps = [json.loads(line) for line in f if line.strip()]
    if len(snaps) < 2:
        return None
    m0 = snaps[0]["mono"]
    win = [s for s in snaps if t_from <= s["mono"] - m0 <= t_to]
    if len(win) < 2:
        win = [snaps[0], snaps[-1]]
    a, b = win[0], win[-1]
    tck = float(a["clk_tck"])
    wall = b["mono"] - a["mono"]
    names = {}
    for s in snaps:
        names.update(s.get("py") or {})
    rows = collections.Counter()
    for tid, (comm, ut, st) in b["tasks"].items():
        u0, s0 = (a["tasks"].get(tid) or [comm, 0, 0])[1:]
        label = "py:" + role(names[tid]) if tid in names else "native:" + comm
        rows[label] += (ut + st - u0 - s0) / tck
    total = sum(rows.values())
    return {"window_s": wall, "from_s": a["mono"] - m0, "to_s": b["mono"] - m0,
            "total_cpu_s": total, "total_pct_one_core": 100.0 * total / wall if wall else None,
            "by_label_pct_one_core": dict((k, round(100.0 * v / wall, 3)) for k, v in rows.most_common())}


def prof_tops(d, top):
    import pstats
    by_role = collections.defaultdict(list)
    for path in glob.glob(os.path.join(d, "*.prof")):
        base = os.path.basename(path)[:-5]
        by_role[role(base.rsplit("-", 1)[0])].append(path)
    out = {}
    for r, paths in sorted(by_role.items()):
        st = pstats.Stats(paths[0])
        for p in paths[1:]:
            st.add(p)
        total = st.total_tt
        rows = []
        for (fn, line, func), (cc, nc, tt, ct, callers) in st.stats.items():
            rows.append((tt, ct, nc, "%s:%d:%s" % (os.path.basename(fn), line, func)))
        own = sorted(rows, reverse=True)[:top]
        cum = sorted(rows, key=lambda x: -x[1])[:top]
        out[r] = {"total_s": total, "n_threads": len(paths),
                  "own": [{"s": round(a, 4), "pct": round(100 * a / total, 1) if total else None, "calls": n, "fn": f}
                          for a, c, n, f in own],
                  "cum": [{"s": round(c, 4), "pct": round(100 * c / total, 1) if total else None, "calls": n, "fn": f}
                          for a, c, n, f in cum]}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--from-s", type=float, default=20.0)
    ap.add_argument("--to-s", type=float, default=1e9)
    ap.add_argument("--top", type=int, default=0)
    ap.add_argument("--json")
    a = ap.parse_args()
    recs = threads(a.dir)
    life = collections.Counter()
    for r in recs:
        life[role(r["name"])] += r["cpu_s"]
    rep = {"threads_life_cpu_s": dict((k, round(v, 3)) for k, v in life.most_common()),
           "main": next((r for r in recs if r["name"] == "MainThread"), None),
           "window": task_window(a.dir, a.from_s, a.to_s)}
    if a.top:
        rep["prof"] = prof_tops(a.dir, a.top)
    text = json.dumps(rep, indent=1)
    if a.json:
        with open(a.json, "x") as f:
            f.write(text)
    print(text)


if __name__ == "__main__":
    main()
