"""CTRLPS5 dose matrix over tools/kv_nav/sim/lockstep_loop.py, both arms of detour_nonfinal_extend (0 = CTRLPS1 rule,
1 = fix), per goal-2 operating point (OPS: room = world.json's goals 1-3; A / B = goals 1-2 with goal 2 moved so the
window entry looks like the container's, see OPS). Groups (each = one timing knob, the others at the ideal 0):
  phase   core ticker phase 0..45 ms (5 ms) x goal gap 1.0..2.0 s (0.25 s)          -- deterministic
  jitter  per-step lateness U(0, D), D = 0 / 20 / 50 / 100 ms, seeds 0..N-1
  odom    pose age D = 0 / 20 / 50 / 100 ms x goal gap 1.0..2.0 s
  slip    chassis tick miss probability 0 / 0.03 / 0.09, seeds 0..N-1
Writes OUT/runs.jsonl (one line per run) and OUT/summary.json (per group / dose / arm: goal-2 detour count,
goal-2 time, transition ratio new / old remaining, >= 253 ticks, all-goal outcomes).
summary also gives the window-entry ratio under the fixed rule: after / (before + old path end -> goal).
Usage: python3 tools/kv_nav/sim/lockstep_sweep.py OUT [--seeds 20] [--jobs 12] [--lib LIB] [--ops room,A,B]"""
import argparse
import json
import multiprocessing as mp
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from tools.kv_nav.sim import lockstep_loop as L  # noqa: E402

_ROUTER = None


def one(job):
    global _ROUTER
    group, dose, arm, kw, lib = job
    kw = dict(kw)
    extra = kw.pop("_params", {})
    if _ROUTER is None:
        _ROUTER = L.Router(kw.get("world") or str(ROOT / "tests/kv_nav_real/world.json"))
    argv = ["--lib", lib, "--params", json.dumps(dict(extra, detour_nonfinal_extend=arm))]
    for k, v in kw.items():
        argv += ["--" + k.replace("_", "-"), str(v)]
    a = L.parser().parse_args(argv)
    r = L.run(a, router=_ROUTER)
    g2 = next((g for g in r["goals"] if g["goal_index"] == 2), {})
    tr = [j for j in g2.get("rem_jumps", []) if j["t"] > 1.0]
    return {"group": group, "dose": dose, "arm": arm, "kw": kw,
            "goals": [{k: g.get(k) for k in ("goal_index", "status", "elapsed_s", "err_xy", "detour", "recover_ticks",
                                              "unsafe", "path_len_m")} for g in r["goals"]],
            "g2_transition": tr[0] if tr else None}


GOAL1 = "1.0 1.2 1.57"
# goal 2 operating points. room: the world's own goal 2 (2.2, -1.4) -- with the NavFn stand-in its route's last leg
# enters the window before the goal does, so the old path at the entry already ends 0.79 m from the goal and the
# window-entry ratio is ~1.32 (the container's NavFn put it at 1.46-1.56, old path ~1 m short).
# A (2.2, -1.3) / B (2.1, -1.3): the container's signature in this loop -- old path ~1.0-1.1 m short of the goal,
# ratio 1.513 (A, just above 1.5, like the two loaded container runs) and 1.466 (B, just below, like CTRLPS3's).
OPS = {"room": "2.2 -1.4 0.0", "A": "2.2 -1.3 0.0", "B": "2.1 -1.3 0.0"}


def jobs(seeds, lib, ops=("room", "A", "B"), three_goals_room=True):
    out = []
    gaps = [1.0, 1.25, 1.5, 1.75, 2.0]
    for op in ops:
        goals = GOAL1 + ";" + OPS[op]
        if op == "room" and three_goals_room:
            goals += ";-1.5 -1.4 3.14"
        for arm in (0, 1):
            base = {"goals": goals}
            for ph in range(0, 50, 5):
                for gp in gaps:
                    out.append((op + ":phase", ph, arm, dict(base, step_phase_ms=ph, gap_s=gp), lib))
            for d in (0, 20, 50, 100):
                for s in range(seeds):
                    out.append((op + ":jitter", d, arm, dict(base, step_jitter_ms=d, seed=s), lib))
            for d in (0, 20, 50, 100):
                for gp in gaps:
                    out.append((op + ":odom", d, arm, dict(base, odom_delay_ms=d, gap_s=gp), lib))
            for p in (0.0, 0.03, 0.09):
                for s in range(seeds):
                    out.append((op + ":slip", p, arm, dict(base, slip=p, seed=s), lib))
    return out


def summarize(rows):
    keys = sorted({(r["group"], r["dose"], r["arm"]) for r in rows}, key=lambda k: (k[0], k[2], k[1]))
    summ = []
    for k in keys:
        rs = [r for r in rows if (r["group"], r["dose"], r["arm"]) == k]
        g2 = [next((g for g in r["goals"] if g["goal_index"] == 2), {}) for r in rs]
        ratios = [r["g2_transition"]["after"] / r["g2_transition"]["before"] for r in rs if r["g2_transition"]]
        fixed = [r["g2_transition"]["after"] / (r["g2_transition"]["before"] + r["g2_transition"]["old_end_to_goal"])
                 for r in rs if r["g2_transition"] and r["g2_transition"].get("old_end_to_goal") is not None]
        t2 = [g["elapsed_s"] for g in g2 if g.get("elapsed_s") is not None]
        summ.append({
            "group": k[0], "dose": k[1], "arm": k[2], "runs": len(rs),
            "g2_detour": sum(bool(g.get("detour")) for g in g2),
            "g2_time_s": {"min": min(t2), "median": statistics.median(t2), "max": max(t2)} if t2 else None,
            "g2_ratio_new_over_old": {"min": round(min(ratios), 3), "median": round(statistics.median(ratios), 3),
                                      "max": round(max(ratios), 3)} if ratios else None,
            "g2_ratio_fixed_rule": {"min": round(min(fixed), 3), "median": round(statistics.median(fixed), 3),
                                    "max": round(max(fixed), 3)} if fixed else None,
            "all_goals_succeeded": sum(all(g.get("status") == "SUCCEEDED" for g in r["goals"]) for r in rs),
            "goals_per_run": len(rs[0]["goals"]),
            "unsafe_ticks": sum(g.get("unsafe", 0) for r in rs for g in r["goals"]),
            "any_detour_any_goal": sum(any(g.get("detour") for g in r["goals"]) for r in rs)})
    return summ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--jobs", type=int, default=12)
    ap.add_argument("--lib", default=str(ROOT / "tools/kv_nav/build/host/libkv_nav.so"))
    ap.add_argument("--ops", default="room,A,B", help="goal-2 operating points (OPS keys)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    js = jobs(a.seeds, a.lib, tuple(a.ops.split(",")))
    rows = []
    with mp.Pool(a.jobs) as pool, (out / "runs.jsonl").open("w") as f:
        for r in pool.imap_unordered(one, js):
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
    summ = summarize(rows)
    (out / "summary.json").write_text(json.dumps(summ, indent=1))
    for s in summ:
        print(json.dumps(s))


if __name__ == "__main__":
    main()
