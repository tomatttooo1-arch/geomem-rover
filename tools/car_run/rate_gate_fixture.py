"""Depth arrival sequences for the producer rate-gate tests, rebuilt from a bench run (2026-10-15).

Source: runs/car-bench-20261015-results/FUSED_DROP_DIG.md (daylight b4_1015_0336_ns: 929 fused / 120 s,
night b4_1014_1935_ns: 1165). The bench ran without --input-log, so only the RELEASED frames' (header
stamp, RAW arrival) are on file (frames.jsonl rows with depth_stamp_ns + acquired_ns, tf_missing rows
included). The rest is rebuilt, rule by rule:

  stamps   producer_summary input: 0 gaps, 0 non-monotonic, every camera frame reached the callback.
           Between two released stamps S_a < S_b, n = round((S_b - S_a) / P) camera steps (P = the
           run's own period from the summary); the n - 1 skipped frames get S_a + i (S_b - S_a) / n
           (linear: exact on the stamp grid up to the camera's sub-ms jitter).
  arrivals released: as recorded. skipped frame i: natural = A_a + (S_i - S_a) (the released frame's
           latency); if that falls in an EMPTY executor window -- a frames.jsonl slow_call spin_once
           row, its first 50 ms (the spin timeout: no message arrived; beyond 50 ms the call may end
           with a callback) -- the frame cannot have arrived then, and it was not after the window
           either in the paired pattern, so it is placed at the window start (it came with the frame
           before: "pairs"). Then the board's own decision bounds it: the old gate skipped it, so it
           arrived < A_a + 0.9 / rate_hz (code fact, host/ros_geo_producer.py before 2026-10-15);
           a later placement is clamped to 1 us under that bound. Then made non-decreasing.
           The empty windows alone place 672 of the 928 daylight third frames under 90 ms (the
           executor also wakes for camera_info / tf, so not every interval has a 50 ms window):
           replaying the old gate on the windows-only rebuild gives 1041, not 929 (--no-clamp). With
           the clamp the old-gate replay reproduces the board's releases exactly: a known-answer
           check of the fixture and of RateGate(mode="arrival"), not independent evidence of the
           pairs. The stamp gate's result does not depend on the arrival times at all.

Output: one gzip CSV per run (stamp_ns, arrival_raw_ns, released_on_board 0/1) and meta.json (window,
period, board count). Usage:
  python3 tools/car_run/rate_gate_fixture.py --run-dir RUN --window WINDOW.txt --name day --out DIR
"""
import argparse
import bisect
import gzip
import json
import os

SPIN_TIMEOUT_NS = 50000000


def load(run_dir):
    puts, empty = [], []
    with open(os.path.join(run_dir, "frames.jsonl")) as fp:
        for line in fp:
            r = json.loads(line)
            if r.get("event") == "slow_call" and r.get("name") == "spin_once":
                st = int(r["start_raw_ns"])
                empty.append((st, min(int(r["end_raw_ns"]), st + SPIN_TIMEOUT_NS)))
            elif "depth_stamp_ns" in r and "acquired_ns" in r and r.get("event") in (None, "tf_missing"):
                puts.append((int(r["depth_stamp_ns"]), int(r["acquired_ns"])))
    with open(os.path.join(run_dir, "producer_summary.json")) as fp:
        inp = json.load(fp)["input"]
    puts.sort()
    empty.sort()
    return puts, empty, inp


def rebuild(puts, empty, period_ns, bound_ns=None):
    starts = [e[0] for e in empty]
    rows = []
    for (sa, aa), (sb, ab) in zip(puts, puts[1:]):
        rows.append((sa, aa, 1))
        n = max(1, int(round((sb - sa) / float(period_ns))))
        for i in range(1, n):
            s = sa + int(round(i * (sb - sa) / float(n)))
            t = aa + (s - sa)
            j = bisect.bisect_right(starts, t) - 1
            if j >= 0 and empty[j][0] <= t < empty[j][1]:
                t = empty[j][0]
            if bound_ns is not None:
                t = min(t, aa + bound_ns - 1000)
            rows.append((s, min(max(t, rows[-1][1]), ab), 0))
    rows.append((puts[-1][0], puts[-1][1], 1))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", required=True)
    ap.add_argument("--window", required=True, help="the run's <tag>.window.txt (RAW start/end)")
    ap.add_argument("--name", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rate-hz", type=float, default=10.0, help="the run's producer --rate-hz (old gate bound)")
    ap.add_argument("--no-clamp", action="store_true", help="empty windows only (no old-gate bound)")
    a = ap.parse_args(argv)
    w = dict(l.strip().split("=", 1) for l in open(a.window) if "=" in l)
    w0, w1 = int(w["window_start_raw_ns"]), int(w["window_end_raw_ns"])
    puts, empty, inp = load(a.run_dir)
    if inp.get("gaps") or inp.get("non_monotonic"):
        raise SystemExit("input gaps / non-monotonic stamps on file: the grid rebuild does not hold")
    period_ns = float(inp["period_ms"]) * 1e6
    bound_ns = None if a.no_clamp else int(0.9 * 1e9 / a.rate_hz)
    rows = rebuild(puts, empty, period_ns, bound_ns)
    os.makedirs(a.out, exist_ok=True)
    with gzip.open(os.path.join(a.out, a.name + ".csv.gz"), "wt") as fp:
        fp.write("stamp_ns,arrival_raw_ns,released_on_board\n")
        for s, t, r in rows:
            fp.write("%d,%d,%d\n" % (s, t, r))
    meta_path = os.path.join(a.out, "meta.json")
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    meta[a.name] = {"run": os.path.basename(os.path.normpath(a.run_dir)), "window_start_raw_ns": w0,
                    "window_end_raw_ns": w1, "period_ms": inp["period_ms"],
                    "board_released_in_window": sum(1 for _s, t in puts if w0 <= t <= w1),
                    "arrivals_rebuilt": len(rows), "arrivals_on_board": inp["arrivals"],
                    "empty_windows": len(empty), "old_gate_bound_ns": bound_ns, "source": "tools/car_run/rate_gate_fixture.py"}
    with open(meta_path, "w") as fp:
        json.dump(meta, fp, indent=1, sort_keys=True)
    print(json.dumps(meta[a.name]))


if __name__ == "__main__":
    main()
