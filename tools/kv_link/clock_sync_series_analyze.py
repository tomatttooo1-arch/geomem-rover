#!/usr/bin/env python3
"""Offline check of a clock_sync_probe --series-csv (B2 RAW retest, 2026-10-11).

Board runs have no known answer for KV - Nano, so each running estimate y +- b (causal: it
only saw exchanges up to its own time) is compared with an offline, NON-causal reference:
for query time t, take the exchanges within +- --ref-half-s, keep the min-delay exchange of
each of --ref-bins equal time bins (the lower envelope, same idea as the estimator), and
least-squares a line theta(t) through them; ref = line(t). The reference sees both sides
of t, so a rate change at t costs it at most ~|delta rate| * half / 4 (75 us for 120 ppm and
2.5 s), far less than a causal 60 s window pays.

Reported:
  overrun          |y - ref| > b                  (estimate outside its own stated bound)
  overrun_margin   |y - ref| > b + ref_unc        (ref_unc = max |residual| of the envelope fit:
                                                   counts only overruns the reference cannot explain)
  rejected         rows with no estimate (err text), grouped into contiguous runs
  necessary check  [y-b, y+b] meets theta_i +- d_i/2 of the same exchange (as in section 8)
  detrended jitter sd per 60 s block, bound median, running drift
--events: optional "label=unix_or_raw_ns" list; per event the overruns/rejects within +-60 s.
Usage: clock_sync_series_analyze.py series.csv [--ref-half-s 2.5] [--ref-bins 10]
"""
import argparse
import bisect
import csv
import json
import statistics as st


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else None


def load(path):
    rows = []
    for r in csv.DictReader(open(path)):
        if not r["t1"]:
            continue
        rows.append({"i": int(r["i"]), "t": (int(r["t1"]) + int(r["t4"])) // 2,
                     "theta": float(r["theta_ns"]), "d": float(r["d_ns"]),
                     "y": float(r["y_ns"]) if r["y_ns"] else None,
                     "b": float(r["b_ns"]) if r["b_ns"] else None,
                     "drift": float(r["drift_ppm"]) if r["drift_ppm"] else None,
                     "cons": r["consistent"], "err": r["err"]})
    return rows


def reference(rows, ts, k, half_ns, bins):
    lo = bisect.bisect_left(ts, ts[k] - half_ns)
    hi = bisect.bisect_right(ts, ts[k] + half_ns)
    width = 2 * half_ns
    best = {}
    for j in range(lo, hi):
        bi = min(bins - 1, (ts[j] - (ts[k] - half_ns)) * bins // width)
        if bi not in best or rows[j]["d"] < rows[best[bi]]["d"]:
            best[bi] = j
    pts = [(float(ts[j] - ts[k]), rows[j]["theta"]) for j in best.values()]
    if len(pts) < 3:
        return None, None
    mx = sum(p[0] for p in pts) / len(pts)
    my = sum(p[1] for p in pts) / len(pts)
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    s = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx if sxx > 0 else 0.0
    c = my - s * mx
    unc = max(abs(p[1] - (c + s * p[0])) for p in pts)
    return c, unc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--ref-half-s", type=float, default=2.5)
    ap.add_argument("--ref-bins", type=int, default=10)
    ap.add_argument("--events", nargs="*", default=[],
                    help="label=raw_ns (same clock as the CSV t1..t4) to count overruns/rejects around")
    a = ap.parse_args()
    rows = load(a.csv)
    ts = [r["t"] for r in rows]
    half_ns = int(a.ref_half_s * 1e9)
    est_k = [k for k, r in enumerate(rows) if r["y"] is not None]
    rej_k = [k for k, r in enumerate(rows) if r["y"] is None and r["err"]]
    over, over_m, ratio, errs, uncs = [], [], [], [], []
    necessary_ok = 0
    for k in est_k:
        r = rows[k]
        ref, unc = reference(rows, ts, k, half_ns, a.ref_bins)
        if abs(r["y"] - r["theta"]) <= r["b"] + r["d"] / 2:
            necessary_ok += 1
        if ref is None:
            continue
        e = r["y"] - ref
        errs.append(e)
        uncs.append(unc)
        ratio.append(abs(e) / r["b"])
        if abs(e) > r["b"]:
            over.append((r["i"], r["t"], e, r["b"]))
        if abs(e) > r["b"] + unc:
            over_m.append((r["i"], r["t"], e, r["b"], unc))
    # contiguous reject runs
    runs = []
    for k in rej_k:
        if runs and rows[k]["i"] <= runs[-1][1] + 2:
            runs[-1][1] = rows[k]["i"]
            runs[-1][2] += 1
        else:
            runs.append([rows[k]["i"], rows[k]["i"], 1, rows[k]["t"], rows[k]["err"]])
    # detrended jitter, 60 s blocks
    est = [rows[k] for k in est_k]
    t0 = est[0]["t"] if est else 0
    blk = {}
    for r in est:
        blk.setdefault(int((r["t"] - t0) // 60_000_000_000), []).append(((r["t"] - t0) / 1e9, r["y"]))
    bres = []
    for v in blk.values():
        if len(v) < 10:
            continue
        mx = st.mean(p[0] for p in v)
        my = st.mean(p[1] for p in v)
        s2 = sum((p[0] - mx) * (p[1] - my) for p in v) / sum((p[0] - mx) ** 2 for p in v)
        bres += [p[1] - (my + s2 * (p[0] - mx)) for p in v]
    b = [r["b"] for r in est]
    dr = [r["drift"] for r in est]
    n6 = max(1, len(est) // 6)
    d = [r["d"] for r in rows]
    out = {
        "csv": a.csv, "exchanges_with_ack": len(rows), "estimates": len(est), "rejected": len(rej_k),
        "reject_runs": [{"i_first": x[0], "i_last": x[1], "n": x[2], "t_ns": x[3], "err": x[4]} for x in runs],
        "reference": {"half_s": a.ref_half_s, "bins": a.ref_bins, "checked": len(errs),
                      "unc_us": {"median": (q(uncs, .5) or 0) / 1e3, "p95": (q(uncs, .95) or 0) / 1e3,
                                 "max": max(uncs) / 1e3 if uncs else None}},
        "abs_err_vs_ref_us": {"median": q([abs(e) for e in errs], .5) / 1e3,
                              "p95": q([abs(e) for e in errs], .95) / 1e3,
                              "max": max(abs(e) for e in errs) / 1e3},
        "err_over_bound": {"median": q(ratio, .5), "p95": q(ratio, .95), "max": max(ratio)},
        "overrun": len(over),
        "overrun_beyond_ref_unc": len(over_m),
        "overrun_first": [{"i": o[0], "t_ns": o[1], "err_us": o[2] / 1e3, "b_us": o[3] / 1e3} for o in over[:12]],
        "overrun_max_excess_us": max((abs(o[2]) - o[3]) / 1e3 for o in over) if over else 0.0,
        "necessary_check": "%d/%d" % (necessary_ok, len(est)),
        "d_us": {"median": q(d, .5) / 1e3, "p95": q(d, .95) / 1e3, "max": max(d) / 1e3},
        "jitter_detrended_60s_us": {"sd": st.stdev(bres) / 1e3, "abs_p95": q([abs(v) for v in bres], .95) / 1e3},
        "bound_us": {"median": st.median(b) / 1e3, "p95": q(b, .95) / 1e3, "max": max(b) / 1e3,
                     "first_sixth_median": st.median(b[:n6]) / 1e3, "last_sixth_median": st.median(b[-n6:]) / 1e3},
        "running_drift_ppm": {"median": st.median(dr), "min": min(dr), "max": max(dr),
                              "first_sixth_median": st.median(dr[:n6]), "last_sixth_median": st.median(dr[-n6:])},
        "consistent_false": sum(1 for r in est if r["cons"] == "False"),
    }
    ev = {}
    for item in a.events:
        lab, val = item.split("=", 1)
        te = int(float(val))
        w = 60_000_000_000
        ev[lab] = {"t_ns": te,
                   "overrun_within_60s": sum(1 for o in over if abs(o[1] - te) <= w),
                   "rejected_within_60s": sum(1 for k in rej_k if abs(rows[k]["t"] - te) <= w),
                   "estimates_within_60s": sum(1 for r in est if abs(r["t"] - te) <= w),
                   "drift_ppm_before_after": [
                       next((r["drift"] for r in reversed(est) if r["t"] < te), None),
                       next((r["drift"] for r in est if r["t"] > te + w), None)]}
    out["events"] = ev
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
