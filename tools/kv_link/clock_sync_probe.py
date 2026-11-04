#!/usr/bin/env python3
"""Clock-mapping probe against a running kv_server.py (B2, host/kv_clock_sync.py).

Runs --n four-timestamp exchanges at --hz on its own socket (the frame channel is
not touched), then prints / writes one JSON report:

  kv_map         offset (KV - Nano, ns) at the min-delay anchor, drift (ppm), bound
  pl_map         PL timebase count -> KV mono fit, or the explicit "degraded" reason
  estimate_jitter  the running offset estimate (evaluated at each exchange's own time,
                   after --settle exchanges) and its bound: spread = what a consumer
                   of the map would see move
  known_answer   only with --expect-offset-ns (and --expect-drift-ppm): every running
                 estimate's error vs the known truth and whether it stayed within its
                 bound. On the server loopback both ends read the same CLOCK_MONOTONIC_RAW,
                 so the truth is offset 0, drift 0.

Frame age is NOT computed from this (it stays a Nano-clock round trip).
Usage: clock_sync_probe.py --server HOST:PORT [--n 600] [--hz 10] [--window-s 60]
                           [--out report.json] [--expect-offset-ns 0 --expect-drift-ppm 0]
"""
import argparse
import csv
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.kv_clock_sync import ClockSync, ClockSyncClient, ClockSyncError  # noqa: E402
from host import fpga_clock  # noqa: E402

SERIES_COLUMNS = ["i", "t1", "t2", "t3", "t4", "theta_ns", "d_ns", "y_ns", "b_ns", "drift_ppm",
                  "consistent", "err"]


def stats(values):
    v = sorted(values)
    if not v:
        return None
    n = len(v)
    mean = sum(v) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in v) / (n - 1)) if n > 1 else 0.0
    return {"n": n, "min": v[0], "p50": v[n // 2], "p95": v[min(n - 1, int(0.95 * n))],
            "max": v[-1], "mean": mean, "sd": sd, "peak_to_peak": v[-1] - v[0]}


def probe(server, n, hz, window_s, settle, expect_offset_ns=None, expect_drift_ppm=None,
          timeout_s=0.5, series_csv=None):
    """series_csv: optional path; one row per exchange (t1..t4 on CLOCK_MONOTONIC_RAW, the
    exchange's theta/d, and the running estimate y +- b / drift / consistent queried at the
    exchange's own time, or the reason no estimate was given)."""
    client = ClockSyncClient(server, sync=ClockSync(window_s=window_s), timeout_s=timeout_s)
    est, bnd, err, within = [], [], [], []
    t_start = client.clock()       # same timebase as t1/t4 (CLOCK_MONOTONIC_RAW)
    fp = w = None
    if series_csv:
        fp = open(series_csv, "w", newline="")
        w = csv.writer(fp)
        w.writerow(SERIES_COLUMNS)
    try:
        for i in range(n):
            ack = client.exchange()
            if ack is None and w is not None:
                w.writerow([i] + [""] * 10 + ["no_ack"])
            if ack is not None and i + 1 < settle and w is not None:
                t1, t2, t3, t4 = ack["t1"], ack["t2"], ack["t3"], ack["t4"]
                w.writerow([i, t1, t2, t3, t4, ((t2 - t1) + (t3 - t4)) / 2.0,
                            (t4 - t1) - (t3 - t2), "", "", "", "", ""])
            if ack is not None and i + 1 >= settle:
                t = (ack["t1"] + ack["t4"]) // 2
                why = ""
                try:
                    m = client.sync.kv_map()
                    y, b = m.offset_at_nano(t)
                except ClockSyncError as e:
                    y = None
                    why = str(e)[:80]
                if w is not None:
                    t1, t2, t3, t4 = ack["t1"], ack["t2"], ack["t3"], ack["t4"]
                    row = [i, t1, t2, t3, t4, ((t2 - t1) + (t3 - t4)) / 2.0, (t4 - t1) - (t3 - t2)]
                    if y is None:
                        row += ["", "", "", "", why]
                    else:
                        ds = m.describe()
                        row += [y, b, ds["drift_ppm"], ds["consistent"], ""]
                    w.writerow(row)
                if y is not None:
                    est.append(y)
                    bnd.append(b)
                    if expect_offset_ns is not None:
                        truth = expect_offset_ns + (expect_drift_ppm or 0.0) * 1e-6 * (t - t_start)
                        e = y - truth
                        err.append(e)
                        within.append(abs(e) <= b)
            if fp is not None and i % 200 == 0:
                fp.flush()
            if i + 1 < n:
                time.sleep(1.0 / hz)
        rep = client.sync.report()
    finally:
        client.close()
        if fp is not None:
            fp.close()
    rep["estimate_jitter"] = {"offset_ns": stats(est), "bound_ns": stats(bnd), "settle": settle}
    if expect_offset_ns is not None:
        rep["known_answer"] = {"expect_offset_ns": expect_offset_ns,
                               "expect_drift_ppm": expect_drift_ppm or 0.0,
                               "error_ns": stats(err), "abs_error_ns": stats([abs(e) for e in err]),
                               "within_bound": sum(within), "checked": len(within),
                               "all_within_bound": bool(within) and all(within)}
    rep["probe"] = {"server": "%s:%d" % server, "n": n, "hz": hz, "window_s": window_s,
                    "python": sys.version.split()[0], "clock": fpga_clock.CLOCK_NAME,
                    "series_csv": series_csv}
    return rep


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", required=True, help="HOST:PORT of kv_server.py")
    ap.add_argument("--n", type=int, default=600)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--window-s", type=float, default=60.0)
    ap.add_argument("--settle", type=int, default=20)
    ap.add_argument("--timeout-s", type=float, default=0.5)
    ap.add_argument("--expect-offset-ns", type=float, default=None)
    ap.add_argument("--expect-drift-ppm", type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--series-csv", default=None,
                    help="per-exchange CSV (t1..t4, theta, d, running y/b/drift/consistent or error)")
    a = ap.parse_args()
    host, port = a.server.rsplit(":", 1)
    rep = probe((host, int(port)), a.n, a.hz, a.window_s, a.settle, a.expect_offset_ns,
                a.expect_drift_ppm, a.timeout_s, a.series_csv)
    text = json.dumps(rep, indent=1, sort_keys=True, default=str)
    if a.out:
        with open(a.out, "w") as fp:
            fp.write(text)
    print(text)
    ka = rep.get("known_answer")
    if ka is not None and not ka["all_within_bound"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
