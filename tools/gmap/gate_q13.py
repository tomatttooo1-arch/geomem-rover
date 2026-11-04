#!/usr/bin/env python3
"""GMAP1 adoption gate (frozen in runs/nationals-prelim-20261102/GMAP0/README.md section 0 before the first run):
the same GridFrame sequence through kv_mapper rule "float" (NN-MAP1) and rule "q13" (Q2.13 int16); final tri-states
compared; pass when differing cells <= 0.1 % of the known cells, on every trip given.

    python3 tools/gmap/gate_q13.py --frames A.npz [--frames B.npz ...] --out gate.json
"""
import argparse
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host import kv_mapper as km  # noqa: E402


def frames_of(path):
    z = np.load(path)
    for i in range(len(z["stamp"])):
        yield km.GridFrame(float(z["stamp"][i]), float(z["origin"][i, 0]), float(z["origin"][i, 1]),
                           np.ascontiguousarray(z["data"][i]), tuple(float(v) for v in z["pose"][i]))


def run(path, rule):
    m = km.KvMapper(rule=rule)
    t = []
    for fr in frames_of(path):
        t0 = time.perf_counter()
        m.integrate(fr)
        t.append((time.perf_counter() - t0) * 1e3)
    return m, t


def gate_one(path, frac=1e-3):
    mf, tf = run(path, "float")
    mq, tq = run(path, "q13")
    a, b = mf.tri(), mq.tri()
    known_f, known_q = int((a != km.UNK).sum()), int((b != km.UNK).sum())
    diff = a != b
    rows, cols = np.nonzero(diff)
    cells = []
    for r, c in zip(rows.tolist(), cols.tolist()):
        cells.append({"row": r, "col": c, "float": int(a[r, c]), "q13": int(b[r, c]),
                      "float_logodds": float(mf.logodds[r, c]), "q13_logodds": int(mq.logodds[r, c]),
                      "q13_logodds_as_float": int(mq.logodds[r, c]) / km.Q13})
    nd = int(diff.sum())
    # where the two rules sit relative to each other over all known cells (diagnosis only)
    seen = mf.seen
    err = np.abs(mq.logodds[seen].astype(np.float64) / km.Q13 - mf.logodds[seen].astype(np.float64))
    return {"frames": path, "n_frames": mf.frames, "known_float": known_f, "known_q13": known_q,
            "seen_equal": bool(np.array_equal(mf.seen, mq.seen)),
            "occupied_float": int((a == km.OCC).sum()), "occupied_q13": int((b == km.OCC).sum()),
            "dropped_float": mf.dropped_cells, "dropped_q13": mq.dropped_cells,
            "diff_cells": nd, "limit_cells": known_f * frac, "pass": nd <= known_f * frac,
            "diff_fraction_of_known": nd / max(1, known_f), "diff_list": cells,
            "abs_logodds_err_max": float(err.max()) if err.size else 0.0,
            "abs_logodds_err_p99": float(np.percentile(err, 99)) if err.size else 0.0,
            "integrate_ms_p50": {"float": float(np.median(tf)), "q13": float(np.median(tq))},
            "status_float": mf.status(), "status_q13": mq.status()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", action="append", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    res = {"rule": "diff cells <= 0.1% of known cells on every trip", "trips": [gate_one(p) for p in a.frames]}
    res["pass"] = all(t["pass"] for t in res["trips"])
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1, default=str)
    for t in res["trips"]:
        print(json.dumps({k: t[k] for k in ("frames", "n_frames", "known_float", "seen_equal", "occupied_float",
                                            "occupied_q13", "diff_cells", "limit_cells", "pass",
                                            "abs_logodds_err_max", "integrate_ms_p50")}))
    print("GATE", "PASS" if res["pass"] else "FAIL")


if __name__ == "__main__":
    main()
