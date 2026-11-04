#!/usr/bin/env python3
"""A2 workload statistics: per-ray step distribution, per-frame totals, longest
ray and tail work, and the raster-order demand profile (rec30 / TUM).

Steps S = crossings of the reference walk (same definition as A0's
tools/m7_clip/ray_step_stats.cpp); a lane spends max(S,1) cycles on a ray
(A1 walk: load merged with the first step). The per-frame totals are checked
against A0's runs/m7-clip-ref-20261011/steps_<ds>.frames.csv (instrument check).

Usage: workload.py --build FRESH_DIR --out-dir DIR [--datasets rec30,tum] [--jobs 32]
Writes DIR/workload_<ds>.json, DIR/workload_<ds>.frames.csv, DIR/workload_<ds>.rows.csv
"""
import argparse
import csv
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.m7_lanes import lanes as L  # noqa: E402

A0_DIR = os.path.join(ROOT, "runs", "m7-clip-ref-20261011")
_G = {}


def _init(so, dsn):
    _G["lm"] = L.Lanes(so)
    _G["ds"] = L.open_dataset(dsn)


def _frame(f):
    ds, lm = _G["ds"], _G["lm"]
    W, H = ds["W"], ds["H"]
    z = L.depth_frame(ds, f)
    with lm.prepare(z, ds["ik"], L.WC, **L.GRID) as p:
        steps, cls = p.profile()
        mism = p.step_core_mismatch
        counts = p.counts
    walked = cls > 0
    s = steps[walked].astype(np.int64)
    cost = np.maximum(s, 1)
    hist_acc = np.bincount(steps[cls == 1], minlength=97)[:97]
    hist_clp = np.bincount(steps[cls == 2], minlength=97)[:97]
    cost_px = np.where(walked, np.maximum(steps.astype(np.int64), 1), 0).reshape(H, W)
    row_work = cost_px.sum(axis=1)                      # cycles of lane work per image row
    row_rays = walked.reshape(H, W).sum(axis=1)
    # Tail work: lane work of the rays in the last k rows (dispatched last in raster order).
    tail = {"last_row": int(row_work[-1:].sum()), "last_8_rows": int(row_work[-8:].sum())}
    # Local demand: lane work per converter pixel over sliding windows of k rows.
    dens = {}
    for k in (1, 8, 32):
        c = np.convolve(row_work, np.ones(k, dtype=np.int64), mode="valid")
        dens["max_work_per_px_%drows" % k] = round(float(c.max()) / (k * W), 3)
    return dict(frame=f, pixels=W * H, accepted=counts[0], invalid=counts[1], clipped=counts[2],
                rays=int(walked.sum()), steps=int(s.sum()), work=int(cost.sum()),
                zero_step_rays=int((s == 0).sum()), max_steps=int(s.max()) if s.size else 0,
                p50=int(np.percentile(s, 50)) if s.size else 0, p99=int(np.percentile(s, 99)) if s.size else 0,
                steps_acc=int(steps[cls == 1].sum()), steps_clp=int(steps[cls == 2].sum()),
                max_acc=int(steps[cls == 1].max()) if (cls == 1).any() else 0,
                max_clp=int(steps[cls == 2].max()) if (cls == 2).any() else 0,
                step_core_mismatch=mism, tail=tail, dens=dens,
                hist_acc=hist_acc.tolist(), hist_clp=hist_clp.tolist(), row_work=row_work.tolist(),
                row_rays=row_rays.tolist())


def pctl_from_hist(h, q):
    c = np.cumsum(h)
    return int(np.searchsorted(c, q * c[-1], side="left"))


def summarize(vals):
    a = np.asarray(vals, dtype=np.float64)
    return {"mean": round(float(a.mean()), 1), "min": float(a.min()), "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)), "p99": float(np.percentile(a, 99)), "max": float(a.max())}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--datasets", default="rec30,tum")
    ap.add_argument("--jobs", type=int, default=32)
    a = ap.parse_args()
    so = L.build(a.build)
    os.makedirs(a.out_dir, exist_ok=True)
    for dsn in a.datasets.split(","):
        ds = L.open_dataset(dsn)
        with Pool(a.jobs, initializer=_init, initargs=(so, dsn)) as pool:
            rows = pool.map(_frame, range(ds["N"]), chunksize=4)
        rows.sort(key=lambda r: r["frame"])
        # instrument check against A0's step statistics
        a0 = {int(r["frame"]): r for r in csv.DictReader(open(os.path.join(A0_DIR, "steps_%s.frames.csv" % dsn)))}
        a0_bad = 0
        for r in rows:
            q = a0[r["frame"]]
            want = (int(q["accepted"]), int(q["invalid"]), int(q["clipped"]), int(q["steps_accepted"]),
                    int(q["steps_clipped"]), int(q["max_steps_accepted"]), int(q["max_steps_clipped"]))
            got = (r["accepted"], r["invalid"], r["clipped"], r["steps_acc"], r["steps_clp"], r["max_acc"], r["max_clp"])
            a0_bad += want != got
        hist_acc = np.sum([r["hist_acc"] for r in rows], axis=0)
        hist_clp = np.sum([r["hist_clp"] for r in rows], axis=0)
        hist_all = hist_acc + hist_clp
        s_idx = np.arange(97)
        row_work = np.asarray([r["row_work"] for r in rows], dtype=np.float64)
        row_rays = np.asarray([r["row_rays"] for r in rows], dtype=np.float64)
        W = ds["W"]
        rep = {
            "dataset": dsn, "prep": ds["prep"], "depth_sha256_meta": ds["meta"].get("depth_sha256"),
            "frames": ds["N"], "width": W, "height": ds["H"], "unit_m": ds["unit"],
            "source": "tools/m7_lanes/workload.py (C model walk; S = crossings as A0)",
            "instrument_check_vs_A0_steps_csv": {"frames": len(rows), "mismatching_frames": a0_bad},
            "pstep_vs_reference_step_count_mismatch_rays": int(sum(r["step_core_mismatch"] for r in rows)),
            "per_ray_steps_all_frames": {
                "rays": int(hist_all.sum()), "mean": round(float((hist_all * s_idx).sum() / hist_all.sum()), 3),
                "p50": pctl_from_hist(hist_all, .5), "p90": pctl_from_hist(hist_all, .9),
                "p99": pctl_from_hist(hist_all, .99), "p999": pctl_from_hist(hist_all, .999),
                "max": int(s_idx[hist_all > 0].max()), "zero_step_rays": int(hist_all[0])},
            "per_frame": {k: summarize([r[k] for r in rows]) for k in
                          ("pixels", "rays", "steps", "work", "zero_step_rays", "max_steps", "p99")},
            "steps_per_pixel": summarize([r["steps"] / r["pixels"] for r in rows]),
            "tail_work": {k: summarize([r["tail"][k] for r in rows]) for k in rows[0]["tail"]},
            "local_demand_work_per_pixel": {k: summarize([r["dens"][k] for r in rows]) for k in rows[0]["dens"]},
            "worst_frame_by_work": max(rows, key=lambda r: r["work"])["frame"],
            "row_profile_mean": {"work_per_pixel_by_row_decile": [
                round(float(row_work[:, i0:i1].mean() / W), 2)
                for i0, i1 in [(ds["H"] * d // 10, ds["H"] * (d + 1) // 10) for d in range(10)]]},
        }
        json.dump(rep, open(os.path.join(a.out_dir, "workload_%s.json" % dsn), "w"), indent=1)
        with open(os.path.join(a.out_dir, "workload_%s.frames.csv" % dsn), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["frame", "pixels", "rays", "accepted", "clipped", "invalid", "steps", "work",
                        "zero_step_rays", "max_steps", "p50", "p99", "tail_last_row", "tail_last_8_rows",
                        "max_work_per_px_1row", "max_work_per_px_8rows", "max_work_per_px_32rows"])
            for r in rows:
                w.writerow([r["frame"], r["pixels"], r["rays"], r["accepted"], r["clipped"], r["invalid"], r["steps"],
                            r["work"], r["zero_step_rays"], r["max_steps"], r["p50"], r["p99"], r["tail"]["last_row"],
                            r["tail"]["last_8_rows"], r["dens"]["max_work_per_px_1rows"],
                            r["dens"]["max_work_per_px_8rows"], r["dens"]["max_work_per_px_32rows"]])
        with open(os.path.join(a.out_dir, "workload_%s.rows.csv" % dsn), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["row", "mean_rays", "mean_work", "mean_work_per_pixel", "max_work_per_pixel"])
            for i in range(ds["H"]):
                w.writerow([i, round(float(row_rays[:, i].mean()), 1), round(float(row_work[:, i].mean()), 1),
                            round(float(row_work[:, i].mean() / W), 3), round(float(row_work[:, i].max() / W), 3)])
        with open(os.path.join(a.out_dir, "workload_%s.hist.csv" % dsn), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["steps", "accepted_rays", "clipped_rays"])
            for i in range(97):
                w.writerow([i, int(hist_acc[i]), int(hist_clp[i])])
        print(json.dumps({k: rep[k] for k in ("dataset", "instrument_check_vs_A0_steps_csv",
                                                "pstep_vs_reference_step_count_mismatch_rays",
                                                "per_ray_steps_all_frames")}, indent=None))
        print(" per_frame work", rep["per_frame"]["work"], " max_steps", rep["per_frame"]["max_steps"])
        print(" local demand", rep["local_demand_work_per_pixel"])
        print(" rows", rep["row_profile_mean"])


if __name__ == "__main__":
    main()
