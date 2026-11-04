#!/usr/bin/env python3
"""Real-data regression (new vs old reference) and DDA step statistics.

On a prepared recording (tools/geo_same_input/prep_rec.py layout: depth_u16.bin
+ meta.json), product grid 32^3 x 5 cm at (-0.8,-0.8,-0.05), identity pose:
  1. new and old hw/m7/frame_depth.cpp, frame by frame: flags and counts;
     the verdict "identical" is only reported together with the number of
     clipped rays (identical is expected only where nothing is clipped);
  2. optional --inside-only: also runs a variant of the recording in which
     every pixel whose endpoint is outside the grid is set to "no return";
     new and old must then be identical on every frame (in-grid bit-identity);
  3. optional --board-csv: the per-frame counts/free/hit recorded on the board
     (10-10 KV260 run) must equal the new reference;
  4. tools/m7_clip/ray_step_stats.cpp: step statistics; its own flags/counts
     dump must equal the new reference on every frame (instrument check).
Usage: regress_and_stats.py --prep DIR --build FRESH --out report.json
         [--unit-m U] [--board-csv frames.csv] [--inside-only] [--label NAME]
"""
import argparse
import csv
import json
import os
import subprocess
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.geo_same_input.common import read_dump  # noqa: E402
from tools.m7_clip.golden import project_frame  # noqa: E402
from tools.m7_clip.kernels import Kernel, build_all  # noqa: E402

GRID = dict(grid_origin=(-0.8, -0.8, -0.05), voxel=0.05, dims=(32, 32, 32))
WC = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]], dtype=np.float32)


def pct(hist, q):
    c = np.cumsum(hist)
    if c[-1] == 0:
        return None
    return int(np.searchsorted(c, q * c[-1], side="left"))


def hist_summary(hist):
    hist = np.asarray(hist, dtype=np.int64)
    n = int(hist.sum())
    if n == 0:
        return {"rays": 0}
    s = np.arange(hist.size)
    return {"rays": n, "mean": round(float((hist * s).sum() / n), 3), "median": pct(hist, 0.5),
            "p90": pct(hist, 0.9), "p99": pct(hist, 0.99), "max": int(s[hist > 0].max()),
            "rays_at_step_cap_96": int(hist[96]) if hist.size > 96 else 0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prep", required=True)
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--unit-m", type=float)
    ap.add_argument("--board-csv")
    ap.add_argument("--inside-only", action="store_true")
    ap.add_argument("--label", default="")
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.prep, "meta.json")))
    W, H, N = meta["width"], meta["height"], meta["frames"]
    unit = a.unit_m if a.unit_m else float(meta.get("depth_unit_m", 0.001))
    ik = np.array([float.fromhex(h) for h in meta["inverse_k_f32_hex"]], dtype=np.float32).reshape(3, 3)
    raw = np.memmap(os.path.join(a.prep, meta["depth_file"]), dtype="<u2", mode="r", shape=(N, H, W))
    os.makedirs(a.build, exist_ok=False)
    libs = build_all(os.path.join(a.build, "libs"))
    new, old = Kernel(libs["new"]), Kernel(libs["old"])
    rep = {"label": a.label, "prep": a.prep, "frames": N, "width": W, "height": H, "unit_m": unit,
           "depth_sha256_meta": meta.get("depth_sha256")}
    new_counts = np.zeros((N, 3), dtype=np.int64)
    new_fh = np.zeros((N, 2), dtype=np.int64)
    new_flags = np.zeros((N, 32 ** 3), dtype=np.uint8)
    ident_frames = flag_diff_frames = count_diff_frames = 0
    clipped_on_identical = 0
    inside_only = {"frames": 0, "identical": 0, "removed_pixels": 0} if a.inside_only else None
    for f in range(N):
        z = raw[f].astype(np.float32) * np.float32(unit)
        z[raw[f] == 0] = np.nan
        nf, nc = new.run(z, ik, WC, **GRID)
        of, oc = old.run(z, ik, WC, **GRID)
        new_counts[f] = nc
        new_flags[f] = nf
        new_fh[f] = ((nf == 1).sum(), (nf == 2).sum())
        same_f, same_c = np.array_equal(nf, of), nc == oc
        if same_f and same_c:
            ident_frames += 1
            clipped_on_identical += nc[2]
        flag_diff_frames += not same_f
        count_diff_frames += not same_c
        if a.inside_only:
            _, _, cls = project_frame(z, ik, WC, **GRID)
            z2 = z.copy().ravel()
            z2[cls == 2] = np.nan
            z2 = z2.reshape(z.shape)
            nf2, nc2 = new.run(z2, ik, WC, **GRID)
            of2, oc2 = old.run(z2, ik, WC, **GRID)
            inside_only["frames"] += 1
            inside_only["identical"] += int(np.array_equal(nf2, of2) and nc2 == oc2 and nc2[2] == 0)
            inside_only["removed_pixels"] += int((cls == 2).sum())
    rep["new_vs_old"] = {"identical_frames": ident_frames, "frames_with_flag_diff": flag_diff_frames,
                         "frames_with_count_diff": count_diff_frames,
                         "clipped_rays_total": int(new_counts[:, 2].sum()),
                         "clipped_rays_on_identical_frames": int(clipped_on_identical),
                         "counts_conserved_frames": int((new_counts.sum(axis=1) == W * H).sum())}
    if inside_only:
        rep["inside_only_variant_new_vs_old"] = inside_only
    tot = new_counts.sum(axis=0)
    px = N * W * H
    rep["class_fractions"] = {"accepted": round(tot[0] / px, 4), "invalid": round(tot[1] / px, 4),
                              "clipped": round(tot[2] / px, 4)}
    rep["per_frame_counts_mean"] = {"accepted": round(float(new_counts[:, 0].mean()), 1),
                                    "invalid": round(float(new_counts[:, 1].mean()), 1),
                                    "clipped": round(float(new_counts[:, 2].mean()), 1),
                                    "free_voxels": round(float(new_fh[:, 0].mean()), 1),
                                    "hit_voxels": round(float(new_fh[:, 1].mean()), 1)}
    if a.board_csv:
        rows = [r for r in csv.DictReader(open(a.board_csv)) if r["pass"] == "0"]
        rows.sort(key=lambda r: int(r["frame"]))
        bad = 0
        for r in rows:
            f = int(r["frame"])
            got = (int(new_counts[f, 0]), int(new_counts[f, 1]), int(new_counts[f, 2]), int(new_fh[f, 0]), int(new_fh[f, 1]))
            want = (int(r["accepted"]), int(r["invalid"]), int(r["outside"]), int(r["free_voxels"]), int(r["hit_voxels"]))
            bad += got != want
        rep["board_csv_vs_new"] = {"csv": a.board_csv, "frames": len(rows), "mismatching_frames": bad}
    # step statistics
    exe = os.path.join(a.build, "ray_step_stats")
    subprocess.run(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
                    os.path.join(ROOT, "tools", "m7_clip", "ray_step_stats.cpp"), "-o", exe], check=True)
    prefix = os.path.join(a.build, "steps")
    subprocess.run([exe, "--depth", os.path.join(a.prep, meta["depth_file"]), "--width", str(W), "--height", str(H),
                    "--frames", str(N), "--unit-m", repr(unit), "--ik", ",".join(meta["inverse_k_f32_hex"]),
                    "--out-prefix", prefix], check=True)
    sc, sf = read_dump(prefix + ".dump", 32 ** 3)
    rep["step_tool_matches_new_reference"] = {
        "frames_flag_equal": int(np.all(sf == new_flags, axis=1).sum()),
        "frames_count_equal": int(np.all(sc.astype(np.int64) == new_counts, axis=1).sum())}
    fr = list(csv.DictReader(open(prefix + ".frames.csv")))
    hist = np.loadtxt(prefix + ".hist.csv", delimiter=",", skiprows=1, dtype=np.int64)
    sa = np.array([int(r["steps_accepted"]) for r in fr])
    scl = np.array([int(r["steps_clipped"]) for r in fr])
    acc = np.array([int(r["accepted"]) for r in fr])
    ua = np.array([int(r["unique_end_accepted"]) for r in fr])
    uc = np.array([int(r["unique_end_clipped"]) for r in fr])
    uw = np.array([int(r["unique_end_walked"]) for r in fr])
    rep["steps"] = {
        "per_ray_accepted": hist_summary(hist[:, 1]),
        "per_ray_clipped": hist_summary(hist[:, 2]),
        "per_ray_walked_all": hist_summary(hist[:, 1] + hist[:, 2]),
        "per_frame_total_accepted": {"mean": round(float(sa.mean()), 1), "min": int(sa.min()), "max": int(sa.max())},
        "per_frame_total_clipped": {"mean": round(float(scl.mean()), 1), "min": int(scl.min()), "max": int(scl.max())},
        "per_frame_total_walked": {"mean": round(float((sa + scl).mean()), 1), "min": int((sa + scl).min()),
                                   "max": int((sa + scl).max())},
        "per_frame_accepted_rays_mean": round(float(acc.mean()), 1),
        "per_frame_unique_q8_end_accepted": {"mean": round(float(ua.mean()), 1), "min": int(ua.min()), "max": int(ua.max())},
        "per_frame_unique_q8_end_clipped": {"mean": round(float(uc.mean()), 1), "min": int(uc.min()), "max": int(uc.max())},
        "per_frame_unique_q8_end_walked": {"mean": round(float(uw.mean()), 1), "min": int(uw.min()), "max": int(uw.max())},
        "files": {"frames_csv": prefix + ".frames.csv", "hist_csv": prefix + ".hist.csv", "dump": prefix + ".dump"}}
    json.dump(rep, open(a.out, "w"), indent=1)
    print(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
