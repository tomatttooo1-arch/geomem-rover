#!/usr/bin/env python3
"""Camera tilt 10 deg, the far-range cap of the ground calibration: per distance ring, how real Gemini 2 floor
depth behaves at the old 27 deg mount (ng3_1017 & co., 10-16/10-17) and at the new 7 deg one (c10_1, 10-17 14:31).
Analysis only (reads the npz, writes OUT.json; nothing is written next to the inputs).
For each capture: the check-mode fit of its own configuration gives the floor plane and the true mount; per ring of
true horizontal distance: predicted samples (configured mount), floor samples (|h| < 5 mm), their local-normal
deviation from the plane (median, p90, share > LOCAL_NORMAL_MAX_DEG), and the inliers of the kernel.
docs/design/CAM_TILT_10_20261017.md section 2.4.

  real_rings.py OUT.json NAME=NPZ:H:P [...]
Python 3.8 compatible."""
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.car_run import ground_calib as gc  # noqa: E402
from tools.car_run import ground_plane_kernel as kernel  # noqa: E402

RINGS = ((0.3, 0.6), (0.6, 0.8), (0.8, 1.0), (1.0, 1.2), (1.2, 1.4), (1.4, 1.6), (1.6, 1.8), (1.8, 2.0))


def one(npz, h, p):
    z = np.load(npz)
    frames, K = list(z["frames"]), np.asarray(z["K"], dtype=np.float64)
    hi, wi = frames[0].shape
    u, v = gc.sample_pixels(wi, hi)
    iv, iu = v.astype(int), u.astype(int)
    shape = gc.grid_shape(wi, hi)
    med = gc.median_depth([gc.depth_mm_to_m(np.asarray(f)[iv, iu]) for f in frames])
    r = gc.calibrate(frames, K, h, p, mode="check")
    full = r["quality"]["full"]
    n, d = np.asarray(full["normal"]), full["d"]
    fit = (full["height_m"], full["pitch_deg"], full["roll_deg"])
    valid = np.isfinite(med) & (med > 0)
    pts = gc.rays(K, u, v) * np.where(valid, med, np.nan)[:, None]
    N = kernel.local_normals(pts.reshape(shape[0], shape[1], 3)).reshape(-1, 3)
    hgt = pts @ n + d
    with np.errstate(invalid="ignore"):
        dev = np.degrees(np.arccos(np.clip(np.abs(N @ n), 0, 1)))
    Rt = gc.mount_rotation(fit[1], fit[2])
    pb = pts @ Rt.T
    horiz = np.hypot(pb[:, 0], pb[:, 1])
    pred = gc.predicted_ground(K, u, v, h, p)
    # kernel inliers, the candidates exactly as ground_calib takes them (configured mount, RANGE_M)
    pc, nc, rc, n_pred = gc.sample_candidates(med, u, v, K, h, p, 0.0, gc.RANGE_M, shape=shape)
    nn, dd, inl = gc.fit_plane(pc, gc.up_normal(p, 0.0), prior_d=h, normals=nc)
    hc = np.hypot(*(pc @ Rt.T)[:, :2].T)
    rings = []
    for a, b in RINGS:
        sel = valid & (horiz >= a) & (horiz < b)
        flo = sel & (np.abs(hgt) < 0.005)
        dv = dev[flo & np.isfinite(dev)]
        csel = (hc >= a) & (hc < b)
        rings.append({"ring_m": [a, b], "predicted_cfg": int((pred & (horiz >= a) & (horiz < b)).sum()),
                      "valid": int(sel.sum()), "floor_5mm": int(flo.sum()),
                      "floor_normal_dev_med_deg": round(float(np.median(dv)), 2) if len(dv) else None,
                      "floor_normal_dev_p90_deg": round(float(np.percentile(dv, 90)), 2) if len(dv) else None,
                      "floor_normal_dev_gt_gate_share": round(float((dv > kernel.LOCAL_NORMAL_MAX_DEG).mean()), 3)
                      if len(dv) else None,
                      "candidates": int(csel.sum()), "inliers": int((csel & inl).sum())})
    # nearest floor the lower image edge can see (fitted mount, image centre column)
    ray = gc.rays(K, np.array([K[0, 2]]), np.array([hi - 0.5])) @ Rt.T
    dep = math.degrees(math.atan2(-ray[0, 2], math.hypot(ray[0, 0], ray[0, 1])))
    return {"npz": npz, "configured": [h, p], "verdict_check": r["verdict"], "fit": [round(x, 4) for x in fit],
            "predicted": n_pred, "inliers": int(inl.sum()), "share": round(float(inl.sum()) / max(1, n_pred), 4),
            "lower_edge_depression_deg": round(dep, 2),
            "nearest_floor_m": round(fit[0] / math.tan(math.radians(dep)), 3), "rings": rings}


def main():
    out = sys.argv[1]
    res = {}
    for arg in sys.argv[2:]:
        name, spec = arg.split("=", 1)
        npz, h, p = spec.rsplit(":", 2)
        res[name] = one(npz, float(h), float(p))
        x = res[name]
        print(name, x["fit"], "share", x["share"], "nearest", x["nearest_floor_m"])
        for r in x["rings"]:
            print("  %.1f-%.1f pred %5d floor %5d dev med %s p90 %s >gate %s inl %d/%d" % (
                r["ring_m"][0], r["ring_m"][1], r["predicted_cfg"], r["floor_5mm"], r["floor_normal_dev_med_deg"],
                r["floor_normal_dev_p90_deg"], r["floor_normal_dev_gt_gate_share"], r["inliers"], r["candidates"]))
    with open(out, "w") as f:
        json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
