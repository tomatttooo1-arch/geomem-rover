#!/usr/bin/env python3
"""Camera tilt 30 -> 10 deg (owner 10-17): does the start-up ground self-calibration (tools/car_run/ground_calib.py,
gates unchanged) still work at 5-15 deg down? docs/design/CAM_TILT_10_20261017.md section 2 (criteria written there
before this ran). Synthetic depth as tests/test_ground_calib.py: mount_sweep.render_floor (+ boxes) at a known mount,
per-frame Gaussian noise sigma = sigma0 * (1 + z^2) (z = optical depth, m), quantised to 16UC1 mm, 10 frames.

  calib_sweep.py OUT.json [--k board|fixture|both]
Two intrinsics: board = the car's live /camera/depth/camera_info (640x480 HW D2C, fx 517.17 -- decisive: the car's
ground_calib reads this K), fixture = mount_sweep.gemini2_k (640x400, 91x66 deg; the unit-test fixture).
Python 3.8 compatible."""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.car_geo_producer.common import MOUNT_XYZ, base_camera  # noqa: E402
from tools.car_geo_producer.mount_sweep import gemini2_k, render_floor  # noqa: E402
from tools.car_run import ground_calib as gc  # noqa: E402

K_BOARD = np.array([[517.1719360351562, 0.0, 324.2197265625], [0.0, 517.1170654296875, 240.64755249023438],
                    [0.0, 0.0, 1.0]])
TOL_DEG, TOL_M = 0.3, 0.005
H0, P0 = 0.375, 10.0
BOX = ((0.9, -0.10, 0.0), (1.1, 0.20, 0.15))


def kset(name):
    if name == "board":
        return K_BOARD, 640, 480
    return gemini2_k()


def render(K, W, H, h, p, r=0.0, boxes=()):
    T = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], h), p, r)
    d = render_floor(K, W, H, T)
    for b in boxes:
        db = render_floor(K, W, H, T, b)
        d = np.where(np.isfinite(db) & ~(db >= d), db, d)
    return d


def frames(depth_m, sigma0=0.002, seed=1, n=gc.FRAMES):
    rng = np.random.RandomState(seed)
    d = depth_m.astype(np.float64)
    sig = sigma0 * (1.0 + np.nan_to_num(d) ** 2)
    out = []
    for _ in range(n):
        z = d + rng.normal(size=d.shape) * sig
        out.append(np.where(np.isfinite(z) & (z > 0), np.round(z * 1000.0), 0).clip(0, 65535).astype(np.uint16))
    return out


def summary(res, truth=None):
    q = res.get("quality") or {}
    f = q.get("full") or {}
    row = {"verdict": res["verdict"], "reasons": res["reasons"],
           "predicted": f.get("predicted_ground_samples"), "candidates": f.get("candidates"),
           "inliers": f.get("inliers"), "share": None if f.get("inlier_share") is None else round(f["inlier_share"], 4),
           "rms_mm": None if f.get("rms_m") is None else round(1000 * f["rms_m"], 3),
           "bend_mm": None if f.get("bend_m") is None else round(1000 * f["bend_m"], 4),
           "halves_deg": None if q.get("halves_delta_deg") is None else round(q["halves_delta_deg"], 4),
           "halves_mm": None if q.get("halves_delta_m") is None else round(1000 * q["halves_delta_m"], 3),
           "valid_depth_share": q.get("valid_depth_share"), "range_m": (res.get("gates") or {}).get("range_m")}
    if truth is not None and res.get("fitted"):
        h, p, r = truth
        fi = res["fitted"]
        row["err"] = {"height_mm": round(1000 * abs(fi["height_m"] - h), 3), "pitch_deg": round(abs(fi["pitch_deg"] - p), 4),
                      "roll_deg": round(abs(fi["roll_deg"] - r), 4)}
        row["within"] = bool(row["err"]["height_mm"] <= 1000 * TOL_M and row["err"]["pitch_deg"] <= TOL_DEG
                             and row["err"]["roll_deg"] <= TOL_DEG)
    return row


def sweep(kname):
    K, W, H = kset(kname)
    out = {"K": K.tolist(), "size": [W, H], "range": [], "misconfig": [], "negative": {}, "wide": [], "stress": []}
    fails = []
    # A. mount range h 0.33-0.38, pitch 5-15, roll +-3, noise sigma0 1-3 mm (cycled), the box on every other cell
    k = 0
    for h in (0.33, 0.355, 0.38):
        for p in (5.0, 7.5, 10.0, 12.5, 15.0):
            for r in (-3.0, 0.0, 3.0):
                k += 1
                s0 = (0.001, 0.002, 0.003)[k % 3]
                box = bool(k % 2)
                res = gc.calibrate(frames(render(K, W, H, h, p, r, (BOX,) if box else ()), s0, seed=k), K, h, p)
                row = dict(summary(res, (h, p, r)), h=h, p=p, r=r, sigma0=s0, box=box)
                ok = res["verdict"] == "adopt" and row.get("within", False)
                row["pass"] = ok
                if not ok:
                    fails.append(("range", h, p, r))
                out["range"].append(row)
    # B. configuration off by 3 deg / 2 cm -> corrected; off by 6 deg / 4 cm -> refused (truth 0.375 / 10)
    fr = frames(render(K, W, H, H0, P0), 0.002, seed=7)
    for ch, cp, want in ((H0, P0 + 3, "adopt"), (H0, P0 - 3, "adopt"), (H0 + 0.02, P0, "adopt"), (H0 - 0.02, P0, "adopt"),
                         (H0 + 0.02, P0 - 3, "adopt"), (H0, P0 + 6, "reject"), (H0, P0 - 6, "reject"),
                         (H0 + 0.04, P0, "reject")):
        res = gc.calibrate(fr, K, ch, cp)
        row = dict(summary(res, (H0, P0, 0.0)), configured=[ch, cp], want=want)
        ok = res["verdict"] == want and (want == "reject" or row.get("within", False))
        if want == "reject":
            ok = ok and "装的位置和配置差太多" in res["reasons"][0]
        row["pass"] = ok
        if not ok:
            fails.append(("misconfig", ch, cp))
        out["misconfig"].append(row)
    # C. negative controls (must be refused)
    neg = {"wall_0.30m": [((0.40, -5.0, 0.0), (0.50, 5.0, 3.0))],
           "box_most": [((0.50, -0.6, 0.0), (0.80, 0.6, 0.6))]}
    for name, boxes in neg.items():
        res = gc.calibrate(frames(render(K, W, H, H0, P0, 0.0, boxes), 0.002), K, H0, P0)
        row = summary(res)
        row["pass"] = res["verdict"] == "reject"
        out["negative"][name] = row
        if not row["pass"]:
            fails.append(("negative", name))
    res = gc.calibrate([np.zeros((H, W), np.uint16)] * 10, K, H0, P0)
    out["negative"]["all_zero"] = dict(summary(res), **{"pass": res["verdict"] == "reject"})
    fr2 = frames(render(K, W, H, H0, P0), 0.002)[:5] + frames(render(K, W, H, H0, P0 + 1.0), 0.002, seed=2)[:5]
    res = gc.calibrate(fr2, K, H0, P0)
    out["negative"]["moving_halves_+1deg"] = dict(summary(res), **{"pass": res["verdict"] == "reject"})
    bumps = np.random.RandomState(5).randint(-60, 61, (H, W))
    rough = [np.where(f > 0, f.astype(np.int64) + bumps, 0).clip(0, 65535).astype(np.uint16)
             for f in frames(render(K, W, H, H0, P0), 0.001)]
    res = gc.calibrate(rough, K, H0, P0)
    out["negative"]["rough_floor_+-6cm"] = dict(summary(res), **{"pass": res["verdict"] == "reject"})
    for name in ("all_zero", "moving_halves_+1deg", "rough_floor_+-6cm"):
        if not out["negative"][name]["pass"]:
            fails.append(("negative", name))
    # D. wide obstacle (the kernel's hard item, here at 0.375 / 10): refused or within the criterion, never adopted wrong
    for dist in (0.5, 0.7, 0.9, 1.2, 1.6):
        for width in (0.2, 0.5, 1.0, 2.0):
            x0 = MOUNT_XYZ[0] + dist
            res = gc.calibrate(frames(render(K, W, H, H0, P0, 0.0, [((x0, -width / 2, 0.0), (x0 + 0.3, width / 2, 0.6))]), 0.002),
                               K, H0, P0)
            row = dict(summary(res, (H0, P0, 0.0)), dist=dist, width=width)
            row["pass"] = res["verdict"] == "reject" or row.get("within", False)
            if not row["pass"]:
                fails.append(("wide", dist, width))
            out["wide"].append(row)
    # E. stress (recorded, not judged): more noise than the written range, at 0.375 / 5, 10, 15
    for p in (5.0, 10.0, 15.0):
        for s0 in (0.005, 0.01):
            res = gc.calibrate(frames(render(K, W, H, H0, p), s0, seed=11), K, H0, p)
            out["stress"].append(dict(summary(res, (H0, p, 0.0)), p=p, sigma0=s0))
    # F. (baton 2, section 9.2 V3; recorded, not judged) 15-20 deg, where neither range cap is validated, and the
    # measured mount 0.39 / 7.0 with the configuration off by +-3 deg / +2 cm
    out["extra"] = []
    k = 100
    for h in (0.33, 0.355, 0.38):
        for p in (17.5, 19.9):
            k += 1
            res = gc.calibrate(frames(render(K, W, H, h, p, 0.0, (BOX,)), 0.002, seed=k), K, h, p)
            out["extra"].append(dict(summary(res, (h, p, 0.0)), case="pitch_15_20", h=h, p=p))
    fr7 = frames(render(K, W, H, 0.39, 7.0), 0.002, seed=17)
    for ch, cp in ((0.39, 7.0), (0.39, 10.0), (0.39, 4.0), (0.37, 7.0), (0.37, 10.0)):
        res = gc.calibrate(fr7, K, ch, cp)
        out["extra"].append(dict(summary(res, (0.39, 7.0, 0.0)), case="truth_0.39_7.0", configured=[ch, cp]))
    out["fails"] = fails
    out["pass"] = not fails
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--k", choices=("board", "fixture", "both"), default="both")
    a = ap.parse_args()
    res = {"criteria": "docs/design/CAM_TILT_10_20261017.md section 2", "tol": [TOL_DEG, TOL_M],
           "noise": "sigma = sigma0 * (1 + z^2), z optical depth m, Gaussian per frame per pixel, rounded to 1 mm"}
    for kn in (("board", "fixture") if a.k == "both" else (a.k,)):
        res[kn] = sweep(kn)
        print(kn, "pass" if res[kn]["pass"] else "FAIL", res[kn]["fails"], flush=True)
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
