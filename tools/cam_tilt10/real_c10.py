#!/usr/bin/env python3
"""Camera tilt 10 deg: why the board's start-up ground calibration c10_1 (another session, runs/cam10-20261017,
10-17 14:31; real Gemini 2 depth, 10 frames, the car at rest in the classroom) was refused at the configured
0.375 m / 10 deg (inlier share 48.3 % < 50 %), while the fit is 0.3915 m / 7.05 deg. Analysis only: the gates and
RANGE_M of tools/car_run/ground_calib.py are NOT changed (the range-cap rows monkeypatch a copy for this script).
docs/design/CAM_TILT_10_20261017.md section 2.3.

  real_c10.py NPZ OUT.json
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

FIT = (0.3915, 7.05, 0.59)


def calib_row(frames, K, h, p, mode="auto"):
    r = gc.calibrate(frames, K, h, p, mode=mode)
    f = r["quality"]["full"] if r.get("quality") else {}
    return {"configured": [h, p], "mode": mode, "verdict": r["verdict"], "reasons": [x[:80] for x in r["reasons"]],
            "predicted": f.get("predicted_ground_samples"), "inliers": f.get("inliers"),
            "share": round(f["inlier_share"], 4) if f.get("inlier_share") is not None else None,
            "fitted": r.get("fitted")}


def synthetic_repro():
    """the same failure on synthetic depth (tools/cam_tilt10/calib_sweep render/frames, board K): truth 0.3915 m /
    7.05 deg / roll 0.59, a 2 m wide 0.6 m tall box whose front face is 1.45 m ahead of the camera (the cabinet of
    c10_1), configured 0.375 / 10 vs 0.39 / 7.0; with no box as the control. Recorded, not judged."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import calib_sweep as S
    from tools.car_geo_producer.common import MOUNT_XYZ
    K, W, H = S.kset("board")
    rows = []
    for front in (None, 1.45, 1.2):
        boxes = [] if front is None else [((MOUNT_XYZ[0] + front, -1.0, 0.0), (MOUNT_XYZ[0] + front + 0.4, 1.0, 0.6))]
        fr = S.frames(S.render(K, W, H, FIT[0], FIT[1], FIT[2], boxes), 0.002, seed=21)
        for h, p, mode in ((0.375, 10.0, "auto"), (0.39, 7.0, "check"), (0.39, 7.0, "auto")):
            r = calib_row(fr, K, h, p, mode)
            r["box_front_m"] = front
            rows.append(r)
    return rows


def main():
    npz, out = sys.argv[1], sys.argv[2]
    z = np.load(npz)
    frames, K = list(z["frames"]), np.asarray(z["K"], dtype=np.float64)
    h_img, w_img = frames[0].shape
    res = {"npz": npz, "fit_used_for_geometry": FIT, "calibrate": [], "range_cap": [], "where": {}}
    for h, p, mode in ((0.375, 10.0, "auto"), (0.375, 10.0, "check"), (0.39, 7.0, "auto"), (0.39, 7.0, "check"),
                       (0.375, 7.0, "auto"), (0.3915, 7.05, "auto"), (0.39, 8.5, "auto"), (0.39, 10.0, "auto")):
        res["calibrate"].append(calib_row(frames, K, h, p, mode))
    # where the predicted ground samples of the 10 deg configuration really land (median depth, the fitted mount)
    u, v = gc.sample_pixels(w_img, h_img)
    iv, iu = v.astype(int), u.astype(int)
    med = gc.median_depth([gc.depth_mm_to_m(np.asarray(f)[iv, iu]) for f in frames])
    full = gc.calibrate(frames, K, 0.39, 7.0, mode="check")["quality"]["full"]
    n = np.asarray(full["normal"])
    d = full["d"]
    pts = gc.rays(K, u, v) * med[:, None]
    hgt = pts @ n + d                                  # height above the fitted floor (m)
    R = gc.mount_rotation(FIT[1], FIT[2])
    pb = pts @ R.T
    horiz = np.hypot(pb[:, 0], pb[:, 1])               # horizontal distance from the camera, fitted mount
    for name, (h, p) in (("cfg_10_0.375", (0.375, 10.0)), ("cfg_7.0_0.39", (0.39, 7.0))):
        pred = gc.predicted_ground(K, u, v, h, p)
        ok = pred & np.isfinite(med)
        on_floor = ok & (np.abs(hgt) < 0.02)
        above = ok & (hgt >= 0.02)
        # the far edge of the prediction: the predicted pixel row nearest the horizon, and where the TRUE mount puts
        # that row's floor
        rows = v[pred]
        top_row = rows.min()
        ray = gc.rays(K, np.array([K[0, 2]]), np.array([float(top_row)])) @ gc.mount_rotation(FIT[1], FIT[2]).T
        dep = -ray[0, 2] / math.hypot(ray[0, 0], ray[0, 1])
        bands = {}
        for a, b in ((0.0, 1.0), (1.0, 1.4), (1.4, 2.0), (2.0, 3.0), (3.0, 99.0)):
            sel = ok & (horiz >= a) & (horiz < b)
            bands["%.1f-%.1f" % (a, b)] = {"samples": int(sel.sum()), "floor_|h|<2cm": int((sel & on_floor).sum()),
                                          "above_floor_>=2cm": int((sel & above).sum())}
        # the kernel's inliers by the same distance bands (candidates as ground_calib takes them)
        pc, nc, rc, _ = gc.sample_candidates(med, u, v, K, h, p, 0.0, gc.RANGE_M, shape=gc.grid_shape(w_img, h_img))
        nn, dd, inl = gc.fit_plane(pc, gc.up_normal(p, 0.0), prior_d=h, normals=nc)
        hc = np.hypot(*(pc @ R.T)[:, :2].T)
        hh = pc @ n + d
        inl_bands = {}
        for a, b in ((0.0, 1.0), (1.0, 1.4), (1.4, 2.0), (2.0, 3.0)):
            sel = (hc >= a) & (hc < b)
            inl_bands["%.1f-%.1f" % (a, b)] = {"candidates": int(sel.sum()), "inliers": int((sel & inl).sum()),
                                              "floor_|h|<2cm_not_inlier": int((sel & ~inl & (np.abs(hh) < 0.02)).sum()),
                                              "floor_|h|<5mm_not_inlier": int((sel & ~inl & (np.abs(hh) < 0.005)).sum())}
        res.setdefault("inliers_by_band", {})[name] = inl_bands
        res["where"][name] = {"predicted": int(pred.sum()), "with_depth": int(ok.sum()),
                              "floor_|h|<2cm": int(on_floor.sum()), "above_floor_>=2cm": int(above.sum()),
                              "below_floor_<=-2cm": int((ok & (hgt <= -0.02)).sum()),
                              "far_edge_row": int(top_row),
                              "far_edge_true_floor_distance_m": round(FIT[0] / dep, 3) if dep > 0 else None,
                              "by_true_horizontal_distance_m": bands}
    # range caps (analysis only): the same 10 frames with RANGE_M (0.3, cap)
    keep = gc.RANGE_M
    try:
        for cap in (2.0, 1.6, 1.4, 1.2, 1.0):
            gc.RANGE_M = (0.3, cap)
            for h, p in ((0.375, 10.0), (0.39, 7.0)):
                # sample_candidates / predicted_ground take range_m as a default bound at import: pass it explicitly
                pred = gc.predicted_ground(K, u, v, h, p, range_m=gc.RANGE_M)
                pts_c, nrm, rws, n_pred = gc.sample_candidates(med, u, v, K, h, p, 0.0, gc.RANGE_M,
                                                                shape=gc.grid_shape(w_img, h_img))
                nn, dd, inl = gc.fit_plane(pts_c, gc.up_normal(p, 0.0), prior_d=h, normals=nrm)
                fh = gc.pose_from_plane(nn, dd) if nn is not None else None
                res["range_cap"].append({"cap_m": cap, "configured": [h, p], "predicted": int(pred.sum()),
                                         "inliers": int(inl.sum()), "share": round(float(inl.sum()) / max(1, n_pred), 4),
                                         "fit": None if fh is None else [round(fh[0], 4), round(fh[1], 3), round(fh[2], 3)]})
    finally:
        gc.RANGE_M = keep
    res["synthetic_repro"] = synthetic_repro()
    with open(out, "w") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print(json.dumps(res, indent=1, ensure_ascii=False)[:6000])


if __name__ == "__main__":
    main()
