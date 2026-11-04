#!/usr/bin/env python3
"""Camera mount sweep for the car's per-frame block (host only, synthetic floor).

Why: replaying a recording (rec30 / TUM) under a synthetic mount is fine for equivalence, but
its fused-frame ratio is not the car's: the recorded surfaces keep the recording camera's
geometry, so pitching the synthetic mount down rotates far surfaces below the floor and out
of the 0.5 m tall block (TUM: 251/303 fused level, 8-10/303 at 10-35 deg). On the car the
floor itself is in view. This renders the depth a camera at (height, pitch) sees of a flat
floor (plus an optional box), runs the producer's block (FollowCameraGrid 32x32x10, the
height_cell check_block_height asks for) through M7Pipeline with the CPU frame core and
reports, per mount: block ok / height_cell needed, whether the frame yields geometry
(valid samples > 0 -> it can be fused), valid / clipped / invalid pixels, the nearest floor
seen ahead of the camera, and the farthest distance along the centre line where the whole
obstacle band (ground + 5 cm .. robot height + 5 cm) is FREE in this one frame.
Usage: mount_sweep.py --out NEW.json [--ref-frame-src frame_depth.cpp] [--robot-height-m 0.25]
"""
import argparse
import json
import math
import os
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.geo_producer import DEFAULT_SHAPE, VOXEL_M, check_block_height  # noqa: E402
from tools.car_geo_producer.common import base_camera  # noqa: E402


def gemini2_k(w=640, h=400, hfov=91.0, vfov=66.0):
    """Gemini 2 depth FOV (datasheet: 91 x 66 deg at 1280x800), half resolution."""
    fx = (w / 2) / math.tan(math.radians(hfov / 2))
    fy = (h / 2) / math.tan(math.radians(vfov / 2))
    return np.array([[fx, 0, (w - 1) / 2], [0, fy, (h - 1) / 2], [0, 0, 1]], dtype=np.float64), w, h


def render_floor(K, w, h, T_bc, box=None):
    """Optical-z depth (m, float32, NaN = no return) of floor z=0 in base_link, optional
    axis-aligned box ((x0,y0,z0),(x1,y1,z1)) in base_link."""
    u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    d_opt = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], -1)   # z = 1
    R, c = T_bc[:3, :3], T_bc[:3, 3]
    d = d_opt @ R.T                                     # direction in base per unit optical z
    t = np.full(u.shape, np.inf)
    down = d[..., 2] < -1e-9
    t[down] = -c[2] / d[..., 2][down]                   # floor hit (t = optical z)
    if box is not None:
        lo, hi = np.asarray(box[0], float), np.asarray(box[1], float)
        with np.errstate(divide="ignore", invalid="ignore"):
            t0 = (lo - c) / d
            t1 = (hi - c) / d
        tmin = np.nanmax(np.minimum(t0, t1), axis=-1)
        tmax = np.nanmin(np.maximum(t0, t1), axis=-1)
        hitb = (tmax >= tmin) & (tmin > 0)
        t = np.where(hitb & (tmin < t), tmin, t)
    depth = t.astype(np.float32)
    depth[~np.isfinite(depth) | (depth > 10.0)] = np.nan
    return depth


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--ref-frame-src", default=None)
    ap.add_argument("--robot-height-m", type=float, default=0.25)
    ap.add_argument("--heights", default="0.25,0.30,0.33,0.35,0.36,0.38,0.40,0.45")
    ap.add_argument("--pitches", default="0,10,15,20,25,27,30,35,40")
    ap.add_argument("--box", action="store_true", help="add a 0.20 x 0.30 x 0.15 m box 0.8 m ahead")
    a = ap.parse_args(argv)
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    from host.m7_grid_policy import FollowCameraGrid
    from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
    from tools.car_geo_producer.equivalence import compile_ref_frame
    build = tempfile.mkdtemp(prefix="mount_sweep_")
    lib = (compile_ref_frame(a.ref_frame_src, os.path.join(build, "f.so")) if a.ref_frame_src
           else compile_frame_native(os.path.join(build, "f.so")))
    backend = M7BackendSelector(cpu_frame_library=lib).frame_backend("cpu")
    K, w, h = gemini2_k()
    shape = DEFAULT_SHAPE
    band = (0.05, a.robot_height_m + 0.05)
    rows = []
    box = ((0.8, -0.10, 0.0), (1.0, 0.10, 0.15)) if a.box else None
    for hm in (float(x) for x in a.heights.split(",")):
        ok, info = check_block_height(hm, 0.0, shape=shape, height_cell=7)
        hc = 7 if ok else info.get("need_height_cell_at_least")
        if not ok and hc is not None and hc <= shape[2] - 2:
            ok, info = check_block_height(hm, 0.0, shape=shape, height_cell=hc)
        for p in (float(x) for x in a.pitches.split(",")):
            row = {"height_m": hm, "pitch_deg": p, "block_ok": bool(ok), "height_cell": hc,
                   "block_bottom_m": info["block_bottom_m"], "block_top_m": info["block_top_m"],
                   "band_top_inside": info["block_top_m"] >= band[1]}
            if not ok:
                rows.append(row)
                continue
            T_bc = base_camera(xyz=(0.0, 0.0, hm), pitch_deg=p)
            depth = render_floor(K, w, h, T_bc, box)
            policy = FollowCameraGrid(VOXEL_M, shape, back_cells=1, height_cell=hc)
            pipe = M7Pipeline(backend, GridSpec((0., 0., 0.), VOXEL_M, shape), grid_policy=policy)
            ident = CaptureIdentity("sweep", 1, 10 ** 12, "c", "cal", "odom", "e")
            out = pipe.ingest_geometry(DepthFrame(ident, depth, K, PoseAtCapture(10 ** 12, "c", "odom", "e", True, T_bc)),
                                       now_ns=10 ** 12 + 1000, now_clock_id="c")
            nav = out.navigation
            row["fusable"] = bool(nav.valid)
            row["reason"] = nav.reason
            ahead = None
            fwd = None
            vis = np.isfinite(depth)
            if vis.any():
                # nearest floor point seen ahead (base x), over all valid pixels on the floor
                u, v = np.meshgrid(np.arange(w), np.arange(h))
                X = ((u - K[0, 2]) / K[0, 0] * depth, (v - K[1, 2]) / K[1, 1] * depth, depth)
                P = np.stack(X, -1)[vis].astype(np.float64) @ T_bc[:3, :3].T + T_bc[:3, 3]
                fl = np.abs(P[:, 2]) < 1e-3
                ahead = float(P[fl, 0].min()) if fl.any() else None
            row["nearest_floor_seen_m"] = None if ahead is None else round(ahead, 3)
            row["pixels"] = int(depth.size)
            if nav.valid:
                pl = nav.payload
                row.update({"valid": pl["valid_grid_samples"], "clipped_or_outside": pl["outside_grid"],
                            "invalid": pl["invalid_depth_or_projection"]})
                g, f = pl["grid"], pl["voxel_flags"]            # f[z, y, x]
                zc = (np.arange(shape[2]) + 0.5) * VOXEL_M + g.origin_m[2]
                zsel = (zc >= band[0]) & (zc <= band[1])
                yc = int(math.floor((0.0 - g.origin_m[1]) / VOXEL_M))
                xc = (np.arange(shape[0]) + 0.5) * VOXEL_M + g.origin_m[0]
                col_free = (f[zsel][:, yc, :] == 1).all(axis=0)
                fwd = float(xc[col_free].max()) if col_free.any() else None
                row["band_free_cols_centre"] = int(col_free.sum())
                row["farthest_band_free_centre_m"] = None if fwd is None else round(fwd, 3)
                row["hit_cells"] = int((f == 2).sum())
                row["free_cells"] = int((f == 1).sum())
            rows.append(row)
    res = {"K": K.tolist(), "size": [w, h], "shape": list(shape), "band_m": band, "box": box,
           "ref_frame_src": a.ref_frame_src, "rows": rows}
    with open(a.out, "x") as fp:
        json.dump(res, fp, indent=1)
    for r in rows:
        print(json.dumps(r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
