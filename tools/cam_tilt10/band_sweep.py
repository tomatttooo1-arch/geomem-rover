#!/usr/bin/env python3
"""Camera tilt 30 -> 10 deg (owner 10-17): false obstacles on an empty floor in the rolling map's 2-D grid.
docs/design/CAM_TILT_10_20261017.md section 5 (record definition written there before this ran).

  band_sweep.py OUT.json [--frames 40]
(sigma0 0 / 2 / 5 / 10 mm as written; 15 / 20 / 30 mm added after the first pass showed 0 at <= 10 mm, to find where it breaks)
The car's producer core + bridge (tools/nav_coldstart_crawl/crawl_lib.CarChain: host.geo_producer.GeoProducer CPU
frame + CPU rolling backends, block 32x32x10, height_cell 8, back 1; host.rolling_nav_layer.RollingLayerBridge
80x80x32, band 0 / 0.31 m, body seed 0.30 + 0.05) fed with the board camera (640x480 K_BOARD, 0.375 m, x 0.10 m),
pitch 10 vs 30 deg, the car at rest at the origin facing +x, a floor only (no walls, no boxes). Every frame gets its
own noise: sigma = sigma0 * (1 + z^2) (z optical depth, m), Gaussian, rounded to 1 mm, code 0 kept 0.
Per frame the published grid is read: LETHAL (100) cells with centre 0.5-3.0 m ahead of the camera and |y| <= 1.0 m
are false obstacles (nothing stands on the floor); FREE / UNKNOWN in the same area are counted too.
Python 3.8 compatible."""
import argparse
import json
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for p in (ROOT, os.path.join(ROOT, "tools", "nav_coldstart_crawl")):
    if p not in sys.path:
        sys.path.insert(0, p)

import crawl_lib as C  # noqa: E402
from tools.car_geo_producer.common import MOUNT_XYZ, base_camera  # noqa: E402

HEIGHT_M = 0.375
AREA_X = (0.5, 3.0)        # ahead of the camera
AREA_Y = 1.0


def noisy(codes, sigma0, rng):
    if sigma0 <= 0:
        return codes
    z = codes.astype(np.float64) / 1000.0
    n = z + rng.normal(size=z.shape) * sigma0 * (1.0 + z * z)
    out = np.where((codes > 0) & (n > 0), np.round(n * 1000.0), 0).clip(0, 65535).astype(np.uint16)
    return out


def area_counts(grid, origin):
    ny, nx = grid.shape
    xc = origin[0] + (np.arange(nx) + 0.5) * C.VOX - MOUNT_XYZ[0]
    yc = origin[1] + (np.arange(ny) + 0.5) * C.VOX
    sel = ((xc[None, :] >= AREA_X[0]) & (xc[None, :] <= AREA_X[1]) & (np.abs(yc[:, None]) <= AREA_Y))
    g = grid[sel]
    lethal = (grid >= 100) & sel
    xs = np.broadcast_to(xc[None, :], grid.shape)[lethal]
    # farthest FREE cell on the centre line (the column through y = 0), ahead of the camera
    j0 = int(np.floor((0.0 - origin[1]) / C.VOX))
    row = grid[j0]
    free_x = xc[(row == 0) & (xc > 0)]
    return {"lethal": int((g >= 100).sum()), "free": int((g == 0).sum()), "unknown": int((g < 0).sum()),
            "lethal_x_from_camera_m": [round(float(v), 3) for v in sorted(xs)][:40],
            "farthest_free_centre_from_camera_m": round(float(free_x.max()), 3) if free_x.size else None,
            "free_by_x_band": {"%.1f-%.1f" % (a, b): int(((g == 0) & (np.broadcast_to(xc[None, :], grid.shape)[sel] >= a)
                                                          & (np.broadcast_to(xc[None, :], grid.shape)[sel] < b)).sum())
                               for a, b in ((0.5, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0))}}


def run(pitch, sigma0, nframes, seed):
    ch = C.CarChain(tmp=tempfile.mkdtemp(prefix="band_"))
    ch.T_bc = base_camera((MOUNT_XYZ[0], MOUNT_XYZ[1], HEIGHT_M), pitch, 0.0)
    clean = C.render_codes((0.0, 0.0, 0.0), [], T_bc=ch.T_bc)
    rng = np.random.RandomState(seed)
    per = []
    for k in range(nframes):
        ch.frame(noisy(clean, sigma0, rng), (0.0, 0.0, 0.0))
        g, go = ch.grid((0.0, 0.0, 0.0), 0.05)
        per.append(area_counts(g, go))
    lethal = [p["lethal"] for p in per]
    return {"pitch_deg": pitch, "sigma0_m": sigma0, "frames": nframes, "valid_code_share": round(float((clean > 0).mean()), 4),
            "max_lethal_any_frame": max(lethal), "frames_with_lethal": int(sum(1 for v in lethal if v)),
            "first_frame": per[0], "last_frame": per[-1], "lethal_per_frame": lethal}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--frames", type=int, default=40)
    a = ap.parse_args()
    rows = []
    for pitch in (10.0, 30.0):
        for s0 in (0.0, 0.002, 0.005, 0.01, 0.015, 0.02, 0.03):
            r = run(pitch, s0, a.frames, seed=int(pitch * 100 + s0 * 1e4))
            rows.append(r)
            print(json.dumps({k: r[k] for k in ("pitch_deg", "sigma0_m", "max_lethal_any_frame", "frames_with_lethal")},
                             ensure_ascii=False), r["last_frame"]["free"], r["last_frame"]["unknown"],
                  r["last_frame"]["farthest_free_centre_from_camera_m"], r["last_frame"]["free_by_x_band"], flush=True)
    json.dump({"area": {"x_from_camera_m": AREA_X, "abs_y_m": AREA_Y}, "height_m": HEIGHT_M,
               "noise": "sigma = sigma0 * (1 + z^2), Gaussian per frame per pixel, 1 mm", "rows": rows},
              open(a.out, "w"), indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
