#!/usr/bin/env python3
"""Known answers before the crawl runs (docs/design/NAV_COLDSTART_CRAWL_20261017.md section 2).

  ka_host.py BOARD_RUN_DIR OUT.json
BOARD_RUN_DIR = .../board_1017_0211/nano/runs (holds ng2_1017/ng/grid_0135.json and ng3_1017/{ground_calib_depth.npz,
ng/grid_pre.json}). Host only (no ROS): the car's producer + bridge on CPU backends, the Nav2 DWB replica.
  K1  the board's own depth frames (ng3 ground calibration, 10 frames, car static) through the chain -> the grid,
      compared cell by cell with the grid the board published at the same place (ng3 grid_pre.json)
  K2  the synthetic camera (floor + room, same K / mount) -> the grid, compared with K1 and the board
  K3  the replica: board fixture (ng2 art) + 5 cm = 144/419, 20/20 rotations, straight <= 0.047 m/s; the same
      numbers on the board ng3 grid, on K1 and on K2
  K4  the three classes of section 9.1 (start outline without margin; 0.15 m/s straight 1.7 s; rotation) on each
"""
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crawl_lib as C  # noqa: E402


def fixture_grid(margin):
    sys.path.insert(0, os.path.join(C.ROOT, "tests"))
    import test_nav_golive_r3 as T
    g = T.published(T.bridge_with(T.art_cells(), margin))
    return np.asarray(g), (-2.0, -2.0)


def board_grid(path, topic='/fpga/rolling_map/grid'):
    d = json.load(open(path))[topic]
    return np.asarray(d['data']).reshape(d['h'], d['w']), tuple(d['origin'])


def compare(a, b, window=None, origin=(-2.0, -2.0)):
    """cell agreement of two [j, i] grids; window (x0, x1, y0, y1) in metres or None = whole"""
    sa, sb = np.sign(a), np.sign(b)          # -1 unknown, 0 free, 1 lethal
    mask = np.ones(a.shape, bool)
    if window is not None:
        x0, x1, y0, y1 = window
        jj, ii = np.mgrid[0:a.shape[0], 0:a.shape[1]]
        cx, cy = origin[0] + (ii + 0.5) * C.VOX, origin[1] + (jj + 0.5) * C.VOX
        mask = (cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1)
    diff = (sa != sb) & mask
    kinds = {}
    for j, i in np.argwhere(diff):
        k = "%d->%d" % (sa[j, i], sb[j, i])
        kinds[k] = kinds.get(k, 0) + 1
    return {"cells": int(mask.sum()), "differ": int(diff.sum()), "kinds": kinds}


def classes(grid, gorigin, margin_grid=None):
    """section 9.1 table rows for the start pose (0, 0, 0) on a margin-0 grid"""
    origin = C.costmap_origin(0.0, 0.0)
    start = C.outline_world_cells([(0.0, 0.0, 0.0)], origin)
    fast = C.outline_world_cells(C.trajectory(0.15, 0.0), origin)
    rot = C.outline_world_cells(C.trajectory(0.0, 0.4) + C.trajectory(0.0, -0.4), origin)
    return {name: C.classify_unknown_outline(grid, gorigin, (0, 0, 0), cells)
            for name, cells in (("start_outline", start), ("straight_0.15_1.7s", fast), ("rotate_0.4", rot))}


def dwb(grid, gorigin):
    origin = C.costmap_origin(0.0, 0.0)
    m = C.master_from_grid(grid, gorigin, origin)
    _, s = C.evaluate(m, origin)
    return s


def main():
    runs, out = sys.argv[1], sys.argv[2]
    res = {}
    # ---- K1: the board's own depth frames through the car chain
    z = np.load(os.path.join(runs, "ng3_1017", "ground_calib_depth.npz"))
    frames, K = z["frames"], z["K"]
    assert np.allclose(K, C.K_BOARD), K
    real = C.CarChain()
    acc = 0
    for rep in range(4):
        for f in frames:
            acc += real.frame(f, (0.0, 0.0, 0.0))
    g_real5, o_real = real.grid((0, 0, 0), 0.05)
    g_real0, _ = real.grid((0, 0, 0), 0.0)
    b3, o_b3 = board_grid(os.path.join(runs, "ng3_1017", "ng", "grid_pre.json"))
    assert o_b3 == (-2.0, -2.0) and o_real == (-2.0, -2.0), (o_b3, o_real)
    near = (-0.6, 1.2, -0.6, 0.6)
    res["K1"] = {"frames_fused": acc,
                 "vs_board_ng3_whole": compare(g_real5, b3), "vs_board_ng3_near": compare(g_real5, b3, near),
                 "art_host_real_m5": C.art(g_real5, o_real), "art_board_ng3": C.art(b3, o_b3)}
    # ---- K2: the synthetic camera, floor + room
    syn = C.CarChain()
    codes = C.render_codes((0.0, 0.0, 0.0), C.ROOM)
    nonzero = float((codes > 0).mean())
    for _ in range(40):
        syn.frame(codes, (0.0, 0.0, 0.0))
    g_syn5, o_syn = syn.grid((0, 0, 0), 0.05)
    g_syn0, _ = syn.grid((0, 0, 0), 0.0)
    fx0, _ = fixture_grid(0.0)
    fx5, _ = fixture_grid(0.05)
    near_floor = (-0.6, 0.9, -0.6, 0.6)       # in front of the board scene's objects (lethal from x 1.45 m)
    res["K2"] = {"valid_pixel_share": nonzero,
                 "synthetic_vs_host_real_near": compare(g_syn5, g_real5, near_floor),
                 "synthetic_vs_board_ng3_near": compare(g_syn5, b3, near_floor),
                 "synthetic_vs_board_ng2_fixture_m0_near": compare(g_syn0, fx0, near_floor),
                 "host_real_vs_board_ng2_fixture_m0_near": compare(g_real0, fx0, near_floor),
                 "art_synthetic_m5": C.art(g_syn5, o_syn), "art_synthetic_m0": C.art(g_syn0, o_syn),
                 "real_vs_synthetic_depth_mm": depth_stats(frames[0], codes)}
    # ---- K3 / K4
    res["K3"] = {"fixture_ng2_m5": dwb(fx5, (-2.0, -2.0)), "fixture_ng2_m0": dwb(fx0, (-2.0, -2.0)),
                 "board_ng3_m5": dwb(b3, o_b3), "host_real_m5": dwb(g_real5, o_real),
                 "synthetic_m5": dwb(g_syn5, o_syn), "synthetic_m0": dwb(g_syn0, o_syn)}
    res["K4"] = {"fixture_ng2_m0": classes(fx0, (-2.0, -2.0)), "host_real_m0": classes(g_real0, o_real),
                 "synthetic_m0": classes(g_syn0, o_syn)}
    json.dump(res, open(out, "w"), indent=1)
    for k in ("K1", "K2"):
        print(k, json.dumps({kk: vv for kk, vv in res[k].items() if not kk.startswith("art")}))
    print("K3", json.dumps(res["K3"], indent=0))
    print("K4", json.dumps(res["K4"]))
    for name in ("art_board_ng3", "art_host_real_m5"):
        print(name)
        print("\n".join(res["K1"][name]))
    print("art_synthetic_m5")
    print("\n".join(res["K2"]["art_synthetic_m5"]))


def depth_stats(real, syn):
    """per image row band: median |real - synthetic| mm where both valid (floor rows only, bottom half)"""
    out = {}
    h = real.shape[0]
    for r0 in range(h // 2, h, 40):
        a, b = real[r0:r0 + 40].astype(float), syn[r0:r0 + 40].astype(float)
        ok = (a > 0) & (b > 0)
        out["rows_%d_%d" % (r0, r0 + 40)] = None if not ok.any() else round(float(np.median(np.abs(a - b)[ok])), 1)
    return out


if __name__ == "__main__":
    main()
