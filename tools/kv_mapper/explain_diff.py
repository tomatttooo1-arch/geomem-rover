#!/usr/bin/env python3
"""NN-MAP1 (2026-11-04): where do the RTAB-Map export and kv_mapper disagree, and why (host analysis, read only).

    python3 tools/kv_mapper/explain_diff.py --cache NODES.pkl --rtab-poses POSES.txt --tri RUN/tri.npz --out X.json

Every node's depth (stride 4) is lifted to 3-D with the node's RTAB-Map OPTIMIZED pose (the export's frame; nodes not
in the export take the correction of the exported node nearest in time, as replay_rtabdb --arm opt), and binned on
the RTAB-Map export lattice. Per cell: points in kv_mapper's obstacle band (ground + 0.05 .. 0.36 m, the car's
CAR_ROBOT_HEIGHT 0.31 + 0.05), points only above it (RTAB-Map counts obstacles up to Grid/MaxObstacleHeight 1.5 m),
and the distance to the nearest node (kv_mapper's window reaches +-2 m around the car, RTAB-Map's Grid/RangeMax 4 m).
The RTAB-Map occupied cells are then split by what kv_mapper says (occupied / free / unknown) and by these causes.
"""
import argparse
import json
import math
import os
import pickle
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.kv_mapper.replay_rtabdb import decode_depth, rvl_lib  # noqa: E402
from host import kv_mapper as km  # noqa: E402

BAND_LO, BAND_HI, TALL_HI = 0.05, 0.36, 1.5


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cache", required=True)
    ap.add_argument("--rtab-poses", required=True)
    ap.add_argument("--tri", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--stride", type=int, default=4)
    a = ap.parse_args(argv)
    from host.map_align import read_opt_poses
    nodes = pickle.load(open(a.cache, "rb"))["nodes"]
    opt = read_opt_poses(a.rtab_poses)
    z = np.load(a.tri)
    ref, ours, res, org = z["ref"], z["ours"], float(z["ref_res"]), z["ref_org"]
    h, w = ref.shape
    lib = rvl_lib(a.build_dir)
    cal = nodes[0]["calib"]
    T_bc = np.eye(4)
    T_bc[:3, :] = np.asarray(cal["local"], float).reshape(3, 4)
    K = np.asarray(cal["K"], float).reshape(3, 3)
    by_id = {nd["id"]: nd for nd in nodes}
    rt_ids = sorted(i for i in opt if i in by_id)
    rt_st = np.array([opt[i][0] for i in rt_ids])
    corr = {}
    for i in rt_ids:
        T = np.eye(4)
        T[:3, :] = np.asarray(by_id[i]["pose12"], float).reshape(3, 4)
        corr[i] = opt[i][1] @ np.linalg.inv(T)
    band = np.zeros((h, w), np.int32)
    tall = np.zeros((h, w), np.int32)
    near = np.full((h, w), np.inf)
    yy, xx = np.mgrid[0:h, 0:w]
    cx, cy = org[0] + (xx + 0.5) * res, org[1] + (yy + 0.5) * res
    for nd in nodes:
        d = decode_depth(lib, nd["depth"])[::a.stride, ::a.stride].astype(np.float64) * 1e-3
        v, u = np.nonzero((d > 0.1) & (d < 4.0))
        zc = d[v, u]
        xc = (u * a.stride - K[0, 2]) / K[0, 0] * zc
        yc = (v * a.stride - K[1, 2]) / K[1, 1] * zc
        T = np.eye(4)
        T[:3, :] = np.asarray(nd["pose12"], float).reshape(3, 4)
        j = rt_ids[int(np.argmin(np.abs(rt_st - nd["stamp"])))]
        Tm = corr[j] @ T @ T_bc
        p = Tm[:3, :3] @ np.vstack([xc, yc, zc]) + Tm[:3, 3:4]
        gx = np.floor((p[0] - org[0]) / res).astype(int)
        gy = np.floor((p[1] - org[1]) / res).astype(int)
        ok = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        hz = p[2]
        inb = ok & (hz >= BAND_LO) & (hz <= BAND_HI)
        tl = ok & (hz > BAND_HI) & (hz <= TALL_HI)
        np.add.at(band, (gy[inb], gx[inb]), 1)
        np.add.at(tall, (gy[tl], gx[tl]), 1)
        bx, by = (corr[j] @ T)[0, 3], (corr[j] @ T)[1, 3]
        near = np.minimum(near, np.hypot(cx - bx, cy - by))
    r_occ = ref == km.OCC
    out = {"cells_rtab_occupied": int(r_occ.sum())}
    for name, sel in (("ours_occupied", ours == km.OCC), ("ours_free", ours == km.FREE_V), ("ours_unknown", ours == km.UNK)):
        m = r_occ & sel
        has_band = m & (band >= 3)
        tall_only = m & (band < 3) & (tall >= 3)
        far = m & (near > 2.0)
        out[name] = {"cells": int(m.sum()), "band_points>=3": int(has_band.sum()),
                     "only_above_band": int(tall_only.sum()), "neither": int((m & (band < 3) & (tall < 3)).sum()),
                     "farther_than_2m_from_every_node": int(far.sum())}
    o_occ_ref_free = (ours == km.OCC) & (ref == km.FREE_V)
    out["ours_occupied_rtab_free"] = {"cells": int(o_occ_ref_free.sum()),
                                      "band_points>=3": int((o_occ_ref_free & (band >= 3)).sum())}
    with open(a.out, "w") as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out))


if __name__ == "__main__":
    main()
