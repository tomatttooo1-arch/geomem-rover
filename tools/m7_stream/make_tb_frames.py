#!/usr/bin/env python3
"""Write the frame file read by hw/m7/pl/hls/tb_frame_stream.cpp (env FPGA_M7_TB_FRAMES).

Record: uint32 w,h,nx,ny,nz; float32 ik[9]; float32 wc[12]; float64 origin[3], voxel;
float32 depth[w*h] (metres, NaN = no return), all little-endian.
Frames (each optional):
  --rec30 DIR:I[:R0-R1]  prepared recording frame I (only rows [R0,R1) if given)
  --tum DIR:I[:R0-R1]    same for TUM (unit from meta depth_unit_m)
  --synthetic NAME       a frame of tools/m7_clip/positive_control realistic_frames
                         (seed 20261011), e.g. tilt_down_below_floor
  --range-gate           a 40x30 frame whose far endpoints fail the +-2^24 Q8 gate
Usage: make_tb_frames.py --out FILE [--rec30 ...] [--tum ...] [--synthetic ...] [--range-gate]
"""
import argparse
import json
import os
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.m7_clip.positive_control import inv_k, pose, realistic_frames  # noqa: E402

WC_ID = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]], dtype=np.float32)


def record(depth, ik, wc, origin, voxel, dims):
    depth = np.ascontiguousarray(depth, dtype="<f4")
    h, w = depth.shape
    return (struct.pack("<5I", w, h, *dims) + np.asarray(ik, dtype="<f4").ravel().tobytes() +
            np.asarray(wc, dtype="<f4").ravel().tobytes() + struct.pack("<4d", *origin, voxel) +
            depth.tobytes())


def prepared(spec):
    parts = spec.split(":")
    d, i = parts[0], int(parts[1])
    rows = None
    if len(parts) > 2:  # "R1" = rows [0,R1), "R0-R1" = rows [R0,R1)
        r = [int(x) for x in parts[2].split("-")]
        rows = (0, r[0]) if len(r) == 1 else (r[0], r[1])
    meta = json.load(open(os.path.join(d, "meta.json")))
    W, H, N = meta["width"], meta["height"], meta["frames"]
    raw = np.memmap(os.path.join(d, meta["depth_file"]), dtype="<u2", mode="r", shape=(N, H, W))[i]
    if rows:
        raw = raw[rows[0]:rows[1]]
    z = raw.astype(np.float32) * np.float32(float(meta.get("depth_unit_m", 0.001)))
    z[raw == 0] = np.nan
    ik = np.array([float.fromhex(x) for x in meta["inverse_k_f32_hex"]], dtype=np.float32)
    g = meta["grid"]
    return record(z, ik, WC_ID, tuple(g["origin_m"]), float(g["voxel_m"]), tuple(g["shape_xyz"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--rec30", action="append", default=[])
    ap.add_argument("--tum", action="append", default=[])
    ap.add_argument("--synthetic", action="append", default=[])
    ap.add_argument("--range-gate", action="store_true")
    a = ap.parse_args()
    recs = []
    for s in a.rec30 + a.tum:
        recs.append(prepared(s))
    if a.synthetic:
        frames = {f["name"]: f for f in realistic_frames(np.random.default_rng(20261011))}
        for n in a.synthetic:
            f = frames[n]
            recs.append(record(f["depth"], f["ik"], f["wc"], f["grid_origin"], f["voxel"], f["dims"]))
    if a.range_gate:
        vox = 5e-5
        d = np.random.default_rng(20261012).uniform(0.11, 9.9, size=(30, 40)).astype(np.float32)
        recs.append(record(d, inv_k(25., 25., 19.5, 14.5), pose(np.eye(3), (0., 0., 0.)),
                           (-16 * vox, -16 * vox, -1 * vox), vox, (32, 32, 32)))
    with open(a.out, "wb") as fp:
        for r in recs:
            fp.write(r)
    print("%s: %d frames, %d bytes" % (a.out, len(recs), sum(len(r) for r in recs)))


if __name__ == "__main__":
    main()
