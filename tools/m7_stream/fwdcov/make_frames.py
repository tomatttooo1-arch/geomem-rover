#!/usr/bin/env python3
"""FWD_COV: synthetic frames that hit the read-modify-write forwarding of the
geometry core's Dlat2 decision (m7_frame_stream.cpp convert(): blk_mark, ev_pl).

Writes OUTSTEM.pack (FPGAGPK01, uint16 mm depths, for the G1d/G1c reference
simulator g1d_ref) and OUTSTEM.bin (tb_frame_stream.cpp frame file; same float32
conversion as G1d crop.py: raw * unit in float32, raw 0 -> NaN).

Camera: world_camera = identity (camera at world 0, looking +z); grid 32^3,
voxel 0.05 m (ev frames 0.1 m), origin chosen off the cell boundaries so the
camera sits inside cell (16, 15, 1). Depth 9000 mm = 9 m leaves the grid ->
clipped ray. Pixel order is raster order; a 2x2 block is (v/2, u/2).

Frames (name: purpose):
  blk_w1_cl  width 1: an odd row reads blk_mark one pixel after the even row
             above wrote it (dependence distance 1). Rows 4k / 4k+1 / 4k+2 /
             4k+3 = clipped / clipped / no depth / clipped: a stale read in row
             4k+3 returns the mark of row 4k (clipped seen) instead of row 4k+2
             (nothing seen) and drops a ray that must be kept, and row 4k+1
             reads row 4k-2 (nothing) instead of 4k (clipped) and keeps one more.
  blk_w1_in  same rows with in-grid rays (bit 0 of the mark) of nearly equal
             direction, so the endpoint voxel is already marked (ev_seen).
  blk_w2_cl  width 2: distance 1 between the write of pixel (v, 1) and the read
             of (v+1, 0). Even rows: no depth, clipped; odd rows: clipped,
             clipped. A stale read returns the mark of (v, 0) (nothing seen)
             and keeps the ray (v+1, 0) that must be dropped; (v+1, 0) points
             to the other side (u spacing 0.6 rad) so its free cells are new.
  blk_w2_in  same with in-grid rays (row spacing small, endpoint voxel seen).
  blk_w3_cl  width 3: same pattern, distance 2 (hypothetical schedule).
  blk_w4_cl  width 4: same pattern, distance 3 (hypothetical schedule).
  ev_d1/2/3  32x16, 0.12 rad per pixel, voxel 0.1 m, sparse: per row pair
             three in-grid pixels B (even row), A and J (odd row, J = A + K
             pixels, K = 1/2/3), A and J end in the same voxel; J's block is
             seen through B, so Dlat2 drops J, and a ev_pl read that misses
             A's write (distance K) keeps J. Pairs chosen by ev_pairs.py so that
             J's walk marks cells no kept ray marks. (A first version with dense
             rows of equal / alternating depth changed at most 12 cells under
             the same faults: extra rays to an already-hit voxel from a nearly
             equal direction add almost nothing.)
usage: make_frames.py OUTSTEM   (env FWDCOV_EVP=a,vox,zmin,zmax,zstep for the ev frames)"""
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ev_pairs  # noqa: E402

NX = NY = NZ = 32
VOX = 0.05
UNIT = 0.001
ORIGIN = (-0.8137, -0.7863, -0.0619)  # camera q8 = (4166, 4025, 316): inside cell (16, 15, 1)
WC = (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0)
FAR = 9000  # mm -> 9 m: clipped
NAN0 = 0    # no depth -> NaN -> invalid pixel


def ik_of(a, cx, b, cy):
    # direction = (a*(u - cx), b*(v - cy), 1): ik = [a 0 -a*cx; 0 b -b*cy; 0 0 1]
    return (a, 0.0, -a * cx, 0.0, b, -b * cy, 0.0, 0.0, 1.0)


EVP = [float(x) for x in os.environ.get("FWDCOV_EVP", "0.12,0.1,150,900,7").split(",")]


def frames():
    out = []
    H = 48
    # --- blk_mark, width 1
    d = np.zeros((H, 1), np.uint16)
    for v in range(H):
        d[v, 0] = (FAR, FAR, NAN0, FAR)[v % 4]
    out.append(("blk_w1_cl", d, ik_of(0.0, 0.0, 0.04, (H - 1) / 2), VOX))
    d = np.zeros((H, 1), np.uint16)
    for v in range(H):
        d[v, 0] = (1100, 1100, NAN0, 1100)[v % 4]
    out.append(("blk_w1_in", d, ik_of(0.0, 0.0, 0.004, (H - 1) / 2), VOX))
    # --- blk_mark, width 2..4 (even rows: even u no depth, odd u clipped / in-grid;
    #     odd rows: everything clipped / in-grid)
    for w, kind in ((2, "cl"), (2, "in"), (3, "cl"), (4, "cl")):
        z = FAR if kind == "cl" else 1100
        d = np.zeros((H, w), np.uint16)
        for v in range(H):
            for u in range(w):
                d[v, u] = (NAN0 if u % 2 == 0 else z) if v % 2 == 0 else z
        a = 0.6 / max(w - 1, 1)
        b = 0.04 if kind == "cl" else 0.004
        out.append((f"blk_w{w}_{kind}", d, ik_of(a, (w - 1) / 2, b, (H - 1) / 2), VOX))
    # --- ev_pl: sparse frames of designed pixel pairs (ev_pairs.py)
    W, H2 = 32, 16
    a_ev, vox_ev = EVP[0], EVP[1]
    ik = ik_of(a_ev, (W - 1) / 2, a_ev, (H2 - 1) / 2)
    org = tuple(o / VOX * vox_ev for o in ORIGIN)
    depths = list(range(int(EVP[2]), int(EVP[3]), int(EVP[4])))
    for K in (1, 2, 3):
        pix, rep = ev_pairs.design(K, W, H2, ik, org, vox_ev, (NX, NY, NZ), depths)
        d = np.zeros((H2, W), np.uint16)
        for (v, u), z in pix.items():
            d[v, u] = z
        print(f"ev_d{K} pairs:", rep, file=sys.stderr)
        out.append((f"ev_d{K}", d, ik, vox_ev))
    return out


def main():
    stem = sys.argv[1]
    fr = frames()
    pk = bytearray(b"FPGAGPK01" + struct.pack("<4I", 1, len(fr), 0, 0))
    tb = bytearray()
    for name, d, ik, vox in fr:
        h, w = d.shape
        org = tuple(o / VOX * vox for o in ORIGIN)  # same camera cell for any voxel size
        hdr = struct.pack("<6I", w, h, NX, NY, NZ, 0) + struct.pack("<f", UNIT) + \
            struct.pack("<9f", *ik) + struct.pack("<12f", *WC) + struct.pack("<4d", *org, vox)
        assert len(hdr) == 144
        pk += hdr + np.ascontiguousarray(d, "<u2").tobytes()
        zf = d.astype(np.float32) * np.float32(UNIT)
        zf[d == 0] = np.nan
        tb += struct.pack("<5I", w, h, NX, NY, NZ) + hdr[28:64] + hdr[64:112] + hdr[112:144] + \
            np.ascontiguousarray(zf, "<f4").tobytes()
        print(f"{name} {w}x{h} valid {int((d > 0).sum())}")
    open(stem + ".pack", "wb").write(pk)
    open(stem + ".bin", "wb").write(tb)
    open(stem + ".names", "w").write("\n".join(f[0] for f in fr) + "\n")


if __name__ == "__main__":
    main()
