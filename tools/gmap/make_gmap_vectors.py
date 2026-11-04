#!/usr/bin/env python3
"""GMAP2 key comparison 2 vectors (GMAP0 DESIGN section 8, written before the run):
  vec1  synthetic, 24 frames: odd / even ix and iy, ix mod 8 all eight, windows pressing the left / right / top /
        bottom edges and the corners, two windows completely outside the map, frame values other than 100 / 0 / -1;
        CLEAR first (from random bytes) and once after frame 12, SCAN with the PGM table (flip) and with the
        tri-state table (no flip) at the end, plus one bad-magic MERGE and one bad op (5).
  vec2  20261101_040338: the first 40 real frames (PS lattice offsets) + SCAN (PGM), starting from the cleared state.
Also writes frames_syn.npz (the 24 synthetic frames in check_gmap.py's format) so the same frames can be run through
the PS reference (kv_mapper integer version) with tools/gmap/check_gmap.py.
    python3 make_gmap_vectors.py OUTDIR FRAMES_040338.npz
"""
import os
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from host import kv_mapper as km  # noqa: E402

PGM = (254, 205, 0)
TRI = (0x00, 0xFF, 0x64)
CT = tuple(int(v) for v in km.COARSE_TABLE)


def prm(ix=0, iy=0, ctab=CT, itab=PGM, flip=1, magic=0x474D):
    return struct.pack("<Hhh3B3BB3x", magic, ix, iy, *ctab, *itab, flip)


def write_vec(path, init, ops):
    with open(path, "wb") as f:
        f.write(b"GMVEC001" + struct.pack("<II", init, len(ops)))
        for op, p, fr in ops:
            f.write(struct.pack("<I", op) + p + struct.pack("<I", 0 if fr is None else 1))
            if fr is not None:
                f.write(np.ascontiguousarray(fr, np.int8).view(np.uint8).tobytes())


def synth_frames():
    rng = np.random.default_rng(20261104)
    offs = [(0, 0), (1, 1), (2, 3), (3, 2), (4, 5), (5, 4), (6, 7), (7, 6),     # ix mod 8, parities, at the corner
            (-30, 100), (350, 101), (161, -25), (160, 351),                    # left / right / top / bottom edges
            (-79, -79), (399, 399), (-200, 50), (100, 480),                    # corners by 1 cell, fully outside
            (150, 150), (151, 151), (155, 149), (-1, 200), (321, -1), (200, 320), (240, 241), (313, 77)]
    frames = []
    for k, (ix, iy) in enumerate(offs):
        d = rng.choice(np.array([-1, 0, 100], np.int8), size=(80, 80), p=[0.4, 0.4, 0.2])
        odd = rng.random((80, 80)) < 0.05
        d[odd] = rng.choice(np.array([1, 50, 99, 101, 127, -128, -2, -100], np.int8), size=int(odd.sum()))
        frames.append((ix, iy, d))
    return frames


def main():
    out, f040338 = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    fr = synth_frames()
    ops = [(3, prm(), None)]
    for k, (ix, iy, d) in enumerate(fr):
        ops.append((1, prm(ix, iy), d))
        if k == 12:
            ops.append((3, prm(), None))
    ops.append((1, prm(10, 10, magic=0x1234), fr[0][2]))      # bad magic: only the result words change
    ops.append((5, prm(), None))                              # bad op
    ops.append((2, prm(itab=PGM, flip=1), None))
    ops.append((2, prm(itab=TRI, flip=0), None))
    write_vec(os.path.join(out, "vec1.bin"), 0, ops)
    v = km.V if hasattr(km, "V") else 0.05
    np.savez(os.path.join(out, "frames_syn.npz"), stamp=np.arange(len(fr), dtype=np.float64),
             origin=np.array([[ix * 0.05 - 10.0, iy * 0.05 - 10.0] for ix, iy, _ in fr]),
             data=np.array([d for _, _, d in fr]), pose=np.zeros((len(fr), 3)))
    z = np.load(f040338)
    ref = km.KvMapper(rule="q13")
    ops2 = []
    for i in range(40):
        off = ref.lattice_offset(float(z["origin"][i, 0]), float(z["origin"][i, 1]))
        ops2.append((1, prm(off[0], off[1]), z["data"][i]))
    ops2.append((2, prm(itab=PGM, flip=1), None))
    write_vec(os.path.join(out, "vec2.bin"), 1, ops2)
    print("vec1 ops %d, vec2 ops %d, v=%s" % (len(ops), len(ops2), v))


if __name__ == "__main__":
    main()
