#!/usr/bin/env python3
"""GMAP1: input files and goldens of the A53 timing program (tools/gmap/a53_gmap_bench.c), format as A53BENCH0.

Per trip one frame file  "GMAPFRM1" | u32 n | n x (int16 ix, int16 iy, 6400 frame bytes)
(ix, iy = kv_mapper.lattice_offset of the frame corner, the PS's part of a MERGE), and two goldens computed by the
PS reference (host/kv_mapper.py rule q13, NOT by the C model):
  merge  sha256(global int16 LE 320,000 B || coarse 40,000 B) after the whole sequence
  scan   sha256(image 160,000 B (PGM table 254/205/0, y flip) || coarse 40,000 B) after one SCAN

    python3 tools/gmap/make_a53_data.py --frames A.npz --name 040338 [--frames B.npz --name 032940] --out DIR
"""
import argparse
import hashlib
import os
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host import kv_mapper as km  # noqa: E402


def one(path, name, out):
    z = np.load(path)
    ref = km.KvMapper(rule="q13")
    recs = []
    for i in range(len(z["stamp"])):
        data = np.ascontiguousarray(z["data"][i])
        fr = km.GridFrame(float(z["stamp"][i]), float(z["origin"][i, 0]), float(z["origin"][i, 1]), data,
                          tuple(float(v) for v in z["pose"][i]))
        off = ref.lattice_offset(fr.origin_x, fr.origin_y)
        if off is None:
            raise ValueError("frame %d off the lattice" % i)
        ref.integrate(fr)
        recs.append(struct.pack("<hh", off[0], off[1]) + data.view(np.uint8).tobytes())
    fpath = os.path.join(out, "frames_%s.bin" % name)
    with open(fpath, "wb") as f:
        f.write(b"GMAPFRM1" + struct.pack("<I", len(recs)) + b"".join(recs))
    coarse = ref.coarse().tobytes()
    g_merge = hashlib.sha256(ref.logodds.astype("<i2").tobytes() + coarse).hexdigest()
    g_scan = hashlib.sha256(ref.image(km.PGM_TABLE, True).tobytes() + coarse).hexdigest()
    return fpath, len(recs), g_merge, g_scan


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", action="append", required=True)
    ap.add_argument("--name", action="append", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    lines = []
    for p, n in zip(a.frames, a.name):
        fpath, cnt, gm, gs = one(p, n, a.out)
        lines.append("%s %d %s %s %s\n" % (n, cnt, os.path.basename(fpath), gm, gs))
    with open(os.path.join(a.out, "golden.txt"), "w") as f:
        f.write("# name frames file sha256_merge(global||coarse) sha256_scan(image||coarse)  (PS reference "
                "host/kv_mapper.py rule q13)\n")
        f.writelines(lines)
    print("".join(lines), end="")


if __name__ == "__main__":
    main()
