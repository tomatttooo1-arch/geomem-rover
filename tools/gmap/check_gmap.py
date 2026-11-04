#!/usr/bin/env python3
"""GMAP1 check (1): the PL fixed-point C model (tools/gmap/gmap_model.c) against the PS reference (host/kv_mapper.py,
rule "q13") on recorded GridFrame sequences. Criteria frozen in runs/nationals-prelim-20261102/GMAP0/README.md
section 0 before the first run:
  1 after every frame the C model's global int16 array == kv_mapper.logodds (160,000 cells, every frame)
  2 every frame's dropped count == the reference's dropped_cells increment
  3 after the last frame the coarse map (incremental from MERGE, and rebuilt by SCAN) == kv_mapper.coarse()
  4 the SCAN image (PGM table, y flip) cropped to bbox + 2 == the pixels of kv_mapper.export's .pgm; bbox equal
Start: the map area filled with random bytes, then CLEAR. Positive controls: --hit 6942 (rebuild the model with
another hit constant) and --ix-plus 1 (driver error) must make criterion 1 fail.

    python3 tools/gmap/check_gmap.py --frames A.npz --out DIR [--hit 6942] [--ix-plus 1] [--label NAME]
"""
import argparse
import ctypes
import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host import kv_mapper as km  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
N, W, CN = 400, 80, 200
OFF_GLOBAL, OFF_COARSE, OFF_FRAME, OFF_PARAM, OFF_IMAGE, OFF_RESULT, AREA = (
    0, 320000, 360000, 366400, 366416, 526416, 526432)
MAGIC = 0x474D
OP_MERGE, OP_SCAN, OP_CLEAR = 1, 2, 3


def build(out_dir, hit=None):
    so = os.path.join(out_dir, "libgmap%s.so" % ("" if hit is None else "_hit%d" % hit))
    cmd = ["gcc", "-O2", "-std=c99", "-Wall", "-Wextra", "-shared", "-fPIC", "-o", so,
           os.path.join(HERE, "gmap_model.c")] + ([] if hit is None else ["-DGMAP_HIT=%d" % hit])
    subprocess.check_call(cmd)
    lib = ctypes.CDLL(so)
    lib.gmap_run.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.gmap_run.restype = ctypes.c_uint32
    return lib, " ".join(cmd)


def put_params(mem, ix=0, iy=0, ctab=km.COARSE_TABLE, itab=km.PGM_TABLE, flip=True):
    p = np.zeros(16, np.uint8)
    p[0:2] = np.frombuffer(np.uint16(MAGIC).tobytes(), np.uint8)
    p[2:4] = np.frombuffer(np.int16(ix).tobytes(), np.uint8)
    p[4:6] = np.frombuffer(np.int16(iy).tobytes(), np.uint8)
    p[6:9] = ctab
    p[9:12] = itab
    p[12] = 1 if flip else 0
    mem[OFF_PARAM:OFF_PARAM + 16] = p


def result(mem):
    return np.frombuffer(mem[OFF_RESULT:OFF_RESULT + 16].tobytes(), "<u4").tolist()


def read_pgm(path):
    raw = open(path, "rb").read()
    m = re.match(rb"P5\n(\d+) (\d+)\n255\n", raw)
    w, h = int(m.group(1)), int(m.group(2))
    return np.frombuffer(raw[m.end():], np.uint8).reshape(h, w)


def check(frames_path, out_dir, hit=None, ix_plus=0, label="trip"):
    os.makedirs(out_dir, exist_ok=True)
    lib, cmd = build(out_dir, hit)
    z = np.load(frames_path)
    nfr = len(z["stamp"])
    ref = km.KvMapper(rule="q13")
    mem = np.random.default_rng(20261104).integers(0, 256, AREA, dtype=np.uint8)
    ptr = mem.ctypes.data
    put_params(mem)
    st = lib.gmap_run(ptr, OP_CLEAR)
    clear_ok = (st == 0x100 and np.array_equal(mem[OFF_GLOBAL:OFF_COARSE].view("<i2"), ref.logodds.ravel())
                and bool((mem[OFF_COARSE:OFF_FRAME] == km.COARSE_TABLE[1]).all()))
    gview = mem[OFF_GLOBAL:OFF_COARSE].view("<i2").reshape(N, N)
    c1_bad_frames, c1_first, c1_first_cells, c1_last_cells = 0, None, None, 0
    c2_bad_frames, c2_first = 0, None
    off_lattice = 0
    dropped_total = 0
    for i in range(nfr):
        data = np.ascontiguousarray(z["data"][i])
        fr = km.GridFrame(float(z["stamp"][i]), float(z["origin"][i, 0]), float(z["origin"][i, 1]), data,
                          tuple(float(v) for v in z["pose"][i]))
        off = ref.lattice_offset(fr.origin_x, fr.origin_y)
        if off is None:                                 # never in the product chain; counted, frame skipped
            off_lattice += 1
            continue
        d0 = ref.dropped_cells
        ref.integrate(fr)
        dref = ref.dropped_cells - d0
        put_params(mem, off[0] + ix_plus, off[1])
        mem[OFF_FRAME:OFF_FRAME + W * W] = data.view(np.uint8).ravel()
        st = lib.gmap_run(ptr, OP_MERGE)
        res = result(mem)
        dropped_total += res[1]
        nd = int((gview != ref.logodds).sum())
        if nd:
            c1_bad_frames += 1
            if c1_first is None:
                c1_first, c1_first_cells = i, nd
        c1_last_cells = nd
        if st != 0x100 or res[1] != dref:
            c2_bad_frames += 1
            if c2_first is None:
                c2_first = {"frame": i, "status": st, "pl": res[1], "ref": dref}
    coarse_ref = ref.coarse()
    coarse_inc = mem[OFF_COARSE:OFF_FRAME].reshape(CN, CN).copy()
    put_params(mem, 0, 0, km.COARSE_TABLE, km.PGM_TABLE, True)
    st_scan = lib.gmap_run(ptr, OP_SCAN)
    res = result(mem)
    coarse_scan = mem[OFF_COARSE:OFF_FRAME].reshape(CN, CN).copy()
    img = mem[OFF_IMAGE:OFF_IMAGE + N * N].reshape(N, N).copy()
    c3_inc = int((coarse_inc != coarse_ref).sum())
    c3_scan = int((coarse_scan != coarse_ref).sum())
    # criterion 4: export, then the PS's crop of the PL image
    exp = ref.export(os.path.join(out_dir, "kv"), label, os.path.join(out_dir, "kv", label + ".db"))
    pgm = read_pgm(exp["pgm"])
    box_ref = ref.bbox()
    if st_scan & 2:
        box_pl = None
    else:
        rmin, rmax, cmin, cmax = res[2] & 0xFFFF, res[2] >> 16, res[3] & 0xFFFF, res[3] >> 16
        box_pl = (max(0, rmin - 2), min(N, rmax + 3), max(0, cmin - 2), min(N, cmax + 3))
    r0, r1, c0, c1 = box_pl or (0, N, 0, N)
    crop = img[N - r1:N - r0, c0:c1]                    # image rows are flipped: image row = 399 - global row
    c4_pix = bool(crop.shape == pgm.shape and np.array_equal(crop, pgm))
    c4_box = box_pl == box_ref
    out = {
        "label": label, "frames": os.path.abspath(frames_path), "n_frames": nfr, "build": cmd,
        "hit": km.Q_HIT if hit is None else hit, "ix_plus": ix_plus, "off_lattice_frames": off_lattice,
        "clear_ok": bool(clear_ok),
        "c1": {"pass": c1_bad_frames == 0, "bad_frames": c1_bad_frames, "first_bad_frame": c1_first,
               "first_bad_cells": c1_first_cells, "last_frame_diff_cells": c1_last_cells},
        "c2": {"pass": c2_bad_frames == 0, "bad_frames": c2_bad_frames, "first": c2_first,
               "dropped_total_pl": dropped_total, "dropped_total_ref": ref.dropped_cells},
        "c3": {"pass": c3_inc == 0 and c3_scan == 0, "incremental_diff": c3_inc, "scan_diff": c3_scan},
        "c4": {"pass": c4_pix and c4_box, "pixels_equal": c4_pix, "bbox_pl": box_pl, "bbox_ref": box_ref,
               "pgm_shape": list(pgm.shape), "scan_status": st_scan},
        "ref_status": ref.status(),
        "sha256": {"global": hashlib.sha256(mem[OFF_GLOBAL:OFF_COARSE].tobytes()).hexdigest(),
                   "coarse": hashlib.sha256(coarse_scan.tobytes()).hexdigest(),
                   "image": hashlib.sha256(img.tobytes()).hexdigest()},
    }
    out["pass"] = all(out[k]["pass"] for k in ("c1", "c2", "c3", "c4")) and out["clear_ok"]
    with open(os.path.join(out_dir, "check.json"), "w") as f:
        json.dump(out, f, indent=1, default=str)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--label", default="trip")
    ap.add_argument("--hit", type=int, default=None)
    ap.add_argument("--ix-plus", type=int, default=0)
    a = ap.parse_args(argv)
    r = check(a.frames, a.out, a.hit, a.ix_plus, a.label)
    print(json.dumps({k: r[k] for k in ("label", "n_frames", "hit", "ix_plus", "off_lattice_frames", "clear_ok",
                                        "c1", "c2", "c3", "c4", "pass")}, default=str))


if __name__ == "__main__":
    main()
