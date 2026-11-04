#!/usr/bin/env python3
"""GMAP3 key comparison 2 vectors (GMVEC002 = GMVEC001 + WINDOW inputs per op; gmap_tb.hpp), written before the run:
  vecA  = GMAP2 vec1 (synthetic MERGE / SCAN / CLEAR, unchanged ops) + CLEAR with byte 13 bit 0 (T "never") +
          30 synthetic WINDOW frames (DESIGN_ADD_PLGRID section 6 vector 1: capsule jump > 0.5 m, 21 points,
          a cell forgotten and marked in the same frame, tall evidence, the exact 300 s boundary and 1 ms past it,
          windows crossing out of the global map and fully outside, seed off / swept off frames, random aux high
          bits and strict = 3), a MERGE of the WINDOW output after every third frame, one bad WINDOW magic and one
          point count 22.
  vecB  = GMAP2 vec2 (040338 first 40 MERGE + SCAN, cleared start) + car1015 first 40 WINDOW frames (CTRLPS4 chain,
          frames_car1015.npz) each followed by a MERGE of its output; init 2 (T "never").
The synthetic WINDOW part is also run through the integer PS reference (window_ref.WindowQ16) and the C model
(ctypes) here: codes, T map and counters must agree on every frame (extends check (1) to the edge cases), and the
coverage counters are printed / asserted.
    python3 make_gmap3_vectors.py OUTDIR GMAP2_VECDIR FRAMES_car1015.npz
"""
import ctypes
import os
import struct
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import window_ref as wr  # noqa: E402

AREA = 1173088
OFF_FRAME, OFF_PARAM, OFF_RESULT, OFF_T, OFF_AUX, OFF_WP = 360000, 366400, 526416, 526432, 1166432, 1172832
CT = (254, 205, 0)          # placeholder coarse table for MERGE params of the WINDOW part (any bytes work)
PGM = (254, 205, 0)


def prm(ix=0, iy=0, ctab=(0, 0xFF, 0x64), itab=PGM, flip=1, magic=0x474D, clear_t=0):
    return struct.pack("<Hhh3B3BBB2x", magic, ix, iy, *ctab, *itab, flip, clear_t)


def read_vec1(path):
    """GMVEC001 -> (init, [(op, param16, frame or None)])"""
    b = open(path, "rb").read()
    assert b[:8] == b"GMVEC001"
    init, n = struct.unpack_from("<II", b, 8)
    o, ops = 16, []
    for _ in range(n):
        op = struct.unpack_from("<I", b, o)[0]
        p = b[o + 4:o + 20]
        hf = struct.unpack_from("<I", b, o + 20)[0]
        o += 24
        fr = None
        if hf:
            fr = b[o:o + 6400]
            o += 6400
        ops.append((op, p, fr, None))
    assert o == len(b)
    return init, ops


def write_vec(path, init, ops):
    with open(path, "wb") as f:
        f.write(b"GMVEC002" + struct.pack("<II", init, len(ops)))
        for op, p, fr, win in ops:
            f.write(struct.pack("<I", op) + p + struct.pack("<I", 0 if fr is None else 1))
            if fr is not None:
                f.write(bytes(fr))
            f.write(struct.pack("<I", 0 if win is None else 1))
            if win is not None:
                aux, wp = win
                assert len(aux) == 6400 and len(wp) == 256
                f.write(bytes(aux) + bytes(wp))


class CModel(object):
    def __init__(self):
        d = tempfile.mkdtemp(prefix="gmap3_vec_")
        so = os.path.join(d, "libgmap.so")
        subprocess.check_call(["gcc", "-O2", "-shared", "-fPIC", "-o", so, os.path.join(HERE, "gmap_model.c")])
        self.lib = ctypes.CDLL(so)
        self.lib.gmap_run.restype = ctypes.c_uint32
        self.lib.gmap_run.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        self.mem = np.zeros(AREA, np.uint8)
        self.mem[OFF_PARAM:OFF_PARAM + 16] = np.frombuffer(prm(clear_t=1), np.uint8)
        assert self.lib.gmap_run(self.mem.ctypes.data, 3) == 0x200

    def window(self, aux, wp):
        self.mem[OFF_AUX:OFF_AUX + 6400] = np.frombuffer(bytes(aux), np.uint8)
        self.mem[OFF_WP:OFF_WP + 256] = np.frombuffer(bytes(wp), np.uint8)
        st = self.lib.gmap_run(self.mem.ctypes.data, 4)
        res = self.mem[OFF_RESULT:OFF_RESULT + 16].view(np.uint32)
        return st, self.mem[OFF_FRAME:OFF_FRAME + 6400].copy(), int(res[1]), int(res[2])


def synth_window(rng):
    """30 synthetic frames: list of (base, body_xy, now_ns, flags, aux u8[6400], note)."""
    V = 0.05
    fr = []
    t_ns = 1_000_000_000_000           # session t0 (ns); now = (t - t0) // 1e6 ms
    t0 = t_ns
    # a straight drive with 0.24 m steps (10 points), then one 0.49 m step (21 points), then a 0.8 m jump
    path = [(1.00 + 0.24 * k, 2.00 + 0.05 * k) for k in range(6)]
    path += [(path[-1][0] + 0.49 * 0.8, path[-1][1] + 0.49 * 0.6)]           # |AB| = 0.49 -> 21 points
    path += [(path[-1][0] + 0.8, path[-1][1])]                                 # jump: A = B
    path += [(path[-1][0] - 0.20, path[-1][1] - 0.15)]                          # back towards swept cells
    times = [k * 100 for k in range(len(path))]                                 # ms
    for k, (xy, tm) in enumerate(zip(path, times)):
        fr.append((xy, tm, 3, "drive"))
    # come back over the start 299.9 s later with tall cells around (evidence), then the exact 300 s boundary
    ev = path[5]
    fr.append(((ev[0] + 0.40, ev[1] + 0.35), 300000 - 50, 3, "evidence"))       # tall near recent cells
    back = path[1]
    fr.append(((back[0] - 0.60, back[1] - 0.40), 300000 + 100, 3, "boundary=300s for frame-1 cells"))
    fr.append(((back[0] - 0.62, back[1] - 0.40), 300000 + 101, 3, "boundary+1ms"))
    fr.append(((back[0] + 0.36, back[1] + 0.25), 300300, 1, "seed only"))
    fr.append(((back[0] + 0.38, back[1] + 0.25), 300400, 2, "swept only"))
    fr.append(((back[0] + 0.40, back[1] + 0.25), 300500, 0, "both off"))
    # windows crossing the global map edges (global cell = base + 200; map 0..399, i.e. odom -10 .. +10 m)
    for k, xy in enumerate([(-9.7, 0.3), (-9.9, -9.8), (9.85, 9.9), (0.2, 9.95), (9.97, -9.9), (-9.95, 5.0)]):
        fr.append((xy, 300600 + 100 * k, 3, "edge"))
    # windows fully outside the map, then back inside
    fr.append(((-14.5, 0.0), 301300, 3, "outside"))
    fr.append(((0.0, 14.5), 301400, 3, "outside"))
    for k in range(7):
        fr.append(((2.0 + 0.1 * k, 2.5 - 0.12 * k), 301500 + 100 * k, 3, "inside"))
    assert len(fr) == 30, len(fr)
    out = []
    for k, (xy, tm, flags, note) in enumerate(fr):
        base = (int(np.floor(xy[0] / V)) - 40, int(np.floor(xy[1] / V)) - 40)
        aux = rng.integers(0, 256, 6400, dtype=np.uint8)                 # strict 0..3, tall random, high bits random
        if note == "evidence":
            aux |= 4                                                       # every cell tall
        out.append((base, xy, t0 + tm * 1_000_000, flags, aux, note))
    return t0, out


def main():
    out, v2dir, carz = sys.argv[1], sys.argv[2], sys.argv[3]
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(20261104 + 3)
    # ---------------------------------------------------------------- vecA
    init, ops = read_vec1(os.path.join(v2dir, "vec1.bin"))
    ops = list(ops)
    ops.append((3, prm(clear_t=1), None, None))
    t0, frames = synth_window(rng)
    ref = wr.WindowQ16(t0)
    cm = CModel()
    cov = dict(frames=0, jump=0, pts21=0, strong=0, strong_and_mark=0, boundary_hit=0, past_boundary=0,
               partly_out=0, fully_out=0, seed_off=0, swept_off=0, strict3=0, cleared=0)
    prev_T = None
    for k, (base, xy, now_ns, flags, aux, note) in enumerate(frames):
        p = ref.params(base, xy, now_ns)
        wp = wr.param_block(p, flags=flags)
        T_before = ref.T.copy()
        code_r, cl_r, ev_r = ref.apply(aux, p, seed=bool(flags & 1), swept=bool(flags & 2))
        st, code_c, cl_c, ev_c = cm.window(aux, wp)
        Tc = cm.mem[OFF_T:OFF_AUX].view(np.int32).reshape(400, 400)
        assert st == 0x200 and np.array_equal(code_r, code_c) and (cl_r, ev_r) == (cl_c, ev_c), (k, note)
        assert np.array_equal(Tc, ref.T.astype(np.int32)), (k, note)
        # coverage
        cov["frames"] += 1
        pts = p["pts"]
        cov["jump"] += int(len(pts) == 2 and pts[0] == pts[1] and k > 0)
        cov["pts21"] += int(len(pts) == 21)
        cov["strong"] += ev_r
        win_T = T_before[max(0, p["iy"]):max(0, p["iy"] + 80), max(0, p["ix"]):max(0, p["ix"] + 80)]
        cov["boundary_hit"] += int((win_T == p["thr"]).sum())
        cov["past_boundary"] += int((win_T == p["thr"] - 1).sum())
        if ev_r and p["ix"] >= 0 and p["iy"] >= 0 and p["ix"] + 80 <= 400 and p["iy"] + 80 <= 400:
            # forgotten and marked in the same frame: recent before, tall, within r_e, and T = now after
            Tb = T_before[p["iy"]:p["iy"] + 80, p["ix"]:p["ix"] + 80]
            Ta = ref.T[p["iy"]:p["iy"] + 80, p["ix"]:p["ix"] + 80]
            dx, dy = wr.axis_dist(p["cx"]), wr.axis_dist(p["cy"])
            near = dy[:, None] ** 2 + dx[None, :] ** 2 <= p["re2"]
            tall = ((aux.reshape(80, 80) >> 2) & 1).astype(bool)
            cov["strong_and_mark"] += int(((Tb >= p["thr"]) & tall & near & (Ta == p["now"])).sum())
        inmap = 0 <= p["ix"] + 79 and p["ix"] < 400 and 0 <= p["iy"] + 79 and p["iy"] < 400
        full = 0 <= p["ix"] and p["ix"] + 80 <= 400 and 0 <= p["iy"] and p["iy"] + 80 <= 400
        cov["partly_out"] += int(inmap and not full)
        cov["fully_out"] += int(not inmap)
        cov["seed_off"] += int(not flags & 1)
        cov["swept_off"] += int(not flags & 2)
        cov["strict3"] += int(((aux & 3) == 3).sum() > 0)
        cov["cleared"] += cl_r
        ops.append((4, prm(), None, (aux.tobytes(), wp)))
        if k % 3 == 2:
            ops.append((1, prm(p["ix"], p["iy"]), None, None))                 # MERGE the WINDOW output
    # bad WINDOW magic and npts 22 (only the result words change)
    p = ref.params(frames[-1][0], frames[-1][1], frames[-1][2] + 1_000_000)
    wp = bytearray(wr.param_block(p))
    bad1 = bytes([0x47, 0x57]) + bytes(wp[2:])
    ops.append((4, prm(), None, (frames[0][4].tobytes(), bad1)))
    bad2 = bytes(wp[:7]) + bytes([22]) + bytes(wp[8:])
    ops.append((4, prm(), None, (frames[0][4].tobytes(), bad2)))
    print("vecA coverage:", cov)
    for key in ("jump", "pts21", "strong", "strong_and_mark", "boundary_hit", "past_boundary", "partly_out",
                "fully_out", "seed_off", "swept_off", "strict3"):
        assert cov[key] > 0, key
    write_vec(os.path.join(out, "vecA.bin"), init, ops)
    # ---------------------------------------------------------------- vecB
    _, ops2 = read_vec1(os.path.join(v2dir, "vec2.bin"))
    ops2 = list(ops2)
    z = np.load(carz)
    ref2 = wr.WindowQ16(int(z["now"][0]))
    for k in range(40):
        p = ref2.params(z["base"][k], z["body"][k], z["now"][k])
        ops2.append((4, prm(), None, (z["aux"][k].tobytes(), wr.param_block(p))))
        ops2.append((1, prm(p["ix"], p["iy"]), None, None))
    write_vec(os.path.join(out, "vecB.bin"), 2, ops2)
    print("vecA ops %d, vecB ops %d" % (len(ops), len(ops2)))


if __name__ == "__main__":
    main()
