"""AR2NANO2 (nationals 2026-11-02): byte-exact check of the fast CPU AR renderer
(hw/m7/ar_overlay_fast.cpp) against the golden reference hw/m7/ar_overlay.cpp (unchanged).

Per case: status equal and the WHOLE fb buffer equal (width*height pixels + 16 sentinel pixels after
them; for status 1 the whole buffer must stay sentinel), reference built through ar_ref_adapter.cpp
(Xilinx headers), fast through ar_fast_adapter.cpp, both x86 g++ -O2. The fast build is also run with
4 threads (FPGA_AR_FAST_THREADS, env AR_FAST_THREADS=4).
Cases: the 71 fixed cases of nano_ar_check.py (check_golden synthetic, multibox, car layout) and
--random N seeded random cases (default 256, seed 20261102): canvas sizes incl. the car layout,
0..32 boxes on edges / overlapping / 1-px / duplicates, garbage beyond count, label strips of 0..4
printable or unsupported characters, frame ids of 1..10 digits, slice bounds beyond +-9999, mini map
clipped or off-canvas, every mode, bad descriptors / controls / dimensions (a few percent). For
--golden-random K the first K random cases are also compared with the numpy golden (golden.py).
Positive controls (each must report a difference):
  input:  car_live_32 with one box's verdict 2->3 in the fast input only; permit_i flipped likewise;
  source: text-patched mutants of ar_overlay_fast.cpp (box colour, permit polarity, line right edge,
          labels below lines, ring, LUT segment, checkerboard phase), each must differ on some case.
Bench: --bench N times the car_live_32 frame in-process (ctypes call, fb buffer prefilled) for both.

  PYTHONHASHSEED=0 PYTHONPATH=. python3 tools/ar_display/fast_ar_check.py --scratch <dir> [--random 256]
Exit 0 only when every case matches and every control differs.
"""
import argparse
import ctypes
import hashlib
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
import golden  # noqa: E402
import check_golden as cg  # noqa: E402
import nano_ar_check as nac  # noqa: E402

R = {n: i for i, n in enumerate(golden.REG_NAMES)}
FAST_CXX = ["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-pthread", "-DFPGA_AR_FAST_THREADS"]

FAST_MUTANTS = {
    "box_colour": ("v == 2 ? 0xFF00FFu", "v == 2 ? 0xFF00FEu"),
    "permit_polarity": ("permit_i != 0", "permit_i == 0"),
    "line_right_edge": ("X1 >= 1 && X1 - 1 > X0 ? X1 - 1 : X0", "X1 > X0 ? X1 : X0"),
    "top_line_over_labels": ("    // mini map\n", "    for (int i = static_cast<int>(f.n) - 1; i >= 0; --i) {\n"
                           "        const u32 X0 = f.bx0[i], X1 = f.bx1[i], Y0 = f.by0[i];\n"
                           "        if (y == Y0 && X1 < W) for (u32 x = X0; x <= X1; ++x) row[x] = f.bcol[i];\n"
                           "    }\n    // mini map\n"),
    "ring_missing_right": ("if (f.mx + 128 >= 0 && f.mx + 128 < iW) row[f.mx + 128] = 0xFFFFFF;", ""),
    "lut_segment": ("kLut[z >> 5]", "kLut[(z + 1) >> 5]"),
    "checker_phase": ("(((x >> 3) + yb) & 1) ? 0xA000A0u", "(((x >> 3) + yb + 1) & 1) ? 0xA000A0u"),
}


def build_fast(scratch, name, patch=None):
    d = os.path.join(scratch, "fast_" + name)
    os.makedirs(d, exist_ok=True)
    src = open(os.path.join(ROOT, "hw/m7/ar_overlay_fast.cpp")).read()
    if patch is not None:
        old, new = patch
        if src.count(old) != 1:
            raise RuntimeError("fast mutant %s: anchor hits %d times" % (name, src.count(old)))
        src = src.replace(old, new)
    for f in ("ar_overlay_fast.hpp", "ar_overlay_lut.inc"):
        with open(os.path.join(d, f), "w") as fh:
            fh.write(open(os.path.join(ROOT, "hw/m7", f)).read())
    with open(os.path.join(d, "ar_overlay_fast.cpp"), "w") as fh:
        fh.write(src)
    so = os.path.join(d, "ar_fast.so")
    subprocess.check_call(FAST_CXX + ["-fPIC", "-shared", "-I", d, os.path.join(HERE, "ar_fast_adapter.cpp"),
                                      os.path.join(d, "ar_overlay_fast.cpp"), "-o", so])
    lib = ctypes.CDLL(so)
    P = ctypes.POINTER
    lib.ar_fast_run.restype = ctypes.c_uint32
    lib.ar_fast_run.argtypes = [P(ctypes.c_uint16), ctypes.c_uint32, P(ctypes.c_uint8), P(ctypes.c_uint32),
                                P(ctypes.c_uint32), ctypes.c_uint32, P(ctypes.c_uint32)]
    return lib


def run_fast(lib, case):
    depth = np.ascontiguousarray(case["depth"], np.uint16)
    grid = np.ascontiguousarray(case["grid"], np.uint8)
    desc = np.ascontiguousarray(case["desc"], np.uint32)
    regs = np.ascontiguousarray(np.array(case["regs"], np.int64) & 0xFFFFFFFF, np.uint32)
    W, H = int(regs[0]), int(regs[1])
    cap = (W * H if 0 < W * H <= 1280 * 800 else 4096) + 16
    fb = np.zeros(cap, np.uint32)
    P = ctypes.POINTER
    st = lib.ar_fast_run(depth.ctypes.data_as(P(ctypes.c_uint16)), depth.size,
                         grid.ctypes.data_as(P(ctypes.c_uint8)), desc.ctypes.data_as(P(ctypes.c_uint32)),
                         fb.ctypes.data_as(P(ctypes.c_uint32)), cap, regs.ctypes.data_as(P(ctypes.c_uint32)))
    return int(st), fb


# ------------------------------------------------------------------ random cases
LAB_CHARS = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ:.?>/-+= abz#\x00\xff"


def random_case(rng, k):
    r = rng
    kind = r.random()
    if kind < 0.25:                                   # car layout
        W, H, Wd, Hd = 1024, 600, 640, 360
        dx, dy = 192, 120
    else:
        W = int(r.integers(2, 321)) * 4
        H = int(r.integers(8, 801))
        Wd = int(r.integers(1, W // 8 + 1)) * 8
        Hd = int(r.integers(1, H + 1))
        dx = int(r.integers(0, W - Wd + 1))
        dy = int(r.integers(0, H - Hd + 1))
    regs = dict(width=W, height=H, depth_width=Wd, depth_height=Hd, depth_dx=dx, depth_dy=dy,
                frame_valid=1, pose_valid=1, age_ms=int(r.choice([0, 5, 99, 143, 999, 1000, 1001, 123456])),
                age_valid=1, max_age_ms=int(r.choice([0, 100, 1000, 100000])), replay=int(r.random() < 0.2),
                object_evidence=int(r.integers(0, 4)),
                frame_id=int(r.choice([0, 7, 42, 987654, 4294967295, int(r.integers(0, 2**32))])),
                permit_present=int(r.random() < 0.8), permit_i=int(r.random() < 0.5),
                grid_valid=int(r.random() < 0.8), slice_y=int(r.integers(0, 32)),
                slice_lo_mm=int(r.choice([-350, 0, -9999, 9999, -10000, 10000, -2**31, int(r.integers(-20000, 20000))])),
                slice_hi_mm=int(r.choice([-300, 50, 9999, 10000, 2**31 - 1, int(r.integers(-20000, 20000))])),
                cam_cell_x=int(r.choice([0, 16, 31, 32, int(r.integers(0, 40))])),
                cam_cell_z=int(r.choice([0, 1, 31, int(r.integers(0, 40))])),
                mini_x=int(r.choice([W - 136, 0, 1, W - 1, W - 60, int(r.integers(0, W + 200)), 5000])),
                mini_y=int(r.choice([H - 136, 0, 5, 12, 40, H - 1, H - 70, int(r.integers(0, H + 200)), 4095])))
    regs["mini_x"] = max(0, regs["mini_x"])
    regs["mini_y"] = max(0, regs["mini_y"])
    # modes: mostly fresh so the boxes are drawn
    m = r.random()
    if m < 0.08:
        regs["frame_valid"] = 0
    elif m < 0.12:
        regs["age_valid"] = 0
        regs["replay"] = 0
    elif m < 0.16:
        regs["pose_valid"] = 0
    elif m < 0.19:
        regs[str(r.choice(["frame_valid", "pose_valid", "age_valid", "replay", "permit_present",
                           "grid_valid"]))] = 2
    elif m < 0.21:
        regs["object_evidence"] = 4
    elif m < 0.23:                                    # status 1: bad dimensions
        regs[str(r.choice(["width", "depth_width", "depth_dx", "height"]))] += int(r.choice([1, 3, 2000]))
    else:
        regs["age_ms"] = min(regs["age_ms"], regs["max_age_ms"])
    # depth: mixture of valid ramps, invalid 0 / >9999, and an all-invalid frame now and then
    n = Wd * Hd
    if r.random() < 0.05:
        depth = r.choice(np.array([0, 50, 99, 10000, 65535], np.uint16), size=n)
    else:
        depth = r.integers(0, 12000, size=n).astype(np.uint16)
        depth[r.random(n) < 0.1] = 0
        depth[int(r.integers(0, n))] = 100
    grid = r.integers(0, 5, size=32768).astype(np.uint8)
    # boxes
    nb = int(r.choice([0, 1, 2, 5, 11, 31, 32, int(r.integers(0, 33))]))
    rows = []
    for i in range(32):
        t = r.random()
        if t < 0.15:                                  # on an edge of the depth image
            x0 = int(r.choice([0, Wd - 1, int(r.integers(0, Wd))]))
            y0 = int(r.choice([0, Hd - 1, int(r.integers(0, Hd))]))
            x1 = int(r.choice([x0, Wd - 1]))
            y1 = int(r.choice([y0, Hd - 1]))
        elif t < 0.25 and rows:                       # duplicate / overlap of an earlier box
            b = rows[int(r.integers(0, len(rows)))]
            x0, y0 = max(0, b[0] - int(r.integers(0, 5))), max(0, b[1] - int(r.integers(0, 5)))
            x1, y1 = min(Wd - 1, b[2] + int(r.integers(0, 9))), min(Hd - 1, b[3] + int(r.integers(0, 9)))
        elif t < 0.35:                                # 1..3 px
            x0, y0 = int(r.integers(0, Wd)), int(r.integers(0, Hd))
            x1, y1 = min(Wd - 1, x0 + int(r.integers(0, 3))), min(Hd - 1, y0 + int(r.integers(0, 3)))
        else:
            x0, y0 = int(r.integers(0, Wd)), int(r.integers(0, Hd))
            x1 = min(Wd - 1, x0 + int(r.integers(0, 200)))
            y1 = min(Hd - 1, y0 + int(r.integers(0, 150)))
        ln = int(r.integers(0, 5))
        lb = bytes(LAB_CHARS[int(r.integers(0, len(LAB_CHARS)))] for _ in range(ln)) + b"\x00" * (4 - ln)
        rows.append([x0, y0, x1, y1, int(r.choice([0, 2, 3])), sum(lb[q] << (8 * q) for q in range(4))])
    desc = cg.make_desc(rows[:nb])
    for i in range(nb, 32):                           # garbage beyond count (must be ignored)
        if r.random() < 0.5:
            desc[2 + 6 * i: 8 + 6 * i] = r.integers(0, 2**32, size=6, dtype=np.uint64).astype(np.uint32)
    b = r.random()
    if b < 0.03:
        desc[0] ^= 1
    elif b < 0.06:
        desc[1] = 33
    elif b < 0.09 and nb:
        desc[4 + 6 * int(r.integers(0, nb))] = Wd    # x1 out of the depth image
    elif b < 0.11 and nb:
        desc[6 + 6 * int(r.integers(0, nb))] = 1     # bad verdict
    return dict(name="rnd_%03d" % k, regs=cg.regs_of(regs), depth=depth, grid=grid, desc=desc)


REF_HASH = {}


def compare(ref, fast, case):
    st_r, fb_r = cg.run_c(ref, case)
    REF_HASH[case["name"]] = (st_r, hashlib.sha256(fb_r.tobytes()).hexdigest())
    st_f, fb_f = run_fast(fast, case)
    nd = int((fb_r != fb_f).sum())
    return dict(case=case["name"], status_ref=st_r, status_fast=st_f, ndiff=nd, equal=st_r == st_f and nd == 0)


def bench(lib_fn, case, n):
    ts = []
    for _ in range(n):
        t0 = time.perf_counter()
        lib_fn(case)
        ts.append((time.perf_counter() - t0) * 1e3)
    return float(np.median(ts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scratch", required=True)
    ap.add_argument("--random", type=int, default=256)
    ap.add_argument("--seed", type=int, default=20261102)
    ap.add_argument("--golden-random", type=int, default=40)
    ap.add_argument("--bench", type=int, default=0)
    a = ap.parse_args()
    os.makedirs(a.scratch, exist_ok=True)
    ref = cg.build(a.scratch, "clean")
    fast = build_fast(a.scratch, "clean")
    fixed = nac.all_cases()
    rng = np.random.default_rng(a.seed)
    rnd = [random_case(rng, k) for k in range(a.random)]
    results = []
    for threads in ("1", "4"):
        os.environ["AR_FAST_THREADS"] = threads
        for c in fixed + rnd:
            r = compare(ref, fast, c)
            r["threads"] = int(threads)
            results.append(r)
    os.environ["AR_FAST_THREADS"] = "1"
    # random cases vs the independent numpy golden as well (status != 1 only)
    gold = []
    for c in rnd[:a.golden_random]:
        st_g, fb_g, _ = golden.render(c["depth"], c["grid"], c["desc"], c["regs"])
        st_f, fb_f = run_fast(fast, c)
        if st_g == 1:
            ok = st_f == 1 and bool(np.all(fb_f == cg.SENT))
        else:
            w, h = c["regs"][0], c["regs"][1]
            ok = st_f == st_g and int((fb_f[:w * h].reshape(h, w) != fb_g).sum()) == 0
        gold.append(dict(case=c["name"], status_golden=st_g, equal=bool(ok)))
    # coverage facts of the random set
    modes = {}
    for c in rnd:
        st, _ = cg.run_c(ref, c)
        key = "status%d" % st
        modes[key] = modes.get(key, 0) + 1
    nboxes = [int(c["desc"][1]) for c in rnd]
    # input positive controls
    car = [c for c in fixed if c["name"] == "car_live_32"][0]
    ctrls = []
    desc = np.array(car["desc"], np.uint32).copy()
    k = next(i for i, rr in enumerate(car["rows"]) if rr[4] == 2)
    desc[6 + 6 * k] = 3
    st_r, fb_r = cg.run_c(ref, car)
    st_f, fb_f = run_fast(fast, dict(car, desc=desc))
    ctrls.append(dict(control="box %d verdict 2->3 in the fast input only" % k, ndiff=int((fb_r != fb_f).sum())))
    regs = list(car["regs"])
    regs[R["permit_i"]] ^= 1
    st_f, fb_f = run_fast(fast, dict(car, regs=regs))
    ctrls.append(dict(control="permit_i flipped in the fast input only", ndiff=int((fb_r != fb_f).sum())))
    # source mutants of the fast renderer
    for name, patch in FAST_MUTANTS.items():
        lib = build_fast(a.scratch, "mut_" + name, patch)
        hit = 0
        first = None
        for c in fixed + rnd:
            st_m, fb_m = run_fast(lib, c)
            if (st_m, hashlib.sha256(fb_m.tobytes()).hexdigest()) != REF_HASH[c["name"]]:
                hit += 1
                first = first or c["name"]
        ctrls.append(dict(control="source mutant " + name, cases_differing=hit, first=first))
    ctrl_ok = all((c.get("ndiff", 0) > 0) or (c.get("cases_differing", 0) > 0) for c in ctrls)
    benches = {}
    if a.bench:
        benches["car_live_32_ref_ms"] = bench(lambda c: cg.run_c(ref, c), car, max(3, a.bench // 10))
        for th in ("1", "2", "4"):
            os.environ["AR_FAST_THREADS"] = th
            benches["car_live_32_fast_t%s_ms" % th] = bench(lambda c: run_fast(fast, c), car, a.bench)
        os.environ["AR_FAST_THREADS"] = "1"
    ok = all(r["equal"] for r in results) and all(g["equal"] for g in gold) and ctrl_ok
    summary = {"pass": ok, "fixed_cases": len(fixed), "random_cases": len(rnd), "seed": a.seed,
               "comparisons": len(results), "failed": [r["case"] + "/t%d" % r["threads"] for r in results
                                                       if not r["equal"]],
               "golden_random": len(gold), "golden_failed": [g["case"] for g in gold if not g["equal"]],
               "random_status_hist": modes,
               "random_box_counts": {"min": min(nboxes) if nboxes else 0, "max": max(nboxes) if nboxes else 0,
                                     "n32": sum(1 for v in nboxes if v == 32), "n0": sum(1 for v in nboxes if v == 0)},
               "controls": ctrls, "controls_ok": ctrl_ok, "bench_inprocess_ms": benches}
    with open(os.path.join(a.scratch, "fast_ar_check.json"), "w") as fp:
        json.dump({"summary": summary, "results": results, "golden": gold}, fp, indent=1)
    print(json.dumps(summary, indent=1))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
