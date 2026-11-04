"""Pixel-exact C-reference vs numpy-golden comparator for the AR overlay core,
with five injected-error positive controls (D, 2026-10-11).

  PYTHONHASHSEED=0 PYTHONPATH=. python3 tools/ar_display/check_golden.py \
      --scratch <dir> [--write-vectors]

Builds hw/m7/ar_overlay.cpp (+ text-patched mutants, each patch must hit
exactly once) through tools/ar_display/ar_ref_adapter.cpp with g++, renders
every synthetic case with both, and asserts: same status, identical pixels,
no sentinel left in the image, 16 sentinel pixels after it untouched (the
whole buffer for status 1), two C runs identical, LUT regenerates
byte-identically, every panel string category and glyph covered. Each mutant
must differ from the golden somewhere AND only at its expected locations.
--write-vectors regenerates hw/m7/pl/hls/tb_ar_overlay_vectors.h from the
golden (not from C) and then runs the HLS testbench under g++.
"""
import argparse
import ctypes
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import golden  # noqa: E402

XINC = "/data1/Xilinx/Vitis_HLS/2022.2/include"
CXX = ["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
       "-Wno-unknown-pragmas", "-pthread", "-isystem", XINC]
SENT = 0xDEADBEEF

MUTANTS = {
    "box_right_edge": ("x + 1 >= X1", "x >= X1"),
    "lut_segment": ("kLut[z >> 5]", "kLut[(z + 1) >> 5]"),
    "valid_lower_100": ("z >= 100", "z > 100"),
    "slice_no_flip": ("31 - (static_cast<u32>(ry) >> 2)", "(static_cast<u32>(ry) >> 2)"),
    "permit_polarity": ("permit_i != 0", "permit_i == 0"),
}


def build(scratch, name, patch=None):
    d = os.path.join(scratch, "build_" + name)
    os.makedirs(d, exist_ok=True)
    src = open(os.path.join(ROOT, "hw/m7/ar_overlay.cpp")).read()
    if patch is not None:
        old, new = patch
        if src.count(old) != 1:
            raise RuntimeError("mutant %s: anchor hits %d times" % (name, src.count(old)))
        src = src.replace(old, new)
    for f in ("ar_overlay.hpp", "ar_overlay_lut.inc"):
        with open(os.path.join(d, f), "w") as fh:
            fh.write(open(os.path.join(ROOT, "hw/m7", f)).read())
    with open(os.path.join(d, "ar_overlay.cpp"), "w") as fh:
        fh.write(src)
    so = os.path.join(d, "ar_ref.so")
    subprocess.check_call(CXX + ["-fPIC", "-shared", "-I", d,
                                 os.path.join(ROOT, "tools/ar_display/ar_ref_adapter.cpp"),
                                 os.path.join(d, "ar_overlay.cpp"), "-o", so])
    lib = ctypes.CDLL(so)
    P = ctypes.POINTER
    lib.ar_ref_run.restype = ctypes.c_uint32
    lib.ar_ref_run.argtypes = [P(ctypes.c_uint16), ctypes.c_uint32, P(ctypes.c_uint8),
                               P(ctypes.c_uint32), P(ctypes.c_uint32), ctypes.c_uint32,
                               P(ctypes.c_uint32)]
    return lib


def run_c(lib, case):
    depth = np.ascontiguousarray(case["depth"], np.uint16)
    grid = np.ascontiguousarray(case["grid"], np.uint8)
    desc = np.ascontiguousarray(case["desc"], np.uint32)
    regs = np.ascontiguousarray(np.array(case["regs"], np.int64) & 0xFFFFFFFF, np.uint32)
    W, H = int(regs[0]), int(regs[1])
    cap = (W * H if 0 < W * H <= 1280 * 800 else 4096) + 16
    fb = np.zeros(cap, np.uint32)
    P = ctypes.POINTER
    st = lib.ar_ref_run(depth.ctypes.data_as(P(ctypes.c_uint16)), depth.size,
                        grid.ctypes.data_as(P(ctypes.c_uint8)),
                        desc.ctypes.data_as(P(ctypes.c_uint32)),
                        fb.ctypes.data_as(P(ctypes.c_uint32)), cap,
                        regs.ctypes.data_as(P(ctypes.c_uint32)))
    return int(st), fb


def lab(s):
    b = s.encode("ascii")
    return sum(b[k] << (8 * k) for k in range(4))


def make_desc(rows, magic=golden.MAGIC, count=None):
    d = np.zeros(196, np.uint32)
    d[0] = magic
    d[1] = len(rows) if count is None else count
    for i, r in enumerate(rows):
        d[2 + 6 * i: 8 + 6 * i] = r
    return d


BASE = dict(width=640, height=480, depth_width=640, depth_height=440, depth_dx=0,
            depth_dy=40, frame_valid=1, pose_valid=1, age_ms=12, age_valid=1,
            max_age_ms=100, replay=0, object_evidence=1, frame_id=1234567890,
            permit_present=1, permit_i=1, grid_valid=1, slice_y=16, slice_lo_mm=0,
            slice_hi_mm=50, cam_cell_x=16, cam_cell_z=1, mini_x=504, mini_y=344)


def regs_of(d):
    return [int(d[k]) & 0xFFFFFFFF for k in golden.REG_NAMES]


def formula_depth(Wd, Hd, seed):
    yy, xx = np.mgrid[0:Hd, 0:Wd].astype(np.int64)
    return ((xx * 37 + yy * 101 + seed * 7919 + (xx * yy) % 13) % 11000).astype(np.uint16).ravel()


def formula_grid(seed):
    z, y, x = np.mgrid[0:32, 0:32, 0:32]
    return ((x * 7 + z * 3 + y + seed) % 4).astype(np.uint8).ravel()


def big_depth(Wd, Hd):
    """Base 200..10999 plus planted values: 100 is the unique global minimum
    next to a 99, 0 and 10000 invalid, 9999, and a 1984..2015 run."""
    z = (200 + formula_depth(Wd, Hd, 3).astype(np.int64) % 10800).reshape(Hd, Wd)
    z[300, 350] = 100   # canvas (350, 340) at dy=40: clear of boxes and minimap
    z[300, 351] = 99
    z[301, 350] = 0
    z[301, 351] = 10000
    z[302, 350] = 9999
    z[120, 100:132] = np.arange(1984, 2016)
    return z.astype(np.uint16).ravel()


BIG_ROWS = [
    [10, 20, 120, 100, 2, lab("BOT0")],
    [60, 50, 200, 150, 3, lab("LAP0")],
    [300, 0, 302, 3, 0, lab("ANC1")],
    [500, 300, 639, 439, 0, lab("WXYZ")],
    [220, 200, 260, 230, 2, lab("GQV-")],
    [400, 100, 400, 100, 3, lab("ab+=")],
    [230, 210, 250, 215, 0, lab("/.:>")],
]


def synthetic_cases():
    cases = []
    bd = big_depth(640, 440)
    grid = formula_grid(5)

    def add(name, depth=bd, rows=BIG_ROWS, desc=None, **kw):
        r = dict(BASE)
        r.update(kw)
        cases.append(dict(name=name, regs=regs_of(r), depth=depth, grid=grid,
                          desc=make_desc(rows) if desc is None else desc,
                          rows=rows if desc is None else []))

    add("fresh_permit1")
    add("fresh_permit0", permit_i=0, object_evidence=2)
    add("replay", replay=1, age_valid=0, permit_present=0, object_evidence=0, frame_id=0)
    add("replay_age_big", replay=1, age_ms=5000, object_evidence=3)
    add("noage", age_valid=0)
    add("stale", age_ms=101, max_age_ms=100)
    add("age999", age_ms=999, max_age_ms=1000)
    add("posebad", pose_valid=0)
    add("nodata", depth=np.zeros(640 * 440, np.uint16))
    add("badframe", frame_valid=0)
    for k, v in (("frame_valid", 2), ("object_evidence", 4), ("slice_y", 32),
                 ("permit_present", 2), ("grid_valid", 2), ("replay", 2), ("age_valid", 2)):
        add("ctrl_" + k, **{k: v})
    add("ctrl_and_desc", frame_valid=2, desc=make_desc([], magic=0))
    add("desc_magic", desc=make_desc(BIG_ROWS, magic=0x12345678))
    add("desc_count33", desc=make_desc(BIG_ROWS, count=33))
    add("desc_x1_ge_wd", rows=[[0, 0, 640, 5, 0, lab("BAD0")]])
    add("desc_verdict1", rows=[[0, 0, 5, 5, 1, lab("BAD0")]])
    add("desc_x0_gt_x1", rows=[[6, 0, 5, 5, 0, lab("BAD0")]])
    add("desc_y1_ge_hd", rows=[[0, 0, 5, 440, 0, lab("BAD0")]])
    add("boxes_32", rows=[[20 * i, 60 + 5 * i, 20 * i + 30, 100 + 5 * i, (0, 2, 3)[i % 3],
                           lab("B%03d" % i)] for i in range(32)])
    add("boxes_0", rows=[])
    for name, kw in (("dim_w0", dict(width=0)), ("dim_w_not4", dict(width=642)),
                     ("dim_w_big", dict(width=1284)), ("dim_h_big", dict(height=801)),
                     ("dim_wd_not8", dict(depth_width=636)), ("dim_dx_over", dict(depth_dx=4)),
                     ("dim_hd_over", dict(depth_height=441)), ("dim_wd0", dict(depth_width=0)),
                     ("dim_dx_huge", dict(depth_dx=0xFFFFFFF0))):
        add(name, **kw)
    add("ev2", object_evidence=2)
    add("ev3", object_evidence=3)
    add("grid_off", grid_valid=0)
    add("cam_off_grid", cam_cell_x=40, cam_cell_z=40)
    add("cam_corner", cam_cell_x=0, cam_cell_z=31, slice_y=0)
    add("mini_clip_right_top", mini_x=600, mini_y=5)
    add("mini_far_away", mini_x=5000, mini_y=5000)
    add("mini_under_panel", mini_x=0, mini_y=12)
    add("mini_over_boxes", mini_x=40, mini_y=60)
    add("mini_bottom_clip", mini_x=200, mini_y=420)
    add("slice_neg", slice_lo_mm=-800, slice_hi_mm=-750)
    add("slice_big", slice_lo_mm=-20000, slice_hi_mm=0x7FFFFFFF)
    add("slice_int_min", slice_lo_mm=0x80000000, slice_hi_mm=-1)
    add("fid_max", frame_id=0xFFFFFFFF)
    rec_d = big_depth(640, 360)
    add("rec30_layout", depth=rec_d, height=400, depth_height=360, mini_y=264,
        rows=[[10, 20, 120, 100, 2, lab("BOT0")], [300, 5, 340, 40, 0, lab("LAP0")]])
    add("offset_small", depth=formula_depth(160, 100, 4), width=320, height=240,
        depth_width=160, depth_height=100, depth_dx=80, depth_dy=100, mini_x=10, mini_y=100,
        rows=[[0, 0, 159, 99, 3, lab("ALL0")], [5, 5, 20, 20, 2, lab("SM00")]])
    add("tiny8", depth=formula_depth(8, 8, 1), width=8, height=8, depth_width=8,
        depth_height=8, depth_dy=0, mini_x=2, mini_y=2, rows=[])
    for c in tb_cases():
        cases.append(dict(name="tb_" + c["name"], regs=c["regs"],
                          depth=formula_depth(max(c["regs"][2], 0), max(c["regs"][3], 0), c["seed"])
                          if c["regs"][2] * c["regs"][3] <= 1280 * 800 else np.zeros(8, np.uint16),
                          grid=formula_grid(c["seed"]),
                          desc=make_desc(c["rows"], c["magic"], c["count"]), rows=c["rows"]))
    return cases


def tb_cases():
    def mk(name, seed, rows, magic=golden.MAGIC, count=None, **kw):
        r = dict(BASE)
        r.update(kw)
        return dict(name=name, seed=seed, rows=rows, magic=magic,
                    count=len(rows) if count is None else count, regs=regs_of(r))
    small = dict(width=64, height=48, depth_width=64, depth_height=48, depth_dx=0,
                 depth_dy=0, object_evidence=2, frame_id=1234, slice_y=5,
                 slice_lo_mm=-550, slice_hi_mm=-500, mini_x=40, mini_y=20)
    return [
        mk("cosim64x48", 1, [[4, 42, 30, 46, 3, lab("BOT0")]], **small),
        mk("panel640x96", 2, [[10, 4, 100, 50, 2, lab("BOT0")], [50, 0, 300, 20, 0, lab("ANC7")],
                              [600, 10, 639, 55, 3, lab("LAP1")]],
           width=640, height=96, depth_width=640, depth_height=56, depth_dy=40, age_ms=250,
           max_age_ms=300, object_evidence=3, frame_id=0xFFFFFFFF, permit_i=0, slice_y=31,
           slice_lo_mm=-9999, slice_hi_mm=10000, cam_cell_x=0, cam_cell_z=31, mini_x=504,
           mini_y=60),
        mk("stale320x64", 3, [], width=320, height=64, depth_width=160, depth_height=16,
           depth_dx=80, depth_dy=48, age_ms=500, max_age_ms=100, mini_x=100, mini_y=50),
        dict(mk("ctrl_bad", 1, [], frame_valid=2, **small)),
        mk("bad_dims", 1, [], width=66, height=48, depth_width=64, depth_height=48, depth_dy=0),
        mk("desc_bad", 1, [[0, 0, 5, 5, 0, lab("BAD0")]], magic=0, **small),
    ]


def fnv_case(status, fb):
    h = 2166136261
    data = b"" if fb is None else fb.astype("<u4").tobytes()
    data += int(status).to_bytes(4, "little")
    for byte in bytearray(data):
        h = ((h ^ byte) * 16777619) & 0xFFFFFFFF
    return h


def compare(lib, case, gold):
    st_g, fb_g, info = gold
    st_c, fb_c = run_c(lib, case)
    W, H = case["regs"][0], case["regs"][1]
    res = dict(status_c=st_c, status_g=st_g)
    if st_g == 1:
        res["ok"] = st_c == 1 and bool(np.all(fb_c == SENT))
        res["diff"] = None
        return res, None
    img = fb_c[:W * H].reshape(H, W)
    tail = fb_c[W * H:]
    diff = img != fb_g
    res["ndiff"] = int(diff.sum())
    res["leftover"] = int((img == SENT).sum())
    res["tail_ok"] = bool(np.all(tail == SENT))
    res["ok"] = st_c == st_g and res["ndiff"] == 0 and res["leftover"] == 0 and res["tail_ok"]
    return res, diff


def depth_canvas(case):
    r = dict(zip(golden.REG_NAMES, case["regs"]))
    W, H = r["width"], r["height"]
    z = np.full((H, W), -1, np.int64)
    Wd, Hd, dx, dy = r["depth_width"], r["depth_height"], r["depth_dx"], r["depth_dy"]
    z[dy:dy + Hd, dx:dx + Wd] = np.asarray(case["depth"])[:Wd * Hd].reshape(Hd, Wd)
    return z, r


def expected_region(mname, case):
    z, r = depth_canvas(case)
    H, W = z.shape
    yy, xx = np.mgrid[0:H, 0:W]
    if mname == "box_right_edge":
        m = np.zeros((H, W), bool)
        for (x0, y0, x1, y1, v, l) in case["rows"]:
            X1 = x1 + r["depth_dx"]
            m |= (xx == X1 - 1) & (yy >= y0 + r["depth_dy"]) & (yy <= y1 + r["depth_dy"])
        return m
    if mname == "lut_segment":
        return (z >= 0) & ((z & 31) == 31)
    if mname == "valid_lower_100":
        return (z == 100) | ((yy < 20) & (xx < 156))
    if mname == "slice_no_flip":
        rx, ry = xx - r["mini_x"], yy - r["mini_y"]
        return (rx >= 0) & (rx < 128) & (ry >= 0) & (ry < 128)
    if mname == "permit_polarity":
        return (yy < 20) & (xx >= 468) & (xx < 636)
    raise KeyError(mname)


ALL_PANEL = ["LIVE", "REPLAY", "BADFRAME", "STALE", "POSEBAD", "NODATA", "NOAGE",
             "AGE N/A", "AGE>999MS", "DRV N/A", "DRIVE OK", "STOP", "RISK N/A", "OBJ ?",
             "OBJ HIT", "OBJ FREE?", "OBJ OCCL", "POSE OK", "POSE BAD", "POSE ?", "Z:??.??M"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scratch", required=True)
    ap.add_argument("--write-vectors", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.scratch, exist_ok=True)
    fails = []

    # LUT regenerates byte-identically from the legacy formula.
    gen = os.path.join(a.scratch, "lutgen")
    subprocess.check_call(["g++", "-std=c++11", "-O2", "-I", os.path.join(ROOT, "hw/m7"),
                           os.path.join(ROOT, "hw/m7/ar_overlay_lut_gen.cpp"), "-o", gen])
    regen = subprocess.check_output([gen]).decode()
    if regen != open(os.path.join(ROOT, "hw/m7/ar_overlay_lut.inc")).read():
        fails.append("lut_regen")
    lut_inc = np.array([int(t.strip().rstrip("u"), 16) for t in
                        regen.split("\n", 2)[2].replace("\n", " ").split(",") if t.strip()],
                       np.uint32)
    if not np.array_equal(lut_inc, golden.LUT):
        fails.append("lut_vs_golden_formula")

    lib = build(a.scratch, "ref")
    cases = synthetic_cases()
    golds = [golden.render(c["depth"], c["grid"], c["desc"], c["regs"]) for c in cases]
    seen_panel, n_ok = set(), 0
    for c, g in zip(cases, golds):
        res, _ = compare(lib, c, g)
        st2, fb2 = run_c(lib, c)
        st1, fb1 = run_c(lib, c)
        if not (st1 == st2 and np.array_equal(fb1, fb2)):
            fails.append("nondeterministic:" + c["name"])
        if not res["ok"]:
            fails.append("mismatch:%s %s" % (c["name"], res))
        else:
            n_ok += 1
        seen_panel.update(g[2].get("panel", []))
    seen_text = " ".join(seen_panel)
    for s in ALL_PANEL:
        if s not in seen_panel:
            fails.append("panel_string_not_covered:" + s)
    for ch in set(seen_text + "".join(golden.slice_label(-1, 1)) + "0123456789?FO"):
        if ch != " " and ch not in golden.FONT:
            fails.append("no_glyph:%r" % ch)
    if not all(d in seen_text for d in "0123456789"):
        fails.append("digits_not_all_covered")
    print("REF cases=%d ok=%d" % (len(cases), n_ok))

    for mname, patch in MUTANTS.items():
        mlib = build(a.scratch, mname, patch)
        hit_cases, outside = 0, 0
        for c, g in zip(cases, golds):
            res, diff = compare(mlib, c, g)
            if diff is None:
                if not res["ok"]:
                    outside += 1
                continue
            if res["ndiff"]:
                hit_cases += 1
                outside += int((diff & ~expected_region(mname, c)).sum())
        extra = ""
        if mname == "valid_lower_100":
            c = cases[0]
            _, diff = compare(mlib, c, golds[0])
            z, _r = depth_canvas(c)
            px, pan = int((diff & (z == 100)).sum()), int((diff[:20, :156]).sum())
            extra = " z100_px=%d panel_px=%d" % (px, pan)
            if px == 0 or pan == 0:
                fails.append("mutant_both_effects:" + mname)
        print("MUTANT %-16s caught_in_cases=%d outside_expected_px=%d%s"
              % (mname, hit_cases, outside, extra))
        if hit_cases == 0:
            fails.append("mutant_missed:" + mname)
        if outside:
            fails.append("mutant_outside_region:" + mname)

    if a.write_vectors:
        tb = tb_cases()
        lines = ["// GENERATED by tools/ar_display/check_golden.py --write-vectors from the",
                 "// numpy golden (tools/ar_display/golden.py), not from the C core.",
                 "//   PYTHONHASHSEED=0 PYTHONPATH=. python3 tools/ar_display/check_golden.py "
                 "--scratch <dir> --write-vectors",
                 "#ifndef TB_AR_OVERLAY_VECTORS_H", "#define TB_AR_OVERLAY_VECTORS_H",
                 "#include <cstdint>",
                 "struct ArTbCase { const char* name; std::uint32_t regs[24]; std::uint32_t seed;",
                 "  std::uint32_t magic; std::uint32_t count; std::uint32_t n_rows;",
                 "  std::uint32_t rows[32][6]; std::uint32_t expect_status; std::uint32_t expect_hash; };",
                 "static const ArTbCase kArTbCases[] = {"]
        for c in tb:
            r = c["regs"]
            d = formula_depth(r[2], r[3], c["seed"]) if r[2] * r[3] <= 1280 * 800 else np.zeros(8, np.uint16)
            st, fb, _ = golden.render(d, formula_grid(c["seed"]),
                                      make_desc(c["rows"], c["magic"], c["count"]), r)
            h = fnv_case(st, None if st == 1 else fb)
            rows = ", ".join("{%s}" % ", ".join("%du" % v for v in row) for row in c["rows"]) or "{0}"
            lines.append('  {"%s", {%s}, %du, 0x%08Xu, %du, %du, {%s}, %du, 0x%08Xu},'
                         % (c["name"], ", ".join("%du" % v for v in r), c["seed"], c["magic"],
                            c["count"], len(c["rows"]), rows, st, h))
        lines += ["};", "static const std::uint32_t kArTbCaseCount = %d;" % len(tb), "#endif", ""]
        vp = os.path.join(ROOT, "hw/m7/pl/hls/tb_ar_overlay_vectors.h")
        with open(vp, "w") as fh:
            fh.write("\n".join(lines))
        exe = os.path.join(a.scratch, "tb_ar")
        subprocess.check_call(CXX + ["-I", os.path.join(ROOT, "hw/m7/pl/hls"),
                                     os.path.join(ROOT, "hw/m7/pl/hls/tb_ar_overlay_axi.cpp"),
                                     os.path.join(ROOT, "hw/m7/pl/hls/m7_ar_overlay_axi.cpp"),
                                     os.path.join(ROOT, "hw/m7/ar_overlay.cpp"), "-o", exe])
        out = subprocess.run([exe], env=dict(os.environ, AR_TB_ALL="1"),
                             stdout=subprocess.PIPE, universal_newlines=True)
        print("\n".join(l for l in out.stdout.splitlines() if l.startswith(("[", "AR_TB"))))
        if out.returncode != 0:
            fails.append("hls_tb_gpp")
            print(out.stdout)

    print("CHECK %s fails=%d %s" % ("PASS" if not fails else "FAIL", len(fails), fails[:12]))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
