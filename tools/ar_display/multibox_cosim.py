"""Multi-box co-simulation gate for the AR overlay core (D, 2026-10-11).

  PYTHONHASHSEED=0 PYTHONPATH=. python3 tools/ar_display/multibox_cosim.py --scratch <dir>
  then, per variant printed at the end:
  bash tools/ar_display/run_multibox_cosim.sh <dir> <variant>

Why: the default HLS testbench co-simulates one 64x48 frame with ONE box, so a
box-index or paint-priority error that exists only in the synthesized RTL
would pass both C simulation and co-simulation. This script

1. builds 4 cases (320x144, depth rectangle 304x96 at (8,40)) sharing one
   descriptor of 11 mutually overlapping boxes: three verdict colours, eleven
   distinct labels, an identical rectangle at a higher index (only the lower
   index may show), boxes on all four depth-image edges, a 2-px-wide box, a
   1-px box, a label clipped by the right canvas edge, labels over other
   boxes' lines and labels. States: LIVE, REPLAY with no frame age (boxes
   drawn, AGE N/A), NOAGE (boxes hidden), STALE;
2. renders them with the independent numpy golden and checks the C reference
   (hw/m7/ar_overlay.cpp via ar_ref_adapter.cpp) matches pixel for pixel;
3. predicts, with the C reference, how many pixels each positive-control
   mutant changes (must be > 0 in LIVE and REPLAY, 0 in NOAGE and STALE,
   where boxes are hidden);
4. writes <scratch>/data/ar_mb_cases.bin (inputs + golden pixels) and one core
   directory per variant: "clean" (unchanged core) and the mutants, whose
   mutation sits under #ifdef __SYNTHESIS__ so C simulation still runs the
   correct code and only the RTL is wrong. Gate: clean co-simulation PASS,
   every mutant C simulation PASS and co-simulation FAIL.
"""
import argparse
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import golden  # noqa: E402
import check_golden as cg  # noqa: E402

# (anchor, replacement) — the replacement is applied only under __SYNTHESIS__.
MUTANTS = {
    # Box index off by one in the "find the hit box, decode once" path: the
    # label colour is fetched for the next box. (Shifting label_idx itself
    # would also shift the glyph column index out of range: UB in C.)
    "hit_index_plus1": (
        "            colour = ink ? 0u : bcol[label_idx];\n",
        "#ifdef __SYNTHESIS__\n"
        "            colour = ink ? 0u : bcol[(label_idx + 1) & 31];\n"
        "#else\n"
        "            colour = ink ? 0u : bcol[label_idx];\n"
        "#endif\n"),
    # Priority reversed: highest index ends up on top.
    "priority_reversed": (
        "        for (int i = 31; i >= 0; --i) {\n",
        "#ifdef __SYNTHESIS__\n"
        "        for (int i = 0; i <= 31; ++i) {\n"
        "#else\n"
        "        for (int i = 31; i >= 0; --i) {\n"
        "#endif\n"),
}

L = cg.lab
ROWS = [  # depth-image coords, closed intervals; Wd=304, Hd=96
    [20, 10, 80, 60, 2, L("BOX0")],
    [40, 20, 120, 70, 3, L("BOX1")],
    [20, 10, 80, 60, 0, L("DUP2")],     # same rect as box 0, other colour
    [0, 0, 303, 95, 0, L("EDGE")],      # touches all four edges
    [250, 30, 303, 95, 3, L("RGT4")],   # right + bottom edge
    [285, 0, 300, 8, 2, L("CLP5")],     # top edge; label clipped at x=320
    [0, 50, 30, 95, 2, L("LFT6")],      # left + bottom edge, label above box
    [60, 55, 140, 90, 0, L("MID7")],
    [100, 5, 101, 40, 3, L("THN8")],    # 2 px wide: all line
    [10, 30, 10, 30, 0, L("PT09")],     # 1 px box
    [45, 15, 47, 90, 2, L("NR10")],
]
BASE = dict(width=320, height=144, depth_width=304, depth_height=96, depth_dx=8,
            depth_dy=40, frame_valid=1, pose_valid=1, age_ms=12, age_valid=1,
            max_age_ms=100, replay=0, object_evidence=1, frame_id=4242,
            permit_present=1, permit_i=1, grid_valid=1, slice_y=7, slice_lo_mm=-50,
            slice_hi_mm=0, cam_cell_x=16, cam_cell_z=1, mini_x=2000, mini_y=2000)
STATES = [
    ("live", dict(), 1),
    ("replay_noage", dict(replay=1, age_valid=0, age_ms=0, object_evidence=3,
                          permit_present=0, grid_valid=0, frame_id=7), 2),
    ("noage", dict(age_valid=0, age_ms=0, permit_i=0), 3),
    ("stale", dict(age_ms=500, object_evidence=2), 4),
]


def cases():
    out = []
    desc = cg.make_desc(ROWS)
    for name, kw, seed in STATES:
        r = dict(BASE, **kw)
        regs = cg.regs_of(r)
        depth = cg.formula_depth(r["depth_width"], r["depth_height"], seed)
        out.append(dict(name=name, regs=regs, desc=desc, depth=depth,
                        grid=cg.formula_grid(seed)))
    return out


def core_dir(scratch, variant, mutation=None):
    d = os.path.join(scratch, "core_" + variant)
    os.makedirs(d, exist_ok=True)
    src = open(os.path.join(ROOT, "hw/m7/ar_overlay.cpp")).read()
    if mutation is not None:
        old, new = mutation
        if src.count(old) != 1:
            raise RuntimeError("%s: anchor hits %d times" % (variant, src.count(old)))
        src = src.replace(old, new)
    for f in ("ar_overlay.hpp", "ar_overlay_lut.inc"):
        with open(os.path.join(d, f), "w") as fh:
            fh.write(open(os.path.join(ROOT, "hw/m7", f)).read())
    with open(os.path.join(d, "ar_overlay.cpp"), "w") as fh:
        fh.write(src)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scratch", required=True)
    a = ap.parse_args()
    os.makedirs(a.scratch, exist_ok=True)
    fails = []
    cs = cases()
    golds = [golden.render(c["depth"], c["grid"], c["desc"], c["regs"]) for c in cs]
    for c, (st, fb, info) in zip(cs, golds):
        n_colours = len(np.unique(fb))
        print("CASE %-13s status=%d mode=%d boxes=%d panel=%s colours=%d"
              % (c["name"], st, info["mode"], info["boxes"], info["panel"], n_colours))
        if st != 0:
            fails.append("status:" + c["name"])
    modes = [g[2]["mode"] for g in golds]
    if modes != [0, 0, 5, 2]:
        fails.append("modes:%s" % modes)
    want = [("LIVE", "AGE:012MS"), ("REPLAY", "AGE N/A"), ("NOAGE", "AGE N/A"),
            ("STALE", "AGE:500MS")]
    for g, (m, age) in zip(golds, want):
        if g[2]["panel"][1:3] != [m, age]:
            fails.append("panel:%s" % g[2]["panel"][1:3])
    fb0 = golds[0][1]
    lib = cg.build(a.scratch, "mb_clean")
    for c, g in zip(cs, golds):
        res, _ = cg.compare(lib, c, g)
        print("REF %-13s %s" % (c["name"], "ok" if res["ok"] else res))
        if not res["ok"]:
            fails.append("ref:" + c["name"])
    # Sensitivity of the cases: removing box 2 from the descriptor must not
    # change LIVE (it is hidden), removing box 0 must.
    for drop, must_change in ((2, False), (0, True)):
        rows = [r for i, r in enumerate(ROWS) if i != drop]
        st, fbx, _ = golden.render(cs[0]["depth"], cs[0]["grid"], cg.make_desc(rows), cs[0]["regs"])
        changed = int((fbx != fb0).sum())
        print("DROP box%d -> %d px change (expect %s)" % (drop, changed, ">0" if must_change else "0"))
        if (changed > 0) != must_change:
            fails.append("drop_box%d" % drop)

    pred = {}
    for m, mut in MUTANTS.items():
        # C prediction: the __SYNTHESIS__ branch applied unconditionally.
        hw_text = mut[1].split("#else\n")[0].replace("#ifdef __SYNTHESIS__\n", "")
        mlib = cg.build(a.scratch, "mb_pred_" + m, (mut[0], hw_text))
        counts = []
        for c, g in zip(cs, golds):
            res, _ = cg.compare(mlib, c, g)
            counts.append(res["ndiff"] if res["status_c"] == res["status_g"] else -1)
        pred[m] = counts
        print("PREDICT %-18s diff_px per case %s" % (m, counts))
        # boxes are drawn only in LIVE/REPLAY (mode 0); NOAGE/STALE hide them
        if min(counts[0], counts[1]) <= 0 or counts[2] != 0 or counts[3] != 0:
            fails.append("predict:" + m)

    data = os.path.join(a.scratch, "data")
    os.makedirs(data, exist_ok=True)
    blob = [struct.pack("<II", 0x424D5241, len(cs))]
    for c, (st, fb, _) in zip(cs, golds):
        dep = np.asarray(c["depth"], np.uint16)
        if dep.size % 2:
            dep = np.append(dep, np.uint16(0))
        blob.append(np.asarray(c["regs"], "<u4").tobytes())
        blob.append(np.asarray(c["desc"], "<u4").tobytes())
        blob.append(struct.pack("<II", st, int(np.asarray(c["depth"]).size)))
        blob.append(dep.astype("<u2").tobytes())
        blob.append(np.asarray(c["grid"], np.uint8).tobytes())
        blob.append(np.asarray(fb, "<u4").ravel().tobytes())
    with open(os.path.join(data, "ar_mb_cases.bin"), "wb") as fh:
        fh.write(b"".join(blob))
    core_dir(a.scratch, "clean")
    for m, mut in MUTANTS.items():
        core_dir(a.scratch, m, mut)
    print("VARIANTS clean %s" % " ".join(MUTANTS))
    print("MB_PREP %s fails=%d %s" % ("PASS" if not fails else "FAIL", len(fails), fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
