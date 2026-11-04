#!/usr/bin/env python3
"""A2 multi-lane C model: bit-identity against the clipped reference, and
positive controls.

Reference: hw/m7/frame_depth.cpp of this checkout (unchanged), built by
tools/m7_clip/kernels.build_all ("new"). Comparator: all nx*ny*nz flag bytes
and the three counts must be equal.

Frame sets:
  rec30    all 310 frames (A0's sha-verified prep)
  tum      200 frames, indices round(linspace(0, 1208, 200)) (same as A1)
  control  A0's synthetic out-of-grid frames: 11 realistic multi-pixel frames
           + 3,039 exact single-ray frames (tools/m7_clip/positive_control.py,
           seed 20261011, --random 3000)

Per frame and per walk core (ref = verbatim reference add_ray per lane; pstep
= A1 hardware form) the lanes model runs:
  shuffle  random ray order + uniformly random lane, N in {2,4,8,16}, 2 seeds
  rr N=4 D=2, idle N=8 D=8, interleave N=16 D=2, tile_col N=4 D=8,
  tile_row_mc N=4, lwl_oracle N=2 D=2, fnf N=16 D=1, direct N=8 (assignment
  and per-lane order produced by the cycle model)
Positive controls (pstep core, rr N=4 D=2 unless noted), each must be caught
by the same comparator on >= 1 frame of every multi-pixel set:
  merge_free_over_hit  merge gives FREE priority over HIT
  drop_on_full         dispatcher drops a record when the target FIFO is full
  skip_last_lane       merge ignores lane N-1
  no_clear             lane planes not cleared between frames (frame f runs
                       after frame f-1 in the same process)
Optional --a1-rev REV: compiles A1's C model (hw/m7/pl/hls/m7_frame_stream.cpp
and tools/m7_stream/csim_abi.cpp at REV, read with git show; A1's worktree is
not touched) and checks on the first --a1-frames frames of rec30/tum that its
walk loop iteration count equals this model's single-lane walk work + 1
(sentinel), i.e. that the cost model max(S,1) per ray is A1's.
Usage: bitexact.py --build FRESH --out report.json [--jobs 16] [--quick] [--a1-rev 5bf7115]
Exit 0 only if every bit-identity check holds and every fault is caught.
"""
import argparse
import ctypes
import json
import os
import subprocess
import sys
import time
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.m7_clip.kernels import Kernel, build_all  # noqa: E402
from tools.m7_clip.positive_control import exact_frames, realistic_frames  # noqa: E402
from tools.m7_lanes import lanes as L  # noqa: E402

A1_WORKTREE = "${FPGA_SRC}/runs/m7-frame-stream-20261011-worktree"
HLS_INC = "/data1/Xilinx/Vitis_HLS/2022.2/include"

GOOD = [("shuffle", dict(lanes=n, policy="shuffle", seed=s)) for n in (2, 4, 8, 16) for s in (1, 2)] + [
    ("rr_N4_D2", dict(lanes=4, policy="rr", fifo=2)),
    ("idle_N8_D8", dict(lanes=8, policy="idle", fifo=8)),
    ("interleave_N16_D2", dict(lanes=16, policy="interleave", fifo=2)),
    ("tile_col_N4_D8", dict(lanes=4, policy="tile_col", fifo=8)),
    ("tile_row_mc_N4", dict(lanes=4, policy="tile_row_mc")),
    ("lwl_oracle_N2_D2", dict(lanes=2, policy="lwl_oracle", fifo=2)),
    ("fnf_N16_D1", dict(lanes=16, policy="fnf", fifo=1)),
    ("direct_N8", dict(lanes=8, policy="direct")),
]
FAULTS = ["merge_free_over_hit", "drop_on_full", "skip_last_lane", "no_clear"]

_G = {}


def _init(so, ref_so, setname):
    _G["lm"] = L.Lanes(so)
    _G["ref"] = Kernel(ref_so)
    _G["set"] = setname
    if setname in ("rec30", "tum"):
        _G["ds"] = L.open_dataset(setname)
    else:
        rng = np.random.default_rng(20261011)
        _G["frames"] = realistic_frames(rng) + exact_frames(rng, _G.get("n_random", 3000))


def _args(i):
    if _G["set"] in ("rec30", "tum"):
        ds = _G["ds"]
        return (L.depth_frame(ds, i), ds["ik"], L.WC) + (L.GRID["grid_origin"], L.GRID["voxel"], L.GRID["dims"])
    f = _G["frames"][i]
    return (f["depth"], f["ik"], f["wc"], f["grid_origin"], f["voxel"], f["dims"])


def _check(job):
    i, prev = job
    lm, ref = _G["lm"], _G["ref"]
    args = _args(i)
    rf, rc = ref.run(*args)
    out = {"frame": i, "fail": [], "caught": {}}
    with lm.prepare(*args) as p:
        out["rays"] = p.rays
        out["pixels"] = p.pixels
        out["mismatch_steps"] = p.step_core_mismatch
        for core in ("ref", "pstep"):
            for name, cfg in GOOD:
                c = dict(cfg)
                if name == "shuffle":
                    c["seed"] = (1000003 * i + 7919 * c["seed"] + c["lanes"]) % (1 << 31)
                    name = "shuffle_N%d_s%d" % (cfg["lanes"], cfg["seed"])
                f, cnt, st = p.run(core=core, **c)
                if not (np.array_equal(f, rf) and cnt == rc):
                    out["fail"].append("%s/%s" % (core, name))
                if name.startswith("rr_N4_D2") and core == "pstep":
                    out["rr_work"] = st["work"]
        for fault in FAULTS:
            if fault == "no_clear":
                if prev is None:
                    continue
                pargs = _args(prev)
                with lm.prepare(*pargs) as pp:
                    pp.run(core="pstep", lanes=4, policy="rr", fifo=2)
                f, cnt, _ = p.run(core="pstep", lanes=4, policy="rr", fifo=2, fault="no_clear")
            else:
                f, cnt, st = p.run(core="pstep", lanes=4, policy="rr", fifo=2, fault=fault)
            out["caught"][fault] = bool(not (np.array_equal(f, rf) and cnt == rc))
    out["walk_rays_eq_counts"] = out["rays"] == rc[0] + rc[2]
    return out


def a1_crosscheck(rev, build, frames_per_set):
    src = os.path.join(build, "a1src")
    os.makedirs(os.path.join(src, "hw", "m7", "pl", "hls"), exist_ok=True)
    os.makedirs(os.path.join(src, "tools", "m7_stream"), exist_ok=True)
    for rel in ("hw/m7/pl/hls/m7_frame_stream.cpp", "tools/m7_stream/csim_abi.cpp", "hw/m7/frame_depth.hpp"):
        blob = subprocess.run(["git", "-C", A1_WORKTREE, "show", "%s:%s" % (rev, rel)], check=True,
                              capture_output=True).stdout
        open(os.path.join(src, rel), "wb").write(blob)
    so = os.path.join(build, "a1_stream.so")
    subprocess.run(["g++", "-std=c++11", "-O2", "-Wno-unknown-pragmas", "-Wno-unused-label", "-ffp-contract=off",
                    "-fPIC", "-shared", "-isystem", HLS_INC, "-I" + os.path.join(src, "hw", "m7"),
                    os.path.join(src, "hw/m7/pl/hls/m7_frame_stream.cpp"),
                    os.path.join(src, "tools/m7_stream/csim_abi.cpp"), "-o", so], check=True)
    a1 = Kernel(so)
    iters = ctypes.c_ulonglong.in_dll(a1.lib, "fpga_m7_stream_walk_iters")
    steps = ctypes.c_ulonglong.in_dll(a1.lib, "fpga_m7_stream_walk_steps")
    lm = L.Lanes(L.build(os.path.join(build, "lanes_for_a1")))
    rows = []
    for dsn in ("rec30", "tum"):
        ds = L.open_dataset(dsn)
        for f in L.tum_subset(ds["N"], frames_per_set):
            z = L.depth_frame(ds, f)
            af, ac = a1.run(z, ds["ik"], L.WC, **L.GRID)
            with lm.prepare(z, ds["ik"], L.WC, **L.GRID) as p:
                mf, mc, st = p.run(lanes=1, policy="rr", fifo=2, core="pstep")
            rows.append(dict(dataset=dsn, frame=f, a1_walk_iters=int(iters.value), a1_steps=int(steps.value),
                             model_work_plus_sentinel=st["work"] + 1, model_steps=st["steps"],
                             flags_equal=bool(np.array_equal(af, mf) and ac == mc)))
    ok = all(r["a1_walk_iters"] == r["model_work_plus_sentinel"] and r["a1_steps"] == r["model_steps"]
             and r["flags_equal"] for r in rows)
    return {"a1_rev": rev, "frames": rows, "all_equal": ok}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=16)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--a1-rev")
    ap.add_argument("--a1-frames", type=int, default=5)
    a = ap.parse_args()
    t0 = time.time()
    os.makedirs(a.build, exist_ok=False)
    so = L.build(os.path.join(a.build, "lanes"))
    ref_so = build_all(os.path.join(a.build, "ref"))["new"]
    rep = {"source": "tools/m7_lanes/bitexact.py", "reference": "hw/m7/frame_depth.cpp (this checkout)",
           "good_configs": [n for n, _ in GOOD], "faults": FAULTS, "sets": {}}
    ok = True
    sets = [("rec30", list(range(L.open_dataset("rec30")["N"]))),
            ("tum", L.tum_subset(L.open_dataset("tum")["N"], 200)),
            ("control", None)]
    for name, idx in sets:
        if name == "control":
            rng = np.random.default_rng(20261011)
            n = len(realistic_frames(rng) + exact_frames(rng, 3000))
            idx = list(range(n))
        if a.quick:
            idx = idx[::max(1, len(idx) // 10)]
        jobs = [(i, idx[k - 1] if k > 0 else None) for k, i in enumerate(idx)]
        with Pool(a.jobs, initializer=_init, initargs=(so, ref_so, name)) as pool:
            res = pool.map(_check, jobs, chunksize=1 if name != "control" else 50)
        multi = [r for r in res if r["pixels"] > 1]
        fails = [r for r in res if r["fail"]]
        s = {"frames": len(res), "multi_pixel_frames": len(multi),
             "runs_per_frame": 2 * len(GOOD), "frames_all_identical": len(res) - len(fails),
             "failures": [{"frame": r["frame"], "configs": r["fail"]} for r in fails][:20],
             "pstep_vs_reference_step_count_mismatch_rays": int(sum(r["mismatch_steps"] for r in res)),
             "walked_rays_eq_accepted_plus_clipped": int(sum(r["walk_rays_eq_counts"] for r in res)),
             "faults_caught_frames": {f: int(sum(r["caught"].get(f, False) for r in res)) for f in FAULTS},
             "faults_caught_multi_pixel_frames": {f: int(sum(r["caught"].get(f, False) for r in multi))
                                                  for f in FAULTS},
             "faults_tested_frames": {f: int(sum(f in r["caught"] for r in res)) for f in FAULTS},
             "faults_tested_multi_pixel_frames": {f: int(sum(f in r["caught"] for r in multi)) for f in FAULTS}}
        rep["sets"][name] = s
        ok = ok and not fails and s["pstep_vs_reference_step_count_mismatch_rays"] == 0
        for f in FAULTS:
            if s["faults_tested_multi_pixel_frames"][f] > 0:
                ok = ok and s["faults_caught_multi_pixel_frames"][f] > 0
        print(name, json.dumps({k: v for k, v in s.items() if k != "failures"}), flush=True)
    if a.a1_rev:
        rep["a1_crosscheck"] = a1_crosscheck(a.a1_rev, a.build, a.a1_frames)
        ok = ok and rep["a1_crosscheck"]["all_equal"]
        print("a1_crosscheck all_equal", rep["a1_crosscheck"]["all_equal"])
    rep["elapsed_s"] = round(time.time() - t0, 1)
    rep["pass"] = bool(ok)
    json.dump(rep, open(a.out, "w"), indent=1)
    print("PASS" if ok else "FAIL", rep["elapsed_s"], "s")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
