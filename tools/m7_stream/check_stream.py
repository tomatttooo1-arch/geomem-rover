#!/usr/bin/env python3
"""C-model bit-identity gate for the streaming frame top (A1, 2026-10-11; A2 lanes).

Compares hw/m7/pl/hls/m7_frame_stream.cpp (compiled with g++ as a C model,
-ffp-contract=off, through tools/m7_stream/csim_abi.cpp) against the reference
hw/m7/frame_depth.cpp of this checkout ("new" of tools/m7_clip/kernels.build_all)
on every byte of the 32768-byte flags output, the three counts and the status:

  synthetic  positive_control.realistic_frames + exact_frames (A0's 3050 frames,
             same seed), plus extra frames: empty (0,0) image, a smaller grid,
             range-gate frames (Q8 beyond +-2^24), and error cases status 1/2/3
             (flags must stay untouched, counts 0)
  rec30      all frames of --rec30 (prepared layout, unit from meta)
  tum        --tum-frames frames of --tum, indices round(linspace(0, N-1, k))

Positive controls: six injected faults of the stream model (NOCLIP, TRUNC,
CLIPHIT, TIE, NEXT, MERGE) must each differ from the reference on >= 1 frame; the report
lists on which dataset each is caught. Also reports the C-model walk-loop
iteration count per frame (= cycles at II=1; an ESTIMATE, not a measurement).
A2 (ray lanes): --lanes N compiles the model with -DFPGA_M7_LANES=N; for N > 1
two more faults (LANE_SKIP: merge leaves out a lane; LANE_NOCLEAR: a lane's planes
are not cleared) join the list, and LANE_NOEND (a lane gets no end-of-frame
record) is checked separately: the C model must block (run in a child process
with a timeout). walk_iters is then the busiest lane's loop count.
Usage: check_stream.py --build <fresh dir> --out report.json --rec30 DIR --tum DIR
         [--tum-frames 200] [--jobs 32] [--quick] [--lanes N]
Exit 0 only if every stream-vs-reference difference is 0 and every fault is caught.
"""
import argparse
import ctypes
import json
import multiprocessing as mp
import os
import subprocess
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.m7_clip.kernels import build_all  # noqa: E402
from tools.m7_clip.positive_control import (PRODUCT, exact_frames, inv_k, pose,  # noqa: E402
                                            realistic_frames)

HLS_INC = "/data1/Xilinx/Vitis_HLS/2022.2/include"
STREAM_SRC = os.path.join(ROOT, "hw", "m7", "pl", "hls", "m7_frame_stream.cpp")
ABI_SRC = os.path.join(ROOT, "tools", "m7_stream", "csim_abi.cpp")
FAULTS = ["NOCLIP", "TRUNC", "CLIPHIT", "TIE", "NEXT", "MERGE"]
LANE_FAULTS = ["LANE_SKIP", "LANE_NOCLEAR"]
MAX_PIXELS = 1280 * 800
CAP = 32 ** 3
WC_ID = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]], dtype=np.float32)


def build_stream(out_dir, lanes=None, faults=FAULTS, extra=()):
    libs = {}
    for name, define in [("stream", None)] + [("fault_" + f.lower(), "FPGA_M7_FAULT_" + f) for f in faults] + list(extra):
        out = os.path.join(out_dir, name + ".so")
        cmd = ["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unknown-pragmas",
               "-Wno-unused-label", "-ffp-contract=off", "-fPIC", "-shared", "-isystem", HLS_INC,
               "-I" + os.path.join(ROOT, "hw", "m7")]
        if lanes is not None:
            cmd += ["-DFPGA_M7_LANES=%d" % lanes]
        if define:
            cmd += ["-D" + define, "-Wno-unused-function"]  # TRUNC leaves floor_q8 unused
        subprocess.run(cmd + [STREAM_SRC, ABI_SRC, "-o", out], check=True)
        libs[name] = out
    return libs


F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")


class Lib:
    """Full-output caller: whole 32768-byte flags (prefilled 0x5A), counts, status."""

    def __init__(self, path):
        self.lib = ctypes.CDLL(path)
        f = self.lib.fpga_m7_depth_frame
        f.argtypes = [F32, F32, F32, F64, ctypes.c_double] + [ctypes.c_uint32] * 5 + [U8, U32, U32]
        f.restype = None
        self.depth = np.zeros(MAX_PIXELS, dtype=np.float32)
        self.instr = {}
        for n in ("walk_iters", "walk_rays", "walk_steps", "shadow_errors"):
            try:
                self.instr[n] = ctypes.c_ulonglong.in_dll(self.lib, "fpga_m7_stream_" + n)
            except ValueError:
                pass
        try:
            self.instr["max_ray_steps"] = ctypes.c_uint.in_dll(self.lib, "fpga_m7_stream_max_ray_steps")
        except ValueError:
            pass
        try:
            self.lanes = ctypes.c_int.in_dll(self.lib, "fpga_m7_stream_lanes").value
            self.lane_iters = (ctypes.c_ulonglong * 16).in_dll(self.lib, "fpga_m7_stream_lane_iters")
            self.lane_rays = (ctypes.c_ulonglong * 16).in_dll(self.lib, "fpga_m7_stream_lane_rays")
        except ValueError:
            self.lanes = None

    def run(self, depth, ik, wc, origin, voxel, dims, wh=None):
        depth = np.asarray(depth, dtype=np.float32)
        h, w = depth.shape if wh is None else wh
        self.depth[:depth.size] = depth.ravel()
        flags = np.full(CAP, 0x5A, dtype=np.uint8)
        counts = np.full(3, 7, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        nx, ny, nz = dims
        self.lib.fpga_m7_depth_frame(self.depth, np.ascontiguousarray(ik, dtype=np.float32).ravel(),
                                    np.ascontiguousarray(wc, dtype=np.float32).ravel(),
                                    np.ascontiguousarray(origin, dtype=np.float64), float(voxel),
                                    w, h, nx, ny, nz, flags, counts, status)
        ins = {k: int(v.value) for k, v in self.instr.items()}
        if self.lanes and status[0] == 0:
            ins["lane_iters"] = [int(self.lane_iters[k]) for k in range(self.lanes)]
            ins["lane_rays"] = [int(self.lane_rays[k]) for k in range(self.lanes)]
        return flags, tuple(int(c) for c in counts), int(status[0]), ins


_W = {}


def _init(libs, rec30, tum, unit_rec, unit_tum, faults=FAULTS):
    _W["_faults"] = list(faults)
    for n, p in libs.items():
        _W[n] = Lib(p)
    _W["_data"] = {}
    for tag, d, unit in (("rec30", rec30, unit_rec), ("tum", tum, unit_tum)):
        if d:
            meta = json.load(open(os.path.join(d, "meta.json")))
            W, H, N = meta["width"], meta["height"], meta["frames"]
            raw = np.memmap(os.path.join(d, meta["depth_file"]), dtype="<u2", mode="r", shape=(N, H, W))
            ik = np.array([float.fromhex(x) for x in meta["inverse_k_f32_hex"]], dtype=np.float32).reshape(3, 3)
            g = meta["grid"]
            _W["_data"][tag] = (raw, ik, unit, tuple(g["origin_m"]), float(g["voxel_m"]), tuple(g["shape_xyz"]))


def _frame_args(task):
    kind = task[0]
    if kind == "syn":
        f = task[2]
        return (f["depth"], f["ik"], f["wc"], f["grid_origin"], f["voxel"], f["dims"]), f.get("wh")
    raw, ik, unit, org, vox, dims = _W["_data"][kind]
    i = task[1]
    z = raw[i].astype(np.float32) * np.float32(unit)
    z[raw[i] == 0] = np.nan
    return (z, ik, WC_ID, org, vox, dims), None


def _work(task):
    args, wh = _frame_args(task)
    ref = _W["new"].run(*args, wh=wh)
    st = _W["stream"].run(*args, wh=wh)
    row = {"set": task[0] if task[0] != "syn" else task[3], "id": task[1],
           "status": ref[2], "counts": ref[1],
           "stream_eq": bool(np.array_equal(ref[0], st[0]) and ref[1] == st[1] and ref[2] == st[2]),
           "flag_bytes_diff": int((ref[0] != st[0]).sum()),
           "pixels": int(np.asarray(args[0]).size) if wh is None else wh[0] * wh[1],
           "instr": st[3]}
    for fl in _W["_faults"]:
        o = _W["fault_" + fl.lower()].run(*args, wh=wh)
        row["fault_" + fl.lower() + "_caught"] = not (np.array_equal(ref[0], o[0]) and ref[1] == o[1] and ref[2] == o[2])
    return row


def extra_frames(rng):
    """Edge cases beyond A0's set: empty image, smaller grid, range gate, error statuses."""
    out = []
    ik = inv_k(50., 50., 39.5, 29.5)
    out.append(dict(name="empty_0x0", depth=np.zeros((1, 1), dtype=np.float32), wh=(0, 0), ik=ik,
                    wc=pose(np.eye(3), (0., 0., 0.)), expect_status=0, **PRODUCT))
    d = rng.uniform(0.11, 9.9, size=(60, 80)).astype(np.float32)
    for dims, vox, org in (((20, 24, 28), 0.125, (-1.2, -1.4, -0.3)), ((1, 1, 1), 0.5, (-0.25, -0.25, -0.1)),
                           ((32, 5, 17), 0.07, (-1.1, -0.2, -0.05))):
        out.append(dict(name="grid_%dx%dx%d" % dims, depth=d, ik=ik, wc=pose(np.eye(3), (0., 0., 0.)),
                        grid_origin=org, voxel=vox, dims=dims, expect_status=0))
    # Range gate: tiny voxel puts far endpoints beyond +-2^24 Q8 (invalid), near ones
    # clipped or inside; mixed so all three classes occur.
    for vox in (2e-5, 5e-5, 1.5e-4):
        dd = rng.uniform(0.11, 9.9, size=(30, 40)).astype(np.float32)
        out.append(dict(name="range_gate_vox%g" % vox, depth=dd, ik=inv_k(25., 25., 19.5, 14.5),
                        wc=pose(np.eye(3), (0., 0., 0.)), grid_origin=(-16 * vox, -16 * vox, -1 * vox),
                        voxel=vox, dims=(32, 32, 32), expect_status=0))
    # Error statuses (flags untouched, counts 0)
    out.append(dict(name="err1_dims0", depth=d, ik=ik, wc=pose(np.eye(3), (0., 0., 0.)),
                    grid_origin=(-0.8, -0.8, -0.05), voxel=0.05, dims=(0, 32, 32), expect_status=1))
    out.append(dict(name="err1_dims33", depth=d, ik=ik, wc=pose(np.eye(3), (0., 0., 0.)),
                    grid_origin=(-0.8, -0.8, -0.05), voxel=0.05, dims=(32, 33, 32), expect_status=1))
    out.append(dict(name="err2_voxneg", depth=d, ik=ik, wc=pose(np.eye(3), (0., 0., 0.)),
                    grid_origin=(-0.8, -0.8, -0.05), voxel=-0.05, dims=(32, 32, 32), expect_status=2))
    bad_ik = ik.copy()
    bad_ik[1, 1] = np.inf
    out.append(dict(name="err2_ikinf", depth=d, ik=bad_ik, wc=pose(np.eye(3), (0., 0., 0.)),
                    grid_origin=(-0.8, -0.8, -0.05), voxel=0.05, dims=(32, 32, 32), expect_status=2))
    out.append(dict(name="err3_origin_out", depth=d, ik=ik, wc=pose(np.eye(3), (5., 0., 0.)),
                    grid_origin=(-0.8, -0.8, -0.05), voxel=0.05, dims=(32, 32, 32), expect_status=3))
    return out


_NOEND_CHILD = r"""
import ctypes, sys
import numpy as np
sys.path.insert(0, sys.argv[2])
from tools.m7_stream.check_stream import Lib
from tools.m7_clip.positive_control import inv_k, pose
L = Lib(sys.argv[1])
d = np.full((30, 40), 2.0, dtype=np.float32)
L.run(d, inv_k(25., 25., 19.5, 14.5), pose(np.eye(3), (0., 0., 0.)), (-0.8, -0.8, -0.05), 0.05, (32, 32, 32))
print("RETURNED")
"""


def noend_blocks(build, lanes, timeout=30):
    """LANE_NOEND: the last lane never sees its end record; the C model must block."""
    os.makedirs(os.path.join(build, "noend"))
    lib = build_stream(os.path.join(build, "noend"), lanes, [], [("fault_lane_noend", "FPGA_M7_FAULT_LANE_NOEND")])
    # control: the same child on the fault-free build must return
    ok = subprocess.run([sys.executable, "-c", _NOEND_CHILD, lib["stream"], ROOT],
                        capture_output=True, text=True, timeout=timeout)
    control = ok.returncode == 0 and "RETURNED" in ok.stdout
    try:
        r = subprocess.run([sys.executable, "-c", _NOEND_CHILD, lib["fault_lane_noend"], ROOT],
                           capture_output=True, text=True, timeout=timeout)
        return {"blocked": False, "control_returned": control, "rc": r.returncode,
                "stdout": r.stdout[-200:], "stderr": r.stderr[-300:]}
    except subprocess.TimeoutExpired:
        return {"blocked": control, "control_returned": control, "timeout_s": timeout}


def pct(a, q):
    return None if len(a) == 0 else float(np.percentile(a, q))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rec30", required=True)
    ap.add_argument("--tum", required=True)
    ap.add_argument("--tum-frames", type=int, default=200)
    ap.add_argument("--seed", type=int, default=20261011)
    ap.add_argument("--random", type=int, default=3000)
    ap.add_argument("--jobs", type=int, default=32)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--lanes", type=int, default=None, help="A2: -DFPGA_M7_LANES=N (default: source default)")
    a = ap.parse_args()
    t0 = time.time()
    os.makedirs(a.build, exist_ok=False)
    ref_libs = build_all(os.path.join(a.build, "ref"))
    libs = {"new": ref_libs["new"]}
    faults = FAULTS + (LANE_FAULTS if (a.lanes or 8) > 1 else [])
    libs.update(build_stream(a.build, a.lanes, faults))
    noend = None
    if (a.lanes or 8) > 1:
        noend = noend_blocks(a.build, a.lanes)
    rng = np.random.default_rng(a.seed)
    syn = realistic_frames(rng) + exact_frames(rng, 300 if a.quick else a.random)
    extra = extra_frames(np.random.default_rng(a.seed + 1))
    tasks = [("syn", f["name"], f, "synthetic") for f in syn] + [("syn", f["name"], f, "extra") for f in extra]
    meta_r = json.load(open(os.path.join(a.rec30, "meta.json")))
    meta_t = json.load(open(os.path.join(a.tum, "meta.json")))
    n_r = meta_r["frames"] if not a.quick else 20
    tasks += [("rec30", i) for i in range(n_r)]
    tum_idx = sorted(set(int(round(x)) for x in np.linspace(0, meta_t["frames"] - 1, a.tum_frames)))
    if a.quick:
        tum_idx = tum_idx[:: max(1, len(tum_idx) // 10)]
    tasks += [("tum", i) for i in tum_idx]
    unit_r = float(meta_r.get("depth_unit_m", 0.001))
    unit_t = float(meta_t.get("depth_unit_m", 0.001))
    # largest jobs first
    order = sorted(range(len(tasks)), key=lambda k: {"tum": 0, "rec30": 1}.get(tasks[k][0], 2))
    with mp.Pool(a.jobs, initializer=_init, initargs=(libs, a.rec30, a.tum, unit_r, unit_t, faults)) as pool:
        res = pool.map(_work, [tasks[k] for k in order], chunksize=1)
    rows = [None] * len(tasks)
    for k, r in zip(order, res):
        rows[k] = r
    expect = {f["name"]: f["expect_status"] for f in extra}
    fail = []
    summary = {"seconds": None, "seed": a.seed, "tum_indices": tum_idx, "unit_rec30": unit_r, "unit_tum": unit_t,
               "rec30_depth_sha256": meta_r.get("depth_sha256"), "tum_depth_sha256": meta_t.get("depth_sha256"),
               "sets": {}}
    for s in ("synthetic", "extra", "rec30", "tum"):
        rs = [r for r in rows if r["set"] == s]
        diff = [r["id"] for r in rs if not r["stream_eq"]]
        shadow = sum(r["instr"].get("shadow_errors", 0) for r in rs)
        info = {"frames": len(rs), "stream_ne_reference": len(diff), "first_diffs": diff[:20],
                "flag_bytes_diff_total": sum(r["flag_bytes_diff"] for r in rs),
                "shadow_cross_errors": shadow,
                "frames_with_clipped": sum(1 for r in rs if r["status"] == 0 and r["counts"][2] > 0),
                "clipped_rays": sum(r["counts"][2] for r in rs if r["status"] == 0),
                "invalid_rays": sum(r["counts"][1] for r in rs if r["status"] == 0)}
        for fl in faults:
            info["fault_%s_caught_frames" % fl.lower()] = sum(r["fault_%s_caught" % fl.lower()] for r in rs)
        if s in ("rec30", "tum"):
            it = np.array([r["instr"]["walk_iters"] for r in rs], dtype=np.float64)
            px = np.array([r["pixels"] for r in rs], dtype=np.float64)
            # ESTIMATE at 100 MHz, II=1: clear 32768 + max(read/convert pixels, walk iters) + write 32768
            est = (2 * 32768 + np.maximum(it, px)) / 1e5
            info["cmodel_walk_iters_per_frame"] = {"median": pct(it, 50), "p95": pct(it, 95), "max": float(it.max())}
            info["cmodel_steps_per_frame_median"] = pct([r["instr"]["walk_steps"] for r in rs], 50)
            info["cmodel_rays_per_frame_median"] = pct([r["instr"]["walk_rays"] for r in rs], 50)
            info["max_ray_steps"] = max(r["instr"]["max_ray_steps"] for r in rs)
            if rs and "lane_iters" in rs[0]["instr"]:
                li = np.array([r["instr"]["lane_iters"] for r in rs], dtype=np.float64)
                lr = np.array([r["instr"]["lane_rays"] for r in rs], dtype=np.float64)
                info["lane_iters_max_over_mean_worst"] = float((li.max(1) / np.maximum(li.mean(1), 1)).max())
                info["lane_rays_spread_worst"] = float((lr.max(1) - lr.min(1)).max())
            info["ESTIMATE_ms_at_100MHz_II1"] = {"median": round(pct(est, 50), 2), "p95": round(pct(est, 95), 2),
                                                 "max": round(float(est.max()), 2)}
        if s == "extra":
            for r in rs:
                if r["status"] != expect[r["id"]]:
                    fail.append("extra %s: reference status %d != expected %d" % (r["id"], r["status"], expect[r["id"]]))
            info["statuses"] = {r["id"]: r["status"] for r in rs}
        if diff:
            fail.append("%s: stream != reference on %d frames" % (s, len(diff)))
        if shadow:
            fail.append("%s: %d cross-product shadow errors" % (s, shadow))
        summary["sets"][s] = info
    summary["lanes"] = a.lanes
    summary["faults"] = faults
    summary["noend_fault"] = noend
    if noend is not None and not noend["blocked"]:
        fail.append("fault LANE_NOEND not caught (C model did not block)")
    for fl in faults:
        caught = {s: summary["sets"][s]["fault_%s_caught_frames" % fl.lower()] for s in summary["sets"]}
        summary["fault_%s_caught_by_set" % fl.lower()] = caught
        if sum(caught.values()) == 0:
            fail.append("fault %s not caught" % fl)
    summary["failures"] = fail
    summary["pass"] = not fail
    summary["seconds"] = round(time.time() - t0, 1)
    summary["libs"] = libs
    json.dump({"summary": summary, "rows": rows}, open(a.out, "w"), indent=1, default=int)
    print(json.dumps({k: v for k, v in summary.items() if k not in ("libs", "tum_indices")}, indent=1))
    return 0 if not fail else 1


if __name__ == "__main__":
    sys.exit(main())
