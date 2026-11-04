"""ctypes wrapper and data loaders for tools/m7_lanes/lanes_model.cpp (A2 design).

Every cycle number this module returns is a MODEL number (per-cycle simulation
of the dispatch structure in lanes_model.cpp), not a measurement.
"""
import ctypes
import json
import os
import subprocess

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "tools", "m7_lanes", "lanes_model.cpp")
FLAGS = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off", "-fPIC", "-shared"]
MAX_PIXELS = 1280 * 800
CAP = 32 ** 3

POLICY = {"rr": 0, "fnf": 1, "idle": 2, "lwl_oracle": 3, "tile_col": 4, "interleave": 5,
          "tile_row_mc": 6, "shuffle": 7, "direct": 8}
FAULT = {"none": 0, "merge_free_over_hit": 1, "drop_on_full": 2, "skip_last_lane": 3, "no_clear": 4}
CORE = {"ref": 0, "pstep": 1}
STAT = {"walk_cycles": 0, "conv_done": 1, "conv_stall": 2, "disp_stall": 3, "work": 4, "steps": 5,
        "rays": 6, "max_ray_cost": 7, "lower_bound": 8, "dropped": 9, "max_fifo_occ": 10,
        "max_sray_occ": 11, "tail": 12, "lanes": 13, "step_core_mismatch": 14, "max_lane_busy": 15,
        "min_lane_busy": 16}

# Product grid and identity pose used for rec30 / TUM (same as tools/m7_clip/regress_and_stats.py).
GRID = dict(grid_origin=(-0.8, -0.8, -0.05), voxel=0.05, dims=(32, 32, 32))
WC = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]], dtype=np.float32)

SCRATCH = "/tmp/claude-1002/-data1-srtp-2025-dev1/21fd542a-fedd-4fcc-940f-2a5a8a8205f9/scratchpad"
DATASETS = {
    "rec30": os.environ.get("M7L_REC30_PREP", os.path.join(SCRATCH, "a0_clip", "prep_rec30")),
    "tum": "${FPGA_SHARED}/data/20261010-dev1-tum-pioneer/prepared",
}

F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
I32 = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
U64 = np.ctypeslib.ndpointer(dtype=np.uint64, flags="C_CONTIGUOUS")


def build(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "lanes_model.so")
    subprocess.run(["g++"] + FLAGS + [SRC, "-o", out], check=True, capture_output=True, text=True)
    return out


class Lanes:
    def __init__(self, path):
        self.lib = ctypes.CDLL(path)
        L = self.lib
        L.m7l_prepare.argtypes = [F32, F32, F32, F64, ctypes.c_double] + [ctypes.c_uint32] * 5 + [U32, U32]
        L.m7l_prepare.restype = ctypes.c_void_p
        L.m7l_free.argtypes = [ctypes.c_void_p]
        L.m7l_num_rays.argtypes = [ctypes.c_void_p]
        L.m7l_num_rays.restype = ctypes.c_uint32
        L.m7l_step_core_mismatch.argtypes = [ctypes.c_void_p]
        L.m7l_step_core_mismatch.restype = ctypes.c_uint64
        L.m7l_profile.argtypes = [ctypes.c_void_p, U8, U8]
        L.m7l_run.argtypes = [ctypes.c_void_p, I32, U8, U32, U64]
        L.m7l_run.restype = ctypes.c_int
        self.depth = np.zeros(MAX_PIXELS, dtype=np.float32)

    def prepare(self, depth, inverse_k, world_camera, grid_origin, voxel, dims):
        depth = np.asarray(depth, dtype=np.float32)
        h, w = depth.shape
        self.depth[:depth.size] = depth.ravel()
        counts = np.zeros(3, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        nx, ny, nz = dims
        hd = self.lib.m7l_prepare(self.depth, np.ascontiguousarray(inverse_k, dtype=np.float32).ravel(),
                                  np.ascontiguousarray(world_camera, dtype=np.float32).ravel(),
                                  np.ascontiguousarray(grid_origin, dtype=np.float64), float(voxel),
                                  w, h, nx, ny, nz, counts, status)
        return Prepared(self, hd, int(status[0]), tuple(int(c) for c in counts), w * h, dims)


class Prepared:
    def __init__(self, lanes, handle, status, counts, pixels, dims):
        self.L, self.h, self.status, self.counts, self.pixels, self.dims = lanes, handle, status, counts, pixels, dims

    def close(self):
        if self.h:
            self.L.lib.m7l_free(self.h)
            self.h = None

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    @property
    def rays(self):
        return int(self.L.lib.m7l_num_rays(self.h))

    @property
    def step_core_mismatch(self):
        return int(self.L.lib.m7l_step_core_mismatch(self.h))

    def profile(self):
        steps = np.zeros(self.pixels, dtype=np.uint8)
        cls = np.zeros(self.pixels, dtype=np.uint8)
        self.L.lib.m7l_profile(self.h, steps, cls)
        return steps, cls

    def run(self, lanes=4, policy="rr", fifo=4, sray=512, conv_ii=1, step_cycles=1, ray_overhead=0,
            core="pstep", fault="none", seed=0, functional=True, timing=True):
        cfg = np.zeros(16, dtype=np.int32)
        cfg[:12] = [lanes, POLICY[policy], fifo, sray, conv_ii, step_cycles, ray_overhead, CORE[core],
                    FAULT[fault], seed, int(functional), int(timing)]
        flags = np.full(CAP, 99, dtype=np.uint8)
        counts = np.full(3, 99, dtype=np.uint32)
        st = np.zeros(128, dtype=np.uint64)
        rc = self.L.lib.m7l_run(self.h, cfg, flags, counts, st)
        if rc != 0:
            raise RuntimeError("m7l_run rc=%d" % rc)
        stats = {k: int(st[i]) for k, i in STAT.items()}
        n = int(stats["lanes"])
        stats["lane_busy"] = [int(x) for x in st[32:32 + n]]
        stats["lane_rays"] = [int(x) for x in st[64:64 + n]]
        nx, ny, nz = self.dims
        return flags[:nx * ny * nz].copy(), tuple(int(c) for c in counts), stats


def open_dataset(name):
    prep = DATASETS[name]
    meta = json.load(open(os.path.join(prep, "meta.json")))
    W, H, N = meta["width"], meta["height"], meta["frames"]
    unit = float(meta.get("depth_unit_m", 0.001))
    ik = np.array([float.fromhex(h) for h in meta["inverse_k_f32_hex"]], dtype=np.float32).reshape(3, 3)
    raw = np.memmap(os.path.join(prep, meta["depth_file"]), dtype="<u2", mode="r", shape=(N, H, W))
    return dict(name=name, prep=prep, meta=meta, W=W, H=H, N=N, unit=unit, ik=ik, raw=raw)


def depth_frame(ds, f):
    """Same conversion as tools/m7_clip/regress_and_stats.py: raw*unit in float32, 0 -> NaN."""
    raw = ds["raw"][f]
    z = raw.astype(np.float32) * np.float32(ds["unit"])
    z[raw == 0] = np.nan
    return z


def tum_subset(n_total, n=200):
    """Uniform subset of n frame indices (same rule as tools/m7_stream subset: linspace, rounded)."""
    return sorted(set(int(round(x)) for x in np.linspace(0, n_total - 1, n)))
