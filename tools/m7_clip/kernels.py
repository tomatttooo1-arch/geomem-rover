"""Build and call the M7 frame kernel variants used by the clip checks.

Variants (all g++ -O2 -ffp-contract=off, the repo flags):
  new           hw/m7/frame_depth.cpp of this checkout (clipped semantics)
  old           hw/m7/frame_depth.cpp at --old-rev (09-21 rule: drop rays whose
                endpoint is outside the grid)
  mut_drop      new source, but a clipped ray is counted and NOT walked
                (fault 1 of the spec: "drop instead of clip")
  mut_trunc     new source, but the endpoint cell uses C '/' (truncation toward
                zero) instead of floor (fault 2 of the spec)
Each mutant is produced by an exact textual patch that must match once.
"""
import ctypes
import os
import shutil
import subprocess

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "hw", "m7", "frame_depth.cpp")
HDR = os.path.join(ROOT, "hw", "m7", "frame_depth.hpp")
MAX_PIXELS = 1280 * 800
GRID_CAPACITY = 32 ** 3
FLAGS = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off", "-fPIC", "-shared"]

MUTANTS = {
    "mut_drop": [(
        "        add_ray(origin, end, dims, grid);\n        if (inside)\n            ++accepted;\n        else\n            ++clipped;\n",
        "        if (!inside) {\n            ++clipped;\n            continue;\n        }\n        add_ray(origin, end, dims, grid);\n        ++accepted;\n")],
    "mut_trunc": [(
        "        target[axis] = floor_q8(to);\n",
        "        target[axis] = to / FPGA_M7_FRAME_Q8;\n")],
}


def _compile(src_dir, name, out_dir):
    out = os.path.join(out_dir, name + ".so")
    extra = ["-Wno-unused-function"] if name.startswith("mut_") else []  # mut_trunc leaves floor_q8 unused
    subprocess.run(["g++"] + FLAGS + extra + [os.path.join(src_dir, "frame_depth.cpp"), "-o", out],
                   check=True, capture_output=True, text=True)
    return out


def build_all(out_dir, old_rev="361b145"):
    os.makedirs(out_dir, exist_ok=True)
    libs = {}
    src = open(SRC).read()
    for name, text in [("new", src)] + [(m, None) for m in MUTANTS]:
        d = os.path.join(out_dir, name)
        os.makedirs(d, exist_ok=True)
        shutil.copy(HDR, os.path.join(d, "frame_depth.hpp"))
        if text is None:
            text = src
            for old, new in MUTANTS[name]:
                if text.count(old) != 1:
                    raise RuntimeError("%s: patch anchor matched %d times" % (name, text.count(old)))
                text = text.replace(old, new)
        open(os.path.join(d, "frame_depth.cpp"), "w").write(text)
        libs[name] = _compile(d, name, out_dir)
    d = os.path.join(out_dir, "old")
    os.makedirs(d, exist_ok=True)
    for f in ("frame_depth.cpp", "frame_depth.hpp"):
        blob = subprocess.run(["git", "-C", ROOT, "show", "%s:hw/m7/%s" % (old_rev, f)],
                              check=True, capture_output=True).stdout
        open(os.path.join(d, f), "wb").write(blob)
    libs["old"] = _compile(d, "old", out_dir)
    return libs


F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")


class Kernel:
    def __init__(self, path):
        self.path = path
        self.lib = ctypes.CDLL(path)
        f = self.lib.fpga_m7_depth_frame
        f.argtypes = [F32, F32, F32, F64, ctypes.c_double] + [ctypes.c_uint32] * 5 + [U8, U32, U32]
        f.restype = None
        self.depth = np.zeros(MAX_PIXELS, dtype=np.float32)

    def run(self, depth, inverse_k, world_camera, grid_origin, voxel, dims):
        depth = np.asarray(depth, dtype=np.float32)
        h, w = depth.shape
        self.depth[:depth.size] = depth.ravel()
        flags = np.full(GRID_CAPACITY, 99, dtype=np.uint8)
        counts = np.full(3, 99, dtype=np.uint32)
        status = np.full(1, 99, dtype=np.uint32)
        nx, ny, nz = dims
        self.lib.fpga_m7_depth_frame(self.depth, np.ascontiguousarray(inverse_k, dtype=np.float32).ravel(),
                                    np.ascontiguousarray(world_camera, dtype=np.float32).ravel(),
                                    np.ascontiguousarray(grid_origin, dtype=np.float64), float(voxel),
                                    w, h, nx, ny, nz, flags, counts, status)
        if status[0] != 0:
            raise RuntimeError("%s: status %d" % (self.path, status[0]))
        return flags[:nx * ny * nz].copy(), tuple(int(c) for c in counts)
