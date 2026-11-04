"""PS side of the fixed-point geometry frame core (GEO_FX3, 2026-11-04).

The frame core since GEO_FX1 (hw/m7/pl/hls/m7_frame_stream.cpp) reads 16-bit depth codes only and needs
per-frame constants from the PS (runs/nationals-prelim-20261102/GEO_FX0/DESIGN.md section 2.2). They are
computed by C++ (hw/m7/frame_fx.cpp ``fpga_m7_fx_prepare``), compiled here with -ffp-contract=off: on the
A53 GCC would otherwise contract a*b+c into FMA. The same library holds ``fpga_m7_fx_frame``, the bit-exact
model of the core from its register values (the expectation of every PL check since GEO_FX3: the mock
device, the host tests, the board check in GEO_FX3/README.md).

Bit-exactness of the constants against the GEO_FX0 golden model: GEO_FX3 tools/ps_check.cpp, rec30 310 +
car570 570 frames, host x86 and aarch64 (qemu), 0 differences; the core model against the golden FX flags,
counts, statistics and crc on the same 880 frames, 0 differences.
"""

import ctypes
from pathlib import Path
import shutil
import struct
import subprocess

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FX_K_WORDS = 24
REACH_W1 = 131072.0          # reach < 2^17: the W1 ray record of the GEO_FX1 W1 / X1 cores
FX_STATUS_ENVELOPE = 5       # PS status: outside the fixed-point envelope (CPU reference instead)

_I64 = np.ctypeslib.ndpointer(dtype=np.int64, flags="C_CONTIGUOUS")
_I32 = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
_U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
_U16 = np.ctypeslib.ndpointer(dtype=np.uint16, flags="C_CONTIGUOUS")
_U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
_F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
_F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")


def compile_frame_fx(output):
    """Build the PS-constant + core-model library from hw/m7/frame_fx.cpp (fresh path)."""
    compiler = shutil.which("g++")
    if compiler is None:
        raise RuntimeError("g++ is required for the fixed-point frame driver")
    output = Path(output).resolve()
    if not output.parent.is_dir() or output.exists():
        raise ValueError("choose a fresh output path within an existing directory")
    subprocess.run([compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
                    "-fPIC", "-shared", str(ROOT / "hw/m7/frame_fx.cpp"), "-o", str(output)],
                   check=True, capture_output=True, text=True)
    return output


def f32_bits(value):
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


class FrameFx:
    """ctypes view of hw/m7/frame_fx.cpp."""

    def __init__(self, library):
        self.path = str(Path(library).resolve())
        lib = ctypes.CDLL(self.path)
        u32, c_double = ctypes.c_uint32, ctypes.c_double
        lib.fpga_m7_fx_prepare.argtypes = [u32, u32, u32, u32, u32, ctypes.c_float, _F32, _F32, _F64,
                                          c_double, c_double, _I64, _I32, _U32, _F64]
        lib.fpga_m7_fx_prepare.restype = ctypes.c_int
        lib.fpga_m7_fx_pack.argtypes = [_I64, _U32]
        lib.fpga_m7_fx_pack.restype = None
        lib.fpga_m7_fx_frame.argtypes = [_U16, u32, u32, u32, u32, u32, _U32, _U32, u32, u32, u32, u32,
                                        _U8, _U32, _U32, _U32]
        lib.fpga_m7_fx_frame.restype = u32
        self.lib = lib

    def prepare(self, width, height, nx, ny, nz, unit_bits, inverse_k, world_camera, origin, voxel,
                reach_limit=REACH_W1):
        """Per-frame constants. Returns a dict: status (0 ok, 2/3 as the reference, 5 envelope),
        words (FX_K register image, 24 x u32), origin (3 x Q8 int), codes (c_lo | c_hi << 16), c20,
        thresholds (c_lo, c_hi, c20), pmax, reach."""
        gk = np.zeros(12, dtype=np.int64)
        og = np.zeros(3, dtype=np.int32)
        thr = np.zeros(3, dtype=np.uint32)
        env = np.zeros(2, dtype=np.float64)
        unit = struct.unpack("<f", struct.pack("<I", int(unit_bits) & 0xFFFFFFFF))[0]
        st = self.lib.fpga_m7_fx_prepare(
            int(width), int(height), int(nx), int(ny), int(nz), unit,
            np.ascontiguousarray(inverse_k, dtype=np.float32).reshape(-1),
            np.ascontiguousarray(world_camera, dtype=np.float32).reshape(-1),
            np.ascontiguousarray(origin, dtype=np.float64).reshape(-1), float(voxel), float(reach_limit),
            gk, og, thr, env)
        words = np.zeros(FX_K_WORDS, dtype=np.uint32)
        self.lib.fpga_m7_fx_pack(gk, words)
        return {"status": int(st), "gk": gk, "words": words, "origin": [int(x) for x in og],
                "codes": (int(thr[0]) | (int(thr[1]) << 16)) & 0xFFFFFFFF, "c20": int(thr[2]),
                "thresholds": [int(x) for x in thr], "pmax": float(env[0]), "reach": float(env[1])}

    def frame(self, codes, width, height, nx, ny, nz, words, origin, codes_reg, c20, dlat2_en, unit_bits):
        """The core from register values. Returns (status, flags[32768], counts[3], stats[4], crc)."""
        pixels = int(width) * int(height)
        c = np.ascontiguousarray(np.asarray(codes, dtype="<u2").reshape(-1)[:pixels], dtype=np.uint16)
        if c.size < max(pixels, 1):
            c = np.concatenate([c, np.zeros(max(pixels, 1) - c.size, dtype=np.uint16)])
        flags = np.zeros(32 ** 3, dtype=np.uint8)
        counts = np.zeros(3, dtype=np.uint32)
        stats = np.zeros(4, dtype=np.uint32)
        crc = np.zeros(1, dtype=np.uint32)
        st = self.lib.fpga_m7_fx_frame(c, int(width), int(height), int(nx), int(ny), int(nz),
                                      np.ascontiguousarray(words, dtype=np.uint32),
                                      np.ascontiguousarray([int(x) & 0xFFFFFFFF for x in origin], dtype=np.uint32),
                                      int(codes_reg) & 0xFFFFFFFF, int(c20) & 0xFFFFFFFF, int(dlat2_en) & 1,
                                      int(unit_bits) & 0xFFFFFFFF, flags, counts, stats, crc)
        return int(st), flags, counts, stats, int(crc[0])
