"""KV260 frame server: the per-pixel data plane in C++ (E7, nationals prelim 2026-11-02).

``kv_server --kv-fast native`` (default ``off`` = exactly as before). Per geometry request the Python
server (host/kv_link.py KvLinkServer._serve) spends its A53 time on three O(pixels) numpy/zlib steps
(car 10-27, 640x480, kv_times medians: decode 2.0 ms -- mostly the codes crc32 --, convert 3.5 ms, depth crc
3.3 ms; runs/fpga-innov-20261025/golive/AR_LATENCY_PLAN_20261027.md section 1 row 6). This module runs them
in tools/kv_link/kvfast/kv_fast.cpp: the codes crc32 (ARMv8 crc32 instructions on the A53), and ONE pass
that converts u16 codes to float32 metres with NEON and crc32s the float bytes while they are in L1.
Everything else (sockets, reassembly, latest slot, permit feed, registers, interrupt, ROI, rolling, AR,
TELEM) stays Python, unchanged: the wire protocol and the PL register semantics do not change.

Bit-exact with the Python path by construction and by test (tests/test_kv_fast_native.py, rec30 310 frames
in tools/kv_link/kvfast/rec30_equivalence.py): same float bits (one IEEE single multiply, code 0 -> the
NaN numpy writes), same zlib crc32. Only the common case goes native (uint16 contiguous codes, at most
MAX_PIXELS); anything else returns None and the caller runs the Python reference (counted).

The library is compiled from the deployed source at kv_server start into a fresh path (the m7_frame.so /
m7ar_fill.so pattern), so no binary is deployed; ``describe()`` names source and .so sha256, compiler, the
CRC implementation in use (``hw`` = ARMv8 crc32 instructions, ``table`` = slicing-by-8) and the SIMD path.
Python 3.8 compatible.
"""

import ctypes
import hashlib
import os
import platform
import shutil
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tools" / "kv_link" / "kvfast" / "kv_fast.cpp"
FLAGS = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off", "-fPIC", "-shared"]
ABI = 1
MODES = ("off", "native")
ENV = "FPGA_KV_FAST"          # default of kv_server --kv-fast when the flag is not given


def default_mode(env=None):
    """kv_server --kv-fast default: $FPGA_KV_FAST when it is a known word, else off."""
    env = os.environ if env is None else env
    v = str(env.get(ENV, "") or "").strip().lower()
    return v if v in MODES else "off"


def _sha256(path):
    with open(str(path), "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def compile_kv_fast(output, mutant=0, source=SOURCE, compiler=None):
    """Build kv_fast.cpp into ``output`` (a fresh path in an existing directory). ``mutant``: tests'
    positive controls only (1 = code 0 not NaN, 2 = depth crc skips the last byte)."""
    compiler = compiler or shutil.which("g++")
    if compiler is None:
        raise RuntimeError("g++ is required for the kv-fast library")
    output = Path(output).resolve()
    if not output.parent.is_dir() or output.exists():
        raise ValueError("choose a fresh library path in an existing directory")
    cmd = [compiler] + FLAGS + (["-DKVFAST_MUTANT=%d" % int(mutant)] if mutant else []) + [
        str(source), "-o", str(output)]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise RuntimeError("%s failed (rc %d): %s" % (" ".join(cmd), r.returncode,
                                                      r.stdout.decode("utf-8", "replace")[-800:]))
    return output


class KvFastNative:
    """ctypes front of kv_fast.so. ``crc32(codes)`` and ``convert_crc(codes, unit_bits, out)``."""

    def __init__(self, library, source=SOURCE, max_pixels=None):
        from host.m7_frame_native import MAX_PIXELS
        self.max_pixels = int(MAX_PIXELS if max_pixels is None else max_pixels)
        self.path = str(Path(library).resolve())
        self.lib = ctypes.CDLL(self.path)            # CDLL: the GIL is released during the call
        L = self.lib
        L.kvfast_abi.restype = ctypes.c_int
        if L.kvfast_abi() != ABI:
            raise RuntimeError("kv_fast ABI %d, %d expected" % (L.kvfast_abi(), ABI))
        L.kvfast_selftest.restype = ctypes.c_int
        st = L.kvfast_selftest()
        if st != 0:
            raise RuntimeError("kv_fast self test failed (%d)" % st)
        L.kvfast_mutant.restype = ctypes.c_int
        L.kvfast_crc_impl.restype = ctypes.c_int
        L.kvfast_simd.restype = ctypes.c_int
        L.kvfast_force_table.argtypes, L.kvfast_force_table.restype = [ctypes.c_int], None
        L.kvfast_crc32.argtypes = [ctypes.c_void_p, ctypes.c_int64]
        L.kvfast_crc32.restype = ctypes.c_uint32
        L.kvfast_convert_crc.argtypes = [ctypes.c_void_p, ctypes.c_int64, ctypes.c_uint32, ctypes.c_void_p,
                                         ctypes.c_void_p]
        L.kvfast_convert_crc.restype = ctypes.c_int
        self.mutant = int(L.kvfast_mutant())
        self.counts = Counter()
        self.source = str(source)
        self.source_sha256 = _sha256(source) if os.path.exists(str(source)) else None
        self.so_sha256 = _sha256(self.path)
        self._crc_out = np.zeros(1, dtype=np.uint32)

    @classmethod
    def build(cls, out_dir, mutant=0, name="kv_fast.so", **kw):
        return cls(compile_kv_fast(os.path.join(str(out_dir), name), mutant=mutant), **kw)

    def crc_impl(self):
        return "hw" if self.lib.kvfast_crc_impl() == 1 else "table"

    def force_table(self, on):
        self.lib.kvfast_force_table(1 if on else 0)

    def describe(self):
        cc = shutil.which("g++")
        ver = None
        if cc is not None:
            try:
                ver = subprocess.run([cc, "--version"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
                                     ).stdout.decode("utf-8", "replace").splitlines()[0]
            except (OSError, IndexError):
                ver = None
        return {"impl": "native", "so": self.path, "so_sha256": self.so_sha256, "source": self.source,
                "source_sha256": self.source_sha256, "cxx": ver, "flags": " ".join(FLAGS),
                "machine": platform.machine(), "crc": self.crc_impl(),
                "simd": "neon" if self.lib.kvfast_simd() == 1 else "portable",
                "mutant": self.mutant, "counts": dict(self.counts)}

    @staticmethod
    def _plain_u16(codes):
        return (isinstance(codes, np.ndarray) and codes.dtype == np.dtype("<u2") and codes.ndim == 1
                and codes.flags["C_CONTIGUOUS"])

    def crc32(self, codes):
        """zlib crc32 of the codes' bytes, or None when the reference must run."""
        if not self._plain_u16(codes):
            self.counts["python:crc_codes_layout"] += 1
            return None
        self.counts["native_crc"] += 1
        return int(self.lib.kvfast_crc32(codes.ctypes.data, 2 * codes.size))

    def convert_crc(self, codes, unit_bits, out):
        """out[:n] = codes_to_m(codes, bits_f32(unit_bits)) bit for bit (n = codes.size); returns
        crc32(out[:n]) -- or None when the reference must run (nothing written then)."""
        why = None
        if not self._plain_u16(codes):
            why = "codes_layout"
        elif codes.size > self.max_pixels:
            why = "size"
        elif not (isinstance(out, np.ndarray) and out.dtype == np.dtype("<f4") and out.flags["C_CONTIGUOUS"]
                  and out.size >= codes.size):
            why = "out_layout"
        elif not isinstance(unit_bits, (int, np.integer)) or not 0 <= int(unit_bits) <= 0xFFFFFFFF:
            why = "unit_bits"
        elif np.may_share_memory(codes, out):
            why = "overlap"
        if why is not None:
            self.counts["python:%s" % why] += 1
            return None
        rc = self.lib.kvfast_convert_crc(codes.ctypes.data, codes.size, int(unit_bits), out.ctypes.data,
                                         self._crc_out.ctypes.data)
        if rc != 0:
            self.counts["python:native_rc%d" % rc] += 1
            return None
        self.counts["native_convert"] += 1
        return int(self._crc_out[0])
