"""The AR sender's depth chain in C++ (2026-10-16, docs/design/AR_COLOUR_MAIN_20261016.md section 13).

``fill_ar_inputs(..., native=ArFillNative(...))`` computes the frame's u16 display depth with
tools/kv_link/ar_helper/m7ar_fill.cpp (a per-code table of the quantized mm, rebuilt only when the
frame's unit changes, then one lookup per pixel plus a table for the stretch) instead of the numpy chain codes_to_m ->
quantize_optical_z -> stretch_depth_mm (about 40 ms per 640x360 frame on the KV260's A53; that
chain stays in host/kv_ar.py as the reference and is what runs when ``native`` is None). The
library is compiled from the deployed source at kv_server start (the same pattern as m7_frame.so /
m7_object.so), into a fresh path; ``describe()`` names the source sha256, the .so sha256 and the
compiler, so every run's summary says which source it ran.

Only the common case goes native: uint16 codes of at least W x H, 1 <= W <= 1280, 1 <= H <= 800,
a 32-bit unit pattern and a finite stretch k <= 65535; anything else takes the numpy reference
(same values, same exceptions) and is counted in ``counts``.

Python 3.8 compatible.
"""

import ctypes
import hashlib
import math
import os
import shutil
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tools" / "kv_link" / "ar_helper" / "m7ar_fill.cpp"
FLAGS = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off", "-fPIC", "-shared"]
MAX_W, MAX_H = 1280, 800
K_MAX = 65535.0
ABI = 1


def _sha256(path):
    with open(str(path), "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def compile_ar_fill(output, mutant=0, source=SOURCE):
    """Build m7ar_fill.cpp into ``output`` (a fresh path in an existing directory). ``mutant``: tests'
    positive controls only (1 stretch without k, 2 truncate, 3 double comparison)."""
    compiler = shutil.which("g++")
    if compiler is None:
        raise RuntimeError("g++ is required for the AR depth fill library")
    output = Path(output).resolve()
    if not output.parent.is_dir() or output.exists():
        raise ValueError("choose a fresh library path in an existing directory")
    cmd = [compiler] + FLAGS + (["-DM7AR_FILL_MUTANT=%d" % int(mutant)] if mutant else []) + [
        str(source), "-o", str(output)]
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise RuntimeError("%s failed (rc %d): %s" % (" ".join(cmd), r.returncode,
                                                      r.stdout.decode("utf-8", "replace")[-800:]))
    return output


class ArFillNative:
    def __init__(self, library, source=SOURCE):
        self.path = str(Path(library).resolve())
        self.lib = ctypes.CDLL(self.path)            # CDLL: the GIL is released during the call
        self.lib.m7ar_fill_abi.restype = ctypes.c_int
        self.lib.m7ar_fill_mutant.restype = ctypes.c_int
        if self.lib.m7ar_fill_abi() != ABI:
            raise RuntimeError("m7ar_fill ABI %d, %d expected" % (self.lib.m7ar_fill_abi(), ABI))
        self.mutant = int(self.lib.m7ar_fill_mutant())
        self.lib.m7ar_mm_lut.argtypes = [ctypes.c_uint32, ctypes.c_void_p]
        self.lib.m7ar_mm_lut.restype = None
        f = self.lib.m7ar_depth_apply
        f.argtypes = [ctypes.c_void_p, ctypes.c_int64, ctypes.c_void_p, ctypes.c_double,
                      ctypes.c_void_p, ctypes.c_void_p]
        f.restype = ctypes.c_int
        self._apply = f
        self._lut_bits, self._lut = None, np.zeros(65536, dtype="<u2")
        self.counts = Counter()
        self.source = str(source)
        self.source_sha256 = _sha256(source) if os.path.exists(str(source)) else None
        self.so_sha256 = _sha256(self.path)
        self.last_stats = None

    @classmethod
    def build(cls, out_dir, mutant=0, name="m7ar_fill.so"):
        return cls(compile_ar_fill(os.path.join(str(out_dir), name), mutant=mutant))

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
                "mutant": self.mutant, "counts": dict(self.counts)}

    def why_not(self, codes, unit_bits, width, height, k):
        """None when the native path applies, else the reason (the caller then runs the reference)."""
        if not isinstance(codes, np.ndarray) or codes.dtype != np.dtype("<u2"):
            return "codes_dtype"
        if not (1 <= width <= MAX_W and 1 <= height <= MAX_H):
            return "size"
        if codes.size < width * height:
            return "codes_short"
        if not isinstance(unit_bits, (int, np.integer)) or not 0 <= int(unit_bits) <= 0xFFFFFFFF:
            return "unit_bits"
        if not isinstance(k, (int, float, np.floating, np.integer)) or not math.isfinite(float(k)) \
                or float(k) > K_MAX:
            return "stretch_k"
        return None

    def depth_u16(self, codes, unit_bits, width, height, k):
        """u16[width*height] display depth (C-contiguous), or None when the reference must run."""
        why = self.why_not(codes, unit_bits, width, height, k)
        if why is not None:
            self.counts["python:%s" % why] += 1
            return None
        n = int(width) * int(height)
        src = np.ascontiguousarray(codes.reshape(-1)[:n])
        out = np.empty(n, dtype="<u2")
        stats = np.zeros(2, dtype=np.int64)
        if self._lut_bits != int(unit_bits):     # the per-code table, rebuilt only when the unit changes
            self.lib.m7ar_mm_lut(int(unit_bits), self._lut.ctypes.data)
            self._lut_bits = int(unit_bits)
            self.counts["lut_builds"] += 1
        self._apply(src.ctypes.data, n, self._lut.ctypes.data, float(k), out.ctypes.data, stats.ctypes.data)
        self.last_stats = (int(stats[0]), int(stats[1]))
        self.counts["native"] += 1
        return out
