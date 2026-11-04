#!/usr/bin/env python3
"""MERGE10: host check of the board backend memory_pl.MMIOBackend on a memory-simulated slot 14.

SimWindow is the 64 KiB slot as host memory: a write of ap_start to AP_CTRL makes it run the core C model on the
words the PS left in the slot (cmd half at the layout's cmd offset) and put the result at the layout's res offset,
then set ap_done (clear on read). The res half persists in the slot between calls like the BRAM / s_axilite array
on the board; if the PS changed it since the core last wrote it, the sim records that (the board core would see it).

Replays a call recording (tools/memk0/record_calls.py format: per call [hdr4, cmd1024, res_before1024,
res_after1024]) twice in one process with two independent copies of the library (separate static state):
  arm A: MMIOBackend(SimWindow(layout, lib copy A)).call(owner, cmd, res)
  arm B: direct ctypes call into lib copy B (what memory_pl.CModel does), res array kept across calls
and compares, per call, the 1024 result words of A, B and the recording's res_after.

usage: mmio_sim_check.py <lib.so> <layout lite|bram> <recording.bin> [--first N] [--mutate cmd_off|res_off|ps_writes_res]
--mutate is a positive control of the comparison: the driver's layout is shifted by one word (cmd or res), or the
driver writes one word into the res half before every call; each must be caught.
"""
import argparse
import ctypes
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from perception.fpga_replay import memory_pl  # noqa: E402


def load_copy(lib, tag):
    d = tempfile.mkdtemp(prefix="mmiosim_")
    p = os.path.join(d, "lib_%s.so" % tag)
    shutil.copy(lib, p)
    L = ctypes.CDLL(p)
    L.memk0_call.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    return L


class SimWindow:
    def __init__(self, lib, layout):
        self.mem = np.zeros(memory_pl.MEMK_WINDOW // 4, np.uint32)
        self.lib, self.off = lib, memory_pl.LAYOUTS[layout]
        self.done = False
        self.last_res = None
        self.ps_wrote_res = 0
        self.runs = 0

    def read32(self, off):
        if off == memory_pl.AP_CTRL:
            v = (memory_pl.AP_DONE if self.done else 0) | memory_pl.AP_IDLE
            self.done = False          # clear on read
            return v
        return int(self.mem[off >> 2])

    def write32(self, off, v):
        if off == memory_pl.AP_CTRL:
            if v & memory_pl.AP_START:
                self._run()
            return
        self.mem[off >> 2] = v

    def write_words(self, off, words):
        k0 = off >> 2
        for k, v in enumerate(np.asarray(words, np.uint32).tolist()):
            self.mem[k0 + k] = v

    def read_words(self, off, out):
        k0 = off >> 2
        for k in range(len(out)):
            out[k] = self.mem[k0 + k]

    def _run(self):
        c0, r0 = self.off["cmd"] >> 2, self.off["res"] >> 2
        res = self.mem[r0:r0 + 1024]
        if self.last_res is not None and not np.array_equal(res, self.last_res):
            self.ps_wrote_res += 1
        cmd = np.ascontiguousarray(self.mem[c0:c0 + 1024])
        out = np.ascontiguousarray(res.copy())
        self.lib.memk0_call(cmd.ctypes.data, out.ctypes.data)
        self.mem[r0:r0 + 1024] = out
        self.last_res = out.copy()
        self.done = True
        self.runs += 1


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("lib"); p.add_argument("layout", choices=sorted(memory_pl.LAYOUTS)); p.add_argument("rec")
    p.add_argument("--first", type=int, default=None)
    p.add_argument("--mutate", choices=("cmd_off", "res_off", "ps_writes_res"))
    a = p.parse_args(argv)
    rec = np.fromfile(a.rec, dtype="<u4").reshape(-1, 4 + 3 * 1024)
    if a.first:
        rec = rec[:a.first]
    LA, LB = load_copy(a.lib, "A"), load_copy(a.lib, "B")
    win = SimWindow(LA, a.layout)
    be = memory_pl.MMIOBackend(win, a.layout)
    if a.mutate in ("cmd_off", "res_off"):
        k = a.mutate[:3]
        be.off = dict(be.off, **{k: be.off[k] + 4})
    owner = object()
    resA = np.zeros(1024, np.uint32)
    resB = np.zeros(1024, np.uint32)
    bad_ab = bad_ar = 0
    first_bad = None
    for i, row in enumerate(rec):
        cmd = np.ascontiguousarray(row[4:1028])
        if a.mutate == "ps_writes_res":
            win.write32(be.off["res"] + 4 * 1023, 0xDEADBEEF)
        be.call(owner, cmd, resA)
        LB.memk0_call(cmd.ctypes.data, resB.ctypes.data)
        ra = row[2052:]
        dab, dar = not np.array_equal(resA, resB), not np.array_equal(resA, ra)
        bad_ab += dab; bad_ar += dar
        if (dab or dar) and first_bad is None:
            first_bad = i
    n = len(rec)
    ok = bad_ab == 0 and bad_ar == 0 and win.ps_wrote_res == 0 and win.runs == n
    print("MMIOSIM lib=%s layout=%s rec=%s calls=%d core_runs=%d mmio_vs_direct_mismatch=%d mmio_vs_recording_mismatch=%d "
          "ps_wrote_res_half=%d first_bad=%s mutate=%s -> %s" % (
              os.path.basename(a.lib), a.layout, os.path.basename(a.rec), n, win.runs, bad_ab, bad_ar, win.ps_wrote_res,
              first_bad, a.mutate, "IDENTICAL" if ok else "DIFFERENT"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
