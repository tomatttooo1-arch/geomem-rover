#!/usr/bin/env python3
"""Replay a recorded rolling-map trace (make_trace.py) through a backend, word by word (v3 L2).

Backends (``--backend``):
  cpu     host/kv_rolling.CpuRollingCore (fresh g++ build; sanity / mutant controls)
  mock    PlRollingCore over MockRollingWindow (the AXI shell's semantics on the g++ core)
          inside MockPlDevice memory
  pl      the PL core on the KV260 (UIO m7_rolling, slot 5; sysinfo gate), DMA buffers of
          DevMemPlDevice -- board only
  remote  RemotePlRollingBackend to a kv_server --rolling cpu|pl at --server H:P (Nano)
Equal = per record: header[24], the first n_delta delta words, n_qout query words, the dump.
Op GRID records (op 4, the next bitstream's fused-core grid op; MERGE1/b1/grid_formal.bin written by
tools/b1_costmap/bench_int.py --trace-out), SW_M1 2026-11-03 from BOARD2 scripts/grid_replay.py: backends mock
and pl only. The core is built for GRID (mock: compile_rolling(grid=True) + MockRollingWindow(grid=True); pl:
PlRollingCore grid="probe", one read-only GRID at start -- capable or not is printed), the record's flags word
(qc) goes to QUERY_COUNT for GRID as for QUERY, and after an accepted GRID the N_DELTA plane words are read back
from the delta buffer and compared like an INTEGRATE's delta. A trace without GRID records replays exactly as
before. ``--corrupt-grid-flags``: positive control, the GRID flags xor 1 -> every accepted GRID must differ
(use with --expect-fail). ON THE BOARD, RELOAD THE BITSTREAM FIRST (xmutil unloadapp; xmutil loadapp <app>):
grid_formal.bin begins with checks on a never-reset core (expected status UNINITIALIZED); a core left
initialised by an earlier replay fails every record (BOARD2 README 3.4).
  positive-control usage (host):  replay_trace.py --trace MERGE1/b1/grid_formal.bin --backend mock     -> PASS
                                  replay_trace.py --trace ... --backend mock --corrupt-grid-flags --expect-fail -> PASS
``--mutant NAME`` (cpu/mock): core built with -DFPGA_M7_ROLL_MUTANT_NAME; ``--shell-mutant
drop_last_delta`` (mock); with ``--expect-fail`` exit 0 only if a mismatch is found.
Writes ``--out`` JSON (verdict, first mismatches, per-op device/prep/readback ns medians).
Python 3.8 compatible; standard library + numpy.
"""
import argparse
import json
import os
import struct
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

REC_HEAD = struct.Struct("<3I6i8I")
OP_QUERY = 2
OP_GRID = 4


def read_trace(path):
    with open(path, "rb") as f:
        data = f.read()
    if data[:4] != b"M7RT":
        raise ValueError("not a rolling trace")
    version, n = struct.unpack_from("<II", data, 4)
    if version != 1:
        raise ValueError("trace version %d" % version)
    off, out = 12, []
    for _ in range(n):
        h = REC_HEAD.unpack_from(data, off)
        off += REC_HEAD.size
        op, epoch, frame = h[0], h[1], h[2]
        wb, ob, dims, qc = h[3:6], h[6:9], h[9:12], h[12]
        nob, nd, nq, ndump = h[13], h[14], h[15], h[16]
        obs = data[off:off + nob]
        off += (nob + 7) // 8 * 8
        qin = 3 * qc if (op == OP_QUERY and qc <= 1024) else 0
        q = np.frombuffer(data, "<i4", qin, off).astype(np.int32)
        off += 4 * qin
        hdr = np.frombuffer(data, "<u4", 24, off).astype(np.uint32)
        off += 96
        delta = np.frombuffer(data, "<u8", nd, off).astype(np.uint64)
        off += 8 * nd
        qout = np.frombuffer(data, "<u4", nq, off).astype(np.uint32)
        off += 4 * nq
        dump = np.frombuffer(data, "<u8", ndump, off).astype(np.uint64)
        off += 8 * ndump
        out.append({"op": op, "epoch": epoch, "frame": frame, "wb": wb, "ob": ob, "dims": dims,
                    "qc": qc, "obs": obs if nob else None, "query": q, "header": hdr,
                    "delta": delta, "qout": qout, "dump": dump})
    if off != len(data):
        raise ValueError("trailing bytes in trace")
    return out


def compare(rec, hdr, delta, qout, dump):
    """Number of differing words (an absent output where one is expected counts fully)."""
    bad = int(np.count_nonzero(np.asarray(hdr, np.uint32) != rec["header"]))
    for want, got in ((rec["delta"], delta), (rec["qout"], qout), (rec["dump"], dump)):
        if want.size == 0:
            continue
        got = np.zeros(0, want.dtype) if got is None else np.asarray(got)
        if got.size < want.size:
            bad += want.size - got.size
            want = want[:got.size]
        bad += int(np.count_nonzero(got[:want.size] != want))
    return bad


def make_caller(a, build, grid=False):
    from host.kv_rolling import CpuRollingCore, MockRollingWindow, PlRollingCore
    from host.m7_rolling import compile_rolling
    gkw = {"grid": "probe"} if grid else {}                  # no GRID record: the core exactly as before
    if a.backend in ("cpu", "mock", "pl"):
        if a.backend == "cpu":
            core = CpuRollingCore(compile_rolling(os.path.join(build, "k.so"), mutant=a.mutant))
        elif a.backend == "mock":
            from host.kv_rolling import kv_config_words
            from host.m7_pl_device import MockPlDevice
            dev = MockPlDevice()
            lib = compile_rolling(os.path.join(build, "k.so"), mutant=a.mutant, **({"grid": True} if grid else {}))
            win = MockRollingWindow(dev._memory, lib, mutant=a.shell_mutant, **({"grid": True} if grid else {}))
            core = PlRollingCore(win, dev, kv_config_words(build), **gkw)
        else:
            from host.kv_regs import open_blocks
            from host.kv_rolling import kv_config_words, open_rolling_window
            from host.m7_pl_device import DevMemPlDevice
            regs = open_blocks()
            win, detail = open_rolling_window(regs.get("sysinfo"))
            core = PlRollingCore(win, DevMemPlDevice.open(), kv_config_words(build), detail=detail, **gkw)
            print(json.dumps({"rolling_gate": detail}), flush=True)
        if grid:
            print(json.dumps({"grid_probe": core.grid_probe}), flush=True)

        def call(r):
            kw = {}
            if r["obs"] is not None:
                kw["obs"] = r["obs"]
            if r["op"] == OP_QUERY:
                kw["query"], kw["qc"] = r["query"], r["qc"]
            if r["op"] == OP_GRID:                       # the flags word rides in QUERY_COUNT (BOARD2 grid_replay)
                kw["qc"] = int(r["qc"]) ^ (1 if a.corrupt_grid_flags else 0)
            hdr, delta, qout, dump, times = core.call(r["op"], r["epoch"], r["frame"], r["wb"], r["ob"],
                                                       r["dims"], **kw)
            if r["op"] == OP_GRID and int(hdr[0]) == 0:  # the planes: N_DELTA words in the delta buffer
                from host.kv_rolling import H
                nd = int(hdr[H["N_DELTA"]])
                core.buffers.from_device(core.b["delta"])
                delta = core.b["delta"].array[:8 * nd].view("<u8").astype(np.uint64)
                core.counts["grid_planes_read"] += 1
            return hdr, delta, qout, dump, {"prep": times[0], "device": times[1], "readback": times[2]}
        return call, core
    from host.kv_rolling import RemotePlRollingBackend
    host, port = a.server.rsplit(":", 1)
    be = RemotePlRollingBackend((host, int(port)), timeout_s=a.timeout_s)

    def call(r):
        kw = dict(epoch=r["epoch"], frame_seq=r["frame"], window_base=r["wb"], obs_base=r["ob"],
                  dims=r["dims"])
        if r["obs"] is not None:
            kw["block_bytes"] = r["obs"]
        if r["op"] == OP_QUERY:
            kw["query"] = r["query"].reshape(-1, 3)
            kw["query_count"] = r["qc"]
        t0 = time.monotonic_ns()
        ack = be.raw(r["op"], **kw)
        t = dict(ack["times_ns"])
        t["rtt"] = time.monotonic_ns() - t0
        return ack["header"], ack["delta"], ack["qout"], ack["dump"], t
    return call, be


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trace", required=True)
    ap.add_argument("--backend", choices=("cpu", "mock", "pl", "remote"), required=True)
    ap.add_argument("--server", default=None)
    ap.add_argument("--timeout-s", type=float, default=0.5)
    ap.add_argument("--mutant", default=None)
    ap.add_argument("--shell-mutant", default=None, choices=(None, "drop_last_delta"))
    ap.add_argument("--expect-fail", action="store_true")
    ap.add_argument("--corrupt-grid-flags", action="store_true",
                    help="positive control for GRID records: their flags xor 1 (with --expect-fail)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    if a.backend == "remote" and not a.server:
        ap.error("--backend remote needs --server")
    if (a.mutant or a.shell_mutant) and a.backend not in ("cpu", "mock"):
        ap.error("mutants are host-only (cpu/mock)")
    recs = read_trace(a.trace)
    grid = any(r["op"] == OP_GRID for r in recs)
    if grid and a.backend not in ("mock", "pl"):
        ap.error("the trace has op GRID records: --backend mock or pl")
    if a.corrupt_grid_flags and not grid:
        ap.error("--corrupt-grid-flags needs a trace with op GRID records")
    build = tempfile.mkdtemp(prefix="m7rr_")
    call, core = make_caller(a, build, grid=grid)
    mism, first, times = 0, [], {}
    words = 0
    t_cpu0, t_wall0 = time.thread_time_ns(), time.monotonic_ns()
    for i, r in enumerate(recs):
        hdr, delta, qout, dump, t = call(r)
        bad = compare(r, hdr, delta, qout, dump)
        words += 24 + r["delta"].size + r["qout"].size + r["dump"].size
        for k, v in t.items():
            times.setdefault("op%d:%s" % (r["op"], k), []).append(int(v))
        if bad:
            mism += 1
            if len(first) < 8:
                first.append({"record": i, "op": r["op"], "frame": r["frame"], "words": bad,
                              "status": [int(hdr[0]), int(r["header"][0])]})
    equal = mism == 0
    summary = {"trace": os.path.abspath(a.trace), "backend": a.backend, "mutant": a.mutant,
               "shell_mutant": a.shell_mutant, "records": len(recs), "words": words,
               **({"grid_records": sum(r["op"] == OP_GRID for r in recs), "corrupt_grid_flags": a.corrupt_grid_flags,
                   "grid_probe": getattr(core, "grid_probe", None),
                   "grid_planes_read": int(core.counts.get("grid_planes_read", 0))} if grid else {}),
               "mismatched_records": mism, "first_mismatches": first, "bit_exact": equal,
               "thread_cpu_ns": time.thread_time_ns() - t_cpu0, "wall_ns": time.monotonic_ns() - t_wall0,
               "median_ns": {k: int(np.median(v)) for k, v in sorted(times.items())},
               "max_ns": {k: int(np.max(v)) for k, v in sorted(times.items())}}
    if a.out:
        json.dump(summary, open(a.out, "w"), indent=1)
    verdict = (not equal) if a.expect_fail else equal
    summary["verdict"] = "PASS" if verdict else "FAIL"
    print(json.dumps({k: summary[k] for k in ("backend", "mutant", "shell_mutant", "records", "words",
                                              "mismatched_records", "bit_exact", "verdict")}))
    return 0 if verdict else 1


if __name__ == "__main__":
    sys.exit(main())
