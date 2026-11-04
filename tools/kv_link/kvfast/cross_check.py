#!/usr/bin/env python3
"""E7: kv_fast.cpp built for aarch64 (NEON + ARMv8 crc32), run under qemu-aarch64 on the host (or natively on
the KV260 with --runner ""), compared byte for byte with the Python reference (host/kv_link.codes_to_m +
zlib crc32) on

  * the prepared recording (rec30: 310 frames of 640x360 Gemini 2 codes), codes at an ODD byte offset with
    the crc32 instructions, and at offset 0 with the table CRC (both CRC paths, both alignments);
  * synthetic edge cases: random codes with 0 / 1 / 65535, odd pixel counts (vector tail), units 0.001,
    0.0002, 1.0, 1e-41 (subnormal results), 3e38 (overflow to inf), a NaN unit.

Usage: cross_check.py --checker BIN [--runner qemu-aarch64-static] --prep DIR --work DIR [--frames N]
Prints one JSON line; exit 0 only when every float bit and every crc is equal.
"""
import argparse
import json
import os
import struct
import subprocess
import sys
import zlib

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.kv_link import bits_f32, codes_to_m  # noqa: E402


def run(runner, checker, codes_path, n, unit_bits, offset, out_f32, out_crc, table):
    cmd = ([runner] if runner else []) + [checker, "convert", codes_path, str(n), "%08x" % unit_bits,
                                          str(offset), out_f32, out_crc] + (["table"] if table else [])
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise SystemExit("checker failed rc %d: %s" % (r.returncode, r.stdout.decode()[-400:]))
    return r.stdout.decode().strip()


def check(codes_frames, unit_bits, out_f32, out_crc):
    """(frames compared, first bad frame or None, reason)."""
    n_frames, n = codes_frames.shape
    got = np.memmap(out_f32, dtype="<u4", mode="r")
    if got.size != n_frames * n:
        return 0, 0, "size %d != %d" % (got.size, n_frames * n)
    crcs = [line.split() for line in open(out_crc)]
    unit = bits_f32(unit_bits)
    for f in range(n_frames):
        codes = np.ascontiguousarray(codes_frames[f])
        with np.errstate(over="ignore", invalid="ignore"):
            ref = np.ascontiguousarray(codes_to_m(codes, unit), dtype="<f4")
        if not np.array_equal(ref.view("<u4"), got[f * n:(f + 1) * n]):
            return f, f, "float bits"
        if int(crcs[f][0], 16) != zlib.crc32(codes.tobytes()) & 0xFFFFFFFF:
            return f, f, "codes crc"
        if int(crcs[f][1], 16) != zlib.crc32(ref.tobytes()) & 0xFFFFFFFF:
            return f, f, "depth crc"
    return n_frames, None, None


def f32_bits(v):
    return struct.unpack("<I", struct.pack("<f", v))[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checker", required=True)
    ap.add_argument("--runner", default="qemu-aarch64-static")
    ap.add_argument("--prep", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--frames", type=int, default=0, help="0 = all frames of the recording")
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=False)
    meta = json.load(open(os.path.join(a.prep, "meta.json")))
    w, h = int(meta["width"]), int(meta["height"])
    n = w * h
    depth = np.memmap(os.path.join(a.prep, meta["depth_file"]), dtype="<u2", mode="r").reshape(-1, n)
    if a.frames:
        depth = depth[:a.frames]
    codes_path = os.path.join(a.work, "codes.bin")
    np.ascontiguousarray(depth).tofile(codes_path)
    unit_bits = f32_bits(float(meta["depth_unit_m"]))
    results = {}
    info = None
    for name, offset, table in (("rec_odd_hw", 1, False), ("rec_even_table", 0, True)):
        out_f32, out_crc = os.path.join(a.work, name + ".f32"), os.path.join(a.work, name + ".crc")
        info = run(a.runner, a.checker, codes_path, n, unit_bits, offset, out_f32, out_crc, table)
        done, bad, why = check(depth, unit_bits, out_f32, out_crc)
        results[name] = {"frames": int(depth.shape[0]), "compared": done, "first_bad": bad, "why": why,
                         "checker": info}
        os.remove(out_f32)              # 285 MB each for rec30: not kept
    rng = np.random.RandomState(20261102)
    syn_n = 1037                        # odd: exercises the scalar tail after the 8-wide vector loop
    syn = rng.randint(0, 65536, size=(7, syn_n)).astype("<u2")
    syn[:, :50] = 0
    syn[:, 50:60] = 1
    syn[:, 60:70] = 65535
    syn_path = os.path.join(a.work, "syn.bin")
    syn.tofile(syn_path)
    for label, bits in (("unit_0.001", f32_bits(0.001)), ("unit_0.0002", f32_bits(0.0002)),
                        ("unit_1", f32_bits(1.0)), ("unit_subnormal", f32_bits(1e-41)),
                        ("unit_overflow", f32_bits(3e38)), ("unit_nan", 0x7fc00001)):
        for offset, table in ((1, False), (0, True)):
            name = "syn_%s_off%d_%s" % (label, offset, "table" if table else "hw")
            out_f32, out_crc = os.path.join(a.work, name + ".f32"), os.path.join(a.work, name + ".crc")
            run(a.runner, a.checker, syn_path, syn_n, bits, offset, out_f32, out_crc, table)
            done, bad, why = check(syn, bits, out_f32, out_crc)
            results[name] = {"frames": 7, "compared": done, "first_bad": bad, "why": why}
    os.remove(codes_path)
    ok = all(r["first_bad"] is None and r["compared"] == r["frames"] for r in results.values())
    out = {"PASS": ok, "checker_info": info, "cases": len(results),
           "failed": {k: v for k, v in results.items() if v["first_bad"] is not None}, "results": results}
    json.dump(out, open(os.path.join(a.work, "cross_check.json"), "w"), indent=1, sort_keys=True)
    print(json.dumps({k: out[k] for k in ("PASS", "checker_info", "cases", "failed")}))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
