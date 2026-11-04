#!/usr/bin/env python3
"""DPUPIPE0: byte-exact check of what reached the DPU input buffer. For every inferred frame in the
receiver's --infer-log (crc32 of the whole size x size x 3 tensor, pad included, from dpu_rx --tensor-crc)
find the camera frame by source_key in bench_tx --keys, recompute host/dpu_prep.DpuPrep(size, fp, numpy)
.full_tensor and compare crc32. Prints JSON {checked, mismatched, missing_key}.
  python3 verify_tensors.py --infer-log inf.jsonl --keys keys.jsonl --size 512 --fix 7 --images a.png ...
"""
import argparse
import json
import os
import sys
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from host import dpu_prep  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_tx import load_frames  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--infer-log", required=True)
    ap.add_argument("--keys", required=True)
    ap.add_argument("--size", type=int, required=True)
    ap.add_argument("--fix", type=int, required=True)
    ap.add_argument("--images", nargs="*", default=[])
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    frames = load_frames(a.images)
    key2idx = {}
    for line in open(a.keys):
        d = json.loads(line)
        key2idx[d["source_key"]] = d["frame_index"]
    prep = dpu_prep.DpuPrep(a.size, a.fix, backend="numpy")
    want = {}
    checked = mism = missing = 0
    for line in open(a.infer_log):
        d = json.loads(line)
        idx = key2idx.get(d["source_key"])
        if idx is None:
            missing += 1
            continue
        if idx not in want:
            want[idx] = zlib.crc32(prep.full_tensor(frames[idx]).tobytes()) & 0xFFFFFFFF
        checked += 1
        mism += int(want[idx] != d["tensor_crc"])
    out = {"checked": checked, "mismatched": mism, "missing_key": missing, "distinct_frames": len(want)}
    print(json.dumps(out))
    if a.out:
        json.dump(out, open(a.out, "w"))
    return 0 if checked and not mism and not missing else 1


if __name__ == "__main__":
    sys.exit(main())
