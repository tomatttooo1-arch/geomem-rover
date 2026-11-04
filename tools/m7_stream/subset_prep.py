#!/usr/bin/env python3
"""Cut a frame subset out of a prepared recording (tools/geo_same_input/prep_rec.py layout).

The frame indices are round(linspace(0, N-1, k)) (the set used by
tools/m7_stream/check_stream.py for TUM), so host C-model numbers and board numbers
refer to the same frames. Per-frame meta lists (color_index, acquired_ns, depth_png)
are cut with the same indices; depth_sha256 is recomputed; everything else is copied.
Usage: subset_prep.py --src DIR --out FRESH_DIR [--frames 200]
"""
import argparse
import hashlib
import json
import os

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=200)
    a = ap.parse_args()
    meta = json.load(open(os.path.join(a.src, "meta.json")))
    W, H, N = meta["width"], meta["height"], meta["frames"]
    idx = sorted(set(int(round(x)) for x in np.linspace(0, N - 1, a.frames)))
    raw = np.memmap(os.path.join(a.src, meta["depth_file"]), dtype="<u2", mode="r", shape=(N, H, W))
    os.makedirs(a.out, exist_ok=False)
    sub = np.ascontiguousarray(raw[idx])
    path = os.path.join(a.out, meta["depth_file"])
    sub.tofile(path)
    out = dict(meta)
    out["frames"] = len(idx)
    for k, v in meta.items():
        if isinstance(v, list) and len(v) == N:
            out[k] = [v[i] for i in idx]
    out["depth_sha256"] = hashlib.sha256(open(path, "rb").read()).hexdigest()
    out["subset_of"] = {"src": a.src, "src_depth_sha256": meta.get("depth_sha256"), "indices": idx,
                        "rule": "round(linspace(0, N-1, %d))" % a.frames}
    json.dump(out, open(os.path.join(a.out, "meta.json"), "w"), indent=1)
    print("%s: %d frames, depth sha256 %s" % (a.out, len(idx), out["depth_sha256"]))


if __name__ == "__main__":
    main()
