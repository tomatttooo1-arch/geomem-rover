#!/usr/bin/env python3
"""Turn a nano_loop recording (rec30) into the shared input of all three arms.

Reads ``frames.jsonl`` + ``depth/*.png`` (uint16 mm, hardware D2C aligned to
colour) + ``calibration.json`` and writes, into a fresh directory:

  depth_u16.bin   N x H x W little-endian uint16, frames in frames.jsonl order
  meta.json       size, K, the float32 inverse K (hex, exactly as FrameNativeM7
                  computes it), grid, pose (identity: fixed camera), per-frame
                  identity (color_index, acquired_ns), sha256 of every input

Only this pair is needed by the arms, so it is also what goes to the KV260.
Usage: prep_rec.py --rec <rec dir> --out <fresh dir>
"""
import argparse
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.geo_same_input.common import (DEPTH_UNIT_M, GRID_ORIGIN_M, GRID_SHAPE,  # noqa: E402
                                         GRID_VOXEL_M, f32_hex, inverse_k_f32, sha256_file)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rec", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    import cv2
    os.makedirs(a.out, exist_ok=False)
    calib_path = os.path.join(a.rec, "calibration.json")
    calib = json.load(open(calib_path))
    frames = [json.loads(line) for line in open(os.path.join(a.rec, "frames.jsonl"))]
    w, h = int(calib["width"]), int(calib["height"])
    out_bin = os.path.join(a.out, "depth_u16.bin")
    valid = []
    with open(out_bin, "wb") as fp:
        for m in frames:
            d = cv2.imread(os.path.join(a.rec, m["depth_png"]), cv2.IMREAD_UNCHANGED)
            if d is None or d.dtype != np.uint16 or d.shape != (h, w):
                raise ValueError("bad depth png %s" % m["depth_png"])
            valid.append(float((d > 0).mean()))
            fp.write(np.ascontiguousarray(d, dtype="<u2").tobytes())
    K = np.asarray(calib["K"], dtype=np.float64)
    meta = {
        "source_rec": os.path.abspath(a.rec),
        "manifest_sha256": sha256_file(os.path.join(a.rec, "MANIFEST.sha256"))
        if os.path.exists(os.path.join(a.rec, "MANIFEST.sha256")) else None,
        "calibration_sha256_file": sha256_file(calib_path),
        "calibration_id": calib["calibration_id"],
        "width": w, "height": h, "frames": len(frames),
        "depth_file": "depth_u16.bin", "depth_sha256": sha256_file(out_bin),
        "depth_unit_m": DEPTH_UNIT_M,
        "K": K.tolist(), "inverse_k_f32_hex": f32_hex(inverse_k_f32(K)),
        "camera_to_local": np.eye(4).tolist(), "pose": "fixed_camera_identity",
        "grid": {"origin_m": list(GRID_ORIGIN_M), "voxel_m": GRID_VOXEL_M, "shape_xyz": list(GRID_SHAPE)},
        "color_index": [int(m["color_index"]) for m in frames],
        "acquired_ns": [int(m["acquired_ns"]) for m in frames],
        "valid_depth_fraction": {"min": round(min(valid), 4), "median": round(float(np.median(valid)), 4),
                                 "max": round(max(valid), 4)},
    }
    json.dump(meta, open(os.path.join(a.out, "meta.json"), "w"), indent=1)
    print(json.dumps({k: meta[k] for k in ("frames", "width", "height", "depth_sha256",
                                           "inverse_k_f32_hex", "valid_depth_fraction")}))


if __name__ == "__main__":
    main()
