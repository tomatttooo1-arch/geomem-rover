#!/usr/bin/env python3
"""Add the D2C-aligned colour of a recording to an existing prep dir (AR colour replay, 2026-10-16).

Reads ``frames.jsonl`` + ``color/*.png`` of the recording (the same rec that prep_rec.py read; the
recording's calibration says align hw_d2c_depth_to_color, so colour pixel (u, v) = depth pixel (u, v))
and writes, next to the prep's depth file, without touching its meta.json:
  colour_rgb8.bin   N x H x W x 3 uint8 RGB, frames in frames.jsonl order (= depth_u16.bin order)
  colour.json       {"colour_file", "colour_sha256", "width", "height", "frames", "source_rec",
                     "color_png": [...] (first and last), "depth_sha256" of the prep it belongs to}
Refuses when the prep's frame count / size / source recording differ.
Usage: prep_colour.py --rec REC_DIR --prep PREP_DIR
"""
import argparse
import hashlib
import json
import os

import numpy as np


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fp:
        for blk in iter(lambda: fp.read(1 << 20), b""):
            h.update(blk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rec", required=True)
    ap.add_argument("--prep", required=True)
    a = ap.parse_args()
    from PIL import Image
    meta = json.load(open(os.path.join(a.prep, "meta.json")))
    frames = [json.loads(line) for line in open(os.path.join(a.rec, "frames.jsonl"))]
    w, h, n = int(meta["width"]), int(meta["height"]), int(meta["frames"])
    if len(frames) != n:
        raise SystemExit("recording has %d frames, prep %d" % (len(frames), n))
    if meta.get("source_rec") and os.path.realpath(meta["source_rec"]) != os.path.realpath(a.rec):
        raise SystemExit("prep was made from %s, not %s" % (meta["source_rec"], a.rec))
    if meta.get("color_index") and [int(m["color_index"]) for m in frames] != meta["color_index"]:
        raise SystemExit("frames.jsonl color_index differs from the prep's")
    out_bin = os.path.join(a.prep, "colour_rgb8.bin")
    if os.path.exists(out_bin):
        raise SystemExit("%s exists" % out_bin)
    with open(out_bin, "wb") as fp:
        for m in frames:
            img = np.asarray(Image.open(os.path.join(a.rec, m["color_png"])).convert("RGB"))
            if img.shape != (h, w, 3):
                raise SystemExit("colour %s is %s, depth is %dx%d" % (m["color_png"], img.shape, w, h))
            fp.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())
    cmeta = {"colour_file": "colour_rgb8.bin", "colour_sha256": sha256_file(out_bin), "width": w,
             "height": h, "frames": n, "format": "rgb8", "source_rec": os.path.abspath(a.rec),
             "color_png": [frames[0]["color_png"], frames[-1]["color_png"]],
             "depth_sha256": meta.get("depth_sha256")}
    json.dump(cmeta, open(os.path.join(a.prep, "colour.json"), "w"), indent=1)
    print(json.dumps(cmeta))


if __name__ == "__main__":
    main()
