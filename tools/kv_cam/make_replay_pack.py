#!/usr/bin/env python3
"""Build a kv_cam replay pack (KVCAMPK1, layout in kv_cam_common.h / protocol.md) from recorded frames.

Input: a recording directory with frames.jsonl rows {"color_png", "depth_png"} (e.g. the rec30 set
shared/artifacts/20260927-dev1-rec30/rec: colour 640x360 RGB PNG, depth 640x360 u16 PNG). Frames are letterboxed
into 640x480 (rows padded top/bottom; colour pad = YUYV black Y=16 U=V=128, depth pad = 0), colour converted
RGB -> YUYV with BT.601 limited-range integer coefficients (U/V averaged over each pixel pair).
The conversion only has to be deterministic: the pack is a replay source for the shared-memory protocol test,
not a colour reference for the PL kernel.

    python3 make_replay_pack.py --rec DIR --out pack.bin [--frames N]
"""
import argparse
import hashlib
import json
import os
import struct

import numpy as np

MAGIC = 0x314b504d4143564b
YUYV = int.from_bytes(b"YUYV", "little")
Y16 = int.from_bytes(b"Y16 ", "little")
W, H = 640, 480


def rgb_to_yuyv(rgb):
    r, g, b = (rgb[..., i].astype(np.int32) for i in range(3))
    y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16
    u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128
    v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128
    u2 = (u[:, 0::2] + u[:, 1::2] + 1) >> 1
    v2 = (v[:, 0::2] + v[:, 1::2] + 1) >> 1
    out = np.empty((rgb.shape[0], rgb.shape[1], 2), np.uint8)
    out[:, :, 0] = np.clip(y, 0, 255)
    out[:, 0::2, 1] = np.clip(u2, 0, 255)
    out[:, 1::2, 1] = np.clip(v2, 0, 255)
    return out


def letterbox(img, fill):
    h = img.shape[0]
    top = (H - h) // 2
    out = np.empty((H,) + img.shape[1:], img.dtype)
    out[...] = fill
    out[top:top + h] = img
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=0)
    a = ap.parse_args()
    import cv2  # host only
    rows = [json.loads(l) for l in open(os.path.join(a.rec, "frames.jsonl")) if l.strip()]
    if a.frames:
        rows = rows[:a.frames]
    cb, db = W * H * 2, W * H * 2
    with open(a.out, "wb") as f:
        f.write(struct.pack("<QII8I", MAGIC, 1, len(rows), W, H, YUYV, cb, W, H, Y16, db) + bytes(16))
        for i, r in enumerate(rows):
            bgr = cv2.imread(os.path.join(a.rec, r["color_png"]), cv2.IMREAD_COLOR)
            dep = cv2.imread(os.path.join(a.rec, r["depth_png"]), cv2.IMREAD_UNCHANGED)
            assert bgr is not None and dep is not None and dep.dtype == np.uint16 and bgr.shape[1] == W
            yuyv = rgb_to_yuyv(bgr[..., ::-1])
            pad = np.zeros((1, 1, 2), np.uint8)
            pad[..., 0], pad[..., 1] = 16, 128
            yuyv = letterbox(yuyv, pad)
            dep = letterbox(dep, 0)
            f.write(struct.pack("<QQQQ", i, int(r.get("dev_ts_color_us", 0)), int(r.get("dev_ts_depth_us", 0)), 0))
            f.write(np.ascontiguousarray(yuyv).tobytes())
            f.write(np.ascontiguousarray(dep.astype("<u2")).tobytes())
    sha = hashlib.sha256(open(a.out, "rb").read()).hexdigest()
    print(json.dumps({"out": a.out, "frames": len(rows), "bytes": os.path.getsize(a.out), "sha256": sha}))


if __name__ == "__main__":
    main()
