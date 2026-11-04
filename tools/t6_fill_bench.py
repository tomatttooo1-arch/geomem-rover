#!/usr/bin/env python3
"""CPU cost of the AR sender's per-frame Python work on the KV260, kv_server stopped (no GIL contention):
fill_ar_inputs (depth chain at stretch k=6, grid, boxes) and encode_ar_message, on replay depth frames of the
prep (640x360) with a 32^3 flag grid; colour-main adds the 640x360 RGB888. Copied from relay 5
(runs/ar-screen-20261016-relay5/tools/t6_fill_bench.py, section 12.9) with the measured window unchanged;
2026-10-16 (section 13) adds the arm: python = fill_ar_inputs() as before, native = fill_ar_inputs(native=lib)
with tools/kv_link/ar_helper/m7ar_fill.cpp compiled here by the board's g++.
t6_fill_bench.py <repo> <prep dir> [n] [--arms python,native] [--modes off,main]"""
import argparse
import json
import os
import sys
import tempfile
import time

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("repo")
ap.add_argument("prep")
ap.add_argument("n", nargs="?", type=int, default=40)
ap.add_argument("--arms", default="python,native")
ap.add_argument("--modes", default="off,main")
a = ap.parse_args()
sys.path.insert(0, a.repo)
from host import kv_ar  # noqa: E402
from host.kv_link import f32_bits  # noqa: E402
W, H = 640, 360
raw = np.fromfile(os.path.join(a.prep, "depth_u16.bin"), dtype=np.uint16, count=W * H * 20).reshape(20, W * H)
rgb = np.fromfile(os.path.join(a.prep, "colour_rgb8.bin"), dtype=np.uint8, count=W * H * 3).tobytes()
lay = kv_ar.ar_layout(1024, 600, W, H)
clay = kv_ar.ar_colour_layout(1024, 600, W, H)
flags = np.random.RandomState(0).randint(0, 3, 32 ** 3).astype(np.uint8)


def snap(i):
    return {"frame_id": i + 1, "codes": raw[i % 20], "unit_bits": f32_bits(0.001), "width": W, "height": H,
            "nx": 32, "ny": 32, "nz": 32, "origin": [-0.8, -0.8, -0.05], "voxel": 0.05,
            "world_camera": np.eye(4)[:3], "flags": flags, "geometry_status": 0, "roi": None, "roi_status": None,
            "roi_rows": [], "nano_elapsed_ns": 5000000, "labels": None, "replay": True,
            "first_mono_ns": 0, "capture_ns": 0, "sensor_stamp_ns": None}


libs = {"python": None}
if "native" in a.arms.split(","):
    from host.kv_ar_native import ArFillNative
    libs["native"] = ArFillNative.build(tempfile.mkdtemp(prefix="t6_fill_"))
res = {"n": a.n, "fill": libs["native"].describe() if libs.get("native") else None}
for arm in a.arms.split(","):
    for mode in a.modes.split(","):
        fill, enc, depths = [], [], []
        for i in range(a.n):
            s = snap(i)
            t0 = time.monotonic()
            inp = kv_ar.fill_ar_inputs(s, clay if mode == "main" else lay, source="replay", depth_stretch=6.0,
                                       native=libs[arm])
            if mode == "main":
                inp["colour"] = {"pip": (clay["pip_x"], clay["pip_y"], clay["pip_div"]), "rgb": rgb, "width": W,
                                 "height": H, "held": None, "source_key": 1}
            t1 = time.monotonic()
            msg = kv_ar.encode_ar_message(inp)
            t2 = time.monotonic()
            fill.append((t1 - t0) * 1e3)
            enc.append((t2 - t1) * 1e3)
            if i < 20:
                depths.append(inp["depth"].tobytes())
        fill.sort()
        enc.sort()
        import hashlib
        res["%s_%s" % (arm, mode)] = {
            "fill_ms_p50": round(fill[a.n // 2], 3), "fill_ms_p95": round(fill[int(a.n * .95)], 3),
            "fill_ms_max": round(fill[-1], 3), "fill_over_100ms": sum(1 for v in fill if v > 100),
            "encode_ms_p50": round(enc[a.n // 2], 3), "encode_ms_p95": round(enc[int(a.n * .95)], 3),
            "msg_bytes": len(msg), "depth20_sha256": hashlib.sha256(b"".join(depths)).hexdigest()}
print(json.dumps(res))
