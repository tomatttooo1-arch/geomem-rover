#!/usr/bin/env python3
"""DPUPIPE0 host/board bench sender: host/kv_colour.ColourSource (the product sender, DPU answers on)
fed by a fake camera that puts frames into a ColourRing at --fps (default 15, the car camera rate).
No ROS, no camera. Frames: --images (PNG/JPG, RGB 640x480 recommended) cycled, or a synthetic gradient.
Each frame's key = time.monotonic_ns() at "capture" (same clock as the C++ receiver on one host).

  python3 bench_tx.py --kv 127.0.0.1:47900 --seconds 30 --out tx.json --keys keys.jsonl [--images a.png ...]
--keys writes {source_key, frame_index} per captured frame so a test can recompute the expected tensor.
"""
import argparse
import json
import os
import resource
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from host.kv_colour import ColourRing, ColourSource  # noqa: E402


def load_frames(paths, n_synth=8):
    import numpy as np
    if not paths:
        y, x = np.mgrid[0:480, 0:640]
        return [np.stack([(x + 7 * k) % 256, (y + 3 * k) % 256, ((x ^ y) + k) % 256], -1).astype(np.uint8)
                for k in range(n_synth)]
    import cv2
    out = []
    for p in paths:
        im = cv2.imread(p, cv2.IMREAD_COLOR)
        if im is None:
            raise SystemExit("cannot read %s" % p)
        out.append(im[..., ::-1].copy())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kv", action="append", required=True)
    ap.add_argument("--bind", default=None, help="host:port to bind the sender socket (default ephemeral)")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--backend", default="auto", choices=("auto", "cv2", "numpy"))
    ap.add_argument("--images", nargs="*", default=[])
    ap.add_argument("--out", default=None)
    ap.add_argument("--keys", default=None)
    ap.add_argument("--warm", default="640:6,512:7,416:7", help="size:fix_point to prepare before start")
    a = ap.parse_args()
    frames = load_frames(a.images)
    kvs = [(h, int(p)) for h, p in (k.rsplit(":", 1) for k in a.kv)]
    ring = ColourRing(12)
    bind = None
    if a.bind:
        h, p = a.bind.rsplit(":", 1)
        bind = (h, int(p))
    src = ColourSource(kvs, ring.lookup, info={"source": "bench_tx"}, latest=ring.newest, hello_s=0.2,
                       dpu_backend=a.backend, bind=bind)
    warm = {}
    for item in filter(None, a.warm.split(",")):
        sz, fp = item.split(":")
        warm[item] = src.warm_dpu(int(sz), int(fp), frames[0].shape[1], frames[0].shape[0])
    keys = open(a.keys, "w") if a.keys else None
    stop = threading.Event()

    def camera():
        period, k, t_next = 1.0 / a.fps, 0, time.monotonic()
        while not stop.is_set():
            idx = k % len(frames)
            key = time.monotonic_ns()
            f = frames[idx]
            ring.put(key, f.shape[1], f.shape[0], f.tobytes())
            if keys:
                keys.write(json.dumps({"source_key": key, "frame_index": idx}) + "\n")
            k += 1
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()))
    r0, t0 = resource.getrusage(resource.RUSAGE_SELF), time.monotonic()
    src.start()
    cam = threading.Thread(target=camera, daemon=True)
    cam.start()
    try:
        time.sleep(a.seconds)
    except KeyboardInterrupt:
        pass
    stop.set()
    cam.join(1.0)
    src.stop()
    r1, t1 = resource.getrusage(resource.RUSAGE_SELF), time.monotonic()
    if keys:
        keys.close()
    cpu = (r1.ru_utime - r0.ru_utime) + (r1.ru_stime - r0.ru_stime)
    d = src.describe()
    sent = d["counts"].get("dpu_frames_sent", 0)
    out = {"tx": d, "frames": len(frames), "run_s": round(t1 - t0, 2), "process_cpu_s": round(cpu, 3),
           "process_cpu_ms_per_dpu_frame": round(1e3 * cpu / sent, 3) if sent else None,
           "prep_ms_p50": d.get("dpu_prep_ms_p50"), "ring": dict(ring.counts), "warm_backend": warm}
    print(json.dumps(out), flush=True)
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
