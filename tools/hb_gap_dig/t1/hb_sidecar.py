#!/usr/bin/env python3
"""T1 (2026-10-16) side receiver: per-frame over-range statistics of the depth stream (record only).

Runs INSIDE the car stack container (same DDS environment), started by docker exec, never supervised by car_inner.sh.
Subscribes --topic (sensor_msgs/Image 16UC1) best effort KEEP_LAST 1. The over-range test is bit-identical to the
producer's (host/m7_pipeline.py:331 on host/kv_link.codes_to_m): float32(code) * float32(0.001) > 20, code 0 = NaN.
Writes OUT/side_frames.csv (every frame: stamp_ns, recv_raw_ns, max_code, n_over, n_zero) and
OUT/side_over.jsonl (frames with n_over > 0: raw-code histogram, 4x4 block counts, row/col histograms, the
8-neighbourhood validity of the over-range pixels, isolated count, neighbour depth median/p90, frame p99/p999).
An observer: it adds one best-effort reader to the driver's depth writer. Stops on SIGTERM/SIGINT.
"""
import argparse
import json
import os
import signal
import sys
import time

import numpy as np


def over_threshold_code(unit=0.001, limit=20.0):
    u = np.float32(unit)
    for c in range(int(limit / unit) - 50, int(limit / unit) + 50):
        if np.float32(c) * u > limit:
            return c
    raise RuntimeError("no threshold")


def neighbours(mask):
    """count of True 8-neighbours for each pixel (zero padding)."""
    m = mask.astype(np.uint8)
    p = np.pad(m, 1)
    h, w = m.shape
    s = np.zeros((h, w), np.uint8)
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            if dy == 1 and dx == 1:
                continue
            s += p[dy:dy + h, dx:dx + w]
    return s


def detail(codes, over, thr, unit):
    h, w = codes.shape
    valid = (codes > 0) & ~over
    ys, xs = np.nonzero(over)
    vals, cnts = np.unique(codes[over], return_counts=True)
    order = np.argsort(-cnts)[:20]
    blk = np.zeros((4, 4), int)
    np.add.at(blk, (ys * 4 // h, xs * 4 // w), 1)
    rowh = np.bincount(ys * 20 // h, minlength=20)
    colh = np.bincount(xs * 32 // w, minlength=32)
    nv = neighbours(valid)[over]           # valid neighbours of each over pixel
    no = neighbours(over)[over]            # over neighbours of each over pixel
    # valid depth in the 8-neighbourhood of over pixels
    ring = (neighbours(over) > 0) & valid
    rd = codes[ring].astype(np.float32) * np.float32(unit)
    vd = codes[valid].astype(np.float32) * np.float32(unit)
    return {
        "raw_code_top": [[int(vals[i]), int(cnts[i])] for i in order], "n_distinct_codes": int(len(vals)),
        "block4x4": blk.tolist(), "row_hist20": rowh.tolist(), "col_hist32": colh.tolist(),
        "y_range": [int(ys.min()), int(ys.max())], "x_range": [int(xs.min()), int(xs.max())],
        "share_with_valid_neighbour": round(float((nv > 0).mean()), 4),
        "mean_valid_neighbours_of_8": round(float(nv.mean()), 3),
        "isolated": int((no == 0).sum()),
        "ring_valid_px": int(ring.sum()),
        "ring_depth_m_p50": round(float(np.median(rd)), 3) if rd.size else None,
        "ring_depth_m_p90": round(float(np.percentile(rd, 90)), 3) if rd.size else None,
        "frame_valid_m_p99": round(float(np.percentile(vd, 99)), 3) if vd.size else None,
        "frame_valid_m_p999": round(float(np.percentile(vd, 99.9)), 3) if vd.size else None,
        "frame_valid_share": round(float(valid.mean()), 4),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/depth/image_raw")
    ap.add_argument("--out", required=True)
    ap.add_argument("--unit", type=float, default=0.001)
    a = ap.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    os.makedirs(a.out, exist_ok=True)
    thr = over_threshold_code(a.unit)
    open(os.path.join(a.out, "sidecar.pid"), "w").write("%d\n" % os.getpid())
    fr = open(os.path.join(a.out, "side_frames.csv"), "a", buffering=1)
    if fr.tell() == 0:
        fr.write("stamp_ns,recv_raw_ns,max_code,n_over,n_zero\n")
    ov = open(os.path.join(a.out, "side_over.jsonl"), "a", buffering=1)
    hb = open(os.path.join(a.out, "side_heartbeat.jsonl"), "a", buffering=1)
    hb.write(json.dumps({"event": "start", "wall": time.time(), "raw_ns": time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                         "over_code_min": thr, "pid": os.getpid()}) + "\n")
    st = {"n": 0, "over_frames": 0, "stop": False, "bad": 0}

    def cb(msg):
        t = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if msg.encoding != "16UC1":
            st["bad"] += 1
            return
        codes = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
        over = codes >= thr
        n_over = int(over.sum())
        mx = int(codes.max())
        fr.write("%d,%d,%d,%d,%d\n" % (stamp, t, mx, n_over, int((codes == 0).sum())))
        st["n"] += 1
        if n_over:
            st["over_frames"] += 1
            d = {"stamp_ns": stamp, "recv_raw_ns": t, "max_code": mx, "nanmax_m": float(np.float32(mx) * np.float32(a.unit)),
                 "n_over": n_over, "shape": [int(msg.height), int(msg.width)]}
            d.update(detail(codes, over, thr, a.unit))
            ov.write(json.dumps(d) + "\n")

    signal.signal(signal.SIGTERM, lambda *_: st.__setitem__("stop", True))
    signal.signal(signal.SIGINT, lambda *_: st.__setitem__("stop", True))
    rclpy.init(args=None)
    node = rclpy.create_node("fpga_t1_sidecar")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(Image, a.topic, cb, qos)
    last = time.monotonic()
    try:
        while not st["stop"]:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() - last >= 10:
                last = time.monotonic()
                hb.write(json.dumps({"wall": time.time(), "raw_ns": time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                                     "frames": st["n"], "over_frames": st["over_frames"], "bad": st["bad"]}) + "\n")
    finally:
        hb.write(json.dumps({"event": "stop", "wall": time.time(), "frames": st["n"], "over_frames": st["over_frames"]}) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
