#!/usr/bin/env python3
"""T2 (2026-10-16): serialized size of the camera's image messages, measured on the wire format (CDR bytes).

Runs INSIDE the stack container after the criteria window (it is a new DDS participant: never inside the window).
  msg_probe.py --out OUT.json [--count 10] [--timeout-s 20]
Best effort KEEP_LAST 5 raw subscriptions (raw=True: the callback gets the serialized bytes, nothing is decoded on
the hot path) to the depth and colour images; one message of each is also decoded for width/height/step/encoding.
Records len(serialized) for the first --count messages per topic, and the Fast DDS shared-memory payload limit of
the default segment (512 KiB = 524288 B; segment file 549408 B) for comparison.
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

TOPICS = {"depth": "/camera/depth/image_raw", "color": "/camera/color/image_raw"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--count", type=int, default=10)
    ap.add_argument("--timeout-s", type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    node = Node("t2_msg_probe")
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    got = {k: [] for k in TOPICS}
    meta = {}

    def mk(k):
        def cb(raw):
            if len(got[k]) < a.count:
                got[k].append(len(raw))
            if k not in meta:
                m = deserialize_message(raw, Image)
                meta[k] = {"width": m.width, "height": m.height, "step": m.step, "encoding": m.encoding,
                           "data_bytes": len(m.data), "frame_id": m.header.frame_id}
        return cb
    for k, t in TOPICS.items():
        node.create_subscription(Image, t, mk(k), qos, raw=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.timeout_s and any(len(v) < a.count for v in got.values()):
        rclpy.spin_once(node, timeout_sec=0.2)
    res = {"topics": TOPICS, "serialized_bytes": got, "decoded": meta,
           "default_shm_payload_bytes": 524288, "default_shm_segment_file_bytes": 549408,
           "exceeds_default_payload": {k: (max(v) > 524288) if v else None for k, v in got.items()}}
    with open(a.out, "w") as fh:
        json.dump(res, fh, indent=1)
    print(json.dumps(res))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if all(got.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
