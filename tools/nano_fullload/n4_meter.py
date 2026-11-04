#!/usr/bin/env python3
"""N4 measurement node (inside the ROS container). docs/design/NANO_FULLLOAD_20260927.md §2-3.

- colour / depth image: one row per message (header stamp, receipt time); colour frames are also
  written to the shared-memory ring for the host-side detector (tools/nano_fullload/shm_ring.py);
- /vo/odom_info (rtabmap OdomInfo) and /vo/info (rtabmap Info): per message timing rows.
Rows are compact CSV-like JSON lists to keep the files small. Stops on SIGINT/SIGTERM.
"""
import argparse
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--ring", required=True)
    a = ap.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from shm_ring import RingWriter

    rclpy.init()
    node = Node("fpga_n4_meter")
    os.makedirs(a.out, exist_ok=True)
    f_img = open(os.path.join(a.out, "images.jsonl"), "w", buffering=1 << 16)
    f_rt = open(os.path.join(a.out, "rtabmap.jsonl"), "w", buffering=1)
    ring = {"w": None}

    def stamp_ns(h):
        return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec

    def on_color(msg):
        rx_mono, rx = time.monotonic_ns(), time.time_ns()
        if ring["w"] is None:
            ring["w"] = RingWriter(a.ring, msg.width, msg.height, 3)
            f_img.write(json.dumps({"color": [msg.width, msg.height, msg.encoding, msg.header.frame_id]}) + "\n")
        ring["w"].write(msg.data, stamp_ns(msg.header), rx)
        f_img.write('["c",%d,%d,%d]\n' % (stamp_ns(msg.header), rx, rx_mono))

    def on_depth(msg):
        rx_mono, rx = time.monotonic_ns(), time.time_ns()
        f_img.write('["d",%d,%d,%d]\n' % (stamp_ns(msg.header), rx, rx_mono))

    node.create_subscription(Image, "/camera/color/image_raw", on_color, qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/depth/image_raw", on_depth, qos_profile_sensor_data)

    try:
        from rtabmap_msgs.msg import Info, OdomInfo

        def on_odom_info(m):
            f_rt.write(json.dumps(["o", stamp_ns(m.header), time.time_ns(), bool(m.lost), m.inliers,
                                   m.features, round(float(m.time_estimation) * 1e3, 3)]) + "\n")

        def on_info(m):
            st = dict(zip(m.stats_keys, m.stats_values))
            f_rt.write(json.dumps(["i", stamp_ns(m.header), time.time_ns(), m.ref_id, m.loop_closure_id,
                                   st.get("Timing/Total/ms"), st.get("Memory/Working_memory_size/")]) + "\n")
        node.create_subscription(OdomInfo, "/vo/odom_info", on_odom_info, 50)
        node.create_subscription(Info, "/vo/info", on_info, 10)
    except ImportError as exc:  # noqa: F841 - image-only metering still useful
        f_rt.write(json.dumps(["no_rtabmap_msgs", str(exc)]) + "\n")

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        f_img.close(); f_rt.close()


if __name__ == "__main__":
    main()
