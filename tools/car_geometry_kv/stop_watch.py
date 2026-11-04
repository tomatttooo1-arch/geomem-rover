#!/usr/bin/env python3
"""Records every /fpga/motion_permit (Bool) and /cmd_vel (Twist) message with its
CLOCK_MONOTONIC_RAW arrival time, one json line each, until SIGTERM/SIGINT. Used by
ros_arm.sh (WATCH_STOP=1) to show that the runtime's stop() messages (permit false, zero
Twist) go out when it is stopped with SIGINT (docs/design/KV_LINK_HELLO_FIX_20261011.md 7).
"""
import argparse
import json
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node("fpga_stop_watch")
    out = open(a.out, "w")

    def rec(topic, data):
        out.write(json.dumps({"raw_ns": time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                              "topic": topic, "data": data}) + "\n")
        out.flush()
    node.create_subscription(Bool, "/fpga/motion_permit", lambda m: rec("/fpga/motion_permit", bool(m.data)), 10)
    node.create_subscription(Twist, "/cmd_vel", lambda m: rec("/cmd_vel", [m.linear.x, m.angular.z]), 10)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except BaseException:       # KeyboardInterrupt / rclpy ExternalShutdownException: just stop
        pass
    out.close()
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
