#!/usr/bin/env python3
"""Every velocity command, in a process of its own (docs/design/NAV_COLDSTART_CRAWL_20261017.md C5).

m2-m4: the driver recorded /cmd_vel in the same executor as the large /evaluation messages; its receipt backlog
reached 1-2 s and /cmd_vel arrived at ~10 Hz where the velocity smoother publishes at 20 Hz, so "every message"
(C5) was checked on a subset. This node only takes the three velocity topics (depth 200) and writes
OUT/cmd_all.jsonl rows {"t": RAW receipt, "topic", "lx", "ly", "az", "n": per-topic receipt count}.
Stops on SIGTERM / SIGINT (a plain flag). Python 3.8+.
"""
import argparse
import json
import os
import signal
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from host.fpga_clock import now_ns

    rclpy.init()
    node = Node("fpga_crawl_cmd_recorder")
    f = open(os.path.join(a.out, "cmd_all.jsonl"), "w")
    n = {}

    def cb(topic):
        def _cb(m):
            n[topic] = n.get(topic, 0) + 1
            f.write(json.dumps({"t": now_ns(), "topic": topic, "lx": m.linear.x, "ly": m.linear.y,
                                "az": m.angular.z, "n": n[topic]}) + "\n")
        return _cb

    for t in ("/cmd_vel_nav", "/cmd_vel", "/fpga/cmd_vel_safe"):
        node.create_subscription(Twist, t, cb(t), 200)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    while rclpy.ok() and not stop[0]:
        rclpy.spin_once(node, timeout_sec=0.05)
    f.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
