#!/usr/bin/env python3
"""Operator confirmation that the start area in front of the car is clear.

Second owner ruling on D-035 item 6 (2026-09-27), design doc section 4.2.
Use it once, at a cold start, after LOOKING at the floor in front of the car:

    python3 tools/cold_start_confirm.py --operator dev2

The bridge logs every attempt (operator, operator's clock, bridge clock, car
pose) before acting, accepts only the first one, never clears an obstacle, and
withdraws the temporary free ring after the car has moved further than its
width or after 30 s. Exit code 0 = accepted, 1 = refused, 2 = no answer.
"""
import argparse
from datetime import datetime, timezone
import json
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--operator", required=True, help="who looked at the start area")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()
    if not args.operator.strip():
        ap.error("--operator must not be empty")
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node("fpga_cold_start_confirm")
    answers = []
    node.create_subscription(String, "/fpga/rolling_map/cold_start_result",
                             lambda m: answers.append(json.loads(m.data)), 10)
    pub = node.create_publisher(String, "/fpga/rolling_map/cold_start_confirm", 10)
    stamp = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
    request = {"operator": args.operator, "operator_stamp": stamp}
    end = time.monotonic() + args.timeout
    # wait for the bridge to be connected, then send exactly one request
    while time.monotonic() < end and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
    pub.publish(String(data=json.dumps(request)))
    while time.monotonic() < end and not any(a.get("request") == request for a in answers):
        rclpy.spin_once(node, timeout_sec=0.05)
    mine = [a for a in answers if a.get("request") == request]
    node.destroy_node()
    rclpy.shutdown()
    if not mine:
        print(json.dumps({"accepted": False, "reason": "no_answer_from_bridge",
                          "request": request}))
        return 2
    print(json.dumps(mine[0], ensure_ascii=False))
    return 0 if mine[0]["accepted"] else 1


if __name__ == "__main__":
    sys.exit(main())
