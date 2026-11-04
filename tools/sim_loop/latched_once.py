#!/usr/bin/env python3
"""Line D2 (2026-10-14): write the first message of a latched std_msgs/String topic (TRANSIENT_LOCAL,
reliable; e.g. /fpga/runtime_info) to a file and exit; exit 1 after --timeout-s without one.

  latched_once.py --topic /fpga/runtime_info --out PATH [--timeout-s 120]
"""
import argparse
import sys
import time


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--topic', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout-s', type=float, default=120.)
    a = ap.parse_args(argv)
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('fpga_latched_once')
    got = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(String, a.topic, lambda m: got.append(m.data), qos)
    deadline = time.monotonic() + a.timeout_s
    try:
        while not got and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    if got:
        with open(a.out, 'x') as f:
            f.write(got[0] + '\n')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if got else 1


if __name__ == '__main__':
    sys.exit(main())
