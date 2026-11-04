#!/usr/bin/env python3
"""Passive simulator motion evidence; never publishes commands or permits.

Line D2 (2026-10-14; defaults unchanged): --odom-period-s / --clock-period-s (default 0.1 s each; 0 =
every message) for stop-latency timestamps, --twist-topic T (repeatable) records extra Twist topics,
e.g. the MCU stand-in's executed speed (tools/chassis_uart/mcu_sim.py --twist-topic)."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--odom-period-s', type=float, default=.1)
    parser.add_argument('--clock-period-s', type=float, default=.1)
    parser.add_argument('--twist-topic', action='append', default=[])
    args = parser.parse_args()
    period = {'/odom': args.odom_period_s, '/clock': args.clock_period_s}
    rclpy.init()
    node = rclpy.create_node('fpga_passive_motion_recorder')
    stream = args.out.open('x')
    previous = {}

    def record(topic, msg):
        now = time.monotonic()
        if topic in period and now - previous.get(topic, 0) < period[topic]:
            return
        previous[topic] = now
        stream.write(json.dumps({'wall_monotonic': now, 'wall_ns': time.time_ns(), 'topic': topic,
                                 'message': message_to_ordereddict(msg)}) + '\n')
        stream.flush()

    for topic, kind in [('/odom', Odometry), ('/clock', Clock), ('/cmd_vel', Twist),
                        ('/fpga/cmd_vel_safe', Twist), ('/fpga/motion_permit', Bool),
                        # line D1: the simulated hardware stop (scripts/sim_estop_gate.py)
                        ('/fpga/sim_estop', Bool)] + [(t, Twist) for t in args.twist_topic]:
        node.create_subscription(kind, topic, lambda msg, t=topic: record(t, msg), qos_profile_sensor_data)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
