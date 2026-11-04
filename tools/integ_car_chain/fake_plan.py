#!/usr/bin/env python3
"""SYNTHETIC /plan publisher for host smoke runs (NOT Nav2): nav_msgs/Path at a fixed rate.

Only so that the hub's STATE nav.path has data in a chain without a planner. The path is a fixed
polyline in frame ``map`` (default 150 poses, 1.6-2 cm apart): a straight run along +x, a quarter arc
(radius 0.5 m) and a straight run along +y -- the hub's polyline simplification then keeps the two
straight ends and a handful of arc points. Every message carries the current ROS time as stamp.
Started by tools/integ_car_chain/nano_chain_inner_v2.sh with FAKE_PLAN=1 (off by default), before
the rig, because no new ROS process is started while the chain runs. Stops on SIGINT/SIGTERM or
after --duration-s.

Usage: fake_plan.py [--hz 1] [--poses 150] [--frame-id map] [--topic /plan] [--duration-s 0]
"""
import argparse
import math
import signal
import sys
import time


def polyline(n, step=0.02, radius=0.5):
    """n points: third straight +x (step apart), third quarter arc, third straight +y (step apart)."""
    a = n // 3
    c = n - 2 * a
    pts = [(i * step, 0.0) for i in range(a)]
    x0 = (a - 1) * step if a else 0.0
    cx, cy = x0, radius                       # arc centre: left of the end of the first run
    dth = (math.pi / 2) / c                   # exactly a quarter turn over the c arc points
    for i in range(1, c + 1):
        th = -math.pi / 2 + i * dth
        pts.append((cx + radius * math.cos(th), cy + radius * math.sin(th)))
    ex, ey = pts[-1]
    for i in range(1, a + 1):
        pts.append((ex, ey + i * step))
    return pts[:n]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--hz", type=float, default=1.0)
    ap.add_argument("--poses", type=int, default=150)
    ap.add_argument("--frame-id", default="map")
    ap.add_argument("--topic", default="/plan")
    ap.add_argument("--duration-s", type=float, default=0.0, help="0 = until signalled")
    args = ap.parse_args(argv)

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path

    rclpy.init()
    node = rclpy.create_node("fpga_fake_plan")
    pub = node.create_publisher(Path, args.topic, 1)
    pts = polyline(args.poses)
    stop = [False]

    def on_signal(signum, frame):
        stop[0] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    sent = 0
    t_end = time.monotonic() + args.duration_s if args.duration_s > 0 else None
    period = 1.0 / args.hz
    nxt = time.monotonic()
    print('{"fake_plan": "synthetic, not Nav2", "topic": "%s", "frame_id": "%s", "poses": %d, "hz": %g}'
          % (args.topic, args.frame_id, len(pts), args.hz), flush=True)
    while not stop[0] and (t_end is None or time.monotonic() < t_end):
        msg = Path()
        msg.header.frame_id = args.frame_id
        msg.header.stamp = node.get_clock().now().to_msg()
        for x, y in pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        pub.publish(msg)
        sent += 1
        nxt += period
        while not stop[0] and time.monotonic() < nxt:
            time.sleep(min(0.05, max(0.0, nxt - time.monotonic())))
    print('{"sent": %d}' % sent, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
