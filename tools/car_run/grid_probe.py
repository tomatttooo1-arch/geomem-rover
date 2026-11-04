#!/usr/bin/env python3
"""Line E, board Nav2 with the rolling costmap (2026-10-15): has the rolling-map bridge published its grid yet?

FPGA_NAV_COSTMAP=rolling makes Nav2's local costmap a StaticLayer on /fpga/rolling_map/grid (rolling_layer_nav2.yaml).
host/ros_rolling_nav.py publishes that grid only after it has accepted a delta or snapshot of the geometry producer
(RollingBridgeNode._after: changed), so a bridge that runs but gets nothing from the producer publishes no grid, and
Nav2 would drive on an all-NO_INFORMATION local costmap. car_inner.sh runs this probe before Nav2 is activated.

Subscribes nav_msgs/OccupancyGrid on --topic with the bridge's GRID_QOS (reliable, transient_local, depth 1: a grid
latched before the probe started is delivered too) and exits 0 on the first message, 1 when --timeout-s passes first
(2: rclpy / message import failed). One JSON line on stdout: received, seconds, the grid's frame, size, resolution and
known-cell share. Python 3.8 compatible."""
import argparse
import json
import time


def known_share(data):
    """share of grid cells that are not -1 (unknown); None for an empty grid."""
    cells = list(data)
    return None if not cells else round(1.0 - cells.count(-1) / float(len(cells)), 4)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/fpga/rolling_map/grid")
    ap.add_argument("--timeout-s", type=float, default=60.0)
    a = ap.parse_args(argv)
    t0 = time.monotonic()
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from nav_msgs.msg import OccupancyGrid
    except ImportError as exc:
        print(json.dumps({"error": "import: %s" % exc}))
        return 2
    got = []
    rclpy.init(args=None)
    node = rclpy.create_node("fpga_grid_probe")
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, a.topic, got.append, qos)
    try:
        while not got and time.monotonic() - t0 < a.timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        m = got[0] if got else None
        row = {"topic": a.topic, "received": len(got), "seconds": round(time.monotonic() - t0, 3),
               "grid": None if m is None else {"frame_id": m.header.frame_id, "width": int(m.info.width),
                                               "height": int(m.info.height),
                                               "resolution": round(float(m.info.resolution), 6),
                                               "known_share": known_share(m.data)}}
        print(json.dumps(row), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if got else 1


if __name__ == "__main__":
    raise SystemExit(main())
