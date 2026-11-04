#!/usr/bin/env python3
"""Measurement probe for the geometry producer rig (read-only subscriber).

Records for --duration-s:
  /fpga/rolling_map/status  bridge freshness rows: map age (bridge's own fpga_clock age of the
                           newest applied delta), fresh flag, reason, counts
  /fpga/rolling_map/grid    OccupancyGrid from the bridge (count, cells free/lethal/unknown)
  /costmap/costmap         Nav2 standalone costmap built from the grid (count, cell values)
  /fpga/geometry_frame      producer bodies (count, failures)
Writes --out (new JSON): counts, status-age percentiles, last grid / costmap summaries.
--costmap-topic changes the costmap topic (e.g. /local_costmap/costmap).
--start-at-wall EPOCH (v3, 10-11 night; measurement fix (a)): create the node and its subscriptions
NOW (the new DDS participant joins during the rig's warm-up), ignore messages until wall time EPOCH,
then record for --duration-s. A participant joining inside the measurement window can stall the
producer's publish for ~300 ms about 2.6-3 s later (the v1/v2 formal-run stalls); the probe used to
join at the window start. Messages seen before EPOCH are only counted in "pre" (the latched grid
usually arrives then, so "grid" counts the live republishes only).
"""
import argparse
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def pct(v, q):
    return round(float(np.percentile(v, q)), 3) if v else None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration-s", type=float, default=180.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--costmap-topic", default="/costmap/costmap")
    ap.add_argument("--start-at-wall", type=float, default=None,
                    help="join now, record from this wall time (time.time()) on")
    a, ros_args = ap.parse_known_args(argv)
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    rclpy.init(args=ros_args)
    node = Node("fpga_geo_probe")
    rec = {"status": 0, "status_fresh": 0, "status_reasons": {}, "status_age_ms": [], "grid": 0,
           "costmap": 0, "frames": 0, "frame_failures": {}, "last_grid": None, "last_costmap": None,
           "first_costmap_t": None, "bridge_counts": None}
    t0 = time.monotonic()
    pre = {"recording": a.start_at_wall is None, "msgs": 0}

    def gate(cb):
        def inner(msg):
            if not pre["recording"]:
                pre["msgs"] += 1
                return
            cb(msg)
        return inner

    def on_status(msg):
        row = json.loads(msg.data)
        rec["status"] += 1
        rec["status_fresh"] += bool(row.get("fresh"))
        r = str(row.get("reason"))
        rec["status_reasons"][r] = rec["status_reasons"].get(r, 0) + 1
        age = row.get("map_age_ns", row.get("age_ns"))
        if age is not None:
            rec["status_age_ms"].append(age / 1e6)
        rec["bridge_counts"] = row.get("counts")
        rec["status_keys"] = sorted(row)

    def grid_summary(msg):
        d = np.asarray(msg.data, dtype=np.int16)
        return {"w": msg.info.width, "h": msg.info.height, "res": msg.info.resolution,
                "origin": [msg.info.origin.position.x, msg.info.origin.position.y],
                "frame": msg.header.frame_id, "values": {str(int(v)): int(c) for v, c in
                                                         zip(*np.unique(d, return_counts=True))}}

    def on_grid(msg):
        rec["grid"] += 1
        rec["last_grid"] = grid_summary(msg)

    def on_costmap(msg):
        rec["costmap"] += 1
        if rec["first_costmap_t"] is None:
            rec["first_costmap_t"] = time.monotonic() - t0
        rec["last_costmap"] = grid_summary(msg)

    def on_frame(msg):
        body = json.loads(msg.data)
        rec["frames"] += 1
        f = str(body.get("failure"))
        rec["frame_failures"][f] = rec["frame_failures"].get(f, 0) + 1

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(String, "/fpga/rolling_map/status", gate(on_status), 50)
    node.create_subscription(OccupancyGrid, "/fpga/rolling_map/grid", gate(on_grid), latched)
    node.create_subscription(OccupancyGrid, a.costmap_topic, gate(on_costmap), 10)
    node.create_subscription(String, "/fpga/geometry_frame", gate(on_frame), 50)
    if a.start_at_wall is not None:
        rec["pre"] = {"joined_wall": time.time(), "start_at_wall": a.start_at_wall}
        while rclpy.ok() and time.time() < a.start_at_wall:
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, a.start_at_wall - time.time())))
        pre["recording"] = True
        rec["pre"].update({"msgs_ignored": pre["msgs"], "recording_wall": time.time()})
        t0 = time.monotonic()
    end = time.monotonic() + a.duration_s
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        ages = rec.pop("status_age_ms")
        rec["status_age_ms"] = {"n": len(ages), "median": pct(ages, 50), "p95": pct(ages, 95),
                                "p99": pct(ages, 99), "max": max(ages) if ages else None,
                                "over_200": sum(1 for x in ages if x > 200)}
        with open(a.out, "x") as fp:
            json.dump(rec, fp, indent=1, sort_keys=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
