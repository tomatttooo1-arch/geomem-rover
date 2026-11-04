#!/usr/bin/env python3
"""B1 car shadow comparison (DRAFT, 2026-11-02; DESIGN 5.2 item 1): /fpga/rolling_map/grid (the producer, from the
KV260 planes) against /fpga/rolling_map/grid_sw (the bridge in shadow mode, CAR_PL_GRID=1 CAR_PL_GRID_SHADOW=1).

Not run yet (no car, no board in this work package). Start it inside the stack container AFTER the stack is READY
(car rule: no new ROS participant while the car drives -- start it before the drive, stop it after):

  python3 tools/b1_costmap/shadow_compare.py --duration-s 600 --out /o/b1_shadow.json

Pairing: both grids are published once per fused frame (the producer right after the INTEGRATE, the bridge when the
delta arrives), each stamped with its own node's clock; a /grid message is paired with the first /grid_sw that
arrives within --pair-ms after or before it with the same origin. Per pair: cells different, occupied-cell IoU, and
the different cells within --body-m of the body (the body seed and swept clearing use the TF pose at publish time,
which differs by the gap between the two publications; a moving car can differ there and only there).
Judgement (CONTROL, frozen before the run): every pair identical, or occupied IoU >= 0.95.
"""
import argparse
import json
import sys
import time

import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration-s", type=float, default=600.0)
    ap.add_argument("--pair-ms", type=float, default=150.0)
    ap.add_argument("--body-m", type=float, default=0.5)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    import rclpy
    import rclpy.time
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener
    from host.ros_rolling_nav import GRID_QOS

    rclpy.init()
    node = Node("fpga_b1_shadow_compare")
    tf = Buffer()
    TransformListener(tf, node)
    pend = {"pl": [], "sw": []}

    def keep(name):
        def cb(m):
            pend[name].append((time.monotonic(), m))
        return cb
    node.create_subscription(OccupancyGrid, "/fpga/rolling_map/grid", keep("pl"), GRID_QOS)
    node.create_subscription(OccupancyGrid, "/fpga/rolling_map/grid_sw", keep("sw"), GRID_QOS)
    res = {"pairs": 0, "identical": 0, "unpaired_pl": 0, "cells_diff_total": 0, "cells_diff_far_total": 0,
           "min_occ_iou": 1.0, "worst": []}

    def pose():
        try:
            t = tf.lookup_transform("odom", "base_link", rclpy.time.Time()).transform.translation
            return t.x, t.y
        except Exception:  # noqa: BLE001
            return None

    def settle(now):
        while pend["pl"] and now - pend["pl"][0][0] > 2 * a.pair_ms / 1e3:
            t_pl, m_pl = pend["pl"].pop(0)
            match = None
            for i, (t_sw, m_sw) in enumerate(pend["sw"]):
                if abs(t_sw - t_pl) * 1e3 <= a.pair_ms and \
                        (m_sw.info.origin.position.x, m_sw.info.origin.position.y) == \
                        (m_pl.info.origin.position.x, m_pl.info.origin.position.y):
                    match = i
                    break
            if match is None:
                res["unpaired_pl"] += 1
                continue
            _, m_sw = pend["sw"].pop(match)
            g, s = np.array(m_pl.data, np.int8), np.array(m_sw.data, np.int8)
            d = g != s
            res["pairs"] += 1
            res["identical"] += int(not d.any())
            res["cells_diff_total"] += int(d.sum())
            occ_a, occ_b = g == 100, s == 100
            u = int((occ_a | occ_b).sum())
            res["min_occ_iou"] = min(res["min_occ_iou"], 1.0 if u == 0 else int((occ_a & occ_b).sum()) / u)
            p = pose()
            if p is not None and d.any():
                w, h, r = m_pl.info.width, m_pl.info.height, m_pl.info.resolution
                xs = m_pl.info.origin.position.x + (np.arange(w) + 0.5) * r
                ys = m_pl.info.origin.position.y + (np.arange(h) + 0.5) * r
                far = np.hypot(xs[None, :] - p[0], ys[:, None] - p[1]).ravel() > a.body_m
                res["cells_diff_far_total"] += int((d & far).sum())
            if d.any() and len(res["worst"]) < 20:
                res["worst"].append({"t": t_pl, "cells": int(d.sum())})
        while pend["sw"] and now - pend["sw"][0][0] > 4 * a.pair_ms / 1e3:
            pend["sw"].pop(0)

    end = time.monotonic() + a.duration_s
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            settle(time.monotonic())
    except KeyboardInterrupt:
        pass
    settle(float("inf"))
    res["verdict"] = ("identical" if res["pairs"] and res["identical"] == res["pairs"] else
                      "iou_ok" if res["pairs"] and res["min_occ_iou"] >= 0.95 else "fail_or_no_data")
    with open(a.out, "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "worst"}))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
