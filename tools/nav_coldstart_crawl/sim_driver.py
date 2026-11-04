#!/usr/bin/env python3
"""Goal + recorder for the cold-start crawl check (docs/design/NAV_COLDSTART_CRAWL_20261017.md section 4).

Waits until /fpga/rolling_map/grid has been arriving for --warmup-s (the car stands still and fuses, like the board
before its goto), sends NavigateToPose map (--goal-x, 0, yaw 0), records until the result (or --timeout-s after
acceptance, then cancels), plus --tail-s. Records ("t" = RAW receipt, host.fpga_clock; the receipt lags by up to ~0.7 s under load (m2), so the analysis
matches by the ROS header stamps "stamp"/"ros" against the scene's truth rows):
  eval.jsonl   one row per DWB /evaluation: [vx, vtheta, total, failing critic] for every sample, best index
  cmd.jsonl    every /cmd_vel and /fpga/cmd_vel_safe message
  odom.jsonl   every /odom
  nav.json     goal sent / accepted / result times and status, feedback (recoveries, distance remaining)
  grids.npz    every /fpga/rolling_map/grid and /local_costmap/costmap (int8 data, origin, header ROS stamp ns)
Python 3.8+.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--warmup-s", type=float, default=5.0)
    ap.add_argument("--goal-x", type=float, default=1.0)
    ap.add_argument("--timeout-s", type=float, default=45.0)
    ap.add_argument("--tail-s", type=float, default=2.0)
    a = ap.parse_args()

    import numpy as np
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import NavigateToPose
    from dwb_msgs.msg import LocalPlanEvaluation
    from host.fpga_clock import now_ns

    rclpy.init()
    node = Node("fpga_crawl_driver")
    O = a.out
    f_eval = open(os.path.join(O, "eval.jsonl"), "w")
    f_cmd = open(os.path.join(O, "cmd.jsonl"), "w")
    f_odom = open(os.path.join(O, "odom.jsonl"), "w")
    grids = {"/fpga/rolling_map/grid": [], "/local_costmap/costmap": []}
    nav = {"events": [], "feedback": []}
    first_grid = [None]

    last_eval = [time.monotonic()]

    def on_eval(m):
        last_eval[0] = time.monotonic()
        tw = []
        for s in m.twists:
            fail = None
            if s.total < 0:
                bad = [c.name for c in s.scores if c.raw_score < 0]
                fail = bad[-1] if bad else (s.scores[-1].name if s.scores else "")
            tw.append([s.traj.velocity.x, s.traj.velocity.theta, s.total, fail])
        f_eval.write(json.dumps({"t": now_ns(), "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                 "best": m.best_index, "tw": tw}) + "\n")

    def cmd_cb(topic):
        def cb(m):
            f_cmd.write(json.dumps({"t": now_ns(), "topic": topic, "lx": m.linear.x, "ly": m.linear.y,
                                    "az": m.angular.z}) + "\n")
        return cb

    def on_odom(m):
        q = m.pose.pose.orientation
        f_odom.write(json.dumps({"t": now_ns(), "ros": m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec,
                                 "x": m.pose.pose.position.x, "y": m.pose.pose.position.y,
                                 "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                                 "v": m.twist.twist.linear.x, "w": m.twist.twist.angular.z}) + "\n")

    def grid_cb(topic):
        def cb(m):
            if topic == "/fpga/rolling_map/grid" and first_grid[0] is None:
                first_grid[0] = time.monotonic()
            grids[topic].append((m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec, m.info.origin.position.x, m.info.origin.position.y, m.info.width,
                                 m.info.height, np.asarray(m.data, dtype=np.int8)))
        return cb

    latched = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(LocalPlanEvaluation, "/evaluation", on_eval, 50)
    node.create_subscription(Twist, "/cmd_vel", cmd_cb("/cmd_vel"), 50)
    node.create_subscription(Twist, "/fpga/cmd_vel_safe", cmd_cb("/fpga/cmd_vel_safe"), 50)
    node.create_subscription(Odometry, "/odom", on_odom, 50)
    for t in grids:
        node.create_subscription(OccupancyGrid, t, grid_cb(t), latched)

    def ev(name, **kw):
        nav["events"].append(dict(name=name, t=now_ns(), **kw))
        print(json.dumps(nav["events"][-1]), flush=True)

    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    deadline = time.monotonic() + 120
    while rclpy.ok() and (first_grid[0] is None or time.monotonic() - first_grid[0] < a.warmup_s):
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.monotonic() > deadline:
            ev("no_grid")
            break
    ok = client.wait_for_server(timeout_sec=30.0)
    ev("server", ok=ok, warmup_grids=len(grids["/fpga/rolling_map/grid"]))
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = a.goal_x
    goal.pose.pose.orientation.w = 1.0

    def fb(msg):
        f = msg.feedback
        nav["feedback"].append({"t": now_ns(), "recoveries": f.number_of_recoveries,
                                "remaining": f.distance_remaining,
                                "nav_time": f.navigation_time.sec + f.navigation_time.nanosec * 1e-9})

    ev("goal_sent", x=a.goal_x)
    fut = client.send_goal_async(goal, feedback_callback=fb)
    while rclpy.ok() and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.02)
    gh = fut.result()
    ev("goal_response", accepted=bool(gh and gh.accepted))
    status = None
    if gh is not None and gh.accepted:
        t_acc = time.monotonic()
        rf = gh.get_result_async()
        cancelled = False
        while rclpy.ok() and not rf.done():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not cancelled and time.monotonic() - t_acc > a.timeout_s:
                ev("timeout_cancel", after_s=a.timeout_s)
                gh.cancel_goal_async()
                cancelled = True
        status = rf.result().status if rf.done() else None
        ev("result", status=status)
    # drain the receipt backlog (m4 lost the last 0.2 s of /evaluation when the tail ended with ~2 s queued):
    # spin at least --tail-s, then until no /evaluation arrived for 1.5 s (cap 30 s)
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < 30.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() - t0 >= a.tail_s and time.monotonic() - last_eval[0] > 1.5:
            break
    ev("drained", after_s=round(time.monotonic() - t0, 2))
    for f in (f_eval, f_cmd, f_odom):
        f.close()
    json.dump(nav, open(os.path.join(O, "nav.json"), "w"), indent=1)
    arrs = {}
    for t, rows in grids.items():
        key = "grid" if "rolling" in t else "costmap"
        if rows:
            arrs[key + "_t"] = np.array([r[0] for r in rows], np.int64)
            arrs[key + "_origin"] = np.array([[r[1], r[2]] for r in rows], np.float64)
            arrs[key + "_wh"] = np.array([[r[3], r[4]] for r in rows], np.int32)
            arrs[key + "_data"] = np.stack([r[5] for r in rows])
    arrs["stamp_kind"] = np.array(["ros_header_ns"])
    np.savez_compressed(os.path.join(O, "grids.npz"), **arrs)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if status == 4 else 1


if __name__ == "__main__":
    sys.exit(main())
