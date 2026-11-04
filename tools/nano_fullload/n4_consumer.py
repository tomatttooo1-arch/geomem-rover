#!/usr/bin/env python3
"""N4 navigation consumer (inside the ROS container). docs/design/NANO_FULLLOAD_20260927.md §2-5.

- logs every /fpga/rolling_map/status on receipt: [rx_mono_ns, stamp_ns, age_ns, fresh, reason]
  (container and host share CLOCK_MONOTONIC, the same clock the bridge stamps with);
- judges the motion permit at 20 Hz with host.rolling_nav_layer.rolling_map_permit (200 ms, the
  D-035 rule), publishes /fpga/motion_permit and logs every revocation;
- goal loop (L5, enabled when <out>/goals_on exists): drive the virtual base 0.6 m forward by
  script (so the map ahead exists), then alternate NavigateToPose goals between two points;
  a revoked permit cancels the goal, which is re-sent once the permit has been back for 1 s.
"""
import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

GOALS = [(1.15, 0.30), (0.10, 0.10)]   # the N3 smoke goals (robot_sim/tests/rolling_nav2_smoke.py)


def mono():
    return time.monotonic_ns()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import Bool, String
    from nav_msgs.msg import Odometry
    from host.rolling_nav_layer import rolling_map_permit

    rclpy.init()
    node = Node("fpga_n4_consumer")
    f_st = open(out / "status.jsonl", "w", buffering=1 << 16)
    f_ev = open(out / "events.jsonl", "w", buffering=1)
    S = {"status": None, "rx": None, "ok": None, "odom": None, "goal": None, "result": None,
         "goal_i": 0, "phase": "idle", "ok_since": None, "wait_until": 0, "cmd_nonzero": 0,
         "cmd_last": None}

    def ev(kind, **kw):
        f_ev.write(json.dumps({"t": mono(), "wall": time.time(), "ev": kind, **kw}) + "\n")

    def on_status(msg):
        rx = mono()
        row = json.loads(msg.data)
        S["status"], S["rx"] = row, rx
        f_st.write('[%d,%d,%s,%d,"%s"]\n' % (rx, int(row.get("stamp_ns") or 0),
                                             "null" if row.get("age_ns") is None else int(row["age_ns"]),
                                             1 if row.get("fresh") is True else 0, row.get("reason")))

    def on_odom(msg):
        S["odom"] = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x)

    node.create_subscription(String, "/fpga/rolling_map/status", on_status, 10)
    node.create_subscription(Odometry, "/odom", on_odom, 10)
    permit_pub = node.create_publisher(Bool, "/fpga/motion_permit", 1)
    control = node.create_publisher(String, "/smoke/control", 10)
    action = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def tick():
        ok, reason = rolling_map_permit(S["status"], received_ns=S["rx"], now_ns=mono())
        driving = S["phase"] in ("goal", "script")
        permit_pub.publish(Bool(data=bool(ok and driving)))
        if ok != S["ok"]:
            ev("permit", ok=ok, reason=reason, phase=S["phase"],
               age_ns=(S["status"] or {}).get("age_ns"), since_rx_ns=None if S["rx"] is None else mono() - S["rx"])
            S["ok"] = ok
            S["ok_since"] = mono() if ok else None
            if not ok and S["phase"] == "goal" and S["goal"] is not None:
                S["goal"].cancel_goal_async()
                ev("goal_cancel_on_revoke", goal=GOALS[S["goal_i"] % 2])
                S["phase"] = "resend"

    node.create_timer(0.05, tick)

    def on_result(fut):
        try:
            r = fut.result()
            st = r.status
        except Exception as exc:  # noqa: BLE001
            st = "exc:%s" % exc
        ev("goal_result", status=st, goal=GOALS[S["goal_i"] % 2], odom=S["odom"])
        if S["phase"] == "goal":
            S["goal_i"] += 1
            S["phase"] = "next"
            S["wait_until"] = mono() + 1_000_000_000

    def on_accept(fut):
        h = fut.result()
        if h is None or not h.accepted:
            ev("goal_rejected", goal=GOALS[S["goal_i"] % 2])
            S["phase"] = "next"
            S["wait_until"] = mono() + 2_000_000_000
            return
        S["goal"] = h
        S["phase"] = "goal"
        ev("goal_accepted", goal=GOALS[S["goal_i"] % 2])
        h.get_result_async().add_done_callback(on_result)

    def send_goal():
        xy = GOALS[S["goal_i"] % 2]
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "odom"
        g.pose.header.stamp = node.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = xy
        g.pose.pose.orientation.w = 1.0
        S["phase"] = "sending"
        ev("goal_send", goal=xy)
        action.send_goal_async(g).add_done_callback(on_accept)

    def loop():
        on = (out / "goals_on").exists()
        t = mono()
        if not on:
            return
        if S["phase"] == "idle":
            control.publish(String(data=json.dumps({"cmd": "script", "v": 0.1, "seconds": 6.0})))
            ev("script_drive", v=0.1, seconds=6.0)
            S["phase"] = "script"
            S["wait_until"] = t + 7_000_000_000
        elif S["phase"] == "script" and t >= S["wait_until"]:
            S["phase"] = "next"
        elif S["phase"] in ("next", "resend") and t >= S["wait_until"]:
            if not S["ok"] or S["ok_since"] is None or t - S["ok_since"] < 1_000_000_000:
                return
            if not action.server_is_ready():
                return
            send_goal()

    node.create_timer(0.2, loop)
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        ev("stop")
        f_st.close(); f_ev.close()


if __name__ == "__main__":
    main()
