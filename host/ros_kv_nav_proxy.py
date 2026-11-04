"""Nano navigation proxy for the KV260 navigation core (CTRLPS2 N2 + N3, 2026-11-03).

Design: runs/nationals-prelim-20261102/CTRLPS0/DESIGN.md 2.3 (G1), 2.5 (case A), 3. Started by car_inner.sh when
CAR_NAV_BACKEND is kv or shadow (default nav2: this node is not started, nothing changes).

--mode kv      the NavigateToPose action server ``navigate_to_pose`` (same name, same goal / result types as Nav2's
               bt_navigator: host/ros_runtime.py is not changed). A goal goes to the KV as NAV_GOAL; odometry
               (TF odom->base_link, map->odom, /odom twist, /fpga/motion_permit) goes as NAV_ODOM at 50 Hz; every
               answer is a NAV_CMD whose (v, w) is published on /cmd_vel at 20 Hz while a goal runs -- then the
               velocity guard, the teleop arbitration and the chassis driver, all unchanged. A command older than
               150 ms (or not VALID) publishes zero. G1 (N3): every 1 s, and at once when the KV asks
               (need_path), Nav2's planner_server action ``compute_path_to_pose`` gives the global route; it is
               thinned to 0.25 m and sent as NAV_PATH.
--mode shadow  Nav2 drives as always; this node publishes NOTHING on /cmd_vel and serves no action. It takes the
               goal and route from Nav2's /plan (last pose = goal), ends the KV goal when Nav2's
               navigate_to_pose has no running goal, and records every NAV_CMD next to Nav2's /cmd_vel (--record).

stdout: one JSON line per event ("ready", "goal", "result", "link"); --record: JSON lines of the commands.
"""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time

from host.kv_link import KvLinkClient, KvFeatureUnavailable, LinkError, NAV_STATE
from host.kv_nav_proxy import LINK_ABORT_S, LINK_LOST_S, LINK_MODES, LinkWatch, NavLink, sparsify
from host.fpga_clock import now_ns

# link lost (no NAV_CMD for LINK_LOST_S while a goal runs): CTRLPS6 default --link-lost hold = zero command (the
# stale rule), warn, keep the goal and resume it with a replan; ABORTED link_lost only after --link-abort-s (30 s).
# --link-lost abort = the CTRLPS2 rule (ABORTED at once). host/kv_nav_proxy.LinkWatch.
ACCEPT_TIMEOUT_S = 1.0     # the KV must accept / reject a goal within this
FEEDBACK_S = 0.5
PATH_TIMEOUT_S = 5.0       # a ComputePathToPose without result for this long is given up (counted path_timeout)


def emit(event, **data):
    print(json.dumps(dict(event=event, t_ns=now_ns(), **data), sort_keys=True, default=str), flush=True)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def parse(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kv-server", required=True, help="host:port of kv_server --nav soft|pl")
    ap.add_argument("--mode", choices=("kv", "shadow"), required=True)
    ap.add_argument("--record", default=None, help="JSON lines of every NAV_CMD (shadow: with Nav2's /cmd_vel)")
    ap.add_argument("--odom-hz", type=float, default=50.0)
    ap.add_argument("--cmd-hz", type=float, default=20.0)
    ap.add_argument("--path-period-s", type=float, default=1.0)
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--permit-topic", default="/fpga/motion_permit")
    ap.add_argument("--teleop-topic", default="/fpga/teleop/active",
                    help="host/ros_teleop_safe.py arbitration 'in hand' (Bool, every tick); NAV_ODOM teleop bit")
    ap.add_argument("--teleop-stale-s", type=float, default=0.5,
                    help="a teleop sample older than this counts as not in hand (the runner's 0.5 s rule)")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--planner-action", default="compute_path_to_pose")
    ap.add_argument("--planner-id", default="", help="ComputePathToPose planner_id ('' = planner_server default)")
    ap.add_argument("--hello-timeout-s", type=float, default=5.0)
    ap.add_argument("--link-lost", choices=LINK_MODES, default=os.environ.get("FPGA_KV_LINK_LOST", "hold"),
                    help="CTRLPS6: no NAV_CMD for %.1f s while a goal runs: hold (zero, warn, keep the goal, replan "
                         "when back; default) or abort (the CTRLPS2 rule: ABORTED link_lost at once)" % LINK_LOST_S)
    ap.add_argument("--link-abort-s", type=float, default=float(os.environ.get("FPGA_KV_LINK_ABORT_S", LINK_ABORT_S)),
                    help="CTRLPS6 hold mode: a link lost for this long ends the goal ABORTED link_lost (default 30)")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    host, port = args.kv_server.rsplit(":", 1)
    client = KvLinkClient((host, int(port)), timeout_s=1.0, poke_s=0.0)
    try:
        info = client.require("nav", hello_timeout_s=args.hello_timeout_s)
    except (KvFeatureUnavailable, LinkError) as exc:
        emit("refused", reason=str(exc))
        return 3

    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    import tf2_ros
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import Bool
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from nav2_msgs.action import NavigateToPose, ComputePathToPose

    rec_fp = open(args.record, "a") if args.record else None
    rec_lock = threading.Lock()

    def record(row):
        if rec_fp is not None:
            with rec_lock:
                rec_fp.write(json.dumps(row, sort_keys=True) + "\n")
                rec_fp.flush()

    class Proxy(Node):
        def __init__(self):
            super().__init__("fpga_kv_nav_proxy")
            self.cbg = ReentrantCallbackGroup()
            self.tf = tf2_ros.Buffer()
            self.tfl = tf2_ros.TransformListener(self.tf, self)
            self.twist = (0.0, 0.0)
            self.permit = False
            self.teleop = (False, 0.0)    # CTRLPS3: (/fpga/teleop/active, monotonic receive time)
            self.nav2_cmd = None
            self.link = NavLink(client, odom_hz=args.odom_hz, on_cmd=self.on_cmd,
                                hold_after_s=LINK_LOST_S if args.link_lost == "hold" else None)
            self.goal = None              # kv: {"id", "handle", "pose", "t0", "preempted"}; shadow: {"id", "pose"}
            self.goal_lock = threading.Lock()
            self.last_path_req = 0.0
            self.path_busy = False
            self.publish_until = 0.0
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
            self.create_subscription(Bool, args.permit_topic, self.on_permit, 10)
            self.create_subscription(Bool, args.teleop_topic, self.on_teleop, 10)
            self.create_timer(1.0 / args.odom_hz, self.tick_odom, callback_group=self.cbg)
            if args.mode == "kv":
                self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 1)
                self.create_timer(1.0 / args.cmd_hz, self.tick_cmd, callback_group=self.cbg)
                self.create_timer(min(0.25, args.path_period_s), self.tick_path, callback_group=self.cbg)
                self.planner = ActionClient(self, ComputePathToPose, args.planner_action, callback_group=self.cbg)
                self.server = ActionServer(self, NavigateToPose, "navigate_to_pose", execute_callback=self.execute,
                                           goal_callback=self.on_goal_request, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                           handle_accepted_callback=self.on_accepted, callback_group=self.cbg)
            else:   # shadow: never a /cmd_vel publisher, never an action server
                self.cmd_pub = None
                self.create_subscription(Path, "/plan", self.on_plan, 1)
                self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self.on_status, 10)
                self.create_subscription(Twist, args.cmd_topic, self.on_nav2_cmd, 10)
                self.shadow_path_t = 0.0
            self.link.start()

        # -- inputs
        def on_odom(self, msg):
            self.twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

        def on_permit(self, msg):
            self.permit = bool(msg.data)

        def on_teleop(self, msg):
            self.teleop = (bool(msg.data), time.monotonic())

        def teleop_in_hand(self):
            """The remote control holds the car (ros_teleop_safe arbitration): the KV core pauses -- no command,
            no stuck time -- and replans when it is released (DESIGN 2.6). No topic (teleop off) = False."""
            active, t = self.teleop
            return active and time.monotonic() - t <= args.teleop_stale_s

        def on_nav2_cmd(self, msg):
            self.nav2_cmd = (msg.linear.x, msg.angular.z, time.monotonic())

        def lookup(self, parent, child):
            try:
                t = self.tf.lookup_transform(parent, child, rclpy.time.Time())
            except Exception:
                return None, 0
            tr = t.transform
            stamp = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
            return (tr.translation.x, tr.translation.y, yaw_of(tr.rotation)), stamp

        def tick_odom(self):
            ob, stamp = self.lookup(args.odom_frame, args.base_frame)
            mo, _ = self.lookup(args.map_frame, args.odom_frame)
            self.link.set_odom(odom_pose=ob or (0.0, 0.0, 0.0), v=self.twist[0], w=self.twist[1],
                               map_odom=mo or (0.0, 0.0, 0.0), odom_stamp_ns=stamp, permit=self.permit,
                               teleop=self.teleop_in_hand(),
                               map_valid=mo is not None, odom_valid=ob is not None)

        def on_cmd(self, c, t):
            row = {"t_ns": now_ns(), "goal_id": c["goal_id"], "state": c["state_name"], "reason": c["reason_name"],
                   "valid": c["valid"], "v": round(c["v"], 4), "w": round(c["w"], 4), "cmd_seq": c["cmd_seq"],
                   "odom_seq": c["odom_seq"], "cmd_age_us": c["cmd_age_us"], "remaining_m": round(c["remaining_m"], 3)}
            if args.mode == "shadow":
                n = self.nav2_cmd
                row["nav2"] = None if n is None else {"v": round(n[0], 4), "w": round(n[1], 4),
                                                       "age_s": round(time.monotonic() - n[2], 3)}
                if c["odom_seq"] and c["odom_seq"] % 5:      # shadow record at 10 Hz
                    return
            elif c["odom_seq"] and c["odom_seq"] % 5 and not c["need_path"]:
                return
            record(row)

        # -- kv mode: action server
        def on_goal_request(self, goal_request):
            return GoalResponse.ACCEPT

        def on_accepted(self, handle):
            with self.goal_lock:
                if self.goal is not None:                  # Nav2 semantics: the new goal preempts the old one
                    self.goal["preempted"] = True
            handle.execute()

        def execute(self, handle):
            p = handle.request.pose
            pose = (p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation))
            gid = self.link.new_goal_id()
            mine = {"id": gid, "handle": handle, "pose": pose, "frame": p.header.frame_id, "t0": time.monotonic(),
                    "preempted": False}
            with self.goal_lock:
                self.goal = mine
            self.link.send_goal(gid, *pose)
            self.publish_until = float("inf")
            self.last_path_req = 0.0
            emit("goal", goal_id=gid, pose=pose, frame=p.header.frame_id)
            outcome, reason = self.wait_goal(mine)
            with self.goal_lock:
                if self.goal is mine:
                    self.goal = None
                    self.link.end_goal()
                    self.publish_until = time.monotonic() + 0.5     # a few zeros after the end
            emit("result", goal_id=gid, outcome=outcome, reason=reason)
            if outcome == "succeeded":
                handle.succeed()
            elif outcome == "canceled":
                handle.canceled()
            else:
                handle.abort()
            return NavigateToPose.Result()

        def wait_goal(self, mine):
            gid = mine["id"]
            t_fb = 0.0
            cancel_sent = None

            def warn(**kw):
                emit("link", goal_id=gid, **kw)
                if kw.get("state") in ("lost", "abort"):
                    self.get_logger().warn("kv nav link %s: no NAV_CMD for %.2f s (goal kept up to %.0f s)"
                                           % (kw["state"], kw["gap_s"], kw["abort_s"]))
                else:
                    self.get_logger().warn("kv nav link resumed after %.2f s: replanning" % kw["gap_s"])

            def resume():
                self.last_path_req = 0.0          # a fresh G1 route at once

            watch = LinkWatch(self.link, mode=args.link_lost, lost_s=LINK_LOST_S, abort_s=args.link_abort_s,
                              warn=warn, on_resume=resume)
            while rclpy.ok():
                time.sleep(0.05)
                now = time.monotonic()
                if mine["preempted"]:
                    self.link.send_cancel(gid)
                    return "aborted", "preempted"
                if mine["handle"].is_cancel_requested and cancel_sent is None:
                    self.link.send_cancel(gid)
                    cancel_sent = now
                c = self.link.goal_status(gid)
                if c is None:
                    if now - mine["t0"] > ACCEPT_TIMEOUT_S:
                        return ("canceled", "link_lost") if cancel_sent else ("aborted", "link_lost")
                    continue
                if not c["accepted"]:
                    return "aborted", "goal_rejected"
                if self.link.terminal(c):
                    return c["state_name"], c["reason_name"]
                if cancel_sent is not None and now - cancel_sent > ACCEPT_TIMEOUT_S:
                    return "canceled", "link_lost"
                lost = watch.check(now)
                if lost is not None:
                    self.link.send_cancel(gid)
                    return lost
                if now - t_fb >= FEEDBACK_S:
                    t_fb = now
                    fb = NavigateToPose.Feedback()
                    fb.distance_remaining = float(c["remaining_m"])
                    fb.number_of_recoveries = int(c["recoveries"])
                    el = now - mine["t0"]
                    fb.navigation_time.sec, fb.navigation_time.nanosec = int(el), int((el % 1) * 1e9)
                    fb.current_pose.header.frame_id = args.map_frame
                    mo, _ = self.lookup(args.map_frame, args.base_frame)
                    if mo is not None:
                        fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = mo[0], mo[1]
                        fb.current_pose.pose.orientation.z = math.sin(mo[2] / 2)
                        fb.current_pose.pose.orientation.w = math.cos(mo[2] / 2)
                    mine["handle"].publish_feedback(fb)
            return "aborted", "link_lost"

        def tick_cmd(self):
            if time.monotonic() > self.publish_until:
                return
            v, w, why = self.link.command()
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
            self.cmd_pub.publish(m)
            if why not in ("ok", "no_goal"):
                self.link.counts["zero:%s" % why] += 1

        # -- G1 (N3): planner_server's route -> NAV_PATH
        def tick_path(self):
            g = self.goal
            now = time.monotonic()
            if self.path_busy and now - self.last_path_req > PATH_TIMEOUT_S:   # planner never answered: try again
                self.path_busy = False
                self.link.counts["path_timeout"] += 1
            if g is None or self.path_busy:
                return
            c = self.link.goal_status(g["id"])
            due = now - self.last_path_req >= args.path_period_s
            urgent = c is not None and c["need_path"] and now - self.last_path_req >= 0.3
            if not (due or urgent):
                return
            if not self.planner.server_is_ready():
                self.link.counts["path_planner_not_ready"] += 1
                return
            self.last_path_req = now
            self.path_busy = True
            req = ComputePathToPose.Goal()
            req.goal.header.frame_id = g["frame"] or args.map_frame
            req.goal.pose.position.x, req.goal.pose.position.y = g["pose"][0], g["pose"][1]
            req.goal.pose.orientation.z = math.sin(g["pose"][2] / 2)
            req.goal.pose.orientation.w = math.cos(g["pose"][2] / 2)
            req.planner_id = args.planner_id
            req.use_start = False
            fut = self.planner.send_goal_async(req)
            fut.add_done_callback(lambda f, gid=g["id"]: self.on_path_accepted(f, gid))

        def on_path_accepted(self, fut, gid):
            try:
                h = fut.result()
                if not h.accepted:
                    raise RuntimeError("planner rejected")
                h.get_result_async().add_done_callback(lambda f: self.on_path(f, gid))
            except Exception as exc:
                self.path_busy = False
                self.link.counts["path_error"] += 1
                emit("path_failed", goal_id=gid, reason=str(exc))

        def on_path(self, fut, gid):
            self.path_busy = False
            try:
                res = fut.result()
                poses = res.result.path.poses
            except Exception as exc:
                self.link.counts["path_error"] += 1
                emit("path_failed", goal_id=gid, reason=str(exc))
                return
            g = self.goal
            if g is None or g["id"] != gid or not poses:
                self.link.counts["path_empty_or_late"] += 1
                return
            n = self.link.send_path(gid, [(q.pose.position.x, q.pose.position.y) for q in poses])
            self.link.counts["path_sent"] += 1
            if self.link.counts["path_sent"] <= 3:
                emit("path", goal_id=gid, raw=len(poses), sent=n)

        # -- shadow mode: observe Nav2
        def on_plan(self, msg):
            if not msg.poses:
                return
            last = msg.poses[-1].pose
            pose = (last.position.x, last.position.y, yaw_of(last.orientation))
            g = self.goal
            if g is None or math.hypot(pose[0] - g["pose"][0], pose[1] - g["pose"][1]) > 0.05:
                gid = self.link.new_goal_id()
                self.goal = {"id": gid, "pose": pose}
                self.link.send_goal(gid, *pose)
                self.shadow_path_t = 0.0
                emit("goal", goal_id=gid, pose=pose, frame=msg.header.frame_id, source="nav2_plan")
            now = time.monotonic()
            if now - self.shadow_path_t >= args.path_period_s:
                self.shadow_path_t = now
                self.link.send_path(self.goal["id"], [(q.pose.position.x, q.pose.position.y) for q in msg.poses])

        def on_status(self, msg):
            running = any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                       GoalStatus.STATUS_CANCELING) for s in msg.status_list)
            g = self.goal
            if g is not None and not running:
                self.link.send_cancel(g["id"])
                emit("result", goal_id=g["id"], outcome="nav2_goal_ended", source="nav2_status")
                self.goal = None
                self.link.end_goal()

        def stop(self):
            self.link.stop()

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupt)
    node = Proxy()
    emit("ready", mode=args.mode, kv_server=args.kv_server, kv_features=info.get("features"),
         cmd_topic=args.cmd_topic if args.mode == "kv" else None)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if args.mode == "kv":
            try:
                node.cmd_pub.publish(Twist())
            except Exception:
                pass
        node.stop()
        emit("exit", counts=dict(node.link.counts), client=dict(client.counts))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if rec_fp is not None:
            rec_fp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
