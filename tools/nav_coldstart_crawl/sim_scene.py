#!/usr/bin/env python3
"""Closed-loop scene for the cold-start crawl check (docs/design/NAV_COLDSTART_CRAWL_20261017.md section 1.2).

Structure after robot_sim/tests/rolling_nav2_smoke.py's scene (that file is not changed): a kinematic differential
robot that executes /fpga/cmd_vel_safe (0.3 s command watchdog), publishes odom->base_link and /odom at 50 Hz, a
/fpga/motion_permit True at 10 Hz (stand-in for the task runner's permit), and the car's producer side:
crawl_lib.render_codes (the board camera: K, 640x480, mount fitted on the car) -> host.geo_producer.GeoProducer
(CPU frame core, row-interleaved on --cpu-threads; CPU rolling backend; block 32x32x10, height_cell 8) ->
host.ros_rolling_nav.RollingProducerLink (/fpga/rolling_map/delta, resync answers). Render and process run in two
threads (render frame k+1 while frame k is fused) at --frame-hz on the RAW monotonic clock (host.fpga_clock).
Boxes (--box x0,x1,y0,y1,z1) are both rendered and solid: a translation that would make the body (radius 0.30 m)
overlap one is refused and counted ("blocked", as in the smoke).
Writes OUT/scene.jsonl (frames, 10 Hz pose rows with the body-to-box clearance, events). Python 3.8+.
"""
import argparse
import json
import math
import os
import queue
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import crawl_lib as C  # noqa: E402


def rect_distance(px, py, box):
    x0, x1, y0, y1, _ = box
    return math.hypot(max(x0 - px, 0.0, px - x1), max(y0 - py, 0.0, py - y1))


class LockedFusion(object):
    """RollingProducerLink answers resync requests from the executor thread while the worker fuses
    (host/ros_geo_producer.py does the same)."""

    def __init__(self, fusion, lock):
        self.f, self.lock = fusion, lock
        self.voxel_m = fusion.voxel_m

    def snapshot(self, **kw):
        with self.lock:
            return self.f.snapshot(**kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--frame-hz", type=float, default=10.0)
    ap.add_argument("--cpu-threads", type=int, default=4)
    ap.add_argument("--box", action="append", default=[], help="x0,x1,y0,y1,z1 (m); repeatable")
    a = ap.parse_args()
    boxes = [tuple(float(v) for v in b.split(",")) for b in a.box]
    for b in boxes:
        if len(b) != 5:
            raise SystemExit("--box needs x0,x1,y0,y1,z1")

    import numpy as np
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Bool
    from tf2_ros import TransformBroadcaster
    from host.fpga_clock import now_ns
    from host.ros_rolling_nav import RollingProducerLink

    prod = C.car_producer(tempfile.mkdtemp(prefix="crawl_scene_"), threads=a.cpu_threads)
    T_bc = C.base_camera_T()
    world = C.ROOM + boxes
    lock = threading.Lock()

    rclpy.init()
    node = Node("fpga_crawl_scene")
    link = RollingProducerLink(node, LockedFusion(prod.fusion, lock))
    tfb = TransformBroadcaster(node)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    permit_pub = node.create_publisher(Bool, "/fpga/motion_permit", 1)
    st = {"x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.0, "w": 0.0, "cmd_t": 0, "blocked": 0, "last": now_ns(),
          "tick": 0, "min_clear": float("inf"), "cmds": 0}
    st_lock = threading.Lock()
    log = open(os.path.join(a.out, "scene.jsonl"), "w")
    log_lock = threading.Lock()

    def write(row):
        with log_lock:
            log.write(json.dumps(row) + "\n")

    write({"event": "start", "t": now_ns(), "boxes": boxes, "frame_hz": a.frame_hz, "cpu_threads": a.cpu_threads,
           "producer": prod.describe(), "cam": {"height": C.CAM_HEIGHT_M, "pitch": C.CAM_PITCH_DEG,
                                                "roll": C.CAM_ROLL_DEG, "K": C.K_BOARD.tolist()}})

    def cmd(msg):
        with st_lock:
            st["v"], st["w"], st["cmd_t"] = msg.linear.x, msg.angular.z, now_ns()
            st["cmds"] += 1
            n = st["cmds"]
        # every command the base executes (C5 "every message" on the consumer side)
        write({"event": "cmd", "t": now_ns(), "lx": msg.linear.x, "ly": msg.linear.y, "az": msg.angular.z, "n": n})

    node.create_subscription(Twist, "/fpga/cmd_vel_safe", cmd, 10)

    def physics():
        t = now_ns()
        with st_lock:
            dt = (t - st["last"]) / 1e9
            st["last"] = t
            if t - st["cmd_t"] > 300_000_000:      # the base's own command watchdog
                st["v"] = st["w"] = 0.0
            nx = st["x"] + st["v"] * math.cos(st["yaw"]) * dt
            ny = st["y"] + st["v"] * math.sin(st["yaw"]) * dt
            if any(rect_distance(nx, ny, b) < C.ROBOT_RADIUS_M for b in world):
                if st["v"] != 0.0:
                    st["blocked"] += 1
            else:
                st["x"], st["y"] = nx, ny
            st["yaw"] += st["w"] * dt
            x, y, yaw, v, w = st["x"], st["y"], st["yaw"], st["v"], st["w"]
            clear = min([rect_distance(x, y, b) - C.ROBOT_RADIUS_M for b in boxes], default=float("inf"))
            st["min_clear"] = min(st["min_clear"], clear)
            st["tick"] += 1
            tick, blocked = st["tick"], st["blocked"]
        ros_now = node.get_clock().now()
        stamp = ros_now.to_msg()
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tfb.sendTransform(tf)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "odom", "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = x, y
        od.pose.pose.orientation = tf.transform.rotation
        od.twist.twist.linear.x, od.twist.twist.angular.z = v, w
        odom_pub.publish(od)
        if tick % 2 == 0:                          # 25 Hz truth rows (RAW t and the ROS stamp of the TF)
            write({"event": "pose", "t": t, "ros": ros_now.nanoseconds, "x": x, "y": y, "yaw": yaw, "v": v, "w": w,
                   "blocked": blocked,
                   "clear": None if clear == float("inf") else clear})

    def permit():
        permit_pub.publish(Bool(data=True))

    node.create_timer(0.02, physics)
    node.create_timer(0.1, permit)

    stop = threading.Event()
    q = queue.Queue(maxsize=1)
    period = 1.0 / a.frame_hz

    def renderer():
        k = 0
        nxt = time.monotonic()
        while not stop.is_set():
            nxt += period
            with st_lock:
                pose = (st["x"], st["y"], st["yaw"])
            t_cap = now_ns()
            codes = C.render_codes(pose, world, T_bc=T_bc)
            k += 1
            # the car's acquisition stamp is the RAW arrival of the depth message at the producer (host.geo_producer
            # docstring; the pose is looked up at the depth header stamp = capture): render = the sensor, so the
            # frame "arrives" when it is rendered, with the pose of its capture
            t_acq = now_ns()
            item = (k, t_acq, pose, codes, t_cap)
            try:
                q.put_nowait(item)
            except queue.Full:                       # the worker is late: the newest frame wins
                try:
                    q.get_nowait()
                    write({"event": "render_overwrite", "t": now_ns(), "k": k})
                except queue.Empty:
                    pass
                q.put_nowait(item)
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()

    def worker():
        while not stop.is_set():
            try:
                k, t_acq, pose, codes, t_rend = q.get(timeout=0.2)   # t_rend = capture
            except queue.Empty:
                continue
            T_oc = C.odom_base_T(*pose) @ T_bc
            t0 = now_ns()
            with lock:
                r = prod.process(codes, C.K_BOARD, depth_stamp_ns=k, acquired_ns=t_acq, T_odom_camera=T_oc,
                                 robot_position_m=(pose[0], pose[1], 0.0), calibration_id="crawl-board-ng3")
            t1 = now_ns()
            row = {"event": "frame", "k": k, "t_acq": t_acq, "pose": pose, "render_ms": (t_acq - t_rend) / 1e6,
                   "wait_ms": (t0 - t_acq) / 1e6, "process_ms": (t1 - t0) / 1e6}
            d = r.get("delta")
            stats = r.get("stats") or {}
            row["nav_reason"] = stats.get("nav_reason", r.get("nav_reason"))
            row["fusion_reason"] = stats.get("fusion_reason", r.get("fusion_reason"))
            if d is not None:
                link.publish(d)
                row.update(seq=d.delta_seq, n_changes=len(d.changes), age_ms=(now_ns() - t_acq) / 1e6)
            write(row)

    th = [threading.Thread(target=renderer, daemon=True), threading.Thread(target=worker, daemon=True)]
    for t in th:
        t.start()
    end = time.monotonic() + a.seconds
    term = [False]
    import signal
    signal.signal(signal.SIGTERM, lambda *_: term.__setitem__(0, True))   # a plain flag (no Event.set here)
    try:
        while rclpy.ok() and time.monotonic() < end and not term[0]:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for t in th:
            t.join(timeout=2)
        write({"event": "end", "t": now_ns(), "blocked": st["blocked"], "min_clear": None if st["min_clear"] ==
               float("inf") else st["min_clear"], "producer_counts": dict(prod.counts), "link": dict(link.counts),
               "cmds": st["cmds"]})
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
