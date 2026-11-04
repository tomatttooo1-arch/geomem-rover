#!/usr/bin/env python3
"""N5 stand-ins for the parts that are not on the Nano (docs/design/NANO_PARTITION_LOAD_20260927.md §2).

Roles:
  record  (server, no ROS) run the N3/N4 smoke producer geometry (synthetic depth -> M7Pipeline(cpu)
          -> RollingFusion) along a scripted trajectory and save every encoded delta to a file.
  replay  (Nano, ROS) the KV260 PL stand-in: publish the recorded deltas at 10 Hz on
          /fpga/rolling_map/delta. No geometry: per frame it only rewrites the JSON header
          (epoch, delta_seq, acquired_ns = now, expires_at_ns = now + recorded TTL) and sends the
          recorded payload. A resync request from the bridge starts a new epoch at delta_seq 1
          (the real producer would send a snapshot; the replay has no map to snapshot).
          Writes scene.jsonl rows in the N4 format (event frame, t_acq, t_pub, accepted) so
          tools/nano_fullload/summarize.py reads them unchanged.
  base    (Nano, ROS) the virtual chassis of the N3/N4 scene (kinematic robot, /odom + TF at 50 Hz,
          /fpga/cmd_vel_safe, /smoke/control script drive). No depth, no geometry.

The replayed map does not follow the virtual chassis (it follows the recorded trajectory); it is
there to load the bridge / Nav2 with the real message sizes and rates, not to be a correct map.
"""
import argparse
import json
import math
import os
import signal
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from host.fpga_clock import CLOCK_ID  # noqa: E402  (the bridge's clock id since 2026-10-11)

MAGIC = b"N5DL"          # file: MAGIC, then per frame: u32 header_len, header JSON, u32 payload_len, payload
DELTA_MAGIC = b"RDL1"    # host/rolling_nav_layer.py wire format
FRAME_HZ = 10.0
SCRIPT_FRAMES = 60       # 6 s at 0.1 m/s from START (the N4 consumer's script drive)
LOOP_WAYPOINTS = [(0.30, 0.30), (1.15, 0.30), (0.30, 0.30), (0.10, 0.10)]


def mono():
    # The bridge ages deltas on host.fpga_clock (CLOCK_MONOTONIC_RAW) since 2026-10-11.
    from host.fpga_clock import now_ns
    return now_ns()


# ----------------------------------------------------------------------------- file format
def split_wire(raw):
    """Wire delta -> (header dict, payload bytes)."""
    if raw[:4] != DELTA_MAGIC:
        raise ValueError("bad magic")
    (n,) = struct.unpack_from("<I", raw, 4)
    return json.loads(raw[8:8 + n].decode("utf-8")), bytes(raw[8 + n:])


def join_wire(header, payload):
    h = json.dumps(header, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return DELTA_MAGIC + struct.pack("<I", len(h)) + h + payload


def write_frames(path, frames):
    with open(path, "wb") as f:
        f.write(MAGIC)
        for header, payload in frames:
            h = json.dumps(header, sort_keys=True, separators=(",", ":")).encode("utf-8")
            f.write(struct.pack("<I", len(h)) + h + struct.pack("<I", len(payload)) + payload)


def read_frames(path):
    data = Path(path).read_bytes()
    if data[:4] != MAGIC:
        raise ValueError("not an N5 delta file")
    out, i = [], 4
    while i < len(data):
        (n,) = struct.unpack_from("<I", data, i)
        header = json.loads(data[i + 4:i + 4 + n].decode("utf-8"))
        i += 4 + n
        (m,) = struct.unpack_from("<I", data, i)
        out.append((header, data[i + 4:i + 4 + m]))
        i += 4 + m
    return out


class ReplaySequencer:
    """Pure bookkeeping of the replay (unit-tested without ROS)."""

    def __init__(self, frames, loop_from):
        if not frames or not 0 <= loop_from < len(frames):
            raise ValueError("need frames and 0 <= loop_from < len(frames)")
        self.frames, self.loop_from = frames, loop_from
        self.i, self.epoch, self.seq = 0, 1, 0
        self.ttls = [h["expires_at_ns"] - h["acquired_ns"] for h, _ in frames]

    def new_epoch(self):
        self.epoch += 1
        self.seq = 0

    def next_wire(self, now_ns):
        header, payload = self.frames[self.i]
        ttl = self.ttls[self.i]
        self.i = self.i + 1 if self.i + 1 < len(self.frames) else self.loop_from
        self.seq += 1
        h = dict(header, epoch=self.epoch, delta_seq=self.seq, acquired_ns=int(now_ns),
                 expires_at_ns=int(now_ns) + ttl, clock_id=CLOCK_ID)
        return join_wire(h, payload), h


# ----------------------------------------------------------------------------- record (server)
def trajectory():
    """Poses (x, y, yaw) at 10 Hz: script drive, then two loops over LOOP_WAYPOINTS."""
    from robot_sim.tests.rolling_nav2_smoke import START
    x, y, yaw = START[0], START[1], 0.0
    poses = []
    for _ in range(SCRIPT_FRAMES):
        x += 0.1 / FRAME_HZ
        poses.append((x, y, yaw))
    for _ in range(2):
        for gx, gy in LOOP_WAYPOINTS:
            want = math.atan2(gy - y, gx - x)
            while True:  # turn in place at 0.8 rad/s
                err = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
                if abs(err) < 1e-6:
                    break
                yaw += max(-0.08, min(0.08, err))
                poses.append((x, y, yaw))
            while True:  # drive at 0.2 m/s
                dist = math.hypot(gx - x, gy - y)
                if dist < 1e-6:
                    break
                step = min(0.02, dist)
                x += step * math.cos(yaw)
                y += step * math.sin(yaw)
                poses.append((x, y, yaw))
    return poses


def run_record(args):
    import tempfile
    import numpy as np
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    from host.m7_grid_policy import FollowCameraGrid
    from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
    from host.m7_rolling import RollingFusion, compile_rolling
    from host.rolling_nav_layer import encode_delta
    from robot_sim.tests.rolling_nav2_smoke import BOX_A, VOXEL, WALLS, intrinsics, render

    tmp = Path(tempfile.mkdtemp(prefix="n5-record-"))
    selector = M7BackendSelector(cpu_frame_library=compile_frame_native(tmp / "frame.so"))
    shape = (32, 32, 10)
    pipeline = M7Pipeline(selector.frame_backend("cpu"), GridSpec((0., 0., 0.), VOXEL, shape),
                          grid_policy=FollowCameraGrid(VOXEL, shape, back_cells=1, height_cell=7))
    fusion = RollingFusion(selector.rolling_backend("cpu", cpu_library=compile_rolling(tmp / "r.so")),
                           voxel_m=VOXEL)
    frames, stats = [], {"poses": 0, "accepted": 0, "rejected": {}}
    for k, pose in enumerate(trajectory(), 1):
        stats["poses"] += 1
        depth, r, c = render(pose, WALLS + [BOX_A])
        tf = np.eye(4)
        tf[:3, :3], tf[:3, 3] = r, c
        t = mono()
        ident = CaptureIdentity("cam0", k, t, CLOCK_ID, "cal-smoke", "odom", "e1")
        out = pipeline.ingest_geometry(
            DepthFrame(ident, depth, intrinsics(), PoseAtCapture(t, CLOCK_ID, "odom", "e1", True, tf)),
            now_ns=mono(), now_clock_id=CLOCK_ID)
        fused = fusion.ingest(out.navigation, pipeline, now_ns=mono(), clock_id=CLOCK_ID,
                              robot_position_m=(pose[0], pose[1], 0.0))
        if not fused.accepted:
            stats["rejected"][fused.reason] = stats["rejected"].get(fused.reason, 0) + 1
            continue
        stats["accepted"] += 1
        frames.append(split_wire(encode_delta(fused.delta)))
    write_frames(args.out, frames)
    stats.update(frames=len(frames), loop_from=SCRIPT_FRAMES, bytes=os.path.getsize(args.out),
                 changes_per_frame_max=max(h["n"] for h, _ in frames),
                 changes_total=sum(h["n"] for h, _ in frames))
    Path(str(args.out) + ".json").write_text(json.dumps(stats, indent=1))
    print(json.dumps(stats))


# ----------------------------------------------------------------------------- replay (Nano)
def run_replay(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, UInt8MultiArray
    from host.ros_rolling_nav import STREAM_QOS

    frames = read_frames(args.deltas)
    seqr = ReplaySequencer(frames, min(SCRIPT_FRAMES, len(frames) - 1))
    rclpy.init()
    node = Node("fpga_n5_map_replay")
    pub = node.create_publisher(UInt8MultiArray, "/fpga/rolling_map/delta", STREAM_QOS)
    counts = {"published": 0, "resync_requests": 0, "new_epochs": 0}
    args.out.mkdir(parents=True, exist_ok=True)
    log = open(args.out / "scene.jsonl", "w", buffering=1 << 16)

    def on_request(msg):
        counts["resync_requests"] += 1
        seqr.new_epoch()
        counts["new_epochs"] += 1
        log.write(json.dumps({"event": "resync_request", "t": mono(), "req": msg.data,
                              "new_epoch": seqr.epoch}) + "\n")

    node.create_subscription(String, "/fpga/rolling_map/resync_request", on_request, 10)
    state = {"frame": 0}

    def tick():
        t = mono()
        state["frame"] += 1
        wire, h = seqr.next_wire(t)
        m = UInt8MultiArray()
        m.data = wire
        pub.publish(m)
        counts["published"] += 1
        log.write(json.dumps({"event": "frame", "t_acq": t, "frame": state["frame"], "accepted": True,
                              "epoch": h["epoch"], "seq": h["delta_seq"], "n_changes": h["n"],
                              "t_pub": mono(), "link": counts}) + "\n")

    node.create_timer(1.0 / FRAME_HZ, tick)
    spin(node, rclpy)
    log.close()


# ----------------------------------------------------------------------------- base (Nano)
def run_base(args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from robot_sim.tests.rolling_nav2_smoke import BOX_A, ROBOT_RADIUS, START, WALLS, rect_distance

    rclpy.init()
    node = Node("fpga_n5_virtual_base")
    tfb = TransformBroadcaster(node)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    s = {"x": START[0], "y": START[1], "yaw": 0.0, "v": 0.0, "w": 0.0, "cmd_t": 0, "script": None,
         "blocked": 0, "last": mono()}
    args.out.mkdir(parents=True, exist_ok=True)
    log = open(args.out / "base.jsonl", "w", buffering=1)

    def cmd(msg):
        if s["script"] is None:
            s["v"], s["w"], s["cmd_t"] = msg.linear.x, msg.angular.z, mono()

    def control(msg):
        c = json.loads(msg.data)
        if c.get("cmd") == "script":
            s["script"] = (float(c["v"]), mono() + int(c["seconds"] * 1e9))
        log.write(json.dumps({"event": "control", "t": mono(), **c}) + "\n")

    node.create_subscription(Twist, "/fpga/cmd_vel_safe", cmd, 10)
    node.create_subscription(String, "/smoke/control", control, 10)

    def physics():  # same kinematics as robot_sim/tests/rolling_nav2_smoke.py run_scene
        t = mono()
        dt = (t - s["last"]) / 1e9
        s["last"] = t
        if s["script"] is not None:
            v, until = s["script"]
            if t >= until:
                s["script"] = None
                s["v"] = s["w"] = 0.0
            else:
                s["v"], s["w"] = v, 0.0
        elif t - s["cmd_t"] > 300_000_000:
            s["v"] = s["w"] = 0.0
        nx = s["x"] + s["v"] * math.cos(s["yaw"]) * dt
        ny = s["y"] + s["v"] * math.sin(s["yaw"]) * dt
        if any(rect_distance(nx, ny, b) < ROBOT_RADIUS for b in WALLS + [BOX_A]):
            if s["v"] != 0.0:
                s["blocked"] += 1
        else:
            s["x"], s["y"] = nx, ny
        s["yaw"] += s["w"] * dt
        stamp = node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = s["x"], s["y"]
        tf.transform.rotation.z = math.sin(s["yaw"] / 2)
        tf.transform.rotation.w = math.cos(s["yaw"] / 2)
        tfb.sendTransform(tf)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "odom", "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = s["x"], s["y"]
        od.pose.pose.orientation = tf.transform.rotation
        od.twist.twist.linear.x, od.twist.twist.angular.z = s["v"], s["w"]
        odom_pub.publish(od)

    def heartbeat():
        log.write(json.dumps({"event": "pose", "t": mono(), "x": round(s["x"], 3), "y": round(s["y"], 3),
                              "yaw": round(s["yaw"], 3), "blocked": s["blocked"]}) + "\n")

    node.create_timer(0.02, physics)
    node.create_timer(1.0, heartbeat)
    spin(node, rclpy)
    log.close()


def spin(node, rclpy):
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("role", choices=("record", "replay", "base"))
    ap.add_argument("--out", type=Path, required=True, help="record: delta file; replay/base: log dir")
    ap.add_argument("--deltas", type=Path, help="replay: the recorded delta file")
    a = ap.parse_args()
    {"record": run_record, "replay": run_replay, "base": run_base}[a.role](a)


if __name__ == "__main__":
    main()
