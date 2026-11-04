#!/usr/bin/env python3
"""Runtime probe, not an online detector: initial scene depth + stop controls.

Run with only simulation.launch.py; do not run concurrently with a task runner
or other /cmd_vel publisher. Uses direct drive commands only when --drive is set.
"""
import argparse
import json
import math
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from geometry_calibration import plane_samples


class Probe(Node):
    def __init__(self):
        super().__init__("fpga_sim_acceptance_probe")
        self.frames = {"rgb": {}, "depth": {}, "camera_info": {}}
        self.latest = {}
        for topic, kind in [("rgb", Image), ("depth", Image), ("camera_info", CameraInfo)]:
            self.create_subscription(kind, "/fpga/" + topic,
                                     lambda msg, key=topic: self.frame(key, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/fpga/points",
                                 lambda msg: self.latest.update(points=msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom",
                                 lambda msg: self.latest.update(odom=msg), qos_profile_sensor_data)
        self.create_subscription(Twist, "/fpga/cmd_vel_safe", self.safe, 10)
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.permit_pub = self.create_publisher(Bool, "/fpga/motion_permit", 1)
        self.safe_commands = []

    def safe(self, msg):
        self.safe_commands.append((time.monotonic(), msg.linear.x, msg.angular.z))
        self.safe_commands = self.safe_commands[-200:]

    def frame(self, key, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        bucket = self.frames[key]
        bucket[stamp] = msg
        while len(bucket) > 20:
            del bucket[next(iter(bucket))]

    def spin_for(self, seconds, command=None, permit=None):
        deadline = time.monotonic() + seconds
        next_publish = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_publish:
                if command is not None:
                    msg = Twist(); msg.linear.x = command
                    self.command_pub.publish(msg)
                if permit is not None:
                    msg = Bool(); msg.data = permit
                    self.permit_pub.publish(msg)
                next_publish = now + .08

    def zero_window(self, start):
        rows = [row for row in self.safe_commands if row[0] >= start + .8]
        return bool(rows) and all(abs(x) < 1e-9 and abs(yaw) < 1e-9 for _, x, yaw in rows)


def main():
    args = argparse.ArgumentParser()
    args.add_argument("--drive", action="store_true")
    options = args.parse_args()
    rclpy.init()
    node = Probe()
    result = {"probe": "initial_scene_sensor_and_stop_controls", "checks": {}}
    try:
        deadline = time.monotonic() + 60
        common = set()
        while time.monotonic() < deadline:
            node.spin_for(.2)
            common = set.intersection(*(set(v) for v in node.frames.values()))
            if common and "points" in node.latest and "odom" in node.latest:
                break
        if not common or "points" not in node.latest or "odom" not in node.latest:
            raise RuntimeError("sensor timeout: no synchronized RGB/depth/info + points + odometry")
        stamp = max(common)
        rgb, depth, info = [node.frames[k][stamp] for k in ("rgb", "depth", "camera_info")]
        result["sensor"] = {"stamp": stamp, "rgb_encoding": rgb.encoding,
                            "depth_encoding": depth.encoding, "size": [rgb.width, rgb.height],
                            "frame_ids": [rgb.header.frame_id, depth.header.frame_id,
                                          info.header.frame_id, node.latest["points"].header.frame_id],
                            "intrinsics": list(info.k)}
        result["checks"]["optical_frame"] = all(x == "camera_optical_frame" for x in result["sensor"]["frame_ids"])
        result["checks"]["image_contract"] = (rgb.encoding == "rgb8" and depth.encoding == "32FC1"
                                                     and rgb.width == depth.width == 320
                                                     and rgb.height == depth.height == 240)
        if depth.encoding != "32FC1":
            raise RuntimeError("depth is not 32FC1; refusing to interpret bytes")
        calibration = plane_samples(
            Path(get_package_share_directory("fpga_sim")) / "worlds/room.sdf",
            list(info.k), node.latest["odom"].pose.pose)
        for sample in calibration:
            u, v = sample["pixel"]
            observed = struct.unpack_from(">f" if depth.is_bigendian else "<f",
                                          depth.data, v*depth.step+u*4)[0]
            valid = math.isfinite(observed) and observed > 0
            sample["observed_optical_z_m"] = observed if valid else None
            sample["passed"] = bool(valid and abs(observed-sample["expected_optical_z_m"]) < .04)
        result["depth_calibration"] = calibration
        result["checks"]["optical_depth_two_planes_m"] = all(s["passed"] for s in calibration)
        if options.drive:
            start = time.monotonic(); node.spin_for(1.5, command=.10, permit=None)
            result["checks"]["no_permit_stops"] = node.zero_window(start)
            before = node.latest["odom"].pose.pose.position.x
            node.spin_for(4.0, command=.10, permit=True)
            after = node.latest["odom"].pose.pose.position.x
            result["motion"] = {"start_x_m": before, "end_x_m": after}
            result["checks"]["permitted_command_moves"] = after-before > .02
            start = time.monotonic(); node.spin_for(1.5, command=.10, permit=None)
            result["checks"]["permit_expiry_stops_with_live_commands"] = node.zero_window(start)
            node.spin_for(.5, command=.10, permit=True)
            start = time.monotonic(); node.spin_for(1.5, command=None, permit=True)
            result["checks"]["command_expiry_stops_with_live_permit"] = node.zero_window(start)
            start = time.monotonic(); node.spin_for(1.0, command=.10, permit=False)
            result["checks"]["false_permit_stops"] = node.zero_window(start)
        result["passed"] = all(result["checks"].values())
    except Exception as exc:
        result["error"] = str(exc); result["passed"] = False
    finally:
        if options.drive:
            node.spin_for(.3, command=0.0, permit=False)
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(result, indent=2, allow_nan=False))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
