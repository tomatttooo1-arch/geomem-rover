#!/usr/bin/env python3
"""Simulation depth adapter (line D1, 2026-10-14): Gazebo RGB-D -> the car chain's inputs.

Formalised from the sil-assess-20261014 smoke-3 stand-in. Gazebo Fortress publishes depth as
32FC1 metres (inf beyond the far clip); the car chain (host/ros_geo_producer.py, the KV260
link) takes 16UC1 millimetre codes like the Gemini 2 Y16 stream. This node:

  in   /fpga/depth        sensor_msgs/Image 32FC1 metres (ros_gz_bridge)
       /fpga/camera_info  sensor_msgs/CameraInfo (only for the obstacle cloud)
  out  /fpga/depth16      sensor_msgs/Image 16UC1 mm, same header; inf / nan / outside
                         [--min-m, --max-m] -> 0 (= no return, as the Gemini 2 stream)
       /fpga/points       sensor_msgs/PointCloud2 xyz float32 in the depth header's frame
                         (camera_optical_frame, REP-103 optical axes), every --cloud-stride-th
                         pixel of the same valid depth; only with --cloud. Stand-in for the
                         Gazebo cloud, whose Fortress XYZ is forward-left-up although its header
                         says camera_optical_frame (host/ros_tunnel.py --fortress-cloud-frame).
       /fpga/sensor_source  latched marker {"sensor_source": "sim", ...}: the producer's
                         SourceMarker cross-checks it against --data-source sim.

It publishes NO TF (the camera's static TF is navigation.launch.py's base_to_camera /
camera_to_optical, map->odom its reference identity; odom->base_link is Gazebo's). The
--summary file (written on exit) counts frames and checks every output is finite.
"""
import argparse
import json
import math
import signal
import sys

import numpy as np

MARKER = {"sensor_source": "sim", "replay_id": None, "publisher": "robot_sim/fpga_sim/scripts/sim_adapter.py"}


def depth_to_mm(depth_m, min_m=0.15, max_m=10.0):
    """32FC1 metres -> (uint16 mm codes, valid mask). Invalid -> 0."""
    d = np.asarray(depth_m, dtype=np.float32)
    ok = np.isfinite(d) & (d >= min_m) & (d <= max_m)
    codes = np.zeros(d.shape, np.uint16)
    codes[ok] = np.rint(d[ok] * 1000.0).astype(np.uint16)
    return codes, ok


def cloud_points(depth_m, ok, k, stride):
    """Optical-frame XYZ (N x 3 float32) of every stride-th valid pixel."""
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if not all(math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        raise ValueError("camera_info K not usable")
    h, w = depth_m.shape
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    sel = ok[0:h:stride, 0:w:stride]
    z = np.asarray(depth_m, np.float32)[0:h:stride, 0:w:stride][sel]
    x = (us[sel].astype(np.float32) - cx) * z / fx
    y = (vs[sel].astype(np.float32) - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--depth-in", default="/fpga/depth")
    ap.add_argument("--depth-out", default="/fpga/depth16")
    ap.add_argument("--info-topic", default="/fpga/camera_info")
    ap.add_argument("--min-m", type=float, default=0.15)
    ap.add_argument("--max-m", type=float, default=10.0)
    ap.add_argument("--cloud", action="store_true", help="also publish /fpga/points (see above)")
    ap.add_argument("--cloud-topic", default="/fpga/points")
    ap.add_argument("--cloud-stride", type=int, default=2)
    ap.add_argument("--summary", help="new JSON file written on exit")
    a, ros_args = ap.parse_known_args(argv)
    if a.cloud_stride < 1 or not 0 < a.min_m < a.max_m:
        ap.error("bad stride or range")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from std_msgs.msg import String

    stats = {"depth_in": 0, "depth_out": 0, "cloud_out": 0, "cloud_skipped_no_info": 0,
             "bad_encoding": 0, "nonfinite_out": 0, "valid_fraction_min": None, "valid_fraction_max": None,
             "valid_fraction_last": None, "cloud_points_last": None, "median_mm_last": None, "frame_id": None,
             "size": None}

    class Adapter(Node):
        def __init__(self):
            super().__init__("fpga_sim_adapter")
            self.k = None
            self.pub = self.create_publisher(Image, a.depth_out, qos_profile_sensor_data)
            self.cloud = self.create_publisher(PointCloud2, a.cloud_topic, qos_profile_sensor_data) if a.cloud else None
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.marker = self.create_publisher(String, "/fpga/sensor_source", latched)
            self.marker.publish(String(data=json.dumps(MARKER)))
            self.create_subscription(Image, a.depth_in, self.on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, a.info_topic, self.on_info, qos_profile_sensor_data)

        def on_info(self, m):
            self.k = list(m.k)

        def on_depth(self, m):
            stats["depth_in"] += 1
            if m.encoding != "32FC1":
                stats["bad_encoding"] += 1
                return
            d = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(m.height, m.width)
            codes, ok = depth_to_mm(d, a.min_m, a.max_m)
            out = Image(header=m.header, height=m.height, width=m.width, encoding="16UC1",
                        is_bigendian=0, step=m.width * 2, data=codes.tobytes())
            self.pub.publish(out)
            stats["depth_out"] += 1
            frac = float(ok.mean())
            stats["valid_fraction_last"] = frac
            stats["valid_fraction_min"] = frac if stats["valid_fraction_min"] is None else min(stats["valid_fraction_min"], frac)
            stats["valid_fraction_max"] = frac if stats["valid_fraction_max"] is None else max(stats["valid_fraction_max"], frac)
            stats["median_mm_last"] = int(np.median(codes[ok])) if ok.any() else None
            stats["frame_id"], stats["size"] = m.header.frame_id, [m.width, m.height]
            if self.cloud is not None:
                if self.k is None:
                    stats["cloud_skipped_no_info"] += 1
                else:
                    pts = cloud_points(d, ok, self.k, a.cloud_stride)
                    if not np.isfinite(pts).all():
                        stats["nonfinite_out"] += 1
                        return
                    c = PointCloud2(header=m.header, height=1, width=int(pts.shape[0]), is_bigendian=False,
                                    is_dense=True, point_step=12, row_step=12 * int(pts.shape[0]),
                                    fields=[PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                                            for i, n in enumerate("xyz")], data=pts.tobytes())
                    self.cloud.publish(c)
                    stats["cloud_out"] += 1
                    stats["cloud_points_last"] = int(pts.shape[0])
            if stats["depth_out"] % 100 == 1:
                self.get_logger().info(json.dumps(stats))

    rclpy.init(args=ros_args)
    node = Adapter()

    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if a.summary:
            with open(a.summary, "x") as f:
                json.dump(dict(stats, args=vars(a), marker=MARKER), f, indent=1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
