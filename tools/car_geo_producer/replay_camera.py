#!/usr/bin/env python3
"""Replay camera + car for the geometry producer rig (no camera connected).

Publishes a prepared depth set (rec30 / TUM) at --hz as the car's camera would:
  /fpga/depth        16UC1 camera codes, header.stamp = ROS now, frame camera_optical_frame
  /fpga/camera_info  K of the prepared set, d = 0
  /tf               odom -> base_link along common.odom_base (stamped with the depth stamp,
                    sent just before the depth message, plus a 50 Hz stream)
  /tf_static        map -> odom (identity), base_link -> camera_optical_frame (common.base_camera)
  /fpga/memory       (optional, --memory-boxes N) a fixed D.1 body made from frame 0 so the
                    producer has ROI boxes to send; runtime_boot_id "replay-runtime", 1 Hz
  /fpga/rgb          (optional, --rgb) rgb8, zero except row 0 carrying the source index
                    (tools/car_geometry_kv/replay_camera.encode_index, read by the stub
                    detector), same stamp as the depth: lets ros_runtime run on this replay
                    (--geometry-source kv_topic; then leave --memory-boxes 0, the runtime
                    publishes /fpga/memory itself)
  /fpga/sensor_source  (latched: reliable + transient_local, once) the replay marker
                    {"sensor_source": "replay", "replay_id": <prep dir name>, "publisher":
                    "replay_camera"} (source_marker below); the producer cross-checks it
                    against its --data-source and reports it in every frame_stats body
Writes --log (new CSV): frame index, source index, ROS stamp ns, RAW publish ns (host.fpga_clock).
Loops the set until --duration-s.
--debug-skip-frames K[,K...] (known-answer control of the producer's input accounting, v8): at those
frame indices the camera sends NO camera_info / rgb / depth (the tick and its index still pass, /tf
goes on) and writes no --log row; each skip is printed ("debug skip k=K stamp_ns=...").
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.fpga_clock import now_ns  # noqa: E402
from tools.car_geometry_kv.replay_camera import as_u8_array  # noqa: E402
from tools.car_geo_producer.common import (base_camera, load_prep, memory_message,  # noqa: E402
                                           odom_base)


def source_marker(prep):
    """Body of the latched /fpga/sensor_source marker (host.geo_producer.SourceMarker reads it)."""
    return {"sensor_source": "replay", "replay_id": os.path.basename(os.path.normpath(prep)),
            "publisher": "replay_camera"}


def depth_wire(codes):
    """16UC1 Image.data for one frame of uint16 codes: little-endian bytes as a uint8
    array.array. rclpy's uint8[] setter takes array.array as is; given bytes it range-checks
    every element in Python (the replay's main CPU cost on the Nano). Same bytes as
    ``np.ascontiguousarray(codes, dtype="<u2").tobytes()``. The replay stands in for the
    camera driver, which is not part of the car's own CPU budget."""
    return as_u8_array(np.ascontiguousarray(codes, dtype="<u2"))


def tf_msg(TransformStamped, parent, child, T, stamp):
    m = TransformStamped()
    m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, parent, child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in T[:3, 3])
    R = T[:3, :3]
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w > 1e-6:
        x, y, z = (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)
    else:                               # 180-degree case (not used by the default mount)
        x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
        y, z = (R[0, 1] + R[1, 0]) / (4 * x), (R[0, 2] + R[2, 0]) / (4 * x)
        w = (R[2, 1] - R[1, 2]) / (4 * x)
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = \
        float(x), float(y), float(z), float(w)
    return m


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--duration-s", type=float, default=200.0)
    ap.add_argument("--memory-boxes", type=int, default=0)
    ap.add_argument("--mount-pitch-deg", type=float, default=None)
    ap.add_argument("--log", default=None)
    ap.add_argument("--speed-mps", type=float, default=0.2)
    ap.add_argument("--rgb", action="store_true", help="also publish /fpga/rgb (index-encoded, for the runtime)")
    ap.add_argument("--debug-skip-frames", default="", help=argparse.SUPPRESS)   # K[,K...]: frames not sent
    a, ros_args = ap.parse_known_args(argv)
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    meta, depth = load_prep(a.prep)
    n_src, h, w = depth.shape
    K = np.asarray(meta["K"], dtype=np.float64)
    Tbc = base_camera() if a.mount_pitch_deg is None else base_camera(pitch_deg=a.mount_pitch_deg)
    rclpy.init(args=ros_args)
    node = Node("fpga_replay_camera")
    depth_pub = node.create_publisher(Image, "/fpga/depth", qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, "/fpga/camera_info", qos_profile_sensor_data)
    tfb, stb = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    rgb_pub = rgb_buf = None
    if a.rgb:
        from tools.car_geometry_kv.replay_camera import encode_index
        rgb_pub = node.create_publisher(Image, "/fpga/rgb", qos_profile_sensor_data)
        rgb_buf = np.zeros((h, w, 3), dtype=np.uint8)
    stamp0 = node.get_clock().now().to_msg()
    stb.sendTransform([tf_msg(TransformStamped, "map", "odom", np.eye(4), stamp0),
                       tf_msg(TransformStamped, "base_link", "camera_optical_frame", Tbc, stamp0)])
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    marker_pub = node.create_publisher(String, "/fpga/sensor_source", latched)
    marker_pub.publish(String(data=json.dumps(source_marker(a.prep))))
    mem_pub = mem = None
    if a.memory_boxes > 0:
        from tools.kv_link.nano_replay import make_roi_memory
        unit = float(meta["depth_unit_m"])
        d0 = np.asarray(depth[0]).astype(np.float32) * np.float32(unit)
        d0[np.asarray(depth[0]) == 0] = np.nan
        mem = memory_message(make_roi_memory(d0, K, a.memory_boxes), odom_base(0) @ Tbc)
        mem_pub = node.create_publisher(String, "/fpga/memory", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    log = open(a.log, "x", newline="") if a.log else None
    writer = csv.writer(log) if log else None
    if writer:
        writer.writerow(["k", "src", "stamp_ns", "publish_raw_ns"])
    state = {"k": 0, "last_mem": 0.0}
    skip = set(int(v) for v in a.debug_skip_frames.split(",") if v.strip())

    def pose_now():
        k = state["k"]
        return odom_base(k, speed_mps=a.speed_mps, hz=a.hz)

    def tf_tick():
        tfb.sendTransform(tf_msg(TransformStamped, "odom", "base_link", pose_now(), node.get_clock().now().to_msg()))

    def frame_tick():
        k = state["k"]
        src = k % n_src
        stamp = node.get_clock().now().to_msg()
        tfb.sendTransform(tf_msg(TransformStamped, "odom", "base_link", pose_now(), stamp))
        if k in skip:                   # known-answer control: this frame is never sent
            print("debug skip k=%d stamp_ns=%d" % (k, stamp.sec * 10 ** 9 + stamp.nanosec), flush=True)
            state["k"] += 1
            return
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, "camera_optical_frame"
        info.width, info.height = w, h
        info.k = [float(v) for v in K.ravel()]
        # R and P too (10-15 car integration): rtabmap's camera model reads fx/fy/cx/cy from P (image_geometry); an
        # all-zero P is "Camera model 0 is invalid" -> no local occupancy grid -> a mapping run exports no pgm/yaml
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [float(K[0, 0]), 0.0, float(K[0, 2]), 0.0, 0.0, float(K[1, 1]), float(K[1, 2]), 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0] * 5
        info.distortion_model = "plumb_bob"
        info_pub.publish(info)
        if rgb_pub is not None:
            encode_index(rgb_buf, src)
            rgb = Image()
            rgb.header.stamp, rgb.header.frame_id = stamp, "camera_optical_frame"
            rgb.height, rgb.width, rgb.encoding, rgb.is_bigendian, rgb.step = h, w, "rgb8", 0, 3 * w
            rgb.data = as_u8_array(rgb_buf)
            rgb_pub.publish(rgb)
        img = Image()
        img.header.stamp, img.header.frame_id = stamp, "camera_optical_frame"
        img.height, img.width, img.encoding, img.is_bigendian, img.step = h, w, "16UC1", 0, 2 * w
        img.data = depth_wire(depth[src])
        depth_pub.publish(img)
        if writer:
            writer.writerow([k, src, stamp.sec * 10 ** 9 + stamp.nanosec, now_ns()])
        state["k"] += 1
        if mem_pub is not None and time.monotonic() - state["last_mem"] >= 1.0:
            m = dict(mem, stamp_ros_s=stamp.sec + stamp.nanosec / 1e9)
            mem_pub.publish(String(data=json.dumps(m)))
            state["last_mem"] = time.monotonic()

    node.create_timer(0.02, tf_tick)
    node.create_timer(1.0 / a.hz, frame_tick)
    end = time.monotonic() + a.duration_s
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if log:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
