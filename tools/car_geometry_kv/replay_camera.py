#!/usr/bin/env python3
"""Replay camera for the ROS runtime board runs (car geometry source switch, 2026-10-11).

Plays a prepared recording (depth_u16.bin + meta.json, e.g. rec30) as the camera driver would:
at a fixed rate it publishes, all with one ``header.stamp = node now`` and
``frame_id = camera_optical_frame``,
  /fpga/camera_info  (K from meta, d = 0, the image size)
  /fpga/rgb          rgb8, all zero except row 0 pixels 0..5 carrying the source index (see
                    ``encode_index``; the stub detector reads it, so detections depend on the
                    source frame, not on how many frames each mode happened to drop)
  /fpga/depth        16UC1, the recording's raw codes (no requantization)
and once, latched, /tf_static: map->base_link and base_link->camera_optical_frame (identity;
the prepared sets are fixed-camera, world = camera).
Loops the recording until --duration-s. Every published frame gets one CSV row:
  seq, source_idx, stamp_ns (ROS header stamp), publish_raw_ns (host/fpga_clock RAW, taken just
  before the rgb publish), so the runtime's RAW rx stamp of the same frame gives publish->rx.

Pure helpers (encode_index / decode_index / load_prep / camera_info_fields) import without ROS.
Usage (inside the ROS image): replay_camera.py --prep DIR --csv NEW.csv --hz 10 --duration-s 150
"""
import argparse
import array
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

MAGIC = (0x50, 0x4C, 0x44)          # "FPGA" in pixel (0, 4)/(0, 5) channels: marks an encoded rgb


def encode_index(rgb, idx):
    """Write source index idx (0 <= idx < 2**32) into row 0 of an rgb8 array, in place."""
    if not 0 <= int(idx) < 2 ** 32:
        raise ValueError("index out of range")
    b = int(idx).to_bytes(4, "little")
    rgb[0, 0:4, 0] = list(b)
    rgb[0, 4, :] = MAGIC
    rgb[0, 5, :] = [b[0] ^ 0xFF, b[1] ^ 0xFF, b[2] ^ 0xFF]     # check bytes
    return rgb


def decode_index(rgb):
    """Source index from an encoded rgb, or None if it does not carry one (or is damaged)."""
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[1] < 6 or rgb.shape[2] != 3:
        return None
    if tuple(int(v) for v in rgb[0, 4, :]) != MAGIC:
        return None
    b = [int(v) for v in rgb[0, 0:4, 0]]
    if [int(v) for v in rgb[0, 5, :]] != [b[0] ^ 0xFF, b[1] ^ 0xFF, b[2] ^ 0xFF]:
        return None
    return int.from_bytes(bytes(b), "little")


def as_u8_array(a):
    """uint8 array.array of the buffer: rclpy's uint8[] setter takes array.array as is; given bytes
    it range-checks every element in Python (~190 ms per 640x360 rgb+depth on the Nano)."""
    out = array.array("B")
    out.frombytes(np.ascontiguousarray(a).tobytes())
    return out


def load_prep(prep):
    meta = json.load(open(os.path.join(prep, "meta.json")))
    w, h, n = int(meta["width"]), int(meta["height"]), int(meta["frames"])
    depth = np.memmap(os.path.join(prep, meta["depth_file"]), dtype="<u2", mode="r")
    if depth.size != w * h * n:
        raise ValueError("depth file size disagrees with meta")
    return meta, depth.reshape(n, h, w)


def camera_info_fields(meta):
    K = np.asarray(meta["K"], dtype=np.float64).reshape(3, 3)
    P = np.zeros((3, 4))
    P[:, :3] = K
    return {"width": int(meta["width"]), "height": int(meta["height"]),
            "k": [float(v) for v in K.ravel()], "d": [0.0] * 5, "distortion_model": "plumb_bob",
            "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], "p": [float(v) for v in P.ravel()]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--csv", required=True, help="new file")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--duration-s", type=float, required=True)
    ap.add_argument("--start-delay-s", type=float, default=2.0,
                    help="publish camera_info/TF this long before the first image")
    ap.add_argument("--camera-frame", default="camera_optical_frame")
    a, ros_args = ap.parse_known_args(argv)
    meta, depth = load_prep(a.prep)
    n, h, w = depth.shape
    unit = float(meta["depth_unit_m"])

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from host.fpga_clock import now_ns

    out = open(a.csv, "x")
    out.write("seq,source_idx,stamp_ns,publish_raw_ns\n")
    info_fields = camera_info_fields(meta)

    class Replay(Node):
        def __init__(self):
            super().__init__("fpga_replay_camera")
            self.info_pub = self.create_publisher(CameraInfo, "/fpga/camera_info", qos_profile_sensor_data)
            self.rgb_pub = self.create_publisher(Image, "/fpga/rgb", qos_profile_sensor_data)
            self.depth_pub = self.create_publisher(Image, "/fpga/depth", qos_profile_sensor_data)
            self.tf = StaticTransformBroadcaster(self)
            stamp = self.get_clock().now().to_msg()
            tfs = []
            for parent, child in (("map", "base_link"), ("base_link", a.camera_frame)):
                t = TransformStamped()
                t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
                t.transform.rotation.w = 1.0
                tfs.append(t)
            self.tf.sendTransform(tfs)
            self.seq = 0
            self.t0 = self.get_clock().now().nanoseconds
            self.rgb = np.zeros((h, w, 3), dtype=np.uint8)
            self.done = False
            self.create_timer(1.0 / a.hz, self.tick)

        def header(self, msg, stamp):
            msg.header.stamp, msg.header.frame_id = stamp, a.camera_frame
            return msg

        def info(self, stamp):
            m = self.header(CameraInfo(), stamp)
            m.width, m.height = info_fields["width"], info_fields["height"]
            m.k, m.d, m.r, m.p = info_fields["k"], info_fields["d"], info_fields["r"], info_fields["p"]
            m.distortion_model = info_fields["distortion_model"]
            return m

        def tick(self):
            now = self.get_clock().now()
            elapsed = (now.nanoseconds - self.t0) / 1e9
            stamp = now.to_msg()
            self.info_pub.publish(self.info(stamp))
            if elapsed < a.start_delay_s:
                return
            if elapsed > a.start_delay_s + a.duration_s:
                self.done = True
                return
            idx = self.seq % n
            encode_index(self.rgb, idx)
            rgb = self.header(Image(), stamp)
            rgb.height, rgb.width, rgb.encoding, rgb.is_bigendian, rgb.step = h, w, "rgb8", 0, w * 3
            rgb.data = as_u8_array(self.rgb)
            d = self.header(Image(), stamp)
            d.height, d.width, d.encoding, d.is_bigendian, d.step = h, w, "16UC1", 0, w * 2
            d.data = as_u8_array(np.ascontiguousarray(depth[idx]).astype("<u2"))
            raw = now_ns()
            self.rgb_pub.publish(rgb)
            self.depth_pub.publish(d)
            out.write("%d,%d,%d,%d\n" % (self.seq, idx, now.nanoseconds, raw))
            self.seq += 1

    rclpy.init(args=ros_args)
    node = Replay()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        print(json.dumps({"published": node.seq, "source_frames": n, "depth_unit_m": unit}), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
