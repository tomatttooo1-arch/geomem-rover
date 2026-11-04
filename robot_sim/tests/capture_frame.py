#!/usr/bin/env python3
"""Save one synchronized simulator RGB/depth pair without labels or boxes."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image
import rclpy

from probe_ros import Probe


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-revision", required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit("Refusing to overwrite existing capture directory")
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + 30
        common = set()
        while time.monotonic() < deadline:
            node.spin_for(.1)
            common = set.intersection(*(set(v) for v in node.frames.values()))
            if common:
                break
        if not common:
            raise RuntimeError("Timed out waiting for synchronized RGB/depth/CameraInfo")
        stamp = max(common)
        rgb, depth, info = [node.frames[key][stamp] for key in ("rgb", "depth", "camera_info")]
        if rgb.encoding != "rgb8" or depth.encoding != "32FC1":
            raise RuntimeError("Unsupported image encoding; refusing implicit reinterpretation")
        color = np.ndarray((rgb.height, rgb.width, 3), dtype=np.uint8,
                           buffer=rgb.data, strides=(rgb.step, 3, 1)).copy()
        meters = np.ndarray((depth.height, depth.width),
                            dtype=">f4" if depth.is_bigendian else "<f4",
                            buffer=depth.data, strides=(depth.step, 4)).copy()
        args.output.mkdir(parents=True, exist_ok=False)
        Image.fromarray(color, "RGB").save(args.output / "rgb.png")
        np.save(args.output / "depth_optical_z_m.npy", meters, allow_pickle=False)
        metadata = {
            "source_revision": args.source_revision,
            "source": "Gazebo Fortress RGB-D sensor, no detection labels",
            "timestamp": {"sec": stamp[0], "nanosec": stamp[1]},
            "width": info.width, "height": info.height,
            "K": list(info.k), "D": list(info.d), "R": list(info.r), "P": list(info.p),
            "distortion_model": info.distortion_model,
            "rgb_frame": rgb.header.frame_id, "depth_frame": depth.header.frame_id,
            "camera_info_frame": info.header.frame_id,
            "rgb_encoding": rgb.encoding, "depth_encoding": depth.encoding,
            "depth_unit": "metre", "depth_definition": "optical_z",
            "sha256": {name: hashlib.sha256((args.output / name).read_bytes()).hexdigest()
                       for name in ("rgb.png", "depth_optical_z_m.npy")},
        }
        (args.output / "camera_info.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(json.dumps({"capture": str(args.output), **metadata}, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
