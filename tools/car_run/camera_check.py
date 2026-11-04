#!/usr/bin/env python3
"""Line 4 (2026-10-15): is the real camera wired the way the runner will accept it? -- one message per topic.

The runner (host/ros_runtime.py prepare_images) drops every pair whose RGB, depth and CameraInfo header.frame_id are
not all its --camera-frame, whose RGB size differs from the CameraInfo size, whose CameraInfo has a non-zero
distortion coefficient, or whose encodings are not rgb8/bgr8 + 16UC1/32FC1 -- silently for the start-up check, which
then times out after RUNTIME_READY_S. car_inner.sh runs this right after the driver's depth stream is up, so a wrong
frame name / the colour (distorted) CameraInfo / an unaligned depth size fails at once with the reason.
Subscribes --color-topic, --depth-topic (sensor_msgs/Image) and --info-topic (CameraInfo; the DEPTH one: with depth
hardware-aligned to colour it carries the colour intrinsics, the colour frame and zero distortion), sensor-data QoS
(matches a reliable or best-effort publisher), plus any --record-info-topic (recorded, not judged). Waits for one of
each up to --timeout-s. One JSON line on stdout (also --out). Exit 0 ok, 1 a check failed, 3 a topic was silent,
2 rclpy / message import failed. Python 3.8 compatible; judge() needs no ROS."""
import argparse
import json
import sys
import time

RGB_ENCODINGS = ("rgb8", "bgr8")
DEPTH_ENCODINGS = ("16UC1", "32FC1")


def image_row(msg):
    return {"frame_id": msg.header.frame_id, "width": int(msg.width), "height": int(msg.height),
            "encoding": msg.encoding}


def info_row(msg):
    return {"frame_id": msg.header.frame_id, "width": int(msg.width), "height": int(msg.height),
            "distortion_model": msg.distortion_model, "d": [float(x) for x in msg.d], "k": [float(x) for x in msg.k]}


def judge(color, depth, info, frame):
    """the runner's own acceptance rules on one message of each -> list of failure reasons (empty = ok)."""
    bad = []
    for name, row in (("color", color), ("depth", depth), ("info", info)):
        if row["frame_id"] != frame:
            bad.append("%s frame_id %r != --camera-frame %r" % (name, row["frame_id"], frame))
    if (color["width"], color["height"]) != (info["width"], info["height"]):
        bad.append("color %dx%d != CameraInfo %dx%d" % (color["width"], color["height"], info["width"], info["height"]))
    if (depth["width"], depth["height"]) != (color["width"], color["height"]):
        bad.append("depth %dx%d != color %dx%d (not aligned)" % (depth["width"], depth["height"], color["width"],
                                                                 color["height"]))
    if any(abs(v) > 1e-8 for v in info["d"]):
        bad.append("CameraInfo has distortion %s (the runner wants rectified intrinsics: use the depth CameraInfo)"
                   % info["d"])
    if color["encoding"] not in RGB_ENCODINGS:
        bad.append("color encoding %s not in %s" % (color["encoding"], RGB_ENCODINGS))
    if depth["encoding"] not in DEPTH_ENCODINGS:
        bad.append("depth encoding %s not in %s" % (depth["encoding"], DEPTH_ENCODINGS))
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--color-topic", required=True)
    ap.add_argument("--depth-topic", required=True)
    ap.add_argument("--info-topic", required=True)
    ap.add_argument("--record-info-topic", action="append", default=[])
    ap.add_argument("--frame", required=True, help="the --camera-frame given to the producer and the runner")
    ap.add_argument("--timeout-s", type=float, default=20.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    t0 = time.monotonic()
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
    except ImportError as exc:
        print(json.dumps({"error": "import: %s" % exc}))
        return 2
    got = {}

    def keep(key, fn):
        def cb(msg):
            if key not in got:
                got[key] = fn(msg)
        return cb

    rclpy.init(args=None)
    node = rclpy.create_node("fpga_camera_check")
    node.create_subscription(Image, a.color_topic, keep("color", image_row), qos_profile_sensor_data)
    node.create_subscription(Image, a.depth_topic, keep("depth", image_row), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, a.info_topic, keep("info", info_row), qos_profile_sensor_data)
    for t in a.record_info_topic:
        if t != a.info_topic:
            node.create_subscription(CameraInfo, t, keep("record:" + t, info_row), qos_profile_sensor_data)
    need = ("color", "depth", "info")
    try:
        while time.monotonic() - t0 < a.timeout_s and not all(k in got for k in need):
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(10):   # a moment more for the record-only topics
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    missing = [k for k in need if k not in got]
    row = {"frame": a.frame, "topics": {"color": a.color_topic, "depth": a.depth_topic, "info": a.info_topic},
           "seconds": round(time.monotonic() - t0, 3), "got": got, "missing": missing}
    if missing:
        row["ok"], row["reasons"] = False, ["no message on %s in %.0f s" % (k, a.timeout_s) for k in missing]
        rc = 3
    else:
        row["reasons"] = judge(got["color"], got["depth"], got["info"], a.frame)
        row["ok"] = not row["reasons"]
        rc = 0 if row["ok"] else 1
    text = json.dumps(row, sort_keys=True)
    print(text, flush=True)
    if a.out:
        with open(a.out, "w") as f:
            f.write(text + "\n")
    return rc


if __name__ == "__main__":
    sys.exit(main())
