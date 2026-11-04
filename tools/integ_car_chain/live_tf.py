#!/usr/bin/env python3
"""TF stand-in for a real camera on a car that is not driving (v8, 2026-10-12; bench run only).

The geometry producer needs odom -> base_link -> <camera optical frame> at every depth stamp. With
the Orbbec driver on the Nano and no base, nobody publishes them. This node
  * waits for the first CameraInfo on --info-topic and takes its header.frame_id as the camera
    frame (written to --frame-out; the rig gives it to the producer as --camera-frame),
  * /tf_static: map -> odom (identity), base_link -> <camera frame> (tools.car_geo_producer.common
    .base_camera: the recommended mount, the same transform the replay camera uses),
  * /tf: odom -> base_link identity at --hz (default 50), stamped with the node's ROS time now.
Start the driver with publish_tf:=false: its own tree would give the optical frame a second parent.
The car is at rest: this is a transport / timing rig, not a mapping test.
--info-out (2026-10-13): the first CameraInfo of --info-topic and of every --extra-info-topic as JSON (frame,
size, distortion model, K, D, P): which intrinsics the producer was given, next to the colour camera's own.
"""
import argparse
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.car_geo_producer.common import base_camera  # noqa: E402
from tools.car_geo_producer.replay_camera import tf_msg  # noqa: E402


def info_row(msg):
    """sensor_msgs/CameraInfo -> plain dict (no ROS import; unit-tested with a stand-in)."""
    return {"frame_id": msg.header.frame_id, "width": int(msg.width), "height": int(msg.height),
            "distortion_model": msg.distortion_model, "k": [float(x) for x in msg.k], "d": [float(x) for x in msg.d],
            "p": [float(x) for x in msg.p]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--info-topic", required=True)
    ap.add_argument("--extra-info-topic", action="append", default=[], help="recorded in --info-out only")
    ap.add_argument("--info-out", default=None, help="JSON: first CameraInfo per topic (rewritten as they arrive)")
    ap.add_argument("--frame-out", required=True, help="new file: the camera frame id, written once known")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--duration-s", type=float, default=300.0)
    a, ros_args = ap.parse_known_args(argv)
    import json
    import rclpy
    import rclpy.executors
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init(args=ros_args)
    node = Node("fpga_live_tf")
    tfb, stb = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    state = {"frame": None, "sent": 0}
    eye = np.eye(4)
    infos = {}

    def record(topic):
        def on(msg):
            if topic not in infos:
                infos[topic] = info_row(msg)
                if a.info_out:
                    tmp = a.info_out + ".tmp"
                    with open(tmp, "w") as f:
                        json.dump(infos, f, indent=1, sort_keys=True)
                    os.replace(tmp, a.info_out)
        return on

    def on_info(msg):
        record(a.info_topic)(msg)
        if state["frame"] is None and msg.header.frame_id:
            state["frame"] = msg.header.frame_id
            now = node.get_clock().now().to_msg()
            stb.sendTransform([tf_msg(TransformStamped, "map", "odom", eye, now),
                               tf_msg(TransformStamped, "base_link", state["frame"], base_camera(), now)])
            with open(a.frame_out, "x") as f:
                f.write(state["frame"] + "\n")
            print("camera frame %s (%dx%d)" % (state["frame"], msg.width, msg.height), flush=True)

    def tick():
        tfb.sendTransform(tf_msg(TransformStamped, "odom", "base_link", eye, node.get_clock().now().to_msg()))
        state["sent"] += 1

    node.create_subscription(CameraInfo, a.info_topic, on_info, qos_profile_sensor_data)
    for t in a.extra_info_topic:
        if t != a.info_topic:
            node.create_subscription(CameraInfo, t, record(t), qos_profile_sensor_data)
    node.create_timer(1.0 / a.hz, tick)
    end = time.monotonic() + a.duration_s
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print("tf sent %d, camera frame %s" % (state["sent"], state["frame"]), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
