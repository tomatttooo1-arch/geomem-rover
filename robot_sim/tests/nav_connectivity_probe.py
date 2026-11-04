#!/usr/bin/env python3
"""Stage A passive inputs/TF/action check. Never sends goals, commands or permits.

This connection check does not replace motion, perception or fault acceptance.
Records header advancement and wall receipt age separately from TF availability.
"""
import argparse
import json
import time
from pathlib import Path
from host.cloud_geometry import check_cloud_depth

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--require-action', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('dev2_nav_connectivity_probe')
    tf = Buffer()
    listener = TransformListener(tf, node)
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    counts, stamps, changed, receipt, frames = {}, {}, {}, {}, {}
    started = time.monotonic()
    output = args.out.open('x')

    def emit(event, **data):
        output.write(json.dumps(dict(event=event, elapsed_s=time.monotonic()-started, **data), allow_nan=False)+'\n')
        output.flush()

    def receive(topic, msg):
        now = time.monotonic()
        stamp = msg.clock if topic == '/clock' else msg.header.stamp
        value = stamp.sec*1000000000+stamp.nanosec
        if topic in aligned:
            aligned[topic][value] = msg
            while len(aligned[topic]) > 10:
                del aligned[topic][next(iter(aligned[topic]))]
        counts[topic] = counts.get(topic, 0)+1
        if value > stamps.get(topic, -1):
            changed[topic] = changed.get(topic, 0)+1
        stamps[topic], receipt[topic] = value, now
        frames[topic] = None if topic == '/clock' else msg.header.frame_id
        if counts[topic] <= 3 or counts[topic] % 20 == 0:
            emit('input', topic=topic, count=counts[topic], stamp_ns=value, frame=frames[topic])

    kinds = {'/clock': Clock, '/odom': Odometry, '/fpga/points': PointCloud2,
             '/fpga/rgb': Image, '/fpga/depth': Image, '/fpga/camera_info': CameraInfo}
    subscriptions = [node.create_subscription(kind, topic, lambda msg, t=topic: receive(t, msg),
                     qos_profile_sensor_data) for topic, kind in kinds.items()]
    geometry = None
    aligned = {t: {} for t in ("/fpga/points", "/fpga/depth", "/fpga/camera_info")}
    passed = False
    # A bounded observation window, not a retry of failed lifecycle startup.
    deadline = started+30
    healthy_since = None
    emit('begin', duration_s=30, consecutive_healthy_s=3, max_receipt_age_s=1,
         require_action=args.require_action, publishes_motion=False)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            transforms = {}
            for child in ('base_link', 'camera_optical_frame'):
                try:
                    value = tf.lookup_transform('map', child, Time())
                    transforms[child] = dict(stamp_ns=value.header.stamp.sec*1000000000+value.header.stamp.nanosec,
                        x=value.transform.translation.x, y=value.transform.translation.y)
                except Exception:
                    pass
            if geometry is None:
                common = set.intersection(*(set(bucket) for bucket in aligned.values()))
                if common:
                    stamp = max(common)
                    geometry = check_cloud_depth(*(aligned[t][stamp] for t in aligned))
                    emit("cloud_depth_geometry", **geometry)
                    if not geometry["passed"]:
                        break
            healthy = (geometry is not None and geometry["passed"] and all(changed.get(t, 0) >= 3 and now-receipt.get(t, 0) <= 1 for t in kinds)
                       and len(transforms) == 2 and (not args.require_action or action.server_is_ready()))
            healthy_since = (healthy_since or now) if healthy else None
            if healthy_since and now-healthy_since >= 3:
                passed = True
                break
        emit('summary', passed=passed, counts=counts, advancing_samples=changed, stamps_ns=stamps,
             receipt_age_s={t: time.monotonic()-v for t, v in receipt.items()}, frames=frames,
             cloud_depth_geometry=geometry, transforms=transforms, navigate_to_pose_available=action.server_is_ready())
    finally:
        output.close()
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
