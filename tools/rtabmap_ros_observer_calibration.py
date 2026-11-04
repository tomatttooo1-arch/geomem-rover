#!/usr/bin/env python3
"""Inject known ROS observations to calibrate replay logging, not SLAM accuracy.

Runs only synthetic publishers/subscribers and the replay tool, on domain 83.
Tests valid pose, lost/high covariance, wrong feedback stamp, and depth-only
timestamp mismatch. Every generated output is new; no algorithms or devices.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        parser.error('set ROS_DOMAIN_ID=83 for isolated calibration')
    import cv2
    import numpy as np
    import rclpy
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    from rtabmap_msgs.msg import OdomInfo, Info
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock

    args.output.mkdir(parents=True, exist_ok=False)
    fixture = args.output / 'fixture'
    fixture.mkdir()
    for folder in ('rgb_sync', 'depth_sync'):
        (fixture / folder).mkdir()
    pairs = []
    for index in range(3):
        timestamp = str(100 + index) + '.000000000'
        row = {'timestamp': timestamp, 'depth_timestamp': timestamp}
        for folder, data in [('rgb_sync', np.full((480, 640, 3), 100 + index, np.uint8)),
                             ('depth_sync', np.full((480, 640), 5000, np.uint16))]:
            path = fixture / folder / (timestamp + '.png')
            assert cv2.imwrite(str(path), data)
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            row[folder] = {'source_sha256': digest, 'input_sha256': digest, 'source': str(path)}
        pairs.append(row)
    (fixture / 'INPUT.json').write_text(json.dumps({'pairs': pairs}))

    rclpy.init(args=[])
    node = rclpy.create_node('known_observation_injector')
    pubs = {key: node.create_publisher(kind, '/probe/' + key, 10) for key, kind in
            [('odom', Odometry), ('odom_info', OdomInfo), ('info', Info),
             ('localization_pose', PoseWithCovarianceStamped)]}
    tf_pub = node.create_publisher(TFMessage, '/tf', 10)
    received = {'rgb': {}, 'depth': {}, 'camera_info': {}}
    emitted = set()
    input_events = []
    clock_stamps = []
    def on_input(kind, message):
        ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        input_events.append({'kind': kind, 'stamp_ns': ns})
        received[kind][ns] = message
        if kind == 'depth':
            assert message.encoding == '32FC1'
            assert np.all(np.frombuffer(message.data, dtype='<f4') == 1.)
        if kind == 'rgb':
            assert message.encoding == 'bgr8'
        if ns in emitted or not all(ns in values for values in received.values()):
            return
        emitted.add(ns)
        index = ns // 1_000_000_000 - 100
        # Only frame 2 receives feedback at a deliberately incorrect stamp.
        feedback_ns = ns + (123 if index == 2 else 0)
        def header(message):
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(feedback_ns, 1_000_000_000)
            message.header.frame_id = 'map'
            return message
        odom = header(Odometry())
        odom.child_frame_id = 'camera_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = 1., 2., 3.
        odom.pose.pose.orientation.w = 1.
        cov = [0.] * 36
        for diagonal in range(6):
            cov[diagonal * 7] = 9999. if index == 1 else .25
        odom.pose.covariance = cov
        pubs['odom'].publish(odom)
        oi = header(OdomInfo())
        oi.lost, oi.inliers, oi.features = index == 1, 0 if index == 1 else 42, 80
        oi.covariance = cov
        pubs['odom_info'].publish(oi)
        if index != 1:
            info = header(Info())
            info.ref_id, info.loop_closure_id, info.proximity_detection_id = 7, 3, 4
            pubs['info'].publish(info)
            pose = header(PoseWithCovarianceStamped())
            pose.pose = odom.pose
            pubs['localization_pose'].publish(pose)
            transform = header(TransformStamped())
            transform.child_frame_id = 'odom'
            transform.transform.translation.x = 1.
            transform.transform.rotation.w = 1.
            tf_pub.publish(TFMessage(transforms=[transform]))
    subscriptions = [node.create_subscription(kind, '/probe/' + name,
        lambda message, name=name: on_input(name, message), 10)
        for name, kind in [('rgb', Image), ('depth', Image), ('camera_info', CameraInfo)]]
    subscriptions.append(node.create_subscription(Clock, '/clock',
        lambda message: clock_stamps.append(message.clock.sec * 1_000_000_000 + message.clock.nanosec), 10))
    processes = []
    try:
        summary = {}
        for name, options in [('known', ['--publish-clock']), ('depth_offset', ['--stop', '1', '--depth-stamp-offset-s', '.1'])]:
            for values in received.values():
                values.clear()
            emitted.clear()
            output = args.output / (name + '.jsonl')
            command = [sys.executable, str(Path(__file__).with_name('rtabmap_ros_replay.py')),
                str(fixture), str(output), '--rate', '5', '--feedback-timeout', '1',
                '--slam-timeout', '1', '--startup-timeout', '10', '--drain-seconds', '1', *options]
            with (args.output / (name + '.log')).open('x') as child_log:
                process = subprocess.Popen(command, stdout=child_log, stderr=subprocess.STDOUT)
                processes.append(process)
                deadline = time.monotonic() + 30
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
                    raise AssertionError('calibration child exceeded bounded wait')
            rows = [json.loads(line) for line in output.read_text().splitlines()]
            assert process.returncode == 2, (name, process.returncode)
            finish = [row for row in rows if row['event'] == 'complete'][-1]
            if name == 'known':
                assert clock_stamps == [100000000000, 101000000000, 102000000000], clock_stamps
                assert finish['feedback_timeout_indices'] == [2]
                assert finish['missing_feedback_after_drain'] == [2]
                assert finish['slam_feedback_timeout_indices'] == []
                odoms = [row for row in rows if row['event'] == 'odom']
                assert len(odoms) == 3
                assert odoms[0]['pose']['pose']['position'] == {'x': 1., 'y': 2., 'z': 3.}
                assert odoms[0]['pose']['covariance'][0] == .25
                info = [row for row in rows if row['event'] == 'odom_info']
                assert len(info) == 3 and info[0]['lost'] is False and info[0]['inliers'] == 42
                assert info[1]['lost'] is True and info[1]['covariance'][0] == 9999.
                assert info[2]['stamp_ns'] == 102000000123
                assert any(row['event'] == 'info' and row['loop_closure_id'] == 3
                           and row['proximity_detection_id'] == 4 for row in rows)
                assert any(row['event'] == 'localization_pose' and row['pose']['pose']['position']['x'] == 1.
                           for row in rows)
                assert any(row['event'] == 'tf' and row['transforms'][0]['child_frame_id'] == 'odom'
                           for row in rows)
            else:
                assert finish['feedback_timeout_indices'] == [0]
                assert not any(row['event'] == 'odom_info' for row in rows)
                publication = next(row for row in rows if row['event'] == 'publish_begin')
                assert publication['published_depth_stamp_ns'] - publication['published_rgb_stamp_ns'] == 100000000
            summary[name] = {'checks_passed': True, 'child_exit_code': process.returncode,
                             'complete': finish}
        (args.output / 'input_messages.json').write_text(json.dumps(input_events, indent=2))
        (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
