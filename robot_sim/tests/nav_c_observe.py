#!/usr/bin/env python3
"""Passive, time-associated Nav C sensor, TF, path and costmap evidence."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import PointCloud2
from host.pointcloud_sample import sample_xyz
from tf2_ros import Buffer, TransformListener


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir()
    stream = (args.out / 'observations.jsonl').open('x')
    rclpy.init()
    node = rclpy.create_node('dev2_c_observer')
    tf_buffer = Buffer(cache_time=Duration(seconds=10))
    tf_listener = TransformListener(tf_buffer, node)
    state = {'clock': None, 'clock_received_monotonic': None,
             'last_clock_log': 0.0, 'last_points': 0.0, 'point_seq': 0}

    def emit(topic, **fields):
        stream.write(json.dumps({
            'topic': topic, 'received_monotonic': time.monotonic(),
            'received_wall_ns': time.time_ns(), 'sim_clock_s': state['clock'],
            'clock_received_monotonic': state['clock_received_monotonic'], **fields,
        }, allow_nan=False) + '\n')
        stream.flush()

    def transform_at(target, source, stamp):
        if target == source:
            identity = TransformStamped()
            identity.header.frame_id = target
            identity.child_frame_id = source
            identity.header.stamp = stamp
            identity.transform.rotation.w = 1.0
            return {'status': 'identity', 'message': message_to_ordereddict(identity)}
        try:
            value = tf_buffer.lookup_transform(target, source, Time.from_msg(stamp),
                                               timeout=Duration(seconds=0))
            return {'status': 'ok', 'message': message_to_ordereddict(value)}
        except Exception as exc:
            return {'status': 'missing', 'error': type(exc).__name__ + ': ' + str(exc)}

    def clock(message):
        now = time.monotonic()
        state['clock'] = stamp_s(message.clock)
        state['clock_received_monotonic'] = now
        if now - state['last_clock_log'] >= .1:
            state['last_clock_log'] = now
            emit('/clock', message=message_to_ordereddict(message))

    def grid(topic, message):
        received = time.monotonic()
        frame = message.header.frame_id
        emit(topic, kind='full', message_received_monotonic=received,
             header=message_to_ordereddict(message.header),
             transform_map_to_frame=transform_at(frame, 'map', message.header.stamp),
             message=message_to_ordereddict(message))

    def update(topic, message):
        received = time.monotonic()
        frame = message.header.frame_id
        emit(topic, kind='update', message_received_monotonic=received,
             header=message_to_ordereddict(message.header),
             transform_map_to_frame=transform_at(frame, 'map', message.header.stamp),
             message=message_to_ordereddict(message))

    def points(message):
        now = time.monotonic()
        if now - state['last_points'] < 1:
            return
        state['last_points'] = now
        state['point_seq'] += 1
        name = f"points-{state['point_seq']:04}.cdr"
        (args.out / name).write_bytes(serialize_message(message))
        # A bounded, uniform sample lets the host prove free rays without ROS CDR
        # libraries. It contains sensor data only; obstacle truth is never supplied.
        xyz = sample_xyz(message, stride=16, limit=20000)
        emit('/fpga/points', file=name, header=message_to_ordereddict(message.header),
             message_received_monotonic=now, stamp_s=stamp_s(message.header.stamp),
             width=message.width, height=message.height,
             finite_xyz_stride=16, finite_xyz=xyz,
             transform_map_from_cloud=transform_at('map', message.header.frame_id,
                                                   message.header.stamp))

    def path(message):
        emit('/plan', header=message_to_ordereddict(message.header),
             poses=[[pose.pose.position.x, pose.pose.position.y] for pose in message.poses])

    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    for topic in ('/local_costmap/costmap', '/global_costmap/costmap'):
        qos = QoSProfile(depth=2, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, topic, lambda msg, t=topic: grid(t, msg), qos)
        node.create_subscription(OccupancyGridUpdate, topic + '_updates',
                                 lambda msg, t=topic: update(t + '_updates', msg), 10)
    node.create_subscription(PointCloud2, '/fpga/points', points, qos_profile_sensor_data)
    node.create_subscription(NavPath, '/plan', path, 10)
    try:
        rclpy.spin(node)
    finally:
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
