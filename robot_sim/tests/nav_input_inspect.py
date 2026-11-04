#!/usr/bin/env python3
"""Bounded, motionless inspection of externally supplied Nav2 pose inputs."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

INPUT_TOPICS = ('/odom', '/tf', '/tf_static', '/fpga/localization_status')
COMMAND_TOPICS = ('/cmd_vel', '/fpga/motion_permit')
LIFECYCLE_NODES = ('map_server', 'controller_server', 'smoother_server', 'planner_server',
                   'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother')
REQUIRED_DYNAMIC = {('map', 'odom'), ('odom', 'base_link')}
REQUIRED_STATIC = {('base_link', 'camera_link'), ('camera_link', 'camera_optical_frame')}
TRACKED_FRAMES = ('camera_link', 'base_link')


def status_contract(row, map_id, tracked_frame='camera_link'):
    if tracked_frame not in TRACKED_FRAMES:
        raise ValueError('unsupported expected tracked frame')
    return bool(isinstance(row, dict) and row.get('schema') == 1
                and row.get('map_id') == map_id and row.get('pose_source') == 'estimated'
                and row.get('world_frame') == 'map'
                and row.get('tracked_frame') == tracked_frame)


def evaluate_observation(map_id, status_rows, odom_rows, transforms, publishers,
                         command_publishers, lifecycle, status_fault_after_valid=False,
                         odom_fault_seen=False, diagnostic_transforms=None,
                         observed_input_sources=None, tracked_frame='camera_link'):
    """Evaluate captured facts without ROS so failure/recovery matrices are testable."""
    if tracked_frame not in TRACKED_FRAMES:
        raise ValueError('unsupported expected tracked frame')
    latest = status_rows[-1] if status_rows else None
    first_valid = next((index for index, row in enumerate(status_rows)
                        if status_contract(row, map_id, tracked_frame)
                        and row.get('state') == 'valid'), None)
    status_stable = bool(not status_fault_after_valid and first_valid is not None and all(
        status_contract(row, map_id, tracked_frame) and row.get('state') == 'valid'
        for row in status_rows[first_valid:]))
    latest_status_valid = bool(status_contract(latest, map_id, tracked_frame)
                               and latest.get('state') == 'valid')
    odom_valid = bool(not odom_fault_seen and odom_rows and all(row.get('frame_id') == 'odom'
                                       and row.get('child_frame_id') == 'base_link'
                                       for row in odom_rows))
    dynamic = {(row.get('parent'), row.get('child')) for row in transforms.get('/tf', [])}
    static = {(row.get('parent'), row.get('child')) for row in transforms.get('/tf_static', [])}
    all_rows = transforms.get('/tf', []) + transforms.get('/tf_static', [])
    edge_sources, child_parents = {}, {}
    for row in all_rows:
        edge = (row.get('parent'), row.get('child'))
        edge_sources.setdefault(edge, set()).add(row.get('source'))
        child_parents.setdefault(row.get('child'), set()).add(row.get('parent'))
    multi_source_edges = sorted(edge for edge, sources in edge_sources.items() if len(sources) != 1)
    multi_parent_children = sorted(child for child, parents in child_parents.items() if len(parents) > 1)
    wrong_channel_edges = sorted((REQUIRED_STATIC & dynamic) | (REQUIRED_DYNAMIC & static))
    missing_edges = sorted((REQUIRED_DYNAMIC - dynamic) | (REQUIRED_STATIC - static))
    diagnostic_rows = ((diagnostic_transforms or transforms).get('/tf', [])
                       + (diagnostic_transforms or transforms).get('/tf_static', []))
    stamps = Counter((row.get('parent'), row.get('child'), row.get('stamp_sec'),
                      row.get('stamp_nanosec'), row.get('source')) for row in diagnostic_rows)
    duplicate_edge_stamps = [
        {'parent': value[0], 'child': value[1], 'stamp': [value[2], value[3]],
         'source': value[4], 'count': count}
        for value, count in stamps.items() if count > 1]
    missing_inputs = [topic for topic in INPUT_TOPICS if not publishers.get(topic)]
    unique_authorities = (len({row.get('source') for row in odom_rows}) == 1
                          and len({row.get('source') for row in status_rows}) == 1
                          and len(publishers.get('/odom', [])) == 1
                          and len(publishers.get('/fpga/localization_status', [])) == 1
                          and not multi_source_edges)
    if observed_input_sources is not None:
        unique_authorities = unique_authorities and all(
            len(observed_input_sources.get(topic, ())) == 1
            for topic in ('/odom', '/fpga/localization_status'))
    lifecycle_bad = {name: lifecycle.get(name, 'service_missing') for name in LIFECYCLE_NODES
                     if lifecycle.get(name) not in ('unconfigured', 'inactive')}
    checks = {'all_input_publishers_present': not missing_inputs,
              'latest_status_valid_for_map': latest_status_valid,
              'status_stable_after_first_valid': status_stable,
              'all_observed_odom_frames_valid': odom_valid,
              'required_tf_edges_on_correct_channels': not missing_edges and not wrong_channel_edges,
              'single_authority_per_input': unique_authorities,
              'single_parent_per_tf_child': not multi_parent_children,
              'all_nav_lifecycle_nonactive': not lifecycle_bad,
              'no_command_publishers': not any(command_publishers.values())}
    return {'checks': checks, 'passed': all(checks.values()),
            'observed_input_sources': ({topic: sorted(sources) for topic, sources
                                        in observed_input_sources.items()}
                                       if observed_input_sources is not None else None),
            'navigation_ready_declared': bool(status_contract(latest, map_id, tracked_frame)
                                              and latest.get('navigation_ready') is True),
            'missing_inputs': missing_inputs, 'missing_tf_edges': missing_edges,
            'wrong_channel_tf_edges': wrong_channel_edges,
            'multi_source_tf_edges': multi_source_edges,
            'multi_parent_tf_children': multi_parent_children,
            'duplicate_tf_edge_stamps': duplicate_edge_stamps,
            'lifecycle_failures': lifecycle_bad}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=15.)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--tracked-frame', choices=TRACKED_FRAMES, default='camera_link',
                        help='expected localization tracked frame; never inferred from status')
    parser.add_argument('--map-yaml', type=Path, required=True)
    args, ros_args = parser.parse_known_args(argv)
    if not 1 <= args.duration <= 60:
        parser.error('--duration must be in 1..60 seconds')
    if not args.map_yaml.is_file():
        parser.error('--map-yaml must exist')

    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=ros_args)
    node = rclpy.create_node('fpga_nav_input_inspect')
    samples = {topic: [] for topic in INPUT_TOPICS}
    state = {'status_seen_valid': False, 'status_fault_after_valid': False,
             'odom_fault_seen': False}
    transform_facts = {'/tf': {}, '/tf_static': {}}
    # Two distinct sources suffice to latch a conflict for the entire run.
    # Sample tails may roll over; authority history must not roll over with them.
    observed_input_sources = {'/odom': set(), '/fpga/localization_status': set()}

    def source_id(info):
        return bytes(info.publisher_gid).hex()

    def append(topic, row, limit):
        samples[topic].append(row)
        del samples[topic][:-limit]

    def odom(message, info):
        row = {'stamp_sec': message.header.stamp.sec,
               'stamp_nanosec': message.header.stamp.nanosec,
               'frame_id': message.header.frame_id,
               'child_frame_id': message.child_frame_id,
               'source': source_id(info)}
        if len(observed_input_sources['/odom']) < 2:
            observed_input_sources['/odom'].add(row['source'])
        if row['frame_id'] != 'odom' or row['child_frame_id'] != 'base_link':
            state['odom_fault_seen'] = True
        append('/odom', row, 32)

    def transforms(topic):
        def receive(message, info):
            source = source_id(info)
            for transform in message.transforms:
                row = {'stamp_sec': transform.header.stamp.sec,
                       'stamp_nanosec': transform.header.stamp.nanosec,
                       'parent': transform.header.frame_id,
                       'child': transform.child_frame_id, 'source': source,
                       'translation': {'x': transform.transform.translation.x,
                                               'y': transform.transform.translation.y,
                                               'z': transform.transform.translation.z},
                       'rotation': {'x': transform.transform.rotation.x,
                                            'y': transform.transform.rotation.y,
                                            'z': transform.transform.rotation.z,
                                            'w': transform.transform.rotation.w}}
                transform_facts[topic][(row['parent'], row['child'], source)] = row
                append(topic, row, 64)
        return receive

    def status(message, info):
        if len(observed_input_sources['/fpga/localization_status']) < 2:
            observed_input_sources['/fpga/localization_status'].add(source_id(info))
        try:
            row = json.loads(message.data)
            if not isinstance(row, dict):
                raise ValueError('status must be an object')
            row = {key: row.get(key) for key in
                   ('schema', 'map_id', 'instance', 'epoch', 'sequence', 'state',
                    'pose_source', 'navigation_ready', 'world_frame', 'tracked_frame')}
            row['source'] = source_id(info)
        except (TypeError, ValueError):
            row = {'malformed': True, 'source': source_id(info)}
        current_valid = (status_contract(row, args.map_id, args.tracked_frame)
                         and row.get('state') == 'valid')
        if state['status_seen_valid'] and not current_valid:
            state['status_fault_after_valid'] = True
        state['status_seen_valid'] = state['status_seen_valid'] or current_valid
        append('/fpga/localization_status', row, 64)

    transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/tf', transforms('/tf'), qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/tf_static', transforms('/tf_static'), transient),
        node.create_subscription(String, '/fpga/localization_status', status, 10),
    ]
    clients = {name: node.create_client(GetState, '/' + name + '/get_state')
               for name in LIFECYCLE_NODES}
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
        lifecycle = {}
        for name, client in clients.items():
            if not client.service_is_ready():
                lifecycle[name] = 'service_missing'
                continue
            future = client.call_async(GetState.Request())
            wait = time.monotonic() + 1.
            while not future.done() and time.monotonic() < wait:
                rclpy.spin_once(node, timeout_sec=.05)
            lifecycle[name] = (future.result().current_state.label
                               if future.done() and future.exception() is None else 'query_failed')

        def endpoints(topic):
            return [{'node_name': info.node_name, 'node_namespace': info.node_namespace,
                     'topic_type': info.topic_type, 'source': bytes(info.endpoint_gid).hex()}
                    for info in node.get_publishers_info_by_topic(topic)]

        publishers = {topic: endpoints(topic) for topic in INPUT_TOPICS}
        command_publishers = {topic: endpoints(topic) for topic in COMMAND_TOPICS}
        observed_transforms = {topic: list(rows.values())
                               for topic, rows in transform_facts.items()}
        verdict = evaluate_observation(args.map_id, samples['/fpga/localization_status'],
                                       samples['/odom'], observed_transforms,
                                       publishers, command_publishers, lifecycle,
                                       state['status_fault_after_valid'], state['odom_fault_seen'],
                                       {'/tf': samples['/tf'], '/tf_static': samples['/tf_static']},
                                       observed_input_sources, tracked_frame=args.tracked_frame)
        result = {'probe': 'estimated_inspect', 'motionless_contract': True,
                  'map_id': args.map_id, 'map_yaml': str(args.map_yaml),
                  'expected_tracked_frame': args.tracked_frame,
                  'duration_s': args.duration, 'bounded_samples': samples,
                  'publishers': publishers, 'command_publishers': command_publishers,
                  'lifecycle': lifecycle, **verdict}
        args.out.write_text(json.dumps(result, indent=2) + '\n')
        return 0 if result['passed'] else 3
    finally:
        subscriptions.clear()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
