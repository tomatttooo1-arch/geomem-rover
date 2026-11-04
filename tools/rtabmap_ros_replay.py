#!/usr/bin/env python3
"""Replay paired TUM images into an isolated ROS 2 probe, observing every frame.

No ground truth, initial pose or velocity is published. Clock publication is
opt-in; each group must use fresh processes to avoid clock rollback. Original RGB
timestamps identify messages; --rate is a wall-clock upper playback rate, not a
performance measurement. Intervals use global zero-based [start, stop) indices.
Requires ROS Humble, rtabmap_msgs, NumPy and OpenCV in the execution environment.
"""
import argparse
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import time


def stamp_ns(value):
    return int(Decimal(str(value)) * 1_000_000_000)


def safe_json(value):
    """Preserve nonfinite evidence explicitly while emitting strict JSON."""
    if isinstance(value, dict):
        return {key: safe_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def playback_schedule(stamps, start, stop, duration=0., now=time.monotonic, cycles=0):
    """Preserve source timing within cycles; explicit one-second discontinuity gap."""
    begun = now()
    period = stamps[stop-1]-stamps[start]+1_000_000_000
    cycle = 0
    while True:
        for index in range(start, stop):
            if duration and now()-begun >= duration:
                return
            yield index, cycle, stamps[index]+cycle*period
        if (not duration and not cycles) or (cycles and cycle+1 >= cycles):
            return
        cycle += 1


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', type=Path, help='Prepared INPUT.json, or its containing directory')
    p.add_argument('output', type=Path, help='New JSONL file; existing output is rejected')
    p.add_argument('--start', type=int, default=0)
    p.add_argument('--stop', type=int, help='Exclusive stop; default is all paired frames')
    p.add_argument('--blank-start', type=int)
    p.add_argument('--blank-stop', type=int)
    p.add_argument('--rate', type=float, default=5.)
    p.add_argument('--depth-stamp-offset-s', type=float, default=0.,
                   help='Depth-only timestamp offset for exact-sync negative control')
    p.add_argument('--publish-clock', action='store_true',
                   help='Publish saved RGB capture time on /clock before each frame')
    p.add_argument('--feedback-timeout', type=float, default=10.)
    p.add_argument('--startup-timeout', type=float, default=30.)
    p.add_argument('--drain-seconds', type=float, default=3.)
    p.add_argument('--wait-slam', action=argparse.BooleanOptionalAction, default=True,
                   help='Wait for same-stamp Info when odometry is not lost')
    p.add_argument('--slam-timeout', type=float, default=5.)
    p.add_argument('--frame-id', default='camera_optical_frame')
    p.add_argument('--prefix', default='/probe')
    p.add_argument('--tf-topic', default='/tf')
    p.add_argument('--repeat-duration-seconds', type=float, default=0.,
                   help='repeat within this publisher process for bounded wall duration; requires clock and gate boundary acknowledgement')
    p.add_argument('--repeat-cycles', type=int, default=0, help='optional exact cycle limit within the existing wall budget')
    p.add_argument('--allocator-socket', type=Path, help='diagnostic allocator observer, separate from localization policy')
    p.add_argument('--memory-pause-after-cycle', type=int, default=7)
    return p


def run(args):
    # Import lazily so --help and input-independent inspection work without ROS.
    import cv2
    import numpy as np
    import rclpy
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rtabmap_msgs.msg import OdomInfo, Info
    from tf2_msgs.msg import TFMessage

    if not 0 <= args.repeat_duration_seconds <= 1800 or (args.repeat_duration_seconds and not args.publish_clock):
        raise ValueError('repeat duration must be 0..1800 seconds and needs published clock')
    if args.repeat_cycles < 0 or (args.repeat_cycles and not args.repeat_duration_seconds):
        raise ValueError('cycle limit requires a bounded repeat duration')
    if args.allocator_socket and not 0 < args.memory_pause_after_cycle < args.repeat_cycles:
        raise ValueError('allocator protocol requires a positive pause cycle before the final cycle')
    manifest_path = args.input / 'INPUT.json' if args.input.is_dir() else args.input
    manifest_bytes = manifest_path.read_bytes()
    manifest = json.loads(manifest_bytes)
    pairs = manifest['pairs']
    stop = len(pairs) if args.stop is None else args.stop
    if not 0 <= args.start < stop <= len(pairs):
        raise ValueError('require 0 <= start < stop <= paired frame count')
    if (args.blank_start is None) != (args.blank_stop is None):
        raise ValueError('blank-start and blank-stop must be specified together')
    if args.blank_start is not None and not 0 <= args.blank_start < args.blank_stop <= len(pairs):
        raise ValueError('invalid blank interval')
    for name in ('rate', 'feedback_timeout', 'startup_timeout', 'drain_seconds', 'slam_timeout'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            raise ValueError(f'{name} must be finite and positive')
    if not math.isfinite(args.depth_stamp_offset_s):
        raise ValueError('depth timestamp offset must be finite')
    depth_stamp_offset_ns = stamp_ns(args.depth_stamp_offset_s)
    stamps = [stamp_ns(row['timestamp']) for row in pairs]
    if any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('input timestamps must be strictly increasing')
    root = manifest_path.parent
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.open('x', buffering=1)
    def record(kind, **fields):
        log.write(json.dumps(safe_json({'event': kind, 'wall_time_ns': time.time_ns(),
            'monotonic_ns': time.monotonic_ns(), **fields}), allow_nan=False) + '\n')

    rclpy.init(args=[])
    node = rclpy.create_node('rtabmap_tum_replay_probe')
    prefix = args.prefix.rstrip('/')
    pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
    sub_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
    rgb_pub = node.create_publisher(Image, prefix + '/rgb', pub_qos)
    depth_pub = node.create_publisher(Image, prefix + '/depth', pub_qos)
    info_pub = node.create_publisher(CameraInfo, prefix + '/camera_info', pub_qos)
    boundary_pub = node.create_publisher(String, '/fpga/localization_discontinuity', pub_qos) if args.repeat_duration_seconds else None
    boundary_ack = set()
    def status_callback(message):
        row = json.loads(message.data)
        if row.get('reason', '').startswith('replay_discontinuity:'):
            boundary_ack.add(int(row['reason'].split(':')[1]))
    status_subscription = node.create_subscription(String, '/fpga/localization_status', status_callback, pub_qos) if boundary_pub else None
    clock_pub = None
    if args.publish_clock:
        from rosgraph_msgs.msg import Clock
        clock_pub = node.create_publisher(Clock, '/clock', pub_qos)
    feedback = set()
    odom_lost = {}
    slam_feedback = set()
    observed = {}
    fields_by_kind = {
        'odom': ('header', 'child_frame_id', 'pose', 'twist'),
        'odom_info': ('header', 'lost', 'matches', 'inliers', 'features', 'covariance',
            'key_frame_added', 'time_estimation', 'interval', 'distance_travelled',
            'local_key_frames', 'type', 'transform', 'transform_filtered', 'guess'),
        'info': ('header', 'ref_id', 'loop_closure_id', 'proximity_detection_id',
            'landmark_id', 'loop_closure_transform', 'stats_keys', 'stats_values'),
        'localization_pose': ('header', 'pose'),
        'tf': ('transforms',),
    }
    def callback(kind, message):
        fields = {}
        for name in fields_by_kind[kind]:
            if hasattr(message, name):
                value = getattr(message, name)
                if hasattr(value, 'get_fields_and_field_types'):
                    value = message_to_ordereddict(value)
                elif not isinstance(value, (bool, int, float, str)):
                    value = [message_to_ordereddict(item) if hasattr(item, 'get_fields_and_field_types')
                             else item for item in value]
                fields[name] = value
        if hasattr(message, 'header'):
            ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            fields['stamp_ns'] = ns
            if kind == 'odom_info':
                feedback.add(ns)
                odom_lost[ns] = bool(message.lost)
            elif kind == 'info':
                slam_feedback.add(ns)
        observed[kind] = observed.get(kind, 0) + 1
        record(kind, **fields)
    subscriptions = []
    for kind, message_type, topic in [('odom', Odometry, prefix + '/odom'),
            ('odom_info', OdomInfo, prefix + '/odom_info'), ('info', Info, prefix + '/info'),
            ('localization_pose', PoseWithCovarianceStamped, prefix + '/localization_pose'),
            ('tf', TFMessage, args.tf_topic)]:
        subscriptions.append(node.create_subscription(message_type, topic,
            lambda message, kind=kind: callback(kind, message), sub_qos))
    def spin_until(deadline, predicate=lambda: False):
        while rclpy.ok() and time.monotonic() < deadline and not predicate():
            rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped')
        return predicate()

    published, timeouts, slam_timeouts = 0, [], []
    dispatched = []
    last_cycle = -1
    try:
        record('start', input=str(manifest_path), input_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
            paired_frames=len(pairs), start=args.start, stop=stop, options={key: str(value)
            if isinstance(value, Path) else value for key, value in vars(args).items()},
            timestamp_policy=('cycle-restamped RGB; source stamps retained; /clock=True' if args.repeat_duration_seconds
                              else 'saved RGB stamps; /clock=' + str(args.publish_clock)), performance_measurement=False)
        def ready():
            return all(pub.get_subscription_count() > 0 for pub in (rgb_pub, depth_pub, info_pub)) \
                and node.count_publishers(prefix + '/odom_info') > 0 \
                and (not args.wait_slam or node.count_publishers(prefix + '/info') > 0)
        if not spin_until(time.monotonic() + args.startup_timeout, ready):
            record('startup_timeout', rgb_subscribers=rgb_pub.get_subscription_count(),
                depth_subscribers=depth_pub.get_subscription_count(),
                camera_info_subscribers=info_pub.get_subscription_count(),
                odom_info_publishers=node.count_publishers(prefix + '/odom_info'))
            return 2
        record('ready')
        replay_started = time.monotonic()
        for index, cycle, published_stamp in playback_schedule(stamps, args.start, stop, args.repeat_duration_seconds,
                                                               cycles=args.repeat_cycles):
            if cycle != last_cycle:
                if args.allocator_socket:
                    if __package__:
                        from .rtabmap_memory_probe import allocator_request
                    else:
                        from rtabmap_memory_probe import allocator_request
                    record('memory_checkpoint', cycle=cycle, phase='cycle_begin',
                           allocator=allocator_request(args.allocator_socket))
                    if cycle == args.memory_pause_after_cycle:
                        record('memory_idle_begin', cycle=cycle)
                        spin_until(time.monotonic()+30.)
                        record('memory_checkpoint', cycle=cycle, phase='idle_before_trim',
                               allocator=allocator_request(args.allocator_socket))
                        record('memory_checkpoint', cycle=cycle, phase='trim',
                               allocator=allocator_request(args.allocator_socket,'trim'))
                        spin_until(time.monotonic()+30.)
                        record('memory_checkpoint', cycle=cycle, phase='idle_after_trim',
                               allocator=allocator_request(args.allocator_socket))
                if cycle and boundary_pub is not None:
                    boundary_pub.publish(String(data=json.dumps({'cycle': cycle})))
                    if not spin_until(time.monotonic()+.5, lambda: cycle in boundary_ack):
                        raise RuntimeError('gate did not acknowledge replay discontinuity within 0.5s')
                    record('repeat_boundary', cycle=cycle, next_stamp_ns=published_stamp, source_stamp_ns=stamps[index], gate_acknowledged=True)
                last_cycle = cycle
            started = time.monotonic()
            row = pairs[index]
            files = {folder: root / folder / (row['timestamp'] + '.png')
                     for folder in ('rgb_sync', 'depth_sync')}
            hashes = {folder: hashlib.sha256(path.read_bytes()).hexdigest() for folder, path in files.items()}
            if any(hashes[folder] != row[folder]['input_sha256'] for folder in files):
                raise ValueError(f'input hash mismatch at global frame {index}')
            rgb = cv2.imread(str(files['rgb_sync']), cv2.IMREAD_COLOR)
            raw_depth = cv2.imread(str(files['depth_sync']), cv2.IMREAD_UNCHANGED)
            if rgb is None or raw_depth is None or raw_depth.dtype != np.uint16 \
                    or raw_depth.shape != (480, 640) or rgb.shape != (480, 640, 3):
                raise ValueError(f'invalid TUM image at global frame {index}')
            depth = np.ascontiguousarray(raw_depth.astype(np.float32) / 5000., dtype='<f4')
            blank = args.blank_start is not None and args.blank_start <= index < args.blank_stop
            original_valid_depth = int(np.count_nonzero(depth))
            if blank:
                rgb = np.zeros_like(rgb)
                depth = np.zeros_like(depth)
            def image_message(array, encoding, step):
                message = Image()
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(published_stamp, 1_000_000_000)
                message.header.frame_id = args.frame_id
                message.height, message.width = array.shape[:2]
                message.encoding, message.is_bigendian, message.step = encoding, 0, step
                message.data = array.tobytes()
                return message
            rgb_message = image_message(rgb, 'bgr8', 640 * 3)
            depth_message = image_message(depth, '32FC1', 640 * 4)
            depth_message.header.stamp.sec, depth_message.header.stamp.nanosec = divmod(
                published_stamp + depth_stamp_offset_ns, 1_000_000_000)
            camera = CameraInfo()
            camera.header = rgb_message.header
            camera.height, camera.width = 480, 640
            camera.distortion_model, camera.d = 'plumb_bob', [0.] * 5
            camera.k = [517.3, 0., 318.6, 0., 516.5, 255.3, 0., 0., 1.]
            camera.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            camera.p = [517.3, 0., 318.6, 0., 0., 516.5, 255.3, 0., 0., 0., 1., 0.]
            record('publish_begin', index=index, stamp_ns=published_stamp, timestamp=row['timestamp'],
                **({'cycle': cycle, 'source_stamp_ns': stamps[index]} if args.repeat_duration_seconds else {}),
                published_rgb_stamp_ns=published_stamp,
                published_depth_stamp_ns=published_stamp + depth_stamp_offset_ns,
                depth_timestamp=row['depth_timestamp'], blank=blank, input_sha256=hashes,
                source_rgb_sha256=row['rgb_sync']['source_sha256'],
                source_depth_sha256=row['depth_sync']['source_sha256'],
                published_rgb_sha256=hashlib.sha256(rgb_message.data).hexdigest(),
                published_depth_sha256=hashlib.sha256(depth_message.data).hexdigest(),
                valid_depth_pixels=int(np.count_nonzero(depth)), original_valid_depth_pixels=original_valid_depth)
            if clock_pub is not None:
                clock_message = Clock()
                clock_message.clock.sec, clock_message.clock.nanosec = divmod(published_stamp, 1_000_000_000)
                clock_pub.publish(clock_message)
                record('clock_published', stamp_ns=published_stamp, index=index)
            info_pub.publish(camera)
            rgb_pub.publish(rgb_message)
            depth_pub.publish(depth_message)
            published += 1
            dispatched.append((index, published_stamp))
            record('published', index=index, stamp_ns=published_stamp)
            received = spin_until(time.monotonic() + args.feedback_timeout,
                                  lambda: published_stamp in feedback)
            record('frame_feedback', index=index, stamp_ns=published_stamp, received=received)
            if not received:
                timeouts.append(index)
            if args.wait_slam and received and not odom_lost[published_stamp]:
                slam_received = spin_until(time.monotonic() + args.slam_timeout,
                    lambda: published_stamp in slam_feedback)
                record('frame_slam_feedback', index=index, stamp_ns=published_stamp, received=slam_received)
                if not slam_received:
                    slam_timeouts.append(index)
            elif args.wait_slam:
                record('frame_slam_skipped', index=index, stamp_ns=published_stamp,
                    reason='odom_lost' if received else 'odom_feedback_missing')
            spin_until(started + 1. / args.rate)
        spin_until(time.monotonic() + args.drain_seconds)
        if args.allocator_socket:
            if __package__:
                from .rtabmap_memory_probe import allocator_request
            else:
                from rtabmap_memory_probe import allocator_request
            record('memory_checkpoint', cycle=last_cycle, phase='after_final_drain',
                   allocator=allocator_request(args.allocator_socket))
        missing = [index for index, stamp in dispatched if stamp not in feedback]
        record('complete', published_frames=published, feedback_timeout_indices=timeouts,
            **({'repeat_wall_seconds': time.monotonic()-replay_started, 'cycles_started': last_cycle+1,
                'missing_stamp_ns': [stamp for _, stamp in dispatched if stamp not in feedback]} if args.repeat_duration_seconds else {}),
            missing_feedback_after_drain=missing, slam_feedback_timeout_indices=slam_timeouts,
            observed_messages=observed)
        return 2 if timeouts or missing or slam_timeouts else 0
    except BaseException as exc:
        record('error', exception=type(exc).__name__, message=str(exc), published_frames=published)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
        log.close()


def main():
    args = parser().parse_args()
    try:
        return run(args)
    except (ValueError, OSError, RuntimeError, ImportError) as exc:
        parser().exit(2, f'replay probe failed: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
