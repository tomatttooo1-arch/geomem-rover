#!/usr/bin/env python3
"""Record the Nav2 handoff interface from inside the container: TF, /odom, /clock, status.

Container only. This host has no ROS 2, so nothing here has ever run; the rclpy part is
syntax-checked and the decisions it delegates are unit tested through
`tools/nav2_interface_core.py`. Output is JSONL, one message per line, read afterwards by
`tools/nav2_interface_check.py` — the recorder itself never judges anything.

Two things make it more than a `ros2 bag`:

* It keeps its OWN `tf2_ros.Buffer`, fed by `set_transform`/`set_transform_static` from the
  same subscriptions it records. There is no `TransformListener`, so there is exactly one
  subscription per topic and the recorded stream and the buffer state cannot disagree.
* On every /clock message it appends a `sample` row with the rehearsal criteria: C1 = the
  age of `lookup_transform(world, base, Time())` relative to that /clock value (or the
  exception name), and can_transform asked at TWO times — `Time(clock)` (`c2_at_clock`,
  evidence only: every `odom->base_link` stamp is a frame stamp and therefore behind the
  clock, so this query is expected to be false throughout and discriminates nothing) and
  `Time(newest odom->base_link stamp)` (`c2_at_odom`, the criterion). These are what turn
  "tf2 will silently file our dynamic edge in the static cache" from an argument about
  tf2 source into a number.

/clock is subscribed with the same QoS `tools/rtabmap_ros_replay.py` publishes it on
(depth 10, RELIABLE, VOLATILE) — not sensor-data — or no /clock would be received at all.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import time

if __package__:
    from .nav2_interface_core import (BASE_FRAME, CLOCK_TOPIC,
                                      DEFAULT_STATUS_TOPICS, MOTION_PERMIT_TOPIC, NewestEdgeStamp,
                                      ODOM_FRAME, ODOM_TOPIC, RepeatedStampCounter, SAMPLE,
                                      TF_STATIC_TOPIC, TF_TOPIC,
                                      WORLD_FRAME, clock_record, odom_record, sample_record,
                                      status_record, summary_record, tf_record, topic_info_record,
                                      transform_row)
else:  # executed as /probe_tools/tf_topic_recorder.py inside the container
    from nav2_interface_core import (BASE_FRAME, CLOCK_TOPIC,
                                     DEFAULT_STATUS_TOPICS, MOTION_PERMIT_TOPIC, NewestEdgeStamp,
                                     ODOM_FRAME, ODOM_TOPIC, RepeatedStampCounter, SAMPLE,
                                     TF_STATIC_TOPIC, TF_TOPIC,
                                     WORLD_FRAME, clock_record, odom_record, sample_record,
                                     status_record, summary_record, tf_record, topic_info_record,
                                     transform_row)

AUTHORITY = 'tf_topic_recorder'
TOPIC_INFO_SUFFIX = '.topic-info.txt'


def parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True,
                   help='new JSONL file; an existing path is rejected, never appended to')
    p.add_argument('--world-frame', default=WORLD_FRAME)
    p.add_argument('--base-frame', default=BASE_FRAME)
    p.add_argument('--odom-frame', default=ODOM_FRAME,
                   help='parent of the odometry edge whose newest stamp c2_at_odom is asked at')
    p.add_argument('--tf-topic', default=TF_TOPIC)
    p.add_argument('--tf-static-topic', default=TF_STATIC_TOPIC)
    p.add_argument('--odom-topic', default=ODOM_TOPIC)
    p.add_argument('--clock-topic', default=CLOCK_TOPIC)
    p.add_argument('--status-topics', nargs='*', default=list(DEFAULT_STATUS_TOPICS),
                   help='std_msgs/String JSON status topics to record')
    p.add_argument('--extra-info-topics', nargs='*', default=[MOTION_PERMIT_TOPIC],
                   help='topics snapshotted with "ros2 topic info" but not subscribed')
    p.add_argument('--topic-info', action=argparse.BooleanOptionalAction, default=True,
                   help='snapshot publisher/subscriber counts at startup')
    p.add_argument('--topic-info-timeout', type=float, default=20.)
    p.add_argument('--cache-time-s', type=float, default=10.,
                   help='tf2 buffer cache; the tf2 default, kept explicit because C1 depends on it')
    return p


def topic_info_path(output):
    return Path(str(output) + TOPIC_INFO_SUFFIX)


def topic_info_snapshot(topics, path, timeout):
    """`ros2 topic info --verbose --no-daemon` per topic into one text file.

    A failure here is evidence, not a reason to lose the recording: every error is written
    into the snapshot file and returned, and the caller keeps going.
    """
    results, blocks = [], []
    for topic in topics:
        block = ['=== topic: %s ===' % topic]
        try:
            done = subprocess.run(['ros2', 'topic', 'info', '--verbose', '--no-daemon', topic],
                                  capture_output=True, text=True, timeout=timeout)
            block.append(done.stdout)
            if done.stderr:
                block.append('--- stderr ---\n' + done.stderr)
            block.append('returncode: %d' % done.returncode)
            results.append({'topic': topic, 'returncode': done.returncode, 'error': None})
        except (OSError, subprocess.SubprocessError) as exc:
            message = '%s: %s' % (type(exc).__name__, exc)
            block.append('error: ' + message)
            results.append({'topic': topic, 'returncode': None, 'error': message})
        blocks.append('\n'.join(block).rstrip() + '\n')
    try:
        path.write_text('\n'.join(blocks))
    except OSError as exc:
        results.append({'topic': None, 'returncode': None,
                        'error': 'snapshot file not written: %s: %s' % (type(exc).__name__, exc)})
    return results


def run(args):
    # Imported lazily so --help and the host-side wiring tests work without ROS.
    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                           ReliabilityPolicy)
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    import tf2_ros

    subscribed = [args.tf_topic, args.tf_static_topic, args.odom_topic, args.clock_topic,
                  *args.status_topics]
    log = args.output.open('x', buffering=1)
    state = {'clock_ns': None, 'samples': 0, 'buffer_errors': 0}
    counts = Counter()
    counter = RepeatedStampCounter()
    odom_edge = NewestEdgeStamp(args.odom_frame, args.base_frame)

    def write(row):
        log.write(json.dumps(row, allow_nan=False) + '\n')

    def now():
        return time.monotonic_ns()

    if args.topic_info:
        results = topic_info_snapshot([*subscribed, *args.extra_info_topics],
                                      topic_info_path(args.output), args.topic_info_timeout)
        write(topic_info_record(now(), state['clock_ns'], results))

    rclpy.init(args=[])
    node = rclpy.create_node('fpga_tf_topic_recorder')
    buffer = tf2_ros.Buffer(cache_time=Duration(seconds=args.cache_time_s))

    # Matched to the publishers: /tf and /tf_static reliable depth 100 (and TRANSIENT_LOCAL
    # for the latched static edges), /odom sensor-data, /clock exactly as the replay
    # publishes it (depth 10, reliable, volatile), status depth 10 reliable.
    tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
    tf_static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)
    status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)

    def transforms(message, topic, static):
        received = now()
        rows = []
        for transform in message.transforms:
            stamp = transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec
            counter.observe(transform.header.frame_id, transform.child_frame_id, stamp)
            odom_edge.observe(transform.header.frame_id, transform.child_frame_id, stamp, static)
            translation, rotation = transform.transform.translation, transform.transform.rotation
            rows.append(transform_row(
                transform.header.frame_id, transform.child_frame_id, stamp, static,
                translation={'x': translation.x, 'y': translation.y, 'z': translation.z},
                rotation={'x': rotation.x, 'y': rotation.y, 'z': rotation.z, 'w': rotation.w}))
            try:
                if static:
                    buffer.set_transform_static(transform, AUTHORITY)
                else:
                    buffer.set_transform(transform, AUTHORITY)
            except Exception as exc:  # a rejected edge must not stop the recording
                state['buffer_errors'] += 1
                rows[-1]['buffer_error'] = type(exc).__name__
        counts[topic] += 1
        write(tf_record(topic, received, state['clock_ns'], rows))

    def odometry(message):
        received = now()
        counts[args.odom_topic] += 1
        write(odom_record(args.odom_topic, received, state['clock_ns'], message.header.frame_id,
                          message.child_frame_id,
                          message.header.stamp.sec * 10**9 + message.header.stamp.nanosec))

    def status(topic, message):
        received = now()
        counts[topic] += 1
        write(status_record(topic, received, state['clock_ns'], message.data))

    def clock(message):
        # One steady-clock read per callback, taken at entry and used for BOTH rows this
        # callback writes. Reading it again below would stamp the sample with the cost of
        # the tf2 lookups instead of the arrival it describes; see host/localization_gate.py
        # CallbackClock for the same rule on the gate side, where a second read moved a
        # recorded timestamp across a decision boundary.
        received = now()
        clock_ns = message.clock.sec * 10**9 + message.clock.nanosec
        state['clock_ns'] = clock_ns
        counts[args.clock_topic] += 1
        write(clock_record(received, clock_ns))
        try:
            found = buffer.lookup_transform(args.world_frame, args.base_frame, Time())
            lookup = found.header.stamp.sec * 10**9 + found.header.stamp.nanosec
        except Exception as exc:
            lookup = exc
        def can_at(when):
            try:
                return buffer.can_transform(args.world_frame, args.base_frame,
                                            Time(nanoseconds=when), timeout=Duration(seconds=0))
            except Exception as exc:
                return exc
        at_clock = can_at(clock_ns)
        at_odom = None if odom_edge.stamp_ns is None else can_at(odom_edge.stamp_ns)
        state['samples'] += 1
        counts[SAMPLE] += 1
        write(sample_record(received, clock_ns, lookup, at_clock, at_odom, odom_edge.stamp_ns,
                            args.world_frame, args.base_frame))

    node.create_subscription(TFMessage, args.tf_topic,
                             lambda m: transforms(m, args.tf_topic, False), tf_qos)
    node.create_subscription(TFMessage, args.tf_static_topic,
                             lambda m: transforms(m, args.tf_static_topic, True), tf_static_qos)
    node.create_subscription(Odometry, args.odom_topic, odometry, qos_profile_sensor_data)
    node.create_subscription(Clock, args.clock_topic, clock, clock_qos)
    for topic in args.status_topics:
        node.create_subscription(String, topic, lambda m, t=topic: status(t, m), status_qos)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if type(exc).__name__ != 'ExternalShutdownException':
            raise
    finally:
        write(summary_record(now(), state['clock_ns'], counts, counter.count, state['samples'],
                             counter.examples(), buffer_errors=state['buffer_errors'],
                             subscribed_topics=subscribed, world_frame=args.world_frame,
                             base_frame=args.base_frame, odom_frame=args.odom_frame,
                             newest_odom_base_stamp_ns=odom_edge.stamp_ns,
                             odom_base_transforms=odom_edge.count))
        log.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        return run(args)
    except (OSError, ValueError, ImportError) as exc:
        parser().exit(2, 'tf topic recorder failed: %s: %s\n' % (type(exc).__name__, exc))


if __name__ == '__main__':
    raise SystemExit(main())
