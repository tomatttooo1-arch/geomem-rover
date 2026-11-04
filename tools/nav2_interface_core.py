#!/usr/bin/env python3
"""Record schemas shared by the TF recorder and the static interface check.

Everything here is pure Python: no ROS, no filesystem, no clock. The recorder
(`tools/tf_topic_recorder.py`) can only run inside the container, so every piece of it
that can be decided without a message bus lives here and is unit tested on the host.
That split is the only reason the recorder's output format is testable at all.

One JSON object per line. Every line carries `topic`, `recv_monotonic_ns` (the recorder's
own steady clock, never the simulated one) and `clock_ns` (the last value seen on /clock,
null before the first one). Two pseudo-topics are used for rows the recorder derives
rather than receives: `sample` (one per /clock message, carrying the C1/C2 lookup probes)
and `summary` (one at shutdown). Neither is a valid ROS topic name, so they cannot
collide with a subscribed topic.
"""
from collections import Counter
import json

SCHEMA = 1

SAMPLE = 'sample'
SUMMARY = 'summary'
TOPIC_INFO = 'topic_info'

TF_TOPIC = '/tf'
TF_STATIC_TOPIC = '/tf_static'
ODOM_TOPIC = '/odom'
CLOCK_TOPIC = '/clock'
LOCALIZATION_STATUS_TOPIC = '/fpga/localization_status'
ADAPTER_STATUS_TOPIC = '/fpga/odom_adapter_status'
MOTION_PERMIT_TOPIC = '/fpga/motion_permit'

DEFAULT_STATUS_TOPICS = (LOCALIZATION_STATUS_TOPIC, ADAPTER_STATUS_TOPIC)

# Frames of the joint profile. They are CLI parameters everywhere; these are the defaults.
WORLD_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
TRACKED_FRAME = 'camera_link'
OPTICAL_FRAME = 'camera_optical_frame'

# Declared status keys each side must carry. Presence is what is checked, never the value:
# a label that is allowed to be false (navigation_ready) must still be present to be read.
LOCALIZATION_STATUS_KEYS = ('pose_source', 'navigation_ready', 'odom_source')
ADAPTER_STATUS_KEYS = ('pose_source', 'odom_source', 'twist_source', 'extrinsics_sha256')


def _clock(clock_ns):
    if clock_ns is None:
        return None
    if type(clock_ns) is not int or clock_ns < 0:
        raise ValueError('clock_ns must be a non-negative integer or None')
    return clock_ns


def base_row(topic, recv_monotonic_ns, clock_ns):
    if not isinstance(topic, str) or not topic:
        raise ValueError('topic must be a non-empty string')
    if type(recv_monotonic_ns) is not int:
        raise ValueError('recv_monotonic_ns must be an integer')
    return {'schema': SCHEMA, 'topic': topic, 'recv_monotonic_ns': recv_monotonic_ns,
            'clock_ns': _clock(clock_ns)}


def transform_row(parent, child, stamp_ns, static, translation=None, rotation=None):
    """One TransformStamped, flattened. `static` records which buffer it was fed into."""
    if type(stamp_ns) is not int or stamp_ns < 0:
        raise ValueError('stamp_ns must be a non-negative integer')
    row = {'parent': str(parent), 'child': str(child), 'stamp_ns': stamp_ns,
           'static': bool(static)}
    if translation is not None:
        row['translation'] = {k: float(translation[k]) for k in ('x', 'y', 'z')}
    if rotation is not None:
        row['rotation'] = {k: float(rotation[k]) for k in ('x', 'y', 'z', 'w')}
    return row


def tf_record(topic, recv_monotonic_ns, clock_ns, transforms):
    row = base_row(topic, recv_monotonic_ns, clock_ns)
    row['transforms'] = list(transforms)
    return row


def odom_record(topic, recv_monotonic_ns, clock_ns, frame_id, child_frame_id, stamp_ns):
    if type(stamp_ns) is not int or stamp_ns < 0:
        raise ValueError('stamp_ns must be a non-negative integer')
    row = base_row(topic, recv_monotonic_ns, clock_ns)
    row.update(frame_id=str(frame_id), child_frame_id=str(child_frame_id), stamp_ns=stamp_ns)
    return row


def clock_record(recv_monotonic_ns, clock_ns):
    return base_row(CLOCK_TOPIC, recv_monotonic_ns, clock_ns)


def status_record(topic, recv_monotonic_ns, clock_ns, text, parse=True):
    """A std_msgs/String status line. Unparseable payloads are recorded, never dropped."""
    row = base_row(topic, recv_monotonic_ns, clock_ns)
    row['raw_length'] = len(text)
    if not parse:
        row['payload'], row['parse_error'] = None, 'not_parsed'
        return row
    try:
        payload = json.loads(text)
    except ValueError as exc:
        row['payload'], row['parse_error'] = None, str(exc)
        return row
    if not isinstance(payload, dict):
        row['payload'], row['parse_error'] = None, 'payload is not a JSON object'
        return row
    row['payload'], row['parse_error'] = payload, None
    return row


def _can_fields(prefix, result, missing_reason):
    """A can_transform outcome: the bool, or why there is none."""
    if result is None:
        return {prefix: None, prefix + '_error': missing_reason}
    if isinstance(result, BaseException):
        return {prefix: None, prefix + '_error': type(result).__name__}
    return {prefix: bool(result), prefix + '_error': None}


def sample_record(recv_monotonic_ns, clock_ns, lookup_result, can_at_clock, can_at_odom,
                  odom_base_stamp_ns=None, world_frame=WORLD_FRAME, base_frame=BASE_FRAME):
    """The C1/C2 probe row derived on every /clock message.

    `lookup_result` is the stamp (ns) of `lookup_transform(world, base, Time())`, or the
    exception it raised. C1 is reported as an age against the same /clock value, so it is
    measured on the simulated clock, the one the consumer's `healthy()` also uses. For a
    chain the Time(0) lookup resolves at the LATEST COMMON time of its edges, so the age is
    governed by the slowest edge — here `odom->base_link`, stamped at the frame time.

    Two can_transform outcomes are recorded, at two different query times:

      c2_at_clock  `Time(clock_ns)` — the consumer's own now. In this replay every
                   `odom->base_link` stamp is a frame stamp and therefore behind the clock,
                   so this query asks tf2 to extrapolate into the future on that edge and
                   is expected to be false throughout. Recorded as evidence; it has no
                   discriminating power here and nothing judges it.
      c2_at_odom   `Time(odom_base_stamp_ns)` — the newest time the odometry edge actually
                   covers. This is the query that can distinguish a live held `map->odom`
                   from a retired one. None until an `odom->base_link` edge has been seen.
    """
    row = base_row(SAMPLE, recv_monotonic_ns, clock_ns)
    row['world_frame'], row['base_frame'] = str(world_frame), str(base_frame)
    if isinstance(lookup_result, BaseException):
        row.update(c1_stamp_ns=None, c1_age_s=None, c1_error=type(lookup_result).__name__)
    else:
        stamp = int(lookup_result)
        row.update(c1_stamp_ns=stamp, c1_error=None,
                   c1_age_s=None if clock_ns is None else (clock_ns - stamp) / 1e9)
    row['odom_base_stamp_ns'] = None if odom_base_stamp_ns is None else int(odom_base_stamp_ns)
    row.update(_can_fields('c2_at_clock', can_at_clock, 'no /clock value yet'))
    row.update(_can_fields('c2_at_odom', can_at_odom, 'no odom->base_link stamp yet'))
    return row


def summary_record(recv_monotonic_ns, clock_ns, counts, repeated_stamp_count,
                   sample_count, repeated_examples=(), **extra):
    row = base_row(SUMMARY, recv_monotonic_ns, clock_ns)
    row.update(counts={str(k): int(v) for k, v in dict(counts).items()},
               repeated_stamp_count=int(repeated_stamp_count),
               sample_count=int(sample_count),
               repeated_stamp_examples=[dict(item) for item in repeated_examples],
               **extra)
    return row


def topic_info_record(recv_monotonic_ns, clock_ns, results):
    row = base_row(TOPIC_INFO, recv_monotonic_ns, clock_ns)
    row['results'] = [dict(item) for item in results]
    return row


class NewestEdgeStamp:
    """Newest stamp seen on one dynamic edge.

    The recorder needs the newest `odom->base_link` stamp to ask tf2 the only C2 question
    with any discriminating power in this replay. Kept here, fed from the same /tf rows the
    recording is built from, so it cannot drift from what was recorded.
    """

    def __init__(self, parent, child):
        self.parent, self.child = str(parent), str(child)
        self.stamp_ns = None
        self.count = 0

    def observe(self, parent, child, stamp_ns, static=False):
        """Return True when this row advanced the newest stamp."""
        if static or str(parent) != self.parent or str(child) != self.child:
            return False
        stamp = int(stamp_ns)
        self.count += 1
        if self.stamp_ns is not None and stamp <= self.stamp_ns:
            return False
        self.stamp_ns = stamp
        return True


class RepeatedStampCounter:
    """Counts `(parent, child, stamp_ns)` triples seen more than once.

    This is the recorder's own account of what tf2 would warn about as
    TF_REPEATED_DATA. It is kept independently of the node logs precisely so the check
    has two books: a log grep can be defeated by a log level, this cannot.
    """

    def __init__(self):
        self.seen = set()
        self.repeats = Counter()
        self.count = 0

    def observe(self, parent, child, stamp_ns):
        """Return True when this exact triple has been seen before."""
        key = (str(parent), str(child), int(stamp_ns))
        if key in self.seen:
            self.count += 1
            self.repeats[key] += 1
            return True
        self.seen.add(key)
        return False

    def examples(self, limit=8):
        return [{'parent': parent, 'child': child, 'stamp_ns': stamp, 'extra_copies': extra}
                for (parent, child, stamp), extra in self.repeats.most_common(limit)]
