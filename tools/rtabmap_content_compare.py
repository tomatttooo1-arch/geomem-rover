#!/usr/bin/env python3
"""Per-frame content comparison of two RTAB-Map probe runs. Pure, no container, no ROS.

The question these functions answer is narrow: given two `observations.jsonl` files from
runs that were fed the SAME bytes, did RTAB-Map produce the same numbers? So the module
reads exactly two kinds of row:

* `publish_begin` — what the replay actually put on the wire. This is the PREREQUISITE:
  if the two runs were not fed identical images, every content difference below is
  explained by the input and nothing can be concluded about RTAB-Map. It is checked
  separately and reported separately, never folded into the content verdict.
* `odom_info` / `info` / `localization_pose` — RTAB-Map's own output, restricted to the
  fields `host/localization_gate.py` reads plus the registration counters that yesterday's
  drift analysis found diverging at frame 1.

Frames are matched by `stamp_ns`, which is the dataset's own timestamp and therefore the
one key that cannot drift between runs. A stamp present in one run and not the other is
reported as such rather than silently dropped: a missing message is a difference.
"""

import json
from pathlib import Path

INPUT_FIELDS = ('published_rgb_sha256', 'published_depth_sha256', 'valid_depth_pixels')

# (event, field) pairs, flattened to 'event.field' keys. §4 of
# docs/design/RTABMAP_RNG_SOURCE_PROTOCOL_20260915.md, transcribed.
CONTENT_FIELDS = (
    ('odom_info', ('features', 'matches', 'inliers', 'lost', 'transform', 'covariance')),
    ('info', ('loop_closure_id', 'proximity_detection_id', 'loop_closure_transform')),
    ('localization_pose', ('pose',)),
)
CONTENT_KEYS = tuple(f'{event}.{field}' for event, fields in CONTENT_FIELDS for field in fields)

MISSING = '<absent>'


def read_events(path):
    """Yield the parsed rows of a jsonl file, skipping blank lines."""
    with Path(path).open() as stream:
        for line in stream:
            line = line.strip()
            if line:
                yield json.loads(line)


def load_stream(path):
    """Index one run's observations by frame stamp.

    Returns {'source', 'order' (stamps in publish order), 'input', 'content',
    'event_counts', 'duplicate_stamps'}. `input[stamp]` and `content[stamp]` are flat dicts;
    a field whose message never arrived holds MISSING, so a dropped message compares as a
    difference instead of disappearing.
    """
    order, seen = [], set()
    duplicates = []
    inputs, content = {}, {}
    counts = {}
    wanted = {event: fields for event, fields in CONTENT_FIELDS}
    for row in read_events(path):
        event = row.get('event')
        counts[event] = counts.get(event, 0) + 1
        stamp = row.get('stamp_ns')
        if stamp is None:
            continue
        if event == 'publish_begin':
            if stamp in seen:
                duplicates.append(stamp)
                continue
            seen.add(stamp)
            order.append(stamp)
            inputs[stamp] = {field: row.get(field, MISSING) for field in INPUT_FIELDS}
            content[stamp] = {key: MISSING for key in CONTENT_KEYS}
        elif event in wanted and stamp in content:
            for field in wanted[event]:
                content[stamp][f'{event}.{field}'] = row.get(field, MISSING)
    return {'source': str(path), 'order': order, 'input': inputs, 'content': content,
            'event_counts': counts, 'duplicate_stamps': duplicates}


def _compare_maps(left, right, stamps, index_of, keys):
    """Field-by-field comparison over a shared stamp list. Pure helper."""
    per_field = {key: 0 for key in keys}
    differing = []
    for stamp in stamps:
        a, b = left[stamp], right[stamp]
        differs = False
        for key in keys:
            if a.get(key, MISSING) != b.get(key, MISSING):
                per_field[key] += 1
                differs = True
        if differs:
            differing.append(stamp)
    first = min((index_of[stamp] for stamp in differing), default=None)
    return {'frames_compared': len(stamps), 'differing_frames': len(differing),
            'first_differing_index': first,
            'first_differing_stamp_ns': (differing[0] if differing else None),
            'identical': not differing,
            'per_field_differing_frames': per_field,
            'differing_indices': sorted(index_of[stamp] for stamp in differing)}


def compare_streams(left, right):
    """Compare two loaded streams. Returns the input check and the content comparison.

    `input_identical` is the prerequisite known answer: false means the content numbers
    below cannot be attributed to RTAB-Map at all.
    """
    shared = [stamp for stamp in left['order'] if stamp in right['content']]
    index_of = {stamp: index for index, stamp in enumerate(left['order'])}
    result = {
        'left': left['source'], 'right': right['source'],
        'frames_left': len(left['order']), 'frames_right': len(right['order']),
        'stamp_order_equal': left['order'] == right['order'],
        'only_in_left': [stamp for stamp in left['order'] if stamp not in right['content']],
        'only_in_right': [stamp for stamp in right['order'] if stamp not in left['content']],
    }
    result['input'] = _compare_maps(left['input'], right['input'], shared, index_of,
                                    INPUT_FIELDS)
    result['input_identical'] = (result['input']['identical'] and result['stamp_order_equal']
                                 and not result['only_in_left'] and not result['only_in_right'])
    result['content'] = _compare_maps(left['content'], right['content'], shared, index_of,
                                      CONTENT_KEYS)
    result['content_identical'] = result['content']['identical'] and result['stamp_order_equal']
    return result


def percentiles(values, points=(50, 90, 99)):
    """Nearest-rank percentiles plus min/max. Small, explicit, and unit tested."""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return {'count': 0}
    result = {'count': len(ordered), 'min': ordered[0], 'max': ordered[-1]}
    for point in points:
        rank = max(1, min(len(ordered), int(-(-point * len(ordered) // 100))))
        result[f'p{point}'] = ordered[rank - 1]
    return result


def verdict(a_identical, c_identical, prerequisite_ok):
    """§5 of the protocol, mechanically. Returns (code, candidates, reason). Pure.

    `a_identical` / `c_identical` are None when the pair was not run or not comparable.
    """
    unjudged = {'H-thread': '未判', 'H-flann': '未判', 'H-ransac': '未判'}
    if not prerequisite_ok:
        return 'INSTRUMENT_ERROR', dict(unjudged), (
            '前置已知答案失败：两臂的输入不是逐字节相同，内容差异不可归因')
    if a_identical is None:
        return 'UNJUDGED', dict(unjudged), 'A1/A2 不可比（容器失败或产物缺失）'
    if a_identical:
        return 'H-THREAD', dict(unjudged, **{'H-thread': '成立'}), (
            'A1 ≡ A2：把 OpenMP 并行度钉成 1 之后 230 帧全部字段逐值相同')
    if c_identical is None:
        return 'UNJUDGED', dict(unjudged, **{'H-thread': '排除'}), (
            'A1 ≠ A2，但 C 臂不可比（未跑或容器失败）')
    if c_identical:
        return 'H-FLANN', dict(unjudged, **{'H-thread': '排除', 'H-flann': '成立'}), (
            'A1 ≠ A2 且 C1 ≡ C2：换暴力最近邻后逐值相同')
    return 'RESIDUAL', {'H-thread': '排除', 'H-flann': '排除', 'H-ransac': '未判'}, (
        'C1 ≠ C2：单线程与暴力最近邻都钉住后仍不同，残差在 RANSAC/其它')
