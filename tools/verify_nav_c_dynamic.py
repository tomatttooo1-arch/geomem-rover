#!/usr/bin/env python3
"""Derive a fail-closed verdict for one baseline/dynamic Nav C run."""
import argparse
import json
import math
from pathlib import Path


DYNAMIC_CASES = frozenset(('obstacle', 'wall', 'removal'))
SUPPORTED_CASES = DYNAMIC_CASES | {'baseline'}
OCCUPIED_MIN = 90


def read_jsonl(path):
    rows = []
    for line in path.read_text().splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _stamp(value):
    return value['sec'] + value['nanosec'] / 1e9


def _pose(row):
    value = row['message']['pose']['pose']
    p, q = value['position'], value['orientation']
    yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                     1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
    return p['x'], p['y'], yaw


def _distance(rows, after):
    odom = [row for row in rows if row.get('topic') == '/odom'
            and row.get('wall_monotonic', -math.inf) >= after]
    if len(odom) < 2:
        return None
    return math.dist(_pose(odom[0])[:2], _pose(odom[-1])[:2])


def _detour(rows, after, center, size):
    poses = [_pose(row) for row in rows if row.get('topic') == '/odom'
             and row.get('wall_monotonic', -math.inf) >= after]
    if len(poses) < 3:
        return None
    x_low, x_high = center[0] - size[0] / 2, center[0] + size[0] / 2
    y_low, y_high = center[1] - size[1] / 2, center[1] + size[1] / 2
    return {'samples': len(poses),
            'starts_before_and_finishes_after': poses[0][1] < y_low and poses[-1][1] > y_high,
            'center_entered_obstacle_footprint': any(x_low <= x <= x_high and y_low <= y <= y_high
                                                     for x, y, _ in poses),
            'max_lateral_offset_m': max(abs(x - center[0]) for x, _, _ in poses),
            'passed': (poses[0][1] < y_low and poses[-1][1] > y_high
                       and not any(x_low <= x <= x_high and y_low <= y <= y_high
                                   for x, y, _ in poses)
                       and max(abs(x - center[0]) for x, _, _ in poses) > size[0] / 2)}


def _transform_xy(transform, point):
    message = transform['message']['transform']
    t, q = message['translation'], message['rotation']
    yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                     1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
    c, s = math.cos(yaw), math.sin(yaw)
    return c * point[0] - s * point[1] + t['x'], s * point[0] + c * point[1] + t['y']


def _transform_xyz(transform, point):
    message = transform['message']['transform']
    t, q = message['translation'], message['rotation']
    # Quaternion-vector rotation without assuming a level camera.
    ux, uy, uz, w = q['x'], q['y'], q['z'], q['w']
    dot = ux * point[0] + uy * point[1] + uz * point[2]
    cross = (uy * point[2] - uz * point[1], uz * point[0] - ux * point[2],
             ux * point[1] - uy * point[0])
    rotated = tuple(2 * dot * u + (w * w - ux * ux - uy * uy - uz * uz) * p + 2 * w * c
                    for u, p, c in zip((ux, uy, uz), point, cross))
    return tuple(rotated[i] + t[axis] for i, axis in enumerate(('x', 'y', 'z')))


def _ray_clears_box(origin, endpoint, center, size):
    """True when a measured endpoint is beyond a box crossed by the sensor ray."""
    enter, leave = 0., 1.
    for axis in range(3):
        low, high = center[axis] - size[axis] / 2, center[axis] + size[axis] / 2
        delta = endpoint[axis] - origin[axis]
        if abs(delta) < 1e-9:
            if origin[axis] < low or origin[axis] > high:
                return False
            continue
        a, b = (low - origin[axis]) / delta, (high - origin[axis]) / delta
        if a > b:
            a, b = b, a
        enter, leave = max(enter, a), min(leave, b)
        if enter > leave:
            return False
    return 0 <= enter <= leave < .98


def free_rays(row, center, size):
    transform = row.get('transform_map_from_cloud', {})
    if transform.get('status') not in ('ok', 'identity'):
        return 0
    message = transform['message']['transform']
    origin = tuple(message['translation'][axis] for axis in ('x', 'y', 'z'))
    points = {tuple(point) for point in row.get('finite_xyz', [])
              if isinstance(point, list) and len(point) == 3
              and all(isinstance(v, (int, float)) and math.isfinite(v) for v in point)}
    return sum(_ray_clears_box(origin, _transform_xyz(transform, point), center, size)
               for point in points)


def _cell(info, point):
    origin = info['origin']
    p, q = origin['position'], origin['orientation']
    yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                     1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
    dx, dy = point[0] - p['x'], point[1] - p['y']
    c, s = math.cos(yaw), math.sin(yaw)
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    return int(math.floor(local_x / info['resolution'])), int(math.floor(local_y / info['resolution']))


def _associated(row, transform_key):
    transform = row.get(transform_key, {})
    header = row.get('header', {}).get('stamp', {})
    try:
        message_stamp = _stamp(header)
        transform_stamp = _stamp(transform['message']['header']['stamp'])
        clock = row['sim_clock_s']
        received = row['message_received_monotonic']
        clock_received = row['clock_received_monotonic']
    except (KeyError, TypeError):
        return False
    values = (message_stamp, transform_stamp, clock, received, clock_received)
    return (transform.get('status') in ('ok', 'identity')
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
            and 0 <= clock - message_stamp <= 1
            and abs(transform_stamp - message_stamp) <= 1e-6
            and 0 <= received - clock_received <= .5)


def costmap_region_samples(rows, center, size):
    """Reconstruct full/update streams and sample the declared map-space footprint."""
    states, samples = {}, []
    for row in rows:
        topic = row.get('topic', '')
        # str.removesuffix is 3.9+; this is its exact behaviour for a non-empty suffix.
        base = topic[:-len('_updates')] if topic.endswith('_updates') else topic
        if base not in ('/local_costmap/costmap', '/global_costmap/costmap'):
            continue
        # A missing/rejected delta breaks reconstruction until a new full grid.
        if not _associated(row, 'transform_map_to_frame'):
            states.pop(base, None)
            continue
        message = row.get('message', {})
        if row.get('kind') == 'full':
            states.pop(base, None)
            info = message.get('info', {})
            width, height = info.get('width'), info.get('height')
            data = message.get('data')
            if (not isinstance(width, int) or not isinstance(height, int)
                    or width <= 0 or height <= 0
                    or not isinstance(data, list) or len(data) != width * height):
                continue
            states[base] = {'info': info, 'data': list(data), 'width': width, 'height': height,
                            'frame': row['header'].get('frame_id'), 'stamp': _stamp(row['header']['stamp'])}
        elif row.get('kind') == 'update' and base in states:
            state = states[base]
            if (row['header'].get('frame_id') != state['frame']
                    or _stamp(row['header']['stamp']) < state['stamp']):
                states.pop(base, None)
                continue
            x, y = message.get('x'), message.get('y')
            width, height, data = message.get('width'), message.get('height'), message.get('data')
            if not all(isinstance(v, int) for v in (x, y, width, height)) or not isinstance(data, list):
                states.pop(base, None)
                continue
            if width <= 0 or height <= 0 or x < 0 or y < 0 or x + width > state['width'] or y + height > state['height'] or len(data) != width * height:
                states.pop(base, None)
                continue
            state['stamp'] = _stamp(row['header']['stamp'])
            for j in range(height):
                start = (y + j) * state['width'] + x
                state['data'][start:start + width] = data[j * width:(j + 1) * width]
        else:
            continue
        state = states.get(base)
        transform = row.get('transform_map_to_frame', {})
        if state is None or not _associated(row, 'transform_map_to_frame'):
            continue
        # Sample a grid across the model footprint; this supports rolling/rotated grids.
        values = []
        for ix in range(9):
            for iy in range(5):
                map_point = (center[0] + size[0] * (ix / 8 - .5),
                             center[1] + size[1] * (iy / 4 - .5))
                gx, gy = _cell(state['info'], _transform_xy(transform, map_point))
                if 0 <= gx < state['width'] and 0 <= gy < state['height']:
                    values.append(state['data'][gy * state['width'] + gx])
        if values:
            samples.append({'topic': base, 'received_monotonic': row['received_monotonic'],
                            'sim_clock_s': row.get('sim_clock_s'),
                            'stamp_s': _stamp(row['header']['stamp']), 'maximum': max(values),
                            'known_cells': sum(value >= 0 for value in values),
                            'occupied_cells': sum(value >= OCCUPIED_MIN for value in values)})
    return samples


def evaluate(case, injection, observations, worker, motion, cdr_exists=lambda name: True):
    if case not in SUPPORTED_CASES:
        raise ValueError('dynamic verifier only accepts baseline/obstacle/wall/removal')
    missing, failures, checks = [], [], {}

    events = {row.get('event'): row for row in injection}
    start_name = 'baseline_started' if case == 'baseline' else 'obstacle_inserted'
    event = events.get(start_name)
    if event is None:
        missing.append(start_name)
        event_time = math.inf
    else:
        event_time = event.get('wall_monotonic', math.inf)
        association = all(isinstance(event.get(key), (int, float))
                          for key in ('sim_clock_s', 'clock_received_monotonic', 'clock_age_s'))
        checks['injection_clock_associated'] = association and 0 <= event['clock_age_s'] <= .5
        if not checks['injection_clock_associated']:
            missing.append(start_name + ':fresh_clock_association')
    if case in DYNAMIC_CASES:
        responses = [row for row in injection if row.get('event') == 'response']
        checks['create_acknowledged'] = bool(responses and responses[0].get('returncode') == 0
                                             and 'data: true' in responses[0].get('stdout', ''))
        if not checks['create_acknowledged']:
            failures.append('create_not_acknowledged')
    if 'injection_complete' not in events:
        missing.append('injection_complete')
    remove = events.get('obstacle_removed')
    if case == 'removal':
        if remove is None:
            missing.append('obstacle_removed')
        else:
            association = all(isinstance(remove.get(key), (int, float))
                              for key in ('sim_clock_s', 'clock_received_monotonic', 'clock_age_s'))
            checks['removal_clock_associated'] = association and 0 <= remove['clock_age_s'] <= .5
            if not checks['removal_clock_associated']:
                missing.append('obstacle_removed:fresh_clock_association')
        responses = [row for row in injection if row.get('event') == 'response']
        checks['remove_acknowledged'] = len(responses) >= 2 and responses[1].get('returncode') == 0 and 'data: true' in responses[1].get('stdout', '')
        if not checks['remove_acknowledged']:
            failures.append('remove_not_acknowledged')

    summaries = [row for row in worker if row.get('event') == 'summary']
    results = [row for row in worker if row.get('event') == 'case_result']
    if len(summaries) != 1 or len(results) != 1:
        missing.append('single_worker_summary_and_result')
    else:
        summary, result = summaries[0], results[0]
        if summary.get('error') == 'goal_wall_timeout':
            failures.append('goal_wall_timeout_is_not_a_verdict')
        expected_status = 6 if case == 'wall' else 4
        checks['action_terminal'] = (summary.get('passed') is True and result.get('passed') is True
                                     and result.get('status') == expected_status
                                     and result.get('stopped') is True)
        if not checks['action_terminal']:
            failures.append('wrong_action_terminal_or_not_stopped')

    event_clock = event.get('sim_clock_s', math.inf) if event else math.inf
    points = [row for row in observations if row.get('topic') == '/fpga/points'
              and _stamp(row.get('header', {}).get('stamp', {'sec': -math.inf, 'nanosec': 0})) > event_clock]
    usable_points = [row for row in points if _associated(row, 'transform_map_from_cloud')
                     and isinstance(row.get('stamp_s'), (int, float))
                     and abs(_stamp(row['header']['stamp']) - row['stamp_s']) <= 1e-6
                     and cdr_exists(row.get('file'))]
    checks['pointcloud_tf_time_associated'] = bool(usable_points)
    if not usable_points:
        missing.append('post_event_pointcloud_with_tf_and_time')
    paths = [row for row in observations if row.get('topic') == '/plan' and row.get('poses')]
    checks['path_recorded'] = bool(paths)
    if not paths:
        missing.append('nonempty_plan')

    distance_after = _distance(motion, event_time)
    checks['motion_after_event_m'] = distance_after
    if case in ('baseline', 'obstacle'):
        if distance_after is None:
            missing.append('post_event_odom')
        elif distance_after < .15:
            failures.append('insufficient_post_event_motion')
    if case == 'obstacle' and event is not None:
        detour = _detour(motion, event_time, event['center_map'], event['size_m'])
        checks['actual_detour'] = detour
        if detour is None:
            missing.append('detour_odom_samples')
        elif not detour['passed']:
            failures.append('actual_motion_does_not_show_detour')
    if case == 'removal' and remove is not None:
        distance_after_remove = _distance(motion, remove.get('wall_monotonic', math.inf))
        checks['motion_after_removal_m'] = distance_after_remove
        post_remove_points = [row for row in usable_points
                              if row['stamp_s'] > remove.get('sim_clock_s', math.inf)]
        if not post_remove_points:
            missing.append('post_removal_pointcloud_with_tf_and_time')
        # One cloud must independently contain the three distinct rays; repeated
        # log entries or repeated frames cannot accumulate evidence.
        clearing_rays = max((free_rays(row, event['center_map'], event['size_m'])
                             for row in post_remove_points), default=0)
        poses = [_pose(row) for row in motion if row.get('topic') == '/odom'
                 and row.get('wall_monotonic', -math.inf) >= remove.get('wall_monotonic', math.inf)]
        center, size = event['center_map'], event['size_m']
        before = through = after = False
        for x, y, _ in poses:
            if y < center[1] - size[1] / 2:
                before = True
            if before and abs(x - center[0]) <= size[0] / 2 and abs(y - center[1]) <= size[1] / 2:
                through = True
            if through and y > center[1] + size[1] / 2:
                after = True
        checks['passage_through_removed_obstacle'] = before and through and after
        if not checks['passage_through_removed_obstacle']:
            failures.append('no_passage_through_removed_obstacle')
        checks['post_removal_free_rays_through_old_volume'] = clearing_rays
        if post_remove_points and clearing_rays < 3:
            missing.append('post_removal_free_ray_evidence')
        if distance_after_remove is None:
            missing.append('post_removal_odom')
        elif distance_after_remove < .05:
            failures.append('no_motion_after_removal')

    if case in DYNAMIC_CASES and event is not None:
        samples = costmap_region_samples(observations, event['center_map'], event['size_m'])
        post_insert = [sample for sample in samples
                       if isinstance(sample.get('stamp_s'), (int, float))
                       and sample['stamp_s'] > event_clock]
        checks['costmap_region_samples'] = len(samples)
        if not post_insert:
            missing.append('post_insert_costmap_region')
        elif not any(sample['occupied_cells'] for sample in post_insert):
            failures.append('obstacle_not_observed_in_costmap')
        if case == 'removal' and remove is not None:
            remove_clock = remove.get('sim_clock_s', math.inf)
            between = [sample for sample in samples if isinstance(sample.get('stamp_s'), (int, float))
                       and event_clock < sample['stamp_s'] < remove_clock]
            after = [sample for sample in samples if isinstance(sample.get('stamp_s'), (int, float))
                     and sample['stamp_s'] > remove_clock]
            if not between or not after:
                missing.append('costmap_samples_before_and_after_removal')
            elif not any(sample['occupied_cells'] for sample in between):
                failures.append('obstacle_never_marked_before_removal')
            elif not any(
                    any(before['topic'] == later['topic'] and before['occupied_cells']
                        for before in between)
                    and later['known_cells'] and not later['occupied_cells']
                    for later in after):
                failures.append('obstacle_region_not_cleared_after_removal')

    if failures:
        verdict = 'failed'
    elif missing:
        verdict = 'insufficient_evidence'
    else:
        verdict = 'passed'
    return {'case': case, 'verdict': verdict, 'passed': verdict == 'passed',
            'checks': checks, 'failures': failures, 'missing_evidence': missing,
            'acceptance_scope': 'single Nav C simulation case', 'performance_claim': False}


def terminal_motion_check(case, worker, motion):
    """Cross-check the worker's terminal result against server odometry."""
    results = [row for row in worker if row.get('event') == 'case_result']
    if len(results) != 1:
        return {'passed': False, 'reason': 'missing_terminal_result'}
    result = results[0]
    at = result.get('sim_s')
    if not isinstance(at, (int, float)) or not math.isfinite(at):
        return {'passed': False, 'reason': 'missing_terminal_sim_time'}
    odom = [row for row in motion if row.get('topic') == '/odom'
            and 0 <= _stamp(row['message']['header']['stamp']) - at <= .75]
    if len(odom) < 5:
        return {'passed': False, 'reason': 'missing_terminal_odom_samples'}
    stamps = [_stamp(row['message']['header']['stamp']) for row in odom]
    if (any(not 0 < b-a <= .3 for a, b in zip(stamps, stamps[1:]))
            or stamps[0]-at > .3 or stamps[-1]-at < .5):
        return {'passed': False, 'reason': 'unordered_or_stale_terminal_odom'}
    # Recorder and worker sample odometry at different rates. Verify the entire
    # post-result window, not five pre-result samples of a slower recorder.
    tail = odom
    speeds = [sum(abs(row['message']['twist']['twist'][part][axis])
                  for part in ('linear', 'angular') for axis in ('x', 'y', 'z'))
              for row in tail]
    poses = [_pose(row) for row in tail]
    stationary = all(math.isfinite(v) and v < .01 for v in speeds)
    stationary = stationary and all(math.dist(poses[0][:2], pose[:2]) < .005
        and abs(math.atan2(math.sin(pose[2]-poses[0][2]), math.cos(pose[2]-poses[0][2]))) < .01
        for pose in poses)
    xy = yaw = None
    arrived = True
    if case != 'wall':
        # Fixed C target, independent of a potentially incorrect worker target.
        xy = math.hypot(poses[-1][0], poses[-1][1] - 2.2)
        yaw = abs(math.atan2(math.sin(poses[-1][2]-math.pi/2), math.cos(poses[-1][2]-math.pi/2)))
        arrived = xy <= .03 and yaw <= .03
    return {'passed': bool(stationary and arrived), 'stopped_samples': len(tail),
            'server_stopped': stationary, 'server_arrived': arrived,
            'xy_error_m': xy, 'yaw_error_rad': yaw}


def verify_run(run):
    try:
        config = json.loads((run / 'run-config.json').read_text())
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return {'case': None, 'verdict': 'insufficient_evidence', 'passed': False,
                'checks': {}, 'failures': [], 'missing_evidence': ['file:run-config'],
                'acceptance_scope': 'single Nav C simulation case', 'performance_claim': False}
    case = config.get('nav_c')
    required = {'injection': run / 'c-injection.jsonl',
                'observations': run / 'agx/c-observe/observations.jsonl',
                'worker': run / 'agx/c-worker.jsonl', 'motion': run / 'motion.jsonl'}
    absent = [name for name, path in required.items() if not path.is_file()]
    if absent:
        return {'case': case, 'verdict': 'insufficient_evidence', 'passed': False,
                'checks': {}, 'failures': [], 'missing_evidence': ['file:' + name for name in absent],
                'acceptance_scope': 'single Nav C simulation case', 'performance_claim': False}
    try:
        rows = {name: read_jsonl(path) for name, path in required.items()}
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return {'case': case, 'verdict': 'insufficient_evidence', 'passed': False,
                'checks': {}, 'failures': [],
                'missing_evidence': ['unreadable_evidence:' + type(exc).__name__],
                'acceptance_scope': 'single Nav C simulation case', 'performance_claim': False}
    observe_dir = run / 'agx/c-observe'
    try:
        report = evaluate(case, rows['injection'], rows['observations'], rows['worker'], rows['motion'],
                          cdr_exists=lambda name: isinstance(name, str) and (observe_dir / name).is_file())
        terminal = terminal_motion_check(case, rows['worker'], rows['motion'])
        report['checks']['server_terminal_motion'] = terminal
        if not terminal['passed']:
            report['passed'] = False
            if 'reason' in terminal:
                report['missing_evidence'].append('server_terminal_motion:' + terminal['reason'])
                if report['verdict'] != 'failed':
                    report['verdict'] = 'insufficient_evidence'
            else:
                report['failures'].append('server_terminal_motion_rejected')
                report['verdict'] = 'failed'
        if any(row.get('event') == 'summary' and row.get('passed') is False for row in rows['worker']):
            report['failures'].append('worker_reported_failure')
            report['passed'], report['verdict'] = False, 'failed'
        return report
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        return {'case': case, 'verdict': 'insufficient_evidence', 'passed': False,
                'checks': {}, 'failures': [],
                'missing_evidence': ['malformed_evidence:' + type(exc).__name__],
                'acceptance_scope': 'single Nav C simulation case', 'performance_claim': False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    report = verify_run(args.run.resolve())
    body = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if args.out:
        args.out.write_text(body)
    print(body, end='')
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
