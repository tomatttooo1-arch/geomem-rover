#!/usr/bin/env python3
"""Audit raw ROS observations without changing scoring or rerunning algorithms.

Reports duplicates rather than silently taking the final message. Supplemental
health checks are not a replacement for the preregistered localization score.
No ground truth is read; finite poses and plausible covariance are not accuracy.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np


def covariance_checks(values):
    try:
        cov = np.asarray(values, dtype=float)
        finite = cov.size == 36 and np.isfinite(cov).all()
        if not finite:
            return {'finite_36': False, 'usable_numeric_covariance': False}
        cov = cov.reshape(6, 6)
        symmetric = bool(np.allclose(cov, cov.T, atol=1e-8, rtol=1e-5))
        minimum = float(np.linalg.eigvalsh((cov + cov.T) / 2).min())
        maximum_diagonal = float(np.diag(cov).max())
        return {'finite_36': True, 'symmetric': symmetric, 'minimum_eigenvalue': minimum,
            'max_diagonal': maximum_diagonal, 'has_9999_or_larger_diagonal': maximum_diagonal >= 9999.,
            'usable_numeric_covariance': bool(symmetric and minimum >= -1e-8 and maximum_diagonal < 9999.)}
    except (ValueError, TypeError, np.linalg.LinAlgError):
        return {'finite_36': False, 'usable_numeric_covariance': False}


def pose_checks(pose):
    try:
        xyz = np.asarray([pose['position'][key] for key in ('x', 'y', 'z')], dtype=float)
        quat = np.asarray([pose['orientation'][key] for key in ('x', 'y', 'z', 'w')], dtype=float)
        finite = bool(np.isfinite(xyz).all() and np.isfinite(quat).all())
        norm = float(np.linalg.norm(quat)) if finite else None
        unit = bool(finite and abs(norm - 1.) <= .001)
        return {'finite': finite, 'quaternion_norm': norm, 'unit_quaternion': unit}
    except (KeyError, ValueError, TypeError):
        return {'finite': False, 'quaternion_norm': None, 'unit_quaternion': False}


def audit(events):
    publications = [row for row in events if row['event'] == 'publish_begin']
    pub_stamps = Counter(row['stamp_ns'] for row in publications)
    pub_indices = Counter(row['index'] for row in publications)
    expected_frames = {'odom': 'odom', 'odom_info': 'odom', 'info': 'map', 'localization_pose': 'map'}
    indexed = {kind: defaultdict(list) for kind in expected_frames}
    message_audit = {}
    valid_by_kind = {}
    for kind, expected_frame in expected_frames.items():
        rows = [row for row in events if row['event'] == kind]
        records = []
        for row in rows:
            stamp = row['stamp_ns']
            indexed[kind][stamp].append(row)
            header = row.get('header', {})
            header_stamp = header.get('stamp', {})
            header_matches = header_stamp.get('sec', -1) * 1_000_000_000 + header_stamp.get('nanosec', -1) == stamp
            frame_correct = header.get('frame_id') == expected_frame
            if kind == 'odom':
                frame_correct = frame_correct and row.get('child_frame_id') == 'camera_link'
            entry = {'stamp_ns': stamp, 'published': stamp in pub_stamps,
                'header_stamp_matches': header_matches, 'frame_correct': frame_correct}
            valid = bool(header_matches and frame_correct and stamp in pub_stamps)
            if kind in ('odom', 'localization_pose'):
                entry['pose'] = pose_checks(row.get('pose', {}).get('pose', {}))
                entry['covariance'] = covariance_checks(row.get('pose', {}).get('covariance', []))
                valid = valid and entry['pose']['unit_quaternion'] and entry['covariance']['usable_numeric_covariance']
            elif kind == 'odom_info':
                entry['covariance'] = covariance_checks(row.get('covariance', []))
                entry['lost'] = row.get('lost', True)
                valid = valid and not entry['lost'] and entry['covariance']['usable_numeric_covariance']
            else:
                entry['has_map_link'] = row.get('loop_closure_id', 0) > 0 or row.get('proximity_detection_id', 0) > 0
                valid = valid and entry['has_map_link']
            entry['supplemental_numeric_health'] = bool(valid)
            records.append(entry)
        counts = Counter(row['stamp_ns'] for row in rows)
        valid_by_kind[kind] = {r['stamp_ns'] for r in records
            if r['supplemental_numeric_health'] and counts[r['stamp_ns']] == 1}
        message_audit[kind] = {'messages': len(rows), 'distinct_stamps': len(counts),
            'duplicate_stamps': {str(stamp): count for stamp, count in counts.items() if count > 1},
            'unknown_stamps': sorted(set(counts) - set(pub_stamps)),
            'missing_input_indices': [p['index'] for p in publications if p['stamp_ns'] not in counts],
            'records': records}
    def region(rows):
        return {'input_indices': [row['index'] for row in rows],
            'observed_odom_info_indices': [p['index'] for p in rows if p['stamp_ns'] in indexed['odom_info']],
            'lost_indices': [p['index'] for p in rows if any(r.get('lost', True)
                for r in indexed['odom_info'].get(p['stamp_ns'], []))],
            'numeric_valid_odom_and_info_indices': [p['index'] for p in rows
                if p['stamp_ns'] in valid_by_kind['odom'] & valid_by_kind['odom_info']],
            'numeric_valid_localization_with_link_and_odom_indices': [p['index'] for p in rows
                if p['stamp_ns'] in set.intersection(*valid_by_kind.values())]}
    start = next((row for row in events if row['event'] == 'start'), {})
    blank_stop = start.get('options', {}).get('blank_stop')
    recovery = None
    if blank_stop is not None:
        end = int(blank_stop)
        recovery = region([p for p in publications if end <= p['index'] < end + 30])
        recovery['window'] = [end, end + 30]
    tf_edges = defaultdict(list)
    for row in events:
        if row['event'] == 'tf':
            for transform in row['transforms']:
                value = transform['transform']
                header = transform['header']
                tf_edges[header['frame_id'] + '->' + transform['child_frame_id']].append({
                    'stamp_ns': header['stamp']['sec'] * 1_000_000_000 + header['stamp']['nanosec'],
                    'pose': pose_checks({'position': value['translation'], 'orientation': value['rotation']})})
    tf_audit = {edge: {'messages': len(rows), 'distinct_stamps': len({r['stamp_ns'] for r in rows}),
        'invalid_transform_count': sum(not r['pose']['unit_quaternion'] for r in rows),
        'min_stamp_ns': min(r['stamp_ns'] for r in rows), 'max_stamp_ns': max(r['stamp_ns'] for r in rows)}
        for edge, rows in tf_edges.items()}
    return {'protocol': 'supplemental raw-message audit; no change to preregistered scoring',
        'publication_count': len(publications),
        'duplicate_publication_stamps': {str(s): n for s, n in pub_stamps.items() if n > 1},
        'duplicate_publication_indices': {str(s): n for s, n in pub_indices.items() if n > 1},
        'clock_publication_count': sum(row['event'] == 'clock_published' for row in events),
        'messages': message_audit, 'all_frames': region(publications),
        'blank_frames': region([p for p in publications if p.get('blank', False)]),
        'recovery': recovery, 'tf': tf_audit,
        'limitations': 'Numeric health is not accuracy. Covariance is estimated, not calibrated uncertainty. '
            'TF may repeat and may use a future validity stamp; edge presence is not transform correctness. '
            'Recovery lists need independent fixed-map error scoring. Duplicate samples are excluded from health sets.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('observations', type=Path)
    args = parser.parse_args()
    result = audit([json.loads(line) for line in args.observations.read_text().splitlines()])
    print(json.dumps(result, indent=2, allow_nan=False))
