#!/usr/bin/env python3
"""Replay recorded arrival order and monotonic intervals through the online policy.

No truth enters the policy. Optional frozen independent evaluation/audit are used
only AFTER approval to report false approvals and availability denominators.
"""
import argparse
import hashlib
import json
from pathlib import Path
from host.localization_gate import LocalizationGate, KINDS
from host.ros_localization_gate import load_prior


def replay(events, map_id, ids, timing='recorded'):
    gate = LocalizationGate(map_id, ids)
    ros_ns = 0
    clock_events = any(e['event'] == 'clock_published' for e in events)
    trace, approvals = [], []
    origin = events[0]['monotonic_ns']
    next_tick = 0.
    for ordinal, event in enumerate(events):
        wall = (event['monotonic_ns']-origin)/1e9 if timing == 'recorded' else ordinal*.001
        while next_tick <= wall:
            gate.tick(next_tick, ros_ns)
            next_tick += .1
        kind = event['event']
        if kind == 'clock_published':
            ros_ns = event['stamp_ns']
        elif kind == 'publish_begin':
            # The publisher sets /clock immediately after begin, before ROS delivers RGB.
            # Use that acquisition clock for registration; all algorithm arrival times stay exact.
            ros_ns = event['stamp_ns']
            gate.acquisition(event['stamp_ns'], wall, ros_ns)
        elif kind in KINDS:
            outputs = gate.ingest(kind, event, wall, ros_ns)
            approvals.extend(outputs)
        else:
            continue
        trace.append({'event': kind, 'wall_s': wall, 'input_stamp_ns': event.get('stamp_ns'),
                      'state': gate.state, 'reason': gate.reason, 'epoch': gate.epoch,
                      'approved_stamps': list(gate.approved)})
    return {'timing': timing, 'clock_publications_present': clock_events, 'approvals': approvals, 'trace': trace}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('observations', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--database', required=True, type=Path)
    p.add_argument('--map-id', required=True)
    p.add_argument('--evaluation', type=Path)
    p.add_argument('--audit', type=Path)
    p.add_argument('--timing', choices=('recorded', 'compact'), default='recorded',
                   help='compact is a separate timing control: 1ms per recorded event, never a runtime result')
    a = p.parse_args()
    raw = a.observations.read_bytes()
    events = [json.loads(line) for line in raw.splitlines()]
    result = replay(events, a.map_id, load_prior(a.database, a.map_id), a.timing)
    pubs = {e['stamp_ns']: e['index'] for e in events if e['event'] == 'publish_begin'}
    approved = {v['stamp_ns'] for v in result['approvals']}
    result['summary'] = {'published': len(pubs), 'approved': len(approved),
                          'approved_indices': [pubs[s] for s in sorted(approved)],
                          'raw_sha256': hashlib.sha256(raw).hexdigest(),
                          'protocol': a.timing+' arrival timing; synthetic steady 0.1s timer; no algorithm rerun'}
    if a.evaluation:
        scored = json.loads(a.evaluation.read_text())['scored_localizations']
        result['summary'].update(
            false_approvals=[r for r in scored if r['stamp_ns'] in approved and not r['good']],
            large_error_inputs=[{'index': r['index'], 'error_m': r['error_m'], 'approved': r['stamp_ns'] in approved}
                                for r in scored if r['error_m'] is not None and r['error_m'] > .15])
    if a.audit:
        healthy = set(json.loads(a.audit.read_text())['all_frames']['numeric_valid_localization_with_link_and_odom_indices'])
        result['summary'].update(healthy_before_gate=len(healthy),
            healthy_withheld_indices=sorted(healthy-set(result['summary']['approved_indices'])))
    with a.output.open('x') as f:
        json.dump(result, f, indent=2, allow_nan=False)
    print(json.dumps(result['summary'], indent=2))


if __name__ == '__main__':
    main()
