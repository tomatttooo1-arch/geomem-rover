"""Offline evidence association; never infer consumer receipt from driver receipt."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


LIMITS = [
    'driver receipt is not runtime receipt; runtime clock/TF/token/queue state remain unknown',
    'gate output bracket includes serialization/write, pose/TF construction/publication and scheduling; not pure logging cost',
    'generic bounded clock/TF timeout also covers localization-token wait; no warning-to-stamp association is available',
    'timestamps are callback observations, not DDS transport latency or CPU time',
    'duplicate callback keys are ambiguous; no interval is selected or averaged',
]


def stamp(message):
    s = message['header']['stamp']
    return s['sec'] * 10**9 + s['nanosec']


def audit(run):
    run = Path(run)
    sources = {}

    def read(name, lines=False, optional=False):
        path = run / name
        if not path.exists() and optional:
            sources[name] = {'state': 'unknown', 'reason': 'file absent'}
            return []
        raw = path.read_bytes()
        sources[name] = {'path': str(path.resolve()), 'sha256': hashlib.sha256(raw).hexdigest()}
        if name.endswith('.log'):
            return raw.decode().splitlines()
        return [json.loads(x) for x in raw.decode().splitlines()] if lines else json.loads(raw)

    result = read('result.json')
    observed = read('observed-messages.json')
    field_states = {}
    for name, obj, fields in [('result.json', result, ['checks']),
                              ('observed-messages.json', observed, ['statuses', 'poses', 'tf'])]:
        if not isinstance(obj, dict):
            raise ValueError(name + ' must contain an object')
        for field in fields:
            if field in obj and not isinstance(obj[field], list):
                raise ValueError(name + ':' + field + ' must be an array')
            field_states[name + ':' + field] = 'recorded' if field in obj else 'unknown'
    gate = read('gate-events.jsonl', lines=True, optional=True)
    runtime = read('runtime-events.jsonl', lines=True)
    log = read('runtime.log')
    frames = {}

    def frame(s):
        return frames.setdefault(s, {'stamp_ns': s, 'driver_valid_status_receipts': [],
            'driver_pose_receipts': [], 'driver_tf_receipts': [], 'runtime_processed': [],
            'gate_events': [], 'gate_output_brackets': [], 'runtime_receipt_and_wait_state': 'unknown'})

    for field, target in [('poses', 'driver_pose_receipts'), ('tf', 'driver_tf_receipts')]:
        for i, (wall, msg) in enumerate(observed.get(field, [])):
            stamps = {stamp(t) for t in msg['transforms']} if field == 'tf' else {stamp(msg)}
            for s in stamps:
                receipt = {'monotonic_s': wall, 'source_index': i}
                if field == 'tf':
                    receipt['edges'] = [{'parent': t['header']['frame_id'], 'child': t['child_frame_id']}
                                        for t in msg['transforms'] if stamp(t) == s]
                    receipt['transform_path_validity'] = 'unknown; receipt does not validate required edges or runtime availability'
                frame(s)[target].append(receipt)
    for i, (wall, row) in enumerate(observed.get('statuses', [])):
        if row['state'] == 'valid':
            for item in row['approved']:
                frame(item['stamp_ns'])['driver_valid_status_receipts'].append({
                    'monotonic_s': wall, 'source_index': i, 'sequence': row['sequence'],
                    'status_stamp_ns': row['stamp_ns'], 'gate_clock_ns': row['clock_ns']})
    for i, row in enumerate(runtime, 1):
        if row['event'] == 'frame_processed':
            name = row.get('frame_id', '')
            if not name.startswith('rgb-') or not name[4:].isdigit():
                raise ValueError('processed event lacks exact integer RGB frame stamp')
            frame(int(name[4:]))['runtime_processed'].append({'source_line': i, 'event': row})

    received, completed = {}, {}
    for i, row in enumerate(gate, 1):
        event = row['event']
        if row.get('stamp_ns', 0) > 0:
            frame(row['stamp_ns'])['gate_events'].append({'source_line': i, 'event': event,
                'monotonic_ns': row.get('monotonic_ns'), 'gate_ros_ns': row.get('ros_ns')})
        if event.startswith('received_'):
            kind = event[len('received_'):]
            received.setdefault((row['stamp_ns'], kind), []).append((i, row))
        elif event == 'callback_output_complete':
            completed.setdefault((row['stamp_ns'], row['kind']), []).append((i, row))
    for key, entries in received.items():
        approved = [(i, row) for i, row in entries if row.get('approved')]
        if not approved:
            continue
        ends = completed.get(key, [])
        unique = len(entries) == len(ends) == 1
        start_line, start = approved[0]
        ordered = unique and ends[0][0] > start_line and ends[0][1]['monotonic_ns'] >= start['monotonic_ns']
        bracket = {'callback_stamp_ns': key[0], 'kind': key[1],
            'state': 'observed' if ordered else ('ambiguous' if len(entries)>1 or len(ends)>1 else 'unknown'),
            'received_source_lines': [i for i, _ in entries],
            'complete_source_lines': [i for i, _ in ends],
            'duration_ns': ends[0][1]['monotonic_ns']-start['monotonic_ns'] if ordered else None,
            'start_monotonic_ns': start['monotonic_ns'] if ordered else None,
            'end_monotonic_ns': ends[0][1]['monotonic_ns'] if ordered else None,
            'meaning': 'output bracket, not pure logging cost'}
        for s in {a['stamp_ns'] for _, row in approved for a in row['approved']}:
            frame(s)['gate_output_brackets'].append(bracket)
    for row in frames.values():
        row['gate_output_bracket_state'] = 'present' if row['gate_output_brackets'] else 'unknown'
        row['driver_field_states'] = {k: field_states['observed-messages.json:' + k]
                                     for k in ('statuses', 'poses', 'tf')}
    checks = result.get('checks', [])
    warnings = [{'source_line': i, 'text': line, 'stamp_ns': None, 'cause': 'unknown'}
                for i, line in enumerate(log, 1) if 'Dropped acquisition after bounded wait for clock/TF' in line]
    for check in checks:
        if not isinstance(check, dict) or ('pass' in check and type(check['pass']) is not bool):
            raise ValueError('check must be an object with optional boolean pass')
    checks_known = field_states['result.json:checks'] == 'recorded'
    return {'schema': 2, 'sources': sources, 'field_states': field_states, 'limitations': LIMITS,
        'checks': {'state': 'recorded' if checks_known else 'unknown',
                   'pass_count': sum(c.get('pass') is True for c in checks) if checks_known else None,
                   'fail_count': sum(c.get('pass') is False for c in checks) if checks_known else None,
                   'measurement_items': [c for c in checks if 'pass' not in c] if checks_known else None},
        'warnings_unassociated': warnings, 'frames': [frames[s] for s in sorted(frames)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = audit(args.run)
    root = Path(__file__).resolve().parents[1]
    data['implementation'] = {'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
        'tool_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), 'version': 2}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as out:
        json.dump(data, out, indent=2, allow_nan=False)
        out.write('\n')


if __name__ == '__main__':
    main()
