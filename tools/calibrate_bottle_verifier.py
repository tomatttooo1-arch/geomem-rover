#!/usr/bin/env python3
"""Calibrate using an untouched passing run and isolated, labelled negative copies."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

from verify_bottle_run import read_rows, verify, SOURCE_EVIDENCE_PATHS


def write_json(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def write_rows(path, rows):
    # Only called on explicitly generated calibration copies.
    path.write_text(''.join(json.dumps(row, ensure_ascii=False) + '\n' for row in rows))


def calibrate(source, out):
    source, out = source.resolve(), out.resolve()
    if source == out or source in out.parents:
        raise ValueError('Calibration output must be outside the original run')
    positive = verify(source)
    if not positive['passed']:
        raise ValueError('Original run must pass unchanged; historical failed records cannot be repaired into a control')
    out.mkdir(parents=True, exist_ok=False)
    files = [Path(p) for p in ('agx/runtime.jsonl', 'agx/scenario.jsonl',
                              'agx/source-revision.txt', 'motion.jsonl', 'completed.json')]
    files += [Path(name) for name in ('run-config.json', 'source.tar', 'generated-room.sdf',
                                     'scene-provenance.json') if (source/name).is_file()]
    files += [Path('source')/name for name in SOURCE_EVIDENCE_PATHS
              if (source/'source'/name).is_file()]
    files += sorted(p.relative_to(source) for p in (source/'agx/frames').glob('*.npz'))
    manifest = {str(p): hashlib.sha256((source/p).read_bytes()).hexdigest() for p in files}
    results = {}
    for case in ('positive', 'old_frame', 'no_motion'):
        target = out/case
        for relative in files:
            destination = target/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source/relative, destination)
        changes = []
        if case == 'old_frame':
            scenario = read_rows(target/'agx/scenario.jsonl')
            runtime = read_rows(target/'agx/runtime.jsonl')
            initial = next(r['frame'] for r in scenario if r['event'] == 'remembered')
            final = next(r['frame'] for r in scenario if r['event'] == 'found_evidence')
            old_data = (target/'agx/frames'/initial['evidence']['file']).read_bytes()
            (target/'agx/frames'/final['evidence']['file']).write_bytes(old_data)
            digest = hashlib.sha256(old_data).hexdigest()
            for row in runtime:
                if row.get('frame_id') == final['frame_id']:
                    row['evidence']['sha256'] = digest
            final['evidence']['sha256'] = digest
            write_rows(target/'agx/runtime.jsonl', runtime)
            write_rows(target/'agx/scenario.jsonl', scenario)
            changes = ['Final NPZ replaced by initial NPZ; evidence hashes updated; new log timestamps retained']
        elif case == 'no_motion':
            motion = read_rows(target/'motion.jsonl')
            origin = next(r['message']['pose']['pose']['position'] for r in motion if r['topic'] == '/odom').copy()
            for row in motion:
                if row['topic'] == '/odom':
                    row['message']['pose']['pose']['position'] = origin.copy()
                    velocity = row['message']['twist']['twist']
                elif row['topic'] == '/fpga/cmd_vel_safe':
                    velocity = row['message']
                else:
                    continue
                for kind in ('linear', 'angular'):
                    for axis in ('x', 'y', 'z'):
                        velocity[kind][axis] = 0.0
            write_rows(target/'motion.jsonl', motion)
            changes = ['All server odometry positions fixed to origin; odometry velocities and protected commands set to zero']
        result = verify(target)
        write_json(target/'verification.json', result)
        write_json(target/'DERIVED_CONTROL.json', {'synthetic_control': case != 'positive',
                   'source_run': str(source), 'changes': changes, 'source_file_sha256': manifest})
        results[case] = result
    passed = (results['positive']['passed'] and not results['old_frame']['passed']
              and results['old_frame']['checks']['raw_acquisition_times_match'] is False
              and not results['no_motion']['passed']
              and results['no_motion']['checks']['simulator_actual_departure'] is False
              and results['no_motion']['checks']['simulator_departure_before_query'] is False)
    summary = {'passed': passed, 'source_run': str(source), 'source_file_sha256': manifest,
               'cases': {name: {'passed': result['passed'],
                               'failed_checks': [k for k, v in result['checks'].items() if not v]}
                         for name, result in results.items()},
               'scope': 'Verifier calibration; derived controls are not physical experiment runs'}
    write_json(out/'calibration.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    result = calibrate(args.source, args.out)
    print(json.dumps(result['cases'], ensure_ascii=False, indent=2))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
