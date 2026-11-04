#!/usr/bin/env python3
"""Summarize frozen offline probe artifacts; never infer tracking rate from nodes."""
import argparse
import json
import re
import sqlite3
import struct
from pathlib import Path


def summarize(root):
    out = {'scope': 'TUM freiburg1_desk, CPU offline; graph-node translation only'}
    for mode in ('normal', 'blank'):
        run = root/f'{mode}-result'
        log = (run/'algorithm.log').read_text()
        iterations = re.findall(r'Iteration (\d+)/(\d+):', log)
        database = sqlite3.connect(f'file:{run / "rtabmap.db"}?mode=ro', uri=True)
        checks = database.execute('pragma integrity_check').fetchall()
        truth_blobs = [r[0] for r in database.execute('select ground_truth_pose from Node') if r[0]]
        out[mode] = {
            'exit_code': int((run/'exit-code.txt').read_text()),
            'processed_iterations': len(iterations),
            'iteration_sequence_complete': [int(x[0]) for x in iterations] == list(range(1, 574)),
            'database_integrity_ok': checks == [('ok',)],
            'nodes': database.execute('select count(*) from Node').fetchone()[0],
            'directed_links_by_type': dict(database.execute('select type,count(*) from Link group by type')),
            'ground_truth_blobs': database.execute('select count(*) from Node where length(ground_truth_pose)>0').fetchone()[0],
            'ground_truth_all_zero_placeholders': sum(len(b) == 48 and not any(struct.unpack('<12f', b)) for b in truth_blobs),
            'reset_warnings': log.count('Odometry is reset'),
            'initialization_feature_failure_warnings': log.count('visual features required to initialize'),
            'loop_marked_iterations': len(re.findall(r'^Iteration .* \*$', log, re.M)),
            'continuous_tracking_validity': 'not directly recorded; do not infer from node count or warnings',
        }
        database.close()
    for name in ('optimized', 'odometry'):
        out[name] = json.loads((root/f'{name}-evaluation.json').read_text())
    out['interpretation'] = {
        'normal_database_and_export': 'functional output observed; geometry accuracy not independently scored',
        'blank_zero_nodes': 'extreme visual failure observed even though process exit code is zero',
        'localization_after_restart': 'not tested; reopening database for export is not relocalization',
        'loop_correctness': 'not yet independently validated',
    }
    return out


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('root', type=Path)
    args = p.parse_args()
    print(json.dumps(summarize(args.root.resolve()), indent=2))
