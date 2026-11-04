#!/usr/bin/env python3
"""Replay fixed saved inputs through the production inference request function.

Diagnosis only: no ROS, Memory, navigation or replacement online observations.
Requires the existing AGX CUDA environment and an acquired AGX resource lock.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from host.inference_service import encode_frame, process_request
from perception.fpga_replay.core import Frame
from perception.fpga_replay.detector import SSDDetector


def matches_online(reply, online):
    """Compare the JSON wire representation, including acquisition identity."""
    reply = json.loads(json.dumps(reply, allow_nan=False))
    if reply['frame_id'] != online['frame_id'] or reply['timestamp'] != online['acquisition_timestamp']:
        return False
    if any(d['frame_id'] != reply['frame_id'] or d['timestamp'] != reply['timestamp']
           for d in reply['detections']):
        return False
    detections = [{key: d[key] for key in ('label', 'score', 'bbox')} for d in reply['detections']]
    return detections == online['detections']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if manifest['repeats'] != 2 or manifest['augmentation'] != 'hflip_shift4':
        raise ValueError('this protocol freezes two repeats and hflip_shift4')
    rows = {r['frame_id']: r for r in map(json.loads, (args.run/'runtime.jsonl').read_text().splitlines())
            if r['event'] == 'frame_processed'}
    inputs = []
    for item in manifest['frames']:
        row = rows[item['frame_id']]
        path = args.run/'frames'/row['evidence']['file']
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256'] or item['sha256'] != row['evidence']['sha256']:
            raise ValueError('saved input differs from frozen manifest')
        with np.load(path, allow_pickle=False) as saved:
            frame = Frame(item['frame_id'], float(saved['timestamp']), saved['rgb'], saved['depth'],
                          saved['K'], saved['T_world_camera'], True, 'reference', 'saved_online_input')
        frame.validate()
        inputs.append((item, row, frame))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    detector = SSDDetector(args.models, score=.4, device='cuda', memory_format='contiguous',
                           cuda_math='fp32', augmentation='hflip_shift4')
    detector.detector = 'ssdlite'
    results = []
    with (args.output/'results.jsonl').open('x') as stream:
        for repeat in range(2):
            for item, online, frame in inputs:
                reply = process_request(encode_frame(frame), detector, 'cuda')
                result = {'repeat': repeat, 'frame_id': frame.frame_id, 'sha256': item['sha256'],
                          'online': online['detections'], 'reply': reply,
                          'exact_detection_match': matches_online(reply, online)}
                stream.write(json.dumps(result, ensure_ascii=False)+'\n'); stream.flush()
                results.append(result)
    summary = {'source_revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
               'samples': len(inputs), 'repeats': 2, 'exact_matches': sum(x['exact_detection_match'] for x in results),
               'all_match': all(x['exact_detection_match'] for x in results),
               'scope': 'saved-input diagnostic; never new online confirmation'}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary))
    if not summary['all_match']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
