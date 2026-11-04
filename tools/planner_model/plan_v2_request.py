"""Plan from a .npy costmap and an explicitly synchronized JSON request. No ROS/TF guesses."""
import argparse
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from tools.planner_model.aligned_v2 import (InputContractError, MapMetadata, PoseSample,
                                           PlannerV2, checked_map)


def load_request(map_path, request_path):
    data = Path(map_path).read_bytes()
    request = json.loads(Path(request_path).read_text())
    digest = hashlib.sha256(data).hexdigest()
    if request.get('map_sha256') != digest:
        raise InputContractError('map_checksum_mismatch')
    if not data.startswith(b'\x93NUMPY'):
        raise InputContractError('costmap_must_be_npy_array')
    m = checked_map(np.load(io.BytesIO(data), allow_pickle=False))
    meta = MapMetadata(**request['metadata'])
    if request.get('pose') is None:
        raise InputContractError('missing_pose')
    pose = PoseSample(**request['pose'])
    goal = request['goal']
    return m, meta, pose, (goal['x'], goal['y']), goal['frame_id'], digest


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--map', required=True)
    ap.add_argument('--request', required=True)
    ap.add_argument('--lib', required=True)
    ap.add_argument('--out')
    ap.add_argument('--numeric', choices=['float', 'fixed'], default='fixed')
    ap.add_argument('--tolerance', type=float, default=.15)
    a = ap.parse_args()
    if a.out and Path(a.out).exists():
        ap.error('--out already exists; preserve previous outputs')
    try:
        m, meta, pose, goal, goal_frame, digest = load_request(a.map, a.request)
        result = PlannerV2(a.lib).plan(m, meta, pose, goal, goal_frame, a.numeric, a.tolerance)
    except (ValueError, TypeError, KeyError) as error:
        ap.error(str(error))
    result['input_map_sha256'] = digest
    content = json.dumps(result, default=serial, indent=2, allow_nan=False)+'\n'
    if a.out:
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(content)
    else:
        print(content, end='')
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
