"""Build the CTRLPS1 tracker re-registration case set from dev2's car1015 hindsight replay (read-only).

For every car1015 request where BOTH v3 tiers succeeded (dev2 comparison: 144), write
  OUT/NNN/map.u8      80x80 body-seeded costmap used by dev2 for that request (maps[frame])
  OUT/NNN/request.txt sx sy gx gy        (start cell, continuous goal; NavFn-round cell coordinates)
  OUT/NNN/path1.f64le tier-1 path (dev2 v3_1, actual start prepended by dev2), x,y interleaved, cells
  OUT/NNN/path2.f64le tier-2 path (dev2 v3_2)
  OUT/NNN/meta.json   frame, source ids, sha256 of the inputs
Exact duplicates (same map bytes, start, goal: the car standing still) are written once; the
duplicate count is kept in manifest.json.  Usage:
  python3 prep_car1015.py HINDSIGHT_RUN_DIR OUT_DIR
HINDSIGHT_RUN_DIR = ${FPGA_SRC}/runs/hindsight-20261103
"""
import hashlib
import json
import os
import sys

import numpy as np


def main():
    root, out = sys.argv[1], sys.argv[2]
    if os.path.exists(out):
        sys.exit('OUT exists; choose a fresh directory')
    os.makedirs(out)
    npz_path = os.path.join(root, 'car1015', 'car1015.npz')
    maps = np.load(npz_path)['maps']
    pairs = [json.loads(l) for l in open(os.path.join(root, 'comparison', 'car1015_pairs.jsonl'))]
    paths = {json.loads(l)['frame']: json.loads(l) for l in open(os.path.join(root, 'comparison', 'car1015_paths.jsonl'))}
    seen, rows, dup = {}, [], 0
    for pr in pairs:
        if not (pr['v3_1']['success'] and pr['v3_2']['success']):
            continue
        f = pr['frame']
        m = np.ascontiguousarray(maps[f], np.uint8)
        key = (hashlib.sha256(m.tobytes()).hexdigest(), tuple(pr['start_cell']), tuple(pr['goal']))
        if key in seen:
            dup += 1
            seen[key]['duplicates'].append(f)
            continue
        name = '%03d' % len(rows)
        d = os.path.join(out, name)
        os.makedirs(d)
        m.tofile(os.path.join(d, 'map.u8'))
        sx, sy = pr['start_cell']
        gx, gy = pr['goal']
        with open(os.path.join(d, 'request.txt'), 'w') as fh:
            fh.write('%d %d %.17g %.17g\n' % (sx, sy, gx, gy))
        p1 = np.asarray(paths[f]['v3_1'], np.float64)
        p2 = np.asarray(paths[f]['v3_2'], np.float64)
        p1.tofile(os.path.join(d, 'path1.f64le'))
        p2.tofile(os.path.join(d, 'path2.f64le'))
        meta = dict(case=name, frame=f, source_id=pr['source_id'], start_cell=pr['start_cell'],
                    start_world=pr['start_world'], goal=pr['goal'], goal_kind=pr['goal_kind'],
                    len1_m=pr['v3_1']['length'], len2_m=pr['v3_2']['length'], map_sha256=key[0],
                    duplicates=[])
        seen[key] = meta
        rows.append(meta)
    for meta in rows:
        json.dump(meta, open(os.path.join(out, meta['case'], 'meta.json'), 'w'), indent=1)
    src = dict(npz=npz_path, npz_sha256=hashlib.sha256(open(npz_path, 'rb').read()).hexdigest(),
               pairs_sha256=hashlib.sha256(open(os.path.join(root, 'comparison', 'car1015_pairs.jsonl'), 'rb').read()).hexdigest(),
               paths_sha256=hashlib.sha256(open(os.path.join(root, 'comparison', 'car1015_paths.jsonl'), 'rb').read()).hexdigest())
    json.dump(dict(source=src, both_tiers_success=len(rows) + dup, unique_cases=len(rows), exact_duplicates=dup),
              open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    print('cases', len(rows), 'duplicates', dup)


if __name__ == '__main__':
    main()
