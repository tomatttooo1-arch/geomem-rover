#!/usr/bin/env python3
"""Prepare independent RTAB-Map input; never copy ground truth to SLAM.

Association follows TUM's sorted candidate, one-to-one nearest-time convention.
RGB timestamp names both copies as expected by rtabmap-rgbd_dataset.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def path_within(path, root):
    """``PurePath.is_relative_to`` for Python 3.8, the declared reference
    interpreter; the standard-library method only exists from Python 3.9.

    This is the CPython 3.9 implementation, so it keeps the exact semantics: a
    purely lexical parts-prefix test that touches no filesystem, does not resolve
    symlinks and does not collapse ``..``, and that answers ``False`` (rather than
    raising) for a relative/absolute mix. The caller below resolves both sides
    first, exactly as it must with the standard-library method.
    """
    try:
        Path(path).relative_to(root)
    except ValueError:
        return False
    return True


def rows(path):
    result = []
    for line in path.read_text().splitlines():
        if line.strip() and not line.lstrip().startswith('#'):
            stamp, name = line.split()
            result.append((float(stamp), stamp, name))
    if len({r[0] for r in result}) != len(result):
        raise ValueError('duplicate timestamp')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--mode', choices=['normal', 'blank'], default='normal')
    args = p.parse_args()
    rgb, depth = rows(args.source/'rgb.txt'), rows(args.source/'depth.txt')
    candidates = sorted((abs(r[0]-d[0]), i, j) for i, r in enumerate(rgb)
                        for j, d in enumerate(depth) if abs(r[0]-d[0]) < .02)
    used_rgb, used_depth, pairs = set(), set(), []
    for delta, i, j in candidates:
        if i not in used_rgb and j not in used_depth:
            used_rgb.add(i)
            used_depth.add(j)
            pairs.append((i, j, delta))
    if not pairs:
        raise ValueError('no associated frames')
    args.output.mkdir(parents=True, exist_ok=False)
    for folder in ['rgb_sync', 'depth_sync']:
        (args.output/folder).mkdir()
    manifest = []
    for i, j, delta in sorted(pairs):
        r, d = rgb[i], depth[j]
        entry = {'timestamp': r[1], 'depth_timestamp': d[1], 'delta_s': delta}
        for folder, row in [('rgb_sync', r), ('depth_sync', d)]:
            source = (args.source/row[2]).resolve()
            if not path_within(source, args.source.resolve()):
                raise ValueError('input path escapes dataset')
            target = args.output/folder/(r[1]+'.png')
            if args.mode == 'blank' and folder == 'rgb_sync':
                import cv2
                import numpy as np
                original = cv2.imread(str(source))
                if original is None or not cv2.imwrite(str(target), np.zeros_like(original)):
                    raise ValueError('cannot create blank image')
            else:
                shutil.copyfile(source, target)
            entry[folder] = {'source': str(source),
                'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                'input_sha256': hashlib.sha256(target.read_bytes()).hexdigest()}
        manifest.append(entry)
    (args.output/'INPUT.json').write_text(json.dumps({'mode': args.mode,
        'rgb_count': len(rgb), 'depth_count': len(depth), 'pairs': manifest,
        'groundtruth_available_to_slam': False}, indent=2)+'\n')
    print(json.dumps({'pairs': len(pairs), 'mode': args.mode, 'output': str(args.output)}))


if __name__ == '__main__':
    main()
