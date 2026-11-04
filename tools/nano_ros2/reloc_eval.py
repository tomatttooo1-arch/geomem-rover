#!/usr/bin/env python3
"""Score RTAB-Map localization-mode output against TUM ground truth in a fixed map frame.

1. Align the map's optimized poses to the mapping sequence's ground truth (SE3, no scale;
   tools/rtabmap_eval.py, TUM association 0.02 s). That alignment is then frozen.
2. Map each localization_pose (map frame) of the localization run through the frozen
   alignment and compare with the localization sequence's ground truth (nearest stamp < 0.02 s).
Updates are split by whether RTAB-Map reported a loop closure / proximity match to the old map.
Translation only. Usage: reloc_eval.py MAP_POSES MAP_GT LOC_EVENTS_JSONL LOC_GT
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rtabmap_eval import evaluate, read_tum  # noqa: E402


def main(map_poses, map_gt, events, loc_gt):
    align = evaluate(map_poses, map_gt)
    R, t = np.asarray(align['alignment_rotation']), np.asarray(align['alignment_translation_m'])
    gt = read_tum(loc_gt)
    ev = [json.loads(l) for l in Path(events).read_text().splitlines() if l.strip()]
    info = {round(e['stamp'], 6): e for e in ev if e['event'] == 'info'}
    rows = []
    for e in ev:
        if e['event'] != 'localization_pose':
            continue
        k = round(e['stamp'], 6)
        i = np.argmin(np.abs(gt[:, 0] - e['stamp']))
        if abs(gt[i, 0] - e['stamp']) >= 0.02:
            continue
        p = R @ np.asarray(e['pose'][:3]) + t
        inf = info.get(k, {})
        matched = inf.get('loop_closure_id', 0) > 0 or inf.get('proximity_detection_id', 0) > 0
        rows.append({'stamp': e['stamp'], 'err_m': float(np.linalg.norm(p - gt[i, 1:4])), 'matched_old_map': matched})
    def s(xs):
        xs = sorted(xs)
        return None if not xs else {'n': len(xs), 'median': round(xs[len(xs) // 2], 4),
                                    'p90': round(xs[int(0.9 * (len(xs) - 1))], 4), 'max': round(xs[-1], 4)}
    first = next((r for r in rows if r['matched_old_map']), None)
    out = {'map_alignment_ate_rmse_m': align['ate_translation_rmse_m'], 'map_matched_samples': align['matched_samples'],
           'loc_updates_scored': len(rows),
           'loc_updates_with_old_map_match': sum(r['matched_old_map'] for r in rows),
           'err_with_match_m': s([r['err_m'] for r in rows if r['matched_old_map']]),
           'err_without_match_m': s([r['err_m'] for r in rows if not r['matched_old_map']]),
           'first_match_after_s': None if not first else round(first['stamp'] - rows[0]['stamp'], 2),
           'rows': rows}
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main(*sys.argv[1:5])
