"""Audit the 55 baseline failures and test a safe NavFn-style goal-tolerance proposal.

Research wrapper only; leaves model.cpp, original maps and registered gate results unchanged.
The flood fill is an independent CPU reachability oracle. A future HLS version would use
one start-seeded converged wave field; this is not an HLS implementation or a new acceptance run.
"""
import argparse
from collections import Counter, deque
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from tools.planner_model.evaluate import Geometry, Planner


def reachable(m, start):
    h, w = m.shape
    seen = np.zeros((h, w), bool)
    def free(x, y):
        return 0 < x < w-1 and 0 < y < h-1 and m[y, x] < 253
    sx, sy = start
    if not free(sx, sy):
        return seen
    seen[sy, sx] = True
    todo = deque([start])
    while todo:
        x, y = todo.popleft()
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if not dx and not dy:
                    continue
                a, b = x+dx, y+dy
                if not free(a, b) or seen[b, a]:
                    continue
                if dx and dy and (not free(a, y) or not free(x, b)):
                    continue
                seen[b, a] = True
                todo.append((a, b))
    return seen


def safe_tolerant_plan(planner, m, start, goal):
    """Exact goal first; else nearest reachable cell in +/-0.15m square, with safety gate."""
    h, w = m.shape
    sx, sy = start
    gx, gy = goal
    empty = np.empty((0, 2))
    if not (0 <= sx < w and 0 <= sy < h) or m[sy, sx] >= 253:
        return empty, 'invalid_start', None
    if not (0 <= gx < w and 0 <= gy < h) or m[gy, gx] >= 253:
        return empty, 'invalid_goal', None
    seen = reachable(m, start)
    target = goal
    if not seen[gy, gx]:
        candidates = [(int((x-gx)**2+(y-gy)**2), y, x)
                      for y in range(max(0, gy-3), min(h, gy+4))
                      for x in range(max(0, gx-3), min(w, gx+4)) if seen[y, x]]
        if not candidates:
            return empty, 'no_reachable_goal_in_tolerance', None
        _, y, x = min(candidates)
        target = (x, y)
    # Existing C++ wavefront builds from its goal. Swap ends and reverse output
    # to prototype the proposed FROM-robot potential direction without changing the kernel.
    reverse, _ = planner.run(m, target, start, 'fixed')
    path = reverse[::-1].copy()
    if not len(path) or Geometry(m).collisions(path):
        return empty, 'rejected_by_continuous_safety_check', target
    return path, 'exact' if target == goal else 'within_tolerance', target


def fixtures(planner):
    m = np.zeros((40, 40), np.uint8)
    wall = m.copy(); wall[:, 20] = 254
    tests = [('door_near_target_a', wall, (5, 13), (21, 13), True),
             ('door_near_target_b', wall, (5, 25), (21, 25), True),
             ('reachable_exact_goal', m, (5, 13), (25, 13), True),
             ('beyond_tolerance', wall, (5, 13), (30, 13), False),
             ('lethal_goal', wall, (5, 13), (20, 13), False)]
    unknown = wall.copy();unknown[13, 21] = 255
    tests.append(('unknown_goal', unknown, (5, 13), (21, 13), False))
    blocked = m.copy();blocked[13, 5] = 253
    tests.append(('blocked_start', blocked, (5, 13), (25, 13), False))
    results = []
    for name, grid, start, goal, expected in tests:
        strict, _ = planner.run(grid, start, goal, 'fixed')
        nav, _ = planner.run(grid, start, goal, 'navfn')
        path, reason, resolved = safe_tolerant_plan(planner, grid, start, goal)
        assert bool(len(path)) == expected, (name, reason)
        assert not Geometry(grid).collisions(path), name
        if len(path):
            assert np.array_equal(path[0], start)
            assert max(abs(path[-1]-goal)) <= 3
            if name == 'reachable_exact_goal':
                assert np.array_equal(path[-1], goal)
        if name.startswith('door_near_target'):
            assert not len(strict) and len(nav) and reason == 'within_tolerance'
        results.append(dict(name=name, strict_success=bool(len(strict)), navfn_success=bool(len(nav)),
                            candidate_success=bool(len(path)), reason=reason, requested_goal=goal,
                            resolved_goal=resolved, path=path.tolist(), safety_hits=[]))
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    run = Path(a.run); out = Path(a.out)
    if out.exists():
        ap.error('choose a new --out file; preserve previous results')
    p = Planner(run/'build/libplanner_model.so')
    rows = [json.loads(line) for line in (run/'results/pairs.jsonl').read_text().splitlines()]
    base = [r for r in rows if 100 < r['frame'] <= 376 and r['navfn_success']]
    failed = [r for r in base if not r['float_success']]
    valid = [r for r in base if r['start_cost'] < 253]
    assert len(failed) == 55 and all(r['start_cost'] == 253 for r in failed)
    assert all(r['float_success'] for r in valid)
    maps = np.load(run/'input/maps.npz')['maps']
    unsafe_shortcut = Counter()
    for r in failed:
        grid = maps[r['frame']-1]
        unchanged, why, _ = safe_tolerant_plan(p, grid, tuple(r['start']), tuple(r['goal']))
        assert not len(unchanged) and why == 'invalid_start'
        cleared = grid.copy();cleared[r['start'][1], r['start'][0]] = 0
        path, _ = p.run(cleared, r['start'], r['goal'], 'fixed')
        if len(path):
            unsafe_shortcut['returned_path'] += 1
            unsafe_shortcut['unsafe_on_original_map'] += bool(Geometry(grid).collisions(path))
    result = dict(code_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                  baseline_code_commit=(run/'CODE_COMMIT.txt').read_text().strip(),
                  baseline_summary_sha256=hashlib.sha256((run/'results/summary.json').read_bytes()).hexdigest(),
                  audit=dict(navfn_success=len(base), candidate_failed=len(failed),
                             failure_start_costs=dict(Counter(r['start_cost'] for r in failed)),
                             failure_sources=dict(Counter(r['source'] for r in failed)),
                             all_failed_navfn_paths_hit_original_obstacles=all(r['navfn_hits'] for r in failed),
                             valid_start_pairs=len(valid), valid_start_success=sum(r['float_success'] for r in valid),
                             maximum_safe_success_with_unchanged_inputs=len(valid)/len(base),
                             unsafe_start_clearing_diagnostic=dict(unsafe_shortcut)),
                  failed_pairs=[dict(frame=r['frame'], pair=r['pair'], start=r['start'], goal=r['goal'],
                                     start_cost=r['start_cost'], navfn_hits=r['navfn_hits']) for r in failed],
                  synthetic_controls=fixtures(p))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result['audit'], indent=2))
    print('Synthetic controls: 7/7 PASS; two specifically constructed tolerance cases 0/2 -> 2/2')
    print('This is a feasibility probe; no new real-data success-rate acceptance claim.')


if __name__ == '__main__':
    main()
