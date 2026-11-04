#!/usr/bin/env python3
"""Offline cross-check of two wheel-odometry records (chassis serial counts vs PL encoder), 2026-10-14.

  cross_check.py A.jsonl B.jsonl [--wheel-diam-m ... --counts-per-rev ... --track-m ...] [--out R.json]

Each input is either a host/ros_wheel_odom.py --record file (rows with 'source', 't_ns', 'left', 'right')
or a host/ros_chassis_driver.py --record file (rows with dir 'rx', 't_ns', 'left_count', 'right_count').
Both are re-run through host/wheel_odom.py with ONE parameter set (so a difference is a difference in the
counts, not in the conversion), on the Nano clock t_ns. Over the time window both records cover, each
source's centre path length and yaw are taken relative to the window start and compared at every sample
time of the sparser record (the denser one linearly interpolated in time: exact for constant wheel
speeds, since both quantities are linear in the counts). Reported: max / median / end absolute
difference of path length (m) and yaw (rad), the relative end difference of path length, and per wheel
the count change over the window. When a record carries the PL's own fixed-point values (pl_dist_*_m,
pl_heading_rad), their deviation from the float recomputation is reported too (information only).
Prints one JSON object; exit 0 unless an input is unusable.
"""
import argparse
import bisect
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.wheel_odom import WheelOdometry, WheelParams  # noqa: E402


def load_samples(path):
    """[(t_ns, left, right, rebaseline_reason or None, pl_extra or None)] in file order."""
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if 'left' in row and 'right' in row and 't_ns' in row:
                reason = row.get('reason') if row.get('event') == 'rebaseline' else None
                extra = {k: row[k] for k in ('pl_dist_left_m', 'pl_dist_right_m', 'pl_dist_center_m',
                                             'pl_heading_rad') if row.get(k) is not None} or None
                out.append((int(row['t_ns']), int(row['left']), int(row['right']), reason, extra))
            elif row.get('dir') == 'rx' and 'left_count' in row:
                reason = 'chassis status after silence' if row.get('after_silence') else None
                out.append((int(row['t_ns']), int(row['left_count']), int(row['right_count']), reason, None))
    if len(out) < 2:
        raise ValueError('%s: fewer than 2 count samples' % path)
    return out


def replay(samples, params):
    """Series of (t_ns, distance, yaw, left_total, right_total) through the shared module."""
    odom = WheelOdometry(params)
    series, pl_dev = [], []
    tot_l = tot_r = 0
    pl0 = None
    for t, left, right, reason, extra in samples:
        if reason:
            odom.rebaseline(reason)
        st = odom.update(left, right, t)
        if st['event'] in ('ok',) and 'delta_counts' in st:
            tot_l += st['delta_counts'][0]
            tot_r += st['delta_counts'][1]
        if st['event'] == 'duplicate':
            continue
        series.append((t, st['distance'], st['yaw'], tot_l, tot_r))
        if extra and 'pl_dist_center_m' in extra and 'pl_heading_rad' in extra:
            if pl0 is None:
                pl0 = (extra['pl_dist_center_m'], extra['pl_heading_rad'], st['distance'], st['yaw'])
            pl_dev.append((abs((extra['pl_dist_center_m'] - pl0[0]) - (st['distance'] - pl0[2])),
                           abs((extra['pl_heading_rad'] - pl0[1]) - (st['yaw'] - pl0[3]))))
    return series, odom.counters, pl_dev


def interp(series, t, idx):
    times = [s[0] for s in series]
    i = bisect.bisect_left(times, t)
    if i < len(series) and times[i] == t:
        return series[i][idx]
    a, b = series[i - 1], series[i]
    f = (t - a[0]) / float(b[0] - a[0])
    return a[idx] + f * (b[idx] - a[idx])


def median(values):
    v = sorted(values)
    n = len(v)
    return None if not n else (v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2.)


def compare(samples_a, samples_b, params):
    sa, ca, pa = replay(samples_a, params)
    sb, cb, pb = replay(samples_b, params)
    t0 = max(sa[0][0], sb[0][0])
    t1 = min(sa[-1][0], sb[-1][0])
    if t1 <= t0:
        raise ValueError('the two records do not overlap in time')
    sparse, dense = (sa, sb) if len(sa) <= len(sb) else (sb, sa)
    a_is_sparse = sparse is sa
    base = {id(s): (interp(s, t0, 1), interp(s, t0, 2), interp(s, t0, 3), interp(s, t0, 4)) for s in (sa, sb)}
    dd, dy = [], []
    for t, *_ in sparse:
        if t < t0 or t > t1:
            continue
        rel = []
        for s in (sparse, dense):
            b = base[id(s)]
            rel.append((interp(s, t, 1) - b[0], interp(s, t, 2) - b[1]))
        dd.append(abs(rel[0][0] - rel[1][0]))
        dy.append(abs(rel[0][1] - rel[1][1]))

    def end(s):
        b = base[id(s)]
        return (interp(s, t1, 1) - b[0], interp(s, t1, 2) - b[1], interp(s, t1, 3) - b[2],
                interp(s, t1, 4) - b[3])
    ea, eb = end(sa), end(sb)
    rel_end = None if ea[0] == 0 else (eb[0] - ea[0]) / ea[0]
    report = {'params': params.describe(), 'window_s': (t1 - t0) / 1e9, 'compared_points': len(dd),
              'sparse_is': 'A' if a_is_sparse else 'B',
              'distance_abs_diff_m': {'max': max(dd) if dd else None, 'median': median(dd),
                                      'end': abs(ea[0] - eb[0])},
              'yaw_abs_diff_rad': {'max': max(dy) if dy else None, 'median': median(dy), 'end': abs(ea[1] - eb[1])},
              'distance_rel_end_diff_b_minus_a': rel_end,
              'A': {'end_distance_m': ea[0], 'end_yaw_rad': ea[1], 'counts_left': ea[2], 'counts_right': ea[3],
                    'odometry': ca},
              'B': {'end_distance_m': eb[0], 'end_yaw_rad': eb[1], 'counts_left': eb[2], 'counts_right': eb[3],
                    'odometry': cb}}
    for name, dev in (('A', pa), ('B', pb)):
        if dev:
            report[name]['pl_fixed_point_vs_float'] = {'distance_max_m': max(d for d, _ in dev),
                                                       'yaw_max_rad': max(y for _, y in dev)}
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('a')
    ap.add_argument('b')
    ap.add_argument('--wheel-diam-m', type=float, default=WheelParams().wheel_diam_m)
    ap.add_argument('--counts-per-rev', type=int, default=WheelParams().counts_per_rev)
    ap.add_argument('--track-m', type=float, default=WheelParams().track_m)
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    params = WheelParams(a.wheel_diam_m, a.counts_per_rev, a.track_m)
    try:
        report = compare(load_samples(a.a), load_samples(a.b), params)
    except (OSError, ValueError) as error:
        print(json.dumps({'error': str(error)}))
        return 2
    report['inputs'] = {'A': a.a, 'B': a.b}
    text = json.dumps(report, indent=1)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(text + '\n')
    print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
