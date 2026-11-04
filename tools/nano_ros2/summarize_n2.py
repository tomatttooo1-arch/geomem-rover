#!/usr/bin/env python3
"""Summarize one measure_rtabmap.sh run directory (copied back from the Nano).

Window for CPU/power = first 'pub' event to 'complete' event of the RTAB-Map run
(or, for detector-only runs, detector frames after warm-up). Idle = the 'idle'
phase at the start of the same run. Core-equivalents = CPU seconds / wall seconds.
Writes odom.tum (odometry output) next to the run; prints JSON.
"""
import json
import re
import statistics as stt
import sys
from pathlib import Path


def pct(xs, q):
    if not xs:
        return None
    xs = sorted(xs)
    k = min(len(xs) - 1, max(0, round(q / 100 * (len(xs) - 1))))
    return xs[k]


def trio(xs):
    return None if not xs else {'n': len(xs), 'median': round(stt.median(xs), 2), 'p95': round(pct(xs, 95), 2),
                                'max': round(max(xs), 2), 'mean': round(stt.mean(xs), 2)}


def load_jsonl(p):
    return [json.loads(l) for l in Path(p).read_text().splitlines() if l.strip()] if Path(p).exists() else []


def window_cpu(samples, t0, t1, ncpu, clk):
    inside = [s for s in samples if t0 <= s['wall'] <= t1]
    if len(inside) < 2:
        return None
    a, b = inside[0], inside[-1]
    dt = b['wall'] - a['wall']
    tot = lambda c: sum(c)
    idle = lambda c: c[3] + c[4]
    ca, cb = a['cpu']['cpu'], b['cpu']['cpu']
    busy_frac = 1 - (idle(cb) - idle(ca)) / (tot(cb) - tot(ca))
    per = {}
    ta = {(p['pid'], p['match']): p['ticks'] for p in a['procs']}
    for p in b['procs']:
        key = (p['pid'], p['match'])
        if key in ta:
            per.setdefault(p['match'], 0.)
            per[p['match']] += (p['ticks'] - ta[key]) / clk / dt
    rss = {}
    for s in inside:
        by = {}
        for p in s['procs']:
            by[p['match']] = by.get(p['match'], 0.) + p['rss_mb']
        for k, v in by.items():
            rss[k] = max(rss.get(k, 0.), v)
    return {'seconds': round(dt, 2), 'board_busy_cores': round(busy_frac * ncpu, 3),
            'proc_cores': {k: round(v, 3) for k, v in per.items()},
            'proc_rss_peak_mb': {k: round(v, 1) for k, v in rss.items()}}


def tegra(path, t0, t1, base_wall):
    """tegrastats lines have no epoch; map by line index at 0.5 s from the first line's time."""
    rows = []
    lines = Path(path).read_text().splitlines() if Path(path).exists() else []
    for i, line in enumerate(lines):
        m = re.search(r'VDD_IN (\d+)mW', line)
        g = re.search(r'GR3D_FREQ (\d+)%', line)
        ram = re.search(r'RAM (\d+)/(\d+)MB', line)
        tm = re.match(r'(\d\d)-(\d\d)-(\d{4}) (\d\d):(\d\d):(\d\d)', line)
        rows.append((line, m and int(m.group(1)), g and int(g.group(1)), ram and int(ram.group(1)), tm))
    return rows


def power(rows, t0, t1):
    import datetime as dtm
    out = []
    for line, mw, gpu, ram, tm in rows:
        if not (tm and mw):
            continue
        mo, d, y, H, M, S = map(int, tm.groups())
        t = dtm.datetime(y, mo, d, H, M, S, tzinfo=dtm.timezone(dtm.timedelta(hours=8))).timestamp()
        if t0 <= t <= t1:
            out.append((mw, gpu, ram))
    if not out:
        return None
    return {'n': len(out), 'vdd_in_mw_mean': round(stt.mean(o[0] for o in out)),
            'gpu_busy_pct_mean': round(stt.mean(o[1] or 0 for o in out), 1),
            'ram_used_mb_max': max(o[2] or 0 for o in out)}


def main(run):
    run = Path(run)
    phases = {}
    for line in (run / 'phases.txt').read_text().splitlines():
        t, name = line.split()
        phases[name] = float(t)
    procs = load_jsonl(run / 'procs.jsonl')
    head, samples = procs[0], [s for s in procs if s.get('event') == 's']
    ev = load_jsonl(run / 'slam' / 'events.jsonl')
    res = {'run': run.name}
    pubs = [e for e in ev if e['event'] == 'pub']
    if pubs:
        t0, t1 = pubs[0]['wall_ns'] / 1e9, [e for e in ev if e['event'] == 'complete'][0]['wall_ns'] / 1e9 - 5.0
        oi = [e for e in ev if e['event'] == 'odom_info']
        info = [e for e in ev if e['event'] == 'info']
        pub_stamps = {round(e['stamp'], 6) for e in pubs}
        res['frames_published'] = len(pubs)
        res['publisher_late_ms'] = trio([e['late_ms'] for e in pubs])
        res['publisher_decode_publish_ms'] = trio([e['decode_publish_ms'] for e in pubs])
        res['odom_frames'] = len(oi)
        res['odom_lost'] = sum(e['lost'] for e in oi)
        res['odom_processed_fraction'] = round(len(oi) / len(pubs), 4)
        res['odom_time_ms'] = trio([e['time_estimation'] * 1e3 for e in oi])
        res['odom_keyframes'] = sum(e['key_frame_added'] for e in oi)
        res['slam_updates'] = len(info)
        tot = [e['stats'].get('Timing/Total/ms') for e in info if 'Timing/Total/ms' in e['stats']]
        res['slam_total_ms'] = trio(tot)
        res['slam_loop_closures'] = sum(1 for e in info if e['loop_closure_id'] > 0)
        res['slam_proximity'] = sum(1 for e in info if e['proximity_detection_id'] > 0)
        wm = [e['stats'].get('Memory/Working_memory_size/') for e in info if 'Memory/Working_memory_size/' in e['stats']]
        res['slam_wm_size_last'] = wm[-1] if wm else None
        res['localization_pose_msgs'] = sum(1 for e in ev if e['event'] == 'localization_pose')
        res['window'] = window_cpu(samples, t0, t1, head['ncpu'], head['clk_tck'])
        odom = [e for e in ev if e['event'] == 'odom' and e['cov0'] < 9000]
        with open(run / 'odom.tum', 'w') as f:
            for e in odom:
                f.write('%.6f %s\n' % (e['stamp'], ' '.join('%.6f' % v for v in e['pose'])))
        loc = [e for e in ev if e['event'] == 'localization_pose']
        if loc:
            with open(run / 'localization.tum', 'w') as f:
                for e in loc:
                    f.write('%.6f %s\n' % (e['stamp'], ' '.join('%.6f' % v for v in e['pose'])))
    else:
        t0, t1 = phases.get('det_start', 0) + 10, phases.get('det_end', 0) - 3
        res['window'] = window_cpu(samples, t0, t1, head['ncpu'], head['clk_tck'])
    rows = tegra(run / 'tegrastats.log', 0, 0, 0)
    res['idle'] = window_cpu(samples, phases['idle_start'] + 2, phases['idle_end'], head['ncpu'], head['clk_tck'])
    res['power_window'] = power(rows, t0, t1)
    res['power_idle'] = power(rows, phases['idle_start'] + 2, phases['idle_end'])
    det = load_jsonl(run / 'det.jsonl')
    if det:
        dw = [d for d in det if not d.get('warmup') and t0 * 1e6 <= d['done_us'] <= t1 * 1e6]
        res['det_in_window'] = {'frames': len(dw), 'e2e_ms': trio([d['e2e_ms'] for d in dw]),
                                'proc_ms': trio([d['proc_ms'] for d in dw]),
                                'infer_ms': trio([d['infer_ms'] for d in dw]),
                                'late': sum(1 for d in dw if d.get('late')),
                                'skipped': sum(d.get('skipped', 0) for d in dw)}
    res['window_wall'] = [t0, t1]
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main(sys.argv[1])
