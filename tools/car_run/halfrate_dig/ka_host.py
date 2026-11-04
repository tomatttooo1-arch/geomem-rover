#!/usr/bin/env python3
"""Known answer of the half-rate rule (10-17, COLOUR_RATE_NAVMODE section 7.2) on every host drv_frames.csv (md5-deduplicated):
global timestamps (= the header stamps the probe reads), window = 3 s after the 30th depth frame (as the probe), the
verdict of depth_probe.classify against the whole-run signature (half 32.3/64.6, normal 33.4/33.4, other)."""
import csv, glob, hashlib, os, statistics, sys, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
from tools.car_run.depth_probe import classify, intervals_ms
root = sys.argv[1] if len(sys.argv) > 1 else '${FPGA_SRC}/runs'
seen = {}
for p in sorted(glob.glob(root + '/**/drv_frames*.csv', recursive=True)):
    h = hashlib.md5(open(p, 'rb').read()).hexdigest()
    if h in seen: continue
    seen[h] = p
rows = []
for h, p in seen.items():
    d = []; c = []
    with open(p) as f:
        for r in csv.DictReader(f):
            for k, lst in (('depth', d), ('color', c)):
                v = r.get(k + '_global_ts_sec')
                if v: lst.append(float(v))
    d.sort(); c.sort()
    if len(d) < 50:
        rows.append((p, 'short', len(d), len(c), None)); continue
    full = (statistics.median(intervals_ms(d)), statistics.median(intervals_ms(c)) if len(c) > 2 else None)
    t30 = d[29]
    for w in (3.0,):
        dw = [x for x in d if t30 < x <= t30 + w]; cw = [x for x in c if t30 < x <= t30 + w]
        v = classify(intervals_ms(dw), intervals_ms(cw))
    whole = 'half' if 32.0 < full[0] < 32.6 and full[1] and 64 < full[1] < 65.2 else ('normal' if 33.2 < full[0] < 33.6 and full[1] and 33.2 < full[1] < 33.6 else 'other')
    rows.append((p.replace(root + '/', ''), whole, round(full[0], 2), round(full[1], 2) if full[1] else None, v))
from collections import Counter
cnt = Counter()
for p, whole, a, b, v in rows:
    if whole == 'short': cnt[('short',)] += 1; continue
    cnt[(whole, v['verdict'])] += 1
    if not ((whole == 'half' and v['verdict'] == 'half') or (whole == 'normal' and v['verdict'] == 'normal')):
        print(whole, a, b, v['verdict'], v['depth_median_ms'], v['colour_p25_ms'], v['colour_median_ms'], v['depth_intervals'], v['colour_intervals'], p)
print(cnt)
# margins
for wh in ('half', 'normal'):
    dm = [v['depth_median_ms'] for p, w, a, b, v in rows if w == wh]
    cp = [v['colour_p25_ms'] for p, w, a, b, v in rows if w == wh and v['colour_p25_ms']]
    print(wh, 'depth median range', min(dm), max(dm), 'colour p25 range', min(cp), max(cp))
