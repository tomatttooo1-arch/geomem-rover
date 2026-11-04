#!/usr/bin/env python3
"""Sample whole-board CPU (/proc/stat) and per-process CPU/RSS on the Nano host.

Processes are chosen by substring of their command line (container processes are
visible on the host). One JSON line per sample; stops when --stop-file appears or
after --seconds. Run on the host, not inside a container.
Usage: proc_sampler.py --out s.jsonl --match rgbd_odometry --match rtabmap_slam ...
"""
import argparse
import json
import os
import time


def cpu_times():
    rows = {}
    with open('/proc/stat') as f:
        for line in f:
            if line.startswith('cpu'):
                parts = line.split()
                rows[parts[0]] = [int(x) for x in parts[1:9]]
    return rows


def proc_rows(patterns):
    out = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
            hit = [p for p in patterns if p in cmd]
            if not hit or 'proc_sampler.py' in cmd:
                continue
            with open(f'/proc/{pid}/stat') as f:
                st = f.read()
            fields = st[st.rfind(')') + 2:].split()
            ut, stt = int(fields[11]), int(fields[12])
            rss_pages = int(fields[21])
            out.append({'pid': int(pid), 'match': hit[0], 'ticks': ut + stt,
                        'rss_mb': rss_pages * os.sysconf('SC_PAGE_SIZE') / 2**20,
                        'cpu': int(fields[36])})
        except (OSError, ValueError, IndexError):
            continue
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', required=True)
    p.add_argument('--match', action='append', default=[])
    p.add_argument('--interval', type=float, default=0.5)
    p.add_argument('--seconds', type=float, default=3600.)
    p.add_argument('--stop-file')
    a = p.parse_args()
    end = time.monotonic() + a.seconds
    with open(a.out, 'x', buffering=1) as log:
        log.write(json.dumps({'event': 'start', 'clk_tck': os.sysconf('SC_CLK_TCK'),
                              'ncpu': os.cpu_count(), 'match': a.match}) + '\n')
        nxt = time.monotonic()
        while time.monotonic() < end and not (a.stop_file and os.path.exists(a.stop_file)):
            log.write(json.dumps({'event': 's', 'mono': time.monotonic(), 'wall': time.time(),
                                  'cpu': cpu_times(), 'procs': proc_rows(a.match)}) + '\n')
            nxt += a.interval
            time.sleep(max(0., nxt - time.monotonic()))


if __name__ == '__main__':
    main()
