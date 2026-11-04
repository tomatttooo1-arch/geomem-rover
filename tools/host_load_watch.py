#!/usr/bin/env python3
"""Host load during a whole replay arm: sample, then judge the arm valid or void (定位门第 11 棒, owner 10-17 12:31).

owner: the host load gate is no longer only checked once before an arm starts; the load is sampled for the whole run
and an arm during which it went over the gate is void (not counted, rerun). Only for arms from 第 11 棒 on (g8r_3 is
not judged again). The limits and the quantities are the existing start gate's (gate_dig/host_g8/g8_go.sh busy()):
  load:  /proc/loadavg field 1 (1-minute load average); busy when > 24      (awk '$1<=24{exit 1}{exit 0}')
  io:    /proc/pressure/io line "full", avg10 (%);       busy when >= 30    (exit (a[2]<30)?1:0)
The start gate's container / car / gzserver checks stay start checks (the arm's own container exists while it runs).

  host_load_watch.py watch --out FILE.jsonl [--interval 2] [--container NAME ...] [--pid PID]
      one JSON row per sample: wall (s), load1/5/15, runnable/threads (/proc/loadavg field 4), io_full_avg10 and,
      per --container (readout only, not judged), `own`: threads of that container in state R or D (cgroup v1
      cpuacct tasks; null until the container exists). Ends on SIGTERM / SIGINT, or when --pid has exited.
  host_load_watch.py verdict --log FILE.jsonl --start WALL --end WALL [--max-load 24] [--max-io 30] [--max-gap-s 6]
      the arm window [start, end] (wall seconds): void when any sample in it has load1 > max-load or io_full_avg10 >=
      max-io; also void ("blind") when the samples do not cover the window: first sample later than start + max-gap,
      last earlier than end - max-gap, or two samples in it more than max-gap apart (the kernel updates loadavg every
      5 s, PSI avg10 every 2 s; the default interval 2 s). Prints JSON; exit 0 valid, 1 void, 2 usage / no log.
Python 3.8, standard library only.
"""
import argparse
import glob
import json
import os
import signal
import subprocess
import sys
import time

MAX_LOAD = 24.0      # g8_go.sh: busy when 1-minute load > 24 (48 cores, half; HOST_CORRECTION_PREREG 10-15 19:46)
MAX_IO = 30.0        # g8_go.sh: busy when io full avg10 >= 30 % (10-15 23:20)
INTERVAL_S = 2.0
MAX_GAP_S = 6.0
EXIT_VALID, EXIT_VOID, EXIT_USAGE = 0, 1, 2


def parse_loadavg(text):
    """'/proc/loadavg' text -> dict load1, load5, load15, runnable, threads."""
    f = text.split()
    run, tot = f[3].split('/')
    return {'load1': float(f[0]), 'load5': float(f[1]), 'load15': float(f[2]),
            'runnable': int(run), 'threads': int(tot)}


def parse_psi_full_avg10(text):
    """'/proc/pressure/io' text -> the 'full' line's avg10 (float); None when there is no full line."""
    for line in text.splitlines():
        f = line.split()
        if f and f[0] == 'full':
            for kv in f[1:]:
                k, _, v = kv.partition('=')
                if k == 'avg10':
                    return float(v)
    return None


def busy(sample, max_load=MAX_LOAD, max_io=MAX_IO):
    """Reasons the sample is over the start gate's limits (empty list = within)."""
    why = []
    if sample.get('load1') is None:
        why.append('load_unreadable')
    elif sample['load1'] > max_load:
        why.append('load')
    io = sample.get('io_full_avg10')
    if io is None:
        why.append('io_unreadable')
    elif io >= max_io:
        why.append('io')
    return why


def verdict(samples, start, end, max_load=MAX_LOAD, max_io=MAX_IO, max_gap_s=MAX_GAP_S):
    """samples: dicts with wall (s) and the parsed fields. -> dict with verdict 'valid' | 'void' and why."""
    win = sorted((s for s in samples if start <= s['wall'] <= end), key=lambda s: s['wall'])
    out = {'start': start, 'end': end, 'window_s': round(end - start, 3), 'n': len(win),
           'max_load': max_load, 'max_io': max_io, 'max_gap_s': max_gap_s, 'reasons': []}
    if end < start:
        out['reasons'].append('window_end_before_start')
    if not win:
        out['reasons'].append('blind:no_samples')
    else:
        gaps = [b['wall'] - a['wall'] for a, b in zip(win, win[1:])]
        out['coverage'] = {'first_after_start_s': round(win[0]['wall'] - start, 3),
                           'last_before_end_s': round(end - win[-1]['wall'], 3),
                           'max_gap_s': round(max(gaps), 3) if gaps else None}
        if win[0]['wall'] - start > max_gap_s:
            out['reasons'].append('blind:late_first_sample')
        if end - win[-1]['wall'] > max_gap_s:
            out['reasons'].append('blind:early_last_sample')
        if gaps and max(gaps) > max_gap_s:
            out['reasons'].append('blind:gap')
        over = [(s, busy(s, max_load, max_io)) for s in win]
        over = [(s, w) for s, w in over if w]
        out['over_samples'] = len(over)
        if over:
            out['first_over'] = {'wall': over[0][0]['wall'], 'since_start_s': round(over[0][0]['wall'] - start, 3),
                                 'why': over[0][1], 'load1': over[0][0].get('load1'),
                                 'io_full_avg10': over[0][0].get('io_full_avg10')}
            for r in sorted({w for _, ws in over for w in ws}):
                out['reasons'].append('over:' + r)
        loads = [s['load1'] for s in win if s.get('load1') is not None]
        ios = [s['io_full_avg10'] for s in win if s.get('io_full_avg10') is not None]
        out['load1_max'] = max(loads) if loads else None
        out['load1_min'] = min(loads) if loads else None
        out['io_full_avg10_max'] = max(ios) if ios else None
        owns = [sum(v for v in s['own'].values() if v is not None) for s in win
                if s.get('own') and any(v is not None for v in s['own'].values())]
        out['own_rd_threads_mean_max'] = [round(sum(owns) / len(owns), 2), max(owns)] if owns else None
    out['verdict'] = 'void' if out['reasons'] else 'valid'
    return out


def read_log(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    pass   # a torn last line when the watcher was stopped mid-write
    return [r for r in rows if r.get('event', 'sample') == 'sample']


# ---------------------------------------------------------------- sampling (host side)
def container_tasks_file(name, cgroot='/sys/fs/cgroup/cpu,cpuacct'):
    """cgroup v1 tasks file of a running docker container (None when it does not run / is not found)."""
    try:
        cid = subprocess.run(['docker', 'inspect', '-f', '{{.Id}}', name], stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, timeout=5, universal_newlines=True).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None
    if not cid:
        return None
    for pat in ('%s/docker/%s/tasks' % (cgroot, cid), '%s/system.slice/docker-%s.scope/tasks' % (cgroot, cid)):
        hit = glob.glob(pat)
        if hit:
            return hit[0]
    return None


def count_rd(tasks_file):
    """Threads in state R or D listed in a cgroup tasks file (None when unreadable)."""
    try:
        with open(tasks_file) as f:
            tids = f.read().split()
    except OSError:
        return None
    n = 0
    for tid in tids:
        try:
            with open('/proc/%s/stat' % tid) as f:
                st = f.read()
        except OSError:
            continue
        state = st[st.rfind(')') + 2:st.rfind(')') + 3]
        n += state in ('R', 'D')
    return n


def sample(containers, tasks, read=lambda p: open(p).read()):
    s = {'event': 'sample', 'wall': time.time()}
    try:
        s.update(parse_loadavg(read('/proc/loadavg')))
    except (OSError, ValueError, IndexError):
        s['load1'] = None
    try:
        s['io_full_avg10'] = parse_psi_full_avg10(read('/proc/pressure/io'))
    except (OSError, ValueError):
        s['io_full_avg10'] = None
    if containers:
        own = {}
        for c in containers:
            if tasks.get(c) is None or not os.path.exists(tasks[c]):
                tasks[c] = container_tasks_file(c)
            own[c] = count_rd(tasks[c]) if tasks.get(c) else None
        s['own'] = own
    return s


STOP = []


def _stop(signum, frame):   # no lock, no Event.set in a handler (feedback: signal handler deadlock)
    STOP.append(signum)


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def cmd_watch(a):
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    tasks = {}
    with open(a.out, 'a', buffering=1) as f:
        f.write(json.dumps({'event': 'start', 'wall': time.time(), 'pid': os.getpid(), 'interval_s': a.interval,
                            'containers': a.container, 'watch_pid': a.pid}) + '\n')
        nxt = time.monotonic()
        while not STOP:
            f.write(json.dumps(sample(a.container, tasks)) + '\n')
            if a.pid is not None and not pid_alive(a.pid):
                break
            nxt += a.interval
            while not STOP and time.monotonic() < nxt:
                time.sleep(min(0.2, max(0., nxt - time.monotonic())))
        f.write(json.dumps(sample(a.container, tasks)) + '\n')   # one more at the end: the window's tail
        f.write(json.dumps({'event': 'stop', 'wall': time.time(), 'signal': STOP[0] if STOP else None}) + '\n')
    return 0


def cmd_verdict(a):
    try:
        rows = read_log(a.log)
    except OSError as e:
        print(json.dumps({'verdict': 'void', 'reasons': ['blind:no_log'], 'error': str(e)}))
        return EXIT_USAGE
    out = verdict(rows, a.start, a.end, a.max_load, a.max_io, a.max_gap_s)
    print(json.dumps(out))
    return EXIT_VALID if out['verdict'] == 'valid' else EXIT_VOID


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd')
    w = sub.add_parser('watch')
    w.add_argument('--out', required=True)
    w.add_argument('--interval', type=float, default=INTERVAL_S)
    w.add_argument('--container', action='append', default=[])
    w.add_argument('--pid', type=int)
    v = sub.add_parser('verdict')
    v.add_argument('--log', required=True)
    v.add_argument('--start', type=float, required=True)
    v.add_argument('--end', type=float, required=True)
    v.add_argument('--max-load', type=float, default=MAX_LOAD)
    v.add_argument('--max-io', type=float, default=MAX_IO)
    v.add_argument('--max-gap-s', type=float, default=MAX_GAP_S)
    a = p.parse_args(argv)
    if a.cmd == 'watch':
        if not a.interval > 0:
            p.error('--interval must be > 0')
        return cmd_watch(a)
    if a.cmd == 'verdict':
        return cmd_verdict(a)
    p.print_help()
    return EXIT_USAGE


if __name__ == '__main__':
    sys.exit(main())
