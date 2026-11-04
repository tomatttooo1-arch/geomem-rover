#!/usr/bin/env python3
"""The camera driver node's OWN Fast DDS shared-memory segment, checked against the profile it was given (2026-10-17).

Why: board 10-16/10-17 (docs/design/HEARTBEAT_GAP_DIG_20261016.md sections 10.2, 10.6, 11). When the driver node
cannot create the segment its profile asks for (the stack container's /dev/shm is 64 MB; the 32 MB profile, or the
8 MB one after a few failed starts left unowned 8 MB segments behind), Fast DDS does not report it: the node falls
back to the 512 KB default segment, which cannot carry one depth (614472 B) or colour (921672 B) image, and the
reliable subscribers' repairs block the producer. This check makes that silent fallback a refused start.

Read-only, no DDS participant: like the round-2 segcheck (t2_session.sh), a process OWNS the segment fastrtps_<id>
whose lock file fastrtps_<id>_el it holds open (/proc/<pid>/fd); the segment's size is the size of
/dev/shm/fastrtps_<id> (the payload plus about 25 KB of header: 8388608 -> 8413728, 524288 -> 549408 on the board).
The process given (--pid) is `ros2 launch`; the driver node (component_container) is its child and INHERITS the
launch process's open _el fds, so the node's own segments are the ones its descendants hold and --pid does not hold
(t2_readout.py "driver_node_own"; board 10-16: in the 32 MB fallback the launch process had the 32 MB segment and the
node only 549408 B). Under ROS_LOCALHOST_ONLY=1 the node has a second, default-sized segment, so the check is on the
LARGEST own segment: >= the profile's <segment_size>.

Fail-closed: exit 0 = passed; 1 = the largest own segment is smaller than the profile's; 2 = could not check (the
pid is gone, it has no child, no child holds an own segment, the profile names no SHM segment_size). --wait-s: poll
until passed (a just-started node may not have opened every fd yet), then report the last reading. --out: JSON.
--proc / --shm: the roots (unit tests use stand-ins). Python 3.8.
"""
import argparse
import json
import os
import re
import sys
import time
import xml.etree.ElementTree as ET

EL_RE = re.compile(r'^(fastrtps_[0-9a-f]{16})_el$')


def profile_segment_size(path):
    """Largest <segment_size> of an SHM transport descriptor in a Fast DDS XML profile, or None."""
    root = ET.parse(path).getroot()
    best = None
    for td in root.iter():
        if td.tag.rsplit('}', 1)[-1] != 'transport_descriptor':
            continue
        kind = size = None
        for c in td:
            t = c.tag.rsplit('}', 1)[-1]
            if t == 'type':
                kind = (c.text or '').strip()
            elif t == 'segment_size':
                size = int((c.text or '').strip())
        if kind == 'SHM' and size is not None:
            best = size if best is None else max(best, size)
    return best


def read_ppid(proc, pid):
    try:
        with open(os.path.join(proc, str(pid), 'stat')) as f:
            s = f.read()
    except OSError:
        return None
    try:
        return int(s.rsplit(')', 1)[1].split()[1])
    except (IndexError, ValueError):
        return None


def descendants(proc, pid):
    kids = {}
    for d in os.listdir(proc):
        if d.isdigit():
            pp = read_ppid(proc, int(d))
            if pp is not None:
                kids.setdefault(pp, []).append(int(d))
    out, todo = [], [pid]
    while todo:
        for c in sorted(kids.get(todo.pop(), [])):
            if c not in out:
                out.append(c)
                todo.append(c)
    return out


def held_segments(proc, pid):
    """{fastrtps_<id>} whose _el lock file process pid holds open (basename of the fd link target)."""
    names = set()
    d = os.path.join(proc, str(pid), 'fd')
    try:
        fds = os.listdir(d)
    except OSError:
        return names
    for fd in fds:
        try:
            m = EL_RE.match(os.path.basename(os.readlink(os.path.join(d, fd))))
        except OSError:
            continue
        if m:
            names.add(m.group(1))
    return names


def cmdline(proc, pid):
    try:
        with open(os.path.join(proc, str(pid), 'cmdline'), 'rb') as f:
            return f.read().replace(b'\0', b' ').decode('utf-8', 'replace').strip()[:120]
    except OSError:
        return ''


def check(pid, need, proc='/proc', shm='/dev/shm'):
    res = {'pid': pid, 'required_bytes': need, 'passed': False}
    if need is None:
        res['reason'] = 'the profile names no SHM transport segment_size: nothing to check against'
        res['code'] = 2
        return res
    if not os.path.isdir(os.path.join(proc, str(pid))):
        res['reason'] = 'driver process %d not found' % pid
        res['code'] = 2
        return res
    parent = held_segments(proc, pid)
    res['launch_cmdline'] = cmdline(proc, pid)
    res['launch_segments'] = {n: seg_size(shm, n) for n in sorted(parent)}
    kids = descendants(proc, pid)
    res['children'] = [{'pid': c, 'cmdline': cmdline(proc, c), 'own': {n: seg_size(shm, n) for n in sorted(held_segments(proc, c) - parent)}}
                       for c in kids]
    if not kids:
        res['reason'] = 'driver process %d has no child process (the driver node)' % pid
        res['code'] = 2
        return res
    own = {}
    for c in res['children']:
        own.update(c['own'])
    sizes = [s for s in own.values() if s is not None]
    res['own_segment_bytes'] = sorted(sizes)
    if not sizes:
        res['reason'] = 'no child of %d holds a shared-memory segment of its own' % pid
        res['code'] = 2
        return res
    res['largest_own_bytes'] = max(sizes)
    if max(sizes) >= need:
        res['passed'] = True
        res['code'] = 0
        res['reason'] = 'largest own segment %d B >= %d B' % (max(sizes), need)
    else:
        res['code'] = 1
        res['reason'] = ('largest own segment %d B < the profile\'s %d B: the driver node fell back to a smaller '
                         'segment (silently)' % (max(sizes), need))
    return res


def seg_size(shm, name):
    try:
        return os.stat(os.path.join(shm, name)).st_size
    except OSError:
        return None


def shm_usage(shm):
    try:
        st = os.statvfs(shm)
        return {'total_kb': st.f_blocks * st.f_frsize // 1024, 'free_kb': st.f_bavail * st.f_frsize // 1024}
    except OSError:
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--pid', type=int, required=True, help='the driver process (ros2 launch)')
    ap.add_argument('--profile', required=True, help='the Fast DDS XML profile the driver was given')
    ap.add_argument('--proc', default='/proc')
    ap.add_argument('--shm', default='/dev/shm')
    ap.add_argument('--wait-s', type=float, default=0.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args(argv)
    try:
        need = profile_segment_size(a.profile)
    except (OSError, ET.ParseError, ValueError) as exc:
        need = None
        bad = 'profile %s unreadable: %s' % (a.profile, exc)
    else:
        bad = None
    end = time.time() + a.wait_s
    while True:
        res = check(a.pid, need, a.proc, a.shm)
        if res['passed'] or time.time() >= end:
            break
        time.sleep(0.5)
    if bad:
        res['reason'] = bad
    res['profile'] = a.profile
    res['shm'] = shm_usage(a.shm)
    res['time'] = time.time()
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(res, f, indent=1, sort_keys=True)
    print(('passed: ' if res['passed'] else 'FAILED: ') + res['reason'])
    return res['code']


if __name__ == '__main__':
    sys.exit(main())
