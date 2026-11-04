"""Line E (2026-10-14): tools/car_run one-command car start/stop -- switch checks, dry runs, stopping by pid
(INT -> TERM -> KILL) and the roll-back of a partial start, with stand-in docker / lock / inference programs
(no docker, no ROS, no GPU). Python 3.8 compatible."""
import json
import re
import os
import shlex
import shutil
import signal
import stat
import subprocess
import tempfile
import textwrap
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAR = os.path.join(ROOT, 'tools', 'car_run')
UP = os.path.join(CAR, 'car_up.sh')
DOWN = os.path.join(CAR, 'car_down.sh')
COMMON = os.path.join(CAR, 'car_common.sh')
SCRIPTS = ['car_common.sh', 'car_up.sh', 'car_down.sh', 'car_inner.sh', 'car_pc.sh', 'car_scenario.sh']

# Stand-in docker: state in $FAKE_DOCKER_DIR/<name>.json; FAKE_STACK=ready writes READY into the directory mounted
# at /o, FAKE_STACK=exit makes the stack container exit at once (a failed start), FAKE_STACK=fail refuses to create it.
FAKE_DOCKER = textwrap.dedent('''\
    #!/usr/bin/env python3
    import json, os, sys
    d = os.environ['FAKE_DOCKER_DIR']
    a = sys.argv[1:]
    def path(n): return os.path.join(d, n + '.json')
    def load(n):
        try:
            return json.load(open(path(n)))
        except OSError:
            return None
    def save(n, s): json.dump(s, open(path(n), 'w'))
    open(os.path.join(d, 'calls.log'), 'a').write(' '.join(a[:4]) + '\\n')
    cmd = a[0]
    if cmd == 'run':
        name = a[a.index('--name') + 1]
        o = next((v.split(':')[0] for v in a if v.endswith(':/o')), None)
        kind = os.environ.get('FAKE_STACK', 'ready') if name.startswith('wde-car-stack-') else 'ok'
        if kind == 'fail':
            sys.stderr.write('fake: cannot create ' + name + '\\n'); sys.exit(125)
        save(name, {'running': kind != 'exit', 'exit': 0 if kind != 'exit' else 3, 'argv': a})
        if kind == 'ready' and o:
            if os.environ.get('FAKE_GC'):   # 10-15: the stack's ground_calib.json (car_up fills car.json from it)
                open(os.path.join(o, 'ground_calib.json'), 'w').write(open(os.environ['FAKE_GC']).read())
            open(os.path.join(o, 'marks.txt'), 'a').write('1.0 ready fake\\n')
            open(os.path.join(o, 'READY'), 'w').write('fake\\n')
        print('id-' + name)
    elif cmd == 'inspect':
        if a[1] == '-f':
            s = load(a[3])
            if s is None: sys.exit(1)
            print('true' if s['running'] else 'false') if 'Running' in a[2] else print(s['exit'])
        else:
            sys.exit(0 if load(a[1]) is not None else 1)
    elif cmd == 'kill':
        name = a[-1]; s = load(name)
        if s is None: sys.exit(1)
        s['running'] = False; s['exit'] = 130; save(name, s)
        open(os.path.join(d, 'signals.log'), 'a').write(name + ' ' + a[2] + '\\n')
    elif cmd == 'logs':
        print('fake log of ' + a[1])
    elif cmd == 'rm':
        try: os.remove(path(a[-1]))
        except OSError: pass
    elif cmd == 'ps':
        for f in sorted(os.listdir(d)):
            if f.endswith('.json'): print(f[:-5])
    elif cmd == 'exec':
        sys.exit(0)
    ''')

FAKE_LOCK = textwrap.dedent('''\
    #!/usr/bin/env python3
    import json, os, sys
    f = os.path.join(os.environ['FAKE_DOCKER_DIR'], 'locks.json')
    held = json.load(open(f)) if os.path.exists(f) else {}
    a = sys.argv[1:]
    if a[0] == 'acquire':
        if a[1] in held: sys.stderr.write('busy\\n'); sys.exit(1)
        held[a[1]] = 'tok' + str(len(a[2]))
        print(json.dumps({'who': 'dev1', 'token': held[a[1]], 'why': a[2]}))
    elif a[0] == 'release':
        tok = a[a.index('--token') + 1]
        if held.get(a[1]) != tok: sys.exit(1)
        del held[a[1]]
        open(os.path.join(os.environ['FAKE_DOCKER_DIR'], 'released.log'), 'a').write(a[1] + ' ' + tok + '\\n')
    elif a[0] == 'status':
        print(json.dumps([{'resource': k, 'token': v} for k, v in held.items()]))
    json.dump(held, open(f, 'w'))
    ''')

# Stand-in inference service (called as  python -m host.inference_service --socket S ...): binds the socket, says
# ready, and ignores SIGINT so that car_down has to escalate to TERM.
FAKE_PY = textwrap.dedent('''\
    #!/usr/bin/env python3
    import json, os, signal, socket, sys, time
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if os.environ.get('FAKE_PY_ENV_OUT'):
        json.dump({'PYTHONPATH': os.environ.get('PYTHONPATH'), 'argv': sys.argv[1:]}, open(os.environ['FAKE_PY_ENV_OUT'], 'w'))
    s = sys.argv[sys.argv.index('--socket') + 1]
    u = socket.socket(socket.AF_UNIX); u.bind(s); u.listen(1)
    print(json.dumps({'ready': True}), flush=True)
    while True:
        time.sleep(1)
    ''')


def write_exec(path, text):
    with open(path, 'w') as f:
        f.write(text)
    os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR)


def alive(pid):
    try:
        with open('/proc/%d/stat' % pid) as f:
            return f.read().rsplit(') ', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


def kill_if_alive(pid):
    """SIGKILL pid if it still runs. It can exit (and be reaped) between alive() and the kill: that is
    ProcessLookupError, the same outcome as not alive, so it is ignored (clean-up only, no assertion depends on it)."""
    if alive(pid):
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


# 10-17 (docs/design/LOAD_TIMING_TESTS_20261017.md): a stand-in child started with `&` from a non-interactive bash
# begins with SIGINT ignored; an INT sent before its own signal.signal() has run is lost (rc 143 after TERM, "alive
# 1 s after INT"). The scripts used to give it a fixed 0.5 s; under host load interpreter start-up took longer. Each
# stand-in now creates a marker file right after setting its handlers and the script waits for the markers
# (bounded: rc 3 and the missing marker on stderr) before the first signal. What car_stop_pid is judged on is unchanged.
WAIT_READY = ('wait_ready() { local f i; for f in "$@"; do for ((i=0; i<1500; i++)); do [ -e "$f" ] && break; '
              'sleep 0.02; done; [ -e "$f" ] || { echo "stand-in not ready: $f" >&2; kill -KILL $(jobs -p) 2>/dev/null; exit 3; }; done; }')


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='carrun_')
        self.state = os.path.join(self.tmp, 'st')
        os.mkdir(self.state)
        self.fd = os.path.join(self.tmp, 'fd')
        os.mkdir(self.fd)
        write_exec(os.path.join(self.tmp, 'docker'), FAKE_DOCKER)
        write_exec(os.path.join(self.tmp, 'lock'), FAKE_LOCK)
        write_exec(os.path.join(self.tmp, 'py'), FAKE_PY)
        # run directories go under this test's own mkdtemp (not the host default <public root>/runs/car-runs, named
        # by the second: two test processes in the same second collided there with `mkdir: File exists`)
        self.runs = os.path.join(self.tmp, 'runs')
        self.env = dict(os.environ, CAR_STATE_DIR=self.state, CAR_RUNS_DIR=self.runs, CAR_DOCKER=os.path.join(self.tmp, 'docker'),
                        CAR_LOCK=os.path.join(self.tmp, 'lock'), CAR_PY_HOST=os.path.join(self.tmp, 'py'),
                        FAKE_DOCKER_DIR=self.fd, CAR_READY_S='20', INF_READY_S='20', CAR_INT_WAIT_S='1',
                        CAR_TERM_WAIT_S='1', CAR_STACK_STOP_S='2', CAR_CONTAINER_STOP_S='2', INF_CPUS='0-%d' % (os.cpu_count() - 1),
                        # 10-26 FPGA profile: the tests written before it pin the a2 fallback, so every one of them
                        # also proves FPGA_PROFILE=a2 is the old chain verbatim (TestFpgaProfile: the file default is m2_166 since SMOKE2 11-03, v3fc2 10-26..11-03)
                        FPGA_PROFILE='a2')

    def tearDown(self):
        # never leave a stand-in behind, whatever the test did
        for root, _, files in os.walk(self.tmp):
            if 'host_pids.txt' in files:
                for line in open(os.path.join(root, 'host_pids.txt')):
                    pid = int(line.split()[1])
                    kill_if_alive(pid)
        shutil.rmtree(self.tmp, ignore_errors=True)

    def up(self, *args, **env):
        e = dict(self.env, **env)
        return subprocess.run(['bash', UP] + list(args), env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True, timeout=180)

    def down(self, *args):
        return subprocess.run(['bash', DOWN] + list(args), env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True, timeout=180)

    def containers(self):
        return sorted(f[:-5] for f in os.listdir(self.fd) if f.endswith('.json') and f != 'locks.json')

    def locks(self):
        p = os.path.join(self.fd, 'locks.json')
        return json.load(open(p)) if os.path.exists(p) else {}



# car_pc.sh before 10-15 (2c5d178), kept verbatim as the positive control: it always connected to 127.0.0.1
OLD_CAR_PC = textwrap.dedent('''\
    #!/bin/bash
    set -u
    source "$(cd "$(dirname "$0")" && pwd)/car_common.sh"; car_default_dirs
    : "${CAR_DOCKER:=docker}" "${HUBPORT:=47431}"
    RUN=${CAR_RUN:-$(readlink "$CAR_STATE_DIR/car.current" 2>/dev/null)}
    [ -n "$RUN" ] && [ -s "$RUN/car.json" ] || { echo "no car running" >&2; exit 2; }
    STACK=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['containers']['stack'])" "$RUN/car.json")
    exec $CAR_DOCKER exec -i -w /src "$STACK" python3 pc/fpgapc.py --host 127.0.0.1 --port "$HUBPORT" --token-file /o/token.txt "$@"
    ''')


def pc_run(t, script=None, args=('estop',), **env):
    """car_pc.sh (or SCRIPT) with a docker stand-in that records its argv; -> (CompletedProcess, argv or None)"""
    argv = os.path.join(t.tmp, 'pc_argv.json')
    if os.path.exists(argv):
        os.remove(argv)
    write_exec(os.path.join(t.tmp, 'dexec'),
               '#!/usr/bin/env python3\nimport json, sys\njson.dump(sys.argv[1:], open(%r, "w"))\n' % argv)
    e = dict(t.env, CAR_DOCKER=os.path.join(t.tmp, 'dexec'))
    for k in ('CAR_HUB_HOST', 'HUBPORT'):
        e.pop(k, None)
    e.update(env)
    r = subprocess.run(['bash', script or os.path.join(CAR, 'car_pc.sh')] + list(args), env=e, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
    return r, (json.load(open(argv)) if os.path.exists(argv) else None)


def pc_argv(t, **env):
    r, a = pc_run(t, **env)
    t.assertEqual(r.returncode, 0, r.stderr)
    return a


class TestCarPc(Base):
    """10-15: car_pc.sh connects to the hub address recorded for the run; nothing recorded -> refused, never 127.0.0.1"""
    def car(self, hub=None, marks=None):
        run = os.path.join(self.tmp, 'car')
        os.makedirs(run, exist_ok=True)
        c = {'containers': {'stack': 'wde-car-stack-x'}}
        if hub is not None:
            c['hub'] = hub
        json.dump(c, open(os.path.join(run, 'car.json'), 'w'))
        mp = os.path.join(run, 'marks.txt')
        if os.path.exists(mp):
            os.remove(mp)
        if marks is not None:
            open(mp, 'w').write('1.0 start\n2.0 ready kv 127.0.0.1:47811\n3.0 ready hub %s\n4.0 stop hub\n' % marks)
        return run

    def test_estop_uses_recorded_car_ip(self):
        run = self.car({'bind': '198.51.100.5', 'port': 47431})
        a = pc_argv(self, CAR_RUN=run)
        self.assertEqual(a[:5], ['exec', '-i', '-w', '/src', 'wde-car-stack-x'])
        self.assertEqual(a[a.index('--host') + 1:a.index('--host') + 4], ['198.51.100.5', '--port', '47431'])
        self.assertEqual(a[-3:], ['--token-file', '/o/token.txt', 'estop'])
        self.assertNotIn('127.0.0.1', a)

    def test_positive_control_old_script_used_loopback(self):
        """the same car.json through the pre-10-15 script: 127.0.0.1 -- the defect this test would catch"""
        d = os.path.join(self.tmp, 'old')
        os.mkdir(d)
        shutil.copy(COMMON, d)
        write_exec(os.path.join(d, 'car_pc.sh'), OLD_CAR_PC)
        run = self.car({'bind': '198.51.100.5', 'port': 47431})
        r, a = pc_run(self, script=os.path.join(d, 'car_pc.sh'), CAR_RUN=run)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(a[a.index('--host') + 1], '127.0.0.1')
        self.assertNotIn('198.51.100.5', a)

    def test_nothing_recorded_refused(self):
        run = self.car()
        r, a = pc_run(self, CAR_RUN=run)
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIsNone(a)                                  # docker never called: no command sent anywhere
        self.assertIn('no hub address recorded', r.stderr)
        self.assertIn('command NOT sent', r.stderr)
        r, a = pc_run(self, CAR_RUN=self.car({'bind': '', 'port': 47431}))
        self.assertEqual(r.returncode, 2, r.stderr)
        self.assertIsNone(a)

    def test_marks_fallback_and_disagreement(self):
        a = pc_argv(self, CAR_RUN=self.car(marks='198.51.100.6:47431'))     # a run from before car.json had "hub"
        self.assertEqual(a[a.index('--host') + 1:a.index('--host') + 4], ['198.51.100.6', '--port', '47431'])
        a = pc_argv(self, CAR_RUN=self.car({'bind': '198.51.100.6', 'port': 47431}, marks='198.51.100.6:47431'))
        self.assertEqual(a[a.index('--host') + 1], '198.51.100.6')
        r, a = pc_run(self, CAR_RUN=self.car({'bind': '198.51.100.5', 'port': 47431}, marks='198.51.100.6:47431'))
        self.assertEqual(r.returncode, 2, r.stderr)
        self.assertIsNone(a)
        self.assertIn('disagree', r.stderr)

    def test_overrides(self):
        run = self.car({'bind': '198.51.100.5', 'port': 47431})
        a = pc_argv(self, CAR_RUN=run, CAR_HUB_HOST='198.18.1.9', HUBPORT='47999')
        self.assertEqual(a[a.index('--host') + 1:a.index('--host') + 4], ['198.18.1.9', '--port', '47999'])
        a = pc_argv(self, CAR_RUN=self.car(), CAR_HUB_HOST='203.0.113.2', HUBPORT='47431')   # nothing recorded, both given
        self.assertEqual(a[a.index('--host') + 1], '203.0.113.2')
        r, a = pc_run(self, CAR_RUN=self.car(), CAR_HUB_HOST='203.0.113.2')                # no port anywhere
        self.assertEqual(r.returncode, 2, r.stderr)
        self.assertIsNone(a)

    def test_no_car(self):
        r, a = pc_run(self)
        self.assertEqual((r.returncode, r.stderr.strip()), (2, 'no car running'))
        self.assertIsNone(a)


class TestSyntax(unittest.TestCase):
    def test_bash_n(self):
        for s in SCRIPTS:
            r = subprocess.run(['bash', '-n', os.path.join(CAR, s)], stderr=subprocess.PIPE, universal_newlines=True)
            self.assertEqual(r.returncode, 0, s + r.stderr)


class TestCombinations(Base):
    INVALID = [
        (['sim', '--chassis', 'uart'], 'uart'),
        (['live', '--pose', 'reference'], 'reference'),
        (['replay', '--nav', 'nav2'], 'nav2'),
        (['sim', '--nav', 'costmap'], 'costmap'),
        (['sim', '--detector', 'stub'], 'stub'),
        (['replay', '--detector', 'yolo'], 'replay'),
        (['replay', '--detector', 'ssdlite'], 'replay'),
        (['live', '--chassis', 'pty'], 'pty'),
        (['live', '--detector', 'ssdlite'], 'ssdlite'),
        (['live', '--yolo-backend', 'torch'], 'trt'),
        (['sim', '--yolo-backend', 'trt'], 'trt'),
        (['replay', '--pose', 'rtabmap'], 'rtabmap'),
        (['sim', '--pose', 'rtabmap'], 'rtabmap'),
        # static (line-4 bench, 10-15): live only, chassis none only, costmap only
        (['live', '--pose', 'static'], 'uart'),
        (['replay', '--pose', 'static'], 'static'),
        (['live', '--pose', 'static', '--chassis', 'none'], 'nav2'),
        (['sim', '--camera', 'usb'], 'sim|replay|live'),
        (['sim', '--chassis', 'can'], 'none|pty|uart'),
        (['--camera', 'sim'], 'missing'),
        (['nosuchpreset'], 'preset'),
    ]

    def test_invalid_refused_before_anything_starts(self):
        for args, word in self.INVALID:
            run = os.path.join(self.tmp, 'r_' + '_'.join(a.strip('-') for a in args))
            r = self.up(*(args + ['--run-dir', run]))
            self.assertEqual(r.returncode, 2, (args, r.stdout, r.stderr))
            self.assertIn(word, r.stderr, args)
            self.assertFalse(os.path.exists(run), args)
        self.assertEqual(os.listdir(self.state), [])
        self.assertEqual(self.containers(), [])
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    def test_valid_dry_runs_start_nothing(self):
        cases = {
            'sim': ['--data-source sim', '--detector yolo', 'mcu_sim.py --pty', 'acquire gpu2', 'nav_startup_probe.py --activate'],
            'replay': ['--data-source replay', 'stub_inference.py', 'replay_camera.py', 'nav2_costmap_2d', '--sensor-source replay'],
            'live': ['--data-source live', '--engine', '--port /dev/fpga_chassis', '--pose-source estimated', 'rtabmap',
                     'gemini2.launch.py', '--sensor-source hardware'],
        }
        for preset, words in cases.items():
            run = os.path.join(self.tmp, 'dry_' + preset)
            r = self.up(preset, '--dry-run', '--run-dir', run)
            self.assertEqual(r.returncode, 0, r.stderr)
            for w in words:
                self.assertIn(w, r.stdout, (preset, w))
            self.assertFalse(os.path.exists(run))
        r = self.up('sim', '--detector', 'ssdlite', '--chassis', 'none', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('sim_estop_gate.py', r.stdout)
        self.assertNotIn('--detector yolo', r.stdout)
        r = self.up('sim', '--yolo-backend', 'ort', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('gpu_lock=0', r.stdout)
        self.assertEqual(os.listdir(self.state), [])
        self.assertEqual(self.containers(), [])
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    def test_live_refused_for_real_on_this_host(self):
        if os.uname().machine == 'aarch64':
            self.skipTest('board')
        r = self.up('live', '--run-dir', os.path.join(self.tmp, 'live'))
        self.assertEqual(r.returncode, 3)
        self.assertFalse(os.path.exists(os.path.join(self.tmp, 'live')))

    def test_source_marker_follows_camera(self):
        for preset, src in (('sim', 'sim'), ('replay', 'replay'), ('live', 'live')):
            r = self.up(preset, '--dry-run')
            self.assertIn('source marker=' + src, r.stdout)
            self.assertIn('hub.hub_ros --data-source ' + src, r.stdout)
            self.assertIn('ros_geo_producer --data-source ' + src, r.stdout)


# Stand-in lsusb (FAKE_USB=1: one Orbbec Gemini 2 on bus 002 device 005; else nothing) and stand-in KV check (called
# like tools/kv_link/hello_connect_loop.py; FAKE_KV=ok|mismatch|down; every call appended to kv_calls.log).
FAKE_LSUSB = textwrap.dedent("""\
    #!/bin/sh
    [ "$1 $2" = "-d 2bc5:" ] && [ "${FAKE_USB:-0}" = 1 ] && echo "Bus 002 Device 005: ID 2bc5:0670 Orbbec 3D Technology International, Inc Gemini 2"
    exit 0
    """)
FAKE_KV_CHECK = textwrap.dedent('''\
    #!/usr/bin/env python3
    import json, os, sys
    a = sys.argv[1:]
    open(os.path.join(os.environ['FAKE_DOCKER_DIR'], 'kv_calls.log'), 'a').write(' '.join(a) + '\\n')
    want = int(a[a.index('--expect-build-id') + 1], 0)
    kind = os.environ.get('FAKE_KV', 'ok')
    row = {'trial': 0, 'ok': kind == 'ok', 'reason': None, 'build_id': want if kind == 'ok' else None, 'ms': 3.0}
    if kind == 'mismatch':
        row['reason'] = 'build_id_mismatch:%s!=%#010x' % (os.environ.get('FAKE_KV_ACTUAL', '0x0117bc0a'), want)
    elif kind == 'down':
        row['reason'] = 'hello_timeout'
    print(json.dumps(row, sort_keys=True))
    print(json.dumps({'trials': 1, 'reasons': {row['reason'] or 'ok': 1}}, sort_keys=True))
    ''')


class TestLiveBoard(Base):
    """Live preset unblocked (2026-10-14): the camera's USB node from lsusb (refused with 3 when absent), the KV260
    checked before anything starts (serving, BUILD_ID 0x00da5041; refused with 9), the board inference service with
    the board's packages on PYTHONPATH, no fake KV container on the board. Board-only real start reached with the
    stand-ins CAR_ARCH / CAR_LSUSB / CAR_USB_ROOT / CAR_KV_CHECK / CAR_PY_BOARD (no camera, no KV260, no board)."""
    PYLIB = '/home/robot/fpga/p2/pylib'
    ENGINE = '/data/fpga/yolo_engine_20261015/yolov8n-seg-640-fp16.engine'   # rebuilt 10-15 (super firmware)

    def setUp(self):
        Base.setUp(self)
        write_exec(os.path.join(self.tmp, 'lsusb'), FAKE_LSUSB)
        write_exec(os.path.join(self.tmp, 'kvcheck'), FAKE_KV_CHECK)
        self.usb = os.path.join(self.tmp, 'usb')
        self.node = os.path.join(self.usb, '002', '005')
        self.env.update(CAR_ARCH='aarch64', CAR_BOARD_DRAFT='1', CAR_HUB_BIND='198.51.100.1',
                        CAR_LSUSB=os.path.join(self.tmp, 'lsusb'), CAR_USB_ROOT=self.usb,
                        CAR_KV_CHECK=os.path.join(self.tmp, 'kvcheck'), CAR_PY_BOARD=os.path.join(self.tmp, 'py'))
        # the live preset is rtabmap: its real start needs a pinned database file (10-15 precheck, exit 2 without)
        self.db = os.path.join(self.tmp, 'pinned.db')
        with open(self.db, 'w') as f:
            f.write('db')
        self.env['RTABMAP_DB'] = self.db
        # 10-15: the board Nav2 map (rtabmap-export of the pinned database, <dir>/run_<stem>/export/*.yaml) and the
        # rtabmap database directory (board default /data/fpga/rtabmap, mkdir -p + writable on the real start)
        self.map_yaml = os.path.join(self.tmp, 'run_pinned', 'export', 'pinned.yaml')
        os.makedirs(os.path.dirname(self.map_yaml))
        with open(self.map_yaml, 'w') as f:
            f.write('image: pinned.pgm\n')
        self.env['RTABMAP_DB_DIR'] = os.path.join(self.tmp, 'rtabmap_dbdir')

    def make_node(self):   # a character device at the resolved path (a link to /dev/null)
        os.makedirs(os.path.dirname(self.node))
        os.symlink('/dev/null', self.node)

    def kv_calls(self):
        p = os.path.join(self.fd, 'kv_calls.log')
        return open(p).read().splitlines() if os.path.exists(p) else []

    def docker_runs(self):
        p = os.path.join(self.fd, 'calls.log')
        return [l for l in open(p).read().splitlines() if l.startswith('run')] if os.path.exists(p) else []

    def line(self, out, prefix):
        return next(l for l in out.splitlines() if l.startswith(prefix))

    def test_dry_run_device_resolved_pythonpath_defaults(self):
        r = self.up('live', '--dry-run', FAKE_USB='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('# camera device: %s (from ' % self.node, r.stdout)
        stack = self.line(r.stdout, '[container stack]')
        # 10-17 (CAM_USB_REENUM_20261017): the whole USB root + 'c 189:* rw', not --device of the node (re-enumeration)
        self.assertIn('-v %s:/dev/bus/usb --device-cgroup-rule c 189:* rw --device /dev/fpga_chassis' % self.usb, stack)
        self.assertNotIn('--device %s' % self.node, stack)   # live preset = uart: the chassis port still mounted
        self.assertNotIn('<Orbbec', stack)
        self.assertIn('-e KV_BUILD_ID=0x00da5041', stack)
        self.assertIn('-e LIVE_START_TRIES=10 -e LIVE_START_WAIT_S=20 -e LIVE_START_GAP_S=10', stack)
        inf = self.line(r.stdout, '[inference]')
        self.assertRegex(inf, r'PYTHONPATH=\S+/src:%s ' % self.PYLIB)          # snapshot first, then the pylib
        self.assertIn('--engine %s ' % self.ENGINE, inf)
        kv = self.line(r.stdout, '[kv precheck]')
        self.assertIn('--server 198.51.100.79:47800', kv)
        self.assertIn('--expect-build-id 0x00da5041', kv)
        self.assertIn('--kv-expect-build-id 0x00da5041', r.stdout)                  # the producer, in the inner plan
        self.assertIn('start retry: up to 10 tries; good = probe ', r.stdout)
        self.assertIn('depth_probe.py --topic /camera/depth/image_raw --count 30 --timeout-s 20', r.stdout)
        self.assertIn('-e LIVE_DEPTH_MSGS=30 ', stack)
        self.assertNotIn('[container kv]', r.stdout)
        # camera mount (measured 10-17 after the owner's re-tilt: 0.39 m, 7.0 deg; 10-16: 0.375 m, 30 deg; 10-14: 0.36 m)
        # -> static TF + producer --height-cell 8
        self.assertIn('-e CAM_HEIGHT=0.39 -e CAM_PITCH_DEG=16.0 -e HEIGHT_CELL=8 -e CAM_MOUNT_TF=0.1 0 0.39 ', stack)
        # car body height (owner 10-16: 0.31 m) -> the stack and the rolling bridge's obstacle band
        self.assertIn('-e ROBOT_HEIGHT=0.31 ', stack)
        self.assertIn('-p robot_height_m:=0.31 ', self.line(r.stdout, '    [bridge]'))
        self.assertIn('# car body: CAR_ROBOT_HEIGHT=0.31 m -> obstacle band top 0.360 m', r.stdout)
        tf = self.line(r.stdout, '    [mount_tf]')
        self.assertIn('static_transform_publisher --x 0.1 --y 0 --z 0.39 --qx -0.564720585 --qy 0.564720585 '
                      '--qz -0.425547484 --qw 0.425547484 --frame-id base_link --child-frame-id camera_color_optical_frame', tf)
        prod = self.line(r.stdout, '    [producer]')
        self.assertIn('--height-cell 8 ', prod)
        self.assertIn('--camera-frame camera_color_optical_frame', prod)
        self.assertNotIn('camera_depth_optical_frame', r.stdout)
        # overrides
        r = self.up('live', '--dry-run', FAKE_USB='1', NANO_PYLIB='/opt/pylib', KV_BUILD_ID='0x0117bc0a',
                    LIVE_START_TRIES='3')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertRegex(self.line(r.stdout, '[inference]'), r'PYTHONPATH=\S+/src:/opt/pylib ')
        self.assertIn('--expect-build-id 0x0117bc0a', r.stdout)
        self.assertIn('-e LIVE_START_TRIES=3', r.stdout)
        self.assertIn('start retry: up to 3 tries', r.stdout)
        # 0.33 m is below the camera-above-band minimum for the 0.31 m car body (0.365 m): this keeps the height_cell 7
        # derivation covered with a 0.25 m body (0.33 >= 0.305)
        r = self.up('live', '--dry-run', CAR_CAM_HEIGHT='0.33', CAR_CAM_PITCH_DEG='27', CAR_ROBOT_HEIGHT='0.25')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('-p robot_height_m:=0.25 ', self.line(r.stdout, '    [bridge]'))
        self.assertIn('--height-cell 7 ', self.line(r.stdout, '    [producer]'))
        self.assertIn('--z 0.33 --qx -0.602907642 --qy 0.602907642 --qz -0.369462278 --qw 0.369462278',
                      self.line(r.stdout, '    [mount_tf]'))
        r = self.up('live', '--dry-run', CAR_HEIGHT_CELL='8', CAR_CAM_HEIGHT='0.38')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('--height-cell 8 ', self.line(r.stdout, '    [producer]'))
        # a block that cannot hold the ground is refused before anything (2)
        for env in ({'CAR_HEIGHT_CELL': '7'}, {'CAR_CAM_HEIGHT': '0.40'}):
            r = self.up('live', '--dry-run', **env)
            self.assertEqual(r.returncode, 2, (env, r.stdout))
            self.assertIn('does not hold the ground', r.stderr)
        # camera below the obstacle band top + 5 mm (owner 10-16) is refused before anything (2): 0.36 < 0.31 + 0.055;
        # exactly 0.365 passes
        for cam, rh, rc in (('0.36', None, 2), ('0.33', None, 2), ('0.365', None, 0), ('0.37', '0.32', 2)):
            env = {'CAR_CAM_HEIGHT': cam}
            if rh:
                env['CAR_ROBOT_HEIGHT'] = rh
            r = self.up('live', '--dry-run', **env)
            self.assertEqual(r.returncode, rc, (env, r.stdout, r.stderr))
            if rc:
                self.assertIn('相机太低', r.stderr)
        # a dry run checks nothing and creates nothing
        self.assertEqual(self.kv_calls(), [])
        self.assertEqual(os.listdir(self.state), [])
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    def nav_plan(self, out):
        """(launch order of the inner plan's [name] lines, the [navigation] line)"""
        names = [l.split(']')[0].strip().lstrip('[').split(',')[0] for l in out.splitlines() if l.startswith('    [')]
        return names, self.line(out, '    [navigation]')

    def test_board_nav2_rolling_dry_run(self):
        """10-15 (merged real-car-nav2ctl): board nav2 defaults to FPGA_NAV_COSTMAP=rolling -- the DWB overrides file
        (nav_controller_variants dwb.yaml), the car speed limits (0.10 m xy goal tolerance) still on, and Nav2 started
        AFTER hub, bridge and producer (the rolling local costmap is the bridge's /fpga/rolling_map/grid, published only
        once the producer feeds it), with the grid probe before activation"""
        for env in ({}, {'FPGA_NAV_COSTMAP': ''}, {'FPGA_NAV_COSTMAP': 'rolling'}):   # empty = not given = rolling
            r = self.up('live', '--dry-run', FAKE_USB='1', **env)
            self.assertEqual(r.returncode, 0, (env, r.stderr))
            names, nav = self.nav_plan(r.stdout)
            self.assertIn('FPGA_NAV_COSTMAP=rolling', nav, env)
            self.assertIn('FPGA_NAV2_OVERRIDES=%s/controllers/dwb.yaml ' % r.stdout.split('# run directory: ')[1].split()[0],
                          nav, env)
            self.assertNotIn('rpp_heading_precision', nav)
            self.assertIn('FPGA_NAV_SPEED_LIMITS=car', nav)
            self.assertIn('-e NAV_COSTMAP=rolling ', self.line(r.stdout, '[container stack]'))
            for before in ('hub', 'bridge', 'producer', 'nav2_configs'):
                self.assertLess(names.index(before), names.index('navigation'), (before, names))
            self.assertLess(names.index('navigation'), names.index('rolling_grid'))
            self.assertLess(names.index('rolling_grid'), names.index('nav2_activate'))
            self.assertLess(names.index('nav2_activate'), names.index('runtime'))
            self.assertIn('grid_probe.py --topic /fpga/rolling_map/grid --timeout-s 60 ',
                          self.line(r.stdout, '    [rolling_grid, once]'))
            # 10-22 find-nav: the board runner's per-goal budget 60 s (both clocks), task 180 s, before --ros-args
            rt = self.line(r.stdout, '    [runtime]')
            self.assertIn('-e NAV_TIMEOUT_S=90 -e TASK_TIMEOUT_S=300 ', self.line(r.stdout, '[container stack]'))
            self.assertIn(' --navigation-timeout-s 90 --task-timeout-s 300 --stream-timeout-s 5.0 --max-nav-failures 5 --pose-trust off --geometry-age-limit-ms 500 --input-hiccup drop ', rt)
            self.assertLess(rt.index('--task-timeout-s'), rt.index('--ros-args'))
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_NAV_TIMEOUT_S='75.5', CAR_TASK_TIMEOUT_S='240')
        self.assertIn(' --navigation-timeout-s 75.5 --task-timeout-s 240 ', self.line(r.stdout, '    [runtime]'))
        for bad in ('0', '-1', 'x', '0.0', '1e3'):   # empty = not given = 60 (the := default)
            run = os.path.join(self.tmp, 'bad_timeout')
            r = self.up('live', '--run-dir', run, FAKE_USB='1', CAR_NAV_TIMEOUT_S=bad)
            self.assertEqual(r.returncode, 2, (bad, r.stdout))
            self.assertIn('CAR_NAV_TIMEOUT_S 须是正数秒', r.stderr)
            self.assertFalse(os.path.exists(run))
        # 10-26: the board's frame-gap watchdog is CAR_STREAM_TIMEOUT_S (default 5.0, 2..20 s), into the container
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_STREAM_TIMEOUT_S='8')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(' -e STREAM_TIMEOUT_S=8 ', self.line(r.stdout, '[container stack]'))
        self.assertIn(' --stream-timeout-s 8 ', self.line(r.stdout, '    [runtime]'))
        for ok in ('2', '20', '2.5'):
            r = self.up('live', '--dry-run', FAKE_USB='1', CAR_STREAM_TIMEOUT_S=ok)
            self.assertEqual(r.returncode, 0, (ok, r.stderr))
        for bad in ('1.9', '20.5', '0', '-1', 'x', '1e1'):   # empty = not given = 5.0 (the := default)
            run = os.path.join(self.tmp, 'bad_stream')
            r = self.up('live', '--run-dir', run, FAKE_USB='1', CAR_STREAM_TIMEOUT_S=bad)
            self.assertEqual(r.returncode, 2, (bad, r.stdout))
            self.assertIn('CAR_STREAM_TIMEOUT_S 须是 2 到 20 之间的秒数', r.stderr)
            self.assertFalse(os.path.exists(run))
        r = self.up('live', '--dry-run', FAKE_USB='1', GRID_READY_S='5')
        self.assertIn('-e GRID_READY_S=5 ', self.line(r.stdout, '[container stack]'))
        self.assertIn('--timeout-s 5 ', self.line(r.stdout, '    [rolling_grid, once]'))
        # explicit sim: the board's pre-10-15 Nav2 (RPP heading precision, obstacle layers), in the old place
        r = self.up('live', '--dry-run', FAKE_USB='1', FPGA_NAV_COSTMAP='sim')
        self.assertEqual(r.returncode, 0, r.stderr)
        names, nav = self.nav_plan(r.stdout)
        self.assertIn('FPGA_NAV_COSTMAP=sim', nav)
        self.assertIn('/controllers/rpp_heading_precision.yaml ', nav)
        self.assertIn('FPGA_NAV_SPEED_LIMITS=car', nav)
        self.assertNotIn('rolling_grid', names)
        self.assertLess(names.index('nav2_activate'), names.index('hub'))
        # anything else: refused before anything starts
        run = os.path.join(self.tmp, 'bad_costmap')
        r = self.up('live', '--run-dir', run, FAKE_USB='1', FPGA_NAV_COSTMAP='obstacle')
        self.assertEqual(r.returncode, 2, r.stdout)
        self.assertIn("FPGA_NAV_COSTMAP 只能是 rolling|sim（给的是 'obstacle'", r.stderr)
        self.assertFalse(os.path.exists(run))
        self.assertEqual(self.kv_calls(), [])
        self.assertEqual(os.listdir(self.state), [])
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    def test_fault_nav_injection_host_only(self):
        """10-25 find-retry: FPGA_FAULT_NAV_FAIL_FIRST reaches the stack container (the runtime reads it) on the host
        simulation only, only when set non-empty; the board container never carries it"""
        r = self.up('sim', '--dry-run', CAR_ARCH='x86_64', FPGA_FAULT_NAV_FAIL_FIRST='2')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(' -e FPGA_FAULT_NAV_FAIL_FIRST=2 ', self.line(r.stdout, '[container stack]'))
        for env in ({}, {'FPGA_FAULT_NAV_FAIL_FIRST': ''}):
            r = self.up('sim', '--dry-run', CAR_ARCH='x86_64', **env)
            self.assertEqual(r.returncode, 0, (env, r.stderr))
            self.assertNotIn('FPGA_FAULT_NAV_FAIL_FIRST', r.stdout)
        r = self.up('live', '--dry-run', FAKE_USB='1', FPGA_FAULT_NAV_FAIL_FIRST='2')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('FPGA_FAULT_NAV_FAIL_FIRST', r.stdout)
        self.assertNotIn('--fault-nav-fail-first', r.stdout)

    def test_host_nav2_costmap_unchanged(self):
        """the simulation's Nav2 (host) keeps the launch default: no FPGA_NAV_COSTMAP, RPP heading precision, Nav2 before
        the hub -- FPGA_NAV_COSTMAP is not used there, whatever it says"""
        for env in ({}, {'FPGA_NAV_COSTMAP': 'rolling'}, {'FPGA_NAV_COSTMAP': 'junk'}):
            r = self.up('sim', '--dry-run', CAR_ARCH='x86_64', **env)
            self.assertEqual(r.returncode, 0, (env, r.stderr))
            names, nav = self.nav_plan(r.stdout)
            self.assertNotIn('FPGA_NAV_COSTMAP', r.stdout)
            self.assertNotIn('NAV_COSTMAP', self.line(r.stdout, '[container stack]'))
            self.assertNotIn('--navigation-timeout-s', self.line(r.stdout, '    [runtime]'))   # host: runner's 30/120
            self.assertNotIn('--task-timeout-s', self.line(r.stdout, '    [runtime]'))
            self.assertNotIn('--stream-timeout-s', self.line(r.stdout, '    [runtime]'))   # host: runner's 2.0
            self.assertIn('/controllers/rpp_heading_precision.yaml ', nav)
            self.assertNotIn('rolling_grid', names)
            self.assertLess(names.index('nav2_activate'), names.index('hub'))

    def test_host_sim_nav_profile_car_and_standin_wall(self):
        """10-26 e2e flow sim: CAR_SIM_NAV_PROFILE=car = the car's Nav2 switches + controller (DWB) + the board's runner
        budgets in the host simulation (host map/costmap unchanged); WALL_X_M reaches the stack only with CAR_MAPPING=1 on the host"""
        r = self.up('sim', '--dry-run', CAR_ARCH='x86_64', CAR_SIM_NAV_PROFILE='car', CAR_MAPPING='1', WALL_X_M='2.5',
                    RTABMAP_DB='', CAR_MAP_YAML='')
        self.assertEqual(r.returncode, 0, r.stderr)
        names, nav = self.nav_plan(r.stdout)
        self.assertIn('FPGA_NAV_SPEED_LIMITS=car', nav)
        self.assertIn('/controllers/dwb.yaml ', nav)    # the rotation shim (car default) wraps DWB only
        self.assertNotIn('FPGA_NAV_COSTMAP', nav)
        stack = self.line(r.stdout, '[container stack]')
        self.assertIn('-e SIM_NAV_PROFILE=car -e NAV_TIMEOUT_S=90 -e TASK_TIMEOUT_S=300 ', stack)
        self.assertIn('-e WALL_X_M=2.5 ', stack)
        rt = self.line(r.stdout, '    [runtime]')
        self.assertIn(' --navigation-timeout-s 90 --task-timeout-s 300 ', rt)
        # not given / sim: the container line and Nav2 as before
        for env in ({}, {'CAR_SIM_NAV_PROFILE': 'sim'}, {'WALL_X_M': '2.5'}):
            r = self.up('sim', '--dry-run', CAR_ARCH='x86_64', **env)
            self.assertEqual(r.returncode, 0, (env, r.stderr))
            names, nav = self.nav_plan(r.stdout)
            self.assertNotIn('FPGA_NAV_SPEED_LIMITS', nav)
            self.assertIn('/controllers/rpp_heading_precision.yaml ', nav)
            stack = self.line(r.stdout, '[container stack]')
            self.assertNotIn('SIM_NAV_PROFILE', stack)
            self.assertNotIn('WALL_X_M', stack)
            self.assertNotIn('--navigation-timeout-s', self.line(r.stdout, '    [runtime]'))
        for env, msg in (({'CAR_SIM_NAV_PROFILE': 'board'}, 'CAR_SIM_NAV_PROFILE 只能是 sim 或 car'),):
            run = os.path.join(self.tmp, 'bad_sim_nav')
            r = self.up('sim', '--run-dir', run, CAR_ARCH='x86_64', **env)
            self.assertEqual(r.returncode, 2, r.stdout)
            self.assertIn(msg, r.stderr)
            self.assertFalse(os.path.exists(run))
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_SIM_NAV_PROFILE='car')
        self.assertEqual(r.returncode, 2, r.stdout)
        self.assertIn('只用于宿主仿真', r.stderr)

    def test_dry_run_device_unresolved(self):
        r = self.up('live', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('# camera device: dry-run 未解析', r.stdout)
        self.assertIn('-v %s:/dev/bus/usb --device-cgroup-rule c 189:* rw ' % self.usb, self.line(r.stdout, '[container stack]'))

    def test_host_presets_keep_no_build_id(self):
        """the A2 build id is a board default only: the host's fake KV is not checked for one"""
        for preset in ('sim', 'replay'):
            r = self.up(preset, '--dry-run')
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn('-e KV_BUILD_ID= ', r.stdout)
            self.assertNotIn('--kv-expect-build-id', r.stdout)
            self.assertNotIn('[kv precheck]', r.stdout)
            self.assertNotIn('--height-cell', r.stdout)           # the producer default (7) and common.py's mount
            self.assertNotIn('mount_tf', r.stdout)
            self.assertNotIn('CAM_MOUNT_TF', r.stdout)

    def test_kv_unregistered_build_warns_and_starts_on_cpu_fusion(self):
        """10-26 2nd pass (owner: success rate): a KV260 serving an UNREGISTERED BUILD_ID no longer exits 9 -- warning
        (stderr, kv_precheck.txt, up_marks.txt), its id used, the producer on the Nano CPU fusion (as for a2)"""
        self.make_node()
        run = os.path.join(self.tmp, 'runs', 'kv_unreg')
        e = dict(self.env)
        e.update(FPGA_PROFILE='v3fc2',             # a remote_pl profile is requested (also the file default)
                 FAKE_USB='1', FAKE_KV='mismatch')     # FAKE_KV_ACTUAL default 0x0117bc0a: in no profile
        before = len(self.kv_calls())
        r = subprocess.run(['bash', UP, 'live', '--run-dir', run], env=e, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, universal_newlines=True, timeout=180)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('WARNING board: KV260 serves BUILD_ID 0x0117bc0a, registered in no profile', r.stderr)
        self.assertIn('geomem-kv-start', r.stderr)                                  # the fix stays in the warning
        self.assertEqual(len(self.kv_calls()) - before, 3)                           # three tries, then on
        argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-kv_unreg.json')))['argv']
        self.assertIn('KV_BUILD_ID=0x0117bc0a', argv)
        self.assertIn('ROLLING_BACKEND=cpu', argv)
        self.assertNotIn('ROLLING_BACKEND=remote_pl', argv)
        pre = open(os.path.join(run, 'kv_precheck.txt')).read()
        self.assertIn('WARN unregistered: KV260 serves BUILD_ID 0x0117bc0a', pre)
        self.assertNotIn('accepted: KV260 serves registered', pre)
        self.assertIn('WARN board precheck: KV260 serves BUILD_ID 0x0117bc0a', open(os.path.join(run, 'up_marks.txt')).read())
        self.assertEqual(self.down().returncode, 0)

    def test_kv_precheck_refuses_before_anything(self):
        """a KV260 that does not answer at all is still refused (9) before anything is created"""
        self.make_node()
        for kind, word in (('down', 'hello_timeout'),):
            run = os.path.join(self.tmp, 'runs', 'kv_' + kind)
            before = len(self.kv_calls())
            r = self.up('live', '--run-dir', run, FAKE_USB='1', FAKE_KV=kind)
            self.assertEqual(r.returncode, 9, r.stdout + r.stderr)
            self.assertIn(word, r.stderr)
            self.assertIn('kv_board.sh start', r.stderr)
            self.assertIn('0x00da5041', r.stderr)
            self.assertEqual(len(self.kv_calls()) - before, 3)                         # three tries
            self.assertFalse(os.path.exists(run))
        self.assertIn('--server 198.51.100.79:47800 --trials 1', self.kv_calls()[0])
        self.assertEqual(os.listdir(self.state), [])
        self.assertEqual(self.docker_runs(), [])
        self.assertEqual(self.containers(), [])

    def test_camera_missing_refused(self):
        run = os.path.join(self.tmp, 'runs', 'nocam')
        r = self.up('live', '--run-dir', run)                           # lsusb lists no Orbbec device
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn('no Orbbec camera', r.stderr)
        self.assertEqual(len(self.kv_calls()), 1)                       # the KV was fine (checked first)
        r = self.up('live', '--run-dir', run, FAKE_USB='1')             # listed, but no device node there
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn(self.node, r.stderr)
        self.assertFalse(os.path.exists(run))
        self.assertEqual(os.listdir(self.state), [])
        self.assertEqual(self.docker_runs(), [])

    def test_board_up_down_with_standins(self):
        """the whole board start with stand-ins: no fake KV container, the camera node and the build id reach the stack
        container, the inference service gets the pylib on PYTHONPATH; car_down leaves nothing"""
        self.make_node()
        run = os.path.join(self.tmp, 'runs', 'board1')
        envout = os.path.join(self.tmp, 'inf_env.json')
        r = self.up('live', '--run-dir', run, FAKE_USB='1', FAKE_PY_ENV_OUT=envout)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(self.containers(), ['wde-car-stack-board1'])
        argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-board1.json')))['argv']
        self.assertEqual(argv[argv.index('--device-cgroup-rule') + 1], 'c 189:* rw')
        self.assertIn('%s:/dev/bus/usb' % self.usb, argv)
        self.assertNotIn(self.node, argv)   # 10-17: the node is the presence check only, not passed
        self.assertIn('KV_BUILD_ID=0x00da5041', argv)
        self.assertIn('HUB_BIND=198.51.100.1', argv)
        self.assertIn('HEIGHT_CELL=8', argv)
        self.assertIn('CAM_MOUNT_TF=0.1 0 0.39 -0.564720585 0.564720585 -0.425547484 0.425547484', argv)
        self.assertIn('ROBOT_HEIGHT=0.31', argv)
        mount = json.load(open(os.path.join(run, 'car.json')))['camera_mount']
        self.assertEqual((mount['height_m'], mount['pitch_deg'], mount['height_cell']), (0.39, 16.0, 8))   # 10-28 default 16
        self.assertEqual(mount['robot_height_m'], 0.31)
        self.assertEqual(argv[-2:], ['fpga-ros:humble-nav-orbbec-scipy-20261017', '/src/tools/car_run/car_inner.sh'])
        inf = json.load(open(envout))
        self.assertEqual(inf['PYTHONPATH'], os.path.join(run, 'src') + ':' + self.PYLIB)
        self.assertEqual(inf['argv'][:2], ['-m', 'host.inference_service'])
        self.assertIn(self.ENGINE, inf['argv'])
        self.assertIn('"ok": true', open(os.path.join(run, 'kv_precheck.txt')).read())
        self.assertIn('board precheck: kv 198.51.100.79:47800 build id 0x00da5041 ok; camera ' + self.node,
                      open(os.path.join(run, 'up_marks.txt')).read())
        self.assertEqual(json.load(open(os.path.join(run, 'car.json')))['platform'], 'board')
        budgets = json.load(open(os.path.join(run, 'car.json')))['runner_budgets']   # 10-26: effective values
        self.assertEqual((budgets['navigation_timeout_s'], budgets['task_timeout_s'], budgets['stream_timeout_s']),
                         (90.0, 300.0, 5.0))   # D-048 (main 10-26): board defaults 90 / 300 s
        self.assertIn('STREAM_TIMEOUT_S=5.0', argv)
        # 10-15: car_up records the hub address, car_pc estop connects there (not 127.0.0.1)
        hub = json.load(open(os.path.join(run, 'car.json')))['hub']
        self.assertEqual((hub['bind'], hub['port']), ('198.51.100.1', 47431))
        a = pc_argv(self, CAR_RUN=run)
        self.assertEqual(a[a.index('--host') + 1], '198.51.100.1')
        self.assertNotIn('127.0.0.1', a)
        pid = int(open(os.path.join(run, 'host_pids.txt')).read().split()[1])
        d = self.down()
        self.assertEqual(d.returncode, 0, d.stdout + d.stderr)
        self.assertEqual(self.containers(), [])
        self.assertFalse(alive(pid))
        self.assertEqual(os.listdir(self.state), [])

    def test_board_wheel_params_in_car_json(self):
        """owner 10-16 (D-041): a real board start with the wheel odometry (--pose odom, chassis uart) records the
        wheel parameters it used and where they came from in car.json; the stack gets them; env still overrides"""
        self.make_node()
        port = os.path.join(self.tmp, 'fpga_chassis')
        open(port, 'w').close()
        for name, env, used, src in (
                ('wd1', {}, {'wheel_diam_m': '0.081', 'counts_per_rev': '1560', 'track_m': '0.394'}, 'default (real car'),
                ('wd2', {'CAR_WHEEL_DIAM_M': '0.07', 'CAR_COUNTS_PER_REV': '1400', 'CAR_TRACK_M': '0.17'},
                 {'wheel_diam_m': '0.07', 'counts_per_rev': '1400', 'track_m': '0.17'}, 'env (')):
            run = os.path.join(self.tmp, 'runs', name)
            r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--run-dir', run, FAKE_USB='1', RTABMAP_DB='',
                        CHASSIS_PORT=port, **env)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-%s.json' % name)))['argv']
            for k, v in (('WHEEL_DIAM_M', used['wheel_diam_m']), ('COUNTS_PER_REV', used['counts_per_rev']),
                         ('TRACK_M', used['track_m'])):
                self.assertIn('%s=%s' % (k, v), argv)
            wo = json.load(open(os.path.join(run, 'car.json')))['wheel_odom']
            self.assertTrue(wo['enabled'])
            self.assertEqual(wo['params_used'], used)
            self.assertTrue(wo['source'].startswith(src), wo['source'])
            self.assertFalse(wo['placeholder'])
            self.assertEqual(wo['placeholder_note'], '')
            self.assertEqual(wo['params_given'], {'wheel_diam_m': env.get('CAR_WHEEL_DIAM_M', ''),   # raw environment
                                                  'counts_per_rev': env.get('CAR_COUNTS_PER_REV', ''),
                                                  'track_m': env.get('CAR_TRACK_M', '')})
            d = self.down()
            self.assertEqual(d.returncode, 0, d.stdout + d.stderr)
            self.assertEqual(self.containers(), [])

    # ---- 10-15: rtabmap needs its database; the line-4 bench combination (live --pose static)
    STATIC = ['live', '--pose', 'static', '--chassis', 'none', '--nav', 'costmap']

    def test_rtabmap_without_db_refused_before_anything(self):
        """board --pose rtabmap with no pinned database: exit 2 before the KV check, the run directory, docker"""
        self.make_node()
        for db in ('', os.path.join(self.tmp, 'no_such.db')):
            run = os.path.join(self.tmp, 'runs', 'nodb')
            r = self.up('live', '--run-dir', run, FAKE_USB='1', RTABMAP_DB=db)
            self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
            self.assertIn('needs RTABMAP_DB', r.stderr)
            self.assertIn('--pose static --chassis none --nav costmap', r.stderr)
            self.assertFalse(os.path.exists(run))
        self.assertEqual(self.kv_calls(), [])
        self.assertEqual(os.listdir(self.state), [])
        self.assertEqual(self.docker_runs(), [])
        # the dry run says so instead of refusing
        r = self.up('live', '--dry-run', RTABMAP_DB='')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("# rtabmap precheck: RTABMAP_DB='' must be a file", r.stdout)

    def test_frame_drop_log_default_live_only(self):
        """10-17 (colour-rate dig 10-16 advice): the live start adds the driver's enable_frame_drop_log:=true to the
        launch arguments by default; LIVE_FRAME_DROP_LOG=false leaves them as given; an explicit value in
        LIVE_LAUNCH_ARGS is kept, not doubled; replay/sim start no driver and carry nothing; a bad value is refused"""
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='')
        self.assertEqual(r.returncode, 0, r.stderr)
        cam = self.line(r.stdout, '    [camera]')
        self.assertIn('gemini2.launch.py color_width:=640 ', cam)
        self.assertIn(' publish_tf:=false enable_frame_drop_log:=true ', cam)
        self.assertEqual(cam.count('enable_frame_drop_log:='), 1)
        stack = self.line(r.stdout, '[container stack]')
        self.assertIn(' enable_frame_drop_log:=true -e LIVE_FRAME_DROP_LOG=true ', stack)
        # the live preset (rtabmap) too
        r = self.up('live', '--dry-run', FAKE_USB='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(' enable_frame_drop_log:=true ', self.line(r.stdout, '    [camera]'))
        # off
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', LIVE_FRAME_DROP_LOG='false')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('enable_frame_drop_log', r.stdout)
        self.assertIn('-e LIVE_FRAME_DROP_LOG=false ', self.line(r.stdout, '[container stack]'))
        self.assertIn(' publish_tf:=false ', self.line(r.stdout, '    [camera]'))
        # given explicitly in LIVE_LAUNCH_ARGS: kept as given, not added a second time
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='',
                    LIVE_LAUNCH_ARGS='color_fps:=15 enable_frame_drop_log:=false')
        self.assertEqual(r.returncode, 0, r.stderr)
        cam = self.line(r.stdout, '    [camera]')
        self.assertIn('gemini2.launch.py color_fps:=15 enable_frame_drop_log:=false ', cam)
        self.assertNotIn('enable_frame_drop_log:=true', r.stdout)
        # bad value: refused before anything
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', LIVE_FRAME_DROP_LOG='1')
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn('LIVE_FRAME_DROP_LOG must be true or false', r.stderr)
        # replay and sim (host presets): no driver, no frame-drop parameter anywhere
        for preset in ('replay', 'sim'):
            e = {k: v for k, v in self.env.items() if k != 'CAR_ARCH'}
            r = subprocess.run(['bash', UP, preset, '--dry-run'], env=dict(e, CAR_ARCH='x86_64'), stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, universal_newlines=True, timeout=180)
            self.assertEqual(r.returncode, 0, (preset, r.stderr))
            self.assertNotIn('enable_frame_drop_log', r.stdout, preset)
            self.assertNotIn('gemini2.launch.py', r.stdout, preset)

    def test_static_dry_run(self):
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', LIVE_LAUNCH_ARGS='color_fps:=15 x:=1')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn('pose=static', out)
        self.assertIn('# pose static: bench only', out)
        self.assertIn('bench_pose.py --odom-hz 0 --map-m 8', self.line(out, '    [bench_pose]'))
        self.assertNotIn('rtabmap', out)                       # no rtabmap node, no rtabmap precheck
        self.assertNotIn('nav_startup_probe', out)
        self.assertIn('nav2_costmap_2d', out)
        self.assertIn('/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d --ros-args', self.line(out, '    [costmap]'))
        self.assertIn('/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args', self.line(out, '    [lifecycle]'))
        self.assertIn('/opt/ros/humble/lib/tf2_ros/static_transform_publisher --x', self.line(out, '    [mount_tf]'))
        self.assertNotIn('ros2 run', out)
        stack = self.line(out, '[container stack]')
        self.assertIn('-v %s:/dev/bus/usb --device-cgroup-rule c 189:* rw ' % self.usb, stack)
        self.assertNotIn('--device ', stack)   # chassis none (--device-cgroup-rule stays): the port is not mounted
        self.assertIn('-e LIVE_LAUNCH_ARGS=color_fps:=15 x:=1 ', stack)   # the driver arguments are not hard-coded
        self.assertIn('-e BENCH_ODOM_HZ=0 -e BENCH_MAP_M=8', stack)
        self.assertIn('-e LIVE_CAMERA_FRAME=camera_color_optical_frame', stack)
        self.assertIn('gemini2.launch.py color_fps:=15 x:=1 ', self.line(out, '    [camera]'))
        self.assertIn('camera_check.py --color-topic /camera/color/image_raw --depth-topic /camera/depth/image_raw '
                      '--info-topic /camera/depth/camera_info', self.line(out, '    [camera_check, once]'))
        rt = self.line(out, '    [runtime]')
        self.assertIn('--pose-source reference', rt)
        self.assertNotIn('--pose-source estimated', rt)
        # the runner reads the driver's topics: remaps after --ros-args, which comes last (everything after it is rclpy's)
        cmd = rt.split(' > ')[0]
        tail = cmd[cmd.index('--ros-args'):].split()
        self.assertEqual(tail, ['--ros-args', '-r', '/fpga/rgb:=/camera/color/image_raw', '-r', '/fpga/depth:=/camera/depth/image_raw',
                                '-r', '/fpga/camera_info:=/camera/depth/camera_info'])
        self.assertIn('--camera-qos auto --camera-frame camera_color_optical_frame --ros-args', cmd)
        # overrides of the bench pose
        r = self.up(*(self.STATIC + ['--dry-run']), BENCH_ODOM_HZ='30', BENCH_MAP_M='0')
        self.assertIn('bench_pose.py --odom-hz 30 --map-m 0', r.stdout)
        self.assertEqual(self.kv_calls(), [])
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    # ---- 10-16 heartbeat-gap T2 measurement switches: defaults leave every command line as before, except (10-17,
    # owner approved) the driver's Fast DDS profile: unset on the board = the 8 MB shared-memory profile
    SHM8 = 'tools/nano_ros2/fastdds_shm8_localhost.xml'

    def test_t2_switches_default_off(self):
        env = {k: v for k, v in self.env.items() if k != 'LIVE_DRIVER_DDS_PROFILE'}
        r = subprocess.run(['bash', UP] + self.STATIC + ['--dry-run'], env=dict(env, FAKE_USB='1', RTABMAP_DB=''),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=180)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertNotIn('--depth-qos', self.line(out, '    [producer]'))
        stack = self.line(out, '[container stack]')
        self.assertNotIn('PRODUCER_DEPTH_QOS', stack)
        # unset -> the 8 MB profile, to the driver alone (not the start probe, not the producer)
        cam = self.line(out, '    [camera]')
        self.assertIn('env FASTRTPS_DEFAULT_PROFILES_FILE=', cam)
        self.assertIn(self.SHM8 + ' env --default-signal=INT ros2 launch orbbec_camera', cam)
        self.assertNotIn('FASTRTPS', cam.split('probe ')[1])
        self.assertNotIn('FASTRTPS', self.line(out, '    [producer]'))
        self.assertIn('-e LIVE_DRIVER_DDS_PROFILE=' + self.SHM8, stack)
        self.assertEqual(cam.count('FASTRTPS_DEFAULT_PROFILES_FILE'), 1)

    def test_t2_driver_profile_explicit_empty_is_fastdds_defaults(self):
        """LIVE_DRIVER_DDS_PROFILE set to empty = the chain before 10-17 (no profile anywhere)"""
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', LIVE_DRIVER_DDS_PROFILE='')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('FASTRTPS_DEFAULT_PROFILES_FILE', self.line(r.stdout, '    [camera]'))
        self.assertNotIn('LIVE_DRIVER_DDS_PROFILE', self.line(r.stdout, '[container stack]'))

    def test_t2_driver_profile_default_file_is_the_8mb_one(self):
        """the default file exists in the tree and asks for an 8 MB segment (not the 32 MB file that falls back)"""
        x = open(os.path.join(ROOT, self.SHM8)).read()
        self.assertIn('<segment_size>8388608</segment_size>', x)
        self.assertIn('<address>127.0.0.1</address>', x)

    def test_t2_switches_passed(self):
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', PRODUCER_DEPTH_QOS='sensor_data',
                    LIVE_DRIVER_DDS_PROFILE='tools/nano_ros2/fastdds_shm_localhost.xml')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn('--depth-qos sensor_data', self.line(out, '    [producer]'))
        cam = self.line(out, '    [camera]')
        self.assertIn('env FASTRTPS_DEFAULT_PROFILES_FILE=', cam)
        self.assertIn('tools/nano_ros2/fastdds_shm_localhost.xml env --default-signal=INT ros2 launch orbbec_camera', cam)
        # the profile goes to the driver only: the start probe and the producer keep their environment
        self.assertNotIn('FASTRTPS', self.line(out, '    [producer]'))
        self.assertNotIn('FASTRTPS', cam.split('probe ')[1])
        stack = self.line(out, '[container stack]')
        self.assertIn('-e PRODUCER_DEPTH_QOS=sensor_data', stack)
        self.assertIn('-e LIVE_DRIVER_DDS_PROFILE=tools/nano_ros2/fastdds_shm_localhost.xml', stack)
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', PRODUCER_DEPTH_QOS='reliable')
        self.assertIn('--depth-qos reliable', self.line(r.stdout, '    [producer]'))

    # ---- 10-17 shm8 guard (docs/design/HEARTBEAT_GAP_DIG_20261016.md section 11): whenever the driver has a profile,
    # fastdds shm clean before every retry and the driver node's own segment checked against the profile, fail-closed
    def guard_plan(self, **env):
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', **env)
        self.assertEqual(r.returncode, 0, r.stderr)
        lines = r.stdout.splitlines()
        cam = [i for i, l in enumerate(lines) if l.startswith('    [camera]')]
        chk = [i for i, l in enumerate(lines) if l.startswith('    [driver_shm_check, once]')]
        cc = [i for i, l in enumerate(lines) if l.startswith('    [camera_check, once]')]
        self.assertEqual((len(cam), len(cc)), (1, 1))
        return lines, cam[0], chk, cc[0]

    def assert_guard(self, prof, **env):
        lines, cam, chk, cc = self.guard_plan(**env)
        self.assertIn('pause; before every retry: fastdds shm clean, again)', lines[cam])
        self.assertEqual(len(chk), 1)
        self.assertTrue(cam < chk[0] < cc, (cam, chk, cc))   # after the driver is up, before camera_check
        c = lines[chk[0]]
        self.assertIn('python3 tools/car_run/shm_segcheck.py --pid <driver pid> --profile ', c)
        self.assertIn('/' + prof + ' --wait-s 10 --out ', c)
        self.assertIn('driver_shm_check.json > ', c)
        self.assertNotIn('FASTRTPS', c)
        self.assertIn('#   -> ready check: driver_shm', lines[chk[0] + 1])

    def test_shm8_guard_planned_for_the_default_profile(self):
        env = {k: v for k, v in self.env.items() if k != 'LIVE_DRIVER_DDS_PROFILE'}
        r = subprocess.run(['bash', UP] + self.STATIC + ['--dry-run'], env=dict(env, FAKE_USB='1', RTABMAP_DB=''),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=180)
        self.assertEqual(r.returncode, 0, r.stderr)
        saved, self.env = self.env, env
        try:
            self.assert_guard(self.SHM8)
        finally:
            self.env = saved

    def test_shm8_guard_planned_for_an_explicit_profile(self):
        # the 32 MB file: the guard is what catches its silent fallback (the known negative control on the board)
        self.assert_guard('tools/nano_ros2/fastdds_shm_localhost.xml',
                          LIVE_DRIVER_DDS_PROFILE='tools/nano_ros2/fastdds_shm_localhost.xml')
        self.assert_guard(self.SHM8, LIVE_DRIVER_DDS_PROFILE=self.SHM8)

    def test_shm8_guard_absent_with_explicit_empty_profile(self):
        lines, cam, chk, cc = self.guard_plan(LIVE_DRIVER_DDS_PROFILE='')
        self.assertEqual(chk, [])
        self.assertNotIn('shm clean', lines[cam])
        self.assertTrue(lines[cam].endswith('s pause, again)'), lines[cam])
        self.assertFalse(any('driver_shm' in l for l in lines))

    def test_t2_switches_refused(self):
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', PRODUCER_DEPTH_QOS='best_effort')
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn('PRODUCER_DEPTH_QOS', r.stderr)
        r = self.up(*(self.STATIC + ['--dry-run']), FAKE_USB='1', RTABMAP_DB='', LIVE_DRIVER_DDS_PROFILE='tools/nope.xml')
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn('no such file', r.stderr)
        self.assertEqual(self.docker_runs(), [])

    def test_static_board_up_down_with_standins(self):
        """the bench start for real with stand-ins: no RTABMAP_DB needed, no chassis port, car.json says what the pose
        is (not mapping), the stand-in map and the hand-measured mount (owner 10-15); car_down leaves nothing"""
        self.make_node()
        run = os.path.join(self.tmp, 'runs', 'bench1')
        r = self.up(*(self.STATIC + ['--run-dir', run]), FAKE_USB='1', RTABMAP_DB='', CAR_HEIGHT_CELL='8')
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('不是建图定位', r.stdout)
        argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-bench1.json')))['argv']
        self.assertNotIn('/dev/fpga_chassis', argv)
        self.assertEqual(argv.count('--device'), 0)   # 10-17: the camera through the USB root bind, chassis none
        self.assertIn('POSE=static', argv)
        self.assertIn('BENCH_MAP_M=8', argv)
        self.assertIn('HEIGHT_CELL=8', argv)
        car = json.load(open(os.path.join(run, 'car.json')))
        self.assertEqual((car['pose'], car['chassis'], car['nav']), ('static', 'none', 'costmap'))
        self.assertIn('不是建图定位', car['pose_note'])
        self.assertEqual(car['bench']['stand_in_map_m'], 8.0)
        self.assertIn('不是建出的地图', car['bench']['stand_in_map_note'])
        m = car['camera_mount']
        self.assertEqual((m['height_m'], m['height_cell'], m['height_uncertainty_m']), (0.39, 8, 0.01))
        self.assertTrue(m['height_cell_given'])
        self.assertIn('手测', m['height_measurement'])
        self.assertIn('近似相同', m['lens_reference'])
        self.assertEqual(car['live_wiring']['camera_frame'], 'camera_color_optical_frame')
        d = self.down()
        self.assertEqual(d.returncode, 0, d.stdout + d.stderr)
        self.assertEqual(self.containers(), [])
        self.assertEqual(os.listdir(self.state), [])

    def test_commit_file_tree_snapshot(self):
        """a tree deployed with git archive (no .git; bench_line4.sh writes COMMIT): car_up snapshots it and records
        the commit; a tree with neither .git nor COMMIT rolls back with 8"""
        self.make_node()
        dep = os.path.join(self.tmp, 'deployed')
        ign = shutil.ignore_patterns('__pycache__', '*.pyc')
        for d in ('host', 'tools'):
            shutil.copytree(os.path.join(ROOT, d), os.path.join(dep, d), ignore=ign)
        self.assertNotEqual(subprocess.run(['git', '-C', dep, 'rev-parse', '--is-inside-work-tree'], stdout=subprocess.DEVNULL,
                                           stderr=subprocess.DEVNULL).returncode, 0)
        up = os.path.join(dep, 'tools', 'car_run', 'car_up.sh')

        def go(name):
            return subprocess.run(['bash', up] + self.STATIC + ['--run-dir', os.path.join(self.tmp, 'runs', name)],
                                  env=dict(self.env, FAKE_USB='1', RTABMAP_DB=''), stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, universal_newlines=True, timeout=180)
        r = go('nocommit')
        self.assertEqual(r.returncode, 8, r.stdout + r.stderr)
        self.assertIn('neither a git work tree nor a deployed tree with a COMMIT file', r.stderr)
        self.assertEqual(os.listdir(self.state), [])
        open(os.path.join(dep, 'COMMIT'), 'w').write('0123456789abcdef0123456789abcdef01234567\n')
        r = subprocess.run(['bash', up] + self.STATIC + ['--dry-run'], env=dict(self.env, RTABMAP_DB=''),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
        self.assertIn('# source snapshot: no git work tree -- the deployed tree of %s/COMMIT (0123456789abcdef' % dep, r.stdout)
        r = go('withcommit')
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        run = os.path.join(self.tmp, 'runs', 'withcommit')
        self.assertEqual(open(os.path.join(run, 'head.txt')).read().strip(), '0123456789abcdef0123456789abcdef01234567')
        self.assertIn('deployed archive', open(os.path.join(run, 'source_kind.txt')).read())
        self.assertEqual(os.path.getsize(os.path.join(run, 'dirty.txt')), 0)
        self.assertTrue(os.path.isfile(os.path.join(run, 'src', 'tools', 'car_run', 'car_inner.sh')))
        self.assertTrue(os.path.isfile(os.path.join(run, 'src', 'host', 'geo_producer.py')))
        self.assertEqual(self.down().returncode, 0)
        self.assertEqual(os.listdir(self.state), [])


class TestFpgaProfile(Base):
    """Go-live 2026-10-26: tools/car_run/fpga_profiles.env is the one place. Board default v3fc2 (10-26 board short test
    passed); FPGA_PROFILE=v3fc2 | v3fc2_166 | v3fc = the producer fuses on the KV260 PL (v3fc3_187 paused: warns, runs v3fc2)
    (--rolling-backend remote_pl) and the precheck expects that profile's BUILD_ID; a2 = the old producer line and 0x00da5041; an unknown name warns and uses
    the default (owner 10-26: no new refusals); a KV260 serving another REGISTERED build is used (its id recorded; a2 => Nano CPU fusion)."""
    setUp = TestLiveBoard.setUp
    line = TestLiveBoard.line
    docker_runs = TestLiveBoard.docker_runs
    make_node = TestLiveBoard.make_node

    @staticmethod
    def profiles():
        kv = {}
        for line in open(os.path.join(ROOT, 'tools/car_run/fpga_profiles.env')):
            line = line.strip()
            if line and not line.startswith('#'):
                k, v = line.split('=', 1)
                kv[k] = v
        return kv

    def plan(self, **env):
        e = dict(self.env, FAKE_USB='1', **env)
        if 'FPGA_PROFILE' not in env:
            e.pop('FPGA_PROFILE')
        r = subprocess.run(['bash', UP, 'live', '--dry-run'], env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=180)
        return r

    def producer(self, out):   # run directory names carry the second: normalised
        return re.sub(r'[0-9][0-9-]*-live', 'RUN-live', next(l for l in out.splitlines() if 'host.ros_geo_producer' in l))

    def test_default_v3fc2_and_a2_rollback(self):
        """10-26: the default is v3fc2 (board short test passed); a2 (rollback) / v3fc2_166 (same BUILD_ID
        0x03c087c9, told apart by the app name) / v3fc are one FPGA_PROFILE=... away; v3fc3_187 is paused
        (its 187.5 MHz clock ran at 200 MHz on the board): it warns and runs v3fc2."""
        kv = self.profiles()
        self.assertEqual(kv['FPGA_PROFILE_DEFAULT'], 'm2_166')   # SMOKE2 11-03 (v3fc2 10-26 .. 11-03)
        self.assertEqual(kv['V3FC3_187_USE'], 'v3fc2')
        self.assertEqual((kv['V3FC2_APP'], kv['V3FC2_BUILD_ID']), ('m7_pl_v3fc2', '0x03c087c9'))
        self.assertEqual((kv['V3FC2_166_APP'], kv['V3FC2_166_BUILD_ID']), ('m7_pl_v3fc2_166', '0x03c087c9'))
        self.assertEqual((kv['V3FC3_187_APP'], kv['V3FC3_187_BUILD_ID']), ('m7_pl_v3fc3', '0x042a646f'))
        self.assertEqual((kv['V3FC_APP'], kv['V3FC_BUILD_ID']), ('m7_pl_v3fc', '0x001256b1'))
        d, a2 = self.plan(), self.plan(FPGA_PROFILE='a2')
        v3, v166, v187, v3fc = (self.plan(FPGA_PROFILE=n) for n in ('v3fc2', 'v3fc2_166', 'v3fc3_187', 'v3fc'))
        rcs = [r.returncode for r in (d, a2, v3, v166, v187, v3fc)]
        self.assertEqual(rcs, [0] * 6, d.stderr + a2.stderr + v3.stderr + v166.stderr + v187.stderr + v3fc.stderr)
        pv, pa = self.producer(v3.stdout), self.producer(a2.stdout)
        self.assertNotEqual(self.producer(d.stdout), pv)                         # default is no longer v3fc2 (SMOKE2)
        self.assertEqual(self.producer(d.stdout), self.producer(self.plan(FPGA_PROFILE='m2_166').stdout))
        self.assertIn('--rolling-backend remote_pl', pv)
        self.assertIn('--kv-expect-build-id 0x03c087c9', pv)
        self.assertIn('--expect-build-id 0x03c087c9', self.line(v3.stdout, '[kv precheck]'))
        self.assertIn('-e ROLLING_BACKEND=remote_pl', self.line(v3.stdout, '[container stack]'))
        self.assertEqual(self.producer(v166.stdout), pv)                         # same id, same Nano side
        # v3fc3_187 paused (10-26): warns and runs v3fc2 (its id, its Nano side)
        self.assertIn("'v3fc3_187' is paused", v187.stderr)
        self.assertEqual(self.producer(v187.stdout), pv)
        self.assertIn('--expect-build-id 0x03c087c9', self.line(v187.stdout, '[kv precheck]'))
        self.assertIn('--kv-expect-build-id 0x001256b1', self.producer(v3fc.stdout))
        self.assertNotIn('rolling-backend', pa)
        self.assertNotIn('ROLLING_BACKEND', a2.stdout)
        self.assertIn('--kv-expect-build-id 0x00da5041', pa)
        self.assertIn('--expect-build-id 0x00da5041', self.line(a2.stdout, '[kv precheck]'))
        # the producer lines differ in exactly the profile's two items
        self.assertEqual(pv.replace(' --rolling-backend remote_pl', '').replace('0x03c087c9', '0x00da5041'), pa)

    def test_profile_of_build_shared_id(self):
        r = subprocess.run(['bash', '-c', 'source "$1"; for b in 0x03c087c9 0x001256b1 0x00da5041 0x0 0x042a646f; do '
                            'fpga_profile_of_build $b || echo none; done; fpga_profile v3fc2_166; '
                            'echo "$FPGA_PROFILE $FPGA_APP $FPGA_BUILD_ID $FPGA_NANO_ROLLING"; '
                            'fpga_profile v3fc3_187; '
                            'echo "$FPGA_PROFILE $FPGA_APP $FPGA_BUILD_ID $FPGA_NANO_ROLLING"',
                            '_', os.path.join(ROOT, 'tools/car_run/fpga_profile.sh')],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.split('\n')[:7], ['v3fc2', 'v3fc', 'a2', 'none', 'v3fc3_187',
                                                    'v3fc2_166 m7_pl_v3fc2_166 0x03c087c9 remote_pl',
                                                    'v3fc2 m7_pl_v3fc2 0x03c087c9 remote_pl'])

    def test_registered_other_build_is_used(self):
        """v3fc2 requested, KV260 serves the a2 build: start goes on, id recorded, producer on the Nano CPU"""
        self.make_node()
        run = os.path.join(self.tmp, 'runs', 'reg_a2')
        e = dict(self.env)
        e.update(FPGA_PROFILE='v3fc2', FAKE_USB='1', FAKE_KV='mismatch', FAKE_KV_ACTUAL='0x00da5041')
        r = subprocess.run(['bash', UP, 'live', '--run-dir', run], env=e, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, universal_newlines=True, timeout=180)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('KV260 serves BUILD_ID 0x00da5041 (registered profile a2', r.stderr)
        argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-reg_a2.json')))['argv']
        self.assertIn('KV_BUILD_ID=0x00da5041', argv)
        self.assertIn('ROLLING_BACKEND=cpu', argv)
        self.assertNotIn('ROLLING_BACKEND=remote_pl', argv)
        self.assertIn('accepted: KV260 serves registered BUILD_ID 0x00da5041', open(os.path.join(run, 'kv_precheck.txt')).read())
        self.assertEqual(self.down().returncode, 0)

    def test_unknown_profile_warns_and_uses_default(self):
        r, d = self.plan(FPGA_PROFILE='v3x'), self.plan()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("unknown FPGA_PROFILE 'v3x'", r.stderr)
        self.assertEqual(self.producer(r.stdout), self.producer(d.stdout))


class TestGroundCalibPlan(Base):
    """10-15 start-up ground self-calibration (tools/car_run/ground_calib.py): CAR_GROUND_CALIB auto (board default) |
    check | off reaches the stack and the plan; the step sits after camera_check and before the mount TF (the producer
    bakes the mount into its calibration_id on its first frame); off = the inner plan of before (no step); the mapping
    run stores the mount beside its database, localization compares against it; car.json filled after READY."""
    setUp = TestLiveBoard.setUp
    line = TestLiveBoard.line
    make_node = TestLiveBoard.make_node

    def inner(self, out):
        return [l for l in out.splitlines() if l.startswith('    ')]

    def names(self, out):
        return [l.split(']')[0].strip().lstrip('[').split(',')[0] for l in self.inner(out) if l.startswith('    [')]

    @staticmethod
    def norm(lines):   # the rtabmap run time stamp differs between two dry runs
        import re
        return [re.sub(r'\d{8}_\d{6}', '<T>', l) for l in lines]

    def test_auto_default_plan(self):
        for args in (['live'], TestLiveBoard.STATIC, ['live', '--pose', 'odom', '--nav', 'costmap']):
            r = self.up(*(args + ['--dry-run']), FAKE_USB='1')
            self.assertEqual(r.returncode, 0, r.stderr)
            stack = self.line(r.stdout, '[container stack]')
            self.assertIn('-e CAM_MOUNT_TF=0.1 0 0.39 -0.564720585 0.564720585 -0.425547484 0.425547484 '
                          '-e GROUND_CALIB=auto -e HEIGHT_CELL_GIVEN= -e ROBOT_HEIGHT=0.31 ', stack)
            self.assertIn('# ground calib: CAR_GROUND_CALIB=auto (configured mount 0.39 m / 16.0 deg / roll 0; '
                          'height_cell derived)', r.stdout)
            n = self.names(r.stdout)
            self.assertLess(n.index('camera_check'), n.index('ground_calib'), n)
            self.assertLess(n.index('ground_calib'), n.index('mount_tf'), n)
            # the camera-above-band hard check (owner 10-16) sits after the ground calibration, before the mount TF
            out = r.stdout
            self.assertLess(out.index('    [ground_calib, once]'), out.index('    # camera above band: '))
            self.assertLess(out.index('ready check: camera_above_band camera 0.3900 m >= obstacle band top 0.3600 m'),
                            out.index('    [mount_tf]'))
            self.assertLess(n.index('mount_tf'), n.index('producer'), n)
            run = r.stdout.split('# run directory: ')[1].split()[0]
            gc = self.line(r.stdout, '    [ground_calib, once]')
            self.assertIn('python3 tools/car_run/ground_calib.py --mode auto --height-m 0.39 --pitch-deg 16.0 '
                          '--depth-topic /camera/depth/image_raw --info-topic /camera/depth/camera_info --frames 10 '
                          '--timeout-s 15 --save-depth %s/ground_calib_depth.npz --out %s/ground_calib.json'
                          % (run, run), gc)
            self.assertNotIn('--height-cell', gc)
            self.assertIn('FITTED mount of ground_calib.json', r.stdout)
            # the plan prints the configured mount (the fitted one exists only on the board)
            self.assertIn('--z 0.39 --qx -0.564720585 --qy 0.564720585', self.line(r.stdout, '    [mount_tf]'))
            self.assertIn('--height-cell 8 ', self.line(r.stdout, '    [producer]'))

    def test_check_and_given_height_cell(self):
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_GROUND_CALIB='check', CAR_HEIGHT_CELL='8')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('-e GROUND_CALIB=check -e HEIGHT_CELL_GIVEN=8 ', self.line(r.stdout, '[container stack]'))
        gc = self.line(r.stdout, '    [ground_calib, once]')
        self.assertIn('--mode check --height-m 0.39 --pitch-deg 16.0 --height-cell 8 ', gc)
        self.assertIn('configured mount is kept when the fit agrees within 1 deg / 1 cm', r.stdout)

    def test_off_is_the_plan_of_before(self):
        for args in (['live'], TestLiveBoard.STATIC, ['live', '--pose', 'map', '--nav', 'costmap']):
            rd = ['--dry-run', '--run-dir', os.path.join(self.tmp, 'runs', 'plan')]   # a dry run creates nothing
            off = self.up(*(args + rd), FAKE_USB='1', CAR_GROUND_CALIB='off')
            auto = self.up(*(args + rd), FAKE_USB='1')
            self.assertEqual((off.returncode, auto.returncode), (0, 0), off.stderr + auto.stderr)
            self.assertIn('-e GROUND_CALIB=off ', self.line(off.stdout, '[container stack]'))
            self.assertIn('CAR_GROUND_CALIB=off', off.stdout)
            self.assertNotIn('ground_calib', '\n'.join(self.inner(off.stdout)))
            self.assertIn('ready check: mount_tf height 0.39 m pitch 16.0 deg height_cell 8\n', off.stdout)
            # auto = off + the step (its lines, the stored-beside-the-map line) + the mount_tf note; nothing else
            strip = [l.replace(' (ground_calib auto: values of ground_calib.json)', '') for l in self.inner(auto.stdout)
                     if 'ground_calib' not in l or l.startswith('    #   -> ready check: mount_tf')]
            self.assertEqual(self.norm(strip), self.norm(self.inner(off.stdout)), args)

    def test_bad_value_refused(self):
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_GROUND_CALIB='on')
        self.assertEqual(r.returncode, 2)
        self.assertIn("CAR_GROUND_CALIB 只能是 auto|off|check（给的是 'on'）", r.stderr)

    def test_host_presets_untouched(self):
        for preset in ('sim', 'replay'):
            r = self.up(preset, '--dry-run', CAR_ARCH='x86_64')
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertNotIn('ground_calib', r.stdout)
            self.assertNotIn('GROUND_CALIB', r.stdout)
            # owner 10-16: the car body height is the live car's only; host presets keep 0.25, no hard check
            self.assertIn('-p robot_height_m:=0.25 ', self.line(r.stdout, '    [bridge]'))
            self.assertNotIn('ROBOT_HEIGHT', r.stdout)
            self.assertNotIn('camera above band', r.stdout)
            self.assertNotIn('camera_above_band', r.stdout)

    def test_camera_above_band_in_stack(self):
        """owner 10-16: car_inner.sh refuses (fail channel, exit 3, the reason in the FAIL line) when the camera height in
        use is below ROBOT_HEIGHT + 0.05 + 0.005 m -- run here on car_inner.sh directly (car_up refuses the configured
        value earlier), with the height the stack would hold after the ground calibration."""
        base = dict(os.environ, CAMERA='live', POSE='static', NAV='costmap', DETECTOR='yolo', CHASSIS='none',
                    DRY_RUN='1', O=os.path.join(self.tmp, 'o'), SRCD=ROOT, CAM_PITCH_DEG='30', HEIGHT_CELL='8',
                    GROUND_CALIB='off', ROBOT_HEIGHT='0.31')
        inner = os.path.join(ROOT, 'tools', 'car_run', 'car_inner.sh')
        for cam, rc in (('0.3788', 0), ('0.365', 0), ('0.364', 3), ('0.36', 3)):
            env = dict(base, CAM_HEIGHT=cam, CAM_MOUNT_TF='0.1 0 %s 0 0 0 1' % cam)
            r = subprocess.run(['bash', inner], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               universal_newlines=True, timeout=60)
            self.assertEqual(r.returncode, rc, (cam, r.stderr[-2000:]))
            if rc:
                self.assertIn('FAIL camera above band refused the start: 相机太低：障碍带上沿 0.360 m', r.stderr)
                self.assertNotIn('[mount_tf]', r.stdout)      # refused before the mount TF
            else:
                self.assertIn('ready check: camera_above_band', r.stdout)
                self.assertIn('-p robot_height_m:=0.31 ', r.stdout)

    def test_map_stores_and_loc_compares(self):
        r = self.up('live', '--pose', 'map', '--nav', 'costmap', '--dry-run', FAKE_USB='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        run = r.stdout.split('# run directory: ')[1].split()[0]
        self.assertRegex(r.stdout, r'# ground_calib: %s/ground_calib.json -> %s/\d{8}_\d{6}\.ground_calib\.json '
                                   r'\(beside the new database\)' % (run, self.env['RTABMAP_DB_DIR']))
        # loc without a stored mount beside the pinned database: said, not compared, nothing mounted
        sidecar = os.path.join(self.tmp, 'pinned.ground_calib.json')
        r = self.up('live', '--dry-run', FAKE_USB='1')
        self.assertIn('# ground calib vs map: %s (absent: not compared)' % sidecar, r.stdout)
        self.assertNotIn('GROUND_CALIB_MAP', r.stdout)
        with open(sidecar, 'w') as f:
            f.write('{}')
        r = self.up('live', '--dry-run', FAKE_USB='1')
        stack = self.line(r.stdout, '[container stack]')
        self.assertIn('-e GROUND_CALIB_MAP=%s ' % sidecar, stack)
        self.assertIn('-v %s:%s:ro ' % (sidecar, sidecar), stack)
        self.assertIn('ground_calib.py --compare %s ' % sidecar, r.stdout)
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_GROUND_CALIB='off')
        self.assertNotIn('GROUND_CALIB_MAP', r.stdout)
        self.assertNotIn('--compare', r.stdout)

    def test_car_json_filled_after_ready(self):
        self.make_node()
        gcj = os.path.join(self.tmp, 'gc.json')
        json.dump({'verdict': 'adopt', 'adopted': {'height_m': 0.3612, 'pitch_deg': 27.4, 'roll_deg': -0.6, 'source': 'fit'},
                   'fitted': {'height_m': 0.3612, 'pitch_deg': 27.4, 'roll_deg': -0.6}, 'height_cell': 8,
                   'mount_tf': '0.1 0 0.3612 1 2 3 4'}, open(gcj, 'w'))
        run = os.path.join(self.tmp, 'runs', 'gc1')
        r = self.up('live', '--run-dir', run, FAKE_USB='1', FAKE_GC=gcj)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        m = json.load(open(os.path.join(run, 'car.json')))['camera_mount']
        self.assertEqual((m['height_m'], m['pitch_deg']), (0.39, 16.0))           # the configured values stay
        g = m['ground_calib']
        self.assertEqual((g['mode'], g['verdict'], g['adopted_source'], g['adopted_pitch_deg'], g['adopted_height_cell']),
                         ('auto', 'adopt', 'fit', 27.4, 8))
        self.assertEqual(g['adopted_base_link_to_camera_xyz_q'], '0.1 0 0.3612 1 2 3 4')
        # 10-28: an adopted fit becomes the last good mount in the state dir ...
        lg = json.load(open(os.path.join(self.state, 'ground_calib_last_good.json')))
        self.assertEqual((lg['height_m'], lg['pitch_deg'], lg['roll_deg'], lg['run']), (0.3612, 27.4, -0.6, 'gc1'))
        self.assertEqual(self.down().returncode, 0)
        # ... copied into the next run's dir (the stack's --last-good); a fallback result does not overwrite it
        json.dump({'verdict': 'adopt', 'adopted': {'height_m': 0.39, 'pitch_deg': 7.0, 'roll_deg': 0.0,
                                                   'source': 'last_good'}, 'height_cell': 8, 'alert': 'x',
                   'mount_tf': '0.1 0 0.39 1 2 3 4'}, open(gcj, 'w'))
        run = os.path.join(self.tmp, 'runs', 'gc1b')
        r = self.up('live', '--run-dir', run, FAKE_USB='1', FAKE_GC=gcj)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(json.load(open(os.path.join(run, 'ground_calib_last_good_in.json')))['pitch_deg'], 27.4)
        self.assertEqual(json.load(open(os.path.join(self.state, 'ground_calib_last_good.json')))['run'], 'gc1')
        self.assertEqual(json.load(open(os.path.join(run, 'car.json')))['camera_mount']['ground_calib']['alert'], 'x')
        self.assertEqual(self.down().returncode, 0)
        run = os.path.join(self.tmp, 'runs', 'gc2')
        r = self.up('live', '--run-dir', run, FAKE_USB='1', CAR_GROUND_CALIB='off')
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        g = json.load(open(os.path.join(run, 'car.json')))['camera_mount']['ground_calib']
        self.assertEqual(g['mode'], 'off')
        self.assertNotIn('verdict', g)
        self.assertEqual(self.down().returncode, 0)


class TestCameraAboveBand(unittest.TestCase):
    """tools/car_run/car_mount.camera_above_band (owner 10-16): camera >= robot height + 0.05 (band top) + 0.005 m."""

    def setUp(self):
        import sys
        sys.path.insert(0, ROOT)
        from tools.car_run import car_mount
        self.cm = car_mount

    def test_constants(self):
        self.assertEqual((self.cm.CAR_ROBOT_HEIGHT_M, self.cm.BAND_CLEARANCE_M, self.cm.CAMERA_MARGIN_M),
                         (0.31, 0.05, 0.005))

    def test_exactly_equal_passes(self):
        # 0.31 + 0.05 + 0.005 is 0.36500000000000005 in floats: compared in micrometres, 0.365 passes
        self.assertTrue(self.cm.camera_above_band(0.365, 0.31)[0])
        self.assertTrue(self.cm.camera_above_band(0.305, 0.25)[0])
        self.assertTrue(self.cm.camera_above_band(0.375, 0.32)[0])

    def test_boundary(self):
        self.assertTrue(self.cm.camera_above_band(0.365001, 0.31)[0])
        self.assertFalse(self.cm.camera_above_band(0.364999, 0.31)[0])
        self.assertTrue(self.cm.camera_above_band(0.375, 0.31)[0])     # the hand-measured mount
        self.assertTrue(self.cm.camera_above_band(0.3788, 0.31)[0])    # the start-up fit

    def test_margin_short(self):
        ok, why = self.cm.camera_above_band(0.364, 0.31)   # above the band top 0.36 but 1 mm short of the 5 mm margin
        self.assertFalse(ok)
        self.assertIn('相机太低：障碍带上沿 0.360 m', why)
        self.assertIn('要求镜头 >= 0.3650 m', why)
        self.assertIn('前方无法判空', why)
        self.assertFalse(self.cm.camera_above_band(0.36, 0.31)[0])     # band top at the lens
        self.assertFalse(self.cm.camera_above_band(0.35, 0.31)[0])     # band top above the lens

    def test_bad_input(self):
        for cam, rh in ((float('nan'), 0.31), (0.375, float('inf')), (0.375, 0.0), (-0.1, 0.31), (True, 0.31),
                        ('0.375', 0.31)):
            self.assertRaises(ValueError, self.cm.camera_above_band, cam, rh)

    def test_cli(self):
        cli = os.path.join(ROOT, 'tools', 'car_run', 'car_mount.py')
        run = lambda *a: subprocess.run(['python3', cli] + list(a), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                        universal_newlines=True, timeout=60)
        r = run('--height-m', '0.365', '--robot-height-m', '0.31')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('band top 0.3600 m', r.stdout)
        r = run('--height-m', '0.36', '--robot-height-m', '0.31')
        self.assertEqual(r.returncode, 2)
        self.assertIn('相机太低', r.stderr)
        r = run('--height-m', '0.375', '--robot-height-m', 'nan')
        self.assertEqual(r.returncode, 2)
        self.assertIn('camera above band: robot_height_m must be a finite number', r.stderr)
        r = run('--height-m', '0.375')                          # the mount mode still needs its pitch
        self.assertEqual(r.returncode, 2)
        self.assertIn('--pitch-deg is required', r.stderr)


class TestCarMount(unittest.TestCase):
    """tools/car_run/car_mount.py: height_cell from the camera height (the producer's own first-frame block check) and
    the base_link -> camera transform; common.py's replay/host mount stays 0.35 m / 27 deg."""

    def test_height_cell_rule_and_common_default_unchanged(self):
        import sys
        sys.path.insert(0, ROOT)
        from tools.car_run.car_mount import mount
        from tools.car_geo_producer.common import MOUNT_PITCH_DEG, MOUNT_XYZ, base_camera
        self.assertEqual((MOUNT_XYZ, MOUNT_PITCH_DEG), ((0.10, 0.0, 0.35), 27.0))
        for h in (0.31, 0.32, 0.33, 0.34):
            self.assertEqual(mount(h, 30.0)[0], 7, h)
        for h in (0.351, 0.36, 0.37, 0.38, 0.39):
            self.assertEqual(mount(h, 30.0)[0], 8, h)
        self.assertRaises(ValueError, mount, 0.40, 30.0)
        self.assertRaises(ValueError, mount, 0.36, 30.0, 7)
        hc, xyz, q = mount(0.36, 30.0)
        self.assertEqual(xyz, (0.10, 0.0, 0.36))
        # the quaternion is the rotation of base_camera(0.36 m, 30 deg): rotate the optical z axis (forward) with it
        import numpy as np
        x, y, z, w = q
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(R, base_camera((0.10, 0.0, 0.36), 30.0)[:3, :3], atol=1e-9)
        fwd = R @ np.array([0.0, 0.0, 1.0])
        self.assertAlmostEqual(float(np.degrees(np.arcsin(-fwd[2]))), 30.0, places=6)   # 30 deg down


class TestStartRetry(unittest.TestCase):
    """car_start_retry (the Orbbec driver's start retry): a start is good only when the probe sees DATA. Board 10-15
    (l_chain32): the driver printed its "depth Frame" profile line and then sent no depth frame -- the rig's grep
    counted that as good. Here every stand-in start prints that line; only starts from the GOOD-th on send "data"."""
    # stand-in driver: like ros2 launch, a parent with one child; prints the profile line on EVERY start, and from the
    # GOOD-th start on also "sends depth" (touches data_<n>); EXIT=1: exits at once without data
    DRIVER = textwrap.dedent('''\
        import os, subprocess, sys, time
        d = sys.argv[1]; good = int(sys.argv[2]); quit_early = len(sys.argv) > 3
        n = len([f for f in os.listdir(d) if f.startswith('start_')]) + 1
        child = subprocess.Popen(['sleep', '300'])
        open(os.path.join(d, 'start_%d' % n), 'w').write('%d %d' % (os.getpid(), child.pid))
        print('try', n, flush=True)
        print('[camera] depth Frame - Width 640 Height 400 Fps 30 Format Y16', flush=True)
        if quit_early:
            child.kill(); sys.exit(1)
        if n >= good:
            open(os.path.join(d, 'data_%d' % n), 'w').write('depth')
        while True:
            time.sleep(1)
        ''')
    # stand-in depth probe: exit 0 once the current start's data file exists, 1 after the timeout
    PROBE = textwrap.dedent('''\
        import os, sys, time
        d = sys.argv[1]; end = time.time() + float(sys.argv[2])
        while time.time() < end:
            n = len([f for f in os.listdir(d) if f.startswith('start_')])
            if n and os.path.exists(os.path.join(d, 'data_%d' % n)):
                print('received 3'); sys.exit(0)
            time.sleep(0.05)
        print('received 0'); sys.exit(1)
        ''')

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='carretry_')
        write_exec(os.path.join(self.tmp, 'drv.py'), self.DRIVER)
        write_exec(os.path.join(self.tmp, 'probe.py'), self.PROBE)

    def tearDown(self):
        for f in os.listdir(self.tmp):
            if f.startswith('start_'):
                for pid in map(int, open(os.path.join(self.tmp, f)).read().split()):
                    kill_if_alive(pid)
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_retry(self, tries, good, extra='', probe=True, before=''):
        t = self.tmp
        script = ('source %s\n' % COMMON
                  + ('CAR_RETRY_PROBE=(python3 %s/probe.py %s 1.5)\n' % (t, t) if probe else '')
                  + before
                  + 'CAR_MARKS_FILE=%s/marks.txt car_start_retry %d 1 0 %s/driver.log python3 %s/drv.py %s %d %s\n'
                  % (t, tries, t, t, t, good, extra)
                  + 'rc=$?; echo "rc=$rc pid=$CAR_RETRY_PID try=$CAR_RETRY_TRY"\n'
                  '[ -n "$CAR_RETRY_PID" ] && kill -KILL $(pgrep -P $CAR_RETRY_PID) $CAR_RETRY_PID\nexit 0\n')
        r = subprocess.run(['bash', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=120)
        self.assertEqual(r.returncode, 0, r.stderr)
        starts = {}
        for f in os.listdir(t):
            if f.startswith('start_'):
                starts[int(f[6:])] = [int(x) for x in open(os.path.join(t, f)).read().split()]
        mp = os.path.join(t, 'marks.txt')
        marks = [l.split(' ', 1)[1] for l in open(mp).read().splitlines()] if os.path.exists(mp) else []
        return r.stdout.strip(), starts, marks

    def test_profile_line_without_data_is_a_failed_start(self):
        out, starts, marks = self.run_retry(5, 3)
        self.assertEqual(marks, ['camera driver try=1 depth_stream=0 probe_rc=1', 'camera driver try=2 depth_stream=0 probe_rc=1',
                                 'camera driver try=3 depth_stream=1 probe_rc=0'])
        self.assertTrue(out.startswith('rc=0 pid=%d try=3' % starts[3][0]), out)
        self.assertEqual(sorted(starts), [1, 2, 3])
        for n in (1, 2):   # printed "depth Frame", sent nothing: judged failed, process and child stopped, logs kept
            self.assertFalse(alive(starts[n][0]) or alive(starts[n][1]), n)
            self.assertIn('depth Frame', open(os.path.join(self.tmp, 'driver_failed_start_%d.log' % n)).read())
            self.assertIn('received 0', open(os.path.join(self.tmp, 'driver_probe_%d.log' % n)).read())
        self.assertIn('received 3', open(os.path.join(self.tmp, 'driver_probe_3.log')).read())
        self.assertFalse(os.path.exists(os.path.join(self.tmp, 'driver_failed_start_3.log')))

    def test_all_tries_fail(self):
        out, starts, marks = self.run_retry(2, 99)
        self.assertTrue(out.startswith('rc=1 pid= try='), out)
        self.assertEqual(marks, ['camera driver try=1 depth_stream=0 probe_rc=1', 'camera driver try=2 depth_stream=0 probe_rc=1'])
        for n in (1, 2):
            self.assertFalse(alive(starts[n][0]) or alive(starts[n][1]), n)
            self.assertTrue(os.path.exists(os.path.join(self.tmp, 'driver_failed_start_%d.log' % n)))

    def test_driver_exiting_is_a_failed_start(self):
        out, starts, marks = self.run_retry(2, 1, extra='quit')
        self.assertTrue(out.startswith('rc=1'), out)
        self.assertEqual(marks, ['camera driver try=1 depth_stream=0 probe_rc=driver_exited',
                                 'camera driver try=2 depth_stream=0 probe_rc=driver_exited'])

    def test_probe_required(self):
        out, starts, marks = self.run_retry(2, 1, probe=False)
        self.assertTrue(out.startswith('rc=2'), out)
        self.assertEqual(starts, {})

    def test_before_hook_runs_before_every_retry_not_the_first(self):
        """CAR_RETRY_BEFORE (10-17: fastdds shm clean): after a failed try's stop and pause, before the next start;
        not before try 1; its output kept per try; a failing hook does not stop the retry"""
        t = self.tmp
        hook = 'CAR_RETRY_BEFORE=(bash -c "echo cleaning; ls %s | grep -c ^start_; exit 7")\n' % t
        out, starts, marks = self.run_retry(5, 3, before=hook)
        self.assertEqual(marks, ['camera driver try=1 depth_stream=0 probe_rc=1', 'camera driver before try=2 rc=7',
                                 'camera driver try=2 depth_stream=0 probe_rc=1', 'camera driver before try=3 rc=7',
                                 'camera driver try=3 depth_stream=1 probe_rc=0'])
        self.assertTrue(out.startswith('rc=0 pid=%d try=3' % starts[3][0]), out)
        self.assertFalse(os.path.exists(os.path.join(t, 'driver_before_1.log')))
        for n in (2, 3):   # it saw the starts before it (n - 1) and none of its own: it ran before try n
            self.assertEqual(open(os.path.join(t, 'driver_before_%d.log' % n)).read().split(), ['cleaning', str(n - 1)])
            self.assertFalse(alive(starts[n - 1][0]) or alive(starts[n - 1][1]), n)

    def test_before_hook_not_run_on_a_first_try_success(self):
        out, starts, marks = self.run_retry(3, 1, before='CAR_RETRY_BEFORE=(touch %s/hook_ran)\n' % self.tmp)
        self.assertEqual(marks, ['camera driver try=1 depth_stream=1 probe_rc=0'])
        self.assertFalse(os.path.exists(os.path.join(self.tmp, 'hook_ran')))

    def test_depth_probe_module(self):
        """the real probe imports without ROS (rclpy only inside main) and reports a missing rclpy as 2"""
        import sys
        sys.path.insert(0, ROOT)
        from tools.car_run.depth_probe import nonzero_share
        self.assertEqual(nonzero_share(b'\x00\x00\x01\x02'), 0.5)
        self.assertIsNone(nonzero_share(b''))
        try:
            import rclpy  # noqa: F401
        except ImportError:
            from tools.car_run.depth_probe import main
            self.assertEqual(main(['--topic', '/camera/depth/image_raw', '--timeout-s', '0.1']), 2)

    def test_grid_probe_module(self):
        """the rolling-grid probe (board nav2, rolling): imports without ROS, known share of an OccupancyGrid's cells,
        a missing rclpy is 2"""
        import sys
        sys.path.insert(0, ROOT)
        from tools.car_run.grid_probe import known_share
        self.assertEqual(known_share([-1, -1, 0, 100]), 0.5)
        self.assertIsNone(known_share([]))
        try:
            import rclpy  # noqa: F401
        except ImportError:
            from tools.car_run.grid_probe import main
            self.assertEqual(main(['--timeout-s', '0.1']), 2)


class TestShmSegcheck(unittest.TestCase):
    """tools/car_run/shm_segcheck.py (10-17 shm8 guard) on a stand-in /proc and /dev/shm: the driver node's OWN segment
    = the _el lock files a descendant of the launch process holds and the launch process does not (the node inherits
    the launch's fds); the largest must be >= the profile's segment_size. Board sizes: 8 MB segment file 8413728 B,
    default 549408 B, 32 MB 33579552 B. Fail-closed: no pid / no child / no own segment / no segment_size = 2."""
    SHM8 = os.path.join(ROOT, 'tools', 'nano_ros2', 'fastdds_shm8_localhost.xml')
    SHM32 = os.path.join(ROOT, 'tools', 'nano_ros2', 'fastdds_shm_localhost.xml')

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='segcheck_')
        self.proc = os.path.join(self.tmp, 'proc')
        self.shm = os.path.join(self.tmp, 'shm')
        os.makedirs(self.proc)
        os.makedirs(self.shm)
        import sys
        sys.path.insert(0, ROOT)
        from tools.car_run import shm_segcheck
        self.m = shm_segcheck

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def seg(self, sid, size):
        name = 'fastrtps_%016x' % sid
        with open(os.path.join(self.shm, name), 'wb') as f:
            f.truncate(size)
        open(os.path.join(self.shm, name + '_el'), 'w').close()
        return name

    def pid(self, pid, ppid, comm, segs, other_fds=()):
        d = os.path.join(self.proc, str(pid))
        os.makedirs(os.path.join(d, 'fd'))
        with open(os.path.join(d, 'stat'), 'w') as f:
            f.write('%d (%s) S %d 1 1 0 -1\n' % (pid, comm, ppid))
        with open(os.path.join(d, 'cmdline'), 'wb') as f:
            f.write(comm.encode() + b'\0--ros-args\0')
        fds = ['/dev/null', 'socket:[123]'] + list(other_fds)
        # the board's link target is /dev/shm/<name>_el; matched by basename (the stand-in shm root may be anywhere)
        fds += ['/dev/shm/%s_el' % n for n in segs]
        for i, target in enumerate(fds):
            os.symlink(target, os.path.join(d, 'fd', str(i)))

    def run_main(self, pid, profile, *extra):
        out = os.path.join(self.tmp, 'res.json')
        rc = self.m.main(['--pid', str(pid), '--profile', profile, '--proc', self.proc, '--shm', self.shm,
                          '--out', out] + list(extra))
        return rc, json.load(open(out))

    def test_8mb_segment_passes(self):
        """board B1b: the launch process 8 MB + default; the node inherits both and has its own 8 MB + default"""
        la, lb = self.seg(1, 8413728), self.seg(2, 549408)
        na, nb = self.seg(3, 8413728), self.seg(4, 549408)
        self.pid(100, 1, 'python3', [la, lb])
        self.pid(101, 100, 'component_container', [la, lb, na, nb], other_fds=['/dev/shm/fastrtps_port7411_el'])
        self.pid(102, 1, 'ros_geo_producer', [self.seg(5, 549408)])
        rc, res = self.run_main(100, self.SHM8)
        self.assertEqual(rc, 0, res)
        self.assertTrue(res['passed'])
        self.assertEqual((res['required_bytes'], res['largest_own_bytes'], res['own_segment_bytes']),
                         (8388608, 8413728, [549408, 8413728]))
        self.assertEqual([c['pid'] for c in res['children']], [101])
        self.assertEqual(sorted(res['launch_segments'].values()), [549408, 8413728])

    def test_32mb_fallback_fails(self):
        """board 10-16 B1 void (the known answer): the launch process got its 32 MB segment, the node fell back to the
        default -- counted without subtracting the launch's inherited fds, the node would seem to hold 32 MB"""
        big = self.seg(1, 33579552)
        lb, nb = self.seg(2, 549408), self.seg(3, 549408)
        self.pid(100, 1, 'python3', [big, lb])
        self.pid(101, 100, 'component_container', [big, lb, nb])
        self.pid(300, 1, 'other', [self.seg(9, 33579552)])   # not a descendant: never counted
        rc, res = self.run_main(100, self.SHM32)
        self.assertEqual(rc, 1, res)
        self.assertFalse(res['passed'])
        self.assertEqual((res['required_bytes'], res['largest_own_bytes']), (33554432, 549408))
        self.assertIn('549408 B < the profile', res['reason'])
        # the same node against the 8 MB profile's threshold also fails (549408 < 8388608)
        rc, res = self.run_main(100, self.SHM8)
        self.assertEqual(rc, 1, res)

    def test_8mb_profile_fallback_fails(self):
        """the case the guard is for: the 8 MB profile, /dev/shm full of unowned segments, the node's falls back"""
        la = self.seg(1, 8413728)
        self.pid(100, 1, 'python3', [la])
        self.pid(101, 100, 'component_container', [la, self.seg(3, 549408)])
        self.assertEqual(self.run_main(100, self.SHM8)[0], 1)

    def test_grandchild_counts(self):
        la = self.seg(1, 8413728)
        self.pid(100, 1, 'python3', [la])
        self.pid(101, 100, 'sh', [la])
        self.pid(102, 101, 'component_container', [la, self.seg(3, 8413728)])
        rc, res = self.run_main(100, self.SHM8)
        self.assertEqual(rc, 0, res)
        self.assertEqual([c['pid'] for c in res['children']], [101, 102])

    def test_fail_closed(self):
        la = self.seg(1, 8413728)
        # no such pid
        rc, res = self.run_main(999, self.SHM8)
        self.assertEqual(rc, 2)
        self.assertIn('not found', res['reason'])
        # the launch process has no child
        self.pid(100, 1, 'python3', [la])
        rc, res = self.run_main(100, self.SHM8)
        self.assertEqual(rc, 2)
        self.assertIn('no child', res['reason'])
        # the child holds only the inherited segments; and a lock file whose segment file is gone has no size
        self.pid(101, 100, 'component_container', [la, 'fastrtps_%016x' % 7])
        rc, res = self.run_main(100, self.SHM8)
        self.assertEqual(rc, 2, res)
        self.assertIn('of its own', res['reason'])
        # a profile with no SHM segment_size: nothing to check against
        x = os.path.join(self.tmp, 'udp.xml')
        with open(x, 'w') as f:
            f.write('<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><transport_descriptors>'
                    '<transport_descriptor><transport_id>u</transport_id><type>UDPv4</type></transport_descriptor>'
                    '</transport_descriptors></profiles>')
        self.pid(102, 100, 'component_container', [self.seg(3, 8413728)])
        self.assertEqual(self.run_main(100, x)[0], 2)
        self.assertEqual(self.run_main(100, self.SHM8)[0], 0)   # the same tree passes with the 8 MB profile
        # an unreadable profile
        rc, res = self.run_main(100, os.path.join(self.tmp, 'missing.xml'))
        self.assertEqual(rc, 2)
        self.assertIn('unreadable', res['reason'])

    def test_profile_segment_sizes_in_tree(self):
        self.assertEqual(self.m.profile_segment_size(self.SHM8), 8388608)
        self.assertEqual(self.m.profile_segment_size(self.SHM32), 33554432)

    def test_wait_polls_until_passed(self):
        """a node that opens its own segment a moment after the check starts passes within --wait-s"""
        la = self.seg(1, 8413728)
        self.pid(100, 1, 'python3', [la])
        self.pid(101, 100, 'component_container', [la, self.seg(2, 549408)])
        own = self.seg(3, 8413728)
        def late():
            time.sleep(1.0)
            os.symlink('/dev/shm/%s_el' % own, os.path.join(self.proc, '101', 'fd', '99'))
        import threading
        th = threading.Thread(target=late)
        th.start()
        rc, res = self.run_main(100, self.SHM8, '--wait-s', '5')
        th.join()
        self.assertEqual(rc, 0, res)


class TestTestSwitches(Base):
    """Final acceptance (2026-10-14): CAR_FAILSTOP must reach car_inner.sh in the stack container (it was read
    there but never passed in), and car_scenario.sh passes extra arguments to bottle_scenario.py (D2 stop tests)."""

    def stack_line(self, out):
        return next(l for l in out.splitlines() if l.startswith('[container stack]'))

    def test_failstop_reaches_stack_container(self):
        r = self.up('sim', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('-e CAR_FAILSTOP=0', self.stack_line(r.stdout))          # default (10-26): restart hub/bridge/preview/producer
        r = self.up('sim', '--dry-run', CAR_FAILSTOP='0')
        self.assertEqual(r.returncode, 0, r.stderr)
        line = self.stack_line(r.stdout)
        self.assertIn('-e CAR_FAILSTOP=0', line)
        self.assertNotIn('CAR_FAILSTOP=1', line)
        self.assertIn('fail-stop=0', r.stdout)
        run = os.path.join(self.tmp, 'runs', 'fs0')
        r = self.up('sim', '--run-dir', run, CAR_FAILSTOP='0')
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(json.load(open(os.path.join(run, 'run_config.json')))['car_failstop'], '0')
        self.assertEqual(self.down().returncode, 0)

    def scenario(self, *args):
        run = os.path.join(self.tmp, 'car')
        os.makedirs(run, exist_ok=True)
        for f in ('scenario.jsonl', 'rc_scenario.txt'):
            if os.path.exists(os.path.join(run, f)):
                os.remove(os.path.join(run, f))
        json.dump({'containers': {'stack': 'wde-car-stack-x'}, 'camera': 'sim', 'record': True,
                   'inference_device': 'cuda', 'detector': 'yolo'}, open(os.path.join(run, 'car.json'), 'w'))
        argv = os.path.join(self.tmp, 'argv.json')
        write_exec(os.path.join(self.tmp, 'dexec'),
                   '#!/usr/bin/env python3\nimport json, sys\njson.dump(sys.argv[1:], open(%r, "w"))\n' % argv)
        e = dict(self.env, CAR_RUN=run, CAR_DOCKER=os.path.join(self.tmp, 'dexec'))
        r = subprocess.run(['bash', os.path.join(CAR, 'car_scenario.sh')] + list(args), env=e, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        a = json.load(open(argv))
        # docker exec -w /src STACK bash -c SCRIPT timeout device detector [extra...]: run the script the way bash does
        i = a.index('-c')
        out = subprocess.run(['bash', '-c', 'for x in "${@:3}"; do printf "%s\\n" "$x"; done'] + a[i + 2:], stdout=subprocess.PIPE,
                             universal_newlines=True).stdout.splitlines()
        return a, a[i + 1], out

    def test_scenario_extra_args_reach_bottle_scenario(self):
        a, script, extra = self.scenario()
        self.assertEqual(a[a.index('-c') + 2:], ['420', 'cuda', 'yolo'])     # no extra: the line-E call unchanged
        self.assertEqual(extra, [])
        self.assertTrue(script.rstrip().endswith('--expect-detector "$2" "${@:3}"'), script)
        a, _, extra = self.scenario('--cut-during-return-s', '3', '--cut-pid-file', '/o/producer.pid')
        self.assertEqual(extra, ['--cut-during-return-s', '3', '--cut-pid-file', '/o/producer.pid'])
        a, _, extra = self.scenario('--estop-during-return-s', '3', '--estop-hold-s', '3', '--estop-release-after-s', '1')
        self.assertEqual(extra[0::2], ['--estop-during-return-s', '--estop-hold-s', '--estop-release-after-s'])

class TestStopPid(Base):
    def test_int_term_kill_escalation(self):
        ready = '\nopen(sys.argv[1], "w").close()\nwhile True: time.sleep(1)'   # marker after the handlers
        code = {'plain': 'import signal,sys,time\nsignal.signal(signal.SIGINT, signal.default_int_handler)' + ready,
                'no_int': 'import signal,sys,time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)' + ready,
                'no_int_term': 'import signal,sys,time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\n'
                               'signal.signal(signal.SIGTERM, signal.SIG_IGN)' + ready}
        script = ['source %s' % COMMON, 'export CAR_RC_FILE=%s/rc.txt CAR_INT_WAIT_S=1 CAR_TERM_WAIT_S=1' % self.tmp,
                  WAIT_READY]
        for n, c in code.items():
            script.append('python3 -c %s %s/ready_%s & p_%s=$!' % (shlex.quote(c), self.tmp, n, n))
        script.append('wait_ready %s' % ' '.join('%s/ready_%s' % (self.tmp, n) for n in code))
        script.append('echo $p_plain $p_no_int $p_no_int_term > %s/pids' % self.tmp)
        for n in code:
            script.append('car_stop_pid %s $p_%s "$(car_pid_start $p_%s)"' % (n, n, n))
        r = subprocess.run(['bash', '-c', '\n'.join(script)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        pids = [int(x) for x in open(os.path.join(self.tmp, 'pids')).read().split()]
        self.assertFalse(any(alive(p) for p in pids))
        rc = open(os.path.join(self.tmp, 'rc.txt')).read()
        self.assertIn('plain_rc=', rc)
        self.assertNotIn('plain alive', rc)
        self.assertIn('no_int alive 1 s after INT, TERM', rc)
        self.assertNotIn('no_int alive after TERM', rc)
        self.assertIn('no_int_term alive after TERM, KILL', rc)
        self.assertIn('no_int_term_rc=137', rc)

    # 10-15 (line-4 bench: costmap / lifecycle / mount_tf 10 s each after INT, then TERM rc 143): `ros2 run` is a
    # Python wrapper that catches INT and keeps waiting for its child (ros2run run_executable: "the subprocess will
    # also receive the signal" -- true only for a terminal Ctrl-C). INT to the wrapper's pid never reaches the node.
    # Known answer with a stand-in of that loop: wrapper -> TERM needed, its child left running (orphan); the same
    # child started directly -> exits on INT, no TERM. car_inner.sh therefore starts the nodes directly (ros_exe).
    # (SIGINT set explicitly: background children of a non-interactive bash start with it ignored, and this host's
    # env has no --default-signal)
    WRAPPER = ('import signal, subprocess, sys\nsignal.signal(signal.SIGINT, signal.default_int_handler)\n'
               'p = subprocess.Popen([sys.executable, "-c", sys.argv[1]])\n'
               'while p.returncode is None:\n    try:\n        p.communicate()\n    except KeyboardInterrupt:\n        pass\n'
               'sys.exit(p.returncode)')
    # (READY: the marker the node creates once its handler is set -- WAIT_READY; the wrapper sets its own handler
    # before it starts the node, so the wrapped node's marker covers both)
    NODE = ('import os, signal, time\nsignal.signal(signal.SIGINT, signal.default_int_handler)\n'
            'open(os.environ["READY"], "w").close()\ntry:\n    time.sleep(60)\nexcept KeyboardInterrupt:\n    pass')

    def test_ros2_run_style_wrapper_swallows_int(self):
        # (outputs to /dev/null: the orphan must not hold this test's pipes)
        script = ['source %s' % COMMON, 'export CAR_RC_FILE=%s/rc.txt CAR_INT_WAIT_S=1 CAR_TERM_WAIT_S=1' % self.tmp,
                  WAIT_READY,
                  'READY=%s/ready_w python3 -c %s %s > /dev/null 2>&1 & w=$!' % (
                      self.tmp, shlex.quote(self.WRAPPER), shlex.quote(self.NODE)),
                  'READY=%s/ready_d python3 -c %s > /dev/null 2>&1 & d=$!' % (self.tmp, shlex.quote(self.NODE)),
                  'wait_ready %s/ready_w %s/ready_d; c=$(pgrep -P $w); echo $c > %s/child' % (self.tmp, self.tmp, self.tmp),
                  'car_stop_pid wrapper $w "$(car_pid_start $w)"', 'car_stop_pid direct $d "$(car_pid_start $d)"']
        r = subprocess.run(['bash', '-c', '\n'.join(script)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        child = int(open(os.path.join(self.tmp, 'child')).read())
        try:
            rc = open(os.path.join(self.tmp, 'rc.txt')).read()
            self.assertIn('wrapper alive 1 s after INT, TERM', rc)
            self.assertIn('wrapper_rc=143', rc)
            self.assertTrue(alive(child), 'the wrapped node should be left running (orphan) -- the defect')
            self.assertIn('direct_rc=0', rc)
            self.assertNotIn('direct alive', rc)
        finally:
            kill_if_alive(child)

    def test_inner_starts_nodes_directly(self):
        text = open(os.path.join(CAR, 'car_inner.sh')).read()
        launches = [l for l in text.splitlines() if l.lstrip().startswith(('launch ', 'run1 '))]
        self.assertTrue(launches)
        self.assertFalse([l for l in launches if 'ros2 run' in l], 'a node started through ros2 run again')
        # ros_exe: <prefix>/lib/PKG/EXE, first executable on AMENT_PREFIX_PATH; none -> rc 1
        lines = text.splitlines()
        a = lines.index('ros_exe() {')
        fn = '\n'.join(lines[a:lines.index('}', a) + 1])
        p1, p2 = os.path.join(self.tmp, 'p1'), os.path.join(self.tmp, 'p2')
        os.makedirs(os.path.join(p1, 'lib', 'pkg')); os.makedirs(os.path.join(p2, 'lib', 'pkg'))
        open(os.path.join(p1, 'lib', 'pkg', 'exe'), 'w').close()          # not executable: skipped
        x = os.path.join(p2, 'lib', 'pkg', 'exe'); open(x, 'w').close(); os.chmod(x, 0o755)
        r = subprocess.run(['bash', '-c', 'DRY_RUN=0 AMENT_PREFIX_PATH=%s:%s\n%s\nros_exe pkg exe; echo rc=$?; '
                            'ros_exe pkg none; echo rc=$?' % (p1, p2, fn)],
                           stdout=subprocess.PIPE, universal_newlines=True, timeout=30)
        self.assertEqual(r.stdout.split(), [x, 'rc=0', 'rc=1'])

    def inner_stop(self, ros2_body, bare_return=False):
        """car_inner.sh's own alive() + costmap pause + cleanup, run from the EXIT trap after INT as in the stack,
        with stand-in children (runtime, lifecycle, costmap) and a stand-in ros2 -> (marks, rc.txt, ros2 argv log)"""
        lines = open(os.path.join(CAR, 'car_inner.sh')).read().splitlines()
        a = lines.index('alive() {   # NAME: the tracked process of that name still runs')
        alive_fn = '\n'.join(lines[a:lines.index('}', a) + 1])
        if bare_return:   # positive control: the alive() before 10-15
            alive_fn = alive_fn.replace('return $?; }', 'return; }')
        b = next(i for i, l in enumerate(lines) if l.startswith('# The standalone Nav2 costmap'))
        c = lines.index('cleanup() {', b)
        stop_fns = '\n'.join(lines[b:lines.index('}', c) + 1])
        bindir = os.path.join(self.tmp, 'bin')
        os.makedirs(bindir, exist_ok=True)
        write_exec(os.path.join(bindir, 'ros2'), '#!/bin/bash\necho "$*" >> %s/ros2.log\n%s\n' % (self.tmp, ros2_body))
        o = os.path.join(self.tmp, 'o')
        os.makedirs(o, exist_ok=True)
        script = '\n'.join([
            'source %s' % COMMON, 'export PATH=%s:$PATH CAR_INT_WAIT_S=2 CAR_TERM_WAIT_S=1' % bindir,
            'O=%s DRY_RUN=0 NAV=costmap T=(env) COSTMAP_PAUSE_S=3' % o,
            'mark() { echo "$*" >> "$O/marks.txt"; }', alive_fn, stop_fns,
            'NAMES=(); PIDS=(); STARTS=()',
            'for n in costmap lifecycle runtime; do sleep 600 & NAMES+=("$n"); PIDS+=($!); STARTS+=("$(car_pid_start $!)"); done',
            'trap cleanup EXIT; trap "exit 130" INT TERM',
            'kill -INT $$; sleep 30'])
        r = subprocess.run(['bash', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=60)
        self.assertEqual(r.returncode, 130, r.stderr)
        rd = lambda f: open(os.path.join(self.tmp, f)).read() if os.path.exists(os.path.join(self.tmp, f)) else ''
        return rd('o/marks.txt').splitlines(), rd('o/rc.txt').split(), rd('ros2.log')

    def test_costmap_paused_before_lifecycle_stop(self):
        """10-15: the costmap is deactivated (lifecycle manager PAUSE) before the manager is stopped; still reverse
        order; the request starts with the stop and is waited for just before 'stop lifecycle'"""
        mk, rc, ros2 = self.inner_stop('sleep 0.3; echo "response:"; echo "ManageLifecycleNodes_Response(success=True)"')
        self.assertEqual(mk, ['cleanup', 'costmap deactivate requested (check_lifecycle manage_nodes PAUSE)',
                              'stop runtime', 'costmap deactivated', 'stop lifecycle', 'stop costmap', 'stopped'])
        self.assertEqual([x.split('_rc=')[0] for x in rc if '_rc=' in x], ['runtime', 'lifecycle', 'costmap'])
        self.assertEqual(ros2.split(), ['service', 'call', '/check_lifecycle/manage_nodes',
                                        'nav2_msgs/srv/ManageLifecycleNodes', '{command:', '1}'])

    def test_costmap_pause_failure_does_not_block_the_stop(self):
        mk, rc, _ = self.inner_stop('echo "service not available" >&2; exit 1')
        self.assertIn('costmap deactivate failed rc=1 (costmap_pause.log); stopping on', mk)
        self.assertEqual(mk[-3:], ['stop lifecycle', 'stop costmap', 'stopped'])
        mk, rc, _ = self.inner_stop('sleep 20')                     # hangs: COSTMAP_PAUSE_S (3 s here) cuts it
        self.assertIn('costmap deactivate failed rc=124 (costmap_pause.log); stopping on', mk)
        self.assertEqual(mk[-1], 'stopped')

    def test_positive_control_bare_return_in_trap(self):
        """the alive() before 10-15 (bare return): in the EXIT trap after INT it answers 130 = not alive, so no pause"""
        mk, _, ros2 = self.inner_stop('echo "success=True"', bare_return=True)
        self.assertNotIn('costmap deactivate requested (check_lifecycle manage_nodes PAUSE)', mk)
        self.assertEqual(ros2, '')

    def test_reused_pid_not_signalled(self):
        """pid + start time is the identity: a different start time means another process, left alone."""
        p = subprocess.Popen(['sleep', '30'])
        try:
            r = subprocess.run(['bash', '-c', 'source %s; CAR_RC_FILE=/dev/stdout car_stop_pid x %d 1' % (COMMON, p.pid)],
                               stdout=subprocess.PIPE, universal_newlines=True, timeout=30)
            self.assertIn('already exited', r.stdout)
            self.assertIsNone(p.poll())
        finally:
            p.kill(); p.wait()


class TestUpDown(Base):
    def test_partial_start_rolled_back(self):
        """the stack container exits before READY: the Gazebo and KV containers, the inference stand-in (ignores
        INT -> TERM), the GPU lock, the IPC directory and the pointer are all undone; exit 7."""
        run = os.path.join(self.tmp, 'runs', 'part1')
        r = self.up('sim', '--run-dir', run, FAKE_STACK='exit')
        self.assertEqual(r.returncode, 7, r.stdout + r.stderr)
        self.assertIn('rolling back', r.stderr)
        self.assertEqual(self.containers(), [])
        self.assertEqual(self.locks(), {})
        released = open(os.path.join(self.fd, 'released.log')).read()
        self.assertIn('gpu2', released)
        pid = int(open(os.path.join(run, 'host_pids.txt')).read().split()[1])
        self.assertFalse(alive(pid))
        rc = open(os.path.join(run, 'down_rc.txt')).read()
        self.assertIn('inference alive 1 s after INT, TERM', rc)
        down = json.load(open(os.path.join(run, 'down.json')))
        self.assertTrue(down['clean'], down)
        self.assertIn('stack container exited', down['rollback_reason'])
        self.assertEqual(os.listdir(self.state), [])
        signals = open(os.path.join(self.fd, 'signals.log')).read().split('\n')
        self.assertEqual([s.split()[0] for s in signals if s][:2], ['wde-car-kv-part1', 'wde-car-gz-part1'])

    def test_container_create_failure_rolled_back(self):
        run = os.path.join(self.tmp, 'runs', 'part2')
        r = self.up('sim', '--run-dir', run, FAKE_STACK='fail')
        self.assertEqual(r.returncode, 5, r.stderr)
        self.assertEqual(self.containers(), [])
        self.assertEqual(self.locks(), {})
        self.assertTrue(json.load(open(os.path.join(run, 'down.json')))['clean'])
        self.assertEqual(os.listdir(self.state), [])

    def test_lock_busy_rolled_back(self):
        subprocess.run([os.path.join(self.tmp, 'lock'), 'acquire', 'gpu2', 'someone else'], env=self.env, check=True,
                       stdout=subprocess.DEVNULL)
        run = os.path.join(self.tmp, 'runs', 'busy')
        r = self.up('sim', '--run-dir', run)
        self.assertEqual(r.returncode, 6, r.stderr)
        self.assertFalse(os.path.exists(os.path.join(run, 'host_pids.txt')))
        self.assertEqual(self.containers(), [])
        self.assertIn('gpu2', self.locks())          # the other holder's lock is untouched
        self.assertEqual(os.listdir(self.state), [])

    def test_up_ready_second_up_refused_then_down_clean(self):
        run = os.path.join(self.tmp, 'runs', 'ok1')
        r = self.up('sim', '--run-dir', run)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertEqual(self.containers(), ['wde-car-gz-ok1', 'wde-car-kv-ok1', 'wde-car-stack-ok1'])
        self.assertEqual(os.readlink(os.path.join(self.state, 'car.current')), run)
        token = json.load(open(os.path.join(run, 'gpu_lock.json')))['token']
        self.assertEqual(self.locks(), {'gpu2': token})
        pid = int(open(os.path.join(run, 'host_pids.txt')).read().split()[1])
        self.assertTrue(alive(pid))
        cfg = json.load(open(os.path.join(run, 'run_config.json')))
        self.assertEqual((cfg['chassis'], cfg['inference']['detector'], cfg['inference']['device']), ('mcu', 'yolo', 'cuda'))
        hub = json.load(open(os.path.join(run, 'car.json')))['hub']
        self.assertEqual((hub['bind'], hub['port']), ('127.0.0.1', 47431))   # host default
        argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-ok1.json')))['argv']
        self.assertIn('HUB_BIND=127.0.0.1', argv)
        self.assertIn('HUBPORT=47431', argv)
        # again while running: refused, nothing new
        run2 = os.path.join(self.tmp, 'runs', 'ok2')
        r2 = self.up('replay', '--run-dir', run2)
        self.assertEqual(r2.returncode, 10, r2.stderr)
        self.assertIn('already running', r2.stderr)
        self.assertFalse(os.path.exists(run2))
        self.assertEqual(len(self.containers()), 3)
        d = self.down()
        self.assertEqual(d.returncode, 0, d.stdout + d.stderr)
        self.assertEqual(self.containers(), [])
        self.assertEqual(self.locks(), {})
        self.assertFalse(alive(pid))
        self.assertEqual(os.listdir(self.state), [])
        self.assertTrue(json.load(open(os.path.join(run, 'down.json')))['clean'])
        self.assertFalse(os.path.exists(os.path.join(run, 'src')))
        # stack first (INT), then KV, then Gazebo
        order = [s.split()[0] for s in open(os.path.join(self.fd, 'signals.log')).read().split('\n') if s]
        self.assertEqual(order, ['wde-car-stack-ok1', 'wde-car-kv-ok1', 'wde-car-gz-ok1'])
        self.assertEqual(self.down().stdout.strip(), 'no car running')

    def test_replay_needs_no_gpu_lock(self):
        run = os.path.join(self.tmp, 'runs', 'rep1')
        r = self.up('replay', '--run-dir', run)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(self.containers(), ['wde-car-kv-rep1', 'wde-car-stack-rep1'])
        self.assertFalse(os.path.exists(os.path.join(run, 'gpu_lock.json')))
        self.assertFalse(os.path.exists(os.path.join(run, 'host_pids.txt')))
        calls = open(os.path.join(self.fd, 'calls.log')).read()
        self.assertNotIn('wde-car-gz', calls)
        self.assertEqual(self.down().returncode, 0)
        self.assertEqual(self.containers(), [])

    def test_down_without_pointer_removes_stray_car_containers_only(self):
        for n in ('wde-car-stack-lost', 'other-thing'):
            json.dump({'running': True, 'exit': 0}, open(os.path.join(self.fd, n + '.json'), 'w'))
        d = self.down()
        self.assertEqual(d.returncode, 0)
        self.assertEqual(self.containers(), ['other-thing'])


class TestCameraRatePlan(Base):
    """10-17 camera half-rate check (docs/design/COLOUR_RATE_NAVMODE_20261017.md section 7; tools/car_run/depth_probe.py
    --rate-s, executed in tests/test_camera_halfrate.py): CAR_CAMERA_RATE_CHECK board default 1 reaches the stack and
    the probe of every driver start; 0 = the plan of before (nothing passed in, no line added); bad values refused."""
    setUp = TestLiveBoard.setUp
    line = TestLiveBoard.line
    PRESETS = (['live'], TestLiveBoard.STATIC, ['live', '--pose', 'odom', '--nav', 'costmap'])

    @staticmethod
    def norm(out):   # the rtabmap run time stamp differs between two dry runs
        import re
        return [re.sub(r'\d{8}_\d{6}', '<T>', l) for l in out.splitlines()]

    def plans(self, args, **env):
        rd = ['--dry-run', '--run-dir', os.path.join(self.tmp, 'runs', 'plan')]
        on = self.up(*(args + rd), FAKE_USB='1', **env)
        off = self.up(*(args + rd), FAKE_USB='1', CAR_CAMERA_RATE_CHECK='0', **env)
        self.assertEqual((on.returncode, off.returncode), (0, 0), on.stderr + off.stderr)
        return on.stdout, off.stdout

    def test_default_on(self):
        for args in self.PRESETS:
            on, off = self.plans(args)
            self.assertIn('-e CAMERA_RATE_CHECK=1 -e CAMERA_RATE_S=3 ', self.line(on, '[container stack]'))
            cam = self.line(on, '    [camera]')
            # 10-29: the camera runs 15 fps by default; the probe judges at that rate (TestCameraFps: 30 = no argument)
            self.assertIn("depth_probe.py --topic /camera/depth/image_raw --count 30 --timeout-s 20 --rate-s 3 "
                          "--color-topic /camera/color/image_raw --rate-fps 15'", cam)
            self.assertTrue(cam.endswith('s pause; before every retry: fastdds shm clean, again)'), cam)
            self.assertIn('    # camera rate check: after the depth images the probe listens 3 s more', on)
            self.assertIn('(retry wait 23 + 5 s)', on)
            self.assertIn('# camera rate check: CAR_CAMERA_RATE_CHECK=1', on)
            # the rate check comes before the shm8 segment check and camera_check in the plan
            lines = on.splitlines()
            i = [n for n, l in enumerate(lines) if l.startswith('    # camera rate check:')][0]
            j = [n for n, l in enumerate(lines) if l.startswith('    [driver_shm_check, once]')][0]
            self.assertLess(i, j)

    def test_off_is_the_plan_of_before(self):
        """off: no trace anywhere; on = off + the probe arguments + the two plan lines + the stack env, nothing else"""
        for args in self.PRESETS:
            for extra in ({}, {'LIVE_DRIVER_DDS_PROFILE': ''}):
                on, off = self.plans(args, **extra)
                for w in ('CAMERA_RATE', '--rate-s', 'camera rate check', 'camera_rate'):
                    self.assertNotIn(w, off, (args, w))
                strip = [l.replace(' --rate-s 3 --color-topic /camera/color/image_raw --rate-fps 15', '')
                          .replace(' -e CAMERA_RATE_CHECK=1 -e CAMERA_RATE_S=3', '')
                         for l in self.norm(on) if 'camera rate check:' not in l]
                self.assertEqual(strip, self.norm(off), (args, extra))

    def test_window_and_bad_values(self):
        on, _ = self.plans(TestLiveBoard.STATIC, CAR_CAMERA_RATE_S='5')
        self.assertIn('--rate-s 5 ', self.line(on, '    [camera]'))
        self.assertIn('(retry wait 25 + 5 s)', on)
        for k, v in (('CAR_CAMERA_RATE_CHECK', '2'), ('CAR_CAMERA_RATE_CHECK', 'yes'), ('CAR_CAMERA_RATE_S', '0'),
                     ('CAR_CAMERA_RATE_S', '6'), ('CAR_CAMERA_RATE_S', '2.5')):
            r = self.up(*(TestLiveBoard.STATIC + ['--dry-run']), FAKE_USB='1', **{k: v})
            self.assertEqual(r.returncode, 2, (k, v, r.stdout))
            self.assertIn(k, r.stderr)

    def test_host_presets_untouched(self):
        for args in (['sim'], ['replay']):
            r = self.up(*(args + ['--dry-run']))
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertNotIn('CAMERA_RATE', r.stdout)
            self.assertNotIn('--rate-s', r.stdout)


class TestPreviewFps(Base):
    """10-25: CAR_PREVIEW_FPS reaches the stack container (preview_publisher.py reads it; car_up used to drop it, so
    only the 10 fps default ever ran on the car); unset = the plan of before."""
    setUp = TestLiveBoard.setUp
    line = TestLiveBoard.line
    norm = staticmethod(TestCameraRatePlan.norm)

    def plan(self, **env):
        rd = ['--dry-run', '--run-dir', os.path.join(self.tmp, 'runs', 'plan')]
        r = self.up(*(TestLiveBoard.STATIC + rd), FAKE_USB='1', CAR_PREVIEW='1', **env)
        self.assertEqual(r.returncode, 0, r.stderr)
        return r.stdout

    def test_rate_passed_only_when_set(self):
        on, off = self.plan(CAR_PREVIEW_FPS='5'), self.plan()
        self.assertIn(' -e CAR_PREVIEW_FPS=5 ', self.line(on, '[container stack]'))
        self.assertNotIn('CAR_PREVIEW_FPS', off)
        self.assertEqual([l.replace(' -e CAR_PREVIEW_FPS=5', '') for l in self.norm(on)], self.norm(off))



class TestCameraFps(Base):
    """10-29 (owner, Nano CPU): the camera runs CAR_CAMERA_FPS (default 15); 30 = the plan of before except the
    one '# camera rate' line; the rate reaches the probe, the runner and the colour sender only when not 30."""
    setUp = TestLiveBoard.setUp
    line = TestLiveBoard.line
    norm = staticmethod(TestCameraRatePlan.norm)

    def plan(self, args=None, **env):
        rd = ['--dry-run', '--run-dir', os.path.join(self.tmp, 'runs', 'plan')]
        r = self.up(*((args or TestLiveBoard.STATIC) + rd), FAKE_USB='1', **env)
        self.assertEqual(r.returncode, 0, r.stderr)
        return r.stdout

    def test_default_15(self):
        out = self.plan(CAR_AR_COLOUR='1')
        self.assertIn('color_fps:=15 ', self.line(out, '[container stack]'))
        self.assertIn('depth_fps:=15 ', self.line(out, '[container stack]'))
        self.assertIn('# camera rate: CAR_CAMERA_FPS=15', out)
        self.assertIn(' --rate-fps 15', self.line(out, '    [camera]'))
        self.assertIn(' --camera-fps 15 --camera-qos ', self.line(out, '    [runtime]'))
        self.assertTrue(self.line(out, '    [colour]').split(' > ')[0].endswith(' --camera-fps 15'))

    def test_30_is_the_plan_of_before(self):
        on, off = self.plan(CAR_AR_COLOUR='1'), self.plan(CAR_AR_COLOUR='1', CAR_CAMERA_FPS='30')
        self.assertNotIn('--camera-fps', off)
        self.assertNotIn('--rate-fps', off)
        self.assertIn('color_fps:=30 ', self.line(off, '[container stack]'))
        strip = lambda o: [l.replace('fps:=15', 'fps:=30').replace(' --rate-fps 15', '').replace(' --camera-fps 15', '')
                           for l in self.norm(o) if not l.startswith('# camera rate')]
        self.assertEqual(strip(on), strip(off))

    def test_explicit_launch_args_win_and_bad_values(self):
        args = ('color_width:=640 color_height:=480 color_fps:=30 color_format:=RGB depth_width:=640 depth_height:=400 '
                'depth_fps:=30 depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true '
                'align_mode:=HW publish_tf:=false')
        out = self.plan(LIVE_LAUNCH_ARGS=args)
        self.assertIn('# camera rate: CAR_CAMERA_FPS=30', out)
        self.assertNotIn('--camera-fps', out)
        for v in ('20', 'x', ''):
            r = self.up(*(TestLiveBoard.STATIC + ['--dry-run']), FAKE_USB='1', CAR_CAMERA_FPS=v)
            if v == '':
                continue        # empty = the default (":=")
            self.assertEqual(r.returncode, 2, (v, r.stdout))
            self.assertIn('CAR_CAMERA_FPS', r.stderr)


class TestPriority(Base):
    """10-29 (owner, Nano CPU): CAR_PRIORITY board default 1 = --cap-add SYS_NICE on the stack container and the
    priority process (docker exec -u 0) after the container starts; 0 = the container line of before, no process."""
    setUp = TestLiveBoard.setUp
    line = TestLiveBoard.line
    make_node = TestLiveBoard.make_node
    norm = staticmethod(TestCameraRatePlan.norm)

    def plan(self, **env):
        rd = ['--dry-run', '--run-dir', os.path.join(self.tmp, 'runs', 'plan')]
        r = self.up(*(TestLiveBoard.STATIC + rd), FAKE_USB='1', **env)
        self.assertEqual(r.returncode, 0, r.stderr)
        return r.stdout

    def test_plan_on_off(self):
        on, off = self.plan(), self.plan(CAR_PRIORITY='0')
        self.assertTrue(self.line(on, '[container stack]').split('  (ready')[0].endswith(
            ' --cap-add SYS_NICE --entrypoint bash fpga-ros:humble-nav-orbbec-scipy-20261017 /src/tools/car_run/car_inner.sh'))
        self.assertIn('exec -u 0 wde-car-stack-plan python3 /src/tools/car_run/car_priority.py --interval-s 3',
                      self.line(on, '[priority]'))
        self.assertNotIn('SYS_NICE', off)
        self.assertNotIn('[priority]', off)
        self.assertEqual([l.replace(' --cap-add SYS_NICE', '') for l in self.norm(on) if not l.startswith('[priority]')],
                         self.norm(off))
        r = self.up(*(TestLiveBoard.STATIC + ['--dry-run']), FAKE_USB='1', CAR_PRIORITY='2')
        self.assertEqual(r.returncode, 2)
        self.assertIn('CAR_PRIORITY', r.stderr)

    def test_host_presets_untouched(self):
        r = self.up('sim', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('SYS_NICE', r.stdout)
        self.assertNotIn('[priority]', r.stdout)

    def test_real_start_runs_it_and_down_is_clean(self):
        self.make_node()
        for prio, name in (('1', 'pr1'), ('0', 'pr0')):
            run = os.path.join(self.tmp, 'runs', name)
            r = self.up(*(TestLiveBoard.STATIC + ['--run-dir', run]), FAKE_USB='1', CAR_PRIORITY=prio)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            argv = json.load(open(os.path.join(self.fd, 'wde-car-stack-%s.json' % name)))['argv']
            calls = open(os.path.join(self.fd, 'calls.log')).read()
            marks = open(os.path.join(run, 'up_marks.txt')).read()
            if prio == '1':
                self.assertEqual(argv[argv.index('--cap-add') + 1], 'SYS_NICE')
                self.assertIn('exec -u 0 wde-car-stack-pr1', calls)
                self.assertIn('priority started', marks)
                self.assertTrue(os.path.exists(os.path.join(run, 'priority.log')))
            else:
                self.assertNotIn('--cap-add', argv)
                self.assertNotIn('exec -u 0 wde-car-stack-pr0', calls)
                self.assertNotIn('priority', marks)
            d = self.down()
            self.assertEqual(d.returncode, 0, d.stdout + d.stderr)
            self.assertTrue(json.load(open(os.path.join(run, 'down.json')))['clean'])
            self.assertEqual(self.containers(), [])


if __name__ == '__main__':
    unittest.main()
