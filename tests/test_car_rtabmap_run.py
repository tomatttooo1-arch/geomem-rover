"""Real car (2026-10-15): tools/car_run with rtabmap -- POSE map (a mapping run: tools/car_run/rtabmap_car.sh map) and
POSE rtabmap (localization: rtabmap_car.sh loc) inside the car stack. Dry runs and stand-ins only (no docker, no ROS,
no camera): the switch combinations, car.json's pose note, the stop-wait file written by car_up and read by car_down,
what the mapping run starts, the localization ready check, the runner's arguments, the wheel odometry forced off
under visual odometry, and rtabmap's own INT wait in car_inner's cleanup. Positive controls: a 1 s stop wait makes the
stop-wait checks fail. Python 3.8 compatible."""
import json
import os
import subprocess
import sys
import textwrap
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from test_car_run import Base, CAR, COMMON, DOWN, FAKE_KV_CHECK, FAKE_LSUSB, write_exec  # noqa: E402

# docker stand-in whose stack container keeps running FAKE_STOP_DELAY_S after INT (rtabmap saving its database);
# TERM / KILL stop it at once. Only what car_down.sh calls.
SLOW_DOCKER = textwrap.dedent('''\
    import json, os, sys, time
    d = os.environ['FAKE_DOCKER_DIR']
    a = sys.argv[1:]
    def path(n): return os.path.join(d, n + '.json')
    def load(n):
        try:
            return json.load(open(path(n)))
        except OSError:
            return None
    def save(n, s): json.dump(s, open(path(n), 'w'))
    def running(s):
        return s['running'] and not (s.get('int_at') and time.time() >= s['int_at'] + float(os.environ['FAKE_STOP_DELAY_S']))
    cmd = a[0]
    if cmd == 'inspect':
        if a[1] == '-f':
            s = load(a[3])
            if s is None: sys.exit(1)
            print('true' if running(s) else 'false') if 'Running' in a[2] else print(0)
        else:
            sys.exit(0 if load(a[1]) is not None else 1)
    elif cmd == 'kill':
        name = a[-1]; s = load(name)
        if s is None: sys.exit(1)
        if a[2] == 'INT':
            s['int_at'] = s.get('int_at') or time.time()
        else:
            s['running'] = False
        save(name, s)
        open(os.path.join(d, 'signals.log'), 'a').write(name + ' ' + a[2] + '\\n')
    elif cmd == 'logs':
        print('fake log of ' + a[1])
    elif cmd == 'rm':
        try: os.remove(path(a[-1]))
        except OSError: pass
    elif cmd == 'ps':
        for f in sorted(os.listdir(d)):
            if f.endswith('.json'): print(f[:-5])
    ''')

MAP_NOTE = 'rtabmap 建图（新库，不是定位；不起导航与运行器）'


def line(out, prefix):
    return next(l for l in out.splitlines() if l.startswith(prefix))


def launched(out):
    """the names of the inner plan's started steps: '    [name] ...' / '    [name, once] ...'"""
    names = []
    for l in out.splitlines():
        if l.startswith('    [') and ']' in l:
            names.append(l[5:l.index(']')].split(',')[0])
    return names


class TestHostCombinations(Base):
    def test_refused_before_anything(self):
        for args, env, msg in (
                (['sim', '--pose', 'map'], {}, 'sim 相机配 map 位姿'),
                (['replay', '--pose', 'map', '--nav', 'nav2'], {}, 'map 位姿配 nav2'),
                (['replay', '--pose', 'odom'], {}, 'replay 相机配 odom 位姿'),
                (['replay', '--pose', 'map'], {'RTABMAP_ODOM': 'visual'}, 'replay 配 RTABMAP_ODOM=visual'),
                (['replay', '--pose', 'map'], {'RTABMAP_ODOM': 'lidar'}, 'RTABMAP_ODOM 只能是'),
                (['replay', '--pose', 'map'], {'CAR_RTABMAP_STOP_S': '4m'}, 'CAR_RTABMAP_STOP_S must be whole seconds')):
            for dry in ([], ['--dry-run']):
                r = self.up(*(args + dry), **env)
                self.assertEqual(r.returncode, 2, (args, env, r.stdout, r.stderr))
                self.assertIn(msg, r.stderr, args)
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))
        self.assertEqual(os.listdir(self.state), [])

    def test_replay_map_plan(self):
        """the mapping run on the host: rtabmap_car.sh map on the replay topics (best effort, no TF from rtabmap,
        odometry from the replay's TF), database under the run (/o/rtabmap_db in the stack); no navigation, no
        inference, no runner; the hub, the bridge and the producer stay"""
        r = self.up('replay', '--pose', 'map', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn('gpu_lock=0', out)
        self.assertNotIn('[inference]', out)
        self.assertNotIn('[gpu_lock]', out)
        names = launched(out)
        for n in ('hub', 'bridge', 'producer', 'rtabmap'):
            self.assertIn(n, names)
        for n in ('stub', 'runtime', 'runtime_info', 'costmap', 'lifecycle', 'navigation', 'nav2_configs',
                  'rtabmap_odom', 'loc_gate'):
            self.assertNotIn(n, names)
        rt = line(out, '    [rtabmap]')
        for part in ('env STANDALONE=0 MODE=map ODOM=wheel DB_DIR=/o/rtabmap_db RUN_TIME=', 'OUT=/o/rtabmap_db/run_',
                     'COLOR_TOPIC=/fpga/rgb DEPTH_TOPIC=/fpga/depth INFO_TOPIC=/fpga/camera_info QOS=2 RTAB_PUBLISH_TF=false',
                     'CAM_MOUNT_TF= CAR_WHEEL_ODOM=0 SAVE_TIMEOUT_S=120 GATE_COUNT_MODE=correction '
                     'GATE_MAX_CORRECTION_AGE_S=2 bash tools/car_run/rtabmap_car.sh map'):
            self.assertIn(part, rt)
        self.assertIn("-> ready check: pose rtabmap map (NEW database, mapping run -- not localization) odom=wheel "
                      "(rtabmap.log 'running --', <= 120 s)", out)
        self.assertNotIn('localization_status', out)
        self.assertIn('# costmap: skipped (pose map', out)
        self.assertIn('# stub, runtime, runtime_info: skipped (pose map', out)
        stack = line(out, '[container stack]')
        self.assertIn('-e RTABMAP_ODOM=wheel -e RTABMAP_DB_DIR=/o/rtabmap_db -e RTABMAP_STOP_S=240', stack)
        self.assertIn('stop wait: rtabmap 240 s, stack 330 s', out)
        r = self.up('replay', '--pose', 'map', '--dry-run', CAR_RTABMAP_STOP_S='300')
        self.assertIn('-e RTABMAP_STOP_S=300', line(r.stdout, '[container stack]'))
        self.assertIn('stack 390 s', r.stdout)
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    def test_reference_replay_untouched(self):
        r = self.up('replay', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('rtabmap', r.stdout)
        self.assertNotIn('RTABMAP_STOP_S', r.stdout)
        self.assertIn('runtime', launched(r.stdout))

    def test_map_real_start_car_json_and_stop_file(self):
        """car.json must take pose map (pose_note is a dict lookup: a missing key was KeyError -> rollback 8); the
        stop-wait file is written into the run and car_down reads it (an explicit CAR_STACK_STOP_S still wins)"""
        r = self.up('replay', '--pose', 'map')
        self.assertEqual(r.returncode, 0, r.stderr)
        run = os.readlink(os.path.join(self.state, 'car.current'))
        car = json.load(open(os.path.join(run, 'car.json')))
        self.assertEqual(car['pose'], 'map')
        self.assertEqual(car['pose_note'], MAP_NOTE)
        self.assertEqual(car['rtabmap']['odom'], 'wheel')
        self.assertEqual(car['rtabmap']['stack_stop_s'], 330)
        self.assertEqual(open(os.path.join(run, 'stack_stop_s.txt')).read().strip(), '330')
        self.assertTrue(os.path.isdir(os.path.join(run, 'rtabmap_db')))
        self.env.pop('CAR_STACK_STOP_S')
        d = self.down()
        self.assertEqual(d.returncode, 0, d.stderr)
        marks = open(os.path.join(run, 'down_marks.txt')).read()
        self.assertIn('stack stop wait 330 s (stack_stop_s.txt)', marks)
        self.assertEqual(self.containers(), [])

    def test_reference_real_start_no_stop_file(self):
        r = self.up('replay')
        self.assertEqual(r.returncode, 0, r.stderr)
        run = os.readlink(os.path.join(self.state, 'car.current'))
        self.assertFalse(os.path.exists(os.path.join(run, 'stack_stop_s.txt')))
        self.assertNotIn('rtabmap', json.load(open(os.path.join(run, 'car.json'))))
        self.env.pop('CAR_STACK_STOP_S')
        self.assertEqual(self.down().returncode, 0)
        self.assertIn('stack stop wait 150 s (default)', open(os.path.join(run, 'down_marks.txt')).read())


class TestStopWait(Base):
    """car_down's wait for the stack container after INT, with a stack that needs 3 s to stop (rtabmap saving)"""
    DELAY_S = 3

    def down_with(self, file_value, **env):
        run = os.path.join(self.tmp, 'run_%d' % len(os.listdir(self.tmp)))
        os.mkdir(run)
        name = 'wde-car-stack-t'
        with open(os.path.join(run, 'containers.txt'), 'w') as f:
            f.write(name + '\n')
        if file_value is not None:
            with open(os.path.join(run, 'stack_stop_s.txt'), 'w') as f:
                f.write(file_value + '\n')
        with open(os.path.join(self.fd, name + '.json'), 'w') as f:
            json.dump({'running': True}, f)
        write_exec(os.path.join(self.tmp, 'slowdocker'), '#!/usr/bin/env python3\n' + SLOW_DOCKER)
        e = dict(self.env, CAR_DOCKER=os.path.join(self.tmp, 'slowdocker'), FAKE_STOP_DELAY_S=str(self.DELAY_S))
        e.pop('CAR_STACK_STOP_S')
        e.update(env)
        r = subprocess.run(['bash', DOWN, '--run', run], env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=120)
        self.assertEqual(r.returncode, 0, r.stderr)
        return open(os.path.join(run, 'down_marks.txt')).read()

    def assert_stack_stopped_by_int(self, marks):
        self.assertIn('wde-car-stack-t: INT', marks)
        self.assertNotIn('TERM', marks)
        self.assertNotIn('KILL', marks)

    def test_file_wait_lets_the_stack_finish(self):
        marks = self.down_with('330')
        self.assertIn('stack stop wait 330 s (stack_stop_s.txt)', marks)
        self.assert_stack_stopped_by_int(marks)

    def test_positive_control_one_second_wait_fails(self):
        """the stand-in: a 1 s stop wait in the file -> TERM after 1 s; the check above must fail on it"""
        marks = self.down_with('1')
        self.assertIn('stack stop wait 1 s (stack_stop_s.txt)', marks)
        with self.assertRaises(AssertionError):
            self.assert_stack_stopped_by_int(marks)

    def test_explicit_env_wins_over_file(self):
        marks = self.down_with('330', CAR_STACK_STOP_S='1')
        self.assertIn('stack stop wait 1 s (env)', marks)
        self.assertIn('still running 1 s after INT, TERM', marks)
        marks = self.down_with(None)
        self.assertIn('stack stop wait 150 s (default)', marks)


class TestInnerRtabmapStop(Base):
    """car_inner.sh's cleanup: the child named rtabmap (rtabmap_car.sh: saves the database after INT) gets
    RTABMAP_STOP_S; every other child keeps CAR_INT_WAIT_S"""

    def inner_cleanup(self, rtabmap_stop_s):
        lines = open(os.path.join(CAR, 'car_inner.sh')).read().splitlines()
        a = lines.index('alive() {   # NAME: the tracked process of that name still runs')
        alive_fn = '\n'.join(lines[a:lines.index('}', a) + 1])
        b = next(i for i, l in enumerate(lines) if l.startswith('# The standalone Nav2 costmap'))
        c = lines.index('cleanup() {', b)
        stop_fns = '\n'.join(lines[b:lines.index('}', c) + 1])
        child = os.path.join(self.tmp, 'saver.py')
        with open(child, 'w') as f:   # INT -> 2 s "saving" -> exit 0
            f.write('import signal, sys, time\n'
                    'got = []\nsignal.signal(signal.SIGINT, lambda *a: got.append(1))\n'
                    'while not got:\n    time.sleep(0.05)\ntime.sleep(2)\nsys.exit(0)\n')
        o = os.path.join(self.tmp, 'o')
        os.makedirs(o, exist_ok=True)
        script = '\n'.join([
            'source %s' % COMMON, 'export CAR_INT_WAIT_S=1 CAR_TERM_WAIT_S=1',
            'O=%s DRY_RUN=0 NAV=nav2 RTABMAP_STOP_S=%d' % (o, rtabmap_stop_s),
            'mark() { echo "$*" >> "$O/marks.txt"; }', alive_fn, stop_fns,
            'NAMES=(); PIDS=(); STARTS=()',
            'for n in rtabmap other; do %s %s & NAMES+=("$n"); PIDS+=($!); STARTS+=("$(car_pid_start $!)"); done'
            % (sys.executable, child),
            'sleep 1; cleanup'])
        r = subprocess.run(['bash', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        return open(os.path.join(o, 'rc.txt')).read()

    def assert_rtabmap_stopped_by_int(self, rc):
        self.assertIn('rtabmap_rc=0', rc)
        self.assertNotIn('rtabmap alive', rc)

    def test_rtabmap_gets_its_wait_others_do_not(self):
        rc = self.inner_cleanup(10)
        self.assert_rtabmap_stopped_by_int(rc)
        self.assertIn('other alive 1 s after INT, TERM', rc)   # the control child: CAR_INT_WAIT_S unchanged

    def test_positive_control_one_second_wait_fails(self):
        rc = self.inner_cleanup(1)
        self.assertIn('rtabmap alive 1 s after INT, TERM', rc)
        with self.assertRaises(AssertionError):
            self.assert_rtabmap_stopped_by_int(rc)


class TestBoardRtabmap(Base):
    """board dry runs (stand-ins as test_car_run.TestLiveBoard): localization (live preset) and mapping on the car"""

    def setUp(self):
        Base.setUp(self)
        write_exec(os.path.join(self.tmp, 'lsusb'), FAKE_LSUSB)
        write_exec(os.path.join(self.tmp, 'kvcheck'), FAKE_KV_CHECK)
        self.usb = os.path.join(self.tmp, 'usb')
        self.env.update(CAR_ARCH='aarch64', CAR_BOARD_DRAFT='1', CAR_HUB_BIND='198.51.100.1',
                        CAR_LSUSB=os.path.join(self.tmp, 'lsusb'), CAR_USB_ROOT=self.usb,
                        CAR_KV_CHECK=os.path.join(self.tmp, 'kvcheck'), CAR_PY_BOARD=os.path.join(self.tmp, 'py'))
        self.db = os.path.join(self.tmp, 'pinned.db')
        with open(self.db, 'w') as f:
            f.write('db')
        self.env['RTABMAP_DB'] = self.db
        self.map_yaml = os.path.join(self.tmp, 'run_pinned', 'export', 'pinned.yaml')
        os.makedirs(os.path.dirname(self.map_yaml))
        with open(self.map_yaml, 'w') as f:
            f.write('image: pinned.pgm\n')
        self.dbdir = os.path.join(self.tmp, 'rtabmap_dbdir')
        self.env['RTABMAP_DB_DIR'] = self.dbdir
        for k in ('CAR_WHEEL_ODOM', 'RTABMAP_ODOM', 'FPGA_NAV_COSTMAP', 'CAR_MAP_YAML'):
            self.env.pop(k, None)

    def test_gate_mode_and_x_reach_rtabmap_car(self):
        # car_up.sh CAR_GATE_MODE / CAR_GATE_X_S -> INNER_ENV -> car_inner.sh -> rtabmap_car.sh (10-15 evening)
        self.env.update(CAR_GATE_MODE='output', CAR_GATE_X_S='3')
        r = self.up('live', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('GATE_COUNT_MODE=output GATE_MAX_CORRECTION_AGE_S=3 bash tools/car_run/rtabmap_car.sh loc',
                      line(r.stdout, '    [rtabmap]'))

    def test_loc_plan(self):
        r = self.up('live', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        names = launched(out)
        self.assertNotIn('rtabmap_odom', names)       # rtabmap_car.sh loc starts the odometry, adapter and gate
        self.assertNotIn('loc_gate', names)
        self.assertNotIn('wheel_odom', names)         # ODOM visual: wheel odometry forced off (live preset: uart)
        self.assertIn('wheel_odom=0', line(out, '    # car_inner plan:'))
        rt = line(out, '    [rtabmap]')
        for part in ('env STANDALONE=0 MODE=loc ODOM=visual DB_DIR=%s DB=%s OUT=%s/run_loc_' % (self.dbdir, self.db, self.dbdir),
                     'COLOR_TOPIC=/camera/color/image_raw DEPTH_TOPIC=/camera/depth/image_raw '
                     'INFO_TOPIC=/camera/depth/camera_info CAMERA_FRAME=camera_color_optical_frame',
                     'CAM_MOUNT_TF= CAR_WHEEL_ODOM=0 SAVE_TIMEOUT_S=120 GATE_COUNT_MODE=correction '
                     'GATE_MAX_CORRECTION_AGE_S=2 bash tools/car_run/rtabmap_car.sh loc'):
            self.assertIn(part, rt)
        self.assertNotIn('QOS=2', rt)
        self.assertIn("(rtabmap.log 'running --' + /fpga/localization_status, <= 120 s)", out)
        run = line(out, '    [runtime]')
        self.assertIn('--pose-source estimated --localization-map-id', run)
        self.assertIn('--localization-tracked-frame base_link', run)
        cmd = run.split(' > ')[0]
        self.assertLess(cmd.index('--localization-tracked-frame'), cmd.index('--ros-args'))
        stack = line(out, '[container stack]')
        self.assertIn('-e WHEEL_ODOM=0 ', stack)
        self.assertIn('-e RTABMAP_ODOM=visual -e RTABMAP_DB_DIR=%s -e RTABMAP_STOP_S=240' % self.dbdir, stack)
        self.assertIn('-v %s:%s ' % (self.dbdir, self.dbdir), stack)
        exp = os.path.dirname(self.map_yaml)
        self.assertIn('-v %s:%s:ro' % (exp, exp), stack)
        self.assertIn('-e MAP_YAML=%s' % self.map_yaml, stack)
        # Nav2 on the board: the launch file by path, the real-car switches; FPGA_NAV_COSTMAP rolling unless given (10-15
        # integ: the board default; tests/test_car_run.py TestLiveBoard.test_board_nav2_rolling_dry_run has the order)
        nav = line(out, '    [navigation]')
        self.assertIn('ros2 launch %s/robot_sim/fpga_sim/launch/navigation.launch.py' % ROOT, nav)
        self.assertIn('FPGA_NAV_AUTOSTART=false FPGA_NAV_USE_SIM_TIME=false FPGA_NAV_STATIC_MAP_ODOM=false '
                      'FPGA_NAV_SIM_CAMERA_TF=false FPGA_NAV_SPEED_LIMITS=car FPGA_NAV_VELOCITY_GUARD=true '
                      'FPGA_NAV_MAP_YAML=%s FPGA_NAV_COSTMAP=rolling ' % self.map_yaml, nav)
        self.assertNotIn('fpga_sim navigation.launch.py', nav)
        r = self.up('live', '--dry-run', FPGA_NAV_COSTMAP='sim')
        self.assertIn('FPGA_NAV_COSTMAP=sim', line(r.stdout, '    [navigation]'))
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    def test_visual_odometry_refuses_wheel_odometry(self):
        r = self.up('live', '--dry-run', CAR_WHEEL_ODOM='1')
        self.assertEqual(r.returncode, 2, r.stdout)
        self.assertIn('RTABMAP_ODOM=visual 配 CAR_WHEEL_ODOM=1', r.stderr)
        r = self.up('live', '--dry-run', CAR_WHEEL_ODOM='0')
        self.assertEqual(r.returncode, 0, r.stderr)
        r = self.up('live', '--dry-run', RTABMAP_ODOM='wheel')     # localization needs the gate (odom_info)
        self.assertEqual(r.returncode, 2)
        self.assertIn('rtabmap 定位配 RTABMAP_ODOM=wheel', r.stderr)

    def test_map_plan(self):
        r = self.up('live', '--pose', 'map', '--nav', 'costmap', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertNotIn('[inference]', out)
        names = launched(out)
        for n in ('hub', 'bridge', 'producer', 'rtabmap', 'camera'):
            self.assertIn(n, names)
        for n in ('runtime', 'runtime_info', 'costmap', 'navigation', 'wheel_odom'):
            self.assertNotIn(n, names)
        rt = line(out, '    [rtabmap]')
        self.assertIn('MODE=map ODOM=visual DB_DIR=%s RUN_TIME=' % self.dbdir, rt)
        self.assertIn('rtabmap_car.sh map', rt)
        self.assertNotIn('MAP_YAML', line(out, '[container stack]'))
        # mapping on wheel odometry: the wheel odometry must run (uart: on by default; 0 refused)
        r = self.up('live', '--pose', 'map', '--nav', 'costmap', '--dry-run', RTABMAP_ODOM='wheel')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('ODOM=wheel', line(r.stdout, '    [rtabmap]'))
        self.assertIn('wheel_odom', launched(r.stdout))
        r = self.up('live', '--pose', 'map', '--nav', 'costmap', '--dry-run', RTABMAP_ODOM='wheel', CAR_WHEEL_ODOM='0')
        self.assertEqual(r.returncode, 2)
        self.assertIn('live 配 RTABMAP_ODOM=wheel 而 CAR_WHEEL_ODOM≠1', r.stderr)

    def test_nav2_without_exported_map_refused(self):
        os.remove(self.map_yaml)
        r = self.up('live')
        self.assertEqual(r.returncode, 2, r.stdout)
        self.assertIn('--nav nav2 needs the map of the pinned database', r.stderr)
        calls = os.path.join(self.fd, 'calls.log')   # (the already-running check may have asked docker ps)
        self.assertEqual([l for l in (open(calls).read().splitlines() if os.path.exists(calls) else [])
                          if not l.startswith('ps')], [])
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'kv_calls.log')))   # refused before the KV precheck
        self.assertEqual(os.listdir(self.state), [])


if __name__ == '__main__':
    unittest.main()
