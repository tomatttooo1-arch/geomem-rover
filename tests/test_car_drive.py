"""Real-car drive chain (2026-10-15): host/ros_teleop_safe.py TeleopCore (limits, dead man, single-command duration,
zero on exit), tools/car_run/teleop_keys.py send mode, the car_up / car_inner switches CAR_TELEOP / CAR_WHEEL_ODOM /
--pose odom (combinations, mutual exclusion, dry-run plans), the board directory defaults and the socket length check,
host/ros_wheel_odom.py status and tools/car_run/bench_pose.py --no-odom-base. No ROS, no docker. Python 3.8 compatible."""
import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from host.ros_teleop_safe import (DEADMAN_CAP, MAX_CMD_CAP, V_CAP, W_CAP, TeleopCore, check_limits,  # noqa: E402
                                  open_socket)
from test_car_run import Base, FAKE_KV_CHECK, FAKE_LSUSB, write_exec  # noqa: E402

MS = 10**6
KEYS = os.path.join(ROOT, 'tools', 'car_run', 'teleop_keys.py')


class TestTeleopCore(unittest.TestCase):
    def test_no_input_is_zero(self):
        c = TeleopCore()
        self.assertEqual(c.command(0), (0., 0., 'no_input'))

    def test_clamped_positive_control(self):
        c = TeleopCore()
        self.assertEqual(c.on_input(1.0, -5.0, 0), (0.15, -0.6))       # beyond the limits: clamped
        self.assertEqual(c.command(10 * MS), (0.15, -0.6, 'drive'))
        self.assertEqual(c.counters['clamped'], 1)
        self.assertEqual(c.on_input(0.1, 0.2, 20 * MS), (0.1, 0.2))      # inside: unchanged
        self.assertEqual(c.counters['clamped'], 1)
        c = TeleopCore(v_max=0.3, w_max=1.0)                              # the caps themselves are allowed
        self.assertEqual(c.on_input(9., 9., 0), (0.3, 1.0))

    def test_non_finite_is_zero(self):
        for bad in (math.nan, math.inf, -math.inf):
            c = TeleopCore()
            self.assertEqual(c.on_input(bad, 0.1, 0), (0., 0.))
            self.assertEqual(c.command(1 * MS)[:2], (0., 0.))
            self.assertEqual(c.counters['non_finite'], 1)

    def test_limits_capped(self):
        for kw in ({'v_max': V_CAP + 0.01}, {'w_max': W_CAP * 2}, {'deadman_s': DEADMAN_CAP + 0.1},
                   {'max_cmd_s': MAX_CMD_CAP + 1}, {'v_max': 0}, {'v_max': -0.1}, {'v_max': math.nan}):
            with self.assertRaises(ValueError, msg=kw):
                TeleopCore(**kw)
        check_limits(0.15, 0.6, 0.3, 3.0)

    def test_deadman(self):
        c = TeleopCore()
        c.on_input(0.1, 0., 0)
        self.assertEqual(c.command(299 * MS), (0.1, 0., 'drive'))
        self.assertEqual(c.command(301 * MS), (0., 0., 'deadman'))
        self.assertEqual(c.counters['deadman'], 1)
        c.on_input(0.1, 0., 400 * MS)                                    # a new press drives again
        self.assertEqual(c.command(410 * MS)[2], 'drive')

    def test_zero_input_stops_at_once(self):
        c = TeleopCore()
        c.on_input(0.1, 0.3, 0)
        c.on_input(0., 0., 50 * MS)
        self.assertEqual(c.command(51 * MS), (0., 0., 'zero_input'))

    def test_max_duration_latched_until_release(self):
        c = TeleopCore(max_cmd_s=1.0)
        t = 0
        while t + MS <= 1000 * MS:                                       # held (auto-repeat every 40 ms) for 1 s
            c.on_input(0.1, 0., t)
            self.assertEqual(c.command(t + MS)[2], 'drive', t)
            t += 40 * MS
        c.on_input(0.1, 0., t)
        self.assertEqual(c.command(t + 30 * MS), (0., 0., 'max_duration'))
        c.on_input(0.05, 0.2, t + 40 * MS)                              # changing the key is not a release
        self.assertEqual(c.command(t + 41 * MS)[2], 'max_duration')
        c.on_input(0., 0., t + 80 * MS)                                  # release (zero) clears the latch
        self.assertEqual(c.command(t + 81 * MS)[2], 'zero_input')
        c.on_input(0.1, 0., t + 120 * MS)
        self.assertEqual(c.command(t + 121 * MS)[2], 'drive')
        # a dead-man expiry is a release too
        c2 = TeleopCore(max_cmd_s=0.2)
        for k in range(10):
            c2.on_input(0.1, 0., k * 40 * MS)
        self.assertEqual(c2.command(361 * MS)[2], 'max_duration')
        self.assertEqual(c2.command(361 * MS + 301 * MS)[2], 'deadman')
        c2.on_input(0.1, 0., 2000 * MS)
        self.assertEqual(c2.command(2001 * MS)[2], 'drive')

    def test_datagrams(self):
        c = TeleopCore()
        self.assertEqual(c.on_datagram(b'{"v": 0.1, "w": -0.2}', 0), (0.1, -0.2))
        self.assertEqual(c.on_datagram(b'{"stop": true}', MS), (0., 0.))
        for bad in (b'not json', b'[1, 2]', b'{"v": 0.1}', b'{"v": "x", "w": 0}', b'\xff\xfe'):
            self.assertIsNone(c.on_datagram(bad, 2 * MS), bad)
        self.assertEqual(c.counters['bad_input'], 5)
        # a malformed datagram does not refresh the dead man
        c = TeleopCore()
        c.on_datagram(b'{"v": 0.1, "w": 0}', 0)
        c.on_datagram(b'garbage', 250 * MS)
        self.assertEqual(c.command(301 * MS)[2], 'deadman')

    def test_stale_datagram_ignored(self):
        c = TeleopCore()
        # queued during a 0.7 s stall: sent at 0, read at 700 ms -> ignored, the dead man still counts from nothing
        self.assertIsNone(c.on_datagram(b'{"v": 0.1, "w": 0, "sent_ns": 0}', 700 * MS))
        self.assertEqual(c.counters['stale_input'], 1)
        self.assertEqual(c.command(701 * MS)[2], 'no_input')
        self.assertEqual(c.on_datagram(b'{"v": 0.1, "w": 0, "sent_ns": 690000000}', 700 * MS), (0.1, 0.))
        self.assertEqual(c.on_datagram(b'{"stop": true, "sent_ns": 0}', 900 * MS), (0., 0.))   # a stale stop still stops
        self.assertIsNone(c.on_datagram(b'{"v": 0.1, "w": 0, "sent_ns": "x"}', 0))

    def test_status_says_teleop(self):
        st = TeleopCore().status()
        self.assertEqual(st['mode'], 'teleop')
        self.assertIn('NOT a navigation run', st['note'])
        self.assertIn('motion_permit', st['note'])
        json.dumps(st, allow_nan=False)


class TestTeleopKeysSend(unittest.TestCase):
    """teleop_keys.py send -> the node's socket -> TeleopCore: the real datagram path, without ROS."""

    def test_send_reaches_core(self):
        d = tempfile.mkdtemp(prefix='tk_')
        path = os.path.join(d, 'teleop.sock')
        s = open_socket(path)
        self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)
        try:
            r = subprocess.run([sys.executable, KEYS, '--socket', path, 'send', '0.5', '0.1', '--seconds', '0.3',
                                '--hz', '20'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               universal_newlines=True, timeout=30)
            self.assertEqual(r.returncode, 0, r.stderr)
            out = json.loads(r.stdout)
            self.assertGreaterEqual(out['sent'], 4)
            c = TeleopCore()
            s.settimeout(0.5)
            got = []
            while True:
                try:
                    got.append(c.on_datagram(s.recv(4096), 0))
                except socket.timeout:
                    break
            self.assertEqual(len(got), out['sent'])
            self.assertTrue(all(g == (0.15, 0.1) for g in got), got)     # 0.5 m/s asked, 0.15 given
        finally:
            s.close()
            os.unlink(path)
            os.rmdir(d)

    def test_keys_refuses_without_terminal(self):
        r = subprocess.run([sys.executable, KEYS, '--socket', '/nonexistent/x.sock'], stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
        self.assertEqual(r.returncode, 2)
        self.assertIn('terminal', r.stderr)


class TestBenchPoseOdom(unittest.TestCase):
    def test_no_odom_base(self):
        from tools.car_run import bench_pose
        p = bench_pose.plan(0, 8.0, 0.05, odom_base=False)
        self.assertEqual(p['static_edges'], [['map', 'odom']])
        self.assertEqual(p['dynamic_edges'], [])
        self.assertIn('轮式里程', p['note'])
        p = bench_pose.plan(30.0, 8.0, 0.05, odom_base=False)              # odom_hz ignored: no odom -> base_link at all
        self.assertEqual((p['static_edges'], p['dynamic_edges']), ([['map', 'odom']], []))
        self.assertEqual(bench_pose.plan(0, 8.0, 0.05)['static_edges'], [['map', 'odom'], ['odom', 'base_link']])


class TestWheelOdomStatus(unittest.TestCase):
    def test_core_status_default_no_tf(self):
        from host.ros_wheel_odom import OdomCore
        st = OdomCore('chassis').status()
        self.assertFalse(st['publishes_tf'])
        self.assertIsNone(st['tf_topic'])


class TestCarDrive(Base):
    """car_up.sh / car_inner.sh dry runs with the drive switches (stand-in docker; nothing starts)."""

    def plan(self, *args, **env):
        r = self.up(*(list(args) + ['--dry-run']), **env)
        return r

    def line(self, out, prefix):
        return next(l for l in out.splitlines() if l.startswith(prefix))

    def test_teleop_refused(self):
        # 10-18: sim / live with nav2 + CAR_TELEOP=1 are the arbitration mode (test_sim_teleop_arbitration_plan);
        # (['sim'], 'Gazebo') -> sim with costmap; (['live', '--chassis', 'uart'], 'nav2') opened
        for args, word in ((['sim', '--nav', 'costmap'], 'Gazebo'), (['replay'], '底盘'),
                           (['live', '--pose', 'static', '--chassis', 'none', '--nav', 'costmap'], '底盘')):
            run = os.path.join(self.tmp, 'r_' + '_'.join(a.strip('-') for a in args))
            r = self.up(*(args + ['--run-dir', run]), CAR_TELEOP='1')
            self.assertEqual(r.returncode, 2, (args, r.stderr))
            self.assertIn(word, r.stderr, args)
            self.assertFalse(os.path.exists(run))
        r = self.up('replay', '--dry-run', CAR_TELEOP='yes')
        self.assertEqual(r.returncode, 2)
        self.assertEqual(os.listdir(self.state), [])
        self.assertFalse(os.path.exists(os.path.join(self.fd, 'calls.log')))

    def test_sim_teleop_arbitration_plan(self):
        """10-18: sim (nav2, chassis pty) + CAR_TELEOP=1: the Gazebo guard on /fpga/nav/cmd_vel_safe, teleop --nav-topic"""
        r = self.plan('sim', CAR_TELEOP='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        gz = self.line(r.stdout, '[container gz]')
        self.assertIn('-e FPGA_SIM_GUARD_OUTPUT=/fpga/nav/cmd_vel_safe', gz)
        self.assertIn('--nav-topic /fpga/nav/cmd_vel_safe', self.line(r.stdout, '    [teleop]'))
        r = self.plan('sim')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('FPGA_SIM_GUARD_OUTPUT', r.stdout)
        self.assertNotIn('[teleop]', r.stdout)

    def test_pose_odom_refused(self):
        # 10-17 (REAL_CAR_NAV_GOLIVE_20261016.md section 12): odom + nav2 (the live preset) is refused only without
        # the mapping export (tests/test_odom_nav2.py has the allowed plan); the old "Nav2 entry not decided" is gone
        for args, word in ((['replay', '--pose', 'odom'], '双发'), (['live', '--pose', 'odom', '--chassis', 'none',
                                                                     '--nav', 'costmap'], 'uart'),
                           (['live', '--pose', 'odom'], '不合法：odom 位姿配 nav2 没有地图')):
            r = self.up(*(args + ['--dry-run']), CAR_MAP_YAML='', RTABMAP_DB='')
            self.assertEqual(r.returncode, 2, (args, r.stderr))
            self.assertIn(word, r.stderr, args)
            self.assertNotIn('板上 Nav2 入口未定', r.stderr, args)
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run', CAR_WHEEL_ODOM='0')
        self.assertEqual(r.returncode, 2)
        self.assertIn('无人发', r.stderr)
        r = self.up('replay', '--dry-run', CAR_WHEEL_ODOM='1')               # chassis none: no counts
        self.assertEqual(r.returncode, 2)

    def test_replay_pty_teleop_plan(self):
        r = self.plan('replay', '--chassis', 'pty', CAR_TELEOP='1', CAR_WHEEL_ODOM='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn('teleop=1 wheel_odom=1', out)
        self.assertIn('# TELEOP: remote control run (NOT a navigation run', out)
        t = self.line(out, '    [teleop]')
        self.assertIn('python3 -m host.ros_teleop_safe --socket %s/' % self.state, t)
        self.assertIn('/teleop.sock --record ', t)
        w = self.line(out, '    [wheel_odom]')
        self.assertIn('--source chassis --tf-topic /fpga/wheel_odom/tf --record', w)   # shadow: replay owns odom TF
        self.assertNotIn('--nav-odom-topic', w)
        self.assertIn('placeholder=1', out)
        self.assertNotIn('velocity_guard', out.split('# inner plan')[1])
        stack = self.line(out, '[container stack]')
        self.assertIn('-e TELEOP=1 -e WHEEL_ODOM=1', stack)
        # chassis order: the driver, then the wheel odometry, then the remote control
        names = [l.split(']')[0].strip(' [') for l in out.splitlines() if l.startswith('    [')]
        self.assertLess(names.index('chassis_driver'), names.index('wheel_odom'))
        self.assertLess(names.index('wheel_odom'), names.index('teleop'))
        # off: unchanged plan (no teleop, no wheel odometry on pty by default)
        r = self.plan('replay', '--chassis', 'pty')
        self.assertNotIn('[teleop]', r.stdout)
        self.assertNotIn('[wheel_odom]', r.stdout)
        self.assertIn('teleop=0 wheel_odom=0', r.stdout)

    def test_host_wheel_defaults_unchanged(self):
        """D-041 changes the board only: host sim / replay with wheel odometry keep the placeholder (no parameter
        arguments to the node, placeholder=1) and print no wheel-params line"""
        for args in (['sim'], ['replay', '--chassis', 'pty']):
            r = self.plan(*args, CAR_WHEEL_ODOM='1')
            self.assertEqual(r.returncode, 0, (args, r.stderr))
            w = self.line(r.stdout, '    [wheel_odom]')
            self.assertNotIn('--wheel-diam-m', w)
            self.assertIn('wheel_diam_m=0.065 counts_per_rev=1320 track_m=0.16 placeholder=1', r.stdout)
            self.assertNotIn('# wheel params:', r.stdout)
            self.assertIn('-e WHEEL_DIAM_M= -e COUNTS_PER_REV= -e TRACK_M=', self.line(r.stdout, '[container stack]'))

    def test_wheel_params_reach_node_and_marks(self):
        r = self.plan('replay', '--chassis', 'pty', CAR_WHEEL_ODOM='1', CAR_WHEEL_DIAM_M='0.07',
                      CAR_COUNTS_PER_REV='1400', CAR_TRACK_M='0.17')
        self.assertEqual(r.returncode, 0, r.stderr)
        w = self.line(r.stdout, '    [wheel_odom]')
        self.assertIn('--wheel-diam-m 0.07 --counts-per-rev 1400 --track-m 0.17', w)
        self.assertIn('placeholder=0', r.stdout)
        r = self.plan('replay', '--chassis', 'pty', CAR_WHEEL_ODOM='1', CAR_WHEEL_DIAM_M='abc')
        self.assertNotEqual(r.returncode, 0)


class TestCarDriveBoard(Base):
    def setUp(self):
        Base.setUp(self)
        write_exec(os.path.join(self.tmp, 'lsusb'), FAKE_LSUSB)
        write_exec(os.path.join(self.tmp, 'kvcheck'), FAKE_KV_CHECK)
        self.env.update(CAR_ARCH='aarch64', CAR_BOARD_DRAFT='1', CAR_HUB_BIND='198.51.100.1',
                        CAR_LSUSB=os.path.join(self.tmp, 'lsusb'), CAR_USB_ROOT=os.path.join(self.tmp, 'usb'),
                        CAR_KV_CHECK=os.path.join(self.tmp, 'kvcheck'), FAKE_USB='1')

    def line(self, out, prefix):
        return next(l for l in out.splitlines() if l.startswith(prefix))

    def test_live_uart_teleop_odom_plan(self):
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run', CAR_TELEOP='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        w = self.line(out, '    [wheel_odom]')
        self.assertIn('--tf-topic /tf --nav-odom-topic /odom', w)
        b = self.line(out, '    [bench_pose]')
        # 10-16 (SELECT_SEMANTICS_20261016.md section 3): the odom stand-in /map is 20 m by default (was 8 m)
        self.assertIn('bench_pose.py --no-odom-base --map-m 20', b)
        self.assertIn('--pose-source reference', self.line(out, '    [runtime]'))
        self.assertIn('[teleop]', out)
        self.assertIn('--device /dev/fpga_chassis', self.line(out, '[container stack]'))
        self.assertNotIn('rtabmap', out.split('# inner plan')[1])
        # live + uart without teleop: wheel odometry still on by default (the TF/odom chain for any real-car run)
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('[wheel_odom]', r.stdout)
        self.assertNotIn('[teleop]', r.stdout)

    def test_board_default_wheel_params_real_car(self):
        """owner 10-16 (D-041): the board's default wheel parameters are the real car's 0.081 / 1560 / 0.394 (not the
        placeholder); they reach the wheel odometry node, the stack container and the dry-run plan with their source"""
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        w = self.line(out, '    [wheel_odom]')
        self.assertIn('--source chassis --wheel-diam-m 0.081 --counts-per-rev 1560 --track-m 0.394 --tf-topic /tf', w)
        self.assertIn('wheel_diam_m=0.081 counts_per_rev=1560 track_m=0.394 placeholder=0', out)
        self.assertNotIn('PLACEHOLDER', out)
        self.assertIn('-e WHEEL_DIAM_M=0.081 -e COUNTS_PER_REV=1560 -e TRACK_M=0.394', self.line(out, '[container stack]'))
        self.assertIn('# wheel params: wheel_diam_m=0.081 counts_per_rev=1560 track_m=0.394 -- source default (real car',
                      out)

    def test_board_wheel_params_env_override(self):
        """environment variables still override the board defaults, each one on its own (source says env / mixed)"""
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run', CAR_WHEEL_DIAM_M='0.07',
                    CAR_COUNTS_PER_REV='1400', CAR_TRACK_M='0.17')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('--wheel-diam-m 0.07 --counts-per-rev 1400 --track-m 0.17', self.line(r.stdout, '    [wheel_odom]'))
        self.assertIn('# wheel params: wheel_diam_m=0.07 counts_per_rev=1400 track_m=0.17 -- source env', r.stdout)
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run', CAR_TRACK_M='0.40')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('--wheel-diam-m 0.081 --counts-per-rev 1560 --track-m 0.40', self.line(r.stdout, '    [wheel_odom]'))
        self.assertIn('source mixed (wheel_diam_m=default counts_per_rev=default track_m=env)', r.stdout)
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run', CAR_WHEEL_DIAM_M='abc')
        self.assertNotEqual(r.returncode, 0)

    def test_board_no_wheel_odom_no_wheel_line(self):
        """wheel odometry off on the board (rtabmap visual / static bench): no wheel line, no wheel env in the stack"""
        for args in (['live', '--pose', 'static', '--chassis', 'none', '--nav', 'costmap'],
                     ['live', '--pose', 'map', '--nav', 'costmap']):
            r = self.up(*(args + ['--dry-run']), RTABMAP_DB='', RTABMAP_DB_DIR=os.path.join(self.tmp, 'rdb'))
            self.assertEqual(r.returncode, 0, (args, r.stderr))
            self.assertNotIn('# wheel params:', r.stdout)
            self.assertNotIn('WHEEL_DIAM_M=', self.line(r.stdout, '[container stack]'))

    def test_static_bench_no_wheel_tf(self):
        r = self.up('live', '--pose', 'static', '--chassis', 'none', '--nav', 'costmap', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('[wheel_odom]', r.stdout)
        self.assertIn('bench_pose.py --odom-hz 0 --map-m 8', r.stdout)

    def test_board_directory_defaults(self):
        env = dict(self.env)
        env.pop('CAR_STATE_DIR')
        env.pop('CAR_RUNS_DIR')   # Base points it at the test's tmp; the defaults are what is checked here (dry runs)
        for args in (['live', '--pose', 'odom', '--nav', 'costmap'], ['replay']):   # aarch64: both on /data/fpga
            r = subprocess.run(['bash', os.path.join(ROOT, 'tools', 'car_run', 'car_up.sh')] + args + ['--dry-run'],
                               env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                               timeout=60)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertIn('# run directory: /data/fpga/car/runs/', r.stdout)
            self.assertIn('IPC /data/fpga/car/state/', r.stdout)
        env.pop('CAR_ARCH')
        env.pop('CAR_RUNS_DIR', None)
        if os.uname().machine != 'aarch64':
            r = subprocess.run(['bash', os.path.join(ROOT, 'tools', 'car_run', 'car_up.sh'), 'replay', '--dry-run'],
                               env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                               timeout=60)
            self.assertIn('# run directory: ${FPGA_SRC}/runs/car-runs/', r.stdout)
            self.assertIn('IPC ${FPGA_HOME}/dev1/.t/wde/', r.stdout)
            # live dry run on the host: the board paths (the plan is for the board)
            r = subprocess.run(['bash', os.path.join(ROOT, 'tools', 'car_run', 'car_up.sh'), 'live', '--dry-run'],
                               env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                               timeout=60)
            self.assertIn('IPC /data/fpga/car/state/', r.stdout)
        # car_down / car_pc share the default (no car there: they say so, they do not look in /data1)
        r = subprocess.run(['bash', '-c', 'source %s; CAR_ARCH=aarch64; unset CAR_STATE_DIR CAR_RUNS_DIR; car_default_dirs; '
                            'echo "$CAR_STATE_DIR $CAR_RUNS_DIR"' % os.path.join(ROOT, 'tools', 'car_run', 'car_common.sh')],
                           stdout=subprocess.PIPE, universal_newlines=True, timeout=30)
        self.assertEqual(r.stdout.split(), ['/data/fpga/car/state', '/data/fpga/car/runs'])

    def test_socket_path_limit(self):
        long_state = '/data/' + 'x' * 70
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run', CAR_STATE_DIR=long_state)
        self.assertEqual(r.returncode, 2)
        self.assertIn('107', r.stderr)
        # the default board path is far below it
        tag = '20261015-153000-live'
        self.assertLessEqual(len('/data/fpga/car/state/%s/teleop.sock' % tag), 107)


if __name__ == '__main__':
    unittest.main()
