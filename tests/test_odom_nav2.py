"""Real-car navigation on the wheel odometry pose (2026-10-17, docs/design/REAL_CAR_NAV_GOLIVE_20261016.md section 12):
car_validate no longer refuses --pose odom --nav nav2 (the board Nav2 entry is in main); it stays pinned to live +
chassis uart, and car_up.sh / car_inner.sh refuse it without the mapping export (Nav2's map_server needs it).
In that combination bench_pose publishes map -> odom only (--map-m 0, never --map-yaml): /map has one publisher, the
map_server; the wheel odometry publishes odom -> base_link on /tf and /odom; the rolling bridge gets body_margin_m 0.05.
Refusals (chassis none, teleop, CAR_NAV_DRYRUN, CAR_WHEEL_ODOM=0, no map) create nothing. The odom + costmap plans
(stand-in 20 m / --map-yaml) are unchanged. No docker, ROS or board: the board stand-ins of test_car_run.
Python 3.8 compatible."""
import json
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (HERE, ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

import test_car_run as tcr  # noqa: E402  (Base / stand-ins only)

COMMON = os.path.join(ROOT, 'tools', 'car_run', 'car_common.sh')
INNER = os.path.join(ROOT, 'tools', 'car_run', 'car_inner.sh')
NO_MAP = '不合法：odom 位姿配 nav2 没有地图'
REMOVED = '不合法：odom 位姿配 nav2——板上 Nav2 入口未定'


def validate(**env):
    """car_validate alone (car_common.sh sourced): -> (rc, stderr)"""
    e = {k: v for k, v in os.environ.items() if k not in ('NAV_DRYRUN', 'TELEOP')}
    e.update(env)
    r = subprocess.run(['bash', '-c', 'source "$1"; car_validate', '_', COMMON], env=e, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
    return r.returncode, r.stderr


CAR = dict(CAMERA='live', POSE='odom', NAV='nav2', DETECTOR='yolo', CHASSIS='uart')


class TestValidate(unittest.TestCase):
    def test_odom_nav2_uart_passes_the_switch_check(self):
        self.assertEqual(validate(**CAR), (0, ''))
        for extra in ({'NAV_DRYRUN': '0'}, {'TELEOP': '0'}):
            self.assertEqual(validate(**dict(CAR, **extra)), (0, ''), extra)

    def test_other_combinations_still_refused(self):
        for change, word in ((dict(CHASSIS='none'), '不合法：odom 位姿配 none 底盘——轮式里程来自真底盘串口，须 --chassis uart'),
                             (dict(CHASSIS='pty'), '不合法：odom 位姿配 pty 底盘'),
                             (dict(CAMERA='replay', CHASSIS='none', DETECTOR='stub'), '不合法：replay 相机配 odom 位姿'),
                             (dict(CAMERA='sim', CHASSIS='pty'), '不合法：sim 相机配 odom 位姿'),
                             # (dict(TELEOP='1'), '不合法：遥控配 nav2') -- opened 10-18 as the arbitration mode
                             # (TestTeleopArbitration below; docs/design/TELEOP_NAV_ARB_20261018.md)
                             (dict(NAV_DRYRUN='1'), 'CAR_NAV_DRYRUN=1 只用于板上 Nav2 空跑 live --pose static --nav nav2 --chassis none')):
            rc, err = validate(**dict(CAR, **change))
            self.assertEqual(rc, 2, change)
            self.assertIn(word, err, change)
            self.assertNotIn(REMOVED, err, change)

    def test_teleop_nav2_is_the_arbitration_mode(self):
        """10-18: TELEOP=1 + nav2 passes (live uart, sim pty); every other teleop refusal is word for word"""
        self.assertEqual(validate(TELEOP='1', **CAR), (0, ''))
        sim = dict(CAMERA='sim', POSE='reference', NAV='nav2', DETECTOR='yolo', CHASSIS='pty')
        self.assertEqual(validate(TELEOP='1', **sim), (0, ''))
        for env, word in ((dict(sim, NAV='costmap'), '不合法：遥控配 sim——仿真里速度守卫（Gazebo 容器）已发 /fpga/cmd_vel_safe，二者互斥'),
                          (dict(CAR, CHASSIS='none'), '不合法：遥控配 none 底盘'),
                          (dict(sim, CHASSIS='none'), '不合法：遥控配 none 底盘')):
            rc, err = validate(TELEOP='1', **env)
            self.assertEqual(rc, 2, env)
            self.assertIn(word, err, env)

    def test_dry_run_switch_unchanged(self):
        """CAR_NAV_DRYRUN still opens static + nav2 + none only"""
        bench = dict(CAMERA='live', POSE='static', NAV='nav2', DETECTOR='yolo', CHASSIS='none')
        self.assertEqual(validate(NAV_DRYRUN='1', **bench), (0, ''))
        rc, err = validate(**bench)
        self.assertEqual(rc, 2)
        self.assertIn('不合法：static 位姿配 nav2', err)


class BoardBase(tcr.Base):
    """car_up end to end with the board stand-ins of test_car_run.TestLiveBoard (no camera, KV260, docker)."""
    ARGS = ['live', '--pose', 'odom', '--nav', 'nav2']     # the live preset: chassis uart, detector yolo (trt)

    def setUp(self):
        tcr.Base.setUp(self)
        tcr.write_exec(os.path.join(self.tmp, 'lsusb'), tcr.FAKE_LSUSB)
        tcr.write_exec(os.path.join(self.tmp, 'kvcheck'), tcr.FAKE_KV_CHECK)
        self.usb = os.path.join(self.tmp, 'usb')
        self.node = os.path.join(self.usb, '002', '005')
        self.map_yaml = os.path.join(self.tmp, 'maps', 'export', 'm.yaml')
        os.makedirs(os.path.dirname(self.map_yaml))
        with open(self.map_yaml, 'w') as f:
            f.write('image: m.pgm\n')
        self.port = os.path.join(self.tmp, 'fpga_chassis')
        open(self.port, 'w').close()
        self.env.update(CAR_ARCH='aarch64', CAR_BOARD_DRAFT='1', CAR_HUB_BIND='198.51.100.1',
                        CAR_LSUSB=os.path.join(self.tmp, 'lsusb'), CAR_USB_ROOT=self.usb,
                        CAR_KV_CHECK=os.path.join(self.tmp, 'kvcheck'), CAR_PY_BOARD=os.path.join(self.tmp, 'py'),
                        RTABMAP_DB='', CAR_MAP_YAML=self.map_yaml, FAKE_USB='1', CHASSIS_PORT=self.port)
        for k in ('CAR_NAV_DRYRUN', 'BENCH_MAP_M', 'CAR_TELEOP', 'CAR_WHEEL_ODOM'):
            self.env.pop(k, None)

    def line(self, out, prefix):
        return next(l for l in out.splitlines() if l.startswith(prefix))

    def names(self, out):
        return [l.split(']')[0].strip().lstrip('[').split(',')[0] for l in out.splitlines() if l.startswith('    [')]

    def nothing_created(self, run):
        """no run directory, no state, no docker call but the real start's read-only already-running check (ps)"""
        self.assertFalse(os.path.exists(run))
        self.assertEqual(os.listdir(self.state), [])
        p = os.path.join(self.fd, 'calls.log')
        calls = open(p).read().splitlines() if os.path.exists(p) else []
        self.assertEqual([c for c in calls if c.split()[:1] != ['ps']], [], calls)


class TestOdomNav2Plan(BoardBase):
    def test_dry_run_plan(self):
        r = self.up(*(self.ARGS + ['--dry-run']), BENCH_MAP_M='20')     # even a given stand-in size is overridden
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn('# car_up dry run: camera=live pose=odom nav=nav2 detector=yolo chassis=uart', out)
        self.assertIn("# pose odom + nav2: map->odom identity (start = origin; bench_pose --no-odom-base --map-m 0)", out)
        self.assertIn("/map from Nav2's map_server = %s" % self.map_yaml, out)
        # who publishes what: the wheel odometry odom -> base_link (/tf) and /odom; bench_pose map -> odom only
        w = self.line(out, '    [wheel_odom]')
        self.assertIn('python3 -m host.ros_wheel_odom --source chassis', w)
        self.assertIn('--wheel-diam-m 0.081 --counts-per-rev 1560 --track-m 0.394', w)   # D-041 board default
        self.assertIn('--tf-topic /tf --nav-odom-topic /odom', w)
        b = self.line(out, '    [bench_pose]')
        self.assertIn('bench_pose.py --no-odom-base --map-m 0 >', b)
        self.assertNotIn('--map-yaml', b)                                  # map_server is the only /map publisher
        self.assertIn('odom + nav2: map->odom identity', self.line(out, '    #   -> ready check: pose'))
        # Nav2 on the board: the export for map_server, car limits, velocity guard, rolling costmap (DWB)
        nav = self.line(out, '    [navigation]')
        for word in ('FPGA_NAV_MAP_YAML=%s ' % self.map_yaml, 'FPGA_NAV_SPEED_LIMITS=car', 'FPGA_NAV_VELOCITY_GUARD=true',
                     'FPGA_NAV_COSTMAP=rolling', 'FPGA_NAV_STATIC_MAP_ODOM=false', 'FPGA_NAV_USE_SIM_TIME=false',
                     'FPGA_NAV2_OVERRIDES=', '/controllers/dwb.yaml', 'FPGA_NAV_AUTOSTART=false'):
            self.assertIn(word, nav)
        self.assertIn('ready check: nav2 costmap=rolling overrides=dwb.yaml', out)
        # the bridge: car body height and the 5 cm body-seed margin (round 3)
        br = self.line(out, '    [bridge]')
        self.assertIn('-p robot_height_m:=0.31 -p robot_radius_m:=0.30 -p body_margin_m:=0.05', br)
        # the container: chassis port, no stand-in, ONE map
        stack = self.line(out, '[container stack]')
        self.assertIn('--device %s --group-add ' % self.port, stack)
        self.assertIn('-e BENCH_MAP_M=0 ', stack)
        self.assertEqual(stack.count('-e MAP_YAML='), 1, stack)
        self.assertIn('-e MAP_YAML=%s ' % self.map_yaml, stack)
        self.assertIn('-v %s:%s:ro' % ((os.path.dirname(self.map_yaml),) * 2), stack)
        self.assertIn('-e WHEEL_ODOM=1 ', stack)
        self.assertIn('-e NAV_COSTMAP=rolling ', stack)
        self.assertNotIn('NAV_DRYRUN', stack)
        self.assertNotIn('-e TELEOP=1', stack)
        # order: chassis driver -> wheel odometry -> map -> odom; producer -> grid -> Nav2 active -> runner
        names = self.names(out)
        self.assertLess(names.index('chassis_driver'), names.index('wheel_odom'))
        self.assertLess(names.index('wheel_odom'), names.index('bench_pose'))
        self.assertLess(names.index('bench_pose'), names.index('navigation'))
        self.assertLess(names.index('producer'), names.index('rolling_grid'))
        self.assertLess(names.index('rolling_grid'), names.index('nav2_activate'))
        self.assertLess(names.index('nav2_activate'), names.index('runtime'))
        for absent in ('teleop', 'rtabmap', 'costmap', 'lifecycle'):
            self.assertNotIn(absent, names)
        self.assertIn('--pose-source reference', self.line(out, '    [runtime]'))
        self.assertIn('[kv precheck]', out)
        self.assertFalse(os.path.exists(self.runs))                        # a dry run creates nothing

    def test_dry_run_plan_teleop_arbitration(self):
        """10-18: CAR_TELEOP=1 + nav2 -> the guard on /fpga/nav/cmd_vel_safe, teleop --nav-topic, teleop before Nav2"""
        r = self.up(*(self.ARGS + ['--dry-run']), CAR_TELEOP='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        nav = self.line(out, '    [navigation]')
        self.assertIn('FPGA_NAV_VELOCITY_GUARD=true', nav)
        self.assertIn('FPGA_NAV_GUARD_OUTPUT=/fpga/nav/cmd_vel_safe', nav)
        tele = self.line(out, '    [teleop]')
        self.assertIn('python3 -m host.ros_teleop_safe', tele)
        self.assertIn('--nav-topic /fpga/nav/cmd_vel_safe', tele)
        names = self.names(out)
        self.assertLess(names.index('teleop'), names.index('navigation'))
        self.assertIn('-e TELEOP=1', self.line(out, '[container stack]'))
        # without teleop: no redirection (the guard keeps /fpga/cmd_vel_safe)
        r = self.up(*(self.ARGS + ['--dry-run']))
        self.assertNotIn('FPGA_NAV_GUARD_OUTPUT', r.stdout)
        self.assertNotIn('--nav-topic', r.stdout)

    def test_map_derived_from_rtabmap_db(self):
        db = os.path.join(self.tmp, 'rt', '20261017_000000.db')
        yaml = os.path.join(self.tmp, 'rt', 'run_20261017_000000', 'export', '20261017_000000.yaml')
        os.makedirs(os.path.dirname(yaml))
        open(db, 'w').close()
        with open(yaml, 'w') as f:
            f.write('image: x.pgm\n')
        r = self.up(*(self.ARGS + ['--dry-run']), CAR_MAP_YAML='', RTABMAP_DB=db)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('FPGA_NAV_MAP_YAML=%s ' % yaml, self.line(r.stdout, '    [navigation]'))
        stack = self.line(r.stdout, '[container stack]')
        self.assertEqual(stack.count('-e MAP_YAML='), 1, stack)
        self.assertIn('bench_pose.py --no-odom-base --map-m 0 >', self.line(r.stdout, '    [bench_pose]'))

    def test_costmap_plans_unchanged(self):
        """odom + costmap keeps its /map from bench_pose: the stand-in 20 m, or the export via --map-yaml"""
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run', CAR_MAP_YAML='')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('bench_pose.py --no-odom-base --map-m 20 >', self.line(r.stdout, '    [bench_pose]'))
        self.assertNotIn('MAP_YAML', self.line(r.stdout, '[container stack]'))
        r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--dry-run')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('bench_pose.py --no-odom-base --map-yaml %s >' % self.map_yaml,
                      self.line(r.stdout, '    [bench_pose]'))
        stack = self.line(r.stdout, '[container stack]')
        self.assertEqual(stack.count('-e MAP_YAML='), 1, stack)
        self.assertNotIn('[navigation]', r.stdout)


class TestOdomNav2Refused(BoardBase):
    def test_no_map_refused_dry_and_real(self):
        for extra in (['--dry-run'], []):
            run = os.path.join(self.tmp, 'runs', 'nomap')
            r = self.up(*(self.ARGS + extra + ['--run-dir', run]), CAR_MAP_YAML='', RTABMAP_DB='')
            self.assertEqual(r.returncode, 2, (extra, r.stdout + r.stderr))
            self.assertIn(NO_MAP, r.stderr)
            self.assertIn('CAR_MAP_YAML', r.stderr)
            self.nothing_created(run)

    def test_named_map_not_a_file_refused_at_start(self):
        for env in ({'CAR_MAP_YAML': os.path.join(self.tmp, 'missing.yaml')},
                    {'CAR_MAP_YAML': '', 'RTABMAP_DB': os.path.join(self.tmp, 'nodir', 'x.db')}):
            run = os.path.join(self.tmp, 'runs', 'badmap')
            r = self.up(*(self.ARGS + ['--run-dir', run]), **env)
            self.assertEqual(r.returncode, 2, (env, r.stdout + r.stderr))
            self.assertIn('board:', r.stderr)
            self.nothing_created(run)
            self.assertEqual(self.kv_calls(), [])                         # refused before the KV260 is even asked

    def kv_calls(self):
        p = os.path.join(self.fd, 'kv_calls.log')
        return open(p).read().splitlines() if os.path.exists(p) else []

    def test_other_refusals(self):
        for args, env, word in (
                (['live', '--pose', 'odom', '--nav', 'nav2', '--chassis', 'none'], {}, '须 --chassis uart'),
                # (self.ARGS, {'CAR_TELEOP': '1'}, '不合法：遥控配 nav2') -- 10-18 arbitration (TestOdomNav2Plan)
                (self.ARGS, {'CAR_NAV_DRYRUN': '1'}, 'CAR_NAV_DRYRUN=1 只用于板上 Nav2 空跑'),
                (self.ARGS, {'CAR_WHEEL_ODOM': '0'}, '不合法：odom 位姿配 CAR_WHEEL_ODOM=0——odom->base_link 无人发')):
            for extra in (['--dry-run'], []):
                run = os.path.join(self.tmp, 'runs', 'refused')
                r = self.up(*(args + extra + ['--run-dir', run]), **env)
                self.assertEqual(r.returncode, 2, (args, env, extra, r.stdout + r.stderr))
                self.assertIn(word, r.stderr, (args, env))
                self.assertNotIn(REMOVED, r.stderr)
                self.nothing_created(run)


class TestOdomNav2Start(BoardBase):
    def test_start_and_stop_with_standins(self):
        os.makedirs(os.path.dirname(self.node))
        os.symlink('/dev/null', self.node)
        run = os.path.join(self.tmp, 'runs', 'odomnav1')
        r = self.up(*(self.ARGS + ['--run-dir', run]))
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        with open(os.path.join(self.fd, 'wde-car-stack-odomnav1.json')) as f:
            argv = json.load(f)['argv']
        for word in ('POSE=odom', 'NAV=nav2', 'CHASSIS=uart', 'WHEEL_ODOM=1', 'BENCH_MAP_M=0',
                     'MAP_YAML=%s' % self.map_yaml, 'NAV_COSTMAP=rolling', self.port):
            self.assertIn(word, argv)
        self.assertEqual(sum(1 for a in argv if a.startswith('MAP_YAML=')), 1, argv)
        self.assertNotIn('NAV_DRYRUN=1', argv)
        with open(os.path.join(run, 'car.json')) as f:
            car = json.load(f)
        self.assertEqual((car['pose'], car['nav'], car['chassis']), ('odom', 'nav2', 'uart'))
        self.assertEqual(car['nav2_map_yaml'], self.map_yaml)
        self.assertEqual(car['bench']['stand_in_map_m'], 0.0)
        self.assertIn('不发 /map', car['bench']['stand_in_map_note'])
        self.assertFalse(car['nav_dryrun']['enabled'])
        self.assertTrue(car['wheel_odom']['enabled'])
        self.assertIn('Nav2 map_server 发的建图导出 %s' % self.map_yaml, r.stdout)
        d = self.down()
        self.assertEqual(d.returncode, 0, d.stdout + d.stderr)
        self.assertEqual(self.containers(), [])


class TestInnerNeedsMap(tcr.Base):
    """car_inner.sh on its own (DRY_RUN=1, as the container would get it) refuses odom + nav2 without MAP_YAML"""

    def inner(self, **env):
        e = {k: v for k, v in os.environ.items() if k not in ('MAP_YAML', 'NAV_DRYRUN', 'TELEOP', 'BENCH_MAP_M')}
        e.update(CAMERA='live', POSE='odom', NAV='nav2', DETECTOR='yolo', CHASSIS='uart', YOLO_BACKEND='trt',
                 DRY_RUN='1', O=os.path.join(self.tmp, 'o'), SRCD=ROOT, IPC=os.path.join(self.tmp, 'ipc'),
                 WHEEL_ODOM='1', GROUND_CALIB='off', CAM_HEIGHT='0.375', CAM_PITCH_DEG='10', HEIGHT_CELL='8',
                 CAM_MOUNT_TF='0.1 0 0.375 -0.54 0.54 -0.45 0.45', ROBOT_HEIGHT='0.31', KV_ADDR='198.51.100.79:47800')
        e.update(env)
        return subprocess.run(['bash', INNER], env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True, timeout=60)

    def test_refused_without_map(self):
        r = self.inner(MAP_YAML='')
        self.assertEqual(r.returncode, 2, r.stdout + r.stderr)
        self.assertIn('FAIL pose odom + nav2 needs MAP_YAML', r.stderr)
        self.assertNotIn('[bench_pose]', r.stdout)
        self.assertNotIn('[navigation]', r.stdout)

    def test_with_map(self):
        r = self.inner(MAP_YAML='/data/fpga/rtabmap/run_x/export/x.yaml')
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('bench_pose.py --no-odom-base --map-m 0 >', r.stdout)
        self.assertNotIn('--map-yaml', r.stdout)
        self.assertIn('FPGA_NAV_MAP_YAML=/data/fpga/rtabmap/run_x/export/x.yaml ', r.stdout)


if __name__ == '__main__':
    unittest.main()
