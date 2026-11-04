"""10-26: the car's Nav2 switches and DWB minimum speeds reach the launch from the Nano's environment.

Before: FPGA_NAV_ROTATION_SHIM / FPGA_NAV_PROGRESS_CHECKER / FPGA_NAV_CAR_PLANNER appeared nowhere in car_up.sh or
car_inner.sh; the stack container gets only its -e list, so an export on the Nano never reached Nav2. The DWB
min_speed_theta / min_speed_xy and the StoppedGoalChecker rot_stopped_velocity were code constants only.
Now: car_up (CAR_DWB_MIN_SPEED_THETA / CAR_DWB_MIN_SPEED_XY / CAR_ROT_STOPPED_VELOCITY and the three switches,
each only when given) -> INNER_ENV (-e) -> car_inner NAVE (board) -> navigation.launch.py (car only).
Known answers on every layer; unset = the old lines; refused values exit 2 before anything starts.
"""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_car_run as tcr   # noqa: E402
import test_nav_rehang_launch as tnl   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INPUTS = tnl.runpy_inputs(tnl.ASSETS)
NEW_NAMES = ('FPGA_NAV_ROTATION_SHIM', 'FPGA_NAV_PROGRESS_CHECKER', 'FPGA_NAV_CAR_PLANNER', 'DWB_MIN_SPEED',
             'ROT_STOPPED_VELOCITY')
ALL_GIVEN = dict(FPGA_NAV_ROTATION_SHIM='false', FPGA_NAV_PROGRESS_CHECKER='simple', FPGA_NAV_CAR_PLANNER='relaxed',
                 CAR_DWB_MIN_SPEED_THETA='0.12', CAR_DWB_MIN_SPEED_XY='0.02', CAR_ROT_STOPPED_VELOCITY='0.2')
BAD = (dict(CAR_DWB_MIN_SPEED_THETA='0.2'),                     # > the default rot_stopped 0.15
       dict(CAR_ROT_STOPPED_VELOCITY='0.10'),                   # == the default theta: not above
       dict(CAR_DWB_MIN_SPEED_THETA='0.3', CAR_ROT_STOPPED_VELOCITY='0.25'),
       dict(CAR_DWB_MIN_SPEED_THETA='0.5', CAR_ROT_STOPPED_VELOCITY='0.6'),   # caps 0.4 / 0.4
       dict(CAR_DWB_MIN_SPEED_XY='0.2'), dict(CAR_DWB_MIN_SPEED_XY='0'), dict(CAR_DWB_MIN_SPEED_XY='-0.01'),
       dict(CAR_DWB_MIN_SPEED_XY='abc'), dict(CAR_ROT_STOPPED_VELOCITY='nan'),
       dict(FPGA_NAV_ROTATION_SHIM='yes'), dict(FPGA_NAV_PROGRESS_CHECKER='posee'), dict(FPGA_NAV_CAR_PLANNER='loose'))


def launch_env(car_up_env):
    """car_up's names -> the launch's (car_inner NAVE does this mapping)."""
    names = {'CAR_DWB_MIN_SPEED_THETA': 'FPGA_NAV_DWB_MIN_SPEED_THETA', 'CAR_DWB_MIN_SPEED_XY': 'FPGA_NAV_DWB_MIN_SPEED_XY',
             'CAR_ROT_STOPPED_VELOCITY': 'FPGA_NAV_ROT_STOPPED_VELOCITY'}
    return {names.get(k, k): v for k, v in car_up_env.items()}


class TestProfileOverrides(unittest.TestCase):
    def test_known_answers(self):
        f = INPUTS['car_min_speed_overrides']
        self.assertEqual(f({}), ({'min_speed_xy': 0.015, 'min_speed_theta': 0.10}, 0.15, {}))
        self.assertEqual(f({'FPGA_NAV_DWB_MIN_SPEED_THETA': '', 'FPGA_NAV_DWB_MIN_SPEED_XY': ' '}), f({}))
        self.assertEqual(f(launch_env(ALL_GIVEN)), ({'min_speed_xy': 0.02, 'min_speed_theta': 0.12}, 0.2,
                                                    {'min_speed_xy': 0.02, 'min_speed_theta': 0.12,
                                                     'rot_stopped_velocity': 0.2}))
        self.assertEqual(f({'FPGA_NAV_DWB_MIN_SPEED_XY': '0.03'}),
                         ({'min_speed_xy': 0.03, 'min_speed_theta': 0.10}, 0.15, {'min_speed_xy': 0.03}))
        self.assertEqual(f({'FPGA_NAV_DWB_MIN_SPEED_THETA': '0.3', 'FPGA_NAV_ROT_STOPPED_VELOCITY': '0.4'})[:2],
                         ({'min_speed_xy': 0.015, 'min_speed_theta': 0.3}, 0.4))   # both caps inclusive
        for bad in BAD:
            env = launch_env({k: v for k, v in bad.items() if not k.startswith('FPGA_NAV_')})
            if env:
                with self.assertRaises(ValueError, msg=bad):
                    f(env)
        self.assertEqual(INPUTS['CAR_DWB_MIN_SPEEDS'], {'min_speed_xy': 0.015, 'min_speed_theta': 0.10})   # untouched


class TestLaunchReadsOverrides(unittest.TestCase):
    def setUp(self):
        self.t = tnl.RealCarEntryTests('test_car_dwb_min_speeds')
        self.t.setUp()
        self.addCleanup(self.t.doCleanups)
        self.car = dict(tnl.CAR_ENV, FPGA_NAV_MAP_YAML=str(self.t.share / 'maps/car.yaml'))

    def test_car_reads_sim_does_not(self):
        over = launch_env({k: v for k, v in ALL_GIVEN.items() if k.startswith('CAR_')})
        _, params, lines = self.t.actions(dict(self.car, **over))
        self.assertEqual(self.t.min_speeds(params), (0.02, 0.12, 0.2))
        self.assertEqual(lines[-1]['dwb_min_speeds'], {'min_speed_xy': 0.02, 'min_speed_theta': 0.12})
        self.assertEqual(lines[-1]['dwb_min_speed_override'],
                         {'min_speed_xy': 0.02, 'min_speed_theta': 0.12, 'rot_stopped_velocity': 0.2,
                          'effective': {'min_speed_xy': 0.02, 'min_speed_theta': 0.12, 'rot_stopped_velocity': 0.2}})
        _, params, lines = self.t.actions(self.car)   # unset: the constants, no override key
        self.assertEqual(self.t.min_speeds(params), (0.015, 0.10, 0.15))
        self.assertNotIn('dwb_min_speed_override', lines[-1])
        self.assertEqual(lines[-1]['dwb_min_speeds'], {'min_speed_xy': 0.015, 'min_speed_theta': 0.10})
        _, params, _ = self.t.actions(over)            # simulation: not read
        self.assertEqual(self.t.min_speeds(params), (0.0, 0.0, 0.03))
        _, params, _ = self.t.actions(dict(over, FPGA_NAV_DWB_MIN_SPEED_THETA='0.9'))   # not even checked there
        self.assertEqual(self.t.min_speeds(params), (0.0, 0.0, 0.03))
        with self.assertRaises(ValueError):
            self.t.actions(dict(self.car, FPGA_NAV_DWB_MIN_SPEED_THETA='0.2'))
        # the three switches through the same environment (the launch side existed; here with the passed names)
        _, params, lines = self.t.actions(dict(self.car, FPGA_NAV_ROTATION_SHIM='false', FPGA_NAV_PROGRESS_CHECKER='simple',
                                               FPGA_NAV_CAR_PLANNER='relaxed'))
        self.assertIsNone(lines[-1]['rotation_shim'])
        self.assertEqual(lines[-1]['progress_checker']['plugin'], 'nav2_controller::SimpleProgressChecker')
        self.assertEqual(lines[-1]['planner'], {'tolerance': 0.25, 'allow_unknown': True})


class TestCarUpPassthrough(tcr.Base):
    setUp = tcr.TestLiveBoard.setUp
    line = tcr.TestLiveBoard.line
    make_node = tcr.TestLiveBoard.make_node

    def test_dry_run_unset_given_refused(self):
        r = self.up('live', '--dry-run', FAKE_USB='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        stack, nav = self.line(r.stdout, '[container stack]'), self.line(r.stdout, '    [navigation]')
        for name in NEW_NAMES:
            self.assertNotIn(name, stack)
            self.assertNotIn(name, nav)
        self.assertIn(' FPGA_NAV_COSTMAP=rolling ros2 launch ', nav)   # unset: the NAVE of before ends the env list
        r = self.up('live', '--dry-run', FAKE_USB='1', **ALL_GIVEN)
        self.assertEqual(r.returncode, 0, r.stderr)
        stack, nav = self.line(r.stdout, '[container stack]'), self.line(r.stdout, '    [navigation]')
        for kv in ('FPGA_NAV_ROTATION_SHIM=false', 'FPGA_NAV_PROGRESS_CHECKER=simple', 'FPGA_NAV_CAR_PLANNER=relaxed',
                   'DWB_MIN_SPEED_THETA=0.12', 'DWB_MIN_SPEED_XY=0.02', 'ROT_STOPPED_VELOCITY=0.2'):
            self.assertIn(' -e %s ' % kv, stack)
        self.assertIn(' FPGA_NAV_COSTMAP=rolling FPGA_NAV_ROTATION_SHIM=false FPGA_NAV_PROGRESS_CHECKER=simple '
                      'FPGA_NAV_CAR_PLANNER=relaxed FPGA_NAV_DWB_MIN_SPEED_THETA=0.12 FPGA_NAV_DWB_MIN_SPEED_XY=0.02 '
                      'FPGA_NAV_ROT_STOPPED_VELOCITY=0.2 ros2 launch ', nav)
        r = self.up('live', '--dry-run', FAKE_USB='1', FPGA_NAV_ROTATION_SHIM='FALSE')   # the launch lowercases too
        self.assertEqual(r.returncode, 0, r.stderr)
        for i, bad in enumerate(BAD):
            run = os.path.join(self.tmp, 'bad%d' % i)
            r = self.up('live', '--run-dir', run, FAKE_USB='1', **bad)
            self.assertEqual(r.returncode, 2, (bad, r.stdout, r.stderr))
            self.assertIn(next(iter(bad)) if next(iter(bad)).startswith('FPGA_NAV_') else 'CAR_DWB_MIN_SPEED_THETA /',
                          r.stderr)
            self.assertFalse(os.path.exists(run))

    def test_car_json_and_container_argv(self):
        self.make_node()
        port = os.path.join(self.tmp, 'fpga_chassis')
        open(port, 'w').close()
        for name, env in (('nv0', {}), ('nv1', ALL_GIVEN)):
            run = os.path.join(self.tmp, 'runs', name)
            r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--run-dir', run, FAKE_USB='1', RTABMAP_DB='',
                        CHASSIS_PORT=port, **env)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            car = json.loads(open(os.path.join(run, 'car.json')).read())
            argv = json.loads(open(os.path.join(self.fd, 'wde-car-stack-%s.json' % name)).read())['argv']
            if env:
                self.assertEqual(car['nav_switches']['given'], {'FPGA_NAV_ROTATION_SHIM': 'false',
                                                                'FPGA_NAV_PROGRESS_CHECKER': 'simple',
                                                                'FPGA_NAV_CAR_PLANNER': 'relaxed'})
                want = {'min_speed_xy': 0.02, 'min_speed_theta': 0.12, 'rot_stopped_velocity': 0.2,
                        'overridden': ['min_speed_theta', 'min_speed_xy', 'rot_stopped_velocity']}
                for kv in ('FPGA_NAV_ROTATION_SHIM=false', 'DWB_MIN_SPEED_THETA=0.12', 'ROT_STOPPED_VELOCITY=0.2'):
                    self.assertIn(kv, argv)
            else:
                self.assertEqual(car['nav_switches']['given'], {})
                want = {'min_speed_xy': 0.015, 'min_speed_theta': 0.10, 'rot_stopped_velocity': 0.15, 'overridden': []}
                self.assertFalse([a for a in argv if any(n in a for n in NEW_NAMES)])
            self.assertEqual({k: car['dwb_min_speeds'][k] for k in want}, want)
            d = self.down()
            self.assertEqual(d.returncode, 0, d.stdout + d.stderr)


class TestTreeWithoutRobotSim(tcr.Base):
    """a deployed tree without robot_sim (bench archives, test_car_run.test_commit_file_tree_snapshot): unset = no
    check (dry run 0); a DWB override given = refused (2), nothing to validate it against."""
    setUp = tcr.TestLiveBoard.setUp

    def test_no_robot_sim(self):
        import shutil
        dep = os.path.join(self.tmp, 'deployed')
        for d in ('host', 'tools'):
            shutil.copytree(os.path.join(ROOT, d), os.path.join(dep, d),
                            ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        with open(os.path.join(dep, 'COMMIT'), 'w') as f:
            f.write('0123456789abcdef0123456789abcdef01234567\n')
        up = os.path.join(dep, 'tools', 'car_run', 'car_up.sh')
        args = ['bash', up] + tcr.TestLiveBoard.STATIC + ['--dry-run']
        import subprocess
        r = subprocess.run(args, env=dict(self.env, RTABMAP_DB=''), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=120)
        self.assertNotIn('被拒', r.stderr)
        r = subprocess.run(args, env=dict(self.env, RTABMAP_DB='', CAR_DWB_MIN_SPEED_THETA='0.12'),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=120)
        self.assertEqual(r.returncode, 2, r.stderr)
        self.assertIn('本部署树没有', r.stderr)


class TestNanoEnvReachesCarUp(unittest.TestCase):
    """`VAR=value bash console_remote.sh up` on the Nano: console_remote starts nano_up.sh with setsid nohup (the
    environment is inherited) and nano_up runs car_up through `env A=B ...` (adds, never clears)."""

    def test_no_environment_scrub(self):
        d = os.path.join(ROOT, 'ui', 'geomem', 'tools', 'competition')
        remote = open(os.path.join(d, 'console_remote.sh')).read()
        up = open(os.path.join(d, 'nano_up.sh')).read()
        self.assertIn('setsid nohup bash "$here/nano_up.sh"', remote)
        self.assertIn('args=(env CAR_BOARD_DRAFT=1 ', up)
        self.assertIn('"${args[@]}" bash tools/car_run/car_up.sh live --pose odom --nav nav2\n', up)
        for text in (remote, up):
            self.assertNotIn('env -i', text)
            self.assertNotIn('sudo', text.replace('sudo -n /usr/local/sbin/geomem-kv-start', ''))


if __name__ == '__main__':
    unittest.main()
