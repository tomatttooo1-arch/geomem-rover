"""10-29: the car's DWB goal critics aim at a point on the path ahead (forward_prune_distance 0.8 m), not at a goal
within 2 m that the path first leads away from (host sim: the car turned to the detour path, then turned back and
forth on the spot until the 80 s timeout).

Known answers: car profile (DWB alone, inside the rotation shim, rolling-rebuilt) -> FollowPath forward_prune_distance
0.8 and shorten_transformed_plan true; FPGA_NAV_DWB_FORWARD_PRUNE_M overrides (2.0 = the behaviour before); the
simulation profile keeps Nav2's default (no key); refused values raise. car_up CAR_DWB_FORWARD_PRUNE_M -> container
-e DWB_FORWARD_PRUNE_M -> board launch line FPGA_NAV_DWB_FORWARD_PRUNE_M; unset = the lines of before; refused = exit 2.

    PYTHONPATH=. python3 tests/test_nav_turn_start_20261029.py
"""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_car_run as tcr   # noqa: E402
import test_nav_rehang_launch as tnl   # noqa: E402

INPUTS = tnl.runpy_inputs(tnl.ASSETS)
BAD = ('abc', '0', '0.2', '-1', '4.5', 'nan')


def follow(params):
    return params['controller_server']['ros__parameters']['FollowPath']


class TestProfile(unittest.TestCase):
    def test_known_answers(self):
        f = INPUTS['car_forward_prune']
        self.assertEqual(f({}), (0.8, False))
        self.assertEqual(f({'FPGA_NAV_DWB_FORWARD_PRUNE_M': ' '}), (0.8, False))
        self.assertEqual(f({'FPGA_NAV_DWB_FORWARD_PRUNE_M': '2.0'}), (2.0, True))
        self.assertEqual(f({'FPGA_NAV_DWB_FORWARD_PRUNE_M': '0.3'}), (0.3, True))   # both ends inclusive
        self.assertEqual(f({'FPGA_NAV_DWB_FORWARD_PRUNE_M': '4'}), (4.0, True))
        for raw in BAD:
            with self.assertRaises(ValueError, msg=raw):
                f({'FPGA_NAV_DWB_FORWARD_PRUNE_M': raw})

    def test_apply_dwb_only(self):
        apply = INPUTS['apply_car_forward_prune']
        dwb = {'controller_server': {'ros__parameters': {'FollowPath': {'plugin': 'dwb_core::DWBLocalPlanner'}}}}
        out = apply(dwb)
        self.assertEqual(follow(out), {'plugin': 'dwb_core::DWBLocalPlanner', 'forward_prune_distance': 0.8,
                                       'shorten_transformed_plan': True})
        self.assertNotIn('forward_prune_distance', follow(dwb))   # a copy
        shim = INPUTS['apply_rotation_shim'](dwb)
        self.assertEqual(follow(apply(shim, 2.0))['forward_prune_distance'], 2.0)
        rpp = {'controller_server': {'ros__parameters': {'FollowPath': {
            'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'}}}}
        self.assertEqual(apply(rpp), rpp)


class TestLaunch(unittest.TestCase):
    def setUp(self):
        self.t = tnl.RealCarEntryTests('test_car_dwb_min_speeds')
        self.t.setUp()
        self.addCleanup(self.t.doCleanups)
        self.car = dict(tnl.CAR_ENV, FPGA_NAV_MAP_YAML=str(self.t.share / 'maps/car.yaml'))

    def test_car_reads_sim_does_not(self):
        for env in (self.car, dict(self.car, FPGA_NAV_ROTATION_SHIM='false')):
            _, params, lines = self.t.actions(env)
            self.assertEqual(follow(params)['forward_prune_distance'], 0.8)
            self.assertIs(follow(params)['shorten_transformed_plan'], True)
            self.assertEqual(lines[-1]['dwb_forward_prune_m'], 0.8)
            self.assertNotIn('dwb_forward_prune_given', lines[-1])
        _, params, lines = self.t.actions(dict(self.car, FPGA_NAV_DWB_FORWARD_PRUNE_M='2.0'))
        self.assertEqual(follow(params)['forward_prune_distance'], 2.0)
        self.assertEqual((lines[-1]['dwb_forward_prune_m'], lines[-1]['dwb_forward_prune_given']), (2.0, True))
        with self.assertRaises(ValueError):
            self.t.actions(dict(self.car, FPGA_NAV_DWB_FORWARD_PRUNE_M='0.1'))
        _, params, lines = self.t.actions({'FPGA_NAV_DWB_FORWARD_PRUNE_M': '9'})   # simulation: not read, not checked
        self.assertNotIn('forward_prune_distance', follow(params))
        self.assertFalse(any('real_car' in line for line in lines))
        with self.t.without('apply_car_forward_prune'):   # positive control: the key comes from this step
            _, params, _ = self.t.actions(self.car)
            self.assertNotIn('forward_prune_distance', follow(params))
        _, params, _ = self.t.actions(self.t.rolling_share())   # the rolling rebuild of FollowPath: last (fixture)
        self.assertEqual(follow(params)['forward_prune_distance'], 0.8)


class TestCarUpPassthrough(tcr.Base):
    setUp = tcr.TestLiveBoard.setUp
    line = tcr.TestLiveBoard.line
    make_node = tcr.TestLiveBoard.make_node

    def test_dry_run_unset_given_refused(self):
        r = self.up('live', '--dry-run', FAKE_USB='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        stack, nav = self.line(r.stdout, '[container stack]'), self.line(r.stdout, '    [navigation]')
        self.assertNotIn('FORWARD_PRUNE', stack)
        self.assertNotIn('FORWARD_PRUNE', nav)
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_DWB_FORWARD_PRUNE_M='2.0')
        self.assertEqual(r.returncode, 0, r.stderr)
        stack, nav = self.line(r.stdout, '[container stack]'), self.line(r.stdout, '    [navigation]')
        self.assertIn(' -e DWB_FORWARD_PRUNE_M=2.0 ', stack)
        self.assertIn(' FPGA_NAV_DWB_FORWARD_PRUNE_M=2.0 ros2 launch ', nav)
        for i, bad in enumerate(BAD):
            run = os.path.join(self.tmp, 'bad%d' % i)
            r = self.up('live', '--run-dir', run, FAKE_USB='1', CAR_DWB_FORWARD_PRUNE_M=bad)
            self.assertEqual(r.returncode, 2, (bad, r.stdout, r.stderr))
            self.assertIn('CAR_DWB_FORWARD_PRUNE_M', r.stderr)
            self.assertFalse(os.path.exists(run))

    def test_car_json(self):
        self.make_node()
        port = os.path.join(self.tmp, 'fpga_chassis')
        open(port, 'w').close()
        for name, env, want in (('fp0', {}, {'m': 0.8, 'given': False}),
                                ('fp1', {'CAR_DWB_FORWARD_PRUNE_M': '1.5'}, {'m': 1.5, 'given': True})):
            run = os.path.join(self.tmp, 'runs', name)
            r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--run-dir', run, FAKE_USB='1', RTABMAP_DB='',
                        CHASSIS_PORT=port, **env)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            car = json.loads(open(os.path.join(run, 'car.json')).read())
            self.assertEqual({k: car['dwb_forward_prune'][k] for k in ('m', 'given')}, want)
            argv = json.loads(open(os.path.join(self.fd, 'wde-car-stack-%s.json' % name)).read())['argv']
            self.assertEqual(bool([x for x in argv if 'DWB_FORWARD_PRUNE_M' in x]), bool(env))
            d = self.down()
            self.assertEqual(d.returncode, 0, d.stdout + d.stderr)


if __name__ == '__main__':
    unittest.main()
