"""In-stack mapping beside Nav2 (2026-10-18, car_up CAR_MAPPING=1; docs/design/TELEOP_NAV_ARB_20261018.md, M1-M6).

Executed: blank_map.py (read back through bench_pose.load_map_yaml, the map_server trinary rules), mapping_standin.py
(as a process, SIGINT -> export), mapping_ctl.MappingCore (state machine), car_validate and the car_up dry-run plans.
Not executed here: rtabmap itself (board), ROS (the container run of the design doc section 5).
"""
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tools', 'car_run'))

import blank_map  # noqa: E402
import mapping_ctl  # noqa: E402
from bench_pose import load_map_yaml  # noqa: E402
import test_odom_nav2 as ton  # noqa: E402

COMMON = os.path.join(ROOT, 'tools', 'car_run', 'car_common.sh')


class TestBlankMap(unittest.TestCase):
    def test_m1_blank_is_all_free_and_centred(self):
        with tempfile.TemporaryDirectory() as d:
            y = blank_map.write_blank(d, 2.0, 0.05)
            w, h, res, origin, data = load_map_yaml(y)
            self.assertEqual((w, h, res), (40, 40, 0.05))
            self.assertEqual(origin, [-1.0, -1.0, 0.0])
            self.assertEqual(set(data), {0})                      # FREE, never unknown (the runner: unknown = occupied)
            for bad in ((0, 0.05), (2.0, 0), (float('nan'), 0.05), (1000.0, 0.05)):
                with self.assertRaises(ValueError):
                    blank_map.write_blank(d, *bad)


class TestStandin(unittest.TestCase):
    def run_standin(self, **env):
        d = tempfile.mkdtemp()
        e = dict(os.environ, OUT=os.path.join(d, 'out'), DB_DIR=os.path.join(d, 'db'), RUN_TIME='t1', **env)
        p = subprocess.Popen([sys.executable, os.path.join(ROOT, 'tools/car_run/mapping_standin.py')], env=e,
                             stderr=subprocess.PIPE, universal_newlines=True)
        line = ''
        while 'running --' not in line:
            line = p.stderr.readline()
            self.assertTrue(line, 'stand-in ended before running --')
        p.send_signal(signal.SIGINT)
        err = p.stderr.read()
        p.wait(10)
        p.stderr.close()
        return p.returncode, err, d

    def test_m2_standin_exports_a_map_with_a_wall(self):
        rc, err, d = self.run_standin()
        self.assertEqual(rc, 0, err)
        self.assertIn('export done', err)
        y = os.path.join(d, 'out', 'export', 't1.yaml')
        w, h, res, origin, data = load_map_yaml(y)
        self.assertIn(100, data)                                  # the wall: a swap is visible
        self.assertEqual(sorted(os.listdir(os.path.join(d, 'out', 'export'))), ['t1.pgm', 't1.yaml'])
        self.assertTrue(os.path.exists(os.path.join(d, 'db', 't1.db')))

    def test_standin_failure_path(self):
        rc, err, d = self.run_standin(STANDIN_FAIL='1')
        self.assertEqual(rc, 5)
        self.assertFalse(os.path.exists(os.path.join(d, 'out', 'export')))


class TestMappingCore(unittest.TestCase):
    def core(self, d):
        t = [0.0]
        return mapping_ctl.MappingCore(d, d + '/db', clock=lambda: t[0]), t

    def test_m3_stop_save_export(self):
        with tempfile.TemporaryDirectory() as d:
            c, _ = self.core(d)
            self.assertEqual(c.state, 'mapping')
            self.assertTrue(c.request_stop())
            self.assertFalse(c.request_stop())                    # one SIGINT only
            self.assertEqual(c.state, 'saving')
            os.makedirs(os.path.join(d, 'export'))
            blank_map.write_blank(os.path.join(d, 'export'), 1.0, 0.1)
            c.on_child_exit(0)
            self.assertEqual(c.state, 'exported')
            self.assertTrue(c.yaml.endswith('export/blank.yaml'))
            c.on_load(True, 0)
            self.assertEqual(c.status()['map_loaded'], True)
            self.assertFalse(c.fatal)

    def test_m4_load_failure_keeps_exported_and_reports(self):
        with tempfile.TemporaryDirectory() as d:
            c, _ = self.core(d)
            c.request_stop()
            os.makedirs(os.path.join(d, 'export'))
            blank_map.write_blank(os.path.join(d, 'export'), 1.0, 0.1)
            c.on_child_exit(0)
            c.on_load(False, reason='service /map_server/load_map not available')
            s = c.status()
            self.assertEqual((s['state'], s['map_loaded']), ('exported', False))
            self.assertIn('blank start map stays', s['reason'])
            self.assertFalse(c.fatal)                             # the stack is not stopped (ruling 10-18)

    def test_m5_failures(self):
        with tempfile.TemporaryDirectory() as d:
            c, _ = self.core(d)
            c.request_stop()
            c.on_child_exit(5)
            self.assertEqual(c.state, 'failed')
            self.assertFalse(c.fatal)
            c2, _ = self.core(d)
            c2.request_stop()
            c2.on_child_exit(0)                                   # no export yaml
            self.assertEqual(c2.state, 'failed')
            self.assertIn('0 yaml', c2.reason)
            c3, _ = self.core(d)
            c3.on_child_exit(1)                                   # died while mapping: fatal -> exit 3 (fail-stop)
            self.assertEqual(c3.state, 'failed')
            self.assertTrue(c3.fatal)


def validate(**env):
    e = {k: v for k, v in os.environ.items() if k not in ('NAV_DRYRUN', 'TELEOP', 'MAPPING')}
    e.update(env)
    r = subprocess.run(['bash', '-c', 'source "$1"; car_validate', '_', COMMON], env=e, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
    return r.returncode, r.stderr


class TestValidate(unittest.TestCase):
    CAR = dict(CAMERA='live', POSE='odom', NAV='nav2', DETECTOR='yolo', CHASSIS='uart')

    def test_m6_combinations(self):
        self.assertEqual(validate(MAPPING='1', **self.CAR), (0, ''))
        self.assertEqual(validate(MAPPING='1', TELEOP='1', **self.CAR), (0, ''))
        sim = dict(CAMERA='sim', POSE='reference', NAV='nav2', DETECTOR='yolo', CHASSIS='pty')
        self.assertEqual(validate(MAPPING='1', **sim), (0, ''))
        for env, word in ((dict(self.CAR, NAV='costmap'), '须 --nav nav2'),
                          (dict(self.CAR, POSE='rtabmap', CHASSIS='uart'), '并栈建图用轮式里程位姿 --pose odom'),
                          (dict(CAMERA='replay', POSE='reference', NAV='costmap', DETECTOR='stub', CHASSIS='none'),
                           '并栈建图只接真车')):
            rc, err = validate(MAPPING='1', **env)
            self.assertEqual(rc, 2, env)
            self.assertIn(word, err, env)
        self.assertEqual(validate(MAPPING='2', **self.CAR)[0], 2)


class TestPlans(ton.BoardBase):
    def test_board_plan_mapping_with_teleop(self):
        r = self.up(*(self.ARGS + ['--dry-run']), CAR_MAPPING='1', CAR_TELEOP='1', CAR_MAP_YAML='')
        self.assertEqual(r.returncode, 0, r.stderr)
        out = r.stdout
        self.assertIn('blank_map.py --out-dir', out)
        m = self.line(out, '    [mapping]')
        for word in ('tools/car_run/mapping_ctl.py', 'MODE=map', 'ODOM=wheel', 'RTAB_PUBLISH_TF=false',
                     'MAP_NS=rtabmap', 'bash tools/car_run/rtabmap_car.sh map'):
            self.assertIn(word, m)
        self.assertNotIn('--standin', m)
        nav = self.line(out, '    [navigation]')
        self.assertIn('/blank_map/blank.yaml', nav)
        names = self.names(out)
        self.assertLess(names.index('bench_pose'), names.index('mapping'))
        self.assertLess(names.index('mapping'), names.index('navigation'))
        stack = self.line(out, '[container stack]')
        self.assertIn('-e MAPPING=1', stack)
        # a given map is refused with CAR_MAPPING=1
        r = self.up(*(self.ARGS + ['--dry-run']), CAR_MAPPING='1')
        self.assertEqual(r.returncode, 2)
        self.assertIn('不合法：CAR_MAPPING=1 配 CAR_MAP_YAML/RTABMAP_DB', r.stderr)


if __name__ == '__main__':
    unittest.main()
