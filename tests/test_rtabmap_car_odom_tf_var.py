"""tools/car_run/rtabmap_car.sh ODOM_TF_LIN_VAR / ODOM_TF_ANG_VAR (10-17 定位门第 11 棒): the variance of rtabmap's TF
odometry links (ODOM=wheel). Defaults stay 0.001 / 0.001 (the yaml text before this change); positive numbers only;
the values reach the inner layer (PASS) and rtabmap.yaml."""
import os
import re
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SH = ROOT / 'tools/car_run/rtabmap_car.sh'


def run(**env):
    e = dict(os.environ)
    for k in ('ODOM_TF_LIN_VAR', 'ODOM_TF_ANG_VAR', 'GATE_COUNT_MODE', 'MODE'):
        e.pop(k, None)
    e.update(env)
    return subprocess.run(['bash', str(SH), 'map'], env=e, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True, timeout=30)


class OdomTfVarTests(unittest.TestCase):
    def test_defaults_are_the_old_yaml_values(self):
        src = SH.read_text()
        self.assertIn(': "${ODOM_TF_LIN_VAR:=0.001}" "${ODOM_TF_ANG_VAR:=0.001}"', src)
        line = next(l for l in src.splitlines() if 'odom_tf_linear_variance' in l and 'printf' in l)
        cmd = 'ODOM=wheel ODOM_TF_LIN_VAR=0.001 ODOM_TF_ANG_VAR=0.001\ncat <<YAML\n%s\nYAML\n' % line
        out = subprocess.run(['bash', '-c', cmd], stdout=subprocess.PIPE, universal_newlines=True).stdout
        self.assertEqual(out, '    odom_frame_id: odom\n    odom_tf_linear_variance: 0.001\n'
                              '    odom_tf_angular_variance: 0.001\n')

    def test_given_values_reach_the_yaml(self):
        src = SH.read_text()
        line = next(l for l in src.splitlines() if 'odom_tf_linear_variance' in l and 'printf' in l)
        cmd = 'ODOM=wheel ODOM_TF_LIN_VAR=0.000001 ODOM_TF_ANG_VAR=0.000002\ncat <<YAML\n%s\nYAML\n' % line
        out = subprocess.run(['bash', '-c', cmd], stdout=subprocess.PIPE, universal_newlines=True).stdout
        self.assertIn('odom_tf_linear_variance: 0.000001\n', out)
        self.assertIn('odom_tf_angular_variance: 0.000002\n', out)
        cmd = 'ODOM=visual ODOM_TF_LIN_VAR=1 ODOM_TF_ANG_VAR=1\ncat <<YAML\n%s\nYAML\n' % line
        self.assertEqual(subprocess.run(['bash', '-c', cmd], stdout=subprocess.PIPE, universal_newlines=True).stdout,
                         '\n')                                  # visual odometry: no TF odometry keys

    def test_passed_into_the_container(self):
        src = SH.read_text()
        pas = re.search(r'PASS=\((.*?)\)', src, re.S).group(1).split()
        self.assertIn('ODOM_TF_LIN_VAR', pas)
        self.assertIn('ODOM_TF_ANG_VAR', pas)

    def test_bad_values_refused(self):
        for v in ('0', '-1', 'abc', '1e-', '1.2.3'):   # '' = unset (:=) -> the default
            for k in ('ODOM_TF_LIN_VAR', 'ODOM_TF_ANG_VAR'):
                r = run(**{k: v})
                self.assertEqual(r.returncode, 2, (k, v, r.stderr))
                self.assertIn('%s must be a positive number' % k, r.stderr)

    def test_good_values_pass_the_check(self):
        # past the check, the next one (GATE_COUNT_MODE) refuses: so the variance check let the value through
        for v in ('0.001', '0.000001', '1e-6', '.5', '2'):
            r = run(ODOM_TF_LIN_VAR=v, ODOM_TF_ANG_VAR=v, GATE_COUNT_MODE='nonsense')
            self.assertEqual(r.returncode, 2, r.stderr)
            self.assertIn('GATE_COUNT_MODE must be', r.stderr)
            self.assertNotIn('positive number', r.stderr)


if __name__ == '__main__':
    unittest.main()
