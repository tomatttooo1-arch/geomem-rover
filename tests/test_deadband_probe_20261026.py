"""Known-answer tests of tools/chassis_uart/deadband_probe.py (2026-10-26).

A fake MCU (mcu_sim.McuModel with an injected dead band of 0.04 rad/s / 0.015 m/s) must give min effective
0.04 rad/s and 0.02 m/s, recommendations 0.06 / 0.03; a latched MCU (no heartbeat) must be reported as
latched, not as a dead band; every abort (exception, Ctrl-C, heartbeat loss, status loss) must end with
enable=0 frames and a stopped fake MCU. One smoke run over a pty against the real mcu_sim.py process.
"""
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from host.chassis_uart import decode_frame
from tools.chassis_uart import deadband_probe as dp
from tools.chassis_uart.mcu_sim import Status2Faults

ROOT = Path(__file__).resolve().parents[1]
SHORT = dict(w_step_s=1.0, v_step_s=1.0, pause_s=0.3)


def quiet(*_):
    pass


def last_command(transport):
    return decode_frame(transport.written[-1])


def run(model, plan=None, **kw):
    t = dp.SimTransport(model, status_until_ms=kw.pop('status_until_ms', None))
    code, result = dp.run_probe(t, plan or dp.build_plan(), log=quiet, **kw)
    return t, code, result


class KnownDeadband(unittest.TestCase):
    def check_known_answer(self, status2):
        t, code, r = run(dp.make_deadband_mcu(0.04, 0.015, status2=status2), status2=status2)
        self.assertEqual(code, 0, r.get('aborted'))
        self.assertIsNone(r['aborted'])
        self.assertEqual(r['w']['min_effective'], 0.04)
        self.assertEqual(r['w']['recommended_min_speed_theta'], 0.06)
        self.assertEqual(r['v']['min_effective'], 0.02)
        self.assertEqual(r['v']['recommended_min_speed_xy'], 0.03)
        self.assertEqual(r['w']['levels_moved'], {'0.02': False, '0.04': True, '0.06': True, '0.08': True,
                                                  '0.1': True, '0.15': True})
        self.assertEqual(r['v']['levels_moved'], {'0.005': False, '0.01': False, '0.02': True, '0.03': True,
                                                  '0.05': True})
        self.assertFalse(r['w']['non_monotone'] or r['v']['non_monotone'])
        for row in r['table']:
            self.assertNotIn('invalid', row)
            if row['moved']:
                self.assertTrue(row['sign_ok'])
                self.assertAlmostEqual(row['ratio'], 1.0, delta=0.03)
        # ends stopped: enable=0 frames, fake MCU executes zero
        self.assertGreaterEqual(r['zero_frames_at_end'], 20)
        self.assertFalse(last_command(t)['enable'])
        self.assertEqual(t.model.executed(), (0., 0.))
        # the frame type the probe asked for is the one it got
        frames = {row.get('frame', '0x81') for row in r['rows'] if row['dir'] == 'rx'}
        self.assertEqual(frames, {'0x82'} if status2 else {'0x81'})

    def test_known_answer_0x82(self):
        self.check_known_answer(True)

    def test_known_answer_0x81(self):
        self.check_known_answer(False)

    def test_gyro_bias_is_removed(self):
        # 0x82 gyro is raw: 0.5 deg/s of bias (0.0087 rad/s, 44 % of the 0.02 level) must not make 0.02 "move"
        model = dp.make_deadband_mcu(0.04, 0.015, status2_faults=Status2Faults(bias_dps=0.5, lx_ly_mm=197.))
        _, code, r = run(model)
        self.assertEqual(code, 0)
        self.assertAlmostEqual(r['bias_rad_s'], 0.5 * 3.141592653589793 / 180, delta=0.0005)
        self.assertEqual(r['w']['min_effective'], 0.04)
        self.assertEqual(r['w']['recommended_min_speed_theta'], 0.06)
        # unremoved bias would read +0.0487 / -0.0313 at +-0.04 (ratio 1.22 / 0.78)
        for row in r['table']:
            if row['moved']:
                self.assertAlmostEqual(row['ratio'], 1.0, delta=0.05, msg=row)

    def test_other_deadband_values(self):
        _, code, r = run(dp.make_deadband_mcu(0.07, 0.025), dp.build_plan(**SHORT))
        self.assertEqual(code, 0)
        self.assertEqual((r['w']['min_effective'], r['w']['recommended_min_speed_theta']), (0.08, 0.12))
        self.assertEqual((r['v']['min_effective'], r['v']['recommended_min_speed_xy']), (0.03, 0.05))

    def test_nothing_moves_gives_exit_1_and_null(self):
        _, code, r = run(dp.make_deadband_mcu(0.5, 0.5), dp.build_plan(**SHORT))
        self.assertEqual(code, 1)
        self.assertIsNone(r['w']['min_effective'])
        self.assertIsNone(r['v']['recommended_min_speed_xy'])


class Safety(unittest.TestCase):
    def assert_stopped(self, t, r):
        self.assertGreaterEqual(r['zero_frames_at_end'], 20)
        tail = [decode_frame(f) for f in t.written[-20:]]
        self.assertTrue(all(not c['enable'] and c['v_mm_s'] == 0 and c['w_mrad_s'] == 0 for c in tail))
        self.assertEqual(t.model.executed(), (0., 0.))

    def test_no_heartbeat_is_latched_not_deadband(self):
        t, code, r = run(dp.make_deadband_mcu(heartbeat=False))
        self.assertEqual(code, 2)
        self.assertIn('NOT a dead band', r['aborted'])
        self.assertNotIn('table', r)                       # no dead-band verdict at all
        self.assertFalse(any(row['dir'] == 'tx' and str(row['phase']).startswith('step') for row in r['rows']))
        self.assert_stopped(t, r)

    def test_heartbeat_lost_mid_level_aborts(self):
        model = dp.make_deadband_mcu(heartbeat_fn=lambda ms: ms < 8000)    # inside the rotation levels
        t, code, r = run(model)
        self.assertEqual(code, 2)
        self.assertIn('stop_latched', r['aborted'])
        self.assert_stopped(t, r)
        invalid = [row for row in r['table'] if row.get('invalid') == 'latched']
        self.assertEqual(len(invalid), 1)                   # the interrupted level is not judged
        self.assertFalse(invalid[0]['moved'])

    def test_short_heartbeat_gap_retries_the_level(self):
        # 40 ms without heartbeat inside the +0.04 rad/s level (step index 2, ~10.5-13.5 s): MCU latches (30 ms rule),
        # unlocks 100 ms later; the level is invalid, re-run at the end, and the known answer still comes out
        model = dp.make_deadband_mcu(heartbeat_fn=lambda ms: not 11500 <= ms < 11540)
        t, code, r = run(model)
        self.assertEqual(code, 0, r.get('aborted'))
        self.assertIsNone(r['aborted'])
        self.assertEqual(r['retried_steps'], [2])
        self.assertTrue(any(row['dir'] == 'tx' and row['phase'] == 'retry2' for row in r['rows']))
        self.assertEqual(model.counters['latches'], 1)
        self.assertNotIn('invalid', r['table'][2])
        self.assertTrue(r['table'][2]['moved'])
        self.assertEqual(r['w']['recommended_min_speed_theta'], 0.06)
        self.assertEqual(r['v']['recommended_min_speed_xy'], 0.03)
        self.assert_stopped(t, r)

    def test_status_frames_stop_aborts(self):
        t, code, r = run(dp.make_deadband_mcu(), status_until_ms=7000)
        self.assertEqual(code, 2)
        self.assertIn('link lost', r['aborted'])
        tail = [decode_frame(f) for f in t.written[-20:]]
        self.assertTrue(all(not c['enable'] for c in tail))
        self.assertEqual(t.model.executed(), (0., 0.))

    def hook_raising(self, error, phase='step4'):
        def hook(prober, ph, now):
            if ph == phase and prober.t.model.executed() != (0., 0.):
                raise error
        return hook

    def test_exception_mid_level_sends_zero(self):
        t, code, r = run(dp.make_deadband_mcu(), hook=self.hook_raising(RuntimeError('boom')))
        self.assertEqual(code, 2)
        self.assertIn('RuntimeError: boom', r['aborted'])
        self.assert_stopped(t, r)

    def test_ctrl_c_mid_level_sends_zero(self):
        t, code, r = run(dp.make_deadband_mcu(), hook=self.hook_raising(KeyboardInterrupt()))
        self.assertEqual(code, 3)
        self.assertIn('Ctrl-C', r['aborted'])
        self.assert_stopped(t, r)

    def test_commands_never_exceed_caps(self):
        t, _, _ = run(dp.make_deadband_mcu(0., 0.))
        cmds = [decode_frame(f) for f in t.written]
        self.assertLessEqual(max(abs(c['w_mrad_s']) for c in cmds), 150)
        self.assertLessEqual(max(abs(c['v_mm_s']) for c in cmds), 50)
        # second cap: the ChassisLink limits
        p = dp.Prober(dp.SimTransport(dp.make_deadband_mcu()), [], log=quiet)
        self.assertEqual((p.link.v_limit, p.link.w_limit), (60, 200))

    def test_plan_refuses_over_caps(self):
        with self.assertRaises(ValueError):
            dp.build_plan(w_levels=(0.25,))
        with self.assertRaises(ValueError):
            dp.build_plan(v_levels=(0.07,))
        with self.assertRaises(ValueError):
            dp.build_plan(v_levels=(0.05, 0.06), v_step_s=3.)     # 0.33 m forward
        plan = dp.build_plan()
        self.assertEqual(len(plan), 17)
        self.assertLess(sum(s['v'] * s['seconds'] for s in plan), 0.3)
        with redirect_stdout(io.StringIO()):
            self.assertEqual(dp.main(['sim', '--w-levels', '0.02,0.3']), 2)


class Cli(unittest.TestCase):
    def test_dry_run_sends_nothing(self):
        out = io.StringIO()
        with redirect_stdout(out):
            code = dp.main(['serial', '--port', '/nonexistent/tty', '--dry-run'])
        self.assertEqual(code, 0)
        self.assertIn('plan: 17 steps', out.getvalue())
        self.assertIn('straight total 0.230 m', out.getvalue())

    def test_confirmation_required(self):
        out = io.StringIO()
        stdin = sys.stdin
        sys.stdin = io.StringIO('n\n')
        try:
            with redirect_stdout(out):
                code = dp.main(['serial', '--port', '/nonexistent/tty'])
        finally:
            sys.stdin = stdin
        self.assertEqual(code, 2)
        self.assertIn('not confirmed, nothing sent', out.getvalue())

    def test_sim_record_then_analyze_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            rec, out1, out2 = (os.path.join(d, n) for n in ('rec.jsonl', 'a.json', 'b.json'))
            with redirect_stdout(io.StringIO()):
                self.assertEqual(dp.main(['sim', '--record', rec, '--out', out1]), 0)
                self.assertEqual(dp.main(['analyze', '--record', rec, '--out', out2]), 0)
            with open(out1) as f1, open(out2) as f2:
                a, b = json.load(f1), json.load(f2)
            for k in ('w', 'v'):
                self.assertEqual(a[k], b[k])
            self.assertEqual(b['w']['recommended_min_speed_theta'], 0.06)
            self.assertEqual(b['v']['recommended_min_speed_xy'], 0.03)


class PtySmoke(unittest.TestCase):
    def test_serial_mode_against_mcu_sim_process(self):
        """Real serial path: mcu_sim.py --pty (no dead band) <- deadband_probe.py serial. Every level moves."""
        sim = subprocess.Popen([sys.executable, str(ROOT / 'tools/chassis_uart/mcu_sim.py'), '--pty',
                                '--seconds', '40', '--calib-s', '0.5', '--track-mm', '394', '--wheel-diam-mm', '81',
                                '--counts-per-rev', '1560', '--s2-lx-ly-mm', '197'],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            port = sim.stdout.readline().strip()
            with tempfile.TemporaryDirectory() as d:
                out = os.path.join(d, 'r.json')
                p = subprocess.run([sys.executable, str(ROOT / 'tools/chassis_uart/deadband_probe.py'), 'serial',
                                    '--port', port, '--yes', '--out', out, '--w-levels', '0.02,0.1',
                                    '--v-levels', '0.01,0.03', '--w-step-s', '1.0', '--v-step-s', '1.0',
                                    '--pause-s', '0.3', '--preroll-s', '1.0'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=60)
                self.assertEqual(p.returncode, 0, p.stdout)
                with open(out) as f:
                    r = json.load(f)
            self.assertEqual(r['w']['min_effective'], 0.02)
            self.assertEqual(r['w']['recommended_min_speed_theta'], 0.03)
            self.assertEqual(r['v']['min_effective'], 0.01)
            self.assertEqual(r['v']['recommended_min_speed_xy'], 0.02)
            self.assertGreaterEqual(r['zero_frames_at_end'], 20)
        finally:
            sim.terminate()
            sim.communicate(timeout=10)


if __name__ == '__main__':
    unittest.main()
