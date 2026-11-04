"""v0.2 Nano side: gyro heading + four-wheel odometry from the 0x82 frame (2026-10-21).

docs/design/CHASSIS_UART_V02_IMU_HANDOFF_20261021.md sections 8 (spec) and 10 (implementation). Known answers
come from the fake MCU (tools/chassis_uart/mcu_sim.py McuModel + Status2Faults: it knows the body's true
yaw), driven through the same ROS-free chain the car runs:
  ChassisLink.next_command (flag b2) -> McuModel.receive / step -> FrameParser -> ChassisLink.on_status
  -> wheel_counts_row (JSON round trip) -> OdomCore.feed_chassis -> (FusedOdometry | WheelOdometry).
No rclpy on the host: the ROS shells only move these dicts (as in tests/test_wheel_odom.py).
"""
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / 'tests'):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from host.chassis_uart import (GYRO_RAW_DEG_S, TYPE_STATUS, TYPE_STATUS2, ChassisLink, FrameParser,  # noqa: E402
                               decode_frame, encode_status2)
from host.gyro_odom import (FusedOdometry, FusedParams, mecanum_forward, mecanum_inverse,  # noqa: E402
                            parse_signs, parse_wheel_map)
from host.ros_chassis_driver import record_row, wheel_counts_row  # noqa: E402
from host.ros_wheel_odom import OdomCore, odometry_fields, parse_args, slip_message  # noqa: E402
from host.wheel_odom import WheelParams  # noqa: E402
from perception.fpga_replay.pose_trust import PoseTrustMonitor  # noqa: E402
from tools.chassis_uart.mcu_sim import McuModel, Status2Faults  # noqa: E402
import golden_wheel_mode  # noqa: E402

MS = 1000000
DEG = math.pi / 180.
LX_LY = 0.215
# the car's wheels (D-041) and, for the wheel path, the effective track that makes it agree with the
# four-wheel model of the fake MCU (track = 2 (lx + ly))
WHEEL = dict(wheel_diam_m=0.081, counts_per_rev=1560, track_m=2 * LX_LY)


def q(bias_dps):
    """The fake MCU's reads are integers: a noise-free bias is seen rounded to the 0.061 deg/s LSB."""
    return round(bias_dps / GYRO_RAW_DEG_S) * GYRO_RAW_DEG_S


class Chain:
    """Fake MCU <-> Nano chain at 1 ms resolution; commands every 20 ms, status every 10 ms."""

    def __init__(self, heading='gyro', faults=None, fused=None, status2=True, old_firmware=False,
                 static_criteria=None):
        self.model = McuModel(track_mm=2000 * LX_LY, wheel_diam_mm=81., counts_per_rev=1560, calib_ms=0,
                              status2_faults=faults or Status2Faults())
        self.link = ChassisLink(status2=status2)
        self.parser = FrameParser(accept={TYPE_STATUS, TYPE_STATUS2})
        self.core = OdomCore('chassis', WheelParams(**WHEEL), heading=heading, fused_params=fused or FusedParams())
        if static_criteria is not None:
            self.core.fused = FusedOdometry(self.core.fused.p, static_criteria=static_criteria)
        self.old_firmware = old_firmware
        self.ms = 0
        self.out = []                 # (t_s, feed_chassis state, wheel-only state, body, derived, slip transitions)
        self.types = []

    def run(self, seconds, v=0., w=0.):
        """Upstream /fpga/cmd_vel_safe = (v m/s, w rad/s) for this many seconds."""
        end = self.ms + int(round(seconds * 1000))
        while self.ms < end:
            self.ms += 10
            ns = self.ms * MS
            if self.ms % 20 == 0:
                self.link.on_velocity(v, w, ns)
                frame, _ = self.link.next_command(ns)
                self.model.receive(frame, self.ms)
                if self.old_firmware:
                    self.model.status2 = False          # v0.1 firmware ignores b2
            data = self.model.step(self.ms)
            for row in self.parser.feed(data):
                self.types.append(row['type'])
                derived = self.link.on_status(row, ns)
                body = json.loads(json.dumps(wheel_counts_row(row, derived, ns, self.link.last_command),
                                             allow_nan=False))
                st = self.core.feed_chassis(body)
                self.out.append((self.ms / 1000., st, self.core.last_wheel_state, body, derived,
                                 self.core.drain_slip_transitions()))
        return self

    def last(self):
        return [o for o in self.out if o[1] is not None][-1][1]

    def true_yaw(self):
        return self.model.true_yaw


def wrap_deg(a):
    return math.degrees(math.atan2(math.sin(a), math.cos(a)))


# ---------------------------------------------------------------------------------------------- kinematics
class Kinematics(unittest.TestCase):
    def test_inverse_forward_round_trip_both_roller_patterns(self):
        for rollers, lever in (('X', 0.215), ('O', 0.01)):
            for fwd, lat, dyaw in ((0.1, 0., 0.), (0., 0.05, 0.), (0., 0., 0.3), (0.07, -0.02, -0.11)):
                w = mecanum_inverse(fwd, lat, dyaw, lever, rollers)
                f, l, d, red = mecanum_forward(*w, yaw_lever_m=lever, rollers=rollers)
                self.assertAlmostEqual(f, fwd, places=12)
                self.assertAlmostEqual(l, lat, places=12)
                self.assertAlmostEqual(d, dyaw, places=12)
                self.assertAlmostEqual(red, 0., places=12)

    def test_section_8_4_formulas_literally(self):
        fl, fr, rl, rr = 0.11, 0.37, -0.05, 0.29
        f, l, d, red = mecanum_forward(fl, fr, rl, rr, 0.215, 'X')
        self.assertAlmostEqual(f, (fl + fr + rl + rr) / 4)
        self.assertAlmostEqual(l, (-fl + fr + rl - rr) / 4)
        self.assertAlmostEqual(d, (-fl + fr - rl + rr) / (4 * 0.215))
        self.assertAlmostEqual(red, (fl + fr) - (rl + rr))

    def test_map_and_sign_parsing(self):
        self.assertEqual(parse_wheel_map('a=fl,b=fr,c=rl,d=rr'), {'fl': 'a', 'fr': 'b', 'rl': 'c', 'rr': 'd'})
        self.assertEqual(parse_signs('+1,-1,1,-1'), {'a': 1, 'b': -1, 'c': 1, 'd': -1})
        for bad in ('a=fl,b=fl,c=rl,d=rr', 'a=fl,b=fr,c=rl', 'e=fl,b=fr,c=rl,d=rr'):
            with self.assertRaises(ValueError):
                parse_wheel_map(bad)
        with self.assertRaises(ValueError):
            parse_signs('1,1,1')
        with self.assertRaises(ValueError):
            FusedParams(rollers='O', track_m=0.2, wheelbase_m=0.2)

    def test_straight_and_spin_have_no_lateral_and_no_redundancy(self):
        """Through the fake MCU's four counters: pure straight and pure in-place spin give lateral = 0 and
        redundancy = 0 on every frame; a wrong wheel map (b and c swapped) does not (positive control)."""
        def worst(fused):
            ch = Chain(fused=fused).run(2.0).run(2.0, v=0.3).run(2.0).run(3.0, w=1.2).run(1.0)
            rows = [o[1] for o in ch.out if o[1] is not None and o[1]['event'] == 'ok']
            self.assertGreater(len(rows), 900)
            return (max(abs(r['lateral']) for r in rows), max(abs(r['redundancy']) for r in rows),
                    sum(abs(r['lateral']) for r in rows))
        lat, red, _ = worst(FusedParams())
        self.assertLessEqual(lat, 1e-12)
        self.assertLessEqual(red, 1e-12)
        lat, red, total = worst(FusedParams(wheel_map='a=fl,c=fr,b=rl,d=rr'))   # front/rear mixed up
        self.assertGreater(red, 1e-4)
        lat, red, total = worst(FusedParams(wheel_map='b=fl,a=fr,c=rl,d=rr'))   # front pair swapped
        self.assertGreater(lat, 1e-4)
        self.assertGreater(total, 0.1)


# ---------------------------------------------------------------------------------------------- heading
class HeadingKnownAnswers(unittest.TestCase):
    def test_constant_rate_five_turns(self):
        """Bias 0.4 deg/s and 2 LSB read noise injected; 3 s static, then w = 1.0 rad/s for five turns.
        The integrated gyro yaw change must match the fake MCU's true yaw within 0.5 deg; without the bias
        estimate it would be off by about 0.4 * 31.4 = 12.6 deg."""
        ch = Chain(faults=Status2Faults(bias_dps=0.4, noise_lsb=2., seed=3)).run(3.0)
        st0, true0 = ch.last(), ch.true_yaw()
        self.assertEqual(st0['heading_source'], 'gyro')
        self.assertAlmostEqual(st0['bias_dps'], 0.4, delta=0.03)
        ch.run(10 * math.pi, w=1.0).run(1.0)
        st = ch.last()
        true_turn = ch.true_yaw() - true0
        self.assertAlmostEqual(true_turn, 10 * math.pi, delta=0.05)          # the drive really did five turns
        err = math.degrees((st['yaw'] - st0['yaw']) - true_turn)
        self.assertLess(abs(err), 0.5, err)
        status = ch.core.status()
        self.assertEqual(status['heading'], 'gyro')
        self.assertEqual(status['mcu_frame'], '0x82')
        self.assertGreater(status['gyro']['fallback_by_reason_s']['bias_not_ready'], 1.5)   # start-up: wheels
        self.assertEqual(status['gyro']['spikes_replaced'], 0)
        HeadingKnownAnswers.five_turn_err_deg = err

    def test_initial_static_needs_two_seconds(self):
        ch = Chain(faults=Status2Faults(bias_dps=0.4)).run(1.8)
        self.assertIsNone(ch.last()['bias_dps'])
        self.assertEqual(ch.last()['heading_source'], 'wheel')
        self.assertIn('bias_not_ready', ch.last()['fallback'])
        ch.run(0.4)
        self.assertAlmostEqual(ch.last()['bias_dps'], q(0.4), delta=1e-9)
        self.assertEqual(ch.last()['heading_source'], 'gyro')

    def test_bias_reestimated_at_every_stop(self):
        """Bias drifting 0.05 deg/s per second: each stop's estimate follows it, the first one is replaced."""
        ch = Chain(faults=Status2Faults(bias_dps=0.2, drift_dps_per_s=0.05, noise_lsb=1., seed=5)).run(3.0)
        b1 = ch.last()['bias_dps']
        ch.run(10.0, v=0.2, w=0.3).run(3.0)
        b2, t = ch.last()['bias_dps'], ch.ms / 1000.
        expected = 0.2 + 0.05 * (t - 1.2)              # mean over the second stop (about 1 s .. 2.8 s into it)
        self.assertAlmostEqual(b1, 0.2 + 0.05 * 1.5, delta=0.05)
        self.assertAlmostEqual(b2, expected, delta=0.06)
        self.assertGreater(b2 - b1, 0.5)
        self.assertEqual(ch.core.fused.bias.counters['stretches'], 2)

    def test_wheels_moving_or_command_blocks_static(self):
        """Command non-zero but the MCU not moving (enable held by the Nano would be stop; here: driving)."""
        ch = Chain(faults=Status2Faults(bias_dps=0.4)).run(3.0).run(0.1, v=0.1)   # (one frame of look-ahead)
        updates = ch.core.fused.bias.counters['updates']
        ch.run(3.0, v=0.1)
        self.assertEqual(ch.core.fused.bias.counters['updates'], updates)
        self.assertFalse(ch.core.fused.bias.static_now)


class Position(unittest.TestCase):
    def test_drive_matches_the_true_path(self):
        """Straight, arcs both ways, spin in place, straight: the four-wheel translation rotated by the mean
        gyro yaw of each frame lands within 1.5 mm of the fake MCU's true position after ~4 m of path."""
        ch = Chain(faults=Status2Faults(bias_dps=0.2, noise_lsb=1., seed=2)).run(3.0)
        st0 = ch.last()
        self.assertEqual((st0['x'], st0['y']), (0., 0.))
        ch.run(3.0, v=0.4).run(4.0, v=0.4, w=1.5).run(3.0, v=0.3, w=-1.0).run(2.0, w=1.5).run(3.0, v=0.4).run(0.5)
        st = ch.last()
        path = ch.core.fused.distance
        self.assertGreater(path, 4.0)
        err = math.hypot(st['x'] - ch.model.true_x, st['y'] - ch.model.true_y)
        self.assertLess(err, 0.0015, err)          # measured 0.4 mm; end-of-frame yaw instead of the mean: 2.9 mm
        Position.err_m = err


class StaticDetectorUnits(unittest.TestCase):
    def feed(self, sb, t0, n, rate=0., amag=1.0, moved=False, stop=True, step=None):
        for k in range(n):
            r = rate if step is None else step(k)
            a = amag if not callable(amag) else amag(k)
            sb.feed(t0 + k * 0.01, moved, stop, r, a)
        return t0 + n * 0.01

    def test_accel_std_blocks_static(self):
        from host.gyro_odom import StaticBias
        sb = StaticBias(FusedParams())
        self.feed(sb, 0., 300, amag=lambda k: 1.0 + (0.05 if k % 2 else -0.05))     # car being shaken / carried
        self.assertFalse(sb.ready)
        sb = StaticBias(FusedParams())
        self.feed(sb, 0., 300)
        self.assertTrue(sb.ready)

    def test_guard_keeps_a_slow_onset_out_of_the_bias(self):
        """A rotation that starts slowly stays under G for a few frames; those frames are not committed."""
        from host.gyro_odom import StaticBias
        out = []
        for guard in (0.2, 0.):
            sb = StaticBias(FusedParams(bias_guard_s=guard))
            t = self.feed(sb, 0., 300)
            self.feed(sb, t, 30, step=lambda k: 0.004 * (k + 1) ** 2)           # 0.004 .. 3.6 deg/s
            out.append(sb.bias_dps)
        self.assertEqual(out[0], 0.)
        self.assertGreater(out[1], 0.001)


class TornAccelReads(unittest.TestCase):
    """Board check 10-21 17:07 (runs/imu-v02-check-20261021, 60 s static with dev2's 0x82 firmware): 283 of 6012
    frames had a torn accel read ((0, 0, 0) or one axis zeroed). Without the outlier rule the 1 s accel std is
    ~0.2 g, nothing is ever static, the bias is never ready and the gyro is never used."""

    def test_fake_mcu_torn_reads(self):
        ch = Chain(faults=Status2Faults(bias_dps=0.3, accel_noise_lsb=3., seed=4, accel_torn_every=20)).run(2.3)
        self.assertAlmostEqual(ch.last()['bias_dps'], q(0.3), delta=0.01)
        self.assertGreater(ch.core.fused.counters['accel_invalid'], 5)
        ch = Chain(faults=Status2Faults(bias_dps=0.3, accel_noise_lsb=3., seed=4, accel_torn_every=20),
                   fused=FusedParams(accel_outlier_frac=0.)).run(5.0)                     # negative control
        self.assertIsNone(ch.last()['bias_dps'])

    def test_real_board_static_frames(self):
        """15 s of the real frames (tests/data/v02_board_static_20261021_15s.jsonl: seq, mcu_ms, enc_a..d,
        gz_sum, gz_n, imu_err, ax, ay, az; 71 torn accel frames, 8 frames gz_n = 0, gyro exactly 0 throughout)."""
        keys = ('seq', 'mcu_ms', 'enc_a', 'enc_b', 'enc_c', 'enc_d', 'gz_sum', 'gz_n', 'imu_err', 'ax', 'ay', 'az')
        rows = [dict(zip(keys, json.loads(line))) for line in
                (ROOT / 'tests/data/v02_board_static_20261021_15s.jsonl').read_text().splitlines()]
        self.assertEqual(len(rows), 1500)

        def run(params):
            f = FusedOdometry(params)
            for k, r in enumerate(rows):
                r = dict(r, t_ns=k * 10 * MS, cmd=[0, 0, False])
                st = f.update(r)
            return f, st
        f, st = run(FusedParams())
        self.assertEqual(f.bias.bias_dps, 0.)               # the module still zeroes the static gyro (T2 open)
        self.assertLess(f.fallback_by_reason['bias_not_ready'], 2.1)
        self.assertEqual(f.counters['accel_invalid'], 71)
        self.assertEqual(st['heading_source'], 'gyro')
        self.assertEqual(f.fallback_by_reason['gyro_gap'], 0.)
        f, st = run(FusedParams(accel_outlier_frac=0.))     # negative control: spec condition 4 taken literally
        self.assertIsNone(f.bias.bias_dps)
        self.assertEqual(st['heading_source'], 'wheel')


class ReplayTool(unittest.TestCase):
    def test_pseudo_0x82_replay_of_a_0x81_record(self):
        """tools/wheel_odom/gyro_replay_0x81.py on a synthetic v0.1 record: 3 s static, then 4 s turning at
        0.5 rad/s with a gyro that agrees with the wheels -> gyro and wheel net yaw agree; a static spike is
        replaced."""
        import contextlib
        import io
        import tempfile
        from tools.wheel_odom import gyro_replay_0x81 as tool
        per_frame = 0.005 * 0.43 / 2 / (math.pi * 0.081 / 1560)     # counts per 10 ms per side at 0.5 rad/s
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'rec.jsonl'
            with open(path, 'w') as f:
                for k in range(700):
                    turning = k >= 300
                    side = int(round((k - 300) * per_frame)) if turning else 0
                    gz = int(round(math.degrees(0.5) * 100)) if turning else (7812 if k == 150 else 0)
                    f.write(json.dumps({'dir': 'tx', 'v_mm_s': 0, 'w_mrad_s': 500 if turning else 0,
                                        'enable': turning, 't_ns': k * 10 * MS}) + '\n')
                    f.write(json.dumps({'dir': 'rx', 't_ns': k * 10 * MS, 'seq': k & 0xFF, 'mcu_ms': (k * 10) & 0xFFFF,
                                        'left_count': -side, 'right_count': side, 'gyro_z_cdps': gz,
                                        'gyro_valid': True, 'after_silence': False}) + '\n')
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                rc = tool.main([str(path), '--track-m', '0.43'])
            out = json.loads(buf.getvalue())
        self.assertEqual(rc, 0)
        self.assertEqual(out['frames'], 700)
        self.assertAlmostEqual(out['wheel_net_yaw_deg'], math.degrees(0.5 * 4), delta=0.5)
        self.assertLess(abs(out['gyro_minus_wheel_deg']), 0.5)
        self.assertEqual(out['spikes_replaced'], 1)


class CollisionNotLearned(unittest.TestCase):
    """10-17 lesson: wheels still while the car is turned. Not learned as bias; negative control learns it."""

    def scenario(self, static_criteria=None):
        faults = Status2Faults(bias_dps=0.3, noise_lsb=1., accel_noise_lsb=2., seed=7,
                               collisions=[(4000, 7000, 60.)])
        ch = Chain(faults=faults, static_criteria=static_criteria).run(3.0)
        b0, y0, t0 = ch.last()['bias_dps'], ch.last()['yaw'], ch.true_yaw()
        ch.run(8.0)                                    # stop commanded throughout; collision 4.0-7.0 s; still after
        return ch, b0, y0, t0

    def test_collision_not_learned_and_heading_follows(self):
        ch, b0, y0, t0 = self.scenario()
        st = ch.last()
        turn = ch.true_yaw() - t0
        self.assertAlmostEqual(math.degrees(turn), 60 * 3 * 2 / math.pi, delta=1.0)    # ~114.6 deg really turned
        self.assertAlmostEqual(st['bias_dps'], 0.3, delta=0.05)
        self.assertLess(abs(math.degrees((st['yaw'] - y0) - turn)), 1.0)
        wheel = ch.out[-1][2]
        self.assertLess(abs(wheel['yaw']), 1e-9)       # the wheels never saw it
        self.assertFalse(any(o[1]['pose_trusted'] is False for o in ch.out if o[1] and o[1]['event'] == 'ok'
                             and o[0] < 4.0))
        self.assertTrue(any(o[1]['pose_trusted'] is False for o in ch.out if o[1] and 4.0 <= o[0] < 7.0))

    def test_negative_control_wheels_only_static_learns_the_collision(self):
        ch, b0, y0, t0 = self.scenario(static_criteria=('wheels',))
        st = ch.last()
        turn = ch.true_yaw() - t0
        self.assertGreater(abs(st['bias_dps'] - 0.3), 5.0)
        self.assertGreater(abs(math.degrees((st['yaw'] - y0) - turn)), 20.0)


class SpecWeakness(unittest.TestCase):
    def test_smooth_constant_rate_turn_passes_the_four_conditions(self):
        """Reported, not fixed: a perfectly smooth 5 deg/s turn with wheels still and no shock (a slow hand
        turn) has std ~ 0 after its first second, so section 8.3's four conditions call it static and its rate
        becomes the bias. The opt-in --bias-max-dps (not in the spec, default off) refuses such an estimate."""
        faults = dict(bias_dps=0.3, noise_lsb=1., seed=11, collisions=[(4000, 8000, 5., 'const')],
                      collision_shock_g=0.)
        ch = Chain(faults=Status2Faults(**faults)).run(3.0).run(5.5)
        self.assertGreater(ch.last()['bias_dps'], 4.0)                     # learned (spec as written)
        ch = Chain(faults=Status2Faults(**faults), fused=FusedParams(bias_max_dps=2.0)).run(3.0).run(5.5)
        self.assertAlmostEqual(ch.last()['bias_dps'], 0.3, delta=0.05)    # opt-in cap refuses it
        self.assertGreater(ch.core.fused.bias.counters['rejected_bias_max'], 0)


class Spikes(unittest.TestCase):
    SPIKE_RAW = 1536                    # 93.75 deg/s = 6 x 256 LSB (high byte only)

    def run_with(self, fused=None, frames=(150, 420, 700, 900, 905, 1150)):
        """Spikes in the start-up static (150), driving straight (420), turning (700, 900, 905: two close
        together) and the final static (1150)."""
        spikes = {f: self.SPIKE_RAW for f in frames}
        ch = Chain(faults=Status2Faults(bias_dps=0.2, spikes=spikes), fused=fused).run(3.0)
        self.assertIsNotNone(ch.last()['bias_dps'])
        y0, t0 = ch.last()['yaw'], ch.true_yaw()
        ch.run(3.0, v=0.2).run(4.0, w=0.8).run(2.0)
        return ch, math.degrees((ch.last()['yaw'] - y0) - (ch.true_yaw() - t0))

    def test_spikes_replaced_and_counted(self):
        ch, err = self.run_with()
        self.assertEqual(ch.core.fused.counters['spikes_replaced'], 6)
        self.assertEqual(ch.core.status()['gyro']['spikes_replaced'], 6)
        self.assertLess(abs(err), 0.3)
        self.assertAlmostEqual(ch.last()['bias_dps'], q(0.2), delta=1e-9)   # static spikes did not move it

    def test_negative_control_filter_off(self):
        """Filter off: a static spike also breaks the gyro-std condition, so the start-up spike is left out
        here (else the bias is never ready and the wheels carry the heading); five spikes ~0.9 deg each."""
        ch, err = self.run_with(FusedParams(spike_dps=1e9), frames=(420, 700, 900, 905, 1150))
        self.assertEqual(ch.core.fused.counters['spikes_replaced'], 0)
        spike = self.SPIKE_RAW * GYRO_RAW_DEG_S - q(0.2)
        expected = 0.01 * (2 * spike + 3 * (spike - math.degrees(0.8)))   # straight + final static, 3 turning
        self.assertAlmostEqual(err, expected, delta=0.1)                   # ~3.3 deg
        self.assertGreater(abs(err), 3.0)

    def test_real_step_is_kept(self):
        """A real 0 -> 57 deg/s step (command w 1.0 rad/s at once) is a suspect for one frame, then kept;
        the one-frame provisional value is corrected afterwards."""
        ch = Chain(faults=Status2Faults(bias_dps=0.2)).run(3.0)
        y0, t0 = ch.last()['yaw'], ch.true_yaw()
        ch.run(2.0, w=1.0)
        # checked while still turning: the start (provisional too low) and the stop (provisional too high)
        # errors would cancel without the correction
        self.assertLess(abs(math.degrees((ch.last()['yaw'] - y0) - (ch.true_yaw() - t0))), 0.3)
        ch.run(1.0)
        self.assertGreaterEqual(ch.core.fused.counters['spike_suspects_kept'], 2)
        self.assertEqual(ch.core.fused.counters['spikes_replaced'], 0)
        self.assertLess(abs(math.degrees((ch.last()['yaw'] - y0) - (ch.true_yaw() - t0))), 0.3)


class Fallbacks(unittest.TestCase):
    def test_gyro_gap_over_50ms_uses_wheels_and_is_timed(self):
        ch = Chain(faults=Status2Faults(bias_dps=0.2, dropouts=[(5000, 5200), (7000, 7030)])).run(3.0)
        y0, t0 = ch.last()['yaw'], ch.true_yaw()
        ch.run(6.0, w=0.6)
        st = ch.core.status()['gyro']
        # 200 ms without reads: the first 50 ms hold the last rate, then the wheels (5.06 .. 5.20 s = 0.15 s)
        self.assertAlmostEqual(st['fallback_by_reason_s']['gyro_gap'], 0.15, delta=0.011)
        self.assertGreater(st['counters']['gyro_missing_frames'], 20)
        sources = {round(o[0], 2): o[1]['heading_source'] for o in ch.out if o[1] and o[1]['event'] == 'ok'}
        self.assertEqual(sources[5.1], 'wheel')
        self.assertEqual(sources[7.02], 'gyro')        # 30 ms gap: held rate, no fallback
        self.assertLess(abs(math.degrees((ch.last()['yaw'] - y0) - (ch.true_yaw() - t0))), 0.5)

    def test_lost_frames_over_50ms_use_wheels(self):
        """Frames lost on the line (dt > 50 ms between two frames): wheel heading for that interval."""
        ch = Chain(faults=Status2Faults(bias_dps=0.2)).run(3.0)
        y0, t0 = ch.last()['yaw'], ch.true_yaw()
        ch.model.drop_every = 0
        for _ in range(3):
            ch.run(1.0, w=0.6)
            for _ in range(7):                         # 70 ms of frames never arrive
                ch.ms += 10
                ch.model.step(ch.ms)
        ch.run(1.0, w=0.6)
        st = ch.core.status()['gyro']
        self.assertAlmostEqual(st['fallback_by_reason_s']['gyro_gap'], 0.24, delta=0.001)   # 3 x 80 ms
        self.assertLess(abs(math.degrees((ch.last()['yaw'] - y0) - (ch.true_yaw() - t0))), 0.5)

    def test_gz_n_jitter_is_not_a_gap(self):
        ch = Chain(faults=Status2Faults(bias_dps=0.2, gz_n_one_every=3)).run(3.0)
        y0, t0 = ch.last()['yaw'], ch.true_yaw()
        ch.run(5.0, w=0.7)
        self.assertEqual(ch.core.status()['gyro']['fallback_by_reason_s']['gyro_gap'], 0.)
        self.assertLess(abs(math.degrees((ch.last()['yaw'] - y0) - (ch.true_yaw() - t0))), 0.3)

    def test_imu_err_burst_falls_back_for_a_second(self):
        ch = Chain(faults=Status2Faults(bias_dps=0.2, imu_err_at={500: 6, 800: 3})).run(3.0).run(6.0, w=0.5)
        st = ch.core.status()['gyro']
        self.assertAlmostEqual(st['fallback_by_reason_s']['imu_err'], 1.0, delta=0.011)   # 6 > 5; 3 alone is not
        self.assertEqual(st['imu_err_total'], 9)


# ---------------------------------------------------------------------------------------------- 0x81 / wheel
class OldFirmwareAndWheelMode(unittest.TestCase):
    def test_0x81_input_gyro_mode_is_the_wheel_output(self):
        a = Chain(heading='gyro', old_firmware=True).run(1.0).run(3.0, v=0.2, w=0.4)
        b = Chain(heading='wheel', old_firmware=True).run(1.0).run(3.0, v=0.2, w=0.4)
        self.assertEqual(set(a.types), {'status'})
        sa = [o[1] for o in a.out]
        sb = [o[1] for o in b.out]
        self.assertEqual(sa, sb)
        st = a.core.status()
        self.assertEqual(st['mcu_frame'], '0x81')
        self.assertIn('not switched to 0x82', st['warning'])
        self.assertEqual(st['heading_source'], 'wheel')
        link = a.link.status(a.ms * MS)
        self.assertEqual(link['mcu_frame'], '0x81')
        self.assertIn('not switched', link['warning'])
        self.assertNotIn('heading', b.core.status())          # wheel mode on 0x81: the v0.1 status keys only

    def test_switch_0x81_to_0x82_and_back_continues_the_pose(self):
        """Gyro mode: 0x81 while driving (old firmware), then 0x82 (the fused pose starts from the wheel pose),
        then 0x81 again (the wheel motion moves the fused pose): no jump at either switch, and the end pose
        agrees with the fake MCU's truth."""
        ch = Chain(old_firmware=True).run(1.0).run(2.0, v=0.2, w=0.5)
        self.assertEqual(set(ch.types), {'status'})
        before = ch.last()
        ch.old_firmware = False
        ch.run(4.0, v=0.2, w=0.5)                      # bias never ready while driving: wheel fallback
        ch.old_firmware = True
        ch.run(2.0, v=0.2, w=0.5).run(0.5)
        kinds = [o[3].get('frame', '0x81') for o in ch.out]
        first82 = kinds.index('0x82')
        back81 = first82 + kinds[first82:].index('0x81')
        states = [o[1] for o in ch.out]
        for i in (first82, back81):
            a = [s for s in states[:i] if s is not None][-1]
            b = states[i] if states[i] is not None else states[i + 1]
            self.assertLess(math.hypot(b['x'] - a['x'], b['y'] - a['y']), 0.01)      # <= one frame of motion
            self.assertLess(abs(b['yaw'] - a['yaw']), 0.01)
        self.assertNotEqual(states[first82 - 1]['x'], before['x'])
        end = ch.last()
        self.assertAlmostEqual(end['yaw'], ch.true_yaw(), delta=0.01)
        self.assertEqual(ch.core.status()['mcu_frame'], '0x81')

    def test_wheel_mode_is_bit_identical_to_the_base(self):
        """tests/data/wheel_mode_golden_de10531.json was written by the unmodified code (de10531)."""
        golden = json.loads(golden_wheel_mode.FIXTURE.read_text())
        now = golden_wheel_mode.golden()
        for name in golden:
            for key in golden[name]:
                self.assertEqual(now[name][key], golden[name][key], '%s / %s' % (name, key))

    def test_wheel_mode_on_0x82_frames_equals_0x81_frames(self):
        """The same counts as 0x82 frames (enc_a/enc_b = left/right): identical poses, records, events; only
        the ROS stamp moves by the longer airtime (52 vs 24 bytes)."""
        golden = json.loads(golden_wheel_mode.FIXTURE.read_text())['realcar_1016']
        link, parser = ChassisLink(status2=True), FrameParser(accept={TYPE_STATUS, TYPE_STATUS2})
        core = OdomCore('chassis', WheelParams(**golden_wheel_mode.PARAM_SETS['realcar_1016']))
        states = []
        for received, data in golden_wheel_mode.input_batches():
            for row in FrameParser(accept={TYPE_STATUS}).feed(data):
                f2 = encode_status2(seq=row['seq'], mcu_ms=row['mcu_ms'], enc_a=row['left_count'],
                                    enc_b=row['right_count'], enc_c=row['left_count'], enc_d=row['right_count'],
                                    gz_sum=0, gz_n=2, az=2048, last_cmd_seq=row['last_cmd_seq'], status=row['status'])
                for r2 in parser.feed(f2):
                    derived = link.on_status(r2, received)
                    body = json.loads(json.dumps(wheel_counts_row(r2, derived, received, link.last_command)))
                    st = core.feed_chassis(body)
                    if st is not None:
                        states.append(st)
        self.assertEqual(len(states), len(golden['states']))
        for a, b in zip(states, golden['states']):
            a = dict(a)
            b = dict(b)
            self.assertNotEqual(a.pop('ros_stamp_ns'), b.pop('ros_stamp_ns'))
            self.assertEqual(json.loads(json.dumps(a)), b)

    def test_driver_rows(self):
        f = encode_status2(seq=1, mcu_ms=123456, enc_a=1000, enc_b=1001, enc_c=-999, enc_d=-1002, gz_sum=-41,
                           gz_n=2, gx_mean=1, gy_mean=-2, ax=12, ay=-8, az=2050, yaw_raw=5461, temp_cdeg=-32768,
                           last_cmd_seq=7, status=0x3C)
        row = decode_frame(f)
        link = ChassisLink(status2=True)
        derived = link.on_status(row, 10 * MS)
        self.assertAlmostEqual(derived['gyro_z_rad_s'], math.radians(-20.5 * 2000 / 32768))
        self.assertEqual(derived['stamp_ns'], 10 * MS - 52 * 10 * 10**9 // 115200)
        body = wheel_counts_row(row, derived, 5, (100, -200, True))
        self.assertEqual((body['left_count'], body['right_count'], body['frame']), (1000, 1001, '0x82'))
        self.assertEqual((body['enc_c'], body['enc_d'], body['gz_sum'], body['gz_n']), (-999, -1002, -41, 2))
        self.assertIsNone(body['temp_cdeg'])
        self.assertEqual(body['cmd'], [100, -200, True])
        rec = record_row(row, derived)
        for k in ('enc_a', 'enc_b', 'enc_c', 'enc_d', 'gz_sum', 'gz_n', 'imu_err', 'gx_mean', 'gy_mean', 'ax', 'ay',
                  'az', 'yaw_raw', 'temp_cdeg', 'gyro_bias_cdps', 'battery_mv', 'last_cmd_seq', 'status', 'seq',
                  'mcu_ms'):
            self.assertEqual(rec[k], row[k])
        self.assertEqual(rec['frame'], '0x82')
        row['gz_n'] = 0
        self.assertIsNone(ChassisLink().on_status(row, 20 * MS)['gyro_z_rad_s'])   # no gyro -> /imu gap
        frame, _ = link.next_command(30 * MS)
        self.assertTrue(decode_frame(frame)['status2'])
        frame, _ = ChassisLink().next_command(30 * MS)
        self.assertFalse(decode_frame(frame)['status2'])

    def test_cli_defaults(self):
        args, _, params, fused, pv, tv = parse_args(['--source', 'chassis'])
        self.assertEqual(args.heading, 'wheel')
        self.assertEqual(args.wheel_heading_topic, '/fpga/wheel_odom/wheel_heading')
        self.assertEqual(args.slip_topic, '/fpga/wheel_odom/slip')
        self.assertEqual(fused.yaw_lever_m, 0.215)
        self.assertIn('wheel_map', fused.placeholders())
        self.assertIn('static_gyro_std_dps', fused.placeholders())
        args, _, params, fused, pv, tv = parse_args(['--source', 'chassis', '--heading', 'gyro', '--pose-variances',
                                                     '1,2,3,4,5,6', '--wheel-map', 'a=rl,b=rr,c=fl,d=fr'])
        self.assertEqual(pv, (1., 2., 3., 4., 5., 6.))
        self.assertNotIn('wheel_map', fused.placeholders())
        self.assertEqual(odometry_fields({'x': 0., 'y': 0., 'yaw': 0., 'v': 1., 'vy': .5, 'w': 0.},
                                         pose_var=pv)['pose_covariance'][7], 2.)


# ---------------------------------------------------------------------------------------------- pose trust
def imu_msg(t, rate):
    sec = int(t)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9)))),
                           angular_velocity=SimpleNamespace(z=rate))


def odom_msg(t, st):
    f = odometry_fields(st)
    q = f['orientation']
    sec = int(t)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9)))),
                           pose=SimpleNamespace(pose=SimpleNamespace(orientation=SimpleNamespace(
                               x=q[0], y=q[1], z=q[2], w=q[3]))))


def gate(ch, topic, with_slip=False):
    """Replay a chain's outputs into the runner's gate the way host/ros_runtime.py does."""
    from host.ros_runtime import feed_pose_trust, feed_pose_trust_slip
    m = PoseTrustMonitor()
    T0 = 1792000000.
    for t, st, wheel, body, derived, slips in ch.out:
        if derived['gyro_z_rad_s'] is not None:
            feed_pose_trust(m, 'imu', imu_msg(T0 + t, derived['gyro_z_rad_s']))
        src = wheel if topic == 'wheel_heading' else st
        if src is not None:
            feed_pose_trust(m, 'odom', odom_msg(T0 + t, src))
        if with_slip:
            for _, trusted, flags in slips:
                feed_pose_trust_slip(m, json.dumps(slip_message((T0 + t) * 1e9, trusted, flags)))
    return m, T0


class PoseTrustGate(unittest.TestCase):
    def collision(self, heading):
        faults = Status2Faults(bias_dps=0.3, noise_lsb=1., seed=7, collisions=[(4000, 7000, 60.)])
        return Chain(heading=heading, faults=faults).run(3.0).run(10.0)

    def test_gyro_mode_gate_on_wheel_heading_trips_on_collision(self):
        ch = self.collision('gyro')
        m, T0 = gate(ch, 'wheel_heading')
        self.assertTrue(m.trusted_at(T0 + 3.9))
        self.assertFalse(m.trusted_at(T0 + 5.0))
        self.assertTrue(m.trusted_at(T0 + 12.9))
        trips = [x for x in m.transitions if not x[1]]
        self.assertEqual(len(trips), 1)
        self.assertLess(trips[0][0] - (T0 + 4.0), 1.5)

    def test_negative_control_gate_on_gyro_heading_is_blind(self):
        ch = self.collision('gyro')
        m, T0 = gate(ch, 'main')                 # the old default /fpga/wheel_odom now carries the gyro yaw
        self.assertEqual([x for x in m.transitions if not x[1]], [])
        self.assertTrue(m.trusted_at(T0 + 5.0))

    def test_wheel_mode_gate_unchanged(self):
        ch = self.collision('wheel')
        a, _ = gate(ch, 'wheel_heading')
        b, _ = gate(ch, 'main')
        self.assertEqual(a.transitions, b.transitions)
        self.assertEqual(len([x for x in a.transitions if not x[1]]), 1)
        self.assertEqual([o[5] for o in ch.out if o[5]], [])     # wheel mode publishes no slip verdicts

    def test_slip_verdicts_and_into_the_gate(self):
        ch = self.collision('gyro')
        m, T0 = gate(ch, 'main', with_slip=True)  # own detector blind (gyro vs gyro); slip verdict alone
        self.assertEqual([x for x in m.transitions if not x[1]], [])
        self.assertFalse(m.trusted_at(T0 + 5.0))
        self.assertTrue(m.trusted_at(T0 + 3.9))
        self.assertTrue(m.trusted_at(T0 + 12.9))
        flags = [x[2] for o in ch.out for x in o[5]]
        self.assertEqual(len(flags), 2)
        self.assertIn('shock', flags[0])
        self.assertEqual(flags[1], ())
        self.assertGreater(ch.core.fused.slip.counters['mismatch'], 0)
        self.assertGreater(ch.core.fused.slip.counters['shock'], 0)

    def test_slip_body_parsing(self):
        from host.ros_runtime import feed_pose_trust_slip
        m = PoseTrustMonitor()
        self.assertIsNone(feed_pose_trust_slip(m, 'nope'))
        self.assertIsNone(feed_pose_trust_slip(m, '{"stamp_ns": 1}'))
        self.assertEqual(feed_pose_trust_slip(m, json.dumps(slip_message(2e9, False, ['shock']))), (2.0, False))
        self.assertTrue(m.trusted_at(1.9))
        self.assertFalse(m.trusted_at(2.5))

    def test_runner_default_topics(self):
        src = (ROOT / 'host/ros_runtime.py').read_text()
        self.assertIn('"--pose-trust-odom-topic", default="/fpga/wheel_odom/wheel_heading"', src)
        self.assertIn('"--pose-trust-slip-topic", default="/fpga/wheel_odom/slip"', src)


if __name__ == '__main__':
    unittest.main()
