"""Fake MCU heartbeat rule, shared wheel odometry, two sources, cross-check tool (2026-10-14).

Pre-registered known answers: work/dev1/runs/wheel-odom-20261014-results/PREREG.md (A: MCU guide
section 2 latch / recovery; B: odometry vs closed form <= 1e-6, chassis and PL paths identical;
C: cross-check tool negative / positive control). No ROS needed: the ROS shells are thin and are
smoke-tested in the Humble container separately.
"""
import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from host.chassis_uart import (TYPE_STATUS, ChassisLink, FrameParser, decode_frame, encode_command,  # noqa: E402
                               encode_status)
from host.kv_link import KvLinkServer, KvTelemetryClient, NullCore, blocks_mask  # noqa: E402
from host.kv_regs import encoder_words, mock_encoder, mock_sysinfo  # noqa: E402
from host.kv_status import decode_encoder  # noqa: E402
from host.ros_chassis_driver import wheel_counts_row  # noqa: E402
from host.ros_wheel_odom import OdomCore, odometry_fields  # noqa: E402
from host.wheel_odom import (PlSampleTracker, WheelOdometry, WheelParams, s32_delta,  # noqa: E402
                             yaw_to_quaternion)
from tools.chassis_uart.mcu_sim import McuModel  # noqa: E402
from tools.wheel_odom import cross_check  # noqa: E402

MS = 1000000
TOL = 1e-6
MPC = math.pi * 0.065 / 1320
TRACK = 0.16


# ------------------------------------------------------------------------------------------- A
class Heartbeat:
    """heartbeat_fn: on except in [off_from, off_to] (inclusive), all in ms."""

    def __init__(self, gaps=()):
        self.gaps = list(gaps)
        self.queried = []

    def __call__(self, ms):
        self.queried.append(ms)
        return not any(a <= ms <= b for a, b in self.gaps)


def cmd(seq, v=200, w=300, enable=True):
    return encode_command(seq, v, w, enable)


class McuLatchKnownAnswers(unittest.TestCase):
    def drive_until_unlocked(self, model, until_ms=200):
        """Commands every 20 ms from t=0 (Nano rate); returns the next seq."""
        seq = 0
        for t in range(0, until_ms + 1, 20):
            model.receive(cmd(seq), t)
            seq += 1
        model.step(until_ms)
        self.assertFalse(model.locked)
        self.assertTrue(model.enabled)
        return seq

    def gap_run(self, n_off):
        """Unlocked and driving; last heartbeat at T=500, off for T+1..T+n_off, back after."""
        twists = []
        hb = Heartbeat([(501, 500 + n_off)])
        model = McuModel(calib_ms=0, heartbeat_fn=hb, twist_fn=lambda ms, v, w: twists.append((ms, v, w)))
        seq = self.drive_until_unlocked(model, 200)
        for t in range(220, 501, 20):
            model.receive(cmd(seq), t)
            seq += 1
        latched_at = None
        for t in range(501, 600):
            model.advance(t)
            if model.locked and latched_at is None:
                latched_at = t
        return model, latched_at, twists

    def test_A1_gap_29ms_does_not_latch(self):
        model, latched_at, _ = self.gap_run(29)
        self.assertIsNone(latched_at)
        self.assertEqual(model.counters['latches'], 0)

    def test_A2_gap_30ms_boundary_does_not_latch(self):
        model, latched_at, _ = self.gap_run(30)
        self.assertIsNone(latched_at)

    def test_A3_gap_31ms_latches_that_millisecond_and_stops(self):
        model, latched_at, twists = self.gap_run(31)
        self.assertEqual(latched_at, 531)
        self.assertEqual(model.events[-1], (531, 'latch'))
        self.assertIn((531, 0., 0.), twists)        # stop reported in the latch millisecond
        self.assertEqual(model.executed(), (0., 0.))
        # positive control on the instrument: a 1 ms gap longer than the boundary is what flipped it
        self.assertIsNone(self.gap_run(30)[1])

    def latched_then_back(self, commands_after_back, back_at=1000):
        """Latched by a gap 600..back_at-1; heartbeat continuous from back_at (R)."""
        hb = Heartbeat([(600, back_at - 1)])
        model = McuModel(calib_ms=0, heartbeat_fn=hb)
        seq = self.drive_until_unlocked(model, 200)
        model.advance(back_at - 1)
        self.assertTrue(model.locked)
        for t in commands_after_back:
            model.receive(cmd(seq), t)
            seq += 1
        return model, seq

    def test_A4_recovery_99_holds_100_releases(self):
        r = 1000
        model, _ = self.latched_then_back([r + 20 * k for k in range(1, 5)] + [r + 99], back_at=r)
        model.advance(r + 99)
        self.assertTrue(model.locked)               # 99 ms of heartbeat: still latched
        model.advance(r + 100)
        self.assertFalse(model.locked)              # 100 ms + a command after R: released
        self.assertEqual(model.events[-1], (r + 100, 'unlock'))

    def test_A5_recovery_without_new_command_holds(self):
        r = 1000
        model, seq = self.latched_then_back([], back_at=r)
        for t in range(r, r + 1001):
            model.advance(t)
            self.assertTrue(model.locked, t)
        model.receive(cmd(seq, 150, 0), r + 1000)   # arrives latched: time-stamped, not executed
        self.assertEqual(model.executed(), (0., 0.))
        model.advance(r + 1001)
        self.assertFalse(model.locked)
        self.assertEqual(model.executed(), (0., 0.))
        model.receive(cmd(seq + 1, 150, 0), r + 1020)
        self.assertEqual(model.executed(), (0.15, 0.))

    def test_A6_command_exactly_at_recovery_start_does_not_count(self):
        r = 1000
        model, _ = self.latched_then_back([r], back_at=r)
        for t in range(r, r + 600):
            model.advance(t)
        self.assertTrue(model.locked)

    def test_A7_power_on_latched_until_100ms_and_new_command(self):
        twists = []
        model = McuModel(calib_ms=0, twist_fn=lambda ms, v, w: twists.append((ms, v, w)))
        moving = []
        for t in range(0, 301, 10):
            if t % 20 == 0:
                model.receive(cmd(t // 20), t)
            row = decode_frame(model.step(t))
            if row['motor_enabled']:
                moving.append(t)
            if t < 100:
                self.assertTrue(row['stop_latched'], t)
        self.assertEqual(moving[0], 100)            # unlock tick 100, the command at 100 runs
        self.assertTrue(all(v == 0. for ms, v, w in twists if ms < 100))

    def test_A8_executed_twist_is_the_limited_command(self):
        twists = []
        model = McuModel(calib_ms=0, twist_fn=lambda ms, v, w: twists.append((ms, v, w)))
        self.drive_until_unlocked(model, 200)
        model.receive(cmd(50, 800, -2000), 220)     # beyond 500 mm/s and 1500 mrad/s
        self.assertEqual(model.executed(), (0.5, -1.5))
        self.assertEqual(twists[-1], (220, 0.5, -1.5))
        model.step(230)
        self.assertEqual(twists[-1], (230, 0.5, -1.5))   # every step republished
        model.step(421)                             # > 200 ms without a command: timeout
        self.assertEqual(model.executed(), (0., 0.))
        # calibrating: no motion even when unlocked
        model = McuModel(calib_ms=1000)
        for t in range(0, 900, 20):
            model.receive(cmd(t // 20), t)
        self.assertFalse(model.locked)
        self.assertEqual(model.executed(), (0., 0.))

    def test_heartbeat_queried_every_millisecond(self):
        hb = Heartbeat()
        model = McuModel(calib_ms=0, heartbeat_fn=hb)
        model.step(0)
        model.step(10)
        model.receive(cmd(0), 15)
        model.step(20)
        self.assertEqual(hb.queried, list(range(21)))

    def test_wheel_counts_follow_executed_speed_per_ms(self):
        model = McuModel(calib_ms=0)
        self.drive_until_unlocked(model, 200)
        left0, right0 = model.left, model.right
        model.receive(cmd(40, 100, 0), 220)
        model.step(320)
        model.receive(cmd(41, 100, 0), 320)
        model.advance(420)
        cpm = 1320 / (math.pi * 65.)
        # ms 201..220 still at 200 mm/s, 300 mrad/s (wheel +-24 mm/s); ms 221..420 at 100 mm/s straight
        self.assertAlmostEqual((model.left - left0) / cpm, (200 - 24) * 0.020 + 100 * 0.200, places=9)
        self.assertAlmostEqual((model.right - right0) / cpm, (200 + 24) * 0.020 + 100 * 0.200, places=9)


class HeartbeatSources(unittest.TestCase):
    def test_file_heartbeat(self):
        from tools.chassis_uart.mcu_sim import FileHeartbeat
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'hb'
            hb = FileHeartbeat(str(p))
            self.assertFalse(hb(0))                 # absent -> no heartbeat
            p.write_text('1')
            self.assertTrue(hb(1))
            p.write_text('0')
            self.assertFalse(hb(2))

    def test_kv_permit_heartbeat_follows_permit_bit(self):
        import time
        from host.kv_regs import PERMIT_STATUS, mock_permit
        from tools.chassis_uart.mcu_sim import KvPermitHeartbeat
        perm, win = mock_permit(status=1)
        sysinfo, _ = mock_sysinfo()
        srv = KvLinkServer(NullCore(), 'null', bind=('127.0.0.1', 0), regs={'permit': perm, 'sysinfo': sysinfo}).start()
        hb = KvPermitHeartbeat(('127.0.0.1', srv.port), hz=200., max_age_s=0.5)
        try:
            def wait_for(value, limit=3.):
                end = time.monotonic() + limit
                while time.monotonic() < end:
                    if hb(0) == value:
                        return True
                    time.sleep(0.005)
                return False
            self.assertTrue(wait_for(True))
            win.words[PERMIT_STATUS] = 0            # PL revoked the permit -> square wave stops
            self.assertTrue(wait_for(False))
            win.words[PERMIT_STATUS] = 1
            self.assertTrue(wait_for(True))
            srv.stop()                              # KV gone: stale poll reads as no heartbeat
            self.assertTrue(wait_for(False))
        finally:
            hb.stop.set()
            hb.thread.join(timeout=2.)
            hb.client.close()
            srv.stop()


# ------------------------------------------------------------------------------------------- B
def seq_straight(n=200, step=7, start=(0, 0)):
    return [(start[0] + step * k, start[1] + step * k) for k in range(n)]


def seq_rotate(n=300, step=5, start=(0, 0)):
    return [(start[0] - step * k, start[1] + step * k) for k in range(n)]


def seq_arc(n=400, start=(0, 0)):
    return [(start[0] + 4 * k, start[1] + 9 * k) for k in range(n)]


def wrap32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v >= 1 << 31 else v


def closed_arc(n_inc, a, b):
    th = n_inc * (b - a) * MPC / TRACK
    radius = TRACK / 2. * (a + b) / (b - a)
    return radius * math.sin(th), radius * (1 - math.cos(th)), th


def run_odom(counts):
    odom = WheelOdometry()
    out = []
    for k, (l, r) in enumerate(counts):
        out.append(odom.update(wrap32(l), wrap32(r), k * 10 * MS))
    return odom, out


class OdometryKnownAnswers(unittest.TestCase):
    def assertPose(self, st, x, y, yaw):
        self.assertLessEqual(abs(st['x'] - x), TOL, (st['x'], x))
        self.assertLessEqual(abs(st['y'] - y), TOL, (st['y'], y))
        self.assertLessEqual(abs(st['yaw'] - yaw), TOL, (st['yaw'], yaw))

    def test_placeholder_parameters(self):
        p = WheelParams()
        self.assertTrue(p.placeholder)
        self.assertEqual(p.m_per_count, MPC)
        self.assertEqual(p.pl_registers(), {'WHEEL_DIAM_UM': 65000, 'COUNTS_PER_REV': 1320,
                                            'TRACK_UM': 160000, 'MAX_DELTA_COUNTS': 32767})
        self.assertFalse(WheelParams(wheel_diam_m=0.066).placeholder)

    def test_B1_straight(self):
        odom, out = run_odom(seq_straight())
        self.assertPose(out[-1], 199 * 7 * MPC, 0., 0.)
        self.assertAlmostEqual(out[-1]['v'], 7 * MPC / 0.01, places=9)
        self.assertEqual(odom.counters['integrated'], 199)

    def test_B2_rotate_in_place(self):
        _, out = run_odom(seq_rotate())
        self.assertPose(out[-1], 0., 0., 299 * 10 * MPC / TRACK)
        self.assertAlmostEqual(out[-1]['w'], 10 * MPC / TRACK / 0.01, places=9)

    def test_B3_arc(self):
        _, out = run_odom(seq_arc())
        self.assertPose(out[-1], *closed_arc(399, 4, 9))
        # every intermediate pose on the closed form too (not only the end)
        for k in (1, 57, 200):
            self.assertPose(out[k], *closed_arc(k, 4, 9))

    def test_B4_wrap_crossing(self):
        start = ((1 << 31) - 50 * 4 - 3, (1 << 31) - 50 * 9 - 1)
        counts = seq_arc(start=start)
        self.assertTrue(any(wrap32(l) < 0 for l, _ in counts) and wrap32(counts[0][0]) > 0)
        odom, out = run_odom(counts)
        self.assertPose(out[-1], *closed_arc(399, 4, 9))
        self.assertEqual(odom.counters['jumps'], 0)
        self.assertEqual(s32_delta(-(1 << 31), (1 << 31) - 1), 1)

    def test_B5_first_and_jump(self):
        counts = seq_straight()
        counts = counts[:100] + [(l + 100000, r + 100000) for l, r in counts[100:]]
        odom, out = run_odom(counts)
        self.assertEqual(out[0]['event'], 'first')
        self.assertEqual((out[0]['x'], out[0]['v']), (0., 0.))
        self.assertEqual(out[100]['event'], 'jump')
        self.assertEqual(odom.counters['jumps'], 1)
        self.assertPose(out[-1], 198 * 7 * MPC, 0., 0.)
        # explicit rebaseline: the next sample only re-anchors
        odom.rebaseline('test')
        st = odom.update(wrap32(counts[-1][0] - 5000), wrap32(counts[-1][1] - 5000), 3000 * MS)
        self.assertEqual(st['event'], 'rebaseline')
        self.assertPose(st, 198 * 7 * MPC, 0., 0.)

    def test_duplicate_stamp_without_motion_is_skipped(self):
        odom = WheelOdometry()
        odom.update(0, 0, 0)
        odom.update(7, 7, 10 * MS)
        self.assertEqual(odom.update(7, 7, 10 * MS)['event'], 'duplicate')

    def test_quaternion(self):
        qx, qy, qz, qw = yaw_to_quaternion(math.pi / 2)
        self.assertAlmostEqual(qz, math.sqrt(.5))
        self.assertAlmostEqual(qw, math.sqrt(.5))


# ---------------------------------------------------------------------------------- B6 two sources
def via_chassis(counts):
    """Counts -> MCU status frames -> parser -> ChassisLink -> driver JSON row -> OdomCore."""
    link, parser, core = ChassisLink(), FrameParser(accept={TYPE_STATUS}), OdomCore('chassis')
    out = []
    for k, (l, r) in enumerate(counts):
        frame = encode_status(k & 0xFF, k * 10, wrap32(l), wrap32(r), 0)
        t = k * 10 * MS
        for row in parser.feed(frame):
            derived = link.on_status(row, t)
            body = json.loads(json.dumps(wheel_counts_row(row, derived, t)))   # through the topic's JSON
            st = core.feed_chassis(body)
            if st is not None:
                out.append(st)
    return out


def via_pl(counts, client, win):
    """Counts -> PL encoder register words -> kv_server TELEM over UDP -> decode -> OdomCore."""
    core = OdomCore('pl')
    out = []
    want = blocks_mask(('encoder', 'board'))
    for k, (l, r) in enumerate(counts):
        win.words.update(encoder_words(update_count=k + 1, left=wrap32(l), right=wrap32(r),
                                       frame_ts=1000 + k, seq_word=(k * 10 & 0xFFFF) << 16 | (k & 0xFF)))
        tm = client.poll(want)
        block = tm.blocks['encoder']
        block.pop('frame_age_ms', None)            # Nano-side stamp = poll time (deterministic here)
        st = core.feed_pl(block, k * 10 * MS)
        if st is not None:
            out.append(st)
    return out


class TwoSourcesAgree(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.enc, cls.win = mock_encoder(encoder_words(left=0, right=0))
        sysinfo, _ = mock_sysinfo()
        cls.srv = KvLinkServer(NullCore(), 'null', bind=('127.0.0.1', 0),
                               regs={'encoder': cls.enc, 'sysinfo': sysinfo}).start()
        cls.client = KvTelemetryClient(('127.0.0.1', cls.srv.port), timeout_s=1.0)

    @classmethod
    def tearDownClass(cls):
        cls.client.close()
        cls.srv.stop()

    def check(self, counts, expected):
        a, b = via_chassis(counts), via_pl(counts, self.client, self.win)
        self.assertEqual(len(a), len(counts))
        self.assertEqual(len(a), len(b))
        worst = max(max(abs(p[k] - q[k]) for k in ('x', 'y', 'yaw')) for p, q in zip(a, b))
        self.assertLessEqual(worst, 1e-9)
        for st in (a[-1], b[-1]):
            for got, want in zip((st['x'], st['y'], st['yaw']), expected):
                self.assertLessEqual(abs(got - want), TOL)
        return worst

    def test_B6_straight_rotate_arc_wrap(self):
        cases = [(seq_straight(), (199 * 7 * MPC, 0., 0.)),
                 (seq_rotate(), (0., 0., 299 * 10 * MPC / TRACK)),
                 (seq_arc(), closed_arc(399, 4, 9)),
                 (seq_arc(start=((1 << 31) - 203, (1 << 31) - 451)), closed_arc(399, 4, 9))]
        worst = [self.check(c, e) for c, e in cases]
        self.assertEqual(worst, [0., 0., 0., 0.])

    def test_pl_path_carries_pl_fixed_point_and_skips_repeats(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'pl.jsonl'
            with open(path, 'w') as f:
                core = OdomCore('pl', record=f)
                want = blocks_mask(('encoder', 'board'))
                self.win.words.update(encoder_words(update_count=7, left=100, right=100, dist_center_um=15470,
                                                    heading_q16=0))
                self.assertEqual(core.feed_pl(self.client.poll(want).blocks['encoder'], 0)['event'], 'first')
                self.assertIsNone(core.feed_pl(self.client.poll(want).blocks['encoder'], 10 * MS))  # same update
                self.assertEqual(core.counters['skipped'], 1)
            row = json.loads(path.read_text().splitlines()[0])
            self.assertEqual((row['pl_dist_center_m'], row['pl_heading_rad']), (0.01547, 0.))

    def test_positive_control_two_paths_would_differ(self):
        """The comparison is not vacuous: one count off in the PL path shows up."""
        counts = seq_arc(n=50)
        bad = counts[:25] + [(l + 1, r) for l, r in counts[25:]]
        a, b = via_chassis(counts), via_pl(bad, self.client, self.win)
        self.assertGreater(abs(a[-1]['yaw'] - b[-1]['yaw']), 1e-5)


class PlTrackerHints(unittest.TestCase):
    def block(self, update, left, right, jumps=0, link_ok=True):
        return {'available': True, 'update_count': update, 'left_count': left, 'right_count': right,
                'counters_named': {'count_jumps': jumps}, 'link_ok': link_ok, 'frame_age_ms': 2.5,
                'seq': {'seq': update & 0xFF, 'mcu_ms': update * 10}}

    def test_hints(self):
        tr = PlSampleTracker()
        s = tr.sample(self.block(1, 0, 0), 100 * MS)
        self.assertEqual(s[2], 100 * MS - 2500000)          # poll time minus the PL frame age
        self.assertIsNone(s[3])
        self.assertIsNone(tr.sample(self.block(1, 0, 0), 110 * MS))          # same publication
        self.assertIsNone(tr.sample(self.block(2, 5, 5, link_ok=False), 120 * MS))
        self.assertIn('regained', tr.sample(self.block(3, 9, 9), 130 * MS)[3])
        self.assertIn('count_jumps', tr.sample(self.block(4, 9, 9, jumps=1), 140 * MS)[3])
        self.assertIsNone(tr.sample({'available': False}, 150 * MS))

    def test_chassis_silence_rebaselines(self):
        core = OdomCore('chassis')
        base = {'seq': 0, 'mcu_ms': 0, 'left_count': 1000, 'right_count': 1000, 'stamp_ns': 0, 't_ns': 0}
        core.feed_chassis(base)
        core.feed_chassis(dict(base, seq=1, mcu_ms=10, left_count=1007, right_count=1007, t_ns=10 * MS))
        st = core.feed_chassis(dict(base, seq=9, mcu_ms=5, left_count=3, right_count=3, t_ns=5000 * MS,
                                    after_silence=True))     # MCU reset during a silence
        self.assertEqual(st['event'], 'rebaseline')
        self.assertAlmostEqual(st['x'], 7 * MPC)
        self.assertEqual(core.feed_chassis({'bad': 1}), None)
        self.assertEqual(core.counters['bad_input'], 1)


class RosShellContract(unittest.TestCase):
    def test_odometry_fields_and_no_tf(self):
        st = {'x': 1., 'y': 2., 'yaw': math.pi, 'v': .3, 'w': -.4}
        f = odometry_fields(st)
        self.assertEqual((f['frame_id'], f['child_frame_id']), ('odom', 'base_link'))
        self.assertEqual(f['linear'], (.3, 0., 0.))
        self.assertEqual(len(f['pose_covariance']), 36)
        src = (ROOT / 'host/ros_wheel_odom.py').read_text()
        code = src.split('"""', 2)[2]               # past the docstring, which mentions TF on purpose
        for forbidden in ('TransformBroadcaster', 'tf2_ros', "'/tf'", 'sendTransform'):
            self.assertNotIn(forbidden, code)
        self.assertFalse(OdomCore('chassis').status()['publishes_tf'])

    def test_record_rows_feed_cross_check(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'a.jsonl'
            with open(path, 'w') as f:
                core = OdomCore('chassis', record=f)
                for k, (l, r) in enumerate(seq_arc(n=30)):
                    core.feed_chassis({'seq': k, 'mcu_ms': k * 10, 'left_count': l, 'right_count': r,
                                       't_ns': k * 10 * MS, 'stamp_ns': k * 10 * MS})
            samples = cross_check.load_samples(str(path))
            self.assertEqual(len(samples), 30)
            series, _, _ = cross_check.replay(samples, WheelParams())
            self.assertLessEqual(abs(series[-1][2] - closed_arc(29, 4, 9)[2]), TOL)


# ------------------------------------------------------------------------------------------- C
def write_record(path, counts, period_ms, offset_ms=0, scale=1.0, source='chassis'):
    with open(path, 'w') as f:
        for k, (l, r) in enumerate(counts):
            if (k * 10) % period_ms:
                continue
            f.write(json.dumps({'source': source, 't_ns': (k * 10 + offset_ms) * MS,
                                'left': int(round(l * scale)), 'right': int(round(r * scale)),
                                'event': 'ok'}) + '\n')


class CrossCheckControls(unittest.TestCase):
    def test_negative_and_positive_control(self):
        counts = seq_arc(n=1000)          # 10 s, constant curvature
        with tempfile.TemporaryDirectory() as d:
            a, b, c = (str(Path(d) / n) for n in ('a.jsonl', 'b.jsonl', 'c.jsonl'))
            write_record(a, counts, 10)                      # chassis 100 Hz
            write_record(b, counts, 50, source='pl')         # PL polled at 20 Hz
            write_record(c, counts, 50, source='pl', scale=1.01)
            same = cross_check.compare(cross_check.load_samples(a), cross_check.load_samples(b), WheelParams())
            self.assertLessEqual(same['distance_abs_diff_m']['max'], TOL)
            self.assertLessEqual(same['yaw_abs_diff_rad']['max'], TOL)
            self.assertGreater(same['compared_points'], 150)
            off = cross_check.compare(cross_check.load_samples(a), cross_check.load_samples(c), WheelParams())
            self.assertLessEqual(abs(off['distance_rel_end_diff_b_minus_a'] - 0.01), 0.0005)
            self.assertGreater(off['distance_abs_diff_m']['max'], 1e-3)
            with contextlib.redirect_stdout(io.StringIO()):
                rc = cross_check.main([a, b, '--out', str(Path(d) / 'r.json')])
            self.assertEqual(rc, 0)
            self.assertIn('distance_abs_diff_m', json.loads((Path(d) / 'r.json').read_text()))

    def test_reads_chassis_driver_record(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'drv.jsonl'
            with open(p, 'w') as f:
                f.write(json.dumps({'dir': 'tx', 't_ns': 0, 'enable': False}) + '\n')
                for k, (l, r) in enumerate(seq_straight(n=20)):
                    f.write(json.dumps({'dir': 'rx', 't_ns': k * 10 * MS, 'left_count': l, 'right_count': r,
                                        'after_silence': False}) + '\n')
            self.assertEqual(len(cross_check.load_samples(str(p))), 20)


if __name__ == '__main__':
    unittest.main()
