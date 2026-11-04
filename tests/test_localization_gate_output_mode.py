"""count_mode='output' (owner decision 2026-10-15): count consecutive rtabmap outputs.

What changed and what must not have: runs/real-car-integ-20261015-results/gate_dig/DIG.md §4-5.
Every test here drives the real LocalizationGate; ROS is not needed. The replay class at the end
feeds the recorded host v6a2 run (tests/data/gate_v6a2_20261015) through both modes.
"""
import collections
import copy
import gzip
import inspect
import json
from pathlib import Path
import re
import unittest

from host.localization_gate import KINDS, LocalizationGate
import host.ros_localization_gate as node

PRIOR = {1, 2}
BASE = 10**9          # stamp of frame 0; frame i is BASE + i*10**8 (10 fps camera)


def stamp_of(i):
    return BASE + i*10**8


def row(kind, stamp, ref=None, **change):
    pose = {'position': dict(x=1., y=2., z=3.), 'orientation': dict(x=0., y=0., z=0., w=1.)}
    cov = [float(i % 7 == 0)*.01 for i in range(36)]
    result = {'header': {'stamp': dict(sec=stamp//10**9, nanosec=stamp % 10**9),
                         'frame_id': 'odom' if kind in KINDS[:2] else 'map'},
              'child_frame_id': 'camera_link', 'pose': {'pose': pose, 'covariance': cov},
              'covariance': cov, 'lost': False, 'loop_closure_id': 1, 'proximity_detection_id': 0}
    if kind == 'info':
        result['ref_id'] = ref
    result.update(change)
    return result


class Counting(LocalizationGate):
    """The real gate; only records each revoke reason on the way through."""

    def __init__(self, *args, **kwargs):
        kwargs.setdefault('count_mode', 'output')
        super().__init__(*args, **kwargs)
        self.revokes = collections.Counter()

    def revoke(self, state, reason):
        self.revokes[reason] += 1
        super().revoke(state, reason)


class Driver:
    """Feeds one gate on a shared clock. wall seconds == ros seconds - 1 unless set."""

    def __init__(self, **kwargs):
        self.gate = Counting('map-hash', PRIOR, **kwargs)
        self.approved = []

    def clock(self, i, late=0.):
        return i*.1 + late, stamp_of(i) + int(round(late*10**9))

    def rgb(self, i, late=0.):
        out = self.gate.acquisition(stamp_of(i), *self.clock(i, late))
        self.approved += out
        return out

    def out(self, kind, i, ref=None, late=.05, **change):
        out = self.gate.ingest(kind, row(kind, stamp_of(i), ref, **change), *self.clock(i, late))
        self.approved += out
        return out

    def outputs(self, i, ref, kinds=KINDS, late=.05, **change):
        out = []
        for kind in kinds:
            out += self.out(kind, i, ref, late, **(change if kind == 'info' else {}))
        return out

    def processed(self, i, ref, **change):
        """A frame odometry and rtabmap both processed, rgb first."""
        self.rgb(i)
        return self.outputs(i, ref, **change)

    def tick(self, i, late):
        self.gate.tick(*self.clock(i, late))


class OutputCountingTests(unittest.TestCase):
    def test_three_consecutive_rtabmap_outputs_validate_across_skipped_frames(self):
        # DIG §5 test 4: frames odometry never processed, and frames only odometry processed,
        # between three healthy rtabmap outputs are skipped: valid, and nothing revoked.
        d = Driver()
        d.processed(1, 68)
        d.rgb(2)                                     # nobody processed it
        d.rgb(3)
        d.outputs(3, None, kinds=('odom', 'odom_info'))   # odometry only
        d.processed(4, 69)
        d.rgb(5)
        self.assertEqual(d.processed(6, 70)[0]['stamp_ns'], stamp_of(6))
        self.assertEqual(d.gate.state, 'valid')
        self.assertEqual(sum(d.gate.revokes.values()), 0, d.gate.revokes)
        self.assertEqual([a['stamp_ns'] for a in d.approved], [stamp_of(6)])
        self.assertFalse(d.gate.pending)

    def test_frame_mode_on_the_same_input_never_validates(self):
        # Discriminating control for the test above: the old count fails on skipped frames.
        d = Driver(count_mode='frame')
        d.processed(1, 68)
        d.rgb(2)
        d.rgb(3)
        d.outputs(3, None, kinds=('odom', 'odom_info'))
        d.processed(4, 69)
        d.rgb(5)
        d.processed(6, 70)
        d.tick(6, .6)
        self.assertFalse(d.approved)
        self.assertIn('incomplete_acquisition_timeout', d.gate.revokes)

    def test_outputs_before_rgb_are_held_then_joined(self):
        # DIG §5 test 1: all four first, and only some first.
        for early in (KINDS, ('odom', 'odom_info'), ('odom',), ('info', 'localization_pose')):
            with self.subTest(early=early):
                d = Driver()
                for n, i in enumerate((1, 2, 3)):
                    ref = 68 + n
                    d.outputs(i, ref, kinds=early, late=0.)      # before the frame arrives
                    self.assertFalse(d.gate.revokes, d.gate.revokes)
                    d.rgb(i, late=.01)
                    d.outputs(i, ref, kinds=[k for k in KINDS if k not in early])
                self.assertEqual(d.gate.state, 'valid')
                self.assertEqual([a['stamp_ns'] for a in d.approved], [stamp_of(3)])
                self.assertFalse(d.gate.early)
                self.assertEqual(sum(d.gate.revokes.values()), 0, d.gate.revokes)

    def test_an_approval_completed_by_the_arriving_frame_is_returned_by_acquisition(self):
        d = Driver()
        d.processed(1, 68)
        d.processed(2, 69)
        d.outputs(3, 70, late=0.)
        out = d.gate.acquisition(stamp_of(3), *d.clock(3, .01))
        self.assertEqual([a['stamp_ns'] for a in out], [stamp_of(3)])
        self.assertEqual(d.gate.state, 'valid')

    def test_held_output_expires_fail_closed(self):
        # DIG §5 test 2: 0.5 s wall, stamp age > 1 s, or a newer frame arrived instead.
        def held_then(after):
            d = Driver()
            d.processed(1, 68)
            d.processed(2, 69)
            self.assertEqual(d.gate.streak, 2)
            d.out('odom', 4, late=.01)           # frame 4's odometry, its rgb not yet in
            self.assertEqual(len(d.gate.early), 1)
            after(d)
            self.assertEqual(d.gate.revokes['unexpected_or_retired_stamp'], 1, d.gate.revokes)
            self.assertEqual(d.gate.streak, 0)
            self.assertFalse(d.gate.early)
        held_then(lambda d: d.tick(4, .51))                                   # held 0.5 s of wall
        held_then(lambda d: d.gate.tick(.42, stamp_of(4) + 10**9 + 1))        # stamp age > 1 s
        held_then(lambda d: (d.rgb(5, late=.01), d.tick(5, .02)))             # a newer frame came
        # Control: held 0.49 s and then joined by its frame is not a failure.
        d = Driver()
        d.processed(1, 68)
        d.out('odom', 2, late=.01)
        d.tick(2, .49)
        d.rgb(2, late=.495)
        self.assertEqual(sum(d.gate.revokes.values()), 0, d.gate.revokes)

    def test_held_duplicate_and_hold_overflow(self):
        # DIG §5 test 3.
        d = Driver()
        d.processed(1, 68)
        d.out('odom', 2, late=.01)
        d.out('odom', 2, late=.02)
        self.assertEqual(d.gate.revokes['duplicate_message'], 1)
        self.assertEqual(d.gate.streak, 0)
        d.rgb(2, late=.03)                        # the frame stays failed
        d.outputs(2, 69, kinds=('odom_info', 'info', 'localization_pose'))
        self.assertGreaterEqual(d.gate.revokes['unhealthy_acquisition'], 1)
        self.assertEqual(d.gate.streak, 0)

        d = Driver()
        d.processed(1, 68)
        d.processed(2, 69)
        for j in range(33):                       # 33 distinct future stamps held at once
            d.gate.ingest('odom', row('odom', stamp_of(3) + j*1000), .25, stamp_of(3) + 33*1000)
        self.assertEqual(d.gate.revokes['pending_overflow'], 1)
        self.assertEqual(d.gate.pending_overflows, 1)
        self.assertEqual(d.gate.streak, 0)
        self.assertLessEqual(len(d.gate.early), 32)

    def test_pending_queue_overflow_is_counted_in_both_modes(self):
        for mode in ('frame', 'output'):
            gate = Counting('map-hash', PRIOR, count_mode=mode)
            for i in range(33):
                gate.acquisition(BASE + i, i*.001, BASE + i)
            self.assertEqual((gate.pending_overflows, gate.revokes['pending_overflow']), (1, 1), mode)

    def test_a_skipped_frame_cannot_launder_a_bad_output(self):
        # DIG §5 test 5: rtabmap never answered frame 3, but its odometry reported lost.
        for via in ('newer_output', 'timeout'):
            with self.subTest(via=via):
                d = Driver()
                d.processed(1, 68)
                d.processed(2, 69)
                d.rgb(3)
                d.out('odom', 3)
                d.out('odom_info', 3, lost=True)
                if via == 'newer_output':
                    d.processed(4, 70)
                else:
                    d.tick(3, .51)
                    d.processed(4, 70)
                # The bad frame reset the count: frame 4 starts it again, three fresh needed.
                self.assertEqual(d.gate.streak, 1, d.gate.revokes)
                self.assertFalse(d.processed(5, 71))
                self.assertFalse(d.approved)
                self.assertEqual(len(d.processed(6, 72)), 1)
                self.assertTrue(d.gate.revokes['odom_info:odometry_lost'])
                if via == 'newer_output':
                    # frame 4 completed while known-bad frame 3 was still ahead of it
                    self.assertTrue(d.gate.revokes['unhealthy_acquisition'], d.gate.revokes)
                else:
                    self.assertTrue(d.gate.revokes['incomplete_acquisition_timeout'], d.gate.revokes)

    def test_an_unlinked_info_breaks_the_count(self):
        # DIG §5 test 6.
        d = Driver()
        d.processed(1, 68)
        d.processed(2, 69, loop_closure_id=999)
        d.processed(3, 70)
        self.assertNotEqual(d.gate.state, 'valid')
        self.assertFalse(d.approved)
        self.assertEqual(d.gate.revokes['info:no_prior_map_match'], 1)
        d.processed(4, 71)
        self.assertEqual(len(d.processed(5, 72)), 1)

    def test_rtabmap_output_gap_resets_and_consecutive_ref_validates(self):
        # DIG §5 test 7: 68, 69, 71 -> gap; three more consecutive needed.
        d = Driver()
        d.processed(1, 68)
        d.processed(2, 69)
        d.rgb(3)                                  # rtabmap's output for frame 3 was lost
        self.assertFalse(d.processed(4, 71))
        self.assertEqual(d.gate.revokes['rtabmap_output_gap'], 1)
        self.assertEqual(d.gate.streak, 1)
        self.assertFalse(d.processed(5, 72))
        self.assertEqual(len(d.processed(6, 73)), 1)
        self.assertEqual(d.gate.state, 'valid')

    def test_info_without_ref_id_is_bad_in_output_mode_only(self):
        d = Driver()
        d.processed(1, None)
        self.assertEqual(d.gate.revokes['info:missing_ref_id'], 1)
        frame = Driver(count_mode='frame')
        for i in (1, 2, 3):
            frame.processed(i, None)
        self.assertEqual(frame.gate.state, 'valid')

    def test_timing_limits_unchanged(self):
        # DIG §5 test 8. Stream: two counted outputs >= 1 s apart.
        d = Driver()
        d.processed(1, 68)
        d.processed(2, 69)
        d.gate.tick(.2 + .05 + 1., stamp_of(3))
        self.assertEqual(d.gate.revokes['localization_stream_timeout'], 1)
        self.assertEqual(d.gate.streak, 0)
        # rtabmap later than the 0.5 s window: its output meets a retired frame and revokes.
        d = Driver()
        d.processed(1, 68)
        d.processed(2, 69)
        d.rgb(3)
        d.outputs(3, None, kinds=('odom', 'odom_info'))
        d.tick(3, .51)
        self.assertEqual(d.gate.streak, 2)        # unanswered by then: skipped, not yet failed
        d.outputs(3, 70, kinds=('info', 'localization_pose'), late=.6)
        self.assertEqual(d.gate.revokes['unexpected_or_retired_stamp'], 2)
        self.assertEqual(d.gate.streak, 0)
        self.assertFalse(d.approved)
        # rtabmap partly answered (info in, pose late): the frame times out exactly as before.
        d = Driver()
        d.processed(1, 68)
        d.rgb(2)
        d.outputs(2, 69, kinds=('odom', 'odom_info', 'info'))
        d.tick(2, .51)
        self.assertEqual(d.gate.revokes['incomplete_acquisition_timeout'], 1)
        # Acquisition stamp age limit is untouched.
        d = Driver()
        d.gate.acquisition(stamp_of(1), .1, stamp_of(1) + 10**9 + 1)
        self.assertEqual(d.gate.revokes['acquisition_stamp_invalid_or_duplicate'], 1)

    def test_discontinuity_and_clock_rollback_clear_output_tracking(self):
        for reset in ('discontinuity', 'rollback'):
            d = Driver()
            d.processed(1, 68)
            d.out('odom', 3, late=-.15)
            if reset == 'discontinuity':
                d.gate.discontinuity(1)
            else:
                d.gate.tick(.0, stamp_of(0))
            self.assertEqual((dict(d.gate.early), d.gate.newest_rtab, d.gate.last_ref), ({}, 0, None), reset)

    def test_count_mode_is_validated(self):
        with self.assertRaises(ValueError):
            LocalizationGate('m', {1}, count_mode='frames')


# ------------------------------------------------------------------------------------------
# Recorded-run replay (host v6a2, 2026-10-15). The stream construction follows
# runs/real-car-integ-20261015-results/gate_dig/gate_sim.py: recorded callbacks in recorded
# order with their recorded clocks, the recorded heartbeats plus a 100 ms synthetic heartbeat
# (phase 0), the ROS clock clamped monotonic; message content rebuilt from the fixture.

FIXTURE = Path(__file__).resolve().parent / 'data' / 'gate_v6a2_20261015' / 'replay.json.gz'
GOOD_COV = [0.01 if i % 7 == 0 else 0. for i in range(36)]
BAD_COV = [99999. if i % 7 == 0 else 0. for i in range(36)]


def load_fixture():
    with gzip.open(str(FIXTURE), 'rt') as handle:
        data = json.load(handle)
    data['info'] = {int(k): v for k, v in data['info'].items()}
    data['lost'] = {int(k): v for k, v in data['lost'].items()}
    data['explicit_bad'] = {(k, s): r for k, s, r in data['explicit_bad']}
    return data


def make_row(kind, stamp, data, info, lost):
    header = {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9}}
    pose = {'position': {'x': 0., 'y': 0., 'z': 0.}, 'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}
    bad = data['explicit_bad'].get((kind, stamp))
    if kind == 'odom':
        if bad == 'invalid_pose':
            pose['orientation']['w'] = 0.
        return {'header': dict(header, frame_id='odom'), 'child_frame_id': 'base_link',
                'pose': {'pose': pose, 'covariance': BAD_COV if bad == 'invalid_covariance' else GOOD_COV}}
    if kind == 'odom_info':
        return {'header': dict(header, frame_id='odom'),
                'lost': bool(lost.get(stamp, False)) or bad == 'odometry_lost',
                'covariance': BAD_COV if bad == 'invalid_covariance' else GOOD_COV}
    if kind == 'info':
        loop, proximity, ref = info.get(stamp, (0, 0, 0))
        return {'header': dict(header, frame_id='map'), 'loop_closure_id': loop,
                'proximity_detection_id': proximity, 'ref_id': ref}
    return {'header': dict(header, frame_id='map'), 'pose': {'pose': pose, 'covariance': GOOD_COV}}


def build_stream(data, keep_rtab=None, drop_kinds=()):
    codes = data['event_codes']
    stream = []
    for code, mono, ros, stamp, state, reason, epoch, pending in data['events']:
        event = codes[code]
        logged = (state, reason, epoch, pending)
        if event == 'acquisition':
            stream.append([mono, ros, 'acq', None, stamp, logged])
        elif event.startswith('received_'):
            kind = event[len('received_'):]
            if kind in drop_kinds:
                continue
            if kind in ('info', 'localization_pose') and keep_rtab is not None and stamp not in keep_rtab:
                continue
            stream.append([mono, ros, 'msg', kind, stamp, logged])
        else:
            stream.append([mono, ros, 'tick', None, None, None])
    first, last = min(x[0] for x in stream), max(x[0] for x in stream)
    offset = min(x[1] - x[0] for x in stream)
    t = first
    while t < last:
        stream.append([t, t + offset, 'tick', None, None, None])
        t += 100 * 10**6
    stream.sort(key=lambda x: (x[0], {'tick': 1, 'acq': 0, 'msg': 0}[x[2]]))
    previous = 0
    for x in stream:
        x[1] = previous = max(x[1], previous)
    return stream


def replay(data, count_mode, info=None, lost=None, keep_rtab=None, drop_kinds=()):
    info = data['info'] if info is None else info
    lost = data['lost'] if lost is None else lost
    gate = Counting('d' * 64, data['prior_ids'], tracked_frame='base_link', count_mode=count_mode)
    stream = build_stream(data, keep_rtab, drop_kinds)
    approvals, timeline = [], []
    match = total = 0
    first_output = next(x[0] for x in stream if x[2] == 'msg')
    for mono, ros, op, kind, stamp, logged in stream:
        wall = mono / 1e9
        if op == 'acq':
            out = gate.acquisition(stamp, wall, ros)
        elif op == 'msg':
            out = gate.ingest(kind, make_row(kind, stamp, data, info, lost), wall, ros)
        else:
            gate.status(wall, ros)
            out = []
        approvals += [(ros, item['stamp_ns']) for item in out]
        timeline.append((mono, gate.state))
        if logged is not None and keep_rtab is None and not drop_kinds:
            total += 1
            match += (gate.state, gate.reason, gate.epoch, len(gate.pending)) == tuple(logged)
    first_valid = next((m for m, s in timeline if s == 'valid'), None)
    return {'approvals': len(approvals),
            'first_valid_s': None if first_valid is None else (first_valid - first_output) / 1e9,
            'max_age_ms': max(((r - s) / 1e6 for r, s in approvals), default=None),
            'revokes': gate.revokes, 'pending_overflows': gate.pending_overflows,
            'match': (match, total)}


@unittest.skipUnless(FIXTURE.exists(), 'replay fixture missing')
class RecordedReplayTests(unittest.TestCase):
    """DIG §5 test 9. Known answers from the offline simulator (gate_dig/results_v6a2.json)."""

    @classmethod
    def setUpClass(cls):
        cls.data = load_fixture()

    def test_fixture_is_the_recorded_run(self):
        codes = collections.Counter(self.data['event_codes'][e[0]] for e in self.data['events'])
        self.assertEqual((codes['acquisition'], codes['received_odom'], codes['received_info']), (830, 458, 220))
        refs = sorted(v[2] for v in self.data['info'].values())
        self.assertEqual(refs, list(range(68, 288)))

    def test_frame_mode_reproduces_the_recorded_gate_and_never_approves(self):
        result = replay(self.data, 'frame')
        match, total = result['match']
        self.assertGreaterEqual(match / total, .998, result['match'])  # simulator: 2182/2186
        self.assertEqual(result['approvals'], 0)

    def test_output_mode_approves_with_bounded_age(self):
        result = replay(self.data, 'output')
        self.assertEqual(result['approvals'], 29, result)              # simulator AB_R: 29
        self.assertLess(result['first_valid_s'], 10.)
        self.assertLess(result['max_age_ms'], 1000.)
        self.assertEqual(result['pending_overflows'], 0)

    def test_negative_no_info_links_the_prior_map(self):
        nolink = {s: [0, 0, v[2]] for s, v in self.data['info'].items()}
        for mode in ('frame', 'output'):
            self.assertEqual(replay(self.data, mode, info=nolink)['approvals'], 0, mode)

    def test_negative_odometry_always_lost(self):
        all_lost = {s: True for s in self.data['lost']}
        for mode in ('frame', 'output'):
            self.assertEqual(replay(self.data, mode, lost=all_lost)['approvals'], 0, mode)

    def test_negative_odometry_stream_absent(self):
        for drop in (('odom',), ('odom_info',), ('odom', 'odom_info')):
            self.assertEqual(replay(self.data, 'output', drop_kinds=drop)['approvals'], 0, drop)

    def test_negative_every_fourth_rtabmap_output_lost(self):
        stamps = sorted(self.data['info'])
        keep = {s for n, s in enumerate(stamps) if n % 4 != 3}
        result = replay(self.data, 'output', keep_rtab=keep)
        self.assertEqual(result['approvals'], 10, result)               # simulator AB_R: 10
        self.assertGreater(result['revokes']['rtabmap_output_gap'], 0)


class NodeOutputWiringTests(unittest.TestCase):
    """DIG §5 test 10, by source (no ROS here; style of test_localization_gate.py)."""

    source = inspect.getsource(node.main)

    def body(self, name):
        match = re.search(r'\n        def %s\(.*?(?=\n        def |\n    rclpy\.init)' % name, self.source, re.S)
        self.assertIsNotNone(match, name)
        return match.group(0)

    def test_acquisition_publishes_and_records_the_approvals_it_completes(self):
        acquisition = self.body('acquisition')
        self.assertIn('outputs = self.gate.acquisition(s.sec*10**9+s.nanosec, *clock)', acquisition)
        self.assertRegex(acquisition, r"self\.record\('acquisition', clock, stamp_ns=[^\n]*approved=outputs")
        self.assertIn('self.publish_approvals(outputs)', acquisition)
        self.assertLess(acquisition.index('self.publish_approvals(outputs)'), acquisition.index('self.heartbeat(clock)'))

    def test_receive_publishes_through_the_same_method(self):
        receive = self.body('receive')
        self.assertIn('self.publish_approvals(outputs)', receive)
        self.assertNotIn('self.pose_pub.publish', receive)
        publish = self.body('publish_approvals')
        self.assertIn('self.pose_pub.publish(msg)', publish)
        self.assertIn('self.tf_pub.publish(TFMessage(transforms=transforms))', publish)

    def test_options_and_defaults(self):
        # node default changed to correction 10-15 evening (test_localization_gate_correction_mode)
        self.assertRegex(self.source, r"'--count-mode', choices=COUNT_MODES, default='correction'")
        self.assertRegex(self.source, r"'--acquisition-topic', choices=\('rgb', 'camera_info'\), default='rgb'")
        self.assertIn('count_mode=args.count_mode', self.source)
        self.assertIn("args.prefix+'/'+args.acquisition_topic", self.source)
        self.assertIn('pending_overflows=self.gate.pending_overflows', self.source)


class CarScriptGateOptionTests(unittest.TestCase):
    """tools/car_run/rtabmap_car.sh starts the gate (loc + visual only) with these options."""

    script = (Path(__file__).resolve().parents[1] / 'tools' / 'car_run' / 'rtabmap_car.sh').read_text()

    def test_defaults_and_validation(self):
        self.assertIn(': "${GATE_COUNT_MODE:=correction}" "${GATE_ACQ:=rgb}" '
                      '"${COLOR_INFO_TOPIC:=/camera/color/camera_info}" "${BUNDLE:=}"', self.script)
        self.assertIn('case "$GATE_COUNT_MODE" in correction|output|frame)', self.script)
        self.assertIn('case "$GATE_ACQ" in rgb|camera_info)', self.script)
        for name in ('GATE_COUNT_MODE', 'GATE_ACQ', 'COLOR_INFO_TOPIC', 'BUNDLE'):
            self.assertRegex(self.script, r'PASS=\([^)]*\b%s\b' % name)

    def test_bundle_only_when_set_and_only_for_the_gate(self):
        self.assertIn('if [ -n "$BUNDLE" ]; then GA+=(--bundle "$BUNDLE")', self.script)
        code = [line for line in self.script.splitlines() if not line.lstrip().startswith('#')]
        self.assertEqual(sum(line.count('--bundle') for line in code), 1)
        self.assertIn('[ "$MODE" = loc ] && [ -n "$BUNDLE" ] && DEV+=(-v "$BUNDLE:$BUNDLE:ro")', self.script)
        self.assertIn('if [ "$MODE" = loc ] && [ "$ODOM" = visual ] && [ -n "$BUNDLE" ]; then', self.script)
        # the gate (and so GA) exists only in loc + visual
        self.assertIn('GATE=0; [ "$MODE" = loc ] && [ "$ODOM" = visual ] && GATE=1', self.script)
        launch = self.script[self.script.index('if [ "$GATE" = 1 ]; then\n  start adapter'):]
        launch = launch[:launch.index('\nfi\n')]
        self.assertIn('"${GA[@]}"', launch)
        self.assertIn('GA=(--count-mode "$GATE_COUNT_MODE" --acquisition-topic "$GATE_ACQ" '
                      '--max-correction-age-s "$GATE_MAX_CORRECTION_AGE_S")', launch)
        self.assertIn('-r "$PX/camera_info:=$COLOR_INFO_TOPIC"', launch)


if __name__ == '__main__':
    unittest.main()
