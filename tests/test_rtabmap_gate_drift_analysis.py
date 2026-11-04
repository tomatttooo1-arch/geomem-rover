"""Synthetic-recording tests for tools/rtabmap_gate_drift_analysis.py.

Each cause class gets a hand-built recording whose ground truth is known by
construction. The recordings are produced by driving the real gate exactly the
way host/ros_localization_gate.py drives it (call, then record the post-call
state), so a recording is self-consistent and the tool's faithful replay must
reproduce it.
"""
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from host.localization_gate import KINDS, LocalizationGate  # noqa: E402
from tools import rtabmap_gate_drift_analysis as drift  # noqa: E402

MAP_ID = 'a'*64
PRIOR = [1, 2, 3]
GOOD_COV = [(.01 if i % 7 == 0 else 0.) for i in range(36)]
BAD_COV = [(9999. if i % 7 == 0 else 0.) for i in range(36)]
IDENTITY_POSE = {'position': {'x': 0., 'y': 0., 'z': 0.},
                 'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}


def header(stamp, frame):
    return {'stamp': {'sec': stamp//10**9, 'nanosec': stamp % 10**9}, 'frame_id': frame}


def message(kind, stamp, flaw=None):
    if kind == 'odom':
        return {'event': 'odom', 'stamp_ns': stamp, 'header': header(stamp, 'odom'),
                'child_frame_id': 'camera_link',
                'pose': {'pose': IDENTITY_POSE,
                         'covariance': BAD_COV if flaw == 'odom_cov' else GOOD_COV}}
    if kind == 'odom_info':
        return {'event': 'odom_info', 'stamp_ns': stamp, 'header': header(stamp, 'odom'),
                'lost': flaw == 'lost', 'covariance': GOOD_COV}
    if kind == 'info':
        link = 0 if flaw == 'no_match' else 2
        return {'event': 'info', 'stamp_ns': stamp, 'header': header(stamp, 'map'),
                'ref_id': 7, 'loop_closure_id': link, 'proximity_detection_id': 0}
    if kind == 'localization_pose':
        return {'event': 'localization_pose', 'stamp_ns': stamp, 'header': header(stamp, 'map'),
                'pose': {'pose': IDENTITY_POSE,
                         'covariance': BAD_COV if flaw == 'pose_cov' else GOOD_COV}}
    raise ValueError(kind)


class Recorder:
    """Drives the real gate and emits gate-events.jsonl rows in the node's schema."""

    def __init__(self, prior=PRIOR):
        self.gate = LocalizationGate(MAP_ID, prior, 'camera_link', 'map', 'odom')
        self.events, self.content, self.published = [], {}, {}
        self.index = 0

    def _record(self, event, wall, ros, stamp, approved=None):
        row = {'event': event, 'monotonic_ns': int(round(wall*1e9)), 'ros_ns': ros,
               'state': self.gate.state, 'reason': self.gate.reason, 'epoch': self.gate.epoch,
               'map_id': self.gate.map_id, 'instance': self.gate.instance,
               'pending_count': len(self.gate.pending), 'approved_count': len(self.gate.approved),
               'stamp_ns': stamp}
        if approved is not None:
            row['approved'] = approved
        self.events.append(row)

    def publish(self, stamp, acquire_at, message_walls=None, flaw=None, acquire=True,
                kinds=KINDS):
        self.published[stamp] = self.index
        self.index += 1
        walls = message_walls or {k: acquire_at for k in kinds}
        for kind in KINDS:
            self.content[(kind, stamp)] = message(kind, stamp, flaw if kind == _flawed(flaw) else None)
        if acquire:
            self.gate.acquisition(stamp, acquire_at, stamp)
            self._record('acquisition', acquire_at, stamp, stamp)
        for kind in kinds:
            wall = walls[kind]
            out = self.gate.ingest(kind, self.content[(kind, stamp)], wall, stamp)
            self._record('received_'+kind, wall, stamp, stamp, approved=out)

    def tick(self, wall, ros):
        self.gate.status(wall, ros)
        self._record('state_transition', wall, ros, self.gate.last_complete)

    def run(self, label='synthetic'):
        return drift.Run(label, self.events, self.content, self.published)


def _flawed(flaw):
    return {'odom_cov': 'odom', 'lost': 'odom_info', 'no_match': 'info',
            'pose_cov': 'localization_pose'}.get(flaw)


def healthy_stream(count, start_wall=100., step=.2, first_stamp=10**9, stamp_step=10**8):
    recorder = Recorder()
    for i in range(count):
        recorder.publish(first_stamp+i*stamp_step, start_wall+i*step)
    return recorder


class CauseAttributionTest(unittest.TestCase):
    def causes(self, run):
        faithful, verdict = drift.known_answer(run, PRIOR)
        self.assertEqual(verdict['verdict'], 'PASS', verdict['first_mismatch'])
        rows = drift.attribute(run, PRIOR, faithful)
        return {stamp: (value['cause'], value['detail']) for stamp, value in rows.items()}, verdict

    def test_streak_then_approved(self):
        causes, verdict = self.causes(healthy_stream(5).run())
        stamps = sorted(causes)
        self.assertEqual([causes[s][0] for s in stamps],
                         ['streak-not-yet-3', 'streak-not-yet-3', 'approved', 'approved', 'approved'])
        self.assertEqual(causes[stamps[0]][1], 'run_start')
        self.assertEqual(verdict['recorded_approved'], 3)

    def test_content_bad_each_flaw(self):
        for flaw, expected in (('no_match', 'info:no_prior_map_match'),
                               ('pose_cov', 'localization_pose:invalid_covariance'),
                               ('lost', 'odom_info:odometry_lost'),
                               ('odom_cov', 'odom:invalid_covariance')):
            recorder = healthy_stream(4)
            recorder.publish(10**9+4*10**8, 100.8, flaw=flaw)
            causes, _ = self.causes(recorder.run())
            self.assertEqual(causes[10**9+4*10**8], ('content-bad', expected), flaw)

    def test_wall_timeout_direct_retires_a_healthy_frame(self):
        recorder = healthy_stream(4)
        stamp = 10**9+4*10**8
        # acquisition lands, the four messages arrive 0.6 s later: pending times out first
        recorder.publish(stamp, 100.8, message_walls={k: 101.45 for k in KINDS})
        causes, _ = self.causes(recorder.run())
        self.assertEqual(causes[stamp], ('wall-timeout-direct', 'incomplete_acquisition_timeout'))

    def test_wall_timeout_cascade_after_stream_gap(self):
        recorder = healthy_stream(4)
        stamp = 10**9+4*10**8
        recorder.publish(stamp, 102.5)          # > 1.0 s since the last healthy completion
        causes, _ = self.causes(recorder.run())
        self.assertEqual(causes[stamp], ('wall-timeout-cascade', 'localization_stream_timeout'))

    def test_streak_not_yet_three_after_content_revoke(self):
        recorder = healthy_stream(4)
        recorder.publish(10**9+4*10**8, 100.8, flaw='no_match')
        recorder.publish(10**9+5*10**8, 101.0)
        causes, _ = self.causes(recorder.run())
        self.assertEqual(causes[10**9+5*10**8], ('streak-not-yet-3', 'info:no_prior_map_match'))

    def test_missing_acquisition(self):
        recorder = healthy_stream(4)
        stamp = 10**9+4*10**8
        recorder.published[stamp] = 4           # published but never reached the gate
        recorder.index += 1
        for kind in KINDS:
            recorder.content[(kind, stamp)] = message(kind, stamp)
        causes, _ = self.causes(recorder.run())
        self.assertEqual(causes[stamp][0], 'missing-acquisition')

    def test_other_when_acquisition_is_rejected(self):
        recorder = healthy_stream(4)
        stamp = 10**9+25*10**7                  # <= last_input (1.3e9): acquisition rejected
        recorder.published[stamp] = 4
        recorder.index += 1
        for kind in KINDS:
            recorder.content[(kind, stamp)] = message(kind, stamp)
        recorder.gate.acquisition(stamp, 100.8, 10**9+3*10**8)
        recorder._record('acquisition', 100.8, 10**9+3*10**8, stamp)
        causes, _ = self.causes(recorder.run())
        self.assertEqual(causes[stamp],
                         ('other', 'acquisition rejected (stamp never entered pending)'))

    def test_every_frame_gets_exactly_one_cause(self):
        recorder = healthy_stream(4)
        recorder.publish(10**9+4*10**8, 100.8, flaw='no_match')
        recorder.publish(10**9+5*10**8, 102.5)
        run = recorder.run()
        causes, _ = self.causes(run)
        self.assertEqual(set(causes), set(run.published))
        for value in causes.values():
            self.assertIn(value[0], drift.CAUSES)


class KnownAnswerTest(unittest.TestCase):
    def test_faithful_replay_reproduces_the_recording(self):
        run = healthy_stream(6).run()
        faithful, verdict = drift.known_answer(run, PRIOR)
        self.assertEqual(verdict['verdict'], 'PASS')
        self.assertEqual(faithful['approved'], run.recorded_approved())
        self.assertEqual(verdict['event_mismatches'], 0)

    def test_corrupted_recording_fails_the_known_answer(self):
        run = healthy_stream(6).run()
        broken = [dict(row) for row in run.events]
        for row in broken:
            if row.get('approved'):
                row['approved'] = []
                break
        faithful, verdict = drift.known_answer(
            drift.Run('broken', broken, run.content, run.published), PRIOR)
        self.assertEqual(verdict['verdict'], 'FAIL')
        self.assertFalse(verdict['approved_list_identical'])

    def test_missing_message_content_is_loud(self):
        run = healthy_stream(3).run()
        content = {k: v for k, v in run.content.items() if k[0] != 'info'}
        with self.assertRaises(KeyError):
            drift.replay(drift.Run('x', run.events, content, run.published), PRIOR)


class IdealizedClockTest(unittest.TestCase):
    """The discriminating experiment: identical content, different wall starvation."""

    def _pair(self):
        fast = Recorder()
        slow = Recorder()
        for i in range(8):
            stamp = 10**9+i*10**8
            fast.publish(stamp, 100.+i*.2)
            # the same content, but frame 5 is starved past the 1.0 s stream timeout
            slow.publish(stamp, 100.+i*.2+(2. if i >= 5 else 0.))
        return fast.run('fast'), slow.run('slow')

    def test_wall_starvation_only_differs_under_the_real_clock(self):
        fast, slow = self._pair()
        self.assertEqual(fast.content.keys(), slow.content.keys())
        for key in fast.content:
            self.assertEqual(fast.content[key], slow.content[key])
        real_fast = set(drift.replay(fast, PRIOR, 'real')['approved'])
        real_slow = set(drift.replay(slow, PRIOR, 'real')['approved'])
        self.assertNotEqual(real_fast, real_slow)
        for mode in ('ros', 'tiny'):
            self.assertEqual(set(drift.replay(fast, PRIOR, mode)['approved']),
                             set(drift.replay(slow, PRIOR, mode)['approved']), mode)

    def test_content_difference_survives_the_idealized_clock(self):
        clean = healthy_stream(8).run('clean')
        dirty_recorder = Recorder()
        for i in range(8):
            dirty_recorder.publish(10**9+i*10**8, 100.+i*.2, flaw='no_match' if i == 5 else None)
        dirty = dirty_recorder.run('dirty')
        for mode in ('ros', 'tiny'):
            self.assertNotEqual(set(drift.replay(clean, PRIOR, mode)['approved']),
                                set(drift.replay(dirty, PRIOR, mode)['approved']), mode)

    def test_idealized_clock_never_removes_an_approval(self):
        _, slow = self._pair()
        real = set(drift.replay(slow, PRIOR, 'real')['approved'])
        ideal = set(drift.replay(slow, PRIOR, 'ros')['approved'])
        self.assertEqual(real-ideal, set())
        self.assertTrue(ideal-real)


class ContentFieldTest(unittest.TestCase):
    def test_gate_relevant_fields(self):
        run = healthy_stream(2).run()
        stamp = sorted(run.published)[0]
        fields = drift.content_fields(run, stamp, PRIOR)
        self.assertTrue(fields['link_in_prior_map'])
        self.assertTrue(fields['localization_pose_covariance_valid'])
        self.assertTrue(fields['odom_covariance_valid'])
        self.assertIs(fields['odometry_lost'], False)
        self.assertEqual(fields['loop_closure_id'], 2)

    def test_message_reason_matches_the_gate(self):
        recorder = healthy_stream(1)
        recorder.publish(10**9+10**8, 100.2, flaw='no_match')
        run = recorder.run()
        self.assertIsNone(run.message_reason(10**9, PRIOR))
        self.assertEqual(run.message_reason(10**9+10**8, PRIOR), 'info:no_prior_map_match')

    def test_absent_content_is_reported(self):
        run = healthy_stream(1).run()
        run.content.pop(('info', 10**9))
        self.assertEqual(run.message_reason(10**9, PRIOR), 'info:absent')


class SummaryTest(unittest.TestCase):
    def runs(self):
        clean = healthy_stream(8).run('clean')
        recorder = Recorder()
        for i in range(8):
            recorder.publish(10**9+i*10**8, 100.+i*.2, flaw='no_match' if i == 5 else None)
        return [clean, recorder.run('dirty')]

    def test_summary_shape_and_pair_partition(self):
        rows, summary, ideal = drift.analyse(self.runs(), PRIOR)
        self.assertEqual(summary['runs'], ['clean', 'dirty'])
        self.assertEqual(len(rows), 16)
        pair = summary['pairs']['clean|dirty']
        self.assertGreater(pair['symmetric_difference'], 0)
        self.assertEqual(sum(pair['only_clean'].values())+sum(pair['only_dirty'].values()),
                         pair['symmetric_difference'])
        self.assertIn('content-bad', pair['only_clean'])
        self.assertEqual(summary['content_identity']['frames'], 8)
        self.assertEqual(summary['content_identity']['identical_across_all_runs'], 7)
        for label in summary['runs']:
            counts = summary['per_run'][label]['cause_counts']
            self.assertEqual(sum(counts.values()), 8)
            self.assertEqual(summary['per_run'][label]['known_answer']['verdict'], 'PASS')
        self.assertEqual(set(ideal['clean']), {'ros', 'tiny'})

    def test_write_outputs(self):
        rows, summary, ideal = drift.analyse(self.runs(), PRIOR)
        with tempfile.TemporaryDirectory() as tmp:
            out = drift.write_outputs(Path(tmp)/'nested', rows, summary, ideal, ('ros', 'tiny'))
            for name in ('per-frame.csv', 'per-frame.json', 'drift-summary.json',
                         'idealized-replay.json'):
                self.assertTrue((out/name).exists(), name)
            text = (out/'per-frame.csv').read_text().splitlines()
            self.assertEqual(len(text), len(rows)+1)
            self.assertIn('approved_ideal_ros', text[0])
            self.assertEqual(json.loads((out/'drift-summary.json').read_text())['runs'],
                             ['clean', 'dirty'])


class PriorDatabaseTest(unittest.TestCase):
    def test_prior_ids_from_db(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'map.db'
            with sqlite3.connect(path) as db:
                db.execute('CREATE TABLE Node (id INTEGER)')
                db.executemany('INSERT INTO Node VALUES (?)', [(3,), (1,), (2,)])
            self.assertEqual(drift.prior_ids_from_db(path), [1, 2, 3])


class RevokeClassTest(unittest.TestCase):
    def test_reason_classes(self):
        self.assertEqual(drift.content_class('localization_stream_timeout'), 'WALL')
        self.assertEqual(drift.content_class('incomplete_acquisition_timeout'), 'WALL')
        self.assertEqual(drift.content_class('confirming_healthy_acquisitions'), 'WALL')
        self.assertEqual(drift.content_class('info:no_prior_map_match'), 'CONTENT')
        self.assertEqual(drift.content_class('odom:invalid_covariance'), 'CONTENT')
        self.assertEqual(drift.content_class('unhealthy_acquisition'), 'CONTENT')
        self.assertEqual(drift.content_class('unexpected_or_retired_stamp'), 'INHERIT')


if __name__ == '__main__':
    unittest.main()
