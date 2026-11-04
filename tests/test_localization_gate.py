"""Policy fault injection, independent of ROS and offline truth scoring."""
import copy
import inspect
import json
from pathlib import Path
import re
import tempfile
import unittest

from host.localization_gate import (CallbackClock, LocalizationGate, LocalizationLease, KINDS,
                                    event_row)
import host.ros_localization_gate as node
from host.ros_localization_gate import navigation_ready


def observation(kind, stamp):
    pose = {'position': dict(x=1., y=2., z=3.), 'orientation': dict(x=0., y=0., z=0., w=1.)}
    cov = [float(i % 7 == 0)*.01 for i in range(36)]
    return {'header': {'stamp': dict(sec=stamp//10**9, nanosec=stamp % 10**9),
                       'frame_id': 'odom' if kind in KINDS[:2] else 'map'},
            'child_frame_id': 'camera_link', 'pose': {'pose': pose, 'covariance': cov},
            'covariance': cov, 'lost': False, 'loop_closure_id': 1, 'proximity_detection_id': 0}


def frame(gate, i, wall=None):
    stamp = 10**9 + i*10**8
    wall = i*.1 if wall is None else wall
    gate.acquisition(stamp, wall, stamp)
    outputs = []
    for kind in KINDS:
        outputs += gate.ingest(kind, observation(kind, stamp), wall, stamp)
    return outputs


class GateTests(unittest.TestCase):
    def gate(self):
        return LocalizationGate('map-hash', {1, 2})

    def healthy(self):
        gate = self.gate()
        self.assertFalse(frame(gate, 1))
        self.assertFalse(frame(gate, 2))
        self.assertEqual(len(frame(gate, 3)), 1)
        return gate

    def test_three_distinct_complete_and_no_retroactive_approval(self):
        gate = self.healthy()
        self.assertEqual(list(gate.approved), [1300000000])
        self.assertEqual(len(frame(gate, 4)), 1)

    def test_immediate_rejection_and_three_frame_recovery(self):
        for kind, change in [
            ('odom_info', lambda r: r.update(lost=True)),
            ('info', lambda r: r.update(loop_closure_id=999)),
            ('odom', lambda r: r.update(child_frame_id='wrong')),
            ('localization_pose', lambda r: r['header'].update(frame_id='odom')),
            ('localization_pose', lambda r: r['pose']['pose']['position'].update(x=float('nan'))),
            ('localization_pose', lambda r: r['pose']['pose']['orientation'].update(w=0.)),
            ('odom', lambda r: r['pose']['covariance'].__setitem__(0, 9999.)),
            ('odom_info', lambda r: r['covariance'].__setitem__(0, -1.)),
            ('odom_info', lambda r: r['covariance'].__setitem__(1, 1.)),
        ]:
            with self.subTest(kind=kind):
                gate = self.healthy()
                s = 1400000000
                gate.acquisition(s, .4, s)
                row = observation(kind, s)
                change(row)
                self.assertFalse(gate.ingest(kind, row, .4, s))
                self.assertEqual(gate.state, 'lost')
                self.assertFalse(gate.approved)
                # Complete the rejected frame; it cannot contribute a healthy count.
                for other in KINDS:
                    if other != kind:
                        gate.ingest(other, observation(other, s), .4, s)
                self.assertFalse(frame(gate, 5))
                self.assertFalse(frame(gate, 6))
                self.assertEqual(len(frame(gate, 7)), 1)

    def test_duplicate_reordered_unknown_stamp_and_missing_stream(self):
        gate = self.healthy()
        gate.ingest('odom', observation('odom', 1300000000), .31, 1300000000)
        self.assertNotEqual(gate.state, 'valid')
        gate = self.healthy()
        gate.acquisition(1400000000, .4, 1400000000)
        gate.ingest('odom', observation('odom', 1400000000), .41, 1400000000)
        gate.tick(.901, 1400000000)
        self.assertEqual(gate.state, 'stale')
        self.assertFalse(gate.pending)

    def test_clock_pause_rollback_and_future(self):
        gate = self.healthy()
        old = gate.instance
        gate.tick(1.301, 1300000000)
        self.assertEqual(gate.state, 'stale')
        gate.tick(1.4, 1000000000)
        self.assertEqual(gate.state, 'initializing')
        self.assertNotEqual(gate.instance, old)
        gate.acquisition(1200000000, 1.5, 1100000000)
        self.assertEqual(gate.state, 'lost')

    def test_out_of_order_streams_are_joined_not_frames_skipped(self):
        gate = self.gate()
        for i in (1, 2):
            s = 10**9+i*10**8
            gate.acquisition(s, i*.1, s)
        for kind in reversed(KINDS):
            gate.ingest(kind, observation(kind, 1200000000), .21, 1200000000)
        self.assertEqual(gate.streak, 0)
        for kind in reversed(KINDS):
            gate.ingest(kind, observation(kind, 1100000000), .22, 1200000000)
        self.assertEqual(gate.streak, 2)
        self.assertEqual(len(frame(gate, 3)), 1)

    def test_newer_lost_prevents_older_completion_rearming(self):
        gate = self.healthy()
        for i in (4, 5):
            s = 10**9+i*10**8
            gate.acquisition(s, i*.1, s)
        bad = observation('odom_info', 1500000000)
        bad['lost'] = True
        gate.ingest('odom_info', bad, .51, 1500000000)
        for kind in KINDS:
            self.assertFalse(gate.ingest(kind, observation(kind, 1400000000), .52, 1500000000))
        self.assertEqual(gate.state, 'lost')

    def test_consumer_epoch_late_result_and_heartbeat_expiry(self):
        gate = self.healthy()
        lease = LocalizationLease('map-hash')
        lease.update(gate.status(.31, 1300000000), .31, 1300000000)
        token = lease.token(1300000000, .32, 1300000000)
        self.assertTrue(lease.permits(token, .33, 1300000000))
        gate.revoke('lost', 'injected')
        lease.update(gate.status(.34, 1300000000), .34, 1300000000)
        self.assertFalse(lease.permits(token, .35, 1300000000))
        for i in (4, 5, 6):
            frame(gate, i)
        lease.update(gate.status(.61, 1600000000), .61, 1600000000)
        self.assertTrue(lease.check(.62, 1600000000))
        self.assertFalse(lease.permits(token, .62, 1600000000))
        self.assertFalse(lease.check(1.12, 1600000000))

    def test_fresh_heartbeats_cannot_extend_frozen_pose_after_timeout(self):
        gate = self.healthy()
        lease = LocalizationLease('map-hash')
        row = gate.status(.31, 1300000000)
        for i in range(15):
            r = copy.deepcopy(row)
            r['sequence'] += i
            lease.update(r, .31+i*.1, 1300000000)
            self.assertEqual(lease.check(.31+i*.1, 1300000000), i < 10)

    def test_restart_map_mismatch_reordered_status_and_clock_reset(self):
        gate = self.healthy()
        lease = LocalizationLease('map-hash')
        row = gate.status(.31, 1300000000)
        lease.update(row, .31, 1300000000)
        token = lease.token(1300000000, .32, 1300000000)
        lease.update(row, .33, 1300000000)
        self.assertFalse(lease.check(.34, 1300000000))
        row['sequence'] += 1
        row['map_id'] = 'another-map'
        lease.update(row, .35, 1300000000)
        self.assertFalse(lease.permits(token, .36, 1300000000))
        row = gate.status(.37, 1300000000)
        lease.update(row, .37, 1300000000)
        self.assertFalse(lease.permits(token, .38, 1300000000))
        self.assertFalse(lease.check(.39, 1000000000))
        row['sequence'] += 1
        lease.update(row, .4, 1300000000)
        self.assertFalse(lease.check(.41, 1300000000))


class FurtherFaultTests(unittest.TestCase):
    def test_confirmation_cannot_span_an_unobserved_long_gap(self):
        gate = LocalizationGate('map-hash', {1})
        frame(gate, 1)
        gate.tick(1.101, 1100000000)
        self.assertEqual(gate.streak, 0)
        self.assertFalse(frame(gate, 2, 1.2))
        self.assertFalse(frame(gate, 3, 1.3))
        self.assertEqual(len(frame(gate, 4, 1.4)), 1)

    def test_complete_frame_never_borrows_other_stamp_health(self):
        gate = LocalizationGate('map-hash', {1})
        for i in (1, 2, 3):
            s=10**9+i*10**8
            gate.acquisition(s, i*.1, s)
            for kind in KINDS[:-1]:
                self.assertFalse(gate.ingest(kind, observation(kind,s), i*.1, s))
        self.assertFalse(gate.ingest('localization_pose',observation('localization_pose',1400000000),.4,1400000000))
        self.assertFalse(gate.approved)

    def test_pending_and_approval_memory_bounded(self):
        gate = LocalizationGate('map-hash', {1})
        for i in range(100):
            gate.acquisition(10**9+i, i*.001, 10**9+i)
            self.assertLessEqual(len(gate.pending),32)
        gate = LocalizationGate('map-hash', {1})
        for i in range(1,100):
            frame(gate,i)
            self.assertLessEqual(len(gate.approved),32)

    def test_increasing_heartbeat_sequence_cannot_hide_stamp_rollback(self):
        gate = LocalizationGate('map-hash', {1})
        for i in (1,2,3): frame(gate,i)
        lease=LocalizationLease('map-hash'); row=gate.status(.31,1300000000)
        lease.update(row,.31,1300000000)
        self.assertTrue(lease.check(.32,1300000000))
        row['sequence']+=1
        row['stamp_ns']-=100000000
        row['approved'][0]['stamp_ns']=row['stamp_ns']
        lease.update(row,.33,1300000000)
        self.assertFalse(lease.check(.34,1300000000))

    def test_new_instance_retires_old_status_and_inflight_token(self):
        gate = LocalizationGate('map-hash', {1})
        for i in (1,2,3): frame(gate,i)
        lease=LocalizationLease('map-hash');old=gate.status(.31,1300000000)
        lease.update(old,.31,1300000000);token=lease.token(1300000000,.32,1300000000)
        new=copy.deepcopy(old);new['instance']='replacement';new['sequence']=1
        lease.update(new,.33,1300000000)
        self.assertTrue(lease.check(.34,1300000000))
        self.assertFalse(lease.permits(token,.34,1300000000))
        old['sequence']+=100
        lease.update(old,.35,1300000000)
        self.assertFalse(lease.check(.36,1300000000))


class DeclaredProvenanceTests(unittest.TestCase):
    """The added status fields are declarations only; they never change a gate decision."""

    def row(self, **kwargs):
        gate = LocalizationGate('map-hash', {1}, **kwargs)
        for i in (1, 2, 3):
            frame(gate, i)
        return gate.status(.31, 1300000000)

    def test_status_adds_provenance_fields_without_changing_schema(self):
        row = self.row()
        self.assertEqual(row['schema'], 1)
        self.assertEqual(row['pose_source'], 'estimated')
        self.assertEqual(row['odom_source'], 'rtabmap_rgbd_odometry')
        self.assertIs(row['navigation_ready'], False)
        self.assertEqual(row['state'], 'valid')
        self.assertIs(json.loads(json.dumps(row, allow_nan=False))['navigation_ready'], False)

    def test_declared_fields_come_from_the_constructor(self):
        row = self.row(pose_source='surveyed', navigation_ready=True, odom_source='wheel_encoder')
        self.assertEqual((row['pose_source'], row['odom_source']), ('surveyed', 'wheel_encoder'))
        self.assertIs(row['navigation_ready'], True)
        with self.assertRaises(ValueError):
            LocalizationGate('map-hash', {1}, pose_source='')
        with self.assertRaises(ValueError):
            LocalizationGate('map-hash', {1}, odom_source='')

    def test_readiness_is_fail_closed_for_non_boolean_claims(self):
        for claim in ('yes', 1, [1], object()):
            self.assertIs(self.row(navigation_ready=claim)['navigation_ready'], False)

    def test_lease_accepts_the_new_fields_and_tolerates_unknown_keys(self):
        row = self.row()
        row['experimental_key'] = {'anything': [1, 2, 3]}
        lease = LocalizationLease('map-hash')
        lease.update(row, .31, 1300000000)
        self.assertTrue(lease.check(.32, 1300000000))
        # Discriminating control: pinned identity is still enforced on the same row shape.
        rejected = self.row()
        rejected['experimental_key'] = {'anything': [1, 2, 3]}
        rejected['world_frame'] = 'other_map'
        other = LocalizationLease('map-hash')
        other.update(rejected, .31, 1300000000)
        self.assertFalse(other.check(.32, 1300000000))


class BundleReadinessTests(unittest.TestCase):
    """A map bundle that is absent, unreadable or silent about readiness is not ready."""

    def test_navigation_ready_reads_only_an_explicit_true(self):
        self.assertIs(navigation_ready(None), False)
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory)
            self.assertIs(navigation_ready(bundle/'absent'), False)
            self.assertIs(navigation_ready(bundle), False)
            for text, expected in (('{"navigation_ready": false}', False), ('{"other": 1}', False),
                                   ('not json', False), ('[]', False),
                                   ('{"navigation_ready": "true"}', False),
                                   ('{"navigation_ready": true}', True)):
                (bundle/'bundle.json').write_text(text)
                self.assertIs(navigation_ready(bundle), expected, text)
                self.assertIs(navigation_ready(str(bundle)), expected, text)


class CallbackClockTests(unittest.TestCase):
    """The one clock reading a callback takes, in both units, from one source."""

    def test_the_seconds_the_gate_sees_are_the_nanoseconds_that_get_recorded(self):
        clock = CallbackClock(5_998_800_000, 1_300_000_000)
        self.assertEqual(clock.wall, 5_998_800_000 / 1e9)
        self.assertEqual(event_row('acquisition', LocalizationGate('m', {1}), clock)['monotonic_ns'],
                         clock.monotonic_ns)

    def test_it_unpacks_as_the_pair_every_gate_entry_point_takes(self):
        clock = CallbackClock(7 * 10**9, 42)
        self.assertEqual(tuple(clock), (7., 42))
        gate = LocalizationGate('map-hash', {1, 2})
        gate.tick(*clock)                      # the call shape the node uses
        self.assertEqual(gate.last_wall, clock.wall)
        self.assertEqual(gate.last_ros, clock.ros_ns)

    def test_non_integer_or_negative_clocks_are_rejected(self):
        for bad in ((1.5, 1), (1, 1.5), (1, -1), (None, 1), (1, None)):
            with self.assertRaises(ValueError):
                CallbackClock(*bad)

    def test_the_recorded_row_keeps_the_schema_and_takes_both_stamps_from_the_clock(self):
        gate = LocalizationGate('map-hash', {1, 2})
        frame(gate, 1)
        clock = CallbackClock(11 * 10**9, 2 * 10**9)
        row = event_row('received_odom', gate, clock, stamp_ns=1234, approved=[])
        self.assertEqual(list(row), ['event', 'monotonic_ns', 'ros_ns', 'state', 'reason',
                                     'epoch', 'map_id', 'instance', 'pending_count',
                                     'approved_count', 'stamp_ns', 'approved'])
        self.assertEqual((row['monotonic_ns'], row['ros_ns']), (clock.monotonic_ns, clock.ros_ns))
        self.assertEqual((row['state'], row['reason'], row['epoch']),
                         (gate.state, gate.reason, gate.epoch))
        self.assertEqual((row['pending_count'], row['approved_count']),
                         (len(gate.pending), len(gate.approved)))

    def test_a_row_cannot_be_written_from_a_raw_timestamp(self):
        # The whole point is that the row cannot be stamped from a second, fresher read.
        with self.assertRaises(ValueError):
            event_row('acquisition', LocalizationGate('m', {1}), 11 * 10**9)


class RecordedClockReplayTests(unittest.TestCase):
    """Known answer: replaying the recorded row must give the decision that was recorded.

    This is the 2026-09-15 drift-analysis failure (`REPORT.md` §3.1) in miniature. An
    ingest 0.9988 s after the last healthy completion is inside the 1.0 s stream timeout,
    but a row stamped after a 2.3 ms callback body reads as 1.0011 s, so an offline replay
    driven by `monotonic_ns/1e9` revokes where the node did not. Both arms are asserted:
    without the negative arm this test would pass on any wiring at all.
    """

    LAST_GOOD_NS = 5 * 10**9
    ROS_NS = 1_300_000_000

    def valid_gate(self):
        gate = LocalizationGate('map-hash', {1, 2})
        for index in (1, 2, 3):
            frame(gate, index, wall=self.LAST_GOOD_NS / 1e9)
        self.assertEqual(gate.state, 'valid')
        return gate

    def replayed_state(self, row):
        gate = self.valid_gate()
        gate.status(row['monotonic_ns'] / 1e9, row['ros_ns'])
        return gate.state

    def test_the_entry_reading_replays_to_the_recorded_state_and_a_later_one_does_not(self):
        gate = self.valid_gate()
        entry = CallbackClock(self.LAST_GOOD_NS + 998_800_000, self.ROS_NS)
        self.assertEqual(gate.status(*entry)['state'], 'valid')
        self.assertEqual(self.replayed_state(event_row('state_transition', gate, entry)), 'valid')
        # Negative control: the pre-fix wiring, which read the clock again to write the row.
        after_body = CallbackClock(entry.monotonic_ns + 2_300_000, entry.ros_ns)
        self.assertEqual(self.replayed_state(event_row('state_transition', gate, after_body)),
                         'stale')


class NodeEventClockWiringTests(unittest.TestCase):
    """No ROS here: pin by source that one reading drives the decision AND the row.

    Style follows tests/test_map_odom_hold.py::NodeWiringTests. What is pinned is the
    invariant an offline replay depends on — every callback reads the steady clock once,
    at entry, and hands that same value to the gate call, to the event row and to the
    trailing heartbeat.
    """

    source = inspect.getsource(node.main)

    def body(self, name):
        match = re.search(r'\n        def %s\(.*?(?=\n        def |\n    rclpy\.init)' % name,
                          self.source, re.S)
        self.assertIsNotNone(match, 'no GateNode.%s in the node source' % name)
        return match.group(0)

    def test_the_steady_clock_is_read_only_where_it_is_meant_to_be(self):
        # Three sites and no more: the per-callback reading, and the two diagnostics-only
        # stage probes (the before/after snapshot and the message-conversion timestamp).
        self.assertEqual(len(re.findall(r'time\.monotonic', self.source)), 3, self.source)
        self.assertIn('return CallbackClock(time.monotonic_ns(), '
                      'self.get_clock().now().nanoseconds)', self.body('clocks'))

    def test_the_row_is_built_by_the_tested_helper_from_the_callback_clock(self):
        record = self.body('record')
        self.assertIn('def record(self, event, clock, **fields):', record)
        self.assertIn('event_row(event, self.gate, clock, **fields)', record)
        self.assertNotIn('time.', record)
        self.assertNotIn('self.get_clock()', record)

    def test_the_acquisition_callback_feeds_one_reading_to_gate_row_and_heartbeat(self):
        acquisition = self.body('acquisition')
        self.assertEqual(acquisition.count('self.clocks()'), 1, acquisition)
        self.assertIn('clock = self.clocks()', acquisition)
        self.assertIn('self.gate.acquisition(s.sec*10**9+s.nanosec, *clock)', acquisition)
        self.assertRegex(acquisition, r"self\.record\('acquisition', clock,")
        self.assertIn('self.heartbeat(clock)', acquisition)

    def test_the_receive_callback_feeds_one_reading_to_gate_row_and_heartbeat(self):
        receive = self.body('receive')
        self.assertIn('clock = self.clocks()', receive)
        self.assertIn('self.gate.ingest(kind, converted, *clock)', receive)
        self.assertRegex(receive, r"self\.record\('received_'\+kind, clock,")
        self.assertIn('self.heartbeat(clock)', receive)
        # The only second reading in this callback is the diagnostics-only stage marker,
        # which exists to bracket the publications and carries no gate decision.
        self.assertEqual(receive.count('self.clocks()'), 2, receive)
        self.assertIn("self.record('callback_output_complete',self.clocks(),", receive)
        self.assertRegex(receive, r"if args\.diagnostics:\n(?:\s*#[^\n]*\n)*"
                                  r"\s*self\.record\('callback_output_complete'")

    def test_the_heartbeat_reuses_the_callers_reading_and_takes_its_own_only_as_a_timer(self):
        heartbeat = self.body('heartbeat')
        self.assertIn('def heartbeat(self, clock=None):', heartbeat)
        self.assertIn('clock = self.clocks() if clock is None else clock', heartbeat)
        self.assertEqual(heartbeat.count('self.clocks()'), 1, heartbeat)
        self.assertIn('self.gate.status(*clock)', heartbeat)
        self.assertRegex(heartbeat, r"self\.record\('state_transition', clock,")
        self.assertIn('self.create_timer(.1, self.heartbeat,', self.source)

    def test_no_callback_hands_the_gate_a_reading_it_does_not_also_record(self):
        # Negative control for the defect itself: a bare self.clocks() splatted straight
        # into a gate call leaves nothing for the row to be stamped with.
        self.assertNotRegex(self.source, r'self\.gate\.\w+\([^)]*\*self\.clocks\(\)')
        self.assertNotIn('self.gate.status(*self.clocks())', self.source)


if __name__ == '__main__':
    unittest.main()
