"""The map->odom correction an approval carries, and what the gate still refuses to decide.

Honest scope: `pose` (map->camera) and `odom_pose` (odom->camera) were already in every
approval and every status row before this change, so a consumer could always have composed
this transform itself. What is pinned here is the CONTRACT and its validation - who
composes it, at which acquisition it was observed, and who fails closed on a corrupt one -
not information the gate used to hide. The other half of what is pinned is what the gate
still does not do: no threshold, no correction counter or version, no epoch advance for a
correction. Those decisions stay with the consumer.

The strongest equivalence evidence needs the pinned 8f090d4 module on disk and therefore
lives outside the repository (2026-09-21: the recorded observations.jsonl replayed through
both versions, 11097 and 10013 scalar leaves compared, 0 differences, with a deliberately
shifted gate as the comparator control). `RecordedRegressionTests` below carries the same
claim inside the suite using the frozen recorded approvals, which are genuine pre-change
output rather than something this file computes.
"""
import copy
import itertools
import json
from pathlib import Path
import re
import unittest

import numpy as np

from host.localization_gate import (KINDS, LocalizationGate, LocalizationLease, RIGID_ATOL,
                                    pose_matrix, rigid_valid)

MAP_ID = 'map-hash'
PRIOR_ID = 1
COV = [float(i % 7 == 0) * .01 for i in range(36)]
IDENTITY_Q = (0., 0., 0., 1.)
# A rotation that is not the identity in any axis, normalised exactly once here.
TILTED_Q = tuple((np.array([.1, .2, .3, .9]) / np.linalg.norm([.1, .2, .3, .9])).tolist())

RECORDING = Path('${FPGA_SRC}/runs/rtabmap-nav2-rehearsal-20260914')

# The keys 8f090d4 produced. Written as literals on purpose: a future change that renames or
# drops one of them must fail here rather than quietly break an unmodified consumer copy.
OLD_STATUS_KEYS = {'schema', 'instance', 'map_id', 'epoch', 'sequence', 'state', 'reason',
                   'stamp_ns', 'clock_ns', 'world_frame', 'tracked_frame', 'pose_source',
                   'navigation_ready', 'odom_source', 'approved'}
OLD_APPROVAL_KEYS = {'stamp_ns', 'pose', 'odom_pose'}
NEW_STATUS_KEYS = {'correction_stamp_ns'}
NEW_APPROVAL_KEYS = {'map_odom'}

# The gate must not grow a correction version, counter or epoch of its own: a threshold on
# this quantity is the consumer's to choose, and no measurement of a real loop closure jump
# exists to set one from (2026-09-21 review).
FORBIDDEN_FIELD = re.compile(r'correction_(epoch|count|counter|version|id|seq|sequence|'
                             r'generation|revision)$|map_version|map_revision')


def pose(x, y=0., z=0., orientation=IDENTITY_Q):
    return {'position': {'x': float(x), 'y': float(y), 'z': float(z)},
            'orientation': dict(zip(('x', 'y', 'z', 'w'), [float(v) for v in orientation]))}


def observation(kind, stamp, odom_x, map_x, orientation=IDENTITY_Q, lost=False):
    """One healthy message. The camera sits at `odom_x` in odom and `map_x` in map."""
    body = pose(odom_x if kind in KINDS[:2] else map_x, orientation=orientation)
    return {'header': {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9},
                       'frame_id': 'odom' if kind in KINDS[:2] else 'map'},
            'child_frame_id': 'camera_link', 'pose': {'pose': body, 'covariance': list(COV)},
            'covariance': list(COV), 'lost': lost, 'loop_closure_id': PRIOR_ID,
            'proximity_detection_id': 0}


def acquisition(gate, index, odom_x, correction_x, orientation=IDENTITY_Q, lost=False):
    """Drive one complete healthy acquisition. Returns (approvals, wall, stamp)."""
    stamp = 10**9 + index * 10**8
    wall = index * .1
    gate.acquisition(stamp, wall, stamp)
    outputs = []
    for kind in KINDS:
        row = observation(kind, stamp, odom_x, odom_x + correction_x, orientation,
                          lost=(lost and kind == 'odom_info'))
        outputs += gate.ingest(kind, row, wall, stamp)
    return outputs, wall, stamp


def gate():
    return LocalizationGate(MAP_ID, {PRIOR_ID})


def translation(matrix):
    return np.asarray(matrix, dtype=float)[:3, 3]


class ConstantCorrectionTests(unittest.TestCase):
    """Negative control: a correction that never moves must not look like one that did."""

    def test_a_hundred_approvals_under_a_constant_correction_are_byte_identical(self):
        # 1/1024 m steps and an identity rotation are exactly representable, so the
        # composition is exact and byte equality is the right criterion here.
        instance, approvals = gate(), []
        for index in range(1, 111):
            outputs, _, _ = acquisition(instance, index, index / 1024., 3.5)
            approvals += outputs
        self.assertGreaterEqual(len(approvals), 100)
        # Positive control inside the negative control: the camera really did move, so a
        # gate that echoed its odom pose instead of the correction would fail this test.
        moved = {json.dumps(item['odom_pose']['position']) for item in approvals}
        self.assertEqual(len(moved), len(approvals))
        first = json.dumps(approvals[0]['map_odom'])
        self.assertEqual([json.dumps(item['map_odom']) for item in approvals],
                         [first] * len(approvals))
        self.assertEqual(json.loads(first)[0][3], 3.5)

    def test_an_unrepresentable_constant_correction_moves_only_by_float64_noise(self):
        # The composition goes through a 4x4 inverse, so with 0.1 m steps and a tilted frame
        # consecutive readings are NOT bit-identical. Stated, not hidden: a consumer may not
        # read bitwise inequality as a correction. The residual is ~1e-15 m against the
        # 0.0207 m of real intra-epoch motion this contract exists to expose.
        instance, approvals = gate(), []
        for index in range(1, 111):
            outputs, _, _ = acquisition(instance, index, index * .1, 3.5, TILTED_Q)
            approvals += outputs
        self.assertGreaterEqual(len(approvals), 100)
        encoded = [json.dumps(item['map_odom']) for item in approvals]
        self.assertGreater(len(set(encoded)), 1, 'expected float noise, got exact equality')
        worst = max(float(np.abs(np.asarray(item['map_odom'], dtype=float)
                                 - np.asarray(approvals[0]['map_odom'], dtype=float)).max())
                    for item in approvals)
        # Bound from the measured scale of the quantity, not from taste: this fixture reads
        # 1.8e-15 (2026-09-21), a picometre leaves it ~560x of room, and no correction worth
        # a consumer's attention hides under a picometre.
        self.assertLess(worst, 1e-12)
        self.assertLess(worst, .0207 / 1e9)


class InjectedCorrectionTests(unittest.TestCase):
    """Positive arm: a 1.0 m correction the consumer can now read, and still must judge."""

    def stream(self, corrections):
        instance, lease = gate(), LocalizationLease(MAP_ID)
        seen = []
        for index, correction in enumerate(corrections, start=1):
            _, wall, stamp = acquisition(instance, index, index * .1, correction)
            row = instance.status(wall, stamp)
            lease.update(row, wall, stamp)
            seen.append((row, wall, stamp))
        return instance, lease, seen

    def test_a_one_metre_correction_is_readable_through_the_lease(self):
        instance, lease, seen = self.stream([0.] * 6 + [1.] * 4)
        row, wall, ros_ns = seen[-1]
        self.assertTrue(lease.check(wall, ros_ns))
        readings = [(item['stamp_ns'], translation(lease.correction(item['stamp_ns'], wall, ros_ns))[0])
                    for item in row['approved']]
        self.assertEqual([stamp for stamp, _ in readings], sorted(stamp for stamp, _ in readings))
        steps = [abs(b - a) for (_, a), (_, b) in zip(readings, readings[1:])]
        self.assertAlmostEqual(max(steps), 1.0, places=9)
        self.assertAlmostEqual(min(steps), 0.0, places=12)
        # The ordering anchor names the newest reading, so a consumer can tell which
        # correction is the current one without scanning the list.
        self.assertEqual(row['correction_stamp_ns'], max(stamp for stamp, _ in readings))
        self.assertEqual(row['correction_stamp_ns'], row['stamp_ns'])
        self.assertEqual([r['correction_stamp_ns'] for r, _, _ in seen if r['state'] == 'valid'],
                         sorted(r['correction_stamp_ns'] for r, _, _ in seen if r['state'] == 'valid'))

    def test_the_gate_reaches_no_verdict_about_the_correction(self):
        instance, lease, seen = self.stream([0.] * 6 + [1.] * 4)
        row = seen[-1][0]
        self.assertEqual(instance.epoch, 0)
        self.assertEqual({r['epoch'] for r, _, _ in seen}, {0})
        self.assertEqual({r['reason'] for r, _, _ in seen if r['state'] == 'valid'},
                         {'three_healthy_acquisitions'})
        for name in list(row) + list(vars(instance)):
            self.assertNotRegex(name, FORBIDDEN_FIELD)

    def test_an_unmoved_correction_reads_as_unmoved_on_the_same_code_path(self):
        # Paired with the arm above: identical message shape, correction constant, so the
        # 1.0 m reading cannot be an artefact of the fixture.
        _, lease, seen = self.stream([0.] * 10)
        row, wall, ros_ns = seen[-1]
        readings = [translation(lease.correction(item['stamp_ns'], wall, ros_ns))[0]
                    for item in row['approved']]
        self.assertEqual(readings, [0.] * len(readings))


class InstrumentControlTests(unittest.TestCase):
    """Proves the fixture can see a change, so the null above is the gate's and not its own."""

    def test_a_real_revocation_still_advances_the_epoch_and_drops_the_lease(self):
        instance, lease = gate(), LocalizationLease(MAP_ID)
        for index in range(1, 6):
            _, wall, stamp = acquisition(instance, index, index * .1, 0.)
            lease.update(instance.status(wall, stamp), wall, stamp)
        self.assertEqual(instance.state, 'valid')
        self.assertEqual(instance.epoch, 0)
        token = lease.token(stamp, wall, stamp)
        self.assertIsNotNone(token)

        _, wall, stamp = acquisition(instance, 6, .6, 0., lost=True)
        lease.update(instance.status(wall, stamp), wall, stamp)
        self.assertEqual(instance.epoch, 1)
        self.assertEqual(instance.state, 'lost')
        self.assertFalse(instance.approved)
        self.assertFalse(lease.check(wall, stamp))
        self.assertFalse(lease.permits(token, wall, stamp))


class PostRevocationTests(unittest.TestCase):
    def counters(self, row):
        return {key: value for key, value in row.items() if type(value) is int}

    def test_the_first_approval_after_a_revocation_advances_no_counter(self):
        instance = gate()
        for index in range(1, 5):
            acquisition(instance, index, index * .1, 0.)
        instance.revoke('lost', 'injected')
        revoked = instance.epoch
        before = instance.status(.5, 10**9 + 5 * 10**8)

        approvals = []
        for index in range(6, 9):
            outputs, wall, stamp = acquisition(instance, index, index * .1, 0.)
            approvals += outputs
        after = instance.status(wall, stamp)

        self.assertEqual(len(approvals), 1)
        self.assertEqual(instance.epoch, revoked)
        self.assertEqual(after['epoch'], before['epoch'])
        # Only the clocks, the heartbeat sequence and the two stamps may move across a
        # recovery; anything else that moved would be a counter this change invented.
        moved = {key for key, value in self.counters(after).items()
                 if self.counters(before).get(key) != value}
        self.assertEqual(moved, {'sequence', 'stamp_ns', 'clock_ns', 'correction_stamp_ns'})
        # The anchor names the new approval, not a stamp carried over from before the loss.
        self.assertEqual(after['correction_stamp_ns'], approvals[0]['stamp_ns'])
        self.assertEqual(before['correction_stamp_ns'], 0)
        for name in list(after) + list(vars(instance)):
            self.assertNotRegex(name, FORBIDDEN_FIELD)


class AppendOnlyTests(unittest.TestCase):
    def valid_row(self):
        instance = gate()
        for index in range(1, 6):
            _, wall, stamp = acquisition(instance, index, index * .1, .25)
            row = instance.status(wall, stamp)
        self.assertEqual(row['state'], 'valid')
        return row

    def test_the_keys_are_the_pinned_old_sets_plus_exactly_the_new_ones(self):
        row = self.valid_row()
        self.assertEqual(set(row), OLD_STATUS_KEYS | NEW_STATUS_KEYS)
        self.assertTrue(row['approved'])
        for item in row['approved']:
            self.assertEqual(set(item), OLD_APPROVAL_KEYS | NEW_APPROVAL_KEYS)

    def test_the_schema_number_is_unchanged(self):
        self.assertEqual(self.valid_row()['schema'], 1)

    def test_the_row_survives_the_serialisation_the_node_publishes_it_with(self):
        row = self.valid_row()
        restored = json.loads(json.dumps(row, allow_nan=False))
        self.assertEqual(restored, row)
        for item in restored['approved']:
            self.assertEqual(np.asarray(item['map_odom']).shape, (4, 4))
            for value in itertools.chain.from_iterable(item['map_odom']):
                self.assertIs(type(value), float)

    def test_the_correction_matches_the_two_poses_the_approval_already_carried(self):
        # The honest statement of what is new: this is composable from fields that were
        # always there. If it ever stops matching them, the contract is lying.
        row = self.valid_row()
        for item in row['approved']:
            expected = pose_matrix(item['pose']['pose']) @ np.linalg.inv(pose_matrix(item['odom_pose']))
            np.testing.assert_array_equal(np.asarray(item['map_odom'], dtype=float), expected)


class RigidValidationTests(unittest.TestCase):
    def composed(self):
        instance = gate()
        for index in range(1, 6):
            outputs, _, _ = acquisition(instance, index, index * .1, .25, TILTED_Q)
        return outputs[0]['map_odom']

    def test_a_genuine_composition_passes(self):
        self.assertTrue(rigid_valid(self.composed()))
        self.assertTrue(rigid_valid(np.eye(4)))

    def test_a_non_orthogonal_rotation_is_rejected(self):
        bad = np.eye(4)
        bad[0, 0] = 1.001
        self.assertFalse(rigid_valid(bad))

    def test_a_determinant_of_minus_one_is_rejected(self):
        # A mirror is perfectly orthogonal, so the determinant has to be checked separately.
        mirrored = np.eye(4)
        mirrored[2, 2] = -1.
        self.assertTrue(np.allclose(mirrored[:3, :3] @ mirrored[:3, :3].T, np.eye(3)))
        self.assertFalse(rigid_valid(mirrored))

    def test_a_wrong_bottom_row_is_rejected(self):
        for bottom in ([0., 0., 0., 2.], [1., 0., 0., 1.], [0., 0., 0., 0.]):
            candidate = np.eye(4)
            candidate[3] = bottom
            self.assertFalse(rigid_valid(candidate), bottom)

    def test_non_finite_wrong_shape_and_non_numeric_are_rejected(self):
        for bad in (np.full((4, 4), np.nan), np.full((4, 4), np.inf), np.eye(3), np.eye(5),
                    [[1., 2.], [3.]], 'not a matrix', None, {'a': 1}, [], np.eye(4)[0]):
            self.assertFalse(rigid_valid(bad), repr(bad))

    def test_the_declared_tolerance_is_what_decides_the_boundary(self):
        # Dose-response, so the constant is doing the work and not some other check.
        inside, outside = np.eye(4), np.eye(4)
        inside[0, 1] = RIGID_ATOL / 10.
        outside[0, 1] = RIGID_ATOL * 10.
        self.assertTrue(rigid_valid(inside))
        self.assertFalse(rigid_valid(outside))
        self.assertTrue(rigid_valid(outside, atol=RIGID_ATOL * 100))


class LeaseValidationTests(unittest.TestCase):
    def valid(self):
        instance, lease = gate(), LocalizationLease(MAP_ID)
        for index in range(1, 6):
            _, wall, stamp = acquisition(instance, index, index * .1, .25)
            row = instance.status(wall, stamp)
        return lease, row, wall, stamp

    def feed(self, mangle=None):
        lease, row, wall, ros_ns = self.valid()
        payload = copy.deepcopy(row)
        if mangle is not None:
            mangle(payload)
        lease.update(payload, wall, ros_ns)
        return lease, lease.check(wall, ros_ns), payload, wall, ros_ns

    def test_an_untouched_row_stays_healthy(self):
        # Negative control for every rejection below: same row, same clocks, no change.
        self.assertTrue(self.feed()[1])

    def test_a_non_rigid_correction_fails_the_lease_closed(self):
        def mirror(row):
            matrix = np.asarray(row['approved'][-1]['map_odom'], dtype=float)
            matrix[2, 2] = -matrix[2, 2]
            row['approved'][-1]['map_odom'] = matrix.tolist()

        def skew(row):
            matrix = np.asarray(row['approved'][0]['map_odom'], dtype=float)
            matrix[0, 0] *= 1.001
            row['approved'][0]['map_odom'] = matrix.tolist()

        def bottom(row):
            matrix = np.asarray(row['approved'][0]['map_odom'], dtype=float)
            matrix[3] = [0., 0., 1., 1.]
            row['approved'][0]['map_odom'] = matrix.tolist()

        def not_finite(row):
            row['approved'][0]['map_odom'][0][3] = float('inf')

        for mangle in (mirror, skew, bottom, not_finite):
            with self.subTest(mangle=mangle.__name__):
                self.assertFalse(self.feed(mangle)[1])

    def test_a_correction_stamp_that_names_the_wrong_approval_is_rejected(self):
        for delta in (1, -1):
            with self.subTest(delta=delta):
                self.assertFalse(self.feed(
                    lambda row: row.update(correction_stamp_ns=row['correction_stamp_ns'] + delta))[1])
        # An older approval of the same row is a stale anchor and is rejected too.
        self.assertGreater(len(self.valid()[1]['approved']), 1, 'need an older approval')
        self.assertFalse(self.feed(
            lambda row: row.update(correction_stamp_ns=row['approved'][0]['stamp_ns']))[1])
        for bad in (1.5, True, '1', None):
            with self.subTest(bad=bad):
                self.assertFalse(self.feed(lambda row: row.update(correction_stamp_ns=bad))[1])

    def test_a_zero_anchor_states_no_anchor_rather_than_a_wrong_one(self):
        # A status assembled without this field reads as 0, and a row that states nothing
        # must not be rejected: tests/test_runtime_diagnostics.py builds exactly such a row
        # out of a fresh gate's status plus a hand-written approval. The relaxation is
        # bounded - any NON-zero anchor is still held to naming the newest approval.
        self.assertTrue(self.feed(lambda row: row.update(correction_stamp_ns=0))[1])

    def test_a_status_recorded_before_the_contract_still_validates(self):
        # Recorded runs are replayed through this lease by
        # tools/rtabmap_nav2_rehearsal_run.py and tools/rtabmap_frame_disposition_probe.py.
        def strip(row):
            row.pop('correction_stamp_ns')
            for item in row['approved']:
                item.pop('map_odom')

        lease, healthy, payload, wall, ros_ns = self.feed(strip)
        self.assertTrue(healthy)
        with self.assertRaises(ValueError):
            lease.correction(payload['stamp_ns'], wall, ros_ns)

    def test_the_correction_accessor_is_gated_like_the_pose_accessor(self):
        lease, healthy, payload, wall, ros_ns = self.feed()
        self.assertTrue(healthy)
        np.testing.assert_array_equal(
            lease.correction(payload['stamp_ns'], wall, ros_ns),
            np.asarray(payload['approved'][-1]['map_odom'], dtype=float))
        with self.assertRaises(ValueError):
            lease.correction(payload['stamp_ns'] + 1, wall, ros_ns)
        with self.assertRaises(ValueError):          # lease expired, no reading at all
            lease.correction(payload['stamp_ns'], wall + 2., ros_ns)


@unittest.skipUnless((RECORDING / 'baseline' / 'normal' / 'gate-events.jsonl').is_file(),
                     'frozen rehearsal recording not present')
class RecordedRegressionTests(unittest.TestCase):
    """Existing fields must survive on real recorded input before the new field matters.

    The expectation is not computed here: it is the `approved` payload the pre-change gate
    wrote into gate-events.jsonl during the 2026-09-14 rehearsal. The changed gate is driven
    from the recorded observations.jsonl of the same run through the same replay tool the
    project already uses, and every approval that both produce is compared field by field.
    """

    def recorded(self, arm):
        base = RECORDING / arm / 'normal'
        approvals, map_id = {}, None
        for line in base.joinpath('gate-events.jsonl').read_bytes().splitlines():
            row = json.loads(line)
            map_id = map_id or row['map_id']
            for item in row.get('approved') or ():
                approvals.setdefault(item['stamp_ns'], item)
        return base, map_id, approvals

    def replayed(self, base, map_id):
        from tools.rtabmap_gate_replay import replay
        from host.ros_localization_gate import load_prior
        events = [json.loads(line)
                  for line in base.joinpath('observations.jsonl').read_bytes().splitlines()]
        result = replay(events, map_id, load_prior(base / 'map.db', map_id))
        return {item['stamp_ns']: item for item in result['approvals']}

    def test_every_field_the_pre_change_gate_recorded_comes_back_bit_for_bit(self):
        for arm in ('baseline', 'loaded'):
            with self.subTest(arm=arm):
                base, map_id, recorded = self.recorded(arm)
                self.assertTrue(recorded)
                # The frozen rows are genuinely pre-change output.
                self.assertFalse(any('map_odom' in item for item in recorded.values()))
                replayed = self.replayed(base, map_id)
                shared = sorted(set(recorded) & set(replayed))
                self.assertGreaterEqual(len(shared), 20, 'nothing to compare')
                for stamp in shared:
                    old, new = recorded[stamp], replayed[stamp]
                    self.assertEqual(set(new), set(old) | NEW_APPROVAL_KEYS)
                    self.assertEqual(json.dumps({k: new[k] for k in old}, sort_keys=True),
                                     json.dumps(old, sort_keys=True))
                # The comparison is not vacuous: 1 mm into one field and it fails.
                stamp = shared[0]
                shifted = copy.deepcopy(recorded[stamp])
                shifted['pose']['pose']['position']['x'] += 1e-3
                self.assertNotEqual(
                    json.dumps({k: replayed[stamp][k] for k in shifted}, sort_keys=True),
                    json.dumps(shifted, sort_keys=True))

    def test_the_correction_recomposes_the_recorded_poses_exactly(self):
        for arm in ('baseline', 'loaded'):
            with self.subTest(arm=arm):
                base, map_id, recorded = self.recorded(arm)
                replayed = self.replayed(base, map_id)
                # Only stamps both runs approved: the offline replay uses a synthetic timer,
                # so it approves a subset of the frames the live node did.
                shared = sorted(set(recorded) & set(replayed))
                for stamp in shared:
                    expected = (pose_matrix(recorded[stamp]['pose']['pose'])
                                @ np.linalg.inv(pose_matrix(recorded[stamp]['odom_pose'])))
                    np.testing.assert_array_equal(
                        np.asarray(replayed[stamp]['map_odom'], dtype=float), expected)
                for item in replayed.values():
                    self.assertTrue(rigid_valid(item['map_odom']))
                self.assertGreaterEqual(len(shared), 20)


if __name__ == '__main__':
    unittest.main()
