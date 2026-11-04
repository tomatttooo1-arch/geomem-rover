"""dev2's navigation consumer, rehung onto main's localization gate (D-023/D-025/D-027).

Every status here is produced by main's real LocalizationGate and crosses to the consumer
as JSON, the way /fpga/localization_status carries it; no status row is written by hand
except where a single field is corrupted on purpose. What this file pins:

1. The consumer runs on the new contract: statuses carrying `map_odom` and
   `correction_stamp_ns` authorise it exactly as before.
2. A map->odom correction inside one authorisation generation is now OBSERVABLE to the
   consumer (lease.correction), and the gate still produces no version for it: the epoch,
   instance, state and key set of the status stream are those of an uncorrected run.
3. The identity tuple (generation, instance, epoch, map_id) does not change on a
   correction. Until 2026-09-27 that meant a correction did not revoke an active goal
   (the gap dev2 recorded in NAV2_STAGE_D.md §5.2, §6). D-035 settled the rule; its tests
   are tests/test_nav_correction_rules.py, and the 1 m case below now revokes.
4. Negative controls: a real revocation does change the identity and revokes the goal;
   an unchanged correction reads as unchanged; a corrupt correction fails closed.

Host-only: no ROS, DDS, TF or action server is exercised.
"""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

import numpy as np

from host.localization_gate import KINDS, LocalizationGate
from host.nav_localization import GoalLifecycle, NavigationLocalization

SPEC = importlib.util.spec_from_file_location(
    'nav_suite_rehang_contract_test',
    Path(__file__).resolve().parents[1] / 'robot_sim/tests/nav_point_to_point.py')
NAV_SUITE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NAV_SUITE)

MAP_ID = 'c' * 64
PRIOR_ID = 7
COV = [float(i % 7 == 0) * .01 for i in range(36)]
# Identity rotation and 1/1024 m steps keep the composition exact, so equal corrections
# compare equal and unequal ones differ by exactly the injected amount.
STEP = 1 / 1024.


def observation(kind, stamp, odom_x, map_x, lost=False):
    in_odom = kind in KINDS[:2]
    body = {'position': {'x': float(odom_x if in_odom else map_x), 'y': 0., 'z': 0.},
            'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}
    return {'header': {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9},
                       'frame_id': 'odom' if in_odom else 'map'},
            'child_frame_id': 'camera_link', 'pose': {'pose': body, 'covariance': list(COV)},
            'covariance': list(COV), 'lost': lost, 'loop_closure_id': PRIOR_ID,
            'proximity_detection_id': 0}


class Handle:
    accepted = True

    def __init__(self):
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1
        return object()


class Run:
    """One gate, one consumer, one steady/ROS clock, driven acquisition by acquisition."""

    def __init__(self):
        self.gate = LocalizationGate(MAP_ID, {PRIOR_ID}, pose_source='estimated',
                                     navigation_ready=True)
        self.consumer = NavigationLocalization('estimated', MAP_ID, MAP_ID)
        self.index = 0
        self.rows = []

    def step(self, correction_x, lost=False):
        """Drive one acquisition, publish one status, deliver it. Returns the status row."""
        self.index += 1
        stamp = 10**9 + self.index * 10**8
        self.wall = self.index * .1
        self.ros = stamp
        odom_x = self.index * STEP
        self.gate.acquisition(stamp, self.wall, stamp)
        for kind in KINDS:
            self.gate.ingest(kind, observation(kind, stamp, odom_x, odom_x + correction_x,
                                               lost=lost and kind == 'odom_info'),
                             self.wall, stamp)
        row = self.gate.status(self.wall, stamp)
        self.rows.append(row)
        self.consumer.update_json(json.dumps(row), self.wall, stamp)
        self.stamp = stamp
        return row

    def identity(self):
        return self.consumer.identity(self.wall, self.ros)

    def correction_x(self, stamp):
        return float(self.consumer.lease.correction(stamp, self.wall, self.ros)[0][3])


def ready_run():
    run = Run()
    for _ in range(3):
        run.step(0.)
    return run


class NewContractAuthorisesTests(unittest.TestCase):
    def test_gate_statuses_with_the_appended_fields_authorise_the_consumer(self):
        run = ready_run()
        row = run.rows[-1]
        # The statuses the consumer accepted really are new-contract rows.
        self.assertEqual(row['state'], 'valid')
        self.assertEqual(row['correction_stamp_ns'], run.stamp)
        self.assertTrue(all('map_odom' in item for item in row['approved']))
        self.assertIsNotNone(run.identity())
        self.assertTrue(run.consumer.healthy(run.wall, run.ros))

    def test_estimated_cli_still_refuses_to_move(self):
        # The refusal stood until the consumer had a rule for the signal. D-035 (2026-09-27)
        # defined it (tests/test_nav_correction_rules.py) and item 3 unlocks the entry.
        args, localization = NAV_SUITE.parse_arguments(['--out', '/nonexistent/out.jsonl',
                                                        '--pose-source', 'estimated',
                                                        '--localization-map-id', MAP_ID,
                                                        '--navigation-map-id', MAP_ID])
        self.assertEqual(localization.mode, 'estimated')


class CorrectionInsideOneGenerationTests(unittest.TestCase):
    def drive(self, correction_after):
        """Three settling steps, a goal, then three steps at `correction_after` metres."""
        run = ready_run()
        before_stamp = run.stamp
        authorization = run.identity()
        lifecycle = GoalLifecycle()
        context = lifecycle.start(authorization)
        handle = Handle()
        lifecycle.accepted(context, handle)
        state = {'fault': None, 'enabled': True}
        permits = []
        for _ in range(3):
            run.step(correction_after)
            permits.append(NAV_SUITE.evaluate_navigation_tick(
                state, lifecycle, run.consumer, True, run.wall, run.ros))
        self.lifecycle = lifecycle
        return run, before_stamp, authorization, context, handle, state, permits

    def test_a_one_metre_correction_reaches_the_consumer_without_a_gate_version(self):
        run, before, authorization, context, handle, state, permits = self.drive(1.0)
        control = self.drive(0.)[0]
        # Observable: the consumer reads the two corrections and they differ by 1 m.
        self.assertEqual(run.correction_x(before), 0.)
        self.assertEqual(run.correction_x(run.stamp), 1.0)
        self.assertEqual(run.rows[-1]['correction_stamp_ns'], run.stamp)
        # No gate version: every field the gate derives is the uncorrected run's.
        for moved, still in zip(run.rows, control.rows):
            self.assertEqual(set(moved), set(still))
            for key in ('epoch', 'state', 'reason', 'sequence', 'stamp_ns',
                        'correction_stamp_ns', 'navigation_ready'):
                self.assertEqual(moved[key], still[key], key)
        self.assertEqual({row['epoch'] for row in run.rows}, {0})
        # D-035 items 1-2 (2026-09-27) replaced the old 'identity unchanged, goal not
        # revoked' statement: the identity is still unchanged, and the consumer's correction
        # rule now revokes and cancels once. It is not a fault; the task layer re-sends.
        self.assertEqual(run.identity(), authorization)
        self.assertEqual(permits, [False, False, False])
        self.assertTrue(context.revoked)
        self.assertEqual(handle.cancels, 1)
        self.assertIsNone(state['fault'])
        self.assertEqual(state['correction'], 'map_correction_exceeded')

    def test_negative_control_an_unchanged_correction_reads_unchanged(self):
        run, before, _, _, _, _, _ = self.drive(0.)
        self.assertEqual(run.correction_x(before), run.correction_x(run.stamp))

    def test_negative_control_a_real_revocation_revokes_the_goal_for_good(self):
        run, _, authorization, context, handle, state, _ = self.drive(0.)
        lost = run.step(0., lost=True)
        self.assertEqual(lost['epoch'], 1)
        self.assertNotEqual(lost['state'], 'valid')
        allowed = NAV_SUITE.evaluate_navigation_tick(
            state, self.lifecycle, run.consumer, True, run.wall, run.ros)
        self.assertFalse(allowed)
        self.assertTrue(context.revoked)
        self.assertEqual(handle.cancels, 1)
        self.assertEqual(state['fault'], 'localization_generation_changed')
        # Recovery produces a new generation; the old authorisation never permits again.
        for _ in range(3):
            run.step(0.)
        self.assertIsNotNone(run.identity())
        self.assertNotEqual(run.identity(), authorization)
        self.assertFalse(run.consumer.permits(authorization, run.wall, run.ros))


class CorruptCorrectionFailsClosedTests(unittest.TestCase):
    def deliver(self, mutate):
        run = ready_run()
        run.step(0.)
        run.step(0.)
        row = copy.deepcopy(run.gate.status(run.wall, run.ros))
        self.assertEqual(len(row['approved']), 3)  # an older anchor exists to point at
        mutate(row)
        consumer = NavigationLocalization('estimated', MAP_ID, MAP_ID)
        consumer.update_json(json.dumps(row), run.wall, run.ros)
        return consumer.identity(run.wall, run.ros)

    def test_negative_control_the_untouched_row_authorises(self):
        self.assertIsNotNone(self.deliver(lambda row: None))

    def test_a_scaled_correction_is_not_rigid_and_fails_closed(self):
        def scale(row):
            matrix = np.asarray(row['approved'][-1]['map_odom'], dtype=float)
            matrix[:3, :3] *= 1.1
            row['approved'][-1]['map_odom'] = matrix.tolist()
        self.assertIsNone(self.deliver(scale))

    def test_a_mirrored_correction_fails_closed(self):
        def mirror(row):
            matrix = np.asarray(row['approved'][-1]['map_odom'], dtype=float)
            matrix[0, 0] = -1.
            row['approved'][-1]['map_odom'] = matrix.tolist()
        self.assertIsNone(self.deliver(mirror))

    def test_an_anchor_that_is_not_the_newest_approval_fails_closed(self):
        def older(row):
            row['correction_stamp_ns'] = min(item['stamp_ns'] for item in row['approved'])
        self.assertIsNone(self.deliver(older))


if __name__ == '__main__':
    unittest.main()
