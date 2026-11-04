"""D-035 items 1-2: the navigation consumer's same-map correction rule.

Rule (docs/design/NAV_CORRECTION_RULES_20260927.md §1-§2): within one unchanged four-part
authorization identity, a correction that moves the tracked point by >= 0.10 m on the map
plane, or turns yaw by >= 5 degrees (shortest angle), revokes the goal and cancels it once.
The reference is the newest approval when the goal was sent; it rides on the authorization
without taking part in its equality.

Every status in the gate-driven classes is produced by main's real LocalizationGate and
reaches the consumer as JSON. Host-only: no ROS, DDS, TF or action server.
tests/test_nav_correction_rule_variants.py swaps wrong implementations in and requires the
named tests here to fail.
"""
import copy
import importlib.util
import json
import math
from pathlib import Path
import unittest

import numpy as np

from host import nav_localization as nl
from host.localization_gate import KINDS, LocalizationGate

SPEC = importlib.util.spec_from_file_location(
    'nav_suite_correction_rules_test',
    Path(__file__).resolve().parents[1] / 'robot_sim/tests/nav_point_to_point.py')
NAV_SUITE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NAV_SUITE)

MAP_ID = 'd' * 64
PRIOR_ID = 11
COV = [float(i % 7 == 0) * .01 for i in range(36)]
STEP = 1 / 1024.


def planar(x, y, yaw):
    return {'position': {'x': float(x), 'y': float(y), 'z': 0.},
            'orientation': {'x': 0., 'y': 0., 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}}


def compose(correction, odom):
    """map pose = correction (x, y, yaw) applied to an odom pose (x, y, yaw)."""
    cx, cy, cyaw = correction
    ox, oy, oyaw = odom
    return (cx + math.cos(cyaw) * ox - math.sin(cyaw) * oy,
            cy + math.sin(cyaw) * ox + math.cos(cyaw) * oy, cyaw + oyaw)


def matrix(x, y, yaw):
    result = np.eye(4)
    result[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    result[:2, 3] = [x, y]
    return result


class Handle:
    accepted = True

    def __init__(self):
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1
        return object()


class Run:
    """One real gate and one consumer; the odom track advances STEP per acquisition."""

    def __init__(self, odom_origin=(0., 0.)):
        self.gate = LocalizationGate(MAP_ID, {PRIOR_ID}, pose_source='estimated',
                                     navigation_ready=True)
        self.consumer = nl.NavigationLocalization('estimated', MAP_ID, MAP_ID)
        self.origin = odom_origin
        self.index = 0
        self.rows = []

    def step(self, correction=(0., 0., 0.), lost=False, mutate=None):
        self.index += 1
        stamp = 10**9 + self.index * 10**8
        self.wall, self.ros, self.stamp = self.index * .1, stamp, stamp
        odom = (self.origin[0] + self.index * STEP, self.origin[1], 0.)
        in_map = compose(correction, odom)
        self.gate.acquisition(stamp, self.wall, stamp)
        for kind in KINDS:
            in_odom = kind in KINDS[:2]
            body = planar(*(odom if in_odom else in_map))
            self.gate.ingest(kind, {
                'header': {'stamp': {'sec': stamp // 10**9, 'nanosec': stamp % 10**9},
                           'frame_id': 'odom' if in_odom else 'map'},
                'child_frame_id': 'camera_link', 'pose': {'pose': body, 'covariance': list(COV)},
                'covariance': list(COV), 'lost': lost and kind == 'odom_info',
                'loop_closure_id': PRIOR_ID, 'proximity_detection_id': 0}, self.wall, stamp)
        row = self.gate.status(self.wall, stamp)
        if mutate is not None:
            row = copy.deepcopy(row)
            mutate(row)
        self.rows.append(row)
        self.consumer.update_json(json.dumps(row), self.wall, stamp)
        return row

    def settle(self, correction=(0., 0., 0.), count=3):
        for _ in range(count):
            self.step(correction)
        return self

    def identity(self):
        return self.consumer.identity(self.wall, self.ros)


class Goal:
    """A goal sent the way the executable sends it, then ticked per delivered status."""

    def __init__(self, run, authorization=None):
        self.run = run
        self.authorization = (authorization if authorization is not None else
                              NAV_SUITE.goal_authorization(run.consumer, run.wall, run.ros))
        self.lifecycle = nl.GoalLifecycle()
        self.context = self.lifecycle.start(self.authorization)
        self.handle = Handle()
        self.lifecycle.accepted(self.context, self.handle)
        self.state = {'fault': None, 'enabled': True, 'correction': None}

    def tick(self):
        return NAV_SUITE.evaluate_navigation_tick(self.state, self.lifecycle, self.run.consumer,
                                                  True, self.run.wall, self.run.ros)

    def drive(self, corrections):
        permits = []
        for correction in corrections:
            self.run.step(correction)
            permits.append(self.tick())
        return permits


class ThresholdTests(unittest.TestCase):
    def test_translation_limit_is_inclusive(self):
        self.assertTrue(nl.correction_exceeds(nl.TRANSLATION_LIMIT_M, 0.))
        self.assertFalse(nl.correction_exceeds(float(np.nextafter(nl.TRANSLATION_LIMIT_M, 0.)), 0.))

    def test_yaw_limit_is_inclusive(self):
        self.assertTrue(nl.correction_exceeds(0., nl.YAW_LIMIT_RAD))
        self.assertFalse(nl.correction_exceeds(0., float(np.nextafter(nl.YAW_LIMIT_RAD, 0.))))

    def test_well_inside_and_well_outside(self):
        self.assertFalse(nl.correction_exceeds(.05, math.radians(2)))
        self.assertTrue(nl.correction_exceeds(.2, 0.))
        self.assertTrue(nl.correction_exceeds(0., math.radians(6)))

    def test_nonfinite_measurement_exceeds(self):
        for value in (float('nan'), float('inf')):
            self.assertTrue(nl.correction_exceeds(value, 0.))
            self.assertTrue(nl.correction_exceeds(0., value))

    def test_limits_are_the_d035_values(self):
        self.assertEqual(nl.TRANSLATION_LIMIT_M, .10)
        self.assertEqual(nl.YAW_LIMIT_RAD, math.radians(5.))


class ShortestAngleTests(unittest.TestCase):
    def test_across_plus_minus_180_is_the_short_way(self):
        self.assertAlmostEqual(nl.shortest_yaw_difference(math.radians(179), math.radians(-179)),
                               math.radians(2), places=12)
        translation, yaw = nl.correction_delta(matrix(0, 0, math.radians(179)),
                                               matrix(0, 0, math.radians(-179)), (0., 0., 0.))
        self.assertAlmostEqual(yaw, math.radians(2), places=12)
        self.assertFalse(nl.correction_exceeds(translation, yaw))

    def test_sign_does_not_matter(self):
        self.assertAlmostEqual(nl.shortest_yaw_difference(0., math.radians(-6)), math.radians(6))
        self.assertAlmostEqual(nl.shortest_yaw_difference(math.radians(-6), 0.), math.radians(6))


class TranslationPointTests(unittest.TestCase):
    """The translation is the tracked point's jump on the map, not the odom origin's."""

    def test_small_turn_about_the_robot_far_from_odom_origin_does_not_exceed(self):
        robot = (20., 0., 0.)
        turn = math.radians(1)
        # Rotation by 1 deg about the robot: its translation column moves ~0.35 m.
        about_robot = matrix(robot[0] - math.cos(turn) * robot[0], -math.sin(turn) * robot[0], turn)
        self.assertGreater(np.hypot(*about_robot[:2, 3]), .3)
        translation, yaw = nl.correction_delta(np.eye(4), about_robot, robot)
        self.assertLess(translation, 1e-9)
        self.assertFalse(nl.correction_exceeds(translation, yaw))

    def test_small_turn_about_a_far_origin_moves_the_robot_and_exceeds(self):
        translation, yaw = nl.correction_delta(np.eye(4), matrix(0, 0, math.radians(1)),
                                               (20., 0., 0.))
        self.assertAlmostEqual(translation, 2 * 20 * math.sin(math.radians(.5)), places=9)
        self.assertTrue(nl.correction_exceeds(translation, yaw))


class GateDrivenRuleTests(unittest.TestCase):
    def test_translation_over_limit_revokes_and_cancels_once(self):
        run = Run().settle()
        goal = Goal(run)
        permits = goal.drive([(.2, 0., 0.)] * 3)
        self.assertEqual(permits, [False, False, False])
        self.assertTrue(goal.context.revoked)
        self.assertEqual(goal.handle.cancels, 1)
        self.assertEqual(goal.state['correction'], nl.MAP_CORRECTION_EXCEEDED)
        self.assertIsNone(goal.state['fault'])
        self.assertEqual(run.identity(), goal.authorization)  # same map, same identity

    def test_translation_under_limit_keeps_the_goal(self):
        run = Run().settle()
        goal = Goal(run)
        self.assertEqual(goal.drive([(.05, 0., 0.)] * 3), [True, True, True])
        self.assertFalse(goal.context.revoked)
        self.assertEqual(goal.handle.cancels, 0)
        self.assertIsNone(goal.state['correction'])

    def test_diagonal_translation_is_planar_distance(self):
        run = Run().settle()
        goal = Goal(run)
        # 0.08 in x and 0.08 in y: each under the limit, together 0.113 m.
        self.assertEqual(goal.drive([(.08, .08, 0.)]), [False])
        self.assertEqual(goal.state['correction'], nl.MAP_CORRECTION_EXCEEDED)

    def test_yaw_over_limit_revokes(self):
        run = Run().settle()
        goal = Goal(run)
        self.assertEqual(goal.drive([(0., 0., math.radians(6))]), [False])
        self.assertEqual(goal.state['correction'], nl.MAP_CORRECTION_EXCEEDED)
        self.assertEqual(goal.handle.cancels, 1)

    def test_yaw_under_limit_keeps_the_goal(self):
        run = Run().settle()
        goal = Goal(run)
        self.assertEqual(goal.drive([(0., 0., math.radians(2))] * 3), [True, True, True])
        self.assertIsNone(goal.state['correction'])

    def test_transient_jump_inside_the_row_window_is_still_seen(self):
        run = Run().settle()
        goal = Goal(run)
        run.step((.2, 0., 0.))   # delivered but not ticked
        run.step((0., 0., 0.))   # newest approval is back at the reference
        self.assertFalse(goal.tick())
        self.assertEqual(goal.state['correction'], nl.MAP_CORRECTION_EXCEEDED)

    def test_generation_change_keeps_the_old_rule_and_is_not_reissuable(self):
        run = Run().settle()
        goal = Goal(run)
        run.step((0., 0., 0.), lost=True)
        self.assertFalse(goal.tick())
        self.assertTrue(goal.context.revoked)
        self.assertEqual(goal.state['fault'], 'localization_generation_changed')
        self.assertIsNone(goal.state['correction'])
        goal.lifecycle.result(goal.context, 5)
        self.assertFalse(NAV_SUITE.correction_reissue_due(goal.state, goal.context, 0))


class ReferenceTests(unittest.TestCase):
    def test_reference_is_the_newest_approval_when_the_goal_is_sent(self):
        run = Run().settle()
        run.step((.5, 0., 0.))
        authorization = NAV_SUITE.goal_authorization(run.consumer, run.wall, run.ros)
        row = run.rows[-1]
        self.assertEqual(authorization.reference.stamp_ns, row['correction_stamp_ns'])
        newest = next(item for item in row['approved'] if item['stamp_ns'] == row['correction_stamp_ns'])
        np.testing.assert_array_equal(np.asarray(authorization.reference.map_odom),
                                      np.asarray(newest['map_odom']))

    def test_a_correction_before_the_goal_does_not_revoke_it(self):
        run = Run().settle()
        run.settle((.5, 0., 0.), count=2)   # older 0.0 approvals are still in the row
        self.assertTrue(any(abs(item['map_odom'][0][3]) < 1e-12 for item in run.rows[-1]['approved']))
        goal = Goal(run)
        self.assertEqual(goal.drive([(.5, 0., 0.)] * 3), [True, True, True])
        self.assertFalse(goal.context.revoked)

    def test_a_correction_after_the_goal_revokes_it(self):
        run = Run().settle()
        run.settle((.5, 0., 0.), count=2)
        goal = Goal(run)
        self.assertEqual(goal.drive([(.7, 0., 0.)]), [False])
        self.assertEqual(goal.state['correction'], nl.MAP_CORRECTION_EXCEEDED)

    def test_identity_is_still_the_four_part_tuple(self):
        run = Run().settle()
        authorization = run.identity()
        self.assertEqual(len(authorization), 4)
        lease = run.consumer.lease
        plain = (lease.generation, run.rows[-1]['instance'], run.rows[-1]['epoch'], MAP_ID)
        self.assertEqual(tuple(authorization), plain)
        self.assertEqual(authorization, plain)
        self.assertEqual(hash(authorization), hash(plain))

    def test_new_approvals_under_the_limit_do_not_change_the_identity(self):
        run = Run().settle()
        goal = Goal(run)
        goal.drive([(0., 0., 0.)] * 5)
        self.assertTrue(run.consumer.permits(goal.authorization, run.wall, run.ros))
        self.assertNotEqual(run.identity().reference.stamp_ns, goal.authorization.reference.stamp_ns)
        self.assertFalse(goal.context.revoked)


class FailClosedTests(unittest.TestCase):
    def test_estimated_goal_without_a_reference_is_revoked_as_a_fault(self):
        run = Run().settle()
        goal = Goal(run, authorization=tuple(run.identity()))  # the reference stripped off
        self.assertFalse(goal.tick())
        self.assertTrue(goal.context.revoked)
        self.assertEqual(goal.state['fault'], nl.MAP_CORRECTION_REFERENCE_MISSING)
        self.assertIsNone(goal.state['correction'])

    def test_a_row_without_map_odom_cannot_send_a_goal(self):
        def strip(row):
            for item in row['approved']:
                item.pop('map_odom')
        run = Run()
        for _ in range(3):
            run.step(mutate=strip)
        self.assertIsNotNone(run.identity())          # still authorised for reading
        self.assertIsNone(run.identity().reference)
        with self.assertRaisesRegex(RuntimeError, 'map_correction_reference_unavailable'):
            NAV_SUITE.goal_authorization(run.consumer, run.wall, run.ros)

    def test_newer_approval_without_odom_pose_is_unreadable(self):
        run = Run().settle()
        goal = Goal(run)
        run.step(mutate=lambda row: row['approved'][-1].pop('odom_pose'))
        self.assertFalse(goal.tick())
        self.assertEqual(goal.state['fault'], nl.MAP_CORRECTION_UNREADABLE)

    def test_reference_mode_has_no_correction_rule(self):
        policy = nl.NavigationLocalization('reference')
        authorization = NAV_SUITE.goal_authorization(policy, 1., 1)
        self.assertEqual(authorization, ('reference',))
        self.assertIsNone(policy.correction_verdict(authorization, 1., 1))


class ReissueTests(unittest.TestCase):
    def revoked_goal(self):
        run = Run().settle()
        goal = Goal(run)
        goal.drive([(.2, 0., 0.)])
        goal.lifecycle.result(goal.context, 5)
        return run, goal

    def test_reissue_after_terminal_uses_the_new_newest_approval(self):
        run, goal = self.revoked_goal()
        self.assertTrue(NAV_SUITE.correction_reissue_due(goal.state, goal.context, 0))
        goal.state['correction'] = None
        again = NAV_SUITE.goal_authorization(run.consumer, run.wall, run.ros)
        self.assertEqual(again, goal.authorization)     # same map, same identity
        self.assertEqual(again.reference.stamp_ns, run.rows[-1]['correction_stamp_ns'])
        context = goal.lifecycle.start(again)
        goal.lifecycle.accepted(context, Handle())
        goal.context = context
        goal.state['enabled'] = True   # what the executable sets on the new acceptance
        # The corrected map is now the reference: staying there keeps the new goal.
        self.assertEqual(goal.drive([(.2, 0., 0.)] * 3), [True, True, True])

    def test_no_reissue_before_the_terminal_result(self):
        run = Run().settle()
        goal = Goal(run)
        goal.drive([(.2, 0., 0.)])
        self.assertFalse(NAV_SUITE.correction_reissue_due(goal.state, goal.context, 0))

    def test_reissue_budget_is_bounded(self):
        _, goal = self.revoked_goal()
        limit = NAV_SUITE.MAX_CORRECTION_REISSUES
        self.assertTrue(NAV_SUITE.correction_reissue_due(goal.state, goal.context, limit - 1))
        self.assertFalse(NAV_SUITE.correction_reissue_due(goal.state, goal.context, limit))
        self.assertEqual(goal.state['fault'], 'map_correction_reissue_budget_exhausted')

    def test_fault_or_transport_error_is_never_reissued(self):
        _, goal = self.revoked_goal()
        goal.state['fault'] = 'input_unhealthy_during_action'
        self.assertFalse(NAV_SUITE.correction_reissue_due(goal.state, goal.context, 0))
        _, goal = self.revoked_goal()
        goal.context.transport_error = 'goal_result_transport_failed:x'
        self.assertFalse(NAV_SUITE.correction_reissue_due(goal.state, goal.context, 0))


if __name__ == '__main__':
    unittest.main()
