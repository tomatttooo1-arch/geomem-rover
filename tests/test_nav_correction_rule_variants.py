"""Fault variants: each wrong implementation of the D-035 rule must be caught by a named test.

A variant is swapped in for the duration of one run of tests/test_nav_correction_rules.py
test methods; the named tests must fail under it, and the same tests must pass with the
real implementation (control). Design: docs/design/NAV_CORRECTION_RULES_20260927.md §6.
"""
import math
import unittest
from unittest.mock import patch

import test_nav_correction_rules as rules
from host import nav_localization as nl

REAL_VERDICT = nl.NavigationLocalization.correction_verdict
REAL_IDENTITY = nl.NavigationLocalization.identity
REAL_REFERENCE = nl.NavigationLocalization.correction_reference
REAL_TICK = rules.NAV_SUITE.evaluate_navigation_tick


def strict_greater(translation, yaw):
    if not (math.isfinite(translation) and math.isfinite(yaw)):
        return True
    return translation > nl.TRANSLATION_LIMIT_M or yaw > nl.YAW_LIMIT_RAD


def reversed_comparison(translation, yaw):
    return translation < nl.TRANSLATION_LIMIT_M or yaw < nl.YAW_LIMIT_RAD


def naive_yaw(reference_yaw, current_yaw):
    return abs(current_yaw - reference_yaw)


def translation_columns(reference, current, _point):
    return (math.hypot(float(current[0][3]) - float(reference[0][3]),
                       float(current[1][3]) - float(reference[1][3])),
            nl.shortest_yaw_difference(nl.planar_yaw(reference), nl.planar_yaw(current)))


def exceed_without_cancel(self, authorization, wall, ros_ns):
    verdict = REAL_VERDICT(self, authorization, wall, ros_ns)
    return None if verdict == nl.MAP_CORRECTION_EXCEEDED else verdict


def newest_only(self, authorization, wall, ros_ns):
    reference = getattr(authorization, 'reference', None)
    if self.lease is None or reference is None:
        return REAL_VERDICT(self, authorization, wall, ros_ns)
    newest = self._newest_approval()
    current = self.lease.correction(newest['stamp_ns'], wall, ros_ns)
    odom = newest['odom_pose']['position']
    delta = nl.correction_delta(reference.map_odom, current, (odom['x'], odom['y'], odom['z']))
    return nl.MAP_CORRECTION_EXCEEDED if nl.correction_exceeds(*delta) else None


def oldest_reference(self, wall, ros_ns):
    if self.lease is None or self.lease.row is None or not self.lease.row['approved']:
        return None
    oldest = min(self.lease.row['approved'], key=lambda item: item['stamp_ns'])
    matrix = self.lease.correction(oldest['stamp_ns'], wall, ros_ns)
    return nl.CorrectionReference(oldest['stamp_ns'], tuple(tuple(map(float, line)) for line in matrix))


def reference_read_at_check_time(self, authorization, wall, ros_ns):
    if self.lease is None:
        return None
    current = nl.Authorization(tuple(authorization), REAL_REFERENCE(self, wall, ros_ns))
    return REAL_VERDICT(self, current, wall, ros_ns)


def stamp_in_identity(self, wall, ros_ns):
    identity = REAL_IDENTITY(self, wall, ros_ns)
    if identity is None or self.lease is None:
        return identity
    return nl.Authorization(tuple(identity) + (self.lease.row['correction_stamp_ns'],),
                            identity.reference)


def missing_reference_allowed(self, authorization, wall, ros_ns):
    if self.lease is not None and getattr(authorization, 'reference', None) is None:
        return None
    return REAL_VERDICT(self, authorization, wall, ros_ns)


def generation_as_correction(state, lifecycle, localization, inputs_healthy, wall, ros_ns):
    if (lifecycle.active is not None
            and not localization.permits(lifecycle.active.authorization, wall, ros_ns)):
        lifecycle.revoke(lifecycle.active)
        state['correction'] = nl.MAP_CORRECTION_EXCEEDED
        state['enabled'] = False
    return REAL_TICK(state, lifecycle, localization, inputs_healthy, wall, ros_ns)


VARIANTS = {
    'threshold uses > instead of >=': (
        lambda: patch.object(nl, 'correction_exceeds', strict_greater),
        ['ThresholdTests.test_translation_limit_is_inclusive',
         'ThresholdTests.test_yaw_limit_is_inclusive']),
    'threshold comparison reversed': (
        lambda: patch.object(nl, 'correction_exceeds', reversed_comparison),
        ['ThresholdTests.test_well_inside_and_well_outside',
         'GateDrivenRuleTests.test_translation_under_limit_keeps_the_goal']),
    'correction over the limit does not cancel': (
        lambda: patch.object(nl.NavigationLocalization, 'correction_verdict', exceed_without_cancel),
        ['GateDrivenRuleTests.test_translation_over_limit_revokes_and_cancels_once',
         'GateDrivenRuleTests.test_yaw_over_limit_revokes']),
    'yaw difference not folded to the shortest angle': (
        lambda: patch.object(nl, 'shortest_yaw_difference', naive_yaw),
        ['ShortestAngleTests.test_across_plus_minus_180_is_the_short_way']),
    'translation from the map->odom columns, not the tracked point': (
        lambda: patch.object(nl, 'correction_delta', translation_columns),
        ['TranslationPointTests.test_small_turn_about_the_robot_far_from_odom_origin_does_not_exceed',
         'TranslationPointTests.test_small_turn_about_a_far_origin_moves_the_robot_and_exceeds']),
    'only the newest approval is compared': (
        lambda: patch.object(nl.NavigationLocalization, 'correction_verdict', newest_only),
        ['GateDrivenRuleTests.test_transient_jump_inside_the_row_window_is_still_seen']),
    'wrong reference: oldest approval in the row': (
        lambda: patch.object(nl.NavigationLocalization, 'correction_reference', oldest_reference),
        ['ReferenceTests.test_a_correction_before_the_goal_does_not_revoke_it',
         'ReferenceTests.test_reference_is_the_newest_approval_when_the_goal_is_sent']),
    'wrong reference: re-read at check time': (
        lambda: patch.object(nl.NavigationLocalization, 'correction_verdict',
                             reference_read_at_check_time),
        ['GateDrivenRuleTests.test_translation_over_limit_revokes_and_cancels_once',
         'ReferenceTests.test_a_correction_after_the_goal_revokes_it']),
    'correction stamp written into the identity tuple': (
        lambda: patch.object(nl.NavigationLocalization, 'identity', stamp_in_identity),
        ['ReferenceTests.test_identity_is_still_the_four_part_tuple',
         'ReferenceTests.test_new_approvals_under_the_limit_do_not_change_the_identity']),
    'goal without a reference allowed': (
        lambda: patch.object(nl.NavigationLocalization, 'correction_verdict',
                             missing_reference_allowed),
        ['FailClosedTests.test_estimated_goal_without_a_reference_is_revoked_as_a_fault']),
    'generation change treated as a re-issuable correction': (
        lambda: patch.object(rules.NAV_SUITE, 'evaluate_navigation_tick', generation_as_correction),
        ['GateDrivenRuleTests.test_generation_change_keeps_the_old_rule_and_is_not_reissuable']),
}


def run(names):
    loader = unittest.TestLoader()
    suite = unittest.TestSuite(loader.loadTestsFromName(name, rules) for name in names)
    result = unittest.TestResult()
    suite.run(result)
    # Only assertion failures count as caught: an exception inside a variant is a broken
    # variant, not evidence that the test discriminates.
    failed = {test.id().split('.', 1)[1] for test, _ in result.failures}
    return result, failed


class FaultVariantTests(unittest.TestCase):
    def test_every_named_test_passes_on_the_real_implementation(self):
        names = sorted({name for _, expected in VARIANTS.values() for name in expected})
        result, failed = run(names)
        self.assertEqual(result.testsRun, len(names))
        self.assertEqual((failed, result.errors), (set(), []))

    def test_every_variant_is_caught_by_its_named_tests(self):
        for label, (make_patch, expected) in VARIANTS.items():
            with self.subTest(variant=label):
                with make_patch():
                    result, failed = run(expected)
                self.assertEqual(result.testsRun, len(expected))
                self.assertEqual(result.errors, [], label)
                self.assertEqual(failed, set(expected), label)


if __name__ == '__main__':
    unittest.main()
