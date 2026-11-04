import importlib.util
import json
from pathlib import Path
import unittest

from host.nav_localization import GoalLifecycle, NavigationLocalization

SPEC = importlib.util.spec_from_file_location(
    'nav_suite_localization_test',
    Path(__file__).resolve().parents[1]/'robot_sim/tests/nav_point_to_point.py')
NAV_SUITE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NAV_SUITE)


MAP_ID = 'a' * 64


def pose_covariance():
    covariance = [0.0] * 36
    for index in range(6):
        covariance[index*6 + index] = 0.1
    return covariance


def status(sequence=1, stamp_ns=1_000_000_000, epoch=0, instance='provider-a',
           map_id=MAP_ID, pose_source='estimated', navigation_ready=True):
    pose = {'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
    return {'schema': 1, 'instance': instance, 'map_id': map_id, 'epoch': epoch,
            'sequence': sequence, 'state': 'valid', 'reason': 'test',
            'stamp_ns': stamp_ns, 'clock_ns': stamp_ns, 'world_frame': 'map',
            'tracked_frame': 'camera_link', 'pose_source': pose_source,
            'navigation_ready': navigation_ready, 'odom_source': 'test',
            'approved': [{'stamp_ns': stamp_ns,
                          'pose': {'pose': pose, 'covariance': pose_covariance()},
                          'odom_pose': pose}]}


class NavigationLocalizationTests(unittest.TestCase):
    def test_reference_keeps_legacy_authorization(self):
        policy = NavigationLocalization('reference')
        identity = policy.identity(10.0, 10_000_000_000)
        self.assertEqual(identity, ('reference',))
        self.assertTrue(policy.permits(identity, 999.0, 1))

    def test_estimated_requires_strict_provenance_readiness_and_map(self):
        for change in ({'pose_source': 'reference'}, {'navigation_ready': 1},
                       {'navigation_ready': False}, {'map_id': 'b' * 64}):
            policy = NavigationLocalization('estimated', MAP_ID, MAP_ID)
            row = status()
            row.update(change)
            policy.update_json(json.dumps(row), 1.0, 1_000_000_000)
            self.assertIsNone(policy.identity(1.0, 1_000_000_000), change)
        policy = NavigationLocalization('estimated', MAP_ID, MAP_ID)
        policy.update_json('{bad json', 1.0, 1_000_000_000)
        self.assertIsNone(policy.identity(1.0, 1_000_000_000))

    def test_navigation_identity_survives_approved_window_rollover(self):
        policy = NavigationLocalization('estimated', MAP_ID, MAP_ID)
        first = status()
        policy.update_json(json.dumps(first), 1.0, first['clock_ns'])
        identity = policy.identity(1.0, first['clock_ns'])
        self.assertIsNotNone(identity)
        for sequence in range(2, 35):
            stamp = sequence * 100_000_000 + 1_000_000_000
            row = status(sequence=sequence, stamp_ns=stamp)
            policy.update_json(json.dumps(row), 1.0 + sequence/100,
                               stamp)
        self.assertTrue(policy.permits(identity, 1.35, row['stamp_ns']))

    def test_epoch_or_instance_change_invalidates_navigation_identity(self):
        policy = NavigationLocalization('estimated', MAP_ID, MAP_ID)
        first = status()
        policy.update_json(json.dumps(first), 1.0, first['clock_ns'])
        identity = policy.identity(1.0, first['clock_ns'])
        changed = status(sequence=2, stamp_ns=1_100_000_000, epoch=1)
        policy.update_json(json.dumps(changed), 1.1, changed['clock_ns'])
        self.assertFalse(policy.permits(identity, 1.1, changed['clock_ns']))
        replacement = policy.identity(1.1, changed['clock_ns'])
        restarted = status(sequence=1, stamp_ns=1_200_000_000, epoch=0,
                           instance='provider-b')
        policy.update_json(json.dumps(restarted), 1.2, restarted['clock_ns'])
        self.assertFalse(policy.permits(replacement, 1.2, restarted['clock_ns']))

    def test_epoch_change_revokes_active_goal_and_recovery_cannot_revive_it(self):
        policy = NavigationLocalization('estimated', MAP_ID, MAP_ID)
        first = status()
        policy.update_json(json.dumps(first), 1.0, first['clock_ns'])
        lifecycle = GoalLifecycle()
        context = lifecycle.start(policy.identity(1.0, first['clock_ns']))
        handle = type('Handle', (), {'accepted': True,
                    'cancel_goal_async': lambda self: object()})()
        lifecycle.accepted(context, handle)
        state = {'fault': None, 'enabled': True}

        changed = status(sequence=2, stamp_ns=1_100_000_000, epoch=1)
        policy.update_json(json.dumps(changed), 1.1, changed['clock_ns'])
        self.assertFalse(NAV_SUITE.evaluate_navigation_tick(
            state, lifecycle, policy, True, 1.1, changed['clock_ns']))
        self.assertTrue(context.revoked)

        recovered = status(sequence=3, stamp_ns=1_200_000_000, epoch=1)
        policy.update_json(json.dumps(recovered), 1.2, recovered['clock_ns'])
        state['enabled'] = True
        self.assertFalse(NAV_SUITE.evaluate_navigation_tick(
            state, lifecycle, policy, True, 1.2, recovered['clock_ns']))
        lifecycle.result(context, 5)
        replacement = lifecycle.start(policy.identity(1.2, recovered['clock_ns']))
        self.assertIs(lifecycle.active, replacement)

    def test_lease_timeout_is_latched_by_goal_lifecycle_in_consumer(self):
        policy = NavigationLocalization('estimated', MAP_ID, MAP_ID)
        row = status()
        policy.update_json(json.dumps(row), 1.0, row['clock_ns'])
        identity = policy.identity(1.0, row['clock_ns'])
        self.assertFalse(policy.permits(identity, 1.5, row['clock_ns']))

    def test_expected_database_declarations_must_match(self):
        with self.assertRaisesRegex(ValueError, 'differ'):
            NavigationLocalization('estimated', MAP_ID, 'b' * 64)
        with self.assertRaisesRegex(ValueError, 'requires'):
            NavigationLocalization('estimated', MAP_ID, None)
