"""Exact-time TF with a bounded wait (controller decision 2026-09-27 after the N1 smoke).

/odom may arrive before the odom->base_link TF of the same stamp. The consumer looks the
transform up only at the admitted /odom stamp; until it arrives, for at most one wait
(default 1.5 periods of the 30 Hz simulated TF), the last released exact pair stands.
After the wait the input is unhealthy. Never pair the newest stamp with an older transform.
The variant class swaps in the two forbidden shapes and requires named tests to fail.
"""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'nav_suite_tf_wait_test', Path(__file__).resolve().parents[1]/'robot_sim/tests/nav_point_to_point.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)

A = 9_900_000_000                     # released sample
B = A + 33_333_333                    # next 30 Hz sample, TF not yet arrived
CLOCK = 10.0
NOW = 20.0


def transform(stamp_ns, x):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9)),
        transform=SimpleNamespace(translation=SimpleNamespace(x=x, y=0.),
                                  rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))


class Buffer:
    """Holds TF by exact stamp; nanoseconds=0 means 'latest', as tf2 does."""

    def __init__(self, available):
        self.available, self.queries = dict(available), []

    def lookup_transform(self, parent, child, stamp):
        self.queries.append(stamp.nanoseconds)
        if stamp.nanoseconds == 0 and self.available:
            return self.available[max(self.available)]
        if stamp.nanoseconds not in self.available:
            raise RuntimeError('Lookup would require extrapolation into the future')
        return self.available[stamp.nanoseconds]


def health(state, buffer, stamp_ns, receipt_age):
    return m.evaluate_health(state, {'/odom': stamp_ns / 1e9}, {'/odom': NOW - receipt_age},
                             {'/odom': object()}, NOW, buffer,
                             lambda nanoseconds: SimpleNamespace(nanoseconds=nanoseconds),
                             m.NavigationLocalization('reference'), {'/odom': stamp_ns})


def released_state():
    state = {'fault': None, 'clock': CLOCK, 'clock_wall': NOW, 'pose': None}
    assert health(state, Buffer({A: transform(A, 1.0)}), A, .01)
    return state


class TfWaitTests(unittest.TestCase):
    def test_default_wait_is_one_and_a_half_simulated_tf_periods(self):
        self.assertAlmostEqual(m.TF_WAIT_S, 1.5 / 30)
        args, _ = m.parse_arguments(['--out', 'unused.jsonl'])
        self.assertEqual(args.tf_wait_s, m.TF_WAIT_S)
        for bad in ('0', '-0.01', '0.3', 'nan'):
            with self.subTest(bad=bad), self.assertRaises(SystemExit):
                m.parse_arguments(['--out', 'unused.jsonl', '--tf-wait-s', bad])

    def test_within_the_wait_the_released_pair_stands(self):
        state = released_state()
        buffer = Buffer({A: transform(A, 1.0), B + 10**9: transform(B + 10**9, 9.0)})
        self.assertTrue(health(state, buffer, B, .02))
        self.assertEqual(state['pose'][0], 1.0)            # still the exact pair at A
        self.assertEqual(state['tf_released_ns'], A)

    def test_after_the_wait_the_input_is_unhealthy(self):
        state = released_state()
        buffer = Buffer({A: transform(A, 1.0), A - 1: transform(A - 1, 7.0)})
        self.assertFalse(health(state, buffer, B, m.TF_WAIT_S + .001))
        self.assertEqual(state['pose'][0], 1.0)

    def test_two_missing_periods_exceed_the_wait_even_if_just_received(self):
        state = released_state()
        later = A + 2 * 33_333_333
        self.assertFalse(health(state, Buffer({A: transform(A, 1.0)}), later, 0.))

    def test_no_released_pair_means_no_wait(self):
        state = {'fault': None, 'clock': CLOCK, 'clock_wall': NOW, 'pose': None}
        self.assertFalse(health(state, Buffer({}), B, 0.))

    def test_arrival_releases_the_new_exact_pair(self):
        state = released_state()
        self.assertTrue(health(state, Buffer({A: transform(A, 1.0), B: transform(B, 2.0)}), B, .02))
        self.assertEqual((state['pose'][0], state['tf_released_ns']), (2.0, B))

    def test_transform_is_only_ever_looked_up_at_an_odom_stamp(self):
        state = released_state()
        state['odom_stamps'] = [A, B]
        buffer = Buffer({A: transform(A, 1.0)})
        for age in (.0, .02, .049, .06):
            health(state, buffer, B, age)
        self.assertTrue(buffer.queries)
        self.assertLessEqual(set(buffer.queries), {A, B})

    def test_an_unticked_earlier_sample_with_its_tf_is_released(self):
        # Ticks (20 Hz) need not see every 30 Hz sample: A was released, M arrived with
        # its TF between ticks, B (newest) still waits for its TF.
        state = released_state()
        M = A + 33_333_333
        newest = M + 33_333_333
        state['odom_stamps'] = [A, M, newest]
        buffer = Buffer({A: transform(A, 1.0), M: transform(M, 1.5)})
        self.assertTrue(health(state, buffer, newest, .01))
        self.assertEqual((state['pose'][0], state['tf_released_ns']), (1.5, M))

    def test_a_stopped_tf_stream_fails_even_while_odom_keeps_coming(self):
        state = released_state()
        M = A + 33_333_333
        newest = M + 33_333_333
        state['odom_stamps'] = [A, M, newest]
        self.assertFalse(health(state, Buffer({A: transform(A, 1.0)}), newest, .01))


def previous_transform(state, buffer, time_type, stamp_ns, ros_ns, receipt_age_s, tf_wait_s):
    """Forbidden: pair the newest stamp with the latest transform tf2 has."""
    try:
        found = buffer.lookup_transform('map', 'base_link', time_type(nanoseconds=stamp_ns))
    except Exception:
        found = buffer.lookup_transform('map', 'base_link', time_type(nanoseconds=0))
    state['pose'] = m.planar_pose(found)
    return True


def no_wait(state, buffer, time_type, stamp_ns, ros_ns, receipt_age_s, tf_wait_s):
    """Forbidden: fail on the first miss, which aborted the 2026-09-27 smoke."""
    try:
        found = buffer.lookup_transform('map', 'base_link', time_type(nanoseconds=stamp_ns))
    except Exception:
        return False
    state['pose'] = m.planar_pose(found)
    state['tf_released_ns'] = stamp_ns
    return True


class TfWaitVariantTests(unittest.TestCase):
    CASES = {
        'previous transform used': (previous_transform, {
            'test_after_the_wait_the_input_is_unhealthy',
            'test_two_missing_periods_exceed_the_wait_even_if_just_received',
            'test_transform_is_only_ever_looked_up_at_an_odom_stamp',
            'test_a_stopped_tf_stream_fails_even_while_odom_keeps_coming'}),
        'no wait': (no_wait, {'test_within_the_wait_the_released_pair_stands',
                              'test_an_unticked_earlier_sample_with_its_tf_is_released'}),
    }

    def run_named(self, names):
        suite = unittest.TestSuite(TfWaitTests(name) for name in sorted(names))
        result = unittest.TestResult()
        suite.run(result)
        return result, {test._testMethodName for test, _ in result.failures}

    def test_control_passes(self):
        names = set().union(*(names for _, names in self.CASES.values()))
        result, failed = self.run_named(names)
        self.assertEqual((failed, result.errors), (set(), []))

    def test_each_forbidden_shape_is_caught(self):
        for label, (variant, names) in self.CASES.items():
            with self.subTest(variant=label):
                with patch.object(m, 'exact_map_pose', variant):
                    result, failed = self.run_named(names)
                self.assertEqual(result.errors, [], label)
                self.assertEqual(failed, names, label)


if __name__ == '__main__':
    unittest.main()
