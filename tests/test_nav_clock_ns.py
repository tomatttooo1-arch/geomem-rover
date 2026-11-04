"""The point-to-point consumer's /clock-to-nanosecond conversion must round, not truncate.

Found by the 2026-09-27 server container smoke (docs/design/NAV_CORRECTION_RULES_20260927.md
§7.1): with int(), an /odom stamp taken at the same simulation step as /clock read as 1 ns
in the future, health failed during the action and the first case aborted.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'nav_suite_clock_ns_test', Path(__file__).resolve().parents[1]/'robot_sim/tests/nav_point_to_point.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def health_at_same_step(clock_s, stamp_ns):
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9)),
        transform=SimpleNamespace(translation=SimpleNamespace(x=0., y=0.),
                                  rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    buffer = SimpleNamespace(lookup_transform=lambda parent, child, stamp: transform)
    state = {'fault': None, 'clock': clock_s, 'clock_wall': 20.0, 'pose': None}
    return m.evaluate_health(state, {'/odom': clock_s}, {'/odom': 20.0}, {'/odom': object()}, 20.1,
                             buffer, lambda nanoseconds: SimpleNamespace(nanoseconds=nanoseconds),
                             m.NavigationLocalization('reference'), {'/odom': stamp_ns})


class ClockNanosecondTests(unittest.TestCase):
    def test_every_millisecond_clock_converts_exactly(self):
        for ms in range(1, 100000):
            self.assertEqual(m.clock_ns({'clock': ms / 1000}), ms * 10**6)

    def test_odom_stamped_at_the_clock_step_is_healthy(self):
        self.assertTrue(health_at_same_step(8.466, 8_466_000_000))

    def test_fault_variant_truncation_is_caught(self):
        with patch.object(m, 'clock_ns', lambda state: int(state['clock'] * 1e9)):
            self.assertFalse(health_at_same_step(8.466, 8_466_000_000))
            self.assertNotEqual(m.clock_ns({'clock': 8.466}), 8_466_000_000)

    def test_no_int_truncation_of_the_clock_remains(self):
        source = Path(m.__file__).read_text()
        self.assertNotIn("int(state['clock']*1e9)", source)


if __name__ == '__main__':
    unittest.main()
