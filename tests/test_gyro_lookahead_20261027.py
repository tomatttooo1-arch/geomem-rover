"""Gyro look-ahead across empty frames (car run 20261027-104727-live).

The MCU fills the gyro field only every other 0x82 frame. The spike filter decides a frame on the next one;
deciding on an empty frame took every turn start/stop as a spike, the median latched onto the old rate and
the gyro heading kept turning ~33 deg/s after the wheels stopped (or stayed still while turning).
"""
import math
import unittest

from host.chassis_uart import GYRO_RAW_DEG_S
from host.gyro_odom import FusedOdometry


def frames(phases):
    """phases: (seconds, gyro deg/s, encoder counts per frame). 100 Hz; every other frame has no gyro reading."""
    out, ms, enc, k = [], 1000, 0, 0
    for secs, dps, dc in phases:
        for _ in range(int(round(secs * 100))):
            ms += 10
            enc += dc
            valid = k % 2 == 0
            out.append({'t_ns': ms * 1000000, 'mcu_ms': ms, 'enc_a': -enc, 'enc_b': enc, 'enc_c': -enc,
                        'enc_d': enc, 'gz_sum': int(round(dps / GYRO_RAW_DEG_S)) if valid else 0,
                        'gz_n': 1 if valid else 0, 'imu_err': 0, 'ax': 0, 'ay': 0, 'az': 2048,
                        'cmd': [0, 600 if dc else 0, True]})
            k += 1
    return out


class LookAheadAcrossEmptyFrames(unittest.TestCase):
    def run_phases(self, phases):
        f = FusedOdometry()
        yaws = []
        for r in frames(phases):
            f.update(r)
            yaws.append(f.yaw)
        return f, yaws

    def test_turn_then_stop_heading_stops(self):
        f, yaws = self.run_phases([(3.0, 0., 0), (3.0, 35., 4), (3.0, 0., 0)])
        turned = math.degrees(yaws[600] - yaws[300])
        after_stop = math.degrees(yaws[-1] - yaws[600])
        self.assertAlmostEqual(turned, 105., delta=3.)          # 35 deg/s * 3 s
        self.assertLess(abs(after_stop), 1.)                     # no phantom turning after the wheels stop
        self.assertEqual(f.source, 'gyro')

    def test_turn_start_is_not_swallowed(self):
        f, yaws = self.run_phases([(3.0, 0., 0), (1.0, -35., 4)])
        self.assertAlmostEqual(math.degrees(yaws[-1] - yaws[300]), -35., delta=3.)

    def test_single_spike_still_replaced(self):
        """A one-frame spike between normal readings (empty frames between) is still replaced by the median."""
        rows = frames([(4.0, 0., 0)])
        rows[301]['gz_sum'] = 0
        rows[300]['gz_sum'] = int(round(200. / GYRO_RAW_DEG_S))   # frame 300 has a reading (even index)
        f = FusedOdometry()
        for r in rows:
            f.update(r)
        self.assertEqual(f.counters['spikes_replaced'], 1)
        self.assertLess(abs(math.degrees(f.yaw)), 0.5)


if __name__ == '__main__':
    unittest.main()
