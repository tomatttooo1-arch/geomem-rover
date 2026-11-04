"""Behavior checks for the lease extracted from pinned upstream 8f090d4."""
import unittest

from host.localization_gate import LocalizationLease


def row(sequence=1, stamp=1_000_000_000):
    covariance = [0.0] * 36
    pose = {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}
    return {'schema': 1, 'instance': 'provider', 'map_id': 'map', 'epoch': 0,
            'sequence': sequence, 'state': 'valid', 'stamp_ns': stamp,
            'clock_ns': stamp, 'world_frame': 'map', 'tracked_frame': 'camera_link',
            'approved': [{'stamp_ns': stamp,
                          'pose': {'pose': pose, 'covariance': covariance}}]}


class PinnedLeaseBehaviorTests(unittest.TestCase):
    def test_heartbeat_and_progress_deadlines_are_strict(self):
        lease = LocalizationLease('map')
        lease.update(row(), 1.0, 1_000_000_000)
        self.assertTrue(lease.check(1.499, 1_999_999_999))
        self.assertFalse(lease.check(1.5, 2_000_000_000))

    def test_bool_sequence_is_rejected_as_non_integer_identity(self):
        lease = LocalizationLease('map')
        malformed = row()
        malformed['sequence'] = True
        lease.update(malformed, 1.0, 1_000_000_000)
        self.assertIsNone(lease.row)
