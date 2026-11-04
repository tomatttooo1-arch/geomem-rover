"""D-035 in the M5 Runtime's estimated navigation (host/ros_runtime.py).

The Runtime binds the newest approval's map->odom to each goal and applies the SAME rule
and limits as the point-to-point consumer (host.nav_localization.lease_correction_*).
Within an unchanged authorization triple, a correction past the limits fails the task
closed with reason map_correction_exceeded: permit false, one cancel. Re-planning on the
new map is the caller's new query (M5_HANDOFF §9.1); the Runtime does not re-send itself.

Driven through the source-loaded production Runtime of tools/rtabmap_runtime_behavior_probe
(real LocalizationGate statuses, deterministic action/ROS stand-ins). That loader needs
Python 3.9+ (D-028); on 3.8 each test instead requires the declared refusal.
"""
import sys
import tempfile
import unittest
from unittest.mock import patch

from host import nav_localization as nl
from host.localization_gate import KINDS
from tools.rtabmap_frame_disposition_probe import observation
from tools.rtabmap_runtime_probe_support import REQUIRED_PYTHON

SUPPORTED = sys.version_info >= REQUIRED_PYTHON


def corrected_status(session, stamp, dx):
    """One real gate acquisition whose localization pose is moved dx metres on the map."""
    gate, world = session.gate, session.w
    world.clock(world.wall + .1, stamp)
    gate.acquisition(stamp, world.wall, world.ros_ns)
    for kind in KINDS:
        row = observation(kind, stamp)
        if kind == 'localization_pose':
            row['pose']['pose']['position']['x'] += dx
        gate.ingest(kind, row, world.wall, world.ros_ns)
    return gate.status(world.wall, world.ros_ns)


class M5RuntimeCorrectionRuleTests(unittest.TestCase):
    def navigating(self):
        """A Runtime navigating under a valid estimated lease, or None on 3.8 after the refusal."""
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        from tools.rtabmap_runtime_behavior_probe import Session
        if not SUPPORTED:
            with self.assertRaisesRegex(RuntimeError, r'requires Python 3\.9\+'):
                Session(temporary.name, 'correction', 'original')
            return None
        session = Session(temporary.name, 'correction', 'original')
        self.addCleanup(session.n.close)
        session.warm(navigation=True)
        return session

    def faults(self, session):
        return [e['payload']['reason'] for e in session.w.events
                if e['event'] == 'publish' and e['topic'] == '/fpga/status'
                and e['payload'].get('event') == 'fault']

    def cancels(self, session):
        return [e for e in session.w.events if e['event'] == 'action_cancel_request']

    def test_goal_carries_the_newest_approval_as_reference(self):
        session = self.navigating()
        if session is None:
            return
        reference = session.n.nav['correction_reference']
        self.assertEqual(reference.stamp_ns, session.n.localization.row['correction_stamp_ns'])
        self.assertEqual(session.n.nav['authorization'],
                         (session.n.localization.generation, 'fixture-A', 0))

    def test_correction_over_the_limit_fails_closed_and_cancels_once(self):
        session = self.navigating()
        if session is None:
            return
        session.status(corrected_status(session, 1_400_000_000, .2))
        self.assertEqual(self.faults(session), [nl.MAP_CORRECTION_EXCEEDED])
        self.assertIs(session.w.publisher_values['/fpga/motion_permit'], False)
        self.assertTrue(session.n.nav['cancelled'])
        self.assertEqual(len(self.cancels(session)), 1)
        # The authorization triple did not change: this is the correction rule, not a
        # continuity loss.
        self.assertEqual(session.n.nav['authorization'][2], session.n.localization.row['epoch'])

    def test_correction_under_the_limit_keeps_navigating(self):
        session = self.navigating()
        if session is None:
            return
        session.status(corrected_status(session, 1_400_000_000, .05))
        session.w.poll('after_small_correction')
        self.assertEqual(self.faults(session), [])
        self.assertFalse(session.n.nav['cancelled'])
        self.assertEqual(self.cancels(session), [])

    def test_the_runtime_uses_the_shared_rule_not_its_own(self):
        # Fault variant: the shared limit function says "never exceeds". If the Runtime
        # carried its own copy of the rule, it would still fault here.
        session = self.navigating()
        if session is None:
            return
        with patch.object(nl, 'correction_exceeds', lambda translation, yaw: False):
            session.status(corrected_status(session, 1_400_000_000, .2))
        self.assertEqual(self.faults(session), [])
        self.assertFalse(session.n.nav['cancelled'])


if __name__ == '__main__':
    unittest.main()
