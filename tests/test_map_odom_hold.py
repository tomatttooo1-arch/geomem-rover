"""Held world->odom republish decisions and per-approval TF edge selection.

Pure policy with an injected clock: this host has no ROS, so the node itself cannot run here.
The wiring tests below read the node source to pin that it calls exactly these decisions
instead of re-deciding inline, which is what keeps them from being decorative.
"""
import inspect
import re
import unittest

import numpy as np

from host.localization_gate import (MapOdomHold, ODOM_TRACKED_EDGE, WORLD_ODOM_EDGE,
                                    approval_edges)
import host.ros_localization_gate as node

FUTURE_NS = 10**8
NOW = 1500000000


def matrix(x=1.):
    result = np.eye(4)
    result[0, 3] = x
    return result


class MapOdomHoldTests(unittest.TestCase):
    def hold(self, future_ns=FUTURE_NS):
        return MapOdomHold(future_ns)

    def test_nothing_is_published_before_the_first_approval(self):
        hold = self.hold()
        self.assertIsNone(hold.tick('valid', NOW))
        self.assertIsNone(hold.tick('valid', NOW + 10**8))

    def test_valid_state_publishes_the_last_approved_matrix_at_a_future_stamp(self):
        hold = self.hold()
        hold.update(matrix(2.))
        stamp, published = hold.tick('valid', NOW)
        self.assertEqual(stamp, NOW + FUTURE_NS)
        self.assertIs(type(stamp), int)
        np.testing.assert_array_equal(published, matrix(2.))
        hold.update(matrix(7.))
        stamp, published = hold.tick('valid', NOW + 2*10**8)
        self.assertEqual(stamp, NOW + 2*10**8 + FUTURE_NS)
        np.testing.assert_array_equal(published, matrix(7.))

    def test_the_future_offset_is_exact_in_integer_nanoseconds(self):
        for future_ns in (0, 1, FUTURE_NS, 5*10**8):
            hold = self.hold(future_ns)
            hold.update(matrix())
            self.assertEqual(hold.tick('valid', NOW)[0], NOW + future_ns)

    def test_a_repeated_stamp_is_never_published_twice(self):
        # A 20 Hz timer under a 200 ms stepped /clock must emit one transform per step.
        hold = self.hold()
        hold.update(matrix())
        self.assertIsNotNone(hold.tick('valid', NOW))
        self.assertIsNone(hold.tick('valid', NOW))
        hold.update(matrix(3.))
        self.assertIsNone(hold.tick('valid', NOW))
        self.assertIsNotNone(hold.tick('valid', NOW + 2*10**8))

    def test_states_other_than_valid_never_publish(self):
        for state in ('initializing', 'confirming', 'lost', 'stale'):
            hold = self.hold()
            hold.update(matrix())
            self.assertIsNone(hold.tick(state, NOW), state)

    def test_leaving_valid_retires_the_edge_until_a_new_approval(self):
        hold = self.hold()
        hold.update(matrix())
        self.assertIsNotNone(hold.tick('valid', NOW))
        self.assertIsNone(hold.tick('lost', NOW + 2*10**8))
        # Returning to valid on its own republishes nothing: the old sample stays retired.
        self.assertIsNone(hold.tick('valid', NOW + 4*10**8))
        self.assertIsNone(hold.tick('valid', NOW + 6*10**8))
        hold.update(matrix(5.))
        stamp, published = hold.tick('valid', NOW + 8*10**8)
        self.assertEqual(stamp, NOW + 8*10**8 + FUTURE_NS)
        np.testing.assert_array_equal(published, matrix(5.))

    def test_the_held_stamp_is_clock_time_and_never_the_acquisition_stamp(self):
        # Negative control for republishing with the image stamp of the approval.
        image_stamp = NOW
        hold = self.hold()
        hold.update(matrix())
        stamp, _ = hold.tick('valid', image_stamp + 3*10**8)
        self.assertNotEqual(stamp, image_stamp)
        self.assertGreater(stamp, image_stamp)

    def test_the_stored_matrix_is_a_copy_of_the_caller_argument(self):
        hold = self.hold()
        supplied = matrix(4.)
        hold.update(supplied)
        supplied[0, 3] = 99.
        _, published = hold.tick('valid', NOW)
        self.assertEqual(published[0, 3], 4.)
        published[1, 3] = 42.
        self.assertEqual(hold.tick('valid', NOW + 2*10**8)[1][1, 3], 0.)

    def test_invalid_offsets_clocks_and_matrices_are_rejected(self):
        for bad in (-1, .1, None, True):
            with self.assertRaises(ValueError):
                MapOdomHold(bad)
        hold = self.hold()
        for bad in (np.eye(3), np.full((4, 4), np.nan), np.full((4, 4), np.inf)):
            with self.assertRaises(ValueError):
                hold.update(bad)
        hold.update(matrix())
        for bad in (-1, 1.5, None):
            with self.assertRaises(ValueError):
                hold.tick('valid', bad)


class ApprovalEdgeTests(unittest.TestCase):
    def test_edges_follow_the_tracked_frame_switch_and_the_hold(self):
        self.assertEqual(approval_edges(True, False), [WORLD_ODOM_EDGE, ODOM_TRACKED_EDGE])
        self.assertEqual(approval_edges(False, False), [WORLD_ODOM_EDGE])
        # Held world->odom leaves the approval message, so only the tracked edge remains.
        self.assertEqual(approval_edges(True, True), [ODOM_TRACKED_EDGE])
        self.assertEqual(approval_edges(False, True), [])

    def test_the_joint_profile_publishes_exactly_one_transform_per_approval(self):
        self.assertEqual(len(approval_edges(False, False)), 1)
        self.assertEqual(len(approval_edges(True, True)), 1)
        self.assertEqual(len(approval_edges(True, False)), 2)


class NodeWiringTests(unittest.TestCase):
    """No ROS here, so pin that the node delegates to the tested decisions."""

    source = inspect.getsource(node.main)

    def test_the_node_selects_approval_edges_with_the_tested_helper(self):
        self.assertRegex(self.source,
                         r'approval_edges\(\s*args\.publish_tracked_frame\s*,[^)]*hold[^)]*\)')
        self.assertNotIn("('map', 'odom'", self.source)

    def test_the_node_publishes_the_held_edge_only_through_the_tested_class(self):
        self.assertRegex(self.source, r'self\.hold\.tick\(\s*self\.gate\.state\s*,')
        self.assertRegex(self.source, r'self\.hold\.update\(')
        self.assertRegex(self.source, r'MapOdomHold\(\s*int\(')
        # An empty TFMessage is never published.
        self.assertIn('if transforms:', self.source)

    def test_the_frame_names_are_arguments_everywhere_they_are_published(self):
        self.assertNotRegex(self.source, r"frame_id\s*=\s*'map'")
        self.assertNotIn("('odom', args.tracked_frame", self.source)
        self.assertIn('msg.header.frame_id = args.world_frame', self.source)
        self.assertGreaterEqual(len(re.findall(r'args\.world_frame', self.source)), 4)
        self.assertGreaterEqual(len(re.findall(r'args\.odom_frame', self.source)), 4)

    def test_numpy_is_imported_once_at_module_scope(self):
        self.assertNotIn('import numpy', self.source)
        self.assertIn('\nimport numpy as np\n', inspect.getsource(node))


if __name__ == '__main__':
    unittest.main()
