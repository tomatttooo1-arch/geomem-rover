"""Known answers and negative controls for the camera->base odometry adapter.

This host has no ROS, so the node itself cannot run here. Everything that decides anything
lives in host.odom_base_adapter and is exercised directly; the node is pinned by reading its
source, so the wiring tests cannot become decorative.

The negative controls matter more than the positive ones. Each of the three plausible sign or
order errors is implemented here and asserted to produce a large error on the same trajectory,
which is what proves the positive test could have failed. They are run against a real extrinsic
with pitch 0.25 and against a 90 degree yaw variant, because with an identity rotation one of
the three wrong implementations agrees with the right one (documented in IdentityBlindnessTests).
"""
import hashlib
import inspect
import json
from pathlib import Path
import re
import tempfile
import unittest

import numpy as np

from host.odom_base_adapter import (adjoint, extrinsics_sha256, load_extrinsics_manifest,
                                    matrix_from_xyz_rpy, OdomBaseAdapter, skew,
                                    transform_covariance)
import host.ros_odom_base_adapter as node

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT/'robot_sim/fpga_sim/config/extrinsics_sim.json'
LAUNCH = ROOT/'robot_sim/fpga_sim/launch/navigation.launch.py'
WORLD = ROOT/'robot_sim/fpga_sim/worlds/room_carmount.sdf'   # default world since line D1 (2026-10-14)

SIM_XYZ, SIM_RPY = (.18, 0., .95), (0., .25, 0.)
# Non-identity rotations: the pitched simulation extrinsic and a 90 degree yaw variant.
SIM = matrix_from_xyz_rpy(SIM_XYZ, SIM_RPY)
# The simulation manifest (extrinsics_sim.json) since line D1 (2026-10-14): the car mount of
# room_carmount.sdf. SIM above stays the algebra fixture of the twist tests (the old mast values).
MANIFEST_XYZ, MANIFEST_RPY = (.13, 0., .35), (0., .471238898, 0.)
MANIFEST_MATRIX = matrix_from_xyz_rpy(MANIFEST_XYZ, MANIFEST_RPY)
YAW90 = matrix_from_xyz_rpy(SIM_XYZ, (0., 0., np.pi/2))
TRANSLATION_ONLY = matrix_from_xyz_rpy(SIM_XYZ, (0., 0., 0.))
# Both halves non-zero so that a misplaced [t]x block cannot hide in a zero input.
BODY_TWIST = np.array([.3, -.1, .05, .02, -.05, .4])
TIMES = (0., .1, .2, .3, .4, .5)
BASE = 1700000000 * 10**9


def se3_exp(xi):
    """Independent closed-form SE(3) exponential in (v, w) order; shares no code with the module."""
    v, w = np.asarray(xi[:3], float), np.asarray(xi[3:], float)
    angle, cross = np.linalg.norm(w), skew(w)
    if angle < 1e-12:
        rotation, jacobian = np.eye(3)+cross, np.eye(3)+cross/2.
    else:
        rotation = (np.eye(3) + np.sin(angle)/angle*cross
                    + (1-np.cos(angle))/angle**2 * cross @ cross)
        jacobian = (np.eye(3) + (1-np.cos(angle))/angle**2*cross
                    + (angle-np.sin(angle))/angle**3 * cross @ cross)
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation, jacobian @ v
    return result


def start_pose():
    return se3_exp([1.5, -.4, .2, .1, .2, -.7])


def truth(time, start=None, twist=BODY_TWIST):
    """T_ob(t) = T_ob(0) exp(t xi_b): constant body twist, so the answer is closed form."""
    return (start_pose() if start is None else start) @ se3_exp(np.asarray(twist, float)*time)


def quaternion(matrix):
    """Matrix -> (x, y, z, w) without scipy, so the fixture shares no code with the module."""
    m = np.asarray(matrix)[:3, :3]
    w = np.sqrt(max(0., 1.+m[0, 0]+m[1, 1]+m[2, 2]))/2.
    if w > 1e-6:
        return [(m[2, 1]-m[1, 2])/(4*w), (m[0, 2]-m[2, 0])/(4*w), (m[1, 0]-m[0, 1])/(4*w), w]
    x = np.sqrt(max(0., 1.+m[0, 0]-m[1, 1]-m[2, 2]))/2.
    return [x, (m[0, 1]+m[1, 0])/(4*x), (m[0, 2]+m[2, 0])/(4*x), (m[2, 1]-m[1, 2])/(4*x)]


def odometry_row(matrix, twist, stamp_ns=BASE, frame_id='odom', child_frame_id='camera_link',
                 pose_covariance=None, twist_covariance=None):
    """The message_to_ordereddict shape of nav_msgs/Odometry."""
    sec, nanosec = divmod(int(stamp_ns), 10**9)
    twist = np.asarray(twist, float)
    return {'header': {'stamp': {'sec': sec, 'nanosec': nanosec}, 'frame_id': frame_id},
            'child_frame_id': child_frame_id,
            'pose': {'pose': {'position': dict(zip(('x', 'y', 'z'),
                                                   [float(v) for v in np.asarray(matrix)[:3, 3]])),
                              'orientation': dict(zip(('x', 'y', 'z', 'w'), quaternion(matrix)))},
                     'covariance': ([0.]*36 if pose_covariance is None
                                    else [float(v) for v in pose_covariance])},
            'twist': {'twist': {'linear': dict(zip(('x', 'y', 'z'), [float(v) for v in twist[:3]])),
                                'angular': dict(zip(('x', 'y', 'z'), [float(v) for v in twist[3:]]))},
                      'covariance': ([0.]*36 if twist_covariance is None
                                     else [float(v) for v in twist_covariance])}}


def camera_input(time, extrinsic, start=None, body=BODY_TWIST):
    """What RTAB-Map would publish: T_oc = T_ob T_bc and the camera-frame body twist."""
    base = truth(time, start, body)
    camera_twist = adjoint(np.linalg.inv(extrinsic)) @ np.asarray(body, float)
    return base, base @ extrinsic, camera_twist


def adapter(extrinsic=SIM, **kwargs):
    kwargs.setdefault('extrinsics_provenance', 'modelled_sim')
    return OdomBaseAdapter(extrinsic, **kwargs)


def output_pose(result):
    return result['tf']['matrix']


def output_twist(result):
    twist = result['odom']['twist']
    return np.array([twist[part][k] for part in ('linear', 'angular') for k in ('x', 'y', 'z')])


def deviation(left, right):
    return float(np.max(np.abs(np.asarray(left, float) - np.asarray(right, float))))


class KnownAnswerTests(unittest.TestCase):
    """The one twist answer that can be checked by hand."""

    def test_a_pure_yaw_at_the_camera_becomes_a_lever_arm_velocity_at_the_base(self):
        # R = I, t = (0.18, 0, 0.95), w_c = (0, 0, 1), v_c = 0.
        # v_b = t x (R w_c) = (0.18, 0, 0.95) x (0, 0, 1) = (0, -0.18, 0).
        result = adapter(TRANSLATION_ONLY).process(
            odometry_row(np.eye(4), [0., 0., 0., 0., 0., 1.]))
        np.testing.assert_allclose(output_twist(result), [0., -.18, 0., 0., 0., 1.], atol=1e-15)

    def test_the_hand_computed_answer_is_not_what_an_uncorrected_relay_would_give(self):
        # Discriminating check on the known answer itself: a relay that ignored the lever arm
        # would report v_b = 0, so the assertion above is capable of failing.
        result = adapter(TRANSLATION_ONLY).process(
            odometry_row(np.eye(4), [0., 0., 0., 0., 0., 1.]))
        self.assertGreater(abs(output_twist(result)[1]), .1)

    def test_the_pose_divides_the_extrinsic_out(self):
        base = truth(.3)
        result = adapter(SIM).process(odometry_row(base @ SIM, np.zeros(6)))
        self.assertLess(deviation(output_pose(result), base), 1e-12)


class TrajectoryRecoveryTests(unittest.TestCase):
    """A constant body twist trajectory is fed through the camera frame and must come back."""

    def recover(self, extrinsic, times=TIMES, body=BODY_TWIST, start=None):
        instance = adapter(extrinsic)
        rows = []
        for index, time in enumerate(times):
            base, camera, camera_twist = camera_input(time, extrinsic, start, body)
            result = instance.process(odometry_row(camera, camera_twist, BASE+index*10**8))
            self.assertIsNotNone(result)
            rows.append((base, result))
        return instance, rows

    def test_the_base_pose_is_recovered_exactly(self):
        for name, extrinsic in (('pitch 0.25', SIM), ('yaw 90', YAW90),
                                ('identity rotation', TRANSLATION_ONLY)):
            with self.subTest(name):
                for base, result in self.recover(extrinsic)[1]:
                    self.assertLess(deviation(output_pose(result), base), 1e-12)

    def test_the_body_twist_is_recovered_exactly(self):
        for name, extrinsic in (('pitch 0.25', SIM), ('yaw 90', YAW90)):
            with self.subTest(name):
                for _, result in self.recover(extrinsic)[1]:
                    self.assertLess(deviation(output_twist(result), BODY_TWIST), 1e-12)

    def test_the_published_twist_is_the_derivative_of_the_published_pose(self):
        # Central difference of the recovered pose against the reported twist: this catches a
        # pose and a twist that are each self-consistent but not consistent with each other.
        step, time = 1e-4, .25
        for name, extrinsic in (('pitch 0.25', SIM), ('yaw 90', YAW90)):
            with self.subTest(name):
                _, rows = self.recover(extrinsic, times=(time-step, time, time+step))
                before, here, after = [output_pose(result) for _, result in rows]
                rotation = here[:3, :3]
                linear = rotation.T @ (after[:3, 3]-before[:3, 3])/(2*step)
                derivative = rotation.T @ (after[:3, :3]-before[:3, :3])/(2*step)
                derivative = (derivative-derivative.T)/2.
                angular = [derivative[2, 1], derivative[0, 2], derivative[1, 0]]
                self.assertLess(deviation(np.concatenate([linear, angular]),
                                          output_twist(rows[1][1])), 1e-6)


class WrongRecipes:
    """The deliberately wrong implementations, kept out of any TestCase so they run once each.

    Each control is asserted separately for the pose and for the twist. Combining them would
    let a correct twist mask a wrong pose inside the same max(), which is exactly the kind of
    masking that makes a negative control decorative.
    """

    threshold = .1

    def errors(self, extrinsic, wrong_pose=None, wrong_twist=None, times=TIMES,
               body=BODY_TWIST, start=None):
        """Largest deviation between the adapter output and a deliberately wrong recipe."""
        instance = adapter(extrinsic)
        worst = 0.
        for index, time in enumerate(times):
            base, camera, camera_twist = camera_input(time, extrinsic, start, body)
            result = instance.process(odometry_row(camera, camera_twist, BASE+index*10**8))
            if wrong_pose is not None:
                worst = max(worst, deviation(output_pose(result), wrong_pose(camera, extrinsic)))
            if wrong_twist is not None:
                worst = max(worst, deviation(output_twist(result),
                                             wrong_twist(camera_twist, extrinsic)))
        return worst

    @staticmethod
    def inverted_direction(camera, extrinsic):
        """T_oc . T_bc instead of T_oc . inv(T_bc): the extrinsic added instead of removed."""
        return camera @ extrinsic

    @staticmethod
    def swapped_order(camera, extrinsic):
        """inv(T_bc) . T_oc: right factor, wrong side."""
        return np.linalg.inv(extrinsic) @ camera

    @staticmethod
    def inverted_adjoint(camera_twist, extrinsic):
        """Ad of the inverse extrinsic applied to the twist."""
        return adjoint(np.linalg.inv(extrinsic)) @ np.asarray(camera_twist, float)

    @staticmethod
    def misplaced_skew(camera_twist, extrinsic):
        """[t]x R written into the angular row instead of the linear row of the adjoint."""
        rotation, translation = extrinsic[:3, :3], extrinsic[:3, 3]
        wrong = np.zeros((6, 6))
        wrong[:3, :3] = rotation
        wrong[3:, :3] = skew(translation) @ rotation
        wrong[3:, 3:] = rotation
        return wrong @ np.asarray(camera_twist, float)


class NegativeControlTests(WrongRecipes, unittest.TestCase):
    """Each plausible wrong implementation must be far off on the same trajectory."""

    def test_applying_the_extrinsic_to_the_pose_in_the_wrong_direction_is_caught(self):
        for name, extrinsic in (('pitch 0.25', SIM), ('yaw 90', YAW90)):
            with self.subTest(name):
                self.assertGreater(self.errors(extrinsic, wrong_pose=self.inverted_direction),
                                   self.threshold)

    def test_applying_the_adjoint_to_the_twist_in_the_wrong_direction_is_caught(self):
        for name, extrinsic in (('pitch 0.25', SIM), ('yaw 90', YAW90)):
            with self.subTest(name):
                self.assertGreater(self.errors(extrinsic, wrong_twist=self.inverted_adjoint),
                                   self.threshold)

    def test_swapping_the_multiplication_order_is_caught(self):
        for name, extrinsic in (('pitch 0.25', SIM), ('yaw 90', YAW90)):
            with self.subTest(name):
                self.assertGreater(self.errors(extrinsic, wrong_pose=self.swapped_order),
                                   self.threshold)

    def test_the_skew_block_in_the_wrong_corner_of_the_adjoint_is_caught(self):
        for name, extrinsic in (('pitch 0.25', SIM), ('yaw 90', YAW90)):
            with self.subTest(name):
                self.assertGreater(self.errors(extrinsic, wrong_twist=self.misplaced_skew),
                                   self.threshold)


class IdentityBlindnessTests(WrongRecipes, unittest.TestCase):
    """Why the negative controls above must not use an identity rotation.

    With R = I the swapped multiplication order leaves the orientation untouched at every pose,
    and on a trajectory that does not turn it is invisible in the translation as well. A suite
    that only ever used an axis-aligned extrinsic would therefore pass with that error in place,
    which is exactly what the pitched and yawed extrinsics above rule out.
    """

    straight = np.array([.3, -.1, .05, 0., 0., 0.])

    def test_with_an_identity_rotation_the_orientation_never_reveals_the_swap(self):
        instance = adapter(TRANSLATION_ONLY)
        for index, time in enumerate(TIMES):
            _, camera, camera_twist = camera_input(time, TRANSLATION_ONLY)
            result = instance.process(odometry_row(camera, camera_twist, BASE+index*10**8))
            wrong = self.swapped_order(camera, TRANSLATION_ONLY)
            self.assertLess(deviation(output_pose(result)[:3, :3], wrong[:3, :3]), 1e-14)

    def test_with_an_identity_rotation_and_no_turning_the_swap_is_completely_invisible(self):
        self.assertLess(self.errors(TRANSLATION_ONLY, wrong_pose=self.swapped_order,
                                    body=self.straight, start=np.eye(4)), 1e-14)

    def test_the_same_control_on_the_same_motion_is_caught_once_the_rotation_is_real(self):
        # Same motion, same control, only the extrinsic rotation restored: now it fails loudly.
        self.assertGreater(self.errors(YAW90, wrong_pose=self.swapped_order,
                                       body=self.straight, start=np.eye(4)), self.threshold)


class LostAndRejectionTests(unittest.TestCase):
    def sentinel(self, value=9999.):
        covariance = [0.]*36
        covariance[0] = value
        return covariance

    def test_the_lost_sentinel_is_counted_and_never_published(self):
        instance = adapter()
        stamp = BASE+7
        self.assertIsNone(instance.process(
            odometry_row(np.eye(4), np.zeros(6), stamp, pose_covariance=self.sentinel())))
        self.assertEqual((instance.lost, instance.published, instance.rejected), (1, 0, 0))
        self.assertEqual(instance.status()['last_lost'], stamp)

    def test_a_doubled_sentinel_is_also_lost(self):
        # The frozen recording shows 19998 on the diagonal as well as 9999.
        instance = adapter()
        self.assertIsNone(instance.process(
            odometry_row(np.eye(4), np.zeros(6), pose_covariance=self.sentinel(19998.))))
        self.assertEqual(instance.lost, 1)

    def test_a_twist_sentinel_is_lost_too(self):
        instance = adapter()
        self.assertIsNone(instance.process(
            odometry_row(np.eye(4), np.zeros(6), twist_covariance=self.sentinel())))
        self.assertEqual(instance.lost, 1)

    def test_a_healthy_covariance_is_not_mistaken_for_the_sentinel(self):
        instance = adapter()
        self.assertIsNotNone(instance.process(
            odometry_row(np.eye(4), np.zeros(6), pose_covariance=self.sentinel(.001))))
        self.assertEqual((instance.lost, instance.published), (0, 1))
        self.assertIsNone(instance.status()['last_lost'])

    def test_a_foreign_parent_frame_is_rejected(self):
        instance = adapter()
        self.assertIsNone(instance.process(odometry_row(np.eye(4), np.zeros(6), frame_id='map')))
        self.assertEqual((instance.rejected, instance.lost, instance.published), (1, 0, 0))
        self.assertEqual(instance.status()['rejected_reasons'], {'frame_mismatch': 1})

    def test_a_foreign_child_frame_is_rejected(self):
        instance = adapter()
        self.assertIsNone(instance.process(
            odometry_row(np.eye(4), np.zeros(6), child_frame_id='base_footprint')))
        self.assertEqual(instance.rejected, 1)

    def test_odometry_before_the_extrinsic_is_known_is_rejected_not_buffered(self):
        instance = OdomBaseAdapter(None)
        self.assertIsNone(instance.process(odometry_row(np.eye(4), np.zeros(6))))
        self.assertEqual(instance.status()['rejected_reasons'], {'no_extrinsics': 1})
        self.assertIsNone(instance.status()['extrinsics_sha256'])
        instance.set_extrinsics(SIM, 'tf_static_unlabelled')
        self.assertIsNotNone(instance.process(odometry_row(np.eye(4), np.zeros(6))))

    def test_a_contradicted_extrinsic_fails_closed(self):
        instance = adapter()
        instance.fail('manifest disagrees with /tf_static')
        self.assertIsNone(instance.process(odometry_row(np.eye(4), np.zeros(6))))
        self.assertEqual(instance.status()['rejected_reasons'], {'extrinsics_error': 1})
        self.assertIn('disagrees', instance.status()['extrinsics_error'])

    def test_a_malformed_message_is_rejected_rather_than_raising(self):
        instance = adapter()
        broken = odometry_row(np.eye(4), np.zeros(6))
        broken['pose']['pose']['orientation'] = {'x': 0., 'y': 0., 'z': 0., 'w': 0.}
        short = odometry_row(np.eye(4), np.zeros(6))
        short['twist']['covariance'] = [0.]*12
        for name, row in (('no header', {'child_frame_id': 'camera_link'}),
                          ('zero stamp', odometry_row(np.eye(4), np.zeros(6), 0)),
                          ('bad quaternion', broken),
                          ('short covariance', short)):
            with self.subTest(name):
                self.assertIsNone(instance.process(row))
        self.assertEqual((instance.rejected, instance.published), (4, 0))


class CovarianceTests(unittest.TestCase):
    def covariance(self, seed=0):
        generator = np.random.default_rng(seed)
        root = generator.normal(size=(6, 6))
        return root @ root.T + 6*np.eye(6)

    def test_the_transformed_covariance_is_symmetric_and_positive_semidefinite(self):
        for name, extrinsic in (('pitch 0.25', SIM), ('yaw 90', YAW90)):
            with self.subTest(name):
                result = adapter(extrinsic).process(
                    odometry_row(truth(.2) @ extrinsic, BODY_TWIST,
                                 pose_covariance=self.covariance().reshape(36),
                                 twist_covariance=self.covariance(1).reshape(36)))
                for key in ('pose_covariance', 'twist_covariance'):
                    out = np.asarray(result['odom'][key]).reshape(6, 6)
                    np.testing.assert_array_equal(out, out.T)
                    self.assertGreaterEqual(np.linalg.eigvalsh(out).min(), -1e-12)

    def test_an_identity_extrinsic_passes_the_covariance_through_unchanged(self):
        covariance = self.covariance()
        result = adapter(np.eye(4)).process(
            odometry_row(np.eye(4), np.zeros(6), pose_covariance=covariance.reshape(36),
                         twist_covariance=covariance.reshape(36)))
        for key in ('pose_covariance', 'twist_covariance'):
            np.testing.assert_allclose(np.asarray(result['odom'][key]).reshape(6, 6),
                                       covariance, atol=1e-12)

    def test_the_transform_is_the_adjoint_congruence_not_a_rotation_of_the_blocks(self):
        # Negative control on the covariance path: the block-diagonal rotation-only version
        # (a common shortcut) differs, so the assertion above is not trivially satisfied.
        covariance = self.covariance()
        blocks = np.zeros((6, 6))
        blocks[:3, :3] = blocks[3:, 3:] = SIM[:3, :3]
        shortcut = blocks @ covariance @ blocks.T
        full = transform_covariance(adjoint(SIM), covariance.reshape(36))
        self.assertGreater(deviation(full, shortcut), .1)

    def test_a_non_symmetric_input_is_symmetrized(self):
        covariance = self.covariance()
        covariance[0, 1] += 1.
        out = transform_covariance(adjoint(np.eye(4)), covariance.reshape(36))
        np.testing.assert_array_equal(out, out.T)


class CountingInvariantTests(unittest.TestCase):
    """published == number of tf outputs == number of odom outputs == received - lost - rejected."""

    def stream(self):
        sentinel = [0.]*36
        sentinel[0] = 9999.
        rows = []
        for index, time in enumerate(TIMES):
            _, camera, camera_twist = camera_input(time, SIM)
            rows.append(odometry_row(camera, camera_twist, BASE+index*10**8))
        rows.insert(2, odometry_row(np.eye(4), np.zeros(6), BASE+10**7, pose_covariance=sentinel))
        rows.insert(4, odometry_row(np.eye(4), np.zeros(6), BASE+2*10**7, frame_id='map'))
        rows.insert(5, odometry_row(np.eye(4), np.zeros(6), BASE+3*10**7,
                                    child_frame_id='base_footprint'))
        rows.append(odometry_row(np.eye(4), np.zeros(6), BASE+9*10**8, pose_covariance=sentinel))
        return rows

    def test_every_publish_decision_yields_exactly_one_tf_and_one_odom(self):
        instance = adapter()
        rows = self.stream()
        transforms, odometries = [], []
        for row in rows:
            result = instance.process(row)
            if result is None:
                continue
            transforms.append(result['tf'])
            odometries.append(result['odom'])
        status = instance.status()
        self.assertEqual(status['received'], len(rows))
        self.assertEqual((status['lost'], status['rejected']), (2, 2))
        self.assertEqual(status['published'], len(transforms))
        self.assertEqual(status['published'], len(odometries))
        self.assertEqual(status['published'],
                         status['received']-status['lost']-status['rejected'])
        self.assertEqual(status['last_lost'], BASE+9*10**8)

    def test_the_outgoing_frame_names_are_the_base_contract_not_the_tracked_frame(self):
        result = adapter().process(odometry_row(truth(.1) @ SIM, BODY_TWIST))
        self.assertEqual((result['tf']['parent'], result['tf']['child']), ('odom', 'base_link'))
        self.assertEqual((result['odom']['frame_id'], result['odom']['child_frame_id']),
                         ('odom', 'base_link'))
        self.assertEqual(result['tf']['stamp_ns'], result['odom']['stamp_ns'])
        self.assertEqual(result['tf']['stamp_ns'], result['stamp_ns'])


class StatusTests(unittest.TestCase):
    def test_the_status_declares_the_estimate_as_an_estimate(self):
        row = adapter(odom_source='rtabmap_rgbd_odometry').status()
        self.assertEqual(row['schema'], 1)
        self.assertEqual(row['pose_source'], 'estimated')
        self.assertEqual(row['twist_source'], 'rtabmap_velocity_guess')
        self.assertEqual(row['odom_source'], 'rtabmap_rgbd_odometry')
        self.assertEqual((row['odom_frame'], row['base_frame'], row['tracked_frame']),
                         ('odom', 'base_link', 'camera_link'))
        self.assertEqual(row['extrinsics_provenance'], 'modelled_sim')
        self.assertEqual((row['received'], row['published'], row['lost'], row['rejected']),
                         (0, 0, 0, 0))
        json.dumps(row, allow_nan=False)

    def test_the_status_carries_the_numeric_extrinsic_as_well_as_its_digest(self):
        row = adapter().status()
        np.testing.assert_array_equal(np.array(row['extrinsics']), SIM)
        self.assertEqual(row['extrinsics_sha256'], extrinsics_sha256(SIM))

    def test_an_unknown_provenance_label_is_refused(self):
        for label in ('vibes', None, ''):
            with self.assertRaises(ValueError):
                OdomBaseAdapter(SIM, extrinsics_provenance=label)

    def test_a_base_frame_equal_to_the_tracked_frame_admits_only_the_identity(self):
        OdomBaseAdapter(np.eye(4), tracked_frame='base_link', extrinsics_provenance='identity')
        with self.assertRaises(ValueError):
            OdomBaseAdapter(SIM, tracked_frame='base_link', extrinsics_provenance='identity')

    def test_the_odom_frame_may_not_collide_with_the_other_two(self):
        for kwargs in ({'base_frame': 'odom'}, {'tracked_frame': 'odom'}):
            with self.assertRaises(ValueError):
                OdomBaseAdapter(np.eye(4), extrinsics_provenance='identity', **kwargs)

    def test_a_non_rigid_extrinsic_is_refused(self):
        for bad in (np.zeros((4, 4)), 2*np.eye(4), np.eye(3)):
            with self.assertRaises(ValueError):
                OdomBaseAdapter(bad, extrinsics_provenance='measured')


class ExtrinsicsDigestTests(unittest.TestCase):
    def test_the_digest_is_the_documented_function_of_the_matrix(self):
        self.assertEqual(extrinsics_sha256(SIM), hashlib.sha256(
            json.dumps(SIM.tolist(), separators=(',', ':')).encode()).hexdigest())

    def test_the_digest_is_stable_for_the_same_matrix(self):
        self.assertEqual(extrinsics_sha256(SIM), extrinsics_sha256(np.array(SIM)))
        self.assertEqual(extrinsics_sha256(SIM),
                         extrinsics_sha256(matrix_from_xyz_rpy(SIM_XYZ, SIM_RPY)))

    def test_the_digest_changes_for_a_perturbed_matrix(self):
        perturbed = np.array(SIM)
        perturbed[0, 3] += 1e-9
        self.assertNotEqual(extrinsics_sha256(SIM), extrinsics_sha256(perturbed))


class ManifestTests(unittest.TestCase):
    """The manifest is only worth having if it cannot drift away from what it claims to describe."""

    manifest = json.loads(MANIFEST.read_text())
    launch = LAUNCH.read_text()
    world = WORLD.read_text()

    def numbers(self, text):
        return [float(v) for v in re.findall(r'-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', text)]

    def test_the_launch_helper_still_maps_rpy_to_roll_pitch_yaw(self):
        # The euler convention the manifest declares is only meaningful while this holds.
        self.assertRegex(self.launch,
                         r'"--roll",\s*str\(rpy\[0\]\),\s*"--pitch",\s*str\(rpy\[1\]\),'
                         r'\s*"--yaw",\s*str\(rpy\[2\]\)')

    def test_the_base_to_tracked_entry_equals_the_launch_literal(self):
        match = re.search(r'static_tf\(\s*"base_to_camera",\s*"base_link",\s*"camera_link",'
                          r'\s*xyz=\(([^)]*)\),\s*rpy=\(([^)]*)\)\s*\)', self.launch, re.S)
        self.assertIsNotNone(match, 'static_tf("base_to_camera", ...) not found in the launch file')
        entry = self.manifest['base_to_tracked']
        self.assertEqual(self.numbers(match.group(1)), [float(v) for v in entry['xyz']])
        self.assertEqual(self.numbers(match.group(2)), [float(v) for v in entry['rpy']])

    def test_the_tracked_to_optical_entry_equals_the_launch_literal(self):
        match = re.search(r'static_tf\(\s*"camera_to_optical",\s*"camera_link",'
                          r'\s*"camera_optical_frame",\s*rpy=\(([^)]*)\)\s*\)', self.launch, re.S)
        self.assertIsNotNone(match, 'static_tf("camera_to_optical", ...) not found')
        entry = self.manifest['tracked_to_optical']
        self.assertEqual(self.numbers(match.group(1)), [float(v) for v in entry['rpy']])
        # No xyz argument in the launch call, so the manifest must carry the helper's default.
        self.assertNotIn('xyz', match.group(0))
        self.assertEqual([float(v) for v in entry['xyz']], [0., 0., 0.])

    def test_the_base_to_tracked_entry_equals_the_world_sensor_pose(self):
        match = re.search(r'<sensor name="rgbd" type="rgbd_camera">\s*<pose>([^<]*)</pose>',
                          self.world)
        self.assertIsNotNone(match, '<sensor name="rgbd"> pose not found in room_carmount.sdf')
        entry = self.manifest['base_to_tracked']
        self.assertEqual(self.numbers(match.group(1)),
                         [float(v) for v in entry['xyz']]+[float(v) for v in entry['rpy']])

    def test_the_frame_names_are_the_ones_the_simulation_publishes(self):
        frames = self.manifest['frames']
        self.assertEqual((frames['base'], frames['tracked'], frames['optical']),
                         ('base_link', 'camera_link', 'camera_optical_frame'))
        self.assertIn('<ignition_frame_id>%s</ignition_frame_id>' % frames['optical'], self.world)

    def test_the_manifest_says_the_numbers_are_modelled_not_measured(self):
        self.assertEqual(self.manifest['provenance'], 'modelled_sim')
        self.assertIn('not a measurement', self.manifest['note'])
        self.assertTrue(self.manifest['sources'])

    def test_loading_the_manifest_round_trips_to_the_matrix_and_the_label(self):
        matrix, provenance = load_extrinsics_manifest(MANIFEST)
        self.assertEqual(provenance, 'modelled_sim')
        np.testing.assert_array_equal(matrix, MANIFEST_MATRIX)
        self.assertEqual(extrinsics_sha256(matrix), extrinsics_sha256(MANIFEST_MATRIX))

    def test_an_unlabelled_or_wrong_schema_manifest_is_refused(self):
        for name, payload in (
                ('no provenance',
                 {'schema': 1, 'base_to_tracked': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}}),
                ('unlabelled provenance', dict(self.manifest, provenance='tf_static_unlabelled')),
                ('wrong schema', dict(self.manifest, schema=2))):
            with self.subTest(name), tempfile.NamedTemporaryFile('w', suffix='.json') as handle:
                json.dump(payload, handle)
                handle.flush()
                with self.assertRaises(ValueError):
                    load_extrinsics_manifest(handle.name)


class NodeWiringTests(unittest.TestCase):
    """No ROS here, so pin that the node delegates every decision to the tested core."""

    source = inspect.getsource(node.main)
    callback = re.search(r'\n        def odometry\(self, message\):\n(.*?)\n        def ',
                         inspect.getsource(node.main), re.S)

    def test_the_callback_was_found_where_the_other_assertions_assume_it_is(self):
        self.assertIsNotNone(self.callback)

    def test_every_odometry_message_goes_through_the_tested_process_call(self):
        self.assertEqual(self.source.count('self.adapter.process('), 1)
        self.assertRegex(
            self.callback.group(1),
            r'\A\s+result = self\.adapter\.process\(message_to_ordereddict\(message\)\)\n')
        self.assertRegex(self.source,
                         r"create_subscription\(Odometry, args\.prefix\+'/odom', self\.odometry,")

    def test_tf_and_odom_are_published_once_each_and_only_from_the_returned_value(self):
        for publisher in ('self.tf_pub.publish(', 'self.odom_pub.publish('):
            self.assertEqual(self.source.count(publisher), 1, publisher)
        body = self.callback.group(1).split('\n')
        guard = next(i for i, line in enumerate(body) if line.strip() == 'if result is not None:')
        indent = len(body[guard])-len(body[guard].lstrip())
        published = 0
        for index, line in enumerate(body):
            if 'tf_pub.publish' in line or 'odom_pub.publish' in line:
                published += 1
                self.assertGreater(index, guard, line)
                self.assertGreater(len(line)-len(line.lstrip()), indent, line)
        self.assertEqual(published, 2)
        self.assertNotIn('else', self.callback.group(1))
        self.assertIn("result['tf']['matrix']", self.callback.group(1))
        self.assertIn("odometry_message(result['odom'])", self.callback.group(1))

    def test_nothing_but_the_status_is_published_when_the_core_returns_none(self):
        self.assertEqual(self.callback.group(1).rstrip().split('\n')[-1].strip(),
                         'self.publish_status()')
        self.assertEqual(self.callback.group(1).count('self.publish_status()'), 1)

    def test_the_static_extrinsic_is_read_once_from_a_transient_local_subscription(self):
        self.assertRegex(self.source,
                         r"create_subscription\(\s*TFMessage, '/tf_static', self\.static_transforms,")
        self.assertRegex(self.source,
                         r'QoSProfile\(depth=100, durability=DurabilityPolicy\.TRANSIENT_LOCAL,\s*'
                         r'reliability=ReliabilityPolicy\.RELIABLE\)')
        self.assertIn('if self.adapter.matrix is not None or self.adapter.extrinsics_error is not None:',
                      self.source)

    def test_a_manifest_disagreement_fails_closed_instead_of_choosing_a_side(self):
        self.assertRegex(self.source,
                         r'if not deviation <= EXTRINSICS_TOLERANCE:\s*\n\s*self\.adapter\.fail\(')
        self.assertEqual(node.EXTRINSICS_TOLERANCE, 1e-9)

    def test_the_published_topics_and_depths_are_the_handoff_contract(self):
        self.assertIn("create_publisher(TFMessage, '/tf', 100)", self.source)
        self.assertIn("create_publisher(Odometry, '/odom', qos_profile_sensor_data)", self.source)
        self.assertIn('create_publisher(String, STATUS_TOPIC, 10)', self.source)
        self.assertEqual(node.STATUS_TOPIC, '/fpga/odom_adapter_status')

    def test_the_frame_names_are_arguments_rather_than_literals(self):
        # Literals are allowed only as argparse defaults, never in the node body.
        body = self.source[self.source.index('    import rclpy'):]
        self.assertNotIn("'base_link'", body)
        self.assertNotIn("'camera_link'", body)
        self.assertGreaterEqual(len(re.findall(r'args\.base_frame', body)), 2)
        self.assertGreaterEqual(len(re.findall(r'args\.tracked_frame', body)), 2)
        self.assertGreaterEqual(len(re.findall(r'args\.odom_frame', body)), 1)

    def test_the_status_timer_runs_on_the_steady_clock(self):
        self.assertRegex(self.source, r'create_timer\(1\./args\.status_hz, self\.publish_status,\s*'
                                      r'clock=Clock\(clock_type=ClockType\.STEADY_TIME\)\)')

    def test_ros_is_imported_inside_main_so_help_works_without_it(self):
        module = inspect.getsource(node)
        self.assertNotRegex(module[:module.index('def main(')], r'\n\s*import rclpy')
        self.assertIn('    import rclpy\n', self.source)

    def test_scipy_is_imported_once_and_never_inside_a_callback(self):
        self.assertEqual(self.source.count('from scipy.spatial.transform import Rotation'), 1)
        self.assertNotIn('Rotation', self.callback.group(1))


if __name__ == '__main__':
    unittest.main()
