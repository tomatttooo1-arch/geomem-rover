"""Camera-frame RTAB-Map odometry -> base_link odometry. No ROS: pure numerics and counters.

RTAB-Map's rgbd_odometry publishes nav_msgs/Odometry with child_frame_id = the tracked frame
it was configured with (camera_link in the recorded runs). Nav2 wants odom->base_link and an
/odom whose child is base_link, so the static extrinsic has to be divided out of the pose and
the twist. The conventions, once, because every sign error below is a silent one:

  T_odom_cam   pose of the tracked (camera) frame in odom; what Odometry.pose carries.
  T_base_cam   the static base_link->camera_link extrinsic, i.e. the child's pose in the
               parent, (R, t). This is exactly what /tf_static publishes for that edge.
  T_odom_base = T_odom_cam . inv(T_base_cam)

Odometry.twist is expressed in the child frame, so it is a body twist of the camera and moves
by the adjoint of T_base_cam, in ROS covariance order (v, w) = (x, y, z, rotx, roty, rotz):

  Ad = [[R, [t]x R], [0, R]]        w_b = R w_c        v_b = R v_c + t x (R w_c)

and both covariance blocks follow as Sigma_b = Ad Sigma_c Ad^T, symmetrized afterwards. The
pose covariance is pushed through the same adjoint: that is a first-order approximation, not
an identity, and it is the same one rtabmap_ros and robot_localization make.

Nothing here declares the result to be anything but a visual estimate: pose_source is always
"estimated" and twist_source always "rtabmap_velocity_guess" (Odometry::getVelocityGuess()),
never a wheel measurement. Losing tracking is reported, not smoothed: RTAB-Map marks a lost
frame with a 9999 covariance sentinel, and such a message is counted and dropped, never
published, so a consumer sees a gap instead of a frozen pose.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from host.localization_gate import pose_matrix, stamp_ns

# RTAB-Map's lost sentinel: rgbd_odometry fills the covariance diagonal with 9999 when the
# transform could not be computed. Observed diagonals in the frozen recording: {0, 0.001,
# 9999, 19998}; 19998 is 2*9999 after a lost frame is composed twice, so the test is >=.
LOST_SENTINEL = 9999.
PROVENANCES = ('modelled_sim', 'measured', 'identity', 'tf_static_unlabelled')
MANIFEST_PROVENANCES = ('modelled_sim', 'measured')
AXES = ('x', 'y', 'z')


def skew(vector):
    x, y, z = [float(v) for v in vector]
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def matrix_from_xyz_rpy(xyz, rpy):
    """REP-103 fixed-axis roll-pitch-yaw, the convention tf2_ros static_transform_publisher uses.

    Fixed-axis RPY is Rz(yaw) Ry(pitch) Rx(roll), which scipy spells as the extrinsic
    from_euler('xyz', [roll, pitch, yaw]) (equivalently the intrinsic 'ZYX' [yaw, pitch, roll]).
    """
    xyz = np.asarray(xyz, dtype=float)
    rpy = np.asarray(rpy, dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite(xyz).all() or not np.isfinite(rpy).all():
        raise ValueError('finite xyz and rpy triples required')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def transform_valid(matrix):
    matrix = np.asarray(matrix, dtype=float)
    return bool(matrix.shape == (4, 4) and np.isfinite(matrix).all()
                and np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-9)
                and np.allclose(matrix[:3, :3] @ matrix[:3, :3].T, np.eye(3), atol=1e-8))


def adjoint(matrix):
    """Ad_T in (v, w) order for T = (R, t): body twists in the child move to the parent by this."""
    if not transform_valid(matrix):
        raise ValueError('finite rigid 4x4 transform required')
    matrix = np.asarray(matrix, dtype=float)
    rotation, translation = matrix[:3, :3], matrix[:3, 3]
    result = np.zeros((6, 6))
    result[:3, :3] = rotation
    result[:3, 3:] = skew(translation) @ rotation
    result[3:, 3:] = rotation
    return result


def transform_covariance(ad, covariance):
    """Sigma_b = Ad Sigma_c Ad^T, symmetrized. Input is the flat ROS 36-vector."""
    cov = np.asarray(covariance, dtype=float)
    if cov.size != 36 or not np.isfinite(cov).all():
        raise ValueError('finite 36 element covariance required')
    result = ad @ cov.reshape(6, 6) @ ad.T
    return (result + result.T) / 2.


def extrinsics_sha256(matrix):
    """Identity of a numeric extrinsic. Stable for the same matrix, different for a perturbed one."""
    payload = json.dumps(np.asarray(matrix, dtype=float).tolist(), separators=(',', ':'))
    return hashlib.sha256(payload.encode()).hexdigest()


def load_extrinsics_manifest(path):
    """Read a base->tracked extrinsics manifest. Returns (4x4, provenance).

    A manifest is a declaration of where the numbers came from, so an unlabelled or
    unknown provenance is an error rather than a default.
    """
    manifest = json.loads(Path(path).read_text())
    if manifest.get('schema') != 1:
        raise ValueError('unsupported extrinsics manifest schema')
    provenance = manifest.get('provenance')
    if provenance not in MANIFEST_PROVENANCES:
        raise ValueError('manifest provenance must be one of '+repr(MANIFEST_PROVENANCES))
    entry = manifest['base_to_tracked']
    return matrix_from_xyz_rpy(entry['xyz'], entry['rpy']), provenance


def quaternion_of(matrix):
    return dict(zip(('x', 'y', 'z', 'w'),
                    [float(v) for v in Rotation.from_matrix(np.asarray(matrix)[:3, :3]).as_quat()]))


class OdomBaseAdapter:
    """Turn one camera-frame Odometry into the odom->base_link TF and /odom to publish, or None.

    None means nothing may be published for that message: the frame names did not match the
    contract (rejected), the extrinsic is not known or is contradicted (rejected, fail closed),
    or RTAB-Map reported lost tracking (lost). The ROS wrapper does message conversion only, so
    the publish decision, the counters and the invariant published == #tf == #odom are testable
    without ROS on a host that has none.
    """

    def __init__(self, T_base_cam, odom_frame='odom', base_frame='base_link',
                 tracked_frame='camera_link', odom_source='rtabmap_rgbd_odometry',
                 extrinsics_provenance=None):
        if not odom_frame or not base_frame or not tracked_frame:
            raise ValueError('frame names required')
        if odom_frame in (base_frame, tracked_frame):
            raise ValueError('the odom frame must differ from the base and tracked frames')
        if not odom_source:
            raise ValueError('odometry provenance label required')
        self.odom_frame, self.base_frame, self.tracked_frame = odom_frame, base_frame, tracked_frame
        self.odom_source = odom_source
        self.matrix = self.adjoint = self.provenance = None
        self.extrinsics_error = None
        self.last_lost = None
        self.received = self.published = self.lost = self.rejected = 0
        self.rejected_reasons = {}
        if T_base_cam is not None:
            self.set_extrinsics(T_base_cam, extrinsics_provenance)

    def set_extrinsics(self, matrix, provenance):
        """Install the one base->tracked extrinsic. Looked up once; never re-decided per message."""
        if provenance not in PROVENANCES:
            raise ValueError('extrinsics provenance must be one of '+repr(PROVENANCES))
        if not transform_valid(matrix):
            raise ValueError('finite rigid 4x4 extrinsic required')
        if self.base_frame == self.tracked_frame and not np.allclose(matrix, np.eye(4), atol=1e-12):
            raise ValueError('a base frame equal to the tracked frame admits only the identity')
        self.matrix = np.array(matrix, dtype=float)
        self.adjoint = adjoint(self.matrix)
        self.provenance = provenance

    def fail(self, detail):
        """Fail closed: record why the extrinsic cannot be trusted and stop publishing."""
        self.extrinsics_error = str(detail)

    def reject(self, reason):
        self.rejected += 1
        self.rejected_reasons[reason] = self.rejected_reasons.get(reason, 0) + 1
        return None

    def lost_sentinel(self, row):
        """RTAB-Map marks a frame it could not track with a 9999 covariance sentinel."""
        for block in ('pose', 'twist'):
            try:
                if float(row[block]['covariance'][0]) >= LOST_SENTINEL:
                    return True
            except (KeyError, IndexError, TypeError, ValueError):
                return False
        return False

    def process(self, row):
        """One Odometry as a message_to_ordereddict mapping in, one publish decision out."""
        self.received += 1
        if self.extrinsics_error is not None:
            return self.reject('extrinsics_error')
        if self.matrix is None:
            return self.reject('no_extrinsics')
        try:
            stamp = stamp_ns(row)
            frames = (row['header']['frame_id'], row['child_frame_id'])
        except (KeyError, TypeError, ValueError):
            return self.reject('malformed')
        if frames != (self.odom_frame, self.tracked_frame):
            return self.reject('frame_mismatch')
        if self.lost_sentinel(row):
            self.lost += 1
            self.last_lost = stamp
            return None
        try:
            odom_cam = pose_matrix(row['pose']['pose'])
            twist = row['twist']['twist']
            camera_twist = np.array([float(twist[part][k]) for part in ('linear', 'angular')
                                     for k in AXES])
            pose_covariance = transform_covariance(self.adjoint, row['pose']['covariance'])
            twist_covariance = transform_covariance(self.adjoint, row['twist']['covariance'])
        except (KeyError, IndexError, TypeError, ValueError):
            return self.reject('malformed')
        if not np.isfinite(camera_twist).all():
            return self.reject('malformed')
        odom_base = odom_cam @ np.linalg.inv(self.matrix)
        base_twist = self.adjoint @ camera_twist
        self.published += 1
        return {'stamp_ns': stamp,
                'tf': {'parent': self.odom_frame, 'child': self.base_frame,
                       'stamp_ns': stamp, 'matrix': odom_base},
                'odom': {'frame_id': self.odom_frame, 'child_frame_id': self.base_frame,
                         'stamp_ns': stamp,
                         'pose': {'position': dict(zip(AXES, [float(v) for v in odom_base[:3, 3]])),
                                  'orientation': quaternion_of(odom_base)},
                         'pose_covariance': [float(v) for v in pose_covariance.reshape(36)],
                         'twist': {'linear': dict(zip(AXES, [float(v) for v in base_twist[:3]])),
                                   'angular': dict(zip(AXES, [float(v) for v in base_twist[3:]]))},
                         'twist_covariance': [float(v) for v in twist_covariance.reshape(36)]}}

    def status(self):
        """Declared provenance and counters. Labels never change a publish decision."""
        return {'schema': 1,
                'pose_source': 'estimated',
                'odom_source': self.odom_source,
                'twist_source': 'rtabmap_velocity_guess',
                'odom_frame': self.odom_frame,
                'base_frame': self.base_frame,
                'tracked_frame': self.tracked_frame,
                'extrinsics': None if self.matrix is None else self.matrix.tolist(),
                'extrinsics_sha256': None if self.matrix is None else extrinsics_sha256(self.matrix),
                'extrinsics_provenance': self.provenance,
                'extrinsics_error': self.extrinsics_error,
                'last_lost': self.last_lost,
                'received': self.received,
                'published': self.published,
                'lost': self.lost,
                'rejected': self.rejected,
                'rejected_reasons': dict(self.rejected_reasons)}
