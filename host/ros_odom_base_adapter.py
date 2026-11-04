"""ROS adapter: python -m host.ros_odom_base_adapter --prefix /probe [--extrinsics-manifest PATH].

The single continuous publisher of odom->base_link and /odom for the Nav2 handoff. It consumes
<prefix>/odom from rtabmap_ros rgbd_odometry, whose raw /tf stays remapped away to
/fpga/localization_raw_tf, and re-expresses pose, twist and both covariances in base_link using
the static base->tracked extrinsic. The output is declared "estimated": a visual odometry
estimate standing in for the M2(a) wheel/IMU odometry, replaceable under the same contract.

Extrinsic policy, in order of preference:
  --tracked-frame == --base-frame  RTAB-Map already runs with frame_id=base_link, the extrinsic
                                   is the identity and this node degrades to a relay that filters
                                   lost frames and labels the source. Recommended for Gazebo and
                                   the real robot, because a gravity-aligned odom frame is what a
                                   2D costmap needs.
  otherwise                        the direct base->tracked edge is looked up ONCE from
                                   /tf_static (TRANSIENT_LOCAL). Odometry arriving before it is
                                   counted rejected, never buffered: a pose published against an
                                   unknown extrinsic is worse than a gap.
  --extrinsics-manifest            must agree with /tf_static to 1e-9. A disagreement is not
                                   reconciled: the node logs it, reports extrinsics_error in
                                   status and refuses to publish. Its provenance label
                                   (modelled_sim or measured) is what status reports; without a
                                   manifest the label is tf_static_unlabelled, i.e. numbers of
                                   unknown origin.

Lost frames (RTAB-Map's 9999 covariance sentinel) are counted and dropped, so consumers see a
gap rather than a stale pose. Every publish decision is made by OdomBaseAdapter.process, which
is tested without ROS; this file only converts messages.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from host.odom_base_adapter import OdomBaseAdapter, extrinsics_sha256, load_extrinsics_manifest

EXTRINSICS_TOLERANCE = 1e-9
STATUS_TOPIC = '/fpga/odom_adapter_status'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--prefix', default='/probe', help='odometry input is <prefix>/odom')
    parser.add_argument('--tracked-frame', default='camera_link',
                        help='child_frame_id RTAB-Map publishes; equal to --base-frame means identity extrinsic')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--extrinsics-manifest', type=Path,
                        help='base->tracked manifest; must agree with /tf_static to 1e-9 or the node refuses to publish')
    parser.add_argument('--odom-source', default='rtabmap_rgbd_odometry',
                        help='declared odometry provenance reported in status')
    parser.add_argument('--status-hz', type=float, default=1.,
                        help='status republish rate on a steady clock; status is also published after every message')
    args, ros_args = parser.parse_known_args(argv)
    if not args.status_hz > 0:
        parser.error('--status-hz must be positive')
    identity = args.tracked_frame == args.base_frame
    manifest = load_extrinsics_manifest(args.extrinsics_manifest) if args.extrinsics_manifest else None
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy)
    from rosidl_runtime_py.convert import message_to_ordereddict
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from scipy.spatial.transform import Rotation

    def transform_stamped(parent, child, matrix, stamp):
        t = TransformStamped()
        t.header.stamp.sec, t.header.stamp.nanosec = divmod(stamp, 10**9)
        t.header.frame_id, t.child_frame_id = parent, child
        for k, v in zip(('x', 'y', 'z'), matrix[:3, 3]):
            setattr(t.transform.translation, k, float(v))
        for k, v in zip(('x', 'y', 'z', 'w'), Rotation.from_matrix(matrix[:3, :3]).as_quat()):
            setattr(t.transform.rotation, k, float(v))
        return t

    def odometry_message(row):
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = row['frame_id'], row['child_frame_id']
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(row['stamp_ns'], 10**9)
        for k in ('x', 'y', 'z'):
            setattr(msg.pose.pose.position, k, row['pose']['position'][k])
        for k in ('x', 'y', 'z', 'w'):
            setattr(msg.pose.pose.orientation, k, row['pose']['orientation'][k])
        msg.pose.covariance = row['pose_covariance']
        for part, target in (('linear', msg.twist.twist.linear), ('angular', msg.twist.twist.angular)):
            for k in ('x', 'y', 'z'):
                setattr(target, k, row['twist'][part][k])
        msg.twist.covariance = row['twist_covariance']
        return msg

    def edge_matrix(transform):
        q = transform.transform.rotation
        result = np.eye(4)
        result[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        result[:3, 3] = [transform.transform.translation.x, transform.transform.translation.y,
                         transform.transform.translation.z]
        return result

    class AdapterNode(Node):
        def __init__(self):
            super().__init__('fpga_odom_base_adapter')
            self.adapter = OdomBaseAdapter(np.eye(4) if identity else None,
                                           odom_frame=args.odom_frame, base_frame=args.base_frame,
                                           tracked_frame=args.tracked_frame,
                                           odom_source=args.odom_source,
                                           extrinsics_provenance='identity' if identity else None)
            self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
            self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
            self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
            if identity:
                if manifest is not None:
                    self.get_logger().warn('--extrinsics-manifest ignored: tracked frame equals base frame')
                self.log_extrinsics()
            else:
                self.create_subscription(
                    TFMessage, '/tf_static', self.static_transforms,
                    QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE))
            self.create_subscription(Odometry, args.prefix+'/odom', self.odometry, qos_profile_sensor_data)
            self.create_timer(1./args.status_hz, self.publish_status,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))

        def log_extrinsics(self):
            row = self.adapter.status()
            self.get_logger().info('base->tracked extrinsic %s provenance=%s sha256=%s'
                                   % (row['extrinsics'], row['extrinsics_provenance'],
                                      row['extrinsics_sha256']))

        def static_transforms(self, message):
            # Looked up once. Later /tf_static messages cannot silently move the robot's camera.
            if self.adapter.matrix is not None or self.adapter.extrinsics_error is not None:
                return
            for transform in message.transforms:
                if (transform.header.frame_id.lstrip('/') != args.base_frame
                        or transform.child_frame_id.lstrip('/') != args.tracked_frame):
                    continue
                matrix = edge_matrix(transform)
                if manifest is not None:
                    deviation = float(np.max(np.abs(matrix - manifest[0])))
                    if not deviation <= EXTRINSICS_TOLERANCE:
                        self.adapter.fail('manifest disagrees with /tf_static by %.3e m/rad' % deviation)
                        self.get_logger().error(
                            'extrinsics manifest %s disagrees with /tf_static by %.3e; refusing to publish'
                            % (args.extrinsics_manifest, deviation))
                        self.publish_status()
                        return
                self.adapter.set_extrinsics(
                    matrix, manifest[1] if manifest is not None else 'tf_static_unlabelled')
                self.log_extrinsics()
                self.publish_status()
                return

        def odometry(self, message):
            result = self.adapter.process(message_to_ordereddict(message))
            if result is not None:
                self.tf_pub.publish(TFMessage(transforms=[transform_stamped(
                    result['tf']['parent'], result['tf']['child'], result['tf']['matrix'],
                    result['tf']['stamp_ns'])]))
                self.odom_pub.publish(odometry_message(result['odom']))
            self.publish_status()

        def publish_status(self):
            self.status_pub.publish(String(data=json.dumps(self.adapter.status(), allow_nan=False)))

    rclpy.init(args=ros_args)
    node = AdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
