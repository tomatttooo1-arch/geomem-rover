"""ROS adapter: python -m host.ros_localization_gate --database DB --map-id SHA256.

Provider must use the same pinned read-only database and must remap raw /tf to
/fpga/localization_raw_tf. This node emits only approved TF on /fpga/localization_tf;
Nav2 consumers must remap /tf there. This is an explicit deployment contract,
not something topic health alone can prove. No raw TF is relayed by this node.

With --map-odom-hold-hz the approved world->odom edge leaves the per-approval message and is
republished on a timer, stamped ROS now plus --map-odom-future-s and deduplicated by stamp, so a
consumer can look it up at its own now instead of extrapolating a retired acquisition stamp.

--count-mode (default correction since 2026-10-15 evening) chooses what is approved; see
host/localization_gate.py COUNT_MODES. correction approves the map->odom correction of rtabmap's
outputs and then publishes one pose per healthy odometry pair (held correction x odometry), at the
odometry stamp; it subscribes no camera topic, so --acquisition-topic and --rgb-reliable do not
apply. Its status stamp_ns is the newest approval ever published (monotonic within an instance)
and correction_stamp_ns the newest approval in the row. --max-correction-age-s is its X.
--odom-jump-check (default on; --no-odom-jump-check is the 4edc10f gate exactly) revokes, or drops the entry
candidates, when one healthy odometry step exceeds --max-odom-speed-mps x dt + --odom-jump-margin-m
(defaults 0.5 m/s and 0.10 m; per platform; correction mode only).
output counts consecutive rtabmap outputs per camera frame; frame is the pre-2026-10-15 gate,
kept for comparison: it counts every camera frame and cannot reach valid while odometry/rtabmap
skip frames.

--acquisition-topic (output/frame modes) chooses the message whose header.stamp opens an acquisition: rgb (default,
the colour Image on PREFIX/rgb) or camera_info (the colour CameraInfo on PREFIX/camera_info, a
small message the gate deserialises far sooner than a 640x480 image). camera_info is only
correct if the camera stamps its colour CameraInfo identically to the colour Image. The host
replay publisher does (tools/tum_rgbd_pub, --stamp-now); the car's Orbbec driver is NOT verified
yet: record 100 colour Image/CameraInfo pairs on the board and require every stamp equal before
switching a car run to camera_info. Remap PREFIX/camera_info to the COLOUR camera_info, never the
depth one rtabmap is fed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import time

import numpy as np

from host.localization_gate import (CallbackClock, COUNT_MODES, KINDS, LocalizationGate, MapOdomHold,
                                    ODOM_TRACKED_EDGE, WORLD_ODOM_EDGE, approval_edges,
                                    event_row, pose_matrix)


def load_prior(path, expected):
    path = Path(path).resolve()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != expected:
        raise ValueError('prior database SHA256 mismatch')
    with sqlite3.connect(path.as_uri()+'?mode=ro&immutable=1', uri=True) as db:
        ids = {row[0] for row in db.execute('SELECT id FROM Node')}
    if not ids:
        raise ValueError('empty prior map')
    return ids


def navigation_ready(bundle):
    """Declared map-bundle readiness. A missing or unreadable bundle is not ready."""
    if bundle is None:
        return False
    try:
        return json.loads((Path(bundle)/'bundle.json').read_text()).get('navigation_ready') is True
    except (OSError, ValueError, AttributeError):
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--database', required=True, type=Path)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--prefix', default='/probe')
    parser.add_argument('--tracked-frame', default='camera_link')
    parser.add_argument('--world-frame', default='map')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--bundle', type=Path,
                        help='map bundle directory; bundle.json navigation_ready is reported, missing is false')
    parser.add_argument('--odom-source', default='rtabmap_rgbd_odometry',
                        help='declared odometry provenance reported in status')
    parser.add_argument('--publish-tracked-frame', action=argparse.BooleanOptionalAction, default=True,
                        help='publish the odom->tracked edge on approval; turn off when another node owns it')
    parser.add_argument('--map-odom-hold-hz', type=float, default=0.,
                        help='0 keeps the per-approval map->odom edge; >0 moves it to a held republisher')
    parser.add_argument('--map-odom-future-s', type=float, default=.1,
                        help='held map->odom stamp offset ahead of ROS now')
    parser.add_argument('--events', type=Path, help='new optional diagnostic log; never overwritten')
    parser.add_argument('--diagnostics', action='store_true', help='add callback stages and policy snapshots to event log only')
    parser.add_argument('--rgb-reliable', action='store_true',
                        help='require a reliable publisher on the acquisition topic (rgb or camera_info); '
                             'use for reliable large-image replay')
    parser.add_argument('--count-mode', choices=COUNT_MODES, default='correction',
                        help='correction (default, owner 2026-10-15 evening): approve the map->odom correction '
                             'of 3 consecutive linked rtabmap outputs, then one pose per odometry pair; no '
                             'camera topic is subscribed. output: count consecutive rtabmap outputs '
                             '(Info.ref_id +1); frame: the pre-2026-10-15 every-camera-frame count')
    parser.add_argument('--max-correction-age-s', type=float, default=2.,
                        help='correction mode: X, the oldest held correction still used (s, > 0). Tiers in '
                             'host/localization_gate.py COUNT_MODES (>= 3 Hz and p90 <= 1.0 s: 2; 1-3 Hz and '
                             'p90 <= 1.5 s: 3; < 1 Hz: no estimated-pose navigation)')
    parser.add_argument('--odom-jump-check', action=argparse.BooleanOptionalAction, default=True,
                        help='correction mode: revoke (valid) or drop the entry candidates when a healthy odometry '
                             'step is longer than V x dt + E (2026-10-17 第 10 棒); --no-odom-jump-check is the '
                             '4edc10f gate exactly')
    parser.add_argument('--max-odom-speed-mps', type=float, default=.5,
                        help='correction mode: V, the fastest the platform drives (m/s, > 0; per platform)')
    parser.add_argument('--odom-jump-margin-m', type=float, default=.1,
                        help='correction mode: E, the step allowance beyond V x dt (m, >= 0; default = the 10 cm '
                             'correction jump limit)')
    parser.add_argument('--acquisition-topic', choices=('rgb', 'camera_info'), default='rgb',
                        help='stamp source of an acquisition: PREFIX/rgb (Image) or PREFIX/camera_info '
                             '(colour CameraInfo; only where it is stamped identically to the colour Image)')
    args, ros_args = parser.parse_known_args(argv)
    if args.diagnostics and not args.events:
        parser.error('--diagnostics requires --events')
    if not args.map_odom_hold_hz >= 0:
        parser.error('--map-odom-hold-hz must be non-negative')
    if not args.map_odom_future_s >= 0:
        parser.error('--map-odom-future-s must be non-negative')
    if not (args.max_correction_age_s > 0 and math.isfinite(args.max_correction_age_s)):
        parser.error('--max-correction-age-s must be a positive finite number')
    if not (args.max_odom_speed_mps > 0 and math.isfinite(args.max_odom_speed_mps)):
        parser.error('--max-odom-speed-mps must be a positive finite number')
    if not (args.odom_jump_margin_m >= 0 and math.isfinite(args.odom_jump_margin_m)):
        parser.error('--odom-jump-margin-m must be a non-negative finite number')
    ready = navigation_ready(args.bundle)
    ids = load_prior(args.database, args.map_id)
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import CameraInfo, Image
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rtabmap_msgs.msg import Info, OdomInfo
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

    class GateNode(Node):
        def __init__(self):
            super().__init__('fpga_localization_gate')
            self.gate = LocalizationGate(args.map_id, ids, args.tracked_frame, args.world_frame, args.odom_frame,
                                         navigation_ready=ready, odom_source=args.odom_source,
                                         count_mode=args.count_mode,
                                         max_correction_age_s=args.max_correction_age_s,
                                         odom_jump_check=args.odom_jump_check,
                                         max_odom_speed_mps=args.max_odom_speed_mps,
                                         odom_jump_margin_m=args.odom_jump_margin_m)
            self.events = args.events.open('x', buffering=1) if args.events else None
            self.last_logged = None
            self.create_subscription(String, '/fpga/localization_discontinuity', self.discontinuity, 10)
            self.status_pub = self.create_publisher(String, '/fpga/localization_status', 10)
            self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/fpga/localization_pose', 10)
            self.tf_pub = self.create_publisher(TFMessage, '/fpga/localization_tf', 10)
            rgb_qos = (QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                       if args.rgb_reliable else qos_profile_sensor_data)
            # Only the header stamp of the acquisition message is read, whichever type it is.
            # correction mode reads no camera frame: no acquisition subscription at all.
            acquisition_type = CameraInfo if args.acquisition_topic == 'camera_info' else Image
            if args.count_mode != 'correction':
                self.create_subscription(acquisition_type, args.prefix+'/'+args.acquisition_topic,
                                         self.acquisition, rgb_qos)
            for kind, msg_type in zip(KINDS, (Odometry, OdomInfo, Info, PoseWithCovarianceStamped)):
                self.create_subscription(msg_type, args.prefix+'/'+kind,
                                         lambda msg, k=kind: self.receive(k, msg), qos_profile_sensor_data)
            self.create_timer(.1, self.heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.hold = MapOdomHold(int(round(args.map_odom_future_s*10**9))) if args.map_odom_hold_hz > 0 else None
            if self.hold is not None:
                self.create_timer(1./args.map_odom_hold_hz, self.hold_map_odom,
                                  clock=Clock(clock_type=ClockType.STEADY_TIME))

        def hold_map_odom(self):
            # Read the state the heartbeat already maintains; never advance the gate clock here.
            held = self.hold.tick(self.gate.state, self.get_clock().now().nanoseconds)
            if held is None:
                return
            stamp, matrix = held
            self.tf_pub.publish(TFMessage(transforms=[
                transform_stamped(args.world_frame, args.odom_frame, matrix, stamp)]))

        def discontinuity(self, message):
            try:
                cycle = json.loads(message.data)['cycle']
                self.gate.discontinuity(cycle)
            except (KeyError, ValueError, TypeError):
                self.gate.revoke('lost', 'invalid_discontinuity_request')
            self.heartbeat()

        def clocks(self):
            """The single steady/ROS reading of one callback; see CallbackClock.

            Called exactly once at the entry of every callback and then passed through, so
            what the gate decided on is what the event log records.
            """
            return CallbackClock(time.monotonic_ns(), self.get_clock().now().nanoseconds)

        def diagnostic_snapshot(self):
            # Diagnostics only. This timestamp is a STAGE probe, deliberately re-read so a
            # before/after pair brackets the callback body; it is never a row's decision clock.
            if not args.diagnostics:
                return None
            return {'monotonic_ns':time.monotonic_ns(),'ros_ns':self.get_clock().now().nanoseconds,
                    'state':self.gate.state,'reason':self.gate.reason,'epoch':self.gate.epoch,
                    'streak':self.gate.streak,'last_complete':self.gate.last_complete,
                    'pending':{str(s):{'wall':v['wall'],'kinds':list(v['messages']),'bad':v['bad']}
                               for s,v in self.gate.pending.items()}}

        def record(self, event, clock, **fields):
            # `clock` is the callback's own reading, never a fresh one taken here: the row
            # must carry the value the decision above it consumed.
            if self.events:
                self.events.write(json.dumps(event_row(event, self.gate, clock, **fields),
                                             allow_nan=False)+'\n')

        def acquisition(self, message):
            clock = self.clocks()
            before=self.diagnostic_snapshot()
            s = message.header.stamp
            # output mode: outputs held for this frame are joined now and can complete approvals.
            outputs = self.gate.acquisition(s.sec*10**9+s.nanosec, *clock)
            self.record('acquisition', clock, stamp_ns=s.sec*10**9+s.nanosec, approved=outputs,
                        pending_overflows=self.gate.pending_overflows,
                        **({'diagnostic':{'before':before,'after':self.diagnostic_snapshot()}} if args.diagnostics else {}))
            self.publish_approvals(outputs)
            self.heartbeat(clock)

        def heartbeat(self, clock=None):
            # Nested calls hand in the caller's reading: one callback, one clock. The 10 Hz
            # timer is its own callback and takes its own.
            clock = self.clocks() if clock is None else clock
            before=self.diagnostic_snapshot()
            row = self.gate.status(*clock)
            self.status_pub.publish(String(data=json.dumps(row, allow_nan=False)))
            identity = (row['state'], row['reason'], row['epoch'])
            if identity != self.last_logged:
                self.record('state_transition', clock, stamp_ns=row['stamp_ns'],
                            **({'diagnostic':{'before':before,'after':self.diagnostic_snapshot()}} if args.diagnostics else {}))
                self.last_logged = identity

        def receive(self, kind, message):
            clock = self.clocks()
            before=self.diagnostic_snapshot()
            converted=message_to_ordereddict(message)
            converted_ns=time.monotonic_ns() if args.diagnostics else None
            outputs = self.gate.ingest(kind, converted, *clock)
            after=self.diagnostic_snapshot()
            s = message.header.stamp
            # correction mode: each correction decided in this callback (stamp, ref_id, verdict,
            # translation/yaw against the held or previous one); absent when there is none
            corrections = self.gate.drain_corrections()
            self.record('received_'+kind, clock, stamp_ns=s.sec*10**9+s.nanosec, approved=outputs,
                        pending_overflows=self.gate.pending_overflows,
                        **({'corrections': corrections} if corrections else {}),
                        **({'diagnostic':{'before':before,'converted_ns':converted_ns,'after':after}} if args.diagnostics else {}))
            self.publish_approvals(outputs)
            if args.diagnostics:
                # Diagnostics-only stage marker, and the one row that takes a NEW reading on
                # purpose: paired with the received_ row above it, it brackets the callback
                # for tools/rtabmap_consumer_evidence_audit.py. It carries no gate decision.
                self.record('callback_output_complete',self.clocks(),kind=kind,stamp_ns=s.sec*10**9+s.nanosec)
            self.heartbeat(clock)

        def publish_approvals(self, outputs):
            # Pose and TF of each approval, whichever callback (acquisition or receive) completed it.
            for approved in outputs:
                stamp = approved['stamp_ns']
                pose = approved['pose']['pose']
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = args.world_frame
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                for k in ('x', 'y', 'z'):
                    setattr(msg.pose.pose.position, k, float(pose['position'][k]))
                for k in ('x', 'y', 'z', 'w'):
                    setattr(msg.pose.pose.orientation, k, float(pose['orientation'][k]))
                msg.pose.covariance = [float(v) for v in approved['pose']['covariance']]
                self.pose_pub.publish(msg)
                odom_camera = pose_matrix(approved['odom_pose'])
                # Taken from the approval instead of recomputed here: composing it in this
                # callback is exactly what kept map->odom out of the published contract.
                world_odom = np.asarray(approved['map_odom'], dtype=float)
                edges = {WORLD_ODOM_EDGE: (args.world_frame, args.odom_frame, world_odom),
                         ODOM_TRACKED_EDGE: (args.odom_frame, args.tracked_frame, odom_camera)}
                transforms = [transform_stamped(*edges[name], stamp)
                              for name in approval_edges(args.publish_tracked_frame, self.hold is not None)]
                if transforms:
                    self.tf_pub.publish(TFMessage(transforms=transforms))
                if self.hold is not None:
                    self.hold.update(world_odom)

    rclpy.init(args=ros_args)
    node = GateNode()
    try:
        rclpy.spin(node)
    finally:
        if node.events:
            node.events.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
