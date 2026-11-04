#!/usr/bin/env python3
"""Stream a TUM RGB-D sequence into ROS 2 at its recorded pace and log RTAB-Map feedback.

Publishes /fpga/rgb (bgr8), /fpga/depth (32FC1 metres = uint16/5000), /fpga/camera_info,
all stamped with the RGB capture time of the pair (RGB/depth paired by nearest stamp,
max 0.02 s, one-to-one, TUM convention). Frames are sent on the recorded schedule
(scaled by --speed); nothing waits for the consumer, so a slow consumer drops frames
exactly as with a live camera. Every published frame and every OdomInfo / Info /
odometry / localization_pose message is written as one JSON line to --out.
Frame selection: global zero-based [--start, --stop).
--stamp-now: restamp every published message (rgb, depth, camera_info share one stamp per frame)
as wall-clock-at-first-frame + (dataset stamp - first dataset stamp), so consumers that reject
stale stamps accept the replay. Inter-frame intervals stay in dataset time, so stamps track wall
time only at --speed 1. Off by default: stamps are the raw dataset stamps.
--out is appended to (2026-10-15): a restart of the publisher (tools/car_run/rtabmap_car.sh retries the
camera command) writes a new session into the same file instead of exiting because it exists. Every
session begins with one `start` row carrying `session` (0, 1, ...) and `pid`; readers take the rows of
the last session with last_session() (a file from before this change holds one session).
--gt-odom-tf "x y z qx qy qz qw" (2026-10-17, 定位门第 11 棒; off by default): also publish the sequence's groundtruth
as odometry, TF odom -> base_link on /tf at each frame's stamp, sent BEFORE that frame's images (rtabmap MODE=map
ODOM=wheel looks it up at the image stamp). The argument is the fixture's mount TF base_link -> camera
(tools/nano_ros2/tum_mount.py); the pose math is tools/nano_ros2/tum_gt_odom.py (3-DoF, odom = base_link at the first
published frame inside the groundtruth span). Frames outside that span get no TF (rtabmap skips them). Each `pub`
row then carries `gt_odom` [x, y, yaw] (null when not covered) and one `gt_odom` row records the anchor.
Not with --pingpong (virtual stamps have no groundtruth).
"""
import argparse
import array
import json
import os
import sys
import time
from pathlib import Path

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:   # run as a script (/src/tools/nano_ros2/tum_rgbd_pub.py): tools.* for --gt-odom-tf
    sys.path.insert(0, ROOT)


def last_session(rows):
    """The rows of the last publisher session in a parsed --out file: from its last `start` row on
    (all rows when there is none)."""
    starts = [k for k, row in enumerate(rows) if row.get('event') == 'start']
    return rows[starts[-1]:] if starts else list(rows)


def count_sessions(path):
    """Sessions already in --out (its `start` rows); 0 when the file does not exist."""
    try:
        text = Path(path).read_text()
    except FileNotFoundError:
        return 0
    n = 0
    for line in text.splitlines():
        try:
            n += json.loads(line).get('event') == 'start'
        except (ValueError, AttributeError):
            pass
    return n


def read_list(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if line and not line.startswith('#'):
            stamp, name = line.split()[:2]
            rows.append((float(stamp), name))
    return rows


def associate(rgb, depth, max_dt=0.02):
    cand = sorted((abs(a[0] - b[0]), i, j) for i, a in enumerate(rgb) for j, b in enumerate(depth)
                  if abs(a[0] - b[0]) < max_dt)
    ui, uj, out = set(), set(), []
    for _, i, j in cand:
        if i not in ui and j not in uj:
            ui.add(i); uj.add(j); out.append((i, j))
    return [(rgb[i], depth[j]) for i, j in sorted(out)]


def frame_stamp(rs, t0_src=None, anchor_ns=None):
    """(sec, nanosec) for a frame whose dataset stamp is ``rs``.

    anchor_ns None: the raw dataset stamp. Otherwise anchor_ns + (rs - t0_src), in integer ns
    so intervals between frames are kept exactly.
    """
    if anchor_ns is None:
        sec, nsec = int(rs), int(round((rs - int(rs)) * 1e9))
        if nsec >= 1_000_000_000:
            sec, nsec = sec + 1, nsec - 1_000_000_000
        return sec, nsec
    return divmod(anchor_ns + int(round((rs - t0_src) * 1e9)), 1_000_000_000)


STAT_KEYS = ('Timing/Total/ms', 'Timing/Memory_update/ms', 'Timing/Proximity_by_space/ms',
             'Timing/Likelihood_computation/ms', 'Timing/Posterior_computation/ms',
             'Timing/Map_optimization/ms', 'Timing/Occupancy_grid/ms',
             'Loop/Id/', 'Loop/Highest_hypothesis_id/', 'Loop/Highest_hypothesis_value/',
             'Loop/Visual_inliers/', 'Proximity/Space_detected_id/', 'Loop/Accepted_hypothesis_id/',
             'Memory/Working_memory_size/', 'Memory/Local_graph_size/', 'Memory/Distance_travelled/m',
             'Memory/RAM_usage/MB', 'Memory/RAM_estimated/MB', 'Keypoint/Current_frame/words')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('seq', type=Path, help='TUM sequence directory (rgb.txt, depth.txt)')
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--start', type=int, default=0)
    p.add_argument('--stop', type=int)
    p.add_argument('--speed', type=float, default=1.0)
    p.add_argument('--startup-timeout', type=float, default=60.)
    p.add_argument('--drain', type=float, default=5.)
    p.add_argument('--frame-id', default='camera_optical_frame')
    p.add_argument('--fx', type=float, default=517.3)
    p.add_argument('--fy', type=float, default=516.5)
    p.add_argument('--cx', type=float, default=318.6)
    p.add_argument('--cy', type=float, default=255.3)
    p.add_argument('--pingpong', type=int, default=0,
                   help='N4: play forward, backward, forward... this many legs with continuous virtual stamps')
    p.add_argument('--info-topic', default='/fpga/info', help='startup waits for a publisher here')
    p.add_argument('--no-subscribe', action='store_true', help='do not record odometry/rtabmap topics')
    p.add_argument('--stamp-now', action='store_true',
                   help='stamp messages with wall clock at first frame + dataset offset (default: dataset stamps)')
    p.add_argument('--gt-odom-tf', metavar='MOUNT_TF',
                   help='publish groundtruth odometry (TF odom -> base_link) using this base_link -> camera mount')
    p.add_argument('--groundtruth', type=Path, help='groundtruth file (default SEQ/groundtruth.txt)')
    p.add_argument('--gt-odom-frame', default='odom')
    p.add_argument('--gt-base-frame', default='base_link')
    a = p.parse_args()
    if a.gt_odom_tf is not None and a.pingpong > 1:
        p.error('--gt-odom-tf does not go with --pingpong')
    gt_base = None
    if a.gt_odom_tf is not None:
        from tools.nano_ros2.tum_gt_odom import GroundtruthBase
        gt_base = GroundtruthBase.from_file(a.groundtruth or a.seq / 'groundtruth.txt', a.gt_odom_tf)

    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rtabmap_msgs.msg import OdomInfo, Info

    pairs = associate(read_list(a.seq / 'rgb.txt'), read_list(a.seq / 'depth.txt'))
    stop = len(pairs) if a.stop is None else min(a.stop, len(pairs))
    pairs_sel = pairs[a.start:stop]
    # append, not 'x': a restarted publisher must start again (g5r_1: 9 restarts all exited at once)
    session = count_sessions(a.out)
    log = a.out.open('a', buffering=1)

    def rec(kind, **kw):
        kw.update(event=kind, mono_ns=time.monotonic_ns(), wall_ns=time.time_ns())
        log.write(json.dumps(kw) + '\n')

    rclpy.init(args=[])
    node = rclpy.create_node('tum_rgbd_pub')
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
    rgb_pub = node.create_publisher(Image, '/fpga/rgb', qos)
    depth_pub = node.create_publisher(Image, '/fpga/depth', qos)
    info_pub = node.create_publisher(CameraInfo, '/fpga/camera_info', qos)
    tf_pub = None
    if gt_base is not None:
        from tf2_msgs.msg import TFMessage
        from geometry_msgs.msg import TransformStamped
        tf_pub = node.create_publisher(TFMessage, '/tf', QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    sub_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    counts = {}

    def st(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def on_odom_info(m):
        counts['odom_info'] = counts.get('odom_info', 0) + 1
        rec('odom_info', stamp=st(m.header), lost=bool(m.lost), matches=m.matches, inliers=m.inliers,
            features=m.features, time_estimation=m.time_estimation, key_frame_added=bool(m.key_frame_added),
            local_map_size=m.local_map_size)

    def on_info(m):
        counts['info'] = counts.get('info', 0) + 1
        stats = {k: v for k, v in zip(m.stats_keys, m.stats_values) if k in STAT_KEYS}
        rec('info', stamp=st(m.header), ref_id=m.ref_id, loop_closure_id=m.loop_closure_id,
            proximity_detection_id=m.proximity_detection_id, stats=stats)

    def pose_row(pose):
        q, t = pose.orientation, pose.position
        return [t.x, t.y, t.z, q.x, q.y, q.z, q.w]

    def on_odom(m):
        counts['odom'] = counts.get('odom', 0) + 1
        rec('odom', stamp=st(m.header), pose=pose_row(m.pose.pose), frame=m.header.frame_id,
            child=m.child_frame_id, cov0=m.pose.covariance[0])

    def on_loc(m):
        counts['localization_pose'] = counts.get('localization_pose', 0) + 1
        rec('localization_pose', stamp=st(m.header), pose=pose_row(m.pose.pose), frame=m.header.frame_id,
            cov0=m.pose.covariance[0])

    if not a.no_subscribe:
        node.create_subscription(OdomInfo, '/fpga/odom_info', on_odom_info, sub_qos)
        node.create_subscription(Info, '/fpga/info', on_info, sub_qos)
        node.create_subscription(Odometry, '/fpga/odom', on_odom, sub_qos)
        node.create_subscription(PoseWithCovarianceStamped, '/fpga/localization_pose', on_loc, sub_qos)

    def spin_until(deadline, pred=lambda: False):
        while time.monotonic() < deadline and not pred():
            rclpy.spin_once(node, timeout_sec=max(0., min(0.005, deadline - time.monotonic())))
        return pred()

    rec('start', seq=str(a.seq), pairs_total=len(pairs), start=a.start, stop=stop, speed=a.speed,
        session=session, pid=os.getpid())
    ok = spin_until(time.monotonic() + a.startup_timeout,
                    lambda: rgb_pub.get_subscription_count() > 0 and depth_pub.get_subscription_count() > 0
                    and node.count_publishers(a.info_topic) > 0)
    if not ok:
        rec('startup_timeout')
        return 2
    spin_until(time.monotonic() + 2.0)
    rec('ready')
    if a.pingpong > 1:
        # continuous virtual time: each leg reverses the frame order, stamps keep increasing
        seq, vt = [], pairs_sel[0][0][0]
        legs = [pairs_sel if i % 2 == 0 else pairs_sel[::-1] for i in range(a.pingpong)]
        prev = None
        for leg in legs:
            for (rs0, rname), (ds0, dname) in leg:
                if prev is not None:
                    dt = abs(rs0 - prev)
                    if dt == 0:
                        continue  # the turning frame is not repeated
                    vt += dt
                prev = rs0
                seq.append(((vt, rname), (vt + (ds0 - rs0), dname)))
        pairs_sel = seq
        rec('pingpong', legs=a.pingpong, pairs=len(pairs_sel), span_s=pairs_sel[-1][0][0] - pairs_sel[0][0][0])
    t0_src = pairs_sel[0][0][0]
    gt_odom = None
    if gt_base is not None:
        from tools.nano_ros2.tum_gt_odom import GtOdom, first_covered, yaw_quat
        t_anchor = first_covered([r[0] for r, _ in pairs_sel], gt_base)
        if t_anchor is None:
            rec('gt_odom', error='no frame inside the groundtruth span')
            return 2
        gt_odom = GtOdom(gt_base, t_anchor)
        rec('gt_odom', anchor_stamp=t_anchor, anchor_world=[float(v) for v in gt_odom.anchor],
            covered=sum(gt_base.covers(r[0]) for r, _ in pairs_sel), frames=len(pairs_sel),
            frame=a.gt_odom_frame, child=a.gt_base_frame, mount_tf=a.gt_odom_tf)
    t0 = time.monotonic() + 0.5
    anchor_ns = time.time_ns() + 500_000_000 if a.stamp_now else None
    if a.stamp_now:
        rec('stamp_now', anchor_ns=anchor_ns, t0_src=t0_src, speed=a.speed)
        if a.speed != 1.0:
            print('tum_rgbd_pub: --stamp-now with --speed %g: stamps drift from wall time' % a.speed, flush=True)
    for k, ((rs, rname), (ds, dname)) in enumerate(pairs_sel):
        due = t0 + (rs - t0_src) / a.speed
        spin_until(due)
        late_ms = (time.monotonic() - due) * 1e3
        c0 = time.monotonic()
        rgb = cv2.imread(str(a.seq / rname), cv2.IMREAD_COLOR)
        d16 = cv2.imread(str(a.seq / dname), cv2.IMREAD_UNCHANGED)
        depth = (d16.astype(np.float32) / 5000.).astype('<f4')
        sec, nsec = frame_stamp(rs, t0_src, anchor_ns)  # one stamp for rgb, depth and camera_info
        gt_row = None
        if gt_odom is not None and gt_odom.covers(rs):
            x, y, yaw = (float(v) for v in gt_odom.at(rs))
            gt_row = [x, y, yaw]
            ts = TransformStamped()
            ts.header.stamp.sec, ts.header.stamp.nanosec = sec, nsec
            ts.header.frame_id, ts.child_frame_id = a.gt_odom_frame, a.gt_base_frame
            ts.transform.translation.x, ts.transform.translation.y = x, y
            qx, qy, qz, qw = yaw_quat(yaw)
            ts.transform.rotation.x, ts.transform.rotation.y = qx, qy
            ts.transform.rotation.z, ts.transform.rotation.w = qz, qw
            tf_pub.publish(TFMessage(transforms=[ts]))   # before the images: in the buffer at lookup

        def img(arr, enc, step):
            m = Image()
            m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
            m.header.frame_id = a.frame_id
            m.height, m.width = arr.shape[:2]
            m.encoding, m.is_bigendian, m.step = enc, 0, step
            # array.array('B') takes rclpy's fast path; plain bytes is validated per element (~0.4 s/frame).
            m.data = array.array('B', arr.tobytes())
            return m
        rm = img(rgb, 'bgr8', rgb.shape[1] * 3)
        dm = img(depth, '32FC1', depth.shape[1] * 4)
        ci = CameraInfo()
        ci.header = rm.header
        ci.height, ci.width = rgb.shape[:2]
        ci.distortion_model, ci.d = 'plumb_bob', [0.] * 5
        ci.k = [a.fx, 0., a.cx, 0., a.fy, a.cy, 0., 0., 1.]
        ci.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        ci.p = [a.fx, 0., a.cx, 0., 0., a.fy, a.cy, 0., 0., 0., 1., 0.]
        info_pub.publish(ci)
        rgb_pub.publish(rm)
        depth_pub.publish(dm)
        extra = {} if anchor_ns is None else {'pub_stamp': [sec, nsec]}
        if gt_odom is not None:
            extra['gt_odom'] = gt_row
        rec('pub', index=a.start + k, stamp=rs, depth_stamp=ds, late_ms=late_ms,
            decode_publish_ms=(time.monotonic() - c0) * 1e3, **extra)
    spin_until(time.monotonic() + a.drain)
    rec('complete', published=len(pairs_sel), counts=counts)
    node.destroy_node()
    rclpy.shutdown()
    log.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
