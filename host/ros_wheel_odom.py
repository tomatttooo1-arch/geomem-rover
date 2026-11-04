"""ROS node: python -m host.ros_wheel_odom --source chassis|pl [--kv-server HOST:PORT] (2026-10-14).

Wheel odometry, hybrid source (owner 10-14): navigation uses the counts of the Nano command serial link,
the PL odometry stays as evidence and cross-check; both go through host/wheel_odom.py (one conversion).

  --source chassis   subscribes /fpga/chassis/wheel_counts (String JSON from host/ros_chassis_driver.py,
                     every 100 Hz status frame); a frame after > 100 ms of silence starts a new baseline
  --source pl        polls the KV TELEM encoder block (host.kv_link.KvTelemetryClient, --poll-hz) and
                     uses its raw LEFT/RIGHT_COUNT; same PL publication twice is skipped; a moved PL
                     count_jumps counter or a regained encoder link starts a new baseline; LINK_OK=0 ->
                     no sample. The PL's own fixed-point distance/heading are recorded next to ours.
  -> /fpga/wheel_odom (nav_msgs/Odometry, frame_id odom, child base_link; pose from the start of this
     node, twist in base_link). Covariances are placeholders until a real-car recording.
  -> /fpga/wheel_odom/status (String JSON, 1 Hz): parameters (placeholder flag), counters, last event.

By default it does NOT broadcast odom -> base_link (or any) TF: the existing TF publisher stays the only one; EKF
fusion of this topic is later work. Opt-in (2026-10-15, real-car drive chain, tools/car_run/car_inner.sh):
  --tf-topic T        also publish odom -> base_link as tf2_msgs/TFMessage on T, same stamp and pose as the
                      Odometry (T = /tf only where nobody else publishes that edge: the live car with chassis uart;
                      replay/sim own odom -> base_link, there car_inner uses a shadow topic /fpga/wheel_odom/tf)
  --nav-odom-topic T  also publish the same Odometry on T (/odom for Nav2; RELIABLE, depth 50)
One node owns both outputs (not host/ros_odom_base_adapter.py: that relay labels its twist rtabmap_velocity_guess,
needs scipy, which no board-run node has used, and publishes /odom best-effort). Wheel parameters default to the PLACEHOLDER 65 mm / 1320 / 160 mm and
the start-up log says so. --record PATH writes one JSON line per sample (input of
tools/wheel_odom/cross_check.py).

v0.2 (2026-10-21, docs/design/CHASSIS_UART_V02_IMU_HANDOFF_20261021.md sections 8 and 10):
  --heading wheel   (default) everything above, bit-identical (tests/golden_wheel_mode.py); 0x82 frames are
                    read through their 0x81-sense left/right counts, the gyro odometry runs in shadow
                    (--record columns gyro_*, status 'gyro').
  --heading gyro    0x82 frames: heading from the raw gyro (own static bias estimate, spike filter, fallback
                    to the four-wheel yaw), translation from the four mecanum wheels (host/gyro_odom.py);
                    0x81 frames (MCU not switched): the wheel output, status 'warning'. Topics, frames and TF
                    unchanged; twist linear.y = lateral speed.
  -> <odom-topic>/wheel_heading (nav_msgs/Odometry, every mode): the wheel-only pose of every frame -- the
     pose trust gate of host/ros_runtime.py subscribes this, not /fpga/wheel_odom (gyro vs gyro otherwise)
  -> <odom-topic>/slip (String JSON {stamp_ns, trusted, flags}, gyro mode only): slip / crash verdict
     transitions (wheel vs gyro rate, redundancy, accel shock), AND-ed into that gate
  --pose-variances / --twist-variances: covariance hooks (default = the placeholders below, unchanged).
"""
import argparse
import json
import sys
import threading
import time

import math

from host.gyro_odom import (DEFAULT_ROLLERS, DEFAULT_WHEEL_MAP, DEFAULT_WHEEL_SIGNS, PLACEHOLDER_DEFAULTS,
                            FusedOdometry, FusedParams)
from host.fpga_clock import now_ns
from host.wheel_odom import (PLACEHOLDER_NOTE, PlSampleTracker, WheelOdometry, WheelParams,
                             chassis_sample, yaw_to_quaternion)

ODOM_TOPIC = '/fpga/wheel_odom'
COUNTS_TOPIC = '/fpga/chassis/wheel_counts'
WHEEL_HEADING_SUFFIX = '/wheel_heading'     # wheel-only Odometry: input of the runner's pose trust gate
SLIP_SUFFIX = '/slip'                       # gyro mode: slip / crash verdict transitions (String JSON)
HEADINGS = ('wheel', 'gyro')

JSON = json.JSONEncoder(allow_nan=False)    # == json.dumps(x, allow_nan=False), one encoder for every row
RECORD_FLUSH_S = 1.0            # --record: buffered, flushed by the 1 Hz status timer (was: every line)

# Placeholders (no real-car recording yet): pose x, y, yaw and twist vx, wz variances; the rest unused.
POSE_VAR = (1e-2, 1e-2, 1e6, 1e6, 1e6, 5e-2)
TWIST_VAR = (1e-3, 1e6, 1e6, 1e6, 1e6, 1e-2)


def diag6(values):
    out = [0.] * 36
    for i, v in enumerate(values):
        out[i * 7] = float(v)
    return out


class OdomCore:
    """Everything between a source record and an Odometry message, ROS free (tested directly).

    heading 'wheel' (default): the v0.1 path, bit-identical to before v0.2 (tests/golden_wheel_mode.py);
    0x82 frames are read through their 0x81-sense left/right counts and the gyro odometry runs in shadow
    (record and status only). heading 'gyro' (v0.2, handoff section 8): 0x82 frames drive
    host.gyro_odom.FusedOdometry (gyro heading, four-wheel translation), started from the wheel pose at the
    first 0x82 frame; 0x81 frames (MCU not switched, old firmware) keep the wheel path -- as long as no 0x82
    frame was seen the output IS the wheel output, afterwards the wheel motion of a 0x81 frame advances the
    fused pose. In both modes the wheel-only state of every frame is kept in last_wheel_state (published
    on the wheel-heading topic, what the runner's pose trust gate compares the gyro with)."""

    def __init__(self, source, params=None, record=None, heading='wheel', fused_params=None):
        if source not in ('chassis', 'pl'):
            raise ValueError(source)
        if heading not in HEADINGS:
            raise ValueError(heading)
        if heading == 'gyro' and source != 'chassis':
            raise ValueError('heading gyro needs --source chassis (0x82 frames)')
        self.source = source
        self.heading = heading
        self.odom = WheelOdometry(params)
        self.pl = PlSampleTracker()
        self.record = record
        self.last_key = None
        self.last_event = None
        self.counters = {'inputs': 0, 'published': 0, 'skipped': 0, 'bad_input': 0}
        self.fused = FusedOdometry(fused_params) if source == 'chassis' else None
        self.fused_started = False          # a 0x82 frame has been integrated
        self.mcu_frame = None
        self.frames_by_type = {'0x81': 0, '0x82': 0}
        self.last_wheel_state = None

    def _feed(self, sample, extra=None):
        if sample is None:
            self.counters['skipped'] += 1
            return None
        left, right, stamp, reason, key = sample
        if key == self.last_key:
            self.counters['skipped'] += 1
            return None
        self.last_key = key
        if reason:
            self.odom.rebaseline(reason)
        st = self.odom.update(left, right, stamp)
        self.last_event = st['event']
        if self.record is not None:
            row = {'source': self.source, 't_ns': stamp, 'left': int(left), 'right': int(right),
                   'key': list(key) if isinstance(key, tuple) else key, **st}
            row.update(extra or {})
            self.record.write(JSON.encode(row) + '\n')
        if st['event'] == 'duplicate':
            return None
        self.counters['published'] += 1
        return st

    def feed_chassis(self, row):
        """One /fpga/chassis/wheel_counts body (dict). Odometry time = Nano receive time t_ns."""
        self.counters['inputs'] += 1
        self.last_wheel_state = None
        try:
            sample = chassis_sample(row, row['t_ns'], row.get('after_silence', False))
        except (KeyError, TypeError, ValueError):
            self.counters['bad_input'] += 1
            return None
        if row.get('frame') != '0x82':
            self.frames_by_type['0x81'] += 1
            if self.mcu_frame is not None or self.heading == 'gyro':
                self.mcu_frame = '0x81'
            if self.heading == 'gyro' and self.fused_started:
                return self._wheel_into_fused(sample, row)
            st = self._feed(sample)
            if st is not None:
                st['ros_stamp_ns'] = int(row.get('stamp_ns', row['t_ns']))
            self.last_wheel_state = st
            return st
        # ---- 0x82 (v0.2)
        self.mcu_frame = '0x82'
        self.frames_by_type['0x82'] += 1
        ros_stamp = int(row.get('stamp_ns', row['t_ns']))
        fst = None
        if sample[4] != self.last_key:
            if not self.fused_started:
                self.fused.set_pose(self.odom.x, self.odom.y, self.odom.yaw)
            try:
                fst = self.fused.update(row)
            except (KeyError, TypeError, ValueError, ZeroDivisionError):
                self.counters['bad_input'] += 1
                return None
            self.fused_started = True
        extra = None if fst is None else {
            'frame': '0x82', 'gyro_x': fst['x'], 'gyro_y': fst['y'], 'gyro_yaw': fst['yaw'],
            'gyro_event': fst['event'], 'heading_source': fst['heading_source'], 'fallback': fst['fallback'],
            'bias_dps': fst['bias_dps'], 'pose_trusted': fst['pose_trusted'], 'redundancy': fst.get('redundancy'),
            'lateral': fst.get('lateral'), 'rate_raw_dps': fst.get('rate_raw_dps')}
        st = self._feed(sample, extra)
        if st is not None:
            st['ros_stamp_ns'] = ros_stamp
        self.last_wheel_state = st
        if self.heading == 'wheel':
            return st
        if fst is None or fst['event'] == 'duplicate':
            return None
        fst['ros_stamp_ns'] = ros_stamp
        return fst

    def _wheel_into_fused(self, sample, row):
        """Gyro mode, a 0x81 frame after 0x82 ones: the wheel path's motion of this frame moves the fused pose."""
        before = (self.odom.x, self.odom.y, self.odom.yaw)
        st = self._feed(sample)
        self.fused.rebaseline('0x81 frame in between')     # encoders c/d and the gyro unknown meanwhile
        if st is None:
            return None
        st['ros_stamp_ns'] = int(row.get('stamp_ns', row['t_ns']))
        self.last_wheel_state = st
        dx, dy = st['x'] - before[0], st['y'] - before[1]
        c, s = math.cos(before[2]), math.sin(before[2])
        self.fused.apply_body_delta(c * dx + s * dy, -s * dx + c * dy, st['yaw'] - before[2])
        self.fused.v, self.fused.vy, self.fused.w = st['v'], 0., st['w']
        self.fused.source = 'wheel_0x81'
        out = self.fused.state(st['event'])
        out['ros_stamp_ns'] = st['ros_stamp_ns']
        return out

    def drain_slip_transitions(self):
        """Gyro mode only: (t, trusted, flags) transitions of the slip / crash detector (handoff 8.5)."""
        if self.fused is None or self.heading != 'gyro':
            return []
        return self.fused.drain_slip_transitions()

    def feed_pl(self, block, poll_ns):
        """One decoded TELEM encoder block (host.kv_status.decode_encoder) polled at Nano time poll_ns."""
        self.counters['inputs'] += 1
        try:
            sample = self.pl.sample(block, poll_ns)
        except (KeyError, TypeError, ValueError):
            self.counters['bad_input'] += 1
            return None
        extra = None
        if sample is not None:
            extra = {'pl_dist_left_m': block.get('dist_left_m'), 'pl_dist_right_m': block.get('dist_right_m'),
                     'pl_dist_center_m': block.get('dist_center_m'), 'pl_heading_rad': block.get('heading_rad')}
        return self._feed(sample, extra)

    def status(self):
        row = {'source': self.source, 'params': self.odom.params.describe(),
               'placeholder_note': PLACEHOLDER_NOTE if self.odom.params.placeholder else None,
               'odometry': dict(self.odom.counters), 'node': dict(self.counters),
               'last_event': self.last_event, 'publishes_tf': False, 'tf_topic': None, 'nav_odom_topic': None}
        if self.fused is not None and (self.heading == 'gyro' or self.mcu_frame is not None):
            # v0.2 keys (a wheel-mode node that never saw a 0x82 frame reports exactly the v0.1 keys)
            gyro_now = self.heading == 'gyro' and self.mcu_frame == '0x82' and self.fused.source == 'gyro'
            row['heading'] = self.heading
            row['heading_source'] = 'gyro' if gyro_now else 'wheel'
            row['mcu_frame'] = self.mcu_frame
            row['frames_by_type'] = dict(self.frames_by_type)
            if self.mcu_frame == '0x81':
                row['warning'] = 'MCU not switched to 0x82: heading from the wheels only (old firmware?)'
            row['gyro'] = self.fused.status()
            row['gyro_params'] = self.fused.p.describe()
        return row


def odometry_fields(st, frame_id='odom', child_frame_id='base_link', pose_var=POSE_VAR, twist_var=TWIST_VAR):
    """Plain-dict form of the Odometry message for one state (what the ROS shell copies). linear.y is the
    four-wheel lateral speed in gyro mode (wheel-mode states have no 'vy' -> 0)."""
    qx, qy, qz, qw = yaw_to_quaternion(st['yaw'])
    return {'frame_id': frame_id, 'child_frame_id': child_frame_id,
            'position': (st['x'], st['y'], 0.), 'orientation': (qx, qy, qz, qw),
            'linear': (st['v'], st.get('vy', 0.), 0.), 'angular': (0., 0., st['w']),
            'pose_covariance': diag6(pose_var), 'twist_covariance': diag6(twist_var)}


def slip_message(t_ros_ns, trusted, flags):
    """Body of one /fpga/wheel_odom/slip String message (runner: --pose-trust-slip-topic)."""
    return {'stamp_ns': int(t_ros_ns), 'trusted': bool(trusted), 'flags': list(flags)}


def build_fused_params(args):
    return FusedParams(wheel_diam_m=args.wheel_diam_m, counts_per_rev=args.counts_per_rev,
                       track_m=args.mecanum_track_m, wheelbase_m=args.mecanum_wheelbase_m,
                       wheel_map=args.wheel_map, wheel_signs=args.wheel_signs, rollers=args.rollers,
                       lateral=args.lateral == 'on', gyro_scale_pos=args.gyro_scale_pos,
                       gyro_scale_neg=args.gyro_scale_neg, wheel_yaw_scale=args.wheel_yaw_scale,
                       static_gyro_std_dps=args.static_gyro_std_dps, static_accel_std_g=args.static_accel_std_g,
                       bias_max_dps=args.bias_max_dps, slip_rate_dps=args.slip_rate_dps,
                       redundancy_mps=args.redundancy_mps, shock_g=args.shock_g)


def var6(text, default):
    """'a,b,c,d,e,f' -> six variances; '' -> default."""
    if not text:
        return default
    vals = tuple(float(v) for v in text.split(','))
    if len(vals) != 6:
        raise ValueError('need six comma-separated variances')
    return vals


def build_params(args):
    return WheelParams(wheel_diam_m=args.wheel_diam_m, counts_per_rev=args.counts_per_rev,
                       track_m=args.track_m, max_delta_counts=args.max_delta_counts)


def parse_args(argv=None):
    """(args, ros_args, WheelParams, FusedParams, pose_var, twist_var); ROS free so the CLI is tested on the host."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--source', choices=('chassis', 'pl'), required=True)
    parser.add_argument('--counts-topic', default=COUNTS_TOPIC)
    parser.add_argument('--kv-server', help='HOST:PORT of kv_server (--source pl)')
    parser.add_argument('--poll-hz', type=float, default=50.)
    parser.add_argument('--odom-topic', default=ODOM_TOPIC)
    parser.add_argument('--frame-id', default='odom')
    parser.add_argument('--child-frame-id', default='base_link')
    parser.add_argument('--wheel-diam-m', type=float, default=WheelParams().wheel_diam_m)
    parser.add_argument('--counts-per-rev', type=int, default=WheelParams().counts_per_rev)
    parser.add_argument('--track-m', type=float, default=WheelParams().track_m)
    parser.add_argument('--max-delta-counts', type=int, default=WheelParams().max_delta_counts)
    parser.add_argument('--record', help='append one JSON line per sample')
    parser.add_argument('--tf-topic', default='', help="publish odom -> base_link as TFMessage here ('' = no TF)")
    parser.add_argument('--nav-odom-topic', default='', help="also publish the Odometry here ('' = off)")
    # ---- v0.2 (docs/design/CHASSIS_UART_V02_IMU_HANDOFF_20261021.md sections 8 and 10)
    v2 = parser.add_argument_group('v0.2 gyro heading (0x82 frames; PLACEHOLDER = not measured yet)')
    v2.add_argument('--heading', choices=HEADINGS, default='wheel',
                    help='wheel (default): v0.1 output, bit-identical; gyro: gyro heading + four-wheel translation')
    v2.add_argument('--wheel-heading-topic', default=None,
                    help='wheel-only Odometry (every mode), the pose trust gate input; default <odom-topic>%s; '
                         "'' = off" % WHEEL_HEADING_SUFFIX)
    v2.add_argument('--slip-topic', default=None,
                    help="gyro mode: slip/crash verdict transitions (String JSON); default <odom-topic>%s; '' = off"
                         % SLIP_SUFFIX)
    v2.add_argument('--mecanum-track-m', type=float, default=0.205, help='left-right wheel distance (dev2 10-16)')
    v2.add_argument('--mecanum-wheelbase-m', type=float, default=0.225, help='front-rear wheel distance (dev2 10-16)')
    v2.add_argument('--wheel-map', default=DEFAULT_WHEEL_MAP, help='encoder -> wheel position (PLACEHOLDER)')
    v2.add_argument('--wheel-signs', default=DEFAULT_WHEEL_SIGNS,
                    help='per encoder a..d, +1 = counts up when that wheel rolls forward (PLACEHOLDER)')
    v2.add_argument('--rollers', choices=('X', 'O'), default=DEFAULT_ROLLERS, help='roller pattern (PLACEHOLDER)')
    v2.add_argument('--lateral', choices=('on', 'off'), default='on', help='integrate the lateral (strafe) motion')
    v2.add_argument('--gyro-scale-pos', type=float, default=PLACEHOLDER_DEFAULTS['gyro_scale_pos'],
                    help='gyro scale for counter-clockwise turns (PLACEHOLDER 1.0 until section 8.7)')
    v2.add_argument('--gyro-scale-neg', type=float, default=PLACEHOLDER_DEFAULTS['gyro_scale_neg'],
                    help='gyro scale for clockwise turns (PLACEHOLDER 1.0)')
    v2.add_argument('--wheel-yaw-scale', type=float, default=PLACEHOLDER_DEFAULTS['wheel_yaw_scale'],
                    help='scale of the four-wheel yaw in fallback (PLACEHOLDER 1.0)')
    v2.add_argument('--static-gyro-std-dps', type=float, default=PLACEHOLDER_DEFAULTS['static_gyro_std_dps'],
                    help='G of section 8.3 (PLACEHOLDER; T2 static std x 3)')
    v2.add_argument('--static-accel-std-g', type=float, default=PLACEHOLDER_DEFAULTS['static_accel_std_g'],
                    help='A of section 8.3 (PLACEHOLDER; T2 static std x 3)')
    v2.add_argument('--bias-max-dps', type=float, default=None,
                    help='opt-in: refuse a static bias estimate above this (not in the spec; default off)')
    v2.add_argument('--slip-rate-dps', type=float, default=PLACEHOLDER_DEFAULTS['slip_rate_dps'],
                    help='wheel vs gyro rate threshold, held 0.3 s (PLACEHOLDER)')
    v2.add_argument('--redundancy-mps', type=float, default=PLACEHOLDER_DEFAULTS['redundancy_mps'],
                    help='|(FL+FR)-(RL+RR)| rate threshold over 0.2 s (PLACEHOLDER)')
    v2.add_argument('--shock-g', type=float, default=PLACEHOLDER_DEFAULTS['shock_g'],
                    help='horizontal accel step threshold vs the 1 s mean (PLACEHOLDER)')
    v2.add_argument('--pose-variances', default='',
                    help="six pose variances x,y,z,roll,pitch,yaw ('' = the current placeholders)")
    v2.add_argument('--twist-variances', default='', help="six twist variances ('' = the current placeholders)")
    args, ros_args = parser.parse_known_args(argv)
    if args.source == 'pl' and not args.kv_server:
        parser.error('--source pl needs --kv-server HOST:PORT')
    if not args.poll_hz > 0:
        parser.error('--poll-hz must be positive')
    if args.heading == 'gyro' and args.source != 'chassis':
        parser.error('--heading gyro needs --source chassis')
    if args.wheel_heading_topic is None:
        args.wheel_heading_topic = args.odom_topic + WHEEL_HEADING_SUFFIX
    if args.slip_topic is None:
        args.slip_topic = args.odom_topic + SLIP_SUFFIX
    try:
        params = build_params(args)
        fused_params = build_fused_params(args)
        pose_var, twist_var = var6(args.pose_variances, POSE_VAR), var6(args.twist_variances, TWIST_VAR)
    except ValueError as error:
        parser.error(str(error))
    return args, ros_args, params, fused_params, pose_var, twist_var


def main(argv=None):
    args, ros_args, params, fused_params, pose_var, twist_var = parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    if args.tf_topic:
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

    class WheelOdomNode(Node):
        def __init__(self):
            super().__init__('fpga_wheel_odom')
            # 10-29: buffered (100 rows/s were 100 write syscalls/s); publish_status flushes it every second
            self.record = open(args.record, 'a') if args.record else None
            self.core = OdomCore(args.source, params, self.record, heading=args.heading,
                                 fused_params=fused_params)
            self.lock = threading.Lock()
            self.pub = self.create_publisher(Odometry, args.odom_topic, 50)
            self.wheel_pub = (self.create_publisher(Odometry, args.wheel_heading_topic, 50)
                              if args.wheel_heading_topic else None)
            self.slip_pub = (self.create_publisher(String, args.slip_topic, 10)
                             if args.slip_topic and args.heading == 'gyro' else None)
            self.nav_pub = self.create_publisher(Odometry, args.nav_odom_topic, 50) if args.nav_odom_topic else None
            self.tf_pub = self.create_publisher(TFMessage, args.tf_topic, 100) if args.tf_topic else None
            self.status_pub = self.create_publisher(String, args.odom_topic + '/status', 10)
            log = self.get_logger()
            log.info('wheel odometry source=%s params=%s' % (args.source, json.dumps(params.describe())))
            log.info('heading=%s wheel-heading topic=%s slip topic=%s gyro params=%s' % (
                args.heading, args.wheel_heading_topic or '-', args.slip_topic if self.slip_pub else '-',
                json.dumps(fused_params.describe())))
            if args.heading == 'gyro' and fused_params.placeholders():
                log.warn('PLACEHOLDER gyro-odometry parameters in use: ' + ', '.join(fused_params.placeholders()))
            if params.placeholder:
                log.warn('PLACEHOLDER wheel parameters in use: ' + PLACEHOLDER_NOTE)
            if self.tf_pub is None:
                log.info('this node publishes no TF (odom -> base_link stays with the existing publisher)')
            else:
                log.info('odom -> base_link TF on %s; Odometry also on %s' % (args.tf_topic, args.nav_odom_topic or '-'))
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(RECORD_FLUSH_S, self.publish_status, clock=steady)
            # 10-29: one Odometry per output (and one TF message) reused for every frame, constant fields
            # (frames, covariances) set once; publish copies synchronously. Building two Odometry per frame
            # (nested constructors + 36-element covariance checks) cost more than publishing them.
            self.odom_template = self.odometry_template()
            self.wheel_template = self.odometry_template()
            if self.tf_pub is not None:
                self.tf_t = TransformStamped()
                self.tf_t.header.frame_id, self.tf_t.child_frame_id = args.frame_id, args.child_frame_id
                self.tf_msg = TFMessage(transforms=[self.tf_t])
            self.running = True
            self.poller = None
            if args.source == 'chassis':
                self.create_subscription(String, args.counts_topic, self.on_counts, 50)
            else:
                from host.kv_link import KvTelemetryClient, blocks_mask
                host, _, port = args.kv_server.rpartition(':')
                self.client = KvTelemetryClient((host, int(port)), timeout_s=0.2, hello_timeout_s=3.0)
                self.want = blocks_mask(('encoder', 'board'))
                self.poll_errors = 0
                self.poller = threading.Thread(target=self.poll_loop, name='kv-encoder-poll', daemon=True)
                self.poller.start()

        def odometry_template(self):
            f = odometry_fields({'x': 0., 'y': 0., 'yaw': 0., 'v': 0., 'w': 0.}, args.frame_id, args.child_frame_id,
                                pose_var, twist_var)
            msg = Odometry()
            msg.header.frame_id, msg.child_frame_id = f['frame_id'], f['child_frame_id']
            msg.pose.covariance = f['pose_covariance']
            msg.twist.covariance = f['twist_covariance']
            return msg

        def odometry_msg(self, st, ros_stamp_ns, msg=None):
            """msg: a template from odometry_template() to fill (every per-frame field is written; frames and
            covariances are constant); None = a new message, as before."""
            f = odometry_fields(st, args.frame_id, args.child_frame_id, pose_var, twist_var)
            fresh = msg is None
            if fresh:
                msg = Odometry()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(int(ros_stamp_ns), 10**9)
            msg.header.frame_id, msg.child_frame_id = f['frame_id'], f['child_frame_id']
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            p.x, p.y, p.z = f['position']
            q.x, q.y, q.z, q.w = f['orientation']
            msg.twist.twist.linear.x = f['linear'][0]
            msg.twist.twist.linear.y = f['linear'][1]
            msg.twist.twist.angular.z = f['angular'][2]
            if fresh:
                msg.header.frame_id, msg.child_frame_id = f['frame_id'], f['child_frame_id']
                msg.pose.covariance = f['pose_covariance']
                msg.twist.covariance = f['twist_covariance']
            return msg, p, q

        def publish(self, st, ros_stamp_ns):
            msg, p, q = self.odometry_msg(st, ros_stamp_ns, self.odom_template)
            self.pub.publish(msg)
            if self.nav_pub is not None:
                self.nav_pub.publish(msg)
            if self.tf_pub is not None:
                t = self.tf_t
                t.header.stamp.sec, t.header.stamp.nanosec = msg.header.stamp.sec, msg.header.stamp.nanosec
                t.transform.translation.x, t.transform.translation.y = p.x, p.y
                r = t.transform.rotation
                r.x, r.y, r.z, r.w = q.x, q.y, q.z, q.w
                self.tf_pub.publish(self.tf_msg)

        def on_counts(self, msg):
            try:
                row = json.loads(msg.data)
            except ValueError:
                with self.lock:
                    self.core.counters['bad_input'] += 1
                return
            with self.lock:
                st = self.core.feed_chassis(row)
                wheel = self.core.last_wheel_state
                slips = self.core.drain_slip_transitions()
            if st is not None:
                self.publish(st, st['ros_stamp_ns'])
            if wheel is not None and self.wheel_pub is not None:
                self.wheel_pub.publish(self.odometry_msg(wheel, wheel['ros_stamp_ns'], self.wheel_template)[0])
            if self.slip_pub is not None:
                stamp = int(row.get('stamp_ns', row.get('t_ns', 0)))
                for _, trusted, flags in slips:     # every transition belongs to this frame
                    self.slip_pub.publish(String(data=json.dumps(slip_message(stamp, trusted, flags))))

        def poll_loop(self):
            from host.kv_link import LinkError
            period = 1. / args.poll_hz
            while self.running:
                t0 = now_ns()
                try:
                    tm = self.client.poll(self.want)
                except (LinkError, OSError) as error:
                    self.poll_errors += 1
                    if self.poll_errors in (1, 10, 100) or self.poll_errors % 1000 == 0:
                        self.get_logger().error('TELEM poll failed (%d so far): %s' % (self.poll_errors, error))
                else:
                    t1 = now_ns()
                    ros_now = self.get_clock().now().nanoseconds
                    with self.lock:
                        st = self.core.feed_pl(tm.blocks.get('encoder'), t1)
                    if st is not None:
                        self.publish(st, ros_now - (t1 - st['stamp_ns']))
                        if self.wheel_pub is not None:          # the PL path is wheel-only by construction
                            self.wheel_pub.publish(self.odometry_msg(st, ros_now - (t1 - st['stamp_ns']),
                                                                     self.wheel_template)[0])
                time.sleep(max(0., period - (now_ns() - t0) / 1e9))

        def publish_status(self):
            with self.lock:
                row = self.core.status()
            row['publishes_tf'] = self.tf_pub is not None
            row['tf_topic'], row['nav_odom_topic'] = args.tf_topic or None, args.nav_odom_topic or None
            if self.poller is not None:
                row['poll_errors'] = self.poll_errors
            self.status_pub.publish(String(data=json.dumps(row, allow_nan=False)))
            if self.record is not None:
                with self.lock:                 # the core writes rows under this lock (PL poller thread)
                    self.record.flush()

        def stop(self):
            self.running = False
            if self.poller is not None:
                self.poller.join(timeout=1.)
            if self.record is not None:
                self.record.close()

    import signal
    from rclpy.signals import SignalHandlerOptions

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupt)
    node = None
    try:
        node = WheelOdomNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
