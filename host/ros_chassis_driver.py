"""ROS node: python -m host.ros_chassis_driver --port /dev/fpga_chassis [--baud 115200].

Nano side of the chassis command serial link, protocol v0.1 (docs/design/CHASSIS_UART_PROTOCOL_20261012.md)
plus v0.2 (docs/design/CHASSIS_UART_V02_IMU_HANDOFF_20261021.md section 8.1, 2026-10-21): every command
carries flag b2 (--status2 on, default) and the parser takes both 0x81 and 0x82. An MCU on old firmware
keeps sending 0x81; then everything below works as in v0.1 and the status reports mcu_frame 0x81 plus a
warning. With 0x82: /imu/data_raw carries the RAW mean rate gz_sum/gz_n (not bias-corrected; gz_n = 0 ->
not published), /fpga/chassis/wheel_counts carries every 0x82 field and the last sent command in the same
message (wheel_counts_row), --record gets every field of every 0x82 frame (record_row).
  /fpga/cmd_vel_safe (Twist)  -> command frame 0x01, 50 Hz on a steady clock, last command repeated;
                                enable=0 when the upstream is older than 200 ms or no status frame came
                                for 100 ms (host.chassis_uart.ChassisLink decides, tested without ROS)
  status frame 0x81 (100 Hz) -> /imu/data_raw (sensor_msgs/Imu, only angular_velocity.z; orientation and
                                linear acceleration marked unavailable with covariance[0] = -1). Frames with
                                the gyro-valid bit clear are NOT published: consumers see a gap, not a zero.
                             -> /fpga/chassis/battery (sensor_msgs/BatteryState) when the MCU samples it
                             -> /fpga/chassis/wheel_counts (String JSON, every status frame): raw cumulative
                                left/right counts with seq, mcu_ms, the ROS stamp and the Nano receive time;
                                input of host/ros_wheel_odom.py --source chassis (wheel_counts_row below)
                             -> /fpga/chassis/status (String JSON, 2 Hz and on change of the MCU status bits):
                                MCU bits, counters (lost status frames, CRC errors, unreported commands),
                                command round-trip p50/p90/max
  static TF base_link -> imu_link (identity: the MCU already reports yaw rate in the vehicle convention,
                                counter-clockwise seen from above positive; protocol section 1)
  service ~/recalibrate_gyro (std_srvs/Trigger): sends flag b1 with enable=0 until the MCU reports b6.

Wheel odometry (owner 10-14, hybrid): navigation uses these counts through host/ros_wheel_odom.py; the
PL keeps its own odometry from the encoder frame as evidence and cross-check (host/wheel_odom.py is the
one conversion both use). This node publishes counts only, no odometry and no odom TF.
--record PATH appends every received frame and every sent command as JSON lines (for the bench and the
first static recording). The serial port is read on its own thread; ROS callbacks and that thread share
the ChassisLink behind one lock.
"""
import argparse
import json
import math
import os
import select
import signal
import sys
import threading
import time

from host.chassis_uart import (BAUD, TEMP_UNAVAILABLE, TYPE_STATUS, TYPE_STATUS2, ChassisLink, FrameParser,
                               open_serial, write_all)
from host.fpga_clock import now_ns

STATUS_TOPIC = '/fpga/chassis/status'
COUNTS_TOPIC = '/fpga/chassis/wheel_counts'
RECORD_FLUSH_S = 0.5            # --record: buffered, flushed at most this long after a row (was: every line)
JSON = json.JSONEncoder(allow_nan=False)    # == json.dumps(x, allow_nan=False), without a new encoder per call


STATUS2_COUNTS_FIELDS = ('enc_a', 'enc_b', 'enc_c', 'enc_d', 'gz_sum', 'gz_n', 'imu_err', 'gx_mean', 'gy_mean',
                         'ax', 'ay', 'az', 'yaw_raw', 'gyro_bias_cdps', 'status')
ENCODERS = ('a', 'b', 'c', 'd')


def status2_legacy_counts(row, legacy_left='a', legacy_right='b'):
    """(left, right) of a 0x82 row in the 0x81 sense: the two encoders the 0x81 frame reports (handoff
    section 0: A and B; PLACEHOLDER until dev2 confirms which TIMs feed 0x81 left/right)."""
    return row['enc_' + legacy_left], row['enc_' + legacy_right]


def wheel_counts_row(row, derived, ros_stamp_ns, command=None, legacy_left='a', legacy_right='b'):
    """JSON body of one /fpga/chassis/wheel_counts message (tested without ROS).

    0x81 rows: exactly the v0.1 body (no new keys). 0x82 rows (v0.2, CHASSIS_UART_V02_IMU_HANDOFF section
    8.1): the same keys (left_count / right_count = the 0x81 pair, so v0.1 consumers keep working) plus
    'frame': '0x82', every raw IMU / encoder field of the frame (temp_cdeg -32768 -> None) and the last
    command the Nano sent ('cmd': [v_mm_s, w_mrad_s, enable], None before the first) -- one message per
    frame, nothing split into a second topic."""
    if row.get('type') != 'status2':
        return {'stamp_ns': int(ros_stamp_ns), 't_ns': derived['received_ns'], 'seq': row['seq'],
                'mcu_ms': row['mcu_ms'], 'left_count': row['left_count'], 'right_count': row['right_count'],
                'after_silence': bool(derived['after_silence']), 'lost_before': derived['lost_before'],
                'motor_enabled': row['motor_enabled']}
    left, right = status2_legacy_counts(row, legacy_left, legacy_right)
    body = {'stamp_ns': int(ros_stamp_ns), 't_ns': derived['received_ns'], 'seq': row['seq'],
            'mcu_ms': row['mcu_ms'], 'left_count': left, 'right_count': right,
            'after_silence': bool(derived['after_silence']), 'lost_before': derived['lost_before'],
            'motor_enabled': row['motor_enabled'], 'frame': '0x82'}
    body.update({k: row[k] for k in STATUS2_COUNTS_FIELDS})
    body['temp_cdeg'] = None if row['temp_cdeg'] == TEMP_UNAVAILABLE else row['temp_cdeg']
    body['cmd'] = None if command is None else [int(command[0]), int(command[1]), bool(command[2])]
    return body


def record_row(row, derived, legacy_left='a', legacy_right='b'):
    """--record line of one received frame: every decoded field ('type' dropped); 0x82 rows also carry
    'frame': '0x82' and the 0x81-sense left_count / right_count (tools/wheel_odom/cross_check.py reads those)."""
    out = {'dir': 'rx', 't_ns': derived['received_ns'], 'lost_before': derived['lost_before'],
           'after_silence': derived['after_silence'],
           'rtt_ms': None if derived['rtt_ns'] is None else derived['rtt_ns'] / 1e6,
           **{k: v for k, v in row.items() if k != 'type'}}
    if row.get('type') == 'status2':
        out['frame'] = '0x82'
        out['left_count'], out['right_count'] = status2_legacy_counts(row, legacy_left, legacy_right)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', default='/dev/fpga_chassis',
                        help='serial device (Nano 40-pin UART: see tools/chassis_uart/nano_uart_setup.sh)')
    parser.add_argument('--baud', type=int, default=BAUD)
    parser.add_argument('--command-hz', type=float, default=50.)
    parser.add_argument('--cmd-topic', default='/fpga/cmd_vel_safe')
    parser.add_argument('--imu-topic', default='/imu/data_raw')
    parser.add_argument('--counts-topic', default=COUNTS_TOPIC,
                        help="raw wheel counts as String JSON; '' disables")
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--imu-frame', default='imu_link')
    parser.add_argument('--gyro-z-variance', type=float, default=1e-4,
                        help='(rad/s)^2 for angular_velocity_covariance[8]; to be set from the static recording')
    parser.add_argument('--v-limit-mm-s', type=int, default=500)
    parser.add_argument('--w-limit-mrad-s', type=int, default=1500)
    parser.add_argument('--status-hz', type=float, default=2.)
    parser.add_argument('--record', help='append received frames and sent commands as JSON lines')
    parser.add_argument('--status2', choices=('on', 'off'), default='on',
                        help='v0.2: set command flag b2 so the MCU sends 0x82 (raw IMU + four encoders); '
                             'an MCU on old firmware keeps sending 0x81 and is reported in the status')
    parser.add_argument('--legacy-left-enc', choices=ENCODERS, default='a',
                        help='0x82 encoder carried as left_count (the 0x81 left; PLACEHOLDER a, handoff section 0)')
    parser.add_argument('--legacy-right-enc', choices=ENCODERS, default='b',
                        help='0x82 encoder carried as right_count (PLACEHOLDER b)')
    args, ros_args = parser.parse_known_args(argv)
    if not args.command_hz > 0 or not args.status_hz > 0:
        parser.error('rates must be positive')

    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped, Twist
    from sensor_msgs.msg import BatteryState, Imu
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

    class ChassisDriver(Node):
        def __init__(self, fd):
            super().__init__('fpga_chassis_driver')
            self.fd = fd
            self.lock = threading.Lock()
            self.link = ChassisLink(v_limit_mm_s=args.v_limit_mm_s, w_limit_mrad_s=args.w_limit_mrad_s,
                                    status2=args.status2 == 'on')
            self.parser = FrameParser(accept={TYPE_STATUS, TYPE_STATUS2})
            self.warned_v01 = False
            # 10-29: buffered (150 rows/s used to be 150 write syscalls/s), flushed every RECORD_FLUSH_S
            self.record = open(args.record, 'a') if args.record else None
            self.record_lock = threading.Lock()
            self.record_flushed = time.monotonic()
            self.rx_reads = 0
            # 10-29: one Imu / BatteryState reused for every frame (publish copies synchronously); building
            # two Imu per frame (one only for its stamp) cost more than the publish itself
            self.imu_msg = Imu()
            self.imu_msg.header.frame_id = args.imu_frame
            self.imu_msg.orientation_covariance[0] = -1.
            self.imu_msg.linear_acceleration_covariance[0] = -1.
            self.imu_msg.angular_velocity_covariance = [1e6, 0., 0., 0., 1e6, 0., 0., 0., args.gyro_z_variance]
            self.battery_msg = BatteryState()
            self.battery_msg.present = True
            b = self.battery_msg
            b.percentage = b.current = b.charge = b.capacity = math.nan
            b.design_capacity = b.temperature = math.nan
            self.last_bits = None
            self.write_errors = self.read_errors = 0
            self.imu_pub = self.create_publisher(Imu, args.imu_topic, qos_profile_sensor_data)
            self.battery_pub = self.create_publisher(BatteryState, '/fpga/chassis/battery', 10)
            self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
            self.counts_pub = (self.create_publisher(String, args.counts_topic, 50) if args.counts_topic
                               else None)
            self.create_subscription(Twist, args.cmd_topic, self.velocity, 10)
            self.create_service(Trigger, '~/recalibrate_gyro', self.recalibrate)
            steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1. / args.command_hz, self.send_command, clock=steady)
            self.create_timer(1. / args.status_hz, self.publish_status, clock=steady)
            static = TransformStamped()
            static.header.stamp = self.get_clock().now().to_msg()
            static.header.frame_id, static.child_frame_id = args.base_frame, args.imu_frame
            static.transform.rotation.w = 1.
            self.static_tf = StaticTransformBroadcaster(self)
            self.static_tf.sendTransform(static)
            self.running = True
            self.reader = threading.Thread(target=self.read_loop, name='chassis-uart-rx', daemon=True)
            self.reader.start()

        def log_row(self, row):
            if self.record is not None:
                text = JSON.encode(row) + '\n'
                with self.record_lock:          # rows come from the rx thread and the command timer
                    self.record.write(text)
                    now = time.monotonic()
                    if now - self.record_flushed >= RECORD_FLUSH_S:
                        self.record.flush()
                        self.record_flushed = now

        def flush_record(self):
            if self.record is not None:
                with self.record_lock:
                    self.record.flush()
                    self.record_flushed = time.monotonic()

        def velocity(self, msg):
            with self.lock:
                self.link.on_velocity(msg.linear.x, msg.angular.z, now_ns())

        def recalibrate(self, request, response):
            with self.lock:
                self.link.request_recalibration(now_ns())
            response.success, response.message = True, 'recalibration requested; enable held at 0 until the MCU reports b6 (max 1 s)'
            return response

        def send_command(self):
            with self.lock:
                frame, decision = self.link.next_command(now_ns())
            try:
                write_all(self.fd, frame)
            except (OSError, TimeoutError) as error:
                self.write_errors += 1
                if self.write_errors in (1, 10, 100) or self.write_errors % 1000 == 0:
                    self.get_logger().error('serial write failed (%d so far): %s' % (self.write_errors, error))
                return
            decision['t_ns'] = now_ns()
            self.log_row({'dir': 'tx', **decision})

        def read_loop(self):
            while self.running:
                try:
                    ready = select.select([self.fd], [], [], 0.05)[0]
                    if not ready:
                        continue
                    data = os.read(self.fd, 4096)
                except BlockingIOError:
                    continue
                except OSError as error:
                    self.read_errors += 1
                    if self.read_errors in (1, 10, 100) or self.read_errors % 1000 == 0:
                        self.get_logger().error('serial read failed (%d so far): %s' % (self.read_errors, error))
                    time.sleep(0.1)
                    continue
                if not data:                    # hang-up (a pty whose other end closed): do not spin
                    time.sleep(0.01)
                    continue
                received = now_ns()
                self.rx_reads += 1
                frames = self.parser.feed(data)
                if not frames:                  # a piece of a frame: nothing to stamp or publish yet
                    continue
                # ROS time of a raw-clock instant = raw + (ROS now - raw now), the two read back to back
                ros_offset = self.get_clock().now().nanoseconds - now_ns()
                for row in frames:
                    with self.lock:
                        derived = self.link.on_status(row, received)
                        command = self.link.last_command
                    self.publish_frame(row, derived, derived['stamp_ns'] + ros_offset, command)

        def publish_frame(self, row, derived, ros_stamp_ns, command=None):
            sec, nanosec = divmod(ros_stamp_ns, 10**9)
            if derived['gyro_z_rad_s'] is not None:
                imu = self.imu_msg
                imu.header.stamp.sec, imu.header.stamp.nanosec = sec, nanosec
                imu.angular_velocity.z = derived['gyro_z_rad_s']
                self.imu_pub.publish(imu)
            if derived['battery_v'] is not None:
                battery = self.battery_msg
                battery.header.stamp.sec, battery.header.stamp.nanosec = sec, nanosec
                battery.voltage = derived['battery_v']
                self.battery_pub.publish(battery)
            if self.counts_pub is not None:
                body = wheel_counts_row(row, derived, ros_stamp_ns, command, args.legacy_left_enc,
                                        args.legacy_right_enc)
                self.counts_pub.publish(String(data=JSON.encode(body)))
            self.log_row(record_row(row, derived, args.legacy_left_enc, args.legacy_right_enc))
            if self.link.status2 and row.get('type') != 'status2' and not self.warned_v01:
                self.warned_v01 = True
                self.get_logger().warn('0x82 requested (flag b2) but the MCU sends 0x81: old firmware? '
                                       'odometry falls back to wheels only')
            if row['status'] != self.last_bits:
                if self.last_bits is not None:
                    self.get_logger().info('MCU status bits 0x%02x -> 0x%02x' % (self.last_bits, row['status']))
                self.last_bits = row['status']
                self.publish_status()

        def publish_status(self):
            with self.lock:
                row = self.link.status(now_ns())
                cmd = self.link.last_command
            # 10-29 health check (host/hub/health.py): the last command built [v mm/s, w mrad/s, enable], so a
            # commanded motion without encoder movement (the MCU ignores serial while its remote is on) shows
            row['cmd'] = None if cmd is None else list(cmd)
            row['parser'] = dict(self.parser.counters)
            row['write_errors'], row['read_errors'] = self.write_errors, self.read_errors
            row['port'] = args.port
            row['rx_reads'] = self.rx_reads     # 10-29: serial reads (wake-ups) vs parser frames
            self.status_pub.publish(String(data=json.dumps(row, allow_nan=False)))
            self.flush_record()

        def stop(self):
            self.running = False
            self.reader.join(timeout=1.)
            # Leave the MCU commanded to stop rather than relying on its 200 ms timeout.
            for _ in range(3):
                with self.lock:
                    self.link.upstream = None
                    frame, _ = self.link.next_command(now_ns())
                try:
                    write_all(self.fd, frame)
                except (OSError, TimeoutError):
                    break
            if self.record is not None:
                self.record.close()

    # As host/ros_runtime.spin_until_interrupt: rclpy installs no signal handler, SIGINT/SIGTERM raise
    # KeyboardInterrupt (the 50 Hz timer returns spin to Python), so stop() still sends enable=0 frames
    # on a live context and the exit status is 0.
    from rclpy.signals import SignalHandlerOptions

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    fd = open_serial(args.port, args.baud)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupt)
    node = None
    try:
        node = ChassisDriver(fd)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        os.close(fd)
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
