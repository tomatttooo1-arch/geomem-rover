"""Nano <-> chassis MCU command serial link, protocol v0.1 (docs/design/CHASSIS_UART_PROTOCOL_20261012.md).

Frames: 0xA5 0x5A, type, fixed-length body, CRC-16/CCITT-FALSE over sync..body (every byte before the CRC,
the two sync bytes included: owner 10-12 night, matching the MCU firmware), CRC low byte first.
  0x01 command  Nano -> MCU, 11 bytes, every 20 ms: seq, v mm/s, w mrad/s, flags (b0 enable, b1 recalibrate)
  0x81 status   MCU -> Nano, 24 bytes, every 10 ms: seq, mcu_ms, left/right counts, gyro z, gyro bias,
                battery mV, last_cmd_seq, status bits
  0x82 status2  MCU -> Nano, 52 bytes, every 10 ms instead of 0x81 while the latest command has b2 set (v0.2,
                CHASSIS_UART_V02_IMU_HANDOFF_20261021.md): 32-bit mcu_ms, four encoders, raw gyro z sum + sample
                count, raw gyro x/y, accelerometer, module yaw, temperature, IMU error count
All multi-byte fields little-endian. This is a separate line from the MCU -> KV260 encoder frame
(PL_PERMIT_UART_SPEC_20261011.md section 4.1, 0xAA 0x55, 15 bytes), which is not touched here.

Everything here is I/O free except open_serial(): the frame codec, a stream parser that resyncs after
garbage, and ChassisLink, the Nano-side policy (when to send enable, how to count lost frames and
measure the command round trip). The ROS node host/ros_chassis_driver.py only moves bytes and messages.
Time is always passed in as integer nanoseconds of one monotonic clock.
"""
import binascii
import math
import os
import struct
import termios

SYNC = b'\xa5\x5a'
TYPE_COMMAND = 0x01
TYPE_STATUS = 0x81
TYPE_STATUS2 = 0x82
FRAME_LENGTHS = {TYPE_COMMAND: 11, TYPE_STATUS: 24, TYPE_STATUS2: 52}
BAUD = 115200
BITS_PER_BYTE = 10                       # 8N1: start + 8 data + stop

COMMAND_BODY = struct.Struct('<BhhB')    # seq, v, w, flags
STATUS_BODY = struct.Struct('<BHiihhHBB')  # seq, mcu_ms, left, right, gyro_z, gyro_bias, battery, last_cmd, status
STATUS2_FIELDS = ('seq', 'mcu_ms', 'enc_a', 'enc_b', 'enc_c', 'enc_d', 'gz_sum', 'gz_n', 'imu_err', 'gx_mean',
                  'gy_mean', 'ax', 'ay', 'az', 'yaw_raw', 'temp_cdeg', 'gyro_bias_cdps', 'battery_mv',
                  'last_cmd_seq', 'status')
STATUS2_BODY = struct.Struct('<BIiiiiiBBhhhhhhhhHBB')
GYRO_RAW_DEG_S = 2000. / 32768              # 0x82 raw units (IMU fixed at +-2000 deg/s, +-16 g)
ACCEL_RAW_G = 16. / 32768
YAW_RAW_DEG = 180. / 32768

FLAG_ENABLE = 0x01
FLAG_RECALIBRATE = 0x02
FLAG_STATUS2 = 0x04                      # ask the MCU for 0x82 instead of 0x81

STATUS_BITS = ('heartbeat_seen', 'motor_enabled', 'stop_latched', 'command_timeout',
               'gyro_valid', 'gyro_calibrated', 'gyro_calibrating', 'driver_fault')

STATUS_STATUS_KEYS = ('seq', 'mcu_ms', 'left_count', 'right_count', 'gyro_z_cdps', 'gyro_bias_cdps',
                      'battery_mv', 'last_cmd_seq') + STATUS_BITS
STATUS2_STATUS_KEYS = tuple(k for k in STATUS2_FIELDS if k != 'status') + STATUS_BITS
TEMP_UNAVAILABLE = -32768

INT16_MIN, INT16_MAX = -32768, 32767
CENTI_DEG_TO_RAD = math.pi / 18000.


def _crc_table():
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
        table.append(crc & 0xFFFF)
    return tuple(table)


_CRC_TABLE = _crc_table()


def crc16_ccitt_false_table(data, crc=0xFFFF):
    """Table-driven reference of crc16_ccitt_false (kept for the equality test)."""
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]
    return crc


def crc16_ccitt_false(data, crc=0xFFFF):
    """Poly 0x1021, init 0xFFFF, no reflection, no final xor; b'123456789' -> 0x29B1.
    10-29: binascii.crc_hqx is exactly this CRC (CCITT poly, MSB first, caller's init) in C -- the chassis
    driver checks 100 frames/s; the Python loop above stays as the reference."""
    return binascii.crc_hqx(data, crc)


def _frame(frame_type, body):
    covered = SYNC + bytes((frame_type,)) + body
    return covered + struct.pack('<H', crc16_ccitt_false(covered))


def airtime_ns(frame_type, baud=BAUD):
    """Time the whole frame occupies the line; its last byte arrives this long after the first started."""
    return FRAME_LENGTHS[frame_type] * BITS_PER_BYTE * 10**9 // baud


def saturate_int16(value):
    return max(INT16_MIN, min(INT16_MAX, int(value)))


def encode_command(seq, v_mm_s, w_mrad_s, enable, recalibrate=False, status2=False):
    flags = ((FLAG_ENABLE if enable else 0) | (FLAG_RECALIBRATE if recalibrate else 0)
             | (FLAG_STATUS2 if status2 else 0))
    return _frame(TYPE_COMMAND, COMMAND_BODY.pack(seq & 0xFF, saturate_int16(v_mm_s),
                                                  saturate_int16(w_mrad_s), flags))


def encode_status(seq, mcu_ms, left_count, right_count, gyro_z_cdps, gyro_bias_cdps=0, battery_mv=0,
                  last_cmd_seq=0xFF, status=0):
    """status: an int, or a dict {bit name: bool} with names from STATUS_BITS. Used by the MCU simulator."""
    if isinstance(status, dict):
        unknown = set(status) - set(STATUS_BITS)
        if unknown:
            raise ValueError('unknown status bits %s' % sorted(unknown))
        status = sum(1 << STATUS_BITS.index(k) for k, v in status.items() if v)
    return _frame(TYPE_STATUS, STATUS_BODY.pack(
        seq & 0xFF, mcu_ms & 0xFFFF, _wrap_i32(left_count), _wrap_i32(right_count),
        saturate_int16(gyro_z_cdps), saturate_int16(gyro_bias_cdps), battery_mv & 0xFFFF,
        last_cmd_seq & 0xFF, status & 0xFF))


def encode_status2(**fields):
    """0x82 frame from keyword fields (names STATUS2_FIELDS; omitted ones are 0, last_cmd_seq 0xFF)."""
    unknown = set(fields) - set(STATUS2_FIELDS)
    if unknown:
        raise ValueError('unknown 0x82 fields %s' % sorted(unknown))
    v = dict.fromkeys(STATUS2_FIELDS, 0)
    v['last_cmd_seq'] = 0xFF
    v.update(fields)
    i32 = ('enc_a', 'enc_b', 'enc_c', 'enc_d', 'gz_sum')
    u8 = ('seq', 'gz_n', 'imu_err', 'last_cmd_seq', 'status')
    vals = [_wrap_i32(v[k]) if k in i32 else v[k] & 0xFF if k in u8 else v[k] & 0xFFFFFFFF if k == 'mcu_ms'
            else v[k] & 0xFFFF if k == 'battery_mv' else saturate_int16(v[k]) for k in STATUS2_FIELDS]
    return _frame(TYPE_STATUS2, STATUS2_BODY.pack(*vals))


def _wrap_i32(value):
    return ((int(value) + 2**31) % 2**32) - 2**31


def decode_body(frame_type, body):
    if frame_type == TYPE_COMMAND:
        seq, v, w, flags = COMMAND_BODY.unpack(body)
        return {'type': 'command', 'seq': seq, 'v_mm_s': v, 'w_mrad_s': w, 'flags': flags,
                'enable': bool(flags & FLAG_ENABLE), 'recalibrate': bool(flags & FLAG_RECALIBRATE),
                'status2': bool(flags & FLAG_STATUS2)}
    if frame_type == TYPE_STATUS:
        seq, mcu_ms, left, right, gz, bias, battery, last_cmd, status = STATUS_BODY.unpack(body)
        row = {'type': 'status', 'seq': seq, 'mcu_ms': mcu_ms, 'left_count': left, 'right_count': right,
               'gyro_z_cdps': gz, 'gyro_bias_cdps': bias, 'battery_mv': battery, 'last_cmd_seq': last_cmd,
               'status': status}
        row.update({name: bool(status >> bit & 1) for bit, name in enumerate(STATUS_BITS)})
        return row
    if frame_type == TYPE_STATUS2:
        row = dict(zip(STATUS2_FIELDS, STATUS2_BODY.unpack(body)))
        row['type'] = 'status2'
        row.update({name: bool(row['status'] >> bit & 1) for bit, name in enumerate(STATUS_BITS)})
        return row
    raise ValueError('unknown frame type 0x%02x' % frame_type)


def decode_frame(frame):
    """One complete frame (bytes) -> dict; raises ValueError on any defect. The parser below is the stream path."""
    if len(frame) < 3 or frame[:2] != SYNC or frame[2] not in FRAME_LENGTHS:
        raise ValueError('not a frame start')
    if len(frame) != FRAME_LENGTHS[frame[2]]:
        raise ValueError('length %d, type 0x%02x needs %d' % (len(frame), frame[2], FRAME_LENGTHS[frame[2]]))
    if crc16_ccitt_false(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
        raise ValueError('crc mismatch')
    return decode_body(frame[2], frame[3:-2])


class FrameParser:
    """Byte stream -> frames. Bad CRC, unknown type and truncated frames are dropped silently (protocol 1),
    but counted. After any rejected candidate only its first sync byte is skipped, so a real frame that
    begins inside the rejected bytes (a truncated frame followed by a good one) is still found."""

    def __init__(self, accept=None):
        self.accept = set(FRAME_LENGTHS if accept is None else accept)
        self.buffer = bytearray()
        self.counters = {'frames': 0, 'crc_errors': 0, 'unknown_type': 0, 'unexpected_type': 0,
                         'discarded_bytes': 0}

    def feed(self, data):
        self.buffer += data
        frames = []
        buffer = self.buffer
        while True:
            start = buffer.find(SYNC)
            if start < 0:
                keep = 1 if buffer[-1:] == SYNC[:1] else 0
                self.counters['discarded_bytes'] += len(buffer) - keep
                del buffer[:len(buffer) - keep]
                break
            if start:
                self.counters['discarded_bytes'] += start
                del buffer[:start]
            if len(buffer) < 3:
                break
            frame_type = buffer[2]
            length = FRAME_LENGTHS.get(frame_type)
            if length is None:
                self.counters['unknown_type'] += 1
                self._skip(1)
                continue
            if len(buffer) < length:
                break
            covered = bytes(buffer[:length - 2])
            if crc16_ccitt_false(covered) != buffer[length - 2] | buffer[length - 1] << 8:
                self.counters['crc_errors'] += 1
                self._skip(1)
                continue
            del buffer[:length]
            if frame_type not in self.accept:
                self.counters['unexpected_type'] += 1
                continue
            self.counters['frames'] += 1
            frames.append(decode_body(frame_type, covered[3:]))
        return frames

    def _skip(self, count):
        self.counters['discarded_bytes'] += count
        del self.buffer[:count]


def seq_gap(previous, current):
    """Frames missing between two u8 sequence numbers (0 when consecutive)."""
    return (current - previous - 1) % 256


class ChassisLink:
    """Nano-side policy of the command link (protocol section 4).

    Commands: 50 Hz, the last upstream velocity is repeated; enable is sent only while
      * the upstream /fpga/cmd_vel_safe is fresher than upstream_stale_ns (200 ms) and
      * a status frame arrived within link_lost_ns (300 ms) -- so nothing is enabled before the MCU is seen.
    The heartbeat from the KV260 still overrides all of this inside the MCU; the Nano cannot bypass it.
    Status: u8 seq gaps count lost status frames; last_cmd_seq is matched against send times to measure the
    command round trip (send -> MCU accepted -> next status received, so it includes up to one 10 ms
    status period) and its jumps count commands the MCU did not report.
    A status frame arriving after more than link_lost_ns of silence is counted in status_silences instead:
    the seq gap across an MCU reset or halt says nothing about line loss (10-12 bring-up: three MCU halts of
    3-8 s read as 276 "lost" frames). Unreported commands are only counted between two acks that both
    matched commands of this session (at start the MCU still reports the previous program's last seq).
    v0.2: status2=True sets flag b2 in every command (the MCU then sends 0x82); on_status takes both frame
    types (0x82: raw mean gyro rate gz_sum/gz_n, no bias correction; 52-byte airtime for the stamp);
    last_command keeps the last (v, w, enable) built, which the driver forwards to the odometry (the
    static detector needs "command is stop"); status() adds mcu_frame / warning only when b2 is requested.
    """

    def __init__(self, v_limit_mm_s=500, w_limit_mrad_s=1500, upstream_stale_ns=200_000_000,
                 # 10-26 owner：重成功率，放宽自 100 ms（MCU 仍有 200 ms 命令超时与心跳锁存兜底）
                 link_lost_ns=300_000_000, recalibrate_timeout_ns=1_000_000_000, rtt_window=512,
                 status2=False):
        self.v_limit, self.w_limit = int(v_limit_mm_s), int(w_limit_mrad_s)
        self.status2 = bool(status2)      # v0.2: set flag b2 in every command (ask for 0x82)
        self.last_command = None          # (v_mm_s, w_mrad_s, enable) of the last frame built
        self.upstream_stale_ns, self.link_lost_ns = int(upstream_stale_ns), int(link_lost_ns)
        self.recalibrate_timeout_ns = int(recalibrate_timeout_ns)
        self.rtt_window = int(rtt_window)
        self.seq = 0
        self.upstream = None              # (v_mm_s, w_mrad_s, received_ns)
        self.sent_ns = {}                 # command seq -> monotonic send time (last 256)
        self.last_status = None
        self.last_status_ns = None
        self.recalibrate_since = None
        self.rtt_ns = []
        self.counters = {'commands_sent': 0, 'commands_enabled': 0, 'upstream_messages': 0,
                         'upstream_saturated': 0, 'upstream_non_finite': 0, 'status_frames': 0,
                         'status_lost': 0, 'commands_acked': 0, 'commands_unreported': 0,
                         'link_lost_events': 0, 'mcu_ack_unknown_seq': 0, 'recalibrate_requests': 0,
                         'status_silences': 0}
        self.link_was_alive = False
        self.ack_matched = False          # the MCU's previous last_cmd_seq was one of our commands

    # ---- upstream -------------------------------------------------------------------------------------
    def on_velocity(self, v_m_s, w_rad_s, now_ns):
        """Upstream Twist (linear.x m/s, angular.z rad/s). Non-finite input is refused (treated as no update)."""
        self.counters['upstream_messages'] += 1
        if not (math.isfinite(v_m_s) and math.isfinite(w_rad_s)):
            self.counters['upstream_non_finite'] += 1
            return
        v = int(round(v_m_s * 1000.))
        w = int(round(w_rad_s * 1000.))
        clipped_v = max(-self.v_limit, min(self.v_limit, v))
        clipped_w = max(-self.w_limit, min(self.w_limit, w))
        if (clipped_v, clipped_w) != (v, w):
            self.counters['upstream_saturated'] += 1
        self.upstream = (clipped_v, clipped_w, int(now_ns))

    def request_recalibration(self, now_ns):
        """Ask the MCU to re-estimate the gyro bias. Enable is forced to 0 until it reports calibrating
        (b6) or the timeout passes; the MCU accepts it only while the car stands still."""
        self.counters['recalibrate_requests'] += 1
        self.recalibrate_since = int(now_ns)

    def link_alive(self, now_ns):
        return self.last_status_ns is not None and now_ns - self.last_status_ns <= self.link_lost_ns

    def upstream_fresh(self, now_ns):
        return self.upstream is not None and now_ns - self.upstream[2] <= self.upstream_stale_ns

    def next_command(self, now_ns):
        """The frame to send now (call at 50 Hz). Returns (bytes, decision dict)."""
        now_ns = int(now_ns)
        alive = self.link_alive(now_ns)
        if self.link_was_alive and not alive:
            self.counters['link_lost_events'] += 1
        self.link_was_alive = alive
        recalibrating = self.recalibrate_since is not None
        if recalibrating and now_ns - self.recalibrate_since > self.recalibrate_timeout_ns:
            self.recalibrate_since, recalibrating = None, False
        fresh = self.upstream_fresh(now_ns)
        enable = alive and fresh and not recalibrating
        v, w = (self.upstream[0], self.upstream[1]) if enable else (0, 0)
        seq = self.seq
        frame = encode_command(seq, v, w, enable, recalibrate=recalibrating, status2=self.status2)
        self.last_command = (v, w, enable)
        self.sent_ns[seq] = now_ns
        self.seq = (seq + 1) & 0xFF
        self.counters['commands_sent'] += 1
        self.counters['commands_enabled'] += int(enable)
        reason = ('enabled' if enable else 'recalibrating' if recalibrating
                  else 'chassis_link_lost' if not alive else 'upstream_stale')
        return frame, {'seq': seq, 'v_mm_s': v, 'w_mrad_s': w, 'enable': enable, 'reason': reason}

    # ---- downstream -----------------------------------------------------------------------------------
    def on_status(self, row, received_ns):
        """A decoded status frame (dict from FrameParser). Returns the derived row for publication."""
        received_ns = int(received_ns)
        previous = self.last_status
        self.counters['status_frames'] += 1
        lost = 0
        silence = previous is not None and received_ns - self.last_status_ns > self.link_lost_ns
        if silence:
            self.counters['status_silences'] += 1
            self.ack_matched = False
        elif previous is not None:
            lost = seq_gap(previous['seq'], row['seq'])
            self.counters['status_lost'] += lost
        rtt = None
        ack = row['last_cmd_seq']
        never_received = ack == 0xFF and row['command_timeout']
        if never_received:
            self.ack_matched = False
        elif (previous is None or ack != previous['last_cmd_seq']):
            sent = self.sent_ns.pop(ack, None)
            matched = sent is not None and sent <= received_ns
            if not matched:
                self.counters['mcu_ack_unknown_seq'] += 1
            else:
                rtt = received_ns - sent
                self.rtt_ns.append(rtt)
                del self.rtt_ns[:-self.rtt_window]
                self.counters['commands_acked'] += 1
                if self.ack_matched:
                    self.counters['commands_unreported'] += seq_gap(previous['last_cmd_seq'], ack)
            self.ack_matched = matched
        if row['gyro_calibrating'] and self.recalibrate_since is not None:
            self.recalibrate_since = None
        self.last_status, self.last_status_ns = row, received_ns
        if row.get('type') == 'status2':
            # raw 0x82 rate: mean of this 10 ms's raw reads, NOT bias-corrected (handoff F2; the odometry
            # node estimates its own bias); gz_n = 0 -> no gyro this frame (consumers see a gap)
            gyro = (math.radians(row['gz_sum'] / row['gz_n'] * GYRO_RAW_DEG_S) if row['gz_n'] > 0 else None)
            frame_type = TYPE_STATUS2
        else:
            gyro = row['gyro_z_cdps'] * CENTI_DEG_TO_RAD if row['gyro_valid'] else None
            frame_type = TYPE_STATUS
        return {'received_ns': received_ns,
                'stamp_ns': received_ns - airtime_ns(frame_type),
                'lost_before': lost, 'after_silence': silence, 'rtt_ns': rtt, 'gyro_z_rad_s': gyro,
                'gyro_bias_rad_s': row['gyro_bias_cdps'] * CENTI_DEG_TO_RAD,
                'battery_v': row['battery_mv'] / 1000. if row['battery_mv'] else None}

    def rtt_summary(self):
        if not self.rtt_ns:
            return None
        ordered = sorted(self.rtt_ns)
        pick = lambda q: ordered[min(len(ordered) - 1, int(q * len(ordered)))] / 1e6
        return {'n': len(ordered), 'p50_ms': pick(.5), 'p90_ms': pick(.9), 'max_ms': ordered[-1] / 1e6}

    def status(self, now_ns):
        last = self.last_status
        row = {'link_alive': self.link_alive(now_ns), 'upstream_fresh': self.upstream_fresh(now_ns),
               'status_age_ms': None if self.last_status_ns is None else (now_ns - self.last_status_ns) / 1e6,
               'mcu': None if last is None else {k: last[k] for k in (
                   STATUS2_STATUS_KEYS if last.get('type') == 'status2' else STATUS_STATUS_KEYS)},
               'rtt': self.rtt_summary(), 'counters': dict(self.counters)}
        if self.status2:
            # v0.2 keys only when 0x82 is requested (a v0.1 link's status JSON stays as it was)
            frame = None if last is None else ('0x82' if last.get('type') == 'status2' else '0x81')
            row['status2_requested'] = True
            row['mcu_frame'] = frame
            row['mcu_status2'] = frame == '0x82' if frame else None
            if frame == '0x81':
                row['warning'] = 'MCU not switched to 0x82 (old firmware?): odometry runs on wheels only'
        return row


# ---- serial port ---------------------------------------------------------------------------------------
_BAUD_CONSTANTS = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
                   57600: termios.B57600, 115200: termios.B115200, 230400: termios.B230400,
                   460800: getattr(termios, 'B460800', None), 921600: getattr(termios, 'B921600', None)}


def configure_raw(fd, baud=BAUD):
    """Raw 8N1, no flow control, no echo, no line discipline; non-blocking reads (VMIN=0, VTIME=0)."""
    speed = _BAUD_CONSTANTS.get(baud)
    if speed is None:
        raise ValueError('unsupported baud %r' % baud)
    iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(fd)
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP | termios.INLCR
               | termios.IGNCR | termios.ICRNL | termios.IXON | termios.IXOFF | termios.IXANY | termios.INPCK)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | getattr(termios, 'CRTSCTS', 0))
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    cc[termios.VMIN], cc[termios.VTIME] = 0, 0
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)


def open_serial(path, baud=BAUD):
    """Open and configure a tty (or a pty for tests). Returns a non-blocking fd; the caller closes it."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        if os.isatty(fd):
            configure_raw(fd, baud)
    except Exception:
        os.close(fd)
        raise
    return fd


def write_all(fd, data, timeout_s=0.05):
    """Write a whole frame to a non-blocking fd (a frame is short, the kernel buffer normally takes it at once)."""
    import select
    view = memoryview(data)
    while view:
        try:
            n = os.write(fd, view)
        except BlockingIOError:
            n = 0
        view = view[n:]
        if view and not select.select([], [fd], [], timeout_s)[1]:
            raise TimeoutError('serial write stalled with %d bytes left' % len(view))
