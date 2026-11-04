#!/usr/bin/env python3
"""Stand-in for the chassis MCU on the command serial link (protocol v0.1, CHASSIS_UART_PROTOCOL_20261012.md).

It implements the MCU duties of protocol section 3 so the Nano side can be exercised without dev2's board:
status frame 0x81 every 10 ms, the 200 ms command timeout (target to zero, b3), limits, the start-up gyro
bias calibration (b6 for --calib-s, then b5) and recalibration on request (flag b1, only standing and
enable=0), last_cmd_seq, and the permit heartbeat rule of PL_PERMIT_UART_MCU_GUIDE_20261011.md section 2
(latched at power-on; 30 ms without heartbeat -> motors off and latched; unlock only after >= 100 ms of
continuous heartbeat AND a command frame that arrived after the heartbeat came back). The heartbeat is
queried every millisecond (McuModel heartbeat_fn); the CLI sources are: always on (default),
--no-heartbeat, --heartbeat-file PATH (on while the file starts with '1'), --heartbeat-kv HOST:PORT (on
while the KV TELEM permit bit is set and the last poll is fresh). Wheels follow the command instantly;
counts and gyro z come from a differential-drive model (--track-mm, --wheel-diam-mm, --counts-per-rev:
placeholders until dev2 confirms them, protocol 5.4).

The speed the wheels really run at (0 when latched / timed out / calibrating) is available as
McuModel.executed() and the twist_fn callback; --twist-topic publishes it as geometry_msgs/Twist (for a
Gazebo chassis: Nano driver -> pty -> this MCU -> Twist -> simulator). rclpy is imported only then.

Faults for known-answer tests: --drop-every N (skip a status frame, seq still advances), --corrupt-every N
(bad CRC), --gyro-sign -1, --gyro-bias-cdps B (bias the MCU forgot to subtract), --noise-bytes-every N.

v0.2 (CHASSIS_UART_V02_IMU_HANDOFF_20261021.md, 2026-10-21): a command with flag b2 switches the status
frame to 0x82 (raw gyro sum of 5 ms reads, four mecanum encoders, accel, true yaw), until a command without
b2 (kept across a command timeout). The fault knobs (bias, drift, noise, spikes, gz_n jitter, dropouts,
imu_err, collision segments with blocked wheels) are on Status2Faults; the CLI exposes --s2-*.

  # pty pair for a host test: prints the path the Nano side opens, runs until killed or --seconds
  mcu_sim.py --pty --seconds 10
  # a PC USB-UART (3.3 V) wired to the Nano 40-pin UART instead of the MCU
  mcu_sim.py --port /dev/ttyUSB0 --seconds 30
At the end it prints one JSON line: commands received / CRC errors / timeouts / latches / frames sent and
dropped.

--log rows carry wall_ns (CLOCK_REALTIME, for mapping onto simulation time; line D2 2026-10-14): commands,
latch / unlock events (unlock with hb_good_since_ms and last_cmd_ms), every change of the executed speed
('executed': [v, w]) and, with --heartbeat-kv, every change of the polled KV permit bit ('kv_permit').
"""
import argparse
import json
import math
import os
import pty
import select
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.chassis_uart import (GYRO_RAW_DEG_S, TEMP_UNAVAILABLE, TYPE_COMMAND, FrameParser,  # noqa: E402
                               encode_status, encode_status2, open_serial, configure_raw, write_all)
from host.gyro_odom import POSITIONS, mecanum_inverse, parse_signs, parse_wheel_map  # noqa: E402

ACCEL_1G_RAW = 2048                      # +-16 g range: 1 g = 32768 / 16


class Status2Faults:
    """v0.2 (0x82) behaviour of the fake MCU, for known-answer tests of the Nano side (2026-10-21).

    Truth: the body turns at the executed w (rad/s) while enabled; during a collision segment
    (start_ms, end_ms, peak_dps) the wheels are blocked (no counts, whatever the command) and the body
    turns with a half-sine rate peak * sin(pi u), u = 0..1 over the segment, plus a horizontal accel
    shock of collision_shock_g in its first 20 ms -- the 10-17 "wheels still, car turning" case.
    Gyro z is read every 5 ms (ms % 5 == 0): raw = round((sign * true + bias + drift * t) / LSB + noise);
    gz_sum / gz_n sum the reads of the 10 ms frame. Faults: spikes {frame number: raw value} replace every
    read of that frame (the 15.625 deg/s high-byte-only values are 256 * k), gz_n_one_every N drops one read
    of every Nth frame (gz_n = 1), dropouts [(start_ms, end_ms)] no reads (gz_n = 0), imu_err_at
    {frame number: increments}, accel_torn_every N reports (0, 0, 0) as the accel of every Nth frame (the
    torn reads of the 10-21 board check). Four encoders: mecanum_inverse(v, 0, w) per ms, mapped to enc_a..d by
    wheel_map / wheel_signs (the same strings as host/ros_wheel_odom.py)."""

    def __init__(self, bias_dps=0., drift_dps_per_s=0., noise_lsb=0., accel_noise_lsb=0., seed=1,
                 spikes=None, gz_n_one_every=0, dropouts=(), collisions=(), collision_shock_g=1.0,
                 imu_err_at=None, lx_ly_mm=215., rollers='X', wheel_map='a=fl,b=fr,c=rl,d=rr',
                 wheel_signs='+1,+1,+1,+1', accel_torn_every=0):
        import random
        self.bias_dps, self.drift_dps_per_s = float(bias_dps), float(drift_dps_per_s)
        self.noise_lsb, self.accel_noise_lsb = float(noise_lsb), float(accel_noise_lsb)
        self.rng = random.Random(seed)
        self.spikes = dict(spikes or {})
        self.gz_n_one_every = int(gz_n_one_every)
        self.dropouts, self.collisions = list(dropouts), list(collisions)
        self.collision_shock_g = float(collision_shock_g)
        self.imu_err_at = dict(imu_err_at or {})
        self.accel_torn_every = int(accel_torn_every)
        self.lx_ly_m, self.rollers = lx_ly_mm / 1000., rollers
        self.wheel_map, self.wheel_signs = parse_wheel_map(wheel_map), parse_signs(wheel_signs)

    def collision_at(self, ms):
        """(true rate rad/s, ms since the segment start) inside a collision segment, else None. A segment is
        (start_ms, end_ms, peak_dps) -- half-sine -- or (start_ms, end_ms, rate_dps, 'const'): a perfectly
        smooth constant-rate turn (the case section 8.3's std criteria cannot see)."""
        for seg in self.collisions:
            start, end, peak = seg[:3]
            if start <= ms < end:
                shape = seg[3] if len(seg) > 3 else 'sine'
                rate = peak if shape == 'const' else peak * math.sin(math.pi * (ms - start) / float(end - start))
                return math.radians(rate), ms - start
        return None

    def dropped(self, ms):
        return any(start <= ms < end for start, end in self.dropouts)


class McuModel:
    """Deterministic MCU behaviour. Time is integer MCU milliseconds passed in by the caller.

    advance(ms) runs the 1 kHz heartbeat tick of PL_PERMIT_UART_MCU_GUIDE_20261011.md section 2 for every
    millisecond up to ms (heartbeat_fn(ms) is queried once per millisecond; True = a valid rising edge in
    that millisecond) and moves the wheels at the executed speed; receive() and step() call it first.
      hb_ok   = now - last_rise <= 30 ms; not ok -> motors off at once, latch, hb_good_since cleared
      unlock  = latched and now - hb_good_since >= 100 and last_cmd_ms > hb_good_since (a command frame
                that arrived after the heartbeat came back; any CRC-valid command frame counts)
      latched at power-on; a command that arrives while latched is only time-stamped (motors_off), so the
      first moving command is the one after the unlock tick.
    step(ms) builds the 10 ms status frame. executed() / twist_fn give the speed the wheels really run at
    (m/s, rad/s): the limited command while enabled, (0, 0) when latched, timed out or calibrating;
    twist_fn(ms, v, w) is called on every change (the latch millisecond included) and on every step."""

    HB_TIMEOUT_MS = 30
    HB_RECOVER_MS = 100

    def __init__(self, track_mm=160., wheel_diam_mm=65., counts_per_rev=1320, v_limit_mm_s=500,
                 w_limit_mrad_s=1500, calib_ms=2000, timeout_ms=200, heartbeat=True, gyro_sign=1,
                 gyro_bias_cdps=0, battery_mv=0, drop_every=0, corrupt_every=0, heartbeat_fn=None,
                 twist_fn=None, status2_faults=None):
        self.track_mm, self.counts_per_mm = track_mm, counts_per_rev / (math.pi * wheel_diam_mm)
        self.v_limit, self.w_limit = v_limit_mm_s, w_limit_mrad_s
        self.calib_ms, self.timeout_ms = calib_ms, timeout_ms
        if heartbeat_fn is None:
            heartbeat_fn = (lambda ms: True) if heartbeat else (lambda ms: False)
        self.heartbeat_fn, self.twist_fn = heartbeat_fn, twist_fn
        self.gyro_sign, self.gyro_bias_cdps = gyro_sign, gyro_bias_cdps
        self.battery_mv = battery_mv
        self.drop_every, self.corrupt_every = drop_every, corrupt_every
        self.parser = FrameParser(accept={TYPE_COMMAND})
        self.seq = 0
        self.frames_built = 0
        self.left = self.right = 0.
        self.target = (0, 0)
        self.last_cmd_seq = 0xFF
        self.last_cmd_ms = None
        self.calib_until = calib_ms
        self.calibrated = False
        self.enabled = False
        # heartbeat rule state (guide section 2)
        self.tick_ms = -1                    # last millisecond advance() processed
        self.last_rise_ms = None
        self.hb_good_since = None
        self.hb_ok = False
        self.locked = True
        self.lock_time_ms = None
        self.counters = {'commands': 0, 'commands_enabled': 0, 'recalibrations': 0, 'timeouts': 0,
                         'status_sent': 0, 'status_dropped': 0, 'status_corrupted': 0, 'latches': 0,
                         'unlocks': 0, 'commands_while_latched': 0}
        self.was_timed_out = True
        self.events = []                     # (ms, 'latch' | 'unlock')
        self.unlock_info = []                # (ms, hb_good_since, last_cmd_ms) of every unlock
        self._reported = None
        # v0.2: 0x82 while the latest CRC-valid command has b2 (handoff section 4); kept on timeout
        self.s2 = status2_faults or Status2Faults()
        self.status2 = False
        self.wheels = dict.fromkeys(POSITIONS, 0.)   # metres per wheel, mecanum model
        self.true_yaw = 0.                   # rad, the body's real rotation (known answer)
        self.true_x = self.true_y = 0.       # m, the body's real position (start = origin, heading +x)
        self.true_rate = 0.
        self.gz_reads = []
        self.imu_err = 0
        self.accel_xy = (0., 0.)
        self.counters['status2_sent'] = 0

    def executed(self):
        """(v m/s, w rad/s) the wheels run at now."""
        v, w = self.target if self.enabled else (0, 0)
        return v / 1000., w / 1000.

    def _report(self, now_ms, force=False):
        cur = self.executed()
        if self.twist_fn is not None and (force or cur != self._reported):
            self.twist_fn(now_ms, cur[0], cur[1])
        self._reported = cur

    def _stop(self, now_ms):
        self.target, self.enabled = (0, 0), False
        self._report(now_ms)

    def advance(self, now_ms):
        """Run the 1 kHz tick for every millisecond up to now_ms; the wheels move 1 ms per tick."""
        half_per_mrad = self.track_mm / 2. / 1000.
        while self.tick_ms < now_ms:
            self.tick_ms += 1
            t = self.tick_ms
            if self.heartbeat_fn(t):
                self.last_rise_ms = t
            self.hb_ok = self.last_rise_ms is not None and t - self.last_rise_ms <= self.HB_TIMEOUT_MS
            if not self.hb_ok:
                if self.enabled or self.target != (0, 0):
                    self._stop(t)
                if not self.locked:
                    self.locked, self.lock_time_ms = True, t
                    self.counters['latches'] += 1
                    self.events.append((t, 'latch'))
                self.hb_good_since = None
            else:
                if self.hb_good_since is None:
                    self.hb_good_since = t
                if (self.locked and t - self.hb_good_since >= self.HB_RECOVER_MS
                        and self.last_cmd_ms is not None and self.last_cmd_ms > self.hb_good_since):
                    self.locked = False
                    self.counters['unlocks'] += 1
                    self.events.append((t, 'unlock'))
                    self.unlock_info.append((t, self.hb_good_since, self.last_cmd_ms))
            hit = self.s2.collision_at(t) if self.s2.collisions else None
            if self.enabled and hit is None:
                v, w = self.target
                half = w * half_per_mrad
                self.left += (v - half) * 0.001 * self.counts_per_mm
                self.right += (v + half) * 0.001 * self.counts_per_mm
                for pos, d in zip(POSITIONS, mecanum_inverse(v * 1e-6, 0., w * 1e-6, self.s2.lx_ly_m,
                                                             self.s2.rollers)):
                    self.wheels[pos] += d             # (mm/s * 1 ms = 1e-6 m; mrad/s * 1 ms = 1e-6 rad)
                self.true_rate = w / 1000.
            elif hit is not None:
                self.true_rate = hit[0]
            else:
                self.true_rate = 0.
            speed = self.target[0] / 1000. if self.enabled and hit is None else 0.
            mid = self.true_yaw + self.true_rate * 0.0005
            self.true_x += speed * 0.001 * math.cos(mid)
            self.true_y += speed * 0.001 * math.sin(mid)
            self.true_yaw += self.true_rate * 0.001
            self.accel_xy = ((self.s2.collision_shock_g, 0.) if hit is not None and hit[1] < 20 else (0., 0.))
            if t % 5 == 0 and not self.s2.dropped(t):
                dps = (self.gyro_sign * math.degrees(self.true_rate) + self.s2.bias_dps
                       + self.s2.drift_dps_per_s * t / 1000.)
                noise = self.s2.rng.gauss(0., self.s2.noise_lsb) if self.s2.noise_lsb else 0.
                self.gz_reads.append(int(round(dps / GYRO_RAW_DEG_S + noise)))

    def receive(self, data, now_ms):
        self.advance(now_ms)
        for cmd in self.parser.feed(data):
            self.counters['commands'] += 1
            self.last_cmd_seq, self.last_cmd_ms = cmd['seq'], now_ms
            self.status2 = cmd['status2']
            standing = self.target == (0, 0)
            if cmd['recalibrate'] and not cmd['enable'] and standing and now_ms >= self.calib_until:
                self.calib_until = now_ms + self.calib_ms
                self.calibrated = False
                self.counters['recalibrations'] += 1
            calibrating = now_ms < self.calib_until
            if self.locked:
                self.counters['commands_while_latched'] += 1
            if cmd['enable'] and not calibrating and not self.locked:
                self.counters['commands_enabled'] += 1
                self.target = (max(-self.v_limit, min(self.v_limit, cmd['v_mm_s'])),
                               max(-self.w_limit, min(self.w_limit, cmd['w_mrad_s'])))
                self.enabled = True
                self._report(now_ms)
            else:
                self._stop(now_ms)

    def _counts(self, pos):
        return int(round(self.wheels[pos] * self.counts_per_mm * 1000.))

    def _step_status2(self, now_ms, timed_out, calibrating):
        s2 = self.s2
        self.frames_built += 1
        n = self.frames_built
        reads = self.gz_reads
        self.gz_reads = []
        if s2.gz_n_one_every and n % s2.gz_n_one_every == 0 and reads:
            reads = reads[:-1]
        if n in s2.spikes and reads:
            reads = [int(s2.spikes[n])] * len(reads)
        self.imu_err = (self.imu_err + int(s2.imu_err_at.get(n, 0))) & 0xFF
        enc = {}
        for pos in POSITIONS:
            letter = s2.wheel_map[pos]
            enc['enc_' + letter] = s2.wheel_signs[letter] * self._counts(pos)
        anoise = (lambda: s2.rng.gauss(0., s2.accel_noise_lsb)) if s2.accel_noise_lsb else (lambda: 0.)
        yaw_deg = math.degrees(math.atan2(math.sin(self.true_yaw), math.cos(self.true_yaw)))
        bits = {'heartbeat_seen': self.hb_ok, 'motor_enabled': self.enabled, 'stop_latched': self.locked,
                'command_timeout': timed_out, 'gyro_valid': bool(reads), 'gyro_calibrated': self.calibrated,
                'gyro_calibrating': calibrating}
        status = sum(1 << i for i, k in enumerate(('heartbeat_seen', 'motor_enabled', 'stop_latched',
                                                   'command_timeout', 'gyro_valid', 'gyro_calibrated',
                                                   'gyro_calibrating')) if bits[k])
        accel = (int(round(self.accel_xy[0] * ACCEL_1G_RAW + anoise())),
                 int(round(self.accel_xy[1] * ACCEL_1G_RAW + anoise())), int(round(ACCEL_1G_RAW + anoise())))
        if s2.accel_torn_every and n % s2.accel_torn_every == 0:
            accel = (0, 0, 0)
        frame = encode_status2(seq=self.seq, mcu_ms=now_ms, gz_sum=sum(reads), gz_n=len(reads),
                               imu_err=self.imu_err, ax=accel[0], ay=accel[1], az=accel[2],
                               yaw_raw=int(round(yaw_deg / 180. * 32768)), temp_cdeg=TEMP_UNAVAILABLE,
                               battery_mv=self.battery_mv, last_cmd_seq=self.last_cmd_seq, status=status, **enc)
        self.seq = (self.seq + 1) & 0xFF
        if self.drop_every and n % self.drop_every == 0:
            self.counters['status_dropped'] += 1
            return b''
        if self.corrupt_every and n % self.corrupt_every == 0:
            self.counters['status_corrupted'] += 1
            frame = frame[:-1] + bytes((frame[-1] ^ 0xFF,))
        self.counters['status_sent'] += 1
        self.counters['status2_sent'] += 1
        return frame

    def step(self, now_ms):
        """Status frame at now_ms (b'' when this frame is dropped); called every 10 ms. 0x82 instead of 0x81
        while the latest command asked for it (flag b2)."""
        self.advance(now_ms)
        timed_out = self.last_cmd_ms is None or now_ms - self.last_cmd_ms > self.timeout_ms
        if timed_out:
            if not self.was_timed_out:
                self.counters['timeouts'] += 1
            self._stop(now_ms)
        self.was_timed_out = timed_out
        calibrating = now_ms < self.calib_until
        if not calibrating:
            self.calibrated = True
        self._report(now_ms, force=True)
        if self.status2:
            return self._step_status2(now_ms, timed_out, calibrating)
        v, w = self.target if self.enabled else (0, 0)
        gyro = self.gyro_sign * w * 18000. / math.pi / 1000. + self.gyro_bias_cdps
        bits = {'heartbeat_seen': self.hb_ok, 'motor_enabled': self.enabled,
                'stop_latched': self.locked, 'command_timeout': timed_out,
                'gyro_valid': True, 'gyro_calibrated': self.calibrated, 'gyro_calibrating': calibrating}
        frame = encode_status(self.seq, now_ms, int(round(self.left)), int(round(self.right)),
                              int(round(gyro)), 0, self.battery_mv, self.last_cmd_seq, bits)
        self.seq = (self.seq + 1) & 0xFF
        self.frames_built += 1
        n = self.frames_built
        if self.drop_every and n % self.drop_every == 0:
            self.counters['status_dropped'] += 1
            return b''
        if self.corrupt_every and n % self.corrupt_every == 0:
            self.counters['status_corrupted'] += 1
            frame = frame[:-1] + bytes((frame[-1] ^ 0xFF,))
        self.counters['status_sent'] += 1
        return frame


class FileHeartbeat:
    """Heartbeat present while PATH exists and starts with '1' (read at most once per millisecond)."""

    def __init__(self, path):
        self.path = path

    def __call__(self, ms):
        try:
            with open(self.path, 'rb') as f:
                return f.read(1) == b'1'
        except OSError:
            return False


class KvPermitHeartbeat:
    """Heartbeat = the KV TELEM permit bit (the PL emits its square wave only while permit is set).

    Polled on a thread at --heartbeat-kv-hz; a poll older than max_age_s (or failed) reads as no
    heartbeat, like a PL that stopped answering."""

    def __init__(self, server, hz=200., max_age_s=0.02):
        import threading
        from host.kv_link import KvTelemetryClient, blocks_mask
        self.client = KvTelemetryClient(server, timeout_s=0.1, hello_timeout_s=3.0)
        self.want = blocks_mask(('permit', 'board'))
        self.period, self.max_age_s = 1. / hz, max_age_s
        self.permit, self.stamp = False, 0.
        self.polls = self.failures = 0
        self.changes = []                    # (monotonic ns, wall ns, permit) at every change of the bit
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name='kv-permit-heartbeat', daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            t0 = time.monotonic()
            try:
                tm = self.client.poll(self.want)
                bits = (tm.blocks.get('permit') or {}).get('status_bits') or {}
                permit = bool(bits.get('permit'))
                if permit != self.permit or not self.polls:
                    self.changes.append((time.monotonic_ns(), time.time_ns(), permit))
                self.permit, self.stamp = permit, time.monotonic()
                self.polls += 1
            except Exception:
                self.failures += 1
            self.stop.wait(max(0., self.period - (time.monotonic() - t0)))

    def __call__(self, ms):
        return self.permit and time.monotonic() - self.stamp <= self.max_age_s


def make_ros_twist_sink(topic):
    """(callback, close) publishing the executed speed as geometry_msgs/Twist on topic."""
    import rclpy
    from geometry_msgs.msg import Twist
    try:   # keep the context alive on SIGINT so the final zero-speed message still goes out (D2 smoke_01)
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init(args=None)
    node = rclpy.create_node('fpga_mcu_sim')
    pub = node.create_publisher(Twist, topic, 10)

    def publish(ms, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        pub.publish(msg)

    def close():
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return publish, close


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument('--port', help='serial device the Nano is wired to')
    where.add_argument('--pty', action='store_true', help='create a pty pair, print the Nano-side path')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--seconds', type=float, default=0., help='0 = until SIGINT/SIGTERM')
    ap.add_argument('--track-mm', type=float, default=160.)
    ap.add_argument('--wheel-diam-mm', type=float, default=65.)
    ap.add_argument('--counts-per-rev', type=int, default=1320)
    ap.add_argument('--calib-s', type=float, default=2.)
    hb = ap.add_mutually_exclusive_group()
    hb.add_argument('--no-heartbeat', action='store_true', help='never a heartbeat: b0 clear, b2 set, no motion')
    hb.add_argument('--heartbeat-file', help="heartbeat while this file starts with '1'")
    hb.add_argument('--heartbeat-kv', help='HOST:PORT of a kv_server: heartbeat while TELEM permit is set')
    ap.add_argument('--heartbeat-kv-hz', type=float, default=200.)
    ap.add_argument('--twist-topic', help='publish the executed speed as geometry_msgs/Twist (needs rclpy)')
    ap.add_argument('--gyro-sign', type=int, choices=(1, -1), default=1)
    ap.add_argument('--gyro-bias-cdps', type=int, default=0)
    ap.add_argument('--battery-mv', type=int, default=0)
    ap.add_argument('--drop-every', type=int, default=0)
    ap.add_argument('--corrupt-every', type=int, default=0)
    ap.add_argument('--noise-bytes-every', type=int, default=0, help='insert 3 garbage bytes after every Nth frame')
    ap.add_argument('--log', help='JSON lines of every command received and every latch / unlock')
    ap.add_argument('--s2-bias-dps', type=float, default=0., help='0x82: gyro bias in the raw reads')
    ap.add_argument('--s2-drift-dps-per-s', type=float, default=0., help='0x82: bias drift')
    ap.add_argument('--s2-noise-lsb', type=float, default=0., help='0x82: gyro read noise (raw units, 1 sigma)')
    ap.add_argument('--s2-lx-ly-mm', type=float, default=215., help='0x82: mecanum lx + ly of the four-wheel model')
    a = ap.parse_args(argv)

    if a.pty:
        master, slave = pty.openpty()
        configure_raw(slave)
        os.set_blocking(master, False)
        print(os.ttyname(slave), flush=True)
        fd = master
    else:
        fd = open_serial(a.port, a.baud)
    heartbeat_fn, kv_hb = None, None
    if a.heartbeat_file:
        heartbeat_fn = FileHeartbeat(a.heartbeat_file)
    elif a.heartbeat_kv:
        host, _, port = a.heartbeat_kv.rpartition(':')
        kv_hb = heartbeat_fn = KvPermitHeartbeat((host, int(port)), hz=a.heartbeat_kv_hz)
    twist_fn, twist_close = (make_ros_twist_sink(a.twist_topic) if a.twist_topic else (None, None))
    log = open(a.log, 'a', buffering=1) if a.log else None
    t0 = time.monotonic()
    wall0 = time.time_ns()

    def wall_of(ms):
        return wall0 + ms * 1000000
    if log is not None:
        executed_last = [None]
        sink = twist_fn

        def twist_fn(ms, v, w):              # noqa: F811 -- the sink, plus a log row on every change
            if executed_last[0] != (v, w):
                executed_last[0] = (v, w)
                log.write(json.dumps({'t_ms': ms, 'wall_ns': wall_of(ms), 'executed': [v, w]}) + '\n')
            if sink is not None:
                sink(ms, v, w)
    model = McuModel(track_mm=a.track_mm, wheel_diam_mm=a.wheel_diam_mm, counts_per_rev=a.counts_per_rev,
                     calib_ms=int(a.calib_s * 1000), heartbeat=not a.no_heartbeat, gyro_sign=a.gyro_sign,
                     gyro_bias_cdps=a.gyro_bias_cdps, battery_mv=a.battery_mv, drop_every=a.drop_every,
                     corrupt_every=a.corrupt_every, heartbeat_fn=heartbeat_fn, twist_fn=twist_fn,
                     status2_faults=Status2Faults(bias_dps=a.s2_bias_dps, drift_dps_per_s=a.s2_drift_dps_per_s,
                                                  noise_lsb=a.s2_noise_lsb, lx_ly_mm=a.s2_lx_ly_mm))
    import signal
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    tick = 0
    logged_events = 0
    logged_unlocks = 0
    logged_kv = 0
    if log is not None:
        log.write(json.dumps({'start': True, 't_ms': 0, 'wall_ns': wall0, 'monotonic_s': t0}) + '\n')

    def mcu_ms():
        return int((time.monotonic() - t0) * 1000)
    try:
        while not stop and not (a.seconds and time.monotonic() - t0 >= a.seconds):
            deadline = t0 + (tick + 1) * 0.01
            while True:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                # wake at least every millisecond so the heartbeat is sampled at the guide's 1 kHz
                ready = select.select([fd], [], [], min(left, 0.001))[0]
                model.advance(mcu_ms())
                if not ready:
                    continue
                try:
                    data = os.read(fd, 4096)
                except (BlockingIOError, OSError):
                    data = b''
                if not data:
                    time.sleep(0.001)
                    continue
                before = model.counters['commands']
                model.receive(data, mcu_ms())
                if log and model.counters['commands'] != before:
                    log.write(json.dumps({'t_ms': mcu_ms(), 'wall_ns': time.time_ns(),
                                          'last_cmd_seq': model.last_cmd_seq,
                                          'target': model.target, 'enabled': model.enabled,
                                          'latched': model.locked}) + '\n')
            tick += 1
            frame = model.step(mcu_ms())
            if log and len(model.events) > logged_events:
                for ms, what in model.events[logged_events:]:
                    row = {'t_ms': ms, 'wall_ns': wall_of(ms), 'event': what}
                    if what == 'unlock' and logged_unlocks < len(model.unlock_info):
                        _, since, cmd_ms = model.unlock_info[logged_unlocks]
                        logged_unlocks += 1
                        row.update(hb_good_since_ms=since, last_cmd_ms=cmd_ms)
                    log.write(json.dumps(row) + '\n')
                logged_events = len(model.events)
            if log and kv_hb is not None and len(kv_hb.changes) > logged_kv:
                for mono_ns, wall_ns, permit in kv_hb.changes[logged_kv:]:
                    log.write(json.dumps({'kv_permit': permit, 'monotonic_ns': mono_ns, 'wall_ns': wall_ns,
                                          't_ms': int((mono_ns / 1e9 - t0) * 1000)}) + '\n')
                logged_kv = len(kv_hb.changes)
            if a.noise_bytes_every and tick % a.noise_bytes_every == 0:
                frame += b'\xa5\x5a\x81'
            if frame:
                try:
                    write_all(fd, frame)
                except (OSError, TimeoutError):
                    pass
    except KeyboardInterrupt:
        pass
    if kv_hb is not None:
        kv_hb.stop.set()
    final_stop = None
    if twist_close is not None:
        try:
            twist_fn(mcu_ms(), 0., 0.)      # leave the simulated chassis stopped
            final_stop = 'published'
        except Exception as error:          # context already shut down: report, do not lose the summary
            final_stop = 'failed: %s: %s' % (type(error).__name__, error)
        twist_close()
    summary = dict(model.counters, parser=model.parser.counters, seconds=round(time.monotonic() - t0, 3))
    if final_stop is not None:
        summary['final_zero_twist'] = final_stop
    if kv_hb is not None:
        summary['kv_heartbeat'] = {'polls': kv_hb.polls, 'failures': kv_hb.failures}
    print(json.dumps(summary), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
