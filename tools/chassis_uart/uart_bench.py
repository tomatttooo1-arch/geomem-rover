#!/usr/bin/env python3
"""Bench checks of the Nano <-> chassis MCU command serial link, without ROS (protocol v0.1).

  loopback  TX wired to RX (Nano 40-pin pin 8 <-> pin 10). Sends status and command frames at the real
            rates and requires every frame back byte-exact. Positive control that this device is the
            pin that was jumpered and that 115200 8N1 raw works. Run once more WITHOUT the jumper: it must
            receive 0 bytes (negative control: no echo from the tty layer, no other talker on the line).
  monitor   Listen to the MCU: status frame rate, lost/CRC-bad frames, the MCU bits and gyro z.
            Sends nothing, so the MCU must report b3 (command timeout) the whole time.
  drive     ChassisLink at 50 Hz with a constant velocity for --seconds, then enable=0. Reports how the MCU
            acknowledged (last_cmd_seq), the command round trip and the bits. Wheels off the ground!

Exit status 0 = the check passed, 1 = it failed, 2 = could not run. One JSON summary line on stdout.
"""
import argparse
import json
import os
import select
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.chassis_uart import (BAUD, TYPE_STATUS, ChassisLink, FrameParser, encode_command,  # noqa: E402
                               encode_status, open_serial, write_all)


def drain(fd, seconds):
    out = bytearray()
    end = time.monotonic() + seconds
    while True:
        left = end - time.monotonic()
        if left <= 0:
            return bytes(out)
        if select.select([fd], [], [], left)[0]:
            try:
                out += os.read(fd, 4096)
            except BlockingIOError:
                pass


def loopback(fd, seconds, expect_silence):
    drain(fd, 0.1)                                    # whatever sat in the buffer before
    sent = []
    received = bytearray()
    t0 = time.monotonic()
    tick = 0
    while time.monotonic() - t0 < seconds:
        deadline = t0 + (tick + 1) * 0.01
        frame = encode_status(tick, tick * 10, tick * 7, -tick * 5, (tick % 200) - 100, 3, 7400, tick // 2, 0x35)
        if tick % 2 == 0:
            frame += encode_command(tick // 2, (tick * 13) % 1001 - 500, -(tick * 29) % 3001 + 1500, True)
        write_all(fd, frame)
        sent.append(frame)
        tick += 1
        received += drain(fd, max(0., deadline - time.monotonic()))
    received += drain(fd, 0.2)
    expected = b''.join(sent)
    parser = FrameParser()
    frames = parser.feed(bytes(received))
    row = {'mode': 'loopback', 'expect_silence': expect_silence, 'bytes_sent': len(expected),
           'bytes_received': len(received), 'frames_sent': sum(1 + (i % 2 == 0) for i in range(tick)),
           'frames_decoded': len(frames), 'byte_exact': bytes(received) == expected, 'parser': parser.counters}
    if expect_silence:
        row['passed'] = len(received) == 0
    else:
        row['passed'] = row['byte_exact'] and row['frames_decoded'] == row['frames_sent']
        if not row['byte_exact']:
            first = next((i for i, (a, b) in enumerate(zip(received, expected)) if a != b),
                         min(len(received), len(expected)))
            row['first_difference_at_byte'] = first
    return row


def monitor(fd, seconds):
    parser = FrameParser(accept={TYPE_STATUS})
    link = ChassisLink()
    rows = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        data = drain(fd, 0.05)
        now = time.monotonic_ns()
        for frame in parser.feed(data):
            link.on_status(frame, now)
            rows.append(frame)
    elapsed = time.monotonic() - t0
    row = {'mode': 'monitor', 'seconds': round(elapsed, 3), 'status_frames': len(rows),
           'rate_hz': round(len(rows) / elapsed, 2), 'status_lost': link.counters['status_lost'],
           'parser': parser.counters}
    if rows:
        gz = [r['gyro_z_cdps'] for r in rows]
        last = rows[-1]
        row.update({'last': {k: v for k, v in last.items() if k != 'type'},
                    'gyro_z_cdps_min_mean_max': [min(gz), round(sum(gz) / len(gz), 2), max(gz)],
                    'mcu_ms_span': (rows[-1]['mcu_ms'] - rows[0]['mcu_ms']) % 65536,
                    'always_timeout': all(r['command_timeout'] for r in rows)})
    row['passed'] = bool(rows) and 90 <= row['rate_hz'] <= 110 and parser.counters['crc_errors'] == 0 \
        and row.get('always_timeout', False)
    return row


def drive(fd, seconds, v, w):
    parser = FrameParser(accept={TYPE_STATUS})
    link = ChassisLink()
    bits_seen = set()
    enabled_sent = 0
    t0 = time.monotonic()
    tick = 0
    total = seconds + 0.5                               # last 0.5 s: upstream stale -> enable=0
    while time.monotonic() - t0 < total:
        now = time.monotonic_ns()
        if time.monotonic() - t0 < seconds:
            link.on_velocity(v, w, now)
        frame, decision = link.next_command(now)
        enabled_sent += decision['enable']
        write_all(fd, frame)
        tick += 1
        deadline = t0 + tick * 0.02
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                break
            if not select.select([fd], [], [], left)[0]:
                continue
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            received = time.monotonic_ns()                  # stamp each chunk on arrival (RTT depends on it)
            for status in parser.feed(data):
                link.on_status(status, received)
                bits_seen.add(status['status'])
    row = {'mode': 'drive', 'v_m_s': v, 'w_rad_s': w, 'commands_sent': link.counters['commands_sent'],
           'commands_enabled': enabled_sent, 'link': link.status(time.monotonic_ns()),
           'parser': parser.counters, 'status_bits_seen': sorted('0x%02x' % b for b in bits_seen)}
    c = link.counters
    row['passed'] = (c['status_frames'] > 0 and c['commands_acked'] > 0.9 * c['commands_sent']
                     and parser.counters['crc_errors'] == 0)
    return row


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=('loopback', 'monitor', 'drive'))
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--seconds', type=float, default=3.)
    ap.add_argument('--expect-silence', action='store_true', help='loopback negative control (jumper removed)')
    ap.add_argument('--v', type=float, default=0.1, help='drive: m/s')
    ap.add_argument('--w', type=float, default=0., help='drive: rad/s')
    a = ap.parse_args(argv)
    try:
        fd = open_serial(a.port, a.baud)
    except OSError as error:
        print(json.dumps({'mode': a.mode, 'passed': False, 'error': str(error)}))
        return 2
    try:
        if a.mode == 'loopback':
            row = loopback(fd, a.seconds, a.expect_silence)
        elif a.mode == 'monitor':
            row = monitor(fd, a.seconds)
        else:
            row = drive(fd, a.seconds, a.v, a.w)
    finally:
        os.close(fd)
    row['port'] = a.port
    print(json.dumps(row))
    return 0 if row['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
