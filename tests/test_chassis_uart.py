"""Chassis command serial link v0.1 (docs/design/CHASSIS_UART_PROTOCOL_20261012.md): codec, stream parser,
Nano-side policy, the C reference codec for the MCU, and the bench tools over a pty. No ROS, no hardware."""
import json
import math
import os
from pathlib import Path
import pty
import random
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

from host.chassis_uart import (CENTI_DEG_TO_RAD, STATUS2_FIELDS, STATUS_BITS, TYPE_STATUS, TYPE_STATUS2,
                               ChassisLink, FrameParser, airtime_ns, configure_raw, crc16_ccitt_false,
                               decode_frame, encode_command, encode_status, encode_status2, seq_gap)
from tools.chassis_uart.mcu_sim import McuModel

ROOT = Path(__file__).resolve().parents[1]
MS = 1_000_000


def status_frame(seq=0, mcu_ms=0, left=0, right=0, gz=0, bias=0, battery=0, ack=0xFF, bits=0x08):
    return encode_status(seq, mcu_ms, left, right, gz, bias, battery, ack, bits)


# Printed verbatim in the handoff document section 3 (test_handoff_document_prints_the_known_answer checks it).
KNOWN_STATUS2_HEX = ('a5 5a 82 01 40 e2 01 00 e8 03 00 00 e9 03 00 00 19 fc ff ff 16 fc ff ff d7 ff ff ff 02 00 '
                     '01 00 fe ff 0c 00 f8 ff 02 08 55 15 35 0c 00 00 00 00 07 3c 1c 76')
HANDOFF_DOC = ROOT / 'docs/design/CHASSIS_UART_V02_IMU_HANDOFF_20261021.md'


class CodecTests(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(crc16_ccitt_false(b'123456789'), 0x29B1)

    def test_crc_c_path_equals_table_reference(self):
        # 10-29: crc16_ccitt_false is binascii.crc_hqx; the table loop stays as the reference
        from host.chassis_uart import crc16_ccitt_false_table
        rng = random.Random(1029)
        for n in list(range(64)) + [512]:
            data = bytes(rng.randrange(256) for _ in range(n))
            init = rng.choice((0xFFFF, 0, rng.randrange(65536)))
            self.assertEqual(crc16_ccitt_false(data, init), crc16_ccitt_false_table(data, init))
            self.assertEqual(crc16_ccitt_false(bytearray(data)), crc16_ccitt_false_table(data))

    def test_status_layout_matches_document_offsets(self):
        f = status_frame(seq=0x12, mcu_ms=0xBEEF, left=-2, right=0x01020304, gz=-12345, bias=77, battery=7400,
                         ack=0x34, bits=0xB5)
        self.assertEqual(len(f), 24)
        self.assertEqual(f[:3], b'\xa5\x5a\x81')
        self.assertEqual(f[3], 0x12)
        self.assertEqual(f[4:6], b'\xef\xbe')
        self.assertEqual(f[6:10], b'\xfe\xff\xff\xff')
        self.assertEqual(f[10:14], b'\x04\x03\x02\x01')
        self.assertEqual(struct.unpack('<h', f[14:16])[0], -12345)
        self.assertEqual(struct.unpack('<h', f[16:18])[0], 77)
        self.assertEqual(struct.unpack('<H', f[18:20])[0], 7400)
        self.assertEqual((f[20], f[21]), (0x34, 0xB5))
        self.assertEqual(f[22] | f[23] << 8, crc16_ccitt_false(f[:22]))       # sync included

    def test_command_layout_matches_document_offsets(self):
        f = encode_command(0xFE, -300, 1200, True, recalibrate=True)
        self.assertEqual(len(f), 11)
        self.assertEqual(f[:4], b'\xa5\x5a\x01\xfe')
        self.assertEqual(struct.unpack('<hh', f[4:8]), (-300, 1200))
        self.assertEqual(f[8], 0x03)
        self.assertEqual(f[9] | f[10] << 8, crc16_ccitt_false(f[:9]))
        self.assertEqual(encode_command(0, 1, 1, False)[8], 0)

    def test_status2_layout_matches_handoff_offsets(self):
        """v0.2 0x82 frame, offsets of CHASSIS_UART_V02_IMU_HANDOFF_20261021.md section 3."""
        f = encode_status2(seq=0x12, mcu_ms=0x01020304, enc_a=-2, enc_b=0x11223344, enc_c=-300000, enc_d=7,
                           gz_sum=-65536, gz_n=2, imu_err=3, gx_mean=-5, gy_mean=6, ax=-100, ay=200, az=2048,
                           yaw_raw=-16384, temp_cdeg=3150, gyro_bias_cdps=-7, battery_mv=24100, last_cmd_seq=0x34,
                           status=0xB5)
        self.assertEqual(len(f), 52)
        self.assertEqual(f[:4], b'\xa5\x5a\x82\x12')
        self.assertEqual(f[4:8], b'\x04\x03\x02\x01')
        self.assertEqual(struct.unpack('<iiiii', f[8:28]), (-2, 0x11223344, -300000, 7, -65536))
        self.assertEqual((f[28], f[29]), (2, 3))
        self.assertEqual(struct.unpack('<hhhhhhhh', f[30:46]), (-5, 6, -100, 200, 2048, -16384, 3150, -7))
        self.assertEqual(struct.unpack('<H', f[46:48])[0], 24100)
        self.assertEqual((f[48], f[49]), (0x34, 0xB5))
        self.assertEqual(f[50] | f[51] << 8, crc16_ccitt_false(f[:50]))       # sync included
        row = decode_frame(f)
        self.assertEqual((row['type'], row['gz_sum'], row['yaw_raw'], row['gyro_valid']), ('status2', -65536, -16384, True))
        self.assertEqual(encode_command(1, 0, 0, False, status2=True)[8], 0x04)

    def test_status2_known_answer_frame_in_handoff(self):
        """The example frame printed in the handoff document (dev2 checks the firmware against these bytes)."""
        f = encode_status2(seq=1, mcu_ms=123456, enc_a=1000, enc_b=1001, enc_c=-999, enc_d=-1002, gz_sum=-41,
                           gz_n=2, imu_err=0, gx_mean=1, gy_mean=-2, ax=12, ay=-8, az=2050, yaw_raw=5461,
                           temp_cdeg=3125, gyro_bias_cdps=0, battery_mv=0, last_cmd_seq=7, status=0x3C)
        self.assertEqual(f.hex(' '), KNOWN_STATUS2_HEX)

    def test_handoff_document_prints_the_known_answer(self):
        self.assertIn(KNOWN_STATUS2_HEX.upper(), HANDOFF_DOC.read_text(encoding='utf-8'))

    def test_status2_not_accepted_by_v01_driver_parser(self):
        """A v0.1 parser (accept={0x81}, the driver before the v0.2 Nano side) gives no rows for a 0x82 stream,
        so an MCU switched by mistake is a visible silence, never misread fields. The v0.2 driver accepts both
        (tests/test_gyro_odom.py)."""
        p = FrameParser(accept={TYPE_STATUS})
        rows = p.feed(encode_status2(seq=1) + status_frame(seq=2))
        self.assertEqual([r['seq'] for r in rows], [2])

    def test_real_mcu_frames_10_12(self):
        """Bytes captured on the Nano 40-pin UART from dev2's MCU (10-12 21:3x, before and after an MCU reset).
        Their CRC covers the sync bytes; this is what made the protocol text change, so they stay as a known answer."""
        captured = bytes.fromhex(
            'a55a8131febbdd04000034f8ffff000000000000004cc292'
            'a55a813208bcdd04000034f8ffff000000000000004c2889'
            'a55a810650280000000000000000000000000000003cf09a'
            'a55a81075a280000000000000000000000000000003cc345')
        p = FrameParser()
        rows = p.feed(captured)
        self.assertEqual(p.counters['crc_errors'], 0)
        self.assertEqual([(r['seq'], r['mcu_ms']) for r in rows], [(0x31, 0xBBFE), (0x32, 0xBC08), (6, 0x2850),
                                                                  (7, 0x285A)])
        self.assertEqual((rows[0]['left_count'], rows[0]['right_count'], rows[0]['status']), (1245, -1996, 0x4C))
        self.assertEqual([rows[2][n] for n in ('stop_latched', 'command_timeout', 'gyro_valid', 'gyro_calibrated')],
                         [True, True, True, True])

    def test_round_trip_and_bit_names(self):
        row = decode_frame(status_frame(seq=9, bits=0b01010011, gz=-1))
        self.assertEqual([row[n] for n in STATUS_BITS], [True, True, False, False, True, False, True, False])
        self.assertEqual(row['gyro_z_cdps'], -1)
        cmd = decode_frame(encode_command(5, 40000, -40000, True))
        self.assertEqual((cmd['v_mm_s'], cmd['w_mrad_s'], cmd['enable']), (32767, -32768, True))

    def test_decode_frame_rejects_defects(self):
        good = status_frame()
        for bad in (good[:-1], good[:-1] + bytes((good[-1] ^ 1,)), b'\xa5\x5a\x7f' + good[3:], good[1:]):
            with self.assertRaises(ValueError):
                decode_frame(bad)

    def test_airtime(self):
        self.assertEqual(airtime_ns(TYPE_STATUS), 24 * 10 * 10**9 // 115200)


class ParserTests(unittest.TestCase):
    def frames(self, n):
        return [status_frame(seq=i, mcu_ms=10 * i, left=i * 3, right=-i, gz=i - 50, ack=i // 2, bits=0x31)
                for i in range(n)]

    def test_split_at_every_byte(self):
        stream = b''.join(self.frames(5))
        p = FrameParser()
        got = []
        for b in stream:
            got += p.feed(bytes((b,)))
        self.assertEqual([r['seq'] for r in got], list(range(5)))
        self.assertEqual(p.counters['discarded_bytes'], 0)

    def test_garbage_bad_crc_truncation_and_false_sync(self):
        f = self.frames(4)
        bad_crc = f[1][:-1] + bytes((f[1][-1] ^ 0xFF,))
        truncated = f[2][:13]
        false_sync = status_frame(seq=99, left=0x815AA5, right=0x5AA5)     # payload holds a5 5a 81 / a5 5a
        stream = b'\x00\xa5\xff' + f[0] + bad_crc + truncated + f[3] + b'\xa5\x5a\x99' + false_sync
        p = FrameParser()
        got = p.feed(stream)
        self.assertEqual([r['seq'] for r in got], [0, 3, 99])
        self.assertEqual(p.counters['crc_errors'] >= 2, True)
        self.assertEqual(p.counters['unknown_type'], 1)

    def test_random_garbage_never_loses_a_good_frame(self):
        rng = random.Random(20261012)
        good = self.frames(300)
        stream = bytearray()
        for f in good:
            stream += bytes(rng.randrange(256) for _ in range(rng.choice((0, 0, 1, 3, 30))))
            if rng.random() < 0.1:
                stream += f[:rng.randrange(1, 24)]                 # a cut-off copy before the real one
            stream += f
        p = FrameParser()
        got = []
        i = 0
        while i < len(stream):
            step = rng.randrange(1, 40)
            got += p.feed(bytes(stream[i:i + step]))
            i += step
        self.assertEqual([r['seq'] for r in got], [i & 0xFF for i in range(300)])

    def test_accept_filter(self):
        p = FrameParser(accept={TYPE_STATUS})
        got = p.feed(encode_command(1, 0, 0, False) + status_frame(seq=4))
        self.assertEqual([r['seq'] for r in got], [4])
        self.assertEqual(p.counters['unexpected_type'], 1)


class LinkPolicyTests(unittest.TestCase):
    def test_no_enable_before_mcu_seen_then_enable(self):
        link = ChassisLink()
        link.on_velocity(0.2, -0.5, 0)
        _, d = link.next_command(1 * MS)
        self.assertEqual((d['enable'], d['reason'], d['v_mm_s']), (False, 'chassis_link_lost', 0))
        link.on_status(decode_frame(status_frame(ack=0)), 2 * MS)
        frame, d = link.next_command(3 * MS)
        self.assertEqual((d['enable'], d['v_mm_s'], d['w_mrad_s']), (True, 200, -500))
        self.assertEqual(decode_frame(frame)['flags'], 1)

    def test_upstream_stale_and_link_lost(self):
        link = ChassisLink(link_lost_ns=100 * MS)   # the logic at the old 100 ms (default 300 ms since 10-26)
        link.on_velocity(0.1, 0., 0)
        link.on_status(decode_frame(status_frame()), 0)
        self.assertTrue(link.next_command(100 * MS)[1]['enable'])
        d = link.next_command(101 * MS)[1]
        self.assertEqual((d['enable'], d['reason']), (False, 'chassis_link_lost'))
        self.assertEqual(link.counters['link_lost_events'], 1)
        link.on_status(decode_frame(status_frame(seq=1)), 150 * MS)
        self.assertTrue(link.next_command(200 * MS)[1]['enable'])
        d = link.next_command(201 * MS)[1]
        self.assertEqual((d['enable'], d['reason']), (False, 'upstream_stale'))

    def test_default_link_lost_300ms_20261026(self):
        # 10-26 owner: a 250 ms status gap (Nano under load reads the UART late) no longer drops enable
        link = ChassisLink(upstream_stale_ns=10_000 * MS)
        link.on_velocity(0.1, 0., 0)
        link.on_status(decode_frame(status_frame()), 0)
        self.assertTrue(link.next_command(250 * MS)[1]['enable'])
        self.assertTrue(link.next_command(300 * MS)[1]['enable'])
        d = link.next_command(301 * MS)[1]
        self.assertEqual((d['enable'], d['reason']), (False, 'chassis_link_lost'))

    def test_saturation_rounding_and_non_finite(self):
        link = ChassisLink()
        link.on_velocity(0.9, -2.0, 0)
        self.assertEqual(link.upstream[:2], (500, -1500))
        link.on_velocity(0.0004, 0.0006, 0)
        self.assertEqual(link.upstream[:2], (0, 1))
        link.on_velocity(math.nan, 0., 5)
        self.assertEqual(link.upstream, (0, 1, 0))
        self.assertEqual((link.counters['upstream_saturated'], link.counters['upstream_non_finite']), (1, 1))

    def test_sequence_wraps(self):
        link = ChassisLink()
        seqs = [decode_frame(link.next_command(i)[0])['seq'] for i in range(258)]
        self.assertEqual(seqs[254:258], [254, 255, 0, 1])
        self.assertEqual(seq_gap(255, 0), 0)
        self.assertEqual(seq_gap(250, 3), 8)

    def test_status_loss_ack_rtt_and_unreported(self):
        link = ChassisLink()
        for i in range(4):
            link.next_command(i * 20 * MS)                 # seq 0..3 sent at 0, 20, 40, 60 ms
        link.on_status(decode_frame(status_frame(seq=254, ack=0, bits=0)), 5 * MS)
        link.on_status(decode_frame(status_frame(seq=255, ack=0, bits=0)), 15 * MS)
        r = link.on_status(decode_frame(status_frame(seq=2, ack=1, bits=0)), 27 * MS)   # 0, 1 lost
        self.assertEqual(r['lost_before'], 2)
        self.assertEqual(r['rtt_ns'], 7 * MS)
        link.on_status(decode_frame(status_frame(seq=3, ack=3, bits=0)), 66 * MS)       # seq 2 unreported
        c = link.counters
        self.assertEqual((c['status_lost'], c['commands_acked'], c['commands_unreported']), (2, 3, 1))
        self.assertEqual(link.rtt_summary()['n'], 3)

    def test_mcu_silence_and_stale_ack_are_not_loss(self):
        """10-12 bring-up: an MCU halt read as lost frames, the previous program's last seq as unreported."""
        link = ChassisLink()
        for i in range(3):
            link.next_command(i * 20 * MS)
        link.on_status(decode_frame(status_frame(seq=10, ack=149, bits=0)), 5 * MS)      # stale ack at start
        link.on_status(decode_frame(status_frame(seq=11, ack=0, bits=0)), 15 * MS)
        link.on_status(decode_frame(status_frame(seq=12, ack=1, bits=0)), 25 * MS)
        r = link.on_status(decode_frame(status_frame(seq=200, ack=0xFF, bits=0x08)), 3025 * MS)   # MCU reset
        self.assertEqual((r['lost_before'], r['after_silence']), (0, True))
        link.next_command(3030 * MS)                                                     # seq 3
        link.on_status(decode_frame(status_frame(seq=201, ack=3, bits=0)), 3035 * MS)
        c = link.counters
        self.assertEqual((c['status_lost'], c['status_silences'], c['commands_unreported'], c['commands_acked'],
                          c['mcu_ack_unknown_seq']), (0, 1, 0, 3, 1))

    def test_never_received_is_not_an_ack(self):
        link = ChassisLink()
        link.next_command(0)
        link.on_status(decode_frame(status_frame(ack=0xFF, bits=0x08)), MS)
        self.assertEqual((link.counters['commands_acked'], link.counters['mcu_ack_unknown_seq']), (0, 0))

    def test_recalibration_holds_enable_until_b6(self):
        link = ChassisLink()
        link.on_velocity(0.1, 0., 0)
        link.on_status(decode_frame(status_frame()), 0)
        link.request_recalibration(1 * MS)
        frame, d = link.next_command(2 * MS)
        self.assertEqual((d['enable'], d['reason'], decode_frame(frame)['flags']), (False, 'recalibrating', 2))
        link.on_status(decode_frame(status_frame(seq=1, bits=0x40)), 10 * MS)
        link.on_velocity(0.1, 0., 11 * MS)
        frame, d = link.next_command(12 * MS)
        self.assertEqual((d['enable'], decode_frame(frame)['flags']), (True, 1))
        link.request_recalibration(20 * MS)                 # no b6 ever: released by the 1 s timeout
        link.on_status(decode_frame(status_frame(seq=2, bits=0x00)), 495 * MS)
        link.on_velocity(0.1, 0., 495 * MS)
        self.assertEqual(link.next_command(500 * MS)[1]['reason'], 'recalibrating')
        link.on_status(decode_frame(status_frame(seq=3, bits=0x00)), 1015 * MS)
        link.on_velocity(0.1, 0., 1015 * MS)
        frame, d = link.next_command(1021 * MS)
        self.assertEqual((d['enable'], decode_frame(frame)['flags']), (True, 1))


class KnownAnswerControls(unittest.TestCase):
    """IMU plan I3: dropped frames, a flipped gyro sign and an injected bias must each be caught."""

    W_MRAD_S = 800

    def run_pipeline(self, **fault):
        model = McuModel(calib_ms=0, **fault)
        link = ChassisLink()
        parser = FrameParser(accept={TYPE_STATUS})
        gyro = []
        for tick in range(300):
            now = tick * 10
            if tick % 2 == 0:
                link.on_velocity(0., self.W_MRAD_S / 1000., now * MS)
                frame, _ = link.next_command(now * MS)
                model.receive(frame, now)
            for row in parser.feed(model.step(now)):
                derived = link.on_status(row, now * MS)
                if row['motor_enabled'] and derived['gyro_z_rad_s'] is not None:
                    gyro.append(derived['gyro_z_rad_s'])
        return link, gyro

    def check(self, link, gyro):
        failures = []
        if link.counters['status_lost']:
            failures.append('lost %d' % link.counters['status_lost'])
        expected = self.W_MRAD_S / 1000.
        if not gyro or any(abs(g - expected) > CENTI_DEG_TO_RAD for g in gyro):
            failures.append('gyro off')
        return failures

    def test_clean_run_passes(self):
        link, gyro = self.run_pipeline()
        self.assertEqual(self.check(link, gyro), [])
        self.assertGreater(len(gyro), 250)

    def test_faults_are_detected(self):
        link, gyro = self.run_pipeline(drop_every=37)
        self.assertEqual(link.counters['status_lost'], 300 // 37)
        for fault in ({'drop_every': 37}, {'gyro_sign': -1}, {'gyro_bias_cdps': 50}):
            with self.subTest(**fault):
                self.assertNotEqual(self.check(*self.run_pipeline(**fault)), [])

    def test_timeout_and_heartbeat_in_model(self):
        # Latched at power-on (MCU guide section 2): >= 100 ms of heartbeat and a command after it first.
        model = McuModel(calib_ms=0)
        model.receive(encode_command(0, 100, 0, True), 0)
        self.assertFalse(decode_frame(model.step(10))['motor_enabled'])
        model.receive(encode_command(1, 100, 0, True), 50)
        model.receive(encode_command(2, 100, 0, True), 100)
        row = decode_frame(model.step(110))
        self.assertEqual((row['motor_enabled'], row['stop_latched']), (True, False))
        row = decode_frame(model.step(301))
        self.assertEqual((row['motor_enabled'], row['command_timeout']), (False, True))
        model = McuModel(calib_ms=0, heartbeat=False)
        model.receive(encode_command(0, 100, 0, True), 0)
        row = decode_frame(model.step(10))
        self.assertEqual((row['motor_enabled'], row['stop_latched']), (False, True))


class CReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('gcc') or shutil.which('cc')
        if compiler is None:
            raise RuntimeError('gcc is required to check the MCU reference codec; no silent skip')
        cls.tmp = tempfile.TemporaryDirectory(prefix='fpga-chassis-c-')
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.exe = str(Path(cls.tmp.name) / 'selftest')
        src = ROOT / 'hw/chassis_mcu'
        subprocess.run([compiler, '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-pedantic',
                        str(src / 'chassis_proto.c'), str(src / 'selftest.c'), '-o', cls.exe],
                       check=True, capture_output=True, text=True)

    def run_c(self, *args):
        return subprocess.run([self.exe, *map(str, args)], check=True, capture_output=True,
                              text=True).stdout.split('\n')[:-1]

    def test_crc(self):
        self.assertEqual(self.run_c('crc'), ['29b1'])

    def test_status_bytes_equal_python(self):
        rng = random.Random(7)
        for _ in range(50):
            v = (rng.randrange(256), rng.randrange(65536), rng.randrange(-2**31, 2**31),
                 rng.randrange(-2**31, 2**31), rng.randrange(-32768, 32768), rng.randrange(-32768, 32768),
                 rng.randrange(65536), rng.randrange(256), rng.randrange(256))
            self.assertEqual(self.run_c('status', *v), [encode_status(*v).hex()])

    def test_status2_bytes_equal_python(self):
        rng = random.Random(9)
        for _ in range(50):
            v = {k: rng.randrange(256) for k in ('seq', 'gz_n', 'imu_err', 'last_cmd_seq', 'status')}
            v['mcu_ms'] = rng.randrange(2**32)
            v.update({k: rng.randrange(-2**31, 2**31) for k in ('enc_a', 'enc_b', 'enc_c', 'enc_d', 'gz_sum')})
            v.update({k: rng.randrange(-32768, 32768) for k in ('gx_mean', 'gy_mean', 'ax', 'ay', 'az', 'yaw_raw',
                                                                 'temp_cdeg', 'gyro_bias_cdps')})
            v['battery_mv'] = rng.randrange(65536)
            self.assertEqual(self.run_c('status2', *(v[k] for k in STATUS2_FIELDS)), [encode_status2(**v).hex()])

    def test_parser_accepts_python_commands_through_garbage(self):
        rng = random.Random(8)
        cmds = [(i, rng.randrange(-32768, 32768), rng.randrange(-32768, 32768), rng.choice((True, False)),
                 rng.choice((True, False))) for i in range(60)]
        stream = bytearray()
        for c in cmds:
            f = encode_command(*c[:4], recalibrate=c[4])
            stream += bytes(rng.randrange(256) for _ in range(rng.choice((0, 1, 5))))
            if rng.random() < 0.15:
                stream += f[:rng.randrange(1, 11)]
            if rng.random() < 0.1:
                stream += f[:-1] + bytes((f[-1] ^ 0x5A,))       # bad CRC copy
            stream += f
        want = ['%d %d %d %d' % (c[0], c[1], c[2], int(c[3]) | int(c[4]) << 1) for c in cmds]
        self.assertEqual(self.run_c('parse', stream.hex()), want)


class PtyEndToEndTests(unittest.TestCase):
    def test_bench_loopback_positive_and_negative_control(self):
        master, slave = pty.openpty()
        configure_raw(slave)
        path = os.ttyname(slave)
        stop = threading.Event()

        def echo():
            while not stop.is_set():
                import select
                if select.select([master], [], [], 0.02)[0]:
                    try:
                        os.write(master, os.read(master, 4096))
                    except OSError:
                        return
        thread = threading.Thread(target=echo, daemon=True)
        thread.start()
        try:
            out = subprocess.run([sys.executable, str(ROOT / 'tools/chassis_uart/uart_bench.py'), 'loopback',
                                  '--port', path, '--seconds', '0.5'], capture_output=True, text=True,
                                 timeout=30)
            row = json.loads(out.stdout)
            self.assertEqual((out.returncode, row['byte_exact']), (0, True), out.stdout + out.stderr)
            self.assertEqual(row['frames_decoded'], row['frames_sent'])
            stop.set()
            thread.join()
            out = subprocess.run([sys.executable, str(ROOT / 'tools/chassis_uart/uart_bench.py'), 'loopback',
                                  '--port', path, '--seconds', '0.3', '--expect-silence'],
                                 capture_output=True, text=True, timeout=30)
            self.assertEqual(out.returncode, 0, out.stdout + out.stderr)
            self.assertEqual(json.loads(out.stdout)['bytes_received'], 0)
        finally:
            stop.set()
            os.close(master)
            os.close(slave)

    def test_mcu_sim_over_pty_drive(self):
        sim = self.start_sim('--calib-s', '0.3', '--battery-mv', '7400')
        try:
            path = sim.stdout.readline().strip()
            self.assertTrue(path.startswith('/dev/'), path)
            out = subprocess.run([sys.executable, str(ROOT / 'tools/chassis_uart/uart_bench.py'), 'drive',
                                  '--port', path, '--seconds', '1.5', '--v', '0.2', '--w', '0.5'],
                                 capture_output=True, text=True, timeout=30)
            row = json.loads(out.stdout)
            self.assertEqual(out.returncode, 0, out.stdout + out.stderr)
            mcu = row['link']['mcu']
            self.assertGreater(row['link']['counters']['status_frames'], 120)
            self.assertTrue(mcu['gyro_calibrated'])
            self.assertEqual(mcu['battery_mv'], 7400)
            self.assertIn('0x33', row['status_bits_seen'])      # heartbeat + enabled + gyro valid/calibrated
            rtt = row['link']['rtt']
            self.assertLess(rtt['p50_ms'], 15, rtt)             # <= one 10 ms status period + scheduling
        finally:
            sim.terminate()
            sim.communicate(timeout=10)

    def start_sim(self, *extra):
        return subprocess.Popen([sys.executable, str(ROOT / 'tools/chassis_uart/mcu_sim.py'), '--pty',
                                 '--seconds', '6', *extra],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def test_monitor_survives_noise_and_sees_timeout(self):
        sim = self.start_sim('--calib-s', '0', '--noise-bytes-every', '25')
        try:
            path = sim.stdout.readline().strip()
            time.sleep(0.3)
            out = subprocess.run([sys.executable, str(ROOT / 'tools/chassis_uart/uart_bench.py'), 'monitor',
                                  '--port', path, '--seconds', '1'], capture_output=True, text=True, timeout=30)
            row = json.loads(out.stdout)
            self.assertTrue(row['always_timeout'], out.stdout)
            self.assertGreater(row['parser']['crc_errors'], 0)  # the injected noise was seen and survived
            self.assertGreater(row['status_frames'], 80)
        finally:
            sim.terminate()
            sim.communicate(timeout=10)


if __name__ == '__main__':
    unittest.main()
