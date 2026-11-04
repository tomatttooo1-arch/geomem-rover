#!/usr/bin/env python3
"""PC-side stand-in for the chassis MCU: sends encoder frames to the KV260.

Frame format and checksum come from enc_uart_model.encode_frame (the same
definition the PL bench is checked against). Use a USB-UART adapter at 3.3 V
(or through the same level shifter as the MCU) on the KV260 encoder RX pin.

Examples
  # 100 Hz, both wheels 0.3 m/s forward, for 10 s:
  mcu_frame_sim.py --port /dev/ttyUSB0 --speed-mm-s 300 300 --seconds 10
  # turning, with a corrupted checksum every 50 frames and a dropped frame every 70:
  mcu_frame_sim.py --port /dev/ttyUSB0 --speed-mm-s 200 350 --bad-every 50 --drop-every 70
  # no hardware: print the bytes of the first 3 frames
  mcu_frame_sim.py --dry-run --frames 3

After a run, compare on the KV260: GOOD_FRAMES / BAD_CHECKSUM / LOST_FRAMES
(0x6C/0x70/0x78) with the summary this script prints, and V_LEFT/V_RIGHT
(Q16.16 mm/s) with --speed-mm-s quantized to whole counts per frame.
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from enc_uart_model import encode_frame  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device, e.g. /dev/ttyUSB0 (needs pyserial)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate-hz", type=float, default=100.0)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--frames", type=int, help="overrides --seconds")
    ap.add_argument("--speed-mm-s", type=float, nargs=2, default=(300.0, 300.0), metavar=("LEFT", "RIGHT"))
    ap.add_argument("--wheel-diam-mm", type=float, default=65.0)
    ap.add_argument("--counts-per-rev", type=int, default=1320)
    ap.add_argument("--start-count", type=int, default=0)
    ap.add_argument("--flags", type=int, default=3, help="b0 heartbeat seen, b1 motor enabled")
    ap.add_argument("--bad-every", type=int, default=0, help="corrupt the checksum of every Nth frame")
    ap.add_argument("--drop-every", type=int, default=0, help="skip sending every Nth frame (seq still advances)")
    ap.add_argument("--dry-run", action="store_true", help="print hex instead of sending")
    a = ap.parse_args()

    n = a.frames if a.frames is not None else int(a.seconds * a.rate_hz)
    mm_per_count = math.pi * a.wheel_diam_mm / a.counts_per_rev
    per_frame = [v / a.rate_hz / mm_per_count for v in a.speed_mm_s]   # counts per frame (float)
    ser = None
    if not a.dry_run:
        if not a.port:
            sys.exit("--port required (or --dry-run)")
        try:
            import serial  # pyserial
        except ImportError:
            sys.exit("pyserial not installed: pip install pyserial")
        ser = serial.Serial(a.port, a.baud, bytesize=8, parity="N", stopbits=1, timeout=0)

    acc = [float(a.start_count), float(a.start_count)]
    sent = bad = dropped = 0
    t0 = time.monotonic()
    for i in range(n):
        seq = i & 0xFF
        ms = int((time.monotonic() - t0) * 1000) & 0xFFFF
        left, right = int(round(acc[0])), int(round(acc[1]))
        corrupt = a.bad_every > 0 and i % a.bad_every == a.bad_every - 1
        drop = a.drop_every > 0 and i % a.drop_every == a.drop_every - 1
        f = encode_frame(seq, ms, left, right, a.flags, corrupt_checksum=corrupt)
        if drop:
            dropped += 1
        elif a.dry_run:
            print("%5d %s" % (i, f.hex(" ")))
            sent += 1; bad += int(corrupt)
        else:
            ser.write(f)
            sent += 1; bad += int(corrupt)
        acc[0] += per_frame[0]; acc[1] += per_frame[1]
        if not a.dry_run:
            next_t = t0 + (i + 1) / a.rate_hz
            time.sleep(max(0.0, next_t - time.monotonic()))
    if ser:
        ser.flush(); ser.close()
    print("frames generated %d, sent %d (checksum corrupted %d), dropped %d" % (n, sent, bad, dropped))
    print("expected on KV260: GOOD_FRAMES +%d, BAD_CHECKSUM +%d, LOST_FRAMES +%d (if no other errors)"
          % (sent - bad, bad, dropped + bad))
    print("speed: %.1f / %.1f counts per frame at %.4f mm/count" % (per_frame[0], per_frame[1], mm_per_count))


if __name__ == "__main__":
    main()
