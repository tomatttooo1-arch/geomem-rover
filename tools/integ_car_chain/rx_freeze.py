#!/usr/bin/env python3
"""Known-answer stress for the input-drop countermeasure (v8, 2026-10-12). HOST ONLY.

SIGSTOP the given pids for MS milliseconds every EVERY seconds during DUR seconds, so that a DDS
receiver stays away from its shared-memory port for a known time. With the default 512 KB Fast DDS
segment on the image publisher the frames published during a long enough freeze are lost before the
subscription callback (producer summary input.missed_frames > 0); with a segment that holds the
backlog they arrive late instead (missed_frames 0, rate_skipped / slot.superseded > 0).

    rx_freeze.py EVERY MS DUR PID...      stdout CSV: stop_raw_ns,cont_raw_ns (one row per freeze)

A process is always continued before this program ends, also on SIGINT/SIGTERM.
"""
import os
import signal
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from host.fpga_clock import now_ns  # noqa: E402


def _cont(pids):
    for p in pids:
        try:
            os.kill(p, signal.SIGCONT)
        except OSError:
            pass


def main(argv):
    every, ms, dur = float(argv[1]), float(argv[2]), float(argv[3])
    pids = [int(p) for p in argv[4:]]
    if every <= 0 or ms <= 0 or ms / 1e3 >= every or not pids:
        raise SystemExit("usage: rx_freeze.py EVERY MS DUR PID... (MS/1000 < EVERY)")

    def bye(*_):
        _cont(pids)
        raise SystemExit(0)

    signal.signal(signal.SIGINT, bye)
    signal.signal(signal.SIGTERM, bye)
    end = time.monotonic() + dur
    print("stop_raw_ns,cont_raw_ns", flush=True)
    try:
        while time.monotonic() + every < end:
            time.sleep(every)
            t0 = now_ns()
            for p in pids:
                os.kill(p, signal.SIGSTOP)
            time.sleep(ms / 1e3)
            _cont(pids)
            print("%d,%d" % (t0, now_ns()), flush=True)
    finally:
        _cont(pids)


if __name__ == "__main__":
    main(sys.argv)
