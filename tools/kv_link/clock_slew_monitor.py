#!/usr/bin/env python3
"""Read-only witness of NTP slewing on one board (B2 RAW retest, 2026-10-11).

Every --period-s it writes one CSV row:
  raw_ns, mono_ns, real_ns       CLOCK_MONOTONIC_RAW / CLOCK_MONOTONIC / CLOCK_REALTIME
  mono_minus_raw_ns              grows at the kernel's current NTP rate correction
  freq_ppm, offset, status, ...  adjtimex(2) with modes = 0 (pure read, changes nothing)
  timesync_clock_mtime           mtime of systemd-timesyncd's clock file (touched on each sync)

A change in the slope of mono_minus_raw (or a jump of freq_ppm / the file mtime) dates an
NTP update on that board. Nothing here adjusts any clock or configuration.
Usage: clock_slew_monitor.py --out slew.csv --duration-s 3000 [--period-s 1]
"""
import argparse
import csv
import ctypes
import os
import time


class _Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class _Timex(ctypes.Structure):
    _fields_ = [("modes", ctypes.c_uint), ("offset", ctypes.c_long), ("freq", ctypes.c_long),
                ("maxerror", ctypes.c_long), ("esterror", ctypes.c_long), ("status", ctypes.c_int),
                ("constant", ctypes.c_long), ("precision", ctypes.c_long),
                ("tolerance", ctypes.c_long), ("time", _Timeval), ("tick", ctypes.c_long),
                ("ppsfreq", ctypes.c_long), ("jitter", ctypes.c_long), ("shift", ctypes.c_int),
                ("stabil", ctypes.c_long), ("jitcnt", ctypes.c_long), ("calcnt", ctypes.c_long),
                ("errcnt", ctypes.c_long), ("stbcnt", ctypes.c_long), ("tai", ctypes.c_int),
                ("_pad", ctypes.c_int * 11)]


_libc = ctypes.CDLL(None, use_errno=True)
CLOCK_FILES = ["/var/lib/systemd/timesync/clock", "/run/systemd/timesync/synchronized"]


def read_timex():
    tx = _Timex()
    tx.modes = 0                                   # read only
    state = _libc.adjtimex(ctypes.byref(tx))
    return state, tx


def mtime(path):
    try:
        return os.stat(path).st_mtime
    except OSError:
        return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration-s", type=float, required=True)
    ap.add_argument("--period-s", type=float, default=1.0)
    a = ap.parse_args()
    cols = ["raw_ns", "mono_ns", "real_ns", "mono_minus_raw_ns", "adjtimex_state", "freq_ppm",
            "offset", "status", "maxerror_us", "esterror_us"] + ["mtime:" + p for p in CLOCK_FILES]
    end = time.clock_gettime(time.CLOCK_MONOTONIC_RAW) + a.duration_s
    with open(a.out, "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(cols)
        while True:
            r0 = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            m = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
            r1 = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            rt = time.clock_gettime_ns(time.CLOCK_REALTIME)
            raw = (r0 + r1) // 2
            state, tx = read_timex()
            w.writerow([raw, m, rt, m - raw, state, tx.freq / 65536.0, tx.offset, tx.status,
                        tx.maxerror, tx.esterror] + [mtime(p) for p in CLOCK_FILES])
            fp.flush()
            if raw / 1e9 >= end:
                break
            time.sleep(a.period_s)


if __name__ == "__main__":
    main()
