"""The single timebase for KV260 <-> Nano clock mapping and frame age (B2, 2026-10-11).

``CLOCK_MONOTONIC_RAW`` on both boards: it is never rate-adjusted by NTP
(systemd-timesyncd / adjtimex slew ``CLOCK_MONOTONIC``, which on the B2 board run
broke the stated bound for 1-3 s after each NTP update). RAW runs at the bare
crystal rate, so the constant-rate model of host/kv_clock_sync.py holds.

Everything that is compared ACROSS events or boards (SYNC t1..t4, PL pair KV
stamps, frame acquisition stamps and the "now" they are aged against) must come
from ``now_ns()`` here. Pure local durations / timeouts (send/RTT splits, socket
deadlines, idle timers) may stay on ``time.monotonic``: a 500 ppm slew changes a
1 s interval by 0.5 ms at most and they are never compared with these stamps.

ROS ``header.stamp`` is CLOCK_REALTIME (or /clock sim time), a different axis
from both MONOTONIC and RAW; it is never subtracted from these stamps. Frame
identities carry ``CLOCK_ID`` and host/m7_pipeline.py refuses (``clock_mismatch``)
to age a frame against a "now" from another clock id.

Python 3.8 compatible (time.clock_gettime_ns and CLOCK_MONOTONIC_RAW exist since
3.7 on Linux). No fallback: on a platform without RAW this import fails loudly.
"""
import time

if not hasattr(time, "CLOCK_MONOTONIC_RAW") or not hasattr(time, "clock_gettime_ns"):
    raise ImportError("host.fpga_clock needs time.CLOCK_MONOTONIC_RAW (Linux, Python >= 3.7)")

_RAW = time.CLOCK_MONOTONIC_RAW
CLOCK_NAME = "CLOCK_MONOTONIC_RAW"
CLOCK_ID = "nano_monotonic_raw_ns"      # clock id of Nano frame identities / ages


def now_ns():
    """CLOCK_MONOTONIC_RAW in integer ns."""
    return time.clock_gettime_ns(_RAW)
