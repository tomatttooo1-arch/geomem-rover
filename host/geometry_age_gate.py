"""Task runner's 200 ms geometry age gate at the point of use (owner 2026-10-11 21:00).

The runner (host/ros_runtime.py) hands one geometry dict per recognition frame to
``Memory.update_memory``. Whatever produced it (``--geometry-source`` cpu, kv or kv_topic,
or the explicit ``--geometry-fallback cpu``), the depth frame behind it must not be older
than 200 ms at that moment; an older one is used as "geometry unavailable" (``{}``, the
same as kv with fallback none), with reason ``geometry_stale`` and its age.

Same rule as the producer, not a new one:
  * limit: ``MAX_GEOMETRY_AGE_NS`` = 200 ms = geo_producer.MAX_FRAME_AGE_NS =
    M7Pipeline's default max_frame_age_ns = rolling_nav_layer.MAX_MAP_AGE_NS (D-035);
  * comparison: ``age > limit`` is stale (M7Pipeline._age ``age > limit``, geo_producer
    ``answered_age > self.max_frame_age_ns``), so exactly 200 ms still passes;
    ``age < 0`` is a future frame (M7Pipeline ``future_frame`` with tolerance 0);
  * clock: host.fpga_clock (CLOCK_MONOTONIC_RAW, clock id ``nano_monotonic_raw_ns``); a
    stamp carrying another clock id is never subtracted (M7Pipeline ``clock_mismatch``);
  * origin: the RAW arrival of the depth message -- for kv_topic the producer's own
    ``producer.acquired_raw_ns`` (the exact number the producer ages), for cpu/kv the
    runner's on_images arrival of the same /fpga/depth (the definition the kv path already
    used for its frame age; it differs from the producer's by the two callbacks' latency).

A frame without an origin stamp (a body without ``acquired_raw_ns``, a test fixture that
bypasses on_images) has an unknown age: it is refused (safe side, controller 2026-10-12),
reason ``geometry_age_unknown``, like any other refusal (event, counter ``refused:...``).
Until 07eccb8 it passed and was only counted ``origin_missing``.

Python 3.8 compatible, no ROS import.
"""

import collections

from host.fpga_clock import CLOCK_ID

MAX_GEOMETRY_AGE_NS = 200_000_000          # D-035; equal to geo_producer.MAX_FRAME_AGE_NS
# 10-26 owner：重成功率，放宽自 200 ms -- the RUNNER's point-of-use limit (ros_runtime builds its gate with it);
# the producer / pipeline / rolling layer keep MAX_GEOMETRY_AGE_NS. A stale geometry only drops negative
# evidence and the occlusion check for that frame (no stop, no confirmation change); 10-22: 67 % refused at 200 ms.
RUNNER_MAX_GEOMETRY_AGE_NS = 500_000_000

STALE = "geometry_stale"
FUTURE = "geometry_future"
CLOCK_MISMATCH = "geometry_clock_mismatch"
AGE_UNKNOWN = "geometry_age_unknown"       # no origin stamp: refused (2026-10-12; was passed)


def age_verdict(origin_ns, now_ns, *, origin_clock_id=None, limit_ns=MAX_GEOMETRY_AGE_NS):
    """(age_ns | None, reason | None). reason None = usable; a missing origin is refused."""
    if origin_ns is None:
        return None, AGE_UNKNOWN
    if origin_clock_id is not None and origin_clock_id != CLOCK_ID:
        return None, CLOCK_MISMATCH
    age = int(now_ns) - int(origin_ns)
    if age < 0:
        return age, FUTURE
    if age > limit_ns:
        return age, STALE
    return age, None


class GeometryAgeGate(object):
    """Counts for the runner summary (``geometry_close.age_gate``)."""

    def __init__(self, limit_ns=MAX_GEOMETRY_AGE_NS):
        limit_ns = int(limit_ns)
        if not 0 < limit_ns <= RUNNER_MAX_GEOMETRY_AGE_NS:
            raise ValueError("geometry age limit must be in (0, 500 ms] (D-035 200 ms; runner 500 ms since 10-26)")
        self.limit_ns = limit_ns
        self.counts = collections.Counter()
        self.max_age_ns = None
        self.max_used_age_ns = None

    def check(self, origin_ns, now_ns, *, origin_clock_id=None):
        """(age_ns | None, reason | None) and count it. Only call for a non-empty geometry."""
        age, reason = age_verdict(origin_ns, now_ns, origin_clock_id=origin_clock_id, limit_ns=self.limit_ns)
        self.counts["checked"] += 1
        if age is not None and (self.max_age_ns is None or age > self.max_age_ns):
            self.max_age_ns = age
        if reason is None:
            self.counts["used"] += 1
            if age is not None and (self.max_used_age_ns is None or age > self.max_used_age_ns):
                self.max_used_age_ns = age
        else:
            self.counts["refused:" + reason] += 1
        return age, reason

    @property
    def refused(self):
        return sum(n for k, n in self.counts.items() if k.startswith("refused:"))

    def summary(self):
        return {"limit_ms": self.limit_ns / 1e6, "counts": dict(self.counts),
                "max_age_ms": None if self.max_age_ns is None else self.max_age_ns / 1e6,
                "max_used_age_ms": None if self.max_used_age_ns is None else self.max_used_age_ns / 1e6}
