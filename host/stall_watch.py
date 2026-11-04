"""In-process stall diagnostics for the geometry producer (10-11 night, ~300 ms publish stall).

The 10-11 board runs saw one ~300 ms stall per formal run between the fusion and the return of
the rolling-map delta publish (v1 frame 27: 342 ms; v2 frame 214: 297 ms), with the producer
process at ~0 CPU through it. These helpers say WHERE the time went the next time it happens;
none of them changes what the producer computes or publishes.

  GcPauses      gc.callbacks start/stop -> every collection's duration (a GC pause burns CPU;
                a 0-CPU stall with no long collection is not GC)
  GilWatchdog   a thread that sleeps ``period_s`` and records every overshoot >= ``min_ms``:
                if a thread sits in a C call holding the GIL, this thread cannot wake on time
  StallDump     times a code section (event row when longer than ``timeout_s``); optionally arms
                faulthandler around it (stacks of ALL threads written by faulthandler's own C thread)
  DeadmanDump   faulthandler "dead man's switch" re-armed by the GilWatchdog every tick: when no
                Python thread can run for ``timeout_s`` (GIL held by a thread blocked in C, the
                10-11 stall signature), faulthandler dumps every thread's Python stack -- whichever
                thread is the blocked one; the watchdog then appends a footer with RAW times
  CallTimer     wall time of callbacks (max, count over a threshold, rows for the long ones)
  hold_gil_sleep / free_gil_sleep   positive controls: sleep in C with / without the GIL held

Event rows are queued (never written from a gc callback) and drained by the caller's writer.
Python 3.8 compatible; no ROS import.
"""

import collections
import ctypes
import faulthandler
import gc
import threading
import time

from host.fpga_clock import now_ns


class EventQueue(object):
    """Thread-safe queue of event dicts, bounded (oldest dropped and counted)."""

    def __init__(self, maxlen=10000):
        self._q = collections.deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self.dropped = 0

    def put(self, ev):
        with self._lock:
            if len(self._q) == self._q.maxlen:
                self.dropped += 1
            self._q.append(ev)

    def drain(self):
        with self._lock:
            out = list(self._q)
            self._q.clear()
        return out


class GcPauses(object):
    """gc.callbacks hook: count, total and max pause per generation; event rows >= min_ms."""

    def __init__(self, events, min_ms=10.0, clock=now_ns):
        self.events, self.min_ns, self.clock = events, int(min_ms * 1e6), clock
        self.n = [0, 0, 0]
        self.total_ns = [0, 0, 0]
        self.max_ns = [0, 0, 0]
        self._start = None
        self.installed = False

    def __call__(self, phase, info):
        if phase == "start":
            self._start = self.clock()
            return
        if self._start is None:
            return
        dt = self.clock() - self._start
        g = int(info.get("generation", 0))
        g = g if 0 <= g <= 2 else 2
        self.n[g] += 1
        self.total_ns[g] += dt
        if dt > self.max_ns[g]:
            self.max_ns[g] = dt
        if dt >= self.min_ns:
            self.events.put({"event": "gc_pause", "generation": g, "pause_ms": dt / 1e6,
                             "start_raw_ns": self._start, "collected": info.get("collected"),
                             "uncollectable": info.get("uncollectable")})
        self._start = None

    def install(self):
        if not self.installed:
            gc.callbacks.append(self)
            self.installed = True
        return self

    def uninstall(self):
        if self.installed:
            try:
                gc.callbacks.remove(self)
            except ValueError:
                pass
            self.installed = False

    def summary(self):
        return {"collections": list(self.n), "total_ms": [t / 1e6 for t in self.total_ns],
                "max_ms": [m / 1e6 for m in self.max_ns], "min_event_ms": self.min_ns / 1e6}


class DeadmanDump(object):
    """faulthandler.dump_traceback_later re-armed from the watchdog thread (see module doc).
    Only one faulthandler timer exists per process: do not combine with an armed StallDump."""

    def __init__(self, path, timeout_s=0.1, rearm_s=0.04, clock=now_ns):
        self.path, self.timeout_s, self.rearm_ns, self.clock = path, timeout_s, int(rearm_s * 1e9), clock
        self.fp = open(path, "a") if path else None
        self.armed_ns = None
        self.fired = 0

    def arm(self, now):
        if self.fp is None:
            return
        if self.armed_ns is None or now - self.armed_ns >= self.rearm_ns:
            faulthandler.dump_traceback_later(self.timeout_s, repeat=False, file=self.fp)
            self.armed_ns = now

    def note(self, t0, t1, late_ns):
        """Called by the watchdog after an overshoot: if the switch could have fired, say so."""
        if self.fp is None or self.armed_ns is None:
            return
        if t1 - self.armed_ns >= self.timeout_s * 1e9 and late_ns > 0:
            self.fired += 1
            self.fp.write("=== ABOVE: process frozen (no Python thread ran) watchdog_late_ms=%.3f "
                          "last_rearm_raw_ns=%d resumed_raw_ns=%d ===\n" % (late_ns / 1e6, self.armed_ns, t1))
            self.fp.flush()
            self.armed_ns = None

    def close(self):
        if self.fp is not None:
            faulthandler.cancel_dump_traceback_later()
            self.fp.close()
            self.fp = None

    def summary(self):
        return {"file": self.path, "timeout_ms": self.timeout_s * 1e3, "fired": self.fired}


class GilWatchdog(threading.Thread):
    """Sleeps ``period_s`` in a loop; an overshoot >= min_ms means this thread could not run
    (GIL held by a thread blocked in C, or the whole process descheduled)."""

    def __init__(self, events, period_s=0.02, min_ms=30.0, clock=now_ns, on_tick=None, deadman=None):
        super(GilWatchdog, self).__init__(name="stall-watchdog", daemon=True)
        self.events, self.period_s, self.min_ns, self.clock = events, period_s, int(min_ms * 1e6), clock
        self.on_tick = on_tick
        self.deadman = deadman
        self.stop_event = threading.Event()
        self.ticks = 0
        self.over = 0
        self.max_over_ns = 0

    def run(self):
        period_ns = int(self.period_s * 1e9)
        while not self.stop_event.is_set():
            t0 = self.clock()
            if self.deadman is not None:
                self.deadman.arm(t0)
            stopped = self.stop_event.wait(self.period_s)
            t1 = self.clock()
            over = (t1 - t0) - period_ns
            if stopped:
                # stop() arrived while this thread was still being kept from running (the owner
                # went straight from the stall to close(), other threads took the GIL first): the
                # stall is real, record it before leaving. A normal stop has over < 0.
                if over >= self.min_ns:
                    self.over += 1
                    self.max_over_ns = max(self.max_over_ns, over)
                    self.events.put({"event": "gil_stall", "late_ms": over / 1e6, "start_raw_ns": t0,
                                     "end_raw_ns": t1})
                break
            self.ticks += 1
            if self.deadman is not None and over >= self.min_ns:
                self.deadman.note(t0, t1, over)
            if over > self.max_over_ns:
                self.max_over_ns = over
            if over >= self.min_ns:
                self.over += 1
                self.events.put({"event": "gil_stall", "late_ms": over / 1e6, "start_raw_ns": t0,
                                 "end_raw_ns": t1})
            if self.on_tick is not None:
                try:
                    self.on_tick(t1)
                except Exception as exc:  # noqa: BLE001 - diagnostics never stop the producer
                    self.events.put({"event": "watchdog_tick_error", "error": repr(exc)[:200]})

    def stop(self):
        self.stop_event.set()

    def summary(self):
        return {"ticks": self.ticks, "overshoots": self.over, "max_overshoot_ms": self.max_over_ns / 1e6,
                "min_event_ms": self.min_ns / 1e6, "period_ms": self.period_s * 1e3}


class StallDump(object):
    """faulthandler stack dump of every thread when a section exceeds ``timeout_s``.

    faulthandler keeps one pending timer per process; only one thread (the producer worker)
    uses this. ``repeat=True``: a long stall gets one dump per timeout period."""

    def __init__(self, path, timeout_s=0.1, events=None, clock=now_ns):
        self.path, self.timeout_s, self.events, self.clock = path, timeout_s, events, clock
        self.fp = open(path, "a") if path else None
        self.sections = 0
        self.long = 0

    def begin(self):
        if self.fp is not None and self.timeout_s > 0:
            faulthandler.dump_traceback_later(self.timeout_s, repeat=True, file=self.fp)
        return self.clock()

    def end(self, t0, label, **detail):
        t1 = self.clock()
        if self.fp is not None and self.timeout_s > 0:
            faulthandler.cancel_dump_traceback_later()
        self.sections += 1
        dt = t1 - t0
        if dt >= self.timeout_s * 1e9:
            self.long += 1
            if self.fp is not None:
                self.fp.write("=== ABOVE: section %s start_raw_ns=%d end_raw_ns=%d ms=%.3f %s ===\n"
                              % (label, t0, t1, dt / 1e6, " ".join("%s=%s" % kv for kv in sorted(detail.items()))))
                self.fp.flush()
            if self.events is not None:
                ev = {"event": "section_stall", "section": label, "ms": dt / 1e6, "start_raw_ns": t0,
                      "end_raw_ns": t1, "dump_file": self.path}
                ev.update(detail)
                self.events.put(ev)
        return t1

    def close(self):
        if self.fp is not None:
            faulthandler.cancel_dump_traceback_later()
            self.fp.close()
            self.fp = None

    def summary(self):
        return {"sections": self.sections, "long": self.long, "timeout_ms": self.timeout_s * 1e3,
                "file": self.path}


class CallTimer(object):
    """Wraps callbacks; per name: calls, max ms, count >= warn_ms; event row >= event_ms."""

    def __init__(self, events, warn_ms=20.0, event_ms=50.0, clock=now_ns):
        self.events, self.warn_ns, self.event_ns, self.clock = events, int(warn_ms * 1e6), int(event_ms * 1e6), clock
        self.stats = {}
        self._lock = threading.Lock()

    def record(self, name, t0, t1):
        dt = t1 - t0
        with self._lock:
            s = self.stats.setdefault(name, {"calls": 0, "max_ms": 0.0, "over_warn": 0})
            s["calls"] += 1
            if dt / 1e6 > s["max_ms"]:
                s["max_ms"] = dt / 1e6
            if dt >= self.warn_ns:
                s["over_warn"] += 1
        if dt >= self.event_ns:
            self.events.put({"event": "slow_call", "name": name, "ms": dt / 1e6, "start_raw_ns": t0,
                             "end_raw_ns": t1})

    def wrap(self, name, fn):
        def timed(*args, **kw):
            t0 = self.clock()
            try:
                return fn(*args, **kw)
            finally:
                self.record(name, t0, self.clock())
        timed.__name__ = getattr(fn, "__name__", name)
        return timed

    def summary(self):
        with self._lock:
            return {"warn_ms": self.warn_ns / 1e6, "event_ms": self.event_ns / 1e6,
                    "calls": dict((k, dict(v)) for k, v in self.stats.items())}


_LIBC_PY = None


def hold_gil_sleep(ms):
    """Positive control: sleep in C WITH the GIL held (ctypes.PyDLL does not release it)."""
    global _LIBC_PY
    if _LIBC_PY is None:
        _LIBC_PY = ctypes.PyDLL(None)
    _LIBC_PY.usleep(ctypes.c_uint(int(ms * 1000)))


def free_gil_sleep(ms):
    """Positive control: sleep with the GIL released (time.sleep)."""
    time.sleep(ms / 1000.0)


def parse_debug_stall(spec):
    """'FRAME:MS:hold|free' -> (frame_id, ms, mode) or None. Test hook only."""
    if not spec:
        return None
    parts = spec.split(":")
    if len(parts) != 3 or parts[2] not in ("hold", "free"):
        raise ValueError("--debug-publish-stall FRAME:MS:hold|free")
    return int(parts[0]), float(parts[1]), parts[2]


# --- DDS publication mode of this process (v3, controller 10-11 night: candidate fix (d)) -------
# Humble rmw_fastrtps 6.2 sets every DataWriter's publish mode from RMW_FASTRTPS_PUBLICATION_MODE
# (SYNCHRONOUS default / ASYNCHRONOUS / AUTO), read when the participant is created (rclpy.init /
# first node). Only AUTO lets a Fast DDS XML profile's <publishMode> through: an XML profile alone
# (FASTRTPS_DEFAULT_PROFILES_FILE) is overridden by the SYNCHRONOUS default (measured 10-11 night,
# ASYNCHRONOUS or AUTO+XML 13 -- the asynchronous writer thread). So the switch sets the variable in
# this process only, before rclpy.init; nothing system-wide, other processes keep their mode.
DDS_PUBLISH_MODES = {"env": None, "sync": "SYNCHRONOUS", "async": "ASYNCHRONOUS"}


def apply_dds_publish_mode(mode, environ):
    """Set RMW_FASTRTPS_PUBLICATION_MODE in ``environ`` (os.environ) for ``mode`` (env: leave it);
    returns the value the rmw will read (None = unset, i.e. the rmw default SYNCHRONOUS)."""
    if mode not in DDS_PUBLISH_MODES:
        raise ValueError("dds publish mode %r not in %s" % (mode, sorted(DDS_PUBLISH_MODES)))
    value = DDS_PUBLISH_MODES[mode]
    if value is not None:
        environ["RMW_FASTRTPS_PUBLICATION_MODE"] = value
    return environ.get("RMW_FASTRTPS_PUBLICATION_MODE")


def thread_count():
    """Threads of this process (/proc/self/task; None off Linux) -- the asynchronous writer
    thread is the observable effect of ASYNCHRONOUS."""
    import os
    try:
        return len(os.listdir("/proc/self/task"))
    except OSError:
        return None
