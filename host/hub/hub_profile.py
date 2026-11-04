"""Opt-in per-thread CPU accounting / profiling for the hub process (default off, zero cost).

The hub's work runs in many threads (rclpy executor spin, state tick, per-client sender and
reader, KV TELEM/e-stop/clock-sync loops), so a main-thread cProfile sees almost nothing.
When the environment variable ``FPGA_HUB_PROFILE`` names a directory, ``install()`` wraps
``threading.Thread.run`` so that every thread started afterwards

  * records its own CPU time (``time.thread_time()``, no tracing overhead) and wall time, and
  * with ``FPGA_HUB_PROFILE_MODE=cprofile`` also runs a ``cProfile.Profile`` timed by the thread's
    own CPU clock for its lifetime (heavy: a clock syscall per event, so only relative shares
    mean anything; never for the CPU comparison arms);

and on thread exit writes ``<dir>/<name>-<ident>.json`` (and ``.prof``). The main thread is
recorded by ``finish_main()``. Native threads the middleware starts (DDS) are not Python
threads: a sampler thread snapshots ``/proc/self/task/*/stat`` every ``FPGA_HUB_PROFILE_TASK_S``
seconds (default 5) into ``tasks.jsonl``, so every kernel task's cumulative CPU is on file and a
steady-state window can be cut; Python threads are matched to tasks by ``native_id``. With the
variable unset ``install()`` returns at once and ``threading.Thread.run`` is untouched.

A snapshot is not instantaneous when another Python thread is CPU-bound: each /proc read releases the
GIL and may wait one switch interval (5 ms) to get it back, so with ~50 kernel tasks one snapshot took
0.3 s beside a busy thread on the host (1.5 ms idle; measured 2026-10-12). ``mono`` is taken before the
reads; the period is the pause BETWEEN snapshots. Nothing may rely on N snapshots per wall time.
"""

import json
import os
import re
import threading
import time

ENV_DIR = "FPGA_HUB_PROFILE"
ENV_MODE = "FPGA_HUB_PROFILE_MODE"
ENV_TASK_S = "FPGA_HUB_PROFILE_TASK_S"
_state = {"dir": None, "mode": None, "main": None, "stop": None, "orig": None, "sampler": None}


def read_tasks(proc="/proc/self/task"):
    """{tid: [comm, utime_ticks, stime_ticks]} for every task of this process."""
    out = {}
    for tid in os.listdir(proc):
        try:
            with open(os.path.join(proc, tid, "stat"), "r") as f:
                raw = f.read()
        except OSError:
            continue
        comm = raw[raw.index("(") + 1:raw.rindex(")")]
        fields = raw[raw.rindex(")") + 2:].split()
        out[int(tid)] = [comm, int(fields[11]), int(fields[12])]
    return out


def _sampler(out_dir, period_s, stop):
    path = os.path.join(out_dir, "tasks.jsonl")
    with open(path, "a", encoding="utf-8") as f:
        while True:
            f.write(json.dumps({"mono": time.monotonic(), "clk_tck": os.sysconf("SC_CLK_TCK"),
                                "tasks": read_tasks(),
                                "py": dict((str(getattr(t, "native_id", None)), t.name)
                                           for t in threading.enumerate())}) + "\n")
            f.flush()
            if stop.wait(period_s):
                break


def _safe(name):
    return re.sub(r"[^A-Za-z0-9_.-]", "_", name or "thread")


def _write(out_dir, name, ident, rec, prof=None):
    base = os.path.join(out_dir, "%s-%d" % (_safe(name), ident))
    with open(base + ".json", "w", encoding="utf-8") as f:
        json.dump(rec, f)
    if prof is not None:
        prof.dump_stats(base + ".prof")


def install(environ=None):
    """Enable the per-thread hook if ``FPGA_HUB_PROFILE`` is set; returns the directory or None."""
    environ = os.environ if environ is None else environ
    out_dir = environ.get(ENV_DIR)
    if not out_dir:
        return None
    mode = environ.get(ENV_MODE) or "cpu"
    if mode not in ("cpu", "cprofile"):
        raise SystemExit("%s must be cpu or cprofile" % ENV_MODE)
    os.makedirs(out_dir, exist_ok=True)
    _state.update({"dir": out_dir, "mode": mode,
                   "main": (time.thread_time(), time.monotonic())})
    orig = threading.Thread.run
    _state["orig"] = orig

    def run(self):
        prof = None
        if mode == "cprofile":
            import cProfile
            prof = cProfile.Profile(time.thread_time)   # this thread's CPU, not wall (waits excluded)
            try:
                prof.enable()          # per thread on <= 3.11 (the ROS images run 3.10)
            except ValueError:         # >= 3.12: one sys.monitoring profiler per process
                prof = None
        c0, w0 = time.thread_time(), time.monotonic()
        try:
            orig(self)
        finally:
            if prof is not None:
                prof.disable()
            rec = {"name": self.name, "ident": self.ident, "native_id": getattr(self, "native_id", None),
                   "cpu_s": time.thread_time() - c0, "wall_s": time.monotonic() - w0, "mode": mode}
            try:
                _write(out_dir, self.name, self.ident, rec, prof)
            except OSError:
                pass

    threading.Thread.run = run
    stop = threading.Event()
    _state["stop"] = stop
    period = float(environ.get(ENV_TASK_S) or 5.0)
    sampler = threading.Thread(target=_sampler, args=(out_dir, period, stop), name="hub-profile-sampler",
                               daemon=True)
    sampler.start()
    _state["sampler"] = sampler
    return out_dir


def uninstall():
    """Undo install() (tests): stop the sampler, restore ``threading.Thread.run``."""
    if _state["dir"] is None:
        return
    _state["stop"].set()
    _state["sampler"].join(timeout=5)
    threading.Thread.run = _state["orig"]
    _state.update({"dir": None, "mode": None, "main": None, "stop": None, "orig": None, "sampler": None})


def finish_main():
    """Record the main thread's CPU (call once on the way out); no-op when not installed."""
    if _state["dir"] is None:
        return
    _state["stop"].set()
    _state["sampler"].join(timeout=2.0)       # a daemon cut at exit leaves an empty record
    c0, w0 = _state["main"]
    rec = {"name": "MainThread", "ident": threading.get_ident(),
           "native_id": getattr(threading, "get_native_id", lambda: None)(), "cpu_s": time.thread_time() - c0,
           "wall_s": time.monotonic() - w0, "mode": _state["mode"],
           "process_cpu_s": list(os.times()[:2])}
    _write(_state["dir"], "MainThread", threading.get_ident(), rec)
