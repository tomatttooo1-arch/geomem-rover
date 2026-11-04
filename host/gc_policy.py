"""Python garbage-collector policy for the car-chain processes (2026-10-12, integration v7).

Board v6: one generation-2 collection per 180 s run in the producer (31-47 ms, inside the
measurement window, a single-frame age spike) and one in the runner (start-up). A full
collection walks every tracked object, most of which (rclpy, numpy, message types, the node)
are created at start-up and live for the whole run.

  default  unchanged interpreter behaviour
  freeze   after the node is built and before it spins: gc.collect() then gc.freeze() -- every
           object alive then moves to the permanent generation, so later collections (also the
           full ones) walk only what was created after start-up. Thresholds are unchanged.
Python 3.8 compatible (gc.freeze since 3.7).
"""

import gc
import time

MODES = ("default", "freeze")


def rss_kb():
    """VmRSS of this process in kB (Linux), None elsewhere."""
    try:
        with open("/proc/self/status") as fp:
            for line in fp:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return None


def apply(mode):
    """Apply ``mode`` once (call after initialisation, before the loop). Returns a description
    for the run's summary / runtime_info."""
    if mode not in MODES:
        raise ValueError("gc mode must be one of %s" % (MODES,))
    info = {"mode": mode, "thresholds": list(gc.get_threshold()), "rss_kb_before": rss_kb()}
    if mode == "freeze":
        t0 = time.perf_counter()
        collected = gc.collect()
        gc.freeze()
        info.update(collect_ms=round((time.perf_counter() - t0) * 1e3, 3), collected=collected,
                    frozen=gc.get_freeze_count())
    info["rss_kb_after"] = rss_kb()
    return info
