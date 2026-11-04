"""SIGINT/SIGTERM shutdown of host/hub/hub_ros (10-11 integration board smoke: Ctrl+C and SIGTERM
did not stop the hub, no stats; after --duration-s it aborted with rc 134 "terminate called
without an active exception"). ROS-free: a fake rclpy that behaves like humble's -- its default
handlers (SignalHandlerOptions.ALL) only shut the context down inside the signal -- and a fake
executor whose spin thread runs until shutdown() wakes it.

1. Repro of the old main tail (rclpy.init default handlers, rclpy.spin daemon thread, run loop):
   SIGINT does not end run(), the stats are never written (bounded by a watchdog here).
2. serve_with_ros under SIGINT, SIGTERM, SIGINT inherited as ignored, and a signal before the
   run loop: run's finally writes the stats while the context is valid; executor shut down and
   its thread joined before destroy_node and rclpy.shutdown; previous handlers restored.
3. A second signal during the shutdown is ignored (the shutdown completes).
Python 3.8+.
"""
import os
from pathlib import Path
import signal
import sys
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host.hub.hub_ros import serve_with_ros  # noqa: E402

NO, ALL = 0, 3


class FakeRclpy(object):
    def __init__(self, log):
        self.valid = False
        self.log = log

    def init(self, args=None, signal_handler_options=None):
        self.valid = True
        opts = ALL if signal_handler_options is None else signal_handler_options
        self.log.append("init:%s" % opts)
        if opts != NO:                        # humble: the handler only shuts the context down
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, self._rcl_handler)

    def _rcl_handler(self, signum, frame):
        self.log.append("rcl_signal_shutdown")
        self.valid = False

    def ok(self):
        return self.valid

    def shutdown(self):
        self.log.append("rclpy_shutdown")
        self.valid = False

    def spin(self, node):                     # old main: rclpy.spin(node) in a daemon thread
        while self.valid:
            time.sleep(0.005)


class FakeExecutor(object):
    def __init__(self, rclpy, log):
        self.rclpy, self.log = rclpy, log
        self.stop = threading.Event()
        self.spinning = threading.Event()

    def add_node(self, node):
        self.log.append("add_node")

    def spin(self):
        self.spinning.set()
        while not self.stop.is_set() and self.rclpy.ok():
            time.sleep(0.005)
        self.log.append("spin_returned")

    def shutdown(self, timeout_sec=None):
        self.log.append("executor_shutdown")
        self.stop.set()
        return True


class FakeNode(object):
    def __init__(self, rclpy, log):
        self.rclpy, self.log = rclpy, log

    def destroy_node(self):
        self.log.append("destroy_node:ctx_valid=%s" % self.rclpy.ok())


def fake_run(rclpy, log, signum, stats, before_loop=False, second=None, max_s=3.0):
    """hub_server.run's shape: loop of sleeps, KeyboardInterrupt ends it, finally writes stats."""
    def run():
        if before_loop:
            os.kill(os.getpid(), signum)
            time.sleep(0.2)
            log.append("run_body_after_early_signal")
        t0 = time.monotonic()
        try:
            i = 0
            while time.monotonic() - t0 < max_s:
                if i == 3 and not before_loop:
                    os.kill(os.getpid(), signum)
                time.sleep(0.02)
                i += 1
            log.append("run_loop_timed_out")
        except KeyboardInterrupt:
            log.append("run_interrupted")
        finally:
            if second is not None:
                os.kill(os.getpid(), second)        # a second signal during the shutdown
                time.sleep(0.05)
            stats.append({"ctx_valid": rclpy.ok()})
            log.append("stats_written:ctx_valid=%s" % rclpy.ok())
    return run


class HubRosShutdownTest(unittest.TestCase):
    def setUp(self):
        self.saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}

    def tearDown(self):
        for s, h in self.saved.items():
            signal.signal(s, h)

    def arm(self, signum, before_loop=False, second=None):
        log, stats = [], []
        rclpy = FakeRclpy(log)
        ex = []

        def factory():
            e = FakeExecutor(rclpy, log)
            ex.append(e)
            return e
        serve_with_ros(rclpy, factory, lambda: FakeNode(rclpy, log),
                       fake_run(rclpy, log, signum, stats, before_loop, second), [], NO)
        return log, stats, ex[0]

    def check_order(self, log, stats):
        self.assertEqual(log[0], "init:%d" % NO)
        self.assertNotIn("rcl_signal_shutdown", log)
        self.assertNotIn("run_loop_timed_out", log)
        self.assertEqual(stats, [{"ctx_valid": True}])
        order = [log.index(x) for x in ("stats_written:ctx_valid=True", "executor_shutdown",
                                        "spin_returned", "destroy_node:ctx_valid=True", "rclpy_shutdown")]
        self.assertEqual(order, sorted(order), log)

    def test_sigint(self):
        log, stats, _ = self.arm(signal.SIGINT)
        self.assertIn("run_interrupted", log)
        self.check_order(log, stats)

    def test_sigterm(self):
        log, stats, _ = self.arm(signal.SIGTERM)
        self.assertIn("run_interrupted", log)
        self.check_order(log, stats)

    def test_sigint_inherited_ignored(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)      # background job of a non-interactive shell
        log, stats, _ = self.arm(signal.SIGINT)
        self.assertIn("run_interrupted", log)
        self.check_order(log, stats)

    def test_signal_before_run_loop(self):
        log, stats, _ = self.arm(signal.SIGINT, before_loop=True)
        self.assertNotIn("run_body_after_early_signal", log)
        self.assertEqual(stats, [])          # run's try had not started: nothing to write yet
        order = [log.index(x) for x in ("executor_shutdown", "spin_returned", "destroy_node:ctx_valid=True",
                                        "rclpy_shutdown")]
        self.assertEqual(order, sorted(order), log)

    def test_second_signal_during_shutdown_ignored(self):
        log, stats, _ = self.arm(signal.SIGINT, second=signal.SIGTERM)
        self.check_order(log, stats)

    def test_handlers_restored(self):
        before = signal.getsignal(signal.SIGTERM)
        self.arm(signal.SIGINT)
        self.assertIs(signal.getsignal(signal.SIGTERM), before)

    def test_old_main_repro(self):
        """Old tail: rclpy.init(default handlers) + rclpy.spin daemon thread + run(): SIGINT
        only shuts the context down, the run loop keeps going (stopped here by its time bound)."""
        log, stats = [], []
        rclpy = FakeRclpy(log)
        rclpy.init(args=[])
        node = FakeNode(rclpy, log)
        spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin.start()
        try:
            fake_run(rclpy, log, signal.SIGINT, stats, max_s=0.5)()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        self.assertIn("rcl_signal_shutdown", log)
        self.assertIn("run_loop_timed_out", log)        # Ctrl+C did not stop the hub
        self.assertNotIn("run_interrupted", log)
        self.assertEqual(stats, [{"ctx_valid": False}])  # and its context was already gone


if __name__ == "__main__":
    unittest.main()
