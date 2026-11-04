"""SIGINT/SIGTERM shutdown order of host/ros_runtime.main (2026-10-11 Nano runs: rc=1, the
geometry_close event lost). ROS-free: a fake rclpy that behaves like humble's -- its default
signal handlers (SignalHandlerOptions.ALL) shut the context down inside the signal, spin then
raises ExternalShutdownException, and a publisher of a shut-down context raises.

1. Repro: the old main tail (verbatim below) under SIGINT -> RCLError out of close(), the
   geometry_close row never reaches the events file, motion stop never published.
2. spin_until_interrupt under SIGINT, SIGTERM, and SIGINT inherited as ignored: no exception,
   close() runs while the context is valid (stop publishes and the event go out), shutdown after.
Python 3.8+.
"""
import os
from pathlib import Path
import signal
import sys
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host.ros_runtime import spin_until_interrupt  # noqa: E402

NO, ALL = 0, 3


class RCLError(RuntimeError):
    pass


class ExternalShutdownException(Exception):
    pass


class FakeRclpy(object):
    def __init__(self):
        self.valid = False
        self.options = None
        self.log = []

    def init(self, args=None, signal_handler_options=None):
        self.valid = True
        self.options = ALL if signal_handler_options is None else signal_handler_options
        if self.options != NO:                       # humble: the handler shuts the context down
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, self._rcl_handler)

    def _rcl_handler(self, signum, frame):
        self.log.append("rcl_signal_shutdown")
        self.valid = False

    def ok(self):
        return self.valid

    def shutdown(self):
        self.log.append("shutdown")
        self.valid = False

    def spin(self, node):
        for i in range(500):                         # the 0.1 s watchdog returns spin to Python
            if not self.valid:
                raise ExternalShutdownException()
            node.tick(i)
            threading.Event().wait(0.005)
        raise AssertionError("signal never ended spin")


class FakeNode(object):
    def __init__(self, rclpy, events_path, signum):
        self.rclpy, self.events_path, self.signum = rclpy, events_path, signum
        self.diagnostics = None
        self.published = []

    def tick(self, i):
        if i == 3:
            os.kill(os.getpid(), self.signum)

    def publish(self, what):
        if not self.rclpy.ok():
            raise RCLError("Failed to publish: publisher's context is invalid")
        self.published.append(what)

    def record(self, kind):                         # Runtime.record: file row first, then publish
        with open(self.events_path, "a") as f:
            f.write(kind + "\n")
        self.publish(kind)

    def close(self):
        self.rclpy.log.append("close")
        try:
            self.publish("motion_permit=false")       # Runtime.stop()
        finally:
            self.record("geometry_close")

    def destroy_node(self):
        self.rclpy.log.append("destroy")


def legacy_main_tail(rclpy, make_node, ros_args):
    """host/ros_runtime.main before this fix (b790f1a), verbatim apart from names."""
    rclpy.init(args=ros_args)
    node = None
    try:
        node = make_node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try:
                node.close()
            finally:
                if node.diagnostics:
                    node.diagnostics.flush()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class ShutdownOrderTests(unittest.TestCase):
    def setUp(self):
        import tempfile
        self.saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        self.dir = tempfile.mkdtemp(prefix="rt_shutdown_")
        self.events = os.path.join(self.dir, "events.jsonl")

    def tearDown(self):
        for s, h in self.saved.items():
            signal.signal(s, h)

    def rows(self):
        return open(self.events).read().split() if os.path.exists(self.events) else []

    def run_fixed(self, signum):
        rclpy = FakeRclpy()
        nodes = []

        def make():
            nodes.append(FakeNode(rclpy, self.events, signum))
            return nodes[0]
        spin_until_interrupt(rclpy, make, [], NO)
        return rclpy, nodes[0]

    def test_legacy_sigint_loses_event_and_raises(self):
        rclpy = FakeRclpy()
        node = FakeNode(rclpy, self.events, signal.SIGINT)
        with self.assertRaises(RCLError):
            legacy_main_tail(rclpy, lambda: node, [])
        self.assertEqual(node.published, [])          # motion stop never went out
        self.assertIn("rcl_signal_shutdown", rclpy.log)

    def test_sigint_closes_on_live_context(self):
        rclpy, node = self.run_fixed(signal.SIGINT)
        self.assertEqual(rclpy.options, NO)
        self.assertEqual(node.published, ["motion_permit=false", "geometry_close"])
        self.assertEqual(self.rows(), ["geometry_close"])
        self.assertEqual(rclpy.log, ["close", "destroy", "shutdown"])
        self.assertIs(signal.getsignal(signal.SIGINT), self.saved[signal.SIGINT])   # restored

    def test_sigterm_same_path(self):
        rclpy, node = self.run_fixed(signal.SIGTERM)
        self.assertEqual(node.published, ["motion_permit=false", "geometry_close"])
        self.assertEqual(rclpy.log, ["close", "destroy", "shutdown"])

    def test_sigint_inherited_ignored_still_closes(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # `cmd &` in a non-interactive shell
        rclpy, node = self.run_fixed(signal.SIGINT)
        self.assertEqual(self.rows(), ["geometry_close"])
        self.assertIs(signal.getsignal(signal.SIGINT), signal.SIG_IGN)


if __name__ == "__main__":
    unittest.main()
