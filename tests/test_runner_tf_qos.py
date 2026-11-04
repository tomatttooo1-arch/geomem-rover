"""Runner /tf reader QoS (2026-10-12): best effort by default, --tf-reliable keeps the old reader.

1. runner_tf_qos: dynamic /tf BEST_EFFORT + VOLATILE + KEEP_LAST 100 by default, RELIABLE with
   reliable=True; it is the producer's profile (host.geo_producer.tf_listener_qos), not a copy.
2. Wiring (source): the reference-pose branch builds the one TransformListener with
   qos=runner_tf_qos(tf_reliable), tf_reliable from args (getattr: offline fixtures' args have no
   such field), and passes no static_qos (/tf_static stays reliable + transient_local).
   Old-behaviour control: the same check on the runner at 877cedf (before this change) fails,
   i.e. the check is not blind.
3. CLI: --tf-reliable defaults off; given -> True (real argparse of main(), captured before ROS
   is imported).
4. With rclpy + tf2_ros (ROS image only; skipped on the host interpreters): a node with the
   runner's listener shows /tf BEST_EFFORT (RELIABLE with reliable=True) and /tf_static
   RELIABLE + TRANSIENT_LOCAL in its own subscription info; the listener still has tf_sub
   (what --stall-diag wraps).
Python 3.8+.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import types
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host import ros_runtime  # noqa: E402
from host.geo_producer import tf_listener_qos  # noqa: E402

OLD_COMMIT = "877cedf"
LISTENER_LINE = "TransformListener(self.buffer, self"


def _fake_qos():
    class Profile(object):
        def __init__(self, **kw):
            self.kw = kw
    enum = lambda *names: types.SimpleNamespace(**{n: n.lower() for n in names})  # noqa: E731
    return types.SimpleNamespace(QoSProfile=Profile,
                                 ReliabilityPolicy=enum("RELIABLE", "BEST_EFFORT"),
                                 DurabilityPolicy=enum("VOLATILE", "TRANSIENT_LOCAL"),
                                 HistoryPolicy=enum("KEEP_LAST", "KEEP_ALL"))


def wiring_problems(src):
    """[] when the runner's reference branch reads /tf through runner_tf_qos(args flag)."""
    problems = []
    lines = [ln for ln in src.splitlines() if LISTENER_LINE in ln]
    if len(lines) != 1:
        return ["%d listener lines" % len(lines)]
    call = src[src.index(lines[0]):]
    call = call[:call.index(")\n") + 1]
    if "qos=runner_tf_qos(tf_reliable)" not in call:
        problems.append("listener without qos=runner_tf_qos(tf_reliable)")
    if "static_qos" in call:
        problems.append("static_qos overridden")
    if 'tf_reliable = bool(getattr(args, "tf_reliable", False))' not in src:
        problems.append("tf_reliable not taken from args")
    if src.count("TransformListener(") != 1:
        problems.append("more than one TransformListener")
    return problems


class RunnerTfQosProfile(unittest.TestCase):
    def test_default_best_effort(self):
        p = ros_runtime.runner_tf_qos(qos=_fake_qos())
        self.assertEqual(p.kw, {"depth": 100, "history": "keep_last", "durability": "volatile",
                                "reliability": "best_effort"})

    def test_reliable_opt_out(self):
        p = ros_runtime.runner_tf_qos(True, qos=_fake_qos())
        self.assertEqual(p.kw, {"depth": 100, "history": "keep_last", "durability": "volatile",
                                "reliability": "reliable"})

    def test_same_profile_as_producer(self):
        q = _fake_qos()
        for reliable in (False, True):
            self.assertEqual(ros_runtime.runner_tf_qos(reliable, qos=q).kw,
                             tf_listener_qos(q, best_effort=not reliable).kw)
        # the producer's helper is called, not re-implemented
        with mock.patch("host.geo_producer.tf_listener_qos", return_value="sentinel") as m:
            self.assertEqual(ros_runtime.runner_tf_qos(True, qos=q), "sentinel")
        m.assert_called_once_with(q, best_effort=False)


class RunnerWiring(unittest.TestCase):
    SRC = (ROOT / "host" / "ros_runtime.py").read_text()

    def test_current_source(self):
        self.assertEqual(wiring_problems(self.SRC), [])

    @unittest.skipUnless(shutil.which("git") and (ROOT / ".git").exists(), "needs the git checkout")
    def test_old_runner_fails_the_check(self):
        try:
            old = subprocess.run(["git", "-C", str(ROOT), "show", OLD_COMMIT + ":host/ros_runtime.py"],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=True,
                                 universal_newlines=True).stdout
        except (subprocess.CalledProcessError, OSError):
            self.skipTest("commit %s not available" % OLD_COMMIT)
        self.assertIn("self.listener = TransformListener(self.buffer, self)\n", old)   # tf2_ros default reader
        self.assertIn("listener without qos=runner_tf_qos(tf_reliable)", wiring_problems(old))

    def test_check_catches_static_override(self):
        bad = self.SRC.replace("qos=runner_tf_qos(tf_reliable))",
                               "qos=runner_tf_qos(tf_reliable), static_qos=None)")
        self.assertNotEqual(bad, self.SRC)
        self.assertIn("static_qos overridden", wiring_problems(bad))


class _Parsed(Exception):
    pass


def parsed_args(argv):
    """main()'s own argparse result, captured before main imports rclpy."""
    real = argparse.ArgumentParser.parse_known_args
    box = {}

    def capture(self, args=None, namespace=None):
        box["r"] = real(self, args, namespace)
        raise _Parsed()
    with mock.patch.object(argparse.ArgumentParser, "parse_known_args", capture):
        try:
            ros_runtime.main(argv)
        except _Parsed:
            return box["r"][0]
    raise AssertionError("main() did not parse")


class RunnerFlag(unittest.TestCase):
    BASE = ["--socket", "/tmp/x.sock", "--pose-source", "reference", "--sensor-source", "hardware"]

    def test_default_off(self):
        self.assertFalse(parsed_args(self.BASE).tf_reliable)

    def test_flag_on(self):
        self.assertTrue(parsed_args(self.BASE + ["--tf-reliable"]).tf_reliable)


try:
    import rclpy  # noqa: F401
    import tf2_ros  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, "needs rclpy + tf2_ros (ROS 2 image)")
class RunnerTfQosWithRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown()

    def _own_sub_qos(self, reliable):
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener
        node = Node("runner_tfqos_%d_%d" % (int(reliable), os.getpid()))
        try:
            listener = TransformListener(Buffer(), node, qos=ros_runtime.runner_tf_qos(reliable),
                                         spin_thread=False)
            self.assertIsNotNone(getattr(listener, "tf_sub", None))
            name = node.get_fully_qualified_name()
            mine = lambda topic: [i for i in node.get_subscriptions_info_by_topic(topic)  # noqa: E731
                                  if "/" + i.node_name == name or i.node_name == node.get_name()]
            tf, static = mine("/tf"), mine("/tf_static")
            self.assertEqual((len(tf), len(static)), (1, 1))
            return tf[0].qos_profile, static[0].qos_profile
        finally:
            node.destroy_node()

    def test_live_profiles(self):
        from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
        for reliable, want in ((False, ReliabilityPolicy.BEST_EFFORT), (True, ReliabilityPolicy.RELIABLE)):
            tf, static = self._own_sub_qos(reliable)
            self.assertEqual(tf.reliability, want)
            self.assertEqual(tf.durability, DurabilityPolicy.VOLATILE)
            self.assertEqual(static.reliability, ReliabilityPolicy.RELIABLE)
            self.assertEqual(static.durability, DurabilityPolicy.TRANSIENT_LOCAL)


if __name__ == "__main__":
    unittest.main()
