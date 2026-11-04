"""Producer /tf reader QoS and the replay's depth wire path (2026-10-11 night, chain v2).

1. tf_listener_qos: dynamic /tf read BEST_EFFORT + VOLATILE + KEEP_LAST 100 by default
   (--tf-reliable -> RELIABLE, the tf2_ros default); the producer passes it as
   TransformListener(qos=...) and passes NO static_qos (tf2_ros keeps /tf_static reliable +
   transient_local). Variant caught: a profile that left reliability RELIABLE, or a call
   that overrode static_qos.
2. With rclpy (ROS image only, skipped on the host interpreters): DDS QoS compatibility via
   rclpy.qos.qos_check_compatible -- the tf2_ros broadcasters' profiles against the
   producer's readers are not ERROR and get the same verdict as the stock reliable pair (Humble
   says WARNING for system_default liveliness on both sides, nothing about reliability or
   durability); positive control: a best-effort writer against a
   reliable reader is reported incompatible (the checker is not blind). Live: a
   TransformBroadcaster and the producer's listener profile in one process; a lookup
   between two real samples interpolates, a lookup past the newest sample raises
   (no extrapolation), i.e. the semantics did not change with the reliability.
3. Replay depth wire: depth_wire() is a uint8 array.array with exactly the bytes of the old
   ``np.ascontiguousarray(codes, dtype="<u2").tobytes()`` path (also for a non-contiguous
   view and a big-endian input); the replay uses it for /fpga/depth.
4. summarize_chain_v2.cpu_window: machine CPU with and without the replay process.
Python 3.8+.
"""
import array
import csv
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import types
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host.geo_producer import TF_LISTENER_DEPTH, tf_listener_qos  # noqa: E402


def _fake_qos():
    class Profile(object):
        def __init__(self, **kw):
            self.kw = kw
    enum = lambda *names: types.SimpleNamespace(**{n: n.lower() for n in names})  # noqa: E731
    return types.SimpleNamespace(QoSProfile=Profile,
                                 ReliabilityPolicy=enum("RELIABLE", "BEST_EFFORT"),
                                 DurabilityPolicy=enum("VOLATILE", "TRANSIENT_LOCAL"),
                                 HistoryPolicy=enum("KEEP_LAST", "KEEP_ALL"))


try:
    import rclpy  # noqa: F401
    import tf2_ros  # noqa: F401
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


class TfQosProfile(unittest.TestCase):
    def test_default_is_best_effort_volatile_keep_last_100(self):
        p = tf_listener_qos(_fake_qos())
        self.assertEqual(p.kw, {"depth": 100, "history": "keep_last", "durability": "volatile",
                                "reliability": "best_effort"})
        self.assertEqual(TF_LISTENER_DEPTH, 100)

    def test_opt_out_is_reliable(self):
        p = tf_listener_qos(_fake_qos(), best_effort=False)
        self.assertEqual(p.kw["reliability"], "reliable")
        self.assertEqual(p.kw["durability"], "volatile")

    def test_variant_left_reliable_is_caught(self):
        # what a profile copied from the tf2_ros default (no reliability set -> RELIABLE) would give
        q = _fake_qos()
        variant = q.QoSProfile(depth=100, history="keep_last", durability="volatile", reliability="reliable")
        self.assertNotEqual(variant.kw, tf_listener_qos(q).kw)


class ProducerWiring(unittest.TestCase):
    SRC = (ROOT / "host" / "ros_geo_producer.py").read_text()

    def test_listener_gets_dynamic_qos_and_no_static_override(self):
        line = [ln for ln in self.SRC.splitlines() if "TransformListener(self.tf" in ln]
        self.assertEqual(len(line), 1)
        call = self.SRC[self.SRC.index(line[0]):]
        call = call[:call.index(")\n") + 1]
        self.assertIn("qos=tf_listener_qos(rclpy_qos, best_effort=not a.tf_reliable)", call)
        self.assertNotIn("static_qos", call)

    def test_flag_default_and_summary(self):
        from host.ros_geo_producer import parse_args
        a, _ = parse_args(["--data-source", "live", "--kv-server", "203.0.113.1:1", "--ground-z-m", "0"])
        self.assertFalse(a.tf_reliable)
        a, _ = parse_args(["--data-source", "live", "--kv-server", "203.0.113.1:1", "--ground-z-m", "0", "--tf-reliable"])
        self.assertTrue(a.tf_reliable)
        self.assertIn('"tf_qos": "reliable" if a.tf_reliable else "best_effort"', self.SRC)


@unittest.skipUnless(HAVE_ROS, "needs rclpy + tf2_ros (ROS 2 image)")
class TfQosWithRos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        import rclpy
        rclpy.shutdown()

    def test_compatibility(self):
        import rclpy.qos as q
        reader = tf_listener_qos(q)
        bcast = q.QoSProfile(depth=100)                 # tf2_ros.TransformBroadcaster default
        static_w = q.QoSProfile(depth=1, durability=q.DurabilityPolicy.TRANSIENT_LOCAL,
                                history=q.HistoryPolicy.KEEP_LAST)       # StaticTransformBroadcaster
        static_r = q.QoSProfile(depth=100, durability=q.DurabilityPolicy.TRANSIENT_LOCAL,
                                history=q.HistoryPolicy.KEEP_LAST)       # TransformListener static default
        # Humble reports WARNING for every pair whose liveliness is system_default on both sides
        # (also the stock reliable/reliable tf2 pair); only ERROR means "will not match", and the
        # reason text must not name reliability or durability.
        stock = q.qos_check_compatible(bcast, tf_listener_qos(q, best_effort=False))
        for verdict, reason in (q.qos_check_compatible(bcast, reader), q.qos_check_compatible(static_w, static_r)):
            self.assertNotEqual(verdict, q.QoSCompatibility.ERROR, reason)
            self.assertEqual(verdict, stock[0], reason)
            self.assertNotIn("reliability", reason.lower())
            self.assertNotIn("durability", reason.lower())
        # positive control: best-effort writer, reliable reader -> ERROR
        be_w = q.QoSProfile(depth=10, reliability=q.ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(q.qos_check_compatible(be_w, tf_listener_qos(q, best_effort=False))[0],
                         q.QoSCompatibility.ERROR)

    def test_live_interpolates_never_extrapolates(self):
        import rclpy
        import rclpy.qos as q
        from geometry_msgs.msg import TransformStamped
        from rclpy.duration import Duration
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformBroadcaster, TransformListener
        pub, sub = Node("tfqos_pub"), Node("tfqos_sub")
        buf = Buffer()
        TransformListener(buf, sub, qos=tf_listener_qos(q))
        tfb = TransformBroadcaster(pub)
        ex = SingleThreadedExecutor()
        ex.add_node(pub)
        ex.add_node(sub)
        try:
            end = time.monotonic() + 5.0
            while time.monotonic() < end and pub.count_subscribers("/tf") < 1:
                ex.spin_once(timeout_sec=0.05)
            self.assertGreaterEqual(pub.count_subscribers("/tf"), 1)    # matched (QoS compatible)
            for sec, x in ((100, 0.0), (101, 1.0)):
                m = TransformStamped()
                m.header.stamp.sec, m.header.frame_id, m.child_frame_id = sec, "odom", "base_link"
                m.transform.translation.x, m.transform.rotation.w = x, 1.0
                tfb.sendTransform(m)
            end = time.monotonic() + 5.0
            while time.monotonic() < end and not buf.can_transform("odom", "base_link", Time(seconds=101)):
                ex.spin_once(timeout_sec=0.05)
            t = buf.lookup_transform("odom", "base_link", Time(seconds=100, nanoseconds=500000000),
                                     timeout=Duration(seconds=0.0))
            self.assertAlmostEqual(t.transform.translation.x, 0.5, places=6)
            with self.assertRaises(Exception) as cm:
                buf.lookup_transform("odom", "base_link", Time(seconds=101, nanoseconds=100000000),
                                     timeout=Duration(seconds=0.0))
            self.assertIn("Extrapolation", type(cm.exception).__name__)
        finally:
            ex.shutdown()
            pub.destroy_node()
            sub.destroy_node()


class ReplayDepthWire(unittest.TestCase):
    def test_same_bytes_as_old_path(self):
        from tools.car_geo_producer.replay_camera import depth_wire
        rng = np.random.RandomState(7)
        full = rng.randint(0, 65536, size=(6, 36, 64)).astype("<u2")
        for codes in (full[3], full[2, ::2, 1:], full[1].astype(">u2")):
            out = depth_wire(codes)
            self.assertIsInstance(out, array.array)
            self.assertEqual(out.typecode, "B")
            self.assertEqual(out.tobytes(), np.ascontiguousarray(codes, dtype="<u2").tobytes())
        self.assertEqual(len(depth_wire(full[0])), 36 * 64 * 2)

    def test_replay_publishes_depth_through_it(self):
        src = (ROOT / "tools" / "car_geo_producer" / "replay_camera.py").read_text()
        self.assertIn("img.data = depth_wire(depth[src])", src)
        self.assertNotIn(".tobytes()\n        depth_pub", src)


class CpuExclReplay(unittest.TestCase):
    def test_machine_cpu_with_and_without_replay(self):
        from tools.integ_car_chain.summarize_chain_v2 import cpu_window
        hz = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
        d = tempfile.mkdtemp(prefix="cpuexcl_")
        with open(os.path.join(d, "cpu.json"), "w") as fp:
            json.dump({"t0_wall": 1000.0, "t1_wall": 1010.0}, fp)
        cols = ["raw_ns", "wall", "InDatagrams", "NoPorts", "InErrors", "RcvbufErrors", "SndbufErrors",
                "cpu_busy", "cpu_total", "load1", "producer", "replay"]
        with open(os.path.join(d, "sys.csv"), "w", newline="") as fp:
            w = csv.writer(fp)
            w.writerow(cols)
            # 10 s, 6 cores: total 6000 jiffies, busy 1800 (1.8 cores); replay 6 s of CPU (0.6 core)
            w.writerow([0, 1000.0, 0, 0, 0, 5, 0, 0, 0, 0.1, 0, 0])
            w.writerow([1, 1010.0, 0, 0, 0, 9, 0, 1800, 6000, 0.1, 4 * hz, 6 * hz])
        c = cpu_window(d)
        self.assertEqual(c["machine_busy_cores_of_6"], 1.8)
        self.assertEqual(c["pct_one_core"]["replay"], 60.0)
        self.assertEqual(c["machine_busy_cores_of_6_excl_replay"], 1.2)
        self.assertEqual(c["machine_busy_pct_all_cores_excl_replay"], 20.0)
        self.assertEqual(c["udp_rcvbuf_errors_in_window"], 4)


if __name__ == "__main__":
    unittest.main()
