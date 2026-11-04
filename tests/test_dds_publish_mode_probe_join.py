"""Publish-stall fixes of chain v3 (10-11 night): (d) the producer's DDS publication mode switch
and (a) the measurement probe joining the ROS graph before the measurement window.

1. stall_watch.apply_dds_publish_mode: env leaves the environment (unset or a preset value),
   sync/async set RMW_FASTRTPS_PUBLICATION_MODE=SYNCHRONOUS/ASYNCHRONOUS, anything else raises.
2. The producer's --dds-publish-mode (choices env/sync/async) is applied BEFORE rclpy.init (the
   rmw reads the variable when the participant is created; a call after init is a no-op --
   variant caught by the source order check) and lands in the summary with the thread count.
3. With rclpy (ROS image only; skipped on the host interpreters): a fresh process per mode,
   apply -> rclpy.init -> Node -> threads: ASYNCHRONOUS has exactly one more thread than
   SYNCHRONOUS (Fast DDS's asynchronous writer thread; known answer measured 12 vs 13 in
   fpga-dev1-sim:humble), env (unset) equals SYNCHRONOUS (the rmw default), and a bogus value
   makes the rmw print its "Using default SYNCHRONOUS" warning (the variable reaches it).
   An XML profile alone does not change the mode (documented in stall_watch; measured, not
   re-run here).
4. rig_inner.sh: PROBE_JOIN early (default) starts board_probe.py with --start-at-wall before
   the warm-up sleep and waits for it; late keeps the v1/v2 start; PUB_MODE passes
   --dds-publish-mode. board_probe.py gates its callbacks until the start time.
Python 3.8+.
"""
import os
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host import stall_watch  # noqa: E402

try:
    import rclpy  # noqa: F401
    HAVE_RCLPY = True
except ImportError:
    HAVE_RCLPY = False


class ApplyModeTests(unittest.TestCase):
    def test_modes(self):
        env = {}
        self.assertIsNone(stall_watch.apply_dds_publish_mode("env", env))
        self.assertEqual(env, {})
        env = {"RMW_FASTRTPS_PUBLICATION_MODE": "AUTO"}
        self.assertEqual(stall_watch.apply_dds_publish_mode("env", env), "AUTO")
        self.assertEqual(stall_watch.apply_dds_publish_mode("sync", env), "SYNCHRONOUS")
        self.assertEqual(env["RMW_FASTRTPS_PUBLICATION_MODE"], "SYNCHRONOUS")
        self.assertEqual(stall_watch.apply_dds_publish_mode("async", env), "ASYNCHRONOUS")
        with self.assertRaises(ValueError):
            stall_watch.apply_dds_publish_mode("ASYNCHRONOUS", {})

    def test_thread_count(self):
        if os.path.isdir("/proc/self/task"):
            self.assertGreaterEqual(stall_watch.thread_count(), 1)


class ProducerWiringTests(unittest.TestCase):
    SRC = (ROOT / "host" / "ros_geo_producer.py").read_text()

    def test_parse(self):
        from host.ros_geo_producer import parse_args
        base = ["--data-source", "live", "--kv-server", "203.0.113.1:1", "--ground-z-m", "0"]
        a, _ = parse_args(base)
        self.assertIn(a.dds_publish_mode, ("env", "sync", "async"))
        for m in ("env", "sync", "async"):
            a, _ = parse_args(base + ["--dds-publish-mode", m])
            self.assertEqual(a.dds_publish_mode, m)
        with self.assertRaises(SystemExit):
            parse_args(base + ["--dds-publish-mode", "fast"])

    def test_applied_before_init_and_reported(self):
        i_apply = self.SRC.index("stall_watch.apply_dds_publish_mode(a.dds_publish_mode, os.environ)")
        i_init = self.SRC.index("rclpy.init(args=ros_args)")
        self.assertLess(i_apply, i_init)
        self.assertEqual(self.SRC.count("rclpy.init("), 1)
        self.assertIn('"dds_publish_mode": {"switch": a.dds_publish_mode, "rmw_env": rmw_pub_mode', self.SRC)
        self.assertIn('"threads_after_node": threads_after_node', self.SRC)


CHILD = r"""
import os, sys, json
sys.path.insert(0, sys.argv[1])
from host import stall_watch
os.environ.pop("RMW_FASTRTPS_PUBLICATION_MODE", None)
if sys.argv[2] == "bogus":
    os.environ["RMW_FASTRTPS_PUBLICATION_MODE"] = "BOGUS"
else:
    stall_watch.apply_dds_publish_mode(sys.argv[2], os.environ)
import rclpy
from rclpy.node import Node
rclpy.init()
n = Node("dds_mode_test")
print(json.dumps({"threads": stall_watch.thread_count()}))
n.destroy_node(); rclpy.shutdown()
"""


@unittest.skipUnless(HAVE_RCLPY, "needs rclpy (ROS image)")
class LiveModeTests(unittest.TestCase):
    def run_child(self, mode):
        env = dict(os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID="91")
        env.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
        p = subprocess.run([sys.executable, "-c", CHILD, str(ROOT), mode], env=env, capture_output=True,
                           text=True, timeout=60)
        self.assertEqual(p.returncode, 0, p.stderr)
        import json
        return json.loads(p.stdout.strip().splitlines()[-1])["threads"], p.stderr + p.stdout

    def test_async_adds_the_writer_thread(self):
        t_sync, _ = self.run_child("sync")
        t_env, _ = self.run_child("env")
        t_async, _ = self.run_child("async")
        self.assertEqual(t_env, t_sync)
        self.assertEqual(t_async, t_sync + 1)

    def test_bogus_value_reaches_the_rmw(self):
        _, out = self.run_child("bogus")
        self.assertIn("Using default SYNCHRONOUS", out)


class ProbeJoinTests(unittest.TestCase):
    RIG = (ROOT / "tools" / "car_geo_producer" / "rig_inner.sh").read_text()
    PROBE = (ROOT / "tools" / "car_geo_producer" / "board_probe.py").read_text()

    def test_rig_starts_probe_before_the_window(self):
        i_start = self.RIG.index('board_probe.py --start-at-wall "$WIN_START"')
        i_sleep = self.RIG.index("time.sleep(max(0.0, float(sys.argv[1]) - time.time()))")
        i_t0 = self.RIG.index("T0=$(date +%s.%N)")
        self.assertLess(i_start, i_sleep)
        self.assertLess(i_sleep, i_t0)
        self.assertIn('if [ "${PROBE_JOIN:-early}" = early ]; then\n  wait $BPROBE', self.RIG)
        self.assertIn('board_probe.py --duration-s "$DUR" --out', self.RIG)          # late path kept
        self.assertIn('EXP+=(--dds-publish-mode "$PUB_MODE")', self.RIG)

    def test_probe_gates_until_start(self):
        self.assertIn('--start-at-wall', self.PROBE)
        for cb in ("on_status", "on_grid", "on_costmap", "on_frame"):
            self.assertIn("gate(%s)" % cb, self.PROBE)
        self.assertIn('pre = {"recording": a.start_at_wall is None', self.PROBE)


if __name__ == "__main__":
    unittest.main()
