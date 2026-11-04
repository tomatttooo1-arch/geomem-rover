"""Producer depth subscription QoS switch (2026-10-15, camera-drop batch 2).

1. depth_sub_qos default returns the sensor data profile object itself (behaviour before the switch).
2. reliable=True gives RELIABLE + VOLATILE + KEEP_LAST depth.
3. ros_geo_producer: --depth-reliable / --depth-qos-depth parsed, both depth subscriptions use one profile,
   summary records depth_qos.
"""
import os
import sys
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.geo_producer import DEPTH_SUB_DEPTH, depth_sub_qos  # noqa: E402


def _fake_qos():
    class Profile(object):
        def __init__(self, **kw):
            self.kw = kw
    enum = lambda *names: types.SimpleNamespace(**{n: n.lower() for n in names})  # noqa: E731
    return types.SimpleNamespace(QoSProfile=Profile, qos_profile_sensor_data=object(),
                                 ReliabilityPolicy=enum("RELIABLE", "BEST_EFFORT"),
                                 DurabilityPolicy=enum("VOLATILE", "TRANSIENT_LOCAL"),
                                 HistoryPolicy=enum("KEEP_LAST", "KEEP_ALL"))


class DepthQos(unittest.TestCase):
    def test_default_is_sensor_data_itself(self):
        q = _fake_qos()
        self.assertIs(depth_sub_qos(q), q.qos_profile_sensor_data)
        self.assertEqual(DEPTH_SUB_DEPTH, 5)

    def test_reliable(self):
        p = depth_sub_qos(_fake_qos(), reliable=True, depth=7)
        self.assertEqual(p.kw, {"depth": 7, "history": "keep_last", "durability": "volatile",
                                "reliability": "reliable"})

    def test_reliable_default_depth(self):
        self.assertEqual(depth_sub_qos(_fake_qos(), reliable=True).kw["depth"], 5)


class ProducerSource(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT, "host", "ros_geo_producer.py")) as f:
            self.src = f.read()

    def test_args_and_wiring(self):
        self.assertIn('"--depth-reliable"', self.src)
        self.assertIn('"--depth-qos"', self.src)
        self.assertIn('"--depth-qos-depth"', self.src)
        self.assertIn("depth_sub_qos(rclpy_qos, reliable=depth_reliable(a), depth=a.depth_qos_depth)", self.src)
        # both depth-side subscriptions take the switched profile, none is left on sensor data
        self.assertIn("cb(\"on_info\", self.on_info), depth_qos)", self.src)
        self.assertIn("cb(\"on_depth\", self.on_depth), depth_qos)", self.src)
        self.assertNotIn("self.on_depth), qos_profile_sensor_data", self.src)
        self.assertIn('"depth_qos":', self.src)

    def test_session_r_arm_passes_flag(self):
        with open(os.path.join(ROOT, "docs", "design", "integ_car_chain_v3_20261011", "session_v3.sh")) as f:
            s = f.read()
        self.assertIn('r*) PX="--depth-reliable" ;; *) PX="--depth-qos=sensor_data" ;;', s)
        self.assertIn("\"'$PX'\" \"'$CE'\"", s)


class AutoByDataSource(unittest.TestCase):
    """--depth-qos auto: reliable only for live (the replay camera / sim adapter publish best effort)."""

    def _parse(self, argv):
        from host.ros_geo_producer import depth_reliable, parse_args
        return depth_reliable(parse_args(argv + ["--ground-z-m", "0", "--kv-server", "127.0.0.1:1"])[0])

    def test_resolution(self):
        self.assertTrue(self._parse(["--data-source", "live"]))
        self.assertFalse(self._parse(["--data-source", "replay"]))
        self.assertFalse(self._parse(["--data-source", "sim"]))
        self.assertFalse(self._parse(["--data-source", "live", "--depth-qos", "sensor_data"]))
        self.assertFalse(self._parse(["--data-source", "live", "--depth-qos=sensor_data"]))
        self.assertTrue(self._parse(["--data-source", "replay", "--depth-reliable"]))
        self.assertTrue(self._parse(["--data-source", "sim", "--depth-qos", "reliable"]))

    def test_best_effort_publishers_documented(self):
        # the publishers a reliable reader would not match: keep them best effort and the auto rule in step
        for rel in ("tools/car_geo_producer/replay_camera.py", "robot_sim/fpga_sim/scripts/sim_adapter.py"):
            with open(os.path.join(ROOT, rel)) as f:
                self.assertIn("qos_profile_sensor_data", f.read(), rel)


if __name__ == "__main__":
    unittest.main()
