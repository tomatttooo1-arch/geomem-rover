"""GeoMem 联调第 3 棒（2026-10-15）：仿真档页面找物被挡的修法（控制器裁定「里程计档允许寻物并提示」）。

hub（设计 §5 H2）：/odom 的位姿随 v/w 一起进 STATE odom 组（x/y/yaw/frame_id）；旧调用（不给位姿）字段为 None。
桥：仅仿真档、没有地图位姿时退到 odom 位姿并标 "odometry"；仿真档有地图位姿照旧 "map"；定位档不退（定位丢失照旧 "none"）。
不开任何监听端口。"""

import json
import sys
import threading
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ui" / "geomem" / "bridge"))

from host.hub.hub_core import HubCore, NS  # noqa: E402
import gmb_proto as gp  # noqa: E402


class FakeMsg(types.SimpleNamespace):
    pass


def robot(state, ri):
    return gp.compose_robot(gp.classify_view(state, ri), state, ri, "idle", False)


class HubOdomPoseTests(unittest.TestCase):
    def test_feed_odom_pose_reaches_state(self):
        core = HubCore("sim")
        core.feed_odom(0.2, 0.1, 5.0, NS, x=1.5, y=-0.5, yaw=0.25, frame_id="odom")
        o = core.compose_state(NS + 10 ** 7)["odom"]
        self.assertEqual((o["x"], o["y"], o["yaw"], o["frame_id"], o["v"]), (1.5, -0.5, 0.25, "odom", 0.2))

    def test_old_call_gives_none_pose_positive_control(self):
        core = HubCore("sim")
        core.feed_odom(0.2, 0.1, 5.0, NS)
        o = core.compose_state(NS + 10 ** 7)["odom"]
        self.assertEqual((o["x"], o["y"], o["yaw"], o["v"]), (None, None, None, 0.2))

    def test_nonfinite_pose_scrubbed_state_still_out(self):
        core = HubCore("sim")
        core.feed_odom(0.0, 0.0, 5.0, NS, x=float("nan"), y=1.0, yaw=0.0)
        s = core.compose_state(NS + 10 ** 7)
        self.assertIsNone(s["odom"]["x"])
        self.assertEqual(s["odom"]["y"], 1.0)

    def test_ros_callback_passes_pose(self):
        from host.hub import hub_ros
        subs = {}

        class Node(object):
            def __init__(self, name, **kw):
                pass

            def create_subscription(self, kind, topic, cb, qos, **kw):
                subs[topic] = cb

            def create_publisher(self, kind, topic, qos, **kw):
                return types.SimpleNamespace(publish=lambda m: None)

        m = types.SimpleNamespace(Node=Node, QoSProfile=lambda **kw: kw,
                                  DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1),
                                  ReliabilityPolicy=types.SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
                                  String=lambda data="": FakeMsg(data=data), Empty=lambda: FakeMsg(),
                                  Bool=object, Twist=object, PoseWithCovarianceStamped=object,
                                  OccupancyGrid=object, Odometry=object, Path=object, CompressedImage=object,
                                  SubscriptionEventCallbacks=lambda **kw: kw, PublisherEventCallbacks=lambda **kw: kw)
        core = HubCore("sim")
        hub_ros.build_node(types.SimpleNamespace(core=core, lock=threading.RLock()), m, preview=False, clock=lambda: 7 * NS)
        P = FakeMsg
        stamp = P(sec=12, nanosec=0)
        q = P(x=0.0, y=0.0, z=0.7071067811865476, w=0.7071067811865476)     # yaw = pi/2
        subs["/odom"](P(header=P(stamp=stamp, frame_id="odom"),
                        pose=P(pose=P(position=P(x=0.8, y=0.3, z=0.0), orientation=q)),
                        twist=P(twist=P(linear=P(x=0.25), angular=P(z=0.05)))))
        o = core.compose_state(8 * NS)["odom"]
        self.assertEqual((o["x"], o["y"], o["frame_id"], o["v"]), (0.8, 0.3, "odom", 0.25))
        self.assertAlmostEqual(o["yaw"], 1.5707963, places=5)
        # 没有 pose 字段的消息（旧测试替身）照旧只给 v/w
        subs["/odom"](P(header=P(stamp=stamp), twist=P(twist=P(linear=P(x=0.1), angular=P(z=0.0)))))
        o = core.compose_state(9 * NS)["odom"]
        self.assertEqual((o["x"], o["v"]), (None, 0.1))


class BridgeOdomFallbackTests(unittest.TestCase):
    RI = {"pose_source": "reference"}

    def st(self, source="sim", pose=None, odom=None):
        return {"source": source, "pose": pose or {"x": None, "y": None, "yaw": None, "loc_state": None, "age_ms": None},
                "odom": odom or {"x": 1.0, "y": -0.5, "yaw": 0.3, "v": -0.1, "age_ms": 40},
                "nav": {"motion_permit": False, "cmd_v": 0.0}}

    def test_sim_without_map_pose_uses_odometry(self):
        r = robot(self.st(), self.RI)
        self.assertEqual((r["position"], r["yaw"], r["pose"], r["poseAgeMs"], r["speed"]),
                         ([1.0, -0.5, 0.0], 0.3, "odometry", 40.0, 0.1))

    def test_sim_with_map_pose_unchanged(self):
        st = self.st(pose={"x": 2.0, "y": 3.0, "yaw": 0.1, "loc_state": None, "age_ms": 20})
        r = robot(st, self.RI)
        self.assertEqual((r["position"], r["pose"], r["poseAgeMs"]), ([2.0, 3.0, 0.0], "map", 20.0))

    def test_sim_without_any_pose_is_none(self):
        self.assertIsNone(robot(self.st(odom={"v": 0.0, "age_ms": 10}), self.RI))

    def test_missing_yaw_gives_none_not_bad_frame(self):
        self.assertIsNone(robot(self.st(odom={"x": 1.0, "y": 2.0, "yaw": None, "age_ms": 10}), self.RI))

    def test_loc_tier_does_not_fall_back(self):
        ri = {"pose_source": "estimated"}
        lost = self.st(source="live", pose={"x": 1.0, "y": 2.0, "yaw": 0.0, "loc_state": "lost", "age_ms": 30})
        self.assertEqual(robot(lost, ri)["pose"], "none")       # 定位丢失照旧「未定位」，找物被拦
        nopose = self.st(source="live", pose={"x": None, "y": None, "yaw": None, "loc_state": "initializing"})
        self.assertIsNone(robot(nopose, ri))                    # 定位档没有地图位姿不退到里程计

    def test_end_to_end_real_hubcore_sim(self):
        core = HubCore("sim")
        core.feed_status(json.dumps({"event": "startup", "sensor_source": "sim"}), NS)
        core.feed_runtime_info(json.dumps({"pose_source": "reference", "sensor_source": "sim"}), NS)
        core.feed_odom(0.0, 0.0, 5.0, NS, x=0.4, y=0.2, yaw=0.0, frame_id="odom")
        s = core.compose_state(NS + 10 ** 7)
        ri = s["task"].get("runtime_info")
        ri = ri.get("body", ri) if isinstance(ri, dict) else ri
        view = gp.classify_view(s, ri)
        self.assertEqual(view["tier"], "sim")
        r = gp.compose_robot(view, s, ri, "idle", False)
        self.assertEqual((r["position"][:2], r["pose"]), ([0.4, 0.2], "odometry"))
        # 阳性对照：同一 hub 不给 odom 位姿（修前的 STATE）→ 桥给 null（页面就会挡找物）
        core2 = HubCore("sim")
        core2.feed_status(json.dumps({"event": "startup", "sensor_source": "sim"}), NS)
        core2.feed_odom(0.0, 0.0, 5.0, NS)
        s2 = core2.compose_state(NS + 10 ** 7)
        self.assertIsNone(gp.compose_robot(gp.classify_view(s2, ri), s2, ri, "idle", False))


if __name__ == "__main__":
    unittest.main()
