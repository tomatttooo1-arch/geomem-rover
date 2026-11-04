"""GeoMem 联调棒（2026-10-15）新增的单测：桥丢弃无类名的物体/候选（浏览器校验要求 className 必填，
否则整帧被拒——T2 A3 在真桥上发现）；试验台入口在隔离网络命名空间外拒绝运行（owner：宿主不开 hub 端口）。
不开任何监听端口。"""

import json
import os
import subprocess
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ui" / "geomem" / "bridge"))

import gmb_proto as gp  # noqa: E402

TESTBED = ROOT / "ui" / "geomem" / "bridge" / "testbed"


def track(tid, label, **kw):
    t = {"id": tid, "label": label, "color": "blue", "position_m": [1.0, 2.0, 0.3], "state": "current_confirmed",
         "last_seen": 100.0, "history_last": {"score": 0.8}}
    t.update(kw)
    return t


class NoLabelDropTests(unittest.TestCase):
    def test_objects_without_class_are_dropped_and_counted(self):
        counts = {}
        mem = {"tracks": [track("object-0001", None), track("object-0002", "cup"), track("object-0003", ""),
                          track("object-0004", 7)]}
        out = gp.compose_objects(mem, counts)
        self.assertEqual([o["id"] for o in out], ["object-0002"])
        self.assertEqual(counts.get("object_no_label"), 3)
        for o in out:                       # mirror of the browser validator's className rule
            self.assertIsInstance(o["className"], str)
            self.assertTrue(o["className"])

    def test_labelled_objects_untouched_positive_control(self):
        counts = {}
        out = gp.compose_objects({"tracks": [track("object-0001", "cup"), track("object-0002", "bottle")]}, counts)
        self.assertEqual([o["className"] for o in out], ["cup", "bottle"])
        self.assertNotIn("object_no_label", counts)

    def test_dropped_no_label_id_does_not_shadow_a_later_duplicate(self):
        out = gp.compose_objects({"tracks": [track("object-0001", None), track("object-0001", "cup")]})
        self.assertEqual([(o["id"], o["className"]) for o in out], [("object-0001", "cup")])

    def test_candidates_without_class_are_dropped(self):
        counts = {}
        g = {"state": "ambiguous", "reason_task": "candidate_selection_required", "target_id": None,
             "candidates": [{"id": "object-0001", "label": None, "color": "red", "position_m": [1, 1, 0]},
                            {"id": "object-0002", "label": "cup", "color": "blue", "position_m": [2, 1, 0]}]}
        t = gp.compose_task(g, "boot:1", counts)
        self.assertEqual([c["id"] for c in t["candidates"]], ["object-0002"])
        self.assertEqual(counts.get("candidate_dropped"), 1)
        self.assertTrue(all(isinstance(c["className"], str) and c["className"] for c in t["candidates"]))


class SourceDetailTests(unittest.TestCase):
    """10-15 联调第 2 棒小修 a：回放细分 sourceDetail（recording | synthetic | null）。"""

    def test_detail_by_markers(self):
        f = gp.frame_source_detail
        self.assertIsNone(f({"source": "live", "replay_file": "pc.fpgarec"}))      # 非回放一律 null
        self.assertIsNone(f({"source": "sim"}))
        self.assertIsNone(f(None))
        self.assertEqual(f({"source": "replay", "replay_id": "hub-replay-feeder"}), "synthetic")
        self.assertEqual(f({"source": "replay", "replay_id": None, "replay_file": "pc.fpgarec"}), "recording")
        # 录下来的合成会话再回放：仍算合成（先判 synthetic）
        self.assertEqual(f({"source": "replay", "replay_id": "hub-replay-feeder", "replay_file": "x.fpgarec"}),
                         "synthetic")
        for bad in ("", None, 7):
            self.assertIsNone(f({"source": "replay", "replay_id": "t2-fixture", "replay_file": bad}))

    def test_constant_matches_hub_server_text(self):
        src = (ROOT / "host" / "hub" / "hub_server.py").read_text(encoding="utf-8")
        feeder = src[src.index("class ReplayFeeder"):]
        self.assertIn('"replay_id": "%s"' % gp.HUB_SYNTHETIC_REPLAY_ID, feeder)

    def test_real_hub_core_synthetic_startup_gives_synthetic(self):
        sys.path.insert(0, str(ROOT))
        from host.hub.hub_core import HubCore
        core = HubCore("replay")
        core.feed_status(json.dumps({"event": "startup", "sensor_source": "simulation",
                                     "replay_id": gp.HUB_SYNTHETIC_REPLAY_ID}), 1)
        st = json.loads(json.dumps(core.compose_state(2)))
        self.assertEqual(st["source"], "replay")
        self.assertEqual(gp.frame_source_detail(st), "synthetic")
        core2 = HubCore("replay")                  # 对照：别的 replay_id 不算合成
        core2.feed_status(json.dumps({"event": "startup", "sensor_source": "simulation", "replay_id": "t2-fixture"}), 1)
        self.assertIsNone(gp.frame_source_detail(core2.compose_state(2)))

    def test_real_force_replay_stamp_gives_recording(self):
        sys.path.insert(0, str(ROOT))
        from pc import fpgac
        from host.hub.hub_core import HubCore
        live = json.loads(json.dumps(HubCore("live").compose_state(1)))
        self.assertIsNone(gp.frame_source_detail(live))
        st = fpgac.force_replay(fpgac.KIND["STATE"], live, "pc.fpgarec")
        self.assertEqual(st["source"], "replay")
        self.assertEqual(gp.frame_source_detail(st), "recording")

    def test_snapshot_and_hello_carry_detail(self):
        task = gp.idle_task()
        def snap(state):
            return gp.compose_snapshot(state, None, None, None, "up", 0, task, None, False)
        self.assertEqual(snap({"source": "replay", "replay_file": "pc.fpgarec"})["sourceDetail"], "recording")
        self.assertEqual(snap({"source": "replay", "replay_id": "hub-replay-feeder"})["sourceDetail"], "synthetic")
        self.assertIsNone(snap({"source": "live"})["sourceDetail"])
        self.assertIn("sourceDetail", snap(None))
        h = gp.make_hello("b", None, "up", "replay", "controller", "recording")
        self.assertEqual(h["sourceDetail"], "recording")
        self.assertIsNone(gp.make_hello("b", None, "up", "replay", "controller", "weird")["sourceDetail"])
        self.assertIsNone(gp.make_hello("b", None, "up", "replay", "controller")["sourceDetail"])


class BridgeSourceDetailTests(unittest.TestCase):
    """真 Bridge.tick：hello 与快照随 sourceDetail 变化（复用 test_geomem_bridge 的假客户端）。"""

    def test_hello_resent_when_detail_changes(self):
        import test_geomem_bridge as tb
        b = tb.gb.Bridge(map_id=None, map_name=None, clock=tb.Clock(), bridge_id="bid").attach(tb.FakeClient())
        b.on_link("up", {})
        s = tb.FakeSession()
        b.register(s)
        s.take()
        b.on_frame(tb.FakeFrame("STATE"), tb.loc_state(source="replay", replay_file="pc.fpgarec"))
        snap = b.tick()
        self.assertEqual(snap["sourceDetail"], "recording")
        hellos = [v for k, v in s.take() if k == "hello"]
        self.assertEqual([h["sourceDetail"] for h in hellos], ["recording"])
        b.on_frame(tb.FakeFrame("STATE"), tb.loc_state(source="replay", replay_id="hub-replay-feeder"))
        self.assertEqual(b.tick()["sourceDetail"], "synthetic")
        self.assertEqual([v["sourceDetail"] for k, v in s.take() if k == "hello"], ["synthetic"])
        b.tick()
        self.assertEqual([k for k, _ in s.take()], ["snapshot"])          # unchanged: no extra hello
        s2 = tb.FakeSession()
        b.register(s2)                                                      # late joiner gets current detail
        self.assertEqual(s2.events[0][1]["sourceDetail"], "synthetic")


class TestbedGuardTests(unittest.TestCase):
    def run_entry(self, name, args):
        env = dict(os.environ)
        env.pop("GEOMEM_TB_NETNS", None)
        env["PYTHONPATH"] = str(ROOT)
        return subprocess.run([sys.executable, str(TESTBED / name)] + args, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, universal_newlines=True, timeout=60)

    def test_fixture_refuses_outside_netns(self):
        r = self.run_entry("t2_hub_fixture.py", ["--token-file", os.devnull, "--port", "0", "--ctl-port", "0"])
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("隔离网络命名空间", r.stderr)

    def test_replay_refuses_outside_netns(self):
        r = self.run_entry("t1_replay.py", [os.devnull, os.devnull, "--port", "0"])
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("隔离网络命名空间", r.stderr)

    def test_state_tap_refuses_outside_netns(self):
        r = self.run_entry("t3_state_tap.py", ["127.0.0.1:1", os.devnull, os.devnull])
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("网络命名空间", r.stderr)

    def test_state_tap_row_shape(self):
        sys.path.insert(0, str(TESTBED))
        import t3_state_tap as tap
        row = tap.row_of({"seq_state": 3, "task": {"state": "ambiguous", "target_id": None, "reason_task": "x"},
                          "nav": {"motion_permit": False}, "memory": {"n_tracks": 2}}, False)
        self.assertEqual((row["seq"], row["task"], row["permit"], row["n_tracks"], row["stale"]),
                         (3, "ambiguous", False, 2, False))
        self.assertIsNone(tap.row_of({}, True)["permit"])


if __name__ == "__main__":
    unittest.main()
