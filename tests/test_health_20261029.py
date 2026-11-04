"""Car stack health check (2026-10-29, owner: report before the mapping what will make the run fail; warn only).

host/hub/health.py (checks, levels, hints, log files) <- HubCore.health_inputs (hub) -> STATE "health" -> bridge
snapshot "health" -> console panel #car-health (operator/src/health-panel.js). Fake inputs and a fake clock; no ROS.
Python 3.8.
"""
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

from host.hub import health as H   # noqa: E402
from host.hub.hub_core import HubCore   # noqa: E402

GC_FIT = {"verdict": "adopt", "adopted": {"height_m": 0.392, "pitch_deg": 15.75, "roll_deg": 0.87, "source": "fit"},
          "fitted": {"height_m": 0.392, "pitch_deg": 15.75, "roll_deg": 0.87}, "reasons": []}
# 10-28 14:19 (the failing run): a clean 5.7 / 5.8 deg fit refused, the configured 7 / 0 deg used
GC_REFUSED = {"verdict": "adopt", "adopted": {"height_m": 0.39, "pitch_deg": 7.0, "roll_deg": 0.0,
                                              "source": "config_fallback"},
              "fitted": {"height_m": 0.389, "pitch_deg": 5.7, "roll_deg": 5.8},
              "alert": "相机角度测量异常，已用默认安装值，建议检查相机", "warnings": ["横滚 5.8° 超出 ±5°"]}
CAR = {"camera": "live", "pose": "odom", "chassis": "uart", "camera_mount": {"ground_calib": {"mode": "auto"}}}
FACTS = {"power_mode": "MAXN_SUPER", "kv_wall_minus_nano_s": 0.2, "kv_clock_rtt_s": 0.3,
         "kv_ready": {"ar": "drm", "ar_screen": "sink_count=1 edid_bytes=256"}}
CLASSES = ["backpack", "bottle", "cup", "banana", "laptop"]   # owner 10-31: the five demo items (medicine box dropped)


class Clock(object):
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Feed(object):
    """A healthy car stack's hub inputs, advanced one second at a time."""

    def __init__(self):
        self.n = 0
        self.seq = 0
        self.enc = [0, 0, 0, 0]
        self.cmd = [0, 0, True]
        self.last_cmd_seq = 0
        self.ack = True
        self.link_alive = True
        self.permit = True
        self.link_ok = True
        self.estop = False
        self.depth_invalid_s = None
        self.ages = {"frame_stats": 0.1, "preview": 0.1, "runtime_info": 30.0, "status_frame": 0.2, "odom": 0.02,
                     "telemetry": 0.1}
        self.wheel = {"heading": "gyro", "heading_source": "gyro", "mcu_frame": "0x82", "gyro": {"ready": True}}
        self.hb = True

    def step(self, moved=0):
        self.n += 1
        self.seq += 2
        if self.ack:
            self.last_cmd_seq = (self.last_cmd_seq + 50) & 0xFF
        self.enc = [e + moved for e in self.enc]
        counts = {"frames_processed_seen": 6 * self.n, "cb:/fpga/preview/jpeg": 10 * self.n,
                  "cb:/fpga/kv_link/frame_stats": 5 * self.n, "cb:/odom": 50 * self.n,
                  "cb:/fpga/chassis/status": 2 * self.n}
        mcu = {"seq": self.seq & 0xFF, "mcu_ms": 1000 * self.n, "enc_a": self.enc[0], "enc_b": self.enc[1],
               "enc_c": self.enc[2], "enc_d": self.enc[3], "last_cmd_seq": self.last_cmd_seq, "battery_mv": 0,
               "heartbeat_seen": self.hb, "driver_fault": False}
        chassis = {"link_alive": self.link_alive, "status_age_ms": 10.0 if self.link_alive else 4000.0, "mcu": mcu,
                   "mcu_frame": "0x82", "cmd": list(self.cmd)}
        state = {"kv": {"link_ok": self.link_ok, "permit": {"avail": True, "permit": self.permit,
                                                           "estop": False}, "ar": {"avail": True, "mode": 1}},
                 "estop": {"latched": self.estop}, "perf": {"frames_per_s": 5.0}, "clock": {"error": None},
                 "nano": {"power_mode": None}}
        return {"state": state, "ages_s": dict(self.ages), "counts": counts, "chassis": chassis,
                "chassis_recv_ns": self.n, "chassis_age_s": 0.2, "wheel_odom": dict(self.wheel),
                "depth_invalid_s": self.depth_invalid_s}


class Base(unittest.TestCase):
    def setUp(self):
        self.run = tempfile.mkdtemp(prefix="health_")
        self.addCleanup(shutil.rmtree, self.run, True)
        self.clock = Clock()
        self.out = io.StringIO()
        self.write("car.json", CAR)
        self.write("ground_calib.json", GC_FIT)
        self.write("host_facts.json", FACTS)
        self.write("inference.log", '[TRT] warning\n' + json.dumps({"ready": True, "socket": "/x", "device": "cuda",
                                                                    "classes": CLASSES, "profile": "combined33"}) + "\n")
        self.write("camera_rate_check.json", {"rate": {"depth_median_ms": 33.4, "colour_median_ms": 33.4,
                                                       "verdict": "normal"}})
        sysfs = os.path.join(self.run, "sys", "class", "hwmon", "hwmon1")
        os.makedirs(sysfs)
        for name, val in (("name", "ina3221"), ("in1_input", "4960")):
            with open(os.path.join(sysfs, name), "w") as f:
                f.write(val + "\n")
        self.mon = H.HealthMonitor(self.run, sysfs_root=os.path.join(self.run, "sys"), mono=self.clock,
                                   wall=lambda: 1792000000.0, out=self.out)
        self.feed = Feed()

    def write(self, name, obj):
        with open(os.path.join(self.run, name), "w", encoding="utf-8") as f:
            f.write(obj if isinstance(obj, str) else json.dumps(obj, ensure_ascii=False))

    def ready(self):
        self.write("READY", "2026-10-29T10:00:00\n")

    def steps(self, n=1, moved=0):
        r = None
        for _ in range(n):
            self.clock.t += 1.0
            r = self.mon.step(self.feed.step(moved))
        return r

    def check(self, r, code):
        found = [c for c in r["checks"] if c["code"] == code]
        return found[0] if found else None


class TestLevels(Base):
    def test_pending_before_ready_then_green(self):
        r = self.steps(3)
        self.assertEqual(r["overall"], "pending")
        self.assertEqual(r["checks"], [])
        self.assertFalse(os.path.exists(os.path.join(self.run, "health.jsonl")))
        self.ready()
        r = self.steps(3)
        self.assertEqual(r["overall"], "green", json.dumps(r, ensure_ascii=False, indent=1))
        self.assertEqual(r["summary"], "体检通过，可以开始建图")
        codes = [c["code"] for c in r["checks"]]
        for code in ("ground_calib", "chassis", "camera", "detector", "kv", "ar", "clock", "gyro", "odom",
                     "power_mode", "disk", "power"):
            self.assertIn(code, codes)
        self.assertTrue(all("hint" not in c for c in r["checks"]))     # green rows carry no hint
        self.assertIn("下倾 15.8°", self.check(r, "ground_calib")["detail"])
        self.assertIn("5 类", self.check(r, "detector")["detail"])
        self.assertIn("Nano 输入 4.96 V", self.check(r, "power")["detail"])
        json.dumps(r)

    def test_ground_calib_refused_is_red_with_hint(self):
        self.write("ground_calib.json", GC_REFUSED)
        self.ready()
        r = self.steps(2)
        self.assertEqual(r["overall"], "red")
        c = r["checks"][0]
        self.assertEqual((c["code"], c["level"]), ("ground_calib", "red"))
        self.assertIn("横滚 5.8°", c["detail"])
        self.assertIn("默认安装值", c["detail"])
        self.assertEqual(c["hint"], "检查相机支架是否被碰歪，扶正拧紧后重新起栈")
        self.assertIn("地面标定", r["summary"])

    def test_ground_calib_variants(self):
        last = dict(GC_REFUSED, adopted=dict(GC_REFUSED["adopted"], source="last_good"))
        self.assertIn("上次标定值", H.ground_calib_check(last)["detail"])
        self.assertEqual(H.ground_calib_check(last)["level"], "red")
        no_floor = {"verdict": "reject", "reasons": ["地面内点太少"]}
        c = H.ground_calib_check(no_floor)
        self.assertEqual(c["level"], "red")
        self.assertIn("地面内点太少", c["detail"])
        self.assertEqual(H.ground_calib_check(None)["level"], "yellow")
        confirm = dict(GC_FIT, verdict="confirm", adopted=dict(GC_FIT["adopted"], source="config"))
        self.assertEqual(H.ground_calib_check(confirm)["level"], "green")

    def test_chassis_link_lost_turns_red_and_recovers(self):
        self.ready()
        self.steps(2)
        self.feed.link_alive = False
        r = self.steps(1)
        c = self.check(r, "chassis")
        self.assertEqual(c["level"], "red")
        self.assertIn("没发状态帧", c["detail"])
        self.feed.link_alive = True
        r = self.steps(1)
        self.assertEqual(self.check(r, "chassis")["level"], "green")

    def test_remote_control_on_mcu_ignores_commands(self):
        """10-26 dev2: with the stock remote on, the MCU does not execute serial commands. Two signs: it stops
        acknowledging command frames (last_cmd_seq frozen), or it is commanded to move and the wheels stay still."""
        self.ready()
        self.steps(2)
        self.feed.ack = False
        r = self.steps(3)
        c = self.check(r, "chassis")
        self.assertEqual(c["level"], "red")
        self.assertIn("没接收", c["detail"])
        self.assertIn("关掉遥控器", c["hint"])
        self.feed.ack = True
        self.assertEqual(self.check(self.steps(1), "chassis")["level"], "green")
        self.feed.cmd = [100, 0, True]                   # 0.1 m/s forward, the wheels do not move
        r = self.steps(3, moved=0)
        c = self.check(r, "chassis")
        self.assertEqual(c["level"], "red")
        self.assertIn("轮子没转", c["detail"])
        self.assertIn("关掉遥控器", c["hint"])
        r = self.steps(3, moved=400)                     # wheels turn: green again
        self.assertEqual(self.check(r, "chassis")["level"], "green")
        self.feed.cmd = [0, 100, True]                   # below the turn threshold: never judged
        self.assertEqual(self.check(self.steps(4, moved=0), "chassis")["level"], "green")

    def test_commanded_still_while_mcu_stop_latched_is_not_blamed_on_the_remote(self):
        self.ready()
        self.steps(2)
        self.feed.cmd = [100, 0, True]
        inp = None
        for _ in range(3):
            self.clock.t += 1.0
            inp = self.feed.step(0)
            inp["chassis"]["mcu"]["stop_latched"] = True
            r = self.mon.step(inp)
        c = self.check(r, "chassis")
        self.assertEqual(c["level"], "red")
        self.assertIn("停机锁存", c["detail"])
        self.assertNotIn("遥控器", c["hint"])

    def test_chassis_heartbeat_missing(self):
        self.ready()
        self.feed.hb = False
        r = self.steps(4)
        c = self.check(r, "chassis")
        self.assertEqual(c["level"], "red")
        self.assertIn("心跳", c["detail"])

    def test_camera_depth_zero_and_stale(self):
        self.ready()
        self.feed.depth_invalid_s = 5.0
        c = self.check(self.steps(2), "camera")
        self.assertEqual(c["level"], "red")
        self.assertIn("全是 0", c["detail"])
        self.feed.depth_invalid_s = None
        self.feed.ages["frame_stats"] = 9.0
        c = self.check(self.steps(1), "camera")
        self.assertEqual(c["level"], "red")
        self.assertIn("深度帧断了", c["detail"])
        self.feed.ages["frame_stats"] = 0.1
        self.feed.ages["preview"] = 7.0
        c = self.check(self.steps(1), "camera")
        self.assertIn("彩色画面断了", c["detail"])
        self.feed.ages["preview"] = 0.1
        c = self.check(self.steps(1), "camera")
        self.assertEqual(c["level"], "green")
        self.assertIn("起栈测 彩色 30 / 深度 30 帧/秒", c["detail"])

    def test_detector(self):
        self.mon.expect_classes = ("bottle", "remote", "glasses")
        self.ready()
        c = self.check(self.steps(2), "detector")
        self.assertEqual(c["level"], "yellow")
        self.assertIn("不认识：遥控器、眼镜", c["detail"])
        self.feed.ages["status_frame"] = 12.0
        c = self.check(self.steps(1), "detector")
        self.assertEqual(c["level"], "red")
        del self.feed.ages["status_frame"]
        del self.feed.ages["runtime_info"]
        c = self.check(self.steps(1), "detector")
        self.assertIn("没有起来", c["detail"])

    def test_mapping_only_and_bench_runs_skip_checks(self):
        self.write("car.json", dict(CAR, pose="map", chassis="none"))
        self.ready()
        r = self.steps(2)
        codes = [c["code"] for c in r["checks"]]
        self.assertNotIn("detector", codes)
        self.assertNotIn("chassis", codes)

    def test_kv_link_permit_estop(self):
        self.ready()
        self.steps(1)
        self.feed.link_ok = False
        self.assertEqual(self.check(self.steps(1), "kv")["level"], "green")     # one lost sample: not yet
        c = self.check(self.steps(3), "kv")
        self.assertEqual(c["level"], "red")
        self.assertIn("遥测断了", c["detail"])
        self.feed.link_ok = True
        self.feed.permit = False
        self.steps(1)
        c = self.check(self.steps(3), "kv")
        self.assertEqual(c["level"], "red")
        self.assertIn("没有放行", c["detail"])
        self.feed.estop = True
        c = self.check(self.steps(1), "kv")
        self.assertEqual(c["level"], "yellow")
        self.assertIn("解除急停", c["hint"])

    def test_ar_clock_power_mode_gyro(self):
        self.write("host_facts.json", {"power_mode": "15W", "kv_wall_minus_nano_s": -3.8 * 86400,
                                       "kv_ready": {"ar": "drm", "ar_screen": "sink_count=0 edid_bytes=0"}})
        self.feed.wheel = {"heading": "gyro", "heading_source": "wheel", "mcu_frame": "0x81"}
        self.ready()
        r = self.steps(2)
        self.assertEqual(self.check(r, "ar")["level"], "red")
        self.assertIn("看不到屏幕", self.check(r, "ar")["detail"])
        c = self.check(r, "clock")
        self.assertEqual(c["level"], "yellow")
        self.assertIn("慢 3.8 天", c["detail"])
        self.assertIn("date -s", c["hint"])
        self.assertEqual(self.check(r, "power_mode")["level"], "yellow")
        self.assertIn("nvpmodel -m 2", self.check(r, "power_mode")["hint"])
        g = self.check(r, "gyro")
        self.assertEqual(g["level"], "yellow")
        self.assertIn("退回轮子", g["detail"])
        self.write("host_facts.json", dict(FACTS, kv_ready={"ar": "off", "ar_reason": "helper missing"}))
        r = self.steps(1)
        self.assertIn("车屏没开（helper missing）", self.check(r, "ar")["detail"])

    def test_disk_and_battery(self):
        self.ready()
        self.mon.disk_t = self.clock.t + 1e9            # keep the value set below
        self.mon.disk = 1.2
        self.feed_battery = 20000
        r = self.steps(1)
        self.assertEqual(self.check(r, "disk")["level"], "red")
        inp = self.feed.step()
        inp["chassis"]["mcu"]["battery_mv"] = 20000
        self.clock.t += 1
        r = self.mon.step(inp)
        self.assertEqual(self.check(r, "power")["level"], "red")
        self.assertIn("电池 20.0 V", self.check(r, "power")["detail"])

    def test_log_files(self):
        self.ready()
        self.steps(2)
        self.feed.link_alive = False
        self.steps(1)
        self.feed.link_alive = True
        self.steps(1)
        with open(os.path.join(self.run, "health.jsonl"), encoding="utf-8") as f:
            rows = [json.loads(l) for l in f]
        self.assertEqual(rows[0]["event"], "first_report")
        changes = [(r["code"], r["from"], r["to"]) for r in rows if r["event"] == "change"]
        self.assertEqual(changes, [("chassis", "green", "red"), ("chassis", "red", "green")])
        with open(os.path.join(self.run, "health.json"), encoding="utf-8") as f:
            snap = json.load(f)
        self.assertEqual(snap["overall"], "green")
        self.assertIn("体检变化：底盘串口 green -> red", self.out.getvalue())
        self.assertIn("体检 green", H.summary_line(snap))


class TestHubAndBridge(unittest.TestCase):
    def test_hub_inputs_and_state(self):
        core = HubCore("live")
        core.feed_chassis_status(json.dumps({"link_alive": True, "mcu": {"seq": 1}, "cmd": [0, 0, True]}), 1000)
        core.feed_wheel_odom_status(json.dumps({"heading": "gyro", "heading_source": "gyro"}), 1000)
        core.feed_chassis_status("{broken", 1000)
        self.assertEqual(core.counts["bad_input:chassis_status"], 1)
        s = core.compose_state(2 * 10**9)
        self.assertNotIn("health", s)
        inp = core.health_inputs(3 * 10**9)
        self.assertIs(inp["state"], core.last_state)
        self.assertAlmostEqual(inp["chassis_age_s"], 3.0 - 1e-6, places=3)
        self.assertEqual(inp["wheel_odom"]["heading_source"], "gyro")
        self.assertIn("cb:/fpga/chassis/status", inp["counts"])
        core.health = {"v": 1, "overall": "red", "summary": "x", "checks": []}
        self.assertEqual(core.compose_state(4 * 10**9)["health"]["overall"], "red")

    def test_bridge_compose_health(self):
        from ui.geomem.bridge.geomem_bridge import compose_health
        self.assertIsNone(compose_health(None))
        self.assertIsNone(compose_health({"overall": "bogus"}))
        h = compose_health({"overall": "red", "summary": "s" * 500, "checks": [
            {"code": "chassis", "level": "red", "title": "底盘串口", "detail": "d", "hint": "关掉遥控器"},
            {"code": "x", "level": "nope", "title": "t"}, "junk",
            {"code": "disk", "level": "green", "title": "磁盘余量", "detail": "剩 80 GB"}]})
        self.assertEqual(len(h["summary"]), 200)
        self.assertEqual([c["code"] for c in h["checks"]], ["chassis", "disk"])
        self.assertEqual(h["checks"][0]["hint"], "关掉遥控器")
        self.assertNotIn("hint", h["checks"][1])

    def test_bridge_snapshot_carries_health(self):
        import test_geomem_bridge as tgb
        t = tgb.KioskSnapshotTests("test_basemap_in_snapshot")
        t.setUp()
        try:
            st = tgb.loc_state()
            self.assertNotIn("health", t.tick(t.bridge(), st))
            st["health"] = {"overall": "yellow", "summary": "可以开始建图；1 项提醒：功率档", "checks": [
                {"code": "power_mode", "level": "yellow", "title": "功率档", "detail": "现在是 15W",
                 "hint": "sudo nvpmodel -m 2"}]}
            snap = t.tick(t.bridge(), st)
            self.assertEqual(snap["health"]["overall"], "yellow")
            self.assertEqual(snap["health"]["checks"][0]["title"], "功率档")
        finally:
            t.tearDown()


class TestWiring(unittest.TestCase):
    def read(self, *p):
        with open(os.path.join(ROOT, *p), encoding="utf-8") as f:
            return f.read()

    def test_served_classes(self):
        from host.inference_service import served_classes

        class D(object):
            def __init__(self, names, classes=None):
                self.names, self.classes = names, classes

        class C(object):
            detectors = (D(("a", "b", "c"), ("a", "b")), D(("b", "x")))
        self.assertEqual(served_classes(C()), ["a", "b", "x"])
        self.assertIsNone(served_classes(object()))

    def test_page_and_launch_wiring(self):
        html = self.read("ui", "geomem", "operator", "index.html")
        self.assertIn('id="car-health"', html)
        self.assertIn('id="car-notice"', html)                      # the 10-28 line stays (older hubs)
        for js in ("app.js", "remote.js"):
            self.assertIn("renderHealth(", self.read("ui", "geomem", "operator", "src", js))
        self.assertIn("./src/health-panel.js", self.read("ui", "geomem", "operator", "sw.js"))
        self.assertIn("start_health_thread(server, HealthMonitor(os.environ[\"O\"]))",
                      self.read("host", "hub", "hub_ros.py"))
        self.assertIn('"/fpga/chassis/status"', self.read("host", "hub", "hub_ros.py"))
        up = self.read("tools", "car_run", "car_up.sh")
        self.assertIn('"$RUN/host_facts.json"', up)
        self.assertIn("CAR_KV_READY_JSON", up)
        self.assertIn('"CAR_KV_READY_JSON=$kv_json"', self.read("ui", "geomem", "tools", "competition", "nano_up.sh"))
        self.assertIn("row['cmd']", self.read("host", "ros_chassis_driver.py"))

    @unittest.skipUnless(shutil.which("node"), "node not installed")
    def test_panel_markup_node(self):
        mod = "file://" + os.path.join(ROOT, "ui", "geomem", "operator", "src", "health-panel.js")
        script = ("import(%r).then(m=>{const h={overall:'red',summary:'有 1 项<b>',checks:["
                  "{code:'ground_calib',level:'red',title:'地面标定',detail:'拟合被拒',hint:'检查相机支架'},"
                  "{code:'disk',level:'green',title:'磁盘余量',detail:'剩 80 GB'}]};"
                  "console.log(m.healthHtml(h));});" % mod)
        out = subprocess.run(["node", "--input-type=module", "-e", script], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)
        html = out.stdout
        self.assertIn("有 1 项&lt;b&gt;", html)                  # escaped
        self.assertIn('class="health-item red"', html)
        self.assertIn("处理：检查相机支架", html)
        self.assertIn("1 项需处理", html)
        self.assertEqual(html.count("health-hint"), 1)


if __name__ == "__main__":
    unittest.main()
