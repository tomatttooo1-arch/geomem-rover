"""Known-answer tests: tools/car_run/accept_readout.py, tools/odom_gyro/turn_check.py, tools/car_run/backpack_gate.py."""
import importlib.util
import io
import json
import math
import os
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_LIVE = "${FPGA_SRC}/runs/realcar-nav-20261017/board_1017_1654/nano/runs/rn_1017b"
REAL_SIM = "${FPGA_SRC}/runs/car-runs/mempersist-e2-20261018"


def load(rel, name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, rel))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


AR = load("tools/car_run/accept_readout.py", "accept_readout")
TC = load("tools/odom_gyro/turn_check.py", "turn_check")
BG = load("tools/car_run/backpack_gate.py", "backpack_gate")

MARKS = ("1.0 start camera=live pose=odom nav=nav2 head=abc\n"
         "2.0 ready wheel_odom tf=/tf heading=gyro\n"
         "3.0 ready nav2 costmap=rolling overrides=dwb.yaml\n"
         "4.0 READY\n")


class Run(object):
    def __init__(self):
        self.d = tempfile.mkdtemp(prefix="accept_")

    def write(self, name, text):
        p = os.path.join(self.d, name)
        if not os.path.isdir(os.path.dirname(p)):
            os.makedirs(os.path.dirname(p))
        with open(p, "w") as f:
            f.write(text)
        return p

    def jsonl(self, name, rows):
        return self.write(name, "".join(json.dumps(r) + "\n" for r in rows))

    def sec(self, idx, **kw):
        return AR.readout(self.d, **kw)["sections"][idx]

    def close(self):
        shutil.rmtree(self.d, ignore_errors=True)


def ev(event, t, **kw):
    d = {"event": event, "time": t, "pose_source": "reference", "clock_source": "system_ros"}
    d.update(kw)
    return d


class Base(unittest.TestCase):
    def setUp(self):
        self.r = Run()

    def tearDown(self):
        self.r.close()


class TestVersion(Base):
    def test_nodata_empty(self):
        self.assertEqual(self.r.sec(0)["verdict"], "NODATA")

    def test_pass_and_budgets(self):
        self.r.write("head.txt", "abc\n")
        self.r.write("dirty.txt", "")
        self.r.write("marks.txt", MARKS)
        self.r.jsonl("runtime.jsonl", [ev("startup", 0, navigation_timeout_s=60, task_timeout_s=180,
                                          nav_retries=2, max_attempts=3)])
        s = self.r.sec(0)
        self.assertEqual(s["verdict"], "PASS")
        self.assertEqual(s["evidence"]["navigation_timeout_s"], 60)
        self.assertEqual(s["evidence"]["task_timeout_s"], 180)
        self.assertEqual(s["evidence"]["heading"], "gyro")
        self.assertEqual(s["evidence"]["retry_params"], {"nav_retries": 2, "max_attempts": 3})

    def test_old_budgets_nodata_text(self):
        self.r.write("head.txt", "abc\n")
        self.r.write("marks.txt", MARKS)
        self.r.jsonl("runtime.jsonl", [ev("startup", 0)])
        s = self.r.sec(0)
        self.assertIn("旧版本", s["evidence"]["navigation_timeout_s"])

    def test_fail_dirty(self):
        self.r.write("head.txt", "abc\n")
        self.r.write("dirty.txt", " M host/x.py\n")
        self.r.write("marks.txt", MARKS)
        self.assertEqual(self.r.sec(0)["verdict"], "FAIL")


ROOT_REASON = ("root check failed: the first graph node 1's correction is %.4f m / 0.0100 rad, not identity "
               "(tol 0.01 m / 0.01 rad)")


class TestAlign(Base):
    def test_nodata_not_mapping(self):
        self.r.write("marks.txt", MARKS)
        self.assertEqual(self.r.sec(1)["verdict"], "NODATA")

    def test_fail_mapping_without_status(self):
        self.r.write("marks.txt", MARKS + "5.0 ready mapping rtabmap\n")
        self.assertEqual(self.r.sec(1)["verdict"], "FAIL")

    def test_fail_red_camera_frame(self):
        self.r.write("mapping_status.json", json.dumps({"state": "stopped", "align": {"ok": False,
                                                                                    "reason": ROOT_REASON % 0.3912}}))
        s = self.r.sec(1)
        self.assertEqual(s["verdict"], "FAIL")
        self.assertAlmostEqual(s["evidence"]["root_offset_m"], 0.3912)
        self.assertTrue(any("相机系" in x for x in s["red"]))

    def test_fail_small_offset_no_red(self):
        self.r.write("mapping_status.json", json.dumps({"state": "stopped", "align": {"ok": False,
                                                                                    "reason": ROOT_REASON % 0.05}}))
        s = self.r.sec(1)
        self.assertEqual(s["verdict"], "FAIL")
        self.assertFalse(any("相机系" in x for x in s["red"]))

    def test_red_threshold_edge(self):
        self.r.write("mapping_status.json", json.dumps({"align": {"ok": False, "reason": ROOT_REASON % 0.2999}}))
        self.assertFalse(any("相机系" in x for x in self.r.sec(1)["red"]))
        self.r.write("mapping_status.json", json.dumps({"align": {"ok": False, "reason": ROOT_REASON % 0.3000}}))
        self.assertTrue(any("相机系" in x for x in self.r.sec(1)["red"]))

    def test_pass_ok(self):
        self.r.write("mapping_status.json", json.dumps({"state": "stopped", "align": {
            "ok": True, "identity": False, "map_odom": [0.1, 0.0, 0.02], "count": 57,
            "root": {"node_id": 1, "offset_m": 0.002, "offset_rad": 0.001}, "unmatched": 0, "applied": True}}))
        s = self.r.sec(1)
        self.assertEqual(s["verdict"], "PASS")
        self.assertEqual(s["evidence"]["node_count"], 57)

    def test_nodata_align_null(self):
        self.r.write("mapping_status.json", json.dumps({"state": "mapping", "align": None}))
        self.assertEqual(self.r.sec(1)["verdict"], "NODATA")


class TestFind(Base):
    def test_nodata(self):
        self.r.jsonl("runtime.jsonl", [ev("startup", 0)])
        self.assertEqual(self.r.sec(2)["verdict"], "NODATA")

    def test_pass_with_goal_attempts_leg_before_task(self):
        task = dict(state="navigating", reason="reobserve", target_id="object-0001", candidates=[],
                    processed_frames=10, dropped_frames=1, goal="red backpack", attempts=1)
        self.r.jsonl("runtime.jsonl", [
            ev("startup", 0),
            ev("navigation_started", 100.0, command_id="ab:1"),   # leg logged just before the first task row
            ev("task", 100.0, **task), ev("task", 100.0, **task),  # duplicate row
            ev("navigation_result", 112.0, command_id="ab:1", succeeded=True, cancelled=False),
            ev("task", 115.0, **dict(task, state="found", reason="fresh_matching_observation", attempts=2)),
        ])
        s = self.r.sec(2)
        self.assertEqual(s["verdict"], "PASS")
        t = s["evidence"]["tasks"]
        self.assertEqual(len(t), 1)
        self.assertEqual(t[0]["goal"], "red backpack")
        self.assertEqual(t[0]["attempts"], 2)
        self.assertEqual(t[0]["duration_s"], 15.0)
        self.assertEqual(t[0]["nav_legs"], [{"command_id": "ab:1", "result": "succeeded", "s": 12.0}])

    def test_fail_second_task_timeout(self):
        base = dict(target_id="o1", candidates=[{"id": "o1", "label": "backpack"}], processed_frames=1,
                    dropped_frames=0)
        self.r.jsonl("runtime.jsonl", [
            ev("task", 1.0, state="searching", reason="x", **base),
            ev("task", 5.0, state="found", reason="y", **base),
            ev("navigation_started", 10.0, command_id="cd:1"),
            ev("task", 10.0, state="navigating", reason="x", **base),
            ev("fault", 70.0, reason="navigation_wall_timeout"),
            ev("navigation_result", 70.5, command_id="cd:1", succeeded=False, cancelled=True),
            ev("task", 70.5, state="failed", reason="navigation_wall_timeout", **base),
        ])
        s = self.r.sec(2)
        self.assertEqual(s["verdict"], "FAIL")
        t = s["evidence"]["tasks"]
        self.assertEqual([x["final_state"] for x in t], ["found", "failed"])
        self.assertEqual(t[1]["faults"], ["navigation_wall_timeout"])
        self.assertEqual(t[1]["goal"], "无数据（旧版本）")
        self.assertEqual(t[1]["candidate_labels"], ["backpack"])

    def test_fail_unfinished(self):
        self.r.jsonl("runtime.jsonl", [ev("task", 1.0, state="searching", reason="x")])
        s = self.r.sec(2)
        self.assertEqual(s["verdict"], "FAIL")
        self.assertIn("未结束", s["evidence"]["tasks"][0]["final_state"])


def nav_line(**extra):
    d = {"real_car": {"speed_limits": "car"}, "speed_limits": {"linear": 0.15}}
    d.update(extra)
    return json.dumps(d) + "\n"


class TestNav(Base):
    def test_nodata_sim(self):
        self.r.write("navigation.log", '{"input_profile": "reference"}\n')
        s = self.r.sec(3)
        self.assertEqual(s["verdict"], "NODATA")
        self.assertIn("仿真", s["notes"][0])

    def test_nodata_old(self):
        self.r.write("navigation.log", nav_line())
        s = self.r.sec(3)
        self.assertEqual(s["verdict"], "NODATA")
        self.assertIn("旧版本", s["notes"][0])

    def test_fail_shim_null(self):
        self.r.write("navigation.log", nav_line(rotation_shim=None, yaw_goal_tolerance=0.10))
        self.assertEqual(self.r.sec(3)["verdict"], "FAIL")

    def test_fail_tol(self):
        self.r.write("navigation.log", nav_line(rotation_shim={"a": 1}, yaw_goal_tolerance=0.25))
        self.assertEqual(self.r.sec(3)["verdict"], "FAIL")

    def test_pass_and_progress_count(self):
        self.r.write("navigation.log", "[controller] Failed to make progress\n"
                     + nav_line(rotation_shim={"plugin": "x"}, yaw_goal_tolerance=0.10)
                     + "[bt] Failed to make progress\n")
        s = self.r.sec(3)
        self.assertEqual(s["verdict"], "PASS")
        self.assertEqual(s["evidence"]["nav2_failed_to_make_progress"], 2)


def frames(results, step_ms=100):
    rows = [{"event": "slow_call", "raw_ns": 1}]
    for i, r in enumerate(results):
        rows.append({"frame_id": i, "acquired_ns": int(i * step_ms * 1e6), "result": r})
    return rows


class TestDepth(Base):
    def test_nodata(self):
        self.assertEqual(self.r.sec(4)["verdict"], "NODATA")

    def test_pass_short_runs(self):
        self.r.jsonl("frames.jsonl", frames(["fused"] * 5 + ["dropped:no_valid_depth"] * 20 + ["fused"] * 5))
        s = self.r.sec(4)
        self.assertEqual(s["verdict"], "PASS")
        self.assertAlmostEqual(s["evidence"]["longest_no_valid_depth_s"], 1.9)

    def test_fail_long_run(self):
        self.r.jsonl("frames.jsonl", frames(["fused"] + ["dropped:no_valid_depth"] * 31 + ["fused"]))
        s = self.r.sec(4)
        self.assertEqual(s["verdict"], "FAIL")
        self.assertAlmostEqual(s["evidence"]["longest_no_valid_depth_s"], 3.0)


class TestDetect(Base):
    def test_nodata_no_rejected(self):
        self.r.jsonl("runtime.jsonl", [ev("frame_processed", 1.0)] * 3)
        s = self.r.sec(5)
        self.assertEqual(s["verdict"], "NODATA")
        self.assertEqual(s["evidence"]["frame_processed_events"], 3)

    def test_pass_task_counts(self):
        self.r.jsonl("runtime.jsonl", [ev("task", 1.0, state="found", processed_frames=90, dropped_frames=10)])
        s = self.r.sec(5)
        self.assertEqual(s["verdict"], "PASS")
        self.assertEqual(s["evidence"]["drop_ratio"], 0.1)

    def test_fail_rejected_events(self):
        self.r.jsonl("runtime.jsonl", [ev("frame_processed", 1.0)] * 7 + [ev("frame_rejected", 1.0)] * 3)
        s = self.r.sec(5)
        self.assertEqual(s["verdict"], "FAIL")
        self.assertEqual(s["evidence"]["drop_ratio"], 0.3)


TEG = ("10-26-2026 10:00:00 RAM 3000/7620MB CPU [40%@1510,50%@1510,60%@1510,30%@1510,off,off] "
       "GR3D_FREQ 70%@[624] cpu@55.5C gpu@54C tj@%sC VDD_IN 9000mW/9000mW\n")
PIDSTAT = ("Linux 5.15 (nano)  10/26/2026  _aarch64_  (6 CPU)\n\n"
           "10:00:01 AM   UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command\n"
           "10:00:06 AM  1000      1234   80.00    5.00    0.00    0.00   85.00     2  python3\n"
           "10:00:06 AM  1000      2222   10.00    0.00    0.00    0.00   10.00     1  rtabmap\n"
           "10:00:11 AM  1000      1234   70.00    5.00    0.00    0.00   75.00     2  python3\n\n"
           "Average:     1000      1234   75.00    5.00    0.00    0.00   80.00     -  python3\n")


class TestLoad(Base):
    def test_nodata(self):
        self.assertEqual(self.r.sec(6)["verdict"], "NODATA")

    def test_pass_and_pidstat(self):
        self.r.write("accept/tegrastats.txt", TEG.replace("%s", "60.0") * 3)
        self.r.write("accept/pidstat.txt", PIDSTAT)
        self.r.write("accept/nvpmodel.txt", "NV Power Mode: MAXN_SUPER\n2\n")
        s = self.r.sec(6)
        self.assertEqual(s["verdict"], "PASS")
        self.assertEqual(s["evidence"]["cpu_avg_pct"], 45.0)
        self.assertEqual(s["evidence"]["gpu_avg_pct"], 70.0)
        self.assertEqual(s["evidence"]["vdd_in_avg_mw"], 9000)
        self.assertEqual(s["evidence"]["temp_max_c"], 60.0)
        self.assertEqual(s["evidence"]["pidstat_top_cpu"][0], ("python3", 80.0))
        self.assertEqual(s["evidence"]["nvpmodel"], "MAXN_SUPER")

    def test_fail_temp(self):
        self.r.write("accept/tegrastats.txt", TEG.replace("%s", "91.0"))
        self.assertEqual(self.r.sec(6)["verdict"], "FAIL")

    def test_nvpmodel_expect(self):
        self.r.write("nvpmodel.txt", "NV Power Mode: 15W\n")
        self.assertEqual(self.r.sec(6)["verdict"], "NODATA")       # only reported without --expect
        self.assertEqual(self.r.sec(6, expect_nvpmodel="MAXN_SUPER")["verdict"], "FAIL")
        self.assertEqual(self.r.sec(6, expect_nvpmodel="15W")["verdict"], "PASS")

    def test_cli_default_expects_maxn_super(self):
        """owner 10-26: without --expect-nvpmodel the CLI judges against MAXN_SUPER; '' = report only"""
        self.r.write("nvpmodel.txt", "NV Power Mode: 15W\n")

        def load(*extra):
            buf = io.StringIO()
            with redirect_stdout(buf):
                self.assertEqual(AR.main([self.r.d, "--json", "-"] + list(extra)), 0)
            return json.loads(buf.getvalue())["sections"][6]
        s = load()
        self.assertEqual(s["verdict"], "FAIL")
        self.assertIn("nvpmodel=15W，应 MAXN_SUPER", " ".join(map(str, s.get("red", s.get("notes", [])))) + json.dumps(s, ensure_ascii=False))
        self.assertEqual(load("--expect-nvpmodel", "")["verdict"], "NODATA")
        self.assertEqual(load("--expect-nvpmodel", "15W")["verdict"], "PASS")
        self.r.write("nvpmodel.txt", "NV Power Mode: MAXN_SUPER\n")
        self.assertEqual(load()["verdict"], "PASS")


WS = ("gateway in hello: {}\n"
      "kind              n    size_med   size_max  chunks_max   gap_med  gap_p95  gap_max  gaps>0.5s  gaps>2s\n"
      "mapping            12      40000     900000           3    2.400    2.600    2.900        12        2\n"
      "snapshot          300       5000       9000           -    0.200    0.300    %s        3        0\n")


class TestFine(Base):
    def test_nodata(self):
        s = self.r.sec(7)
        self.assertEqual(s["verdict"], "NODATA")

    def test_pass(self):
        self.r.jsonl("accept/live_mapping_status.jsonl", [{"voxel": {"ms": 520}}, {"voxel": {"ms": 610}}, {"x": 1}])
        self.r.write("accept/ws_probe.txt", WS % "1.100")
        self.r.jsonl("frames.jsonl", frames(["fused"] * 3))
        self.r.write("accept/bridge-stats-run1.json", json.dumps({"rejects": {}, "snapshot_issues_max": {},
                                                                  "cloud_rev": 4}))
        s = self.r.sec(7)
        self.assertEqual(s["verdict"], "PASS")
        self.assertEqual(s["evidence"]["voxel_ms"]["max"], 610)
        self.assertEqual(s["evidence"]["snapshot"]["gap_max_s"], 1.1)
        self.assertEqual(s["evidence"]["bridge_stats"]["cloud_rev"], 4)

    def test_fail_voxel_single_object(self):
        self.r.write("accept/live_mapping_status.json", json.dumps({"voxel": {"ms": 900}}))
        self.r.write("accept/ws_probe.txt", WS % "1.100")
        self.r.jsonl("frames.jsonl", frames(["fused"] * 3))
        self.assertEqual(self.r.sec(7)["verdict"], "FAIL")

    def test_fail_snapshot_gap(self):
        self.r.write("accept/ws_probe.txt", WS % "2.500")
        s = self.r.sec(7)
        self.assertEqual(s["verdict"], "FAIL")
        self.assertEqual(s["evidence"]["items"]["snapshot_gap"], "FAIL")


class TestCli(Base):
    def test_json_inside_run_refused_and_read_only(self):
        self.r.write("head.txt", "abc\n")
        before = sorted(os.listdir(self.r.d))
        with redirect_stdout(io.StringIO()):
            rc = AR.main([self.r.d, "--json", os.path.join(self.r.d, "out.json")])
        self.assertEqual(rc, 2)
        out = tempfile.mktemp(suffix=".json")
        try:
            buf = io.StringIO()
            with redirect_stdout(buf):
                self.assertEqual(AR.main([self.r.d, "--json", out]), 0)
            self.assertIn("版本与配置", buf.getvalue())
            with open(out) as f:
                rep = json.load(f)
            self.assertEqual(len(rep["sections"]), 8)
            self.assertIn("task.goal", rep["fields_assumed"])
        finally:
            if os.path.exists(out):
                os.remove(out)
        self.assertEqual(sorted(os.listdir(self.r.d)), before)


class TestRealDirs(unittest.TestCase):
    def test_live_1017b(self):
        if not os.path.isdir(REAL_LIVE):
            self.skipTest("no rn_1017b on this host")
        rep = AR.readout(REAL_LIVE)
        v = dict((s["name"], s["verdict"]) for s in rep["sections"])
        self.assertEqual(v["版本与配置"], "PASS")
        self.assertEqual(v["建图对齐"], "NODATA")
        self.assertEqual(v["寻物任务"], "NODATA")
        self.assertEqual(v["导航配置"], "NODATA")       # old version: no rotation_shim key
        self.assertEqual(v["深度"], "PASS")
        self.assertEqual(v["识别"], "NODATA")           # no frame_rejected -> no false PASS
        depth = [s for s in rep["sections"] if s["name"] == "深度"][0]
        self.assertEqual(depth["evidence"]["results"]["fused"], 7459)

    def test_sim_e2(self):
        if not os.path.isdir(REAL_SIM):
            self.skipTest("no mempersist-e2 on this host")
        rep = AR.readout(REAL_SIM)
        s = dict((x["name"], x) for x in rep["sections"])
        self.assertEqual(s["寻物任务"]["verdict"], "PASS")
        t = s["寻物任务"]["evidence"]["tasks"]
        self.assertEqual(len(t), 1)
        self.assertEqual(t[0]["final_state"], "found")
        self.assertEqual(len(t[0]["nav_legs"]), 1)
        self.assertEqual(s["识别"]["evidence"]["dropped"], 313)
        self.assertIn("仿真", s["导航配置"]["notes"][0])


class TestTurnCheck(Base):
    def test_wheel_odom_three_turns(self):
        rows = []
        for i in range(301):          # 30 s, gyro 3 turns, wheel 3 turns * 1.45
            a = 3 * 2 * math.pi * i / 300.
            rows.append({"t_ns": int(i * 1e8), "yaw": a * 1.45, "gyro_yaw": a,
                         "heading_source": "gyro" if i % 10 else "wheel", "fallback": [] if i % 10 else ["gyro_gap"]})
        self.r.jsonl("wheel_odom.jsonl", rows)
        r = TC.check(self.r.d, 3)
        self.assertAlmostEqual(r["gyro_ratio"], 1.0, places=3)
        self.assertAlmostEqual(r["wheel_ratio"], 1.45, places=3)
        self.assertAlmostEqual(r["gyro_over_wheel"], 1 / 1.45, places=3)
        self.assertAlmostEqual(r["heading_source_gyro_frac"], 270 / 301., places=3)

    def test_chassis_0x81_wrap(self):
        rows = []
        # 360 deg/s = 36000 cdps for 3 s, 10 ms frames, mcu_ms starts near the 16-bit wrap
        for i in range(301):
            rows.append({"dir": "rx", "seq": i % 256, "mcu_ms": (64000 + 10 * i) % 65536,
                         "gyro_z_cdps": 36000, "gyro_bias_cdps": 0, "gyro_valid": True})
            if i == 50:
                rows.append(dict(rows[-1]))          # duplicate frame
        rows.append({"dir": "tx", "seq": 1})
        self.r.jsonl("chassis_record.jsonl", rows)
        r = TC.check(self.r.d, 3)
        self.assertEqual(r["source"], "chassis_record.jsonl rx（0x81）")
        self.assertAlmostEqual(r["gyro_rad"], 3 * 2 * math.pi, places=3)

    def test_nodata(self):
        self.assertEqual(TC.check(self.r.d, 3)["verdict"], "NODATA")


def track(tid, label, scores):
    return {"id": tid, "label": label, "state": "current_confirmed", "position_m": [0, 0, 0],
            "history_len": len(scores) + 5, "history": [{"score": s} for s in scores]}


class TestBackpack(Base):
    def test_separable(self):
        self.r.write("memory_final.json", json.dumps({"tracks": [
            track("a", "backpack", [0.6, 0.7, 0.8]), track("b", "backpack", [0.3, 0.35, 0.4]),
            track("c", "bottle", [0.9])]}))
        r = BG.gate(self.r.d, truth={"a": True, "b": False})
        self.assertEqual(len(r["tracks"]), 2)
        self.assertEqual(r["separating_gate"], [0.35, 0.7])
        self.assertEqual(r["verdict"], "PASS")
        self.assertEqual(r["current_gate_keeps"], ["a"])
        self.assertEqual(r["current_gate_drops"], ["b"])

    def test_not_separable_snapshot_fallback(self):
        self.r.write("memory_snapshot.json", json.dumps({"tracks": [
            track("a", "backpack", [0.34, 0.38]), track("b", "backpack", [0.36, 0.40])]}))
        r = BG.gate(self.r.d, truth={"a": True, "b": False})
        self.assertEqual(r["verdict"], "FAIL")
        self.assertIsNone(r["separating_gate"])
        self.assertTrue(r["file"].endswith("memory_snapshot.json"))

    def test_nodata(self):
        self.assertEqual(BG.gate(self.r.d)["verdict"], "NODATA")


if __name__ == "__main__":
    unittest.main()
