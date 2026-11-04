"""tools/integ_car_chain/cam_drop/ (2026-10-13 camera-drop layer experiments, entered 2026-10-14).

1. Every shell script parses (bash -n), both Python tools compile.
2. bx_inner.sh, the START_TRIES defect of arm l_a1: the defaults block of the script is EXECUTED under set -u
   with no START_* in the environment; afterwards START_TRIES / START_WAIT_S / START_GAP_S are set in the main
   shell. Control: the same block without the head-of-script defaults line reproduces the l_a1 failure
   ("START_TRIES: unbound variable" after the $(seq ...) default), so the test can see the defect.
3. bx_readout.py on hand-built arms with known answers:
   replay camera: 60 publisher stamps; the receiver misses 5 colour images (runs 1, 2, 2) and 1 camera_info
   (at a stamp whose colour image is missing too), depth complete, nothing extra;
   driver record: 40 frame sets, one depth row arrived without a publish time, the receiver misses 3 depth
   images (one of them is the unpublished one) and the depth camera_info of one of them.
4. (2026-10-14) the DDS message-lost field: counted inside the window by event wall time, missing minus lost,
   null for a receiver without the field (old arms, cam_probe), unsupported reported as such; the producer's
   input_log.csv read as receiver "producer" in a chain directory (missing_by_receiver_set over three receivers).
5. batch2.txt arms use bx.sh's variables; the chain hooks of l_chain32 (rig_inner.sh RX_LITE / LIVE_DRV_LOG,
   default off; nano_chain_inner_v2.sh hands them on; nano_v3.sh accepts them; session_v3.sh arm l = arm f +
   the two hooks); the rig_inner.sh window.txt line is executed.
Python 3.8+.
"""
import csv
import json
import os
from pathlib import Path
import py_compile
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
D = ROOT / "tools/integ_car_chain/cam_drop"
INNER = D / "bx_inner.sh"
READOUT = D / "bx_readout.py"
PERIOD_NS = 33333333
T0_NS = 1791876400000000000


class Parse(unittest.TestCase):
    def test_shell_and_python(self):
        shs = sorted(D.glob("*.sh"))
        self.assertGreaterEqual(len(shs), 8)
        for s in shs:
            r = subprocess.run(["bash", "-n", str(s)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(r.returncode, 0, "%s: %s" % (s.name, r.stdout))
        tmp = tempfile.mkdtemp()
        try:
            for p in (READOUT, D / "sysmon.py"):
                py_compile.compile(str(p), cfile=os.path.join(tmp, p.name + "c"), doraise=True)
        finally:
            shutil.rmtree(tmp)

    def test_batch1_arms_use_known_variables(self):
        allowed = set(re.search(r"RX\|RX_INFO\|[A-Z_|]+\)", (D / "bx.sh").read_text()).group(0)[:-1].split("|"))
        allowed |= {"STRESS", "DUR", "WARM", "CAMERA"}
        arms = [x.split() for x in (D / "batch1.txt").read_text().splitlines() if x.strip() and not x.startswith("#")]
        self.assertEqual([a[0] for a in arms], ["l_b1", "l_s1", "r_shm32", "r_def2"])
        for a in arms:
            for kv in a[1:]:
                self.assertIn(kv.split("=", 1)[0], allowed, a)
            for kv in a[1:]:
                if kv.startswith("CAM_DDS_PROFILE="):       # /repo is the worktree inside the container
                    self.assertTrue((ROOT / kv.split("=", 1)[1][len("/repo/"):]).is_file(), kv)


    def test_batch2_arms_use_known_variables(self):
        allowed = set(re.search(r"RX\|RX_INFO\|[A-Z_|]+\)", (D / "bx.sh").read_text()).group(0)[:-1].split("|"))
        allowed |= {"STRESS", "DUR", "WARM", "CAMERA"}
        text = (D / "batch2.txt").read_text()
        arms = [x.split() for x in text.splitlines() if x.strip() and not x.startswith("#")]
        self.assertEqual([a[0] for a in arms], ["l_s1", "l_b1"])
        b1 = dict((x.split()[0], x) for x in (D / "batch1.txt").read_text().splitlines()
                  if x.strip() and not x.startswith("#"))
        for a in arms:
            self.assertEqual(" ".join(a), b1[a[0]])                    # the same arms as batch 1
            for kv in a[1:]:
                self.assertIn(kv.split("=", 1)[0], allowed, a)
        # l_chain32: one command line, INTEG_W a placeholder for a worktree of main (never a concrete path:
        # the cam-drop-20261013 worktree it named before is deleted), the session started from $INTEG_W itself
        chain = [x for x in text.splitlines() if "V8CAM_ORDER=" in x]
        self.assertEqual(len(chain), 1, chain)
        self.assertRegex(chain[0], r"^#   INTEG_W=<a worktree of main[^<>/]*> V8CAM_ORDER=l1 "
                                   r"bash \$INTEG_W/docs/design/integ_car_chain_v3_20261011/session_v3\.sh v8cam$")
        self.assertEqual(len(re.findall(r"INTEG_W=", text)), 1)              # deploy main, not the default
        self.assertNotIn("/data1/", chain[0])
        self.assertNotIn("cam-drop-20261013-worktree", text)


class ChainHooks(unittest.TestCase):
    RIG = ROOT / "tools/car_geo_producer/rig_inner.sh"
    INNER2 = ROOT / "tools/integ_car_chain/nano_chain_inner_v2.sh"
    NANO = ROOT / "docs/design/integ_car_chain_v3_20261011/nano_v3.sh"
    SESSION = ROOT / "docs/design/integ_car_chain_v3_20261011/session_v3.sh"

    def test_rig_hooks_default_off_and_live_only(self):
        text = self.RIG.read_text()
        self.assertIn('[ "${LIVE_DRV_LOG:-0}" = 1 ] && LIVE_LAUNCH_ARGS="$LIVE_LAUNCH_ARGS enable_frame_drop_log:=true '
                      'frame_timestamp_csv_file:=$OUT/drv_frames.csv"', text)
        live = text[text.index('echo "live_launch_args=$LIVE_LAUNCH_ARGS"'):text.index('LIVE=(--depth-topic')]
        self.assertIn("frame_timestamp_csv_file", text[text.index("else\n  [ \"${LIVE_DRV_LOG"):text.index("for try in")])
        a = text.index('python3 tools/integ_car_chain/cam_probe.py')
        b = text.index('LIVE=(--depth-topic')
        block = text[a:b]
        self.assertIn('if [ "${RX_LITE:-0}" = 1 ]; then', block)          # inside the live branch only
        self.assertIn("python3 tools/integ_car_chain/rx_lite.py", block)
        for tok in ('--out "$OUT/rx_chain.csv"', '--summary "$OUT/rx_chain.json"', "--reliability best_effort --depth 5",
                    '--info-topic "$LIVE_INFO_TOPIC" --info-topic "$LIVE_COLOR_INFO_TOPIC"', "pids+=($!)"):
            self.assertIn(tok, block)
        self.assertEqual(text.count("rx_lite.py"), 2)                    # the header comment and the one start
        self.assertIn('if [ "${RX_LITE:-0}" = 1 ]; then', live)

    def test_rig_window_line_executes(self):
        text = self.RIG.read_text()
        i = text.index('[ "${RX_LITE:-0}" = 1 ] && python3 -c')
        line = text[i:text.index('> "$OUT/window.txt"', i) + len('> "$OUT/window.txt"')]
        tmp = Path(tempfile.mkdtemp())
        try:
            for rx_lite, expect in (("1", "1791876400.500 1791876580.500"), ("0", None)):
                env = dict(os.environ, RX_LITE=rx_lite, OUT=str(tmp), WIN_START="1791876400.500", DUR="180")
                subprocess.run(["bash", "-c", line], env=env, check=False)
                w = tmp / "window.txt"
                if expect is None:
                    self.assertFalse(w.exists())
                else:
                    self.assertEqual(w.read_text().strip(), expect)
                    w.unlink()
        finally:
            shutil.rmtree(str(tmp))

    def test_inner_session_and_nano_pass_the_hooks(self):
        inner = self.INNER2.read_text()
        self.assertIn('  RX_LITE="${RX_LITE:-0}" LIVE_DRV_LOG="${LIVE_DRV_LOG:-0}" \\\n', inner)
        nano = self.NANO.read_text()
        live = nano[nano.index("livechain)\n"):nano.index("camargs)\n")]
        self.assertIn("|RX_LITE=*|LIVE_DRV_LOG=*|", live)       # (+ RUNTIME=*|RUNTIME_EXTRA=* since 2026-10-15)
        chain = nano[nano.index("\nchain)\n"):nano.index("livechain)\n")]
        self.assertNotIn("RX_LITE", chain)                              # the replay chain does not take them
        src = self.SESSION.read_text()
        arms = dict(re.findall(r'(?m)^      ([fcl])\*\) CE="([^"]*)" ;;$', src))
        self.assertEqual(arms["l"], arms["f"] + " RX_LITE=1 LIVE_DRV_LOG=1")
        for p in (self.RIG, self.INNER2, self.NANO, self.SESSION):
            r = subprocess.run(["bash", "-n", str(p)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(r.returncode, 0, r.stdout)


class StartTriesDefaults(unittest.TestCase):
    def _block(self, keep_head_defaults):
        lines = INNER.read_text().splitlines()
        a = lines.index("set -u")
        b = next(i for i, x in enumerate(lines) if x.startswith("OUT=/out"))
        block = [x for x in lines[a:b] if not x.startswith("cd /repo")]
        if not keep_head_defaults:
            block = [x for x in block if not x.startswith(': "${START_TRIES:=')]
        # what the retry branch evaluates: the loop header's $(seq ...) default, then the bare name
        block += ['n=$(seq 1 "${START_TRIES:=10}" | wc -l)',
                  'echo "n=$n tries=$START_TRIES wait=$START_WAIT_S gap=$START_GAP_S"']
        env = dict((k, v) for k, v in os.environ.items() if not k.startswith("START_"))
        return subprocess.run(["bash", "-c", "\n".join(block)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              env=env, universal_newlines=True)

    def test_defaults_survive_in_main_shell(self):
        r = self._block(True)
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertEqual(r.stdout.strip().splitlines()[-1], "n=10 tries=10 wait=20 gap=10")

    def test_control_without_head_defaults_reproduces_l_a1(self):
        r = self._block(False)
        self.assertNotEqual(r.returncode, 0, r.stdout)
        self.assertIn("START_TRIES: unbound variable", r.stdout)


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def sec_text(ns):
    return "%d.%06d" % (ns // 10 ** 9, (ns % 10 ** 9) // 1000)


class Readout(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(str(self.tmp))

    def _run(self, d):
        out = self.tmp / "out.json"
        r = subprocess.run([sys.executable, str(READOUT), str(d), "--json", str(out)], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, universal_newlines=True)
        self.assertEqual(r.returncode, 0, r.stdout)
        return json.loads(out.read_text())

    def _window(self, d, n):
        # window strictly around the n stamps (whole-us stamps; window in seconds)
        (d / "window.txt").write_text("%.3f %.3f\n" % ((T0_NS - 10 ** 7) / 1e9, (T0_NS + n * PERIOD_NS) / 1e9))

    def test_replay_camera_known_answer(self):
        d = self.tmp / "r1"
        d.mkdir()
        n = 60
        stamps = [T0_NS + i * PERIOD_NS for i in range(n)]
        self._window(d, n)
        (d / "env.txt").write_text("camera=replay\n")
        write_csv(d / "replay.csv", ["k", "src", "stamp_ns", "publish_raw_ns"], [[i, 0, s, s] for i, s in enumerate(stamps)])
        miss_rgb = {10, 20, 21, 40, 41}
        miss_info = {21}
        rows = []
        for i, s in enumerate(stamps):
            rows.append(["/fpga/depth", s, s + 7000000, 0, 460868])
            if i not in miss_rgb:
                rows.append(["/fpga/rgb", s, s + 5000000, 0, 691268])
            if i not in miss_info:
                rows.append(["/fpga/camera_info", s, s + 1000000, 0, 376])
        write_csv(d / "rx_be.csv", ["topic", "stamp_ns", "wall_ns", "arrival_raw_ns", "bytes"], rows)
        res = self._run(d)
        self.assertEqual(res["publisher"]["rows_in_window"], n)
        t = res["receivers"]["rx_be"]["topics"]
        self.assertEqual(t["/fpga/depth"]["vs_publisher"]["missing"], 0)
        rgb = t["/fpga/rgb"]["vs_publisher"]
        self.assertEqual((rgb["missing"], rgb["extra_not_in_publisher_record"]), (5, 0))
        self.assertEqual(rgb["missing_run_lengths"], {"1": 1, "2": 2})
        sm = t["/fpga/rgb"]["small_message_control"]
        self.assertEqual((sm["image_missing"], sm["of_those_camera_info_present"], sm["camera_info_missing"],
                          sm["both_missing"]), (5, 4, 1, 1))
        self.assertEqual(t["/fpga/camera_info"]["vs_publisher"]["missing"], 1)

    def test_driver_record_known_answer(self):
        d = self.tmp / "l1"
        d.mkdir()
        n = 40
        stamps = [T0_NS + i * PERIOD_NS for i in range(n)]
        self._window(d, n)
        (d / "env.txt").write_text("driver_start try=1 depth_stream=0\ndriver_start try=2 depth_stream=1\n")
        cols = []
        for s in ("color", "depth"):
            cols += [s + x for x in ("_sdk_frame_index", "_hardware_frame_number", "_sensor_ts_sec", "_sensor_ts_delta_us",
                                     "_device_ts_sec", "_device_ts_delta_us", "_global_ts_sec", "_global_ts_delta_us",
                                     "_system_ts_sec", "_system_ts_delta_us", "_arrival_steady_delta_us",
                                     "_publish_steady_delta_us", "_arrival_to_publish_steady_us",
                                     "_sdk_delay_from_global_us", "_sdk_delay_from_system_us")]
        unpublished = 15
        rows = []
        for i, st in enumerate(stamps):
            row = []
            for s in ("color", "depth"):
                a2p = "" if (s == "depth" and i == unpublished) else "1300"
                row += [i + 1, "", "", "", sec_text(st - 2000), "", sec_text(st), "", "", "", "", "", a2p, 67000, ""]
            rows.append(row)
        write_csv(d / "drv_frames.csv", cols, rows)
        miss_depth = {unpublished, 30, 31}
        rx = []
        for i, st in enumerate(stamps):
            if i not in miss_depth:
                rx.append(["/camera/depth/image_raw", st, st + 70000000, 0, 614472])
            if i != 30:
                rx.append(["/camera/depth/camera_info", st, st + 68000000, 0, 408])
            rx.append(["/camera/color/image_raw", st, st + 76000000, 0, 921672])
            rx.append(["/camera/color/camera_info", st, st + 70000000, 0, 408])
        write_csv(d / "rx_be.csv", ["topic", "stamp_ns", "wall_ns", "arrival_raw_ns", "bytes"], rx)
        (d / "driver.log").write_text("[WARN] stage=SDK_RECEIVE gap\nok\n")
        res = self._run(d)
        self.assertEqual(res["driver_start_tries"], 2)
        pd = res["publisher"]["depth"]
        self.assertEqual((pd["arrived_at_driver"], pd["published"], pd["arrived_not_published"]), (n, n - 1, 1))
        self.assertEqual((pd["device_stamp_gap_rule"]["missed"], pd["sdk_index_missing"]), (0, 0))
        self.assertEqual(res["publisher"]["color"]["arrived_not_published"], 0)
        t = res["receivers"]["rx_be"]["topics"]
        dv = t["/camera/depth/image_raw"]["vs_publisher"]
        # the unpublished frame is not in the publisher's published list, so it is not counted as missing
        self.assertEqual((dv["published"], dv["missing"], dv["extra_not_in_publisher_record"]), (n - 1, 2, 0))
        self.assertEqual(dv["missing_run_lengths"], {"2": 1})
        sm = t["/camera/depth/image_raw"]["small_message_control"]
        self.assertEqual((sm["image_missing"], sm["of_those_camera_info_present"], sm["both_missing"]), (2, 1, 1))
        self.assertEqual(t["/camera/color/image_raw"]["vs_publisher"]["missing"], 0)
        self.assertEqual((res["driver_log"]["drop_sdk_receive_lines"], res["driver_log"]["drop_ros_publish_lines"]), (1, 0))


    def _lost(self, events):
        """rx_lite "lost" record from [(wall_ns, change)]."""
        total, log = 0, []
        for w, ch in events:
            total += ch
            log.append([w, ch, total])
        return {"supported": True, "total_count": total, "events": len(log), "change_sum": total, "log": log}

    def test_dds_message_lost_field(self):
        d = self.tmp / "lost"
        d.mkdir()
        n = 60
        stamps = [T0_NS + i * PERIOD_NS for i in range(n)]
        self._window(d, n)
        (d / "env.txt").write_text("camera=replay\n")
        write_csv(d / "replay.csv", ["k", "src", "stamp_ns", "publish_raw_ns"], [[i, 0, s, s] for i, s in enumerate(stamps)])
        miss = {5, 6, 30}
        rows = []
        for i, s in enumerate(stamps):
            rows.append(["/fpga/depth", s, s + 7000000, 0, 460868])
            if i not in miss:
                rows.append(["/fpga/rgb", s, s + 5000000, 0, 691268])
            rows.append(["/fpga/camera_info", s, s + 1000000, 0, 376])
        for name in ("rx_be", "rx_old", "rx_x"):
            write_csv(d / (name + ".csv"), ["topic", "stamp_ns", "wall_ns", "arrival_raw_ns", "bytes"], rows)
        before = T0_NS - 5 * 10 ** 8                                   # an event in the warm-up: whole run only
        lost_rgb = self._lost([(before, 1)] + [(stamps[i] + 40000000, 1) for i in (5, 6)])   # 30 not counted
        (d / "rx_be.json").write_text(json.dumps({"_receiver": {"kind": "rclpy raw subscription"},
                                                  "/fpga/rgb": {"lost": lost_rgb}, "/fpga/depth": {"lost": self._lost([])},
                                                  "/fpga/camera_info": {"lost": self._lost([])}}))
        (d / "rx_old.json").write_text(json.dumps({"_receiver": {}, "/fpga/rgb": {"parse_check": {}}}))  # before 10-14
        (d / "rx_x.json").write_text(json.dumps({"/fpga/rgb": {"lost": {"supported": False,
                                                                       "error": "UnsupportedEventTypeError()"}}}))
        res = self._run(d)
        t = res["receivers"]["rx_be"]["topics"]
        lo = t["/fpga/rgb"]["dds_message_lost"]
        self.assertEqual((lo["whole_run_total"], lo["events"], lo["in_window"], lo["missing_minus_lost_in_window"]),
                         (3, 3, 2, 1))
        self.assertTrue(lo["log_complete"])
        self.assertEqual(t["/fpga/depth"]["dds_message_lost"]["in_window"], 0)
        self.assertIsNone(res["receivers"]["rx_old"]["topics"]["/fpga/rgb"]["dds_message_lost"])     # not recorded
        self.assertIsNone(res["receivers"]["rx_old"]["topics"]["/fpga/depth"]["dds_message_lost"])
        self.assertEqual(res["receivers"]["rx_x"]["topics"]["/fpga/rgb"]["dds_message_lost"],
                         {"supported": False, "error": "UnsupportedEventTypeError()"})
        table = dict(((r["receiver"], r["topic"]), r) for r in res["dds_lost_summary"])
        self.assertEqual(len(table), 9)
        self.assertEqual((table[("rx_be", "/fpga/rgb")]["missing"], table[("rx_be", "/fpga/rgb")]["lost_in_window"]), (3, 2))
        self.assertEqual((table[("rx_old", "/fpga/rgb")]["lost_in_window"], table[("rx_old", "/fpga/rgb")]["recorded"]),
                         (None, False))
        self.assertFalse(table[("rx_x", "/fpga/rgb")]["recorded"])

    def test_chain_dir_with_producer_input_log(self):
        d = self.tmp / "rig"
        d.mkdir()
        n = 40
        stamps = [T0_NS + i * PERIOD_NS for i in range(n)]
        self._window(d, n)
        (d / "env.txt").write_text("driver_start try=1 depth_stream=1\n")
        cols = []
        for s in ("color", "depth"):
            cols += [s + x for x in ("_sdk_frame_index", "_hardware_frame_number", "_device_ts_sec", "_global_ts_sec",
                                     "_arrival_to_publish_steady_us", "_sdk_delay_from_global_us")]
        write_csv(d / "drv_frames.csv", cols,
                  [[i + 1, "", sec_text(st), sec_text(st), "1300", 67000] * 2 for i, st in enumerate(stamps)])
        # producer misses 3, 4, 20; rx_chain misses 4; cam_probe misses 4, 20
        write_csv(d / "input_log.csv", ["stamp_ns", "arrival_raw_ns", "disposition"],
                  [[st, 0, "put" if i % 3 == 0 else "rate_skipped"] for i, st in enumerate(stamps) if i not in (3, 4, 20)])
        write_csv(d / "rx_chain.csv", ["topic", "stamp_ns", "wall_ns", "arrival_raw_ns", "bytes"],
                  [["/camera/depth/image_raw", st, st + 70000000, 0, 614472] for i, st in enumerate(stamps) if i != 4])
        write_csv(d / "cam_probe.csv", ["topic", "stamp_ns", "ros_ns"],
                  [["/camera/depth/image_raw", st, st + 70000000] for i, st in enumerate(stamps) if i not in (4, 20)])
        res = self._run(d)
        self.assertEqual(sorted(res["receivers"]), ["cam_probe", "producer", "rx_chain"])
        pv = res["receivers"]["producer"]["topics"]["/camera/depth/image_raw"]
        self.assertEqual((pv["vs_publisher"]["missing"], pv["vs_publisher"]["extra_not_in_publisher_record"]), (3, 0))
        self.assertIsNone(pv["dds_message_lost"])
        self.assertEqual(res["missing_by_receiver_set"]["depth"],
                         {"producer": 1, "cam_probe+producer+rx_chain": 1, "cam_probe+producer": 1})


if __name__ == "__main__":
    unittest.main()
