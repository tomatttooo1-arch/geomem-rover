"""Camera half-rate check (2026-10-17, docs/design/COLOUR_RATE_NAVMODE_20261017.md section 7).

The Gemini 2 enters, at random on a start and for the whole run, a half-rate state: depth every 32.30 ms, colour every
64.61 ms (15.5 Hz); the normal state is 33.40 ms / 33.40 ms. tools/car_run/depth_probe.py --rate-s judges a window of
header-stamp intervals after the depth images (classify): depth median < 32.8 ms or colour 25th percentile > 50 ms =
half-rate (exit 3), < 20 depth / 10 colour intervals = insufficient (exit 4); car_inner.sh (CAMERA_RATE_CHECK=1) lets
car_start_retry restart such a start. Known answers:
1. classify on synthetic intervals of both measured signatures, the boundaries, too few intervals, a lossy normal
   stream (colour losses only lengthen intervals), the third signature of 10-15 rt_2039 (pre-declared: half).
2. --summarize on probe logs: one readable line per try.
3. car_inner.sh EXECUTED (not the dry run) with stand-ins (`ros2` = a driver that runs; the probe = a schedule of
   verdicts built by the real classify): half then normal -> try 1 restarted, try 2 kept, camera_rate_check.json of
   try 2; half x3 -> refused with the reason in stack_fail.txt; switch off -> the probe has no --rate-s and a "half"
   schedule is not even asked; with a DDS profile the shm8 segment check comes after the rate check.
Python 3.8+.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.car_run import depth_probe as dp  # noqa: E402

INNER = ROOT / "tools/car_run/car_inner.sh"

NORMAL_D = [33.40] * 90
NORMAL_C = [33.40] * 90
HALF_D = [32.30] * 93
HALF_C = [64.61] * 46


class TestClassify(unittest.TestCase):
    def test_two_signatures(self):
        n = dp.classify(NORMAL_D, NORMAL_C)
        self.assertEqual(n["verdict"], "normal", n)
        self.assertEqual(dp.rate_rc(n["verdict"]), 0)
        h = dp.classify(HALF_D, HALF_C)
        self.assertEqual(h["verdict"], "half", h)
        self.assertEqual(dp.rate_rc(h["verdict"]), dp.RC_HALF)
        self.assertEqual(len(h["reasons"]), 2)   # both rules see it
        self.assertEqual((h["depth_median_ms"], h["colour_p25_ms"]), (32.3, 64.61))

    def test_jitter_does_not_matter(self):
        import random
        rnd = random.Random(7)
        d = [33.40 + rnd.uniform(-0.3, 0.3) for _ in range(90)]
        c = [33.40 + rnd.uniform(-2, 2) for _ in range(90)]
        self.assertEqual(dp.classify(d, c)["verdict"], "normal")
        d = [32.30 + rnd.uniform(-0.3, 0.3) for _ in range(93)]
        c = [64.61 + rnd.uniform(-2, 2) for _ in range(46)]
        self.assertEqual(dp.classify(d, c)["verdict"], "half")

    def test_each_rule_alone(self):
        # depth rule alone (colour still normal: e.g. a half-rate timing with a short exposure, prediction 6)
        r = dp.classify(HALF_D, NORMAL_C)
        self.assertEqual(r["verdict"], "half")
        self.assertEqual(len(r["reasons"]), 1)
        self.assertIn("depth median", r["reasons"][0])
        # colour rule alone (depth lost every other frame on the way: 64.6 ms intervals, colour 64.6)
        r = dp.classify([64.6] * 46, HALF_C)
        self.assertEqual(r["verdict"], "half")
        self.assertIn("colour p25", r["reasons"][0])

    def test_boundaries(self):
        self.assertEqual(dp.classify([dp.DEPTH_HALF_MS] * 90, NORMAL_C)["verdict"], "normal")   # strict <
        self.assertEqual(dp.classify([dp.DEPTH_HALF_MS - 0.01] * 90, NORMAL_C)["verdict"], "half")
        self.assertEqual(dp.classify(NORMAL_D, [dp.COLOUR_HALF_MS] * 90)["verdict"], "normal")   # strict >
        self.assertEqual(dp.classify(NORMAL_D, [dp.COLOUR_HALF_MS + 0.01] * 90)["verdict"], "half")
        # the 25th percentile: 3/4 long colour intervals is the edge (linear interpolation between order statistics)
        c = [33.4] * 26 + [66.8] * 74   # p25 between index 24 and 25 -> 33.4: normal
        self.assertEqual(dp.classify(NORMAL_D, c)["verdict"], "normal")
        c = [33.4] * 24 + [66.8] * 76   # p25 at index 24.75 -> between 33.4 and 66.8, above 50: half
        self.assertEqual(dp.classify(NORMAL_D, c)["verdict"], "half")

    def test_lossy_normal_stream_is_normal(self):
        """a subscriber that loses colour frames: intervals become 2x / 3x, never shorter; 60 % lost is still normal
        (a median rule would have called it half-rate)"""
        c = [33.4] * 30 + [66.8] * 40 + [100.2] * 20
        r = dp.classify(NORMAL_D, c)
        self.assertGreater(r["colour_median_ms"], dp.COLOUR_HALF_MS)   # the median rule would refuse this start
        self.assertEqual(r["verdict"], "normal", r)
        # depth losses never shorten a depth interval either
        self.assertEqual(dp.classify([33.4] * 50 + [66.8] * 40, NORMAL_C)["verdict"], "normal")

    def test_insufficient(self):
        for d, c in (([], []), ([33.4] * 19, NORMAL_C), (NORMAL_D, [33.4] * 9), (HALF_D, [64.61] * 9)):
            r = dp.classify(d, c)
            self.assertEqual(r["verdict"], "insufficient", (len(d), len(c)))
            self.assertEqual(dp.rate_rc(r["verdict"]), dp.RC_INSUFFICIENT)
        self.assertEqual(dp.classify([33.4] * 20, [33.4] * 10)["verdict"], "normal")   # exactly enough

    def test_third_signature_rt_2039_is_half(self):
        """pre-declared (section 7): 10-15 rt_2039, colour 33.33 ms, depth mixed 32.2 / 31.9 / 39.3 ms (median about
        32.2 on the host files) -- not the normal 33.40 timing -> half -> one more start"""
        d = [32.2] * 50 + [31.9] * 20 + [39.3] * 18
        self.assertEqual(dp.classify(d, [33.33] * 89)["verdict"], "half")

    def test_intervals_from_stamps(self):
        st = [100.0 + i * 0.0334 for i in range(10)]
        iv = dp.intervals_ms(list(reversed(st)) + [st[3]])   # order and a repeated stamp do not matter
        self.assertEqual(len(iv), 9)
        self.assertAlmostEqual(dp.quantile(iv, 0.5), 33.4, places=6)
        self.assertEqual(dp.quantile([1.0, 2.0, 3.0, 4.0], 0.25), 1.75)


class TestSummarize(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="halfrate_sum_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def log(self, n, row, noise="[INFO] something\n"):
        p = os.path.join(self.tmp, "driver_probe_%d.log" % n)
        with open(p, "w") as f:
            f.write(noise + ("" if row is None else json.dumps(row) + "\n"))
        return p

    def test_lines(self):
        base = {"topic": "/d", "received": 30, "needed": 30, "seconds": 1.2}
        paths = [self.log(10, dict(base, rate=dp.classify(NORMAL_D, NORMAL_C))),
                 self.log(1, dict(base, rate=dp.classify(HALF_D, HALF_C))),
                 self.log(2, dict(base, received=3, rate=None)),
                 self.log(3, dict(base, rate=dp.classify([33.4] * 5, []))),
                 self.log(4, {"error": "import: x"}),
                 self.log(5, None)]
        lines = dp.summarize(paths)
        self.assertEqual([l.split(":")[0] for l in lines], ["try 1", "try 2", "try 3", "try 4", "try 5", "try 10"])
        self.assertTrue(lines[0].startswith("try 1: HALF-RATE (depth median 32.3 ms, colour p25 64.61 ms"), lines[0])
        self.assertTrue(lines[1].startswith("try 2: no depth stream (3 of 30"), lines[1])
        self.assertTrue(lines[2].startswith("try 3: too few frames for the rate check"), lines[2])
        self.assertEqual(lines[3], "try 4: probe error: import: x")
        self.assertEqual(lines[4], "try 5: no probe result")
        self.assertTrue(lines[5].startswith("try 10: normal (depth median 33.4 ms"), lines[5])
        self.assertEqual(dp.main(["--summarize"] + paths[:1]), 0)

    def test_old_usage_unchanged(self):
        """without --rate-s the probe is the probe of before (rig_inner.sh, bx_inner.sh, rtabmap_car.sh use it)"""
        try:
            import rclpy  # noqa: F401
        except ImportError:
            self.assertEqual(dp.main(["--topic", "/camera/depth/image_raw", "--timeout-s", "0.1"]), 2)


# --- car_inner.sh executed with stand-ins -------------------------------------------------------------------------
# the probe stand-in: tools/car_run/depth_probe.py in a stand-in source tree. --summarize and the imported helpers are
# the REAL module's; a probe call takes the next verdict of $PROBE_SCHEDULE (half | normal | insufficient | nodepth),
# prints the row the real probe would (rate from the real classify on the signature's intervals) and exits as it would
PROBE_STANDIN = textwrap.dedent('''\
    import importlib.util, json, os, sys, time
    spec = importlib.util.spec_from_file_location("real_probe", os.environ["REAL_PROBE"])
    real = importlib.util.module_from_spec(spec); spec.loader.exec_module(real)
    last_json, summarize, classify, rate_rc = real.last_json, real.summarize, real.classify, real.rate_rc
    SIG = {"normal": ([33.40] * 90, [33.40] * 90), "half": ([32.30] * 93, [64.61] * 46),
           "insufficient": ([33.40] * 5, [])}
    def main(argv):
        if "--summarize" in argv:
            return real.main(argv)
        d = os.environ["STANDIN_DIR"]
        with open(os.path.join(d, "probe_argv.txt"), "a") as f:
            f.write(" ".join(argv) + "\\n")
        n = sum(1 for _ in open(os.path.join(d, "probe_argv.txt")))
        v = os.environ["PROBE_SCHEDULE"].split()[n - 1]
        time.sleep(0.5)   # the driver has time to run
        row = {"topic": "/camera/depth/image_raw", "received": 0 if v == "nodepth" else 30, "needed": 30, "seconds": 1.0}
        rc = 1 if v == "nodepth" else 0
        if "--rate-s" in argv:
            row["rate"] = None
            if v != "nodepth":
                row["rate"] = classify(*SIG[v])
                rc = rate_rc(row["rate"]["verdict"])
        print(json.dumps(row), flush=True)
        return rc
    if __name__ == "__main__":
        sys.exit(main(sys.argv[1:]))
    ''')
# stand-in driver: ros2 launch ... runs until stopped; INT ends it (a background child of a non-interactive bash starts
# with SIGINT ignored -- the real chain uses env --default-signal=INT, which the env stand-in below drops)
ROS2_STANDIN = ("#!%s\nimport signal, sys, time\nsignal.signal(signal.SIGINT, lambda *a: sys.exit(0))\n"
                "time.sleep(300)\n" % sys.executable)


class TestInnerExecuted(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="halfrate_inner_")
        self.src = os.path.join(self.tmp, "src")
        self.o = os.path.join(self.tmp, "o")
        self.bin = os.path.join(self.tmp, "bin")
        for d in (os.path.join(self.src, "tools", "car_run"), self.o, self.bin, os.path.join(self.tmp, "ipc")):
            os.makedirs(d)
        for pkg in ("tools", "tools/car_run"):
            open(os.path.join(self.src, pkg, "__init__.py"), "w").close()
        with open(os.path.join(self.src, "tools", "car_run", "depth_probe.py"), "w") as f:
            f.write(PROBE_STANDIN)
        with open(os.path.join(self.src, "dds.xml"), "w") as f:
            f.write("<profiles/>\n")
        # this host's env has no --default-signal (coreutils < 8.31): a stand-in drops it (the board image has it)
        real_env = shutil.which("env")
        for name, body in (("ros2", ROS2_STANDIN),
                           ("env", '#!/bin/bash\n[ "$1" = --default-signal=INT ] && shift\nexec %s "$@"\n' % real_env)):
            p = os.path.join(self.bin, name)
            with open(p, "w") as f:
                f.write(body)
            os.chmod(p, 0o755)

    def tearDown(self):
        pids = os.path.join(self.o, "pids.txt")
        if os.path.exists(pids):
            with open(pids) as f:
                rows = f.read().splitlines()
            for line in rows:
                try:
                    os.kill(int(line.split()[1]), 9)
                except (OSError, ValueError, IndexError):
                    pass
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_inner(self, schedule, rate_check="1", tries="3", profile=""):
        env = dict(os.environ, CAMERA="live", POSE="static", NAV="costmap", DETECTOR="yolo", CHASSIS="none",
                   DRY_RUN="0", O=self.o, SRCD=self.src, IPC=os.path.join(self.tmp, "ipc"), HEAD="test",
                   LIVE_START_TRIES=tries, LIVE_START_WAIT_S="1", LIVE_START_GAP_S="0",
                   CAMERA_RATE_CHECK=rate_check, CAMERA_RATE_S="1", LIVE_DRIVER_DDS_PROFILE=profile,
                   PROBE_SCHEDULE=schedule, STANDIN_DIR=self.tmp, REAL_PROBE=str(ROOT / "tools/car_run/depth_probe.py"),
                   PATH=self.bin + os.pathsep + os.environ["PATH"])
        r = subprocess.run(["bash", str(INNER)], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True, timeout=180)
        marks = [l.split(" ", 1)[1] for l in Path(self.o, "marks.txt").read_text().splitlines()]
        argv = Path(self.tmp, "probe_argv.txt").read_text().splitlines() if Path(self.tmp, "probe_argv.txt").exists() else []
        fail = Path(self.o, "stack_fail.txt").read_text() if Path(self.o, "stack_fail.txt").exists() else ""
        return r, marks, argv, fail

    def test_half_then_normal_restarts_and_keeps_try_2(self):
        r, marks, argv, fail = self.run_inner("half normal")
        tries = [m for m in marks if m.startswith("camera driver try=")]
        self.assertEqual(tries, ["camera driver try=1 depth_stream=0 probe_rc=3",
                                 "camera driver try=2 depth_stream=1 probe_rc=0"], r.stdout)
        self.assertEqual(len(argv), 2)
        for a in argv:
            self.assertIn("--count 30 --timeout-s 1 --rate-s 1 --color-topic /camera/color/image_raw", a)
        summ = next(m for m in marks if m.startswith("camera rate check tries: "))
        self.assertIn("try 1: HALF-RATE (depth median 32.3 ms", summ)
        self.assertIn("; try 2: normal (depth median 33.4 ms", summ)
        self.assertIn("ready camera_rate try 2: normal", "\n".join(marks))
        js = json.load(open(os.path.join(self.o, "camera_rate_check.json")))
        self.assertEqual((js["try"], js["rate"]["verdict"], js["rate"]["depth_median_ms"]), (2, "normal", 33.4))
        self.assertTrue(os.path.exists(os.path.join(self.o, "driver_failed_start_1.log")))
        self.assertFalse(os.path.exists(os.path.join(self.o, "driver_failed_start_2.log")))
        # the camera is kept: "ready camera live"; the stand-in tree has no camera_check.py, the run stops there
        self.assertIn("ready camera live", marks)
        self.assertIn("camera check", fail)

    def test_every_try_half_is_refused_with_the_reason(self):
        r, marks, argv, fail = self.run_inner("half insufficient half")
        self.assertEqual([m for m in marks if m.startswith("camera driver try=")],
                         ["camera driver try=%d depth_stream=0 probe_rc=%d" % (n, rc) for n, rc in ((1, 3), (2, 4), (3, 3))])
        self.assertIn("Orbbec driver: no good start in 3 starts", fail)
        self.assertIn("try 1: HALF-RATE", fail)
        self.assertIn("try 2: too few frames for the rate check", fail)
        self.assertIn("try 3: HALF-RATE", fail)
        self.assertFalse(os.path.exists(os.path.join(self.o, "camera_rate_check.json")))
        self.assertNotIn("ready camera live", marks)
        for n in (1, 2, 3):
            self.assertTrue(os.path.exists(os.path.join(self.o, "driver_failed_start_%d.log" % n)))

    def test_depth_failures_still_reported(self):
        r, marks, argv, fail = self.run_inner("nodepth nodepth", tries="2")
        self.assertIn("try 1: no depth stream (0 of 30", fail)

    def test_switch_off_is_the_probe_of_before(self):
        r, marks, argv, fail = self.run_inner("half half", rate_check="0")
        self.assertEqual(argv, ["--topic /camera/depth/image_raw --count 30 --timeout-s 1"])
        self.assertEqual([m for m in marks if m.startswith("camera driver try=")],
                         ["camera driver try=1 depth_stream=1 probe_rc=0"])
        self.assertFalse(any("camera rate" in m or "camera_rate" in m for m in marks), marks)
        self.assertFalse(os.path.exists(os.path.join(self.o, "camera_rate_check.json")))
        # control: the same schedule with the switch on is a failed start (the stand-in can say "half")
        shutil.rmtree(self.o); os.makedirs(self.o); os.remove(os.path.join(self.tmp, "probe_argv.txt"))
        r, marks, argv, fail = self.run_inner("half half", rate_check="1", tries="2")
        self.assertIn("camera driver try=1 depth_stream=0 probe_rc=3", marks)

    def test_shm_check_after_the_rate_check(self):
        """with a driver DDS profile: the retry cleans before try 2, the rate check keeps try 2, THEN the shm8
        segment check runs on the kept start (here it fails: no shm_segcheck.py in the stand-in tree)"""
        r, marks, argv, fail = self.run_inner("half normal", profile="dds.xml")
        i_before = next(i for i, m in enumerate(marks) if m.startswith("camera driver before try=2 rc="))
        i_rate = next(i for i, m in enumerate(marks) if m.startswith("ready camera_rate try 2: normal"))
        i_shm = next(i for i, m in enumerate(marks) if m.startswith("FAIL camera driver shared memory"))
        self.assertLess(i_before, i_rate)
        self.assertLess(i_rate, i_shm)

    def test_bad_switch_values_refused(self):
        for k, v in (("CAMERA_RATE_CHECK", "2"), ("CAMERA_RATE_S", "0"), ("CAMERA_RATE_S", "6"), ("CAMERA_RATE_S", "1.5")):
            env = dict(os.environ, CAMERA="live", POSE="static", NAV="costmap", DETECTOR="yolo", CHASSIS="none",
                       DRY_RUN="1", **{k: v})
            r = subprocess.run(["bash", str(INNER)], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               universal_newlines=True, timeout=60)
            self.assertEqual(r.returncode, 2, (k, v, r.stdout))


if __name__ == "__main__":
    unittest.main()
