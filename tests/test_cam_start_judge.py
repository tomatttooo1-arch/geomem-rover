"""Live driver start judged by depth DATA (2026-10-15, cam-start-dig), rig_inner.sh and bx_inner.sh.

The retry loops of tools/car_geo_producer/rig_inner.sh (CAMERA=live) and tools/integ_car_chain/cam_drop/bx_inner.sh
are cut out of the scripts and EXECUTED with stand-ins: `ros2` (the driver: prints "depth Frame" on every start, as
the real driver did on starts that then sent no depth image, and keeps running unless told to exit) and `python3`
(tools/car_run/depth_probe.py: its exit codes follow a schedule). Known answers:
1. probe fails, fails, succeeds -> tries 1 and 2 judged bad (although every try printed "depth Frame"), try 3 good,
   the loop stops there; failed tries keep driver_failed_start_N.log; every try has driver_probe_N.log.
2. probe succeeds but the driver already exited -> bad.
3. control: the old judgment (grep "depth Frame") would have called try 1 good -- the stand-in really prints the line.
Also: no "depth Frame" grep decides a start any more; depth_probe.py is py3.8-compilable.
Python 3.8+.
"""
import os
from pathlib import Path
import py_compile
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
RIG = ROOT / "tools/car_geo_producer/rig_inner.sh"
BX = ROOT / "tools/integ_car_chain/cam_drop/bx_inner.sh"
PROBE = ROOT / "tools/car_run/depth_probe.py"

STUBS = r'''
set -m   # background jobs keep SIGINT (a non-interactive shell would start them with SIGINT ignored)
ros2() {
  echo "[camera] depth Frame - Width: 640 Height: 400"
  [ "${STUB_DRIVER_EXITS:-0}" = 1 ] && return 0
  sleep 30
}
python3() {
  local n; n=$(( $(cat "$T/probe_calls") + 1 )); echo $n > "$T/probe_calls"
  local rc; rc=$(echo "$PROBE_RCS" | cut -d' ' -f$n); echo "probe call $n args=$* rc=$rc"
  sleep 2     # the real probe waits for messages: the driver, launched 1 s after it, has time to run (or to exit)
  return "${rc:-1}"
}
'''


def loop_block(path, header):
    return loop_block_text(path.read_text(), header)


def loop_block_text(text, header):
    lines = text.splitlines()
    a = next(i for i, x in enumerate(lines) if x.strip() == header)
    b = next(i for i in range(a + 1, len(lines)) if lines[i] == "  done")
    return "\n".join(lines[a:b + 1])


class StartJudgedByDepthData(unittest.TestCase):
    def run_loop(self, path, header, env_lines, probe_rcs, driver_exits=False):
        t = tempfile.mkdtemp(prefix="camstart_")
        Path(t, "probe_calls").write_text("0\n")
        script = "\n".join(["set -u", "T=%s" % t, STUBS] + env_lines + [loop_block(path, header)]
                           + ['echo "LOOP_END try=$try started=$started"'])
        env = dict(os.environ, PROBE_RCS=probe_rcs, STUB_DRIVER_EXITS="1" if driver_exits else "0")
        r = subprocess.run(["bash", "-c", script], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                           universal_newlines=True, timeout=120)
        return t, r

    RIG_ENV = ['OUT=$T', 'LIVE_DEPTH_TOPIC=/camera/depth/image_raw', 'LIVE_LAUNCH_ARGS="a:=1"', 'CAM_ENV=()',
               'LIVE_START_TRIES=3', 'LIVE_START_WAIT_S=1', 'LIVE_START_GAP_S=0', 'touch $T/env.txt']
    RIG_HEADER = 'for try in $(seq 1 "${LIVE_START_TRIES:-10}"); do'

    def test_rig_fail_fail_good(self):
        t, r = self.run_loop(RIG, self.RIG_HEADER, self.RIG_ENV, "1 1 0")
        self.assertIn("LOOP_END try=3 started=1", r.stdout, r.stdout)
        tries = [x for x in Path(t, "env.txt").read_text().splitlines() if x.startswith("driver_start")]
        self.assertEqual(tries, ["driver_start try=1 depth_stream=0 probe_rc=1",
                                 "driver_start try=2 depth_stream=0 probe_rc=1",
                                 "driver_start try=3 depth_stream=1 probe_rc=0"])
        for n in (1, 2):
            self.assertTrue(Path(t, "driver_failed_start_%d.log" % n).exists())
        self.assertFalse(Path(t, "driver_failed_start_3.log").exists())
        for n in (1, 2, 3):
            self.assertIn("--topic /camera/depth/image_raw --count 30 --timeout-s 2",
                          Path(t, "driver_probe_%d.log" % n).read_text())
        # control: every try printed the line the old judgment grepped for
        self.assertIn("depth Frame", Path(t, "driver_failed_start_1.log").read_text())

    def test_rig_probe_good_but_driver_gone_is_bad(self):
        t, r = self.run_loop(RIG, self.RIG_HEADER, self.RIG_ENV, "0 0 0", driver_exits=True)
        self.assertIn("LOOP_END try=3 started=0", r.stdout, r.stdout)
        self.assertIn("driver_start try=1 depth_stream=0 probe_rc=0", Path(t, "env.txt").read_text())

    def test_bx_fail_good(self):
        env = ['OUT=$T', 'E=$T/env.txt', 'DEPTH=/camera/depth/image_raw', 'ARGS="a:=1"', 'CAM_ENV=()',
               'START_TRIES=3', 'START_WAIT_S=1', 'START_GAP_S=0', 'touch $E']
        t, r = self.run_loop(BX, 'for try in $(seq 1 "${START_TRIES:=10}"); do', env, "1 0")
        self.assertIn("LOOP_END try=2 started=1", r.stdout, r.stdout)
        tries = [x.split(" at=")[0] for x in Path(t, "env.txt").read_text().splitlines() if x.startswith("driver_start")]
        self.assertEqual(tries, ["driver_start try=1 depth_stream=0 probe_rc=1", "driver_start try=2 depth_stream=1 probe_rc=0"])

    # 10-15 (controller, board v8rt): dd5082b wrote the depth probe's pid into PROBE inside the start loop, and
    # rig_inner.sh takes PROBE=1 from outside as the system sampler switch (tf_probe.py --sys-csv): after any live
    # start PROBE held a pid, the sampler never started and sys.csv was missing. Known answer: PROBE=1 before the
    # loop -> still 1 after it, and the sampler block starts tf_probe.py. Control: the same run on the old form
    # (the pid stored in PROBE) must lose the sampler -- the check can see the defect.
    SAMPLER_STUBS = r'''
set -m
ros2() { exec sleep 30; }
python3() {
  echo "python3 $*" >> "$T/py_calls"
  case "$*" in *depth_probe.py*) sleep 2;; esac   # the depth probe: the data arrived
  return 0
}
'''

    @staticmethod
    def sampler_block(text):
        lines = text.splitlines()
        a = lines.index('if [ "${PROBE:-0}" = 1 ]; then')
        b = next(i for i in range(a + 1, len(lines)) if lines[i] == "fi")
        return "\n".join(lines[a:b + 1])

    def run_live_start_then_sampler(self, text):
        t = tempfile.mkdtemp(prefix="camstart_probe_")
        loop = loop_block_text(text, self.RIG_HEADER)
        script = "\n".join(["set -u", "T=%s" % t, self.SAMPLER_STUBS, "PROBE=1"] + self.RIG_ENV + [loop]
                           + ["pids=()", "PROD=11 BRIDGE=12 REPLAY=13 WARM=1 DUR=1",
                              self.sampler_block(text),
                              'for p in "${pids[@]}"; do wait "$p"; done',
                              'kill "$LAUNCH" 2>/dev/null',
                              'echo "AFTER try=$try started=$started PROBE=$PROBE"'])
        r = subprocess.run(["bash", "-c", script], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True, timeout=120)
        calls = Path(t, "py_calls").read_text() if Path(t, "py_calls").exists() else ""
        return r.stdout, [c for c in calls.splitlines() if "tf_probe.py" in c]

    @staticmethod
    def after_probe(out):
        """PROBE value on the exact 'AFTER try=1 started=1 PROBE=<v>' line, or None if that line is missing."""
        m = re.search(r"^AFTER try=1 started=1 PROBE=(\S*)$", out, re.M)
        return m.group(1) if m else None

    def test_rig_probe_switch_survives_live_start(self):
        text = RIG.read_text()
        out, sampler = self.run_live_start_then_sampler(text)
        self.assertEqual(self.after_probe(out), "1", out)
        self.assertEqual(len(sampler), 1, out)
        self.assertIn("--sys-csv", sampler[0])
        self.assertFalse("    PROBE=$!" in text, "rig_inner.sh stores a pid in PROBE again")
        # positive control: the dd5082b form (depth probe pid in PROBE) loses the switch and the sampler
        old = text.replace("DPROBE=$!", "PROBE=$!").replace("wait $DPROBE", "wait $PROBE")
        self.assertNotEqual(old, text)
        out, sampler = self.run_live_start_then_sampler(old)
        # whole-value compare: a pid such as 12113 must not read as "1" (substring check was pid-dependent)
        v = self.after_probe(out)
        self.assertIsNotNone(v, out)
        self.assertTrue(v.isdigit(), out)   # PROBE holds the depth probe's pid
        self.assertNotEqual(v, "1", out)
        self.assertEqual(sampler, [], out)

    def test_bx_depth_probe_pid_not_probe(self):
        text = BX.read_text()
        self.assertNotIn("PROBE=$!", text.replace("DPROBE=$!", ""))
        self.assertIn("wait $DPROBE", text)

    def test_no_depth_frame_grep_decides(self):
        for path in (RIG, BX):
            text = path.read_text()
            self.assertNotIn('grep -a -q "depth Frame"', text, path)
            self.assertIn("tools/car_run/depth_probe.py", text, path)

    def test_probe_compiles(self):
        py_compile.compile(str(PROBE), cfile=os.path.join(tempfile.mkdtemp(), "p.pyc"), doraise=True)


# 10-15 (board-fixes 38ae200 wired in): rig_inner.sh's camera-graph snapshot is tools/integ_car_chain/graph_snapshot.py
# (polled until DDS discovery converged) instead of one `ros2 topic info -v` per topic. The block is cut out of
# rig_inner.sh and EXECUTED under set -u with the real graph_snapshot.py and a stand-in `ros2` on PATH (answers per
# topic from a schedule: "Unknown topic" first, the converged section afterwards). Known answers:
# 1. no runner (RT_INFO unset): expect none, ready on the first attempt when the three publishers are there (a
#    session without a runner is not slowed down); header = the real camera frame and the driver/launch pids.
# 2. runner reliable (RT_INFO -> runtime_info.json camera_qos reliable:5): one unconverged round (colour image
#    "Unknown topic", as v8rt r2/b2), then ready on attempt 2; runner_qos_readout.graph_check accepts it.
# 3. never converging: timed out after GRAPH_WAIT_S, the block still exits 0 and the wait record says timed_out.
# Control: the same harness with REPLAY left undefined fails (set -u is really on in the cut block).
GID_PUB = "01.0f.b9.5b.c6.00.ea.a2.00.00.00.00.00.00.81.03.00.00.00.00.00.00.00.00"
GID_RUNNER = "01.0f.b9.5b.77.01.00.00.00.00.00.00.00.00.11.04.00.00.00.00.00.00.00.00"


def section(typ, runner_reliability=None):
    def ep(node, kind, gid, rel):
        return ("Node name: %s\nNode namespace: /\nTopic type: %s\nEndpoint type: %s\nGID: %s\nQoS profile:\n"
                "  Reliability: %s\n  Durability: VOLATILE\n\n" % (node, typ, kind, gid, rel))
    text = "Type: %s\n\nPublisher count: 1\n\n" % typ + ep("camera", "PUBLISHER", GID_PUB, "RELIABLE")
    if runner_reliability:
        text += "Subscription count: 1\n\n" + ep("_NODE_NAME_UNKNOWN_", "SUBSCRIPTION", GID_RUNNER, runner_reliability)
    else:
        text += "Subscription count: 0\n\n"
    return text


FAKE_ROS2 = r"""#!/bin/bash
# stand-in for `ros2 topic info -v TOPIC`: answer n of TOPIC = $GS_DIR/<slug>_<n>.txt (last one repeats)
[ "$1 $2 $3" = "topic info -v" ] || { echo "fake ros2: unexpected $*" >&2; exit 9; }
slug=$(echo "$4" | tr '/' '_')
c="$GS_DIR/count$slug"; n=$(( $(cat "$c" 2>/dev/null || echo 0) + 1 )); echo $n > "$c"
f="$GS_DIR/$slug""_$n.txt"; [ -f "$f" ] || f=$(ls "$GS_DIR/$slug"_*.txt | sort -t_ -k2 | tail -1)
cat "$f"
"""

TOPICS = ("/camera/depth/image_raw", "/camera/color/image_raw", "/camera/depth/camera_info")
TYPES = ("sensor_msgs/msg/Image", "sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo")


def snapshot_block(text):
    lines = text.splitlines()
    a = next(i for i, x in enumerate(lines) if x.startswith("  GS=()"))
    b = next(i for i in range(a, len(lines)) if "graph_snapshot.log" in lines[i])
    return "\n".join(lines[a:b + 1])


class GraphSnapshotWiring(unittest.TestCase):
    def run_block(self, answers, env_lines, text=None):
        """answers: per topic a list of section texts (call n gets answers[n-1], the last repeats)."""
        import sys
        t = tempfile.mkdtemp(prefix="gsnap_rig_")
        gs = Path(t, "gs"); gs.mkdir()
        for topic, seq in zip(TOPICS, answers):
            for n, sec in enumerate(seq, 1):
                Path(gs, "%s_%d.txt" % (topic.replace("/", "_"), n)).write_text(sec)
        bindir = Path(t, "bin"); bindir.mkdir()
        Path(bindir, "ros2").write_text(FAKE_ROS2)
        os.chmod(str(Path(bindir, "ros2")), 0o755)
        Path(bindir, "python3").symlink_to(sys.executable)      # this interpreter runs graph_snapshot.py (py3.8 too)
        Path(t, "camera_frame.txt").write_text("camera_color_optical_frame\n")
        script = "\n".join(["set -u", "OUT=%s" % t, "LIVE_DEPTH_TOPIC=%s" % TOPICS[0], "LIVE_COLOR_TOPIC=%s" % TOPICS[1],
                            "LIVE_INFO_TOPIC=%s" % TOPICS[2]] + env_lines
                           + [snapshot_block(text if text is not None else RIG.read_text()), 'echo "BLOCK_RC=$?"'])
        env = dict(os.environ, GS_DIR=str(gs), PATH="%s:%s" % (bindir, os.environ.get("PATH", "")),
                   PYTHONPATH=str(ROOT))
        env.pop("RT_INFO", None)
        r = subprocess.run(["bash", "-c", script], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
                           cwd=str(ROOT), universal_newlines=True, timeout=120)
        return Path(t), r

    @staticmethod
    def converged(runner=None):
        return [[section(ty, runner)] for ty in TYPES]

    def wait(self, t):
        import json
        return json.loads(Path(t, "camera_topics_wait.json").read_text())

    def test_no_runner_ready_first_attempt(self):
        t, r = self.run_block(self.converged(), ["REPLAY=13 LAUNCH=12"])
        self.assertIn("BLOCK_RC=0", r.stdout, r.stdout)
        w = self.wait(t)
        self.assertEqual((w["ready"], w["timed_out"], w["attempts"], w["expect"]), (True, False, 1, "none"), w)
        self.assertEqual(w["topics"], list(TOPICS))
        snap = Path(t, "camera_topics.txt").read_text().splitlines()
        self.assertEqual(snap[0], "camera_frame=camera_color_optical_frame driver_pid=13 launch_pid=12")
        self.assertEqual(sum(1 for x in snap if x.startswith("Type: ")), 3)
        self.assertIn("graph snapshot ready", Path(t, "graph_snapshot.log").read_text())

    def test_runner_reliable_waits_for_discovery(self):
        import json
        ans = self.converged("RELIABLE")
        ans[1] = ["Unknown topic '%s'\n" % TOPICS[1], ans[1][0]]          # v8rt r2/b2: colour image not known yet
        rt = tempfile.mktemp(prefix="rtinfo_", suffix=".json")
        Path(rt).write_text(json.dumps({"camera_qos": "reliable:5"}))
        t, r = self.run_block(ans, ["REPLAY=13 LAUNCH=12", "RT_INFO=%s" % rt])
        self.assertIn("BLOCK_RC=0", r.stdout, r.stdout)
        w = self.wait(t)
        self.assertEqual((w["ready"], w["attempts"], w["expect"]), (True, 2, "reliable"), w)
        self.assertIn("camera_qos=reliable:5", w["expect_from"])
        from tools.integ_car_chain.runner_qos_readout import graph_check
        g = graph_check(str(t), "reliable:5")
        self.assertTrue(g["valid"], g["reasons"])
        # the same snapshot judged against a best-effort runner: invalid (the check can say no)
        self.assertFalse(graph_check(str(t), "sensor_data")["valid"])

    def test_never_converging_times_out_and_rig_goes_on(self):
        ans = [["Unknown topic '%s'\n" % tp] for tp in TOPICS]
        t, r = self.run_block(ans, ["REPLAY=13 LAUNCH=12", "GRAPH_WAIT_S=1.5"])
        self.assertIn("BLOCK_RC=0", r.stdout, r.stdout)
        w = self.wait(t)
        self.assertEqual((w["ready"], w["timed_out"]), (False, True), w)
        self.assertGreaterEqual(w["attempts"], 2)
        self.assertIn("TIMED OUT", Path(t, "graph_snapshot.log").read_text())

    def test_set_u_is_on_in_the_cut_block(self):
        t, r = self.run_block(self.converged(), ["LAUNCH=12"])         # control: REPLAY undefined
        self.assertNotIn("BLOCK_RC=0", r.stdout, r.stdout)
        self.assertIn("REPLAY", r.stdout)

    def test_placement_and_old_loop_gone(self):
        text = RIG.read_text()
        lines = text.splitlines()
        self.assertNotIn('ros2 topic info -v "$t"', text)
        at = next(i for i, x in enumerate(lines) if "tools/integ_car_chain/graph_snapshot.py" in x and "python3" in x)
        # after the driver pid (REPLAY) and the camera frame, before the producer (it subscribes RELIABLE in v8rt)
        self.assertLess(next(i for i, x in enumerate(lines) if x.strip().startswith("REPLAY=$(pgrep -P $LAUNCH")), at)
        self.assertLess(next(i for i, x in enumerate(lines) if 'camera_frame.txt" ] && break' in x), at)
        self.assertGreater(next(i for i, x in enumerate(lines) if "python3 -m host.ros_geo_producer" in x), at)
        # before cam_probe / rx_lite: their --duration-s (WARM + DUR + 22) counts from their start, a long wait here
        # would cut their tail inside the measurement window
        self.assertGreater(next(i for i, x in enumerate(lines) if "python3 tools/integ_car_chain/cam_probe.py" in x), at)
        self.assertGreater(next(i for i, x in enumerate(lines) if "python3 tools/integ_car_chain/rx_lite.py" in x), at)
        # only in the live branch
        live = next(i for i, x in enumerate(lines) if x.startswith("else") and i > lines.index('if [ "$CAMERA" = replay ]; then'))
        end = next(i for i in range(live, len(lines)) if lines[i] == "fi")
        self.assertTrue(live < at < end)
        inner = (ROOT / "tools/integ_car_chain/nano_chain_inner_v2.sh").read_text()
        self.assertIn('RT_INFO=""; [ "$RUNTIME" = 1 ] && RT_INFO="$OUT/runtime/runtime_info.json"', inner)
        self.assertIn('RT_INFO="$RT_INFO"', inner)
        # the runtime_info.json the snapshot reads is the one the caller writes before starting the rig
        self.assertLess(inner.index('--out "$OUT/runtime/runtime_info.json"'), inner.index("bash tools/car_geo_producer/rig_inner.sh"))

    def test_graph_snapshot_compiles(self):
        for f in ("tools/integ_car_chain/graph_snapshot.py", "tools/integ_car_chain/runner_qos_readout.py"):
            py_compile.compile(str(ROOT / f), cfile=os.path.join(tempfile.mkdtemp(), "g.pyc"), doraise=True)


if __name__ == "__main__":
    unittest.main()
