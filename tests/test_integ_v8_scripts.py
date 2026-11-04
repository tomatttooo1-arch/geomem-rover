"""Integration v8 (2026-10-12): the image publisher's Fast DDS profile switch through the chain scripts.

1. nano_v3.sh accepts CAM_DDS_PROFILE= / INPUT_LOG= (stub docker, fake inner: the values arrive) and
   refuses RX_FREEZE= (host stress only) like any unknown name.
2. nano_chain_inner_v2.sh hands the three knobs to rig_inner.sh; rig_inner.sh's profile block is
   EXECUTED: only the command prefixed with CAM_ENV sees FASTRTPS_DEFAULT_PROFILES_FILE, empty = nobody
   (control arm), a missing file = exit 4; the replay camera is the only prefixed command.
3. tools/nano_ros2/fastdds_shm.xml: one shared-memory segment that holds many RGB+depth ticks.
4. session_v3.sh v8 mode: fix arms = control arm + CAM_DDS_PROFILE only; older modes untouched; bash -n.
5. tools/integ_car_chain/rx_freeze.py: the target is stopped for about MS, continued afterwards, and
   continued when the stressor is terminated in the middle of a freeze.
Python 3.8+.
"""
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import test_integ_v5_source_scripts as v5  # noqa: E402

SESSION = ROOT / "docs/design/integ_car_chain_v3_20261011/session_v3.sh"
INNER = ROOT / "tools/integ_car_chain/nano_chain_inner_v2.sh"
RIG = ROOT / "tools/car_geo_producer/rig_inner.sh"
XML = ROOT / "tools/nano_ros2/fastdds_shm.xml"
FREEZE = ROOT / "tools/integ_car_chain/rx_freeze.py"
FAKE_INNER = ('echo "INNER CAM_DDS_PROFILE=<${CAM_DDS_PROFILE-UNSET}> INPUT_LOG=<${INPUT_LOG-UNSET}> '
              'STALL_DIAG=<${STALL_DIAG-UNSET}> HZ=<${HZ-UNSET}>"\n')


class BoardPassThrough(unittest.TestCase):
    setUp = v5.BoardScriptPassThrough.setUp
    tearDown = v5.BoardScriptPassThrough.tearDown
    _script = v5.BoardScriptPassThrough._script
    _run = v5.BoardScriptPassThrough._run
    _inner = v5.BoardScriptPassThrough._inner

    def test_nano_v3_profile_and_input_log(self):
        (self.tmp / "repo/tools/integ_car_chain/nano_chain_inner_v2.sh").write_text(FAKE_INNER)
        s = self._script(v5.NANO_V3.read_text(), "nano_v3.sh")
        rc, out = self._run(s, ["chain", "t1", "10", "5", "-1", "1", "1", "1", "",
                                "STALL_DIAG=1 INPUT_LOG=1 CAM_DDS_PROFILE=tools/nano_ros2/fastdds_shm_localhost.xml"])
        self.assertEqual(rc, 0, out)
        self.assertEqual(self._inner(out),
                         "INNER CAM_DDS_PROFILE=<tools/nano_ros2/fastdds_shm_localhost.xml> INPUT_LOG=<1> STALL_DIAG=<1> HZ=<10>")
        rc, out = self._run(s, ["chain", "t2", "10", "5", "-1", "1", "1", "1", "", "STALL_DIAG=1 INPUT_LOG=1"])
        self.assertEqual(rc, 0, out)                                   # control arm: no profile at all
        self.assertEqual(self._inner(out), "INNER CAM_DDS_PROFILE=<UNSET> INPUT_LOG=<1> STALL_DIAG=<1> HZ=<10>")
        rc, out = self._run(s, ["chain", "t4", "10", "5", "-1", "1", "1", "1", "", "INPUT_LOG=1 HZ=20"])
        self.assertEqual(rc, 0, out)                                   # frame-rate sweep: overrides the 10
        self.assertEqual(self._inner(out), "INNER CAM_DDS_PROFILE=<UNSET> INPUT_LOG=<1> STALL_DIAG=<UNSET> HZ=<20>")
        rc, out = self._run(s, ["chain", "t3", "10", "5", "-1", "1", "1", "1", "", "RX_FREEZE=1.7:250"])
        self.assertEqual(rc, 2, out)                                   # the stress never reaches a board
        self.assertIn("bad env RX_FREEZE=1.7:250", out)


class RigProfile(unittest.TestCase):
    def block(self):
        lines = RIG.read_text().splitlines()
        a = lines.index("CAM_ENV=()")
        b = next(i for i in range(a, len(lines)) if lines[i] == "fi")
        return "\n".join(lines[a:b + 1])

    def run_block(self, profile):
        script = "set -u\n" + self.block() + "\n" + "\n".join([
            '"${CAM_ENV[@]}" bash -c \'echo "CAM=<${FASTRTPS_DEFAULT_PROFILES_FILE-UNSET}>"\'',
            'bash -c \'echo "OTHER=<${FASTRTPS_DEFAULT_PROFILES_FILE-UNSET}>"\''])
        env = dict((k, v) for k, v in os.environ.items() if k not in ("CAM_DDS_PROFILE", "FASTRTPS_DEFAULT_PROFILES_FILE"))
        if profile is not None:
            env["CAM_DDS_PROFILE"] = profile
        r = subprocess.run(["bash", "-c", script], stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env,
                           cwd=str(ROOT))
        return r.returncode, r.stdout.decode().split(), r.stderr.decode()

    def test_only_the_prefixed_command_gets_the_profile(self):
        rc, out, err = self.run_block("tools/nano_ros2/fastdds_shm.xml")       # relative to the repo root
        self.assertEqual((rc, out), (0, ["CAM=<%s>" % XML, "OTHER=<UNSET>"]), err)
        rc, out, err = self.run_block(str(XML))
        self.assertEqual((rc, out), (0, ["CAM=<%s>" % XML, "OTHER=<UNSET>"]), err)

    def test_empty_or_unset_is_the_default_for_everybody(self):
        for profile in ("", None):
            rc, out, err = self.run_block(profile)
            self.assertEqual((rc, out), (0, ["CAM=<UNSET>", "OTHER=<UNSET>"]), err)

    def test_missing_file_aborts(self):
        rc, out, err = self.run_block("tools/nano_ros2/no_such_profile.xml")
        self.assertEqual((rc, out), (4, []))
        self.assertIn("no such file", err)

    def test_the_replay_camera_is_the_only_prefixed_command(self):
        text = RIG.read_text()
        uses = [ln.strip() for ln in text.splitlines() if '"${CAM_ENV[@]}"' in ln]
        self.assertEqual(len(uses), 2, uses)                           # the two image publishers, nobody else
        self.assertTrue(uses[0].startswith('"${CAM_ENV[@]}" python3 tools/car_geo_producer/replay_camera.py '), uses)
        self.assertTrue(uses[1].startswith('"${CAM_ENV[@]}" ros2 launch orbbec_camera gemini2.launch.py '), uses)
        self.assertIn('[ "${INPUT_LOG:-0}" = 1 ] && EXP+=(--input-log "$OUT/input_log.csv")', text)
        # the effect check lands in env.txt
        self.assertIn("shm_segment_bytes_max=", text)
        self.assertIn("replay_env_profile=", text)

    def test_inner_hands_the_knobs_to_the_rig(self):
        text = INNER.read_text()
        self.assertIn('CAM_DDS_PROFILE="${CAM_DDS_PROFILE:-}" INPUT_LOG="${INPUT_LOG:-0}" RX_FREEZE="${RX_FREEZE:-}" '
                      'CAMERA="$CAMERA" \\\n  bash tools/car_geo_producer/rig_inner.sh', text)
        for p in (INNER, RIG, v5.NANO_V3):
            r = subprocess.run(["bash", "-n", str(p)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(r.returncode, 0, r.stdout)


class Profile(unittest.TestCase):
    def test_segment_holds_many_ticks(self):
        ns = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        root = ET.parse(str(XML)).getroot()
        shm = [t for t in root.iter("{%s}transport_descriptor" % ns["f"]) if t.find("f:type", ns).text == "SHM"]
        self.assertEqual(len(shm), 1)
        seg = int(shm[0].find("f:segment_size", ns).text)
        tick_replay = 640 * 360 * (2 + 3)            # depth 16UC1 + rgb8, the replay set
        tick_camera = 640 * 480 * (2 + 3)            # the Orbbec driver's stream
        self.assertGreater(tick_replay, 512 * 1024)                    # why the 512 KB default is too small
        self.assertGreaterEqual(seg // tick_camera, 20)                # 32 MB: >= 20 ticks (2 s at 10 Hz)
        part = root.find("f:participant", ns)
        self.assertEqual(part.get("is_default_profile"), "true")       # applies without code changes
        ids = [e.text for e in part.iter("{%s}transport_id" % ns["f"])]
        self.assertEqual(ids, ["fpga_shm", "fpga_udp"])                  # UDP kept: discovery and other hosts

    def test_localhost_variant_is_loopback_only(self):
        # board 2026-10-12: fastdds_shm.xml on the camera alone (UDP on all interfaces) while the other
        # participants are loopback-only (ROS_LOCALHOST_ONLY=1) -> the camera was never discovered, 0 frames.
        # The chain's profile restricts UDP to 127.0.0.1 and keeps the same 32 MB segment.
        ns = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        lo = ET.parse(str(ROOT / "tools/nano_ros2/fastdds_shm_localhost.xml")).getroot()
        kinds = {t.find("f:type", ns).text: t for t in lo.iter("{%s}transport_descriptor" % ns["f"])}
        self.assertEqual(sorted(kinds), ["SHM", "UDPv4"])
        self.assertEqual(int(kinds["SHM"].find("f:segment_size", ns).text), 33554432)
        self.assertEqual([a.text for a in kinds["UDPv4"].iter("{%s}address" % ns["f"])], ["127.0.0.1"])
        allif = ET.parse(str(XML)).getroot()
        self.assertEqual([a.text for a in allif.iter("{%s}address" % ns["f"])], [])     # the difference
        self.assertNotIn("--", re.sub(r"(?s)^.*?<!--(.*?)-->.*$", r"\1",
                                      (ROOT / "tools/nano_ros2/fastdds_shm_localhost.xml").read_text()))

    def test_well_formed_for_a_strict_parser(self):
        # 2026-10-12: the comment used to contain "--shm-size" ("--" is not allowed inside an XML comment).
        # Fast DDS (tinyxml2) took it anyway; a strict parser would drop the whole profile, i.e. silently
        # fall back to the 512 KB segment.
        ET.parse(str(XML))
        self.assertNotRegex(re.sub(r"(?s)^.*?<!--(.*?)-->.*$", r"\1", XML.read_text()), "--")


class Session(unittest.TestCase):
    def test_v8_mode(self):
        src = SESSION.read_text()
        gates = re.findall(r'(?m)^case "\$MODE" in ([^)]*)\) : "\$\{EXPECT_HELPER_SHA:=([0-9a-f]{64})\}"', src)
        self.assertEqual([g[0] for g in gates], ["v4|v5|v6|v7", "v8|v8cam|v8rt"])    # v8 / v8rt: the same helper gate
        self.assertEqual(gates[0][1], gates[1][1])
        self.assertIn("for arm in ${V8_ORDER:-f1 c1 f2 f3}; do", src)
        arms = dict(re.findall(r'(?m)^      ([fc])\*\) formal _v8\$arm "" "([^"]*)" ;;$', src))
        self.assertEqual(arms["c"], "STALL_DIAG=1 INPUT_LOG=1")
        self.assertEqual(arms["f"], arms["c"] + " CAM_DDS_PROFILE=tools/nano_ros2/fastdds_shm_localhost.xml")
        self.assertTrue((ROOT / "tools/nano_ros2/fastdds_shm_localhost.xml").is_file())
        sweep = [ln.strip() for ln in src.splitlines() if ln.strip().startswith("trial z${T}_hz$h ")]
        self.assertEqual(sweep, ['trial z${T}_hz$h ${V8_HZ_DUR:-120} 20 -1 1 1 1 "" 600 "%s HZ=$h"' % arms["f"]])
        self.assertIn('if [ -n "${V8_HZ-10 15 20}" ]; then', src)       # V8_HZ= (empty) skips the sweep
        # the older modes are untouched
        self.assertIn('formal _v7s "" "STALL_DIAG=1 RECORD_WRITE=sync GC_MODE=default"', src)
        self.assertIn('elif [ "$MODE" = v5 ] || [ "$MODE" = v6 ]; then', src)
        r = subprocess.run(["bash", "-n", str(SESSION)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(r.returncode, 0, r.stdout)


class LiveCamera(unittest.TestCase):
    """Real-camera mode: the same chain with the Orbbec driver as the image publisher."""
    setUp = v5.BoardScriptPassThrough.setUp
    tearDown = v5.BoardScriptPassThrough.tearDown
    _script = v5.BoardScriptPassThrough._script
    _run = v5.BoardScriptPassThrough._run

    def inner_src(self, env):
        """Run the whole source block (SRC=replay .. its fi) under set -u; return rc, SRC, RT_SRC, CAMERA, RT_LIVE."""
        lines = INNER.read_text().splitlines()
        a = lines.index("SRC=replay")
        b = next(i for i in range(a, len(lines)) if lines[i] == "fi")
        self.assertIn('if [ "$CAMERA" = live ]; then', lines[a:b + 1])     # the block is the camera branch
        script = ("set -u\n" + "\n".join(lines[a:b + 1]) +
                  '\necho "SRC=$SRC RT_SRC=$RT_SRC CAMERA=$CAMERA"\nprintf "%s\\n" "${RT_LIVE[@]+${RT_LIVE[@]}}"\n')
        drop = ("CAMERA", "RUNTIME", "OUT", "LIVE_CAMERA_FRAME", "LIVE_COLOR_TOPIC", "LIVE_DEPTH_TOPIC", "LIVE_INFO_TOPIC")
        e = dict((k, v) for k, v in os.environ.items() if k not in drop)
        e["OUT"] = str(self.tmp / "out")                                   # what the block refers to
        e.update(env)
        r = subprocess.run(["bash", "-c", script], stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=e)
        out = r.stdout.decode().splitlines()
        return r.returncode, (out[0] if out else ""), [x for x in out[1:] if x]

    def test_inner_source_follows_the_camera(self):
        out = str(self.tmp / "out")
        self.assertEqual(self.inner_src({"RUNTIME": "1"}), (0, "SRC=replay RT_SRC=replay CAMERA=replay", []))
        self.assertEqual(self.inner_src({"RUNTIME": "0", "CAMERA": "live"})[:2], (0, "SRC=live RT_SRC=hardware CAMERA=live"))
        # live + the runner (2026-10-15, runner camera-QoS arm): allowed; the runner reads the driver's topics
        rc, head, rt_live = self.inner_src({"RUNTIME": "1", "CAMERA": "live"})
        self.assertEqual((rc, head), (0, "SRC=live RT_SRC=hardware CAMERA=live"))
        self.assertEqual(rt_live, ["--camera-frame", "camera_color_optical_frame",
                                   "--input-log", out + "/runtime/rx_runner.csv",
                                   "--ros-args", "-r", "/fpga/rgb:=/camera/color/image_raw",
                                   "-r", "/fpga/depth:=/camera/depth/image_raw",
                                   "-r", "/fpga/camera_info:=/camera/depth/camera_info"])
        # an unknown CAMERA never turns the chain live here (rig_inner.sh rejects it: test_rig_live_branch)
        self.assertEqual(self.inner_src({"RUNTIME": "1", "CAMERA": "usb"}), (0, "SRC=replay RT_SRC=replay CAMERA=usb", []))

    def test_rig_live_branch(self):
        text = RIG.read_text()
        self.assertIn('[ "${CAMERA:-replay}" = live ] && source /opt/orbbec_ws/install/setup.bash\nset -u', text)
        self.assertIn("publish_tf:=false", text)                       # live_tf.py owns base_link -> camera frame
        self.assertIn("depth_registration:=true align_mode:=HW", text)
        self.assertIn('python3 -m host.ros_geo_producer "${EXP[@]}" "${LIVE[@]}" ', text)
        self.assertIn('LIVE=(--depth-topic "$LIVE_DEPTH_TOPIC" --info-topic "$LIVE_INFO_TOPIC" --camera-frame ', text)
        r = subprocess.run(["bash", "-c", 'CAMERA=usb OUT=/nonexistent/x PREP=p KV=fake ROS_DOMAIN_ID=1 bash "$0"', str(RIG)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if b"CAMERA must be" in r.stderr:                              # needs /opt/ros (inside the ROS image)
            self.assertEqual(r.returncode, 2)

    def test_nano_v3_livechain(self):
        (self.tmp / "bin/lsusb").write_text("#!/bin/bash\nexit 0\n")   # no camera on this machine
        (self.tmp / "bin/lsusb").chmod(0o755)
        s = self._script(v5.NANO_V3.read_text(), "nano_v3.sh")
        rc, out = self._run(s, ["livechain", "t1", "10", "5", "", "RX_FREEZE=1:100"])
        self.assertEqual(rc, 2, out)
        self.assertIn("bad env RX_FREEZE=1:100", out)
        rc, out = self._run(s, ["livechain", "t1", "10", "5", "", "INPUT_LOG=1"])
        self.assertEqual(rc, 3, out)                                   # refuses to start without the camera
        self.assertIn("no Orbbec camera on USB", out)
        self.assertNotIn("ARG<", out)                                  # docker was not called
        text = v5.NANO_V3.read_text()
        live = text[text.index("livechain)\n"):text.index("camargs)\n")]
        for token in ("$IMG_CAM", '--device "$BUSDEV"', "CAMERA=live", "RUNTIME=0", "HUB=1", "ESTOP_AT=-1",
                      "--name fpga-dev1-integ3-$name"):
            self.assertIn(token, live)

    def test_session_v8cam(self):
        src = SESSION.read_text()
        self.assertIn("for arm in ${V8CAM_ORDER:-c1 f1}; do", src)
        arms = dict(re.findall(r'(?m)^      ([fc])\*\) CE="([^"]*)" ;;$', src))
        self.assertEqual(arms["c"], "STALL_DIAG=1 INPUT_LOG=1")
        self.assertEqual(arms["f"], arms["c"] + " CAM_DDS_PROFILE=tools/nano_ros2/fastdds_shm_localhost.xml")
        self.assertIn('ct=$(acq camera)', src)
        self.assertIn('echo "camera $ct" >> $TOK', src)                # released by the EXIT trap like the others

    def test_cam_probe_known_answer(self):
        from tools.integ_car_chain.cam_probe import summarize
        period = 33333333
        full = [1000 + i * period for i in range(300)]
        out = summarize(full, [2.5] * 300)
        self.assertEqual((out["arrivals"], out["missed_frames"], out["steps_over_1p5_period"]), (300, 0, 0))
        self.assertAlmostEqual(out["rate_hz"], 30.0, places=2)
        self.assertEqual(out["stamp_to_arrival_ms"]["p50"], 2.5)
        holed = [st for i, st in enumerate(full) if i not in (40, 41, 170)]     # a 2-frame hole and a 1-frame hole
        out = summarize(holed)
        self.assertEqual((out["arrivals"], out["missed_frames"], out["steps_over_1p5_period"]), (297, 3, 2))
        self.assertEqual(out["gap_steps_ms"], [100.0, 66.7])
        self.assertEqual(summarize(full[:5]), {"arrivals": 5, "non_monotonic": 0})   # too few to say anything
        self.assertEqual(summarize([5, 5, 4] + full)["non_monotonic"], 2)

    def test_cam_probe_graph_is_read_while_alive(self):
        # board smoke 2026-10-13: the QoS was read once, on exit, after SIGINT had invalidated the context ->
        # RCLError, nothing on record. Now polled while running; a later failure or an empty list (the peer
        # has left) never erases the last non-empty reading.
        from collections import namedtuple
        from tools.integ_car_chain.cam_probe import GraphKeeper, endpoint_rows
        Q = namedtuple("Q", "reliability durability history depth")
        E = namedtuple("E", "node_name qos_profile")
        pub = E("camera", Q("ReliabilityPolicy.RELIABLE", "DurabilityPolicy.VOLATILE", "HistoryPolicy.UNKNOWN", 0))
        sub = E("fpga_geo_producer", Q("ReliabilityPolicy.BEST_EFFORT", "DurabilityPolicy.VOLATILE", "HistoryPolicy.UNKNOWN", 0))
        self.assertEqual(endpoint_rows([pub]), [{"node": "camera", "reliability": "RELIABLE", "durability": "VOLATILE",
                                                 "history": "UNKNOWN", "depth": 0}])
        script = {"mode": "up"}

        def read(kind, topic):
            if script["mode"] == "dead":
                raise RuntimeError("rcl node's context is invalid")
            if script["mode"] == "gone":
                return []
            return [pub] if kind == "publishers" else [sub]

        g = GraphKeeper(["/d", "/c"], read)
        self.assertEqual(g.result("/d")["publishers"], None)           # nothing read yet: says so, no invention
        g.poll()
        script["mode"] = "gone"
        g.poll()
        script["mode"] = "dead"
        g.poll()
        r = g.result("/d")
        self.assertEqual([x["node"] for x in r["publishers"]], ["camera"])
        self.assertEqual([(x["node"], x["reliability"]) for x in r["subscriptions"]], [("fpga_geo_producer", "BEST_EFFORT")])
        self.assertEqual((r["graph_reads"], r["graph_read_errors"]), (8, 4))
        src = (ROOT / "tools/integ_car_chain/cam_probe.py").read_text()
        self.assertNotIn("get_publishers_info_by_topic(t)]", src)       # the exit-time read is gone
        self.assertIn("graph.poll()", src)

    def test_live_tf_records_the_camera_info(self):
        from collections import namedtuple
        from tools.integ_car_chain.live_tf import info_row
        H = namedtuple("H", "frame_id")
        M = namedtuple("M", "header width height distortion_model k d p")
        k = [475.5, 0.0, 320.25, 0.0, 475.5, 240.5, 0.0, 0.0, 1.0]
        row = info_row(M(H("camera_color_optical_frame"), 640, 480, "rational_polynomial", k, [0.1, -0.2], k[:3] + [0.0] + k[3:6] + [0.0] + k[6:] + [0.0]))
        self.assertEqual((row["frame_id"], row["width"], row["height"], row["k"]), ("camera_color_optical_frame", 640, 480, k))
        self.assertEqual(len(row["p"]), 12)
        text = RIG.read_text()
        self.assertIn('--info-out "$OUT/camera_info.json" --extra-info-topic "$LIVE_COLOR_INFO_TOPIC"', text)
        self.assertIn(': "${LIVE_COLOR_INFO_TOPIC:=/camera/color/camera_info}"', text)
        src = (ROOT / "tools/integ_car_chain/live_tf.py").read_text()
        self.assertIn("except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):", src)


def state(pid):
    with open("/proc/%d/stat" % pid) as f:
        raw = f.read()
    return raw[raw.rindex(")") + 2]


class SessionLockTokens(unittest.TestCase):
    """session_v3.sh acq() EXECUTED against the real lock code (tools/admin/workspace.py ``lock``, what lock.sh
    execs) on a throw-away root (2026-10-15): our token on success; nothing on a refusal, whose text names the
    holder's token -- the old grep read that one as ours (positive control below)."""

    def setUp(self):
        import tempfile
        self.root = tempfile.mkdtemp(prefix="lockroot_")
        for d in (".workspace", "repo.git", "work", "shared", "locks"):
            os.makedirs(os.path.join(self.root, d))
        src = SESSION.read_text()
        m = re.search(r"(?ms)^acq\(\) \{.*?\| head -1; \}$", src)
        self.assertIsNotNone(m)
        self.acq = m.group(0)
        self.log = os.path.join(self.root, "session.log")
        self.lock = "%s %s --root %s lock" % (sys.executable, ROOT / "tools/admin/workspace.py", self.root)

    def bash(self, body):
        script = 'LOCK="%s"; WHY="test"; log() { echo "$*" >> %s; }\n%s\n%s\n' % (self.lock, self.log, self.acq, body)
        return subprocess.run(["bash", "-c", script], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True)

    def test_acquire_and_refusal(self):
        import json
        r = self.bash('t=$(acq camera); echo "T=$t"')
        tok = re.search(r"T=(\S*)", r.stdout).group(1)
        info = json.loads(Path(self.root, "locks/camera.lock.d/info.json").read_text())
        self.assertEqual(tok, info["token"])
        r = self.bash('t=$(acq camera); echo "T=<$t>"')
        self.assertIn("T=<>", r.stdout)                               # refused: no token
        self.assertIn("lock camera not acquired", Path(self.log).read_text())
        # positive control: the old line on the same refusal takes the holder's token
        old = self.bash('c=$($LOCK acquire camera "$WHY" -u dev1 2>&1); ct=$(echo "$c" | grep -o -E \'[0-9a-f]{32}\' '
                        '| tail -1); echo "T=<$ct>"')
        self.assertIn("T=<%s>" % info["token"], old.stdout)

    def test_session_uses_acq_everywhere(self):
        src = SESSION.read_text()
        self.assertEqual(len(re.findall(r"\$LOCK acquire", src)), 1)          # only inside acq()
        for line in ("kt=$(acq kv260)", "nt=$(acq orin-nano)", "ct=$(acq camera)"):
            self.assertIn(line, src)


class NanoRunDir(unittest.TestCase):
    """2026-10-15: the Nano run directory is on the NVMe /data (root fs 88 % full); session and board script agree;
    the chain's only Unix socket is in the container's /tmp, not under the run directory."""

    def test_same_nb_on_data(self):
        sess = re.search(r"(?m)^KB=\S+; NB=(\S+)$", SESSION.read_text()).group(1)
        nano = re.search(r"(?m)^NB=(\S+)", (ROOT / "docs/design/integ_car_chain_v3_20261011/nano_v3.sh").read_text()).group(1)
        self.assertEqual(sess, nano)
        self.assertEqual(sess, "/data/fpga/integ3")

    def test_socket_path_length(self):
        inner = INNER.read_text()
        socks = re.findall(r"(?m)^\s*SOCK=(\S+)$", inner)
        self.assertEqual(socks, ["/tmp/fpga_stub_$$.sock"])
        longest = socks[0].replace("$$", "4194304")             # pid_max on 64-bit Linux: 7 digits
        self.assertLessEqual(len(longest.encode()), 107)        # sun_path 108 bytes incl. the NUL
        self.assertEqual(len(longest), 26)


class RxFreeze(unittest.TestCase):
    def setUp(self):
        self.child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])

    def tearDown(self):
        try:
            os.kill(self.child.pid, signal.SIGCONT)
        except OSError:
            pass
        self.child.kill()
        self.child.wait()

    def test_freezes_then_continues(self):
        p = subprocess.Popen([sys.executable, str(FREEZE), "0.3", "150", "1.0", str(self.child.pid)],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        seen = set()
        while p.poll() is None:
            seen.add(state(self.child.pid))
            time.sleep(0.005)
        out, err = p.communicate()
        self.assertEqual(p.returncode, 0, err)
        rows = out.decode().split()
        self.assertEqual(rows[0], "stop_raw_ns,cont_raw_ns")
        spans = [(int(b) - int(a)) / 1e6 for a, b in (r.split(",") for r in rows[1:])]
        self.assertGreaterEqual(len(spans), 2)
        self.assertTrue(all(150 <= s < 400 for s in spans), spans)
        self.assertIn("T", seen)                                       # it really was stopped
        self.assertNotEqual(state(self.child.pid), "T")                # and is running again

    def test_terminated_mid_freeze_continues_the_target(self):
        p = subprocess.Popen([sys.executable, str(FREEZE), "1.0", "900", "30", str(self.child.pid)],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        deadline = time.monotonic() + 10
        while state(self.child.pid) != "T" and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertEqual(state(self.child.pid), "T")
        t_stop = time.monotonic()
        p.terminate()
        p.communicate()
        self.assertLess(time.monotonic() - t_stop, 0.8)                # ended inside the 900 ms freeze
        deadline = time.monotonic() + 5
        while state(self.child.pid) == "T" and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertNotEqual(state(self.child.pid), "T")

    def test_bad_arguments(self):
        r = subprocess.run([sys.executable, str(FREEZE), "0.1", "150", "1", str(self.child.pid)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertNotEqual(r.returncode, 0)                           # MS/1000 >= EVERY
        self.assertNotEqual(state(self.child.pid), "T")


if __name__ == "__main__":
    unittest.main()
