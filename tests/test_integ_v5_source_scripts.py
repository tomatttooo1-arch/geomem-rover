"""Integration v5 (2026-10-12): one data source for the replay rig, and RUNTIME_EXTRA through the
board scripts.

1. Runner --sensor-source gains ``replay``: same clock rules as hardware (wall time, use_sim_time
   refused), only the provenance tag differs; the hub maps it to replay. Old-behaviour control: the
   b2114c2 runner's choices refuse ``replay`` and its clock helper raises for it.
2. tools/integ_car_chain/nano_chain_inner_v2.sh gives the runner, the hub and the producer the same
   source from one variable (replay); with it the hub's frame-level source is ``replay``
   (old: runner hardware -> inconsistent).
3. RUNTIME_EXTRA reaches the container command: nano_ag.sh (environment) and nano_v3.sh (env
   argument, whitelist) run with a stub ``docker`` that prints its argv; the container's -c string is
   then run against a fake repo whose inner script prints what it got. Old-behaviour controls:
   d9e0490 nano_ag.sh drops it, b2114c2 nano_v3.sh refuses the env.
Python 3.8+, no ROS.
"""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host import ros_runtime  # noqa: E402
from host.hub.hub_core import HubCore  # noqa: E402

INNER = ROOT / "tools/integ_car_chain/nano_chain_inner_v2.sh"
NANO_AG = ROOT / "docs/design/runner_age_gate_20261011/nano_ag.sh"
NANO_V3 = ROOT / "docs/design/integ_car_chain_v3_20261011/nano_v3.sh"


def git_show(rev, path):
    if shutil.which("git") is None:
        return None
    r = subprocess.run(["git", "-C", str(ROOT), "show", "%s:%s" % (rev, path)],
                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    return r.stdout.decode() if r.returncode == 0 else None


class _Parsed(Exception):
    pass


def parsed_args(argv):
    real = argparse.ArgumentParser.parse_known_args
    box = {}

    def capture(self, args=None, namespace=None):
        box["r"] = real(self, args, namespace)
        raise _Parsed()
    with mock.patch.object(argparse.ArgumentParser, "parse_known_args", capture):
        try:
            ros_runtime.main(argv)
        except _Parsed:
            return box["r"][0]
    raise AssertionError("main() did not parse")


class RunnerReplaySource(unittest.TestCase):
    BASE = ["--socket", "/tmp/x.sock", "--pose-source", "reference", "--sensor-source"]

    def test_clock_rules_of_replay_equal_hardware(self):
        for src in ("hardware", "replay"):
            self.assertFalse(ros_runtime.use_simulation_clock(src, False))
            with self.assertRaises(ValueError):
                ros_runtime.use_simulation_clock(src, True)
        self.assertTrue(ros_runtime.use_simulation_clock("simulation", False))
        with self.assertRaises(ValueError):
            ros_runtime.use_simulation_clock("sim", False)

    def test_cli_accepts_replay(self):
        self.assertEqual(parsed_args(self.BASE + ["replay"]).sensor_source, "replay")
        self.assertEqual(parsed_args(self.BASE + ["hardware"]).sensor_source, "hardware")

    def test_old_runner_refused_replay(self):
        old = git_show("b2114c2", "host/ros_runtime.py")
        if old is None:
            self.skipTest("no git history")
        m = re.search(r'"--sensor-source", choices=\(([^)]*)\)', old)
        self.assertIsNotNone(m)
        self.assertNotIn("replay", m.group(1))
        src = re.search(r"def use_simulation_clock.*?\n    return False\n", old, re.S).group(0)
        ns = {}
        exec(src, ns)
        with self.assertRaises(ValueError):
            ns["use_simulation_clock"]("replay", False)

    def test_hub_maps_replay_runner_to_consistent(self):
        core = HubCore("replay")
        core.feed_status('{"event": "startup", "sensor_source": "replay"}', 1)
        self.assertEqual((core.source_state, core.frame_source()), ("consistent", "replay"))
        old = HubCore("replay")                      # control: the v4 rig's runner said hardware
        old.feed_status('{"event": "startup", "sensor_source": "hardware"}', 1)
        self.assertEqual(old.frame_source(), "inconsistent")


class InnerScriptOneSource(unittest.TestCase):
    def _launch_lines(self, text):
        hub = re.search(r"host\.hub\.hub_ros --data-source (\S+)", text).group(1)
        rt = re.search(r"host\.ros_runtime .*?--sensor-source (\S+)", text, re.S).group(1)
        rig = re.search(r'OUT="\$OUT/rig"(.*?)bash tools/car_geo_producer/rig_inner.sh', text, re.S).group(1)
        m = re.search(r'DATA_SOURCE="?([^"\s]+)"?', rig)
        return hub, rt, (m.group(1) if m else None)

    def test_runner_hub_producer_one_variable(self):
        text = INNER.read_text()
        self.assertRegex(text, r"(?m)^SRC=replay$")
        # 2026-10-15 (runner on the real camera): the runner's word for the source is RT_SRC, set in the same two
        # places as SRC (replay -> replay; CAMERA=live -> SRC=live, RT_SRC=hardware, the hub's live)
        self.assertRegex(text, r"(?m)^RT_SRC=replay; RT_LIVE=\(\)$")
        self.assertRegex(text, r"(?m)^  SRC=live; RT_SRC=hardware$")
        self.assertEqual(self._launch_lines(text), ('"$SRC"', '"$RT_SRC"', "$SRC"))

    def test_old_inner_mixed_sources(self):
        old = git_show("b2114c2", "tools/integ_car_chain/nano_chain_inner_v2.sh")
        if old is None:
            self.skipTest("no git history")
        self.assertEqual(self._launch_lines(old), ("replay", "hardware", None))


STUB_DOCKER = "#!/bin/bash\nfor a in \"$@\"; do printf 'ARG<%s>\\n' \"$a\"; done\n"
FAKE_INNER = ('echo "INNER RUNTIME_EXTRA=<${RUNTIME_EXTRA-UNSET}> STALL_DIAG=<${STALL_DIAG-UNSET}> '
              'PRODUCER_EXTRA=<${PRODUCER_EXTRA-UNSET}>"\n')


@unittest.skipIf(shutil.which("bash") is None or shutil.which("awk") is None, "needs bash")
class BoardScriptPassThrough(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="v5_dry_"))
        (self.tmp / "bin").mkdir()
        (self.tmp / "bin/docker").write_text(STUB_DOCKER)
        (self.tmp / "bin/docker").chmod(0o755)
        (self.tmp / "nb/runs").mkdir(parents=True)
        (self.tmp / "repo/tools/integ_car_chain").mkdir(parents=True)
        (self.tmp / "repo/tools/integ_car_chain/nano_chain_inner_v2.sh").write_text(FAKE_INNER)

    def tearDown(self):
        shutil.rmtree(str(self.tmp), ignore_errors=True)

    def _script(self, text, name):
        p = self.tmp / name
        p.write_text(re.sub(r"(?m)^NB=.*$", "NB=%s" % (self.tmp / "nb"), text))
        return p

    def _run(self, script, args, env_extra=None):
        env = dict(os.environ, PATH="%s:%s" % (self.tmp / "bin", os.environ.get("PATH", "")))
        env.pop("RUNTIME_EXTRA", None)
        env.pop("STALL_DIAG", None)
        env.update(env_extra or {})
        r = subprocess.run(["bash", str(script)] + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           env=env)
        return r.returncode, r.stdout.decode()

    def _inner(self, docker_out):
        """Run the container -c string (paths mapped into the temp dir) -> the fake inner's line."""
        lines = docker_out.splitlines()
        c = lines[lines.index("ARG<-c>") + 1][4:-1]
        c = c.replace("/repo", str(self.tmp / "repo")).replace("/w/", str(self.tmp / "nb") + "/")
        subprocess.run(["bash", "-c", c], check=True)
        logs = sorted((self.tmp / "nb/runs").glob("*.log"))
        out = logs[-1].read_text().strip()
        for f in (self.tmp / "nb/runs").iterdir():
            f.unlink()
        return out

    def test_nano_ag_passes_runtime_extra(self):
        s = self._script(NANO_AG.read_text(), "nano_ag.sh")
        rc, out = self._run(s, ["chain", "t1", "10", "5", "-1", "1", "1", "1", ""],
                            {"RUNTIME_EXTRA": "--tf-reliable", "STALL_DIAG": "1"})
        self.assertEqual(rc, 0, out)
        self.assertEqual(self._inner(out), "INNER RUNTIME_EXTRA=<--tf-reliable> STALL_DIAG=<1> PRODUCER_EXTRA=<>")
        rc, out = self._run(s, ["chain", "t2", "10", "5", "-1", "1", "1", "1", ""])
        self.assertEqual(self._inner(out), "INNER RUNTIME_EXTRA=<> STALL_DIAG=<0> PRODUCER_EXTRA=<>")

    def test_old_nano_ag_dropped_it(self):
        old = git_show("d9e0490", "docs/design/runner_age_gate_20261011/nano_ag.sh")
        if old is None:
            self.skipTest("no git history")
        s = self._script(old, "old_nano_ag.sh")
        rc, out = self._run(s, ["chain", "t1", "10", "5", "-1", "1", "1", "1", ""],
                            {"RUNTIME_EXTRA": "--tf-reliable", "STALL_DIAG": "1"})
        self.assertEqual(self._inner(out), "INNER RUNTIME_EXTRA=<UNSET> STALL_DIAG=<1> PRODUCER_EXTRA=<>")

    def test_nano_v3_env_argument(self):
        s = self._script(NANO_V3.read_text(), "nano_v3.sh")
        rc, out = self._run(s, ["chain", "t1", "10", "5", "-1", "1", "1", "1", "",
                                "STALL_DIAG=1 RUNTIME_EXTRA=--tf-reliable FPGA_HUB_EXECUTOR=lean"])
        self.assertEqual(rc, 0, out)
        self.assertEqual(self._inner(out), "INNER RUNTIME_EXTRA=<--tf-reliable> STALL_DIAG=<1> PRODUCER_EXTRA=<>")
        rc, out = self._run(s, ["chain", "t2", "10", "5", "-1", "1", "1", "1", "", "FOO=1"])
        self.assertEqual((rc, out.strip()), (2, "bad env FOO=1"))

    def test_old_nano_v3_refused_it(self):
        old = git_show("b2114c2", "docs/design/integ_car_chain_v3_20261011/nano_v3.sh")
        if old is None:
            self.skipTest("no git history")
        s = self._script(old, "old_nano_v3.sh")
        rc, out = self._run(s, ["chain", "t1", "10", "5", "-1", "1", "1", "1", "", "RUNTIME_EXTRA=--tf-reliable"])
        self.assertEqual((rc, out.strip()), (2, "bad env RUNTIME_EXTRA=--tf-reliable"))

    def test_scripts_parse(self):
        for p in (INNER, NANO_AG, NANO_V3, ROOT / "docs/design/runner_age_gate_20261011/session_ag.sh",
                  ROOT / "docs/design/integ_car_chain_v3_20261011/session_v3.sh"):
            r = subprocess.run(["bash", "-n", str(p)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            self.assertEqual(r.returncode, 0, (p, r.stdout))


if __name__ == "__main__":
    unittest.main()
