"""10-15: tools/integ_car_chain/graph_snapshot.py on an unparseable --runtime-info (e.g. runtime_info.json caught
mid-write by rig_inner.sh). Before: json.load raised and the poller exited with a traceback, leaving the rig with
neither rig/camera_topics.txt nor rig/camera_topics_wait.json. Now: the input is skipped and counted (one stderr
summary line; the wait record's expect_from says so), the expectation falls back to "none" (publishers only), the
snapshot and wait record keep their format, exit status 0; and the readout voids such an arm when the runner did
ask for RELIABLE (the expectation differs from its switch), so the fallback cannot pass a runner arm.

Positive cases: truncated JSON, a JSON array (not an object), undecodable bytes. Control: the same run on a valid
runtime_info.json says nothing on stderr and reads its camera_qos.
"""
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.integ_car_chain import graph_snapshot  # noqa: E402
from tools.integ_car_chain.runner_qos_readout import graph_check  # noqa: E402

V8RT = ROOT / "tests" / "data" / "camera_topics_v8rt_20261015"
TOPICS = ["/camera/depth/image_raw", "/camera/color/image_raw", "/camera/depth/camera_info"]
SCRIPT = ROOT / "tools" / "integ_car_chain" / "graph_snapshot.py"


class GraphSnapshotInvalidJson(unittest.TestCase):
    def setUp(self):
        self.d = Path(tempfile.mkdtemp(prefix="gsnap_bad_"))
        # r1: every topic with a publisher, the runner not on the colour image -> ready for "none", not "reliable"
        snap = self.d / "snap.txt"
        snap.write_text((V8RT / "r1.txt").read_text())
        fake = self.d / "fake_ros2.py"
        fake.write_text("import sys\nt = sys.argv[2]\nwhole = open(sys.argv[1]).read()\n"
                        "import re\nmarks = [m.start() for m in re.finditer(r'(?m)^(?:Type: |Unknown topic )', whole)]"
                        " + [len(whole)]\nsecs = [whole[a:b] for a, b in zip(marks, marks[1:])]\n"
                        "sys.stdout.write(secs[%r.index(t)])\n" % (TOPICS,))
        self.cmd = "%s %s %s" % (sys.executable, fake, snap)
        self.rig = self.d / "rig"
        self.rig.mkdir()
        self.rt = self.d / "runtime_info.json"

    def argv(self):
        a = []
        for t in TOPICS:
            a += ["--topic", t]
        return a + ["--out", str(self.rig / "camera_topics.txt"), "--wait-json", str(self.rig / "camera_topics_wait.json"),
                    "--runtime-info", str(self.rt), "--header", "camera_frame=cf driver_pid=1 launch_pid=2",
                    "--interval-s", "0", "--timeout-s", "2", "--cmd", self.cmd]

    def run_main(self):
        err = io.StringIO()
        with mock.patch("sys.stdout", new=io.StringIO()) as out, mock.patch("sys.stderr", new=err):
            rc = graph_snapshot.main(self.argv())
        return rc, out.getvalue(), err.getvalue()

    def check_skipped(self, rc, out, err):
        self.assertEqual(rc, 0)
        self.assertEqual(err.count("\n"), 1, err)
        self.assertIn("graph_snapshot: skipped 1 invalid JSON input(s): %s: invalid JSON" % self.rt, err)
        self.assertIn("graph snapshot ready", out)
        rec = json.loads((self.rig / "camera_topics_wait.json").read_text())
        self.assertEqual(sorted(rec), sorted(["ready", "timed_out", "waited_s", "attempts", "expect", "expect_from",
                                              "unmet", "topics"]))         # same record keys as before
        self.assertEqual((rec["ready"], rec["timed_out"], rec["expect"]), (True, False, "none"))
        self.assertIn("invalid JSON", rec["expect_from"])
        self.assertTrue((self.rig / "camera_topics.txt").read_text().startswith("camera_frame=cf driver_pid=1"))
        # the fallback never passes a runner arm: the runner asked RELIABLE, the snapshot waited for "none"
        self.assertFalse(graph_check(str(self.rig), "reliable:5")["valid"])

    def test_truncated_json_skipped_and_counted(self):
        self.rt.write_text('{"camera_qos": "reliable:5", "ca')
        self.check_skipped(*self.run_main())

    def test_json_array_skipped_and_counted(self):
        self.rt.write_text('["reliable:5"]')
        self.check_skipped(*self.run_main())

    def test_undecodable_bytes_skipped_and_counted(self):
        self.rt.write_bytes(b"\xff\xfe{\x00")
        self.check_skipped(*self.run_main())

    def test_cli_exit_zero_on_invalid_json(self):
        """The rig's own entry: a separate interpreter, exit status 0, both files written."""
        self.rt.write_text("{not json")
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        r = subprocess.run([sys.executable, str(SCRIPT)] + self.argv(), stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, env=env, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr.decode())
        self.assertNotIn(b"Traceback", r.stderr)
        self.assertIn(b"skipped 1 invalid JSON input(s)", r.stderr)
        self.assertTrue((self.rig / "camera_topics.txt").exists())
        self.assertEqual(json.loads((self.rig / "camera_topics_wait.json").read_text())["expect"], "none")

    def test_control_valid_json_says_nothing(self):
        self.rt.write_text(json.dumps({"camera_qos": "sensor_data"}))
        rc, out, err = self.run_main()
        self.assertEqual((rc, err), (0, ""))
        rec = json.loads((self.rig / "camera_topics_wait.json").read_text())
        self.assertEqual(rec["expect"], "best_effort")
        self.assertIn("camera_qos=sensor_data", rec["expect_from"])
        self.assertNotIn("invalid", rec["expect_from"])


if __name__ == "__main__":
    unittest.main()
