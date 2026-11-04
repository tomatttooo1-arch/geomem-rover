"""Integration v6 (2026-10-12): the runner's --tf-retry reaches the runner from the board scripts.

1. CLI (main()'s own argparse, captured before ROS is imported): --tf-retry defaults to arrival;
   "--tf-retry poll" and the one-token form "--tf-retry=poll" (nano_v3.sh env values cannot contain
   spaces) both give poll; another value is refused.
2. nano_v3.sh with env "STALL_DIAG=1 RUNTIME_EXTRA=--tf-retry=poll" (stub docker, fake inner script,
   the same dry-run harness as test_integ_v5_source_scripts) hands the token to the inner script.
3. session_v3.sh: v6 mode exists, its poll arm uses that one-token env, both arms STALL_DIAG=1; parses.
Python 3.8+.
"""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import test_integ_v5_source_scripts as v5  # noqa: E402
from test_runner_tf_qos import parsed_args  # noqa: E402

SESSION = ROOT / "docs/design/integ_car_chain_v3_20261011/session_v3.sh"
BASE = ["--socket", "/tmp/x.sock", "--pose-source", "reference", "--sensor-source", "replay"]


class Cli(unittest.TestCase):
    def test_tf_retry_values(self):
        self.assertEqual(parsed_args(BASE).tf_retry, "arrival")
        self.assertEqual(parsed_args(BASE + ["--tf-retry", "poll"]).tf_retry, "poll")
        self.assertEqual(parsed_args(BASE + ["--tf-retry=poll"]).tf_retry, "poll")
        with self.assertRaises(SystemExit):
            parsed_args(BASE + ["--tf-retry", "never"])


class BoardPassThrough(unittest.TestCase):
    # the dry-run harness of test_integ_v5_source_scripts (methods borrowed, its tests not re-run)
    setUp = v5.BoardScriptPassThrough.setUp
    tearDown = v5.BoardScriptPassThrough.tearDown
    _script = v5.BoardScriptPassThrough._script
    _run = v5.BoardScriptPassThrough._run
    _inner = v5.BoardScriptPassThrough._inner

    def test_nano_v3_tf_retry_token(self):
        s = self._script(v5.NANO_V3.read_text(), "nano_v3.sh")
        rc, out = self._run(s, ["chain", "t1", "10", "5", "-1", "1", "1", "1", "",
                                "STALL_DIAG=1 RUNTIME_EXTRA=--tf-retry=poll"])
        self.assertEqual(rc, 0, out)
        self.assertEqual(self._inner(out), "INNER RUNTIME_EXTRA=<--tf-retry=poll> STALL_DIAG=<1> PRODUCER_EXTRA=<>")


class Session(unittest.TestCase):
    def test_v6_mode(self):
        src = SESSION.read_text()
        # v6 is in the helper-sha mode list (later versions may append themselves: v7 on 10-12)
        self.assertRegex(src, r'case "\$MODE" in v4\|v5\|v6(\|v[0-9]+)*\)')
        self.assertIn('elif [ "$MODE" = v5 ] || [ "$MODE" = v6 ]; then', src)
        arms = [ln for ln in src.splitlines() if "trial r${T}_" in ln]
        self.assertEqual(len(arms), 2, arms)
        self.assertTrue(arms[0].rstrip().endswith('"STALL_DIAG=1"'), arms[0])
        self.assertTrue(arms[1].rstrip().endswith('"STALL_DIAG=1 RUNTIME_EXTRA=--tf-retry=poll"'), arms[1])
        r = subprocess.run(["bash", "-n", str(SESSION)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(r.returncode, 0, r.stdout)


if __name__ == "__main__":
    unittest.main()
