"""10-25 nano_gateway.sh stop is synchronous: `stop; start` right after must really restart the supervisor
(10-22: stop returned while the supervisor sat in a foreground sleep, start said "already running", nothing came up).
No car stack: car.current has no READY, so the supervisor only idles; no bridge, no sockets."""
import os
import shutil
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "ui/geomem/tools/competition/nano_gateway.sh"


def pid_alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


@unittest.skipUnless(shutil.which("setsid"), "setsid not available")
class NanoGatewayStopTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="gw-test-")
        self.env = dict(os.environ, GEOMEM_GATEWAY_DIR=self.dir)

    def tearDown(self):
        self.gw("stop")
        shutil.rmtree(self.dir, ignore_errors=True)

    def gw(self, cmd, preexec_fn=None):
        return subprocess.run(["bash", str(SCRIPT), cmd], env=self.env, capture_output=True, text=True, timeout=60,
                              preexec_fn=preexec_fn)

    def supervisor_pid(self):
        return int(Path(self.dir, "supervisor.pid").read_text())

    def test_stop_then_start_restarts(self):
        first = self.gw("start")
        self.assertEqual(first.returncode, 0, first.stderr)
        old = self.supervisor_pid()
        time.sleep(0.5)                       # supervisor is now inside its 2 s idle nap
        t0 = time.monotonic()
        stop = self.gw("stop")
        self.assertIn("gateway stopped", stop.stdout)
        self.assertLess(time.monotonic() - t0, 5.0)
        self.assertFalse(pid_alive(old))
        self.assertFalse(Path(self.dir, "supervisor.pid").exists())
        again = self.gw("start")
        self.assertEqual(again.returncode, 0, again.stderr)
        self.assertNotIn("already running", again.stdout)
        new = self.supervisor_pid()
        self.assertNotEqual(new, old)
        self.assertTrue(pid_alive(new))

    def test_stop_when_started_with_sigterm_ignored(self):
        """10-26: the full suite started the gateway from a process whose SIGTERM was ignored (inherited by
        bash, which cannot trap it); stop then waited 15 s and fell back to SIGKILL. start must hand the
        supervisor default signal dispositions and an empty mask, so the TERM trap runs."""
        def ignore_and_block():
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.pthread_sigmask(signal.SIG_BLOCK, [signal.SIGTERM])
        first = self.gw("start", preexec_fn=ignore_and_block)
        self.assertEqual(first.returncode, 0, first.stderr)
        old = self.supervisor_pid()
        time.sleep(0.5)
        t0 = time.monotonic()
        stop = self.gw("stop")
        self.assertIn("gateway stopped", stop.stdout)
        self.assertLess(time.monotonic() - t0, 5.0)
        self.assertFalse(pid_alive(old))
        self.assertNotIn("SIGKILL", Path(self.dir, "supervisor.log").read_text())

    def test_stop_when_not_running(self):
        self.assertIn("gateway not running", self.gw("stop").stdout)


if __name__ == "__main__":
    unittest.main()
