"""kv_server.py rcvbuf_warning (kv-timeout dig 2026-10-14).

A plain-user kv_server gets its receive buffer capped at rmem_max (416 KB on this host), less than
one 640x400 request (9 x 60000 B); the request tail overflows and the Nano side sees LinkTimeout.
The server now says so on stderr. W1: the pure function (capped -> warning, forced / large enough /
given socket -> none). W2: the CLI in this environment: the warning appears exactly when
SO_RCVBUFFORCE is not allowed here and the read-back is below --rcvbuf, with the same numbers as the
startup line; a small --rcvbuf (always satisfied) never warns (negative control); stdout keeps its
startup keys.
"""
import importlib.util
import json
import os
import socket
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def load_kv_server():
    spec = importlib.util.spec_from_file_location("kv_server_rcvbuf_test", str(ROOT / "tools/kv_link/kv_server.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def force_allowed():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, getattr(socket, "SO_RCVBUFFORCE", 33), 8 << 20)
        return True
    except OSError:
        return False
    finally:
        s.close()


class PureFunctionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.f = staticmethod(load_kv_server().rcvbuf_warning)

    def test_w1_capped_warns(self):
        w = self.f(("SO_RCVBUF(capped by rmem_max)", 425984), 8 << 20, 60000)
        self.assertIsNotNone(w)
        self.assertEqual(w["effective_bytes"], 425984)
        self.assertEqual(w["requested_bytes"], 8 << 20)
        self.assertEqual(w["request_640x400_bytes"], 9 * 60000)     # 512 000 B of codes -> 9 chunks
        self.assertGreater(w["request_640x400_bytes"], w["effective_bytes"])

    def test_w1_forced_or_large_enough_or_given_silent(self):
        self.assertIsNone(self.f(("SO_RCVBUFFORCE", 16777216), 8 << 20, 60000))
        self.assertIsNone(self.f(("SO_RCVBUF(capped by rmem_max)", 16777216), 8 << 20, 60000))
        self.assertIsNone(self.f(("given", None), 8 << 20, 60000))
        self.assertIsNone(self.f(("SO_RCVBUF(capped by rmem_max)", 425984), 200000, 60000))


class CliTests(unittest.TestCase):
    def _run(self, extra):
        tmp = tempfile.mkdtemp(prefix="kvrb_")
        p = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--core", "null",
                              "--bind", "127.0.0.1", "--port", "0", "--out", tmp,
                              "--uio-sysfs", os.path.join(tmp, "no_uio")] + extra,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                             env=dict(os.environ, PYTHONPATH=str(ROOT)))
        try:
            start = json.loads(p.stdout.readline())
        finally:
            p.terminate()
            try:
                _, err = p.communicate(timeout=10)
            except subprocess.TimeoutExpired:     # never leave a hung server behind
                p.kill()
                p.communicate()
                raise
        self.assertEqual(p.returncode, 0, err[-2000:])   # SIGTERM -> clean exit with summary.json
        self.assertTrue(os.path.exists(os.path.join(tmp, "summary.json")))
        warns = []
        for line in err.splitlines():
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if isinstance(rec, dict) and "rcvbuf_warning" in rec:
                warns.append(rec)
        return start, warns

    def test_w2_default_rcvbuf_matches_environment(self):
        start, warns = self._run([])
        self.assertEqual(sorted(start), ["core", "pid", "port", "rcvbuf"])
        method, effective = start["rcvbuf"]
        if force_allowed():
            self.assertEqual(method, "SO_RCVBUFFORCE")
            self.assertEqual(warns, [])
        elif effective < (8 << 20):
            self.assertEqual(len(warns), 1)
            self.assertEqual(warns[0]["effective_bytes"], effective)
            self.assertEqual(warns[0]["method"], method)
        else:                               # rmem_max raised on this machine: nothing to warn about
            self.assertEqual(warns, [])

    def test_w2_negative_control_small_request_never_warns(self):
        start, warns = self._run(["--rcvbuf", "4096"])
        self.assertGreaterEqual(start["rcvbuf"][1], 4096)
        self.assertEqual(warns, [])


if __name__ == "__main__":
    unittest.main()
