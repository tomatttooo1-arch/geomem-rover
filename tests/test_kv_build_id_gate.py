"""kv_server --expect-build-id and ar_screen.sh start (2026-10-16, controller).

The AR screen ran on the single-lane A1 bitstream m7_pl_merge (0x07784f75) because nothing on the KV
side compared the loaded bitstream with the car chain's A2 m7_pl_merge_a2 (0x00da5041). Now kv_server
reads the sysinfo BUILD_ID itself (UIO + MAGIC, the block open_blocks already opens; no other PL
address) and with --expect-build-id refuses to start, before any core or device is opened, with the
reason and the fix. ar_screen.sh start always passes --expect-build-id 0x00da5041 and turns that
refusal into rc 3 REFUSED.

Known answers on a fake UIO tree (real kv_server process, real kv_board.sh / ar_screen.sh):
  * A2 + --expect-build-id 0x00da5041 -> serves; ready line build_id 0x00da5041; SIGTERM -> rc 0;
  * A1 / base / no sysinfo + the flag -> no ready line, rc != 0, both ids + app name + "Not starting";
  * without the flag: A1 serves (as before) and the ready line names it; without a sysinfo block the
    ready line keys are exactly the old ones;
  * ar_screen.sh start on A1 -> rc 3 REFUSED naming 0x07784f75 (positive control of the pass-through:
    the refusal can only come from the flag reaching kv_server); on A2 it gets past the BUILD_ID gate
    (the next gate, --roi-core pl without an m7_roi node, stops it on the host before any device).
Run: cd tests && PYTHONPATH=.. python3 -m unittest discover -s . -p test_kv_build_id_gate.py
"""
import json
import os
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = str(ROOT / "tools" / "kv_link" / "kv_server.py")
AR_SCREEN = str(ROOT / "tools" / "kv_link" / "ar_screen.sh")
A2, A1, BASE = 0x00DA5041, 0x07784F75, 0x0117BC0A


def _write(path, data):
    with open(path, "wb" if isinstance(data, bytes) else "w") as fp:
        fp.write(data)


def _read(path):
    if not os.path.exists(path):
        return ""
    with open(path) as fp:
        return fp.read()


def fake_sysfs(root, build_id):
    """A sysinfo UIO node with this BUILD_ID (None = no sysinfo node at all) -> (sysfs, dev)."""
    sysfs, dev = os.path.join(root, "sys"), os.path.join(root, "dev")
    os.makedirs(dev, exist_ok=True)
    os.makedirs(sysfs, exist_ok=True)
    if build_id is not None:
        d = os.path.join(sysfs, "uio0", "maps", "map0")
        os.makedirs(d)
        _write(os.path.join(sysfs, "uio0", "name"), "m7_sysinfo\n")
        _write(os.path.join(d, "addr"), "0xa0000000\n")
        _write(os.path.join(d, "size"), "0x10000\n")
        buf = bytearray(0x10000)
        for off, v in {0x00: 0x4D37504C, 0x04: 0x10000, 0x08: build_id, 0x10: 100000000,
                       0x20: 0x3D7}.items():
            struct.pack_into("<I", buf, off, v)
        _write(os.path.join(dev, "uio0"), bytes(buf))
    return sysfs, dev


def env():
    return dict(os.environ, PYTHONPATH=str(ROOT))


class ServerGate(unittest.TestCase):
    def tmp(self):
        t = tempfile.mkdtemp(prefix="bidgate_")
        self.addCleanup(shutil.rmtree, t, True)
        return t

    def serve(self, build_id, *extra):
        """Start kv_server --core null on the fake tree; (ready dict or None, rc, stderr)."""
        t = self.tmp()
        sysfs, dev = fake_sysfs(t, build_id)
        out = os.path.join(t, "out")
        p = subprocess.Popen([sys.executable, TOOL, "--core", "null", "--port", "0", "--bind", "127.0.0.1",
                              "--uio-sysfs", sysfs, "--uio-dev", dev, "--out", out] + list(extra),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env(),
                             universal_newlines=True)
        try:
            line = p.stdout.readline()
            ready = None
            while line:
                rec = json.loads(line)
                if "port" in rec:
                    ready = rec
                    break
                line = p.stdout.readline()
            if ready is not None:
                time.sleep(0.2)
                p.send_signal(signal.SIGTERM)
            _o, err = p.communicate(timeout=30)
        finally:
            if p.poll() is None:
                p.kill()
                p.wait()
        if ready is not None:
            self.assertTrue(os.path.exists(os.path.join(out, "summary.json")))
        return ready, p.returncode, err

    def test_a2_expected_serves(self):
        ready, rc, err = self.serve(A2, "--expect-build-id", "0x00da5041")
        self.assertIsNotNone(ready, err)
        self.assertEqual(ready["build_id"], "0x00da5041")
        self.assertEqual(rc, 0, err)

    def test_other_bitstreams_refused_with_reason(self):
        for bid, name in ((A1, "m7_pl_merge"), (BASE, "m7_pl")):
            ready, rc, err = self.serve(bid, "--expect-build-id", "0x00da5041")
            self.assertIsNone(ready, (hex(bid), err))
            self.assertNotEqual(rc, 0)
            self.assertIn("loaded bitstream BUILD_ID is %#010x (%s), not 0x00da5041 (m7_pl_merge_a2)"
                          % (bid, name), err)
            self.assertIn("xmutil loadapp m7_pl_merge_a2", err)
            self.assertIn("Not starting", err)

    def test_unidentified_refused(self):
        ready, rc, err = self.serve(None, "--expect-build-id", "0x00da5041")
        self.assertIsNone(ready, err)
        self.assertNotEqual(rc, 0)
        self.assertIn("cannot identify the loaded bitstream", err)
        self.assertIn("Not starting", err)

    def test_no_flag_as_before(self):
        ready, rc, err = self.serve(A1)
        self.assertIsNotNone(ready, err)
        self.assertEqual(ready["build_id"], "0x07784f75")      # reported, not judged
        self.assertEqual(rc, 0, err)
        ready, rc, err = self.serve(None)
        self.assertEqual(sorted(ready), ["core", "pid", "port", "rcvbuf"])   # the old ready line
        self.assertEqual(rc, 0, err)


class ArScreenStart(unittest.TestCase):
    def start(self, build_id):
        t = tempfile.mkdtemp(prefix="arstart_")
        self.addCleanup(shutil.rmtree, t, True)
        base = os.path.join(t, "base")
        os.makedirs(base)
        os.symlink(str(ROOT), os.path.join(base, "repo"))
        sysfs, dev = fake_sysfs(t, build_id)
        out = os.path.join(t, "out")
        e = dict(os.environ, AR_SCREEN_BASE=base, PATH="%s:%s" % (os.path.dirname(sys.executable),
                                                                  os.environ["PATH"]))
        r = subprocess.run(["bash", AR_SCREEN, "start", out, "1024x600", "replay",
                            "--uio-sysfs", sysfs, "--uio-dev", dev], env=e, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, universal_newlines=True, timeout=120)
        return r, _read(os.path.join(out, "server.err"))

    def test_a1_refused_rc3(self):
        r, err = self.start(A1)
        self.assertEqual(r.returncode, 3, (r.stdout, r.stderr, err))
        self.assertIn("REFUSED", r.stderr)
        self.assertIn("0x07784f75", r.stderr)
        self.assertIn("ar_screen.sh prep", r.stderr)

    def test_a2_passes_the_build_id_gate(self):
        r, err = self.start(A2)
        self.assertNotEqual(r.returncode, 0)            # the host has no m7_roi node / PL
        self.assertNotEqual(r.returncode, 3, r.stderr)
        self.assertNotIn("--expect-build-id", err)
        self.assertIn("--roi-core pl", err)             # stopped by the next gate, before any device


if __name__ == "__main__":
    unittest.main()
