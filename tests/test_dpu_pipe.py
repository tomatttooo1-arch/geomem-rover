"""DPUPIPE0 (2026-11-03): colour -> DPU input path. host/kv_colour.py WANT_DPU / FRAME_DPU (types 44/45),
a ColourSource serving several receivers at once, and the C++ KV receiver tools/dpu_pipe (frame ring
unit test + loopback end to end with byte-exact tensors).
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_dpu_pipe.py
"""
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import zlib
from collections import Counter

import numpy as np

from host import dpu_prep, kv_colour
from host.kv_link import CHUNK_MTU, Reassembler, set_rcvbuf, split

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIPE = os.path.join(ROOT, "tools", "dpu_pipe")


def frame(k, w=640, h=480):
    y, x = np.mgrid[0:h, 0:w]
    return np.stack([(x + 5 * k) % 256, (y + 3 * k) % 256, ((x ^ y) + k) % 256], -1).astype(np.uint8)


class PyDpuReceiver:
    """Minimal Python WANT_DPU client (the test's stand-in; the product receiver is tools/dpu_pipe)."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        set_rcvbuf(self.sock, 8 << 20)   # capped by rmem_max (416 KB on the build host): a burst can drop
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.counts = Counter()
        self.rx = Reassembler(self.counts)
        self.src = None
        self.want_id = 0

    def wait_hello(self, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                data, addr = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            got = self.rx.feed(data)
            if got and got[0] == kv_colour.CHELLO:
                self.src = addr
                self.rx.forget_done(kv_colour.CHELLO)
                return json.loads(got[4].decode())
        raise AssertionError("no HELLO")

    def get(self, size, fp, after_key=0, wait_ms=300, chunk=60000, timeout=1.5, tries=4):
        """Loopback without pacing can overflow a 416 KB receive buffer (KV_LINK_20261011.md): re-ask."""
        for _ in range(tries - 1):
            try:
                return self._get(size, fp, after_key, wait_ms, chunk, timeout)
            except AssertionError:
                self.lost = getattr(self, "lost", 0) + 1
        return self._get(size, fp, after_key, wait_ms, chunk, timeout)

    def _get(self, size, fp, after_key, wait_ms, chunk, timeout):
        self.want_id += 1
        for d in split(kv_colour.CWANT_DPU, 77, self.want_id,
                       kv_colour.encode_want_dpu(size, fp, after_key, wait_ms, chunk), CHUNK_MTU):
            self.sock.sendto(d, self.src)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                data, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            got = self.rx.feed(data)
            if not got or got[3] != self.want_id:
                continue
            if got[0] == kv_colour.CFRAME_DPU:
                return kv_colour.decode_frame_dpu(got[4])
            if got[0] == kv_colour.CNONE:
                return {"none": kv_colour.NONE_REASONS[kv_colour.NONE_BODY.unpack_from(got[4])[1]]}
        raise AssertionError("no answer")

    def close(self):
        self.sock.close()


class Protocol(unittest.TestCase):
    def test_want_roundtrip_and_limits(self):
        b = kv_colour.encode_want_dpu(512, 7, 123, 250, 1400, kv_t_ns=9)
        self.assertEqual(len(b), 32)
        d = kv_colour.decode_want_dpu(b)
        self.assertEqual((d["size"], d["fix_point"], d["after_key"], d["wait_ms"], d["chunk_bytes"], d["kv_t_ns"]),
                         (512, 7, 123, 250, 1400, 9))
        self.assertEqual(kv_colour.decode_want_dpu(kv_colour.encode_want_dpu(640, 6, 0, 99999, 60000))["wait_ms"],
                         kv_colour.DPU_WAIT_MAX_MS)
        with self.assertRaises(ValueError):
            kv_colour.decode_want_dpu(kv_colour.encode_want_dpu(600, 6, 0, 10, 60000))
        with self.assertRaises(ValueError):
            kv_colour.decode_want_dpu(kv_colour.encode_want_dpu(640, 6, 0, 10, 70000))

    def test_frame_roundtrip_and_crc(self):
        p = dpu_prep.DpuPrep(416)
        g, content, be = p.content(frame(1))
        b = kv_colour.encode_frame_dpu(g, content, fix_point=7, pad_q=p.pad_q, backend=be, source_key=5,
                                       after_key=4, kv_t_ns=3, prep_ns=2, hold_ns=1, seq=9)
        self.assertEqual(len(b), 64 + 416 * 312 * 3)
        d = kv_colour.decode_frame_dpu(b)
        self.assertEqual((d["w"], d["h"], d["top"], d["left"], d["pad_q"], d["seq"]), (416, 312, 52, 0, 57, 9))
        self.assertEqual(d["content"], content)
        bad = bytearray(b)
        bad[100] ^= 1
        with self.assertRaises(ValueError):
            kv_colour.decode_frame_dpu(bytes(bad))


class MultiReceiver(unittest.TestCase):
    def test_raw_sink_and_dpu_receiver_served_together(self):
        ring = kv_colour.ColourRing(6)
        sink = kv_colour.ColourSink(("127.0.0.1", 0), rcvbuf=1 << 20, tol_ns=5_000_000).start()
        dpu = PyDpuReceiver()
        src = kv_colour.ColourSource([("127.0.0.1", sink.port), ("127.0.0.1", dpu.port)], ring.lookup,
                                     chunk_bytes=8000, hello_s=0.05, latest=ring.newest, dpu_backend="numpy")
        try:
            for k in range(3):
                ring.put(1000 + k, 640, 480, frame(k).tobytes())
            src.start()
            info = dpu.wait_hello()
            self.assertTrue(info["dpu"])
            end = time.monotonic() + 3
            while not sink.has_source() and time.monotonic() < end:
                time.sleep(0.02)
            # raw colour to the screen sink (old path, unchanged)
            self.assertTrue(sink.want(7, 1001, 1001))
            res, why = sink.take(7, 1001, 2.0)
            self.assertIsNone(why)
            self.assertEqual(res["rgb"], frame(1).tobytes())
            # DPU bytes to the second receiver: newest frame, identical to DpuPrep
            for size, fp in ((640, 6), (512, 7), (416, 7)):
                d = dpu.get(size, fp, chunk=1400 if size == 416 else 60000)
                self.assertEqual(d["source_key"], 1002)
                g, want, _ = dpu_prep.DpuPrep(size, fp, "numpy").content(frame(2))
                self.assertEqual(d["content"], want)
                self.assertEqual((d["top"], d["size"]), (g.top, size))
            # long poll: nothing newer than 1002 -> NONE(no_new) after wait_ms; a frame arriving in the
            # window is answered at once
            t0 = time.monotonic()
            self.assertEqual(dpu.get(416, 7, after_key=1002, wait_ms=150), {"none": "no_new"})
            self.assertGreater(time.monotonic() - t0, 0.12)
            import threading
            threading.Timer(0.05, lambda: ring.put(1003, 640, 480, frame(3).tobytes())).start()
            d = dpu.get(416, 7, after_key=1002, wait_ms=1000)
            self.assertEqual(d["source_key"], 1003)
            self.assertGreaterEqual(d["hold_ns"], 20_000_000)
            self.assertEqual(src.counts["dpu_frames_sent"], 4 + getattr(dpu, "lost", 0))
        finally:
            src.stop()
            sink.stop()
            dpu.close()

    def test_dpu_off_answers_none(self):
        ring = kv_colour.ColourRing(2)
        dpu = PyDpuReceiver()
        src = kv_colour.ColourSource(("127.0.0.1", dpu.port), ring.lookup, hello_s=0.05)   # no latest
        try:
            src.start()
            dpu.wait_hello()
            self.assertEqual(dpu.get(640, 6), {"none": "error"})
        finally:
            src.stop()
            dpu.close()


@unittest.skipIf(shutil.which("g++") is None or shutil.which("make") is None, "no C++ toolchain")
class CppReceiver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.mkdtemp(prefix="dpupipe_")
        subprocess.run(["make", "-s", "-C", PIPE, "OUT=" + cls.build], check=True)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.build, ignore_errors=True)

    def test_ring_and_geometry(self):
        out = subprocess.run([os.path.join(self.build, "test_dpu_ring")], capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertIn('"pass"', out.stdout)

    def _e2e(self, size, fp, chunk):
        ring = kv_colour.ColourRing(12)
        port = 48000 + (os.getpid() + size + chunk) % 1000
        inf = os.path.join(self.build, "inf_%d_%d.jsonl" % (size, chunk))
        rx = subprocess.Popen([os.path.join(self.build, "dpu_rx"), "--port", str(port), "--size", str(size),
                               "--fix", str(fp), "--chunk", str(chunk), "--infer-ms", "30", "--seconds", "2.5",
                               "--rcvbuf", str(4 << 20), "--infer-log", inf, "--tensor-crc", "--crc"],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        src = kv_colour.ColourSource(("127.0.0.1", port), ring.lookup, hello_s=0.05, latest=ring.newest,
                                     dpu_backend="numpy")
        keys = {}
        frames = [frame(i).tobytes() for i in range(4)]
        try:
            src.warm_dpu(size, fp)
            src.start()
            t_end = time.monotonic() + 3.0
            k = 0
            while time.monotonic() < t_end and rx.poll() is None:
                key = time.monotonic_ns()
                keys[key] = k % 4
                ring.put(key, 640, 480, frames[k % 4])
                k += 1
                time.sleep(1 / 15)
            out, err = rx.communicate(timeout=10)
        finally:
            src.stop()
            if rx.poll() is None:
                rx.terminate()
        summary = json.loads(out.strip().splitlines()[-1])
        prep = dpu_prep.DpuPrep(size, fp, "numpy")
        want = {i: zlib.crc32(prep.full_tensor(frame(i)).tobytes()) & 0xFFFFFFFF for i in range(4)}
        rows = [json.loads(line) for line in open(inf)]
        self.assertGreaterEqual(len(rows), 15, summary)
        for r in rows:
            self.assertEqual(r["tensor_crc"], want[keys[r["source_key"]]])   # whole tensor, pad included
        self.assertEqual(summary["rx"]["crc_bad"], 0)
        self.assertEqual(summary["rx"]["bad_head"], 0)
        return summary

    def test_e2e_fragmented_640(self):
        s = self._e2e(640, 6, 60000)
        self.assertTrue(s["rx"]["direct"])

    def test_e2e_mtu_chunks_416(self):
        self._e2e(416, 7, 1400)


if __name__ == "__main__":
    unittest.main()
