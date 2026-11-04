"""DPUPIPE1 (2026-11-03): C++ output processing (tools/dpu_pipe/det_post.cpp) against the Python reference
(tools/dpu_pipe/det_post_ref.py = DPUB1 full1.py dfl_decode_f + nms) on synthetic DPU output heads, plus
dpu_rx with several stub workers end to end.

  PYTHONPATH=. python3 -m unittest discover -s tests -p test_det_post.py

Known, measured difference: on x86 numpy's float32 exp is its own SIMD routine and differs from glibc expf
(the C++ side) by an ulp in ~40% of values, so boxes differ by ~1 ulp of the coordinate. The exact test swaps
the reference's np.exp for glibc expf (ctypes) to isolate that: then every value must be equal. On the KV260
(aarch64, no SIMD exp in numpy) the board check compares without the swap.
"""
import ctypes
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import types
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PIPE = os.path.join(ROOT, "tools", "dpu_pipe")
sys.path.insert(0, PIPE)
import det_post_ref as R  # noqa: E402

SIZES = {640: [(80, 80, 70, 2), (40, 40, 70, 2), (20, 20, 70, 2)],
         512: [(64, 64, 70, 2), (32, 32, 70, 2), (16, 16, 70, 2)],
         416: [(52, 52, 70, 2), (26, 26, 70, 2), (13, 13, 70, 2)]}


def synth(rng, heads, clusters=6):
    """Box logits ~N(0,12) int8; class logits mostly far below the 0.25 threshold, with 3x3 clusters of
    positives of one class (overlapping boxes, many equal scores: the int8 logits are coarse)."""
    bufs = []
    for h, w, c, _ in heads:
        a = np.empty((h, w, c), np.int8)
        a[..., :64] = np.clip(rng.normal(0, 12, (h, w, 64)), -128, 127).astype(np.int8)
        cl = np.clip(rng.normal(-40, 15, (h, w, c - 64)), -128, 127).astype(np.int8)
        for _ in range(int(rng.integers(0, clusters + 1))):
            y, x = int(rng.integers(0, h - 3)), int(rng.integers(0, w - 3))
            cl[y:y + 3, x:x + 3, int(rng.integers(0, c - 64))] = rng.integers(-6, 30, (3, 3))
        a[..., 64:] = cl
        bufs.append(a.tobytes())
    return b"".join(bufs)


def glibc_exp_np():
    lm = ctypes.CDLL("libm.so.6")
    lm.expf.restype = ctypes.c_float
    lm.expf.argtypes = [ctypes.c_float]
    vexp = np.vectorize(lambda v: lm.expf(float(v)), otypes=[np.float32])
    shim = types.SimpleNamespace(**{k: getattr(np, k) for k in dir(np) if not k.startswith("__")})
    shim.exp = lambda a: vexp(a).astype(np.float32)
    return shim


@unittest.skipIf(shutil.which("g++") is None or shutil.which("make") is None, "no C++ toolchain")
class DetPostTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.mkdtemp(prefix="detpost_")
        subprocess.run(["make", "-s", "-C", PIPE, "OUT=" + cls.build], check=True)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.build, ignore_errors=True)

    def cxx(self, raw, size, heads):
        p = os.path.join(self.build, "o.bin")
        open(p, "wb").write(raw)
        spec = ",".join("x".join(map(str, h)) for h in heads)
        out = subprocess.run([os.path.join(self.build, "det_post_cli"), "--size", str(size), "--heads", spec, "--in", p],
                             capture_output=True, text=True, check=True).stdout
        j = json.loads(out)
        return np.array(j["dets"], np.float32).reshape(-1, 6), j["candidates"]

    def ref(self, raw, size, heads, exact_exp, kind="stable"):
        saved = R.np
        if exact_exp:
            R.np = glibc_exp_np()
        try:
            d, det = R.run(R.heads_from_bytes(raw, heads), size, kind)
        finally:
            R.np = saved
        return det.astype(np.float32), len(d)

    def test_exact_with_same_exp(self):
        rng = np.random.default_rng(11)
        n_det = 0
        for size, heads in SIZES.items():
            for _ in range(4):
                raw = synth(rng, heads)
                c, cc = self.cxx(raw, size, heads)
                r, rc = self.ref(raw, size, heads, exact_exp=True)
                self.assertEqual(cc, rc)
                self.assertEqual(c.shape, r.shape)
                self.assertTrue(np.array_equal(c, r), "size %d max diff %g" % (size, np.abs(c - r).max()))
                n_det += len(r)
        self.assertGreater(n_det, 1000)   # the comparison saw real boxes

    def test_numpy_exp_within_ulps(self):
        rng = np.random.default_rng(12)
        for size, heads in SIZES.items():
            raw = synth(rng, heads)
            c, _ = self.cxx(raw, size, heads)
            r, _ = self.ref(raw, size, heads, exact_exp=False)
            self.assertEqual(c.shape, r.shape)
            self.assertTrue(np.array_equal(c[:, 5], r[:, 5]))
            self.assertLessEqual(float(np.abs(c[:, :4] - r[:, :4]).max()), 2e-4 * size / 64)

    def test_negative_control(self):
        """One box-logit byte of a surviving cell changed -> C++ output differs from the unmodified reference."""
        rng = np.random.default_rng(13)
        size, heads = 512, SIZES[512]
        raw = bytearray(synth(rng, heads, clusters=6))
        r, _ = self.ref(bytes(raw), size, heads, exact_exp=True)
        self.assertGreater(len(r), 0)
        h, w, c, _ = heads[0]
        a = np.frombuffer(bytes(raw[:h * w * c]), np.int8).reshape(h * w, c)
        cell = int(np.nonzero(a[:, 64:].max(1) * 0.25 > np.log(1 / 3))[0][0])
        raw[cell * c + 3] = (raw[cell * c + 3] + 40) % 256
        cc, _ = self.cxx(bytes(raw), size, heads)
        self.assertFalse(cc.shape == r.shape and np.array_equal(cc, r))

    def test_empty(self):
        heads = SIZES[416]
        raw = b"".join(np.full((h, w, c), -100, np.int8).tobytes() for h, w, c, _ in heads)
        c, cc = self.cxx(raw, 416, heads)
        self.assertEqual((c.shape[0], cc), (0, 0))

    def test_rx_stub_two_workers(self):
        """dpu_rx with 2 stub workers and --inflight 2 (one simulated DPU core, 40 ms): rate <= 1000/40, both
        workers used, per-frame timestamps ordered, one result line per inferred frame."""
        sys.path.insert(0, ROOT)
        from host.kv_colour import ColourRing, ColourSource
        port = 47990
        outlog = os.path.join(self.build, "out.jsonl")
        ring = ColourRing(12)
        src = ColourSource([("127.0.0.1", port)], ring.lookup, info={"source": "test"}, latest=ring.newest,
                           hello_s=0.2, dpu_backend="auto")
        src.warm_dpu(416, 7, 640, 480)   # before the receiver starts: the auto backend self-check takes time
        rx = subprocess.Popen([os.path.join(self.build, "dpu_rx"), "--port", str(port), "--size", "416", "--fix", "7",
                               "--chunk", "60000", "--infer-ms", "40", "--seconds", "5", "--warm-s", "2",
                               "--workers", "2", "--inflight", "2", "--rcvbuf", str(4 << 20), "--out-log", outlog],
                              stdout=subprocess.PIPE, text=True)
        stop = threading.Event()
        y, x = np.mgrid[0:480, 0:640]

        def cam():
            import time
            k = 0
            while not stop.is_set():
                f = np.stack([(x + k) % 256, y % 256, (x ^ y) % 256], -1).astype(np.uint8)
                ring.put(time.monotonic_ns(), 640, 480, f.tobytes())
                k += 1
                time.sleep(1 / 60)
        src.start()
        t = threading.Thread(target=cam, daemon=True)
        t.start()
        out, _ = rx.communicate(timeout=30)
        stop.set()
        src.stop()
        s = json.loads(out.strip().splitlines()[-1])
        self.assertEqual(s["workers"], 2)
        self.assertGreater(s["out_fps_steady"], 12)   # the Python fake camera on a loaded host can be the limit
        self.assertLess(s["out_fps_steady"], 25.5)    # never above one simulated DPU core (1000 / 40 ms)
        lines = [json.loads(l) for l in open(outlog)]
        self.assertEqual(len(lines), s["inferred"])
        self.assertEqual({l["worker"] for l in lines}, {0, 1})
        for l in lines:
            self.assertTrue(l["t_want"] <= l["t_first"] <= l["t_done"] <= l["t_acq"] <= l["t_staged"] <= l["t_infer"] <= l["t_out"])


if __name__ == "__main__":
    unittest.main()
