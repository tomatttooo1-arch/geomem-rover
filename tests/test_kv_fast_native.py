"""E7 kv-fast (2026-11-02): the KV server's per-pixel data plane in C++ (host/kv_fast_native.py,
tools/kv_link/kvfast/kv_fast.cpp) is byte-identical to the Python path, and the switch defaults to off.

* Library level: crc32 == zlib on random bytes at every alignment; convert_crc == codes_to_m + zlib on
  random codes (0 / 1 / 65535 included), odd pixel counts, odd byte offsets (an unaligned request payload),
  several units. Positive controls: the two mutant builds (code 0 not NaN; crc skips the last byte) are
  caught by the same comparisons.
* Server level (KvLinkServer in process, CPU core, Nano client through M7Pipeline): native and off answer
  the same flags / counts / depth crc; a mutant server is caught end to end by the Nano's depth crc check;
  a replaced ``convert`` (the crc positive control of test_kv_link) bypasses the native path; a codes crc
  mismatch is a bad request with native as without.
* Switch: kv_server --kv-fast default comes from $FPGA_KV_FAST, else off; a failed build warns and serves
  the Python path (no start refusal).
The aarch64 build (NEON + crc32 instructions) is checked under qemu by tools/kv_link/kvfast/cross_check.py.
Run: PYTHONPATH=. python3 -m unittest tests.test_kv_fast_native (also /usr/bin/python3.8).
"""
import argparse
import os
import sys
import tempfile
import time
import unittest
import zlib
from pathlib import Path

import numpy as np

from host import kv_fast_native
from host.kv_fast_native import KvFastNative, default_mode
from host.kv_link import (CHUNK, KvLinkClient, KvLinkServer, RemotePlFrameBackend, bits_f32, codes_to_m,
                          crc32, encode_request)
from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
from tools.geo_same_input.common import depth_u16_to_m

ROOT = Path(__file__).resolve().parents[1]
GRID = GridSpec((-0.8, -0.8, -0.05), 0.05, (32, 32, 32))
H, W = 64, 96
K = np.array([[80.0, 0.0, 47.5], [0.0, 80.0, 31.5], [0.0, 0.0, 1.0]])
CLOCK, LOCAL, EPOCH = "test_mono", "cam", "e1"
UNIT = 0.001


class Libs:
    _tmp = None

    @classmethod
    def get(cls):
        if cls._tmp is None:
            cls._tmp = tempfile.TemporaryDirectory()
            d = cls._tmp.name
            cls.fast = KvFastNative.build(d)
            cls.m1 = KvFastNative.build(d, mutant=1, name="m1.so")
            cls.m2 = KvFastNative.build(d, mutant=2, name="m2.so")
            cls.frame = compile_frame_native(Path(d) / "frame.so")
        return cls


def f32_bits(v):
    return int(np.array([v], dtype="<f4").view("<u4")[0])


def unaligned_u16(codes, offset):
    """codes as a read-only u16 view starting ``offset`` bytes into a bytes object (like decode_request)."""
    raw = b"\0" * offset + np.ascontiguousarray(codes, dtype="<u2").tobytes()
    return np.frombuffer(raw, dtype="<u2", count=codes.size, offset=offset)


def reference(codes, unit_bits):
    with np.errstate(over="ignore", invalid="ignore"):
        d = np.ascontiguousarray(codes_to_m(codes, bits_f32(unit_bits)), dtype="<f4")
    return d, zlib.crc32(d.tobytes()) & 0xFFFFFFFF


class LibraryTests(unittest.TestCase):
    def test_crc32_equals_zlib_at_every_alignment_and_length(self):
        L = Libs.get().fast
        rng = np.random.RandomState(1)
        blob = rng.randint(0, 256, size=5000).astype(np.uint8).tobytes()
        for off in range(9):
            for n in (0, 1, 7, 8, 9, 31, 32, 33, 1023, 4097):
                u = np.frombuffer(blob, dtype=np.uint8, count=n, offset=off)
                got = int(L.lib.kvfast_crc32(u.ctypes.data, n))
                self.assertEqual(got, zlib.crc32(blob[off:off + n]) & 0xFFFFFFFF, (off, n))

    def test_convert_crc_bit_exact_on_random_codes_units_and_offsets(self):
        L = Libs.get().fast
        rng = np.random.RandomState(20261102)
        for n in (1, 7, 8, 1023, 1024, 1025, 640 * 37 + 3):
            codes = rng.randint(0, 65536, size=n).astype("<u2")
            codes[: min(n, 5)] = 0
            if n > 10:
                codes[5:8] = (1, 65535, 0)
            for unit in (0.001, 0.0002, 1.0, 1e-41, 3e38):
                bits = f32_bits(unit)
                ref, ref_crc = reference(codes, bits)
                for off in (0, 1):
                    src = unaligned_u16(codes, off)
                    out = np.full(n + 3, 7.0, dtype=np.float32)
                    crc = L.convert_crc(src, bits, out)
                    self.assertEqual(crc, ref_crc, (n, unit, off))
                    self.assertEqual(out[:n].view("<u4").tobytes(), ref.view("<u4").tobytes(), (n, unit, off))
                    self.assertTrue((out[n:] == 7.0).all())          # nothing written past n
                    self.assertEqual(L.crc32(src), zlib.crc32(codes.tobytes()) & 0xFFFFFFFF)

    def test_every_code_value_in_both_car_units(self):
        L = Libs.get().fast
        codes = np.arange(65536, dtype="<u2")
        for unit in (0.001, 0.0002):
            out = np.empty(65536, dtype=np.float32)
            crc = L.convert_crc(codes, f32_bits(unit), out)
            self.assertEqual(out.tobytes(), depth_u16_to_m(codes.reshape(256, 256), unit).tobytes())
            self.assertEqual(crc, crc32(out))

    def test_mutants_are_caught_positive_controls(self):
        libs = Libs.get()
        codes = np.array([0, 1, 2, 650, 0, 65535, 9, 10, 11], dtype="<u2")
        bits = f32_bits(UNIT)
        ref, ref_crc = reference(codes, bits)
        out = np.empty(codes.size, dtype=np.float32)
        libs.m1.convert_crc(codes, bits, out)
        self.assertNotEqual(out.tobytes(), ref.tobytes())          # code 0 -> 0.0, not NaN
        crc = libs.m2.convert_crc(codes, bits, out)
        self.assertEqual(out.tobytes(), ref.tobytes())
        self.assertNotEqual(crc, ref_crc)                          # crc skipped the last byte

    def test_layouts_it_does_not_take_go_to_python_and_are_counted(self):
        L = KvFastNative(Libs.get().fast.path)
        out = np.empty(16, dtype=np.float32)
        self.assertIsNone(L.convert_crc(np.zeros(8, dtype=np.int32), 0, out))
        self.assertIsNone(L.convert_crc(np.zeros(8, dtype="<u2"), 0, np.empty(4, dtype=np.float32)))
        self.assertIsNone(L.convert_crc(np.zeros(8, dtype="<u2"), -1, out))
        self.assertIsNone(L.convert_crc(np.zeros(32, dtype="<u2")[::2], 0, out))
        big = KvFastNative(L.path, max_pixels=4)
        self.assertIsNone(big.convert_crc(np.zeros(8, dtype="<u2"), 0, out))
        self.assertEqual(sum(v for k, v in L.counts.items() if k.startswith("python:")), 4)
        self.assertEqual(big.counts["python:size"], 1)
        d = L.describe()
        self.assertEqual(d["impl"], "native")
        self.assertIn(d["crc"], ("hw", "table"))


def frame(fid, codes, ns, unit=UNIT):
    ident = CaptureIdentity("s1", fid, ns, CLOCK, "cal", LOCAL, EPOCH)
    pose = PoseAtCapture(ns, CLOCK, LOCAL, EPOCH, True, np.eye(4))
    return DepthFrame(ident, depth_u16_to_m(codes, unit), K, pose)


def codes_for(i, h=H, w=W):
    yy, xx = np.mgrid[0:h, 0:w]
    c = (650 + 300 * np.sin(xx / 11. + i) + 150 * np.cos(yy / 7. - i)).astype(np.uint16)
    c[(xx * 7 + yy * 3 + i) % 13 == 0] = 0
    return c


class ServerTests(unittest.TestCase):
    def serve(self, fast):
        core = FrameNativeM7(Libs.get().frame).lib.fpga_m7_depth_frame
        srv = KvLinkServer(core, "cpu", bind=("127.0.0.1", 0), fast=fast).start()
        self.addCleanup(srv.stop)
        client = KvLinkClient(("127.0.0.1", srv.port), chunk_bytes=2001, timeout_s=2.0)
        client.hello(2.0)
        self.addCleanup(client.close)
        be = RemotePlFrameBackend(client, depth_unit_m=UNIT)
        return srv, be, M7Pipeline(be, GRID)

    def run_frames(self, fast, n=4):
        srv, be, pipe = self.serve(fast)
        outs = []
        for fid in range(1, n + 1):
            ns = time.monotonic_ns()
            nav = pipe.ingest_geometry(frame(fid, codes_for(fid), ns), now_ns=time.monotonic_ns(),
                                       now_clock_id=CLOCK).navigation
            outs.append((nav.valid, nav.reason, None if not nav.valid else (
                nav.payload["voxel_flags"].tobytes(), nav.payload["valid_grid_samples"],
                nav.payload["invalid_depth_or_projection"], nav.payload["outside_grid"])))
        return srv, be, outs

    def test_native_answers_exactly_what_python_answers(self):
        _, _, ref = self.run_frames(None)
        srv, be, got = self.run_frames(Libs.get().fast)
        self.assertTrue(all(v for v, _, _ in ref))
        self.assertEqual(got, ref)
        self.assertGreaterEqual(srv.fast.counts["native_convert"], 4)
        self.assertGreaterEqual(srv.fast.counts["native_crc"], 4)
        self.assertEqual(be.counts.get("depth_crc_mismatch", 0), 0)
        self.assertEqual(srv.info["kv_fast"]["impl"], "native")

    def test_off_keeps_server_info_as_before(self):
        srv, _, _ = self.run_frames(None, n=1)
        self.assertNotIn("kv_fast", srv.info)

    def test_mutant_server_is_caught_by_the_nano_depth_crc(self):
        srv, be, outs = self.run_frames(Libs.get().m2, n=2)
        self.assertEqual([r for _, r, _ in outs], ["backend_failure:RuntimeError"] * 2)
        self.assertEqual(be.counts["depth_crc_mismatch"], 2)

    def test_replaced_convert_bypasses_native(self):
        srv, be, pipe = self.serve(Libs.get().fast)
        srv.convert = lambda c, u: codes_to_m(c, np.float32(float(u) * 1.0001))
        ns = time.monotonic_ns()
        nav = pipe.ingest_geometry(frame(1, codes_for(1), ns), now_ns=time.monotonic_ns(),
                                   now_clock_id=CLOCK).navigation
        self.assertEqual(nav.reason, "backend_failure:RuntimeError")
        self.assertEqual(be.counts["depth_crc_mismatch"], 1)
        self.assertEqual(srv.counts["kv_fast_bypass:convert_hook"], 1)

    def test_codes_crc_mismatch_is_a_bad_request_with_native_too(self):
        for fast in (None, Libs.get().fast):
            core = FrameNativeM7(Libs.get().frame).lib.fpga_m7_depth_frame
            srv = KvLinkServer(core, "cpu", bind=("127.0.0.1", 0), fast=fast)
            codes = codes_for(1)
            body = bytearray(encode_request(
                frame_id=1, capture_ns=1, pose_ns=1,
                identity={"stream_instance": "s", "clock_id": "c", "calibration_id": "k", "pose_epoch": "e"},
                width=W, height=H, nx=32, ny=32, nz=32, unit_f32=np.float32(UNIT), voxel=0.05,
                origin=np.array([-0.8, -0.8, -0.05]), inverse_k=np.linalg.inv(K).astype(np.float32),
                world_camera=np.eye(4)[:3].astype(np.float32), codes=codes, depth_crc=0))
            body[-1] ^= 1                                   # one code bit flipped after the crc
            sent = []
            srv._sendto = lambda *a, **k: sent.append(a)
            srv._serve({"addr": ("127.0.0.1", 9), "session": 1, "frame_id": 1, "payload": bytes(body),
                        "first_t": time.monotonic(), "done_t": time.monotonic()}, time.monotonic())
            self.assertEqual(srv.counts["bad_request:ValueError"], 1, fast)
            self.assertEqual(sent, [])
            srv.sock.close()


class SwitchTests(unittest.TestCase):
    def test_default_mode_from_environment(self):
        self.assertEqual(default_mode({}), "off")
        self.assertEqual(default_mode({"FPGA_KV_FAST": "native"}), "native")
        self.assertEqual(default_mode({"FPGA_KV_FAST": " NATIVE "}), "native")
        self.assertEqual(default_mode({"FPGA_KV_FAST": "fast"}), "off")

    def test_failed_build_warns_and_serves_python(self):
        sys.path.insert(0, str(ROOT / "tools" / "kv_link"))
        try:
            import kv_server
        finally:
            sys.path.pop(0)
        a = argparse.Namespace(kv_fast="native")
        orig = kv_fast_native.compile_kv_fast

        def broken(*_a, **_k):
            raise RuntimeError("no compiler")
        kv_fast_native.compile_kv_fast = broken
        try:
            with tempfile.TemporaryDirectory() as d:
                self.assertIsNone(kv_server.build_kv_fast(a, d))
        finally:
            kv_fast_native.compile_kv_fast = orig
        self.assertTrue(a.kv_fast_status.startswith("failed: RuntimeError"))
        a = argparse.Namespace(kv_fast=None)
        old = os.environ.pop("FPGA_KV_FAST", None)
        try:
            with tempfile.TemporaryDirectory() as d:
                self.assertIsNone(kv_server.build_kv_fast(a, d))
            self.assertEqual(a.kv_fast_status, "off")
        finally:
            if old is not None:
                os.environ["FPGA_KV_FAST"] = old


if __name__ == "__main__":
    unittest.main()
