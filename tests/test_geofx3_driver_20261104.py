"""GEO_FX3 (2026-11-04): key checks of the fixed-point frame core's PS driver.

1. PS constants (hw/m7/frame_fx.cpp fpga_m7_fx_prepare, built by host.m7_frame_fx with -ffp-contract=off)
   bit for bit equal to the GEO_FX0 golden fx_prepare on the fixture frames of rec30 / car570
   (tests/data/geofx3_ps_golden_20261104.jsonl, dumped by GEO_FX3/tools/ps_check.cpp from the golden).
   Positive control: the same source with -DFPGA_M7_FX_CONTROL_SWAP=1 (three products added (2+1)+0)
   must differ on car570 frame 20 (the full 880-frame sweep: GEO_FX3/logs/ps_check_*.err). The swap shows
   only in the envelope doubles max|P| / reach (they decide status 5), never in the register words: rounding
   to 2^-36 / 2^-22 absorbs a last-bit change of a double. So the words get their own instrument control:
   one input float moved by one ulp must change them.
2. Driver + mock device (the mock runs FrameFx.frame on the REGISTER values the driver wrote) equal to the
   FrameFx expectation: flags, counts, the four statistics registers, depth crc. Positive controls: one
   output voxel flipped by the device; two FX_K words exchanged by the driver.
3. Fallback (warn and keep going, no refusal): PS status 5 / 3 and a bitstream without the fixed-point
   registers run the CPU reference on the A53 and give its result; the old bitstream never sees a write at
   0x200 or above (its 9-bit address decoder would alias FX_K onto AP_CTRL).
4. kv_server (--core pl --device mock) end to end with the Nano replay tool: every answered frame's flags
   and counts equal the FrameFx expectation of the same codes; the statistics log equals it too;
   core_detail.fx is "on". In process (kv_link server + Nano client): a frame the PS sends to the CPU
   reference (camera outside the grid) gets its depth crc on the A53 and the Nano sees no mismatch; a core
   whose DEPTH_CRC is off by one bit fails the frame on the Nano (ported from test_d16_20261103).
Also ported from test_d16_20261103: the odd-pixel buffer layout and the unit precondition d16_unit_ok.
"""

import json
import os
from pathlib import Path
import subprocess
import struct
import sys
import tempfile
import unittest

import numpy as np

import geofx3_ref as R
from host.kv_link import codes_to_m, crc32
from host.m7_backend import PlFrameBackend, d16_unit_ok
from host.m7_frame_fx import FrameFx, f32_bits
from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_pl_device import FRAME, MockPlDevice

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/data/geofx3_ps_golden_20261104.jsonl"
K = np.array([[80.0, 0.0, 47.5], [0.0, 80.0, 31.5], [0.0, 0.0, 1.0]])
GRID = ((-0.8, -0.8, -0.05), 0.05, (32, 32, 32))


def _tmp():
    if not hasattr(_tmp, "d"):
        _tmp.t = tempfile.TemporaryDirectory(prefix="geofx3_drv_")
        _tmp.d = Path(_tmp.t.name)
    return _tmp.d


def _f32s(bits):
    return np.array([struct.unpack("<f", struct.pack("<I", b))[0] for b in bits], dtype=np.float32)


def _f64(hexbits):
    return struct.unpack("<d", bytes.fromhex(hexbits)[::-1])[0]


def _fixture():
    return [json.loads(line) for line in FIXTURE.read_text().splitlines() if line.strip()]


def _prepare(fx, r):
    nx, ny, nz = r["dims"]
    return fx.prepare(r["w"], r["h"], nx, ny, nz, r["unit_bits"], _f32s(r["ik_bits"]), _f32s(r["wc_bits"]),
                      np.array([_f64(b) for b in r["org_bits"]]), _f64(r["voxel_bits"]),
                      reach_limit=1048576.0)        # the golden's reach limit (ps_check passes the same)


def _matches(p, g):
    gk = [int(x) for x in p["gk"]]
    return (p["status"] == g["status"] and gk[0:3] == g["Gu"] and gk[3:6] == g["Gv"] and gk[6:9] == g["G0"] and
            gk[9:12] == g["O"] and p["origin"] == g["origin"] and p["thresholds"] == [g["c_lo"], g["c_hi"], g["c20"]] and
            p["pmax"] == _f64(g["pmax_bits"]) and p["reach"] == _f64(g["reach_bits"]))


def _words_equal(p, g):
    gk = [int(x) for x in p["gk"]]
    return gk == g["Gu"] + g["Gv"] + g["G0"] + g["O"] and p["origin"] == g["origin"]


def codes_for(i, h=64, w=96):
    yy, xx = np.mgrid[0:h, 0:w]
    c = (650 + 300 * np.sin(xx / 11. + i) + 150 * np.cos(yy / 7. - i)).astype(np.uint16)
    c[(xx * 7 + yy * 3 + i) % 13 == 0] = 0
    return c


class LoggingDevice:
    def __init__(self, dev):
        self.dev, self.log = dev, []

    def __getattr__(self, name):
        return getattr(self.dev, name)

    def wr(self, block, off, value):
        self.log.append(("wr", block, off))
        return self.dev.wr(block, off, value)


# ---------------------------------------------------------------- 1. PS constants vs the golden
class PsConstantTests(unittest.TestCase):
    def test_fixture_bit_exact_and_order_swap_is_caught(self):
        rows = _fixture()
        self.assertGreaterEqual(len(rows), 9)
        self.assertIn(("car570", 20), [(r["tag"], r["frame"]) for r in rows])
        good = R.fx()
        for r in rows:
            with self.subTest(tag=r["tag"], frame=r["frame"]):
                self.assertEqual(r["golden"]["status"], 0)
                self.assertTrue(_matches(_prepare(good, r), r["golden"]))
        lib = _tmp() / "frame_fx_swap1.so"
        if not lib.exists():
            subprocess.run(["g++", "-std=c++11", "-O2", "-ffp-contract=off", "-DFPGA_M7_FX_CONTROL_SWAP=1",
                            "-fPIC", "-shared", str(ROOT / "hw/m7/frame_fx.cpp"), "-o", str(lib)], check=True)
        swapped = FrameFx(lib)
        caught = [(r["tag"], r["frame"]) for r in rows if not _matches(_prepare(swapped, r), r["golden"])]
        self.assertIn(("car570", 20), caught)
        # instrument control for the register words: inverse_k[0] one float32 ulp up -> Gu / G0 words differ
        r = dict(next(r for r in rows if (r["tag"], r["frame"]) == ("car570", 20)))
        self.assertTrue(_words_equal(_prepare(good, r), r["golden"]))
        r["ik_bits"] = [r["ik_bits"][0] + 1] + r["ik_bits"][1:]
        self.assertFalse(_words_equal(_prepare(good, r), r["golden"]))

    def test_library_is_built_without_fma_contraction(self):
        src = (ROOT / "host/m7_frame_fx.py").read_text()
        self.assertIn('"-ffp-contract=off"', src)


# ---------------------------------------------------------------- 2. driver + mock vs FrameFx
class DriverTests(unittest.TestCase):
    def backend(self, **mock):
        self.dev = MockPlDevice(frame_fx_library=R.fx_library(), **mock)
        return PlFrameBackend(self.dev, timeout_s=2.0, fx_library=R.fx_library())

    def test_driver_equals_fx_model(self):
        be = self.backend()
        self.assertEqual(be.fx_state, "on")
        n = 0
        for i in range(3):
            args = (codes_for(i),) + (K, np.eye(4)) + GRID
            want, got = R.expect(*args), R.call_codes(be, *args)
            self.assertIsNotNone(want)
            self.assertTrue(R.same(want, got), (want.counts, got.counts, want.stats, got.stats))
            self.assertGreater(want.accepted_rays, 1000)
            self.assertTrue((want.flags == 1).any() and (want.flags == 2).any())
            # the crc is the depth crc of the float32 depth f32(code) * unit (code 0 -> NaN)
            self.assertEqual(got.crc, int(crc32(codes_to_m(codes_for(i).reshape(-1), np.float32(0.001)))))
            n += 1
        self.assertEqual((self.dev.fx_starts, be.counts["fx_frames"]), (n, n))

    def test_flipped_output_voxel_is_seen(self):
        args = (codes_for(1),) + (K, np.eye(4)) + GRID
        want = R.expect(*args)
        free = int(np.flatnonzero(want.flags.ravel() == 1)[0])
        be = self.backend(corrupt_next=(FRAME, free))
        self.assertFalse(R.same(want, R.call_codes(be, *args)))

    def test_odd_pixel_count_layout_and_result(self):
        """Codes land at 2 bytes per pixel from the buffer start (odd tail: half a word)."""
        be = self.backend()
        c = (np.arange(35, dtype=np.uint16).reshape(7, 5) * 7 + 600).astype(np.uint16)
        be.lib.buffer("depth", 128).array[:128] = 0xEE
        args = (c, np.array([[6., 0., 2.], [0., 6., 3.], [0., 0., 1.]]), np.eye(4)) + GRID
        self.assertTrue(R.same(R.expect(*args), R.call_codes(be, *args)))
        self.assertEqual(bytes(be.lib._buffers["depth"].array[:70]), c.astype("<u2").tobytes())

    def test_unit_precondition(self):
        for u in (0.001, 0.0002, 1.0, np.float32(1.1754944e-38), np.float32(2.0 ** 111)):
            self.assertTrue(d16_unit_ok(f32_bits(np.float32(u))), u)
        for u in (0.0, -0.001, np.float32(1e-40), float("inf"), float("nan"), np.float32(2.0 ** 112)):
            self.assertFalse(d16_unit_ok(f32_bits(np.float32(u))), u)

    def test_exchanged_fx_k_words_are_seen(self):
        args = (codes_for(1),) + (K, np.eye(4)) + GRID
        want = R.expect(*args)
        be = self.backend()
        prepare = be.lib.fx.prepare

        def bad(*a, **kw):
            k = prepare(*a, **kw)
            k["words"] = k["words"].copy()
            k["words"][[12, 14]] = k["words"][[14, 12]]          # G0 x low <-> G0 y low
            return k
        be.lib.fx.prepare = bad
        self.assertFalse(R.same(want, R.call_codes(be, *args)))


# ---------------------------------------------------------------- 3. fallback to the CPU reference
class FallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpu_lib = compile_frame_native(_tmp() / "frame_cpu.so")

    def cpu_ref(self, codes, unit, T, k):
        ik, wc = R.geometry(k, T)
        h, w = codes.shape
        full = np.zeros(1280 * 800, np.float32)
        full[:h * w] = codes_to_m(codes.reshape(-1), np.float32(unit))
        flags = np.full(32 ** 3, 99, np.uint8)
        counts = np.full(3, 99, np.uint32)
        status = np.full(1, 99, np.uint32)
        FrameNativeM7(self.cpu_lib).lib.fpga_m7_depth_frame(full, ik.ravel(), wc.ravel(), np.array(GRID[0]), GRID[1],
                                                           w, h, 32, 32, 32, flags, counts, status)
        return flags, counts, int(status[0])

    def run_case(self, unit, T, fx_reg=True, k=K):
        dev = LoggingDevice(MockPlDevice(frame_fx_library=R.fx_library(), frame_fx_reg=fx_reg))
        be = PlFrameBackend(dev, timeout_s=2.0, cpu_library=self.cpu_lib, fx_library=R.fx_library())
        codes = codes_for(2)
        out = R.call_codes(be, codes, k, T, *GRID, unit_bits=f32_bits(unit))
        flags, counts, status = self.cpu_ref(codes, unit, T, k)
        self.assertEqual(out.status, status)
        self.assertEqual(out.counts, [int(x) for x in counts])
        self.assertTrue(np.array_equal(out.flags.ravel(), flags[:32 ** 3]))
        self.assertEqual(dev.dev.starts[FRAME], 0)
        return be, dev, out

    def test_envelope_status_5_runs_cpu_reference(self):
        wide = np.array([[2.0, 0.0, 47.5], [0.0, 2.0, 31.5], [0.0, 0.0, 1.0]])
        be, _, out = self.run_case(0.001, np.eye(4), k=wide)        # corner rays ~24:1, max|P| >= 15.5
        self.assertEqual(be.counts["cpu_reference:fx_status_5"], 1)
        self.assertEqual(out.status, 0)
        self.assertGreater(out.accepted_rays + out.outside_grid, 0)

    def test_camera_outside_grid_status_3_runs_cpu_reference(self):
        T = np.eye(4)
        T[:3, 3] = [5., 5., 5.]
        be, _, out = self.run_case(0.001, T)
        self.assertEqual(be.counts["cpu_reference:fx_status_3"], 1)
        self.assertNotEqual(out.status, 0)

    def test_old_bitstream_warns_runs_cpu_reference_and_never_writes_fx_k(self):
        import io
        from contextlib import redirect_stderr
        err = io.StringIO()
        with redirect_stderr(err):
            be, dev, out = self.run_case(0.001, np.eye(4), fx_reg=False)
        self.assertEqual(be.fx_state, "absent")
        self.assertIn("no fixed-point registers", err.getvalue())
        self.assertEqual((be.counts["fx_core_absent"], be.counts["cpu_reference:no_fx_core"]), (1, 1))
        self.assertEqual([e for e in dev.log if e[2] >= 0x200], [])
        self.assertEqual(out.status, 0)
        self.assertGreater(out.accepted_rays, 1000)


# ---------------------------------------------------------------- 4. kv_server end to end
class ServerEndToEndTests(unittest.TestCase):
    def test_kv_server_mock_equals_fx_model(self):
        from tools.geo_same_input.common import read_dump
        env = dict(os.environ, PYTHONPATH=str(ROOT))
        srv = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--bind", "127.0.0.1",
                                "--port", "0", "--max-idle-s", "60", "--core", "pl", "--device", "mock",
                                "--wait", "poll", "--fx-stats-log-every", "1"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
        n = 6
        try:
            port = json.loads(srv.stdout.readline())["port"]
            with tempfile.TemporaryDirectory() as t:
                prep = os.path.join(t, "prep")
                os.makedirs(prep)
                frames = [codes_for(i) for i in range(n)]
                np.stack(frames).astype("<u2").tofile(os.path.join(prep, "depth_u16.bin"))
                with open(os.path.join(prep, "meta.json"), "w") as fp:
                    json.dump({"width": 96, "height": 64, "frames": n, "depth_file": "depth_u16.bin",
                               "depth_unit_m": 0.001, "K": K.tolist(), "calibration_id": "synthetic",
                               "grid": {"origin_m": list(GRID[0]), "voxel_m": GRID[1], "shape_xyz": list(GRID[2])}},
                              fp)
                out = os.path.join(t, "run")
                done = subprocess.run([sys.executable, str(ROOT / "tools/kv_link/nano_replay.py"), "--prep", prep,
                                       "--out", out, "--hz", "10", "--backend", "remote_pl",
                                       "--server", "127.0.0.1:%d" % port, "--chunk-bytes", "2000"],
                                      capture_output=True, text=True, env=env)
                self.assertEqual(done.returncode, 0, done.stderr[-2000:])
                summary = json.load(open(os.path.join(out, "summary.json")))
                counts, flags = read_dump(os.path.join(out, "flags.dump"), 32 ** 3)
                index = [tuple(int(x) for x in ln.split(",")) for ln in
                         open(os.path.join(out, "dump_index.csv")).read().splitlines()[1:]]
        finally:
            srv.terminate()
            _, stderr = srv.communicate(timeout=10)
        self.assertGreaterEqual(len(index), n - 1)
        self.assertEqual(len(index), counts.shape[0])
        wants = {}
        for row, (_, src) in enumerate(index):
            want = R.expect(frames[src], K, np.eye(4), *GRID)
            wants[src] = want
            self.assertTrue(np.array_equal(flags[row], want.flags.ravel()), "frame %d" % src)
            self.assertEqual([int(x) for x in counts[row]], want.counts)
        logged = [json.loads(ln) for ln in stderr.splitlines() if ln.startswith('{"fx_stats"')]
        self.assertGreaterEqual(len(logged), len(index))
        expected_stats = [w.stats for w in wants.values()]
        for rec in logged:
            self.assertIn(rec["fx_stats"], expected_stats)
            self.assertEqual(rec["fx"]["status"], 0)
        self.assertEqual(json.dumps(summary).count('"fx": "on"'), 1, "core_detail.fx")


class BoardCheckToolTests(unittest.TestCase):
    def test_board_check_tool_on_the_mock(self):
        """tools/kv_link/geofx3_board_check.py (the README board step) passes on the mock, control caught."""
        with tempfile.TemporaryDirectory() as t:
            np.stack([codes_for(i) for i in range(5)]).astype("<u2").tofile(os.path.join(t, "depth_u16.bin"))
            with open(os.path.join(t, "meta.json"), "w") as fp:
                json.dump({"width": 96, "height": 64, "frames": 5, "depth_file": "depth_u16.bin",
                           "depth_unit_m": 0.001, "K": K.tolist(),
                           "grid": {"origin_m": list(GRID[0]), "voxel_m": GRID[1], "shape_xyz": list(GRID[2])}}, fp)
            done = subprocess.run([sys.executable, str(ROOT / "tools/kv_link/geofx3_board_check.py"), "--prep", t,
                                   "--device", "mock", "--frames", "3"], capture_output=True, text=True,
                                  env=dict(os.environ, PYTHONPATH=str(ROOT)))
        self.assertEqual(done.returncode, 0, done.stderr[-2000:])
        s = json.loads(done.stdout.strip().splitlines()[-1])
        self.assertEqual((s["fx_state"], s["frames_checked"], s["frames_equal"]), ("on", 3, 3))
        self.assertTrue(s["control"]["caught"])


class InProcessCrcTests(unittest.TestCase):
    def test_fallback_frame_crc_on_a53_and_crc_control(self):
        import time
        from host.m7_pipeline import CaptureIdentity, DepthFrame, M7Pipeline, PoseAtCapture
        from test_kv_link import CLOCK, EPOCH, GRID as KGRID, K as KK, LOCAL, remote, start_server
        crc_reg = __import__("host.m7_pl_device", fromlist=["FRAME_REGS"]).FRAME_REGS["DEPTH_CRC_DATA"]

        def ingest(pipe, fid, pose):
            ns = time.monotonic_ns()
            ident = CaptureIdentity("s1", fid, ns, CLOCK, "cal", LOCAL, EPOCH)
            fr = DepthFrame(ident, codes_to_m(codes_for(fid), np.float32(0.001)), KK,
                            PoseAtCapture(ns, CLOCK, LOCAL, EPOCH, True, pose))
            return pipe.ingest_geometry(fr, now_ns=time.monotonic_ns(), now_clock_id=CLOCK).navigation

        for mutant in (False, True):
            dev = MockPlDevice(frame_fx_library=R.fx_library())
            if mutant:
                ex = dev._execute

                def bad_execute(block, ex=ex, dev=dev):
                    ex(block)
                    if block == FRAME:
                        dev._regs.wr(FRAME, crc_reg, dev._regs.rd(FRAME, crc_reg) ^ 1)
                dev._execute = bad_execute
            be = PlFrameBackend(dev, timeout_s=2.0, fx_library=R.fx_library())
            srv = start_server(be.lib.fpga_m7_depth_frame, "pl:mock",
                               core_call_codes=be.lib.fpga_m7_depth_frame_codes, timing_source=be.lib)
            try:
                nano = remote(srv.port)
                pipe = M7Pipeline(nano, KGRID)
                nav = ingest(pipe, 1, np.eye(4))
                if mutant:
                    self.assertEqual(nav.reason, "backend_failure:RuntimeError")
                    self.assertEqual(nano.counts["depth_crc_mismatch"], 1)
                    continue
                self.assertTrue(nav.valid, nav.reason)
                want = R.expect(codes_for(1), KK, np.eye(4), KGRID.origin_m, KGRID.voxel_m, KGRID.shape)
                self.assertTrue(np.array_equal(nav.payload["voxel_flags"], want.flags))
                far = np.eye(4)
                far[:3, 3] = [50.0, 0.0, 0.0]                      # camera outside the grid: PS status 3
                ingest(pipe, 2, far)
                self.assertEqual(nano.counts["depth_crc_mismatch"], 0)
                self.assertEqual(be.counts["cpu_reference:fx_status_3"], 1)
                self.assertEqual(be.counts["fx_frames"], 1)
            finally:
                srv.stop()


if __name__ == "__main__":
    unittest.main()
