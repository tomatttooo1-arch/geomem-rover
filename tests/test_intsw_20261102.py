"""INT_SW (nationals 2026-11-02): PS side of the next bitstream, on the host.

1. Frame core DLAT2_EN (G1d ray thinning): the register header carries the offset of the G1d export
   (0x13c); host/m7_backend.setup_dlat2 never writes it with the switch at 0 on a current bitstream
   (MockPlDevice frame_dlat2_reg=False), clears a stale 1 on a next bitstream, and with the switch at 1
   writes it on every start of a next bitstream / only warns on a current one.
2. Rolling core op GRID (B1h) through PlRollingCore: probe, status mapping, cost table buffer, and the
   formal trace (10-26 rec30) through two RollingServices fed the same ROLL v2 bytes -- the PL path
   (PlRollingCore + MockRollingWindow(grid=True) running the fused core's GRID code) against the CPU
   path with the independent C model (hw/b1_costmap/costmap.cpp, B1's reference): every ACK field
   (header, delta, grid status, grid / aux / cost planes, lut crc) equal. Positive control: a driver
   that unpacks the planes one cell off must be caught. A current bitstream (window without GRID):
   probe says no, every call exactly as with the switch off.
3. Cost table rule "every row non-increasing along |dy|" (PL reason 4) on the KV (CONFIG v2 refused)
   and on the Nano (not sent, warning).
4. kv_server --roi-concurrent 1 end to end (mock): the same ROI rows as the serial server and as the CPU.
5. kv_start.sh: no switch file = command line as before; the switch file adds the three flags.
Run: PYTHONPATH=. python3 -m pytest -q tests/test_intsw_20261102.py   (about 1-2 minutes)
"""
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr

import numpy as np

from host import kv_rolling as kr
from host import rolling_nav_layer as rnl
from host.kv_rolling import (MODE_INLINE, OP_CONFIG, ROLL_VER_GRID, CpuRollingCore, MockRollingWindow,
                             PlRollingCore, RemotePlRollingBackend, RollingService, crc32, decode_roll_ack,
                             encode_roll_req)
from host.m7_backend import M7BackendSelector, setup_dlat2
from host.m7_frame_native import compile_frame_native
from host.m7_pl_device import FRAME, FRAME_REGS, MockPlDevice
from host.m7_rolling import compile_rolling

import geofx3_ref as fxr
from host.pl_grid_layer import (GRID_F_BODY, GRID_F_COST, GRID_F_GRID, LUT_DIM, band_params, body_q16,
                                default_lut, lut_ok, lut_problem)

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tests") not in sys.path:       # sibling test modules (harnesses reused below)
    sys.path.insert(0, str(ROOT / "tests"))
TRACE = os.environ.get("B1_TRACE", "${FPGA_SRC}/runs/fpga-innov-20261025/fuse/traces/formal.bin")
G1D_HW_H = ("${FPGA_RUN}/G1d/hls/fin-rec30/ip_repo/"
            "fpga_m7_frame_axi/drivers/fpga_m7_frame_axi_v1_0/src/xfpga_m7_frame_axi_hw.h")
OUT = os.environ.get("INTSW_TEST_OUT")
V = 0.05


class _Tmp:
    _t = None
    _n = 0

    @classmethod
    def path(cls, name):
        if cls._t is None:
            cls._t = tempfile.TemporaryDirectory()
        cls._n += 1
        return os.path.join(cls._t.name, "%d_%s" % (cls._n, name))


class LoggingDevice:
    """MockPlDevice wrapper that logs every register access of one block."""

    def __init__(self, dev):
        self.dev, self.log = dev, []

    def __getattr__(self, name):
        return getattr(self.dev, name)

    def rd(self, block, off):
        v = self.dev.rd(block, off)
        self.log.append(("rd", block, off))
        return v

    def wr(self, block, off, value):
        self.log.append(("wr", block, off, int(value) & 0xFFFFFFFF))
        self.dev.wr(block, off, value)


# ---------------------------------------------------------------- 1. DLAT2_EN
class Dlat2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = compile_frame_native(_Tmp.path("frame.so"))

    def test_header_offset_is_the_g1d_export(self):
        text = (ROOT / "hw/m7/pl/sw/m7pl_hls_regs.h").read_text()
        self.assertIn("#define XFPGA_M7_FRAME_AXI_CONTROL_ADDR_DLAT2_EN_DATA       0x13c", text)
        self.assertEqual(FRAME_REGS["DLAT2_EN_DATA"], 0x13c)
        # GEO_SLIM5 (2026-11-03): the header was regenerated from the merged geometry 7b + D16 frame
        # export (hw.h sha256 2787e0b8...; the G1d export was 0ec5e969...). Every frame define of the
        # MERGE3 header (fe53281, the one with DLAT2_EN) must still be there byte-identical: old offsets
        # never move, the D16 export only appends.
        # GEO_FX3 (2026-11-04): regenerated again from the fixed-point core export (GEO_FX3 hls/x3, hw.h
        # sha256 0e25d4d1...); the old offsets still never move (checked below against fe53281).
        self.assertIn("0e25d4d11b5983903e8db33d1115111c1571ba28168dca72d1b85466ab7b0771", text)
        try:
            old = subprocess.run(["git", "-C", str(ROOT), "show", "fe53281:hw/m7/pl/sw/m7pl_hls_regs.h"],
                                 check=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL).stdout.decode()
        except (OSError, subprocess.CalledProcessError):
            old = None
        if old is not None:
            self.assertIn("0ec5e9696c9d96b4a357cd9ed0474f01abd42e9be675addd77e169db5939280c", old)
            old_frame = [ln for ln in old.splitlines() if ln.startswith("#define XFPGA_M7_FRAME_AXI_")]
            self.assertGreater(len(old_frame), 40)
            new_lines = set(text.splitlines())
            self.assertEqual([ln for ln in old_frame if ln not in new_lines], [])
        if os.path.exists(G1D_HW_H):
            hw = open(G1D_HW_H).read()
            self.assertIn("#define XFPGA_M7_FRAME_AXI_CONTROL_ADDR_DLAT2_EN_DATA       0x13c", hw)
        # the other frame registers are unchanged: DLAT2_EN is the last entry before the D16 three (and the
        # GEO_FX1/GEO_FX3 fixed-point FX_* / ST_* registers after them)
        d16 = ("DEPTH_FMT_DATA", "DEPTH_UNIT_DATA", "DEPTH_CRC_DATA")
        self.assertLess(max(v for k, v in FRAME_REGS.items()
                            if k.endswith("_DATA") and k != "DLAT2_EN_DATA" and k not in d16 and
                            not k.startswith(("FX_", "ST_"))), 0x13c)
        self.assertGreater(min(v for k, v in FRAME_REGS.items() if k.startswith(("FX_", "ST_"))),
                           max(FRAME_REGS[k] for k in d16))
        self.assertGreater(min(FRAME_REGS[k] for k in d16), 0x13c)

    def _backend(self, reg, dlat2, preset=None):
        dev = MockPlDevice(frame_library=self.lib, frame_dlat2_reg=reg, frame_fx_library=fxr.fx_library())
        if preset is not None:
            dev.wr(FRAME, 0x13c, preset)
        log = LoggingDevice(dev)
        sel = M7BackendSelector(pl_device_factory=lambda: log, pl_timeout_s=0.5, pl_frame_dlat2=dlat2,
                                cpu_frame_library=self.lib, pl_frame_fx_library=fxr.fx_library())
        return sel.frame_backend("pl"), log, dev

    def _frames(self, be, n=2):
        # GEO_FX3: the codes call is the only PL path (the float call is the CPU reference on the A53)
        from test_m7_backend_switch import frame_cases
        return [fxr.call_codes(be, *fxr.case_args(frame_cases()[0])) for _ in range(n)]

    def test_switch_0_on_a_current_bitstream_never_writes(self):
        be, log, dev = self._backend(False, 0)
        self.assertEqual(be.dlat2_state, "untouched")
        self._frames(be)
        self.assertFalse([e for e in log.log if e[0] == "wr" and e[2] == 0x13c])
        self.assertEqual([e for e in log.log if e[2] == 0x13c], [("rd", FRAME, 0x13c)])   # the one probe read
        self.assertEqual(dev.dlat2_starts, 0)

    def test_switch_0_register_traffic_equals_the_old_driver(self):
        """Old driver = a register map without DLAT2_EN (GEO_FX3: the fixed-point registers are needed by
        every PL frame now, so only DLAT2_EN is taken out of the map)."""
        old_regs = {k: v for k, v in FRAME_REGS.items() if k != "DLAT2_EN_DATA"}
        from host.m7_backend import PlFrameBackend
        logs = []
        for regs in (old_regs, FRAME_REGS):
            dev = LoggingDevice(MockPlDevice(frame_library=self.lib, frame_dlat2_reg=False,
                                             frame_fx_library=fxr.fx_library()))
            be = PlFrameBackend(dev, timeout_s=0.5, regs=regs, dlat2=0, cpu_library=self.lib,
                                fx_library=fxr.fx_library())
            del dev.log[:]                       # construction (probe read) is compared separately above
            self._frames(be)
            logs.append(dev.log)
        self.assertEqual(logs[0], logs[1])

    def test_switch_0_on_a_next_bitstream_fresh_and_stale(self):
        be, log, dev = self._backend(True, 0)
        self.assertEqual(be.dlat2_state, "untouched")
        self._frames(be)
        self.assertFalse([e for e in log.log if e[0] == "wr" and e[2] == 0x13c])
        be, log, dev = self._backend(True, 0, preset=1)            # left at 1 by an earlier process
        self.assertEqual(be.dlat2_state, "cleared")
        self._frames(be)
        self.assertEqual(dev.dlat2_starts, 0)
        self.assertEqual([e for e in log.log if e[0] == "wr" and e[2] == 0x13c], [("wr", FRAME, 0x13c, 0)])

    def test_switch_1_next_bitstream_on_every_start(self):
        be, log, dev = self._backend(True, 1)
        self.assertEqual(be.dlat2_state, "on")
        self._frames(be, 3)
        self.assertEqual(dev.dlat2_starts, 3)
        self.assertEqual(len([e for e in log.log if e[0] == "wr" and e[2] == 0x13c]), 1 + 3)

    def test_switch_1_current_bitstream_warns_and_runs_without(self):
        err = io.StringIO()
        with redirect_stderr(err):
            be, log, dev = self._backend(False, 1)
        self.assertEqual(be.dlat2_state, "unsupported")
        self.assertIn("no DLAT2_EN register", err.getvalue())
        self.assertEqual(be.counts["dlat2_unsupported"], 1)
        n0 = len([e for e in log.log if e[0] == "wr" and e[2] == 0x13c])
        self._frames(be)
        self.assertEqual(len([e for e in log.log if e[0] == "wr" and e[2] == 0x13c]), n0)   # no per-frame write
        self.assertEqual(dev.dlat2_starts, 0)

    def test_bad_value_refused(self):
        with self.assertRaises(ValueError):
            setup_dlat2(None, 2)


# ---------------------------------------------------------------- 2. op GRID on the PL path
def pl_core(grid_build=True, probe=True, core_cls=PlRollingCore):
    dev = MockPlDevice()
    lib = compile_rolling(_Tmp.path("roll_mock.so"), grid=grid_build)
    win = MockRollingWindow(dev._memory, lib, grid=grid_build)
    words = CpuRollingCore(compile_rolling(_Tmp.path("cfg.so"))).config_words()
    return core_cls(win, dev, words, **({"grid": "probe"} if probe else {})), win


def cpu_service():
    return RollingService(CpuRollingCore(compile_rolling(_Tmp.path("roll_cpu.so")),
                                         grid_library=_Tmp.path("grid.so")))


def lut_config(lut):
    raw = np.ascontiguousarray(lut, dtype=np.uint8).tobytes()
    return encode_roll_req(op=OP_CONFIG, cmd_id=1, inline=raw, obs_crc=crc32(raw), grid=(0,) * 7)


def ack_fields(body):
    a = decode_roll_ack(body)
    g = a.get("grid") or {}
    return {"result": a["result"], "header": a["header"].tolist() if a["header"] is not None else None,
            "delta": a["delta"].tolist(), "qout": a["qout"].tolist(),
            "dump": None if a["dump"] is None else a["dump"].tobytes(),
            "grid_status": g.get("status"), "lut_crc": g.get("lut_crc"),
            "grid": None if g.get("grid") is None else g["grid"].tobytes(),
            "aux": None if g.get("aux") is None else g["aux"].tobytes(),
            "cost": None if g.get("cost") is None else g["cost"].tobytes()}


class ShiftedPlanes(PlRollingCore):
    """Positive control: the plane unpack one cell off (grid starts at cell 1)."""

    def _grid_after(self, *a, **kw):
        g, dev, rb = PlRollingCore._grid_after(self, *a, **kw)
        if g is not None and g[1] is not None:
            g = (g[0], np.roll(g[1], -1), g[2], g[3])
        return g, dev, rb


def flags_for(k):
    m = k % 4
    return {0: GRID_F_GRID | GRID_F_BODY | GRID_F_COST, 1: GRID_F_GRID | GRID_F_BODY,
            2: GRID_F_GRID | GRID_F_COST, 3: GRID_F_GRID}[m]


def body_pose(base, k):
    fx = 0.05 + 0.9 * ((k * 0.6180339887) % 1.0)
    fy = 0.05 + 0.9 * ((k * 0.7548776662 + 0.3) % 1.0)
    return ((base[0] + 40 + fx) * V, (base[1] + 40 + fy) * V)


def replay(svcs, limit=None):
    """Same ROLL bytes into every service; returns (per-field diff counts, stats)."""
    from tools.m7_rolling_pl.replay_trace import read_trace
    band = rnl.HeightBand(ground_z_m=0.0, robot_height_m=0.31)
    n = {"records": 0, "integrate": 0, "grid_asked": 0, "planes": 0, "cost_planes": 0, "band_outside": 0,
         "diff": {}, "statuses": {}, "sync_delta_mismatch": 0, "occupied": 0}
    cfg = lut_config(default_lut())
    acks = [ack_fields(s.handle(cfg)) for s in svcs]
    for a in acks[1:]:
        assert a == acks[0], "CONFIG v2 answers differ"
    cmd, k = 10, 0
    for r in read_trace(TRACE):
        cmd += 1
        n["records"] += 1
        grid = None
        if r["op"] == 1:
            n["integrate"] += 1
            try:
                lo, hi, tall = band_params(band, int(r["wb"][2]), 32, V)
                k += 1
                cx, cy, rq = body_q16(r["wb"], V, body_pose(r["wb"], k), 0.35)
                grid = (flags_for(k), lo, hi, tall, cx, cy, rq)
                n["grid_asked"] += 1
            except ValueError:
                n["band_outside"] += 1
        inline = b"" if r["obs"] is None else bytes(r["obs"])
        body = encode_roll_req(op=r["op"], cmd_id=cmd, mode=MODE_INLINE if r["op"] == 1 else 0,
                               epoch=r["epoch"], frame_seq=r["frame"], window_base=r["wb"], obs_base=r["ob"],
                               dims=r["dims"], obs_crc=crc32(inline), inline=inline,
                               query=r["query"] if r["op"] == 2 else None,
                               query_count=r["qc"] if r["op"] == 2 else None, grid=grid)
        res = [ack_fields(s.handle(body)) for s in svcs]
        for f in res[0]:
            if any(x[f] != res[0][f] for x in res[1:]):
                n["diff"][f] = n["diff"].get(f, 0) + 1
        a = res[0]
        if r["op"] == 1 and a["header"] is not None and a["header"][0] == 0:
            if a["delta"] != r["delta"].tolist():
                n["sync_delta_mismatch"] += 1
        if grid is not None:
            st = a["grid_status"]
            n["statuses"][str(st)] = n["statuses"].get(str(st), 0) + 1
            if st == 0:
                n["planes"] += 1
                n["cost_planes"] += int(a["cost"] is not None and len(a["cost"]) > 0)
                n["occupied"] += int((np.frombuffer(a["grid"], np.int8) == 100).sum())
        if limit and n["integrate"] >= limit:
            break
    return n


class GridProbeTests(unittest.TestCase):
    def test_probe_next_and_current_bitstream(self):
        core, _ = pl_core(grid_build=True)
        self.assertTrue(core.grid_capable)
        self.assertEqual(core.grid_probe["status"], 1)          # uninitialized core: GRID exists
        self.assertTrue(RollingService(core).grid_capable)
        old, _ = pl_core(grid_build=False)
        self.assertFalse(old.grid_capable)
        self.assertEqual((old.grid_probe["status"], old.grid_probe["reason"]), (2, 0))
        self.assertFalse(RollingService(old).grid_capable)
        off, _ = pl_core(grid_build=True, probe=False)
        self.assertFalse(off.grid_capable)
        self.assertNotIn("lut", off.b)
        self.assertNotIn("grid_probe", off.describe())

    def test_current_bitstream_traffic_equals_switch_off(self):
        """After the probe, a current bitstream's core does exactly what the switch-off core does."""
        logs = []
        for probe in (False, True):
            core, win = pl_core(grid_build=False, probe=probe)
            seen = []
            orig_wr = win.wr

            def wr(off, value, seen=seen, orig_wr=orig_wr):
                seen.append((off, int(value)))
                orig_wr(off, value)
            win.wr = wr
            svc = RollingService(core)
            svc.handle(lut_config(default_lut()))
            svc.handle(encode_roll_req(op=0, cmd_id=2, epoch=1, window_base=(0, 0, 0)))
            obs = bytes(np.ones(4 * 4 * 4, np.uint8))
            ack = decode_roll_ack(svc.handle(encode_roll_req(
                op=1, cmd_id=3, mode=MODE_INLINE, epoch=1, frame_seq=1, window_base=(0, 0, 0),
                obs_base=(10, 10, 5), dims=(4, 4, 4), obs_crc=crc32(obs), inline=obs,
                grid=(GRID_F_GRID, 0, 3, 2, 0, 0, 0))))
            self.assertEqual((ack["result"], ack["grid"]["status"]), (0, kr.GRID_NOT_RUN))
            logs.append(seen)
        self.assertEqual(logs[0], logs[1])

    def test_status_mapping_and_reasons(self):
        core, _ = pl_core()
        svc = RollingService(core)
        svc.handle(lut_config(default_lut()))
        svc.handle(encode_roll_req(op=0, cmd_id=2, epoch=1, window_base=(0, 0, 0)))
        obs = bytes(np.ones(4 * 4 * 4, np.uint8))
        cmd = [10]

        def integ(grid):
            cmd[0] += 1
            return decode_roll_ack(svc.handle(encode_roll_req(
                op=1, cmd_id=cmd[0], mode=MODE_INLINE, epoch=1, frame_seq=cmd[0], window_base=(0, 0, 0),
                obs_base=(10, 10, 5), dims=(4, 4, 4), obs_crc=crc32(obs), inline=obs, grid=grid)))
        ok = integ((GRID_F_GRID | GRID_F_COST, 0, 7, 3, 0, 0, 0))
        self.assertEqual(ok["grid"]["status"], 0)
        self.assertEqual((ok["grid"]["grid"].size, ok["grid"]["cost"].size), (6400, 6400))
        self.assertEqual(integ((GRID_F_GRID, 7, 3, 8, 0, 0, 0))["grid"]["status"], 2)          # band reversed
        self.assertEqual(integ((GRID_F_GRID | GRID_F_BODY, 0, 7, 3, 40 << 16, 40 << 16, 41 << 16))["grid"]["status"], 3)
        bad = default_lut().reshape(LUT_DIM, LUT_DIM).copy()
        bad[2, 1], bad[2, 2] = 10, 50                            # row 2 increases along |dy|
        svc.lut = bad.ravel()                                    # bypass the KV check: the core's own reason 4
        self.assertEqual(integ((GRID_F_GRID | GRID_F_COST, 0, 7, 3, 0, 0, 0))["grid"]["status"], 4)
        self.assertEqual(core.counts["grid_reason:4"], 1)
        # INTEGRATE refused (stale frame) -> no GRID started, 0x100
        n0 = core.counts["grid_calls"]
        cmd[0] += 1
        stale = decode_roll_ack(svc.handle(encode_roll_req(
            op=1, cmd_id=cmd[0], mode=MODE_INLINE, epoch=1, frame_seq=1, window_base=(0, 0, 0),
            obs_base=(10, 10, 5), dims=(4, 4, 4), obs_crc=crc32(obs), inline=obs,
            grid=(GRID_F_GRID, 0, 7, 3, 0, 0, 0))))
        self.assertNotEqual(int(stale["header"][0]), 0)
        self.assertEqual(stale["grid"]["status"], kr.GRID_NOT_RUN)
        self.assertEqual(core.counts["grid_calls"], n0)


class GridTraceTests(unittest.TestCase):
    def test_formal_trace_pl_mock_equals_cmodel(self):
        if not os.path.exists(TRACE):
            self.skipTest("trace %s not on this host" % TRACE)
        core, _ = pl_core()
        pl, cpu = RollingService(core), cpu_service()
        n = replay([pl, cpu])
        n["pl_counts"] = {k: v for k, v in core.counts.items() if "grid" in k}
        if OUT:
            with open(OUT, "w") as f:
                json.dump(n, f, indent=1, default=int)
        print("INT_SW GRID trace:", n)
        self.assertEqual(n["diff"], {})
        self.assertEqual(n["planes"], 376)
        self.assertEqual(n["band_outside"], 15)
        self.assertGreater(n["cost_planes"], 100)
        self.assertGreater(n["occupied"], 0)
        self.assertEqual(n["sync_delta_mismatch"], 0)

    def test_positive_controls(self):
        if not os.path.exists(TRACE):
            self.skipTest("trace %s not on this host" % TRACE)
        core, _ = pl_core(core_cls=ShiftedPlanes)
        n = replay([RollingService(core), cpu_service()], limit=40)
        self.assertGreater(n["diff"].get("grid", 0), 0)
        old, _ = pl_core(grid_build=False)                       # a current bitstream: no planes at all
        n = replay([RollingService(old), cpu_service()], limit=40)
        self.assertGreater(n["diff"].get("grid_status", 0), 0)
        self.assertNotIn("delta", n["diff"])                     # the INTEGRATE answers themselves equal


# ---------------------------------------------------------------- 3. cost table rule on both ends
class LutRuleTests(unittest.TestCase):
    def test_default_ok_and_increasing_row_refused(self):
        lut = default_lut()
        self.assertTrue(lut_ok(lut))
        bad = lut.reshape(LUT_DIM, LUT_DIM).copy()
        bad[3, 1], bad[3, 2] = 20, 60
        self.assertEqual(lut_problem(bad), "row_increases")
        svc = cpu_service()
        a = decode_roll_ack(svc.handle(lut_config(bad)))
        self.assertEqual(a["grid"]["lut_crc"], 0)
        self.assertEqual(svc.counts["lut_bad"], 1)

    def test_nano_does_not_send_a_refused_table(self):
        from test_b1_pl_grid_nano import InProcLink
        svc = cpu_service()
        bad = default_lut().reshape(LUT_DIM, LUT_DIM).copy()
        bad[3, 1], bad[3, 2] = 20, 60
        err = io.StringIO()
        with redirect_stderr(err):
            remote = RemotePlRollingBackend(None, link=InProcLink(svc), grid_lut=bad.ravel())
        self.assertIn("refused on the Nano", err.getvalue())
        self.assertEqual(remote.counts["grid_lut_rejected:row_increases"], 1)
        self.assertEqual((svc.counts["lut_set"], svc.counts["lut_bad"], remote.grid_lut_crc), (0, 0, 0))
        good = RemotePlRollingBackend(None, link=InProcLink(svc))
        self.assertEqual(good.grid_lut_crc, crc32(default_lut().tobytes()))


# ---------------------------------------------------------------- 4. concurrent ROI end to end
class RoiConcurrentTests(unittest.TestCase):
    def test_concurrent_equals_serial_and_cpu(self):
        from test_kv_roi_pl import ServerEndToEndTests
        runs = {}
        for conc in ("0", "1"):
            t = ServerEndToEndTests("test_pl_mock_end_to_end_and_mutants")
            with tempfile.TemporaryDirectory() as d:
                orig = subprocess.Popen

                def popen(cmd, *a, **kw):
                    if "kv_server.py" in " ".join(map(str, cmd)):
                        cmd = list(cmd) + ["--roi-concurrent", conc, "--out", d]
                    return orig(cmd, *a, **kw)
                subprocess.Popen = popen
                try:
                    s = t._run(None)
                finally:
                    subprocess.Popen = orig
                srv = json.load(open(os.path.join(d, "summary.json")))
            runs[conc] = (s, srv["total_counts"])
        for conc, (s, counts) in runs.items():
            self.assertTrue(s["roi"]["compare"]["equal"], conc)         # rows == CPU reference, every frame
            self.assertTrue(s["roi"]["compare"]["control_caught"], conc)
            self.assertEqual(s["roi"]["frames_ok"], s["answered"], conc)
        self.assertEqual(runs["0"][1].get("roi_concurrent_started", 0), 0)
        self.assertEqual(runs["1"][1].get("roi_concurrent_started"), runs["1"][0]["answered"])
        self.assertEqual(runs["0"][0]["roi"]["compare"]["rows_compared"], runs["1"][0]["roi"]["compare"]["rows_compared"])


# ---------------------------------------------------------------- 5. kv_start.sh switch file
class KvStartSwitchTests(unittest.TestCase):
    def run_start(self, conf, *extra):
        from test_competition_kv_service import KvStartTests
        t = KvStartTests("test_no_argument_is_old_a2_command")
        orig = subprocess.run

        def run(cmd, *a, **kw):
            if conf is not None and "kv_start.sh" in " ".join(map(str, cmd)):
                comp = Path(cmd[2]) / "ui/geomem/tools/competition"
                comp.mkdir(parents=True, exist_ok=True)
                (comp / "kv_nat_switches.conf").write_text(conf)
            return orig(cmd, *a, **kw)
        subprocess.run = run
        try:
            r, calls, old = t.run_start(*extra)
        finally:
            subprocess.run = orig
        t.doCleanups()
        return r, calls, old

    def test_absent_or_zero_file_is_the_old_command(self):
        for conf in (None, "KV_FRAME_DLAT2=0\nKV_ROI_CONCURRENT=0\nKV_ROLLING_GRID=none\n"):
            for prof, tail in (((), ""), (("v3fc2",), " --rolling pl")):
                r, calls, old = self.run_start(conf, *prof)
                self.assertEqual(r.returncode, 0, r.stderr)
                self.assertEqual(calls[-1], "kv_board " + old + tail)

    def test_switches_on(self):
        conf = "KV_FRAME_DLAT2=1\nKV_ROI_CONCURRENT=1  # comment\nKV_ROLLING_GRID=pl\n"
        r, calls, old = self.run_start(conf, "v3fc2")
        self.assertEqual(r.returncode, 0, r.stderr)
        # SW_M1 (2026-11-03): the concurrent ROI comes with its interrupt wait (KV_ROI_WAIT=poll: without it)
        self.assertEqual(calls[-1], "kv_board " + old + " --rolling pl --dlat2 1 --roi-concurrent 1 --roi-wait irq --rolling-grid pl")
        r, calls, old = self.run_start(conf)                     # a2 profile: no rolling -> grid ignored, warned
        self.assertEqual(calls[-1], "kv_board " + old + " --dlat2 1 --roi-concurrent 1 --roi-wait irq")
        self.assertIn("KV_ROLLING_GRID=pl ignored", r.stderr)
        r, calls, old = self.run_start(conf + "KV_ROI_WAIT=poll\n", "v3fc2")
        self.assertEqual(calls[-1], "kv_board " + old + " --rolling pl --dlat2 1 --roi-concurrent 1 --rolling-grid pl")


if __name__ == "__main__":
    unittest.main()
