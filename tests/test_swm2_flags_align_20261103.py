"""SW_M2 (nationals 2026-11-03): the FLAGS_DATA buffer must be 4-byte aligned.

Since WB_BURST (bitstreams m7_pl_m2 / m7_pl_m2_166) the geometry core writes the flags grid back as 32-bit words
in one burst; the circuit does not check the address. Every host place that writes FLAGS_DATA checks it:

* host/m7_backend.py (_PlFrameCall, the product driver on the KV260): an aligned address runs exactly as before;
  pa = aligned + 2 raises PlDeviceError naming the address, counts ``flags_misaligned``, and the core is never
  started (no register written).
* hw/m7/pl/sw/m7pl_test.cpp (board self-test): mock run PASS; ``--flags-pa-offset 2`` FAILs with the address.
* a census: no other source file writes FLAGS_DATA (a new writer must get the check and be listed here).
"""

import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
from host.m7_backend import FLAGS_PA_ALIGN, PlFrameBackend, check_flags_pa  # noqa: E402
from host.m7_frame_native import compile_frame_native  # noqa: E402
from host.m7_pl_device import (FRAME, FRAME_GRID_CAPACITY, FRAME_REGS, MockPlDevice, PlBuffer,
                               PlDeviceError)  # noqa: E402
from test_m7_backend_switch import frame_cases  # noqa: E402
import geofx3_ref as fxr  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
HLS_INC = Path("/data1/Xilinx/Vitis_HLS/2022.2/include")


class CheckFunctionTests(unittest.TestCase):
    def test_aligned_pass_and_misaligned_raise_with_address(self):
        self.assertEqual(FLAGS_PA_ALIGN, 4)
        for pa in (0, 0x1000, 0x1010a000, 0x7FFFFFFC):
            self.assertEqual(check_flags_pa(pa), pa)
        for pa in (0x1010a001, 0x1010a002, 0x1010a003, 0x7FFFFFFE):
            with self.assertRaisesRegex(PlDeviceError, "0x%x is not 4-byte aligned" % pa):
                check_flags_pa(pa)


class _ShiftedFlagsDevice(MockPlDevice):
    """Mock device whose flags-sized buffers come back at pa + shift (the buffer itself is unchanged)."""

    def __init__(self, shift, **kw):
        super().__init__(**kw)
        self.shift = shift
        self.writes = []

    def alloc(self, nbytes):
        buf = super().alloc(nbytes)
        if nbytes == FRAME_GRID_CAPACITY and self.shift:
            return PlBuffer(buf.pa + self.shift, buf.array)
        return buf

    def wr(self, block, offset, value):
        self.writes.append((block, offset))
        return super().wr(block, offset, value)


class BackendTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="swm2-flags-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.frame_lib = compile_frame_native(Path(cls.tmp.name) / "frame.so")

    def backend(self, shift):
        dev = _ShiftedFlagsDevice(shift, frame_library=self.frame_lib, frame_fx_library=fxr.fx_library())
        pl = PlFrameBackend(dev, cpu_library=self.frame_lib, fx_library=fxr.fx_library())
        del dev.writes[:]               # GEO_FX3: construction writes the FX_CODES probe; frames are checked
        return dev, pl

    def raw_call(self, pl):
        # GEO_FX3: the codes call is the only PL path (the float call runs the CPU reference on the A53)
        return fxr.call_codes(pl, *fxr.case_args(frame_cases()[0]))

    def test_aligned_address_runs_as_before(self):
        dev, pl = self.backend(0)
        args = fxr.case_args(frame_cases()[0])
        self.assertTrue(fxr.same(fxr.expect(*args), fxr.call_codes(pl, *args)))
        self.assertEqual(dev.starts[FRAME], 1)
        self.assertEqual(pl.counts["flags_misaligned"], 0)
        self.assertIn((FRAME, FRAME_REGS["FLAGS_DATA"]), dev.writes)

    def test_misaligned_by_two_is_caught_before_any_write(self):
        dev, pl = self.backend(2)
        with self.assertRaises(PlDeviceError) as cm:
            self.raw_call(pl)
        flags_pa = pl.lib._buffers["flags"].pa
        self.assertEqual(flags_pa % 4, 2)
        self.assertIn("0x%x is not 4-byte aligned" % flags_pa, str(cm.exception))
        self.assertEqual(pl.counts["flags_misaligned"], 1)
        self.assertEqual(dev.starts[FRAME], 0)
        self.assertNotIn((FRAME, FRAME_REGS["FLAGS_DATA"]), dev.writes)
        self.assertEqual([w for w in dev.writes if w[0] == FRAME], [])

    # GEO_FX3 (2026-11-04): test_pipeline_frame_is_invalid_not_silent removed: frame_from_depth (the float call
    # the pipeline makes) runs the CPU reference on the A53 since the PL reads codes only, so it never touches
    # FLAGS_DATA; the codes call (kv_server's only PL path) is test_misaligned_by_two_is_caught_before_any_write.


@unittest.skipUnless(shutil.which("g++") and (HLS_INC / "ap_int.h").exists(), "needs g++ and the Vitis HLS headers")
class SelfTestToolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="swm2-m7pl-")
        cls.addClassCleanup(cls.tmp.cleanup)
        r = subprocess.run(["make", "-s", "host", "BUILD=" + cls.tmp.name], cwd=ROOT / "hw/m7/pl/sw",
                           capture_output=True, text=True)
        if r.returncode:
            raise AssertionError("m7pl_test host build failed:\n" + r.stderr[-2000:])
        cls.bin = Path(cls.tmp.name) / "host/m7pl_test"

    def run_tool(self, *args):
        return subprocess.run([str(self.bin), "--provider", "mock", *args, "frame"], capture_output=True, text=True)

    def test_aligned_pass(self):
        r = self.run_tool()
        self.assertEqual(r.returncode, 0, r.stdout[-1500:])
        self.assertIn("RESULT PASS", r.stdout)
        self.assertNotIn("not 4-byte aligned", r.stdout)

    def test_offset_two_is_caught_with_the_address(self):
        r = self.run_tool("--flags-pa-offset", "2")
        self.assertNotEqual(r.returncode, 0)
        m = re.search(r"FAIL flags buffer pa=0x([0-9a-f]+) is not 4-byte aligned", r.stdout)
        self.assertIsNotNone(m, r.stdout[-1500:])
        self.assertEqual(int(m.group(1), 16) % 4, 2)


class CensusTests(unittest.TestCase):
    """Every source line that writes FLAGS_DATA is one of the checked ones."""

    CHECKED = {
        "host/m7_backend.py": 'self.wr64("FLAGS_DATA", flags_buf.pa)',
        "hw/m7/pl/sw/m7pl_test.cpp": "d.wr64(FRAME, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_FLAGS_DATA, flags_pa)",
    }

    def test_all_writers_are_checked(self):
        r = subprocess.run(["git", "grep", "-n", "-E", r"(wr64|wr|write)\w*\([^)]*FLAGS_DATA", "--",
                            "*.py", "*.cpp", "*.c", "*.h", "*.hpp", ":!tests/"],
                           cwd=ROOT, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        found = {}
        for line in r.stdout.splitlines():
            path, _, text = line.split(":", 2)
            found.setdefault(path, []).append(text.strip())
        self.assertEqual(sorted(found), sorted(self.CHECKED), r.stdout)
        for path, want in self.CHECKED.items():
            self.assertEqual(len(found[path]), 1, found[path])
            self.assertIn(want, found[path][0])
        self.assertIn("check_flags_pa(flags_buf.pa)", (ROOT / "host/m7_backend.py").read_text())
        self.assertIn("flags_pa % 4 != 0", (ROOT / "hw/m7/pl/sw/m7pl_test.cpp").read_text())


if __name__ == "__main__":
    unittest.main()
