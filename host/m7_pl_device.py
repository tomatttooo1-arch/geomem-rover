"""Minimal Python PL device layer for the M7 frame/object cores (2026-09-27).

Mirrors the C++ ``m7pl::Device`` of the PL integration branch (55c7172,
hw/m7/pl/sw/m7pl_dev.hpp): 32-bit register windows plus physically addressed
DMA buffers. Two implementations:

* ``MockPlDevice``: registers are memory, buffers are numpy arrays at fake
  physical addresses. Writing ap_start decodes the call ONLY from register
  contents and buffer memory and runs the same g++-compiled core, then writes
  ap_return / count registers following the AXI shells' contract. It checks
  the driver's register and buffer choreography; it is not RTL or board
  evidence.
* ``DevMemPlDevice``: board use. /dev/mem register windows and a DMA buffer
  provider chosen like the C++ layer: udmabuf -> dmaheap (auto), physmem only
  when named. Exercised on the server only with a mocked /dev, /sys, ioctl
  and pagemap; not on hardware.

See docs/design/M7_BACKEND_SWITCH_20260927.md and
docs/design/PL_DMA_PROVIDERS_20260927.md.
"""

from collections import Counter
import ctypes
import fcntl
import mmap
import os
from pathlib import Path
import platform
import select
import shutil
import struct
import subprocess
import tempfile
import threading
import zlib
import time

import numpy as np

# Block names and bases: hw/m7/pl/sw/m7pl_addrmap.h (generated from bd/m7_bd.tcl).
FRAME, OBJECT = "frame", "object"
BLOCK_BASE = {FRAME: 0xA0010000, OBJECT: 0xA0020000}
WINDOW_BYTES = 0x10000
DMA_LIMIT = 0x80000000  # HLS masters map low DDR only (bd/m7_bd.tcl)

AP_START, AP_DONE, AP_IDLE = 0x1, 0x2, 0x4

# Offsets transcribed from hw/m7/pl/sw/m7pl_hls_regs.h at 55c7172 (generated
# from the exported HLS drivers). tests/test_m7_backend_switch.py re-parses
# that header when available and compares every entry.
FRAME_REGS = {
    "AP_CTRL": 0x000, "AP_RETURN": 0x010,
    "DEPTH_M_DATA": 0x018, "FLAGS_DATA": 0x024,
    "GRID_ORIGIN_X_DATA": 0x030, "GRID_ORIGIN_Y_DATA": 0x0c0,
    "GRID_ORIGIN_Z_DATA": 0x0cc, "VOXEL_SIZE_M_DATA": 0x0d8,
    "WIDTH_DATA": 0x0e4, "HEIGHT_DATA": 0x0ec,
    "NX_DATA": 0x0f4, "NY_DATA": 0x0fc, "NZ_DATA": 0x104,
    "COUNT_ACCEPTED_DATA": 0x10c, "COUNT_INVALID_DATA": 0x11c,
    "COUNT_OUTSIDE_DATA": 0x12c,  # HLS name kept; counts CLIPPED rays (被裁剪) since M7_CLIP_SPEC_20261011
    # a523f39 = G3 9176a78, sha256 0ec5e969...): last s_axilite entry, reset 0 = the current behaviour.
    # Only in the next bitstream; an older frame core has no register there (reads 0, writes ignored).
    "DLAT2_EN_DATA": 0x13c,
    # D16 16-bit depth input: the three entries after DLAT2_EN (DEPTH_FMT must be 1 on the fixed-point core).
    "DEPTH_FMT_DATA": 0x144, "DEPTH_UNIT_DATA": 0x14c, "DEPTH_CRC_DATA": 0x154,
    "INVERSE_K_BASE": 0x040, "WORLD_CAMERA_BASE": 0x080,
    # GEO_FX3 (2026-11-04) fixed-point core (GEO_FX1 X1 + one s_axilite array for the 12 constants), export
    # GEO_FX3/hls/x3 (xfpga_m7_frame_axi_hw.h): every entry above unchanged; appended after DEPTH_CRC.
    #   FX_ORIGIN_*  Q8 camera origin (13 bit)   FX_CODES  c_lo [15:0] | c_hi [31:16]   FX_C20  16 bit
    #   ST_*         read-only depth statistics (GEO_FX0 DESIGN.md section 3.3)
    #   FX_K_BASE    24 x u32 array (0x200..0x25f used): 12 two's complement 64-bit slots, low word first,
    #                Gu x/y/z, Gv, G0, O (hw/m7/frame_fx.hpp). An older core decodes 9 address bits: 0x200
    #                aliases AP_CTRL there -- the driver writes FX_K only after the FX_CODES probe (0x17c).
    "FX_ORIGIN_X_DATA": 0x164, "FX_ORIGIN_Y_DATA": 0x16c, "FX_ORIGIN_Z_DATA": 0x174,
    "FX_CODES_DATA": 0x17c, "FX_C20_DATA": 0x184,
    "ST_NPOS_DATA": 0x18c, "ST_LE20_DATA": 0x19c, "ST_MM0_DATA": 0x1ac, "ST_MM1_DATA": 0x1bc,
    "FX_K_BASE": 0x200,
}
FX_K_WORDS = 24
FX_STAT_REGS = ("ST_NPOS_DATA", "ST_LE20_DATA", "ST_MM0_DATA", "ST_MM1_DATA")
_FX_OFFSETS = frozenset([FRAME_REGS[k] for k in ("FX_ORIGIN_X_DATA", "FX_ORIGIN_Y_DATA", "FX_ORIGIN_Z_DATA",
                                                 "FX_CODES_DATA", "FX_C20_DATA") + FX_STAT_REGS] +
                        [FRAME_REGS["FX_K_BASE"] + 4 * i for i in range(FX_K_WORDS)])
_FRAME_RO_OFFSETS = frozenset([FRAME_REGS["DEPTH_CRC_DATA"]] + [FRAME_REGS[k] for k in FX_STAT_REGS])
# D16 (2026-11-03) registers, kept by the fixed-point core: DEPTH_FMT bit 0 must be 1 (16-bit codes at
# DEPTH_M; 0 -> core status 6, the float32 input is gone since GEO_FX1), DEPTH_UNIT = float32 bits of the
# code unit (feeds only the depth crc), DEPTH_CRC (read) = zlib crc32 of the float32 depth f32(code) * unit.
OBJECT_REGS = {
    "AP_CTRL": 0x000, "AP_RETURN": 0x010,
    "DEPTH_DATA": 0x018, "MASK_DATA": 0x024, "XYZ_LOCAL_DATA": 0x030,
    "WIDTH_DATA": 0x03c, "HEIGHT_DATA": 0x044,
    "MASK_PRESENT_DATA": 0x04c, "ROI_ENABLED_DATA": 0x054,
    "ROI_X0_DATA": 0x05c, "ROI_Y0_DATA": 0x064,
    "ROI_X1_DATA": 0x06c, "ROI_Y1_DATA": 0x074,
    "NEAR_Z_DATA": 0x07c, "FAR_Z_DATA": 0x088, "DEPTH_MARGIN_M_DATA": 0x094,
    "MIN_PIXELS_DATA": 0x0a0, "MIN_VALID_FRACTION_DATA": 0x0a8,
    "FREE_FRACTION_DATA": 0x0b4,
    "COUNT_INPUT_DATA": 0x0c0, "COUNT_MASK_DATA": 0x0d0,
    "COUNT_PROJECTED_VALID_DATA": 0x0e0, "COUNT_ROI_DATA": 0x0f0,
    "COUNT_ROI_VALID_DATA": 0x200, "COUNT_ROI_FRONT_DATA": 0x210,
    "COUNT_ROI_BEHIND_DATA": 0x220, "CANDIDATE_DATA": 0x230,
    "INVERSE_K_BASE": 0x100, "LOCAL_CAMERA_BASE": 0x180,
}
OBJECT_COUNT_REGS = ("COUNT_INPUT_DATA", "COUNT_MASK_DATA", "COUNT_PROJECTED_VALID_DATA",
                     "COUNT_ROI_DATA", "COUNT_ROI_VALID_DATA", "COUNT_ROI_FRONT_DATA",
                     "COUNT_ROI_BEHIND_DATA", "CANDIDATE_DATA")
FRAME_GRID_CAPACITY = 32 ** 3

# Standard HLS AXI-lite interrupt registers, the same in every shell
# (m7pl_test.cpp cmd_irq). Kept out of FRAME_REGS/OBJECT_REGS, which the
# backend-switch test compares entry by entry with the generated header.
IRQ_REGS = {"GIE": 0x04, "IER": 0x08, "ISR": 0x0C}
# UIO node names (/sys/class/uio/uioN/name) of the 10-10 bitstream.
UIO_NODE = {FRAME: "m7_frame", OBJECT: "m7_object"}
UIO_SYSFS = "/sys/class/uio"
WAIT_MODES = ("poll", "irq")


class PlDeviceUnavailable(RuntimeError):
    """The PL device cannot be opened here; callers must not fall back.

    ``attempts`` lists (provider, "ok" or reason); ``counts`` the per-provider
    tried/failed/selected counters of the failed open.
    """

    def __init__(self, message, attempts=(), counts=None):
        super().__init__(message)
        self.attempts = list(attempts)
        self.counts = Counter(counts or {})


class PlDeviceError(RuntimeError):
    """A PL core did not run to completion (busy, timeout, bad buffer)."""


def find_uio(name, sysfs_root=UIO_SYSFS):
    """Index N of the one /sys/class/uio/uioN whose ``name`` is ``name``, else None.

    Same rule as m7pl_test.cpp find_uio; two nodes with the same name are an
    error rather than "last one wins".
    """
    root = Path(sysfs_root)
    if not root.is_dir():
        return None
    found = []
    for entry in sorted(root.iterdir()):
        if not entry.name.startswith("uio"):
            continue
        try:
            text = (entry / "name").read_text().strip()
        except OSError:
            continue
        if text == name:
            found.append(int(entry.name[3:]))
    if len(found) > 1:
        raise PlDeviceUnavailable("two UIO nodes named %r: %s" % (name, found))
    return found[0] if found else None


class UioIrq:
    """One UIO interrupt line (uio_pdrv_genirq): write 1 to (re)enable,
    poll() for the event, read the 4-byte event count. The kernel driver
    disables the line in its handler, so it must be re-armed before every
    wait, and the core's level ISR must be cleared before re-arming or the
    line fires again at once."""

    def __init__(self, fd, path, name):
        self.fd, self.path, self.name = fd, path, name
        self.arms = self.events = 0

    @classmethod
    def open(cls, name, sysfs_root=UIO_SYSFS, dev_root="/dev"):
        index = find_uio(name, sysfs_root)
        if index is None:
            raise PlDeviceUnavailable("no UIO node named %r under %s (needs "
                                      "uio_pdrv_genirq.of_id=generic-uio)" % (name, sysfs_root))
        path = os.path.join(dev_root, "uio%d" % index)
        try:
            fd = os.open(path, os.O_RDWR | os.O_CLOEXEC)
        except OSError as exc:
            raise PlDeviceUnavailable("cannot open %s for irq %r: %s" % (path, name, exc)) from exc
        return cls(fd, path, name)

    def arm(self):
        if os.write(self.fd, struct.pack("<I", 1)) != 4:
            raise PlDeviceError("%s: short write enabling the interrupt" % self.path)
        self.arms += 1

    def wait(self, timeout_s):
        """Block (no CPU) until the line fires; return the event count or None on timeout."""
        poller = select.poll()
        poller.register(self.fd, select.POLLIN)
        if not poller.poll(max(0, int(timeout_s * 1000 + 0.999))):
            return None
        raw = os.read(self.fd, 4)
        if len(raw) != 4:
            raise PlDeviceError("%s: short read of the interrupt count" % self.path)
        self.events += 1
        return struct.unpack("<I", raw)[0]

    def describe(self):
        return {"irq": self.name, "node": self.path, "arms": self.arms, "events": self.events}

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class _PipeIrq(UioIrq):
    """MockPlDevice's interrupt line: an os.pipe() standing in for /dev/uioN.

    Fires like a level interrupt routed through uio_pdrv_genirq: only while
    armed and only if GIE, IER bit 0 and ISR bit 0 are all set; firing
    disarms. Arming while the ISR is still set fires at once (the real
    spurious-wake hazard)."""

    def __init__(self, device, block):
        rfd, self._wfd = os.pipe()
        super().__init__(rfd, "pipe:%s" % block, UIO_NODE.get(block, block))
        self.device, self.block = device, block
        self.armed, self.count = False, 0
        self._lock = threading.Lock()

    def arm(self):
        with self._lock:
            self.armed = True
            self.arms += 1
        self.device._maybe_fire(self.block)

    def fire(self):
        with self._lock:
            if not self.armed:
                return False
            self.armed = False
            self.count += 1
            os.write(self._wfd, struct.pack("<I", self.count))
            return True

    def close(self):
        super().close()
        if self._wfd is not None:
            os.close(self._wfd)
            self._wfd = None


class PlBuffer:
    """A DMA buffer: ``pa`` is what the core sees, ``array`` is a uint8 view."""

    def __init__(self, pa, array, *, provider="mock", cached=False, va=None, token=None):
        self.pa, self.array = int(pa), array
        self.provider, self.cached, self.va, self.token = provider, cached, va, token

    @property
    def size(self):
        return self.array.size


class _Registers:
    def __init__(self):
        self.words = {}

    def rd(self, block, off):
        return self.words.get((block, off), 0)

    def wr(self, block, off, value):
        self.words[(block, off)] = int(value) & 0xFFFFFFFF

    def u64(self, block, off):
        return self.rd(block, off) | (self.rd(block, off + 4) << 32)

    def f64(self, block, off):
        return struct.unpack("<d", struct.pack("<Q", self.u64(block, off)))[0]

    def array(self, block, base, count, fmt):
        width = np.dtype("<" + fmt).itemsize
        raw = b"".join(struct.pack("<I", self.rd(block, base + 4 * i))
                       for i in range(count * width // 4))
        return np.frombuffer(raw, dtype="<" + fmt).copy()


class MockPlDevice:
    """Host stand-in that runs the g++ cores behind the HLS register map."""

    kind = "mock"

    def __init__(self, *, frame_library=None, object_library=None, hang=False,
                 busy=False, corrupt_next=None, irq=False, delay_s=0.0, frame_dlat2_reg=True,
                 frame_fx_reg=True, frame_fx_library=None):
        self._regs = _Registers()
        # frame_dlat2_reg (INT_SW 2026-11-02): True = the next bitstream's frame core (DLAT2_EN_DATA is a
        # read/write register); False = a current bitstream (that offset reads 0, writes are dropped), so the
        # driver's version probe can be tested against both. The mock runs the reference core either way: it
        # checks the register choreography, it does NOT model the Dlat2 thinning (dlat2_starts counts starts
        # that ran with the register at 1).
        self.frame_dlat2_reg = bool(frame_dlat2_reg)
        self.dlat2_starts = 0
        # GEO_FX3 (2026-11-04): the frame core is the fixed-point core. frame_fx_reg True (default) = its
        # registers (FX_K array, FX_ORIGIN, FX_CODES, FX_C20; ST_* read-only) exist and a start runs
        # host.m7_frame_fx.FrameFx.frame -- the bit-exact model of the core -- on the REGISTER values the
        # driver wrote (codes two per word at DEPTH_M, DEPTH_FMT bit 0 must be 1 else status 6). It checks the
        # driver's register image, buffer layout and choreography; the PL itself is proven by the C model and
        # C/RTL co-simulation (GEO_FX1/GEO_FX3) and on the board (GEO_FX3/README.md). False = an older
        # bitstream (those offsets read 0, writes dropped): the driver's probe must notice.
        # frame_library (the CPU reference) is no longer executed by the mock.
        self.frame_fx_reg = bool(frame_fx_reg)
        self.fx_starts = 0
        self._fx_library = frame_fx_library
        self._fx = None
        self._lock = threading.RLock()   # registers are shared with the delayed-completion thread
        self._buffers = {}
        self._next_pa = 0x10000000
        self._frame_lib = frame_library  # GEO_FX3: kept for callers; the FX model runs the frame block
        self._object_lib = self._load_object(object_library) if object_library else None
        # Fault hooks used only by tests: never complete, stay busy, or flip
        # one output byte ((block, byte_index)) after the next run.
        self.hang, self.busy, self.corrupt_next = hang, busy, corrupt_next
        # irq: offer interrupt lines (os.pipe); delay_s: the core finishes this
        # long after ap_start, in a thread that sleeps (stands in for ~600 ms of
        # PL device time without using CPU, so wait modes can be compared).
        self.irq_enabled, self.delay_s = bool(irq), float(delay_s)
        self._irqs = {}
        self.thread_errors = []
        self.starts = {FRAME: 0, OBJECT: 0}
        for block in (FRAME, OBJECT):
            self._regs.wr(block, 0, AP_IDLE)

    @staticmethod
    def _load_frame(path):
        lib = ctypes.CDLL(str(Path(path).resolve()))
        f32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        f64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        u8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
        u32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
        lib.fpga_m7_depth_frame.argtypes = [
            f32, f32, f32, f64, ctypes.c_double, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, u8, u32, u32]
        lib.fpga_m7_depth_frame.restype = None
        return lib

    @staticmethod
    def _load_object(path):
        lib = ctypes.CDLL(str(Path(path).resolve()))
        f32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
        f64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        u8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
        u32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
        lib.fpga_m7_object_evidence.argtypes = [
            f32, u8, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, u32, ctypes.c_double, ctypes.c_double,
            ctypes.c_double, f64, f64, ctypes.c_uint32, ctypes.c_double,
            ctypes.c_double, f64, u32, u32]
        lib.fpga_m7_object_evidence.restype = None
        return lib

    # -- device interface --------------------------------------------------
    def rd(self, block, off):
        with self._lock:
            value = self._regs.rd(block, off)
            if off == 0:
                self._regs.wr(block, 0, value & ~AP_DONE)  # ap_done clear-on-read
            return value

    def wr(self, block, off, value):
        with self._lock:
            self._wr(block, off, value)

    def _wr(self, block, off, value):
        if block == FRAME and off == FRAME_REGS["DLAT2_EN_DATA"] and not self.frame_dlat2_reg:
            return                      # a current bitstream: no register at this offset
        if block == FRAME and off in _FX_OFFSETS and not self.frame_fx_reg:
            return                      # an older bitstream: no fixed-point register here
        if block == FRAME and off in _FRAME_RO_OFFSETS:
            return                      # DEPTH_CRC / ST_*: read-only
        if off == 0 and value & AP_START:
            if block == FRAME and self._regs.rd(FRAME, FRAME_REGS["DLAT2_EN_DATA"]) & 1:
                self.dlat2_starts += 1
            if block == FRAME and self._regs.rd(FRAME, FRAME_REGS["DEPTH_FMT_DATA"]) & 1:
                self.fx_starts += 1
            self.starts[block] += 1
            if self.busy:
                return
            self._regs.wr(block, 0, 0)  # running: neither idle nor done
            if self.hang:
                return
            if self.delay_s > 0:
                threading.Thread(target=self._finish_later, args=(block,), daemon=True).start()
                return
            self._execute(block)
            self._complete(block)
            return
        if off == IRQ_REGS["ISR"]:
            value = self._regs.rd(block, off) ^ (int(value) & 0x3)   # toggle-on-write
        self._regs.wr(block, off, value)
        if off in (IRQ_REGS["GIE"], IRQ_REGS["IER"]):
            self._maybe_fire(block)

    def _finish_later(self, block):
        time.sleep(self.delay_s)
        with self._lock:
            try:
                self._execute(block)
            except Exception as exc:      # a hang, visible to the test through thread_errors
                self.thread_errors.append(repr(exc))
                return
            self._complete(block)

    def _complete(self, block):
        self._regs.wr(block, 0, AP_IDLE | AP_DONE)
        if self._regs.rd(block, IRQ_REGS["IER"]) & 1:
            self._regs.wr(block, IRQ_REGS["ISR"], self._regs.rd(block, IRQ_REGS["ISR"]) | 1)
        self._maybe_fire(block)

    def _maybe_fire(self, block):
        with self._lock:
            self._maybe_fire_locked(block)

    def _maybe_fire_locked(self, block):
        line = self._irqs.get(block)
        r = self._regs
        if (line is not None and r.rd(block, IRQ_REGS["GIE"]) & 1 and
                r.rd(block, IRQ_REGS["IER"]) & 1 and r.rd(block, IRQ_REGS["ISR"]) & 1):
            line.fire()

    def irq(self, block):
        if not self.irq_enabled:
            raise PlDeviceUnavailable("mock: built without interrupt lines (irq=False)")
        if block not in self._irqs:
            self._irqs[block] = _PipeIrq(self, block)
        return self._irqs[block]

    def is_idle(self, block):
        with self._lock:
            return bool(self._regs.rd(block, 0) & AP_IDLE) and not self.busy

    def alloc(self, nbytes):
        size = (int(nbytes) + 4095) & ~4095
        array = np.zeros(size, dtype=np.uint8)
        pa = self._next_pa
        self._buffers[pa] = array
        self._next_pa += size + 0x10000
        return PlBuffer(pa, array)

    def release(self, buf):
        self._buffers.pop(buf.pa, None)

    def to_device(self, buf):
        pass  # mock memory is coherent

    def describe(self):
        return {"provider": "mock", "cached": False, "live_buffers": len(self._buffers),
                "irq": self.irq_enabled, "delay_s": self.delay_s}

    def from_device(self, buf):
        pass

    # -- execution from register contents only ----------------------------
    def _memory(self, pa, nbytes, dtype):
        for base, array in self._buffers.items():
            if base <= pa and pa + nbytes <= base + array.size:
                start = pa - base
                return array[start:start + nbytes].view(dtype)
        raise PlDeviceError("mock: core accessed unmapped physical 0x%x" % pa)

    def _execute(self, block):
        r = self._regs
        if block == FRAME:
            g = FRAME_REGS
            width, height = r.rd(block, g["WIDTH_DATA"]), r.rd(block, g["HEIGHT_DATA"])
            pixels = width * height
            flags_view = self._memory(r.u64(block, g["FLAGS_DATA"]), FRAME_GRID_CAPACITY, np.uint8)
            fmt = r.rd(block, g["DEPTH_FMT_DATA"]) & 1
            for name in ("COUNT_ACCEPTED_DATA", "COUNT_INVALID_DATA", "COUNT_OUTSIDE_DATA", "DEPTH_CRC_DATA") + FX_STAT_REGS:
                r.wr(block, g[name], 0)
            if not self.frame_fx_reg:
                raise PlDeviceError("mock: frame core without fixed-point registers started")
            dims = [r.rd(block, g[n]) for n in ("NX_DATA", "NY_DATA", "NZ_DATA")]
            bad_dims = ((width == 0) != (height == 0) or width > 1280 or height > 800 or
                        min(dims) == 0 or max(dims) > 32)
            if bad_dims or not fmt:
                r.wr(block, g["AP_RETURN"], 1 if bad_dims else 6)   # the core's status order
                return
            if self._fx is None:
                from host.m7_frame_fx import FrameFx, compile_frame_fx
                lib = self._fx_library
                if lib is None:
                    import tempfile
                    lib = compile_frame_fx(Path(tempfile.mkdtemp(prefix="mock_fx_")) / "m7_frame_fx.so")
                self._fx = FrameFx(lib)
            codes = (self._memory(r.u64(block, g["DEPTH_M_DATA"]), 4 * ((pixels + 1) // 2), "<u2")[:pixels]
                     if pixels else np.zeros(0, dtype=np.uint16))
            words = [r.rd(block, g["FX_K_BASE"] + 4 * i) for i in range(FX_K_WORDS)]
            origin = [r.rd(block, g[n]) for n in ("FX_ORIGIN_X_DATA", "FX_ORIGIN_Y_DATA", "FX_ORIGIN_Z_DATA")]
            st, flags, counts, stats, crc = self._fx.frame(
                codes, width, height, dims[0], dims[1], dims[2], words, origin, r.rd(block, g["FX_CODES_DATA"]),
                r.rd(block, g["FX_C20_DATA"]), r.rd(block, g["DLAT2_EN_DATA"]) if self.frame_dlat2_reg else 0,
                r.rd(block, g["DEPTH_UNIT_DATA"]))
            r.wr(block, g["AP_RETURN"], st)
            if st != 0:
                return
            flags_view[:] = flags
            for name, value in zip(("COUNT_ACCEPTED_DATA", "COUNT_INVALID_DATA", "COUNT_OUTSIDE_DATA"), counts):
                r.wr(block, g[name], int(value))
            for name, value in zip(FX_STAT_REGS, stats):
                r.wr(block, g[name], int(value))
            r.wr(block, g["DEPTH_CRC_DATA"], crc)
            if self.corrupt_next and self.corrupt_next[0] == FRAME:
                flags_view[self.corrupt_next[1]] ^= 0x3
                self.corrupt_next = None
        elif block == OBJECT:
            if self._object_lib is None:
                raise PlDeviceError("mock: no object core loaded")
            g = OBJECT_REGS
            width, height = r.rd(block, g["WIDTH_DATA"]), r.rd(block, g["HEIGHT_DATA"])
            pixels = width * height
            depth = np.ascontiguousarray(
                self._memory(r.u64(block, g["DEPTH_DATA"]), 4 * pixels, "<f4").astype(np.float32))
            mask = np.ascontiguousarray(
                self._memory(r.u64(block, g["MASK_DATA"]), pixels, np.uint8).copy())
            xyz_view = self._memory(r.u64(block, g["XYZ_LOCAL_DATA"]), 24 * pixels, "<f8")
            xyz = np.ascontiguousarray(xyz_view.astype(np.float64))
            roi = np.array([r.rd(block, g[n]) for n in
                            ("ROI_X0_DATA", "ROI_Y0_DATA", "ROI_X1_DATA", "ROI_Y1_DATA")],
                           dtype=np.uint32)
            counts = np.zeros(8, dtype=np.uint32)          # shell: counts[8] = {0,...}
            status = np.full(1, 0xFFFFFFFF, dtype=np.uint32)
            self._object_lib.fpga_m7_object_evidence(
                depth, mask, width, height, r.rd(block, g["MASK_PRESENT_DATA"]),
                r.rd(block, g["ROI_ENABLED_DATA"]), roi,
                r.f64(block, g["NEAR_Z_DATA"]), r.f64(block, g["FAR_Z_DATA"]),
                r.f64(block, g["DEPTH_MARGIN_M_DATA"]),
                r.array(block, g["INVERSE_K_BASE"], 9, "f8").astype(np.float64),
                r.array(block, g["LOCAL_CAMERA_BASE"], 12, "f8").astype(np.float64),
                r.rd(block, g["MIN_PIXELS_DATA"]), r.f64(block, g["MIN_VALID_FRACTION_DATA"]),
                r.f64(block, g["FREE_FRACTION_DATA"]), xyz, counts, status)
            xyz_view[:] = xyz
            r.wr(block, g["AP_RETURN"], int(status[0]))
            for name, value in zip(OBJECT_COUNT_REGS, counts):
                r.wr(block, g[name], int(value))
            if self.corrupt_next and self.corrupt_next[0] == OBJECT:
                xyz_view.view(np.uint8)[self.corrupt_next[1]] ^= 0x1
                self.corrupt_next = None
        else:
            raise PlDeviceError("mock: unknown block %r" % (block,))


# ---------------------------------------------------------------- DMA buffer providers
# Same providers and auto order as hw/m7/pl/sw/m7pl_dev.cpp (pick_provider):
#   auto  = udmabuf -> dmaheap ; physmem only when named explicitly (it needs a
#   caller-given physical window). docs/design/PL_DMA_PROVIDERS_20260927.md.
PAGE = mmap.PAGESIZE
PROVIDERS = ("udmabuf", "dmaheap", "physmem")
AUTO_ORDER = ("udmabuf", "dmaheap")
UDMABUF_SYSFS_ROOTS = ("/sys/class/u-dma-buf", "/sys/class/udmabuf")
# linux,cma: a device-tree "linux,cma-default" node; reserved: the default CMA
# area from cma= / Kconfig (kernel/dma/contiguous.c). "system" is never
# contiguous and is never chosen.
HEAP_CANDIDATES = ("linux,cma", "reserved")
# _IOWR('H', 0x0, struct dma_heap_allocation_data {u64 len; u32 fd; u32 fd_flags; u64 heap_flags})
DMA_HEAP_IOCTL_ALLOC = (3 << 30) | (24 << 16) | (ord("H") << 8) | 0x0
_HEAP_ALLOC = struct.Struct("<QIIQ")
_PFN_MASK = (1 << 55) - 1
ENV_KEYS = {"provider": "FPGA_M7_DMA_PROVIDER", "udmabuf": "FPGA_M7_UDMABUF",
            "heap": "FPGA_M7_DMA_HEAP", "phys_base": "FPGA_M7_PHYS_BASE",
            "phys_size": "FPGA_M7_PHYS_SIZE"}

_CACHE_SOURCE = r"""
#include <stddef.h>
#include <stdint.h>
/* Same maintenance as m7pl_dev.cpp dcache_range(): EL0 may issue DC CVAC /
   DC CIVAC (Linux sets SCTLR_EL1.UCI); DC IVAC is EL1-only. */
static uintptr_t line_bytes(void) {
    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    return (uintptr_t)4u << ((ctr >> 16) & 0xF);
}
void m7pl_dcache_clean(uintptr_t p, size_t len) {
    const uintptr_t line = line_bytes(), end = p + len;
    for (uintptr_t a = p & ~(line - 1); a < end; a += line)
        __asm__ volatile("dc cvac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb sy" ::: "memory");
}
void m7pl_dcache_clean_invalidate(uintptr_t p, size_t len) {
    const uintptr_t line = line_bytes(), end = p + len;
    for (uintptr_t a = p & ~(line - 1); a < end; a += line)
        __asm__ volatile("dc civac, %0" :: "r"(a) : "memory");
    __asm__ volatile("dsb sy" ::: "memory");
}
void m7pl_barrier(void) { __asm__ volatile("dsb sy" ::: "memory"); }
"""


_CACHE_CFLAGS = ("-x", "c", "-O2", "-fPIC", "-shared")


def _page_round(nbytes):
    return (max(int(nbytes), 1) + PAGE - 1) & ~(PAGE - 1)


class _ArmCacheOps:
    """DC CVAC / DC CIVAC / DSB SY from a tiny helper compiled on the board."""

    def __init__(self):
        if platform.machine() not in ("aarch64", "arm64"):
            raise OSError("cache maintenance helper is aarch64-only (this host: %s)"
                          % platform.machine())
        compiler = shutil.which("gcc") or shutil.which("cc") or shutil.which("g++")
        if compiler is None:
            raise OSError("no C compiler for the cache maintenance helper (sudo apt install gcc)")
        tmp = tempfile.mkdtemp(prefix="m7pl_cache_")
        try:
            src, lib = os.path.join(tmp, "m7pl_cache.c"), os.path.join(tmp, "m7pl_cache.so")
            with open(src, "w") as fp:
                fp.write(_CACHE_SOURCE)
            done = subprocess.run([compiler, *_CACHE_CFLAGS, src, "-o", lib],
                                  capture_output=True, text=True)
            if done.returncode != 0:
                raise OSError("cache helper build failed: %s" % done.stderr.strip()[-300:])
            self._lib = ctypes.CDLL(lib)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        for name in ("m7pl_dcache_clean", "m7pl_dcache_clean_invalidate"):
            getattr(self._lib, name).argtypes = [ctypes.c_size_t, ctypes.c_size_t]
            getattr(self._lib, name).restype = None
        self._lib.m7pl_barrier.argtypes, self._lib.m7pl_barrier.restype = [], None

    def clean(self, va, nbytes):
        self._lib.m7pl_dcache_clean(va, nbytes)

    def clean_invalidate(self, va, nbytes):
        self._lib.m7pl_dcache_clean_invalidate(va, nbytes)

    def barrier(self):
        self._lib.m7pl_barrier()


class SystemOps:
    """The OS calls the providers make; tests substitute a fake."""

    def ioctl(self, fd, request, buf):
        fcntl.ioctl(fd, request, buf, True)

    def pagemap(self, va, npages):
        """Raw /proc/self/pagemap entries for ``npages`` pages from ``va``."""
        fd = os.open("/proc/self/pagemap", os.O_RDONLY)
        try:
            raw = os.pread(fd, 8 * npages, (va // PAGE) * 8)
        finally:
            os.close(fd)
        if len(raw) != 8 * npages:
            raise OSError("short read from /proc/self/pagemap")
        return struct.unpack("<%dQ" % npages, raw)

    def cache_ops(self):
        return _ArmCacheOps()


_LIBC = None


def _libc():
    global _LIBC
    if _LIBC is None:
        lib = ctypes.CDLL(None, use_errno=True)
        lib.mmap.restype = ctypes.c_void_p
        lib.mmap.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_int,
                             ctypes.c_int, ctypes.c_int64]
        lib.munmap.restype = ctypes.c_int
        lib.munmap.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        _LIBC = lib
    return _LIBC


class _Mapping:
    """A raw shared mapping (libc mmap) so the virtual address is known for
    pagemap and cache maintenance, and munmap does not depend on Python
    buffer-export bookkeeping (same lifetime rule as the C++ layer)."""

    def __init__(self, fd, size, offset=0):
        lib = _libc()
        addr = lib.mmap(None, size, mmap.PROT_READ | mmap.PROT_WRITE, mmap.MAP_SHARED,
                        fd, offset)
        if addr is None or addr == ctypes.c_void_p(-1).value:
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err))
        self.addr, self.size = addr, size

    def array(self, offset, nbytes):
        return np.ctypeslib.as_array((ctypes.c_uint8 * nbytes).from_address(self.addr + offset))

    def close(self):
        if self.addr:
            _libc().munmap(self.addr, self.size)
            self.addr = 0


class _Region:
    """First-fit allocator over one contiguous region; freed blocks coalesce."""

    def __init__(self, name, size):
        self.name, self.size = name, int(size)
        self.free = [(0, self.size)]      # sorted (offset, bytes)
        self.used = {}                    # offset -> bytes

    def take(self, nbytes):
        want = _page_round(nbytes)
        for i, (off, size) in enumerate(self.free):
            if size >= want:
                if size == want:
                    del self.free[i]
                else:
                    self.free[i] = (off + want, size - want)
                self.used[off] = want
                return off, want
        largest = max((size for _, size in self.free), default=0)
        raise PlDeviceError("%s: region exhausted: need %d bytes, largest free block %d, "
                            "region %d bytes, %d buffers live"
                            % (self.name, want, largest, self.size, len(self.used)))

    def give_back(self, off):
        size = self.used.pop(off, None)
        if size is None:
            raise PlDeviceError("%s: release of a buffer this region did not hand out "
                                "(offset 0x%x, double release?)" % (self.name, off))
        self.free.append((off, size))
        self.free.sort()
        merged = []
        for start, length in self.free:
            if merged and merged[-1][0] + merged[-1][1] == start:
                merged[-1] = (merged[-1][0], merged[-1][1] + length)
            else:
                merged.append((start, length))
        self.free = merged

    @property
    def live_bytes(self):
        return sum(self.used.values())


def _check_limit(name, pa, nbytes):
    if pa + nbytes > DMA_LIMIT:
        raise PlDeviceError("%s: buffer 0x%x+0x%x is outside the PL DMA window (low 2 GiB)"
                            % (name, pa, nbytes))


class _RegionProvider:
    """udmabuf / physmem: one uncached physically contiguous region."""

    cached = False
    region_cls = _Region

    def __init__(self, name, mapping, pa, fds, barrier=None):
        self.name, self.mapping, self.pa, self._fds = name, mapping, int(pa), list(fds)
        self.region = self.region_cls(name, mapping.size)
        self.barrier = barrier

    def alloc(self, nbytes):
        off, size = self.region.take(nbytes)
        _check_limit(self.name, self.pa + off, size)
        return PlBuffer(self.pa + off, self.mapping.array(off, size), provider=self.name,
                        va=self.mapping.addr + off, token=off)

    def release(self, buf):
        self.region.give_back(buf.token)

    def to_device(self, buf):
        if self.barrier is not None:
            self.barrier()   # uncached, but order the buffer writes before ap_start

    def from_device(self, buf):
        if self.barrier is not None:
            self.barrier()

    def close(self):
        self.mapping.close()
        for fd in self._fds:
            os.close(fd)
        self._fds = []

    def describe(self):
        return {"region_pa": "0x%x" % self.pa, "region_size": self.mapping.size,
                "live_bytes": self.region.live_bytes}


class _HeapProvider:
    """dma_heap (CMA): one dma-buf per buffer; PA from /proc/self/pagemap (root)
    with page-by-page contiguity check; cached mapping -> DC CVAC / DC CIVAC."""

    name = "dmaheap"
    cached = True

    def __init__(self, heap, heap_fd, ops, cache):
        self.heap, self._heap_fd, self.ops, self.cache = heap, heap_fd, ops, cache
        self._live = {}   # pa -> (fd, mapping)

    def physical(self, va, nbytes):
        entries = self.ops.pagemap(va, nbytes // PAGE)
        first = None
        for k, entry in enumerate(entries):
            if not entry >> 63:
                raise PlDeviceError("dmaheap: page %d not present in pagemap" % k)
            pfn = entry & _PFN_MASK
            if pfn == 0:
                raise PlDeviceError("dmaheap: pagemap PFN hidden (need root/CAP_SYS_ADMIN)")
            pa = pfn * PAGE
            if first is None:
                first = pa
            elif pa != first + k * PAGE:
                raise PlDeviceError("dmaheap: buffer is not physically contiguous "
                                    "(page %d at 0x%x, expected 0x%x)" % (k, pa, first + k * PAGE))
        return first

    def alloc(self, nbytes):
        size = _page_round(nbytes)
        req = bytearray(_HEAP_ALLOC.pack(size, 0, os.O_RDWR | os.O_CLOEXEC, 0))
        try:
            self.ops.ioctl(self._heap_fd, DMA_HEAP_IOCTL_ALLOC, req)
        except OSError as exc:
            raise PlDeviceError("dmaheap %s: DMA_HEAP_IOCTL_ALLOC(%d bytes): %s"
                                % (self.heap, size, exc)) from exc
        fd = _HEAP_ALLOC.unpack(bytes(req))[1]
        try:
            mapping = _Mapping(fd, size)
        except OSError as exc:
            os.close(fd)
            raise PlDeviceError("dmaheap: mmap dma-buf: %s" % exc) from exc
        try:
            array = mapping.array(0, size)
            array[:] = 0                      # fault every page in before pagemap
            pa = self.physical(mapping.addr, size)
            _check_limit(self.name, pa, size)
        except Exception:
            mapping.close()
            os.close(fd)
            raise
        self.cache.clean(mapping.addr, size)  # push the zeroing out before any DMA
        self._live[pa] = (fd, mapping)
        return PlBuffer(pa, array, provider=self.name, cached=True, va=mapping.addr, token=pa)

    def release(self, buf):
        entry = self._live.pop(buf.token, None)
        if entry is None:
            raise PlDeviceError("dmaheap: release of a buffer not live here (pa 0x%x, double "
                                "release?)" % buf.pa)
        fd, mapping = entry
        mapping.close()
        os.close(fd)

    def to_device(self, buf):
        self.cache.clean(buf.va, buf.size)

    def from_device(self, buf):
        self.cache.clean_invalidate(buf.va, buf.size)

    def close(self):
        for fd, mapping in self._live.values():
            mapping.close()
            os.close(fd)
        self._live = {}
        if self._heap_fd is not None:
            os.close(self._heap_fd)
            self._heap_fd = None

    def describe(self):
        return {"heap": self.heap, "live_buffers": len(self._live)}


def _env_int(text):
    return int(str(text), 16)


class DevMemPlDevice:
    """Board device: /dev/mem register windows + a DMA buffer provider.

    UNTESTED ON HARDWARE (written 2026-09-27 before the KV260 arrived; the
    providers are exercised on the server with a mocked /dev, /sys, ioctl and
    pagemap). The buffer provider is chosen before /dev/mem is touched, so in
    auto mode a machine with no provider fails without mapping physical memory.
    """

    kind = "devmem"
    AUTO_ORDER = AUTO_ORDER
    region_provider_cls = _RegionProvider
    heap_provider_cls = _HeapProvider

    def __init__(self):
        self._windows, self._files = {}, []
        self._irq_lines = []
        self.provider = None
        self._buffers = None
        self.provider_attempts, self.provider_counts = [], Counter()
        self.config = {}

    # -- configuration ----------------------------------------------------
    @classmethod
    def _config(cls, env, **given):
        env = os.environ if env is None else env
        conf = {}
        for key, var in ENV_KEYS.items():
            value = given.get(key)
            if value is None and env.get(var, "") != "":
                value = env[var]
                if key in ("phys_base", "phys_size"):
                    value = _env_int(value)
            conf[key] = value
        conf["provider"] = conf["provider"] or "auto"
        conf["udmabuf"] = conf["udmabuf"] or "udmabuf0"
        if conf["provider"] not in ("auto",) + PROVIDERS:
            raise ValueError("DMA provider must be auto|%s, got %r"
                             % ("|".join(PROVIDERS), conf["provider"]))
        return conf

    @classmethod
    def _order(cls, provider):
        return cls.AUTO_ORDER if provider == "auto" else (provider,)

    # -- opening ----------------------------------------------------------
    @classmethod
    def open(cls, *, provider=None, udmabuf=None, heap=None, phys_base=None, phys_size=None,
             sysfs_root=None, dev_root="/dev", mem_path="/dev/mem", env=None, ops=None):
        conf = cls._config(env, provider=provider, udmabuf=udmabuf, heap=heap,
                           phys_base=phys_base, phys_size=phys_size)
        ops = SystemOps() if ops is None else ops
        roots = (sysfs_root,) if sysfs_root else UDMABUF_SYSFS_ROOTS
        device = cls()
        device.config = dict(conf)
        order = cls._order(conf["provider"])
        errors = {}
        for name in order:
            device.provider_counts["provider_tried:%s" % name] += 1
            try:
                device._buffers = getattr(device, "_open_" + name)(
                    conf, roots=roots, dev_root=dev_root, mem_path=mem_path, ops=ops)
            except (OSError, PlDeviceError, ValueError) as exc:
                errors[name] = str(exc)
                device.provider_counts["provider_failed:%s" % name] += 1
                device.provider_attempts.append((name, str(exc)))
                continue
            device.provider = name
            device.provider_counts["provider_selected:%s" % name] += 1
            device.provider_attempts.append((name, "ok"))
            break
        if device.provider is None:
            if "physmem" not in order:
                errors["physmem"] = ("not tried in auto mode: needs provider=physmem with "
                                     "phys_base/phys_size (%s)" % ENV_KEYS["provider"])
            message = "no DMA buffer provider: " + " ".join(
                "%s[%s]" % (name, errors.get(name, "not tried")) for name in PROVIDERS)
            raise PlDeviceUnavailable(message, attempts=device.provider_attempts,
                                      counts=device.provider_counts)
        try:
            mem = os.open(mem_path, os.O_RDWR | os.O_SYNC)
            device._files.append(mem)
            for block, base in BLOCK_BASE.items():
                device._windows[block] = mmap.mmap(mem, WINDOW_BYTES, mmap.MAP_SHARED,
                                                   mmap.PROT_READ | mmap.PROT_WRITE,
                                                   offset=base)
        except OSError as exc:
            device.close()
            raise PlDeviceUnavailable("cannot open PL registers (%s): %s" % (mem_path, exc),
                                      attempts=device.provider_attempts,
                                      counts=device.provider_counts) from exc
        return device

    @staticmethod
    def _barrier(ops):
        try:
            return ops.cache_ops().barrier
        except OSError:
            return None   # uncached providers still work; describe() says "none"

    def _open_udmabuf(self, conf, *, roots, dev_root, mem_path, ops):
        name = conf["udmabuf"]
        phys = size = None
        for root in roots:
            sysfs = Path(root) / name
            if (sysfs / "phys_addr").is_file():
                phys = int((sysfs / "phys_addr").read_text().strip(), 16)
                size = int((sysfs / "size").read_text().strip(), 10)
                break
        if phys is None or not size:
            raise PlDeviceError("no %s" % " or ".join(str(Path(r) / name) for r in roots))
        if phys + size > DMA_LIMIT:
            raise PlDeviceError("udmabuf region 0x%x+0x%x is outside the PL DMA window "
                                "(low 2 GiB)" % (phys, size))
        node = str(Path(dev_root) / name)
        fd = os.open(node, os.O_RDWR | os.O_SYNC)
        try:
            mapping = _Mapping(fd, size)
        except OSError:
            os.close(fd)
            raise
        return self.region_provider_cls("udmabuf", mapping, phys, [fd], self._barrier(ops))

    def _open_dmaheap(self, conf, *, roots, dev_root, mem_path, ops):
        heap_dir = Path(dev_root) / "dma_heap"
        if conf["heap"]:
            names = [conf["heap"]]
        else:
            present = sorted(p.name for p in heap_dir.iterdir()) if heap_dir.is_dir() else []
            names = [n for n in HEAP_CANDIDATES if n in present]
            names += [n for n in present if n not in HEAP_CANDIDATES and n != "system"]
            if not names:
                raise PlDeviceError("no CMA heap under %s (present: %s; tried %s; 'system' is "
                                    "not contiguous)" % (heap_dir, ",".join(present) or "none",
                                                         ", ".join(HEAP_CANDIDATES)))
        try:
            cache = ops.cache_ops()
        except OSError as exc:
            raise PlDeviceError("dmaheap needs cache maintenance, no helper: %s" % exc) from exc
        errors = []
        for heap in names:
            try:
                fd = os.open(str(heap_dir / heap), os.O_RDWR)
            except OSError as exc:
                errors.append("%s: open: %s" % (heap, exc))
                continue
            provider = self.heap_provider_cls(heap, fd, ops, cache)
            try:
                probe = provider.alloc(PAGE)   # PA must be readable now, not at frame 1
                provider.release(probe)
            except PlDeviceError as exc:
                provider.close()
                errors.append("%s: %s" % (heap, exc))
                continue
            return provider
        raise PlDeviceError("; ".join(errors))

    def _open_physmem(self, conf, *, roots, dev_root, mem_path, ops):
        base, size = conf["phys_base"], conf["phys_size"]
        if not size:
            raise PlDeviceError("needs phys_base/phys_size (%s/%s)"
                                % (ENV_KEYS["phys_base"], ENV_KEYS["phys_size"]))
        base, size = int(base or 0), int(size)
        if base % PAGE or size % PAGE:
            raise PlDeviceError("physmem window 0x%x+0x%x is not page aligned" % (base, size))
        if base + size > DMA_LIMIT:
            raise PlDeviceError("physmem window 0x%x+0x%x is outside the PL DMA window "
                                "(low 2 GiB)" % (base, size))
        fd = os.open(mem_path, os.O_RDWR | os.O_SYNC)
        try:
            mapping = _Mapping(fd, size, base)
        except OSError:
            os.close(fd)
            raise
        return self.region_provider_cls("physmem", mapping, base, [fd], self._barrier(ops))

    def describe(self):
        info = {"provider": self.provider, "config": dict(self.config),
                "cached": bool(self._buffers and self._buffers.cached),
                "barrier": "dsb" if getattr(self._buffers, "barrier", None) or (
                    self._buffers is not None and self._buffers.cached) else "none",
                "attempts": ["%s: %s" % item for item in self.provider_attempts],
                "counts": dict(self.provider_counts)}
        if self._buffers is not None:
            info.update(self._buffers.describe())
        return info

    def irq(self, block, *, sysfs_root=UIO_SYSFS, dev_root="/dev"):
        """The block's UIO interrupt line; PlDeviceUnavailable if there is none."""
        if block not in UIO_NODE:
            raise PlDeviceUnavailable("no interrupt line known for block %r" % (block,))
        line = UioIrq.open(UIO_NODE[block], sysfs_root, dev_root)
        self._irq_lines.append(line)
        return line

    def close(self):
        for line in getattr(self, "_irq_lines", []):
            line.close()
        self._irq_lines = []
        for window in self._windows.values():
            window.close()
        if self._buffers is not None:
            self._buffers.close()
        for fd in self._files:
            os.close(fd)
        self._windows, self._files, self._buffers = {}, [], None

    # -- device interface --------------------------------------------------
    def rd(self, block, off):
        return struct.unpack_from("<I", self._windows[block], off)[0]

    def wr(self, block, off, value):
        struct.pack_into("<I", self._windows[block], off, int(value) & 0xFFFFFFFF)

    def is_idle(self, block):
        return bool(self.rd(block, 0) & AP_IDLE)

    def alloc(self, nbytes):
        if self._buffers is None:
            raise PlDeviceError("device is closed")
        self.provider_counts["alloc"] += 1
        try:
            return self._buffers.alloc(nbytes)
        except PlDeviceError:
            self.provider_counts["alloc_failed"] += 1
            raise

    def release(self, buf):
        self.provider_counts["release"] += 1
        self._buffers.release(buf)

    def to_device(self, buf):
        self._buffers.to_device(buf)

    def from_device(self, buf):
        self._buffers.from_device(buf)
