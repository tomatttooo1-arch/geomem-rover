"""Explicit M7 backend selection: ``backend="cpu"``, ``"pl"`` or ``"remote_pl"`` (2026-09-27).

``"remote_pl"`` (2026-10-11) is the frame core on the KV260 reached from the
Nano over UDP (host/kv_link.py); frame only. ``pl_wait`` picks how the local
PL backends wait for ap_done: "poll" (spin) or "irq" (UIO interrupt).

One consumer interface. The PL backends subclass the CPU classes and replace
only the final core call, so input checks, output validation and error
semantics are inherited unchanged. There is no "auto" and no fallback: asking
for "pl" when no device can be opened raises ``PlBackendUnavailable`` and is
counted; a device failure during a call raises ``PlDeviceError`` (a
RuntimeError, which M7Pipeline turns into an invalid frame) and is counted.
See docs/design/M7_BACKEND_SWITCH_20260927.md.
"""

from collections import Counter
import struct
import time

import numpy as np

from pathlib import Path

from host.m7_frame_native import FrameNativeM7
from host.m7_frame_fx import FrameFx, compile_frame_fx
from host.m7_object_native import NativeObjectEvidence
from host.m7_pl_device import (FRAME, OBJECT, FRAME_REGS, FX_STAT_REGS, OBJECT_REGS,
                               OBJECT_COUNT_REGS, FRAME_GRID_CAPACITY, AP_START,
                               AP_DONE, IRQ_REGS, WAIT_MODES, PlDeviceError,
                               PlDeviceUnavailable)

CPU_BACKEND_NAME = "cpu_reference"
BACKENDS = ("cpu", "pl", "remote_pl")


class PlBackendUnavailable(RuntimeError):
    """``backend="pl"`` was requested but no PL device/core is available."""


class _PlCore:
    """Shared register choreography for one HLS AXI shell."""

    def __init__(self, device, block, regs, *, timeout_s, counts, wait="poll"):
        """``wait``: "poll" spins on AP_CTRL (one PS core busy for the whole
        device time); "irq" sleeps in poll() on the block's UIO interrupt.
        Explicit, no fallback: "irq" without an interrupt line raises
        PlDeviceUnavailable here, at construction."""
        if wait not in WAIT_MODES:
            raise ValueError("wait must be one of %s, got %r" % (WAIT_MODES, wait))
        self.device, self.block, self.regs = device, block, dict(regs)
        self.timeout_s, self.counts = float(timeout_s), counts
        self.wait = wait
        self.irq = None
        if wait == "irq":
            get = getattr(device, "irq", None)
            if get is None:
                raise PlDeviceUnavailable("%s device has no interrupt lines" % device.kind)
            self.irq = get(block)
        self._buffers = {}

    def buffer(self, name, nbytes):
        buf = self._buffers.get(name)
        if buf is None or buf.size < nbytes:
            if buf is not None:
                self.device.release(buf)
            buf = self.device.alloc(max(int(nbytes), 1))
            self._buffers[name] = buf
        return buf

    def wr(self, name, value):
        self.device.wr(self.block, self.regs[name], int(value) & 0xFFFFFFFF)

    def wr64(self, name, value):
        value = int(value)
        self.device.wr(self.block, self.regs[name], value & 0xFFFFFFFF)
        self.device.wr(self.block, self.regs[name] + 4, (value >> 32) & 0xFFFFFFFF)

    def wr_f64(self, name, value):
        self.wr64(name, struct.unpack("<Q", struct.pack("<d", float(value)))[0])

    def wr_array(self, base_name, values, fmt):
        raw = np.ascontiguousarray(values, dtype="<" + fmt).tobytes()
        base = self.regs[base_name]
        for i in range(len(raw) // 4):
            self.device.wr(self.block, base + 4 * i, struct.unpack_from("<I", raw, 4 * i)[0])

    def rd(self, name):
        return self.device.rd(self.block, self.regs[name])

    def run(self):
        if not self.device.is_idle(self.block):
            self.counts["device_busy"] += 1
            raise PlDeviceError("%s core is not idle" % self.block)
        self.counts["calls"] += 1
        if self.wait == "irq":
            return self._run_irq()
        self.device.wr(self.block, self.regs["AP_CTRL"], AP_START)
        deadline = time.monotonic() + self.timeout_s
        while True:
            if self.device.rd(self.block, self.regs["AP_CTRL"]) & AP_DONE:
                return
            if time.monotonic() > deadline:
                self.counts["device_timeout"] += 1
                raise PlDeviceError("%s core did not finish within %.3f s"
                                    % (self.block, self.timeout_s))


    def _clear_isr(self):
        isr = IRQ_REGS["ISR"]
        self.device.wr(self.block, isr, self.device.rd(self.block, isr))   # toggle-on-write

    def _run_irq(self):
        """m7pl_test.cpp cmd_irq order: clear ISR, enable the UIO line, GIE/IER,
        ap_start, poll(), read the count, consume ap_done, clear ISR."""
        dev, blk = self.device, self.block
        self._clear_isr()          # a stale ISR would wake the line at once
        self.irq.arm()
        dev.wr(blk, IRQ_REGS["GIE"], 1)
        dev.wr(blk, IRQ_REGS["IER"], 1)
        dev.wr(blk, self.regs["AP_CTRL"], AP_START)
        deadline = time.monotonic() + self.timeout_s
        try:
            while True:
                left = deadline - time.monotonic()
                if left <= 0 or self.irq.wait(left) is None:
                    self.counts["device_timeout"] += 1
                    raise PlDeviceError("%s core did not interrupt within %.3f s"
                                        % (self.block, self.timeout_s))
                self.counts["irq_wakeups"] += 1
                # Clear ISR before reading ap_done: a completion after the read
                # sets ISR again and fires the re-armed line, so none is lost.
                self._clear_isr()
                if dev.rd(blk, self.regs["AP_CTRL"]) & AP_DONE:
                    return
                self.counts["irq_spurious"] += 1
                self.irq.arm()
        finally:
            dev.wr(blk, IRQ_REGS["IER"], 0)
            dev.wr(blk, IRQ_REGS["GIE"], 0)


# WB_BURST (nationals, in m7_pl_m2 / m7_pl_m2_166 and later): the geometry core writes the flags grid back as
# 32-bit words in one m_axi burst, so the buffer behind FLAGS_DATA must start on a 4-byte boundary. The circuit
# does not check it (a misaligned address would write shifted bytes), so the host checks every write of
# FLAGS_DATA and fails the call loudly, with the address, before touching the core (SW_M2 2026-11-03).
FLAGS_PA_ALIGN = 4


def check_flags_pa(pa):
    """Raise PlDeviceError naming ``pa`` when it is not FLAGS_PA_ALIGN-byte aligned."""
    pa = int(pa)
    if pa % FLAGS_PA_ALIGN:
        raise PlDeviceError("FLAGS_DATA physical address 0x%x is not %d-byte aligned (geometry write-back is "
                            "32-bit bursts since WB_BURST; the core does not check it)" % (pa, FLAGS_PA_ALIGN))
    return pa


class _PlFrameCall(_PlCore):
    """Same signatures as the ctypes ``fpga_m7_depth_frame`` used by FrameNativeM7, plus the codes call.

    GEO_FX3 (2026-11-04): the frame core is the fixed-point core (GEO_FX1/GEO_FX2 X1, GEO_FX3 fx_k array).
    It reads only 16-bit codes; its per-frame constants come from the PS (``host.m7_frame_fx``).
      fpga_m7_depth_frame_codes  the only PL path: PS constants (C++), registers, start, counts, statistics,
                                depth crc. A frame whose PS status is not 0 (2/3 like the reference, 5 =
                                outside the fixed-point envelope) or a core without the fixed-point
                                registers runs the CPU reference on the A53 instead (counted, never refused).
      fpga_m7_depth_frame        float32 metres: the float input of the PL is removed, so this is the CPU
                                reference on the A53 (kv_link uses it only when the unit fails d16_unit_ok).
    """

    dlat2_active = False                # PlFrameBackend sets it from setup_dlat2
    fx_active = False                   # PlFrameBackend sets it from setup_fx
    fx = None                           # host.m7_frame_fx.FrameFx
    cpu_frame = None                    # ctypes fpga_m7_depth_frame of the CPU reference (FrameNativeM7)
    last_stats = None                   # [ST_NPOS, ST_LE20, ST_MM0, ST_MM1] of the last PL frame
    last_fx = None                      # PS summary of the last codes frame

    def fpga_m7_depth_frame(self, depth_full, inverse_k, world_camera, origin, voxel,
                           width, height, nx, ny, nz, flags, counts, status):
        t0 = time.perf_counter()
        self.counts["cpu_reference_float"] += 1
        self.cpu_frame(depth_full, inverse_k, world_camera, origin, voxel, width, height, nx, ny, nz,
                       flags, counts, status)
        self.last_stats = None
        self.last_times = {"prep_s": 0.0, "device_s": 0.0, "readback_s": time.perf_counter() - t0}

    def fpga_m7_depth_frame_codes(self, codes, unit_bits, inverse_k, world_camera, origin, voxel,
                                 width, height, nx, ny, nz, flags, counts, status):
        """The frame from the camera's uint16 codes. ``unit_bits``: float32 bits of the unit (callers check
        d16_unit_ok). Returns the depth crc: zlib crc32 of the float32 depth f32(code) * unit (code 0 ->
        NaN) -- from the core's DEPTH_CRC, or made on the A53 when the CPU reference ran; 0 when the core
        status is not 0 (kv_link then makes it on the A53)."""
        t0 = time.perf_counter()
        pixels = int(width) * int(height)
        k = self.fx.prepare(width, height, nx, ny, nz, unit_bits, inverse_k, world_camera, origin, voxel)
        self.last_fx = {"status": k["status"], "pmax": k["pmax"], "reach": k["reach"],
                        "thresholds": k["thresholds"]}
        if k["status"] != 0 or not self.fx_active:
            self.counts["cpu_reference:%s" % ("fx_status_%d" % k["status"] if self.fx_active
                                               else "no_fx_core")] += 1
            from host.kv_link import codes_to_m, crc32
            depth = codes_to_m(np.ascontiguousarray(codes, dtype="<u2").reshape(-1)[:pixels],
                               np.float32(struct.unpack("<f", struct.pack("<I", int(unit_bits) & 0xFFFFFFFF))[0]))
            full = np.zeros(max(pixels, 1), dtype=np.float32)
            full[:pixels] = depth
            self.cpu_frame(full, inverse_k, world_camera, origin, voxel, width, height, nx, ny, nz,
                           flags, counts, status)
            self.last_stats = None
            self.last_times = {"prep_s": time.perf_counter() - t0, "device_s": 0.0, "readback_s": 0.0}
            return int(crc32(depth)) if int(status[0]) == 0 else 0
        depth_buf = self.buffer("depth", 4 * max(pixels, 1))
        flags_buf = self.buffer("flags", FRAME_GRID_CAPACITY)
        try:
            check_flags_pa(flags_buf.pa)        # SW_M2: before any write, the frame fails loudly
        except PlDeviceError:
            self.counts["flags_misaligned"] += 1
            raise
        depth_buf.array[:2 * pixels] = np.ascontiguousarray(codes, dtype="<u2").reshape(-1)[:pixels].view(np.uint8)
        self.wr("DEPTH_FMT_DATA", 1)
        self.wr("DEPTH_UNIT_DATA", int(unit_bits) & 0xFFFFFFFF)
        for i, w in enumerate(k["words"]):
            self.device.wr(self.block, self.regs["FX_K_BASE"] + 4 * i, int(w))
        for name, value in zip(("FX_ORIGIN_X_DATA", "FX_ORIGIN_Y_DATA", "FX_ORIGIN_Z_DATA"), k["origin"]):
            self.wr(name, value)
        self.wr("FX_CODES_DATA", k["codes"])
        self.wr("FX_C20_DATA", k["c20"])
        self._finish(t0, depth_buf, flags_buf, width, height, nx, ny, nz, flags, counts, status)
        self.counts["fx_frames"] += 1
        self.last_stats = [int(self.rd(n)) for n in FX_STAT_REGS]
        return int(self.rd("DEPTH_CRC_DATA"))

    def _finish(self, t0, depth_buf, flags_buf, width, height, nx, ny, nz, flags, counts, status):
        # Pre-load the caller's sentinel: the core writes flags only on
        # success, so an untouched buffer reads back exactly as it went in.
        flags_buf.array[:FRAME_GRID_CAPACITY] = flags[:FRAME_GRID_CAPACITY]
        self.device.to_device(depth_buf)
        self.device.to_device(flags_buf)
        self.wr64("DEPTH_M_DATA", depth_buf.pa)
        self.wr64("FLAGS_DATA", flags_buf.pa)               # alignment checked above (check_flags_pa)
        # GEO_FX3: INVERSE_K / WORLD_CAMERA / GRID_ORIGIN / VOXEL_SIZE are still registers of the core but
        # it never reads them (its constants are FX_K / FX_ORIGIN / FX_CODES / FX_C20): not written.
        for name, value in (("WIDTH_DATA", width), ("HEIGHT_DATA", height),
                            ("NX_DATA", nx), ("NY_DATA", ny), ("NZ_DATA", nz)):
            self.wr(name, value)
        if self.dlat2_active:           # INT_SW: only when dlat2=1 was asked AND read back (next bitstream)
            self.wr("DLAT2_EN_DATA", 1)
        t1 = time.perf_counter()
        self.run()
        t2 = time.perf_counter()
        self.device.from_device(flags_buf)
        flags[:FRAME_GRID_CAPACITY] = flags_buf.array[:FRAME_GRID_CAPACITY]
        # [accepted, invalid, clipped (被裁剪; register keeps the name COUNT_OUTSIDE)]
        counts[:] = [self.rd("COUNT_ACCEPTED_DATA"), self.rd("COUNT_INVALID_DATA"),
                     self.rd("COUNT_OUTSIDE_DATA")]
        status[0] = self.rd("AP_RETURN")
        if status[0] != 0:
            self.counts["core_rejected"] += 1
        # Seconds of the last call: buffer copies + register writes, ap_start ->
        # done (poll or interrupt), read-back. Same clock as the caller's.
        self.last_times = {"prep_s": t1 - t0, "device_s": t2 - t1,
                           "readback_s": time.perf_counter() - t2}


class _PlObjectCall(_PlCore):
    """Same signature as the ctypes ``fpga_m7_object_evidence``."""

    def fpga_m7_object_evidence(self, depth, mask, width, height, mask_present,
                               roi_enabled, roi, near_z, far_z, margin, inverse_k,
                               local_camera, min_pixels, min_valid_fraction,
                               free_fraction, points, counts, status):
        pixels = int(width) * int(height)
        depth_buf = self.buffer("depth", 4 * pixels)
        mask_buf = self.buffer("mask", pixels)
        xyz_buf = self.buffer("xyz", 24 * pixels)
        depth_buf.array[:4 * pixels] = np.ascontiguousarray(depth, dtype="<f4").view(
            np.uint8).ravel()
        mask_buf.array[:pixels] = np.ascontiguousarray(mask, dtype=np.uint8).ravel()
        self.device.to_device(depth_buf)
        self.device.to_device(mask_buf)
        self.wr64("DEPTH_DATA", depth_buf.pa)
        self.wr64("MASK_DATA", mask_buf.pa)
        self.wr64("XYZ_LOCAL_DATA", xyz_buf.pa)
        for name, value in (("WIDTH_DATA", width), ("HEIGHT_DATA", height),
                            ("MASK_PRESENT_DATA", mask_present),
                            ("ROI_ENABLED_DATA", roi_enabled),
                            ("ROI_X0_DATA", roi[0]), ("ROI_Y0_DATA", roi[1]),
                            ("ROI_X1_DATA", roi[2]), ("ROI_Y1_DATA", roi[3]),
                            ("MIN_PIXELS_DATA", min_pixels)):
            self.wr(name, value)
        for name, value in (("NEAR_Z_DATA", near_z), ("FAR_Z_DATA", far_z),
                            ("DEPTH_MARGIN_M_DATA", margin),
                            ("MIN_VALID_FRACTION_DATA", min_valid_fraction),
                            ("FREE_FRACTION_DATA", free_fraction)):
            self.wr_f64(name, value)
        self.wr_array("INVERSE_K_BASE", inverse_k, "f8")
        self.wr_array("LOCAL_CAMERA_BASE", local_camera, "f8")
        self.run()
        status[0] = self.rd("AP_RETURN")
        if status[0] != 0:
            # Shell contract: on nonzero status the core leaves xyz and counts
            # untouched and the shell's count registers read 0. The core ABI
            # the CPU class checks leaves the caller's arrays untouched, so
            # nothing is copied back.
            self.counts["core_rejected"] += 1
            return
        values = [self.rd(name) for name in OBJECT_COUNT_REGS]
        counts[:] = values
        valid = min(int(values[2]), pixels)
        self.device.from_device(xyz_buf)
        points[:3 * valid] = xyz_buf.array[:24 * valid].view("<f8")


D16_MODES = (0, 1)


def d16_unit_ok(unit_bits):
    """D16 driver precondition on the code unit (float32 bits): positive, normal (exponent field 1..0xee),
    so that 1 * unit .. 65535 * unit are all normal and finite. There the PL multiplier (Xilinx
    floating-point core: no subnormals) and IEEE single multiplication agree on every code (RUN/D16
    d16_exhaustive + co-simulation over all 65536 codes); outside it the A53 path is used."""
    b = int(unit_bits) & 0xFFFFFFFF
    e = (b >> 23) & 0xFF
    return not (b >> 31) and 1 <= e <= 0xEE


def setup_fx(core):
    """GEO_FX3: is the loaded frame core the fixed-point core? Probe = write a pattern to FX_CODES_DATA
    (a write-only-by-driver, read-back-able register after DEPTH_CRC) and read it back. Older cores end at
    0x158 (D16) or earlier: the offset reads 0 and drops the write. "on" -> every codes frame runs on the PL;
    "absent" -> a warning, counted, every frame on the CPU reference (owner rule: warn and keep going).
    The probe leaves FX_CODES at 0 (every frame start rewrites it). Returns (state, active)."""
    pattern = 0x2710_0064
    core.wr("FX_CODES_DATA", pattern)
    ok = core.rd("FX_CODES_DATA") == pattern
    core.wr("FX_CODES_DATA", 0)
    if ok:
        return "on", True
    core.counts["fx_core_absent"] += 1
    import sys
    print("WARNING: the loaded frame core has no fixed-point registers (FX_CODES read back wrong): "
          "every geometry frame runs on the CPU reference", file=sys.stderr, flush=True)
    return "absent", False


DLAT2_MODES = (0, 1)


def setup_dlat2(core, want):
    """G1d Dlat2 ray thinning switch of the next bitstream's frame core (INT_SW 2026-11-02).

    Register DLAT2_EN_DATA (0x13c, the last s_axilite entry, reset 0 = current behaviour) exists only in
    the next bitstream; on a current one the offset is unmapped (an HLS s_axilite slave answers 0 to the
    read and drops the write; its address space already reaches 0x130, so 0x13c does not alias). Version
    probe = read back:
      want 0  read the register once; 0 -> nothing is written, ever (current bitstreams and a freshly
              loaded next bitstream: exactly the old register choreography). Non-zero can only be a next
              bitstream left at 1 by an earlier process -> write 0 and read back ("cleared").
      want 1  write 1 and read back. 1 -> "on": every frame start re-writes 1 (one register write).
              Otherwise the bitstream has no Dlat2 -> "unsupported": a warning line, counted, frames run
              exactly as with want 0 (owner rule: warn and keep going, no refusal).
    Returns (state, active)."""
    if want not in DLAT2_MODES:
        raise ValueError("dlat2 must be 0 or 1, got %r" % (want,))
    if "DLAT2_EN_DATA" not in core.regs:
        return ("no_register_in_map" if want else "untouched"), False
    if want == 0:
        if core.rd("DLAT2_EN_DATA") == 0:
            return "untouched", False
        core.wr("DLAT2_EN_DATA", 0)
        core.counts["dlat2_cleared"] += 1
        return ("cleared" if core.rd("DLAT2_EN_DATA") == 0 else "clear_failed"), False
    core.wr("DLAT2_EN_DATA", 1)
    if core.rd("DLAT2_EN_DATA") == 1:
        return "on", True
    core.counts["dlat2_unsupported"] += 1
    import sys
    print("WARNING: dlat2=1 asked but the loaded frame core has no DLAT2_EN register (read back 0): "
          "running without ray thinning", file=sys.stderr, flush=True)
    return "unsupported", False


class PlFrameBackend(FrameNativeM7):
    """FrameNativeM7 with the core call routed through a PL device.

    ``dlat2`` (INT_SW 2026-11-02): 0 (default) or 1, see ``setup_dlat2``.
    GEO_FX3: the PL frame core is the fixed-point core (16-bit codes only). ``cpu_library`` (the CPU
    reference, host.m7_frame_native) and ``fx_library`` (host.m7_frame_fx) are built into a temporary
    directory when not given."""

    def __init__(self, device, *, timeout_s=2.0, regs=FRAME_REGS, wait="poll", dlat2=0,
                 cpu_library=None, fx_library=None):
        self.counts = Counter()
        self.device = device
        self.backend_name = "pl:%s" % device.kind
        cpu_library, fx_library = _frame_libraries(cpu_library, fx_library)
        self.lib = _PlFrameCall(device, FRAME, regs, timeout_s=timeout_s, counts=self.counts,
                                wait=wait)
        self.lib.cpu_frame = FrameNativeM7(cpu_library).lib.fpga_m7_depth_frame
        self.lib.fx = FrameFx(fx_library)
        self.wait_mode = self.lib.wait
        self.dlat2_state, self.lib.dlat2_active = setup_dlat2(self.lib, int(dlat2))
        self.fx_state, self.lib.fx_active = setup_fx(self.lib)


def _frame_libraries(cpu_library, fx_library):
    if cpu_library is not None and fx_library is not None:
        return cpu_library, fx_library
    import tempfile
    from host.m7_frame_native import compile_frame_native
    d = Path(tempfile.mkdtemp(prefix="m7_frame_libs_"))
    if cpu_library is None:
        cpu_library = compile_frame_native(d / "m7_frame.so")
    if fx_library is None:
        fx_library = compile_frame_fx(d / "m7_frame_fx.so")
    return cpu_library, fx_library


# Legacy object core = control slot 2 (0xA0020000). The v3a bitstream
# backend on a board must first see slot 2 in sysinfo SLOT_MAP (0x20); without
# this check a call on v3a would start a core in an empty address window.
OBJECT_SLOT = 2
SYSINFO_BASE, SYSINFO_MAGIC_VALUE, SYSINFO_SLOT_MAP = 0xA0000000, 0x4D37504C, 0x20


def read_board_slot_map(mem_path="/dev/mem"):
    """sysinfo SLOT_MAP via /dev/mem (read-only); PlDeviceUnavailable if not M7 sysinfo."""
    import mmap
    import os
    try:
        fd = os.open(mem_path, os.O_RDONLY | os.O_SYNC)
    except OSError as exc:
        raise PlDeviceUnavailable("cannot read sysinfo SLOT_MAP (%s): %s" % (mem_path, exc)) from exc
    try:
        win = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ, offset=SYSINFO_BASE)
        try:
            magic = struct.unpack_from("<I", win, 0)[0]
            slot_map = struct.unpack_from("<I", win, SYSINFO_SLOT_MAP)[0]
        finally:
            win.close()
    except OSError as exc:
        raise PlDeviceUnavailable("cannot map sysinfo at %#x: %s" % (SYSINFO_BASE, exc)) from exc
    finally:
        os.close(fd)
    if magic != SYSINFO_MAGIC_VALUE:
        raise PlDeviceUnavailable("sysinfo MAGIC %#010x != %#010x: unknown bitstream"
                                  % (magic, SYSINFO_MAGIC_VALUE))
    return slot_map


class PlObjectBackend(NativeObjectEvidence):
    """NativeObjectEvidence with the core call routed through a PL device.

    ``slot_map``: sysinfo SLOT_MAP of the loaded bitstream. When omitted it is
    read from the board for a ``devmem`` device (mock devices always carry the
    object core). Slot 2 absent -> PlDeviceUnavailable, before any register
    write (the selector turns it into PlBackendUnavailable).
    """

    slot_map_reader = staticmethod(read_board_slot_map)

    def __init__(self, device, *, timeout_s=2.0, regs=OBJECT_REGS, wait="poll", slot_map=None):
        if slot_map is None and getattr(device, "kind", None) == "devmem":
            slot_map = self.slot_map_reader()
        if slot_map is not None and not (slot_map >> OBJECT_SLOT) & 1:
            raise PlDeviceUnavailable("bitstream has no legacy object core (SLOT_MAP %#x, bit %d "
                                      "clear; removed in v3a)" % (slot_map, OBJECT_SLOT))
        self.counts = Counter()
        self.device = device
        self.backend_name = "pl:%s" % device.kind
        self.lib = _PlObjectCall(device, OBJECT, regs, timeout_s=timeout_s, counts=self.counts,
                                 wait=wait)
        self.wait_mode = self.lib.wait


def _label(backend):
    backend.backend_name = CPU_BACKEND_NAME
    return backend


class M7BackendSelector:
    """Build frame/object/rolling backends by explicit name, never falling back.

    ``pl_device_factory`` is a zero-argument callable returning a PL device
    (``MockPlDevice`` in tests, ``DevMemPlDevice.open`` on the board); it may
    raise ``PlDeviceUnavailable``. The device is opened once and shared.
    """

    def __init__(self, *, cpu_frame_library=None, cpu_object_library=None,
                 pl_device_factory=None, pl_timeout_s=2.0, pl_wait="poll",
                 remote_pl_factory=None, pl_rolling_factory=None, remote_rolling_factory=None,
                 pl_frame_dlat2=0, pl_frame_fx_library=None):
        if pl_wait not in WAIT_MODES:
            raise ValueError("pl_wait must be one of %s, got %r" % (WAIT_MODES, pl_wait))
        self.cpu_frame_library = cpu_frame_library
        self.cpu_object_library = cpu_object_library
        self.pl_device_factory = pl_device_factory
        self.pl_timeout_s = pl_timeout_s
        self.pl_wait = pl_wait
        self.pl_frame_dlat2 = int(pl_frame_dlat2)     # INT_SW: G1d Dlat2 switch of the PL frame core
        self.pl_frame_fx_library = pl_frame_fx_library  # GEO_FX3: host.m7_frame_fx library (None: built)
        self.remote_pl_factory = remote_pl_factory
        # v3 L2 (2026-10-26, host/kv_rolling.py): zero-argument callables returning a
        # ready rolling backend -- the KV-local PL core (CoreRollingBackend over
        # PlRollingCore) / the Nano's RemotePlRollingBackend. None -> that name is
        # unavailable (PlBackendUnavailable), never a fallback to cpu.
        self.pl_rolling_factory = pl_rolling_factory
        self.remote_rolling_factory = remote_rolling_factory
        self.counts = Counter()
        self._device = None

    @staticmethod
    def _check(name):
        if name not in BACKENDS:
            raise ValueError("backend must be one of %s, got %r" % (BACKENDS, name))

    def _pl_device(self, what):
        if self._device is not None:
            return self._device
        if self.pl_device_factory is None:
            self.counts["pl_unavailable:%s" % what] += 1
            raise PlBackendUnavailable("backend='pl' for %s: no PL device factory configured" % what)
        try:
            self._device = self.pl_device_factory()
        except PlDeviceUnavailable as exc:
            self.counts["pl_unavailable:%s" % what] += 1
            raise PlBackendUnavailable("backend='pl' for %s: %s" % (what, exc)) from exc
        return self._device

    def _remote_frame(self):
        """backend="remote_pl": the frame core on the KV260 over host/kv_link.py.

        ``remote_pl_factory`` is a zero-argument callable returning a ready
        ``RemotePlFrameBackend``; it may raise PlBackendUnavailable (no
        server / no answer to the handshake). No fallback to a local core.
        """
        if self.remote_pl_factory is None:
            self.counts["pl_unavailable:frame"] += 1
            raise PlBackendUnavailable("backend='remote_pl' for frame: no remote factory configured")
        try:
            return self.remote_pl_factory()
        except PlBackendUnavailable:
            self.counts["pl_unavailable:frame"] += 1
            raise

    def frame_backend(self, backend):
        self._check(backend)
        self.counts["selected:frame:%s" % backend] += 1
        if backend == "cpu":
            if self.cpu_frame_library is None:
                raise ValueError("backend='cpu' needs cpu_frame_library")
            return _label(FrameNativeM7(self.cpu_frame_library))
        if backend == "remote_pl":
            return self._remote_frame()
        device = self._pl_device("frame")
        try:
            return PlFrameBackend(device, timeout_s=self.pl_timeout_s, wait=self.pl_wait,
                                  dlat2=self.pl_frame_dlat2, cpu_library=self.cpu_frame_library,
                                  fx_library=self.pl_frame_fx_library)
        except PlDeviceUnavailable as exc:
            self.counts["pl_unavailable:frame"] += 1
            raise PlBackendUnavailable("backend='pl' (wait=%s) for frame: %s"
                                       % (self.pl_wait, exc)) from exc

    def object_backend(self, backend):
        self._check(backend)
        self.counts["selected:object:%s" % backend] += 1
        if backend == "cpu":
            if self.cpu_object_library is None:
                raise ValueError("backend='cpu' needs cpu_object_library")
            return _label(NativeObjectEvidence(self.cpu_object_library))
        if backend == "remote_pl":
            self.counts["pl_unavailable:object"] += 1
            raise PlBackendUnavailable("backend='remote_pl' exists for the frame core only")
        device = self._pl_device("object")
        try:
            return PlObjectBackend(device, timeout_s=self.pl_timeout_s, wait=self.pl_wait)
        except PlDeviceUnavailable as exc:
            self.counts["pl_unavailable:object"] += 1
            raise PlBackendUnavailable("backend='pl' (wait=%s) for object: %s"
                                       % (self.pl_wait, exc)) from exc

    def rolling_backend(self, backend, *, cpu_library=None):
        """cpu: RollingKernelHost (the car default). pl / remote_pl: only through the
        factories (the rolling core exists only in the v3 candidate bitstream m7_pl_v3f,
        slot 5; on any other bitstream the factory itself refuses)."""
        self._check(backend)
        self.counts["selected:rolling:%s" % backend] += 1
        if backend in ("pl", "remote_pl"):
            factory = self.pl_rolling_factory if backend == "pl" else self.remote_rolling_factory
            if factory is None:
                self.counts["pl_unavailable:rolling"] += 1
                raise PlBackendUnavailable(
                    "backend=%r for rolling: no rolling factory configured (the rolling core "
                    "is only in the v3 candidate bitstream m7_pl_v3f)" % backend)
            try:
                return factory()
            except PlBackendUnavailable:
                self.counts["pl_unavailable:rolling"] += 1
                raise
        from host.m7_rolling import RollingKernelHost
        if cpu_library is None:
            raise ValueError("backend='cpu' for rolling needs cpu_library")
        return RollingKernelHost(cpu_library)
