"""PL driver for the E2 object-ROI core (``--roi-core pl``), 2026-10-11.

The core is ``fpga_m7_object_roi_axi`` (hw/m7/pl/hls/m7_object_roi_axi.cpp, OBJECT_ROI_SPEC
§3/§11.5) as wired by the merged bitstream m7_pl_merge (feat/dev1/pl-merge-20261011
7784f75, BUILD_ID 0x07784f75): control slot 9 at 0xA009_0000, GIC SPI 91 (pl_ps_irq0 bit 2),
DMA through HP1, generic-uio node ``m7_roi`` (hw/m7/pl/overlay/m7_pl.dtso).

Rules (PC_DATAPLANE A.2/A.3, kv_regs.py):

* Registers only through the UIO node's map0 (``/dev/uioN``), never a physical-address
  window. The HLS core has no MAGIC register, so the gate is the MAGIC-equivalent:
  the sysinfo block is up (its own UIO + MAGIC 0x4D37504C), sysinfo SLOT_MAP (0x20) has
  the ROI slot bit (bit 9; 0x3D7 on 0x07784f75, 0x1F on 0x0117bc0a), the ``m7_roi`` node
  exists with map0 at 0xA009_0000 and >= 4 KiB, and AP_CTRL reads idle. Any miss ->
  ``RoiUnavailable`` and kv_server refuses to start (no fallback to cpu).
* Depth goes in a DMA buffer of the geometry device's provider (dmaheap ``reserved`` on
  the board; physical address < 2 GiB), allocated once at 1280x800 and reused; each call
  copies the frame's float32 depth into it (the copy is ``times_ns.prep``).
* Completion by polling AP_CTRL.ap_done (GIE/IER stay 0; the line is wired but unused) -- ``wait="poll"``,
  the default, register traffic exactly as before.
* SW_M1 (2026-11-03) ``wait="irq"`` (kv_server --roi-wait irq): the core's interrupt (GIC SPI 91, the same UIO
  node m7_roi, a second fd for the event count) as the geometry core's _run_irq does it: clear ISR, arm the UIO
  line, GIE/IER 1, ap_start, block in poll() (no CPU, the GIL released) for at most ``irq_slice_s``, clear ISR,
  then read ap_done. Why: with --roi-concurrent 1 the Python busy poll (~350 register reads per frame) held the
  interpreter lock while the geometry thread's interrupt had already arrived; CPython only forces a hand-over
  every 5 ms, so the geometry answer waited (BOARD2 3.5: geometry 3.39 -> 7.33 ms, total service only -0.24 ms).
  A lost interrupt costs at most one slice: when poll() times out, ap_done is read anyway (counted
  ``irq_rescued``); the call fails only at ``timeout_s`` as before. Spurious wake-ups re-arm (``irq_spurious``).
* Read-back per OR §11.5: the result table is an s_axilite output RAM the host cannot
  preset, so rows past ``box_count`` and the whole table on AP_RETURN != 0 hold the
  previous call's values -- they are never read. Only status 0 reads the first
  ``box_count`` rows. The Nano still checks every row (check_roi_answer).

``MockRoiWindow`` is the host stand-in: registers in a dict, ap_start runs the same g++
``fpga_m7_object_roi_batch`` on the depth it finds at the DEPTH physical address in the
geometry mock device's memory, and writes only what the PL shell writes. Two mutants
(positive controls): ``row_shift`` (rows land one slot off) and ``threshold`` (one box's
near_z off by a margin -- a lane with a wrong key: counts stay self-consistent).
Python 3.8 compatible.
"""

from collections import Counter
import os
import struct
import threading
import time

import numpy as np

ROI_BASE = 0xA0090000
ROI_SLOT = 9
ROI_UIO = "m7_roi"
ROI_MIN_MAP = 0x1000           # 12-bit s_axilite: the result table ends at 0xBFF
SYS_SLOT_MAP = 0x20
MAX_BOXES = 32
FIELDS = 7
MAX_PIXELS = 1280 * 800
AP_START, AP_DONE, AP_IDLE = 0x1, 0x2, 0x4

# hw/m7/pl/sw/m7pl_hls_regs.h at 7784f75, section "roi" (XFPGA_M7_OBJECT_ROI_AXI_CONTROL_*);
# tests/test_kv_roi_pl.py re-parses that header from git and compares every entry.
ROI_REGS = {
    "AP_CTRL": 0x000, "GIE": 0x004, "IER": 0x008, "ISR": 0x00c, "AP_RETURN": 0x010,
    "DEPTH_DATA": 0x018, "WIDTH_DATA": 0x024, "HEIGHT_DATA": 0x02c, "BOX_COUNT_DATA": 0x034,
    "DEPTH_MARGIN_M_DATA": 0x03c, "MIN_PIXELS_DATA": 0x048,
    "MIN_VALID_FRACTION_DATA": 0x050, "FREE_FRACTION_DATA": 0x05c,
    "BOX_ENABLED_BASE": 0x080, "NEAR_Z_BASE": 0x100, "BOX_XYXY_BASE": 0x200,
    "FAR_Z_BASE": 0x400, "OBJECT_ID_BASE": 0x500, "RESULT_BASE": 0x800,
}
R = ROI_REGS


class RoiUnavailable(RuntimeError):
    """The PL ROI core is not usable in this process (gate failed); kv_server refuses."""


class RoiDeviceError(RuntimeError):
    """One call failed on the device (busy, timeout); the frame's ROI is void."""


def _f64_words(v):
    q = struct.unpack("<Q", struct.pack("<d", float(v)))[0]
    return q & 0xFFFFFFFF, q >> 32


def _words_f64(lo, hi):
    return struct.unpack("<d", struct.pack("<Q", (int(hi) << 32) | int(lo)))[0]


def open_roi_window(sysinfo, *, name=ROI_UIO, sysfs_root="/sys/class/uio", dev_root="/dev",
                    opener=os.open, window_cls=None):
    """The MAGIC-equivalent gate (module docstring). Returns (window, detail) or raises
    RoiUnavailable. Nothing is mapped unless the sysinfo checks pass first."""
    from host.kv_regs import UioWindow, _read_sysfs_int
    from host.m7_pl_device import find_uio
    window_cls = window_cls or UioWindow
    if sysinfo is None or not sysinfo.available:
        raise RoiUnavailable("sysinfo block unavailable (%s): cannot identify the bitstream"
                             % (getattr(sysinfo, "reason", "not opened")))
    info = sysinfo.sysinfo()
    detail = {"build_id": "%#010x" % info["build_id"], "slot_map": "%#x" % info["slot_map"]}
    if not info["slot_map"] >> ROI_SLOT & 1:
        raise RoiUnavailable("bitstream %s has no ROI slot (SLOT_MAP %#x, bit %d clear): the "
                             "ROI core exists only in m7_pl_merge (0x07784f75)"
                             % (detail["build_id"], info["slot_map"], ROI_SLOT))
    n = find_uio(name, sysfs_root)
    if n is None:
        raise RoiUnavailable("no UIO node named %r under %s although SLOT_MAP has slot %d"
                             % (name, sysfs_root, ROI_SLOT))
    mdir = os.path.join(sysfs_root, "uio%d" % n, "maps", "map0")
    try:
        addr = _read_sysfs_int(os.path.join(mdir, "addr"))
        size = _read_sysfs_int(os.path.join(mdir, "size"))
    except (OSError, ValueError) as exc:
        raise RoiUnavailable("uio%d map0 unreadable: %s" % (n, exc))
    detail.update(uio="uio%d" % n, map_addr="%#x" % addr, map_size="%#x" % size)
    if addr != ROI_BASE or size < ROI_MIN_MAP:
        raise RoiUnavailable("uio%d map0 %#x+%#x is not the ROI slot %#x (>= %#x)"
                             % (n, addr, size, ROI_BASE, ROI_MIN_MAP))
    try:
        win = window_cls(os.path.join(dev_root, "uio%d" % n), size, opener=opener)
    except (OSError, ValueError) as exc:
        raise RoiUnavailable("mmap %s/uio%d failed: %s" % (dev_root, n, exc))
    ctrl = win.rd(R["AP_CTRL"])
    detail["ap_ctrl_at_open"] = "%#x" % ctrl
    if not ctrl & AP_IDLE:
        win.close()
        raise RoiUnavailable("ROI core not idle at open (AP_CTRL %#x)" % ctrl)
    return win, detail


class PlRoiCore:
    """``(depth2d_f32, roi_dict) -> {status, rows, times_ns, cycles}`` like CpuRoiCore."""

    name = "pl"

    def __init__(self, window, buffers, *, timeout_s=0.5, detail=None, wait="poll", irq=None,
                 irq_slice_s=0.01):
        self.win, self.buffers = window, buffers
        self.timeout_s = float(timeout_s)
        self.detail = dict(detail or {})
        self.counts = Counter()
        if wait not in ("poll", "irq"):
            raise ValueError("wait must be 'poll' or 'irq', got %r" % (wait,))
        if wait == "irq" and irq is None:
            raise ValueError("wait='irq' needs the ROI interrupt line")
        self.wait, self.irq, self.irq_slice_s = wait, irq, float(irq_slice_s)
        self.lock = threading.Lock()
        self.buf = buffers.alloc(4 * MAX_PIXELS)       # once; reused every frame

    def describe(self):
        d = {"core": "pl", "detail": self.detail, "buffer_pa": "%#x" % self.buf.pa,
             "timeout_s": self.timeout_s, "counts": dict(self.counts)}
        if self.wait != "poll":                       # poll: the keys exactly as before
            d.update(wait=self.wait, irq_slice_s=self.irq_slice_s,
                     irq=self.irq.describe() if hasattr(self.irq, "describe") else None)
        return d

    def _wr(self, off, v):
        self.win.wr(off, int(v) & 0xFFFFFFFF)

    def _wr_f64(self, off, v):
        lo, hi = _f64_words(v)
        self.win.wr(off, lo)
        self.win.wr(off + 4, hi)

    def __call__(self, depth2d, roi):
        with self.lock:
            return self._call(depth2d, roi)

    def _call(self, depth2d, roi):
        t0 = time.monotonic_ns()
        rows_in = roi["rows"]
        n = len(rows_in)
        height, width = depth2d.shape
        if n > MAX_BOXES or width * height > MAX_PIXELS:
            raise RoiDeviceError("request exceeds core capacity (%d boxes, %dx%d)" % (n, width, height))
        pixels = width * height
        self.buf.array[:4 * pixels] = np.ascontiguousarray(depth2d, dtype="<f4").reshape(-1).view(np.uint8)
        self.buffers.to_device(self.buf)
        w = self.win
        if not w.rd(R["AP_CTRL"]) & AP_IDLE:
            self.counts["device_busy"] += 1
            raise RoiDeviceError("ROI core not idle")
        for b in range(MAX_BOXES):          # every row written; rows >= n are zero (as cpu)
            if b < n:
                row_id, en, x0, y0, x1, y1, nz, fz = rows_in[b]
            else:
                row_id, en, x0, y0, x1, y1, nz, fz = 0, 0, 0, 0, 0, 0, 0.0, 0.0
            self._wr(R["BOX_ENABLED_BASE"] + 4 * b, en)
            self._wr(R["OBJECT_ID_BASE"] + 4 * b, row_id)
            self._wr_f64(R["NEAR_Z_BASE"] + 8 * b, nz)
            self._wr_f64(R["FAR_Z_BASE"] + 8 * b, fz)
            for k, v in enumerate((x0, y0, x1, y1)):
                self._wr(R["BOX_XYXY_BASE"] + 16 * b + 4 * k, v)
        pa = self.buf.pa
        self._wr(R["DEPTH_DATA"], pa & 0xFFFFFFFF)
        self._wr(R["DEPTH_DATA"] + 4, pa >> 32)
        self._wr(R["WIDTH_DATA"], width)
        self._wr(R["HEIGHT_DATA"], height)
        self._wr(R["BOX_COUNT_DATA"], n)
        cfg = roi["cfg"]
        self._wr_f64(R["DEPTH_MARGIN_M_DATA"], cfg["depth_margin_m"])
        self._wr(R["MIN_PIXELS_DATA"], cfg["min_pixels"])
        self._wr_f64(R["MIN_VALID_FRACTION_DATA"], cfg["min_valid_fraction"])
        self._wr_f64(R["FREE_FRACTION_DATA"], cfg["free_fraction"])
        t1 = time.monotonic_ns()
        if self.wait == "irq":
            polls = self._start_wait_irq(t1)
        else:
            w.wr(R["AP_CTRL"], AP_START)
            deadline = t1 + int(self.timeout_s * 1e9)
            polls = 0
            while True:
                polls += 1
                if w.rd(R["AP_CTRL"]) & AP_DONE:
                    break
                if time.monotonic_ns() > deadline:
                    self.counts["device_timeout"] += 1
                    raise RoiDeviceError("ROI core did not finish within %.3f s" % self.timeout_s)
        t2 = time.monotonic_ns()
        status = w.rd(R["AP_RETURN"])
        rows = []
        if status == 0:
            base = R["RESULT_BASE"]
            rows = [tuple(w.rd(base + 4 * (FIELDS * b + f)) for f in range(FIELDS))
                    for b in range(n)]
        t3 = time.monotonic_ns()
        self.counts["calls"] += 1
        self.counts["status:%d" % status] += 1
        self.counts["polls"] += polls
        return {"status": int(status), "rows": rows,
                "times_ns": {"prep": t1 - t0, "device": t2 - t1, "readback": t3 - t2,
                             "total": t3 - t0},
                "cycles": None}

    def _clear_isr(self):
        self.win.wr(R["ISR"], self.win.rd(R["ISR"]))     # toggle-on-write

    def _start_wait_irq(self, t1):
        """ap_start and the interrupt wait (m7_backend._PlCore._run_irq order); returns the ap_done reads."""
        w, irq = self.win, self.irq
        self._clear_isr()                                 # a stale ISR would wake the line at once
        irq.arm()
        w.wr(R["GIE"], 1)
        w.wr(R["IER"], 1)
        w.wr(R["AP_CTRL"], AP_START)
        deadline = t1 + int(self.timeout_s * 1e9)
        reads = 0
        try:
            while True:
                left = (deadline - time.monotonic_ns()) / 1e9
                if left <= 0:
                    self.counts["device_timeout"] += 1
                    raise RoiDeviceError("ROI core did not finish within %.3f s (irq wait)" % self.timeout_s)
                fired = irq.wait(min(self.irq_slice_s, left)) is not None
                if fired:
                    self.counts["irq_wakeups"] += 1
                    # clear ISR before reading ap_done: a completion after the read sets ISR again and
                    # fires the re-armed line, so none is lost
                    self._clear_isr()
                reads += 1
                if w.rd(R["AP_CTRL"]) & AP_DONE:
                    if not fired:
                        self.counts["irq_rescued"] += 1   # done, but no event within the slice
                    return reads
                if fired:
                    self.counts["irq_spurious"] += 1
                    irq.arm()
        finally:
            w.wr(R["IER"], 0)
            w.wr(R["GIE"], 0)

    def close(self):
        try:
            self.buffers.release(self.buf)
        except Exception:
            pass
        if self.irq is not None and hasattr(self.irq, "close"):
            self.irq.close()
        if hasattr(self.win, "close"):
            self.win.close()


# ------------------------------------------------------------------ host mock
class MockRoiWindow:
    """Register window of a mock ROI core. ``memory(pa, nbytes, dtype)`` resolves the
    DEPTH address (MockPlDevice._memory); ``library`` has fpga_m7_object_roi_batch
    (host.m7_object_native.NativeObjectEvidence(...).lib). ``mutant``: None,
    "row_shift" or "threshold" (box ``mutant_box`` gets near_z + ``mutant_dz``)."""

    def __init__(self, memory, library, *, mutant=None, mutant_box=0, mutant_dz=0.25,
                 hang=False, delay_s=0.0, irq_lost=False):
        self.memory, self.lib = memory, library
        self.mutant, self.mutant_box, self.mutant_dz = mutant, int(mutant_box), float(mutant_dz)
        self.hang = hang
        self.words = {R["AP_CTRL"]: AP_IDLE}
        self.writes = Counter()
        self.starts = 0
        # SW_M1 interrupt model (as MockPlDevice): ISR bit 0 set on completion while IER bit 0; the line fires
        # while armed with GIE, IER and ISR set. delay_s: the core finishes in a thread after that long;
        # irq_lost: the line never fires (positive control of the lost-interrupt rescue).
        self.delay_s, self.irq_lost = float(delay_s), bool(irq_lost)
        self.line = None
        self._lock = threading.RLock()

    def irq_line(self):
        """The core's interrupt line (an os.pipe() like MockPlDevice.irq), created once."""
        if self.line is None:
            from host.m7_pl_device import _PipeIrq
            self.line = _PipeIrq(self, "roi")
        return self.line

    def _maybe_fire(self, block=None):
        with self._lock:
            w = self.words
            if (self.line is not None and not self.irq_lost and w.get(R["GIE"], 0) & 1
                    and w.get(R["IER"], 0) & 1 and w.get(R["ISR"], 0) & 1):
                self.line.fire()

    def _complete(self):
        with self._lock:
            self.words[R["AP_CTRL"]] = AP_IDLE | AP_DONE
            if self.words.get(R["IER"], 0) & 1:
                self.words[R["ISR"]] = self.words.get(R["ISR"], 0) | 1
        self._maybe_fire()

    def _finish_later(self):
        time.sleep(self.delay_s)
        with self._lock:
            self._execute()
        self._complete()

    def rd(self, off):
        with self._lock:
            v = int(self.words.get(off, 0))
            if off == R["AP_CTRL"]:
                self.words[off] = v & ~AP_DONE            # ap_done clear-on-read
            return v

    def wr(self, off, value):
        with self._lock:
            self.writes[off] += 1
            if off == R["AP_CTRL"]:
                if value & AP_START:
                    self.starts += 1
                    self.words[off] = 0
                    if self.hang:
                        return
                    if self.delay_s > 0:
                        threading.Thread(target=self._finish_later, daemon=True).start()
                        return
                    self._execute()
                else:
                    return
            elif R["RESULT_BASE"] <= off < R["RESULT_BASE"] + 4 * FIELDS * MAX_BOXES:
                return                                     # output RAM: host writes do nothing
            elif off == R["ISR"]:
                self.words[off] = self.words.get(off, 0) ^ (int(value) & 0x3)   # toggle-on-write
                return
            else:
                self.words[off] = int(value) & 0xFFFFFFFF
                if off not in (R["GIE"], R["IER"]):
                    return
        if off == R["AP_CTRL"]:
            self._complete()
        else:
            self._maybe_fire()

    def _u32(self, off):
        return int(self.words.get(off, 0))

    def _f64(self, off):
        return _words_f64(self._u32(off), self._u32(off + 4))

    def _execute(self):
        width, height = self._u32(R["WIDTH_DATA"]), self._u32(R["HEIGHT_DATA"])
        count = self._u32(R["BOX_COUNT_DATA"])
        pa = self._u32(R["DEPTH_DATA"]) | (self._u32(R["DEPTH_DATA"] + 4) << 32)
        pixels = width * height
        if 0 < pixels <= MAX_PIXELS:
            depth = np.ascontiguousarray(self.memory(pa, 4 * pixels, "<f4").astype(np.float32))
        else:
            depth = np.zeros(1, dtype=np.float32)
        en = np.array([self._u32(R["BOX_ENABLED_BASE"] + 4 * b) for b in range(MAX_BOXES)], np.uint32)
        ids = np.array([self._u32(R["OBJECT_ID_BASE"] + 4 * b) for b in range(MAX_BOXES)], np.uint32)
        xyxy = np.array([self._u32(R["BOX_XYXY_BASE"] + 4 * j) for j in range(4 * MAX_BOXES)], np.uint32)
        near = np.array([self._f64(R["NEAR_Z_BASE"] + 8 * b) for b in range(MAX_BOXES)], np.float64)
        far = np.array([self._f64(R["FAR_Z_BASE"] + 8 * b) for b in range(MAX_BOXES)], np.float64)
        if self.mutant == "threshold":
            near[self.mutant_box] += self.mutant_dz
            far[self.mutant_box] = max(far[self.mutant_box], near[self.mutant_box])
        result = np.full(FIELDS * MAX_BOXES, 0xFFFFFFFF, np.uint32)
        status = np.full(1, 0xFFFFFFFF, np.uint32)
        self.lib.fpga_m7_object_roi_batch(
            depth, width, height, count, en, xyxy, near, far, ids,
            self._f64(R["DEPTH_MARGIN_M_DATA"]), self._u32(R["MIN_PIXELS_DATA"]),
            self._f64(R["MIN_VALID_FRACTION_DATA"]), self._f64(R["FREE_FRACTION_DATA"]),
            result, status)
        self.words[R["AP_RETURN"]] = int(status[0])
        if status[0] != 0:
            return                                     # the table keeps the previous call
        rows = result.reshape(MAX_BOXES, FIELDS)[:count].copy()
        if self.mutant == "row_shift" and count > 1:
            rows = np.roll(rows, 1, axis=0)
        for b in range(count):
            for f in range(FIELDS):
                self.words[R["RESULT_BASE"] + 4 * (FIELDS * b + f)] = int(rows[b, f])

    def close(self):
        if self.line is not None:
            self.line.close()
            self.line = None
