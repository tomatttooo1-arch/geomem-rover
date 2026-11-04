"""Rolling map (cross-frame log-odds fusion) on the KV260 PL -- v3 L2, 2026-10-26.

The car fuses on the Nano CPU (host/m7_rolling.py RollingKernelHost: g++ build of
hw/m7/rolling_map.cpp). The PL core ``fpga_m7_rolling_axi`` (hw/m7/pl/hls/m7_rolling_axi.cpp)
is the SAME core behind an AXI shell, map resident in URAM. This module is the optional
path that runs it; the car default stays ``cpu`` (kv_server ``--rolling`` default none).

Data flow (fuse/RESULTS.md section 2): the frame core leaves frame k's three-state block in
its DDR output buffer, in the rolling core's obs layout ((z*ny+y)*nx+x, first nx*ny*nz bytes).
The Nano keeps every decision: it validates the frame (freshness, lattice, frame id;
RollingFusion.ingest) and only then sends ROLL INTEGRATE naming that block by its crc32;
the KV260 checks the block it still holds (frame id, nvox, crc32) and starts the rolling
core on that very buffer (zero copy: the PS writes registers only). A block the KV no
longer holds (a newer frame replaced it, or a failed frame) is answered ``not_held``; the
Nano then resends the same command with the block inline (counted ``roll:inline_resend``).
The KV never fuses on a frame request by itself (red line: a stale frame the Nano rejects
must not enter the map).

Wire (kv_link types ROLL_REQ 15 / ROLL_ACK 16, feature "rolling"; bodies sealed with crc32):
  ROLL_REQ  ver B, op B, mode B (0 held, 1 inline), rsv B, cmd_id Q, epoch I, frame_seq I,
            window_base 3i, obs_base 3i, nx I, ny I, nz I, query_count I, held_frame_id Q,
            obs_crc I, n_inline I; then n_inline obs bytes; then 3*query_count i32 (QUERY).
  ROLL_ACK  ver B, result B, rsv H, cmd_id Q, header 24I, n_delta I, n_qout I, n_dump I,
            prep/device/readback ns 3Q; then n_delta u64, n_qout u32, n_dump u64.
  op 0..3 = the core's RESET/INTEGRATE/QUERY/DUMP; op 15 CONFIG answers the core's 16
  config words in header[0:16]. Commands are idempotent by cmd_id: the KV keeps the last
  answer and returns it again for a resend (an INTEGRATE is never run twice).
  Go-live 2026-10-26 (feature "rolling_replay"): mode 2 REPLAY asks "did you run cmd_id?"
  -- the KV answers the cached ACK if cmd_id is its last command, else ``not_executed``,
  and never runs or caches anything for it. The Nano uses it after an INTEGRATE whose
  ACK was lost (RollingFusion link recovery, remote backend only).
  B1 2026-11-02 (feature "rolling_grid", ROLL version 2; NANO_SIDE.md "接口约定"): a v2 ROLL_REQ carries
  after its head the grid extension  flags B (1 grid+aux, 2 body seed on the core, 4 cost plane),
  band_lo B, band_hi B, tall_lo B, body_cx i32, body_cy i32 (window-local Q16 voxels of the window base
  of THIS command), body_r u32 (Q16); a v2 CONFIG carries the 14x14 Nav2 cost table as its 196 inline
  bytes (obs_crc = its crc32). A v2 ROLL_ACK (answer to a v2 request only) carries after its head
  grid_status I (costmap.hpp B1_* code; 0x100 not run), n_grid I (0 or mx*my), n_cost I, lut_crc I, and
  after the dump: grid int8[n_grid], aux u8[n_grid], cost u8[n_cost] -- the planes of the map AFTER this
  INTEGRATE, local order ly*mx+lx, same window base as header BASE. v1 bodies are unchanged; a v1 peer
  never sees v2 (the Nano sends v2 only when the HELLO announced "rolling_grid").

Python 3.8 compatible.
"""

from collections import Counter
import os
import struct
import threading
import time
import zlib

import numpy as np

from host.m7_rolling import (H, OP_DUMP, OP_INTEGRATE, OP_QUERY, OP_RESET, RollingConfig,
                             RollingKernelHost, _RollingBackend, _coords3)

ROLLING_BASE = 0xA0050000
ROLLING_SLOT = 5
ROLLING_UIO = "m7_rolling"
ROLLING_MIN_MAP = 0x100
SYS_SLOT_MAP = 0x20
AP_START, AP_DONE, AP_IDLE = 0x1, 0x2, 0x4
OBS_CAPACITY = 32 ** 3
HEADER_WORDS = 24
OP_CONFIG = 15
MODE_HELD, MODE_INLINE, MODE_REPLAY = 0, 1, 2
RESULT_OK, RESULT_NOT_HELD, RESULT_BAD_REQUEST, RESULT_DEVICE_ERROR, RESULT_NOT_EXECUTED = 0, 1, 2, 3, 4
RESULT_NAMES = {RESULT_OK: "ok", RESULT_NOT_HELD: "not_held", RESULT_BAD_REQUEST: "bad_request",
                RESULT_DEVICE_ERROR: "device_error", RESULT_NOT_EXECUTED: "not_executed"}
# Smallest per-call wait worth sending a ROLL for (frame budget nearly spent => not sent).
ROLL_MIN_WAIT_S = 0.005
ROLL_VER = 1
ROLL_VER_GRID = 2                                 # B1 (2026-11-02): v1 + the grid extension
ROLL_REQ_HEAD = struct.Struct("<BBBBQII3i3iIIIIQII")
ROLL_ACK_HEAD = struct.Struct("<BBHQ24IIII3Q")
ROLL_REQ_GRID = struct.Struct("<BBBBiiI")         # flags, band_lo, band_hi, tall_lo, body_cx, body_cy, body_r
ROLL_ACK_GRID = struct.Struct("<IIII")            # grid_status, n_grid, n_cost, lut_crc
GRID_NOT_RUN = 0x100
# INT_SW (2026-11-02): op GRID of the fused core (hw/m7/rolling_map.hpp, -DFPGA_M7_ROLL_GRID, B1h). Reuses the
# existing ports (B1/INTEGRATION.md 1.1): ONX/ONY/ONZ = band_lo/band_hi/tall_lo, OBS_BASE_X/Y = body centre
# (window-local Q16), OBS_BASE_Z = body radius (Q16), QUERY_COUNT = core flags (bit0 body, bit1 inflation),
# OBS_DATA -> the 196-byte cost table (the shell fetches 25 words), planes out through DELTA_DATA.
OP_GRID = 4
CORE_STATUS_UNINITIALIZED, CORE_STATUS_BAD_ARGS = 1, 2
GRID_CORE_F_BODY, GRID_CORE_F_INFL = 1, 2
GRID_LUT_WORDS = 25                               # ceil(196 / 8): what the shell reads from OBS_DATA
GRID_REASONS = (2, 3, 4, 5)                       # header CHANGED_CELLS on BAD_ARGS = costmap.hpp B1_* (5 flags)
CRC = struct.Struct("<I")
OPS = (OP_RESET, OP_INTEGRATE, OP_QUERY, OP_DUMP, OP_CONFIG)

# hw/m7/pl/sw/m7pl_hls_regs.h section "rolling" (XFPGA_M7_ROLLING_AXI_CONTROL_ADDR_*),
# generated from the exported HLS driver; tests/test_m7_rolling_pl.py re-parses the header.
ROLLING_REGS = {
    "AP_CTRL": 0x000, "GIE": 0x004, "IER": 0x008, "ISR": 0x00c, "AP_RETURN": 0x010,
    "OBS_DATA": 0x018, "QUERY_XYZ_DATA": 0x024, "QUERY_OUT_DATA": 0x030, "DELTA_DATA": 0x03c,
    "DUMP_DATA": 0x048, "HEADER_OUT_DATA": 0x054, "OP_DATA": 0x060, "EPOCH_DATA": 0x068,
    "FRAME_SEQ_DATA": 0x070, "WINDOW_BASE_X_DATA": 0x078, "WINDOW_BASE_Y_DATA": 0x080,
    "WINDOW_BASE_Z_DATA": 0x088, "OBS_BASE_X_DATA": 0x090, "OBS_BASE_Y_DATA": 0x098,
    "OBS_BASE_Z_DATA": 0x0a0, "ONX_DATA": 0x0a8, "ONY_DATA": 0x0b0, "ONZ_DATA": 0x0b8,
    "QUERY_COUNT_DATA": 0x0c0,
}
R = ROLLING_REGS


class RollingUnavailable(RuntimeError):
    """No rolling core: the bitstream lacks slot 5, no UIO node, or not idle."""


class RollingDeviceError(RuntimeError):
    pass


class RollBudgetExpired(RuntimeError):
    """The frame's freshness budget is (nearly) spent: the ROLL was NOT sent."""


def crc32(data):
    return zlib.crc32(bytes(data)) & 0xFFFFFFFF


def seal(body):
    return body + CRC.pack(zlib.crc32(body) & 0xFFFFFFFF)


def unseal(buf):
    buf = bytes(buf)
    if len(buf) < CRC.size + 1:
        raise ValueError("short ROLL body")
    body, (crc,) = buf[:-CRC.size], CRC.unpack_from(buf, len(buf) - CRC.size)
    if zlib.crc32(body) & 0xFFFFFFFF != crc:
        raise ValueError("ROLL body crc")
    return body


# ------------------------------------------------------------------ codec
def encode_roll_req(*, op, cmd_id, mode=MODE_HELD, epoch=0, frame_seq=0, window_base=(0, 0, 0),
                    obs_base=(0, 0, 0), dims=(0, 0, 0), query=None, held_frame_id=0,
                    obs_crc=0, inline=b"", query_count=None, grid=None):
    """``grid`` None: a v1 body (byte for byte as before B1). Else a v2 body with the grid extension
    (flags, band_lo, band_hi, tall_lo, body_cx, body_cy, body_r)."""
    q = np.zeros((0, 3), np.int32) if query is None else np.asarray(query, dtype=np.int32).reshape(-1, 3)
    qc = q.shape[0] if query_count is None else int(query_count)
    if qc <= 1024 and q.shape[0] != qc:
        raise ValueError("query_count disagrees with the coordinates")
    head = ROLL_REQ_HEAD.pack(ROLL_VER if grid is None else ROLL_VER_GRID, int(op), int(mode), 0, int(cmd_id),
                              int(epoch) & 0xFFFFFFFF,
                              int(frame_seq) & 0xFFFFFFFF, *(int(v) for v in window_base),
                              *(int(v) for v in obs_base), *(int(v) for v in dims), qc,
                              int(held_frame_id), int(obs_crc), len(inline))
    if grid is not None:
        head += ROLL_REQ_GRID.pack(*(int(v) for v in grid))
    return seal(head + bytes(inline) + q.astype("<i4").tobytes())


def decode_roll_req(buf):
    body = unseal(buf)
    if len(body) < ROLL_REQ_HEAD.size:
        raise ValueError("short ROLL_REQ")
    v = ROLL_REQ_HEAD.unpack_from(body)
    if v[0] not in (ROLL_VER, ROLL_VER_GRID):
        raise ValueError("ROLL_REQ version %d" % v[0])
    req = {"op": v[1], "mode": v[2], "cmd_id": v[4], "epoch": v[5], "frame_seq": v[6],
           "window_base": tuple(v[7:10]), "obs_base": tuple(v[10:13]), "dims": tuple(v[13:16]),
           "query_count": v[16], "held_frame_id": v[17], "obs_crc": v[18], "n_inline": v[19],
           "ver": v[0], "grid": None}
    off = ROLL_REQ_HEAD.size
    if v[0] == ROLL_VER_GRID:
        if len(body) < off + ROLL_REQ_GRID.size:
            raise ValueError("short ROLL_REQ grid extension")
        g = ROLL_REQ_GRID.unpack_from(body, off)
        req["grid"] = {"flags": g[0], "band_lo": g[1], "band_hi": g[2], "tall_lo": g[3],
                       "body_cx": g[4], "body_cy": g[5], "body_r": g[6]}
        off += ROLL_REQ_GRID.size
    n = req["n_inline"]
    if n > OBS_CAPACITY:
        raise ValueError("inline block too large")
    req["inline"] = body[off:off + n]
    off += n
    qc = req["query_count"]
    nq = 3 * qc if (req["op"] == OP_QUERY and qc <= 1024) else 0
    req["query"] = np.frombuffer(body, dtype="<i4", count=nq, offset=off).astype(np.int32)
    if len(req["inline"]) != n or off + 4 * nq != len(body):
        raise ValueError("ROLL_REQ length")
    return req


def encode_roll_ack(*, result, cmd_id, header=None, delta=None, qout=None, dump=None, times=(0, 0, 0),
                    ver=ROLL_VER, grid_status=GRID_NOT_RUN, grid=None, aux=None, cost=None, lut_crc=0):
    """``ver`` ROLL_VER: byte for byte the pre-B1 body. ROLL_VER_GRID (answer to a v2 request): the grid
    extension follows the head and the planes follow the dump (grid and aux together or neither)."""
    header = np.zeros(HEADER_WORDS, np.uint32) if header is None else np.asarray(header, np.uint32)
    delta = np.zeros(0, np.uint64) if delta is None else np.asarray(delta, dtype=np.uint64)
    qout = np.zeros(0, np.uint32) if qout is None else np.asarray(qout, dtype=np.uint32)
    dump = np.zeros(0, np.uint64) if dump is None else np.asarray(dump, dtype=np.uint64)
    head = ROLL_ACK_HEAD.pack(int(ver), int(result), 0, int(cmd_id), *(int(h) for h in header),
                              delta.size, qout.size, dump.size, *(int(t) for t in times))
    tail = b""
    if int(ver) == ROLL_VER_GRID:
        g = np.zeros(0, np.int8) if grid is None else np.asarray(grid, dtype=np.int8).ravel()
        a = np.zeros(0, np.uint8) if aux is None else np.asarray(aux, dtype=np.uint8).ravel()
        c = np.zeros(0, np.uint8) if cost is None else np.asarray(cost, dtype=np.uint8).ravel()
        if g.size != a.size:
            raise ValueError("grid and aux planes differ in size")
        head += ROLL_ACK_GRID.pack(int(grid_status), g.size, c.size, int(lut_crc) & 0xFFFFFFFF)
        tail = g.tobytes() + a.tobytes() + c.tobytes()
    return seal(head + delta.astype("<u8").tobytes() + qout.astype("<u4").tobytes() +
                dump.astype("<u8").tobytes() + tail)


def decode_roll_ack(buf):
    body = unseal(buf)
    v = ROLL_ACK_HEAD.unpack_from(body)
    if v[0] not in (ROLL_VER, ROLL_VER_GRID):
        raise ValueError("ROLL_ACK version %d" % v[0])
    nd, nq, ndump = v[28], v[29], v[30]
    off = ROLL_ACK_HEAD.size
    gx = None
    if v[0] == ROLL_VER_GRID:
        if len(body) < off + ROLL_ACK_GRID.size:
            raise ValueError("short ROLL_ACK grid extension")
        gx = ROLL_ACK_GRID.unpack_from(body, off)
        off += ROLL_ACK_GRID.size
    ngrid, ncost = (gx[1], gx[2]) if gx else (0, 0)
    if off + 8 * nd + 4 * nq + 8 * ndump + 2 * ngrid + ncost != len(body):
        raise ValueError("ROLL_ACK length")
    delta = np.frombuffer(body, dtype="<u8", count=nd, offset=off).astype(np.uint64)
    off += 8 * nd
    qout = np.frombuffer(body, dtype="<u4", count=nq, offset=off).astype(np.uint32)
    off += 4 * nq
    dump = np.frombuffer(body, dtype="<u8", count=ndump, offset=off).astype(np.uint64)
    out = {"result": v[1], "cmd_id": v[3], "header": np.array(v[4:28], dtype=np.uint32),
           "delta": delta, "qout": qout, "dump": dump,
           "times_ns": {"prep": v[31], "device": v[32], "readback": v[33]}, "ver": v[0], "grid": None}
    if gx is not None:
        off += 8 * ndump
        out["grid"] = {"status": gx[0], "lut_crc": gx[3],
                       "grid": np.frombuffer(body, dtype=np.int8, count=ngrid, offset=off).copy(),
                       "aux": np.frombuffer(body, dtype=np.uint8, count=ngrid, offset=off + ngrid).copy(),
                       "cost": (np.frombuffer(body, dtype=np.uint8, count=ncost, offset=off + 2 * ngrid).copy()
                                if ncost else None)}
    return out


# ------------------------------------------------------------------ B1 grid C model (A53 / host stand-in)
def compile_grid_cmodel(out_so):
    """g++ build of hw/b1_costmap/costmap.cpp (the PL grid core's C model, read only here) -> its function.
    The A53 / host stand-in for the PL grid op until the fused core is on the board."""
    import ctypes
    import subprocess
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    subprocess.run(["g++", "-O2", "-std=c++11", "-shared", "-fPIC", "-Wno-unknown-pragmas", "-Wno-unused-label",
                    "-ffp-contract=off", os.path.join(root, "hw/b1_costmap/costmap.cpp"), "-o", out_so], check=True)
    u8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
    i8 = np.ctypeslib.ndpointer(dtype=np.int8, flags="C_CONTIGUOUS")
    u64 = np.ctypeslib.ndpointer(dtype=np.uint64, flags="C_CONTIGUOUS")
    f = ctypes.CDLL(out_so).fpga_b1_costmap
    f.argtypes = [u64] + [ctypes.c_uint32] * 6 + [ctypes.c_uint32, ctypes.c_int32, ctypes.c_int32,
                                                  ctypes.c_uint32, ctypes.c_uint32, u8, i8, u8, u8]
    f.restype = ctypes.c_uint32
    return f


def run_grid_cmodel(fn, dump_words, offsets, spec, lut, cells):
    """One grid op on a DUMP (physical words + ring offsets) -> (status, grid, aux, cost or None)."""
    from host.pl_grid_layer import GRID_F_BODY, GRID_F_COST, LUT_SIZE
    flags = int(spec["flags"])
    infl = 1 if (flags & GRID_F_COST and lut is not None) else 0
    table = np.zeros(LUT_SIZE, np.uint8) if lut is None else np.ascontiguousarray(lut, dtype=np.uint8)
    grid, aux, cost = np.zeros(cells, np.int8), np.zeros(cells, np.uint8), np.zeros(cells, np.uint8)
    st = fn(np.ascontiguousarray(dump_words, dtype=np.uint64), int(offsets[0]), int(offsets[1]), int(offsets[2]),
            int(spec["band_lo"]), int(spec["band_hi"]), int(spec["tall_lo"]), 1 if flags & GRID_F_BODY else 0,
            int(spec["body_cx"]), int(spec["body_cy"]), int(spec["body_r"]), infl, table, grid, aux, cost)
    return int(st), grid, aux, (cost if infl else None)


# ------------------------------------------------------------------ cores (raw call)
class CpuRollingCore:
    """The g++ core on the KV A53 (kv_server --rolling cpu; host tests). ``grid_library`` (B1): a
    compile_grid_cmodel output path -- the core then answers the grid op (C model on a DUMP after the
    INTEGRATE) and the server announces "rolling_grid"; None = exactly as before."""

    name = "cpu"
    zero_copy = False

    def __init__(self, library, grid_library=None):
        self.k = RollingKernelHost(library)
        self.config = self.k.config
        self.counts = Counter()
        self.grid_fn = None if grid_library is None else compile_grid_cmodel(grid_library)
        self.grid_capable = self.grid_fn is not None
        self.last_grid = None

    def config_words(self):
        out = np.zeros(16, dtype=np.uint32)
        self.k.lib.fpga_m7_rolling_config(out)
        return out

    def call(self, op, epoch, frame_seq, wb, ob, dims, *, obs=None, obs_pa=None, query=None, qc=0,
             grid=None, lut=None):
        """``grid`` (B1, the decoded v2 extension) on an INTEGRATE: also the planes of the map after it, left
        in ``last_grid`` (status, grid, aux, cost) -- None when not asked or the INTEGRATE was refused."""
        self.last_grid = None
        if obs_pa is not None:
            raise RollingDeviceError("cpu core has no device buffer")
        t0 = time.monotonic_ns()
        buf = None
        if obs is not None:
            buf = np.zeros(OBS_CAPACITY, np.uint8)
            buf[:len(obs)] = np.frombuffer(bytes(obs), np.uint8)
        qbuf = None
        if query is not None and len(query):
            qbuf = np.zeros(3 * self.config.query_capacity, np.int32)
            qbuf[:len(query)] = query
        t1 = time.monotonic_ns()
        hdr = self.k._call(op, epoch=epoch, frame_seq=frame_seq, window_base=np.asarray(wb, np.int32),
                           obs_base=np.asarray(ob, np.int32), obs=buf, dims=dims, query=qbuf,
                           query_count=qc)
        t2 = time.monotonic_ns()
        ok = int(hdr[0]) == 0
        nd = int(hdr[H["N_DELTA"]]) if ok and op == OP_INTEGRATE else 0
        out = (hdr.copy(), self.k._delta[:nd].copy(),
               self.k._query_out[:qc].copy() if ok and op == OP_QUERY else None,
               self.k._dump.copy() if ok and op == OP_DUMP else None)
        if grid is not None and self.grid_fn is not None and ok and op == OP_INTEGRATE:
            dh = self.k._call(OP_DUMP)          # read only: the core's state is not changed by a DUMP
            offs = [int(dh[H["OFFSET"] + a]) for a in range(3)]
            mx, my, _ = self.config.dims
            self.last_grid = run_grid_cmodel(self.grid_fn, self.k._dump, offs, grid, lut, mx * my)
            self.counts["grid"] += 1
        t3 = time.monotonic_ns()
        self.counts["calls"] += 1
        return out + ((t1 - t0, t3 - t1, time.monotonic_ns() - t3),)


def open_rolling_window(sysinfo, *, name=ROLLING_UIO, sysfs_root="/sys/class/uio", dev_root="/dev",
                        opener=os.open, window_cls=None):
    """kv_roi_pl.open_roi_window's gate for slot 5: sysinfo up, SLOT_MAP bit 5, the UIO
    node with map0 at 0xA005_0000, AP_CTRL idle. Nothing is mapped before sysinfo passes."""
    from host.kv_regs import UioWindow, _read_sysfs_int
    from host.m7_pl_device import find_uio
    window_cls = window_cls or UioWindow
    if sysinfo is None or not sysinfo.available:
        raise RollingUnavailable("sysinfo block unavailable (%s): cannot identify the bitstream"
                                 % (getattr(sysinfo, "reason", "not opened")))
    info = sysinfo.sysinfo()
    detail = {"build_id": "%#010x" % info["build_id"], "slot_map": "%#x" % info["slot_map"]}
    if not info["slot_map"] >> ROLLING_SLOT & 1:
        raise RollingUnavailable("bitstream %s has no rolling slot (SLOT_MAP %#x, bit %d clear): "
                                 "the rolling core exists only in m7_pl_v3f"
                                 % (detail["build_id"], info["slot_map"], ROLLING_SLOT))
    n = find_uio(name, sysfs_root)
    if n is None:
        raise RollingUnavailable("no UIO node named %r under %s although SLOT_MAP has slot %d"
                                 % (name, sysfs_root, ROLLING_SLOT))
    mdir = os.path.join(sysfs_root, "uio%d" % n, "maps", "map0")
    try:
        addr = _read_sysfs_int(os.path.join(mdir, "addr"))
        size = _read_sysfs_int(os.path.join(mdir, "size"))
    except (OSError, ValueError) as exc:
        raise RollingUnavailable("uio%d map0 unreadable: %s" % (n, exc))
    detail.update(uio="uio%d" % n, map_addr="%#x" % addr, map_size="%#x" % size)
    if addr != ROLLING_BASE or size < ROLLING_MIN_MAP:
        raise RollingUnavailable("uio%d map0 %#x+%#x is not the rolling slot %#x"
                                 % (n, addr, size, ROLLING_BASE))
    try:
        win = window_cls(os.path.join(dev_root, "uio%d" % n), size, opener=opener)
    except (OSError, ValueError) as exc:
        raise RollingUnavailable("mmap %s/uio%d failed: %s" % (dev_root, n, exc))
    ctrl = win.rd(R["AP_CTRL"])
    detail["ap_ctrl_at_open"] = "%#x" % ctrl
    if not ctrl & AP_IDLE:
        win.close()
        raise RollingUnavailable("rolling core not idle at open (AP_CTRL %#x)" % ctrl)
    return win, detail


class PlRollingCore:
    """The PL core through its register window; DMA buffers from ``buffers`` (the geometry
    device: alloc/release/to_device/from_device, PlBuffer .pa/.array). Polls ap_done."""

    name = "pl"
    zero_copy = True
    # INT_SW (2026-11-02): the fused core's op GRID (B1h, INTEGRATION.md 1.2 alignment 1): after an accepted
    # INTEGRATE that asked for the grid, the same ``call`` starts op GRID at once (synchronous, nothing else
    # runs in between, so the planes are those of the map after this INTEGRATE) and leaves ``last_grid`` =
    # (status, grid int8[cells], aux u8[cells], cost u8[cells] or None) as CpuRollingCore does. ``grid``:
    #   "off"   (default) exactly as before: no probe, no table buffer, grid_capable False.
    #   "probe" one GRID at construction (read-only op): OK / UNINITIALIZED -> the bitstream has it,
    #           grid_capable True; BAD_ARGS (op 4 of a current bitstream) or any error -> grid_capable False
    #           and every later call is exactly as with "off".
    grid_capable = False

    def __init__(self, window, buffers, config_words, *, timeout_s=0.2, detail=None, grid="off"):
        self.win, self.buffers = window, buffers
        self._config_words = np.asarray(config_words, dtype=np.uint32).copy()
        self.config = RollingConfig.from_words(self._config_words)
        c = self.config
        self.timeout_s = float(timeout_s)
        self.detail = dict(detail or {})
        self.counts = Counter()
        self.lock = threading.Lock()
        self.b = {name: buffers.alloc(n) for name, n in (
            ("obs", OBS_CAPACITY), ("query", 12 * c.query_capacity), ("qout", 4 * c.query_capacity),
            ("delta", 8 * c.delta_capacity), ("dump", 8 * c.words), ("header", 4 * HEADER_WORDS))}
        self.last_grid = None
        self.grid_probe = None
        self._lut_key = None
        if grid not in ("off", "probe"):
            raise ValueError("grid must be 'off' or 'probe', got %r" % (grid,))
        if grid == "probe":
            self.b["lut"] = buffers.alloc(8 * GRID_LUT_WORDS)
            self.set_lut(None)
            self.grid_probe = self._probe_grid()
            self.grid_capable = self.grid_probe["capable"]

    # ---------------------------------------------------------------- INT_SW: op GRID
    def set_lut(self, lut):
        """The Nav2 cost table into its DDR buffer (CONFIG v2 time; INTEGRATION.md alignment 2'): op GRID then
        points OBS_DATA at it, the INTEGRATE keeps reading the frame core's output in place. None = zeros
        (the core ignores the table while inflation is off)."""
        if "lut" not in self.b:
            return
        raw = b"" if lut is None else np.ascontiguousarray(lut, dtype=np.uint8).tobytes()
        if raw == self._lut_key:
            return
        arr = self.b["lut"].array
        arr[:8 * GRID_LUT_WORDS] = 0
        arr[:len(raw)] = np.frombuffer(raw, np.uint8)
        self.buffers.to_device(self.b["lut"])
        self._lut_key = raw
        self.counts["grid_lut_written"] += 1

    def _probe_grid(self):
        """One read-only GRID with valid arguments (band 0..0, no tall layer, no body, no inflation)."""
        try:
            with self.lock:
                hdr, _, _ = self._grid_run((0, 0, 1), (0, 0, 0), 0)
        except Exception as exc:
            self.counts["grid_probe_error"] += 1
            return {"capable": False, "why": "error:%s" % type(exc).__name__}
        st, why = int(hdr[0]), int(hdr[H["CHANGED_CELLS"]])
        capable = st in (0, CORE_STATUS_UNINITIALIZED)
        self.counts["grid_probe:%s" % ("capable" if capable else "absent")] += 1
        return {"capable": capable, "status": st, "reason": why}

    def _grid_run(self, band, body, core_flags, keep=None):
        """Start op GRID, wait, read the header; returns (header, device_ns, polls). ``keep``: (epoch,
        frame_seq, wb) to leave in those registers (the core ignores them for GRID)."""
        b, w = self.b, self.win
        hv = b["header"].array[:4 * HEADER_WORDS].view("<u4")
        hv[:] = 0xDEADBEEF
        self.buffers.to_device(b["header"])
        if not w.rd(R["AP_CTRL"]) & AP_IDLE:
            self.counts["device_busy"] += 1
            raise RollingDeviceError("rolling core not idle")
        epoch, frame_seq, wb = keep if keep is not None else (0, 0, (0, 0, 0))
        self._wr64("OBS_DATA", b["lut"].pa)
        self._wr64("QUERY_XYZ_DATA", b["query"].pa)
        self._wr64("QUERY_OUT_DATA", b["qout"].pa)
        self._wr64("DELTA_DATA", b["delta"].pa)
        self._wr64("DUMP_DATA", b["dump"].pa)
        self._wr64("HEADER_OUT_DATA", b["header"].pa)
        for name, v in (("OP_DATA", OP_GRID), ("EPOCH_DATA", epoch), ("FRAME_SEQ_DATA", frame_seq),
                        ("WINDOW_BASE_X_DATA", wb[0]), ("WINDOW_BASE_Y_DATA", wb[1]),
                        ("WINDOW_BASE_Z_DATA", wb[2]), ("OBS_BASE_X_DATA", body[0]),
                        ("OBS_BASE_Y_DATA", body[1]), ("OBS_BASE_Z_DATA", body[2]),
                        ("ONX_DATA", band[0]), ("ONY_DATA", band[1]), ("ONZ_DATA", band[2]),
                        ("QUERY_COUNT_DATA", core_flags)):
            self._wr(name, v)
        t1 = time.monotonic_ns()
        w.wr(R["AP_CTRL"], AP_START)
        deadline = t1 + int(self.timeout_s * 1e9)
        polls = 0
        while True:
            polls += 1
            if w.rd(R["AP_CTRL"]) & AP_DONE:
                break
            if time.monotonic_ns() > deadline:
                self.counts["device_timeout"] += 1
                raise RollingDeviceError("rolling core (GRID) did not finish within %.3f s" % self.timeout_s)
        dev = time.monotonic_ns() - t1
        status = w.rd(R["AP_RETURN"])
        self.buffers.from_device(b["header"])
        hdr = hv.astype(np.uint32).copy()
        if int(hdr[0]) != status:
            self.counts["header_status_mismatch"] += 1
            raise RollingDeviceError("GRID: AP_RETURN %d but header status %d" % (status, int(hdr[0])))
        return hdr, dev, polls

    def _grid_after(self, spec, lut, epoch, frame_seq, wb, ihdr):
        """op GRID right after an accepted INTEGRATE -> (last_grid, device_ns, readback_ns). Never raises:
        the INTEGRATE's answer stands whatever happens here (a failure is counted and reads as not run)."""
        from host.pl_grid_layer import GRID_F_BODY, GRID_F_COST
        if lut is not None:
            self.set_lut(lut)
        flags = int(spec["flags"])
        infl = bool(flags & GRID_F_COST and lut is not None)
        core_flags = (GRID_CORE_F_BODY if flags & GRID_F_BODY else 0) | (GRID_CORE_F_INFL if infl else 0)
        band = (int(spec["band_lo"]), int(spec["band_hi"]), int(spec["tall_lo"]))
        body = (int(spec["body_cx"]), int(spec["body_cy"]), int(spec["body_r"]))
        try:
            hdr, dev, polls = self._grid_run(band, body, core_flags, keep=(epoch, frame_seq, wb))
        except Exception as exc:
            self.counts["grid_error:%s" % type(exc).__name__] += 1
            return None, 0, 0
        self.counts["polls"] += polls
        self.counts["grid_calls"] += 1
        t0 = time.monotonic_ns()
        st = int(hdr[0])
        if st == CORE_STATUS_BAD_ARGS:
            why = int(hdr[H["CHANGED_CELLS"]])
            self.counts["grid_reason:%d" % why] += 1
            return ((why, None, None, None) if why in GRID_REASONS else None), dev, time.monotonic_ns() - t0
        if st != 0:
            self.counts["grid_core_status:%d" % st] += 1
            return None, dev, time.monotonic_ns() - t0
        if any(int(hdr[H["BASE"] + a]) != int(ihdr[H["BASE"] + a]) for a in range(3)):
            self.counts["grid_base_mismatch"] += 1
            return None, dev, time.monotonic_ns() - t0
        mx, my, _ = self.config.dims
        cells = mx * my
        pw = cells // 8
        nd = int(hdr[H["N_DELTA"]])
        if nd != (3 if infl else 2) * pw:
            self.counts["grid_bad_n_delta"] += 1
            return None, dev, time.monotonic_ns() - t0
        self.buffers.from_device(self.b["delta"])
        raw = self.b["delta"].array[:8 * nd].copy()       # little-endian lanes: byte i = cell i of the plane
        grid = raw[:cells].view(np.int8).copy()
        aux = raw[cells:2 * cells].copy()
        cost = raw[2 * cells:3 * cells].copy() if infl else None
        self.counts["grid_ok"] += 1
        return (0, grid, aux, cost), dev, time.monotonic_ns() - t0

    def config_words(self):
        return self._config_words.copy()

    def describe(self):
        d = {"core": "pl", "detail": self.detail, "timeout_s": self.timeout_s,
             "buffers": {k: "%#x" % v.pa for k, v in self.b.items()}, "counts": dict(self.counts)}
        if self.grid_probe is not None:         # grid "off": keys exactly as before
            d["grid_probe"] = dict(self.grid_probe)
        return d

    def _wr(self, name, v):
        self.win.wr(R[name], int(v) & 0xFFFFFFFF)

    def _wr64(self, name, pa):
        self.win.wr(R[name], pa & 0xFFFFFFFF)
        self.win.wr(R[name] + 4, (pa >> 32) & 0xFFFFFFFF)

    def call(self, op, epoch, frame_seq, wb, ob, dims, *, obs=None, obs_pa=None, query=None, qc=0,
             grid=None, lut=None):
        """``grid`` / ``lut`` (B1 / INT_SW): as CpuRollingCore.call; only used when ``grid_capable``."""
        with self.lock:
            self.last_grid = None
            out = self._call(op, epoch, frame_seq, wb, ob, dims, obs, obs_pa, query, qc)
            if grid is None or not self.grid_capable or op != OP_INTEGRATE or int(out[0][0]) != 0:
                return out
            g, dev, rb = self._grid_after(grid, lut, epoch, frame_seq, wb, out[0])
            self.last_grid = g
            t = out[4]
            return out[:4] + ((t[0], t[1] + dev, t[2] + rb),)

    def _call(self, op, epoch, frame_seq, wb, ob, dims, obs, obs_pa, query, qc):
        t0 = time.monotonic_ns()
        b = self.b
        if obs_pa is None:
            n = 0 if obs is None else len(obs)
            if n:
                b["obs"].array[:n] = np.frombuffer(bytes(obs), np.uint8)
                self.buffers.to_device(b["obs"])
            obs_pa = b["obs"].pa
            self.counts["obs_copied"] += 1
        else:
            self.counts["obs_zero_copy"] += 1
        if query is not None and len(query):
            b["query"].array[:4 * len(query)] = np.asarray(query, dtype="<i4").view(np.uint8)
            self.buffers.to_device(b["query"])
        hv = b["header"].array[:4 * HEADER_WORDS].view("<u4")
        hv[:] = 0xDEADBEEF                       # an unwritten header word cannot pass
        self.buffers.to_device(b["header"])
        w = self.win
        if not w.rd(R["AP_CTRL"]) & AP_IDLE:
            self.counts["device_busy"] += 1
            raise RollingDeviceError("rolling core not idle")
        self._wr64("OBS_DATA", obs_pa)
        self._wr64("QUERY_XYZ_DATA", b["query"].pa)
        self._wr64("QUERY_OUT_DATA", b["qout"].pa)
        self._wr64("DELTA_DATA", b["delta"].pa)
        self._wr64("DUMP_DATA", b["dump"].pa)
        self._wr64("HEADER_OUT_DATA", b["header"].pa)
        for name, v in (("OP_DATA", op), ("EPOCH_DATA", epoch), ("FRAME_SEQ_DATA", frame_seq),
                        ("WINDOW_BASE_X_DATA", wb[0]), ("WINDOW_BASE_Y_DATA", wb[1]),
                        ("WINDOW_BASE_Z_DATA", wb[2]), ("OBS_BASE_X_DATA", ob[0]),
                        ("OBS_BASE_Y_DATA", ob[1]), ("OBS_BASE_Z_DATA", ob[2]),
                        ("ONX_DATA", dims[0]), ("ONY_DATA", dims[1]), ("ONZ_DATA", dims[2]),
                        ("QUERY_COUNT_DATA", qc)):
            self._wr(name, v)
        t1 = time.monotonic_ns()
        w.wr(R["AP_CTRL"], AP_START)
        deadline = t1 + int(self.timeout_s * 1e9)
        polls = 0
        while True:
            polls += 1
            if w.rd(R["AP_CTRL"]) & AP_DONE:
                break
            if time.monotonic_ns() > deadline:
                self.counts["device_timeout"] += 1
                raise RollingDeviceError("rolling core did not finish within %.3f s" % self.timeout_s)
        t2 = time.monotonic_ns()
        status = w.rd(R["AP_RETURN"])
        self.buffers.from_device(b["header"])
        hdr = hv.astype(np.uint32).copy()
        if int(hdr[0]) != status:
            self.counts["header_status_mismatch"] += 1
            raise RollingDeviceError("AP_RETURN %d but header status %d" % (status, int(hdr[0])))
        ok = status == 0
        nd = int(hdr[H["N_DELTA"]]) if ok and op == OP_INTEGRATE else 0
        if nd > self.config.delta_capacity:
            raise RollingDeviceError("n_delta %d exceeds capacity" % nd)
        delta = qout = dump = None
        if nd:
            self.buffers.from_device(b["delta"])
        delta = b["delta"].array[:8 * nd].view("<u8").astype(np.uint64)
        if ok and op == OP_QUERY:
            self.buffers.from_device(b["qout"])
            qout = b["qout"].array[:4 * qc].view("<u4").astype(np.uint32)
        if ok and op == OP_DUMP:
            self.buffers.from_device(b["dump"])
            dump = b["dump"].array[:8 * self.config.words].view("<u8").astype(np.uint64)
        t3 = time.monotonic_ns()
        self.counts["calls"] += 1
        self.counts["polls"] += polls
        return hdr, delta, qout, dump, (t1 - t0, t2 - t1, t3 - t2)

    def close(self):
        for buf in self.b.values():
            try:
                self.buffers.release(buf)
            except Exception:
                pass
        if hasattr(self.win, "close"):
            self.win.close()


class MockRollingWindow:
    """Host stand-in for the PL core's register window: on ap_start it does what the AXI
    shell does (fetch ceil(n/8) obs words / 3*qc query words from 'DDR', run the core on
    local copies, write header[24], n_delta delta words, query_count query words, the dump),
    with the g++ core (a fresh compile_rolling library). ``memory(pa, nbytes, dtype)`` is
    the geometry mock device's ``_memory``. ``mutant``: "drop_last_delta" (the shell writes
    one delta word fewer) -- a shell fault the comparators must catch."""

    def __init__(self, memory, library, *, mutant=None, grid=False):
        """``grid`` (INT_SW 2026-11-02): ``library`` is a compile_rolling(grid=True) build and the shell also
        does what m7_rolling_axi.cpp's two FPGA_M7_ROLL_GRID blocks do (op GRID: fetch 25 table words from
        OBS_DATA, write N_DELTA plane words through DELTA_DATA). False: exactly as before (op 4 is
        BAD_ARGS from the core), i.e. a current bitstream."""
        self.grid = bool(grid)
        self.memory = memory
        self.k = RollingKernelHost(library)
        self.regs = {R["AP_CTRL"]: AP_IDLE}
        self.mutant = mutant
        self.lobs = np.zeros(OBS_CAPACITY, np.uint8)   # the shell's local copy persists
        self.starts = 0

    def rd(self, off):
        v = self.regs.get(off, 0)
        if off == R["AP_CTRL"]:
            self.regs[off] = v & ~AP_DONE
        return v

    def wr(self, off, value):
        if off == R["AP_CTRL"] and value & AP_START:
            self.starts += 1
            self._execute()
            self.regs[off] = AP_IDLE | AP_DONE
            return
        self.regs[off] = int(value) & 0xFFFFFFFF

    def _u64(self, name):
        return self.regs.get(R[name], 0) | (self.regs.get(R[name] + 4, 0) << 32)

    def _i32(self, name):
        return int(np.uint32(self.regs.get(R[name], 0)).view(np.int32))

    def _execute(self):
        g = lambda n: self.regs.get(R[n], 0)  # noqa: E731
        op, qc = g("OP_DATA"), g("QUERY_COUNT_DATA")
        dims = (g("ONX_DATA"), g("ONY_DATA"), g("ONZ_DATA"))
        if op == OP_INTEGRATE and all(1 <= d <= 32 for d in dims):
            nw = (dims[0] * dims[1] * dims[2] + 7) // 8
            self.lobs[:8 * nw] = self.memory(self._u64("OBS_DATA"), 8 * nw, np.uint8)
        if self.grid and op == OP_GRID:
            self.lobs[:8 * GRID_LUT_WORDS] = self.memory(self._u64("OBS_DATA"), 8 * GRID_LUT_WORDS, np.uint8)
        lq = np.zeros(3 * self.k.config.query_capacity, np.int32)
        if op == OP_QUERY and qc <= self.k.config.query_capacity and qc:
            lq[:3 * qc] = self.memory(self._u64("QUERY_XYZ_DATA"), 12 * qc, "<i4")
        wb = [self._i32(n) for n in ("WINDOW_BASE_X_DATA", "WINDOW_BASE_Y_DATA", "WINDOW_BASE_Z_DATA")]
        ob = [self._i32(n) for n in ("OBS_BASE_X_DATA", "OBS_BASE_Y_DATA", "OBS_BASE_Z_DATA")]
        hdr = self.k._call(op, epoch=g("EPOCH_DATA"), frame_seq=g("FRAME_SEQ_DATA"),
                           window_base=np.asarray(wb, np.int32), obs_base=np.asarray(ob, np.int32),
                           obs=self.lobs, dims=dims, query=lq, query_count=qc)
        ok = int(hdr[0]) == 0
        if ok and op == OP_DUMP:
            self.memory(self._u64("DUMP_DATA"), 8 * self.k.config.words, "<u8")[:] = self.k._dump
        nd = int(hdr[H["N_DELTA"]]) if ok and (op == OP_INTEGRATE or (self.grid and op == OP_GRID)) else 0
        if self.mutant == "drop_last_delta" and nd:
            nd -= 1
        if nd:
            self.memory(self._u64("DELTA_DATA"), 8 * nd, "<u8")[:] = self.k._delta[:nd]
        if ok and op == OP_QUERY and qc:
            self.memory(self._u64("QUERY_OUT_DATA"), 4 * qc, "<u4")[:] = self.k._query_out[:qc]
        self.memory(self._u64("HEADER_OUT_DATA"), 4 * HEADER_WORDS, "<u4")[:] = hdr
        self.regs[R["AP_RETURN"]] = int(hdr[0])

    def close(self):
        pass


# ------------------------------------------------------------------ KV side service
class RollingService:
    """KV260 end of ROLL: the held block, idempotent commands, one core. ``hold`` is called
    by the server's worker right after a frame (same thread as ``handle``)."""

    def __init__(self, core, *, frame_buffer=None):
        """``frame_buffer``: zero-argument callable -> the frame core's flags PlBuffer (PL
        frame core) or None; only then (and only with a zero-copy core) is the held block
        read in place."""
        self.core = core
        self.frame_buffer = frame_buffer
        self.counts = Counter()
        self._held = None
        self._last_cmd, self._last_body = None, None
        self.config_words = core.config_words()
        # B1 (2026-11-02): the core answers the grid op -> HELLO "rolling_grid"; the cost table from CONFIG v2
        self.grid_capable = bool(getattr(core, "grid_capable", False))
        self.lut, self.lut_crc = None, 0
        # CTRLPS3 (2026-11-03): the newest OK grid plane, for the KV navigation core (kv_server --nav, host/kv_nav.py
        # RollingGridSource): (time.monotonic() when made, window_base (x, y, z) of that INTEGRATE, grid int8 [ly*mx+lx],
        # aux uint8 [ly*mx+lx] -- CTRLPS4: strict tri-state + tall bit, for the KV-side swept clearing).
        # Only a reference is kept; nothing else changes (no extra op, the ACK bytes are the same).
        self.nav_planes = None

    def describe(self):
        d = {"core": self.core.name, "zero_copy": bool(self.core.zero_copy and self.frame_buffer),
             "counts": dict(self.counts)}
        if hasattr(self.core, "describe"):
            d["core_detail"] = self.core.describe()
        return d

    def hold(self, frame_id, status, flags, nvox):
        if status != 0:
            self._held = None
            self.counts["held_cleared"] += 1
            return
        block = np.ascontiguousarray(flags[:nvox], dtype=np.uint8)
        pa = None
        if self.core.zero_copy and self.frame_buffer is not None:
            buf = self.frame_buffer()
            pa = None if buf is None else buf.pa
        self._held = {"frame_id": int(frame_id), "n": int(nvox), "crc": crc32(block.tobytes()),
                      "pa": pa, "copy": None if pa is not None else block.tobytes()}
        self.counts["held"] += 1

    def handle(self, payload):
        try:
            req = decode_roll_req(payload)
        except (ValueError, struct.error) as exc:
            self.counts["bad_request:%s" % type(exc).__name__] += 1
            return None
        if req["cmd_id"] == self._last_cmd and self._last_body is not None:
            self.counts["replay_hit" if req["mode"] == MODE_REPLAY else "duplicate_cmd"] += 1
            return self._last_body
        if req["mode"] == MODE_REPLAY:
            # Probe only: never run, never cached (the cached last answer stays).
            self.counts["replay_not_executed"] += 1
            return encode_roll_ack(result=RESULT_NOT_EXECUTED, cmd_id=req["cmd_id"])
        body = self._execute(req)
        self._last_cmd, self._last_body = req["cmd_id"], body
        return body

    def _execute(self, req):
        op, cmd = req["op"], req["cmd_id"]
        ver = req.get("ver", ROLL_VER)
        self.counts["op:%d" % op] += 1
        if op == OP_CONFIG:
            hdr = np.zeros(HEADER_WORDS, np.uint32)
            hdr[:16] = self.config_words
            if ver == ROLL_VER_GRID:
                self._take_lut(req)
                return encode_roll_ack(result=RESULT_OK, cmd_id=cmd, header=hdr, ver=ver, lut_crc=self.lut_crc)
            return encode_roll_ack(result=RESULT_OK, cmd_id=cmd, header=hdr)
        kw = {}
        gspec = req.get("grid")
        if ver == ROLL_VER_GRID and op == OP_INTEGRATE and gspec and gspec["flags"] & 1 and self.grid_capable:
            kw["grid"], kw["lut"] = gspec, self.lut
        if op == OP_INTEGRATE:
            n = req["dims"][0] * req["dims"][1] * req["dims"][2]
            if req["mode"] == MODE_INLINE:
                if len(req["inline"]) != n or crc32(req["inline"]) != req["obs_crc"]:
                    self.counts["inline_bad"] += 1
                    return encode_roll_ack(result=RESULT_BAD_REQUEST, cmd_id=cmd)
                kw["obs"] = req["inline"]
                self.counts["integrate_inline"] += 1
            else:
                h = self._held
                if h is None or h["n"] != n or h["crc"] != req["obs_crc"]:
                    self.counts["not_held"] += 1
                    return encode_roll_ack(result=RESULT_NOT_HELD, cmd_id=cmd)
                if h["pa"] is not None:
                    kw["obs_pa"] = h["pa"]
                    self.counts["integrate_zero_copy"] += 1
                else:
                    kw["obs"] = h["copy"]
                    self.counts["integrate_held_copy"] += 1
        elif op == OP_QUERY:
            kw["query"], kw["qc"] = req["query"], req["query_count"]
        elif op not in OPS:
            kw = {}
        try:
            hdr, delta, qout, dump, times = self.core.call(
                op, req["epoch"], req["frame_seq"], req["window_base"], req["obs_base"],
                req["dims"], **kw)
        except Exception as exc:
            self.counts["device_error:%s" % type(exc).__name__] += 1
            return encode_roll_ack(result=RESULT_DEVICE_ERROR, cmd_id=cmd, ver=ver)
        self.counts["status:%d" % int(hdr[0])] += 1
        if ver != ROLL_VER_GRID:
            return encode_roll_ack(result=RESULT_OK, cmd_id=cmd, header=hdr, delta=delta, qout=qout,
                                   dump=dump, times=times)
        g = getattr(self.core, "last_grid", None) if "grid" in kw else None
        if g is None:
            return encode_roll_ack(result=RESULT_OK, cmd_id=cmd, header=hdr, delta=delta, qout=qout,
                                   dump=dump, times=times, ver=ver, lut_crc=self.lut_crc)
        status, grid, aux, cost = g
        self.counts["grid_status:%d" % status] += 1
        ok = status == 0
        if ok and grid is not None:
            self.nav_planes = (time.monotonic(), tuple(int(v) for v in req["window_base"]), grid, aux)
        return encode_roll_ack(result=RESULT_OK, cmd_id=cmd, header=hdr, delta=delta, qout=qout, dump=dump,
                               times=times, ver=ver, grid_status=status, grid=grid if ok else None,
                               aux=aux if ok else None, cost=cost if ok else None, lut_crc=self.lut_crc)

    def _take_lut(self, req):
        """CONFIG v2: the 14x14 cost table (196 inline bytes, obs_crc its crc32); a bad one is not kept."""
        from host.pl_grid_layer import LUT_SIZE, lut_ok
        raw = req["inline"]
        if len(raw) == 0:
            return
        if len(raw) != LUT_SIZE or crc32(raw) != req["obs_crc"] or not lut_ok(np.frombuffer(raw, np.uint8)):
            self.counts["lut_bad"] += 1
            return
        self.lut = np.frombuffer(bytes(raw), np.uint8).copy()
        self.lut_crc = crc32(raw)
        self.counts["lut_set"] += 1
        if hasattr(self.core, "set_lut"):   # INT_SW: the PL core keeps the table in its own DDR buffer
            self.core.set_lut(self.lut)


# ------------------------------------------------------------------ Nano side backend
class RemotePlRollingBackend(_RollingBackend):
    """RollingFusion backend on the Nano: every call is one ROLL round trip to the KV260
    (rolling_backend("remote_pl")). Same interface and results as RollingKernelHost.

    Go-live (2026-10-26): ``link_recovery`` tells RollingFusion to survive link faults
    (``link_errors``) instead of raising; ``set_deadline_ns`` caps every ROLL wait at the
    frame's remaining freshness budget (monotonic ns); an INTEGRATE whose answer never came
    is kept in ``unanswered`` (cmd_id) for a REPLAY ``probe`` when the server announces
    "rolling_replay"."""

    link_recovery = True

    def __init__(self, server, *, timeout_s=0.3, resend_s=0.05, hello_timeout_s=2.0,
                 chunk_bytes=None, sock=None, session=None, link=None, grid_lut="default"):
        """``grid_lut`` (B1): the Nav2 cost table sent with CONFIG v2 when the server announces "rolling_grid"
        ("default" = pl_grid_layer.default_lut(), None = none sent)."""
        from host.kv_link import CHUNK_FRAGMENTED, KvLinkClient, LinkTimeout
        self.link = link if link is not None else KvLinkClient(
            server, chunk_bytes=chunk_bytes or CHUNK_FRAGMENTED, session=session, sock=sock,
            rcvbuf=4 << 20)
        self.counts = self.link.counts
        self.timeout_s, self.resend_s = float(timeout_s), float(resend_s)
        self._cmd = int.from_bytes(os.urandom(4), "little") << 20
        self.last_times = None
        self.kv_core = None
        self.link_errors = (LinkTimeout, RollBudgetExpired, RollingDeviceError, OSError,
                            ValueError, struct.error, RuntimeError)
        self._deadline_ns = None
        self.unanswered = None
        # B1 (2026-11-02): RollingFusion.enable_grid sets grid_band (a HeightBand) / skip_changes
        self.grid_capable = False
        self.grid_band = None
        self.grid_voxel_m = 0.05
        self.skip_changes = False
        self.grid_lut_crc = 0
        try:
            info = self.link.require("rolling", hello_timeout_s)
            feats = (info or {}).get("features") or {}
            self.replay_ok = isinstance(feats, dict) and feats.get("rolling_replay") == 1
            self.grid_capable = isinstance(feats, dict) and feats.get("rolling_grid") == 1
            if self.grid_capable:
                from host.pl_grid_layer import default_lut, lut_problem
                lut = default_lut() if isinstance(grid_lut, str) and grid_lut == "default" else grid_lut
                why = None if lut is None else lut_problem(lut)
                if why is not None:     # INT_SW: a table the PL would refuse (reason 4) is not sent
                    self.counts["grid_lut_rejected:" + why] += 1
                    import sys
                    print("WARNING: rolling grid cost table refused on the Nano (%s; PL reason 4): no cost "
                          "plane this session" % why, file=sys.stderr, flush=True)
                    lut = None
                raw = b"" if lut is None else np.ascontiguousarray(lut, dtype=np.uint8).tobytes()
                ack = self._ok(self._roll(OP_CONFIG, inline=raw, obs_crc=crc32(raw), grid=(0, 0, 0, 0, 0, 0, 0)))
                self.grid_lut_crc = (ack.get("grid") or {}).get("lut_crc", 0)
                if raw and self.grid_lut_crc != crc32(raw):
                    self.counts["grid_lut_not_taken"] += 1
            else:
                ack = self._ok(self._roll(OP_CONFIG))
        except Exception:
            self.link.close()
            raise
        self.config = RollingConfig.from_words(ack["header"][:16])
        self.kv_core = self._ask_core()

    def _ask_core(self, timeout_s=0.5):
        """10-29 console FPGA panel (display only): the core the KV260 fuses on, "pl" / "cpu" (one STATS at
        connect: the rolling service's describe(); the HELLO does not carry it). None when not answered --
        never raises, nothing else depends on it."""
        try:
            st = self.link.stats(timeout_s=timeout_s)
        except Exception:
            self.counts["roll_core_unknown"] += 1
            return None
        r = st.get("rolling") if isinstance(st, dict) else None
        core = r.get("core") if isinstance(r, dict) else None
        if not isinstance(core, str):
            self.counts["roll_core_unknown"] += 1
            return None
        return core

    def close(self):
        self.link.close()

    def set_deadline_ns(self, deadline_ns):
        """Monotonic ns by which every following ROLL must be answered (None = timeout_s only)."""
        self._deadline_ns = None if deadline_ns is None else int(deadline_ns)

    def _wait_s(self):
        wait = self.timeout_s
        if self._deadline_ns is not None:
            wait = min(wait, (self._deadline_ns - time.monotonic_ns()) / 1e9)
            if wait < ROLL_MIN_WAIT_S:
                self.counts["roll_budget_expired"] += 1
                raise RollBudgetExpired("frame budget spent; ROLL not sent")
        return wait

    def _roll(self, op, cmd_id=None, **kw):
        from host.kv_link import LinkTimeout, ROLL_ACK, ROLL_REQ
        wait = self._wait_s()
        if cmd_id is None:
            self._cmd += 1
            cmd_id = self._cmd
        body = encode_roll_req(op=op, cmd_id=cmd_id, **kw)
        t0 = time.monotonic_ns()
        # the same body (same cmd_id) is resent with doubling gaps within ``wait``
        payload = self.link._control_resend(ROLL_REQ, ROLL_ACK, body, wait, self.resend_s)
        self.last_rtt_ns = time.monotonic_ns() - t0
        if payload is None:
            self.counts["roll_timeout"] += 1
            if op == OP_INTEGRATE and kw.get("mode", MODE_HELD) != MODE_REPLAY:
                self.unanswered = cmd_id
            raise LinkTimeout("no ROLL answer within %.3f s" % wait)
        ack = decode_roll_ack(payload)
        if ack["cmd_id"] != cmd_id:
            self.counts["roll_foreign_ack"] += 1
            raise RollingDeviceError("ROLL answer for cmd %d, sent %d" % (ack["cmd_id"], cmd_id))
        self.counts["roll:%s" % RESULT_NAMES.get(ack["result"], ack["result"])] += 1
        self.last_times = ack["times_ns"]
        return ack

    def clear_unanswered(self):
        self.unanswered = None

    def probe(self, cmd_id):
        """REPLAY probe for an unanswered INTEGRATE: the cached ACK (it ran) or
        ``not_executed``; LinkTimeout if the KV does not answer."""
        self.counts["roll_probe"] += 1
        return self._roll(OP_INTEGRATE, cmd_id=cmd_id, mode=MODE_REPLAY)

    def _ok(self, ack):
        if ack["result"] != RESULT_OK:
            raise RollingDeviceError("ROLL answered %s" % RESULT_NAMES.get(ack["result"], ack["result"]))
        return ack

    def raw(self, op, *, epoch=0, frame_seq=0, window_base=(0, 0, 0), obs_base=(0, 0, 0),
            block_bytes=None, dims=(0, 0, 0), query=None, query_count=None):
        """Unchecked call, block always inline (trace replay); returns the decoded ACK."""
        kw = dict(epoch=epoch, frame_seq=frame_seq, window_base=window_base, obs_base=obs_base,
                  dims=dims)
        if block_bytes is not None:
            kw.update(mode=MODE_INLINE, inline=bytes(block_bytes), obs_crc=crc32(block_bytes))
        if query is not None:
            kw["query"] = query
        if query_count is not None:
            kw["query_count"] = query_count
        return self._ok(self._roll(op, **kw))

    def reset(self, epoch, window_base):
        ack = self._ok(self._roll(OP_RESET, epoch=int(epoch),
                                  window_base=_coords3(window_base, "window_base")))
        return self._result(ack["header"])

    def _grid_ext(self, window_base):
        """B1: the v2 grid extension for an INTEGRATE at ``window_base``, or None (grid off / band outside)."""
        if not self.grid_capable or self.grid_band is None:
            return None
        from host.pl_grid_layer import GRID_F_GRID, band_params
        try:
            lo, hi, tall_lo = band_params(self.grid_band, window_base[2], self.config.dims[2], self.grid_voxel_m)
        except ValueError:
            self.counts["grid_band_outside"] += 1
            return None
        return (GRID_F_GRID, lo, hi, tall_lo, 0, 0, 0)

    def _planes(self, ack):
        """B1: the ACK's planes -> PlGridPlanes (status 0, well formed) or None (counted)."""
        g = ack.get("grid")
        if g is None:
            self.counts["grid_missing"] += 1
            return None
        if g["status"] != 0:
            self.counts["grid_status:%d" % g["status"]] += 1
            return None
        from host.pl_grid_layer import PlGridPlanes, plane_problems
        planes = PlGridPlanes(0, g["grid"], g["aux"], g["cost"], body=False)
        bad = plane_problems(planes, self.config.dims[0] * self.config.dims[1])
        if bad:
            for why in bad:
                self.counts["grid_bad:" + why] += 1
            return None
        self.counts["grid_ok"] += 1
        return planes

    def integrate(self, epoch, frame_seq, window_base, obs_base, obs):
        buffer, nx, ny, nz = self._obs_buffer(obs)
        raw = buffer[:nx * ny * nz].tobytes()
        wb = _coords3(window_base, "window_base")
        kw = dict(epoch=int(epoch), frame_seq=int(frame_seq),
                  window_base=wb,
                  obs_base=_coords3(obs_base, "obs_base"), dims=(nx, ny, nz), obs_crc=crc32(raw))
        gx = self._grid_ext(wb)
        if gx is not None:
            kw["grid"] = gx
        ack = self._roll(OP_INTEGRATE, mode=MODE_HELD, **kw)
        if ack["result"] == RESULT_NOT_HELD:
            self.counts["roll:inline_resend"] += 1
            ack = self._roll(OP_INTEGRATE, mode=MODE_INLINE, inline=raw, **kw)
        self._ok(ack)
        header = ack["header"]
        if int(header[H["N_DELTA"]]) != ack["delta"].size and int(header[0]) == 0:
            raise RollingDeviceError("delta count disagrees with header")
        planes = self._planes(ack) if (gx is not None and int(header[0]) == 0) else None
        if self.skip_changes:
            # B1 "on": nobody consumes the per-cell change list (no bridge) -- the Python per-lane decode is
            # skipped; the header's CHANGED_CELLS still counts the changes
            self.counts["changes_not_decoded"] += 1
            return self._result(header, (), grid=planes)
        return self._result(header, RollingKernelHost.decode_delta(self, header, ack["delta"]), grid=planes)

    def query(self, coords):
        buffer, count = self._query_buffer(coords)
        ack = self._ok(self._roll(OP_QUERY, query=buffer[:3 * count].reshape(-1, 3)))
        result = self._result(ack["header"])
        q = ack["qout"] if result.ok else np.zeros(count, np.uint32)
        return result, self.decode_query(q.copy())

    def snapshot(self):
        ack = self._ok(self._roll(OP_DUMP))
        result = self._result(ack["header"])
        if not result.ok:
            return result, None
        mx, my, mz = self.config.dims
        words = ack["dump"]
        lanes = np.stack([(words >> np.uint64(8 * j)) & np.uint64(0xFF) for j in range(8)],
                         axis=1).astype(np.uint8).view(np.int8)
        off = result.offsets
        local = np.roll(lanes.reshape(mz, my, mx), shift=(-off[2], -off[1], -off[0]), axis=(0, 1, 2))
        return result, np.ascontiguousarray(local)


def remote_rolling_factory(server, **kw):
    """For M7BackendSelector(remote_rolling_factory=...): connect + CONFIG, or raise."""
    from host.kv_link import LinkError
    from host.m7_backend import PlBackendUnavailable

    def make():
        try:
            return RemotePlRollingBackend(server, **kw)
        except (LinkError, OSError, RollingDeviceError) as exc:
            raise PlBackendUnavailable("backend='remote_pl' for rolling: %s" % exc) from exc
    return make


class CoreRollingBackend(_RollingBackend):
    """rolling_backend("pl") on the KV260 itself (board self-test, replays): the
    RollingKernelHost interface over a raw core (PlRollingCore; CpuRollingCore in tests)."""

    def __init__(self, core):
        self.core = core
        self.config = core.config
        self.last_times = None

    def _do(self, op, *a, **kw):
        hdr, delta, qout, dump, times = self.core.call(op, *a, **kw)
        self.last_times = times
        return hdr, delta, qout, dump

    def reset(self, epoch, window_base):
        hdr, _, _, _ = self._do(OP_RESET, int(epoch), 0, _coords3(window_base, "window_base"),
                                (0, 0, 0), (0, 0, 0))
        return self._result(hdr)

    def integrate(self, epoch, frame_seq, window_base, obs_base, obs):
        buffer, nx, ny, nz = self._obs_buffer(obs)
        hdr, delta, _, _ = self._do(OP_INTEGRATE, int(epoch), int(frame_seq),
                                    _coords3(window_base, "window_base"), _coords3(obs_base, "obs_base"),
                                    (nx, ny, nz), obs=buffer[:nx * ny * nz].tobytes())
        return self._result(hdr, RollingKernelHost.decode_delta(self, hdr, delta))

    def query(self, coords):
        buffer, count = self._query_buffer(coords)
        hdr, _, qout, _ = self._do(OP_QUERY, 0, 0, (0, 0, 0), (0, 0, 0), (0, 0, 0),
                                   query=buffer[:3 * count], qc=count)
        result = self._result(hdr)
        return result, self.decode_query((qout if qout is not None else np.zeros(count, np.uint32)).copy())

    def snapshot(self):
        hdr, _, _, dump = self._do(OP_DUMP, 0, 0, (0, 0, 0), (0, 0, 0), (0, 0, 0))
        result = self._result(hdr)
        if not result.ok:
            return result, None
        mx, my, mz = self.config.dims
        lanes = np.stack([(dump >> np.uint64(8 * j)) & np.uint64(0xFF) for j in range(8)],
                         axis=1).astype(np.uint8).view(np.int8)
        off = result.offsets
        local = np.roll(lanes.reshape(mz, my, mx), shift=(-off[2], -off[1], -off[0]), axis=(0, 1, 2))
        return result, np.ascontiguousarray(local)


def kv_config_words(build_dir):
    """The core's 16 config words from a fresh g++ build (the PL shell has no config
    register; header MAGIC/DIMS are checked on every answer, so a mismatch cannot pass)."""
    from host.m7_rolling import compile_rolling
    return CpuRollingCore(compile_rolling(os.path.join(build_dir, "roll_config.so"))).config_words()
