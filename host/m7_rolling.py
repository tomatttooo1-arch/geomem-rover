"""CPU-callable M7 cross-frame rolling log-odds map and its host contract.

Two C++ backends share one Python API:

* ``RollingKernelHost``: the HLS top ``hw/m7/rolling_map.cpp`` compiled for
  the host. Its map is a ``static`` array (on-chip URAM in hardware), so every
  instance must load a separately compiled library file.
* ``RollingReference``: the plain reference ``hw/m7/rolling_map_ref.cpp``
  (one byte per cell, window move copies data, no ring offsets).

Neither runs a KV260 or connects AXI/DMA. ``RollingFusion`` is the host-side
consumer of ``M7Pipeline`` navigation outputs: it re-validates each output
at consumption time, maps context/generation changes to map epochs, checks
lattice alignment and never feeds rejected frames to the map. It does NOT
make a timed-out, dropped or invalid frame look like free space: those frames
are simply not integrated, and the reason is counted.

``RollingDeltaMirror`` is a reference Nav2-side consumer of the compact
deltas; it rejects stale epochs, duplicate or out-of-order deltas and
expired deltas, and requires a snapshot resync after a gap.

Window size, log-odds constants and time limits are a VERIFICATION
CONFIGURATION (see docs/design/M7_ROLLING_FUSION_INTERFACE_20260926.md), not
product decisions.
"""

from collections import Counter
from dataclasses import dataclass
import ctypes
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Optional

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
BACKEND = "cpu_reference"
MUTANTS = ("UNKNOWN_CLEARS", "NO_FREE_CLEAR", "DECAY", "SHIFT_SIGN",
           "NO_EVICT", "IGNORE_EPOCH")
COMPILE_FLAGS = ["-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                 # HLS pragmas and loop labels are meaningful to Vitis only.
                 "-Wno-unknown-pragmas", "-Wno-unused-label",
                 "-ffp-contract=off", "-fPIC", "-shared"]
UNKNOWN, FREE, OCCUPIED = 0, 1, 2
OP_RESET, OP_INTEGRATE, OP_QUERY, OP_DUMP = 0, 1, 2, 3
STATUS_NAMES = {0: "ok", 1: "uninitialized", 2: "bad_args",
                3: "malformed_observation", 4: "stale_epoch", 5: "stale_frame"}
H = dict(STATUS=0, OP=1, EPOCH=2, DELTA_SEQ=3, LAST_FRAME=4, BASE=5, OFFSET=8,
         N_DELTA=11, CHANGED_CELLS=12, OBS_OUTSIDE=13, OBS_INSIDE=14, SHIFT=15,
         ACCEPTED=16, REJ_EPOCH=17, REJ_FRAME=18, REJ_MALFORMED=19, MAGIC=20,
         DIMS=21, HAS_FRAME=22, OVERFLOW=23)
MAGIC = 0x4D375246
COORD_LIMIT = 1 << 28

I32 = np.ctypeslib.ndpointer(dtype=np.int32, flags="C_CONTIGUOUS")
U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
I8 = np.ctypeslib.ndpointer(dtype=np.int8, flags="C_CONTIGUOUS")
U64 = np.ctypeslib.ndpointer(dtype=np.uint64, flags="C_CONTIGUOUS")


def compile_rolling(output, *, source="kernel", mutant=None, dims=None, grid=False):
    """Build a fresh host library from the kernel or the reference source.

    ``mutant`` names one test-only fault variant (see rolling_map.hpp);
    ``dims`` optionally overrides (MX, MY, MZ) for a test build.
    ``grid`` (INT_SW 2026-11-02): build with -DFPGA_M7_ROLL_GRID, i.e. the next bitstream's fused core
    with op GRID (B1h); False = the command line exactly as before.
    """
    compiler = shutil.which("g++")
    if compiler is None:
        raise RuntimeError("g++ is required for the M7 rolling host reference")
    files = {"kernel": "hw/m7/rolling_map.cpp",
             "reference": "hw/m7/rolling_map_ref.cpp"}
    if source not in files:
        raise ValueError("source must be 'kernel' or 'reference'")
    if mutant is not None and mutant not in MUTANTS:
        raise ValueError("unknown mutant")
    output = Path(output).resolve()
    if not output.parent.is_dir() or output.exists():
        raise ValueError("choose a fresh output path within an existing directory")
    defines = ["-DFPGA_M7_ROLL_GRID"] if grid else []
    if mutant is not None:
        defines.append("-DFPGA_M7_ROLL_MUTANT_" + mutant)
    if dims is not None:
        mx, my, mz = (int(v) for v in dims)
        defines += ["-DFPGA_M7_ROLL_MX=%d" % mx, "-DFPGA_M7_ROLL_MY=%d" % my,
                    "-DFPGA_M7_ROLL_MZ=%d" % mz]
    subprocess.run([compiler] + COMPILE_FLAGS + defines +
                   [str(ROOT / files[source]), "-o", str(output)],
                   check=True, capture_output=True, text=True)
    return output


@dataclass(frozen=True)
class RollingConfig:
    dims: tuple  # (MX, MY, MZ) voxels
    lanes: int
    words: int
    obs_axis: int
    delta_capacity: int
    query_capacity: int
    header_words: int
    hit: int
    miss: int
    lmax: int
    lmin: int
    t_occ: int
    t_free: int
    mutant_mask: int

    @classmethod
    def from_words(cls, out):
        signed = [int(np.array(v, dtype=np.uint32).view(np.int32)) for v in out]
        return cls((int(out[0]), int(out[1]), int(out[2])), int(out[3]), int(out[4]),
                   int(out[5]), int(out[6]), int(out[7]), int(out[8]), signed[9],
                   signed[10], signed[11], signed[12], signed[13], signed[14],
                   int(out[15]))

    @property
    def cells(self):
        return self.dims[0] * self.dims[1] * self.dims[2]

    def state_of(self, logodds):
        """Tri-state readout of an int8 log-odds array (same rule as C++)."""
        values = np.asarray(logodds, dtype=np.int32)
        return np.where(values >= self.t_occ, OCCUPIED,
                        np.where(values <= self.t_free, FREE, UNKNOWN)).astype(np.uint8)


@dataclass(frozen=True)
class CallResult:
    status: int
    status_name: str
    header: np.ndarray
    epoch: int
    delta_seq: int
    last_frame: int
    window_base: tuple
    offsets: tuple
    changes: tuple  # sorted ((gx, gy, gz, new_state), ...)
    changed_cells: int
    obs_outside: int
    obs_inside: int
    shift_mask: int
    accepted_total: int
    rejected_epoch_total: int
    rejected_frame_total: int
    rejected_malformed_total: int
    initialized: bool
    has_frame: bool
    delta_overflow: int
    # B1 (2026-11-02): the PL navigation grid of this INTEGRATE (host.pl_grid_layer.PlGridPlanes) when the
    # backend was asked for it (RollingFusion.enable_grid) and the KV answered one; None otherwise.
    grid: Optional[object] = None

    @property
    def ok(self):
        return self.status == 0


def _signed(value):
    return int(np.array(value, dtype=np.uint32).view(np.int32))


def _coords3(values, name):
    out = tuple(int(v) for v in values)
    if len(out) != 3 or any(not -COORD_LIMIT < v < COORD_LIMIT for v in out):
        raise ValueError("%s must be three integers within +-2^28" % name)
    return out


class _RollingBackend:
    """Shared marshalling. Subclasses provide the four raw operations."""

    config: RollingConfig

    def _result(self, header, changes=(), grid=None):
        header = np.array(header, dtype=np.uint32, copy=True)
        mx, my, mz = self.config.dims
        if (int(header[H["MAGIC"]]) != MAGIC or
                int(header[H["DIMS"]]) != (mx | (my << 8) | (mz << 16))):
            raise RuntimeError("rolling map header magic/dimensions mismatch")
        status = int(header[H["STATUS"]])
        if status not in STATUS_NAMES:
            raise RuntimeError("rolling map returned an unknown status")
        return CallResult(
            status, STATUS_NAMES[status], header, int(header[H["EPOCH"]]),
            int(header[H["DELTA_SEQ"]]), int(header[H["LAST_FRAME"]]),
            tuple(_signed(header[H["BASE"] + a]) for a in range(3)),
            tuple(int(header[H["OFFSET"] + a]) for a in range(3)),
            tuple(sorted(changes)), int(header[H["CHANGED_CELLS"]]),
            int(header[H["OBS_OUTSIDE"]]), int(header[H["OBS_INSIDE"]]),
            int(header[H["SHIFT"]]), int(header[H["ACCEPTED"]]),
            int(header[H["REJ_EPOCH"]]), int(header[H["REJ_FRAME"]]),
            int(header[H["REJ_MALFORMED"]]), bool(header[H["HAS_FRAME"]] & 2),
            bool(header[H["HAS_FRAME"]] & 1), int(header[H["OVERFLOW"]]), grid)

    def _obs_buffer(self, obs):
        flags = np.asarray(obs)
        if flags.ndim != 3 or flags.dtype != np.uint8:
            raise ValueError("observation block must be uint8 [z, y, x]")
        nz, ny, nx = flags.shape
        cap = self.config.obs_axis
        if not (1 <= nx <= cap and 1 <= ny <= cap and 1 <= nz <= cap):
            raise ValueError("observation block axes must be in [1, 32]")
        buffer = np.zeros(cap ** 3, dtype=np.uint8)
        buffer[:flags.size] = flags.ravel()
        return buffer, nx, ny, nz

    def _query_buffer(self, coords):
        coords = np.asarray(coords, dtype=np.int64)
        if coords.ndim != 2 or coords.shape[1] != 3:
            raise ValueError("query coordinates must be an (N, 3) integer array")
        if coords.shape[0] > self.config.query_capacity:
            raise ValueError("query exceeds capacity")
        if np.any(np.abs(coords) >= 2 ** 31):
            raise ValueError("query coordinate overflows int32")
        buffer = np.zeros(3 * self.config.query_capacity, dtype=np.int32)
        buffer[:coords.size] = coords.ravel().astype(np.int32)
        return buffer, coords.shape[0]

    @staticmethod
    def decode_query(raw):
        raw = np.asarray(raw, dtype=np.uint32)
        states = (raw & 3).astype(np.uint8)
        logodds = ((raw >> 8) & 0xFF).astype(np.uint8).view(np.int8)
        outside = ((raw >> 16) & 1).astype(bool)
        return states, logodds, outside


class RollingKernelHost(_RollingBackend):
    """HLS top compiled for the host; one resident map per library file."""

    _loaded = set()

    def __init__(self, library):
        path = str(Path(library).resolve())
        if path in RollingKernelHost._loaded:
            # dlopen() returns the same handle, i.e. the SAME static map.
            raise ValueError("each RollingKernelHost needs a freshly compiled library")
        RollingKernelHost._loaded.add(path)
        self.lib = ctypes.CDLL(path)
        self.lib.fpga_m7_rolling_config.argtypes = [U32]
        self.lib.fpga_m7_rolling_config.restype = None
        out = np.zeros(16, dtype=np.uint32)
        self.lib.fpga_m7_rolling_config(out)
        self.config = RollingConfig.from_words(out)
        c = self.config
        self.lib.fpga_m7_rolling_map.argtypes = [
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, I32, I32,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, U8,
            ctypes.c_uint32, I32, U32, U64, U64, U32]
        self.lib.fpga_m7_rolling_map.restype = None
        self._obs = np.zeros(c.obs_axis ** 3, dtype=np.uint8)
        self._query = np.zeros(3 * c.query_capacity, dtype=np.int32)
        self._query_out = np.zeros(c.query_capacity, dtype=np.uint32)
        self._delta = np.zeros(c.delta_capacity, dtype=np.uint64)
        self._dump = np.zeros(c.words, dtype=np.uint64)

    def _call(self, op, *, epoch=0, frame_seq=0, window_base=(0, 0, 0),
              obs_base=(0, 0, 0), obs=None, dims=(0, 0, 0), query=None,
              query_count=0):
        header = np.full(self.config.header_words, 0xDEADBEEF, dtype=np.uint32)
        self._delta.fill(0)
        self.lib.fpga_m7_rolling_map(
            op, epoch, frame_seq, np.asarray(window_base, dtype=np.int32),
            np.asarray(obs_base, dtype=np.int32), dims[0], dims[1], dims[2],
            self._obs if obs is None else obs, query_count,
            self._query if query is None else query, self._query_out,
            self._delta, self._dump, header)
        return header

    def raw_call(self, op, **kwargs):
        """Unchecked call for contract tests (e.g. an unknown op code)."""
        return self._result(self._call(op, **kwargs))

    def reset(self, epoch, window_base):
        return self._result(self._call(OP_RESET, epoch=int(epoch),
                                       window_base=_coords3(window_base, "window_base")))

    def integrate(self, epoch, frame_seq, window_base, obs_base, obs):
        buffer, nx, ny, nz = self._obs_buffer(obs)
        header = self._call(OP_INTEGRATE, epoch=int(epoch), frame_seq=int(frame_seq),
                            window_base=_coords3(window_base, "window_base"),
                            obs_base=_coords3(obs_base, "obs_base"), obs=buffer,
                            dims=(nx, ny, nz))
        n = int(header[H["N_DELTA"]])
        if n > self.config.delta_capacity:
            raise RuntimeError("delta count exceeds capacity")
        return self._result(header, self.decode_delta(header, self._delta[:n]))

    def decode_delta(self, header, records):
        """Changed-word records -> world ((gx, gy, gz, state), ...)."""
        mx = self.config.dims[0]
        base = [_signed(header[H["BASE"] + a]) for a in range(3)]
        off_x = int(header[H["OFFSET"]])
        changes = []
        for record in (int(r) for r in records):
            states, mask = record & 0xFFFF, (record >> 16) & 0xFF
            word, ly, lz = (record >> 24) & 0xFF, (record >> 32) & 0xFF, (record >> 40) & 0xFF
            if mask == 0 or record >> 48:
                raise RuntimeError("malformed delta record")
            for lane in range(8):
                if mask & (1 << lane):
                    lx = (8 * word + lane - off_x) % mx
                    changes.append((base[0] + lx, base[1] + ly, base[2] + lz,
                                    (states >> (2 * lane)) & 3))
        if len(set(c[:3] for c in changes)) != len(changes):
            raise RuntimeError("delta lists one cell twice")
        return changes

    def query(self, coords):
        buffer, count = self._query_buffer(coords)
        header = self._call(OP_QUERY, query=buffer, query_count=count)
        return self._result(header), self.decode_query(self._query_out[:count].copy())

    def snapshot(self):
        """Local-order int8 log-odds [lz, ly, lx] rebuilt from the physical dump."""
        header = self._call(OP_DUMP)
        result = self._result(header)
        if not result.ok:
            return result, None
        mx, my, mz = self.config.dims
        words = self._dump.copy()
        lanes = np.stack([(words >> np.uint64(8 * j)) & np.uint64(0xFF)
                          for j in range(8)], axis=1).astype(np.uint8).view(np.int8)
        physical = lanes.reshape(mz, my, mx)
        off = result.offsets
        local = np.roll(physical, shift=(-off[2], -off[1], -off[0]), axis=(0, 1, 2))
        return result, np.ascontiguousarray(local)


class RollingReference(_RollingBackend):
    """Plain C++ reference with caller-owned state (many instances allowed)."""

    def __init__(self, library):
        self.lib = ctypes.CDLL(str(Path(library).resolve()))
        L = self.lib
        L.fpga_m7_roll_ref_config.argtypes = [U32]
        L.fpga_m7_roll_ref_state_bytes.restype = ctypes.c_uint32
        out = np.zeros(16, dtype=np.uint32)
        L.fpga_m7_roll_ref_config(out)
        self.config = RollingConfig.from_words(out)
        c = self.config
        L.fpga_m7_roll_ref_init.argtypes = [ctypes.c_void_p]
        L.fpga_m7_roll_ref_reset.argtypes = [ctypes.c_void_p, ctypes.c_uint32, I32, U32]
        L.fpga_m7_roll_ref_integrate.argtypes = [
            ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, I32, I32,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, U8, I32, U32]
        L.fpga_m7_roll_ref_query.argtypes = [ctypes.c_void_p, ctypes.c_uint32, I32, U32, U32]
        L.fpga_m7_roll_ref_dump.argtypes = [ctypes.c_void_p, I8, U32]
        for name in ("init", "reset", "integrate", "query", "dump"):
            getattr(L, "fpga_m7_roll_ref_" + name).restype = None
        self._state = ctypes.create_string_buffer(int(L.fpga_m7_roll_ref_state_bytes()) + 64)
        # Align the placement-new target to 16 bytes.
        address = ctypes.addressof(self._state)
        self._ptr = ctypes.c_void_p(address + (-address) % 16)
        L.fpga_m7_roll_ref_init(self._ptr)
        self._changes = np.zeros(4 * c.cells, dtype=np.int32)
        self._query_out = np.zeros(c.query_capacity, dtype=np.uint32)
        self._cells = np.zeros(c.cells, dtype=np.int8)

    def _header(self):
        return np.full(self.config.header_words, 0xDEADBEEF, dtype=np.uint32)

    def reset(self, epoch, window_base):
        header = self._header()
        self.lib.fpga_m7_roll_ref_reset(self._ptr, int(epoch),
                                       np.asarray(_coords3(window_base, "window_base"),
                                                  dtype=np.int32), header)
        return self._result(header)

    def integrate(self, epoch, frame_seq, window_base, obs_base, obs):
        buffer, nx, ny, nz = self._obs_buffer(obs)
        header = self._header()
        self.lib.fpga_m7_roll_ref_integrate(
            self._ptr, int(epoch), int(frame_seq),
            np.asarray(_coords3(window_base, "window_base"), dtype=np.int32),
            np.asarray(_coords3(obs_base, "obs_base"), dtype=np.int32),
            nx, ny, nz, buffer, self._changes, header)
        n = int(header[H["N_DELTA"]])
        rows = self._changes[:4 * n].reshape(n, 4)
        return self._result(header, [tuple(int(v) for v in row) for row in rows])

    def query(self, coords):
        buffer, count = self._query_buffer(coords)
        header = self._header()
        self.lib.fpga_m7_roll_ref_query(self._ptr, count, buffer, self._query_out, header)
        return self._result(header), self.decode_query(self._query_out[:count].copy())

    def snapshot(self):
        header = self._header()
        self.lib.fpga_m7_roll_ref_dump(self._ptr, self._cells, header)
        result = self._result(header)
        if not result.ok:
            return result, None
        mx, my, mz = self.config.dims
        return result, self._cells.reshape(mz, my, mx).copy()


# ---------------------------------------------------------------------------
# Host-side fusion of M7Pipeline navigation outputs and the Nav2-side mirror.

@dataclass(frozen=True)
class NavigationDelta:
    """Compact change record for one accepted frame (changes in odom voxels)."""
    epoch: int
    delta_seq: int  # contiguous from 1 within an epoch
    frame_id: int
    acquired_ns: int
    clock_id: str
    expires_at_ns: int
    window_base: tuple
    window_dims: tuple
    voxel_m: float
    changes: tuple  # ((gx, gy, gz, new_state), ...), eviction implied by window_base
    backend: str = BACKEND


@dataclass(frozen=True)
class FusionResult:
    accepted: bool
    reason: str
    delta: Optional[NavigationDelta] = None
    kernel: Optional[CallResult] = None

    @property
    def grid(self):
        """B1: the PL navigation grid of this frame (host.pl_grid_layer.PlGridPlanes) or None."""
        return None if self.kernel is None else self.kernel.grid


@dataclass(frozen=True)
class MapSnapshot:
    valid: bool
    reason: str
    epoch: int
    delta_seq: int
    window_base: tuple
    logodds: Optional[np.ndarray]  # [lz, ly, lx] int8, None when not valid
    states: Optional[np.ndarray]


@dataclass(frozen=True)
class MemoryQuery:
    valid: bool
    reason: str
    epoch: int
    delta_seq: int
    map_age_ns: Optional[int]
    states: Optional[np.ndarray]    # None when not valid
    logodds: Optional[np.ndarray]
    outside: Optional[np.ndarray]


class RollingFusion:
    """Consume validated M7Pipeline navigation outputs into a rolling map."""

    def __init__(self, backend, *, voxel_m, window_anchor_voxels=None,
                 max_map_age_ns=200_000_000):
        if not isinstance(backend, _RollingBackend):
            raise ValueError("explicit rolling backend required")
        if not (isinstance(voxel_m, float) and math.isfinite(voxel_m) and voxel_m > 0):
            raise ValueError("voxel_m must be a positive finite float in metres")
        if (isinstance(max_map_age_ns, bool) or not isinstance(max_map_age_ns, int) or
                max_map_age_ns < 0):
            raise ValueError("max_map_age_ns must be a nonnegative integer")
        mx, my, mz = backend.config.dims
        if window_anchor_voxels is None:
            window_anchor_voxels = (mx // 2, my // 2, mz // 4)
        anchor = tuple(int(v) for v in window_anchor_voxels)
        if len(anchor) != 3 or any(not 0 <= a < d for a, d in zip(anchor, (mx, my, mz))):
            raise ValueError("window anchor must lie inside the window")
        self.backend, self.voxel_m, self.anchor = backend, voxel_m, anchor
        self.max_map_age_ns = max_map_age_ns
        self.counts = Counter()
        self._epoch = 0
        self._key = None
        self._clock_id = None
        self._last_acquired_ns = None
        self._delta_seq = 0
        # Go-live 2026-10-26: link recovery only for a backend that asks for it (the
        # remote KV260 backend). The CPU/in-process backends keep the raise-on-fault path.
        self._recovery = bool(getattr(backend, "link_recovery", False))
        self._link_errors = tuple(getattr(backend, "link_errors", ())) if self._recovery else ()
        self._force_reset = None          # reason the map must be rebuilt (both sides cleared)
        self._unanswered_acq = None       # acquired_ns of the INTEGRATE whose ACK was lost
        self._warn_at = 0.0

    @property
    def epoch(self):
        return self._epoch

    def enable_grid(self, band, *, skip_changes=False):
        """B1 (2026-11-02): ask the backend for the PL navigation grid with every INTEGRATE (``band`` a
        host.rolling_nav_layer.HeightBand). Only a backend with ``grid_capable`` True (the remote KV260 backend
        whose server announces "rolling_grid") can; returns whether it is on. ``skip_changes``: the deltas'
        per-lane change lists are not decoded (FusionResult.delta.changes is ()) -- only when nobody consumes
        them (no bridge, no in-process fallback)."""
        backend = self.backend
        if not getattr(backend, "grid_capable", False):
            self.counts["grid_unavailable"] += 1
            return False
        backend.grid_band = band
        backend.grid_voxel_m = self.voxel_m
        backend.skip_changes = bool(skip_changes)
        return True

    def window_base_for(self, robot_position_m):
        position = np.asarray(robot_position_m, dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError("robot position must be finite odom XYZ metres")
        voxel = np.floor(position / self.voxel_m).astype(np.int64)
        return tuple(int(v - a) for v, a in zip(voxel, self.anchor))

    def lattice_base(self, grid):
        """Integer odom voxel of a per-frame GridSpec origin, or None."""
        if not math.isclose(float(grid.voxel_m), self.voxel_m, rel_tol=1e-12, abs_tol=0):
            return None
        base = []
        for value in grid.origin_m:
            ratio = float(value) / self.voxel_m
            nearest = round(ratio)
            if abs(ratio - nearest) > 1e-6:
                return None
            base.append(int(nearest))
        return tuple(base)

    def _drop(self, reason):
        self.counts["dropped:" + reason] += 1
        return FusionResult(False, reason)

    def ingest(self, output, pipeline, *, now_ns, clock_id, robot_position_m):
        if getattr(output, "channel", None) != "navigation":
            raise ValueError("only navigation outputs feed the rolling map")
        checked = output.validate_at(pipeline, now_ns, clock_id)
        if not checked.valid:
            return self._drop(checked.reason)
        identity = checked.identity
        grid = checked.payload.get("grid")
        flags = checked.payload.get("voxel_flags")
        if grid is None or flags is None:
            return self._drop("missing_geometry_payload")
        obs_base = self.lattice_base(grid)
        if obs_base is None:
            return self._drop("grid_not_on_map_lattice")
        if not 0 <= identity.frame_id < 2 ** 32:
            return self._drop("frame_id_exceeds_u32")
        window_base = self.window_base_for(robot_position_m)
        key = (identity.context, checked.generation)
        if self._recovery:
            return self._ingest_recovering(identity, key, window_base, obs_base, flags, now_ns)
        if key != self._key:
            # New stream/clock/calibration/local frame/pose epoch or a new
            # pipeline generation: the old map must not leak into it.
            self._epoch += 1
            reset = self.backend.reset(self._epoch, window_base)
            if not reset.ok:
                raise RuntimeError("rolling map reset failed: " + reset.status_name)
            self._key, self._clock_id = key, identity.clock_id
            self._last_acquired_ns, self._delta_seq = None, 0
            self.counts["map_reset"] += 1
        result = self.backend.integrate(self._epoch, identity.frame_id, window_base,
                                        obs_base, np.ascontiguousarray(flags, dtype=np.uint8))
        if not result.ok:
            self.counts["kernel_rejected:" + result.status_name] += 1
            return FusionResult(False, "kernel_" + result.status_name, kernel=result)
        if result.delta_seq != self._delta_seq + 1 or result.epoch != self._epoch:
            raise RuntimeError("rolling map sequence diverged from host bookkeeping")
        self._delta_seq = result.delta_seq
        self._last_acquired_ns = identity.acquired_ns
        self.counts["accepted"] += 1
        delta = NavigationDelta(self._epoch, result.delta_seq, identity.frame_id,
                                identity.acquired_ns, identity.clock_id,
                                identity.acquired_ns + self.max_map_age_ns,
                                result.window_base, self.backend.config.dims,
                                self.voxel_m, result.changes)
        return FusionResult(True, "fused", delta, result)

    # ---------------------------------------------------------- link recovery (remote only)
    # Rules (golive/HANDOVER.md, 2026-10-26): a ROLL wait never exceeds the frame's freshness
    # budget; a lost INTEGRATE answer is resolved by a REPLAY probe next frame (late ACK with
    # a contiguous header is adopted -- the navigation mirror then misses one delta and
    # resyncs; not executed is forgotten); any non-contiguous / uninitialized / foreign-epoch
    # / undecodable answer means the map is lost: epoch += 1, RESET, both sides start empty
    # (the mirror clears on a fresh epoch at delta_seq 1). No delta is built from anything
    # but a fully decoded, contiguous ACK.
    REBUILD_MIN_NS = 20_000_000

    def _warn(self, text):
        now = time.monotonic()
        if now - self._warn_at >= 5.0:
            self._warn_at = now
            sys.stderr.write("rolling map: %s\n" % text)

    def _map_lost(self, reason):
        self._force_reset = reason
        self.counts["map_lost:" + reason] += 1
        self._warn("map lost (%s): epoch %d will be rebuilt empty on both sides"
                   % (reason, self._epoch))

    def _budget_ns(self, deadline_mono_ns):
        return deadline_mono_ns - time.monotonic_ns()

    def _resolve_unanswered(self):
        """None when settled, else the drop reason for this frame."""
        backend = self.backend
        cmd = backend.unanswered
        if not getattr(backend, "replay_ok", False):
            backend.clear_unanswered()
            self._map_lost("unanswered_no_replay")
            return None
        from host.kv_rolling import RESULT_NOT_EXECUTED, RESULT_NOT_HELD, RESULT_BAD_REQUEST, RESULT_OK
        try:
            ack = backend.probe(cmd)
        except self._link_errors as exc:
            if type(exc).__name__ in ("LinkTimeout", "RollBudgetExpired"):
                self.counts["roll_unreachable"] += 1
                return "expired_roll_unreachable"
            backend.clear_unanswered()
            self._map_lost("probe_error:" + type(exc).__name__)
            return None
        backend.clear_unanswered()
        if ack["result"] in (RESULT_NOT_EXECUTED, RESULT_NOT_HELD, RESULT_BAD_REQUEST):
            self.counts["roll_recovered:not_executed"] += 1
            return None
        if ack["result"] != RESULT_OK:
            self._map_lost("late_ack_device_error")
            return None
        res = backend._result(ack["header"])
        if not res.ok:                    # the kernel refused it: the map did not change
            self.counts["roll_recovered:late_reject"] += 1
            return None
        if res.epoch == self._epoch and res.delta_seq == self._delta_seq + 1:
            self._delta_seq = res.delta_seq
            self._last_acquired_ns = self._unanswered_acq
            self.counts["roll_recovered:late_ack"] += 1
            return None
        self._map_lost("late_ack_diverged")
        return None

    def _ingest_recovering(self, identity, key, window_base, obs_base, flags, now_ns):
        backend = self.backend
        remaining = identity.acquired_ns + self.max_map_age_ns - int(now_ns)
        deadline = time.monotonic_ns() + remaining
        backend.set_deadline_ns(deadline)
        stage = "probe"
        try:
            if backend.unanswered is not None:
                if key != self._key or self._force_reset:
                    backend.clear_unanswered()      # the RESET below wipes it anyway
                else:
                    why = self._resolve_unanswered()
                    if why is not None:
                        return self._drop(why)
            obs = np.ascontiguousarray(flags, dtype=np.uint8)
            for attempt in (0, 1):
                if key != self._key or self._force_reset:
                    stage = "reset"
                    rebuilt = self._force_reset if key == self._key else None
                    self._epoch += 1
                    reset = backend.reset(self._epoch, window_base)
                    if not reset.ok:
                        self._map_lost("reset_" + reset.status_name)
                        return self._drop("roll_reset_failed")
                    self._key, self._clock_id = key, identity.clock_id
                    self._last_acquired_ns, self._delta_seq = None, 0
                    self._force_reset = None
                    self.counts["map_reset"] += 1
                    if rebuilt:
                        self.counts["map_rebuilt:" + rebuilt] += 1
                stage = "integrate"
                self._unanswered_acq = identity.acquired_ns
                result = backend.integrate(self._epoch, identity.frame_id, window_base, obs_base, obs)
                if result.ok and result.epoch == self._epoch and result.delta_seq == self._delta_seq + 1:
                    break
                if not result.ok and result.status_name not in ("uninitialized", "stale_epoch"):
                    self.counts["kernel_rejected:" + result.status_name] += 1
                    return FusionResult(False, "kernel_" + result.status_name, kernel=result)
                self._map_lost(("kernel_" + result.status_name) if not result.ok else "seq_diverged")
                if attempt == 1 or self._budget_ns(deadline) < self.REBUILD_MIN_NS:
                    return self._drop("expired_roll_rebuild")
        except self._link_errors as exc:
            name = type(exc).__name__
            if name == "RollBudgetExpired":
                if stage == "reset":
                    self._force_reset = self._force_reset or "reset_not_sent"
                return self._drop("expired_roll_budget")
            if name == "LinkTimeout":
                if stage == "reset":
                    self._map_lost("reset_timeout")
                return self._drop("expired_roll_timeout")
            self._map_lost("roll_error:" + name)
            return self._drop("roll_error")
        finally:
            backend.set_deadline_ns(None)
        self._delta_seq = result.delta_seq
        self._last_acquired_ns = identity.acquired_ns
        self.counts["accepted"] += 1
        delta = NavigationDelta(self._epoch, result.delta_seq, identity.frame_id,
                                identity.acquired_ns, identity.clock_id,
                                identity.acquired_ns + self.max_map_age_ns,
                                result.window_base, backend.config.dims,
                                self.voxel_m, result.changes)
        return FusionResult(True, "fused", delta, result)

    def _cap_wait(self, clear=False):
        """Remote only: a QUERY/DUMP (resync answer, under the worker's lock) waits at most
        one freshness budget."""
        if self._recovery:
            self.backend.set_deadline_ns(None if clear else time.monotonic_ns() + self.max_map_age_ns)

    def _link_unusable(self):
        if not self._recovery:
            return None
        if self._force_reset:
            return "map_lost"
        if getattr(self.backend, "unanswered", None) is not None:
            return "roll_pending"     # a DUMP/QUERY now would displace the KV's cached answer
        return None

    def _freshness(self, now_ns, clock_id):
        if self._key is None or self._last_acquired_ns is None:
            return "no_fused_frame", None
        if clock_id != self._clock_id:
            return "clock_mismatch", None
        age = int(now_ns) - self._last_acquired_ns
        if age < 0:
            return "future_query", age
        if age > self.max_map_age_ns:
            # Cells are kept (never cleared by time); consumers must treat
            # the whole map as stale and apply their own stop/wait rules.
            return "map_stale", age
        return None, age

    def query_world(self, coords, *, now_ns, clock_id):
        reason, age = self._freshness(now_ns, clock_id)
        if reason is None:
            reason = self._link_unusable()
        if reason is not None:
            return MemoryQuery(False, reason, self._epoch, self._delta_seq, age,
                               None, None, None)
        try:
            self._cap_wait()
            result, (states, logodds, outside) = self.backend.query(coords)
        except self._link_errors as exc:
            self.counts["query_error:" + type(exc).__name__] += 1
            return MemoryQuery(False, "roll_error:" + type(exc).__name__, self._epoch,
                               self._delta_seq, age, None, None, None)
        finally:
            self._cap_wait(clear=True)
        if not result.ok:
            return MemoryQuery(False, "kernel_" + result.status_name, self._epoch,
                               self._delta_seq, age, None, None, None)
        return MemoryQuery(True, "map_current", result.epoch, result.delta_seq, age,
                           states, logodds, outside)

    def snapshot(self, *, now_ns, clock_id):
        reason, _ = self._freshness(now_ns, clock_id)
        if reason is None:
            reason = self._link_unusable()
        if reason is not None:
            return MapSnapshot(False, reason, self._epoch, self._delta_seq,
                               (0, 0, 0), None, None)
        try:
            self._cap_wait()
            result, local = self.backend.snapshot()
        except self._link_errors as exc:
            self.counts["snapshot_error:" + type(exc).__name__] += 1
            return MapSnapshot(False, "roll_error:" + type(exc).__name__, self._epoch,
                               self._delta_seq, (0, 0, 0), None, None)
        finally:
            self._cap_wait(clear=True)
        if not result.ok:
            return MapSnapshot(False, "kernel_" + result.status_name, self._epoch,
                               self._delta_seq, (0, 0, 0), None, None)
        return MapSnapshot(True, "map_current", result.epoch, result.delta_seq,
                           result.window_base, local,
                           self.backend.config.state_of(local))


class RollingDeltaMirror:
    """Reference Nav2-side consumer: rebuilds tri-state from deltas only."""

    def __init__(self, dims):
        self.dims = tuple(int(v) for v in dims)
        self.counts = Counter()
        self.epoch = None
        self.delta_seq = 0
        self.window_base = None
        self.states = None
        self.needs_resync = False

    def _reject(self, reason):
        self.counts["rejected:" + reason] += 1
        return False, reason

    def _move(self, new_base):
        mx, my, mz = self.dims
        old, old_base = self.states, self.window_base
        new = np.zeros((mz, my, mx), dtype=np.uint8)
        d = [n - o for n, o in zip(new_base, old_base)]
        # new local l' holds world new_base + l' = old_base + (l' + d).
        src = [slice(max(0, dd), min(m, m + dd)) for dd, m in zip(d, (mx, my, mz))]
        dst = [slice(max(0, -dd), min(m, m - dd)) for dd, m in zip(d, (mx, my, mz))]
        if all(s.start < s.stop for s in src):
            new[dst[2], dst[1], dst[0]] = old[src[2], src[1], src[0]]
        self.states, self.window_base = new, tuple(new_base)

    def apply(self, delta, *, now_ns):
        if not isinstance(delta, NavigationDelta) or delta.window_dims != self.dims:
            raise ValueError("NavigationDelta with matching window dims required")
        if self.epoch is not None and delta.epoch < self.epoch:
            return self._reject("stale_epoch")
        fresh_epoch = self.epoch is None or delta.epoch > self.epoch
        if fresh_epoch and delta.delta_seq != 1:
            self.needs_resync = True
            return self._reject("sequence_gap")
        if not fresh_epoch:
            if delta.delta_seq <= self.delta_seq:
                return self._reject("duplicate_or_old_delta")
            if self.needs_resync:
                return self._reject("awaiting_resync")
            if delta.delta_seq != self.delta_seq + 1:
                self.needs_resync = True
                return self._reject("sequence_gap")
        if now_ns > delta.expires_at_ns:
            # Skipping it would desynchronise the mirror, applying it would
            # present stale geometry as current: require a snapshot instead.
            self.needs_resync = True
            if fresh_epoch:
                self.epoch, self.delta_seq = delta.epoch, 0
            return self._reject("expired_delta")
        if fresh_epoch:
            mx, my, mz = self.dims
            self.epoch, self.delta_seq = delta.epoch, 0
            self.states = np.zeros((mz, my, mx), dtype=np.uint8)
            self.window_base = tuple(delta.window_base)
            self.needs_resync = False
            self.counts["epoch_start"] += 1
        if tuple(delta.window_base) != self.window_base:
            self._move(delta.window_base)
        mx, my, mz = self.dims
        for gx, gy, gz, state in delta.changes:
            lx, ly, lz = (gx - self.window_base[0], gy - self.window_base[1],
                          gz - self.window_base[2])
            if not (0 <= lx < mx and 0 <= ly < my and 0 <= lz < mz) or state > 2:
                raise RuntimeError("delta change outside its own window")
            self.states[lz, ly, lx] = state
        self.delta_seq = delta.delta_seq
        self.counts["accepted"] += 1
        return True, "applied"

    def resync(self, snapshot):
        if not isinstance(snapshot, MapSnapshot) or not snapshot.valid:
            return self._reject("invalid_snapshot")
        self.epoch, self.delta_seq = snapshot.epoch, snapshot.delta_seq
        self.window_base = tuple(snapshot.window_base)
        self.states = snapshot.states.copy()
        self.needs_resync = False
        self.counts["resync"] += 1
        return True, "resynced"
