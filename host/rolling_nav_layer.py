"""Rolling map -> Nav2 local obstacle layer: the ROS-free half (2026-09-27).

See docs/design/ROLLING_NAV2_LAYER_20260927.md. The chosen integration is a
bridge that keeps a ``RollingDeltaMirror`` of the tri-state window, projects
it to 2-D with the D-035 height band and publishes a whole
``nav_msgs/OccupancyGrid`` for a Nav2 ``StaticLayer`` in the rolling local
costmap. Everything here is pure Python/NumPy so both interpreters test it:

* ``HeightBand`` / ``project_columns``: D-035 height projection;
* ``occupancy_values``: OCCUPIED/FREE/UNKNOWN -> 100/0/-1;
* ``encode_delta`` ... ``decode_snapshot``: byte transport for deltas and
  resync snapshots (carried as ``std_msgs/UInt8MultiArray``);
* ``RollingLayerBridge``: rejects wrong epochs, gaps, duplicates and expired
  deltas (via ``RollingDeltaMirror``), asks for a snapshot, replays deltas that
  arrived while waiting, and reports map freshness;
* ``rolling_map_permit``: the navigation consumer's stop rule;
* ``check_costmap_config``: refuses Nav2 settings that silently break the
  mapping (e.g. ``track_unknown_space: false`` turns UNKNOWN into free space).
"""

from collections import Counter
from dataclasses import dataclass, replace
import json
import math
import struct

import numpy as np

from host.m7_rolling import (FREE, OCCUPIED, UNKNOWN, MapSnapshot, NavigationDelta,
                             RollingDeltaMirror)

# OccupancyGrid values understood by StaticLayer with trinary_costmap and
# track_unknown_space enabled (lethal_cost_threshold 100, unknown -1 == 255).
GRID_LETHAL, GRID_FREE, GRID_UNKNOWN = 100, 0, -1
MAX_MAP_AGE_NS = 200_000_000  # D-035: map not updated for 200 ms -> stop
BODY_MARGIN_MAX_M = 0.05      # body seed margin cap (footprint + <= 5 cm), see RollingLayerBridge
FRONT_SEED_DEG = 35.0         # front-sector seed half angle (2026-10-26, default OFF), see RollingLayerBridge
# Swept-area clearing (2026-10-28, owner approved; car_inner.sh SWEPT_CLEAR_S, default 300 s on the car), see
# SweptCells / RollingLayerBridge._apply_swept:
SWEPT_EVIDENCE_NEAR_M = 1.0   # a swept cell is re-marked OCCUPIED only by an obstacle seen within this of the body
SWEPT_EVIDENCE_MIN_H_M = 0.15  # ... with an OCCUPIED voxel whose centre is at least this above the ground
SWEPT_MAX_STEP_M = 0.5        # successive body poses farther apart than this are not joined (a jump, not a drive)
BELOW, GROUND, BAND, ABOVE = 0, 1, 2, 3
_UM = 1_000_000  # heights are compared in integer micrometres


class HeightBand:
    """D-035 height rule: [ground+5 cm, ground+robot_height+5 cm] is obstacle.

    Layers whose voxel CENTRE lies within +-5 cm of the ground are ground;
    everything below/above is ignored. Both ground_z_m (odom z of the floor)
    and robot_height_m come from calibration and have no defaults.
    """

    def __init__(self, *, ground_z_m, robot_height_m, clearance_m=0.05, ground_tol_m=0.05):
        for name, value in (("ground_z_m", ground_z_m), ("robot_height_m", robot_height_m),
                            ("clearance_m", clearance_m), ("ground_tol_m", ground_tol_m)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or \
                    not math.isfinite(value):
                raise ValueError("%s must be a finite number of metres" % name)
        if robot_height_m <= 0 or clearance_m < 0 or ground_tol_m < 0:
            raise ValueError("robot height must be positive, margins nonnegative")
        self.ground_z_m, self.robot_height_m = float(ground_z_m), float(robot_height_m)
        self.clearance_m, self.ground_tol_m = float(clearance_m), float(ground_tol_m)

    def class_of(self, k, voxel_m):
        """Class of the world voxel layer k (voxel centre vs. ground)."""
        centre = (int(k) + 0.5) * float(voxel_m)
        h = round((centre - self.ground_z_m) * _UM)
        if h < -round(self.ground_tol_m * _UM):
            return BELOW
        if h < round(self.clearance_m * _UM):
            return GROUND
        if h <= round((self.robot_height_m + self.clearance_m) * _UM):
            return BAND
        return ABOVE

    def classify(self, window_base_z, mz, voxel_m):
        """Class of each local z layer 0..mz-1 of a window based at voxel z."""
        return np.array([self.class_of(int(window_base_z) + lz, voxel_m)
                         for lz in range(int(mz))], dtype=np.uint8)

    def band_layers(self, window_base_z, mz, voxel_m):
        """Local z indices of the obstacle band; the band must fit the window."""
        lo = math.floor((self.ground_z_m + self.clearance_m) / voxel_m) - 2
        hi = math.ceil((self.ground_z_m + self.robot_height_m + self.clearance_m) / voxel_m) + 2
        world = [k for k in range(lo, hi + 1) if self.class_of(k, voxel_m) == BAND]
        z0 = int(window_base_z)
        if not world or world[0] < z0 or world[-1] >= z0 + int(mz):
            raise ValueError("obstacle height band is not fully inside the map window")
        return np.array([k - z0 for k in world], dtype=np.int64)


def project_columns(states, band_layers):
    """3-D tri-state [lz, ly, lx] -> 2-D tri-state [ly, lx].

    OCCUPIED if any band voxel is OCCUPIED; FREE only if every band voxel is
    FREE; otherwise UNKNOWN ("not fully seen is not free").
    """
    states = np.asarray(states)
    if states.ndim != 3 or states.dtype != np.uint8 or np.any(states > OCCUPIED):
        raise ValueError("states must be a uint8 [lz, ly, lx] tri-state array")
    band = states[np.asarray(band_layers, dtype=np.int64)]
    if band.shape[0] == 0:
        raise ValueError("empty obstacle band")
    out = np.full(states.shape[1:], UNKNOWN, dtype=np.uint8)
    out[np.all(band == FREE, axis=0)] = FREE
    out[np.any(band == OCCUPIED, axis=0)] = OCCUPIED
    return out


def occupancy_values(tri2d):
    """2-D tri-state -> int8 OccupancyGrid values (row-major y, x)."""
    tri2d = np.asarray(tri2d)
    if np.any(tri2d > OCCUPIED):
        raise ValueError("unknown tri-state value")
    out = np.full(tri2d.shape, GRID_UNKNOWN, dtype=np.int8)
    out[tri2d == FREE] = GRID_FREE
    out[tri2d == OCCUPIED] = GRID_LETHAL
    return out


@dataclass(frozen=True)
class GridFields:
    """Everything a nav_msgs/OccupancyGrid needs (no ROS import here)."""
    frame_id: str
    resolution: float
    width: int
    height: int
    origin_xy: tuple
    data: np.ndarray  # int8, len width*height, index ly*width + lx
    epoch: int
    delta_seq: int
    acquired_ns: int


# ---------------------------------------------------------------------------
# Byte transport. A 4-byte magic, uint32 JSON header length, JSON header,
# then a little-endian payload. Decoders validate everything.

DELTA_MAGIC, SNAPSHOT_MAGIC = b"RDL1", b"RSN1"


def _pack(magic, header, payload):
    raw = json.dumps(header, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return magic + struct.pack("<I", len(raw)) + raw + payload


def _unpack(data, magic):
    data = bytes(data)
    if len(data) < 8 or data[:4] != magic:
        raise ValueError("bad magic")
    (n,) = struct.unpack_from("<I", data, 4)
    if 8 + n > len(data):
        raise ValueError("truncated header")
    header = json.loads(data[8:8 + n].decode("utf-8"))
    if not isinstance(header, dict):
        raise ValueError("header must be an object")
    return header, data[8 + n:]


def _int(header, key, low=None):
    value = header.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or (low is not None and value < low):
        raise ValueError("bad field %s" % key)
    return value


def _ints3(header, key, positive=False):
    value = header.get(key)
    if (not isinstance(value, list) or len(value) != 3 or
            any(isinstance(v, bool) or not isinstance(v, int) for v in value) or
            (positive and any(v <= 0 for v in value))):
        raise ValueError("bad field %s" % key)
    return tuple(value)


def _voxel(header):
    value = header.get("voxel_m")
    if not isinstance(value, float) or not math.isfinite(value) or value <= 0:
        raise ValueError("bad field voxel_m")
    return value


def encode_delta(delta):
    if not isinstance(delta, NavigationDelta):
        raise ValueError("NavigationDelta required")
    changes = np.asarray(delta.changes, dtype=np.int64).reshape(-1, 4)
    if changes.size and (np.any(np.abs(changes) >= 2 ** 31)):
        raise ValueError("change overflows int32")
    header = dict(epoch=delta.epoch, delta_seq=delta.delta_seq, frame_id=delta.frame_id,
                  acquired_ns=delta.acquired_ns, clock_id=delta.clock_id,
                  expires_at_ns=delta.expires_at_ns, window_base=list(delta.window_base),
                  window_dims=list(delta.window_dims), voxel_m=float(delta.voxel_m),
                  n=int(changes.shape[0]), backend=delta.backend)
    return _pack(DELTA_MAGIC, header, changes.astype("<i4").tobytes())


def decode_delta(data):
    header, payload = _unpack(data, DELTA_MAGIC)
    n = _int(header, "n", 0)
    if len(payload) != 16 * n:
        raise ValueError("payload length does not match change count")
    rows = np.frombuffer(payload, dtype="<i4").reshape(n, 4) if n else np.zeros((0, 4), "<i4")
    if n and (np.any(rows[:, 3] < 0) or np.any(rows[:, 3] > OCCUPIED)):
        raise ValueError("change state out of range")
    clock_id, backend = header.get("clock_id"), header.get("backend")
    if not isinstance(clock_id, str) or not clock_id or not isinstance(backend, str):
        raise ValueError("bad clock/backend")
    return NavigationDelta(
        _int(header, "epoch", 1), _int(header, "delta_seq", 1), _int(header, "frame_id", 0),
        _int(header, "acquired_ns", 0), clock_id, _int(header, "expires_at_ns", 0),
        _ints3(header, "window_base"), _ints3(header, "window_dims", positive=True),
        _voxel(header), tuple(tuple(int(v) for v in row) for row in rows), backend)


@dataclass(frozen=True)
class WireSnapshot:
    snapshot: MapSnapshot
    acquired_ns: int  # capture time of the last frame fused into this map
    clock_id: str
    voxel_m: float


def encode_snapshot(snapshot, *, acquired_ns, clock_id, voxel_m):
    if not isinstance(snapshot, MapSnapshot) or not snapshot.valid:
        raise ValueError("valid MapSnapshot required")
    states = np.ascontiguousarray(snapshot.states, dtype=np.uint8)
    header = dict(epoch=snapshot.epoch, delta_seq=snapshot.delta_seq,
                  window_base=list(snapshot.window_base), dims=list(states.shape[::-1]),
                  acquired_ns=int(acquired_ns), clock_id=clock_id, voxel_m=float(voxel_m))
    return _pack(SNAPSHOT_MAGIC, header, states.tobytes())


def decode_snapshot(data):
    header, payload = _unpack(data, SNAPSHOT_MAGIC)
    mx, my, mz = _ints3(header, "dims", positive=True)
    if len(payload) != mx * my * mz:
        raise ValueError("snapshot payload length does not match dims")
    states = np.frombuffer(payload, dtype=np.uint8).reshape(mz, my, mx).copy()
    if np.any(states > OCCUPIED):
        raise ValueError("snapshot state out of range")
    clock_id = header.get("clock_id")
    if not isinstance(clock_id, str) or not clock_id:
        raise ValueError("bad clock_id")
    snap = MapSnapshot(True, "map_current", _int(header, "epoch", 1),
                       _int(header, "delta_seq", 0), _ints3(header, "window_base"),
                       None, states)
    return WireSnapshot(snap, _int(header, "acquired_ns", 0), clock_id, _voxel(header))


# ---------------------------------------------------------------------------

class RollingLayerBridge:
    """Nav2-side bridge state: mirror + projection + freshness. No ROS here.

    ``on_delta`` / ``on_snapshot`` return ``(changed_grid, reason, request)``
    where ``request`` is a dict to send as a resync request, or None.
    """

    def __init__(self, *, dims, voxel_m, band, clock_id, frame_id="odom",
                 max_map_age_ns=MAX_MAP_AGE_NS, resync_retry_ns=100_000_000,
                 pending_limit=16, body_radius_m=None, cold_start_width_m=None,
                 cold_start_timeout_ns=30_000_000_000, cold_start_log=None, body_margin_m=0.0,
                 front_seed_m=None, front_seed_deg=FRONT_SEED_DEG, swept_clear_s=None,
                 swept_evidence_near_m=SWEPT_EVIDENCE_NEAR_M, swept_evidence_min_h_m=SWEPT_EVIDENCE_MIN_H_M):
        if not isinstance(band, HeightBand):
            raise ValueError("HeightBand required")
        self.dims, self.voxel_m, self.band = tuple(int(v) for v in dims), float(voxel_m), band
        self.clock_id, self.frame_id = clock_id, frame_id
        self.max_map_age_ns, self.resync_retry_ns = int(max_map_age_ns), int(resync_retry_ns)
        self.pending_limit = int(pending_limit)
        if body_radius_m is not None and not (isinstance(body_radius_m, (int, float)) and
                                              math.isfinite(body_radius_m) and
                                              body_radius_m > 0):
            raise ValueError("body_radius_m must be a positive number of metres")
        self.body_radius_m = None if body_radius_m is None else float(body_radius_m)
        # Seed margin (2026-10-17, nav go-live r3, docs/design/REAL_CAR_NAV_GOLIVE_20261016.md 9):
        # UNKNOWN cells within body_radius_m + body_margin_m of the body centre are seeded FREE,
        # still only around the current pose. Nav2 rasterises the 16-gon outline with Bresenham
        # between vertex cells, so an outline cell's closest point can lie up to ~1.6 cm outside
        # the radius (random-pose sweep), and a cell at exactly the radius is lost to float
        # rounding (6 * 0.05 = 0.30000000000000004 > 0.30): the board's local costmap then had
        # three UNKNOWN cells on the CURRENT outline and DWB rejected all 419 trajectories
        # (ng2_1017). Capped at 5 cm (controller rule: the car's own footprint plus <= 5 cm).
        if not (isinstance(body_margin_m, (int, float)) and not isinstance(body_margin_m, bool)
                and math.isfinite(body_margin_m) and 0.0 <= body_margin_m <= BODY_MARGIN_MAX_M):
            raise ValueError("body_margin_m must be between 0 and %.2f m" % BODY_MARGIN_MAX_M)
        self.body_margin_m = float(body_margin_m)
        # Front-sector seed (2026-10-26, DEFAULT OFF -- it widens the owner's 09-27 rolling-layer rule, so it is on
        # only when the owner switches it on; car_up.sh CAR_FRONT_SEED_M). The camera's nearest visible ground is
        # ~0.64-0.73 m from the body centre (tools/cam_tilt10/geom.py), the body seed ends at radius + margin
        # (0.35 m): after an in-place turn to an unseen heading the ring between is UNKNOWN and DWB rejects every
        # forward trajectory (in-place dither). On: UNKNOWN cells whose closest point lies within front_seed_m of the
        # body centre AND whose centre bearing is within +-front_seed_deg of the CURRENT heading are sent FREE.
        # OCCUPIED and FREE are never changed; nothing is kept after the body has moved or turned. None / 0 = off.
        if front_seed_m is not None and not (
                isinstance(front_seed_m, (int, float)) and not isinstance(front_seed_m, bool)
                and math.isfinite(front_seed_m) and front_seed_m >= 0.0):
            raise ValueError("front_seed_m must be a nonnegative number of metres (0 / None = off)")
        if not (isinstance(front_seed_deg, (int, float)) and not isinstance(front_seed_deg, bool)
                and math.isfinite(front_seed_deg) and 0.0 < front_seed_deg <= 90.0):
            raise ValueError("front_seed_deg must be in (0, 90]")
        self.front_seed_m = float(front_seed_m) if front_seed_m else None
        self.front_seed_deg = float(front_seed_deg)
        # Swept-area clearing (2026-10-28, owner approved; widens the 09-27 rule "nothing is kept after the body has
        # moved on"): every cell the body circle (body_radius_m, no margin) has covered -- the car was physically
        # there, so it was free -- is sent FREE for swept_clear_s after the body left it, whatever the projection
        # says (OCCUPIED or UNKNOWN). Reason: a ground-calibration angle error lifts the floor into the lowest band
        # layers (10-28 14:19: 1050 false OCCUPIED cells around the car, DWB "Trajectory Hits Obstacle" everywhere).
        # A REAL obstacle put down later on a swept cell wins back: an OCCUPIED voxel centred >= swept_evidence_min_h_m
        # above the ground while the cell is within swept_evidence_near_m of the body ends the clearing of that cell
        # for good (until the body covers it again). Within 1 m an angle error of 6 deg lifts the floor <= ~10 cm,
        # so the floor alone does not reach 15 cm there; farther away the clearing stays until the car comes near.
        # Cost: an obstacle lower than 15 cm put on the car's own track is ignored for swept_clear_s.
        # None / 0 = off (the grid exactly as before).
        if swept_clear_s is not None and not (
                isinstance(swept_clear_s, (int, float)) and not isinstance(swept_clear_s, bool)
                and math.isfinite(swept_clear_s) and swept_clear_s >= 0.0):
            raise ValueError("swept_clear_s must be a nonnegative number of seconds (0 / None = off)")
        self.swept_clear_ns = int(swept_clear_s * 1e9) if swept_clear_s else None
        self.swept_evidence_near_m = float(swept_evidence_near_m)
        self.swept_evidence_min_h_m = float(swept_evidence_min_h_m)
        self.swept = SweptCells(self.voxel_m) if self.swept_clear_ns else None
        self.last_swept_cleared = 0
        self._tri = None
        if cold_start_width_m is not None and not (
                isinstance(cold_start_width_m, (int, float)) and
                math.isfinite(cold_start_width_m) and cold_start_width_m > 0):
            raise ValueError("cold_start_width_m must be a positive number of metres")
        self.cold_start_width_m = (None if cold_start_width_m is None
                                   else float(cold_start_width_m))
        self.cold_start_timeout_ns = int(cold_start_timeout_ns)
        self.cold_start_log = cold_start_log  # callable(str); None refuses confirmations
        self._cold = None
        self.mirror = RollingDeltaMirror(self.dims)
        self.counts = Counter()
        self._acquired_ns = None      # capture time behind the current mirror content
        self._request_sent_ns = None
        self._pending = {}            # delta_seq -> delta received while awaiting resync
        self._grid = None

    # -- inputs ------------------------------------------------------------
    def _request(self, now_ns, reason):
        if (self._request_sent_ns is not None and
                now_ns - self._request_sent_ns < self.resync_retry_ns):
            return None
        self._request_sent_ns = now_ns
        self.counts["resync_requested"] += 1
        return {"reason": reason, "epoch": self.mirror.epoch,
                "delta_seq": self.mirror.delta_seq, "stamp_ns": int(now_ns)}

    def _apply(self, delta, now_ns):
        ok, reason = self.mirror.apply(delta, now_ns=now_ns)
        if ok:
            self._acquired_ns = delta.acquired_ns
        return ok, reason

    def on_delta(self, delta, *, now_ns):
        if delta.window_dims != self.dims or not math.isclose(delta.voxel_m, self.voxel_m,
                                                             rel_tol=1e-12, abs_tol=0):
            self.counts["rejected:geometry_mismatch"] += 1
            return False, "geometry_mismatch", None
        if delta.clock_id != self.clock_id:
            self.counts["rejected:clock_mismatch"] += 1
            return False, "clock_mismatch", None
        if self.mirror.needs_resync and self.mirror.epoch == delta.epoch:
            # Keep it: the snapshot may be older than this delta.
            if delta.delta_seq > self.mirror.delta_seq:
                self._pending[delta.delta_seq] = delta
                while len(self._pending) > self.pending_limit:
                    self._pending.pop(min(self._pending))
            self.counts["rejected:awaiting_resync"] += 1
            return False, "awaiting_resync", self._request(now_ns, "awaiting_resync")
        ok, reason = self._apply(delta, now_ns)
        if ok:
            self._grid = None
            self.counts["applied"] += 1
            return True, reason, None
        self.counts["rejected:" + reason] += 1
        request = None
        if self.mirror.needs_resync:
            if self.mirror.epoch == delta.epoch and delta.delta_seq > self.mirror.delta_seq \
                    and reason == "sequence_gap":
                self._pending[delta.delta_seq] = delta
            request = self._request(now_ns, reason)
        return False, reason, request

    def on_snapshot(self, wire, *, now_ns):
        if not isinstance(wire, WireSnapshot):
            raise ValueError("WireSnapshot required")
        snap = wire.snapshot
        if (snap.states.shape != self.dims[::-1] or wire.clock_id != self.clock_id or
                not math.isclose(wire.voxel_m, self.voxel_m, rel_tol=1e-12, abs_tol=0)):
            self.counts["snapshot_rejected:mismatch"] += 1
            return False, "snapshot_mismatch", None
        if not self.mirror.needs_resync and self.mirror.epoch is not None:
            self.counts["snapshot_ignored:not_waiting"] += 1
            return False, "not_waiting", None
        if self.mirror.epoch is not None and (snap.epoch, snap.delta_seq) < \
                (self.mirror.epoch, self.mirror.delta_seq):
            self.counts["snapshot_rejected:older"] += 1
            return False, "snapshot_older", None
        ok, reason = self.mirror.resync(snap)
        if not ok:
            return False, reason, None
        self._acquired_ns = wire.acquired_ns
        self._request_sent_ns = None
        self._grid = None
        # Replay deltas that arrived while waiting, in order.
        for seq in sorted(self._pending):
            delta = self._pending[seq]
            if delta.epoch != snap.epoch or seq <= self.mirror.delta_seq:
                continue
            applied, why = self._apply(delta, now_ns)
            self.counts[("replayed" if applied else "replay_rejected:" + why)] += 1
            if not applied:
                break
        self._pending.clear()
        request = self._request(now_ns, "replay_gap") if self.mirror.needs_resync else None
        return True, "resynced", request

    # -- outputs -----------------------------------------------------------
    def status(self, *, now_ns, clock_id):
        """Freshness row for the navigation consumer (JSON-serialisable)."""
        row = {"epoch": self.mirror.epoch, "delta_seq": self.mirror.delta_seq,
               "clock_id": self.clock_id, "stamp_ns": int(now_ns), "age_ns": None,
               "max_map_age_ns": self.max_map_age_ns}
        if clock_id != self.clock_id:
            reason = "clock_mismatch"
        elif self.mirror.states is None or self._acquired_ns is None:
            reason = "no_map"
        elif self.mirror.needs_resync:
            reason = "awaiting_resync"
        else:
            age = int(now_ns) - self._acquired_ns
            row["age_ns"] = age
            if age < 0:
                reason = "future_map"
            elif age > self.max_map_age_ns:
                reason = "map_stale"
            else:
                reason = "map_current"
        row["cold_start"] = self.cold_start_state()
        if self.swept is not None:   # 10-28 swept-area clearing (on): what the last grid cleared
            row["swept"] = {"cleared_cells": getattr(self, "last_swept_cleared", 0),
                            "cleared_total": int(self.counts["swept_cleared_cells"]),
                            "evidence_cells_total": int(self.counts["swept_evidence_cells"])}
        row["fresh"] = reason == "map_current"
        row["reason"] = reason
        return row

    def tri2d(self):
        if self.mirror.states is None:
            return None
        layers = self.band.band_layers(self.mirror.window_base[2], self.dims[2], self.voxel_m)
        return project_columns(self.mirror.states, layers)

    # -- cold-start confirmation (second owner ruling, design doc 4.2) -------
    def confirm_cold_start(self, *, operator, operator_stamp, now_ns, body_pose):
        """One operator confirmation that the start area ahead is clear.

        Every attempt, accepted or refused, is appended to ``cold_start_log``
        first; if that write fails the attempt is refused. Accepted only once
        per bridge (the car's cold start), only with a known body pose and only
        when ``cold_start_width_m`` is configured.
        """
        row = {"event": "cold_start_confirm", "operator": operator,
               "operator_stamp": operator_stamp, "bridge_now_ns": int(now_ns),
               "body_pose": None if body_pose is None else [float(v) for v in body_pose],
               "width_m": self.cold_start_width_m, "epoch": self.mirror.epoch,
               "delta_seq": self.mirror.delta_seq}
        if self.cold_start_width_m is None:
            reason = "cold_start_confirmation_not_configured"
        elif not isinstance(operator, str) or not operator.strip():
            reason = "operator_required"
        elif not isinstance(operator_stamp, str) or not operator_stamp.strip():
            reason = "operator_time_required"
        elif body_pose is None or len(body_pose) != 3 or not all(
                math.isfinite(float(v)) for v in body_pose):
            reason = "body_pose_unknown"
        elif self._cold is not None:
            reason = "already_used_once"
        else:
            reason = "accepted"
        row["result"] = reason
        if not self._write_log(row):
            self.counts["cold_start_refused:log_failed"] += 1
            return False, "log_failed"
        if reason != "accepted":
            self.counts["cold_start_refused:" + reason] += 1
            return False, reason
        self._cold = {"pose": tuple(float(v) for v in body_pose), "t": int(now_ns),
                      "operator": operator, "active": True, "ended": None}
        self.counts["cold_start_accepted"] += 1
        return True, reason

    def _write_log(self, row):
        if self.cold_start_log is None:
            return False
        try:
            self.cold_start_log(json.dumps(row, sort_keys=True))
        except Exception:  # noqa: BLE001 - any failure to record refuses the action
            return False
        return True

    def cold_start_state(self, *, body_xy=None, now_ns=None):
        """'none' | 'active' | 'ended:<reason>'; ends the temporary FREE on move/timeout."""
        c = self._cold
        if c is None:
            return "none"
        if c["active"] and now_ns is not None:
            reason = None
            if int(now_ns) - c["t"] > self.cold_start_timeout_ns:
                reason = "timeout"
            elif body_xy is not None and math.hypot(body_xy[0] - c["pose"][0],
                                                    body_xy[1] - c["pose"][1]) \
                    > self.cold_start_width_m:
                reason = "moved"
            if reason is not None:
                c["active"], c["ended"] = False, reason
                self.counts["cold_start_ended:" + reason] += 1
                self._write_log({"event": "cold_start_end", "reason": reason,
                                 "operator": c["operator"], "bridge_now_ns": int(now_ns),
                                 "body_xy": list(body_xy) if body_xy else None})
        return "active" if c["active"] else "ended:" + str(c["ended"])

    def cold_start_cells(self):
        """[ly, lx] mask of the temporary ring, or None when not active."""
        c = self._cold
        if c is None or not c["active"] or self.mirror.window_base is None:
            return None
        x, y, yaw = c["pose"]
        return cold_ring_cells(self.mirror.window_base, self.dims, self.voxel_m, (x, y), yaw,
                               self.body_radius_m or 0.0, self.cold_start_width_m)

    def grid(self, body_xy=None, now_ns=None, body_yaw=None):
        """Grid for Nav2: strict map + body outline (+ front sector when on) (+ swept cells when on) + (after a
        confirmation) the ring."""
        base = self._grid_body(body_xy, body_yaw, now_ns)
        if base is None or self._cold is None:
            return base
        self.cold_start_state(body_xy=body_xy, now_ns=now_ns)
        mask = self.cold_start_cells()
        if mask is None:
            return base
        tri = np.full(base.data.shape, UNKNOWN, np.uint8)
        tri[base.data == GRID_FREE] = FREE
        tri[base.data == GRID_LETHAL] = OCCUPIED
        ring = occupancy_values(seed_body(tri.reshape(self.dims[1], self.dims[0]), mask))
        self.counts["cold_start_ring_published"] += 1
        return replace(base, data=ring.ravel())

    def _grid_body(self, body_xy=None, body_yaw=None, now_ns=None):
        """Whole OccupancyGrid content for the current mirror, or None.

        With ``body_radius_m`` set (owner ruling 2026-09-27, see the design
        doc section 4.1), UNKNOWN cells the robot outline at ``body_xy``
        (odom metres, the pose at publish time) overlaps, widened by
        ``body_margin_m`` (<= 5 cm), are sent as FREE. OCCUPIED cells are never changed and nothing is kept
        after the body has moved on (no swept-area clearing). Without a pose
        the strict grid is sent. With ``front_seed_m`` set (default off) and a
        heading ``body_yaw`` (rad, odom), the front sector is added to the mask
        (``front_sector_cells``); without a heading only the body is seeded.
        """
        strict = self.strict_grid()
        if strict is None or self.body_radius_m is None:
            return strict
        if body_xy is None:
            self.counts["body_pose_missing"] += 1
            return strict
        mask = body_cells(self.mirror.window_base, self.dims, self.voxel_m, body_xy,
                          self.body_radius_m + self.body_margin_m)
        if self.front_seed_m is not None:
            if body_yaw is None or not math.isfinite(float(body_yaw)):
                self.counts["front_seed_no_yaw"] += 1
            else:
                mask = mask | front_sector_cells(self.mirror.window_base, self.dims, self.voxel_m, body_xy,
                                                 float(body_yaw), self.front_seed_m,
                                                 math.radians(self.front_seed_deg))
                self.counts["front_seeded"] += 1
        tri = seed_body(self._tri, mask)
        if self.swept is not None:
            tri = self._apply_swept(tri, body_xy, now_ns)
        seeded = occupancy_values(tri).ravel()
        self.counts["body_seeded"] += 1
        return replace(strict, data=seeded)

    def _apply_swept(self, tri, body_xy, now_ns):
        """tri [ly, lx] -> with the swept cells FREE (see __init__): first the strong near evidence ends the
        clearing of its cells, then the current body circle is recorded, then the recent swept cells go FREE."""
        now = int(now_ns) if now_ns is not None else None
        if now is None:
            self.counts["swept_no_time"] += 1
            return tri
        base, dims, v = self.mirror.window_base, self.dims, self.voxel_m
        recent = self.swept.window_mask(base, dims, now, self.swept_clear_ns)
        if recent.any():
            tall = tall_occupied(self.mirror.states, base[2], v, self.band.ground_z_m + self.swept_evidence_min_h_m,
                                 self.band.band_layers(base[2], dims[2], v))
            strong = recent & tall & body_cells(base, dims, v, body_xy, self.swept_evidence_near_m)
            if strong.any():
                self.swept.forget(base, strong)
                self.counts["swept_evidence_cells"] += int(strong.sum())
        self.swept.mark(body_xy, self.body_radius_m, now)
        recent = self.swept.window_mask(base, dims, now, self.swept_clear_ns)
        out = np.array(tri, dtype=np.uint8, copy=True)
        n = int((recent & (out != FREE)).sum())
        out[recent] = FREE
        self.counts["swept_cleared_cells"] += n
        self.last_swept_cleared = n
        return out

    def strict_grid(self):
        """The grid by D-035 alone, without the body outline."""
        if self.mirror.states is None or self.mirror.needs_resync:
            return None
        if self._grid is None:
            tri = self.tri2d()
            base = self.mirror.window_base
            self._grid = GridFields(self.frame_id, self.voxel_m, self.dims[0], self.dims[1],
                                    (base[0] * self.voxel_m, base[1] * self.voxel_m),
                                    occupancy_values(tri).ravel().copy(), self.mirror.epoch,
                                    self.mirror.delta_seq, self._acquired_ns)
            self._tri = tri
        return self._grid


def tall_occupied(states, window_base_z, voxel_m, min_z_m, band_layers):
    """[ly, lx] bool: an OCCUPIED voxel of the obstacle band whose centre is at odom z >= min_z_m."""
    layers = [int(lz) for lz in band_layers if (int(window_base_z) + int(lz) + 0.5) * float(voxel_m) >= min_z_m]
    states = np.asarray(states)
    if not layers:
        return np.zeros(states.shape[1:], dtype=bool)
    return np.any(states[np.asarray(layers, dtype=np.int64)] == OCCUPIED, axis=0)


class SweptCells:
    """When the body circle last covered each odom cell (int ns), kept in 64 x 64 tiles (only where the car went).

    mark(): the body circle at xy, joined to the previous mark by a capsule when the step is <= SWEPT_MAX_STEP_M
    (marks come with the grid publications, a few cm apart; the capsule closes the gaps between them).
    """

    TILE = 64

    def __init__(self, voxel_m):
        self.v = float(voxel_m)
        self.tiles = {}
        self.prev_xy = None

    def _cells_capsule(self, a, b, r):
        v = self.v
        lo_x, hi_x = min(a[0], b[0]) - r, max(a[0], b[0]) + r
        lo_y, hi_y = min(a[1], b[1]) - r, max(a[1], b[1]) + r
        ix = np.arange(int(math.floor(lo_x / v)), int(math.floor(hi_x / v)) + 1)
        iy = np.arange(int(math.floor(lo_y / v)), int(math.floor(hi_y / v)) + 1)
        # closest point of each cell to the segment a-b: sample the segment finely enough (<= v / 2 apart)
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(seg / (v / 2.0))))
        keep = np.zeros((len(iy), len(ix)), dtype=bool)
        x0, y0 = ix * v, iy * v
        for t in np.linspace(0.0, 1.0, n + 1):
            px, py = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            dx = np.maximum(np.maximum(x0 - px, px - (x0 + v)), 0.0)[None, :]
            dy = np.maximum(np.maximum(y0 - py, py - (y0 + v)), 0.0)[:, None]
            keep |= dx * dx + dy * dy <= r * r
        return ix, iy, keep

    def mark(self, xy, radius_m, now_ns):
        xy = (float(xy[0]), float(xy[1]))
        a = self.prev_xy if (self.prev_xy is not None and
                             math.hypot(xy[0] - self.prev_xy[0], xy[1] - self.prev_xy[1]) <= SWEPT_MAX_STEP_M) else xy
        ix, iy, keep = self._cells_capsule(a, xy, float(radius_m))
        T = self.TILE
        yy, xx = np.nonzero(keep)
        gx, gy = ix[xx], iy[yy]
        for tx, ty in set(zip((gx // T).tolist(), (gy // T).tolist())):
            sel = (gx // T == tx) & (gy // T == ty)
            tile = self.tiles.get((tx, ty))
            if tile is None:
                tile = self.tiles[(tx, ty)] = np.full((T, T), -1, dtype=np.int64)
            tile[gy[sel] - ty * T, gx[sel] - tx * T] = int(now_ns)
        self.prev_xy = xy

    def _window_times(self, window_base, dims):
        mx, my = int(dims[0]), int(dims[1])
        bx, by = int(window_base[0]), int(window_base[1])
        out = np.full((my, mx), -1, dtype=np.int64)
        T = self.TILE
        for ty in range(by // T, (by + my - 1) // T + 1):
            for tx in range(bx // T, (bx + mx - 1) // T + 1):
                tile = self.tiles.get((tx, ty))
                if tile is None:
                    continue
                x0, x1 = max(bx, tx * T), min(bx + mx, (tx + 1) * T)
                y0, y1 = max(by, ty * T), min(by + my, (ty + 1) * T)
                out[y0 - by:y1 - by, x0 - bx:x1 - bx] = tile[y0 - ty * T:y1 - ty * T, x0 - tx * T:x1 - tx * T]
        return out

    def window_mask(self, window_base, dims, now_ns, hold_ns):
        """[ly, lx] bool: covered by the body within the last hold_ns."""
        t = self._window_times(window_base, dims)
        return (t >= 0) & (int(now_ns) - t <= int(hold_ns))

    def forget(self, window_base, mask):
        """the cells of the window mask are no longer swept."""
        bx, by = int(window_base[0]), int(window_base[1])
        T = self.TILE
        ly, lx = np.nonzero(np.asarray(mask, bool))
        gx, gy = lx + bx, ly + by
        for x, y in zip(gx.tolist(), gy.tolist()):
            tile = self.tiles.get((x // T, y // T))
            if tile is not None:
                tile[y - (y // T) * T, x - (x // T) * T] = -1


def body_cells(window_base, dims, voxel_m, center_xy, radius_m):
    """[ly, lx] mask of cells the body circle overlaps (closest cell point within radius).

    Overlap, not "centre inside": Nav2's footprint check rasterises the
    outline into every cell it touches, so a centre-only mask leaves outline
    cells UNKNOWN and forbids even the current pose (smoke try12).
    """
    mx, my = int(dims[0]), int(dims[1])
    v = float(voxel_m)
    x0 = (int(window_base[0]) + np.arange(mx)) * v
    y0 = (int(window_base[1]) + np.arange(my)) * v
    px, py = float(center_xy[0]), float(center_xy[1])
    dx = np.maximum(np.maximum(x0 - px, px - (x0 + v)), 0.0)[None, :]
    dy = np.maximum(np.maximum(y0 - py, py - (y0 + v)), 0.0)[:, None]
    return dx * dx + dy * dy <= float(radius_m) ** 2


def cold_ring_cells(window_base, dims, voxel_m, center_xy, yaw, radius_m, width_m):
    """[ly, lx] mask: cells overlapped by the circle radius + width but not by the
    body circle, whose centres lie ahead of the body (front half).

    Anchored at the confirmation pose (odom), it does not move with the car.
    """
    mx, my = int(dims[0]), int(dims[1])
    cx = (int(window_base[0]) + np.arange(mx) + 0.5) * float(voxel_m)
    cy = (int(window_base[1]) + np.arange(my) + 0.5) * float(voxel_m)
    dx = cx[None, :] - float(center_xy[0])
    dy = cy[:, None] - float(center_xy[1])
    ahead = dx * math.cos(yaw) + dy * math.sin(yaw) >= 0.0
    outer = body_cells(window_base, dims, voxel_m, center_xy, float(radius_m) + float(width_m))
    inner = body_cells(window_base, dims, voxel_m, center_xy, float(radius_m))
    return outer & ~inner & ahead


def front_sector_cells(window_base, dims, voxel_m, center_xy, yaw, radius_m, half_angle_rad):
    """[ly, lx] mask of the front sector: cells whose closest point lies within radius_m of the body
    centre (as body_cells) and whose CENTRE bearing from the body centre is within +-half_angle_rad of yaw.

    Same geometry as the 10-25 camera-cone harness (scratchpad carnav/repro/harness.py SEED_FWD_M).
    """
    mx, my = int(dims[0]), int(dims[1])
    cx = (int(window_base[0]) + np.arange(mx) + 0.5) * float(voxel_m) - float(center_xy[0])
    cy = (int(window_base[1]) + np.arange(my) + 0.5) * float(voxel_m) - float(center_xy[1])
    bearing = np.abs((np.arctan2(cy[:, None], cx[None, :]) - float(yaw) + np.pi) % (2 * np.pi) - np.pi)
    near = body_cells(window_base, dims, voxel_m, center_xy, radius_m)
    return near & (bearing <= float(half_angle_rad))


def seed_body(tri2d, mask):
    """UNKNOWN -> FREE inside the body mask; OCCUPIED and FREE unchanged."""
    out = np.array(tri2d, dtype=np.uint8, copy=True)
    out[np.asarray(mask, bool) & (out == UNKNOWN)] = FREE
    return out


def rolling_map_permit(row, *, received_ns, now_ns, max_map_age_ns=MAX_MAP_AGE_NS):
    """Consumer rule: move only while the map is younger than 200 ms.

    The map age is re-estimated on the consumer's own monotonic clock as the
    bridge's ``age_ns`` plus the time elapsed since the status row was
    received, so a dead bridge or a stalled link stops the car within the same
    200 ms as a stale map (no separate, tighter status timeout).
    """
    if row is None or received_ns is None:
        return False, "no_rolling_status"
    if not isinstance(row, dict) or row.get("fresh") is not True:
        return False, "rolling_" + str(row.get("reason") if isinstance(row, dict) else "bad")
    map_age = row.get("age_ns")
    elapsed = int(now_ns) - int(received_ns)
    if isinstance(map_age, bool) or not isinstance(map_age, int) or map_age < 0 or elapsed < 0:
        return False, "rolling_status_invalid"
    limit = min(int(max_map_age_ns), int(row.get("max_map_age_ns", max_map_age_ns)))
    if map_age + elapsed > limit:
        return False, "rolling_map_stale"
    return True, "rolling_map_current"


REQUIRED_COSTMAP = {"track_unknown_space": True, "trinary_costmap": True,
                    "lethal_cost_threshold": 100, "unknown_cost_value": 255,
                    "use_maximum": False, "rolling_window": True, "global_frame": "odom"}
CLEARING_PLUGINS = ("nav2_costmap_2d::ObstacleLayer", "nav2_costmap_2d::VoxelLayer")


def check_costmap_config(params, *, layer="rolling_layer", voxel_m=0.05,
                         topic="/fpga/rolling_map/grid", controller_plugin=None,
                         follow_path=None, planner=None):
    """Problems (list of strings) with a local costmap ros__parameters dict.

    ``follow_path`` (controller_server FollowPath params) and ``planner``
    (planner_server GridBased params), when given, are checked against the
    owner ruling of 2026-09-27: footprint-edge check kept, NavFn tolerance 0.
    """
    problems = []
    if follow_path is not None:
        controller_plugin = follow_path.get("plugin", controller_plugin)
        if "ObstacleFootprint" not in list(follow_path.get("critics", [])):
            problems.append("DWB critics must include ObstacleFootprint (body outline check)")
    if planner is not None:
        if planner.get("tolerance", 0.5) != 0.0:
            problems.append("NavFn tolerance must be 0 (else success next to an unreachable goal)")
        if planner.get("allow_unknown", True) is not False:
            problems.append("NavFn allow_unknown must be false")
    for key, want in REQUIRED_COSTMAP.items():
        # Nav2 defaults: track_unknown_space false, rolling_window false.
        default = {"track_unknown_space": False, "trinary_costmap": True,
                   "lethal_cost_threshold": 100, "unknown_cost_value": 255,
                   "use_maximum": False, "rolling_window": False,
                   "global_frame": "map"}[key]
        if params.get(key, default) != want:
            problems.append("%s must be %r" % (key, want))
    resolution = params.get("resolution", 0.1)
    if not isinstance(resolution, (int, float)) or not math.isclose(resolution, voxel_m,
                                                                   rel_tol=1e-9):
        problems.append("resolution must equal the voxel size")
    plugins = list(params.get("plugins", []))
    if not plugins or plugins[0] != layer:
        problems.append("%s must be the first costmap layer" % layer)
    spec = params.get(layer, {})
    if spec.get("plugin") != "nav2_costmap_2d::StaticLayer":
        problems.append("%s must be a StaticLayer" % layer)
    if spec.get("map_topic") != topic:
        problems.append("%s.map_topic must be %s" % (layer, topic))
    if params.get("footprint_padding", 0.01) != 0.0:
        # Nav2 pads the footprint by 1 cm by default; the bridge outline uses the
        # bare robot radius, so the padded outline would sit in UNKNOWN cells
        # and forbid even standing still (smoke try13). Padding would also grow the
        # body DWB checks; the car's seed margin (body_margin_m) only widens the
        # FREE seed and leaves the checked outline at the bare radius.
        problems.append("footprint_padding must be 0.0 to match the bridge body outline")
    if spec.get("footprint_clearing_enabled", False):
        problems.append("footprint_clearing_enabled must stay false (it moves with the body and "
                        "overwrites LETHAL)")
    if spec.get("subscribe_to_updates", False):
        problems.append("OccupancyGridUpdate cannot express a moving window origin")
    if spec.get("map_subscribe_transient_local", True) is not True:
        problems.append("map_subscribe_transient_local must match the transient_local publisher")
    for name in plugins[1:]:
        other = params.get(name, {}).get("plugin", "")
        if other in CLEARING_PLUGINS or other == "nav2_costmap_2d::StaticLayer":
            problems.append("layer %s (%s) would overwrite or ray-clear the rolling map"
                            % (name, other))
    if controller_plugin is not None and "RegulatedPurePursuit" in controller_plugin:
        problems.append("Humble RPP treats NO_INFORMATION as collision-free; use DWB")
    return problems
