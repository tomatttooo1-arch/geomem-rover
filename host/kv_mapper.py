"""kv_mapper core (NN-MAP1, 2026-11-04): the global grid on the KV260 PS, stitched from the PL navigation grids.

Design: runs/nationals-prelim-20261102/NONANO0/DESIGN.md section 2.3 (case M-B: RTAB-Map removed, the PS stitches
the 80 x 80 / 5 cm odom-frame grids the PL cost-map core already returns with every INTEGRATE into one global grid).
Package README: runs/nationals-prelim-20261102/NN-MAP1/README.md. No ROS import, numpy + standard library,
Python 3.8 compatible (the KV260 runs Ubuntu 22.04's Python 3.10).

Input, one ``GridFrame`` per fused frame: the navigation grid exactly as /fpga/rolling_map/grid carries it today
(int8 100 / 0 / -1, row = y, origin = the window corner in odom metres, 0.05 m; host/pl_grid_layer.PlGridLayer or
host/rolling_nav_layer.RollingLayerBridge output: band projection + body seed + swept clearing) plus the base pose
(odom x, y, yaw) at that frame. Optionally ``add_voxels`` with the occupied voxels of a rolling-map DUMP (3-D display).

Global grid: fixed square of ``size_m`` (20 m = car_inner MAPPING_BLANK_M, the blank start map Nav2 starts on),
centred on the start (map -> odom identity, the blank map's origin [-size/2, -size/2]). Per cell a log-odds value
with RTAB-Map's own global-grid update constants (GridGlobal/ProbHit 0.7, ProbMiss 0.4, clamping 0.1192 / 0.971,
OccupancyThr 0.5; rtabmap 0.23 Parameters.h): an occupied observation adds logit(0.7), a free one adds logit(0.4),
unknown cells of the frame change nothing; occupied when log-odds > logit(0.5) = 0, free when observed and <= 0,
unknown when never observed. Frame cells outside the global square are dropped and counted (never an error).

Integer rule "q13" (GMAP1, 2026-11-04; the product rule, runs/nationals-prelim-20261102/GMAP0/DESIGN.md section 3.3):
the same update in Q2.13 int16 -- hit +6941, miss -3322 (round(logit(p) x 8192)), clamp [-16384, 28762], occupied
when > 0, and -32768 marks a cell never observed (it replaces the separate "seen" array). One frame cell -> one
global cell: v = (c == -32768 ? 0 : c); c' = clip(v + d, -16384, 28762) (sum in 17+ bits). This is exactly what the
PL map-merge operation (gmap_op MERGE in the planner core) computes, so the PS reference and the PL agree by
construction. Cost against the float32 rule: the log-odds are quantised (H/|M| 2.08969 -> 2.08940), so a cell that
hovers near 0 can end in another tri-state (GMAP1 README: the adoption gate counted them on two real trips).
Rule "float" keeps the NN-MAP1 float32 arithmetic for comparison. ``coarse`` / ``image`` are the PS references of
the PL's 200 x 200 coarse map (2 x 2 most optimistic: free < unknown < occupied, then a 3-byte code table) and its
full-map scan image (rank -> byte, optional y flip).

Nodes: a frame becomes a node (its local grid kept, a trajectory point) when it is the first one, or at least
``node_period_s`` after the last node AND the base moved >= ``node_dist_m`` or turned >= ``node_yaw_deg``; a parked
car adds no nodes. Nodes feed the export poses, the search waypoints / nodes and -- the loop-closure hook -- the
rebuild.

Loop-closure hook (owner decision pending; NOT implemented here): ``apply_correction({node id: 3x3 SE(2)})`` takes a
per-node correction C (map <- odom of that node, the same definition as host/map_align: optimized = C x odometry),
rebuilds the global grid from the kept node grids moved by their C, and from then on maps every new frame with the
correction of the newest node (``map_to_odom``). The export then writes the corrected poses as the "optimized" poses
and the odometry into the database's Node table, so tools/car_run/mapping_ctl.py derives map -> odom and moves the
memory exactly as it does for RTAB-Map today (host/map_align.compute). With no correction (this version) the
optimized poses ARE the odometry: the alignment is identity up to float32 rounding (< 1e-6 m, see README).

Export (``export``), the contract tools/car_run/mapping_ctl.py relies on (rtabmap_car.sh's files, same names):
  OUT/export/<name>.pgm + <name>.yaml  map_server map (0 occupied, 254 free, 205 unknown -- rtabmap-export's values),
                                       cropped to the observed cells + 2 cells, origin on the global lattice
  OUT/export/<name>_poses.txt          format 11 "stamp x y z qx qy qz qw id" (host/map_align.read_opt_poses)
  DB  (DB_DIR/<name>.db)               SQLite: Node(id, stamp, pose = 12 float32 odometry, host/map_align
                                       read_db_odom), KvGrid (node local grids), KvMap (global log-odds), KvMeta
  OUT/sha256.txt                       sha256sum of the database + every export file (host/memory_store
                                       export_source_db: the map identity = sha256 of this database)
Shared map for G2 / hub (``write_shared``): one file (tmpfs), header + int8 tri-state, replaced atomically.
"""
import hashlib
import json
import math
import os
import sqlite3
import struct
import time
from dataclasses import dataclass

import numpy as np

RES_M = 0.05
SIZE_M = 20.0
OCC, FREE_V, UNK = 100, 0, -1                     # nav_msgs/OccupancyGrid values (frame and global)
PGM_OCC, PGM_FREE, PGM_UNK = 0, 254, 205          # rtabmap-export / map_server trinary


def logit(p):
    return math.log(p / (1.0 - p))


# RTAB-Map 0.23 GridGlobal defaults (Parameters.h): ProbHit 0.7, ProbMiss 0.4, ProbClampingMin 0.1192,
# ProbClampingMax 0.971, OccupancyThr 0.5
L_HIT, L_MISS = logit(0.7), logit(0.4)
L_MIN, L_MAX, L_THR = logit(0.1192), logit(0.971), logit(0.5)

# Q2.13 integer rule (GMAP0 DESIGN section 3.3); asserted against the float constants below
Q13 = 8192
Q_HIT, Q_MISS, Q_MIN, Q_MAX = 6941, -3322, -16384, 28762
Q_UNSEEN = -32768
assert (Q_HIT, Q_MISS, Q_MIN, Q_MAX) == tuple(int(round(v * Q13)) for v in (L_HIT, L_MISS, L_MIN, L_MAX))
DEFAULT_RULE = "q13"
RULES = ("q13", "float")
# tri-state ranks of the coarse map / scan (free 0 < unknown 1 < occupied 2) and the byte tables per rank
RANK_FREE, RANK_UNK, RANK_OCC = 0, 1, 2
COARSE_TABLE = (0, 0, 254)                         # free 0, unknown 0 (first trip: unknown is walkable), occupied 254
PGM_TABLE = (PGM_FREE, PGM_UNK, PGM_OCC)           # 254, 205, 0
TRI_TABLE = (FREE_V & 0xFF, UNK & 0xFF, OCC)       # int8 0, -1, 100 as bytes

SHARED_MAGIC = b"FPGAKVMP1"
SHARED_HEAD = struct.Struct("<8sIIIIIfdddd")        # magic, version, session, seq, width, height, res, ox, oy, stamp, pad
DB_VERSION = 1                                      # float rule; the q13 rule writes 2 (int16 KvMap.logodds)


@dataclass
class GridFrame:
    """One navigation grid (as /fpga/rolling_map/grid) and the base pose at its frame (odom)."""
    stamp: float                  # seconds (the frame's capture time; trajectory and node stamps)
    origin_x: float               # odom metres of the corner of cell (0, 0)
    origin_y: float
    data: np.ndarray              # int8 (height, width): 100 / 0 / -1, row = y
    pose: tuple                   # (x, y, yaw) odom <- base_link at the frame
    resolution: float = RES_M


def se2(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def se2_xyyaw(m):
    return float(m[0, 2]), float(m[1, 2]), math.atan2(float(m[1, 0]), float(m[0, 0]))


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def pose12_of(x, y, yaw):
    """Node.pose blob of a planar pose: 3 x 4 row-major float32 (rtabmap Transform, host/map_align.read_db_odom)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return struct.pack("<12f", c, -s, 0.0, x, s, c, 0.0, y, 0.0, 0.0, 1.0, 0.0)


def sha256_path(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


class KvMapper(object):
    """Global grid + nodes + trajectory + display voxels. Single-threaded; the process shell serialises calls."""

    def __init__(self, size_m=SIZE_M, res_m=RES_M, node_period_s=1.0, node_dist_m=0.05, node_yaw_deg=5.0,
                 display_min_hits=2, clock=time.time, rule=None):
        rule = DEFAULT_RULE if rule is None else rule
        if rule not in RULES:
            raise ValueError("rule %r not in %r" % (rule, RULES))
        self.rule = rule
        n = int(round(size_m / res_m))
        if not 1 <= n <= 4000:
            raise ValueError("global grid of %d x %d cells refused (1..4000)" % (n, n))
        self.n, self.res = n, float(res_m)
        self.origin = (-n * res_m / 2.0, -n * res_m / 2.0)   # blank_map.write_blank's origin
        self.node_period_s, self.node_dist_m = float(node_period_s), float(node_dist_m)
        self.node_yaw = math.radians(node_yaw_deg)
        self.display_min_hits = int(display_min_hits)
        self.clock = clock
        self.session = 0
        self.correction_name = "identity"
        self._clear()

    # ------------------------------------------------------------------ state
    def _clear(self):
        self._clear_grid()
        self.nodes = []            # dicts: id, stamp, odom (x, y, yaw), grid (int8), origin (x, y)
        self.corr = {}             # node id -> 3x3 SE(2) map <- odom (absent: identity)
        self.voxels = {}           # (i, j, k) world voxel -> hits
        self.voxel_m = None
        self.frames = 0
        self.dropped_cells = 0
        self.last = None           # (stamp, x, y, yaw) odom of the last frame
        self.seq = 0               # bumps on every change of the global grid

    def _clear_grid(self):
        if self.rule == "q13":
            self.logodds = np.full((self.n, self.n), Q_UNSEEN, np.int16)  # [row = y, col = x]; -32768 = never seen
            self._seen = None
        else:
            self.logodds = np.zeros((self.n, self.n), np.float32)    # [row = y, col = x]
            self._seen = np.zeros((self.n, self.n), bool)

    @property
    def seen(self):
        """bool [row = y, col = x]: observed at least once (q13: the cell is not the -32768 sentinel)."""
        return (self.logodds != Q_UNSEEN) if self.rule == "q13" else self._seen

    def reset(self):
        """「重置地图」: everything this session built is discarded; session + 1. Returns the new session."""
        self._clear()
        self.session += 1
        self.correction_name = "identity"
        return self.session

    # ------------------------------------------------------------------ input
    def map_to_odom(self):
        """3x3 SE(2) map <- odom now: the newest node's correction (identity without loop closure)."""
        if self.nodes and self.nodes[-1]["id"] in self.corr:
            return self.corr[self.nodes[-1]["id"]]
        return np.eye(3)

    def integrate(self, frame):
        """One GridFrame. Returns True when it became a node. Bad frames raise ValueError (the caller counts)."""
        data = np.asarray(frame.data)
        if data.ndim != 2 or data.dtype != np.int8:
            raise ValueError("grid must be a 2-D int8 array")
        if abs(float(frame.resolution) - self.res) > 1e-9:
            raise ValueError("grid resolution %r != %r" % (frame.resolution, self.res))
        x, y, yaw = (float(v) for v in frame.pose)
        if not all(math.isfinite(v) for v in (x, y, yaw, frame.origin_x, frame.origin_y, frame.stamp)):
            raise ValueError("non-finite pose / origin / stamp")
        self._stamp_cells(data, float(frame.origin_x), float(frame.origin_y), self.map_to_odom())
        self.frames += 1
        node = self._maybe_node(frame, data, x, y, yaw)
        self.last = (float(frame.stamp), x, y, yaw)
        self.seq += 1
        return node

    def _cells(self, data, ox, oy, corr):
        """(global row, global col, value) of the frame's known cells under map <- odom ``corr``."""
        ly, lx = np.nonzero(data != UNK)
        vals = data[ly, lx]
        if np.array_equal(corr, np.eye(3)):
            fx, fy = (ox - self.origin[0]) / self.res, (oy - self.origin[1]) / self.res
            ix, iy = int(round(fx)), int(round(fy))
            if abs(fx - ix) < 1e-3 and abs(fy - iy) < 1e-3:       # same 5 cm lattice: integer offset
                return ly + iy, lx + ix, vals                      # injective: one global cell per frame cell
        px = ox + (lx + 0.5) * self.res                        # cell centres, odom
        py = oy + (ly + 0.5) * self.res
        mx = corr[0, 0] * px + corr[0, 1] * py + corr[0, 2]
        my = corr[1, 0] * px + corr[1, 1] * py + corr[1, 2]
        gx = np.floor((mx - self.origin[0]) / self.res).astype(np.int64)
        gy = np.floor((my - self.origin[1]) / self.res).astype(np.int64)
        return gy, gx, vals

    def lattice_offset(self, ox, oy):
        """(ix, iy) when the frame corner (odom ox, oy) lies on the global 5 cm lattice, else None -- the PL MERGE
        condition (with an identity map <- odom); the global cell of frame cell (lx, ly) is (lx + ix, ly + iy)."""
        fx, fy = (ox - self.origin[0]) / self.res, (oy - self.origin[1]) / self.res
        ix, iy = int(round(fx)), int(round(fy))
        if abs(fx - ix) < 1e-3 and abs(fy - iy) < 1e-3:
            return ix, iy
        return None

    def _stamp_cells(self, data, ox, oy, corr):
        gy, gx, vals = self._cells(data, ox, oy, corr)
        inside = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
        self.dropped_cells += int((~inside).sum())
        gy, gx, vals = gy[inside], gx[inside], vals[inside]
        if self.rule == "q13":
            self._stamp_q13(gy, gx, vals)
            return
        delta = np.where(vals == OCC, np.float32(L_HIT), np.float32(L_MISS)).astype(np.float32)
        # several frame cells can land in one global cell under a rotated correction: accumulate, then clamp
        np.add.at(self.logodds, (gy, gx), delta)
        np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)
        self._seen[gy, gx] = True

    def _stamp_q13(self, gy, gx, vals):
        """Q2.13: per touched cell v = (c == unseen ? 0 : c), c' = clip(v + sum of its deltas). The identity path
        touches each cell once (exactly the PL MERGE rule); a rotated correction may land several frame cells in one
        global cell: their deltas are summed first, then clamped once (as the float rule)."""
        flat = gy.astype(np.int64) * self.n + gx
        delta = np.where(vals == OCC, Q_HIT, Q_MISS).astype(np.int32)
        cells, inv = np.unique(flat, return_inverse=True)
        acc = np.zeros(cells.shape, np.int32)
        np.add.at(acc, inv.ravel(), delta)
        lo = self.logodds.ravel()                                   # view (C-contiguous)
        cur = lo[cells].astype(np.int32)
        cur[cur == Q_UNSEEN] = 0
        lo[cells] = np.clip(cur + acc, Q_MIN, Q_MAX).astype(np.int16)

    def _maybe_node(self, frame, data, x, y, yaw):
        if self.nodes:
            last = self.nodes[-1]
            lx, ly, lyaw = last["odom"]
            moved = math.hypot(x - lx, y - ly) >= self.node_dist_m
            turned = abs(math.atan2(math.sin(yaw - lyaw), math.cos(yaw - lyaw))) >= self.node_yaw
            if frame.stamp - last["stamp"] < self.node_period_s or not (moved or turned):
                return False
        nid = (self.nodes[-1]["id"] + 1) if self.nodes else 1
        self.nodes.append({"id": nid, "stamp": float(frame.stamp), "odom": (x, y, yaw), "grid": data.copy(),
                           "origin": (float(frame.origin_x), float(frame.origin_y))})
        if self.corr:                      # a corrected graph: new nodes inherit the newest correction
            prev = self.nodes[-2]["id"] if len(self.nodes) > 1 else None
            if prev in self.corr:
                self.corr[nid] = self.corr[prev]
        return True

    def add_voxels(self, keys, voxel_m):
        """3-D display: occupied voxels (N x 3 integer world voxel indices, odom) of a rolling-map DUMP."""
        keys = np.asarray(keys, dtype=np.int64).reshape(-1, 3)
        self.voxel_m = float(voxel_m)
        for k in map(tuple, keys.tolist()):
            self.voxels[k] = self.voxels.get(k, 0) + 1

    # ------------------------------------------------------------------ loop-closure hook (not used in this version)
    def apply_correction(self, corrections, name="external"):
        """Per-node map <- odom corrections {node id: 3x3}; nodes absent keep the nearest earlier node's (or
        identity). Rebuilds the global grid from the kept node grids. Returns the number of nodes rebuilt."""
        corr, cur = {}, np.eye(3)
        for nd in self.nodes:
            if nd["id"] in corrections:
                cur = np.asarray(corrections[nd["id"]], dtype=float)
            corr[nd["id"]] = cur
        self.corr = corr
        self.correction_name = name
        self._clear_grid()
        for nd in self.nodes:
            self._stamp_cells(nd["grid"], nd["origin"][0], nd["origin"][1], corr[nd["id"]])
        self.seq += 1
        return len(self.nodes)

    # ------------------------------------------------------------------ outputs
    def tri(self):
        """Global grid int8 [row = y, col = x]: 100 / 0 / -1."""
        out = np.full((self.n, self.n), UNK, np.int8)
        seen = self.seen
        thr = 0 if self.rule == "q13" else L_THR
        out[seen & (self.logodds <= thr)] = FREE_V
        out[seen & (self.logodds > thr)] = OCC
        return out

    def ranks(self):
        """uint8 [row = y, col = x]: free 0 < unknown 1 < occupied 2 (the PL's 2-bit rank)."""
        t = self.tri()
        out = np.full(t.shape, RANK_UNK, np.uint8)
        out[t == FREE_V] = RANK_FREE
        out[t == OCC] = RANK_OCC
        return out

    def coarse(self, table=COARSE_TABLE):
        """uint8 [n/2, n/2] (row = y): the planner's 200 x 200 global input, 2 x 2 most optimistic (the minimum
        rank of the four fine cells, so one walkable cell keeps the coarse cell walkable), then ``table[rank]``."""
        if self.n % 2:
            raise ValueError("coarse map needs an even grid")
        r = self.ranks().reshape(self.n // 2, 2, self.n // 2, 2).min(axis=(1, 3))
        return np.asarray(table, np.uint8)[r]

    def image(self, table=PGM_TABLE, flip_y=True):
        """uint8 [n, n]: ``table[rank]`` per fine cell; flip_y puts row 0 at the highest y (pgm / map_server)."""
        img = np.asarray(table, np.uint8)[self.ranks()]
        return img[::-1] if flip_y else img

    def trajectory(self):
        """[(stamp, x, y, yaw)] map frame (nodes, corrected)."""
        out = []
        for nd in self.nodes:
            m = self.corr.get(nd["id"], np.eye(3)) @ se2(*nd["odom"])
            out.append((nd["stamp"],) + se2_xyyaw(m))
        return out

    def search(self):
        """(search_waypoints, search_nodes) in mapping_ctl's status format (host/trajectory_nodes)."""
        from host.trajectory_nodes import NODES_SPACING_M, SEARCH_MIN_FROM_START_M, dense_nodes, search_waypoints
        nodes = self.trajectory()
        dense = dense_nodes(nodes) if nodes else []
        sn = ({"ok": True, "nodes": dense, "spacing_m": NODES_SPACING_M, "frame": "map"} if dense
              else {"ok": False, "nodes": [], "reason": "empty trajectory"})
        wps = search_waypoints(nodes) if nodes else []
        sw = ({"ok": True, "waypoints": wps, "frame": "map", "nodes": len(nodes), "source": "kv_mapper"} if wps
              else {"ok": False, "waypoints": [], "reason": "trajectory of %d node(s) has no point %.1f m from its "
                    "start" % (len(nodes), SEARCH_MIN_FROM_START_M)})
        return sw, sn

    def display_cloud(self):
        """(xyz float32 N x 3, rgb uint8 N x 3) map frame, ui/geomem live_mapping.pack_cloud's planar layout.
        From the DUMP voxels seen >= display_min_hits times; without any, the occupied global cells at 0.15 m."""
        if self.voxels and self.voxel_m:
            keys = np.array([k for k, h in self.voxels.items() if h >= self.display_min_hits], np.float64)
            keys = keys.reshape(-1, 3)
            xyz = (keys + 0.5) * self.voxel_m
        else:
            gy, gx = np.nonzero(self.tri() == OCC)
            xyz = np.column_stack([self.origin[0] + (gx + 0.5) * self.res, self.origin[1] + (gy + 0.5) * self.res,
                                   np.full(gx.shape, 0.15)])
        if len(xyz):
            c = self.map_to_odom()
            xy = xyz[:, :2] @ c[:2, :2].T + c[:2, 2]
            xyz = np.column_stack([xy, xyz[:, 2]])
        t = np.clip(xyz[:, 2] / 1.0, 0.0, 1.0) if len(xyz) else np.zeros(0)
        rgb = np.column_stack([255 * t, 80 + 100 * (1 - abs(2 * t - 1)), 255 * (1 - t)]).astype(np.uint8)
        return xyz.astype(np.float32), rgb

    def bbox(self, margin=2):
        """(r0, r1, c0, c1) of the observed cells + margin (exclusive ends), or None."""
        rows, cols = np.nonzero(self.seen)
        if not len(rows):
            return None
        return (int(max(0, rows.min() - margin)), int(min(self.n, rows.max() + 1 + margin)),
                int(max(0, cols.min() - margin)), int(min(self.n, cols.max() + 1 + margin)))

    def status(self):
        tri = self.tri()
        return {"session": self.session, "frames": self.frames, "nodes": len(self.nodes), "seq": self.seq,
                "known_cells": int((tri != UNK).sum()), "occupied_cells": int((tri == OCC).sum()),
                "dropped_cells": self.dropped_cells, "last": self.last, "correction": self.correction_name,
                "display_voxels": len(self.voxels), "size_cells": self.n, "res_m": self.res, "rule": self.rule}

    def write_shared(self, path, stamp=None):
        """The global grid for G2 / hub (tmpfs file, replaced atomically): SHARED_HEAD + int8 [row = y]."""
        tri = self.tri()
        head = SHARED_HEAD.pack(SHARED_MAGIC, 1, self.session, self.seq & 0xFFFFFFFF, self.n, self.n, self.res,
                                self.origin[0], self.origin[1], self.clock() if stamp is None else stamp, 0.0)
        with open(path + ".tmp", "wb") as f:
            f.write(head)
            f.write(tri.tobytes())
        os.replace(path + ".tmp", path)

    # ------------------------------------------------------------------ export
    def export(self, out, name, db):
        """Writes the mapping_ctl contract (module note). Returns a summary dict; raises OSError / sqlite3.Error."""
        exp = os.path.join(out, "export")
        os.makedirs(exp, exist_ok=True)
        os.makedirs(os.path.dirname(os.path.abspath(db)), exist_ok=True)
        tri = self.tri()
        box = self.bbox() or (0, self.n, 0, self.n)
        r0, r1, c0, c1 = box
        crop = tri[r0:r1, c0:c1]
        img = np.full(crop.shape, PGM_UNK, np.uint8)
        img[crop == FREE_V] = PGM_FREE
        img[crop == OCC] = PGM_OCC
        img = img[::-1]                                  # pgm row 0 = the highest y (map_server)
        pgm, yml = os.path.join(exp, name + ".pgm"), os.path.join(exp, name + ".yaml")
        poses = os.path.join(exp, name + "_poses.txt")
        with open(pgm + ".tmp", "wb") as f:
            f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
            f.write(img.tobytes())
        os.replace(pgm + ".tmp", pgm)
        ox, oy = float(self.origin[0] + int(c0) * self.res), float(self.origin[1] + int(r0) * self.res)  # never np repr
        with open(yml + ".tmp", "w") as f:
            f.write("image: %s\nresolution: %r\norigin: [%r, %r, 0.0]\nnegate: 0\noccupied_thresh: 0.65\n"
                    "free_thresh: 0.196\n" % (os.path.basename(pgm), float(self.res), round(ox, 6), round(oy, 6)))
        os.replace(yml + ".tmp", yml)
        with open(poses + ".tmp", "w") as f:
            f.write("#timestamp x y z qx qy qz qw id\n")
            for nd, (stamp, x, y, yaw) in zip(self.nodes, self.trajectory()):
                qx, qy, qz, qw = yaw_quat(yaw)
                f.write("%.6f %.9g %.9g 0 %.9g %.9g %.9g %.9g %d\n" % (stamp, x, y, qx, qy, qz, qw, nd["id"]))
        os.replace(poses + ".tmp", poses)
        self._write_db(db)
        table = os.path.join(out, "sha256.txt")
        rows = [(sha256_path(p), os.path.abspath(p)) for p in [db] + [os.path.join(exp, n) for n in sorted(os.listdir(exp))]]
        with open(table + ".tmp", "w") as f:
            f.write("".join("%s  %s\n" % r for r in rows))
            f.flush()
            os.fsync(f.fileno())
        os.replace(table + ".tmp", table)
        xyz, rgb = self.display_cloud()
        np.savez_compressed(os.path.join(out, "display_cloud.npz"), xyz=xyz, rgb=rgb)
        summary = dict(self.status(), yaml=os.path.abspath(yml), pgm=os.path.abspath(pgm), poses=os.path.abspath(poses),
                       db=os.path.abspath(db), db_sha256=rows[0][0], crop=[int(v) for v in box],
                       origin=[round(ox, 6), round(oy, 6)])
        with open(os.path.join(out, "kv_mapper_export.json"), "w") as f:
            json.dump(summary, f, indent=1, default=str)
        return summary

    def _write_db(self, db):
        tmp = db + ".tmp"
        if os.path.exists(tmp):
            os.remove(tmp)
        con = sqlite3.connect(tmp)
        try:
            con.execute("CREATE TABLE Node (id INTEGER PRIMARY KEY, stamp REAL, pose BLOB)")
            con.execute("CREATE TABLE KvGrid (id INTEGER PRIMARY KEY, origin_x REAL, origin_y REAL, width INTEGER, "
                        "height INTEGER, data BLOB)")
            con.execute("CREATE TABLE KvMap (origin_x REAL, origin_y REAL, res REAL, size INTEGER, logodds BLOB, "
                        "seen BLOB)")
            con.execute("CREATE TABLE KvMeta (key TEXT PRIMARY KEY, value TEXT)")
            for nd in self.nodes:
                con.execute("INSERT INTO Node VALUES (?, ?, ?)", (nd["id"], nd["stamp"], pose12_of(*nd["odom"])))
                g = nd["grid"]
                con.execute("INSERT INTO KvGrid VALUES (?, ?, ?, ?, ?, ?)",
                            (nd["id"], nd["origin"][0], nd["origin"][1], g.shape[1], g.shape[0], g.tobytes()))
            if self.rule == "q13":
                upd = {"rule": "q13", "q": Q13, "hit": Q_HIT, "miss": Q_MISS, "min": Q_MIN, "max": Q_MAX,
                       "unseen": Q_UNSEEN, "thr": 0, "logodds": "int16 little-endian, row = y"}
            else:
                upd = {"rule": "float", "l_hit": L_HIT, "l_miss": L_MISS, "l_min": L_MIN, "l_max": L_MAX,
                       "l_thr": L_THR}
            con.execute("INSERT INTO KvMap VALUES (?, ?, ?, ?, ?, ?)",
                        (self.origin[0], self.origin[1], self.res, self.n, self.logodds.tobytes(),
                         np.packbits(self.seen).tobytes()))
            meta = {"db_version": 2 if self.rule == "q13" else DB_VERSION, "producer": "host/kv_mapper.py",
                    "session": self.session, "frames": self.frames, "correction": self.correction_name,
                    "update": upd,
                    "node_rule": {"period_s": self.node_period_s, "dist_m": self.node_dist_m,
                                  "yaw_rad": self.node_yaw}}
            for k, v in meta.items():
                con.execute("INSERT INTO KvMeta VALUES (?, ?)", (k, json.dumps(v)))
            con.commit()
        finally:
            con.close()
        os.replace(tmp, db)


def read_shared(path):
    """(header dict, int8 grid) of a write_shared file."""
    with open(path, "rb") as f:
        raw = f.read()
    h = SHARED_HEAD.unpack_from(raw)
    if h[0] != SHARED_MAGIC:
        raise ValueError("not a kv_mapper shared map")
    w, hh = h[4], h[5]
    grid = np.frombuffer(raw, np.int8, count=w * hh, offset=SHARED_HEAD.size).reshape(hh, w)
    return {"version": h[1], "session": h[2], "seq": h[3], "width": w, "height": hh, "res": h[6],
            "origin": (h[7], h[8]), "stamp": h[9]}, grid


# ---------------------------------------------------------------------- frame transport (producer -> kv_mapper)
FRAME_MAGIC, VOXEL_MAGIC = b"KVMAPGF1", b"KVMAPVX1"
FRAME_HEAD = struct.Struct("<8sIddddddfHH")         # magic, seq, stamp, ox, oy, x, y, yaw, res, w, h
VOXEL_HEAD = struct.Struct("<8sIfI")                # magic, seq, voxel_m, n  (+ n x 3 int16)
MAX_VOXELS_PER_DGRAM = 30000                        # 180 KB: below the AF_UNIX datagram default (~208 KB)


def encode_frame(frame, seq=0):
    d = np.ascontiguousarray(frame.data, np.int8)
    return FRAME_HEAD.pack(FRAME_MAGIC, seq & 0xFFFFFFFF, frame.stamp, frame.origin_x, frame.origin_y,
                           frame.pose[0], frame.pose[1], frame.pose[2], frame.resolution,
                           d.shape[1], d.shape[0]) + d.tobytes()


def encode_voxels(keys, voxel_m, seq=0):
    k = np.asarray(keys, np.int16).reshape(-1, 3)[:MAX_VOXELS_PER_DGRAM]
    return VOXEL_HEAD.pack(VOXEL_MAGIC, seq & 0xFFFFFFFF, voxel_m, len(k)) + k.tobytes()


def decode(buf):
    """-> ("frame", GridFrame) | ("voxels", (keys, voxel_m)); ValueError on anything malformed."""
    if buf[:8] == FRAME_MAGIC:
        if len(buf) < FRAME_HEAD.size:
            raise ValueError("short frame")
        _m, _seq, stamp, ox, oy, x, y, yaw, res, w, h = FRAME_HEAD.unpack_from(buf)
        if len(buf) != FRAME_HEAD.size + w * h:
            raise ValueError("frame size %d != header %d x %d" % (len(buf) - FRAME_HEAD.size, w, h))
        data = np.frombuffer(buf, np.int8, offset=FRAME_HEAD.size).reshape(h, w)
        return "frame", GridFrame(stamp, ox, oy, data, (x, y, yaw), res)
    if buf[:8] == VOXEL_MAGIC:
        _m, _seq, vm, n = VOXEL_HEAD.unpack_from(buf)
        if len(buf) != VOXEL_HEAD.size + 6 * n:
            raise ValueError("voxel message size mismatch")
        return "voxels", (np.frombuffer(buf, np.int16, offset=VOXEL_HEAD.size).reshape(n, 3), vm)
    raise ValueError("unknown message magic %r" % bytes(buf[:8]))
