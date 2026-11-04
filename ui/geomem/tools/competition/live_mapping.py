#!/usr/bin/env python3
"""Read-only RTAB-Map display feed. Does not open cameras or publish motion commands.

Run next to rtabmap_util/map_assembler (live_mapping.yaml). The assembler regenerates
colored 3D grids from RTAB-Map RGB-D nodes and uses its optimized pose graph.
This process only subscribes to its complete cloud and the original map graph.
"""
import argparse
import collections
import hashlib
import json
import os
import re
import sys
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np


def pack_cloud(msg):
    """PointCloud2 -> planar float32 XYZ + uint8 RGB, honoring row padding/endian."""
    if msg.header.frame_id != "map":
        raise ValueError("expected RTAB-Map map frame")
    fields = {f.name: f for f in msg.fields}
    color = fields.get("rgb") or fields.get("rgba")
    if color is None or color.datatype not in (6, 7) or color.count != 1:
        raise ValueError("missing packed RGB")
    names, formats, offsets = [], [], []
    endian = ">" if msg.is_bigendian else "<"
    for name in ("x", "y", "z"):
        f = fields.get(name)
        if f is None or f.datatype != 7 or f.count != 1:
            raise ValueError("expected float32 XYZ")
        names.append(name); formats.append(endian + "f4"); offsets.append(f.offset)
    names.append("rgb"); formats.append(endian + "u4"); offsets.append(color.offset)
    if msg.width * msg.height > 3000000 or msg.row_step < msg.width * msg.point_step:
        raise ValueError("cloud size exceeds display limit")
    if len(msg.data) != msg.row_step * msg.height or any(o + 4 > msg.point_step for o in offsets):
        raise ValueError("invalid PointCloud2 layout")
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=msg.point_step))
    raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step)).reshape(-1)
    xyz = np.column_stack([raw[n] for n in ("x", "y", "z")]).astype("<f4")
    valid = np.isfinite(xyz).all(axis=1)
    xyz, rgb = xyz[valid], raw["rgb"][valid]
    colors = np.column_stack([(rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255]).astype("u1")
    body = xyz.tobytes() + colors.tobytes()
    return body, {"count": len(xyz), "sha256": hashlib.sha256(body).hexdigest(),
                  "bounds": [xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()] if len(xyz) else None}


Q5Z_SCALE = 0.005         # metres per step; uint16 per axis -> 327 m span


def pack_q5z(body, count):
    """f32 XYZ + u8 RGB (pack_cloud's body) -> compact display cloud (10-21).

    Positions are quantised to 5 mm (the display grid is 1 cm, so this is below what is drawn),
    stored as uint16 offsets from an integer origin, planar per axis, then RGB planar; the whole
    thing is zlib-compressed (browser: DecompressionStream('deflate')). ~15 B/point -> ~3-5 B/point.
    Returns (header fields, compressed bytes)."""
    xyz = np.frombuffer(body, dtype="<f4", count=3 * count).reshape(count, 3)
    rgb = np.frombuffer(body, dtype="u1", offset=12 * count, count=3 * count).reshape(count, 3)
    q = np.round(xyz.astype(np.float64) / Q5Z_SCALE).astype(np.int64)
    origin = q.min(axis=0) if count else np.zeros(3, dtype=np.int64)
    d = q - origin
    if count and int(d.max()) > 65535:
        raise ValueError("cloud span over %.0f m" % (65535 * Q5Z_SCALE))
    raw = b"".join([d[:, i].astype("<u2").tobytes() for i in range(3)] + [rgb[:, i].tobytes() for i in range(3)])
    return {"encoding": "q5z", "scale": Q5Z_SCALE, "origin": [int(v) for v in origin],
            "rawBytes": len(raw)}, zlib.compress(raw, 6)


# ---------------------------------------------------------------------------- voxel deltas (10-22)
# map_assembler regenerates the whole cloud every ~2.4 s (voxels jitter, loop closures shift everything), so the
# full cloud was pushed every time (1.4 MB at 3 cm) and blocked the operator link. The feed now keeps a voxel
# state at the display cell and serves adds/removes since a client's (epoch, seq). Pure numpy, no ROS: unit-tested
# on the host (tests/test_mapping_delta.py) and mirrored by the page's VoxelStore (operator/src/voxel-store.js).
#
# Key: absolute voxel index a = floor(xyz / cell); per epoch a fixed integer origin o; relative r = a - o in
# [0, 65536) per axis; composite key = rx | ry << 16 | rz << 32 (int64, also an exact JS number). A voxel is drawn
# at (o + r + 0.5) * cell. Colour = first point of the voxel in the assembler's order; a surviving voxel keeps
# its old colour unless a channel moved by more than color_tol (display jitter is not re-sent).

U16 = 65536
VDZ = "vdz1"


def _fmix32(h):
    h = h ^ (h >> np.uint32(16))
    h = h * np.uint32(0x85EBCA6B)
    h = h ^ (h >> np.uint32(13))
    h = h * np.uint32(0xC2B2AE35)
    return h ^ (h >> np.uint32(16))


def key_hash(keys):
    """XOR over voxels of fmix32(imul(rx ^ ry << 16, 0x9E3779B1) ^ imul(rz, 0x85EBCA77)) -- order-free set
    fingerprint, updated incrementally by the page (voxel-store.js keyHash)."""
    keys = np.asarray(keys, dtype=np.int64)
    if not len(keys):
        return 0
    lo = (keys & 0xFFFFFFFF).astype(np.uint32)          # rx | ry << 16
    rz = ((keys >> 32) & 0xFFFF).astype(np.uint32)
    with np.errstate(over="ignore"):
        h = _fmix32((lo * np.uint32(0x9E3779B1)) ^ (rz * np.uint32(0x85EBCA77)))
    return int(np.bitwise_xor.reduce(h))


def split_keys(keys):
    keys = np.asarray(keys, dtype=np.int64)
    return keys & 0xFFFF, (keys >> 16) & 0xFFFF, (keys >> 32) & 0xFFFF


def _delta16(v):
    """Ascending-key plane -> successive differences mod 2**16 (page: running sum mod 2**16)."""
    v = np.asarray(v, dtype=np.int64)
    d = np.empty_like(v)
    if len(v):
        d[0] = v[0]
        d[1:] = v[1:] - v[:-1]
    return (d & 0xFFFF).astype("<u2")


def encode_vdz(add_keys, add_rgb, rm_keys, level=6):
    """vdz1 = zlib( adds x|y|z planes (delta u16) | removes x|y|z planes (delta u16) | adds r|g|b planes ).
    Keys must be ascending (they are: np.unique order). add_rgb is packed 0xRRGGBB uint32."""
    ax, ay, az = split_keys(add_keys)
    rx, ry, rz = split_keys(rm_keys)
    parts = [_delta16(p).tobytes() for p in (ax, ay, az, rx, ry, rz)]
    rgb = np.asarray(add_rgb, dtype=np.uint32)
    parts += [((rgb >> s) & 255).astype("u1").tobytes() for s in (16, 8, 0)]
    raw = b"".join(parts)
    return len(raw), zlib.compress(raw, level)


def decode_vdz(raw, adds, removes):
    """Reference decoder (tests; the page has its own in voxel-store.js). -> (add_keys, add_rgb, rm_keys)."""
    n, m = adds, removes
    if len(raw) != 9 * n + 6 * m:
        raise ValueError("vdz size")
    u = np.frombuffer(raw, dtype="<u2", count=3 * (n + m)).astype(np.int64)

    def planes(off, k):
        return [np.cumsum(u[off + i * k: off + (i + 1) * k]) & 0xFFFF for i in range(3)]
    ax, ay, az = planes(0, n)
    rx, ry, rz = planes(3 * n, m)
    c = np.frombuffer(raw, dtype="u1", offset=6 * (n + m), count=3 * n).reshape(3, n).astype(np.uint32)
    return ax | ay << 16 | az << 32, c[0] << 16 | c[1] << 8 | c[2], rx | ry << 16 | rz << 32


class VoxelFeed(object):
    """Voxel state + ring of per-revision deltas. Not thread-safe: the caller holds its lock around update()
    and delta_arrays(); encoding (zlib) can run outside it on the returned arrays."""

    def __init__(self, cell, ring=32, change_limit=0.3, color_tol=24, span=U16, epoch0=None):
        if not 0.001 <= cell <= 0.5:
            raise ValueError("display cell out of range")
        self.cell, self.ring_max, self.change_limit = float(cell), int(ring), float(change_limit)
        self.color_tol, self.span = int(color_tol), int(span)
        # a restarted feed must not continue a page's old epoch numbers
        self.epoch = int(time.time() * 1000) % (1 << 31) if epoch0 is None else int(epoch0)
        self.started = False
        self.seq = 0
        self.epoch_seq = 0                     # seq at which this epoch's full state was created
        self.origin = np.zeros(3, dtype=np.int64)
        self.keys = np.zeros(0, dtype=np.int64)
        self.rgb = np.zeros(0, dtype=np.uint32)
        self.hash = 0
        self.ring = collections.deque()        # (seq, add_keys, add_rgb, rm_keys) with base seq - 1
        self.last = {}

    def _new_epoch(self, a):
        lo, hi = a.min(axis=0), a.max(axis=0)
        if (hi - lo >= self.span).any():
            raise ValueError("cloud span over %.0f m" % (self.span * self.cell))
        self.origin = (lo + hi) // 2 - self.span // 2
        if self.started:
            self.epoch = (self.epoch + 1) % (1 << 31)
        self.started = True
        self.ring.clear()

    def _keys(self, a):
        r = a - self.origin
        return r[:, 0] | r[:, 1] << 16 | r[:, 2] << 32

    def update(self, xyz, rgb):
        """One assembler cloud (N x 3 float, N x 3 uint8). Returns an info dict; mode is 'same' (nothing
        changed: seq unchanged), 'delta' or 'epoch' (new epoch: clients must take a full)."""
        xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        rgb = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)
        ok = np.isfinite(xyz).all(axis=1)
        xyz, rgb = xyz[ok], rgb[ok]
        if not len(xyz):
            raise ValueError("empty cloud")
        a = np.floor(xyz / self.cell).astype(np.int64)
        reason = None
        if not self.started:
            reason = "start"
        else:
            r = a - self.origin
            if (r < 0).any() or (r >= self.span).any():
                reason = "out_of_range"
        if reason:
            self._new_epoch(a)
        keys, first = np.unique(self._keys(a), return_index=True)
        packed = (rgb[first, 0].astype(np.uint32) << 16) | (rgb[first, 1].astype(np.uint32) << 8) | rgb[first, 2]
        if reason is None:
            old_k, old_c = self.keys, self.rgb
            pos = np.searchsorted(old_k, keys)
            pos_c = np.minimum(pos, max(len(old_k) - 1, 0))
            present = (pos < len(old_k)) & (old_k[pos_c] == keys) if len(old_k) else np.zeros(len(keys), bool)
            back = np.minimum(np.searchsorted(keys, old_k), len(keys) - 1)   # both sorted: no isin sort
            removed = keys[back] != old_k
            n_new, n_rm = int((~present).sum()), int(removed.sum())
            if max(n_new, n_rm) > self.change_limit * max(len(old_k), 1):
                reason = "large_change"
                self._new_epoch(a)
                keys, first = np.unique(self._keys(a), return_index=True)
                packed = ((rgb[first, 0].astype(np.uint32) << 16) | (rgb[first, 1].astype(np.uint32) << 8)
                          | rgb[first, 2])
            else:
                prev = old_c[pos_c]
                diff = np.zeros(len(keys), dtype=bool)
                if present.any():
                    d = np.maximum.reduce([np.abs(((packed >> s) & 255).astype(np.int16)
                                                  - ((prev >> s) & 255).astype(np.int16)) for s in (16, 8, 0)])
                    diff = present & (d > self.color_tol)
                keep = present & ~diff
                packed = np.where(keep, prev, packed).astype(np.uint32)
                up = ~present | diff
                add_k, add_c, rm_k = keys[up], packed[up], old_k[removed]
                self.keys, self.rgb = keys, packed
                if not len(add_k) and not len(rm_k):
                    self.last = dict(mode="same", adds=0, removes=0, recolors=0, count=len(keys))
                    return self.last
                self.seq += 1
                self.hash = key_hash(keys)
                self.ring.append((self.seq, add_k, add_c, rm_k))
                while len(self.ring) > self.ring_max:
                    self.ring.popleft()
                self.last = dict(mode="delta", adds=n_new, removes=n_rm, recolors=int(diff.sum()), count=len(keys))
                return self.last
        self.keys, self.rgb = keys, packed.astype(np.uint32)
        self.seq += 1
        self.epoch_seq = self.seq
        self.hash = key_hash(keys)
        self.last = dict(mode="epoch", reason=reason, count=len(keys))
        return self.last

    def delta_arrays(self, epoch=None, seq=None):
        """Changes taking a client from (epoch, seq) to the current state, merged over the ring, or the full
        state when that base is unknown (other epoch, too old, ahead, or None). -> (header, add_k, add_c, rm_k)."""
        head = {"encoding": VDZ, "epoch": self.epoch, "seq": self.seq, "cell": self.cell,
                "origin": [int(v) for v in self.origin], "count": int(len(self.keys)), "keyHash": self.hash}
        oldest = self.ring[0][0] - 1 if self.ring else self.seq
        base_ok = (epoch == self.epoch and seq is not None and self.epoch_seq <= seq <= self.seq
                   and seq >= oldest)
        if not base_ok:
            head.update(mode="full", baseSeq=None)
            return head, self.keys, self.rgb, np.zeros(0, dtype=np.int64)
        head.update(mode="delta", baseSeq=int(seq))
        steps = [r for r in self.ring if r[0] > seq]
        if not steps:
            z = np.zeros(0, dtype=np.int64)
            return head, z, np.zeros(0, dtype=np.uint32), z
        if len(steps) == 1:
            _, ak, ac, rk = steps[0]
            return head, ak, ac, rk
        # last event per key wins: add (upsert, its colour) or remove
        k = np.concatenate([np.concatenate((s[1], s[3])) for s in steps])
        c = np.concatenate([np.concatenate((s[2], np.zeros(len(s[3]), np.uint32))) for s in steps])
        op = np.concatenate([np.concatenate((np.ones(len(s[1]), bool), np.zeros(len(s[3]), bool))) for s in steps])
        u, last_rev = np.unique(k[::-1], return_index=True)
        idx = len(k) - 1 - last_rev
        add = op[idx]
        return head, u[add], c[idx][add], u[~add]


def encode_delta(head, add_k, add_c, rm_k, meta=None):
    """-> wire body: u32 L | JSON header (feed status + delta header) | vdz1 payload (empty when no change)."""
    head = dict(meta or {}, **head)
    head.update(adds=int(len(add_k)), removes=int(len(rm_k)))
    if len(add_k) or len(rm_k):
        raw_n, payload = encode_vdz(add_k, add_c, rm_k)
    else:
        raw_n, payload = 0, b""
    head["rawBytes"] = raw_n
    header = json.dumps(head, allow_nan=False, separators=(",", ":")).encode()
    return struct.pack("<I", len(header)) + header + payload


def delta_body(head, add_k, add_c, rm_k, meta, cache=None, cache_max=16):
    """/delta response: the vdz1 body is cached per (epoch, base, seq) (zlib once per revision for all pages);
    the feed status fields in the header (ages, graph) are refreshed per request."""
    meta = dict(meta)
    for k in ("voxel", "sha256", "bounds", "epoch", "seq", "voxels"):
        meta.pop(k, None)
    ck = (head["epoch"], head["baseSeq"], head["seq"])
    hit = cache.get(ck) if cache is not None else None
    if hit is None:
        hit = encode_delta(head, add_k, add_c, rm_k)
        if cache is not None:
            cache[ck] = hit
            while len(cache) > cache_max:
                cache.popitem(last=False)
    (L,) = struct.unpack_from("<I", hit, 0)
    h = json.loads(hit[4:4 + L].decode())
    h.update(meta)
    h.update(head)
    header = json.dumps(h, allow_nan=False, separators=(",", ":")).encode()
    return struct.pack("<I", len(header)) + header + hit[4 + L:]


# ---------------------------------------------------------------------------- 10-26「重置地图」
# The console's 重置地图 (host/map_reset.py) gives the runner a NEW runtime_boot_id (= the page's memory.scope) and
# makes mapping_ctl restart the RTAB-Map map (state resetting, then session + 1). A feed pinned to its start-up
# --scope would then be refused by the page for good, and its voxels are the old map: the feed follows both.
SCOPE_RE = re.compile(r"[0-9a-f]{16}")
STALE_GATE_S = 15.0        # clouds stamped before the clear are dropped this long after it (a stamp 0 never sticks)


class MappingFollow(object):
    """Pure state (no ROS): scope from /fpga/runtime_info, clears from /fpga/mapping/status, stale-cloud gate."""

    def __init__(self, scope, clock=time.time):
        self.scope, self.scope_source, self.clock = scope, "arg", clock
        self.state = self.session = None
        self.cleared_at = None
        self.cleared_in_reset = False
        self.resets = 0
        self.dropped = 0

    def on_runtime_info(self, text):
        """/fpga/runtime_info JSON -> True when the scope changed."""
        try:
            doc = json.loads(text)
        except ValueError:
            return False
        scope = doc.get("runtime_boot_id") if isinstance(doc, dict) else None
        if not isinstance(scope, str) or not SCOPE_RE.fullmatch(scope):
            return False
        changed = scope != self.scope
        self.scope, self.scope_source = scope, "topic"
        return changed

    def on_mapping_status(self, text):
        """/fpga/mapping/status JSON -> True when the voxels must be cleared now: entering resetting, or a new
        session whose resetting was not seen (late joiner / missed sample)."""
        try:
            doc = json.loads(text)
        except ValueError:
            return False
        if not isinstance(doc, dict):
            return False
        state, session = doc.get("state"), doc.get("session")
        session = session if isinstance(session, int) and not isinstance(session, bool) else None
        clear = False
        if state == "resetting" and self.state != "resetting":
            clear, self.cleared_in_reset = True, True
        elif session is not None and self.session is not None and session != self.session:
            clear = not self.cleared_in_reset
            self.cleared_in_reset = False
        self.state = state
        if session is not None:
            self.session = session
        if clear:
            self.cleared_at = self.clock()
            self.resets += 1
        return clear

    def accept_cloud(self, stamp_s):
        """False: an assembler cloud of the old map (during the reset, or stamped before the clear)."""
        if self.state == "resetting":
            self.dropped += 1
            return False
        c = self.cleared_at
        if (c is not None and isinstance(stamp_s, (int, float)) and 1e9 < stamp_s < c
                and self.clock() - c < STALE_GATE_S):
            self.dropped += 1
            return False
        return True

    def fields(self):
        return {"scope": self.scope, "scopeSource": self.scope_source, "mappingSession": self.session,
                "mappingState": self.state, "resets": self.resets, "resetDropped": self.dropped}


# ---------------------------------------------------------------------------- 10-30 graph gate (first level)
# A parked car: RTAB-Map adds no node, but the assembler's cloud used to come every second and this feed unpacked,
# hashed and diffed all of it (1.8 s on the board for 1 voxel + 9 recolours). Now a cloud is processed only when the
# map graph changed since the last processed cloud (/mapGraph through map_data_gate.GraphGate: the parked frame's
# temporary node does not count), else it waits as the pending cloud: processed as soon as a graph change with a
# stamp <= its own arrives, or FALLBACK_S after the last processed cloud. The newest cloud is therefore always shown
# within FALLBACK_S. No graph message since start / reset = no evidence: every cloud is processed. The second level
# (map_data_gate.py beside the assembler) already stops most parked clouds at the source; this level also covers an
# assembler still fed by the ungated /mapData.
GRAPH_FALLBACK_S = 10.0


def gate_module():
    """map_data_gate.py next to this file (start_live_mapping.sh copies both into the same directory)."""
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    import map_data_gate
    return map_data_gate


class CloudGate(object):
    """Pure state (no ROS). Stamps in seconds (cloud stamp = the /mapData stamp the assembler copies = the /mapGraph
    stamp of the same RTAB-Map update); now = a monotonic clock."""

    def __init__(self, fallback_s=GRAPH_FALLBACK_S, graph_gate=None):
        self.fallback_s = float(fallback_s)
        if graph_gate is None:
            graph_gate = gate_module().GraphGate(fallback_s=float("inf"))
        self.graph = graph_gate
        self.reset()
        self.processed = self.skipped = 0

    def reset(self):
        self.graph.reset("reset")
        self.graph_seen = False
        self.change_stamp = None          # stamp of the latest graph change
        self.done_stamp = None            # stamp of the last processed cloud
        self.done_at = None
        self.pending = None               # stamp of the waiting cloud (the caller keeps its bytes)

    def on_graph(self, ids, poses, stamp, now):
        """-> True when this graph is a change (the caller then calls poll())."""
        if not len(ids):
            changed = True
        else:
            changed, _, _ = self.graph.decide(ids, poses, now, node_ids=[int(max(ids))])
        self.graph_seen = True
        if changed and (self.change_stamp is None or stamp > self.change_stamp):
            self.change_stamp = stamp
        return changed

    def _due(self, stamp, now):
        if not self.graph_seen or self.done_at is None or now - self.done_at >= self.fallback_s or now < self.done_at:
            return True
        c = self.change_stamp
        return c is not None and (self.done_stamp is None or c > self.done_stamp) and stamp >= c

    def on_cloud(self, stamp, now):
        """-> True: process this cloud now; False: it is the pending cloud (replacing an older one)."""
        if self._due(stamp, now):
            return True
        if self.pending is not None:
            self.skipped += 1             # the older pending cloud is never processed
        self.pending = stamp
        return False

    def poll(self, now):
        """-> True: process the pending cloud now."""
        return self.pending is not None and self._due(self.pending, now)

    def done(self, stamp, now):
        self.processed += 1
        self.done_stamp, self.done_at = stamp, now
        if self.pending is not None and self.pending <= stamp:
            self.pending = None

    def fields(self):
        return {"processed": self.processed, "skipped": self.skipped, "graphSeen": self.graph_seen,
                "pending": self.pending is not None, "graphGate": dict(self.graph.counts)}


def read_cell_size(yaml_path, default=0.03):
    """Grid/CellSize from live_mapping.yaml (no yaml module needed); the display cell follows the assembler."""
    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            m = re.search(r"Grid/CellSize:\s*\"?([0-9.]+)", f.read())
        return float(m.group(1)) if m else default
    except (OSError, ValueError):
        return default


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from rtabmap_msgs.msg import MapGraph

    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--scope", required=True, help="Current runtime_info.json runtime_boot_id")
    ap.add_argument("--port", type=int, default=47802)
    ap.add_argument("--cell", type=float, default=None,
                    help="display voxel (m); default Grid/CellSize of live_mapping.yaml next to this file")
    ap.add_argument("--graph-fallback-s", type=float, default=GRAPH_FALLBACK_S,
                    help="10-30: a cloud waiting for a graph change is processed at the latest this long after the "
                         "last processed one")
    args = ap.parse_args()
    cell = args.cell or read_cell_size(os.path.join(os.path.dirname(os.path.abspath(__file__)), "live_mapping.yaml"))
    voxels = VoxelFeed(cell)
    follow = MappingFollow(args.scope)
    gate = CloudGate(args.graph_fallback_s)
    cdr_stamp = gate_module().cdr_stamp
    pending = {"raw": None}                    # bytes of the cloud waiting for a graph change (CloudGate.pending)
    glock = threading.Lock()                   # gate + pending
    enc_cache = collections.OrderedDict()     # (epoch, base seq or None, seq) -> encoded /delta body
    rclpy.init()
    node = Node("operator_mapping_feed")
    lock = threading.Lock()
    vlock = threading.Lock()
    elock = threading.Lock()
    state = dict(format="geomem-live-mapping", version=1, source="rtabmap",
                 session=args.session, scope=args.scope, frame="map", cellM=cell, count=0,
                 topic="/operator_map/cloud_map", revision=0, graphNodes=0)
    data = b""
    graph_at, cloud_at = 0., 0.
    compact = {"entry": None}     # q5z cache (sha, header fields, bytes): one encode per cloud
    state.update(follow.fields())

    def clear_voxels():
        """10-26 重置地图: the old map's voxels go; a new epoch so every page takes a full of the new map (the
        bridge pushes nothing while there are no voxels: /delta answers 503 until the first new cloud)."""
        nonlocal voxels, data
        with glock:
            gate.reset()                       # the new map's first cloud is processed whatever the graph says
            pending["raw"] = None
        with vlock:
            voxels = VoxelFeed(cell, epoch0=(voxels.epoch + 1) % (1 << 31))
        with elock:
            enc_cache.clear()
        compact["entry"] = None
        with lock:
            data = b""
            for k in ("sha256", "bounds", "voxel", "stamp", "error"):
                state.pop(k, None)
            state.update(count=0, epoch=voxels.epoch, seq=0, voxels=0, revision=state["revision"] + 1,
                         **follow.fields())

    def runtime_info(msg):
        with lock:
            changed = follow.on_runtime_info(msg.data)
            state.update(follow.fields())
        if changed:
            node.get_logger().info("scope -> %s (runtime_info)" % follow.scope)

    def mapping_status(msg):
        with lock:
            clear = follow.on_mapping_status(msg.data)
            state.update(follow.fields())
        if clear:
            clear_voxels()
            node.get_logger().info("map reset: voxels cleared (mapping session %s, state %s)"
                                   % (follow.session, follow.state))

    def graph(msg):
        nonlocal graph_at
        t = msg.map_to_odom
        with lock:
            graph_at = time.monotonic()
            state["graphNodes"] = len(msg.poses_id)
            state["mapToOdom"] = dict(position=[t.translation.x, t.translation.y, t.translation.z],
                                      quaternion=[t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        ids = np.asarray(msg.poses_id, dtype=np.int64)
        poses = np.array([[p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y,
                           p.orientation.z, p.orientation.w] for p in msg.poses], dtype=np.float64).reshape(-1, 7)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        with glock:
            gate.on_graph(ids, poses, stamp, time.monotonic())
        flush_pending()

    def flush_pending():
        """the pending cloud, when a graph change or the fallback made it due"""
        with glock:
            raw = pending["raw"] if gate.poll(time.monotonic()) else None
            pending["raw"] = None if raw is not None else pending["raw"]
        if raw is not None:
            process(raw)

    def cloud(raw):
        """10-30: raw CDR bytes -- unpacked only when CloudGate lets this cloud through"""
        try:
            stamp = cdr_stamp(raw)
        except ValueError:
            stamp = None
        with glock:
            now_due = stamp is None or gate.on_cloud(stamp, time.monotonic())
            if not now_due:
                pending["raw"] = raw
        if now_due:
            process(raw)

    def process(raw):
        nonlocal data, cloud_at
        try:
            msg = deserialize_message(raw, PointCloud2)
        except Exception as exc:            # noqa: BLE001 -- a broken message must not stop the feed
            with lock:
                state["error"] = "cloud: %s" % exc
            return
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        with glock:
            gate.done(stamp_s, time.monotonic())
        with lock:
            ok = follow.accept_cloud(stamp_s)
            state["resetDropped"] = follow.dropped
            gen = follow.resets             # a clear while this cloud is diffed (~0.5 s on Nano) drops it
            state["gate"] = gate.fields()
        if not ok:
            return                          # the old map's cloud (reset running / stamped before the clear)
        try:
            body, meta = pack_cloud(msg)
            n = meta["count"]
            xyz = np.frombuffer(body, dtype="<f4", count=3 * n).reshape(n, 3)
            rgb = np.frombuffer(body, dtype="u1", offset=12 * n, count=3 * n).reshape(n, 3)
            t0 = time.monotonic()
            with vlock:                     # own lock: /status never waits for the voxel diff (~0.5 s on Nano)
                if follow.resets != gen:
                    return
                info = voxels.update(xyz, rgb) if n else {"mode": "empty"}
            with lock:
                if follow.resets != gen:
                    return
                state["voxel"] = dict(info, epoch=voxels.epoch, seq=voxels.seq, ms=round((time.monotonic() - t0) * 1000))
                state["epoch"], state["seq"], state["voxels"] = voxels.epoch, voxels.seq, int(len(voxels.keys))
            with lock:
                cloud_at = time.monotonic()
                data = body
                state.update(meta)
                state["revision"] += 1
                state["stamp"] = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                state.pop("error", None)
        except (ValueError, TypeError) as exc:
            with lock:
                state["error"] = str(exc)
            node.get_logger().error(str(exc))

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PointCloud2, "/operator_map/cloud_map", cloud, qos, raw=True)
    node.create_subscription(MapGraph, "/mapGraph", graph, 1)
    node.create_timer(1.0, flush_pending)     # the fallback: the newest cloud is shown within --graph-fallback-s
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(String, "/fpga/runtime_info", runtime_info, latched)
    node.create_subscription(String, "/fpga/mapping/status", mapping_status, latched)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _delta(self, query):
            """/delta[?epoch=E&seq=S] -> changes since (E, S), or the full voxel state (mode full)."""
            q = dict(kv.partition("=")[::2] for kv in query.split("&") if kv)
            try:
                epoch = int(q["epoch"]) if "epoch" in q else None
                seq = int(q["seq"]) if "seq" in q else None
            except ValueError:
                self.send_error(400); return
            with vlock:
                if not voxels.started or not len(voxels.keys):
                    self.send_error(503, "no voxels yet"); return
                head, ak, ac, rk = voxels.delta_arrays(epoch, seq)
            with lock:
                meta = dict(state, graphAgeMs=(time.monotonic() - graph_at) * 1000,
                            cloudAgeMs=(time.monotonic() - cloud_at) * 1000)
            with elock:                     # handler threads share the encode cache; one zlib per revision
                payload = delta_body(head, ak, ac, rk, meta, enc_cache)
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            route, _, query = self.path.partition("?")
            if self.path not in ("/status", "/cloud.bin", "/cloud.bin?format=q5z") and route != "/delta":
                self.send_error(404); return
            if self.headers.get("Host") != "127.0.0.1:%d" % args.port or self.headers.get("Origin"):
                self.send_error(403); return
            if route == "/delta":
                return self._delta(query)
            with lock:
                meta = dict(state, graphAgeMs=(time.monotonic() - graph_at) * 1000,
                            cloudAgeMs=(time.monotonic() - cloud_at) * 1000)
                body, sha, count = data, state.get("sha256"), state.get("count", 0)
            if self.path == "/cloud.bin?format=q5z":
                entry = compact["entry"]               # one tuple read: never mixes two clouds
                if entry is None or entry[0] != sha:
                    try:
                        entry = (sha,) + pack_q5z(body, count)
                    except ValueError as exc:
                        self.send_error(503, str(exc)); return
                    compact["entry"] = entry
                meta.update(entry[1])
                header = json.dumps(meta, allow_nan=False, separators=(",", ":")).encode()
                payload = struct.pack("<I", len(header)) + header + entry[2]
            else:
                header = json.dumps(meta, allow_nan=False, separators=(",", ":")).encode()
                payload = header if self.path == "/status" else struct.pack("<I", len(header)) + header + body
            self.send_response(200)
            self.send_header("Content-Type", "application/json" if self.path == "/status" else "application/octet-stream")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    finally:
        server.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
