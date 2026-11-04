#!/usr/bin/env python3
"""3D map display gate (10-30): forward RTAB-Map's /mapData to the display's map_assembler only when the map graph
changed. Read-only next to the car stack (start_live_mapping.sh starts it beside the assembler); it opens no camera
and publishes no motion.

Why: a parked car's RTAB-Map adds no node (RGBD/LinearUpdate 0.1 m / AngularUpdate 0.1 rad) but still processes one
frame per second and publishes /mapData; map_assembler then re-assembled and re-published the whole cloud every
second (~0.3 s, 777k points on the board) and live_mapping.py diffed all of it (1.8 s) although nothing was added
(runs/cpu-growth-20261029-results/RESULTS.md).

What a parked /mapData carries (rtabmap 0.23.7 source, Rtabmap::process + CoreWrapper::publishStats): the graph
(every node id + optimized pose) PLUS the frame just processed as a TEMPORARY node -- a new id each second, its pose
re-inserted into the published poses (lastSignatureLocalizedPose) although RTAB-Map deleted it ("Ignoring location
... displacement is too small"), and its sensor data. Its id is the largest in the message. The assembler appends
that temporary node's grid to the assembled cloud (its id is new) and re-voxelizes: the 1 voxel + 9 recolours per
second the board showed. Nothing else in such a message is new.

Rule (GraphGate.decide; any doubt forwards):
  forward when the gate has no reference (start, map reset, a new assembler subscriber, a parse failure, the mapping
  status in resetting), when FALLBACK_S passed since the last forward, when the message carries any node data other
  than its newest node, when a node other than the newest is not in the last forwarded graph (a node RTAB-Map kept),
  when a node of the last forwarded graph (other than its newest) is gone, when any pose moved by more than
  POSE_EPS_M / POSE_EPS_RAD (a loop closure or any re-optimization), and when the newest node is new and lies at or
  beyond TEMP_LIN_M / TEMP_ANG_RAD from the latest kept node -- half of RTAB-Map's own 0.1 m / 0.1 rad, so every
  node RTAB-Map adds for motion is forwarded the moment it appears -- unless it is within SAME_LIN_M / SAME_ANG_RAD
  of the newest node of the last forwarded message (a car parked 5-10 cm past its last node: its first parked
  frame is forwarded, the identical ones after it are not). Skipped: only a message whose only novelty is such a
  newest node (the parked frame).
  A node skipped that way but then kept by RTAB-Map (a loop closure on the parked frame) shows up in the next
  message as a kept node -> that message is forwarded and the skipped message's node data are injected into it
  (the assembler builds a node's grid only from data it received).
Cheap: the graph part of the message (ids, poses, node ids) is read straight from the CDR bytes
(parse_map_data_prefix); the message is forwarded as the same bytes, deserialized only to inject a node.
Pure parts (GraphGate, parse_map_data_prefix) have no ROS import: tests/test_map3d_gate.py.
"""
import collections
import json
import struct
import time

import numpy as np

FALLBACK_S = 10.0          # forward at least this often whatever the graph (owner: 10 s safety net)
POSE_EPS_M = 0.001         # a pose correction of 1 mm / 1 mrad already counts as a change
POSE_EPS_RAD = 0.001
TEMP_LIN_M = 0.05          # RTAB-Map RGBD/LinearUpdate default 0.1 m (rtabmap.yaml does not set it): half
TEMP_ANG_RAD = 0.05        # RGBD/AngularUpdate default 0.1 rad: half
SAME_LIN_M = 0.02          # a parked frame within 2 cm / 0.02 rad (1.1 deg) of the newest node already forwarded:
SAME_ANG_RAD = 0.02        # the same viewpoint the assembler already has (a car parked 5-10 cm past its last node)


# ---------------------------------------------------------------------------- CDR (XCDR1) prefix of MapData
def _align(pos, n, base=4):
    return pos + (-(pos - base)) % n


def _string(buf, pos, end):
    pos = _align(pos, 4)
    (n,) = struct.unpack_from(end + "I", buf, pos)
    return pos + 4 + n


def _header(buf, pos, end):
    pos = _align(pos, 4)
    sec, nsec = struct.unpack_from(end + "iI", buf, pos)
    return _string(buf, pos + 8, end), sec + nsec * 1e-9


def cdr_stamp(buf):
    """header.stamp (seconds) of any serialized message starting with std_msgs/Header (PointCloud2, MapData)."""
    if len(buf) < 12 or buf[0] != 0 or buf[1] not in (0, 1):
        raise ValueError("not a plain CDR message")
    end = "<" if buf[1] == 1 else ">"
    sec, nsec = struct.unpack_from(end + "iI", buf, 4)
    return sec + nsec * 1e-9


def parse_map_data_prefix(buf):
    """Serialized rtabmap_msgs/MapData -> (stamp, ids int64[n], poses float64[n, 7] (x y z qx qy qz qw),
    node_ids list) without touching the node payloads (images). Raises ValueError on anything unexpected."""
    try:
        if len(buf) < 8 or buf[0] != 0 or buf[1] not in (0, 1):
            raise ValueError("not a plain CDR message")
        end = "<" if buf[1] == 1 else ">"
        pos, stamp = _header(buf, 4, end)                  # MapData.header
        pos, _ = _header(buf, pos, end)                    # graph.header
        pos = _align(pos, 8) + 56                          # graph.map_to_odom (Transform: 7 float64)
        pos = _align(pos, 4)
        (n,) = struct.unpack_from(end + "I", buf, pos)
        pos += 4
        if n > 1000000:
            raise ValueError("graph size %d" % n)
        ids = np.frombuffer(buf, dtype=end + "i4", count=n, offset=pos).astype(np.int64)
        pos += 4 * n
        pos = _align(pos, 4)
        (m,) = struct.unpack_from(end + "I", buf, pos)
        pos += 4
        if m != n:
            raise ValueError("poses %d != ids %d" % (m, n))
        if n:
            pos = _align(pos, 8)
            poses = np.frombuffer(buf, dtype=end + "f8", count=7 * n, offset=pos).reshape(n, 7).astype(np.float64)
            pos += 56 * n
        else:
            poses = np.zeros((0, 7))
        pos = _align(pos, 4)
        (k,) = struct.unpack_from(end + "I", buf, pos)     # graph.links: 3 int32 | Transform | float64[36]
        pos += 4
        if k > 10000000:
            raise ValueError("links %d" % k)
        for _ in range(k):
            pos = _align(pos, 4) + 12
            pos = _align(pos, 8) + 56 + 288
        pos = _align(pos, 4)
        (c,) = struct.unpack_from(end + "I", buf, pos)     # nodes
        pos += 4
        node_ids = []
        if c:
            pos = _align(pos, 4)
            node_ids.append(struct.unpack_from(end + "i", buf, pos)[0])
            if c > 1:
                node_ids.append(None)                     # more nodes: not read (the gate forwards)
        if pos > len(buf):
            raise ValueError("truncated")
        return stamp, ids, poses, node_ids
    except struct.error as exc:
        raise ValueError("truncated (%s)" % exc)


# ---------------------------------------------------------------------------- the rule
def _quat_angle(a, b):
    """Rotation angle (rad) between unit quaternions, rows of a and b (n x 4)."""
    d = np.abs(np.sum(a * b, axis=1)) / np.maximum(np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1), 1e-12)
    return 2.0 * np.arccos(np.clip(d, 0.0, 1.0))


class GraphGate(object):
    """Pure decision state (no ROS). decide() -> (forward, reason, need_ids): need_ids = kept nodes the assembler
    never got in a forwarded graph (inject their data from the skipped messages)."""

    def __init__(self, fallback_s=FALLBACK_S, pose_eps_m=POSE_EPS_M, pose_eps_rad=POSE_EPS_RAD,
                 temp_lin_m=TEMP_LIN_M, temp_ang_rad=TEMP_ANG_RAD, same_lin_m=SAME_LIN_M, same_ang_rad=SAME_ANG_RAD,
                 check_poses=True):
        self.fallback_s = float(fallback_s)
        self.pose_eps_m, self.pose_eps_rad = float(pose_eps_m), float(pose_eps_rad)
        self.temp_lin_m, self.temp_ang_rad = float(temp_lin_m), float(temp_ang_rad)
        self.same_lin_m, self.same_ang_rad = float(same_lin_m), float(same_ang_rad)
        self.check_poses = bool(check_poses)        # False only in the positive control (tests / bench)
        self.reset("start")
        self.counts = {}

    def reset(self, why="reset"):
        self.ref_ids = None                          # sorted int64 ids of the last forwarded graph
        self.ref_poses = None
        self.last_forward = None
        self.reset_reason = why

    def _count(self, forward, reason):
        key = ("fwd:" if forward else "skip:") + reason
        self.counts[key] = self.counts.get(key, 0) + 1

    def _forward(self, reason, ids, poses, now, need=()):
        self.ref_ids, self.ref_poses, self.last_forward = ids, poses, now
        self._count(True, reason)
        return True, reason, set(int(i) for i in need)

    def decide(self, ids, poses, now, node_ids=None):
        ids = np.asarray(ids, dtype=np.int64).reshape(-1)
        poses = np.asarray(poses, dtype=np.float64).reshape(-1, 7)
        if len(ids) != len(poses) or not np.isfinite(poses).all():
            self.reset("malformed")
            return self._forward("malformed", None, None, now)
        if self.ref_ids is None:
            return self._forward(self.reset_reason, ids, poses, now)
        if self.last_forward is None or now - self.last_forward >= self.fallback_s or now < self.last_forward:
            return self._forward("fallback", ids, poses, now)
        if not len(ids):
            return self._forward("empty_graph", ids, poses, now)
        if len(ids) > 1 and (np.diff(ids) <= 0).any():          # rtabmap sends std::map order
            order = np.argsort(ids, kind="stable")
            ids, poses = ids[order], poses[order]
            if (np.diff(ids) <= 0).any():
                return self._forward("duplicate_ids", ids, poses, now)
        newest = int(ids[-1])
        if node_ids is None or len(node_ids) > 1 or (len(node_ids) == 1 and node_ids[0] != newest):
            return self._forward("node_data", ids, poses, now)
        kept_ids, kept_poses = ids[:-1], poses[:-1]
        ref_ids, ref_poses = self.ref_ids, self.ref_poses
        # every kept node was in the last forwarded graph
        pos = np.searchsorted(ref_ids, kept_ids)
        posc = np.minimum(pos, max(len(ref_ids) - 1, 0))
        known = (pos < len(ref_ids)) & (ref_ids[posc] == kept_ids) if len(ref_ids) else np.zeros(len(kept_ids), bool)
        if not known.all():
            return self._forward("node_kept", ids, poses, now, need=kept_ids[~known])
        # no node of the last forwarded graph is gone (its newest may: a parked frame RTAB-Map deleted)
        ref_kept = ref_ids[:-1]
        if len(ref_kept):
            back = np.minimum(np.searchsorted(ids, ref_kept), len(ids) - 1)
            if (ids[back] != ref_kept).any():
                return self._forward("node_removed", ids, poses, now)
        i = int(np.searchsorted(ref_ids, newest))
        newest_known = bool(i < len(ref_ids) and ref_ids[i] == newest)
        # poses of the kept nodes (and of the newest when it is not new)
        cmp_ids = ids if newest_known else kept_ids
        if self.check_poses and len(cmp_ids):
            rp = ref_poses[np.searchsorted(ref_ids, cmp_ids)]
            cp = poses[np.searchsorted(ids, cmp_ids)]
            if (np.linalg.norm(cp[:, :3] - rp[:, :3], axis=1) > self.pose_eps_m).any() or \
                    (_quat_angle(cp[:, 3:], rp[:, 3:]) > self.pose_eps_rad).any():
                return self._forward("pose_changed", ids, poses, now)
        if not newest_known:
            if not len(kept_ids):
                return self._forward("new_node", ids, poses, now)
            a, b, f = poses[-1], poses[-2], ref_poses[-1]  # newest | the latest kept node | last forwarded newest
            lin = float(np.linalg.norm(a[:3] - b[:3]))
            ang = float(_quat_angle(a[None, 3:], b[None, 3:])[0])
            near_kept = lin < self.temp_lin_m and ang < self.temp_ang_rad
            lin_f = float(np.linalg.norm(a[:3] - f[:3]))
            ang_f = float(_quat_angle(a[None, 3:], f[None, 3:])[0])
            if not near_kept and not (lin_f < self.same_lin_m and ang_f < self.same_ang_rad):
                return self._forward("new_node", ids, poses, now)
        self._count(False, "parked_frame" if not newest_known else "same_graph")
        return False, "parked_frame" if not newest_known else "same_graph", set()


class StatusFollow(object):
    """/fpga/mapping/status (mapping_ctl, latched JSON) -> True when the gate must drop its reference: entering
    resetting, while resetting (fail-open), or a new mapping session (a reset or a new mapping process)."""

    def __init__(self):
        self.state = self.session = None

    def on_status(self, text):
        try:
            doc = json.loads(text)
        except ValueError:
            return False
        if not isinstance(doc, dict):
            return False
        state, session = doc.get("state"), doc.get("session")
        session = session if isinstance(session, int) and not isinstance(session, bool) else None
        drop = state == "resetting" or (session is not None and self.session is not None and session != self.session)
        self.state = state
        if session is not None:
            self.session = session
        return drop


class MapDataForwarder(object):
    """The /mapData -> assembler forwarding decision on raw CDR bytes (pure: the ROS node and the host bench both
    drive it). handle() -> (bytes to publish or None, reason). inject(raw, [(node id, cached raw)]) -> bytes with
    those nodes' data appended (the node supplies it with rclpy (de)serialization); None = forward raw as is."""

    def __init__(self, gate=None, inject=None, keep=8):
        self.gate = gate or GraphGate()
        self.inject = inject
        self.keep = int(keep)
        self.follow = StatusFollow()
        self.skipped = collections.OrderedDict()    # newest id -> raw of a skipped message (its node data)
        self.subs = 0
        self.st = {"fwd": 0, "skip": 0, "inject": 0, "parse_errors": 0, "last": None}

    def on_status(self, text):
        if self.follow.on_status(text):
            self.gate.reset("mapping_reset")
            self.skipped.clear()

    def handle(self, raw, now, subs=None):
        if subs is not None:
            if subs > self.subs:
                self.gate.reset("new_subscriber")     # an assembler (re)joined: give it the current graph now
            self.subs = subs
        if self.follow.state == "resetting":
            self.gate.reset("resetting")
        try:
            stamp, ids, poses, node_ids = parse_map_data_prefix(raw)
        except ValueError as exc:
            self.st["parse_errors"] += 1
            self.st["fwd"] += 1
            self.gate.reset("parse_error")
            self.st["last"] = {"forward": True, "reason": "parse_error: %s" % exc}
            return raw, "parse_error"
        fwd, reason, need = self.gate.decide(ids, poses, now, node_ids)
        self.st["last"] = {"forward": fwd, "reason": reason, "nodes": int(len(ids)), "stamp": stamp}
        if not fwd:
            if len(ids):
                self.skipped[int(ids[-1])] = raw
                while len(self.skipped) > self.keep:
                    self.skipped.popitem(last=False)
            self.st["skip"] += 1
            return None, reason
        out = raw
        cached = [(nid, self.skipped[nid]) for nid in sorted(need) if nid in self.skipped]
        if cached and self.inject is not None:
            got = self.inject(raw, cached)
            if got is not None:
                out = got
                self.st["inject"] += 1
                reason += "+inject"
        self.skipped.clear()
        self.st["fwd"] += 1
        return out, reason


def ros_injector(msg_type):
    """inject() for MapDataForwarder with rclpy serialization: append the cached nodes' data to the message."""
    from rclpy.serialization import deserialize_message, serialize_message

    def inject(raw, cached):
        extra = []
        for nid, craw in cached:
            extra += [n for n in deserialize_message(craw, msg_type).nodes if n.id == nid]
        if not extra:
            return None
        msg = deserialize_message(raw, msg_type)
        have = set(n.id for n in msg.nodes)
        msg.nodes = list(msg.nodes) + [n for n in extra if n.id not in have]
        return serialize_message(msg)
    return inject


def main():
    import argparse
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import String
    from rtabmap_msgs.msg import MapData

    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--in-topic", default="/mapData")
    ap.add_argument("--out-topic", default="/operator_map/mapData_gated")
    ap.add_argument("--status-topic", default="/fpga/mapping/status")
    ap.add_argument("--fallback-s", type=float, default=FALLBACK_S)
    args = ap.parse_args()

    rclpy.init()
    node = Node("operator_map_data_gate")
    log = node.get_logger()
    fw = MapDataForwarder(GraphGate(fallback_s=args.fallback_s), ros_injector(MapData))
    pub = node.create_publisher(MapData, args.out_topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    stat_pub = node.create_publisher(String, "/operator_map/gate_status", latched)
    timing = {"ms": 0.0}

    def on_map_data(raw):
        t0 = time.monotonic()
        out, reason = fw.handle(raw, t0, pub.get_subscription_count())
        timing["ms"] = round((time.monotonic() - t0) * 1000, 2)
        if reason == "parse_error":
            log.warning("mapData not parsed (%s): forwarded" % fw.st["last"]["reason"], throttle_duration_sec=30)
        elif reason.endswith("+inject"):
            log.info("forwarded %s with skipped node data" % reason)
        if out is not None:
            pub.publish(out)

    def report():
        stat_pub.publish(String(data=json.dumps(dict(fw.st, counts=fw.gate.counts, decideMs=timing["ms"]),
                                                separators=(",", ":"))))

    node.create_subscription(MapData, args.in_topic, on_map_data,
                             QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE), raw=True)
    node.create_subscription(String, args.status_topic, lambda m: fw.on_status(m.data), latched)
    node.create_timer(5.0, report)
    log.info("display gate %s -> %s (fallback %.0f s)" % (args.in_topic, args.out_topic, args.fallback_s))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
