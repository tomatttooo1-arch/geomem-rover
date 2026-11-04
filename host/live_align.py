"""Live map alignment while mapping (2026-10-27; docs/design/LIVE_ALIGN_20261027.md).

The 10-25 alignment (host/map_align.py) corrects the car (map -> odom) and the object memory once, when the mapping
run stops. A drift duplicate made while mapping (10-26 run 150138: bottle 13 / bottle 26, the run never stopped) stays
until then. This module computes the same per-node correction C_i = optimized_i x odometry_i^-1 DURING mapping, from
rtabmap's /mapData, and hands mapping_ctl.py a new alignment message whenever a loop closure (or any optimization)
changes it noticeably; mapping_ctl publishes it on the existing latched /fpga/mapping/map_align, bench_pose re-sends
map -> odom, the runner moves its memory incrementally (map_align.apply_memory(prev=...)).

Data (rtabmap_msgs 0.23.7, ROS 2 Humble -- the deb ros-humble-rtabmap-msgs 0.23.7-1jammy in the image
fpga-dev1-rtabmap:humble-0.23.7-20260910, share/rtabmap_msgs/msg/*.msg):
  MapData   header, graph (MapGraph), nodes (Node[])
  MapGraph  header, map_to_odom (geometry_msgs/Transform), poses_id (int32[]), poses (geometry_msgs/Pose[]), links
  Node      id, map_id, weight, stamp (float64), label, pose (geometry_msgs/Pose, "Pose from odometry not corrected"), ...
rtabmap sends the whole optimized graph in every message but the node data only of the new node(s), so the odometry
poses are accumulated here (id -> (stamp, pose)). Node stamps are the same signature stamps the 10-25 export uses.

Frame check (as 10-25): the graph's root node (smallest positive id) must have the identity correction. When its data
was never received (the subscription came up after it), C_last must instead agree with rtabmap's own
graph.map_to_odom (MAP_TO_ODOM_TOL_*). A failed check publishes nothing; the reason is kept for the status.

When a message goes out: some node's correction moved its node by more than MIN_CHANGE_M (2 cm) or turned it by more
than MIN_CHANGE_RAD (0.5 deg) against the last published alignment (the odometry frame before the first one), and at
least MIN_INTERVAL_S (2 s) after the previous message. Why: 2 cm is under the memory's own position noise (fusion
sample sd ~3-5 cm), 0.5 deg is ~2.6 cm at 3 m range; every message makes bench_pose re-send /tf_static and the runner
move its whole memory, so pure noise must not trigger it; 2 s bounds that work to well under the ~1 Hz node rate while
a loop closure still reaches the memory within ~2 s. A change that waits for the 2 s is sent then, even without a new
/mapData (poll()).

No ROS imports. Python 3.8 compatible.
"""
import math
import time

import numpy as np

from host import map_align

MIN_CHANGE_M = 0.02
MIN_CHANGE_RAD = math.radians(0.5)
MIN_INTERVAL_S = 2.0
MAP_TO_ODOM_TOL_M = 0.05
MAP_TO_ODOM_TOL_RAD = math.radians(1.0)
SOURCE = "live:/mapData"


def pose_msg_matrix(p):
    """4x4 of a geometry_msgs/Pose (duck-typed: .position.x/y/z, .orientation.x/y/z/w)."""
    return map_align.pose7_matrix(p.position.x, p.position.y, p.position.z,
                                  p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)


def transform_msg_matrix(t):
    """4x4 of a geometry_msgs/Transform (.translation, .rotation)."""
    return map_align.pose7_matrix(t.translation.x, t.translation.y, t.translation.z,
                                  t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)


def tables_from_map_data(msg):
    """(nodes [(id, stamp, 4x4 odometry)], graph {id: 4x4 optimized}, map_to_odom 4x4 or None) of a
    rtabmap_msgs/MapData. Nodes / poses that do not convert are left out (counted by the caller via the sizes)."""
    nodes = []
    for n in msg.nodes:
        try:
            nodes.append((int(n.id), float(n.stamp), pose_msg_matrix(n.pose)))
        except map_align.AlignError:
            pass
    graph = {}
    for i, p in zip(msg.graph.poses_id, msg.graph.poses):
        try:
            graph[int(i)] = pose_msg_matrix(p)
        except map_align.AlignError:
            pass
    try:
        m2o = transform_msg_matrix(msg.graph.map_to_odom)
    except (map_align.AlignError, AttributeError):
        m2o = None
    return nodes, graph, m2o


class LiveAlign(object):
    """Accumulates /mapData, decides when a new alignment goes out. mapping_ctl owns one per process."""

    def __init__(self, run="", clock=time.monotonic, min_change_m=MIN_CHANGE_M, min_change_rad=MIN_CHANGE_RAD,
                 min_interval_s=MIN_INTERVAL_S):
        self.run, self.clock = run or "", clock
        self.min_change_m, self.min_change_rad, self.min_interval_s = min_change_m, min_change_rad, min_interval_s
        self.seq = 0               # every alignment message of this process (live, final, reset) gets the next one
        self.session = 0
        self.odom = {}             # node id -> (stamp, 4x4 odometry pose)
        self.published = None      # the last alignment sent (None: the odometry frame / identity)
        self.pending = None        # computed, over the threshold, waiting for the interval
        self.last_pub_t = None
        self.messages = self.computed = self.sent = 0
        self.reason = None         # why the last message gave no alignment (None: fine)
        self.last_change = None    # {"m", "rad"} of the last computed alignment against the published one

    # ---------------------------------------------------------------- input
    def reset(self):
        """A map reset (mapping_ctl): the old graph is gone; the identity map -> odom is sent by the reset itself."""
        self.session += 1
        self.odom.clear()
        self.published = self.pending = None
        self.last_pub_t = None
        self.reason = self.last_change = None

    def on_map_data(self, nodes, graph, map_to_odom=None):
        """nodes [(id, stamp, 4x4)], graph {id: 4x4}; -> the computed alignment (also kept as pending when it is a
        change worth sending) or None (reason in self.reason). Never raises."""
        self.messages += 1
        for nid, stamp, m in nodes:
            if nid > 0 and math.isfinite(stamp):
                self.odom[nid] = (float(stamp), m)
        opt = {i: m for i, m in graph.items() if i > 0}
        if not opt:
            self.reason = "graph without nodes"
            return None
        root = min(opt)
        check_root = root in self.odom
        try:
            a = map_align.compute_tables(opt, self.odom, {i: v[0] for i, v in self.odom.items()},
                                         check_root=check_root)
        except map_align.AlignError as exc:
            self.reason = str(exc)
            self.pending = None
            return None
        except Exception as exc:                  # noqa: BLE001 - a bad message must not end the controller
            self.reason = "%s: %s" % (type(exc).__name__, exc)
            self.pending = None
            return None
        if not check_root:
            if map_to_odom is None:
                self.reason = "graph root node %d not received and no map_to_odom to check against" % root
                self.pending = None
                return None
            d = map_align.pose7_matrix(*a["map_odom"]) @ np.linalg.inv(map_to_odom)
            dm, dr = float(np.linalg.norm(d[:3, 3])), map_align.rotation_angle(d)
            if not (dm <= MAP_TO_ODOM_TOL_M and dr <= MAP_TO_ODOM_TOL_RAD):
                self.reason = ("graph root node %d not received and C_last differs from rtabmap's map_to_odom by "
                               "%.3f m / %.4f rad" % (root, dm, dr))
                self.pending = None
                return None
            a["root"] = dict(a["root"], node_id=None, missing_root=root, checked_by="map_to_odom",
                             map_to_odom_diff_m=dm, map_to_odom_diff_rad=dr)
        self.computed += 1
        self.reason = None
        dm, dr = self.change(a)
        a["change"] = self.last_change = {"m": dm, "rad": dr}
        self.pending = a if (dm > self.min_change_m or dr > self.min_change_rad) else None
        return a

    def change(self, a):
        """(max m, max rad) over a's nodes: how far each node's corrected pose moves against the published
        alignment's correction at the same stamp (the odometry frame when none was published)."""
        by_stamp = {v[0]: v[1] for v in self.odom.values()}
        prev = None if self.published is None else map_align.Corrections(self.published)
        dm = dr = 0.0
        for n in a["nodes"]:
            c = map_align.pose7_matrix(*n[1:])
            cp = np.eye(4) if prev is None else prev.at(float(n[0]))
            d = c @ np.linalg.inv(cp)
            o = by_stamp.get(n[0])
            p = np.zeros(3) if o is None else o[:3, 3]
            moved = (c[:3, :3] @ p + c[:3, 3]) - (cp[:3, :3] @ p + cp[:3, 3])
            dm = max(dm, float(np.linalg.norm(moved)))
            dr = max(dr, map_align.rotation_angle(d))
        return dm, dr

    # ---------------------------------------------------------------- output
    def next_seq(self):
        self.seq += 1
        return self.seq

    def poll(self, now=None):
        """The /fpga/mapping/map_align message to publish now, or None."""
        if self.pending is None:
            return None
        now = self.clock() if now is None else now
        if self.last_pub_t is not None and now - self.last_pub_t < self.min_interval_s:
            return None
        a, self.pending = self.pending, None
        self.published, self.last_pub_t = a, now
        self.sent += 1
        seq = self.next_seq()
        return {"key": "live:%s:%d:%d" % (self.run, self.session, seq), "seq": seq, "live": True,
                "map_odom": a["map_odom"], "node_id": a["node_id"], "nodes": a["nodes"], "count": a["count"],
                "root": a["root"], "change": a["change"], "source": SOURCE}

    def nonidentity_sent(self):
        """True when the last alignment sent (this session) is not the identity: map -> odom is not identity."""
        return self.published is not None and not map_align.is_identity(self.published)

    def status(self):
        return {"enabled": True, "session": self.session, "messages": self.messages, "nodes": len(self.odom),
                "computed": self.computed, "sent": self.sent, "seq": self.seq, "reason": self.reason,
                "last_change": self.last_change,
                "map_odom": None if self.published is None else list(self.published["map_odom"]),
                "pending": self.pending is not None}


# -------------------------------------------------------------------- runner side (host/ros_runtime.py, --live-align)
# The runner keeps on its node: memory_align_pending ({"doc", "key", "t", "seq", "live"} or None), memory_align_done
# (keys), memory_align_cur (the alignment doc the memory is in; None = the odometry frame), memory_align_seq (its
# seq), live_align_frames (counts). These functions take that node (any object with the attributes) as ``st``.
# Invariant kept: every remembered position = C_cur(its stamp) x its odometry-frame position, where C_cur is the
# alignment last applied -- apply_memory moves by C_new C_cur^-1, frames are brought into C_cur before merging.

def new_runner_state(st):
    st.memory_align_pending, st.memory_align_done = None, set()
    st.memory_align_cur, st.memory_align_seq = None, None
    st.live_align_frames = {"tagged": 0, "untagged": 0, "converted": 0, "max_shift_m": 0.0}
    return st


def runner_receive(st, doc, key, now):
    """Hold an alignment message (already validated). -> ("stale", fields) when an equal or newer seq was applied or
    is held (dropped, key marked done), else ("received", fields) with it held (replacing an older held one)."""
    seq = doc.get("seq")
    seq = seq if isinstance(seq, int) and not isinstance(seq, bool) else None
    known = [v for v in (st.memory_align_seq, (st.memory_align_pending or {}).get("seq")) if v is not None]
    if seq is not None and known and seq <= max(known):
        st.memory_align_done.add(key)
        return "stale", {"key": key, "seq": seq, "newest": max(known)}
    st.memory_align_pending = {"doc": doc, "key": key, "t": now, "seq": seq, "live": bool(doc.get("live"))}
    return "received", {"key": key, "seq": seq, "live": bool(doc.get("live"))}


def runner_apply(st, memory):
    """Move the memory to the held alignment (the caller saw TF map -> odom become it). -> apply_memory's info."""
    p = st.memory_align_pending
    st.memory_align_pending = None
    st.memory_align_done.add(p["key"])
    info = map_align.apply_memory(memory, p["doc"], prev=st.memory_align_cur)
    st.memory_align_cur = p["doc"]
    if p.get("seq") is not None:
        st.memory_align_seq = p["seq"]
    return dict(info, key=p["key"], seq=p.get("seq"), live=p.get("live"), frames=dict(st.live_align_frames))


def runner_reset(st, align_key):
    """A map reset cleared the memory; map -> odom goes back to identity (mapping_ctl's message is no memory move)."""
    st.memory_align_pending = None
    st.memory_align_done.add(align_key)
    st.memory_align_cur = None


def runner_align_frame(st, memory, frame):
    """Before a frame is merged: when its tag (frame.map_odom_tag, the map -> odom its pose was looked up with) is the
    held alignment, TF has become it -> apply it now (-> the apply info, else None); then frame.T_world_camera (and
    frame.ground_z_m from frame.world_base_matrix) <- C_cur(stamp) x tag^-1 x it. Untagged frames: unchanged."""
    tag = getattr(frame, "map_odom_tag", None)
    if tag is None:
        st.live_align_frames["untagged"] += 1
        return None
    st.live_align_frames["tagged"] += 1
    applied = None
    p = st.memory_align_pending
    if p is not None and np.allclose(map_align.pose7_matrix(*p["doc"]["map_odom"]), tag, atol=1e-6):
        applied = runner_apply(st, memory)
    d = map_align.frame_correction(st.memory_align_cur, tag, frame.timestamp)
    if not np.allclose(d, np.eye(4), atol=1e-9):
        frame.T_world_camera = d @ np.asarray(frame.T_world_camera, dtype=float)
        wb = getattr(frame, "world_base_matrix", None)
        if wb is not None:
            frame.ground_z_m = float((d @ np.asarray(wb, dtype=float))[2, 3])
        st.live_align_frames["converted"] += 1
        st.live_align_frames["max_shift_m"] = max(st.live_align_frames["max_shift_m"], float(np.linalg.norm(d[:3, 3])))
    return applied
