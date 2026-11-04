"""ros_runtime ``--geometry-source kv_topic``: geometry evidence from /fpga/geometry_frame.

The producer (host/ros_geo_producer.py) is the car's only KV260 frame client. For every
depth frame it publishes one /fpga/geometry_frame body (host/geo_producer.py: the D.2
region-verdict body + a ``producer`` block). The runtime no longer calls the KV260; for a
recognition frame it takes the body of the SAME camera frame -- matched on the depth
header stamp (integer ns; both nodes receive the same /fpga/depth message, while each
node's RAW arrival time differs by its own callback latency) -- or, when the producer
skipped that frame (10 Hz producer, faster camera), the nearest one within
``tolerance_ns``.

``GeometryFrameCache.lookup`` returns ``kv_geometry.GeometryOutcome``-shaped
(evidence | None, reason, info). Evidence = the body's verdict words for tracks that are
still in memory, in memory order (compute_geometry's order). Refusals (each counted):
  no_geometry_frame        nothing within tolerance (caller may wait, then falls back)
  foreign_runtime          body planned from another runtime's /fpga/memory
  world_frame_mismatch     body not in the runtime's world frame
  pose_disagrees           body's T_world_camera differs from the frame's by more than
                           POSE_TOL_M / POSE_TOL_DEG (the producer reads raw TF; with
                           estimated poses the runtime's certified pose must agree)
  <producer failure>       expired_output (200 ms rule), roi_skipped:*, geometry_not_run:*,
                           region backend failures -- no evidence, never partial
Age: the producer withdraws answers older than 200 ms (``expired_output``); the runtime
re-applies the same rule where it uses the evidence (host/geometry_age_gate.py: origin
``producer.acquired_raw_ns``, its ``clock_id`` checked), because its own recognition step
can take longer than the producer's budget (2026-10-11 v2 chain: 10/1779 frames > 200 ms).
Tracks in memory without a verdict in the body (created after the memory revision the
producer used, or overflow) are absent from the dict, i.e. "unknown" for update_memory --
the same as kv overflow; counted in info["uncovered"].

Python 3.8 compatible, no ROS import.
"""

import collections
import json
import math

import numpy as np

from host.region_verdicts import validate_message

POSE_TOL_M = 0.03
POSE_TOL_DEG = 2.0


def _pose_gap(a, b):
    a = np.asarray(a, dtype=np.float64).reshape(4, 4)
    b = np.asarray(b, dtype=np.float64).reshape(4, 4)
    dt = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    r = a[:3, :3].T @ b[:3, :3]
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(r)) - 1.0) / 2.0))))
    return dt, ang


class GeometryFrameCache(object):
    def __init__(self, runtime_boot_id, world_frame, *, tolerance_ns, capacity=64):
        self.runtime_boot_id, self.world_frame = runtime_boot_id, world_frame
        self.tolerance_ns = int(tolerance_ns)
        self.capacity = int(capacity)
        self.bodies = collections.OrderedDict()       # depth stamp ns -> (body, arrival raw ns)
        self.counts = collections.Counter()

    def on_message(self, text, arrival_ns):
        try:
            body = json.loads(text) if isinstance(text, (str, bytes)) else dict(text)
            validate_message(body)
            stamp = int(body["producer"]["depth_stamp_ns"])
        except (ValueError, TypeError, KeyError):
            self.counts["malformed"] += 1
            return False
        self.bodies[stamp] = (body, int(arrival_ns))
        self.bodies.move_to_end(stamp)
        while len(self.bodies) > self.capacity:
            self.bodies.popitem(last=False)
        self.counts["received"] += 1
        return True

    def match(self, depth_stamp_ns):
        """(stamp, body, arrival, how) of the exact or nearest body within tolerance, or None."""
        target = int(depth_stamp_ns)
        if target in self.bodies:
            body, arrival = self.bodies[target]
            return target, body, arrival, "exact"
        best = None
        for stamp in self.bodies:
            gap = abs(stamp - target)
            if gap <= self.tolerance_ns and (best is None or gap < abs(best - target)):
                best = stamp
        if best is None:
            return None
        body, arrival = self.bodies[best]
        return best, body, arrival, "nearest"

    def lookup(self, depth_stamp_ns, memory, T_world_camera, *, now_raw_ns, count=True):
        info = {"match": None, "match_dt_ms": None, "producer_frame_id": None, "acquired_raw_ns": None,
                "frame_age_ms": None, "roi_core": None, "uncovered": 0, "boxes": 0,
                "memory_revision_used": None}
        hit = self.match(depth_stamp_ns)
        if hit is None:
            if count:
                self.counts["no_geometry_frame"] += 1
            return None, "no_geometry_frame", info
        stamp, body, arrival, how = hit
        prod = body.get("producer") or {}
        info.update(match=how, match_dt_ms=(stamp - int(depth_stamp_ns)) / 1e6,
                    producer_frame_id=prod.get("frame_id"), acquired_raw_ns=prod.get("acquired_raw_ns"),
                    roi_core=body.get("roi_core"), memory_revision_used=body.get("memory_revision"),
                    arrival_raw_ns=arrival, clock_id=prod.get("clock_id"),
                    producer_geometry_version=prod.get("geometry_version", "v1"))   # 10-27 (absent = v1)
        if prod.get("acquired_raw_ns") is not None:
            info["frame_age_ms"] = (int(now_raw_ns) - int(prod["acquired_raw_ns"])) / 1e6
        reason = None
        if body.get("failure") is not None:
            reason = str(body["failure"])
        elif body.get("runtime_boot_id") != self.runtime_boot_id:
            reason = "foreign_runtime"
        elif body.get("world_frame") != self.world_frame:
            reason = "world_frame_mismatch"
        else:
            dt, ang = _pose_gap(body["T_world_camera"], T_world_camera)
            info["pose_gap"] = [dt, ang]
            if dt > POSE_TOL_M or ang > POSE_TOL_DEG:
                reason = "pose_disagrees"
        if reason is not None:
            self.counts["refused:" + reason.split(":", 1)[0]] += 1
            return None, reason, info
        words = {}
        for v in body.get("verdicts") or ():
            words.setdefault(v["track_id"], v["geometry_evidence"])
        evidence = {}
        for tid in memory.tracks:              # memory order, like compute_geometry
            if tid in words:
                evidence[tid] = words[tid]
        info["uncovered"] = len(memory.tracks) - len(evidence)
        info["boxes"] = sum(1 for v in body.get("verdicts") or () if v.get("code") is not None)
        self.counts["used:" + how] += 1
        self.counts["uncovered_tracks"] += info["uncovered"]
        return evidence, None, info
