"""/fpga/region_verdicts message (spec D.2) and the runtime merge rules (spec D.3).

Pure functions plus one small stateful merger; no ROS. The runtime calls
``RegionMerger.apply`` from its single executor thread (same thread as update_memory).
"""

import collections
import json
import math

import numpy as np

from perception.fpga_replay.core import Frame

WORDS = ("out_of_view", "too_small", "invalid_depth", "occluded", "free", "surface_or_uncertain")
CODES = (None, 0, 2, 3)
REQUIRED = ("v", "frame_tag", "stamp_ros_s", "world_frame", "T_world_camera", "pose_valid",
            "runtime_boot_id", "memory_revision", "verdicts", "unchecked", "failure")


def frame_tag(depth_stamp_ns):
    return "kvroi-%d" % int(depth_stamp_ns)


def build_message(*, depth_stamp_ns, capture_nano_ns, kv_frame_id, world_frame, T_world_camera,
                  pose_valid, runtime_boot_id, memory_revision, backend="remote_pl", roi_core=None,
                  roi_status=None, roi_times_ms=None, roi_cycles=None, verdicts=(), unchecked=(),
                  failure=None):
    T = np.asarray(T_world_camera, dtype=float)
    msg = {"v": 1, "frame_tag": frame_tag(depth_stamp_ns), "stamp_ros_s": depth_stamp_ns / 1e9,
           "capture_nano_ns": capture_nano_ns, "kv_frame_id": kv_frame_id, "world_frame": world_frame,
           "T_world_camera": [float(x) for x in T.reshape(16)], "pose_valid": bool(pose_valid),
           "runtime_boot_id": runtime_boot_id, "memory_revision": memory_revision,
           "backend": backend, "roi_core": roi_core, "roi_status": roi_status,
           "roi_times_ms": roi_times_ms, "roi_cycles": roi_cycles,
           "verdicts": [] if failure else [dict(v) for v in verdicts],
           "unchecked": [dict(u) for u in unchecked], "failure": failure}
    validate_message(msg)
    return msg


def validate_message(msg):
    if not isinstance(msg, dict):
        raise ValueError("message must be an object")
    missing = [k for k in REQUIRED if k not in msg]
    if missing:
        raise ValueError("missing keys: " + ",".join(missing))
    if msg["v"] != 1:
        raise ValueError("unsupported version")
    if not isinstance(msg["stamp_ros_s"], (int, float)) or not math.isfinite(msg["stamp_ros_s"]):
        raise ValueError("stamp_ros_s must be finite")
    T = msg["T_world_camera"]
    if not isinstance(T, list) or len(T) != 16:
        raise ValueError("T_world_camera must hold 16 numbers")
    if msg["failure"] is not None and msg["verdicts"]:
        raise ValueError("a failed region batch carries no verdicts")
    for v in msg["verdicts"]:
        if not isinstance(v, dict) or not isinstance(v.get("track_id"), str):
            raise ValueError("verdict needs track_id")
        if v.get("geometry_evidence") not in WORDS or v.get("code") not in CODES:
            raise ValueError("verdict word/code outside the core vocabulary")
    return msg


class RegionMerger(object):
    """D.3 merge of /fpga/region_verdicts into Memory via update_geometry_only."""

    def __init__(self, runtime_boot_id, world_frame, max_age_s=1.0):
        self.runtime_boot_id, self.world_frame = runtime_boot_id, world_frame
        self.max_age_s = float(max_age_s)
        self.counts = collections.Counter()
        self.last_region = {}

    def _reject(self, reason, tag=None):
        self.counts["region_rejected:" + reason] += 1
        return {"event": "region_rejected", "reason": reason, "frame_tag": tag, "applied": 0,
                "skipped": {}, "counts": dict(self.counts)}

    def apply(self, memory, message, now_ros_s):
        """Returns an event row; never raises for a bad message, never touches memory on reject."""
        try:
            msg = json.loads(message) if isinstance(message, (str, bytes)) else message
            validate_message(msg)
        except (ValueError, TypeError) as exc:
            return self._reject("malformed", None)
        tag, stamp = msg["frame_tag"], float(msg["stamp_ros_s"])
        if msg["runtime_boot_id"] != self.runtime_boot_id:
            return self._reject("foreign_runtime", tag)
        if msg["world_frame"] != self.world_frame:
            return self._reject("world_frame_mismatch", tag)
        if msg["failure"] is not None:
            self.counts["region_failure"] += 1
            return {"event": "region_failure", "reason": msg["failure"], "frame_tag": tag, "applied": 0,
                    "skipped": {}, "counts": dict(self.counts)}
        if stamp > now_ros_s:
            return self._reject("region_future", tag)
        if now_ros_s - stamp > self.max_age_s:
            return self._reject("region_stale", tag)
        if stamp <= memory.last_geometry_timestamp:
            return self._reject("region_out_of_order", tag)
        evidence, skipped, notes = {}, collections.Counter(), {}
        for v in msg["verdicts"]:
            tid = v["track_id"]
            track = memory.tracks.get(tid)
            if track is None:
                skipped["unknown_track"] += 1
                continue
            if track.last_seen >= stamp:
                skipped["newer_visual_observation"] += 1
                notes[tid] = (v, False, "newer_visual_observation")
                continue
            if tid in evidence:
                skipped["duplicate_track"] += 1
                continue
            evidence[tid] = v["geometry_evidence"]
            notes[tid] = (v, True, None)
        if evidence:
            T = np.asarray(msg["T_world_camera"], dtype=float).reshape(4, 4)
            frame = Frame(tag, stamp, np.zeros((1, 1, 3), dtype=np.uint8),
                          np.full((1, 1), np.nan, dtype=np.float32), np.eye(3), T,
                          bool(msg["pose_valid"]))
            try:
                memory.update_geometry_only(evidence, frame)
            except ValueError as exc:
                return self._reject("update_refused", tag)
        for tid, (v, applied, why) in notes.items():
            self.last_region[tid] = {"frame_tag": tag, "stamp_ros_s": stamp,
                                     "geometry_evidence": v["geometry_evidence"], "code": v.get("code"),
                                     "applied": applied, "skip_reason": why}
        self.counts["region_applied"] += 1
        self.counts["region_tracks_applied"] += len(evidence)
        for k, n in skipped.items():
            self.counts["region_skip:" + k] += n
        return {"event": "region_applied", "reason": None, "frame_tag": tag, "applied": len(evidence),
                "skipped": dict(skipped), "memory_revision_used": msg["memory_revision"],
                "counts": dict(self.counts)}
