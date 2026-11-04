"""The three extra jobs of the car's geometry host (spec D.2), as library functions.

The ROS node that sends /fpga/depth to the KV260 does not exist yet (B1 car integration);
it will call these. No ROS and no clock reads here: the caller passes its steady clock.

1. ``MemoryWatch``: keep only the newest /fpga/memory; refuse to plan boxes (counted
   ``roi_skipped:<reason>``) when there is none, its world frame is not the one the node
   can look up camera poses in, or the heartbeat stopped for more than 5 s. Geometry runs
   regardless -- a skipped frame is sent without boxes.
2. ``plan_frame``: project every track with package 1's ``plan_boxes`` (the same function
   ``check_boxes`` uses -- not a copy) in the A.2/O11 order of ``memory_priority``, at most
   32 enabled boxes, stamped with the memory revision via ``batch.with_revision``.
3. ``verdict_message`` / ``frame_stats``: the /fpga/region_verdicts and
   /fpga/kv_link/frame_stats bodies. The word for each kernel row follows
   ``NativeObjectEvidence.check_boxes`` exactly (``evidence_word``), so the KV answer and the
   local check give the same words for the same input.
"""

import collections
import json

from host.m7_object_native import ROI_MAX_BOXES, ROI_VERDICTS, memory_priority, plan_boxes
from host.region_verdicts import build_message

HEARTBEAT_LOST_S = 5.0


class MemoryWatch(object):
    def __init__(self, camera_world_frame, heartbeat_lost_s=HEARTBEAT_LOST_S):
        self.camera_world_frame = camera_world_frame
        self.heartbeat_lost_s = float(heartbeat_lost_s)
        self.latest = None
        self.received_s = None
        self.counts = collections.Counter()

    def on_memory(self, text, now_s):
        """/fpga/memory callback: keep the newest message (by arrival)."""
        try:
            msg = json.loads(text) if isinstance(text, (str, bytes)) else dict(text)
            if not isinstance(msg.get("tracks"), list) or not isinstance(msg.get("revision"), int):
                raise ValueError("not a /fpga/memory body")
        except (ValueError, TypeError, AttributeError):
            self.counts["memory_malformed"] += 1
            return False
        self.latest, self.received_s = msg, float(now_s)
        self.counts["memory_received"] += 1
        return True

    def usable(self, now_s):
        """(memory message, None) or (None, reason); every refusal is counted."""
        if self.latest is None:
            reason = "no_memory"
        elif self.latest.get("world_frame") != self.camera_world_frame:
            reason = "world_frame_mismatch"
        elif float(now_s) - self.received_s > self.heartbeat_lost_s:
            reason = "memory_heartbeat_lost"
        else:
            return self.latest, None
        self.counts["roi_skipped:" + reason] += 1
        return None, reason


def track_entries(memory_msg, with_position=False):
    """(entries for plan_boxes, priority tracks dict, unusable track ids). ``with_position``
    (10-27 geometry v2): each entry also carries the track's position_m (None when absent)."""
    entries, tracks, unusable = [], {}, []
    for t in memory_msg.get("tracks") or ():
        tid, bounds = t.get("id"), t.get("bounds_m")
        try:
            lower, upper = [[float(v) for v in corner] for corner in bounds]
            if len(lower) != 3 or len(upper) != 3:
                raise ValueError
        except (TypeError, ValueError):
            unusable.append(tid)
            continue
        if with_position:
            pos = t.get("position_m")
            try:
                pos = [float(v) for v in pos]
                if len(pos) != 3:
                    raise ValueError
            except (TypeError, ValueError):
                pos = None
            entries.append((tid, [lower, upper], pos))
        else:
            entries.append((tid, [lower, upper]))
        tracks[tid] = {"label": t.get("label"), "last_seen": t.get("last_seen"),
                       "position_m": t.get("position_m")}
    return entries, tracks, unusable


def plan_frame(memory_msg, depth_shape, K, T_world_camera, cfg, max_boxes=ROI_MAX_BOXES, counts=None,
               ground_z_m=None):
    """PlanResult whose batch carries the memory revision, or None (no pose: counted).
    ``ground_z_m``: world ground height for the geometry-v2 probe (None: cfg.ground_z_m)."""
    if T_world_camera is None:
        if counts is not None:
            counts["roi_skipped:no_world_pose"] += 1
        return None
    entries, tracks, unusable = track_entries(memory_msg, with_position=bool(getattr(cfg, "geometry_v2", False)))
    if counts is not None and unusable:
        counts["roi_track_unusable"] += len(unusable)
    plan = plan_boxes(depth_shape, K, T_world_camera, entries, cfg, max_boxes=max_boxes,
                      priority=memory_priority(tracks, memory_msg.get("target_label")), ground_z_m=ground_z_m)
    if plan.batch is not None:
        plan = plan.__class__(plan.batch.with_revision(memory_msg["revision"]), plan.host_resolved,
                              plan.overflow, plan.row_map, plan.plans)
    return plan


def evidence_word(row, cfg):
    """core.py word of one checked kernel row -- the same branches as check_boxes.
    10-27 cfg.geometry_v2: the word is recomputed from the four counts (geometry_v2.word_from_counts,
    50 % rule); the kernel's verdict code is its own 0.9-rule echo and is not used."""
    if getattr(cfg, "geometry_v2", False):
        from perception.fpga_replay.geometry_v2 import word_from_counts
        return word_from_counts(row["roi_pixels"], row["roi_valid_pixels"], row["roi_front_pixels"],
                                row["roi_behind_pixels"], cfg)
    verdict = ROI_VERDICTS[row["verdict_code"]]
    if (row["roi_valid_pixels"] < cfg.min_pixels or
            row["roi_valid_pixels"] / row["roi_pixels"] < cfg.min_valid_fraction):
        return "invalid_depth"
    if verdict == "occluded":
        return "occluded"
    if verdict == "free_space_conflict":
        return "free"
    return "surface_or_uncertain"


def verdict_rows(plan, outcome, cfg):
    """(verdicts, unchecked) of one frame from the plan and the checked KV answer."""
    verdicts = [{"track_id": tid, "geometry_evidence": word, "code": None, "roi_xyxy": None,
                 "counts": None} for tid, word in plan.host_resolved.items()]
    unchecked = [{"track_id": tid, "reason": "overflow"} for tid in plan.overflow]
    if plan.batch is not None:
        for sent, row in zip(plan.batch.rows, outcome.rows):
            verdicts.append({"track_id": plan.row_map[sent.row_id], "geometry_evidence": evidence_word(row, cfg),
                             "code": int(row["verdict_code"]),
                             "roi_xyxy": None if sent.xyxy is None else [int(v) for v in sent.xyxy],
                             "counts": {"pixels": int(row["roi_pixels"]), "valid": int(row["roi_valid_pixels"]),
                                        "front": int(row["roi_front_pixels"]),
                                        "behind": int(row["roi_behind_pixels"])}})
    return verdicts, unchecked


def verdict_message(plan, outcome, *, memory_msg, depth_stamp_ns, capture_nano_ns, kv_frame_id, world_frame,
                    T_world_camera, pose_valid, cfg):
    """/fpga/region_verdicts body (D.2). ``outcome``: backend.last_roi (RoiOutcome) for a
    frame that carried a batch; None when every track was resolved on the host."""
    failure = None
    if plan.batch is not None and (outcome is None or not outcome.ok):
        failure = outcome.reason if outcome is not None else "region_backend_failure:no_outcome"
    if failure:
        verdicts, unchecked = [], []
    else:
        verdicts, unchecked = verdict_rows(plan, outcome, cfg)
    times = dict(outcome.times_ns) if outcome is not None and outcome.times_ns else None
    return build_message(depth_stamp_ns=depth_stamp_ns, capture_nano_ns=capture_nano_ns, kv_frame_id=kv_frame_id,
                         world_frame=world_frame, T_world_camera=T_world_camera, pose_valid=pose_valid,
                         runtime_boot_id=memory_msg["runtime_boot_id"], memory_revision=memory_msg["revision"],
                         roi_core=outcome.roi_core if outcome is not None else None,
                         roi_status=0 if outcome is not None and outcome.ok else None,
                         roi_times_ms=None if times is None else {k: v / 1e6 for k, v in times.items()},
                         roi_cycles=outcome.roi_cycles if outcome is not None else None,
                         verdicts=verdicts, unchecked=unchecked, failure=failure)


def frame_stats(*, kv_frame_id, capture_nano_ns, done_nano_ns, result, mailbox_wait_ms, client_last=None,
                client_counts=None):
    """/fpga/kv_link/frame_stats body (D.2; KL s.2.5: one Nano clock, channel = age - PL device
    - mailbox wait). ``client_last``: backend.client.last (rtt_s, kv_ns{9 segments})."""
    age_ms = (int(done_nano_ns) - int(capture_nano_ns)) / 1e6
    kv_ms = None
    rtt_ms = channel_ms = None
    if client_last and "kv_ns" in client_last:
        kv_ms = {k: v / 1e6 for k, v in client_last["kv_ns"].items()}
        rtt_ms = client_last.get("rtt_s") * 1e3 if client_last.get("rtt_s") is not None else None
        if kv_ms.get("core_device") is not None and mailbox_wait_ms is not None:
            channel_ms = age_ms - kv_ms["core_device"] - mailbox_wait_ms
    return {"v": 1, "kv_frame_id": kv_frame_id, "capture_nano_ns": int(capture_nano_ns), "result": result,
            "frame_age_ms": age_ms, "channel_ms": channel_ms, "mailbox_wait_ms": mailbox_wait_ms,
            "rtt_ms": rtt_ms, "kv_times_ms": kv_ms, "client_counts": dict(client_counts or {})}
