"""/fpga/memory message and its publish schedule (spec D.1). No ROS; the runtime calls it.

Full snapshot, never deltas; history is not sent (history_len + history_last only).
Revision +1 exactly when content changes (tracks or target label); changes inside a 0.5 s
window are merged into one publish at the window end; every 1 s the current copy is
re-sent with the same revision as a heartbeat.
"""

import collections
import hashlib
import json
import math
import os


def new_boot_id():
    return os.urandom(8).hex()


def _plain(value):
    """JSON-safe copy: numpy scalars/arrays -> Python, non-finite floats -> None."""
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if hasattr(value, "tolist"):
        return _plain(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def track_rows(memory, last_region=None):
    rows = []
    last_region = last_region or {}
    for t in memory.tracks.values():
        last = t.history[-1] if t.history else None
        rows.append(_plain({
            "id": t.id, "label": t.label, "color": t.color, "position_m": t.position,
            "bounds_m": [t.lower, t.upper], "last_seen": float(t.last_seen),
            "last_seen_frame": t.last_seen_frame, "state": t.state, "reason": t.reason,
            "negative_views": len(t.negative_poses), "history_len": len(t.history),
            "history_last": None if last is None else {k: last.get(k) for k in
                                                       ("frame_id", "timestamp", "bbox", "score")},
            "last_region": last_region.get(t.id)}))
        if getattr(t, "number", None) is not None:      # 10-27: class-local display number (瓶子 2)
            rows[-1]["number"] = int(t.number)
        prior = getattr(t, "prior", None)
        if prior is not None:           # 10-18: loaded from an earlier run (host/memory_store.py restore)
            rows[-1]["prior"] = _plain({k: prior.get(k) for k in ("runtime_boot_id", "written_wall", "last_seen",
                                                                  "state")})
    return rows


def shown_ids(tracks):
    """Ids the map shows (the console hides position_invalid): a change here is published without delay."""
    return frozenset(r.get("id") for r in tracks or () if r.get("state") != "position_invalid")


def canonical(obj):
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
                      allow_nan=False).encode("utf-8")


def digest(tracks):
    return hashlib.sha256(canonical(tracks)).hexdigest()


def target_label_of(task, memory):
    """Current task's target class (O11 priority input), or None without an active task."""
    if task is None or task.state in task.TERMINAL:
        return None
    track = memory.tracks.get(task.target_id) if task.target_id else None
    if track is not None:
        return track.label
    for row in getattr(task, "candidates", None) or ():
        if row.get("label"):
            return row["label"]
    return None


class MemoryFeed(object):
    def __init__(self, runtime_boot_id, world_frame, sensor_source, min_interval_s=0.5,
                 heartbeat_s=1.0):
        self.runtime_boot_id, self.world_frame, self.sensor_source = runtime_boot_id, world_frame, sensor_source
        self.min_interval_s, self.heartbeat_s = min_interval_s, heartbeat_s
        self.revision = -1
        self.key = None
        self.shown = None                  # ids not position_invalid in the last published message
        self.last_publish = None
        self.last_change_publish = None
        self.counts = collections.Counter()

    def step(self, memory, now_mono, stamp_ros_s, target_label=None, last_region=None):
        """The message to publish now, or None. ``now_mono`` is a steady clock in seconds."""
        tracks = track_rows(memory, last_region)
        tracks_digest = digest(tracks)
        key = (tracks_digest, target_label)
        shown = shown_ids(tracks)
        if key != self.key:
            # 10-27 fast removal: an object leaving (position_invalid) or (re)appearing on the map is sent at once,
            # not held to the end of the 0.5 s window (the console hides position_invalid tracks)
            urgent = shown != self.shown
            if (not urgent and self.last_change_publish is not None
                    and now_mono - self.last_change_publish < self.min_interval_s):
                self.counts["held_in_window"] += 1
                return None
            if urgent and self.last_change_publish is not None:
                self.counts["shown_changed_now"] += 1
            self.key = key
            self.shown = shown
            self.revision += 1
            self.last_change_publish = now_mono
            self.counts["changed"] += 1
        elif self.last_publish is not None and now_mono - self.last_publish < self.heartbeat_s:
            return None
        else:
            self.counts["heartbeat"] += 1
        self.last_publish = now_mono
        cfg = memory.cfg
        return {"v": 1, "runtime_boot_id": self.runtime_boot_id, "revision": self.revision,
                "stamp_ros_s": stamp_ros_s, "world_frame": self.world_frame,
                "sensor_source": self.sensor_source, "strategy": memory.strategy,
                "target_label": target_label,
                "cfg": {"depth_margin_m": cfg.depth_margin_m, "min_pixels": cfg.min_pixels,
                        "min_valid_fraction": cfg.min_valid_fraction, "free_fraction": cfg.free_fraction,
                        "negative_views": cfg.negative_views},
                "digest": tracks_digest, "tracks": tracks}
