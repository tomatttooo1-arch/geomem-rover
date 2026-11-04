"""See-through adaptation (docs/design/GEOMETRY_V2_20261027.md; scope widened 10-27, MEMORY_EXISTENCE_20261027.md).

A transparent or sparse object (glass bottle, chair) lets the depth camera see the wall behind it, so
v2's "behind the expected surface" test can say ``free`` while the object is still there. The tell is
a contradiction inside one frame: the detector still sees the object (an observation of the same class
within association range of the track) while the geometry says free. ``SeeThroughFilter`` counts those
contradictions per track; from ``cfg.see_through_conflicts`` (2) on, every ``free`` of that track becomes
``surface_or_uncertain``. A contradicting frame's own ``free`` is neutralised too (the detection in the
same frame proves it wrong). The runtime applies it to the geometry dict just before
``Memory.update_memory``. It never touches Memory.

Active when ``cfg.see_through_filter`` and (``cfg.existence_v3`` or ``cfg.geometry_v2``). 10-27 candidate merge:
with the existence score one free vote matters, and the 10-27 car log replay showed v1's free on flat
small objects (phone, glasses) is often wrong while the object is still detected (13 of 16 invalidations
decided by free) -- so the filter now runs for v1 too. existence_v3=False and geometry_v2=False -> the dict
is returned unchanged (0f377be behaviour).

Sustained free (10-27, owner approved; ``SustainedFree``): the block above was permanent, so a blocked object
of a low-P_D class (phone: misses alone stop at miss_floor) that was really taken away never left memory. For a
blocked track one free still does not count; but when, over the last ``cfg.sustained_free_window_s`` (5 s), the
track had >= ``sustained_free_min_windows`` (6) in-view vote windows, >= ``sustained_free_fraction`` (0.8) of
them with a raw free word, and no same-class detection within association_m at all, the raw free is passed to
memory unchanged (one ordinary free vote per window while it lasts; the existence score decides). Passing the
free vote rather than forcing position_invalid keeps one decision path (the score, its reason "free", the
relocation hooks on the fall) and needs no new state in Memory. Real-car logs: the longest false free run with
the object still there was ~3.8 s (plastic packaging), then glasses / medicine bottle ~2.1 s.
``cfg.sustained_free_override=False`` (or no frame time given to ``apply``) = the permanent block as before.

No ROS, no torch; Python 3.8.
"""
import collections
import math

import numpy as np

# geometry words that put the track's place in view for the sustained-free count (anything else -- out_of_view,
# too_small, unknown, invalid_*, pose words -- is not an in-view vote)
IN_VIEW_WORDS = frozenset(("free", "surface_or_uncertain", "occluded"))


class SustainedFree(object):
    """Per-track history of the last window_s seconds: (time, vote window, in view, raw free, detected).

    ``observe`` one frame of one track, then ``holds`` says whether the sustained-free rule fires now. Shared by
    the runtime filter and tools/memory_quality/existence_log_replay.py."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.hist = collections.defaultdict(collections.deque)

    def _window(self, now):
        return int(math.floor(float(now) / self.cfg.existence_window_s))

    def observe(self, key, now, raw_word, detected):
        h = self.hist[key]
        h.append((float(now), self._window(now), raw_word in IN_VIEW_WORDS, raw_word == "free", bool(detected)))
        while h and h[0][0] < float(now) - self.cfg.sustained_free_window_s:
            h.popleft()

    def holds(self, key):
        h = self.hist.get(key)
        if not h or any(e[4] for e in h):
            return False
        windows = {}
        for _, w, in_view, free, _ in h:
            if in_view:
                windows[w] = windows.get(w, False) or free      # a window votes free if any frame was free (as core)
        if len(windows) < self.cfg.sustained_free_min_windows:
            return False
        return sum(windows.values()) >= self.cfg.sustained_free_fraction * len(windows) - 1e-9

    def forget(self, key):
        self.hist.pop(key, None)


class SeeThroughFilter(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.conflicts = {}          # track id -> detected-and-free frames so far
        self.see_through = set()     # track ids whose free is no longer believed
        self.counts = {"conflicts": 0, "free_neutralised": 0, "sustained_free": 0}
        self.sustained = SustainedFree(cfg)
        self.last = []

    def _resolve(self, memory, tid):
        merged = getattr(memory, "merged", None) or {}
        seen = set()
        while tid in merged and tid not in seen:
            seen.add(tid); tid = merged[tid]
        return tid

    def _detected(self, track, observations):
        reach = self.cfg.association_m
        for o in observations:
            if o.label == track.label and float(np.linalg.norm(np.asarray(o.position) - track.position)) <= reach:
                return True
        return False

    def active(self):
        cfg = self.cfg
        return bool(getattr(cfg, "see_through_filter", True)
                    and (getattr(cfg, "existence_v3", False) or getattr(cfg, "geometry_v2", False)))

    def blocked(self, memory, tid):
        """True when this track's free is no longer believed (for the per-frame log)."""
        return self._resolve(memory, tid) in self.see_through

    def _sustained_on(self, now):
        return bool(getattr(self.cfg, "sustained_free_override", False)) and now is not None

    def apply(self, geometry, observations, memory, now=None):
        """Filtered copy of ``geometry`` (track id -> word); a list of the changes in ``self.last``.
        ``now`` = the frame's acquisition time (needed by the sustained-free rule; None = rule off)."""
        self.last = []
        if not self.active():
            return geometry
        if self._sustained_on(now) and self.see_through and memory is not None:
            # history of the blocked tracks, every frame (a frame without a word is not in view; a detection
            # counts even when the geometry dict is empty)
            words = geometry or {}
            for tid, track in memory.tracks.items():
                key = self._resolve(memory, tid)
                if key != tid or key not in self.see_through:
                    continue
                self.sustained.observe(key, now, words.get(tid), self._detected(track, observations))
        if not geometry:
            return geometry
        out = dict(geometry)
        for tid, word in geometry.items():
            if word != "free":
                continue
            track = memory.tracks.get(tid)
            key = self._resolve(memory, tid)
            if track is not None and self._detected(track, observations):
                self.conflicts[key] = self.conflicts.get(key, 0) + 1
                self.counts["conflicts"] += 1
                if self.conflicts[key] >= self.cfg.see_through_conflicts:
                    self.see_through.add(key)
                out[tid] = "surface_or_uncertain"
                self.last.append({"track_id": tid, "why": "detected_same_frame", "conflicts": self.conflicts[key]})
            elif key in self.see_through and self._sustained_on(now) and self.sustained.holds(key):
                self.counts["sustained_free"] += 1      # free passes: the place was empty long enough
                self.last.append({"track_id": tid, "why": "sustained_free"})
            elif key in self.see_through:
                out[tid] = "surface_or_uncertain"
                self.last.append({"track_id": tid, "why": "see_through_track"})
        self.counts["free_neutralised"] += sum(1 for c in self.last if c["why"] != "sustained_free")
        return out
