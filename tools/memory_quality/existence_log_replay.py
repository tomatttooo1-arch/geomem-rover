"""Evidence-level replay of the 10-27 existence score (Config.existence_v3) over a car runtime.jsonl.

The car logs have no depth images and no camera pose, so the full memory cannot be re-run. What they do
have, per frame_processed row and per track: the geometry word the memory saw (track.reason; a fresh
detection shows as last_seen_frame == this frame) and the frame's detection labels. This tool feeds those
words, track by track, to the v3 vote code itself (core.Memory._ex_roll / _ex_hit on stand-in tracks), to
estimate how often the new rule would have emptied a place whose object was in fact still there:

  invalidated      the track's score fell to invalid_below (v3 position_invalid)
  re-detected      ... and the same id was detected again later in the log (the object was there, or put
                   back -- an upper bound on false deletions)

Range is unknown, so P_D's range factor is taken as 1.0 (every miss as if <= 1.2 m: the most aggressive
case). Two blocking variants: "worst" ignores other detections; "label" lets any same-label detection of the
frame block the track's negative votes (the real rule needs the box inside the track's gate).
Step 2 (relocation) is not replayed. Read-only on the log.

See-through filter (10-27 candidate merge): it counts frames where a track is detected AND judged free.
Logs before the candidate have no geometry word for a detected track (its reason is new_visual_observation),
so the filter cannot be evaluated on them; the "+filter" variants run only when a log carries the
frame_processed.track_ev field (raw word per track, detected ones included). There the raw word is used, a
"detected and raw free" frame counts as a conflict (stand-in for the runtime's same-class box within
association_m), and from cfg.see_through_conflicts on that track's free is read as surface_or_uncertain.
"+filter+sustained" (10-27) adds the sustained-free rule (see_through.SustainedFree, Config.sustained_free_*): a
blocked track's raw free passes again when the last 5 s held enough in-view windows, mostly raw free, and no
detection of the track (the replay's "detected": a box on this track in the frame). "+filter" stays the permanent
block (the runtime with sustained_free_override=False).

  python3 tools/memory_quality/existence_log_replay.py RUN/runtime.jsonl [RUN2/runtime.jsonl ...] [--json OUT]
"""
import argparse
import collections
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import numpy as np  # noqa: E402

from perception.fpga_replay.core import Config, Memory, Track  # noqa: E402
from perception.fpga_replay.see_through import SustainedFree  # noqa: E402

DETECTED = ("new_visual_observation", "relocated")


EV_WORDS = {"f": "free", "s": "surface_or_uncertain", "o": "occluded", "v": "out_of_view", "t": "too_small",
            "d": "invalid_depth", "u": "unknown"}


def has_track_ev(path):
    with open(path) as stream:
        for line in stream:
            if '"frame_processed"' in line:
                return '"track_ev"' in line
    return False


def replay(path, variant, cfg=Config(), use_filter=False, sustained=False):
    m = Memory("geometry", cfg)
    conflicts = collections.Counter()  # see-through filter stand-in (use_filter)
    sf = SustainedFree(cfg)            # sustained-free stand-in (use_filter and sustained)
    seen = {}                          # id -> stand-in Track
    events = []
    flips = collections.Counter()
    frames = 0
    with open(path) as stream:
        for line in stream:
            if '"map_reset_started"' in line:
                seen = {}                  # the memory restarts; ids may be reused for other objects
                continue
            if '"frame_processed"' not in line:
                continue
            row = json.loads(line)
            if row.get("event") != "frame_processed" or not row.get("tracks"):
                continue
            frames += 1
            now = float(row["acquisition_timestamp"])
            fid = row["frame_id"]
            labels = {d.get("label") for d in row.get("detections") or ()}
            window = m._ex_window(now)
            track_ev = row.get("track_ev") or {}
            for tr in row["tracks"]:
                tid = tr["id"]
                t = seen.get(tid)
                detected = tr.get("last_seen_frame") == fid or tr.get("reason") in DETECTED
                ev_row = track_ev.get(tid)
                raw = EV_WORDS.get(ev_row[0]) if ev_row else None
                was_blocked = conflicts[tid] >= cfg.see_through_conflicts
                if use_filter and raw == "free" and detected:
                    conflicts[tid] += 1
                if use_filter and sustained and was_blocked:
                    sf.observe(tid, now, raw, detected)
                if t is None:
                    p = np.array(tr.get("position_m") or [0., 0., 0.], float)
                    t = Track(tid, tr["label"], "", p, p, p, now)
                    t.v3_invalid_at, t.v3_event = None, None
                    seen[tid] = t
                    if not detected:
                        t.existence = cfg.existence_init          # first seen in the log without a box
                if detected:
                    was_invalid = t.state == "position_invalid"
                    m._ex_hit(t, now)
                    if was_invalid:
                        flips["invalid_to_confirmed"] += 1
                        t.v3_event["redetected_after_s"] = round(now - t.v3_invalid_at, 2)
                    t.state = "current_confirmed"
                    continue
                if m._ex_roll(t, window):
                    flips["to_invalid"] += 1
                    t.v3_invalid_at = now
                    t.v3_event = {"id": tid, "label": t.label, "t": now, "reason": t.reason,
                                  "redetected_after_s": None}
                    events.append(t.v3_event)
                ev = raw or tr.get("reason") or "unknown"
                if use_filter and ev == "free" and conflicts[tid] >= cfg.see_through_conflicts \
                        and not (sustained and sf.holds(tid)):
                    ev = "surface_or_uncertain"
                if ev not in ("free", "surface_or_uncertain"):
                    continue
                if variant == "label" and t.label in labels:
                    continue
                if ev == "free":
                    t.ex_free = True
                pd = m._miss_pd(t.label, 1.0)                    # range unknown: as if <= 1.2 m
                if pd > 0:
                    t.ex_miss_pd = pd if t.ex_miss_pd is None else min(t.ex_miss_pd, pd)
    return frames, events, flips


def summary(path, variant):
    frames, events, flips = replay(path, variant.split("+")[0], use_filter="+filter" in variant,
                                   sustained=variant.endswith("+sustained"))
    by_label = collections.defaultdict(lambda: [0, 0])
    for e in events:
        by_label[e["label"]][0] += 1
        by_label[e["label"]][1] += e["redetected_after_s"] is not None
    return {"log": str(path), "variant": variant, "frames_with_tracks": frames,
            "invalidated": len(events), "redetected_later": sum(e["redetected_after_s"] is not None for e in events),
            "by_label_invalidated_redetected": {k: v for k, v in sorted(by_label.items())},
            "flips": dict(flips), "events": events}


def old_rule_counts(path):
    """The log's own (negative_views) transitions into position_invalid, for comparison."""
    last, n = {}, 0
    with open(path) as stream:
        for line in stream:
            if '"frame_processed"' not in line:
                continue
            row = json.loads(line)
            for tr in row.get("tracks") or ():
                if tr.get("state") == "position_invalid" and last.get(tr["id"]) != "position_invalid":
                    n += 1
                last[tr["id"]] = tr.get("state")
    return n


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    out = []
    for path in a.logs:
        old = old_rule_counts(path)
        variants = ("worst", "label") + (("worst+filter", "label+filter", "worst+filter+sustained", "label+filter+sustained")
                                         if has_track_ev(path) else ())
        if not has_track_ev(path):
            print("%s: no frame_processed.track_ev -> see-through filter not evaluable (detected tracks' "
                  "geometry words were not logged)" % Path(path).parent.name)
        for variant in variants:
            s = summary(path, variant)
            s["old_rule_to_invalid"] = old
            out.append(s)
            print("%s [%s] frames=%d invalidated=%d redetected_later=%d flips=%s old_rule_to_invalid=%d"
                  % (Path(path).parent.name, variant, s["frames_with_tracks"], s["invalidated"],
                     s["redetected_later"], s["flips"], old))
            for label, (n, r) in s["by_label_invalidated_redetected"].items():
                print("    %-18s invalidated %3d  redetected later %3d" % (label, n, r))
    if a.json:
        with open(a.json, "w") as stream:
            json.dump(out, stream, indent=1, ensure_ascii=False)


if __name__ == "__main__":
    main()
