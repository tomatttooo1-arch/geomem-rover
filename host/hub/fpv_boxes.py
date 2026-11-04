"""Console first-person boxes (owner 10-27: "the console's first-person view gets the same boxes as the car
screen AR"). Pure Python, no ROS, no numpy; the hub runs it on the Nano (one json.loads per producer frame).

Source = the car screen's own source: the geometry producer's per-frame /fpga/geometry_frame body (D.2
region-verdict rows, host/geometry_region_glue.verdict_rows), the rows the KV260 checked and the AR core
draws. Same choice as host/kv_ar.py (ar_inputs + select_display_boxes, kv_server --ar-box-max 5
--ar-box-nms 0.5): rows with a ROI answer (code in {0, 2, 3}) and a pixel box, in request order, same-class
greedy NMS IoU > 0.5 hidden, then the first 5. A failed frame (expired_output, roi_skipped, backend failure)
carries no rows: it is NOT kept, so the preview keeps the last good frame's boxes (no flicker), exactly
like the car screen's hold. Labels = the memory track's class (the producer sends the same one in the REQ
LABEL block), plus its class-local number.

10-28 (owner: "the 3D map drops an object taken away in front of the car / found gone on return, but its box
stays in the first-person view"): rows are planned for EVERY memory track, position_invalid included (the
existence score still takes their votes), while the console map hides position_invalid tracks
(operator/src/remote.js). The first-person view now follows the map: a row whose track is position_invalid in
the newest memory is dropped before NMS / the 5-box cut (so it neither takes a slot nor hides a shown box), and
``pair`` drops it again with the memory current at preview time (a frame kept just before the leave is pushed).

Pairing (at preview send time, host/hub/hub_core.preview_frame): the kept frame whose depth stamp is
nearest the preview's colour stamp (depth is HW-registered to the colour camera, one clock, same
640x480 pixels). Nearest frame older than HOLD_MS -> no boxes (stale: the car screen's max_age_ms 1000
gives STALE as well); nothing ever fed -> no ``ar`` key at all.
"""

import collections
import json
import math

MAX_SHOW = 5           # kv_server --ar-box-max default (the car)
NMS_IOU = 0.5          # kv_server --ar-box-nms default
MAX_ROWS = 32          # kv_ar.MAX_BOXES
HOLD_MS = 1000.0       # kv_ar_tune.json max_age_ms (the car's STALE age)
RING = 16              # ~1.6 s of producer frames at 10 Hz
VERDICTS = (0, 2, 3)   # host/m7_object_native.ROI_VERDICTS: 0 ?, 2 free-space conflict, 3 occluded
# 10-27 geometry v2: the kernel's code is its own 0.9-rule echo, the Nano's word (geometry_evidence, from the
# four counts) is what memory got -- with producer.geometry_version "v2" the console colour follows the word
# (row selection still by the code, as on the car screen). v1 bodies carry no version: colour = code, unchanged.
V2_WORD_CODE = {"free": 2, "occluded": 3}
HIDDEN_STATE = "position_invalid"   # the console map hides these tracks (operator/src/remote.js)


def box_iou(a, b):
    """IoU of two closed-interval pixel boxes (x0, y0, x1, y1) -- == host/kv_ar._box_iou."""
    ix = min(a[2], b[2]) - max(a[0], b[0]) + 1
    iy = min(a[3], b[3]) - max(a[1], b[1]) + 1
    if ix <= 0 or iy <= 0:
        return 0.0
    inter = ix * iy
    area = lambda r: (r[2] - r[0] + 1) * (r[3] - r[1] + 1)  # noqa: E731
    return inter / float(area(a) + area(b) - inter)


def select(cands, max_show=MAX_SHOW, nms_iou=NMS_IOU):
    """== host/kv_ar.select_display_boxes on dicts (xyxy, key): (kept, hidden)."""
    kept, hidden = [], 0
    for c in cands:
        if nms_iou is not None and any(k["key"] == c["key"] and box_iou(k["xyxy"], c["xyxy"]) > nms_iou
                                       for k in kept):
            hidden += 1
            continue
        if len(kept) >= max_show:
            hidden += 1
            continue
        kept.append(c)
    return kept, hidden


def _int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def track_index(memory_row):
    """{track id: (label, number)} of a /fpga/memory body (anything else -> {})."""
    out = {}
    tracks = memory_row.get("tracks") if isinstance(memory_row, dict) else None
    for t in tracks or ():
        if isinstance(t, dict) and isinstance(t.get("id"), str):
            label = t.get("label") if isinstance(t.get("label"), str) else None
            number = t.get("number") if _int(t.get("number")) else None
            out[t["id"]] = (label, number)
    return out


def hidden_ids(memory_row):
    """Ids of the tracks the console map does not show (state position_invalid) in a /fpga/memory body."""
    tracks = memory_row.get("tracks") if isinstance(memory_row, dict) else None
    return frozenset(t["id"] for t in tracks or () if isinstance(t, dict) and isinstance(t.get("id"), str)
                     and t.get("state") == HIDDEN_STATE)


def frame_boxes(body, tracks, hide=frozenset()):
    """One /fpga/geometry_frame body -> {stamp_s, frame_id, depth_wh, boxes, hidden} or None (failed /
    malformed frame: not kept). ``tracks``: track_index of the newest memory; ``hide``: its hidden_ids
    (rows of those tracks are dropped before the selection, counted in ``invalid``)."""
    if not isinstance(body, dict) or body.get("failure") is not None:
        return None
    stamp = body.get("stamp_ros_s")
    if not isinstance(stamp, (int, float)) or isinstance(stamp, bool) or not math.isfinite(stamp):
        return None
    rows = body.get("verdicts")
    if not isinstance(rows, list):
        return None
    prod = body.get("producer") if isinstance(body.get("producer"), dict) else {}
    wh = prod.get("depth_wh")
    depth_wh = [int(wh[0]), int(wh[1])] if (isinstance(wh, list) and len(wh) == 2 and all(_int(v) and v > 0 for v in wh)) else None
    v2 = prod.get("geometry_version") == "v2"
    cands, invalid = [], 0
    for v in rows:
        if not isinstance(v, dict) or v.get("code") not in VERDICTS:
            continue
        if v.get("track_id") in hide:
            invalid += 1                               # gone from the map -> gone from the first-person view
            continue
        xy = v.get("roi_xyxy")
        if not (isinstance(xy, list) and len(xy) == 4 and all(_int(c) for c in xy)
                and 0 <= xy[0] <= xy[2] and 0 <= xy[1] <= xy[3]):
            continue
        if depth_wh is not None and not (xy[2] < depth_wh[0] and xy[3] < depth_wh[1]):
            continue                                   # kv_ar skips a box outside the depth image too
        if len(cands) >= MAX_ROWS:
            break
        tid = v.get("track_id")
        label, number = tracks.get(tid, (None, None))
        code = V2_WORD_CODE.get(v.get("geometry_evidence"), 0) if v2 else v["code"]
        cands.append({"id": tid, "label": label, "number": number, "v": code, "xyxy": list(xy),
                      "key": (label or "").strip().lower() or str(tid)})
    kept, hidden = select(cands)
    for k in kept:
        del k["key"]
    fid = prod.get("frame_id")
    return {"stamp_s": float(stamp), "frame_id": fid if _int(fid) else None, "depth_wh": depth_wh,
            "boxes": kept, "hidden": hidden, "invalid": invalid}


class FpvBoxes(object):
    """Ring of the last RING good producer frames; ``pair`` picks one for a preview frame."""

    def __init__(self, ring=RING, hold_ms=HOLD_MS):
        self.ring = collections.deque(maxlen=int(ring))
        self.hold_ms = float(hold_ms)
        self.fed = False
        self.counts = collections.Counter()

    def feed(self, text, memory_row):
        self.fed = True
        try:
            body = json.loads(text) if isinstance(text, (str, bytes)) else text
        except ValueError:
            self.counts["fpv_geo_bad_json"] += 1
            return None
        f = frame_boxes(body, track_index(memory_row), hidden_ids(memory_row))
        if f is None:
            self.counts["fpv_geo_not_kept"] += 1
            return None
        self.counts["fpv_geo_kept"] += 1
        if f["invalid"]:
            self.counts["fpv_box_position_invalid"] += f["invalid"]
        self.ring.append(f)
        return f

    def pair(self, preview_stamp_s, hide=frozenset()):
        """``ar`` block for a preview meta, or None when nothing was ever fed. ``hide``: hidden_ids of the
        memory current now -- boxes of tracks that left after their frame was kept are not drawn."""
        if not self.fed:
            return None
        best = None
        if isinstance(preview_stamp_s, (int, float)) and math.isfinite(preview_stamp_s) and self.ring:
            best = min(self.ring, key=lambda f: abs(f["stamp_s"] - preview_stamp_s))
        if best is None:
            self.counts["fpv_pair_none"] += 1
            return {"boxes": [], "state": "none"}
        dt_ms = (best["stamp_s"] - preview_stamp_s) * 1000.0
        if abs(dt_ms) > self.hold_ms:
            self.counts["fpv_pair_stale"] += 1
            return {"boxes": [], "state": "stale", "dt_ms": round(dt_ms, 1)}
        self.counts["fpv_pair_exact" if abs(dt_ms) < 5.0 else "fpv_pair_near"] += 1
        boxes = [dict(b) for b in best["boxes"] if b.get("id") not in hide]
        if len(boxes) < len(best["boxes"]):
            self.counts["fpv_pair_position_invalid"] += 1
        return {"boxes": boxes, "state": "ok", "dt_ms": round(dt_ms, 1),
                "frame_id": best["frame_id"], "depth_wh": best["depth_wh"], "hidden": best["hidden"]}
