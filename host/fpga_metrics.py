"""Per-frame FPGA (KV260 PL) numbers for the console's live panel (10-29, display only).

Nothing here is measured anew and no KV260 code changes: every number is already in the answers the Nano gets
for each depth frame.

  geometry core   kv_link RESP times (host/kv_link.py TIMING_FIELDS, KV clock): ``core_device`` = PS start of the
                  PL frame core -> its done interrupt; RESP counts [accepted, invalid, clipped] (the PL's own
                  per-pixel counters, hw/m7/frame_depth.hpp: every pixel lands in exactly one) -> pixels per frame
  evidence core   the frame's ROI answer (host/kv_link.py RoiOutcome): times_ns["device"], one row per object box
  fusion core     the ROLL_ACK of the frame's INTEGRATE (host/kv_rolling.py decode_roll_ack times_ns["device"]) and
                  the number of map cells it changed (the navigation delta)

``frame_fields`` runs in the geometry producer once per frame (its dict goes into /fpga/kv_link/frame_stats as
"fpga"); ``summarize`` runs in the hub over the last frame_stats rows (STATE perf.fpga). A core that did not run
on the PL, or did not run in a frame, gives None -- never a stand-in value.

Python 3.8, no ROS / numpy imports.
"""


def is_pl(core):
    """True for a PL core name: "pl" (TELEM / hello) or "pl:<device>:<wait>" (the geometry RESP core string)."""
    return isinstance(core, str) and (core == "pl" or core.startswith("pl:"))


def _num(v):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    v = float(v)
    return v if v == v and v not in (float("inf"), float("-inf")) else None


def _ms(ns):
    v = _num(ns)
    return None if v is None or v <= 0 else v / 1e6


def frame_fields(client_last=None, roi=None, roll_times_ns=None, roll_core=None, roll_changes=None,
                 nav_counts=None):
    """One frame -> {geo_core, pixels, roi_core, roi_device_ms, roi_boxes, roll_core, roll_device_ms,
    roll_changes}.

    ``client_last``: the geometry client's ``last`` of THIS frame (kv_ns, core) or None; ``roi``: the frame's
    RoiOutcome or None (no boxes sent); ``roll_times_ns``: the ROLL_ACK times of this frame's INTEGRATE (None when
    the frame was not fused); ``roll_core``: the KV's rolling core name from its hello; ``nav_counts``: the
    frame's {valid_grid_samples, invalid_depth_or_projection, outside_grid}."""
    last = client_last if isinstance(client_last, dict) else {}
    out = {"geo_core": last.get("core") if isinstance(last.get("core"), str) else None,
           "pixels": None, "roi_core": None, "roi_device_ms": None, "roi_boxes": None,
           "roll_core": roll_core if isinstance(roll_core, str) else None, "roll_device_ms": None,
           "roll_changes": None}
    if isinstance(nav_counts, dict):
        parts = [nav_counts.get(k) for k in ("valid_grid_samples", "invalid_depth_or_projection", "outside_grid")]
        if all(isinstance(p, int) and not isinstance(p, bool) and p >= 0 for p in parts):
            out["pixels"] = sum(parts)
    if roi is not None:
        core = getattr(roi, "roi_core", None)
        out["roi_core"] = core if isinstance(core, str) and core else None
        if getattr(roi, "ok", False):
            out["roi_device_ms"] = _ms((getattr(roi, "times_ns", None) or {}).get("device"))
            rows = getattr(roi, "rows", None)
            out["roi_boxes"] = len(rows) if isinstance(rows, (list, tuple)) else None
    if isinstance(roll_times_ns, dict):
        out["roll_device_ms"] = _ms(roll_times_ns.get("device"))
        if isinstance(roll_changes, int) and not isinstance(roll_changes, bool) and roll_changes >= 0:
            out["roll_changes"] = roll_changes
    return out


def _median(values):
    if not values:
        return None
    s = sorted(values)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def summarize(rows):
    """Hub: the last frame_stats rows (oldest first) -> STATE perf.fpga.

    Times are medians over the rows where that core ran on the PL; counts are the newest such row's; ``*_frames``
    says in how many of the rows the core ran (the evidence core runs only when remembered objects are in view)."""
    geo, roi, roll, rtt = [], [], [], []
    pixels = roi_boxes = roll_changes = None
    for r in rows:
        if not isinstance(r, dict):
            continue
        f = r.get("fpga") if isinstance(r.get("fpga"), dict) else {}
        kv = r.get("kv_times_ms") if isinstance(r.get("kv_times_ms"), dict) else {}
        if is_pl(f.get("geo_core")):
            d = _num(kv.get("core_device"))
            if d is not None and d > 0:
                geo.append(d)
                if isinstance(f.get("pixels"), int):
                    pixels = f["pixels"]
        t = _num(r.get("rtt_ms"))
        if t is not None and t > 0:
            rtt.append(t)
        if is_pl(f.get("roi_core")) and _num(f.get("roi_device_ms")) is not None:
            roi.append(float(f["roi_device_ms"]))
            if isinstance(f.get("roi_boxes"), int):
                roi_boxes = f["roi_boxes"]
        if is_pl(f.get("roll_core")) and _num(f.get("roll_device_ms")) is not None:
            roll.append(float(f["roll_device_ms"]))
            if isinstance(f.get("roll_changes"), int):
                roll_changes = f["roll_changes"]
    return {"geo_device_ms": _median(geo), "geo_frames": len(geo), "pixels": pixels,
            "roi_device_ms": _median(roi), "roi_frames": len(roi), "roi_boxes": roi_boxes,
            "roll_device_ms": _median(roll), "roll_frames": len(roll), "roll_changes": roll_changes,
            "rtt_ms": _median(rtt), "rows": sum(1 for r in rows if isinstance(r, dict))}
