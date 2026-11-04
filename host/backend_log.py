"""Inference backend metadata in runtime events: written in full once, then by reference (10-29).

Every runtime event (runtime.jsonl and /fpga/status) used to carry the whole reply metadata of the last
inference -- model, sha256, the 80-name class table... (~1.5 KB) -- so 24 min of a car run wrote 36 MB,
60 % of it the same block again and again, and the hub parsed it on every /fpga/status message.

BackendLog.field(meta, now) returns what goes into the event's ``inference_backend``:
  * the full metadata (as before) the first time, whenever its stable part changes (another model,
    detector, device, class table...) and again every ``refresh_s`` seconds (a reader that starts in the
    middle of a file, or a late /fpga/status subscriber, finds a full row within that time);
  * otherwise a short row: the per-frame keys (ok, frame_id, timestamp, inference_seconds) and every
    other scalar key (detector, device), plus ``backend_ref`` = the frame_id of the last full row.
    Nested values (``execution``) are left out -- they are equal to the full row's.
None stays None.

Readers that need the nested part on every row use BackendExpander / expand_rows (restores the full
metadata from the last full row before it, in file order).
"""

VOLATILE = ("ok", "frame_id", "timestamp", "inference_seconds")
REF_KEY = "backend_ref"
REFRESH_S = 60.0


def _stable(meta):
    return {k: v for k, v in meta.items() if k not in VOLATILE}


class BackendLog:
    def __init__(self, refresh_s=REFRESH_S):
        self.refresh_s = float(refresh_s)
        self.stable = None              # stable part of the last full row written
        self.full_frame_id = None
        self.full_at = None
        self.seen = None                # the metadata object last compared (the client keeps one per reply)

    def field(self, meta, now):
        if not meta:
            return meta
        changed = False
        if meta is not self.seen:
            self.seen = meta
            changed = self.stable is None or _stable(meta) != self.stable
        if changed or self.full_at is None or not 0 <= now - self.full_at < self.refresh_s:
            self.stable, self.full_frame_id, self.full_at = _stable(meta), meta.get("frame_id"), now
            return meta
        short = {k: v for k, v in meta.items() if not isinstance(v, (dict, list, tuple))}
        short[REF_KEY] = self.full_frame_id
        return short


class BackendExpander:
    """Restores short rows (stateful; feed rows in the order they were written / published)."""

    def __init__(self):
        self.full = None

    def expand(self, backend):
        if not isinstance(backend, dict):
            return backend
        if REF_KEY not in backend:
            self.full = backend
            return backend
        if self.full is None:
            return backend               # no full row seen yet: left short
        out = _stable(self.full)
        out.update((k, v) for k, v in backend.items() if k != REF_KEY)
        return out

    def row(self, row):
        if isinstance(row, dict) and "inference_backend" in row:
            row["inference_backend"] = self.expand(row["inference_backend"])
        return row


def expand_rows(rows):
    """Runtime JSONL rows (dicts, file order) with every short inference_backend restored in place."""
    expander = BackendExpander()
    for row in rows:
        expander.row(row)
    return rows
