#!/usr/bin/env python3
"""What the PC side saw of the runner geometry age gate, from a pc.fpgarec recording.

  pc_gate.py <pc.fpgarec>  -> one JSON line: frame kinds, STATE task.geometry_age_refused
  (first/last), task.geometry_age_last_refused, number of forwarded geometry_stale EVENTs and of
  all age-gate refusal EVENTs by reason (geometry_age_unknown added 2026-10-12).
Python 3.8 compatible.
"""
import collections
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from pc import fpgac  # noqa: E402

AGE_EVENTS = ("geometry_stale", "geometry_future", "geometry_clock_mismatch", "geometry_age_unknown")


def main(path):
    _, rows = fpgac.read_fpgarec(path)
    kinds = collections.Counter()
    refused, ev, last = [], [], None
    by_reason = collections.Counter()
    for _, raw in rows:
        f = fpgac.decode_one(raw)
        kinds[f.name] += 1
        try:
            obj = json.loads(f.body)
        except (ValueError, TypeError, UnicodeDecodeError):
            continue
        if f.name == "STATE":
            refused.append(obj["task"].get("geometry_age_refused"))
            last = obj["task"].get("geometry_age_last_refused")
        if f.name == "EVENT" and isinstance(obj.get("event"), dict) and obj["event"].get("event") in AGE_EVENTS:
            ev.append(obj["event"].get("refused_total"))
            by_reason[obj["event"]["event"]] += 1
    return {"frames": dict(kinds),
            "state_geometry_age_refused_first_last": [refused[0] if refused else None,
                                                      refused[-1] if refused else None],
            "state_last_refused": last, "event_geometry_stale": by_reason["geometry_stale"],
            "event_age_refusals": len(ev), "event_age_refusals_by_reason": dict(by_reason),
            "event_refused_total_last": ev[-1] if ev else None}


if __name__ == "__main__":
    print(json.dumps(main(sys.argv[1])))
