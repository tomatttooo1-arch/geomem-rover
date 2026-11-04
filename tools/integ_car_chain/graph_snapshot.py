#!/usr/bin/env python3
"""graph_snapshot.py: the rig's camera-graph snapshot (rig/camera_topics.txt), polled until DDS discovery converged.

2026-10-15, after session_v3.sh v8rt: rig_inner.sh took ``ros2 topic info -v`` once, right after the driver came up,
and in all four arms DDS discovery had not converged yet (b1/r2: depth image without publisher; r1: the runner not
yet on the colour image; r2/b2: colour image "Unknown topic"), which voided three arms' validity clause 3.
This script repeats the same three calls until runner_qos_readout.graph_unmet says nothing is missing (every topic
known with a publisher; and, as the runner's switch says, its RELIABLE subscription on all three topics from one
participant / no RELIABLE subscription), or until --timeout-s. It always writes the last snapshot (same text format
as before: optional header line, then the CLI's output per topic, stderr included) and a wait record beside it:
  {"ready", "timed_out", "waited_s", "attempts", "expect", "expect_from", "unmet", "topics"}
runner_qos_readout.graph_check marks an arm invalid when the record says timed_out. Exit status 0 either way (the
rig goes on; the readout decides), 2 on bad arguments.

The expectation comes from the runner's own latched start-up facts when --runtime-info is given (camera_qos
"reliable:N" -> reliable, "sensor_data" -> best_effort; file absent = no runner -> none), else from --expect.

  python3 tools/integ_car_chain/graph_snapshot.py --topic D --topic C --topic I --out rig/camera_topics.txt \
      --wait-json rig/camera_topics_wait.json [--runtime-info runtime/runtime_info.json | --expect reliable]
      [--header "camera_frame=..."] [--timeout-s 30] [--interval-s 1]

NOT wired into tools/car_geo_producer/rig_inner.sh yet (that file is being changed on another branch); the line to
replace there is the ``for t in ...; do ros2 topic info -v "$t"; done; } > "$OUT/camera_topics.txt"`` block.
Python 3.8+.
"""
import argparse
import json
import os
import shlex
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.integ_car_chain.runner_qos_readout import (EXPECTED_TOPICS, expected_reliability,  # noqa: E402
                                                      graph_blocks, graph_unmet)


def expectation(runtime_info, expect, invalid=None):
    """(expect, where it came from). An unparseable runtime_info.json (not JSON, or not a JSON object: e.g. caught
    mid-write) is skipped instead of raising (10-15: the rig would otherwise get neither the snapshot nor the wait
    record): counted in ``invalid`` (a list, one note per skipped input) and treated as giving no expectation
    ("none": only the three publishers are waited for); the source string says so, and the readout's graph_check
    then voids an arm whose runner did ask for RELIABLE (expectation differs from the runner's switch)."""
    if runtime_info:
        try:
            with open(runtime_info) as f:
                info = json.load(f)
        except OSError:
            return "none", "no %s (no runner)" % runtime_info
        except ValueError as exc:                     # JSONDecodeError and UnicodeDecodeError are both ValueError
            note = "%s: invalid JSON (%s)" % (runtime_info, exc)
            if invalid is not None:
                invalid.append(note)
            return "none", "%s, skipped" % note
        if not isinstance(info, dict):
            note = "%s: invalid JSON (top level %s, not an object)" % (runtime_info, type(info).__name__)
            if invalid is not None:
                invalid.append(note)
            return "none", "%s, skipped" % note
        return expected_reliability(info.get("camera_qos")), "%s camera_qos=%s" % (runtime_info, info.get("camera_qos"))
    return expect, "--expect"


def take(cmd, topics, call_timeout_s):
    """One snapshot: the CLI once per topic, stdout and stderr together in topic order (as rig_inner.sh's 2>&1)."""
    parts = []
    for t in topics:
        try:
            r = subprocess.run(cmd + [t], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=call_timeout_s)
            parts.append(r.stdout.decode("utf-8", "replace"))
        except subprocess.TimeoutExpired as exc:
            parts.append((exc.stdout or b"").decode("utf-8", "replace") + "\n[graph_snapshot: %s timed out]\n" % t)
    return "".join(p if p.endswith("\n") or not p else p + "\n" for p in parts)


def poll(topics, expect, timeout_s, interval_s, cmd, call_timeout_s=15.0, clock=time.monotonic, sleep=time.sleep):
    """-> (last text, record). Polls at least once."""
    t0 = clock()
    n = 0
    while True:
        n += 1
        text = take(cmd, topics, call_timeout_s)
        unmet = graph_unmet(graph_blocks(text), expect, expected_topics=len(topics))
        waited = clock() - t0
        if not unmet or waited >= timeout_s:
            return text, {"ready": not unmet, "timed_out": bool(unmet), "waited_s": round(waited, 2),
                          "attempts": n, "expect": expect, "unmet": unmet, "topics": list(topics)}
        sleep(max(0.0, min(interval_s, timeout_s - waited)))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--topic", action="append", required=True, help="in order: depth image, colour image, CameraInfo")
    ap.add_argument("--out", required=True)
    ap.add_argument("--wait-json", required=True)
    ap.add_argument("--runtime-info", help="the runner's runtime_info.json (absent file = no runner)")
    ap.add_argument("--expect", choices=("reliable", "best_effort", "none"), default="none")
    ap.add_argument("--header", default="", help="first line of the snapshot (rig_inner.sh: camera_frame=...)")
    ap.add_argument("--timeout-s", type=float, default=30.0)
    ap.add_argument("--interval-s", type=float, default=1.0)
    ap.add_argument("--call-timeout-s", type=float, default=15.0)
    ap.add_argument("--cmd", default="ros2 topic info -v", help="the CLI, topic appended (tests: a fake)")
    a = ap.parse_args(argv)
    if len(a.topic) != EXPECTED_TOPICS:
        ap.error("exactly %d --topic (depth image, colour image, depth CameraInfo), got %d"
                 % (EXPECTED_TOPICS, len(a.topic)))
    invalid = []
    expect, source = expectation(a.runtime_info, a.expect, invalid)
    if invalid:
        print("graph_snapshot: skipped %d invalid JSON input(s): %s" % (len(invalid), "; ".join(invalid)),
              file=sys.stderr)
    text, rec = poll(a.topic, expect, a.timeout_s, a.interval_s, shlex.split(a.cmd), a.call_timeout_s)
    rec["expect_from"] = source
    with open(a.out, "w") as f:
        f.write((a.header + "\n" if a.header else "") + text)
    with open(a.wait_json, "w") as f:
        json.dump(rec, f, indent=1)
        f.write("\n")
    print("graph snapshot %s after %.1f s (%d attempts, expect %s)%s" % (
        "ready" if rec["ready"] else "TIMED OUT", rec["waited_s"], rec["attempts"], expect,
        "" if rec["ready"] else ": " + "; ".join(rec["unmet"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
