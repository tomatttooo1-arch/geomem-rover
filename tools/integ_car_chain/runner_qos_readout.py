#!/usr/bin/env python3
"""runner_qos_readout.py RUN_DIR [RUN_DIR ...] [--json OUT]: readout of the runner camera-QoS A/B arms
(session_v3.sh v8rt, 2026-10-15; prereg docs/design/integ_car_chain_v3_20261011/PREREG_RUNNER_CAMERA_QOS_20261015.md).

RUN_DIR = one pulled Nano run (runs/rt_<T>_<arm>: rc.txt, runtime/, rig/). Everything inside the rig's measurement
window (rig/window.txt, wall seconds; the runner's events by their "time" = ROS time = wall on hardware):
  validity   rc (rig, runtime), runner camera_qos from runtime_info.json and rx_runner.json, RELIABLE subscriptions in
             the ROS graph (rig/camera_topics.txt, taken after the runner subscribed and before the producer started;
             counted per topic and by participant, since rclpy nodes show as _NODE_NAME_UNKNOWN_ there), producer depth_qos,
             driver: device-stamp gaps and arrived-not-published (bx_readout), the 32 MB segment (env.txt);
             graph_check (2026-10-15, after v8rt): the snapshot must hold 3 topic sections (else "error" and exit 3,
             never fewer counted), every topic known with a publisher, the runner's RELIABLE subscriptions as the
             switch says (graph_unmet), and the poller's wait record (rig/camera_topics_wait.json,
             tools/integ_car_chain/graph_snapshot.py) not timed out -- otherwise valid False with the reasons
  q1 input   runner per-topic missing vs the driver's per-frame record (bx_readout receiver rx_runner) and its DDS
             message-lost count in the window (missing - lost = dropped inside the reader: keep-last overflow while the
             executor did not take messages); late arrivals (> 200 ms after the stamp) and the largest stamp step
             (a reliable stream held behind one lost sample until the resend); the producer and the best-effort
             witness rx_chain beside it
  q2 output  runner frame_processed rows in the window: count, geometry results, frame_age_ms (runner use - producer
             acquisition) p50/p95/p99/max, rows over 200 ms, age-gate refusals
  q3 cpu     runner / producer / driver process CPU (% of one core) and machine busy cores (summarize_chain_v2)
Needs tools/integ_car_chain/cam_drop/bx_readout.py (run as a subprocess on RUN_DIR/rig).
"""
import collections
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

BX = os.path.join(ROOT, "tools", "integ_car_chain", "cam_drop", "bx_readout.py")


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else None


def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def jsonl(path):
    out = []
    try:
        with open(path) as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    continue
    except OSError:
        pass
    return out


def read_text(path):
    with open(path) as f:
        return f.read()


def rc_values(path):
    out = {}
    try:
        for line in read_text(path).splitlines():
            m = re.match(r"^(\w+_rc)=(-?\d+)", line.strip())
            if m:
                out[m.group(1)] = int(m.group(2))
    except OSError:
        pass
    return out


EXPECTED_TOPICS = 3          # depth image, colour image, depth CameraInfo (rig_inner.sh, in this order)
RUNNER_NODE = "fpga_runtime"


class GraphSnapshotError(ValueError):
    """The snapshot does not hold the expected number of topic sections: refuse instead of counting fewer."""


def graph_blocks(text, node=RUNNER_NODE):
    """``ros2 topic info -v`` text (one call per topic, one after another) -> one block per topic section, in order.
    A section starts at "Type: ..." or at "Unknown topic '...'" (the CLI's answer for a topic nobody has announced
    yet, 10-15 v8rt r2/b2): an unknown topic is a block too (``unknown`` True, no endpoints), so a snapshot taken
    before DDS discovery converged keeps its three positions instead of silently shrinking to two."""
    marks = list(re.finditer(r"(?m)^(?:Type: (.*)|Unknown topic '([^']*)'.*)$", text))
    blocks = []
    for i, m in enumerate(marks):
        part = text[m.end():marks[i + 1].start() if i + 1 < len(marks) else len(text)]
        if m.group(2) is not None:
            blocks.append({"type": None, "unknown": True, "topic": m.group(2), "publishers": [], "subscriptions": [],
                           "reliable_subscriptions": 0, "node": []})
            continue
        ents = re.split(r"(?m)^Node name: ", part)[1:]
        pubs, subs, mine = [], [], []
        for e in ents:
            name = e.splitlines()[0].strip()
            kind = re.search(r"Endpoint type: (\w+)", e)
            rel = re.search(r"Reliability: (\w+)", e)
            gid = re.search(r"GID: ([0-9a-f.]+)", e)
            kind, rel = (kind.group(1) if kind else None), (rel.group(1) if rel else None)
            prefix = ".".join(gid.group(1).split(".")[:12]) if gid else None
            if kind == "PUBLISHER":
                pubs.append(rel)
            else:
                subs.append({"reliability": rel, "gid_prefix": prefix, "node": name})
                if name == node:
                    mine.append(rel)
        blocks.append({"type": m.group(1).strip(), "unknown": False, "topic": None, "publishers": pubs,
                       "subscriptions": subs,
                       "reliable_subscriptions": sum(1 for x in subs if x["reliability"] == "RELIABLE"), "node": mine})
    return blocks


def graph_reliability(path, node=RUNNER_NODE):
    """rig/camera_topics.txt (``ros2 topic info -v`` per topic, one after another: depth image, colour image, depth
    CameraInfo) -> [{"type", "unknown", "topic", "publishers": [reliability...],
    "subscriptions": [{"reliability", "gid_prefix", "node"}], "reliable_subscriptions": n,
    "node": [reliability of NODE's subscriptions, if the graph names it]}] (graph_blocks).
    On the board, rclpy endpoints are listed as _NODE_NAME_UNKNOWN_ (10-15 record), so the name is a bonus only:
    gid_prefix = the first 12 bytes of the endpoint GID = the DDS participant, shared by all subscriptions of one
    process."""
    try:
        text = read_text(path)
    except OSError:
        return None
    return graph_blocks(text, node)


def graph_summary(blocks, expected_topics=None):
    """Per topic the number of RELIABLE subscriptions, and the participants (gid prefixes) that subscribe RELIABLE
    on every listed topic. When camera_topics.txt is taken (runner subscribed, producer not started yet, every other
    subscriber -- cam_probe, rx_lite, live_tf -- best effort), the r arm must show one such participant on all three
    topics and the b arm none. EXPECTED_TOPICS given (the readout: 3): a different number of topic sections raises
    GraphSnapshotError (10-15 v8rt: two sections were read as if the third did not exist)."""
    if expected_topics is not None and len(blocks or []) != expected_topics:
        raise GraphSnapshotError("camera_topics.txt holds %d topic sections, expected %d"
                                 % (len(blocks or []), expected_topics))
    if not blocks:
        return None
    sets = [set(x["gid_prefix"] for x in b["subscriptions"] if x["reliability"] == "RELIABLE") for b in blocks]
    common = set.intersection(*sets) if sets else set()
    return {"topics": len(blocks), "unknown_topics": sum(1 for b in blocks if b.get("unknown")),
            "publishers": [len(b["publishers"]) for b in blocks],
            "reliable_subscriptions": [b["reliable_subscriptions"] for b in blocks],
            "reliable_on_all_topics": sorted(p for p in common if p)}


def expected_reliability(camera_qos):
    """runtime_info camera_qos label -> what the runner's camera subscriptions must look like in the graph:
    "reliable:N" -> "reliable", "sensor_data" -> "best_effort", nothing (no runner) -> "none"."""
    if not camera_qos:
        return "none"
    if str(camera_qos).startswith("reliable"):
        return "reliable"
    if camera_qos == "sensor_data":
        return "best_effort"
    raise ValueError("unknown runner camera_qos %r" % (camera_qos,))


def graph_unmet(blocks, expect, expected_topics=EXPECTED_TOPICS):
    """What the snapshot still lacks (empty list = converged and consistent with the switch). Shared by the
    poller (graph_snapshot.py: keep polling while this is not empty) and the readout (graph_check).
      every topic: known to the graph and at least one publisher (the driver);
      expect "reliable" (r arm): exactly one RELIABLE subscription per topic, all three from one participant
        (the runner; prereg clause 3);
      expect "best_effort" (b arm): no RELIABLE subscription on any topic;
      expect "none" (no runner in this chain): publishers only."""
    if expect not in ("reliable", "best_effort", "none"):
        raise ValueError("expect must be reliable, best_effort or none, not %r" % (expect,))
    blocks = blocks or []
    unmet = []
    if len(blocks) != expected_topics:
        unmet.append("%d topic sections, expected %d" % (len(blocks), expected_topics))
    for i, b in enumerate(blocks):
        if b.get("unknown"):
            unmet.append("topic %d (%s) unknown to the graph" % (i + 1, b.get("topic")))
        elif not b["publishers"]:
            unmet.append("topic %d (%s) has no publisher" % (i + 1, b["type"]))
    if blocks and not any(b.get("unknown") for b in blocks):
        rel = [b["reliable_subscriptions"] for b in blocks]
        if expect == "reliable":
            common = graph_summary(blocks)["reliable_on_all_topics"]
            if rel != [1] * len(blocks) or len(common) != 1:
                unmet.append("runner RELIABLE subscriptions per topic %s, one participant on all topics: %d "
                             "(expected 1 each, from one participant)" % (rel, len(common)))
        elif expect == "best_effort" and any(rel):
            unmet.append("RELIABLE subscriptions per topic %s on a best-effort arm (expected none)" % (rel,))
    elif expect == "reliable":
        unmet.append("runner RELIABLE subscriptions not checkable: a topic is unknown to the graph")
    return unmet


def graph_check(rig, camera_qos):
    """Validity of the graph snapshot of one arm: rig/camera_topics.txt (+ rig/camera_topics_wait.json written by
    graph_snapshot.py; absent on snapshots taken before 2026-10-15's poller). Never raises: a wrong number of topic
    sections is reported as "error" with valid False."""
    out = {"valid": False, "reasons": [], "expect": None, "wait": load_json(os.path.join(rig, "camera_topics_wait.json"))}
    blocks = graph_reliability(os.path.join(rig, "camera_topics.txt"))
    if blocks is None:
        out["reasons"].append("no camera_topics.txt")
        return out
    try:
        graph_summary(blocks, expected_topics=EXPECTED_TOPICS)
    except GraphSnapshotError as exc:
        out["error"] = str(exc)       # main() exits non-zero on it; graph_unmet below names it as a reason too
    try:
        out["expect"] = expected_reliability(camera_qos)
    except ValueError as exc:
        out["reasons"].append(str(exc))
        return out
    out["reasons"].extend(graph_unmet(blocks, out["expect"]))
    wait = out["wait"]
    if wait is None:
        out["note"] = "no wait record (snapshot not taken by graph_snapshot.py)"
    elif wait.get("timed_out"):
        out["reasons"].insert(0, "snapshot timed out after %s s waiting for DDS discovery: %s"
                              % (wait.get("waited_s"), "; ".join(wait.get("unmet") or []) or "?"))
    elif wait.get("expect") not in (None, out["expect"]):
        out["reasons"].append("snapshot waited for %s, runtime_info says %s" % (wait.get("expect"), out["expect"]))
    out["valid"] = not out["reasons"]
    return out


def runner_output(events, w0, w1, limit_ms=200.0):
    fp = [e for e in events if e.get("event") == "frame_processed" and w0 <= float(e.get("time") or 0) <= w1]
    geo = [(e.get("geometry") or {}) for e in fp]
    ages = [g["frame_age_ms"] for g in geo if isinstance(g.get("frame_age_ms"), (int, float))]
    res = collections.Counter(str(g.get("result")).split(":")[0] for g in geo)
    gate = collections.Counter(str(g.get("age_gate")) for g in geo)
    others = collections.Counter(e.get("event") for e in events
                                 if e.get("event") != "frame_processed" and w0 <= float(e.get("time") or 0) <= w1)
    drops = [e.get("dropped_frames") for e in fp if isinstance(e.get("dropped_frames"), int)]
    return {"frame_processed": len(fp), "geometry_results": dict(res), "age_gate": dict(gate),
            "frame_age_ms": {"n": len(ages), "p50": pct(ages, 0.5), "p95": pct(ages, 0.95), "p99": pct(ages, 0.99),
                             "max": max(ages) if ages else None},
            "over_%d_ms" % limit_ms: sum(1 for a in ages if a > limit_ms),
            "dropped_frames_delta": (drops[-1] - drops[0]) if len(drops) > 1 else None,
            "other_events_in_window": dict(others)}


def receiver_topics(bx, name):
    entry = (bx.get("receivers") or {}).get(name)
    if not entry:
        return None
    out = {}
    for topic, t in entry["topics"].items():
        vp, lo = t.get("vs_publisher"), t.get("dds_message_lost")
        out[topic] = {"received": t.get("received_in_window"),
                      "published": vp["published"] if vp else None,
                      "missing": vp["missing"] if vp else (t.get("gap_rule") or {}).get("missed"),
                      "missing_pct": vp["missing_pct"] if vp else (t.get("gap_rule") or {}).get("missed_pct"),
                      "missing_by": "publisher record" if vp else "stamp-gap rule",
                      "dds_lost_in_window": lo.get("in_window") if lo and lo.get("supported") else None}
    return {"qos": entry.get("qos"), "topics": out}


def late_arrivals(path, w0, w1, limit_ms=200.0):
    """rx_runner.csv rows in the window per topic: count whose arrival (wall) is more than LIMIT after the header
    stamp, and the largest stamp step (a reliable stream held back behind one lost sample shows both)."""
    rows = collections.defaultdict(list)
    try:
        with open(path) as f:
            next(f, None)
            for line in f:
                p = line.rstrip("\n").split(",")
                if len(p) >= 3:
                    rows[p[0]].append((int(p[1]), int(p[2])))
    except (OSError, ValueError):
        return None
    out = {}
    for topic, r in rows.items():
        r = [(s, w) for s, w in r if w0 * 1e9 <= s <= w1 * 1e9]
        st = sorted(s for s, _ in r)
        out[topic] = {"rows": len(r), "late_over_%d_ms" % limit_ms: sum(1 for s, w in r if (w - s) / 1e6 > limit_ms),
                      "max_stamp_to_arrival_ms": round(max((w - s) / 1e6 for s, w in r), 1) if r else None,
                      "max_stamp_step_ms": round(max((b - a) / 1e6 for a, b in zip(st, st[1:])), 1) if len(st) > 1 else None}
    return out


def readout(run):
    rig = os.path.join(run, "rig")
    w0, w1 = (float(x) for x in read_text(os.path.join(rig, "window.txt")).split())
    fd, tmp = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    try:
        subprocess.run([sys.executable, BX, rig, "--json", tmp], check=True, stdout=subprocess.DEVNULL)
        bx = load_json(tmp) or {}
    finally:
        os.unlink(tmp)
    info = load_json(os.path.join(run, "runtime", "runtime_info.json")) or {}
    rxs = load_json(os.path.join(rig, "rx_runner.json")) or {}
    prod = load_json(os.path.join(rig, "producer_summary.json")) or {}
    env = ""
    try:
        env = read_text(os.path.join(rig, "env.txt"))
    except OSError:
        pass
    shm = re.search(r"shm_segment_bytes_max=(\d+)", env)
    publisher = bx.get("publisher") or {}
    from tools.integ_car_chain.summarize_chain_v2 import cpu_window
    cpu = cpu_window(rig)
    procs = cpu.get("pct_one_core") or {}
    return {
        "run": os.path.basename(os.path.normpath(run)), "window_s": round(w1 - w0, 3),
        "validity": {
            "rc": rc_values(os.path.join(run, "rc.txt")),
            "runtime_info_camera_qos": [info.get("camera_qos"), info.get("camera_qos_switch"), info.get("sensor_source")],
            "rx_runner_camera_qos": (rxs.get("_receiver") or {}).get("camera_qos"),
            "graph": graph_reliability(os.path.join(rig, "camera_topics.txt")),
            "graph_summary": graph_summary(graph_reliability(os.path.join(rig, "camera_topics.txt"))),
            "graph_check": graph_check(rig, info.get("camera_qos")),
            "producer_depth_qos": prod.get("depth_qos"),
            "driver": {s: {k: (publisher.get(s) or {}).get(k) for k in
                           ("published", "arrived_not_published", "arrival_to_publish_ms")}
                       for s in ("depth", "color")},
            "driver_device_stamp_missed": {s: ((publisher.get(s) or {}).get("device_stamp_gap_rule") or {}).get("missed")
                                           for s in ("depth", "color")},
            "driver_start_tries": bx.get("driver_start_tries"),
            "shm_segment_bytes_max": int(shm.group(1)) if shm else None},
        "q1_input": {"runner": receiver_topics(bx, "rx_runner"),
                     "runner_late": late_arrivals(os.path.join(rig, "rx_runner.csv"), w0, w1),
                     "producer": receiver_topics(bx, "producer"),
                     "rx_chain_witness": receiver_topics(bx, "rx_chain")},
        "q2_output": runner_output(jsonl(os.path.join(run, "runtime", "events.jsonl")), w0, w1),
        "q3_cpu": {"runtime_pct_one_core": procs.get("runtime"), "producer_pct_one_core": procs.get("producer"),
                   "driver_pct_one_core": procs.get("replay"), "machine_busy_cores": cpu.get("machine_busy_cores_of_6"),
                   "window_s": cpu.get("window_s")}}


def main(argv):
    out_path = None
    if "--json" in argv:
        i = argv.index("--json")
        out_path = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    res = [readout(r) for r in argv]
    text = json.dumps(res, indent=1)
    if out_path:
        with open(out_path, "w") as f:
            f.write(text + "\n")
    print(text)
    bad = [(r["run"], r["validity"]["graph_check"]) for r in res]
    for run, g in bad:
        if not g["valid"]:
            sys.stderr.write("%s: graph snapshot INVALID: %s\n" % (run, "; ".join(g["reasons"])))
    errors = [run for run, g in bad if g.get("error")]
    if errors:
        sys.stderr.write("ERROR: camera_topics.txt does not hold %d topic sections in %s\n"
                         % (EXPECTED_TOPICS, ", ".join(errors)))
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
