#!/usr/bin/env python3
"""bx_readout.py RUN_DIR [--json OUT]: readout of one bx.sh arm (camera-drop layer experiments, 2026-10-13).

Layers, all inside the window of window.txt (by header stamp, wall axis):
  device -> driver callback   drv_frames.csv rows: gaps in the device stamp (and hardware frame number)
  driver callback -> publish  rows that arrived but carry no publish time
  publish -> receiver         published stamps that a receiver (rx_*.csv, cam_probe.csv) does not have
The publisher record is replay.csv (replay camera) or drv_frames.csv (driver). Receivers are matched by
stamp in whole microseconds; "extra" = received stamps the publisher record does not have (must be 0, the
check of the matching itself). Without a publisher record only the stamp-gap rule is available (a step over
1.5 median periods is round(step/period)-1 frames, as cam_probe.summarize).
Small-message control: for the image stamps a receiver misses, how many of the same receiver's camera_info
rows with that stamp are present.
sysmon.jsonl: per-process and per-thread CPU in the window, cpu frequency, machine busy cores, UDP deltas.
Repo copy (2026-10-14) of the readout behind board/readout/a_neg1.json and l_a2.json; no hard-coded paths.
DDS message-lost (2026-10-14 b): rx_lite summaries carry the rmw "message lost" event per topic (see rx_lite.py);
each receiver topic then gets "dds_message_lost": whole-run total, events, the count whose event wall time falls
inside the window, and, with a publisher record, missing minus lost-in-window (lost ~ missing: the sample was
lost in transport; lost 0 with frames missing: the receiver lost them itself, e.g. keep-last overflow).
Receivers without the field (old arms, cam_probe) get null = not recorded, never 0. "dds_lost_summary" lists
all of them in one table.
Chain runs (rig_inner.sh with RX_LITE=1, the l_chain32 arm): the rig directory has window.txt, rx_chain.csv,
cam_probe.csv, drv_frames.csv (LIVE_DRV_LOG=1) and the producer's input_log.csv (INPUT_LOG=1); the producer's
depth arrivals (put and rate_skipped rows: both arrived) are read as receiver "producer".
"""
import collections
import csv
import json
import os
import sys


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * len(v)))] if v else None


def gap_rule(stamps_us):
    s = sorted(set(stamps_us))
    steps = [b - a for a, b in zip(s, s[1:])]
    if len(steps) < 8:
        return {"n": len(s)}
    period = pct(steps, 0.5)
    gaps = [x for x in steps if x > 1.5 * period]
    runs = collections.Counter(max(1, int(round(x / period)) - 1) for x in gaps)
    missed = sum(k * v for k, v in runs.items())
    return {"n": len(s), "period_ms": round(period / 1000.0, 3), "missed": missed,
            "expected": len(s) + missed, "missed_pct": round(100.0 * missed / (len(s) + missed), 2),
            "run_lengths": dict(sorted(runs.items()))}


def us_from_sec_text(t):
    """'1791868606.782171' -> integer microseconds without float rounding."""
    if "." not in t:
        return int(t) * 1000000
    a, b = t.split(".")
    b = (b + "000000")[:6]
    return int(a) * 1000000 + int(b) if not a.startswith("-") else int(a) * 1000000 - int(b)


def load_driver_csv(path):
    """drv_frames.csv -> {"depth": [row...], "color": [...]}, row = dict(us, dev_us, idx, hw, published)."""
    out = {"depth": [], "color": []}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        cols = r.fieldnames or []
        for row in r:
            for s in ("depth", "color"):
                g = row.get(s + "_global_ts_sec", "")
                if not g:
                    continue
                hw = row.get(s + "_hardware_frame_number", "")
                a2p = row.get(s + "_arrival_to_publish_steady_us", "")
                out[s].append({"us": us_from_sec_text(g), "dev_us": us_from_sec_text(row[s + "_device_ts_sec"]),
                               "idx": int(row[s + "_sdk_frame_index"]), "hw": int(hw) if hw else None,
                               "published": a2p != "", "a2p_us": int(a2p) if a2p else None,
                               "sdk_delay_us": int(row[s + "_sdk_delay_from_global_us"] or 0)})
    out["_columns"] = cols
    return out


def load_rx(path):
    """receiver CSV -> {topic: [(stamp_us, wall_or_ros_ns)]} (rx_lite and cam_probe share the first three columns)."""
    out = collections.defaultdict(list)
    with open(path, newline="") as f:
        r = csv.reader(f)
        next(r, None)
        for row in r:
            if len(row) < 3:
                continue
            out[row[0]].append((int(row[1]) // 1000, int(row[2])))
    return out


def load_producer_input_log(path, topic="/camera/depth/image_raw"):
    """host.ros_geo_producer --input-log CSV (stamp_ns, arrival_raw_ns, disposition) -> {topic: [(stamp_us, 0)]}:
    every row is a depth message that reached the producer's callback (put or rate_skipped)."""
    out = collections.defaultdict(list)
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                out[topic].append((int(row["stamp_ns"]) // 1000, 0))
            except (KeyError, TypeError, ValueError):
                continue
    return out


def lost_readout(lo, w0, w1, missing=None):
    """rx_lite's per-topic "lost" record -> the readout entry (None = not recorded)."""
    if lo is None:
        return None
    if not lo.get("supported", True):
        return {"supported": False, "error": lo.get("error")}
    log = lo.get("log") or []
    inw = sum(max(int(ch), 0) for wall_ns, ch, _n in log if w0 <= wall_ns / 1e9 <= w1)
    out = {"supported": True, "whole_run_total": lo.get("total_count"), "events": lo.get("events"),
           "in_window": inw, "log_rows": len(log),
           "log_complete": len(log) == lo.get("events")}
    if missing is not None:
        out["missing_minus_lost_in_window"] = missing - inw
    return out


def runs_of(missing_flags):
    runs, n = collections.Counter(), 0
    for m in missing_flags:
        if m:
            n += 1
        elif n:
            runs[n] += 1
            n = 0
    if n:
        runs[n] += 1
    return dict(sorted(runs.items()))


def stream_of(topic):
    return "color" if ("color" in topic or topic.endswith("/rgb")) else "depth"


def sysmon(path, w0, w1):
    recs = [json.loads(x) for x in open(path) if x.strip()]
    inw = [r for r in recs if "freq" in r and w0 <= r["t"] <= w1]
    out = {"samples_in_window": len(inw)}
    if len(inw) < 3:
        return out
    a, b = inw[0], inw[-1]
    dt = b["t"] - a["t"]
    tck = next((r["clk_tck"] for r in recs if "clk_tck" in r), 100)
    busy = (b["cpu"]["cpu"][0] - a["cpu"]["cpu"][0]) / float(tck) / dt
    out["machine_busy_cores"] = round(busy, 3)
    out["per_cpu_busy_pct"] = dict((k, round(100.0 * (b["cpu"][k][0] - a["cpu"][k][0]) / float(tck) / dt, 1))
                                   for k in sorted(a["cpu"]) if k != "cpu" and k in b["cpu"])
    allf = [f for r in inw for f in r["freq"] if f > 0]
    out["freq_khz"] = {"min": min(allf), "p50": pct(allf, 0.5), "p90": pct(allf, 0.9), "max": max(allf),
                       "share_at_max": round(sum(1 for f in allf if f >= max(allf)) / float(len(allf)), 3)}
    procs = {}
    for pid, p in a["th"].items():
        if pid not in b["th"]:
            continue
        tb = b["th"][pid]["t"]
        th = []
        for tid, (name, ticks, _cpu) in p["t"].items():
            if tid in tb:
                th.append((round(100.0 * (tb[tid][1] - ticks) / float(tck) / dt, 1), name, tid))
        th.sort(reverse=True)
        label = p["cmd"].split("|")[0]
        key = label if label not in procs else label + "#" + pid
        procs[key] = {"pct_of_one_core": round(sum(x[0] for x in th), 1), "threads": len(th),
                      "top_threads": [list(x) for x in th[:6]], "cmd": p["cmd"].split("|", 1)[1][:80]}
    out["processes"] = procs
    u = [r["udp"] for r in recs if "udp" in r]
    if len(u) >= 2:
        out["udp_delta_whole_run"] = dict((k, u[-1][k] - u[0][k]) for k in u[0] if k in u[-1])
    return out


def main(argv):
    d = argv[0]
    w0, w1 = (float(x) for x in open(os.path.join(d, "window.txt")).read().split())
    w0u, w1u = int(w0 * 1e6), int(w1 * 1e6)
    res = {"run": os.path.basename(os.path.normpath(d)), "window_s": round(w1 - w0, 3), "window_wall": [w0, w1]}
    env = open(os.path.join(d, "env.txt")).read()
    res["driver_start_tries"] = env.count("driver_start try=")
    pub = {}                                        # stream -> ordered list of published stamp_us in the window
    if os.path.exists(os.path.join(d, "replay.csv")):
        st = [int(r["stamp_ns"]) // 1000 for r in csv.DictReader(open(os.path.join(d, "replay.csv"), newline=""))]
        inw = [s for s in st if w0u <= s <= w1u]
        pub = {"depth": inw, "color": inw}
        res["publisher"] = {"kind": "replay.csv", "rows_in_window": len(inw), "gap_rule": gap_rule(inw)}
    elif os.path.exists(os.path.join(d, "drv_frames.csv")):
        drv = load_driver_csv(os.path.join(d, "drv_frames.csv"))
        res["publisher"] = {"kind": "drv_frames.csv", "columns": len(drv["_columns"])}
        for s in ("depth", "color"):
            rows = [r for r in drv[s] if w0u <= r["us"] <= w1u]
            if not rows:
                continue
            dev = gap_rule([r["dev_us"] for r in rows])
            hw = [r["hw"] for r in rows if r["hw"] is not None]
            hw_missing = sum(b - a - 1 for a, b in zip(hw, hw[1:]) if b - a > 1) if len(hw) > 1 else None
            idx_missing = sum(b["idx"] - a["idx"] - 1 for a, b in zip(rows, rows[1:]) if b["idx"] - a["idx"] > 1)
            npub = sum(1 for r in rows if r["published"])
            a2p = [r["a2p_us"] for r in rows if r["a2p_us"] is not None]
            sd = [r["sdk_delay_us"] for r in rows]
            pub[s] = [r["us"] for r in rows if r["published"]]
            res["publisher"][s] = {
                "arrived_at_driver": len(rows), "device_stamp_gap_rule": dev,
                "hardware_frame_number_missing": hw_missing, "sdk_index_missing": idx_missing,
                "published": npub, "arrived_not_published": len(rows) - npub,
                "arrival_to_publish_ms": {"p50": round(pct(a2p, 0.5) / 1000.0, 2), "p99": round(pct(a2p, 0.99) / 1000.0, 2),
                                          "max": round(max(a2p) / 1000.0, 2)} if a2p else None,
                "global_stamp_to_driver_arrival_ms": {"p50": round(pct(sd, 0.5) / 1000.0, 2),
                                                      "p99": round(pct(sd, 0.99) / 1000.0, 2), "max": round(max(sd) / 1000.0, 2)}}
    res["receivers"] = {}
    rx_files = [fn for fn in sorted(os.listdir(d))
                if fn.endswith(".csv") and (fn.startswith("rx_") or fn == "cam_probe.csv") and fn != "rx_freeze.csv"]
    if os.path.exists(os.path.join(d, "input_log.csv")):
        rx_files.append("input_log.csv")
    loaded = {}
    for fn in rx_files:
        if fn == "input_log.csv":
            rx, name = load_producer_input_log(os.path.join(d, fn)), "producer"
        else:
            rx, name = load_rx(os.path.join(d, fn)), fn[:-4]
        loaded[name] = rx
        sm = {}
        try:
            sm = json.load(open(os.path.join(d, name + ".json")))
        except (OSError, ValueError):
            pass
        entry = {"qos": sm.get("_receiver"), "topics": {}}
        got = {}
        for topic, rows in sorted(rx.items()):
            inw = [(s, w) for s, w in rows if w0u <= s <= w1u]
            stamps = [s for s, _ in inw]
            got[topic] = set(stamps)
            t = {"received_in_window": len(stamps), "duplicates": len(stamps) - len(set(stamps)),
                 "gap_rule": gap_rule(stamps)}
            if name.startswith("rx_") and inw:
                dl = [(w // 1000 - s) / 1000.0 for s, w in inw]
                t["stamp_to_arrival_ms"] = {"p50": round(pct(dl, 0.5), 2), "p99": round(pct(dl, 0.99), 2), "max": round(max(dl), 2)}
            if topic in sm and isinstance(sm[topic], dict):
                t["parse_check"] = sm[topic].get("parse_check")
                t["first"] = sm[topic].get("first")
            p = pub.get(stream_of(topic))
            if p:
                ps = set(p)
                flags = [s not in got[topic] for s in p]
                t["vs_publisher"] = {"published": len(p), "missing": sum(flags),
                                     "missing_pct": round(100.0 * sum(flags) / len(p), 2),
                                     "extra_not_in_publisher_record": len(got[topic] - ps),
                                     "missing_run_lengths": runs_of(flags)}
            lo = sm[topic].get("lost") if isinstance(sm.get(topic), dict) else None
            t["dds_message_lost"] = lost_readout(lo, w0, w1, t["vs_publisher"]["missing"] if p else None)
            entry["topics"][topic] = t
        # small-message control: image stamps this receiver misses (by the gap rule's grid = publisher record
        # if any, else the union of what its own topics of that stream saw) against its camera_info stamps
        for topic in list(entry["topics"]):
            if "camera_info" in topic:
                continue
            s = stream_of(topic)
            infos = [t for t in got if "camera_info" in t and (stream_of(t) == s or t == "/fpga/camera_info")]
            if not infos:
                continue
            info = got[infos[0]]
            ref = set(pub[s]) if pub.get(s) else (got[topic] | info)
            miss_img = ref - got[topic]
            entry["topics"][topic]["small_message_control"] = {
                "info_topic": infos[0], "reference": "publisher record" if pub.get(s) else "union of image and camera_info stamps",
                "reference_stamps": len(ref), "image_missing": len(miss_img),
                "of_those_camera_info_present": len(miss_img & info),
                "camera_info_missing": len(ref - info), "both_missing": len((ref - got[topic]) - info)}
        res["receivers"][name] = entry
    # which image stamps are missed by which receivers (depth)
    names = [n for n in res["receivers"]]
    for s, topic_hint in (("depth", "depth"), ("color", "color")):
        if not pub.get(s) or len(names) < 2:
            continue
        sets = {}
        for n in names:
            for topic, rows in loaded[n].items():
                if "camera_info" not in topic and stream_of(topic) == s:
                    sets[n] = set(x for x, _ in rows)
        if len(sets) >= 2:
            combo = collections.Counter()
            for st in pub[s]:
                miss = tuple(sorted(n for n in sets if st not in sets[n]))
                if miss:
                    combo["+".join(miss)] += 1
            res.setdefault("missing_by_receiver_set", {})[s] = dict(combo)
    # one table of the DDS message-lost field: every receiver topic, null where not recorded
    rows = []
    for n, entry in sorted(res["receivers"].items()):
        for topic, t in sorted(entry["topics"].items()):
            lo = t.get("dds_message_lost")
            vp = t.get("vs_publisher")
            rows.append({"receiver": n, "topic": topic,
                         "missing": vp["missing"] if vp else t["gap_rule"].get("missed"),
                         "missing_by": "publisher record" if vp else "stamp-gap rule",
                         "lost_in_window": lo.get("in_window") if lo and lo.get("supported") else None,
                         "lost_whole_run": lo.get("whole_run_total") if lo and lo.get("supported") else None,
                         "recorded": lo is not None and bool(lo.get("supported"))})
    res["dds_lost_summary"] = rows
    fz = os.path.join(d, "rx_freeze.csv")
    if os.path.exists(fz):
        res["rx_freeze_rows"] = sum(1 for x in open(fz) if x.strip() and x[0].isdigit())
    log = os.path.join(d, "driver.log")
    if os.path.exists(log):
        txt = open(log, errors="replace").read()
        res["driver_log"] = {"lines": txt.count("\n"), "drop_sdk_receive_lines": txt.count("stage=SDK_RECEIVE"),
                             "drop_ros_publish_lines": txt.count("stage=ROS_PUBLISH"),
                             "warn": txt.count("[WARN]"), "error": txt.count("[ERROR]")}
    sp = os.path.join(d, "sysmon.jsonl")
    if os.path.exists(sp):
        res["sys"] = sysmon(sp, w0, w1)
    out = json.dumps(res, indent=1)
    if "--json" in argv:
        open(argv[argv.index("--json") + 1], "w").write(out + "\n")
    print(out)


if __name__ == "__main__":
    main(sys.argv[1:])
