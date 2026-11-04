#!/usr/bin/env python3
"""NN-CAM2: whole-chain latency / rate / drop report from the stamp logs of kv_server and dpu_rx (--stamp-log,
local frame source). All instants are CLOCK_MONOTONIC ns of one board; the origin of every latency is
t_cam_arrive_ns (kv_cam got the frameset from the SDK) -- the earliest instant on the KV clock. The camera's own
stamps (device clock, SDK host clock) are separate columns and are only used for the diagnostics below.

  depth chain : arrive -> geometry done (t_geo_ns) -> fusion + GRID done (t_grid_ns) -> potential field done
                (the "field" row of the first replan that used this frame's grid; kv_server --nav)
  colour chain: arrive -> preprocessing done (t_prep_ns) -> DPU done (t_dpu_ns) -> post-processing done (t_post_ns)

Per chain: stage latencies (median / p95 / p99, ms), output rate of the last stage (frames/s over the steady window),
drop rate = 1 - frames that reached the stage / camera frames offered in the window (frame index span: kv_cam's
index counts every frameset the camera delivered), camera device-clock interval (fps check).

  chain_report.py STAMPS.jsonl [...] [--warm-s 5] [--json out.json]
"""
import argparse
import json
import sys


def pct(v, q):
    if not v:
        return None
    s = sorted(v)
    i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
    return round(s[i], 3)


def stats(v):
    return {"n": len(v), "median": pct(v, 0.5), "p95": pct(v, 0.95), "p99": pct(v, 0.99),
            "max": round(max(v), 3) if v else None}


def load(paths):
    rows = []
    for p in paths:
        with open(p) as fp:
            for line in fp:
                line = line.strip()
                if line:
                    try:
                        rows.append(json.loads(line))
                    except ValueError:
                        pass
    return rows


def window(frames, warm_s):
    if not frames:
        return frames, None
    t0 = min(r["t_cam_arrive_ns"] for r in frames) + int(warm_s * 1e9)
    return [r for r in frames if r["t_cam_arrive_ns"] >= t0], t0


def rate(ts):
    ts = sorted(ts)
    if len(ts) < 2 or ts[-1] == ts[0]:
        return None
    return round((len(ts) - 1) / ((ts[-1] - ts[0]) / 1e9), 3)


def chain_block(frames, stages, offered_idx):
    out = {"stages": {}, "frames_in_window": len(frames)}
    lo, hi = (min(offered_idx), max(offered_idx)) if offered_idx else (None, None)
    offered = (hi - lo + 1) if offered_idx else 0
    out["camera_frames_offered"] = offered
    for name, key in stages:
        got = [r for r in frames if r.get(key)]
        lat = [(r[key] - r["t_cam_arrive_ns"]) / 1e6 for r in got]
        per_frame = name != "field"     # the field is built per replan (0.5 s), not per frame: no drop rate
        out["stages"][name] = dict(stats(lat), rate_fps=rate([r[key] for r in got]),
                                   drop_rate=round(1 - len(got) / offered, 4) if (offered and per_frame) else None)
    # camera period: device-clock gap / frame-index gap of consecutive processed frames (a chain that keeps only
    # every other frame must still read the camera's 33.4 ms, not 66.8 ms)
    dev = sorted((r["frame"], r["cam_dev_ts_us"]) for r in frames if r.get("cam_dev_ts_us"))
    gaps = [(b[1] - a[1]) / 1e3 / (b[0] - a[0]) for a, b in zip(dev, dev[1:]) if b[0] > a[0] and b[1] > a[1]]
    out["cam_dev_interval_ms"] = stats(gaps)
    out["camera_fps_from_dev_ts"] = round(1e3 / pct(gaps, 0.5), 3) if gaps else None
    pub = [(r["t_cam_publish_ns"] - r["t_cam_arrive_ns"]) / 1e6 for r in frames if r.get("t_cam_publish_ns")]
    rd = [(r["t_read_ns"] - r["t_cam_publish_ns"]) / 1e6 for r in frames if r.get("t_cam_publish_ns")]
    out["kv_cam_arrive_to_publish_ms"] = stats(pub)
    out["publish_to_reader_ms"] = stats(rd)
    return out


def report(rows, warm_s):
    res = {"warm_s": warm_s}
    clock = next((r for r in rows if r.get("ev") == "clock"), None)
    depth = [r for r in rows if r.get("chain") == "depth" and "ev" not in r and r.get("t_cam_arrive_ns")]
    fields = [r for r in rows if r.get("chain") == "depth" and r.get("ev") == "field" and r.get("frame") is not None]
    colour = [r for r in rows if r.get("chain") == "colour" and r.get("src") == "kvcam" and r.get("t_cam_arrive_ns")]
    skipped_udp = sum(1 for r in rows if r.get("chain") == "colour" and r.get("src") != "kvcam")
    if skipped_udp:
        res["note"] = "%d colour rows from the UDP source ignored (no camera stamps)" % skipped_udp
    if depth:
        f_by = {}
        for f in fields:
            f_by.setdefault(f["frame"], f["t_field_ns"])
        for r in depth:
            if r["frame"] in f_by:
                r["t_field_ns"] = f_by[r["frame"]]
        dw, t0 = window(depth, warm_s)
        idx = [r["frame"] for r in dw]
        stages = [("geometry", "t_geo_ns"), ("grid", "t_grid_ns")]
        if fields:
            stages.append(("field", "t_field_ns"))
        res["depth"] = chain_block(dw, stages, idx)
        res["depth"]["e2e_stage"] = stages[-1][0]
        res["depth"]["geo_status_nonzero"] = sum(1 for r in dw if r.get("geo_status"))
        if fields:
            fw = [f for f in fields if f["t_field_ns"] >= t0]
            res["depth"]["field_refresh_hz"] = rate([f["t_field_ns"] for f in fw])
            res["depth"]["grid_to_field_ms"] = stats([(r["t_field_ns"] - r["t_grid_ns"]) / 1e6 for r in dw
                                                       if r.get("t_field_ns") and r.get("t_grid_ns")])
            res["depth"]["plan_ms"] = stats([f["plan_ms"] for f in fw if f.get("plan_ms") is not None])
        if clock is not None:   # SDK host stamp -> arrive, if the SDK stamps CLOCK_REALTIME (diagnostic only)
            off = clock["realtime_ns"] - clock["mono_ns"]
            d = [(r["t_cam_arrive_ns"] + off - r["cam_sys_ts_us"] * 1000) / 1e6 for r in dw if r.get("cam_sys_ts_us")]
            res["depth"]["sdk_sys_ts_to_arrive_ms_if_realtime"] = stats(d)
    if colour:
        cw, _ = window(colour, warm_s)
        res["colour"] = chain_block(cw, [("prep", "t_prep_ns"), ("dpu", "t_dpu_ns"), ("post", "t_post_ns")],
                                    [r["frame"] for r in cw])
        res["colour"]["e2e_stage"] = "post"
        res["colour"]["not_ok"] = sum(1 for r in cw if not r.get("ok", 1))
        res["colour"]["prep_cpu_us"] = stats([r["prep_cpu_us"] for r in cw if "prep_cpu_us" in r])
    return res


def show(res):
    for ch in ("depth", "colour"):
        b = res.get(ch)
        if not b:
            continue
        print("[%s] frames %d / camera offered %d; camera %s fps (device clock)" % (
            ch, b["frames_in_window"], b["camera_frames_offered"], b["camera_fps_from_dev_ts"]))
        for name, s in b["stages"].items():
            print("  arrive -> %-8s median %7s  p95 %7s  p99 %7s ms   out %7s fps  drop %s" % (
                name, s["median"], s["p95"], s["p99"], s["rate_fps"], s["drop_rate"]))
        if b.get("field_refresh_hz") is not None:
            print("  field refresh %s Hz; grid -> field median %s ms" % (b["field_refresh_hz"],
                                                                         b["grid_to_field_ms"]["median"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stamps", nargs="+")
    ap.add_argument("--warm-s", type=float, default=5.0)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    res = report(load(a.stamps), a.warm_s)
    show(res)
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)
    return 0 if ("depth" in res or "colour" in res) else 1


if __name__ == "__main__":
    sys.exit(main())
