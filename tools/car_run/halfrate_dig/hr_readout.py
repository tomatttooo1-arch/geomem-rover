#!/usr/bin/env python3
"""Readout of the half-rate dig session (10-17, COLOUR_RATE_NAVMODE section 7.3/7.4) from the pulled Nano runs.

  hr_readout.py RUNS_DIR [--json OUT]       RUNS_DIR = <session>/nano/runs

Per driver-only start (e1/sN, e3/sN), three instruments, each judged on its own:
  probe   the A check's own verdict (rate_probe.log, header stamps, 3 s after 30 depth images)
  info    crdig_probe.py state.json: camera_info header Hz of colour / depth over 10 s, exposure / gain / brightness
  csv     drv_frames.csv device timestamps over the whole start (after the first 1 s): depth / colour interval medians
          -> half if depth median < 32.8 ms (the same depth rule), else normal
  sdk     OrbbecSDK log "Color/Depth backend frame callback, frameRate" (the device-side truth), last values
E2 rows: the manual-exposure windows (manual20000.json / manual33000.json): the readback after the set and the Hz.
Chain starts (cN): the camera driver try sequence from marks.txt, camera_rate_check.json, the kept start's CSV medians,
the time from "start" to "camera driver try=1" and to READY.
Second baton (section 8.1, ARM runs dir runs_<ARM>): E2c chain starts hN (A off; hr/ holds A's probe in the stack,
the 10 s state window, chk verdict, manual20000/33000/30000.json) and E5 driver + inference starts e5/sN."""
import argparse
import csv
import glob
import json
import math
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(HERE))))
from tools.car_run.depth_probe import last_json, DEPTH_HALF_MS  # noqa: E402


def load(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def csv_medians(p, skip_s=1.0):
    d, c = [], []
    try:
        with open(p) as f:
            for r in csv.DictReader(f):
                for k, lst in (("depth", d), ("color", c)):
                    v = r.get(k + "_device_ts_sec")
                    if v:
                        lst.append(float(v))
    except OSError:
        return None
    out = {}
    for k, s in (("depth", sorted(set(d))), ("colour", sorted(set(c)))):
        if s:
            s = [x for x in s if x >= s[0] + skip_s]
        iv = [(b - a) * 1e3 for a, b in zip(s, s[1:])]
        out[k + "_n"] = len(s)
        out[k + "_median_ms"] = round(statistics.median(iv), 2) if iv else None
        out[k + "_hz"] = round((len(s) - 1) / (s[-1] - s[0]), 3) if len(s) > 2 else None
    dm = out.get("depth_median_ms")
    out["verdict"] = None if dm is None or out["depth_n"] < 30 else ("half" if dm < DEPTH_HALF_MS else "normal")
    return out


def sdk_rates(d):
    res = {}
    for p in sorted(glob.glob(os.path.join(d, "**", "OrbbecSDK_*.log"), recursive=True)):
        with open(p, errors="replace") as f:
            for line in f:
                m = re.search(r"(Color|Depth) backend frame callback, frameRate=([\d.]+)fps", line)
                if m:
                    res.setdefault(m.group(1).lower(), []).append(float(m.group(2)))
    return {k: (round(v[-1], 2), len(v)) for k, v in res.items()}


def info_row(st):
    if not st:
        return None
    return {"colour_hz": st["color"].get("hz"), "depth_hz": st["depth"].get("hz"),
            "colour_p50_ms": st["color"].get("dt_ms_p50"), "depth_p50_ms": st["depth"].get("dt_ms_p50"),
            "exp": [st["start"]["cam"]["exposure"].get("data"), st["end"]["cam"]["exposure"].get("data")],
            "gain": [st["start"]["cam"]["gain"].get("data"), st["end"]["cam"]["gain"].get("data")],
            "bright": [(st["start"]["bright"] or {}).get("mean"), (st["end"]["bright"] or {}).get("mean")],
            "actions": st.get("actions")}


def wilson(k, n, z=1.96):
    if n == 0:
        return None
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return [round(c - h, 3), round(c + h, 3)]


MANUAL = (20000, 33000, 30000)


def drv_starts(runs, arm):
    rows = []
    for d in sorted(glob.glob(os.path.join(runs, arm, "s*")), key=lambda x: int(re.sub(r"\D", "", os.path.basename(x)))):
        pr = last_json(os.path.join(d, "rate_probe.log"))
        rc = None
        try:
            with open(os.path.join(d, "marks.txt")) as f:
                m = re.search(r"probe_rc=(\d+)", f.read())
                rc = int(m.group(1)) if m else None
        except OSError:
            pass
        r = {"start": os.path.basename(d), "probe_rc": rc,
             "probe": (pr or {}).get("rate"), "depth_up": bool(pr) and pr.get("received", 0) >= pr.get("needed", 1),
             "info": info_row(load(os.path.join(d, "state.json"))), "csv": csv_medians(os.path.join(d, "drv_frames.csv")),
             "sdk": sdk_rates(os.path.join(d, "sdklog"))}
        for e in MANUAL:
            mj = load(os.path.join(d, "manual%d.json" % e))
            if mj:
                r["manual%d" % e] = info_row(mj)
        pv = (r["probe"] or {}).get("verdict")
        cv = (r["csv"] or {}).get("verdict")
        iv = None
        if r["info"] and r["info"]["depth_p50_ms"]:
            iv = "half" if r["info"]["depth_p50_ms"] < DEPTH_HALF_MS else "normal"
        r["verdicts"] = {"probe": pv, "info": iv, "csv": cv}
        vs = [v for v in (pv, iv, cv) if v in ("half", "normal")]
        r["agree"] = (len(set(vs)) == 1) if vs else None
        try:
            with open(os.path.join(d, "driver.log"), errors="replace") as f:
                r["ae_max_lines"] = [l.strip()[-120:] for l in f if "AE max exposure" in l]
        except OSError:
            r["ae_max_lines"] = []
        rows.append(r)
    return rows


def chain_starts(runs, pat="c[0-9]*"):
    rows = []
    for d in sorted(glob.glob(os.path.join(runs, pat)), key=lambda x: int(re.sub(r"\D", "", os.path.basename(x)))):
        if not os.path.isdir(d):
            continue
        marks = []
        try:
            with open(os.path.join(d, "marks.txt")) as f:
                marks = [l.rstrip("\n").split(" ", 1) for l in f]
        except OSError:
            pass
        t = {m[1].split(" ")[0] if m[1].startswith(("start", "READY")) else m[1]: float(m[0]) for m in marks if len(m) == 2}
        t_start = next((float(m[0]) for m in marks if m[1].startswith("start ")), None)
        t_try1 = next((float(m[0]) for m in marks if m[1].startswith("camera driver try=1 ")), None)
        t_ready = next((float(m[0]) for m in marks if m[1] == "READY"), None)
        tries = [m[1] for m in marks if len(m) == 2 and m[1].startswith("camera driver ")]
        rc = load(os.path.join(d, "camera_rate_check.json"))
        fail = None
        if os.path.exists(os.path.join(d, "stack_fail.txt")):
            with open(os.path.join(d, "stack_fail.txt")) as f:
                fail = f.read().strip()
        rows.append({"start": os.path.basename(d), "tries": tries,
                     "rate_check": (rc or {}).get("rate"), "kept_try": (rc or {}).get("try"),
                     "probe_seconds": (rc or {}).get("seconds"), "depth_seconds": (rc or {}).get("depth_seconds"),
                     "csv_kept": csv_medians(os.path.join(d, "drv_frames.csv")),
                     "info": info_row(load(os.path.join(d, "hr", "state.json"))),
                     "s_to_try1": round(t_try1 - t_start, 2) if t_try1 and t_start else None,
                     "s_to_ready": round(t_ready - t_start, 2) if t_ready and t_start else None,
                     "ready": t_ready is not None, "fail": fail, "sdk": sdk_rates(os.path.join(d, "hr"))})
        hr = os.path.join(d, "hr")
        if os.path.exists(os.path.join(hr, "rate_probe.log")):   # E2c: the stack-side check of section 8.1
            chk = None
            try:
                with open(os.path.join(hr, "marks.txt")) as f:
                    m = re.search(r"probe_rc=(\d+) verdict=(\w+)", f.read())
                    chk = {"probe_rc": int(m.group(1)), "verdict": m.group(2)} if m else None
            except OSError:
                pass
            rows[-1]["chk"] = chk
            rows[-1]["chk_probe"] = (last_json(os.path.join(hr, "rate_probe.log")) or {}).get("rate")
            for e in MANUAL:
                mj = load(os.path.join(hr, "manual%d.json" % e))
                if mj:
                    rows[-1]["manual%d" % e] = info_row(mj)
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs")
    ap.add_argument("--json")
    a = ap.parse_args(argv)
    e1 = drv_starts(a.runs, "e1")
    e3 = drv_starts(a.runs, "e3")
    e5 = drv_starts(a.runs, "e5")
    ch = chain_starts(a.runs) + chain_starts(a.runs, "h[0-9]*")
    up = [r for r in e1 if r["depth_up"]]
    half = [r for r in up if r["verdicts"]["probe"] == "half"]
    summ = {"e1_starts": len(e1), "e1_depth_up": len(up), "e1_half": len(half),
            "e1_half_fraction": round(len(half) / len(up), 3) if up else None, "e1_half_wilson95": wilson(len(half), len(up)),
            "e1_disagreements": [r["start"] for r in e1 if r["agree"] is False]}
    up5 = [r for r in e5 if r["depth_up"]]
    h5 = [r for r in up5 if r["verdicts"]["probe"] == "half"]
    summ.update({"e5_starts": len(e5), "e5_depth_up": len(up5), "e5_half": len(h5),
                 "e5_half_wilson95": wilson(len(h5), len(up5)), "e5_disagreements": [r["start"] for r in e5 if r["agree"] is False],
                 "e2c_starts": [(r["start"], (r.get("chk") or {}).get("verdict"), (r["csv_kept"] or {}).get("verdict"))
                                for r in ch if r["start"].startswith("h")]})
    def manual_lines(r):
        for e in MANUAL:
            m = r.get("manual%d" % e)
            if m:
                acts = [x for x in (m.get("actions") or []) if "after_set_exp" in x or "after_set_ae" in x]
                print("    manual %d: colour %s Hz depth %s Hz (p50 %s / %s ms) exp %s gain %s bright %s | readback %s" % (
                    e, m["colour_hz"], m["depth_hz"], m["colour_p50_ms"], m["depth_p50_ms"], m["exp"], m["gain"],
                    m["bright"], json.dumps(acts)))

    for r in e1 + e3 + e5:
        i, c = r["info"] or {}, r["csv"] or {}
        print("%s/%s rc=%s probe=%s info=%s csv=%s agree=%s | probe d/c %s/%s | info c/d %s/%s Hz | csv d/c %s/%s ms | "
              "sdk %s | exp %s gain %s bright %s%s" % (
                  "e1" if r in e1 else ("e3" if r in e3 else "e5"), r["start"], r["probe_rc"], r["verdicts"]["probe"], r["verdicts"]["info"],
                  r["verdicts"]["csv"], r["agree"], (r["probe"] or {}).get("depth_median_ms"),
                  (r["probe"] or {}).get("colour_p25_ms"), i.get("colour_hz"), i.get("depth_hz"),
                  c.get("depth_median_ms"), c.get("colour_median_ms"), r["sdk"], i.get("exp"), i.get("gain"),
                  i.get("bright"), (" AE-max: %s" % r["ae_max_lines"]) if r in e3 else ""))
        manual_lines(r)
    for r in ch:
        print("%s ready=%s tries=%s | kept try %s %s | csv kept d/c %s/%s ms | to try1 %s s, to READY %s s | probe %s s "
              "(depth %s s) | exp %s | fail %s" % (
                  r["start"], r["ready"], r["tries"], r["kept_try"], (r["rate_check"] or {}).get("verdict"),
                  (r["csv_kept"] or {}).get("depth_median_ms"), (r["csv_kept"] or {}).get("colour_median_ms"),
                  r["s_to_try1"], r["s_to_ready"], r["probe_seconds"], r["depth_seconds"], (r["info"] or {}).get("exp"),
                  (r["fail"] or "")[:200]))
        if "chk" in r:
            i, cp = r["info"] or {}, r["chk_probe"] or {}
            print("    chk %s | stack probe d/c %s/%s ms | info c/d %s/%s Hz p50 %s/%s ms | exp %s gain %s bright %s | sdk %s" % (
                r["chk"], cp.get("depth_median_ms"), cp.get("colour_p25_ms"), i.get("colour_hz"), i.get("depth_hz"),
                i.get("colour_p50_ms"), i.get("depth_p50_ms"), i.get("exp"), i.get("gain"), i.get("bright"), r["sdk"]))
            manual_lines(r)
    print(json.dumps(summ))
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"summary": summ, "e1": e1, "e3": e3, "e5": e5, "chain": ch}, f, indent=1)


if __name__ == "__main__":
    main()
