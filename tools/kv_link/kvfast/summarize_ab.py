#!/usr/bin/env python3
"""E7 board A/B summary: Python path (off) vs C++ data plane (native), two calibers.

  --server off=DIR native=DIR   nano_replay.py --out dirs (frames.csv, kv_*_ms columns = the KV server's
                                own per-request stage durations, the same fields the car's
                                /fpga/kv_link/frame_stats carries): decode, convert, crc, core_prep,
                                core_device, core_readback, core_total, KV service (slot_wait..core_total),
                                rtt and frame age on the Nano.
  --segments off=CSV native=CSV py_arm.py frames.csv of the PL arm (C_pl_*), segment rules of
                                tools/kv_power/segments.py (commit 836e2f5; segments_a2.md): device,
                                core-device, call-core, convert, e2e -- only convert changes with --convert.
Percentile = nearest rank, index round(q*(n-1)) (segments.py rule). Prints a Markdown table; --json too.
Usage: summarize_ab.py [--server off=D native=D] [--segments off=C native=C] [--json out.json]
"""
import argparse
import csv
import json
import os
import statistics
import sys


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else float("nan")


def stat(v):
    return {"n": len(v), "median": statistics.median(v) if v else float("nan"), "p95": pct(v, .95)}


SERVER_COLS = ("kv_decode_ms", "kv_convert_ms", "kv_crc_ms", "kv_core_prep_ms", "kv_core_device_ms",
               "kv_core_readback_ms", "kv_core_total_ms", "kv_service_ms", "rtt_ms", "age_ms")


def server_arm(d):
    rows = [r for r in csv.DictReader(open(os.path.join(d, "frames.csv"))) if r.get("rtt_ms")]
    out = {}
    for c in SERVER_COLS:
        out[c] = stat([float(r[c]) for r in rows if r.get(c) not in (None, "")])
    # the A53 data-plane time the switch replaces, per request: decode + convert + crc
    out["decode+convert+crc"] = stat([float(r["kv_decode_ms"]) + float(r["kv_convert_ms"]) + float(r["kv_crc_ms"])
                                      for r in rows])
    return out


def segments_arm(path):
    rows = list(csv.DictReader(open(path)))
    f = lambda k: [float(r[k]) for r in rows]  # noqa: E731
    dev, core, call, conv, e2e = f("device_ms"), f("core_ms"), f("call_ms"), f("convert_ms"), f("e2e_ms")
    return {"device": stat(dev), "core-device": stat([c - d for c, d in zip(core, dev)]),
            "call-core": stat([a - c for a, c in zip(call, core)]), "convert": stat(conv), "e2e": stat(e2e)}


def kv(pairs):
    out = {}
    for p in pairs or []:
        k, v = p.split("=", 1)
        out[k] = v
    return out


def table(title, arms, keys):
    names = list(arms)
    lines = ["### " + title, "", "| 段 | " + " | ".join("%s 中位 / p95 (n)" % n for n in names) +
             (" | 中位差 native−off |" if set(names) >= {"off", "native"} else " |"),
             "|---|" + "---|" * len(names) + ("---|" if set(names) >= {"off", "native"} else "")]
    for k in keys:
        cells = ["%.3f / %.3f (%d)" % (arms[n][k]["median"], arms[n][k]["p95"], arms[n][k]["n"]) for n in names]
        diff = ""
        if set(names) >= {"off", "native"}:
            diff = " | %+.3f" % (arms["native"][k]["median"] - arms["off"][k]["median"])
        lines.append("| %s | %s%s |" % (k, " | ".join(cells), diff))
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--server", nargs="*")
    ap.add_argument("--segments", nargs="*")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    res, md = {}, []
    if a.server:
        res["server"] = {k: server_arm(v) for k, v in kv(a.server).items()}
        md.append(table("kv_server 每请求分段（KV 自己的计时，ms）", res["server"],
                        ("kv_decode_ms", "kv_convert_ms", "kv_crc_ms", "decode+convert+crc", "kv_core_prep_ms",
                         "kv_core_device_ms", "kv_core_readback_ms", "kv_service_ms", "rtt_ms", "age_ms")))
    if a.segments:
        res["segments"] = {k: segments_arm(v) for k, v in kv(a.segments).items()}
        md.append(table("py_arm 逐段（segments.py 口径，ms）", res["segments"],
                        ("device", "core-device", "call-core", "convert", "e2e")))
    if not res:
        ap.error("give --server and/or --segments")
    print("\n".join(md))
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1, sort_keys=True)


if __name__ == "__main__":
    sys.exit(main())
