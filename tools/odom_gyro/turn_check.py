#!/usr/bin/env python3
"""陀螺 3 圈检查（10-25 验收）：原地转 N 圈（默认 3），比较陀螺航向与轮速航向的转角和真值 N*2*pi。

    python3 tools/odom_gyro/turn_check.py <run_dir> [--turns 3] [--t0 S --t1 S] [--json OUT]

数据源（按顺序，第一个有数据的用）：
  1 RUN/wheel_odom.jsonl 里同时有 gyro_yaw 与 yaw 的行（0x82 融合行；两者都是不折返的累积角，
    host/ros_wheel_odom.py / host/gyro_odom.py）；时间 t_ns。
  2 回退 RUN/chassis_record.jsonl 的 rx 行（文件顺序，连续重复帧去掉，dt 用 mcu_ms 差；0x81 的 mcu_ms 16 位回绕已展开）：
      type == status2（0x82）：gz_sum / gz_n * 2000/32768 deg/s（原始，未扣零偏；用 --t0 前的静止段均值当零偏）
      其余（0x81，无 type 键）：gyro_z_cdps（0.01 deg/s）；另给扣 gyro_bias_cdps 后的积分（是否已扣零偏代码未写明）
     回退时轮速转角取 wheel_odom.jsonl 任意有 yaw 的行（窗口相对各自首行，两时钟不严格对齐）。
--t0/--t1 是相对首行的秒数窗口（默认全程）。只读，标准库，py3.8。
"""
import argparse
import json
import math
import os
import sys

GYRO_RAW_DEG_S = 2000. / 32768      # host/chassis_uart.py:38


def iter_jsonl(p):
    try:
        f = open(p, "r", encoding="utf-8", errors="replace")
    except (OSError, IOError):
        return
    with f:
        for line in f:
            try:
                o = json.loads(line)
            except ValueError:
                continue
            if isinstance(o, dict):
                yield o


def from_wheel_odom(p, t0, t1):
    rows = []
    for o in iter_jsonl(p):
        if "gyro_yaw" in o and "yaw" in o and isinstance(o.get("t_ns"), (int, float)):
            if o["gyro_yaw"] is None or o["yaw"] is None:
                continue
            rows.append((o["t_ns"] / 1e9, float(o["gyro_yaw"]), float(o["yaw"]), o.get("heading_source"),
                         bool(o.get("fallback"))))
    if not rows:
        return None
    base = rows[0][0]
    rows = [r for r in rows if (t0 is None or r[0] - base >= t0) and (t1 is None or r[0] - base <= t1)]
    if len(rows) < 2:
        return None
    n = len(rows)
    gyro_src = sum(1 for r in rows if r[3] == "gyro")
    fb = sum(1 for r in rows if r[4])
    return {"source": "wheel_odom.jsonl (0x82 融合行)", "rows": n, "span_s": round(rows[-1][0] - rows[0][0], 3),
            "gyro_rad": rows[-1][1] - rows[0][1], "wheel_rad": rows[-1][2] - rows[0][2],
            "heading_source_gyro_frac": round(gyro_src / float(n), 4), "fallback_frac": round(fb / float(n), 4)}


def from_chassis(p, t0, t1):
    rows, last_key, wrap, prev_ms = [], None, 0, None
    for o in iter_jsonl(p):
        if o.get("dir") != "rx" or not isinstance(o.get("mcu_ms"), (int, float)):
            continue
        key = (o.get("seq"), o.get("mcu_ms"))
        if key == last_key:                      # same frame recorded twice
            continue
        last_key = key
        ms = o["mcu_ms"]
        if o.get("type") != "status2" and prev_ms is not None and ms + wrap < prev_ms - 32768:
            wrap += 65536                        # 0x81 mcu_ms is 16-bit (STATUS_BODY '<BHii...')
        prev_ms = ms + wrap
        if o.get("type") == "status2":
            n = o.get("gz_n") or 0
            rate = math.radians(o["gz_sum"] / float(n) * GYRO_RAW_DEG_S) if n > 0 else None
            bias = None
            kind = "0x82"
        else:
            if "gyro_z_cdps" not in o or o.get("gyro_valid") is False:
                continue
            rate = math.radians(o["gyro_z_cdps"] / 100.)
            bias = math.radians((o.get("gyro_bias_cdps") or 0) / 100.)
            kind = "0x81"
        rows.append(((ms + wrap) / 1000., rate, bias, kind))
    if len(rows) < 2:
        return None
    base = rows[0][0]
    lo = t0 if t0 is not None else 0.
    still = [r[1] for r in rows if r[3] == "0x82" and r[1] is not None and r[0] - base < lo] if t0 else []
    raw_bias82 = sum(still) / len(still) if still else 0.
    raw = corr = 0.
    used = 0
    for a, b in zip(rows, rows[1:]):
        rel = b[0] - base
        if (t0 is not None and rel < t0) or (t1 is not None and rel > t1):
            continue
        dt = b[0] - a[0]
        if dt <= 0 or dt > 0.5 or b[1] is None:
            continue
        raw += b[1] * dt
        corr += (b[1] - (b[2] if b[2] is not None else raw_bias82)) * dt
        used += 1
    kinds = sorted(set(r[3] for r in rows))
    return {"source": "chassis_record.jsonl rx（%s）" % "/".join(kinds), "rows": used,
            "gyro_rad": raw, "gyro_rad_bias_corrected": corr, "wheel_rad": None,
            "bias_note": ("0x82 零偏取 --t0 前静止段均值" if "0x82" in kinds else
                          "0x81 gyro_z_cdps 是否已扣零偏未写明：同时给原始与扣 gyro_bias_cdps 的积分")}


def wheel_only(p, t0, t1):
    """Wheel yaw change from wheel_odom.jsonl (any row with yaw); window relative to its own first row."""
    first = last = base = None
    for o in iter_jsonl(p):
        t, y = o.get("t_ns"), o.get("yaw")
        if not isinstance(t, (int, float)) or not isinstance(y, (int, float)):
            continue
        base = t if base is None else base
        rel = (t - base) / 1e9
        if (t0 is not None and rel < t0) or (t1 is not None and rel > t1):
            continue
        first = y if first is None else first
        last = y
    return None if first is None else last - first


def check(run, turns=3., t0=None, t1=None):
    truth = 2 * math.pi * turns
    r = from_wheel_odom(os.path.join(run, "wheel_odom.jsonl"), t0, t1)
    if r is None:
        r = from_chassis(os.path.join(run, "chassis_record.jsonl"), t0, t1)
        if r is not None:
            r["wheel_rad"] = wheel_only(os.path.join(run, "wheel_odom.jsonl"), t0, t1)
    if r is None:
        return {"verdict": "NODATA", "truth_rad": truth, "note": "wheel_odom.jsonl 无 gyro_yaw 行，chassis_record.jsonl 无 rx 行"}
    r["truth_rad"] = round(truth, 4)
    for k in ("gyro_rad", "wheel_rad", "gyro_rad_bias_corrected"):
        if r.get(k) is not None:
            r[k] = round(r[k], 4)
            r[k.replace("_rad", "_ratio")] = round(abs(r[k]) / truth, 4)
    if r.get("gyro_rad") is not None and r.get("wheel_rad"):
        r["gyro_over_wheel"] = round(r["gyro_rad"] / r["wheel_rad"], 4) if r["wheel_rad"] else None
    r["verdict"] = "INFO"
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("run_dir")
    ap.add_argument("--turns", type=float, default=3.)
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)
    r = check(os.path.realpath(a.run_dir), a.turns, a.t0, a.t1)
    print("陀螺 %.3g 圈检查：真值 %.3f rad" % (a.turns, r["truth_rad"]))
    for k, v in r.items():
        if k != "truth_rad":
            print("  %s: %s" % (k, v))
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(r, f, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
