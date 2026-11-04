"""Record the Orbbec Gemini 2 built-in IMU (accelerometer + gyroscope) for a fixed time.

Uses pyorbbecsdk2 sensor-level callbacks, so every sample the SDK delivers is kept
(no frame-set aggregation).  Writes one CSV row per sample and a summary JSON with
sample rate, timestamp monotonicity and static mean / standard deviation.

The camera must be static for the whole recording if the mean/std is to be read as
bias/noise.  Run on the Jetson host (PYTHONPATH=~/fpga/p2/pylib) or in a container
with the USB device node passed through.

    python3 nano_imu_record.py --seconds 35 --out DIR [--rate-hz 200]
"""

import argparse
import csv
import json
import math
import os
import platform
import sys
import threading
import time
from pathlib import Path

import numpy as np

import pyorbbecsdk as ob


def list_profiles(sensor, kind):
    plist = sensor.get_stream_profile_list()
    rows = []
    for i in range(plist.get_count()):
        prof = plist.get_stream_profile_by_index(i)
        prof = prof.as_accel_stream_profile() if kind == "accel" else prof.as_gyro_stream_profile()
        rate = prof.get_sample_rate()
        rows.append({"index": i, "profile": prof,
                     "rate": str(rate),
                     "rate_hz": ob.convert_imu_sample_rate_to_value(rate) if hasattr(
                         ob, "convert_imu_sample_rate_to_value") else None,
                     "full_scale": str(prof.get_full_scale_range())})
    return rows


def pick(rows, rate_hz):
    if rate_hz is None:
        return rows[0]
    for row in rows:
        if row["rate_hz"] is not None and abs(float(row["rate_hz"]) - rate_hz) < 1e-6:
            return row
    raise SystemExit(f"no profile at {rate_hz} Hz; available: {[r['rate'] for r in rows]}")


def stats(samples, kind):
    if len(samples) < 2:
        return {"count": len(samples)}
    arr = np.array([[s[1], s[2], s[3], s[4], s[5], s[6], s[7]] for s in samples], dtype=np.float64)
    idx, dev_us, sys_us, host_ns = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
    xyz = arr[:, 4:7]
    d_dev = np.diff(dev_us)
    d_sys = np.diff(sys_us)
    d_host = np.diff(host_ns)
    d_idx = np.diff(idx)
    span_s = (dev_us[-1] - dev_us[0]) * 1e-6
    mean = xyz.mean(axis=0)
    std = xyz.std(axis=0, ddof=1)
    out = {
        "count": int(len(samples)),
        "device_span_s": span_s,
        "rate_hz_device_clock": (len(samples) - 1) / span_s if span_s > 0 else None,
        "rate_hz_host_clock": (len(samples) - 1) / ((host_ns[-1] - host_ns[0]) * 1e-9),
        "device_dt_us": {"median": float(np.median(d_dev)), "min": float(d_dev.min()),
                         "max": float(d_dev.max()), "p99": float(np.percentile(d_dev, 99))},
        "device_ts_nonincreasing": int((d_dev <= 0).sum()),
        "system_ts_nonincreasing": int((d_sys <= 0).sum()),
        "host_monotonic_nonincreasing": int((d_host <= 0).sum()),
        "index_nonincreasing": int((d_idx <= 0).sum()),
        "index_gaps": int((d_idx > 1).sum()),
        "index_missing_total": int(np.clip(d_idx - 1, 0, None).sum()),
        "large_device_gaps_gt_2x_median": int((d_dev > 2 * np.median(d_dev)).sum()),
        "mean_xyz": mean.tolist(),
        "std_xyz": std.tolist(),
        "mean_norm": float(np.linalg.norm(mean)),
    }
    # device clock vs host monotonic clock: slope of (dev - host) over time, in ppm
    host_s = (host_ns - host_ns[0]) * 1e-9
    off_us = (dev_us - dev_us[0]) - host_s * 1e6
    slope = np.polyfit(host_s, off_us, 1)[0]
    out["device_minus_host_drift_ppm"] = float(slope)
    out["callback_latency_proxy_us"] = {
        "note": "host_monotonic - device_ts after removing first-sample offset and linear drift; "
                "spread only, not an absolute latency",
        "p50_minus_min": float(np.percentile(-(off_us - slope * host_s), 50) -
                               np.min(-(off_us - slope * host_s))),
        "p99_minus_min": float(np.percentile(-(off_us - slope * host_s), 99) -
                               np.min(-(off_us - slope * host_s)))}
    if kind == "gyro":
        out["std_xyz_deg_s"] = [math.degrees(v) for v in std]
        out["mean_xyz_deg_s"] = [math.degrees(v) for v in mean]
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=35.0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--rate-hz", type=float, default=None)
    ap.add_argument("--list-only", action="store_true")
    args = ap.parse_args(argv)
    try:
        ob.Context.set_logger_level(ob.OBLogLevel.WARNING)
    except Exception:
        pass

    ctx = ob.Context()
    devs = ctx.query_devices()
    if devs.get_count() == 0:
        raise SystemExit("no Orbbec device found")
    dev = devs.get_device_by_index(0)
    info = dev.get_device_info()
    sensors = {"accel": dev.get_sensor(ob.OBSensorType.ACCEL_SENSOR),
               "gyro": dev.get_sensor(ob.OBSensorType.GYRO_SENSOR)}
    profiles = {k: list_profiles(s, k) for k, s in sensors.items()}
    listing = {k: [{kk: vv for kk, vv in r.items() if kk != "profile"} for r in rows]
               for k, rows in profiles.items()}
    if args.list_only:
        print(json.dumps(listing, indent=1))
        return
    chosen = {k: pick(rows, args.rate_hz) for k, rows in profiles.items()}

    args.out.mkdir(parents=True, exist_ok=False)
    samples = {"accel": [], "gyro": []}
    delivered = {}
    lock = threading.Lock()

    def make_cb(kind):
        def cb(frame):
            host_ns = time.monotonic_ns()
            f = frame.as_accel_frame() if kind == "accel" else frame.as_gyro_frame()
            v = f.get_value()
            row = (kind, f.get_index(), f.get_timestamp_us(), f.get_system_timestamp_us(),
                   host_ns, v.x, v.y, v.z, f.get_temperature())
            with lock:
                if kind not in delivered:
                    try:
                        prof = f.get_stream_profile()
                        prof = (prof.as_accel_stream_profile() if kind == "accel"
                                else prof.as_gyro_stream_profile())
                        delivered[kind] = str(prof.get_sample_rate())
                    except Exception as exc:  # record, do not hide
                        delivered[kind] = f"unreadable: {exc!r}"
                samples[kind].append(row)
        return cb

    wall_start = time.time()
    for kind in ("accel", "gyro"):
        sensors[kind].start(chosen[kind]["profile"], make_cb(kind))
    time.sleep(args.seconds)
    for kind in ("gyro", "accel"):
        sensors[kind].stop()
    time.sleep(0.2)
    with lock:
        snap = {k: list(v) for k, v in samples.items()}

    with (args.out / "imu.csv").open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["sensor", "index", "device_ts_us", "system_ts_us", "host_monotonic_ns",
                    "x", "y", "z", "temperature_c"])
        for kind in ("accel", "gyro"):
            w.writerows(snap[kind])
    summary = {
        "device": {"name": info.get_name(), "serial": info.get_serial_number(),
                   "firmware": info.get_firmware_version(),
                   "connection": info.get_connection_type()},
        "host": {"machine": platform.machine(), "python": sys.version.split()[0],
                 "in_container": os.path.exists("/.dockerenv"),
                 "sdk_file": ob.__file__},
        "requested_seconds": args.seconds, "wall_start": wall_start,
        "profiles_available": listing,
        "profile_used": {k: {kk: vv for kk, vv in r.items() if kk != "profile"}
                         for k, r in chosen.items()},
        "frame_profile_rate_first_sample": delivered,
        "units": {"accel": "m/s^2", "gyro": "rad/s", "timestamps": "us (device/system), ns (host monotonic)"},
        "temperature_c_mean": {k: (float(np.mean([s[8] for s in v])) if v else None)
                               for k, v in snap.items()},
        "accel": stats(snap["accel"], "accel"),
        "gyro": stats(snap["gyro"], "gyro"),
    }
    for kind in ("accel", "gyro"):
        want = chosen[kind]["rate_hz"]
        got = summary[kind].get("rate_hz_device_clock")
        # nominal N Hz is delivered at ~1.004*N on this device; flag anything off by >5 %
        summary[kind]["measured_vs_requested_ok"] = bool(
            want and got and abs(got / float(want) - 1) < 0.05)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("profile_used", "frame_profile_rate_first_sample",
                                              "accel", "gyro")}, indent=1))


if __name__ == "__main__":
    main()
