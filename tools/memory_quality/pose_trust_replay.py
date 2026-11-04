"""Replay the 10-16 real-car run through the pose-trust gate (docs/design/MEMORY_POSE_TRUST_20261017.md).

Inputs (read only): the observation stream (extract_realcar_stream.py full stream, or the compact
fixture) and the yaw series, either rebuilt from the car's ``chassis_record.jsonl`` +
``wheel_odom.jsonl`` at their full 100 Hz (``--chassis``/``--wheel-odom``; clock map = the stream's
``clock_fit`` + ``extra_s``, the map the observation poses were rebuilt with), or the 20 Hz fixture
(``--yaw-series``). ``--write-yaw-series`` writes that fixture from the 100 Hz inputs and exits.

The gyro is integrated as the runner does it (rate x MCU period per status frame; a serial batch
shares one stamp), the wheel yaw is the wheel odometry's own yaw. Arms (same observations):
  main          git ref's core (positive control for the memory-quality harness);
  new / legacy  this tree, no frame carries pose_trusted (= the gate inactive);
  new+trust / legacy+trust  frames carry the monitor's verdict at their acquisition time.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.fpga_replay import core as new_core  # noqa: E402
from perception.fpga_replay import pose_trust  # noqa: E402
from tools.memory_quality import replay_stream as rs  # noqa: E402

REL0 = 1792092965.0 - 2128.0          # relative-time origin of the design docs (crash = 2128 s)
CRASH_ONSET = 2128.0
X_S = 1.5
NORMAL_END = 2127.9
HAND_TURN = (221.0, 222.6)
CRASH_GYRO_END = 2137.0
CRASH_RECOVERY = (2137.0, 2140.5)
HAND_RECOVERY = (222.6, 225.5)
CRASH_TRACKS = ["object-0019", "object-0020", "object-0021", "object-0022", "object-0023"]
GYRO_PERIOD_S = 0.01
FIXTURE_STEP_S = 0.05


def clock_map(fit, extra_s):
    def real(raw_s):
        r = raw_s + fit["base"] + fit["intercept"]
        return raw_s + fit["base"] + fit["intercept"] + fit["slope"] * (r - fit["t0"]) + extra_s
    return real


def series_from_car(chassis, wheel_odom, fit, extra_s):
    """-> (gyro (times, cum yaw), wheel (times, unwrapped yaw)), ROS seconds; same-stamp rows merged."""
    real = clock_map(fit, extra_s)
    gt, gy, s = [], [], 0.0
    with open(chassis) as stream:
        for line in stream:
            r = json.loads(line)
            if r.get("dir") != "rx" or not r.get("gyro_valid"):
                continue
            s += math.radians(r["gyro_z_cdps"] / 100.0) * GYRO_PERIOD_S
            t = real(r["t_ns"] / 1e9)
            if gt and t <= gt[-1]:
                gy[-1] = s
                continue
            gt.append(t); gy.append(s)
    wt, wy, prev = [], [], None
    with open(wheel_odom) as stream:
        for line in stream:
            r = json.loads(line)
            prev = pose_trust.unwrap(prev, r["yaw"])
            t = real(r["t_ns"] / 1e9)
            if wt and t <= wt[-1]:
                wy[-1] = prev
                continue
            wt.append(t); wy.append(prev)
    return (gt, gy), (wt, wy)


def write_yaw_series(gyro, wheel, path):
    t0 = max(gyro[0][0], wheel[0][0])
    t1 = min(gyro[0][-1], wheel[0][-1])
    grid = np.arange(t0, t1, FIXTURE_STEP_S)
    out = {"schema": "fpga.posetrust.yaw_series.v1", "t0": float(grid[0]), "step_s": FIXTURE_STEP_S,
           "note": "cumulative yaw on a 20 Hz grid, as deltas of the centidegree-rounded cumulative value "
                   "(no drift); gyro = rate x 0.01 s per MCU frame; wheel = wheel_odom yaw (unwrapped)"}
    for name, (times, values) in (("gyro", gyro), ("wheel", wheel)):
        cd = np.round(np.degrees(np.interp(grid, times, values)) * 100).astype(np.int64)
        out[name + "_cdeg"] = [int(cd[0])] + np.diff(cd).tolist()
    with open(path, "x") as stream:
        json.dump(out, stream, separators=(",", ":"))
    return len(grid)


def series_from_fixture(path):
    doc = json.loads(Path(path).read_text())
    if doc.get("schema") != "fpga.posetrust.yaw_series.v1":
        raise ValueError("unknown yaw series schema")
    n = len(doc["gyro_cdeg"])
    times = (doc["t0"] + doc["step_s"] * np.arange(n)).tolist()
    out = []
    for name in ("gyro", "wheel"):
        out.append((times, np.radians(np.cumsum(doc[name + "_cdeg"]) / 100.0).tolist()))
    return out[0], out[1]


def run_monitor(gyro, wheel, params=pose_trust.DEFAULT_PARAMS):
    """Feed both series in time order (gyro first on equal stamps, as either may arrive first)."""
    m = pose_trust.PoseTrustMonitor(params, keep_s=1e9)
    i = j = 0
    gt, gv = gyro
    wt, wv = wheel
    while i < len(gt) or j < len(wt):
        if j >= len(wt) or (i < len(gt) and gt[i] <= wt[j]):
            m.add_gyro_yaw(gt[i], gv[i]); i += 1
        else:
            m.add_wheel_yaw(wt[j], wv[j], wrapped=False); j += 1
    return m


def episodes(transitions):
    """[(trip_rel, recovery_rel or None)] from the monitor's transitions (relative seconds)."""
    out = []
    for t, trusted, _ in transitions:
        if not trusted:
            out.append([t - REL0, None])
        elif out:
            out[-1][1] = t - REL0
    return [tuple(e) for e in out]


def in_window(t, a, b):
    return a <= t - REL0 < b


def evaluate(monitor, arms, rej, data_end):
    eps = episodes(monitor.transitions)
    normal = [e for e in eps if e[0] <= NORMAL_END]
    crash = [e for e in eps if e[0] >= CRASH_ONSET]
    first_crash = crash[0] if crash else (None, None)
    trip, rec = first_crash
    rec_w = rec if rec is not None else math.inf
    r = {"episodes_rel_s": [[round(a, 3), None if b is None else round(b, 3)] for a, b in eps],
         "unchecked_evaluations": monitor.unchecked, "yaw_data_end_rel_s": round(data_end - REL0, 3)}
    c = {}
    c["T1_crash_trip_within_X"] = {"trip_rel_s": trip, "x_s": X_S,
                                   "pass": trip is not None and CRASH_ONSET <= trip <= CRASH_ONSET + X_S}
    win = (trip if trip is not None else math.inf, rec_w)

    def applied_in(arm):
        return [a for a in arms[arm][2] if in_window(a[0], *win)]

    def crash_tracks(arm):
        return sorted({a[1] for a in arms[arm][2] if a[1] in CRASH_TRACKS})

    def rejected_in(arm, reason):
        return sum(1 for t, why in rej[arm] if why == reason and in_window(t, *win))

    detail = {}
    for arm in ("new+trust", "legacy+trust", "new", "legacy"):
        detail[arm] = {"applied_in_window": len(applied_in(arm)), "crash_tracks_received": crash_tracks(arm),
                       "below_ground_in_window": rejected_in(arm, "below_ground"),
                       "pose_untrusted_in_window": rejected_in(arm, "pose_untrusted"),
                       "applied_below_0.03": sum(1 for a in arms[arm][2] if a[3][2] < -0.03)}
    t2 = (detail["new+trust"]["applied_in_window"] == 0 and not detail["new+trust"]["crash_tracks_received"]
          and detail["new+trust"]["below_ground_in_window"] == 0
          and detail["new+trust"]["pose_untrusted_in_window"] >= 25
          and detail["legacy+trust"]["applied_in_window"] == 0
          and not detail["legacy+trust"]["crash_tracks_received"])
    control = (detail["legacy"]["applied_in_window"] >= 25 and bool(detail["legacy"]["crash_tracks_received"])
               and detail["new"]["below_ground_in_window"] == 25)
    c["T2_crash_window_stops_writes"] = {"window_rel_s": [trip, rec], "detail": detail, "pass": t2,
                                         "negative_control_pass": control}
    c["T3_normal_segment"] = {"trips": [e[0] for e in normal],
                              "pass": len(normal) == 1 and HAND_TURN[0] <= normal[0][0] <= HAND_TURN[1]}
    untrusted_through = rec is None or rec >= CRASH_GYRO_END
    c["T4a_crash_recovery"] = {"recovery_rel_s": rec,
                               "pass": untrusted_through and rec is not None and
                               CRASH_RECOVERY[0] <= rec <= CRASH_RECOVERY[1]}
    hand_rec = normal[0][1] if normal else None
    after_hand = [a for a in arms["new+trust"][2] if hand_rec is not None and a[0] - REL0 >= hand_rec
                  and a[0] - REL0 < hand_rec + 5]
    c["T4b_hand_turn_recovery"] = {"recovery_rel_s": hand_rec, "applied_within_5s_after": len(after_hand),
                                   "pass": hand_rec is not None and HAND_RECOVERY[0] <= hand_rec <= HAND_RECOVERY[1]
                                   and len(after_hand) > 0}
    after = [a for a in arms["new+trust"][2] if rec is not None and a[0] - REL0 >= rec]
    c["T4c_writes_after_crash_recovery"] = {"applied": len(after), "pass": len(after) > 0}
    created = {}
    for t, lt, tid, p in arms["new+trust"][2]:
        if rec is not None and t - REL0 >= rec and tid not in created:
            before = any(a[2] == tid and a[0] - REL0 < (trip or math.inf) for a in arms["new+trust"][2])
            if not before:
                created[tid] = {"first_rel_s": round(t - REL0, 2), "original_track": lt,
                                "label": arms["new+trust"][0].tracks[tid].label}
    held = sum(1 for f in arms["_frames"] if f[0] > data_end)
    r["T5_report"] = {"tracks_created_after_crash_recovery": created,
                      "later_episodes": [e for e in crash[1:]],
                      "frames_after_yaw_data_end_held_state": held}
    return r, c


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("stream", type=Path)
    p.add_argument("--chassis", type=Path)
    p.add_argument("--wheel-odom", type=Path)
    p.add_argument("--clock-stream", type=Path, help="full stream whose clock_fit maps the car clocks (default: stream)")
    p.add_argument("--yaw-series", type=Path)
    p.add_argument("--write-yaw-series", type=Path)
    p.add_argument("--main-ref", default="origin/main")
    p.add_argument("--out", type=Path)
    a = p.parse_args(argv)
    if a.chassis:
        clock = json.loads((a.clock_stream or a.stream).read_text())
        gyro, wheel = series_from_car(a.chassis, a.wheel_odom, clock["clock_fit"], clock["extra_s"])
        if a.write_yaw_series:
            print(json.dumps({"rows": write_yaw_series(gyro, wheel, a.write_yaw_series)}))
            return 0
        source = "car_100hz"
    else:
        gyro, wheel = series_from_fixture(a.yaw_series)
        source = "fixture_20hz"
    monitor = run_monitor(gyro, wheel)
    data_end = min(gyro[0][-1], wheel[0][-1])
    K, T_bc, image, frames, doc = rs.load_stream(a.stream)
    points = {"find1": 1792092486.9923933, "find2": 1792092813.9154556, "kidnap": 1792092965.0}
    cfg_new, cfg_legacy = new_core.Config(), new_core.Config(memory_quality=False)
    arms, rej = {}, {}
    for name, cfg, trust in (("new", cfg_new, None), ("legacy", cfg_legacy, None),
                             ("new+trust", cfg_new, monitor.trusted_at), ("legacy+trust", cfg_legacy, monitor.trusted_at)):
        rej[name] = []
        arms[name] = rs.run_arm(new_core, cfg, frames, K, T_bc, image, points, True, trust=trust, rejections=rej[name])
    arms["main"] = rs.run_arm(rs.core_from_git(a.main_ref), None, frames, K, T_bc, image, points, False)
    arms["_frames"] = frames
    report, crit = evaluate(monitor, arms, rej, data_end)
    del arms["_frames"]
    mq = {"new": arms["new+trust"], "main": arms["main"]}
    ev = rs.evaluate(mq, points["kidnap"], points["find1"], cfg_new.below_ground_tol_m)
    crit["T4d_memory_quality_C1_C4_with_trust"] = {k: v for k, v in rs.criteria(ev, cfg_new.below_ground_tol_m).items()}
    crit["T4d_memory_quality_C1_C4_with_trust"]["pass"] = all(
        v["pass"] for v in crit["T4d_memory_quality_C1_C4_with_trust"].values() if isinstance(v, dict))
    logged = {(t["id"], t["label"]): t["position_m"] for t in doc["logged_final_tracks"]}
    ends = {k: {(t["id"], t["label"]): t["position_m"] for t in arms[k][1]["end"]} for k in ("main", "legacy", "new")}
    tol = 1e-3 if doc.get("schema") == "fpga.memq.realcar_compact.v1" else 1e-9
    harness = {
        "main_reproduces_logged_tracks": logged.keys() == ends["main"].keys() and all(
            np.allclose(logged[k], ends["main"][k], atol=tol, rtol=0) for k in logged),
        "legacy_equals_main": ends["main"].keys() == ends["legacy"].keys() and all(
            np.array_equal(ends["main"][k], ends["legacy"][k]) for k in ends["main"]),
        "new_rejections_as_memory_quality_doc": dict(arms["new"][0].rejected) == {
            "size_exceeds_class": 29, "clipped_box_cannot_create": 3, "same_frame_duplicate": 1, "below_ground": 25},
    }
    out = {"stream": str(a.stream), "yaw_source": source, "frames": len(frames), "harness": harness,
           "params": pose_trust.DEFAULT_PARAMS._asdict(), "report": report, "criteria": crit,
           "rejected_totals": {k: dict(arms[k][0].rejected) for k in ("new", "new+trust", "legacy+trust")}}
    text = json.dumps(out, ensure_ascii=False, indent=1, default=float)
    if a.out:
        with a.out.open("x") as s:
            s.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
