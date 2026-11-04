"""Offline evaluation: wheel odometry with gyro heading (docs/design/ODOM_GYRO_HEADING_20261017.md).

Reads (only) the 10-16 real-car records -- chassis_record.jsonl (MCU status frames: gyro + counts),
wheel_odom.jsonl, the rebuilt observation stream (extract_realcar_stream.py; its clock_fit + extra_s
is the one clock map used for everything) and the rtabmap export -- and writes one JSON report.

Arms (same observations, same camera mount, same clock map):
  wheel     heading = wheel_odom yaw (what ran on the car)
  gyro_raw  heading = gyro rate x 0.01 s per MCU frame, no despiking (ablation, report only)
  fused     heading = despiked gyro (median-5, > 15 deg/s from the median -> median) minus a static-bias
            EMA (wheels still >= 1 s), wheel increments where the gyro has a gap > 50 ms or no data;
            translation = wheel path increments along that heading (midpoint).
The memory replay uses the memory code of ``--code-ref`` (default ceb1b4d, pose-trust gate included),
exported with ``git archive`` into a temporary directory; the pose-trust monitor always compares the
WHEEL yaw with the raw gyro (the gate asks whether the wheels saw the turn).

Section 4 of the design doc is the frozen criteria; this tool computes them.
"""
import argparse
import importlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CAR = Path("${FPGA_SRC}/runs/realcar-first-hour-20261016/nano/runs/20261016-025937-live")
MEMQ = Path("${FPGA_SRC}/runs/memq-20261016")
DEFAULTS = {
    "chassis": CAR / "chassis_record.jsonl",
    "wheel_odom": MEMQ / "nano-025937" / "wheel_odom.jsonl",
    "stream": MEMQ / "stream_025937.json",
    "rtab_poses": CAR / "rtabmap" / "run1" / "export" / "20261015_190638_poses.txt",
}

REL0 = 1792092965.0 - 2128.0
GYRO_PERIOD_S = 0.01
DESPIKE_WINDOW = 5
DESPIKE_THR_CDPS = 1500.0          # 15 deg/s
BIAS_STILL_S = 1.0
BIAS_ALPHA = 0.01
GAP_S = 0.05
CRASH_WINDOW = (2127.9, 2139.15)   # normal end .. pose-trust recovery (MEMORY_POSE_TRUST section 6)
CRASH_ONSET = 2128.0
POST_WINDOW = (2139.15, 3031.6)
PRE_WINDOW = (0.0, 2127.9)
THETA_TRUNC_M = 1.5
MISPLACED = ["object-00%d" % i for i in range(24, 31)]
LOOP_PAIRS = [(242, 298), (242, 317), (242, 297), (222, 272), (268, 318), (110, 610), (109, 611), (109, 629),
              (592, 637), (586, 638), (586, 654), (586, 655), (748, 801), (753, 812), (975, 1066)]
CLUSTER_FIRST = [(242, 298), (222, 272), (268, 318), (110, 610), (592, 637), (748, 801), (753, 812), (975, 1066)]
PHYS_BOTTLES = {"milk": ["object-0003", "object-0010"], "nongfu": ["object-0007", "object-0009", "object-0016"],
                "redcap": ["object-0008", "object-0017", "object-0018"]}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- inputs
def clock_map(fit, extra_s):
    def real(raw_s):
        r = raw_s + fit["base"] + fit["intercept"]
        return raw_s + fit["base"] + fit["intercept"] + fit["slope"] * (r - fit["t0"]) + extra_s
    return real


def load_chassis(path):
    rows = []
    with open(path) as s:
        for line in s:
            r = json.loads(line)
            if r.get("dir") == "rx":
                rows.append((r["t_ns"], r["gyro_z_cdps"], int(bool(r["gyro_valid"])), r["left_count"], r["right_count"]))
    a = np.array(rows, dtype=np.int64)
    return {"t_ns": a[:, 0], "gz": a[:, 1].astype(float), "valid": a[:, 2].astype(bool), "left": a[:, 3], "right": a[:, 4]}


def load_wheel(path):
    t, x, y, yaw, dist = [], [], [], [], []
    with open(path) as s:
        for line in s:
            r = json.loads(line)
            t.append(r["t_ns"]); x.append(r["x"]); y.append(r["y"]); yaw.append(r["yaw"]); dist.append(r["distance"])
    return {"t_ns": np.array(t, dtype=np.int64), "x": np.array(x), "y": np.array(y), "yaw": np.array(yaw),
            "distance": np.array(dist)}


def load_rtab(path):
    a = np.array([l.split() for l in open(path) if not l.startswith("#")], dtype=float)
    return {"t": a[:, 0], "x": a[:, 1], "y": a[:, 2], "yaw": 2 * np.arctan2(a[:, 6], a[:, 7]), "id": a[:, 8].astype(int)}


def dedup_last(t, v):
    """Keep the last value of each run of equal stamps (serial batches share one stamp)."""
    keep = np.r_[t[1:] > t[:-1], True]
    return t[keep], v[keep]


# ---------------------------------------------------------------- heading
def despike(g, thr=DESPIKE_THR_CDPS, w=DESPIKE_WINDOW):
    """Centred median-w; samples further than thr from it are replaced (online: w//2 frames of delay)."""
    from numpy.lib.stride_tricks import sliding_window_view
    h = w // 2
    med = np.median(sliding_window_view(np.pad(g, (h, h), mode="edge"), w), axis=1)
    bad = np.abs(g - med) > thr
    out = g.copy()
    out[bad] = med[bad]
    return out, bad


def static_bias(g, left, right, still_frames=int(round(BIAS_STILL_S / GYRO_PERIOD_S)), alpha=BIAS_ALPHA):
    """Causal EMA of the (despiked) rate over frames whose counts have not moved for >= still_frames."""
    moved = np.r_[True, (np.diff(left) != 0) | (np.diff(right) != 0)]
    bias = np.zeros(len(g))
    b, since = 0.0, 0
    for i in range(len(g)):
        since = 0 if moved[i] else since + 1
        bias[i] = b                      # the value subtracted from frame i (estimated before it)
        if since >= still_frames:
            b = (1 - alpha) * b + alpha * g[i]
    return bias


def gyro_cumulative(ch, real, mode):
    """-> (stamps, cumulative yaw rad) on de-duplicated arrival stamps, plus diagnostics."""
    g = ch["gz"].copy()
    g[~ch["valid"]] = 0.0
    diag = {"frames": int(len(g)), "invalid_frames": int((~ch["valid"]).sum())}
    if mode in ("fused", "fused_nobias"):
        g, bad = despike(g)
        diag["despiked_frames"] = int(bad.sum())
    if mode == "fused":
        bias = static_bias(g, ch["left"], ch["right"])
        diag["bias_cdps_max_abs"] = float(np.abs(bias).max())
        g = g - bias
    cum = np.cumsum(np.radians(g / 100.0) * GYRO_PERIOD_S)
    t = real(ch["t_ns"] / 1e9)
    t, cum = dedup_last(t, cum)
    return t, cum, diag


def heading_on_wheel_rows(wheel_t, wheel_yaw, gyro_t, gyro_cum):
    """Per wheel row interval: gyro increment where covered with no gap > GAP_S, else the wheel increment."""
    dw = np.diff(wheel_yaw)
    gi = np.interp(wheel_t, gyro_t, gyro_cum)
    dg = np.diff(gi)
    covered = (wheel_t[:-1] >= gyro_t[0]) & (wheel_t[1:] <= gyro_t[-1])
    gaps = np.flatnonzero(np.diff(gyro_t) > GAP_S)
    use_wheel = ~covered
    for j in gaps:
        a, b = gyro_t[j], gyro_t[j + 1]
        use_wheel |= (wheel_t[1:] > a) & (wheel_t[:-1] < b)
    d = np.where(use_wheel, dw, dg)
    return wheel_yaw[0] + np.r_[0.0, np.cumsum(d)], int(use_wheel.sum())


def trajectory(wheel, psi):
    dd = np.diff(wheel["distance"])
    mid = 0.5 * (psi[1:] + psi[:-1])
    x = wheel["x"][0] + np.r_[0.0, np.cumsum(dd * np.cos(mid))]
    y = wheel["y"][0] + np.r_[0.0, np.cumsum(dd * np.sin(mid))]
    return x, y


class Arm:
    def __init__(self, name, t, x, y, psi):
        self.name = name
        tt, v = dedup_last(t, np.c_[x, y, psi])
        self.t, self.x, self.y, self.psi = tt, v[:, 0], v[:, 1], v[:, 2]

    def pose(self, ts):
        return np.interp(ts, self.t, self.x), np.interp(ts, self.t, self.y), np.interp(ts, self.t, self.psi)


# ---------------------------------------------------------------- observations
def reproject(frames, arm):
    """Stream frames (t, base, obs) -> same frames under ``arm`` (obs moved rigidly with the base)."""
    ts = np.array([f[0] for f in frames])
    X, Y, P = arm.pose(ts)
    out = []
    for (t, base, obs), x, y, p in zip(frames, X, Y, P):
        bx, by, bp = base
        c0, s0 = math.cos(-bp), math.sin(-bp)
        c1, s1 = math.cos(p), math.sin(p)
        new_obs = []
        for o in obs:
            q = o["position_m"]
            dx, dy = q[0] - bx, q[1] - by
            u, v = c0 * dx - s0 * dy, s0 * dx + c0 * dy
            o2 = dict(o)
            o2["position_m"] = [x + c1 * u - s1 * v, y + s1 * u + c1 * v, q[2]]
            new_obs.append(o2)
        out.append((t, [float(x), float(y), float(p)], new_obs))
    return out


def theta_scan(frames, arm):
    """Camera-referenced residual crash rotation (doc section 4, 2a)."""
    pre, post = {}, {}
    for t, base, obs in frames:
        r = t - REL0
        for o in obs:
            if o["label"] not in ("backpack", "cell phone"):
                continue
            if PRE_WINDOW[0] <= r < PRE_WINDOW[1]:
                pre.setdefault(o["label"], []).append(o["position_m"][:2])
            elif POST_WINDOW[0] <= r < POST_WINDOW[1]:
                post.setdefault(o["label"], []).append(o["position_m"][:2])
    cx, cy, _ = arm.pose(np.array([REL0 + POST_WINDOW[0]]))
    c = np.array([cx[0], cy[0]])
    thetas = np.arange(-180, 180)
    cost = np.zeros(len(thetas))
    n = 0
    for label, pts in post.items():
        P = np.array(pts) - c
        Q = np.array(pre.get(label, []))
        if not len(Q):
            continue
        n += len(P)
        for k, th in enumerate(np.radians(thetas)):
            R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            Pr = P @ R.T + c
            dmin = np.sqrt(((Pr[:, None, :] - Q[None, :, :]) ** 2).sum(-1)).min(1)
            cost[k] += np.minimum(dmin, THETA_TRUNC_M).sum()
    cost /= max(n, 1)
    k = int(cost.argmin())
    far = np.abs(((thetas - thetas[k]) + 180) % 360 - 180) > 30
    k2 = int(np.flatnonzero(far)[cost[far].argmin()])
    return {"theta_star_deg": int(thetas[k]), "cost_min_m": round(float(cost[k]), 4),
            "second_theta_deg": int(thetas[k2]), "second_cost_m": round(float(cost[k2]), 4),
            "distinct": bool(cost[k2] >= 1.2 * cost[k]), "cost_at_0_m": round(float(cost[thetas == 0][0]), 4),
            "post_observations": n, "pre_observations": {k_: len(v) for k_, v in pre.items()},
            "post_by_label": {k_: len(v) for k_, v in post.items()},
            "car_at_recovery": [round(float(c[0]), 3), round(float(c[1]), 3)],
            "curve_every_10deg": {int(th): round(float(cost[thetas == th][0]), 3) for th in range(-180, 180, 10)}}


def landmark_spread(frames, objs=PHYS_BOTTLES, lo=225.0, hi=2127.9):
    out = {}
    for name, ids in objs.items():
        rows = [(t, o["position_m"]) for t, _, obs in frames if lo <= t - REL0 < hi for o in obs if o["logged_track"] in ids]
        ts = np.array([r[0] for r in rows]); P = np.array([r[1][:2] for r in rows])
        cut = np.flatnonzero(np.diff(ts) > 20) + 1
        meds = np.array([np.median(p, 0) for p in np.split(P, cut)])
        out[name] = {"visits": len(meds), "rms_m": round(float(np.sqrt(np.mean(np.sum((meds - meds.mean(0)) ** 2, 1)))), 4)}
    out["sum_sq"] = round(sum(v["rms_m"] ** 2 for v in out.values() if isinstance(v, dict)), 4)
    return out


# ---------------------------------------------------------------- memory replay
def import_code(ref):
    """Memory code + replay harness of ``ref`` (git archive into a temporary directory, imported from there)."""
    tmp = Path(tempfile.mkdtemp(prefix="gyrohead-code-"))
    tar = tmp / "code.tar"
    subprocess.check_call(["git", "-C", str(ROOT), "archive", "-o", str(tar), ref,
                           "perception/fpga_replay", "tools/memory_quality"])
    with tarfile.open(tar) as t:
        t.extractall(tmp)  # own git archive, trusted
    sys.path.insert(0, str(tmp))
    core = importlib.import_module("perception.fpga_replay.core")
    gates = importlib.import_module("perception.fpga_replay.memory_gates")
    pt = importlib.import_module("perception.fpga_replay.pose_trust")
    rs = importlib.import_module("tools.memory_quality.replay_stream")
    ptr = importlib.import_module("tools.memory_quality.pose_trust_replay")
    for m in (core, gates, pt, rs, ptr):
        assert m.__file__.startswith(str(tmp)), m.__file__
    return {"core": core, "gates": gates, "pose_trust": pt, "rs": rs, "ptr": ptr, "dir": str(tmp)}


def run_memory(code, frames, K, T_bc, image, trust):
    """replay_stream.run_arm, plus a per-observation outcome: created / updated / rejected:<reason> / dropped."""
    core, gates, rs = code["core"], code["gates"], code["rs"]
    cfg = core.Config()
    memory = core.Memory("geometry", cfg)
    rgb, depth = np.zeros((2, 2, 3), np.uint8), np.full((2, 2), np.nan, np.float32)
    w, h = image
    outcomes, applied, first_seen = [], [], {}
    for i, (t, base, obs) in enumerate(frames):
        fid = "replay-%06d" % i
        T_wc = rs.base_matrix(base) @ T_bc
        frame = core.Frame(fid, t, rgb, depth, K, T_wc, True, "replay_wheel_odom", "replay")
        frame.ground_z_m = 0.0
        if trust is not None:
            frame.pose_trusted = bool(trust(t))
        observations = []
        for o in obs:
            p = np.array(o["position_m"], dtype=float)
            z_cam = float(((p - T_wc[:3, 3]) @ T_wc[:3, :3])[2])
            observations.append(core.Observation(
                o["label"], o["score"], p, p - .05, p + .05, o["color"], 1.0, fid, t, tuple(o["bbox"]), "replay",
                extent_m=gates.apparent_extent(o["bbox"], z_cam, K),
                truncated=gates.is_truncated(o["bbox"], (h, w), cfg.edge_margin_px)))
        before = set(memory.tracks)
        result = memory.update_memory(observations, {}, frame)
        rejected = result.get("rejected") or []
        touched = {tid: tr for tid, tr in memory.tracks.items() if tr.last_seen_frame == fid}
        for o in obs:
            hit = next((tid for tid, tr in touched.items()
                        if tr.label == o["label"] and np.allclose(tr.position, o["position_m"])), None)
            if hit is not None:
                kind = "updated" if hit in before else "created"
                first_seen.setdefault(hit, t)
                applied.append((t, o["logged_track"], hit, list(map(float, o["position_m"]))))
            else:
                rr = next((r for r in rejected if r["label"] == o["label"] and
                           np.allclose(r["position_m"], o["position_m"])), None)
                kind = "rejected:" + rr["reason"] if rr is not None else "dropped"
            outcomes.append((t, o["logged_track"], o["label"], kind, hit))
    return memory, applied, outcomes, first_seen


def misplaced_summary(memory, outcomes, first_seen):
    rows = [o for o in outcomes if o[1] in MISPLACED]
    created = sorted({o[4] for o in rows if o[3] == "created"})
    counts = {}
    for t, lt, label, kind, hit in rows:
        if kind == "updated":
            kind = "updated_pre_crash_track" if first_seen.get(hit, math.inf) - REL0 < CRASH_ONSET else "updated_post_crash_track"
        counts[kind] = counts.get(kind, 0) + 1
    detail = {tid: {"label": memory.tracks[tid].label, "first_rel_s": round(first_seen[tid] - REL0, 2),
                    "position_m": [round(v, 3) for v in memory.tracks[tid].position.tolist()],
                    "from_original": sorted({o[1] for o in rows if o[4] == tid})} for tid in created}
    targets = {}
    for t, lt, label, kind, hit in rows:
        if hit is not None and first_seen.get(hit, math.inf) - REL0 < CRASH_ONSET:
            targets.setdefault(hit, {"label": memory.tracks[hit].label, "n": 0,
                                     "first_rel_s": round(first_seen[hit] - REL0, 2)})["n"] += 1
    return {"observations": len(rows), "outcomes": counts, "new_tracks": len(created), "created": detail,
            "absorbed_into_pre_crash_tracks": targets}


def created_after(memory, first_seen, rec_rel):
    return {tid: {"label": memory.tracks[tid].label, "first_rel_s": round(t - REL0, 2)}
            for tid, t in sorted(first_seen.items(), key=lambda kv: kv[1]) if t - REL0 >= rec_rel}


# ---------------------------------------------------------------- main
def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for k, v in DEFAULTS.items():
        p.add_argument("--" + k.replace("_", "-"), type=Path, default=v)
    p.add_argument("--code-ref", default="ceb1b4d")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--posthoc-nobias", action="store_true",
                   help="also evaluate arm fused_nobias (despike only, no static-bias EMA): added AFTER the frozen run "
                        "showed the EMA learning the crash as bias; reported separately, never a frozen verdict")
    a = p.parse_args(argv)
    if a.out.exists():
        raise SystemExit("output exists: %s" % a.out)
    doc = json.loads(a.stream.read_text())
    real = clock_map(doc["clock_fit"], doc["extra_s"])
    ch = load_chassis(a.chassis)
    wh = load_wheel(a.wheel_odom)
    rt = load_rtab(a.rtab_poses)
    wt = real(wh["t_ns"] / 1e9)
    rep = {"inputs": {k: str(getattr(a, k)) for k in DEFAULTS}, "code_ref": a.code_ref,
           "params": {"despike_window": DESPIKE_WINDOW, "despike_thr_deg_s": DESPIKE_THR_CDPS / 100,
                      "bias_still_s": BIAS_STILL_S, "bias_alpha": BIAS_ALPHA, "gap_s": GAP_S}}

    # arms
    arms, gyro_diag = {}, {}
    arms["wheel"] = Arm("wheel", wt, wh["x"], wh["y"], wh["yaw"])
    for mode in ("gyro_raw", "fused") + (("fused_nobias",) if a.posthoc_nobias else ()):
        gt, gc, diag = gyro_cumulative(ch, real, mode)
        psi, n_fallback = heading_on_wheel_rows(wt, wh["yaw"], gt, gc)
        diag["wheel_rows_with_wheel_increment"] = n_fallback
        diag["gyro_end_rel_s"] = round(float(gt[-1] - REL0), 3)
        x, y = trajectory(wh, psi)
        arms[mode] = Arm(mode, wt, x, y, psi)
        gyro_diag[mode] = diag
    rep["gyro"] = gyro_diag

    # harness
    xh, yh = trajectory(wh, wh["yaw"])
    K, T_bc, image = np.array(doc["K"]), np.array(doc["T_base_camera"]), doc["image_size"]
    frames = [(f["t"], f["base"], f["obs"]) for f in doc["frames"]]
    fr = {name: reproject(frames, arm) for name, arm in arms.items()}
    h3 = max((math.dist(o2["position_m"], o["position_m"]) for (_, _, ob), (_, _, ob2) in zip(frames, fr["wheel"])
              for o, o2 in zip(ob, ob2)), default=0.0)
    early = rt["id"] <= 109
    wx, wy, wp = arms["wheel"].pose(rt["t"][early])
    rep["harness"] = {
        "H2_reintegrated_xy_max_err_m": float(max(np.abs(xh - wh["x"]).max(), np.abs(yh - wh["y"]).max())),
        "H3_reproject_wheel_max_err_m": float(h3),
        "H4_rtab_early_max_dxy_m": float(np.hypot(rt["x"][early] - wx, rt["y"][early] - wy).max()),
        "H4_rtab_early_max_dyaw_deg": float(np.degrees(np.abs([wrap(v) for v in rt["yaw"][early] - wp])).max()),
    }

    # 2a crash heading
    t0, t1 = REL0 + CRASH_WINDOW[0], REL0 + CRASH_WINDOW[1]
    crash = {}
    for name, arm in arms.items():
        _, _, ps = arm.pose(np.array([t0, t1]))
        crash[name] = round(float(np.degrees(ps[1] - ps[0])), 2)
    rep["2a_crash_heading_change_deg"] = crash
    rep["2a_despike_vs_raw_in_crash_deg"] = round(crash["fused"] - crash["gyro_raw"], 2)
    rep["2a_theta"] = {name: theta_scan(fr[name], arms[name]) for name in arms}

    # 2c rtabmap loop pairs
    idx = {i: k for k, i in enumerate(rt["id"])}
    pairs = {}
    for name, arm in arms.items():
        res = []
        for i, j in LOOP_PAIRS:
            ta, tb = rt["t"][idx[i]], rt["t"][idx[j]]
            _, _, ps = arm.pose(np.array([ta, tb]))
            res.append(math.degrees(wrap((ps[1] - ps[0]) - (rt["yaw"][idx[j]] - rt["yaw"][idx[i]]))))
        clus = [r for (pr, r) in zip(LOOP_PAIRS, res) if pr in CLUSTER_FIRST]
        _, _, pk = arm.pose(rt["t"])
        absd = np.degrees([wrap(v) for v in rt["yaw"] - (pk - pk[0] + rt["yaw"][0])])
        pairs[name] = {"pairs_deg": [round(r, 2) for r in res], "rms_pairs_deg": round(float(np.sqrt(np.mean(np.square(res)))), 3),
                       "rms_clusters_deg": round(float(np.sqrt(np.mean(np.square(clus)))), 3),
                       "max_abs_deg": round(float(np.max(np.abs(res))), 2),
                       "abs_vs_rtab_keyframes_max_deg": round(float(np.abs(absd).max()), 2),
                       "abs_vs_rtab_keyframes_rms_deg": round(float(np.sqrt(np.mean(absd ** 2))), 2)}
    rep["2c_rtab_loop_pairs"] = pairs
    rep["2c_landmark_spread_exploratory"] = {name: landmark_spread(fr[name]) for name in arms}

    # 2b memory replay
    code = import_code(a.code_ref)
    rep["code_dir"] = code["dir"]
    ptr = code["ptr"]
    gyro_s, wheel_s = ptr.series_from_car(a.chassis, a.wheel_odom, doc["clock_fit"], doc["extra_s"])
    monitor = ptr.run_monitor(gyro_s, wheel_s)
    rep["trust_episodes_rel_s"] = [[round(x, 3), None if y is None else round(y, 3)] for x, y in ptr.episodes(monitor.transitions)]
    recovery = next(e[1] for e in ptr.episodes(monitor.transitions) if e[0] >= CRASH_ONSET)
    mem = {}
    for name in ("wheel", "fused") + (("fused_nobias",) if a.posthoc_nobias else ()):
        for gate in (True, False):
            key = name + ("+trust" if gate else "")
            memory, applied, outcomes, first_seen = run_memory(code, fr[name], K, T_bc, image,
                                                               monitor.trusted_at if gate else None)
            mem[key] = {"tracks_end": len(memory.tracks), "rejected": dict(memory.rejected),
                        "created_after_crash_recovery": created_after(memory, first_seen, recovery),
                        "misplaced_set": misplaced_summary(memory, outcomes, first_seen),
                        "_applied": applied, "_memory": memory}
    # H1 + cross-check with replay_stream.run_arm on the wheel+trust arm
    ref_mem, _, ref_applied = code["rs"].run_arm(code["core"], code["core"].Config(), fr["wheel"], K, T_bc, image, {}, True,
                                                 trust=monitor.trusted_at)
    exp = [2285.7, 2289.3, 2312.1, 2328.6, 3067.9, 3098.0]
    got = mem["wheel+trust"]["created_after_crash_recovery"]
    got_t = sorted(v["first_rel_s"] for v in got.values())
    rep["harness"]["H1_wheel_trust_created_after_recovery"] = got
    rep["harness"]["H1_pass"] = (len(got_t) == 6 and all(abs(x - y) <= 0.1 for x, y in zip(got_t, exp)) and
                                 sorted(v["label"] for v in got.values()) == ["backpack"] * 5 + ["cell phone"])
    rep["harness"]["run_memory_equals_run_arm"] = ([x[:3] for x in mem["wheel+trust"]["_applied"]] ==
                                                   [x[:3] for x in ref_applied])
    # memory-quality picture on the trust arms (report only; section 4 2b)
    rs = code["rs"]
    points = {"find1": 1792092486.9923933, "find2": 1792092813.9154556, "kidnap": 1792092965.0}
    mq = {}
    for key in ("fused+trust", "wheel+trust"):
        mq[key] = rs.run_arm(code["core"], code["core"].Config(), fr[key.split("+")[0]], K, T_bc, image, points, True,
                             trust=monitor.trusted_at)
    ev = rs.evaluate(mq, points["kidnap"], points["find1"], code["core"].Config().below_ground_tol_m)
    rep["2b_memory_quality_report_only"] = {
        k: {"find1_bottle_tracks": len(v["bottle_tracks_find1"]), "find1_true_bottles": v["find1_true_bottles"],
            "bins_in_bottle_or_cup_tracks": v["identity_full_run"]["bins"],
            "one_track_per_bottle_before_crash": {p_: v["identity_before_kidnap"][p_] for p_ in ("milk", "nongfu", "redcap")},
            "mixed_before_crash": v["mixed_tracks_before_kidnap"], "applied_below_0.03": v["applied_below_tol"],
            "tracks_end": v["tracks_end"]} for k, v in ev.items()}
    rep["2b_memory"] = {k: {kk: vv for kk, vv in v.items() if not kk.startswith("_")} for k, v in mem.items()}

    # verdicts (doc section 4)
    th_f, th_w = rep["2a_theta"]["fused"], rep["2a_theta"]["wheel"]
    expect = crash["fused"] - crash["wheel"]
    rep["criteria"] = {
        "harness_all": bool(rep["harness"]["H1_pass"] and rep["harness"]["H2_reintegrated_xy_max_err_m"] <= 1e-9 and
                            rep["harness"]["H3_reproject_wheel_max_err_m"] <= 1e-9 and
                            rep["harness"]["H4_rtab_early_max_dxy_m"] <= 0.002 and
                            rep["harness"]["H4_rtab_early_max_dyaw_deg"] <= 0.1),
        "2a_fused_theta_le_20": {"theta": th_f["theta_star_deg"], "distinct": th_f["distinct"],
                                 "pass": abs(th_f["theta_star_deg"]) <= 20 and th_f["distinct"]},
        "2a_wheel_control": {"theta": th_w["theta_star_deg"], "expected": round(expect, 1), "distinct": th_w["distinct"],
                             "pass": abs(math.degrees(wrap(math.radians(th_w["theta_star_deg"] - expect)))) <= 20
                             and th_w["distinct"]},
        "2b_fused_trust_new_tracks_le_1": {"value": mem["fused+trust"]["misplaced_set"]["new_tracks"],
                                           "wheel_trust": mem["wheel+trust"]["misplaced_set"]["new_tracks"],
                                           "pass": mem["fused+trust"]["misplaced_set"]["new_tracks"] <= 1},
        "2c_fused_rms_le_wheel": {"fused": pairs["fused"]["rms_pairs_deg"], "wheel": pairs["wheel"]["rms_pairs_deg"],
                                  "pass": pairs["fused"]["rms_pairs_deg"] <= pairs["wheel"]["rms_pairs_deg"]},
    }
    if a.posthoc_nobias:
        th_n = rep["2a_theta"]["fused_nobias"]
        rep["posthoc_fused_nobias_same_tests_not_a_verdict"] = {
            "2a_theta": {"theta": th_n["theta_star_deg"], "distinct": th_n["distinct"],
                         "within_20": abs(th_n["theta_star_deg"]) <= 20 and th_n["distinct"]},
            "2b_new_tracks": mem["fused_nobias+trust"]["misplaced_set"]["new_tracks"],
            "2c_rms_pairs": pairs["fused_nobias"]["rms_pairs_deg"],
            "crash_heading_change_deg": crash["fused_nobias"]}
    text = json.dumps(rep, ensure_ascii=False, indent=1, default=float)
    with a.out.open("x") as s:
        s.write(text + "\n")
    print(json.dumps(rep["criteria"], ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
