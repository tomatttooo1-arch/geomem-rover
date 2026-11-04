#!/usr/bin/env python3
"""Heartbeat-gap dig (2026-10-16, docs/design/HEARTBEAT_GAP_DIG_20261016.md). Offline only, no ROS.

Why the motion permit drops while the car drives ("一顿一顿"): the KV260 PL permit is fresh for
TIMEOUT = 200 ms after each geometry-core COMPLETION edge (CAPTURE_AGE is kept 0 by every feed,
host/kv_regs.py; m7_permit_axil.v), the MCU latches when it sees no heartbeat edge for 150 ms
(plan A, D-040; 30 ms before). So the quantity that decides a latch is the interval between two
consecutive geometry completions, not the acquisition interval.

Per run this script reads the producer's per-frame record (frames.jsonl, --stats-out), and
optionally the chassis driver record (chassis_record.jsonl, same CLOCK_MONOTONIC_RAW) and:
  1. completion proxy C = acquired + mailbox_wait + tf + geometry (the KV answer at the Nano;
     the PL edge is ~15-20 ms earlier, constant to a few ms) for every frame that called the KV;
  2. classifies every completion gap > 200 ms by a fixed priority chain (exclusive classes);
  3. finds stall regimes (60 s buckets with > 5 late depth frames / min);
  4. matches MCU latch rises to the gap that contains (latch - 250 ms);
  5. counterfactuals: PL window 300/350 ms, over-range frames not refused, no head-of-line
     blocking (re-simulated gate + one-slot mailbox + serial worker), loss sensitivity.

Usage:
  hb_gap_dig.py --run NAME FRAMES_JSONL [--status CHASSIS_RECORD_JSONL] [--exclude T0:T1 ...]
                [--run ...] --out OUT.json
  T0:T1 are seconds from the run's first completion (e.g. an e-stop test window).
Python 3.8 compatible; standard library only.
"""
import argparse
import bisect
import collections
import json
import math
import random
import statistics
import sys

P_NOM_MS = 33.4          # camera period (producer summary input.period_ms 33.40)
LATE_MS = 80.0           # a depth frame is "late" when its arrival delay exceeds the local floor by this
FLOOR_HALF = 50          # local floor of (arrival - stamp): min over +-50 processed rows (~+-5 s)
LIM_PL_MS = 200.0        # PL TIMEOUT (m7_permit_axil TIMEOUT_CYCLES 2e7 @ 100 MHz)
THR_PRE_A_MS = 230.0     # MCU 30 ms rule (before D-040): 200 + 30
THR_POST_A_MS = 337.0    # MCU 150 ms rule, on the C basis: board-fitted (latches start 335-357 ms after C)
REGIME_LATE_PER_MIN = 5.0


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))] if xs else None


def load_frames(path):
    rows, ev = [], []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            (ev if "event" in d else rows).append(d)
    for r in rows:
        r["ran"] = r.get("rtt_ms") is not None
        r["lat"] = r["mailbox_wait_ms"] + r["tf_ms"] + r["geometry_ms"]
        r["core"] = r["tf_ms"] + r["geometry_ms"]
        r["C"] = r["acquired_ns"] + 1e6 * r["lat"]
        r["d0"] = r["acquired_ns"] - r["depth_stamp_ns"]     # RAW - REALTIME: constant + delivery delay
    n = len(rows)
    ds = [r["d0"] for r in rows]
    for i, r in enumerate(rows):
        r["late"] = (ds[i] - min(ds[max(0, i - FLOOR_HALF):i + FLOOR_HALF + 1])) / 1e6
    return rows, ev


def refuse_kind(r):
    nr = r.get("nav_reason") or ""
    if nr.startswith("input_refused:depth appears outside indoor metre range"):
        return "over_range"
    if nr.startswith("no_valid_depth"):
        return "no_valid_depth"
    return "refused_other:" + nr[:30]


def load_status(path):
    """MCU status rows: (t_ns, heartbeat_seen, stop_latched, left, right) and tx rows."""
    rx, tx = [], []
    with open(path) as fh:
        for line in fh:
            d = json.loads(line)
            if d.get("dir") == "rx":
                rx.append((d["t_ns"], d["heartbeat_seen"], d["stop_latched"], d["left_count"], d["right_count"]))
            elif d.get("dir") == "tx":
                tx.append((d["t_ns"], d["enable"], abs(d.get("v_mm_s", 0)) + abs(d.get("w_mrad_s", 0)) > 0))
    return rx, tx


class Run(object):
    def __init__(self, name, frames, status=None, exclude=()):
        self.name = name
        self.rows, self.ev = load_frames(frames)
        self.ran = [r for r in self.rows if r["ran"]]
        self.t0 = self.ran[0]["C"]
        self.span_min = (self.ran[-1]["C"] - self.t0) / 6e10
        self.exclude = [(self.t0 + a * 1e9, self.t0 + b * 1e9) for a, b in exclude]
        self.gaps_in = sorted((e for e in self.ev if e["event"] == "input_gap"), key=lambda e: e["stamp_ns"])
        self.gap_stamps = [e["stamp_ns"] for e in self.gaps_in]
        self.tfmiss = sorted(e["acquired_ns"] for e in self.ev if e["event"] == "tf_missing")
        self.status = load_status(status) if status else None
        self.med_lat = statistics.median(r["lat"] for r in self.ran)
        self.med_core = statistics.median(r["core"] for r in self.ran)
        self.med_geo = statistics.median(r["geometry_ms"] for r in self.ran)
        self.regimes()

    # ---- regimes ---------------------------------------------------------------------------
    def regimes(self, bucket_s=60):
        nb = int(self.span_min * 60 / bucket_s) + 1
        late = [0] * nb
        for r in self.rows:
            if r["late"] > LATE_MS:
                k = int((r["acquired_ns"] - self.t0) / 1e9 / bucket_s)
                if 0 <= k < nb:
                    late[k] += 1
        self.bucket_s = bucket_s
        self.on = [x * 60.0 / bucket_s > REGIME_LATE_PER_MIN for x in late]
        self.late_per_bucket = late
        first_on = next((i for i, x in enumerate(self.on) if x), None)
        self.first_on_s = None if first_on is None else first_on * bucket_s

    def in_on(self, t_ns):
        k = int((t_ns - self.t0) / 1e9 / self.bucket_s)
        return 0 <= k < len(self.on) and self.on[k]

    def excluded(self, a, b):
        return any(not (b < x0 or a > x1) for x0, x1 in self.exclude)

    # ---- gap classification ----------------------------------------------------------------
    def classify(self):
        rows = self.rows
        idx = [i for i, r in enumerate(rows) if r["ran"]]
        out = []
        for ia, ib in zip(idx, idx[1:]):
            A, B = rows[ia], rows[ib]
            g = (B["C"] - A["C"]) / 1e6
            if g <= LIM_PL_MS:
                continue
            between = rows[ia + 1:ib]
            refused = collections.Counter(refuse_kind(r) for r in between)
            lo = bisect.bisect_right(self.tfmiss, A["acquired_ns"])
            hi = bisect.bisect_left(self.tfmiss, B["acquired_ns"])
            ntf = hi - lo
            lo = bisect.bisect_right(self.gap_stamps, A["depth_stamp_ns"])
            hi = bisect.bisect_right(self.gap_stamps, B["depth_stamp_ns"])
            miss = sum(e["missed"] for e in self.gaps_in[lo:hi])
            any_late = max([B["late"]] + [r["late"] for r in between])
            dst = (B["depth_stamp_ns"] - A["depth_stamp_ns"]) / 1e6
            dgeo = B["geometry_ms"] - self.med_geo
            dmb = B["mailbox_wait_ms"]
            dtf = B["tf_ms"]
            if self.excluded(A["C"], B["C"]):
                cls = "excluded_window"
            elif ntf:
                cls = "tf_missing"
            elif refused.get("no_valid_depth"):
                cls = "no_valid_depth"
            elif refused.get("over_range"):
                cls = "over_range"
            elif sum(refused.values()):
                cls = "refused_other"
            elif miss and any_late > LATE_MS:
                cls = "hol_after_loss"
            elif any_late > LATE_MS:
                cls = "late_no_loss"
            elif miss:
                cls = "loss_no_hol"
            elif dgeo > 40:
                cls = "kv_rtt_spike"
            elif dmb + dtf > 40:
                cls = "producer_slow"
            else:
                cls = "residual"
            out.append({"t_s": (A["C"] - self.t0) / 1e9, "g": g, "cls": cls, "dst": dst, "miss": miss,
                        "late_ms": any_late, "refused": dict(refused), "tf_missing": ntf,
                        "dgeo": dgeo, "mailbox": dmb, "tf_ms": dtf, "on": self.in_on(A["C"]),
                        "C_a": A["C"], "C_b": B["C"]})
        self.gap_list = out
        return out

    # ---- completion series -> counts ---------------------------------------------------------
    def counts(self, comps, shift=0.0):
        g = [(b - a) / 1e6 for a, b in zip(comps, comps[1:])
             if not self.excluded(a, b)]
        span = self.span_min
        res = {}
        for name, thr in (("gt200", LIM_PL_MS + shift), ("pre_a_gt230", THR_PRE_A_MS + shift),
                          ("post_a_gt337", THR_POST_A_MS + shift)):
            n = sum(x > thr for x in g)
            res[name] = {"n": n, "per_min": round(n / span, 2)}
        res["max_ms"] = round(max(g), 1) if g else None
        return res

    def recorded_comps(self):
        return [r["C"] for r in self.ran]

    def comps_over_range_fixed(self):
        c = [r["C"] for r in self.ran]
        for r in self.rows:
            if not r["ran"] and refuse_kind(r) == "over_range":
                c.append(r["acquired_ns"] + 1e6 * (r["mailbox_wait_ms"] + self.med_core))
        return sorted(c)

    # ---- re-simulation of gate + one-slot mailbox + serial worker --------------------------
    def resim(self, hol=False, extra_loss=0.0, k=3, fix_over_range=False, seed=1, gate="stamp",
              segment=None):
        """Camera stamps on the measured grid; frames missing as recorded (input_gap) plus iid
        extra_loss; arrival = stamp + local floor + typical jitter (no head-of-line blocking) unless
        hol=True (then processed rows keep their recorded arrival -- only meaningful for checks).
        Service time per frame = recorded tf+geometry of that stamp if it was processed, else the
        median; refused frames (recorded) produce no completion unless fix_over_range."""
        rng = random.Random(seed)
        rows = self.rows
        by_stamp = {r["depth_stamp_ns"]: r for r in rows}
        tf_stamps = set(e["depth_stamp_ns"] for e in self.ev if e["event"] == "tf_missing")
        missing = set()
        for e in self.gaps_in:
            step = e["stamp_ns"] - e["prev_stamp_ns"]
            n = int(e["missed"])
            for j in range(1, n + 1):
                missing.add(e["prev_stamp_ns"] + int(round(j * step / (n + 1))))
        s0 = rows[0]["depth_stamp_ns"]
        s1 = rows[-1]["depth_stamp_ns"]
        if segment is not None:
            s0, s1 = segment
        P = P_NOM_MS * 1e6
        # floor of d0 as a function of stamp (piecewise from processed rows)
        st_sorted = [r["depth_stamp_ns"] for r in rows]
        floor = []
        ds = [r["d0"] for r in rows]
        for i in range(len(rows)):
            floor.append(min(ds[max(0, i - FLOOR_HALF):i + FLOOR_HALF + 1]))
        ontime_jit = [r["late"] for r in rows if r["late"] < 40]
        jit_med = statistics.median(ontime_jit) if ontime_jit else 0.0
        # build the frame list
        frames = []
        n = int(round((s1 - s0) / P))
        miss_sorted = sorted(missing)
        for j in range(n + 1):
            s = s0 + j * P
            # snap to a recorded stamp if one is within 5 ms
            m = bisect.bisect_left(st_sorted, s - 5e6)
            if m < len(st_sorted) and abs(st_sorted[m] - s) < 5e6:
                s = st_sorted[m]
            mm = bisect.bisect_left(miss_sorted, s - 5e6)
            if mm < len(miss_sorted) and abs(miss_sorted[mm] - s) < 5e6:
                continue
            if extra_loss and rng.random() < extra_loss:
                continue
            fi = min(len(rows) - 1, bisect.bisect_left(st_sorted, s))
            r = by_stamp.get(s)
            if hol and r is not None:
                arr = r["acquired_ns"]
            else:
                arr = s + floor[fi] + jit_med * 1e6
            frames.append((s, arr, r))
        frames.sort(key=lambda x: x[1])
        # gate
        released = []
        last = None
        last_arr = None
        thr = (k - 0.5) * P
        for s, arr, r in frames:
            if gate == "stamp":
                if last is None or s - last >= thr:
                    last = s
                    released.append((s, arr, r))
            else:   # arrival gate (before 2026-10-15): 0.9 / rate_hz on the arrival
                if last_arr is None or arr - last_arr >= 0.9 * 100e6:
                    last_arr = arr
                    released.append((s, arr, r))
        # one-slot mailbox + serial worker
        comps = []
        busy_until = 0.0
        slot = None
        i = 0
        t = 0.0
        events = released
        pending = list(events)
        pi = 0
        while pi < len(pending) or slot is not None:
            # next arrival or worker free
            if slot is None:
                s, arr, r = pending[pi]
                pi += 1
                if arr >= busy_until:
                    start = arr
                    item = (s, arr, r)
                else:
                    slot = (s, arr, r)
                    continue
            else:
                # worker frees at busy_until; newer arrivals before that overwrite the slot
                while pi < len(pending) and pending[pi][1] < busy_until:
                    slot = pending[pi]
                    pi += 1
                item = slot
                slot = None
                start = max(busy_until, item[1])
            s, arr, r = item
            if r is not None:
                core = r["core"] if r["ran"] else (r["tf_ms"] + r["geometry_ms"])
                refused = (not r["ran"]) and not (fix_over_range and refuse_kind(r) == "over_range")
                if (not r["ran"]) and fix_over_range and refuse_kind(r) == "over_range":
                    core = self.med_core
            else:
                core = self.med_core
                refused = s in tf_stamps
            busy_until = start + core * 1e6
            if not refused:
                comps.append(busy_until)
        return sorted(comps)


def summarise_run(run, sims=True):
    gaps = run.classify()
    out = {"name": run.name, "span_min": round(run.span_min, 2), "completions": len(run.ran),
           "first_on_s": run.first_on_s,
           "on_fraction": round(sum(run.on) / max(1, len(run.on)), 3),
           "regime_timeline_60s": "".join("#" if x else "." for x in run.on),
           "median_ms": {"lat": round(run.med_lat, 1), "geometry": round(run.med_geo, 1)},
           "input_gaps": len(run.gaps_in),
           "late_frames": sum(1 for r in run.rows if r["late"] > LATE_MS)}
    # head-of-line signature: first frame after every input gap late?
    pst = {}
    for r in run.rows:
        pst[r["depth_stamp_ns"]] = r
    sig = 0
    acq = [r["acquired_ns"] for r in run.rows]
    for e in run.gaps_in:
        i = min(len(run.rows) - 1, bisect.bisect_left(acq, e["raw_ns"]))
        lo, hi = max(0, i - FLOOR_HALF), i + FLOOR_HALF + 1
        fl = min(r["d0"] for r in run.rows[lo:hi])
        sig += ((e["raw_ns"] - e["stamp_ns"]) - fl) / 1e6 > LATE_MS
    out["hol_signature"] = {"input_gaps": len(run.gaps_in), "first_frame_after_gap_late": sig}
    late = sorted(r["late"] for r in run.rows if r["late"] > LATE_MS)
    out["late_excess_ms_p10_p50_p90"] = [round(q(late, p), 1) for p in (0.1, 0.5, 0.9)] if late else None
    # classes x thresholds
    tab = {}
    for thr_name, thr in (("gt200", LIM_PL_MS), ("pre_a_gt230", THR_PRE_A_MS), ("post_a_gt337", THR_POST_A_MS)):
        c = collections.Counter(x["cls"] for x in gaps if x["g"] > thr)
        tab[thr_name] = dict(c.most_common())
        tab[thr_name + "_in_on_regime"] = sum(1 for x in gaps if x["g"] > thr and x["on"])
    out["classes"] = tab
    out["gap_ms_by_class"] = {}
    for cls in sorted(set(x["cls"] for x in gaps)):
        gs = [x["g"] for x in gaps if x["cls"] == cls]
        out["gap_ms_by_class"][cls] = {"n": len(gs), "min": round(min(gs)), "p50": round(q(gs, .5)),
                                       "max": round(max(gs))}
    out["recorded"] = run.counts(run.recorded_comps())
    out["on_off"] = {}
    for flag in (True, False):
        comps = [r["C"] for r in run.ran if run.in_on(r["C"]) == flag]
        minutes = sum(1 for x in run.on if x == flag) * run.bucket_s / 60.0
        g = [(b - a) / 1e6 for a, b in zip(comps, comps[1:]) if (b - a) < 5e9 and not run.excluded(a, b)]
        out["on_off"]["on" if flag else "off"] = {
            "minutes": minutes,
            "gt200_per_min": round(sum(x > LIM_PL_MS for x in g) / max(minutes, 1e-9), 2),
            "post_a_per_min": round(sum(x > THR_POST_A_MS for x in g) / max(minutes, 1e-9), 2)}
    if run.status is not None:
        out["latches"] = match_latches(run)
    if sims:
        s = {}
        s["pl_window_300"] = run.counts(run.recorded_comps(), shift=100.0)
        s["pl_window_350"] = run.counts(run.recorded_comps(), shift=150.0)
        s["over_range_not_refused"] = run.counts(run.comps_over_range_fixed())
        s["resim_no_hol"] = run.counts(run.resim())
        s["resim_no_hol_over_range_fixed"] = run.counts(run.resim(fix_over_range=True))
        for loss in (0.05, 0.15, 0.30):
            s["resim_no_hol_extra_loss_%d" % int(loss * 100)] = run.counts(run.resim(extra_loss=loss))
        s["resim_no_hol_k2"] = run.counts(run.resim(k=2))
        out["sims"] = s
        # pipeline-model control: OFF-regime buckets, recorded vs re-simulated (no losses there)
        out["model_check_off_regime"] = model_check(run)
    return out


def model_check(run):
    """Recorded vs re-simulated gap counts in the OFF regime only (no stalls there): checks the
    gate + mailbox + worker model, not the loss model."""
    rec = [r["C"] for r in run.ran if not run.in_on(r["C"])]
    sim = [c for c in run.resim() if not run.in_on(c)]

    def cnt(c):
        g = [(b - a) / 1e6 for a, b in zip(c, c[1:]) if (b - a) < 1e9 and not run.excluded(a, b)]
        return {"n_gaps": len(g), "p50": round(q(g, .5), 1) if g else None,
                "p99": round(q(g, .99), 1) if g else None, "gt200": sum(x > 200 for x in g),
                "gt150": sum(x > 150 for x in g)}
    return {"recorded": cnt(rec), "resim": cnt(sim)}


def match_latches(run):
    rx, tx = run.status
    C = run.recorded_comps()
    rises = []
    for a, b in zip(rx, rx[1:]):
        if (not a[2]) and b[2]:
            rises.append(b[0])
    res = []
    for t in rises:
        i = bisect.bisect_right(C, t - 250e6) - 1
        if i < 0 or i + 1 >= len(C):
            res.append({"t_s": (t - run.t0) / 1e9, "kind": "outside_frames"})
            continue
        g = (C[i + 1] - C[i]) / 1e6
        since = (t - C[i]) / 1e6
        cls = None
        for x in run.gap_list:
            if abs(x["C_a"] - C[i]) < 1 and x["g"] == g:
                cls = x["cls"]
        k = bisect.bisect_left([x[0] for x in rx], t)
        moving = rx[min(k, len(rx) - 1)][3] != rx[max(0, k - 50)][3]
        ex = run.excluded(t - 1e9, t)
        res.append({"t_s": round((t - run.t0) / 1e9, 1), "since_C_ms": round(since), "gap_ms": round(g),
                    "cls": "excluded_window" if ex else (cls or ("no_gap" if g <= LIM_PL_MS else "?")),
                    "moving": moving})
    # confusion vs the post-A threshold on the C basis
    pred = [(a, b) for a, b in zip(C, C[1:]) if (b - a) / 1e6 > THR_POST_A_MS and not run.excluded(a, b)]
    hit = 0
    rs = sorted(rises)
    for a, b in pred:
        j = bisect.bisect_left(rs, a + 280e6)
        hit += j < len(rs) and rs[j] <= b + 60e6
    obs = [x for x in res if x.get("kind") != "outside_frames" and x["cls"] != "excluded_window"]
    since = [x["since_C_ms"] for x in obs if x["since_C_ms"] < 2000]
    return {"rises": len(rises), "list": res,
            "since_C_ms_min_p50_max": [min(since), q(since, .5), max(since)] if since else None,
            "predicted_post_a": len(pred), "predicted_with_latch": hit,
            "observed_not_excluded": len(obs)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", nargs=2, action="append", metavar=("NAME", "FRAMES"), required=True)
    ap.add_argument("--status", action="append", default=[], metavar="NAME=CHASSIS_RECORD")
    ap.add_argument("--exclude", action="append", default=[], metavar="NAME=T0:T1")
    ap.add_argument("--arrival-gate-control", nargs=2, metavar=("NAME", "FRAMES"),
                    help="10-15 best-effort arm (arrival gate): recorded vs re-simulated, loss model check")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    status = dict(s.split("=", 1) for s in a.status)
    excl = collections.defaultdict(list)
    for e in a.exclude:
        n, w = e.split("=", 1)
        t0, t1 = w.split(":")
        excl[n].append((float(t0), float(t1)))
    result = {"constants": {"late_ms": LATE_MS, "pl_ms": LIM_PL_MS, "pre_a_ms": THR_PRE_A_MS,
                            "post_a_ms_C_basis": THR_POST_A_MS, "regime_late_per_min": REGIME_LATE_PER_MIN},
              "runs": []}
    for name, frames in a.run:
        run = Run(name, frames, status.get(name), excl.get(name, ()))
        s = summarise_run(run)
        result["runs"].append(s)
        print(json.dumps({k: s[k] for k in ("name", "span_min", "recorded", "classes", "first_on_s",
                                            "on_fraction")}, ensure_ascii=False))
        sys.stdout.flush()
    if a.arrival_gate_control:
        name, frames = a.arrival_gate_control
        run = Run(name, frames)
        run.classify()
        result["arrival_gate_control"] = {
            "name": name, "span_min": round(run.span_min, 2), "input_gaps": len(run.gaps_in),
            "recorded": run.counts(run.recorded_comps()),
            "resim_arrival_gate_recorded_losses": run.counts(run.resim(gate="arrival")),
            "resim_stamp_gate_recorded_losses": run.counts(run.resim(gate="stamp"))}
    with open(a.out, "w") as fh:
        json.dump(result, fh, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
