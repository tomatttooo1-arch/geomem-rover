#!/usr/bin/env python3
"""Same-input gate: rolling map fused on the KV260 (remote_pl) vs on the Nano CPU (v3 L2, 2026-10-26).

Both arms are tools/car_geo_producer/equivalence.py's reference arm (M7Pipeline + FollowCameraGrid
32x32x10 + RollingFusion, synthetic odom 0.2 m/s 10 Hz, recommended mount):
  cpu arm     frame core = g++ on this host/Nano, rolling = RollingKernelHost (the car today)
  subject arm frame core = remote_pl (the KV260 frame core), rolling = RemotePlRollingBackend
              (ROLL, zero copy on the frame core's buffer; inline resend if not held)
Per frame: (0) the block, cell by cell (a frame-core difference is reported apart); (1) the
fusion result, epoch / delta_seq / window base / changes and the raw header[24]; every
``--snapshot-every`` frames the whole map (DUMP) and 256 queries. Nano-side cost per integrate:
thread CPU time (time.thread_time_ns) and wall time of the backend call, both arms (median,
p90, max); the subject's wall time includes the round trip.
``--fake``: spawns kv_server --core pl --device mock --wait poll --rolling pl (host mock of
both cores). ``--server H:P``: a running kv_server --rolling pl|cpu (the board).
Exit 0 only when every compared item is equal; with --tamper-every N, exit 0 only when the tamper
control passes as frozen (every tampered frame: block AND same-frame delta differ). There is no
--expect-fail flag (fuse/gates/run_py_gates.sh line 13 passed one; that call died in argparse).
--tamper-mode / --tamper-log: G4 cause finding, see fuse/RESULTS.md "G4 查因".
Usage: fusion_equivalence.py --prep DIR --out NEWDIR (--fake | --server H:P) [--frames N]
"""
import argparse
import threading
import json
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


class Timed(object):
    """Wraps a rolling backend; times integrate (thread CPU + wall)."""

    def __init__(self, inner):
        self.inner, self.config = inner, inner.config
        self.cpu_ns, self.wall_ns = [], []

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def integrate(self, *a, **kw):
        c0, w0 = time.thread_time_ns(), time.monotonic_ns()
        r = self.inner.integrate(*a, **kw)
        self.cpu_ns.append(time.thread_time_ns() - c0)
        self.wall_ns.append(time.monotonic_ns() - w0)
        return r


def stats(v):
    if not v:
        return None
    a = np.asarray(v, dtype=np.float64) / 1e6
    return {"n": len(v), "median_ms": round(float(np.median(a)), 4), "p90_ms": round(float(np.percentile(a, 90)), 4),
            "max_ms": round(float(a.max()), 4)}


def spawn_fake(out_dir):
    # stderr to a file and stdout drained by a thread: an undrained PIPE fills
    # (64 KiB of server log) and blocks the server mid-run (2026-10-26: every
    # frame request then timed out).
    err = open(os.path.join(out_dir, "fake_kv_server.stderr"), "w")
    proc = subprocess.Popen([sys.executable, os.path.join(ROOT, "tools/kv_link/kv_server.py"), "--bind", "127.0.0.1",
                             "--port", "0", "--max-idle-s", "600", "--core", "pl", "--device", "mock",
                             "--wait", "poll", "--rolling", "pl", "--chunk-bytes", "1400"],
                            stdout=subprocess.PIPE, stderr=err, text=True,
                            env=dict(os.environ, PYTHONPATH=ROOT))
    line = proc.stdout.readline()
    if not line:
        raise SystemExit("fake kv_server did not start: see %s" % err.name)
    out = open(os.path.join(out_dir, "fake_kv_server.stdout"), "w")
    out.write(line)

    def drain():
        for rest in proc.stdout:
            out.write(rest)
        out.close()
    threading.Thread(target=drain, daemon=True).start()
    return proc, ("127.0.0.1", json.loads(line)["port"])


class TamperLog(object):
    """G4 cause finding (2026-10-26): picks/applies the tamper and logs it cell by cell.

    The rolling core updates every cell independently (HIT +32 clamp 96, MISS -12 clamp -32,
    flag 0 = no-op) and lists a cell in the delta only when its tri-state (OCC >= 16,
    FREE <= -16) changes, so the per-cell pre/post log-odds of both arms (QUERY) predict
    whether the tampered frame's delta can differ. The predictions are checked against the
    compared deltas."""

    def __init__(self, out, roll_a, rb, fus_b, rng, mode):
        self.roll_a, self.rb, self.fus_b, self.rng, self.mode = roll_a, rb, fus_b, rng, mode
        self.rng2 = np.random.default_rng(7)
        self.dims = tuple(roll_a.config.dims)
        self.f = open(os.path.join(out, "tamper.jsonl"), "w")
        self.fd = open(os.path.join(out, "result_diff.jsonl"), "w")
        self.rows, self.pending, self.tampered_cells = [], [], set()
        self.snap_frames = []  # (k, chain snapshot differs)
        from host.m7_rolling import H
        self.hname = {v: n for n, v in H.items()}
        for n_, base in (("BASE", H["BASE"]), ("OFFSET", H["OFFSET"])):
            for j in range(3):
                self.hname[base + j] = "%s%d" % (n_, j)

    def _q(self, backend, coords):
        coords = np.asarray(coords, dtype=np.int64).reshape(-1, 3)
        out_s, out_l, out_o = [], [], []
        for i in range(0, len(coords), 1024):
            r, (s, l, o) = backend.query(coords[i:i + 1024])
            if not r.ok:
                s, l, o = np.zeros(len(coords[i:i + 1024]), np.uint8), np.zeros(len(coords[i:i + 1024]), np.int8), \
                    np.ones(len(coords[i:i + 1024]), bool)
            out_s.append(s), out_l.append(l), out_o.append(o)
        return np.concatenate(out_s), np.concatenate(out_l), np.concatenate(out_o)

    def tamper(self, k, na, nb, robot):
        f = nb.payload["voxel_flags"]
        flat = f.ravel()
        same_before = bool(np.array_equal(na.payload["voxel_flags"], f))
        ob = self.fus_b.lattice_base(nb.payload["grid"])
        wb = self.fus_b.window_base_for(robot)
        zz, yy, xx = np.unravel_index(np.arange(f.size), f.shape)
        world = np.stack([ob[0] + xx, ob[1] + yy, ob[2] + zz], axis=1)
        inwin = np.all((world >= np.asarray(wb)) & (world < np.asarray(wb) + np.asarray(self.dims)), axis=1)
        n_eligible = None
        if self.mode == "random":
            i = int(self.rng.integers(0, f.size))
            new = (int(flat[i]) + 1) % 3
        else:
            cand = np.flatnonzero((flat == 0) & inwin)
            cand = np.asarray([c for c in cand if tuple(world[c]) not in self.tampered_cells], dtype=np.int64)
            s, l, o = self._q(self.rb, world[cand])
            pre_l = np.where(o, 0, l.astype(np.int32))  # outside the pre-shift window: cleared to 0
            ok = (pre_l > -16) & (pre_l < 16)
            cand = cand[ok]
            n_eligible = int(cand.size)
            if not n_eligible:
                raise RuntimeError("threshold tamper: no eligible cell at frame %d" % k)
            i = int(cand[int(self.rng2.integers(0, cand.size))])
            new = 2
        old = int(flat[i])
        counts = [int(np.count_nonzero(flat == v)) for v in (0, 1, 2)]
        c = tuple(int(v) for v in world[i])
        pa, pb = self._q(self.roll_a, [c]), self._q(self.rb, [c])
        flat[i] = new
        self.tampered_cells.add(c)
        z, y, x = (int(v) for v in np.unravel_index(i, f.shape))
        return {"k": k, "mode": self.mode, "index": i, "zyx": [z, y, x], "cell": list(c), "old": old, "new": new,
                "in_window_post": bool(inwin[i]), "block_equal_before_tamper": same_before,
                "n_eligible": n_eligible, "block_flag_counts_before": counts,
                "pre_cpu": [int(pa[0][0]), int(pa[1][0]), bool(pa[2][0])],
                "pre_subject": [int(pb[0][0]), int(pb[1][0]), bool(pb[2][0])]}

    def after_frame(self, k, rec, ra, rb_, bad):
        sa = set(ra.delta.changes) if ra.accepted else set()
        sb = set(rb_.delta.changes) if rb_.accepted else set()
        diffcells = sorted(set(t[:3] for t in sa ^ sb))
        if diffcells:
            self.fd.write(json.dumps({"k": k, "tamper_frame": rec is not None,
                                      "cells": [list(map(int, c)) for c in diffcells],
                                      "all_previously_tampered_or_now": all(tuple(c) in self.tampered_cells
                                                                            for c in diffcells)}) + "\n")
        if rec is None:
            return
        c = tuple(rec["cell"])
        qa, qb = self._q(self.roll_a, [c]), self._q(self.rb, [c])
        rec["post_cpu"] = [int(qa[0][0]), int(qa[1][0]), bool(qa[2][0])]
        rec["post_subject"] = [int(qb[0][0]), int(qb[1][0]), bool(qb[2][0])]
        full_clear = bool(ra.kernel is not None and ra.kernel.shift_mask & 8)

        def eff_pre(p):  # tri-state right after this frame's shift
            return 0 if (p[2] or full_clear) else p[0]
        pres_a = eff_pre(rec["pre_cpu"]) != rec["post_cpu"][0]
        pres_b = eff_pre(rec["pre_subject"]) != rec["post_subject"][0]
        rec["pred_delta_differs"] = bool(pres_a != pres_b or (pres_a and rec["post_cpu"][0] != rec["post_subject"][0]))
        rec["cell_in_delta_cpu"] = any(t[:3] == c for t in sa)
        rec["cell_in_delta_subject"] = any(t[:3] == c for t in sb)
        rec["delta_diff_cells"] = [list(map(int, x)) for x in diffcells]
        rec["result_differs"] = "result" in bad
        rec["block_differs"] = "block" in bad
        rec["n_delta"] = [len(sa), len(sb)]
        if ra.kernel is not None and rb_.kernel is not None:
            hd = np.flatnonzero(ra.kernel.header != rb_.kernel.header)
            rec["header_diff"] = {self.hname.get(int(j), str(int(j))): [int(ra.kernel.header[j]), int(rb_.kernel.header[j])]
                                  for j in hd}
            rec["subject_delta_overflow"] = int(rb_.kernel.header[23])
        rec["header_differs"] = "header" in bad
        rec["logodds_differs_now"] = rec["post_cpu"][1] != rec["post_subject"][1]
        self.rows.append(rec)
        self.pending.append(rec)

    def at_snapshot(self, k, chain_snapshot_differs):
        self.snap_frames.append((k, chain_snapshot_differs))
        if not self.pending:
            return
        cells = [r["cell"] for r in self.pending]
        qa, qb = self._q(self.roll_a, cells), self._q(self.rb, cells)
        for j, r in enumerate(self.pending):
            r["first_snapshot"] = {"k": k, "chain_snapshot_differs": bool(chain_snapshot_differs),
                                   "cpu": [int(qa[0][j]), int(qa[1][j]), bool(qa[2][j])],
                                   "subject": [int(qb[0][j]), int(qb[1][j]), bool(qb[2][j])],
                                   "cell_logodds_differs": bool(qa[1][j] != qb[1][j] or qa[2][j] != qb[2][j])}
        self.pending = []

    def finish(self):
        for r in self.rows:
            self.f.write(json.dumps(r) + "\n")
        self.f.close()
        self.fd.close()
        rows = self.rows
        fs = [r.get("first_snapshot") for r in rows]
        t = {"diag_rows": len(rows),
             "tampered_block_differs": sum(r["block_differs"] for r in rows),
             "tampered_header_differs": sum(r["header_differs"] for r in rows),
             "tampered_result_differs": sum(r["result_differs"] for r in rows),
             "pred_matches_result": sum(r["pred_delta_differs"] == r["result_differs"] for r in rows),
             "tampered_logodds_differs_now": sum(r["logodds_differs_now"] for r in rows),
             "alt_raw_block_and_(delta_or_next_chain_snapshot)": sum(
                 r["block_differs"] and (r["result_differs"] or bool(s and s["chain_snapshot_differs"]))
                 for r, s in zip(rows, fs)),
             "alt_attr_block_and_(delta_or_cell_at_next_snapshot)": sum(
                 r["block_differs"] and (r["result_differs"] or bool(s and s["cell_logodds_differs"]))
                 for r, s in zip(rows, fs)),
             "no_later_snapshot": sum(s is None for s in fs),
             "subject_delta_overflow_max": max([r.get("subject_delta_overflow", 0) for r in rows] or [0])}
        first = rows[0]["k"] if rows else None
        t["chain_snapshots_differ_before_first_tamper"] = sum(d for kk, d in self.snap_frames if first is not None and kk < first)
        t["chain_snapshots_before_first_tamper"] = sum(1 for kk, d in self.snap_frames if first is not None and kk < first)
        return t


def main(argv=None):
    from host.geo_producer import DEFAULT_HEIGHT_CELL, DEFAULT_SHAPE, calibration_id
    from host.kv_link import CHUNK_MTU, KvLinkClient, remote_pl_factory
    from host.kv_rolling import RemotePlRollingBackend, remote_rolling_factory
    from host.m7_backend import M7BackendSelector
    from host.m7_grid_policy import FollowCameraGrid
    from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
    from host.m7_rolling import RollingFusion
    from host.fpga_clock import CLOCK_ID
    from tools.car_geo_producer.common import base_camera, load_prep, odom_base
    from tools.car_geo_producer.equivalence import T0, VOXEL, reference_arm, runtime_depth
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--server", default=None)
    ap.add_argument("--fake", action="store_true")
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--snapshot-every", type=int, default=10)
    ap.add_argument("--chunk-bytes", type=int, default=CHUNK_MTU)
    ap.add_argument("--timeout-s", type=float, default=2.0)
    ap.add_argument("--tamper-every", type=int, default=0,
                    help="positive control: every N-th frame the subject integrates a block with one cell "
                         "changed (must be not_held -> inline, and its delta must then differ from the cpu arm)")
    ap.add_argument("--tamper-mode", choices=("random", "threshold"), default="random",
                    help="random (frozen G4 control): one random block cell, flag (v+1)%%3. threshold "
                         "(G4 cause-finding control, 10-26): a flag-0 cell inside the window whose "
                         "subject log-odds is UNKNOWN (-16<l<16) and never tampered before, set to 2 "
                         "(OCCUPIED: l+32>=16), so the subject's same-frame delta must gain that cell")
    ap.add_argument("--tamper-log", action="store_true",
                    help="write tamper.jsonl: per tampered frame the cell, flags, pre/post log-odds of "
                         "both arms (QUERY), delta presence, differing header words, and the cell at the "
                         "first later snapshot; plus result_diff.jsonl for every frame whose delta differs")
    a = ap.parse_args(argv)
    if a.fake == bool(a.server):
        ap.error("exactly one of --fake / --server")
    os.makedirs(a.out, exist_ok=False)
    proc = None
    if a.fake:
        proc, addr = spawn_fake(a.out)
    else:
        h, p = a.server.rsplit(":", 1)
        addr = (h, int(p))
    try:
        meta, depth = load_prep(a.prep)
        unit = float(meta["depth_unit_m"])
        K = np.asarray(meta["K"], dtype=np.float64)
        n = depth.shape[0] if not a.frames else min(depth.shape[0], a.frames)
        build = tempfile.mkdtemp(prefix="m7rfe_")
        pipe_a, fus_a = reference_arm(build, DEFAULT_SHAPE, DEFAULT_HEIGHT_CELL)
        roll_a = Timed(fus_a.backend)
        fus_a.backend = roll_a
        sel = M7BackendSelector(
            remote_pl_factory=remote_pl_factory(addr, depth_unit_m=unit, chunk_bytes=a.chunk_bytes,
                                                timeout_s=a.timeout_s),
            remote_rolling_factory=remote_rolling_factory(addr, timeout_s=a.timeout_s))
        policy = FollowCameraGrid(VOXEL, DEFAULT_SHAPE, back_cells=1, height_cell=DEFAULT_HEIGHT_CELL)
        fb = sel.frame_backend("remote_pl")
        # Same-input gate: a frame request lost in transport (UDP, 512 KB) would leave the subject
        # arm one integrate short and every later frame unequal. The frame core is stateless and
        # runs before any ROLL, so the harness retries the frame call on LinkTimeout (the car does
        # not; transport loss is reported, not compared). ROLL itself resends by cmd_id.
        frame_retries = {"retried": 0, "gave_up": 0}
        inner_ffd = fb.frame_from_depth

        def frame_from_depth_retry(*args, **kw):
            from host.kv_link import LinkTimeout
            for attempt in range(4):
                try:
                    return inner_ffd(*args, **kw)
                except LinkTimeout:
                    if attempt == 3:
                        frame_retries["gave_up"] += 1
                        raise
                    frame_retries["retried"] += 1
        fb.frame_from_depth = frame_from_depth_retry
        pipe_b = M7Pipeline(fb, GridSpec((0., 0., 0.), VOXEL, DEFAULT_SHAPE), grid_policy=policy)
        rb = sel.rolling_backend("remote_pl")
        fus_b = RollingFusion(rb, voxel_m=VOXEL)  # type check wants the real backend
        roll_b = Timed(rb)
        fus_b.backend = roll_b
        Tbc = base_camera()
        calib = calibration_id(K, Tbc, depth.shape[2], depth.shape[1])
        tally = {"frames": n, "both_blocks": 0, "block_equal": 0, "fused_cpu": 0, "fused_subject": 0,
                 "result_equal": 0, "header_equal": 0, "snapshots": 0, "snapshot_equal": 0,
                 "queries_equal": 0, "tampered": 0, "tampered_differs": 0}
        mism = open(os.path.join(a.out, "mismatches.jsonl"), "w")
        rng = np.random.default_rng(5)
        diag = bool(a.tamper_every) and (a.tamper_log or a.tamper_mode != "random")
        tlog = TamperLog(a.out, roll_a, rb, fus_b, rng, a.tamper_mode) if diag else None
        for k in range(n):
            T_ob = odom_base(k)
            T_oc = T_ob @ Tbc
            acq = T0 + int(k * 100e6)
            now = acq + 5_000_000
            res = []
            rec = None
            if diag:
                # Geometry of both arms first, then the tamper (its rng draw sits at the same place
                # in the rng stream as in the frozen loop), then both fusions in the frozen order.
                navs = []
                for tag, pipe in (("cpu", pipe_a), ("subject", pipe_b)):
                    ident = CaptureIdentity("fe-" + tag, k + 1, acq, CLOCK_ID, calib, "odom", "odom_v1")
                    navs.append(pipe.ingest_geometry(
                        DepthFrame(ident, runtime_depth(depth[k], unit), K,
                                   PoseAtCapture(acq, CLOCK_ID, "odom", "odom_v1", True, T_oc)),
                        now_ns=now, now_clock_id=CLOCK_ID).navigation)
                if navs[1].valid and (k + 1) % a.tamper_every == 0:
                    rec = tlog.tamper(k, navs[0], navs[1], tuple(T_ob[:3, 3]))
                    tally["tampered"] += 1
                for nav, pipe, fus in ((navs[0], pipe_a, fus_a), (navs[1], pipe_b, fus_b)):
                    res.append((nav, fus.ingest(nav, pipe, now_ns=now, clock_id=CLOCK_ID,
                                                robot_position_m=tuple(T_ob[:3, 3]))))
            for tag, pipe, fus in (() if diag else (("cpu", pipe_a, fus_a), ("subject", pipe_b, fus_b))):
                ident = CaptureIdentity("fe-" + tag, k + 1, acq, CLOCK_ID, calib, "odom", "odom_v1")
                out = pipe.ingest_geometry(DepthFrame(ident, runtime_depth(depth[k], unit), K,
                                                      PoseAtCapture(acq, CLOCK_ID, "odom", "odom_v1", True, T_oc)),
                                           now_ns=now, now_clock_id=CLOCK_ID)
                nav = out.navigation
                if tag == "subject" and a.tamper_every and nav.valid and (k + 1) % a.tamper_every == 0:
                    f = nav.payload["voxel_flags"]
                    i = int(rng.integers(0, f.size))
                    f.ravel()[i] = (f.ravel()[i] + 1) % 3
                    tally["tampered"] += 1
                res.append((nav, fus.ingest(nav, pipe, now_ns=now, clock_id=CLOCK_ID,
                                            robot_position_m=tuple(T_ob[:3, 3]))))
            (na, ra), (nb, rb_) = res
            bad = {}
            if na.valid and nb.valid:
                tally["both_blocks"] += 1
                if np.array_equal(na.payload["voxel_flags"], nb.payload["voxel_flags"]):
                    tally["block_equal"] += 1
                else:
                    bad["block"] = int(np.count_nonzero(na.payload["voxel_flags"] != nb.payload["voxel_flags"]))
            tally["fused_cpu"] += bool(ra.accepted)
            tally["fused_subject"] += bool(rb_.accepted)
            ka = None if not ra.accepted else (ra.delta.epoch, ra.delta.delta_seq, ra.delta.window_base,
                                               ra.delta.changes)
            kb = None if not rb_.accepted else (rb_.delta.epoch, rb_.delta.delta_seq, rb_.delta.window_base,
                                                rb_.delta.changes)
            if ka == kb and ra.reason == rb_.reason:
                tally["result_equal"] += 1
            else:
                bad["result"] = [ra.reason, rb_.reason]
            if ra.kernel is not None and rb_.kernel is not None and np.array_equal(ra.kernel.header, rb_.kernel.header):
                tally["header_equal"] += 1
            elif (ra.kernel is None) != (rb_.kernel is None) or ra.kernel is not None:
                bad["header"] = True
            if "block" in bad and "result" in bad:
                tally["tampered_differs"] += 1
            if diag:
                tlog.after_frame(k, rec, ra, rb_, bad)
            if a.snapshot_every and (k + 1) % a.snapshot_every == 0 and ra.accepted:
                tally["snapshots"] += 1
                _, la = roll_a.inner.snapshot()
                _, lb = rb.snapshot()
                if la is not None and lb is not None and np.array_equal(la, lb):
                    tally["snapshot_equal"] += 1
                else:
                    bad["snapshot"] = True
                if diag:
                    tlog.at_snapshot(k, "snapshot" in bad)
                base = ra.delta.window_base
                q = np.stack([rng.integers(base[i] - 4, base[i] + d + 4, 256) for i, d in
                              enumerate(roll_a.config.dims)], axis=1)
                qa, qb = roll_a.inner.query(q)[1], rb.query(q)[1]
                if all(np.array_equal(u, v) for u, v in zip(qa, qb)):
                    tally["queries_equal"] += 1
                else:
                    bad["query"] = True
            if bad:
                mism.write(json.dumps({"k": k, **bad}) + "\n")
        mism.close()
        if diag:
            tally.update(tlog.finish())
        cl = KvLinkClient(addr)
        try:
            kv = cl.stats().get("rolling")
        finally:
            cl.close()
        summary = {"prep": os.path.abspath(a.prep), "server": "fake" if a.fake else a.server, "tally": tally,
                   "nano_integrate_cpu": {"cpu_arm_thread_cpu": stats(roll_a.cpu_ns), "cpu_arm_wall": stats(roll_a.wall_ns),
                                          "subject_thread_cpu": stats(roll_b.cpu_ns), "subject_wall_incl_rtt": stats(roll_b.wall_ns)},
                   "subject_counts": {k: v for k, v in rb.counts.items() if k.startswith("roll")},
                   "harness_frame_retries": frame_retries,
                   "kv_rolling": kv, "measured_on": os.uname().nodename}
        ctrl = a.tamper_every > 0
        if ctrl:
            ok = tally["tampered"] > 0 and tally["tampered_differs"] == tally["tampered"] and \
                rb.counts["roll:inline_resend"] == tally["tampered"]
        else:
            fused = tally["fused_cpu"]
            ok = (tally["result_equal"] == n and tally["header_equal"] == fused and tally["block_equal"] == tally["both_blocks"]
                  and tally["snapshot_equal"] == tally["snapshots"] and tally["queries_equal"] == tally["snapshots"]
                  and fused > 0)
        summary["verdict"] = "PASS" if ok else "FAIL"
        json.dump(summary, open(os.path.join(a.out, "summary.json"), "w"), indent=1, default=str)
        print(json.dumps({"verdict": summary["verdict"], "tally": tally, "cost": summary["nano_integrate_cpu"],
                          "subject_counts": summary["subject_counts"]}, default=str))
        return 0 if ok else 1
    finally:
        if proc is not None:
            proc.terminate()
            proc.communicate(timeout=10)


if __name__ == "__main__":
    sys.exit(main())
