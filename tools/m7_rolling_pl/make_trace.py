#!/usr/bin/env python3
"""Record a rolling-map call trace from the CPU core for the PL core to replay (2026-10-26, v3 L2).

The car's CPU fusion path is driven exactly as tools/car_geo_producer/equivalence.py's reference
arm drives it (M7Pipeline(cpu frame core, FollowCameraGrid(0.05, 32x32x10, back_cells=1,
height_cell)) + RollingFusion, synthetic odom common.odom_base 0.2 m/s 10 Hz, camera on the
recommended mount), on prepared depth sets (rec30: car Gemini 2 recording, TUM pioneer). The
rolling backend handed to RollingFusion is a recorder around RollingKernelHost (g++ build of
hw/m7/rolling_map.cpp -- the very library the Nano runs). Every reset / integrate the car
would issue is recorded with its raw inputs and the core's raw outputs (header[24], the
n_delta change records in order); every ``--dump-every`` accepted frames a DUMP (25,600
words) and a QUERY (256 coordinates) are added. Each call is cross-checked against the
independent RollingReference (hw/m7/rolling_map_ref.cpp): status / epoch / delta_seq /
window base / sorted changes, and snapshots cell by cell; any disagreement aborts.

Fixed edge segments (``--synthetic``): calls before the first RESET (uninitialized), a RESET
with an out-of-range base, stale frame / stale epoch / malformed block / bad dims, window
moves on every axis incl. near-full and full clears, blocks partly outside the window, a
32x32x32 block, QUERY with 1 / 1024 / 1025 (bad args) coordinates, an unknown op, and an
epoch switch. Their expected statuses are asserted here so the segment exercises the branch.

Trace format: see hw/m7/pl/hls/tb_rolling_axi.cpp. Also writes <out>.json (per record:
segment, op, frame, status, n_delta; totals). Usage:
  make_trace.py --out FILE [--rec30 DIR] [--rec30-frames N] [--tum DIR] [--tum-frames N]
                [--dump-every 10] [--synthetic] [--mutant NAME]
``--mutant`` records from a mutant core (Python-level positive control material only).
"""
import argparse
import json
import os
import struct
import sys
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.m7_rolling import (H, OP_DUMP, OP_INTEGRATE, OP_QUERY, OP_RESET,  # noqa: E402
                             RollingKernelHost, RollingReference, _RollingBackend,
                             compile_rolling)

REC_HEAD = struct.Struct("<3I6i8I")
STATUS_OK = 0


class TraceWriter(object):
    def __init__(self):
        self.chunks, self.meta = [], []

    def add(self, seg, op, epoch, frame, wb, ob, dims, qc, obs_bytes, qxyz, header, delta, qout, dump):
        obs_bytes = b"" if obs_bytes is None else bytes(obs_bytes)
        pad = (-len(obs_bytes)) % 8
        delta = np.zeros(0, np.uint64) if delta is None else np.asarray(delta, dtype="<u8")
        qout = np.zeros(0, np.uint32) if qout is None else np.asarray(qout, dtype="<u4")
        dump = np.zeros(0, np.uint64) if dump is None else np.asarray(dump, dtype="<u8")
        head = REC_HEAD.pack(op, epoch & 0xFFFFFFFF, frame & 0xFFFFFFFF, *(int(v) for v in wb),
                             *(int(v) for v in ob), dims[0], dims[1], dims[2], qc,
                             len(obs_bytes), delta.size, qout.size, dump.size)
        parts = [head, obs_bytes, b"\0" * pad]
        if qxyz is not None:
            parts.append(np.asarray(qxyz, dtype="<i4").tobytes())
        parts += [np.asarray(header, dtype="<u4").tobytes(), delta.tobytes(), qout.tobytes(), dump.tobytes()]
        self.chunks.append(b"".join(parts))
        self.meta.append({"seg": seg, "op": int(op), "frame": int(frame), "status": int(header[0]),
                          "n_delta": int(delta.size), "n_qout": int(qout.size), "dump": bool(dump.size),
                          "shift": int(header[H["SHIFT"]])})

    def save(self, path):
        with open(path, "wb") as f:
            f.write(b"M7RT" + struct.pack("<II", 1, len(self.chunks)))
            for c in self.chunks:
                f.write(c)


class Recorder(_RollingBackend):
    """A RollingFusion backend that records every call of the wrapped CPU core."""

    def __init__(self, kernel_lib, ref_lib, writer, dump_every):
        self.k = RollingKernelHost(kernel_lib)
        self.ref = RollingReference(ref_lib) if ref_lib is not None else None
        self.config = self.k.config
        self.w, self.dump_every, self.seg = writer, int(dump_every), "init"
        self.accepted = 0
        self.rng = np.random.default_rng(20261026)

    def _check(self, got, ref):
        if self.ref is None:
            return
        a = (got.status, got.epoch, got.delta_seq, got.window_base, got.changes)
        b = (ref.status, ref.epoch, ref.delta_seq, ref.window_base, ref.changes)
        if a[:4] != b[:4] or a[4] != b[4]:
            raise SystemExit("CPU core disagrees with RollingReference in segment %s: %r vs %r"
                             % (self.seg, a[:4], b[:4]))

    # -- raw calls (any arguments; returns header) ------------------------------------------
    def raw(self, op, epoch=0, frame=0, wb=(0, 0, 0), ob=(0, 0, 0), block=None, dims=None,
            qxyz=None, qc=None):
        k = self.k
        obs_bytes, buf = None, None
        if block is not None:
            buf, nx, ny, nz = self._obs_buffer(block)
            dims = (nx, ny, nz)
            obs_bytes = buf[:nx * ny * nz].tobytes()
        dims = dims or (0, 0, 0)
        qbuf = None
        if qxyz is not None:
            q = np.asarray(qxyz, dtype=np.int32).reshape(-1, 3)
            qbuf = np.zeros(3 * k.config.query_capacity, dtype=np.int32)
            qbuf[:q.size] = q.ravel()
            qc = q.shape[0] if qc is None else qc
        qc = 0 if qc is None else int(qc)
        hdr = k._call(op, epoch=epoch, frame_seq=frame, window_base=np.asarray(wb, np.int32),
                      obs_base=np.asarray(ob, np.int32), obs=buf, dims=dims, query=qbuf,
                      query_count=qc)
        ok = int(hdr[0]) == STATUS_OK
        n = int(hdr[H["N_DELTA"]]) if (ok and op == OP_INTEGRATE) else 0
        delta = k._delta[:n].copy()
        qout = k._query_out[:qc].copy() if (ok and op == OP_QUERY) else None
        dump = k._dump.copy() if (ok and op == OP_DUMP) else None
        qin = None
        if op == OP_QUERY and qc <= k.config.query_capacity:
            qin = (qbuf if qbuf is not None else np.zeros(3 * k.config.query_capacity, np.int32))[:3 * qc]
        self.w.add(self.seg, op, epoch, frame, wb, ob, dims, qc, obs_bytes, qin, hdr, delta, qout, dump)
        return hdr, delta

    # -- the four checked operations RollingFusion uses ------------------------------------
    def reset(self, epoch, window_base):
        hdr, _ = self.raw(OP_RESET, epoch=int(epoch), wb=window_base)
        res = self.k._result(hdr)
        if self.ref is not None:
            self._check(res, self.ref.reset(epoch, window_base))
        return res

    def integrate(self, epoch, frame_seq, window_base, obs_base, obs, *, extra=True):
        hdr, delta = self.raw(OP_INTEGRATE, epoch=int(epoch), frame=int(frame_seq), wb=window_base,
                              ob=obs_base, block=np.ascontiguousarray(obs, dtype=np.uint8))
        res = self.k._result(hdr, self.k.decode_delta(hdr, delta))
        if self.ref is not None:
            self._check(res, self.ref.integrate(epoch, frame_seq, window_base, obs_base, obs))
        if res.ok:
            self.accepted += 1
            if extra and self.dump_every and self.accepted % self.dump_every == 0:
                self.dump_and_query()
        return res

    def dump_and_query(self, n_query=256):
        hdr, _ = self.raw(OP_DUMP)
        if int(hdr[0]) == STATUS_OK and self.ref is not None:
            got = self.k._result(hdr)
            physical = self.k._dump
            lanes = np.stack([(physical >> np.uint64(8 * j)) & np.uint64(0xFF) for j in range(8)],
                             axis=1).astype(np.uint8).view(np.int8)
            mx, my, mz = self.config.dims
            local = np.roll(lanes.reshape(mz, my, mx), shift=tuple(-o for o in got.offsets[::-1]),
                            axis=(0, 1, 2))
            rres, rlocal = self.ref.snapshot()
            if rlocal is None or not np.array_equal(local, rlocal):
                raise SystemExit("snapshot disagrees with RollingReference in segment %s" % self.seg)
        base = [int(np.int32(np.uint32(hdr[H["BASE"] + a]))) for a in range(3)]
        dims = self.config.dims
        q = np.stack([self.rng.integers(base[a] - 8, base[a] + dims[a] + 8, n_query) for a in range(3)],
                     axis=1).astype(np.int32)
        qh, _ = self.raw(OP_QUERY, qxyz=q)
        if self.ref is not None and int(qh[0]) == STATUS_OK:
            rres, (rs, rl, ro) = self.ref.query(q)
            got = self.k.decode_query(self.k._query_out[:n_query])
            if not all(np.array_equal(a, b) for a, b in zip(got, (rs, rl, ro))):
                raise SystemExit("query disagrees with RollingReference in segment %s" % self.seg)

    def query(self, coords):
        return self.k.query(coords)

    def snapshot(self):
        return self.k.snapshot()


def drive_dataset(rec, prep, frames, build, tag):
    """equivalence.py reference arm, frame by frame; the recorder is the rolling backend."""
    from host.geo_producer import DEFAULT_HEIGHT_CELL, DEFAULT_SHAPE, calibration_id
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    from host.m7_grid_policy import FollowCameraGrid
    from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
    from host.m7_rolling import RollingFusion
    from host.fpga_clock import CLOCK_ID
    from tools.car_geo_producer.common import base_camera, load_prep, odom_base
    from tools.car_geo_producer.equivalence import T0, VOXEL, runtime_depth
    meta, depth = load_prep(prep)
    unit = float(meta["depth_unit_m"])
    K = np.asarray(meta["K"], dtype=np.float64)
    n = depth.shape[0] if not frames else min(depth.shape[0], frames)
    lib = compile_frame_native(os.path.join(build, "frame_%s.so" % tag))
    frame_backend = M7BackendSelector(cpu_frame_library=lib).frame_backend("cpu")
    policy = FollowCameraGrid(VOXEL, DEFAULT_SHAPE, back_cells=1, height_cell=DEFAULT_HEIGHT_CELL)
    pipe = M7Pipeline(frame_backend, GridSpec((0., 0., 0.), VOXEL, DEFAULT_SHAPE), grid_policy=policy)
    fusion = RollingFusion(rec, voxel_m=VOXEL)
    Tbc = base_camera()
    calib = calibration_id(K, Tbc, depth.shape[2], depth.shape[1])
    rec.seg = tag
    fused = 0
    for k in range(n):
        T_ob = odom_base(k)
        T_oc = T_ob @ Tbc
        acq = T0 + int(k * 100e6)
        now = acq + 5_000_000
        ident = CaptureIdentity("trace-" + tag, k + 1, acq, CLOCK_ID, calib, "odom", "odom_v1")
        out = pipe.ingest_geometry(DepthFrame(ident, runtime_depth(depth[k], unit), K,
                                              PoseAtCapture(acq, CLOCK_ID, "odom", "odom_v1", True, T_oc)),
                                   now_ns=now, now_clock_id=CLOCK_ID)
        r = fusion.ingest(out.navigation, pipe, now_ns=now, clock_id=CLOCK_ID,
                          robot_position_m=tuple(T_ob[:3, 3]))
        fused += bool(r.accepted)
    return {"frames": n, "fused": fused, "counts": dict(fusion.counts), "epoch": fusion.epoch}


def synthetic(rec, prefix_only=False):
    """Edge segments; statuses asserted so each one reaches its branch."""
    rng = np.random.default_rng(7)

    def expect(hdr, status, what):
        if int(hdr[0]) != status:
            raise SystemExit("synthetic %s: status %d, expected %d" % (what, int(hdr[0]), status))

    def block(shape=(10, 32, 32), p=(0.6, 0.3, 0.1)):
        return rng.choice(np.array([0, 1, 2], np.uint8), size=shape, p=p)

    if prefix_only:
        rec.seg = "uninit"
        expect(rec.raw(OP_INTEGRATE, epoch=1, frame=1, block=block())[0], 1, "integrate before reset")
        expect(rec.raw(OP_QUERY, qxyz=[[0, 0, 0]])[0], 1, "query before reset")
        expect(rec.raw(OP_DUMP)[0], 1, "dump before reset")
        expect(rec.raw(OP_RESET, epoch=1, wb=(1 << 28, 0, 0))[0], 2, "reset out of range")
        return
    rec.seg = "synthetic"
    epoch = 900
    wb = [-40, -40, -8]
    if not rec.reset(epoch, tuple(wb)).ok:
        raise SystemExit("synthetic reset failed")
    frame = 1

    def integ(ob_off=(4, 20, 2), shape=(10, 32, 32), status=0, what="integrate", fr=None, ep=None, blk=None):
        nonlocal frame
        if fr is None:
            frame += 1
            fr = frame
        ob = [wb[a] + ob_off[a] for a in range(3)]
        b = block(shape) if blk is None else blk
        res = rec.integrate(epoch if ep is None else ep, fr, tuple(wb), tuple(ob), b, extra=False)
        if res.status != status:
            raise SystemExit("synthetic %s: status %d, expected %d" % (what, res.status, status))
        return res

    integ(what="first")
    integ(fr=frame, status=5, what="stale frame")
    integ(ep=epoch + 5, status=4, what="stale epoch")
    bad = block()
    bad[3, 7, 9] = 3
    integ(blk=bad, status=3, what="malformed")
    expect(rec.raw(OP_INTEGRATE, epoch=epoch, frame=frame + 1, wb=wb, ob=wb, dims=(0, 32, 10))[0], 2, "dims 0")
    expect(rec.raw(OP_INTEGRATE, epoch=epoch, frame=frame + 1, wb=wb, ob=wb, dims=(33, 32, 10))[0], 2, "dims 33")
    for axis in range(3):                       # small moves, both signs, every axis
        for d in (1, -1, 3, -5, 7, 8, -9, 13):
            wb[axis] += d
            integ(ob_off=tuple(int(v) for v in rng.integers([-6, -6, -3], [60, 60, 25])),
                  what="move axis %d by %d" % (axis, d))
    rec.dump_and_query()
    for d in ((79, 0, 0), (0, -79, 0), (0, 0, 31), (80, 0, 0), (0, 0, -32), (-200, 300, 5)):
        wb = [wb[a] + d[a] for a in range(3)]   # near-full and full clears
        res = integ(what="move %r" % (d,))
        if max(abs(v) for v in d) >= 80 or abs(d[2]) >= 32:
            if not res.shift_mask & 8:
                raise SystemExit("synthetic: full clear not taken for %r" % (d,))
    integ(ob_off=(-10, 70, -4), what="block partly outside")
    integ(ob_off=(60, -20, 28), what="block mostly outside")
    big = block((32, 32, 32), p=(0.2, 0.5, 0.3))
    integ(ob_off=(5, 5, 0), blk=big, what="32^3 block")
    integ(ob_off=(0, 0, 0), shape=(1, 1, 8), what="tiny block")
    integ(ob_off=(9, 3, 1), shape=(7, 13, 29), what="odd block")
    rec.dump_and_query(n_query=1024)
    q = np.zeros((1, 3), np.int32) + np.array(wb, np.int32)
    expect(rec.raw(OP_QUERY, qxyz=q)[0], 0, "query 1")
    expect(rec.raw(OP_QUERY, qc=1025)[0], 2, "query 1025")
    expect(rec.raw(7)[0], 2, "unknown op")
    epoch += 1
    wb = [10, -3, 2]
    if not rec.reset(epoch, tuple(wb)).ok:
        raise SystemExit("synthetic epoch switch reset failed")
    frame = 0
    integ(what="after epoch switch")
    integ(what="after epoch switch 2")
    rec.dump_and_query()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rec30", default=None)
    ap.add_argument("--rec30-frames", type=int, default=0)
    ap.add_argument("--tum", default=None)
    ap.add_argument("--tum-frames", type=int, default=200)
    ap.add_argument("--dump-every", type=int, default=10)
    ap.add_argument("--synthetic", action="store_true")
    ap.add_argument("--mutant", default=None)
    a = ap.parse_args(argv)
    if os.path.exists(a.out):
        raise SystemExit("output exists: %s" % a.out)
    build = tempfile.mkdtemp(prefix="m7rt_")
    klib = compile_rolling(os.path.join(build, "k.so"), mutant=a.mutant)
    rlib = compile_rolling(os.path.join(build, "r.so"), source="reference") if a.mutant is None else None
    w = TraceWriter()
    rec = Recorder(klib, rlib, w, a.dump_every)
    summary = {"mutant": a.mutant, "dump_every": a.dump_every, "datasets": {}}
    synthetic(rec, prefix_only=True)
    if a.rec30:
        summary["datasets"]["rec30"] = drive_dataset(rec, a.rec30, a.rec30_frames, build, "rec30")
        rec.dump_and_query()
    if a.synthetic:
        synthetic(rec)
    if a.tum:
        summary["datasets"]["tum"] = drive_dataset(rec, a.tum, a.tum_frames, build, "tum")
        rec.dump_and_query()
    w.save(a.out)
    ops = [m["op"] for m in w.meta]
    summary.update(records=len(w.meta), integrate=ops.count(OP_INTEGRATE), reset=ops.count(OP_RESET),
                   query=ops.count(OP_QUERY), dump=sum(m["dump"] for m in w.meta),
                   delta_words=sum(m["n_delta"] for m in w.meta),
                   status_hist={str(s): sum(1 for m in w.meta if m["status"] == s)
                                for s in sorted(set(m["status"] for m in w.meta))},
                   full_clears=sum(1 for m in w.meta if m["op"] == OP_INTEGRATE and m["shift"] & 8),
                   shifted=sum(1 for m in w.meta if m["op"] == OP_INTEGRATE and m["shift"] & 7),
                   bytes=os.path.getsize(a.out), cross_checked=rlib is not None)
    json.dump({"summary": summary, "records": w.meta}, open(a.out + ".json", "w"), indent=1)
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
