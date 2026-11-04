"""B1 Nano side (nationals, 2026-11-02): the producer's PL-grid outputs equal the bridge's, cell by cell.

runs/nationals-prelim-20261102/B1/NANO_SIDE.md. What is checked here (host only; no board, no car):

1. ROLL v1 bodies are byte for byte what the pre-B1 module (git tag nat-b1n-pre-20261102) encodes -- a v1
   peer sees no change; v2 request / answer round trip (grid extension, planes, cost table).
2. The main equivalence on the recorded car trace (10-26 formal.bin, rec30): every INTEGRATE goes through the
   Nano backend -> ROLL v2 -> RollingService -> CPU core + the PL grid core's C model (hw/b1_costmap) -> ACK
   planes -> PlGridLayer (the producer's grid layer), and the SAME ACK's deltas -> RollingLayerBridge (the
   bridge node's code). Final grid, raw grid and status row are compared every frame, for the car
   configuration (body seed r 0.30 + 0.05, swept clearing 300 s) and a variant (front seed 0.5 m, no sweep).
   The body pose is synthetic (window centre + sub-voxel jitter, as tools/b1_costmap/bench.py).
3. Positive controls: one tampered aux cell is seen by the grid comparison and by the DUMP cross-check;
   the DUMP cross-check matches on the untampered layer.
4. RollingFusion.enable_grid end to end with depth frames: grid on (skip_changes) vs the CPU fusion arm +
   bridge, identical grids; the change list is not decoded, the header still counts the changes.

Run: PYTHONPATH=. python3 tests/test_b1_pl_grid_nano.py   (about a minute; B1_TRACE_FRAMES=N to shorten)
"""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from host import kv_rolling as kr  # noqa: E402
from host import rolling_nav_layer as rnl  # noqa: E402
from host.kv_rolling import (CpuRollingCore, RemotePlRollingBackend, RollingService, decode_roll_ack,  # noqa: E402
                             decode_roll_req, encode_roll_ack, encode_roll_req)
from host.m7_rolling import NavigationDelta, RollingFusion, RollingKernelHost, compile_rolling  # noqa: E402
from host.pl_grid_layer import (GRID_F_GRID, LUT_SIZE, PlGridLayer, PlGridPlanes, band_params,  # noqa: E402
                                default_lut, dump_check)

TRACE = os.environ.get("B1_TRACE", "${FPGA_SRC}/runs/fpga-innov-20261025/fuse/traces/formal.bin")
V, CLOCK = 0.05, "b1test"
OUT = os.environ.get("B1_TEST_OUT")       # optional JSON with the per-run counts (NANO_SIDE.md)


class _Tmp:
    _t = None
    _n = 0

    @classmethod
    def path(cls, name):
        if cls._t is None:
            cls._t = tempfile.TemporaryDirectory()
        cls._n += 1
        return os.path.join(cls._t.name, "%d_%s" % (cls._n, name))


class InProcLink:
    """KvLinkClient stand-in: ROLL bodies straight into a RollingService (as test_m7_rolling_pl.FaultLink)."""

    def __init__(self, svc):
        from collections import Counter
        self.svc, self.counts = svc, Counter()
        self.feats = {"rolling": 1, "rolling_replay": 1, "rolling_grid": 1 if svc.grid_capable else 0}

    def require(self, feature, hello_timeout_s=3.0):
        return {"features": dict(self.feats)}

    def _control_resend(self, msg_type, ack_type, body, timeout_s, first_gap_s, flags=0):
        return self.svc.handle(body)

    def stats(self, timeout_s=0.5):
        return {"rolling": self.svc.describe()}

    def close(self):
        pass


def grid_service():
    return RollingService(CpuRollingCore(compile_rolling(_Tmp.path("roll.so")), grid_library=_Tmp.path("grid.so")))


def car_band():
    return rnl.HeightBand(ground_z_m=0.0, robot_height_m=0.31)


def layer_kw(band, **over):
    kw = dict(dims=(80, 80, 32), voxel_m=V, band=band, clock_id=CLOCK, frame_id="odom",
              max_map_age_ns=200_000_000, body_radius_m=0.30, body_margin_m=0.05, swept_clear_s=300.0)
    kw.update(over)
    return kw


def body_pose(base, k):
    """tools/b1_costmap/bench.py: window centre + a deterministic sub-voxel jitter (golden-ratio sequence)."""
    fx = 0.05 + 0.9 * ((k * 0.6180339887) % 1.0)
    fy = 0.05 + 0.9 * ((k * 0.7548776662 + 0.3) % 1.0)
    return ((base[0] + 40 + fx) * V, (base[1] + 40 + fy) * V, (k * 0.37) % 6.283 - 3.14)


def status_core(row):
    return {k: row.get(k) for k in ("epoch", "delta_seq", "fresh", "reason", "age_ns", "max_map_age_ns",
                                    "swept", "clock_id", "cold_start")}


def load_pre_b1_kv_rolling():
    """host/kv_rolling.py as of the tag nat-b1n-pre-20261102 (the encoder before B1), or None."""
    try:
        src = subprocess.run(["git", "-C", str(ROOT), "show", "nat-b1n-pre-20261102:host/kv_rolling.py"],
                             check=True, capture_output=True, text=True).stdout
    except (subprocess.CalledProcessError, OSError):
        return None
    path = _Tmp.path("kv_rolling_pre.py")
    with open(path, "w") as f:
        f.write(src)
    spec = importlib.util.spec_from_file_location("kv_rolling_pre_b1", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def reset_rec(remote, r):
    """A trace RESET: through the checked call when the Nano would send it, else raw (contract records)."""
    try:
        return remote.reset(r["epoch"], r["wb"]).ok
    except ValueError:
        ack = remote.raw(0, epoch=r["epoch"], window_base=r["wb"])
        return int(ack["header"][0]) == 0


def integrate_rec(remote, r):
    """A trace INTEGRATE through the Nano backend; None for a record the Nano would never send (sent raw)."""
    nx, ny, nz = r["dims"]
    try:
        obs = np.frombuffer(bytes(r["obs"]), np.uint8)[:nx * ny * nz].reshape(nz, ny, nx)
        return remote.integrate(r["epoch"], r["frame"], r["wb"], r["ob"], obs)
    except (ValueError, TypeError):
        try:      # malformed contract records of the trace: the core refuses them, the map is unchanged
            remote.raw(1, epoch=r["epoch"], frame_seq=r["frame"], window_base=r["wb"], obs_base=r["ob"],
                       block_bytes=None if r["obs"] is None else bytes(r["obs"]), dims=r["dims"])
        except kr.RollingDeviceError:
            pass
        return None


class CodecTests(unittest.TestCase):
    def test_v1_bytes_unchanged_and_v2_roundtrip(self):
        old = load_pre_b1_kv_rolling()
        if old is None:
            self.skipTest("tag nat-b1n-pre-20261102 not available")
        reqs = [dict(op=1, cmd_id=7, mode=1, epoch=3, frame_seq=9, window_base=(1, -2, -8), obs_base=(4, 5, -1),
                     dims=(32, 32, 10), obs_crc=0x1234, inline=b"\x01\x02\x00"),
                dict(op=2, cmd_id=8, query=np.array([[1, 2, 3], [4, 5, 6]], np.int32)),
                dict(op=15, cmd_id=9), dict(op=3, cmd_id=10)]
        for r in reqs:
            self.assertEqual(encode_roll_req(**r), old.encode_roll_req(**r))
        hdr = np.arange(24, dtype=np.uint32)
        acks = [dict(result=0, cmd_id=5, header=hdr, delta=np.array([1, 2], np.uint64), times=(1, 2, 3)),
                dict(result=1, cmd_id=6), dict(result=0, cmd_id=7, header=hdr, dump=np.arange(5, dtype=np.uint64))]
        for a in acks:
            self.assertEqual(encode_roll_ack(**a), old.encode_roll_ack(**a))
        # v2 request: the grid extension survives, the old decoder refuses it (a v1 KV never gets one)
        body = encode_roll_req(op=1, cmd_id=11, dims=(2, 2, 2), inline=b"\x00" * 8, obs_crc=kr.crc32(b"\x00" * 8),
                               mode=1, grid=(GRID_F_GRID, 9, 14, 12, -5, 7, 3 << 16))
        req = decode_roll_req(body)
        self.assertEqual((req["ver"], req["grid"]["band_lo"], req["grid"]["band_hi"], req["grid"]["tall_lo"],
                          req["grid"]["body_cx"], req["grid"]["body_r"]), (2, 9, 14, 12, -5, 3 << 16))
        self.assertEqual(req["inline"], b"\x00" * 8)
        with self.assertRaises(ValueError):
            old.decode_roll_req(body)
        g = np.array([100, 0, -1, 0], np.int8)
        aux = np.array([2, 1, 0 | 4, 1], np.uint8)
        ack = decode_roll_ack(encode_roll_ack(result=0, cmd_id=11, header=hdr, delta=np.array([3], np.uint64),
                                              ver=2, grid_status=0, grid=g, aux=aux, lut_crc=77))
        self.assertEqual((ack["ver"], ack["grid"]["status"], ack["grid"]["lut_crc"]), (2, 0, 77))
        self.assertTrue(np.array_equal(ack["grid"]["grid"], g) and np.array_equal(ack["grid"]["aux"], aux))
        self.assertIsNone(ack["grid"]["cost"])
        self.assertEqual(int(ack["delta"][0]), 3)

    def test_config_lut_and_feature(self):
        svc = grid_service()
        self.assertTrue(svc.grid_capable)
        self.assertFalse(RollingService(CpuRollingCore(compile_rolling(_Tmp.path("r.so")))).grid_capable)
        remote = RemotePlRollingBackend(None, link=InProcLink(svc))
        lut = default_lut()
        self.assertEqual(lut.size, LUT_SIZE)
        self.assertEqual(remote.grid_lut_crc, kr.crc32(lut.tobytes()))
        self.assertEqual(svc.counts["lut_set"], 1)
        self.assertTrue(remote.grid_capable)
        # a server without the grid op: v1 CONFIG, grid off
        plain = RemotePlRollingBackend(None, link=InProcLink(
            RollingService(CpuRollingCore(compile_rolling(_Tmp.path("r2.so"))))))
        self.assertFalse(plain.grid_capable)


class TraceEquivalenceTests(unittest.TestCase):
    """The producer's grid layer on the KV planes == the bridge on the same ACK's deltas, every frame."""

    def test_formal_trace_cell_by_cell(self):
        if not os.path.exists(TRACE):
            self.skipTest("trace %s not on this host" % TRACE)
        from tools.m7_rolling_pl.replay_trace import read_trace
        limit = int(os.environ.get("B1_TRACE_FRAMES", "0")) or None
        svc = grid_service()
        remote = RemotePlRollingBackend(None, link=InProcLink(svc))
        band = car_band()
        remote.grid_band, remote.grid_voxel_m, remote.skip_changes = band, V, False   # = enable_grid, shadow
        variants = {"car": {}, "front_noswept": dict(front_seed_m=0.5, swept_clear_s=0.0), "car_tall_zeroed": {}}
        pairs = None
        n = {"frames": 0, "band_outside": 0, "delta_mismatch": 0, "grid_cells": 0, "raw_cells": 0,
             "status_rows_diff": 0, "planes_missing": 0, "swept_cleared_total": 0, "occupied_cells": 0,
             "tamper_tried": 0, "tamper_seen": 0}
        t0 = time.time()
        for r in read_trace(TRACE):
            if r["op"] == 0:
                ok = reset_rec(remote, r)
                self.assertEqual(ok, int(r["header"][0]) == 0)
                if ok:
                    pairs = {k: (rnl.RollingLayerBridge(**layer_kw(band, **o)), PlGridLayer(**layer_kw(band, **o)))
                             for k, o in variants.items()}
                continue
            if r["op"] != 1:
                continue
            res = integrate_rec(remote, r)
            if res is None:
                n["raw_records"] = n.get("raw_records", 0) + 1
                continue
            self.assertEqual(res.status, int(r["header"][0]))
            if res.ok and res.changes != tuple(sorted(RollingKernelHost.decode_delta(remote, r["header"], r["delta"]))):
                n["delta_mismatch"] += 1                     # the replay left the recorded run (sync check)
            if not res.ok:
                continue
            n["frames"] += 1
            now = n["frames"] * 100_000_000
            dl = NavigationDelta(int(r["epoch"]), res.delta_seq, int(r["frame"]), now, CLOCK, now + 10 ** 15,
                                 res.window_base, (80, 80, 32), V, res.changes)
            planes = res.grid
            if planes is None:
                n["planes_missing"] += 1
                with self.assertRaises(ValueError):          # only the band-outside-window frames lack planes
                    band_params(band, res.window_base[2], 32, V)
                n["band_outside"] += 1
            pose = body_pose(res.window_base, n["frames"])
            for name, (br, pl) in pairs.items():
                ok, why, req = br.on_delta(dl, now_ns=now)
                self.assertTrue(ok, why)
                self.assertIsNone(req)
                if planes is None:
                    continue
                pin = planes
                if name == "car_tall_zeroed":     # positive control: the swept evidence needs the tall bit
                    pin = PlGridPlanes(0, planes.grid, planes.aux & 3)
                ok, why = pl.on_planes(epoch=dl.epoch, delta_seq=dl.delta_seq, window_base=dl.window_base,
                                       acquired_ns=now, clock_id=CLOCK, planes=pin)
                self.assertTrue(ok, why)
                g_sw = br.grid(body_xy=pose[:2], now_ns=now + 5_000_000, body_yaw=pose[2])
                g_pl = pl.grid(body_xy=pose[:2], now_ns=now + 5_000_000, body_yaw=pose[2])
                if name == "car_tall_zeroed":
                    n["tall_zeroed_cells"] = n.get("tall_zeroed_cells", 0) + int((g_sw.data != g_pl.data).sum())
                    n["swept_evidence_cells_sw"] = int(br.counts["swept_evidence_cells"])
                    continue
                n["grid_cells"] += int((g_sw.data != g_pl.data).sum())
                n["raw_cells"] += int((br.strict_grid().data != pl.strict_grid().data).sum())
                self.assertEqual((g_sw.origin_xy, g_sw.epoch, g_sw.delta_seq, g_sw.acquired_ns),
                                 (g_pl.origin_xy, g_pl.epoch, g_pl.delta_seq, g_pl.acquired_ns))
                for t in (now + 5_000_000, now + 250_000_000):      # fresh, then stale
                    if status_core(br.status(now_ns=t, clock_id=CLOCK)) != status_core(pl.status(now_ns=t, clock_id=CLOCK)):
                        n["status_rows_diff"] += 1
                if name == "car":
                    n["swept_cleared_total"] = int(br.counts["swept_cleared_cells"])
                    n["occupied_cells"] += int((g_sw.data == 100).sum())
                    # positive control: one tampered aux cell (an unknown/free cell made occupied) must show
                    if n["frames"] % 25 == 0:
                        cand = np.nonzero((planes.aux & 3) != 2)[0]
                        if cand.size:
                            n["tamper_tried"] += 1
                            aux = planes.aux.copy()
                            aux[int(cand[cand.size // 2])] = (aux[int(cand[cand.size // 2])] & 4) | 2
                            t = PlGridLayer(**layer_kw(band, swept_clear_s=0.0))
                            t.on_planes(epoch=dl.epoch, delta_seq=dl.delta_seq, window_base=dl.window_base,
                                        acquired_ns=now, clock_id=CLOCK, planes=PlGridPlanes(0, planes.grid, aux))
                            n["tamper_seen"] += int((t.strict_grid().data != br.strict_grid().data).sum() > 0)
            if limit and n["frames"] >= limit:
                break
        n["seconds"] = round(time.time() - t0, 1)
        n["kv_counts"] = {k: v for k, v in svc.counts.items() if "grid" in k or "lut" in k}
        n["nano_counts"] = {k: v for k, v in remote.counts.items() if "grid" in k}
        if OUT:
            import json
            with open(OUT, "w") as f:
                json.dump(n, f, indent=1, default=int)
        print("B1 trace equivalence:", n)
        self.assertGreater(n["frames"], 100 if not limit else 0)
        self.assertEqual((n["grid_cells"], n["raw_cells"], n["status_rows_diff"], n["delta_mismatch"]), (0, 0, 0, 0))
        self.assertEqual(n["planes_missing"], n["band_outside"])
        self.assertGreater(n["occupied_cells"], 0)                    # the comparison saw obstacles
        self.assertEqual(n["tamper_seen"], n["tamper_tried"])
        self.assertGreater(n["tall_zeroed_cells"], 0)      # the tall bit matters on this trace: a lost bit shows
        self.assertGreater(n["tamper_tried"], 0)


class DumpCheckTests(unittest.TestCase):
    def test_dump_check_matches_and_catches_tamper(self):
        if not os.path.exists(TRACE):
            self.skipTest("trace %s not on this host" % TRACE)
        from tools.m7_rolling_pl.replay_trace import read_trace
        svc = grid_service()
        remote = RemotePlRollingBackend(None, link=InProcLink(svc))
        band = car_band()
        remote.grid_band, remote.grid_voxel_m, remote.skip_changes = band, V, True
        layer = PlGridLayer(**layer_kw(band))
        results = {"match": 0, "mismatch": 0, "not_comparable": 0, "tamper_caught": 0, "tamper_tried": 0}
        frames = 0
        for r in read_trace(TRACE):
            if r["op"] == 0:
                reset_rec(remote, r)
                continue
            if r["op"] != 1:
                continue
            res = integrate_rec(remote, r)
            if res is None or not res.ok:
                continue
            frames += 1
            self.assertEqual(res.changes, ())                        # skip_changes: no per-lane decode
            if res.grid is None or frames % 20:
                continue
            layer.on_planes(epoch=res.epoch, delta_seq=res.delta_seq, window_base=res.window_base,
                            acquired_ns=frames, clock_id=CLOCK, planes=res.grid)
            result, local = remote.snapshot()
            from host.m7_rolling import MapSnapshot
            snap = MapSnapshot(True, "map_current", result.epoch, result.delta_seq, result.window_base, local,
                               remote.config.state_of(local))
            results[dump_check(snap, layer, band)["result"]] += 1
            aux = res.grid.aux.copy()
            i = int(np.nonzero((aux & 3) == 1)[0][0]) if np.any((aux & 3) == 1) else 0
            aux[i] = (aux[i] & 4) | 2
            bad = PlGridLayer(**layer_kw(band))
            bad.on_planes(epoch=res.epoch, delta_seq=res.delta_seq, window_base=res.window_base, acquired_ns=frames,
                          clock_id=CLOCK, planes=PlGridPlanes(0, res.grid.grid, aux))
            results["tamper_tried"] += 1
            results["tamper_caught"] += int(dump_check(snap, bad, band)["result"] == "mismatch")
            if frames >= 200:
                break
        print("B1 dump check:", results)
        self.assertGreater(results["match"], 3)
        self.assertEqual(results["mismatch"], 0)
        self.assertEqual(results["tamper_caught"], results["tamper_tried"])


class FusionGridTests(unittest.TestCase):
    """RollingFusion.enable_grid with depth frames: PL grid layer == CPU fusion arm + bridge."""

    def test_enable_grid_end_to_end(self):
        import test_m7_rolling_pl as t
        from host.m7_frame_native import FrameNativeM7
        from host.m7_pipeline import M7Pipeline
        svc = grid_service()
        remote = RemotePlRollingBackend(None, link=InProcLink(svc))
        fus_pl = RollingFusion(remote, voxel_m=0.05)
        band = car_band()
        self.assertTrue(fus_pl.enable_grid(band, skip_changes=True))
        fus_cpu = RollingFusion(RollingKernelHost(compile_rolling(_Tmp.path("cpu.so"))), voxel_m=0.05)
        self.assertFalse(fus_cpu.enable_grid(band))                   # the Nano CPU core has no grid op
        kw = layer_kw(band)
        kw["clock_id"] = t.CLOCK
        bridge, layer = rnl.RollingLayerBridge(**kw), PlGridLayer(**kw)
        lib = t.Env.frame_lib()
        pipes = {}
        cells = 0
        for fid in range(1, 13):
            ns = time.monotonic_ns()
            pos = (0.07 * fid, -0.03 * fid, 0.0)
            out = {}
            for name, fus in (("pl", fus_pl), ("cpu", fus_cpu)):
                pipe = pipes.setdefault(name, M7Pipeline(FrameNativeM7(lib), t.GRID))
                nav = pipe.ingest_geometry(t.frame(fid, ns), now_ns=ns + 1000, now_clock_id=t.CLOCK).navigation
                self.assertTrue(nav.valid, nav.reason)
                out[name] = fus.ingest(nav, pipe, now_ns=ns + 2000, clock_id=t.CLOCK, robot_position_m=pos)
                self.assertTrue(out[name].accepted, out[name].reason)
            a, b = out["pl"], out["cpu"]
            self.assertIsNotNone(a.grid)
            self.assertIsNone(b.grid)
            self.assertEqual(a.delta.changes, ())
            self.assertEqual(a.kernel.changed_cells, len(b.delta.changes))
            self.assertTrue(bridge.on_delta(b.delta, now_ns=ns + 3000)[0])
            d = a.delta
            self.assertTrue(layer.on_planes(epoch=d.epoch, delta_seq=d.delta_seq, window_base=d.window_base,
                                            acquired_ns=d.acquired_ns, clock_id=d.clock_id, planes=a.grid)[0])
            xy = (pos[0] + 0.01, pos[1] - 0.02)
            cells += int((bridge.grid(body_xy=xy, now_ns=ns + 4000).data != layer.grid(body_xy=xy, now_ns=ns + 4000).data).sum())
            self.assertEqual(status_core(bridge.status(now_ns=ns + 5000, clock_id=t.CLOCK)),
                             status_core(layer.status(now_ns=ns + 5000, clock_id=t.CLOCK)))
        self.assertEqual(cells, 0)
        self.assertGreater(svc.counts["grid_status:0"], 0)


if __name__ == "__main__":
    unittest.main()
