#!/usr/bin/env python3
"""Host gate: car geometry producer (KV path) vs the same-input pure-CPU producer.

Input: a prepared depth set (rec30 / TUM) driven along tools/car_geo_producer/common.py's
synthetic odom trajectory (camera on base_link, odom <- camera = odom <- base x base <- camera).

  subject arm   host/geo_producer.GeoProducer, frame backend RemotePlFrameBackend through
                kv-link to a KV end (``--fake cpu``: separate-process KvLinkServer with the
                g++ frame core and the C ROI core; ``--server H:P``: a real kv_server, e.g.
                the KV260), the frame's camera codes sent as they are, ROI boxes of a fixed
                /fpga/memory attached to the same frame;
  reference arm written here from rolling_nav2_smoke.run_scene (CPU path): M7Pipeline(cpu,
                GridSpec, grid_policy=FollowCameraGrid(0.05, shape, back_cells=1,
                height_cell)) + RollingFusion(cpu), depth by ros_runtime's 16UC1 rule.

Both arms get the same acquisition stamps and "now" (injected clock: frame k acquired at
T0 + k*period, processed 5 ms later), so the deltas must be identical. Every delta goes
through the wire format (encode_delta/decode_delta) into its own RollingLayerBridge (clock
id fpga_clock.CLOCK_ID, the bridge's), and the OccupancyGrid content is compared too.
Per frame: (1) nav/fusion results, (2) delta epoch/seq/window_base/changes, (3) bridge
accept + strict grid bytes; per ROI row: counts and verdict code vs the local C reference
(NativeObjectEvidence.evaluate_boxes on the same float32 depth).

Positive controls (``--variant``), each must be caught (exit 0 only when caught):
  pose_lag1    subject uses the previous frame's odom <- camera
  camera_frame subject grid in the camera frame (T = I, local frame camera_local), the
               pre-producer kv_geometry convention
  clock_mono   subject stamps with clock id "mono" (the bridge's old one)
  origin_shift1 / origin_shift1_z
               the KV end computes the block one voxel off (x / z, z being the 10-cell axis)
               while the host keeps labelling it with the nominal origin (the backend call's
               grid_origin_m is shifted, the pipeline's grid is not): a PL that misplaces
               the block. The origin key cannot see this; the per-frame cell-by-cell flag
               comparison must catch it (in >= 90% of the frames)
Per frame, also (0): the backend's own block, cell by cell -- the voxel flags (UNKNOWN/FREE/HIT
of every one of the nx*ny*nz cells), the block origin and the sample counts (valid / invalid /
outside) of the subject (KV) vs the reference (CPU core); <out>/cells.jsonl one row per frame.
Writes <out>/summary.json and mismatches.jsonl.
Usage: equivalence.py --prep DIR --out NEWDIR (--fake cpu | --server H:P) [--frames N]
                      [--stride S] [--variant none|pose_lag1|camera_frame|clock_mono|origin_shift1|origin_shift1_z]
"""
import argparse
import hashlib
import json
import os
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.geo_producer import (DEFAULT_HEIGHT_CELL, DEFAULT_SHAPE, GeoProducer, KvConnection,  # noqa: E402
                               calibration_id, check_block_height)
from host.fpga_clock import CLOCK_ID  # noqa: E402
from tools.car_geo_producer.common import (base_camera, load_prep, memory_message,  # noqa: E402
                                           odom_base)

VOXEL = 0.05
T0 = 10 ** 15


def runtime_depth(codes, scale):
    """host/ros_runtime.py prepare_images, 16UC1 branch, verbatim."""
    depth_array = np.asarray(codes).astype(np.float32)
    depth_array *= scale
    depth_array[depth_array <= 0] = np.nan
    return depth_array


class Clock(object):
    def __init__(self):
        self.t = T0

    def __call__(self):
        return self.t


def compile_ref_frame(src_cpp, output):
    """compile_frame_native's flags on another frame_depth.cpp (its .hpp beside it), e.g. the
    clip (A1) reference of the bitstream under test, 7784f75 hw/m7/frame_depth.cpp."""
    import shutil
    import subprocess
    subprocess.run([shutil.which("g++"), "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-ffp-contract=off", "-fPIC", "-shared", src_cpp, "-o", output],
                   check=True, capture_output=True, text=True)
    return output


def reference_arm(build, shape, height_cell, ref_frame_src=None):
    """rolling_nav2_smoke.run_scene's CPU producer, line for line (backend, grid, policy)."""
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    from host.m7_grid_policy import FollowCameraGrid
    from host.m7_pipeline import GridSpec, M7Pipeline
    from host.m7_rolling import RollingFusion, compile_rolling
    lib = (compile_ref_frame(ref_frame_src, os.path.join(build, "frame_ref.so")) if ref_frame_src
           else compile_frame_native(os.path.join(build, "frame.so")))
    selector = M7BackendSelector(cpu_frame_library=lib)
    frame_backend = selector.frame_backend("cpu")
    rolling_backend = selector.rolling_backend("cpu", cpu_library=compile_rolling(os.path.join(build, "r.so")))
    policy = FollowCameraGrid(VOXEL, shape, back_cells=1, height_cell=height_cell)
    pipeline = M7Pipeline(frame_backend, GridSpec((0., 0., 0.), VOXEL, shape), grid_policy=policy)
    fusion = RollingFusion(rolling_backend, voxel_m=VOXEL)
    return pipeline, fusion


def make_bridge():
    from host.rolling_nav_layer import HeightBand, RollingLayerBridge
    return RollingLayerBridge(dims=(80, 80, 32), voxel_m=VOXEL,
                              band=HeightBand(ground_z_m=0.0, robot_height_m=0.25),
                              clock_id=CLOCK_ID, frame_id="odom")


def through_bridge(bridge, delta, now):
    from host.rolling_nav_layer import decode_delta, encode_delta
    if delta is None:
        return None, None, None
    changed, reason, _ = bridge.on_delta(decode_delta(encode_delta(delta)), now_ns=now)
    g = bridge.strict_grid() if changed else None
    sha = None if g is None else hashlib.sha256(np.asarray(g.data, dtype=np.int8).tobytes()).hexdigest()[:16]
    return changed, reason, sha


def delta_key(d):
    if d is None:
        return None
    return [d.epoch, d.delta_seq, list(d.window_base), d.acquired_ns, d.clock_id,
            hashlib.sha256(json.dumps([list(c) for c in d.changes]).encode()).hexdigest()[:16], len(d.changes)]


def main(argv=None):
    if argv is None and len(sys.argv) == 3 and sys.argv[1] == "--serve-fake":
        from tools.car_geometry_kv.equivalence import serve_fake
        return serve_fake(sys.argv[2])
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--server", default=None)
    ap.add_argument("--fake", choices=("cpu",), default=None)
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--boxes", type=int, default=24)
    ap.add_argument("--variant", choices=("none", "pose_lag1", "camera_frame", "clock_mono", "origin_shift1",
                                            "origin_shift1_z"), default="none")
    ap.add_argument("--shape", default="%d,%d,%d" % DEFAULT_SHAPE)
    ap.add_argument("--height-cell", type=int, default=DEFAULT_HEIGHT_CELL)
    ap.add_argument("--expect-build-id", default=None)
    ap.add_argument("--timeout-s", type=float, default=1.0)
    ap.add_argument("--pace-ms", type=float, default=None)
    ap.add_argument("--period-ms", type=float, default=100.0)
    ap.add_argument("--ref-frame-src", default=None,
                    help="reference arm frame core from this frame_depth.cpp (default: this tree's)")
    ap.add_argument("--mount-pitch-deg", type=float, default=None,
                    help="camera pitch down on base_link (default common.MOUNT_PITCH_DEG = 27)")
    a = ap.parse_args(argv)
    if (a.server is None) == (a.fake is None):
        ap.error("exactly one of --server / --fake")
    shape = tuple(int(v) for v in a.shape.split(","))
    os.makedirs(a.out, exist_ok=False)
    meta, depth = load_prep(a.prep)
    unit = float(meta["depth_unit_m"])
    K = np.asarray(meta["K"], dtype=np.float64)
    n_src, h, w = depth.shape
    idx = list(range(0, n_src if not a.frames else min(n_src, a.frames), max(1, a.stride)))
    build = tempfile.mkdtemp(prefix="geoprod_eq_")
    Tbc = base_camera() if a.mount_pitch_deg is None else base_camera(pitch_deg=a.mount_pitch_deg)
    ok, block = check_block_height(Tbc[2, 3], 0.0, shape=shape, height_cell=a.height_cell)
    if not ok:
        raise SystemExit("block does not hold the ground: %s" % block)
    # KV end
    from tools.car_geometry_kv.equivalence import FakeServerProcess
    server = None
    if a.fake:
        server = FakeServerProcess(a.fake)
        addr = ("127.0.0.1", server.port)
    else:
        host, port = a.server.rsplit(":", 1)
        addr = (host, int(port))
    pace = a.pace_ms if a.pace_ms is not None else (2.0 if a.fake else 0.0)
    factory = None
    if pace > 0:
        from host.kv_link import KvLinkClient, TYPE_NAMES, split

        class PacedClient(KvLinkClient):
            """Host fixture only (sandbox SO_RCVBUF cap, see tools/car_geometry_kv/equivalence.py)."""
            def _send(self, msg_type, frame_id, payload, flags=0):
                n = 0
                for dgram in split(msg_type, self.session, frame_id, payload, self.chunk_bytes, flags):
                    if n:
                        time.sleep(pace / 1e3)
                    self.sock.sendto(dgram, self.server)
                    n += 1
                self.counts["tx_chunks"] += n
                self.counts["tx_chunks:%s" % TYPE_NAMES[msg_type]] += n
                self.counts["tx_messages:%s" % TYPE_NAMES[msg_type]] += 1
                return n
        factory = lambda srv: PacedClient(srv, timeout_s=a.timeout_s)  # noqa: E731
    from host.kv_geometry import parse_build_id
    conn = KvConnection(addr, depth_unit_m=unit, timeout_s=a.timeout_s, hello_timeout_s=2.0,
                        expect_build_id=parse_build_id(a.expect_build_id), client_factory=factory)
    # same as the node (ros_geo_producer --kv-connect-wait-s): a HELLO / TELEM board poll can
    # time out right after another session closed (board 10-11: 3 of 4 first attempts)
    for attempt in range(10):
        if conn.connect():
            break
        time.sleep(1.0)
    else:
        raise SystemExit("KV not usable: %s (10 attempts)" % conn.reason)
    conn.backend.roi_enabled = conn.roi_available
    from host.m7_backend import M7BackendSelector
    from host.m7_rolling import compile_rolling
    roll = M7BackendSelector().rolling_backend("cpu", cpu_library=compile_rolling(os.path.join(build, "subj_r.so")))
    clock = Clock()
    clock_id = "mono" if a.variant == "clock_mono" else CLOCK_ID
    kw = {}
    if a.variant == "camera_frame":
        kw = {"local_frame": "camera_local", "pose_epoch": "camera_local_v1"}
    subj = GeoProducer(conn.backend, roll, depth_unit_m=unit, shape=shape, height_cell=a.height_cell,
                       clock=clock, clock_id=clock_id, stream="eq-subject", **kw)
    if a.variant.startswith("origin_shift1"):
        axis = 2 if a.variant.endswith("_z") else 0
        inner_call = conn.backend.frame_from_depth

        def shifted_call(**kw):
            o = list(kw["grid_origin_m"])
            o[axis] += kw["voxel_size_m"]
            kw["grid_origin_m"] = tuple(o)
            return inner_call(**kw)
        conn.backend.frame_from_depth = shifted_call
    ref_pipe, ref_fusion = reference_arm(build, shape, a.height_cell, a.ref_frame_src)
    from host.m7_pipeline import CaptureIdentity, DepthFrame, PoseAtCapture
    from host.m7_object_native import NativeObjectEvidence, compile_object_native
    from perception.fpga_replay.core import Config
    from tools.kv_link.nano_replay import make_roi_memory
    cfg = Config()
    refroi = NativeObjectEvidence(compile_object_native(os.path.join(build, "object_ref.so")))
    T_map_cam0 = odom_base(0) @ Tbc
    snap = make_roi_memory(runtime_depth(depth[idx[0]], unit), K, a.boxes)
    mem = memory_message(snap, T_map_cam0)
    calib = calibration_id(K, Tbc, w, h)
    b_subj, b_ref = make_bridge(), make_bridge()
    mism = open(os.path.join(a.out, "mismatches.jsonl"), "w")
    cells = open(os.path.join(a.out, "cells.jsonl"), "w")
    ctally = {"frames_both_blocks": 0, "presence_equal": 0, "flags_equal_frames": 0, "cell_diff_frames": 0,
              "cells_compared": 0, "cells_differ": 0, "origin_equal": 0, "sample_counts_equal": 0,
              "ref_hit_cells": 0, "ref_free_cells": 0, "ref_unknown_cells": 0}
    tally = {"frames": 0, "result_equal": 0, "delta_equal": 0, "grid_equal": 0, "ref_fused": 0, "subj_fused": 0,
             "bridge_subj": {}, "bridge_ref": {}, "roi_ok": 0, "roi_rows": 0, "roi_rows_equal": 0,
             "roi_failures": {}, "words": {}, "roi_core": set()}
    rtts = []
    prev_T = None
    t_wall = time.monotonic()
    for k, i in enumerate(idx):
        codes = np.asarray(depth[i])
        T_ob = odom_base(k)
        T_oc = T_ob @ Tbc
        acq = T0 + int(k * a.period_ms * 1e6)
        now = acq + 5_000_000
        clock.t = now
        # reference arm
        d_ref = runtime_depth(codes, unit)
        ident = CaptureIdentity("eq-ref", k + 1, acq, CLOCK_ID, calib, "odom", "odom_v1")
        out = ref_pipe.ingest_geometry(DepthFrame(ident, d_ref, K, PoseAtCapture(acq, CLOCK_ID, "odom", "odom_v1",
                                                                                True, T_oc)),
                                       now_ns=now, now_clock_id=CLOCK_ID)
        rf = ref_fusion.ingest(out.navigation, ref_pipe, now_ns=now, clock_id=CLOCK_ID,
                               robot_position_m=tuple(T_ob[:3, 3]))
        d_r = rf.delta if rf.accepted else None
        # subject arm
        T_subj = T_oc
        if a.variant == "pose_lag1":
            T_subj = prev_T if prev_T is not None else T_oc
        elif a.variant == "camera_frame":
            T_subj = np.eye(4)
        prev_T = T_oc
        res = subj.process(codes, K, depth_stamp_ns=1_700_000_000_000_000_000 + k, acquired_ns=acq,
                           T_odom_camera=T_subj, robot_position_m=tuple(T_ob[:3, 3]), calibration_id=calib,
                           T_world_camera=T_oc, memory_msg=mem)
        d_s = res["delta"]
        tally["frames"] += 1
        bad = {}
        # (0) the per-frame block, cell by cell
        ref_flags = out.navigation.payload.get("voxel_flags") if out.navigation.valid else None
        ref_grid = out.navigation.payload.get("grid") if out.navigation.valid else None
        sub_flags, sub_grid = res["voxel_flags"], res["grid"]
        crow = {"k": k, "frame": i, "ref": ref_flags is not None, "subj": sub_flags is not None}
        if (ref_flags is None) == (sub_flags is None):
            ctally["presence_equal"] += 1
        else:
            bad["block_presence"] = [crow["ref"], crow["subj"]]
        if ref_flags is not None and sub_flags is not None:
            ctally["frames_both_blocks"] += 1
            rf_, sf_ = np.asarray(ref_flags), np.asarray(sub_flags)
            if rf_.shape != sf_.shape:
                ndiff = int(rf_.size)
                bad["block_shape"] = [list(rf_.shape), list(sf_.shape)]
            else:
                ndiff = int(np.count_nonzero(rf_ != sf_))
            ctally["cells_compared"] += int(rf_.size)
            ctally["cells_differ"] += ndiff
            ctally["ref_hit_cells"] += int(np.count_nonzero(rf_ == 2))
            ctally["ref_free_cells"] += int(np.count_nonzero(rf_ == 1))
            ctally["ref_unknown_cells"] += int(np.count_nonzero(rf_ == 0))
            if ndiff == 0:
                ctally["flags_equal_frames"] += 1
            else:
                ctally["cell_diff_frames"] += 1
                bad["cells_differ"] = ndiff
            ro, so = [float(v) for v in ref_grid.origin_m], [float(v) for v in sub_grid.origin_m]
            if ro == so:
                ctally["origin_equal"] += 1
            else:
                bad["origin"] = [ro, so]
            rc = [out.navigation.payload.get(x) for x in ("valid_grid_samples", "invalid_depth_or_projection",
                                                         "outside_grid")]
            sc = [res["nav_counts"].get(x) for x in ("valid_grid_samples", "invalid_depth_or_projection",
                                                     "outside_grid")]
            if rc == sc:
                ctally["sample_counts_equal"] += 1
            else:
                bad["sample_counts"] = [rc, sc]
            crow.update({"shape_zyx": list(rf_.shape), "cells_differ": ndiff, "origin_ref": ro, "origin_subj": so,
                         "ref_hit": int(np.count_nonzero(rf_ == 2)), "ref_free": int(np.count_nonzero(rf_ == 1)),
                         "subj_hit": int(np.count_nonzero(sf_ == 2)), "subj_free": int(np.count_nonzero(sf_ == 1)),
                         "counts_ref": rc, "counts_subj": sc})
        cells.write(json.dumps(crow) + "\n")
        r_ref = (out.navigation.reason, rf.reason)
        r_sub = (res["nav_reason"], res["fusion_reason"])
        tally["ref_fused"] += d_r is not None
        tally["subj_fused"] += d_s is not None
        if r_ref == r_sub:
            tally["result_equal"] += 1
        else:
            bad["result"] = [r_ref, r_sub]
        ks, kr = delta_key(d_s), delta_key(d_r)
        # the subject's acquired stamps equal the reference's; clock id differs only in clock_mono
        if ks == kr:
            tally["delta_equal"] += 1
        else:
            bad["delta"] = [kr, ks]
        cs, rs_, gs = through_bridge(b_subj, d_s, now)
        cr, rr_, gr = through_bridge(b_ref, d_r, now)
        if d_s is not None:
            tally["subj_changes"] = tally.get("subj_changes", 0) + len(d_s.changes)
        tally["bridge_subj"][str(rs_)] = tally["bridge_subj"].get(str(rs_), 0) + 1
        tally["bridge_ref"][str(rr_)] = tally["bridge_ref"].get(str(rr_), 0) + 1
        if gs is not None and gs == gr:
            tally["grid_equal"] += 1
        elif gs != gr:
            bad["grid"] = [gr, gs]
        body = res["body"]
        if body["failure"] is None:
            tally["roi_ok"] += 1
            tally["roi_core"].add(body.get("roi_core"))
            checked = [v for v in body["verdicts"] if v["code"] is not None]
            for v in body["verdicts"]:
                tally["words"][v["geometry_evidence"]] = tally["words"].get(v["geometry_evidence"], 0) + 1
            if checked:
                # near/far are not in the body: re-plan on the reference pose, then the C reference
                from host.geometry_region_glue import plan_frame
                plan = plan_frame(mem, d_ref.shape, K, T_oc, cfg)
                rows = [(r.row_id, r.xyxy, r.near_z, r.far_z) for r in plan.batch.rows]
                want = refroi.evaluate_boxes(d_ref, rows, cfg)
                by_tid = {plan.row_map[r.row_id]: ww for r, ww in zip(plan.batch.rows, want)}
                for v in checked:
                    tally["roi_rows"] += 1
                    ww = by_tid.get(v["track_id"])
                    got = v["counts"]
                    if ww is not None and [ww["roi_pixels"], ww["roi_valid_pixels"], ww["roi_front_pixels"],
                                           ww["roi_behind_pixels"]] == [got["pixels"], got["valid"], got["front"],
                                                                        got["behind"]] \
                            and int(ww["verdict_code"]) == v["code"]:
                        tally["roi_rows_equal"] += 1
                    else:
                        bad.setdefault("roi", []).append(v["track_id"])
        else:
            f = body["failure"].split(":", 1)[0]
            tally["roi_failures"][f] = tally["roi_failures"].get(f, 0) + 1
        last = res["client_last"] or {}
        if last.get("rtt_s") is not None:
            rtts.append(last["rtt_s"] * 1e3)
        if bad:
            bad["frame"] = i
            bad["k"] = k
            mism.write(json.dumps(bad, default=str) + "\n")
    wall = time.monotonic() - t_wall
    g = b_ref.strict_grid()
    if g is not None:      # final reference grid: not vacuous (Nav2 values 0 free, 100 lethal, -1 unknown)
        vals = np.asarray(g.data, dtype=np.int16)
        tally["final_ref_grid"] = {"free": int((vals == 0).sum()), "lethal": int((vals == 100).sum()),
                                   "unknown": int((vals == -1).sum())}
    mism.close()
    cells.close()
    conn.close()
    if server is not None:
        server.stop()
    n = tally["frames"]
    tally["roi_core"] = sorted(x for x in tally["roi_core"] if x)
    all_equal = (tally["result_equal"] == n and tally["delta_equal"] == n and
                 tally["grid_equal"] == tally["ref_fused"] and tally["subj_fused"] == tally["ref_fused"] and
                 tally["roi_rows_equal"] == tally["roi_rows"] and tally["roi_ok"] == n and tally["ref_fused"] > 0 and
                 ctally["presence_equal"] == n and ctally["frames_both_blocks"] > 0 and
                 ctally["flags_equal_frames"] == ctally["frames_both_blocks"] and
                 ctally["origin_equal"] == ctally["frames_both_blocks"] and
                 ctally["sample_counts_equal"] == ctally["frames_both_blocks"])
    caught = None
    if a.variant != "none":
        if a.variant == "clock_mono":
            caught = (tally["bridge_subj"].get("clock_mismatch", 0) == tally["subj_fused"] > 0 and
                      tally["grid_equal"] == 0)
        elif a.variant.startswith("origin_shift1"):
            # the cell comparator alone must see it, in (nearly) every frame
            caught = ctally["frames_both_blocks"] > 0 and \
                ctally["cell_diff_frames"] >= 0.9 * ctally["frames_both_blocks"]
        else:
            caught = not all_equal and (n - tally["delta_equal"]) > 0
    summary = {"prep": os.path.abspath(a.prep), "depth_sha256": meta.get("depth_sha256"),
               "frames_idx": [idx[0], idx[-1], a.stride], "variant": a.variant, "shape": list(shape),
               "height_cell": a.height_cell, "block": block, "ref_frame_src": a.ref_frame_src,
               "ref_frame_src_sha256": (hashlib.sha256(open(a.ref_frame_src, "rb").read()).hexdigest()
                                        if a.ref_frame_src else None), "mount_pitch_deg": a.mount_pitch_deg, "kv": conn.describe(), "fake": a.fake,
               "pace_ms": pace, "numpy": np.__version__, "python": sys.version.split()[0],
               "tally": tally, "cells": ctally, "all_equal": all_equal, "caught": caught,
               "rtt_ms_median": float(np.median(rtts)) if rtts else None,
               "producer_counts": dict(subj.counts), "conn_counts": dict(conn.counts), "connect_attempts": conn.counts.get("connect_attempts"),
               "fake_server_counts": dict(server.counts) if server is not None else None,
               "wall_s": round(wall, 3)}
    with open(os.path.join(a.out, "summary.json"), "w") as fp:
        json.dump(summary, fp, indent=1, sort_keys=True, default=str)
    print(json.dumps({"variant": a.variant, "all_equal": all_equal, "caught": caught, "frames": n,
                      "cells": ctally, "shape": list(shape),
                      "delta_equal": tally["delta_equal"], "grid_equal": tally["grid_equal"],
                      "ref_fused": tally["ref_fused"], "subj_fused": tally["subj_fused"],
                      "bridge_subj": tally["bridge_subj"], "roi_ok": tally["roi_ok"],
                      "roi_rows": tally["roi_rows"], "roi_rows_equal": tally["roi_rows_equal"],
                      "roi_core": tally["roi_core"], "words": tally["words"],
                      "subj_changes": tally.get("subj_changes"), "final_ref_grid": tally.get("final_ref_grid"),
                      "rtt_ms_median": summary["rtt_ms_median"], "numpy": np.__version__}))
    if a.variant != "none":
        return 0 if caught else 3
    return 0 if all_equal else 2


if __name__ == "__main__":
    sys.exit(main())
