#!/usr/bin/env python3
"""E7 host check: kv_server --kv-fast native vs off on a prepared recording (rec30: 310 frames).

Two real kv_server.py processes on this host (mock PL device = the g++ cores behind the register map;
geometry core + ROI core + rolling core, i.e. --core pl --device mock --wait irq --roi-core pl --rolling pl,
the car's service shape minus AR), one with --kv-fast off (the Python path, the reference) and one with
--kv-fast native. Each is driven by the SAME deterministic Nano-side chain as tools/kv_link/nano_replay.py
(M7Pipeline + remote_pl frame backend, ROI boxes of one snapshot, RollingFusion on the KV rolling core),
but sequentially -- one frame after the other, nothing dropped -- so the two runs see identical inputs:

  geometry   every frame's voxel flags + the three counts the Nano consumes (nav payload)
  evidence   every frame's ROI rows (the per-box verdicts and pixel counts) as the Nano decodes them
  fusion     every frame's fusion outcome + the rolling map DUMP after the last frame (int8 log-odds)
  depth      every frame's KV float32 depth crc32 equals the Nano's (checked by the client per frame;
             any mismatch fails that frame), and the harness records it

Output (fresh --out): <arm>/records.jsonl (per frame), <arm>/map_final.bin, compare.json (byte equality
per category, first differing frame, plus a positive control: one flag byte of one arm flipped must be
caught). Exit 0 only when every category is byte-equal and the positive control fires.
Usage: rec30_equivalence.py --prep DIR --out DIR [--frames 310] [--python python3]
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def start_server(py, mode, out):
    cmd = [py, os.path.join(ROOT, "tools/kv_link/kv_server.py"), "--core", "pl", "--device", "mock",
           "--wait", "irq", "--roi-core", "pl", "--rolling", "pl", "--port", "0", "--bind", "127.0.0.1",
           "--out", out, "--max-idle-s", "120", "--kv-fast", mode]
    err = open(os.path.join(out + ".err"), "w")
    env = dict(os.environ, PYTHONPATH=ROOT, OPENBLAS_NUM_THREADS="1")
    env.pop("FPGA_KV_FAST", None)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=err, env=env, cwd=ROOT)
    line = p.stdout.readline().decode()
    if not line:
        p.wait(5)
        raise SystemExit("kv_server (%s) did not start, see %s.err" % (mode, out))
    return p, json.loads(line)


def stop_server(p):
    p.terminate()
    try:
        p.wait(30)
    except subprocess.TimeoutExpired:
        p.kill()


def run_arm(a, mode, meta, depth, unit, snap, arm_dir):
    from host.kv_link import remote_pl_factory
    from host.kv_rolling import remote_rolling_factory
    from host.m7_backend import M7BackendSelector
    from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
    from host.m7_rolling import RollingFusion
    from host.fpga_clock import CLOCK_ID, now_ns
    from perception.fpga_replay.core import Config
    from tools.geo_same_input.common import depth_u16_to_m
    from tools.kv_link.nano_replay import roi_plan

    os.makedirs(arm_dir)
    proc, ready = start_server(a.python, mode, os.path.join(arm_dir, "server"))
    try:
        server = ("127.0.0.1", int(ready["port"]))
        K = np.asarray(meta["K"], dtype=np.float64)
        g = meta["grid"]
        grid = GridSpec(tuple(float(v) for v in g["origin_m"]), float(g["voxel_m"]),
                        tuple(int(v) for v in g["shape_xyz"]))
        sel = M7BackendSelector(remote_pl_factory=remote_pl_factory(server, depth_unit_m=unit, timeout_s=5.0))
        backend = sel.frame_backend("remote_pl")
        backend.data_source = "replay"
        sel.remote_rolling_factory = remote_rolling_factory(server, timeout_s=5.0)
        roll = sel.rolling_backend("remote_pl")
        # map age gate off: a retried frame (below) must not expire the map in one arm only
        fusion = RollingFusion(roll, voxel_m=grid.voxel_m, max_map_age_ns=10 ** 12)
        # age rules off (200 ms output / map age): on this shared host a slow frame would expire in one arm
        # only; ages are a timing property, not the data plane under test
        pipe = M7Pipeline(backend, grid, max_frame_age_ns=10 ** 12, max_cache_age_ns=10 ** 12)
        n, h, w = depth.shape
        plan, batch = roi_plan(snap, (h, w), K, Config())
        info = dict(backend.client.server_info or {})
        rec_fp = open(os.path.join(arm_dir, "records.jsonl"), "w")
        stream = "e7-%s-%d" % (mode, os.getpid())
        kv_times = []
        fid = 0
        retries = 0
        for f in range(min(a.frames, n)):
            codes = np.asarray(depth[f])
            depth_m = depth_u16_to_m(codes, unit)
            # A request lost on this host's loopback (plain user: the server's receive buffer is capped at
            # rmem_max, kv_server rcvbuf_warning) times out in either arm at random; the same source frame is
            # then sent again under a new frame id (the failed attempt is not fused). Counted, not compared.
            for attempt in range(8):
                fid += 1
                acq = now_ns()
                ident = CaptureIdentity(stream, fid, acq, CLOCK_ID, str(meta.get("calibration_id", "x")),
                                        "camera_fixed_replay", "fixed_camera_replay_v1")
                pose = PoseAtCapture(acq, CLOCK_ID, "camera_fixed_replay", "fixed_camera_replay_v1", True,
                                     np.eye(4))
                backend.last_roi = None
                backend.set_next_roi(batch)
                out = pipe.ingest_geometry(DepthFrame(ident, depth_m, K, pose), now_ns=now_ns(),
                                           now_clock_id=CLOCK_ID)
                nav = out.navigation
                if nav.reason != "backend_failure:LinkTimeout":
                    break
                retries += 1
            fused = fusion.ingest(nav, pipe, now_ns=now_ns(), clock_id=CLOCK_ID, robot_position_m=(0., 0., 0.))
            last = backend.lib.last or {}
            backend.lib.last = None
            ro = backend.last_roi
            rec = {"frame": f, "nav_valid": bool(nav.valid), "nav_reason": nav.reason,
                   "fused": bool(fused.accepted), "fusion_reason": fused.reason,
                   "roi_ok": bool(ro and ro.ok), "roi_reason": ro.reason if ro else "no_outcome",
                   "roi_rows": ro.rows if ro else None,
                   # the Nano's own crc of its float32 depth; the client failed the frame if the KV's differed
                   "depth_crc_nano": "%08x" % (int(__import__("zlib").crc32(
                       np.ascontiguousarray(depth_m, dtype=np.float32).tobytes())) & 0xFFFFFFFF)}
            if nav.valid:
                p = nav.payload
                flags = np.ascontiguousarray(p["voxel_flags"], dtype=np.uint8)
                rec.update(counts=[int(p["valid_grid_samples"]), int(p["invalid_depth_or_projection"]),
                                   int(p["outside_grid"])],
                           flags_sha256=hashlib.sha256(flags.tobytes()).hexdigest())
            if "kv_ns" in last:
                kv_times.append({k: v / 1e6 for k, v in last["kv_ns"].items()})
            rec_fp.write(json.dumps(rec, sort_keys=True, default=str) + "\n")
        rec_fp.close()
        res, local = roll.snapshot()
        map_bytes = b"" if local is None else np.ascontiguousarray(local).tobytes()
        with open(os.path.join(arm_dir, "map_final.bin"), "wb") as fp:
            fp.write(map_bytes)
        stats = backend.client.stats()
        summary = {"mode": mode, "link_timeout_retries": retries, "server_info_kv_fast": (stats.get("info") or {}).get("kv_fast"),
                   "hello": info, "client_counts": dict(backend.client.counts),
                   "backend_counts": dict(getattr(backend.lib, "counts", {})),
                   "map_ok": bool(res.ok), "map_sha256": hashlib.sha256(map_bytes).hexdigest(),
                   "server_counts": stats.get("total_counts"),
                   "kv_ms_median_host": {k: float(np.median([t[k] for t in kv_times])) for k in
                                         (kv_times[0] if kv_times else {})}}
        json.dump(summary, open(os.path.join(arm_dir, "summary.json"), "w"), indent=1, sort_keys=True,
                  default=str)
        return summary
    finally:
        stop_server(proc)


def load_records(arm_dir):
    return [json.loads(line) for line in open(os.path.join(arm_dir, "records.jsonl"))]


CATEGORIES = {
    "geometry": ("nav_valid", "nav_reason", "counts", "flags_sha256"),
    "evidence": ("roi_ok", "roi_reason", "roi_rows"),
    "fusion": ("fused", "fusion_reason"),
    "depth": ("depth_crc_nano",),
}


def compare(ref, cand):
    out = {}
    for cat, keys in CATEGORIES.items():
        first = None
        for r, c in zip(ref, cand):
            if any(json.dumps(r.get(k), sort_keys=True) != json.dumps(c.get(k), sort_keys=True) for k in keys):
                first = r["frame"]
                break
        out[cat] = {"equal": first is None and len(ref) == len(cand), "first_diff_frame": first,
                    "frames": len(ref)}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=10 ** 9)
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--roi-boxes", type=int, default=24)
    a = ap.parse_args()
    from tools.geo_same_input.common import depth_u16_to_m, meta_depth_unit_m
    from tools.kv_link.nano_replay import load, make_roi_memory
    meta, depth = load(a.prep)
    unit, _ = meta_depth_unit_m(meta)
    os.makedirs(a.out, exist_ok=False)
    snap = make_roi_memory(depth_u16_to_m(np.asarray(depth[0]), unit), np.asarray(meta["K"]), a.roi_boxes)
    json.dump(snap, open(os.path.join(a.out, "roi_snapshot.json"), "w"), indent=1)
    t0 = time.time()
    sums = {}
    for mode in ("off", "native"):
        sums[mode] = run_arm(a, mode, meta, depth, unit, snap, os.path.join(a.out, mode))
    ref, cand = load_records(os.path.join(a.out, "off")), load_records(os.path.join(a.out, "native"))
    result = compare(ref, cand)
    result["map_final"] = {"equal": sums["off"]["map_sha256"] == sums["native"]["map_sha256"],
                           "sha256": sums["off"]["map_sha256"], "ok": sums["off"]["map_ok"]}
    # positive control: the comparator must see a single flipped flag digest / one ROI count
    mut = [dict(r) for r in ref]
    k = len(mut) // 2
    if mut[k].get("flags_sha256"):
        mut[k]["flags_sha256"] = mut[k]["flags_sha256"][:-1] + ("0" if mut[k]["flags_sha256"][-1] != "0" else "1")
    pc_geo = compare(ref, mut)["geometry"]
    mut = [dict(r) for r in ref]
    rows = [dict(x) for x in (mut[k].get("roi_rows") or [])]
    if rows:
        rows[0]["roi_valid_pixels"] = int(rows[0].get("roi_valid_pixels", 0)) + 1
        mut[k]["roi_rows"] = rows
    pc_roi = compare(ref, mut)["evidence"]
    result["positive_control"] = {"geometry_flip_caught": pc_geo["first_diff_frame"] == k,
                                  "evidence_count_caught": pc_roi["first_diff_frame"] == k}
    stats = {}
    for mode in ("off", "native"):
        recs = ref if mode == "off" else cand
        stats[mode] = {"frames": len(recs), "nav_valid": sum(r["nav_valid"] for r in recs),
                       "fused": sum(r["fused"] for r in recs), "roi_ok": sum(r["roi_ok"] for r in recs),
                       "kv_fast": sums[mode]["server_info_kv_fast"],
                       "link_timeout_retries": sums[mode]["link_timeout_retries"],
                       "client_counts": sums[mode]["client_counts"],
                       "kv_ms_median_host": sums[mode]["kv_ms_median_host"]}
    result["arms"] = stats
    result["wall_s"] = round(time.time() - t0, 1)
    ok = (all(result[c]["equal"] for c in CATEGORIES) and result["map_final"]["equal"]
          and all(result["positive_control"].values())
          and stats["native"]["kv_fast"] and stats["native"]["kv_fast"].get("impl") == "native"
          and stats["off"]["kv_fast"] is None and stats["off"]["nav_valid"] == stats["off"]["frames"])
    result["PASS"] = bool(ok)
    json.dump(result, open(os.path.join(a.out, "compare.json"), "w"), indent=1, sort_keys=True, default=str)
    print(json.dumps({"PASS": result["PASS"], "geometry": result["geometry"]["equal"],
                      "evidence": result["evidence"]["equal"], "fusion": result["fusion"]["equal"],
                      "map_final": result["map_final"]["equal"], "frames": len(ref),
                      "positive_control": result["positive_control"], "out": a.out}))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
