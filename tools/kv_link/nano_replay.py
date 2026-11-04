#!/usr/bin/env python3
"""Nano end of the KV260 frame channel: a paced replay through the real consumers.

A capture thread plays a prepared recording (depth_u16.bin + meta.json, depth unit
from meta) at a fixed rate: each tick stamps CLOCK_MONOTONIC_RAW (host/fpga_clock) as the acquisition
time, gives the frame a new increasing frame_id and drops it into a one-slot
mailbox (an unconsumed older frame is overwritten and counted: the camera does not
wait for the map). The consumer thread takes the newest frame, converts the depth
codes with the reference rule, and runs M7Pipeline.ingest_geometry (backend
remote_pl = the KV260, or cpu = local reference) and RollingFusion.ingest, exactly
as the product does. Timing is on the Nano's CLOCK_MONOTONIC_RAW only; the KV260
reports durations, never timestamps.

  frame age (age_ms)  = moment the result is handed to the rolling map - acquisition
  channel_ms          = age - KV260 core device time - Nano mailbox wait
  transport_ms        = round trip - KV260 service time (slot wait..core), see host/kv_link.py

Writes into a fresh --out: frames.csv (one row per consumed frame), flags.dump +
dump_index.csv (voxel flags of every answered frame, by source frame index),
summary.json (counts, percentiles, client/server counters, /proc/net/snmp deltas of
both ends, CPU). --compare then recomputes every dumped frame with the local CPU
reference and compares voxel by voxel (tools/geo_same_input/compare.py rules,
with its two positive controls).
--roi-memory SNAP.json (remote_pl only; PC_DATAPLANE A.2): every frame also carries the
memory boxes of a fixed snapshot ({"memory_revision", "target_label", "tracks": [{"id",
"label", "bounds": [[lo xyz], [hi xyz]], "last_seen", "position_m"}]}; fixed camera, so world =
camera frame), planned by host.m7_object_native.plan_boxes; the KV260's per-box answers go to
roi.jsonl. With --compare every answered frame's rows are recomputed by the local C reference
(evaluate_boxes, same float32 depth) and compared field by field (positive control: one field
changed must be caught). --make-roi-memory SNAP.json writes such a snapshot from one source
frame's depth (boxes in front of / on / behind the measured surface, plus out-of-view ones).
Car AR (2026-10-11): with --roi-memory each ROI row also carries its track's class name
("label" of the snapshot, English ASCII, <= 8 bytes) in the REQ LABEL block, but only when the
server announces feature label_ext (kv_server --ar drm); --no-roi-labels sends none.
Replay mark (10-12): this tool replays a recording, so every request carries the replay mark
(ext_flags bit3) whenever the server announces feature replay_ext (kv_server --ar drm): the AR
panel says REPLAY, never LIVE.
AR colour (2026-10-16, host/kv_colour.py): --colour-port P (the KV's colour bypass port, kv_server
--ar-colour-port) serves the prepared colour of each frame (<prep>/colour.json + colour_rgb8.bin from
tools/geo_same_input/prep_colour.py, D2C aligned, same frame order as the depth) on request; every
request then carries the sensor key = source frame index + 1 (STAMP block, only when the server
announces stamp_ext), so the KV pairs exactly. Without --colour-port nothing changes.
--colour-watermark (test path only, default off): the served colour carries the source key in the first
and last 4 pixels of every row (colour_watermark), so a board dump shows tearing / stale colour.
E7B (2026-11-03): meta.json may carry "camera_to_world" (4x4, world <- camera; e.g. a car recording with
the static bench pose of its E2 pack); absent = identity, exactly as before (ROI boxes stay planned in the
camera frame). --map-dump (rolling remote_pl only) writes the KV rolling map after the last frame to
map_final.bin and its sha256 into summary.json (cross-arm equality of the fusion output).
Usage: nano_replay.py --prep DIR --out DIR --hz 10 --backend remote_pl --server HOST:PORT
                      [--passes 1] [--max-frames N] [--chunk-bytes 60000] [--compare]
                      [--roi-memory SNAP.json]
       nano_replay.py --prep DIR --make-roi-memory SNAP.json [--roi-frame 0] [--roi-boxes 24]
"""
import argparse
import csv
import json
import os
import resource
import sys
import tempfile
import threading
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host.kv_link import (CHUNK_FRAGMENTED, LatestSlot, chunk_loss, read_snmp,  # noqa: E402
                          remote_pl_factory, snmp_delta)
from host.m7_backend import M7BackendSelector  # noqa: E402
from host.fpga_clock import CLOCK_ID, now_ns as raw_ns  # noqa: E402
from host.m7_frame_native import compile_frame_native  # noqa: E402
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,  # noqa: E402
                              PoseAtCapture)
from host.m7_rolling import RollingFusion, compile_rolling  # noqa: E402
from tools.geo_same_input.common import (depth_u16_to_m, meta_depth_unit_m,  # noqa: E402
                                         record_bytes)

CLOCK = CLOCK_ID                 # "nano_monotonic_raw_ns" (host/fpga_clock.py)
LOCAL_FRAME = "camera_fixed_replay"
POSE_EPOCH = "fixed_camera_replay_v1"


def load(prep):
    meta = json.load(open(os.path.join(prep, "meta.json")))
    w, h, n = int(meta["width"]), int(meta["height"]), int(meta["frames"])
    depth = np.memmap(os.path.join(prep, meta["depth_file"]), dtype="<u2", mode="r")
    if depth.size != w * h * n:
        raise ValueError("depth file holds %d values, meta says %d" % (depth.size, w * h * n))
    return meta, depth.reshape(n, h, w)


def pct(values, q):
    return round(float(np.percentile(np.asarray(values, dtype=np.float64), q)), 3) if len(values) else None


def dist(values):
    return {"n": len(values), "median": pct(values, 50), "p95": pct(values, 95),
            "p99": pct(values, 99), "p99_9": pct(values, 99.9),
            "max": round(float(max(values)), 3) if len(values) else None,
            "p99_9_note": "needs >= 1000 samples to mean anything" if len(values) < 1000 else ""}


class Capture(threading.Thread):
    def __init__(self, slot, n_src, total, hz):
        super().__init__(name="capture", daemon=True)
        self.slot, self.n_src, self.total, self.period = slot, n_src, total, 1.0 / hz
        self.issued = self.late = 0
        self.finished = threading.Event()

    def run(self):
        tick = time.perf_counter()
        for seq in range(self.total):
            tick += self.period
            now = time.perf_counter()
            if now > tick:
                self.late += 1
            else:
                time.sleep(tick - now)
            self.slot.put({"frame_id": seq + 1, "src": seq % self.n_src,
                           "acquired_ns": raw_ns()})
            self.issued += 1
        self.finished.set()


WATERMARK_PX = 4          # test mark: this many pixels at both ends of every colour row
WATERMARK_B = 0xA5


def colour_watermark(img, key):
    """TEST PATH ONLY (--colour-watermark; board check of tearing / stale colour, 2026-10-16): a copy of
    the H x W x 3 colour with the source key written into the first and last WATERMARK_PX pixels of
    every row: R = key & 0xFF, G = (key >> 8) & 0xFF, B = 0xA5."""
    out = np.array(img, dtype=np.uint8, copy=True)
    k = int(key)
    mark = np.array([k & 0xFF, (k >> 8) & 0xFF, WATERMARK_B], dtype=np.uint8)
    out[:, :WATERMARK_PX] = mark
    out[:, -WATERMARK_PX:] = mark
    return out


def read_colour_watermark(pixels_rgb):
    """Key of one mark pixel (R, G, B) or None when B is not the mark byte."""
    r, g, b = (int(v) for v in pixels_rgb)
    return (g << 8 | r) if b == WATERMARK_B else None


def start_colour_source(prep, kv_addr, shape, n_src, watermark=False):
    """ColourSource serving <prep>/colour_rgb8.bin (N x H x W x 3, frame order of the depth file):
    key k -> source frame k - 1 (exact; tolerance irrelevant). ``watermark`` (test path only): every
    frame served carries colour_watermark(frame, k)."""
    from host.kv_colour import ColourSource
    cmeta = json.load(open(os.path.join(prep, "colour.json")))
    h, w = shape
    if (int(cmeta["width"]), int(cmeta["height"]), int(cmeta["frames"])) != (w, h, n_src):
        raise SystemExit("colour.json %sx%s x %s does not match the depth %dx%d x %d"
                         % (cmeta["width"], cmeta["height"], cmeta["frames"], w, h, n_src))
    rgb = np.memmap(os.path.join(prep, cmeta["colour_file"]), dtype=np.uint8, mode="r").reshape(n_src, h, w, 3)

    def provider(key, _tol_ns):
        k = int(key)
        if not 1 <= k <= n_src:
            return "no_match"
        if watermark:
            return k, w, h, colour_watermark(rgb[k - 1], k).tobytes()
        return k, w, h, rgb[k - 1].tobytes()
    info = {"source": "replay", "width": w, "height": h, "prep": os.path.abspath(prep)}
    if watermark:                       # default path: HELLO exactly as before
        info["watermark"] = True
    return ColourSource(kv_addr, provider, info=info).start()


def make_roi_memory(depth_m, K, n_boxes, seed=20261011):
    """Snapshot of boxes back-projected from one frame (camera frame = world): for valid
    pixels on a grid, a box in front of the surface (expect free_space_conflict), on it
    (indeterminate) or behind it (occluded), half-size 3-8 % of the depth; plus two out of view."""
    rng = np.random.RandomState(seed)
    h, w = depth_m.shape
    kinv = np.linalg.inv(np.asarray(K, dtype=np.float64))
    tracks = []
    cand = [(u, v) for v in range(h // 8, h - h // 8, max(1, h // 8))
            for u in range(w // 8, w - w // 8, max(1, w // 10))]
    rng.shuffle(cand)
    kinds = ("front", "surface", "behind")
    for u, v in cand:
        if len(tracks) >= n_boxes - 2:
            break
        z = float(depth_m[v, u])
        if not np.isfinite(z) or not 0.4 < z < 6.0:
            continue
        kind = kinds[len(tracks) % 3]
        zc = {"front": 0.55 * z, "surface": z, "behind": 1.6 * z}[kind]
        p = zc * (kinv @ np.array([u, v, 1.0]))
        half = float(rng.uniform(0.03, 0.08)) * z
        tracks.append({"id": "t%02d_%s" % (len(tracks), kind), "label": ["cup", "bottle", "book"][len(tracks) % 3],
                       "bounds": [(p - half).tolist(), (p + half).tolist()],
                       "last_seen": float(len(tracks)), "position_m": p.tolist()})
    tracks.append({"id": "t_behind_camera", "label": "cup", "bounds": [[-0.1, -0.1, -1.0], [0.1, 0.1, -0.8]],
                   "last_seen": 0.0, "position_m": [0.0, 0.0, -0.9]})
    tracks.append({"id": "t_far_left", "label": "book", "bounds": [[-9.0, 0.0, 1.0], [-8.8, 0.2, 1.2]],
                   "last_seen": 0.0, "position_m": [-8.9, 0.1, 1.1]})
    return {"memory_revision": 7, "target_label": "bottle", "tracks": tracks}


def roi_plan(snap, shape, K, cfg):
    from host.m7_object_native import memory_priority, plan_boxes
    entries = [(t["id"], np.asarray(t["bounds"], dtype=np.float64)) for t in snap["tracks"]]
    tracks = {t["id"]: t for t in snap["tracks"]}
    plan = plan_boxes(shape, K, np.eye(4), entries, cfg,
                      priority=memory_priority(tracks, snap.get("target_label")))
    if plan.batch is None:
        raise SystemExit("--roi-memory: no box of the snapshot is in view (host resolved %r)"
                         % plan.host_resolved)
    return plan, plan.batch.with_revision(int(snap["memory_revision"]))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--make-roi-memory", default=None)
    ap.add_argument("--roi-frame", type=int, default=0)
    ap.add_argument("--roi-boxes", type=int, default=24)
    ap.add_argument("--roi-memory", default=None)
    ap.add_argument("--no-roi-labels", action="store_true",
                    help="do not attach the snapshot's class names (car AR LABEL block)")
    pre, _ = ap.parse_known_args()
    if pre.make_roi_memory:
        meta, depth = load(pre.prep)
        unit, _src = meta_depth_unit_m(meta, None)
        snap = make_roi_memory(depth_u16_to_m(np.asarray(depth[pre.roi_frame]), unit),
                               meta["K"], pre.roi_boxes)
        snap["source"] = {"prep": os.path.abspath(pre.prep), "frame": pre.roi_frame,
                          "depth_sha256": meta.get("depth_sha256")}
        with open(pre.make_roi_memory, "w") as fp:
            json.dump(snap, fp, indent=1)
        print(json.dumps({"tracks": len(snap["tracks"]), "out": pre.make_roi_memory}))
        return
    ap.add_argument("--out", required=True)
    ap.add_argument("--hz", type=float, required=True)
    ap.add_argument("--backend", choices=("remote_pl", "cpu"), required=True)
    ap.add_argument("--server", default=None, help="HOST:PORT of kv_server.py (remote_pl)")
    ap.add_argument("--rolling-backend", choices=("cpu", "remote_pl"), default="cpu",
                    help="remote_pl: fuse on the KV260 PL rolling core (kv_server --rolling pl, v3 L2)")
    ap.add_argument("--passes", type=int, default=1)
    ap.add_argument("--max-frames", type=int, default=0, help="stop after this many captured frames")
    ap.add_argument("--chunk-bytes", type=int, default=CHUNK_FRAGMENTED)
    ap.add_argument("--timeout-s", type=float, default=2.0)
    ap.add_argument("--depth-unit-m", type=float, default=None,
                    help="only for a prepared input whose meta.json lacks depth_unit_m")
    ap.add_argument("--compare", action="store_true")
    ap.add_argument("--map-dump", action="store_true",
                    help="after the last frame, dump the rolling map (remote_pl rolling) to map_final.bin")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--colour-port", type=int, default=None,
                    help="AR colour: the KV's colour bypass port (host = --server's); needs <prep>/colour.json")
    ap.add_argument("--colour-watermark", action="store_true",
                    help="TEST ONLY: mark the source key into both ends of every colour row (board check of "
                         "tearing / stale colour); off by default")
    a = ap.parse_args()
    meta, depth = load(a.prep)
    unit, unit_source = meta_depth_unit_m(meta, a.depth_unit_m)
    n_src, h, w = depth.shape
    total = n_src * a.passes
    if a.max_frames:
        total = min(total, a.max_frames)
    os.makedirs(a.out, exist_ok=False)
    build = tempfile.mkdtemp(prefix="nano_replay_")
    K = np.asarray(meta["K"], dtype=np.float64)
    T_cw = np.asarray(meta.get("camera_to_world", np.eye(4)), dtype=np.float64)
    if T_cw.shape != (4, 4):
        raise ValueError("meta camera_to_world must be 4x4")
    g = meta["grid"]
    grid = GridSpec(tuple(float(v) for v in g["origin_m"]), float(g["voxel_m"]),
                    tuple(int(v) for v in g["shape_xyz"]))
    if a.backend == "remote_pl":
        if not a.server:
            ap.error("--backend remote_pl needs --server HOST:PORT")
        host, port = a.server.rsplit(":", 1)
        sel = M7BackendSelector(remote_pl_factory=remote_pl_factory(
            (host, int(port)), depth_unit_m=unit, chunk_bytes=a.chunk_bytes, timeout_s=a.timeout_s))
    else:
        sel = M7BackendSelector(cpu_frame_library=compile_frame_native(os.path.join(build, "frame.so")))
    backend = sel.frame_backend(a.backend)
    if a.backend == "remote_pl":
        backend.data_source = "replay"          # a recording, always (replay mark)
    client = getattr(backend, "client", None)
    colour_src = None
    if a.colour_port is not None:
        if a.backend != "remote_pl":
            ap.error("--colour-port needs --backend remote_pl")
        colour_src = start_colour_source(a.prep, (host, a.colour_port), (h, w), n_src,
                                         watermark=a.colour_watermark)
    pipe = M7Pipeline(backend, grid)
    if a.rolling_backend == "remote_pl":
        if a.backend != "remote_pl":
            ap.error("--rolling-backend remote_pl needs --backend remote_pl")
        from host.kv_rolling import remote_rolling_factory
        sel.remote_rolling_factory = remote_rolling_factory((host, int(port)))
        roll = sel.rolling_backend("remote_pl")
    else:
        roll = sel.rolling_backend("cpu", cpu_library=compile_rolling(os.path.join(build, "roll.so")))
    fusion = RollingFusion(roll, voxel_m=grid.voxel_m)
    roi_batch = roi_plan_info = roi_labels = None
    roi_out = None
    if a.roi_memory:
        if a.backend != "remote_pl":
            ap.error("--roi-memory needs --backend remote_pl")
        from perception.fpga_replay.core import Config
        snap = json.load(open(a.roi_memory))
        plan, roi_batch = roi_plan(snap, (h, w), K, Config())
        if not a.no_roi_labels:     # class name of each row's track (row order = batch order)
            by_id = {t["id"]: t for t in snap["tracks"]}
            roi_labels = [by_id[plan.row_map[r.row_id]].get("label") for r in roi_batch.rows]
        roi_plan_info = {"snapshot": os.path.abspath(a.roi_memory), "tracks": len(snap["tracks"]),
                         "rows_sent": len(roi_batch.rows), "host_resolved": plan.host_resolved,
                         "overflow": plan.overflow,
                         "row_map": {str(k): v for k, v in plan.row_map.items()},
                         "memory_revision": roi_batch.memory_revision,
                         "labels": roi_labels,
                         "server_label_ext": (client.server_info or {}).get("features", {})
                         .get("label_ext") if client else None}
        roi_out = open(os.path.join(a.out, "roi.jsonl"), "w")
    stream = "kvlink-%s-%d" % (a.tag or os.path.basename(os.path.normpath(a.out)), os.getpid())
    calib = str(meta.get("calibration_id", "unknown"))

    server_reset = client.stats(reset=True) if client else None
    client_base = dict(client.counts) if client else {}
    snmp0 = read_snmp()
    slot = LatestSlot()
    cap = Capture(slot, n_src, total, a.hz)
    rows = []
    dump = open(os.path.join(a.out, "flags.dump"), "wb")
    index = []
    ru0, wall0 = resource.getrusage(resource.RUSAGE_SELF), time.monotonic()
    cap.start()
    while True:
        item = slot.take(timeout=0.05)
        if item is None:
            if cap.finished.is_set():
                break
            continue
        t_take = raw_ns()
        acq = item["acquired_ns"]
        codes = np.asarray(depth[item["src"]])
        depth_m = depth_u16_to_m(codes, unit)
        t_conv = raw_ns()
        ident = CaptureIdentity(stream, item["frame_id"], acq, CLOCK, calib, LOCAL_FRAME, POSE_EPOCH)
        pose = PoseAtCapture(acq, CLOCK, LOCAL_FRAME, POSE_EPOCH, True, T_cw)
        if colour_src is not None:
            backend.set_next_sensor_key(item["src"] + 1)     # recording frame key (never 0)
        if roi_batch is not None:
            # A frame the pipeline drops before the backend (e.g. the tail frame that
            # expired in the mailbox) must not inherit the previous frame's ROI answer
            # (board run 10-11: frame 1240/src 309 carried src 304's rows).
            backend.last_roi = None
            backend.set_next_roi(roi_batch, labels=roi_labels)
        out = pipe.ingest_geometry(DepthFrame(ident, depth_m, K, pose), now_ns=raw_ns(),
                                   now_clock_id=CLOCK)
        t_geo = raw_ns()
        nav = out.navigation
        fused = fusion.ingest(nav, pipe, now_ns=t_geo, clock_id=CLOCK, robot_position_m=(0., 0., 0.))
        t_fused = raw_ns()
        last = getattr(backend.lib, "last", None) if client else None
        if client:
            backend.lib.last = None
        row = {"frame_id": item["frame_id"], "src": item["src"],
               "mailbox_wait_ms": (t_take - acq) / 1e6, "convert_ms": (t_conv - t_take) / 1e6,
               "geometry_ms": (t_geo - t_conv) / 1e6, "fusion_ms": (t_fused - t_geo) / 1e6,
               "age_ms": (t_geo - acq) / 1e6, "nav_valid": int(nav.valid), "nav_reason": nav.reason,
               "fused": int(fused.accepted), "fusion_reason": fused.reason}
        if last and "rtt_s" in last:
            kv = last["kv_ns"]
            row.update({"quantize_ms": last["quantize_s"] * 1e3, "encode_ms": last["encode_s"] * 1e3,
                        "send_ms": last["send_s"] * 1e3, "rtt_ms": last["rtt_s"] * 1e3,
                        "unpack_ms": last["unpack_s"] * 1e3, "tx_chunks": last["tx_chunks"],
                        "kv_service_ms": last["kv_service_s"] * 1e3,
                        "transport_ms": last["transport_s"] * 1e3})
            for k, v in kv.items():
                row["kv_%s_ms" % k] = v / 1e6
            row["channel_ms"] = row["age_ms"] - row["kv_core_device_ms"] - row["mailbox_wait_ms"]
        elif last:
            row["link_failure"] = last.get("failed")
        if roi_batch is not None:
            ro = backend.last_roi
            row["roi_ok"] = int(bool(ro and ro.ok))
            row["roi_reason"] = (ro.reason if ro else "no_outcome") or ""
            row["roi_core"] = (ro.roi_core if ro else "") or ""
            if ro and ro.times_ns:
                row["roi_total_ms"] = ro.times_ns.get("total", 0) / 1e6
            roi_out.write(json.dumps({"frame_id": item["frame_id"], "src": item["src"],
                                      "ok": bool(ro and ro.ok), "reason": ro.reason if ro else None,
                                      "roi_core": ro.roi_core if ro else None,
                                      "times_ns": ro.times_ns if ro else None,
                                      "rows": ro.rows if ro else []}) + "\n")
        if nav.valid:
            p = nav.payload
            dump.write(record_bytes((p["valid_grid_samples"], p["invalid_depth_or_projection"],
                                     p["outside_grid"]), p["voxel_flags"]))
            index.append((item["frame_id"], item["src"]))
        rows.append(row)
    wall1, ru1 = time.monotonic(), resource.getrusage(resource.RUSAGE_SELF)
    dump.close()
    snmp1 = read_snmp()
    server_end = client.stats() if client else None
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(os.path.join(a.out, "frames.csv"), "w", newline="") as fp:
        wr = csv.DictWriter(fp, fieldnames=keys)
        wr.writeheader()
        wr.writerows(rows)
    with open(os.path.join(a.out, "dump_index.csv"), "w") as fp:
        fp.write("frame_id,src\n")
        fp.writelines("%d,%d\n" % pair for pair in index)
    answered = [r for r in rows if "rtt_ms" in r]
    reasons, fusion_reasons = {}, {}
    for r in rows:
        reasons[r["nav_reason"]] = reasons.get(r["nav_reason"], 0) + 1
        fusion_reasons[r["fusion_reason"]] = fusion_reasons.get(r["fusion_reason"], 0) + 1
    client_counts = {}
    if client:
        for k, v in client.counts.items():
            if v - client_base.get(k, 0):
                client_counts[k] = v - client_base.get(k, 0)
    srv_counts = (server_end or {}).get("counts", {})
    if server_end and "session_counts" in server_end:
        loss, loss_source = chunk_loss(client_counts, server_end["session_counts"]), "per_session"
    else:
        loss, loss_source = {}, ("server without per-session counters: not computed"
                                 if server_end else None)
    map_final = None
    if a.map_dump:
        if a.rolling_backend != "remote_pl":
            map_final = {"error": "--map-dump needs --rolling-backend remote_pl"}
        else:
            import hashlib
            res_m, local_m = roll.snapshot()
            mb = b"" if local_m is None else np.ascontiguousarray(local_m).tobytes()
            with open(os.path.join(a.out, "map_final.bin"), "wb") as fp:
                fp.write(mb)
            map_final = {"ok": bool(res_m.ok), "bytes": len(mb), "sha256": hashlib.sha256(mb).hexdigest()}
    summary = {
        "map_final": map_final,
        "tag": a.tag, "prep": os.path.abspath(a.prep), "depth_sha256": meta.get("depth_sha256"),
        "depth_unit_m": unit, "depth_unit_source": unit_source, "width": w, "height": h,
        "source_frames": n_src, "hz": a.hz, "backend": a.backend,
        "backend_name": getattr(backend, "backend_name", None), "chunk_bytes": a.chunk_bytes,
        "timeout_s": a.timeout_s, "server": a.server,
        "capture": {"issued": cap.issued, "late_ticks": cap.late,
                    "mailbox_superseded_on_nano": slot.superseded, "consumed": slot.takes},
        "consumed_rows": len(rows), "answered": len(answered),
        "nav_reasons": reasons, "fusion_reasons": fusion_reasons, "dumped_frames": len(index),
        "age_ms_answered": dist([r["age_ms"] for r in answered]),
        "channel_ms": dist([r["channel_ms"] for r in answered]),
        "stages_ms_median": {k: pct([r[k] for r in answered], 50) for k in keys
                             if k.endswith("_ms") and answered and k in answered[0]},
        "stages_ms_p99": {k: pct([r[k] for r in answered], 99) for k in keys
                          if k.endswith("_ms") and answered and k in answered[0]},
        "drops": {
            "app_nano_mailbox_superseded": slot.superseded,
            "app_kv_slot_superseded": srv_counts.get("slot_superseded", 0),
            "app_stale_response": client_counts.get("stale_response", 0),
            "app_identity_mismatch": client_counts.get("identity_mismatch", 0),
            "app_superseded_notice": client_counts.get("superseded_notice", 0),
            "net_timeouts": client_counts.get("timeout", 0),
            "net_kv_unfinished_overtaken": srv_counts.get("rx_incomplete_superseded", 0),
            "net_kv_missing_chunks": srv_counts.get("rx_incomplete_missing_chunks", 0),
            # per session and frame traffic only (host.kv_link.chunk_loss): the server's global
            # counters also hold every other client's chunks (SYNC/TELEM/ESTOP sessions)
            "net_chunks_sent_by_nano": loss.get("nano_sent"),
            "net_chunks_received_by_kv": loss.get("kv_received"),
            "net_chunks_lost_nano_to_kv": loss.get("nano_to_kv"),
            "net_chunks_sent_by_kv": loss.get("kv_sent"),
            "net_chunks_received_by_nano": loss.get("nano_received"),
            "net_chunks_lost_kv_to_nano": loss.get("kv_to_nano"),
            "net_chunk_loss_source": loss_source,
        },
        "client_counts": client_counts, "backend_counts": dict(getattr(backend, "counts", {})),
        "colour_source": colour_src.describe() if colour_src is not None else None,
        "snmp_delta_nano": snmp_delta(snmp0, snmp1), "server_window": server_end,
        "server_reset_ack": bool(server_reset) if client else None,
        "nano_cpu_s": round((ru1.ru_utime - ru0.ru_utime) + (ru1.ru_stime - ru0.ru_stime), 3),
        "wall_s": round(wall1 - wall0, 3),
        "python": sys.version.split()[0], "numpy": np.__version__,
    }
    if roi_batch is not None:
        roi_out.close()
        okrows = [r for r in rows if r.get("roi_ok")]
        roi_reasons = {}
        for r in rows:
            roi_reasons[r["roi_reason"]] = roi_reasons.get(r["roi_reason"], 0) + 1
        cores_seen = {}
        for r in rows:
            if "roi_core" in r:
                cores_seen[r["roi_core"]] = cores_seen.get(r["roi_core"], 0) + 1
        summary["roi"] = {"plan": roi_plan_info, "frames_ok": len(okrows), "reasons": roi_reasons,
                          "roi_core_seen": cores_seen,
                          "kv_roi_total_ms": dist([r["roi_total_ms"] for r in okrows
                                                   if "roi_total_ms" in r])}
        if a.compare:
            summary["roi"]["compare"] = compare_roi(a, build, depth, unit, roi_batch)
    if a.compare:
        summary["compare"] = compare(a, build, meta, depth, unit, grid, K, index, T_cw)
    json.dump(summary, open(os.path.join(a.out, "summary.json"), "w"), indent=1, sort_keys=True,
              default=str)
    import shutil
    shutil.rmtree(build, ignore_errors=True)
    brief = {k: summary[k] for k in ("answered", "consumed_rows", "dumped_frames", "nav_reasons")}
    brief["age_median_ms"] = summary["age_ms_answered"]["median"]
    brief["drops"] = summary["drops"]
    if "compare" in summary:
        brief["compare_equal"] = summary["compare"].get("equal")
        brief["controls_pass"] = summary["compare"].get("controls_pass")
    print(json.dumps(brief, default=str))
    if client:
        client.close()
    if colour_src is not None:
        colour_src.stop()
    if "roi" in summary and "compare" in summary["roi"]:
        brief["roi_compare_equal"] = summary["roi"]["compare"].get("equal")
        brief["roi_frames_ok"] = summary["roi"]["frames_ok"]
    if "compare" in summary and not summary["compare"].get("equal"):
        sys.exit(1)
    if "roi" in summary and "compare" in summary["roi"] and not summary["roi"]["compare"].get("equal"):
        sys.exit(1)


def compare_roi(a, build, depth, unit, batch):
    """Local C reference (evaluate_boxes) on the same float32 depth, row by row."""
    from host.m7_object_native import NativeObjectEvidence, compile_object_native
    native = NativeObjectEvidence(compile_object_native(os.path.join(build, "obj_ref.so")))
    boxes = [(r.row_id, r.xyxy, r.near_z, r.far_z) for r in batch.rows]
    frames = rows_cmp = 0
    diffs = []
    first = None
    for line in open(os.path.join(a.out, "roi.jsonl")):
        rec = json.loads(line)
        if not rec["ok"]:
            continue
        ref = native.evaluate_boxes(depth_u16_to_m(np.asarray(depth[rec["src"]]), unit), boxes, batch.cfg)
        frames += 1
        rows_cmp += len(ref)
        if first is None:
            first = (rec["rows"], ref)
        if len(ref) != len(rec["rows"]) or any(x != y for x, y in zip(ref, rec["rows"])):
            diffs.append(rec["frame_id"])
    control = None
    if first is not None:              # positive control: one changed field must be caught
        bad = [dict(r) for r in first[0]]
        bad[0]["roi_valid_pixels"] += 1
        control = any(x != y for x, y in zip(first[1], bad))
    verdicts = {}
    for line in open(os.path.join(a.out, "roi.jsonl")):
        for r in json.loads(line)["rows"]:
            verdicts[r["verdict_code"]] = verdicts.get(r["verdict_code"], 0) + 1
    return {"frames_compared": frames, "rows_compared": rows_cmp, "frames_different": diffs[:20],
            "n_frames_different": len(diffs), "control_caught": control,
            "verdict_code_counts": {str(k): v for k, v in sorted(verdicts.items())},
            "equal": bool(frames > 0 and not diffs and control)}


def compare(a, build, meta, depth, unit, grid, K, index, T_cw=None):
    """CPU reference on every dumped source frame, same conversion; voxel by voxel."""
    from host.m7_frame_native import FrameNativeM7
    from tools.geo_same_input.common import read_dump
    from tools.geo_same_input.compare import controls, diff_report
    if not index:
        return {"equal": False, "error": "no answered frames to compare"}
    ref = FrameNativeM7(compile_frame_native(os.path.join(build, "ref.so")))
    path = os.path.join(a.out, "ref.dump")
    with open(path, "wb") as fp:
        for fid, src in index:
            ev = ref.frame_from_depth(
                depth_m=depth_u16_to_m(np.asarray(depth[src]), unit), k=K, camera_to_world=np.eye(4) if T_cw is None else T_cw,
                grid_origin_m=grid.origin_m, voxel_size_m=grid.voxel_m, shape=grid.shape,
                frame_id=fid, capture_time_ns=1, pose_time_ns=1, calibration_version="ref",
                pose_confirmed=True)
            fp.write(record_bytes((ev.accepted_rays, ev.invalid_depth_or_projection, ev.outside_grid),
                                  ev.flags))
    nvox = int(np.prod(grid.shape))
    rc, rf = read_dump(path, nvox)
    cc, cf = read_dump(os.path.join(a.out, "flags.dump"), nvox)
    rep = diff_report(rc, rf, cc, cf, shape=grid.shape)
    ok, ctl = controls(rc, rf) if len(index) >= 2 else (False, {"note": "need >= 2 frames"})
    rep.update(controls_pass=ok, controls=ctl, frames_compared=len(index))
    rep["equal"] = bool(rep["equal"] and ok)
    return rep


if __name__ == "__main__":
    main()
