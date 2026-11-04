"""Render AR-overlay replay videos from recorded data (D, 2026-10-11).

  PYTHONHASHSEED=0 PYTHONPATH=. python3 tools/ar_display/render_replay.py \
      --dataset tum|rec30 --prep <prepared dir> [--rec <rec30 dir>] \
      --out <dir> --scratch <dir> [--golden-stride 1]

Every frame goes through the C reference of the AR core (hw/m7/ar_overlay.cpp
via the g++ adapter); frames on --golden-stride are also rendered by the
numpy golden and compared pixel by pixel. Element sources (spec s.4):
  depth colour / nearest Z : prepared depth_u16 * depth_unit_m -> float32 m ->
                             host quantize_optical_z (no shortcut for rec30)
  minimap                  : clipped A0 geometry core (8e21077 frame_depth.cpp)
                             on the same float32 depth, identity camera pose
                             (meta pose = fixed_camera_identity), layer y=16
                             = camera-y [0, 50) mm
  boxes                    : E1 check_boxes roi_xyxy + verdict_code
      rec30: first-frame detections (color_index 84, score >= 0.5) -> mask ->
             median valid depth in mask, mask-centroid ray -> synthetic_bounds
             (class size, up_axis=1) -> check_boxes every frame, identity pose
      tum  : the 8 anchors of tools/m7_object_roi_tum_demo.py (reference
             trajectory poses); frames without a pose get no boxes
  replay=1, age_valid=0 (AGE N/A), permit_present=0 (DRV N/A), risk N/A,
  object_evidence=0 (OBJ ?), frame id = TUM frame index / rec30 seq.
Detection boxes themselves are never drawn.
"""
import argparse
import ctypes
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
import check_golden  # noqa: E402
import golden  # noqa: E402
from host.m7_display_native import quantize_optical_z  # noqa: E402
from host.m7_object_native import NativeObjectEvidence, compile_object_native  # noqa: E402
from host.m7_object_sizes import synthetic_bounds  # noqa: E402

GEOM_REV = "8e21077"
MAX_PIXELS = 1280 * 800


def build_geometry(scratch):
    d = os.path.join(scratch, "geom_" + GEOM_REV)
    os.makedirs(d, exist_ok=True)
    for f in ("frame_depth.cpp", "frame_depth.hpp"):
        txt = subprocess.check_output(["git", "-C", ROOT, "show", "%s:hw/m7/%s" % (GEOM_REV, f)])
        open(os.path.join(d, f), "wb").write(txt)
    so = os.path.join(d, "frame_depth.so")
    subprocess.check_call(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                           "-ffp-contract=off", "-fPIC", "-shared",
                           os.path.join(d, "frame_depth.cpp"), "-o", so])
    lib = ctypes.CDLL(so)
    F32 = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
    F64 = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
    U8 = np.ctypeslib.ndpointer(dtype=np.uint8, flags="C_CONTIGUOUS")
    U32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags="C_CONTIGUOUS")
    lib.fpga_m7_depth_frame.argtypes = [F32, F32, F32, F64, ctypes.c_double] + \
        [ctypes.c_uint32] * 5 + [U8, U32, U32]
    lib.fpga_m7_depth_frame.restype = None
    return lib


def run_geometry(lib, depth_m, ik, origin, voxel):
    h, w = depth_m.shape
    buf = np.zeros(MAX_PIXELS, np.float32)
    buf[:depth_m.size] = depth_m.ravel()
    flags = np.full(32 ** 3, 99, np.uint8)
    counts = np.zeros(3, np.uint32)
    status = np.full(1, 99, np.uint32)
    wc = np.ascontiguousarray(np.eye(3, 4, dtype=np.float32).ravel())
    lib.fpga_m7_depth_frame(buf, ik, wc, origin, float(voxel), w, h, 32, 32, 32,
                           flags, counts, status)
    return int(status[0]), flags


def rec30_entries(rec, depth0_m, K):
    sys.path.insert(0, ROOT)
    from tools.nano_loop.chain import mask_full
    from tools.nano_detector.nano_detector import Letterbox
    frames = [json.loads(l) for l in open(os.path.join(rec, "frames.jsonl"))]
    dets = {json.loads(l)["color_index"]: json.loads(l)["dets"]
            for l in open(os.path.join(rec, "detections.jsonl"))}
    ci0 = frames[0]["color_index"]
    h, w = depth0_m.shape
    entries, labels, notes, counter = [], [], [], {}
    for det in dets.get(ci0, []):
        if det["score"] < 0.5:
            continue
        mask = mask_full(det, Letterbox(w, h, 640), w, h)
        sel = mask & np.isfinite(depth0_m)
        if sel.sum() == 0:
            notes.append({"name": det["name"], "skipped": "no valid depth in mask"}); continue
        z = float(np.median(depth0_m[sel]))
        vs, us = np.nonzero(sel)
        u, v = float(us.mean()), float(vs.mean())
        pos = np.linalg.inv(K) @ np.array([u, v, 1.]) * z
        tag = det["name"][:3].upper()
        idx = counter.get(tag, 0)
        counter[tag] = idx + 1
        label = "%s%d" % (tag, idx)
        entries.append((label, synthetic_bounds(det["name"], pos.tolist(), 1)))
        labels.append(label)
        notes.append({"label": label, "name": det["name"], "score": det["score"],
                      "mask_valid_px": int(sel.sum()), "median_z_m": z,
                      "centroid_px": [u, v], "position_cam_m": pos.tolist()})
    return frames, entries, labels, notes


def tum_entries(meta, depth_m_fn, K):
    from perception.fpga_replay.io import interpolate_pose
    truth = Path(meta["source_rec"]) / "groundtruth.txt"
    rows, last = [], None
    for line in truth.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        row = line.split()
        if last is not None and float(row[0]) <= last:
            continue
        rows.append(row)
        last = float(row[0])
    times = np.array([float(r[0]) for r in rows])
    poses = [interpolate_pose(ns / 1e9, rows, times, max_gap=.05) for ns in meta["acquired_ns"]]
    ref = next(i for i, (_, ok) in enumerate(poses) if ok)
    d0, t0 = depth_m_fn(ref), poses[ref][0]
    anchors = []
    for v, u in ((240, 320), (180, 160), (300, 480), (360, 320)):
        z = float(d0[v, u])
        if np.isfinite(z) and 1. <= z <= 4.:
            cam = np.linalg.inv(K) @ np.array([u, v, 1.]) * z
            for kind, scale in (("surface", 1.), ("air", (z - .6) / z)):
                anchors.append({"kind": kind, "pixel": [u, v], "z_m": z * scale,
                                "world": (t0[:3, :3] @ (cam * scale) + t0[:3, 3]).tolist()})
    up_axis = int(np.argmax(np.abs(t0[:3, :3] @ np.array([0., -1., 0.]))))
    labels = ["ANC%d" % i for i in range(len(anchors))]
    entries = [(labels[i], synthetic_bounds("bottle", a["world"], up_axis))
               for i, a in enumerate(anchors)]
    notes = [dict(label=labels[i], **a) for i, a in enumerate(anchors)]
    return poses, entries, labels, notes, ref, up_axis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", choices=("tum", "rec30"), required=True)
    ap.add_argument("--prep", required=True)
    ap.add_argument("--rec")
    ap.add_argument("--out", required=True)
    ap.add_argument("--scratch", required=True)
    ap.add_argument("--golden-stride", type=int, default=1)
    ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args()
    import cv2
    meta = json.loads(open(os.path.join(a.prep, "meta.json")).read())
    W0, H0, N = meta["width"], meta["height"], meta["frames"]
    if a.limit:
        N = min(N, a.limit)
    unit = np.float32(meta["depth_unit_m"])
    raw = np.memmap(os.path.join(a.prep, meta["depth_file"]), dtype="<u2", mode="r",
                    shape=(meta["frames"], H0, W0))
    K = np.asarray(meta["K"], np.float64)
    ik = np.array([float.fromhex(x) for x in meta["inverse_k_f32_hex"]], np.float32)
    g = meta["grid"]
    assert g["shape_xyz"] == [32, 32, 32]
    origin = np.asarray(g["origin_m"], np.float64)
    voxel = float(g["voxel_m"])
    cam_cell = [int(round((0. - origin[i]) / voxel)) for i in range(3)]  # (16,16,1)
    assert cam_cell == [16, 16, 1], cam_cell
    slice_y = 16
    lo_mm = int(round((origin[1] + slice_y * voxel) * 1000))
    hi_mm = int(round((origin[1] + (slice_y + 1) * voxel) * 1000))

    def depth_m(i):
        d = raw[i].astype(np.float32) * unit
        d[raw[i] == 0] = np.nan
        return d

    if a.dataset == "rec30":
        W, H, dy = 640, 400, 40
        frames, entries, labels, notes = rec30_entries(a.rec, depth_m(0), K)
        frame_ids = [int(f["seq"]) for f in frames]
        assert [int(f["acquired_ns"]) for f in frames] == meta["acquired_ns"]
        poses = [(np.eye(4), True)] * meta["frames"]
        extra = {"first_frame_color_index": frames[0]["color_index"], "objects": notes,
                 "pose": "identity (fixed camera), up_axis=1 (camera y)"}
    else:
        W, H, dy = 640, 480, 0
        poses, entries, labels, notes, ref, up_axis = tum_entries(meta, depth_m, K)
        frame_ids = list(range(meta["frames"]))
        extra = {"anchors": notes, "anchor_reference_frame": ref, "up_axis": up_axis,
                 "pose": "boxes: reference-trajectory poses; minimap: identity camera "
                         "(meta pose fixed_camera_identity) -- two frames on purpose"}
    regs_base = dict(width=W, height=H, depth_width=W0, depth_height=H0, depth_dx=0,
                     depth_dy=dy, frame_valid=1, pose_valid=1, age_ms=0, age_valid=0,
                     max_age_ms=0, replay=1, object_evidence=0, frame_id=0,
                     permit_present=0, permit_i=0, grid_valid=1, slice_y=slice_y,
                     slice_lo_mm=lo_mm, slice_hi_mm=hi_mm, cam_cell_x=cam_cell[0],
                     cam_cell_z=cam_cell[2], mini_x=504, mini_y=H - 136)

    os.makedirs(os.path.join(a.out, "png"), exist_ok=True)
    os.makedirs(a.scratch, exist_ok=True)
    lib = check_golden.build(a.scratch, "replay_ref")
    geom = build_geometry(a.scratch)
    tmp = tempfile.mkdtemp(dir=a.scratch)
    native = NativeObjectEvidence(compile_object_native(Path(tmp) / "object.so"))

    stats = {"frames": N, "status": {}, "verdicts": {}, "excluded": {}, "box_frames": 0,
             "golden_frames": 0, "golden_mismatch_frames": 0, "golden_mismatch_px": 0,
             "geometry_status_nonzero": 0, "nearest_mm": [], "per_frame": []}
    for i in range(N):
        dm = depth_m(i)
        mm = quantize_optical_z(dm)
        mm = mm[0] if isinstance(mm, tuple) else mm
        gst, flags = run_geometry(geom, dm, ik, origin, voxel)
        regs = dict(regs_base)
        regs["frame_id"] = frame_ids[i]
        if gst != 0:
            stats["geometry_status_nonzero"] += 1
            regs["grid_valid"] = 0
        rows = []
        pose, ok = poses[i]
        frame_rec = {"i": i, "frame_id": frame_ids[i], "boxes": []}
        if ok and entries:
            for (label, _), (verdict, evidence, det) in zip(entries, native.check_boxes(dm, K, pose, entries)):
                roi = det["roi_xyxy"]
                code = int(det["kernel_row"]["verdict_code"])
                if roi is None:
                    stats["excluded"][evidence] = stats["excluded"].get(evidence, 0) + 1
                    frame_rec["boxes"].append([label, evidence])
                    continue
                x0, y0, x1, y1 = roi
                rows.append([x0, y0, x1, y1, code, check_golden.lab(label)])
                stats["verdicts"][verdict] = stats["verdicts"].get(verdict, 0) + 1
                frame_rec["boxes"].append([label, verdict, evidence, list(roi)])
        else:
            frame_rec["no_pose"] = not ok
        stats["box_frames"] += bool(rows)
        case = dict(regs=check_golden.regs_of(regs), depth=mm.ravel(), grid=flags,
                    desc=check_golden.make_desc(rows), rows=rows)
        st, fb = check_golden.run_c(lib, case)
        stats["status"][st] = stats["status"].get(st, 0) + 1
        img = fb[:W * H].reshape(H, W)
        if not np.all(fb[W * H:] == check_golden.SENT):
            raise RuntimeError("frame %d: sentinel tail overwritten" % i)
        if a.golden_stride and i % a.golden_stride == 0:
            gs, gfb, info = golden.render(case["depth"], flags, case["desc"], case["regs"])
            nd = int((gfb != img).sum()) if gs != 1 else -1
            stats["golden_frames"] += 1
            if gs != st or nd != 0:
                stats["golden_mismatch_frames"] += 1
                stats["golden_mismatch_px"] += max(nd, 0)
            frame_rec["nearest_mm"] = info.get("nearest")
            frame_rec["mode"] = info.get("mode")
        stats["per_frame"].append(frame_rec)
        rgb = np.stack([(img >> 16) & 255, (img >> 8) & 255, img & 255], -1).astype(np.uint8)
        cv2.imwrite(os.path.join(a.out, "png", "%05d.png" % i), rgb[:, :, ::-1])
        if i % 100 == 0:
            print("frame %d/%d status=%d boxes=%d" % (i, N, st, len(rows)), flush=True)

    ns = meta["acquired_ns"][:N]
    fps = (len(ns) - 1) / ((ns[-1] - ns[0]) / 1e9)
    mp4 = os.path.join(a.out, "ar_overlay_%s.mp4" % a.dataset)
    subprocess.check_call(["/usr/bin/ffmpeg", "-y", "-loglevel", "error", "-framerate",
                           "%.3f" % fps, "-i", os.path.join(a.out, "png", "%05d.png"),
                           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", mp4])
    stats.update(dataset=a.dataset, prep=os.path.abspath(a.prep), fps=fps, mp4=mp4,
                 canvas=[W, H], depth_offset_y=dy, cam_cell=cam_cell, slice_y=slice_y,
                 slice_mm=[lo_mm, hi_mm], geometry_rev=GEOM_REV, box_labels=labels, **extra)
    stats["status"] = {str(k): v for k, v in stats["status"].items()}
    json.dump(stats, open(os.path.join(a.out, "result_%s.json" % a.dataset), "w"), indent=1)
    summary = {k: stats[k] for k in ("frames", "status", "verdicts", "excluded", "box_frames",
                                      "golden_frames", "golden_mismatch_frames",
                                      "golden_mismatch_px", "geometry_status_nonzero", "fps",
                                      "cam_cell", "slice_mm")}
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
