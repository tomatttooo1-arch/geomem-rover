"""P3 fixed-camera chain: detector batch + same-frame depth -> M7 evidence -> M4 document.

Camera, GPU and threads live in ``nano_loop.py``; this module is pure CPU so the
live loop, the offline replay and the unit tests drive exactly the same code.
It only composes existing consumers: ``M7Pipeline`` (geometry cache, original-
frame semantic evidence), the CPU reference backends ``FrameNativeM7`` /
``NativeObjectEvidence`` (tomorrow's PL swap points) and ``build_m4_document``.

The pose is a written-down identity ("fixed camera"), not localization.
"""

from dataclasses import asdict
import hashlib
import json

import numpy as np

from host.m7_memory import build_m4_document
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)
from tools.nano_detector.nano_detector import rle_decode


CLOCK = "nano_host_realtime_sdk_receive"
LOCAL_FRAME = "gemini2_color_optical_fixed"
POSE_EPOCH = "fixed_camera_v1"
POSE_VERSION = "fixed_camera_identity_v1"
POSE_SOURCE = "fixed_camera_identity_not_localization"
MAP_ID = "p3_fixed_camera"
MAP_VERSION = "fixed-v1"
SOURCE_ID = "gemini2_nano_p3"
DET_SOURCE = "yolov8n_seg_640_fp16_trt_nano"
WATCH_MATCH_M = 0.04      # same physical object for the watch list (bottles sit ~8.5 cm apart; per-object scatter < 2 cm)
WATCH_ARM_OBS = 5         # support observations before an object is watched
WATCH_IOU = 0.3           # a same-label detection overlapping this much counts as "seen"
WATCH_HALF_DEPTH_M = 0.05


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def canonical_sha(value):
    return sha256_bytes(json.dumps(value, sort_keys=True, separators=(",", ":")).encode())


def calibration_record(serial, firmware, K, width, height, extra=None):
    """Calibration identity bound to the exact intrinsics used for depth back-projection."""
    K = np.asarray(K, dtype=np.float64)
    body = {"serial": serial, "firmware": firmware, "width": int(width), "height": int(height),
            "K": [[round(float(v), 6) for v in row] for row in K], "align": "hw_d2c_depth_to_color",
            "depth_units": "uint16_mm_zero_invalid", "optical_frame": LOCAL_FRAME}
    if extra:
        body.update(extra)
    tag = canonical_sha(body)[:8]
    body["calibration_id"] = "gemini2-%s-c%dx%d-hwd2c-fw%s-K%s" % (serial, width, height, firmware, tag)
    body["sha256"] = canonical_sha(body)
    return body


def depth_mm_to_m(depth_u16, invalid_as_free_m=None):
    """uint16 mm (0 = no return) -> float32 optical Z metres, NaN = missing.

    ``invalid_as_free_m`` exists only for the deliberately wrong replay
    variant (treat missing returns as far free space); the live loop never
    sets it.
    """
    depth = np.asarray(depth_u16)
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError("depth must be HxW uint16 millimetres")
    out = depth.astype(np.float32) * np.float32(0.001)
    out[depth == 0] = np.nan if invalid_as_free_m is None else np.float32(invalid_as_free_m)
    return out


def default_grid():
    # 1.6 m cube around the fixed camera, 5 cm voxels (grid capacity is 32^3). The frame kernel
    # requires the camera origin inside the grid, so z starts just behind the camera.
    return GridSpec((-0.8, -0.8, -0.05), 0.05, (32, 32, 32))


def mask_full(det, letterbox, width, height):
    """Detector prototype-grid crop (P2 JSON) -> full original-resolution bool mask."""
    m = det.get("mask")
    if m is None:
        return None
    x0, y0 = m["box_cells"][0], m["box_cells"][1]
    cells = rle_decode(m["rle"], m["shape"])
    s = float(m["stride_px"])
    ox0 = int(round((x0 * (s * letterbox.r) - letterbox.left) / letterbox.r))
    oy0 = int(round((y0 * (s * letterbox.r) - letterbox.top) / letterbox.r))
    ow, oh = int(round(cells.shape[1] * s)), int(round(cells.shape[0] * s))
    out = np.zeros((height, width), dtype=bool)
    if ow <= 0 or oh <= 0:
        return out
    rows = np.minimum((np.arange(oh) / s).astype(int), cells.shape[0] - 1)
    cols = np.minimum((np.arange(ow) / s).astype(int), cells.shape[1] - 1)
    big = cells[rows][:, cols]
    X0, Y0 = max(ox0, 0), max(oy0, 0)
    X1, Y1 = min(ox0 + ow, width), min(oy0 + oh, height)
    if X1 > X0 and Y1 > Y0:
        out[Y0:Y1, X0:X1] = big[Y0 - oy0:Y1 - oy0, X0 - ox0:X1 - ox0]
    return out


def iou(a, b):
    ix = max(0., min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0., min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.


def bounds_from_box(bbox, position, K, half_depth=WATCH_HALF_DEPTH_M):
    """Camera-frame box: detector bbox back-projected at the object's median depth."""
    z = float(position[2])
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u0, v0, u1, v1 = bbox
    lo = [(u0 - cx) * z / fx, (v0 - cy) * z / fy, z - half_depth]
    hi = [(u1 - cx) * z / fx, (v1 - cy) * z / fy, z + half_depth]
    return np.array([lo, hi], dtype=np.float64)


class FixedCameraChain:
    """One stream of fixed-camera frames through the unchanged M7 consumers.

    Single-threaded by contract: every ``M7Pipeline`` call happens here.
    """

    def __init__(self, frame_backend, object_backend, calibration, run_name, *,
                 letterbox, prediction_set, width, height, grid=None):
        self.pipeline = M7Pipeline(frame_backend, grid or default_grid(),
                                   object_backend=object_backend)
        self.calibration = calibration
        self.K = np.asarray(calibration["K"], dtype=np.float64)
        self.run_name = run_name
        self.stream = "nano-loop-" + run_name
        self.letterbox = letterbox
        self.prediction_set = prediction_set
        self.width, self.height = width, height
        self.frames = {}          # color_index -> frame record (bounded)
        self.latest_index = None
        self.watch = []
        self.calibration_ref = {"path": "nano-loop://%s/calibration.json" % run_name,
                                "sha256": calibration["sha256"]}

    # ---- geometry ---------------------------------------------------------
    def identity(self, color_index, acquired_ns):
        return CaptureIdentity(self.stream, int(color_index), int(acquired_ns), CLOCK,
                               self.calibration["calibration_id"], LOCAL_FRAME, POSE_EPOCH)

    def ingest_frame(self, color_index, acquired_ns, depth_u16, now_ns, *, rgb_sha256,
                     invalid_as_free_m=None, keep=None):
        ident = self.identity(color_index, acquired_ns)
        depth_m = depth_mm_to_m(depth_u16, invalid_as_free_m)
        pose = PoseAtCapture(int(acquired_ns), CLOCK, LOCAL_FRAME, POSE_EPOCH, True, np.eye(4))
        geometry = self.pipeline.ingest_geometry(DepthFrame(ident, depth_m, self.K, pose),
                                                 now_ns=int(now_ns), now_clock_id=CLOCK)
        nav = geometry.navigation
        rec = {"identity": ident, "geometry": geometry, "rgb_sha256": rgb_sha256,
               "color_index": int(color_index), "keep": keep}
        self.frames[int(color_index)] = rec
        self.latest_index = int(color_index)
        while len(self.frames) > 8:
            self.frames.pop(min(self.frames))
        summary = {"valid": nav.valid, "reason": nav.reason,
                   "valid_depth_frac": round(float(np.isfinite(depth_m).mean()), 4)}
        if nav.valid:
            summary.update(nearest_m=round(geometry.display.payload["nearest_optical_z_m"], 4),
                           hit_voxels=nav.payload["hit_voxels"],
                           valid_grid_samples=nav.payload["valid_grid_samples"])
        return rec, summary

    # ---- semantic + memory document ---------------------------------------
    def _instances(self, color_index, dets, masks):
        out = []
        for i, d in enumerate(dets):
            mask = masks[i]
            out.append({"instance_id": str(i), "label": " ".join(d["name"].lower().split()),
                        "score": float(d["score"]), "source": DET_SOURCE,
                        "mask_content_sha256": None if mask is None else
                            sha256_bytes(mask.astype(np.uint8).tobytes()),
                        "evidence_refs": [{"path": "nano-loop://%s/%d/det/%d" % (self.run_name, color_index, i),
                                           "sha256": canonical_sha(d)}]})
        return out

    def consume_detections(self, color_index, dets, now_ns, *, watch_negatives=True):
        """Bind one complete detector batch to its original frame's cached depth.

        Returns a dict with per-instance evidence, removal-watch results and the
        M4 document (or the reason there is none).
        """
        rec = self.frames.get(int(color_index))
        result = {"color_index": int(color_index), "instances": [], "negatives": [],
                  "late": self.latest_index is not None and self.latest_index != int(color_index),
                  "frames_behind": None if self.latest_index is None else
                      (self.latest_index - int(color_index)) // 2,
                  "m4": None, "m4_status": None}
        if rec is None:
            result["m4_status"] = "rejected:frame_record_evicted"
            return result, None
        geometry = rec["geometry"]
        nav = geometry.navigation
        masks = [mask_full(d, self.letterbox, self.width, self.height) for d in dets]
        outputs = {}
        for i, d in enumerate(dets):
            row = {"id": str(i), "label": d["name"], "score": d["score"], "box": d["box"]}
            if not nav.valid:
                row.update(candidate="rejected", reason="geometry_invalid:" + nav.reason)
            else:
                out = self.pipeline.ingest_semantic(
                    SemanticPacket(rec["identity"], nav.frame_token, str(i), masks[i]),
                    now_ns=int(now_ns), now_clock_id=CLOCK)
                outputs[str(i)] = out
                p = out.payload
                if out.valid:
                    row.update(candidate=p["candidate"], reason=out.reason,
                               mask_pixels=p.get("mask_pixels"),
                               mask_valid_pixels=p.get("mask_valid_pixels"),
                               valid_fraction=p.get("valid_fraction"),
                               position_m=p.get("position_local_m"))
                else:
                    row.update(candidate="rejected", reason=out.reason)
            result["instances"].append(row)
        if nav.valid:
            for row in result["instances"]:
                if row["candidate"] == "support_present":
                    self._update_watch(row["label"], row["position_m"], row["box"])
            if watch_negatives:
                result["negatives"] = self._negatives(rec, dets, now_ns)
        # M4 document through the unchanged bridge; any revoked instance rejects the batch.
        capture = {"identity": asdict(rec["identity"]),
                   "acquisition_id": "%s:color:%d" % (self.stream, rec["color_index"]),
                   "source_id": SOURCE_ID, "image_sha256": rec["rgb_sha256"],
                   "time_source": "orbbec_sdk_system_timestamp_realtime",
                   "evidence_refs": [{"path": "nano-loop://%s/%d/rgb" % (self.run_name, rec["color_index"]),
                                      "sha256": rec["rgb_sha256"]}, self.calibration_ref]}
        instances = self._instances(rec["color_index"], dets, masks)
        detection = {"identity": asdict(rec["identity"]), "prediction_set": self.prediction_set,
                     "status": "ok", "error": None,
                     "expected_instance_ids": [x["instance_id"] for x in instances],
                     "instances": instances}
        anchor = {"map_id": MAP_ID, "map_version": MAP_VERSION, "world_frame": LOCAL_FRAME,
                  "pose_source": POSE_SOURCE, "source_kind": "fixed_camera",
                  "source_ref": self.calibration_ref, "local_frame_id": LOCAL_FRAME,
                  "pose_epoch": POSE_EPOCH, "pose_version": POSE_VERSION,
                  "T_map_local": np.eye(4).tolist(), "evidence_refs": [self.calibration_ref],
                  "calibration_id": self.calibration["calibration_id"]}
        try:
            document, audit = build_m4_document(
                self.pipeline, geometry, outputs if nav.valid else {}, capture=capture,
                detection=detection, anchor=anchor, now_ns=int(now_ns), clock_id=CLOCK)
        except ValueError as exc:
            result["m4_status"] = "rejected:" + str(exc)[:80]
            return result, None
        result["m4_status"] = audit["status"] + ":" + audit.get("reason", "")
        return result, document

    # ---- removal watch (explicit negative evidence; never edits memory) ----
    def _update_watch(self, label, position, box):
        p = np.asarray(position, dtype=np.float64)
        for w in self.watch:
            if w["label"] == label and np.linalg.norm(p - w["position"]) <= WATCH_MATCH_M:
                w.update(position=p, box=list(box), n=w["n"] + 1,
                         bounds=bounds_from_box(box, p, self.K))
                return
        if len(self.watch) < 16:
            self.watch.append({"key": "w%d" % len(self.watch), "label": label, "position": p,
                               "box": list(box), "n": 1, "bounds": bounds_from_box(box, p, self.K)})

    def _negatives(self, rec, dets, now_ns):
        nav = rec["geometry"].navigation
        rows = []
        for w in self.watch:
            if w["n"] < WATCH_ARM_OBS:
                continue
            if any(d["name"] == w["label"] and iou(d["box"], w["box"]) >= WATCH_IOU for d in dets):
                continue
            out = self.pipeline.ingest_semantic(
                SemanticPacket(rec["identity"], nav.frame_token, "watch:" + w["key"],
                               np.zeros((self.height, self.width), dtype=bool), w["bounds"]),
                now_ns=int(now_ns), now_clock_id=CLOCK)
            rows.append({"watch": w["key"], "label": w["label"],
                         "candidate": out.payload.get("candidate") if out.valid else "rejected",
                         "reason": out.reason})
        return rows
