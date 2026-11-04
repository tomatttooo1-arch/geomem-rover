"""Board service smoke (PREREG.md section 4, S1-S4): a running
`python3 -m host.inference_service --detector yolo --engine ... --device cuda` answered through
host.inference_service.InferenceClient (the client ros_runtime uses). Same shape as line A's
runs/yolo-runner-20261014-results/scripts/service_smoke.py, with expected_device="cuda".
Usage: python3 -m tools.yolo_trt_board.board_smoke <socket> <frames dir> <reference json> <engine sha256> <out json>
"""
import json
import math
import os
import sys

import cv2
import numpy as np

from host.inference_service import InferenceClient
from perception.fpga_replay.core import Frame
from perception.fpga_replay.yolo_detector import VENUE_CLASSES


def main():
    sock, frames_dir, ref_path, engine_sha, out_path = sys.argv[1:6]
    ref = json.load(open(ref_path))
    names = [n for n in ref["frames"] if any(d["name"] == "bottle" for d in ref["detections"][n])][:5]
    assert len(names) == 5, names
    client = InferenceClient(sock, timeout=30, expected_device="cuda", expected_detector="yolo")
    rows, total, ok, meta, fr = [], 0, True, {}, None
    s1_errors = []
    for i, name in enumerate(names):
        rgb = np.ascontiguousarray(cv2.imread(os.path.join(frames_dir, name))[:, :, ::-1])
        h, w = rgb.shape[:2]
        fr = Frame(frame_id="smoke-" + name, timestamp=100.0 + i, rgb=rgb, depth=np.ones((h, w), np.float32),
                   K=np.array([[500., 0, w / 2], [0, 500., h / 2], [0, 0, 1]]), T_world_camera=np.eye(4))
        try:
            dets = client.detect(fr)                       # strict Detection(**item) inside
        except Exception as error:
            s1_errors.append({"frame": name, "error": type(error).__name__ + ": " + str(error)})
            continue
        meta = client.last_metadata
        finite = all(math.isfinite(d.score) and all(math.isfinite(v) for v in d.bbox) for d in dets)
        whitelisted = all(d.label in VENUE_CLASSES for d in dets)   # D-044 default set (venue classes)
        ok = ok and finite and whitelisted
        total += len(dets)
        rows.append({"frame": name, "n": len(dets), "labels": [d.label for d in dets],
                     "scores": [d.score for d in dets], "boxes": [list(d.bbox) for d in dets],
                     "sources": sorted({d.source for d in dets}),
                     "finite": finite, "whitelisted": whitelisted, "detector": meta.get("detector"),
                     "device": meta.get("device"), "inference_seconds": meta.get("inference_seconds")})
    s1 = len(rows) == 5 and not s1_errors
    s2 = total >= 1 and ok
    try:
        InferenceClient(sock, timeout=30, expected_device="cuda", expected_detector="ssdlite").detect(fr)
        s3, s3_error = False, None
    except ValueError as error:
        s3, s3_error = True, str(error)
    execution = meta.get("execution") or {}
    s4 = {"backend": execution.get("backend"), "precision": execution.get("precision"),
          "model_sha256": execution.get("model_sha256"),
          "backend_is_trt": execution.get("backend") == "trt", "precision_label_is_fp16": execution.get("precision") == "fp16",
          "model_sha_is_engine": execution.get("model_sha256") == engine_sha,
          "note": "precision is a label written by YoloSegDetector, not read from the engine"}
    result = {"prereg": "PREREG.md section 4", "frames": rows, "S1_errors": s1_errors, "total_detections": total,
              "S1_all_ok_strict": s1, "S2_nonempty_finite_whitelist": s2,
              "S3_wrong_expected_detector_refused": s3, "S3_error": s3_error, "S4_record": s4,
              "execution": execution, "PASS": s1 and s2 and s3}
    with open(out_path, "w") as f:
        json.dump(result, f, indent=1)
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
