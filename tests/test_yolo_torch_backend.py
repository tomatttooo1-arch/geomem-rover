"""YOLO host GPU backend (line D2, 2026-10-14): YoloSegDetector(backend="torch") and
inference_service --pt.

  - argument wiring (pure, both interpreters): --pt needs --device cuda, is exclusive with
    --onnx/--engine, and selects backend "torch"; defaults (ssdlite / --onnx / --engine) unchanged.
  - GPU equivalence (skipped without CUDA torch + ultralytics + the model files; python3.8 here):
    torch vs ort on rec30 frames, line A criteria (same classes, IoU >= 0.9, |dscore| <= 0.02,
    no unmatched), with a positive control (detections present) and a known-different control
    (boxes shifted 20 px must fail the IoU criterion).

Run with CUDA_VISIBLE_DEVICES set to a GPU you hold the lock for, and PYTHONPATH including
ultralytics 8.3.40 (${FPGA_HOME}/dev1/.t/ya/ul8340) for the GPU case.
"""
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
TESTS = Path(__file__).resolve().parent
if str(TESTS) not in sys.path:
    sys.path.insert(0, str(TESTS))

from perception.fpga_replay.core import Frame  # noqa: E402
from perception.fpga_replay import yolo_detector as yd  # noqa: E402
from host import inference_service as svc  # noqa: E402
import test_yolo_detector as _tyd  # noqa: E402  (module, so its TestCases are not re-collected here)

PT = "${FPGA_SHARED}/models/20260914-dev3-m3-single-image/yolov8n-seg.pt"
ONNX = "${FPGA_SRC}/runs/yolo-runner-20261014-results/export/yolov8n-seg-640.onnx"
REC = "${FPGA_SHARED}/artifacts/20260927-dev1-rec30/rec/color"


def _gpu_ready():
    try:
        import torch
        import onnxruntime  # noqa: F401
        import cv2  # noqa: F401
        import ultralytics  # noqa: F401
    except ImportError as error:
        return "missing module: %s" % error
    try:  # the torch backend loads through 8.3.x API; newer ultralytics (8.4) dropped it
        from ultralytics.nn.tasks import attempt_load_one_weight  # noqa: F401
    except ImportError:
        return ("ultralytics %s lacks attempt_load_one_weight (the backend needs the pinned "
                "8.3.40 first on PYTHONPATH, tools/sim_loop/run_sim_loop.sh YOLO_UL_PATH)"
                % getattr(ultralytics, "__version__", "?"))
    if not torch.cuda.is_available():
        return "no CUDA device visible"
    missing = [p for p in (PT, ONNX, REC) if not os.path.exists(p)]
    if missing:
        return "missing files: %s" % missing
    return None


GPU_SKIP = _gpu_ready()


def args(*extra):
    return _tyd.PureHelpers.service_args(*extra)


class TorchWiring(unittest.TestCase):
    def test_pt_selects_torch_backend_on_cuda(self):
        made = {}

        class Fake(object):
            def __init__(self, model, backend, **k):
                made["call"] = (model, backend, k)
        with mock.patch.object(yd, "YoloSegDetector", Fake):
            svc.build_detector(args("--detector", "yolo", "--pt", "m.pt", "--device", "cuda"))
        model, backend, kwargs = made["call"]
        self.assertEqual((model, backend), ("m.pt", "torch"))
        self.assertEqual((kwargs["conf"], kwargs["iou"], kwargs["classes"]), (0.25, 0.7, yd.default_classes()))

    def test_existing_selections_unchanged(self):
        made = []

        class Fake(object):
            def __init__(self, model, backend, **k):
                made.append((model, backend))
        with mock.patch.object(yd, "YoloSegDetector", Fake):
            svc.build_detector(args("--detector", "yolo", "--onnx", "a.onnx"))
            svc.build_detector(args("--detector", "yolo", "--engine", "e.engine", "--device", "cuda"))
        self.assertEqual(made, [("a.onnx", "ort"), ("e.engine", "trt")])
        self.assertEqual(args().detector, "ssdlite")
        self.assertIsNone(args().pt)

    def test_refused_combinations(self):
        for extra in (["--detector", "yolo", "--pt", "m.pt"],                               # cpu
                      ["--detector", "yolo", "--pt", "m.pt", "--onnx", "a", "--device", "cuda"],
                      ["--detector", "yolo", "--pt", "m.pt", "--engine", "e", "--device", "cuda"],
                      ["--models", "/m", "--pt", "m.pt"]):                                 # yolo flag on ssdlite
            with self.assertRaises(ValueError, msg=str(extra)):
                svc.build_detector(args(*extra))

    def test_unknown_backend_refused(self):
        with self.assertRaises(ValueError):
            yd.YoloSegDetector(__file__, backend="cuda")


def _iou(a, b):
    w = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    h = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = w * h
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def _verdict(ref, test):
    """line A pairing: per class greedy by IoU >= 0.5; pass = no unmatched (>= 0.27), IoU >= 0.9, dscore <= 0.02."""
    cand = sorted(((_iou(r.bbox, t.bbox), i, j) for i, r in enumerate(ref) for j, t in enumerate(test)
                   if r.label == t.label), key=lambda c: -c[0])
    used_r, used_t, pairs = set(), set(), []
    for v, i, j in cand:
        if v < 0.5 or i in used_r or j in used_t:
            continue
        used_r.add(i); used_t.add(j)
        pairs.append((v, abs(ref[i].score - test[j].score)))
    hard = [d for k, d in enumerate(ref) if k not in used_r and d.score >= 0.27] + \
           [d for k, d in enumerate(test) if k not in used_t and d.score >= 0.27]
    ok = not hard and all(v >= 0.9 and ds <= 0.02 for v, ds in pairs)
    return ok, len(pairs)


@unittest.skipIf(GPU_SKIP is not None, "GPU equivalence skipped: %s" % GPU_SKIP)
class TorchGpuEquivalence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import cv2
        cls.ort = yd.YoloSegDetector(ONNX, backend="ort", classes=None)
        cls.gpu = yd.YoloSegDetector(PT, backend="torch", classes=None)
        names = sorted(os.listdir(REC))[0:310:8][:6]
        cls.frames = []
        for i, name in enumerate(names):
            rgb = np.ascontiguousarray(cv2.imread(os.path.join(REC, name), cv2.IMREAD_COLOR)[:, :, ::-1])
            h, w = rgb.shape[:2]
            cls.frames.append(Frame(frame_id=name, timestamp=float(i), rgb=rgb, depth=np.ones((h, w), np.float32),
                                    K=np.array([[500., 0, w / 2], [0, 500., h / 2], [0, 0, 1]]),
                                    T_world_camera=np.eye(4)))

    def test_execution_record(self):
        ex = self.gpu.execution
        self.assertEqual((ex["device"], ex["backend"], ex["precision"], ex["tf32"]), ("cuda", "torch", "fp32", False))
        self.assertEqual(self.gpu.source, "yolov8n_seg_torch_fp32")
        self.assertEqual(len(ex["model_sha256"]), 64)

    def test_matches_ort_with_positive_and_known_different_controls(self):
        total = 0
        for fr in self.frames:
            ref, test = self.ort(fr), self.gpu(fr)
            ok, n = _verdict(ref, test)
            self.assertTrue(ok, fr.frame_id)
            total += n
            if test:   # known-different control: a 20 px shift must break the comparator
                shifted = [d.__class__(d.label, d.score, (d.bbox[0] + 20, d.bbox[1], d.bbox[2] + 20, d.bbox[3]),
                                       d.source, d.frame_id, d.timestamp) for d in test]
                self.assertFalse(_verdict(ref, shifted)[0], fr.frame_id)
        self.assertGreaterEqual(total, 6)       # positive control: rec30 has bottles in every sampled frame

    def test_fp16_refused(self):
        with self.assertRaises(ValueError):
            yd.YoloSegDetector(PT, backend="torch", precision="fp16")


if __name__ == "__main__":
    unittest.main()
