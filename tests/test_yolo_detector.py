"""YOLOv8n-seg runner backend (perception/fpga_replay/yolo_detector.py) and its wiring.

Host-only: no TensorRT, no board. What is executed here:
  - the numpy post-processing against the board's own nano_detector.Detector.postprocess
    (run on CPU torch with the TensorRT buffers replaced by CPU tensors) on identical raw heads;
  - the onnxruntime backend end to end on a tiny constant-output ONNX graph (known answers:
    letterbox mapping, conf, class-aware NMS, D-036 whitelist, strict Detection construction);
  - inference_service --detector selection and the InferenceClient expected_detector check
    over a real Unix socket;
  - ros_runtime detector_selection / runtime_info, and source checks of the ROS wiring.
Skips (stated in the skip reason): onnxruntime/onnx absent (python3.8 here), torchvision absent.
Not executed: the TensorRT backend (board only).
"""
import argparse
import contextlib
from dataclasses import asdict, fields
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.fpga_replay.core import Detection, Frame  # noqa: E402
from perception.fpga_replay import yolo_detector as yd  # noqa: E402
from host import inference_service as svc  # noqa: E402

try:
    import onnxruntime  # noqa: F401
    import onnx  # noqa: F401
    HAVE_ORT = True
except ImportError:
    HAVE_ORT = False
try:
    import torch
    import torchvision  # noqa: F401
    HAVE_TV = True
except ImportError:
    HAVE_TV = False

COCO = yd.coco_names()
W, H, S = 640, 360, 640          # rec30 frame size; letterbox r = 1, top pad 140


def frame(fid="rgb-1", ts=1.0, w=W, h=H):
    return Frame(frame_id=fid, timestamp=ts, rgb=np.zeros((h, w, 3), np.uint8),
                 depth=np.ones((h, w), np.float32), K=np.array([[500., 0, w / 2], [0, 500., h / 2], [0, 0, 1]]),
                 T_world_camera=np.eye(4))


def head(rows, n=None, nc=80):
    """(1, 4+nc+32, N) detection head; rows = [(cx, cy, w, h, cls, score)] in network pixels."""
    n = n or len(rows)
    out = np.zeros((1, 4 + nc + 32, n), np.float32)
    for j, (cx, cy, bw, bh, cls, score) in enumerate(rows):
        out[0, :4, j] = (cx, cy, bw, bh)
        out[0, 4 + cls, j] = score
        out[0, 4 + (cls + 1) % nc, j] = score / 3       # a lower runner-up class
    return out


# known-answer head (network pixels; frame y = net y - 140)
ROWS = [
    (100, 300, 40, 100, 39, 0.90),    # bottle            -> kept
    (102, 301, 40, 100, 39, 0.80),    # bottle, IoU>0.7   -> suppressed by NMS
    (101, 300, 40, 100, 41, 0.70),    # cup, same box     -> kept (class-aware NMS)
    (400, 250, 200, 150, 63, 0.95),   # laptop            -> kept by NMS, dropped by the D-036 whitelist
    (300, 320, 30, 60, 67, 0.60),     # cell phone        -> kept
    (500, 400, 50, 50, 24, 0.24),     # backpack below conf 0.25 -> dropped
    (550, 200, 60, 80, 24, 0.50),     # backpack          -> kept
]


class _CpuTorch(object):
    """torch with cuda.stream() as a no-op context: the board function runs on CPU tensors
    without initialising CUDA (a CUDA context on a busy host GPU can take minutes)."""
    cuda = SimpleNamespace(stream=lambda stream: contextlib.nullcontext())

    def __getattr__(self, name):
        return getattr(torch, name)


def board_postprocess(raw, lb, conf=0.25, iou=0.7, max_det=100):
    """The board's Detector.postprocess, executed on CPU torch (TensorRT buffers -> CPU tensors)."""
    nd = yd._board_module()
    det = nd.Detector.__new__(nd.Detector)
    det.torch, det.tv = _CpuTorch(), torchvision
    det.m = SimpleNamespace(buf={"output0": torch.from_numpy(raw.copy()),
                                 "output1": torch.zeros((1, 32, S // 4, S // 4))}, stream=None)
    det.lb, det.conf, det.iou, det.max_det = lb, conf, iou, max_det
    det.out_det, det.out_proto, det.proto_stride = "output0", "output1", 4.0
    return nd.Detector.postprocess(det, masks=False)


def constant_onnx(path, raw, protos_nan=False):
    """Tiny ONNX: images (1,3,S,S) -> output0 = raw + 0*mean(images), output1 = zeros protos."""
    from onnx import TensorProto, helper, numpy_helper
    zero = numpy_helper.from_array(np.zeros((), np.float32), "zero")
    const = numpy_helper.from_array(raw.astype(np.float32), "const")
    protos = numpy_helper.from_array(np.zeros((1, 32, S // 4, S // 4), np.float32), "protos")
    nodes = [helper.make_node("ReduceMean", ["images"], ["m"], keepdims=0),
             helper.make_node("Mul", ["m", "zero"], ["z"]),
             helper.make_node("Add", ["const", "z"], ["output0"]),
             helper.make_node("Add", ["protos", "z"], ["output1"])]
    graph = helper.make_graph(
        nodes, "const_yolo", [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, S, S])],
        [helper.make_tensor_value_info("output0", TensorProto.FLOAT, list(raw.shape)),
         helper.make_tensor_value_info("output1", TensorProto.FLOAT, [1, 32, S // 4, S // 4])],
        [zero, const, protos])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 8
    onnx.save(model, path)
    return path


class PureHelpers(unittest.TestCase):
    def test_module_import_is_lazy(self):
        code = ("import sys; import perception.fpga_replay.yolo_detector as y; "
                "bad = [m for m in ('tensorrt', 'onnxruntime', 'torch', 'cv2') if m in sys.modules]; "
                "print(','.join(bad)); sys.exit(1 if bad else 0)")
        r = subprocess.run([sys.executable, "-c", code], cwd=str(ROOT), env=dict(os.environ, PYTHONPATH=str(ROOT)),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_d036_default_and_class_parsing(self):
        self.assertEqual(yd.D036_CLASSES, ("bottle", "cup", "backpack", "cell phone"))
        self.assertTrue(all(c in COCO for c in yd.D036_CLASSES))
        self.assertEqual(yd.parse_classes("bottle,cup,backpack,cell phone"), yd.D036_CLASSES)
        self.assertIsNone(yd.parse_classes("all"))
        for bad in ("bottle,phone", "", " , "):
            with self.assertRaises(ValueError):
                yd.parse_classes(bad)
        # 2026-10-27: nine venue classes suppressed by default; explicit exclusions replace them. 'all'
        # stays "no whitelist" (80)
        dflt = yd.default_classes()
        self.assertEqual(len(dflt), 15)   # owner 10-27 evening: umbrella off too
        self.assertNotIn("person", dflt)
        self.assertNotIn("tv", dflt)
        self.assertEqual(dflt, tuple(c for c in yd.VENUE_CLASSES if c not in yd.DEFAULT_EXCLUDE_CLASSES))
        self.assertEqual(list(dflt), [c for c in COCO if c in yd.VENUE_CLASSES and c not in yd.DEFAULT_EXCLUDE_CLASSES])
        self.assertEqual(set(yd.DEFAULT_EXCLUDE_CLASSES), {
            "handbag", "wine glass", "sports ball", "chair", "potted plant", "dining table",
            "clock", "vase", "teddy bear", "umbrella"})
        self.assertEqual(yd.parse_classes(None), dflt)
        self.assertEqual(yd.parse_classes(None, ("cup",)), tuple(c for c in yd.VENUE_CLASSES if c != "cup"))
        self.assertEqual(yd.parse_exclude(""), ())
        self.assertEqual(yd.parse_exclude(" , "), ())
        self.assertEqual(yd.parse_exclude("person, cup"), ("person", "cup"))
        with self.assertRaises(ValueError):
            yd.parse_exclude("phone")
        with self.assertRaises(ValueError):
            yd.default_classes(("phone",))
        # the service passes nothing by default: the detector side resolves the D-044 set
        args = self.service_args()
        self.assertIsNone(args.classes)
        self.assertIsNone(args.exclude_classes)

    def test_service_class_options_reach_the_detector(self):
        def built(*extra):
            made = {}

            class Fake(object):
                def __init__(self, model, backend, **k):
                    made["classes"] = k["classes"]
            with mock.patch.object(yd, "YoloSegDetector", Fake):
                svc.build_detector(self.service_args("--detector", "yolo", "--onnx", "m.onnx", *extra))
            return made["classes"]

        def floors(*extra):
            made = {}

            class Fake(object):
                def __init__(self, model, backend, **k):
                    made["floors"] = k["class_min_scores"]
            with mock.patch.object(yd, "YoloSegDetector", Fake):
                svc.build_detector(self.service_args("--detector", "yolo", "--onnx", "m.onnx", *extra))
            return made["floors"]
        self.assertIsNone(floors())                                     # detector applies its default
        self.assertEqual(floors("--class-min-score", ""), {})
        self.assertEqual(floors("--class-min-score", "backpack=0.6"), {"backpack": 0.6})
        with self.assertRaises(ValueError):
            floors("--class-min-score", "backpack=2")
        venue = yd.VENUE_CLASSES
        self.assertEqual(built(), yd.default_classes())
        self.assertEqual(built("--exclude-classes", ""), venue)                       # nothing removed
        self.assertEqual(built("--exclude-classes", "person,cup"), tuple(c for c in venue if c != "cup"))
        self.assertIsNone(built("--classes", "all"))                                    # all 80, no whitelist
        self.assertEqual(built("--classes", "bottle,cup,backpack,cell phone"), yd.D036_CLASSES)
        for extra in (["--exclude-classes", "phone"], ["--classes", "cup", "--exclude-classes", "person"]):
            with self.assertRaises(ValueError, msg=str(extra)):
                built(*extra)

    @staticmethod
    def service_args(*extra):
        # rebuild main()'s parser by running main with parse_args intercepted
        captured = {}

        def fake_parse(self, *a, **k):
            captured["parser"] = self
            raise SystemExit(0)
        with mock.patch.object(argparse.ArgumentParser, "parse_args", fake_parse):
            try:
                svc.main()
            except SystemExit:
                pass
        return captured["parser"].parse_args(["--socket", "s", "--device", "cpu"] + list(extra))

    def test_postprocess_known_answer(self):
        lb = yd._board_module().Letterbox(W, H, S)
        dets = yd.postprocess_raw(head(ROWS), lb, .25, .7, 100, COCO)
        got = [(d["name"], d["score"]) for d in dets]
        self.assertEqual(got, [("laptop", .95), ("bottle", .9), ("cup", .7), ("cell phone", .6), ("backpack", .5)])
        bottle = dets[1]["box"]
        self.assertEqual(bottle, [80.0, 110.0, 120.0, 210.0])   # (100±20, 300-140±50)
        self.assertEqual(dets[0]["box"], [300.0, 35.0, 500.0, 185.0])
        self.assertEqual(yd.postprocess_raw(head(ROWS), lb, .25, .7, 2, COCO)[1]["name"], "bottle")  # max_det
        self.assertEqual(yd.postprocess_raw(head([]  , n=4), lb, .25, .7, 100, COCO), [])

    @unittest.skipUnless(HAVE_TV, "torch/torchvision not installed")
    def test_numpy_postprocess_matches_board_function(self):
        nd = yd._board_module()
        rng = np.random.default_rng(7)
        cases = [(head(ROWS), nd.Letterbox(W, H, S))]
        for size in ((1280, 720), (640, 480), (640, 360)):
            n = 600
            raw = np.zeros((1, 116, n), np.float32)
            raw[0, 0] = rng.uniform(0, S, n); raw[0, 1] = rng.uniform(100, 540, n)
            raw[0, 2] = rng.uniform(8, 200, n); raw[0, 3] = rng.uniform(8, 200, n)
            raw[0, 4:84] = rng.uniform(0, .2, (80, n))
            hot = rng.choice(n, 120, replace=False)
            raw[0, 4 + rng.integers(0, 6, 120), hot] = rng.uniform(.2, .99, 120)   # few classes -> NMS work
            raw[0, 84:] = rng.normal(size=(32, n))
            cases.append((raw, nd.Letterbox(size[0], size[1], S)))
        for raw, lb in cases:
            ours = yd.postprocess_raw(raw, lb, .25, .7, 100, COCO)
            board = board_postprocess(raw, lb)
            self.assertEqual(len(ours), len(board))
            self.assertGreater(len(board), 2)
            for a, b in zip(ours, board):
                self.assertEqual((a["cls"], a["name"]), (b["cls"], b["name"]))
                self.assertLessEqual(abs(a["score"] - b["score"]), 1e-4)
                self.assertLessEqual(max(abs(x - y) for x, y in zip(a["box"], b["box"])), 0.1 + 1e-9)


@unittest.skipUnless(HAVE_ORT, "onnxruntime/onnx not installed in this interpreter (host python3.8)")
class OrtBackend(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.model = constant_onnx(os.path.join(self.tmp.name, "c.onnx"), head(ROWS, n=16))

    def tearDown(self):
        self.tmp.cleanup()

    def test_interface_whitelist_and_strict_construction(self):
        det = yd.YoloSegDetector(self.model, backend="ort")
        self.assertEqual(det.detector, "yolo")
        self.assertEqual(det.execution["device"], "cpu")
        self.assertEqual((det.execution["backend"], det.execution["precision"]), ("ort", "fp32"))
        self.assertEqual(det.execution["classes"], list(yd.default_classes()))   # D-044 default
        self.assertEqual(len(det.execution["model_sha256"]), 64)
        f = frame("rgb-9", 9.5)
        out = det(f)
        self.assertEqual([d.label for d in out], ["laptop", "bottle", "cup", "cell phone", "backpack"])
        d036 = yd.YoloSegDetector(self.model, "ort", classes=yd.D036_CLASSES)(frame("rgb-9", 9.5))
        self.assertEqual([d.label for d in d036], ["bottle", "cup", "cell phone", "backpack"])  # no laptop
        for d in out:
            self.assertIsInstance(d, Detection)
            self.assertEqual((d.frame_id, d.timestamp, d.source), ("rgb-9", 9.5, "yolov8n_seg_ort_fp32"))
            self.assertTrue(np.isfinite(d.bbox).all() and np.isfinite(d.score))
            item = json.loads(json.dumps(asdict(d)))
            self.assertEqual(set(item), {f.name for f in fields(Detection)})   # nothing extra (no mask)
            self.assertEqual(asdict(Detection(**item)), item)
        # whitelist off -> laptop present; custom whitelist honoured
        self.assertIn("laptop", [d.label for d in yd.YoloSegDetector(self.model, "ort", classes=None)(f)])
        self.assertEqual([d.label for d in yd.YoloSegDetector(self.model, "ort", classes=("cup",))(f)], ["cup"])
        # frame size change keeps the network size and remaps boxes
        big = det(frame("rgb-10", 10.0, 1280, 720))
        self.assertEqual(big[1].label, "bottle")                         # after laptop (0.95)
        self.assertEqual(big[1].bbox, (160.0, 220.0, 240.0, 420.0))   # bottle: r = 0.5, top pad 140

    def test_default_drops_person_and_all_keeps_it(self):
        rows = [(100, 300, 40, 100, 0, 0.90), (400, 250, 80, 80, 56, 0.80)]     # person, chair
        model = constant_onnx(os.path.join(self.tmp.name, "p.onnx"), head(rows, n=8))
        f = frame("rgb-p", 1.0)
        self.assertEqual([d.label for d in yd.YoloSegDetector(model, "ort")(f)], [])
        self.assertEqual([d.label for d in yd.YoloSegDetector(model, "ort", classes=yd.default_classes(()))(f)],
                         ["chair"])                                              # person is not a venue class
        self.assertEqual([d.label for d in yd.YoloSegDetector(model, "ort", classes=None)(f)], ["person", "chair"])

    def test_class_min_score_floors(self):
        rows = [(100, 300, 40, 100, 24, 0.44), (400, 250, 80, 80, 24, 0.46), (300, 200, 60, 60, 39, 0.30)]
        model = constant_onnx(os.path.join(self.tmp.name, "b.onnx"), head(rows, n=8))   # backpack x2, bottle
        f = frame("rgb-b", 1.0)
        det = yd.YoloSegDetector(model, "ort")                                 # default backpack floor 0.45
        self.assertEqual([(d.label, d.score) for d in det(f)], [("backpack", 0.46), ("bottle", 0.3)])
        self.assertEqual(det.execution["class_min_scores"], {"backpack": 0.45})
        self.assertEqual(len(yd.YoloSegDetector(model, "ort", class_min_scores={})(f)), 3)      # no floor
        self.assertEqual([d.label for d in yd.YoloSegDetector(model, "ort", class_min_scores={"bottle": 0.5})(f)],
                         ["backpack", "backpack"])                             # override replaces the default
        self.assertEqual(yd.parse_class_min_scores(""), {})
        self.assertEqual(yd.parse_class_min_scores(" backpack=0.5 , cup=0.3"), {"backpack": 0.5, "cup": 0.3})
        for bad in ("backpack", "backpack=1", "backpack=-0.1", "backpack=x", "bag=0.5"):
            with self.assertRaises(ValueError, msg=bad):
                yd.parse_class_min_scores(bad)
        with self.assertRaises(ValueError):
            yd.YoloSegDetector(model, "ort", class_min_scores={"bag": 0.5})

    def test_default_drops_cancelled_venue_classes(self):
        rows = [(100, 300, 40, 100, 4, 0.90), (400, 250, 80, 80, 28, 0.80), (300, 200, 60, 60, 41, 0.70)]
        model = constant_onnx(os.path.join(self.tmp.name, "v.onnx"), head(rows, n=8))   # airplane, suitcase, cup
        f = frame("rgb-v", 1.0)
        self.assertEqual([d.label for d in yd.YoloSegDetector(model, "ort")(f)], ["cup"])
        self.assertEqual([d.label for d in yd.YoloSegDetector(model, "ort", classes=None)(f)],
                         ["airplane", "suitcase", "cup"])

    def test_rejects_bad_inputs_and_nonfinite_output(self):
        with self.assertRaises(ValueError):
            yd.YoloSegDetector(self.model, backend="cuda")
        with self.assertRaises(ValueError):
            yd.YoloSegDetector(self.model, backend="ort", classes=("phone",))
        with self.assertRaises(FileNotFoundError):
            yd.YoloSegDetector(self.model + ".missing", backend="ort")
        bad = head(ROWS, n=16); bad[0, 0, 3] = np.nan
        det = yd.YoloSegDetector(constant_onnx(os.path.join(self.tmp.name, "nan.onnx"), bad), "ort")
        with self.assertRaisesRegex(RuntimeError, "non-finite"):
            det(frame())

    def test_service_build_and_socket_roundtrip_with_detector_check(self):
        args = PureHelpers.service_args("--detector", "yolo", "--onnx", self.model)
        det = svc.build_detector(args)
        self.assertIsInstance(det, yd.YoloSegDetector)
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "ya.sock")
            server = start_server(path, det, "cpu", 3)
            out = svc.InferenceClient(path, timeout=5, expected_device="cpu", expected_detector="yolo").detect(frame("a", 1.0))
            self.assertEqual([d.label for d in out], ["laptop", "bottle", "cup", "cell phone", "backpack"])  # D-044
            with self.assertRaisesRegex(ValueError, "detector"):
                svc.InferenceClient(path, timeout=5, expected_detector="ssdlite").detect(frame("b", 2.0))
            # unchecked legacy client still accepts it
            self.assertEqual(len(svc.InferenceClient(path, timeout=5).detect(frame("c", 3.0))), 5)
            server.join(5)


def serve_n(path, detector, device, n, ready=None):
    """serve() for exactly n requests (same per-request code path: process_request). ``ready``
    (threading.Event) is set once the socket listens: the socket file already exists after
    bind(), so a client waiting only for the file can connect before listen() and be refused."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(path)
        listener.listen(1)
        if ready is not None:
            ready.set()
        for _ in range(n):
            conn, _ = listener.accept()
            with conn:
                try:
                    reply = svc.process_request(svc.receive_message(conn), detector, device)
                except Exception as error:
                    reply = {"ok": False, "error": type(error).__name__ + ": " + str(error)}
                svc.send_message(conn, json.dumps(reply, allow_nan=False).encode())


def start_server(path, detector, device, n, seconds=5.0):
    """serve_n on a daemon thread; returns once it listens (not merely once the file exists)."""
    ready = threading.Event()
    server = threading.Thread(target=serve_n, args=(path, detector, device, n, ready), daemon=True)
    server.start()
    if not ready.wait(seconds):
        raise TimeoutError(path)
    return server


class ServiceSelection(unittest.TestCase):
    def test_ssdlite_path_unchanged(self):
        args = PureHelpers.service_args("--models", "/m")
        self.assertEqual(args.detector, "ssdlite")
        made = {}

        class FakeSSD(object):
            def __init__(self, *a, **k):
                made["call"] = (a, k)
        with mock.patch("perception.fpga_replay.detector.SSDDetector", FakeSSD):
            det = svc.build_detector(args)
        self.assertEqual(det.detector, "ssdlite")
        self.assertEqual(made["call"], (("/m",), dict(score=0.4, device="cpu", memory_format="contiguous",
                                                      cuda_math="fp32", augmentation="none")))

    def test_argument_combinations_refused(self):
        for extra in ([],                                              # ssdlite without --models
                      ["--models", "/m", "--onnx", "x.onnx"],          # yolo flags on ssdlite
                      ["--detector", "yolo"],                          # neither --onnx nor --engine
                      ["--detector", "yolo", "--onnx", "a", "--engine", "b"],
                      ["--detector", "yolo", "--engine", "e"],         # TensorRT on --device cpu
                      ["--detector", "yolo", "--onnx", "a", "--classes", "phone"]):
            with self.assertRaises(ValueError, msg=str(extra)):
                svc.build_detector(PureHelpers.service_args(*extra))


class ClientDetectorCheck(unittest.TestCase):
    def rpc(self, declared, expected):
        f = frame("rgb-3", 3.0)
        det = lambda fr: [Detection("bottle", .9, (1., 2., 3., 4.), "yolov8n_seg_ort_fp32", fr.frame_id, fr.timestamp)]
        if declared is not None:
            det = type("D", (), {"detector": declared, "__call__": lambda self, fr: [
                Detection("bottle", .9, (1., 2., 3., 4.), "yolov8n_seg_ort_fp32", fr.frame_id, fr.timestamp)]})()
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "ya.sock")
            server = start_server(path, det, "cpu", 1)
            try:
                return svc.InferenceClient(path, timeout=5, expected_device="cpu", expected_detector=expected).detect(f)
            finally:
                server.join(5)

    def test_match_mismatch_missing_and_unchecked(self):
        self.assertEqual(len(self.rpc("yolo", "yolo")), 1)
        self.assertEqual(len(self.rpc("ssdlite", None)), 1)
        self.assertEqual(len(self.rpc(None, None)), 1)
        with self.assertRaisesRegex(ValueError, "differs from requested detector"):
            self.rpc("ssdlite", "yolo")
        with self.assertRaisesRegex(ValueError, "differs from requested detector"):
            self.rpc(None, "ssdlite")          # an undeclared detector is not a match


class RuntimeWiring(unittest.TestCase):
    SRC = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")

    def test_detector_selection_and_runtime_info(self):
        from host import ros_runtime as rr
        self.assertEqual(rr.DETECTORS, ("ssdlite", "yolo"))
        base = dict(sensor_source="replay", pose_source="reference", inference_device="cpu")
        self.assertEqual(rr.detector_selection(SimpleNamespace(**base)), ("ssdlite", None))
        self.assertEqual(rr.detector_selection(SimpleNamespace(detector=None, **base)), ("ssdlite", None))
        self.assertEqual(rr.detector_selection(SimpleNamespace(detector="yolo", **base)), ("yolo", "yolo"))
        with self.assertRaises(ValueError):
            rr.detector_selection(SimpleNamespace(detector="rcnn", **base))
        body = rr.runtime_info_body(SimpleNamespace(detector="yolo", **base), "b", "cpu", 1, 200.0, version={})
        self.assertEqual((body["detector"], body["detector_checked"]), ("yolo", True))
        body = rr.runtime_info_body(SimpleNamespace(**base), "b", "cpu", 1, 200.0, version={})
        self.assertEqual((body["detector"], body["detector_checked"]), ("ssdlite", False))

    def test_source_wiring(self):
        self.assertRegex(self.SRC, r'add_argument\("--detector", choices=DETECTORS, default=None')
        self.assertRegex(self.SRC, r"InferenceClient\(args\.socket, timeout=10, expected_device=args\.inference_device,"
                                   r"\s+expected_detector=expected_detector\)")
        self.assertRegex(self.SRC, r'self\.record\("startup", pose_source=args\.pose_source,.{0,200}?\n.{0,120}?\n\s+detector=self\.detector_name, view_refinement')
        self.assertNotIn('detector="ssdlite"', self.SRC)


if __name__ == "__main__":
    unittest.main()
