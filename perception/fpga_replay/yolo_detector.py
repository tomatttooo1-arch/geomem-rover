"""YOLOv8n-seg recognition backend for the runner (host/inference_service.py --detector yolo).

Same contract as detector.SSDDetector: ``__call__(frame) -> list[Detection]`` plus the
``.detector`` / ``.execution`` attributes the inference service reports.

Two explicit backends, no automatic fallback:
  trt : board (Orin Nano). Reuses tools/nano_detector/nano_detector.py unchanged -- Letterbox,
        TRTModel, Detector.preprocess / Detector.postprocess, set_cuda_sync -- imported the same
        way as tools/nano_loop/nano_loop.py. tensorrt/torch/cv2 are imported only when this
        backend is constructed, so the module imports on a host without TensorRT.
  ort : host. onnxruntime CPU on an ONNX export of the same .pt, with numpy versions of the
        board's pre/post-processing (letterbox_input / postprocess_raw below). The geometry
        (Letterbox) and class names (COCO) are the board module's own objects; the numpy
        post-processing is checked against the board's Detector.postprocess on identical raw
        outputs in tests/test_yolo_detector.py.
  torch : host GPU (line D2, 2026-10-14). The same .pt loaded with ultralytics (8.3.40 on the
        host, importable via PYTHONPATH), fused, eval, fp32, forward on CUDA with TF32 off and
        cudnn.benchmark off. Pre/post-processing is the ort backend's (letterbox_input on the CPU,
        then upload; postprocess_raw on the downloaded (1, 116, N) head), so only the forward
        differs from ort. Equivalence vs ort: runs/sim-loop-d2-20261014-results (A1-A3 of line A).

Masks are not passed on (Detection(**item) is a strict constructor on the client side).
The class whitelist is applied after class-aware NMS (identical to filtering before it except
for the max_det cap, which is the board's order of operations).
"""
import hashlib
import os

import numpy as np

from . import coco_zh as _coco_zh
from .household_names import NAMES as HOUSEHOLD_NAMES, NAMES33 as HOUSEHOLD_NAMES33
from .core import Detection

# D-036 item 1 (DECISIONS.md, 2026-09-27): the first-version four search classes. Superseded by
# D-044 (2026-10-21) as the default; kept as a named set (--classes "bottle,cup,backpack,cell phone"
# restores it; tools that check old runs compare against it).
D036_CLASSES = ("bottle", "cup", "backpack", "cell phone")
# D-044 revised (owner 2026-10-21, second ruling): the default whitelist is VENUE_CLASSES, the 25
# (26 until tv left on 10-22) COCO classes that can turn up at the defence venue (indoor classroom / lab); the controller picked
# the names. The 55 cancelled classes and the reason for each are coco_zh.CANCELLED_REASONS: person
# (people walk about: new tracks, position-invalid evidence where they stood, PL ROI rows, kiosk
# ghosts), traffic / street, animals, tie (worn by the judges), outdoor sport, suitcase / microwave /
# oven / refrigerator / airplane / car / fire hydrant (false tracks on cartons, table legs, floor tape
# and shoes in the 10-16 classroom recording), fork / knife, cooked food, couch / bed / toilet,
# kitchen and bathroom appliances, tv (10-22 owner: venue monitors / projector / laptop screens read as
# tv; not a find target). --classes all = all 80; --exclude-classes edits the venue set.
VENUE_CLASSES = _coco_zh.VENUE_CLASSES
# User 2026-10-27: suppress these outputs by default, retain model/classes and old memory.
# Owner 2026-10-27 evening: umbrella too (no umbrella at the defence venue): default combined 23 -> 22.
# --exclude-classes '' restores the full 25-class venue set (umbrella included).
DEFAULT_EXCLUDE_CLASSES = (
    "handbag", "wine glass", "sports ball", "chair", "potted plant", "dining table",
    "clock", "vase", "teddy bear", "umbrella", "remote",   # remote: owner 2026-10-28, no longer recognised
    "waste paper", "plastic packaging", "drink can",   # owner 2026-10-28: not recognised
    "earbud case", "medicine bottle",   # owner 2026-10-28: not recognised
    # glasses, medicine box: excluded 10-28, recognised again (owner 2026-10-29)
    # owner 2026-10-29 night: keep only banana, cup, laptop, backpack, medicine box, bottle
    "spoon", "bowl", "apple", "orange", "mouse", "keyboard", "cell phone", "book", "scissors",
    "key", "glasses",
    "medicine box",   # owner 2026-10-31: unstable on site, no longer recognised
)
DEFAULT_CLASSES = object()   # sentinel: YoloSegDetector(classes=DEFAULT_CLASSES) -> default_classes()
# 10-22: per-class score floors on top of --conf. backpack: 1108 detections in the 10-22 venue run had
# median 0.34 / p90 0.44 and 7 of 10 memory tracks were backpacks (one real); a provisional 0.45 until a
# lap with the real backpack shows where real and false tracks separate. --class-min-score overrides.
DEFAULT_CLASS_MIN_SCORES = {"backpack": 0.45}


def _board_module():
    # Import path as in tools/nano_loop/nano_loop.py:177 (repo root on sys.path).
    from tools.nano_detector import nano_detector
    return nano_detector


def coco_names():
    return list(_board_module().COCO)


def profile_names(profile):
    if profile == 'household9':
        return list(HOUSEHOLD_NAMES)
    if profile == 'coco-seg':
        return coco_names()
    if profile == 'combined':
        return list(dict.fromkeys(coco_names() + list(HOUSEHOLD_NAMES)))
    if profile == 'household33':
        return list(HOUSEHOLD_NAMES33)
    if profile == 'combined33':
        return list(dict.fromkeys(coco_names() + list(HOUSEHOLD_NAMES33)))
    raise ValueError('unknown YOLO profile: %s' % profile)


def _validated(names, coco):
    unknown = [n for n in names if n not in coco]
    if unknown:
        raise ValueError("unknown COCO class name(s): %s" % ", ".join(unknown))
    return tuple(names)


def default_classes(exclude=DEFAULT_EXCLUDE_CLASSES, coco=None):
    """D-044 default whitelist: VENUE_CLASSES minus ``exclude`` (validated), in board COCO order."""
    coco = list(coco) if coco is not None else coco_names()
    _validated(VENUE_CLASSES, coco)
    exclude = _validated(tuple(exclude), coco)
    return tuple(n for n in coco if n in VENUE_CLASSES and n not in exclude)


def combined_classes(exclude=DEFAULT_EXCLUDE_CLASSES, household=None):
    """Enabled COCO venue classes plus household labels, preserving semantic labels."""
    household = HOUSEHOLD_NAMES if household is None else household
    return tuple(n for n in dict.fromkeys(VENUE_CLASSES + tuple(household)) if n not in exclude)


def parse_exclude(text, profile='coco-seg'):
    """--exclude-classes: comma-separated COCO names; '' (or only commas/blanks) -> () = exclude nothing."""
    names = [part.strip() for part in (text or "").split(",") if part.strip()]
    return _validated(names, profile_names(profile))


def parse_classes(text, exclude=None, profile='coco-seg'):
    """--classes: None -> the D-044 default (VENUE_CLASSES minus ``exclude``); 'all' -> None (no
    whitelist: all 80 classes, person included); otherwise comma-separated COCO names, validated,
    taken literally (``exclude`` does not apply)."""
    if exclude is None:
        exclude = DEFAULT_EXCLUDE_CLASSES if profile in ('coco-seg', 'combined', 'combined33') else ()
    if text is None:
        if profile == 'combined33':
            _validated(exclude, profile_names(profile))
            return combined_classes(exclude, HOUSEHOLD_NAMES33)
        if profile == 'combined':
            _validated(exclude, profile_names(profile))
            return combined_classes(exclude)
        if profile != 'coco-seg':
            names = profile_names(profile)
            _validated(exclude, names)
            return tuple(n for n in names if n not in exclude)
        return default_classes(exclude)
    if text.strip().lower() == "all":
        return None
    names = [part.strip() for part in text.split(",") if part.strip()]
    if not names:
        raise ValueError("unknown COCO class name(s): <empty>")
    return _validated(names, profile_names(profile))


def parse_class_min_scores(text, profile='coco-seg'):
    """--class-min-score: 'name=score,...' (COCO names, score in [0,1)); '' -> {} (no per-class floor)."""
    out = {}
    for part in (text or "").split(","):
        if not part.strip():
            continue
        name, sep, value = part.partition("=")
        name = name.strip()
        try:
            score = float(value) if sep else float("nan")
        except ValueError:
            score = float("nan")
        if not 0.0 <= score < 1.0:
            raise ValueError("--class-min-score wants name=score with score in [0,1): %r" % part.strip())
        out[name] = score
    _validated(tuple(out), profile_names(profile))
    return out


def letterbox_input(rgb, lb, cv2):
    """numpy twin of nano_detector.Detector.preprocess: resize (INTER_LINEAR) into the
    letterbox, pad 114, /255, NCHW float32."""
    small = cv2.resize(rgb, (lb.nw, lb.nh), interpolation=cv2.INTER_LINEAR)
    x = np.full((1, 3, lb.S, lb.S), np.float32(114.0 / 255.0), dtype=np.float32)
    x[0, :, lb.top:lb.top + lb.nh, lb.left:lb.left + lb.nw] = \
        small.transpose(2, 0, 1).astype(np.float32) / np.float32(255.0)
    return x


def nms_numpy(boxes, scores, iou):
    """Greedy NMS, torchvision.ops.nms semantics (suppress IoU > iou, score-descending)."""
    order = np.argsort(-scores, kind="stable")
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    area = (x2 - x1) * (y2 - y1)
    keep = []
    while order.size:
        i = order[0]
        keep.append(i)
        rest = order[1:]
        w = np.clip(np.minimum(x2[i], x2[rest]) - np.maximum(x1[i], x1[rest]), 0, None)
        h = np.clip(np.minimum(y2[i], y2[rest]) - np.maximum(y1[i], y1[rest]), 0, None)
        inter = w * h
        overlap = inter / (area[i] + area[rest] - inter)
        order = rest[~(overlap > iou)]
    return np.asarray(keep, dtype=np.int64)


def batched_nms_numpy(boxes, scores, classes, iou):
    """torchvision.ops.batched_nms coordinate trick: offset boxes per class."""
    if boxes.shape[0] == 0:
        return np.zeros(0, dtype=np.int64)
    offset = classes.astype(boxes.dtype) * (boxes.max() + 1)
    return nms_numpy(boxes + offset[:, None], scores, iou)


def postprocess_raw(out_det, lb, conf, iou, max_det, names, mask_channels=32):
    """numpy twin of nano_detector.Detector.postprocess(masks=False) on the (1, 4+nc+32, N)
    detection head output; same rounding (score 4 decimals, box 0.1 px) and dict layout."""
    raw = np.asarray(out_det, dtype=np.float32)
    if raw.ndim != 3 or raw.shape[0] != 1 or raw.shape[1] != 4 + len(names) + mask_channels:
        raise ValueError('YOLO head shape does not match class/mask profile: %s' % (raw.shape,))
    if not np.isfinite(raw).all():
        raise RuntimeError('non-finite detector output')
    p = raw[0].T
    nc = len(names)
    cls_scores = p[:, 4:4 + nc]
    cl = cls_scores.argmax(1)
    sc = cls_scores[np.arange(p.shape[0]), cl]
    k = sc > conf
    p, sc, cl = p[k], sc[k], cl[k]
    xy, wh = p[:, :2], p[:, 2:4]
    boxes = np.concatenate([xy - wh / 2, xy + wh / 2], 1)
    keep = batched_nms_numpy(boxes, sc, cl, iou)[:max_det]
    boxes, sc, cl = boxes[keep], sc[keep], cl[keep]
    ob = lb.to_orig(boxes) if len(boxes) else boxes
    dets = []
    for i in range(len(sc)):
        c = int(cl[i])
        dets.append({"cls": c, "name": names[c] if c < len(names) else str(c),
                     "score": round(float(sc[i]), 4), "box": [round(float(v), 1) for v in ob[i]]})
    return dets


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


class YoloSegDetector:
    detector = "yolo"

    def __init__(self, model, backend, conf=0.25, iou=0.7, classes=DEFAULT_CLASSES, max_det=100,
                 threads=2, precision=None, cuda_sync="blocking", class_min_scores=None, profile='coco-seg'):
        if backend not in ("ort", "trt", "torch"):
            raise ValueError("backend must be ort (host onnxruntime CPU), torch (host CUDA, .pt) "
                             "or trt (board TensorRT)")
        if not (0.0 <= float(conf) < 1.0) or not (0.0 < float(iou) <= 1.0) or int(max_det) <= 0:
            raise ValueError("conf in [0,1), iou in (0,1], max_det > 0 required")
        if not os.path.isfile(model):
            raise FileNotFoundError(model)
        nd = _board_module()
        self._nd = nd
        self.names = profile_names(profile)
        self.profile = profile
        self.mask_channels = 0 if profile in ('household9', 'household33') else 32
        self.task = 'detect' if self.mask_channels == 0 else 'segment'
        if classes is DEFAULT_CLASSES:
            classes = parse_classes(None, profile=profile)
        if classes is not None:
            classes = tuple(classes)
            unknown = [c for c in classes if c not in self.names]
            if not classes or unknown:
                raise ValueError("unknown COCO class name(s): %s" % ", ".join(unknown or ["<empty>"]))
        self.classes = classes
        self.conf, self.iou, self.max_det = float(conf), float(iou), int(max_det)
        floors = DEFAULT_CLASS_MIN_SCORES if profile == 'coco-seg' else {}
        self.class_min_scores = dict(floors if class_min_scores is None else class_min_scores)
        unknown = [c for c in self.class_min_scores if c not in self.names]
        if unknown or not all(0.0 <= v < 1.0 for v in self.class_min_scores.values()):
            raise ValueError("bad class_min_scores: %s" % self.class_min_scores)
        self.backend = backend
        self._geometry = None
        if backend == "ort":
            import onnxruntime as ort
            import cv2
            self._cv2 = cv2
            precision = precision or "fp32"
            options = ort.SessionOptions()
            options.intra_op_num_threads = int(threads)
            options.inter_op_num_threads = 1
            self.session = ort.InferenceSession(model, sess_options=options, providers=["CPUExecutionProvider"])
            if self.session.get_providers() != ["CPUExecutionProvider"]:
                raise RuntimeError("onnxruntime CPU provider explicitly requested; got %s" % self.session.get_providers())
            inputs = self.session.get_inputs()
            outputs = self.session.get_outputs()
            if len(inputs) != 1 or len(outputs) != (2 if self.mask_channels else 1):
                raise ValueError("YOLO export input/output count does not match profile")
            self.input_name = inputs[0].name
            shape = inputs[0].shape
            if (len(shape) != 4 or shape[:2] != [1, 3] or shape[2] != shape[3]
                    or not isinstance(shape[2], int) or shape[2] <= 0):
                raise ValueError("static square NCHW input required, got %s" % (shape,))
            self.size = int(shape[2])
            ranked = sorted(outputs, key=lambda o: len(o.shape))   # (1,116,N) before (1,32,ph,pw)
            if len(ranked[0].shape) != 3 or ranked[0].shape[1] != 4 + len(self.names) + self.mask_channels:
                raise ValueError('YOLO head shape does not match profile')
            self.out_det = ranked[0].name
            device, runtime = "cpu", {"onnxruntime": ort.__version__, "threads": int(threads)}
        elif backend == "torch":
            device, runtime, precision = self._init_torch(model, precision)
        else:
            precision = precision or "fp16"
            self.cuda = nd.set_cuda_sync(cuda_sync)
            self._engine = model
            self._board = None                     # nd.Detector, built on the first frame's size
            self.size = None
            device, runtime = "cuda", {"tensorrt_engine": True, "cuda_sync": cuda_sync}
        self.precision = precision
        self.source = "yolov8n_%s_%s_%s" % ('seg' if self.mask_channels else 'detect', backend, precision)
        self.execution = {"device": device, "backend": backend, "precision": precision,
                          "model": os.path.basename(model), "model_sha256": _sha256(model),
                          "input": self.size, "conf": self.conf, "iou": self.iou, "max_det": self.max_det,
                          "classes": list(self.classes) if self.classes is not None else "all",
                          "class_min_scores": dict(self.class_min_scores)}
        self.execution.update(profile=profile, task=self.task, names=list(self.names), mask_channels=self.mask_channels)
        self.execution.update(runtime)

    def _init_torch(self, model, precision):
        import cv2
        import torch
        precision = precision or "fp32"
        if precision != "fp32":
            raise ValueError("torch backend is fp32 only")
        if not torch.cuda.is_available():
            raise RuntimeError("torch backend needs a CUDA device (CUDA_VISIBLE_DEVICES)")
        import ultralytics
        from ultralytics.nn.tasks import attempt_load_one_weight
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        net, _ = attempt_load_one_weight(model, device="cpu", inplace=True, fuse=True)
        if getattr(net, "task", None) != self.task:
            raise ValueError("expected task %s, got task %r" % (self.task, getattr(net, "task", None)))
        for parameter in net.parameters():
            parameter.requires_grad = False
        self._torch = torch
        self._cv2 = cv2
        self.net = net.to("cuda").float().eval()
        self.size = 640
        nc = len(getattr(net, "names", {}) or {})
        if nc and nc != len(self.names):
            raise ValueError("checkpoint has %d classes, board COCO table has %d" % (nc, len(self.names)))
        runtime = {"torch": torch.__version__, "ultralytics": ultralytics.__version__,
                   "cuda_device": torch.cuda.get_device_name(0), "tf32": False, "fused": True}
        return "cuda", runtime, precision

    def _forward_torch(self, x):
        torch = self._torch
        with torch.inference_mode():
            out = self.net(torch.from_numpy(x).to("cuda"))
            head = out[0] if isinstance(out, (tuple, list)) else out
            return head.float().cpu().numpy()

    # ------------------------------------------------------------------ geometry
    def _letterbox(self, width, height):
        if self._geometry is None or (self._geometry.W, self._geometry.H) != (width, height):
            if self.backend in ("ort", "torch"):
                self._geometry = self._nd.Letterbox(width, height, self.size)
            else:
                if self._board is None:
                    options = {'names': self.names, 'mask_channels': 0} if not self.mask_channels else {}
                    self._board = self._nd.Detector(self._engine, width, height, self.conf, self.iou, self.max_det, **options)
                    self.size = self._board.m.size
                    self.execution["input"] = self.size
                else:   # same engine, new frame size: only the letterbox and its upload buffer change
                    self._board.lb = self._nd.Letterbox(width, height, self._board.m.size)
                    self._board.upload = self._board.torch.empty(
                        (self._board.lb.nh, self._board.lb.nw, 3), dtype=self._board.torch.uint8).pin_memory()
                self._geometry = self._board.lb
        return self._geometry

    def raw_detections(self, rgb):
        """Board-format dicts {cls, name, score, box} for one RGB uint8 HxWx3 image, before the
        class whitelist."""
        rgb = np.ascontiguousarray(rgb)
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
            raise ValueError("RGB uint8 HxWx3 required")
        lb = self._letterbox(rgb.shape[1], rgb.shape[0])
        if self.backend in ("ort", "torch"):
            x = letterbox_input(rgb, lb, self._cv2)
            if self.backend == "ort":
                head = self.session.run([self.out_det], {self.input_name: x})[0]
            else:
                head = self._forward_torch(x)
            if not np.isfinite(head).all():
                raise RuntimeError("non-finite detector output; backend failure, not an empty scene")
            return postprocess_raw(head, lb, self.conf, self.iou, self.max_det, self.names, self.mask_channels)
        self._board.preprocess(rgb)
        self._board.m.run()
        dets = self._board.postprocess(masks=False)
        if not all(np.isfinite(d["score"]) and np.isfinite(d["box"]).all() for d in dets):
            raise RuntimeError("non-finite detector output; backend failure, not an empty scene")
        return dets

    def __call__(self, frame):
        out = []
        for d in self.raw_detections(frame.rgb):
            if self.classes is not None and d["name"] not in self.classes:
                continue
            if d["score"] < self.class_min_scores.get(d["name"], 0.0):
                continue
            out.append(Detection(d["name"], float(d["score"]), tuple(float(v) for v in d["box"]),
                                 self.source, frame.frame_id, frame.timestamp))
        return out


class CombinedDetector:
    """Run both models on the same acquired frame, then apply class-aware cross-model NMS."""
    detector = "yolo"

    def __init__(self, detectors, iou=0.7, max_det=100):
        self.detectors = tuple(detectors)
        self.iou, self.max_det = float(iou), int(max_det)
        self.execution = {"device": self.detectors[0].execution["device"],
                          "profile": "combined", "merge": "class-aware-nms",
                          "models": [d.execution for d in self.detectors]}

    def __call__(self, frame):
        import time
        detections, timings = [], []
        for detector in self.detectors:
            started = time.monotonic()
            found = detector(frame)
            timings.append(time.monotonic() - started)
            if any(d.frame_id != frame.frame_id or d.timestamp != frame.timestamp for d in found):
                raise ValueError("combined detector returned a foreign frame")
            detections.extend(found)
        self.execution["model_seconds"] = timings
        if not detections:
            return []
        names = {label: i for i, label in enumerate(sorted({d.label for d in detections}))}
        keep = batched_nms_numpy(np.array([d.bbox for d in detections], dtype=float),
                                 np.array([d.score for d in detections]),
                                 np.array([names[d.label] for d in detections]), self.iou)
        return [detections[int(i)] for i in keep[:self.max_det]]
