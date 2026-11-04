"""Local-only inference bridge between JetPack Python and ROS Python.

Length-prefixed NPZ requests (no pickle), JSON responses; one frame per connection.
Frame identity is checked at both ends. The socket is owner-only and never TCP.
"""
import argparse
from dataclasses import asdict
import io
import json
import os
import signal
from pathlib import Path
import socket
import struct
import time
import zipfile

import numpy as np

from perception.fpga_replay.core import Detection, Frame

MAX_BYTES = 32 * 1024 * 1024


def receive_message(stream, deadline=None):
    if deadline is None and stream.gettimeout() is not None:
        deadline = time.monotonic() + stream.gettimeout()
    def exact(size):
        chunks = bytearray()
        while len(chunks) < size:
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("inference request deadline exceeded")
                stream.settimeout(remaining)
            block = stream.recv(size - len(chunks))
            if not block:
                raise ConnectionError("incomplete inference message")
            chunks.extend(block)
        return bytes(chunks)
    size = struct.unpack("!I", exact(4))[0]
    if not 0 < size <= MAX_BYTES:
        raise ValueError("inference message exceeds size limit")
    return exact(size)


def send_message(stream, body):
    if not 0 < len(body) <= MAX_BYTES:
        raise ValueError("inference message exceeds size limit")
    stream.sendall(struct.pack("!I", len(body)) + body)


def encode_frame(frame):
    frame.validate()
    metadata = {key: getattr(frame, key) for key in (
        "frame_id", "timestamp", "depth_timestamp", "pose_valid", "pose_source", "calibration_id",
        "pair_tolerance_s")}
    out = io.BytesIO()
    # Deliberately uncompressed: predictable local IPC, no per-frame image loss.
    np.savez(out, metadata=json.dumps(metadata, allow_nan=False), rgb=frame.rgb,
             depth=frame.depth, K=frame.K, T=frame.T_world_camera)
    body = out.getvalue()
    if len(body) > MAX_BYTES:
        raise ValueError("frame exceeds size limit")
    return body


def decode_frame(body):
    if len(body) > MAX_BYTES:
        raise ValueError("frame exceeds size limit")
    with zipfile.ZipFile(io.BytesIO(body)) as archive:
        if sum(entry.file_size for entry in archive.infolist()) > MAX_BYTES:
            raise ValueError("expanded frame exceeds size limit")
        expected = {name + ".npy" for name in ("metadata", "rgb", "depth", "K", "T")}
        if len(archive.namelist()) != len(expected) or set(archive.namelist()) != expected:
            raise ValueError("unexpected frame members")
    with np.load(io.BytesIO(body), allow_pickle=False) as arrays:
        meta = json.loads(str(arrays["metadata"].item()))
        frame = Frame(rgb=arrays["rgb"], depth=arrays["depth"], K=arrays["K"],
                      T_world_camera=arrays["T"], **meta)
    frame.validate()
    return frame


def process_request(body, detector, device):
    frame = decode_frame(body)
    started = time.monotonic()
    detections = detector(frame)
    if any(d.frame_id != frame.frame_id or d.timestamp != frame.timestamp for d in detections):
        raise ValueError("detector returned stale or foreign frame")
    return {"ok": True, "frame_id": frame.frame_id, "timestamp": frame.timestamp,
            "detector": getattr(detector, "detector", None),
            "execution": getattr(detector, "execution", {"device": device}),
            "device": device, "inference_seconds": time.monotonic() - started,
            "detections": [asdict(d) for d in detections]}


class InferenceClient:
    def __init__(self, socket_path, timeout=10, expected_device=None, expected_detector=None):
        self.path = str(socket_path)
        self.timeout = float(timeout)
        if not np.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError("positive finite socket timeout required")
        self.expected_device = expected_device
        # Like expected_device: None = unchecked (legacy callers, stub/fixture services);
        # a name = the service's declared "detector" must equal it, or the frame is refused.
        self.expected_detector = expected_detector
        self.last_metadata = None

    def detect(self, frame):
        deadline = time.monotonic() + self.timeout
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stream:
            stream.settimeout(self.timeout)
            stream.connect(self.path)
            send_message(stream, encode_frame(frame))
            reply = json.loads(receive_message(stream, deadline=deadline))
        if not reply.get("ok"):
            raise RuntimeError("inference service failed: " + reply.get("error", "unknown"))
        if reply.get("frame_id") != frame.frame_id or reply.get("timestamp") != frame.timestamp:
            raise ValueError("inference reply belongs to a different frame")
        if self.expected_device is not None and reply.get("device") != self.expected_device:
            raise ValueError("inference backend differs from requested device")
        if self.expected_detector is not None and reply.get("detector") != self.expected_detector:
            raise ValueError("inference detector %r differs from requested detector %r"
                             % (reply.get("detector"), self.expected_detector))
        detections = [Detection(**item) for item in reply["detections"]]
        for det in detections:
            if det.frame_id != frame.frame_id or det.timestamp != frame.timestamp:
                raise ValueError("detection belongs to a different frame")
            if not np.isfinite(det.bbox).all() or not np.isfinite(det.score):
                raise ValueError("non-finite detection reply")
        self.last_metadata = {k: v for k, v in reply.items() if k != "detections"}
        return detections


def warm_up(detector, frames, size):
    """Run the detector `frames` times on a blank frame of `size` (W, H) before the service says ready.

    D2 run_b2 (2026-10-14): the first request of every run cost 2.4-10+ s of round trip (CUDA/cuDNN lazy
    initialisation inside the first forward, ~2.3 s CPU on an idle host; all 20 pre-fix D1+D2 runs with
    stall_diag: 2.37-10.01 s), steady state
    ~45 ms; under host load it passed the runner's 10 s client timeout -> runner fault. Paying it here
    moves the cost before "ready". Returns the per-call seconds (printed in the ready line)."""
    width, height = size
    frame = Frame(frame_id="warmup", timestamp=0.0, rgb=np.zeros((height, width, 3), np.uint8),
                  depth=np.full((height, width), np.nan, np.float32),
                  K=np.array([[width / 2.0, 0.0, width / 2.0], [0.0, width / 2.0, height / 2.0], [0.0, 0.0, 1.0]]),
                  T_world_camera=np.eye(4), pose_source="warmup", calibration_id="warmup")
    frame.validate()
    seconds = []
    for _ in range(int(frames)):
        started = time.monotonic()
        detector(frame)
        seconds.append(round(time.monotonic() - started, 4))
    return seconds


def parse_size(text):
    width, height = (int(v) for v in str(text).lower().split("x"))
    if width <= 0 or height <= 0:
        raise ValueError("size must be WxH with positive integers")
    return width, height


def serve(socket_path, detector, device, request_timeout=10, ready_extra=None):
    path = Path(socket_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Existing sockets may be live. Never unlink a path owned by another launch.
    if path.exists() or path.is_symlink():
        raise FileExistsError("socket path already exists; inspect its owner before cleanup")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as listener:
        listener.bind(str(path))
        inode = path.stat().st_ino
        os.chmod(str(path), 0o600)
        listener.listen(1)
        print(json.dumps(dict({"ready": True, "socket": str(path), "device": device}, **(ready_extra or {}))),
              flush=True)
        try:
            while True:
                connection, _ = listener.accept()
                with connection:
                    connection.settimeout(request_timeout)
                    try:
                        reply = process_request(receive_message(connection), detector, device)
                    except Exception as error:
                        reply = {"ok": False, "error": type(error).__name__ + ": " + str(error)}
                    try:
                        send_message(connection, json.dumps(reply, allow_nan=False).encode())
                    except (OSError, ValueError):
                        pass  # timed-out client cannot poison the next acquisition
        finally:
            if path.exists() and path.stat().st_ino == inode:
                path.unlink()


def build_detector(args, parser=None):
    """Detector named by --detector; ssdlite keeps its pre-2026-10-14 construction unchanged."""
    def fail(message):
        if parser is not None:
            parser.error(message)
        raise ValueError(message)
    if args.detector == "ssdlite":
        if not args.models:
            fail("--detector ssdlite requires --models")
        if args.onnx or args.engine or getattr(args, "pt", None):
            fail("--onnx/--engine/--pt belong to --detector yolo")
        from perception.fpga_replay.detector import SSDDetector
        detector = SSDDetector(args.models, score=args.score, device=args.device,
                               memory_format=args.memory_format, cuda_math=args.cuda_math,
                               augmentation=args.augmentation)
        detector.detector = "ssdlite"
        return detector
    from perception.fpga_replay.yolo_detector import (DEFAULT_CLASS_MIN_SCORES, DEFAULT_EXCLUDE_CLASSES, YoloSegDetector,
                                                     parse_class_min_scores, parse_classes, parse_exclude)
    pt = getattr(args, "pt", None)
    if sum(bool(x) for x in (args.onnx, args.engine, pt)) != 1:
        fail("--detector yolo requires exactly one of --onnx (host onnxruntime CPU), "
             "--pt (host torch CUDA) or --engine (board TensorRT)")
    if args.onnx and args.device != "cpu":
        fail("--onnx runs on onnxruntime CPU only; use --device cpu")
    if args.engine and args.device != "cuda":
        fail("--engine runs on TensorRT; use --device cuda")
    if pt and args.device != "cuda":
        fail("--pt runs on torch CUDA; use --device cuda")
    exclude_text = getattr(args, "exclude_classes", None)
    if args.classes is not None and exclude_text is not None:
        fail("--exclude-classes only edits the default class set; with --classes list the classes wanted")
    try:
        profile = getattr(args, 'yolo_profile', 'coco-seg')
        exclude = (parse_exclude(exclude_text, profile) if exclude_text is not None else
                   (DEFAULT_EXCLUDE_CLASSES if profile in ('coco-seg', 'combined', 'combined33') else ()))
        classes = parse_classes(args.classes, exclude, profile)
        min_text = getattr(args, "class_min_score", None)
        class_min_scores = parse_class_min_scores(min_text, profile) if min_text is not None else None
    except ValueError as error:
        fail(str(error))
    backend = "ort" if args.onnx else ("torch" if pt else "trt")
    if profile in ('combined', 'combined33'):
        from perception.fpga_replay.yolo_detector import CombinedDetector, profile_names
        coco_model = getattr(args, 'coco_model', None)
        if not coco_model:
            fail("combined profile requires --coco-model (same backend as the household model)")
        route = None
        if profile == 'combined33':
            try:
                route = yolo_route(args)
            except ValueError as error:
                fail(str(error))
        detectors, assignment = [], {}
        hkind = 'household33' if profile == 'combined33' else 'household9'
        for kind, model in ((hkind, args.onnx or pt or args.engine), ('coco-seg', coco_model)):
            names = profile_names(kind)
            selected = route_classes(kind, names, classes, route)
            assignment[kind] = None if selected is None else list(selected)
            if selected == ():
                continue                    # no class for this model: it is not built (all33: the COCO engine)
            # 10-30: the default floors go to whichever model serves the class (YoloSegDetector applies them by
            # itself only for the coco-seg profile). The 33-class model has its own defaults (backpack 0.35, owner
            # 10-30: 838 backpack-free car frames gave 0 false backpacks at 0.35); the COCO model keeps 0.45.
            defaults = HOUSEHOLD33_CLASS_MIN_SCORES if kind == 'household33' else DEFAULT_CLASS_MIN_SCORES
            floors = {n: v for n, v in (defaults if class_min_scores is None
                                        else class_min_scores).items() if n in names}
            detectors.append(YoloSegDetector(model, backend=backend, conf=args.conf, iou=args.iou,
                                            classes=selected, threads=args.threads,
                                            class_min_scores=floors, profile=kind))
        combined = CombinedDetector(detectors, iou=args.iou)
        if route is not None:
            combined.yolo_route = route
            combined.assignment = assignment
        return combined
    return YoloSegDetector(args.onnx or pt or args.engine, backend=backend,
                           conf=args.conf, iou=args.iou, classes=classes, threads=args.threads,
                           class_min_scores=class_min_scores, profile=profile)


# 10-30 (owner): default per-class floors on the 33-class model (combined33). The COCO model keeps
# yolo_detector.DEFAULT_CLASS_MIN_SCORES (backpack 0.45, set 10-22 from COCO false-backpack scores).
# Evidence: runs/detect-night-20261030-results (33-class backpack max 0.30 on 420 backpack-free night frames).
HOUSEHOLD33_CLASS_MIN_SCORES = {"backpack": 0.35}

YOLO_ROUTES = ("all33", "primary")
YOLO_ROUTE_ENV = "CAR_YOLO_ROUTE"


def yolo_route(args, environ=None):
    """10-30 (owner): which model of combined33 serves which enabled class.
    all33 (default): every enabled class the 33-class model knows (household_names.NAMES33) goes to it; only
    classes it does not know go to the COCO model (none of the six demo items: the COCO model is then not built).
    primary: the pre-10-30 split, the 33-class model only for household_names.PRIMARY33 (rollback).
    --yolo-route wins over $CAR_YOLO_ROUTE; unset / empty = all33; anything else raises ValueError."""
    route = getattr(args, "yolo_route", None)
    source = "--yolo-route"
    if route is None:
        route = (os.environ if environ is None else environ).get(YOLO_ROUTE_ENV, "")
        source = YOLO_ROUTE_ENV
        if route == "":
            return "all33"
    if route not in YOLO_ROUTES:
        raise ValueError("%s must be one of %s (got %r)" % (source, "|".join(YOLO_ROUTES), route))
    return route


def route_classes(kind, names, classes, route):
    """The classes model `kind` (class table `names`) serves: None = all of its own, () = none (not built).
    route None = the combined (household9) profile: every model its own enabled classes, as before."""
    selected = None if classes is None else tuple(n for n in classes if n in names)
    if route == "primary":
        from perception.fpga_replay.household_names import PRIMARY33
        if selected is not None:
            selected = tuple(n for n in selected if (n in PRIMARY33) == (kind == 'household33'))
    elif route == "all33" and kind == "coco-seg":
        from perception.fpga_replay.household_names import NAMES33
        selected = tuple(n for n in (names if selected is None else selected) if n not in NAMES33)
    return selected


def served_classes(detector):
    """10-29: the class names this detector can return (the ready line's "classes", read by the stack's health
    check, host/hub/health.py); None when the detector does not say."""
    subs = getattr(detector, "detectors", None)
    if subs is not None:
        out = []
        for d in subs:
            out += [c for c in (served_classes(d) or ()) if c not in out]
        return out
    classes = getattr(detector, "classes", None)
    if classes is not None:
        return list(classes)
    names = getattr(detector, "names", None)
    return None if names is None else list(names)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True)
    parser.add_argument("--detector", choices=("ssdlite", "yolo"), default="ssdlite")
    parser.add_argument("--models", help="ssdlite: torch hub directory (required for ssdlite)")
    parser.add_argument("--device", choices=("cpu", "cuda"), required=True)
    parser.add_argument("--score", type=float, default=0.4, help="ssdlite score threshold")
    parser.add_argument("--memory-format", choices=("contiguous", "channels_last"), default="contiguous")
    parser.add_argument("--cuda-math", choices=("fp32", "tf32"), default="fp32")
    parser.add_argument("--augmentation", choices=("none", "hflip_shift4"), default="none")
    parser.add_argument("--onnx", help="yolo: ONNX export, onnxruntime CPU (host; needs --device cpu)")
    parser.add_argument("--engine", help="yolo: TensorRT engine (board; needs --device cuda)")
    parser.add_argument('--yolo-profile', choices=('coco-seg', 'household9', 'combined', 'household33', 'combined33'), default='coco-seg',
                        help='explicit class order and head format; household9 is detection-only')
    parser.add_argument("--yolo-route", choices=YOLO_ROUTES, default=None,
                        help="combined33 split (10-30): all33 = every enabled class the 33-class model knows goes to "
                             "it, COCO only for the rest (not built when it gets none); primary = the old PRIMARY33 "
                             "split (rollback). Default: $CAR_YOLO_ROUTE, else all33")
    parser.add_argument("--coco-model", help="combined: original COCO model, same backend as primary household model")
    parser.add_argument("--pt", help="yolo: .pt checkpoint, torch CUDA fp32 via ultralytics (host; needs --device cuda)")
    parser.add_argument("--conf", type=float, default=0.25, help="yolo score threshold")
    parser.add_argument("--iou", type=float, default=0.7, help="yolo class-aware NMS IoU")
    parser.add_argument("--classes", default=None,
                        help="yolo class whitelist, COCO names comma-separated, taken literally; 'all' = no "
                             "whitelist (all 80, person included). Default (D-044 revised 2026-10-21; tv off "
                             "since 10-22): the 25 defence-venue classes (perception/fpga_replay/coco_zh.py VENUE_CLASSES) minus "
                             "--exclude-classes. D-036's four: 'bottle,cup,backpack,cell phone'")
    parser.add_argument("--exclude-classes", default=None,
                        help="yolo: COCO names removed from the DEFAULT (venue, 25) class set (default: ten disabled incl. umbrella, leaving 15). "
                             "Only with the default set, not with --classes. Note (D-044 revised): '' now "
                             "restores all 25 venue classes, not all 80; person and "
                             "the other cancelled classes come back only through --classes")
    parser.add_argument("--class-min-score", default=None,
                        help="yolo: per-class score floors on top of --conf, 'name=score,...' "
                             "(default: COCO model yolo_detector.DEFAULT_CLASS_MIN_SCORES backpack=0.45; 33-class model HOUSEHOLD33_CLASS_MIN_SCORES backpack=0.35; '' = none)")
    parser.add_argument("--threads", type=int, default=2, help="yolo onnxruntime intra-op threads")
    parser.add_argument("--warmup-frames", type=int, default=0,
                        help="blank-frame detector calls before 'ready' (pays CUDA lazy init up front; 0 = none)")
    parser.add_argument("--warmup-size", default="640x400", help="warm-up frame WxH (the stream's camera size)")
    args = parser.parse_args()
    if args.warmup_frames < 0:
        parser.error("--warmup-frames must be >= 0")
    try:
        warmup_size = parse_size(args.warmup_size)
    except ValueError as error:
        parser.error("--warmup-size: %s" % error)
    detector = build_detector(args, parser)
    warmup_s = warm_up(detector, args.warmup_frames, warmup_size)
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    extra = {"warmup_s": warmup_s} if args.warmup_frames else {}
    try:
        classes = served_classes(detector)
    except Exception:                      # the ready line must never fail on this
        classes = None
    if classes is not None:
        extra.update(classes=classes, profile=getattr(args, "yolo_profile", None) if args.detector == "yolo" else "ssdlite")
    if getattr(detector, "yolo_route", None) is not None:   # 10-30: the combined33 split, grep-able in inference.log
        extra.update(yolo_route=detector.yolo_route, assignment=detector.assignment)
    try:
        serve(args.socket, detector, args.device, ready_extra=extra or None)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
