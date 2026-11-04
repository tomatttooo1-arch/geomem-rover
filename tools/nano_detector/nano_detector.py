#!/usr/bin/env python3
"""nano_detector: Gemini 2 colour frames -> YOLOv8n-seg (TensorRT FP16) -> one JSON line per frame.

Board-side recognition service for the Orin Nano (JetPack 6.2, TensorRT 10.3, torch 2.5 with CUDA).

Frame source: pyorbbecsdk2 (bundles OrbbecSDK 2.9.3, the same SDK as the C++ probe), colour
1280x720@30 RGB (native UVC format, no JPEG decode). A grabber thread keeps only the newest frame,
so a slow consumer never builds a queue inside the SDK (latency stays bounded, stale frames are
counted as skipped instead).

Pipeline per frame (main thread):
  pre   : cv2.resize (CPU, INTER_LINEAR, same as ultralytics letterbox) -> upload -> pad 114, /255 on GPU
  infer : TensorRT execute_async_v3 on a torch CUDA stream, synchronised
  post  : on GPU with torch: score filter, class-aware NMS (torchvision), mask = coeff @ protos,
          cropped to the box at prototype resolution (1/4 of the network input); only the
          small cropped masks are copied back and run-length encoded on the CPU.

Output (--out, JSON lines). Per processed frame:
  seq, frame_index (device frame counter), dev_ts_us (device clock), sdk_host_ts_us (SDK host
  receive time, epoch us), grab_us / done_us (this process, epoch us), wait_ms, pre_ms, infer_ms,
  post_ms, proc_ms (grab->done), e2e_ms (sdk_host_ts->done), skipped (newer frames that arrived
  and were dropped since the previous processed frame), late (paced mode: e2e > period), warmup,
  dets: [{cls, name, score, box [x0,y0,x1,y1] in 1280x720 pixels,
          mask {stride_px (original pixels per mask cell), box_cells [x0,y0,x1,y1] on the
                prototype grid, shape [h,w], rle (row-major run lengths, first run = zeros),
                area_px (mask cells x stride^2, original pixels)}}]
A final JSON summary is written to --summary.

Modes: --mode paced --hz 15 processes only frames whose device index is a multiple of 30/hz
(scheduled frames); a scheduled frame that is superseded before it is taken is counted as
missed. --mode best takes the newest frame every time.
"""
import argparse, json, os, sys, threading, time

import numpy as np

COCO = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
        "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
        "scissors", "teddy bear", "hair drier", "toothbrush"]


def rle_encode(m):
    """Row-major run lengths of a 2-D bool array; the first run counts zeros (may be 0)."""
    f = m.reshape(-1).astype(np.uint8)
    if f.size == 0:
        return []
    d = np.flatnonzero(np.diff(f)) + 1
    b = np.concatenate(([0], d, [f.size]))
    runs = np.diff(b).tolist()
    return runs if f[0] == 0 else [0] + runs


def rle_decode(runs, shape):
    out = np.zeros(int(np.prod(shape)), np.uint8); p = 0; v = 0
    for r in runs:
        out[p:p + r] = v; p += r; v ^= 1
    return out.reshape(shape).astype(bool)


class Letterbox:
    """Geometry of the ultralytics letterbox (scale to fit, centred, pad 114)."""
    def __init__(self, W, H, S):
        self.W, self.H, self.S = W, H, S
        self.r = min(S / W, S / H)
        self.nw, self.nh = int(round(W * self.r)), int(round(H * self.r))
        self.left = int(round((S - self.nw) / 2 - 0.1)); self.top = int(round((S - self.nh) / 2 - 0.1))

    def to_orig(self, xyxy):  # network-input pixels -> original pixels (numpy, (k,4))
        o = xyxy.copy()
        o[:, [0, 2]] = (o[:, [0, 2]] - self.left) / self.r
        o[:, [1, 3]] = (o[:, [1, 3]] - self.top) / self.r
        o[:, [0, 2]] = o[:, [0, 2]].clip(0, self.W); o[:, [1, 3]] = o[:, [1, 3]].clip(0, self.H)
        return o


class TRTModel:
    def __init__(self, path):
        import tensorrt as trt, torch
        self.torch = torch
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(path, "rb") as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError('TensorRT engine deserialization failed')
        self.ctx = self.engine.create_execution_context()
        tmap = {trt.float32: torch.float32, trt.float16: torch.float16, trt.int32: torch.int32}
        self.buf, self.inputs, self.outputs = {}, [], []
        for i in range(self.engine.num_io_tensors):
            n = self.engine.get_tensor_name(i)
            shp = tuple(self.engine.get_tensor_shape(n))
            t = torch.empty(shp, dtype=tmap[self.engine.get_tensor_dtype(n)], device="cuda")
            self.buf[n] = t; self.ctx.set_tensor_address(n, t.data_ptr())
            (self.inputs if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT else self.outputs).append(n)
        self.stream = torch.cuda.Stream()
        self.inp = self.buf[self.inputs[0]]
        self.size = self.inp.shape[-1]

    def run(self):
        with self.torch.cuda.stream(self.stream):
            if not self.ctx.execute_async_v3(self.stream.cuda_stream):
                raise RuntimeError('TensorRT execution failed')
        self.stream.synchronize()


class Detector:
    def __init__(self, engine, W=1280, H=720, conf=0.25, iou=0.7, max_det=100, names=None, mask_channels=32):
        import torch, torchvision, cv2
        self.torch, self.tv, self.cv2 = torch, torchvision, cv2
        self.m = TRTModel(engine)
        self.lb = Letterbox(W, H, self.m.size)
        self.conf, self.iou, self.max_det = conf, iou, max_det
        self.names = list(COCO if names is None else names)
        self.mask_channels = int(mask_channels)
        if self.mask_channels not in (0, 32):
            raise ValueError('supported mask channel counts are 0 and 32')
        o = sorted(self.m.outputs, key=lambda n: len(self.m.buf[n].shape))
        if len(self.m.inputs) != 1 or tuple(self.m.inp.shape) != (1, 3, self.m.size, self.m.size):
            raise ValueError('static square NCHW engine input required')
        if len(o) != (2 if self.mask_channels else 1):
            raise ValueError('TensorRT output count does not match class/mask profile')
        self.out_det, self.out_proto = o[0], o[1] if self.mask_channels else None
        shape = tuple(self.m.buf[self.out_det].shape)
        if len(shape) != 3 or shape[0] != 1 or shape[1] != 4 + len(self.names) + self.mask_channels:
            raise ValueError('TensorRT head shape does not match class/mask profile: %s' % (shape,))
        self.proto_stride = self.m.size / self.m.buf[self.out_proto].shape[-1] if self.mask_channels else 0.
        self.upload = torch.empty((self.lb.nh, self.lb.nw, 3), dtype=torch.uint8).pin_memory()

    def preprocess(self, rgb):
        cv2, torch = self.cv2, self.torch
        small = cv2.resize(rgb, (self.lb.nw, self.lb.nh), interpolation=cv2.INTER_LINEAR)
        self.upload.numpy()[:] = small
        with torch.cuda.stream(self.m.stream):
            g = self.upload.to("cuda", non_blocking=True)
            x = self.m.inp
            x.fill_(114.0 / 255.0)
            x[0, :, self.lb.top:self.lb.top + self.lb.nh, self.lb.left:self.lb.left + self.lb.nw] = \
                g.permute(2, 0, 1).to(x.dtype) / 255.0
        self.m.stream.synchronize()

    def postprocess(self, masks=True):
        torch = self.torch
        mask_channels = getattr(self, 'mask_channels', 32)
        names = getattr(self, 'names', COCO)
        if masks and not mask_channels:
            raise ValueError('detection-only model has no segmentation masks')
        with torch.cuda.stream(self.m.stream):
            p = self.m.buf[self.out_det][0].float().T            # (N, 4+80+32)
            nc = p.shape[1] - 4 - mask_channels
            if nc != len(names):
                raise ValueError('TensorRT class table mismatch')
            if not torch.isfinite(p).all():
                raise RuntimeError('non-finite TensorRT detector output')
            sc, cl = p[:, 4:4 + nc].max(1)
            k = sc > self.conf
            p, sc, cl = p[k], sc[k], cl[k]
            xy, wh = p[:, :2], p[:, 2:4]
            boxes = torch.cat([xy - wh / 2, xy + wh / 2], 1)
            keep = self.tv.ops.batched_nms(boxes, sc, cl, self.iou)[:self.max_det]
            boxes, sc, cl, coef = boxes[keep], sc[keep], cl[keep], p[keep, 4 + nc:]
            host = torch.cat([boxes, sc[:, None], cl[:, None].float()], 1).cpu().numpy()   # one sync
            binm = None
            if masks and len(keep):
                pr = self.m.buf[self.out_proto][0].float()        # (32, ph, pw)
                ph, pw = pr.shape[1:]
                binm = ((coef @ pr.reshape(32, -1)).reshape(-1, ph, pw) > 0).cpu().numpy()   # sigmoid > 0.5
        boxes, sc, cl = host[:, :4], host[:, 4], host[:, 5].astype(int)
        crops = []
        if binm is not None:
            s = self.proto_stride; ph, pw = binm.shape[1:]
            # crop each mask to its box on the prototype grid, limited to the non-padding region
            vx0, vy0 = self.lb.left / s, self.lb.top / s
            vx1, vy1 = (self.lb.left + self.lb.nw) / s, (self.lb.top + self.lb.nh) / s
            for i in range(len(sc)):
                x0 = int(np.clip(np.floor(max(boxes[i, 0] / s, vx0)), 0, pw)); y0 = int(np.clip(np.floor(max(boxes[i, 1] / s, vy0)), 0, ph))
                x1 = int(np.clip(np.ceil(min(boxes[i, 2] / s, vx1)), 0, pw)); y1 = int(np.clip(np.ceil(min(boxes[i, 3] / s, vy1)), 0, ph))
                crops.append(((x0, y0, x1, y1), binm[i, y0:y1, x0:x1]))
        ob = self.lb.to_orig(boxes) if len(boxes) else boxes
        cell = self.proto_stride / self.lb.r                      # original pixels per mask cell
        dets = []
        for i in range(len(sc)):
            d = {"cls": int(cl[i]), "name": names[int(cl[i])],
                 "score": round(float(sc[i]), 4), "box": [round(float(v), 1) for v in ob[i]]}
            if crops:
                b, c = crops[i]
                d["mask"] = {"stride_px": round(cell, 3), "box_cells": list(b), "shape": list(c.shape),
                             "rle": rle_encode(c), "area_px": int(round(int(c.sum()) * cell * cell))}
            dets.append(d)
        return dets


class Grabber(threading.Thread):
    """Keeps the newest colour frame; counts every frame delivered by the SDK."""
    def __init__(self, W, H, fps, fmt):
        super().__init__(daemon=True)
        from pyorbbecsdk import Pipeline, Config, OBSensorType, OBFormat
        self.pipe = Pipeline(); cfg = Config()
        prof = self.pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        cfg.enable_stream(prof.get_video_stream_profile(W, H, getattr(OBFormat, fmt), fps))
        self.pipe.start(cfg)
        info = self.pipe.get_device().get_device_info()
        self.device = {"name": info.get_name(), "serial": info.get_serial_number(),
                       "firmware": info.get_firmware_version(), "connection": info.get_connection_type(),
                       "stream": f"color {W}x{H}@{fps}:{fmt}"}
        self.cond = threading.Condition(); self.latest = None; self.delivered = 0; self.stop_flag = False
        self.W, self.H = W, H

    def run(self):
        while not self.stop_flag:
            fs = self.pipe.wait_for_frames(100)
            if fs is None:
                continue
            f = fs.get_color_frame()
            if f is None:
                continue
            rec = (f.get_index(), f.get_timestamp_us(), f.get_system_timestamp_us(), f, fs)
            with self.cond:
                self.latest = rec; self.delivered += 1; self.cond.notify_all()

    def take(self, pred, timeout=1.0):
        """Newest frame whose index satisfies pred(index) and is newer than the last taken."""
        end = time.time() + timeout
        with self.cond:
            while True:
                r = self.latest
                if r is not None and pred(r[0]):
                    self.latest = None
                    return r
                left = end - time.time()
                if left <= 0:
                    return None
                self.cond.wait(left)

    def close(self):
        self.stop_flag = True; self.join(2); self.pipe.stop()


class _ShmFrame:
    def __init__(self, arr):
        self.arr = arr

    def get_data(self):
        return self.arr


class ShmGrabber(threading.Thread):
    """Frames from the N4 shared-memory ring (tools/nano_fullload/shm_ring.py) written by the ROS
    measurement node (the ROS driver owns the camera). Same interface as Grabber: index = ring seq,
    device ts = SDK-host ts = the driver's message header stamp (us); tap receipt time is kept too."""
    def __init__(self, path, poll_s=0.002):
        super().__init__(daemon=True)
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "nano_fullload"))
        from shm_ring import RingReader
        self.ring = RingReader(path)
        self.W, self.H = self.ring.w, self.ring.h
        self.device = {"name": "shm-ring", "path": path, "stream": f"color {self.W}x{self.H} via ROS driver"}
        self.cond = threading.Condition(); self.latest = None; self.delivered = 0; self.stop_flag = False
        self.poll_s = poll_s; self.last_rx_us = None

    def run(self):
        last = self.ring.seq()
        while not self.stop_flag:
            s = self.ring.seq()
            if s != last:
                with self.cond:
                    self.delivered += s - last; last = s
                    self.latest = s; self.cond.notify_all()
            time.sleep(self.poll_s)

    def take(self, pred, timeout=1.0):
        end = time.time() + timeout
        with self.cond:
            while True:
                s = self.latest
                if s is not None and pred(s):
                    self.latest = None
                    break
                left = end - time.time()
                if left <= 0:
                    return None
                self.cond.wait(left)
        r = self.ring.read(s)
        if r is None:
            return None
        arr, stamp_ns, rx_ns = r
        self.last_rx_us = rx_ns // 1000
        return (s, stamp_ns // 1000, stamp_ns // 1000, _ShmFrame(arr), None)

    def close(self):
        self.stop_flag = True; self.join(2)


def annotate(rgb, dets, lb, out_path, width=640, quality=70):
    import cv2
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    over = img.copy()
    for d in dets:
        if "mask" in d:
            m = d["mask"]; x0, y0, x1, y1 = m["box_cells"]
            c = rle_decode(m["rle"], m["shape"]).astype(np.uint8)
            s = m["stride_px"]
            ox0 = int(round((x0 * (s * lb.r) - lb.left) / lb.r)); oy0 = int(round((y0 * (s * lb.r) - lb.top) / lb.r))
            ow, oh = int(round(c.shape[1] * s)), int(round(c.shape[0] * s))
            if ow > 0 and oh > 0:
                big = cv2.resize(c, (ow, oh), interpolation=cv2.INTER_NEAREST).astype(bool)
                X0, Y0 = max(ox0, 0), max(oy0, 0)
                X1, Y1 = min(ox0 + ow, img.shape[1]), min(oy0 + oh, img.shape[0])
                reg = over[Y0:Y1, X0:X1]
                reg[big[Y0 - oy0:Y1 - oy0, X0 - ox0:X1 - ox0]] = (0, 200, 255)
    img = cv2.addWeighted(over, 0.35, img, 0.65, 0)
    for d in dets:
        x0, y0, x1, y1 = [int(v) for v in d["box"]]
        cv2.rectangle(img, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(img, f"{d['name']} {d['score']:.2f}", (x0 + 2, max(y0 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    h = int(round(img.shape[0] * width / img.shape[1]))
    cv2.imwrite(out_path, cv2.resize(img, (width, h), interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, quality])


def set_cuda_sync(mode):
    """Set the primary-context scheduling flag before any CUDA use (torch and TensorRT share the
    primary context). 'blocking' makes stream/event waits sleep instead of spin, which is what
    keeps the detector's CPU cost low; 'spin' / 'auto' are the CUDA alternatives. Returns the
    flags read back (after the context exists they are re-read in main)."""
    import ctypes
    flags = {"auto": 0, "spin": 1, "yield": 2, "blocking": 4}[mode]
    cu = ctypes.CDLL("libcuda.so.1")
    if cu.cuInit(0) != 0:
        raise RuntimeError("cuInit failed")
    dev = ctypes.c_int()
    cu.cuDeviceGet(ctypes.byref(dev), 0)
    fn = getattr(cu, "cuDevicePrimaryCtxSetFlags_v2", None) or cu.cuDevicePrimaryCtxSetFlags
    r = fn(dev, ctypes.c_uint(flags))
    if r != 0:
        raise RuntimeError(f"cuDevicePrimaryCtxSetFlags -> {r}")
    return cu, dev


def cuda_ctx_flags(cu, dev):
    import ctypes
    fl, act = ctypes.c_uint(), ctypes.c_int()
    cu.cuDevicePrimaryCtxGetState(dev, ctypes.byref(fl), ctypes.byref(act))
    return {"flags": fl.value, "active": act.value}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", required=True)
    ap.add_argument("--mode", choices=["paced", "best"], default="paced")
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--cam-fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=60.0, help="measured window")
    ap.add_argument("--warmup-s", type=float, default=5.0)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.7)
    ap.add_argument("--no-masks", action="store_true")
    ap.add_argument("--cuda-sync", choices=["blocking", "spin", "yield", "auto"], default="blocking")
    ap.add_argument("--out", required=True)
    ap.add_argument("--summary")
    ap.add_argument("--phase-file"); ap.add_argument("--tag", default="det")
    ap.add_argument("--source", default="orbbec", help="orbbec (open the camera) or shm:<ring file> (N4)")
    ap.add_argument("--snapshot", help="save annotated JPEG (<=640 wide, q70) of the last frame, plus raw PNG next to it")
    a = ap.parse_args()

    import signal
    def _term(*_):  # N4: background jobs ignore SIGINT, so the orchestrator stops us with SIGTERM
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    cu, cudev = set_cuda_sync(a.cuda_sync)
    import cv2, torch
    cv2.setNumThreads(1); torch.set_num_threads(1)
    W, H = 1280, 720
    t_load = time.perf_counter()
    if a.source.startswith("shm:"):
        gr = ShmGrabber(a.source[4:]); W, H = gr.W, gr.H
        det = Detector(a.engine, W, H, a.conf, a.iou); gr.start()
    else:
        det = Detector(a.engine, W, H, a.conf, a.iou)
        gr = Grabber(W, H, a.cam_fps, "RGB"); gr.start()
    t_load = time.perf_counter() - t_load
    step = max(1, int(round(a.cam_fps / a.hz))) if a.mode == "paced" else 1
    period_ms = 1000.0 * step / a.cam_fps
    ph = open(a.phase_file, "a") if a.phase_file else None
    def mark(s):
        if ph:
            ph.write(f"{time.time():.9f} {s}\n"); ph.flush()
    # shm source (N4): line-buffered so a between-layer check never reads a stale tail
    out = open(a.out, "w", buffering=1 if a.source.startswith("shm:") else 1 << 16)
    last_idx = None; seq = 0; t_start = time.time(); win = None; stats = []
    last_rgb = last_dets = None
    t_end_warm = t_start + a.warmup_s
    try:
        while True:
            now = time.time()
            if win is None and now >= t_end_warm:
                win = (now, time.process_time(), gr.delivered); mark(f"start {a.tag}")
            if win is not None and now >= win[0] + a.seconds:
                break
            t_w = time.perf_counter()
            r = gr.take((lambda i: (i % step == 0) and (last_idx is None or i > last_idx)))
            if r is None:
                continue
            idx, dev_us, sdk_us, frame, _fs = r
            grab_us = time.time_ns() // 1000
            wait_ms = (time.perf_counter() - t_w) * 1e3
            rgb = np.asarray(frame.get_data()).reshape(H, W, 3)
            t0 = time.perf_counter(); det.preprocess(rgb)
            t1 = time.perf_counter(); det.m.run()
            t2 = time.perf_counter(); dets = det.postprocess(masks=not a.no_masks)
            t3 = time.perf_counter()
            done_us = time.time_ns() // 1000
            skipped = 0 if last_idx is None else (idx - last_idx) // step - 1
            e2e = (done_us - sdk_us) / 1e3
            rec = {"seq": seq, "frame_index": idx, "dev_ts_us": dev_us, "sdk_host_ts_us": sdk_us, "grab_us": grab_us,
                   "done_us": done_us, "wait_ms": round(wait_ms, 3), "pre_ms": round((t1 - t0) * 1e3, 3),
                   "infer_ms": round((t2 - t1) * 1e3, 3), "post_ms": round((t3 - t2) * 1e3, 3),
                   "proc_ms": round((done_us - grab_us) / 1e3, 3), "e2e_ms": round(e2e, 3), "skipped": skipped,
                   "late": bool(a.mode == "paced" and e2e > period_ms), "warmup": win is None, "dets": dets}
            if getattr(gr, "last_rx_us", None) is not None:
                rec["tap_rx_us"] = gr.last_rx_us
            out.write(json.dumps(rec, separators=(",", ":")) + "\n")
            if win is not None:
                stats.append(rec)
            last_idx = idx; seq += 1
            if a.snapshot:
                last_rgb, last_dets = rgb.copy(), dets
            del frame, _fs, r
    finally:
        t_stop = time.time(); cpu = time.process_time(); delivered = gr.delivered
        mark(f"end {a.tag}")
        gr.close(); out.close()
    if a.snapshot and last_rgb is not None:
        annotate(last_rgb, last_dets, det.lb, a.snapshot)
        cv2.imwrite(os.path.splitext(a.snapshot)[0] + "_raw.png", cv2.cvtColor(last_rgb, cv2.COLOR_RGB2BGR))

    def pct(v, q):
        v = sorted(v); return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else None
    summ = {"tag": a.tag, "engine": os.path.abspath(a.engine), "input": det.m.size, "mode": a.mode,
            "hz": a.hz if a.mode == "paced" else None, "period_ms": period_ms if a.mode == "paced" else None,
            "device": gr.device, "load_s": round(t_load, 2), "masks": not a.no_masks,
            "cuda_sync": a.cuda_sync, "cuda_ctx": cuda_ctx_flags(cu, cudev), "conf": a.conf, "iou": a.iou}
    if win is not None and stats:
        wall = t_stop - win[0]
        summ.update({"window_s": round(wall, 3), "frames": len(stats), "fps": round(len(stats) / wall, 2),
                     "camera_frames_delivered": delivered - win[2],
                     "skipped_scheduled": sum(r["skipped"] for r in stats),
                     "late": sum(r["late"] for r in stats),
                     "proc_cpu_pct_one_core": round(100 * (cpu - win[1]) / wall, 1),
                     "cpu_ms_per_frame": round(1000 * (cpu - win[1]) / len(stats), 2),
                     "rss_mb": int(open("/proc/self/status").read().split("VmRSS:")[1].split()[0]) // 1024})
        for k in ("e2e_ms", "proc_ms", "pre_ms", "infer_ms", "post_ms", "wait_ms"):
            v = [r[k] for r in stats]
            summ[k] = {"median": round(pct(v, .5), 3), "p95": round(pct(v, .95), 3), "max": round(max(v), 3)}
        summ["dets_per_frame_median"] = pct([len(r["dets"]) for r in stats], .5)
    js = json.dumps(summ)
    print(js)
    if a.summary:
        open(a.summary, "w").write(js + "\n")


if __name__ == "__main__":
    main()
