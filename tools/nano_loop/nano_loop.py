#!/usr/bin/env python3
"""P3 live loop on the Orin Nano: Gemini 2 colour + HW-D2C depth -> detector -> M7 CPU reference
-> object evidence -> M4 memory candidates. Fixed camera, identity pose, no motion.

Threads: grabber (newest complete colour+depth set), detector (P2 TensorRT YOLOv8n-seg, one frame
at a time), main (every M7Pipeline call, M4 document building, overlay, ledger). Memory runs in a
separate process (OfflineMemory import is pure Python and grows with the store).

Outputs in --out: ledger.jsonl (type=frame / type=det rows), memory.json (M4 store),
memory_ledger.jsonl, snap/*.jpg, calibration.json, run.json, DONE or FAILED.
With --record-seconds, the first S seconds of processed frames are also written as PNG
(colour, uint16 depth mm) + frames.jsonl + detections.jsonl for offline replay.
"""
import argparse
import hashlib
import json
import multiprocessing as mp
import os
import queue
import sys
import threading
import time
import traceback

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

W, H, CAM_FPS = 640, 360, 30  # 640x360 keeps the 16:9 field of view (fx ~345 px); 640x480 is a narrower 4:3 view


# ---------------------------------------------------------------- memory process
def memory_worker(q_in, q_out, store_path, ledger_path, save_every_s):
    from perception.offline_memory import OfflineMemory
    mem = OfflineMemory()
    last_save = 0.0
    n = 0
    with open(ledger_path, "w", buffering=1) as led:
        while True:
            msg = q_in.get()
            if msg[0] == "doc":
                _, doc, meta = msg
                t0 = time.perf_counter()
                try:
                    r = mem.import_document(doc)
                    decisions = {}
                    for b in r["batches"]:
                        for d in b.get("decisions", []):
                            decisions[d["decision"]] = decisions.get(d["decision"], 0) + 1
                    row = {"ok": True, "objects": r["objects"], "observations": r["observations"],
                           "decisions": decisions}
                except Exception as exc:  # recorded, never silently dropped
                    row = {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)}
                row.update(meta, import_ms=round((time.perf_counter() - t0) * 1e3, 2),
                           lag_ms=round((time.time_ns() - meta["queued_ns"]) / 1e6, 2))
                led.write(json.dumps(row) + "\n")
                n += 1
                if time.time() - last_save > save_every_s:
                    t0 = time.perf_counter()
                    mem.save(store_path)
                    led.write(json.dumps({"save_ms": round((time.perf_counter() - t0) * 1e3, 2)}) + "\n")
                    last_save = time.time()
            elif msg[0] == "stop":
                mem.save(store_path)
                q_out.put({"imports": n, "objects": len(mem.data["objects"]),
                           "observations": len(mem.data["observations"])})
                return


# ---------------------------------------------------------------- camera
def open_camera(attempts=6, wait_s=3.0):
    """HW D2C colour+depth. Depth start on this unit sometimes hangs in STARTING on the
    first pipeline of a process (09-27 probe); a bounded stop/recreate retry is logged."""
    from pyorbbecsdk import (Pipeline, Config, OBSensorType, OBFormat, OBAlignMode,
                             OBFrameAggregateOutputMode, OBPropertyID)
    log = []
    for attempt in range(attempts):
        pipe = Pipeline()
        cp = pipe.get_stream_profile_list(OBSensorType.COLOR_SENSOR).get_video_stream_profile(
            W, H, OBFormat.RGB, CAM_FPS)
        dl = pipe.get_d2c_depth_profile_list(cp, OBAlignMode.HW_MODE)
        dps = [dl.get_stream_profile_by_index(i).as_video_stream_profile() for i in range(dl.get_count())]
        dp = [p for p in dps if p.get_width() == 640 and p.get_fps() == CAM_FPS and "Y16" in str(p.get_format())][0]
        cfg = Config()
        cfg.enable_stream(cp)
        cfg.enable_stream(dp)
        cfg.set_align_mode(OBAlignMode.HW_MODE)
        cfg.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
        t0 = time.time()
        pipe.start(cfg)
        ok = False
        while time.time() - t0 < wait_s:
            fs = pipe.wait_for_frames(100)
            if fs is not None and fs.get_depth_frame() is not None and fs.get_color_frame() is not None:
                ok = True
                break
        log.append({"attempt": attempt, "ok": ok, "s": round(time.time() - t0, 2)})
        if ok:
            dev = pipe.get_device()
            info = dev.get_device_info()
            p = pipe.get_camera_param()
            ri, rd = p.rgb_intrinsic, p.rgb_distortion
            laser = dev.get_int_property(OBPropertyID.OB_PROP_LASER_BOOL)
            d = fs.get_depth_frame()
            meta = {"serial": info.get_serial_number(), "firmware": info.get_firmware_version(),
                    "connection": info.get_connection_type(), "start_attempts": log,
                    "laser_bool_while_streaming": laser, "depth_scale_mm": d.get_depth_scale(),
                    "depth_profile": "%dx%d@%d:%s" % (dp.get_width(), dp.get_height(), dp.get_fps(), dp.get_format()),
                    "color_profile": "%dx%d@%d:RGB" % (W, H, CAM_FPS),
                    "K": [[ri.fx, 0.0, ri.cx], [0.0, ri.fy, ri.cy], [0.0, 0.0, 1.0]],
                    "intrinsic_wh": [ri.width, ri.height],
                    "rgb_distortion_not_applied": [rd.k1, rd.k2, rd.p1, rd.p2, rd.k3]}
            return pipe, meta
        pipe.stop()
        del pipe
        time.sleep(1.0)
    raise RuntimeError("camera depth stream did not start: %s" % log)


class Grabber(threading.Thread):
    def __init__(self, pipe):
        super().__init__(daemon=True)
        self.pipe = pipe
        self.cond = threading.Condition()
        self.latest = None
        self.stop_flag = False
        self.c = {"delivered": 0, "timeouts": 0, "incomplete": 0, "color_index_gaps": 0,
                  "overwritten_untaken": 0}
        self.last_ci = None

    def run(self):
        while not self.stop_flag:
            fs = self.pipe.wait_for_frames(100)
            if fs is None:
                self.c["timeouts"] += 1
                continue
            cf, df = fs.get_color_frame(), fs.get_depth_frame()
            if cf is None or df is None:
                self.c["incomplete"] += 1
                continue
            ci = cf.get_index()
            if self.last_ci is not None and ci > self.last_ci + 1:
                self.c["color_index_gaps"] += ci - self.last_ci - 1
            self.last_ci = ci
            rgb = np.frombuffer(cf.get_data(), dtype=np.uint8).reshape(H, W, 3).copy()
            depth = np.frombuffer(df.get_data(), dtype=np.uint16).reshape(df.get_height(), df.get_width()).copy()
            rec = {"ci": ci, "di": df.get_index(), "dev_c_us": cf.get_timestamp_us(),
                   "dev_d_us": df.get_timestamp_us(), "sys_us": cf.get_system_timestamp_us(),
                   "grab_ns": time.time_ns(), "rgb": rgb, "depth": depth}
            with self.cond:
                if self.latest is not None and self.latest["ci"] % 2 == 0:
                    self.c["overwritten_untaken"] += 1
                self.latest = rec
                self.c["delivered"] += 1
                self.cond.notify_all()

    def take(self, pred, timeout):
        end = time.time() + timeout
        with self.cond:
            while True:
                r = self.latest
                if r is not None and pred(r["ci"]):
                    self.latest = None
                    return r
                left = end - time.time()
                if left <= 0:
                    return None
                self.cond.wait(left)


# ---------------------------------------------------------------- detector thread
class DetectorThread(threading.Thread):
    def __init__(self, engine, conf):
        super().__init__(daemon=True)
        from tools.nano_detector.nano_detector import Detector
        self.det = Detector(engine, W, H, conf)
        self.jobs = queue.Queue(maxsize=1)
        self.done = queue.Queue()
        self.busy = False
        self.stop_flag = False

    def submit(self, ci, rgb):
        self.busy = True
        self.jobs.put((ci, rgb, time.time_ns()))

    def run(self):
        while not self.stop_flag:
            try:
                ci, rgb, t_sub = self.jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            t0 = time.perf_counter()
            self.det.preprocess(rgb)
            t1 = time.perf_counter()
            self.det.m.run()
            t2 = time.perf_counter()
            dets = self.det.postprocess(masks=True)
            t3 = time.perf_counter()
            self.done.put((ci, dets, {"queue_ms": round((time.time_ns() - t_sub) / 1e6 - (t3 - t0) * 1e3, 3),
                                      "pre_ms": round((t1 - t0) * 1e3, 3), "infer_ms": round((t2 - t1) * 1e3, 3),
                                      "post_ms": round((t3 - t2) * 1e3, 3), "done_ns": time.time_ns()}))
            self.busy = False


# ---------------------------------------------------------------- overlay
def draw_overlay(path, rgb, res, acquired_ns, backend):
    import cv2
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    age_ms = (time.time_ns() - acquired_ns) / 1e6
    colors = {"support_present": (0, 220, 0), "indeterminate": (0, 200, 255), "rejected": (0, 0, 255)}
    for row in res["instances"]:
        x0, y0, x1, y1 = [int(v) for v in row["box"]]
        col = colors.get(row["candidate"], (255, 255, 255))
        cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
        lines = ["%s %.2f" % (row["label"], row["score"])]
        if row.get("position_m"):
            lines.append("xyz %.2f %.2f %.2f m" % tuple(row["position_m"]))
        lines.append(row["candidate"] if row["candidate"] == "support_present" else
                     "%s:%s" % (row["candidate"], row["reason"][:24]))
        inside = y0 - 16 * len(lines) < 22   # no room above the box: write inside it
        for k, t in enumerate(lines):
            y = y0 + 16 * (k + 1) if inside else y0 - 6 - 16 * (len(lines) - 1 - k)
            cv2.putText(img, t, (x0 + 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(img, t, (x0 + 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1)
    head = "frame %d  age %.0f ms  %s  fixed camera (identity pose)" % (res["color_index"], age_ms, backend)
    cv2.rectangle(img, (0, 0), (W, 20), (0, 0, 0), -1)
    cv2.putText(img, head, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    for k, n in enumerate(res["negatives"][:4]):
        t = "watch %s %s: %s:%s" % (n["watch"], n["label"], n["candidate"], n["reason"])
        cv2.putText(img, t, (4, H - 8 - 16 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 0), 1)
    cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 70])  # already 640 wide


# ---------------------------------------------------------------- recorder
class Recorder(threading.Thread):
    def __init__(self, root):
        super().__init__(daemon=True)
        import cv2
        self.cv2 = cv2
        self.root = root
        os.makedirs(os.path.join(root, "color"), exist_ok=True)
        os.makedirs(os.path.join(root, "depth"), exist_ok=True)
        self.q = queue.Queue(maxsize=64)
        self.frames = open(os.path.join(root, "frames.jsonl"), "w", buffering=1)
        self.dets = open(os.path.join(root, "detections.jsonl"), "w", buffering=1)
        self.dropped = 0
        self.written = 0

    def add(self, meta, rgb, depth):
        try:
            self.q.put_nowait((meta, rgb, depth))
        except queue.Full:
            self.dropped += 1

    def add_dets(self, ci, dets):
        self.dets.write(json.dumps({"color_index": ci, "dets": dets}, separators=(",", ":")) + "\n")

    def run(self):
        cv2 = self.cv2
        while True:
            item = self.q.get()
            if item is None:
                break
            meta, rgb, depth = item
            name = "%06d.png" % meta["color_index"]
            cv2.imwrite(os.path.join(self.root, "color", name), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_PNG_COMPRESSION, 1])
            cv2.imwrite(os.path.join(self.root, "depth", name), depth, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            self.frames.write(json.dumps(dict(meta, color_png="color/" + name, depth_png="depth/" + name)) + "\n")
            self.written += 1

    def close(self):
        self.q.put(None)
        self.join(60)
        self.frames.close()
        self.dets.close()


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--engine", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", type=int, default=900, help="processed frames after warm-up")
    ap.add_argument("--warmup-frames", type=int, default=30)
    ap.add_argument("--hz", type=float, default=15.0)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--snap-every", type=int, default=75)
    ap.add_argument("--memory-interval-s", type=float, default=1.0)
    ap.add_argument("--save-every-s", type=float, default=5.0)
    ap.add_argument("--record-seconds", type=float, default=0.0)
    ap.add_argument("--color", default="640x360", help="colour profile WxH (RGB@30); depth is HW-D2C aligned to it")
    a = ap.parse_args()
    global W, H
    W, H = [int(v) for v in a.color.split("x")]
    out = os.path.abspath(a.out)
    os.makedirs(os.path.join(out, "snap"), exist_ok=False)
    try:
        run(a, out)
        open(os.path.join(out, "DONE"), "w").write(time.strftime("%F %T") + "\n")
    except BaseException:
        open(os.path.join(out, "FAILED"), "w").write(traceback.format_exc())
        raise


def run(a, out):
    # Memory process first: fork before any CUDA context or thread exists.
    ctx = mp.get_context("fork")
    mq_in, mq_out = ctx.Queue(), ctx.Queue()
    mproc = ctx.Process(target=memory_worker, args=(mq_in, mq_out, os.path.join(out, "memory.json"),
                                                     os.path.join(out, "memory_ledger.jsonl"), a.save_every_s))
    mproc.start()

    from tools.nano_detector.nano_detector import set_cuda_sync, Letterbox
    set_cuda_sync("blocking")
    import cv2
    cv2.setNumThreads(1)
    from host.m7_frame_native import FrameNativeM7, compile_frame_native
    from host.m7_object_native import NativeObjectEvidence, compile_object_native
    from tools.nano_loop.chain import FixedCameraChain, calibration_record, canonical_sha

    frame_backend = FrameNativeM7(compile_frame_native(os.path.join(out, "m7_frame.so")))
    object_backend = NativeObjectEvidence(compile_object_native(os.path.join(out, "m7_object.so")))
    engine_sha = hashlib.sha256(open(a.engine, "rb").read()).hexdigest()
    det = DetectorThread(a.engine, a.conf)
    pipe, cam = open_camera()
    if cam["depth_scale_mm"] != 1.0:
        raise RuntimeError("depth scale is not 1 mm: %r" % cam["depth_scale_mm"])
    if cam["intrinsic_wh"] != [W, H]:
        raise RuntimeError("SDK intrinsics are for %r, not the colour stream" % cam["intrinsic_wh"])
    calib = calibration_record(cam["serial"], cam["firmware"], cam["K"], W, H,
                               {"rgb_distortion_not_applied": cam["rgb_distortion_not_applied"],
                                "depth_profile": cam["depth_profile"]})
    json.dump(calib, open(os.path.join(out, "calibration.json"), "w"), indent=1)
    run_name = os.path.basename(out)
    lb = Letterbox(W, H, det.det.m.size)
    chain = FixedCameraChain(frame_backend, object_backend, calib, run_name, letterbox=lb,
                             prediction_set="P2_yolov8n_seg_640_fp16_" + engine_sha[:12],
                             width=W, height=H)
    backend_name = chain.pipeline.backend.__class__.__name__ + "+" + object_backend.__class__.__name__
    json.dump({"camera": cam, "engine": os.path.abspath(a.engine), "engine_sha256": engine_sha,
               "argv": sys.argv, "backend": backend_name, "calibration_id": calib["calibration_id"],
               "start": time.strftime("%F %T")}, open(os.path.join(out, "run.json"), "w"), indent=1)
    gr = Grabber(pipe)
    gr.start()
    det.start()
    rec = Recorder(os.path.join(out, "rec")) if a.record_seconds > 0 else None
    if rec:
        json.dump(calib, open(os.path.join(out, "rec", "calibration.json"), "w"), indent=1)
        rec.start()

    step = max(1, int(round(CAM_FPS / a.hz)))
    led = open(os.path.join(out, "ledger.jsonl"), "w", buffering=1 << 16)
    total = a.frames + a.warmup_frames
    seq = 0
    last_ci = None
    last_import_ns = 0
    pending_snap = set()
    acquired = {}
    rec_until_ns = None
    t_cpu0 = time.process_time()
    t_wall0 = time.time()
    try:
        while seq < total:
            # 1) consume finished detections first (they belong to an older, cached frame)
            while True:
                try:
                    ci_d, dets, dt = det.done.get_nowait()
                except queue.Empty:
                    break
                t0 = time.perf_counter()
                now = time.time_ns()
                res, doc = chain.consume_detections(ci_d, dets, now)
                sem_ms = (time.perf_counter() - t0) * 1e3
                acq_ns = acquired.get(ci_d)
                row = {"type": "det", "color_index": ci_d, "warmup": ci_d in acquired and acquired[ci_d] is None,
                       "late": res["late"], "index_gap": res["frames_behind"], "n_det": len(dets),
                       "det": dt, "sem_ms": round(sem_ms, 3),
                       "result_age_ms": None if acq_ns is None else round((now - acq_ns) / 1e6, 2),
                       "instances": res["instances"], "negatives": res["negatives"], "m4": res["m4_status"]}
                if doc is not None and now - last_import_ns >= a.memory_interval_s * 1e9:
                    mq_in.put(("doc", doc, {"color_index": ci_d, "queued_ns": now}))
                    last_import_ns = now
                    row["m4_sent"] = True
                if rec is not None and rec_until_ns is not None and acq_ns is not None and acq_ns <= rec_until_ns:
                    rec.add_dets(ci_d, dets)
                if ci_d in pending_snap:
                    pending_snap.discard(ci_d)
                    t1 = time.perf_counter()
                    frame = chain.frames.get(ci_d)
                    if frame is not None and frame["keep"] is not None:
                        draw_overlay(os.path.join(out, "snap", "f%06d.jpg" % ci_d), frame["keep"], res,
                                     frame["identity"].acquired_ns, "cpu_reference")
                        frame["keep"] = None
                    row["overlay_ms"] = round((time.perf_counter() - t1) * 1e3, 2)
                led.write(json.dumps(row, separators=(",", ":")) + "\n")
            # 2) newest scheduled frame (short timeout so detections are consumed promptly)
            r = gr.take(lambda i: i % step == 0 and (last_ci is None or i > last_ci), 0.005)
            if r is None:
                continue
            t_loop = time.perf_counter()
            ci = r["ci"]
            skipped = 0 if last_ci is None else (ci - last_ci) // step - 1
            last_ci = ci
            warm = seq < a.warmup_frames
            acquired_ns = r["sys_us"] * 1000
            t0 = time.perf_counter()
            rgb_sha = hashlib.sha256(r["rgb"].tobytes()).hexdigest()
            t1 = time.perf_counter()
            snap = (not warm) and ((seq - a.warmup_frames) % a.snap_every == 0)
            acquired[ci] = None if warm else acquired_ns
            if len(acquired) > 64:
                acquired.pop(min(acquired))
            # Recognition only needs the colour image: start it before geometry so the two overlap.
            # Its result is consumed on this thread after this frame's geometry is cached.
            if det.busy:
                det_state = "skipped:detector_busy"
            else:
                det.submit(ci, r["rgb"])
                det_state = "submitted"
                if snap:
                    pending_snap.add(ci)
            t1g = time.perf_counter()
            frame, gsum = chain.ingest_frame(ci, acquired_ns, r["depth"], time.time_ns(), rgb_sha256=rgb_sha,
                                             keep=r["rgb"] if snap else None)
            t2 = time.perf_counter()
            if rec is not None and not warm:
                if rec_until_ns is None:
                    rec_until_ns = acquired_ns + int(a.record_seconds * 1e9)
                if acquired_ns <= rec_until_ns:
                    rec.add({"seq": seq, "color_index": ci, "depth_index": r["di"], "dev_ts_color_us": r["dev_c_us"],
                             "dev_ts_depth_us": r["dev_d_us"], "acquired_ns": acquired_ns,
                             "rgb_sha256": rgb_sha}, r["rgb"], r["depth"])
            led.write(json.dumps({"type": "frame", "seq": seq, "warmup": warm, "color_index": ci,
                                  "depth_index": r["di"], "dev_ts_color_us": r["dev_c_us"],
                                  "dev_ts_depth_us": r["dev_d_us"], "acquired_ns": acquired_ns,
                                  "recv_to_take_ms": round((time.time_ns() - acquired_ns) / 1e6 - (time.perf_counter() - t_loop) * 1e3, 3),
                                  "skipped_scheduled": skipped, "sha_ms": round((t1 - t0) * 1e3, 3),
                                  "geometry_ms": round((t2 - t1g) * 1e3, 3), "geometry": gsum, "det": det_state,
                                  "loop_ms": round((time.perf_counter() - t_loop) * 1e3, 3)},
                                 separators=(",", ":")) + "\n")
            seq += 1
        # drain the last detection
        time.sleep(0.3)
    finally:
        wall = time.time() - t_wall0
        cpu = time.process_time() - t_cpu0
        gr.stop_flag = True
        det.stop_flag = True
        gr.join(2)
        pipe.stop()
        led.close()
        if rec:
            rec.close()
        mq_in.put(("stop",))
        try:
            mem_summary = mq_out.get(timeout=300)
        except queue.Empty:
            mem_summary = {"error": "memory process did not stop"}
        mproc.join(10)
        json.dump({"wall_s": round(wall, 2), "main_process_cpu_s": round(cpu, 2),
                   "main_process_cpu_pct_one_core": round(100 * cpu / wall, 1),
                   "grabber": gr.c, "memory": mem_summary,
                   "recorder": None if rec is None else {"written": rec.written, "dropped": rec.dropped},
                   "rss_mb": int(open("/proc/self/status").read().split("VmRSS:")[1].split()[0]) // 1024,
                   "end": time.strftime("%F %T")},
                  open(os.path.join(out, "run_end.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
