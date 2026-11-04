"""Board side of line 3 (PREREG.md sections 1-3): YoloSegDetector(backend="trt") on the packaged
rec30 frames, detections + per-call latency + environment.

Runs on the Orin Nano's host-system python3 (TensorRT 10.3, JetPack torch), from run_board.sh:
  PYTHONPATH=<deploy>/src:~/fpga/p2/pylib python3 -m tools.yolo_trt_board.board_run \
      --engine E --frames-dir <deploy>/frames --manifest <deploy>/frames_manifest.json \
      --out OUT --n-frames 39 --passes 5 --warmup 10 --tag full
Outputs in OUT: board_detections.json (pass 1 detections, judged by compare_board.py; passes 2..N
compared bitwise, report only), latency.csv (one row per call), latency_summary.json, board_env.json.

What is timed (perf_counter wall clock): `YoloSegDetector.__call__` end to end (RGB already in
memory -> Detection list), and inside it Detector.preprocess / TRTModel.run / Detector.postprocess,
by wrapping those three bound methods on the instances (timing only, same arguments, same results).
"""
import argparse
import glob
import hashlib
import json
import os
import platform
import subprocess
import sys
import time

import numpy as np


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception as error:          # recorded, not fatal: the shell entry does the hard checks
        return "unavailable: %s" % error


def read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def gpu_devfreq_dir():
    hits = sorted(glob.glob("/sys/class/devfreq/*.gpu")) + sorted(glob.glob("/sys/class/devfreq/*gpu*"))
    return hits[0] if hits else None


def gpu_state(d):
    if d is None:
        return None
    return {k: read(os.path.join(d, k)) for k in ("cur_freq", "min_freq", "max_freq", "governor", "available_frequencies")}


def cpufreq_state():
    """schedutil policies (2026-10-15 rebuild A/B: CPU frequency drifts between whole runs)."""
    rows = {}
    for d in sorted(glob.glob("/sys/devices/system/cpu/cpufreq/policy*")):
        rows[os.path.basename(d)] = {k: read(os.path.join(d, k)) for k in
                                     ("related_cpus", "scaling_governor", "scaling_cur_freq", "scaling_max_freq")}
    return rows


def pct(values, q):
    v = sorted(values)
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else None


class Timer:
    """Wraps a bound method; appends its wall time (ms) to self.last."""
    def __init__(self, fn):
        self.fn, self.last = fn, None

    def __call__(self, *args, **kwargs):
        t0 = time.perf_counter()
        try:
            return self.fn(*args, **kwargs)
        finally:
            self.last = (time.perf_counter() - t0) * 1e3


def engine_io(m):
    """Name / shape / dtype / mode of every engine IO tensor (build log said 5 output network tensors)."""
    eng = m.engine
    rows = []
    for i in range(eng.num_io_tensors):
        n = eng.get_tensor_name(i)
        rows.append({"name": n, "shape": [int(v) for v in eng.get_tensor_shape(n)],
                     "dtype": str(eng.get_tensor_dtype(n)), "mode": str(eng.get_tensor_mode(n))})
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", required=True)
    ap.add_argument("--frames-dir", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--n-frames", type=int, default=39, help="first N frames of the manifest (5 for the d5 run)")
    ap.add_argument("--passes", type=int, default=5)
    ap.add_argument("--warmup", type=int, default=10, help="calls excluded from latency statistics")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--package-info", help="PACKAGE_INFO.json of the deployed package (recorded)")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    t_start = time.time()

    manifest = json.load(open(a.manifest))
    frames = manifest["frames"][:a.n_frames]
    if len(frames) != a.n_frames:
        raise SystemExit("manifest has %d frames, %d requested" % (len(manifest["frames"]), a.n_frames))
    bad = [n for n in frames if sha256_file(os.path.join(a.frames_dir, n)) != manifest["sha256"][n]]
    if bad:
        raise SystemExit("frame sha256 mismatch vs manifest: %s" % bad)

    engine_sha = sha256_file(a.engine)
    env = {"tag": a.tag, "started_epoch": t_start, "argv": sys.argv if argv is None else argv,
           "engine": os.path.abspath(a.engine), "engine_sha256": engine_sha,
           "python": sys.version, "platform": platform.platform(),
           "nvpmodel_q": sh("nvpmodel -q 2>&1"), "l4t": read("/etc/nv_tegra_release"),
           "uptime": sh("uptime"), "top": sh("top -bn1 | head -20"),
           "gpu_devfreq_dir": gpu_devfreq_dir()}
    env["gpu_before"] = gpu_state(env["gpu_devfreq_dir"])
    env["cpu_affinity"] = sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None
    env["cpufreq_before"] = cpufreq_state()
    if a.package_info and os.path.isfile(a.package_info):
        env["package_info"] = json.load(open(a.package_info))

    # Construct first: YoloSegDetector(trt) sets the CUDA primary-context flags before any CUDA use.
    from perception.fpga_replay.core import Frame
    from perception.fpga_replay.yolo_detector import YoloSegDetector
    from tools.nano_detector import nano_detector as nd
    t0 = time.perf_counter()
    det = YoloSegDetector(a.engine, backend="trt", conf=0.25, iou=0.7, classes=None, max_det=100)
    import cv2
    first = cv2.imread(os.path.join(a.frames_dir, frames[0]), cv2.IMREAD_COLOR)
    det._letterbox(first.shape[1], first.shape[0])     # builds nd.Detector (engine load) for this frame size
    env["load_s"] = round(time.perf_counter() - t0, 3)
    board = det._board
    env["fake_trt"] = bool(getattr(nd, "FAKE_TRT", False))
    import perception.fpga_replay.yolo_detector as yd_mod
    env["module_files"] = {"yolo_detector": os.path.abspath(yd_mod.__file__), "nano_detector": os.path.abspath(nd.__file__),
                           "board_run": os.path.abspath(__file__), "cwd": os.getcwd()}
    env["execution"] = det.execution
    env["engine_io"] = engine_io(board.m)
    env["detector_outputs_used"] = [board.out_det, board.out_proto]
    env["letterbox"] = {"W": board.lb.W, "H": board.lb.H, "S": board.lb.S, "nw": board.lb.nw, "nh": board.lb.nh,
                        "left": board.lb.left, "top": board.lb.top}
    try:
        import tensorrt
        env["tensorrt"] = tensorrt.__version__
    except Exception as error:
        env["tensorrt"] = "unavailable: %s" % error
    try:
        import torch
        env["torch"] = torch.__version__
        env["torch_threads"] = torch.get_num_threads()
        env["cuda_device"] = torch.cuda.get_device_name(0) if torch.cuda.is_available() else None
    except Exception as error:
        env["torch"] = "unavailable: %s" % error
    env["cv2"] = cv2.__version__
    env["cv2_threads"] = cv2.getNumThreads()
    env["cuda_ctx"] = nd.cuda_ctx_flags(*det.cuda) if det.cuda else None

    t_pre, t_inf, t_post = Timer(board.preprocess), Timer(board.m.run), Timer(board.postprocess)
    board.preprocess, board.m.run, board.postprocess = t_pre, t_inf, t_post
    gdir = env["gpu_devfreq_dir"]
    images = {}
    for n in frames:
        bgr = cv2.imread(os.path.join(a.frames_dir, n), cv2.IMREAD_COLOR)
        images[n] = np.ascontiguousarray(bgr[:, :, ::-1])
    rows, per_pass = [], []
    call = 0
    for p in range(a.passes):
        dets_pass = {}
        for i, n in enumerate(frames):
            rgb = images[n]
            h, w = rgb.shape[:2]
            fr = Frame(frame_id=n, timestamp=float(i), rgb=rgb, depth=np.ones((h, w), np.float32),
                       K=np.array([[500., 0, w / 2], [0, 500., h / 2], [0, 0, 1]]), T_world_camera=np.eye(4))
            t_pre.last = t_inf.last = t_post.last = None
            c0 = time.perf_counter()
            out = det(fr)
            call_ms = (time.perf_counter() - c0) * 1e3
            freq = read(os.path.join(gdir, "cur_freq")) if gdir else None
            dets_pass[n] = [{"name": d.label, "score": d.score, "box": list(d.bbox)} for d in out]
            rows.append({"call": call, "pass": p, "index": i, "frame": n, "warmup": call < a.warmup,
                         "pre_ms": t_pre.last, "infer_ms": t_inf.last, "post_ms": t_post.last, "call_ms": call_ms,
                         "n_det": len(out), "gpu_cur_freq": freq})
            call += 1
        per_pass.append(dets_pass)
    env["gpu_after"] = gpu_state(gdir)
    env["cpufreq_after"] = cpufreq_state()
    env["finished_epoch"] = time.time()

    mismatch = [{"pass": p, "frame": n} for p in range(1, len(per_pass)) for n in frames
                if per_pass[p][n] != per_pass[0][n]]
    result = {"tag": a.tag, "what": "board YoloSegDetector(trt) detections, pass 1; all 80 classes, conf 0.25 iou 0.7 max_det 100",
              "fake_trt": env["fake_trt"], "engine_sha256": engine_sha, "execution": det.execution,
              "frames": frames, "frame_sha256": {n: manifest["sha256"][n] for n in frames},
              "passes": a.passes, "passes_identical_to_pass1": not mismatch, "pass_mismatches": mismatch,
              "detections": per_pass[0]}
    with open(os.path.join(a.out, "board_detections.json"), "w") as f:
        json.dump(result, f, indent=1)
    keys = ["call", "pass", "index", "frame", "warmup", "pre_ms", "infer_ms", "post_ms", "call_ms", "n_det", "gpu_cur_freq"]
    with open(os.path.join(a.out, "latency.csv"), "w") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join("" if r[k] is None else (("%.4f" % r[k]) if isinstance(r[k], float) else str(r[k]))
                             for k in keys) + "\n")
    timed = [r for r in rows if not r["warmup"]]
    summ = {"tag": a.tag, "fake_trt": env["fake_trt"], "calls": len(rows), "warmup_excluded": a.warmup,
            "samples": len(timed), "layer": "YoloSegDetector.__call__ (RGB in memory -> Detection list); "
            "pre/infer/post = Detector.preprocess / TRTModel.run / Detector.postprocess(masks=False)"}
    if timed:
        for k in ("pre_ms", "infer_ms", "post_ms", "call_ms"):
            v = [r[k] for r in timed if r[k] is not None]
            summ[k] = {"median": round(pct(v, .5), 3), "p95": round(pct(v, .95), 3), "max": round(max(v), 3)} if v else None
        freqs = [r["gpu_cur_freq"] for r in timed if r["gpu_cur_freq"]]
        summ["gpu_cur_freq_hist"] = {f: freqs.count(f) for f in sorted(set(freqs))}
    else:
        summ["note"] = "no samples after warm-up (d5 run: detections only)"
    with open(os.path.join(a.out, "latency_summary.json"), "w") as f:
        json.dump(summ, f, indent=1)
    with open(os.path.join(a.out, "board_env.json"), "w") as f:
        json.dump(env, f, indent=1)
    print(json.dumps({"tag": a.tag, "fake_trt": env["fake_trt"], "frames": len(frames), "passes": a.passes,
                      "detections_pass1": sum(len(v) for v in per_pass[0].values()),
                      "passes_identical_to_pass1": not mismatch, "latency": summ}, indent=1))


if __name__ == "__main__":
    main()
