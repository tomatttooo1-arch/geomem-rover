"""Run from repository root: python -m host.replay --help."""
import argparse
from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import cv2
import numpy as np
from perception.fpga_replay.core import Config, Memory, compute_geometry, process_frame
from perception.fpga_replay.io import load_frame, load_records, sha256, tum_records
from perception.fpga_replay.synthetic import scenarios, scene


def runtime_versions():
    """Imported code is authoritative; system OpenCV need not have pip metadata."""
    packages = {"numpy": "numpy", "opencv-python": "cv2", "torch": "torch", "torchvision": "torchvision"}
    distributions = {}
    modules = {}
    for package, module_name in packages.items():
        module = importlib.import_module(module_name)
        try:
            distributions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            distributions[package] = None
        modules[module_name] = {"version": module.__version__, "file": module.__file__}
    return distributions, modules


def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+"\n")


def evidence_image(frame, detections, tracks, path):
    rgb = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
    for d in detections:
        x1, y1, x2, y2 = map(int, d.bbox)
        cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 1)
        cv2.putText(rgb, d.label, (max(0, x1), max(15, y1)), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 0), 1)
    scaled = np.nan_to_num(np.clip(frame.depth/4, 0, 1), nan=0)
    depth = cv2.applyColorMap((scaled*255).astype(np.uint8), cv2.COLORMAP_TURBO)
    depth[~np.isfinite(frame.depth)] = 0
    panel = np.full((max(rgb.shape[0], 36+20*len(tracks)), 560, 3), 245, dtype=np.uint8)
    cv2.putText(panel, "Memory (world metres); software replay", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1)
    for row, track in enumerate(tracks):
        xyz = ",".join(f"{v:.2f}" for v in track["position_m"])
        text = f"{track['id']} {track['label']} {track['state']} ({xyz})"
        cv2.putText(panel, text, (10, 45+20*row), cv2.FONT_HERSHEY_SIMPLEX, .36, (0, 0, 0), 1)
    input_panel = np.concatenate([rgb, depth], axis=1)
    combined = np.full((max(panel.shape[0], rgb.shape[0]), input_panel.shape[1]+560, 3), 245, np.uint8)
    combined[:input_panel.shape[0], :input_panel.shape[1]] = input_panel
    combined[:panel.shape[0], input_panel.shape[1]:] = panel
    if not cv2.imwrite(str(path), combined):
        raise IOError(f"cannot write {path}")


def run_sequence(frames, out, query):
    memories = {name: Memory(name) for name in ("static", "miss", "geometry")}
    all_events = []; query_time = None
    for i, (frame, detections) in enumerate(frames):
        observations = process_frame(frame, detections)
        if query_time is None:
            query_time = frame.timestamp  # command issued immediately after first acquisition
        event = {"frame_id": frame.frame_id, "timestamp": frame.timestamp, "pose_source": frame.pose_source,
                 "calibration_id": frame.calibration_id, "detections": [asdict(d) for d in detections],
                 "observation_count": len(observations), "strategies": {}}
        for name, memory in memories.items():
            result = memory.update_memory(observations, compute_geometry(frame, memory), frame)
            result["query"] = memory.query_memory(query, after=query_time)
            event["strategies"][name] = result
        all_events.append(event)
        evidence_image(frame, detections, memories["geometry"].snapshot(), out/f"frame-{i:04d}.png")
    if not all_events:
        raise ValueError("empty replay")
    (out/"events.jsonl").write_text("".join(json.dumps(e, ensure_ascii=False, allow_nan=False)+"\n" for e in all_events))
    return {"frames": len(all_events), "detections": sum(len(e["detections"]) for e in all_events),
            "observations": sum(e["observation_count"] for e in all_events),
            "final": {k: v.snapshot() for k, v in memories.items()}, "query_issued_at": query_time}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["synthetic", "tum", "manifest"])
    p.add_argument("--data", type=Path)
    p.add_argument("--manifest", type=Path)
    p.add_argument("--out", type=Path, required=True, help="new output directory; never overwritten")
    p.add_argument("--models", type=Path)
    p.add_argument("--limit", type=int, default=30)
    p.add_argument("--stride", type=int, default=15)
    p.add_argument("--query", default="找红色杯子")
    p.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    p.add_argument("--memory-format", choices=("contiguous", "channels_last"), default="contiguous")
    p.add_argument("--cuda-math", choices=("fp32", "tf32"), default="fp32")
    args = p.parse_args()
    if args.mode != "synthetic" and (args.data is None or args.models is None):
        p.error("real-data modes require --data and --models")
    if args.mode == "manifest" and args.manifest is None:
        p.error("manifest mode requires --manifest")
    args.out.mkdir(parents=True, exist_ok=False)
    versions, module_versions = runtime_versions()
    metadata = {"command": sys.argv, "python": sys.version, "platform": platform.platform(), "versions": versions,
                "module_versions": module_versions,
                "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
                "git_dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], text=True).strip()),
                "config": asdict(Config()), "mode": args.mode,
                "device": args.device if args.mode != "synthetic" else "synthetic_oracle_box_no_inference",
                "performance_claim": False,
                "memory_format": args.memory_format, "cuda_math": args.cuda_math if args.device == "cuda" else None,
                "query_policy": "issued after first acquisition; confirmation requires a later frame"}
    dump(args.out/"run.json", metadata)
    if args.mode == "synthetic":
        summary = {}
        for name, frames in scenarios():
            target = args.out/name; target.mkdir()
            summary[name] = run_sequence(frames, target, args.query)
        # Diagnostic factorial: same known scene, independently vary pose and detection.
        for disturbed_pose in (False, True):
            for suppressed in (False, True):
                name = f"factor_pose-{int(disturbed_pose)}_miss-{int(suppressed)}"
                target = args.out/name; target.mkdir()
                frames = [scene(0)] + [scene(i, camera_x=0.12*(i-1), pose_error=0.6 if disturbed_pose else 0,
                                               suppress=suppressed) for i in (1, 2)]
                summary[name] = run_sequence(frames, target, args.query)
        dump(args.out/"summary.json", summary)
    else:
        from perception.fpga_replay.detector import SSDDetector
        records = tum_records(args.data, args.stride, args.limit) if args.mode == "tum" else load_records(args.manifest)
        (args.out/"input.jsonl").write_text("".join(json.dumps(r)+"\n" for r in records))
        detector = SSDDetector(args.models, device=args.device, memory_format=args.memory_format, cuda_math=args.cuda_math)
        def frames():
            for record in records:
                frame = load_frame(record, args.data)
                yield frame, detector(frame)
        summary = run_sequence(frames(), args.out, args.query)
        summary["model_files"] = {str(f): sha256(f) for f in sorted(args.models.rglob("*.pth"))}
        summary["input_manifest_sha256"] = sha256(args.out/"input.jsonl")
        dump(args.out/"summary.json", summary)
    hashes = {str(f.relative_to(args.out)): sha256(f) for f in sorted(args.out.rglob("*")) if f.is_file()}
    dump(args.out/"manifest.json", hashes)
    print(json.dumps({"out": str(args.out.resolve()), "mode": args.mode, "complete": True}))


if __name__ == "__main__":
    main()
