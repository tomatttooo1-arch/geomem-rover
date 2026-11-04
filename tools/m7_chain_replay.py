"""Deterministic CPU replay of M7 frame, object evidence and display pixels.

This writes actual native-kernel RGB888 PNGs and a JSON provenance summary.
Run after freezing all three backend sources. It is not a camera, DMA, video,
board, real detector, or navigation test. Output directory must be new.
"""

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import zlib

import numpy as np

from host.m7_display_native import NativeM7Display, compile_native as compile_display
from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_object_native import NativeObjectEvidence, compile_object_native
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)


ROOT = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT = 320, 200
GRID = GridSpec((-2., -2., -1.), .25, (16, 16, 20))
BOUNDS = np.array([[-.25, -.25, 1.5], [.25, .25, 2.5]])
SOURCE_FILES = (
    "hw/m7/frame_depth.cpp", "hw/m7/object_evidence.cpp",
    "hw/m7/display_overlay.cpp", "host/m7_frame_native.py",
    "host/m7_object_native.py", "host/m7_display_native.py",
    "host/m7_pipeline.py", "tools/m7_chain_replay.py",
)


def source_hashes():
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in SOURCE_FILES}


def fixture(sequence, kind):
    timestamp = 1_000_000_000 + sequence * 10_000_000
    identity = CaptureIdentity("synthetic-camera-a", sequence, timestamp,
                               "camera-clock", "calibration-a", "odom", "pose-a")
    depth = np.full((HEIGHT, WIDTH), 3., np.float32)
    if kind == "present":
        depth[92:107, 152:167] = 2.
    elif kind == "occluded":
        depth[:] = 1.
    elif kind == "invalid_depth":
        depth[:] = np.nan
    elif kind != "removed":
        raise ValueError("unknown fixed scene")
    intrinsics = np.array([[120., 0., 159.5], [0., 120., 99.5], [0., 0., 1.]])
    pose = PoseAtCapture(timestamp, identity.clock_id, identity.local_frame_id,
                         identity.pose_epoch, True, np.eye(4))
    return DepthFrame(identity, depth, intrinsics, pose)


def png_chunk(kind, data):
    payload = kind + data
    return (struct.pack(">I", len(data)) + payload +
            struct.pack(">I", zlib.crc32(payload) & 0xffffffff))


def write_actual_png(path, rgb888):
    """Encode the kernel's RGB888 output, with no synthetic artwork."""
    h, w = rgb888.shape
    bytes_rgb = np.empty((h, w, 3), dtype=np.uint8)
    bytes_rgb[:, :, 0] = (rgb888 >> 16) & 255
    bytes_rgb[:, :, 1] = (rgb888 >> 8) & 255
    bytes_rgb[:, :, 2] = rgb888 & 255
    raw = b"".join(b"\0" + bytes_rgb[y].tobytes() for y in range(h))
    png = (b"\x89PNG\r\n\x1a\n" +
           png_chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) +
           png_chunk(b"IDAT", zlib.compress(raw, level=9)) +
           png_chunk(b"IEND", b""))
    path.write_bytes(png)
    return hashlib.sha256(bytes_rgb.tobytes()).hexdigest()


def object_packet(frame, geometry, object_id, mask):
    return SemanticPacket(frame.identity, geometry.display.frame_token,
                          object_id, mask, BOUNDS.copy())


def geometry_counts(frame_backend, frame, geometry):
    identity = frame.identity
    raw = frame_backend.frame_from_depth(
        depth_m=frame.depth_m, k=frame.K,
        camera_to_world=frame.pose.T_local_camera,
        grid_origin_m=GRID.origin_m, voxel_size_m=GRID.voxel_m,
        shape=GRID.shape, frame_id=identity.frame_id,
        capture_time_ns=identity.acquired_ns,
        pose_time_ns=frame.pose.stamp_ns,
        calibration_version=identity.calibration_id, pose_confirmed=True)
    counts = {
        "source": "same_input_native_recheck_not_pipeline_telemetry",
        "published_payload_compared": bool(geometry.navigation.valid),
        "input_pixels": int(frame.depth_m.size),
        "accepted_rays": raw.accepted_rays,
        "invalid_depth_or_projection": raw.invalid_depth_or_projection,
        "outside_grid": raw.outside_grid,
        "published": bool(geometry.navigation.valid),
        "published_reason": geometry.navigation.reason,
    }
    assert (counts["input_pixels"] == counts["accepted_rays"] +
            counts["invalid_depth_or_projection"] + counts["outside_grid"])
    if geometry.navigation.valid:
        payload = geometry.navigation.payload
        assert (counts["input_pixels"] == payload["input_pixels"] and
                counts["accepted_rays"] == payload["valid_grid_samples"] and
                counts["invalid_depth_or_projection"] == payload["invalid_depth_or_projection"] and
                counts["outside_grid"] == payload["outside_grid"])
    return counts


def semantic_row(semantic):
    if semantic is None:
        return None
    payload = semantic.payload
    return {
        "valid_at_production": bool(semantic.valid),
        "source_frame_id": semantic.identity.frame_id,
        "source_capture_ns": semantic.identity.acquired_ns,
        "candidate": payload.get("candidate"),
        "reason": semantic.reason,
        "position_local_m": payload.get("position_local_m"),
        "kernel_counts": payload.get("kernel_counts"),
    }


def display_row(name, output_dir, display, pipeline, frame, geometry, now_ns,
                counts, semantic=None):
    pixels = display.render_consumer_output(
        pipeline, frame, geometry.display, now_ns=now_ns,
        clock_id=frame.identity.clock_id, semantic_output=semantic)
    png_path = output_dir / (name + ".png")
    actual_rgb_sha = write_actual_png(png_path, pixels.rgb888)
    return {
        "identity": asdict(frame.identity),
        "now_ns": int(now_ns),
        "actual_age_ns": int(now_ns - frame.identity.acquired_ns),
        "geometry_counts": counts,
        "semantic_input": semantic_row(semantic),
        "display_valid_at_production": bool(geometry.display.valid),
        "display_reason": pixels.reason,
        "frame_fresh": bool(pixels.fresh),
        "nearest_optical_z_mm": pixels.nearest_optical_z_mm,
        "object_evidence_code": pixels.object_evidence,
        "risk_status": "UNKNOWN_not_computed",
        "valid_depth_pixels": pixels.valid_depth_pixels,
        "invalid_depth_pixels": pixels.invalid_depth_pixels,
        "rgb_sha256": actual_rgb_sha,
        "png": png_path.name,
        "png_sha256": hashlib.sha256(png_path.read_bytes()).hexdigest(),
    }


def run(output_dir, source_commit, source_at_start):
    frame_backend = FrameNativeM7(compile_frame_native(output_dir / "libframe.so"))
    object_backend = NativeObjectEvidence(compile_object_native(output_dir / "libobject.so"))
    display = NativeM7Display(compile_display(output_dir / "libdisplay.so"))
    pipeline = M7Pipeline(frame_backend, GRID, object_backend=object_backend)
    rows = {}
    mask = np.zeros((HEIGHT, WIDTH), dtype=np.bool_)
    mask[92:107, 152:167] = True
    empty = np.zeros_like(mask)

    present = fixture(1, "present")
    geometry1 = pipeline.ingest_geometry(present,
        now_ns=present.identity.acquired_ns+2_000_000,
        now_clock_id=present.identity.clock_id)
    assert geometry1.display.valid
    counts1 = geometry_counts(frame_backend, present, geometry1)
    semantic1 = pipeline.ingest_semantic(object_packet(present, geometry1, "cup", mask),
        now_ns=present.identity.acquired_ns+3_000_000,
        now_clock_id=present.identity.clock_id)
    assert semantic1.valid and semantic1.payload["candidate"] == "support_present"
    rows["present"] = display_row("present", output_dir, display, pipeline,
        present, geometry1, present.identity.acquired_ns+4_000_000,
        counts1, semantic1)
    assert rows["present"]["object_evidence_code"] == 1
    assert rows["present"]["nearest_optical_z_mm"] == 2000

    removed = fixture(2, "removed")
    geometry2 = pipeline.ingest_geometry(removed,
        now_ns=removed.identity.acquired_ns+2_000_000,
        now_clock_id=removed.identity.clock_id)
    assert geometry2.display.valid
    counts2 = geometry_counts(frame_backend, removed, geometry2)
    late = pipeline.ingest_semantic(object_packet(present, geometry1, "cup-late", mask),
        now_ns=removed.identity.acquired_ns+3_000_000,
        now_clock_id=removed.identity.clock_id)
    assert late.valid and late.payload["candidate"] == "support_present"
    assert abs(late.payload["position_local_m"][2] - 2.) < 1e-6
    rows["new_frame_old_memory"] = display_row("new_frame_old_memory", output_dir,
        display, pipeline, removed, geometry2, removed.identity.acquired_ns+4_000_000,
        counts2, late)
    assert rows["new_frame_old_memory"]["frame_fresh"]
    assert rows["new_frame_old_memory"]["object_evidence_code"] == 0
    rows["old_display_revoked"] = display_row("old_display_revoked", output_dir,
        display, pipeline, present, geometry1, removed.identity.acquired_ns+4_000_000,
        counts1, late)
    assert not rows["old_display_revoked"]["frame_fresh"]
    assert rows["old_display_revoked"]["nearest_optical_z_mm"] is None
    semantic2 = pipeline.ingest_semantic(object_packet(removed, geometry2, "cup", empty),
        now_ns=removed.identity.acquired_ns+5_000_000,
        now_clock_id=removed.identity.clock_id)
    assert semantic2.valid and semantic2.payload["candidate"] == "free_space_conflict"
    rows["free_space_conflict"] = display_row("free_space_conflict", output_dir,
        display, pipeline, removed, geometry2, removed.identity.acquired_ns+6_000_000,
        counts2, semantic2)
    assert rows["free_space_conflict"]["object_evidence_code"] == 2

    occluded = fixture(3, "occluded")
    geometry3 = pipeline.ingest_geometry(occluded,
        now_ns=occluded.identity.acquired_ns+2_000_000,
        now_clock_id=occluded.identity.clock_id)
    assert geometry3.display.valid
    counts3 = geometry_counts(frame_backend, occluded, geometry3)
    semantic3 = pipeline.ingest_semantic(object_packet(occluded, geometry3, "cup", empty),
        now_ns=occluded.identity.acquired_ns+3_000_000,
        now_clock_id=occluded.identity.clock_id)
    assert semantic3.valid and semantic3.payload["candidate"] == "indeterminate"
    rows["occluded_unknown"] = display_row("occluded_unknown", output_dir,
        display, pipeline, occluded, geometry3, occluded.identity.acquired_ns+4_000_000,
        counts3, semantic3)
    assert rows["occluded_unknown"]["object_evidence_code"] == 0

    invalid = fixture(4, "invalid_depth")
    geometry4 = pipeline.ingest_geometry(invalid,
        now_ns=invalid.identity.acquired_ns+2_000_000,
        now_clock_id=invalid.identity.clock_id)
    assert not geometry4.display.valid
    counts4 = geometry_counts(frame_backend, invalid, geometry4)
    rows["invalid_depth"] = display_row("invalid_depth", output_dir,
        display, pipeline, invalid, geometry4, invalid.identity.acquired_ns+3_000_000,
        counts4)
    assert not rows["invalid_depth"]["frame_fresh"]
    assert rows["invalid_depth"]["nearest_optical_z_mm"] is None
    rows["expired_after_stop"] = display_row("expired_after_stop", output_dir,
        display, pipeline, occluded, geometry3, geometry3.display.expires_at_ns+1,
        counts3)
    assert not rows["expired_after_stop"]["frame_fresh"]
    assert rows["expired_after_stop"]["nearest_optical_z_mm"] is None

    event_order = list(rows)
    assert all(rows[event_order[i]]["now_ns"] <= rows[event_order[i+1]]["now_ns"]
               for i in range(len(event_order)-1))
    return {
        "event_order": event_order,
        "source_commit_at_start": source_commit,
        "source_sha256_at_start": source_at_start,
        "input_type": "fixed_synthetic_depth_and_masks_not_camera_or_detector",
        "width": WIDTH, "height": HEIGHT,
        "backend": {
            "geometry": "FrameNativeM7_C++_CPU",
            "object_counts_projection": "NativeObjectEvidence_C++_CPU",
            "object_median_historical_ROI": "M7Pipeline_CPU",
            "display_pixels": "NativeM7Display_C++_CPU",
        },
        "scenes": rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    output_dir = args.output.resolve()
    if output_dir.exists():
        parser.error("output directory already exists")
    source_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=str(ROOT), universal_newlines=True).strip()
    at_start = source_hashes()
    output_dir.mkdir(parents=True)
    result = run(output_dir, source_commit, at_start)
    if source_hashes() != at_start:
        raise RuntimeError("source changed during replay; retain run but reject evidence")
    (output_dir / "replay.json").write_text(
        json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False) + "\n")
    files = sorted(path for path in output_dir.iterdir() if path.is_file())
    (output_dir / "MANIFEST.sha256").write_text("".join(
        "%s  %s\n" % (hashlib.sha256(path.read_bytes()).hexdigest(), path.name)
        for path in files))
    print(json.dumps({"scenes": len(result["scenes"]),
                      "source_commit": result["source_commit_at_start"],
                      "output": str(output_dir)}, sort_keys=True))


if __name__ == "__main__":
    main()
