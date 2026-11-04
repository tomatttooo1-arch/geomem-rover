"""Deterministic input scenes; CPU native M7 voxel core + async host replay.

Run `python -m tools.m7_replay --output runs/m7-start-20260921/host-NAME`
after a code freeze. The directory must not already exist. This is a host
reference check, not FPGA timing, an FPGA overlay, Nav2 or memory integration.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import subprocess

import numpy as np

from host.m7_native import NativeM7, compile_native
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)


ROOT = Path(__file__).resolve().parents[1]
GRID = GridSpec((-2., -2., -1.), .25, (16, 16, 20))
BOUNDS = np.array([[-.25, -.25, 1.5], [.25, .25, 2.5]])


def fixture(seq, kind):
    stamp = 1_000_000_000 + seq * 10_000_000
    identity = CaptureIdentity("gemini-replay-1", seq, stamp, "camera-sync-1",
                               "depth-25-v1", "odom", "odom-1")
    depth = np.full((25, 25), 3., np.float32)
    if kind == "present":
        depth[10:15, 10:15] = 2.
    elif kind == "occluded":
        depth[10:15, 10:15] = 1.
    elif kind == "invalid_depth":
        depth[:] = np.nan
    elif kind != "removed":
        raise ValueError("unknown fixture")
    k = np.array([[12., 0., 12.], [0., 12., 12.], [0., 0., 1.]])
    pose = PoseAtCapture(stamp, identity.clock_id, "odom", "odom-1", True, np.eye(4))
    return DepthFrame(identity, depth, k, pose)


def result_dict(item):
    result = {"identity": asdict(item.identity), "valid": item.valid,
              "age_ns": item.age_ns, "expires_at_ns": item.expires_at_ns,
              "backend": item.backend, "reason": item.reason}
    payload = dict(item.payload)
    if "voxel_flags" in payload:
        voxels = payload.pop("voxel_flags")
        payload["voxel_sha256"] = hashlib.sha256(voxels.tobytes()).hexdigest()
    if "grid" in payload:
        payload["grid"] = asdict(payload["grid"])
    result["payload"] = payload
    return result


def run(backend):
    engine = M7Pipeline(backend, GRID)
    results = {}
    live_geometry = {}
    delayed_present = None
    mask = np.zeros((25, 25), dtype=np.bool_)
    mask[10:15, 10:15] = True
    for seq, kind in enumerate(("present", "removed", "occluded", "invalid_depth"), 1):
        f = fixture(seq, kind)
        stamp = f.identity.acquired_ns
        geometry = engine.ingest_geometry(f, now_ns=stamp + 2_000_000,
                                          now_clock_id=f.identity.clock_id)
        row = {"geometry": result_dict(geometry.navigation),
               "display": result_dict(geometry.display)}
        if geometry.navigation.valid:
            live_geometry[kind] = geometry.navigation
        if kind == "present":
            delayed_present = (f, geometry)
        if geometry.navigation.valid:
            if kind == "removed":
                original_frame, original_geometry = delayed_present
                delayed = engine.ingest_semantic(SemanticPacket(original_frame.identity,
                    original_geometry.navigation.frame_token, "cup-1", mask.copy(),
                    BOUNDS.copy()), now_ns=stamp + 3_000_000,
                    now_clock_id=f.identity.clock_id)
                results["present"]["object_evidence"] = result_dict(delayed)
                results["present"]["late_original_frame_evidence"] = result_dict(delayed)
            if kind != "present":
                pkt = SemanticPacket(f.identity, geometry.navigation.frame_token, "cup-1",
                                     np.zeros_like(mask), BOUNDS.copy())
                evidence = engine.ingest_semantic(pkt, now_ns=stamp + 4_000_000,
                                                  now_clock_id=f.identity.clock_id)
                row["object_evidence"] = result_dict(evidence)
        results[kind] = row
    f = fixture(5, "removed")
    geometry = engine.ingest_geometry(f, now_ns=f.identity.acquired_ns + 1,
                                      now_clock_id=f.identity.clock_id)
    missing = engine.ingest_semantic(SemanticPacket(f.identity, geometry.navigation.frame_token,
                                                   "cup-1", None, BOUNDS.copy()),
                                     now_ns=f.identity.acquired_ns + 2,
                                     now_clock_id=f.identity.clock_id)
    results["missing_mask"] = result_dict(missing)
    wrong = fixture(6, "removed")
    bad_mask = engine.ingest_semantic(SemanticPacket(wrong.identity,
        geometry.navigation.frame_token, "cup-1", mask),
        now_ns=wrong.identity.acquired_ns + 1, now_clock_id=wrong.identity.clock_id)
    results["wrong_frame"] = result_dict(bad_mask)
    stopped_ns = wrong.identity.acquired_ns + engine.max_frame_age_ns + 1
    for kind, previous in live_geometry.items():
        results[kind]["navigation_recheck_after_stop"] = result_dict(
            previous.validate_at(engine, stopped_ns, f.identity.clock_id))
    engine.reset()
    results["old_semantic_after_reset"] = result_dict(engine.ingest_semantic(
        SemanticPacket(f.identity, geometry.navigation.frame_token, "cup-2", mask),
        now_ns=stopped_ns, now_clock_id=f.identity.clock_id))
    expected = {"present": "support_present", "removed": "free_space_conflict",
                "occluded": "indeterminate"}
    for case, state in expected.items():
        assert results[case]["object_evidence"]["payload"]["candidate"] == state, case
    assert results["present"]["geometry"]["payload"]["hit_voxels"] > 0
    assert results["present"]["geometry"]["payload"]["free_voxels"] > 0
    for key in ("invalid_depth", "wrong_frame", "old_semantic_after_reset"):
        output = results[key]["geometry"] if key == "invalid_depth" else results[key]
        assert output["valid"] is False, key
    assert results["missing_mask"]["payload"]["candidate"] == "indeterminate"
    assert results["present"]["late_original_frame_evidence"]["payload"]["position_local_m"][2] == 2.
    assert all(results[x]["navigation_recheck_after_stop"]["valid"] is False for x in expected)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path, help="new isolated result directory")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=False, exist_ok=False)
    library = compile_native(output / "m7.so")
    evidence = run(NativeM7(library))
    inputs = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in
              [ROOT / "tools/m7_replay.py", ROOT / "host/m7_pipeline.py",
               ROOT / "host/m7_native.py", ROOT / "hw/m7/ray_flags.cpp",
               ROOT / "hw/geometry/geometry.cpp"]}
    report = {"role": "CPU host reference, no FPGA/board/Nav2/memory/display pixel integration",
              "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"],
                                                       cwd=ROOT, text=True).strip(),
              "python": platform.python_version(),
              "gxx": subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
              "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
              "source_sha256": inputs,
              "grid": asdict(GRID),
              "config": {"max_frame_age_ns": 200_000_000, "max_cache_age_ns": 500_000_000,
                         "max_pose_age_ns": 50_000_000, "free_fraction": .9,
                         "min_valid_fraction": .7, "min_pixels": 9, "depth_margin_m": .04},
              "cases": evidence}
    (output / "replay.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"result": "PASS", "artifact": str(output / "replay.json"),
                      "cases": list(evidence)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
