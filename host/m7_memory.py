"""Strict, CPU-only bridge from a complete M7 frame to dev3's offline M4 schema.

This module does not infer recognition identity, map revisions, or absence. A
caller must use one complete detector result per RGB acquisition and persist
the returned M4 document through the existing OfflineMemory API.
"""

from copy import deepcopy
from dataclasses import asdict
import math
from pathlib import Path

import numpy as np

from host.m7_pipeline import ConsumerOutput, GeometryOutputs, M7Pipeline
from perception.offline_memory import OfflineMemory, label_name, references, transform


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _text(value, name):
    _require(isinstance(value, str) and bool(value.strip()), f"{name} must be nonempty")
    return value


def _sha(value, name):
    _require(isinstance(value, str) and len(value) == 64 and
             all(c in "0123456789abcdef" for c in value), f"{name} must be lowercase SHA256")
    return value


def _refs(value, name):
    _require(isinstance(value, list) and bool(value), f"{name} requires evidence references")
    references(value)
    return deepcopy(value)


def _instance(value):
    _require(not isinstance(value, bool) and isinstance(value, (str, int)) and
             bool(str(value)), "instance_id must be a single-frame string or integer")
    return str(value)


def _metadata(capture, detection, anchor, identity):
    _require(all(isinstance(x, dict) for x in (capture, detection, anchor)),
             "capture, detection and anchor must be dictionaries")
    expected_identity = asdict(identity)
    _require(capture.get("identity") == expected_identity and
             detection.get("identity") == expected_identity,
             "RGB/detection/depth capture identity mismatch")
    _text(capture.get("acquisition_id"), "capture.acquisition_id")
    _text(capture.get("source_id"), "capture.source_id")
    rgb_sha = _sha(capture.get("image_sha256"), "capture.image_sha256")
    _text(capture.get("time_source"), "capture.time_source")
    capture_refs = _refs(capture.get("evidence_refs"), "capture.evidence_refs")
    _require(any(r["sha256"] == rgb_sha for r in capture_refs),
             "capture evidence must contain the declared RGB image SHA256")
    _text(detection.get("prediction_set"), "detection.prediction_set")
    _require(detection.get("status") in ("ok", "failed"), "detection.status must be ok or failed")
    expected_raw = detection.get("expected_instance_ids")
    actual_raw = detection.get("instances")
    _require(isinstance(expected_raw, list) and isinstance(actual_raw, list),
             "complete detector instance lists required")
    expected = [_instance(x) for x in expected_raw]
    actual = [_instance(x.get("instance_id")) for x in actual_raw if isinstance(x, dict)]
    _require(len(actual) == len(actual_raw) and len(expected) == len(set(expected)) and
             len(actual) == len(set(actual)) and set(expected) == set(actual),
             "incomplete or duplicate detector instance batch")
    if detection["status"] == "failed":
        _text(detection.get("error"), "detection.error")
        _require(not expected, "failed detector batch cannot contain instances")
    else:
        _require(detection.get("error") is None, "successful detector batch cannot contain error")
    _text(anchor.get("map_id"), "anchor.map_id")
    _text(anchor.get("map_version"), "anchor.map_version")
    _text(anchor.get("world_frame"), "anchor.world_frame")
    _text(anchor.get("pose_source"), "anchor.pose_source")
    _text(anchor.get("source_kind"), "anchor.source_kind")
    _text(anchor.get("pose_version"), "anchor.pose_version")
    _require(anchor.get("local_frame_id") == identity.local_frame_id and
             anchor.get("pose_epoch") == identity.pose_epoch,
             "map anchor does not bind the M7 local frame and pose epoch")
    source_ref = anchor.get("source_ref")
    _refs([source_ref] if isinstance(source_ref, dict) else None, "anchor.source_ref")
    anchor_refs = _refs(anchor.get("evidence_refs"), "anchor.evidence_refs")
    pose = transform(anchor.get("T_map_local"))
    _require(anchor.get("calibration_id") == identity.calibration_id,
             "map anchor calibration does not match captured depth")
    return expected, actual_raw, capture_refs, anchor_refs, pose


def build_m4_document(pipeline, geometry, semantic_outputs, *, capture, detection,
                      anchor, now_ns, clock_id):
    """Return one complete M4 document and an audit, or ``None`` for empty/failure.

    ``semantic_outputs`` is keyed by the detector's single-frame instance ID.
    An existing memory output can be valid after newer navigation geometry has
    replaced this frame; it is rechecked against its own cached frame.
    """
    _require(isinstance(pipeline, M7Pipeline) and isinstance(geometry, GeometryOutputs),
             "M7Pipeline and its GeometryOutputs required")
    nav = geometry.navigation
    _require(isinstance(nav, ConsumerOutput) and nav.channel == "navigation" and
             isinstance(geometry.display, ConsumerOutput) and
             geometry.display.channel == "display" and
             geometry.display.identity == nav.identity,
             "mismatched geometry channels")
    identity = nav.identity
    _require(clock_id == identity.clock_id and type(now_ns) is int and now_ns >= identity.acquired_ns,
             "caller clock or time does not match acquisition")
    expected, instances, capture_refs, anchor_refs, pose = _metadata(
        capture, detection, anchor, identity)
    _require(isinstance(semantic_outputs, dict), "semantic outputs must be a dictionary")
    if nav.valid:
        _require(all(isinstance(k, str) for k in semantic_outputs) and
                 set(semantic_outputs) == set(expected),
                 "semantic outputs must cover exactly the complete instance set")
    else:
        _require(not semantic_outputs, "invalid geometry cannot carry semantic outputs")
    audit = {"acquisition_id": capture["acquisition_id"],
             "identity": asdict(identity), "prediction_set": detection["prediction_set"],
             "geometry_reason": nav.reason, "instances": [], "status": ""}

    if detection["status"] == "failed" or not expected:
        _require(not semantic_outputs, "empty or failed batch cannot carry semantic outputs")
        audit["status"] = "not_imported_to_m4"
        audit["reason"] = "detector_failed" if detection["status"] == "failed" else "detector_empty"
        return None, audit

    if not nav.valid:
        # Invalid geometry has no cached-frame token whose live generation can
        # be checked. Keeping it audit-only prevents stale invalid output from
        # becoming a fresh acquisition after reset or map revision.
        audit["status"] = "not_imported_to_m4"
        audit["reason"] = "geometry_unavailable_or_revoked"
        audit["instances"] = [{"instance_id": key, "candidate": "indeterminate",
                               "reason": nav.reason} for key in expected]
        return None, audit
    _require(nav.frame_token is not None, "valid geometry missing frame token")

    observations = []
    for item in instances:
        instance_id = _instance(item["instance_id"])
        _require(item.get("label") == label_name(item.get("label")),
                 "instance.label must be canonical lowercase")
        _text(item.get("source"), "instance.source")
        score = item.get("score")
        _require(not isinstance(score, bool) and isinstance(score, (int, float)) and
                 math.isfinite(score) and 0 <= score <= 1,
                 "instance.score outside [0,1]")
        instance_refs = _refs(item.get("evidence_refs"), "instance.evidence_refs")
        declared_mask_sha = item.get("mask_content_sha256")
        if declared_mask_sha is not None:
            _sha(declared_mask_sha, "instance.mask_content_sha256")
        output = semantic_outputs[instance_id]
        _require(isinstance(output, ConsumerOutput) and output.channel == "memory" and
                 output.identity == identity and output.frame_token == nav.frame_token and
                 output.payload.get("object_id") == instance_id,
                 "semantic output instance/identity/token mismatch")
        checked = output.validate_at(pipeline, now_ns, clock_id)
        _require(checked.valid, f"semantic output revoked: {checked.reason}")
        payload = checked.payload
        _require(payload.get("mask_content_sha256") == declared_mask_sha,
                 "semantic mask content does not match detection source")
        mask_pixels = payload.get("mask_pixels")
        candidate = payload.get("candidate")
        _require(candidate in ("support_present", "free_space_conflict", "indeterminate"),
                 "unknown M7 candidate")
        if candidate == "support_present":
            n = payload.get("mask_valid_pixels")
            fraction = payload.get("valid_fraction")
            position = payload.get("position_local_m")
            _require(type(mask_pixels) is int and mask_pixels > 0 and
                     type(n) is int and 0 <= n <= mask_pixels and
                     isinstance(fraction, (float, int)) and
                     math.isfinite(fraction) and
                     math.isclose(fraction, n / mask_pixels, rel_tol=0, abs_tol=1e-12) and
                     isinstance(position, list) and len(position) == 3 and
                     all(isinstance(v, (float, int)) and math.isfinite(v) for v in position),
                     "M7 support lacks true mask fraction or local position")
            geom = {"valid": True, "reason": None,
                    "position_local_m": deepcopy(position),
                    "keyframe_id": capture["acquisition_id"],
                    "pose_version": anchor["pose_version"],
                    "frame_id": identity.local_frame_id, "units": "m",
                    "summary_method": "m7_median_of_per_point_transformed_mask_depth",
                    "valid_fraction": fraction, "mask_pixels": mask_pixels,
                    "valid_depth_pixels": n}
        else:
            geom = {"valid": False, "reason": "m7_" + candidate + ":" + checked.reason,
                    "mask_pixels": mask_pixels,
                    "mask_valid_pixels": payload.get("mask_valid_pixels")}
        audit["instances"].append({"instance_id": instance_id,
                                   "candidate": candidate, "reason": checked.reason})
        observations.append({"instance_id": item["instance_id"],
                             "instance_kind": "single_frame_detector_instance",
                             "label": item["label"], "score": score,
                             "source": item["source"], "geometry": geom,
                             "evidence_refs": capture_refs + instance_refs + anchor_refs})

    map_row = {"map_id": anchor["map_id"], "map_version": anchor["map_version"],
               "world_frame": anchor["world_frame"], "units": "m",
               "pose_source": anchor["pose_source"], "source_kind": anchor["source_kind"],
               "source_ref": deepcopy(anchor["source_ref"]),
               "calibration_id": identity.calibration_id}
    pose_row = {"map_id": anchor["map_id"], "keyframe_id": capture["acquisition_id"],
                "version": anchor["pose_version"], "frame_id": identity.local_frame_id,
                "T_map_local": pose.tolist(), "source": anchor["pose_source"],
                "evidence_refs": anchor_refs}
    acquisition = {"id": capture["acquisition_id"], "source_id": capture["source_id"],
                   "image_sha256": capture["image_sha256"],
                   "timestamp": identity.acquired_ns,
                   "time_source": capture["time_source"],
                   "clock_domain": identity.clock_id + ":ns",
                   "calibration_id": identity.calibration_id}
    document = {"schema": "dev3.m4.input.v1", "maps": [map_row], "poses": [pose_row],
                "batches": [{"map_id": anchor["map_id"],
                             "map_version": anchor["map_version"],
                             "acquisition": acquisition,
                             "prediction_set": detection["prediction_set"],
                             "status": "ok", "error": None,
                             "observations": observations}]}
    audit["status"] = "complete_m4_document"
    audit["reason"] = "candidate_observations_only_no_absence_transition"
    return document, audit


def apply_map_revision(memory, pipeline, revision_doc, store_path=None):
    """Commit an explicit M4 revision before revoking M7's old generation.

    A failed revision or save leaves both caller objects unchanged. A duplicate
    revision is idempotent and does not revoke a newer M7 generation.
    """
    _require(isinstance(memory, OfflineMemory) and isinstance(pipeline, M7Pipeline),
             "OfflineMemory and M7Pipeline required")
    candidate = deepcopy(memory)
    result = candidate.revise(revision_doc)
    if result["status"] == "duplicate_revision":
        return result
    if store_path is not None:
        candidate.save(Path(store_path))
    memory.data = candidate.data
    memory._loaded_path = candidate._loaded_path
    memory._loaded_hash = candidate._loaded_hash
    pipeline.reset()
    return result
