"""M7→M4 boundary tests with real CPU geometry and controlled evidence."""

from copy import deepcopy
from dataclasses import asdict, replace
import hashlib
from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.m7_memory import apply_map_revision, build_m4_document
from host.m7_native import NativeM7, compile_native
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GeometryOutputs, GridSpec,
                              M7Pipeline, PoseAtCapture, SemanticPacket)
from perception.offline_memory import OfflineMemory


GRID = GridSpec((-4., -4., -1.), .25, (32, 32, 32))


def digest(value):
    return hashlib.sha256(value).hexdigest()


def ref(name, data):
    return {"path": "synthetic-fixture:" + name, "sha256": digest(data)}


MAP_REF = ref("map", b"controlled map declaration")
POSE_REF = ref("pose", b"controlled absolute map-to-odom transform")


def transform(translation=(0., 0., 0.), rotation=None):
    t = np.eye(4)
    if rotation is not None:
        t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def frame(seq=1, *, stamp=None, depth=None, camera_pose=None, epoch="odom-1"):
    if stamp is None:
        stamp = 1_000_000_000 + seq * 10_000_000
    identity = CaptureIdentity("synthetic-stream-1", seq, stamp, "fixture-clock",
                               "depth-calibration-v1", "odom", epoch)
    if depth is None:
        depth = np.full((25, 25), 3., np.float32)
        depth[10:15, 10:15] = 2.
        depth[10:15, 16:21] = 2.
    k = np.array([[12., 0., 12.], [0., 12., 12.], [0., 0., 1.]])
    pose = PoseAtCapture(stamp, identity.clock_id, identity.local_frame_id,
                         identity.pose_epoch, True,
                         transform() if camera_pose is None else camera_pose)
    return DepthFrame(identity, depth, k, pose)


def mask(which):
    result = np.zeros((25, 25), np.bool_)
    if which == "one":
        result[10:15, 10:15] = True
    elif which == "two":
        result[10:15, 16:21] = True
    elif which != "empty":
        raise ValueError(which)
    return result


def mask_digest(value):
    return digest(value.astype(np.uint8).tobytes()) if value is not None else None


def source(f, masks, *, status="ok", acquisition_id=None, image_bytes=None,
           map_version="map-v1", map_pose=None, pose_version="map-odom-v1"):
    identity = asdict(f.identity)
    acquisition_id = acquisition_id or f"capture-{f.identity.frame_id}"
    image_bytes = image_bytes or f"RGB frame {f.identity.frame_id}".encode()
    capture = {"identity": identity, "acquisition_id": acquisition_id,
               "source_id": "controlled-stream", "image_sha256": digest(image_bytes),
               "time_source": "synthetic fixture integer nanoseconds",
               "evidence_refs": [ref("rgb-" + acquisition_id, image_bytes),
                                 ref("depth-" + acquisition_id, f.depth_m.tobytes())]}
    instances = []
    for instance_id, value in masks.items():
        instances.append({"instance_id": instance_id,
                          "label": "cup", "score": .8,
                          "source": "controlled_detector",
                          "mask_content_sha256": mask_digest(value),
                          "evidence_refs": [ref("mask-" + acquisition_id + "-" + instance_id,
                                                b"missing mask" if value is None else
                                                value.astype(np.uint8).tobytes())]})
    detection = {"identity": identity, "prediction_set": "controlled-model-and-config-v1",
                 "status": status, "error": "controlled failure" if status == "failed" else None,
                 "expected_instance_ids": list(masks), "instances": instances}
    anchor = {"map_id": "controlled-map", "map_version": map_version,
              "world_frame": "map", "pose_source": "controlled_exact_pose",
              "source_kind": "synthetic", "source_ref": deepcopy(MAP_REF),
              "local_frame_id": f.identity.local_frame_id,
              "pose_epoch": f.identity.pose_epoch,
              "pose_version": pose_version,
              "T_map_local": (transform() if map_pose is None else map_pose).tolist(),
              "evidence_refs": [deepcopy(POSE_REF)],
              "calibration_id": f.identity.calibration_id}
    return capture, detection, anchor


class ContinuousMemoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="m7-memory-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.backend = NativeM7(compile_native(Path(cls.temp.name) / "m7.so"))

    def setUp(self):
        self.pipeline = M7Pipeline(self.backend, GRID)

    def geometry(self, f):
        return self.pipeline.ingest_geometry(f, now_ns=f.identity.acquired_ns,
                                             now_clock_id=f.identity.clock_id)

    def semantic(self, f, geometry, instance_id, value, bounds=None, now_ns=None):
        return self.pipeline.ingest_semantic(SemanticPacket(
            f.identity, geometry.navigation.frame_token, instance_id, value, bounds),
            now_ns=f.identity.acquired_ns + 1 if now_ns is None else now_ns,
            now_clock_id=f.identity.clock_id)

    def build(self, f, geometry, outputs, metadata, *, now_ns=None):
        capture, detection, anchor = metadata
        return build_m4_document(self.pipeline, geometry, outputs, capture=capture,
                                 detection=detection, anchor=anchor,
                                 now_ns=f.identity.acquired_ns + 1 if now_ns is None else now_ns,
                                 clock_id=f.identity.clock_id)

    def test_complete_two_instance_batch_uses_one_map_transform_and_true_fraction(self):
        rz = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        f = frame(camera_pose=transform((1., 2., 0.), rz))
        geometry = self.geometry(f)
        self.assertTrue(geometry.navigation.valid)
        masks = {"1": mask("one"), "2": mask("two")}
        outputs = {key: self.semantic(f, geometry, key, value) for key, value in masks.items()}
        map_pose = transform((10., 20., 0.), rz.T)
        document, audit = self.build(f, geometry, outputs,
                                     source(f, masks, map_pose=map_pose))
        self.assertEqual(audit["status"], "complete_m4_document")
        self.assertEqual(len(document["batches"]), 1)
        self.assertEqual(len(document["batches"][0]["observations"]), 2)
        self.assertNotIn(geometry.navigation.frame_token, repr(document))
        for key, value in masks.items():
            v, u = np.nonzero(value)
            z = f.depth_m[v, u]
            camera = np.column_stack(((u - 12) * z / 12, (v - 12) * z / 12, z))
            local = np.median(camera @ rz.T + np.array([1., 2., 0.]), axis=0)
            raw = next(o for o in document["batches"][0]["observations"]
                       if str(o["instance_id"]) == key)
            np.testing.assert_allclose(raw["geometry"]["position_local_m"], local)
            self.assertEqual(raw["geometry"]["valid_fraction"], 1.)
            self.assertEqual(raw["geometry"]["mask_pixels"], 25)
        memory = OfflineMemory()
        memory.import_document(document)
        objects = memory.query()["objects"]
        self.assertEqual(len(objects), 2)
        for obj in objects:
            raw = memory.data["observations"][obj["observation_ids"][0]]["raw"]
            local = np.array(raw["geometry"]["position_local_m"])
            np.testing.assert_allclose(obj["position_m"], map_pose[:3, :3] @ local + map_pose[:3, 3])
            self.assertEqual(obj["physical_observation_count"], 1)

    def test_non_axis_rotation_proves_per_point_median_order(self):
        angle = np.deg2rad(30)
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                             [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        coordinates = [(8, 8, 1.8), (8, 11, 2.2), (9, 14, 1.9),
                       (10, 10, 2.4), (10, 13, 1.7), (11, 9, 2.1),
                       (11, 15, 2.5), (12, 12, 1.6), (13, 10, 2.3),
                       (14, 14, 2.0)]
        depth = np.full((25, 25), 3., np.float32)
        value = np.zeros((25, 25), np.bool_)
        for v, u, z in coordinates:
            value[v, u] = True
            depth[v, u] = z
        f = frame(depth=depth, camera_pose=transform((.3, -.2, .1), rotation))
        geometry = self.geometry(f)
        output = self.semantic(f, geometry, "1", value)
        self.assertEqual(output.payload["candidate"], "support_present")
        document, _ = self.build(f, geometry, {"1": output}, source(f, {"1": value}))
        camera = np.array([[(u - 12) * z / 12, (v - 12) * z / 12, z]
                           for v, u, z in coordinates])
        correct = np.median(camera @ rotation.T + [.3, -.2, .1], axis=0)
        incorrect = np.median(camera, axis=0) @ rotation.T + [.3, -.2, .1]
        self.assertGreater(np.linalg.norm(correct - incorrect), .02)
        position = document["batches"][0]["observations"][0]["geometry"]["position_local_m"]
        np.testing.assert_allclose(position, correct, atol=1e-7)

    def test_late_semantic_original_frame_and_wrong_sources_fail_closed(self):
        old = frame(1)
        old_geometry = self.geometry(old)
        newer = frame(2, depth=np.full((25, 25), 3., np.float32))
        self.geometry(newer)
        self.assertFalse(old_geometry.navigation.validate_at(
            self.pipeline, newer.identity.acquired_ns + 1, old.identity.clock_id).valid)
        value = mask("one")
        output = self.semantic(old, old_geometry, "1", value,
                               now_ns=newer.identity.acquired_ns + 1)
        metadata = source(old, {"1": value})
        document, _ = self.build(old, old_geometry, {"1": output}, metadata,
                                 now_ns=newer.identity.acquired_ns + 1)
        self.assertTrue(document["batches"][0]["observations"][0]["geometry"]["valid"])
        wrong_mask = deepcopy(metadata)
        wrong_mask[1]["instances"][0]["mask_content_sha256"] = mask_digest(mask("two"))
        with self.assertRaisesRegex(ValueError, "mask content"):
            self.build(old, old_geometry, {"1": output}, wrong_mask,
                       now_ns=newer.identity.acquired_ns + 1)
        wrong_rgb = deepcopy(metadata)
        wrong_rgb[1]["identity"]["frame_id"] = newer.identity.frame_id
        with self.assertRaisesRegex(ValueError, "capture identity"):
            self.build(old, old_geometry, {"1": output}, wrong_rgb,
                       now_ns=newer.identity.acquired_ns + 1)
        with self.assertRaisesRegex(ValueError, "complete instance set"):
            self.build(old, old_geometry, {}, metadata,
                       now_ns=newer.identity.acquired_ns + 1)
        self.pipeline.reset()
        with self.assertRaisesRegex(ValueError, "revoked"):
            self.build(old, old_geometry, {"1": output}, metadata,
                       now_ns=newer.identity.acquired_ns + 1)

    def test_measured_mask_fraction_is_preserved_and_low_quality_stays_pending(self):
        for valid_count, expected in ((18, "support_present"), (17, "indeterminate")):
            with self.subTest(valid_count=valid_count):
                depth = np.full((25, 25), 3., np.float32)
                patch = depth[10:15, 10:15]
                patch[:] = 2.
                patch.flat[valid_count:] = np.nan
                f = frame(depth=depth)
                engine = M7Pipeline(self.backend, GRID)
                geometry = engine.ingest_geometry(f, now_ns=f.identity.acquired_ns,
                                                  now_clock_id=f.identity.clock_id)
                value = mask("one")
                output = engine.ingest_semantic(SemanticPacket(
                    f.identity, geometry.navigation.frame_token, "1", value),
                    now_ns=f.identity.acquired_ns + 1, now_clock_id=f.identity.clock_id)
                self.assertEqual(output.payload["candidate"], expected)
                self.assertEqual(output.payload["mask_pixels"], 25)
                capture, detection, anchor = source(f, {"1": value})
                document, _ = build_m4_document(engine, geometry, {"1": output},
                    capture=capture, detection=detection, anchor=anchor,
                    now_ns=f.identity.acquired_ns + 1, clock_id=f.identity.clock_id)
                raw = document["batches"][0]["observations"][0]
                if valid_count == 18:
                    self.assertAlmostEqual(output.payload["valid_fraction"], 18 / 25)
                    self.assertAlmostEqual(raw["geometry"]["valid_fraction"], 18 / 25)
                    memory = OfflineMemory()
                    memory.import_document(document)
                    self.assertEqual(len(memory.query("cup")["objects"]), 1)
                else:
                    self.assertFalse(raw["geometry"]["valid"])
                    memory = OfflineMemory()
                    memory.import_document(document)
                    self.assertEqual(memory.query("cup")["objects"], [])

    def test_retransmit_restart_and_integer_nanosecond_order(self):
        first_stamp = 1_300_000_000_000_000_000
        self.assertEqual(float(first_stamp / 1_000_000_000),
                         float((first_stamp + 1) / 1_000_000_000))
        first = frame(1, stamp=first_stamp)
        g1 = self.geometry(first)
        value = mask("one")
        o1 = self.semantic(first, g1, "1", value)
        doc1, _ = self.build(first, g1, {"1": o1}, source(first, {"1": value}))
        second = frame(2, stamp=first_stamp + 1)
        g2 = self.geometry(second)
        o2 = self.semantic(second, g2, "1", value)
        doc2, _ = self.build(second, g2, {"1": o2}, source(second, {"1": value}))
        self.assertEqual(doc2["batches"][0]["acquisition"]["timestamp"] -
                         doc1["batches"][0]["acquisition"]["timestamp"], 1)
        self.assertEqual(doc1["batches"][0]["acquisition"]["clock_domain"], "fixture-clock:ns")
        memory = OfflineMemory()
        memory.import_document(doc1)
        memory.import_document(doc2)
        obj = memory.query("cup")["objects"][0]
        self.assertEqual(obj["last_seen"], second.identity.acquired_ns)
        self.assertEqual(obj["physical_observation_count"], 2)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "memory.json"
            memory.save(path)
            loaded = OfflineMemory.load(path)
            before = path.read_bytes()
            self.assertEqual(loaded.import_document(doc1)["batches"][0]["status"], "duplicate")
            self.assertEqual(loaded.import_document(doc2)["batches"][0]["status"], "duplicate")
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(loaded.query("cup")["objects"][0]["physical_observation_count"], 2)

    def test_revision_is_atomic_and_revokes_old_output_only_on_success(self):
        f = frame()
        geometry = self.geometry(f)
        value = mask("one")
        output = self.semantic(f, geometry, "1", value)
        document, _ = self.build(f, geometry, {"1": output}, source(f, {"1": value}))
        memory = OfflineMemory()
        memory.import_document(document)
        before = deepcopy(memory.data)
        revised_pose = deepcopy(document["poses"][0])
        revised_pose["version"] = "map-odom-v2"
        revised_pose["T_map_local"][0][3] = 3.
        revision = {"schema": "dev3.m4.revision.v1", "revision_id": "controlled-revision-1",
                    "map_id": "controlled-map", "from_version": "map-v1",
                    "to_version": "map-v2", "reason": "controlled synthetic correction",
                    "poses": [revised_pose], "unchanged_keyframes": []}
        invalid = deepcopy(revision)
        invalid["from_version"] = "wrong-version"
        with self.assertRaisesRegex(ValueError, "version chain"):
            apply_map_revision(memory, self.pipeline, invalid)
        self.assertEqual(memory.data, before)
        self.assertTrue(output.validate_at(self.pipeline,
                                           f.identity.acquired_ns + 2, f.identity.clock_id).valid)
        with tempfile.TemporaryDirectory() as temp:
            bad_store = Path(temp) / "already-exists.json"
            bad_store.write_text("not our store")
            with self.assertRaisesRegex(ValueError, "exists without load"):
                apply_map_revision(memory, self.pipeline, revision, store_path=bad_store)
            self.assertEqual(memory.data, before)
            self.assertTrue(output.validate_at(self.pipeline,
                                               f.identity.acquired_ns + 2, f.identity.clock_id).valid)
            good_store = Path(temp) / "memory.json"
            result = apply_map_revision(memory, self.pipeline, revision, store_path=good_store)
            self.assertEqual(result["status"], "revised")
            self.assertFalse(output.validate_at(self.pipeline,
                                                f.identity.acquired_ns + 2, f.identity.clock_id).valid)
            obj = OfflineMemory.load(good_store).query("cup")["objects"][0]
            self.assertEqual(obj["map_version"], "map-v2")
            self.assertEqual(obj["physical_observation_count"], 1)
            self.assertEqual(obj["position_m"][0], before["objects"][obj["id"]]["position_m"][0] + 3.)
            generation = self.pipeline._generation
            self.assertEqual(apply_map_revision(memory, self.pipeline, revision,
                                                store_path=good_store)["status"], "duplicate_revision")
            self.assertEqual(self.pipeline._generation, generation)

    def test_missing_anchor_pending_and_negative_or_invalid_evidence_cannot_remove(self):
        f = frame()
        geometry = self.geometry(f)
        value = mask("one")
        output = self.semantic(f, geometry, "1", value)
        doc, _ = self.build(f, geometry, {"1": output}, source(f, {"1": value}))
        memory = OfflineMemory()
        memory.import_document(doc)
        old_id = memory.query("cup")["objects"][0]["id"]
        missing = {"schema": "dev3.m4.revision.v1", "revision_id": "no-anchor-v2",
                   "map_id": "controlled-map", "from_version": "map-v1",
                   "to_version": "map-v2", "reason": "controlled missing anchor",
                   "poses": [], "unchanged_keyframes": []}
        apply_map_revision(memory, self.pipeline, missing)
        obj = memory.query("cup")["objects"][0]
        self.assertEqual(obj["id"], old_id)
        self.assertEqual(obj["state"], "pending_revision")
        self.assertIsNone(obj["position_m"])
        self.assertEqual(obj["physical_observation_count"], 1)

        other = M7Pipeline(self.backend, GRID)
        removed = frame(2, depth=np.full((25, 25), 3., np.float32))
        g2 = other.ingest_geometry(removed, now_ns=removed.identity.acquired_ns,
                                   now_clock_id=removed.identity.clock_id)
        empty = mask("empty")
        bounds = np.array([[-.25, -.25, 1.5], [.25, .25, 2.5]])
        negative = other.ingest_semantic(SemanticPacket(removed.identity, g2.navigation.frame_token,
            "1", empty, bounds), now_ns=removed.identity.acquired_ns + 1,
            now_clock_id=removed.identity.clock_id)
        self.assertEqual(negative.payload["candidate"], "free_space_conflict")
        capture, detection, anchor = source(removed, {"1": empty}, map_version="map-v2")
        anchor["pose_version"] = "map-odom-v2"
        document, audit = build_m4_document(other, g2, {"1": negative}, capture=capture,
            detection=detection, anchor=anchor, now_ns=removed.identity.acquired_ns + 1,
            clock_id=removed.identity.clock_id)
        self.assertEqual(audit["instances"][0]["candidate"], "free_space_conflict")
        self.assertFalse(document["batches"][0]["observations"][0]["geometry"]["valid"])
        memory.import_document(document)
        self.assertEqual(memory.query("cup")["objects"][0]["physical_observation_count"], 1)

    def test_empty_failed_and_invalid_geometry_are_audit_only_even_after_reset(self):
        f = frame(depth=np.full((25, 25), np.nan, np.float32))
        geometry = self.geometry(f)
        self.assertFalse(geometry.navigation.valid)
        value = mask("one")
        metadata = source(f, {"1": value})
        document, audit = self.build(f, geometry, {}, metadata)
        self.assertIsNone(document)
        self.assertEqual(audit["reason"], "geometry_unavailable_or_revoked")
        self.pipeline.reset()
        document, audit = self.build(f, geometry, {}, metadata)
        self.assertIsNone(document)
        self.assertEqual(audit["instances"][0]["reason"], "no_valid_depth")
        fresh = frame(2)
        valid = self.geometry(fresh)
        for status in ("ok", "failed"):
            capture, detection, anchor = source(fresh, {}, status=status)
            document, audit = self.build(fresh, valid, {}, (capture, detection, anchor))
            self.assertIsNone(document)
            self.assertEqual(audit["status"], "not_imported_to_m4")
        memory = OfflineMemory()
        self.assertEqual(memory.query()["objects"], [])


    def partial_depth_fixture(self, valid_count=18):
        """A capture whose mask has a measurable, non-trivial valid fraction."""
        depth = np.full((25, 25), 3., np.float32)
        patch = depth[10:15, 10:15]
        patch[:] = 2.
        patch.flat[valid_count:] = np.nan
        f = frame(depth=depth)
        pipeline = M7Pipeline(self.backend, GRID)
        geometry = pipeline.ingest_geometry(f, now_ns=f.identity.acquired_ns,
                                            now_clock_id=f.identity.clock_id)
        value = mask("one")
        output = pipeline.ingest_semantic(
            SemanticPacket(f.identity, geometry.navigation.frame_token, "1", value),
            now_ns=f.identity.acquired_ns + 1, now_clock_id=f.identity.clock_id)
        return pipeline, f, geometry, value, output

    def test_now_ns_must_be_a_true_python_int_not_a_float_or_numpy_integer(self):
        """``type(now_ns) is int`` is the only thing that stops a smuggled stamp.

        The downstream ``_nanoseconds`` check accepts ``np.integer``, so a
        numpy stamp reaches M4 unnoticed if this boundary check is weakened;
        ``isinstance`` would additionally let a bool through.
        """
        f = frame()
        geometry = self.geometry(f)
        value = mask("one")
        output = self.semantic(f, geometry, "1", value)
        metadata = source(f, {"1": value})
        exact = f.identity.acquired_ns + 1
        document, _ = self.build(f, geometry, {"1": output}, metadata, now_ns=exact)
        self.assertEqual(document["batches"][0]["acquisition"]["timestamp"],
                         f.identity.acquired_ns)
        for smuggled in (float(exact), np.int64(exact), np.float64(exact)):
            with self.subTest(kind=type(smuggled).__name__):
                self.assertEqual(smuggled, exact)  # equal in value, wrong in type
                with self.assertRaisesRegex(ValueError, "caller clock or time"):
                    self.build(f, geometry, {"1": output}, metadata, now_ns=smuggled)
        # bool is an int subclass, so only `type(...) is int` rejects it. Pin it
        # on a capture early enough that True would clear the ordering check.
        early = frame(1, stamp=1)
        self.assertGreaterEqual(True, early.identity.acquired_ns)
        pipeline = M7Pipeline(self.backend, GRID)
        early_geometry = pipeline.ingest_geometry(
            early, now_ns=early.identity.acquired_ns,
            now_clock_id=early.identity.clock_id)
        early_value = mask("one")
        early_output = pipeline.ingest_semantic(
            SemanticPacket(early.identity, early_geometry.navigation.frame_token,
                           "1", early_value),
            now_ns=2, now_clock_id=early.identity.clock_id)
        capture, detection, anchor = source(early, {"1": early_value})
        self.assertIsNotNone(build_m4_document(
            pipeline, early_geometry, {"1": early_output}, capture=capture,
            detection=detection, anchor=anchor, now_ns=2,
            clock_id=early.identity.clock_id)[0])
        with self.assertRaisesRegex(ValueError, "caller clock or time"):
            build_m4_document(pipeline, early_geometry, {"1": early_output},
                              capture=capture, detection=detection, anchor=anchor,
                              now_ns=True, clock_id=early.identity.clock_id)

    def test_declared_valid_fraction_must_equal_the_measured_pixel_ratio(self):
        """M4 never imports a fraction that the pixel counts do not support."""
        pipeline, f, geometry, value, output = self.partial_depth_fixture()
        capture, detection, anchor = source(f, {"1": value})

        def build(semantic):
            return build_m4_document(pipeline, geometry, {"1": semantic},
                                     capture=capture, detection=detection,
                                     anchor=anchor, now_ns=f.identity.acquired_ns + 1,
                                     clock_id=f.identity.clock_id)

        self.assertEqual(output.payload["candidate"], "support_present")
        n, pixels = output.payload["mask_valid_pixels"], output.payload["mask_pixels"]
        self.assertLess(n, pixels)  # a trivially-1.0 fraction would prove nothing
        geom = build(output)[0]["batches"][0]["observations"][0]["geometry"]
        self.assertEqual(geom["valid_fraction"], n / pixels)
        self.assertEqual((geom["valid_depth_pixels"], geom["mask_pixels"]), (n, pixels))
        for wrong in (1.0, 0.0, n / pixels + 1e-9, n / pixels - 1e-9):
            with self.subTest(wrong=wrong):
                self.assertNotEqual(wrong, n / pixels)
                forged = replace(output, payload={**output.payload,
                                                  "valid_fraction": wrong})
                with self.assertRaisesRegex(ValueError, "true mask fraction"):
                    build(forged)

    def test_memory_output_must_carry_the_token_of_the_geometry_it_ships_with(self):
        """The boundary rebinds output to geometry itself, not via the pipeline."""
        f = frame()
        geometry = self.geometry(f)
        value = mask("one")
        output = self.semantic(f, geometry, "1", value)
        metadata = source(f, {"1": value})
        token = geometry.navigation.frame_token
        other = token[::-1]
        self.assertEqual(output.frame_token, token)
        self.assertNotEqual(other, token)
        self.assertIsNotNone(self.build(f, geometry, {"1": output}, metadata)[0])
        # A retained memory output that names some other cached frame.
        with self.assertRaisesRegex(ValueError, "instance/identity/token mismatch"):
            self.build(f, geometry, {"1": replace(output, frame_token=other)}, metadata)
        # Geometry that is not the frame the output was computed against. The
        # pipeline still revalidates this output happily, so only the
        # boundary's own binding check can refuse the pairing.
        forged = GeometryOutputs(replace(geometry.navigation, frame_token=other),
                                 geometry.display)
        self.assertTrue(output.validate_at(self.pipeline, f.identity.acquired_ns + 1,
                                           f.identity.clock_id).valid)
        with self.assertRaisesRegex(ValueError, "instance/identity/token mismatch"):
            self.build(f, forged, {"1": output}, metadata)


    def test_region_channel_output_is_never_consumed_as_m4_memory(self):
        from host.m7_object_native import NativeObjectEvidence, compile_object_native
        from host.m7_pipeline import RegionCheckPacket
        region = NativeObjectEvidence(compile_object_native(Path(self.temp.name) / "roi.so"))
        self.pipeline = M7Pipeline(self.backend, GRID, region_backend=region)
        f = frame()
        geometry = self.geometry(f)
        value = mask("one")
        output = self.semantic(f, geometry, "1", value)
        metadata = source(f, {"1": value})
        document, _ = self.build(f, geometry, {"1": output}, metadata)
        self.assertIsNotNone(document)
        bounds = np.array([[-.3, -.3, 1.9], [.3, .3, 2.1]])
        regions = self.pipeline.ingest_region_checks(
            RegionCheckPacket(f.identity, geometry.navigation.frame_token, (("1", bounds),)),
            now_ns=f.identity.acquired_ns + 1, now_clock_id=f.identity.clock_id)
        self.assertEqual(len(regions), 1)
        self.assertTrue(regions[0].valid)
        self.assertEqual(regions[0].channel, "region")
        self.assertNotIn("candidate", regions[0].payload)
        with self.assertRaisesRegex(ValueError, "semantic output instance"):
            self.build(f, geometry, {"1": regions[0]}, metadata)


if __name__ == "__main__":
    unittest.main()
