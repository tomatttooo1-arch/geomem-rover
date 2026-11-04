"""D-029: the FPGA object path supplies ``valid_fraction`` for support evidence.

The PL kernel returns integer counts only (mask pixels, masked valid points).
The host forms the ratio from exactly those two integers with Python true
division, the same arithmetic the memory consumer uses to check it.

The consumer tests feed the real memory-side ``build_m4_document`` with payloads
produced by the real pipeline and the real native object backend, unmodified.
They run only where the memory bridge exists (after integration); on the M7
branch alone that consumer is absent and they are reported as skipped.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.m7_native import NativeM7, compile_native
from host.m7_object_native import (NativeObjectEvidence, RawObjectEvidence,
                                   compile_object_native)
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)


ROOT = Path(__file__).resolve().parents[1]
K = np.array([[12., 0., 12.], [0., 12., 12.], [0., 0., 1.]])
GRID = GridSpec((-2., -2., -1.), .25, (16, 16, 20))
HAS_MEMORY = (ROOT / "host" / "m7_memory.py").is_file()


def square_scene(invalid):
    """5x5 mask at depth 2 m; ``invalid`` masked pixels get invalid depth."""
    depth = np.full((25, 25), 3., np.float32)
    mask = np.zeros((25, 25), np.bool_)
    depth[10:15, 10:15] = 2.
    mask[10:15, 10:15] = True
    bad = [(10 + i // 5, 10 + i % 5) for i in range(invalid)]
    # Three different invalidity forms: NaN, too near, too far.
    for j, (v, u) in enumerate(bad):
        depth[v, u] = (np.nan, .05, 12.)[j % 3]
    return depth, mask


def irregular_scene():
    """27-pixel mask, 20 valid: a non-dyadic ratio (20/27)."""
    depth = np.full((25, 25), 3., np.float32)
    mask = np.zeros((25, 25), np.bool_)
    mask[9:12, 9:18] = True  # 3 x 9 = 27 pixels
    depth[9:12, 9:18] = 2.
    for i, (v, u) in enumerate([(9, 9), (9, 11), (10, 13), (11, 15),
                                (11, 17), (10, 10), (9, 16)]):
        depth[v, u] = (np.nan, .05, 12.)[i % 3]
    return depth, mask


def expected_valid(depth, mask):
    good = np.isfinite(depth) & (depth > .1) & (depth < 10)
    return int(np.count_nonzero(mask & good)), int(np.count_nonzero(mask))


def make_frame(seq, depth):
    stamp = 1_000_000_000 + seq * 10_000_000
    identity = CaptureIdentity("stream-vf", seq, stamp, "camera-clock", "K-25-v1",
                               "odom", "epoch-1")
    pose = PoseAtCapture(stamp, "camera-clock", "odom", "epoch-1", True, np.eye(4))
    return DepthFrame(identity, depth.copy(), K.copy(), pose)


class Tamper:
    """Wrap the real native backend and alter only its returned details."""

    def __init__(self, inner, edit):
        self.inner, self.edit = inner, edit

    def candidate(self, *args):
        candidate, reason, details = self.inner.candidate(*args)
        return candidate, reason, self.edit(details)


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="m7-valid-fraction-")
        cls.addClassCleanup(cls.tmp.cleanup)
        path = Path(cls.tmp.name)
        cls.object_library = compile_object_native(path / "object.so")
        cls.object_native = NativeObjectEvidence(cls.object_library)
        cls.voxel_native = NativeM7(compile_native(path / "voxel.so"))

    def run_one(self, object_backend, depth, masks, seq=1):
        engine = M7Pipeline(self.voxel_native, GRID, object_backend=object_backend)
        f = make_frame(seq, depth)
        geometry = engine.ingest_geometry(f, now_ns=f.identity.acquired_ns,
                                          now_clock_id=f.identity.clock_id)
        self.assertTrue(geometry.navigation.valid)
        outputs = {key: engine.ingest_semantic(
                       SemanticPacket(f.identity, geometry.navigation.frame_token, key, value),
                       now_ns=f.identity.acquired_ns + 1, now_clock_id=f.identity.clock_id)
                   for key, value in masks.items()}
        return engine, f, geometry, outputs


class ValidFractionProducerTests(_Base):
    SUPPORT_CASES = (("full", lambda: square_scene(0), 25, 25),
                     ("18/25", lambda: square_scene(7), 18, 25),
                     ("20/27", irregular_scene, 20, 27))

    def test_support_fraction_is_exact_ratio_of_kernel_counts(self):
        for name, build, n_expected, m_expected in self.SUPPORT_CASES:
            with self.subTest(case=name):
                depth, mask = build()
                self.assertEqual(expected_valid(depth, mask), (n_expected, m_expected))
                _, _, _, native = self.run_one(self.object_native, depth, {"1": mask})
                _, _, _, python = self.run_one(None, depth, {"1": mask})
                payload = native["1"].payload
                self.assertTrue(native["1"].valid)
                self.assertEqual(payload["candidate"], "support_present")
                counts = payload["kernel_counts"]
                self.assertEqual(counts["mask_pixels"], m_expected)
                self.assertEqual(counts["mask_pixels"], int(np.count_nonzero(mask)))
                self.assertEqual(payload["mask_valid_pixels"], n_expected)
                fraction = payload["valid_fraction"]
                self.assertIs(type(fraction), float)
                # Bitwise, not approximate: same integer pair, same division.
                self.assertEqual(fraction.hex(), (n_expected / m_expected).hex())
                self.assertEqual(fraction.hex(),
                                 python["1"].payload["valid_fraction"].hex())
                if n_expected < m_expected:
                    self.assertLess(fraction, 1.)  # not a filled-in constant

    def test_below_threshold_mask_carries_no_fraction_on_either_backend(self):
        depth, mask = square_scene(8)  # 17/25 = .68 < .7
        _, _, _, native = self.run_one(self.object_native, depth, {"1": mask})
        _, _, _, python = self.run_one(None, depth, {"1": mask})
        for output in (native["1"], python["1"]):
            self.assertEqual(output.payload["candidate"], "indeterminate")
            self.assertEqual(output.reason, "invalid_mask_depth")
            self.assertEqual(output.payload["mask_valid_pixels"], 17)
            self.assertNotIn("valid_fraction", output.payload)

    def test_kernel_and_host_mask_count_disagreement_fails_closed(self):
        class Skewed(NativeObjectEvidence):
            def evaluate(self, *args, **kwargs):
                raw = super().evaluate(*args, **kwargs)
                counts = dict(raw.counts)
                counts["mask_pixels"] += 1
                return RawObjectEvidence(counts, raw.points_local_m)

        depth, mask = square_scene(7)
        _, _, _, skewed = self.run_one(Skewed(self.object_library), depth, {"1": mask})
        self.assertFalse(skewed["1"].valid)
        self.assertEqual(skewed["1"].reason, "object_backend_failure:RuntimeError")
        self.assertNotIn("valid_fraction", skewed["1"].payload)

    def test_backend_cannot_override_host_mask_denominator_or_identity(self):
        # The consumer's denominator and mask provenance are host-computed;
        # an object backend must not be able to replace them via details.
        depth, mask = square_scene(7)
        for key, value in (("mask_pixels", 18), ("mask_content_sha256", "0" * 64)):
            with self.subTest(key=key):
                edit = (lambda k, v: lambda d: dict(d, **{k: v}))(key, value)
                _, _, _, out = self.run_one(Tamper(self.object_native, edit), depth, {"1": mask})
                self.assertFalse(out["1"].valid)
                self.assertEqual(out["1"].reason, "object_backend_failure:ValueError")
        _, _, _, ok = self.run_one(self.object_native, depth, {"1": mask})
        self.assertEqual(ok["1"].payload["mask_pixels"], 25)
        self.assertEqual(ok["1"].payload["valid_fraction"],
                         ok["1"].payload["mask_valid_pixels"] / ok["1"].payload["mask_pixels"])


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def _ref(name, data):
    return {"path": "synthetic-fixture:" + name, "sha256": _digest(data)}


def _metadata(f, masks):
    identity = asdict(f.identity)
    image = ("RGB frame %d" % f.identity.frame_id).encode()
    capture = {"identity": identity, "acquisition_id": "capture-%d" % f.identity.frame_id,
               "source_id": "controlled-stream", "image_sha256": _digest(image),
               "time_source": "synthetic fixture integer nanoseconds",
               "evidence_refs": [_ref("rgb", image), _ref("depth", f.depth_m.tobytes())]}
    instances = [{"instance_id": key, "label": "cup", "score": .8,
                  "source": "controlled_detector",
                  "mask_content_sha256": _digest(value.astype(np.uint8).tobytes()),
                  "evidence_refs": [_ref("mask-" + key, value.astype(np.uint8).tobytes())]}
                 for key, value in masks.items()]
    detection = {"identity": identity, "prediction_set": "controlled-model-and-config-v1",
                 "status": "ok", "error": None,
                 "expected_instance_ids": list(masks), "instances": instances}
    anchor = {"map_id": "controlled-map", "map_version": "map-v1", "world_frame": "map",
              "pose_source": "controlled_exact_pose", "source_kind": "synthetic",
              "source_ref": _ref("map", b"controlled map declaration"),
              "local_frame_id": f.identity.local_frame_id,
              "pose_epoch": f.identity.pose_epoch, "pose_version": "map-odom-v1",
              "T_map_local": np.eye(4).tolist(),
              "evidence_refs": [_ref("pose", b"controlled absolute map-to-odom transform")],
              "calibration_id": f.identity.calibration_id}
    return capture, detection, anchor


@unittest.skipUnless(HAS_MEMORY, "memory bridge host/m7_memory.py not in this tree")
class ValidFractionConsumerTests(_Base):
    """Real producer payload -> real memory-side validation, unmodified."""

    def masks(self):
        depth, partial = square_scene(7)          # 18/25
        full = np.zeros_like(partial)
        full[10:15, 16:21] = True
        depth[10:15, 16:21] = 2.                   # 25/25
        return depth, {"1": partial, "2": full}

    def build(self, backend):
        from host.m7_memory import build_m4_document
        depth, masks = self.masks()
        engine, f, geometry, outputs = self.run_one(backend, depth, masks)
        capture, detection, anchor = _metadata(f, masks)
        return outputs, lambda: build_m4_document(
            engine, geometry, outputs, capture=capture, detection=detection,
            anchor=anchor, now_ns=f.identity.acquired_ns + 1, clock_id=f.identity.clock_id)

    def test_native_payload_passes_strict_memory_check_and_imports(self):
        from perception.offline_memory import OfflineMemory
        outputs, build = self.build(self.object_native)
        for output in outputs.values():
            self.assertEqual(output.payload["candidate"], "support_present")
            self.assertEqual(output.payload["mask_pixels"],
                             output.payload["kernel_counts"]["mask_pixels"])
        document, audit = build()
        self.assertEqual(audit["status"], "complete_m4_document")
        rows = {str(o["instance_id"]): o["geometry"]
                for o in document["batches"][0]["observations"]}
        self.assertEqual((rows["1"]["valid_depth_pixels"], rows["1"]["mask_pixels"]), (18, 25))
        self.assertEqual(rows["1"]["valid_fraction"].hex(), (18 / 25).hex())
        self.assertEqual(rows["2"]["valid_fraction"], 1.)
        memory = OfflineMemory()
        memory.import_document(deepcopy(document))
        self.assertEqual(len(memory.data["observations"]), 2)

    def test_negative_controls_are_rejected_by_memory(self):
        def drop(details):  # pre-D-029 FPGA payload
            details = dict(details)
            details.pop("valid_fraction", None)
            return details

        def replace_with(fn):
            def edit(details):
                details = dict(details)
                if "valid_fraction" in details:
                    details["valid_fraction"] = fn(details)
                return details
            return edit

        controls = {
            "missing_field": drop,
            "filled_one": replace_with(lambda d: 1.),
            "wrong_denominator_input_pixels": replace_with(
                lambda d: d["mask_valid_pixels"] / d["kernel_counts"]["input_pixels"]),
            "fixed_point_q16": replace_with(
                lambda d: round(d["valid_fraction"] * 65536) / 65536),
            "float32_rounded": replace_with(lambda d: float(np.float32(d["valid_fraction"]))),
        }
        for name, edit in controls.items():
            with self.subTest(control=name):
                outputs, build = self.build(Tamper(self.object_native, edit))
                self.assertTrue(all(o.valid for o in outputs.values()))
                with self.assertRaisesRegex(ValueError, "true mask fraction"):
                    build()
        # Positive control through the same wrapper with an identity edit.
        _, build = self.build(Tamper(self.object_native, dict))
        self.assertEqual(build()[1]["status"], "complete_m4_document")


if __name__ == "__main__":
    unittest.main()
