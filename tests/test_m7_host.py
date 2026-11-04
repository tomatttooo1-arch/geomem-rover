"""M7 stream/cache contract tests against the independently compiled native ray core."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.m7_native import NativeM7, compile_native
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)


GRID = GridSpec((-2., -2., -1.), .25, (16, 16, 20))
BOUNDS = np.array([[-.25, -.25, 1.5], [.25, .25, 2.5]])


def frame(seq=1, *, stamp=None, stream="usb-restart-a", calibration="depth-25-v1",
          epoch="odom-1", depth=None, pose_stamp=None, pose_valid=True, k=None, transform=None):
    stamp = 1_000_000_000 + seq * 10_000_000 if stamp is None else stamp
    identity = CaptureIdentity(stream, seq, stamp, "camera-sync-1", calibration,
                               "odom", epoch)
    if depth is None:
        depth = np.full((25, 25), 3., np.float32)
        depth[10:15, 10:15] = 2.
    if k is None:
        k = np.array([[12., 0., 12.], [0., 12., 12.], [0., 0., 1.]])
    if transform is None:
        transform = np.eye(4)
    pose = PoseAtCapture(stamp if pose_stamp is None else pose_stamp, identity.clock_id,
                         identity.local_frame_id, identity.pose_epoch, pose_valid, transform)
    return DepthFrame(identity, depth, k, pose)


def packet(geometry, f, mask=None, bounds=None, object_id="cup-1"):
    return SemanticPacket(f.identity, geometry.navigation.frame_token, object_id, mask, bounds)


class M7HostContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="m7-host-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.backend = NativeM7(compile_native(Path(cls.temp.name) / "m7.so"))

    def engine(self, **kwargs):
        return M7Pipeline(self.backend, GRID, **kwargs)

    def emit(self, engine, f, *, delay=2_000_000):
        return engine.ingest_geometry(f, now_ns=f.identity.acquired_ns + delay,
                                      now_clock_id=f.identity.clock_id)

    def semantic(self, engine, pkt, f, *, delay=4_000_000):
        return engine.ingest_semantic(pkt, now_ns=f.identity.acquired_ns + delay,
                                      now_clock_id=f.identity.clock_id)

    def test_geometry_precedes_semantic_and_has_real_flags_and_expiry(self):
        engine = self.engine()
        f = frame()
        geometry = self.emit(engine, f)
        nav, display = geometry.navigation, geometry.display
        self.assertTrue(nav.valid)
        self.assertEqual(nav.backend, "cpu_reference")
        self.assertEqual(nav.identity, f.identity)
        self.assertGreater(nav.payload["valid_grid_samples"], 0)
        self.assertEqual(nav.payload["input_pixels"], 625)
        self.assertEqual(sum(nav.payload[key] for key in
                         ("valid_grid_samples", "invalid_depth_or_projection", "outside_grid")), 625)
        self.assertGreater(nav.payload["outside_grid"], 0)
        self.assertEqual(display.payload["outside_grid"], nav.payload["outside_grid"])
        self.assertEqual(nav.payload["voxel_flags"].shape, (20, 16, 16))
        self.assertEqual(nav.payload["voxel_flags"][12, 8, 8], 2)
        self.assertEqual(nav.payload["voxel_flags"][10, 8, 8], 1)
        self.assertEqual(display.payload["stage"], "board_overlay_input_only")
        self.assertEqual(display.payload["risk_status"], "not_computed")
        self.assertEqual(display.payload["nearest_optical_z_m"], 2.)
        self.assertFalse(nav.validate_at(engine, nav.expires_at_ns + 1, f.identity.clock_id).valid)
        self.assertFalse(nav.validate_at(engine, f.identity.acquired_ns + 1, "unrelated").valid)
        self.assertTrue(nav.validate_at(engine, nav.expires_at_ns, f.identity.clock_id).valid)

    def test_present_removed_occlusion_missing_and_invalid_depth(self):
        cases = (("present", 2., True, "support_present"),
                 ("removed", 3., False, "free_space_conflict"),
                 ("occluded", 1., False, "indeterminate"),
                 ("uncertain", 2., False, "indeterminate"))
        for _, depth_value, positive, expected in cases:
            with self.subTest(positive=positive, expected=expected, depth=depth_value):
                depth = np.full((25, 25), 3., np.float32)
                depth[10:15, 10:15] = depth_value
                f = frame(depth=depth)
                engine = self.engine()
                geometry = self.emit(engine, f)
                mask = np.zeros((25, 25), dtype=np.bool_)
                if positive:
                    mask[10:15, 10:15] = True
                evidence = self.semantic(engine, packet(geometry, f, mask, BOUNDS), f)
                self.assertTrue(evidence.valid)
                self.assertEqual(evidence.payload["candidate"], expected)
                self.assertEqual(evidence.identity, f.identity)
                self.assertEqual(evidence.backend, "cpu_reference")
                self.assertEqual(evidence.payload["association"], "upstream_object_id_unverified")
        f = frame(depth=np.full((25, 25), 3., np.float32))
        engine = self.engine()
        geometry = self.emit(engine, f)
        missing = self.semantic(engine, packet(geometry, f, None, BOUNDS), f)
        self.assertEqual(missing.payload["candidate"], "indeterminate")
        self.assertEqual(missing.reason, "missing_mask")
        invalid = frame(depth=np.full((25, 25), np.nan, np.float32))
        bad_engine = self.engine()
        result = self.emit(bad_engine, invalid)
        self.assertFalse(result.navigation.valid)
        self.assertFalse(result.display.valid)
        self.assertEqual(bad_engine.cached_frames, 0)

    def test_copies_depth_intrinsics_pose_and_separates_output_arrays(self):
        engine = self.engine()
        f = frame()
        geometry = self.emit(engine, f)
        display_snapshot = geometry.display.payload["voxel_flags"].copy()
        geometry.navigation.payload["voxel_flags"][:] = 0
        np.testing.assert_array_equal(geometry.display.payload["voxel_flags"], display_snapshot)
        f.depth_m[:] = 1.
        f.K[0, 0] = 999.
        f.pose.T_local_camera[0, 3] = 15.
        positive = np.zeros((25, 25), dtype=np.bool_)
        positive[10:15, 10:15] = True
        evidence = self.semantic(engine, packet(geometry, f, positive, BOUNDS), f)
        self.assertEqual(evidence.payload["candidate"], "support_present")
        np.testing.assert_allclose(evidence.payload["position_local_m"], [0., 0., 2.])

    def test_no_silent_latest_frame_substitution_wrong_identity_and_token(self):
        engine = self.engine()
        old = frame(1)
        original = self.emit(engine, old)
        new = frame(2, depth=np.full((25, 25), 3., np.float32))
        self.emit(engine, new)
        mask = np.zeros((25, 25), dtype=np.bool_)
        mask[10:15, 10:15] = True
        evidence = self.semantic(engine, packet(original, old, mask), old)
        self.assertEqual(evidence.payload["candidate"], "support_present")
        self.assertEqual(evidence.payload["position_local_m"][2], 2.)
        wrong_frame = self.semantic(engine, packet(original, new, mask), new)
        self.assertFalse(wrong_frame.valid)
        self.assertEqual(wrong_frame.reason, "frame_token_mismatch")
        wrong_calibration = replace(original.navigation.identity, calibration_id="other")
        self.assertFalse(self.semantic(engine, SemanticPacket(wrong_calibration,
                         original.navigation.frame_token, "another", mask), old).valid)
        wrong_timestamp = replace(old.identity, acquired_ns=old.identity.acquired_ns + 1)
        self.assertFalse(self.semantic(engine, SemanticPacket(wrong_timestamp,
                         original.navigation.frame_token, "another", mask), old).valid)
        self.assertEqual(self.semantic(engine, packet(original, old, mask), old).reason,
                         "duplicate_object_frame")
        wrong_resolution = np.zeros((24, 25), dtype=np.bool_)
        self.assertEqual(self.semantic(engine, packet(original, old, wrong_resolution,
                              object_id="other"), old).reason,
                         "mask_resolution_or_type_mismatch")

    def test_newer_geometry_then_late_original_semantic_uses_old_depth(self):
        engine = self.engine()
        old_frame = frame(1)
        original = self.emit(engine, old_frame)
        removed_frame = frame(2, depth=np.full((25, 25), 3., np.float32))
        current = self.emit(engine, removed_frame)
        mask = np.zeros((25, 25), dtype=np.bool_)
        mask[10:15, 10:15] = True
        delayed = engine.ingest_semantic(packet(original, old_frame, mask, BOUNDS),
                 now_ns=removed_frame.identity.acquired_ns + 3_000_000,
                 now_clock_id=removed_frame.identity.clock_id)
        self.assertTrue(delayed.valid)
        self.assertEqual(delayed.payload["candidate"], "support_present")
        self.assertEqual(delayed.payload["position_local_m"][2], 2.)
        current_negative = self.semantic(engine, packet(current, removed_frame,
                                np.zeros_like(mask), BOUNDS), removed_frame)
        self.assertEqual(current_negative.payload["candidate"], "free_space_conflict")

    def test_present_mask_mismatch_cannot_confirm_historical_object(self):
        engine = self.engine()
        f = frame()
        geometry = self.emit(engine, f)
        mask = np.zeros((25, 25), dtype=np.bool_)
        mask[10:15, 10:15] = True
        distant_bounds = np.array([[1., 1., 1.], [1.2, 1.2, 1.2]])
        evidence = self.semantic(engine, packet(geometry, f, mask, distant_bounds), f)
        self.assertTrue(evidence.valid)
        self.assertEqual(evidence.payload["candidate"], "indeterminate")
        self.assertEqual(evidence.reason, "mask_not_in_historical_bounds")

    def test_host_candidate_retains_old_conservative_roi_fraction_boundary(self):
        for free_count, expected in ((22, "indeterminate"),
                                     (23, "free_space_conflict")):
            with self.subTest(free_count=free_count):
                depth = np.full((25, 25), 3., np.float32)
                patch = depth[10:15, 10:15]
                patch.flat[free_count:] = 2.
                engine = self.engine()
                f = frame(depth=depth)
                geometry = self.emit(engine, f)
                evidence = self.semantic(engine, packet(geometry, f,
                         np.zeros((25, 25), np.bool_), BOUNDS), f)
                self.assertEqual(evidence.payload["candidate"], expected)
        for invalid_depth in (np.nan, .1, 10.):
            with self.subTest(invalid_depth=invalid_depth):
                depth = np.full((25, 25), 3., np.float32)
                depth[10:15, 10:15].flat[:8] = invalid_depth  # 17/25 valid: below .7
                engine = self.engine()
                f = frame(depth=depth)
                geometry = self.emit(engine, f)
                evidence = self.semantic(engine, packet(geometry, f,
                         np.zeros((25, 25), np.bool_), BOUNDS), f)
                self.assertEqual(evidence.payload["candidate"], "indeterminate")
                self.assertEqual(evidence.reason, "invalid_historical_region_depth")

    def test_backend_drop_count_corruption_invalidates_frame(self):
        class BadCounts:
            def __init__(self, delegate, field, value):
                self.delegate, self.field, self.value = delegate, field, value

            def frame_from_depth(self, **kwargs):
                evidence = self.delegate.frame_from_depth(**kwargs)
                return replace(evidence, **{self.field: self.value(evidence)})

        for field, mutate in (("outside_grid", lambda e: e.outside_grid + 1),
                              ("invalid_depth_or_projection", lambda e: -1)):
            with self.subTest(field=field):
                engine = M7Pipeline(BadCounts(self.backend, field, mutate), GRID)
                result = self.emit(engine, frame())
                self.assertFalse(result.navigation.valid)
                self.assertFalse(result.display.valid)
                self.assertEqual(result.navigation.payload, {})
                self.assertEqual(result.navigation.reason, "backend_failure:ValueError")
                self.assertEqual(engine.cached_frames, 0)

    def test_eviction_expiry_and_bounded_object_counts(self):
        engine = self.engine(max_frames=1, max_objects_per_frame=1)
        first = frame(1)
        g1 = self.emit(engine, first)
        second = frame(2)
        g2 = self.emit(engine, second)
        self.assertEqual(engine.cached_frames, 1)
        self.assertEqual(self.semantic(engine, packet(g1, first), first).reason,
                         "frame_not_cached")
        self.assertTrue(self.semantic(engine, packet(g2, second), second).valid)
        self.assertEqual(self.semantic(engine, packet(g2, second,
                              object_id="another"), second).reason,
                         "object_capacity_exceeded")
        self.assertEqual(engine.ingest_semantic(packet(g2, second, object_id="another"),
                         now_ns=second.identity.acquired_ns + engine.max_cache_age_ns + 1,
                         now_clock_id=second.identity.clock_id).reason, "expired_frame")
        self.assertFalse(g2.navigation.validate_at(engine, second.identity.acquired_ns +
                         engine.max_frame_age_ns + 1, second.identity.clock_id).valid)

    def test_previous_navigation_revoked_on_new_frame_reset_and_pose_loss(self):
        engine = self.engine()
        f = frame(1)
        old = self.emit(engine, f)
        check_ns = f.identity.acquired_ns + 5_000_000
        self.assertTrue(old.navigation.validate_at(engine, check_ns, f.identity.clock_id).valid)
        newer_frame = frame(2)
        newer = self.emit(engine, newer_frame)
        self.assertFalse(old.navigation.validate_at(engine, check_ns, f.identity.clock_id).valid)
        # Historical evidence may still consume its own frame after new geometry.
        m = self.semantic(engine, packet(old, f, None, BOUNDS), f)
        self.assertTrue(m.validate_at(engine, check_ns, f.identity.clock_id).valid)
        failed = self.emit(engine, frame(3, pose_valid=False))
        self.assertFalse(failed.navigation.valid)
        self.assertFalse(newer.display.validate_at(engine,
                         newer_frame.identity.acquired_ns + 5_000_000,
                         f.identity.clock_id).valid)
        self.assertTrue(m.validate_at(engine, check_ns, f.identity.clock_id).valid)
        engine.reset()
        self.assertFalse(m.validate_at(engine, check_ns, f.identity.clock_id).valid)
        self.assertFalse(old.navigation.validate_at(engine, check_ns, f.identity.clock_id).valid)

    def test_accepted_invalid_depth_and_backend_failure_revoke_prior_geometry(self):
        for failing_frame in (frame(2, depth=np.full((25, 25), np.nan, np.float32)),
                              frame(2, transform=np.array([[1., 0., 0., 20.],
                              [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]))):
            with self.subTest(depth_failure=np.isnan(failing_frame.depth_m).all()):
                engine = self.engine()
                old_frame = frame(1)
                old = self.emit(engine, old_frame)
                self.assertTrue(old.navigation.validate_at(engine,
                    old_frame.identity.acquired_ns + 1, old_frame.identity.clock_id).valid)
                failed = self.emit(engine, failing_frame)
                self.assertFalse(failed.navigation.valid)
                self.assertFalse(old.navigation.validate_at(engine,
                    old_frame.identity.acquired_ns + 1, old_frame.identity.clock_id).valid)

    def test_invalid_pose_late_duplicate_epoch_calibration_and_time_domain(self):
        engine = self.engine()
        invalid = frame(pose_valid=False)
        self.assertFalse(self.emit(engine, invalid).navigation.valid)
        self.assertEqual(engine.cached_frames, 0)
        self.assertEqual(self.emit(engine, frame(2, pose_stamp=0)).navigation.reason,
                         "pose_outside_capture_window")
        future = frame(3, pose_stamp=frame(3).identity.acquired_ns + 1)
        self.assertEqual(self.emit(engine, future).navigation.reason,
                         "pose_outside_capture_window")
        good = frame(4)
        g = self.emit(engine, good)
        self.assertTrue(g.navigation.valid)
        self.assertEqual(self.emit(engine, good).navigation.reason, "duplicate_geometry")
        self.assertEqual(self.emit(engine, frame(3)).navigation.reason, "duplicate_geometry")
        self.assertEqual(self.emit(engine, frame(5, stamp=good.identity.acquired_ns-1)).navigation.reason,
                         "late_geometry")
        self.assertEqual(self.emit(engine, frame(5, epoch="map-correction")).navigation.reason,
                         "context_changed_reset_required")
        engine.reset()
        self.emit(engine, good)
        self.assertEqual(self.emit(engine, frame(5, calibration="depth-v2")).navigation.reason,
                         "context_changed_reset_required")
        engine.reset()
        self.emit(engine, good)
        changed_k = frame(5)
        changed_k.K[0, 0] = 20.
        self.assertEqual(self.emit(engine, changed_k).navigation.reason,
                         "calibration_content_changed_reset_required")
        engine.reset()
        self.emit(engine, good)
        different_shape = frame(5, depth=np.full((24, 25), 3., np.float32))
        self.assertEqual(self.emit(engine, different_shape).navigation.reason,
                         "calibration_content_changed_reset_required")
        self.assertEqual(engine.ingest_geometry(frame(5), now_ns=frame(5).identity.acquired_ns+1,
                         now_clock_id="other").navigation.reason, "clock_mismatch")
        self.assertEqual(self.emit(engine, frame(5), delay=engine.max_frame_age_ns+1).navigation.reason,
                         "expired_frame")
        self.assertEqual(engine.ingest_geometry(frame(5), now_ns=frame(5).identity.acquired_ns-1,
                         now_clock_id="camera-sync-1").navigation.reason, "future_frame")
        engine.reset()
        newer = self.emit(engine, frame(1, epoch="map-correction"))
        self.assertTrue(newer.navigation.valid)
        old_packet = packet(g, good)
        self.assertFalse(self.semantic(engine, old_packet, good).valid)
        self.assertEqual(engine.cached_frames, 1)

    def test_malformed_inputs_and_backend_failure_never_publish_free(self):
        engine = self.engine(max_pixels=625)
        with self.assertRaises(ValueError):
            self.emit(engine, frame(depth=np.ones((26, 25), dtype=np.float32)))
        with self.assertRaises(ValueError):
            self.emit(engine, frame(depth=np.full((25, 25), np.inf)))
        bad = frame()
        bad.pose.T_local_camera[0, 0] = 2.
        with self.assertRaises(ValueError):
            self.emit(engine, bad)
        with self.assertRaises(ValueError):
            CaptureIdentity("stream", 1, 1.0, "clock", "calib", "odom", "epoch")
        with self.assertRaises(ValueError):
            M7Pipeline(self.backend, GRID, max_frames=0)
        far = frame(depth=np.full((25, 25), 3., np.float32),
                    transform=np.diag([1., 1., 1., 1.]))
        far.pose.T_local_camera[:3, 3] = [20., 0., 0.]
        failed = self.emit(self.engine(), far)
        self.assertFalse(failed.navigation.valid)
        self.assertFalse(failed.display.valid)
        self.assertEqual(failed.navigation.payload, {})
        self.assertIn("backend_failure", failed.navigation.reason)


if __name__ == "__main__":
    unittest.main()
