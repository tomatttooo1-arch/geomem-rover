"""Independent pixel answers and pipeline provenance for M7 display candidate."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.m7_display_native import (FREE_CONFLICT, MAX_HEIGHT, MAX_WIDTH,
                                    NativeM7Display, OCCLUDED, PRESENT, UNKNOWN,
                                    compile_native, quantize_optical_z)
from host.m7_native import NativeM7, compile_native as compile_geometry
from host.m7_pipeline import (CaptureIdentity, DepthFrame, GridSpec, M7Pipeline,
                              PoseAtCapture, SemanticPacket)


# These are intended visible 3x5 glyphs, recorded independently of C++ bit
# packing. Decoding actual RGB pixels catches wrong digits, empty letters and
# stale labels even if status-bar background colours happen to look right.
GLYPHS = {
    "0": (7,5,5,5,7), "1": (2,6,2,2,7), "2": (7,1,7,4,7),
    "3": (7,1,7,1,7), "4": (5,5,7,1,1), "5": (7,4,7,1,7),
    "6": (7,4,7,5,7), "7": (7,1,1,1,1), "8": (7,5,7,5,7),
    "9": (7,5,7,1,7), "A": (2,5,7,5,5), "B": (6,5,6,5,6),
    "C": (3,4,4,4,3), "D": (6,5,5,5,6), "E": (7,4,6,4,7),
    "F": (7,4,6,4,4), "H": (5,5,7,5,5), "I": (7,2,2,2,7),
    "J": (1,1,1,5,7), "K": (5,5,6,5,5), "L": (4,4,4,4,7),
    "M": (5,7,7,5,5), "N": (5,7,7,7,5), "O": (7,5,5,5,7),
    "P": (6,5,6,4,4), "R": (6,5,6,5,5), "S": (3,4,7,1,6),
    "T": (7,2,2,2,2), "U": (5,5,5,5,7), "Z": (7,1,2,4,7),
    "m": (0,7,7,5,5), "s": (0,3,4,1,6), ":": (0,2,0,2,0),
    ".": (0,0,0,0,2), "?": (7,1,2,0,2), ">": (4,2,1,2,4),
    " ": (0,0,0,0,0),
}


def read_panel(rgb, segment, expected):
    h, w = rgb.shape
    scale = 3 if w >= 640 and h >= 24 else 1
    top = 2 if scale == 3 else 3
    assert h >= top + 5*scale and w >= 5*(len(expected)*4*scale+2*scale)
    left = segment * (w//5) + 2*scale
    for index, char in enumerate(expected):
        rows = GLYPHS[char]
        for row in range(5):
            for col in range(3):
                y = top + row*scale
                x = left + index*4*scale + col*scale
                block = rgb[y:y+scale, x:x+scale]
                expected_white = bool(rows[row] & (1 << (2-col)))
                assert np.all((block == 0xFFFFFF) == expected_white), (
                    segment, index, char, row, col, block.tolist())
    return expected


def make_frame(seq, *, depth=None, valid_pose=True):
    stamp = 1_000_000_000 + seq*10_000_000
    identity = CaptureIdentity("camera-a", seq, stamp, "camera-clock",
                               "calib-a", "odom", "pose-a")
    if depth is None:
        depth = np.full((25,25), 3.0, dtype=np.float32)
        depth[10:15,10:15] = 2.0
    k = np.array([[12.0,0,12.0],[0,12.0,12.0],[0,0,1.0]])
    pose = PoseAtCapture(stamp, "camera-clock", "odom", "pose-a",
                         valid_pose, np.eye(4))
    return DepthFrame(identity, depth, k, pose)


class M7DisplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="m7-display-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.display = NativeM7Display(compile_native(Path(cls.tmp.name) / "display.so"))
        cls.geometry = NativeM7(compile_geometry(Path(cls.tmp.name) / "geometry.so"))

    def engine(self):
        return M7Pipeline(self.geometry, GridSpec((-2.,-2.,-1.), .25, (16,16,20)))

    def emit(self, engine, frame):
        return engine.ingest_geometry(frame,
            now_ns=frame.identity.acquired_ns+2_000_000,
            now_clock_id=frame.identity.clock_id)

    def test_float_boundary_quantization_and_invalid_black(self):
        values = np.array([[.1, .1001, 2., 5., 9.9999, 10.,
                            np.nan, np.inf, -.5, 0.]], dtype=np.float64)
        mm, valid, invalid = quantize_optical_z(values)
        np.testing.assert_array_equal(mm, [[0,100,2000,5000,9999,0,0,0,0,0]])
        self.assertEqual((valid, invalid), (4, 6))
        image = np.zeros((20,320), dtype=np.float64)
        image[12,:10] = values[0]
        rendered = self.display.render(image, frame_valid=True, pose_valid=True,
                                       age_ms=5, max_age_ms=200)
        np.testing.assert_array_equal(rendered.rgb888[12,:10],
            [0,0xFF0000,0xFFFF00,0x00FFFF,0x0000FF,0,0,0,0,0])
        self.assertEqual(rendered.nearest_optical_z_mm, 100)
        self.assertEqual((rendered.valid_depth_pixels,
                          rendered.invalid_depth_pixels), (4, 6396))
        self.assertEqual(read_panel(rendered.rgb888, 0, "Z:00.10m"), "Z:00.10m")

    def test_readable_numbers_evidence_risk_and_large_age(self):
        for z, expected in ((1.23, "Z:01.23m"),
                            (2.00, "Z:02.00m"),
                            (9.99, "Z:09.99m")):
            with self.subTest(z=z):
                depth = np.full((20,320), z, dtype=np.float64)
                item = self.display.render(depth, frame_valid=True, pose_valid=True,
                                           age_ms=5, max_age_ms=2000,
                                           object_evidence=PRESENT)
                self.assertEqual(read_panel(item.rgb888, 0, expected), expected)
                self.assertEqual(read_panel(item.rgb888, 1, "A:005ms"), "A:005ms")
                self.assertEqual(read_panel(item.rgb888, 2, "POSE OK"), "POSE OK")
                self.assertEqual(read_panel(item.rgb888, 3, "OBJ HIT"), "OBJ HIT")
                self.assertEqual(read_panel(item.rgb888, 4, "RISK ?"), "RISK ?")
        depth = np.full((20,320), 2., dtype=np.float32)
        long_age = self.display.render(depth, frame_valid=True, pose_valid=True,
                                       age_ms=1001, max_age_ms=2000,
                                       object_evidence=FREE_CONFLICT)
        self.assertEqual(read_panel(long_age.rgb888, 1, "A:>999ms"), "A:>999ms")
        self.assertEqual(read_panel(long_age.rgb888, 3, "OBJ FREE?"), "OBJ FREE?")

    def test_native_width_scaled_labels_and_panel_boundary(self):
        depth = np.full((24,640), 2., dtype=np.float32)
        item = self.display.render(depth, frame_valid=True, pose_valid=True,
                                   age_ms=5, max_age_ms=200,
                                   object_evidence=FREE_CONFLICT)
        self.assertEqual(read_panel(item.rgb888, 0, "Z:02.00m"), "Z:02.00m")
        self.assertEqual(read_panel(item.rgb888, 1, "A:005ms"), "A:005ms")
        self.assertEqual(read_panel(item.rgb888, 2, "POSE OK"), "POSE OK")
        self.assertEqual(read_panel(item.rgb888, 3, "OBJ FREE?"), "OBJ FREE?")
        self.assertEqual(read_panel(item.rgb888, 4, "RISK ?"), "RISK ?")
        self.assertEqual(int(item.rgb888[20,0]), 0xFFFF00)

    def test_stale_bad_pose_and_no_depth_never_leave_old_number(self):
        depth = np.full((20,320), 2., dtype=np.float32)
        fresh = self.display.render(depth, frame_valid=True, pose_valid=True,
                                    age_ms=199, max_age_ms=200,
                                    object_evidence=PRESENT)
        stale = self.display.render(depth, frame_valid=True, pose_valid=True,
                                    age_ms=201, max_age_ms=200,
                                    object_evidence=PRESENT)
        self.assertEqual(read_panel(fresh.rgb888, 0, "Z:02.00m"), "Z:02.00m")
        self.assertEqual(read_panel(stale.rgb888, 0, "Z:??.??m"), "Z:??.??m")
        self.assertEqual(read_panel(stale.rgb888, 1, "STALE"), "STALE")
        self.assertEqual(read_panel(stale.rgb888, 2, "POSE ?"), "POSE ?")
        self.assertEqual(read_panel(stale.rgb888, 3, "OBJ ?"), "OBJ ?")
        self.assertEqual(read_panel(stale.rgb888, 4, "RISK ?"), "RISK ?")
        self.assertEqual(stale.rgb888[12,0], 0xA000A0 if (12//8)&1 else 0x180018)
        self.assertIsNone(stale.nearest_optical_z_mm)
        self.assertEqual(stale.object_evidence, UNKNOWN)
        bad_pose = self.display.render(depth, frame_valid=True, pose_valid=False,
                                       age_ms=1, max_age_ms=200)
        self.assertEqual(read_panel(bad_pose.rgb888, 2, "POSE BAD"), "POSE BAD")
        self.assertEqual(read_panel(bad_pose.rgb888, 0, "Z:??.??m"), "Z:??.??m")
        no_depth = self.display.render(np.full((20,320), np.nan, np.float32),
                                       frame_valid=True, pose_valid=True,
                                       age_ms=1, max_age_ms=200)
        self.assertEqual(read_panel(no_depth.rgb888, 1, "NODATA"), "NODATA")
        self.assertEqual(int(no_depth.rgb888[12,0]), 0)
        self.assertNotEqual(int(no_depth.rgb888[12,0]), int(stale.rgb888[12,0]))
        self.assertIsNone(no_depth.nearest_optical_z_mm)
        bad_frame = self.display.render(depth, frame_valid=False, pose_valid=True,
                                        age_ms=0, max_age_ms=200)
        self.assertEqual(read_panel(bad_frame.rgb888, 1, "BADFRAME"), "BADFRAME")
        self.assertFalse(bad_frame.fresh)

    def test_maximum_frame_and_continuous_calls_write_every_pixel(self):
        depth = np.full((MAX_HEIGHT,MAX_WIDTH), 2., dtype=np.float32)
        first = self.display.render(depth, frame_valid=True, pose_valid=True,
                                    age_ms=0, max_age_ms=200)
        self.assertEqual(first.rgb888.shape, (MAX_HEIGHT,MAX_WIDTH))
        self.assertEqual((first.valid_depth_pixels, first.invalid_depth_pixels),
                         (MAX_HEIGHT*MAX_WIDTH, 0))
        self.assertEqual(int(first.rgb888[MAX_HEIGHT-1,MAX_WIDTH-1]), 0xFFFF00)
        self.assertFalse(np.any(first.rgb888 == 0xDEADBEEF))
        second = self.display.render(np.full((MAX_HEIGHT,MAX_WIDTH), np.nan, np.float32),
                                     frame_valid=True, pose_valid=True,
                                     age_ms=0, max_age_ms=200)
        self.assertEqual((second.valid_depth_pixels, second.invalid_depth_pixels),
                         (0, MAX_HEIGHT*MAX_WIDTH))
        self.assertEqual(int(second.rgb888[MAX_HEIGHT-1,MAX_WIDTH-1]), 0)
        self.assertFalse(np.any(second.rgb888 == 0xDEADBEEF))

    def test_consumer_bridge_same_frame_late_memory_and_expiry(self):
        engine = self.engine()
        original = make_frame(1)
        geometry1 = self.emit(engine, original)
        self.assertTrue(geometry1.display.valid)
        mask = np.zeros((25,25), dtype=np.bool_)
        mask[10:15,10:15] = True
        packet1 = SemanticPacket(original.identity, geometry1.display.frame_token,
                                 "cup", mask)
        semantic1 = engine.ingest_semantic(packet1,
            now_ns=original.identity.acquired_ns+4_000_000,
            now_clock_id=original.identity.clock_id)
        self.assertEqual(semantic1.payload["candidate"], "support_present")
        current1 = self.display.render_consumer_output(engine, original, geometry1.display,
            now_ns=original.identity.acquired_ns+5_000_000,
            clock_id=original.identity.clock_id, semantic_output=semantic1)
        self.assertTrue(current1.fresh)
        self.assertEqual(current1.object_evidence, PRESENT)
        self.assertEqual(current1.nearest_optical_z_mm, 2000)
        original.depth_m[:] = 1.0  # bridge must use the cached original depth
        still = self.display.render_consumer_output(engine, original, geometry1.display,
            now_ns=original.identity.acquired_ns+6_000_000,
            clock_id=original.identity.clock_id, semantic_output=semantic1)
        self.assertEqual(still.nearest_optical_z_mm, 2000)
        next_frame = make_frame(2, depth=np.full((25,25), 3., np.float32))
        geometry2 = self.emit(engine, next_frame)
        old_memory_current_display = self.display.render_consumer_output(
            engine, next_frame, geometry2.display,
            now_ns=next_frame.identity.acquired_ns+5_000_000,
            clock_id=next_frame.identity.clock_id, semantic_output=semantic1)
        self.assertTrue(old_memory_current_display.fresh)
        self.assertEqual(old_memory_current_display.object_evidence, UNKNOWN)
        self.assertEqual(old_memory_current_display.nearest_optical_z_mm, 3000)
        bounds = np.array([[-.25,-.25,1.5],[.25,.25,2.5]])
        packet2 = SemanticPacket(next_frame.identity, geometry2.display.frame_token,
                                 "cup", np.zeros((25,25), np.bool_), bounds)
        semantic2 = engine.ingest_semantic(packet2,
            now_ns=next_frame.identity.acquired_ns+4_000_000,
            now_clock_id=next_frame.identity.clock_id)
        self.assertEqual(semantic2.payload["candidate"], "free_space_conflict")
        current_conflict = self.display.render_consumer_output(engine, next_frame,
            geometry2.display, now_ns=next_frame.identity.acquired_ns+5_000_000,
            clock_id=next_frame.identity.clock_id, semantic_output=semantic2)
        self.assertEqual(current_conflict.object_evidence, FREE_CONFLICT)
        old_display = self.display.render_consumer_output(engine, original, geometry1.display,
            now_ns=next_frame.identity.acquired_ns+5_000_000,
            clock_id=next_frame.identity.clock_id, semantic_output=semantic1)
        self.assertFalse(old_display.fresh)
        self.assertIsNone(old_display.nearest_optical_z_mm)
        self.assertEqual(int(old_display.rgb888[12,0]), 0xA000A0)
        expired = self.display.render_consumer_output(engine, next_frame, geometry2.display,
            now_ns=geometry2.display.expires_at_ns+1,
            clock_id=next_frame.identity.clock_id)
        self.assertFalse(expired.fresh)
        self.assertEqual(expired.reason, "expired_output")
        self.assertIsNone(expired.nearest_optical_z_mm)

    def test_consumer_bridge_rejects_wrong_identity_channel_and_clock(self):
        engine = self.engine()
        frame = make_frame(1)
        outputs = self.emit(engine, frame)
        current_time = frame.identity.acquired_ns+5_000_000
        other = make_frame(2)
        mismatch = self.display.render_consumer_output(engine, other, outputs.display,
            now_ns=current_time, clock_id=frame.identity.clock_id)
        self.assertFalse(mismatch.fresh)
        self.assertEqual(mismatch.reason, "display_identity_or_token_mismatch")
        wrong_channel = self.display.render_consumer_output(engine, frame,
            outputs.navigation, now_ns=current_time,
            clock_id=frame.identity.clock_id)
        self.assertFalse(wrong_channel.fresh)
        self.assertEqual(wrong_channel.reason, "display_identity_or_token_mismatch")
        wrong_clock = self.display.render_consumer_output(engine, frame,
            outputs.display, now_ns=current_time, clock_id="wrong-clock")
        self.assertFalse(wrong_clock.fresh)
        self.assertEqual(wrong_clock.reason, "clock_mismatch")
        self.assertEqual(wrong_clock.object_evidence, UNKNOWN)

    def test_bad_types_and_capacity_fail_explicitly(self):
        with self.assertRaises(ValueError):
            quantize_optical_z(np.ones((MAX_HEIGHT+1, 1), dtype=np.float32))
        with self.assertRaises(ValueError):
            quantize_optical_z(np.ones((1, 1), dtype=np.uint16))
        with self.assertRaises(ValueError):
            self.display.render(np.ones((1,1), np.float32), frame_valid=1,
                                pose_valid=True, age_ms=1, max_age_ms=2)
        with self.assertRaises(ValueError):
            self.display.render(np.ones((1,1), np.float32), frame_valid=True,
                                pose_valid=True, age_ms=1, max_age_ms=2,
                                object_evidence=99)


    def test_region_occluded_label_and_channel_mapping(self):
        for w, h in ((320, 20), (640, 480)):
            depth = np.full((h, w), 2., dtype=np.float32)
            item = self.display.render(depth, frame_valid=True, pose_valid=True,
                                       age_ms=5, max_age_ms=200, object_evidence=OCCLUDED)
            self.assertEqual(read_panel(item.rgb888, 3, "OBJ OCCL"), "OBJ OCCL")
            self.assertEqual(item.object_evidence, OCCLUDED)
            free = self.display.render(depth, frame_valid=True, pose_valid=True,
                                       age_ms=5, max_age_ms=200, object_evidence=FREE_CONFLICT)
            seg = w // 5 * 3 + 1
            self.assertNotEqual(int(item.rgb888[h - 2 if h < 24 else 1, seg + w // 5 - 2]), 0)
            self.assertNotEqual(int(item.rgb888[1, seg + w // 5 - 2]),
                                int(free.rgb888[1, seg + w // 5 - 2]))
            stale = self.display.render(depth, frame_valid=True, pose_valid=True,
                                        age_ms=201, max_age_ms=200, object_evidence=OCCLUDED)
            self.assertEqual(read_panel(stale.rgb888, 3, "OBJ ?"), "OBJ ?")
        from host.m7_object_native import NativeObjectEvidence, compile_object_native
        from host.m7_pipeline import RegionCheckPacket
        region = NativeObjectEvidence(compile_object_native(Path(self.tmp.name) / "roi.so"))
        bounds = np.array([[-.25,-.25,1.5],[.25,.25,2.5]])
        depth = np.full((25,25), 3., np.float32)
        depth[10:15,10:15] = 1.  # something in front of the remembered object
        engine = M7Pipeline(self.geometry, GridSpec((-2.,-2.,-1.), .25, (16,16,20)),
                            region_backend=region)
        f = make_frame(1, depth=depth)
        g = self.emit(engine, f)
        (out,) = engine.ingest_region_checks(
            RegionCheckPacket(f.identity, g.display.frame_token, (("cup", bounds),)),
            now_ns=f.identity.acquired_ns+3_000_000, now_clock_id=f.identity.clock_id)
        self.assertEqual(out.payload["verdict"], "occluded")
        shown = self.display.render_consumer_output(engine, f, g.display,
            now_ns=f.identity.acquired_ns+5_000_000, clock_id=f.identity.clock_id,
            semantic_output=out)
        self.assertEqual(shown.object_evidence, OCCLUDED)
        # The mask path's occlusion stays "OBJ ?".
        sem = engine.ingest_semantic(SemanticPacket(f.identity, g.display.frame_token, "cup",
                                                    np.zeros((25,25), np.bool_), bounds),
            now_ns=f.identity.acquired_ns+3_000_000, now_clock_id=f.identity.clock_id)
        self.assertEqual(sem.reason, "historical_region_occluded")
        shown = self.display.render_consumer_output(engine, f, g.display,
            now_ns=f.identity.acquired_ns+5_000_000, clock_id=f.identity.clock_id,
            semantic_output=sem)
        self.assertEqual(shown.object_evidence, UNKNOWN)

if __name__ == "__main__":
    unittest.main()
