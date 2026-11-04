"""Producer --kv-depth-stride and kv_server --ar-depth-upsample-to (2026-10-16, AR_COLOUR_MAIN section 10).

* decimate_depth_codes: block minimum of the VALID codes (0 = invalid, all-invalid block -> 0), uint16 kept.
* scale_intrinsics: the core casts the ray of pixel u through u itself (no +0.5), so an output pixel's ray
  must be the full image's ray through the block centre.
* GeoProducer(depth_stride=): identity at 1; the ROI pixel floor stays 9 (the ROI core refuses less:
  a positive control shows min_pixels 3 is answered with status 4).
* Region verdicts (ROI core input = the decimated depth the KV gets, boxes planned on the decimated K):
  same words at full resolution and at stride 2 on the glue's reference scene, scaled up.
* upsample_snapshot: codes repeated, ROI boxes scaled, the PiP (depth / 2) is exactly the decimated frame
  and the nearest-Z reading equals the full frame's.
"""

import dataclasses
import shutil
import tempfile
from pathlib import Path
import unittest

import numpy as np

from host.geo_producer import decimate_depth_codes, scale_intrinsics


class Decimate(unittest.TestCase):
    def test_block_min_of_valid_codes(self):
        c = np.array([[5, 0, 0, 0, 7, 9],
                      [3, 4, 0, 0, 0xFFFF, 8],
                      [0, 1, 2, 2, 6, 6],
                      [9, 9, 2, 2, 6, 0]], dtype=np.uint16)
        d = decimate_depth_codes(c, 2)
        self.assertEqual(d.dtype, np.uint16)
        self.assertEqual(d.tolist(), [[3, 0, 7], [1, 2, 6]])

    def test_identity_at_1_and_crop(self):
        c = np.arange(1, 1 + 5 * 7, dtype=np.uint16).reshape(5, 7)
        self.assertIs(decimate_depth_codes(c, 1), c)
        d = decimate_depth_codes(c, 2)
        self.assertEqual(d.shape, (2, 3))
        self.assertEqual(int(d[1, 2]), int(c[2:4, 4:6].min()))
        self.assertEqual(decimate_depth_codes(np.full((2, 2), 0xFFFF, np.uint16), 2).tolist(), [[0xFFFF]])

    def test_refuses_bad_input(self):
        with self.assertRaises(ValueError):
            decimate_depth_codes(np.zeros((4, 4), np.float32), 2)
        with self.assertRaises(ValueError):
            decimate_depth_codes(np.zeros((1, 1), np.uint16), 2)

    def test_positive_control_topleft_is_not_min(self):
        c = np.array([[9, 1], [1, 1]], dtype=np.uint16)
        self.assertEqual(int(decimate_depth_codes(c, 2)[0, 0]), 1)      # top-left would give 9


class Intrinsics(unittest.TestCase):
    def test_output_ray_is_block_centre_ray(self):
        K = np.array([[517.17, 0.3, 324.22], [0., 517.12, 240.65], [0., 0., 1.]])
        for s in (2, 3):
            Ks = scale_intrinsics(K, s)
            for u, v in ((0, 0), (17, 5), (159, 119)):
                a = np.linalg.inv(Ks) @ [u, v, 1.]
                b = np.linalg.inv(K) @ [s * u + (s - 1) / 2., s * v + (s - 1) / 2., 1.]
                np.testing.assert_allclose(a, b, rtol=0, atol=1e-12)
        np.testing.assert_array_equal(scale_intrinsics(K, 1), K)

    def test_positive_control_plain_halving_is_off_by_quarter_pixel(self):
        K = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
        naive = K.copy()
        naive[:2] /= 2
        naive[2] = [0, 0, 1]
        a = np.linalg.inv(naive) @ [10, 10, 1.]
        b = np.linalg.inv(K) @ [20.5, 20.5, 1.]
        self.assertGreater(abs(a[0] - b[0]) * 500, 0.4)                # half a full pixel off


class ProducerStride(unittest.TestCase):
    def make(self, stride):
        from host.geo_producer import GeoProducer
        from host.m7_backend import M7BackendSelector
        from host.m7_frame_native import compile_frame_native
        from host.m7_rolling import compile_rolling
        tmp = tempfile.mkdtemp(prefix="stride_")
        self.addCleanup(shutil.rmtree, tmp, True)
        sel = M7BackendSelector(cpu_frame_library=compile_frame_native(Path(tmp) / "f.so"))
        roll = sel.rolling_backend("cpu", cpu_library=compile_rolling(Path(tmp) / "r.so"))
        return GeoProducer(sel.frame_backend("cpu"), roll, depth_unit_m=0.001, depth_stride=stride)

    @unittest.skipIf(shutil.which("g++") is None, "g++ needed")
    def test_prepare_and_cfg(self):
        K = np.array([[500., 0., 319.5], [0., 500., 239.5], [0., 0., 1.]])
        codes = np.random.default_rng(1).integers(0, 3000, (480, 640)).astype(np.uint16)
        p1 = self.make(1)
        c1, k1 = p1.prepare_depth(codes, K)
        self.assertIs(c1, codes)
        self.assertEqual(p1.cfg.min_pixels, 9)
        self.assertEqual(p1.describe()["depth_stride"], 1)
        p2 = self.make(2)
        c2, k2 = p2.prepare_depth(codes, K)
        self.assertEqual(c2.shape, (240, 320))
        np.testing.assert_array_equal(c2, decimate_depth_codes(codes, 2))
        np.testing.assert_allclose(k2, [[250., 0., 159.5], [0., 250., 119.5], [0., 0., 1.]])
        self.assertEqual(p2.cfg.min_pixels, 9)          # not scaled: the ROI core refuses < 9 (status 4)
        self.assertEqual(p2.describe()["roi_min_pixels"], 9)
        self.assertEqual(p2.counts["depth_decimated"], 1)
        with self.assertRaises(ValueError):
            self.make(0)


@unittest.skipIf(shutil.which("g++") is None, "g++ needed for the object-evidence reference")
class RegionVerdictsStride(unittest.TestCase):
    """The KV's ROI core reads the depth of the request (the decimated frame) and the Nano plans the
    boxes on that frame's shape and scaled K, so both sides are in decimated pixels. Words must match."""

    @classmethod
    def setUpClass(cls):
        from host.kv_link import CpuRoiCore
        from host.m7_object_native import compile_object_native
        cls.tmp = tempfile.mkdtemp()
        cls.core = CpuRoiCore(compile_object_native(Path(cls.tmp) / "object.so"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def words(self, depth, K, cfg, msg):
        from host import geometry_region_glue as glue
        from host.kv_link import check_roi_answer
        from test_geometry_region_glue import WORLD, kv_answer
        plan = glue.plan_frame(msg, depth.shape, K, np.eye(4), cfg)
        outcome = None
        if plan.batch is not None:
            outcome = check_roi_answer(plan.batch, kv_answer(self.core, depth, plan.batch))
            self.assertTrue(outcome.ok, outcome.reason)
        out = glue.verdict_message(plan, outcome, memory_msg=msg, depth_stamp_ns=10 ** 10, capture_nano_ns=5,
                                   kv_frame_id=3, world_frame=WORLD, T_world_camera=np.eye(4), pose_valid=True,
                                   cfg=cfg)
        return {v["track_id"]: v["geometry_evidence"] for v in out["verdicts"]}, plan

    def test_same_words_full_and_stride2(self):
        from host.kv_link import codes_to_m
        from perception.fpga_replay.core import Config
        from test_geometry_region_glue import memory_msg, scene
        small, tracks = scene()                                   # 64 x 48 reference scene, K f=50
        f = 8                                                     # -> 512 x 384, f = 400
        depth = np.repeat(np.repeat(small, f, 0), f, 1)
        K = np.array([[50. * f, 0., 32. * f - 0.5], [0., 50. * f, 24. * f - 0.5], [0., 0., 1.]])
        codes = np.where(np.isfinite(depth), np.rint(depth / 0.001), 0).astype(np.uint16)
        msg = memory_msg(tracks)
        cfg = Config()
        full, plan_full = self.words(codes_to_m(codes, np.float32(0.001)), K, cfg, msg)
        cfg2 = cfg                                                # the producer keeps min_pixels 9
        d2 = codes_to_m(decimate_depth_codes(codes, 2), np.float32(0.001))
        s2, plan_s2 = self.words(d2, scale_intrinsics(K, 2), cfg2, msg)
        self.assertEqual(s2, full)
        self.assertGreaterEqual(len(set(full.values())), 4)       # the scene exercises several words
        # positive control: scaling the pixel floor with the area (9 -> 3) is refused by the ROI core
        from host import geometry_region_glue as glue
        from host.kv_link import check_roi_answer
        from test_geometry_region_glue import kv_answer
        cfg3 = dataclasses.replace(cfg, min_pixels=3)
        plan3 = glue.plan_frame(msg, d2.shape, scale_intrinsics(K, 2), np.eye(4), cfg3)
        bad = check_roi_answer(plan3.batch, kv_answer(self.core, d2, plan3.batch))
        self.assertFalse(bad.ok)
        self.assertIn("0x4", bad.reason)
        # the boxes the KV core gets are in decimated pixels: half the full ones (+-1 at the edges)
        for a, b in zip(plan_full.batch.rows, plan_s2.batch.rows):
            if a.xyxy is not None and b.xyxy is not None:
                for va, vb in zip(a.xyxy, b.xyxy):
                    self.assertLessEqual(abs(va / 2. - vb), 1.0)


class Upsample(unittest.TestCase):
    def snap(self, W, H, codes, rows=None):
        return {"frame_id": 1, "codes": codes.reshape(-1), "width": W, "height": H,
                "roi": None if rows is None else {"memory_revision": 1, "rows": rows, "cfg": {}}}

    def test_codes_rows_and_untouched_cases(self):
        from host.kv_ar import upsample_snapshot
        rng = np.random.default_rng(3)
        full = rng.integers(0, 4000, (480, 640)).astype(np.uint16)
        full[rng.random(full.shape) < 0.2] = 0
        dec = decimate_depth_codes(full, 2)
        rows = [(5, 1, 10, 20, 30, 40, 0.5, 1.5), (6, 0, 0, 0, 0, 0, 0., 0.)]
        snap = self.snap(320, 240, dec, rows)
        out, s = upsample_snapshot(snap, (640, 480))
        self.assertEqual(s, 2)
        self.assertEqual((out["width"], out["height"]), (640, 480))
        up = np.asarray(out["codes"]).reshape(480, 640)
        np.testing.assert_array_equal(up[::2, ::2], dec)          # PiP (div 2) = the decimated frame
        np.testing.assert_array_equal(up[1::2, 1::2], dec)
        self.assertEqual(out["roi"]["rows"][0], (5, 1, 20, 40, 61, 81, 0.5, 1.5))
        self.assertEqual(snap["width"], 320)                       # input not mutated
        self.assertEqual(snap["roi"]["rows"][0][2], 10)
        for target in (None, (320, 240), (500, 480), (640, 360)):
            o, s1 = upsample_snapshot(snap, target)
            self.assertIs(o, snap)
            self.assertEqual(s1, 1)

    def test_nearest_z_reading_equals_full_frame(self):
        from host.kv_ar import upsample_snapshot
        from host.kv_link import codes_to_m
        from host.m7_display_native import quantize_optical_z
        rng = np.random.default_rng(4)
        for _ in range(5):
            full = rng.integers(400, 6000, (480, 640)).astype(np.uint16)
            full[rng.random(full.shape) < 0.3] = 0
            out, _ = upsample_snapshot(self.snap(320, 240, decimate_depth_codes(full, 2)), (640, 480))

            def nearest(c):
                mm = np.asarray(quantize_optical_z(codes_to_m(c, np.float32(0.001)))[0]).reshape(-1)
                return int(mm[mm != 0].min())
            self.assertEqual(nearest(np.asarray(out["codes"]).reshape(480, 640)), nearest(full))
            # positive control: top-left sampling loses the minimum in general
            tl = np.repeat(np.repeat(full[::2, ::2], 2, 0), 2, 1)
            if int(full[full > 0].min()) not in set(full[::2, ::2][full[::2, ::2] > 0].tolist()):
                self.assertNotEqual(nearest(tl), nearest(full))


class ServerFlag(unittest.TestCase):
    def test_parse_wxh(self):
        from tools.kv_link.kv_server import parse_wxh
        self.assertEqual(parse_wxh("640x480"), (640, 480))
        self.assertIsNone(parse_wxh(None))
        with self.assertRaises(ValueError):
            parse_wxh("0x480")


if __name__ == "__main__":
    unittest.main()
