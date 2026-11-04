"""DPUPIPE0 (2026-11-03): host/dpu_prep.py — Nano-side DPU input preprocessing must be byte-identical to
dev3's evaluator (ultralytics 8.4.69 validator pixels + the xmodel's input data-fix).

  * quant_lut vs the float32 path dev3 runs (pixels.astype(float32)/255. then x*2^fp, DPU_ROUND and rint)
  * letterbox geometry of the car camera at 640/512/416
  * numpy INTER_LINEAR replica vs cv2.resize (random images, several geometries) + a negative control
  * golden sha256 of full tensors for a fixed synthetic frame (the implementation that matched the
    ultralytics validator on 74 real images, DPUPIPE0/results/prep_consistency.json)
  * opt-in (DPUPIPE_ULTRA=1, ~1 min, needs ultralytics): the same synthetic frames through the ultralytics
    validator dataloader itself.
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_dpu_prep.py
"""
import hashlib
import os
import tempfile
import unittest

import numpy as np

from host import dpu_prep

GOLDEN = {  # (src_w, src_h, size) -> sha256 of DpuPrep(size).full_tensor(synth(src_w, src_h, 7))
    (640, 480, 640): "c0e4d221c4db68fc124272532e1fb585455604e2e3ef26d2b71b4ac42af3c28f",
    (640, 480, 512): "fad9d985c42c29546327b1a815803d9883042a6e7348b7bb8bc1adba1f986e10",
    (640, 480, 416): "e95276fcb847abf56e807f71686d7ea0b9ad87ac38ee935717681cf666ae8427",
    (1280, 720, 640): "5769be11ba64270f81638caf089188383cd7633856f553b19d75fd758ab75243",
    (1280, 720, 512): "77874e8afa75cc7b40c20ed917d1c6efe3fa723b699b1418a6baefd843f7170e",
    (1280, 720, 416): "427a0d60e07ea9b3f00c47b5d85f1248e0a91a282f0fe75de7fc563580674e14",
}


def synth(w, h, seed):
    r = np.random.RandomState(seed)
    y, x = np.mgrid[0:h, 0:w]
    im = np.stack([(x * 255) // (w - 1), (y * 255) // (h - 1), ((x // 7 + y // 5) * 37) % 256], -1).astype(np.int32)
    im = im + r.randint(-20, 21, (h, w, 3))
    return np.clip(im, 0, 255).astype(np.uint8)


def float_quant(px, fp, mode):
    x = px.astype(np.float32) / 255.
    y = x * np.float32(2 ** fp)
    q = np.floor(y + np.float32(0.5)) if mode == "dpu_round" else np.rint(y)
    return np.clip(q, -128, 127).astype(np.int8)


try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None


class QuantLut(unittest.TestCase):
    def test_matches_float_path_both_roundings(self):
        px = np.arange(256, dtype=np.uint8)
        for fp in range(8):
            lut = dpu_prep.quant_lut(fp)
            np.testing.assert_array_equal(lut, float_quant(px, fp, "dpu_round"))
            np.testing.assert_array_equal(lut, float_quant(px, fp, "rint"))

    def test_known_values(self):
        self.assertEqual(int(dpu_prep.quant_lut(6)[114]), 29)
        self.assertEqual(int(dpu_prep.quant_lut(7)[114]), 57)
        self.assertEqual(int(dpu_prep.quant_lut(7)[255]), 127)   # 128 saturates
        self.assertEqual(int(dpu_prep.quant_lut(6)[255]), 64)
        self.assertEqual(dpu_prep.DEFAULT_FIX_POINT, {640: 6, 512: 7, 416: 7})


class Geometry(unittest.TestCase):
    def test_car_camera(self):
        for size, h, top in ((640, 480, 80), (512, 384, 64), (416, 312, 52)):
            g = dpu_prep.LetterboxGeometry(640, 480, size)
            self.assertEqual((g.w, g.h, g.top, g.bottom, g.left, g.right), (size, h, top, top, 0, 0))
            self.assertTrue(g.contiguous)
            self.assertEqual(g.content_offset, top * size * 3)
            self.assertGreaterEqual(g.content_offset, 64)          # room for the 64-byte FRAME_DPU head

    def test_other_shapes(self):
        g = dpu_prep.LetterboxGeometry(1280, 720, 640)
        self.assertEqual((g.load_w, g.load_h, g.top), (640, 360, 140))
        g = dpu_prep.LetterboxGeometry(480, 640, 640)
        self.assertEqual((g.w, g.h, g.left, g.top), (480, 640, 80, 0))
        self.assertFalse(g.contiguous)


@unittest.skipIf(cv2 is None, "cv2 missing")
class ResizeReplica(unittest.TestCase):
    def test_bit_exact_vs_cv2(self):
        rng = np.random.RandomState(3)
        for sw, sh, w, h in ((640, 480, 512, 384), (640, 480, 416, 312), (1280, 720, 640, 360),
                             (1280, 720, 416, 234), (500, 375, 416, 312), (641, 479, 416, 311)):
            for _ in range(2):
                im = rng.randint(0, 256, (sh, sw, 3)).astype(np.uint8)
                ref = cv2.resize(im, (w, h), interpolation=cv2.INTER_LINEAR)
                np.testing.assert_array_equal(dpu_prep.resize_linear_numpy(im, w, h), ref)

    def test_enlarging_refused(self):
        with self.assertRaises(NotImplementedError):
            dpu_prep.resize_linear_numpy(np.zeros((240, 320, 3), np.uint8), 416, 312)

    def test_negative_control_detects_rounding_change(self):
        # the comparison above has discriminating power: a rounding variant of the vertical pass differs
        im = np.random.RandomState(4).randint(0, 256, (480, 640, 3)).astype(np.uint8)
        ref = cv2.resize(im, (416, 312), interpolation=cv2.INTER_LINEAR)
        plan = dpu_prep._LinearPlan(640, 480, 416, 312)
        S = im[plan.rows].astype(np.int64)
        H = S[:, plan.x0] * plan.xa0 + S[:, plan.x1] * plan.xa1
        alt = (plan.yb0 * H[plan.r0] + plan.yb1 * H[plan.r1] + (1 << 21)) >> 22
        self.assertGreater(int((np.clip(alt, 0, 255).astype(np.uint8) != ref).sum()), 1000)

    def test_cv2_backend_equals_numpy_backend(self):
        im = synth(640, 480, 11)
        for size in (640, 512, 416):
            a = dpu_prep.DpuPrep(size, backend="numpy").content(im)
            b = dpu_prep.DpuPrep(size, backend="cv2").content(im)
            self.assertEqual(a[1], b[1])


class Golden(unittest.TestCase):
    def test_golden_full_tensors(self):
        for (w, h, size), sha in GOLDEN.items():
            t = dpu_prep.DpuPrep(size, backend="numpy").full_tensor(synth(w, h, 7))
            self.assertEqual(t.shape, (size, size, 3))
            self.assertEqual(hashlib.sha256(t.tobytes()).hexdigest(), sha, (w, h, size))

    def test_pad_and_content_placement(self):
        p = dpu_prep.DpuPrep(512)
        im = synth(640, 480, 1)
        g, content, _ = p.content(im)
        t = p.full_tensor(im)
        flat = t.view(np.uint8).reshape(-1)
        self.assertEqual(flat[g.content_offset:g.content_offset + g.content_bytes].tobytes(), content)
        self.assertTrue((t[:g.top] == p.pad_q).all() and (t[g.top + g.h:] == p.pad_q).all())


@unittest.skipUnless(os.environ.get("DPUPIPE_ULTRA") == "1", "opt-in: DPUPIPE_ULTRA=1 (ultralytics validator)")
class AgainstUltralytics(unittest.TestCase):
    def test_validator_pixels(self):
        os.environ["CUDA_VISIBLE_DEVICES"] = ""      # never touch a GPU (host GPU guard kills such processes)
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools", "dpu_pipe"))
        import prep_consistency as pc
        with tempfile.TemporaryDirectory() as d:
            paths = []
            for w, h in ((640, 480), (1280, 720)):
                p = os.path.join(d, "s_%d.png" % w)
                cv2.imwrite(p, synth(w, h, 7)[..., ::-1])
                paths.append(p)
            for size in (640, 512, 416):
                ref = pc.reference_pixels(paths, size, d)
                fp = dpu_prep.DEFAULT_FIX_POINT[size]
                for p, (w, h) in zip(paths, ((640, 480), (1280, 720))):
                    rgb = cv2.imread(p)[..., ::-1].copy()
                    full = dpu_prep.DpuPrep(size, backend="numpy").full_tensor(rgb)
                    np.testing.assert_array_equal(full, pc.ref_quant(ref[os.path.abspath(p)], fp, "dpu_round"))
                    self.assertEqual(hashlib.sha256(full.tobytes()).hexdigest(), GOLDEN[(w, h, size)])


if __name__ == "__main__":
    unittest.main()
