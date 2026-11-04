"""Host-side checks of the pure helpers in nano_detector.py (no camera, no GPU).
Run: python3 -m unittest discover -s tools/nano_detector -p 'test_*.py'"""
import importlib.util
import unittest
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location("nano_detector", Path(__file__).with_name("nano_detector.py"))
nd = importlib.util.module_from_spec(spec); spec.loader.exec_module(nd)

class NanoDetectorHelpers(unittest.TestCase):
    def test_rle_roundtrip_and_zero_first_run(self):
        rng = np.random.default_rng(0)
        for shape in [(1, 1), (3, 5), (17, 9), (66, 27)]:
            m = rng.random(shape) > 0.6
            r = nd.rle_encode(m)
            assert sum(r) == m.size
            assert np.array_equal(nd.rle_decode(r, shape), m)
        assert nd.rle_encode(np.ones((2, 2), bool)) == [0, 4]      # first run counts zeros
        assert nd.rle_encode(np.zeros((2, 2), bool)) == [4]
        assert nd.rle_encode(np.zeros((0, 3), bool)) == []

    def test_letterbox_matches_ultralytics_geometry(self):
        lb = nd.Letterbox(1280, 720, 640)
        assert (lb.r, lb.nw, lb.nh, lb.left, lb.top) == (0.5, 640, 360, 0, 140)
        lb3 = nd.Letterbox(1280, 720, 320)
        assert (lb3.r, lb3.nw, lb3.nh, lb3.left, lb3.top) == (0.25, 320, 180, 0, 70)
        # a box covering the whole non-padding region maps to the full frame; padding is clipped
        o = lb.to_orig(np.array([[0., 140., 640., 500.], [-5., 0., 700., 640.]]))
        assert np.allclose(o[0], [0, 0, 1280, 720]) and np.allclose(o[1], [0, 0, 1280, 720])

    def test_coco_names(self):
        assert len(nd.COCO) == 80
        assert (nd.COCO[39], nd.COCO[41], nd.COCO[63], nd.COCO[66]) == ("bottle", "cup", "laptop", "keyboard")

if __name__ == "__main__":
    unittest.main()
