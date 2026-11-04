import unittest
import cv2
import numpy as np
from tools.rtabmap_view_audit import quality


class QualityTests(unittest.TestCase):
    def test_black_and_missing_depth(self):
        q=quality(np.zeros((80,80,3),np.uint8),np.zeros((80,80),np.uint16))
        self.assertEqual(q['gftt_capped_count'],0);self.assertEqual(q['valid_depth_fraction'],0)
        self.assertEqual(q['laplacian_variance'],0);self.assertIsNone(q['feature_valid_depth_fraction'])
    def test_texture_blur_and_depth_are_separate_factors(self):
        rng=np.random.default_rng(0);rgb=rng.integers(0,256,(80,80,3),dtype=np.uint8);depth=np.ones((80,80),np.uint16)
        q=quality(rgb,depth);blur=quality(cv2.GaussianBlur(rgb,(31,31),8),depth);zero=quality(rgb,np.zeros_like(depth))
        self.assertGreater(q['gftt_capped_count'],0);self.assertLess(blur['laplacian_variance'],q['laplacian_variance'])
        self.assertEqual(zero['laplacian_variance'],q['laplacian_variance']);self.assertEqual(zero['feature_valid_depth_fraction'],0)


if __name__=='__main__':unittest.main()
