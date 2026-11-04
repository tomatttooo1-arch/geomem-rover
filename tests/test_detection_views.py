import unittest
import numpy as np
from perception.fpga_replay.detector import restore_boxes


class DetectionViewTests(unittest.TestCase):
    def test_flip_maps_to_original_depth_pixels_and_is_involutive(self):
        original = np.array([[10., 20., 30., 60.], [0., 1., 320., 239.]])
        flipped = restore_boxes(original, 320, 240, 'flip')
        np.testing.assert_array_equal(flipped[0], [290, 20, 310, 60])
        np.testing.assert_array_equal(restore_boxes(flipped, 320, 240, 'flip'), original)
        np.testing.assert_array_equal(original[0], [10, 20, 30, 60])

    def test_opposite_shifts_restore_known_correspondence_and_clip_padding(self):
        np.testing.assert_array_equal(restore_boxes(np.array([[6.,20.,26.,60.]]),320,240,'left4'),
                                      [[10.,20.,30.,60.]])
        np.testing.assert_array_equal(restore_boxes(np.array([[14.,20.,34.,60.]]),320,240,'right4'),
                                      [[10.,20.,30.,60.]])
        np.testing.assert_array_equal(restore_boxes(np.array([[0.,0.,2.,10.]]),320,240,'right4'),
                                      [[0.,0.,0.,10.]])
        with self.assertRaises(ValueError):restore_boxes(np.zeros((1,4)),320,240,'unknown')


if __name__=='__main__':unittest.main()
