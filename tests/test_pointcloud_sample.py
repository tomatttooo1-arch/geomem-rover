import math
import struct
import unittest
from types import SimpleNamespace as Obj
from host.pointcloud_sample import sample_xyz


def cloud(big=False):
    rows = [[(float(i), float(i+1), float(i+2)) for i in range(5)],
            [(float(i+5), float(i+6), float(i+7)) for i in range(5)]]
    rows[0][2] = (math.nan, 3., 4.)
    endian = '>' if big else '<'
    data = b''.join(b''.join(struct.pack(endian+'fffI', *point, 42) for point in row)+b'padbytes'
                    for row in rows)
    return Obj(width=5, height=2, point_step=16, row_step=88, data=data, is_bigendian=big,
               fields=[Obj(name=n, count=1, datatype=7, offset=i*4) for i,n in enumerate(('x','y','z'))])


class PointSampleTests(unittest.TestCase):
    def test_padded_rows_endian_and_filter_then_stride_match_struct_reference(self):
        for big in (False, True):
            message = cloud(big)
            reference = []
            for y in range(message.height):
                for x in range(message.width):
                    xyz = struct.unpack_from(('>' if big else '<')+'fff', message.data,
                                             y*message.row_step+x*message.point_step)
                    if all(math.isfinite(v) for v in xyz):
                        reference.append(list(xyz))
            self.assertEqual(sample_xyz(message, 2, 3), reference[::2][:3])

    def test_invalid_layout_and_truncation_are_rejected(self):
        for attribute, value in (('row_step', 1), ('data', b''), ('point_step', 4)):
            message = cloud()
            setattr(message, attribute, value)
            with self.assertRaises(ValueError):
                sample_xyz(message)

    def test_inf_is_invalid_and_sampling_remains_bounded(self):
        message = cloud()
        message.data = struct.pack('<fffI', math.inf, 1., 2., 42) + message.data[16:]
        result = sample_xyz(message, 1, 2)
        self.assertEqual(len(result), 2)
        self.assertTrue(all(math.isfinite(v) for row in result for v in row))
