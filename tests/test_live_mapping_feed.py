"""Transport packing only; these fixtures never touch ROS or vehicle controls."""
import importlib.util
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

spec = importlib.util.spec_from_file_location("live_mapping", Path(__file__).resolve().parents[1] / "ui/geomem/tools/competition/live_mapping.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class CloudPacking(unittest.TestCase):
    def message(self, big=False):
        endian = ">" if big else "<"
        # Two padded rows; NaN points must be discarded along with their color.
        data = b"".join(struct.pack(endian + "fffI", *p) + b"padding!" for p in [(1., 2., 3., 0x123456), (float("nan"), 5., 6., 0xFFFFFF)])
        return NS(header=NS(frame_id="map"), fields=[NS(name=n, offset=o, datatype=t, count=1) for n,o,t in [("x",0,7),("y",4,7),("z",8,7),("rgb",12,7)]],
                  is_bigendian=big, height=2, width=1, point_step=16, row_step=24, data=data)

    def test_padding_endianness_and_finite_filter(self):
        for big in (False, True):
            body, meta = module.pack_cloud(self.message(big))
            self.assertEqual(meta["count"], 1)
            self.assertEqual(body, struct.pack("<fff",1,2,3) + bytes([0x12,0x34,0x56]))
            self.assertEqual(meta["bounds"], [[1.,2.,3.],[1.,2.,3.]])

    def test_reject_malformed_layout_and_wrong_frame(self):
        for key, value in [("data", b"short"), ("row_step", 8), ("point_step", 8), ("header", NS(frame_id="odom"))]:
            msg = self.message(); setattr(msg, key, value)
            with self.assertRaises(ValueError): module.pack_cloud(msg)


if __name__ == "__main__":
    unittest.main()
