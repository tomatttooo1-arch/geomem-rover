import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.rtabmap_reload_coordinate_prepare import DATASET_NAME, split_input


CALIBRATION = """%YAML:1.0
---
camera_name: rtabmap_calib
image_width: 640
image_height: 480
camera_matrix:
   rows: 3
   cols: 3
   data: [ 517.3, 0., 318.6, 0., 516.5, 255.3, 0., 0., 1. ]
local_transform:
   rows: 3
   cols: 4
   data: [ 0., 0., 1., 0., -1., 0., 0., 0., 0., -1., 0., 0. ]
"""


class ReloadCoordinatePrepareTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "source" / DATASET_NAME
        (self.source / "rgb_sync").mkdir(parents=True)
        (self.source / "depth_sync").mkdir()
        self.calibration = self.root / "rtabmap_calib.yaml"
        self.calibration.write_text(CALIBRATION)

    @staticmethod
    def digest(data):
        return hashlib.sha256(data).hexdigest()

    def make_manifest(self, count=10, duplicate_pair=False):
        pairs = []
        for index in range(count):
            stamp = f"{100 + index}.000000"
            rgb = (b"rgb-0" if duplicate_pair and index == count - 1 else f"rgb-{index}".encode())
            depth = (b"depth-0" if duplicate_pair and index == count - 1 else f"depth-{index}".encode())
            (self.source / "rgb_sync" / (stamp + ".png")).write_bytes(rgb)
            (self.source / "depth_sync" / (stamp + ".png")).write_bytes(depth)
            pairs.append({
                "timestamp": stamp,
                "depth_timestamp": f"{100 + index}.010000",
                "delta_s": 0.01,
                "rgb_sync": {"source": f"/unmounted/raw/rgb/{stamp}.png",
                             "source_sha256": self.digest(rgb), "input_sha256": self.digest(rgb)},
                "depth_sync": {"source": f"/unmounted/raw/depth/{stamp}.png",
                               "source_sha256": self.digest(depth), "input_sha256": self.digest(depth)},
            })
        path = self.source / "INPUT.json"
        path.write_text(json.dumps({"mode": "normal", "pairs": pairs,
                                    "groundtruth_available_to_slam": False}))
        return path

    def test_fixed_split_copies_and_hashes_disjoint_inputs(self):
        source = self.make_manifest()
        result = split_input(source, self.root / "split", self.calibration)
        self.assertEqual(result["split_index"], 6)
        self.assertEqual(result["mapping"]["original_range"], [0, 6])
        self.assertEqual(result["query"]["original_range"], [6, 10])
        query_path = self.root / "split/query" / DATASET_NAME / "INPUT.json"
        query = json.loads(query_path.read_text())
        self.assertEqual([row["original_index"] for row in query["pairs"]], [6, 7, 8, 9])
        copied = query_path.parent / "rgb_sync" / "106.000000.png"
        self.assertEqual(hashlib.sha256(copied.read_bytes()).hexdigest(),
                         query["pairs"][0]["rgb_sync"]["input_sha256"])
        self.assertFalse((query_path.parent / "groundtruth.txt").exists())
        self.assertEqual((query_path.parent / "rtabmap_calib.yaml").read_text(), CALIBRATION)

    def test_reused_pair_hash_is_rejected_before_writing(self):
        source = self.make_manifest(duplicate_pair=True)
        output = self.root / "rejected"
        with self.assertRaisesRegex(ValueError, "paired input hash overlap"):
            split_input(source, output, self.calibration)
        self.assertFalse(output.exists())

    def test_reused_source_path_is_rejected_before_writing(self):
        source = self.make_manifest()
        manifest = json.loads(source.read_text())
        manifest["pairs"][-1]["rgb_sync"]["source"] = manifest["pairs"][0]["rgb_sync"]["source"]
        source.write_text(json.dumps(manifest))
        output = self.root / "rejected"
        with self.assertRaisesRegex(ValueError, "source path overlap"):
            split_input(source, output, self.calibration)
        self.assertFalse(output.exists())

    def test_wrong_calibration_is_rejected_before_writing(self):
        source = self.make_manifest()
        self.calibration.write_text(CALIBRATION.replace("517.3", "500.0"))
        output = self.root / "rejected"
        with self.assertRaisesRegex(ValueError, "fixed Freiburg1"):
            split_input(source, output, self.calibration)
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
