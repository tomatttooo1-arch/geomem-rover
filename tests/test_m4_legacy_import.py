"""Dataset-root containment guard for the legacy M4 export path."""

from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from perception.fpga_replay.io import sha256
from tools.m4_legacy_import import _image_reference, _path_within


class DatasetRootGuardTests(unittest.TestCase):
    """Paired controls: a legal image resolves, an escaping one is refused.

    ``Path.is_relative_to`` is 3.9+, so on the declared 3.8 reference
    interpreter this guard used to raise ``AttributeError`` rather than judge
    the path.  Both directions have to run clean on 3.8.
    """

    def test_path_within_agrees_with_the_stdlib_method_it_replaces(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp).resolve()
            cases = [(base / "data/rgb/1.png", base / "data", True),
                     (base / "data", base / "data", True),
                     (base / "other/1.png", base / "data", False),
                     (base / "data", base / "data/rgb", False),
                     (Path("a/b"), Path("/a"), False)]
            for path, other, expected in cases:
                self.assertIs(_path_within(path, other), expected, (path, other))
                if hasattr(Path("/"), "is_relative_to"):  # 3.9+: differential control
                    self.assertIs(_path_within(path, other),
                                  Path(path).is_relative_to(other), (path, other))

    def test_image_outside_the_dataset_root_is_refused(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp).resolve()
            data = base / "data"
            (data / "rgb").mkdir(parents=True)
            inside = "rgb/1.000000.png"
            cv2.imwrite(str(data / inside), np.zeros((4, 4, 3), np.uint8))
            table = {inside: ["1.000000", inside]}
            record = {"rgb": inside, "rgb_sha256": sha256(data / inside),
                      "timestamp": 1.0}
            self.assertEqual(_image_reference(data, record, "rgb", table),
                             {"path": str(data / inside),
                              "sha256": sha256(data / inside)})
            # The escaping entry is present in the table, readable, and carries
            # its own true checksum and timestamp, so every other check in
            # _image_reference passes: only containment can reject it.
            escape = base / "escape.png"
            cv2.imwrite(str(escape), np.full((4, 4, 3), 255, np.uint8))
            outside = "../escape.png"
            escaping_table = {outside: ["1.000000", outside]}
            escaping_record = {"rgb": outside, "rgb_sha256": sha256(escape),
                               "timestamp": 1.0}
            self.assertEqual(sha256((data / outside).resolve()), sha256(escape))
            with self.assertRaisesRegex(ValueError, "escapes dataset root"):
                _image_reference(data, escaping_record, "rgb", escaping_table)


if __name__ == "__main__":
    unittest.main()
