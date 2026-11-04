"""Deployment regressions: explicit device and Python 3.8 path containment."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from perception.fpga_replay.detector import SSDDetector
from perception.fpga_replay.io import load_frame
from host.replay import runtime_versions
import importlib.metadata


class BackendTests(unittest.TestCase):
    def test_nonfinite_features_fail_instead_of_reporting_empty_scene(self):
        import torch
        SSDDetector._finite_guard(None, (), torch.ones(2, 3))
        for value in (torch.tensor([float("nan")]), {"logits": torch.tensor([float("inf")])}):
            with self.assertRaisesRegex(RuntimeError, "not an empty scene"):
                SSDDetector._finite_guard(None, (), value)

    def test_system_opencv_without_pip_metadata_keeps_actual_provenance(self):
        original = importlib.metadata.version
        def version(name):
            if name == "opencv-python":
                raise importlib.metadata.PackageNotFoundError(name)
            return original(name)
        with patch("importlib.metadata.version", side_effect=version):
            packages, modules = runtime_versions()
        self.assertIsNone(packages["opencv-python"])
        self.assertTrue(modules["cv2"]["version"])
        self.assertTrue(modules["cv2"]["file"])

    def test_requested_cuda_cannot_silently_fall_back(self):
        with patch("torch.cuda.is_available", return_value=False):
            with self.assertRaisesRegex(RuntimeError, "no CPU fallback"):
                SSDDetector("unused", device="cuda")

    def test_unknown_backend_rejected_before_loading_weights(self):
        with self.assertRaisesRegex(ValueError, "device"):
            SSDDetector("unused", device="automatic")

    def test_path_check_preserves_parent_and_symlink_rejection(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / "data"
            root.mkdir()
            outside = Path(td) / "outside.png"
            outside.write_bytes(b"not an image")
            (root / "link.png").symlink_to(outside)
            for name in ("../outside.png", "link.png", str(outside)):
                with self.subTest(name=name):
                    with self.assertRaisesRegex(ValueError, "escapes"):
                        load_frame({"rgb": name}, root)


if __name__ == "__main__":
    unittest.main()
