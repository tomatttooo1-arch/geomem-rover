"""Calibrate the frozen RGB source guard against changed bytes and table rows."""

from pathlib import Path
import tempfile
import unittest

from perception.fpga_replay.io import sha256
from tools.m7_tum_continuous_memory import path_within, verify_frozen_rgb


class FrozenRgbSourceTests(unittest.TestCase):
    def test_original_bytes_and_timestamp_row_must_both_match(self):
        with tempfile.TemporaryDirectory() as directory:
            data = Path(directory)
            (data / "rgb").mkdir()
            stamp = "1305031454.659528"
            relative = "rgb/" + stamp + ".png"
            rgb = data / relative
            rgb.write_bytes(b"controlled original RGB bytes")
            table = data / "rgb.txt"
            table.write_text(stamp + " " + relative + "\n")
            selected = {"source_path": str(rgb), "sha256": sha256(rgb)}
            image = {"source_path": str(rgb), "sha256": sha256(rgb),
                     "frame_id": "sha256:" + sha256(rgb),
                     "timestamp_source": {"path": str(table), "sha256": sha256(table),
                                          "row": stamp + " " + relative}}
            self.assertEqual(verify_frozen_rgb(image, selected, data, stamp), rgb)
            rgb.write_bytes(b"changed bytes at same path")
            with self.assertRaisesRegex(ValueError, "RGB bytes"):
                verify_frozen_rgb(image, selected, data, stamp)
            rgb.write_bytes(b"controlled original RGB bytes")
            image["timestamp_source"]["row"] = stamp + " rgb/another.png"
            with self.assertRaisesRegex(ValueError, "acquisition row"):
                verify_frozen_rgb(image, selected, data, stamp)


    def fixture(self, data):
        """A legal frozen-RGB fixture rooted at ``data``."""
        (data / "rgb").mkdir(parents=True)
        stamp = "1305031454.659528"
        relative = "rgb/" + stamp + ".png"
        rgb = data / relative
        rgb.write_bytes(b"controlled original RGB bytes")
        table = data / "rgb.txt"
        table.write_text(stamp + " " + relative + "\n")
        selected = {"source_path": str(rgb), "sha256": sha256(rgb)}
        image = {"source_path": str(rgb), "sha256": sha256(rgb),
                 "frame_id": "sha256:" + sha256(rgb),
                 "timestamp_source": {"path": str(table), "sha256": sha256(table),
                                      "row": stamp + " " + relative}}
        return stamp, rgb, image, selected

    def test_rgb_outside_the_dataset_is_refused_on_the_reference_interpreter(self):
        """Paired control for the containment half of the frozen-RGB guard.

        ``Path.is_relative_to`` is 3.9+, so on the declared 3.8 reference
        interpreter this guard used to raise ``AttributeError`` instead of
        judging the path.  Both directions must therefore run clean on 3.8.
        """
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory).resolve()
            data = base / "data"
            stamp, rgb, image, selected = self.fixture(data)
            self.assertEqual(verify_frozen_rgb(image, selected, data, stamp), rgb)
            self.assertTrue(path_within(rgb, data))
            # Same bytes, same declared row, but the file sits outside the
            # dataset: the selection/content checks cannot tell, only
            # containment can.
            stray = base / "stray/rgb" / (stamp + ".png")
            stray.parent.mkdir(parents=True)
            stray.write_bytes(rgb.read_bytes())
            self.assertEqual(sha256(stray), sha256(rgb))
            self.assertFalse(path_within(stray, data))
            escaped = dict(image, source_path=str(stray))
            escaped_selection = dict(selected, source_path=str(stray))
            with self.assertRaisesRegex(ValueError, "selected RGB path mismatch"):
                verify_frozen_rgb(escaped, escaped_selection, data, stamp)


if __name__ == "__main__":
    unittest.main()
