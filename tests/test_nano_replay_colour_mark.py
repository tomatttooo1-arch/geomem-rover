"""nano_replay --colour-watermark (TEST PATH ONLY, board check of tearing / stale colour, 2026-10-16).

Known answers: the mark is the source key in the first and last 4 pixels of every row (R = low byte,
G = high byte, B = 0xA5), the rest of the image is untouched, the input is not modified; every key the
replay can serve (1..65535) reads back; a natural pixel (B != 0xA5) reads as None. Default path: the
provider without the flag serves the prepared bytes exactly and the HELLO info has no watermark key.
Run: cd tests && PYTHONPATH=.. python3 -m unittest discover -s . -p test_nano_replay_colour_mark.py
"""
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools" / "kv_link"))

import nano_replay  # noqa: E402


class MarkTests(unittest.TestCase):
    def test_mark_round_trip_and_untouched_rest(self):
        rng = np.random.RandomState(3)
        img = rng.randint(0, 255, (36, 64, 3)).astype(np.uint8)
        keep = img.copy()
        for key in (1, 255, 256, 310, 65535):
            m = nano_replay.colour_watermark(img, key)
            np.testing.assert_array_equal(img, keep)
            np.testing.assert_array_equal(m[:, 4:-4], img[:, 4:-4])
            for y in (0, 17, 35):
                for x in (0, 3, 60, 63):
                    self.assertEqual(nano_replay.read_colour_watermark(m[y, x]), key)
        self.assertIsNone(nano_replay.read_colour_watermark((1, 2, 3)))

    def test_default_provider_serves_prepared_bytes(self):
        n, h, w = 3, 6, 8
        rgb = (np.arange(n * h * w * 3) % 251).astype(np.uint8)
        with tempfile.TemporaryDirectory() as d:
            rgb.tofile(os.path.join(d, "c.bin"))
            json.dump({"colour_file": "c.bin", "width": w, "height": h, "frames": n},
                      open(os.path.join(d, "colour.json"), "w"))
            got = {}

            class FakeSource:
                def __init__(self, kv, provider, info=None):
                    got["provider"], got["info"] = provider, info

                def start(self):
                    return self
            import host.kv_colour as kc
            real = kc.ColourSource
            kc.ColourSource = FakeSource
            try:
                nano_replay.start_colour_source(d, ("127.0.0.1", 1), (h, w), n)
                plain = got["provider"](2, 0)
                info_plain = dict(got["info"])
                nano_replay.start_colour_source(d, ("127.0.0.1", 1), (h, w), n, watermark=True)
                marked = got["provider"](2, 0)
                info_mark = dict(got["info"])
            finally:
                kc.ColourSource = real
        frame = rgb.reshape(n, h, w, 3)[1]
        self.assertEqual(plain, (2, w, h, frame.tobytes()))
        self.assertNotIn("watermark", info_plain)
        self.assertTrue(info_mark["watermark"])
        m = np.frombuffer(marked[3], np.uint8).reshape(h, w, 3)
        self.assertEqual(nano_replay.read_colour_watermark(m[5, 7]), 2)
        np.testing.assert_array_equal(m[:, 4:-4], frame[:, 4:-4])
        self.assertEqual(got["provider"](9, 0), "no_match")


if __name__ == "__main__":
    unittest.main()
