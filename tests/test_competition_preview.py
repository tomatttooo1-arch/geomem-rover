"""Real JPEG encoding on padded ROS Image-like buffers, without ROS or camera access."""
import sys
import types
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/car_run"))
from preview_publisher import encode_preview, preview_due
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


@unittest.skipIf(cv2 is None, "OpenCV/numpy unavailable in this interpreter")
class PreviewEncoderTests(unittest.TestCase):
    def test_rgb_stride_resolution_and_color(self):
        pixels = np.zeros((480, 640 * 3 + 8), dtype=np.uint8)
        pixels[:, :640 * 3:3] = 255
        msg = types.SimpleNamespace(width=640, height=480, step=pixels.shape[1], encoding="rgb8", data=pixels.tobytes())
        jpeg = encode_preview(msg, cv2, np)
        self.assertLessEqual(len(jpeg), 160 * 1024)
        decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertEqual(decoded.shape, (480, 640, 3))      # 10-21: full camera size, no downscale
        self.assertGreater(int(decoded[100, 100, 2]), 245)
        self.assertLess(int(decoded[100, 100, 0]), 10)
        msg.data = msg.data[:-1]
        self.assertIsNone(encode_preview(msg, cv2, np))

    def test_oversized_frame_is_downscaled_to_the_hub_limit(self):
        pixels = np.random.RandomState(0).randint(0, 255, (720, 1280 * 3), dtype=np.uint8)   # noise: worst case
        msg = types.SimpleNamespace(width=1280, height=720, step=1280 * 3, encoding="bgr8", data=pixels.tobytes())
        jpeg = encode_preview(msg, cv2, np)
        decoded = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.assertLessEqual(decoded.shape[1], 640)
        self.assertLessEqual(decoded.shape[0], 480)
        self.assertLessEqual(len(jpeg), 160 * 1024)

    def test_buffer_types_give_the_same_jpeg(self):
        # 10-29: msg.data is read in place (rclpy gives array.array); bytes / array / list must encode identically
        import array
        pixels = np.random.RandomState(1).randint(0, 255, (480, 640 * 3), dtype=np.uint8)
        base = types.SimpleNamespace(width=640, height=480, step=640 * 3, encoding="rgb8", data=pixels.tobytes())
        want = encode_preview(base, cv2, np)
        self.assertIsNotNone(want)
        reference = cv2.imencode(".jpg", cv2.cvtColor(pixels.reshape(480, 640, 3), cv2.COLOR_RGB2BGR),
                                 [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes()
        self.assertEqual(want, reference)                   # q60, full size, RGB -> BGR as before
        for data in (array.array("B", pixels.tobytes()), bytearray(pixels.tobytes()), list(pixels.tobytes()[:12])):
            msg = types.SimpleNamespace(**dict(vars(base), data=data))
            if isinstance(data, list):
                msg.width, msg.height, msg.step = 2, 2, 6
                self.assertEqual(encode_preview(msg, cv2, np), cv2.imencode(
                    ".jpg", cv2.cvtColor(np.frombuffer(bytes(data), np.uint8).reshape(2, 2, 3), cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes())
            else:
                self.assertEqual(encode_preview(msg, cv2, np), want)

    def test_unsupported_encoding_is_not_misread(self):
        msg = types.SimpleNamespace(width=640, height=480, step=1280, encoding="16UC1", data=b"x")
        self.assertIsNone(encode_preview(msg, cv2, np))


class PreviewPayloadTests(unittest.TestCase):
    def test_payload_is_a_byte_array_with_the_same_bytes(self):
        # 10-29: array.array('B') skips the per-byte check of the CompressedImage.data setter
        from preview_publisher import jpeg_payload
        data = bytes(range(256)) * 3
        payload = jpeg_payload(data)
        self.assertEqual(payload.typecode, "B")
        self.assertEqual(payload.tobytes(), data)


class PreviewScheduleTests(unittest.TestCase):
    """10-29: the camera runs 15 fps by default; the preview must still go out at 10 fps (and 10 fps at 30 fps)."""

    @staticmethod
    def sent(camera_fps, seconds=10.0, preview_fps=10.0, jitter_ms=0.0, gap=None):
        import random
        rnd, period, due, out, t = random.Random(1), 1.0 / preview_fps, None, [], 0.0
        while t < seconds:
            now = t + rnd.uniform(-jitter_ms, jitter_ms) / 1e3
            send, due = preview_due(now, due, period)
            if send:
                out.append(now)
            t += 1.0 / camera_fps
            if gap and gap[0] <= t < gap[0] + gap[1]:
                t = gap[0] + gap[1]
        return out

    def test_15_fps_camera_gives_10_fps(self):
        for jitter in (0.0, 4.0, 8.0):
            n = len(self.sent(15, jitter_ms=jitter))
            self.assertTrue(98 <= n <= 101, (jitter, n))      # the old 0.9-period rule: 75

    def test_30_fps_camera_still_10_fps(self):
        for jitter in (0.0, 4.0):
            n = len(self.sent(30, jitter_ms=jitter))
            self.assertTrue(98 <= n <= 101, (jitter, n))

    def test_slow_camera_sends_every_frame_and_no_burst_after_a_stall(self):
        self.assertEqual(len(self.sent(5, seconds=4.0)), 20)
        out = self.sent(15, gap=(3.0, 2.0))
        gaps = [b - a for a, b in zip(out, out[1:])]
        self.assertGreaterEqual(min(gaps), 1.0 / 15 - 1e-9)       # never two previews from one camera period
        after = [t for t in out if 5.0 <= t < 6.0]
        self.assertTrue(9 <= len(after) <= 11, len(after))       # back to 10 fps right after the stall

    def test_preview_rate_above_camera_rate_sends_every_frame(self):
        self.assertTrue(148 <= len(self.sent(15, preview_fps=15.0)) <= 151)


if __name__ == "__main__":
    unittest.main()
