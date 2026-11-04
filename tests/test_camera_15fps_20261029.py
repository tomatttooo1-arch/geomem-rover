"""10-29 camera 15 fps (car_up CAR_CAMERA_FPS): the rate-dependent rules follow the configured rate and 30 fps keeps
the rules of before (tools/car_run/depth_probe.py rate_limits / classify / pair_offset_ms, host/ros_runtime.py
camera_pairing, tools/car_run/rtabmap_car.sh approx sync)."""
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.car_run import depth_probe as dp  # noqa: E402


class TestProbeRate(unittest.TestCase):
    def test_limits(self):
        self.assertEqual(dp.rate_limits(30), (dp.DEPTH_HALF_MS, dp.COLOUR_HALF_MS, dp.MIN_DEPTH_IV, dp.MIN_COLOUR_IV))
        self.assertEqual(dp.rate_limits(), dp.rate_limits(30.0))
        self.assertEqual(dp.rate_limits(15), (None, 100.0, 10, 5))
        self.assertEqual(dp.rate_limits(10)[:2], (None, 150.0))
        for bad in (0, -15):
            with self.assertRaises(ValueError):
                dp.rate_limits(bad)

    def test_30_fps_record_unchanged(self):
        r = dp.classify([33.4] * 90, [33.4] * 90)
        self.assertNotIn("fps", r)
        self.assertEqual(r, dp.classify([33.4] * 90, [33.4] * 90, fps=30))
        self.assertTrue(r["rule"].startswith("half if depth median < 32.8 ms or colour p25 > 50.0 ms"))

    def test_15_fps_normal_lossy_half_insufficient(self):
        n = dp.classify([66.7] * 44, [66.7] * 44, fps=15)
        self.assertEqual((n["verdict"], n["fps"]), ("normal", 15.0))
        self.assertIn("no depth rule", n["rule"])
        # the 30 fps depth rule would call a 15 fps stream's 64.6 ms depth "normal" anyway, but a fast 32 ms depth
        # interval (driver delivering more than asked) is no refusal at 15 fps
        self.assertEqual(dp.classify([32.3] * 44, [66.7] * 44, fps=15)["verdict"], "normal")
        # colour losses: half the colour frames lost (133 ms intervals) is still normal (p25 66.7)
        c = [66.7] * 22 + [133.4] * 22
        self.assertEqual(dp.classify([66.7] * 44, c, fps=15)["verdict"], "normal")
        # colour at 7.5 fps (every interval 133 ms) = half
        h = dp.classify([66.7] * 44, [133.4] * 22, fps=15)
        self.assertEqual(h["verdict"], "half")
        self.assertIn("colour p25 133.40 ms > 100.0", h["reasons"][0])
        # boundaries of the scaled colour threshold and the minimum counts
        self.assertEqual(dp.classify([66.7] * 10, [100.0] * 5, fps=15)["verdict"], "normal")
        self.assertEqual(dp.classify([66.7] * 10, [100.01] * 5, fps=15)["verdict"], "half")
        self.assertEqual(dp.classify([66.7] * 9, [66.7] * 44, fps=15)["verdict"], "insufficient")
        self.assertEqual(dp.classify([66.7] * 44, [66.7] * 4, fps=15)["verdict"], "insufficient")
        # a 30 fps window would be "insufficient" at 15 fps counts? no: 10 / 5 are lower than 20 / 10
        self.assertEqual(dp.classify([66.7] * 15, [66.7] * 8, fps=15)["verdict"], "normal")
        self.assertEqual(dp.classify([66.7] * 15, [66.7] * 8)["verdict"], "insufficient")

    def test_pair_offset(self):
        d = [10.0 + i / 15.0 for i in range(30)]
        c = [x + 0.0119 for x in d]
        self.assertAlmostEqual(dp.pair_offset_ms(d, c), 11.9, places=3)
        self.assertAlmostEqual(dp.pair_offset_ms(d, [x - 0.005 for x in d]), -5.0, places=3)
        lossy = [x for i, x in enumerate(c) if i % 4]                            # a quarter of the colour lost
        self.assertAlmostEqual(dp.pair_offset_ms(d, lossy), 11.9, places=3)      # median over the depth frames
        self.assertIsNone(dp.pair_offset_ms(d, []))
        self.assertIsNone(dp.pair_offset_ms([], c))

    def test_cli_rejects_bad_rate(self):
        r = subprocess.run([sys.executable, str(ROOT / "tools/car_run/depth_probe.py"), "--topic", "/x",
                            "--rate-fps", "0"], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 2)
        self.assertIn("--rate-fps must be > 0", r.stderr)


class TestRunnerPairing(unittest.TestCase):
    def test_camera_pairing(self):
        from host import ros_runtime
        self.assertEqual(ros_runtime.camera_pairing(30), (0.02, 0.05))
        self.assertEqual(ros_runtime.camera_pairing(60), (0.02, 0.05))
        self.assertEqual(ros_runtime.camera_pairing(15), (0.038, 0.073))      # owner 10-29: half a period + 5 ms
        self.assertEqual(ros_runtime.camera_pairing(10.0), (0.055, 0.11))
        self.assertEqual(ros_runtime.camera_pairing(5), (0.105, 0.22))
        self.assertEqual(ros_runtime.camera_pairing(29), (0.022, 0.05))
        from perception.fpga_replay.core import PAIR_TOLERANCE_MAX_S
        for fps in (29.9, 25, 15, 12, 10, 7.5, 5, 2, 1, 0.5, 0.1, 1e-3):
            slop, match = ros_runtime.camera_pairing(fps)
            self.assertTrue(0.02 <= slop <= PAIR_TOLERANCE_MAX_S, fps)        # Frame.validate accepts every value
            self.assertGreaterEqual(match, slop, fps)                         # producer match never narrower

    def test_frame_validate_follows_pair_slop(self):
        """car 10-29 22:56: 15 fps stamps sat 26.6 ms apart; Frame.validate's fixed 20 ms refused every frame"""
        import numpy as np
        from host import inference_service, ros_runtime
        from perception.fpga_replay.core import Frame
        K = np.array([[500.0, 0, 320], [0, 500.0, 240], [0, 0, 1]])
        def frame(tol=None):
            f = Frame("f", 10.0, np.zeros((4, 4, 3), np.uint8), np.ones((4, 4), np.float32), K, np.eye(4),
                      depth_timestamp=10.0266)
            if tol is not None:
                f.pair_tolerance_s = tol
            return f
        with self.assertRaisesRegex(ValueError, "mismatch > 20 ms"):
            frame().validate()                                   # default (30 fps) unchanged
        with self.assertRaisesRegex(ValueError, "mismatch > 20 ms"):
            frame(ros_runtime.camera_pairing(30)[0]).validate()
        f = frame(ros_runtime.camera_pairing(15)[0])
        f.validate()
        back = inference_service.decode_frame(inference_service.encode_frame(f))   # crosses the inference socket
        self.assertEqual(back.pair_tolerance_s, 0.038)
        Frame("h", 10.0, f.rgb, f.depth, K, np.eye(4), depth_timestamp=10.0333, pair_tolerance_s=0.038).validate()
        with self.assertRaisesRegex(ValueError, "mismatch > 38 ms"):
            Frame("g", 10.0, f.rgb, f.depth, K, np.eye(4), depth_timestamp=10.04, pair_tolerance_s=0.038).validate()
        for slow in (0.105, 0.505, 1.0):                         # 5 fps, 1 fps, the cap: accepted
            frame(slow).validate()
        for bad in (0.0, -1.0, 1.5, 33.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                frame(bad).validate()


class TestRtabmapSync(unittest.TestCase):
    """the approx-sync default of rtabmap_car.sh per CAMERA_FPS (the shell lines themselves, evaluated)"""

    def sync(self, fps=None, given=None):
        text = (ROOT / "tools/car_run/rtabmap_car.sh").read_text()
        lines = [l for l in text.splitlines()
                 if l.startswith('case "${CAMERA_FPS:-30}"') or l.startswith(': "${APPROX_SYNC:=true}"')]
        self.assertEqual(len(lines), 2)
        env = "unset CAMERA_FPS SYNC_MAX_S; "
        if fps is not None:
            env += "CAMERA_FPS=%s; " % fps
        if given is not None:
            env += "SYNC_MAX_S=%s; " % given
        out = subprocess.run(["bash", "-c", env + "\n".join(lines) + '\necho "$SYNC_MAX_S"'],
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip()

    def test_values(self):
        self.assertEqual(self.sync(), "0.02")
        self.assertEqual(self.sync(30), "0.02")
        self.assertEqual(self.sync(15), "0.038")
        self.assertEqual(self.sync(10), "0.055")
        self.assertEqual(self.sync(5), "0.105")
        from host import ros_runtime
        for fps in (15, 10, 5):                                   # the shell table = camera_pairing
            self.assertEqual(float(self.sync(fps)), ros_runtime.camera_pairing(fps)[0])
        self.assertEqual(self.sync(15, given="0.04"), "0.04")      # given from outside wins


if __name__ == "__main__":
    unittest.main()
