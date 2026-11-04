"""NN-CAM2 host tests: chain_report on stamps with known answers (+ a shifted-input control), the local source's
grid constants against the car producer's, and the local request against the Nano client's request bytes."""
import json
import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools", "nn_cam2"))
import chain_report  # noqa: E402


def synth(shift_geo_ms=0.0):
    """100 camera frames 33.4 ms apart; every 10th never processed (drop 10 %); geometry 10 + (i % 5) ms after
    arrival, GRID 2 ms later, field 20 ms after GRID for every 3rd processed frame; colour prep 8, DPU 30, post 31."""
    rows = [{"ev": "clock", "mono_ns": 0, "realtime_ns": 0}]
    for i in range(100):
        if i % 10 == 9:
            continue
        a = int(1e9 + i * 33.4e6)
        geo = a + int((10 + i % 5 + shift_geo_ms) * 1e6)
        d = {"chain": "depth", "frame": i, "t_cam_arrive_ns": a, "t_cam_publish_ns": a + 100000,
             "t_read_ns": a + 200000, "cam_dev_ts_us": 1000000 + i * 33400, "t_geo_ns": geo,
             "t_grid_ns": geo + 2000000, "geo_status": 0}
        rows.append(d)
        if i % 3 == 0:
            rows.append({"chain": "depth", "ev": "field", "frame": i, "t_field_ns": geo + 22000000, "plan_ms": 1.5})
        rows.append({"chain": "colour", "src": "kvcam", "frame": i, "t_cam_arrive_ns": a, "t_cam_publish_ns": a,
                     "t_read_ns": a, "cam_dev_ts_us": 1000000 + i * 33400, "t_prep_ns": a + 8000000,
                     "t_dpu_ns": a + 30000000, "t_post_ns": a + 31000000, "ok": 1, "prep_cpu_us": 7000.0})
    return rows


class ChainReport(unittest.TestCase):
    def test_known_answers(self):
        r = chain_report.report(synth(), warm_s=0)
        d, c = r["depth"], r["colour"]
        self.assertEqual(d["camera_frames_offered"], 99)          # frames 0..98 (99 is a dropped one)
        self.assertEqual(d["frames_in_window"], 90)
        self.assertEqual(d["stages"]["geometry"]["median"], 12.0)
        self.assertEqual(d["stages"]["grid"]["median"], 14.0)
        self.assertEqual(d["stages"]["grid"]["drop_rate"], round(1 - 90 / 99, 4))
        self.assertEqual(d["stages"]["field"]["n"], 30)
        self.assertEqual(d["grid_to_field_ms"]["median"], 20.0)
        self.assertAlmostEqual(d["camera_fps_from_dev_ts"], 1e3 / 33.4, places=2)
        self.assertEqual(c["stages"]["post"]["median"], 31.0)
        self.assertEqual(c["stages"]["prep"]["p99"], 8.0)
        # 90 outputs spread over frames 0..98 -> about 90/99 of 29.94 fps
        self.assertAlmostEqual(c["stages"]["post"]["rate_fps"], 89 / (98 * 0.0334), places=2)

    def test_camera_rate_with_heavy_drops(self):   # second control: keep only every 3rd frame -> still 29.94 fps
        rows = [r for r in synth() if r.get("chain") != "colour" or r["frame"] % 3 == 0]
        c = chain_report.report(rows, warm_s=0)["colour"]
        self.assertAlmostEqual(c["camera_fps_from_dev_ts"], 1e3 / 33.4, places=2)

    def test_shift_is_seen(self):   # positive control: +5 ms on every geometry stamp moves the reading by 5 ms
        a = chain_report.report(synth(), warm_s=0)["depth"]["stages"]["geometry"]["median"]
        b = chain_report.report(synth(5.0), warm_s=0)["depth"]["stages"]["geometry"]["median"]
        self.assertEqual(round(b - a, 3), 5.0)

    def test_cli(self):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "s.jsonl")
            with open(p, "w") as fp:
                for row in synth():
                    fp.write(json.dumps(row) + "\n")
            res = chain_report.report(chain_report.load([p]), 0)
            self.assertIn("depth", res)
            self.assertIn("colour", res)


class LocalParams(unittest.TestCase):
    def test_grid_constants_match_car_producer(self):
        sys.path.insert(0, ROOT)
        from host import kv_local_frames as lf
        try:
            from host import geo_producer as gp
        except Exception as exc:   # the Nano runtime may be absent on this interpreter
            self.skipTest("host.geo_producer not importable: %r" % (exc,))
        self.assertEqual(lf.VOXEL_M, gp.VOXEL_M)
        self.assertEqual(tuple(lf.DEFAULT_SHAPE), tuple(gp.DEFAULT_SHAPE))
        self.assertEqual(lf.DEFAULT_HEIGHT_CELL, gp.DEFAULT_HEIGHT_CELL)
        from host import kv_nav
        self.assertEqual(lf.ROBOT_HEIGHT_M, kv_nav.ROBOT_HEIGHT_M)

    def test_request_decodes_to_the_same_core_input(self):
        sys.path.insert(0, ROOT)
        from host.kv_link import decode_request
        from host.kv_local_frames import FrameParams
        p = FrameParams({"intrinsics": {"fx": 344.8, "fy": 344.7, "cx": 322.5, "cy": 240.4}})
        codes = (np.arange(640 * 480, dtype=np.uint32) % 5000).astype(np.uint16).reshape(480, 640)
        req = decode_request(p.request(7, 123, codes, 640, 480))
        self.assertEqual(req["frame_id"], 7)
        self.assertTrue(np.array_equal(np.asarray(req["codes"]).ravel(), codes.ravel()))
        self.assertTrue(np.array_equal(np.asarray(req["inverse_k"], np.float32).ravel(), p.inverse_k.ravel()))
        self.assertTrue(np.array_equal(np.asarray(req["world_camera"], np.float32).ravel(), p.world_camera.ravel()))
        self.assertEqual((req["nx"], req["ny"], req["nz"]), (32, 32, 10))


if __name__ == "__main__":
    unittest.main()
