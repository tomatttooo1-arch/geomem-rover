"""Board-run tools of the geometry switch: replay camera index codec, stub detector, report.

ROS-free (the rclpy parts of replay_camera.py run only on the board image)."""
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from perception.fpga_replay.core import Frame, process_frame  # noqa: E402
from tools.car_geometry_kv import replay_camera as rc  # noqa: E402
from tools.car_geometry_kv import ros_report as rr  # noqa: E402
from tools.car_geometry_kv.stub_inference import SCHEDULE, FixedDetector  # noqa: E402

REC30 = os.environ.get("FPGA_REC30_PREP",
                       "${FPGA_SRC}/runs/ar-display-20261011-scratch/rec30_prep")
K = [[344.78, 0, 322.48], [0, 344.74, 180.43], [0, 0, 1]]


def frame(idx, fid="rgb-1", ts=1.0, h=360, w=640, depth=None):
    rgb = np.zeros((h, w, 3), np.uint8)
    if idx is not None:
        rc.encode_index(rgb, idx)
    d = depth if depth is not None else np.full((h, w), 0.5, np.float32)
    return Frame(fid, ts, rgb, d, np.array(K, float), np.eye(4), True, "reference", "c", None)


class IndexCodec(unittest.TestCase):
    def test_roundtrip_and_rejects(self):
        for i in (0, 1, 255, 256, 309, 65537, 2 ** 32 - 1):
            rgb = np.zeros((4, 8, 3), np.uint8)
            self.assertEqual(rc.decode_index(rc.encode_index(rgb, i)), i)
        self.assertIsNone(rc.decode_index(np.zeros((4, 8, 3), np.uint8)))
        rgb = rc.encode_index(np.zeros((4, 8, 3), np.uint8), 1234)
        rgb[0, 1, 0] ^= 1                         # positive control: one damaged byte
        self.assertIsNone(rc.decode_index(rgb))
        with self.assertRaises(ValueError):
            rc.encode_index(np.zeros((4, 8, 3), np.uint8), -1)

    def test_u8_array_bytes_identical(self):
        a = np.arange(12, dtype="<u2").reshape(3, 4)
        self.assertEqual(rc.as_u8_array(a).tobytes(), a.tobytes())
        self.assertEqual(rc.as_u8_array(a).typecode, "B")

    def test_camera_info_fields(self):
        f = rc.camera_info_fields({"width": 640, "height": 360, "K": K})
        self.assertEqual((f["width"], f["height"]), (640, 360))
        self.assertEqual(f["d"], [0.0] * 5)
        self.assertEqual(f["k"][0], 344.78)
        self.assertEqual(f["p"][3], 0.0)
        for key in ("k", "d", "r", "p"):          # sensor_msgs asserts float elements
            self.assertTrue(all(type(v) is float for v in f[key]), key)


class StubDetector(unittest.TestCase):
    def test_depends_on_source_index_only(self):
        det = FixedDetector()
        a = det(frame(35, "rgb-10", 10.0))
        b = det(frame(35, "rgb-99", 99.0))
        self.assertEqual([(d.label, d.bbox) for d in a], [(d.label, d.bbox) for d in b])
        self.assertEqual(sorted(d.label for d in a), ["chair", "cup"])          # 35 % 5, 35 % 7
        self.assertEqual([d.frame_id for d in b], ["rgb-99"] * 2)
        self.assertEqual(det(frame(1)), [])
        self.assertEqual(det(frame(None)), [])
        self.assertEqual(det.counts["no_index"], 1)
        self.assertEqual(sorted(d.label for d in det(frame(70))), ["bottle", "chair", "cup"])
        self.assertEqual(sorted(d.label for d in det(frame(170))), ["chair"])  # bottle only < 100

    def test_socket_roundtrip_through_inference_client(self):
        from host.inference_service import InferenceClient, serve
        path = os.path.join(tempfile.mkdtemp(prefix="cg", dir="/tmp"), "s")
        det = FixedDetector()
        threading.Thread(target=serve, args=(path, det, "cpu"), daemon=True).start()
        for _ in range(100):
            if os.path.exists(path):
                break
            time.sleep(0.02)
        out = InferenceClient(path, timeout=5, expected_device="cpu").detect(frame(10, "rgb-7", 7.0))
        self.assertEqual(sorted(d.label for d in out), ["bottle", "chair"])
        with self.assertRaises(ValueError):          # the runtime refuses a device mismatch
            InferenceClient(path, timeout=5, expected_device="cuda").detect(frame(10, "rgb-8", 8.0))

    @unittest.skipUnless(os.path.exists(os.path.join(REC30, "meta.json")), "rec30 prep not here")
    def test_every_box_yields_an_observation_on_rec30(self):
        meta, depth = rc.load_prep(REC30)
        for i in (0, 70, 140, 210, 280):
            d = depth[i].astype(np.float32) * float(meta["depth_unit_m"])
            d[d <= 0] = np.nan
            f = frame(i, "rgb-%d" % i, float(i + 1), depth=d)
            for label, bbox, _ in SCHEDULE:
                det = FixedDetector(schedule=((label, bbox, lambda _i: True),))(f)
                self.assertEqual(len(process_frame(f, det)), 1, (i, label))


def write_arm(d, tracks_state="current_confirmed", drop_seq=None, kv=True):
    os.makedirs(d)
    rows, rep = [], ["seq,source_idx,stamp_ns,publish_raw_ns"]
    for s in range(20):
        stamp = 10 ** 18 + s * 10 ** 8
        rep.append("%d,%d,%d,%d" % (s, s % 7, stamp, 5 * 10 ** 9 + s * 10 ** 8))
        if s == drop_seq:
            continue
        r = {"event": "frame_processed", "time": 100.0 + s * 0.1, "frame_id": "rgb-%d" % stamp,
             "acquisition_timestamp": 100.0 + s * 0.1 - 0.25,
             "observation_count": int(s % 5 == 0),
             "tracks": [{"id": "chair-1", "label": "chair", "state": tracks_state if s >= 10 else "current_confirmed",
                         "reason": None, "position_m": [0.1, 0.2, 0.5], "last_seen": 1.0 * s}]}
        if kv:
            rx = 5 * 10 ** 9 + s * 10 ** 8 + 2 * 10 ** 6       # publish->rx 2 ms
            r["geometry"] = {"result": "fused", "roi_core": "pl", "boxes_sent": 1, "capture_nano_ns": rx,
                             "frame_age_ms": 30.0, "chain_ms": {"rx_to_start_ms": 10.0, "start_to_send_ms": 5.0,
                                                                "send_to_back_ms": 12.0, "back_to_memory_ms": 3.0},
                             "kv_times_ms": {"core_device": 2.0}, "roi_times_ms": None}
        rows.append(r)
    rows.append({"event": "geometry_close", "time": 200.0, "counts": {"ok": 19}, "shadow": {"frames": 19, "equal": 19}})
    open(d + "/events.jsonl", "w").write("".join(json.dumps(r) + "\n" for r in rows))
    open(d + "/replay.csv", "w").write("\n".join(rep) + "\n")
    snaps = [{"label": "start", "wall": 100.0, "ticks": {"runtime": 1000, "stub": 10, "replay": 50},
              "proc_stat_cpu": [1000, 0, 0, 5000, 0, 0, 0, 0, 0, 0], "hz": 100, "ncpu": 6, "loadavg": ["0", "0", "0"]},
             {"label": "end", "wall": 102.0, "ticks": {"runtime": 1100, "stub": 12, "replay": 60},
              "proc_stat_cpu": [1300, 0, 0, 5900, 0, 0, 0, 0, 0, 0], "hz": 100, "ncpu": 6, "loadavg": ["0", "0", "0"]}]
    open(d + "/cpu_snaps.jsonl", "w").write("".join(json.dumps(s) + "\n" for s in snaps))


class Report(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="cgr_")

    def test_chain_cpu_and_compare(self):
        write_arm(self.tmp + "/kv")
        write_arm(self.tmp + "/cpu", kv=False)
        s = rr.arm_summary(self.tmp + "/kv")
        self.assertEqual(s["chain_ms"]["publish_to_rx_ms"]["median"], 2.0)
        self.assertEqual(s["chain_ms"]["publish_to_memory_ms"]["median"], 32.0)
        self.assertEqual(s["kv_times_ms"]["core_device"]["median"], 2.0)
        self.assertEqual(s["roi_core"], {"pl": 20})
        self.assertEqual(s["stamp_to_record_ms"]["median"], 250.0)
        cpu = s["cpu"]
        self.assertEqual(cpu["frames_in_window"], 20)          # times 100.0 .. 101.9
        self.assertEqual(cpu["procs"]["runtime"]["cpu_s"], 1.0)
        self.assertEqual(cpu["procs"]["runtime"]["pct_one_core"], 50.0)
        self.assertEqual(cpu["procs"]["runtime"]["ms_per_frame"], 50.0)
        self.assertEqual(cpu["machine"]["busy_core_equiv"], 1.5)   # 300 busy ticks / 100 Hz / 2 s
        c = rr.compare(s, rr.arm_summary(self.tmp + "/cpu"))
        self.assertEqual((c["aligned_frames"], c["tracks_equal"], c["observation_count_equal"]), (20, 20, 20))

    def test_compare_positive_controls(self):
        write_arm(self.tmp + "/a")
        write_arm(self.tmp + "/b", tracks_state="temporarily_occluded", drop_seq=3)
        c = rr.compare(rr.arm_summary(self.tmp + "/a"), rr.arm_summary(self.tmp + "/b"))
        self.assertEqual(c["aligned_frames"], 19)
        self.assertEqual(c["tracks_equal"], 9)                   # seq 0..9 minus the dropped 3
        self.assertEqual(c["first_track_diff_seq"], 10)
        self.assertEqual(c["first_drop_difference_seq"], 3)


class Scripts(unittest.TestCase):
    def test_shell_syntax(self):
        for name in ("ros_arm.sh", "nano_ros_arm.sh"):
            subprocess.check_call(["bash", "-n", os.path.join(ROOT, "tools", "car_geometry_kv", name)])


if __name__ == "__main__":
    unittest.main()
