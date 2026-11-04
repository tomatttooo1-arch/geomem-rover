"""detect_range report: known answer on a synthetic run (10 frames, phone hit in 8, remote never)."""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools", "car_run"))
import detect_range  # noqa: E402


class DetectRangeTest(unittest.TestCase):
    def test_known_answer(self):
        d = tempfile.mkdtemp()
        rt, mk = os.path.join(d, "runtime.jsonl"), os.path.join(d, "marks.jsonl")
        with open(rt, "w") as f:
            for i in range(10):
                dets = [{"label": "cell phone", "score": 0.5, "bbox": [300, 200, 340, 220]}] if i < 8 else []
                if i == 0:
                    dets.append({"label": "book", "score": 0.4, "bbox": [0, 0, 10, 10]})
                f.write(json.dumps({"event": "frame_processed", "acquisition_timestamp": 100 + i * 0.1,
                                    "detections": dets, "tracks": []}) + "\n")
            f.write(json.dumps({"event": "frame_processed", "acquisition_timestamp": 200, "detections": [
                {"label": "cell phone", "score": 0.5, "bbox": [300, 200, 340, 220]}], "tracks": []}) + "\n")
        with open(mk, "w") as f:
            stale = {"round": 1, "t_start": 50, "t_end": 51, "items": [{"object": "phone_flat", "distance_m": 0.9}]}
            f.write(json.dumps(stale) + "\n")          # redone round: the later row wins
            f.write(json.dumps({"round": 1, "t_start": 100, "t_end": 101, "items": [
                {"object": "phone_flat", "distance_m": 0.9}, {"object": "remote", "distance_m": 1.2}]}) + "\n")
        res = detect_range.analyse(detect_range.load_frames(rt), detect_range.load_marks(mk), 640, 86.0)
        phone, remote = res
        self.assertEqual((phone["frames"], phone["hits"], phone["rate"]), (10, 8, 0.8))
        self.assertEqual(phone["box_w_px"], 40.0)
        self.assertEqual(phone["off_axis_deg"], 0.0)
        self.assertEqual(remote["hits"], 0)
        self.assertEqual(phone["other_labels"], {"book": 1})
        s = detect_range.summary(res, 0.8)
        self.assertEqual(s["phone_flat"]["farthest_ok_m"], 0.9)
        self.assertIsNone(s["remote"]["farthest_ok_m"])

    def test_plan_each_object_each_distance_once(self):
        plan = detect_range.default_plan()
        self.assertEqual(len(plan), 5)
        for obj in detect_range.ROUND_OBJECTS:
            got = sorted(it["distance_m"] for st in plan for it in st["items"] if it["object"] == obj)
            self.assertEqual(got, sorted(detect_range.DISTANCES))
        for st in plan:
            ds = [it["distance_m"] for it in st["items"]]
            self.assertEqual(len(ds), len(set(ds)))


if __name__ == "__main__":
    unittest.main()
