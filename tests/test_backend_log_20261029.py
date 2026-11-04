"""host/backend_log.py: backend metadata in runtime events written in full once, then by reference (10-29)."""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host.backend_log import REF_KEY, BackendExpander, BackendLog, expand_rows  # noqa: E402

EXECUTION = {"device": "cuda", "backend": "trt", "model": "m.engine", "model_sha256": "ab" * 32,
             "names": ["person", "bottle"] * 40, "class_min_scores": {"backpack": 0.45}}


def reply(i, execution=EXECUTION, detector="yolo"):
    # the client builds a new dict per reply (json.loads), so equal content arrives as a new object
    return json.loads(json.dumps({"ok": True, "frame_id": "rgb-%d" % i, "timestamp": float(i),
                                  "detector": detector, "execution": execution, "device": "cuda",
                                  "inference_seconds": 0.01 * i}))


class BackendLogTest(unittest.TestCase):
    def test_full_once_then_short_with_reference(self):
        log = BackendLog()
        self.assertIsNone(log.field(None, 0.))           # before the first inference: None as before
        first = reply(1)
        self.assertEqual(log.field(first, 1.), first)
        self.assertEqual(log.field(first, 1.1), {"ok": True, "frame_id": "rgb-1", "timestamp": 1.0,
                                                  "detector": "yolo", "device": "cuda",
                                                  "inference_seconds": 0.01, REF_KEY: "rgb-1"})
        short = log.field(reply(2), 2.)
        self.assertNotIn("execution", short)
        self.assertEqual((short["frame_id"], short["timestamp"], short[REF_KEY]), ("rgb-2", 2.0, "rgb-1"))

    def test_change_and_refresh_write_full_again(self):
        log = BackendLog(refresh_s=60.)
        log.field(reply(1), 0.)
        other = dict(EXECUTION, model="other.engine")
        self.assertIn("execution", log.field(reply(2, other), 1.))          # model changed
        self.assertNotIn("execution", log.field(reply(3, other), 2.))
        self.assertIn("execution", log.field(reply(4, other, "frcnn"), 3.))  # detector changed
        self.assertNotIn("execution", log.field(reply(5, other, "frcnn"), 62.9))
        self.assertIn("execution", log.field(reply(6, other, "frcnn"), 63.))  # 60 s since the last full row
        self.assertIn("execution", log.field(reply(7, other, "frcnn"), 10.))  # clock went back (sim reset)

    def test_expand_restores_every_row_in_file_order(self):
        log = BackendLog()
        originals, rows = [], []
        for i, execution in enumerate([EXECUTION] * 3 + [dict(EXECUTION, device="cpu")] * 2):
            meta = reply(i, execution)
            originals.append(meta)
            rows.append(json.loads(json.dumps({"event": "frame_processed",
                                               "inference_backend": log.field(meta, float(i))})))
        rows.insert(0, {"event": "startup", "inference_backend": None})
        self.assertEqual(sum("execution" in r["inference_backend"] for r in rows[1:]), 2)
        expanded = expand_rows(rows)
        self.assertIsNone(expanded[0]["inference_backend"])
        self.assertEqual([r["inference_backend"] for r in expanded[1:]], originals)

    def test_expander_without_full_row_leaves_short(self):
        short = {"ok": True, "frame_id": "rgb-9", REF_KEY: "rgb-1"}
        self.assertEqual(BackendExpander().expand(dict(short)), short)


if __name__ == "__main__":
    unittest.main()
