"""10-27 console first-person boxes (owner: same boxes as the car screen AR on the console's first-person view).

Known answers for host/hub/fpv_boxes.py (the car's choice: ROI-answered rows, <= 5, same-class NMS 0.5, failed
frames not kept -> hold), cross-check against host/kv_ar.select_display_boxes (when numpy is there), the hub's
pairing on preview_frame (nearest depth stamp, stale > 1 s, no producer -> no ``ar`` key), hub_ros subscribing
/fpga/geometry_frame, and the bridge forwarding ``ar`` with Chinese names. No ROS, no sockets.
"""
import json
import struct
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ui/geomem/bridge"))

from host.hub import fpv_boxes as fb  # noqa: E402
from host.hub.hub_core import HubCore  # noqa: E402

NS = 10 ** 9


def row(tid, code, xyxy, word="surface_or_uncertain"):
    return {"track_id": tid, "geometry_evidence": word, "code": code, "roi_xyxy": xyxy, "counts": None}


def body(stamp_s, rows, failure=None, fid=7, wh=(640, 480)):
    return {"v": 1, "stamp_ros_s": stamp_s, "verdicts": [] if failure else rows, "failure": failure,
            "producer": {"frame_id": fid, "depth_wh": list(wh) if wh else None}}


MEM = {"tracks": [{"id": "t1", "label": "bottle", "number": 1}, {"id": "t2", "label": "bottle", "number": 2},
                  {"id": "t3", "label": "cup"}, {"id": "t4", "label": "backpack", "number": 1}]}


class FrameBoxes(unittest.TestCase):
    def test_car_choice_known_answer(self):
        rows = [row("t1", 0, [100, 100, 199, 199]),
                row("t2", 2, [110, 110, 209, 209]),          # same class, IoU 0.68 > 0.5 -> hidden (car NMS)
                row("t3", 3, [105, 105, 204, 204]),          # other class: kept despite the overlap
                row("t4", None, None, "out_of_view"),        # host-resolved, no ROI answer -> not drawn
                row("t5", 0, [600, 400, 640, 410]),          # x1 = 640 outside 640 wide -> skipped (kv_ar too)
                row("t6", 1, [0, 0, 9, 9])]                  # code outside {0,2,3}
        f = fb.frame_boxes(body(5.0, rows), fb.track_index(MEM))
        self.assertEqual([(b["id"], b["v"], b["label"], b["number"]) for b in f["boxes"]],
                         [("t1", 0, "bottle", 1), ("t3", 3, "cup", None)])
        self.assertEqual((f["hidden"], f["depth_wh"], f["frame_id"]), (1, [640, 480], 7))

    def test_max_five_in_request_order(self):
        rows = [row("t%d" % k, 0, [k * 60, 0, k * 60 + 50, 50]) for k in range(8)]
        f = fb.frame_boxes(body(1.0, rows), {})
        self.assertEqual([b["id"] for b in f["boxes"]], ["t0", "t1", "t2", "t3", "t4"])
        self.assertEqual(f["hidden"], 3)

    def test_v2_colour_follows_nano_word(self):
        # 10-27 candidate: under geometry v2 the kernel code is its 0.9 echo; the colour follows the Nano word
        rows = [row("t1", 0, [0, 0, 49, 49], "free"), row("t3", 0, [100, 0, 149, 49], "occluded"),
                row("t4", 2, [200, 0, 249, 49], "surface_or_uncertain"), row("t6", 1, [300, 0, 349, 49], "free")]
        b2 = body(1.0, rows); b2["producer"]["geometry_version"] = "v2"
        f = fb.frame_boxes(b2, fb.track_index(MEM))
        self.assertEqual([(b["id"], b["v"]) for b in f["boxes"]], [("t1", 2), ("t3", 3), ("t4", 0)])
        f1 = fb.frame_boxes(body(1.0, rows), fb.track_index(MEM))    # v1 body: colour = kernel code, unchanged
        self.assertEqual([(b["id"], b["v"]) for b in f1["boxes"]], [("t1", 0), ("t3", 0), ("t4", 2)])

    def test_failed_or_malformed_not_kept(self):
        self.assertIsNone(fb.frame_boxes(body(1.0, [], failure="expired_output"), {}))
        self.assertIsNone(fb.frame_boxes({"stamp_ros_s": float("nan"), "verdicts": [], "failure": None}, {}))
        self.assertIsNone(fb.frame_boxes("x", {}))

    def test_same_as_kv_ar_selection(self):
        try:
            from host.kv_ar import select_display_boxes
        except ImportError:
            self.skipTest("numpy unavailable")
        boxes = [(100, 100, 199, 199, 0, "bottle"), (110, 110, 209, 209, 2, "bottle"), (150, 150, 249, 249, 0, "bottle"),
                 (300, 10, 350, 60, 3, "cup"), (305, 12, 352, 61, 0, "cup"), (0, 0, 20, 20, 0, "key"),
                 (400, 300, 500, 400, 0, "key"), (420, 300, 520, 400, 0, "glasses")]
        car, _ = select_display_boxes([(x0, y0, x1, y1, v, 0, k) for (x0, y0, x1, y1, v, k) in boxes], 5, 0.5)
        mine, _ = fb.select([{"xyxy": [x0, y0, x1, y1], "v": v, "key": k} for (x0, y0, x1, y1, v, k) in boxes])
        self.assertEqual([tuple(c[:4]) for c in car], [tuple(m["xyxy"]) for m in mine])


class HubPairing(unittest.TestCase):
    def preview(self, core, stamp, t):
        core.preview_last_ns = None
        core.feed_preview({"w": 640, "h": 480, "stamp_ros_s": stamp}, b"\xff\xd8x\xff\xd9", t)
        return core.preview_frame(t)[0]

    def test_no_producer_no_ar_key(self):
        core = HubCore("live")
        self.assertNotIn("ar", self.preview(core, 100.0, 5 * NS))

    def test_nearest_hold_and_stale(self):
        core = HubCore("live")
        core.feed_memory(json.dumps(MEM), NS)
        core.feed_geometry_frame(json.dumps(body(100.00, [row("t1", 0, [10, 20, 30, 40])], fid=1)), 2 * NS)
        core.feed_geometry_frame(json.dumps(body(100.10, [row("t3", 3, [50, 60, 70, 80])], fid=2)), 2 * NS)
        core.feed_geometry_frame(json.dumps(body(100.20, [], failure="expired_output", fid=3)), 2 * NS)
        ar = self.preview(core, 100.09, 3 * NS)["ar"]                       # nearest = frame 2
        self.assertEqual((ar["state"], ar["frame_id"], ar["boxes"][0]["id"], ar["boxes"][0]["label"]),
                         ("ok", 2, "t3", "cup"))
        self.assertAlmostEqual(ar["dt_ms"], 10.0, places=3)
        ar = self.preview(core, 100.21, 4 * NS)["ar"]                       # frame 3 failed: frame 2 held, no flicker
        self.assertEqual((ar["state"], ar["frame_id"]), ("ok", 2))
        ar = self.preview(core, 101.5, 5 * NS)["ar"]                        # 1.4 s past the newest good frame
        self.assertEqual((ar["state"], ar["boxes"]), ("stale", []))
        core.feed_geometry_frame(json.dumps(body(101.5, [], fid=4)), 6 * NS)   # nothing in view: ok, empty
        ar = self.preview(core, 101.5, 6 * NS)["ar"]
        self.assertEqual((ar["state"], ar["boxes"]), ("ok", []))
        self.assertGreaterEqual(core.counts["fpv_geo_not_kept"], 1)

    def test_hub_ros_subscribes_geometry_frame(self):
        from host.hub import hub_ros
        self.assertIn("/fpga/geometry_frame", hub_ros.SUBSCRIPTIONS)


class BridgeForward(unittest.TestCase):
    def test_ar_forwarded_with_chinese_name(self):
        import geomem_bridge as gb
        import gmb_proto as gp
        b = gb.Bridge(clock=lambda: 10.0)
        b.client = types.SimpleNamespace(link_state="up")

        class Sess(object):
            preview = []

            def put_preview(self, d):
                self.preview.append(d)
        s = Sess()
        b.sessions.add(s)
        jpeg = b"\xff\xd8x\xff\xd9"
        ar = {"state": "ok", "dt_ms": 12.0, "frame_id": 3, "depth_wh": [640, 480], "hidden": 1,
              "boxes": [{"id": "t1", "label": "bottle", "number": 2, "v": 0, "xyxy": [1, 2, 30, 40]},
                        {"id": "t9", "label": "bottle", "number": None, "v": 7, "xyxy": [1, 2, 3, 4]}]}
        b._on_frame("PREVIEW", ({"encoding": "jpeg", "w": 640, "h": 480, "age_ms": 5, "ar": ar}, jpeg))
        b._on_frame("PREVIEW", ({"encoding": "jpeg", "w": 640, "h": 480, "age_ms": 5, "ar": {"state": "bad"}}, jpeg))
        b._on_frame("PREVIEW", ({"encoding": "jpeg", "w": 640, "h": 480, "age_ms": 5}, jpeg))
        heads = []
        for f in s.preview:
            (L,) = struct.unpack_from("<I", f, 0)
            heads.append(json.loads(f[4:4 + L].decode("utf-8")))
        self.assertEqual(len(heads), 3)                                # the picture always goes out
        self.assertEqual(heads[0]["ar"]["boxes"], [{"id": "t1", "name": gp.object_name("bottle", None, 2),
                                                    "label": "bottle", "number": 2, "v": 0, "xyxy": [1, 2, 30, 40]}])
        self.assertEqual(heads[0]["ar"]["boxes"][0]["name"], "瓶子 2")
        self.assertEqual((heads[0]["ar"]["state"], heads[0]["ar"]["depthWh"]), ("ok", [640, 480]))
        self.assertNotIn("ar", heads[1])
        self.assertNotIn("ar", heads[2])
        self.assertEqual(b.counts["preview_ar_invalid"], 1)


if __name__ == "__main__":
    unittest.main()
