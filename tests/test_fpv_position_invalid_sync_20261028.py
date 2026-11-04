"""10-28 first-person boxes follow the map (owner: "taken away in front of the car / found gone on return: the
3D map drops the object, but its box stays in the first-person view").

Cause: the producer plans a ROI row for every memory track (position_invalid too -- the existence score still
takes those votes) and both first-person displays drew every answered row, while the console map hides
position_invalid tracks (operator/src/remote.js). Known answers:
  1. console (host/hub/fpv_boxes.py): a position_invalid row is dropped before NMS / the 5-box cut (it neither
     hides a shown same-class box nor takes a slot); the hub drops it at pairing time too when the memory says
     "left" after the frame was kept; a detection bringing the track back shows the box again;
  2. car screen: geo_producer.roi_row_labels marks a position_invalid row's name with "~" (on the wire too),
     kv_ar.fill_ar_inputs does not draw a "~" row and counts it (invalid_boxes), the ROI rows are untouched;
  3. browser (operator/src/fpv-boxes.js shownBoxes): same rule on the console snapshot (node, when present).
Positive control: without the hide set, the old behaviour (box kept) comes back.
"""
import json
import os
import shutil
import subprocess
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from host.hub import fpv_boxes as fb  # noqa: E402
from host.hub.hub_core import HubCore  # noqa: E402

NS = 10 ** 9


def row(tid, code, xyxy):
    return {"track_id": tid, "geometry_evidence": "surface_or_uncertain", "code": code, "roi_xyxy": xyxy,
            "counts": None}


def body(stamp_s, rows, fid=1):
    return {"v": 1, "stamp_ros_s": stamp_s, "verdicts": rows, "failure": None,
            "producer": {"frame_id": fid, "depth_wh": [640, 480]}}


def mem(states, rev=1):
    labels = {"t1": "bottle", "t2": "bottle", "t3": "cup", "t4": "book", "t5": "key", "t6": "glasses"}
    return {"revision": rev, "tracks": [{"id": k, "label": labels[k], "number": 1, "state": s}
                                        for k, s in states.items()]}


class Console(unittest.TestCase):
    def test_invalid_row_dropped_before_selection(self):
        m = mem({"t1": "position_invalid", "t2": "current_confirmed", "t3": "current_confirmed",
                 "t4": "current_confirmed", "t5": "current_confirmed", "t6": "current_confirmed"})
        rows = [row("t1", 2, [100, 100, 199, 199]),          # left: magenta box at the old place
                row("t2", 0, [110, 110, 209, 209]),          # same class, IoU 0.68 with t1: must stay
                row("t3", 0, [300, 0, 349, 49]), row("t4", 0, [400, 0, 449, 49]),
                row("t5", 0, [500, 0, 549, 49]), row("t6", 0, [0, 300, 49, 349])]
        f = fb.frame_boxes(body(1.0, rows), fb.track_index(m), fb.hidden_ids(m))
        self.assertEqual([b["id"] for b in f["boxes"]], ["t2", "t3", "t4", "t5", "t6"])
        self.assertEqual((f["hidden"], f["invalid"]), (0, 1))
        old = fb.frame_boxes(body(1.0, rows), fb.track_index(m))          # positive control: old behaviour
        self.assertEqual([b["id"] for b in old["boxes"]], ["t1", "t3", "t4", "t5", "t6"])

    def preview(self, core, stamp, t):
        core.preview_last_ns = None
        core.feed_preview({"w": 640, "h": 480, "stamp_ros_s": stamp}, b"\xff\xd8x\xff\xd9", t)
        return core.preview_frame(t)[0]["ar"]

    def test_hub_follows_memory_leave_and_return(self):
        core = HubCore("live")
        core.feed_memory(json.dumps(mem({"t1": "current_confirmed"})), NS)
        core.feed_geometry_frame(json.dumps(body(100.0, [row("t1", 0, [10, 20, 30, 40])])), 2 * NS)
        self.assertEqual([b["id"] for b in self.preview(core, 100.0, 2 * NS)["boxes"]], ["t1"])
        # the bottle is taken away: memory says left after this frame was kept -> dropped at pairing
        core.feed_memory(json.dumps(mem({"t1": "position_invalid"}, 2)), 3 * NS)
        self.assertEqual(self.preview(core, 100.05, 3 * NS)["boxes"], [])
        self.assertGreaterEqual(core.counts["fpv_pair_position_invalid"], 1)
        # next producer frame still checks the row (score votes) but it is not drawn
        core.feed_geometry_frame(json.dumps(body(100.1, [row("t1", 2, [10, 20, 30, 40])], 2)), 4 * NS)
        ar = self.preview(core, 100.1, 4 * NS)
        self.assertEqual((ar["state"], ar["boxes"]), ("ok", []))
        self.assertGreaterEqual(core.counts["fpv_box_position_invalid"], 1)
        # put back and detected again -> the box comes back with the map
        core.feed_memory(json.dumps(mem({"t1": "current_confirmed"}, 3)), 5 * NS)
        core.feed_geometry_frame(json.dumps(body(100.2, [row("t1", 0, [10, 20, 30, 40])], 3)), 5 * NS)
        self.assertEqual([b["id"] for b in self.preview(core, 100.2, 5 * NS)["boxes"]], ["t1"])


class CarScreen(unittest.TestCase):
    def test_producer_marks_invalid_rows(self):
        from host.geo_producer import roi_row_labels, AR_HIDE_MARK
        m = {"tracks": [{"id": "a", "label": "bottle", "state": "position_invalid"},
                        {"id": "b", "label": "cup", "state": "current_confirmed"},
                        {"id": "c", "state": "position_invalid"}]}
        plan = types.SimpleNamespace(row_map={1: "b", 2: "a", 3: "c"},
                                     batch=types.SimpleNamespace(rows=[types.SimpleNamespace(row_id=i) for i in (1, 2, 3)]))
        self.assertEqual(AR_HIDE_MARK, "~")
        self.assertEqual(roi_row_labels(m, plan), ["cup", "~bottle", "~"])
        from host.kv_link import clean_wire_label
        self.assertEqual([clean_wire_label(n)[0] for n in ("~bottle", "~medicine bottle", "~")],
                         [b"~bottle", b"~medicin", b"~"])

    def test_ar_skips_marked_rows(self):
        try:
            from host import kv_ar
            import test_kv_ar_display_boxes as disp
        except ImportError as e:
            self.skipTest("numpy / AR test base unavailable: %s" % e)
        rows = list(disp.ROWS)
        rows[0] = (rows[0][0], rows[0][1], b"~cup")          # A left: B (same class, overlapping) must show
        snap = disp.snap_with(rows)
        before = (list(snap["roi"]["rows"]), list(snap["roi_rows"]))
        inp = kv_ar.fill_ar_inputs(snap, disp.base.layout(), source="live", box_max=5, box_nms=0.5)
        got = [b[:4] for b in disp.boxes_of(inp)]
        want = [r[0][2:6] for i, r in enumerate(disp.ROWS) if i in (1, 2, 3, 4, 5)]
        self.assertEqual(got, want)
        self.assertEqual((inp["boxes"], inp["hidden_boxes"], inp["invalid_boxes"]), (5, 1, 1))
        self.assertEqual((snap["roi"]["rows"], snap["roi_rows"]), before)      # display only
        rows[0] = (rows[0][0], rows[0][1], "~cup")           # str names (replay path) too
        inp = kv_ar.fill_ar_inputs(disp.snap_with(rows), disp.base.layout(), source="live", box_max=5, box_nms=0.5)
        self.assertEqual(inp["invalid_boxes"], 1)


class Browser(unittest.TestCase):
    def test_shown_boxes(self):
        node = shutil.which("node") or str(ROOT / "ui/geomem/runtime/node")
        if not os.path.exists(node) and not shutil.which(node):
            self.skipTest("node unavailable")
        mod = (ROOT / "ui/geomem/operator/src/fpv-boxes.js").as_uri()
        js = ("import(%s).then(m=>{const b=[{id:'t1'},{id:'t2'},{id:null}];"
              "const o=[{id:'t1',state:'position_invalid'},{id:'t2',state:'current_confirmed'}];"
              "console.log(JSON.stringify([m.shownBoxes(b,o).map(x=>x.id),m.shownBoxes(b,[]).length,"
              "m.shownBoxes(b,undefined).length]));})" % json.dumps(mod))
        out = subprocess.run([node, "--input-type=module", "-e", js], capture_output=True, text=True, timeout=30)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertEqual(json.loads(out.stdout), [["t2", None], 3, 3])


if __name__ == "__main__":
    unittest.main()
