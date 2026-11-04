"""AR colour refresh between geometry frames (AR_LATENCY_PLAN_20261027 plan C, owner 10-27).

Between two geometry frames the AR sender re-sends the LAST geometry frame's inputs (boxes, depth inset,
panel, its own growing age) with the NEWEST colour frame every tune colour_refresh_ms. Checked with the
real helper (host build, mock provider) and a fake colour sink whose newest colour key follows the clock:
  - refresh pictures carry the real frame's boxes / registers / age basis unchanged (no box-less flash)
    and a strictly newer colour each time; the screen rate rises well above the geometry rate;
  - a new geometry frame replaces the refreshed one at once and refreshes continue from it;
  - colour_refresh_ms 0 (tune) = the behaviour before: one picture per geometry frame;
  - no refresh once the last frame's age would reach max_age_ms (never turns a picture STALE), and
    none after nodepth_after_ms (the no-depth pull takes over as before);
  - a real frame whose own colour is older than the one on screen keeps the newer one (no jump back).
Run: cd tests && PYTHONPATH=..:. python3 -m unittest test_ar_colour_refresh_20261027 (3.13 and 3.8).
"""
import json
import os
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

from host import kv_ar  # noqa: E402
import test_kv_ar as base  # noqa: E402
import test_kv_ar_colour as tc  # noqa: E402

WD, HD = base.WD, base.HD
OFFSET = 7 * 10 ** 15        # sensor clock = KV CLOCK_MONOTONIC + OFFSET


class ClockSink:
    """Colour source whose newest frame was acquired 5 ms before the WANTed key; a real frame's own
    colour (no WANT by early() here) has the frame's sensor key."""

    def __init__(self):
        self.port = 0
        self.lock = threading.Lock()
        self.wants = {}
        self.real_keys = {}
        self.rgb = tc.gradient_rgb(seed=3)

    def has_source(self):
        return True

    def want(self, frame_id, capture_ns, key, tol_ns=None):
        with self.lock:
            self.wants[(int(frame_id), int(capture_ns))] = int(key)
        return True

    def take(self, frame_id, capture_ns, timeout_s):
        with self.lock:
            key = self.wants.pop((int(frame_id), int(capture_ns)), None)
            if key is None:
                key = self.real_keys.get(int(frame_id))
                if key is None:
                    return None, "not_wanted"
                return {"rgb": self.rgb, "width": WD, "height": HD, "source_key": key, "want_key": key}, None
        return {"rgb": self.rgb, "width": WD, "height": HD, "source_key": key - 5_000_000,
                "want_key": key}, None

    def describe(self):
        return {"fake": True}


class Recorder:
    """Wraps kv_ar.encode_ar_message: every message the sender writes, in order."""

    def __init__(self):
        self.inps = []
        self.orig = kv_ar.encode_ar_message

    def __enter__(self):
        def rec(inp):
            self.inps.append((time.monotonic(), inp))
            return self.orig(inp)
        kv_ar.encode_ar_message = rec
        return self

    def __exit__(self, *a):
        kv_ar.encode_ar_message = self.orig


class RefreshTests(unittest.TestCase):
    def presenter(self, d, sink, tune, max_age_ms=1000):
        cmd = [base.Env.helper(), "--provider", "mock", "--display", "none", "--mock-size", "%dx%d" % base.CANVAS,
               "--mock-permit", "1", "--stale-ms", "3000", "--refresh-ms", "1000"]
        tf = os.path.join(d, "tune.json")
        json.dump(tune, open(tf, "w"))
        return kv_ar.ArPresenter(cmd, source="live", max_age_ms=max_age_ms, out_dir=d, colour=sink,
                                 colour_main=True, colour_wait_ms=20, tune_file=tf)

    def real(self, sink, fid, elapsed_ns=100_000_000):
        s = base.synth_snap(frame_id=fid, nano_elapsed_ns=elapsed_ns)
        fm = int(time.monotonic() * 1e9)
        s["first_mono_ns"], s["capture_ns"] = fm, 5
        s["sensor_stamp_ns"] = OFFSET + fm - elapsed_ns
        sink.real_keys[fid] = s["sensor_stamp_ns"]
        return s

    def run_case(self, tune, frames, gap_s, tail_s, max_age_ms=1000, elapsed_ns=100_000_000):
        sink = ClockSink()
        with tempfile.TemporaryDirectory() as d, Recorder() as rec:
            ar = self.presenter(d, sink, tune, max_age_ms=max_age_ms)
            try:
                for i, fid in enumerate(frames):
                    if i:
                        time.sleep(gap_s)
                    ar.submit(self.real(sink, fid, elapsed_ns))
                time.sleep(tail_s)
                t = ar.telem()
            finally:
                ar.close()
            with open(os.path.join(d, "ar_colour.jsonl")) as f:
                log = [json.loads(x) for x in f]
        return [x for x in rec.inps if x[1]["frame_id"] in frames], t, log

    def test_refresh_keeps_boxes_and_brings_newer_colour(self):
        inps, t, log = self.run_case({"colour_main": 1, "colour_refresh_ms": 40, "nodepth_after_ms": 500},
                                     [11], 0, 0.35)
        self.assertGreaterEqual(len(inps), 5, t)
        real = inps[0][1]
        self.assertGreater(real["boxes"], 0)
        keys = [real["colour"]["source_key"]]
        for _t, inp in inps[1:]:
            self.assertEqual(inp["frame_id"], 11)
            self.assertTrue(np.array_equal(inp["desc"], real["desc"]))          # boxes kept: no box-less picture
            self.assertEqual(list(inp["regs"]), list(real["regs"]))
            self.assertEqual((inp["age_basis_ns"], inp["first_mono_ns"]),
                             (real["age_basis_ns"], real["first_mono_ns"]))       # the geometry's own age grows
            self.assertTrue(np.array_equal(inp["depth"], real["depth"]))
            self.assertIsNotNone(inp["colour"]["rgb"])
            keys.append(inp["colour"]["source_key"])
        self.assertEqual(keys, sorted(set(keys)))                                # strictly newer colours
        gaps = [b[0] - a[0] for a, b in zip(inps, inps[1:])]
        self.assertLess(max(gaps), 0.1)
        self.assertGreaterEqual(t["colour"]["refresh"]["sent"], 4)
        self.assertTrue(any(r.get("decision") == "refresh" for r in log))

    def test_real_frame_replaces_refresh_at_once(self):
        inps, t, log = self.run_case({"colour_main": 1, "colour_refresh_ms": 40, "nodepth_after_ms": 500},
                                     [21, 22, 23], 0.1, 0.1)
        fids = [inp["frame_id"] for _t, inp in inps]
        self.assertEqual(fids[0], 21)
        # every frame appears, in order; after a new frame no refresh of an older one follows
        seen = [f for i, f in enumerate(fids) if i == 0 or fids[i - 1] != f]
        self.assertEqual(seen, [21, 22, 23])
        self.assertGreater(len(fids), 6)
        for fid in (22, 23):
            first = fids.index(fid)
            self.assertNotEqual(inps[first][1]["desc"][1], 0)
        # a real frame's own (older) colour is not shown after a newer refresh colour: keys never go back
        keys = [inp["colour"]["source_key"] for _t, inp in inps]
        self.assertEqual(keys, sorted(keys))
        self.assertTrue(any(r.get("decision") == "paired_newer" for r in log))

    def test_tune_off_is_one_picture_per_frame(self):
        inps, t, log = self.run_case({"colour_main": 1, "colour_refresh_ms": 0, "nodepth_after_ms": 500},
                                     [31, 32], 0.1, 0.3)
        self.assertEqual([inp["frame_id"] for _t, inp in inps], [31, 32])
        self.assertEqual(t["colour"]["refresh"]["sent"], 0)
        self.assertEqual([r["decision"] for r in log], ["paired", "paired"])

    def test_no_refresh_near_max_age(self):
        # the frame is already 900 ms old with max_age 1000: refreshing it would only show STALE sooner
        inps, t, _log = self.run_case({"colour_main": 1, "colour_refresh_ms": 40, "nodepth_after_ms": 500},
                                      [41], 0, 0.3, elapsed_ns=900_000_000)
        self.assertEqual(len(inps), 1)

    def test_refresh_stops_at_nodepth_handover(self):
        inps, t, log = self.run_case({"colour_main": 1, "colour_refresh_ms": 40, "nodepth_after_ms": 200},
                                     [51], 0, 0.45)
        t0 = inps[0][0]
        refresh = [ts - t0 for ts, inp in inps[1:] if inp["desc"][1] > 0]
        nodepth = [ts - t0 for ts, inp in inps[1:] if inp["desc"][1] == 0]
        self.assertTrue(refresh and nodepth)
        self.assertLess(max(refresh), 0.2 + 0.01)
        self.assertGreaterEqual(min(nodepth), 0.2 - 0.01)


if __name__ == "__main__":
    unittest.main()
