"""Input accounting of the geometry producer (v8, 2026-10-12: depth frames lost before the callback).

1. host.geo_producer.InputGapCounter on scripted stamps (no ROS): a regular stream with jitter counts
   nothing; one frame taken out is one gap of one frame; two in a row are one gap of two; the period is
   the source's own (10 Hz and 30 Hz), a gap step never enters the period window; nothing is counted
   before MIN_STEPS steps; a repeated / older stamp is ``non_monotonic``, not a gap. Variant: a counter
   whose GAP_FACTOR is too large sees none of the removed frames (the assertions depend on the rule).
2. Wiring, read from the source (no ROS): the callback feeds the counter BEFORE the rate limit and the
   mailbox, so a rate-skipped or superseded frame is never counted as lost.
3. With rclpy (ROS image only, skipped on the host interpreters): the real replay camera with
   --debug-skip-frames (frames the camera never sends) into the real producer node (cpu backend):
   summary input.missed_frames == the number skipped, the input_gap rows name the stamps around each
   hole, --input-log has one row per arrival; the same run without skips counts 0.
Python 3.8+.
"""
import csv
import json
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host.geo_producer import InputGapCounter  # noqa: E402

try:
    import rclpy  # noqa: F401
    import tf2_ros  # noqa: F401
    HAVE_ROS = True
except Exception:  # noqa: BLE001
    HAVE_ROS = False


def stamps(n, period_ns, removed=(), jitter_ns=0, seed=7):
    rng = random.Random(seed)
    out = []
    for k in range(n):
        if k in removed:
            continue
        out.append(1000000000 + k * period_ns + (rng.randint(-jitter_ns, jitter_ns) if jitter_ns else 0))
    return out


def run(counter, seq):
    events = []
    for i, s in enumerate(seq):
        ev = counter.feed(s, 5000 + i)
        if ev is not None:
            events.append(ev)
    return events


class CounterTests(unittest.TestCase):
    def test_regular_stream_counts_nothing(self):
        for period in (100000000, 33333333):
            c = InputGapCounter()
            self.assertEqual(run(c, stamps(300, period, jitter_ns=period // 20)), [])
            s = c.summary()
            self.assertEqual((s["arrivals"], s["gaps"], s["missed_frames"], s["non_monotonic"]), (300, 0, 0, 0))
            self.assertAlmostEqual(s["period_ms"], period / 1e6, delta=period / 1e6 * 0.06)

    def test_one_removed_frame_is_one_gap_of_one(self):
        c = InputGapCounter()
        seq = stamps(200, 100000000, removed={50}, jitter_ns=3000000)
        ev = run(c, seq)
        self.assertEqual(len(ev), 1)
        self.assertEqual(ev[0]["event"], "input_gap")
        self.assertEqual(ev[0]["missed"], 1)
        self.assertEqual((ev[0]["prev_stamp_ns"], ev[0]["stamp_ns"]), (seq[49], seq[50]))
        self.assertAlmostEqual(ev[0]["step_ms"], 200.0, delta=7.0)
        self.assertEqual(ev[0]["raw_ns"], 5050)
        self.assertEqual((c.summary()["gaps"], c.summary()["missed_frames"]), (1, 1))

    def test_two_in_a_row_and_separate_holes(self):
        c = InputGapCounter()
        ev = run(c, stamps(300, 100000000, removed={80, 81, 150, 220}, jitter_ns=2000000))
        self.assertEqual([e["missed"] for e in ev], [2, 1, 1])
        self.assertEqual((c.summary()["gaps"], c.summary()["missed_frames"]), (3, 4))
        self.assertAlmostEqual(c.summary()["period_ms"], 100.0, delta=3.0)     # gap steps stay out of the window
        self.assertAlmostEqual(c.summary()["max_step_ms"], 300.0, delta=5.0)

    def test_source_period_not_rate_limit(self):
        # a 30 fps camera: one missing frame is a 66 ms step, far below a 10 Hz period, still a gap
        c = InputGapCounter()
        ev = run(c, stamps(400, 33333333, removed={200}, jitter_ns=1000000))
        self.assertEqual([e["missed"] for e in ev], [1])
        self.assertAlmostEqual(ev[0]["period_ms"], 33.3, delta=1.5)

    def test_nothing_counted_before_min_steps(self):
        c = InputGapCounter()
        self.assertEqual(run(c, stamps(40, 100000000, removed={3})), [])
        self.assertEqual(c.summary()["missed_frames"], 0)
        self.assertLess(InputGapCounter.MIN_STEPS, 20)

    def test_non_monotonic_is_not_a_gap(self):
        c = InputGapCounter()
        seq = stamps(60, 100000000)
        seq.insert(30, seq[29])            # the same stamp again
        seq.insert(41, seq[35])            # an older stamp
        self.assertEqual(run(c, seq), [])
        s = c.summary()
        self.assertEqual((s["non_monotonic"], s["gaps"], s["arrivals"]), (2, 0, 62))

    def test_variant_rule_too_loose_sees_nothing(self):
        class Loose(InputGapCounter):
            GAP_FACTOR = 5.0
        seq = stamps(300, 100000000, removed={80, 81, 150}, jitter_ns=2000000)
        self.assertEqual(run(Loose(), seq), [])
        self.assertEqual(len(run(InputGapCounter(), seq)), 2)


class WiringTests(unittest.TestCase):
    def test_counter_fed_before_rate_limit_and_mailbox(self):
        src = (ROOT / "host" / "ros_geo_producer.py").read_text()
        body = src[src.index("        def on_depth(self, msg):"):src.index("        def lookup(self, target")]
        feed, rate, put = (body.index("self.input_gaps.feed("), body.index('self.counts["rate_skipped"] += 1'),
                           body.index("self.slot.put("))
        self.assertLess(feed, rate)
        self.assertLess(rate, put)
        self.assertIn('"input": dict(node.input_gaps.summary()', src)
        self.assertIn('"input_missed": self.input_gaps.counts["missed_frames"]', src)


def write_prep(d, frames=20, w=64, h=48):
    """A tiny prepared depth set (flat floor seen by the default mount is not needed: the test reads
    the input accounting, not the geometry)."""
    import numpy as np
    depth = np.full((frames, h, w), 1500, dtype="<u2")
    depth.tofile(os.path.join(d, "depth_u16.bin"))
    K = [[50.0, 0.0, w / 2.0], [0.0, 50.0, h / 2.0], [0.0, 0.0, 1.0]]
    with open(os.path.join(d, "meta.json"), "w") as fp:
        json.dump({"width": w, "height": h, "frames": frames, "depth_file": "depth_u16.bin",
                   "depth_unit_m": 0.001, "K": K}, fp)


@unittest.skipUnless(HAVE_ROS, "needs rclpy + tf2_ros (ROS 2 image)")
class RosKnownAnswerTests(unittest.TestCase):
    def chain(self, skip, domain):
        d = tempfile.mkdtemp(prefix="input_gap_")
        prep = os.path.join(d, "prep")
        os.mkdir(prep)
        write_prep(prep)
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", PYTHONUNBUFFERED="1",
                   PYTHONPATH=str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""))
        prod = subprocess.Popen(
            [sys.executable, "-m", "host.ros_geo_producer", "--data-source", "replay", "--backend", "cpu",
             "--ground-z-m", "0.0", "--rate-hz", "10", "--duration-s", "14", "--no-roi", "--no-stall-diag",
             "--stats-out", os.path.join(d, "frames.jsonl"), "--summary-out", os.path.join(d, "summary.json"),
             "--input-log", os.path.join(d, "input.csv")],
            cwd=str(ROOT), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        cam = subprocess.Popen(
            [sys.executable, str(ROOT / "tools" / "car_geo_producer" / "replay_camera.py"), "--prep", prep,
             "--hz", "10", "--duration-s", "11", "--log", os.path.join(d, "replay.csv")]
            + (["--debug-skip-frames", ",".join(str(k) for k in skip)] if skip else []),
            cwd=str(ROOT), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        cam_out = cam.communicate(timeout=60)[0].decode(errors="replace")
        prod_out = prod.communicate(timeout=60)[0].decode(errors="replace")
        self.assertTrue(os.path.exists(os.path.join(d, "summary.json")), prod_out[-2000:])
        with open(os.path.join(d, "summary.json")) as fp:
            summary = json.load(fp)
        with open(os.path.join(d, "frames.jsonl")) as fp:
            rows = [json.loads(line) for line in fp]
        with open(os.path.join(d, "replay.csv"), newline="") as fp:
            sent = list(csv.DictReader(fp))
        with open(os.path.join(d, "input.csv"), newline="") as fp:
            arrivals = list(csv.DictReader(fp))
        return summary, rows, sent, arrivals, cam_out

    def test_skipped_frames_are_counted_and_none_without(self):
        skip = [30, 31, 60]
        summary, rows, sent, arrivals, cam_out = self.chain(skip, 71)
        self.assertEqual(cam_out.count("debug skip k="), 3, cam_out[-1500:])
        gaps = [r for r in rows if r.get("event") == "input_gap"]
        inp = summary["input"]
        self.assertEqual((inp["gaps"], inp["missed_frames"]), (2, 3), (inp, gaps))
        self.assertEqual(sorted(g["missed"] for g in gaps), [1, 2])
        by_k = dict((int(r["k"]), int(r["stamp_ns"])) for r in sent)
        self.assertNotIn(30, by_k)
        self.assertEqual(sorted((g["prev_stamp_ns"], g["stamp_ns"]) for g in gaps),
                         sorted([(by_k[29], by_k[32]), (by_k[59], by_k[61])]))
        # every frame the camera did send inside the producer's span arrived: transport lost none here
        got = set(int(r["stamp_ns"]) for r in arrivals)
        span = [s for s in by_k.values() if min(got) <= s <= max(got)]
        self.assertEqual([s for s in span if s not in got], [])
        self.assertEqual(inp["arrivals"], summary["counts"]["depth_rx"])
        self.assertEqual(len(arrivals), inp["arrivals"])
        self.assertEqual(inp["log"]["dropped"], 0)
        frames = [r for r in rows if "frame_id" in r and "event" not in r]
        self.assertGreater(len(frames), 60)
        self.assertEqual(frames[-1]["input_missed"], 3)
        # control: the same chain with nothing skipped counts nothing
        summary0, rows0, _, _, _ = self.chain([], 72)
        self.assertEqual((summary0["input"]["gaps"], summary0["input"]["missed_frames"]), (0, 0))
        self.assertEqual([r for r in rows0 if r.get("event") == "input_gap"], [])
        self.assertGreater(summary0["input"]["arrivals"], 80)


if __name__ == "__main__":
    unittest.main()
