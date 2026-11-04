"""Hub: planner route as a polyline in STATE (nav.path) and the producer's live/replay mark
(geometry_nano.frame_source, frame-level inconsistent + one EVENT per edge), spec B.3 / C.4 / C.7.
Known answers and positive controls; no ROS needed."""

import json
import math
import threading
import types
import unittest

from pc import fpgac
from host.hub import hub_core, hub_ros
from host.hub.hub_core import HubCore, NS, MS, rdp, simplify_path, summarize_plan
from host.hub.fake_kv import FakeKv, KvTelemetryClient

K = fpgac.KIND


def line(n, x0=0.0, y0=0.0, dx=0.01, dy=0.0):
    return [(x0 + i * dx, y0 + i * dy) for i in range(n)]


def zigzag(n, amp=0.5, step=0.05, x0=0.0, y0=0.0):
    return [(x0 + i * step, y0 + (amp if i % 2 else -amp)) for i in range(n)]


class SimplifyTests(unittest.TestCase):
    def test_straight_200_points_to_2(self):
        pts = line(200)
        self.assertEqual(rdp(pts, 0.025), [pts[0], pts[-1]])
        poly, eps = simplify_path(pts)
        self.assertEqual((poly, eps), ([[0.0, 0.0], [1.99, 0.0]], 0.025))

    def test_l_shape_to_3(self):
        pts = line(101, dx=0.02) + line(100, x0=2.0, y0=0.02, dx=0.0, dy=0.02)
        poly, eps = simplify_path(pts)
        self.assertEqual(poly, [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]])

    def test_small_wiggle_under_half_cell_is_dropped_bigger_kept(self):
        wiggle = [(i * 0.05, 0.02 if i == 20 else 0.0) for i in range(41)]
        self.assertEqual(len(simplify_path(wiggle)[0]), 2)             # 2 cm < 2.5 cm
        bump = [(i * 0.05, 0.03 if i == 20 else 0.0) for i in range(41)]
        # 3 cm kept; its neighbours are 2.85 cm off the new chords, so they stay too (exact RDP)
        self.assertEqual(simplify_path(bump)[0], [[0.0, 0.0], [0.95, 0.0], [1.0, 0.03], [1.05, 0.0], [2.0, 0.0]])

    def test_zigzag_over_cap_doubles_eps_keeps_ends(self):
        pts = zigzag(500)
        poly, eps = simplify_path(pts)
        self.assertLessEqual(len(poly), hub_core.PATH_MAX_POINTS)
        self.assertGreater(eps, hub_core.PATH_EPS0_M)
        self.assertEqual(poly[0], [round(pts[0][0], 2), round(pts[0][1], 2)])
        self.assertEqual(poly[-1], [round(pts[-1][0], 2), round(pts[-1][1], 2)])
        # positive control: the start tolerance alone would exceed the cap
        self.assertGreater(len(rdp(pts, hub_core.PATH_EPS0_M)), hub_core.PATH_MAX_POINTS)

    def test_byte_budget_with_worst_case_coordinates(self):
        import random
        r = random.Random(3)
        pts, x, y = [], -9999.99, -9990.0                 # 7-8 character coordinates, 2 m steps
        for _ in range(600):
            x += r.uniform(-2.0, 2.0)
            y += r.uniform(-2.0, 2.0)
            pts.append((x, y))
        poly, eps = simplify_path(pts)
        self.assertLessEqual(len(fpgac.dumps(poly)), hub_core.PATH_MAX_BYTES)
        self.assertLessEqual(len(poly), hub_core.PATH_MAX_POINTS)
        # positive control: by the point cap alone this path fits (<= 64) but is over the byte
        # cap, so it is the byte rule that doubled eps here
        loose, loose_eps = simplify_path(pts, max_bytes=10 ** 9)
        self.assertLessEqual(len(loose), hub_core.PATH_MAX_POINTS)
        self.assertGreater(len(fpgac.dumps(loose)), hub_core.PATH_MAX_BYTES)
        self.assertGreater(eps, loose_eps)
        self.assertGreater(len(fpgac.dumps([[-9999.99, -9999.99]] * 64)), hub_core.PATH_MAX_BYTES)

    def test_degenerate_inputs(self):
        self.assertEqual(simplify_path([(1.234, 5.678)])[0], [[1.23, 5.68]])
        loop = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)]       # chord of length 0
        self.assertEqual(simplify_path(loop)[0], [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 0.0]])
        long_line = line(5000, dx=0.001)                                # deep input, no recursion
        self.assertEqual(len(simplify_path(long_line)[0]), 2)

    def test_one_pass_equals_rdp_per_doubling(self):
        import random
        mismatches = []
        for seed in range(120):
            r = random.Random(seed)
            pts, x, y, h = [], 0.0, 0.0, 0
            noise = r.choice([0.0, 0.005, 0.02, 0.05, 0.3])
            for i in range(r.randint(1, 1200)):
                if i % r.randint(20, 200) == 0:
                    h = r.randrange(8)
                x += 0.05 * math.cos(h * math.pi / 4)
                y += 0.05 * math.sin(h * math.pi / 4)
                pts.append((x + r.uniform(-noise, noise), y + r.uniform(-noise, noise)))
            if simplify_path(pts) != hub_core.simplify_path_reference(pts):
                mismatches.append(seed)
        self.assertEqual(mismatches, [])
        # positive control: the comparison can fail (a different eps gives a different answer)
        bump = [(i * 0.05, 0.03 if i == 20 else 0.0) for i in range(41)]
        self.assertNotEqual(simplify_path(bump), hub_core.simplify_path_reference(bump, eps0=0.05))

    def test_summarize_plan_reasons(self):
        self.assertEqual(summarize_plan([], "map", 1.0), (0.0, None, "empty_plan"))
        self.assertEqual(summarize_plan([(0.0, 0.0), (float("nan"), 1.0)], "map", 1.0), (None, None, "bad_plan"))
        self.assertEqual(summarize_plan([(0.0, float("inf"))], "map", 1.0), (None, None, "bad_plan"))
        length, path, reason = summarize_plan([(0.0, 0.0), (3.0, 4.0), (3.0, 5.0)], "map", 12.5)
        self.assertEqual((length, reason), (6.0, None))
        self.assertEqual(path, {"frame_id": "map", "stamp_ros_s": 12.5, "points": [[0.0, 0.0], [3.0, 4.0], [3.0, 5.0]],
                                "n_source_poses": 3, "length_m": 6.0, "epsilon_m": 0.025})


class NavPathStateTests(unittest.TestCase):
    T = 10 * NS

    def core_with(self, xy, frame_id="map", stamp=77.25):
        core = HubCore("live", hub_boot_id="hb")
        length, path, reason = summarize_plan(xy, frame_id, stamp)
        core.feed_plan(length, self.T, path=path, path_reason=reason)
        return core

    def test_path_in_state_with_age_and_stamp_passed_through(self):
        core = self.core_with(line(200) + [(1.99, 1.0)])
        s = core.compose_state(self.T + 40 * MS + 7)
        p = s["nav"]["path"]
        self.assertIsNone(s["nav"]["path_reason"])
        self.assertEqual(p["points"], [[0.0, 0.0], [1.99, 0.0], [1.99, 1.0]])
        self.assertEqual((p["frame_id"], p["stamp_ros_s"], p["n_source_poses"]), ("map", 77.25, 201))
        self.assertEqual(p["recv_age_ms"], (40 * MS + 7) / 1e6)
        self.assertAlmostEqual(p["length_m"], 2.99, places=9)
        self.assertEqual(s["nav"]["path_len_m"], p["length_m"])
        self.assertEqual(s["nav"]["age_ms"], (40 * MS + 7) / 1e6)
        fpgac.dumps(s)

    def test_three_reasons(self):
        s = HubCore("live").compose_state(self.T)
        self.assertEqual((s["nav"]["path"], s["nav"]["path_reason"], s["nav"]["path_len_m"]),
                         (None, "no_plan_received", None))
        s = self.core_with([]).compose_state(self.T)
        self.assertEqual((s["nav"]["path"], s["nav"]["path_reason"], s["nav"]["path_len_m"]), (None, "empty_plan", 0.0))
        s = self.core_with([(0.0, 0.0), (float("nan"), 0.0)]).compose_state(self.T)
        self.assertEqual((s["nav"]["path"], s["nav"]["path_reason"], s["nav"]["path_len_m"]), (None, "bad_plan", None))
        fpgac.dumps(s)                    # a NaN pose never reaches the wire, the STATE still goes out

    def test_positive_control_nan_length_breaks_state(self):
        # the old adapter fed the raw sum: NaN -> allow_nan=False refuses the whole STATE ...
        with self.assertRaises(ValueError):
            fpgac.dumps({"nav": {"path_len_m": float("nan")}})
        # ... since v5 (2026-10-12) compose_state nulls any non-finite float and names it
        # (tests/test_hub_v5_fixes.py), so even a raw NaN fed here no longer stops the STATE
        core = HubCore("live")
        core.feed_plan(float("nan"), self.T, path=None, path_reason="x")
        s = core.compose_state(self.T)
        fpgac.dumps(s)
        self.assertEqual((s["nav"]["path_len_m"], s["nonfinite_fields"]), (None, ["nav.path_len_m"]))

    def test_length_only_feeder(self):
        core = HubCore("live")
        core.feed_plan(3.5, self.T)
        s = core.compose_state(self.T)
        self.assertEqual((s["nav"]["path_len_m"], s["nav"]["path"], s["nav"]["path_reason"]), (3.5, None, "length_only"))

    def test_state_byte_budget(self):
        import random
        r = random.Random(3)
        pts, x, y = [], -9999.99, -9990.0
        for _ in range(600):
            x += r.uniform(-2.0, 2.0)
            y += r.uniform(-2.0, 2.0)
            pts.append((x, y))
        base = HubCore("live", hub_boot_id="hb")
        worst = self.core_with(pts, frame_id="map", stamp=1791728717.123456789)
        worst.hub_boot_id = "hb"
        a = len(fpgac.dumps(base.compose_state(self.T)))
        b = len(fpgac.dumps(worst.compose_state(self.T)))
        self.assertLessEqual(b - a, hub_core.PATH_MAX_BYTES + 256)     # points + the other path keys
        # typical room route: 64 vertices of 1-2 digit coordinates stay well under 1 KB
        room = [[round(1.23 + 0.1 * i, 2), round(-4.56 + 0.05 * i, 2)] for i in range(64)]
        self.assertLess(len(fpgac.dumps(room)), 900)


class FpgacNavPathTests(unittest.TestCase):
    def state(self, path, reason=None):
        return {"nav": {"path": path, "path_reason": reason}}

    def good(self):
        return {"frame_id": "map", "stamp_ros_s": 1.5, "points": [[0.0, 0.0], [1, 2.5]], "n_source_poses": 9,
                "length_m": 2.7, "epsilon_m": 0.025, "recv_age_ms": 12.0}

    def test_valid_and_reasons(self):
        r = fpgac.nav_path(self.state(self.good()))
        self.assertEqual((r["reason"], r["frame_id"], r["points"], r["n_source_poses"]),
                         (None, "map", [(0.0, 0.0), (1.0, 2.5)], 9))
        for reason in ("no_plan_received", "empty_plan", "bad_plan", "length_only"):
            self.assertEqual(fpgac.nav_path(self.state(None, reason)), {"reason": reason})
        self.assertEqual(fpgac.PATH_MAX_POINTS, hub_core.PATH_MAX_POINTS)

    def test_round_trip_from_hub(self):
        core = HubCore("live")
        length, path, reason = summarize_plan(zigzag(300), "map", 3.0)
        core.feed_plan(length, NS, path=path, path_reason=reason)
        wire = json.loads(fpgac.dumps(core.compose_state(NS)))
        r = fpgac.nav_path(wire)
        self.assertIsNone(r["reason"])
        self.assertEqual([list(p) for p in r["points"]], path["points"])

    def test_malformed_rejected(self):
        bad = []
        for mutate in (lambda p: p.update(points=[[0.0, 0.0]] * (fpgac.PATH_MAX_POINTS + 1)),
                       lambda p: p.update(points=[[0.0, float("nan")]]),
                       lambda p: p.update(points=[[0.0, float("inf")]]),
                       lambda p: p.update(points=[["1", 2.0]]),
                       lambda p: p.update(points=[[True, 2.0]]),
                       lambda p: p.update(points=[[1.0, 2.0, 3.0]]),
                       lambda p: p.update(points=[]),
                       lambda p: p.update(points="0,0"),
                       lambda p: p.update(frame_id=5),
                       lambda p: p.pop("frame_id")):
            p = self.good()
            mutate(p)
            bad.append(fpgac.nav_path(self.state(p)))
        bad.append(fpgac.nav_path(self.state(None, "something_else")))
        bad.append(fpgac.nav_path(self.state("not a dict")))
        bad.append(fpgac.nav_path({"nav": None}))
        self.assertEqual(bad, [{"reason": "malformed"}] * len(bad))
        # positive control: the unmutated path is accepted, so the rejections above are the checks
        self.assertIsNone(fpgac.nav_path(self.state(self.good()))["reason"])


class LegacyFrameStatsCore(HubCore):
    """The pre-10-12 hub: frame_stats.source is ignored (the producer mark never counted)."""

    def feed_frame_stats(self, text, nano_ns):
        row = json.loads(text)
        self._put("frame_stats", row, nano_ns)
        self.frame_stats.append((nano_ns, row))


def stats(i, **kw):
    row = {"kv_frame_id": i, "frame_age_ms": 100.0, "client_counts": {}}
    row.update(kw)
    return json.dumps(row)


class ProducerSourceTests(unittest.TestCase):
    def setUp(self):
        self.t = NS

    def start(self, cls=HubCore, data_source="replay", sensor="simulation"):
        core = cls(data_source, hub_boot_id="hb")
        core.feed_status(json.dumps({"event": "startup", "sensor_source": sensor}), self.t)
        return core

    def mismatch_events(self, core):
        return [e for e in core.pop_events(self.t + NS) if e["event"].get("event") == "source_mismatch"]

    def test_consistent_mark(self):
        core = self.start()
        core.feed_frame_stats(stats(1, source="replay", source_marker="m1", source_mismatch=False), self.t)
        s = core.compose_state(self.t)
        self.assertEqual((s["source"], s["geometry_nano"]["frame_source"], s["geometry_nano"]["src"]),
                         ("replay", "replay", "replay"))
        self.assertIs(s["geometry_nano"]["frame_source_mismatch"], False)
        self.assertEqual(self.mismatch_events(core), [])

    def test_missing_mark_is_no_information(self):
        core = self.start()
        core.feed_frame_stats(stats(1), self.t)
        s = core.compose_state(self.t)
        self.assertEqual((s["source"], s["geometry_nano"]["frame_source"]), ("replay", None))
        self.assertEqual(self.mismatch_events(core), [])

    def test_mismatch_inconsistent_and_one_event_per_edge(self):
        core = self.start()
        for i in range(5):
            core.feed_frame_stats(stats(i, source="live"), self.t + i * MS)
        s = core.compose_state(self.t + 10 * MS)
        self.assertEqual((s["source"], s["geometry_nano"]["frame_source"]), ("inconsistent", "live"))
        self.assertEqual(core.memory_body(self.t, force=True), None)       # no memory yet
        core.feed_memory(json.dumps({"revision": 1, "tracks": []}), self.t)
        self.assertEqual(core.memory_body(self.t, force=True)["source"], "inconsistent")
        ev = self.mismatch_events(core)
        self.assertEqual(len(ev), 1)
        self.assertEqual(ev[0]["src"], "unknown")
        self.assertEqual(ev[0]["event"], {"event": "source_mismatch", "origin": "frame_stats", "data_source": "replay",
                                          "producer_source": "live"})
        # group src stays driven by the runner's startup check (documented rule)
        self.assertEqual(s["geometry_nano"]["src"], "replay")
        # back to consistent: frame-level recovers, no event; a new mismatch edge -> a new event
        core.feed_frame_stats(stats(9, source="replay"), self.t + 20 * MS)
        self.assertEqual(core.compose_state(self.t + 21 * MS)["source"], "replay")
        self.assertEqual(self.mismatch_events(core), [])
        core.feed_frame_stats(stats(10, source="live"), self.t + 30 * MS)
        self.assertEqual(len(self.mismatch_events(core)), 1)

    def test_either_input_makes_inconsistent(self):
        core = self.start(data_source="live", sensor="simulation")     # runner disagrees
        core.feed_frame_stats(stats(1, source="live"), self.t)          # producer agrees
        self.assertEqual(core.compose_state(self.t)["source"], "inconsistent")
        core = self.start(data_source="live", sensor="hardware")       # runner agrees
        core.feed_frame_stats(stats(1, source="replay"), self.t)        # producer disagrees
        self.assertEqual(core.compose_state(self.t)["source"], "inconsistent")
        core = self.start(data_source="live", sensor="hardware")
        core.feed_frame_stats(stats(1, source="live"), self.t)
        self.assertEqual(core.compose_state(self.t)["source"], "live")

    def test_positive_control_legacy_ignores_producer_mark(self):
        legacy = self.start(cls=LegacyFrameStatsCore)
        legacy.feed_frame_stats(stats(1, source="live"), self.t)
        self.assertEqual(legacy.compose_state(self.t)["source"], "replay")        # the miss
        self.assertEqual(self.mismatch_events(legacy), [])
        new = self.start()
        new.feed_frame_stats(stats(1, source="live"), self.t)
        self.assertEqual(new.compose_state(self.t)["source"], "inconsistent")     # caught

    def test_non_dict_frame_stats_counted_not_crashing(self):
        core = self.start()
        core.feed_frame_stats("[1, 2]", self.t)
        self.assertEqual(core.counts["bad_input:frame_stats"], 1)
        fpgac.dumps(core.compose_state(self.t))

    def test_kv_ar_replay_core_passed_through(self):
        core = HubCore("live")
        tel = KvTelemetryClient(FakeKv()).poll()
        blocks = dict(tel.blocks)
        blocks["ar"] = {"mode": "replay", "age_ms": 12.0, "age_valid": True, "replay": True,
                        "replay_core": "cpu", "frame_id": 42}
        tel = tel._replace(blocks=blocks, avail=tuple(set(tel.avail or ()) | {"ar"}))
        core.feed_telemetry(tel, NS)
        ar = core.compose_state(NS)["kv"]["ar"]
        self.assertEqual((ar["avail"], ar["mode"], ar["replay"], ar["replay_core"], ar["frame_id"], ar["age_ms"]),
                         (True, "replay", True, "cpu", 42, 12.0))
        self.assertNotIn("ar_age_ms", ar)           # v5: the spec name kv.ar.age_ms only


class GridBytesTests(unittest.TestCase):
    def test_same_bytes_as_the_per_cell_generator(self):
        import array
        old = lambda data: bytes((v + 256) % 256 for v in data)          # noqa: E731 (pre-10-12)
        values = list(range(-128, 128)) * 25 + [0, -1, 100, 127, -128]
        for data in (values, array.array("b", values)):
            msg = types.SimpleNamespace(data=data)
            self.assertEqual(hub_ros.grid_bytes(msg), old(data))
        self.assertEqual(hub_ros.grid_bytes(types.SimpleNamespace(data=[-1, 0, 100])), b"\xff\x00\x64")
        # positive control: a plain bytes() without two's complement would refuse -1
        with self.assertRaises(ValueError):
            bytes([-1])


class PlanCallbackTests(unittest.TestCase):
    def test_plan_polyline_computed_outside_the_lock(self):
        subs = {}
        node_kw, sub_kw, pub_kw = {}, {}, {}

        class Node(object):
            def __init__(self, name, **kw):
                node_kw.update(kw)

            def create_subscription(self, kind, topic, cb, qos, **kw):
                subs[topic] = cb
                sub_kw[topic] = kw

            def create_publisher(self, kind, topic, qos, **kw):
                pub_kw[topic] = kw
                return types.SimpleNamespace(publish=lambda m: None)

        P = types.SimpleNamespace
        m = P(Node=Node, QoSProfile=lambda **kw: kw, DurabilityPolicy=P(TRANSIENT_LOCAL=1),
              ReliabilityPolicy=P(RELIABLE=1, BEST_EFFORT=2), String=object, Empty=object, Bool=object, Twist=object,
              PoseWithCovarianceStamped=object, OccupancyGrid=object, Odometry=object, Path=object,
              CompressedImage=object, SubscriptionEventCallbacks=lambda **kw: kw,
              PublisherEventCallbacks=lambda **kw: kw)
        core = HubCore("live")
        server = P(core=core, lock=threading.RLock())
        held = []
        real = hub_ros.summarize_plan

        def spy(*a):
            held.append(server.lock._is_owned())
            return real(*a)

        hub_ros.summarize_plan = spy
        try:
            hub_ros.build_node(server, m, clock=lambda: 3 * NS)
            poses = [P(pose=P(position=P(x=x, y=y))) for x, y in line(300, dx=0.01)]
            subs["/plan"](P(header=P(frame_id="map", stamp=P(sec=4, nanosec=250000000)), poses=poses))
        finally:
            hub_ros.summarize_plan = real
        self.assertEqual(held, [False])
        s = core.compose_state(3 * NS + 5 * MS)
        self.assertEqual((s["nav"]["path"]["points"], s["nav"]["path"]["stamp_ros_s"], s["nav"]["path"]["recv_age_ms"]),
                         ([[0.0, 0.0], [2.99, 0.0]], 4.25, 5.0))
        self.assertEqual(core.counts["cb:/plan"], 1)


if __name__ == "__main__":
    unittest.main()
