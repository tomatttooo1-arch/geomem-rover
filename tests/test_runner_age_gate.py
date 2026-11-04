"""Task runner 200 ms geometry age gate (host/geometry_age_gate.py, host/ros_runtime.py).

Docs: docs/design/RUNNER_AGE_GATE_20261011.md.

1. Rule parity with the producer: limit == geo_producer.MAX_FRAME_AGE_NS == M7Pipeline's
   default max_frame_age_ns == rolling_nav_layer.MAX_MAP_AGE_NS; the verdict at 199 / 200 /
   200.000001 / 201 ms and at -1 ns equals M7Pipeline._age's (the producer's comparator):
   200 ms exactly passes, one ns more is stale, negative is future; another clock id is
   never subtracted; a missing origin is refused (geometry_age_unknown, 2026-10-12) -- the
   gate module of 07eccb8 (before the change) let it pass, checked as the old-behaviour control.
2. ros_runtime on the M5 software replay (9 scenarios), every source:
   * cpu / kv / kv_topic with a 584 ms old origin (the v2 board maximum): every geometry is
     refused (geometry_stale events with the age, {} to update_memory) -> the stream differs
     from the ungated one in scenario_occlusion_changes_view; the integration HEAD
     (16f550b, no gate) fed the SAME fixture uses the 584 ms geometry (kv_topic result
     "fused", frame_age_ms >= 584) -- the old behaviour this gate removes;
   * fresh origins: stream equals the cpu baseline after removing the kv rows (kv/kv_topic)
     or the cpu summary row; age_gate summary used == frames;
   * expired during processing (kv / kv_topic): fresh at the lookup / KV answer (150 ms),
     the clock moves 100 ms before the use -> refused at the use point;
   * no origin (cpu frame without the arrival stamp, kv_topic body without acquired_raw_ns):
     every geometry refused as geometry_age_unknown; the same fixture with the 07eccb8 gate
     module swapped in uses every geometry (the old "pass and count origin_missing").
3. Production origin path (rtabmap probe world, real on_images -> prepare_images): the frame
   carries the RAW arrival; a 300 ms delay between arrival and use is refused, a 50 ms one
   passes.
4. hub: geometry_stale / geometry_age_unknown -> STATE task.geometry_age_refused / geometry_age_last_refused, and the
   row is still forwarded as an EVENT.
Python 3.8+.
"""

import json
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from host import geometry_age_gate as gag  # noqa: E402
from host.fpga_clock import CLOCK_ID, now_ns  # noqa: E402

MS = 1000000
OLD = "16f550b"
OLD_GATE = "07eccb8"        # gate that let a missing origin pass (before geometry_age_unknown)
V2_MAX_NS = 584 * MS        # v2 board run: largest age the runner used


def old_source():
    try:
        return subprocess.check_output(["git", "show", "%s:host/ros_runtime.py" % OLD], cwd=str(ROOT),
                                       stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None


def old_gate_module():
    """host/geometry_age_gate.py as of OLD_GATE (missing origin passed), or None."""
    try:
        text = subprocess.check_output(["git", "show", "%s:host/geometry_age_gate.py" % OLD_GATE], cwd=str(ROOT),
                                       stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None
    module = types.ModuleType("host.geometry_age_gate")
    module.__file__ = "geometry_age_gate@" + OLD_GATE
    exec(compile(text, module.__file__, "exec"), module.__dict__)
    return module


# ---------------------------------------------------------------- 1. rule parity
class RuleParity(unittest.TestCase):
    def producer_age(self, age_ns):
        """M7Pipeline._age on the same numbers (the producer's comparator)."""
        from host.m7_pipeline import M7Pipeline
        ident = types.SimpleNamespace(clock_id=CLOCK_ID, acquired_ns=10 ** 12)
        _, reason = M7Pipeline._age(types.SimpleNamespace(future_tolerance_ns=0), ident,
                                    10 ** 12 + age_ns, CLOCK_ID, 200 * MS)
        return reason

    def test_limit_is_the_producers(self):
        import inspect
        from host.geo_producer import MAX_FRAME_AGE_NS
        from host.m7_pipeline import M7Pipeline
        from host.rolling_nav_layer import MAX_MAP_AGE_NS
        default = inspect.signature(M7Pipeline.__init__).parameters["max_frame_age_ns"].default
        self.assertEqual(gag.MAX_GEOMETRY_AGE_NS, 200 * MS)
        self.assertEqual({gag.MAX_GEOMETRY_AGE_NS, MAX_FRAME_AGE_NS, default, MAX_MAP_AGE_NS}, {200 * MS})
        gag.GeometryAgeGate(500 * MS)              # 10-26: the runner's own limit (RUNNER_MAX_GEOMETRY_AGE_NS)
        with self.assertRaises(ValueError):
            gag.GeometryAgeGate(500 * MS + 1)

    def test_boundaries_match_producer(self):
        origin = 10 ** 12
        cases = [(199 * MS, None, None), (200 * MS, None, None), (200 * MS + 1, gag.STALE, "expired_frame"),
                 (201 * MS, gag.STALE, "expired_frame"), (0, None, None), (-1, gag.FUTURE, "future_frame")]
        for age, ours, producer in cases:
            got_age, reason = gag.age_verdict(origin, origin + age)
            self.assertEqual((got_age, reason), (age, ours), age)
            self.assertEqual(self.producer_age(age), producer, age)

    def test_clock_and_missing_origin(self):
        self.assertEqual(gag.age_verdict(1, 2, origin_clock_id="mono"), (None, gag.CLOCK_MISMATCH))
        self.assertEqual(gag.age_verdict(1, 2, origin_clock_id=CLOCK_ID), (1, None))
        self.assertEqual(gag.age_verdict(None, 2), (None, gag.AGE_UNKNOWN))
        self.assertEqual(gag.AGE_UNKNOWN, "geometry_age_unknown")
        g = gag.GeometryAgeGate()
        g.check(None, 5)
        g.check(0, 201 * MS)
        g.check(0, 200 * MS)
        g.check(0, 1, origin_clock_id="mono")
        s = g.summary()
        self.assertEqual(s["counts"], {"checked": 4, "used": 1, "refused:geometry_stale": 1,
                                       "refused:geometry_clock_mismatch": 1, "refused:geometry_age_unknown": 1})
        self.assertEqual((s["max_age_ms"], s["max_used_age_ms"], g.refused), (201.0, 200.0, 3))

    def test_old_gate_let_missing_origin_pass(self):
        old = old_gate_module()
        if old is None:
            self.skipTest("git history not available")
        self.assertEqual(old.age_verdict(None, 2), (None, None))          # the old behaviour
        g = old.GeometryAgeGate()
        g.check(None, 5)
        self.assertEqual(dict(g.counts), {"checked": 1, "origin_missing": 1, "used": 1})


# ---------------------------------------------------------------- 2. runtime on the M5 replay
def capture_aged(source, name, extra, *, source_kind, origin_age_ns, use_delay_ns=0):
    """M5 replay with an origin stamp on every frame that reaches the detections queue.
    cpu/kv: the runner arrival (frame.geometry_origin_ns / frame.kv_rx_ns) = now - age;
    kv_topic: the emulated producer body's acquired_raw_ns = now - age (published before the
    detections, like the producer). use_delay_ns: the node's RAW clock jumps by this much right
    after the geometry step (kv_geometry / topic_geometry return), i.e. between the lookup /
    KV answer and update_memory. origin_age_ns None: no origin at all (cpu/kv: the frame's
    stamps are None; kv_topic: the body has no acquired_raw_ns)."""
    import test_geo_producer as tgp
    import test_region_source_none_regression as reg
    from host.m7_object_native import NativeObjectEvidence, compile_object_native
    if tgp._ROI is None and source_kind == "kv_topic":
        tgp._ROI = NativeObjectEvidence(compile_object_native(str(Path(tempfile.mkdtemp()) / "o.so")))
    original = reg.make_loader

    def make_loader(src, module, extra_args=None):
        load = original(src, module, extra_args)

        def load_runtime(world, path):
            node = load(world, path)
            skew = [0]
            node.kv_now_ns = lambda: now_ns() + skew[0]
            state = {"n": 0}

            class Inject(queue.Queue):
                def put(self, item, *a, **k):
                    frame, _ = item
                    state["n"] += 1
                    origin = None if origin_age_ns is None else node.kv_now_ns() - origin_age_ns
                    if source_kind == "kv_topic":
                        body = json.loads(tgp.emulated_body(node, frame, frame.depth, state["n"]))
                        if origin is None:
                            body["producer"].pop("acquired_raw_ns", None)
                        else:
                            body["producer"]["acquired_raw_ns"] = origin
                        node.on_geometry_frame(types.SimpleNamespace(data=json.dumps(body)))
                    elif source_kind == "kv":
                        frame.kv_rx_ns = origin
                        if origin is None:
                            frame.geometry_origin_ns = None
                    else:
                        frame.geometry_origin_ns = origin
                    return queue.Queue.put(self, item, *a, **k)
            node.results = Inject(maxsize=1)
            if use_delay_ns:
                for meth in ("kv_geometry", "topic_geometry"):
                    inner = getattr(node, meth, None)
                    if inner is None:
                        continue

                    def delayed(frame, _inner=inner):
                        out = _inner(frame)
                        skew[0] += use_delay_ns
                        return out
                    setattr(node, meth, delayed)
            return node
        return load_runtime
    with patch.object(reg, "make_loader", make_loader):
        return reg.capture(source, name, extra)


def topic_args(**over):
    a = dict(geometry_source="kv_topic", geometry_fallback="none", kv_server=None, kv_expect_build_id=None,
             kv_depth_unit_m=None, kv_grid=None, kv_shadow_check=False, region_source="none",
             max_region_age_s=1.0, geometry_topic_tolerance_s=0.05, geometry_topic_wait_s=0.0)
    a.update(over)
    return a


AGE_EVENTS = ("geometry_stale", "geometry_future", "geometry_clock_mismatch", "geometry_age_unknown")


class RuntimeGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import test_car_geometry_kv as cg
        cls.cg = cg
        if cg.baseline_source() is None or old_source() is None:
            raise unittest.SkipTest("git history not available")
        cls.src = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        cls.old = old_source()
        cls.before, _ = cg.capture(cg.baseline_source(), "rag_cpu_baseline")
        cls.build = tempfile.mkdtemp(prefix="rag_kv_")

    def rows(self, streams, kind):
        return self.cg.status_rows(streams, kind)

    def gate_summary(self, streams):
        closes = self.rows(streams, "geometry_close")
        self.assertEqual(len(closes), len(streams))
        tot = {}
        for c in closes:
            for k, v in c["age_gate"]["counts"].items():
                tot[k] = tot.get(k, 0) + v
        return tot

    def assert_stale_everywhere(self, streams, ok, source_kind):
        # scenario self-checks may fail here (no geometry -> occlusion is not seen); not asserted
        stale = self.rows(streams, "geometry_stale")
        processed = self.rows(streams, "frame_processed")
        self.assertGreater(len(processed), 10)
        self.assertEqual(len(stale), len(processed))
        self.assertTrue(all(r["age_ms"] >= 584 and r["limit_ms"] == 200 and r["source"] == source_kind
                            for r in stale))
        for v in streams.values():          # per runtime: refused_total counts 1, 2, 3, ...
            totals = [r["payload"]["refused_total"] for r in v if r.get("event") == "publish"
                      and r.get("topic") == "/fpga/status" and r["payload"].get("event") == "geometry_stale"]
            self.assertEqual(totals, list(range(1, len(totals) + 1)))
        tot = self.gate_summary(streams)
        self.assertEqual(tot.get("refused:geometry_stale"), len(processed))
        self.assertEqual(tot.get("used", 0), 0)
        self.assertIn("scenario_occlusion_changes_view", self.cg.diff_names(self.before, streams, self.cg.kv_normalise))

    def assert_fresh(self, streams, ok):
        self.assertTrue(all(o for o, _ in ok.values()), ok)
        self.assertEqual(self.cg.diff_names(self.before, streams, self.cg.kv_normalise), [])
        for kind in AGE_EVENTS:
            self.assertEqual(self.rows(streams, kind), [])
        tot = self.gate_summary(streams)
        n = len(self.rows(streams, "frame_processed"))
        self.assertEqual((tot.get("used"), tot.get("checked")), (n, n))
        self.assertEqual([k for k in tot if k.startswith("refused:") or k == "origin_missing"], [])

    # cpu ----------------------------------------------------------------
    def test_cpu_584ms_refused_and_old_runtime_used_it(self):
        streams, ok = capture_aged(self.src, "rag_cpu_584", {"geometry_source": "cpu"}, source_kind="cpu",
                                   origin_age_ns=V2_MAX_NS)
        self.assert_stale_everywhere(streams, ok, "cpu")
        old, _ = capture_aged(self.old, "rag_cpu_584_old", {"geometry_source": "cpu"}, source_kind="cpu",
                              origin_age_ns=V2_MAX_NS)
        self.assertEqual(self.cg.diff_names(self.before, old, self.cg.strict), [])     # used the 584 ms geometry
        self.assertEqual(self.rows(old, "geometry_stale"), [])

    def test_cpu_fresh_and_boundary(self):
        streams, ok = capture_aged(self.src, "rag_cpu_fresh", {"geometry_source": "cpu"}, source_kind="cpu",
                                   origin_age_ns=0)
        self.assert_fresh(streams, ok)
        self.assertEqual(self.cg.diff_names(self.before, streams, self.cg.strict), [])
        # 150 ms passes, 250 ms is refused (the 199/200/201 edges are in RuleParity: the
        # replay's own processing time adds a few ms, so exact edges are tested on the rule)
        s150, _ = capture_aged(self.src, "rag_cpu_150", {"geometry_source": "cpu"}, source_kind="cpu",
                               origin_age_ns=150 * MS)
        self.assertEqual(self.rows(s150, "geometry_stale"), [])
        s250, _ = capture_aged(self.src, "rag_cpu_250", {"geometry_source": "cpu"}, source_kind="cpu",
                               origin_age_ns=250 * MS)
        self.assertEqual(len(self.rows(s250, "geometry_stale")), len(self.rows(s250, "frame_processed")))

    def test_cpu_plain_replay_has_origin(self):
        # the plain M5 replay stamps the arrival at its inference boundary (2026-10-12): aged, used
        streams, ok = self.cg.capture(self.src, "rag_cpu_plain", {"geometry_source": "cpu"})
        self.assert_fresh(streams, ok)
        self.assertEqual(self.cg.diff_names(self.before, streams, self.cg.strict), [])

    def assert_unknown_everywhere(self, streams, source_kind):
        unknown = self.rows(streams, "geometry_age_unknown")
        processed = self.rows(streams, "frame_processed")
        self.assertGreater(len(processed), 10)
        self.assertEqual(len(unknown), len(processed))
        self.assertTrue(all(r["age_ms"] is None and r["reason"] == "geometry_age_unknown"
                            and r["source"] == source_kind and r["limit_ms"] == 200 for r in unknown))
        for v in streams.values():
            totals = [r["payload"]["refused_total"] for r in v if r.get("event") == "publish"
                      and r.get("topic") == "/fpga/status" and r["payload"].get("event") == "geometry_age_unknown"]
            self.assertEqual(totals, list(range(1, len(totals) + 1)))
        tot = self.gate_summary(streams)
        self.assertEqual((tot.get("refused:geometry_age_unknown"), tot.get("used", 0)), (len(processed), 0))
        self.assertIn("scenario_occlusion_changes_view", self.cg.diff_names(self.before, streams, self.cg.kv_normalise))

    def capture_old_gate(self, name, extra, source_kind):
        old = old_gate_module()
        if old is None:
            self.skipTest("git history not available")
        # swap only this one entry (patch.dict would also drop every module first imported inside)
        current = sys.modules["host.geometry_age_gate"]
        sys.modules["host.geometry_age_gate"] = old
        try:
            return capture_aged(self.src, name, extra, source_kind=source_kind, origin_age_ns=None)
        finally:
            sys.modules["host.geometry_age_gate"] = current

    def test_cpu_no_origin_refused_old_gate_used_it(self):
        streams, _ = capture_aged(self.src, "rag_cpu_noorigin", {"geometry_source": "cpu"}, source_kind="cpu",
                                  origin_age_ns=None)
        self.assert_unknown_everywhere(streams, "cpu")
        self.assertTrue(all(r["age_basis"] == "runner_arrival" for r in self.rows(streams, "geometry_age_unknown")))
        # old behaviour (07eccb8 gate, same runtime, same fixture): passed and counted origin_missing
        old, _ = self.capture_old_gate("rag_cpu_noorigin_oldgate", {"geometry_source": "cpu"}, "cpu")
        tot = self.gate_summary(old)
        n = len(self.rows(old, "frame_processed"))
        self.assertEqual((tot.get("used"), tot.get("origin_missing")), (n, n))
        self.assertEqual(self.rows(old, "geometry_age_unknown"), [])
        self.assertEqual(self.cg.diff_names(self.before, old, self.cg.strict), [])

    def test_cpu_kv_only_region_not_gated(self):
        streams, _ = capture_aged(self.src, "rag_cpu_kvonly", {"geometry_source": "cpu", "region_source": "kv_only"},
                                  source_kind="cpu", origin_age_ns=V2_MAX_NS)
        self.assertEqual(self.rows(streams, "geometry_stale"), [])
        self.assertEqual(self.gate_summary(streams), {})

    # kv_topic ------------------------------------------------------------
    def test_kv_topic_584ms_refused_and_old_runtime_used_it(self):
        streams, ok = capture_aged(self.src, "rag_topic_584", topic_args(), source_kind="kv_topic",
                                   origin_age_ns=V2_MAX_NS)
        self.assert_stale_everywhere(streams, ok, "kv_topic")
        processed = self.rows(streams, "frame_processed")
        self.assertTrue(all(p["geometry"]["result"] == "unavailable:geometry_stale"
                            and p["geometry"]["age_gate"] == "refused"
                            and p["geometry"]["geometry_age_ms"] >= 584 for p in processed))
        self.assertTrue(all(r["age_basis"] == "producer_acquired" for r in self.rows(streams, "geometry_stale")))
        old, _ = capture_aged(self.old, "rag_topic_584_old", topic_args(), source_kind="kv_topic",
                              origin_age_ns=V2_MAX_NS)
        old_p = self.rows(old, "frame_processed")
        self.assertGreater(len(old_p), 10)
        # the old runtime fused every 584 ms answer: the stream equals the fresh cpu one
        self.assertTrue(all(p["geometry"]["result"] == "fused" and p["geometry"]["frame_age_ms"] >= 584
                            for p in old_p))
        self.assertEqual(self.cg.diff_names(self.before, old, self.cg.kv_normalise), [])

    def test_kv_topic_fresh_equal(self):
        streams, ok = capture_aged(self.src, "rag_topic_fresh", topic_args(), source_kind="kv_topic",
                                   origin_age_ns=0)
        self.assert_fresh(streams, ok)
        self.assertTrue(all(p["geometry"]["age_gate"] == "pass" and p["geometry"]["geometry_age_ms"] < 200
                            for p in self.rows(streams, "frame_processed")))

    def test_kv_topic_expires_while_processing(self):
        streams, _ = capture_aged(self.src, "rag_topic_late", topic_args(), source_kind="kv_topic",
                                  origin_age_ns=150 * MS, use_delay_ns=100 * MS)
        processed = self.rows(streams, "frame_processed")
        self.assertGreater(len(processed), 10)
        for p in processed:
            g = p["geometry"]
            self.assertEqual(g["age_gate"], "refused")
            self.assertGreaterEqual(g["geometry_age_ms"], 250)
            self.assertEqual(len(self.rows(streams, "geometry_stale")), len(processed))
        # at the lookup it was fresh: the cache did not refuse it
        closes = [c for c in self.rows(streams, "geometry_close") if c.get("source") == "kv_topic"]
        self.assertEqual(sum(c["counts"].get("used:exact", 0) for c in closes), len(processed))

    def test_kv_topic_no_acquired_refused_old_gate_used_it(self):
        streams, _ = capture_aged(self.src, "rag_topic_noorigin", topic_args(), source_kind="kv_topic",
                                  origin_age_ns=None)
        self.assert_unknown_everywhere(streams, "kv_topic")
        processed = self.rows(streams, "frame_processed")
        self.assertTrue(all(p["geometry"]["result"] == "unavailable:geometry_age_unknown"
                            and p["geometry"]["age_gate"] == "refused" and p["geometry"]["geometry_age_ms"] is None
                            for p in processed))
        self.assertTrue(all(r["age_basis"] == "producer_acquired" for r in self.rows(streams, "geometry_age_unknown")))
        old, _ = self.capture_old_gate("rag_topic_noorigin_oldgate", topic_args(), "kv_topic")
        old_p = self.rows(old, "frame_processed")
        # (the age_gate word is the current runtime's: the old gate returned no reason -> "pass", no age)
        self.assertTrue(old_p and all(p["geometry"]["result"] == "fused" and p["geometry"]["geometry_age_ms"] is None
                                      for p in old_p))
        self.assertEqual(self.cg.diff_names(self.before, old, self.cg.kv_normalise), [])

    def test_kv_topic_foreign_clock_refused(self):
        import test_geo_producer as tgp
        orig = tgp.emulated_body

        def mono_body(node, frame, depth, n):
            body = json.loads(orig(node, frame, depth, n))
            body["producer"]["clock_id"] = "mono"
            return json.dumps(body)
        with patch.object(tgp, "emulated_body", mono_body):
            streams, _ = capture_aged(self.src, "rag_topic_mono", topic_args(), source_kind="kv_topic",
                                      origin_age_ns=0)
        rows = self.rows(streams, "geometry_clock_mismatch")
        self.assertEqual(len(rows), len(self.rows(streams, "frame_processed")))

    # kv ------------------------------------------------------------------
    def kv(self, name, origin_age_ns, use_delay_ns=0, src=None):
        fake = self.cg.FakeKv(self.build)
        try:
            return capture_aged(src or self.src, name, self.cg.kv_args(fake.port, kv_shadow_check=False),
                                source_kind="kv", origin_age_ns=origin_age_ns, use_delay_ns=use_delay_ns)
        finally:
            fake.stop()

    def test_kv_584ms_refused_and_old_runtime_used_it(self):
        streams, ok = self.kv("rag_kv_584", V2_MAX_NS)
        self.assert_stale_everywhere(streams, ok, "kv")
        stats = [json.loads(r["payload"]["data"]) for v in streams.values() for r in v
                 if r.get("event") == "publish" and r.get("topic") == "/fpga/kv_link/frame_stats"]
        self.assertTrue(stats and all(s["result"] == "unavailable:geometry_stale" and s["age_gate"] == "refused"
                                      for s in stats))
        old, _ = self.kv("rag_kv_584_old", V2_MAX_NS, src=self.old)
        old_p = self.rows(old, "frame_processed")
        self.assertTrue(old_p and all(p["geometry"]["result"] == "fused" and p["geometry"]["frame_age_ms"] >= 584
                                      for p in old_p))
        self.assertEqual(self.cg.diff_names(self.before, old, self.cg.kv_normalise), [])

    def test_kv_fresh_and_expires_while_processing(self):
        streams, ok = self.kv("rag_kv_fresh", 0)
        self.assert_fresh(streams, ok)
        late, _ = self.kv("rag_kv_late", 150 * MS, use_delay_ns=100 * MS)
        processed = self.rows(late, "frame_processed")
        self.assertTrue(processed and all(p["geometry"]["age_gate"] == "refused" for p in processed))
        self.assertEqual(len(self.rows(late, "geometry_stale")), len(processed))


# ---------------------------------------------------------------- 3. production origin path
class ProductionOrigin(unittest.TestCase):
    def session(self, name, delay_ns):
        # the probe's own loader needs ast.unparse (3.9+, D-028); the regression test's loader
        # (same Runtime class text, same probe bindings) runs on 3.8 too
        from tools import rtabmap_runtime_behavior_probe as probe
        import test_region_source_none_regression as reg
        src = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        module = reg.module_from_source(src, "rag_probe_runtime_" + name)
        self.addCleanup(sys.modules.pop, "rag_probe_runtime_" + name, None)
        with patch.object(probe, "load_runtime", reg.make_loader(src, module)):
            s = probe.Session(tempfile.mkdtemp(prefix="rag_probe_"), name, "original")
        n = s.n
        skew = [0]
        # frozen RAW clock: the age is exactly the injected delay (a real clock adds the fixture's
        # own run time, which on the shared host has reached ~600 ms once in ~10 runs: a flake)
        base = now_ns()
        n.kv_now_ns = lambda: base + skew[0]
        orig_finish = s.w.finish_detection

        def finish():
            skew[0] += delay_ns          # recognition took this long after on_images
            return orig_finish()
        s.w.finish_detection = finish
        s.warm()
        return s

    def test_arrival_stamp_reaches_the_gate(self):
        s = self.session("fresh", 50 * MS)
        self.assertIsInstance(s.n.frame.geometry_origin_ns, int)
        self.assertEqual(s.n.age_gate.counts["used"], 1, s.n.age_gate.summary())
        self.assertEqual(s.n.age_gate.counts.get("refused:geometry_age_unknown", 0), 0)
        self.assertEqual(s.n.age_gate.max_used_age_ns, 50 * MS)
        s.finish()

    def test_delay_after_arrival_is_refused(self):
        s = self.session("late", 300 * MS)
        rows = [e["payload"] for e in s.w.events if e.get("event") == "publish"
                and e.get("topic") == "/fpga/status" and e["payload"].get("event") == "geometry_stale"]
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["age_ms"], 300.0)
        self.assertEqual(rows[0]["age_basis"], "runner_arrival")
        self.assertEqual(s.n.processed, 1)       # the frame is still processed, without geometry
        s.finish()


# ---------------------------------------------------------------- 4. hub
class HubVisibility(unittest.TestCase):
    def test_state_and_event(self):
        from host.hub.hub_core import HubCore
        core = HubCore("replay")
        core.feed_status(json.dumps({"event": "geometry_stale", "frame_id": "rgb-7", "reason": "geometry_stale",
                                     "age_ms": 584.2, "refused_total": 3, "source": "kv_topic"}), 10 * MS)
        s = core.compose_state(20 * MS)
        self.assertEqual(s["task"]["geometry_age_refused"], 3)
        self.assertEqual(s["task"]["geometry_age_last_refused"],
                         {"frame_id": "rgb-7", "reason": "geometry_stale", "age_ms": 584.2})
        self.assertEqual([b["event"] for _, _, b in core.events], ["geometry_stale"])
        core.feed_status(json.dumps({"event": "geometry_age_unknown", "frame_id": "rgb-8",
                                     "reason": "geometry_age_unknown", "age_ms": None, "refused_total": 4,
                                     "source": "cpu"}), 30 * MS)
        s = core.compose_state(40 * MS)
        self.assertEqual(s["task"]["geometry_age_refused"], 4)
        self.assertEqual(s["task"]["geometry_age_last_refused"],
                         {"frame_id": "rgb-8", "reason": "geometry_age_unknown", "age_ms": None})
        self.assertEqual([b["event"] for _, _, b in core.events], ["geometry_stale", "geometry_age_unknown"])
        fresh = HubCore("replay").compose_state(1)
        self.assertIsNone(fresh["task"]["geometry_age_refused"])


if __name__ == "__main__":
    unittest.main()
