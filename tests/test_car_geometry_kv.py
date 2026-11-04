"""ros_runtime --geometry-source cpu|kv (docs/design/CAR_GEOMETRY_KV_RESULTS_20261011.md).

1. Default cpu is unchanged: the M5 software replay (9 scenarios) with the Runtime from
   4b96ade (before this change) and with today's file give identical event streams (only the
   ``loader`` row, which carries source hashes, is removed). Positive control: a variant whose
   cpu branch hands update_memory an empty geometry differs.
2. kv mode on the same replay against an in-process kv_server (g++ frame core + E2 C ROI
   core, loopback): every scenario passes and the stream equals the cpu stream once the rows
   kv mode adds on purpose are removed (geometry_* events, /fpga/kv_link/frame_stats, the
   ``geometry`` field of frame_processed); the shadow check counts every frame equal.
   Positive control: a server whose ROI core reads the previous request's depth differs.
3. KV unavailable: fallback none -> geometry_unavailable event with the reason, update_memory
   gets {} (stream differs from cpu); fallback cpu -> every fallback frame recorded and the
   remaining stream equals cpu.
4. KvGeometry refusals (no server, roi core none, BUILD_ID mismatch / match, depth not
   representable, crc mismatch on wrong codes, reconnect rate limit) and byte identity of the
   request without ``codes``.
5. Known answer: ros_runtime's 16UC1 conversion equals kv_link.codes_to_m bit for bit for all
   65536 codes (units 0.001 and 0.0002) under the running NumPy.
Python 3.8+ (the loader comes from test_region_source_none_regression, which avoids ast.unparse).
"""

import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from test_region_source_none_regression import capture  # noqa: E402

BASELINE = "4b96ade"
KV_EVENTS = ("geometry_source", "geometry_available", "geometry_unavailable", "geometry_close",
             "geometry_fallback_cpu", "geometry_shadow_mismatch",
             "geometry_stale", "geometry_future", "geometry_clock_mismatch", "geometry_age_unknown")
# 200 ms age gate, then goes to update_memory (one call for all three sources)
CPU_CALL = '''                        geometry = ({} if self.region_source == "kv_only"
                                    else compute_geometry(frame, self.memory))'''


def baseline_source():
    try:
        return subprocess.check_output(["git", "show", "%s:host/ros_runtime.py" % BASELINE], cwd=str(ROOT),
                                       stderr=subprocess.DEVNULL).decode("utf-8")
    except (OSError, subprocess.CalledProcessError):
        return None


BOOT_ID = re.compile(r'(runtime_boot_id\\": \\")[0-9a-f]+')


def _text(row):
    """Row as JSON with the per-process random /fpga/memory runtime_boot_id masked."""
    return BOOT_ID.sub(r"\1<boot>", json.dumps(row, sort_keys=True, ensure_ascii=False, default=repr))


def is_cpu_gate_close(r):
    """The runner summary row the 200 ms age gate added for the cpu source at close
    (feat/dev1/runner-age-gate-20261011); its content is checked in test_runner_age_gate."""
    payload = r.get("payload") or {}
    return (r.get("event") == "publish" and r.get("topic") == "/fpga/status"
            and payload.get("event") == "geometry_close" and payload.get("source") == "cpu")


def is_runtime_info(r):
    """The latched /fpga/runtime_info start-up publish (integration v5, 2026-10-12); its body is
    checked in test_hub_v5_fixes. A new row at start-up, so it moves every later sequence by one."""
    return r.get("event") == "publish" and r.get("topic") == "/fpga/runtime_info"


# 10-25 find-retry (host/task_control.py retry, host/ros_runtime.py): the startup row records the effective budgets
# and retry options, task rows carry goal + attempts, a failed navigation_result its reason, and a failed attempt adds
# its own rows. The baseline runtime predates all of them; they are compared in test_task_control /
# test_find_nav_timeouts, here removed so the rest of each stream is still compared byte for byte.
RETRY_STARTUP_KEYS = ("navigation_timeout_s", "task_timeout_s", "nav_retries_same_goal", "max_nav_failures",
                      "stream_timeout_s",
                      "min_attempt_time_s", "retry_pause_s", "fault_nav_fail_first")
RETRY_ROWS = ("navigation_attempt_failed", "costmap_clear_skipped", "costmap_clear_requested", "costmap_cleared")


# 10-26 find-near (host/task_control.py en-route verdict + final approach; tests/test_find_near_20261026.py): a found
# task adds the approach command and its outcome rows, task rows may carry en_route / approach. Removed here likewise.
FIND_NEAR_ROWS = ("approach_started", "approach_result")


def retry_normalise(r):
    """None for a row the retry (or find-near) added, else the row without the keys they added."""
    if r.get("event") != "publish" or r.get("topic") != "/fpga/status":
        return r
    p = r.get("payload") or {}
    kind = p.get("event")
    if kind in RETRY_ROWS or (kind == "navigation_retired" and p.get("for_retry")):
        return None
    if kind in FIND_NEAR_ROWS or (kind == "command" and (p.get("command") or {}).get("kind") == "approach"):
        return None
    drop = {"startup": RETRY_STARTUP_KEYS, "task": ("goal", "attempts", "en_route", "approach"),
            "navigation_result": ("reason",)}.get(kind)
    return dict(r, payload={k: v for k, v in p.items() if k not in drop}) if drop else r


def strict(events):
    """Only the loader row (source hashes), the cpu age-gate summary row at close and the v5
    /fpga/runtime_info start-up row are removed and the random boot id masked; sequence numbers
    stay, except that rows after a removed runtime_info row are counted as if it was not there
    (it is published once at start-up; the summary row is the last row, so it moves nothing)."""
    out, shift = [], 0
    for r in events:
        if retry_normalise(r) is None:
            shift += 1
            continue
        r = retry_normalise(r)
        if is_runtime_info(r):
            shift += 1
            continue
        if r.get("event") == "loader" or is_cpu_gate_close(r):
            continue
        if shift and "sequence" in r:
            r = dict(r, sequence=r["sequence"] - shift)
        out.append(_text(r))
    return out


def kv_normalise(events):
    out = []
    for r in events:
        r = retry_normalise(r)
        if r is None or r.get("event") == "loader" or is_runtime_info(r):
            continue
        if r.get("event") == "publish" and r.get("topic") == "/fpga/kv_link/frame_stats":
            continue
        if r.get("event") == "publish" and r.get("topic") == "/fpga/status":
            payload = r.get("payload") or {}
            if payload.get("event") in KV_EVENTS:
                continue
            if payload.get("event") == "frame_processed" and "geometry" in payload:
                r = dict(r, payload={k: v for k, v in payload.items() if k != "geometry"})
        r = dict(r)
        r.pop("sequence", None)
        out.append(_text(r))
    return out


def diff_names(a, b, norm):
    return sorted(n for n in set(a) | set(b) if norm(a.get(n, [])) != norm(b.get(n, [])))


def status_rows(streams, kind):
    return [r["payload"] for v in streams.values() for r in v
            if r.get("event") == "publish" and r.get("topic") == "/fpga/status"
            and (r.get("payload") or {}).get("event") == kind]


class FakeKv(object):
    """In-process kv_server: g++ frame core + E2 C ROI core (optionally wrapped)."""

    def __init__(self, build, wrap=None, roi=True, regs=None):
        from host.kv_link import CpuRoiCore, KvLinkServer
        from host.m7_frame_native import FrameNativeM7, compile_frame_native
        from host.m7_object_native import compile_object_native
        frame = Path(build) / "frame.so"
        obj = Path(build) / "object.so"
        if not frame.exists():
            compile_frame_native(str(frame))
        if not obj.exists():
            compile_object_native(str(obj))
        core = CpuRoiCore(str(obj)) if roi else None
        if core is not None and wrap is not None:
            core = wrap(core)
        self.server = KvLinkServer(FrameNativeM7(str(frame)).lib.fpga_m7_depth_frame, "cpu",
                                   bind=("127.0.0.1", 0), roi_core=core, regs=regs)
        self.server.start()
        self.port = self.server.port

    def stop(self):
        self.server.stop()


def kv_args(port, **over):
    a = dict(geometry_source="kv", geometry_fallback="none", kv_server="127.0.0.1:%d" % port,
             kv_timeout_s=2.0, kv_hello_timeout_s=2.0, kv_retry_s=0.0, kv_expect_build_id=None,
             kv_depth_unit_m=2.0 ** -10, kv_grid=None, kv_shadow_check=True, region_source="none",
             max_region_age_s=1.0)
    a.update(over)
    return a


def dead_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@unittest.skipIf(baseline_source() is None, "git history with %s not available" % BASELINE)
class RuntimeGeometrySourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.mkdtemp(prefix="cargeo_test_")
        cls.new_source = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        cls.before, cls.before_ok = capture(baseline_source(), "ros_runtime_cargeo_baseline")
        cls.after, cls.after_ok = capture(cls.new_source, "ros_runtime_cargeo_default")
        fake = FakeKv(cls.build)
        try:
            cls.kv, cls.kv_ok = capture(cls.new_source, "ros_runtime_cargeo_kv", kv_args(fake.port))
        finally:
            fake.stop()

    # 1. default cpu unchanged
    def test_default_cpu_stream_identical_to_baseline(self):
        self.assertTrue(all(ok for ok, _ in self.before_ok.values()), self.before_ok)
        self.assertTrue(all(ok for ok, _ in self.after_ok.values()), self.after_ok)
        self.assertEqual(diff_names(self.before, self.after, strict), [])
        n = sum(len(strict(v)) for v in self.before.values())
        sys.stderr.write("\n[cpu-default] %d scenarios, %d events identical (sequence kept)\n"
                         % (len(self.before), n))
        self.assertGreater(n, 300)

    def test_explicit_cpu_identical_and_positive_control(self):
        explicit, ok = capture(self.new_source, "ros_runtime_cargeo_cpu", {"geometry_source": "cpu"})
        self.assertTrue(all(o for o, _ in ok.values()), ok)
        self.assertEqual(diff_names(self.before, explicit, strict), [])
        self.assertIn(CPU_CALL, self.new_source)
        variant = self.new_source.replace(CPU_CALL, CPU_CALL.replace(
            '{} if self.region_source == "kv_only"\n                                    else compute_geometry(frame, self.memory)',
            "{}"))
        self.assertNotEqual(variant, self.new_source)
        streams, _ = capture(variant, "ros_runtime_cargeo_cpu_variant")
        self.assertIn("scenario_occlusion_changes_view", diff_names(self.before, streams, strict))

    # 2. kv mode, same replay
    def test_kv_mode_stream_equals_cpu_after_removing_kv_rows(self):
        self.assertTrue(all(ok for ok, _ in self.kv_ok.values()), self.kv_ok)
        self.assertEqual(diff_names(self.before, self.kv, kv_normalise), [])
        stats = [json.loads(r["payload"]["data"]) for v in self.kv.values() for r in v
                 if r.get("event") == "publish" and r.get("topic") == "/fpga/kv_link/frame_stats"]
        processed = status_rows(self.kv, "frame_processed")
        self.assertEqual(len(stats), len(processed))
        self.assertGreater(len(stats), 10)
        self.assertTrue(all(p["geometry"]["result"] == "fused" for p in processed))
        self.assertTrue(all(p["geometry"]["shadow_equal"] in (True, None) for p in processed))
        self.assertEqual(status_rows(self.kv, "geometry_shadow_mismatch"), [])
        closes = status_rows(self.kv, "geometry_close")
        self.assertEqual(len(closes), len(self.kv))
        frames = sum(c["shadow"]["frames"] for c in closes)
        equal = sum(c["shadow"]["equal"] for c in closes)
        boxes = sum(s["boxes_sent"] for s in stats)
        sys.stderr.write("\n[kv-m5] %d frames through the KV path, shadow equal %d/%d, boxes sent %d\n"
                         % (len(stats), equal, frames, boxes))
        self.assertEqual(frames, equal)
        self.assertGreater(boxes, 0)
        for p in processed:      # frame-age chain present, all on one clock
            self.assertIsNotNone(p["geometry"]["chain_ms"]["send_to_back_ms"])
            # board decomposition joins these: KV segment durations and the RAW rx stamp
            self.assertIn("core_device", p["geometry"]["kv_times_ms"])
            self.assertIn("capture_nano_ns", p["geometry"])
            self.assertIsNotNone(p["geometry"]["kv_frame_id"])

    def test_kv_positive_control_previous_frame_depth_differs(self):
        from tools.car_geometry_kv.equivalence import _PrevFrameRoi
        fake = FakeKv(self.build, wrap=_PrevFrameRoi)
        try:
            streams, _ = capture(self.new_source, "ros_runtime_cargeo_kv_prev",
                                 kv_args(fake.port, kv_shadow_check=False))
        finally:
            fake.stop()
        self.assertNotEqual(diff_names(self.before, streams, kv_normalise), [])

    # 3. unavailable
    def test_unavailable_fallback_none_records_reason_and_differs(self):
        streams, ok = capture(self.new_source, "ros_runtime_cargeo_dead",
                              kv_args(dead_port(), kv_hello_timeout_s=0.05, kv_retry_s=60.0))
        rows = status_rows(streams, "geometry_unavailable")
        self.assertEqual(len(rows), len(streams))       # once per runtime (transition), with reason
        self.assertTrue(all(r["reason"] == "hello_timeout" and r["fallback"] == "none" for r in rows))
        processed = status_rows(streams, "frame_processed")
        self.assertTrue(processed)
        self.assertTrue(all(p["geometry"]["result"] == "backend_failure:hello_timeout" for p in processed))
        self.assertIn("scenario_occlusion_changes_view", diff_names(self.before, streams, kv_normalise))
        self.assertEqual(status_rows(streams, "geometry_fallback_cpu"), [])

    def test_unavailable_fallback_cpu_is_recorded_every_frame(self):
        streams, ok = capture(self.new_source, "ros_runtime_cargeo_dead_cpu",
                              kv_args(dead_port(), kv_hello_timeout_s=0.05, kv_retry_s=60.0,
                                      geometry_fallback="cpu", kv_shadow_check=False))
        self.assertTrue(all(o for o, _ in ok.values()), ok)
        fb = status_rows(streams, "geometry_fallback_cpu")
        processed = status_rows(streams, "frame_processed")
        self.assertEqual(len(fb), len(processed))
        self.assertTrue(all(p["geometry"]["fallback"] == "cpu" for p in processed))
        self.assertEqual(diff_names(self.before, streams, kv_normalise), [])


class KvGeometryUnitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.mkdtemp(prefix="cargeo_unit_")

    def frame_and_memory(self, depth=None):
        from perception.fpga_replay.core import Frame, Memory, Track
        h, w = 60, 80
        K = np.array([[60., 0, 40], [0, 60, 30], [0, 0, 1]])
        if depth is None:
            depth = np.full((h, w), np.float32(2.0), np.float32)
            depth[20:40, 30:50] = np.float32(1.0)
        frame = Frame("u-1", 5.0, np.zeros((h, w, 3), np.uint8), depth, K, np.eye(4), True, "t", "cal", None)
        mem = Memory()
        for tid, lo, hi in (("a", [-.2, -.2, 1.4], [.2, .2, 1.6]), ("b", [-.1, -.1, 2.6], [.1, .1, 2.8]),
                            ("c", [9., 9., 1.], [9.2, 9.2, 1.2])):
            lo, hi = np.array(lo), np.array(hi)
            mem.tracks[tid] = Track(tid, "cup", "red", (lo + hi) / 2, lo, hi, 0.0)
        return frame, mem

    def geo(self, port, **kw):
        from host.kv_geometry import KvGeometry
        kw.setdefault("depth_unit_m", 2.0 ** -10)
        kw.setdefault("hello_timeout_s", 1.0)
        kw.setdefault("timeout_s", 2.0)
        return KvGeometry(("127.0.0.1", port), **kw)

    def test_answer_equals_compute_geometry(self):
        from perception.fpga_replay.core import compute_geometry
        fake = FakeKv(self.build)
        try:
            g = self.geo(fake.port)
            frame, mem = self.frame_and_memory()
            out = g.evaluate(frame, mem)
            self.assertIsNone(out.reason)
            self.assertEqual(list(out.evidence.items()), list(compute_geometry(frame, mem).items()))
            self.assertEqual(out.evidence["c"], "out_of_view")
            self.assertEqual(out.info["roi_core"], "cpu")
            g.close()
        finally:
            fake.stop()

    def test_refusals(self):
        frame, mem = self.frame_and_memory()
        g = self.geo(dead_port(), hello_timeout_s=0.05, retry_s=60.0)
        out = g.evaluate(frame, mem)
        self.assertEqual((out.evidence, out.reason), (None, "hello_timeout"))
        out = g.evaluate(frame, mem)              # rate limited: no second HELLO
        self.assertEqual(out.reason, "hello_timeout")
        self.assertEqual(g.counts["connect_attempts"], 1)
        fake = FakeKv(self.build, roi=False)
        try:
            g2 = self.geo(fake.port)
            self.assertFalse(g2.connect())
            self.assertEqual(g2.reason, "kv_roi_core_none")
        finally:
            fake.stop()

    def test_build_id_gate(self):
        from host.kv_regs import mock_sysinfo
        blk, _ = mock_sysinfo(build_id=0x0117BC0A)
        fake = FakeKv(self.build, regs={"sysinfo": blk})
        try:
            g = self.geo(fake.port, expect_build_id=0x0117BC0A)
            self.assertTrue(g.connect(), g.reason)
            self.assertEqual(g.describe()["build_id"], "0x0117bc0a")
            g.close()
            g = self.geo(fake.port, expect_build_id=0x0117BC0B)
            self.assertFalse(g.connect())
            self.assertTrue(g.reason.startswith("build_id_mismatch"), g.reason)
        finally:
            fake.stop()

    def test_depth_not_representable_and_wrong_codes(self):
        from host.kv_link import m_to_codes
        fake = FakeKv(self.build)
        try:
            frame, mem = self.frame_and_memory()
            frame.depth[0, 0] = np.float32(1.0001)       # not on the 2^-10 grid
            out = self.geo(fake.port).evaluate(frame, mem)
            self.assertEqual(out.reason, "depth_not_representable")
            frame, mem = self.frame_and_memory()
            g = self.geo(fake.port)
            codes = m_to_codes(frame.depth, np.float32(2.0 ** -10))
            ok = g.evaluate(frame, mem, codes=codes)
            self.assertIsNone(ok.reason)
            self.assertEqual(g.backend.counts["codes_supplied"], 1)
            codes[5, 5] += 1                              # one code wrong: the crc must refuse
            bad = g.evaluate(frame, mem, codes=codes)
            self.assertEqual(bad.reason, "depth_crc_mismatch")
        finally:
            fake.stop()

    def test_request_bytes_unchanged_without_codes(self):
        """codes=None -> the request is byte-identical to the requantizing path's."""
        from host.kv_link import RemotePlFrameBackend, m_to_codes
        from host.m7_pipeline import CaptureIdentity

        class Recorder(object):
            server_info = {"core": "x", "features": {"roi_ext": 1}, "roi_max_boxes": 32}

            def __init__(self):
                self.sent, self.counts, self.last = [], {}, None

            def require(self, feature, hello_timeout_s=3.0):
                return self.server_info

            def call(self, req, ident, tail=None):    # tail: car-ar AGE block hook (c37818d)
                self.sent.append(bytes(req))
                raise __import__("host.kv_link", fromlist=["LinkTimeout"]).LinkTimeout("recorded")
        frame, _ = self.frame_and_memory()
        sent = []
        for codes in (None, m_to_codes(frame.depth, np.float32(2.0 ** -10))):
            rec = Recorder()
            be = RemotePlFrameBackend(rec, depth_unit_m=2.0 ** -10)
            ident = CaptureIdentity("s", 1, 10, "c", "cal", "l", "e")
            with self.assertRaises(Exception):
                be.frame_from_depth(depth_m=frame.depth, k=frame.K, camera_to_world=np.eye(4),
                                    grid_origin_m=(-.8, -.8, -.05), voxel_size_m=.05, shape=(32, 32, 32),
                                    frame_id=1, capture_time_ns=10, pose_time_ns=10, calibration_version="cal",
                                    pose_confirmed=True, identity=ident, codes=codes)
            sent.append(rec.sent[0])
        self.assertEqual(sent[0], sent[1])


class DepthConversionKnownAnswer(unittest.TestCase):
    def test_runtime_16uc1_conversion_is_codes_to_m(self):
        from host.kv_link import codes_to_m, m_to_codes
        from tools.car_geometry_kv.equivalence import runtime_depth
        codes = np.arange(65536, dtype=np.uint16).reshape(256, 256)
        for unit in (0.001, 0.0002):
            d = runtime_depth(codes, unit)
            ref = codes_to_m(codes, np.float32(unit))
            self.assertTrue(np.array_equal(np.isnan(d), np.isnan(ref)))
            self.assertTrue(np.array_equal(d.view(np.uint32)[~np.isnan(d)], ref.view(np.uint32)[~np.isnan(ref)]))
            self.assertTrue(np.array_equal(m_to_codes(d, np.float32(unit)), codes))
        # positive control: a float64 multiply then cast is NOT always the same bits
        alt = (codes.astype(np.float64) * 0.001).astype(np.float32)
        alt[codes == 0] = np.nan
        ref = codes_to_m(codes, np.float32(0.001))
        self.assertFalse(np.array_equal(alt[1:], ref[1:]) and np.array_equal(alt.view(np.uint32)[codes > 0],
                                                                           ref.view(np.uint32)[codes > 0]))
        sys.stderr.write("\n[depth] numpy %s: 16UC1 conversion == codes_to_m for 65536 codes x 2 units\n"
                         % np.__version__)


class ArrivalKeyTests(unittest.TestCase):
    """The RAW arrival stamp is stored in on_images and looked up in prepare_images; both
    must key by integer ns. Float keys (sec + nanosec/1e9 vs Time.nanoseconds/1e9) differ in
    the last ulp for realtime stamps and the lookup would miss silently (frame age null)."""

    def test_both_sites_use_integer_ns_keys(self):
        src = (ROOT / "host" / "ros_runtime.py").read_text(encoding="utf-8")
        self.assertIn("self.kv_arrivals[rgb.header.stamp.sec * 10**9 + rgb.header.stamp.nanosec]", src)
        self.assertIn("self.kv_arrivals.pop(stamp.nanoseconds, None)", src)
        self.assertNotIn("self.kv_arrivals[stamp]", src)

    def test_positive_control_float_keys_disagree(self):
        sec, nanosec = 1760000000, 15838
        self.assertNotEqual(sec + nanosec / 1e9, (sec * 10**9 + nanosec) / 1e9)
        self.assertEqual(sec * 10**9 + nanosec, int(sec) * 1000000000 + int(nanosec))


if __name__ == "__main__":
    unittest.main()
