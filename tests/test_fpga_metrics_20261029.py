"""10-29 console FPGA panel (display only): per-frame fields (host/fpga_metrics.py), hub STATE perf.fpga, bridge
snapshot fpga.live (ui/geomem/bridge/gmb_proto.compose_fpga_live), the KV rolling core name asked once at connect.

Run: PYTHONPATH=. python3 tests/test_fpga_metrics_20261029.py
"""

import json
import sys
import unittest
from collections import Counter, namedtuple
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ui" / "geomem" / "bridge"))

from host import fpga_metrics as fm  # noqa: E402
from host.hub.hub_core import HubCore, MS, NS  # noqa: E402
from host.kv_link import RoiOutcome  # noqa: E402
import gmb_proto as gp  # noqa: E402

COUNTS = {"valid_grid_samples": 200000, "invalid_depth_or_projection": 50000, "outside_grid": 6000}
LAST_PL = {"core": "pl:devmem:irq", "rtt_s": 0.0142, "kv_ns": {"core_device": 3100000}}


def roi_ok(n=3, device_ns=420000, core="pl"):
    return RoiOutcome(True, None, [{"row_id": i} for i in range(n)], core,
                      {"prep": 10000, "device": device_ns, "readback": 5000, "total": 500000}, 1234)


def stats_row(geo_ms=3.1, rtt_ms=14.2, fpga=None, geo_core="pl:devmem:irq"):
    f = {"geo_core": geo_core, "pixels": 256000, "roi_core": None, "roi_device_ms": None, "roi_boxes": None,
         "roll_core": "pl", "roll_device_ms": 0.28, "roll_changes": 412}
    f.update(fpga or {})
    return {"v": 1, "kv_frame_id": 1, "capture_nano_ns": 0, "result": "fused", "frame_age_ms": 60.0,
            "channel_ms": 20.0, "mailbox_wait_ms": 1.0, "rtt_ms": rtt_ms,
            "kv_times_ms": {"core_device": geo_ms}, "client_counts": {}, "fpga": f}


class FrameFieldsTests(unittest.TestCase):
    def test_pl_frame_all_three_cores(self):
        f = fm.frame_fields(client_last=LAST_PL, roi=roi_ok(), roll_times_ns={"prep": 1, "device": 280000,
                                                                             "readback": 2},
                            roll_core="pl", roll_changes=412, nav_counts=COUNTS)
        self.assertEqual(f, {"geo_core": "pl:devmem:irq", "pixels": 256000, "roi_core": "pl", "roi_device_ms": 0.42,
                             "roi_boxes": 3, "roll_core": "pl", "roll_device_ms": 0.28, "roll_changes": 412})
        json.dumps(f, allow_nan=False)

    def test_missing_pieces_are_none_not_stand_ins(self):
        f = fm.frame_fields()
        self.assertTrue(all(v is None for v in f.values()), f)
        # no boxes sent / ROI failed / frame not fused / counts missing a key
        f = fm.frame_fields(client_last=LAST_PL, roi=RoiOutcome(False, "region_backend_failure:x", [], "pl", {}, None),
                            roll_times_ns=None, roll_core="pl", roll_changes=None,
                            nav_counts={"valid_grid_samples": 5})
        self.assertEqual((f["roi_core"], f["roi_device_ms"], f["roi_boxes"]), ("pl", None, None))
        self.assertEqual((f["roll_device_ms"], f["roll_changes"], f["pixels"]), (None, None, None))
        # a zero device time is "not measured", not 0 ms
        f = fm.frame_fields(roi=roi_ok(device_ns=0), roll_times_ns={"device": 0}, roll_changes=3)
        self.assertEqual((f["roi_device_ms"], f["roll_device_ms"]), (None, None))

    def test_is_pl(self):
        self.assertTrue(fm.is_pl("pl") and fm.is_pl("pl:devmem:irq"))
        self.assertFalse(fm.is_pl("cpu") or fm.is_pl("place") or fm.is_pl(None) or fm.is_pl(""))


class SummarizeTests(unittest.TestCase):
    def test_medians_counts_and_pl_only(self):
        rows = [stats_row(geo_ms=g, rtt_ms=r) for g, r in ((3.0, 14.0), (3.2, 15.0), (9.0, 30.0))]
        rows[1]["fpga"].update(roi_core="pl", roi_device_ms=0.5, roi_boxes=2)
        rows[2]["fpga"].update(roi_core="pl", roi_device_ms=0.3, roi_boxes=4, roll_changes=99)
        s = fm.summarize(rows)
        self.assertEqual(s, {"geo_device_ms": 3.2, "geo_frames": 3, "pixels": 256000, "roi_device_ms": 0.4,
                             "roi_frames": 2, "roi_boxes": 4, "roll_device_ms": 0.28, "roll_frames": 3,
                             "roll_changes": 99, "rtt_ms": 15.0, "rows": 3})

    def test_cpu_cores_and_old_rows_are_not_shown_as_fpga(self):
        cpu = stats_row(geo_core="cpu", fpga={"roi_core": "cpu", "roi_device_ms": 2.0, "roi_boxes": 1,
                                              "roll_core": "cpu"})
        old = stats_row()
        del old["fpga"]                                  # producer before 10-29
        s = fm.summarize([cpu, old, "junk"])
        self.assertEqual((s["geo_device_ms"], s["roi_device_ms"], s["roll_device_ms"], s["pixels"]),
                         (None, None, None, None))
        self.assertEqual((s["geo_frames"], s["roi_frames"], s["roll_frames"], s["rows"]), (0, 0, 0, 2))
        self.assertEqual(s["rtt_ms"], 14.2)              # the round trip is a link number, any core


class HubPerfTests(unittest.TestCase):
    def test_state_perf_fpga_from_frame_stats(self):
        core = HubCore("live", hub_boot_id="hb")
        t0 = 1000 * NS
        for i in range(12):
            core.feed_frame_stats(json.dumps(stats_row(geo_ms=3.0 + 0.1 * (i % 3))), t0 + i * 100 * MS)
        s = core.compose_state(t0 + 1200 * MS)
        f = s["perf"]["fpga"]
        self.assertEqual((f["geo_frames"], f["rows"], f["pixels"], f["roll_changes"]), (10, 10, 256000, 412))
        self.assertAlmostEqual(f["geo_device_ms"], 3.1)
        self.assertAlmostEqual(s["perf"]["frames_per_s"], 10.0)
        self.assertEqual(s["nonfinite_fields"], [])
        self.assertEqual(s["perf"]["age_ms"], 100.0)
        json.dumps(s, allow_nan=False)

    def test_no_frames_yet(self):
        s = HubCore("live", hub_boot_id="hb").compose_state(5 * NS)
        self.assertEqual(s["perf"]["fpga"]["rows"], 0)
        self.assertIsNone(s["perf"]["fpga"]["geo_device_ms"])


def bridge_state(perf_age=100.0, link_ok=True, permit=True, accepted=1234, clock_age=50.0, clock_error=None,
                 estop=False):
    perf = {"age_ms": perf_age, "frames_per_s": 9.8,
            "fpga": {"geo_device_ms": 3.1, "geo_frames": 10, "pixels": 256000, "roi_device_ms": 0.42,
                     "roi_frames": 6, "roi_boxes": 3, "roll_device_ms": 0.28, "roll_frames": 10, "roll_changes": 412,
                     "rtt_ms": 14.2, "rows": 10}}
    clock = {"age_ms": clock_age, "ok": clock_error is None, "bound_us": 750.0, "error": clock_error}
    kv = {"link_ok": link_ok, "age_ms": 120.0, "geometry": {"avail": True, "age_ms": 60.0, "pl_device_ms": 3.0},
          "roi": {"avail": True, "age_ms": 300.0, "rows": 3},
          "permit": {"avail": True, "permit": permit, "heartbeat_on": permit, "estop": estop,
                     "frames_accepted": accepted}}
    return {"perf": perf, "clock": clock, "kv": kv, "estop": {"latched": estop}}


class BridgeLiveTests(unittest.TestCase):
    def test_live_values(self):
        live = gp.compose_fpga(bridge_state())["live"]
        self.assertEqual(live, {"framesAgeMs": 100.0, "fps": 9.8, "geoMs": 3.1, "pixels": 256000, "roiMs": 0.42,
                                "roiBoxes": 3, "roiFrames": 6, "rollMs": 0.28, "rollCells": 412, "rollFrames": 10,
                                "frames": 10, "rttMs": 14.2, "syncBoundMs": 0.75, "permit": True, "renewals": 1234,
                                "estop": False})
        gp.dumps({"fpga": gp.compose_fpga(bridge_state())})

    def test_frames_stopped_gives_nulls_not_old_values(self):
        live = gp.compose_fpga(bridge_state(perf_age=2500.0))["live"]
        for k in ("fps", "geoMs", "pixels", "roiMs", "roiBoxes", "rollMs", "rollCells", "rttMs", "frames"):
            self.assertIsNone(live[k], k)
        self.assertEqual(live["framesAgeMs"], 2500.0)
        self.assertEqual(live["permit"], True)            # the permit has its own source (TELEM)

    def test_link_lost_and_clock_error(self):
        live = gp.compose_fpga(bridge_state(link_ok=False, clock_error="no_anchor"))["live"]
        self.assertEqual((live["permit"], live["renewals"], live["estop"], live["syncBoundMs"]),
                         (None, None, None, None))
        live = gp.compose_fpga(bridge_state(clock_age=5000.0))["live"]
        self.assertIsNone(live["syncBoundMs"])

    def test_hub_stopped_sending_state(self):
        live = gp.compose_fpga(bridge_state(), hub_age_ms=2500.0)["live"]
        self.assertTrue(all(live[k] is None for k in live if k != "framesAgeMs"), live)
        self.assertEqual(live["framesAgeMs"], 2600.0)
        live = gp.compose_fpga(bridge_state(), hub_age_ms=300.0)["live"]
        self.assertEqual((live["geoMs"], live["permit"], live["syncBoundMs"]), (3.1, True, 0.75))

    def test_permit_off_and_estop(self):
        live = gp.compose_fpga(bridge_state(permit=False, estop=True))["live"]
        self.assertEqual((live["permit"], live["estop"]), (False, True))

    def test_state_without_perf(self):
        st = bridge_state()
        del st["perf"], st["clock"]
        live = gp.compose_fpga(st)["live"]
        self.assertIsNone(live["geoMs"])
        self.assertIsNone(live["framesAgeMs"])
        self.assertIsNone(gp.compose_fpga({"perf": {}}))  # no kv group: no fpga key at all (as before)


class RollingCoreNameTests(unittest.TestCase):
    def _backend(self, stats):
        from host.kv_rolling import RemotePlRollingBackend
        b = RemotePlRollingBackend.__new__(RemotePlRollingBackend)

        class Link(object):
            counts = Counter()

            def stats(self, timeout_s):
                if isinstance(stats, Exception):
                    raise stats
                return stats
        b.link = Link()
        b.counts = b.link.counts
        return b

    def test_ask_core(self):
        self.assertEqual(self._backend({"rolling": {"core": "pl", "counts": {}}})._ask_core(), "pl")
        self.assertEqual(self._backend({"rolling": {"core": "cpu"}})._ask_core(), "cpu")
        b = self._backend(None)                          # STATS not answered
        self.assertIsNone(b._ask_core())
        self.assertEqual(b.counts["roll_core_unknown"], 1)
        self.assertIsNone(self._backend(OSError("down"))._ask_core())
        self.assertIsNone(self._backend({"rolling": None})._ask_core())


if __name__ == "__main__":
    unittest.main()
