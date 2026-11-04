"""kv group age of the hub STATE (10-11 integration board smoke: kv.age_ms median 97 s while
kv.geometry.age_ms was 58 ms). The group age took the maximum over every TELEM block age,
including roi, whose last answer is from the last frame that carried boxes (none in view after
the first ~3 s of the replay) -- a block the group does not even show.

Rule now (spec A.3): kv.age_ms = TELEM sample age + max over blocks with live data (geometry,
permit, a connected encoder); unavailable / no-data / not-connected / event-driven blocks are
excluded and named in kv.age_excluded, never dropped silently. fpgac.kv_blocks reads it.

1. Known answer: roi 95 s old and an encoder without MCU do not enter the age; both are visible.
2. Encoder connected (LINK_OK, frame age 40 ms) enters; disconnected after a while (LINK_OK low,
   frame age 90 s) is not_connected and excluded, its frame_age_ms still shown.
3. Variant: the old rule (max over all age_upper_ms) gives the 95 s -- the test detects it.
4. Permit before any frame (since_done None): no_data, excluded.
5. fpgac.kv_blocks states; fpgapc summary line carries kv_age_ms and the not-live blocks.
Python 3.8+, no ROS.
"""
import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "pc"))

import fpgac  # noqa: E402
import fpgapc  # noqa: E402
from host.hub.hub_core import HubCore, MS, NS  # noqa: E402
from host.kv_status import Telemetry  # noqa: E402


def telemetry(encoder_link=False, encoder_age=None, permit_since=3.0, roi_age=95000.0):
    blocks = {
        "geometry": {"available": True, "core": "pl", "wait": "irq", "served": 955,
                     "last": {"frame_id": 955, "status": 0, "counts": [1, 2, 3],
                              "times_ms": {"core_device": 5.5}}},
        "roi": {"available": True, "core": "pl", "served": 27,
                "last": {"frame_id": 27, "roi_status": 0, "rows": 3}},
        "permit": {"available": True, "status": 0x137, "last_cause": 1, "revoke_count": 1,
                   "frames_accepted": 955, "frames_rejected": 0, "age_ms": 64.0},
        "encoder": {"available": True, "status": 1 if encoder_link else 0, "seq_word": 0, "counters": [0] * 11,
                    "v_left_mm_s": 0.0, "v_right_mm_s": 0.0, "v_center_mm_s": 0.0, "dist_center_m": 0.0,
                    "heading_rad": 0.0, "frame_age_ms": encoder_age},
        "ar": {"available": True, "mode": "LIVE", "presenter": "drm"},
        "proc": {"available": True, "cpu_per_wall_1s": 0.4},
        "estop": {"available": True},
        "board": {"available": True},
    }
    ages = {"geometry": 5.0}
    if roi_age is not None:
        ages["roi"] = roi_age
    if permit_since is not None:
        ages["permit"] = permit_since
    if encoder_age is not None:
        ages["encoder"] = encoder_age
    return Telemetry(set(blocks), blocks, ages, 1.5, "kvboot")


T0 = 5 * NS
NOW = T0 + 20 * MS
DELTA = 20.0


def state(tel, cls=HubCore):
    core = cls("live", hub_boot_id="hb")
    core.feed_telemetry(tel, T0)
    return core.compose_state(NOW)


class OldRule(HubCore):
    """Variant: kv age = TELEM age + max over ALL age_upper_ms (the 10-11 rule)."""

    def _kv(self, now):
        g = HubCore._kv(self, now)
        tel = self._val("telemetry")
        ups = [v for v in tel.age_upper_ms.values() if v is not None]
        g["age_ms"] = (now - self.latest["telemetry"][1]) / 1e6 + (max(ups) if ups else 0.0)
        return g


class KvGroupAgeTests(unittest.TestCase):
    def test_roi_and_unconnected_encoder_excluded_but_visible(self):
        s = state(telemetry())
        kv = s["kv"]
        self.assertEqual(kv["age_ms"], DELTA + 5.0)
        self.assertEqual(kv["age_basis"], ["geometry", "permit"])
        self.assertEqual(kv["age_excluded"], {"encoder": "not_connected", "roi": "event_driven"})
        self.assertEqual(kv["roi"]["age_ms"], 95000.0 + DELTA)          # still shown, with its age
        self.assertEqual((kv["roi"]["last_frame_id"], kv["roi"]["served"]), (27, 27))
        self.assertFalse(kv["encoder"]["connected"])
        self.assertTrue(kv["encoder"]["avail"])

    def test_old_rule_variant_is_caught(self):
        s = state(telemetry(), cls=OldRule)
        self.assertNotEqual(s["kv"]["age_ms"], DELTA + 5.0)
        self.assertEqual(s["kv"]["age_ms"], DELTA + 95000.0)

    def test_connected_encoder_enters(self):
        kv = state(telemetry(encoder_link=True, encoder_age=40.0))["kv"]
        self.assertTrue(kv["encoder"]["connected"])
        self.assertEqual(kv["age_ms"], DELTA + 40.0)
        self.assertEqual(kv["age_basis"], ["encoder", "geometry", "permit"])

    def test_encoder_link_lost_excluded_age_shown(self):
        kv = state(telemetry(encoder_link=False, encoder_age=90000.0))["kv"]
        self.assertEqual(kv["age_excluded"]["encoder"], "not_connected")
        self.assertEqual(kv["age_ms"], DELTA + 5.0)
        self.assertEqual(kv["encoder"]["age_ms"], 90000.0 + DELTA)

    def test_permit_without_frames_no_data(self):
        kv = state(telemetry(permit_since=None))["kv"]
        self.assertEqual(kv["age_excluded"]["permit"], "no_data")
        self.assertEqual(kv["age_basis"], ["geometry"])

    def test_unavailable_block(self):
        tel = telemetry()
        tel.blocks["permit"] = {"available": False, "reason": "no_uio_node"}
        tel.avail.discard("permit")
        tel.age_upper_ms.pop("permit")
        kv = state(tel)["kv"]
        self.assertEqual(kv["age_excluded"]["permit"], "unavailable")
        self.assertEqual(kv["permit"]["reason"], "no_uio_node")

    def test_fpgac_kv_blocks_and_summary(self):
        s = json.loads(fpgac.dumps(state(telemetry())))
        b = fpgac.kv_blocks(s)
        self.assertEqual({k: v["state"] for k, v in b.items()},
                         {"geometry": "live", "permit": "live", "encoder": "not_connected",
                          "roi": "event_driven", "ar": "live", "proc": "live"})
        line = fpgapc.summary(s)
        self.assertIn("kv_age_ms=25.0", line)
        self.assertIn("kv_not_live=encoder:not_connected,roi:event_driven", line)
        self.assertEqual(fpgac.kv_blocks({})["geometry"]["state"], "unknown")


if __name__ == "__main__":
    unittest.main()
