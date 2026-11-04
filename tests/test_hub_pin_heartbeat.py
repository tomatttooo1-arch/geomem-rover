"""Heartbeat indicator of the hub STATE (kv-permit 10-14 finding).

STATUS b1 (the heartbeat pin) is a 100 Hz square wave while permit is 1, else 0 (RTL
m7_permit_axil.v:184-186). The hub polls TELEM at 10 Hz -- an integer multiple of the wave period,
phase-locked -- so a single read of b1 is a near-constant level (PC STATE pin=1 fraction during
permit 0.003/0/0 vs 0.46 on a 10 ms recorder). "Heartbeat being driven" is therefore b0.

1. heartbeat_on equals the permit bit for permit=1 and permit=0.
2. Aliasing: pin sampled 0 every poll while permit=1 -> heartbeat shown as on (hub and fpgac).
3. Compatibility: "pin" still carries the raw b1 sample; fpgac falls back to "permit" for an
   older hub without heartbeat_on; unavailable block -> None.
4. Positive control: an indicator that reads "pin" (the old reading) fails the aliasing case.
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

T0 = 5 * NS
NOW = T0 + 20 * MS


def telemetry(status, permit_avail=True):
    blocks = {"permit": {"available": permit_avail, "status": status, "last_cause": 0,
                         "revoke_count": 0, "frames_accepted": 10, "frames_rejected": 0,
                         "age_ms": 12.0}}
    if not permit_avail:
        blocks["permit"] = {"available": False, "reason": "no_uio_node"}
    avail = {"permit"} if permit_avail else set()
    return Telemetry(avail, blocks, {"permit": 2.0} if permit_avail else {}, 1.5, "kvboot")


def state(status, permit_avail=True):
    core = HubCore("live", hub_boot_id="hb")
    core.feed_telemetry(telemetry(status, permit_avail), T0)
    return json.loads(fpgac.dumps(core.compose_state(NOW)))


def pin_indicator(s):
    """The old reading: heartbeat = the single pin sample (what this change retires)."""
    p = (s.get("kv") or {}).get("permit") or {}
    return p.get("pin") if p.get("avail") else None


class HeartbeatIndicatorTests(unittest.TestCase):
    def test_heartbeat_on_follows_permit_bit(self):
        for status, want in ((0b100001, True), (0b100011, True), (0b000000, False), (0b001000, False)):
            p = state(status)["kv"]["permit"]
            self.assertIs(p["heartbeat_on"], want, bin(status))
            self.assertIs(p["permit"], want)
            self.assertIs(fpgac.kv_heartbeat(state(status)), want)

    def test_aliased_pin_zero_with_permit_shows_heartbeat(self):
        # every 10 Hz poll lands on the low half of the 100 Hz wave: b1 == 0, b0 == 1
        samples = [state(0b101101) for _ in range(5)]
        for s in samples:
            self.assertIs(s["kv"]["permit"]["pin"], False)
            self.assertIs(s["kv"]["permit"]["heartbeat_on"], True)
            self.assertIs(fpgac.kv_heartbeat(s), True)
            self.assertIn(" kv_hb=True", fpgapc.summary(s))

    def test_positive_control_pin_reading_fails_aliasing_case(self):
        s = state(0b101101)
        self.assertIsNot(pin_indicator(s), fpgac.kv_heartbeat(s))

    def test_raw_pin_field_kept(self):
        self.assertIs(state(0b000011)["kv"]["permit"]["pin"], True)
        self.assertIs(state(0b000001)["kv"]["permit"]["pin"], False)

    def test_older_hub_without_heartbeat_on_falls_back_to_permit(self):
        s = state(0b000001)
        del s["kv"]["permit"]["heartbeat_on"]
        self.assertIs(fpgac.kv_heartbeat(s), True)
        s["kv"]["permit"]["permit"] = False
        s["kv"]["permit"]["pin"] = True                 # pin is never consulted
        self.assertIs(fpgac.kv_heartbeat(s), False)

    def test_unavailable_or_missing(self):
        s = state(0, permit_avail=False)
        self.assertNotIn("heartbeat_on", s["kv"]["permit"])
        self.assertIsNone(fpgac.kv_heartbeat(s))
        self.assertIn(" kv_hb=None", fpgapc.summary(s))
        self.assertIsNone(fpgac.kv_heartbeat({}))
        self.assertIsNone(fpgac.kv_heartbeat(None))


if __name__ == "__main__":
    unittest.main()
