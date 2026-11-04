"""kv group age while the e-stop is latched (10-11 v2 formal run; controller ruling 10-11 night).

Measured: during a 2 s latch the PC's kv.age_ms climbed 100 -> 2100 ms while kv.geometry stayed
~100 ms and the link was fresh. The climbing block is permit: its age is since_done (PL
timebase now - LAST_DONE_TS + round trip), and the PL accepts no frame done-edge while forced
off (FRAMES_ACCEPTED frozen at 1040 the whole latch; the AGE_CYCLES register frozen at 81.8 ms).

Rule now (spec A.3): while latched (the hub's latch, incl. one adopted from the KV TELEM estop
block, or the permit block's own ESTOP status bit in that sample) permit leaves the group age with
reason "estop_latched" (急停锁存); permit.age_ms and permit_age_ms stay in the STATE. Not
latched: unchanged (a stale permit still counts).

1. Known answer: latched, since_done 2100 ms -> group age = TELEM age + geometry; permit
   excluded "estop_latched", its own ages still shown.
2. Old rule (permit always counted) gives the 2.1 s -- the variant is caught.
3. Not latched, same 2100 ms permit -> it still counts (exclusion only under the latch).
4. Hub latched before the KV applies it (permit ESTOP bit 0): excluded too.
5. PL ESTOP bit set while the hub has not latched (yet): excluded.
6. Latch adopted from the KV's TELEM estop block: excluded.
7. A hub that never saw the latch (e.g. restarted after the release), ESTOP bit clear: permit
   counts with its real age (nothing tells it the age is the latch's).
9-14 (v3, controller 10-11 night: estop_release_pending, 解除待接受新帧): after the latch ends,
   until the permit block's FRAMES_ACCEPTED moves past the value it was frozen at, permit stays
   out with reason "estop_release_pending" (its ages still shown); the first sample with a newly
   accepted frame counts it again. Old rule variant (pre-v3: counts right at the release, the
   2.1 s shows for that sample) caught. Re-latch while pending -> estop_latched; counter unknown
   (None) or another KV boot -> no pending (permit counts). Latch seen only as the PL ESTOP bit
   (hub restarted during the latch) works the same.
8. fpgac.kv_blocks keeps permit "live" with its age (no UI change); fpgapc summary carries
   the group age.
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
DELTA = 20.0
GEO = 80.0
PERMIT_SINCE = 2100.0
ESTOP_BIT = 1 << 3


def telemetry(permit_since=PERMIT_SINCE, pl_estop=True, kv_estop_block=None, fa=1040, boot="kvboot"):
    status = 0x34 | (ESTOP_BIT if pl_estop else 0) | (0 if pl_estop else 1)   # permit bit 0 off when forced
    blocks = {
        "geometry": {"available": True, "core": "pl", "wait": "irq", "served": 1100,
                     "last": {"frame_id": 1100, "status": 0, "counts": [1, 2, 3],
                              "times_ms": {"core_device": 5.4}}},
        "roi": {"available": True, "core": "pl", "served": 27, "last": {"frame_id": 27, "roi_status": 0, "rows": 3}},
        "permit": {"available": True, "status": status, "last_cause": 2, "revoke_count": 3,
                   "frames_accepted": fa, "frames_rejected": 0, "age_ms": 81.8},
        "encoder": {"available": True, "status": 0, "seq_word": 0, "counters": [0] * 11,
                    "frame_age_ms": None},
        "ar": {"available": True, "mode": "LIVE", "presenter": "drm"},
        "proc": {"available": True, "cpu_per_wall_1s": 0.4},
        "estop": dict({"available": True}, **(kv_estop_block or {})),
        "board": {"available": True},
    }
    ages = {"geometry": GEO, "roi": 95000.0}
    if permit_since is not None:
        ages["permit"] = permit_since
    return Telemetry(set(blocks), blocks, ages, 1.5, boot)


def state(tel, cls=HubCore, hub_latch=False):
    core = cls("live", hub_boot_id="hb")
    if hub_latch:
        core.estop.assert_request(("pc1", 1, "estop"), T0 - 2 * NS)
    core.feed_telemetry(tel, T0)
    return core, core.compose_state(NOW)


class OldRule(HubCore):
    """Variant: the rule before 10-11 night (permit counted whatever the latch)."""

    def _kv(self, now):
        g = HubCore._kv(self, now)
        if g.get("age_excluded", {}).get("permit") == "estop_latched":
            g["age_ms"] = max(g["age_ms"], g["permit"]["age_ms"])
            g["age_basis"] = sorted(g["age_basis"] + ["permit"])
            del g["age_excluded"]["permit"]
        return g


class PreV3Release(HubCore):
    """Variant: the v2 rule (no estop_release_pending: permit counts right at the release)."""

    def _kv(self, now):
        g = HubCore._kv(self, now)
        if g.get("age_excluded", {}).get("permit") == "estop_release_pending":
            g["age_ms"] = max(g["age_ms"], g["permit"]["age_ms"])
            g["age_basis"] = sorted(g["age_basis"] + ["permit"])
            del g["age_excluded"]["permit"]
        return g


def sequence(steps, cls=HubCore):
    """steps: telemetry samples fed one by one (each followed by a STATE); returns the kv groups."""
    core = cls("live", hub_boot_id="hb")
    out = []
    for i, tel in enumerate(steps):
        t = T0 + i * 100 * MS
        core.feed_telemetry(tel, t)
        out.append(core.compose_state(t + 20 * MS)["kv"])
    return core, out


class KvAgeEstopTests(unittest.TestCase):
    def test_latched_permit_excluded_ages_visible(self):
        core, s = state(telemetry(), hub_latch=True)
        kv = s["kv"]
        self.assertTrue(s["estop"]["latched"])
        self.assertEqual(kv["age_ms"], DELTA + GEO)
        self.assertEqual(kv["age_basis"], ["geometry"])
        self.assertEqual(kv["age_excluded"]["permit"], "estop_latched")
        self.assertEqual(kv["permit"]["age_ms"], PERMIT_SINCE + DELTA)     # still shown
        self.assertEqual(kv["permit"]["permit_age_ms"], 81.8)
        self.assertTrue(kv["permit"]["estop"])
        self.assertFalse(kv["permit"]["permit"])

    def test_old_rule_variant_is_caught(self):
        _, s = state(telemetry(), cls=OldRule, hub_latch=True)
        self.assertEqual(s["kv"]["age_ms"], DELTA + PERMIT_SINCE)          # the 2.1 s the PC showed
        self.assertNotEqual(s["kv"]["age_ms"], DELTA + GEO)

    def test_not_latched_stale_permit_still_counts(self):
        _, s = state(telemetry(pl_estop=False))
        kv = s["kv"]
        self.assertFalse(s["estop"]["latched"])
        self.assertEqual(kv["age_ms"], DELTA + PERMIT_SINCE)
        self.assertEqual(kv["age_basis"], ["geometry", "permit"])
        self.assertNotIn("permit", kv["age_excluded"])

    def test_hub_latched_before_kv_applies(self):
        _, s = state(telemetry(permit_since=3.0, pl_estop=False), hub_latch=True)
        kv = s["kv"]
        self.assertEqual(kv["age_excluded"]["permit"], "estop_latched")
        self.assertEqual(kv["age_ms"], DELTA + GEO)
        self.assertEqual(kv["permit"]["age_ms"], 3.0 + DELTA)

    def test_pl_estop_bit_without_hub_latch(self):
        _, s = state(telemetry(pl_estop=True))
        self.assertFalse(s["estop"]["latched"])
        self.assertEqual(s["kv"]["age_excluded"]["permit"], "estop_latched")
        self.assertEqual(s["kv"]["age_ms"], DELTA + GEO)

    def test_latch_adopted_from_kv(self):
        core, s = state(telemetry(pl_estop=False, kv_estop_block={"latched": True, "epoch": 4}))
        self.assertEqual(core.counts["estop_adopted"], 1)
        self.assertTrue(s["estop"]["latched"])
        self.assertEqual(s["kv"]["age_excluded"]["permit"], "estop_latched")

    def test_after_release_permit_counts_again(self):
        _, s = state(telemetry(pl_estop=False))       # released, next done-edge not yet in
        self.assertEqual(s["kv"]["age_basis"], ["geometry", "permit"])
        self.assertEqual(s["kv"]["age_ms"], DELTA + PERMIT_SINCE)

    def test_release_pending_until_next_accepted_frame(self):
        _, kv = sequence([telemetry(fa=1040),                                   # latched (PL bit)
                          telemetry(pl_estop=False, fa=1040),                   # released, no new frame
                          telemetry(permit_since=2200.0, pl_estop=False, fa=1040),
                          telemetry(permit_since=12.0, pl_estop=False, fa=1041)])  # next frame accepted
        self.assertEqual(kv[0]["age_excluded"]["permit"], "estop_latched")
        for g, since in ((kv[1], PERMIT_SINCE), (kv[2], 2200.0)):
            self.assertEqual(g["age_excluded"]["permit"], "estop_release_pending")
            self.assertEqual(g["age_ms"], DELTA + GEO)
            self.assertEqual(g["age_basis"], ["geometry"])
            self.assertEqual(g["permit"]["age_ms"], since + DELTA)               # still shown
        self.assertNotIn("permit", kv[3]["age_excluded"])
        self.assertEqual(kv[3]["age_basis"], ["geometry", "permit"])
        self.assertEqual(kv[3]["age_ms"], DELTA + GEO)                           # 12 < 80: geometry max

    def test_pre_v3_release_variant_is_caught(self):
        _, kv = sequence([telemetry(fa=1040), telemetry(pl_estop=False, fa=1040)], cls=PreV3Release)
        self.assertEqual(kv[1]["age_ms"], DELTA + PERMIT_SINCE)                  # the 2.1 s at the release
        _, kv = sequence([telemetry(fa=1040), telemetry(pl_estop=False, fa=1040)])
        self.assertEqual(kv[1]["age_ms"], DELTA + GEO)

    def test_release_pending_ends_on_counter_change_only_after_latch(self):
        # stale permit with the counter moving and no latch before: counts (unchanged rule)
        _, kv = sequence([telemetry(pl_estop=False, fa=5), telemetry(pl_estop=False, fa=5)])
        self.assertEqual(kv[1]["age_basis"], ["geometry", "permit"])
        self.assertEqual(kv[1]["age_ms"], DELTA + PERMIT_SINCE)

    def test_relatch_while_pending(self):
        _, kv = sequence([telemetry(fa=1040), telemetry(pl_estop=False, fa=1040),
                          telemetry(fa=1040), telemetry(pl_estop=False, fa=1040),
                          telemetry(pl_estop=False, fa=1042)])
        self.assertEqual([g["age_excluded"].get("permit") for g in kv],
                         ["estop_latched", "estop_release_pending", "estop_latched",
                          "estop_release_pending", None])

    def test_unknown_counter_or_new_kv_boot_no_pending(self):
        _, kv = sequence([telemetry(fa=None), telemetry(pl_estop=False, fa=None)])
        self.assertEqual(kv[0]["age_excluded"]["permit"], "estop_latched")
        self.assertNotIn("permit", kv[1]["age_excluded"])
        self.assertEqual(kv[1]["age_ms"], DELTA + PERMIT_SINCE)
        _, kv = sequence([telemetry(fa=1040), telemetry(pl_estop=False, fa=None)])
        self.assertNotIn("permit", kv[1]["age_excluded"])
        _, kv = sequence([telemetry(fa=1040), telemetry(pl_estop=False, fa=1040, boot="kvboot2")])
        self.assertNotIn("permit", kv[1]["age_excluded"])
        self.assertEqual(kv[1]["age_ms"], DELTA + PERMIT_SINCE)

    def test_hub_latch_release_path(self):
        core = HubCore("live", hub_boot_id="hb")
        core.estop.assert_request(("pc1", 1, "estop"), T0 - 2 * NS)
        core.feed_telemetry(telemetry(permit_since=3.0, pl_estop=False, fa=1040), T0)
        self.assertEqual(core.compose_state(NOW)["kv"]["age_excluded"]["permit"], "estop_latched")
        core.estop.latched = False                                               # released on the hub
        core.feed_telemetry(telemetry(pl_estop=False, fa=1040), T0 + 100 * MS)
        kv = core.compose_state(NOW + 100 * MS)["kv"]
        self.assertEqual(kv["age_excluded"]["permit"], "estop_release_pending")
        s = json.loads(fpgac.dumps(core.compose_state(NOW + 100 * MS)))
        self.assertEqual(fpgac.kv_blocks(s)["permit"]["state"], "live")
        core.feed_telemetry(telemetry(permit_since=5.0, pl_estop=False, fa=1041), T0 + 200 * MS)
        self.assertEqual(core.compose_state(NOW + 200 * MS)["kv"]["age_basis"], ["geometry", "permit"])

    def test_fpgac_reading_unchanged(self):
        _, s = state(telemetry(), hub_latch=True)
        s = json.loads(fpgac.dumps(s))
        b = fpgac.kv_blocks(s)
        self.assertEqual(b["permit"]["state"], "live")
        self.assertEqual(b["permit"]["age_ms"], PERMIT_SINCE + DELTA)
        self.assertIn("kv_age_ms=%.1f" % (DELTA + GEO), fpgapc.summary(s))


if __name__ == "__main__":
    unittest.main()
