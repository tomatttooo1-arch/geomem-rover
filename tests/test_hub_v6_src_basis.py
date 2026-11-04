"""Integration v6 (2026-10-12) controller ruling 1: ROS group ``src`` falls back to the runner's latched
/fpga/runtime_info ``sensor_source`` when the volatile /fpga/status ``startup`` event never reached the hub
(hub joined after the runner), and every ROS group says which input decided: ``src_basis`` =
"startup_event" | "runtime_info" | null.

1. Late hub: runtime_info only (replay, hub --data-source replay) -> every ROS group with input is
   src replay / src_basis runtime_info; EVENT, MEMORY frames follow; the module before v6 (git 4a247b6)
   fed the same inputs keeps every ROS group unknown (old-behaviour control).
2. Startup wins whenever present: arrives after runtime_info -> basis flips to startup_event; a
   disagreeing startup with an agreeing runtime_info stays unknown (startup_event).
3. Mapping (same table as the v5 runner-vs-producer check): hardware->live, replay/simulation/sim->replay;
   disagreeing or unmapped value -> unknown (basis runtime_info); body without sensor_source or no
   runtime_info at all -> unknown, basis null.
4. Shape: groups without input stay src unavailable, src_basis null; non-ROS groups (kv, estop, clock,
   nano, hub) have no src_basis; fpgac's C.6 flag check accepts the STATE; frame-level ``source`` is not
   changed by runtime_info (it stays the startup/producer check; the runner-vs-producer disagreement
   is the separate runner_source_warning).
Python 3.8+, no ROS.
"""
import json
from pathlib import Path
import sys
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from pc import fpgac  # noqa: E402
from host import ros_runtime  # noqa: E402
from host.hub.hub_core import HubCore, NS, MS, ROS_GROUPS  # noqa: E402

PRE_V6 = "4a247b6"
ARGS = types.SimpleNamespace(sensor_source="replay", pose_source="reference", inference_device="cpu",
                             geometry_fallback="none", region_source="none", tf_reliable=False, stall_diag=None)


def info(sensor="replay", boot="rt-1"):
    a = types.SimpleNamespace(**dict(vars(ARGS), sensor_source=sensor))
    return json.dumps(ros_runtime.runtime_info_body(a, boot, "kv_topic", 5, 200.0, version={"commit": None}))


def feed_ros_inputs(core, t=NS):
    """One input for each ROS group that this test drives (task, memory, nav, geometry_nano/perf)."""
    core.feed_status(json.dumps({"event": "task", "state": "idle", "processed_frames": 1}), t)
    core.feed_memory(json.dumps({"revision": 1, "tracks": []}), t)
    core.feed_motion_permit(False, t)
    core.feed_frame_stats(json.dumps({"v": 1, "kv_frame_id": 0, "source": "replay"}), t)


FED = ("task", "memory", "nav", "geometry_nano", "perf")


def startup(sensor="replay"):
    return json.dumps({"event": "startup", "sensor_source": sensor})


class LateHub(unittest.TestCase):
    def test_runtime_info_decides_without_startup(self):
        core = HubCore("replay")
        core.feed_runtime_info(info("replay"), NS)
        feed_ros_inputs(core)
        s = core.compose_state(NS + 10 * MS)
        for g in FED:
            self.assertEqual((s[g]["src"], s[g]["src_basis"]), ("replay", "runtime_info"), g)
        self.assertEqual(s["source"], "replay")
        core.feed_status(json.dumps({"event": "query_rejected", "reason": "x"}), NS + 11 * MS)
        evs = core.pop_events(NS + 12 * MS)
        self.assertEqual([e["src"] for e in evs], ["replay"])
        self.assertEqual(core.memory_body(NS + 12 * MS, force=True)["src"], "replay")
        self.assertIsNone(fpgac.check_source_flags(fpgac.KIND["STATE"], s))

    def test_old_module_stays_unknown(self):
        import test_hub_v5_rulings as v5
        old = v5.old_module("host/hub/hub_core.py", "hub_core_pre_v6", commit=PRE_V6)
        if old is None:
            self.skipTest("needs git history (%s)" % PRE_V6)
        core = old.HubCore("replay")
        core.feed_runtime_info(info("replay"), NS)
        feed_ros_inputs(core)
        s = core.compose_state(NS + 10 * MS)
        for g in FED:
            self.assertEqual(s[g]["src"], "unknown", g)
            self.assertNotIn("src_basis", s[g])
        # and the same module with the startup event: replay (the inputs above are sufficient)
        core.feed_status(startup("replay"), NS + 20 * MS)
        self.assertEqual(core.compose_state(NS + 30 * MS)["task"]["src"], "replay")


class StartupWins(unittest.TestCase):
    def test_startup_after_runtime_info_flips_basis(self):
        core = HubCore("replay")
        core.feed_runtime_info(info("replay"), NS)
        feed_ros_inputs(core)
        self.assertEqual(core.compose_state(NS)["task"]["src_basis"], "runtime_info")
        core.feed_status(startup("replay"), NS + MS)
        s = core.compose_state(NS + 2 * MS)
        for g in FED:
            self.assertEqual((s[g]["src"], s[g]["src_basis"]), ("replay", "startup_event"), g)

    def test_disagreeing_startup_is_not_overridden(self):
        core = HubCore("replay")
        core.feed_status(startup("hardware"), NS)         # startup says live, hub replay
        core.feed_runtime_info(info("replay"), NS)        # runtime_info agrees -- does not override
        feed_ros_inputs(core)
        s = core.compose_state(NS + MS)
        for g in FED:
            self.assertEqual((s[g]["src"], s[g]["src_basis"]), ("unknown", "startup_event"), g)
        self.assertEqual(s["source"], "inconsistent")     # unchanged B.3 frame-level rule

    def test_agreeing_startup_disagreeing_runtime_info(self):
        core = HubCore("replay")
        core.feed_status(startup("replay"), NS)
        core.feed_runtime_info(info("hardware"), NS)
        feed_ros_inputs(core)
        s = core.compose_state(NS + MS)
        self.assertEqual((s["task"]["src"], s["task"]["src_basis"]), ("replay", "startup_event"))
        self.assertEqual(s["geometry_nano"]["runner_source_warning"], "runner_producer_mismatch")   # v5, unchanged


class Mapping(unittest.TestCase):
    def state(self, data_source, body):
        core = HubCore(data_source)
        if body is not None:
            core.feed_runtime_info(body, NS)
        feed_ros_inputs(core)
        return core.compose_state(NS + MS)

    def test_table(self):
        cases = [("live", info("hardware"), ("live", "runtime_info")),
                 ("replay", info("simulation"), ("replay", "runtime_info")),
                 ("replay", info("sim"), ("replay", "runtime_info")),
                 ("replay", info("hardware"), ("unknown", "runtime_info")),
                 ("live", info("replay"), ("unknown", "runtime_info")),
                 ("replay", info("bogus"), ("unknown", "runtime_info")),
                 ("replay", json.dumps({"v": 1}), ("unknown", None)),
                 ("replay", None, ("unknown", None))]
        for ds, body, want in cases:
            s = self.state(ds, body)
            self.assertEqual((s["task"]["src"], s["task"]["src_basis"]), want, (ds, body and json.loads(body).get("sensor_source")))

    def test_frame_source_not_driven_by_runtime_info(self):
        s = self.state("replay", info("hardware"))       # no startup: frame-level stays the old rule
        self.assertEqual(s["source"], "replay")
        self.assertEqual(s["task"]["src"], "unknown")


class Shape(unittest.TestCase):
    def test_unavailable_and_non_ros_groups(self):
        core = HubCore("replay")
        core.feed_runtime_info(info("replay"), NS)
        s = core.compose_state(NS)
        for g in ROS_GROUPS:
            if s[g]["src"] == "unavailable":
                self.assertIsNone(s[g]["src_basis"], g)
            else:
                self.assertIn("src_basis", s[g], g)
        self.assertEqual(s["region"]["src"], "unavailable")
        for g in ("kv", "estop", "clock", "nano", "hub"):
            self.assertNotIn("src_basis", s[g], g)


if __name__ == "__main__":
    unittest.main()
