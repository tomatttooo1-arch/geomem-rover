"""PLNCHAIN0 (2026-11-04) host end-to-end: PLN=gmap3 inside the real kv_nav core (libkv_nav via host.kv_nav).

The planner core is the GMAP3 v9 kernel C source on a mock bus (hw/planner/drv_gmap3_chain `make mocklib`:
libplanner_drv_gmap3_mock.so, same shim as the board library, only planner_g3_open differs). World = the CTRLPS3
test world (tests/kv_nav_real/world.json), a unicycle integrates the published (v, w) for ~8 s in real time.

1. KV_NAV_PLN=gmap3: the field hook is the GMAP3 one; path fetch calls grow with the replans, every core path ==
   the PS extraction on the same field (mismatch 0); cost windows are MERGEd into the global map, SCAN runs;
   StampedNavCore field rows carry "pln".  The car moves toward the goal (the field the core built is used).
2. KV_NAV_PLN=gmap3 with a missing library -> WARN, m5 driver tried (no hardware on the host -> soft field):
   the chain still plans (FPGA rule: never refuse).
Needs tools/kv_nav/build/host/libkv_nav.so and PLNCHAIN0_MOCKLIB (default: the work dir build).
Run: python3 tests/test_plnchain0_gmap3_20261104.py -v   (not "-m unittest tests.…": a site-packages "tests" shadows this dir here)
"""
import io
import json
import math
import os
import sys
import time
import unittest
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
WORLD = os.path.join(ROOT, "tests/kv_nav_real/world.json")
MOCKLIB = os.environ.get("PLNCHAIN0_MOCKLIB", os.path.join(
    ROOT, "hw/planner/drv_gmap3_chain/build/host/libplanner_drv_gmap3_mock.so"))
GOAL = (1.0, 1.2, 1.57)


def drive(core, seconds=8.0, stamps=None):
    from host import kv_local_frames as F
    x = y = yaw = 0.0
    v = w = 0.0
    nav = core
    if stamps is not None:
        nav = F.StampedNavCore(core, lambda: None, stamps)
    nav.set_goal(7, *GOAL)
    t0 = time.monotonic()
    last = t0
    while time.monotonic() - t0 < seconds:
        now = time.monotonic()
        dt = now - last
        last = now
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        nav.on_odom({"odom_pose": (x, y, yaw), "v": v, "w": w, "permit": True, "map_odom": (0.0, 0.0, 0.0),
                     "odom_valid": True})
        out = nav.step(now)
        v, w = out.get("v", 0.0), out.get("w", 0.0)
        time.sleep(0.05)
    return (x, y), out


class Rows:
    def __init__(self):
        self.rows = []

    def write(self, r):
        self.rows.append(r)


@unittest.skipUnless(os.path.exists(MOCKLIB), "no mock library %s (make -C hw/planner/drv_gmap3_chain mocklib)" % MOCKLIB)
class PlnChain0(unittest.TestCase):
    def test_gmap3_in_kv_nav(self):
        from host import kv_nav
        os.environ["KV_NAV_PLN"] = "gmap3"
        os.environ["KV_NAV_PL_DRIVER_GMAP3"] = MOCKLIB
        os.environ["KV_NAV_PLN_SCAN_S"] = "2"
        buf = io.StringIO()
        with redirect_stdout(buf):
            core = kv_nav.make_core(field="pl", grid_source=kv_nav.SimWorldGrid(WORLD))
        first = json.loads(buf.getvalue().splitlines()[0])
        self.assertEqual(first.get("pln"), "gmap3", buf.getvalue())
        self.assertIn('"gmap_enabled":true', first["driver"])
        s0 = core.pln_stats()
        rows = Rows()
        (x, y), out = drive(core, stamps=rows)
        s = core.pln_stats()
        print("\npln stats:", json.dumps(s), "\nmoved to", (round(x, 3), round(y, 3)), "state", out.get("state"))
        self.assertGreaterEqual(s["path_calls"], 3)
        self.assertGreater(s["path_calls"], s0["path_calls"])
        self.assertEqual(s["path_mismatch"], 0)
        self.assertEqual(s["path_match"], s["field_ok"])     # every ready field: core path == PS extraction
        self.assertEqual(s["field_fail"], 0)
        self.assertGreater(s["merge_ok"], 3)
        self.assertEqual(s["merge_fail"], 0)
        self.assertGreater(s["scan_ok"], 0)
        self.assertGreater(math.hypot(x, y), 0.2)             # it drives on the PL field
        self.assertEqual(core.describe()["pln"]["path_calls"], s["path_calls"])
        frows = [r for r in rows.rows if r.get("ev") == "field"]
        self.assertTrue(frows and all("pln" in r for r in frows))
        pc = [r["pln"]["path_calls"] for r in frows]
        self.assertEqual(pc, sorted(pc))
        self.assertGreater(pc[-1], pc[0])

    def test_gmap3_missing_falls_back(self):
        from host import kv_nav
        os.environ["KV_NAV_PLN"] = "gmap3"
        os.environ["KV_NAV_PL_DRIVER_GMAP3"] = "/nonexistent/libplanner_drv_gmap3.so"
        buf = io.StringIO()
        with redirect_stdout(buf):
            core = kv_nav.make_core(field="pl", grid_source=kv_nav.SimWorldGrid(WORLD))
        lines = [json.loads(l) for l in buf.getvalue().splitlines()]
        self.assertIn("KV_NAV_PLN=gmap3", lines[0].get("warning", ""))
        self.assertIsNone(core.pln_stats())
        (x, y), out = drive(core, seconds=4.0)
        self.assertGreater(math.hypot(x, y), 0.1)

    def tearDown(self):
        for k in ("KV_NAV_PLN", "KV_NAV_PL_DRIVER_GMAP3", "KV_NAV_PLN_SCAN_S"):
            os.environ.pop(k, None)


if __name__ == "__main__":
    unittest.main()
