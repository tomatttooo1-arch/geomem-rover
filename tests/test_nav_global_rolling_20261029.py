"""10-29: the rolling local layer's obstacles go into Nav2's global map too (host/nav_global_map.py stamp_rolling,
GlobalMapCore.on_rolling / on_rolling_tf; car_up CAR_NAV_GLOBAL_ROLLING). Host sim before: the global map lacked a
wall's end the local layer had, NavFn planned straight over it, the car stopped at its inscribed edge and turned on
the spot until the goal failed.

Known answers: identity lattice -> exactly the rolling cells; a half-cell offset -> each cell covers its 2 x 2 map
cells (never less than the local layer, at most one cell more); a 90 deg turn; off-map cells skipped. Property: for
random map <- odom transforms every rolling cell's centre lands on a lethal map cell and every lethal map cell lies
within one cell of a rolling cell. Core: rolling cells in base mode and in live mode, none without a transform, moved
with a new transform, gone when the local layer drops them, out during the planner fallback (which they now trigger
too), back after it. car_up passes CAR_NAV_GLOBAL_ROLLING (default 1, junk refused); car_inner turns the node's
rolling topic off only for 0.

    PYTHONPATH=. python3 tests/test_nav_global_rolling_20261029.py
"""
import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tests"))

from host import nav_global_map as ngm  # noqa: E402


def base_grid(n=200, res=0.05):
    return ngm.Grid(np.zeros((n, n), np.int8), res, -n * res / 2, -n * res / 2)


def rolling_grid(n=80, res=0.05, ox=-2.0, oy=-2.0, frame="odom"):
    """The local layer's 4 m window (odom frame), all FREE."""
    return ngm.Grid(np.zeros((n, n), np.int8), res, ox, oy, 0.0, frame)


def lethal_xy(grid, data):
    r, c = np.nonzero(data == ngm.LETHAL)
    return grid.cells_to_world(r, c, 1)


class Stamp(unittest.TestCase):
    def test_identity_lattice_exact(self):
        base, roll = base_grid(), rolling_grid()
        roll.data[40, 30:35] = 100          # 5 cells at y = 0.025, x = -0.475 .. -0.275
        roll.data[10, 10] = -1              # unknown / free never written
        out = base.data.copy()
        self.assertEqual(ngm.stamp_rolling(out, base, roll, (0.0, 0.0, 0.0)), 5)
        x, y = lethal_xy(base, out)
        self.assertEqual(sorted(np.round(x, 3)), [-0.475, -0.425, -0.375, -0.325, -0.275])
        self.assertTrue(np.allclose(y, 0.025))
        self.assertEqual(ngm.stamp_rolling(out, base, roll, (0.0, 0.0, 0.0)), 0)    # already lethal

    def test_half_cell_offset_covers_two_by_two(self):
        base, roll = base_grid(), rolling_grid()
        roll.data[40, 40] = 100             # odom centre (0.025, 0.025)
        out = base.data.copy()
        ngm.stamp_rolling(out, base, roll, (0.025, 0.025, 0.0))     # cell spans map 0.025..0.075 both axes
        x, y = lethal_xy(base, out)
        self.assertEqual(sorted(zip(np.round(x, 3), np.round(y, 3))),
                         [(0.025, 0.025), (0.025, 0.075), (0.075, 0.025), (0.075, 0.075)])

    def test_quarter_turn(self):
        base, roll = base_grid(), rolling_grid()
        roll.data[40, 50] = 100             # odom (0.525, 0.025) -> map (-0.025, 0.525) under +90 deg
        out = base.data.copy()
        ngm.stamp_rolling(out, base, roll, (0.0, 0.0, math.pi / 2))
        x, y = lethal_xy(base, out)
        self.assertEqual(list(zip(np.round(x, 3), np.round(y, 3))), [(-0.025, 0.525)])

    def test_off_map_skipped(self):
        base, roll = base_grid(n=40), rolling_grid()
        roll.data[0, 0] = roll.data[40, 40] = 100
        out = base.data.copy()
        self.assertEqual(ngm.stamp_rolling(out, base, roll, (0.0, 0.0, 0.0)), 1)

    def test_random_transforms_cover_and_stay_within_a_cell(self):
        rng = np.random.default_rng(29)
        base = base_grid()
        for _ in range(30):
            roll = rolling_grid()
            roll.data[rng.integers(0, 80, 40), rng.integers(0, 80, 40)] = 100
            tf = (rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-math.pi, math.pi))
            out = base.data.copy()
            ngm.stamp_rolling(out, base, roll, tf)
            r, c = np.nonzero(roll.data == 100)
            gx, gy = roll.cells_to_world(r, c, 1)
            cs, sn = math.cos(tf[2]), math.sin(tf[2])
            mx, my = tf[0] + cs * gx - sn * gy, tf[1] + sn * gx + cs * gy
            br, bc, inside = base.world_to_cells(mx, my)
            self.assertTrue(inside.all())
            self.assertTrue((out[br, bc] == ngm.LETHAL).all())                # never less than the local layer
            lx, ly = lethal_xy(base, out)
            d = np.hypot(lx[:, None] - mx[None, :], ly[:, None] - my[None, :]).min(axis=1)
            self.assertLessEqual(d.max(), 0.05 * math.sqrt(2) + 1e-9)        # at most a cell more


class Core(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.core = ngm.GlobalMapCore(record=lambda e, **f: self.events.append(dict(f, event=e)))
        self.base = base_grid()
        self.roll = rolling_grid()
        self.roll.data[40, 30:35] = 100

    def names(self):
        return [e["event"] for e in self.events]

    def test_base_mode_gets_the_rolling_cells_with_a_transform_only(self):
        c = self.core
        c.on_base(self.base, "map_server")
        c.tick(0.0)
        c.on_rolling(self.roll)
        self.assertIsNone(c.tick(0.5))                                   # no transform yet: the same base map
        c.on_rolling_tf((0.0, 0.0, 0.0))
        out = c.tick(1.0)
        self.assertEqual(int((out.data == ngm.LETHAL).sum()), 5)
        self.assertEqual((c.mode, c.reason), ("base", "no_mapping_status"))
        self.assertEqual(c.status(1.0)["rolling_marked"], 5)
        self.assertIsNone(c.tick(1.5))                                   # unchanged: no republish
        c.on_rolling_tf((0.001, 0.0, 0.0))                               # below the 5 mm step: same map
        self.assertIsNone(c.tick(2.0))
        c.on_rolling_tf((1.0, 0.0, 0.0))                                 # a map <- odom correction moves them
        out = c.tick(2.5)
        x, _ = lethal_xy(self.base, out.data)
        self.assertAlmostEqual(float(x.min()), 0.525, places=3)
        cleared = rolling_grid()
        c.on_rolling(cleared)                                            # the local layer cleared them
        out = c.tick(3.0)
        self.assertEqual(int((out.data == ngm.LETHAL).sum()), 0)
        c.on_rolling(self.roll)
        c.on_rolling_tf(None)                                            # TF lost: out again, logged
        out = c.tick(3.5)
        self.assertIsNone(out)                                           # already the plain map
        self.assertIn("rolling_tf_lost", self.names())
        self.assertTrue(np.array_equal(c.published.data, self.base.data))

    def test_live_mode_adds_them_to_rtabmaps_cells(self):
        c = self.core
        c.on_base(self.base, "map_server")
        c.on_status({"state": "mapping", "session": 0}, 0.0)
        live = ngm.Grid(np.full((100, 100), -1, np.int8), 0.05, -2.5, -2.5)
        live.data[80, 20:80] = 100
        c.on_live(live, 0.0)
        c.on_rolling(self.roll)
        c.on_rolling_tf((0.0, 0.0, 0.0))
        out = c.tick(0.5)
        self.assertEqual(c.mode, "live")
        self.assertEqual(int((out.data == ngm.LETHAL).sum()), 65)

    def test_planner_failure_in_base_mode_falls_back_without_them(self):
        c = self.core
        c.on_base(self.base, "map_server")
        c.tick(0.0)
        self.assertFalse(c.on_planner_failure(0.1, "failed"))           # plain map: nothing to fall back from
        c.on_rolling(self.roll)
        c.on_rolling_tf((0.0, 0.0, 0.0))
        c.tick(0.5)
        self.assertTrue(c.on_planner_failure(1.0, "GridBased: failed to create plan with tolerance 0.00."))
        out = c.tick(1.0)
        self.assertEqual((c.mode, c.reason), ("base", "planner_failed_fallback"))
        self.assertTrue(np.array_equal(out.data, self.base.data))
        self.assertFalse(c.on_planner_failure(2.0, "again"))
        out = c.tick(1.0 + ngm.PLANNER_FALLBACK_S + 0.5)
        self.assertEqual(int((out.data == ngm.LETHAL).sum()), 5)
        self.assertIn("planner_fallback_over", self.names())


class RunScripts(unittest.TestCase):
    def test_car_up_passes_the_switch_and_car_inner_wires_it(self):
        import test_car_run as tcr

        class T(tcr.Base):
            def runTest(self):
                pass
        t = T()
        t.setUp()
        try:
            for val, on in (('', True), ('1', True), ('0', False)):
                env = dict(CAR_ARCH='x86_64', CAR_MAPPING='1', RTABMAP_DB='', CAR_MAP_YAML='')
                if val:
                    env['CAR_NAV_GLOBAL_ROLLING'] = val
                r = t.up('sim', '--dry-run', **env)
                self.assertEqual(r.returncode, 0, r.stderr)
                self.assertIn('-e NAV_GLOBAL_ROLLING=%s ' % (val or '1'), r.stdout)
                navmap = [l for l in r.stdout.splitlines() if l.strip().startswith('[navmap]')]
                self.assertEqual(len(navmap), 1, r.stdout[-3000:])
                self.assertIn('python3 -m host.ros_nav_global_map --record ', navmap[0])
                self.assertEqual('--rolling-topic' in navmap[0], not on, navmap[0])
            r = t.up('sim', '--dry-run', CAR_ARCH='x86_64', CAR_MAPPING='1', CAR_NAV_GLOBAL_ROLLING='yes')
            self.assertEqual(r.returncode, 2)
            self.assertIn('CAR_NAV_GLOBAL_ROLLING', r.stderr)
        finally:
            t.tearDown()


if __name__ == "__main__":
    unittest.main()
