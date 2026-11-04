"""Nav2's global map while mapping (2026-10-29; host/nav_global_map.py, host/ros_nav_global_map.py, car_up
CAR_NAV_LIVE_GLOBAL, navigation.launch.py FPGA_NAV_GLOBAL_MAP_TOPIC).

Covered without ROS:
  * compose: live OCCUPIED cells become lethal in the base's own cells; UNKNOWN / FREE leave the base; a lone cell is
    dropped; cells within 0.35 m of a place the car drove are not written (known distances); a coarser / rotated /
    off-map live grid; the car's swept centre line stays plannable for a 0.30 m inscribed radius
  * SweptTrack: 5 cm thinning; the alignment's per-stamp correction (90 deg known answer, nearest node, out of span)
  * GlobalMapCore: base only / live / stale status / stop (export takes over, never overwritten) / reset (old grid
    dropped, a grid during resetting ignored) / failed plan -> base for the fallback, then live again; no republish
    of an unchanged map
  * the planner log filter; nav_input_profile's switch; the launch sets the global static_layer's map_topic only when
    given; car_up passes CAR_NAV_LIVE_GLOBAL and refuses junk; car_inner starts the node and gives Nav2 the topic only
    in a mapping run with live alignment and the switch on; car_exit_action restarts it
"""
import math
import os
import subprocess
import sys
import unittest

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from host import nav_global_map as ngm  # noqa: E402

E = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def base_grid(n=200, res=0.05):
    """n x n FREE (the blank start map), centred on the origin."""
    return ngm.Grid(np.zeros((n, n), np.int8), res, -n * res / 2, -n * res / 2)


def live_grid(n=100, res=0.05, ox=-2.5, oy=-2.5, yaw=0.0, frame="map"):
    return ngm.Grid(np.full((n, n), -1, np.int8), res, ox, oy, yaw, frame)


def cell_value(grid, data, x, y):
    r, c, inside = grid.world_to_cells(np.array([x]), np.array([y]))
    assert inside[0]
    return int(data[r[0], c[0]])


def yaw_pose7(x, y, yaw):
    return [x, y, 0.0, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]


class Compose(unittest.TestCase):
    def test_wall_written_unknown_and_free_untouched_speckle_dropped(self):
        base, live = base_grid(), live_grid()
        live.data[50, 20:80] = 100            # a 3 m wall at y = 0.025 m (row 50: y in [0, 0.05))
        live.data[60:70, 20:80] = 0           # FREE
        live.data[10, 10] = 100               # a lone speckle
        out, st = ngm.compose(base, live)
        self.assertEqual(st["live_occupied"], 61)
        self.assertEqual(st["speckles_dropped"], 1)
        self.assertEqual(st["marked"], 60)
        self.assertEqual(cell_value(base, out, 0.0, 0.025), 100)
        self.assertEqual(cell_value(base, out, -2.0 + 0.025, -2.0 + 0.025), 0)   # the speckle's cell: not written
        self.assertEqual(cell_value(base, out, 0.0, 0.6), 0)                      # live FREE: base FREE
        self.assertEqual(cell_value(base, out, 2.0, 2.0), 0)                      # live UNKNOWN: base FREE
        self.assertEqual(int((out == 100).sum()), 60)
        self.assertEqual(int((out == -1).sum()), 0)                               # no unknown ever written
        self.assertTrue(np.all(base.data == 0))                                   # the base itself untouched

    def test_base_unknown_and_occupied_kept(self):
        base = base_grid()
        base.data[:, :10] = -1
        base.data[0, 50] = 100
        out, _ = ngm.compose(base, live_grid())
        self.assertTrue(np.array_equal(out, base.data))

    def test_swept_clear_known_distances(self):
        """A wall along y = 0.025 (cell centres); the car drove at the cell centre (0.025, 0.325): wall cells whose
        centres are within 0.35 m are not written (|dx| <= sqrt(0.35^2 - 0.3^2) = 0.180 m: centres -0.125 .. 0.175,
        7 cells), farther ones are. Distances are cell centre to cell centre, as the planner sees them."""
        base, live = base_grid(), live_grid()
        live.data[50, 20:80] = 100
        out, st = ngm.compose(base, live, np.array([[0.025, 0.325]]), 0.35)
        self.assertEqual(st["swept_cleared"], 7)
        self.assertEqual(cell_value(base, out, 0.025, 0.025), 0)
        self.assertEqual(cell_value(base, out, 0.175, 0.025), 0)
        self.assertEqual(cell_value(base, out, -0.125, 0.025), 0)
        self.assertEqual(cell_value(base, out, 0.225, 0.025), 100)
        self.assertEqual(cell_value(base, out, -0.175, 0.025), 100)
        out2, st2 = ngm.compose(base, live, np.array([[0.0, 0.325]]), 0.0)   # clearing off: all written
        self.assertEqual((st2["swept_cleared"], st2["marked"]), (0, 60))
        self.assertEqual(cell_value(base, out2, 0.025, 0.025), 100)

    def test_swept_centre_line_stays_plannable(self):
        """Noise across a corridor the car drove through: no lethal cell within the 0.30 m inscribed radius of any
        swept point (NavFn would treat that centre cell as an obstacle)."""
        base, live = base_grid(), live_grid()
        rng = np.random.default_rng(3)
        live.data[rng.random(live.shape) < 0.25] = 100
        path = np.array([[x, 0.4 * math.sin(x)] for x in np.arange(-2.0, 2.0, 0.05)])
        out, st = ngm.compose(base, live, path, ngm.SWEPT_CLEAR_M)
        self.assertGreater(st["marked"], 500)
        rr, cc = np.nonzero(out == 100)
        cx = base.origin_x + (cc + 0.5) * base.resolution
        cy = base.origin_y + (rr + 0.5) * base.resolution
        for px, py in path:
            # the base cell centre the planner uses for this point
            r, c, _ = base.world_to_cells(np.array([px]), np.array([py]))
            qx, qy = base.origin_x + (c[0] + 0.5) * base.resolution, base.origin_y + (r[0] + 0.5) * base.resolution
            self.assertGreater(float(np.min(np.hypot(cx - qx, cy - qy))), 0.30, (px, py))

    def test_coarser_rotated_and_off_map_grids(self):
        base = base_grid()
        coarse = live_grid(n=50, res=0.1)
        coarse.data[25, 25] = coarse.data[25, 26] = 100      # two 10 cm cells -> 2 x (2 x 2) base cells
        out, st = ngm.compose(base, coarse)
        self.assertEqual(st["marked"], 8)
        rot = live_grid(n=100, ox=0.0, oy=0.0, yaw=math.pi / 2)  # x_world = -v, y_world = u
        rot.data[0, 40:42] = 100                                  # u 2.0 .. 2.1 m, v 0 .. 0.05 m
        out, st = ngm.compose(base, rot)
        self.assertEqual(st["marked"], 2)
        self.assertEqual(cell_value(base, out, -0.025, 2.025), 100)
        far = live_grid(ox=50.0, oy=50.0)
        far.data[10, 10:20] = 100
        out, st = ngm.compose(base, far)
        self.assertEqual((st["marked"], st["outside_base"]), (0, 10))
        self.assertTrue(np.array_equal(out, base.data))


class Swept(unittest.TestCase):
    def test_thinning(self):
        t = ngm.SweptTrack()
        self.assertTrue(t.add(1.0, 0.0, 0.0))
        self.assertFalse(t.add(1.1, 0.03, 0.0))
        self.assertTrue(t.add(1.2, 0.06, 0.0))
        self.assertFalse(t.add(1.3, float("nan"), 0.0))
        self.assertEqual(len(t), 2)
        self.assertTrue(np.allclose(t.map_xy(), [[0.0, 0.0], [0.06, 0.0]]))   # no alignment: map = odom

    def test_alignment_per_stamp(self):
        """Nodes at t = 10 (identity) and t = 20 (90 deg about the origin), map_odom = the last: a point stamped 11
        stays, one stamped 19 turns 90 deg, one far outside the span gets map_odom."""
        t = ngm.SweptTrack()
        t.add(11.0, 1.0, 0.0)
        t.add(19.0, 2.0, 0.0)
        t.add(1e6, 3.0, 0.0)
        doc = {"map_odom": yaw_pose7(0, 0, math.pi / 2), "nodes": [[10.0] + E, [20.0] + yaw_pose7(0, 0, math.pi / 2)]}
        t.set_align(doc)
        self.assertTrue(np.allclose(t.map_xy(), [[1.0, 0.0], [0.0, 2.0], [0.0, 3.0]], atol=1e-9))
        t.set_align({"map_odom": E, "nodes": [[10.0] + E]})      # identity again (a reset)
        self.assertTrue(np.allclose(t.map_xy(), [[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]]))

    def test_matches_map_align_corrections(self):
        rng = np.random.default_rng(1)
        nodes = [[float(s)] + yaw_pose7(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-3, 3))
                 for s in np.arange(100.0, 160.0, 1.0)]
        doc = {"map_odom": nodes[-1][1:], "nodes": nodes}
        t = ngm.SweptTrack(step_m=0.0)
        stamps = list(rng.uniform(60, 200, 50)) + [100.0, 159.0, 99.5, 130.5]
        for i, s in enumerate(stamps):
            t.add(s, 0.1 * i, -0.05 * i)
        t.set_align(doc)
        from host import map_align
        corr = map_align.Corrections(doc)
        want = [(corr.at(s) @ np.array([x, y, 0.0, 1.0]))[:2] for s, (x, y) in zip(t.stamps, t.xy)]
        self.assertTrue(np.allclose(t.map_xy(), want, atol=1e-9))


class Core(unittest.TestCase):
    def setUp(self):
        self.events = []
        self.core = ngm.GlobalMapCore(record=lambda e, **f: self.events.append(dict(f, event=e)))
        self.base = base_grid()
        self.live = live_grid()
        self.live.data[50, 20:80] = 100

    def names(self):
        return [e["event"] for e in self.events]

    def mapping(self, now, session=0, state="mapping"):
        self.core.on_status({"state": state, "session": session}, now)

    def test_live_then_stop_export_takes_over(self):
        c = self.core
        self.assertIsNone(c.tick(0.0))                       # no base yet: nothing
        c.on_base(self.base, "start_map_yaml")
        out = c.tick(0.0)
        self.assertTrue(np.array_equal(out.data, self.base.data))
        self.assertEqual((c.mode, c.reason), ("base", "no_mapping_status"))
        self.mapping(0.5)
        c.on_live(self.live, 0.5)
        out = c.tick(1.0)
        self.assertEqual(c.mode, "live")
        self.assertEqual(int((out.data == 100).sum()), 60)
        self.assertTrue(out.same_geometry(self.base))
        self.assertIsNone(c.tick(1.5))                       # nothing changed: no republish
        c.on_odom(5.0, 3.0, 3.0)                             # far from the wall: same map, no republish
        self.assertIsNone(c.tick(2.0))
        self.mapping(2.5, state="saving")                    # 结束建图
        out = c.tick(2.6)
        self.assertEqual((c.mode, c.reason), ("base", "not_mapping:saving"))
        self.assertTrue(np.array_equal(out.data, self.base.data))
        self.assertIn("live_grid_dropped", self.names())
        export = ngm.Grid(np.full((300, 240), -1, np.int8), 0.05, -6.0, -7.0)
        export.data[100:200, 50:150] = 0
        self.mapping(3.0, state="exported")
        c.on_live(self.live, 3.1)                            # a late grid of the finished map
        self.assertIn("live_grid_ignored", self.names())
        c.on_base(export, "map_server")                      # load_map
        out = c.tick(3.2)
        self.assertTrue(out.same_geometry(export) and np.array_equal(out.data, export.data))
        for now in (4.0, 5.0, 10.0):
            self.mapping(now, state="exported")
            res = c.tick(now)
            self.assertTrue(res is None or np.array_equal(res.data, export.data))
        self.assertTrue(np.array_equal(c.published.data, export.data))

    def test_stale_status_falls_back_and_comes_back(self):
        c = self.core
        c.on_base(self.base, "map_server")
        self.mapping(0.0)
        c.on_live(self.live, 0.0)
        c.tick(0.1)
        self.assertEqual(c.mode, "live")
        out = c.tick(0.1 + ngm.STATUS_MAX_AGE_S + 0.5)
        self.assertEqual((c.mode, c.reason), ("base", "mapping_status_stale"))
        self.assertTrue(np.array_equal(out.data, self.base.data))
        self.mapping(10.0)
        out = c.tick(10.1)
        self.assertEqual(c.mode, "live")                     # the grid was kept (same session)
        self.assertEqual(int((out.data == 100).sum()), 60)

    def test_reset_drops_the_old_grid(self):
        c = self.core
        c.on_base(self.base, "map_server")
        self.mapping(0.0)
        c.on_live(self.live, 0.0)
        c.tick(0.1)
        self.mapping(1.0, state="resetting")
        self.assertFalse(c.on_live(self.live, 1.1))          # the old map's last grid during the reset
        out = c.tick(1.2)
        self.assertEqual(c.reason, "not_mapping:resetting")
        self.mapping(2.0, session=1)
        out = c.tick(2.1)
        self.assertEqual((c.mode, c.reason), ("base", "no_live_grid"))
        self.assertTrue(np.array_equal(out.data, self.base.data))
        fresh = live_grid()
        fresh.data[30, 30:40] = 100
        c.on_live(fresh, 2.5)
        out = c.tick(3.0)
        self.assertEqual((c.mode, int((out.data == 100).sum())), ("live", 10))

    def test_planner_failure_fallback(self):
        c = self.core
        c.on_base(self.base, "map_server")
        self.mapping(0.0)
        c.on_live(self.live, 0.0)
        c.tick(0.1)
        self.assertTrue(c.on_planner_failure(1.0, "GridBased: failed to create plan with tolerance 0.00."))
        self.mapping(1.0)
        out = c.tick(1.0)
        self.assertEqual((c.mode, c.reason), ("base", "planner_failed_fallback"))
        self.assertTrue(np.array_equal(out.data, self.base.data))
        self.assertFalse(c.on_planner_failure(2.0, "again"))   # not the live cells' doing: no extension
        self.mapping(1.0 + ngm.PLANNER_FALLBACK_S - 1)
        self.assertIsNone(c.tick(1.0 + ngm.PLANNER_FALLBACK_S - 1))
        self.mapping(1.0 + ngm.PLANNER_FALLBACK_S + 0.5)
        out = c.tick(1.0 + ngm.PLANNER_FALLBACK_S + 0.5)
        self.assertEqual(c.mode, "live")
        self.assertEqual(int((out.data == 100).sum()), 60)
        self.assertIn("planner_fallback_over", self.names())

    def test_driving_next_to_the_wall_clears_it_there(self):
        c = self.core
        c.on_base(self.base, "map_server")
        self.mapping(0.0)
        c.on_live(self.live, 0.0)
        before = c.tick(0.1)
        self.assertEqual(int((before.data == 100).sum()), 60)
        c.on_odom(1.0, 0.025, 0.325)
        out = c.tick(1.1)
        self.assertEqual(int((out.data == 100).sum()), 53)
        self.assertEqual(c.status(1.1)["swept_cleared"], 7)

    def test_other_frame_ignored(self):
        c = self.core
        c.on_base(self.base, "map_server")
        self.mapping(0.0)
        self.assertFalse(c.on_live(live_grid(frame="odom"), 0.0))
        c.tick(0.1)
        self.assertEqual(c.reason, "no_live_grid")
        self.assertEqual(self.events[-2]["event"], "live_grid_ignored")

    def test_bad_alignment_kept_out(self):
        c = self.core
        c.on_align({"map_odom": E})
        c.on_align({"map_odom": [0, 0, 0, 0, 0, 0, 0], "nodes": []})
        self.assertEqual(self.names(), ["align_ignored", "align_ignored"])
        self.assertIsNone(c.track.align)


class PlannerText(unittest.TestCase):
    def test_filter(self):
        sys.path.insert(0, os.path.join(ROOT, "host"))
        import importlib
        mod = importlib.import_module("host.ros_nav_global_map")
        f = mod.planner_failure_text
        self.assertTrue(f("planner_server", 30, "GridBased: failed to create plan with tolerance 0.00."))
        self.assertTrue(f("planner_server", 30, "Planning algorithm GridBased failed to generate a valid path to "
                                                "(1.00, 2.00)"))
        self.assertTrue(f("planner_server", 40, "GridBased: The goal sent to the planner is off the global costmap."))
        self.assertFalse(f("planner_server", 20, "GridBased: failed to create plan with tolerance 0.00."))
        self.assertFalse(f("controller_server", 30, "Failed to make progress"))
        self.assertFalse(f("planner_server", 30, "Planner loop missed its desired rate of 20.0 Hz"))


class Switches(unittest.TestCase):
    def inputs(self):
        import runpy
        return runpy.run_path(os.path.join(ROOT, "robot_sim/fpga_sim/config/nav_input_profile.py"))

    def test_profile_topic(self):
        i = self.inputs()
        self.assertIsNone(i["global_map_topic"]({}))
        self.assertIsNone(i["global_map_topic"]({"FPGA_NAV_GLOBAL_MAP_TOPIC": " "}))
        self.assertEqual(i["global_map_topic"]({"FPGA_NAV_GLOBAL_MAP_TOPIC": "/fpga/nav/global_map"}),
                         "/fpga/nav/global_map")
        for bad in ("fpga/nav", "/a b"):
            with self.assertRaises(ValueError):
                i["global_map_topic"]({"FPGA_NAV_GLOBAL_MAP_TOPIC": bad})
        params = {"global_costmap": {"global_costmap": {"ros__parameters": {
            "plugins": ["static_layer", "inflation_layer"], "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer"}}}}}
        out = i["apply_global_map_topic"](params, "/fpga/nav/global_map")
        self.assertEqual(out["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"],
                         {"plugin": "nav2_costmap_2d::StaticLayer", "map_topic": "/fpga/nav/global_map"})
        self.assertNotIn("map_topic", params["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"])
        self.assertEqual(i["apply_global_map_topic"](params, None), params)
        with self.assertRaises(ValueError):
            i["apply_global_map_topic"]({"global_costmap": {"global_costmap": {"ros__parameters": {
                "plugins": ["inflation_layer"]}}}}, "/x")

    def test_launch_sets_the_global_static_layer_only_when_given(self):
        sys.path.insert(0, os.path.join(ROOT, "tests"))
        import test_nav_rehang_launch as tnl
        t = tnl.RealCarEntryTests("test_car_entry_real_time_no_sim_transforms_limited_speeds")
        t.setUp()
        try:
            p = t.share / "params/nav2_params.yaml"
            params = yaml.safe_load(p.read_text())
            params["global_costmap"] = {"global_costmap": {"ros__parameters": {
                "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
                "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer", "map_subscribe_transient_local": True}}}}
            params["local_costmap"] = {"local_costmap": {"ros__parameters": {
                "plugins": ["static_layer"], "static_layer": {"plugin": "nav2_costmap_2d::StaticLayer"}}}}
            p.write_text(yaml.safe_dump(params))
            car = dict(tnl.CAR_ENV, FPGA_NAV_MAP_YAML=str(t.share / "maps/car.yaml"))
            _, plain, lines = t.actions(car)
            self.assertNotIn("map_topic", plain["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"])
            self.assertFalse(any("global_map_topic" in line for line in lines))
            _, live, lines = t.actions(dict(car, FPGA_NAV_GLOBAL_MAP_TOPIC="/fpga/nav/global_map"))
            glob = live["global_costmap"]["global_costmap"]["ros__parameters"]
            self.assertEqual(glob["static_layer"]["map_topic"], "/fpga/nav/global_map")
            self.assertEqual(live["local_costmap"], plain["local_costmap"])      # the local costmap untouched
            self.assertEqual(live["map_server"], plain["map_server"])            # /map itself unchanged
            self.assertIn({"global_map_topic": "/fpga/nav/global_map"}, lines)
            self.assertIn("real_car", lines[-1])                                 # the evidence line stays last
            _, sim, _ = t.actions({"FPGA_NAV_GLOBAL_MAP_TOPIC": "/fpga/nav/global_map"})   # host simulation too
            self.assertEqual(sim["global_costmap"]["global_costmap"]["ros__parameters"]["static_layer"]["map_topic"],
                             "/fpga/nav/global_map")
        finally:
            t.doCleanups()


class RunScripts(unittest.TestCase):
    def test_exit_action_restarts_it(self):
        out = subprocess.run(["bash", "-c", "source tools/car_run/car_common.sh; car_exit_action 0 navmap; "
                              "car_exit_action 1 navmap"], cwd=ROOT, stdout=subprocess.PIPE,
                             universal_newlines=True, timeout=30).stdout.split()
        self.assertEqual(out, ["restart", "stop"])

    def test_car_up_passes_the_switch_and_car_inner_wires_it(self):
        sys.path.insert(0, os.path.join(ROOT, "tests"))
        import test_car_run as tcr

        class T(tcr.Base):
            def runTest(self):
                pass
        t = T()
        t.setUp()
        try:
            for val, live in (('', True), ('1', True), ('0', False)):
                env = dict(CAR_ARCH='x86_64', CAR_MAPPING='1', RTABMAP_DB='', CAR_MAP_YAML='')
                if val:
                    env['CAR_NAV_LIVE_GLOBAL'] = val
                r = t.up('sim', '--dry-run', **env)
                self.assertEqual(r.returncode, 0, r.stderr)
                self.assertIn('-e NAV_LIVE_GLOBAL=%s ' % (val or '1'), r.stdout)
                self.check_inner(r.stdout, live)
            # live alignment off: rtabmap's grid is not in Nav2's map frame -> the node is not started
            r = t.up('sim', '--dry-run', CAR_ARCH='x86_64', CAR_MAPPING='1', RTABMAP_DB='', CAR_MAP_YAML='',
                     CAR_LIVE_ALIGN='0')
            self.assertEqual(r.returncode, 0, r.stderr)
            self.check_inner(r.stdout, False)
            # not a mapping run: nothing of it
            r = t.up('sim', '--dry-run', CAR_ARCH='x86_64', RTABMAP_DB='', CAR_MAP_YAML='')
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertNotIn('NAV_LIVE_GLOBAL', r.stdout)
            self.check_inner(r.stdout, False)
            r = t.up('sim', '--dry-run', CAR_ARCH='x86_64', CAR_MAPPING='1', CAR_NAV_LIVE_GLOBAL='yes')
            self.assertEqual(r.returncode, 2)
            self.assertIn('CAR_NAV_LIVE_GLOBAL', r.stderr)
        finally:
            t.tearDown()

    def check_inner(self, out, live):
        """car_inner's dry run (inside car_up's): the navmap launch before Nav2 and Nav2's env, or neither."""
        lines = out.splitlines()
        nav = [l for l in lines if l.strip().startswith('[navigation]')]
        navmap = [i for i, l in enumerate(lines) if l.strip().startswith('[navmap]')]
        self.assertEqual(len(nav), 1, out[-3000:])
        if live:
            self.assertEqual(len(navmap), 1)
            self.assertIn('python3 -m host.ros_nav_global_map --record ', lines[navmap[0]])
            self.assertLess(navmap[0], lines.index(nav[0]))
            self.assertIn(' FPGA_NAV_GLOBAL_MAP_TOPIC=/fpga/nav/global_map ', nav[0])
        else:
            self.assertEqual(navmap, [])
            self.assertNotIn('FPGA_NAV_GLOBAL_MAP_TOPIC', nav[0])


if __name__ == "__main__":
    unittest.main()
