"""Front-sector seed in the rolling bridge (2026-10-26, DEFAULT OFF; car_up CAR_FRONT_SEED_M).

The camera's nearest visible ground is ~0.64-0.73 m from the body centre, the body seed ends at 0.35 m: after an
in-place turn to an unseen heading DWB sees UNKNOWN ahead and dithers. On: UNKNOWN cells within R (closest cell
point) of the body centre and within +-35 deg (cell-centre bearing) of the CURRENT heading are sent FREE; OCCUPIED and
FREE never change; nothing is kept after the body moves/turns. Off (None / 0): cell-for-cell the old grid.
Known-answer checks on hand-built grids; wiring checks on car_up --dry-run / a stand-in real start. Python 3.8.
"""
import json
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host import rolling_nav_layer as L   # noqa: E402
from host.m7_rolling import FREE, OCCUPIED, UNKNOWN   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V = 0.05
DIMS = (80, 80, 32)        # window 4 m x 4 m, base (-40, -40) -> odom x, y in [-2, 2)
BASE = (-40, -40, 0)
R_BODY, MARGIN = 0.30, 0.05


def read(*parts):
    with open(os.path.join(ROOT, *parts)) as f:
        return f.read()


def bridge(front_seed_m=None):
    band = L.HeightBand(ground_z_m=0.0, robot_height_m=0.31)
    kw = {} if front_seed_m == 'absent' else {'front_seed_m': front_seed_m}
    return L.RollingLayerBridge(dims=DIMS, voxel_m=V, band=band, clock_id='steady', body_radius_m=R_BODY,
                                body_margin_m=MARGIN, **kw)


def install(b, tri):
    """Put a hand-built tri-state 2-D map [ly, lx] into the bridge (as strict_grid would project it)."""
    b.mirror.window_base = BASE
    b.mirror.states = np.zeros((1, 1, 1), np.uint8)   # not None: the bridge has a map
    b.mirror.needs_resync = False
    b.mirror.epoch, b.mirror.delta_seq = 1, 0
    b.tri2d = lambda: np.array(tri, np.uint8, copy=True)
    b._grid = None
    return b


def centres():
    c = (np.arange(80) + BASE[0] + 0.5) * V
    return np.meshgrid(c, c)   # X[ly, lx], Y[ly, lx]


def ref_sector(x, y, yaw, r, deg):
    """Independent reference: closest-point distance <= r and centre bearing within deg."""
    out = np.zeros((80, 80), bool)
    for ly in range(80):
        for lx in range(80):
            x0, y0 = (BASE[0] + lx) * V, (BASE[1] + ly) * V
            dx = max(x0 - x, x - (x0 + V), 0.0)
            dy = max(y0 - y, y - (y0 + V), 0.0)
            cx, cy = x0 + V / 2 - x, y0 + V / 2 - y
            b = math.degrees(abs(math.atan2(math.sin(math.atan2(cy, cx) - yaw), math.cos(math.atan2(cy, cx) - yaw))))
            out[ly, lx] = dx * dx + dy * dy <= r * r and b <= deg + 1e-9
    return out


def grid2d(b, pose):
    return b.grid(body_xy=pose[:2], body_yaw=pose[2]).data.reshape(80, 80)


class TestBridgeSector(unittest.TestCase):
    def setUp(self):
        X, Y = centres()
        tri = np.full((80, 80), UNKNOWN, np.uint8)
        tri[X < -1.0] = FREE                                    # a seen strip far behind
        self.obst = (np.abs(X - 0.45) < 0.03) & (np.abs(Y - 0.025) < 0.03)   # an obstacle cell inside the sector
        tri[self.obst] = OCCUPIED
        self.free_in = (np.abs(X - 0.575) < 0.01) & (np.abs(Y - 0.075) < 0.01)  # an already FREE cell in the sector
        tri[self.free_in] = FREE
        self.tri = tri

    def test_off_is_cell_for_cell_the_old_grid(self):
        for pose in ((0.0, 0.0, 0.0), (0.12, -0.31, 2.9), (-0.4, 0.7, -1.2)):
            old = install(bridge('absent'), self.tri).grid(body_xy=pose[:2]).data
            for off in (None, 0, 0.0):
                b = install(bridge(off), self.tri)
                self.assertTrue(np.array_equal(b.grid(body_xy=pose[:2], body_yaw=pose[2]).data, old), (pose, off))
                self.assertEqual(b.counts['front_seeded'], 0)

    def test_known_answer_yaw0(self):
        pose = (0.0, 0.0, 0.0)
        off = grid2d(install(bridge(None), self.tri), pose)
        on = grid2d(install(bridge(0.65), self.tri), pose)
        sector = ref_sector(pose[0], pose[1], pose[2], 0.65, 35.0)
        body = L.body_cells(BASE, DIMS, V, pose[:2], R_BODY + MARGIN)
        changed = on != off
        # exactly: UNKNOWN cells of the sector outside the body seed went FREE, nothing else changed
        expect = sector & ~body & (self.tri == UNKNOWN)
        self.assertGreater(int(expect.sum()), 20)
        self.assertTrue(np.array_equal(changed, expect))
        self.assertTrue(np.all(on[expect] == L.GRID_FREE))
        self.assertTrue(np.all(on[self.obst] == L.GRID_LETHAL))          # obstacle untouched
        self.assertTrue(np.all(on[self.free_in] == L.GRID_FREE))
        # concrete cells: (0.575, 0.025) ahead -> FREE; (-0.575, 0.025) behind, (0.025, 0.575) beside -> UNKNOWN
        X, Y = centres()
        at = lambda x, y: (np.abs(X - x) < 0.01) & (np.abs(Y - y) < 0.01)   # noqa: E731
        self.assertEqual(int(on[at(0.575, 0.025)][0]), L.GRID_FREE)
        self.assertEqual(int(off[at(0.575, 0.025)][0]), L.GRID_UNKNOWN)   # positive control: off keeps it unknown
        self.assertEqual(int(on[at(-0.575, 0.025)][0]), L.GRID_UNKNOWN)
        self.assertEqual(int(on[at(0.025, 0.575)][0]), L.GRID_UNKNOWN)
        self.assertEqual(int(on[at(0.775, 0.025)][0]), L.GRID_UNKNOWN)   # beyond R
        # 0.50 m: a smaller sector, a subset of the 0.65 one
        on50 = grid2d(install(bridge(0.50), self.tri), pose)
        ch50 = on50 != off
        self.assertTrue(np.array_equal(ch50, ref_sector(0, 0, 0, 0.50, 35.0) & ~body & (self.tri == UNKNOWN)))
        self.assertTrue(np.all(changed[ch50]))
        self.assertLess(int(ch50.sum()), int(changed.sum()))

    def test_heading_across_the_wrap_and_moving_keeps_nothing(self):
        b = install(bridge(0.65), self.tri)
        off = install(bridge(None), self.tri)
        for pose in ((0.1, -0.2, math.pi - 0.05), (0.1, -0.2, -math.pi + 0.05), (-0.3, 0.4, 1.9), (0.0, 0.0, -2.4)):
            on, ref = grid2d(b, pose), grid2d(off, pose)
            body = L.body_cells(BASE, DIMS, V, pose[:2], R_BODY + MARGIN)
            expect = ref_sector(pose[0], pose[1], pose[2], 0.65, 35.0) & ~body & (self.tri == UNKNOWN)
            self.assertGreater(int(expect.sum()), 20, pose)
            self.assertTrue(np.array_equal(on != ref, expect), pose)   # only this pose's sector: nothing kept
            self.assertTrue(np.all(on[self.tri == OCCUPIED] == L.GRID_LETHAL))
        # turned 180 deg in place: the cells ahead before are UNKNOWN again
        a = grid2d(b, (0.0, 0.0, 0.0))
        c = grid2d(b, (0.0, 0.0, math.pi))
        X, Y = centres()
        ahead = (np.abs(X - 0.575) < 0.01) & (np.abs(Y - 0.025) < 0.01)
        self.assertEqual(int(a[ahead][0]), L.GRID_FREE)
        self.assertEqual(int(c[ahead][0]), L.GRID_UNKNOWN)

    def test_no_yaw_seeds_body_only_and_validation(self):
        b = install(bridge(0.65), self.tri)
        old = install(bridge(None), self.tri).grid(body_xy=(0.0, 0.0)).data
        self.assertTrue(np.array_equal(b.grid(body_xy=(0.0, 0.0)).data, old))
        self.assertEqual(b.counts['front_seed_no_yaw'], 1)
        for bad in (-0.1, float('nan'), float('inf'), True, '0.65'):
            with self.assertRaises(ValueError):
                bridge(bad)
        with self.assertRaises(ValueError):
            L.RollingLayerBridge(dims=DIMS, voxel_m=V, band=L.HeightBand(ground_z_m=0.0, robot_height_m=0.31),
                                 clock_id='steady', body_radius_m=R_BODY, front_seed_m=0.65, front_seed_deg=0)
        self.assertEqual(L.FRONT_SEED_DEG, 35.0)
        self.assertEqual(L.BODY_MARGIN_MAX_M, 0.05)   # the owner's body-margin cap is not touched


class TestNodeWiring(unittest.TestCase):
    def test_node_declares_and_passes_yaw(self):
        node = read('host', 'ros_rolling_nav.py')
        self.assertIn('("front_seed_m", 0.0)', node)
        self.assertIn('front_seed_m=p("front_seed_m")', node)
        self.assertIn('body_yaw=None if pose is None else pose[2]', node)
        inner = read('tools', 'car_run', 'car_inner.sh')
        self.assertIn('-p body_margin_m:=0.05 ${FRONT_SEED_M:+-p front_seed_m:=$FRONT_SEED_M}\n', inner)


try:
    from tests import test_car_run as tcr
except ImportError:   # run from inside tests/
    import test_car_run as tcr


class TestCarUpFrontSeed(tcr.Base):
    setUp = tcr.TestLiveBoard.setUp
    line = tcr.TestLiveBoard.line
    make_node = tcr.TestLiveBoard.make_node

    def test_dry_run_off_and_on(self):
        r = self.up('live', '--dry-run', FAKE_USB='1')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('FRONT_SEED_M', self.line(r.stdout, '[container stack]'))
        br = self.line(r.stdout, '    [bridge]')
        self.assertNotIn('front_seed', br)
        self.assertIn('-p body_margin_m:=0.05 > ', br)   # off: the launch line of before
        r = self.up('live', '--dry-run', FAKE_USB='1', CAR_FRONT_SEED_M='0.65')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('-e FRONT_SEED_M=0.65 ', self.line(r.stdout, '[container stack]'))
        self.assertIn('-p body_margin_m:=0.05 -p front_seed_m:=0.65 > ', self.line(r.stdout, '    [bridge]'))
        for bad in ('0', '0.0', '-0.5', 'x', '1.5', '1e-1'):
            run = os.path.join(self.tmp, 'bad_seed')
            r = self.up('live', '--run-dir', run, FAKE_USB='1', CAR_FRONT_SEED_M=bad)
            self.assertEqual(r.returncode, 2, (bad, r.stdout))
            self.assertIn('CAR_FRONT_SEED_M 须为空', r.stderr)
            self.assertFalse(os.path.exists(run))

    def test_car_json_records_effective_value(self):
        self.make_node()
        port = os.path.join(self.tmp, 'fpga_chassis')
        open(port, 'w').close()
        for name, env, want in (('fs0', {}, {'enabled': False, 'm': None}),
                                ('fs1', {'CAR_FRONT_SEED_M': '0.50'}, {'enabled': True, 'm': 0.5})):
            run = os.path.join(self.tmp, 'runs', name)
            r = self.up('live', '--pose', 'odom', '--nav', 'costmap', '--run-dir', run, FAKE_USB='1', RTABMAP_DB='',
                        CHASSIS_PORT=port, **env)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            fs = json.loads(read(run, 'car.json'))['front_seed']
            for k, v in want.items():
                self.assertEqual(fs[k], v, name)
            argv = json.loads(read(self.fd, 'wde-car-stack-%s.json' % name))['argv']
            self.assertEqual('FRONT_SEED_M=0.50' in argv, bool(env), name)
            d = self.down()
            self.assertEqual(d.returncode, 0, d.stdout + d.stderr)


if __name__ == '__main__':
    unittest.main()
