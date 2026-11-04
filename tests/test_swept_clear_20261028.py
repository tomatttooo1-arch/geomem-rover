"""Swept-area clearing in the rolling bridge (2026-10-28, owner approved; car_inner.sh SWEPT_CLEAR_S, default 300 s).

Cells the body circle covered are sent FREE for swept_clear_s (the car was physically there), whatever the
projection says -- against floor lifted into the band by a ground-calibration angle error (10-28 14:19 run). A real
obstacle put down later wins back: an OCCUPIED voxel centred >= 15 cm above the ground while the cell is within 1 m
of the body ends the clearing of that cell. Off (None / 0): cell for cell the grid of before.
Known answers on hand-built 3-D windows (synthetic grid + trajectory). Python 3.8.
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from host import rolling_nav_layer as L   # noqa: E402
from host.m7_rolling import FREE, OCCUPIED, UNKNOWN   # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V = 0.05
DIMS = (80, 80, 32)        # window 4 m x 4 m, base (-40, -40, 0) -> odom x, y in [-2, 2); layer k centre (k+.5)*5 cm
BASE = (-40, -40, 0)
R_BODY, MARGIN = 0.30, 0.05
S = 1_000_000_000


def bridge(swept_clear_s=300.0):
    band = L.HeightBand(ground_z_m=0.0, robot_height_m=0.31)
    return L.RollingLayerBridge(dims=DIMS, voxel_m=V, band=band, clock_id='steady', body_radius_m=R_BODY,
                                body_margin_m=MARGIN, swept_clear_s=swept_clear_s)


def install(b, states):
    b.mirror.window_base = BASE
    b.mirror.states = states
    b.mirror.needs_resync = False
    b.mirror.epoch, b.mirror.delta_seq = 1, 0
    b._grid = None
    return b


def cell(x, y):
    return int(np.floor(y / V)) - BASE[1], int(np.floor(x / V)) - BASE[0]   # (ly, lx)


def world():
    """every band column FREE (seen floor), plus a ring of LOW false obstacles (layers 1-2: 7.5 / 12.5 cm)
    everywhere |y| in [0.4, 0.9] -- the 10-28 picture -- and a corridor y in [-0.3, 0.3] also low-occupied ahead."""
    st = np.full((DIMS[2], DIMS[1], DIMS[0]), UNKNOWN, np.uint8)
    st[1:7] = FREE
    c = (np.arange(80) + BASE[0] + 0.5) * V
    X, Y = np.meshgrid(c, c)
    low = ((np.abs(Y) >= 0.4) & (np.abs(Y) <= 0.9)) | ((np.abs(Y) <= 0.3) & (X > 0.4))
    st[1:3][:, low] = OCCUPIED
    return st, X, Y


def drive(b, xs, y=0.0, t0=0, dt=0.1):
    t = t0
    for x in xs:
        g = b.grid(body_xy=(x, y), now_ns=t, body_yaw=0.0)
        t += int(dt * S)
    return g.data.reshape(80, 80), t


class TestSwept(unittest.TestCase):
    def test_off_is_the_grid_of_before(self):
        st, _, _ = world()
        for off in (None, 0, 0.0):
            a = install(bridge(off), st.copy())
            ref = install(L.RollingLayerBridge(dims=DIMS, voxel_m=V, band=L.HeightBand(ground_z_m=0.0,
                                               robot_height_m=0.31), clock_id='steady', body_radius_m=R_BODY,
                                               body_margin_m=MARGIN), st.copy())
            for x in np.arange(-1.0, 1.0, 0.05):
                ga = a.grid(body_xy=(x, 0.0), now_ns=int((x + 1) * S), body_yaw=0.0)
                gr = ref.grid(body_xy=(x, 0.0), now_ns=int((x + 1) * S), body_yaw=0.0)
                self.assertTrue(np.array_equal(ga.data, gr.data))
            self.assertIsNone(a.swept)
            self.assertNotIn("swept", a.status(now_ns=0, clock_id='steady'))

    def test_track_cleared_outside_unchanged(self):
        st, X, Y = world()
        b = install(bridge(), st)
        raw = L.occupancy_values(b.tri2d())
        g, _ = drive(b, np.arange(-1.0, 1.0001, 0.02))       # 2 cm steps along y = 0 from x -1 to 1
        # inside the track (closest point within 0.30 m of the segment): FREE, even where it was low-occupied
        lx = np.arange(80); ly = np.arange(80)
        x0 = (BASE[0] + lx) * V; y0 = (BASE[1] + ly) * V
        dx = np.maximum(np.maximum(x0 - 1.0, -1.0 - (x0 + V)), 0.0)[None, :]
        dy = np.maximum(np.maximum(y0 - 0.0, 0.0 - (y0 + V)), 0.0)[:, None]
        track = dx * dx + dy * dy <= R_BODY ** 2
        self.assertTrue(np.all(g[track] == L.GRID_FREE))
        self.assertTrue(np.any(raw[track] == L.GRID_LETHAL))   # there was something to clear (corridor x > 0.4)
        # the ring at |y| >= 0.4 is outside the 0.30 m track (+ 0.05 m body margin of the current pose): untouched
        ring = (np.abs(Y) >= 0.4) & (np.abs(Y) <= 0.9)
        self.assertTrue(np.all(g[ring] == L.GRID_LETHAL))
        self.assertGreater(b.counts["swept_cleared_cells"], 0)
        self.assertIn("swept", b.status(now_ns=0, clock_id='steady'))

    def test_steps_joined_jumps_not(self):
        st, _, _ = world()
        b = install(bridge(), st)
        g, t = drive(b, [0.5, 0.8])                          # 0.3 m step: the capsule covers x 0.5..0.8
        self.assertEqual(g[cell(0.65, 0.0)], L.GRID_FREE)
        b2 = install(bridge(), world()[0])
        g2, _ = drive(b2, [-1.5, 1.5])                       # 3 m jump: only the two circles
        self.assertEqual(g2[cell(0.65, 0.0)], L.GRID_LETHAL)
        self.assertEqual(g2[cell(1.5, 0.0)], L.GRID_FREE)

    def test_hold_expires(self):
        st, _, _ = world()
        b = install(bridge(swept_clear_s=10.0), st)
        drive(b, np.arange(0.5, 1.2, 0.02))                  # sweep the low corridor x 0.5..1.2, then leave
        g = b.grid(body_xy=(-1.5, 0.0), now_ns=9 * S, body_yaw=0.0).data.reshape(80, 80)
        self.assertEqual(g[cell(0.9, 0.0)], L.GRID_FREE)     # within 10 s
        g = b.grid(body_xy=(-1.5, 0.0), now_ns=30 * S, body_yaw=0.0).data.reshape(80, 80)
        self.assertEqual(g[cell(0.9, 0.0)], L.GRID_LETHAL)   # expired: the projection again

    def test_real_obstacle_later_wins_back_when_near(self):
        st, _, _ = world()
        b = install(bridge(), st)
        _, t = drive(b, np.arange(-1.0, 1.0001, 0.02))
        # afterwards a bottle (tall: layers 1-5, up to 27.5 cm) at (0.0, 0.0) on the track, and a LOW thing (layer 1)
        # at (-0.5, 0.0) on the track
        ly, lx = cell(0.0, 0.0); st[1:6, ly, lx] = OCCUPIED
        ly2, lx2 = cell(-0.5, 0.0); st[1, ly2, lx2] = OCCUPIED
        b._grid = None
        far = b.grid(body_xy=(1.5, 0.0), now_ns=t, body_yaw=0.0).data.reshape(80, 80)      # 1.5 m away
        self.assertEqual(far[ly, lx], L.GRID_FREE)           # far: not yet strong evidence
        near = b.grid(body_xy=(0.8, 0.0), now_ns=t + S, body_yaw=0.0).data.reshape(80, 80)  # 0.8 m: near
        self.assertEqual(near[ly, lx], L.GRID_LETHAL)
        self.assertGreater(b.counts["swept_evidence_cells"], 0)
        back = b.grid(body_xy=(1.5, 0.0), now_ns=t + 2 * S, body_yaw=0.0).data.reshape(80, 80)
        self.assertEqual(back[ly, lx], L.GRID_LETHAL)        # the clearing of that cell ended for good
        # the low thing stays cleared (documented cost), also near
        g = b.grid(body_xy=(-0.2, 0.0), now_ns=t + 3 * S, body_yaw=0.0).data.reshape(80, 80)
        self.assertEqual(g[ly2, lx2], L.GRID_FREE)

    def test_bad_value_refused(self):
        for bad in (-1.0, float('nan'), True, '300'):
            with self.assertRaises(ValueError):
                bridge(bad)

    def test_tiles_cross_negative_coordinates(self):
        sw = L.SweptCells(V)
        sw.mark((-0.01, -0.01), 0.3, 5)
        m = sw.window_mask(BASE, DIMS, 5, 1)
        self.assertTrue(m[cell(-0.01, -0.01)] and m[cell(0.2, 0.0)] and m[cell(-0.2, 0.0)])
        self.assertFalse(m[cell(0.5, 0.0)])
        sw.forget(BASE, m)
        self.assertFalse(sw.window_mask(BASE, DIMS, 5, 1).any())

    def test_wiring(self):
        with open(os.path.join(ROOT, 'tools', 'car_run', 'car_inner.sh')) as f:
            inner = f.read()
        self.assertIn('-p swept_clear_s:="${SWEPT_CLEAR_S:-300.0}"', inner)
        with open(os.path.join(ROOT, 'host', 'ros_rolling_nav.py')) as f:
            node = f.read()
        self.assertIn('("swept_clear_s", 0.0)', node)
        self.assertIn('swept_clear_s=float(p("swept_clear_s"))', node)
        self.assertIn('ParameterDescriptor(dynamic_typing=True)', node)   # 10-28: an INTEGER -p value no longer crashes


if __name__ == '__main__':
    unittest.main()
