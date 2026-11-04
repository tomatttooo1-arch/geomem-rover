"""Rolling map -> Nav2 local layer bridge (2026-09-27).

Criteria U1-U7 of docs/design/ROLLING_NAV2_LAYER_20260927.md section 6.
Every rule is written as a ``check_*`` function that takes the thing under
test; each test runs it on the delivered code (must pass) and on a
deliberately broken variant (must fail), proving the check can see the fault.
Expected answers are re-typed by hand, not computed with the code under test.
"""

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

import host.rolling_nav_layer as L
from host.m7_rolling import (FREE, OCCUPIED, UNKNOWN, MapSnapshot, NavigationDelta,
                             RollingFusion, RollingKernelHost, compile_rolling)
from host.rolling_nav_layer import (BAND, BELOW, GROUND, ABOVE, HeightBand, RollingLayerBridge,
                                    check_costmap_config, decode_delta, decode_snapshot,
                                    encode_delta, encode_snapshot, occupancy_values,
                                    project_columns, rolling_map_permit)

U, F, O = UNKNOWN, FREE, OCCUPIED
MS = 1_000_000


# -- U1 projection -------------------------------------------------------------
def u1_case():
    """Window 4x3 columns, 6 z layers; band = local layers 2..4 (re-typed)."""
    s = np.zeros((6, 3, 4), dtype=np.uint8)
    # (x=0,y=0): obstacle at band layer 3 -> OCCUPIED
    s[2:5, 0, 0] = F
    s[3, 0, 0] = O
    # (x=1,y=0): whole band free, ground (layer 1) hit -> FREE
    s[2:5, 0, 1] = F
    s[1, 0, 1] = O
    # (x=2,y=0): band free except one unknown voxel -> UNKNOWN
    s[2, 0, 2] = F
    s[4, 0, 2] = F
    # (x=3,y=0): only ground hit, band unseen -> UNKNOWN
    s[1, 0, 3] = O
    # (x=0,y=1): obstacle above the band (layer 5) with free band -> FREE
    s[2:5, 1, 0] = F
    s[5, 1, 0] = O
    # (x=1,y=1): obstacle below the band (layer 0) with free band -> FREE
    s[2:5, 1, 1] = F
    s[0, 1, 1] = O
    # (x=2,y=1): occupied AND unknown in band -> OCCUPIED
    s[4, 1, 2] = O
    # (x=3,y=1): all free incl. outside band -> FREE
    s[:, 1, 3] = F
    # row y=2: all unknown except (x=0) band top occupied
    s[4, 2, 0] = O
    expected = np.array([[O, F, U, U],
                         [F, F, O, F],
                         [O, U, U, U]], dtype=np.uint8)
    return s, [2, 3, 4], expected


def check_u1(project):
    states, band, expected = u1_case()
    got = project(states, band)
    assert got.shape == (3, 4) and np.array_equal(got, expected), got


def project_any_free(states, band_layers):  # FAULT: one FREE voxel frees the column
    band = np.asarray(states)[list(band_layers)]
    out = np.full(band.shape[1:], UNKNOWN, np.uint8)
    out[np.any(band == FREE, axis=0)] = FREE
    out[np.any(band == OCCUPIED, axis=0)] = OCCUPIED
    return out


# -- U2 mapping ----------------------------------------------------------------
def check_u2(values):
    got = values(np.array([[O, F, U]], dtype=np.uint8))
    assert got.dtype == np.int8 and got.tolist() == [[100, 0, -1]], got


def values_unknown_as_free(tri):  # FAULT: UNKNOWN published as free (0)
    out = np.zeros(np.shape(tri), np.int8)
    out[np.asarray(tri) == OCCUPIED] = 100
    return out


GOOD_COSTMAP = {
    "global_frame": "odom", "robot_base_frame": "base_link", "rolling_window": True,
    "resolution": 0.05, "width": 4, "height": 4, "track_unknown_space": True,
    "footprint_padding": 0.0,
    "plugins": ["rolling_layer", "inflation_layer"],
    "rolling_layer": {"plugin": "nav2_costmap_2d::StaticLayer",
                      "map_topic": "/fpga/rolling_map/grid",
                      "map_subscribe_transient_local": True, "subscribe_to_updates": False,
                      "footprint_clearing_enabled": False},
    "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer"},
}


# -- U3 height band ------------------------------------------------------------
def check_u3(band_cls):
    """Voxel k=1 (centre 7.5 cm) seen from a moved ground: 4.9/5.0/5.1 cm."""
    v = 0.05
    cases = [  # (ground_z, robot_h, layer k, expected class)
        (0.026, 0.30, 1, GROUND),   # centre is 4.9 cm above ground
        (0.025, 0.30, 1, BAND),     # exactly 5.0 cm
        (0.024, 0.30, 1, BAND),     # 5.1 cm
        (0.0, 0.225, 5, BAND),      # centre 27.5 cm == robot 22.5 + 5 cm
        (0.0, 0.224, 5, ABOVE),     # 0.1 cm above the band top
        (0.025, 0.30, -1, GROUND),  # centre -5.0 cm: still ground
        (0.026, 0.30, -1, BELOW),   # centre -5.1 cm: below ground
        (0.0, 0.30, 0, GROUND),     # floor voxel on a perfect calibration
        (-0.001, 0.30, 0, GROUND),  # 1 mm calibration error keeps it ground
    ]
    for ground, height, k, want in cases:
        band = band_cls(ground_z_m=ground, robot_height_m=height)
        got = int(band.classify(k, 1, v)[0])
        assert got == want, (ground, height, k, got, want)


class SwappedBand(HeightBand):  # FAULT: ground and obstacle classes written the wrong way
    def class_of(self, k, voxel_m):
        c = super().class_of(k, voxel_m)
        return {GROUND: BAND, BAND: GROUND}.get(c, c)


class StrictLowerBand(HeightBand):  # FAULT: ">" instead of ">=" at ground + 5 cm
    def class_of(self, k, voxel_m):
        c = super().class_of(k, voxel_m)
        h = round(((k + 0.5) * voxel_m - self.ground_z_m) * 1e6)
        return GROUND if c == BAND and h == round(self.clearance_m * 1e6) else c


# -- U4 / U5 bridge ------------------------------------------------------------
DIMS = (4, 3, 6)
VOX = 0.05


def band():
    # window z base 0: layers centred at 2.5, 7.5, 12.5 ... cm; ground 0, robot 10 cm
    return HeightBand(ground_z_m=0.0, robot_height_m=0.10)


def delta(epoch, seq, t_ns, changes=(), base=(0, 0, 0), clock="mono"):
    return NavigationDelta(epoch, seq, seq, t_ns, clock, t_ns + 200 * MS, base, DIMS, VOX,
                           tuple(changes))


def new_bridge(cls=RollingLayerBridge):
    return cls(dims=DIMS, voxel_m=VOX, band=band(), clock_id="mono")


def full_column(x, y, state):
    return [(x, y, z, state) for z in range(6)]


def snapshot_of(epoch, seq, states, base=(0, 0, 0), acquired=0):
    wire = encode_snapshot(MapSnapshot(True, "map_current", epoch, seq, base, None, states),
                           acquired_ns=acquired, clock_id="mono", voxel_m=VOX)
    return decode_snapshot(wire)


def check_u4(bridge_cls):
    b = new_bridge(bridge_cls)
    t = 10_000 * MS
    ok, why, req = b.on_delta(delta(1, 1, t, full_column(0, 0, F)), now_ns=t + MS)
    assert ok and req is None, why
    ok, why, _ = b.on_delta(delta(1, 2, t + 100 * MS, [(1, 0, 1, O)]), now_ns=t + 101 * MS)
    assert ok, why
    # duplicate / old
    ok, why, req = b.on_delta(delta(1, 2, t + 150 * MS), now_ns=t + 151 * MS)
    assert (ok, why, req) == (False, "duplicate_or_old_delta", None), (ok, why)
    # wrong (older) epoch
    ok, why, req = b.on_delta(delta(0, 3, t + 150 * MS), now_ns=t + 151 * MS)
    assert (ok, why) == (False, "stale_epoch"), (ok, why)
    grid_before = b.grid().data.copy()
    # gap: seq 3 lost, seq 4 arrives -> rejected, snapshot requested
    ok, why, req = b.on_delta(delta(1, 4, t + 300 * MS, [(2, 0, 1, O)]), now_ns=t + 301 * MS)
    assert (ok, why) == (False, "sequence_gap") and req is not None, (ok, why, req)
    assert req["epoch"] == 1 and req["delta_seq"] == 2, req
    assert b.grid() is None  # nothing is published while desynchronised
    assert not b.status(now_ns=t + 302 * MS, clock_id="mono")["fresh"]
    # seq 5 arrives while waiting: kept, no second request inside the retry window
    ok, why, req = b.on_delta(delta(1, 5, t + 340 * MS, [(3, 0, 1, O)]), now_ns=t + 341 * MS)
    assert (ok, why, req) == (False, "awaiting_resync", None), (ok, why, req)
    # snapshot at seq 4 (the producer's state when it answered)
    s = np.zeros((6, 3, 4), np.uint8)
    s[:, 0, 0] = F
    s[1, 0, 1] = O
    s[1, 0, 2] = O
    ok, why, req = b.on_snapshot(snapshot_of(1, 4, s, acquired=t + 300 * MS),
                                 now_ns=t + 342 * MS)
    assert ok and req is None, (why, req)
    assert b.mirror.delta_seq == 5, b.mirror.delta_seq  # seq 5 replayed
    assert b.grid().data.tolist() == [0, 100, 100, 100, -1, -1, -1, -1, -1, -1, -1, -1], \
        b.grid().data.tolist()
    assert not np.array_equal(grid_before, b.grid().data)
    # expired delta -> rejected + request
    ok, why, req = b.on_delta(delta(1, 6, t + 500 * MS), now_ns=t + 701 * MS)
    assert (ok, why) == (False, "expired_delta") and req is not None, (ok, why)
    # clock mismatch never applies
    b2 = new_bridge(bridge_cls)
    ok, why, _ = b2.on_delta(delta(1, 1, t, clock="ros"), now_ns=t)
    assert (ok, why) == (False, "clock_mismatch"), (ok, why)
    # new epoch must start at seq 1
    b3 = new_bridge(bridge_cls)
    ok, why, req = b3.on_delta(delta(3, 2, t), now_ns=t)
    assert (ok, why) == (False, "sequence_gap") and req is not None, (ok, why)


class ApplyThroughGaps(RollingLayerBridge):  # FAULT: gaps are applied, not rejected
    def on_delta(self, delta, *, now_ns):
        if self.mirror.epoch == delta.epoch and delta.delta_seq > self.mirror.delta_seq + 1:
            self.mirror.delta_seq = delta.delta_seq - 1
        return super().on_delta(delta, now_ns=now_ns)


def check_u5(bridge_cls, permit=rolling_map_permit):
    b = new_bridge(bridge_cls)
    t = 50_000 * MS
    assert b.status(now_ns=t, clock_id="mono")["reason"] == "no_map"
    b.on_delta(delta(1, 1, t, full_column(0, 0, F)), now_ns=t + MS)
    for dt, fresh in ((0, True), (199 * MS, True), (200 * MS, True), (200 * MS + 1, False),
                      (900 * MS, False)):
        row = b.status(now_ns=t + dt, clock_id="mono")
        assert row["fresh"] is fresh, (dt, row)
        ok, _ = permit(row, received_ns=5, now_ns=5)
        assert ok is fresh, (dt, row)
    row = b.status(now_ns=t + 250 * MS, clock_id="mono")
    assert row["reason"] == "map_stale", row
    assert b.status(now_ns=t, clock_id="ros")["reason"] == "clock_mismatch"
    # the consumer adds the time since receipt: a dead bridge stops within 200 ms
    fresh_row = b.status(now_ns=t + 10 * MS, clock_id="mono")  # map age 10 ms
    assert permit(fresh_row, received_ns=0, now_ns=190 * MS)[0] is True
    assert permit(fresh_row, received_ns=0, now_ns=190 * MS + 1) == \
        (False, "rolling_map_stale")
    assert permit(None, received_ns=None, now_ns=0) == (False, "no_rolling_status")
    assert permit(dict(fresh_row, age_ns=None), received_ns=0, now_ns=0)[0] is False


class NoStaleStop(RollingLayerBridge):  # FAULT: the age limit is ignored
    def status(self, *, now_ns, clock_id):
        row = super().status(now_ns=now_ns, clock_id=clock_id)
        if row["reason"] == "map_stale":
            row.update(fresh=True, reason="map_current", age_ns=0)
        return row


def permit_ignores_elapsed(row, *, received_ns, now_ns):  # FAULT: dead bridge never stops
    return L.rolling_map_permit(row, received_ns=now_ns, now_ns=now_ns)


# -- U8 body outline (owner ruling 2026-09-27) ------------------------------------
def check_u8(cells, seed):
    """4x3 window at base (0,0); body r = 4 cm centred on the corner of 4 cells."""
    mask = cells((0, 0, 0), (4, 3, 6), VOX, (0.10, 0.05), 0.04)
    # the four cells meeting at the centre overlap it; every other cell's
    # closest point is 5 cm away (> 4 cm)
    want = np.zeros((3, 4), bool)
    want[0:2, 1:3] = True
    assert np.array_equal(mask, want), mask
    tri = np.array([[U, U, O, U],
                    [U, F, U, U],
                    [U, U, U, U]], np.uint8)
    got = seed(tri, mask)
    expected = np.array([[U, F, O, U],
                         [U, F, F, U],
                         [U, U, U, U]], np.uint8)
    assert np.array_equal(got, expected), got


def body_cells_one_cell_wider(base, dims, voxel, center, radius):  # FAULT: outside outline
    return L.body_cells(base, dims, voxel, center, radius + voxel)


def seed_all_unknown(tri, mask):  # FAULT: every UNKNOWN treated as free
    out = np.array(tri, np.uint8, copy=True)
    out[out == UNKNOWN] = FREE
    return out


def seed_over_occupied(tri, mask):  # FAULT: the outline clears obstacles too
    out = np.array(tri, np.uint8, copy=True)
    out[np.asarray(mask, bool)] = FREE
    return out


# -- U9 Nav2 settings of the ruling ----------------------------------------------
def shipped_fragment():
    import yaml
    path = Path(__file__).resolve().parents[1] / \
        "robot_sim/fpga_sim/config/rolling_layer_nav2.yaml"
    return yaml.safe_load(path.read_text())


def check_u9(doc):
    return check_costmap_config(
        doc["local_costmap"]["local_costmap"]["ros__parameters"],
        follow_path=doc["controller_server"]["ros__parameters"]["FollowPath"],
        planner=doc["planner_server"]["ros__parameters"]["GridBased"])


def _clearing_on(doc):
    doc["local_costmap"]["local_costmap"]["ros__parameters"]["rolling_layer"][
        "footprint_clearing_enabled"] = True


def _no_footprint_critic(doc):
    fp = doc["controller_server"]["ros__parameters"]["FollowPath"]
    fp["critics"] = [c for c in fp["critics"] if c != "ObstacleFootprint"]


def _tolerance(doc):
    doc["planner_server"]["ros__parameters"]["GridBased"]["tolerance"] = 0.5


def _track_unknown_off(doc):
    doc["local_costmap"]["local_costmap"]["ros__parameters"]["track_unknown_space"] = False


def _padding(doc):
    doc["local_costmap"]["local_costmap"]["ros__parameters"].pop("footprint_padding")


U9_VARIANTS = {"footprint_padding_default_1cm": _padding,
               "footprint_clearing_on": _clearing_on,
               "obstacle_footprint_removed": _no_footprint_critic,
               "navfn_tolerance_0.5": _tolerance,
               "unknown_as_free_space": _track_unknown_off}


# -- U10 cold-start confirmation (second owner ruling 2026-09-27) ---------------
CDIMS = (12, 12, 6)


def cold_bridge(cls, log):
    return cls(dims=CDIMS, voxel_m=VOX, band=band(), clock_id="mono", body_radius_m=0.10,
               cold_start_width_m=0.10, cold_start_timeout_ns=30_000 * MS,
               cold_start_log=log.append)


def cdelta(seq, t, changes=()):
    return NavigationDelta(1, seq, seq, t, "mono", t + 200 * MS, (0, 0, 0), CDIMS, VOX,
                           tuple(changes))


def check_u10(cls):
    log = []
    b = cold_bridge(cls, log)
    t = 5_000 * MS
    # one obstacle inside the future ring (world cell (8,6), ahead of a body at 0.30, 0.30)
    b.on_delta(cdelta(1, t, [(8, 6, 2, O)]), now_ns=t)
    body = (0.30, 0.30)
    # 1. without confirmation: exactly the body-outline grid, nothing else
    plain = b.grid(body_xy=body, now_ns=t).data.copy()
    assert np.array_equal(plain, b._grid_body(body).data), "unconfirmed grid differs"
    assert b.status(now_ns=t, clock_id="mono")["cold_start"] == "none"
    # 2. confirmation: logged, ring ahead becomes FREE, OCCUPIED untouched
    ok, why = b.confirm_cold_start(operator="dev2", operator_stamp="2026-09-27T20:00:00+08:00",
                                   now_ns=t, body_pose=(0.30, 0.30, 0.0))
    assert ok and why == "accepted", why
    assert len(log) == 1 and json.loads(log[0])["operator"] == "dev2", log
    g = b.grid(body_xy=body, now_ns=t + MS).data.reshape(12, 12)
    ring = L.cold_ring_cells((0, 0, 0), CDIMS, VOX, body, 0.0, 0.10, 0.10)
    body_mask = L.body_cells((0, 0, 0), CDIMS, VOX, body, 0.10)
    assert int(g[6, 8]) == 100, "ring overwrote an obstacle"
    freed = (g == 0) & ~body_mask
    assert freed.any() and np.array_equal(freed, ring & (g != 100)), "ring cells wrong"
    assert not (freed & ~ring).any(), "FREE outside the ring"
    # the ring lies ahead only (x >= body x) and outside the body
    assert not (ring & body_mask).any() and not ring[:, :5].any()
    # 3. automatic end: moving more than the ring width
    g2 = b.grid(body_xy=(0.41, 0.30), now_ns=t + 2 * MS).data
    assert b.cold_start_state() == "ended:moved", b.cold_start_state()
    assert np.array_equal(g2, b._grid_body((0.41, 0.30)).data), "ring not withdrawn"
    assert any(json.loads(r)["event"] == "cold_start_end" for r in log), log
    # 4. once only, and the refused repeat is logged too
    n = len(log)
    ok, why = b.confirm_cold_start(operator="dev2", operator_stamp="2026-09-27T20:01:00+08:00",
                                   now_ns=t + 3 * MS, body_pose=(0.41, 0.30, 0.0))
    assert (ok, why) == (False, "already_used_once"), (ok, why)
    assert len(log) == n + 1 and json.loads(log[-1])["result"] == "already_used_once", log
    # 5. timeout ends it as well (fresh bridge)
    log2 = []
    b2 = cold_bridge(cls, log2)
    b2.on_delta(cdelta(1, t), now_ns=t)
    b2.confirm_cold_start(operator="dev3", operator_stamp="x", now_ns=t,
                          body_pose=(0.30, 0.30, 0.0))
    b2.grid(body_xy=body, now_ns=t + 30_000 * MS)
    assert b2.cold_start_state() == "active"
    b2.grid(body_xy=body, now_ns=t + 30_000 * MS + 1)
    assert b2.cold_start_state() == "ended:timeout", b2.cold_start_state()
    # 6. no operator / no log sink -> refused
    b3 = cold_bridge(cls, [])
    assert b3.confirm_cold_start(operator="", operator_stamp="x", now_ns=t,
                                 body_pose=(0, 0, 0))[0] is False
    b4 = cls(dims=CDIMS, voxel_m=VOX, band=band(), clock_id="mono", body_radius_m=0.10,
             cold_start_width_m=0.10, cold_start_log=None)
    assert b4.confirm_cold_start(operator="dev2", operator_stamp="x", now_ns=t,
                                 body_pose=(0, 0, 0)) == (False, "log_failed")


class RingWithoutConfirmation(RollingLayerBridge):  # FAULT: released without an operator
    def grid(self, body_xy=None, now_ns=None):
        if self._cold is None and body_xy is not None:
            self._cold = {"pose": (body_xy[0], body_xy[1], 0.0), "t": now_ns or 0,
                          "operator": None, "active": True, "ended": None}
        return super().grid(body_xy=body_xy, now_ns=now_ns)


class RingOverOccupied(RollingLayerBridge):  # FAULT: the ring clears obstacles too
    def grid(self, body_xy=None, now_ns=None):
        out = super().grid(body_xy=body_xy, now_ns=now_ns)
        mask = self.cold_start_cells()
        if out is None or mask is None:
            return out
        data = out.data.copy()
        data[mask.ravel()] = 0
        return replace(out, data=data)


class RingNeverEnds(RollingLayerBridge):  # FAULT: no automatic withdrawal
    def cold_start_state(self, *, body_xy=None, now_ns=None):
        return super().cold_start_state()


class RepeatWithoutLog(RollingLayerBridge):  # FAULT: repeat allowed and not logged
    def confirm_cold_start(self, **kw):
        if self._cold is not None:
            self._cold = None
            self.cold_start_log, keep = (lambda row: None), self.cold_start_log
            try:
                return super().confirm_cold_start(**kw)
            finally:
                self.cold_start_log = keep
        return super().confirm_cold_start(**kw)


class Checks(unittest.TestCase):
    def assert_fault_seen(self, check, *args):
        with self.assertRaises(AssertionError):
            check(*args)

    def test_u1_projection_known_answer(self):
        check_u1(project_columns)
        self.assert_fault_seen(check_u1, project_any_free)

    def test_u1_band_outside_window_is_refused(self):
        with self.assertRaises(ValueError):
            HeightBand(ground_z_m=0.0, robot_height_m=0.30).band_layers(0, 4, VOX)
        for bad in (None, float("nan"), True, "0.1"):
            with self.assertRaises(ValueError):
                HeightBand(ground_z_m=bad, robot_height_m=0.3)

    def test_u2_tristate_mapping(self):
        check_u2(occupancy_values)
        self.assert_fault_seen(check_u2, values_unknown_as_free)

    def test_u2_costmap_config_guard(self):
        self.assertEqual(check_costmap_config(GOOD_COSTMAP), [])
        for key, value in (("track_unknown_space", False), ("resolution", 0.1),
                           ("use_maximum", True), ("global_frame", "map")):
            bad = dict(GOOD_COSTMAP, **{key: value})
            self.assertTrue(check_costmap_config(bad), key)
        missing = {k: v for k, v in GOOD_COSTMAP.items() if k != "track_unknown_space"}
        self.assertIn("track_unknown_space must be True", check_costmap_config(missing))
        bad = dict(GOOD_COSTMAP, plugins=["rolling_layer", "obstacle_layer", "inflation_layer"],
                   obstacle_layer={"plugin": "nav2_costmap_2d::ObstacleLayer"})
        self.assertTrue(check_costmap_config(bad))
        bad = dict(GOOD_COSTMAP, rolling_layer=dict(GOOD_COSTMAP["rolling_layer"],
                                                    subscribe_to_updates=True))
        self.assertTrue(check_costmap_config(bad))
        self.assertTrue(check_costmap_config(
            GOOD_COSTMAP, controller_plugin="nav2_regulated_pure_pursuit_controller::"
                                            "RegulatedPurePursuitController"))

    def test_u2_shipped_nav2_fragment_passes_the_guard(self):
        self.assertEqual(check_u9(shipped_fragment()), [])

    def test_u8_body_outline_known_free(self):
        check_u8(L.body_cells, L.seed_body)
        self.assert_fault_seen(check_u8, body_cells_one_cell_wider, L.seed_body)
        self.assert_fault_seen(check_u8, L.body_cells, seed_all_unknown)
        self.assert_fault_seen(check_u8, L.body_cells, seed_over_occupied)

    def test_u8_bridge_seeds_only_the_current_outline(self):
        b = RollingLayerBridge(dims=DIMS, voxel_m=VOX, band=band(), clock_id="mono",
                               body_radius_m=0.05)
        t = 1000 * MS
        b.on_delta(delta(1, 1, t, [(3, 2, 2, O)]), now_ns=t)
        strict = b.grid().data.tolist()
        self.assertEqual(strict.count(-1), 11)
        # body r = 5 cm centred on cell (1,1): it overlaps the 3x3 block around it
        # (diagonal cells' closest corner is 3.5 cm away), not column 3 (7.5 cm)
        seeded = b.grid(body_xy=(0.075, 0.075)).data.reshape(3, 4)
        self.assertEqual(np.argwhere(seeded == 0).tolist(),
                         [[r, c] for r in range(3) for c in range(3)])
        # the body moved on: nothing it covered before stays FREE
        moved = b.grid(body_xy=(10.0, 10.0)).data.tolist()
        self.assertEqual(moved, strict)
        # an obstacle under the body stays LETHAL
        self.assertEqual(int(b.grid(body_xy=(0.175, 0.125)).data.reshape(3, 4)[2, 3]), 100)

    def test_u10_cold_start_confirmation(self):
        check_u10(RollingLayerBridge)
        for fault in (RingWithoutConfirmation, RingOverOccupied, RingNeverEnds,
                      RepeatWithoutLog):
            with self.subTest(fault=fault.__name__):
                self.assert_fault_seen(check_u10, fault)

    def test_u9_ruling_config_variants_are_refused(self):
        self.assertEqual(check_u9(shipped_fragment()), [])
        for name, mutate in U9_VARIANTS.items():
            with self.subTest(variant=name):
                doc = shipped_fragment()
                mutate(doc)
                self.assertTrue(check_u9(doc), name)

    def test_u3_height_band_boundaries(self):
        check_u3(HeightBand)
        self.assert_fault_seen(check_u3, SwappedBand)
        self.assert_fault_seen(check_u3, StrictLowerBand)

    def test_u4_reject_and_resync(self):
        check_u4(RollingLayerBridge)
        self.assert_fault_seen(check_u4, ApplyThroughGaps)

    def test_u5_stale_map_stops(self):
        check_u5(RollingLayerBridge)
        self.assert_fault_seen(check_u5, NoStaleStop)
        self.assert_fault_seen(check_u5, RollingLayerBridge, permit_ignores_elapsed)

    def test_u6_codec_round_trip_and_rejections(self):
        d = NavigationDelta(7, 3, 99, 123456789, "mono", 323456789, (-40, 12, -8),
                            (80, 80, 32), 0.05, ((-40, 12, -8, 2), (39, 91, 23, 1)))
        self.assertEqual(decode_delta(encode_delta(d)), d)
        raw = encode_delta(d)
        for broken in (raw[:-1], b"XXXX" + raw[4:], raw[:8]):
            with self.assertRaises(ValueError):
                decode_delta(broken)
        bad_state = encode_delta(NavigationDelta(7, 3, 99, 1, "mono", 2, (0, 0, 0),
                                                 (80, 80, 32), 0.05, ((0, 0, 0, 3),)))
        with self.assertRaises(ValueError):
            decode_delta(bad_state)
        states = np.random.default_rng(3).integers(0, 3, (6, 3, 4)).astype(np.uint8)
        w = snapshot_of(2, 9, states, base=(5, -6, 7), acquired=42)
        self.assertEqual((w.snapshot.epoch, w.snapshot.delta_seq, w.snapshot.window_base,
                          w.acquired_ns, w.clock_id, w.voxel_m), (2, 9, (5, -6, 7), 42,
                                                                  "mono", 0.05))
        self.assertTrue(np.array_equal(w.snapshot.states, states))
        raw = encode_snapshot(w.snapshot, acquired_ns=42, clock_id="mono", voxel_m=0.05)
        with self.assertRaises(ValueError):
            decode_snapshot(raw[:-3])
        with self.assertRaises(ValueError):
            decode_snapshot(raw[:-1] + b"\x07")

    def test_u6_grid_geometry(self):
        b = new_bridge()
        t = 1000 * MS
        b.on_delta(delta(1, 1, t, full_column(-3, 5, F), base=(-4, 5, 0)), now_ns=t)
        g = b.grid()
        self.assertEqual((g.frame_id, g.width, g.height, g.resolution), ("odom", 4, 3, 0.05))
        self.assertAlmostEqual(g.origin_xy[0], -0.20, places=12)
        self.assertAlmostEqual(g.origin_xy[1], 0.25, places=12)
        # world column (-3, 5) is local (1, 0): data index ly*width + lx = 1
        self.assertEqual(np.flatnonzero(g.data == 0).tolist(), [1])


# -- U7 end to end with the real CPU rolling kernel -----------------------------
class EndToEnd(unittest.TestCase):
    def test_u7_bridge_grid_equals_projected_fusion_snapshot(self):
        from host.m7_frame_native import FrameNativeM7, compile_frame_native
        from host.m7_grid_policy import FollowCameraGrid
        from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, \
            PoseAtCapture
        with tempfile.TemporaryDirectory(prefix="rolling-nav-") as tmp:
            frame_lib = compile_frame_native(Path(tmp) / "frame.so")
            fusion = RollingFusion(RollingKernelHost(compile_rolling(Path(tmp) / "roll.so")),
                                   voxel_m=0.05)
            shape = (32, 32, 10)
            policy = FollowCameraGrid(0.05, shape, back_cells=1, height_cell=5)
            pipeline = M7Pipeline(FrameNativeM7(frame_lib), GridSpec((0., 0., 0.), 0.05, shape),
                                  grid_policy=policy)
            hb = HeightBand(ground_z_m=0.0, robot_height_m=0.20)
            bridge = RollingLayerBridge(dims=(80, 80, 32), voxel_m=0.05, band=hb,
                                        clock_id="mono")
            k = np.array([[20., 0., 7.5], [0., 20., 5.5], [0., 0., 1.]])
            r = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])  # optical axis +x
            t = 1_000_000_000
            wall_x = 1.0125
            for i in range(6):
                t += 100 * MS
                x = 0.013 + 0.05 * i
                pose = np.eye(4)
                pose[:3, :3], pose[:3, 3] = r, (x, 0.012, 0.21)
                depth = np.full((12, 16), wall_x - x, np.float32)  # wall facing the camera
                ident = CaptureIdentity("cam0", i + 1, t, "mono", "cal", "odom", "e1")
                out = pipeline.ingest_geometry(
                    DepthFrame(ident, depth, k, PoseAtCapture(t, "mono", "odom", "e1", True,
                                                              pose)),
                    now_ns=t + MS, now_clock_id="mono")
                fused = fusion.ingest(out.navigation, pipeline, now_ns=t + 2 * MS,
                                      clock_id="mono", robot_position_m=(x, 0.012, 0.0))
                self.assertTrue(fused.accepted, fused.reason)
                wire = decode_delta(encode_delta(fused.delta))
                ok, why, _ = bridge.on_delta(wire, now_ns=t + 3 * MS)
                self.assertTrue(ok, why)
                snap = fusion.snapshot(now_ns=t + 3 * MS, clock_id="mono")
                layers = hb.band_layers(snap.window_base[2], 32, 0.05)
                expected = occupancy_values(project_columns(snap.states, layers)).ravel()
                self.assertTrue(np.array_equal(bridge.grid().data, expected), i)
            data = bridge.grid().data.reshape(80, 80)
            base = bridge.mirror.window_base
            wall_cell = (int(np.floor(wall_x / 0.05)) - base[0], 0 - base[1])
            self.assertEqual(int(data[wall_cell[1], wall_cell[0]]), 100)
            self.assertGreater(int(np.sum(data == 0)), 0)
            self.assertEqual(bridge.counts["applied"], 6)


if __name__ == "__main__":
    unittest.main()
