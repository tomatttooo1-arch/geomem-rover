"""Per-frame grid that follows the camera, fused into the rolling map (2026-09-27).

Rules and their fault variants: docs/design/M7_FOLLOW_GRID_20260927.md section 3.
"""

from pathlib import Path
import tempfile
import unittest

import numpy as np

from host.m7_frame_native import FrameNativeM7, compile_frame_native
from host.m7_grid_policy import FollowCameraGrid
from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
from host.m7_rolling import FREE, OCCUPIED, RollingFusion, RollingKernelHost, compile_rolling

VOXEL = 0.05
SHAPE = (32, 32, 8)
K = np.array([[40., 0., 4.], [0., 40., 4.], [0., 0., 1.]])
# Optical axis +x (camera x -> -y, camera y -> -z) and +y (camera x -> +x, y -> -z).
R_PLUS_X = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
R_PLUS_Y = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])
WALL_X = 3.0125   # world voxel x = 60
WALL_Y = 1.0125   # world voxel y = 20
# Camera positions are deliberately off the lattice (x fraction 0.74 cell,
# y/z at half a cell) so an unsnapped or rounded origin is observable.
PATH = ([("x", (1.837 + 0.1 * i, 0.025, 0.025)) for i in range(7)] +
        [("y", (2.437, 0.025 + 0.1 * i, 0.025)) for i in range(5)])
# Re-typed from the design (not read from the policy): camera cell in block.
EXPECTED_ANCHOR = {"x": (1, 16, 1), "y": (16, 1, 1)}


def pose_for(direction, position):
    t = np.eye(4)
    t[:3, :3] = R_PLUS_X if direction == "x" else R_PLUS_Y
    t[:3, 3] = position
    return t


def depth_for(direction, position):
    distance = (WALL_X - position[0]) if direction == "x" else (WALL_Y - position[1])
    return np.full((9, 9), distance, np.float32)  # plane facing the camera: constant optical Z


class RoundingPolicy(FollowCameraGrid):
    def cell_of(self, position_m):  # FAULT: round instead of floor
        return tuple(int(v) for v in np.round(np.asarray(position_m) / self.voxel_m))


class UnsnappedPolicy(FollowCameraGrid):
    def grid_for(self, transform):  # FAULT: origin not an integer number of voxels
        anchor = np.array(self.anchor_for(np.asarray(transform)[:3, :3]))
        origin = np.asarray(transform)[:3, 3] - (anchor + 0.5) * self.voxel_m
        return GridSpec(tuple(float(v) for v in origin), self.voxel_m, self.shape)


class FlippedPolicy(FollowCameraGrid):
    def base_for(self, transform):  # FAULT: base = cell + anchor
        t = np.asarray(transform)
        return tuple(c + a for c, a in zip(self.cell_of(t[:3, 3]), self.anchor_for(t[:3, :3])))


class FollowGridTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="m7-follow-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.dir = Path(cls.tmp.name)
        cls.frame_lib = compile_frame_native(cls.dir / "frame.so")
        cls.builds = 0

    @classmethod
    def fresh_fusion(cls):
        cls.builds += 1
        return RollingFusion(RollingKernelHost(compile_rolling(cls.dir / ("r%d.so" % cls.builds))),
                             voxel_m=VOXEL)

    def drive(self, policy=None, fixed_grid=None):
        grid = fixed_grid or GridSpec((0., 0., 0.), VOXEL, SHAPE)
        pipeline = M7Pipeline(FrameNativeM7(self.frame_lib), grid, grid_policy=policy)
        fusion = self.fresh_fusion()
        rows, t = [], 1_000_000_000
        for frame_id, (direction, position) in enumerate(PATH, start=1):
            t += 50_000_000
            transform = pose_for(direction, position)
            identity = CaptureIdentity("cam0", frame_id, t, "mono", "cal-1", "odom", "e1")
            pose = PoseAtCapture(t, "mono", "odom", "e1", True, transform)
            out = pipeline.ingest_geometry(DepthFrame(identity, depth_for(direction, position),
                                                      K, pose),
                                           now_ns=t + 1_000_000, now_clock_id="mono")
            fused = fusion.ingest(out.navigation, pipeline, now_ns=t + 2_000_000,
                                  clock_id="mono", robot_position_m=position)
            rows.append((direction, position, transform, out.navigation, fused))
        return rows, fusion, t

    def assert_all_fused(self, rows):
        for direction, position, _, nav, fused in rows:
            self.assertTrue(nav.valid, (position, nav.reason))
            self.assertTrue(fused.accepted, (position, fused.reason))

    def world_states(self, fusion, t, cells):
        q = fusion.query_world(cells, now_ns=t + 3_000_000, clock_id="mono")
        self.assertTrue(q.valid, q.reason)
        return [int(s) for s in q.states]

    def assert_known_walls(self, fusion, t):
        occupied = [(60, 0, 0), (48, 20, 0)]
        free = [(55, 0, 0), (58, 0, 0), (48, 15, 0)]
        states = self.world_states(fusion, t, occupied + free)
        self.assertEqual(states, [OCCUPIED] * 2 + [FREE] * 3)

    # -- rules -----------------------------------------------------------
    def test_following_grid_fuses_every_frame_and_finds_the_walls(self):
        rows, fusion, t = self.drive(FollowCameraGrid(VOXEL, SHAPE))
        self.assert_all_fused(rows)
        self.assertEqual(dict(fusion.counts), {"map_reset": 1, "accepted": len(PATH)})
        self.assert_known_walls(fusion, t)
        origins = {nav.payload["grid"].origin_m for *_, nav, _ in rows}
        self.assertGreater(len(origins), 8)  # the grid really moved

    def test_camera_sits_on_the_designed_anchor_cell(self):
        policy = FollowCameraGrid(VOXEL, SHAPE)
        rows, fusion, _ = self.drive(policy)
        for direction, position, _, nav, _ in rows:
            base = fusion.lattice_base(nav.payload["grid"])
            self.assertIsNotNone(base)
            cell = tuple(int(np.floor(p / VOXEL)) for p in position)
            self.assertEqual(tuple(c - b for c, b in zip(cell, base)),
                             EXPECTED_ANCHOR[direction], position)

    def test_following_grid_equals_a_fixed_pipeline_on_the_same_grid(self):
        rows, _, _ = self.drive(FollowCameraGrid(VOXEL, SHAPE))
        for frame_id, (direction, position, transform, nav, _) in enumerate(rows, start=1):
            grid = nav.payload["grid"]
            fixed = M7Pipeline(FrameNativeM7(self.frame_lib), grid)
            t = 5_000_000_000 + frame_id
            identity = CaptureIdentity("cam1", frame_id, t, "mono", "cal-1", "odom", "e1")
            out = fixed.ingest_geometry(
                DepthFrame(identity, depth_for(direction, position), K,
                           PoseAtCapture(t, "mono", "odom", "e1", True, transform)),
                now_ns=t + 1, now_clock_id="mono")
            np.testing.assert_array_equal(out.navigation.payload["voxel_flags"],
                                          nav.payload["voxel_flags"])
            self.assertEqual(out.navigation.payload["valid_grid_samples"],
                             nav.payload["valid_grid_samples"])

    def test_policy_must_match_declared_voxel_and_shape(self):
        with self.assertRaises(ValueError):
            M7Pipeline(FrameNativeM7(self.frame_lib), GridSpec((0., 0., 0.), VOXEL, (16, 16, 8)),
                       grid_policy=FollowCameraGrid(VOXEL, SHAPE))
        with self.assertRaises(ValueError):
            FollowCameraGrid(VOXEL, SHAPE, back_cells=0)

    # -- faults ------------------------------------------------------------
    def test_fault_fixed_grid_stops_fusing_once_the_car_leaves_it(self):
        start = FollowCameraGrid(VOXEL, SHAPE).grid_for(pose_for("x", (0.037, 0.025, 0.025)))
        rows, _, _ = self.drive(policy=None, fixed_grid=start)
        with self.assertRaises(AssertionError):
            self.assert_all_fused(rows)
        self.assertTrue(rows[0][3].reason.startswith("backend_failure"))

    def test_fault_unsnapped_origin_is_dropped_by_the_lattice_check(self):
        rows, fusion, _ = self.drive(UnsnappedPolicy(VOXEL, SHAPE))
        with self.assertRaises(AssertionError):
            self.assert_all_fused(rows)
        self.assertGreater(fusion.counts["dropped:grid_not_on_map_lattice"], 0)

    def test_fault_rounded_camera_cell_misses_the_anchor(self):
        rows, fusion, _ = self.drive(RoundingPolicy(VOXEL, SHAPE))
        misplaced = 0
        for direction, position, _, nav, _ in rows:
            base = fusion.lattice_base(nav.payload["grid"]) if nav.valid else None
            cell = tuple(int(np.floor(p / VOXEL)) for p in position)
            if base is None or tuple(c - b for c, b in zip(cell, base)) != EXPECTED_ANCHOR[direction]:
                misplaced += 1
        self.assertGreater(misplaced, 0)

    def test_fault_flipped_base_is_caught(self):
        rows, fusion, t = self.drive(FlippedPolicy(VOXEL, SHAPE))
        with self.assertRaises(AssertionError):
            self.assert_all_fused(rows)


if __name__ == "__main__":
    unittest.main()
