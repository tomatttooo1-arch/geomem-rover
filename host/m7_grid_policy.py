"""Per-frame grid that follows the camera on the rolling-map lattice (2026-09-27).

See docs/design/M7_FOLLOW_GRID_20260927.md. The per-frame kernel needs the
camera inside its <=32^3 grid; a fixed grid stops working as soon as the car
leaves it. ``FollowCameraGrid`` puts the camera at a fixed anchor cell of each
frame's grid, with most of the block ahead of the camera, and snaps the origin
to an integer number of voxels so the rolling map's lattice check passes.
"""

import math

import numpy as np

from host.m7_pipeline import GridSpec


class FollowCameraGrid:
    def __init__(self, voxel_m, shape, *, back_cells=1, height_cell=1):
        if not isinstance(voxel_m, (int, float)) or not math.isfinite(voxel_m) or voxel_m <= 0:
            raise ValueError("voxel_m must be positive and finite")
        if (len(shape) != 3 or any(type(n) is not int or not 3 <= n <= 32 for n in shape)):
            raise ValueError("shape needs three integers in 3..32")
        nx, ny, nz = shape
        # At least one cell from every face: the kernel quantises the camera
        # from float32, the host from float64; a boundary case may differ by one.
        if type(back_cells) is not int or not 1 <= back_cells <= min(nx, ny) - 2:
            raise ValueError("back_cells must keep the camera at least one cell inside")
        if type(height_cell) is not int or not 1 <= height_cell <= nz - 2:
            raise ValueError("height_cell must keep the camera at least one cell inside")
        self.voxel_m, self.shape = float(voxel_m), tuple(shape)
        self.back_cells, self.height_cell = back_cells, height_cell

    def cell_of(self, position_m):
        """Map lattice cell containing a local-frame position (float64 floor)."""
        return tuple(int(v) for v in np.floor(np.asarray(position_m, np.float64) / self.voxel_m))

    def anchor_for(self, rotation):
        """Camera cell inside the block, chosen from the horizontal optical axis."""
        nx, ny, _ = self.shape
        axis = np.asarray(rotation, np.float64)[:, 2]
        back = self.back_cells
        if abs(axis[0]) >= abs(axis[1]):
            x = back if axis[0] >= 0 else nx - 1 - back
            return (x, ny // 2, self.height_cell)
        y = back if axis[1] >= 0 else ny - 1 - back
        return (nx // 2, y, self.height_cell)

    def base_for(self, transform):
        t = np.asarray(transform, np.float64)
        cell, anchor = self.cell_of(t[:3, 3]), self.anchor_for(t[:3, :3])
        return tuple(c - a for c, a in zip(cell, anchor))

    def grid_for(self, transform):
        base = self.base_for(transform)
        return GridSpec(origin_m=tuple(float(b * self.voxel_m) for b in base),
                        voxel_m=self.voxel_m, shape=self.shape)
