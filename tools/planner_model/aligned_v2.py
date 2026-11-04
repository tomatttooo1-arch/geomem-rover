"""Explicit pose/costmap contract and standalone C++ planner v2 bindings. No ROS."""
import ctypes
from dataclasses import dataclass
import math
import numbers
from pathlib import Path

import numpy as np
from tools.planner_model.evaluate import Geometry, U8, F64

STATUSES = ['exact', 'within_tolerance', 'invalid_start', 'invalid_goal', 'no_path',
            'safety_rejected', 'invalid_arguments', 'not_converged']


def finite_number(value):
    return isinstance(value, numbers.Real) and not isinstance(value, (bool, np.bool_)) and math.isfinite(value)


class InputContractError(ValueError):
    pass


@dataclass(frozen=True)
class MapMetadata:
    frame_id: str
    sequence: int
    stamp_ns: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    coordinates: str

    def validate(self):
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise InputContractError('missing_map_frame')
        for field in ['sequence', 'stamp_ns']:
            v = getattr(self, field)
            if isinstance(v, (bool, np.bool_)) or not isinstance(v, numbers.Integral) or v < 0:
                raise InputContractError('invalid_map_'+field)
        if not all(finite_number(v) for v in [self.resolution, self.origin_x, self.origin_y, self.origin_yaw]):
            raise InputContractError('nonfinite_map_metadata')
        if self.resolution < 1e-6:
            raise InputContractError('invalid_resolution')
        if self.coordinates not in ['cell_center', 'navfn_round']:
            raise InputContractError('explicit_coordinate_convention_required')


@dataclass(frozen=True)
class PoseSample:
    frame_id: str
    sequence: int
    stamp_ns: int
    x: float
    y: float
    provenance: str

    def validate(self, meta):
        for field in ['sequence', 'stamp_ns']:
            value = getattr(self, field)
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Integral) or value < 0:
                raise InputContractError('invalid_pose_'+field)
        if self.frame_id != meta.frame_id:
            raise InputContractError('pose_frame_mismatch')
        if self.sequence != meta.sequence:
            raise InputContractError('pose_sequence_mismatch')
        if self.stamp_ns != meta.stamp_ns:
            raise InputContractError('pose_timestamp_mismatch')
        if not all(finite_number(v) for v in [self.x, self.y]):
            raise InputContractError('nonfinite_pose')
        if self.provenance not in ['recorded_pose', 'synthetic_pose', 'test_fixture']:
            raise InputContractError('explicit_pose_provenance_required')


def checked_map(m):
    a = np.asarray(m)
    if a.ndim != 2 or min(a.shape) < 3 or max(a.shape) > 1024:
        raise InputContractError('invalid_map_dimensions')
    if not np.issubdtype(a.dtype, np.integer) or np.any(a < 0) or np.any(a > 255):
        raise InputContractError('invalid_cost_values')
    return np.array(a, dtype=np.uint8, order='C', copy=True)


def world_to_grid(meta, point, shape):
    meta.validate()
    if len(point) != 2 or not all(finite_number(v) for v in point):
        raise InputContractError('nonfinite_world_point')
    c, s = math.cos(meta.origin_yaw), math.sin(meta.origin_yaw)
    x, y = point[0]-meta.origin_x, point[1]-meta.origin_y
    q = np.array([c*x+s*y, -s*x+c*y])/meta.resolution
    h, w = shape
    if np.any(q < 0):
        raise InputContractError('world_point_outside_map')
    if meta.coordinates == 'cell_center':
        if q[0] >= w or q[1] >= h:
            raise InputContractError('world_point_outside_map')
        cell = np.floor(q).astype(int)
        raw = q-.5
    else:
        cell = np.floor(q+.5).astype(int)  # C++ round for nonnegative map coordinates
        raw = q
    if not (0 <= cell[0] < w and 0 <= cell[1] < h):
        raise InputContractError('world_point_outside_map')
    return tuple(map(int, cell)), raw


def grid_to_world(meta, points):
    p = np.asarray(points, dtype=float).reshape((-1, 2))
    offset = .5 if meta.coordinates == 'cell_center' else 0.
    local = (p+offset)*meta.resolution
    c, s = math.cos(meta.origin_yaw), math.sin(meta.origin_yaw)
    return local.dot(np.array([[c, s], [-s, c]]))+np.array([meta.origin_x, meta.origin_y])


class PlannerV2:
    def __init__(self, library):
        self.lib = ctypes.CDLL(str(Path(library).resolve()))
        self.lib.wave_plan_v2.argtypes = [U8] + [ctypes.c_int]*4 + [ctypes.c_double]*4 + [ctypes.c_int] + [
            F64, F64, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_int), F64]
        self.lib.wave_plan_v2.restype = ctypes.c_int

    def plan_cells(self, m, start, goal, resolution=.05, tolerance=.15, mode='fixed'):
        m = checked_map(m)
        if mode not in ['float', 'fixed']:
            raise InputContractError('unknown_numeric_mode')
        if len(start) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) for v in start):
            raise InputContractError('integer_start_required')
        if len(goal) != 2 or not all(finite_number(v) for v in goal):
            raise InputContractError('invalid_goal_coordinates')
        if not finite_number(resolution) or not finite_number(tolerance):
            raise InputContractError('invalid_resolution_or_tolerance')
        h, w = m.shape
        cap = w*h+2
        x, y, resolved = np.zeros(cap), np.zeros(cap), np.full(2, np.nan)
        rounds, status = ctypes.c_uint32(), ctypes.c_int(6)
        n = self.lib.wave_plan_v2(m, w, h, int(start[0]), int(start[1]), float(goal[0]), float(goal[1]),
                                 resolution, tolerance, int(mode == 'fixed'), x, y, cap,
                                 ctypes.byref(rounds), ctypes.byref(status), resolved)
        name = STATUSES[status.value]
        path = np.column_stack((x[:max(0, n)], y[:max(0, n)]))
        return dict(status=name, success=n>0, path=path, rounds=rounds.value,
                    requested_goal=np.asarray(goal, float),
                    resolved_goal=resolved.copy() if n>0 else None)

    def plan(self, m, meta, pose, goal_world, goal_frame_id, mode='fixed', tolerance=.15):
        # Snapshot before planning; no shared array mutation or implicit TF lookup.
        m = checked_map(m)
        meta.validate()
        if pose is None:
            raise InputContractError('missing_pose')
        pose.validate(meta)
        if goal_frame_id != meta.frame_id:
            raise InputContractError('goal_frame_mismatch')
        start, actual_start = world_to_grid(meta, (pose.x, pose.y), m.shape)
        _, raw_goal = world_to_grid(meta, goal_world, m.shape)
        result = self.plan_cells(m, start, raw_goal, meta.resolution, tolerance, mode)
        if result['success']:
            # Include the segment from the real requested pose to its representative cell.
            if not np.array_equal(actual_start, result['path'][0]):
                result['path'] = np.vstack((actual_start, result['path']))
            if Geometry(m).collisions(result['path']):
                result.update(status='safety_rejected', success=False, path=np.empty((0, 2)), resolved_goal=None)
        result.update(frame_id=meta.frame_id, sequence=meta.sequence, stamp_ns=meta.stamp_ns,
                      pose_provenance=pose.provenance, start_cell=start,
                      requested_goal_world=np.asarray(goal_world, float))
        result['world_path'] = grid_to_world(meta, result['path'])
        result['resolved_goal_world'] = (grid_to_world(meta, [result['resolved_goal']])[0]
                                         if result['success'] else None)
        return result
