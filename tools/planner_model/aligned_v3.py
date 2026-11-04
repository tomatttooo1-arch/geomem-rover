"""NavFn-compatible start handling and weighted, clearance-preserving CPU planner."""
import ctypes
from pathlib import Path

import numpy as np
from tools.planner_model.evaluate import Geometry, U8, F64

from tools.planner_model.aligned_v2 import (STATUSES, InputContractError, checked_map,
    finite_number, MapMetadata, PoseSample, world_to_grid, grid_to_world)


class PlannerV3:
    def __init__(self, library):
        self.lib = ctypes.CDLL(str(Path(library).resolve()))
        self.lib.wave_plan_v3.argtypes = [U8] + [ctypes.c_int]*4 + [ctypes.c_double]*4 + [ctypes.c_int] + [
            F64, F64, ctypes.c_int, ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_int), F64, ctypes.c_int]
        self.lib.wave_plan_v3.restype = ctypes.c_int

    def plan_cells(self, m, start, goal, resolution=.05, tolerance=.15, mode='fixed', variant=2):
        m = checked_map(m)
        if isinstance(variant, bool) or variant not in [0, 1, 2]:
            raise InputContractError('unknown_variant')
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
        n = self.lib.wave_plan_v3(m, w, h, int(start[0]), int(start[1]), float(goal[0]), float(goal[1]),
                                 resolution, tolerance, int(mode == 'fixed'), x, y, cap,
                                 ctypes.byref(rounds), ctypes.byref(status), resolved, variant)
        name = STATUSES[status.value]
        path = np.column_stack((x[:max(0, n)], y[:max(0, n)]))
        return dict(status=name, success=n>0, path=path, rounds=rounds.value,
                    variant=variant, start_policy="navfn_clear_cell",
                    original_start_cost=int(m[start[1],start[0]]) if 0<=start[0]<w and 0<=start[1]<h else None,
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
            effective = m.copy();effective[start[1],start[0]] = 0
            if Geometry(effective).collisions(result['path']):
                result.update(status='safety_rejected', success=False, path=np.empty((0, 2)), resolved_goal=None)
        result.update(frame_id=meta.frame_id, sequence=meta.sequence, stamp_ns=meta.stamp_ns,
                      pose_provenance=pose.provenance, start_cell=start,
                      requested_goal_world=np.asarray(goal_world, float))
        result['original_map_hits'] = Geometry(m).collisions(result['path'])
        result['world_path'] = grid_to_world(meta, result['path'])
        result['resolved_goal_world'] = (grid_to_world(meta, [result['resolved_goal']])[0]
                                         if result['success'] else None)
        return result
