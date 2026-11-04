"""Input alignment contract; these tests need no compiled model or ROS."""
from dataclasses import replace
import math
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from tools.planner_model.aligned_v2 import (InputContractError, MapMetadata, PoseSample,
                                           checked_map, grid_to_world, world_to_grid)


from tools.planner_model.plan_v2_request import load_request


class AlignmentContractTest(unittest.TestCase):
    def setUp(self):
        self.meta = MapMetadata('odom', 23, 900, .05, 10., -3., math.pi/2, 'cell_center')
        self.pose = PoseSample('odom', 23, 900, 8., -2., 'test_fixture')

    def test_rotated_translated_origin_roundtrip(self):
        points = np.array([[10., 20.], [30.25, 40.125]])
        world = grid_to_world(self.meta, points)
        for p, w in zip(points, world):
            _, raw = world_to_grid(self.meta, w, (80, 80))
            np.testing.assert_allclose(raw, p, atol=1e-12)
        self.assertEqual(world_to_grid(self.meta, grid_to_world(self.meta, [[10., 20.]])[0], (80, 80))[0], (10, 20))

    def test_standard_floor_versus_explicit_navfn_round(self):
        meta = replace(self.meta, origin_x=0., origin_y=0., origin_yaw=0.)
        self.assertEqual(world_to_grid(meta, (.079, .079), (80, 80))[0], (1, 1))
        self.assertEqual(world_to_grid(replace(meta, coordinates='navfn_round'), (.079, .079), (80, 80))[0], (2, 2))

    def test_no_frame_sequence_or_timestamp_guessing(self):
        self.pose.validate(self.meta)
        for field, value in [('frame_id', 'map'), ('sequence', 24), ('stamp_ns', 901), ('provenance', 'unknown')]:
            with self.assertRaises(InputContractError):
                replace(self.pose, **{field: value}).validate(self.meta)

    def test_map_file_checksum_binds_request_to_snapshot(self):
        with tempfile.TemporaryDirectory(prefix='dev2-alignment-test-') as tmp:
            p = Path(tmp)
            np.save(p/'map.npy', np.zeros((8, 8), np.uint8))
            packet = dict(metadata=self.meta.__dict__, pose=self.pose.__dict__,
                          goal=dict(x=1., y=2., frame_id='odom'),
                          map_sha256=hashlib.sha256((p/'map.npy').read_bytes()).hexdigest())
            (p/'request.json').write_text(json.dumps(packet))
            self.assertEqual(load_request(p/'map.npy', p/'request.json')[0].shape, (8, 8))
            m = np.zeros((8, 8), np.uint8);m[2, 2] = 254
            np.save(p/'map.npy', m)
            with self.assertRaisesRegex(InputContractError, 'map_checksum_mismatch'):
                load_request(p/'map.npy', p/'request.json')

    def test_boolean_and_fractional_ids_are_not_timestamps(self):
        for field, value in [('sequence', True), ('stamp_ns', 900.), ('x', True)]:
            with self.assertRaises(InputContractError):
                replace(self.pose, **{field: value}).validate(self.meta)
        with self.assertRaises(InputContractError):
            replace(self.meta, resolution=True).validate()

    def test_reject_nonfinite_and_outside(self):
        for point in [(float('nan'), 0.), (1e6, 1e6)]:
            with self.assertRaises(InputContractError):
                world_to_grid(self.meta, point, (80, 80))
        with self.assertRaises(InputContractError):
            replace(self.meta, resolution=0.).validate()

    def test_cost_values_never_wrap_to_uint8(self):
        for values in [np.full((8, 8), -1), np.full((8, 8), 256), np.full((8, 8), .5)]:
            with self.assertRaises(InputContractError):
                checked_map(values)
        source = np.zeros((8, 8), np.uint8)
        copy = checked_map(source);source[2, 2] = 254
        self.assertEqual(copy[2, 2], 0)


if __name__ == '__main__':
    unittest.main()
