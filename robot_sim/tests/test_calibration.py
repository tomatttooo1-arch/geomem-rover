import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from geometry_calibration import ray_box, plane_samples


class CalibrationTests(unittest.TestCase):
    def test_depth_is_ray_parameter_not_euclidean_length(self):
        self.assertEqual(ray_box([0,0,2],[0,0,-2],[-1,-1,0],[1,1,1]), (.5,2))
        self.assertEqual(ray_box([0,0,0],[2,0,0],[4,-1,-1],[6,1,1]), (2.0,0))

    def test_miss_and_behind_are_not_hits(self):
        self.assertIsNone(ray_box([2,0,2],[0,0,-1],[-1,-1,0],[1,1,1]))
        self.assertIsNone(ray_box([0,0,2],[0,0,1],[-1,-1,0],[1,1,1]))

    def test_fixed_pixels_hit_different_architectural_planes(self):
        pose=SimpleNamespace(position=SimpleNamespace(x=0,y=0,z=0),
                             orientation=SimpleNamespace(x=0,y=0,z=0,w=1))
        k=[277.12812920993167,0,160,0,277.12812920993167,120,0,0,1]
        world=Path(__file__).resolve().parents[1]/'fpga_sim/worlds/room.sdf'
        samples=plane_samples(world,k,pose)
        self.assertEqual([s['plane_axis'] for s in samples],[2,0])
        # Independent plane equations from configured tabletop/wall positions.
        d=math.sin(.25)+math.cos(.25)*(90-120)/k[4]
        self.assertAlmostEqual(samples[0]['expected_optical_z_m'],(.95-.7)/d)
        d=math.cos(.25)-math.sin(.25)*(40-120)/k[4]
        self.assertAlmostEqual(samples[1]['expected_optical_z_m'],(2.9-.18)/d)


if __name__=='__main__':unittest.main()
