"""Independent input/geometry controls for hindsight (no external recordings needed)."""
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import unittest
import numpy as np
from tools.car_geo_producer.common import base_camera
from tools.planner_model.hindsight.inputs import Pack, FH, RH, body_poses
from tools.planner_model.hindsight.compare import target, pose_controls, point_cost, evaluate
from tools.planner_model.hindsight.pipeline import generate_car1015
from tools.planner_model.aligned_v3 import PlannerV3
from tools.planner_model.realpose.compare import NavFn
from tools.planner_model.realpose.core import MissingInput


class HindsightTest(unittest.TestCase):
    def test_known_mount_inverse_with_rotation(self):
        mount = base_camera(xyz=(.1,0,.38),pitch_deg=27.3)
        world_body = np.eye(4)
        world_body[:2,:2] = [[0.,-1.],[1.,0.]]
        world_body[:3,3] = [1.2,-2.3,.02]
        actual = body_poses(world_body @ mount, mount)
        np.testing.assert_allclose(actual,world_body,atol=1e-12)
        # Confusing camera translation with body translation has an observable 10cm error.
        self.assertGreater(np.linalg.norm((world_body@mount)[:2,3]-actual[:2,3]),.09)
        invalid=mount.copy();invalid[0,0]=2.
        with self.assertRaises(ValueError):body_poses(world_body,invalid)
        invalid=mount.copy();invalid[:3,0]*=-1
        with self.assertRaises(ValueError):body_poses(world_body,invalid)

    def test_first_future_not_last_or_nearest_to_target(self):
        points=np.array([[0.,0.],[.6,.6],[1.,0.],[0.,1.5]])
        goal,proof=target(points,[-40,-40,-8])
        np.testing.assert_array_equal(goal,[60,40])
        self.assertEqual(proof['future_offset'],2)
        with self.assertRaisesRegex(MissingInput,'no_future'):
            target(points[:2],[-40,-40,-8])

    def test_first_exit_wins_even_if_trajectory_reenters(self):
        # Start 10cm from the valid left edge: a 20cm motion leaves before reaching 1m.
        points=np.array([[0.,0.],[-.2,0.],[.5,0.],[1.5,0.]])
        goal,proof=target(points,[-3,-40,-8])
        np.testing.assert_allclose(goal,[1.,40.])
        self.assertEqual(proof['kind'],'first_trajectory_exit')
        np.testing.assert_allclose(proof['physical_exit'],[0.,40.])

    def test_exact_1m_and_record_end(self):
        g,p=target([[0.,0.],[.999,0.],[1.,0.]],[-40,-40,-8])
        self.assertEqual(p['future_offset'],2)
        with self.assertRaises(MissingInput):target([[0.,0.]],[-40,-40,-8])

    def test_unseeded_control_detects_translation(self):
        cost=np.full((3,80,80),255,np.uint8)
        cost[:,39:42,39:42]=0
        body=np.repeat(np.eye(4)[None],3,axis=0)
        bases=np.tile([-40,-40,-8],(3,1))
        stats,rows=pose_controls(cost,body,bases,np.ones(3,bool))
        self.assertTrue(stats['passed'])
        self.assertEqual(stats['x_positive_increase'],1.)
        # All unknown must fail normal; all free must fail positive. No circular body seed.
        self.assertFalse(pose_controls(np.full_like(cost,255),body,bases,np.ones(3,bool))[0]['passed'])
        self.assertFalse(pose_controls(np.zeros_like(cost),body,bases,np.ones(3,bool))[0]['passed'])
        self.assertEqual(point_cost(cost[0],[10.,0.],bases[0]),-1)

    def test_pack_known_header_and_corruption(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'tiny.pack'
            wc=np.eye(4)[:3]
            header=RH.pack(2,2,3,3,3,0,.001,*np.eye(3).ravel(),*wc.ravel(),-.05,-.05,-.05,.05)
            data=FH.pack(b'FPGAGPK01',1,1,0,0)+header+np.array([[0,1000],[999,65535]],'<u2').tobytes()
            p.write_bytes(data)
            obj=Pack(p);self.assertEqual(len(obj),1)
            np.testing.assert_array_equal(obj.frame(0)['raw'],[[0,1000],[999,65535]])
            self.assertEqual(obj.audit()['unique_matrices'],1)
            for bad in [data[:-1],data+b'x',data[:10],b'badmagic'+data[8:]]:
                p.write_bytes(bad)
                with self.assertRaises(ValueError):Pack(p)

    def test_failed_calibration_blocks_confirmation_before_reading_inputs(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'calibration.json';p.write_text(json.dumps({'passed':False}))
            out=Path(td)/'must_not_exist'
            with self.assertRaisesRegex(ValueError,'calibration_not_passed'):
                generate_car1015(Path(td)/'missing_input',p,out)
            self.assertFalse(out.exists())

    @unittest.skipUnless(os.environ.get('dev2_REALPOSE_NAVFN_LIB') and os.environ.get('dev2_PLANNER_V3_LIB'), 'requires realpose compiled libraries')
    def test_actual_frozen_comparison_counts_tail_rejection_and_outputs(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);body=np.repeat(np.eye(4)[None],7,axis=0)
            body[:,0,3]=np.arange(7)*.2
            maps=np.zeros((7,80,80),np.uint8)
            p=root/'fixture.npz'
            bases=np.tile([-40,-40,-8],(7,1))
            # Follow the body as production rolling windows do; keep the +1m control in bounds.
            bases[:,0]+=np.floor(body[:,0,3]/.05).astype(int)
            np.savez_compressed(p,maps=maps,unseeded=maps,bases=bases,body=body)
            summary,rows,paths=evaluate('rec30',p,None,PlannerV3(os.environ['dev2_PLANNER_V3_LIB']),
                NavFn(os.environ['dev2_REALPOSE_NAVFN_LIB']),root)
            self.assertEqual(summary['requests'],2)
            self.assertEqual(summary['accepted'],1)
            self.assertEqual(summary['rejection_reasons'],{'no_future_point_at_least_1m':1})
            self.assertFalse(summary['real_pose_confirmation'])
            self.assertFalse(summary['pose_calibration'])
            self.assertEqual(rows[0]['future_frame'],5)
            for v in ['v3_1','v3_2']:
                self.assertTrue(rows[0][v]['fixed_equal'])
                self.assertTrue(rows[0][v]['success'])
                self.assertFalse(rows[0][v]['hits'])
            self.assertEqual(len(paths),1)


if __name__=='__main__':unittest.main()
