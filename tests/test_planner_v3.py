"""Real compiled v3 integration controls; set dev2_PLANNER_V3_LIB to the built library."""
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from tools.planner_model.aligned_v2 import MapMetadata, PoseSample, InputContractError, grid_to_world
from tools.planner_model.aligned_v3 import PlannerV3
from tools.planner_model.evaluate import Geometry


@unittest.skipUnless(os.environ.get('dev2_PLANNER_V3_LIB'), 'requires compiled v3 library')
class PlannerV3IntegrationTest(unittest.TestCase):
    def setUp(self):
        self.lib=Path(os.environ['dev2_PLANNER_V3_LIB']).resolve()
        self.planner=PlannerV3(self.lib)
        self.meta=MapMetadata('odom',12,123000,.05,10.,-3.,np.pi/2,'cell_center')
        self.start=np.array([10.2,10.1]);self.goal=np.array([23.25,16.1])
        self.sw,self.gw=grid_to_world(self.meta,[self.start,self.goal])
        self.pose=PoseSample('odom',12,123000,float(self.sw[0]),float(self.sw[1]),'test_fixture')

    def test_actual_pose_and_fractional_goal_after_clearing(self):
        m=np.zeros((48,48),np.uint8);m[10,10]=253
        before=m.copy()
        r=self.planner.plan(m,self.meta,self.pose,self.gw,'odom')
        self.assertTrue(r['success'])
        np.testing.assert_allclose(r['world_path'][0],self.sw)
        np.testing.assert_allclose(r['world_path'][-1],self.gw)
        np.testing.assert_array_equal(m,before)
        self.assertTrue(r['original_map_hits'])
        self.assertTrue(all((x,y)==(10,10) for x,y,c in r['original_map_hits']))

    def test_frame_time_and_origin_contract_still_enforced(self):
        m=np.zeros((48,48),np.uint8)
        for pose in [None,replace(self.pose,frame_id='map'),replace(self.pose,stamp_ns=123001),replace(self.pose,sequence=13)]:
            with self.assertRaises(InputContractError):self.planner.plan(m,self.meta,pose,self.gw,'odom')
        with self.assertRaises(InputContractError):self.planner.plan(m,self.meta,self.pose,self.gw,'map')

    def test_pose_contact_with_other_blocked_cell_rejected(self):
        m=np.zeros((48,48),np.uint8);m[10,10]=253;m[10,9]=254
        meta=replace(self.meta,origin_x=0.,origin_y=0.,origin_yaw=0.,coordinates='cell_center')
        # Actual pose at x=10.0 lies on the closed boundary of cells 9 and 10.
        pose=replace(self.pose,x=.5,y=.525)
        r=self.planner.plan(m,meta,pose,(1.5,1.5),'odom')
        self.assertFalse(r['success']);self.assertEqual(r['status'],'safety_rejected')

    def test_smoothed_float_fixed_world_paths_match(self):
        m=np.zeros((48,48),np.uint8);m[20:23,20:25]=254
        a=self.planner.plan(m,self.meta,self.pose,self.gw,'odom',mode='float')
        b=self.planner.plan(m,self.meta,self.pose,self.gw,'odom',mode='fixed')
        self.assertTrue(a['success']);np.testing.assert_array_equal(a['world_path'],b['world_path'])
        self.assertFalse(Geometry(m).collisions(a['path']))

    def test_enclosed_start_and_original_blocked_goal_rejected(self):
        m=np.zeros((48,48),np.uint8);m[9:12,9:12]=254
        self.assertEqual(self.planner.plan_cells(m,(10,10),(35,35))['status'],'no_path')
        self.assertEqual(self.planner.plan_cells(m,(10,10),(10,10))['status'],'invalid_goal')

    def test_file_cli_positive_and_hash_negative(self):
        with tempfile.TemporaryDirectory(prefix='dev2-v3-cli-') as tmp:
            p=Path(tmp);m=np.zeros((48,48),np.uint8);m[10,10]=253
            np.save(p/'map.npy',m)
            request=dict(map_sha256=hashlib.sha256((p/'map.npy').read_bytes()).hexdigest(),
                metadata=self.meta.__dict__,pose=self.pose.__dict__,goal=dict(frame_id='odom',x=float(self.gw[0]),y=float(self.gw[1])))
            (p/'request.json').write_text(json.dumps(request))
            cmd=[sys.executable,'-m','tools.planner_model.plan_v3_request','--map',str(p/'map.npy'),
                '--request',str(p/'request.json'),'--lib',str(self.lib),'--out',str(p/'result.json')]
            result=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            packet=json.loads((p/'result.json').read_text())
            self.assertTrue(packet['success']);self.assertEqual(packet['start_policy'],'navfn_clear_cell')
            self.assertTrue(packet['original_map_hits'])
            request['map_sha256']='0'*64;(p/'request.json').write_text(json.dumps(request))
            cmd[-1]=str(p/'bad-result.json')
            result=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            self.assertEqual(result.returncode,2);self.assertFalse((p/'bad-result.json').exists())


if __name__=='__main__':unittest.main()
