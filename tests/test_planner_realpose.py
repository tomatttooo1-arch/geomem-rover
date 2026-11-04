"""Offline input matching and continuous-window controls, without ROS dependencies."""
import math
import unittest
import numpy as np
from tools.planner_model.realpose.core import (MissingInput,Transforms,first_exit,select_map,to_raw,from_raw,transform_xy)


def tf(parent,child,stamp,x=0.,yaw=0.,static=False):
 return dict(parent=parent,child=child,stamp_ns=stamp,translation=[x,0.,0.],
             rotation=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)],static=static)


class RealposeCoreTest(unittest.TestCase):
 def test_tf_chain_interpolation_rotation_and_inverse(self):
  store=Transforms([tf('odom','base_link',1000000000,1.),tf('odom','base_link',1100000000,2.),
                    tf('map','odom',0,3.,math.pi/2,True)])
  m,e=store.lookup('map','base_link',1050000000)
  np.testing.assert_allclose(transform_xy(m,[[0,0]]),[[3,1.5]],atol=1e-12)
  inv,_=store.lookup('base_link','map',1050000000)
  np.testing.assert_allclose(inv.dot(m),np.eye(4),atol=1e-12)
  self.assertEqual(len(e),2)
 def test_no_extrapolation_or_wide_interpolation(self):
  store=Transforms([tf('odom','base_link',1000000000),tf('odom','base_link',1300000000)])
  for stamp in [900000000,1400000000,1150000000]:
   with self.assertRaises(MissingInput):store.lookup('odom','base_link',stamp)
 def test_ambiguous_tf_authority_rejected(self):
  with self.assertRaises(MissingInput):Transforms([tf('map','base_link',1),tf('odom','base_link',2)])
  with self.assertRaises(MissingInput):Transforms([tf('odom','base_link',1,1.),tf('odom','base_link',1,2.)])
 def test_tf_quaternion_sign_equivalence(self):
  a=tf('odom','base_link',1000000000,yaw=math.pi/2)
  b=tf('odom','base_link',1100000000,yaw=math.pi/2);b['rotation']=[-x for x in b['rotation']]
  m,_=Transforms([a,b]).lookup('odom','base_link',1050000000)
  np.testing.assert_allclose(transform_xy(m,[[1,0]]),[[0,1]],atol=1e-12)
 def test_map_never_future_and_age_limit(self):
  grid=dict(stamp_ns=1000000000,width=80,height=80,resolution=.05)
  self.assertEqual(select_map([grid,dict(grid,stamp_ns=1400000000)],1100000000),grid)
  for stamp in [900000000,1200000001]:
   with self.assertRaises(MissingInput):select_map([grid],stamp)
  with self.assertRaises(MissingInput):select_map([dict(grid,width=100)],1100000000)
 def test_first_exit_preserves_path_direction(self):
  np.testing.assert_allclose(first_exit([[40,40],[70,60],[90,70]],[1,1],[78,78]),[78,64])
  np.testing.assert_allclose(first_exit([[40,40],[-10,40]],[1,1],[78,78]),[1,40])
  with self.assertRaises(MissingInput):first_exit([[90,20],[40,40]],[1,1],[78,78])
  with self.assertRaises(MissingInput):first_exit([[40,40],[50,50]],[1,1],[78,78])
 def test_origin_rotation_roundtrip_for_outside_points(self):
  grid=dict(origin=[10.,-3.,math.pi/2],resolution=.05)
  raw=np.array([[40.,40.],[90.,-10.],[-1.,120.]])
  np.testing.assert_allclose(to_raw(grid,from_raw(grid,raw)),raw,atol=1e-12)
 def test_changing_static_transform_rejected(self):
  with self.assertRaises(MissingInput):Transforms([tf('map','odom',0,1.,static=True),tf('map','odom',0,2.,static=True)])



# These controls require the actual compiled libraries; ordinary full-suite discovery skips them.
import os
from pathlib import Path
from tools.planner_model.evaluate import Planner
from tools.planner_model.realpose.compare import NavFn,choose_goal,verify_frozen


@unittest.skipUnless(os.environ.get('dev2_REALPOSE_NAVFN_LIB') and os.environ.get('dev2_PLANNER_V3_LIB'), 'requires realpose compiled libraries')
class RealposeCompiledTest(unittest.TestCase):
 def setUp(self):
  self.nav=NavFn(os.environ['dev2_REALPOSE_NAVFN_LIB'])
  self.old=Planner(os.environ['dev2_PLANNER_V3_LIB'])
 def test_integer_wrapper_exactly_matches_existing_baseline(self):
  m=np.zeros((48,48),np.uint8);m[:,24]=255
  for start in [(8,20),(38,20)]:
   for goal in [(25,20),(26,20),(27,20),(23,20),(22,20),(21,20),(24,20)]:
    a,_=self.old.run(m,start,goal,'navfn');b=self.nav.plan(m,start,goal,.05)
    np.testing.assert_array_equal(a,b)
 def test_fractional_goal_is_preserved(self):
  m=np.zeros((48,48),np.uint8)
  r=self.nav.plan(m,(8,20),(38.25,30.1),.05)
  self.assertTrue(len(r));np.testing.assert_array_equal(r[-1],[38.25,30.1])
 def test_malformed_requests_count_as_rejected_without_proxy_fallback(self):
  import json,subprocess,sys,tempfile
  with tempfile.TemporaryDirectory(prefix='dev2-realpose-negative-') as tmp:
   root=Path(tmp);src=root/'input';src.mkdir()
   packet=dict(source_kind='test_fixture',request_topic='/dev2/planner_request',
       counts={'/dev2/planner_request':1},decode_errors=[dict(topic='/dev2/planner_request',received_ns=1,reason='missing_source_timestamp')])
   (src/'extraction.json').write_text(json.dumps(packet))
   (src/'events.jsonl').write_text(json.dumps(dict(kind='plan',epoch=0,stamp_ns=1000000000,frame_id='odom',points=[[0.,0.],[1.,0.]]))+'\n')
   result=subprocess.run([sys.executable,'-m','tools.planner_model.realpose.compare','--input',str(src),
       '--lib',os.environ['dev2_PLANNER_V3_LIB'],'--navfn-lib',os.environ['dev2_REALPOSE_NAVFN_LIB'],'--out',str(root/'result')],
       stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
   self.assertEqual(result.returncode,0,result.stderr)
   report=json.loads((root/'result/summary.json').read_text())
   self.assertEqual(report['requests'],1);self.assertEqual(report['rejected'],1);self.assertEqual(report['accepted'],0)
   self.assertFalse(report['real_pose_confirmation'])
 def test_frozen_algorithm_unchanged(self):verify_frozen()
 def test_outside_target_uses_associated_path(self):
  grid=dict(frame_id='odom',origin=[0.,0.,0.],resolution=.05)
  req=dict(stamp_ns=1000000000,frame_id='odom',goal=[5.,2.])
  plan=dict(stamp_ns=1050000000,frame_id='odom',points=[[2.,2.],[3.,2.],[5.,2.]])
  goal,proof=choose_goal(grid,req,[plan],Transforms([]),np.array([2.,2.]))
  np.testing.assert_allclose(goal,[3.9,2.]);np.testing.assert_allclose(proof['physical_exit_grid'],[80.,40.]);self.assertEqual(proof['kind'],'global_path_exit')
  with self.assertRaises(MissingInput):choose_goal(grid,req,[],Transforms([]),np.array([2.,2.]))

if __name__=='__main__':unittest.main()
