import struct
import json
import numpy as np
import unittest
from types import SimpleNamespace as O
from host.cloud_geometry import check_cloud_depth


def fixture():
    h=lambda f: O(stamp=O(sec=1,nanosec=0),frame_id=f)
    cloud=O(header=h('camera_link'),width=1,height=1,point_step=16,row_step=16,is_bigendian=False,
            fields=[O(name=n,offset=i*4,count=1,datatype=7) for i,n in enumerate(('x','y','z'))],
            data=struct.pack('<fffI',2.,.5,-.25,42))
    depth=O(header=h('camera_optical_frame'),width=1,height=1,encoding='32FC1',step=4,is_bigendian=False,data=struct.pack('<f',2.))
    info=O(header=h('camera_optical_frame'),k=[4.,0.,1.,0.,4.,-.5,0.,0.,1.])
    return cloud,depth,info


class GeometryTests(unittest.TestCase):
    def test_forward_cloud_matches_independent_depth_projection(self):
        self.assertTrue(check_cloud_depth(*fixture(),pixels=((0,0),))['passed'])
    def test_old_wrong_optical_label_is_rejected(self):
        c,d,i=fixture();c.header.frame_id='camera_optical_frame'
        self.assertFalse(check_cloud_depth(c,d,i,pixels=((0,0),))['passed'])
    def test_unaligned_and_unknown_frames_are_rejected(self):
        c,d,i=fixture();c.header.stamp.sec=2
        with self.assertRaises(ValueError):check_cloud_depth(c,d,i,pixels=((0,0),))
        c,d,i=fixture();c.header.frame_id='unknown'
        with self.assertRaises(ValueError):check_cloud_depth(c,d,i,pixels=((0,0),))
    def test_ros_numpy_intrinsics_produce_json_serializable_evidence(self):
        c,d,i=fixture();i.k=np.array(i.k,dtype=np.float64)
        result=check_cloud_depth(c,d,i,pixels=((0,0),))
        self.assertTrue(json.loads(json.dumps(result,allow_nan=False))["passed"])
    def test_invalid_depth_cannot_pass(self):
        c,d,i=fixture();d.data=struct.pack('<f',float('nan'))
        self.assertFalse(check_cloud_depth(c,d,i,pixels=((0,0),))['passed'])
