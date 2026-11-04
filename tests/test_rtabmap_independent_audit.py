import unittest
import numpy as np
from tools.rtabmap_independent_audit import relative_error, motion_pairs


class RelativeMotionTests(unittest.TestCase):
    def test_global_frame_is_not_needed_but_global_error_is_invisible(self):
        a=np.eye(4);b=np.eye(4);b[0,3]=1
        offset=np.array([[0.,-1,0,7],[1,0,0,-3],[0,0,1,2],[0,0,0,1]])
        translation,angle=relative_error(offset@a,offset@b,a,b)
        self.assertAlmostEqual(translation,0);self.assertAlmostEqual(angle,0)

    def test_optical_extrinsic_is_applied_before_relative_motion(self):
        rows=[]
        for second,x in [(1,0.),(2,1.)]:
            rows.append({'stamp_ns':second*10**9,'pose':{'pose':{
                'position':dict(x=x,y=0.,z=0.),'orientation':dict(x=.5,y=-.5,z=.5,w=.5)}}})
        truth=np.array([[1,0,0,0,0,0,0,1],[2,1,0,0,0,0,0,1]],dtype=float)
        result=motion_pairs(rows,truth)
        self.assertEqual(result['pairs'],1)
        self.assertAlmostEqual(result['translation_rmse_m'],0)
        self.assertAlmostEqual(result['rotation_rmse_deg'],0)

    def test_wrong_motion_and_rotation_are_detected(self):
        a=np.eye(4);b=np.eye(4);b[0,3]=1
        wrong=b.copy();wrong[0,3]=2
        self.assertAlmostEqual(relative_error(a,wrong,a,b)[0],1)
        wrong[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        self.assertAlmostEqual(relative_error(a,wrong,a,b)[1],90)


if __name__=='__main__':unittest.main()
