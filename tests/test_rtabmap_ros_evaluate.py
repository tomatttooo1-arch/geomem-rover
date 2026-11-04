"""Known-answer checks for fixed-map scoring, not current-run alignment."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('ros_eval',Path(__file__).parents[1]/'tools/rtabmap_ros_evaluate.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class FixedMapEvaluation(unittest.TestCase):
    def run_case(self, offset=0, covariance=.1, linked=True, lost=False):
        events=[]
        for i in range(3):
            s=(100+i)*10**9
            events += [{'event':'publish_begin','stamp_ns':s,'index':i,'blank':False},
                       {'event':'odom_info','stamp_ns':s,'lost':lost,'covariance':(np.eye(6)*covariance).flatten().tolist()},
                       {'event':'info','stamp_ns':s,'loop_closure_id':8 if linked else 0},
                       {'event':'odom','stamp_ns':s,'header':{'frame_id':'odom'},'child_frame_id':'camera_link',
                        'pose':{'covariance':(np.eye(6)*covariance).flatten().tolist(),
                                'pose':{'position':{'x':float(i),'y':0.,'z':0.},
                                        'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}}}},
                       {'event':'localization_pose','stamp_ns':s,'header':{'frame_id':'map'},
                        'pose':{'covariance':(np.eye(6)*.1).flatten().tolist(),
                                'pose':{'position':{'x':float(i)+offset,'y':0.,'z':0.},
                                        'orientation':{'x':0.,'y':0.,'z':0.,'w':1.}}}}]
        truth=np.array([[100+i,10+i,0,0,0,0,0,1] for i in range(3)])
        align={'alignment_rotation':np.eye(3).tolist(),'alignment_translation_m':[10,0,0]}
        return module.evaluate(events,truth,align)

    def test_known_transform_and_wrong_map(self):
        positive=self.run_case()
        self.assertEqual(positive['first_three_good_localizations'],[0,1,2])
        self.assertEqual(positive['localization_rmse_m'],0)
        wrong=self.run_case(offset=.3)
        self.assertIsNone(wrong['first_three_good_localizations'])
        self.assertAlmostEqual(wrong['localization_rmse_m'],.3)

    def test_unmatched_pose_is_not_localized(self):
        self.assertIsNone(self.run_case(linked=False)['first_three_good_localizations'])

    def test_lost_and_initial_covariance_are_invalid(self):
        key='finite_not_lost_below_invalid_covariance'
        self.assertEqual(self.run_case()['all_frames'][key],3)
        self.assertEqual(self.run_case(covariance=9999)['all_frames'][key],0)
        self.assertEqual(self.run_case(lost=True)['all_frames'][key],0)

    def test_empty_is_not_zero_error(self):
        out=module.evaluate([],np.array([[100,0,0,0]]),
            {'alignment_rotation':np.eye(3).tolist(),'alignment_translation_m':[0,0,0]})
        self.assertIsNone(out['localization_rmse_m'])
        self.assertIsNone(out['first_three_good_localizations'])


if __name__=='__main__': unittest.main()
