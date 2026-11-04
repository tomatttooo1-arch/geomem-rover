import itertools
import unittest
from tools.rtabmap_ros_replay import playback_schedule
from host.localization_gate import LocalizationGate
from test_localization_gate import frame


class RepeatTests(unittest.TestCase):
    def test_single_pass_preserves_original_stamps(self):
        self.assertEqual(list(playback_schedule([10,20,30,40],1,3)),[(1,0,20),(2,0,30)])

    def test_repeat_stamps_strictly_increase_and_keep_source_intervals(self):
        rows=list(itertools.islice(playback_schedule([10,20,40],0,3,1800,now=lambda:0.),7))
        self.assertEqual([r[1] for r in rows],[0,0,0,1,1,1,2])
        self.assertTrue(all(b[2]>a[2] for a,b in zip(rows,rows[1:])))
        self.assertEqual(rows[3][2]-rows[2][2],10**9)
        self.assertEqual(rows[5][2]-rows[4][2],20)

    def test_wall_duration_ends_schedule(self):
        counter=iter([0.,0.,.5,1.,1.5,2.])
        self.assertEqual(len(list(playback_schedule([10,20],0,2,2.,now=lambda:next(counter)))),4)

    def test_cycle_limit_and_hard_wall_budget_both_apply(self):
        rows=list(playback_schedule([10,20],0,2,600,now=lambda:0.,cycles=8))
        self.assertEqual(len(rows),16)
        self.assertEqual(rows[-1][1],7)
        counter=iter([0.,0.,.5,1.,1.5,2.])
        self.assertEqual(len(list(playback_schedule([10,20],0,2,2.,now=lambda:next(counter),cycles=8))),4)

    def test_boundary_revokes_old_approval_and_needs_three_new_frames(self):
        gate=LocalizationGate('map',{1})
        for i in (1,2,3):frame(gate,i)
        epoch=gate.epoch
        gate.discontinuity(1)
        self.assertGreater(gate.epoch,epoch)
        self.assertFalse(gate.approved)
        self.assertEqual(gate.reason,'replay_discontinuity:1')
        self.assertFalse(frame(gate,4));self.assertFalse(frame(gate,5))
        self.assertEqual(len(frame(gate,6)),1)


if __name__=='__main__':unittest.main()
