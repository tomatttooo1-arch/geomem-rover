"""Calibrate the new ROS probe's time/frame boundaries, without running ROS."""
import unittest
from tools.rtabmap_nav_continuity_ros_test import fresh_ready, fresh_permit

class ObserverTests(unittest.TestCase):
    def row(self, stamp=2., ready=True):
        return dict(event='frame_processed', input_ready=ready, acquisition_timestamp=stamp)
    def test_new_ready_positive(self):
        self.assertTrue(fresh_ready([(2., self.row())], 1.5, 2000000000))
    def test_old_receipt_and_old_frame_rejected(self):
        self.assertFalse(fresh_ready([(1., self.row())], 1.5, 2000000000))
        self.assertFalse(fresh_ready([(2., self.row(1.))], 1.5, 2000000000))
    def test_new_not_ready_rejected(self):
        self.assertFalse(fresh_ready([(2., self.row(ready=False))], 1.5, 2000000000))
    def test_permit_requires_new_true(self):
        self.assertTrue(fresh_permit([(2., True)], 1.5))
        self.assertFalse(fresh_permit([(1., True),(2., False)], 1.5))
    def test_ready_followed_by_fault_rejected(self):
        self.assertFalse(fresh_ready([(2.,self.row()),(2.1,dict(event='fault'))],1.5,2000000000))
    def test_new_ready_after_fault_allowed(self):
        self.assertTrue(fresh_ready([(1.8,dict(event='fault')),(2.,self.row())],1.5,2000000000))

class SequenceAndClockTests(unittest.TestCase):
    def report(self, values):
        return dict(events=[dict(event='status_schedule',wall=0.,delivered=True,instance='A',epoch=0,stamp_ns=1),
                            dict(event='scenario_boundary',wall=1.),
                            dict(event='status_schedule',wall=2.,delivered=True,instance='A',epoch=1,stamp_ns=2)],
                    runtime=[(3.,dict(event='frame_processed',frame_id='rgb-2')),
                             (5.,dict(event='navigation_result')),(6.,dict(event='navigation_started'))],
                    permits=[(3.1+i*.1,v) for i,v in enumerate(values)])
    def test_entire_sequence_detects_revival_despite_final_false(self):
        from tools.rtabmap_nav_continuity_ros_test import continuity_window
        self.assertFalse(continuity_window(self.report([False,True,False]))['passed'])
        self.assertTrue(continuity_window(self.report([False,False,False]))['passed'])
    def test_publisher_without_consumed_frame_is_not_evidence(self):
        from tools.rtabmap_nav_continuity_ros_test import continuity_window
        r=self.report([False]);r['runtime'][0]=(3.,dict(event='frame_processed',frame_id='rgb-1'))
        with self.assertRaises(ValueError):continuity_window(r)
    def test_stage_accounting_and_clock_regression(self):
        from tools.rtabmap_nav_continuity_ros_test import stage_durations
        s=[dict(stage=str(i),wall=float(i),thread_cpu=i*.1) for i in range(3)]
        r=stage_durations(s)
        self.assertAlmostEqual(sum(e['wall_seconds'] for e in r),s[-1]['wall']-s[0]['wall'])
        self.assertAlmostEqual(sum(e['cpu_seconds'] for e in r),s[-1]['thread_cpu']-s[0]['thread_cpu'])
        with self.assertRaises(ValueError):stage_durations(list(reversed(s)))
    def test_wall_and_thread_cpu_distinguish_sleep_from_work(self):
        import time
        wall,cpu=time.monotonic(),time.thread_time();time.sleep(.02)
        slept=(time.monotonic()-wall,time.thread_time()-cpu)
        wall,cpu=time.monotonic(),time.thread_time()
        while time.thread_time()-cpu<.02:pass
        worked=(time.monotonic()-wall,time.thread_time()-cpu)
        self.assertGreater(slept[0],slept[1]);self.assertGreater(worked[1],slept[1])
