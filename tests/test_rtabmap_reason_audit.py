import copy
import unittest
from host.localization_gate import KINDS, LocalizationGate
from tools.rtabmap_reason_audit import ledger
from test_localization_gate import observation


class Recording:
    def __init__(self):
        self.gate=LocalizationGate('map',{1});self.raw=[];self.log=[];self.wall=1.;self.ros=0;self.last_state=None
    def record(self,event,stamp,**fields):
        self.log.append(dict(event=event,stamp_ns=stamp,monotonic_ns=int(self.wall*1e9),ros_ns=self.ros,
                             reason=self.gate.reason,state=self.gate.state,epoch=self.gate.epoch,**fields))
    def state(self):
        identity=(self.gate.state,self.gate.reason,self.gate.epoch)
        # Match GateNode.heartbeat: inherited global reasons are not transitions.
        if identity!=self.last_state:
            self.record('state_transition',self.gate.last_complete);self.last_state=identity
    def acquisition(self,index,cycle=0,clock_behind=False):
        stamp=10**9+len([r for r in self.raw if r['event']=='publish_begin'])*10**8
        self.wall+=.1;self.ros=stamp-(1 if clock_behind else 0)
        self.raw.append(dict(event='publish_begin',index=index,cycle=cycle,stamp_ns=stamp,monotonic_ns=int(self.wall*1e9)))
        self.gate.acquisition(stamp,self.wall,self.ros);self.record('acquisition',stamp);self.state();return stamp
    def receive(self,kind,stamp,nomatch=False,lost=False):
        row=observation(kind,stamp)
        if kind=='info' and nomatch:row['loop_closure_id']=0
        if kind=='odom_info' and lost:row['lost']=True
        self.raw.append(dict(event=kind,stamp_ns=stamp,**row))
        outputs=self.gate.ingest(kind,row,self.wall,self.ros)
        self.record('received_'+kind,stamp,approved=outputs);self.state()
    def frame(self,index,cycle=0,nomatch=False,lost=False):
        s=self.acquisition(index,cycle)
        for k in KINDS:self.receive(k,s,nomatch,lost)
    def result(self):return ledger(self.raw,self.log,{1})


class ReasonTests(unittest.TestCase):
    def test_confirmation_and_recovery_with_repeated_source_indices(self):
        r=Recording()
        for i in range(3):r.frame(i)
        r.frame(3,nomatch=True)
        for i in range(3):r.frame(i,cycle=1)
        result=r.result()
        self.assertEqual(result['summary']['primary_reasons'],{'reconfirmation':4,'approved':2,'no_prior_match':1})
        self.assertEqual(result['rows'][4]['confirmation_trigger']['reason'],'no_prior_match')
        self.assertEqual(result['cycles']['1']['published'],3)
    def test_timeout_stays_cause_after_late_message(self):
        r=Recording();s=r.acquisition(0)
        for k in KINDS:
            if k!='info':r.receive(k,s)
        r.wall+=.6;r.gate.tick(r.wall,r.ros);r.state();r.receive('info',s)
        row=r.result()['rows'][0]
        self.assertEqual(row['primary_reason'],'message_timeout')
        self.assertEqual(row['first_block']['missing_kinds'],['info'])
        self.assertTrue(row['raw_numeric_healthy'])
        # The global retired-stamp reason is inherited by the next healthy
        # partial callbacks until their acquisition completes.
        for i in range(1,4):r.frame(i)
        self.assertEqual(r.result()['summary']['primary_reasons'],{'message_timeout':1,'reconfirmation':2,'approved':1})
    def test_clock_behind_is_not_missing_message_timeout(self):
        r=Recording();s=r.acquisition(0,clock_behind=True)
        for k in KINDS:r.receive(k,s)
        self.assertEqual(r.result()['rows'][0]['primary_reason'],'acquisition_clock_behind')
    def test_snapshot_expires_only_the_old_pending_acquisition(self):
        r=Recording();old=r.acquisition(0);new=r.acquisition(1)
        before={str(s):{} for s in r.gate.pending}
        r.wall=1.65;r.gate.tick(r.wall,r.ros);r.state()
        r.log[-1]['diagnostic']={'before':{'pending':before},'after':{'pending':{str(s):{} for s in r.gate.pending}}}
        for k in KINDS:r.receive(k,new)
        rows=r.result()['rows']
        self.assertEqual(rows[0]['primary_reason'],'message_timeout')
        self.assertEqual(rows[1]['primary_reason'],'reconfirmation')
    def test_boundary_retires_in_flight_last_frame(self):
        r=Recording();s=r.acquisition(0)
        for k in KINDS:
            if k!='info':r.receive(k,s)
        r.gate.discontinuity(1);r.state();r.receive('info',s)
        self.assertEqual(r.result()['rows'][0]['primary_reason'],'replay_boundary')
    def test_lost_and_unobserved_stream_remain_visible(self):
        r=Recording();r.frame(0,lost=True);r.acquisition(1)
        out=r.result()
        self.assertEqual(out['rows'][0]['primary_reason'],'odometry_lost')
        self.assertEqual(out['rows'][1]['primary_reason'],'unresolved_incomplete_frame')
        self.assertIn('info:missing',out['rows'][1]['raw_health_flags'])
    def test_duplicate_stamp_and_corrupt_approval_are_detected(self):
        r=Recording()
        for i in range(3):r.frame(i)
        corrupt=copy.deepcopy(r.log)
        next(x for x in corrupt if x.get('approved'))['approved'][0]['pose']['pose']['position']['x']=99
        with self.assertRaises(ValueError):ledger(r.raw,corrupt,{1})
        r.raw.append(r.raw[0].copy())
        with self.assertRaises(ValueError):r.result()


if __name__=='__main__':unittest.main()
