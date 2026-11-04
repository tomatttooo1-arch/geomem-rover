"""Bounded actual-Runtime navigation controls; external I/O fixtures, no ROS/devices."""
import argparse
from concurrent.futures import Future
import hashlib
import json
from pathlib import Path
import subprocess
import traceback
from types import SimpleNamespace as NS
from host.task_control import PoseGoal
from tools.rtabmap_runtime_behavior_probe import Session
from tools.rtabmap_runtime_probe_support import healthy

CASES = ('healthy', 'epoch', 'instance', 'local_invalidate', 'gap_update_first',
         'gap_poll_first', 'gap_result_first', 'late_accept', 'nested_healthy',
         'nested_retired', 'old_callbacks', 'cancel_receipt', 'accept_exception',
         'result_exception', 'nonterminal_result', 'cancel_exception', 'rejected',
         'task_healthy', 'task_retired', 'heartbeat_multiframe', 'task_gap_result_first', 'send_exception')

def run_case(root, name):
    s = Session(root, name, 'original'); w, n = s.w, s.n
    checks = []
    def check(label, condition):
        checks.append(dict(check=label, passed=bool(condition)))
        w.emit('criterion', check=label, passed=bool(condition))
    def request(): return w.action_requests[-1]
    def accept(r=None):
        r = r or request(); w.emit('accept_delivery', request_id=r['handle'].request_id)
        r['future'].set_result(r['handle'])
    def result(status=4, r=None):
        r = r or request(); w.emit('result_delivery', request_id=r['handle'].request_id, status=status)
        r['handle'].result_future.set_result(NS(status=status))
    def fresh(wall=.4, stamp=1400000000, change=None):
        w.clock(wall, stamp); row = healthy(w, s.gate, stamp)
        if change == 'epoch': row['epoch'] += 1
        if change == 'instance': row['instance'] = 'fixture-B'
        s.status(row); w.image(stamp); s.service('fresh_frame')
    def blocked(context):
        return n.nav is context and context['cancelled'] and n.motion.inhibited and w.publisher_values['/fpga/motion_permit'] is False
    error = None
    try:
        s.warm(); task_case = name.startswith('task_')
        if name == 'send_exception':
            n.actions.send_exception = RuntimeError('synchronous send transport')
            try: n.navigate(PoseGoal(.5, .2, .1), None)
            except Exception: pass
            check('send_exception_keeps_occupancy', n.nav is not None and blocked(n.nav))
            return dict(name=name, checks=checks, error=None, passed=all(r['passed'] for r in checks))
        if task_case:
            w.waypoints.append(PoseGoal(.5, .2, .1)); n.query('find cup')
            assert n.task is not None and n.task.state == 'navigating'
        else: n.navigate(PoseGoal(.5, .2, .1), None)
        old = n.nav; first = request(); assert old is not None
        late = name in ('late_accept', 'nested_healthy', 'nested_retired', 'accept_exception', 'rejected')
        if not late:
            accept(); w.poll('accepted_positive'); assert w.publisher_values['/fpga/motion_permit'] is True
        check('real_initialization_and_processed_frame', n.processed == 1 and 'rgb-1300000000' in n.memory.seen_frames)
        if name in ('healthy', 'epoch', 'instance', 'local_invalidate', 'task_healthy', 'task_retired'):
            if name == 'local_invalidate': n.localization.invalidate()
            fresh(change='epoch' if name in ('epoch', 'task_retired') else 'instance' if name == 'instance' else None)
            check('new_frame_processed', n.processed == 2)
            if name in ('healthy', 'task_healthy'):
                check('healthy_navigation_survives', n.nav is old and not old['cancelled'] and not n.motion.inhibited)
                result(); check('normal_terminal_releases', n.nav is None)
                if task_case: check('real_task_enters_observing', n.task.state == 'observing')
            else:
                check('continuity_interrupt_retires', blocked(old)); result()
                check('retired_terminal_releases', n.nav is None)
                if task_case: check('real_task_not_advanced', n.task.state in n.task.TERMINAL and n.task.state != 'observing')
        elif name in ('gap_update_first', 'gap_poll_first'):
            w.clock(.85, 1400000000)
            if name == 'gap_poll_first': w.poll('before_recovery')
            s.status(healthy(w, s.gate, 1400000000)); w.image(1400000000); s.service('after_recovery')
            check('long_gap_cannot_revive', blocked(old))
        elif name == 'gap_result_first':
            n.patrol = [PoseGoal(.8, .2, .1)]; w.clock(.85, 1400000000); result()
            check('result_checks_unobserved_gap', len(w.action_requests) == 1 and not n.patrol and n.nav is None)
        elif name == 'task_gap_result_first':
            w.clock(.85, 1400000000); result()
            check('old_task_not_observing_after_unchecked_gap', n.task.state in n.task.TERMINAL and n.task.state != 'observing')
        elif name == 'heartbeat_multiframe':
            for i in range(4, 45):
                wall = i/10; stamp = 10**9+i*10**8
                fresh(wall, stamp)
                w.clock(wall+.02, stamp)
                s.status(s.gate.status(w.wall, w.ros_ns)); w.poll('same_stamp_heartbeat')
            check('heartbeats_and_evicted_initial_frame_do_not_cancel',
                  n.nav is old and not old['cancelled'] and n.processed == 42
                  and w.publisher_values['/fpga/motion_permit'] is True
                  and all(r['stamp_ns'] != 1300000000 for r in n.localization.row['approved']))
        elif name == 'late_accept':
            fresh(change='epoch'); accept(); w.poll('late_accept')
            check('late_accept_not_reauthorized', blocked(old) and bool(first['handle'].cancels))
        elif name in ('nested_healthy', 'nested_retired'):
            n.patrol = [PoseGoal(.8, .2, .1)]
            if name == 'nested_retired':
                n.localization.invalidate(); w.clock(.4, 1400000000)
                n.localization.update(healthy(w, s.gate, 1400000000), w.wall, w.ros_ns)
            result(r=first); accept(first)
            if name == 'nested_healthy':
                check('nested_completion_starts_next', len(w.action_requests) == 2 and n.nav is not old and not n.motion.inhibited)
                accept(); w.poll('new_goal_positive'); check('new_goal_permitted', w.publisher_values['/fpga/motion_permit'] is True)
            else:
                check('nested_retired_result_no_continuation', len(w.action_requests) == 1 and n.nav is None and not n.patrol and n.motion.inhibited)
        elif name == 'old_callbacks':
            result(); n.navigate(PoseGoal(.8, .2, .1), None); new = n.nav
            accept(); w.poll('new_context'); bad = Future(); bad.set_exception(RuntimeError('old accept transport'))
            n.accepted(old, bad); n.accepted(old, first['future']); n.result(old, first['handle'].result_future); n.result(old, bad)
            check('old_callbacks_do_not_stop_new', n.nav is new and not new['cancelled'] and not n.motion.inhibited)
        elif name == 'cancel_receipt':
            n.stop()
            for f in first['handle'].cancels: f.set_result(NS(goals_canceling=['fixture']))
            n.begin_patrol(NS()); check('cancel_receipt_keeps_occupancy', blocked(old) and len(w.action_requests) == 1)
            result(5); fresh(); w.waypoints.append(PoseGoal(.8, .2, .1)); n.begin_patrol(NS())
            check('explicit_new_after_terminal', len(w.action_requests) == 2 and n.nav is not old)
        elif name == 'accept_exception':
            first['future'].set_exception(RuntimeError('accept transport')); check('accept_exception_keeps_occupancy', blocked(old))
        elif name == 'result_exception':
            first['handle'].result_future.set_exception(RuntimeError('result transport')); check('result_exception_keeps_occupancy', blocked(old))
        elif name == 'nonterminal_result':
            result(2); check('nonterminal_keeps_occupancy', blocked(old))
        elif name == 'cancel_exception':
            first['handle'].cancel_exception = RuntimeError('cancel transport')
            try: n.stop()
            except Exception: pass
            check('cancel_exception_still_latches', blocked(old)); fresh(); check('cancel_exception_recovery_no_resume', blocked(old))
            first['handle'].cancel_exception = None
        elif name == 'rejected':
            first['handle'].accepted = False; accept(); check('rejection_releases_occupancy', n.nav is None and n.motion.inhibited)
    except Exception:
        error = traceback.format_exc(); w.emit('probe_error', error=error)
    finally: s.finish(error=error)
    return dict(name=name, checks=checks, error=error, passed=not error and all(r['passed'] for r in checks))

def main():
    p = argparse.ArgumentParser(); p.add_argument('output', type=Path); a = p.parse_args(); a.output.mkdir(exist_ok=False)
    rows = [run_case(a.output, name) for name in CASES]
    report = dict(source_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
                  runtime_sha256=hashlib.sha256(Path('host/ros_runtime.py').read_bytes()).hexdigest(), cases=rows,
                  limitation='actual Runtime + actual TaskController; external I/O fixtures; not ROS or physical stop')
    (a.output/'criteria.json').write_text(json.dumps(report,indent=2)); print(json.dumps({r['name']:r['passed'] for r in rows},indent=2))
    raise SystemExit(0 if all(r['passed'] for r in rows) else 1)
if __name__ == '__main__': main()
