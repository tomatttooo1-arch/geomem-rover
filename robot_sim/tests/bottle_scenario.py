#!/usr/bin/env python3
"""Repeatable user-level bottle scenario on AGX; no velocity, permit or truth input.

Starts with a fresh runtime/world. Waits for actual bottle memory, requests map
patrol, cancels after physical departure and out-of-view evidence, then queries.
The runtime chooses the return pose and Nav2 controls all motion.

Line D1 (2026-10-14, server simulation; defaults keep the AGX behaviour):
  --expect-device cpu        the found frame must come from CPU inference (AGX: cuda + fp32 math)
  --query-via hub            the query goes through the hub like a PC user (pc/fpgapc.py find), not
                             straight onto /fpga/query; the fpgapc exit code and its ACK/RESULT are logged
  --estop-during-return-s N  N wall seconds after the return navigation starts, a PC e-stop
                             (pc/fpgapc.py estop); then the run passes only if the car stops (5 /odom
                             samples with |v|+|w| < 0.01) within --estop-stop-limit-s of SIMULATION time
                             after the command and stays stopped for --estop-hold-s wall seconds.
                             The task outcome after the e-stop is recorded, not judged.

Line D2 (2026-10-14; defaults keep the D1 behaviour):
  readiness                  the GetState request to velocity_smoother has a timeout and is re-sent
                             (scenario_readiness.ServiceStatePoller; D1 run_b2 sat 420 s in "remember"
                             with no record of which condition failed); every change of each readiness
                             condition (action server ready, velocity_smoother active, /odom received,
                             runtime input_ready / navigation_available, 8 s warm-up) is a 'readiness'
                             row with wall and simulation time, plus a 'readiness_waiting' row every 30 s
  --expect-detector NAME     the found frame must come from that detector (inference_backend.detector)
  --estop-release-after-s H  after the e-stop stop and --estop-hold-s, a PC release (fpgapc release --epoch
                             E, E from the e-stop RESULT) H wall seconds later; then the car must move
                             again (|v|+|w| >= 0.01) within --resume-limit-s wall seconds. Ordering of
                             new frame / heartbeat / unlock / command is judged offline from the records.
  --cut-during-return-s N    N wall seconds after the return navigation starts (car moving >= 0.05),
  --trigger-min-v V          (2026-10-14 D2 close-out, PREREG §10) the e-stop / cut fires only when the odometry
  --trigger-max-w W          reads |v| >= V and |w| <= W (straight driving), still after the return start delay;
                             without either flag the original trigger (|v|+|w| >= 0.05) is unchanged
  --cut-pid-file PATH        SIGTERM to the geometry producer (pid read from PATH): the frame stream stops,
                             the permit times out. Passes like the e-stop (still within
                             --cut-stop-limit-s simulation seconds, stays still --cut-hold-s wall seconds);
                             otherwise ends 'cut_not_stopped' after limit + hold (the positive control
                             expects that). The chain times are judged offline (tools/verify_bottle_run.py).
"""
import argparse
import json
import math
import os
import platform
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scenario_readiness import ReadinessLog, ServiceStatePoller  # noqa: E402
sys.path.append(str(Path(__file__).resolve().parents[2]))
from host.backend_log import BackendExpander  # noqa: E402

READINESS = ('action_server_ready', 'velocity_smoother_active', 'odom_received', 'runtime_input_ready',
             'runtime_navigation_available', 'warmup_8s')



def trigger_ok(state, args):
    """E-stop / cut trigger on the latest odometry. Default (no --trigger-* flag): |v|+|w| >= 0.05, as frozen
    for PREREG §3/§4. With --trigger-min-v / --trigger-max-w (PREREG §10, straight driving): |v| >= min_v
    and |w| <= max_w, and still |v|+|w| >= 0.05."""
    if (state['speed'] or 0) < .05:
        return False
    if args.trigger_min_v is None and args.trigger_max_w is None:
        return True
    if state['v'] is None or state['w'] is None:
        return False
    return ((args.trigger_min_v is None or abs(state['v']) >= args.trigger_min_v)
            and (args.trigger_max_w is None or abs(state['w']) <= args.trigger_max_w))

def cuda_fp32(execution):
    """The service's execution record says fp32 CUDA math: SSDLite reports cuda_math 'fp32'; the YOLO torch
    backend (line D2) reports precision 'fp32' with TF32 off (it has no cuda_math key)."""
    if 'cuda_math' in execution:
        return execution['cuda_math'] == 'fp32'
    return (execution.get('backend') == 'torch' and execution.get('precision') == 'fp32'
            and execution.get('tf32') is False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--expect-device', choices=('cuda', 'cpu'), default='cuda')
    parser.add_argument('--query-via', choices=('ros', 'hub'), default='ros')
    parser.add_argument('--hub-host', default='127.0.0.1')
    parser.add_argument('--hub-port', type=int)
    parser.add_argument('--token-file')
    parser.add_argument('--estop-during-return-s', type=float)
    parser.add_argument('--estop-stop-limit-s', type=float, default=1.5)
    parser.add_argument('--estop-hold-s', type=float, default=5.0)
    parser.add_argument('--estop-release-after-s', type=float)
    parser.add_argument('--resume-limit-s', type=float, default=10.0)
    parser.add_argument('--expect-detector')
    parser.add_argument('--cut-during-return-s', type=float)
    parser.add_argument('--cut-pid-file', type=Path)
    parser.add_argument('--cut-stop-limit-s', type=float, default=1.5)
    parser.add_argument('--cut-hold-s', type=float, default=5.0)
    parser.add_argument('--lifecycle-timeout-s', type=float, default=2.0)
    parser.add_argument('--trigger-min-v', type=float)
    parser.add_argument('--trigger-max-w', type=float)
    args = parser.parse_args()
    if (args.query_via == 'hub' or args.estop_during_return_s is not None) and not (
            args.hub_port and args.token_file):
        parser.error('--query-via hub and --estop-during-return-s need --hub-port and --token-file')
    if (args.cut_during_return_s is None) != (args.cut_pid_file is None):
        parser.error('--cut-during-return-s and --cut-pid-file go together')
    if args.cut_during_return_s is not None and args.estop_during_return_s is not None:
        parser.error('one of --cut-during-return-s / --estop-during-return-s')
    if args.estop_release_after_s is not None and args.estop_during_return_s is None:
        parser.error('--estop-release-after-s needs --estop-during-return-s')
    fpgapc = [sys.executable, str(Path(__file__).resolve().parents[2]/'pc/fpgapc.py'), '--host', args.hub_host,
             '--port', str(args.hub_port), '--token-file', str(args.token_file)]
    rclpy.init()
    node = rclpy.create_node('fpga_bottle_scenario')
    stream = args.out.open('x')
    started = time.monotonic()
    backend_expander = BackendExpander()
    state = {'stage': 'remember', 'target': None, 'frame': None, 'odom': None,
             'origin': None, 'outside_frames': 0, 'arrived': None, 'stopped_samples': 0,
             'retired': False, 'result': None, 'clock': None, 'speed': None, 'odom_stamp': None, 'v': None, 'w': None,
             'query_proc': None, 'return_started': None, 'estop': None, 'cut': None, 'odom_hist': []}
    navigation = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    lifecycle = node.create_client(GetState, '/velocity_smoother/get_state')
    poller = ServiceStatePoller(period_s=.5, timeout_s=args.lifecycle_timeout_s)
    readiness = ReadinessLog(READINESS)
    navigation_active = False
    last_waiting_log = 0.
    patrol = node.create_publisher(Empty, '/fpga/patrol', 10)
    cancel = node.create_publisher(Empty, '/fpga/cancel', 10)
    query = node.create_publisher(String, '/fpga/query', 10)

    def log(event, **fields):
        row = {'event': event, 'elapsed_wall': time.monotonic()-started, **fields}
        stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
        stream.flush()
        print(json.dumps(row, ensure_ascii=False), flush=True)

    def odom(msg):
        p = msg.pose.pose.position
        state['odom'] = (p.x, p.y)
        if state['origin'] is None:
            state['origin'] = (p.x, p.y)
        if state['stage'] in ('remember', 'depart'):   # departure is judged on odometry stamped <= the frame time
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            state['odom_hist'].append((stamp, p.x, p.y))
        speed = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.angular.z)
        state['v'], state['w'] = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        state['stopped_samples'] = state['stopped_samples']+1 if speed < .01 else 0
        state['speed'] = speed
        state['odom_stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        e = state['estop'] or state['cut']
        if e is not None and e.get('stopped_sim') is None:
            if state['stopped_samples'] == 1:
                e['first_still'] = (state['odom_stamp'], time.time_ns())
            if state['stopped_samples'] >= 5:
                e['stopped_sim'], e['stopped_wall_ns'] = e['first_still']
        if e is not None and e.get('stopped_sim') is not None and speed >= .01:
            if e.get('release_wall_ns') is None:
                e['moved_after_stop'] = e.get('moved_after_stop', 0) + 1
            elif e.get('resumed_sim') is None:
                e['resumed_sim'], e['resumed_wall_ns'] = state['odom_stamp'], time.time_ns()

    def clock(msg):
        state['clock'] = msg.clock.sec + msg.clock.nanosec/1e9

    def status(msg):
        row = backend_expander.row(json.loads(msg.data))   # 10-29: short inference_backend rows restored
        event = row['event']
        if event != 'frame_processed' and event != 'task':
            log('runtime_event', row=row)
        if event == 'navigation_started' and state['stage'] == 'query' and state['return_started'] is None:
            state['return_started'] = time.monotonic()
            log('return_navigation_started', row=row)
        if state['estop'] is not None or state['cut'] is not None:
            if event == 'task':
                log('task_after_estop' if state['estop'] is not None else 'task_after_cut', row=row)
                if (state['estop'] is not None and state['estop'].get('resumed_sim') is not None
                        and row['state'] in ('failed', 'not_found', 'unable_to_confirm', 'cancelled',
                                             'ambiguous', 'found', 'needs_manual_approach')):
                    state['estop']['task_after_resume'] = row['state']
            return
        if event in ('fault', 'patrol_rejected', 'query_rejected'):
            state['result'] = {'passed': False, 'reason': row.get('reason', event)}
        if event == 'navigation_retired' and state['stage'] == 'cancel':
            state['retired'] = True
        if event == 'navigation_result' and row.get('command_id') and row.get('succeeded'):
            state['arrived'] = row['time']
            log('return_arrived', row=row)
        if event == 'frame_processed':
            state['frame'] = row
            tracks = row['tracks']
            target = next((t for t in tracks if t['id'] == state['target']), None)
            for change in readiness.update(time.monotonic()-started, state['clock'],
                                           runtime_input_ready=row.get('input_ready'),
                                           runtime_navigation_available=row.get('navigation_available')):
                log('readiness', **change)
            if (state['stage'] == 'remember' and navigation_active and navigation.server_is_ready()
                    and row.get('input_ready') and row.get('navigation_available')
                    and time.monotonic()-started >= 8 and state['odom']):
                bottles = [t for t in tracks if t['label'] == 'bottle' and t['state'] == 'current_confirmed']
                if len(bottles) == 1 and row.get('evidence'):
                    state['target'] = bottles[0]['id']
                    state['stage'] = 'depart'
                    log('remembered', frame=row, odom=state['odom'])
                    patrol.publish(Empty())
                    log('patrol_requested')
            elif state['stage'] == 'depart' and target and state['odom']:
                # D2 smoke_02: the latest odometry could be a few cm ahead of the frame's processing time, so the
                # scenario declared 0.656 m where the verifier (odometry stamped <= that frame's time) saw < 0.65 m.
                # Same quantity as the verifier now: displacement reached by the frame's processing time.
                distance = max((math.dist(state['origin'], (x, y)) for t, x, y in state['odom_hist']
                                if t <= row['time']), default=0.)
                outside = target['reason'] == 'out_of_view' and not any(d['label']=='bottle' for d in row['detections'])
                state['outside_frames'] = state['outside_frames']+1 if outside else 0
                if state['outside_frames'] >= 3 and distance >= .65:
                    state['stage'] = 'cancel'
                    state['stopped_samples'] = 0
                    log('departed_out_of_view', frame=row, odom=state['odom'], displacement_m=distance)
                    cancel.publish(Empty())
                    log('patrol_cancel_requested')
        if (event == 'task' and state['stage'] == 'query'
                and (args.estop_during_return_s is not None or args.cut_during_return_s is not None)
                and row['state'] in ('failed', 'not_found', 'unable_to_confirm', 'cancelled', 'ambiguous', 'found',
                                 'needs_manual_approach')):
            state['result'] = {'passed': False, 'reason': 'task_ended_before_estop:'+row['state']}
            log('task_ended_before_estop', task=row)
        elif event == 'task' and state['stage'] == 'query':
            if row['state'] in ('failed', 'not_found', 'unable_to_confirm', 'cancelled', 'ambiguous',
                                'needs_manual_approach'):
                state['result'] = {'passed': False, 'reason': row['state']+':'+row['reason']}
            elif row['state'] == 'found':
                frame = state['frame']
                target = next((t for t in frame['tracks'] if t['id']==state['target']), None)
                valid = bool(state['arrived'] is not None and frame['acquisition_timestamp'] > state['arrived']
                             and row['target_id']==state['target'] and target
                             and target['last_seen_frame']==frame['frame_id']
                             and target['state']=='current_confirmed' and frame.get('evidence')
                             and frame['inference_device']==args.expect_device and frame['sensor_source']=='simulation'
                             and frame['inference_backend']['execution']['device']==args.expect_device
                             and (args.expect_detector is None
                                  or frame['inference_backend'].get('detector')==args.expect_detector)
                             and (args.expect_device != 'cuda'
                                  or cuda_fp32(frame['inference_backend']['execution'])))
                state['result'] = {'passed': valid, 'reason': 'fresh_return_observation' if valid else 'invalid_found_evidence'}
                state['stage'] = 'done'
                log('found_evidence', task=row, frame=frame, arrived=state['arrived'], odom=state['odom'])

    node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data)
    node.create_subscription(String, '/fpga/status', status, 50)
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)

    def pc_command(words):
        """Start pc/fpgapc.py (a PC user's command line); returns (Popen, start wall ns)."""
        t_ns = time.time_ns()
        proc = subprocess.Popen(fpgapc + words, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, env=dict(os.environ, PYTHONIOENCODING='utf-8'))
        return proc, t_ns

    def pc_finish(proc, t_ns, name):
        out = proc.stdout.read()
        lines = [line for line in out.splitlines() if line.startswith(('ACK ', 'RESULT '))]
        parsed = {}
        for line in lines:
            key, _, body = line.partition(' ')
            try:
                parsed[key.lower()] = json.loads(body)
            except ValueError:
                parsed[key.lower()] = body
        log(name+'_pc_command', returncode=proc.returncode, sent_wall_ns=t_ns,
            done_wall_ns=time.time_ns(), output=out[-2000:], **parsed)
        return proc.returncode, parsed

    log('scenario_start', platform=platform.platform(), machine=platform.machine(),
        query='找瓶子', departure_min_m=.65, outside_frames_required=3,
        expect_device=args.expect_device, query_via=args.query_via,
        estop_during_return_s=args.estop_during_return_s, estop_stop_limit_sim_s=args.estop_stop_limit_s,
        estop_hold_wall_s=args.estop_hold_s, estop_release_after_wall_s=args.estop_release_after_s,
        resume_limit_wall_s=args.resume_limit_s, expect_detector=args.expect_detector,
        cut_during_return_s=args.cut_during_return_s, cut_stop_limit_sim_s=args.cut_stop_limit_s,
        cut_hold_wall_s=args.cut_hold_s, lifecycle_timeout_s=args.lifecycle_timeout_s,
        trigger_min_v=args.trigger_min_v, trigger_max_w=args.trigger_max_w,
        readiness_items=list(READINESS))
    try:
        while state['result'] is None and time.monotonic()-started < args.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()-started
            outcome = poller.check(now)
            if outcome is not None and outcome[0] == 'answer':
                response = outcome[1].result()
                navigation_active = response is not None and response.current_state.id == 3
            elif outcome is not None:          # no answer within the timeout: drop it, ask again
                drop = getattr(lifecycle, 'remove_pending_request', None)
                if drop is not None:
                    drop(outcome[1])
                log('lifecycle_query_timeout', timeouts=poller.counts['timeouts'], sent=poller.counts['sent'])
            if poller.due(now) and lifecycle.service_is_ready():
                poller.sent(now, lifecycle.call_async(GetState.Request()))
            for change in readiness.update(now, state['clock'], action_server_ready=navigation.server_is_ready(),
                                           velocity_smoother_active=navigation_active,
                                           odom_received=state['odom'] is not None, warmup_8s=now >= 8):
                log('readiness', **change)
            if state['stage'] == 'remember' and now - last_waiting_log >= 30:
                last_waiting_log = now
                log('readiness_waiting', readiness=readiness.snapshot(now), lifecycle=dict(poller.counts),
                    clock=state['clock'])
            if state['stage']=='cancel' and state['retired'] and state['stopped_samples']>=5:
                state['stage']='query'
                log('query_requested', query='找瓶子', target_id=state['target'], odom=state['odom'],
                    via=args.query_via)
                if args.query_via == 'hub':
                    state['query_proc'] = pc_command(['find', '找瓶子'])
                else:
                    query.publish(String(data='找瓶子'))
            if state['query_proc'] is not None and state['query_proc'][0].poll() is not None:
                rc, parsed = pc_finish(*state['query_proc'], 'query')
                state['query_proc'] = None
                if rc != 0 or (parsed.get('ack') or {}).get('status') != 'forwarded':
                    state['result'] = {'passed': False, 'reason': 'hub_query_not_forwarded'}
            if (args.estop_during_return_s is not None and state['estop'] is None
                    and state['return_started'] is not None
                    and time.monotonic()-state['return_started'] >= args.estop_during_return_s
                    and trigger_ok(state, args)):   # only a moving car tests the stop
                state['estop'] = {'clock_at_command': state['clock'], 'speed_at_command': state['speed'],
                                  'odom_at_command': state['odom']}
                proc, t_ns = pc_command(['estop'])
                state['estop'].update(command_wall_ns=t_ns, proc=proc)
                log('estop_requested', clock=state['clock'], speed=state['speed'], odom=state['odom'],
                    v=state['v'], w=state['w'],
                    wall_ns=t_ns)
            e = state['estop']
            if e is not None and e.get('proc') is not None and e['proc'].poll() is not None:
                rc, parsed = pc_finish(e['proc'], e['command_wall_ns'], 'estop')
                e.update(proc=None, returncode=rc, ack=parsed.get('ack'), result=parsed.get('result'))
            if e is not None and e.get('release_proc') is not None and e['release_proc'].poll() is not None:
                rc, parsed = pc_finish(e['release_proc'], e['release_wall_ns'], 'release')
                e.update(release_proc=None, release_returncode=rc, release_ack=parsed.get('ack'),
                         release_result=parsed.get('result'))
            if (e is not None and args.estop_release_after_s is not None and e.get('hold_done')
                    and e.get('release_wall_ns') is None and time.monotonic() >= e['release_at']):
                epoch = (e.get('result') or {}).get('epoch')
                proc, t_ns = pc_command(['release', '--epoch', str(epoch)])
                e.update(release_proc=proc, release_wall_ns=t_ns, release_clock=state['clock'],
                         resume_deadline=time.monotonic()+args.resume_limit_s)
                log('release_requested', epoch=epoch, clock=state['clock'], wall_ns=t_ns)
            if (e is not None and e.get('release_wall_ns') is not None and e.get('release_proc') is None
                    and (e.get('resumed_sim') is not None or time.monotonic() >= e['resume_deadline'])
                    and e.get('resume_judged') is None):
                e['resume_judged'] = time.monotonic()
                log('resume_observed', resumed_sim=e.get('resumed_sim'), resumed_wall_ns=e.get('resumed_wall_ns'),
                    release_clock=e.get('release_clock'), release_wall_ns=e['release_wall_ns'])
            if (e is not None and e.get('resume_judged') is not None and state['result'] is None
                    and (e.get('task_after_resume') is not None or time.monotonic() >= e['resume_judged']+60
                         or e.get('resumed_sim') is None)):
                ok = bool(e['stop_ok'] and e.get('release_returncode') == 0
                          and (e.get('release_ack') or {}).get('status') == 'forwarded'
                          and e.get('resumed_sim') is not None)
                state['result'] = dict(e['stop_result'], passed=ok,
                                       reason='estop_stopped_released_resumed' if ok else 'estop_release_or_resume_failed',
                                       release_returncode=e.get('release_returncode'),
                                       release_ack=e.get('release_ack'), release_result=e.get('release_result'),
                                       resumed_after_release_sim_s=(None if e.get('resumed_sim') is None
                                                                    or e.get('release_clock') is None
                                                                    else e['resumed_sim']-e['release_clock']),
                                       task_after_resume=e.get('task_after_resume'))
                state['stage'] = 'done'
                log('estop_evidence', **{k: v for k, v in e.items() if not k.endswith('proc')})
            if e is not None and e.get('stopped_sim') is not None and e.get('proc') is None and not e.get('hold_done'):
                if e.get('hold_until') is None:
                    e['hold_until'] = time.monotonic()+args.estop_hold_s
                elif time.monotonic() >= e['hold_until']:
                    stop_sim = e['stopped_sim']-e['clock_at_command'] if e['clock_at_command'] is not None else None
                    ack = e.get('ack') or {}
                    ok = bool(e.get('returncode') == 0 and ack.get('status') == 'latched_on_nano'
                              and (e['speed_at_command'] or 0) >= .01
                              and stop_sim is not None and 0 <= stop_sim <= args.estop_stop_limit_s
                              and not e.get('moved_after_stop'))
                    stop_result = {'passed': ok, 'reason': 'estop_stopped' if ok else 'estop_not_stopped',
                                   'estop_to_still_sim_s': stop_sim,
                                   'estop_to_still_wall_s': (e['stopped_wall_ns']-e['command_wall_ns'])/1e9,
                                   'speed_at_command': e['speed_at_command'],
                                   'moved_after_stop_samples': e.get('moved_after_stop', 0)}
                    e['hold_done'] = True
                    if args.estop_release_after_s is None or not ok:
                        state['result'] = stop_result
                        state['stage'] = 'done'
                        log('estop_evidence', **{k: v for k, v in e.items() if not k.endswith('proc')})
                    else:
                        e.update(stop_ok=ok, stop_result=stop_result,
                                 release_at=time.monotonic()+args.estop_release_after_s)
                        log('estop_stop_judged', **stop_result)
            c = state['cut']
            if (args.cut_during_return_s is not None and c is None and state['return_started'] is not None
                    and time.monotonic()-state['return_started'] >= args.cut_during_return_s
                    and trigger_ok(state, args)):
                pid = int(args.cut_pid_file.read_text().split()[0])
                t_ns = time.time_ns()
                os.kill(pid, signal.SIGTERM)
                state['cut'] = c = {'pid': pid, 'clock_at_command': state['clock'], 'speed_at_command': state['speed'],
                                    'odom_at_command': state['odom'], 'command_wall_ns': t_ns,
                                    'give_up': time.monotonic()+args.cut_stop_limit_s+args.cut_hold_s+5}
                log('cut_requested', pid=pid, clock=state['clock'], speed=state['speed'], odom=state['odom'],
                    v=state['v'], w=state['w'],
                    wall_ns=t_ns)
            if c is not None and state['result'] is None:
                if c.get('stopped_sim') is not None and c.get('hold_until') is None:
                    c['hold_until'] = time.monotonic()+args.cut_hold_s
                done = c.get('hold_until') is not None and time.monotonic() >= c['hold_until']
                if done or time.monotonic() >= c['give_up']:
                    stop_sim = (c['stopped_sim']-c['clock_at_command']
                                if c.get('stopped_sim') is not None and c['clock_at_command'] is not None else None)
                    ok = bool(done and (c['speed_at_command'] or 0) >= .01 and stop_sim is not None
                              and 0 <= stop_sim <= args.cut_stop_limit_s and not c.get('moved_after_stop'))
                    state['result'] = {'passed': ok, 'reason': 'cut_stopped' if ok else 'cut_not_stopped',
                                       'cut_to_still_sim_s': stop_sim,
                                       'cut_to_still_wall_s': (None if c.get('stopped_wall_ns') is None
                                                               else (c['stopped_wall_ns']-c['command_wall_ns'])/1e9),
                                       'speed_at_command': c['speed_at_command'],
                                       'moved_after_stop_samples': c.get('moved_after_stop', 0)}
                    state['stage'] = 'done'
                    log('cut_evidence', **c)
        if state['result'] is None:
            now = time.monotonic()-started
            state['result']={'passed': False, 'reason': 'scenario_wall_timeout', 'stage': state['stage'],
                             'readiness': readiness.snapshot(now), 'lifecycle': dict(poller.counts)}
        # Permit withdrawal is the runtime's responsibility; collect physical settling too.
        settle = time.monotonic()+2
        while time.monotonic()<settle:
            rclpy.spin_once(node, timeout_sec=.05)
        state['result']['stopped'] = state['stopped_samples']>=5
        if not state['result']['stopped']:
            state['result']['passed']=False
        log('scenario_result', **state['result'])
    finally:
        cancel.publish(Empty())
        rclpy.spin_once(node, timeout_sec=.1)
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not state['result']['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
