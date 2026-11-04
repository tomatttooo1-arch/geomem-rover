#!/usr/bin/env python3
"""Bounded Nav2 component suite; explicit map goals, no object truth or detector.

Motion permit is revoked on unhealthy inputs, action completion or cancellation.
Transport and server velocity guard retain their existing expiry checks.
"""
import argparse
import json
import math
from pathlib import Path
import time

from host.nav_localization import (GoalLifecycle, MAP_CORRECTION_EXCEEDED,
                                   NavigationLocalization)

# Sequential, no world teleport between cases. Each run starts at (0,0,0).
CASES = [('straight', (.6, 0., 0.)), ('turn', (.6, .6, math.pi/2)),
         ('uturn', (.0, .6, math.pi)), ('near', (-.06, .6, math.pi)),
         ('cancel', (-1.2, .6, math.pi)), ('unreachable', (4., .6, 0.))]


def fresh(clock, stamps, receipts, now):
    return bool(stamps and set(stamps) == set(receipts) and all(
        math.isfinite(s) and 0 <= clock-s <= 1 and 0 <= now-receipts[t] <= .5
        for t, s in stamps.items()))


def promote(clock, pending, stamps, receipts, stamp_nanos=None):
    for topic, samples in pending.items():
        eligible=[s for s in samples if s[0]<=clock]
        if eligible:
            sample=max(eligible)
            stamp, received=sample[:2]
            if stamp>=stamps.get(topic,-1):
                stamps[topic],receipts[topic]=stamp,received
                if stamp_nanos is not None:
                    stamp_nanos[topic] = sample[2] if len(sample) > 2 else round(stamp*1e9)
        pending[topic]=[s for s in samples if s[0]>clock][-32:]


def clock_ns(state):
    """The /clock reading in integer nanoseconds, rounded, never truncated.

    int(8.466 * 1e9) is 8465999999: truncation puts the clock 1 ns before an /odom stamp
    taken at the same simulation step (about 2 % of millisecond clock values), which read
    as an input from the future and aborted the first case of the 2026-09-27 server smoke.
    """
    return round(state['clock'] * 1e9)


def within_goal(pose, goal):
    return (math.hypot(pose[0]-goal[0], pose[1]-goal[1]) <= .03 and
            abs(math.atan2(math.sin(pose[2]-goal[2]), math.cos(pose[2]-goal[2]))) <= .03)


def planar_pose(transform):
    """Read a map pose from an exact-time map->base_link transform."""
    p, q = transform.transform.translation, transform.transform.rotation
    values = (p.x, p.y, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nonfinite map pose')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if abs(norm-1) > .001:
        raise ValueError('invalid map pose quaternion')
    return (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y),
                                 1-2*(q.y*q.y+q.z*q.z)))


def send_goal(action, lifecycle, authorization, goal):
    """Begin the same lifecycle generation before handing a goal to ROS."""
    context = lifecycle.start(authorization)
    try:
        response = action.send_goal_async(goal)
        response.add_done_callback(lambda future: accept_goal_response(
            lifecycle, context, future))
        return context, response
    except Exception:
        # The client cannot prove that a transport error preceded server receipt.
        context.revoked = True
        context.transport_error = 'goal_send_transport_failed'
        raise


def accept_goal_response(lifecycle, context, future):
    """Own even a late goal response and cancel an already-revoked acceptance."""
    try:
        handle = future.result()
        lifecycle.accepted(context, handle)
        if handle.accepted:
            context.result_future = handle.get_result_async()
            context.result_future.add_done_callback(
                lambda result: accept_goal_result(lifecycle, context, result))
    except Exception as exc:
        context.response_processed = True
        context.revoked = True
        context.transport_error = 'goal_response_transport_failed:' + str(exc)
        lifecycle.revoke(context)


def accept_goal_result(lifecycle, context, future):
    try:
        status = future.result().status
        lifecycle.result(context, status)
    except Exception as exc:
        context.revoked = True
        context.transport_error = 'goal_result_transport_failed:' + str(exc)
        lifecycle.revoke(context)


def motion_allowed(enabled, inputs_healthy, lifecycle):
    context = lifecycle.active
    return bool(enabled and inputs_healthy and context is not None
                and context.phase == 'accepted' and not context.revoked)


def retire_and_wait(lifecycle, context, spin_until, seconds=5):
    """Revoke once and wait for a terminal action result, not its cancel response."""
    lifecycle.revoke(context)
    return spin_until(lambda: context.phase == 'terminal', seconds)


# /odom and its odom->base_link TF leave the same publisher with the same stamp but on two
# topics, so /odom can arrive first. In simulation both come from Gazebo's OdometryPublisher
# at odom_publish_frequency 30 Hz (robot_sim/fpga_sim/worlds/room.sdf): one TF period is
# 33.3 ms and the default wait is 1.5 periods. On the car the TF source and its rate differ;
# pass --tf-wait-s = 1.0-1.5 x that measured period.
SIM_TF_PERIOD_S = 1 / 30
TF_WAIT_S = 1.5 * SIM_TF_PERIOD_S


ODOM_STAMP_HISTORY = 8


def remember_odom_stamp(state, stamp_ns):
    """Keep the last few /odom stamps so an earlier sample can be released with its own TF."""
    history = state.setdefault('odom_stamps', [])
    history.append(stamp_ns)
    del history[:-ODOM_STAMP_HISTORY]


def release_pair(state, transform_buffer, time_type, stamp_ns):
    """Look the transform up at exactly stamp_ns; on success that pair becomes the pose."""
    transform = transform_buffer.lookup_transform(
        'map', 'base_link', time_type(nanoseconds=stamp_ns))
    returned_ns = (transform.header.stamp.sec*10**9
                   + transform.header.stamp.nanosec)
    if returned_ns != stamp_ns:
        return False
    state['pose'] = planar_pose(transform)
    state['tf_released_ns'] = stamp_ns
    return True


def exact_map_pose(state, transform_buffer, time_type, stamp_ns, ros_ns, receipt_age_s,
                   tf_wait_s):
    """Map pose at exactly an /odom stamp, with tf2 message-filter semantics.

    The newest /odom sample is released only together with the map->base_link transform
    at its own stamp. While that transform has not arrived, for at most tf_wait_s after
    the sample's receipt, the newest earlier /odom sample whose own exact transform has
    arrived, no more than tf_wait_s of simulation time older, is released instead (or
    stays released) and state['pose'] is that exact pair. A transform is only ever looked
    up at an /odom stamp; pairing a stamp with a transform of another time would misalign
    them. Past the wait the input is unhealthy, which the caller handles as before.
    """
    try:
        return release_pair(state, transform_buffer, time_type, stamp_ns)
    except Exception:
        pass
    if not 0 <= receipt_age_s < tf_wait_s:
        return False
    wait_ns = round(tf_wait_s*1e9)
    released = state.get('tf_released_ns')
    earlier = sorted({s for s in list(state.get('odom_stamps', ())) + [released]
                      if s is not None and 0 < stamp_ns-s <= wait_ns and 0 <= ros_ns-s <= 10**9},
                     reverse=True)
    for candidate in earlier:
        if candidate == released:
            return True
        try:
            if release_pair(state, transform_buffer, time_type, candidate):
                return True
        except Exception:
            continue
    return False


def evaluate_health(state, stamps, receipts, topics, now, transform_buffer,
                    time_type, localization, stamp_nanos=None, tf_wait_s=TF_WAIT_S):
    """Evaluate the runtime's exact acquisition-time map pose and lease."""
    if state['fault'] or set(stamps) != set(topics) or now-state['clock_wall'] > .5:
        return False
    if not fresh(state['clock'], stamps, receipts, now):
        return False
    try:
        stamp_ns = ((stamp_nanos or {}).get('/odom')
                    if stamp_nanos is not None else round(stamps['/odom']*1e9))
        if stamp_ns is None:
            return False
        ros_ns = clock_ns(state)
        if not 0 <= ros_ns-stamp_ns <= 10**9:
            return False
        if not exact_map_pose(state, transform_buffer, time_type, stamp_ns, ros_ns,
                              now-receipts['/odom'], tf_wait_s):
            return False
        return localization.healthy(now, ros_ns)
    except Exception:
        return False


# D-035 item 1: the task layer re-sends the same map goal after a same-map correction,
# within the case's unchanged wall limit, at most this many times per case.
MAX_CORRECTION_REISSUES = 3


def evaluate_navigation_tick(state, lifecycle, localization, inputs_healthy,
                             wall, ros_ns):
    """Apply fail-closed lease transitions before deriving a motion permit.

    Order: inputs, then the four-part identity (a generation change keeps its old,
    non-reissuable rule), then the same-map correction rule of D-035.
    """
    if lifecycle.active is not None and not inputs_healthy:
        state['fault'] = 'input_unhealthy_during_action'
        state['enabled'] = False
        lifecycle.revoke(lifecycle.active)
    if (lifecycle.active is not None
            and not localization.permits(lifecycle.active.authorization, wall, ros_ns)):
        lifecycle.revoke(lifecycle.active)
        state['fault'] = state['fault'] or 'localization_generation_changed'
        state['enabled'] = False
    if lifecycle.active is not None and not lifecycle.active.revoked:
        verdict = localization.correction_verdict(lifecycle.active.authorization, wall, ros_ns)
        if verdict is not None:
            lifecycle.revoke(lifecycle.active)
            state['enabled'] = False
            if verdict == MAP_CORRECTION_EXCEEDED:
                # Revoke and cancel, never convert the goal; the task layer re-sends it.
                state['correction'] = verdict
            else:
                state['fault'] = state['fault'] or verdict
    return motion_allowed(state['enabled'], inputs_healthy, lifecycle)


def goal_authorization(localization, wall, ros_ns):
    """The authorization a new goal is sent under, refusing an estimated one without a reference."""
    authorization = localization.identity(wall, ros_ns)
    if authorization is None:
        raise RuntimeError('navigation_authorization_unavailable')
    if localization.mode == 'estimated' and getattr(authorization, 'reference', None) is None:
        raise RuntimeError('map_correction_reference_unavailable')
    return authorization


def correction_reissue_due(state, context, reissues):
    """After a terminal result: re-send only a goal the correction rule itself revoked."""
    if state.get('correction') != MAP_CORRECTION_EXCEEDED or state['fault']:
        return False
    if not context.revoked or context.phase != 'terminal' or context.transport_error:
        return False
    if reissues >= MAX_CORRECTION_REISSUES:
        state['fault'] = 'map_correction_reissue_budget_exhausted'
        return False
    return True


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--c-worker', action='store_true', help='single navigation goal; external C injector determines fault verdict')
    parser.add_argument('--c-case', choices=('baseline', 'command_loss', 'permit_loss',
                        'input_loss', 'runner_exit', 'obstacle', 'wall', 'removal'),
                        help='declared C case; wall requires an explicit ABORTED action result')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--pose-source', choices=('reference', 'estimated'), default='reference')
    parser.add_argument('--localization-map-id')
    parser.add_argument('--navigation-map-id')
    parser.add_argument('--tf-wait-s', type=float, default=TF_WAIT_S,
                        help='longest wait for the exact-time odom->base_link TF after its /odom '
                             'sample; 1.0-1.5 TF periods (default: 1.5 x the 30 Hz simulated TF)')
    args = parser.parse_args(argv)
    if not 0 < args.tf_wait_s <= .2:
        parser.error('--tf-wait-s must be in (0, 0.2] seconds: one to one-and-a-half TF periods')
    if bool(args.c_case) != args.c_worker:
        parser.error('--c-case and --c-worker must be used together')
    try:
        localization = NavigationLocalization(args.pose_source, args.localization_map_id,
                                              args.navigation_map_id)
    except ValueError as exc:
        parser.error(str(exc))
    # D-035 item 3: unlocked once items 1-2 (the same-map correction rule and its goal
    # reference) were implemented and tested (tests/test_nav_correction_rules.py). This
    # admits the consumer only; a navigable map and the real car remain unverified.
    return args, localization


def main():
    args, localization = parse_arguments()
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, Path as NavPath
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2, Image, CameraInfo
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('dev2_nav_point_to_point')
    output = args.out.open('x')
    begin = time.monotonic()
    state = dict(clock=-1., clock_wall=0., pose=None,
                 stopped=0, enabled=False,
                 fault=None, correction=None, case=None, last_permit=None, sample=0)
    stamps, receipts, stamp_nanos, pending = {}, {}, {}, {}
    raw_stamps, raw_receipts = {}, {}
    tf = Buffer(); listener = TransformListener(tf, node)
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    lifecycle = GoalLifecycle()
    permit = node.create_publisher(Bool, '/fpga/motion_permit', 1)
    topics = {'/odom': Odometry, '/fpga/points': PointCloud2, '/fpga/rgb': Image,
              '/fpga/depth': Image, '/fpga/camera_info': CameraInfo}

    def log(event, **fields):
        row = dict(event=event, wall_s=time.monotonic()-begin, wall_ns=time.time_ns(),
                   wall_monotonic=time.monotonic(), sim_s=state['clock'],
                   case=state['case'], **fields)
        output.write(json.dumps(row, allow_nan=False)+'\n'); output.flush()

    def clock(msg):
        value = msg.clock.sec+msg.clock.nanosec/1e9
        if value < state['clock']: state['fault'] = 'clock_rewind'
        if value > state['clock']: state['clock_wall'] = time.monotonic()
        state['clock'] = value
        promote(value,pending,stamps,receipts,stamp_nanos)

    def receive(topic, msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        raw_stamps[topic], raw_receipts[topic] = stamp, time.monotonic()
        stamp_ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if topic == '/odom':
            remember_odom_stamp(state, stamp_ns)
        pending.setdefault(topic,[]).append((stamp,raw_receipts[topic],stamp_ns))
        promote(state['clock'],pending,stamps,receipts,stamp_nanos)
        if topic == '/odom':
            v = msg.twist.twist
            state['stopped'] = state['stopped']+1 if abs(v.linear.x)+abs(v.angular.z)<.01 else 0
            state['sample'] += 1
            if state['sample'] % 5 == 0:
                log('odom', pose=state['pose'], linear=v.linear.x, angular=v.angular.z)

    def plan(msg):
        log('plan', frame=msg.header.frame_id, poses=[[p.pose.position.x,p.pose.position.y] for p in msg.poses])

    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    subscriptions = [node.create_subscription(k,t,lambda m,t=t:receive(t,m),qos_profile_sensor_data) for t,k in topics.items()]
    node.create_subscription(NavPath, '/plan', plan, 10)

    def localization_status(msg):
        localization.update_json(msg.data, time.monotonic(), clock_ns(state))

    if args.pose_source == 'estimated':
        node.create_subscription(String, '/fpga/localization_status', localization_status, 10)

    def healthy():
        return evaluate_health(state, stamps, receipts, topics, time.monotonic(),
                               tf, Time, localization, stamp_nanos, args.tf_wait_s)

    def tick():
        ok=healthy()
        allowed=evaluate_navigation_tick(
            state, lifecycle, localization, ok, time.monotonic(),
            clock_ns(state))
        permit.publish(Bool(data=allowed))
        if allowed!=state['last_permit']:
            log('permit', allowed=allowed, reason=state['fault'], stamps=stamps,
                receipt_ages={t:time.monotonic()-v for t,v in receipts.items()},
                raw_stamps=raw_stamps, raw_receipt_ages={t:time.monotonic()-v for t,v in raw_receipts.items()},
                clock_receipt_age_s=time.monotonic()-state['clock_wall'],
                pending_stamps={t:[v[0] for v in samples] for t,samples in pending.items()})
            state['last_permit']=allowed

    node.create_timer(.05,tick)  # default wall clock, not /clock

    def spin_until(predicate, seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate(): return True
        return False

    def revoke():
        state['enabled']=False
        permit.publish(Bool(data=False))

    results=[]; handle=None
    try:
        log('begin', cases=CASES, per_goal_wall_limit_s=90, cancel_after_displacement_m=.15,
            xy_tolerance_m=.03,yaw_tolerance_rad=.03,stop_speed_sum_limit=.01,stopped_samples=5)
        if not spin_until(lambda:healthy() and action.server_is_ready(),15):
            raise RuntimeError('initial_input_or_action_not_ready')
        if args.c_worker:
            ready_since=None
            def settled_inputs():
                nonlocal ready_since
                now=time.monotonic()
                ready_since=(ready_since or now) if healthy() else None
                return ready_since is not None and now-ready_since>=3
            if not spin_until(settled_inputs,15):raise RuntimeError('C_inputs_not_continuously_ready')
        if not within_goal(state['pose'], (0,0,0)): raise RuntimeError('unexpected_world_start')
        for name, target in ([('c_navigation',(0.,2.2,math.pi/2))] if args.c_worker else CASES):
            state['case']=name
            if not healthy(): raise RuntimeError('unhealthy_before_goal')
            origin=state['pose']; started=time.monotonic(); cancelled=False; reissues=0
            while True:  # D-035: a same-map correction re-sends the same map goal, never converts it
                authorization = goal_authorization(localization, time.monotonic(), clock_ns(state))
                goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
                ns=clock_ns(state);goal.pose.header.stamp.sec=ns//1000000000;goal.pose.header.stamp.nanosec=ns%1000000000
                goal.pose.pose.position.x,goal.pose.pose.position.y=target[:2]
                goal.pose.pose.orientation.z=math.sin(target[2]/2);goal.pose.pose.orientation.w=math.cos(target[2]/2)
                context, request = send_goal(action, lifecycle, authorization, goal)
                if not spin_until(lambda: context.response_processed, 5):
                    lifecycle.revoke(context)
                    if spin_until(lambda: context.response_processed, 5):
                        spin_until(lambda: context.phase == 'terminal', 5)
                    log('navigation_terminal_unconfirmed', phase=context.phase,
                        result_available=context.result_future is not None,
                        terminal=context.phase == 'terminal', detail=context.transport_error)
                    raise RuntimeError('goal_response_timeout')
                handle=context.handle
                if context.transport_error:
                    raise RuntimeError(context.transport_error)
                reference=getattr(authorization,'reference',None)
                log('goal_response',accepted=handle.accepted,goal_id=[int(x) for x in handle.goal_id.uuid],target=target,origin=origin,
                    reissue=reissues,reference_stamp_ns=reference.stamp_ns if reference else None)
                if not handle.accepted: raise RuntimeError('goal_rejected')
                result=context.result_future;state['enabled']=not context.revoked
                while context.phase != 'terminal' and time.monotonic()-started<90:
                    rclpy.spin_once(node,timeout_sec=.02)
                    if context.transport_error:
                        state['fault'] = context.transport_error
                        break
                    if state['fault']: break
                    if name=='cancel' and not cancelled and math.dist(origin[:2],state['pose'][:2])>=.15:
                        revoke(); request=lifecycle.revoke(context); cancelled=True
                        if request is None: raise RuntimeError('cancel_not_sent')
                        if not spin_until(request.done,5): raise RuntimeError('cancel_response_timeout')
                        lifecycle.cancel_response(context)
                        log('cancel_response',return_code=request.result().return_code,
                            goals_canceling=len(request.result().goals_canceling),pose=state['pose'])
                        if request.result().return_code!=0: raise RuntimeError('cancel_rejected')
                if not correction_reissue_due(state, context, reissues):
                    break
                reissues += 1
                revoke()
                log('goal_reissued', cause=state['correction'], reissue=reissues,
                    revoked_status=context.terminal_status, pose=state['pose'])
                state['correction'] = None
                # Re-authorise under the current approval; the case's wall limit is not extended.
                if not spin_until(lambda: healthy() and localization.identity(
                        time.monotonic(), clock_ns(state)) is not None,
                        max(0., 90-(time.monotonic()-started))):
                    state['fault'] = 'reauthorization_timeout_after_correction'
                    break
            revoke()
            if context.phase != 'terminal':
                reason = state['fault'] or 'goal_wall_timeout'
                # Terminal cleanup cannot extend the case's acceptance window.
                state['fault'] = reason
                if not retire_and_wait(lifecycle, context, spin_until):
                    log('navigation_terminal_unconfirmed', phase=context.phase,
                        cancel_sent=context.cancel_sent, reason=reason,
                        detail=context.transport_error)
                    raise RuntimeError(reason + ':terminal_result_timeout')
            status=context.terminal_status
            state['stopped']=0
            stopped=spin_until(lambda:state['stopped']>=5 and healthy(),2)
            wall_case = args.c_worker and args.c_case == 'wall'
            expected_wall_abort = wall_case and status == 6
            passed=(not state['fault'] and stopped and (expected_wall_abort or
                    (name=='cancel' and cancelled and status==5) or
                    (name=='unreachable' and status==6) or
                    (not wall_case and name not in ('cancel','unreachable') and status==4
                     and within_goal(state['pose'],target))))
            row=dict(name=name,passed=passed,status=status,pose=state['pose'],target=target,
                     stopped=stopped,elapsed_wall_s=time.monotonic()-started,
                     expected_outcome='ABORTED' if wall_case else 'SUCCEEDED')
            results.append(row);log('case_result',**row);handle=None
            if not passed: raise RuntimeError('case_failed:'+name)
        log('summary',passed=True,results=results)
    except Exception as exc:
        log('summary',passed=False,error=str(exc),results=results)
        raise
    finally:
        revoke()
        if lifecycle.active is not None:
            lifecycle.revoke(lifecycle.active)
        spin_until(lambda:False,1)
        output.close();action.destroy();node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
