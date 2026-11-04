"""Real ROS DDS/action transport controls with fake localization/Nav2/empty inference.

Each invocation runs one finite scenario and a fresh full Runtime subprocess.
No devices, SLAM engine, physical motion, or hardware timing claims.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

SCENARIOS = ('healthy', 'epoch', 'instance', 'local_invalid', 'heartbeat_expiry', 'late_accept', 'cancel_receipt')


def fresh_ready(rows, boundary, minimum_stamp):
    frames = [(i, t, row) for i, (t, row) in enumerate(rows) if row['event'] == 'frame_processed']
    if not frames:
        return False
    index, received, row = frames[-1]
    return bool(received >= boundary and row['input_ready']
                and row['acquisition_timestamp'] >= minimum_stamp/1e9
                and not any(later['event'] == 'fault' for _, later in rows[index+1:]))


def fresh_permit(permits, boundary):
    return any(t >= boundary and value for t, value in permits)


def continuity_window(report):
    """Audit the entire old-request interval after a changed-authority frame was processed."""
    boundary = next(e['wall'] for e in report['events'] if e['event'] == 'scenario_boundary')
    before = [e for e in report['events'] if e['event'] == 'status_schedule' and e['delivered'] and e['wall'] < boundary]
    previous = before[-1]
    changed = [e for e in report['events'] if e['event'] == 'status_schedule' and e['delivered']
               and e['wall'] >= boundary and (e['instance'], e['epoch']) != (previous['instance'], previous['epoch'])]
    if not changed:
        raise ValueError('no delivered changed-authority status')
    stamps = {e['stamp_ns'] for e in changed}
    witness = next((t for t, r in report['runtime'] if t >= boundary and r['event'] == 'frame_processed'
                    and r['frame_id'] in {'rgb-'+str(stamp) for stamp in stamps}), None)
    terminal = next((t for t, r in report['runtime'] if t >= boundary and r['event'] == 'navigation_result'), None)
    if witness is None or terminal is None or witness >= terminal:
        raise ValueError('missing processed changed-authority frame before terminal receipt')
    following = next((t for t, r in report['runtime'] if t > terminal and r['event'] == 'navigation_started'), None)
    if following is None:
        raise ValueError('no explicit following request boundary')
    window = [(t, value) for t, value in report['permits'] if witness <= t < following]
    if not window:
        raise ValueError('no permission observations in old-request window')
    return dict(processed_witness=witness, terminal_receipt=terminal, following_request=following,
                true_permits=[t for t, value in window if value],
                passed=not any(value for _, value in window))


def stage_durations(stages):
    result = []
    for before, after in zip(stages, stages[1:]):
        wall, cpu = after['wall']-before['wall'], after['thread_cpu']-before['thread_cpu']
        if wall < 0 or cpu < 0:
            raise ValueError('nonmonotonic diagnostic clock')
        result.append(dict(start=before['stage'], end=after['stage'], wall_seconds=wall, cpu_seconds=cpu))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path); p.add_argument('--scenario', choices=SCENARIOS, required=True)
    p.add_argument('--schedule-diagnostics', action='store_true')
    a = p.parse_args(); a.output.mkdir(exist_ok=False)
    import numpy as np
    import rclpy
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Bool, Empty, String
    from tf2_msgs.msg import TFMessage
    from host.inference_service import serve
    from host.localization_gate import LocalizationGate, KINDS
    from tools.rtabmap_frame_disposition_probe import observation

    map_id = 'a'*64
    rclpy.init(); node = Node('navigation_continuity_driver')
    executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    stop = threading.Event(); accept_release = threading.Event(); terminal_release = threading.Event()
    if a.scenario != 'late_accept': accept_release.set()
    events, rows, permits, received_goals, executing, cancel_requests = [], [], [], [], [], []
    checks, files = [], []; lock = threading.RLock(); stages = []
    gate = LocalizationGate(map_id, {1}); gate.instance = 'ros-fixture-A'
    paused = False; stamp = 10**10; last_status = None
    child = None; feeder = None
    def event(name, **fields):
        with lock: events.append(dict(event=name, wall=time.monotonic(), **fields))
    def keep(target, convert):
        def callback(msg):
            with lock: target.append((time.monotonic(), convert(msg)))
        return callback
    node.create_subscription(String, '/fpga/status', keep(rows, lambda m: json.loads(m.data)), 100)
    node.create_subscription(Bool, '/fpga/motion_permit', keep(permits, lambda m: m.data), 100)
    status_pub = node.create_publisher(String, '/fpga/localization_status', 100)
    rgb = node.create_publisher(Image, '/fpga/rgb', 10); depth = node.create_publisher(Image, '/fpga/depth', 10)
    camera = node.create_publisher(CameraInfo, '/fpga/camera_info', 10)
    clock = node.create_publisher(Clock, '/clock', 10)
    patrol = node.create_publisher(Empty, '/fpga/patrol', 10)
    cancel = node.create_publisher(Empty, '/fpga/cancel', 10)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    static = node.create_publisher(TFMessage, '/tf_static', durable)
    grid_pub = node.create_publisher(OccupancyGrid, '/map', durable)
    def goal_callback(_):
        with lock: received_goals.append(time.monotonic())
        event('goal_request')
        if not accept_release.wait(4): return GoalResponse.REJECT
        event('goal_accept_response'); return GoalResponse.ACCEPT
    def cancel_callback(_):
        with lock: cancel_requests.append(time.monotonic())
        event('cancel_response_accept'); return CancelResponse.ACCEPT
    def execute(handle):
        with lock: executing.append(time.monotonic())
        event('goal_execute')
        while not stop.is_set() and not terminal_release.wait(.01): pass
        if handle.is_cancel_requested:
            handle.canceled(); event('terminal_canceled')
        elif stop.is_set():
            handle.abort(); event('terminal_aborted_cleanup')
        else:
            handle.succeed(); event('terminal_succeeded')
        return NavigateToPose.Result()
    server = ActionServer(node, NavigateToPose, 'navigate_to_pose', execute,
                          goal_callback=goal_callback, cancel_callback=cancel_callback,
                          callback_group=ReentrantCallbackGroup())
    spinner = threading.Thread(target=executor.spin, daemon=True); spinner.start()
    sock = a.output/'inference.sock'
    threading.Thread(target=serve, args=(sock, lambda frame: [], 'cpu'), daemon=True).start()
    def wait_for(predicate, timeout=3):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if child is not None and child.poll() is not None: raise RuntimeError('runtime exited')
            with lock:
                if predicate(): return True
            time.sleep(.01)
        return False
    def check(name, condition):
        passed = bool(condition); checks.append(dict(check=name, passed=passed)); event('check', criterion=name, passed=passed)
        print(name, passed, flush=True)
        if not passed: raise AssertionError(name)
    def processed(): return [r for _,r in rows if r['event'] == 'frame_processed']
    def stage(label):
        if a.schedule_diagnostics:
            stages.append(dict(stage=label, wall=time.monotonic(), thread_cpu=time.thread_time()))
    def cpu_stat():
        for path in ('/sys/fs/cgroup/cpu.stat','/sys/fs/cgroup/cpu/cpu.stat'):
            if Path(path).exists(): return dict(path=path, value=Path(path).read_text())
        return None
    cpu_before = cpu_stat() if a.schedule_diagnostics else None
    def feed():
        nonlocal stamp, last_status
        try:
            while not stop.is_set():
                stage('before_lock')
                with lock:
                    stage('lock_acquired')
                    if not paused:
                        stamp += 10**8; now = time.monotonic(); sec,nano = divmod(stamp,10**9)
                        c=Clock();c.clock.sec,c.clock.nanosec=sec,nano
                        stage('before_clock_publish');clock.publish(c);stage('after_clock_publish')
                        gate.acquisition(stamp,now,stamp)
                        for kind in KINDS: gate.ingest(kind,observation(kind,stamp),now,stamp)
                        row=gate.status(now,stamp)
                        stage('after_gate')
                        # Omit intermediate invalid states by construction; log every omission.
                        event('status_schedule', delivered=row['state']=='valid', instance=row['instance'],epoch=row['epoch'],sequence=row['sequence'],stamp_ns=stamp)
                        if row['state']=='valid':
                            stage('before_status_publish');status_pub.publish(String(data=json.dumps(row)))
                            stage('after_status_publish');last_status=row
                        im=Image();im.header.frame_id='camera_optical_frame';im.header.stamp.sec,im.header.stamp.nanosec=sec,nano
                        im.height=im.width=20;im.encoding='rgb8';im.step=60;im.data=bytes(20*20*3)
                        info=CameraInfo();info.header=im.header;info.width=info.height=20
                        info.k=[20.,0.,10.,0.,20.,10.,0.,0.,1.]
                        stage('before_camera_publish');camera.publish(info);stage('after_camera_publish')
                        # Keep initial/confirmation images out of this navigation-specific control.
                        if row['state']=='valid':
                            stage('before_rgb_publish');rgb.publish(im);stage('after_rgb_publish')
                            dm=Image();dm.header=im.header;dm.width=dm.height=20
                            dm.encoding='32FC1';dm.step=80;dm.data=np.ones((20,20),np.float32).tobytes()
                            stage('before_depth_publish');depth.publish(dm);stage('after_depth_publish')
                stage('lock_released')
                stop.wait(.1)
                stage('after_wait')
        except Exception as exc:
            event('feeder_error', detail=str(exc)); stop.set()
    try:
        log=(a.output/'runtime.log').open('x');files.append(log)
        argv=[sys.executable,'-m','host.ros_runtime','--socket',str(sock),'--pose-source','estimated',
              '--sensor-source','simulation','--inference-device','cpu','--localization-map-id',map_id,
              '--events',str(a.output/'runtime-events.jsonl')]
        event('runtime_launch', argv=argv)
        child=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        check('ROS subscriptions ready',wait_for(lambda: status_pub.get_subscription_count() and patrol.get_subscription_count() and rgb.get_subscription_count() and camera.get_subscription_count()))
        transforms=[]
        for parent,child_frame in [('base_link','camera_link'),('camera_link','camera_optical_frame')]:
            t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child_frame;t.transform.rotation.w=1.;transforms.append(t)
        static.publish(TFMessage(transforms=transforms))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=100
        grid.info.resolution=.1;grid.info.origin.position.x=grid.info.origin.position.y=-5.
        grid.info.origin.orientation.w=1.;grid.data=[0]*10000;grid_pub.publish(grid)
        feeder=threading.Thread(target=feed,daemon=True);feeder.start()
        check('real runtime input positive',wait_for(lambda: processed() and processed()[-1]['input_ready'] and processed()[-1]['navigation_available']))
        patrol.publish(Empty());check('goal request sent',wait_for(lambda: received_goals))
        if a.scenario != 'late_accept':
            check('accepted motion positive',wait_for(lambda: executing and permits and permits[-1][1]))
        boundary=time.monotonic();event('scenario_boundary')
        if a.scenario=='healthy':
            initial=len(processed());check('healthy multi-frame navigation survives',wait_for(lambda: len(processed())>=initial+6 and permits[-1][1] and not cancel_requests))
            terminal_release.set();check('normal result continues patrol',wait_for(lambda: len(received_goals)>=2))
        else:
            with lock:
                if a.scenario in ('epoch','late_accept'):
                    gate.revoke('lost','controlled_epoch_change')
                elif a.scenario=='instance':
                    gate=LocalizationGate(map_id,{1});gate.instance='ros-fixture-B'
                elif a.scenario=='local_invalid':
                    status_pub.publish(String(data='invalid-json'));event('local_invalid_delivery')
                elif a.scenario=='heartbeat_expiry': paused=True
                elif a.scenario=='cancel_receipt': cancel.publish(Empty())
            if a.scenario=='late_accept':
                check('replacement valid arrives before acceptance',wait_for(lambda: last_status and last_status['epoch']>0))
                # Runtime's status event/fault proves delivery, not merely a publisher-side mutation.
                check('pending request retired before acceptance',wait_for(lambda: any(t>=boundary and r['event']=='fault' and r['reason']=='localization_invalid' for t,r in rows)))
                accept_release.set()
            check('cancel transported to action server',wait_for(lambda: cancel_requests))
            check('permit false after interruption',wait_for(lambda: permits and not permits[-1][1]))
            with lock:
                paused=False
                recovery_boundary=time.monotonic(); recovery_stamp=stamp+10**8
                event('recovery_observation_boundary', minimum_stamp=recovery_stamp)
            check('new sensing recovers while old action retained',wait_for(
                lambda: last_status and last_status['stamp_ns'] >= recovery_stamp
                and (last_status['instance'], last_status['epoch']) == (gate.instance, gate.epoch)
                and fresh_ready(rows, recovery_boundary, recovery_stamp)))
            patrol.publish(Empty())
            check('new request rejected before terminal',wait_for(lambda: any(t>=boundary and r['event']=='patrol_rejected' and r['navigation_busy'] for t,r in rows)))
            with lock: check('cancel response did not release occupancy',len(received_goals)==1 and not permits[-1][1])
            terminal_release.set()
            check('terminal receipt retires old request',wait_for(lambda: any(t>=boundary and r['event']=='navigation_retired' for t,r in rows)))
            with lock:
                terminal_boundary=time.monotonic(); terminal_stamp=stamp+10**8
                event('post_terminal_recovery_boundary', minimum_stamp=terminal_stamp)
            check('fresh input after terminal before explicit task',wait_for(
                lambda: last_status and last_status['stamp_ns'] >= terminal_stamp
                and (last_status['instance'], last_status['epoch']) == (gate.instance, gate.epoch)
                and fresh_ready(rows, terminal_boundary, terminal_stamp)))
            # Prevent the next explicitly requested goal from completing immediately.
            terminal_release.clear();new_request_boundary=time.monotonic();patrol.publish(Empty())
            check('explicit new request after terminal',wait_for(lambda: len(received_goals)==2))
            check('explicit new request can move',wait_for(lambda: fresh_permit(permits,new_request_boundary)))
            if a.scenario in ('epoch','instance','late_accept'):
                audit=continuity_window(dict(events=events,runtime=rows,permits=permits))
                event('continuity_interval', **audit)
                check('entire retired-request interval never permits motion',audit['passed'])
    finally:
        stop.set();accept_release.set();terminal_release.set()
        if feeder:feeder.join(1)
        if child and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(2)
        with (a.output/'result.json').open('x') as f:
            json.dump(dict(scenario=a.scenario,checks=checks,events=events,runtime=rows,permits=permits,
                           goals=received_goals,cancel_requests=cancel_requests,child_stopped=child is None or child.poll() is not None,
                           limitation='real DDS/action transport and full Runtime; fake localization, fake Nav2, empty detector; no physical stop'),f,indent=2)
        if a.schedule_diagnostics:
            with (a.output/'schedule-diagnostics.json').open('x') as f:
                json.dump(dict(stages=stages,segments=stage_durations(stages),cpu_before=cpu_before,cpu_after=cpu_stat()),f,indent=2)
        server.destroy();executor.shutdown(timeout_sec=2);node.destroy_node();rclpy.shutdown()
        for f in files:f.close()
if __name__=='__main__':main()
