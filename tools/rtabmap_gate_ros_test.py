#!/usr/bin/env python3
"""Isolated ROS integration probe with explicit fake perception/Nav2, no hardware.

Runs real gate, runtime, TF, ROS action transport and velocity guard. A synthetic
pose has x=1 while raw TF is poisoned with x=99. Fake perception serves empty
results and can block a request until after localization loss.
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

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--database', required=True)
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--gate-diagnostics',action='store_true')
    parser.add_argument('--consumer-diagnostics', action='store_true')
    parser.add_argument('--startup-only', action='store_true')
    parser.add_argument('--startup-ready', action='store_true')
    a = parser.parse_args()
    if a.startup_ready and not a.consumer_diagnostics:
        parser.error('--startup-ready requires --consumer-diagnostics')
    a.output.mkdir(exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rosidl_runtime_py.set_message import set_message_fields
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String, Empty, Bool
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from nav2_msgs.action import NavigateToPose
    from rtabmap_msgs.msg import Info, OdomInfo
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from host.inference_service import serve
    from host.localization_gate import KINDS
    from host.ros_localization_gate import load_prior
    prior_id = min(load_prior(a.database, a.map_id))
    children, files, checks = [], [], []
    rows, statuses, poses, transforms, permits, safe, goals, cancelled, requests = [], [], [], [], [], [], [], [], []
    infrastructure_acks, discovery_observations = [], []
    lock = threading.Lock()
    delayed = threading.Event()
    release = threading.Event()
    block_next = threading.Event()
    terminate = threading.Event()
    sock = a.output/'fake-inference.sock'

    def detect(frame):
        requests.append({'frame_id': frame.frame_id, 'position': frame.T_world_camera[:3, 3].tolist()})
        if block_next.is_set():
            block_next.clear()
            delayed.set()
            release.wait(3)
        return []

    threading.Thread(target=serve, args=(sock, detect, 'cpu'), daemon=True).start()
    rclpy.init()
    node = Node('gate_integration_driver')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    def keep(target, convert=lambda m: m):
        def callback(message):
            with lock:
                target.append((time.monotonic(), convert(message)))
        return callback
    node.create_subscription(String, '/fpga/status', keep(rows, lambda m: json.loads(m.data)), 100)
    node.create_subscription(String, '/fpga/localization_status', keep(statuses, lambda m: json.loads(m.data)), 100)
    node.create_subscription(PoseWithCovarianceStamped, '/fpga/localization_pose', keep(poses), 100)
    node.create_subscription(TFMessage, '/fpga/localization_tf', keep(transforms), 100)
    node.create_subscription(Bool, '/fpga/motion_permit', keep(permits, lambda m: m.data), 100)
    node.create_subscription(Twist, '/fpga/cmd_vel_safe', keep(safe, lambda m: m.linear.x), 100)
    if a.consumer_diagnostics:
        node.create_subscription(String, '/fpga/diagnostic_infrastructure_ready',
                                 keep(infrastructure_acks, lambda m: json.loads(m.data)), 10)
    def execute(handle):
        goals.append(time.monotonic())
        while not terminate.is_set() and not handle.is_cancel_requested:
            time.sleep(.01)
        if handle.is_cancel_requested:
            cancelled.append(time.monotonic())
            handle.canceled()
        else:
            handle.abort()
        return NavigateToPose.Result()
    server = ActionServer(node, NavigateToPose, 'navigate_to_pose', execute,
                          goal_callback=lambda _: GoalResponse.ACCEPT,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    clock = node.create_publisher(Clock, '/clock', 10)
    raw_rgb = node.create_publisher(Image, '/probe/rgb', 10)
    rgb = node.create_publisher(Image, '/fpga/rgb', 10)
    depth = node.create_publisher(Image, '/fpga/depth', 10)
    camera = node.create_publisher(CameraInfo, '/fpga/camera_info', 10)
    patrol = node.create_publisher(Empty, '/fpga/patrol', 10)
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    pubtypes = dict(zip(KINDS, (Odometry, OdomInfo, Info, PoseWithCovarianceStamped)))
    pubs = {k: node.create_publisher(t, '/probe/'+k, 10) for k, t in pubtypes.items()}
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    static = node.create_publisher(TFMessage, '/tf_static', durable)
    raw_tf = node.create_publisher(TFMessage, '/tf', 10)
    grid_pub = node.create_publisher(OccupancyGrid, '/map', durable)
    stamp = 10**10

    def launch(label, command):
        log = (a.output/(label+'.log')).open('x'); files.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((label, child)); return child

    def wait_for(predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            for label, child in children:
                if child.poll() is not None and label != 'gate-stopped':
                    raise RuntimeError(label+' exited '+str(child.returncode))
            if predicate():
                return True
            time.sleep(.01)
        return False

    def check(name, predicate):
        success = bool(predicate)
        checks.append({'check': name, 'pass': success})
        print(name, success, flush=True)
        if not success:
            raise AssertionError(name)

    def send_frame(bad=None, images=True):
        nonlocal stamp
        stamp += 100000000
        sec, nano = divmod(stamp, 10**9)
        c = Clock(); c.clock.sec, c.clock.nanosec = sec, nano; clock.publish(c)
        time.sleep(.03)
        im = Image(); im.header.frame_id = 'camera_optical_frame'
        im.header.stamp.sec, im.header.stamp.nanosec = sec, nano
        im.height = im.width = 20; im.encoding = 'rgb8'; im.step = 60
        im.data = bytes(20*20*3)
        raw_rgb.publish(im)
        info = CameraInfo(); info.header = im.header; info.width = info.height = 20
        info.k = [20., 0., 10., 0., 20., 10., 0., 0., 1.]; camera.publish(info)
        if images:
            rgb.publish(im)
            dm = Image(); dm.header = im.header; dm.width = dm.height = 20
            dm.encoding = '32FC1'; dm.step = 80; dm.data = np.ones((20,20),np.float32).tobytes(); depth.publish(dm)
        time.sleep(.035)
        for kind in KINDS:
            row = {'header': {'stamp': {'sec': sec, 'nanosec': nano},
                              'frame_id': 'odom' if kind in KINDS[:2] else 'map'}}
            pose = {'position': dict(x=1., y=0., z=0.), 'orientation': dict(x=0., y=0., z=0., w=1.)}
            cov = [float(i%7 == 0)*.01 for i in range(36)]
            if kind == 'odom':
                row.update(child_frame_id='camera_link', pose={'pose':pose,'covariance':cov})
            elif kind == 'localization_pose':
                row['pose'] = {'pose':pose,'covariance':cov}
            elif kind == 'odom_info':
                row.update(lost=bad == 'lost', covariance=cov)
            else:
                row.update(loop_closure_id=prior_id if bad != 'no_match' else 0)
            if bad == 'initial' and kind in ('odom', 'localization_pose'):
                row['pose']['covariance'][0] = 9999.
            if bad == 'missing' and kind == 'info':
                continue
            msg = pubtypes[kind](); set_message_fields(msg, row); pubs[kind].publish(msg)
        time.sleep(.12)
        return stamp

    def processed():
        return [r for _, r in rows if r['event'] == 'frame_processed']

    try:
        gate = launch('gate', [sys.executable,'-m','host.ros_localization_gate','--database',a.database,
             '--map-id',a.map_id,*(['--events',str(a.output/'gate-events.jsonl'),'--diagnostics'] if a.gate_diagnostics else []),
             '--ros-args','-p','use_sim_time:=true'])
        launch('runtime', [sys.executable,'-m','host.ros_runtime','--socket',str(sock),'--pose-source','estimated',
            '--sensor-source','simulation','--inference-device','cpu','--localization-map-id',a.map_id,
            '--events',str(a.output/'runtime-events.jsonl'),
            *(['--diagnostics', str(a.output/'runtime-diagnostics.json')] if a.consumer_diagnostics else [])])
        launch('velocity', [sys.executable,'robot_sim/fpga_sim/scripts/velocity_guard.py'])
        check('ROS discovery', wait_for(lambda: raw_rgb.get_subscription_count() and pubs['info'].get_subscription_count()
                                        and patrol.get_subscription_count() and statuses and rows))
        ts = []
        for parent, child, x in [('base_link','camera_link',0.),('camera_link','camera_optical_frame',0.),
                                  ('map','base_link',99.)]:
            t = TransformStamped(); t.header.frame_id=parent; t.child_frame_id=child
            t.transform.rotation.w=1.; t.transform.translation.x=x; ts.append(t)
        static.publish(TFMessage(transforms=ts))
        raw_tf.publish(TFMessage(transforms=[ts[-1]]))
        grid = OccupancyGrid(); grid.header.frame_id='map'; grid.info.width=grid.info.height=100
        grid.info.resolution=.1; grid.info.origin.position.x=grid.info.origin.position.y=-5.
        grid.info.origin.orientation.w=1.; grid.data=[0]*10000; grid_pub.publish(grid)
        time.sleep(.15)
        def infrastructure_discovered():
            values = {name: pub.get_subscription_count() for name, pub in
                      [('raw_rgb', raw_rgb), ('rgb', rgb), ('depth', depth), ('camera_info', camera),
                       ('clock', clock), ('static_tf', static), ('map', grid_pub), *list(pubs.items())]}
            discovery_observations.append({'monotonic': time.monotonic(), 'subscriptions': values,
                                           'consumer_ack_received': bool(infrastructure_acks)})
            return bool(infrastructure_acks) and all(values.values())
        if a.consumer_diagnostics:
            if a.startup_ready:
                check('consumer infrastructure barrier', wait_for(infrastructure_discovered))
            else:
                infrastructure_discovered()
        send_frame('initial'); patrol.publish(Empty()); time.sleep(.1)
        check('initial pose and navigation blocked', not poses and not goals and not requests)
        send_frame(); send_frame(); good_stamp=send_frame()
        # Initial queued image can take its original .5s bounded timeout to drain.
        for _ in range(3):
            send_frame()
        check('normal inference available', wait_for(lambda: processed()))
        if a.startup_only:
            return
        check('raw and static map TF cannot bypass approved geometry', all(abs(r['position'][0]-1)<1e-6 for r in requests))
        check('approved pose and TF positive control', poses and transforms)
        patrol.publish(Empty())
        check('goal accepted under valid localization', wait_for(lambda: goals))
        check('motion permit true positive control', wait_for(lambda: any(v for _,v in permits)))
        velocity = Twist(); velocity.linear.x=.12; cmd.publish(velocity)
        check('velocity positive control', wait_for(lambda: any(v>.1 for _,v in safe)))
        block_next.set(); send_frame()
        check('inference in flight', delayed.wait(2))
        blocked_frame = requests[-1]['frame_id']
        before = time.monotonic(); send_frame('lost', images=False)
        check('loss cancels active ROS action', wait_for(lambda: cancelled, 1))
        check('loss revokes permit and zeroes guarded velocity', wait_for(lambda: any(t>=before and not v for t,v in permits)
                          and any(t>=before and v==0 for t,v in safe), 1))
        # Recover before releasing the old result, while its acquisition is still fresh.
        # This exercises epoch revocation, not only a currently-false status check.
        for _ in range(3):
            send_frame(images=False)
        check('localization recovered before old inference completes', statuses[-1][1]['state']=='valid')
        release.set(); time.sleep(.2)
        check('late inference cannot write memory', not any(r['frame_id']==blocked_frame for r in processed()))
        check('late inference rejection observed', any(r['event']=='frame_rejected' and r['frame_id']==blocked_frame for _,r in rows))
        before_goals = len(goals)
        for _ in range(5):
            send_frame()
        check('recovery restores input', wait_for(lambda: statuses[-1][1]['state']=='valid' and processed()[-1]['input_ready']))
        check('recovery never resumes old task', len(goals)==before_goals and not permits[-1][1])
        patrol.publish(Empty()); check('explicit new task can navigate', wait_for(lambda: len(goals)>before_goals))
        before=time.monotonic(); send_frame('missing', images=False)
        check('missing stream causes bounded stop', wait_for(lambda: any(t>=before and r['state']=='stale' for t,r in statuses)
                                                          and not permits[-1][1], 1))
        for _ in range(5):
            send_frame()
        patrol.publish(Empty()); check('new task after missing-stream recovery', wait_for(lambda: len(goals)>before_goals+1))
        # Hold /clock and all acquisitions, but leave the gate heartbeats alive.
        before=time.monotonic()
        check('clock pause stops motion despite live heartbeats', wait_for(
            lambda: statuses[-1][1]['state']=='stale' and not permits[-1][1] and len(cancelled)>=3, 1.5))
        check('heartbeats continued during pause', statuses[-1][0]>before+.5)
        for _ in range(5):
            send_frame()
        patrol.publish(Empty()); check('new task after clock-pause recovery', wait_for(lambda: len(goals)>before_goals+2))
        # Stop the gate while runtime, action server and velocity guard remain alive.
        os.killpg(gate.pid, signal.SIGINT); gate.wait(timeout=5)
        children[0]=('gate-stopped',gate)
        before=time.monotonic()
        check('gate death heartbeat expiry cancels motion', wait_for(lambda: not permits[-1][1] and len(cancelled)>=4, 1))
        checks.append({'check':'gate_death_stop_seconds','value':time.monotonic()-before})
        check('safe output zero after gate death', safe[-1][1]==0)
    finally:
        release.set(); terminate.set()
        for label, child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try: child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL); child.wait(timeout=2)
        with (a.output/'observed-messages.json').open('x') as f:
            json.dump({'statuses':statuses, 'runtime':rows, 'permits':permits, 'safe_velocity_x':safe,
                       'poses':[(t,message_to_ordereddict(m)) for t,m in poses],
                       'tf':[(t,message_to_ordereddict(m)) for t,m in transforms],
                       **({'infrastructure_acks': infrastructure_acks,
                           'infrastructure_discovery': discovery_observations} if a.consumer_diagnostics else {})}, f, indent=2)
        with (a.output/'result.json').open('x') as f:
            json.dump({'checks':checks,'requests':requests,'processed':processed(),
                       'goals':goals,'cancelled':cancelled,'status_messages':len(statuses),
                       'approved_pose_count':len(poses),'approved_tf_count':len(transforms),
                       'children_stopped':all(p.poll() is not None for _,p in children),
                       'limitation':'synthetic ROS integration; fake detector/action server, no real Nav2 or hardware'},f,indent=2)
        for f in files: f.close()
        server.destroy(); executor.shutdown(timeout_sec=2); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
