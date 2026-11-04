"""CPU ROS/Nav2 kinematic smoke in an isolated container; record actual planner actions."""
import argparse
from collections import Counter
import gc
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np


def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid,Odometry,Path as NavPath
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from std_msgs.msg import String
    from lifecycle_msgs.srv import ChangeState
    import rosbag2_py
    import yaml
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',required=True);a=ap.parse_args()
    out=Path(a.out)
    if out.exists():ap.error('output must not exist')
    out.mkdir(parents=True)
    # Use a private network namespace and ROS domain; never call car_up or production services.
    if os.environ.get('ROS_DOMAIN_ID')!='173' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise SystemExit('requires isolated smoke ROS_DOMAIN_ID=173 and ROS_LOCALHOST_ONLY=1')
    with open('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml') as f:params=yaml.safe_load(f)
    params['planner_server']['ros__parameters'].update(use_sim_time=False,expected_planner_frequency=1.0)
    params['planner_server']['ros__parameters']['GridBased'].update(tolerance=.15,allow_unknown=False,use_astar=False)
    common=dict(global_frame='odom',robot_base_frame='base_link',use_sim_time=False,rolling_window=False,
        width=4,height=4,resolution=.05,origin_x=-2.,origin_y=-2.,robot_radius=.25,footprint_padding=0.,
        update_frequency=10.,publish_frequency=10.,track_unknown_space=True,always_send_full_costmap=True,
        plugins=['static_layer','inflation_layer'],
        static_layer=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic='/fpga/rolling_map/grid',
                          map_subscribe_transient_local=True,subscribe_to_updates=False,footprint_clearing_enabled=False),
        inflation_layer=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=.45,cost_scaling_factor=3.0))
    params['global_costmap']['global_costmap']['ros__parameters'].update(common)
    params_path=out/'nav2_params.yaml';params_path.write_text(yaml.safe_dump(params))
    log=(out/'planner_server.log').open('w')
    process=subprocess.Popen(['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',str(params_path)],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('dev2_planner_record_smoke')
    writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(out/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
    topics={'/fpga/rolling_map/grid':('nav_msgs/msg/OccupancyGrid',OccupancyGrid),
            '/global_costmap/costmap_raw':('nav2_msgs/msg/Costmap',Costmap),
            '/tf':('tf2_msgs/msg/TFMessage',TFMessage),'/tf_static':('tf2_msgs/msg/TFMessage',TFMessage),
            '/odom':('nav_msgs/msg/Odometry',Odometry),'/plan':('nav_msgs/msg/Path',NavPath),
            '/dev2/planner_request':('std_msgs/msg/String',String)}
    counts=Counter();subscriptions=[]
    qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
    durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def record(topic,msg):
        counts[topic]+=1;writer.write(topic,serialize_message(msg),node.get_clock().now().nanoseconds)
    for topic,(kind,cls) in topics.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=kind,serialization_format='cdr'))
        subscriptions.append(node.create_subscription(cls,topic,lambda msg,t=topic:record(t,msg),
            durable if topic in ['/tf_static','/fpga/rolling_map/grid'] else qos))
    gridpub=node.create_publisher(OccupancyGrid,'/fpga/rolling_map/grid',durable)
    tfpub=node.create_publisher(TFMessage,'/tf',qos);staticpub=node.create_publisher(TFMessage,'/tf_static',durable)
    odompub=node.create_publisher(Odometry,'/odom',qos);requestpub=node.create_publisher(String,'/dev2/planner_request',qos)
    origin=time.monotonic();ticks=0
    def publish_scene():
        nonlocal ticks
        ticks+=1;now=node.get_clock().now().to_msg();t=time.monotonic()-origin
        x=-.5+.05*math.sin(t*.3);y=.05*math.sin(t*.2)
        tf=TransformStamped();tf.header.stamp=now;tf.header.frame_id='odom';tf.child_frame_id='base_link'
        tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation.w=1.
        tfpub.publish(TFMessage(transforms=[tf]))
        odom=Odometry();odom.header=tf.header;odom.child_frame_id='base_link';odom.pose.pose.position.x=x;odom.pose.pose.position.y=y;odom.pose.pose.orientation.w=1.;odompub.publish(odom)
        if ticks%5==0:
            grid=OccupancyGrid();grid.header.stamp=now;grid.header.frame_id='odom';grid.info.resolution=.05
            grid.info.width=grid.info.height=80;grid.info.origin.position.x=grid.info.origin.position.y=-2.;grid.info.origin.orientation.w=1.
            m=np.zeros((80,80),np.int8);m[0,:]=m[-1,:]=m[:,0]=m[:,-1]=100
            m[36:45,49:54]=100;m[60:66,24:31]=100
            grid.data=m.ravel().tolist();gridpub.publish(grid)
    node.create_timer(.02,publish_scene)
    static=TransformStamped();static.header.frame_id='map';static.child_frame_id='odom';static.transform.rotation.w=1.
    staticpub.publish(TFMessage(transforms=[static]))
    def spin_for(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if process.poll() is not None:raise RuntimeError('planner_server_exited')
            rclpy.spin_once(node,timeout_sec=.02)
    def await_future(future,seconds):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end:spin_for(.02)
        if not future.done():raise RuntimeError('ROS_future_timeout')
        return future.result()
    outcomes=[]
    try:
        change=node.create_client(ChangeState,'/planner_server/change_state')
        deadline=time.monotonic()+20
        while not change.service_is_ready() and time.monotonic()<deadline:spin_for(.1)
        if not change.service_is_ready():raise RuntimeError('planner_lifecycle_unavailable')
        spin_for(.3)
        for transition in [1,3]:
            req=ChangeState.Request();req.transition.id=transition
            result=await_future(change.call_async(req),15)
            if not result.success:raise RuntimeError('planner_transition_failed:%d'%transition)
        spin_for(1.)
        action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
        deadline=time.monotonic()+10
        while not action.server_is_ready() and time.monotonic()<deadline:spin_for(.1)
        targets=[(1.25,.65),(1.25,-.65),(.1,1.15),(-1.2,-1.0),(.6,1.0),(1.15,-1.0),(1.0,.9),(-1.1,.9)]
        for i,(x,y) in enumerate(targets):
            spin_for(.2)
            now=node.get_clock().now();goal=ComputePathToPose.Goal();goal.goal.header.frame_id='odom';goal.goal.header.stamp=now.to_msg()
            goal.goal.pose.position.x=x;goal.goal.pose.position.y=y;goal.goal.pose.orientation.w=1.;goal.planner_id='GridBased';goal.use_start=False
            requestpub.publish(String(data=json.dumps(dict(request_id='sim-action-%d'%i,stamp_ns=now.nanoseconds,frame_id='odom',goal=[x,y]))))
            handle=await_future(action.send_goal_async(goal),5)
            if not handle.accepted:raise RuntimeError('compute_path_goal_rejected')
            result=await_future(handle.get_result_async(),10)
            outcomes.append(dict(request_id='sim-action-%d'%i,status=int(result.status),poses=len(result.result.path.poses)))
        spin_for(.4)
        if counts['/global_costmap/costmap_raw']==0:raise RuntimeError('no_raw_costmap_recorded')
        summary=dict(source_kind='simulation',model='kinematic_pose_and_synthetic_occupancy_with_actual_Nav2_planner_server',
            ros_domain_id=173,network='private_container_no_external_network',counts=dict(counts),outcomes=outcomes)
        (out/'capture.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
    finally:
        node.destroy_node();rclpy.shutdown();del writer;gc.collect()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
        log.close()


if __name__=='__main__':main()
