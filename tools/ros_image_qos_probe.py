"""Bounded 2x2 RGB size/reliability transport probe, fresh receiver per cell."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    p.add_argument('--receiver',action='store_true')
    p.add_argument('--reliable',action='store_true')
    p.add_argument('--large',action='store_true')
    a=p.parse_args()
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    rclpy.init(args=[]);node=rclpy.create_node('image_qos_receiver' if a.receiver else 'image_qos_publisher')
    if a.receiver:
        rows=[]
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE if a.reliable else ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Image,'/qos_probe/rgb',lambda m:rows.append({'stamp':m.header.stamp.sec,
            'bytes':len(m.data),'monotonic_ns':time.monotonic_ns()}),qos)
        try:
            until=time.monotonic()+15
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        except KeyboardInterrupt:pass
        finally:
            with a.output.open('x') as f:json.dump(rows,f,indent=2)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        return
    a.output.mkdir(exist_ok=False)
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE)
    pub=node.create_publisher(Image,'/qos_probe/rgb',qos)
    child=None;sent=[]
    try:
        command=[sys.executable,'-m','tools.ros_image_qos_probe',str(a.output/'received.json'),'--receiver']
        if a.reliable:command+=['--reliable']
        with (a.output/'receiver.log').open('x') as log:
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+5
            while not pub.get_subscription_count() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            if not pub.get_subscription_count():raise RuntimeError('receiver discovery timeout')
            msg=Image();msg.header.frame_id='camera_optical_frame'
            msg.width,msg.height=(640,480) if a.large else (20,20)
            msg.encoding='rgb8';msg.step=msg.width*3;msg.data=bytes(msg.step*msg.height)
            for i in range(20):
                msg.header.stamp.sec=i+1
                pub.publish(msg);sent.append({'stamp':i+1,'monotonic_ns':time.monotonic_ns()})
                deadline=time.monotonic()+.2
                while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            os.killpg(child.pid,signal.SIGINT);child.wait(timeout=5)
    finally:
        if child and child.poll() is None:
            os.killpg(child.pid,signal.SIGKILL);child.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        received=json.loads((a.output/'received.json').read_text()) if (a.output/'received.json').exists() else []
        summary={'reliable':a.reliable,'large':a.large,'published':sent,'received':received,
                 'received_count':len(received),'child_exit_code':child.returncode if child else None,
                 'protocol':'20 synthetic RGB messages at <=5Hz; fresh separate receiver process; publisher reliable depth10'}
        with (a.output/'result.json').open('x') as f:json.dump(summary,f,indent=2)
        print(json.dumps({k:v for k,v in summary.items() if k not in ('published','received')}))


if __name__=='__main__':main()
