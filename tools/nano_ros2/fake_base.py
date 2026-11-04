#!/usr/bin/env python3
"""Virtual differential base for Nav2 software smoke tests (no hardware).

Integrates /cmd_vel (unicycle, clamped) and publishes /odom plus TF odom->base_link
at --rate Hz. Every received command is logged to --log (JSON lines). Start pose is
given in the odom frame. This is not a vehicle model; it only closes the software loop.
"""
import argparse
import json
import math
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--x', type=float, default=0.)
    p.add_argument('--y', type=float, default=0.)
    p.add_argument('--yaw', type=float, default=0.)
    p.add_argument('--rate', type=float, default=50.)
    p.add_argument('--log', required=True)
    a = p.parse_args()
    import rclpy
    from geometry_msgs.msg import Twist, TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformBroadcaster
    rclpy.init(args=[])
    node = rclpy.create_node('fake_base')
    odom_pub = node.create_publisher(Odometry, 'odom', 10)
    tfb = TransformBroadcaster(node)
    state = {'x': a.x, 'y': a.y, 'th': a.yaw, 'v': 0., 'w': 0., 't': time.monotonic()}
    log = open(a.log, 'x', buffering=1)

    def on_cmd(m):
        state['v'] = max(-0.5, min(0.5, m.linear.x))
        state['w'] = max(-1.5, min(1.5, m.angular.z))
        log.write(json.dumps({'wall': time.time(), 'vx': m.linear.x, 'wz': m.angular.z,
                              'x': state['x'], 'y': state['y'], 'th': state['th']}) + '\n')
    node.create_subscription(Twist, 'cmd_vel', on_cmd, 10)

    def tick():
        now = time.monotonic()
        dt, state['t'] = now - state['t'], now
        state['th'] += state['w'] * dt
        state['x'] += state['v'] * math.cos(state['th']) * dt
        state['y'] += state['v'] * math.sin(state['th']) * dt
        stamp = node.get_clock().now().to_msg()
        qz, qw = math.sin(state['th'] / 2), math.cos(state['th'] / 2)
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y = state['x'], state['y']
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        tfb.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = state['x'], state['y']
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = state['v'], state['w']
        odom_pub.publish(o)
    node.create_timer(1. / a.rate, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        log.close()


if __name__ == '__main__':
    main()
