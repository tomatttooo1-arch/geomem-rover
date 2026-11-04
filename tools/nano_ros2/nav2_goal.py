#!/usr/bin/env python3
"""Send one NavigateToPose goal and log what Nav2 does (plan, cmd_vel, feedback, result).

Writes a JSON summary to --out. Exit code 0 only if the goal SUCCEEDED.
"""
import argparse
import json
import math
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--x', type=float, required=True)
    p.add_argument('--y', type=float, required=True)
    p.add_argument('--yaw', type=float, default=0.)
    p.add_argument('--timeout', type=float, default=120.)
    p.add_argument('--server-timeout', type=float, default=120.)
    p.add_argument('--out', required=True)
    a = p.parse_args()
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Path, Odometry
    from geometry_msgs.msg import Twist
    rclpy.init(args=[])
    node = rclpy.create_node('nav2_goal_probe')
    s = {'plans': 0, 'plan_len_first': None, 'plan_len_m_first': None, 'cmd_vel': 0, 'cmd_vel_nonzero': 0,
         'cmd_vel_nav': 0, 'first_cmd_vel_s': None, 'max_vx': 0., 'max_abs_wz': 0., 'feedback': 0,
         'last_distance_remaining': None, 'odom_last': None}
    t_goal = [None]

    def on_plan(m):
        s['plans'] += 1
        if s['plan_len_first'] is None:
            pts = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
            s['plan_len_first'] = len(pts)
            s['plan_len_m_first'] = sum(math.dist(u, v) for u, v in zip(pts, pts[1:]))

    def on_cmd(m, key='cmd_vel'):
        s[key] += 1
        if key == 'cmd_vel':
            if abs(m.linear.x) > 1e-4 or abs(m.angular.z) > 1e-4:
                s['cmd_vel_nonzero'] += 1
                if s['first_cmd_vel_s'] is None and t_goal[0]:
                    s['first_cmd_vel_s'] = time.monotonic() - t_goal[0]
            s['max_vx'] = max(s['max_vx'], m.linear.x)
            s['max_abs_wz'] = max(s['max_abs_wz'], abs(m.angular.z))

    def on_odom(m):
        s['odom_last'] = [m.pose.pose.position.x, m.pose.pose.position.y]
    node.create_subscription(Path, 'plan', on_plan, 10)
    node.create_subscription(Twist, 'cmd_vel', on_cmd, 50)
    node.create_subscription(Twist, 'cmd_vel_nav', lambda m: on_cmd(m, 'cmd_vel_nav'), 50)
    node.create_subscription(Odometry, 'odom', on_odom, 10)
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    t0 = time.monotonic()
    ready = client.wait_for_server(timeout_sec=a.server_timeout)
    s['server_wait_s'] = time.monotonic() - t0
    result = {'status': None}
    if ready:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = a.x, a.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(a.yaw / 2), math.cos(a.yaw / 2)

        def on_fb(fb):
            s['feedback'] += 1
            s['last_distance_remaining'] = fb.feedback.distance_remaining
        # bt_navigator rejects goals until the lifecycle manager has activated it: retry.
        s['goal_attempts'] = 0
        while True:
            s['goal_attempts'] += 1
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            t_goal[0] = time.monotonic()
            fut = client.send_goal_async(goal, feedback_callback=on_fb)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=10.)
            handle = fut.result()
            if (handle and handle.accepted) or time.monotonic() - t0 > a.server_timeout:
                break
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        s['accepted_after_s'] = time.monotonic() - t0
        s['goal_accepted'] = bool(handle and handle.accepted)
        if s['goal_accepted']:
            rf = handle.get_result_async()
            rclpy.spin_until_future_complete(node, rf, timeout_sec=a.timeout)
            if rf.done():
                result['status'] = rf.result().status
            else:
                handle.cancel_goal_async()
                result['status'] = 'timeout'
        s['goal_elapsed_s'] = time.monotonic() - t_goal[0]
    # STATUS_SUCCEEDED = 4, ABORTED = 6, CANCELED = 5
    s.update(server_ready=ready, result_status=result['status'], goal=[a.x, a.y, a.yaw])
    with open(a.out, 'x') as f:
        json.dump(s, f, indent=1)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['status'] == 4 else 1


if __name__ == '__main__':
    raise SystemExit(main())
