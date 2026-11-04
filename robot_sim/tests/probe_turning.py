#!/usr/bin/env python3
"""Actuator calibration only; run with Nav2 and task control stopped."""
import json
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from probe_ros import Probe


def sample(node):
    msg = node.latest["odom"]
    q = msg.pose.pose.orientation
    return {"stamp": msg.header.stamp.sec + msg.header.stamp.nanosec/1e9,
            "x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y,
            "yaw": math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
            "linear_x": msg.twist.twist.linear.x, "angular_z": msg.twist.twist.angular.z}


def command(node, omega, duration):
    end = time.monotonic() + duration
    next_pub = 0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        if time.monotonic() >= next_pub:
            permit=Bool(); permit.data=True; node.permit_pub.publish(permit)
            twist=Twist(); twist.angular.z=omega; node.command_pub.publish(twist)
            next_pub=time.monotonic()+.08


def main():
    rclpy.init(); node=Probe(); rows=[]
    try:
        node.spin_for(1.0)
        if "odom" not in node.latest:
            raise RuntimeError("No odometry")
        for omega in [0.0, 0.031578947368420957, 0.4, -0.4]:
            before=sample(node); command(node, omega, 3.0); after=sample(node)
            change=math.atan2(math.sin(after["yaw"]-before["yaw"]), math.cos(after["yaw"]-before["yaw"]))
            rows.append({"command_rad_s":omega,"before":before,"after":after,"yaw_change":change})
            command(node,0.0,.5)
    finally:
        node.spin_for(.3,command=0.0,permit=False)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps({"probe":"angular_command_calibration","rows":rows},indent=2))


if __name__=="__main__":
    main()
