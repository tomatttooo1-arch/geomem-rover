#!/usr/bin/env python3
"""Server-side command expiry catches AGX/network death, using wall time.

Optional e-stop input (line D1, 2026-10-14; ROS parameter ``estop_topic``, empty = off and the
behaviour is unchanged): a std_msgs/Bool topic (robot_sim/fpga_sim/scripts/sim_estop_gate.py
publishes the KV260 e-stop latch on it). When set, a command passes only while the newest
e-stop sample is False and at most 0.5 s old (fail-closed: no sample, a stale one or True ->
zero velocity) -- the motion permit had the same 0.5 s until 10-26 (now PERMIT_FRESH_S 2 s).

Optional output topic (teleop/Nav2 arbitration, 2026-10-18; ROS parameter ``output_topic``, default
``/fpga/cmd_vel_safe`` = the behaviour is unchanged): the arbitration mode (car_up CAR_TELEOP=1 with
--nav nav2) sets ``/fpga/nav/cmd_vel_safe``, so host/ros_teleop_safe.py stays the only publisher of
/fpga/cmd_vel_safe and passes the guard's output through while the remote control is not in hand
(docs/design/TELEOP_NAV_ARB_20261018.md).
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool

PERMIT_FRESH_S = 2.0   # 10-26 owner：重成功率，放宽自 0.5 s（运行器许可的新鲜度窗）


class VelocityGuard(Node):
    def __init__(self):
        super().__init__("fpga_velocity_guard")
        self.last_command = None
        self.last_permit = None
        self.permitted = False
        self.stopped = False
        self.estop_topic = self.declare_parameter("estop_topic", "").value
        self.estop = None            # (latched, monotonic receipt) of the newest sample
        self.output_topic = self.declare_parameter("output_topic", "/fpga/cmd_vel_safe").value or "/fpga/cmd_vel_safe"
        self.output = self.create_publisher(Twist, self.output_topic, 1)
        if self.output_topic != "/fpga/cmd_vel_safe":
            self.get_logger().info("output on %s (teleop arbitration)" % self.output_topic)
        self.create_subscription(Twist, "/cmd_vel", self.command, 1)
        self.create_subscription(Bool, "/fpga/motion_permit", self.permit, 1)
        if self.estop_topic:
            self.create_subscription(Bool, self.estop_topic, self.on_estop, 1)
            self.get_logger().info("e-stop input on %s (fail-closed, 0.5 s)" % self.estop_topic)
        # This node deliberately retains wall clock when simulator is paused.
        self.create_timer(0.1, self.tick)

    def command(self, msg):
        self.last_command = time.monotonic()
        if self.permit_is_fresh():
            self.stopped = False
            self.output.publish(msg)
        else:
            self.output.publish(Twist())

    def permit_is_fresh(self):
        # 10-26 owner：重成功率，放宽自 0.5 s（许可每处理完一帧才发一次，Nano 高负载时处理流会停顿；
        # /cmd_vel 新鲜度 0.5 s 与急停 0.5 s 不动）
        return (self.permitted and self.last_permit is not None
                and time.monotonic() - self.last_permit <= PERMIT_FRESH_S and self.estop_clear())

    def estop_clear(self):
        if not self.estop_topic:
            return True
        return (self.estop is not None and not self.estop[0]
                and time.monotonic() - self.estop[1] <= 0.5)

    def on_estop(self, msg):
        self.estop = (bool(msg.data), time.monotonic())
        if msg.data:
            self.output.publish(Twist())

    def permit(self, msg):
        self.last_permit = time.monotonic()
        self.permitted = bool(msg.data)
        if not self.permitted:
            self.output.publish(Twist())

    def tick(self):
        if (not self.permit_is_fresh() or self.last_command is None
                or time.monotonic() - self.last_command > 0.5):
            self.output.publish(Twist())
            if not self.stopped:
                self.get_logger().info("Command or motion permit expired: zero velocity published")
                self.stopped = True


def main():
    rclpy.init()
    node = VelocityGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
