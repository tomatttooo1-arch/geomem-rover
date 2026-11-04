"""CTRLPS2 mock run: unicycle chassis stand-in. /fpga/cmd_vel_safe (the velocity guard's output) -> integrated at
50 Hz -> TF odom->base_link + /odom; publishes /fpga/motion_permit True at 10 Hz and an EMPTY /fpga/points at 5 Hz
(the sim global costmap's observation source: without it the costmap is never "current" and planner_server waits
forever -- no camera in this run). Records its pose to --out (argv[1]) every argv[2] ticks of 50 Hz (default 10;
CTRLPS3's real-core run logs every tick for the >= 253 check)."""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros


class Chassis(Node):
    def __init__(self, out, every=10):
        super().__init__("mock_chassis")
        self.x = self.y = self.th = 0.0
        self.v = self.w = 0.0
        self.t_cmd = 0.0
        self.n_cmd = 0
        self.out = open(out, "w")
        self.tfb = tf2_ros.TransformBroadcaster(self)
        self.odom = self.create_publisher(Odometry, "/odom", 10)
        self.permit = self.create_publisher(Bool, "/fpga/motion_permit", 1)
        self.create_subscription(Twist, "/fpga/cmd_vel_safe", self.on_cmd, 10)
        self.create_timer(0.02, self.tick)
        self.create_timer(0.1, lambda: self.permit.publish(Bool(data=True)))
        self.points = self.create_publisher(PointCloud2, "/fpga/points", 1)
        self.create_timer(0.2, self.empty_cloud)
        self.k = 0
        self.every = max(1, int(every))

    def empty_cloud(self):
        c = PointCloud2()
        c.header.stamp, c.header.frame_id = self.get_clock().now().to_msg(), "base_link"
        c.height, c.width, c.point_step, c.row_step, c.is_dense = 1, 0, 12, 0, True
        c.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate("xyz")]
        self.points.publish(c)

    def on_cmd(self, m):
        self.v, self.w, self.t_cmd = m.linear.x, m.angular.z, time.monotonic()
        self.n_cmd += 1

    def tick(self):
        dt = 0.02
        if time.monotonic() - self.t_cmd > 0.2:      # like the MCU: 200 ms without a command -> stop
            self.v = self.w = 0.0
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        self.th = math.atan2(math.sin(self.th + self.w * dt), math.cos(self.th + self.w * dt))
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.tfb.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = t.transform.rotation
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom.publish(o)
        self.k += 1
        if self.k % self.every == 0:
            self.out.write(json.dumps({"t": time.time(), "x": round(self.x, 4), "y": round(self.y, 4),
                                       "th": round(self.th, 4), "v": self.v, "w": self.w, "n_cmd": self.n_cmd}) + "\n")
            self.out.flush()


rclpy.init()
n = Chassis(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 10)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
