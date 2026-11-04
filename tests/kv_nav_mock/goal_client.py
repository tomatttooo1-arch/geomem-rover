"""CTRLPS2 mock run: the runner's side -- one NavigateToPose goal to ``navigate_to_pose`` exactly as
host/ros_runtime.py sends it; prints feedback count, the result status and the final map pose (JSON)."""
import json, math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
import tf2_ros

gx, gy, gyaw, timeout = (float(a) for a in sys.argv[1:5])
rclpy.init()
node = Node("mock_goal_client")
tf = tf2_ros.Buffer(); tfl = tf2_ros.TransformListener(tf, node)
counts = {"feedback": 0, "cmd_vel": 0, "cmd_vel_safe": 0, "cmd_vel_nonzero": 0}
node.create_subscription(Twist, "/cmd_vel", lambda m: counts.__setitem__("cmd_vel", counts["cmd_vel"] + 1) or
                         (counts.__setitem__("cmd_vel_nonzero", counts["cmd_vel_nonzero"] + 1) if (m.linear.x or m.angular.z) else None), 10)
node.create_subscription(Twist, "/fpga/cmd_vel_safe", lambda m: counts.__setitem__("cmd_vel_safe", counts["cmd_vel_safe"] + 1), 10)
ac = ActionClient(node, NavigateToPose, "navigate_to_pose")
if not ac.wait_for_server(timeout_sec=30):
    print(json.dumps({"error": "no navigate_to_pose server"})); sys.exit(2)
g = NavigateToPose.Goal()
g.pose.header.frame_id = "map"
g.pose.header.stamp = node.get_clock().now().to_msg()
g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
t0 = time.monotonic()
fut = ac.send_goal_async(g, feedback_callback=lambda f: counts.__setitem__("feedback", counts["feedback"] + 1))
rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
h = fut.result()
if h is None or not h.accepted:
    print(json.dumps({"error": "goal not accepted"})); sys.exit(3)
rf = h.get_result_async()
rclpy.spin_until_future_complete(node, rf, timeout_sec=timeout)
status = rf.result().status if rf.done() else None
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.05)
try:
    t = tf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
    th = math.atan2(2 * t.rotation.w * t.rotation.z, 1 - 2 * t.rotation.z ** 2)
    pose = [round(t.translation.x, 3), round(t.translation.y, 3), round(th, 3)]
    err = [round(math.hypot(pose[0] - gx, pose[1] - gy), 3), round(abs(math.atan2(math.sin(th - gyaw), math.cos(th - gyaw))), 3)]
except Exception as exc:
    pose, err = None, str(exc)
names = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED", GoalStatus.STATUS_CANCELED: "CANCELED"}
print(json.dumps({"status": names.get(status, status), "elapsed_s": round(time.monotonic() - t0, 1), "final_map_pose": pose,
                  "err_xy_yaw": err, "goal": [gx, gy, gyaw], **counts}))
rclpy.shutdown()
