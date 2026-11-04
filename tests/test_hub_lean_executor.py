"""Hub lean executor (host/hub/hub_executor.py) against rclpy humble's SingleThreadedExecutor.

Needs rclpy (skipped on a host without ROS); run inside the humble image, e.g.
    docker run --rm --network none -v $W:/w -w /w --entrypoint bash fpga-dev1-sim:humble -c \
      'source /opt/ros/humble/setup.bash; ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=91 PYTHONPATH=.:$PYTHONPATH \
       python3 -m unittest discover -s tests -p test_hub_lean_executor.py -v'

Known answer: the same publish script (a reliable depth-10 topic, a depth-1 topic and a
TRANSIENT_LOCAL topic published before the subscriber exists) is received identically, message by
message and in per-topic order, by the lean loop and by the stock executor. Positive controls: a
node with a timer, or with the QoS default event handlers, is refused at spin start.
"""

import threading
import time
import unittest

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.qos_event import SubscriptionEventCallbacks
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid, Path
    from geometry_msgs.msg import PoseStamped
    HAVE_RCLPY = True
except ImportError:                      # host without ROS
    HAVE_RCLPY = False

if HAVE_RCLPY:
    from host.hub.hub_executor import LeanExecutor

NO_EV = None
GRID_VALUES = [-1, 0, 100, 50, -1, 100, 0, 1] * 800            # 80 x 80 int8, incl. unknown (-1)
PATH_POSES = 200


def grid_msg(i):
    m = OccupancyGrid()
    m.header.frame_id = "odom%d" % i
    m.info.width, m.info.height, m.info.resolution = 80, 80, 0.05
    m.data = [v if k % 97 else (i % 100) for k, v in enumerate(GRID_VALUES)]
    return m


def path_msg(i):
    m = Path()
    m.header.frame_id = "map"
    for k in range(PATH_POSES):
        p = PoseStamped()
        p.pose.position.x, p.pose.position.y = 0.05 * k + i, -0.01 * k
        m.poses.append(p)
    return m


def no_ev():
    return SubscriptionEventCallbacks(use_default_callbacks=False)


@unittest.skipUnless(HAVE_RCLPY, "rclpy not available (run in the humble image)")
class LeanExecutorTests(unittest.TestCase):
    def setUp(self):
        self.ctx = Context()
        rclpy.init(context=self.ctx)

    def tearDown(self):
        if self.ctx.ok():
            rclpy.shutdown(context=self.ctx)

    def run_script(self, executor_cls, tag):
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        pub_node = Node("pub_" + tag, context=self.ctx, start_parameter_services=False)
        pa = pub_node.create_publisher(String, "/lean_test/a_" + tag, 10)
        pd = pub_node.create_publisher(String, "/lean_test/d1_" + tag, 1)
        pl = pub_node.create_publisher(String, "/lean_test/latched_" + tag, latched)
        pl.publish(String(data="latched-0"))
        pg = pub_node.create_publisher(OccupancyGrid, "/lean_test/grid_" + tag, 1)
        pp = pub_node.create_publisher(Path, "/lean_test/plan_" + tag, 1)
        got = []
        lock = threading.Lock()

        def cb(name):
            def f(msg):
                with lock:
                    got.append((name, msg.data))
            return f

        sub_node = Node("sub_" + tag, context=self.ctx, start_parameter_services=False)
        sub_node.create_subscription(String, "/lean_test/a_" + tag, cb("a"), 10, event_callbacks=no_ev())
        sub_node.create_subscription(String, "/lean_test/d1_" + tag, cb("d1"), 1, event_callbacks=no_ev())
        sub_node.create_subscription(String, "/lean_test/latched_" + tag, cb("latched"), latched,
                                     event_callbacks=no_ev())
        sub_node.create_subscription(OccupancyGrid, "/lean_test/grid_" + tag,
                                     lambda m: cb("grid")(String(data=m.header.frame_id + ":" + bytes(
                                         m.data.tobytes()).hex())), 1, event_callbacks=no_ev())
        sub_node.create_subscription(Path, "/lean_test/plan_" + tag,
                                     lambda m: cb("plan")(String(data=repr([(p.pose.position.x, p.pose.position.y)
                                                                            for p in m.poses]))), 1,
                                     event_callbacks=no_ev())
        ex = executor_cls() if executor_cls is LeanExecutor else executor_cls(context=self.ctx)
        ex.add_node(sub_node)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        deadline = time.monotonic() + 10
        while (pa.get_subscription_count() < 1 or pd.get_subscription_count() < 1
               or pg.get_subscription_count() < 1 or pp.get_subscription_count() < 1):
            self.assertLess(time.monotonic(), deadline, "discovery")
            time.sleep(0.05)
        time.sleep(0.3)
        for i in range(60):
            pa.publish(String(data="a-%d" % i))
            if i % 3 == 0:
                pd.publish(String(data="d1-%d" % i))
            if i % 20 == 5:
                pg.publish(grid_msg(i))
                pp.publish(path_msg(i))
            time.sleep(0.01)
        time.sleep(0.5)
        t0 = time.monotonic()
        self.assertTrue(ex.shutdown(timeout_sec=2.0))
        th.join(timeout=2.0)
        self.assertFalse(th.is_alive())
        self.assertLess(time.monotonic() - t0, 1.5)
        sub_node.destroy_node()                    # must not hang on handles the loop held
        pub_node.destroy_node()
        with lock:
            return list(got)

    def test_same_messages_and_order_as_stock_executor(self):
        lean = self.run_script(LeanExecutor, "lean")
        stock = self.run_script(SingleThreadedExecutor, "stock")
        per = lambda rows, k: [d for n, d in rows if n == k]    # noqa: E731
        expect_a = ["a-%d" % i for i in range(60)]
        expect_d = ["d1-%d" % i for i in range(0, 60, 3)]
        for rows in (lean, stock):
            self.assertEqual(per(rows, "a"), expect_a)
            self.assertEqual(per(rows, "d1"), expect_d)
            self.assertEqual(per(rows, "latched"), ["latched-0"])
        self.assertEqual(sorted(lean), sorted(stock))
        # big / binary messages byte for byte: OccupancyGrid int8 data, Path of 200 poses
        import array
        expect_g = ["odom%d:" % i + array.array("b", [v if k % 97 else (i % 100) for k, v in enumerate(GRID_VALUES)]).tobytes().hex()
                    for i in range(5, 60, 20)]
        expect_p = [repr([(0.05 * k + i, -0.01 * k) for k in range(PATH_POSES)]) for i in range(5, 60, 20)]
        for rows in (lean, stock):
            self.assertEqual(per(rows, "grid"), expect_g)
            self.assertEqual(per(rows, "plan"), expect_p)

    def test_refuses_timer_positive_control(self):
        node = Node("timer_node", context=self.ctx, start_parameter_services=False)
        node.create_timer(1.0, lambda: None)
        ex = LeanExecutor()
        with self.assertRaises(SystemExit) as cm:
            ex.add_node(node)                               # refused in the caller's thread
        self.assertIn("timers=1", str(cm.exception))
        node.destroy_node()

    def test_refuses_default_event_handlers_and_parameter_services(self):
        node = Node("ev_node", context=self.ctx)            # parameter services on (default)
        node.create_subscription(String, "/lean_test/ev", lambda m: None, 10)   # default QoS events
        ex = LeanExecutor()
        with self.assertRaises(SystemExit) as cm:
            ex.add_node(node)
        self.assertIn("services=", str(cm.exception))
        self.assertIn("waitables=", str(cm.exception))
        node.destroy_node()

    def test_hub_node_passes_and_context_shutdown_ends_spin(self):
        from host.hub import hub_ros
        from host.hub.hub_core import HubCore
        core = HubCore("live")
        server = type("S", (), {"core": core, "lock": threading.RLock()})()
        orig_init = hub_ros.ros_modules().Node.__init__
        m = hub_ros.ros_modules()

        class CtxNode(m.Node):                              # the hub node on this test's context
            def __init__(self, name, **kw):
                orig_init(self, name, context=self_ctx, **kw)

        self_ctx = self.ctx
        m.Node = CtxNode
        node = hub_ros.build_node(server, m, preview=True)
        ex = LeanExecutor()
        ex.add_node(node)                                   # no services, timers, waitables
        tolerated = LeanExecutor.tolerated_waitables(node)
        self.assertEqual(len(tolerated), 1)                 # only /parameter_events' handler
        self.assertEqual(len(list(node.waitables)), 1)
        self.assertEqual(len(list(node.subscriptions)), len(hub_ros.SUBSCRIPTIONS))
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        time.sleep(0.3)
        rclpy.shutdown(context=self.ctx)                    # external shutdown wakes the loop
        th.join(timeout=2.0)
        self.assertFalse(th.is_alive())
        node.destroy_node()


if __name__ == "__main__":
    unittest.main()
