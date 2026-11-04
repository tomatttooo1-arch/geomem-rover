#!/usr/bin/env python3
"""ROS 2 node: Nav2's global map while mapping (2026-10-29; decisions in host/nav_global_map.py).

    python3 -m host.ros_nav_global_map [--start-map-yaml Y] [--record F] [--rolling-topic T]

Topics:
  in   /map                     nav_msgs/OccupancyGrid  map_server (transient local): the base
       /fpga/rtabmap/map         nav_msgs/OccupancyGrid  rtabmap's live grid (transient local)
       /fpga/rolling_map/grid    nav_msgs/OccupancyGrid  the rolling local layer (transient local; --rolling-topic, ""
                                                        = off), brought into the map frame by TF map <- its frame
       /fpga/mapping/status      std_msgs/String JSON    mapping_ctl (state, session; transient local, 1 s)
       /fpga/mapping/map_align   std_msgs/String JSON    mapping_ctl's alignment (swept places into the map frame)
       /odom                    nav_msgs/Odometry       the car base in the odometry frame (swept places)
       /rosout                  rcl_interfaces/Log      planner_server's failed-plan warnings
  out  /fpga/nav/global_map      nav_msgs/OccupancyGrid  Nav2's global static_layer (transient local, reliable, 1)
       /fpga/nav/global_map/status std_msgs/String JSON  mode / reason / counts on every publish (transient local)
--start-map-yaml: the yaml map_server starts with (car_inner MAP_YAML); published at once, so the global costmap has
its map before Nav2 is activated (car_inner waits for the "published" line). Every event is one JSON line on stdout
(car_inner: nav_global_map.log) and, with --record, in that file. No exception ends the node (car_inner restarts it
if it ever exits). Python 3.8 compatible.
"""
import argparse
import array
import json
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from host import nav_global_map as ngm  # noqa: E402

PLANNER_LOGGER = "planner_server"
WARN_LEVEL = 30


def planner_failure_text(name, level, text):
    """True for a planner_server warning / error that says a plan could not be made (Humble NavFn
    "GridBased: failed to create plan with tolerance 0.00.", planner_server "Planning algorithm GridBased failed to
    generate a valid path to (x, y)", "... goal ... off the global costmap ...")."""
    if PLANNER_LOGGER not in (name or "") or level < WARN_LEVEL:
        return False
    t = (text or "").lower()
    return ("fail" in t and ("plan" in t or "path" in t)) or "off the global costmap" in t


def grid_from_msg(msg):
    q = msg.info.origin.orientation
    if abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or abs(q.z * q.z + q.w * q.w - 1) > 1e-5:
        raise ValueError("nonplanar/invalid map origin")
    import numpy as np
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return ngm.Grid(data, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y,
                    2 * math.atan2(q.z, q.w), msg.header.frame_id)


def grid_from_yaml(path):
    sys.path.insert(0, os.path.join(ROOT, "tools", "car_run"))
    import bench_pose
    import numpy as np
    w, h, res, origin, data = bench_pose.load_map_yaml(path)
    return ngm.Grid(np.asarray(data, dtype=np.int8).reshape(h, w), res, origin[0], origin[1], origin[2], ngm.FRAME)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start-map-yaml", default="")
    ap.add_argument("--record", default="")
    ap.add_argument("--out-topic", default=ngm.OUT_TOPIC)
    ap.add_argument("--live-topic", default=ngm.LIVE_TOPIC)
    ap.add_argument("--rolling-topic", default=ngm.ROLLING_TOPIC,
                    help='the local layer\'s grid whose OCCUPIED cells go in too ("" = off, the map of before)')
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--clear-m", type=float, default=ngm.SWEPT_CLEAR_M)
    ap.add_argument("--fallback-s", type=float, default=ngm.PLANNER_FALLBACK_S)
    ap.add_argument("--min-neighbours", type=int, default=ngm.MIN_NEIGHBOURS)
    ap.add_argument("--tick-s", type=float, default=1.0)
    a = ap.parse_args(argv)

    rec = open(a.record, "a", buffering=1) if a.record else None

    def record(event, **fields):
        row = {"time": time.time(), "event": event}
        row.update(fields)
        try:
            line = json.dumps(row, default=str)
        except (TypeError, ValueError):
            line = json.dumps({"time": row["time"], "event": event, "unserialisable": repr(fields)})
        print(line, flush=True)
        if rec is not None:
            try:
                rec.write(line + "\n")
            except OSError:
                pass

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rcl_interfaces.msg import Log
    from std_msgs.msg import String

    latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    core = ngm.GlobalMapCore(record=record, fallback_s=a.fallback_s, clear_m=a.clear_m,
                             min_neighbours=a.min_neighbours)

    rclpy.init()
    node = Node("fpga_nav_global_map")
    pub = node.create_publisher(OccupancyGrid, a.out_topic, latched)
    status_pub = node.create_publisher(String, ngm.STATUS_OUT_TOPIC, latched)

    def guarded(name, fn):
        def run(*args):
            try:
                return fn(*args)
            except Exception as exc:                  # noqa: BLE001 - log, keep the node (module doc)
                record("callback_failed", callback=name, reason=repr(exc))
        return run

    def publish(grid):
        msg = OccupancyGrid()
        stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = grid.frame
        msg.header.stamp = stamp
        msg.info.map_load_time = stamp
        msg.info.resolution = float(grid.resolution)
        msg.info.height, msg.info.width = int(grid.shape[0]), int(grid.shape[1])
        msg.info.origin.position.x, msg.info.origin.position.y = grid.origin_x, grid.origin_y
        msg.info.origin.orientation.z = math.sin(grid.origin_yaw / 2)
        msg.info.origin.orientation.w = math.cos(grid.origin_yaw / 2)
        msg.data = array.array("b", grid.data.tobytes())
        pub.publish(msg)
        st = core.status(time.monotonic())
        status_pub.publish(String(data=json.dumps(st, default=str)))
        record("published", **st)

    def tick():
        if a.rolling_topic:
            rolling_tf()
        out = core.tick(time.monotonic())
        if out is not None:
            publish(out)

    def on_base(msg):
        core.on_base(grid_from_msg(msg), "map_server")
        tick()                                        # the export / start map goes on at once

    def on_live(msg):
        try:
            grid = grid_from_msg(msg)
        except ValueError as exc:
            if core.live_problem != str(exc):
                core.live_problem = str(exc)
                record("live_grid_ignored", topic=a.live_topic, reason=str(exc))
            return
        core.on_live(grid, time.monotonic())

    def on_status(msg):
        core.on_status(json.loads(msg.data), time.monotonic())

    def on_align(msg):
        core.on_align(json.loads(msg.data))

    def on_rolling(msg):
        core.on_rolling(grid_from_msg(msg))

    tf_state = {"buffer": None, "listener": None, "warned": False}

    def rolling_tf():
        """map <- the rolling grid's frame from TF (the transform Nav2 moves its plan with), into the core."""
        if core.rolling is None:
            return
        if tf_state["buffer"] is None:
            import tf2_ros
            tf_state["buffer"] = tf2_ros.Buffer()
            tf_state["listener"] = tf2_ros.TransformListener(tf_state["buffer"], node)
        try:
            import rclpy.time
            tf = tf_state["buffer"].lookup_transform(ngm.FRAME, core.rolling.frame, rclpy.time.Time())
        except Exception as exc:                      # noqa: BLE001 - no transform: the cells stay out
            if not tf_state["warned"]:
                tf_state["warned"] = True
                record("rolling_tf_missing", frame=core.rolling.frame, reason=str(exc)[:200])
            core.on_rolling_tf(None)
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        core.on_rolling_tf((t.x, t.y, yaw))

    def on_odom(msg):
        p = msg.pose.pose.position
        core.on_odom(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec, p.x, p.y)

    def on_rosout(msg):
        if planner_failure_text(msg.name, msg.level, msg.msg):
            if core.on_planner_failure(time.monotonic(), msg.msg):
                tick()

    record("start", out_topic=a.out_topic, live_topic=a.live_topic, rolling_topic=a.rolling_topic or None,
           start_map_yaml=a.start_map_yaml or None,
           clear_m=a.clear_m, fallback_s=a.fallback_s, min_neighbours=a.min_neighbours)
    if a.start_map_yaml:
        try:
            core.on_base(grid_from_yaml(a.start_map_yaml), "start_map_yaml")
            tick()
        except Exception as exc:                      # noqa: BLE001 - map_server's /map will bring it
            record("start_map_failed", yaml=a.start_map_yaml, reason=repr(exc), level="warn")
    node.create_subscription(OccupancyGrid, ngm.BASE_TOPIC, guarded("base", on_base), latched)
    node.create_subscription(OccupancyGrid, a.live_topic, guarded("live", on_live), latched)
    node.create_subscription(String, ngm.MAPPING_STATUS_TOPIC, guarded("status", on_status), latched)
    if a.rolling_topic:
        node.create_subscription(OccupancyGrid, a.rolling_topic, guarded("rolling", on_rolling), latched)
    from host import map_align
    node.create_subscription(String, map_align.TOPIC, guarded("align", on_align), latched)
    node.create_subscription(Odometry, a.odom_topic, guarded("odom", on_odom), qos_profile_sensor_data)
    node.create_subscription(Log, "/rosout", guarded("rosout", on_rosout), QoSProfile(
        depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
    node.create_timer(a.tick_s, guarded("tick", tick))
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.5)
            except KeyboardInterrupt:
                raise
            except Exception as exc:                  # noqa: BLE001
                record("spin_failed", reason=repr(exc))
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        record("stop", publishes=core.publishes)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:                             # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
