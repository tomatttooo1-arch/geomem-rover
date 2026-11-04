"""rclpy adapter for the Nano hub (spec B.1/B.2). Thin: every callback is one HubCore.feed_*.

Run in the ROS container (same image/domain as the other nodes, --net=host, the existing
ROS_LOCALHOST_ONLY setting unchanged):
    python3 -m host.hub.hub_ros --data-source live --token-file /path/token --kv 198.51.100.2:9400

Never subscribes /fpga/rgb or /fpga/depth (B.1 ban): preview frames come only from the
optional /fpga/preview/jpeg topic published by a process that already holds images.
"""

import argparse
import array
import math
import os
import signal
import sys
import threading
import time
import zlib

from host import fpga_clock
from host.hub import hub_profile
from host.hub.hub_core import HubCore, summarize_plan
from host.hub.hub_server import HubServer, common_args, read_token, run

SUBSCRIPTIONS = (
    "/fpga/status", "/fpga/memory", "/fpga/region_verdicts", "/fpga/kv_link/frame_stats",
    "/fpga/motion_permit", "/cmd_vel", "/fpga/localization_status", "/fpga/localization_pose",
    "/fpga/rolling_map/status", "/fpga/rolling_map/grid", "/map", "/odom",
    "/fpga/odom_adapter_status", "/plan", "/fpga/preview/jpeg", "/fpga/runtime_info", "/fpga/mapping/status",
    "/fpga/map_reset/status", "/fpga/search/belief", "/fpga/geometry_frame", "/fpga/chassis/status",
    "/fpga/wheel_odom/status")
PUBLICATIONS = ("/fpga/query", "/fpga/select_candidate", "/fpga/cancel", "/fpga/goto", "/fpga/teleop/input",
                "/fpga/mapping/stop", "/fpga/map_reset")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_meta(msg, rev):
    o = msg.info.origin
    return {"rev": rev, "width": msg.info.width, "height": msg.info.height,
            "resolution_m": msg.info.resolution,
            "origin": [o.position.x, o.position.y, yaw_of(o.orientation)],
            "frame_id": msg.header.frame_id}


def grid_bytes(msg):
    """int8 cells as their two's-complement bytes (== bytes((v + 256) % 256 for v in data), the
    old per-cell generator: 0.56 ms per 80x80 grid on the host; rclpy gives array('b'))."""
    return array.array("b", msg.data).tobytes()


class RosActions(object):
    def __init__(self, node, String, Empty, pub_kw=None):
        self.String, self.Empty = String, Empty
        kw = pub_kw or {}
        self.query = node.create_publisher(String, "/fpga/query", 10, **kw)
        self.select = node.create_publisher(String, "/fpga/select_candidate", 10, **kw)
        self.cancel = node.create_publisher(Empty, "/fpga/cancel", 10, **kw)
        self.goto = node.create_publisher(String, "/fpga/goto", 10, **kw)   # 10-17: the runner's single-goal entry
        # 10-18 (hub_core teleop / mapping_stop): the teleop node's ROS input, the in-stack mapping listener
        self.teleop = node.create_publisher(String, "/fpga/teleop/input", 10, **kw)
        self.mapping_stop = node.create_publisher(Empty, "/fpga/mapping/stop", 10, **kw)
        self.map_reset = node.create_publisher(String, "/fpga/map_reset", 10, **kw)   # 10-26 (hub_core map_reset)

    def publish_query(self, text):
        self.query.publish(self.String(data=text))

    def publish_select(self, cid):
        self.select.publish(self.String(data=cid))

    def publish_cancel(self):
        self.cancel.publish(self.Empty())

    def publish_goto(self, text):
        self.goto.publish(self.String(data=text))

    def publish_teleop(self, text):
        self.teleop.publish(self.String(data=text))

    def publish_mapping_stop(self):
        self.mapping_stop.publish(self.Empty())

    def publish_map_reset(self, text):
        self.map_reset.publish(self.String(data=text))


def build_node(server, m, preview=False, clock=fpga_clock.now_ns):
    """``m``: namespace with Node, QoSProfile, DurabilityPolicy, ReliabilityPolicy and message types."""
    core, lock = server.core, server.lock

    counts = core.counts

    def feed(topic, fn):
        key = "cb:" + topic             # per-topic callback count (hub_stats; board vs host load)

        def callback(msg):
            with lock:
                counts[key] += 1
                fn(msg, clock())
        return callback

    def on_plan(msg):
        """/plan (Nav2 planner_server, default topic name; frame = header.frame_id as published,
        Nav2 default global_frame map -- on the car not yet checked). Receive time first, the
        polyline is computed outside the hub lock, only the result is fed under it."""
        t = clock()
        h = msg.header
        summary = summarize_plan([(p.pose.position.x, p.pose.position.y) for p in msg.poses], h.frame_id,
                                 h.stamp.sec + h.stamp.nanosec / 1e9)
        with lock:
            counts["cb:/plan"] += 1
            core.feed_plan(summary[0], t, path=summary[1], path_reason=summary[2])

    # No parameter services and no QoS default event handlers (they only log "incompatible QoS
    # publisher discovered"; diagnose with `ros2 topic info -v`): each one is an entity the
    # executor walks on every wake-up, and the lean executor serves subscriptions only.
    sub_kw = {"event_callbacks": m.SubscriptionEventCallbacks(use_default_callbacks=False)}
    pub_kw = {"event_callbacks": m.PublisherEventCallbacks(use_default_callbacks=False)}

    class HubNode(m.Node):
        def __init__(self):
            m.Node.__init__(self, "fpga_hub", start_parameter_services=False)
            latched = m.QoSProfile(depth=1, durability=m.DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=m.ReliabilityPolicy.RELIABLE)
            self.global_rev = [0, None]
            def sub(kind, topic, cb, qos):
                self.create_subscription(kind, topic, cb, qos, **sub_kw)

            def on(kind, topic, fn, qos):
                sub(kind, topic, feed(topic, fn), qos)

            on(m.String, "/fpga/status", lambda x, t: core.feed_status(x.data, t), 50)
            on(m.String, "/fpga/memory", lambda x, t: core.feed_memory(x.data, t), latched)
            on(m.String, "/fpga/runtime_info", lambda x, t: core.feed_runtime_info(x.data, t), latched)
            on(m.String, "/fpga/region_verdicts", lambda x, t: core.feed_region(x.data, t), 10)
            on(m.String, "/fpga/kv_link/frame_stats", lambda x, t: core.feed_frame_stats(x.data, t), 10)
            on(m.Bool, "/fpga/motion_permit", lambda x, t: core.feed_motion_permit(x.data, t), 10)
            on(m.Twist, "/cmd_vel", lambda x, t: core.feed_cmd_vel(x.linear.x, x.angular.z, t), 10)
            on(m.String, "/fpga/localization_status", lambda x, t: core.feed_loc_status(x.data, t), 10)
            on(m.PoseWithCovarianceStamped, "/fpga/localization_pose", self.on_pose, 10)
            on(m.String, "/fpga/rolling_map/status", lambda x, t: core.feed_rolling_status(x.data, t), 10)
            on(m.OccupancyGrid, "/fpga/rolling_map/grid",
               lambda x, t: core.feed_map("rolling", grid_meta(x, None), grid_bytes(x), t), 1)
            on(m.OccupancyGrid, "/map", self.on_global_map, latched)
            # /odom best effort (10-17): its publishers are the wheel odometry (reliable) and
            # host/ros_odom_base_adapter.py (best effort, the rtabmap tier); a reliable reader matched only
            # the first, and the adapter logged "incompatible QoS". Nav2 Humble reads /odom best effort too.
            on(m.Odometry, "/odom", self.on_odom,
               m.QoSProfile(depth=10, reliability=m.ReliabilityPolicy.BEST_EFFORT))
            on(m.String, "/fpga/odom_adapter_status", lambda x, t: core.feed_odom_adapter(x.data, t), 10)
            on(m.String, "/fpga/mapping/status", lambda x, t: core.feed_mapping_status(x.data, t), latched)
            on(m.String, "/fpga/map_reset/status", lambda x, t: core.feed_map_reset_status(x.data, t), latched)
            # 10-26 search heat map (runner, latched JSON <= 1 Hz): MAP which=belief to the console
            on(m.String, "/fpga/search/belief", lambda x, t: core.feed_belief(x.data, t), latched)
            # 10-29 health check (host/hub/health.py): the chassis driver's 2 Hz status, the wheel odometry's 1 Hz
            on(m.String, "/fpga/chassis/status", lambda x, t: core.feed_chassis_status(x.data, t), 10)
            on(m.String, "/fpga/wheel_odom/status", lambda x, t: core.feed_wheel_odom_status(x.data, t), 10)
            sub(m.Path, "/plan", on_plan, 1)
            if preview:
                on(m.CompressedImage, "/fpga/preview/jpeg", self.on_preview, 1)
                # 10-27 console first-person boxes = the car screen AR's ROI rows (host/hub/fpv_boxes.py);
                # the producer already publishes this per frame (~10 Hz, a few KB JSON): no new Nano load
                on(m.String, "/fpga/geometry_frame", lambda x, t: core.feed_geometry_frame(x.data, t), 10)
            core.actions = RosActions(self, m.String, m.Empty, pub_kw)

        @staticmethod
        def on_pose(x, t):
            p, c = x.pose.pose, x.pose.covariance
            stamp = x.header.stamp.sec + x.header.stamp.nanosec / 1e9
            core.feed_loc_pose({"x": p.position.x, "y": p.position.y, "yaw": yaw_of(p.orientation),
                                "cov_xy_trace": c[0] + c[7], "stamp_ros_s": stamp}, t)

        def on_global_map(self, x, t):
            data = grid_bytes(x)
            crc = zlib.crc32(data)
            if crc != self.global_rev[1]:
                self.global_rev = [self.global_rev[0] + 1, crc]
            core.feed_map("global", grid_meta(x, self.global_rev[0]), data, t)

        @staticmethod
        def on_odom(x, t):
            stamp = x.header.stamp.sec + x.header.stamp.nanosec / 1e9
            p = getattr(getattr(x, "pose", None), "pose", None)     # H2 (10-15): the odometry pose rides along
            kw = {} if p is None else {"x": p.position.x, "y": p.position.y, "yaw": yaw_of(p.orientation),
                                       "frame_id": getattr(x.header, "frame_id", None) or None}
            core.feed_odom(x.twist.twist.linear.x, x.twist.twist.angular.z, stamp, t, **kw)

        @staticmethod
        def on_preview(x, t):
            stamp = x.header.stamp.sec + x.header.stamp.nanosec / 1e9
            w, h = _jpeg_size(bytes(x.data))
            core.feed_preview({"w": w, "h": h, "stamp_ros_s": stamp}, bytes(x.data), t)
            push = getattr(server, "push_preview", None)       # under the hub lock (feed() wrapper)
            if push is not None:
                push(t)

    return HubNode()


def _jpeg_size(data):
    """(w, h) from the first SOF marker; (9999, 9999) if unreadable (then dropped by limits)."""
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2):
            return (data[i + 7] << 8) | data[i + 8], (data[i + 5] << 8) | data[i + 6]
        i += 2 + ((data[i + 2] << 8) | data[i + 3])
    return 9999, 9999


def ros_modules():
    import types
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import Bool, Empty, String
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from sensor_msgs.msg import CompressedImage
    from rclpy.qos_event import SubscriptionEventCallbacks, PublisherEventCallbacks
    return types.SimpleNamespace(**locals())


def _interrupt(signum, frame):
    # first SIGINT/SIGTERM: leave run()'s loop (it writes the stats); later ones are ignored
    # so the shutdown below is not cut half way
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.SIG_IGN)
    raise KeyboardInterrupt


def serve_with_ros(rclpy, executor_factory, make_node, run_fn, ros_args, no_signal_handlers):
    """Run the hub (``run_fn``, blocking) with the ROS node spun by an executor thread; on
    SIGINT/SIGTERM stop cleanly. rclpy's own handlers (SignalHandlerOptions.ALL) shut the
    context down inside the signal and leave the main loop running (10-11 board smoke: Ctrl+C
    and SIGTERM did not stop the hub, no stats); so rclpy installs none
    (``no_signal_handlers``) and both signals raise KeyboardInterrupt in the main thread (set
    explicitly: a background job of a non-interactive shell inherits SIGINT ignored). Order on
    the way out, context still valid: run_fn's own finally (KV loops stop, stats file), then the
    executor is shut down and its thread joined BEFORE the node is destroyed and rclpy shut
    down (a spin thread still inside rcl at interpreter exit aborted the process: "terminate
    called without an active exception", rc 134)."""
    rclpy.init(args=ros_args, signal_handler_options=no_signal_handlers)
    previous = {sig: signal.signal(sig, _interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    node = executor = spin = None
    try:
        node = make_node()
        executor = executor_factory()
        executor.add_node(node)
        spin = threading.Thread(target=executor.spin, name="rclpy-spin", daemon=True)
        spin.start()
        try:
            run_fn()
        except KeyboardInterrupt:       # a signal before run_fn's own loop started
            pass
    finally:
        try:
            if executor is not None:
                executor.shutdown(timeout_sec=2.0)
            if spin is not None:
                spin.join(timeout=5.0)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for sig, handler in previous.items():
                if handler is not None:
                    signal.signal(sig, handler)


def executor_chosen(server, info, clock=fpga_clock.now_ns):
    """Record the executor actually used (stdout line, hub STATE hub.executor, EVENT on fallback)."""
    line = "hub executor=%s" % info["used"]
    if info.get("fallback_reason"):
        line += " requested=%s fallback_reason=%s" % (info["requested"], info["fallback_reason"])
    sys.stdout.write(line + "\n")
    sys.stdout.flush()
    with server.lock:
        server.core.feed_executor_info(info, clock())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    common_args(parser)
    parser.add_argument("--preview", action="store_true", help="forward /fpga/preview/jpeg (default off, O15)")
    parser.add_argument("--executor", choices=("lean", "rclpy"), default=os.environ.get("FPGA_HUB_EXECUTOR") or "lean",
                        help="lean (default; host/hub/hub_executor.py, rclpy humble internals) or rclpy "
                             "(stock SingleThreadedExecutor); env FPGA_HUB_EXECUTOR sets the default")
    args, ros_args = parser.parse_known_args(argv)
    hub_profile.install()              # FPGA_HUB_PROFILE=<dir>: per-thread CPU (default off)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    core = HubCore(args.data_source, state_hz=args.state_hz)
    if os.environ.get("O"):            # 10-28: the stack's run dir (/o) -> start-up notices (ground calib fallback)
        from host.hub.hub_core import ground_calib_notices
        core.notices = ground_calib_notices(os.path.join(os.environ["O"], "ground_calib.json"))
        if core.notices:
            print("hub notices: %s" % "; ".join(n["text"] for n in core.notices), flush=True)
    server = HubServer(core, read_token(args.token_file), port=args.port, host=args.bind,
                       deny_nets=args.deny_local_net or ("198.51.100.0/24",),
                       features=["preview"] if args.preview else [], sndbuf=args.sndbuf)
    if os.environ.get("O"):            # 10-29: health check of the running stack (warn only) -> STATE "health"
        from host.hub.health import HealthMonitor, start_health_thread
        start_health_thread(server, HealthMonitor(os.environ["O"]))
    if args.executor == "lean":
        # lean only if its start-up probe of the rclpy internals passes, else the stock executor;
        # the choice goes to stdout (board scripts grep "hub executor=") and into STATE/EVENT
        from host.hub.hub_executor import SelectingExecutor
        executor_factory = (lambda: SelectingExecutor(SingleThreadedExecutor,
                                                      on_choice=lambda info: executor_chosen(server, info)))
    else:
        executor_factory = SingleThreadedExecutor
        executor_chosen(server, {"requested": "rclpy", "used": "rclpy", "fallback_reason": None})
    serve_with_ros(rclpy, executor_factory,
                   lambda: build_node(server, ros_modules(), preview=args.preview),
                   lambda: run(server, args), ros_args, SignalHandlerOptions.NO)
    hub_profile.finish_main()


if __name__ == "__main__":
    main()
