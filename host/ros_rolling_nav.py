"""ROS 2 wrappers for the rolling-map -> Nav2 bridge (2026-09-27).

See docs/design/ROLLING_NAV2_LAYER_20260927.md. All decisions live in
``host/rolling_nav_layer.py``; this file only moves bytes and messages.

Topics (defaults):
  /fpga/rolling_map/delta           std_msgs/UInt8MultiArray  producer -> bridge
  /fpga/rolling_map/snapshot        std_msgs/UInt8MultiArray  producer -> bridge
  /fpga/rolling_map/resync_request  std_msgs/String (JSON)    bridge -> producer
  /fpga/rolling_map/grid            nav_msgs/OccupancyGrid    bridge -> Nav2 StaticLayer
                                   (transient_local, reliable, depth 1)
  /fpga/rolling_map/status          std_msgs/String (JSON)    bridge -> nav consumer
  /fpga/rolling_map/grid_raw        nav_msgs/OccupancyGrid    strict grid without the body
                                   outline (measurement only)

  /fpga/rolling_map/cold_start_confirm  std_msgs/String (JSON {"operator", "operator_stamp"})
                                   tools/cold_start_confirm.py -> bridge
  /fpga/rolling_map/cold_start_result   std_msgs/String (JSON)  bridge -> CLI

The body outline (owner ruling 2026-09-27) uses the latest odom->base_link TF
at publish time; without it the strict grid is published. The cold-start
ring (second ruling) needs ``cold_start_width_m`` and ``cold_start_log_path``;
every confirmation attempt is appended to that file before it takes effect.

B1 (2026-11-02, car_inner CAR_PL_GRID=1): the producer publishes /grid, /grid_raw and /status itself from
the KV260's navigation planes (GridOutput below, host/pl_grid_layer.py) and this bridge is not started; with
CAR_PL_GRID_SHADOW=1 it runs beside it as the software reference on the parameters grid_topic / raw_topic /
status_topic (/fpga/rolling_map/grid_sw, /grid_raw_sw, /status_sw) for the cell-by-cell car comparison.
Defaults = the topics above, unchanged.

Both ends use host.fpga_clock (CLOCK_MONOTONIC_RAW, clock id "nano_monotonic_raw_ns"),
the clock of the KV260 frame chain: the producer (host/ros_geo_producer.py) stamps
acquisitions with it, so the bridge must age them on the same clock (2026-10-11;
was time.monotonic_ns()/"mono", which made every KV-chain delta clock_mismatch).
"""

import json
import math
import os
import sys

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, UInt8MultiArray
from tf2_ros import Buffer, TransformListener

from host.fpga_clock import CLOCK_ID, now_ns  # noqa: F401 (re-exported: producers import both)
from host.rolling_nav_layer import (HeightBand, RollingLayerBridge, decode_delta,
                                    decode_snapshot, encode_delta, encode_snapshot)

LOOSE_NUMBER_PARAMS = ("swept_clear_s",)   # 10-28: integer or double on the command line, used as float

GRID_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
STREAM_QOS = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)


def bytes_msg(raw):
    msg = UInt8MultiArray()
    msg.data = bytes(raw)
    return msg


def grid_msg(fields, stamp):
    msg = OccupancyGrid()
    msg.header.frame_id = fields.frame_id
    msg.header.stamp = stamp
    msg.info.map_load_time = stamp
    msg.info.resolution = float(fields.resolution)
    msg.info.width, msg.info.height = int(fields.width), int(fields.height)
    msg.info.origin.position.x = float(fields.origin_xy[0])
    msg.info.origin.position.y = float(fields.origin_xy[1])
    msg.info.origin.orientation.w = 1.0
    msg.data = fields.data.astype(int).tolist()
    return msg


class RollingBridgeNode(Node):
    """Nav2-side bridge: deltas in, OccupancyGrid + freshness status out."""

    def __init__(self, *, dims=(80, 80, 32), voxel_m=0.05):
        super().__init__("fpga_rolling_nav_bridge")
        for name, default in (("ground_z_m", float("nan")), ("robot_height_m", float("nan")),
                              ("frame_id", "odom"), ("status_hz", 20.0),
                              ("max_map_age_ms", 200.0), ("robot_radius_m", float("nan")),
                              ("body_margin_m", 0.0),
                              ("front_seed_m", 0.0),   # 10-26 front-sector seed, 0 = off (car_up CAR_FRONT_SEED_M)
                              ("swept_clear_s", 0.0),  # 10-28 swept-area clearing, 0 = off (car_inner: 300)
                              ("base_frame", "base_link"),
                              # B1 shadow mode (CAR_PL_GRID_SHADOW=1): the bridge's outputs on other topics
                              ("grid_topic", "/fpga/rolling_map/grid"),
                              ("raw_topic", "/fpga/rolling_map/grid_raw"),
                              ("status_topic", "/fpga/rolling_map/status"),
                              ("cold_start_width_m", float("nan")),
                              ("cold_start_timeout_s", 30.0), ("cold_start_log_path", "")):
            if name in LOOSE_NUMBER_PARAMS:
                # 10-28: `-p swept_clear_s:=300` arrives as INTEGER; a DOUBLE declaration then raised
                # InvalidParameterTypeException and took the whole stack down -- accept any number, float() below
                self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
            else:
                self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        band = HeightBand(ground_z_m=p("ground_z_m"), robot_height_m=p("robot_height_m"))
        self.bridge = RollingLayerBridge(dims=dims, voxel_m=voxel_m, band=band,
                                         clock_id=CLOCK_ID, frame_id=p("frame_id"),
                                         max_map_age_ns=int(p("max_map_age_ms") * 1e6),
                                         body_radius_m=p("robot_radius_m"),
                                         body_margin_m=p("body_margin_m"),
                                         front_seed_m=p("front_seed_m"),
                                         swept_clear_s=float(p("swept_clear_s")),
                                         cold_start_width_m=(None if p("cold_start_width_m") !=
                                                             p("cold_start_width_m") else
                                                             p("cold_start_width_m")),
                                         cold_start_timeout_ns=int(
                                             p("cold_start_timeout_s") * 1e9),
                                         cold_start_log=self._log_writer(
                                             p("cold_start_log_path")))
        self.cold_result_pub = self.create_publisher(
            String, "/fpga/rolling_map/cold_start_result", 10)
        self.create_subscription(String, "/fpga/rolling_map/cold_start_confirm",
                                 self.on_cold_start_confirm, 10)
        self.base_frame = p("base_frame")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.raw_pub = self.create_publisher(OccupancyGrid, p("raw_topic"), GRID_QOS)
        self.grid_pub = self.create_publisher(OccupancyGrid, p("grid_topic"), GRID_QOS)
        self.status_pub = self.create_publisher(String, p("status_topic"), 10)
        self.request_pub = self.create_publisher(String, "/fpga/rolling_map/resync_request", 10)
        self.create_subscription(UInt8MultiArray, "/fpga/rolling_map/delta", self.on_delta,
                                 STREAM_QOS)
        self.create_subscription(UInt8MultiArray, "/fpga/rolling_map/snapshot",
                                 self.on_snapshot, STREAM_QOS)
        self.create_timer(1.0 / p("status_hz"), self.publish_status)
        self._last_fresh = None

    def _after(self, changed, request):
        if request is not None:
            self.request_pub.publish(String(data=json.dumps(request)))
        if changed:
            raw = self.bridge.strict_grid()
            if raw is not None:
                stamp = self.get_clock().now().to_msg()
                pose = self.body_pose()
                fields = self.bridge.grid(body_xy=None if pose is None else pose[:2], now_ns=now_ns(),
                                          body_yaw=None if pose is None else pose[2])
                self.grid_pub.publish(grid_msg(fields, stamp))
                self.raw_pub.publish(grid_msg(raw, stamp))
                self.get_logger().debug("grid epoch %d seq %d" % (fields.epoch, fields.delta_seq))
        self.publish_status()

    @staticmethod
    def _log_writer(path):
        if not path:
            return None

        def write(line):
            with open(path, "a") as stream:
                stream.write(line + "\n")
                stream.flush()
                os.fsync(stream.fileno())
        return write

    def body_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.bridge.frame_id, self.base_frame,
                                                 rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - any TF failure means "no pose": strict grid
            if self.bridge.counts["body_pose_missing"] < 3:
                self.get_logger().warn("no body pose, strict grid: %s" % exc)
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    def body_xy(self):
        pose = self.body_pose()
        return None if pose is None else pose[:2]

    def on_cold_start_confirm(self, msg):
        try:
            req = json.loads(msg.data)
        except ValueError:
            req = {}
        ok, reason = self.bridge.confirm_cold_start(
            operator=req.get("operator"), operator_stamp=req.get("operator_stamp"),
            now_ns=now_ns(), body_pose=self.body_pose())
        self.get_logger().warn("cold-start confirmation by %r: %s" % (req.get("operator"), reason))
        self.cold_result_pub.publish(String(data=json.dumps(
            {"accepted": ok, "reason": reason, "request": req,
             "state": self.bridge.cold_start_state()})))
        self._after(ok, None)

    def on_delta(self, msg):
        try:
            delta = decode_delta(bytes(msg.data))
        except (ValueError, KeyError) as exc:
            self.bridge.counts["rejected:malformed"] += 1
            self.get_logger().warn("malformed delta: %s" % exc)
            return
        changed, reason, request = self.bridge.on_delta(delta, now_ns=now_ns())
        if not changed:
            self.get_logger().info("delta %d/%d rejected: %s" % (delta.epoch, delta.delta_seq,
                                                                 reason))
        self._after(changed, request)

    def on_snapshot(self, msg):
        try:
            wire = decode_snapshot(bytes(msg.data))
        except (ValueError, KeyError) as exc:
            self.bridge.counts["snapshot_rejected:malformed"] += 1
            self.get_logger().warn("malformed snapshot: %s" % exc)
            return
        changed, reason, request = self.bridge.on_snapshot(wire, now_ns=now_ns())
        self.get_logger().info("snapshot %d/%d: %s" % (wire.snapshot.epoch,
                                                       wire.snapshot.delta_seq, reason))
        self._after(changed, request)

    def publish_status(self):
        row = self.bridge.status(now_ns=now_ns(), clock_id=CLOCK_ID)
        row["counts"] = dict(self.bridge.counts)
        self.status_pub.publish(String(data=json.dumps(row, sort_keys=True)))
        if row["fresh"] != self._last_fresh:
            self.get_logger().info("rolling map fresh=%s (%s)" % (row["fresh"], row["reason"]))
            self._last_fresh = row["fresh"]


class RollingProducerLink:
    """Producer side: publish accepted deltas, answer resync requests.

    ``fusion`` is a ``RollingFusion``; the caller passes each accepted
    ``FusionResult.delta`` to ``publish``. ``drop_next`` is a test hook that
    withholds exactly one delta to exercise the gap/resync path.
    """

    def __init__(self, node, fusion, *, answer_thread=False):
        """``answer_thread`` (remote KV rolling backend, go-live 2026-10-26): resync requests
        only wake a daemon thread that takes the snapshot (a DUMP round trip to the KV260),
        so the ROS executor thread never waits on the link; requests arriving while one is
        being answered are merged into one more answer."""
        self.node, self.fusion = node, fusion
        self.delta_pub = node.create_publisher(UInt8MultiArray, "/fpga/rolling_map/delta",
                                               STREAM_QOS)
        self.snapshot_pub = node.create_publisher(UInt8MultiArray, "/fpga/rolling_map/snapshot",
                                                  STREAM_QOS)
        node.create_subscription(String, "/fpga/rolling_map/resync_request", self.on_request, 10)
        self.last_acquired_ns = None
        self.drop_next = False
        self.counts = {"published": 0, "dropped_on_purpose": 0, "snapshots": 0,
                       "snapshot_unavailable": 0}
        self._wake = None
        if answer_thread:
            import threading
            self.counts.update(snapshot_error=0, requests_merged=0)
            self._wake = threading.Event()
            threading.Thread(target=self._answer_loop, name="rolling-resync", daemon=True).start()

    def _answer_loop(self):
        while True:
            self._wake.wait()
            self._wake.clear()
            try:
                self.answer()
            except Exception as exc:            # never let one failed answer end the thread
                self.counts["snapshot_error"] += 1
                sys.stderr.write("rolling resync answer failed: %r\n" % (exc,))

    def publish(self, delta, timings=None):
        """``timings`` (optional dict) gets RAW ns stamps: enc0 (before encode), enc1 (message
        built), pub1 (DDS publish call returned) -- the producer's stall split (10-11 night)."""
        self.last_acquired_ns = delta.acquired_ns
        if self.drop_next:
            self.drop_next = False
            self.counts["dropped_on_purpose"] += 1
            return
        if timings is not None:
            timings["enc0"] = now_ns()
        msg = bytes_msg(encode_delta(delta))
        if timings is not None:
            timings["enc1"] = now_ns()
        self.delta_pub.publish(msg)
        if timings is not None:
            timings["pub1"] = now_ns()
        self.counts["published"] += 1

    def on_request(self, msg):
        if self._wake is not None:
            if self._wake.is_set():
                self.counts["requests_merged"] += 1
            self._wake.set()
            return
        self.answer()

    def answer(self):
        snap = self.fusion.snapshot(now_ns=now_ns(), clock_id=CLOCK_ID)
        if not snap.valid or self.last_acquired_ns is None:
            self.counts["snapshot_unavailable"] += 1
            return
        self.snapshot_pub.publish(bytes_msg(encode_snapshot(
            snap, acquired_ns=self.last_acquired_ns, clock_id=CLOCK_ID,
            voxel_m=self.fusion.voxel_m)))
        self.counts["snapshots"] += 1


class GridOutput:
    """B1 (2026-11-02): /fpga/rolling_map/grid, /grid_raw and /status published by the producer node from a
    layer -- host.pl_grid_layer.PlGridLayer (the KV260's planes) or, when the KV260 has no grid op, an
    in-process RollingLayerBridge fed with the producer's own deltas (source "nano_sw"). Same messages,
    QoS, stamps and status fields as RollingBridgeNode (plus "grid_source"); the status timer runs at
    ``status_hz`` (20 Hz, as the bridge). ``pose`` () -> (x, y, yaw) in the layer's frame or None: the
    latest odom<-base_link, as the bridge's body_pose. ``update`` runs on the producer's worker thread,
    the timer on the executor thread: one lock around the layer."""

    def __init__(self, node, layer, *, pose, source, status_hz=20.0, grid_topic="/fpga/rolling_map/grid",
                 raw_topic="/fpga/rolling_map/grid_raw", status_topic="/fpga/rolling_map/status"):
        import threading
        self.node, self.layer, self.pose, self.source = node, layer, pose, source
        self.lock = threading.Lock()
        self.grid_pub = node.create_publisher(OccupancyGrid, grid_topic, GRID_QOS)
        self.raw_pub = node.create_publisher(OccupancyGrid, raw_topic, GRID_QOS)
        self.status_pub = node.create_publisher(String, status_topic, 10)
        self.counts = {"grids": 0, "no_grid": 0, "pose_missing": 0}
        self._last_fresh = None
        node.create_timer(1.0 / float(status_hz), self.publish_status)

    def publish_grid(self):
        """After the layer took a new map: the bridge's _after(changed=True)."""
        with self.lock:
            raw = self.layer.strict_grid()
            if raw is None:
                self.counts["no_grid"] += 1
                fields = None
            else:
                pose = self.pose()
                if pose is None:
                    self.counts["pose_missing"] += 1
                fields = self.layer.grid(body_xy=None if pose is None else pose[:2], now_ns=now_ns(),
                                         body_yaw=None if pose is None else pose[2])
        if fields is not None:
            stamp = self.node.get_clock().now().to_msg()
            self.grid_pub.publish(grid_msg(fields, stamp))
            self.raw_pub.publish(grid_msg(raw, stamp))
            self.counts["grids"] += 1
        self.publish_status()

    def publish_status(self):
        with self.lock:
            row = self.layer.status(now_ns=now_ns(), clock_id=CLOCK_ID)
            row["counts"] = dict(self.layer.counts)
        row["grid_source"] = self.source
        self.status_pub.publish(String(data=json.dumps(row, sort_keys=True)))
        if row["fresh"] != self._last_fresh:
            self.node.get_logger().info("rolling map fresh=%s (%s, %s)" % (row["fresh"], row["reason"],
                                                                         self.source))
            self._last_fresh = row["fresh"]


def main(args=None):
    rclpy.init(args=args)
    node = RollingBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
