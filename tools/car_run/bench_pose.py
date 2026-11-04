#!/usr/bin/env python3
"""Line 4 bench (2026-10-15): the pose of a car that cannot move -- car_up.sh --pose static.

The real camera is fixed on a bench in the car's mount (0.36 m, 30 deg down) and there is no chassis, so the car's
pose is a constant: map -> odom -> base_link = identity IS the true pose for the whole run. This node publishes
exactly that, and nothing else:
  /tf_static  map -> odom identity, odom -> base_link identity
              (--odom-hz > 0: odom -> base_link on /tf at that rate instead, stamped ROS now -- the way
              tools/integ_car_chain/live_tf.py ran the 10-12/10-15 bench rigs; default 0 = static, which answers a
              lookup at any stamp and so never makes a consumer wait for a TF sample)
  /map        (latched: RELIABLE + TRANSIENT_LOCAL, depth 1; frame "map") a FREE occupancy grid of --map-m x --map-m
              metres at --map-res-m, centred on the base -- a bench STAND-IN, not a built map: host/ros_runtime.py
              ready() needs a map in frame "map" before it takes a query, and nothing else publishes /map without
              Nav2. --map-m 0 = no map. Default 8 m (the line-4 bench); car_up.sh --pose odom passes 20 m
              (SELECT_SEMANTICS_20261016.md section 3: 8 m left every object past ~3.7 m without an observation pose).
              --map-yaml Y (car_up --pose odom with a mapping export, 10-16): publish that rtabmap-export --map
              (map_server yaml + PGM, read the way map_server's trinary mode reads it) instead of the stand-in. Its frame
              is the mapping run's map frame: right only when the car starts where the mapping run started, same heading.
--map-align (car_inner MAPPING=1, 10-25 owner plan A): also subscribes host/map_align.TOPIC (latched JSON from
tools/car_run/mapping_ctl.py when the mapping run stops) and re-sends map -> odom on /tf_static as its "map_odom"
(the optimized-graph correction C_last), still static and still the one publisher of that edge; a message that is not
a finite rigid pose is refused (logged), the edge then keeps its value. Without the flag nothing changes.
It does NOT publish base_link -> camera optical frame: that edge is the camera mount, published by car_inner.sh's
static_transform_publisher (one publisher per edge). It is not mapping and not localization; car.json and the run's
marks say so. Runs until SIGINT/SIGTERM. Python 3.8 compatible (no ROS needed to import it)."""
import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

EDGES = (("map", "odom"), ("odom", "base_link"))
NOTE = "台架静态位姿（相机固定、无底盘：map->odom->base_link 恒等即真位姿），不是建图定位"
# --no-odom-base (car_up --pose odom, 10-15): only map -> odom identity (+ the stand-in map); odom -> base_link comes
# from host/ros_wheel_odom.py --tf-topic /tf (one publisher per edge)
NOTE_ODOM = "轮式里程位姿：map->odom 恒等（起点即原点）+ odom->base_link 由轮式里程发（占位轮参数时只是粗推算），不是建图定位"


def free_grid(size_m, res_m):
    """(width, height, origin_x, origin_y, data) of a free square grid centred on the origin; None for size 0."""
    if size_m <= 0:
        return None
    if not res_m > 0:
        raise ValueError("map resolution must be positive")
    n = int(round(size_m / res_m))
    if n < 1:
        raise ValueError("map smaller than one cell")
    half = n * res_m / 2.0
    return n, n, -half, -half, [0] * (n * n)


def _pgm_tokens(raw):
    """(magic, width, height, maxval, offset of the first pixel byte) of a PGM header; '#' comments allowed."""
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b"#":
            while i < len(raw) and raw[i:i + 1] not in (b"\n", b"\r"):
                i += 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace() and raw[j:j + 1] != b"#":
            j += 1
        if j == i:
            raise ValueError("PGM header truncated")
        tokens.append(raw[i:j])
        i = j
    if tokens[0] not in (b"P5", b"P2"):
        raise ValueError("not a greyscale PGM (P5/P2): %r" % tokens[0])
    w, h, maxval = (int(t) for t in tokens[1:])
    if w < 1 or h < 1 or not 0 < maxval < 65536:
        raise ValueError("bad PGM size or maxval")
    return tokens[0], w, h, maxval, i + 1     # exactly one whitespace byte after maxval


def read_pgm(path):
    """(width, height, maxval, pixel values top row first) of a P5/P2 PGM."""
    with open(path, "rb") as f:
        raw = f.read()
    magic, w, h, maxval, off = _pgm_tokens(raw)
    if magic == b"P2":
        values = [int(t) for t in raw[off:].split()]
    elif maxval < 256:
        values = list(raw[off:off + w * h])
    else:
        body = raw[off:off + 2 * w * h]
        values = [body[k] << 8 | body[k + 1] for k in range(0, len(body) - 1, 2)]
    if len(values) < w * h:
        raise ValueError("PGM %s: %d pixels, header says %d" % (path, len(values), w * h))
    return w, h, maxval, values[:w * h]


def load_map_yaml(path):
    """A map_server map (yaml + PGM, e.g. rtabmap-export --map) as an OccupancyGrid body.

    map_server trinary mode: p = (maxval - v) / maxval (negate: v / maxval); p > occupied_thresh -> 100,
    p < free_thresh -> 0, else -1 (unknown). Image row 0 is the TOP of the map (largest y): rows are flipped so that
    data[0] is the cell at the origin, as in OccupancyGrid. Returns (width, height, resolution, origin [x, y, yaw], data)."""
    import yaml
    with open(path) as f:
        meta = yaml.safe_load(f)
    if not isinstance(meta, dict) or "image" not in meta or "resolution" not in meta or "origin" not in meta:
        raise ValueError("map yaml %s: image, resolution and origin required" % path)
    mode = meta.get("mode", "trinary")
    if mode != "trinary":
        raise ValueError("map yaml %s: only mode trinary is read (given %r)" % (path, mode))
    image = meta["image"]
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"]]
    if len(origin) == 2:
        origin.append(0.0)
    occ_t, free_t = float(meta.get("occupied_thresh", 0.65)), float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))
    if not res > 0 or len(origin) != 3 or not all(np.isfinite(origin)):
        raise ValueError("map yaml %s: bad resolution or origin" % path)
    w, h, maxval, values = read_pgm(image)
    v = np.asarray(values, dtype=float).reshape(h, w)[::-1]        # bottom row first
    p = v / maxval if negate else (maxval - v) / maxval
    grid = np.full((h, w), -1, dtype=np.int8)
    grid[p > occ_t] = 100
    grid[p < free_t] = 0
    return w, h, res, origin, grid.reshape(-1).tolist()


def plan(odom_hz, map_m, map_res_m, odom_base=True, map_yaml=""):
    """What the node will publish (also printed at start and used by the tests). odom_base False: no odom -> base_link
    at all (another node -- the wheel odometry -- owns that edge). map_yaml: the exported map instead of the stand-in."""
    if not odom_base:
        odom_hz = 0
    p = {"static_edges": [list(e) for e in (EDGES if odom_hz <= 0 and odom_base else EDGES[:1])],
         "dynamic_edges": [] if odom_hz <= 0 else [list(EDGES[1])], "odom_hz": odom_hz, "odom_base": odom_base}
    if map_yaml:
        w, h, res, origin, data = load_map_yaml(map_yaml)
        p["map"] = {"frame": "map", "width": w, "height": h, "resolution": res, "origin": origin, "free": False,
                    "stand_in": False, "source": "export", "yaml": os.path.abspath(map_yaml),
                    "cells": {"free": data.count(0), "occupied": data.count(100), "unknown": data.count(-1)}}
    else:
        g = free_grid(map_m, map_res_m)
        p["map"] = None if g is None else {"frame": "map", "width": g[0], "height": g[1], "resolution": map_res_m,
                                           "origin": [g[2], g[3]], "free": True, "stand_in": True}
    p["note"] = NOTE if odom_base else NOTE_ODOM
    return p


def grid_body(a):
    """(width, height, resolution, origin_x, origin_y, origin_yaw, data) the node publishes on /map, or None."""
    if a.map_yaml:
        w, h, res, (ox, oy, yaw), data = load_map_yaml(a.map_yaml)
        return w, h, res, ox, oy, yaw, data
    g = free_grid(a.map_m, a.map_res_m)
    if g is None:
        return None
    w, h, ox, oy, data = g
    return w, h, float(a.map_res_m), ox, oy, 0.0, data


def transforms(p, map_odom=None):
    """(static, dynamic) lists of (parent, child, 4x4 transform) the node sends for plan p: every edge identity, except
    map -> odom = map_odom (a 4x4, the post-mapping alignment) when given."""
    def tf(a, b):
        return np.array(map_odom, dtype=float) if map_odom is not None and (a, b) == EDGES[0] else np.eye(4)
    return ([(a, b, tf(a, b)) for a, b in p["static_edges"]], [(a, b, tf(a, b)) for a, b in p["dynamic_edges"]])


def align_matrix(text):
    """The 4x4 map -> odom of a host/map_align.TOPIC message (JSON with "map_odom": [x, y, z, qx, qy, qz, qw]) and
    its key; raises ValueError for anything else."""
    from host.map_align import pose7_matrix
    doc = json.loads(text)
    pose = [float(v) for v in doc["map_odom"]]
    if len(pose) != 7:
        raise ValueError("map_odom must be [x, y, z, qx, qy, qz, qw]")
    if abs(math.sqrt(sum(v * v for v in pose[3:])) - 1.0) > 1e-6:
        raise ValueError("map_odom quaternion is not unit")
    return pose7_matrix(*pose), doc.get("key")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--odom-hz", type=float, default=0.0)
    ap.add_argument("--map-m", type=float, default=8.0)
    ap.add_argument("--map-res-m", type=float, default=0.05)
    ap.add_argument("--no-odom-base", action="store_true", help="publish no odom -> base_link (wheel odometry owns it)")
    ap.add_argument("--map-yaml", default="", help="publish this map_server map (rtabmap-export) instead of the stand-in")
    ap.add_argument("--map-align", action="store_true",
                    help="re-send map -> odom from the mapping run's alignment (host/map_align.TOPIC)")
    a, ros_args = ap.parse_known_args(argv)
    p = plan(a.odom_hz, a.map_m, a.map_res_m, odom_base=not a.no_odom_base, map_yaml=a.map_yaml)
    print(json.dumps(p, ensure_ascii=False), flush=True)
    import rclpy
    import rclpy.executors
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from tools.car_geo_producer.replay_camera import tf_msg

    rclpy.init(args=ros_args)
    node = Node("fpga_bench_pose")
    static, dynamic = transforms(p)
    now = node.get_clock().now().to_msg()
    stb = StaticTransformBroadcaster(node)
    stb.sendTransform([tf_msg(TransformStamped, parent, child, T, now) for parent, child, T in static])
    if a.map_align and list(EDGES[0]) in p["static_edges"]:
        from std_msgs.msg import String
        from host.map_align import TOPIC as ALIGN_TOPIC
        done = set()

        def on_align(msg):
            try:
                m, key = align_matrix(msg.data)
            except (ValueError, KeyError, TypeError) as exc:
                node.get_logger().error("map align refused, map -> odom unchanged: %s" % exc)
                return
            if key in done:
                return
            done.add(key)
            # StaticTransformBroadcaster keeps one entry per child frame: odom's is replaced, the others stay
            stb.sendTransform([tf_msg(TransformStamped, "map", "odom", m, node.get_clock().now().to_msg())])
            print("map align: map -> odom re-sent %s (key %s)" % (json.loads(msg.data)["map_odom"], key), flush=True)

        node.create_subscription(String, ALIGN_TOPIC, on_align, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    if dynamic:
        tfb = TransformBroadcaster(node)
        parent, child, T = dynamic[0]
        node.create_timer(1.0 / a.odom_hz, lambda: tfb.sendTransform(
            tf_msg(TransformStamped, parent, child, T, node.get_clock().now().to_msg())))
    if p["map"] is not None:
        w, h, res, ox, oy, yaw, data = grid_body(a)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        pub = node.create_publisher(OccupancyGrid, "/map", qos)
        m = OccupancyGrid()
        m.header.frame_id = "map"
        m.header.stamp = now
        m.info.resolution = float(res)
        m.info.width, m.info.height = w, h
        m.info.origin.position.x, m.info.origin.position.y = float(ox), float(oy)
        m.info.origin.orientation.z, m.info.origin.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.data = data
        pub.publish(m)
    print("bench pose up: %s" % p["note"], flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
