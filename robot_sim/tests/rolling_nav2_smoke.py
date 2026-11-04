#!/usr/bin/env python3
"""Container smoke: CPU M7Pipeline + RollingFusion -> bridge -> Nav2 (2026-09-27).

Software only (no car, no Nano, no KV260). Roles:
  scene   kinematic robot + synthetic depth camera + M7Pipeline(cpu) +
          RollingFusion + RollingProducerLink (the producer side)
  driver  navigation consumer and measurement (permit, goals, checks)
  params  write the merged Nav2 parameter file

See docs/design/ROLLING_NAV2_LAYER_20260927.md sections 6 and 8.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

VOXEL = 0.05
ROBOT_RADIUS = 0.15
ROBOT_HEIGHT = 0.25
GROUND_Z = 0.0
CAM_XYZ = (0.10, 0.0, 0.30)
CAM_PITCH = math.radians(25.0)
IMG_W, IMG_H, HFOV = 64, 48, math.radians(70.0)
FRAME_HZ = 10.0
# Static scene (axis-aligned boxes: xmin, xmax, ymin, ymax, zmax).
WALLS = [(1.45, 1.55, -1.2, 1.2, 0.6), (-1.5, 1.55, 0.9, 1.0, 0.6),
         (-1.5, 1.55, -1.0, -0.9, 0.6)]
BOX_A = (0.55, 0.75, -0.25, 0.05, 0.35)     # static obstacle between start and goal
BOX_B = (0.80, 0.95, 0.25, 0.40, 0.30)      # pop-up obstacle, inside seen-FREE cells
START = (-0.60, 0.0)
GOAL_1 = (1.15, 0.30)
GOAL_2 = (0.10, 0.10)


def mono():
    # The bridge ages deltas on host.fpga_clock (CLOCK_MONOTONIC_RAW) since 2026-10-11.
    from host.fpga_clock import now_ns
    return now_ns()


# ----------------------------------------------------------------------------- params
def write_params(out, *, s0=False):
    import yaml
    with open("/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml") as f:
        p = yaml.safe_load(f)
    rolling = {"plugin": "nav2_costmap_2d::StaticLayer", "map_topic": "/fpga/rolling_map/grid",
               "map_subscribe_transient_local": True, "subscribe_to_updates": False,
               "footprint_clearing_enabled": False, "transform_tolerance": 0.1}
    if os.environ.get("FPGA_SMOKE_CLEARING") == "1":  # variant: the first-try settings
        rolling["footprint_clearing_enabled"] = True
    inflation = {"plugin": "nav2_costmap_2d::InflationLayer", "inflation_radius": 0.35,
                 "cost_scaling_factor": 3.0, "inflate_unknown": False}
    common = {"global_frame": "odom", "robot_base_frame": "base_link", "rolling_window": True,
              "width": 4, "height": 4, "resolution": VOXEL, "origin_x": 0.0, "origin_y": 0.0,
              "update_frequency": 10.0, "publish_frequency": 10.0,
              "robot_radius": ROBOT_RADIUS, "footprint_padding": 0.0,
              "track_unknown_space": True,
              "trinary_costmap": True, "lethal_cost_threshold": 100,
              "unknown_cost_value": 255, "use_maximum": False,
              "always_send_full_costmap": True,
              "plugins": ["rolling_layer", "inflation_layer"],
              "rolling_layer": rolling, "inflation_layer": inflation}
    for name in ("local_costmap", "global_costmap"):
        p[name][name]["ros__parameters"].update(json.loads(json.dumps(common)))
    p["bt_navigator"]["ros__parameters"]["global_frame"] = "odom"
    p["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] = False
    # Default tolerance 0.5 lets NavFn plan to a reachable point near an unknown
    # goal and report success there (seen in the first cold-start run).
    p["planner_server"]["ros__parameters"]["GridBased"]["tolerance"] = 0.0
    fp = p["controller_server"]["ros__parameters"]["FollowPath"]
    fp.update({"xy_goal_tolerance": 0.10, "max_vel_x": 0.20, "max_speed_xy": 0.20,
               "max_vel_theta": 0.8})
    if os.environ.get("FPGA_SMOKE_CLEARING") != "1":
        # Footprint-edge check: the body, not only the centre, must stay out of
        # LETHAL and NO_INFORMATION cells.
        fp["critics"] = ["RotateToGoal", "Oscillation", "BaseObstacle", "ObstacleFootprint",
                         "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
        fp["ObstacleFootprint.scale"] = 0.02
    p["controller_server"]["ros__parameters"]["general_goal_checker"]["xy_goal_tolerance"] = 0.10
    p["velocity_smoother"]["ros__parameters"].update(
        {"max_velocity": [0.20, 0.0, 0.8], "min_velocity": [-0.20, 0.0, -0.8],
         "velocity_timeout": 0.5})
    # Standalone check costmaps: rolling layer only, no inflation, no footprint clearing.
    check = dict(common, plugins=["rolling_layer"],
                 rolling_layer=dict(rolling, footprint_clearing_enabled=False))
    check.pop("inflation_layer")
    # Nav2's standalone costmap executable pins its name to /costmap/costmap,
    # so only one check costmap per run; S0 (track_unknown_space false) is a
    # separate run with FPGA_SMOKE_S0=1.
    p["costmap"] = {"costmap": {"ros__parameters": dict(
        check, use_sim_time=False, track_unknown_space=not s0)}}
    p["check_lifecycle"] = {"ros__parameters": {
        "use_sim_time": False, "autostart": True, "bond_timeout": 0.0,
        "node_names": ["/costmap/costmap"]}}
    from host.rolling_nav_layer import check_costmap_config
    problems = check_costmap_config(p["local_costmap"]["local_costmap"]["ros__parameters"],
                                    follow_path=fp,
                                    planner=p["planner_server"]["ros__parameters"]["GridBased"])
    if os.environ.get("FPGA_SMOKE_CLEARING") == "1":
        problems = []  # the deliberately unsafe first-try variant
    if problems:
        raise SystemExit("local costmap config refused: %s" % problems)
    class NoAlias(yaml.SafeDumper):  # rcl's params parser rejects YAML aliases
        def ignore_aliases(self, data):
            return True
    with open(out, "w") as f:
        yaml.dump(p, f, Dumper=NoAlias)


# ----------------------------------------------------------------------------- scene
def camera_rotation(yaw):
    level = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    c, s = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    pitch = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]])
    return rz @ pitch @ level


def intrinsics():
    fx = (IMG_W / 2) / math.tan(HFOV / 2)
    return np.array([[fx, 0., (IMG_W - 1) / 2], [0., fx, (IMG_H - 1) / 2], [0., 0., 1.]])


def render(pose_xyyaw, boxes):
    """Optical-axis depth of a floor plane plus axis-aligned boxes (NaN = no return)."""
    x, y, yaw = pose_xyyaw
    r = camera_rotation(yaw)
    c = np.array([x + CAM_XYZ[0] * math.cos(yaw), y + CAM_XYZ[0] * math.sin(yaw), CAM_XYZ[2]])
    k_inv = np.linalg.inv(intrinsics())
    u, v = np.meshgrid(np.arange(IMG_W), np.arange(IMG_H))
    pix = np.stack([u, v, np.ones_like(u)], -1).reshape(-1, 3).astype(np.float64)
    d = (r @ (k_inv @ pix.T)).T  # camera-z = 1 => ray parameter == optical depth
    best = np.full(d.shape[0], np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -c[2] / d[:, 2]
        best = np.where((d[:, 2] < 0) & (t > 0), t, best)
        for (x0, x1, y0, y1, z1) in boxes:
            lo, hi = np.array([x0, y0, 0.0]), np.array([x1, y1, z1])
            t0, t1 = (lo - c) / d, (hi - c) / d
            tmin = np.max(np.minimum(t0, t1), axis=1)
            tmax = np.min(np.maximum(t0, t1), axis=1)
            hit = (tmax >= tmin) & (tmax > 0)
            best = np.where(hit & (tmin > 0) & (tmin < best), tmin, best)
    best[best > 4.0] = np.nan
    return best.reshape(IMG_H, IMG_W).astype(np.float32), r, c


def run_scene(args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from host.m7_backend import M7BackendSelector
    from host.m7_frame_native import compile_frame_native
    from host.m7_grid_policy import FollowCameraGrid
    from host.m7_pipeline import CaptureIdentity, DepthFrame, GridSpec, M7Pipeline, PoseAtCapture
    from host.m7_rolling import RollingFusion, compile_rolling
    from host.rolling_nav_layer import HeightBand, occupancy_values, project_columns
    from host.ros_rolling_nav import CLOCK_ID, RollingProducerLink

    tmp = Path(tempfile.mkdtemp(prefix="rolling-smoke-"))
    selector = M7BackendSelector(cpu_frame_library=compile_frame_native(tmp / "frame.so"))
    frame_backend = selector.frame_backend("cpu")
    rolling_backend = selector.rolling_backend("cpu", cpu_library=compile_rolling(tmp / "r.so"))
    shape = (32, 32, 10)
    policy = FollowCameraGrid(VOXEL, shape, back_cells=1, height_cell=7)
    pipeline = M7Pipeline(frame_backend, GridSpec((0., 0., 0.), VOXEL, shape),
                          grid_policy=policy)
    fusion = RollingFusion(rolling_backend, voxel_m=VOXEL)
    band = HeightBand(ground_z_m=GROUND_Z, robot_height_m=ROBOT_HEIGHT)

    rclpy.init()
    node = Node("fpga_smoke_scene")
    link = RollingProducerLink(node, fusion)
    tfb = TransformBroadcaster(node)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    truth_pub = node.create_publisher(String, "/smoke/truth", 50)
    state = {"x": START[0], "y": START[1], "yaw": 0.0, "v": 0.0, "w": 0.0, "cmd_t": 0,
             "boxes": {"A": BOX_A}, "frozen": False, "script": None, "frame": 0,
             "blocked": 0,
             "last": mono()}
    log = open(args.out / "scene.jsonl", "w")

    def cmd(msg):
        if state["script"] is None:
            state["v"], state["w"], state["cmd_t"] = msg.linear.x, msg.angular.z, mono()

    def control(msg):
        c = json.loads(msg.data)
        name = c["cmd"]
        if name == "spawn":
            if rect_distance(state["x"], state["y"], tuple(c["box"])) < ROBOT_RADIUS:
                c["refused"] = "box would overlap the robot"
            else:
                state["boxes"][c["id"]] = tuple(c["box"])
        elif name == "remove":
            state["boxes"].pop(c["id"], None)
        elif name == "freeze":
            state["frozen"] = True
        elif name == "unfreeze":
            state["frozen"] = False
        elif name == "drop_next":
            link.drop_next = True
        elif name == "script":
            state["script"] = (float(c["v"]), mono() + int(c["seconds"] * 1e9))
        log.write(json.dumps({"event": "control", "t": mono(), **c}) + "\n")

    node.create_subscription(Twist, "/fpga/cmd_vel_safe", cmd, 10)
    node.create_subscription(String, "/smoke/control", control, 10)

    def physics():
        t = mono()
        dt = (t - state["last"]) / 1e9
        state["last"] = t
        if state["script"] is not None:
            v, until = state["script"]
            if t >= until:
                state["script"] = None
                state["v"] = state["w"] = 0.0
            else:
                state["v"], state["w"] = v, 0.0
        elif t - state["cmd_t"] > 300_000_000:  # the base's own command watchdog
            state["v"] = state["w"] = 0.0
        nx = state["x"] + state["v"] * math.cos(state["yaw"]) * dt
        ny = state["y"] + state["v"] * math.sin(state["yaw"]) * dt
        # Bodies cannot overlap: a translation into a box or wall is refused
        # (and counted), so "the body already overlaps an obstacle" is impossible.
        if any(rect_distance(nx, ny, b) < ROBOT_RADIUS
               for b in WALLS + list(state["boxes"].values())):
            if state["v"] != 0.0:
                state["blocked"] += 1
        else:
            state["x"], state["y"] = nx, ny
        state["yaw"] += state["w"] * dt
        stamp = node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = state["x"], state["y"]
        tf.transform.rotation.z = math.sin(state["yaw"] / 2)
        tf.transform.rotation.w = math.cos(state["yaw"] / 2)
        tfb.sendTransform(tf)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "odom", "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y = state["x"], state["y"]
        od.pose.pose.orientation = tf.transform.rotation
        od.twist.twist.linear.x, od.twist.twist.angular.z = state["v"], state["w"]
        odom_pub.publish(od)

    def frame():
        if state["frozen"]:
            return
        t = mono()
        state["frame"] += 1
        pose = (state["x"], state["y"], state["yaw"])
        boxes = dict(state["boxes"])
        depth, r, c = render(pose, WALLS + list(boxes.values()))
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = r, c
        ident = CaptureIdentity("cam0", state["frame"], t, CLOCK_ID, "cal-smoke", "odom", "e1")
        out = pipeline.ingest_geometry(
            DepthFrame(ident, depth, intrinsics(),
                       PoseAtCapture(t, CLOCK_ID, "odom", "e1", True, transform)),
            now_ns=mono(), now_clock_id=CLOCK_ID)
        fused = fusion.ingest(out.navigation, pipeline, now_ns=mono(), clock_id=CLOCK_ID,
                              robot_position_m=(pose[0], pose[1], 0.0))
        row = {"event": "frame", "t_acq": t, "frame": state["frame"], "pose": pose,
               "boxes": sorted(boxes), "nav_valid": out.navigation.valid,
               "nav_reason": out.navigation.reason, "accepted": fused.accepted,
               "reason": fused.reason}
        if fused.accepted:
            link.publish(fused.delta)
            row.update(epoch=fused.delta.epoch, seq=fused.delta.delta_seq,
                       n_changes=len(fused.delta.changes), t_pub=mono())
            # Diagnostic projection hash; skipped with FPGA_SCENE_NO_SNAPSHOT=1 (N4 load runs:
            # not part of the real producer). A snapshot refused by the producer's own
            # freshness check (heavy load) is recorded instead of crashing the scene.
            if os.environ.get("FPGA_SCENE_NO_SNAPSHOT") != "1":
                snap = fusion.snapshot(now_ns=mono(), clock_id=CLOCK_ID)
                if snap.states is None:
                    row.update(snapshot_refused=snap.reason)
                else:
                    layers = band.band_layers(snap.window_base[2], 32, VOXEL)
                    values = occupancy_values(project_columns(snap.states, layers)).ravel()
                    row.update(grid_sha=hashlib.sha256(values.tobytes()).hexdigest()[:16],
                               base=list(snap.window_base))
        row["link"] = dict(link.counts)
        row["blocked"] = state["blocked"]
        log.write(json.dumps(row) + "\n")
        truth_pub.publish(String(data=json.dumps(row)))

    node.create_timer(0.02, physics)
    node.create_timer(1.0 / FRAME_HZ, frame)
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        log.close()
        node.destroy_node()
        rclpy.shutdown()


# ----------------------------------------------------------------------------- driver
def box_cells(grid_info, box):
    """(index list) of costmap cells whose centre lies inside a box footprint."""
    x0, x1, y0, y1, _ = box
    ox, oy, res = grid_info.origin.position.x, grid_info.origin.position.y, grid_info.resolution
    out = []
    for j in range(grid_info.height):
        for i in range(grid_info.width):
            cx, cy = ox + (i + 0.5) * res, oy + (j + 0.5) * res
            if x0 <= cx <= x1 and y0 <= cy <= y1:
                out.append(j * grid_info.width + i)
    return out


def rect_distance(px, py, box):
    x0, x1, y0, y1, _ = box
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    return math.hypot(dx, dy)


def run_driver(args):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry
    from std_msgs.msg import Bool, String
    from host.rolling_nav_layer import rolling_map_permit

    rclpy.init()
    node = Node("fpga_smoke_driver")
    grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    S = {"status": None, "status_rx": None, "odom": None, "grid": None, "grid_rx": None,
         "local": None, "check": None, "notrack": None, "truth": [], "permit_reason": None,
         "goal_active": False, "local_log": [], "grid_log": [], "odom_log": [],
         "revocations": [], "frozen": False}
    log = open(args.out / "driver.jsonl", "w")

    def emit(event, **fields):
        row = {"event": event, "t": mono(), **fields}
        log.write(json.dumps(row) + "\n")
        log.flush()
        print(json.dumps(row), flush=True)

    def on_status(msg):
        S["status"], S["status_rx"] = json.loads(msg.data), mono()
        S.setdefault("status_log", []).append((S["status"]["stamp_ns"], S["status_rx"],
                                                S["status"].get("age_ns"), S["goal_active"]))

    def on_odom(msg):
        S["odom"] = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.twist.twist.linear.x, msg.twist.twist.angular.z, mono())
        q = msg.pose.pose.orientation
        S["yaw"] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        S["odom_log"].append(S["odom"])
        if S["goal_active"] and creep["leg"] is not None and len(S["odom_log"]) % 5 == 0:
            d = creep_depth(S["odom"][0], S["odom"][1])
            leg = creep["per_leg"].setdefault(creep["leg"], 0.0)
            if d > leg:
                creep["per_leg"][creep["leg"]] = d

    def on_grid(msg):
        S["grid"], S["grid_rx"] = msg, mono()
        S.setdefault("grid_msgs", []).append((mono(), msg))
        del S["grid_msgs"][:-40]
        S["grid_log"].append((mono(), hashlib.sha256(
            np.asarray(msg.data, np.int8).tobytes()).hexdigest()[:16]))

    def on_local(msg):
        S["local"] = msg
        S["local_log"].append((mono(), msg))
        if len(S["local_log"]) > 400:
            S["local_log"].pop(0)

    node.create_subscription(String, "/fpga/rolling_map/status", on_status, 10)
    node.create_subscription(Odometry, "/odom", on_odom, 20)
    node.create_subscription(OccupancyGrid, "/fpga/rolling_map/grid", on_grid, grid_qos)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", on_local, 10)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                             lambda m: S.__setitem__("global", m), 10)
    K = set()             # world cells ever seen FREE in the strict (raw) grid
    creep = {"leg": None, "seed": set(), "max": 0.0, "at": None, "per_leg": {}}

    def on_raw(msg):
        S["raw"] = msg
        d = np.asarray(msg.data, np.int8).reshape(msg.info.height, msg.info.width)
        bx = int(round(msg.info.origin.position.x / msg.info.resolution))
        by = int(round(msg.info.origin.position.y / msg.info.resolution))
        for j, i in np.argwhere(d == 0):
            K.add((bx + int(i), by + int(j)))

    def body_world_cells(x, y):
        r = ROBOT_RADIUS
        out = set()
        for i in range(int(math.floor((x - r) / VOXEL)), int(math.floor((x + r) / VOXEL)) + 1):
            for j in range(int(math.floor((y - r) / VOXEL)), int(math.floor((y + r) / VOXEL)) + 1):
                if ((i + .5) * VOXEL - x) ** 2 + ((j + .5) * VOXEL - y) ** 2 <= r * r:
                    out.add((i, j))
        return out

    def creep_depth(x, y):
        """How far the body outline reaches into cells never seen FREE (0 = none)."""
        known = K | creep["seed"]
        worst = 0.0
        for (i, j) in body_world_cells(x, y) - known:
            best = None
            for ring in range(1, 12):
                for a in range(-ring, ring + 1):
                    for b in (-ring, ring):
                        for c in ((i + a, j + b), (i + b, j + a)):
                            if c in known:
                                dd = math.hypot(c[0] - i, c[1] - j) * VOXEL
                                best = dd if best is None else min(best, dd)
                if best is not None:
                    break
            worst = max(worst, best if best is not None else 12 * VOXEL)
        return worst

    node.create_subscription(OccupancyGrid, "/fpga/rolling_map/grid_raw", on_raw, grid_qos)
    node.create_subscription(OccupancyGrid, "/costmap/costmap",
                             lambda m: S.__setitem__("check", (mono(), m)), 10)
    node.create_subscription(String, "/smoke/truth",
                             lambda m: S["truth"].append(json.loads(m.data)), 50)
    control = node.create_publisher(String, "/smoke/control", 10)
    permit_pub = node.create_publisher(Bool, "/fpga/motion_permit", 1)
    action = ActionClient(node, NavigateToPose, "navigate_to_pose")
    handle = {"goal": None, "result": None}

    def ctl(**c):
        control.publish(String(data=json.dumps(c)))
        emit("control", **c)

    def permit_now():
        ok, reason = rolling_map_permit(S["status"], received_ns=S["status_rx"], now_ns=mono())
        return ok, reason

    def tick():
        ok, reason = permit_now()
        allowed = ok and S["goal_active"]
        permit_pub.publish(Bool(data=allowed))
        if S["goal_active"] and not ok:
            S["goal_active"] = False
            S["revocations"].append((reason, S["frozen"]))
            emit("permit_revoked", reason=reason, status=S["status"], frozen=S["frozen"])
            if handle["goal"] is not None:
                handle["goal"].cancel_goal_async()
        if reason != S["permit_reason"]:
            emit("rolling_permit", ok=ok, reason=reason)
            S["permit_reason"] = reason

    node.create_timer(0.05, tick)

    def spin_for(seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            if until is not None and until():
                return True
        return False

    def send_goal(xy):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "odom"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = xy
        goal.pose.pose.orientation.w = 1.0
        handle["goal"], handle["result"] = None, None
        spin_for(30, action.server_is_ready)  # keep spinning: status must stay fresh
        for attempt in range(30):  # bt_navigator may still be activating
            future = action.send_goal_async(goal)
            spin_for(5, future.done)
            handle["goal"] = future.result()
            if handle["goal"] is not None and handle["goal"].accepted:
                break
            emit("goal_rejected", goal=xy, attempt=attempt)
            spin_for(1.0)
        else:
            return False
        S["goal_active"] = True
        creep["leg"] = "%.2f,%.2f" % tuple(xy)
        creep["seed"] = body_world_cells(S["odom"][0], S["odom"][1])  # body at hand-over
        creep["per_leg"].setdefault(creep["leg"], 0.0)
        res = handle["goal"].get_result_async()
        res.add_done_callback(lambda f: handle.__setitem__("result", f.result()))
        emit("goal_sent", goal=xy)
        return True

    def grid_compare(costmap_msg, grid_msg):
        """Cell-by-cell: check costmap (rolling layer only) vs the bridge grid."""
        a = np.asarray(costmap_msg.data, np.int16).reshape(costmap_msg.info.height,
                                                           costmap_msg.info.width)
        g = np.asarray(grid_msg.data, np.int16).reshape(grid_msg.info.height,
                                                        grid_msg.info.width)
        res = grid_msg.info.resolution
        di = int(round((grid_msg.info.origin.position.x - costmap_msg.info.origin.position.x)
                       / res))
        dj = int(round((grid_msg.info.origin.position.y - costmap_msg.info.origin.position.y)
                       / res))
        rows = {"compared": 0, "mismatch": 0, "by_value": {}, "outside_grid_values": {}}
        for j in range(a.shape[0]):
            for i in range(a.shape[1]):
                gi, gj = i - di, j - dj
                if 0 <= gi < g.shape[1] and 0 <= gj < g.shape[0]:
                    rows["compared"] += 1
                    key = "%d->%d" % (g[gj, gi], a[j, i])
                    rows["by_value"][key] = rows["by_value"].get(key, 0) + 1
                    if g[gj, gi] != a[j, i]:
                        rows["mismatch"] += 1
                else:
                    k = str(int(a[j, i]))
                    rows["outside_grid_values"][k] = rows["outside_grid_values"].get(k, 0) + 1
        return rows

    emit("begin")
    ok = spin_for(60, lambda: S["status"] is not None and S["status"]["fresh"]
                  and (S["local"] is not None or os.environ.get("FPGA_SMOKE_S0") == "1")
                  and S["check"] is not None and S["odom"] is not None)
    emit("ready", ok=ok, status=S["status"])

    def outline_effect(when):
        if S.get("raw") is not None and S["grid"] is not None:
            g = np.asarray(S["grid"].data, np.int8)
            r = np.asarray(S["raw"].data, np.int8)
            emit("body_outline_effect", when=when,
                 unknown_to_free=int(np.sum((r == -1) & (g == 0))),
                 other_differences=int(np.sum((r != g) & ~((r == -1) & (g == 0)))),
                 note="grid vs grid_raw; the body outline covers about 28 cells")
    outline_effect("cold_start")
    if not ok:
        raise SystemExit(3)
    # S6 cold start: goal straight ahead before the car has moved at all.
    # FPGA_SMOKE_CONFIRM_FIRST=1 skips it so the confirmed start (S8) runs on the
    # untouched cold pose (S6's recoveries spin the car in place).
    if os.environ.get("FPGA_SMOKE_S0") != "1" and os.environ.get("FPGA_SMOKE_CONFIRM_FIRST") != "1":
        cold_pose = S["odom"]
        if send_goal((START[0] + 0.6, START[1])):
            spin_for(25, lambda: handle["result"] is not None)
            status = handle["result"].status if handle["result"] is not None else None
            if handle["result"] is None and handle["goal"] is not None:
                handle["goal"].cancel_goal_async()
                spin_for(3, lambda: handle["result"] is not None)
            S["goal_active"] = False
            emit("S6_cold_start_goal", status=status, moved_m=math.hypot(
                S["odom"][0] - cold_pose[0], S["odom"][1] - cold_pose[1]))
        spin_for(1.0)
        if math.hypot(S["odom"][0] - START[0], S["odom"][1] - START[1]) > 0.02:
            emit("S6_note", note="robot moved during cold start; warm-up continues from here",
                 odom=S["odom"])
    # S8 cold start WITH the operator confirmation (second owner ruling): the
    # same start pose, the real CLI, then the same goal. S6 above is the
    # unconfirmed control and must not have moved.
    if os.environ.get("FPGA_SMOKE_S0") != "1":
        import subprocess
        spin_for(1.0)
        before = S["odom"]
        env = dict(os.environ, PYTHONPATH=str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""))
        cli = [sys.executable, str(ROOT / "tools/cold_start_confirm.py"), "--operator",
               "smoke-driver"]
        first = subprocess.run(cli, capture_output=True, text=True, env=env, timeout=20)
        spin_for(0.5)
        outline_effect("cold_start_confirmed")
        emit("S8_confirm", rc=first.returncode, answer=first.stdout.strip()[-400:])
        spin_for(1.0)
        for name, msg in (("bridge_grid", S["grid"]), ("global", S.get("global")),
                          ("local", S["local"])):
            if msg is not None:
                np.savez(args.out / ("S8_%s.npz" % name),
                         data=np.asarray(msg.data, np.int16).reshape(msg.info.height,
                                                                    msg.info.width),
                         origin=[msg.info.origin.position.x, msg.info.origin.position.y],
                         odom=list(S["odom"]))
        # The unconfirmed attempt's recoveries may have turned the car in place; the
        # operator confirms what is in front of it now, so the goal lies straight ahead.
        yaw = S.get("yaw", 0.0)
        goal8 = (before[0] + 0.5 * math.cos(yaw), before[1] + 0.5 * math.sin(yaw))
        attempts, status = 0, None
        while attempts < 3 and status != 4:
            attempts += 1
            spin_for(5, lambda: permit_now()[0])  # re-send only on a fresh map again
            if not send_goal(goal8):
                break
            spin_for(40, lambda: handle["result"] is not None)
            status = handle["result"].status if handle["result"] is not None else None
            if handle["result"] is None and handle["goal"] is not None:
                handle["goal"].cancel_goal_async()
                spin_for(3, lambda: handle["result"] is not None)
            S["goal_active"] = False
            if status != 4 and not (S["revocations"] and S["revocations"][-1][0] ==
                                    "rolling_map_stale"):
                break  # only a false stale-stop is retried (task layer re-sends)
        emit("S8_cold_start_confirmed_goal", status=status, attempts=attempts, goal=goal8,
             yaw=yaw, moved_m=math.hypot(S["odom"][0] - before[0], S["odom"][1] - before[1]),
             final=S["odom"], ring_state=S["status"].get("cold_start"))
        again = subprocess.run(cli, capture_output=True, text=True, env=env, timeout=20)
        emit("S8_second_confirm", rc=again.returncode, answer=again.stdout.strip()[-400:])
        spin_for(1.0)
    # Warm-up: scripted drive to x = 0 (not Nav2): the map ahead exists
    # only because the car came from behind. Cold start is reported separately.
    cold = S["local"] if S["local"] is not None else S["check"][1]
    remaining = -S["odom"][0]
    if remaining > 0.01:
        ctl(cmd="script", v=0.15, seconds=remaining / 0.15)
        spin_for(remaining / 0.15 + 1.0)
    emit("warmup_done", odom=S["odom"])
    spin_for(1.0)

    # S1 (or S0 with FPGA_SMOKE_S0=1): cell-by-cell, robot standing still.
    label = "S0_notrack_vs_bridge" if os.environ.get("FPGA_SMOKE_S0") == "1" \
        else "S1_check_costmap_vs_bridge"
    # The check costmap lags the bridge by up to one update period, so compare
    # it with each of the last bridge grids received before it; report the best.
    S["grid_msgs"] = []
    spin_for(2.0)
    t_check, check = S["check"]
    candidates = [(t, g) for t, g in S["grid_msgs"] if t < t_check][-10:]
    results = [(grid_compare(check, g), len(candidates) - k) for k, (t, g) in
               enumerate(candidates)]
    best, lag = min(results, key=lambda r: r[0]["mismatch"])
    emit(label, grids_considered=len(results), best_lag_grids=lag,
         distinct_contents=len({hashlib.sha256(bytes(np.asarray(g.data, np.int8).tobytes()))
                                .hexdigest() for _, g in candidates}), **best)
    outline_effect("after_warmup")
    cold_unknown = sum(1 for v in cold.data if v == -1)
    emit("cold_start_local_costmap", unknown=cold_unknown, cells=len(cold.data))
    if os.environ.get("FPGA_SMOKE_S0") == "1":
        emit("end_s0")
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        return

    # S2: pop-up obstacle latency (robot still), 5 repetitions.
    lat = []
    for rep in range(5):
        spin_for(0.5)
        S["truth"].clear()
        S["grid_msgs"] = []
        ctl(cmd="spawn", id="B", box=list(BOX_B))
        t_spawn = mono()
        first = {}

        def seen():
            for row in S["truth"]:
                if "B" in row.get("boxes", []) and "acq" not in first:
                    first["acq"] = row["t_acq"]
            if S["local"] is not None and "acq" in first:
                t_rx, msg = S["local_log"][-1]
                cells = box_cells(msg.info, BOX_B)
                if t_rx > first["acq"] and any(msg.data[c] == 100 for c in cells):
                    first["local"] = t_rx
                    return True
            return False

        spin_for(3.0, seen)
        grid_t = None
        for t_rx, g in S.get("grid_msgs", []):
            if "acq" in first and t_rx > first["acq"] and \
                    any(g.data[c] == 100 for c in box_cells(g.info, BOX_B)):
                grid_t = t_rx
                break
        frames_to_lethal = None
        if "local" in first:
            frames_to_lethal = sum(1 for r in S["truth"] if "B" in r.get("boxes", []) and
                                   r["t_acq"] <= first["local"])
            lat.append((first["local"] - first["acq"]) / 1e6)
        emit("S2_popup", rep=rep, spawn_to_first_frame_ms=(first.get("acq", 0) - t_spawn) / 1e6,
             acq_to_local_costmap_lethal_ms=(first["local"] - first["acq"]) / 1e6
             if "local" in first else None, frames_with_box_until_lethal=frames_to_lethal,
             acq_to_bridge_grid_lethal_ms=(grid_t - first["acq"]) / 1e6 if grid_t else None,
             deciding_frame_acq_to_local_lethal_ms=(
                 first["local"] - max(r["t_acq"] for r in S["truth"]
                                      if "B" in r.get("boxes", []) and
                                      r["t_acq"] <= first["local"])) / 1e6
             if "local" in first else None)
        ctl(cmd="remove", id="B")
        t_remove = mono()

        def cleared():
            msg = S["local"]
            cells = box_cells(msg.info, BOX_B)
            return S["local_log"][-1][0] > t_remove and all(msg.data[c] != 100 for c in cells)

        ok = spin_for(4.0, cleared)
        emit("S2_clear", rep=rep, cleared=ok, remove_to_not_lethal_ms=(mono() - t_remove) / 1e6
             if ok else None)
    emit("S2_summary", latencies_ms=lat, median_ms=float(np.median(lat)) if lat else None,
         max_ms=max(lat) if lat else None)

    # S3: navigate around box A.
    min_clear = {"v": 9.0}
    S["odom_log"].clear()
    send_pose = S["odom"]
    if send_goal(GOAL_1):
        def done():
            if S["odom"] is not None:
                min_clear["v"] = min(min_clear["v"], rect_distance(S["odom"][0], S["odom"][1],
                                                                   BOX_A))
            return handle["result"] is not None
        spin_for(90, done)
        S["goal_active"] = False
        status = handle["result"].status if handle["result"] is not None else None
        path = [(round(o[0], 3), round(o[1], 3)) for o in S["odom_log"][::10]]
        sx, sy = send_pose[0], send_pose[1]
        straight_hits = min(rect_distance(sx + a * (GOAL_1[0] - sx), sy + a * (GOAL_1[1] - sy),
                                          BOX_A) for a in np.linspace(0, 1, 201))
        emit("S3_navigate", status=status, final=S["odom"], min_center_to_boxA_m=min_clear["v"],
             min_body_gap_to_boxA_m=min_clear["v"] - ROBOT_RADIUS,
             goal_error_m=math.hypot(S["odom"][0] - GOAL_1[0], S["odom"][1] - GOAL_1[1]),
             robot_radius_m=ROBOT_RADIUS, path=path,
             straight_line_min_dist_to_boxA_m=straight_hits)

    # S4: stale map during motion -> permit revoked, goal cancelled, robot stops.
    spin_for(1.0)
    S["truth"].clear()
    if send_goal(GOAL_2):
        start = S["odom"]
        spin_for(30, lambda: S["odom"] is not None and abs(S["odom"][2]) > 0.05 and
                 math.hypot(S["odom"][0] - start[0], S["odom"][1] - start[1]) > 0.10)
        moving = S["odom"]
        S["frozen"] = True
        ctl(cmd="freeze")
        t_freeze = mono()
        revoked = {}

        def stopped():
            if "t" not in revoked and not S["goal_active"]:
                revoked["t"] = mono()
            return "t" in revoked and S["odom"] is not None and S["odom"][2] == 0.0 \
                and S["odom"][3] == 0.0
        ok = spin_for(5, stopped)
        t_stop = mono()
        last_acq = max((r["t_acq"] for r in S["truth"] if r.get("accepted")), default=None)
        stop_pose = S["odom"]
        spin_for(1.0)
        after = S["odom"]
        emit("S4_stale_stop", stopped=ok, speed_before=moving[2],
             last_delta_acq_to_revoke_ms=(revoked["t"] - last_acq) / 1e6
             if last_acq and "t" in revoked else None,
             freeze_to_revoke_ms=(revoked["t"] - t_freeze) / 1e6 if "t" in revoked else None,
             last_delta_acq_to_zero_speed_ms=(t_stop - last_acq) / 1e6 if last_acq else None,
             travel_after_freeze_m=math.hypot(stop_pose[0] - moving[0], stop_pose[1] - moving[1]),
             drift_in_next_1s_m=math.hypot(after[0] - stop_pose[0], after[1] - stop_pose[1]),
             goal_result_status=handle["result"].status if handle["result"] else None)
        ctl(cmd="unfreeze")
        S["frozen"] = False
        spin_for(1.5)
        emit("S4_after_unfreeze", status=S["status"])

    # S5: drop one delta -> gap -> snapshot resync -> grid equals producer projection.
    S["truth"].clear()
    ctl(cmd="drop_next")
    spin_for(2.0)
    truth_by_sha = {r.get("grid_sha") for r in S["truth"] if r.get("accepted")}
    latest = [r for r in S["truth"] if r.get("accepted")][-1]
    spin_for(0.3)
    sha_now = S["grid_log"][-1][1]
    emit("S5_gap_resync", bridge_counts=S["status"].get("counts"), status=S["status"],
         latest_truth_seq=latest.get("seq"),
         bridge_grid_equals_a_producer_projection=sha_now in truth_by_sha)
    permit_pub.publish(Bool(data=False))
    st = S.get("status_log", [])
    pub_iv = np.diff([r[0] for r in st]) / 1e6
    delay = np.array([(r[1] - r[0]) / 1e6 for r in st])
    ages = np.array([r[2] / 1e6 for r in st if r[2] is not None])
    pct = lambda a: [round(float(np.percentile(a, q)), 2) for q in (50, 99)] + \
        [round(float(a.max()), 2)] if len(a) else None  # noqa: E731
    blocked = max((r.get("blocked", 0) for r in S["truth"]), default=None)
    emit("ruling_summary", creep_depth_m_per_leg=creep["per_leg"],
         false_stops=[r for r, frozen in S["revocations"] if not frozen],
         stops_during_freeze=[r for r, frozen in S["revocations"] if frozen],
         physics_blocked_steps_last_seen=blocked)
    emit("status_timing", n=len(st), bridge_publish_interval_ms_p50_p99_max=pct(pub_iv),
         bridge_to_driver_delay_ms_p50_p99_max=pct(delay),
         map_age_at_bridge_ms_p50_p99_max=pct(ages))
    emit("end")
    log.close()
    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("role", choices=("scene", "driver", "params"))
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--seconds", type=float, default=240.0)
    args = ap.parse_args()
    if args.role == "params":
        write_params(args.out, s0=os.environ.get("FPGA_SMOKE_S0") == "1")
        return
    args.out.mkdir(parents=True, exist_ok=True)
    {"scene": run_scene, "driver": run_driver}[args.role](args)


if __name__ == "__main__":
    main()
