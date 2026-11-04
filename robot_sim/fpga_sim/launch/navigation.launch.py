"""AGX ROS process: known-map Nav2 with explicitly simulated reference pose."""
from copy import deepcopy
from pathlib import Path
import tempfile
import os
import yaml
import json
import hashlib
import runpy
from host.nav_recovery import bounded_recovery

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, GroupAction, ExecuteProcess
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


def merge(base, overlay):
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def force_sim_time(value, enabled=True):
    if isinstance(value, dict):
        if "ros__parameters" in value:
            value["ros__parameters"]["use_sim_time"] = enabled
        for child in value.values():
            force_sim_time(child, enabled)


# use_sim_time of the static transform publishers; generate_launch_description() sets it from FPGA_NAV_USE_SIM_TIME
# (the static_tf(...) calls below must stay literal: robot_sim/tests/test_assets.py and tests/test_sim_loop_d1.py
# read them with ast.literal_eval)
STATIC_TF_SIM_TIME = [True]


def static_tf(name, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return Node(
        package="tf2_ros", executable="static_transform_publisher", name=name,
        arguments=["--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                   "--roll", str(rpy[0]), "--pitch", str(rpy[1]), "--yaw", str(rpy[2]),
                   "--frame-id", parent, "--child-frame-id", child],
        parameters=[{"use_sim_time": STATIC_TF_SIM_TIME[0]}],
    )


def package_share():
    """fpga_sim's share directory; the board image has no fpga_sim package (PackageNotFoundError, a KeyError) and
    starts this file by path -- then this launch directory's parent, the source tree's robot_sim/fpga_sim."""
    try:
        return Path(get_package_share_directory("fpga_sim"))
    except KeyError:
        return Path(__file__).resolve().parents[1]


def velocity_guard_script(share):
    for candidate in (share / "scripts/velocity_guard.py", share.parents[1] / "lib/fpga_sim/velocity_guard.py"):
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("velocity_guard.py not found next to %s" % share)


def car_rotation_shim(environ):
    """FPGA_NAV_ROTATION_SHIM (car only, default true): wrap the car's DWB in the rotation shim; false = DWB alone."""
    value = environ.get("FPGA_NAV_ROTATION_SHIM", "true").lower()
    if value not in ("true", "false"):
        raise ValueError("FPGA_NAV_ROTATION_SHIM must be true or false")
    return value == "true"


def follow_path(params):
    """controller_server FollowPath block of a parsed parameter file ({} when absent)."""
    return (((params or {}).get("controller_server") or {}).get("ros__parameters") or {}).get("FollowPath") or {}


def rolling_costmap(share, inputs, defaults, overrides, params):
    """FPGA_NAV_COSTMAP=rolling: the rolling-map local costmap + DWB (nav_input_profile.apply_rolling_costmap),
    refused unless host/rolling_nav_layer.check_costmap_config (the 09-27 owner ruling) finds nothing."""
    from host.rolling_nav_layer import check_costmap_config
    source = share / "config/rolling_layer_nav2.yaml"
    with source.open() as stream:
        rolling = yaml.safe_load(stream)
    with (share / "config/nav2_overrides.yaml").open() as stream:
        formal = yaml.safe_load(stream)
    result, evidence = inputs['apply_rolling_costmap'](params, rolling, follow_path(defaults), follow_path(overrides),
                                                       follow_path(formal))
    follow = result["controller_server"]["ros__parameters"]["FollowPath"]
    problems = check_costmap_config(result["local_costmap"]["local_costmap"]["ros__parameters"], follow_path=follow,
                                    planner=result["planner_server"]["ros__parameters"]["GridBased"])
    if follow.get("plugin") != inputs['DWB_PLUGIN']:
        problems.append("FollowPath plugin must be %s" % inputs['DWB_PLUGIN'])
    if problems:
        raise ValueError("FPGA_NAV_COSTMAP=rolling refused by check_costmap_config: %s" % problems)
    evidence["source"] = str(source)
    return result, evidence


def generate_launch_description():
    share = package_share()
    inputs = runpy.run_path(str(share / "config/nav_input_profile.py"))
    profile, autostart, map_yaml = inputs['navigation_inputs'](os.environ, share / "maps/room.yaml")
    car = inputs['car_inputs'](os.environ, profile)
    sim_time = car["use_sim_time"]
    STATIC_TF_SIM_TIME[0] = sim_time
    nav_share = Path(get_package_share_directory("nav2_bringup"))
    with (nav_share / "params/nav2_params.yaml").open() as stream:
        defaults = yaml.safe_load(stream)
    override_file = Path(os.environ.get("FPGA_NAV2_OVERRIDES", str(share / "config/nav2_overrides.yaml")))
    with override_file.open() as stream:
        overrides = yaml.safe_load(stream)
    params = merge(defaults, overrides)
    rolling_evidence = None
    if car["costmap"] == "rolling":   # before force_sim_time and the speed limits, which then cover its blocks too
        params, rolling_evidence = rolling_costmap(share, inputs, defaults, overrides, params)
    bt_source = Path(get_package_share_directory("nav2_bt_navigator")) / "behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml"
    original_tree = bt_source.read_text()
    effective_tree = bounded_recovery(original_tree)
    with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", prefix="fpga_nav_bt_", delete=False) as stream:
        stream.write(effective_tree)
        bt_file = stream.name
    params["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = bt_file
    print(json.dumps({"navigation_bt_source": str(bt_source),
                      "source_sha256": hashlib.sha256(original_tree.encode()).hexdigest(),
                      "effective_xml": effective_tree, "outer_recovery_retries": 1,
                      "costmap_recovery": "retain_observations_wait_1s",
                      "detour_observation": {"near_path_m": 3.0, "length_factor": 1.5, "wait_s": 5,
                                             "stationary_sweep_rad": [0.45, -0.9, 0.45]}}), flush=True)
    force_sim_time(params, sim_time)
    rotation_shim = car["speed_limits"] == "car" and car_rotation_shim(os.environ)
    if rotation_shim:
        # 2026-10-22 find-nav dig: DWB alone never turns towards a path 120-175 deg behind the car (board nav1/nav3);
        # the shim turns in place first, then the same DWB drives. Before the speed limits, which cap its rate/accel.
        params = inputs['apply_rotation_shim'](params)
    progress_checker = planner = None   # the simulation's progress checker and planner stay as merged
    forward_prune = forward_prune_given = None
    min_speeds, min_speed_override = inputs['CAR_DWB_MIN_SPEEDS'], {}
    if car["speed_limits"] == "car":
        params = inputs['apply_speed_limits'](params, inputs['CAR_SPEED_LIMITS'])
        # owner 10-15: the car's xy goal tolerance is 0.10 m (0.03 stopped 4.7-4.8 cm short, n2ctl1/2); after the
        # rolling merge, so its rebuilt DWB FollowPath is covered too; the simulation keeps 0.03
        params = inputs['apply_car_goal_tolerance'](params, inputs['CAR_XY_GOAL_TOLERANCE'])
        # 2026-10-22: yaw 0.03 sits inside DWB's +-0.021 rad/s stall band on the car (CAR_YAW_GOAL_TOLERANCE note)
        params = inputs['apply_car_yaw_goal_tolerance'](params, inputs['CAR_YAW_GOAL_TOLERANCE'])
        # 2026-10-25: progress = 0.05 m or 0.50 rad in 20 s (PoseProgressChecker; FPGA_NAV_PROGRESS_CHECKER=simple
        # falls back); NavFn stays at rolling_layer_nav2.yaml's 0 / false unless FPGA_NAV_CAR_PLANNER=relaxed (0.25 /
        # allow_unknown true, applied after rolling_costmap()'s check_costmap_config gate; owner to decide)
        # 2026-10-25 audit: no DWB command inside the chassis dead band (CAR_DWB_MIN_SPEEDS note); after the shim
        # 2026-10-26: FPGA_NAV_DWB_MIN_SPEED_THETA / _XY / FPGA_NAV_ROT_STOPPED_VELOCITY override the constants (car only;
        # unset = the constants, the call of before); refused values raise before any node starts
        min_speeds, rot_stopped, min_speed_override = inputs['car_min_speed_overrides'](os.environ)
        params = inputs['apply_car_min_speeds'](params, min_speeds, rot_stopped)
        # 2026-10-29: DWB's GoalDist / GoalAlign target a point 0.8 m ahead on the path, not a goal within 2 m that
        # the path first leads away from (nav_input_profile CAR_DWB_FORWARD_PRUNE_M; FPGA_NAV_DWB_FORWARD_PRUNE_M=2.0 =
        # Nav2's default, the behaviour before); a refused value raises before any node starts
        forward_prune, forward_prune_given = inputs['car_forward_prune'](os.environ)
        params = inputs['apply_car_forward_prune'](params, forward_prune)
        progress_checker = inputs['car_progress_checker'](os.environ)
        planner = inputs['car_planner'](os.environ)
        params = inputs['apply_car_progress_checker'](params, progress_checker)
        params = inputs['apply_car_planner'](params, planner)
    # 2026-10-29 (car_inner, mapping run, CAR_NAV_LIVE_GLOBAL): the global static_layer reads the live global map
    global_topic = inputs['global_map_topic'](os.environ)
    params = inputs['apply_global_map_topic'](params, global_topic)
    params["map_server"]["ros__parameters"]["yaml_filename"] = map_yaml
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", prefix="fpga_nav2_", delete=False) as stream:
        yaml.safe_dump(params, stream)
        params_file = stream.name

    def cleanup(context):
        Path(params_file).unlink(missing_ok=True)
        Path(bt_file).unlink(missing_ok=True)
        return []

    reference_transforms = [
        # Identity valid only for reference pose and this world's origin.
        static_tf("reference_map_to_odom", "map", "odom"),
    ] if car["static_map_odom"] else []
    reference_transforms += [
        # Car mount (worlds/room_carmount.sdf rgbd sensor pose; config/extrinsics_sim.json).
        static_tf("base_to_camera", "base_link", "camera_link", xyz=(0.13, 0, 0.35), rpy=(0, 0.471238898, 0)),
        static_tf("camera_to_optical", "camera_link", "camera_optical_frame",
                  rpy=(-1.5707963267948966, 0, -1.5707963267948966)),
    ] if car["sim_camera_tf"] else []
    print(json.dumps({"input_profile": profile, "map_yaml": map_yaml,
                      "autostart": autostart, "static_extrinsics_source":
                      "simulation" if car["sim_camera_tf"] else "external"}), flush=True)
    if global_topic:   # only when given, so the default output stays as before
        print(json.dumps({"global_map_topic": global_topic}), flush=True)
    guard = []
    if car != {"use_sim_time": True, "static_map_odom": profile == "reference", "sim_camera_tf": profile == "reference",
               "speed_limits": "sim", "velocity_guard": False, "costmap": "sim"}:   # the simulation's output stays byte-identical
        print(json.dumps(dict({"real_car": car, "speed_limits": inputs['CAR_SPEED_LIMITS']
                               if car["speed_limits"] == "car" else None,
                               "rotation_shim": inputs['CAR_ROTATION_SHIM'] if rotation_shim else None,
                               "yaw_goal_tolerance": inputs['CAR_YAW_GOAL_TOLERANCE']
                               if car["speed_limits"] == "car" else None,
                               "progress_checker": progress_checker, "planner": planner,
                               "dwb_min_speeds": min_speeds if car["speed_limits"] == "car"
                               and inputs['follow_path_is_dwb'](follow_path(params)) else None},
                              **({"rolling_costmap": rolling_evidence} if rolling_evidence else {}),
                              # 2026-10-29: the car's DWB forward_prune_distance (with "given" when overridden)
                              **({"dwb_forward_prune_m": forward_prune} if forward_prune is not None
                                 and inputs['follow_path_is_dwb'](follow_path(params)) else {}),
                              **({"dwb_forward_prune_given": True} if forward_prune_given else {}),
                              # 2026-10-26: only when an override was given, so the default line stays as before
                              **({"dwb_min_speed_override": dict(min_speed_override, effective=dict(
                                  min_speeds, rot_stopped_velocity=rot_stopped))} if min_speed_override else {}))),
              flush=True)
    if car["velocity_guard"]:
        # /cmd_vel (the velocity smoother's output) + /fpga/motion_permit -> /fpga/cmd_vel_safe, zero after 0.5 s
        # without a fresh command or permit (wall clock). Started by path: the board has no fpga_sim package.
        # (an empty "-p estop_topic:=" is refused by rcl: the e-stop input is passed only when given)
        estop = os.environ.get("FPGA_NAV_GUARD_ESTOP_TOPIC", "")
        # FPGA_NAV_GUARD_OUTPUT (2026-10-18, teleop arbitration; unset = /fpga/cmd_vel_safe as before): the arbitration
        # mode publishes on /fpga/nav/cmd_vel_safe and host/ros_teleop_safe.py --nav-topic passes it on
        output = os.environ.get("FPGA_NAV_GUARD_OUTPUT", "")
        params = (["-p", "estop_topic:=" + estop] if estop else []) + (["-p", "output_topic:=" + output] if output else [])
        guard = [ExecuteProcess(
            cmd=["python3", str(velocity_guard_script(share))]
                + (["--ros-args"] + params if params else []),
            name="velocity_guard", output="screen")]
    # CTRLPS2 (2026-11-03, car_up CAR_NAV_BACKEND=kv): the KV260 drives (host/ros_kv_nav_proxy.py serves
    # navigate_to_pose and publishes /cmd_vel); of Nav2 only the G1 global route is kept -- planner_server (its global
    # costmap inside) under lifecycle_manager_navigation; map_server and the velocity guard as always. Unset or nav2:
    # the launch description below, exactly as before.
    backend = os.environ.get("FPGA_NAV_BACKEND", "nav2") or "nav2"
    if backend not in ("nav2", "kv"):
        raise ValueError("FPGA_NAV_BACKEND must be nav2 or kv (shadow runs the full Nav2: nav2)")
    if backend == "kv":
        print(json.dumps({"nav_backend": "kv", "nav2_nodes": ["map_server", "planner_server"]}), flush=True)
        nav_nodes = [
            Node(package="nav2_planner", executable="planner_server", name="planner_server",
                 parameters=[params_file], output="screen"),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_navigation",
                 parameters=[{"use_sim_time": sim_time, "autostart": autostart == "true",
                              "node_names": ["planner_server"]}], output="screen"),
        ]
    return LaunchDescription(reference_transforms + guard + [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[params_file], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_map",
             parameters=[{"use_sim_time": sim_time, "autostart": autostart == "true",
                          "node_names": ["map_server"]}], output="screen"),
        *(nav_nodes if backend == "kv" else []),
        *([] if backend == "kv" else [GroupAction([
            *([SetRemap(src='cmd_vel', dst='/fpga/inspection/cmd_vel'),
               SetRemap(src='cmd_vel_nav', dst='/fpga/inspection/cmd_vel'),
               SetRemap(src='behavior_server:cmd_vel', dst='/fpga/inspection/cmd_vel')]
              if profile == 'estimated_inspect' else
              [SetRemap(src='behavior_server:cmd_vel', dst='cmd_vel_nav')]),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / "launch/navigation_launch.py")),
            launch_arguments={"use_sim_time": "true" if sim_time else "false", "autostart": autostart,
                              "params_file": params_file, "use_composition": "False"}.items(),
        ),
        ])]),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
    ])
