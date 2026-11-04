"""Input ownership for reference navigation and motionless estimated inspection.

The inspection profile is deliberately not an estimated navigation runner.
Localization leases and map-frame goal verification must precede that runner.
"""
from copy import deepcopy
from pathlib import Path


def input_profile(environ):
    profile = environ.get('FPGA_NAV_INPUT_PROFILE', 'reference')
    if profile not in ('reference', 'estimated_inspect'):
        raise ValueError('FPGA_NAV_INPUT_PROFILE must be reference or estimated_inspect')
    return profile


def navigation_inputs(environ, reference_map):
    profile = input_profile(environ)
    autostart = environ.get('FPGA_NAV_AUTOSTART', 'true').lower()
    if autostart not in ('true', 'false'):
        raise ValueError('FPGA_NAV_AUTOSTART must be true or false')
    if profile == 'estimated_inspect':
        if autostart != 'false':
            raise ValueError('estimated_inspect requires FPGA_NAV_AUTOSTART=false')
        map_path = environ.get('FPGA_NAV_MAP_YAML')
        if not map_path or not Path(map_path).is_file():
            raise ValueError('estimated_inspect requires an existing FPGA_NAV_MAP_YAML')
    else:
        # real car (10-15): the reference profile takes FPGA_NAV_MAP_YAML too (rtabmap-export --map of the car's
        # database); unset = the simulation room as before, set but missing = refused
        map_path = environ.get('FPGA_NAV_MAP_YAML') or reference_map
        if environ.get('FPGA_NAV_MAP_YAML') and not Path(map_path).is_file():
            raise ValueError('FPGA_NAV_MAP_YAML does not exist: %s' % map_path)
    return profile, autostart, str(Path(map_path).resolve())


# Real-car speed limits (10-15): the first real-car Nav2 runs. The wheel parameters are placeholders, not calibrated
# (docs/team car runbook step 7); ChassisLink clamps at +-500 mm/s and the e-stop chain takes about 10 ms -- so the
# car runs at about half the simulation's speeds, with gentle accelerations.
CAR_SPEED_LIMITS = {'linear': 0.15, 'angular': 0.4, 'linear_accel': 0.3, 'angular_accel': 0.6}
# Real-car xy goal tolerance (m): owner 10-15 定 0.10；0.03 下宿主停在 4.7-4.8 cm 失败（n2ctl1/2: the car stopped
# 4.7-4.8 cm short, "Failed to make progress"; 0.10 arrived in 6.5 s, n2ctl3_dwb_xytol010). The simulation keeps
# config/nav2_overrides.yaml's 0.03 (camera framing). Yaw tolerance unchanged.
CAR_XY_GOAL_TOLERANCE = 0.10
# Real-car yaw goal tolerance (rad), 2026-10-22 find-nav dig (runs/20261022-153440-live nav2): DWB's RotateToGoal picks
# the in-place rate whose 1.7 s end yaw lands nearest the goal yaw, on the 20-sample grid +-0.021, +-0.063, ... rad/s.
# Once the remaining error is below ~0.07 rad it picks +-0.021 rad/s, which the chassis does not turn at (odom flat
# for 5-8 s on the board, every navigation of that run), so 0.03 can never be reached and only the wall timeout ends
# the goal. 0.10 rad (5.7 deg, ~40 px of the 640 px colour image) leaves margin above that stall band; the simulation
# keeps nav2_overrides.yaml's 0.03.
CAR_YAW_GOAL_TOLERANCE = 0.10
# Real-car rotation shim (2026-10-22 find-nav dig): with the heading 120-175 deg off the path DWB's PathAlign/GoalAlign
# (forward point 0.1 m = two 5 cm cells) score every in-place rate within one cell step, so DWB settles on +-0.021 rad/s
# and the car never turns towards the path (board nav1/nav3; reproduced on an all-free costmap). Nav2 Humble's
# RotationShimController turns in place towards the path first (beyond angular_dist_threshold), then hands the same
# DWB (primary_controller, same FollowPath.* parameters, still the only controller that translates) the path.
ROTATION_SHIM_PLUGIN = 'nav2_rotation_shim_controller::RotationShimController'
CAR_ROTATION_SHIM = {'angular_dist_threshold': 0.785, 'forward_sampling_distance': 0.5,
                     'rotate_to_heading_angular_vel': 0.4, 'max_angular_accel': 0.6, 'simulate_ahead_time': 1.0}

# Real-car DWB minimum speeds (2026-10-25 find-nav audit, board only). Three of the four 10-22 failures: DWB's best
# trajectory turned at +-0.021 rad/s (and drove at 0.008 m/s) -- below the chassis dead band (turned at 45-105
# mrad/s, not at 21), so the car stood still with a valid-looking command. DWB samples the full [-0.4, 0.4] rad/s
# (20 samples: +-0.021, +-0.063, +-0.105, ...; acc_lim x sim_time 1.7 s covers the range) and [0, 0.15] m/s (0.0079
# steps). Humble KinematicParameters::isValidSpeed rejects a sample only when BOTH speeds are low (|v| < min_speed_xy
# AND |w| < min_speed_theta), and v = w = 0: with 0.015 / 0.10 the slow samples that remain either translate at
# >= 0.0158 m/s or turn at >= 0.105 rad/s, so no legal command sits in the turn dead band (in-place turns, incl. the
# end-of-path RotateToGoal phase where only v = 0 is legal, run at >= 0.105 rad/s; the rotation shim covers only the
# start). DWB then can never command a stop by itself -- the goal checker must declare "stopped" while it still
# turns at the lowest legal rate: StoppedGoalChecker rot_stopped_velocity 0.15 (> 0.105 + odometry noise; the
# simulation's 0.03 would never be met and the car would turn through the 0.10 rad yaw window). The overshoot after
# the zero command is ~0.105^2 / (2 x 0.6) = 0.01 rad.
# min_speed_xy 0.015, not the audit's 0.03: it removes the 0.0079 m/s step (the 10-22 best straight speed) but keeps
# 0.0158 / 0.0237 -- the cold-start crawl (CAM_TILT_10_20261017.md 4.3: first feasible straight 0.024 m/s; the
# camera's nearest floor is ~0.64 m from the body centre, the body seed ends at 0.35 m) must stay legal: in the
# 10-22 harness's camera mode (unseen ring 0.35-0.65 m) nav3 fails with 0.03 or 0.02 and arrives with 0.015, crawling
# at 0.0158-0.047 m/s for ~0.3 m, then 0.15 m/s. The chassis' linear dead band is not measured (board check).
CAR_DWB_MIN_SPEEDS = {'min_speed_xy': 0.015, 'min_speed_theta': 0.10}
CAR_ROT_STOPPED_VELOCITY = 0.15
STOPPED_GOAL_CHECKER = 'nav2_controller::StoppedGoalChecker'
# 2026-10-26 on-site override (car only; car_up CAR_DWB_MIN_SPEED_THETA / CAR_DWB_MIN_SPEED_XY /
# CAR_ROT_STOPPED_VELOCITY -> car_inner -> these launch variables): the chassis dead band is measured on the day; a
# value outside the constants above must not need a code change and a redeploy. Empty / unset = the constant. Given:
# a positive number <= the cap, and the effective rot_stopped_velocity must stay ABOVE the effective min_speed_theta
# (else DWB's slowest legal turn never counts as stopped and the car turns through the yaw window, note above).
CAR_MIN_SPEED_ENV = (('FPGA_NAV_DWB_MIN_SPEED_THETA', 'min_speed_theta', 0.4),
                     ('FPGA_NAV_DWB_MIN_SPEED_XY', 'min_speed_xy', 0.1),
                     ('FPGA_NAV_ROT_STOPPED_VELOCITY', 'rot_stopped_velocity', 0.4))


def car_min_speed_overrides(environ):
    """(min_speeds, rot_stopped, overridden): the effective CAR_DWB_MIN_SPEEDS dict, rot_stopped_velocity and the
    {key: value} actually given through CAR_MIN_SPEED_ENV ({} = all constants, the 10-25 values). ValueError on a
    non-number, a value <= 0 or above the cap, or rot_stopped_velocity <= min_speed_theta."""
    effective = dict(CAR_DWB_MIN_SPEEDS, rot_stopped_velocity=CAR_ROT_STOPPED_VELOCITY)
    overridden = {}
    for name, key, cap in CAR_MIN_SPEED_ENV:
        raw = (environ.get(name) or '').strip()
        if not raw:
            continue
        try:
            value = float(raw)
        except ValueError:
            raise ValueError('%s must be a number (given %r)' % (name, raw))
        if not 0.0 < value <= cap:   # NaN fails too
            raise ValueError('%s must be > 0 and <= %g (given %r)' % (name, cap, raw))
        effective[key] = overridden[key] = value
    if not effective['rot_stopped_velocity'] > effective['min_speed_theta']:
        raise ValueError('rot_stopped_velocity %g must be > min_speed_theta %g (FPGA_NAV_ROT_STOPPED_VELOCITY / '
                         'FPGA_NAV_DWB_MIN_SPEED_THETA)' % (effective['rot_stopped_velocity'],
                                                           effective['min_speed_theta']))
    rot_stopped = effective.pop('rot_stopped_velocity')
    return effective, rot_stopped, overridden

# Real-car DWB local goal distance (2026-10-29 nav-turn-start, board only). DWB's GoalDist and GoalAlign critics score
# the Manhattan distance (|dx| + |dy|, not the path length) from a trajectory's end to the LAST pose of the plan they
# get; DWB hands them the plan up to the first pose farther than forward_prune_distance (Nav2 default 2.0 m,
# Euclidean) from the car. A goal within 2 m therefore is the critics' target itself, even when the path first leads
# away from it (round a wall or a chair, or the goal beside / behind the car). Then every trajectory along the path
# scores worse on GoalDist + GoalAlign (24 + 24) and no better on PathDist / PathAlign (an in-place turn stays on the
# path too), so DWB's best command is an in-place turn at its lowest legal rate: host sim, wall 0.55 m in front of the
# goal, the path's first leg 90 deg off -- the car turned to the path, then turned back and forth on the spot for 80 s
# (nav-turn-start-20261029-results base2: at v 0.10 m/s +4 cells GoalDist, v 0 best). 0.8 m makes the critics'
# target a point on the path ahead (the first pose 0.8 m away), which every step along the path gets closer to; it is
# three times the longest trajectory (0.15 m/s x 1.7 s = 0.26 m), so a full-speed step does not pass it. Within 0.8 m
# of the goal the target is the goal as before. CAR_DWB_FORWARD_PRUNE_M (car_up -> FPGA_NAV_DWB_FORWARD_PRUNE_M)
# overrides it on the day; 2.0 = Nav2's default (the behaviour before).
CAR_DWB_FORWARD_PRUNE_M = 0.8
CAR_FORWARD_PRUNE_ENV = 'FPGA_NAV_DWB_FORWARD_PRUNE_M'
CAR_FORWARD_PRUNE_RANGE = (0.3, 4.0)


def car_forward_prune(environ):
    """(distance, given): DWB forward_prune_distance for the car -- CAR_DWB_FORWARD_PRUNE_M, or the number in
    FPGA_NAV_DWB_FORWARD_PRUNE_M (given = True). ValueError on a non-number or a value outside CAR_FORWARD_PRUNE_RANGE
    (below 0.3 m the target sits within one trajectory; above 4 m is beyond any local costmap)."""
    raw = (environ.get(CAR_FORWARD_PRUNE_ENV) or '').strip()
    if not raw:
        return CAR_DWB_FORWARD_PRUNE_M, False
    try:
        value = float(raw)
    except ValueError:
        raise ValueError('%s must be a number (given %r)' % (CAR_FORWARD_PRUNE_ENV, raw))
    low, high = CAR_FORWARD_PRUNE_RANGE
    if not low <= value <= high:   # NaN fails too
        raise ValueError('%s must be within %g..%g m (given %r)' % (CAR_FORWARD_PRUNE_ENV, low, high, raw))
    return value, True


def apply_car_forward_prune(params, distance=None):
    """FollowPath forward_prune_distance = distance (CAR_DWB_FORWARD_PRUNE_M) when FollowPath drives with DWB (alone
    or inside the rotation shim, which configures DWB under the same FollowPath.* keys); not DWB: unchanged. DWB's
    shorten_transformed_plan (default true) is what makes it count; it is set true here so a file cannot turn it off.
    Returns a copy."""
    distance = CAR_DWB_FORWARD_PRUNE_M if distance is None else distance
    result = deepcopy(params)
    follow = result.get('controller_server', {}).get('ros__parameters', {}).get('FollowPath')
    if isinstance(follow, dict) and follow_path_is_dwb(follow):
        follow['forward_prune_distance'] = distance
        follow['shorten_transformed_plan'] = True
    return result

# Real-car progress checker (2026-10-25, board only). The simulation keeps nav2_overrides.yaml's SimpleProgressChecker
# 0.15 m / 15 s (xy only). On the car the 10-22 run had 11 "Failed to make progress" aborts. PoseProgressChecker
# nav2_controller/plugins/pose_progress_checker.cpp) also counts a heading change of required_movement_angle as
# progress, so the rotation shim's in-place turn and DWB's end-of-path yaw alignment are progress too. It is measured
# in the local costmap frame (odom on the car), so a map -> odom correction (map align at mapping_stop) is not
# progress. Values from the car limits (CAR_SPEED_LIMITS 0.15 m/s, 0.4 rad/s, accel 0.3 / 0.6) and the 10-17 cold-
# start crawl (~0.03 m/s):
#   radius 0.05 m   crawl: 1.7 s; full speed: 0.33 s + accel ramp; above wheel-odometry jitter (odom flat at rest)
#   angle  0.50 rad (Nav2's default) shim turn at 0.4 rad/s: 1.25 s (+0.67 s ramp); DWB's slowest legal turn
#                   (0.105 rad/s, CAR_DWB_MIN_SPEEDS): 4.8 s. Above the in-place dither DWB falls into when every
#                   forward trajectory hits the unseen ring ahead (harness: +-0.105..0.147 rad/s back and forth, yaw
#                   span 0.22-0.26 rad): 0.20 counted that dither as progress and hid the stall until the 60 s wall
#                   clock; 0.50 lets it abort at 20 s so the BT recovery (incl. BackUp 0.30 m) gets its turn
#   allowance 20 s  one progress abort -> BT FollowPath retry (Wait 1 s) -> second abort -> outer recovery fits in
#                   2 x 20 + ~3 s = 43 s < CAR_NAV_TIMEOUT_S 60 (ros_runtime's wall clock); 30 s would let the wall
#                   timeout end the goal before the first retry has run its course; it is 5 s longer than the
#                   simulation's 15 s to ride out short motion-permit pauses (geometry_stale)
# FPGA_NAV_PROGRESS_CHECKER=simple falls back to SimpleProgressChecker with the same radius and allowance (for an
# image without the pose plugin: the log line "Created progress_checker : progress_checker of type ..." says which).
POSE_PROGRESS_PLUGIN = 'nav2_controller::PoseProgressChecker'
SIMPLE_PROGRESS_PLUGIN = 'nav2_controller::SimpleProgressChecker'
CAR_PROGRESS_CHECKER = {'pose': {'plugin': POSE_PROGRESS_PLUGIN, 'required_movement_radius': 0.05,
                                 'required_movement_angle': 0.50, 'movement_time_allowance': 20.0},
                        'simple': {'plugin': SIMPLE_PROGRESS_PLUGIN, 'required_movement_radius': 0.05,
                                   'movement_time_allowance': 20.0}}
# Real-car global planner (2026-10-25, board only), NavFn GridBased on the global costmap (car: static_layer = the
# exported RTAB-Map grid + inflation_layer, no obstacle layer). config/rolling_layer_nav2.yaml keeps tolerance 0 /
# allow_unknown false and host/rolling_nav_layer.check_costmap_config still gates that file (09-27 owner ruling,
# docs/design/ROLLING_NAV2_LAYER_20260927.md section 4.1 item 4); the car relaxes it AFTER that gate:
#   allow_unknown true  the 10-22 run after mapping_stop: 8 x "GridBased: failed to create plan with tolerance 0.00"
#                       -- goals and paths through cells the export left UNKNOWN. The local costmap (rolling layer,
#                       track_unknown_space true, footprint clearing off) and DWB BaseObstacle + ObstacleFootprint
#                       still refuse NO_INFORMATION and LETHAL under the body, so a global path through unknown
#                       cells cannot drive the car into what it has not seen free; the failure becomes "wait / no
#                       progress" at the unknown border instead of "no plan" (and unknown cells turn free as the
#                       car looks). NavFn clears the start cell (NavfnPlanner::clearRobotCell).
#   tolerance 0.25 m    a goal inside a LETHAL cell (an observation pose on a mapped obstacle) is replaced by the
#                       best reachable cell within 0.25 m; covers the 0.15 m band between robot_radius 0.30 and
#                       inflation_radius 0.45 plus a cell. Smoke try5 (tolerance 0.5, allow_unknown false): NavFn
#                       moved an unknown-cell goal to known cells and Nav2 reported success 0.55 m short; with
#                       allow_unknown true an unknown goal is planned to directly and the shortfall is <= 0.25 m.
# Default strict (coordinator 10-25: the relaxation conflicts with the 09-27 owner ruling and waits for the owner):
# the car keeps the rolling yaml's 0 / false; FPGA_NAV_CAR_PLANNER=relaxed applies CAR_PLANNER.
CAR_PLANNER = {'tolerance': 0.25, 'allow_unknown': True}


def _choice(environ, name, default, allowed):
    value = environ.get(name, default).lower()
    if value not in allowed:
        raise ValueError('%s must be one of %s' % (name, '|'.join(allowed)))
    return value


def car_inputs(environ, profile):
    """The real-car switches of navigation.launch.py, all env (the launch builds its actions without launch
    arguments; tests/test_nav_rehang_launch.py). Defaults = the simulation's behaviour:
      FPGA_NAV_USE_SIM_TIME     true | false   every node's use_sim_time (the car has no /clock)
      FPGA_NAV_STATIC_MAP_ODOM  auto | true | false   the identity map -> odom (auto: reference profile only)
      FPGA_NAV_SIM_CAMERA_TF    auto | true | false   the simulated camera mount base_link -> camera_link ->
                               camera_optical_frame (auto: reference only; the car's mount TF comes from car_inner)
      FPGA_NAV_SPEED_LIMITS     sim | car      car: apply_speed_limits(CAR_SPEED_LIMITS) on the merged parameters,
                               then apply_car_goal_tolerance(CAR_XY_GOAL_TOLERANCE) (0.10 m; the simulation's 0.03),
                               apply_car_progress_checker / apply_car_planner (2026-10-25; launch-only switches
                               FPGA_NAV_PROGRESS_CHECKER pose|simple, FPGA_NAV_CAR_PLANNER strict|relaxed, car only)
      FPGA_NAV_VELOCITY_GUARD   false | true   start velocity_guard.py (/cmd_vel + /fpga/motion_permit ->
                               /fpga/cmd_vel_safe, 0.5 s expiry); on the car only -- the simulation starts its own
                               in simulation.launch.py, and the teleop node must not run with it (one publisher)
      FPGA_NAV_COSTMAP          sim | rolling  rolling: apply_rolling_costmap() -- the local costmap becomes
                               config/rolling_layer_nav2.yaml's rolling layer (/fpga/rolling_map/grid from
                               host/ros_rolling_nav.py, fed by the geometry producer) and the controller DWB with
                               ObstacleFootprint; sim: nav2_overrides.yaml's obstacle layers on /fpga/points (which
                               nothing publishes on the live car). No separate controller switch: the 09-27 design
                               (docs/design/ROLLING_NAV2_LAYER_20260927.md F10, section 2(a), owner ruling 4.1 item 4)
                               forbids the rolling layer with Regulated Pure Pursuit, which drives into NO_INFORMATION
    The static transforms exist only in the reference profile (estimated_inspect never had them): true there is
    refused."""
    sim_time = _choice(environ, 'FPGA_NAV_USE_SIM_TIME', 'true', ('true', 'false')) == 'true'
    flags = {}
    for key, name in (('static_map_odom', 'FPGA_NAV_STATIC_MAP_ODOM'), ('sim_camera_tf', 'FPGA_NAV_SIM_CAMERA_TF')):
        value = _choice(environ, name, 'auto', ('auto', 'true', 'false'))
        if value == 'true' and profile != 'reference':
            raise ValueError('%s=true needs the reference profile' % name)
        flags[key] = profile == 'reference' if value == 'auto' else value == 'true'
    speed = _choice(environ, 'FPGA_NAV_SPEED_LIMITS', 'sim', ('sim', 'car'))
    guard = _choice(environ, 'FPGA_NAV_VELOCITY_GUARD', 'false', ('true', 'false')) == 'true'
    costmap = _choice(environ, 'FPGA_NAV_COSTMAP', 'sim', ('sim', 'rolling'))
    return dict(use_sim_time=sim_time, speed_limits=speed, velocity_guard=guard, costmap=costmap, **flags)


DWB_PLUGIN = 'dwb_core::DWBLocalPlanner'
ROLLING_GLOBAL_LAYERS = ['static_layer', 'inflation_layer']


def _merge(base, overlay):
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _without_layers(block, keep=()):
    """block minus every costmap layer sub-block (a dict with a 'plugin' key) not named in keep."""
    return {key: deepcopy(value) for key, value in block.items()
            if key in keep or not (isinstance(value, dict) and 'plugin' in value)}


def apply_rolling_costmap(params, rolling, default_follow_path, overlay_follow_path, formal_follow_path):
    """FPGA_NAV_COSTMAP=rolling (10-15, real car): params (Nav2 defaults merged with the overrides file) with
      local_costmap   its layer blocks dropped (no ObstacleLayer on /fpga/points left behind), then rolling's
                      local_costmap merged in: plugins [rolling_layer, inflation_layer], odom rolling window
      global_costmap  plugins [static_layer, inflation_layer] (the known map; its ObstacleLayer subscribed
                      /fpga/points too), other layer blocks dropped
      FollowPath      rebuilt, not deep-merged: default_follow_path (Nav2's DWB block) + the overrides file's
                      FollowPath when that is DWB (plugin absent or DWB_PLUGIN), else formal_follow_path
                      (config/nav2_overrides.yaml's DWB tuning) -- an RPP override file
                      (nav_controller_variants rpp_*.yaml) must not leave its keys behind -- then rolling's
                      FollowPath (plugin DWB, critics with ObstacleFootprint)
      GridBased       rolling's planner block merged (allow_unknown false, tolerance 0)
    rolling = the parsed rolling_layer_nav2.yaml. Returns (params copy, evidence dict). The caller must still run
    host/rolling_nav_layer.check_costmap_config on the result."""
    result = deepcopy(params)
    local = result.setdefault('local_costmap', {}).setdefault('local_costmap', {}).setdefault('ros__parameters', {})
    result['local_costmap']['local_costmap']['ros__parameters'] = _merge(
        _without_layers(local), rolling['local_costmap']['local_costmap']['ros__parameters'])
    glob = result.setdefault('global_costmap', {}).setdefault('global_costmap', {}).setdefault('ros__parameters', {})
    glob = _without_layers(glob, keep=ROLLING_GLOBAL_LAYERS)
    glob['plugins'] = list(ROLLING_GLOBAL_LAYERS)
    result['global_costmap']['global_costmap']['ros__parameters'] = glob
    overlay_plugin = overlay_follow_path.get('plugin', DWB_PLUGIN)
    tuning = overlay_follow_path if overlay_plugin == DWB_PLUGIN else formal_follow_path
    controller = result.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    controller['FollowPath'] = _merge(_merge(default_follow_path, tuning),
                                      rolling['controller_server']['ros__parameters']['FollowPath'])
    planner = result.setdefault('planner_server', {}).setdefault('ros__parameters', {})
    planner['GridBased'] = _merge(planner.get('GridBased', {}),
                                  rolling['planner_server']['ros__parameters']['GridBased'])
    evidence = {'local_plugins': result['local_costmap']['local_costmap']['ros__parameters']['plugins'],
                'global_plugins': glob['plugins'], 'follow_path_plugin': controller['FollowPath']['plugin'],
                'overrides_follow_path_plugin': overlay_plugin,
                'follow_path_tuning': 'overrides' if tuning is overlay_follow_path else 'nav2_overrides.yaml'}
    return result, evidence


def apply_speed_limits(params, limits):
    """Nav2 parameters with every speed / acceleration key that exists capped at limits (linear m/s, angular rad/s,
    linear_accel m/s^2, angular_accel rad/s^2): the controller plugins (DWB and Regulated Pure Pursuit keys), the
    velocity smoother (its [x, y, theta] lists) and the behavior server. Only existing keys change, only ever
    towards slower (min of the positive limits, max of the negative ones); returns a copy."""
    lin, ang, lacc, aacc = limits['linear'], limits['angular'], limits['linear_accel'], limits['angular_accel']
    caps = {'max_vel_x': lin, 'max_speed_xy': lin, 'desired_linear_vel': lin,
            'regulated_linear_scaling_min_speed': lin, 'max_vel_theta': ang, 'rotate_to_heading_angular_vel': ang,
            'acc_lim_x': lacc, 'acc_lim_theta': aacc, 'max_angular_accel': aacc}
    floors = {'min_vel_x': -lin, 'decel_lim_x': -lacc, 'decel_lim_theta': -aacc}
    result = deepcopy(params)

    def clamp(block):
        for key, cap in caps.items():
            if key in block:
                block[key] = min(block[key], cap)
        for key, floor in floors.items():
            if key in block:
                block[key] = max(block[key], floor)

    controller = result.get('controller_server', {}).get('ros__parameters', {})
    for value in controller.values():
        if isinstance(value, dict):
            clamp(value)
    smoother = result.get('velocity_smoother', {}).get('ros__parameters', {})
    for key, bound, sign in (('max_velocity', (lin, lin, ang), 1), ('min_velocity', (lin, lin, ang), -1),
                             ('max_accel', (lacc, lacc, aacc), 1), ('max_decel', (lacc, lacc, aacc), -1)):
        if key in smoother:
            smoother[key] = [min(v, b) if sign > 0 else max(v, -b) for v, b in zip(smoother[key], bound)]
    behavior = result.get('behavior_server', {}).get('ros__parameters', {})
    for key, cap in (('max_rotational_vel', ang), ('min_rotational_vel', ang), ('rotational_acc_lim', aacc)):
        if key in behavior:
            behavior[key] = min(behavior[key], cap)
    return result


def apply_car_goal_tolerance(params, xy_tolerance):
    """FPGA_NAV_SPEED_LIMITS=car: Nav2 parameters with xy_goal_tolerance = xy_tolerance on every goal checker block
    controller_server lists in goal_checker_plugins (Nav2's default list: general_goal_checker) and on FollowPath when
    it is DWB (plugin absent counts as DWB, as in apply_rolling_costmap) -- DWB's own copy decides when it stops
    translating and only rotates. An RPP FollowPath gets no key; yaw_goal_tolerance is untouched. Returns a copy."""
    result = deepcopy(params)
    controller = result.get('controller_server', {}).get('ros__parameters', {})
    for name in controller.get('goal_checker_plugins', ['general_goal_checker']):
        if isinstance(controller.get(name), dict):
            controller[name]['xy_goal_tolerance'] = xy_tolerance
    follow = controller.get('FollowPath')
    if isinstance(follow, dict) and follow_path_is_dwb(follow):
        follow['xy_goal_tolerance'] = xy_tolerance
    return result


def follow_path_is_dwb(follow):
    """True when the FollowPath block drives with DWB: plugin DWB (absent counts as DWB), or the rotation shim
    wrapping DWB as its primary controller (DWB then reads the same FollowPath.* keys)."""
    plugin = follow.get('plugin', DWB_PLUGIN)
    return plugin == DWB_PLUGIN or (plugin == ROTATION_SHIM_PLUGIN and follow.get('primary_controller') == DWB_PLUGIN)


def apply_car_yaw_goal_tolerance(params, yaw_tolerance):
    """FPGA_NAV_SPEED_LIMITS=car: yaw_goal_tolerance = yaw_tolerance on every goal checker block controller_server
    lists in goal_checker_plugins (default general_goal_checker). DWB has no yaw key of its own (RotateToGoal only
    decides when to stop translating). Returns a copy."""
    result = deepcopy(params)
    controller = result.get('controller_server', {}).get('ros__parameters', {})
    for name in controller.get('goal_checker_plugins', ['general_goal_checker']):
        if isinstance(controller.get(name), dict):
            controller[name]['yaw_goal_tolerance'] = yaw_tolerance
    return result


def apply_rotation_shim(params, shim=None):
    """FollowPath (must be DWB) wrapped in Nav2 Humble's RotationShimController with DWB as primary_controller;
    every DWB key stays (the shim configures DWB under the same plugin name). shim = the shim's own keys
    (CAR_ROTATION_SHIM by default); apply before apply_speed_limits, which caps rotate_to_heading_angular_vel and
    max_angular_accel. Refused (ValueError) when FollowPath is not DWB. Returns a copy."""
    result = deepcopy(params)
    controller = result.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    follow = controller.get('FollowPath')
    if not isinstance(follow, dict) or follow.get('plugin', DWB_PLUGIN) != DWB_PLUGIN:
        raise ValueError('rotation shim wraps DWB only; FollowPath plugin is %r'
                         % (follow.get('plugin') if isinstance(follow, dict) else None))
    follow['plugin'] = ROTATION_SHIM_PLUGIN
    follow['primary_controller'] = DWB_PLUGIN
    follow.update(deepcopy(CAR_ROTATION_SHIM if shim is None else shim))
    return result


def apply_car_min_speeds(params, min_speeds=None, rot_stopped=None):
    """DWB min_speed_xy / min_speed_theta = min_speeds (CAR_DWB_MIN_SPEEDS) on FollowPath when it drives with DWB
    (alone or as the rotation shim's primary controller, which reads the same keys), and rot_stopped_velocity =
    rot_stopped (CAR_ROT_STOPPED_VELOCITY) on every StoppedGoalChecker block goal_checker_plugins lists (default
    general_goal_checker; other goal checkers have no such key). Not DWB: unchanged. Returns a copy."""
    min_speeds = CAR_DWB_MIN_SPEEDS if min_speeds is None else min_speeds
    rot_stopped = CAR_ROT_STOPPED_VELOCITY if rot_stopped is None else rot_stopped
    result = deepcopy(params)
    controller = result.get('controller_server', {}).get('ros__parameters', {})
    follow = controller.get('FollowPath')
    if not (isinstance(follow, dict) and follow_path_is_dwb(follow)):
        return result
    follow.update(min_speeds)
    for name in controller.get('goal_checker_plugins', ['general_goal_checker']):
        block = controller.get(name)
        if isinstance(block, dict) and block.get('plugin') == STOPPED_GOAL_CHECKER:
            block['rot_stopped_velocity'] = rot_stopped
    return result


def car_progress_checker(environ):
    """FPGA_NAV_PROGRESS_CHECKER (car only, default pose): pose | simple -> the CAR_PROGRESS_CHECKER block."""
    return CAR_PROGRESS_CHECKER[_choice(environ, 'FPGA_NAV_PROGRESS_CHECKER', 'pose', ('pose', 'simple'))]


def car_planner(environ):
    """FPGA_NAV_CAR_PLANNER (car only, default strict = the 09-27 ruling): strict -> None (no change); relaxed ->
    CAR_PLANNER (owner to decide)."""
    return CAR_PLANNER if _choice(environ, 'FPGA_NAV_CAR_PLANNER', 'strict', ('relaxed', 'strict')) == 'relaxed' \
        else None


def apply_car_progress_checker(params, block):
    """controller_server's progress checker (the block progress_checker_plugin names; Nav2's default name
    progress_checker) replaced by block -- replaced, not merged, so a pose -> simple fallback leaves no
    required_movement_angle behind. Returns a copy."""
    result = deepcopy(params)
    controller = result.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    controller[controller.get('progress_checker_plugin', 'progress_checker')] = deepcopy(block)
    return result


def apply_car_planner(params, planner):
    """planner_server GridBased tolerance / allow_unknown = planner's (None: unchanged). Other keys stay. Copy."""
    result = deepcopy(params)
    if planner is not None:
        grid = result.setdefault('planner_server', {}).setdefault('ros__parameters', {}).setdefault('GridBased', {})
        grid.update(deepcopy(planner))
    return result


# 2026-10-29 find while mapping (car_up CAR_NAV_LIVE_GLOBAL, default on; host/nav_global_map.py): in the mapping run
# car_inner starts host/ros_nav_global_map.py and sets FPGA_NAV_GLOBAL_MAP_TOPIC=/fpga/nav/global_map -- map_server's map
# with rtabmap's live occupied cells on top while mapping, map_server's map as it is otherwise (the export after the
# stop). Only the GLOBAL costmap's static_layer reads it; /map (the runner, the console) and the local costmap are
# unchanged. Unset / empty = static_layer on /map, the line of before.
def global_map_topic(environ):
    """FPGA_NAV_GLOBAL_MAP_TOPIC -> the topic, or None when unset / empty. ValueError for a relative name."""
    value = (environ.get('FPGA_NAV_GLOBAL_MAP_TOPIC') or '').strip()
    if not value:
        return None
    if not value.startswith('/') or any(c.isspace() for c in value):
        raise ValueError('FPGA_NAV_GLOBAL_MAP_TOPIC must be an absolute topic name (given %r)' % value)
    return value


def apply_global_map_topic(params, topic):
    """global_costmap's static_layer map_topic = topic (None: unchanged). Refused (ValueError) when the global
    costmap has no static_layer in its plugins. Returns a copy."""
    result = deepcopy(params)
    if topic is None:
        return result
    glob = result.setdefault('global_costmap', {}).setdefault('global_costmap', {}).setdefault('ros__parameters', {})
    if 'static_layer' not in (glob.get('plugins') or []):
        raise ValueError('FPGA_NAV_GLOBAL_MAP_TOPIC needs a global static_layer (plugins %r)' % glob.get('plugins'))
    glob.setdefault('static_layer', {})['map_topic'] = topic
    return result


def bridge_inputs(entries, profile):
    if profile not in ('reference', 'estimated_inspect'):
        raise ValueError('unknown input profile')
    result = deepcopy(entries)
    if profile == 'reference':
        return result
    # Keep simulator truth on diagnostic topics only. No ROS command can reach
    # Gazebo through this bridge, even if an old permit publisher is present.
    result = [entry for entry in result if entry['direction'] == 'GZ_TO_ROS']
    for entry in result:
        topic = entry['ros_topic_name']
        if topic in ('/odom', '/tf'):
            entry['ros_topic_name'] = '/reference' + topic
    return result


CLOUD_SOURCES = ('gazebo', 'adapter')


def cloud_source(environ):
    """FPGA_SIM_CLOUD: gazebo (default, unchanged: /fpga/points bridged from Fortress, whose XYZ is
    forward-left-up under an optical header -- the AGX relay relabelled it) or adapter (line D1:
    not bridged; scripts/sim_adapter.py --cloud publishes an optical-frame cloud from /fpga/depth)."""
    value = environ.get('FPGA_SIM_CLOUD', 'gazebo')
    if value not in CLOUD_SOURCES:
        raise ValueError('FPGA_SIM_CLOUD must be gazebo or adapter')
    return value


def cloud_inputs(entries, cloud):
    if cloud not in CLOUD_SOURCES:
        raise ValueError('unknown cloud source')
    if cloud == 'gazebo':
        return deepcopy(entries)
    return [deepcopy(entry) for entry in entries if entry['ros_topic_name'] != '/fpga/points']


SIM_WHEEL_COMMAND = '/fpga/cmd_vel'      # the Gazebo DiffDrive topic (worlds/*.sdf)


def command_input(entries, ros_topic):
    """Line D2 (2026-10-14): the single ROS -> Gazebo wheel command comes from ros_topic (the MCU
    stand-in's executed speed) instead of /fpga/cmd_vel_safe. Exactly one ROS_TO_GZ entry may target the
    wheel command; every other entry is copied unchanged. Raises if there is no such entry (e.g. the
    estimated_inspect profile, which bridges no command at all)."""
    if not ros_topic or not ros_topic.startswith('/'):
        raise ValueError('command topic must be an absolute ROS topic')
    result = [deepcopy(entry) for entry in entries]
    targets = [entry for entry in result
               if entry['direction'] == 'ROS_TO_GZ' and entry['gz_topic_name'] == SIM_WHEEL_COMMAND]
    if len(targets) != 1:
        raise ValueError('expected exactly one ROS_TO_GZ entry for %s, found %d' % (SIM_WHEEL_COMMAND, len(targets)))
    targets[0]['ros_topic_name'] = ros_topic
    return result
