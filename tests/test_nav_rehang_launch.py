"""The Nav2 launch applies the input profile's autostart to both lifecycle managers.

dev2's 6d0ad28 moved the FPGA_NAV_AUTOSTART parsing into config/nav_input_profile.py and
makes estimated_inspect REQUIRE "false" ("lifecycle stays inactive", NAV2_STAGE_D.md §1).
On dev2's D branch the managers read that value through lines that first arrived with the
abandoned 643ca69 (b416ef8); main has them since dev2's M5 (6a99cba). A first rehang onto
c9aea50, which predates M5 and hard-codes autostart true, silently lost them: the profile
parsed "false", printed "false" into its evidence line, and every Nav2 lifecycle node
would still have been activated. dev2's own launch test builds the actions but never reads
the managers' autostart, so it passed either way. This file reads it, so the wiring
cannot be dropped again without a failing test.

Host-only: the real launch builder runs against lightweight action records. No ROS, DDS,
lifecycle transition or motion is exercised or claimed.
"""
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'robot_sim/fpga_sim'


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


def load_navigation_launch(share):
    definitions = {
        'ament_index_python.packages': {'get_package_share_directory': lambda _: str(share)},
        'launch': {'LaunchDescription': Action},
        'launch.actions': {x: Action for x in ('IncludeLaunchDescription', 'RegisterEventHandler',
                                               'OpaqueFunction', 'GroupAction', 'ExecuteProcess')},
        'launch.event_handlers': {'OnShutdown': Action},
        'launch.launch_description_sources': {'PythonLaunchDescriptionSource': Action},
        'launch_ros.actions': {'Node': Action, 'SetRemap': Action},
    }
    modules = {}
    for key, attrs in definitions.items():
        module = types.ModuleType(key)
        module.__dict__.update(attrs)
        modules[key] = module
    spec = importlib.util.spec_from_file_location('rehang_navigation_launch',
                                                  ASSETS / 'launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class LaunchAutostartTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        share = self.share = Path(self.temp.name)
        for sub in ('params', 'config', 'behavior_trees', 'maps'):
            (share / sub).mkdir()
        (share / 'params/nav2_params.yaml').write_text(yaml.safe_dump({
            'map_server': {'ros__parameters': {}}, 'bt_navigator': {'ros__parameters': {}}}))
        (share / 'config/nav2_overrides.yaml').write_text('{}')
        (share / 'config/nav_input_profile.py').write_text(
            (ASSETS / 'config/nav_input_profile.py').read_text())
        (share / 'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml').write_text('<root/>')
        (share / 'maps/room.yaml').write_text('image: room.pgm')
        self.module = load_navigation_launch(share)

    def build(self, env):
        """Return (map manager autostart, Nav2 include autostart, printed profile line)."""
        printed = io.StringIO()
        with patch.dict(os.environ, env, clear=True), \
                patch.object(self.module, 'bounded_recovery', lambda text: text), \
                redirect_stdout(printed):
            actions = self.module.generate_launch_description().args[0]
        try:
            manager = next(a for a in actions if a.kwargs.get('name') == 'lifecycle_manager_map')
            group = next(a for a in actions if a.args and isinstance(a.args[0], list))
            include = next(a for a in group.args[0] if 'launch_arguments' in a.kwargs)
            arguments = dict(include.kwargs['launch_arguments'])
            lines = [json.loads(line) for line in printed.getvalue().splitlines() if line.strip()]
            profile_line = next(line for line in lines if 'input_profile' in line)
            return manager.kwargs['parameters'][0]['autostart'], arguments['autostart'], profile_line
        finally:
            actions[-1].args[0].kwargs['on_shutdown'][0].kwargs['function'](None)

    def test_estimated_inspect_starts_no_lifecycle_node(self):
        manager, include, printed = self.build({
            'FPGA_NAV_INPUT_PROFILE': 'estimated_inspect', 'FPGA_NAV_AUTOSTART': 'false',
            'FPGA_NAV_MAP_YAML': str(self.share / 'maps/room.yaml')})
        self.assertIs(manager, False)
        self.assertEqual(include, 'false')
        # The evidence line and the actions must agree; before the wiring they did not.
        self.assertEqual(printed['autostart'], 'false')

    def test_reference_without_the_variable_keeps_mains_autostart(self):
        # Negative control: the path main's runner takes today (variable unset) is unchanged,
        # so the wiring cannot be passing by switching autostart off everywhere.
        manager, include, printed = self.build({})
        self.assertIs(manager, True)
        self.assertEqual(include, 'true')
        self.assertEqual(printed['input_profile'], 'reference')

    def test_reference_with_explicit_false_is_honoured_too(self):
        manager, include, _ = self.build({'FPGA_NAV_AUTOSTART': 'false'})
        self.assertIs(manager, False)
        self.assertEqual(include, 'false')



CAR_ENV = {'FPGA_NAV_USE_SIM_TIME': 'false', 'FPGA_NAV_STATIC_MAP_ODOM': 'false', 'FPGA_NAV_SIM_CAMERA_TF': 'false',
           'FPGA_NAV_SPEED_LIMITS': 'car', 'FPGA_NAV_AUTOSTART': 'false'}


class RealCarEntryTests(LaunchAutostartTests):
    """10-15 real-car entry: the same launch with env switches (no launch arguments: see above)."""

    def setUp(self):
        super().setUp()
        params = yaml.safe_load((self.share / 'params/nav2_params.yaml').read_text())
        params.update({
            'controller_server': {'ros__parameters': {
                # the simulation's progress checker (nav2_overrides.yaml + Nav2's plugin), so the car's is a change
                'progress_checker_plugin': 'progress_checker',
                'progress_checker': {'plugin': 'nav2_controller::SimpleProgressChecker',
                                     'required_movement_radius': 0.15, 'movement_time_allowance': 15.0},
                # the simulation's goal tolerances (config/nav2_overrides.yaml), so the car's 0.10 is a real change
                'general_goal_checker': {'plugin': 'nav2_controller::StoppedGoalChecker', 'rot_stopped_velocity': 0.03,
                                         'xy_goal_tolerance': 0.03, 'yaw_goal_tolerance': 0.03},
                'FollowPath': {'xy_goal_tolerance': 0.03, 'min_speed_xy': 0.0, 'min_speed_theta': 0.0,
                'max_vel_x': 0.26, 'max_speed_xy': 0.26, 'max_vel_theta': 1.0, 'acc_lim_x': 2.5,
                'decel_lim_x': -2.5, 'acc_lim_theta': 3.2, 'decel_lim_theta': -3.2, 'min_vel_x': -0.5,
                'desired_linear_vel': 0.25, 'regulated_linear_scaling_min_speed': 0.25,
                'rotate_to_heading_angular_vel': 0.6, 'max_angular_accel': 1.0, 'max_vel_y': 0.0}}},
            'velocity_smoother': {'ros__parameters': {'max_velocity': [0.26, 0.0, 1.0],
                                                      'min_velocity': [-0.26, 0.0, -1.0],
                                                      'max_accel': [2.5, 0.0, 3.2], 'max_decel': [-2.5, 0.0, -3.2]}},
            'behavior_server': {'ros__parameters': {'max_rotational_vel': 1.0, 'min_rotational_vel': 0.4,
                                                    'rotational_acc_lim': 3.2}},
            # the simulation's NavFn values (nav2_overrides.yaml), so the car's 0.25 / true is a real change
            'planner_server': {'ros__parameters': {'GridBased': {'plugin': 'nav2_navfn_planner/NavfnPlanner',
                                                                 'tolerance': 0.15, 'allow_unknown': False}}}})
        (self.share / 'params/nav2_params.yaml').write_text(yaml.safe_dump(params))
        (self.share / 'maps/car.yaml').write_text('image: car.pgm')
        (self.share / 'scripts').mkdir()
        (self.share / 'scripts/velocity_guard.py').write_text('')

    def actions(self, env):
        printed = io.StringIO()
        with patch.dict(os.environ, env, clear=True), \
                patch.object(self.module, 'bounded_recovery', lambda text: text), redirect_stdout(printed):
            actions = self.module.generate_launch_description().args[0]
        map_node = next(a for a in actions if a.kwargs.get('name') == 'map_server')
        params = yaml.safe_load(Path(map_node.kwargs['parameters'][0]).read_text())
        actions[-1].args[0].kwargs['on_shutdown'][0].kwargs['function'](None)
        lines = [json.loads(line) for line in printed.getvalue().splitlines() if line.strip()]
        return actions, params, lines

    def test_car_entry_real_time_no_sim_transforms_limited_speeds(self):
        actions, params, lines = self.actions(dict(CAR_ENV, FPGA_NAV_MAP_YAML=str(self.share / 'maps/car.yaml')))
        manager = next(a for a in actions if a.kwargs.get('name') == 'lifecycle_manager_map')
        group = next(a for a in actions if a.args and isinstance(a.args[0], list))
        include = next(a for a in group.args[0] if 'launch_arguments' in a.kwargs)
        self.assertIs(manager.kwargs['parameters'][0]['use_sim_time'], False)
        self.assertEqual(dict(include.kwargs['launch_arguments'])['use_sim_time'], 'false')
        self.assertTrue(all(block['ros__parameters']['use_sim_time'] is False
                            for block in params.values() if 'ros__parameters' in block))
        self.assertEqual([a for a in actions if a.kwargs.get('executable') == 'static_transform_publisher'], [])
        self.assertEqual(params['map_server']['ros__parameters']['yaml_filename'], str(self.share / 'maps/car.yaml'))
        remaps = {r.kwargs['src']: r.kwargs['dst'] for r in group.args[0] if 'src' in r.kwargs}
        self.assertEqual(remaps, {'behavior_server:cmd_vel': 'cmd_vel_nav'})
        follow = params['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual({k: follow[k] for k in ('max_vel_x', 'max_speed_xy', 'desired_linear_vel',
                                                 'regulated_linear_scaling_min_speed', 'max_vel_theta',
                                                 'rotate_to_heading_angular_vel', 'acc_lim_x', 'decel_lim_x',
                                                 'acc_lim_theta', 'decel_lim_theta', 'max_angular_accel',
                                                 'min_vel_x', 'max_vel_y')},
                         {'max_vel_x': 0.15, 'max_speed_xy': 0.15, 'desired_linear_vel': 0.15,
                          'regulated_linear_scaling_min_speed': 0.15, 'max_vel_theta': 0.4,
                          'rotate_to_heading_angular_vel': 0.4, 'acc_lim_x': 0.3, 'decel_lim_x': -0.3,
                          'acc_lim_theta': 0.6, 'decel_lim_theta': -0.6, 'max_angular_accel': 0.6,
                          'min_vel_x': -0.15, 'max_vel_y': 0.0})
        smoother = params['velocity_smoother']['ros__parameters']
        self.assertEqual((smoother['max_velocity'], smoother['min_velocity'], smoother['max_accel'],
                          smoother['max_decel']),
                         ([0.15, 0.0, 0.4], [-0.15, 0.0, -0.4], [0.3, 0.0, 0.6], [-0.3, 0.0, -0.6]))
        behavior = params['behavior_server']['ros__parameters']
        self.assertEqual((behavior['max_rotational_vel'], behavior['min_rotational_vel'],
                          behavior['rotational_acc_lim']), (0.4, 0.4, 0.6))
        self.assertEqual(lines[-1]['real_car']['speed_limits'], 'car')
        self.assertEqual([a for a in actions if a.kwargs.get('name') == 'velocity_guard'], [])
        self.assert_goal_tolerance(params, 0.10, 0.10)
        self.assert_rotation_shim(follow)
        self.assertEqual(lines[-1]['rotation_shim'], runpy_inputs(self.share)['CAR_ROTATION_SHIM'])
        self.assertEqual(lines[-1]['yaw_goal_tolerance'], 0.10)

    def assert_goal_tolerance(self, params, xy, yaw):
        """general_goal_checker and FollowPath xy_goal_tolerance == xy; general_goal_checker yaw_goal_tolerance == yaw
        (the car 0.10 since 10-22; the simulation keeps 0.03)."""
        controller = params['controller_server']['ros__parameters']
        self.assertEqual((controller['general_goal_checker']['xy_goal_tolerance'],
                          controller['FollowPath']['xy_goal_tolerance'],
                          controller['general_goal_checker']['yaw_goal_tolerance']), (xy, xy, yaw))

    def assert_rotation_shim(self, follow):
        """FollowPath is the rotation shim around DWB, its own keys capped by the car limits (rate 0.4, accel 0.6)."""
        inputs = runpy_inputs(self.share)
        self.assertEqual((follow['plugin'], follow['primary_controller']),
                         (inputs['ROTATION_SHIM_PLUGIN'], inputs['DWB_PLUGIN']))
        self.assertEqual({k: follow[k] for k in inputs['CAR_ROTATION_SHIM']},
                         dict(inputs['CAR_ROTATION_SHIM'], rotate_to_heading_angular_vel=0.4, max_angular_accel=0.6))

    def without(self, *names):
        """patch the launch's runpy so the given nav_input_profile functions become no-ops (positive controls)."""
        runpy = self.module.runpy   # the launch's own runpy (load_navigation_launch may have imported a fresh one)
        real = runpy.run_path

        def stubbed(path, *args, **kwargs):
            inputs = real(path, *args, **kwargs)
            for name in names:
                inputs[name] = lambda params, *_: params
            return inputs
        return patch.object(runpy, 'run_path', stubbed)

    def test_car_goal_tolerance_comes_from_the_override(self):
        """Positive controls: with apply_car_goal_tolerance and apply_car_yaw_goal_tolerance made no-ops, the car
        entry keeps 0.03 / 0.03; with only the yaw one a no-op, xy 0.10 and yaw 0.03 -- so the 0.10 assertions above
        (car only, car + rolling) fail without each override."""
        car = dict(CAR_ENV, FPGA_NAV_MAP_YAML=str(self.share / 'maps/car.yaml'))
        with self.without('apply_car_goal_tolerance', 'apply_car_yaw_goal_tolerance'):
            _, params, _ = self.actions(car)
            self.assert_goal_tolerance(params, 0.03, 0.03)
            _, params, _ = self.actions(self.rolling_share())
            self.assert_goal_tolerance(params, 0.03, 0.03)
        with self.without('apply_car_yaw_goal_tolerance'):
            _, params, _ = self.actions(car)
            self.assert_goal_tolerance(params, 0.10, 0.03)
            _, params, _ = self.actions(self.rolling_share())
            self.assert_goal_tolerance(params, 0.10, 0.03)

    POSE = {'plugin': 'nav2_controller::PoseProgressChecker', 'required_movement_radius': 0.05,
            'required_movement_angle': 0.50, 'movement_time_allowance': 20.0}
    SIMPLE = {'plugin': 'nav2_controller::SimpleProgressChecker', 'required_movement_radius': 0.05,
              'movement_time_allowance': 20.0}
    SIM_PROGRESS = {'plugin': 'nav2_controller::SimpleProgressChecker', 'required_movement_radius': 0.15,
                    'movement_time_allowance': 15.0}

    @staticmethod
    def progress_and_planner(params):
        controller = params['controller_server']['ros__parameters']
        grid = params['planner_server']['ros__parameters']['GridBased']
        return controller['progress_checker'], (grid['tolerance'], grid['allow_unknown'])

    def test_car_progress_checker_and_planner(self):
        """Known answers (2026-10-25): the car gets PoseProgressChecker 0.05 m / 0.50 rad / 20 s; with
        FPGA_NAV_CAR_PLANNER=relaxed NavFn 0.25 / allow_unknown true, on the plain and the rolling costmap (rolling:
        after check_costmap_config passed the rolling yaml's 0 / false); the GridBased plugin key stays; the evidence
        line carries both. Default (strict) is covered in test_progress_checker_and_planner_switches."""
        car = dict(CAR_ENV, FPGA_NAV_MAP_YAML=str(self.share / 'maps/car.yaml'), FPGA_NAV_CAR_PLANNER='relaxed')
        for rolling in (False, True):   # rolling_share() rewrites the fixture, so the plain car runs first
            env = dict(self.rolling_share(), FPGA_NAV_CAR_PLANNER='relaxed') if rolling else car
            _, params, lines = self.actions(env)
            self.assertEqual(self.progress_and_planner(params), (self.POSE, (0.25, True)))
            if not rolling:
                self.assertEqual(params['planner_server']['ros__parameters']['GridBased']['plugin'],
                                 'nav2_navfn_planner/NavfnPlanner')
            self.assertEqual((lines[-1]['progress_checker'], lines[-1]['planner']),
                             (self.POSE, {'tolerance': 0.25, 'allow_unknown': True}))

    def min_speeds(self, params):
        controller = params['controller_server']['ros__parameters']
        follow = controller['FollowPath']
        return follow['min_speed_xy'], follow['min_speed_theta'], controller['general_goal_checker'][
            'rot_stopped_velocity']

    def test_car_dwb_min_speeds(self):
        """Known answers (2026-10-25 audit): the car's DWB (inside the rotation shim, alone, rolling-rebuilt) gets
        min_speed_xy 0.015 / min_speed_theta 0.10 and the StoppedGoalChecker rot_stopped_velocity 0.15; the simulation
        keeps 0 / 0 / 0.03; positive control: apply_car_min_speeds a no-op leaves the car at 0 / 0 / 0.03."""
        car = dict(CAR_ENV, FPGA_NAV_MAP_YAML=str(self.share / 'maps/car.yaml'))
        _, sim, _ = self.actions({})
        self.assertEqual(self.min_speeds(sim), (0.0, 0.0, 0.03))
        for env in (car, dict(car, FPGA_NAV_ROTATION_SHIM='false')):
            _, params, lines = self.actions(env)
            self.assertEqual(self.min_speeds(params), (0.015, 0.10, 0.15))
            self.assertEqual(lines[-1]['dwb_min_speeds'], {'min_speed_xy': 0.015, 'min_speed_theta': 0.10})
        with self.without('apply_car_min_speeds'):
            _, params, _ = self.actions(car)
            self.assertEqual(self.min_speeds(params), (0.0, 0.0, 0.03))
        _, params, _ = self.actions(self.rolling_share())       # rewrites the fixture: last
        follow = params['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual((follow['min_speed_xy'], follow['min_speed_theta']), (0.015, 0.10))

    def test_car_progress_checker_and_planner_come_from_the_overrides(self):
        """Positive control: with apply_car_progress_checker and apply_car_planner no-ops the car keeps the
        simulation's Simple 0.15 / 15 and NavFn 0.15 / false (rolling: the rolling yaml's 0 / false), so the
        assertions above fail without them."""
        car = dict(CAR_ENV, FPGA_NAV_MAP_YAML=str(self.share / 'maps/car.yaml'), FPGA_NAV_CAR_PLANNER='relaxed')
        with self.without('apply_car_progress_checker', 'apply_car_planner'):
            _, params, _ = self.actions(car)
            self.assertEqual(self.progress_and_planner(params), (self.SIM_PROGRESS, (0.15, False)))
            _, params, _ = self.actions(dict(self.rolling_share(), FPGA_NAV_CAR_PLANNER='relaxed'))
            self.assertEqual(self.progress_and_planner(params), (self.SIM_PROGRESS, (0.0, False)))

    def test_progress_checker_and_planner_switches(self):
        """FPGA_NAV_PROGRESS_CHECKER pose (default) | simple: SimpleProgressChecker 0.05 / 20 with no angle key left
        behind; FPGA_NAV_CAR_PLANNER strict (default, unset): the merged values unchanged | relaxed 0.25 / true; anything else refused;
        the simulation reads neither."""
        car = dict(CAR_ENV, FPGA_NAV_MAP_YAML=str(self.share / 'maps/car.yaml'))
        for value in ('pose', 'POSE'):
            _, params, _ = self.actions(dict(car, FPGA_NAV_PROGRESS_CHECKER=value))
            self.assertEqual(self.progress_and_planner(params)[0], self.POSE)
        _, params, lines = self.actions(dict(car, FPGA_NAV_PROGRESS_CHECKER='simple'))
        self.assertEqual(self.progress_and_planner(params), (self.SIMPLE, (0.15, False)))   # default strict
        self.assertIsNone(lines[-1]['planner'])
        self.assertEqual(lines[-1]['progress_checker'], self.SIMPLE)
        _, params, lines = self.actions(dict(car, FPGA_NAV_CAR_PLANNER='strict'))
        self.assertEqual(self.progress_and_planner(params), (self.POSE, (0.15, False)))
        self.assertIsNone(lines[-1]['planner'])
        # before rolling_share(), which rewrites the fixture
        for value in ('simple', 'junk'):                       # sim: the switches are not even read
            _, sim, lines = self.actions({'FPGA_NAV_PROGRESS_CHECKER': value, 'FPGA_NAV_CAR_PLANNER': value})
            self.assertEqual(self.progress_and_planner(sim), (self.SIM_PROGRESS, (0.15, False)))
            self.assertFalse(any('real_car' in line for line in lines))
        _, params, _ = self.actions(dict(self.rolling_share(), FPGA_NAV_CAR_PLANNER='strict'))
        self.assertEqual(self.progress_and_planner(params)[1], (0.0, False))
        _, params, _ = self.actions(self.rolling_share())                       # unset = strict
        self.assertEqual(self.progress_and_planner(params)[1], (0.0, False))
        _, params, _ = self.actions(dict(car, FPGA_NAV_CAR_PLANNER='relaxed'))
        self.assertEqual(self.progress_and_planner(params)[1], (0.25, True))
        for name, value in (('FPGA_NAV_PROGRESS_CHECKER', 'Pose2'), ('FPGA_NAV_PROGRESS_CHECKER', ''),
                            ('FPGA_NAV_CAR_PLANNER', 'loose'), ('FPGA_NAV_CAR_PLANNER', '')):
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                self.actions(dict(car, **{name: value}))

    def test_rotation_shim_switch(self):
        """FPGA_NAV_ROTATION_SHIM: unset/true = shim around DWB on the car (every DWB key kept, the same values);
        false = DWB alone, nothing of the shim; anything else refused; the simulation never gets it."""
        car = dict(CAR_ENV, FPGA_NAV_MAP_YAML=str(self.share / 'maps/car.yaml'))
        _, plain, lines = self.actions(dict(car, FPGA_NAV_ROTATION_SHIM='false'))
        dwb = plain['controller_server']['ros__parameters']['FollowPath']
        inputs = runpy_inputs(self.share)
        self.assertNotIn('primary_controller', dwb)
        # the fixture's FollowPath already carries rotate_to_heading_angular_vel / max_angular_accel (RPP-style keys)
        self.assertEqual(set(inputs['CAR_ROTATION_SHIM']) & set(dwb), {'rotate_to_heading_angular_vel',
                                                                      'max_angular_accel'})
        self.assertIsNone(lines[-1]['rotation_shim'])
        self.assert_goal_tolerance(plain, 0.10, 0.10)          # the yaw 0.10 does not depend on the shim
        for value in (None, 'true', 'TRUE'):
            env = car if value is None else dict(car, FPGA_NAV_ROTATION_SHIM=value)
            _, params, _ = self.actions(env)
            follow = params['controller_server']['ros__parameters']['FollowPath']
            self.assert_rotation_shim(follow)
            self.assertEqual({k: v for k, v in follow.items() if k in dwb and k != 'plugin'},
                             {k: v for k, v in dwb.items() if k != 'plugin'}, value)
        _, params, _ = self.actions(self.rolling_share())       # rolling: DWB rebuilt first, then wrapped
        follow = params['controller_server']['ros__parameters']['FollowPath']
        self.assert_rotation_shim(follow)
        self.assertIn('ObstacleFootprint', follow['critics'])
        for value in ('yes', '1', ''):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.actions(dict(car, FPGA_NAV_ROTATION_SHIM=value))
        for value in ('true', 'junk'):                         # sim: the switch is not even read
            _, sim, lines = self.actions({'FPGA_NAV_ROTATION_SHIM': value})
            sim_follow = sim['controller_server']['ros__parameters']['FollowPath']
            self.assertEqual((sim_follow['plugin'], sim_follow['rotate_to_heading_angular_vel']),
                             ('dwb_core::DWBLocalPlanner', 0.6))   # rolling_share above wrote plugin DWB
            self.assertNotIn('primary_controller', sim_follow)
            self.assertFalse(any('real_car' in line for line in lines))

    def test_rotation_shim_and_yaw_helpers(self):
        inputs = runpy_inputs(self.share)
        dwb_plugin, shim_plugin = inputs['DWB_PLUGIN'], inputs['ROTATION_SHIM_PLUGIN']
        base = {'controller_server': {'ros__parameters': {
            'goal_checker_plugins': ['general_goal_checker', 'precise'],
            'general_goal_checker': {'yaw_goal_tolerance': 0.03}, 'precise': {'yaw_goal_tolerance': 0.01},
            'FollowPath': {'plugin': dwb_plugin, 'critics': ['RotateToGoal'], 'max_vel_theta': 1.0}}}}
        wrapped = inputs['apply_rotation_shim'](base)
        self.assertEqual(base['controller_server']['ros__parameters']['FollowPath']['plugin'], dwb_plugin)   # copy
        follow = wrapped['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual((follow['plugin'], follow['primary_controller'], follow['critics'], follow['max_vel_theta']),
                         (shim_plugin, dwb_plugin, ['RotateToGoal'], 1.0))
        self.assertTrue(inputs['follow_path_is_dwb'](follow))
        self.assertFalse(inputs['follow_path_is_dwb'](dict(follow, primary_controller='other::Controller')))
        self.assertTrue(inputs['follow_path_is_dwb']({}))
        own = inputs['apply_rotation_shim'](base, {'angular_dist_threshold': 0.5})
        self.assertEqual(own['controller_server']['ros__parameters']['FollowPath']['angular_dist_threshold'], 0.5)
        self.assertNotIn('forward_sampling_distance', own['controller_server']['ros__parameters']['FollowPath'])
        rpp = {'controller_server': {'ros__parameters': {'FollowPath': {
            'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'}}}}
        for refused in (rpp, wrapped, {}):                     # RPP, already wrapped, no FollowPath
            with self.subTest(refused=refused), self.assertRaises(ValueError):
                inputs['apply_rotation_shim'](refused)
        tol = inputs['apply_car_goal_tolerance'](wrapped, 0.10)   # the shim still gets DWB's xy copy
        self.assertEqual(tol['controller_server']['ros__parameters']['FollowPath']['xy_goal_tolerance'], 0.10)
        yaw = inputs['apply_car_yaw_goal_tolerance'](base, 0.10)
        controller = yaw['controller_server']['ros__parameters']
        self.assertEqual((controller['general_goal_checker']['yaw_goal_tolerance'],
                          controller['precise']['yaw_goal_tolerance']), (0.10, 0.10))
        self.assertNotIn('yaw_goal_tolerance', controller['FollowPath'])
        self.assertEqual(base['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'],
                         0.03)
        self.assertEqual((inputs['CAR_YAW_GOAL_TOLERANCE'], inputs['CAR_XY_GOAL_TOLERANCE']), (0.10, 0.10))

    def test_limits_only_slow_down_and_sim_untouched(self):
        _, sim, lines = self.actions({})
        self.assertEqual(sim['controller_server']['ros__parameters']['FollowPath']['max_vel_x'], 0.26)
        self.assert_goal_tolerance(sim, 0.03, 0.03)                # the simulation keeps 0.03 / 0.03
        self.assertNotIn('plugin', sim['controller_server']['ros__parameters']['FollowPath'])   # no shim
        self.assertTrue(all(block['ros__parameters']['use_sim_time'] is True
                            for block in sim.values() if 'ros__parameters' in block))
        self.assertFalse(any('real_car' in line for line in lines))   # the simulation's printout is unchanged
        limits = runpy_inputs(self.share)
        slow = {'controller_server': {'ros__parameters': {'FollowPath': {'max_vel_x': 0.1, 'decel_lim_x': -0.1}}}}
        self.assertEqual(limits['apply_speed_limits'](slow, limits['CAR_SPEED_LIMITS']), slow)

    def test_velocity_guard_and_refusals(self):
        actions, _, _ = self.actions(dict(CAR_ENV, FPGA_NAV_VELOCITY_GUARD='true', FPGA_NAV_GUARD_ESTOP_TOPIC='/e'))
        guard = next(a for a in actions if a.kwargs.get('name') == 'velocity_guard')
        self.assertEqual(guard.kwargs['cmd'][1:], [str(self.share / 'scripts/velocity_guard.py'), '--ros-args',
                                                   '-p', 'estop_topic:=/e'])
        actions, _, _ = self.actions(dict(CAR_ENV, FPGA_NAV_VELOCITY_GUARD='true'))
        guard = next(a for a in actions if a.kwargs.get('name') == 'velocity_guard')
        self.assertEqual(guard.kwargs['cmd'][1:], [str(self.share / 'scripts/velocity_guard.py')])   # rcl refuses "x:="
        for env in ({'FPGA_NAV_USE_SIM_TIME': 'no'}, {'FPGA_NAV_SPEED_LIMITS': 'fast'},
                    {'FPGA_NAV_MAP_YAML': str(self.share / 'maps/missing.yaml')},
                    {'FPGA_NAV_INPUT_PROFILE': 'estimated_inspect', 'FPGA_NAV_AUTOSTART': 'false',
                     'FPGA_NAV_MAP_YAML': str(self.share / 'maps/car.yaml'), 'FPGA_NAV_STATIC_MAP_ODOM': 'true'}):
            with self.subTest(env=env), self.assertRaises(ValueError):
                self.actions(env)

    def rolling_share(self, **local_changes):
        """The share with the real rolling_layer_nav2.yaml (local_changes edit its local costmap) and Nav2-like
        local/global costmaps and planner (an ObstacleLayer on /fpga/points in both, as nav2_overrides.yaml has)."""
        rolling = yaml.safe_load((ASSETS / 'config/rolling_layer_nav2.yaml').read_text())
        rolling['local_costmap']['local_costmap']['ros__parameters'].update(local_changes)
        (self.share / 'config/rolling_layer_nav2.yaml').write_text(yaml.safe_dump(rolling))
        params = yaml.safe_load((self.share / 'params/nav2_params.yaml').read_text())
        obstacle = {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'observation_sources': 'depth',
                    'depth': {'topic': '/fpga/points'}}
        params['controller_server']['ros__parameters']['FollowPath']['plugin'] = 'dwb_core::DWBLocalPlanner'
        params.update({
            'local_costmap': {'local_costmap': {'ros__parameters': {
                'plugins': ['voxel_layer', 'inflation_layer'], 'voxel_layer': dict(obstacle),
                'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer'}}}},
            'global_costmap': {'global_costmap': {'ros__parameters': {
                'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
                'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer'}, 'obstacle_layer': dict(obstacle),
                'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer'}}}},
            'planner_server': {'ros__parameters': {'GridBased': {'tolerance': 0.5, 'allow_unknown': True}}}})
        (self.share / 'params/nav2_params.yaml').write_text(yaml.safe_dump(params))
        rpp = self.share / 'rpp.yaml'   # car_inner's nav2 branch passes nav_controller_variants' rpp_*.yaml
        rpp.write_text(yaml.safe_dump({'controller_server': {'ros__parameters': {'FollowPath': {
            'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            'lookahead_dist': 0.6, 'desired_linear_vel': 0.25}}}}))
        return dict(CAR_ENV, FPGA_NAV_COSTMAP='rolling', FPGA_NAV2_OVERRIDES=str(rpp))

    def test_rolling_costmap_with_dwb_on_the_car(self):
        _, params, lines = self.actions(self.rolling_share())
        local = params['local_costmap']['local_costmap']['ros__parameters']
        self.assertEqual(local['plugins'], ['rolling_layer', 'inflation_layer'])
        self.assertEqual(local['rolling_layer']['map_topic'], '/fpga/rolling_map/grid')
        self.assertTrue(local['track_unknown_space'])
        self.assertNotIn('voxel_layer', local)
        self.assertIs(local['use_sim_time'], False)                  # force_sim_time runs after the rolling merge
        glob = params['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(glob['plugins'], ['static_layer', 'inflation_layer'])
        self.assertNotIn('obstacle_layer', glob)
        follow = params['controller_server']['ros__parameters']['FollowPath']
        self.assert_rotation_shim(follow)                           # 10-22: the car's DWB inside the rotation shim
        self.assertIn('ObstacleFootprint', follow['critics'])
        self.assertNotIn('lookahead_dist', follow)                   # the RPP override file's keys are gone
        self.assertEqual((follow['max_vel_x'], follow['max_vel_theta']), (0.15, 0.4))   # still speed-limited
        grid = params['planner_server']['ros__parameters']['GridBased']
        # 2026-10-25: the car keeps the rolling yaml's 0 / false by default (FPGA_NAV_CAR_PLANNER strict)
        self.assertEqual((grid['tolerance'], grid['allow_unknown']), (0.0, False))
        self.assertEqual(lines[-1]['real_car']['costmap'], 'rolling')
        # the rolling evidence is taken at the rolling merge, before the shim wraps that DWB
        self.assertEqual(lines[-1]['rolling_costmap']['follow_path_plugin'], 'dwb_core::DWBLocalPlanner')
        self.assert_goal_tolerance(params, 0.10, 0.10)             # DWB rebuilt before, the car's 0.10 after

    def test_rolling_costmap_refusals(self):
        # the launch's gate is check_costmap_config: a rolling yaml breaking the 09-27 ruling is refused
        for change in ({'track_unknown_space': False}, {'footprint_padding': 0.01}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.actions(self.rolling_share(**change))
        with self.assertRaises(ValueError):
            self.actions(dict(self.rolling_share(), FPGA_NAV_COSTMAP='foo'))

    def test_share_falls_back_to_the_source_tree_without_the_package(self):
        def missing(_):
            raise KeyError('fpga_sim')   # ament_index_python's PackageNotFoundError is a KeyError
        with patch.object(self.module, 'get_package_share_directory', missing):
            self.assertEqual(self.module.package_share(), ASSETS)


def runpy_inputs(share):
    import runpy
    return runpy.run_path(str(share / 'config/nav_input_profile.py'))


if __name__ == '__main__':
    unittest.main()
