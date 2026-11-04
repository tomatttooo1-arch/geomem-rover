"""Host checks of generated launch actions; no ROS/DDS or motion claim."""
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types
import runpy
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'robot_sim/fpga_sim'
INPUTS = runpy.run_path(str(ASSETS / 'config/nav_input_profile.py'))
bridge_inputs, navigation_inputs = INPUTS['bridge_inputs'], INPUTS['navigation_inputs']


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class InputProfileTests(unittest.TestCase):
    def test_reference_bridge_unchanged(self):
        entries = yaml.safe_load((ASSETS / 'config/bridge.yaml').read_text())
        self.assertEqual(bridge_inputs(entries, 'reference'), entries)
        inspection = bridge_inputs(entries, 'estimated_inspect')
        self.assertTrue(all(x['direction'] == 'GZ_TO_ROS' for x in inspection))
        topics = {x['ros_topic_name'] for x in inspection}
        self.assertTrue({'/reference/odom', '/reference/tf'} <= topics)
        self.assertFalse({'/odom', '/tf', '/fpga/cmd_vel_safe'} & topics)
        self.assertIn('/odom', {x['ros_topic_name'] for x in entries})

    def test_fail_closed_configuration(self):
        reference = ASSETS / 'maps/room.yaml'
        for env in ({'FPGA_NAV_INPUT_PROFILE': 'estimate'},
                    {'FPGA_NAV_INPUT_PROFILE': 'estimated_inspect'},
                    {'FPGA_NAV_INPUT_PROFILE': 'estimated_inspect', 'FPGA_NAV_AUTOSTART': 'false'},
                    {'FPGA_NAV_AUTOSTART': 'yes'}):
            with self.subTest(env=env), self.assertRaises(ValueError):
                navigation_inputs(env, reference)

    def load_launch(self, name, share):
        definitions = {
            'ament_index_python.packages': {'get_package_share_directory': lambda _: str(share)},
            'launch': {'LaunchDescription': Action},
            'launch.actions': {x: Action for x in ('IncludeLaunchDescription', 'RegisterEventHandler',
                'OpaqueFunction', 'GroupAction', 'ExecuteProcess', 'EmitEvent')},
            'launch.event_handlers': {'OnShutdown': Action, 'OnProcessExit': Action},
            'launch.events': {'Shutdown': Action},
            'launch.launch_description_sources': {'PythonLaunchDescriptionSource': Action},
            'launch_ros.actions': {'Node': Action, 'SetRemap': Action},
        }
        modules = {}
        for key, attrs in definitions.items():
            module = types.ModuleType(key)
            module.__dict__.update(attrs)
            modules[key] = module
        spec = importlib.util.spec_from_file_location('profile_launch', ASSETS / f'launch/{name}.launch.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        return module

    def test_navigation_generated_actions(self):
        # Exercise the real launch builder with lightweight ROS action records.
        with tempfile.TemporaryDirectory() as directory:
            share = Path(directory)
            for sub in ('params', 'config', 'behavior_trees', 'maps'):
                (share / sub).mkdir()
            (share / 'params/nav2_params.yaml').write_text(yaml.safe_dump({
                'map_server': {'ros__parameters': {}}, 'bt_navigator': {'ros__parameters': {}}}))
            (share / 'config/nav2_overrides.yaml').write_text('{}')
            (share / 'config/nav_input_profile.py').write_text((ASSETS / 'config/nav_input_profile.py').read_text())
            (share / 'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml').write_text('<root/>')
            (share / 'maps/room.yaml').write_text('image: room.pgm')
            module = self.load_launch('navigation', share)
            for profile in ('reference', 'estimated_inspect'):
                env = {'FPGA_NAV_INPUT_PROFILE': profile, 'FPGA_NAV_AUTOSTART': 'false',
                       'FPGA_NAV_MAP_YAML': str(share / 'maps/room.yaml')}
                with self.subTest(profile=profile), patch.dict(os.environ, env, clear=True), \
                        patch.object(module, 'bounded_recovery', lambda text: text):
                    actions = module.generate_launch_description().args[0]
                    static = [a for a in actions if a.kwargs.get('executable') == 'static_transform_publisher']
                    self.assertEqual(len(static), 3 if profile == 'reference' else 0)
                    map_node = next(a for a in actions if a.kwargs.get('name') == 'map_server')
                    params_file = Path(map_node.kwargs['parameters'][0])
                    params = yaml.safe_load(params_file.read_text())
                    self.assertEqual(params['map_server']['ros__parameters']['yaml_filename'],
                                     str(share / 'maps/room.yaml'))
                    group = next(a for a in actions if a.args and isinstance(a.args[0], list))
                    remaps = [a.kwargs for a in group.args[0] if 'src' in a.kwargs]
                    self.assertEqual({r['dst'] for r in remaps},
                                     {'cmd_vel_nav'} if profile == 'reference' else {'/fpga/inspection/cmd_vel'})
                    bt_file = Path(params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'])
                    actions[-1].args[0].kwargs['on_shutdown'][0].kwargs['function'](None)
                    self.assertFalse(params_file.exists())
                    self.assertFalse(bt_file.exists())

    def test_simulation_uses_isolated_bridge_and_cleans_it(self):
        module = self.load_launch('simulation', ASSETS)
        with patch.dict(os.environ, {'FPGA_NAV_INPUT_PROFILE': 'estimated_inspect'}, clear=True):
            actions = module.generate_launch_description().args[0]
        bridge_file = Path(actions[1].kwargs['parameters'][0]['config_file'])
        entries = yaml.safe_load(bridge_file.read_text())
        self.assertTrue(all(x['direction'] == 'GZ_TO_ROS' for x in entries))
        self.assertFalse({'/odom', '/tf'} & {x['ros_topic_name'] for x in entries})
        actions[3].args[0].kwargs['on_shutdown'][0].kwargs['function'](None)
        self.assertFalse(bridge_file.exists())



class RollingCostmapTests(unittest.TestCase):
    """FPGA_NAV_COSTMAP=rolling (10-15): rolling-map local costmap + DWB, per the 09-27 ruling
    (docs/design/ROLLING_NAV2_LAYER_20260927.md F10 / 4.1 item 4). Host-only, no ROS."""

    def setUp(self):
        self.rolling = yaml.safe_load((ASSETS / 'config/rolling_layer_nav2.yaml').read_text())
        self.formal = yaml.safe_load((ASSETS / 'config/nav2_overrides.yaml').read_text())
        obstacle = {'plugin': 'nav2_costmap_2d::ObstacleLayer', 'observation_sources': 'depth',
                    'depth': {'topic': '/fpga/points'}}
        self.params = {
            'local_costmap': {'local_costmap': {'ros__parameters': {
                'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'], 'always_send_full_costmap': True,
                'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer'}, 'obstacle_layer': dict(obstacle),
                'voxel_layer': {'plugin': 'nav2_costmap_2d::VoxelLayer'},
                'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer', 'inflation_radius': 0.45}}}},
            'global_costmap': {'global_costmap': {'ros__parameters': {
                'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'], 'global_frame': 'map',
                'static_layer': {'plugin': 'nav2_costmap_2d::StaticLayer', 'map_subscribe_transient_local': True},
                'obstacle_layer': dict(obstacle),
                'inflation_layer': {'plugin': 'nav2_costmap_2d::InflationLayer'}}}},
            'controller_server': {'ros__parameters': {'FollowPath': {'plugin': 'x'}}},
            'planner_server': {'ros__parameters': {'GridBased': {'plugin': 'nav2_navfn_planner/NavfnPlanner',
                                                                 'tolerance': 0.15, 'allow_unknown': False}}}}
        self.default_fp = {'plugin': INPUTS['DWB_PLUGIN'], 'max_vel_x': 0.26, 'critics': ['BaseObstacle']}

    def apply(self, overlay_fp):
        return INPUTS['apply_rolling_costmap'](self.params, self.rolling, self.default_fp, overlay_fp,
                                               self.formal['controller_server']['ros__parameters']['FollowPath'])

    def test_switch_parsing(self):
        car_inputs = INPUTS['car_inputs']
        self.assertEqual(car_inputs({}, 'reference')['costmap'], 'sim')
        self.assertEqual(car_inputs({'FPGA_NAV_COSTMAP': 'rolling'}, 'reference')['costmap'], 'rolling')
        for value in ('foo', 'dwb', ''):
            with self.subTest(value=value), self.assertRaises(ValueError):
                car_inputs({'FPGA_NAV_COSTMAP': value}, 'reference')

    def test_rpp_overrides_rebuilt_as_dwb_with_formal_tuning(self):
        before = yaml.safe_dump(self.params)
        rpp = {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
               'lookahead_dist': 0.6, 'desired_linear_vel': 0.25}
        result, evidence = self.apply(rpp)
        self.assertEqual(yaml.safe_dump(self.params), before)        # input untouched
        local = result['local_costmap']['local_costmap']['ros__parameters']
        self.assertEqual(local['plugins'], ['rolling_layer', 'inflation_layer'])
        self.assertEqual(local['rolling_layer']['map_topic'], '/fpga/rolling_map/grid')
        self.assertFalse({'static_layer', 'obstacle_layer', 'voxel_layer'} & set(local))
        self.assertTrue(local['always_send_full_costmap'])           # non-layer keys survive
        glob = result['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(glob['plugins'], ['static_layer', 'inflation_layer'])
        self.assertNotIn('obstacle_layer', glob)
        self.assertEqual(glob['global_frame'], 'map')
        fp = result['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(fp['plugin'], INPUTS['DWB_PLUGIN'])
        self.assertIn('ObstacleFootprint', fp['critics'])
        self.assertFalse({'lookahead_dist', 'desired_linear_vel'} & set(fp))   # no RPP key left behind
        self.assertEqual(fp['max_vel_x'], self.formal['controller_server']['ros__parameters']['FollowPath']['max_vel_x'])
        grid = result['planner_server']['ros__parameters']['GridBased']
        self.assertEqual((grid['tolerance'], grid['allow_unknown'], grid['plugin']),
                         (0.0, False, 'nav2_navfn_planner/NavfnPlanner'))
        self.assertEqual(evidence['follow_path_tuning'], 'nav2_overrides.yaml')
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        from host.rolling_nav_layer import check_costmap_config
        self.assertEqual(check_costmap_config(local, follow_path=fp, planner=grid), [])

    def test_dwb_overrides_keep_their_tuning(self):
        result, evidence = self.apply({'max_vel_x': 0.2})
        fp = result['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual((fp['plugin'], fp['max_vel_x']), (INPUTS['DWB_PLUGIN'], 0.2))
        self.assertEqual(evidence['follow_path_tuning'], 'overrides')



class CarGoalToleranceTests(unittest.TestCase):
    """FPGA_NAV_SPEED_LIMITS=car (owner 10-15): xy goal tolerance 0.10 m on the car, 0.03 in the simulation."""

    def setUp(self):
        self.formal = yaml.safe_load((ASSETS / 'config/nav2_overrides.yaml').read_text())
        self.apply = INPUTS['apply_car_goal_tolerance']

    def test_simulation_file_keeps_003_and_car_gets_010(self):
        controller = self.formal['controller_server']['ros__parameters']
        self.assertEqual((controller['general_goal_checker']['xy_goal_tolerance'],
                          controller['FollowPath']['xy_goal_tolerance']), (0.03, 0.03))
        self.assertEqual(INPUTS['CAR_XY_GOAL_TOLERANCE'], 0.10)
        car = self.apply(self.formal, INPUTS['CAR_XY_GOAL_TOLERANCE'])['controller_server']['ros__parameters']
        self.assertEqual((car['general_goal_checker']['xy_goal_tolerance'], car['FollowPath']['xy_goal_tolerance'],
                          car['general_goal_checker']['yaw_goal_tolerance']), (0.10, 0.10, 0.03))
        self.assertEqual(controller['general_goal_checker']['xy_goal_tolerance'], 0.03)   # a copy

    def test_named_goal_checkers_dwb_only_follow_path(self):
        params = {'controller_server': {'ros__parameters': {
            'goal_checker_plugins': ['precise', 'loose'], 'precise': {'xy_goal_tolerance': 0.03},
            'loose': {'plugin': 'nav2_controller::SimpleGoalChecker'}, 'general_goal_checker': {'xy_goal_tolerance': 1},
            'FollowPath': {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'}}}}
        controller = self.apply(params, 0.10)['controller_server']['ros__parameters']
        self.assertEqual((controller['precise']['xy_goal_tolerance'], controller['loose']['xy_goal_tolerance'],
                          controller['general_goal_checker']['xy_goal_tolerance']), (0.10, 0.10, 1))
        self.assertNotIn('xy_goal_tolerance', controller['FollowPath'])   # RPP has no such key
        dwb = self.apply({'controller_server': {'ros__parameters': {'FollowPath': {}}}}, 0.10)
        self.assertEqual(dwb['controller_server']['ros__parameters']['FollowPath']['xy_goal_tolerance'], 0.10)



class CarProgressAndPlannerTests(unittest.TestCase):
    """2026-10-25 car progress checker / planner helpers; the simulation's files keep their values."""

    def test_simulation_files_unchanged(self):
        formal = yaml.safe_load((ASSETS / 'config/nav2_overrides.yaml').read_text())
        controller = formal['controller_server']['ros__parameters']
        self.assertEqual(controller['progress_checker'],
                         {'required_movement_radius': 0.15, 'movement_time_allowance': 15.0})
        self.assertEqual(formal['planner_server']['ros__parameters']['GridBased'],
                         {'tolerance': 0.15, 'allow_unknown': False})
        rolling = yaml.safe_load((ASSETS / 'config/rolling_layer_nav2.yaml').read_text())
        self.assertEqual(rolling['planner_server']['ros__parameters']['GridBased'],
                         {'tolerance': 0.0, 'allow_unknown': False})   # still the 09-27 ruling, gated by the launch

    def test_progress_checker_replaced_not_merged(self):
        params = {'controller_server': {'ros__parameters': {
            'progress_checker': {'plugin': 'nav2_controller::PoseProgressChecker', 'required_movement_angle': 0.5}}}}
        out = INPUTS['apply_car_progress_checker'](params, INPUTS['CAR_PROGRESS_CHECKER']['simple'])
        self.assertEqual(out['controller_server']['ros__parameters']['progress_checker'],
                         {'plugin': 'nav2_controller::SimpleProgressChecker', 'required_movement_radius': 0.05,
                          'movement_time_allowance': 20.0})
        self.assertEqual(params['controller_server']['ros__parameters']['progress_checker']['required_movement_angle'],
                         0.5)                                                    # a copy
        named = INPUTS['apply_car_progress_checker'](
            {'controller_server': {'ros__parameters': {'progress_checker_plugin': 'pc'}}},
            INPUTS['CAR_PROGRESS_CHECKER']['pose'])['controller_server']['ros__parameters']
        self.assertEqual(named['pc']['plugin'], 'nav2_controller::PoseProgressChecker')
        self.assertNotIn('progress_checker', named)

    def test_planner_and_switch_parsing(self):
        params = {'planner_server': {'ros__parameters': {'GridBased': {'plugin': 'navfn', 'tolerance': 0.0,
                                                                       'allow_unknown': False, 'use_astar': False}}}}
        grid = INPUTS['apply_car_planner'](params, INPUTS['CAR_PLANNER'])['planner_server']['ros__parameters'][
            'GridBased']
        self.assertEqual(grid, {'plugin': 'navfn', 'tolerance': 0.25, 'allow_unknown': True, 'use_astar': False})
        self.assertEqual(INPUTS['apply_car_planner'](params, None), params)
        self.assertEqual(params['planner_server']['ros__parameters']['GridBased']['tolerance'], 0.0)   # a copy
        self.assertEqual(INPUTS['car_progress_checker']({})['plugin'], 'nav2_controller::PoseProgressChecker')
        self.assertEqual(INPUTS['car_progress_checker']({'FPGA_NAV_PROGRESS_CHECKER': 'simple'})['plugin'],
                         'nav2_controller::SimpleProgressChecker')
        self.assertIsNone(INPUTS['car_planner']({}))                              # default strict (09-27 ruling)
        self.assertIsNone(INPUTS['car_planner']({'FPGA_NAV_CAR_PLANNER': 'strict'}))
        self.assertEqual(INPUTS['car_planner']({'FPGA_NAV_CAR_PLANNER': 'relaxed'}),
                         {'tolerance': 0.25, 'allow_unknown': True})
        for fn, name in ((INPUTS['car_progress_checker'], 'FPGA_NAV_PROGRESS_CHECKER'),
                         (INPUTS['car_planner'], 'FPGA_NAV_CAR_PLANNER')):
            with self.assertRaises(ValueError):
                fn({name: 'bogus'})


    def test_min_speeds_dwb_only_stopped_checkers_only(self):
        shim, dwb = INPUTS['ROTATION_SHIM_PLUGIN'], INPUTS['DWB_PLUGIN']
        params = {'controller_server': {'ros__parameters': {
            'goal_checker_plugins': ['general_goal_checker', 'simple'],
            'general_goal_checker': {'plugin': 'nav2_controller::StoppedGoalChecker', 'rot_stopped_velocity': 0.03},
            'simple': {'plugin': 'nav2_controller::SimpleGoalChecker'},
            'FollowPath': {'plugin': shim, 'primary_controller': dwb}}}}
        out = INPUTS['apply_car_min_speeds'](params)['controller_server']['ros__parameters']
        self.assertEqual((out['FollowPath']['min_speed_xy'], out['FollowPath']['min_speed_theta'],
                          out['general_goal_checker']['rot_stopped_velocity']), (0.015, 0.10, 0.15))
        self.assertNotIn('rot_stopped_velocity', out['simple'])
        self.assertEqual(params['controller_server']['ros__parameters']['general_goal_checker'][
            'rot_stopped_velocity'], 0.03)                                         # a copy
        rpp = {'controller_server': {'ros__parameters': {
            'general_goal_checker': {'plugin': 'nav2_controller::StoppedGoalChecker', 'rot_stopped_velocity': 0.03},
            'FollowPath': {'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'}}}}
        self.assertEqual(INPUTS['apply_car_min_speeds'](rpp), rpp)                # not DWB: untouched
        # the dead-band argument: the lowest legal slow rotation sample stays below rot_stopped_velocity
        self.assertLess(INPUTS['CAR_DWB_MIN_SPEEDS']['min_speed_theta'], INPUTS['CAR_ROT_STOPPED_VELOCITY'])


if __name__ == '__main__':
    unittest.main()
