"""The probe's command plan, pinned without a container.

`plan_commands` is pure, so the exact argv of every child process can be compared here
against the one the frozen 51/230 run actually recorded. That comparison is the only proof
that adding the joint-rehearsal processes did not move the configuration the acquisition
numbers were measured under; it is a regression test, not a decoration.
"""
import json
from pathlib import Path
import tempfile
import unittest

import yaml

from tools import rtabmap_ros_probe as probe

CHECKOUT = Path(__file__).resolve().parents[1]
FROZEN_RUN = (Path(__file__).resolve().parents[2] / 'rtabmap-reload-coordinate-20260912'
              / 'query-normal-stage-v1' / 'normal')
FROZEN = FROZEN_RUN / 'commands.json'
FROZEN_MAP_ID = '2a450fd79970fe4c145af5afab6b76a9420b2880c8bbb70581e9aafa1bfab8c7'
FROZEN_ARGV = ['normal', '--mode', 'normal', '--online-gate', '--gate-rgb-reliable',
               '--start-index', '0', '--stop-index', '230',
               '--input', '/query/rgbd_dataset_freiburg1_desk']
OUT = Path('/rosresults/normal')
SIM_EXTRINSICS = CHECKOUT / 'robot_sim' / 'fpga_sim' / 'config' / 'extrinsics_sim.json'
RAW_TF_REMAP = ['-r', '/tf:=/fpga/localization_raw_tf']


def plan(argv, out=OUT, map_id=FROZEN_MAP_ID):
    a = probe.parser().parse_args(argv)
    probe.validate(a)
    return probe.plan_commands(a, out, out / 'map.db', map_id)


def named(names, commands, name):
    return commands[names.index(name)]


def parsed(argv):
    a = probe.parser().parse_args(argv)
    probe.validate(a)
    return a


class FrozenConfigurationTests(unittest.TestCase):
    @unittest.skipUnless(FROZEN.is_file(), 'frozen commands.json not present')
    def test_the_frozen_51_of_230_plan_is_reproduced_exactly(self):
        names, commands, replay = plan(FROZEN_ARGV)
        frozen = json.loads(FROZEN.read_text())
        self.assertEqual(commands, frozen['nodes'])
        self.assertEqual(replay, frozen['replay'])
        self.assertEqual(names, ['static_tf', 'odometry', 'localization', 'gate'])

    def test_both_rtabmap_nodes_keep_their_raw_tf_isolated(self):
        names, commands, _ = plan(FROZEN_ARGV)
        for name in ('odometry', 'localization'):
            argv = named(names, commands, name)
            self.assertIn('-r', argv)
            self.assertEqual([argv[index:index + 2] for index in range(len(argv) - 1)
                              if argv[index:index + 2] == RAW_TF_REMAP], [RAW_TF_REMAP])

    def test_the_new_flags_default_to_off(self):
        a = probe.parser().parse_args(FROZEN_ARGV)
        self.assertEqual((a.gate_arg, a.gate_remap, a.adapter, a.static_extrinsics, a.recorder,
                          a.extra_static_identity), ([], [], False, None, False, []))

    def test_no_extra_static_publisher_exists_unless_it_is_asked_for(self):
        names, commands, _ = plan(FROZEN_ARGV)
        self.assertFalse([name for name in names if name.startswith('static_tf_extra_')])
        self.assertEqual(len(commands), 4)


class GatePassThroughTests(unittest.TestCase):
    def gate(self, extra):
        names, commands, _ = plan(FROZEN_ARGV + extra)
        return named(names, commands, 'gate')

    def test_gate_arguments_land_before_ros_args_and_remaps_after(self):
        argv = self.gate(['--gate-arg=--no-publish-tracked-frame', '--gate-arg=--map-odom-hold-hz',
                          '--gate-arg=20', '--gate-remap', '/fpga/localization_tf:=/tf'])
        boundary = argv.index('--ros-args')
        self.assertEqual(argv[boundary - 3:boundary],
                         ['--no-publish-tracked-frame', '--map-odom-hold-hz', '20'])
        self.assertEqual(argv[boundary:], ['--ros-args', '-p', 'use_sim_time:=true',
                                           '-r', '/fpga/localization_tf:=/tf'])
        # the existing gate flags keep their place ahead of the new ones
        self.assertEqual(argv[boundary - 4], '--rgb-reliable')

    def test_several_remaps_are_appended_in_order(self):
        argv = self.gate(['--gate-remap', '/fpga/localization_tf:=/tf',
                          '--gate-remap', '/fpga/localization_status:=/fpga/status'])
        self.assertEqual(argv[-4:], ['-r', '/fpga/localization_tf:=/tf',
                                     '-r', '/fpga/localization_status:=/fpga/status'])

    def test_a_malformed_remap_is_refused(self):
        for bad in ('/tf', '/a:=', ':=/b', '/a:=/b:=/c'):
            with self.assertRaises(ValueError):
                plan(FROZEN_ARGV + ['--gate-remap', bad])

    def test_diagnostics_and_gate_arguments_do_not_reorder_each_other(self):
        names, commands, _ = plan(FROZEN_ARGV + ['--gate-diagnostics', '--gate-arg=--bundle',
                                                 '--gate-arg=/rosresults/bundle'])
        argv = named(names, commands, 'gate')
        boundary = argv.index('--ros-args')
        self.assertEqual(argv[boundary - 4:boundary],
                         ['--rgb-reliable', '--diagnostics', '--bundle', '/rosresults/bundle'])


class RehearsalProfileTests(unittest.TestCase):
    ARGV = ['--gate-arg=--no-publish-tracked-frame', '--gate-arg=--map-odom-hold-hz',
            '--gate-arg=20', '--gate-arg=--map-odom-future-s', '--gate-arg=0.1',
            '--gate-remap', '/fpga/localization_tf:=/tf', '--adapter', '--recorder',
            '--static-extrinsics', str(SIM_EXTRINSICS)]

    def setUp(self):
        self.names, self.commands, self.replay = plan(FROZEN_ARGV + self.ARGV)

    def test_the_extra_processes_join_the_same_cleanup_list(self):
        self.assertEqual(self.names, ['static_tf', 'odometry', 'localization', 'gate',
                                      'static_tf_base_camera', 'odom_adapter', 'recorder'])
        self.assertEqual(len(self.names), len(self.commands))

    def test_the_adapter_is_launched_with_sim_time_and_the_manifest(self):
        self.assertEqual(named(self.names, self.commands, 'odom_adapter'),
                         ['python3', '-m', 'host.ros_odom_base_adapter', '--prefix', '/probe',
                          '--tracked-frame', 'camera_link', '--base-frame', 'base_link',
                          '--odom-frame', 'odom', '--extrinsics-manifest', str(SIM_EXTRINSICS),
                          '--ros-args', '-p', 'use_sim_time:=true'])

    def test_the_static_extrinsic_comes_only_from_the_manifest(self):
        manifest = json.loads(SIM_EXTRINSICS.read_text())
        xyz, rpy = manifest['base_to_tracked']['xyz'], manifest['base_to_tracked']['rpy']
        self.assertEqual(named(self.names, self.commands, 'static_tf_base_camera'),
                         ['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                          '--x', repr(float(xyz[0])), '--y', repr(float(xyz[1])),
                          '--z', repr(float(xyz[2])), '--roll', repr(float(rpy[0])),
                          '--pitch', repr(float(rpy[1])), '--yaw', repr(float(rpy[2])),
                          '--frame-id', 'base_link', '--child-frame-id', 'camera_link'])

    def test_the_recorder_writes_into_the_run_directory(self):
        self.assertEqual(named(self.names, self.commands, 'recorder'),
                         ['python3', '/probe_tools/tf_topic_recorder.py',
                          '--output', '/rosresults/normal/tf-record.jsonl'])

    def test_the_provider_nodes_are_untouched_by_the_rehearsal_profile(self):
        plain_names, plain_commands, plain_replay = plan(FROZEN_ARGV)
        for name in ('static_tf', 'odometry', 'localization'):
            self.assertEqual(named(self.names, self.commands, name),
                             named(plain_names, plain_commands, name))
        self.assertEqual(self.replay, plain_replay)

    def test_an_adapter_without_a_manifest_still_runs_but_unlabelled(self):
        names, commands, _ = plan(FROZEN_ARGV + ['--adapter'])
        self.assertNotIn('static_tf_base_camera', names)
        self.assertNotIn('--extrinsics-manifest', named(names, commands, 'odom_adapter'))


class ExtraStaticIdentityTests(unittest.TestCase):
    """The negative control that makes the silent failure happen on purpose."""

    def test_an_identity_edge_is_published_with_an_all_zero_pose(self):
        names, commands, _ = plan(FROZEN_ARGV + ['--recorder',
                                                 '--extra-static-identity', 'map:odom'])
        self.assertEqual(names[-2:], ['static_tf_extra_map_odom', 'recorder'])
        self.assertEqual(named(names, commands, 'static_tf_extra_map_odom'),
                         ['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                          '--x', '0', '--y', '0', '--z', '0',
                          '--roll', '0', '--pitch', '0', '--yaw', '0',
                          '--frame-id', 'map', '--child-frame-id', 'odom'])

    def test_several_identities_keep_their_order_and_get_distinct_names(self):
        names, _, _ = plan(FROZEN_ARGV + ['--extra-static-identity', 'map:odom',
                                          '--extra-static-identity', '/odom:/base_link'])
        self.assertEqual(names[-2:], ['static_tf_extra_map_odom',
                                      'static_tf_extra_odom_base_link'])

    def test_it_is_published_beside_the_other_static_edges_not_after_the_recorder(self):
        names, _, _ = plan(FROZEN_ARGV + RehearsalProfileTests.ARGV
                           + ['--extra-static-identity', 'map:odom'])
        self.assertEqual(names, ['static_tf', 'odometry', 'localization', 'gate',
                                 'static_tf_base_camera', 'static_tf_extra_map_odom',
                                 'odom_adapter', 'recorder'])

    def test_a_malformed_specification_is_refused_before_anything_starts(self):
        for bad in ('map', 'map:', ':odom', 'map:odom:extra', ''):
            with self.assertRaises(ValueError, msg=bad):
                plan(FROZEN_ARGV + ['--extra-static-identity', bad])

    def test_the_process_name_is_usable_as_a_log_filename(self):
        _, name = probe.identity_static_command('/map/frame:odom.link')
        self.assertEqual(name, 'static_tf_extra_map_frame_odom_link')


class ExtrinsicsManifestTests(unittest.TestCase):
    def manifest(self, **changes):
        body = {'schema': 1, 'provenance': 'modelled_sim',
                'frames': {'base': 'base_link', 'tracked': 'camera_link'},
                'base_to_tracked': {'xyz': [1., 2., 3.], 'rpy': [.1, .2, .3]}}
        body.update(changes)
        directory = tempfile.mkdtemp()
        path = Path(directory) / 'extrinsics.json'
        path.write_text(json.dumps(body))
        return path

    def test_frames_are_taken_from_the_manifest(self):
        path = self.manifest(frames={'base': 'chassis', 'tracked': 'sensor'})
        argv = probe.static_extrinsics_command(path)
        self.assertEqual(argv[-4:], ['--frame-id', 'chassis', '--child-frame-id', 'sensor'])
        self.assertEqual(argv[1:13], ['--x', '1.0', '--y', '2.0', '--z', '3.0',
                                      '--roll', '0.1', '--pitch', '0.2', '--yaw', '0.3'])

    def test_an_unlabelled_provenance_is_refused(self):
        for provenance in ('tf_static_unlabelled', 'identity', None, 'guessed'):
            with self.assertRaises(ValueError):
                probe.static_extrinsics_command(self.manifest(provenance=provenance))

    def test_a_measured_manifest_is_accepted(self):
        probe.static_extrinsics_command(self.manifest(provenance='measured'))

    def test_a_wrong_schema_or_shape_is_refused(self):
        with self.assertRaises(ValueError):
            probe.static_extrinsics_command(self.manifest(schema=2))
        with self.assertRaises(ValueError):
            probe.static_extrinsics_command(
                self.manifest(base_to_tracked={'xyz': [1., 2.], 'rpy': [0., 0., 0.]}))

    def test_the_simulation_manifest_is_the_one_the_adapter_reads(self):
        self.assertTrue(SIM_EXTRINSICS.is_file())
        self.assertEqual(json.loads(SIM_EXTRINSICS.read_text())['provenance'], 'modelled_sim')


class FrozenParameterFileTests(unittest.TestCase):
    """The parameter files are the other half of the frozen configuration.

    `plan_commands` pins the child argv; these pin what the two `--params-file` arguments
    actually contain. Without them an override flag could change the frozen query's
    measured configuration and every argv test would still be green.
    """

    def files(self, argv, out=OUT):
        a = parsed(argv)
        odom, slam = probe.parameter_sets(a, out / 'map.db')
        return odom, slam

    def dumped(self, params):
        return yaml.safe_dump({'/**': {'ros__parameters': params}})

    @unittest.skipUnless((FROZEN_RUN / 'odom.yaml').is_file(), 'frozen yaml not present')
    def test_the_frozen_parameter_files_are_reproduced_byte_for_byte(self):
        odom, slam = self.files(FROZEN_ARGV + ['--reset-countdown', '10', '--odom-tf-wait', '0'])
        self.assertEqual(self.dumped(odom), (FROZEN_RUN / 'odom.yaml').read_text())
        self.assertEqual(self.dumped(slam), (FROZEN_RUN / 'slam.yaml').read_text())

    def test_the_override_flags_default_to_empty_and_change_nothing(self):
        a = parsed(FROZEN_ARGV)
        self.assertEqual((a.odom_param, a.slam_param, a.omp_num_threads), ([], [], 2))
        self.assertEqual(self.files(FROZEN_ARGV), self.files(FROZEN_ARGV))

    def test_an_override_lands_in_exactly_one_file_as_a_string(self):
        base_odom, base_slam = self.files(FROZEN_ARGV)
        odom, slam = self.files(FROZEN_ARGV + ['--odom-param', 'Vis/CorNNType=3',
                                               '--slam-param', 'Kp/NNStrategy=3'])
        self.assertEqual(odom, dict(base_odom, **{'Vis/CorNNType': '3'}))
        self.assertEqual(slam, dict(base_slam, **{'Kp/NNStrategy': '3'}))
        self.assertNotIn('Kp/NNStrategy', odom)
        self.assertNotIn('Vis/CorNNType', slam)

    def test_an_override_can_replace_a_pinned_parameter(self):
        _, slam = self.files(FROZEN_ARGV + ['--slam-param', 'Rtabmap/DetectionRate=1'])
        self.assertEqual(slam['Rtabmap/DetectionRate'], '1')

    def test_malformed_overrides_are_refused(self):
        for spec in ('Vis/CorNNType', '=3', 'Vis/CorNNType=', 'Vis CorNNType=3',
                     'Vis//CorNNType=3', 'Vis/Cor-NNType=3', ''):
            with self.assertRaises(ValueError, msg=spec):
                probe.parse_param_override(spec)
            with self.assertRaises(ValueError, msg=spec):
                parsed(FROZEN_ARGV + ['--slam-param', spec])

    def test_overrides_are_applied_in_order_and_do_not_mutate_the_input(self):
        base = {'A/B': '1'}
        self.assertEqual(probe.apply_param_overrides(base, ['A/B=2', 'A/B=3']), {'A/B': '3'})
        self.assertEqual(base, {'A/B': '1'})

    def test_the_plan_argv_is_untouched_by_parameter_overrides(self):
        names, commands, replay = plan(FROZEN_ARGV)
        with_overrides = plan(FROZEN_ARGV + ['--odom-param', 'Vis/CorNNType=3',
                                             '--slam-param', 'Kp/NNStrategy=3',
                                             '--omp-num-threads', '1'])
        self.assertEqual((names, commands, replay), with_overrides)


class ChildEnvironmentTests(unittest.TestCase):
    def environment(self, argv, base=None):
        return probe.child_environment(parsed(argv), base if base is not None else {})

    def test_the_frozen_environment_is_the_default(self):
        self.assertEqual(self.environment(FROZEN_ARGV),
                         {'ROS_DOMAIN_ID': '82', 'ROS_LOCALHOST_ONLY': '1',
                          'OMP_NUM_THREADS': '2', 'OPENBLAS_NUM_THREADS': '1'})

    def test_the_probe_overrides_an_inherited_omp_setting(self):
        """A container-level --env cannot reach the children on its own; this is why."""
        inherited = {'OMP_NUM_THREADS': '1', 'PATH': '/usr/bin'}
        self.assertEqual(self.environment(FROZEN_ARGV, inherited)['OMP_NUM_THREADS'], '2')
        self.assertEqual(
            self.environment(FROZEN_ARGV + ['--omp-num-threads', '1'],
                             {'OMP_NUM_THREADS': '2'})['OMP_NUM_THREADS'], '1')

    def test_unrelated_environment_is_carried_through_without_mutation(self):
        base = {'PATH': '/usr/bin'}
        self.assertEqual(self.environment(FROZEN_ARGV, base)['PATH'], '/usr/bin')
        self.assertEqual(base, {'PATH': '/usr/bin'})

    def test_an_out_of_range_thread_count_is_refused(self):
        for value in ('0', '-1', '65'):
            with self.assertRaises(ValueError, msg=value):
                parsed(FROZEN_ARGV + ['--omp-num-threads', value])


class MainShadowingTests(unittest.TestCase):
    """`main` must not rebind the name of a helper it also calls.

    This is not hypothetical: the first attempt at the RNG-source round died in every
    container with `UnboundLocalError: child_environment` because `main` assigned a local of
    that name further down, which makes the earlier call to the module-level function a read
    of an unassigned local. Nothing in argv-level testing can see it — `main` is only ever
    executed inside the container — so it is pinned statically here instead.
    """

    @staticmethod
    def shadowed_helpers(module, function):
        """Module-level callables whose name `function` also uses as a local.

        The check is on the local-name tables and deliberately not on `co_names`: once a
        name is assigned anywhere in a function it is compiled as a local everywhere in it,
        so the shadowed call is a LOAD_FAST and the name disappears from `co_names`. Looking
        for "called AND assigned" therefore finds nothing — which is how the bug shipped.

        `co_varnames` alone is not the whole local-name table. A local that a nested scope
        also reads — a comprehension, for instance — is a *cell* variable, so it lives in
        `co_cellvars` and never appears in `co_varnames`. Before PEP 709 (Python 3.12) a
        comprehension always had its own code object, and `main` reads the shadowed name
        from a dict comprehension, so on every interpreter this code actually runs on
        (3.10 in the ROS container, 3.8 on the host) the name to catch is a cellvar and
        `co_varnames` alone saw nothing. That is why the positive control below failed on
        3.8 while passing on 3.13. All three local-name tables are therefore searched:
        strictly more names than before, so nothing this used to reject is now accepted.
        """
        helpers = {name for name, value in vars(module).items() if callable(value)
                   and getattr(value, '__module__', None) == module.__name__}
        code = function.__code__
        return helpers.intersection(code.co_varnames + code.co_cellvars + code.co_freevars)

    def test_no_module_level_helper_is_shadowed_by_a_local_in_main(self):
        self.assertEqual(self.shadowed_helpers(probe, probe.main), set())

    def test_the_guard_catches_the_bug_it_was_written_for(self):
        """Positive control on the real source: reintroduce the rename, guard must fire."""
        import types
        source = Path(probe.__file__).read_text().replace('process_environment',
                                                          'child_environment')
        module = types.ModuleType('shadowed_probe')
        exec(compile(source, 'shadowed_probe', 'exec'), module.__dict__)
        self.assertEqual(self.shadowed_helpers(module, module.main), {'child_environment'})


class OnlineGateRequirementTests(unittest.TestCase):
    def test_every_new_flag_requires_the_online_gate(self):
        for extra in (['--gate-arg=--bundle'], ['--gate-remap', '/a:=/b'], ['--adapter'],
                      ['--static-extrinsics', str(SIM_EXTRINSICS)], ['--recorder'],
                      ['--extra-static-identity', 'map:odom']):
            with self.assertRaises(ValueError, msg=extra):
                plan(['probe', '--start-index', '0', '--stop-index', '10'] + extra)

    def test_the_existing_gate_flag_requirement_is_unchanged(self):
        with self.assertRaises(ValueError):
            plan(['probe', '--gate-rgb-reliable'])


if __name__ == '__main__':
    unittest.main()
