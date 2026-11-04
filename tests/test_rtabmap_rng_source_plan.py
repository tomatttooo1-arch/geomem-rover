"""The RNG-source round's plan: argv, container environment and arm schedule.

These are the parts of the runner that decide what actually gets executed, and they are all
pure, so they can be pinned without Docker. The point of pinning them is that the round's
whole claim is "the ONLY thing that changed between A1 and A2 is nothing at all".
"""
from pathlib import Path
import unittest

from tools import rtabmap_nav2_rehearsal_run as rehearsal
from tools import rtabmap_ros_probe as probe
from tools import rtabmap_rng_source_run as rng

ROOT = Path('/tmp/rng-source-test-root')


class ArmArgvTests(unittest.TestCase):
    def test_every_arm_starts_from_the_frozen_baseline_argv(self):
        frozen = rehearsal.probe_args('baseline')
        for arm in rng.ARM_ORDER:
            self.assertEqual(rng.arm_argv(arm)[:len(frozen)], frozen, msg=arm)

    def test_the_two_a_arms_are_argv_identical(self):
        self.assertEqual(rng.arm_argv('a1'), rng.arm_argv('a2'))

    def test_the_two_c_arms_are_argv_identical(self):
        self.assertEqual(rng.arm_argv('c1'), rng.arm_argv('c2'))

    def test_the_a_arms_differ_from_the_frozen_argv_only_by_the_thread_count(self):
        frozen = rehearsal.probe_args('baseline')
        self.assertEqual(rng.arm_argv('a1'), frozen + ['--omp-num-threads', '1'])

    def test_b1_is_the_frozen_configuration(self):
        self.assertEqual(rng.arm_argv('b1'),
                         rehearsal.probe_args('baseline') + ['--omp-num-threads', '2'])
        self.assertEqual(rng.arm_environment('b1'), [])

    def test_the_c_arms_pin_brute_force_nearest_neighbour_on_both_nodes(self):
        argv = rng.arm_argv('c1')
        self.assertEqual([argv[index + 1] for index, item in enumerate(argv)
                          if item == '--odom-param'], ['Vis/CorNNType=3'])
        self.assertEqual([argv[index + 1] for index, item in enumerate(argv)
                          if item == '--slam-param'],
                         ['Vis/CorNNType=3', 'Kp/NNStrategy=3'])

    def test_every_arm_argv_is_accepted_by_the_probe_it_is_handed_to(self):
        for arm in rng.ARM_ORDER:
            parsed = probe.parser().parse_args(rng.arm_argv(arm))
            probe.validate(parsed)
            self.assertEqual(parsed.omp_num_threads, rng.ARMS[arm]['omp'], msg=arm)

    def test_the_parameter_overrides_reach_the_right_parameter_file(self):
        parsed = probe.parser().parse_args(rng.arm_argv('c1'))
        odom, slam = probe.parameter_sets(parsed, Path('/rosresults/normal/map.db'))
        self.assertEqual(odom['Vis/CorNNType'], '3')
        self.assertEqual(slam['Vis/CorNNType'], '3')
        self.assertEqual(slam['Kp/NNStrategy'], '3')
        self.assertNotIn('Kp/NNStrategy', odom)

    def test_the_a_arms_leave_the_frozen_parameter_files_untouched(self):
        for arm in ('a1', 'a2', 'b1'):
            parsed = probe.parser().parse_args(rng.arm_argv(arm))
            frozen = probe.parser().parse_args(rehearsal.probe_args('baseline'))
            self.assertEqual(probe.parameter_sets(parsed, Path('/db')),
                             probe.parameter_sets(frozen, Path('/db')), msg=arm)


class ContainerCommandTests(unittest.TestCase):
    def command(self, arm):
        return rng.create_command(f'fpga-dev1-rng-{arm}', arm, ROOT / arm)

    def test_the_thread_count_is_set_on_the_container_as_well_as_the_probe(self):
        argv = self.command('a1')
        self.assertIn('OMP_NUM_THREADS=1', argv)
        self.assertEqual(argv[argv.index('OMP_NUM_THREADS=1') - 1], '--env')
        self.assertEqual(argv[-2:], ['--omp-num-threads', '1'])

    def test_the_frozen_arm_adds_no_container_environment(self):
        self.assertNotIn('OMP_NUM_THREADS=2', self.command('b1'))

    def test_the_container_discipline_is_the_rehearsal_s(self):
        argv = self.command('a1')
        for expected in ('--network', 'none', '--init', '--cpus', '4', '--memory', '8g'):
            self.assertIn(expected, argv)
        self.assertIn('--name', argv)
        self.assertEqual(argv[argv.index('--name') + 1], 'fpga-dev1-rng-a1')

    def test_the_database_and_dataset_are_mounted_read_only(self):
        argv = self.command('a1')
        mounts = [argv[index + 1] for index, item in enumerate(argv) if item == '--mount']
        readonly = [mount for mount in mounts if mount.endswith(',readonly')]
        self.assertEqual(len(mounts) - len(readonly), 1)
        self.assertTrue(any('dst=/results,readonly' in mount for mount in mounts))
        self.assertTrue(any(mount.endswith('dst=/rosresults') for mount in mounts))

    def test_a1_and_a2_differ_only_in_the_container_name_and_result_path(self):
        left, right = self.command('a1'), self.command('a2')
        renamed = [item.replace('-a1', '-a2').replace('/a1', '/a2') for item in left]
        self.assertEqual(renamed, right)


class ScheduleTests(unittest.TestCase):
    def test_the_a_arms_run_first_and_unconditionally(self):
        self.assertEqual(rng.arms_to_run([], 0., None), 'a1')
        self.assertEqual(rng.arms_to_run(['a1'], 100., None), 'a2')

    def test_the_c_arms_run_only_when_the_a_pair_differs(self):
        self.assertEqual(rng.arms_to_run(['a1', 'a2'], 100., False), 'c1')
        self.assertEqual(rng.arms_to_run(['a1', 'a2', 'c1'], 200., False), 'c2')
        self.assertEqual(rng.arms_to_run(['a1', 'a2'], 100., True), 'b1')
        self.assertEqual(rng.arms_to_run(['a1', 'a2'], 100., None), 'b1')

    def test_b1_runs_last_and_only_inside_its_own_time_limit(self):
        self.assertEqual(rng.arms_to_run(['a1', 'a2', 'c1', 'c2'], 900., False), 'b1')
        self.assertIsNone(rng.arms_to_run(['a1', 'a2', 'c1', 'c2'],
                                          rng.B1_ELAPSED_LIMIT_S, False))

    def test_the_round_budget_stops_everything(self):
        self.assertIsNone(rng.arms_to_run([], rng.ROUND_BUDGET_S, None))

    def test_the_schedule_terminates_when_every_arm_has_run(self):
        self.assertIsNone(rng.arms_to_run(list(rng.ARM_ORDER), 0., False))

    def test_a_full_schedule_never_repeats_an_arm(self):
        executed, elapsed = [], 0.
        while True:
            arm = rng.arms_to_run(executed, elapsed, False if len(executed) >= 2 else None)
            if arm is None:
                break
            self.assertNotIn(arm, executed)
            executed.append(arm)
            elapsed += 240.
        self.assertEqual(executed, list(rng.ARM_ORDER))


class PairTests(unittest.TestCase):
    def test_pairs_are_unordered_and_follow_the_arm_order(self):
        self.assertEqual(rng.pair_names(['a2', 'a1', 'b1']),
                         [('a1', 'a2'), ('a1', 'b1'), ('a2', 'b1')])

    def test_one_arm_produces_no_pair(self):
        self.assertEqual(rng.pair_names(['a1']), [])


if __name__ == '__main__':
    unittest.main()
