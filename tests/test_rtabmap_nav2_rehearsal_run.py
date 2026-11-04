"""The rehearsal runner's pure parts, pinned before the containers run.

Three things are worth testing without a container: the argv plan for each of the four
pre-registered runs (because an argv typo is indistinguishable from a design failure in the
results), the approved-stamp extraction (because G1/G2 are decided by it and the frozen run
supplies a known answer of exactly 51), and the verdict rule (because it is the only place
where "what happened" becomes "PASS / FAIL / INSTRUMENT_*", and the protocol fixed it in
advance).
"""
import json
from pathlib import Path
import tempfile
import unittest

from tools import rtabmap_nav2_rehearsal_run as rehearsal
from tools import rtabmap_ros_probe as probe

FROZEN_DIR = (Path(__file__).resolve().parents[2] / 'rtabmap-reload-coordinate-20260912'
              / 'query-normal-stage-v1' / 'normal')
FROZEN_EVENTS = FROZEN_DIR / 'gate-events.jsonl'
FROZEN_COMMANDS = FROZEN_DIR / 'commands.json'


HOST_EXTRINSICS = (Path(__file__).resolve().parents[1] / 'robot_sim' / 'fpga_sim' / 'config'
                   / 'extrinsics_sim.json')


def plan(run, out=Path('/rosresults/normal')):
    """The probe's node plan for one run, read on the host.

    `plan_commands` actually opens the extrinsics manifest, and the runner names it by its
    in-container path, so the host copy is substituted here. That is the only difference
    between what this test plans and what the container will run.
    """
    argv = [str(HOST_EXTRINSICS) if item == rehearsal.EXTRINSICS_IN_CONTAINER else item
            for item in rehearsal.probe_args(run)]
    a = probe.parser().parse_args(argv)
    probe.validate(a)
    return probe.plan_commands(a, out, out / 'map.db', rehearsal.DATABASE_SHA256)


def clean_state(**overrides):
    state = {'all_containers_clean': True, 'databases_unchanged': True,
             'negative_controls_discriminating': True, 'config_identity_ok': True,
             'g1': True, 'g2': True, 'loaded_checks_pass': True}
    state.update(overrides)
    return state


class ProbeArgvTests(unittest.TestCase):
    def test_every_run_parses_and_validates(self):
        for run in rehearsal.RUNS:
            with self.subTest(run=run):
                a = probe.parser().parse_args(rehearsal.probe_args(run))
                probe.validate(a)
                self.assertEqual(a.name, 'normal')
                self.assertEqual(a.mode, 'normal')
                self.assertEqual((a.reset_countdown, a.odom_tf_wait, a.start_index), (10, 0., 0))
                self.assertEqual((a.timeout_seconds, a.online_gate, a.gate_rgb_reliable),
                                 (600., True, True))
                self.assertEqual(a.input, '/query/rgbd_dataset_freiburg1_desk')

    def test_the_frozen_block_is_identical_across_runs_apart_from_stop_index(self):
        frozen_block = rehearsal.probe_args('baseline')
        self.assertEqual(frozen_block[-2:], ['--online-gate', '--gate-rgb-reliable'])
        for run in rehearsal.RUNS:
            argv = rehearsal.probe_args(run)
            self.assertEqual(argv[:len(frozen_block)],
                             [item if item != '230' else str(rehearsal.stop_index(run))
                              for item in frozen_block])

    def test_stop_indices_follow_the_protocol(self):
        self.assertEqual([rehearsal.stop_index(run) for run in rehearsal.RUNS],
                         [230, 230, 140, 140])

    @unittest.skipUnless(FROZEN_COMMANDS.is_file(), 'frozen commands.json not present')
    def test_baseline_reproduces_the_frozen_51_of_230_command_plan_byte_for_byte(self):
        names, commands, replay = plan('baseline')
        frozen = json.loads(FROZEN_COMMANDS.read_text())
        self.assertEqual(commands, frozen['nodes'])
        self.assertEqual(replay, frozen['replay'])
        self.assertEqual(names, ['static_tf', 'odometry', 'localization', 'gate'])

    def test_loaded_adds_exactly_the_protocol_processes_and_gate_arguments(self):
        names, commands, _ = plan('loaded')
        self.assertEqual(names, ['static_tf', 'odometry', 'localization', 'gate',
                                 'static_tf_base_camera', 'odom_adapter', 'recorder'])
        gate = commands[names.index('gate')]
        head = gate[:gate.index('--ros-args')]
        self.assertEqual(head[-8:], ['--rgb-reliable', '--no-publish-tracked-frame',
                                     '--map-odom-hold-hz', '20', '--map-odom-future-s', '0.1',
                                     '--bundle', '/bundle'])
        self.assertEqual(gate[-2:], ['-r', '/fpga/localization_tf:=/tf'])
        self.assertIn('--extrinsics-manifest', commands[names.index('odom_adapter')])

    def test_the_negative_controls_differ_from_loaded_in_exactly_one_way_each(self):
        loaded = set(rehearsal.probe_args('loaded'))
        self.assertEqual(set(rehearsal.probe_args('neg-static-map-odom')) - loaded,
                         {'--extra-static-identity', 'map:odom', '140'})
        self.assertEqual(loaded - set(rehearsal.probe_args('neg-no-adapter')),
                         {'--adapter', '230'})
        names, _, _ = plan('neg-static-map-odom')
        self.assertIn('static_tf_extra_map_odom', names)
        names, commands, _ = plan('neg-no-adapter')
        self.assertNotIn('odom_adapter', names)
        self.assertIn('recorder', names)

    def test_an_unknown_run_is_refused_rather_than_silently_planned(self):
        with self.assertRaises(ValueError):
            rehearsal.probe_args('loaded-ish')


class ApprovedStampTests(unittest.TestCase):
    @unittest.skipUnless(FROZEN_EVENTS.is_file(), 'frozen gate-events.jsonl not present')
    def test_the_frozen_run_yields_exactly_the_pre_registered_51(self):
        stamps = rehearsal.approved_stamps(FROZEN_EVENTS)
        self.assertEqual(len(stamps), rehearsal.FROZEN_APPROVED_COUNT)
        self.assertEqual(stamps, sorted(set(stamps)))

    @unittest.skipUnless(FROZEN_EVENTS.is_file(), 'frozen gate-events.jsonl not present')
    def test_forty_of_the_frozen_approvals_fall_inside_the_negative_controls_140_frames(self):
        early = set(rehearsal.acquisition_stamps(FROZEN_EVENTS)[:rehearsal.NEGATIVE_STOP_INDEX])
        self.assertEqual(len(early), rehearsal.NEGATIVE_STOP_INDEX)
        self.assertEqual(len([s for s in rehearsal.approved_stamps(FROZEN_EVENTS) if s in early]), 40)

    def test_only_received_events_contribute_and_duplicates_collapse(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'gate-events.jsonl'
            path.write_text(''.join(json.dumps(row) + '\n' for row in [
                {'event': 'acquisition', 'stamp_ns': 7},
                {'event': 'received_odom', 'approved': [{'stamp_ns': 5}, {'stamp_ns': 3}]},
                {'event': 'received_info', 'approved': [{'stamp_ns': 3}]},
                {'event': 'state_transition'},
                # A non-received event carrying approvals must not contribute.
                {'event': 'policy_snapshot', 'approved': [{'stamp_ns': 99}]},
            ]) + '\n')
            self.assertEqual(rehearsal.approved_stamps(path), [3, 5])
            self.assertEqual(rehearsal.acquisition_stamps(path), [7])

    def test_stamp_comparison_names_the_difference_in_both_directions(self):
        comparison = rehearsal.stamp_comparison([1, 2, 4], [1, 2, 3])
        self.assertFalse(comparison['equal'])
        self.assertEqual((comparison['only_in_observed'], comparison['only_in_reference']),
                         ([4], [3]))
        self.assertTrue(rehearsal.stamp_comparison([1, 2], [1, 2])['equal'])


class NegativeControlTests(unittest.TestCase):
    def test_a_passing_negative_control_is_never_discriminating(self):
        self.assertFalse(rehearsal.negative_is_discriminating(
            'neg-static-map-odom', {'pass': True, 'failed': [], 'not_evaluable': []}))

    def test_the_static_map_odom_control_must_trip_the_static_versus_dynamic_check(self):
        self.assertTrue(rehearsal.negative_is_discriminating(
            'neg-static-map-odom',
            {'pass': False, 'failed': ['no_child_both_static_and_dynamic'], 'not_evaluable': []}))
        self.assertFalse(rehearsal.negative_is_discriminating(
            'neg-static-map-odom',
            {'pass': False, 'failed': ['stamp_clock_relation'], 'not_evaluable': []}))

    def test_the_no_adapter_control_must_trip_both_the_edge_and_an_odom_check(self):
        self.assertTrue(rehearsal.negative_is_discriminating(
            'neg-no-adapter', {'pass': False, 'failed': ['required_edges_present'],
                               'not_evaluable': ['odom_message_frames']}))
        self.assertFalse(rehearsal.negative_is_discriminating(
            'neg-no-adapter', {'pass': False, 'failed': ['required_edges_present'],
                               'not_evaluable': []}))

    def test_a_missing_verdict_is_not_taken_for_discrimination(self):
        self.assertFalse(rehearsal.negative_is_discriminating('neg-no-adapter', None))


class VerdictLevelTests(unittest.TestCase):
    def test_everything_holding_is_the_only_way_to_pass(self):
        self.assertEqual(rehearsal.verdict_level(clean_state())[0], 'PASS')

    def test_a_broken_container_outranks_every_other_conclusion(self):
        state = clean_state(all_containers_clean=False, g1=False, g2=False,
                            loaded_checks_pass=False, negative_controls_discriminating=False)
        self.assertEqual(rehearsal.verdict_level(state)[0], 'INSTRUMENT_ERROR')

    def test_a_changed_read_only_database_is_an_instrument_error(self):
        self.assertEqual(rehearsal.verdict_level(clean_state(databases_unchanged=False))[0],
                         'INSTRUMENT_ERROR')

    def test_a_non_discriminating_negative_control_forbids_a_verdict_on_the_design(self):
        state = clean_state(negative_controls_discriminating=False, g2=False)
        self.assertEqual(rehearsal.verdict_level(state)[0], 'INSTRUMENT_ERROR')

    def test_g1_failing_is_drift_and_outranks_g2(self):
        level, reason = rehearsal.verdict_level(clean_state(g1=False, g2=False))
        self.assertEqual(level, 'INSTRUMENT_DRIFT')
        self.assertIn('G1', reason)

    def test_configuration_drift_is_drift_not_failure(self):
        self.assertEqual(rehearsal.verdict_level(clean_state(config_identity_ok=False))[0],
                         'INSTRUMENT_DRIFT')

    def test_g2_failing_is_a_verdict_on_the_design(self):
        level, reason = rehearsal.verdict_level(clean_state(g2=False))
        self.assertEqual(level, 'FAIL')
        self.assertIn('G2', reason)

    def test_the_interface_checks_failing_on_loaded_is_a_failure(self):
        self.assertEqual(rehearsal.verdict_level(clean_state(loaded_checks_pass=False))[0], 'FAIL')

    def test_an_empty_state_never_reads_as_pass(self):
        self.assertEqual(rehearsal.verdict_level({})[0], 'INSTRUMENT_ERROR')


class VerdictInputTests(unittest.TestCase):
    """`verdict_inputs` is what turns the recorded run files into the booleans above."""

    def summary(self, **run_overrides):
        def record(run, **overrides):
            base = {'run': run, 'container_exit_code': 0, 'timed_out': False,
                    'products_present': True, 'database_unchanged': True,
                    'config_identity_ok': True,
                    'runner_result': {'all_owned_processes_exited': True, 'replay_exit_code': 0},
                    'checker': None, 'checker_allow_missing': None}
            base.update(overrides)
            return base
        runs = {'baseline': record('baseline', g1=True),
                'loaded': record('loaded', g2=True, checker={'verdict': {
                    'pass': True, 'failed': [], 'not_evaluable': []}}),
                'neg-static-map-odom': record('neg-static-map-odom', checker={'verdict': {
                    'pass': False, 'failed': ['no_child_both_static_and_dynamic'],
                    'not_evaluable': []}}),
                'neg-no-adapter': record('neg-no-adapter', checker={'verdict': {
                    'pass': False, 'failed': ['required_edges_present'],
                    'not_evaluable': ['odom_message_frames']}})}
        for run, overrides in run_overrides.items():
            runs[run].update(overrides)
        return {'runs': runs}

    def test_a_complete_clean_round_reads_as_pass(self):
        inputs = rehearsal.verdict_inputs(self.summary(), list(rehearsal.RUNS))
        self.assertEqual(rehearsal.verdict_level(inputs)[0], 'PASS')

    def test_a_timed_out_container_is_seen_even_with_a_zero_exit_code(self):
        summary = self.summary(loaded={'timed_out': True})
        inputs = rehearsal.verdict_inputs(summary, list(rehearsal.RUNS))
        self.assertFalse(inputs['all_containers_clean'])

    def test_a_nonzero_replay_exit_code_is_not_silently_accepted(self):
        summary = self.summary(baseline={'runner_result': {'all_owned_processes_exited': True,
                                                           'replay_exit_code': 1}})
        self.assertFalse(rehearsal.verdict_inputs(summary, list(rehearsal.RUNS))['all_containers_clean'])

    def test_running_a_subset_can_never_read_as_a_clean_full_round(self):
        inputs = rehearsal.verdict_inputs(self.summary(), ['baseline'])
        self.assertFalse(inputs['all_containers_clean'])
        self.assertEqual(rehearsal.verdict_level(inputs)[0], 'INSTRUMENT_ERROR')

    def test_the_allow_missing_rerun_is_the_verdict_that_counts_when_present(self):
        summary = self.summary(loaded={
            'checker': {'verdict': {'pass': False, 'failed': ['motion_permit_publisher_unique'],
                                    'not_evaluable': ['motion_permit_publisher_unique']}},
            'checker_allow_missing': {'verdict': {'pass': True, 'failed': [],
                                                  'not_evaluable': ['motion_permit_publisher_unique']}}})
        inputs = rehearsal.verdict_inputs(summary, list(rehearsal.RUNS))
        self.assertTrue(inputs['loaded_checks_pass'])
        self.assertEqual(inputs['loaded_not_evaluable_names'], ['motion_permit_publisher_unique'])


if __name__ == '__main__':
    unittest.main()
