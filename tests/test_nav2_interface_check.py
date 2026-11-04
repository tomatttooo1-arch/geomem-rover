"""Synthetic recordings for the Nav2 static interface check, and the recorder's pure pieces.

The generator below writes exactly the rows `tools/tf_topic_recorder.py` would write, using
the same constructors, for a rehearsal that behaves as the plan says it should. It follows
the measured replay geometry, not a round number: the frozen replay publishes /clock as the
RGB capture stamp, so the simulated step is the dataset's 33.3 ms, held `map->odom` is
stamped 0.1 s ahead, `odom->base_link` is stamped at the frame time, and the recorder's
sample is taken in the /clock callback, BEFORE that step's publications.

Both can_transform answers are modelled from the stamp sets by tf2's own rule — a query
resolves only when samples bracket it on both sides, which also gives the single-sample
case correctly. `c2_at_clock` comes out false at every sample, because `odom->base_link` is
always behind the clock; that is why it is recorded as evidence and never judged.
`c2_at_odom`, asked at the newest odometry stamp, is false for the first four samples after
localization becomes valid and true from the fifth — which is what `--c2-grace-samples`
exists for — and goes false again four samples (0.133 s) after the hold stops.

Every negative is a single mutation of that scenario, and each one asserts the EXACT set of
checks it is expected to break — a negative that also trips an unrelated check would mean
the checks are not separating what they claim to.
"""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from tools import nav2_interface_core as core
from tools.nav2_interface_check import evaluate, main, parse_topic_info, parser

CLOCK0 = 1_000_000_000_000
STEP = 33_333_333           # one /clock per replayed frame, at the dataset's 30 Hz stamps
FUTURE = 100_000_000        # --map-odom-future-s 0.1
STEPS = 62                  # long enough for the whole C1 staleness window after the loss
SHORT_STEPS = 46            # stops 0.4 s before that window closes; see cut_scenario
FIRST_VALID, LAST_VALID = 2, 20
APPROVED = [CLOCK0 + index * STEP for index in (5, 9, 15)]
GRACE = 6                   # --c2-grace-samples default

TRANSLATION = {'x': .18, 'y': 0., 'z': .95}
ROTATION = {'x': 0., 'y': .1246747, 'z': 0., 'w': .9921977}

CHECKOUT = Path(__file__).resolve().parents[1]


def state_at(step):
    if step < FIRST_VALID:
        return 'initializing'
    return 'valid' if step <= LAST_VALID else 'lost'


class Clockwork:
    """Monotonic receive stamps; the recorder's own steady clock, never the simulated one."""

    def __init__(self):
        self.value = 10**9

    def __call__(self):
        self.value += 10**6
        return self.value


def localization_status(step, clock):
    approved = [{'stamp_ns': stamp} for stamp in APPROVED if stamp <= clock] \
        if state_at(step) == 'valid' else []
    return {'schema': 1, 'state': state_at(step), 'reason': 'ok', 'epoch': 1,
            'clock_ns': clock, 'world_frame': 'map', 'tracked_frame': 'camera_link',
            'pose_source': 'estimated', 'navigation_ready': False,
            'odom_source': 'rtabmap_rgbd_odometry', 'approved': approved}


ADAPTER_STATUS = {'schema': 1, 'pose_source': 'estimated', 'odom_source': 'rtabmap_rgbd_odometry',
                  'twist_source': 'rtabmap_velocity_guess', 'extrinsics_provenance': 'modelled_sim',
                  'extrinsics_sha256': '0' * 64, 'extrinsics_error': None, 'last_lost': None}


def transform(parent, child, stamp, static):
    return core.transform_row(parent, child, stamp, static, TRANSLATION, ROTATION)


def held_stamps(step):
    """Held map->odom stamps already on /tf when the sample for `step` is taken.

    The hold timer publishes once per clock step, after the /clock message that moved the
    node's ROS now, so the sample taken inside the /clock callback sees only earlier steps.
    """
    return [CLOCK0 + index * STEP + FUTURE for index in range(step) if state_at(index) == 'valid']


def odom_base_stamps(step):
    """odom->base_link stamps already on /tf; one per frame, stamped at the frame time."""
    return [CLOCK0 + index * STEP for index in range(step)]


def resolvable(stamps, when):
    """tf2 answers a query only when its samples bracket it on both sides.

    One sample therefore answers only its own exact time, and a query past the newest
    sample is the extrapolation tf2 refuses — both fall out of this one expression.
    """
    return any(stamp <= when for stamp in stamps) and any(stamp >= when for stamp in stamps)


def sample_for(step):
    """C1 and the two can_transform answers, as tf2 would give them from those stamp sets.

    C1 is the Time(0) lookup, which resolves a chain at its latest COMMON time: the held
    edge is 0.1 s ahead but the odometry edge is at the frame time, so the odometry edge
    governs and the age comes out at one clock step.
    """
    clock = CLOCK0 + step * STEP
    held, odom = held_stamps(step), odom_base_stamps(step)
    newest_odom = max(odom) if odom else None
    lookup = min(max(held), newest_odom) if held and odom else LookupError('LookupException')
    at_clock = resolvable(held, clock) and resolvable(odom, clock)
    at_odom = None if newest_odom is None else (resolvable(held, newest_odom)
                                                and resolvable(odom, newest_odom))
    return lookup, at_clock, at_odom, newest_odom


def scenario(steps=STEPS):
    """A recording that must pass every check. `steps` is where the recording stops."""
    tick = Clockwork()
    rows = [core.tf_record('/tf_static', tick(), None,
                           [transform('base_link', 'camera_link', CLOCK0, True)]),
            core.tf_record('/tf_static', tick(), None,
                           [transform('camera_link', 'camera_optical_frame', CLOCK0, True)])]
    samples = 0
    for step in range(steps):
        clock = CLOCK0 + step * STEP
        rows.append(core.clock_record(tick(), clock))
        rows.append(core.sample_record(tick(), clock, *sample_for(step)))
        samples += 1
        if state_at(step) == 'valid':
            rows.append(core.tf_record('/tf', tick(), clock,
                                       [transform('map', 'odom', clock + FUTURE, False)]))
        rows.append(core.tf_record('/tf', tick(), clock,
                                   [transform('odom', 'base_link', clock, False)]))
        rows.append(core.odom_record('/odom', tick(), clock, 'odom', 'base_link', clock))
        rows.append(core.status_record('/fpga/localization_status', tick(), clock,
                                       json.dumps(localization_status(step, clock))))
        rows.append(core.status_record('/fpga/odom_adapter_status', tick(), clock,
                                       json.dumps(ADAPTER_STATUS)))
    counts = {'/tf': sum(1 for row in rows if row['topic'] == '/tf'),
              '/tf_static': 2, '/odom': steps, '/clock': steps, 'sample': samples,
              '/fpga/localization_status': steps, '/fpga/odom_adapter_status': steps}
    rows.append(core.summary_record(tick(), CLOCK0 + (steps - 1) * STEP, counts, 0, samples))
    return rows


def without(rows, predicate):
    return [row for row in rows if not predicate(row)]


def is_edge(row, parent, child):
    return row['topic'] in ('/tf', '/tf_static') and any(
        (t['parent'], t['child']) == (parent, child) for t in row['transforms'])


TOPIC_INFO = '''=== topic: /tf ===
Type: tf2_msgs/msg/TFMessage

Publisher count: 2

returncode: 0

=== topic: /fpga/motion_permit ===
Type: std_msgs/msg/Bool

Publisher count: %d

returncode: 0
'''


class InterfaceCheckTests(unittest.TestCase):
    def check(self, rows, *extra, permit_publishers=1, topic_info=True):
        with tempfile.TemporaryDirectory() as directory:
            record = Path(directory) / 'tf-record.jsonl'
            record.write_text(''.join(json.dumps(row) + '\n' for row in rows))
            if topic_info:
                Path(str(record) + '.topic-info.txt').write_text(TOPIC_INFO % permit_publishers)
            out = Path(directory) / 'verdict.json'
            arguments = ['check', str(record), '--out', str(out), *extra]
            options = parser().parse_args(arguments)
            options.topic_info = Path(str(record) + '.topic-info.txt')
            verdict = evaluate(rows, options)
            with open('/dev/null', 'w') as quiet:
                code = main(arguments, stream=quiet)
            self.assertEqual(code, 0 if verdict['pass'] else 1)
            self.assertEqual(json.loads(out.read_text())['pass'], verdict['pass'])
            return verdict

    def assert_fails(self, verdict, expected, intended):
        self.assertIn(intended, expected)
        self.assertEqual(set(verdict['failed']), set(expected),
                         'unexpected failure set: ' + json.dumps(
                             {name: verdict['checks'][name] for name in verdict['failed']},
                             indent=2, sort_keys=True)[:4000])
        self.assertFalse(verdict['pass'])

    # --- positive control ------------------------------------------------------------
    def test_the_rehearsal_scenario_passes_every_check(self):
        verdict = self.check(scenario())
        self.assertTrue(verdict['pass'], json.dumps(verdict['checks'], indent=2)[:4000])
        self.assertEqual(verdict['not_evaluable'], [])
        self.assertEqual(sorted(verdict['checks']), sorted(core_checks_expected()))

    def test_expected_approved_count_is_an_opt_in_check(self):
        verdict = self.check(scenario(), '--expect-approved', str(len(APPROVED)))
        self.assertTrue(verdict['pass'])
        self.assertEqual(verdict['checks']['approved_frame_count']['evidence']
                         ['distinct_approved_stamps'], len(APPROVED))
        wrong = self.check(scenario(), '--expect-approved', '51')
        self.assert_fails(wrong, {'approved_frame_count'}, 'approved_frame_count')

    # --- negatives -------------------------------------------------------------------
    def test_a_static_map_to_odom_edge_is_caught(self):
        rows = scenario()
        rows.insert(2, core.tf_record('/tf_static', 10**9, None,
                                      [transform('map', 'odom', CLOCK0, True)]))
        self.assert_fails(self.check(rows), {'no_child_both_static_and_dynamic'},
                          'no_child_both_static_and_dynamic')

    def test_a_dynamic_camera_link_edge_beside_the_static_one_is_caught(self):
        rows = scenario()
        rows.insert(2, core.tf_record('/tf', 10**9, CLOCK0,
                                      [transform('odom', 'camera_link', CLOCK0, False)]))
        self.assert_fails(self.check(rows), {'no_child_both_static_and_dynamic'},
                          'no_child_both_static_and_dynamic')

    def test_two_dynamic_parents_for_one_child_are_caught(self):
        rows = scenario()
        rows.insert(2, core.tf_record('/tf', 10**9, CLOCK0,
                                      [transform('map', 'base_link', CLOCK0, False)]))
        self.assert_fails(self.check(rows), {'single_parent_per_dynamic_child'},
                          'single_parent_per_dynamic_child')

    def test_a_missing_odom_to_base_link_edge_is_caught(self):
        # The edge is the odometry half of the TF tree, so its absence also breaks the two
        # invariants derived from it. The intended check is the missing required edge.
        rows = without(scenario(), lambda row: is_edge(row, 'odom', 'base_link'))
        self.assert_fails(self.check(rows), {'required_edges_present', 'stamp_clock_relation',
                                             'odom_message_count_matches_tf'},
                          'required_edges_present')

    def test_a_repeated_stamp_is_caught(self):
        rows = scenario()
        repeated = next(row for row in rows if is_edge(row, 'map', 'odom'))
        rows.insert(rows.index(repeated) + 1, json.loads(json.dumps(repeated)))
        summary = rows[-1]
        summary['repeated_stamp_count'] = 1
        summary['repeated_stamp_examples'] = [{'parent': 'map', 'child': 'odom',
                                               'stamp_ns': repeated['transforms'][0]['stamp_ns'],
                                               'extra_copies': 1}]
        self.assert_fails(self.check(rows), {'tf_repeated_data_absent'}, 'tf_repeated_data_absent')

    def test_a_tf_repeated_data_line_in_a_passed_log_is_caught(self):
        rows = scenario()
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / 'gate.log'
            log.write_text('[INFO] started\n[WARN] [tf2_ros]: TF_REPEATED_DATA ignoring data '
                           'with redundant timestamp for frame odom\n')
            verdict = self.check(rows, '--gate-log', str(log))
        self.assert_fails(verdict, {'tf_repeated_data_absent'}, 'tf_repeated_data_absent')

    def test_more_odom_messages_than_transforms_is_caught(self):
        rows = scenario()
        duplicate = next(row for row in rows if row['topic'] == '/odom')
        rows.insert(rows.index(duplicate) + 1, json.loads(json.dumps(duplicate)))
        self.assert_fails(self.check(rows), {'odom_message_count_matches_tf'},
                          'odom_message_count_matches_tf')

    def test_a_hold_that_did_not_stop_after_the_loss_is_caught(self):
        # The held map->odom kept being republished after localization stopped being valid,
        # so the consumer can still resolve the pose it must no longer trust.
        rows = scenario()
        for row in rows:
            if row['topic'] == core.SAMPLE and row['clock_ns'] > CLOCK0 + LAST_VALID * STEP:
                row['c2_at_odom'] = True
        verdict = self.check(rows)
        self.assert_fails(verdict, {'lookup_samples_follow_status'},
                          'lookup_samples_follow_status')
        transition = verdict['checks']['lookup_samples_follow_status']['evidence']['transitions'][0]
        self.assertFalse(transition['c2']['pass'])
        self.assertTrue(transition['c2']['still_resolvable'])

    def test_a_lookup_that_never_goes_stale_after_the_loss_is_caught(self):
        rows = scenario()
        held = CLOCK0 + LAST_VALID * STEP + FUTURE
        for row in rows:
            if row['topic'] == core.SAMPLE and row['clock_ns'] > held:
                row['c1_stamp_ns'] = row['clock_ns']
                row['c1_age_s'] = 0.
        self.assert_fails(self.check(rows), {'lookup_samples_follow_status'},
                          'lookup_samples_follow_status')

    def test_c2_that_never_becomes_true_after_the_grace_window_is_caught(self):
        rows = scenario()
        for row in rows:
            if row['topic'] == core.SAMPLE:
                row['c2_at_odom'] = False
        verdict = self.check(rows)
        self.assert_fails(verdict, {'lookup_samples_follow_status'},
                          'lookup_samples_follow_status')
        violations = verdict['checks']['lookup_samples_follow_status']['evidence']
        self.assertTrue(all(not item['in_grace_window']
                            for item in violations['valid_violations']))
        self.assertEqual(violations['valid_violation_count'], violations['valid_samples_judged'])

    def test_the_grace_window_is_what_forgives_the_first_samples(self):
        # With no grace the very same recording fails, which is what makes the default
        # load-bearing rather than decorative.
        self.assert_fails(self.check(scenario(), '--c2-grace-samples', '0'),
                          {'lookup_samples_follow_status'}, 'lookup_samples_follow_status')
        self.assertTrue(self.check(scenario(), '--c2-grace-samples', str(GRACE))['pass'])

    def test_a_held_edge_too_far_ahead_of_the_clock_is_caught_by_the_lower_bound(self):
        rows = scenario()
        target = next(row for row in rows if row['topic'] == core.SAMPLE
                      and row['clock_ns'] == CLOCK0 + (FIRST_VALID + GRACE + 4) * STEP)
        target['c1_stamp_ns'] = target['clock_ns'] + 500_000_000
        target['c1_age_s'] = -.5
        verdict = self.check(rows)
        self.assert_fails(verdict, {'lookup_samples_follow_status'},
                          'lookup_samples_follow_status')
        self.assertIn('< -0.167', verdict['checks']['lookup_samples_follow_status']['evidence']
                      ['valid_violations'][0]['reasons'][0])
        # widening the bound accepts the same recording, so the bound is what rejected it
        self.assertTrue(self.check(rows, '--c1-min-age-s', '-1.0')['pass'])

    def test_the_scenario_reproduces_the_measured_c2_geometry(self):
        rows = scenario()
        samples = [row for row in rows if row['topic'] == core.SAMPLE]
        valid = [row for row in samples
                 if FIRST_VALID < (row['clock_ns'] - CLOCK0) // STEP <= LAST_VALID + 1]
        # entry: false for the first four samples, true from the fifth
        self.assertEqual([row['c2_at_odom'] for row in valid][:6],
                         [False, False, False, False, True, True])
        self.assertTrue(all(row['c2_at_odom'] for row in valid[4:]))
        self.assertAlmostEqual(valid[4]['c1_age_s'], STEP / 1e9)
        # the query at the consumer's own now never resolves, which is why it is not judged
        self.assertTrue(all(row['c2_at_clock'] is False for row in samples))
        # drop: the fourth sample after the last held stamp, 0.133 s later
        left = CLOCK0 + (LAST_VALID + 1) * STEP
        after = [row for row in samples if row['clock_ns'] >= left]
        self.assertEqual([row['c2_at_odom'] for row in after][:5],
                         [True, True, True, True, False])
        self.assertFalse(any(row['c2_at_odom'] for row in after[4:]))
        self.assertAlmostEqual((after[4]['clock_ns'] - left) / 1e9, 4 * STEP / 1e9, places=6)

    def test_a_stale_lookup_while_valid_is_caught_by_the_threshold(self):
        rows = scenario()
        # The Time(0) age is one clock step, so a threshold just above it passes and one
        # just below it fails: the bound is doing the work, not the recording.
        self.assertTrue(self.check(rows, '--c1-max-age-s', '0.034')['pass'])
        self.assert_fails(self.check(rows, '--c1-max-age-s', '0.03'),
                          {'lookup_samples_follow_status'}, 'lookup_samples_follow_status')
        target = next(row for row in rows if row['topic'] == core.SAMPLE
                      and row['clock_ns'] == CLOCK0 + (FIRST_VALID + GRACE + 4) * STEP)
        target['c1_stamp_ns'] = target['clock_ns'] - 10**9
        target['c1_age_s'] = 1.
        self.assert_fails(self.check(rows), {'lookup_samples_follow_status'},
                          'lookup_samples_follow_status')

    def test_a_wrong_odom_child_frame_is_caught(self):
        rows = scenario()
        for row in rows:
            if row['topic'] == '/odom':
                row['child_frame_id'] = 'camera_link'
        self.assert_fails(self.check(rows), {'odom_message_frames'}, 'odom_message_frames')

    def test_a_status_without_the_declared_labels_is_caught(self):
        rows = scenario()
        for row in rows:
            if row['topic'] == '/fpga/odom_adapter_status':
                row['payload'].pop('extrinsics_sha256')
        self.assert_fails(self.check(rows), {'status_fields_present'}, 'status_fields_present')

    def test_a_second_motion_permit_publisher_is_caught(self):
        self.assert_fails(self.check(scenario(), permit_publishers=2),
                          {'motion_permit_publisher_unique'}, 'motion_permit_publisher_unique')

    # --- missing inputs are failures, not silence -------------------------------------
    def test_a_missing_topic_info_snapshot_is_not_evaluable_and_fails(self):
        verdict = self.check(scenario(), topic_info=False)
        self.assertFalse(verdict['pass'])
        self.assertEqual(verdict['not_evaluable'], ['motion_permit_publisher_unique'])
        allowed = self.check(scenario(), '--allow-missing', 'motion_permit_publisher_unique',
                             topic_info=False)
        self.assertTrue(allowed['pass'])
        self.assertEqual(allowed['not_evaluable'], ['motion_permit_publisher_unique'])

    def test_a_recording_without_a_summary_cannot_clear_the_repeated_data_check(self):
        rows = [row for row in scenario() if row['topic'] != core.SUMMARY]
        verdict = self.check(rows)
        self.assertFalse(verdict['pass'])
        self.assertIn('tf_repeated_data_absent', verdict['not_evaluable'])

    def test_a_recording_without_samples_cannot_clear_the_lookup_check(self):
        rows = [row for row in scenario() if row['topic'] != core.SAMPLE]
        verdict = self.check(rows)
        self.assertIn('lookup_samples_follow_status', verdict['not_evaluable'])
        self.assertFalse(verdict['pass'])

    def test_localization_that_never_leaves_valid_cannot_clear_the_lookup_check(self):
        rows = [row for row in scenario()
                if not (row['topic'] == '/fpga/localization_status'
                        and row['payload']['state'] != 'valid')]
        verdict = self.check(rows)
        self.assertIn('lookup_samples_follow_status', verdict['not_evaluable'])

    def test_the_windows_are_command_line_parameters(self):
        options = parser().parse_args(['check', 'r', '--out', 'o'])
        self.assertEqual((options.c1_max_age_s, options.c1_min_age_s, options.c1_stale_s,
                          options.c1_stale_age_s, options.c2_drop_s, options.c2_grace_samples),
                         (.3, -.167, 1.2, 1., .15, 6))
        widened = parser().parse_args(['check', 'r', '--out', 'o', '--c2-drop-s', '5'])
        self.assertEqual(widened.c2_drop_s, 5.)

    def test_widening_a_window_past_the_recording_buys_not_evaluable_not_a_pass(self):
        # A threshold loose enough that no sample falls inside the window must not turn a
        # violated criterion into silence: the transition stops being evaluable instead.
        rows = scenario()
        for row in rows:
            if row['topic'] == core.SAMPLE and row['clock_ns'] > CLOCK0 + LAST_VALID * STEP:
                row['c2_at_odom'] = True
        self.assert_fails(self.check(rows), {'lookup_samples_follow_status'},
                          'lookup_samples_follow_status')
        widened = self.check(rows, '--c2-drop-s', '5')
        self.assertIn('lookup_samples_follow_status', widened['not_evaluable'])
        self.assertFalse(widened['pass'])
        self.assertEqual(widened['checks']['lookup_samples_follow_status']['evidence']
                         ['unevaluable_transitions'][0]['skipped'], 'no sample past the C2 window')

    # --- a recording that stops inside a window ---------------------------------------
    def left_valid_clock_ns(self):
        """The only leave-valid transition in the scenario: the first non-valid status."""
        return CLOCK0 + (LAST_VALID + 1) * STEP

    def c1_window_end_ns(self, stale_s=1.2):
        return self.left_valid_clock_ns() + int(stale_s * 1e9)

    def cut_scenario(self, steps, c1_age_s=None):
        """The scenario stopped after `steps`, optionally holding C1 fresh after the loss.

        Nothing else changes: the same transition, the same window, only how much of the
        window the recording actually covers.
        """
        rows = scenario(steps)
        if c1_age_s is not None:
            for row in rows:
                # From the first sample the loss can already be visible in: the held edge
                # reaches FUTURE past the last valid step, so earlier samples are still
                # labelled valid and their age is bound by --c1-max-age-s instead.
                if (row['topic'] == core.SAMPLE
                        and row['clock_ns'] > CLOCK0 + LAST_VALID * STEP + FUTURE):
                    row['c1_age_s'] = c1_age_s
                    row['c1_stamp_ns'] = row['clock_ns'] - int(c1_age_s * 1e9)
        return rows

    def test_the_cut_recording_stops_0_4_s_before_the_c1_window_closes(self):
        # Fixes the geometry the next two tests rest on, so a change to the scenario
        # constants cannot quietly turn them into tests of something else.
        rows = self.cut_scenario(SHORT_STEPS)
        last = [row['clock_ns'] for row in rows if row['topic'] == core.SAMPLE][-1]
        self.assertAlmostEqual((self.c1_window_end_ns() - last) / 1e9, .4, places=3)
        self.assertGreater(last, self.left_valid_clock_ns())

    def test_a_recording_that_ends_inside_the_c1_window_is_unevaluable_not_failed(self):
        # The window never got the chance to fill, so neither "it stayed stale" nor "it
        # came back" was observable. Whatever C1 last read is not a verdict, and the
        # branch must fire either way: fresh (what the loaded rehearsal recording had) and
        # stale (which would have passed the criterion on the partial window).
        for age_s in (None, .7333, 1.1):
            verdict = self.check(self.cut_scenario(SHORT_STEPS, age_s))
            evidence = verdict['checks']['lookup_samples_follow_status']['evidence']
            self.assertEqual([entry['skipped'] for entry in evidence['unevaluable_transitions']],
                             ['record ended inside the C1 window'], age_s)
            entry = evidence['unevaluable_transitions'][0]
            self.assertEqual(entry['left_valid_clock_ns'], self.left_valid_clock_ns())
            self.assertEqual(entry['c1_window_end_clock_ns'], self.c1_window_end_ns())
            self.assertEqual(entry['record_end_clock_ns'], evidence['record_end_clock_ns'])
            self.assertEqual(evidence['transitions'], [])
            self.assertEqual(evidence['valid_violation_count'], 0)
            # Nothing was judged, so the check is not_evaluable — which still fails the run,
            # but on "there was nothing to judge", never on a criterion.
            check = verdict['checks']['lookup_samples_follow_status']
            self.assertTrue(check['not_evaluable'])
            self.assertEqual(check['reason'],
                             'localization never left valid with samples in both windows')
            self.assertEqual(set(verdict['failed']), {'lookup_samples_follow_status'})

    def test_the_same_recording_carried_past_the_window_still_fails_on_a_fresh_c1(self):
        # Positive control for the branch above: extend the identical recording past the
        # window and the fresh lookup is a real violation again, reported as a failure.
        verdict = self.check(self.cut_scenario(STEPS, .7333))
        self.assert_fails(verdict, {'lookup_samples_follow_status'},
                          'lookup_samples_follow_status')
        evidence = verdict['checks']['lookup_samples_follow_status']['evidence']
        self.assertEqual(evidence['unevaluable_transitions'], [])
        self.assertEqual(len(evidence['transitions']), 1)
        self.assertFalse(evidence['transitions'][0]['c1']['pass'])
        self.assertEqual(evidence['transitions'][0]['c1']['last_age_s'], .7333)
        self.assertGreater(evidence['record_end_clock_ns'], self.c1_window_end_ns())


def core_checks_expected():
    from tools.nav2_interface_check import CHECKS
    return [name for name in CHECKS if name != 'approved_frame_count']


class TopicInfoParsingTests(unittest.TestCase):
    def test_publisher_counts_are_read_per_topic(self):
        parsed = parse_topic_info(TOPIC_INFO % 1)
        self.assertEqual(parsed['/tf']['publishers'], 2)
        self.assertEqual(parsed['/fpga/motion_permit']['publishers'], 1)

    def test_an_error_block_has_no_publisher_count(self):
        parsed = parse_topic_info('=== topic: /fpga/motion_permit ===\nerror: OSError: no ros2\n')
        self.assertIsNone(parsed['/fpga/motion_permit']['publishers'])


class RecorderCoreTests(unittest.TestCase):
    def test_the_repeated_counter_only_counts_exact_triples(self):
        counter = core.RepeatedStampCounter()
        self.assertFalse(counter.observe('map', 'odom', 10))
        self.assertFalse(counter.observe('map', 'odom', 11))
        self.assertFalse(counter.observe('odom', 'base_link', 10))
        self.assertEqual(counter.count, 0)
        self.assertTrue(counter.observe('map', 'odom', 10))
        self.assertTrue(counter.observe('map', 'odom', 10))
        self.assertEqual(counter.count, 2)
        self.assertEqual(counter.examples(), [{'parent': 'map', 'child': 'odom', 'stamp_ns': 10,
                                               'extra_copies': 2}])

    def test_the_sample_builder_reports_an_age_against_the_clock(self):
        row = core.sample_record(5, 1_000_000_000, 900_000_000, False, True, 900_000_000)
        self.assertEqual(row['topic'], core.SAMPLE)
        self.assertAlmostEqual(row['c1_age_s'], .1)
        self.assertEqual((row['c1_error'], row['c2_at_clock'], row['c2_at_odom'],
                          row['c2_at_odom_error'], row['odom_base_stamp_ns']),
                         (None, False, True, None, 900_000_000))

    def test_the_sample_builder_records_an_exception_by_name(self):
        row = core.sample_record(5, 1_000_000_000, LookupError('frame does not exist'),
                                 RuntimeError('boom'), None)
        self.assertEqual(row['c1_error'], 'LookupError')
        self.assertIsNone(row['c1_age_s'])
        self.assertIsNone(row['c1_stamp_ns'])
        self.assertEqual(row['c2_at_clock_error'], 'RuntimeError')
        self.assertIsNone(row['c2_at_clock'])
        # no odometry edge yet is recorded as a reason, not as a false answer
        self.assertIsNone(row['c2_at_odom'])
        self.assertEqual(row['c2_at_odom_error'], 'no odom->base_link stamp yet')
        self.assertIsNone(row['odom_base_stamp_ns'])
        json.dumps(row)

    def test_the_newest_edge_stamp_only_follows_its_own_dynamic_edge(self):
        edge = core.NewestEdgeStamp('odom', 'base_link')
        self.assertIsNone(edge.stamp_ns)
        self.assertFalse(edge.observe('map', 'odom', 50))
        self.assertFalse(edge.observe('odom', 'base_link', 50, static=True))
        self.assertIsNone(edge.stamp_ns)
        self.assertTrue(edge.observe('odom', 'base_link', 50))
        self.assertFalse(edge.observe('odom', 'base_link', 40))
        self.assertEqual(edge.stamp_ns, 50)
        self.assertTrue(edge.observe('odom', 'base_link', 60))
        self.assertEqual((edge.stamp_ns, edge.count), (60, 3))

    def test_record_rows_reject_a_negative_stamp(self):
        with self.assertRaises(ValueError):
            core.transform_row('map', 'odom', -1, False)
        with self.assertRaises(ValueError):
            core.clock_record(1, -5)

    def test_an_unparseable_status_payload_is_recorded_not_dropped(self):
        row = core.status_record('/fpga/localization_status', 1, 2, 'not json')
        self.assertIsNone(row['payload'])
        self.assertTrue(row['parse_error'])


class RecorderWiringTests(unittest.TestCase):
    """The recorder cannot run here, so pin by source inspection what it must do."""

    def setUp(self):
        self.source = (CHECKOUT / 'tools' / 'tf_topic_recorder.py').read_text()

    def test_it_feeds_its_own_buffer_instead_of_adding_a_second_subscription(self):
        self.assertIn('buffer.set_transform_static(transform, AUTHORITY)', self.source)
        self.assertIn('buffer.set_transform(transform, AUTHORITY)', self.source)
        self.assertNotIn('TransformListener(', self.source)

    def test_clock_qos_matches_the_replay_publisher(self):
        replay = (CHECKOUT / 'tools' / 'rtabmap_ros_replay.py').read_text()
        self.assertIn('pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,\n'
                      '                         durability=DurabilityPolicy.VOLATILE)', replay)
        self.assertIn("clock_pub = node.create_publisher(Clock, '/clock', pub_qos)", replay)
        self.assertIn('clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,\n'
                      '                           durability=DurabilityPolicy.VOLATILE)',
                      self.source)
        self.assertIn('args.clock_topic, clock, clock_qos', self.source)

    def test_tf_static_is_subscribed_transient_local_depth_100(self):
        self.assertIn('tf_static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,\n'
                      '                               durability=DurabilityPolicy.TRANSIENT_LOCAL)',
                      self.source)

    def test_every_clock_message_appends_a_sample_with_both_can_transform_times(self):
        # `received`, not a second now(): the /clock callback reads its steady clock once at
        # entry and stamps both the clock row and the sample row with that one reading, so a
        # sample never carries the cost of the tf2 lookups taken between them.
        self.assertIn('write(sample_record(received, clock_ns, lookup, at_clock, at_odom, '
                      'odom_edge.stamp_ns,', self.source)
        clock_callback = self.source.split('    def clock(message):')[1].split('\n    node.')[0]
        self.assertEqual(clock_callback.count('now()'), 1, clock_callback)
        self.assertIn('write(clock_record(received, clock_ns))', clock_callback)
        self.assertIn('Time(nanoseconds=when), timeout=Duration(seconds=0)', self.source)
        self.assertIn('at_clock = can_at(clock_ns)', self.source)
        self.assertIn("at_odom = None if odom_edge.stamp_ns is None else can_at(odom_edge.stamp_ns)",
                      self.source)

    def test_the_newest_odometry_stamp_comes_from_the_same_rows_that_are_recorded(self):
        self.assertIn('odom_edge.observe(transform.header.frame_id, transform.child_frame_id, '
                      'stamp, static)', self.source)
        self.assertIn('odom_edge = NewestEdgeStamp(args.odom_frame, args.base_frame)', self.source)

    def test_the_output_is_never_appended_to_and_a_summary_is_always_written(self):
        self.assertIn("log = args.output.open('x', buffering=1)", self.source)
        self.assertIn('write(summary_record(', self.source)

    def test_the_topic_info_snapshot_cannot_fail_the_recording(self):
        self.assertIn("['ros2', 'topic', 'info', '--verbose', '--no-daemon', topic]", self.source)
        self.assertIn('except (OSError, subprocess.SubprocessError) as exc:', self.source)


class LoadedRehearsalRecordTests(unittest.TestCase):
    """The frozen 2026-09-14 rehearsal recording, re-judged with the protocol §9 numbers.

    Read-only: the recording, its logs and its topic-info snapshot are the sealed run
    directory, and the verdict is written to a temporary file. The recording stopped 0.77 s
    before the last transition's C1 window closed, so before the fix its third transition
    was failed on a C1 that had not been given time to go stale. Skipped where the frozen
    run directory is not present, like the other frozen-artifact tests.
    """

    RUN = Path('${FPGA_SRC}/runs/rtabmap-nav2-rehearsal-20260914'
               '/loaded/normal')
    LOGS = ('gate.log', 'localization.log', 'odom_adapter.log', 'odometry.log', 'recorder.log',
            'replay.log', 'static_tf.log', 'static_tf_base_camera.log')

    def verdict(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / 'verdict.json'
            argv = ['check', str(self.RUN / 'tf-record.jsonl'), '--out', str(out),
                    '--c2-grace-samples', '6', '--c1-min-age-s', '-0.167',
                    '--c1-max-age-s', '0.3', '--c2-drop-s', '0.2', '--c1-stale-s', '1.2',
                    '--c1-stale-age-s', '1.0', '--map-odom-mode', 'hold',
                    '--expect-approved', '51']
            for name in self.LOGS:
                argv += ['--log', str(self.RUN / name)]
            with open('/dev/null', 'w') as quiet:
                code = main(argv, stream=quiet)
            return code, json.loads(out.read_text())

    @unittest.skipUnless(RUN.is_file() or (RUN / 'tf-record.jsonl').is_file(),
                         'frozen rehearsal recording not present')
    def test_only_the_truncated_transition_changes_and_the_approved_count_still_fails(self):
        code, verdict = self.verdict()
        evidence = verdict['checks']['lookup_samples_follow_status']['evidence']
        skipped = [entry['skipped'] for entry in evidence['unevaluable_transitions']]
        # Five transitions were already unevaluable for returning to valid; the sixth is the
        # one this fix stops failing. Everything judged still passes on its own merits.
        self.assertEqual(skipped.count('record ended inside the C1 window'), 1)
        self.assertEqual(len(skipped), 6)
        self.assertEqual([t['pass'] for t in evidence['transitions']], [True, True])
        self.assertTrue(verdict['checks']['lookup_samples_follow_status']['pass'])
        self.assertEqual(evidence['valid_violation_count'], 0)
        self.assertEqual(evidence['record_end_clock_ns'], 1305031473196069000)
        last = evidence['unevaluable_transitions'][-1]
        self.assertEqual(last['left_valid_clock_ns'], 1305031472763578000)
        self.assertEqual(last['c1_window_end_clock_ns'], 1305031473963578000)
        # The run still fails, and on the line it always failed on: 47 approved, not 51.
        approved = verdict['checks']['approved_frame_count']
        self.assertFalse(approved['pass'])
        self.assertEqual((approved['evidence']['distinct_approved_stamps'],
                          approved['evidence']['expected']), (47, 51))
        self.assertEqual(set(verdict['failed']),
                         {'approved_frame_count', 'motion_permit_publisher_unique'})
        self.assertEqual(verdict['not_evaluable'], ['motion_permit_publisher_unique'])
        self.assertEqual(code, 1)


class CommandLineTests(unittest.TestCase):
    def test_the_tool_exits_nonzero_on_a_failing_recording(self):
        rows = without(scenario(), lambda row: is_edge(row, 'odom', 'base_link'))
        with tempfile.TemporaryDirectory() as directory:
            record = Path(directory) / 'tf-record.jsonl'
            record.write_text(''.join(json.dumps(row) + '\n' for row in rows))
            Path(str(record) + '.topic-info.txt').write_text(TOPIC_INFO % 1)
            out = Path(directory) / 'verdict.json'
            done = subprocess.run([sys.executable, str(CHECKOUT / 'tools' / 'nav2_interface_check.py'),
                                   'check', str(record), '--out', str(out)],
                                  capture_output=True, text=True, cwd=str(CHECKOUT))
            self.assertEqual(done.returncode, 1, done.stderr)
            self.assertIn('required_edges_present', done.stdout)
            self.assertFalse(json.loads(out.read_text())['pass'])


if __name__ == '__main__':
    unittest.main()
