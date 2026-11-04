"""Fail-closed offline checks for Nav C dynamic evidence."""
import unittest
import json
import tempfile
from pathlib import Path

from tools.verify_nav_c_dynamic import evaluate, verify_run, costmap_region_samples, free_rays, terminal_motion_check


def clock_event(name, at, **fields):
    return {'event': name, 'wall_monotonic': at, 'sim_clock_s': at,
            'clock_received_monotonic': at - .05, 'clock_age_s': .05, **fields}


def worker(status, passed=True, error=None):
    result = {'event': 'case_result', 'passed': passed, 'status': status,
              'stopped': True, 'name': 'c_navigation'}
    summary = {'event': 'summary', 'passed': passed, 'results': [result]}
    if error:
        summary['error'] = error
    return [result, summary]


def point(at, free=False):
    return {'topic': '/fpga/points', 'received_monotonic': at, 'sim_clock_s': at,
            'message_received_monotonic': at, 'clock_received_monotonic': at - .05,
            'header': {'frame_id': 'camera', 'stamp': {'sec': int(at), 'nanosec': 0}},
            'stamp_s': at, 'file': 'points.cdr',
            'finite_xyz': [[0., 3., .75], [.05, 3., .75], [-.05, 3., .75]] if free else [],
            'transform_map_from_cloud': {
                'status': 'ok',
                'message': {'header': {'stamp': {'sec': int(at), 'nanosec': 0}}, **transform()}}}


def transform():
    return {'transform': {'translation': {'x': 0., 'y': 0., 'z': 0.},
                          'rotation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}


def plan(at=1):
    return {'topic': '/plan', 'received_monotonic': at, 'poses': [[0., 0.], [0., 2.2]]}


def odom(at, y, x=0.):
    return {'topic': '/odom', 'wall_monotonic': at,
            'message': {'pose': {'pose': {'position': {'x': x, 'y': y, 'z': 0.},
                                         'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}}}


def grid(at, occupied):
    width = height = 40
    data = [0] * (width * height)
    if occupied:
        # The obstacle center (0, 1.1) is cell (20,31) with this grid.
        for y in range(29, 34):
            for x in range(16, 25):
                data[y * width + x] = 100
    return {'topic': '/local_costmap/costmap', 'kind': 'full',
            'received_monotonic': at, 'sim_clock_s': at,
            'message_received_monotonic': at, 'clock_received_monotonic': at - .05,
            'header': {'frame_id': 'map', 'stamp': {'sec': int(at), 'nanosec': 0}},
            'transform_map_to_frame': {
                'status': 'identity',
                'message': {'header': {'stamp': {'sec': int(at), 'nanosec': 0}}, **transform()}},
            'message': {'info': {'width': width, 'height': height, 'resolution': .1,
                                 'origin': {'position': {'x': -2., 'y': -2., 'z': 0.},
                                            'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}},
                        'data': data}}


def create_response():
    return {'event': 'response', 'returncode': 0, 'stdout': 'data: true\n', 'stderr': ''}


class DynamicEvidenceTests(unittest.TestCase):
    def test_baseline_passes_with_same_observer_load_and_successful_motion(self):
        injection = [clock_event('baseline_started', 10), {'event': 'injection_complete'}]
        observations = [plan(), point(11)]
        motion = [odom(10, .2), odom(12, .5)]
        report = evaluate('baseline', injection, observations, worker(4), motion)
        self.assertEqual(report['verdict'], 'passed')

    def test_obstacle_and_wall_require_distinct_action_terminals(self):
        event = clock_event('obstacle_inserted', 10, center_map=[0., 1.1, .75],
                            size_m=[.65, .25, 1.5])
        injection = [create_response(), event, {'event': 'injection_complete'}]
        observations = [plan(), point(11), grid(11, True)]
        # AGX and simulator-host monotonic epochs are deliberately incomparable;
        # ordering across hosts comes from the associated simulation clock.
        observations[1]['received_monotonic'] = 1
        observations[2]['received_monotonic'] = 1.1
        obstacle = evaluate('obstacle', injection, observations, worker(4),
                            [odom(10, .2), odom(11, 1.1, .5), odom(12, 2.)])
        wall_event = dict(event, size_m=[5.8, .25, 1.5])
        wall = evaluate('wall', [create_response(), wall_event, {'event': 'injection_complete'}], observations, worker(6),
                        [odom(10, .2), odom(12, .2)])
        self.assertEqual(obstacle['verdict'], 'passed')
        self.assertEqual(wall['verdict'], 'passed')
        wrong = evaluate('wall', [create_response(), wall_event, {'event': 'injection_complete'}], observations, worker(4),
                         [odom(10, .2), odom(12, .5)])
        self.assertEqual(wrong['verdict'], 'failed')
        no_detour = evaluate('obstacle', injection, observations, worker(4),
                             [odom(10, .2), odom(11, 1.1), odom(12, 2.)])
        self.assertEqual(no_detour['verdict'], 'failed')
        self.assertIn('actual_motion_does_not_show_detour', no_detour['failures'])

    def test_removal_requires_fresh_empty_observation_clear_and_motion(self):
        inserted = clock_event('obstacle_inserted', 10, center_map=[0., 1.1, .75],
                               size_m=[.65, .25, 1.5])
        removed = clock_event('obstacle_removed', 15)
        injection = [create_response(), inserted, create_response(), removed,
                     {'event': 'injection_complete'}]
        observations = [plan(), point(11), grid(12, True), point(16, free=True), grid(17, False)]
        report = evaluate('removal', injection, observations, worker(4),
                          [odom(10, .2), odom(15, .7), odom(16, 1.1), odom(17, 1.5)])
        self.assertEqual(report['verdict'], 'passed')
        uncleared = evaluate('removal', injection, observations[:-1] + [grid(17, True)],
                             worker(4), [odom(10, .2), odom(15, .7), odom(16, 1.1), odom(17, 1.5)])
        self.assertEqual(uncleared['verdict'], 'failed')
        self.assertIn('obstacle_region_not_cleared_after_removal', uncleared['failures'])
        no_free_rays = evaluate('removal', injection,
                                [plan(), point(11), grid(12, True), point(16), grid(17, False)],
                                worker(4), [odom(10, .2), odom(15, .7), odom(16, 1.1), odom(17, 1.5)])
        self.assertEqual(no_free_rays['verdict'], 'insufficient_evidence')
        self.assertIn('post_removal_free_ray_evidence', no_free_rays['missing_evidence'])

    def removal_fixture(self):
        inserted = clock_event('obstacle_inserted', 10, center_map=[0., 1.1, .75],
                               size_m=[.65, .25, 1.5])
        injection = [create_response(), inserted, create_response(),
                     clock_event('obstacle_removed', 15), {'event': 'injection_complete'}]
        observations = [plan(), point(11), grid(12, True), point(16, True), grid(17, False)]
        return injection, observations, [odom(10, .2), odom(15, .7), odom(16, 1.1), odom(17, 1.5)]

    def test_invalid_update_cannot_poison_later_valid_samples(self):
        initial = grid(11, False)
        bad = grid(12, True)
        bad.update(kind='update', topic='/local_costmap/costmap_updates')
        bad['message'].update(x=0, y=0, width=40, height=40)
        bad['transform_map_to_frame']['status'] = 'missing'
        later = grid(13, False)
        later.update(kind='update', topic='/local_costmap/costmap_updates')
        later['message'] = {'x': 0, 'y': 0, 'width': 1, 'height': 1, 'data': [0]}
        samples = costmap_region_samples([initial, bad, later], [0., 1.1, .75], [.65, .25, 1.5])
        self.assertFalse(any(s['occupied_cells'] for s in samples))
        self.assertEqual(len(samples), 1, 'a rejected delta invalidates the stream until a full grid')

    def test_delayed_pre_removal_cloud_and_grid_cannot_prove_clearance(self):
        injection, observations, motion = self.removal_fixture()
        for row in observations[-2:]:
            row['header']['stamp'] = {'sec': 14, 'nanosec': 800000000}
            row['sim_clock_s'] = 15.2
            row['stamp_s'] = 14.8
            key = 'transform_map_from_cloud' if row['topic'] == '/fpga/points' else 'transform_map_to_frame'
            row[key]['message']['header']['stamp'] = row['header']['stamp']
        report = evaluate('removal', injection, observations, worker(4), motion)
        self.assertFalse(report['passed'])
        self.assertIn('post_removal_pointcloud_with_tf_and_time', report['missing_evidence'])

    def test_removal_requires_crossing_old_footprint_after_deletion(self):
        injection, observations, _ = self.removal_fixture()
        for motion in ([odom(15, .7), odom(17, .8)],
                       [odom(15, .7, 1.), odom(17, 1.5, 1.)]):
            report = evaluate('removal', injection, observations, worker(4), motion)
            self.assertFalse(report['passed'])
            self.assertIn('no_passage_through_removed_obstacle', report['failures'])

    def test_duplicate_rays_and_duplicate_frames_do_not_satisfy_three_rays(self):
        injection, observations, motion = self.removal_fixture()
        cloud = observations[-2]
        cloud['finite_xyz'] = [[0., 3., .75]] * 3
        self.assertEqual(free_rays(cloud, [0., 1.1, .75], [.65, .25, 1.5]), 1)
        observations.extend([cloud, cloud])
        report = evaluate('removal', injection, observations, worker(4), motion)
        self.assertFalse(report['passed'])
        self.assertIn('post_removal_free_ray_evidence', report['missing_evidence'])

    def terminal_fixture(self):
        import math
        rows = []
        for i in range(6):
            row = odom(20.1 + i * .1, 2.2)
            row['message']['header'] = {'stamp': {'sec': 20, 'nanosec': 100000000 + i * 100000000}}
            row['message']['pose']['pose']['orientation'].update(z=math.sin(math.pi/4), w=math.cos(math.pi/4))
            row['message']['twist'] = {'twist': {'linear': dict(x=0., y=0., z=0.),
                                                  'angular': dict(x=0., y=0., z=0.)}}
            rows.append(row)
        events = worker(4)
        events[0]['sim_s'] = 20.
        return events, rows

    def test_server_terminal_rejects_false_stopped_and_wrong_goal(self):
        events, rows = self.terminal_fixture()
        self.assertTrue(terminal_motion_check('baseline', events, rows)['passed'])
        rows[-1]['message']['twist']['twist']['linear']['x'] = .02
        self.assertFalse(terminal_motion_check('baseline', events, rows)['passed'])
        rows[-1]['message']['twist']['twist']['linear']['x'] = 0.
        for row in rows:
            row['message']['pose']['pose']['position']['y'] = .8
        self.assertFalse(terminal_motion_check('baseline', events, rows)['passed'])
        self.assertTrue(terminal_motion_check('wall', events, rows)['passed'])

    def test_terminal_window_uses_post_result_samples_and_rejects_late_stop(self):
        import copy
        events, rows = self.terminal_fixture()
        earlier = copy.deepcopy(rows[0])
        earlier['message']['header']['stamp'] = {'sec': 19, 'nanosec': 900000000}
        earlier['message']['twist']['twist']['angular']['z'] = .1
        self.assertTrue(terminal_motion_check('baseline', events, [earlier] + rows)['passed'])
        rows[0]['message']['twist']['twist']['angular']['z'] = .1
        self.assertFalse(terminal_motion_check('baseline', events, rows)['passed'])

    def test_server_terminal_rejects_missing_stale_and_duplicate_odom(self):
        events, rows = self.terminal_fixture()
        self.assertFalse(terminal_motion_check('wall', events, rows[:2])['passed'])
        self.assertFalse(terminal_motion_check('wall', events, [rows[-1]] * 5)['passed'])
        events[0]['sim_s'] = 25.
        self.assertFalse(terminal_motion_check('wall', events, rows)['passed'])

    def test_missing_tf_or_logs_is_evidence_insufficient(self):
        injection = [clock_event('baseline_started', 10)]
        bad_point = point(11)
        bad_point['transform_map_from_cloud'] = {'status': 'missing'}
        report = evaluate('baseline', injection, [plan(), bad_point], worker(4),
                          [odom(10, .2), odom(12, .5)])
        self.assertEqual(report['verdict'], 'insufficient_evidence')
        self.assertIn('post_event_pointcloud_with_tf_and_time', report['missing_evidence'])

    def test_timeout_is_explicit_failure_and_never_wall_aborted(self):
        event = clock_event('obstacle_inserted', 10, center_map=[0., 1.1, .75],
                            size_m=[5.8, .25, 1.5])
        report = evaluate('wall', [create_response(), event, {'event': 'injection_complete'}], [plan(), point(11), grid(11, True)],
                          worker(6, passed=False, error='goal_wall_timeout'),
                          [odom(10, .2), odom(12, .2)])
        self.assertEqual(report['verdict'], 'failed')
        self.assertIn('goal_wall_timeout_is_not_a_verdict', report['failures'])

    def test_stale_injection_clock_fails_closed_as_insufficient(self):
        event = clock_event('baseline_started', 10)
        event['clock_age_s'] = 2
        report = evaluate('baseline', [event, {'event': 'injection_complete'}], [plan(), point(11)], worker(4),
                          [odom(10, .2), odom(12, .5)])
        self.assertEqual(report['verdict'], 'insufficient_evidence')
        self.assertIn('baseline_started:fresh_clock_association', report['missing_evidence'])

    def test_missing_run_files_are_named_and_fail_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            run = Path(temp)
            (run / 'run-config.json').write_text(json.dumps({'nav_c': 'baseline'}))
            report = verify_run(run)
        self.assertEqual(report['verdict'], 'insufficient_evidence')
        self.assertIn('file:observations', report['missing_evidence'])


if __name__ == '__main__':
    unittest.main()
