"""Synthetic streams for the per-frame content comparison.

Everything the round's verdict rests on is here: that identical streams compare identical,
that a single changed number is found at the right frame and attributed to the right field,
that a changed INPUT is separated from changed CONTENT rather than folded into it, and that
the verdict table is the protocol's, not an interpretation of it.
"""
import json
from pathlib import Path
import tempfile
import unittest

from tools import rtabmap_content_compare as compare


def frame(index, stamp, features=100, inliers=50, loop_id=0, rgb='aa', depth='bb'):
    """The four rows one replayed frame produces, in the order the probe writes them."""
    return [
        {'event': 'publish_begin', 'index': index, 'stamp_ns': stamp,
         'published_rgb_sha256': rgb, 'published_depth_sha256': depth,
         'valid_depth_pixels': 1000},
        {'event': 'odom_info', 'stamp_ns': stamp, 'features': features, 'matches': 80,
         'inliers': inliers, 'lost': False, 'transform': {'x': 1.0},
         'covariance': [1.0, 2.0], 'time_estimation': 0.01},
        {'event': 'info', 'stamp_ns': stamp, 'loop_closure_id': loop_id,
         'proximity_detection_id': 0, 'loop_closure_transform': {'x': 0.0}},
        {'event': 'localization_pose', 'stamp_ns': stamp, 'pose': {'position': {'x': 0.0}}},
    ]


class StreamFixture(unittest.TestCase):
    def setUp(self):
        self.directory = Path(tempfile.mkdtemp())

    def write(self, name, rows):
        path = self.directory / name
        path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
        return compare.load_stream(path)

    def stream(self, name, changes=None):
        rows = []
        for index in range(5):
            rows += frame(index, 1000 + index, **(changes or {}).get(index, {}))
        return self.write(name, rows)


class LoadTests(StreamFixture):
    def test_every_frame_is_indexed_by_its_stamp_with_all_fields_present(self):
        stream = self.stream('a.jsonl')
        self.assertEqual(stream['order'], [1000, 1001, 1002, 1003, 1004])
        self.assertEqual(stream['event_counts']['odom_info'], 5)
        self.assertNotIn(compare.MISSING, stream['content'][1000].values())
        self.assertEqual(set(stream['content'][1000]), set(compare.CONTENT_KEYS))

    def test_a_missing_message_becomes_an_explicit_absent_marker(self):
        rows = [row for row in frame(0, 1000) if row['event'] != 'info']
        stream = self.write('partial.jsonl', rows)
        self.assertEqual(stream['content'][1000]['info.loop_closure_id'], compare.MISSING)
        self.assertEqual(stream['content'][1000]['odom_info.features'], 100)

    def test_blank_lines_and_rows_without_a_stamp_are_tolerated(self):
        path = self.directory / 'noisy.jsonl'
        path.write_text('\n'.join([json.dumps({'event': 'start'}), ''] +
                                  [json.dumps(row) for row in frame(0, 1000)] + ['', '']))
        stream = compare.load_stream(path)
        self.assertEqual(stream['order'], [1000])
        self.assertEqual(stream['event_counts']['start'], 1)

    def test_a_repeated_publish_is_recorded_and_not_counted_twice(self):
        stream = self.write('dup.jsonl', frame(0, 1000) + frame(1, 1000))
        self.assertEqual(stream['order'], [1000])
        self.assertEqual(stream['duplicate_stamps'], [1000])


class IdenticalStreamTests(StreamFixture):
    def test_two_identical_streams_are_identical_everywhere(self):
        result = compare.compare_streams(self.stream('a.jsonl'), self.stream('b.jsonl'))
        self.assertTrue(result['input_identical'])
        self.assertTrue(result['content_identical'])
        self.assertIsNone(result['content']['first_differing_index'])
        self.assertEqual(result['content']['differing_frames'], 0)
        self.assertEqual(set(result['content']['per_field_differing_frames'].values()), {0})
        self.assertEqual(result['content']['frames_compared'], 5)

    def test_a_field_outside_the_protocol_list_is_not_compared(self):
        """`time_estimation` is timing, not content; it must not make a pair differ."""
        rows = []
        for index in range(5):
            block = frame(index, 1000 + index)
            block[1]['time_estimation'] = 9.9
            rows += block
        result = compare.compare_streams(self.stream('a.jsonl'), self.write('slow.jsonl', rows))
        self.assertTrue(result['content_identical'])


class DifferingStreamTests(StreamFixture):
    def test_one_changed_counter_is_located_and_attributed(self):
        result = compare.compare_streams(self.stream('a.jsonl'),
                                         self.stream('b.jsonl', {2: {'inliers': 51}}))
        self.assertTrue(result['input_identical'])
        self.assertFalse(result['content_identical'])
        self.assertEqual(result['content']['first_differing_index'], 2)
        self.assertEqual(result['content']['first_differing_stamp_ns'], 1002)
        self.assertEqual(result['content']['differing_frames'], 1)
        self.assertEqual(result['content']['differing_indices'], [2])
        per_field = result['content']['per_field_differing_frames']
        self.assertEqual(per_field['odom_info.inliers'], 1)
        self.assertEqual(per_field['odom_info.features'], 0)

    def test_several_fields_and_frames_are_counted_separately(self):
        right = self.stream('b.jsonl', {1: {'features': 101}, 3: {'loop_id': 7},
                                        4: {'features': 99, 'loop_id': 9}})
        result = compare.compare_streams(self.stream('a.jsonl'), right)
        self.assertEqual(result['content']['differing_frames'], 3)
        self.assertEqual(result['content']['differing_indices'], [1, 3, 4])
        per_field = result['content']['per_field_differing_frames']
        self.assertEqual(per_field['odom_info.features'], 2)
        self.assertEqual(per_field['info.loop_closure_id'], 2)

    def test_a_missing_message_counts_as_a_difference(self):
        rows = []
        for index in range(5):
            block = frame(index, 1000 + index)
            rows += [row for row in block if not (index == 2 and row['event'] == 'info')]
        result = compare.compare_streams(self.stream('a.jsonl'), self.write('gap.jsonl', rows))
        self.assertEqual(result['content']['differing_indices'], [2])
        self.assertEqual(
            result['content']['per_field_differing_frames']['info.loop_closure_id'], 1)


class InputPrerequisiteTests(StreamFixture):
    def test_a_changed_image_hash_breaks_the_prerequisite_not_the_content_read(self):
        result = compare.compare_streams(self.stream('a.jsonl'),
                                         self.stream('b.jsonl', {0: {'rgb': 'zz'}}))
        self.assertFalse(result['input_identical'])
        self.assertEqual(result['input']['first_differing_index'], 0)
        self.assertEqual(result['input']['per_field_differing_frames']
                         ['published_rgb_sha256'], 1)
        self.assertTrue(result['content_identical'])

    def test_a_missing_frame_shows_up_on_both_sides_of_the_ledger(self):
        rows = []
        for index in range(4):
            rows += frame(index, 1000 + index)
        result = compare.compare_streams(self.stream('a.jsonl'), self.write('short.jsonl', rows))
        self.assertFalse(result['stamp_order_equal'])
        self.assertEqual(result['only_in_left'], [1004])
        self.assertEqual(result['only_in_right'], [])
        self.assertFalse(result['input_identical'])
        self.assertFalse(result['content_identical'])
        self.assertEqual(result['content']['frames_compared'], 4)


class PercentileTests(unittest.TestCase):
    def test_nearest_rank_on_a_known_list(self):
        result = compare.percentiles(list(range(1, 101)))
        self.assertEqual((result['min'], result['max'], result['count']), (1., 100., 100))
        self.assertEqual((result['p50'], result['p90'], result['p99']), (50., 90., 99.))

    def test_a_single_sample_and_an_empty_list(self):
        self.assertEqual(compare.percentiles([2.5])['p99'], 2.5)
        self.assertEqual(compare.percentiles([]), {'count': 0})


class VerdictTests(unittest.TestCase):
    def test_the_protocol_table_is_reproduced(self):
        table = [((True, None, True), 'H-THREAD', 'H-thread', '成立'),
                 ((True, True, True), 'H-THREAD', 'H-thread', '成立'),
                 ((False, True, True), 'H-FLANN', 'H-flann', '成立'),
                 ((False, False, True), 'RESIDUAL', 'H-ransac', '未判'),
                 ((False, None, True), 'UNJUDGED', 'H-thread', '排除'),
                 ((None, None, True), 'UNJUDGED', 'H-thread', '未判'),
                 ((True, True, False), 'INSTRUMENT_ERROR', 'H-thread', '未判')]
        for inputs, code, candidate, state in table:
            result, candidates, reason = compare.verdict(*inputs)
            self.assertEqual(result, code, msg=inputs)
            self.assertEqual(candidates[candidate], state, msg=inputs)
            self.assertTrue(reason)

    def test_a_failed_prerequisite_beats_every_other_input(self):
        for a in (True, False, None):
            for c in (True, False, None):
                code, candidates, _ = compare.verdict(a, c, False)
                self.assertEqual(code, 'INSTRUMENT_ERROR')
                self.assertEqual(set(candidates.values()), {'未判'})

    def test_h_flann_is_never_claimed_without_the_c_pair(self):
        for a in (True, False, None):
            code, candidates, _ = compare.verdict(a, None, True)
            self.assertEqual(candidates['H-flann'], '未判')
            self.assertNotEqual(code, 'H-FLANN')


if __name__ == '__main__':
    unittest.main()
