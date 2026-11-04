"""Calibrate association against distinct misleading evidence patterns."""
import json
from pathlib import Path
import tempfile
import unittest

from tools.rtabmap_consumer_evidence_audit import audit


class ConsumerAuditTests(unittest.TestCase):
    def fixture(self, root, processed=False, gate=True, duplicate=False, observed=None, result=None):
        def write(name, data, lines=False):
            with (root/name).open('x') as f:
                f.write('\n'.join(json.dumps(x) for x in data) if lines else json.dumps(data))
        write('result.json', result if result is not None else {'checks': [{'check': 'discovery', 'pass': True},
            {'check': 'normal inference', 'pass': processed}, {'check': 'duration', 'seconds': .2}]})
        write('observed-messages.json', observed if observed is not None else {'statuses': [[2., {'state': 'valid', 'sequence': 1,
            'stamp_ns': 100, 'clock_ns': 100, 'approved': [{'stamp_ns': 100}]}]], 'poses': [], 'tf': []})
        write('runtime-events.jsonl', [{'event': 'frame_processed', 'frame_id': 'rgb-100'}] if processed else [], True)
        with (root/'runtime.log').open('x') as f:
            f.write('Dropped acquisition after bounded wait for clock/TF\n')
        if gate:
            rows = [{'event': 'received_localization_pose', 'stamp_ns': 100,
                     'monotonic_ns': 1000, 'approved': [{'stamp_ns': 100}]},
                    {'event': 'callback_output_complete', 'kind': 'localization_pose',
                     'stamp_ns': 100, 'monotonic_ns': 1100}]
            write('gate-events.jsonl', rows*2 if duplicate else rows, True)

    def test_approved_but_not_processed_does_not_infer_runtime_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); self.fixture(root)
            row = audit(root); frame = row['frames'][0]
            self.assertTrue(frame['driver_valid_status_receipts'])
            self.assertEqual(frame['runtime_processed'], [])
            self.assertEqual(frame['runtime_receipt_and_wait_state'], 'unknown')
            self.assertEqual(frame['gate_output_brackets'][0]['duration_ns'], 100)
            self.assertIn('not pure logging cost', frame['gate_output_brackets'][0]['meaning'])
            self.assertIsNone(row['warnings_unassociated'][0]['stamp_ns'])
            self.assertEqual(row['warnings_unassociated'][0]['cause'], 'unknown')
            self.assertEqual(row['checks']['pass_count'], 1)
            self.assertEqual(row['checks']['fail_count'], 1)
            self.assertEqual(len(row['checks']['measurement_items']), 1)

    def test_warning_can_coexist_with_success_and_missing_gate_is_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); self.fixture(root, processed=True, gate=False)
            row = audit(root); frame = row['frames'][0]
            self.assertEqual(len(frame['runtime_processed']), 1)
            self.assertEqual(frame['gate_output_brackets'], [])
            self.assertEqual(frame['gate_output_bracket_state'], 'unknown')
            self.assertEqual(row['sources']['gate-events.jsonl']['state'], 'unknown')
            self.assertEqual(row['checks']['pass_count'], 2)
            self.assertTrue(row['warnings_unassociated'])

    def test_duplicate_callback_stamp_rejects_ambiguous_pairing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); self.fixture(root, duplicate=True)
            bracket = audit(root)['frames'][0]['gate_output_brackets'][0]
            self.assertEqual(bracket['state'], 'ambiguous')
            self.assertIsNone(bracket['duration_ns'])
            self.assertEqual(bracket['received_source_lines'], [1, 3])

    def test_missing_fields_differ_from_recorded_empty_arrays(self):
        for recorded in (False, True):
            with self.subTest(recorded=recorded), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                self.fixture(root, observed={'statuses': [], 'poses': [], 'tf': []} if recorded else {},
                             result={'checks': []} if recorded else {})
                row = audit(root)
                self.assertEqual(set(row['field_states'].values()), {'recorded' if recorded else 'unknown'})
                self.assertEqual(row['checks']['pass_count'], 0 if recorded else None)
                self.assertEqual(row['frames'][0]['driver_field_states']['tf'], 'recorded' if recorded else 'unknown')

    def test_invalid_field_types_rejected(self):
        for observed, result in [({'tf': None}, {}), ({}, {'checks': {}}),
                                  ({}, {'checks': [{'pass': 'true'}]})]:
            with self.subTest(observed=observed, result=result), tempfile.TemporaryDirectory() as directory:
                root = Path(directory); self.fixture(root, observed=observed, result=result)
                with self.assertRaises(ValueError):
                    audit(root)

    def test_unrelated_tf_edge_is_preserved_without_readiness_claim(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root, observed={'tf': [[2., {'transforms': [{'header': {
                'stamp': {'sec': 0, 'nanosec': 100}, 'frame_id': 'unrelated_map'},
                'child_frame_id': 'unrelated_sensor'}]}]]})
            receipt = audit(root)['frames'][0]['driver_tf_receipts'][0]
            self.assertEqual(receipt['edges'], [{'parent': 'unrelated_map', 'child': 'unrelated_sensor'}])
            self.assertTrue(receipt['transform_path_validity'].startswith('unknown'))


if __name__ == '__main__':
    unittest.main()
