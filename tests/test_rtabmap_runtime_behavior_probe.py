"""Source-loaded callback calibration and fixed causal matrix checks."""
import json
import os
from pathlib import Path
import unittest



class RuntimeBehaviorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = os.environ.get('RTABMAP_RUNTIME_EVIDENCE')
        if not path:
            raise unittest.SkipTest('set RTABMAP_RUNTIME_EVIDENCE to one existing formal run; tests never rerun matrix')
        cls.root = Path(path)
        cls.results = json.loads((cls.root/'summary.json').read_text())
        cls.calibration_root = Path(cls.results['calibration_evidence'])
        cls.matrix = {r['name']: r for r in cls.results['matrix']}
        cls.calibration = {r['name']: r for r in cls.results['calibration']}

    def events(self, group, name):
        root = self.calibration_root if group == 'calibration' else self.root
        return json.loads((root/group/name/'events.json').read_text())

    def test_all_registered_cells_and_calibrations_present(self):
        self.assertEqual(len(self.matrix), 16)
        self.assertEqual(len(self.calibration), 14)
        self.assertEqual(len(self.results['controls']), 6)

    def test_real_memory_positive_and_exact_validation_negative(self):
        for policy in ('original', 'candidate'):
            good = self.calibration[policy+'__normal_result']
            bad = self.calibration[policy+'__depth_timestamp_invalid']
            self.assertIn('rgb-1400000000', good['memory_seen'])
            self.assertEqual(good['memory_last_timestamp'], 1.4)
            self.assertEqual(bad['submitted'], ['rgb-1300000000'])
            self.assertEqual(bad['memory_seen'], ['rgb-1300000000'])
            raw = json.loads((self.calibration_root/'calibration'/bad['name']/'runtime-diagnostics.json').read_text())
            error = next(e for e in raw['events'] if e['event'] == 'prepare_error')
            self.assertIn('RGB/depth timestamp mismatch', error['detail'])
            self.assertEqual(error['branch'], 'matrix')

    def test_old_future_rejected_after_epoch_or_instance_change(self):
        for policy in ('original', 'candidate'):
            for fault in ('late_future_epoch_revoke', 'late_future_instance_revoke'):
                row = self.calibration[policy+'__'+fault]
                self.assertIn('rgb-1400000000', row['submitted'])
                self.assertNotIn('rgb-1400000000', row['memory_seen'])
                self.assertEqual(row['memory_last_timestamp'], 1.3)

    def test_navigation_calibration_was_armed_before_fault_outputs(self):
        for policy in ('original', 'candidate'):
            row = self.calibration[policy+'__navigation_fault']
            events = self.events('calibration', row['name'])
            self.assertTrue(any(e['event'] == 'publish' and e['topic'] == '/fpga/motion_permit'
                                and e['payload'] is True and e['sequence'] < row['episode_start_sequence'] for e in events))
            for key in ('first_permit_false', 'first_zero_command', 'first_cancel_request'):
                self.assertIsNotNone(row[key])
                self.assertLess(row[key]['sequence'], row['episode_end_sequence'])
            self.assertTrue(row['final_motion_inhibited'])

    def test_complete_delivery_first_stop_precedes_old_frame_expiry(self):
        for evolution in ('same_instance', 'restart'):
            for policy in ('original', 'candidate'):
                row = self.matrix['__'.join((policy, evolution, 'nav', 'complete'))]
                self.assertEqual(row['first_zero_command']['wall'], .35)
                self.assertEqual(row['first_cancel_request']['wall'], .35)
                self.assertFalse(any(e['value'] for e in row['permit_timeline']))

    def test_restart_real_consumer_can_process_once_delivered_old_rgb(self):
        original = self.matrix['original__restart__idle__complete']
        candidate = self.matrix['candidate__restart__idle__complete']
        self.assertIn('rgb-1400000000', original['memory_seen'])
        self.assertNotIn('rgb-1400000000', candidate['memory_seen'])
        for row in (original, candidate):
            events = self.events('matrix', row['name'])
            target_images = [e for e in events if e['event'] == 'rgb_delivery' and e['stamp_ns'] == 1400000000]
            self.assertEqual(len(target_images), 1)

    def test_omitted_status_is_recorded_fault_and_stop_difference_visible(self):
        original = self.matrix['original__same_instance__nav__omit_intermediate']
        candidate = self.matrix['candidate__same_instance__nav__omit_intermediate']
        self.assertEqual(original['first_cancel_request']['wall'], .901)
        self.assertIsNone(candidate['first_cancel_request'])
        self.assertEqual(original['first_permit_false']['wall'], candidate['first_permit_false']['wall'])
        for row, expected_recovery in ((original, False), (candidate, True)):
            events = self.events('matrix', row['name'])
            before_result = next(e['sequence'] for e in events if e['event'] == 'action_result_delivery')
            recovered = any(e['value'] and row['first_permit_false']['sequence'] < e['sequence'] < before_result
                            for e in row['permit_timeline'])
            self.assertEqual(recovered, expected_recovery)
        for row in (original, candidate):
            events = self.events('matrix', row['name'])
            self.assertTrue(any(e['event'] == 'status_schedule' and not e['delivered'] and e['omission_fault_injection'] for e in events))

    def test_inflight_preserved_and_original_deadline_not_extended(self):
        for row in self.results['controls']:
            if 'original_deadline' in row['name']:
                self.assertEqual(row['submitted'], ['rgb-1300000000'])
                raw = json.loads((self.root/'controls'/row['name']/'runtime-diagnostics.json').read_text())
                expiry = [e for e in raw['events'] if e['event'] == 'prepare_expiry' and e['expired']]
                self.assertEqual([e['queue']['current']['received_at'] for e in expiry], [.4, .7])
            else:
                self.assertIn('rgb-1400000000', row['memory_seen'])

    def test_full_class_loader_and_single_branch_ast_receipts(self):
        for row in self.results['calibration']:
            loader = self.events('calibration', row['name'])[0]
            self.assertEqual(loader['event'], 'loader')
            self.assertIn('__init__', loader['preserved_methods'])
            for method in ('on_images', 'on_localization', 'submit', 'poll', 'stop', 'fault', 'ready', 'navigate', 'accepted'):
                self.assertIn(method, loader['preserved_methods'])
            if row['policy'] == 'original':
                self.assertEqual(loader['source_class_ast_sha256'], loader['loaded_class_ast_sha256'])
                self.assertEqual(loader['candidate_branch_insertions'], 0)
            else:
                self.assertNotEqual(loader['source_class_ast_sha256'], loader['loaded_class_ast_sha256'])
                self.assertEqual(loader['candidate_branch_insertions'], 1)

    def test_static_tf_nonidentity_and_inverse_and_missing_edge_controls(self):
        for policy in ('original', 'candidate'):
            events = self.events('calibration', policy+'__static_transform_calibration')
            self.assertEqual(sum(e['event'] == 'tf_rejection_calibration' for e in events), 2)
            self.assertTrue(any(e['event'] == 'tf_calibration' for e in events))
            missing = self.calibration[policy+'__missing_static_tf']
            self.assertEqual(missing['submitted'], ['rgb-1300000000'])


if __name__ == '__main__':
    unittest.main()
