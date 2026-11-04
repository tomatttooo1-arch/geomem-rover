"""Discriminating offline controls, not deployment acceptance tests."""
import copy
import unittest
from tools.rtabmap_frame_disposition_probe import run_controls, disposition
from host.localization_gate import LocalizationLease


class FrameDispositionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = {r['name']: r for r in run_controls()}

    def test_confirming_old_frames_never_borrow_successor_approval(self):
        r = self.rows['confirmation_old_blocks_approved_successor']
        self.assertEqual(r['outcomes']['wait']['eligible'], [1400000000])
        for p in ('conservative', 'discard_all_empty'):
            self.assertEqual(r['outcomes'][p]['eligible'], [1300000000, 1400000000])
        replacement = next(e for e in r['events'] if e['event'] == 'acquire' and e['stamp_ns'] == 1400000000)
        self.assertEqual(replacement['queues']['wait']['previous_latest'], 1300000000)
        self.assertEqual(replacement['queues']['wait']['push_dropped'], 1)
        polls = [e for e in r['events'] if e['event'] == 'poll']
        self.assertTrue(any(e.get('stamp_ns') == 1100000000 and e.get('latest_stamp_ns') == 1300000000 and e['action'] == 'wait' for e in polls))
        self.assertTrue(any(e.get('disposition') == 'completed_confirmation_no_reapproval_same_instance' for e in polls))

    def test_two_causes_same_consumer_prefix_both_recover_exact_frame(self):
        prefixes = []
        for name in ('pending_completes_before_deadline', 'approved_status_in_transit'):
            r = self.rows[name]
            for p in ('wait', 'conservative'):
                self.assertEqual(r['outcomes'][p]['eligible'], [1400000000])
            self.assertEqual(r['outcomes']['discard_all_empty']['eligible'], [])
            e = next(e for e in r['events'] if e['event'] == 'poll' and e['policy'] == 'wait')
            normalized = copy.deepcopy(e); normalized['row']['instance'] = 'same-known-instance'
            prefixes.append(normalized)
        self.assertEqual(*prefixes)

    def test_out_of_order_completion_is_not_retired(self):
        r = self.rows['newer_components_wait_for_older_pending']
        poll = next(e for e in r['events'] if e['event'] == 'poll' and e['policy'] == 'conservative')
        self.assertEqual(poll['disposition'], 'possibly_inflight_wait')
        self.assertEqual(r['outcomes']['conservative']['eligible'], [1500000000])
        self.assertEqual(r['outcomes']['discard_all_empty']['eligible'], [])

    def test_restart_can_reapprove_same_stamp_candidate_loses_availability(self):
        r = self.rows['restart_reapproves_same_stamp_availability_cost']
        self.assertEqual(r['outcomes']['wait']['eligible'], [1300000000])
        self.assertEqual(r['outcomes']['conservative']['eligible'], [])
        self.assertEqual(r['outcomes']['conservative']['dropped'], [1300000000])

    def test_epoch_and_restart_reject_previously_valid_token(self):
        for name in ('epoch_revokes_old_token', 'restart_revokes_old_token'):
            e = self.rows[name]['events'][-1]
            self.assertTrue(all(e['tokens'].values()))
            self.assertFalse(any(e['permitted'].values()))

    def test_fault_controls_never_eligible(self):
        for name in ('heartbeat_expired', 'queue_expired', 'revoked', 'restart', 'invalid_status', 'reordered_status', 'confirming_stale'):
            with self.subTest(name=name):
                r = self.rows[name]
                self.assertTrue(all(not v['eligible'] for v in r['outcomes'].values()))
                for e in r['events']:
                    if e['event'] == 'poll' and e['action'] != 'expired':
                        self.assertEqual(e['disposition'], 'unknown_wait')

    def test_original_deadline_and_successor_receive_time_survive(self):
        for r in self.rows.values():
            receipts = {}
            for e in r['events']:
                if e['event'] == 'poll' and 'stamp_ns' in e:
                    key = e['policy'], e['stamp_ns']
                    self.assertEqual(e['deadline'], e['received_at']+.5)
                    self.assertEqual(receipts.setdefault(key, e['received_at']), e['received_at'])
        r = self.rows['confirmation_old_blocks_approved_successor']
        successor = next(e for e in r['events'] if e['event'] == 'poll' and e['policy'] == 'wait' and e.get('stamp_ns') == 1400000000)
        self.assertAlmostEqual(successor['received_at'], .4)

    def test_no_last_status_fallback_and_snapshot_pure(self):
        lease = LocalizationLease('offline-map')
        lease.last_status = {'state': 'valid', 'stamp_ns': 9999999999}
        before = copy.deepcopy(lease.__dict__)
        self.assertEqual(disposition(lease, 1, 'old', None, 0., 1), 'unknown_wait')
        self.assertEqual(lease.__dict__, before)

    def test_repeatable_trace_not_wall_time_benchmark(self):
        self.assertEqual(list(self.rows.values()), run_controls())


if __name__ == '__main__':
    unittest.main()
