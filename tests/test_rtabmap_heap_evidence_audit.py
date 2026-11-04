import unittest

from tools.rtabmap_heap_evidence_audit import audit


class HeapEvidenceAuditTests(unittest.TestCase):
    def test_real_tree_has_attribution_without_nested_double_count(self):
        result = audit('snapshot=0\ntime=1\nmem_heap_B=100\nheap_tree=detailed\n'
                       'n2: 100 root\n n1: 70 allocate_a\n  n0: 70 caller\n'
                       " n0: 30 in 2 places, all below massif's threshold (1)\n")
        self.assertEqual(result['actual_stack_nodes'], 2)
        self.assertEqual(result['records'][0]['root_child_bytes'], 100)
        self.assertEqual(result['records'][0]['attributed_root_child_bytes'], 70)

    def test_detailed_label_does_not_prove_stack_evidence(self):
        result = audit('snapshot=0\ntime=1\nmem_heap_B=100\nheap_tree=detailed\n'
                       "n1: 100 root\n n0: 0 in 20 places, all below massif's threshold (1)\n")
        self.assertEqual(result['detailed_without_actual_stacks'], 1)
        self.assertEqual(result['actual_stack_nodes'], 0)
        self.assertEqual(result['records'][0]['root_child_bytes'], 0)

    def test_sparse_empty_and_truncated_records_are_distinct(self):
        result = audit('snapshot=0\ntime=1\nmem_heap_B=10\nheap_tree=empty\n'
                       'snapshot=1\ntime=2\nmem_heap_B=20\n')
        self.assertEqual(result['snapshots'], 2)
        self.assertEqual(result['metadata_complete_records'], 1)
        self.assertEqual(result['detailed_without_actual_stacks'], 0)
        self.assertEqual(result['issues'][0]['missing'], ['heap_tree'])

    def test_time_reversal_is_reported(self):
        result = audit('snapshot=0\ntime=2\nmem_heap_B=1\nheap_tree=empty\n'
                       'snapshot=1\ntime=1\nmem_heap_B=1\nheap_tree=empty\n')
        self.assertEqual(result['issues'][0]['error'], 'time decreases')

    def test_partial_tree_with_real_stack_is_incomplete(self):
        result = audit('snapshot=0\ntime=1\nmem_heap_B=100\nheap_tree=detailed\n'
                       'n2: 100 root\n n0: 70 allocate_a\n')
        self.assertEqual(result['actual_stack_nodes'], 1)
        self.assertEqual(result['records'][0]['root_child_gap_bytes'], 30)
        self.assertEqual(result['records'][0]['child_count_errors'][0]['actual'], 1)

    def test_nested_gap_orphan_and_wrong_root(self):
        for tree in ['n1: 100 root\n n1: 100 parent\n  n0: 999 child\n',
                     'n0: 100 root\n  n0: 999 orphan\n',
                     'n1: 50 root\n n0: 50 child\n']:
            with self.subTest(tree=tree):
                self.assertTrue(audit('snapshot=0\ntime=1\nmem_heap_B=100\nheap_tree=detailed\n'+tree)['issues'])


if __name__ == '__main__':
    unittest.main()
