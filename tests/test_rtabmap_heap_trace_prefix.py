import io
import unittest
from tools.rtabmap_heap_trace_prefix import prefix


class PrefixTests(unittest.TestCase):
    def test_symbol_only_instruction_pointer(self):
        data = b'v 10300 3\ns 3 abc\ni ff 1 1\nt 1 0\na 40 1\n+ 0\nc a\n'
        self.assertEqual(prefix(io.BytesIO(data), io.BytesIO(), 10)['alive_bytes_from_events'], 64)

    def test_cutoff_precedes_following_release(self):
        data = b'v 10300 3\na 40 0\n+ 0\nc a\n- 0\nc 14\n'
        target = io.BytesIO()
        self.assertEqual(prefix(io.BytesIO(data), target, 10)['alive_bytes_from_events'], 64)
        self.assertTrue(target.getvalue().endswith(b'c a\n'))
        self.assertEqual(prefix(io.BytesIO(data), io.BytesIO(), 20)['alive_bytes_from_events'], 0)

    def test_reject_bad_version_reference_and_truncation(self):
        for data in [b'v 10300 2\n', b'v 10300 3\n+ 0\n',
                     b'v 10300 3\nc a', b'v 10300 3\ns 5 abc\n',
                     b'v 10300 3\ni 1 0 ff\n', b'v 10300 3\nX a\nX b\n',
                     b'v 10300 3\na 40 0\n- 0\n']:
            with self.subTest(data=data), self.assertRaises(ValueError):
                prefix(io.BytesIO(data), io.BytesIO(), 10)
