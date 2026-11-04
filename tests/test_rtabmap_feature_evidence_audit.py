import sqlite3
import struct
from contextlib import closing
import tempfile
from pathlib import Path
import unittest

from tools.rtabmap_feature_evidence_audit import audit


class FeatureEvidenceTests(unittest.TestCase):
    def fixture(self, path, rows):
        with closing(sqlite3.connect(path)) as db:
            db.executescript('CREATE TABLE Node(id INTEGER); INSERT INTO Node VALUES(35); CREATE TABLE Admin(version TEXT); INSERT INTO Admin VALUES("fixture"); CREATE TABLE Feature(node_id INTEGER, word_id INTEGER, descriptor_size INTEGER, descriptor BLOB, depth_x FLOAT, depth_y FLOAT, depth_z FLOAT);')
            db.executemany('INSERT INTO Feature VALUES(35,?,?,?,?,?,?)', rows)
            db.commit()

    def test_good_features_duplicate_words_and_missing_query(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.db'
            self.fixture(path, [(7, 2, b'ab', 1., 2., 3.), (7, 2, b'cd', 1., 2., 0.)])
            result = audit(path, (35, 38), 582)
            row = result['nodes'][0]
            self.assertEqual(row['descriptor_classification'], {'binary_uint8': 2})
            self.assertEqual(row['extra_word_occurrences'], 1)
            self.assertEqual(row['depth_classification']['finite_z_zero'], 1)
            self.assertFalse(result['nodes'][1]['node_exists'])
            self.assertFalse(result['query_signature']['node_exists'])
            self.assertEqual(result['database_sha256_before'], result['database_sha256_after'])

    def test_corrupt_or_missing_descriptors_and_nonfinite_depth(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.db'
            self.fixture(path, [(1, 2, b'a', 1., 2., float('inf')), (2, 2, None, None, 2., 3.), (3, 2, b'', 1., 2., -1.), (4, 0, b'x', 1., 2., 1.), (5, 2, 'ab', 1., 2., 1.)])
            row = audit(path, (35,))['nodes'][0]
            self.assertEqual(row['descriptor_classification'], {'length_mismatch': 1, 'missing': 1, 'empty': 1, 'invalid_declared_size': 1, 'non_blob': 1})
            self.assertEqual(row['depth_classification']['nonfinite_xyz'], 1)
            self.assertEqual(row['depth_classification']['null_xyz'], 1)
            self.assertEqual(row['depth_classification']['finite_z_negative'], 1)

    def test_sidecar_rejected_and_missing_database_not_created(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.db'
            with self.assertRaises(FileNotFoundError):
                audit(path)
            self.assertFalse(path.exists())
            self.fixture(path, [])
            Path(str(path) + '-wal').touch()
            with self.assertRaisesRegex(ValueError, 'sidecars'):
                audit(path)

    def test_float32_format_and_strict_length_controls(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.db'
            self.fixture(path, [(1, 2, struct.pack('=ff', 1., 2.), 1., 2., 3.), (2, 2, struct.pack('=ff', float('nan'), float('inf')), 1., 2., 3.), (3, 2, b'x' * 6, 1., 2., 3.), (4, 2, b'x' * 10, 1., 2., 3.), (5, 2, b'x' * 9, 1., 2., 3.)])
            row = audit(path, (35,))['nodes'][0]
            self.assertEqual(row['descriptor_classification'], {'float32': 2, 'length_mismatch': 3})
            self.assertEqual(row['float32_value_inventory_native_endian'], {'finite_elements': 2, 'nonfinite_elements': 2, 'descriptors_with_nonfinite_elements': 1})

    def test_word_anomalies_and_duplicate_node_requests(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.db'
            self.fixture(path, [(None, 2, b'ab', 1., 2., 3.), ('bad-id', 2, b'cd', 1., 2., 3.), (7, 2, b'ef', 1., 2., 3.)])
            row = audit(path, (35,))['nodes'][0]
            self.assertEqual(row['word_id_anomalies'], {'missing': 1, 'non_integer': 1})
            self.assertEqual(row['distinct_word_ids'], 1)
            with self.assertRaisesRegex(ValueError, 'duplicate'):
                audit(path, (35, 35))


if __name__ == '__main__':
    unittest.main()
