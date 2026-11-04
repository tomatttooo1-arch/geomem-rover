"""Run-2 preflight against real temporary exports and memory documents; no hardware."""
import argparse
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from host import memory_store as ms
from perception.fpga_replay.core import Memory, Track

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'competition_demo', ROOT / 'ui/geomem/tools/competition/check_demo.py')
demo = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(demo)


class DemoPreflightTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.db = self.root / 'mapping.db'
        self.db.write_bytes(b'recorded mapping database')
        self.export = self.root / 'run_mapping' / 'export'
        self.export.mkdir(parents=True)
        self.map = self.export / 'mapping.yaml'
        self.image = self.export / 'mapping.pgm'
        self.image.write_bytes(b'P5\n2 2\n255\n\x00\x7f\xff\x00')
        self.map.write_text('image: mapping.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n',
                            encoding='utf-8')
        self.table = self.export.parent / 'sha256.txt'
        self.write_table()
        self.digest = ms.sha256_file(self.db)
        memory = Memory()
        position = np.array([1.25, -0.45, 0.15])
        track = Track('object-0001', 'private-test-label', 'green', position,
                      position - 0.05, position + 0.05, 300.0)
        memory.tracks[track.id] = track
        pending = ms.dump(memory, complete=True, runtime_boot_id='test-boot', world_frame='map',
                          pose_source='reference', sensor_source='hardware', pose_mode='odom',
                          map_identity_doc=ms.map_identity(mapping_db_dir=str(self.root)))
        self.document = ms.bind(pending, str(self.map))
        self.memory = self.root / 'memory_final.json'
        self.write_memory()

    def write_table(self):
        self.table.write_text(''.join('%s  %s\n' % (ms.sha256_file(path), path)
                                      for path in (self.db, self.map, self.image)), encoding='utf-8')

    def write_memory(self):
        ms.save_atomic(self.document, str(self.memory))

    def snapshot(self):
        return {str(path.relative_to(self.root)): (path.read_bytes(), path.stat().st_mtime_ns)
                for path in self.root.rglob('*') if path.is_file()}

    def check(self, digest=None):
        return demo.check_demo(str(self.map), str(self.memory), self.digest if digest is None else digest)

    def invoke(self, digest=None):
        output, error = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
            code = demo.main(['--map-yaml', str(self.map), '--memory', str(self.memory),
                              '--db-sha256', self.digest if digest is None else digest])
        return code, output.getvalue(), error.getvalue()

    def refused(self, code):
        before = self.snapshot()
        with self.assertRaises(ms.MemoryStoreError) as error:
            self.check()
        self.assertEqual(error.exception.code, code)
        self.assertEqual(self.snapshot(), before)

    def test_valid_bound_memory_and_export_have_only_a_read_only_summary(self):
        before = self.snapshot()
        code, output, error = self.invoke()
        self.assertEqual(code, 0)
        self.assertEqual(error, '')
        self.assertEqual(json.loads(output), {'ok': True, 'db_sha256': self.digest, 'tracks': 1})
        self.assertNotIn('private-test-label', output)
        self.assertNotIn('position_m', output)
        self.assertNotIn('history', output)
        self.assertEqual(self.snapshot(), before)

    def test_wrong_configured_database_hash_is_refused(self):
        before = self.snapshot()
        code, output, error = self.invoke('f' * 64)
        self.assertEqual(code, 6)
        self.assertEqual(output, '')
        self.assertIn('demo_db_mismatch', error)
        self.assertEqual(self.snapshot(), before)

    def test_invalid_configured_hash_is_refused(self):
        for digest in ('', 'a' * 63, 'a' * 65, 'A' * 64, 'g' * 64, 'a' * 64 + '\n', None, 123):
            with self.subTest(digest=digest):
                with self.assertRaises(argparse.ArgumentTypeError):
                    demo.check_demo('missing-map', 'missing-memory', digest)
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                demo.main(['--map-yaml', str(self.map), '--memory', str(self.memory),
                           '--db-sha256', 'A' * 64])
        self.assertEqual(error.exception.code, 2)

    def test_missing_image_is_refused(self):
        self.image.unlink()
        self.refused('map_image_missing')

    def test_missing_yaml_is_refused(self):
        self.map.unlink()
        self.refused('map_yaml_missing')

    def test_yaml_without_image_is_refused(self):
        self.map.write_text('resolution: 0.05\n', encoding='utf-8')
        self.refused('map_yaml_no_image')

    def test_missing_or_corrupt_export_table_is_refused(self):
        for contents in (None, 'invalid table\n', '%s  %s\n' % (self.digest, self.db)):
            with self.subTest(contents=contents):
                if contents is None:
                    self.table.unlink()
                else:
                    self.table.write_text(contents, encoding='utf-8')
                self.refused('demo_export_unbound')

    def test_changed_yaml_without_updated_table_is_refused(self):
        self.map.write_text(self.map.read_text(encoding='utf-8') + '# changed\n', encoding='utf-8')
        self.refused('demo_export_unbound')

    def test_changed_yaml_with_updated_table_is_refused_even_when_database_matches(self):
        self.map.write_text(self.map.read_text(encoding='utf-8') + '# changed\n', encoding='utf-8')
        self.write_table()
        current = ms.map_identity(map_yaml=str(self.map))
        self.assertTrue(ms.identity_match(self.document['map_identity'], current)[0])
        self.refused('demo_export_changed')

    def test_changed_image_is_refused_even_when_database_matches(self):
        self.image.write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
        current = ms.map_identity(map_yaml=str(self.map))
        self.assertTrue(ms.identity_match(self.document['map_identity'], current)[0])
        self.refused('demo_export_changed')

    def test_incomplete_memory_is_refused(self):
        self.document['complete'] = False
        self.write_memory()
        self.refused('incomplete')

    def test_missing_memory_document_fields_are_refused(self):
        del self.document['memory']
        self.write_memory()
        self.refused('missing_field')

    def test_damaged_track_is_refused_by_memory_store(self):
        del self.document['tracks'][0]['position_m']
        self.write_memory()
        self.refused('missing_field')

    def test_invalid_memory_json_is_refused(self):
        self.memory.write_text('{', encoding='utf-8')
        self.refused('bad_json')

    def test_unbound_or_export_only_memory_is_refused(self):
        for identity in (ms.map_identity(), ms.map_identity(mapping_db_dir=str(self.root)),
                         dict(self.document['map_identity'], kind='map_export', db_sha256=None),
                         dict(self.document['map_identity'], db_sha256=None)):
            with self.subTest(kind=identity['kind']):
                self.document['map_identity'] = identity
                self.write_memory()
                self.refused('demo_memory_unbound')

    def test_memory_bound_to_another_database_is_refused(self):
        self.document['map_identity']['db_sha256'] = 'e' * 64
        self.write_memory()
        self.refused('map_mismatch')

    def test_memory_with_only_database_identity_is_refused(self):
        self.document['map_identity'] = ms.map_identity(rtabmap_db=str(self.db))
        self.write_memory()
        self.refused('demo_export_changed')


if __name__ == '__main__':
    unittest.main()
