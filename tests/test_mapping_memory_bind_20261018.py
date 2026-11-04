"""In-stack mapping run -> object memory identity (2026-10-18, relay 3; docs/design/TELEOP_NAV_ARB_20261018.md 2.4).

Executed: mapping_standin.py (as a process: OUT/sha256.txt in the host/memory_store convention), MappingCore.identify
(table present / missing -> written / no database), mapping_memory_bind.bind_run on a built RUN directory (stack paths
/o/rtabmap_db in mapping_status.json and sha256.txt, host paths through car.json), car_down.sh calling it (marks, exit
code unchanged by a failed bind), and the car_up dry-run identity (mapping_pending on the stack's database directory,
CAR_MEMORY_IN refused). Not executed here: the runner writing memory_final.json in a stack, rtabmap itself.
"""
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tools', 'car_run'))

import mapping_ctl  # noqa: E402
import mapping_memory_bind as mmb  # noqa: E402
import mapping_standin  # noqa: E402
from host import memory_store as ms  # noqa: E402
import test_memory_persist as tmp_  # noqa: E402
import test_odom_nav2 as ton  # noqa: E402

CAR = os.path.join(ROOT, 'tools', 'car_run')


def _text(path):
    with open(path) as f:
        return f.read()


def _json(path):
    with open(path) as f:
        return json.load(f)


def _sha(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


class TestStandinTable(unittest.TestCase):
    def test_standin_writes_the_memory_store_table(self):
        with tempfile.TemporaryDirectory() as d:
            out, dbd = os.path.join(d, 'run_x'), os.path.join(d, 'db')
            env = dict(os.environ, OUT=out, DB_DIR=dbd, RUN_TIME='20261018_200000')
            p = subprocess.Popen([sys.executable, os.path.join(CAR, 'mapping_standin.py')], env=env,
                                 stderr=subprocess.PIPE, universal_newlines=True)
            line = ''
            t0 = time.time()
            while 'running --' not in line and time.time() - t0 < 20:
                line = p.stderr.readline()
            p.send_signal(signal.SIGINT)
            p.communicate(timeout=20)
            self.assertEqual(p.returncode, 0)
            yml = os.path.join(out, 'export', '20261018_200000.yaml')
            db_sha, db_path, note = ms.export_source_db(yml, ms.sha256_file(yml))
            self.assertEqual(db_path, os.path.join(dbd, '20261018_200000.db'), note)
            self.assertEqual(db_sha, _sha(db_path))


class TestIdentify(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix='mapid_')
        self.out = os.path.join(self.d, 'run_t')
        self.db = os.path.join(self.d, 't.db')
        with open(self.db, 'wb') as f:
            f.write(b'rtabmap db bytes')
        self.yml = mapping_standin.write_export(self.out, 't')

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def core(self, db):
        c = mapping_ctl.MappingCore(self.out, self.d, db=db)
        self.assertTrue(c.request_stop())
        c.on_child_exit(0)
        self.assertEqual(c.state, 'exported')
        return c

    def test_table_present_is_read(self):
        mapping_standin.write_sha_table(self.out, self.db)
        before = _text(os.path.join(self.out, 'sha256.txt'))
        c = self.core(self.db)
        self.assertEqual(c.db_sha256, _sha(self.db))
        self.assertTrue(c.sha_table.startswith('ok'), c.sha_table)
        self.assertEqual(_text(os.path.join(self.out, 'sha256.txt')), before)
        self.assertEqual(c.status()['db_sha256'], _sha(self.db))

    def test_table_missing_is_written(self):
        c = self.core(self.db)
        self.assertEqual(c.db_sha256, _sha(self.db))
        self.assertTrue(c.sha_table.startswith('written by mapping_ctl'), c.sha_table)
        self.assertEqual(ms.export_source_db(self.yml, ms.sha256_file(self.yml))[0], _sha(self.db))

    def test_table_without_db_is_replaced_and_kept(self):
        with open(os.path.join(self.out, 'sha256.txt'), 'w') as f:   # sha256sum of the db failed: export only
            f.write('%s  %s\n' % (_sha(self.yml), self.yml))
        c = self.core(self.db)
        self.assertEqual(c.db_sha256, _sha(self.db))
        self.assertTrue(os.path.isfile(os.path.join(self.out, 'sha256.rtabmap_car.txt')))

    def test_no_database_still_exported(self):
        c = self.core(os.path.join(self.d, 'missing.db'))
        self.assertIsNone(c.db_sha256)
        self.assertIn('not usable', c.sha_table)


def build_run(run, pending_dir='/o/rtabmap_db', kind='pending', with_db=True, state='exported'):
    """A host RUN as a CAR_MAPPING=1 sim trip leaves it: stack /o = RUN, database dir /o/rtabmap_db."""
    host_db_dir = os.path.join(run, 'rtabmap_db')
    out = os.path.join(host_db_dir, 'run_t')
    os.makedirs(out)
    db = os.path.join(host_db_dir, 't.db')
    with open(db, 'wb') as f:
        f.write(b'db')
    yml = mapping_standin.write_export(out, 't')
    rows = ['%s  %s\n' % (_sha(db), '/o/rtabmap_db/t.db')] if with_db else []
    for n in sorted(os.listdir(os.path.join(out, 'export'))):
        rows.append('%s  /o/rtabmap_db/run_t/export/%s\n' % (_sha(os.path.join(out, 'export', n)), n))
    with open(os.path.join(out, 'sha256.txt'), 'w') as f:
        f.write(''.join(rows))
    with open(os.path.join(run, 'mapping_status.json'), 'w') as f:
        json.dump({'state': state, 'yaml': '/o/rtabmap_db/run_t/export/t.yaml', 'out': '/o/rtabmap_db/run_t'}, f)
    with open(os.path.join(run, 'car.json'), 'w') as f:
        json.dump({'rtabmap': {'db_dir': host_db_dir, 'db_dir_in_stack': '/o/rtabmap_db'}}, f)
    ident = ms.map_identity(mapping_db_dir=pending_dir) if kind == 'pending' else ms.map_identity()
    ms.save_atomic(tmp_._doc(ident=ident), os.path.join(run, 'memory_final.json'))
    return db, yml


class TestBindRun(unittest.TestCase):
    def setUp(self):
        self.run = tempfile.mkdtemp(prefix='mapbind_')

    def tearDown(self):
        shutil.rmtree(self.run, ignore_errors=True)

    def test_bound_and_loadable_by_the_next_run(self):
        db, yml = build_run(self.run)
        pending_sha = _sha(os.path.join(self.run, 'memory_final.json'))
        rc, out = mmb.bind_run(self.run)
        self.assertEqual((rc, out['bind']), (0, 'bound'), out)
        doc = _json(os.path.join(self.run, 'memory_final.json'))
        self.assertEqual(doc['map_identity']['kind'], 'rtabmap_db')
        self.assertEqual(doc['map_identity']['db_sha256'], _sha(db))
        self.assertEqual(_sha(os.path.join(self.run, 'memory_final.pending.json')), pending_sha)
        rec = _json(os.path.join(self.run, 'memory_bind.json'))
        self.assertEqual(rec['pending_sha256'], pending_sha)
        # the second trip (CAR_MAP_YAML = this export) computes the same identity: the bound file is accepted
        ms.check(ms.read_document(os.path.join(self.run, 'memory_final.json')), ms.map_identity(map_yaml=yml))
        # idempotent: a second car_down call says already bound and changes nothing
        rc2, out2 = mmb.bind_run(self.run)
        self.assertEqual((rc2, out2['bind']), (0, 'already_bound'))

    def test_refusals_keep_the_file(self):
        cases = (
            (dict(kind='none'), 'not mapping_pending'),
            (dict(pending_dir='/data/fpga/rtabmap'), 'bind_wrong_dir'),
            (dict(with_db=False), 'does not name its database'),
            (dict(state='failed'), "mapping state 'failed'"),
        )
        for kw, word in cases:
            run = tempfile.mkdtemp(prefix='mapbindneg_')
            try:
                build_run(run, **kw)
                before = _sha(os.path.join(run, 'memory_final.json'))
                rc, out = mmb.bind_run(run)
                self.assertEqual(rc, 1, kw)
                self.assertIn(word, out['why'], kw)
                self.assertEqual(_sha(os.path.join(run, 'memory_final.json')), before, kw)
                self.assertFalse(os.path.exists(os.path.join(run, 'memory_final.pending.json')), kw)
            finally:
                shutil.rmtree(run, ignore_errors=True)

    def test_database_changed_after_the_table(self):
        db, _ = build_run(self.run)
        with open(db, 'ab') as f:
            f.write(b'x')
        rc, out = mmb.bind_run(self.run)
        self.assertEqual(rc, 1)
        self.assertIn('differs from its sha256.txt', out['why'])

    def test_not_a_mapping_run(self):
        self.assertEqual(mmb.bind_run(self.run)[1]['bind'], 'skipped')


class TestCarDown(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='mapdown_')
        self.run = os.path.join(self.tmp, 'run')
        os.mkdir(self.run)
        self.env = dict(os.environ, CAR_STATE_DIR=os.path.join(self.tmp, 'st'), CAR_DOCKER='false',
                        CAR_LOCK='false', CAR_STACK_STOP_S='1')
        os.mkdir(self.env['CAR_STATE_DIR'])

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def down(self):
        return subprocess.run(['bash', os.path.join(CAR, 'car_down.sh'), '--run', self.run], env=self.env,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)

    def marks(self):
        return _text(os.path.join(self.run, 'down_marks.txt'))

    def test_car_down_binds(self):
        db, _ = build_run(self.run)
        r = self.down()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('memory bind rc=0', self.marks())
        doc = _json(os.path.join(self.run, 'memory_final.json'))
        self.assertEqual(doc['map_identity']['db_sha256'], _sha(db))

    def test_failed_bind_is_a_mark_only(self):
        build_run(self.run, kind='none')
        r = self.down()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('memory bind rc=1', self.marks())
        self.assertIn('not mapping_pending', self.marks())

    def test_no_mapping_no_mark(self):
        r = self.down()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('memory bind', self.marks())


class TestCarUpIdentity(ton.BoardBase):
    def test_board_mapping_identity_pending_and_memory_in_refused(self):
        r = self.up(*(self.ARGS + ['--dry-run']), CAR_MAPPING='1', CAR_TELEOP='1', CAR_MAP_YAML='')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('map identity: mapping run (pending', r.stdout)
        self.assertIn('-> mapping_pending', r.stdout)
        pose = self.line(r.stdout, '    [pose]') if '    [pose]' in r.stdout else ''
        self.assertNotIn('<CAR_MAP_YAML', r.stdout)
        self.assertNotIn('<CAR_MAP_YAML', pose)
        names = self.names(r.stdout)
        self.assertLess(names.index('blank_map'), names.index('bench_pose'))
        m = self.line(r.stdout, '    [mapping]')
        self.assertIn('--db /data/fpga/rtabmap/', m)
        with tempfile.TemporaryDirectory() as d:
            mem = os.path.join(d, 'm.json')
            ms.save_atomic(tmp_._doc(), mem)
            r = self.up(*(self.ARGS + ['--dry-run']), CAR_MAPPING='1', CAR_MAP_YAML='', CAR_MEMORY_IN=mem)
            self.assertEqual(r.returncode, 2)
            self.assertIn('mapping_pending', r.stderr)


if __name__ == '__main__':
    unittest.main()
