"""KV service identity checks; no FPGA access or process signalling."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('kv_service', ROOT / 'ui/geomem/tools/competition/kv_service.py')
kv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(kv)


class ServiceIdentityTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.proc = self.root / 'proc'
        self.proc.mkdir()

    def server(self, pid, build='0x00da5041'):
        entry = self.proc / str(pid)
        entry.mkdir()
        run = self.root / ('run-' + str(pid))
        run.mkdir()
        (entry / 'cmdline').write_bytes(('python3\0/opt/source/tools/kv_link/kv_server.py\0--out\0' + str(run) + '\0').encode())
        (run / 'ready.json').write_text(json.dumps(dict(pid=pid, port=47800, build_id=build)))

    def test_valid_running_identity_is_reused(self):
        self.server(123)
        self.assertEqual(kv.running_server(self.proc)['pid'], 123)

    def test_wrong_build_refuses_without_replacing_server(self):
        self.server(123, '0xdeadbeef')
        with self.assertRaisesRegex(RuntimeError, 'leaving it untouched'):
            kv.running_server(self.proc)

    def test_duplicate_servers_refuse(self):
        self.server(123)
        self.server(124)
        with self.assertRaisesRegex(RuntimeError, 'Multiple'):
            kv.running_server(self.proc)

    def test_empty_process_table_requires_start(self):
        self.assertIsNone(kv.running_server(self.proc))



class ProfileTests(unittest.TestCase):
    """Go-live 2026-10-26: the helper reads the FPGA profiles from the installed snapshot's
    tools/car_run/fpga_profiles.env; without that file (older snapshot) only a2 exists."""

    def test_profiles_from_repo_file(self):
        p = kv.load_profiles(ROOT / 'tools/car_run/fpga_profiles.env')
        self.assertEqual(p['a2'], {'app': 'm7_pl_merge_a2', 'build': '0x00da5041', 'kv_rolling': 'none'})
        self.assertEqual((p['v3fc']['app'], p['v3fc']['kv_rolling']), ('m7_pl_v3fc', 'pl'))
        self.assertEqual((p['v3fc2']['app'], p['v3fc2']['kv_rolling']), ('m7_pl_v3fc2', 'pl'))
        self.assertEqual(p['v3fc']['build'], '0x001256b1')
        self.assertEqual(p['v3fc2'], {'app': 'm7_pl_v3fc2', 'build': '0x03c087c9', 'kv_rolling': 'pl'})
        # same RTL, same BUILD_ID, different fast clock: told apart by the app name only
        self.assertEqual(p['v3fc2_166'], {'app': 'm7_pl_v3fc2_166', 'build': '0x03c087c9', 'kv_rolling': 'pl'})
        # optional 187.5 MHz package (AR on the 100 MHz clock): its own BUILD_ID
        self.assertEqual(p['v3fc3_187'], {'app': 'm7_pl_v3fc3', 'build': '0x042a646f', 'kv_rolling': 'pl'})

    def test_missing_file_is_a2_only(self):
        self.assertEqual(sorted(kv.load_profiles(Path('/nonexistent/fpga_profiles.env'))), ['a2'])

    def test_running_other_profile_is_ours_but_not_reused(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        t = ServiceIdentityTests()
        t.root, t.proc = Path(tmp.name), Path(tmp.name) / 'proc'
        t.proc.mkdir()
        t.server(123, '0x0abcdef0')
        self.assertEqual(kv.running_server(t.proc, builds={'0x00da5041', '0x0abcdef0'})['pid'], 123)
        with self.assertRaisesRegex(RuntimeError, 'leaving it untouched'):
            kv.running_server(t.proc)                    # default: a2 identity only (old behaviour)

    def test_argument_check(self):
        import sys
        for argv in (['x', 'v3x'], ['x', 'a2', 'stop']):
            old = sys.argv
            sys.argv = argv
            try:
                with self.assertRaisesRegex(RuntimeError, r'\[v3fc2\|v3fc2_166\|v3fc3_187\|v3fc\|a2\|m1_166\|m1\|m2_166\|m2\|stop\]'):
                    kv.main()
            finally:
                sys.argv = old



class KvStartTests(unittest.TestCase):
    """kv_start.sh with stand-ins (xmutil, pgrep, kv_board.sh): no argument = the old a2 command verbatim."""
    OLD = ('start {repo} {out} --core pl --device devmem --wait irq --roi-core pl --port 47800')

    def run_start(self, *extra, fw=True, kv_fast=None):
        import os
        import subprocess
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        t = Path(tmp.name)
        repo, bin_, log = t / 'repo', t / 'bin', t / 'calls.log'
        (repo / 'tools/kv_link').mkdir(parents=True)
        (repo / 'tools/car_run').mkdir(parents=True)
        for f in ('fpga_profile.sh', 'fpga_profiles.env'):
            text = (ROOT / 'tools/car_run' / f).read_text()
            if f == 'fpga_profiles.env':     # E7 switch line; None = off (the old command lines below)
                import re
                text, n = re.subn(r'(?m)^FPGA_KV_FAST_DEFAULT=.*$', 'FPGA_KV_FAST_DEFAULT=%s' % (kv_fast or 'off'), text)
                assert n == 1, 'fpga_profiles.env must hold exactly one FPGA_KV_FAST_DEFAULT line'
            (repo / 'tools/car_run' / f).write_text(text)
        (repo / 'tools/kv_link/kv_board.sh').write_text('echo "kv_board $*" >> %s\n' % log)
        bin_.mkdir()
        for name, body in (('xmutil', 'echo "xmutil $*" >> %s' % log), ('pgrep', 'exit 1')):
            (bin_ / name).write_text('#!/bin/sh\n' + body + '\n')
            (bin_ / name).chmod(0o755)
        fwdir = t / 'fw'
        if fw:
            for app in ('m7_pl_merge_a2', 'm7_pl_v3fc', 'm7_pl_v3fc2', 'm7_pl_v3fc2_166', 'm7_pl_v3fc3'):
                (fwdir / app).mkdir(parents=True)
        out = t / 'run1'
        r = subprocess.run(['bash', str(ROOT / 'ui/geomem/tools/competition/kv_start.sh'), str(repo), str(out)] + list(extra),
                           env=dict(os.environ, PATH='%s:%s' % (bin_, os.environ['PATH']), FPGA_FW_DIR=str(fwdir),
                                    # SMOKE1: the board reads back the BUILD_ID that was asked for
                                    GEOMEM_KV_BUILD_ID_CMD='printf "%s\\n" "$FPGA_BUILD_ID"'),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=30)
        calls = log.read_text().splitlines() if log.exists() else []
        return r, calls, self.OLD.format(repo=repo, out=out)

    def test_no_argument_is_old_a2_command(self):
        r, calls, old = self.run_start()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(calls, ['xmutil unloadapp', 'xmutil loadapp m7_pl_merge_a2', 'kv_board ' + old])

    def test_v3fc_loads_its_app_and_serves_rolling(self):
        r, calls, old = self.run_start('v3fc')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(calls, ['xmutil unloadapp', 'xmutil loadapp m7_pl_v3fc', 'kv_board ' + old + ' --rolling pl'])

    def test_v3fc2_v3fc2_166_v3fc3_187_load_their_own_apps(self):
        # v3fc3_187 paused (10-26: its 187.5 MHz clock ran at 200 MHz on the board): warns and loads v3fc2
        for prof, app in (('v3fc2', 'm7_pl_v3fc2'), ('v3fc2_166', 'm7_pl_v3fc2_166'), ('v3fc3_187', 'm7_pl_v3fc2')):
            r, calls, old = self.run_start(prof)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertEqual("'v3fc3_187' is paused" in r.stderr, prof == 'v3fc3_187', r.stderr)
            self.assertEqual(calls, ['xmutil unloadapp', 'xmutil loadapp ' + app, 'kv_board ' + old + ' --rolling pl'])

    def test_kv_fast_switch_adds_the_flag_only_when_native(self):
        # E7 2026-11-02: fpga_profiles.env FPGA_KV_FAST_DEFAULT; E7B 2026-11-03: native in the repo (board A/B passed)
        self.assertIn('FPGA_KV_FAST_DEFAULT=native\n', (ROOT / 'tools/car_run/fpga_profiles.env').read_text())
        r, calls, old = self.run_start('v3fc2', kv_fast='off')
        self.assertEqual(calls[-1], 'kv_board ' + old + ' --rolling pl')
        r, calls, old = self.run_start('v3fc2', kv_fast='native')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(calls[-1], 'kv_board ' + old + ' --rolling pl --kv-fast native')
        r, calls, old = self.run_start('v3fc2', kv_fast='bogus')
        self.assertEqual(calls[-1], 'kv_board ' + old + ' --rolling pl')

    def test_missing_app_starts_a2_instead(self):
        r, calls, old = self.run_start('v3fc', fw=False)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn('starting a2 instead', r.stderr)
        self.assertEqual(calls, ['xmutil unloadapp', 'xmutil loadapp m7_pl_merge_a2', 'kv_board ' + old])


if __name__ == '__main__':
    unittest.main()
