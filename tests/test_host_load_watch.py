"""tools/host_load_watch.py: the in-run load gate of the host replay arms (定位门第 11 棒, owner 10-17 12:31).

Known answers: the kernel line formats parse to the numbers in them; the limits are the start gate's (g8_go.sh:
load1 > 24 busy, io full avg10 >= 30 busy) on both sides of each bound; a window whose samples stay within is valid,
one sample over voids it; samples that do not cover the window void it as blind; the watch subcommand writes rows
that the verdict subcommand reads back (a real short run against a stand-in /proc)."""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from tools import host_load_watch as lw

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / 'tools/host_load_watch.py'

LOADAVG = '22.31 25.00 27.28 58/7723 2108223\n'
PSI_IO = ('some avg10=2.58 avg60=4.78 avg300=5.98 total=229673071246\n'
          'full avg10=16.29 avg60=26.15 avg300=30.42 total=210378337882\n')


def s(wall, load1=20., io=5., own=None):
    r = {'event': 'sample', 'wall': wall, 'load1': load1, 'io_full_avg10': io}
    if own is not None:
        r['own'] = own
    return r


def series(start, end, step=2., **kw):
    out, t = [], start
    while t <= end + 1e-9:
        out.append(s(t, **kw))
        t += step
    return out


class ParseTests(unittest.TestCase):
    def test_loadavg(self):
        self.assertEqual(lw.parse_loadavg(LOADAVG), {'load1': 22.31, 'load5': 25.0, 'load15': 27.28,
                                                     'runnable': 58, 'threads': 7723})

    def test_psi_full_avg10(self):
        self.assertEqual(lw.parse_psi_full_avg10(PSI_IO), 16.29)
        self.assertIsNone(lw.parse_psi_full_avg10('some avg10=1.00 avg60=0 avg300=0 total=1\n'))

    def test_limits_are_the_start_gate_values(self):
        self.assertEqual((lw.MAX_LOAD, lw.MAX_IO), (24.0, 30.0))
        # the start gate's own text, so a change of either side shows up here
        g8 = ("awk '$1<=24{exit 1}{exit 0}'", "exit (a[2]<30)?1:0")
        doc = TOOL.read_text()
        for frag in g8:
            self.assertIn(frag, doc)


class BusyTests(unittest.TestCase):
    def test_load_bound(self):
        self.assertEqual(lw.busy(s(0, load1=24.0)), [])          # start gate: <= 24 is open
        self.assertEqual(lw.busy(s(0, load1=24.01)), ['load'])

    def test_io_bound(self):
        self.assertEqual(lw.busy(s(0, io=29.99)), [])            # start gate: < 30 is open
        self.assertEqual(lw.busy(s(0, io=30.0)), ['io'])

    def test_unreadable_is_busy(self):
        self.assertEqual(lw.busy(s(0, load1=None, io=None)), ['load_unreadable', 'io_unreadable'])


class VerdictTests(unittest.TestCase):
    def test_within_is_valid(self):
        v = lw.verdict(series(100., 200., load1=23.9, io=29.9), 100., 200.)
        self.assertEqual(v['verdict'], 'valid')
        self.assertEqual(v['reasons'], [])
        self.assertEqual(v['n'], 51)
        self.assertEqual(v['load1_max'], 23.9)

    def test_one_sample_over_voids(self):
        rows = series(100., 200.)
        rows[30]['load1'] = 29.17                             # g8r_3's launch.txt reading
        v = lw.verdict(rows, 100., 200.)
        self.assertEqual(v['verdict'], 'void')
        self.assertEqual(v['reasons'], ['over:load'])
        self.assertEqual(v['over_samples'], 1)
        self.assertEqual(v['first_over']['since_start_s'], 60.0)
        rows = series(100., 200.)
        rows[-1]['io_full_avg10'] = 30.0
        self.assertEqual(lw.verdict(rows, 100., 200.)['reasons'], ['over:io'])

    def test_samples_outside_window_do_not_count(self):
        rows = series(90., 210.)
        for r in rows:
            if r['wall'] < 100. or r['wall'] > 200.:
                r['load1'] = 40.
        self.assertEqual(lw.verdict(rows, 100., 200.)['verdict'], 'valid')

    def test_blind_windows_void(self):
        self.assertEqual(lw.verdict([], 100., 200.)['reasons'], ['blind:no_samples'])
        self.assertIn('blind:late_first_sample', lw.verdict(series(107., 200.), 100., 200.)['reasons'])
        self.assertIn('blind:early_last_sample', lw.verdict(series(100., 193.), 100., 200.)['reasons'])
        rows = [r for r in series(100., 200.) if not 140. < r['wall'] < 148.]
        v = lw.verdict(rows, 100., 200.)
        self.assertEqual(v['reasons'], ['blind:gap'])
        self.assertEqual(v['verdict'], 'void')
        # exactly the allowed gap is covered
        self.assertEqual(lw.verdict(series(100., 200., step=6.) + [s(200.)], 100., 200.)['verdict'], 'valid')

    def test_own_threads_readout_does_not_judge(self):
        rows = series(100., 110., own={'c': 7})
        rows[0]['own'] = {'c': None}
        v = lw.verdict(rows, 100., 110.)
        self.assertEqual(v['verdict'], 'valid')
        self.assertEqual(v['own_rd_threads_mean_max'], [7.0, 7])

    def test_torn_last_line_is_skipped(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / 'w.jsonl'
            p.write_text(json.dumps({'event': 'start', 'wall': 1.}) + '\n' + json.dumps(s(2.)) + '\n{"event": "sam')
            self.assertEqual(lw.read_log(p), [s(2.)])


class SampleTests(unittest.TestCase):
    def test_sample_reads_both_files(self):
        files = {'/proc/loadavg': LOADAVG, '/proc/pressure/io': PSI_IO}
        r = lw.sample([], {}, read=lambda p: files[p])
        self.assertEqual((r['load1'], r['io_full_avg10'], r['runnable']), (22.31, 16.29, 58))
        self.assertNotIn('own', r)

    def test_unreadable_sample_is_marked(self):
        def boom(p):
            raise OSError(p)
        r = lw.sample([], {}, read=boom)
        self.assertIsNone(r['load1'])
        self.assertIsNone(r['io_full_avg10'])
        self.assertEqual(lw.busy(r), ['load_unreadable', 'io_unreadable'])

    def test_count_rd(self):
        with tempfile.TemporaryDirectory() as d:
            tasks = Path(d) / 'tasks'
            tasks.write_text('%d\n999999999\n' % os.getpid())     # this process (running: R) and a gone tid
            self.assertEqual(lw.count_rd(str(tasks)), 1)
            self.assertIsNone(lw.count_rd(str(Path(d) / 'missing')))


class WatchCliTests(unittest.TestCase):
    def test_watch_then_verdict(self):
        """A real watcher process (interval 0.2 s) stopped by SIGTERM; its log judged over the window it ran."""
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / 'w.jsonl'
            env = dict(os.environ, PYTHONPATH=str(ROOT))
            t0 = time.time()
            p = subprocess.Popen([sys.executable, str(TOOL), 'watch', '--out', str(out), '--interval', '0.2'], env=env)
            time.sleep(1.5)
            p.send_signal(signal.SIGTERM)
            self.assertEqual(p.wait(timeout=10), 0)
            t1 = time.time()
            rows = [json.loads(l) for l in out.read_text().splitlines()]
            self.assertEqual(rows[0]['event'], 'start')
            self.assertEqual(rows[-1]['event'], 'stop')
            self.assertEqual(rows[-1]['signal'], signal.SIGTERM)
            samples = lw.read_log(out)
            self.assertGreaterEqual(len(samples), 5)
            self.assertTrue(all(isinstance(r['load1'], float) for r in samples))
            # judged with limits far above / below the host's current load: valid, then void
            hi = subprocess.run([sys.executable, str(TOOL), 'verdict', '--log', str(out), '--start', str(t0 + 0.3),
                                 '--end', str(t1 - 0.3), '--max-load', '1e9', '--max-io', '101', '--max-gap-s', '1'],
                                env=env, stdout=subprocess.PIPE, universal_newlines=True)
            self.assertEqual(hi.returncode, 0, hi.stdout)
            self.assertEqual(json.loads(hi.stdout)['verdict'], 'valid')
            lo = subprocess.run([sys.executable, str(TOOL), 'verdict', '--log', str(out), '--start', str(t0 + 0.3),
                                 '--end', str(t1 - 0.3), '--max-load', '-1', '--max-io', '101', '--max-gap-s', '1'],
                                env=env, stdout=subprocess.PIPE, universal_newlines=True)
            self.assertEqual(lo.returncode, 1, lo.stdout)
            self.assertEqual(json.loads(lo.stdout)['reasons'], ['over:load'])

    def test_watch_ends_with_watched_pid(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / 'w.jsonl'
            env = dict(os.environ, PYTHONPATH=str(ROOT))
            child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(0.6)'])
            p = subprocess.Popen([sys.executable, str(TOOL), 'watch', '--out', str(out), '--interval', '0.1',
                                  '--pid', str(child.pid)], env=env)
            child.wait()
            self.assertEqual(p.wait(timeout=10), 0)
            rows = [json.loads(l) for l in out.read_text().splitlines()]
            self.assertEqual(rows[-1], dict(rows[-1], event='stop', signal=None))

    def test_verdict_without_log(self):
        r = subprocess.run([sys.executable, str(TOOL), 'verdict', '--log', '/nonexistent/x.jsonl', '--start', '0',
                            '--end', '1'], env=dict(os.environ, PYTHONPATH=str(ROOT)), stdout=subprocess.PIPE,
                           universal_newlines=True)
        self.assertEqual(r.returncode, 2)
        self.assertEqual(json.loads(r.stdout)['reasons'], ['blind:no_log'])


if __name__ == '__main__':
    unittest.main()
