"""N5 stand-in and stall tools (docs/design/NANO_PARTITION_LOAD_20260927.md)."""
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "nano_partition"))

import n5_map_replay as rp  # noqa: E402
import stalls  # noqa: E402
from host.m7_rolling import NavigationDelta  # noqa: E402
from host.rolling_nav_layer import decode_delta, encode_delta  # noqa: E402


def delta(seq, acquired, changes):
    return NavigationDelta(1, seq, seq, acquired, "mono", acquired + 200_000_000, (0, 0, 0), (80, 80, 32),
                           0.05, tuple(changes), "cpu_reference")


class ReplayTest(unittest.TestCase):
    def frames(self):
        return [rp.split_wire(encode_delta(delta(i, 1000 * i, [(i, 1, 2, 2)]))) for i in range(1, 4)]

    def test_file_roundtrip(self):
        fr = self.frames()
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "x.bin"
            rp.write_frames(p, fr)
            self.assertEqual(rp.read_frames(p), fr)

    def test_sequencer_rewrites_header_loops_and_keeps_payload(self):
        s = rp.ReplaySequencer(self.frames(), 1)
        got = [decode_delta(s.next_wire(10_000 + k)[0]) for k in range(5)]
        self.assertEqual([g.delta_seq for g in got], [1, 2, 3, 4, 5])
        self.assertEqual([g.changes[0][0] for g in got], [1, 2, 3, 2, 3])  # loops from index 1
        self.assertEqual(got[4].acquired_ns, 10_004)
        self.assertEqual(got[4].expires_at_ns, 10_004 + 200_000_000)
        s.new_epoch()
        g = decode_delta(s.next_wire(7)[0])
        self.assertEqual((g.epoch, g.delta_seq), (2, 1))

    def test_bad_loop_point(self):
        with self.assertRaises(ValueError):
            rp.ReplaySequencer(self.frames(), 3)


class StallTest(unittest.TestCase):
    def test_merge(self):
        s = 1_000_000_000
        ev = stalls.merge([(0, 600_000_000, "status"), (s, 2 * s, "detector"), (5 * s, 6 * s, "status")])
        self.assertEqual(len(ev), 2)
        self.assertEqual((ev[0]["start"], ev[0]["end"]), (0, 2 * s))
        self.assertEqual(set(ev[0]["sources"]), {"status", "detector"})

    def test_verdict(self):
        base = {"rows": 30, "expected": 30}
        io = dict(base, io_ticks_ms=900, procs_blocked_rows=3)
        self.assertEqual(stalls.verdict(1000.0, io, True), "io")
        self.assertEqual(stalls.verdict(1000.0, io, False), "undetermined")
        self.assertEqual(stalls.verdict(1000.0, dict(base, io_ticks_ms=50, procs_blocked_rows=0), True), "non_io")
        self.assertEqual(stalls.verdict(1000.0, dict(base, io_ticks_ms=300, procs_blocked_rows=0), True),
                         "undetermined")
        self.assertEqual(stalls.verdict(1000.0, dict(io, rows=5), True), "undetermined")


if __name__ == "__main__":
    unittest.main()
