"""Producer rate gate (2026-10-15; runs/car-bench-20261015-results/FUSED_DROP_DIG.md): host.geo_producer.RateGate.

1. Board sequences (tests/data/rate_gate_20261015, rebuilt by tools/car_run/rate_gate_fixture.py from the
   bench originals log_1015_0336 b4_1015_0336_ns daylight and log_1014_1935 b4_1014_1935_ns night):
   the old gate (mode "arrival") reproduces the board's 929 / 1165 releases in the 120 s window
   (known answer of the fixture + control arm); the stamp gate releases every 3rd camera frame in both
   (day 1238 at 30.96 fps, night 1200), > the PREREG 1080 validity floor.
2. Synthetic: depth arriving in pairs 2 periods apart (the daylight pattern), and with +-20 ms arrival
   jitter: old gate every 4th, stamp gate every 3rd. Other source rates: 10 Hz replay (every frame),
   15 fps (every 2nd), 60 fps (every 6th); the period estimate comes from InputGapCounter.
3. Positive controls: a repeated stamp and an older stamp are never released; a regression shorter
   than rebase_s releases nothing; only a sustained one re-anchors (once, event); a stamp stuck at one
   value never re-anchors. Variant: a gate without the stamp checks (mode "arrival") does release them
   (the assertions depend on the checks).
4. Wiring read from the source (no ROS): on_depth feeds InputGapCounter, then the gate, then the mailbox;
   frame_stats carries "gate"; the default is --rate-gate auto. Mailbox: still one slot, newest wins.
5. --rate-gate auto (controller 2026-10-15): stamp for --data-source live, arrival for replay and sim;
   on the daylight board sequence auto/live == stamp and auto/replay|sim == arrival (the two modes differ
   there: positive control); a sim at RTF 0.5 keeps the arrival gate's 7.5 Hz (stamp would give 5 Hz);
   stats() records mode in effect, switch and basis.
Python 3.8+.
"""
import gzip
import json
from pathlib import Path
import random
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from host.geo_producer import InputGapCounter, RateGate  # noqa: E402

DATA = ROOT / "tests" / "data" / "rate_gate_20261015"
MS = 1000000


def load(name):
    with gzip.open(str(DATA / (name + ".csv.gz")), "rt") as fp:
        next(fp)
        rows = [tuple(int(v) for v in line.split(",")) for line in fp]
    return rows, json.loads((DATA / "meta.json").read_text())[name]


def replay(gate, seq):
    """seq: (stamp_ns, arrival_ns, ...) in arrival order -> [(stamp, arrival, reason)] released."""
    ig, out = InputGapCounter(), []
    for row in seq:
        s, t = row[0], row[1]
        ig.feed(s, t)
        ok, reason, _ev = gate.decide(s, t, ig.period_ns())
        if ok:
            out.append((s, t, reason))
    return out


def steps(released, period_ns):
    return [int(round((b[0] - a[0]) / float(period_ns))) for a, b in zip(released, released[1:])]


def paired(n, period_ns, jitter_ns=0, seed=3, lat_ns=20 * MS):
    """Camera at period_ns; frames delivered in pairs: frame 2j and 2j+1 both arrive with 2j+1.
    Delivery stays in stamp order (one DDS writer -> reader): jitter never reorders."""
    rng = random.Random(seed)
    out, last = [], 0
    for i in range(n):
        s = 10 ** 18 + i * period_ns
        t = 5 * 10 ** 12 + ((i | 1) * period_ns) + lat_ns + (i % 2) * 100000 + rng.randint(-jitter_ns, jitter_ns)
        last = max(t, last + 1)
        out.append((s, last))
    return out


def regular(n, period_ns, jitter_ns=0, seed=5):
    rng = random.Random(seed)
    return [(10 ** 18 + i * period_ns + rng.randint(-jitter_ns, jitter_ns),
             5 * 10 ** 12 + i * period_ns + 30 * MS) for i in range(n)]


class BoardSequenceTests(unittest.TestCase):
    def check(self, name, old_count):
        rows, m = load(name)
        w0, w1 = m["window_start_raw_ns"], m["window_end_raw_ns"]
        inw = lambda rel: [r for r in rel if w0 <= r[1] <= w1]  # noqa: E731
        old = inw(replay(RateGate(10.0, mode="arrival"), rows))
        self.assertEqual(len(old), old_count)
        self.assertEqual(len(old), m["board_released_in_window"])
        self.assertEqual([r[:2] for r in old], [r[:2] for r in rows if r[2] and w0 <= r[1] <= w1])
        new = inw(replay(RateGate(10.0), rows))
        self.assertEqual(set(steps(new, m["period_ms"] * MS)), {3})
        self.assertGreaterEqual(len(new), 1165)
        self.assertLessEqual(len(new), 1245)
        self.assertGreater(len(new), 1080)          # PREREG 3 validity floor
        return len(old), len(new)

    def test_daylight_pairs(self):
        old, new = self.check("day_1015_0336_ns", 929)
        self.assertEqual(new, 1238)                 # 120 s / (3 x 32.30 ms)

    def test_night(self):
        old, new = self.check("night_1014_1935_ns", 1165)
        self.assertEqual(new, 1200)


class SyntheticTests(unittest.TestCase):
    P30 = 32300000

    def test_pairs_old_every_4th_new_every_3rd(self):
        seq = paired(900, self.P30)
        self.assertEqual(set(steps(replay(RateGate(10.0, mode="arrival"), seq), self.P30)[1:]), {4})
        self.assertEqual(set(steps(replay(RateGate(10.0), seq), self.P30)), {3})

    def test_arrival_jitter_does_not_move_stamp_gate(self):
        for seed in range(5):
            seq = paired(900, self.P30, jitter_ns=20 * MS, seed=seed)
            self.assertEqual(set(steps(replay(RateGate(10.0), seq), self.P30)), {3}, seed)

    def test_stamp_jitter_within_half_period(self):
        seq = regular(900, self.P30, jitter_ns=5 * MS)
        self.assertEqual(set(steps(replay(RateGate(10.0), seq), self.P30)), {3})

    def test_other_source_rates(self):
        # stamp jitter well inside a quarter period (board camera: < 0.1 ms on 32.3 ms)
        for period, k, jit in ((100 * MS, 1, 3 * MS), (66666667, 2, 3 * MS), (16666667, 6, 1 * MS),
                               (33333333, 3, 3 * MS)):
            seq = regular(600, period, jitter_ns=jit)
            rel = replay(RateGate(10.0), seq)
            rel = [r for r in rel if r[1] > seq[12][1]]   # after 8 steps: measured period, not nominal
            self.assertEqual(set(steps(rel, period)), {k}, period)
        g = RateGate(10.0)
        self.assertEqual(g.threshold(None), (3, 2.5 * 1e9 / 30))     # nominal 30 fps before the period
        self.assertEqual(g.threshold(100 * MS), (1, 50 * MS))

    def test_fixed_threshold(self):
        g = RateGate(10.0, min_step_ms=60.0)
        self.assertEqual(g.threshold(self.P30), (None, 60 * MS))
        self.assertEqual(set(steps(replay(g, regular(300, self.P30)), self.P30)), {2})


class StampControlTests(unittest.TestCase):
    P = 32300000

    def base(self, gate, n=20):
        seq = regular(n, self.P)
        replay(gate, seq)
        return seq[-1]

    def test_repeat_and_older_never_released(self):
        g = RateGate(10.0)
        s0, t0 = 10 ** 18, 10 ** 12
        self.assertEqual(g.decide(s0, t0)[:2], (True, None))
        self.assertEqual(g.decide(s0, t0 + 200 * MS)[:2], (False, "stamp_repeat"))
        ok, reason, ev = g.decide(s0 - 1 * MS, t0 + 300 * MS)
        self.assertEqual((ok, reason), (False, "stamp_older"))
        self.assertEqual(ev["event"], "depth_stamp_regression")
        self.assertEqual(g.decide(s0 + 97 * MS, t0 + 400 * MS)[:2], (True, None))
        st = g.stats()
        self.assertEqual(st["skipped"], {"rate": 0, "stamp_repeat": 1, "stamp_older": 1})

    def test_variant_without_stamp_checks_releases_them(self):
        g = RateGate(10.0, mode="arrival")
        s0, t0 = 10 ** 18, 10 ** 12
        g.decide(s0, t0)
        self.assertTrue(g.decide(s0, t0 + 200 * MS)[0])
        self.assertTrue(g.decide(s0 - 1 * MS, t0 + 400 * MS)[0])

    def test_short_regression_releases_nothing(self):
        g = RateGate(10.0, rebase_s=2.0)
        s, t = self.base(g)
        # the clock went back 10 s for 1.9 s of arrivals, then the old timeline resumes
        rel = replay(g, [(s - 10 * 10 ** 9 + i * self.P, t + (i + 1) * self.P) for i in range(58)])
        self.assertEqual(rel, [])
        self.assertEqual(g.counts["regression_episodes"], 1)
        rel = replay(g, [(s + (i + 1) * self.P, t + (i + 60) * self.P) for i in range(10)])
        self.assertTrue(rel and all(r[2] is None for r in rel))

    def test_sustained_regression_rebases_once(self):
        g = RateGate(10.0, rebase_s=2.0)
        s, t = self.base(g)
        seq = [(s - 10 * 10 ** 9 + i * self.P, t + (i + 1) * self.P) for i in range(200)]
        evs = []
        for st, at in seq:
            _ok, reason, ev = g.decide(st, at, self.P)
            if ev is not None:
                evs.append(ev["event"])
        self.assertEqual(evs, ["depth_stamp_regression", "depth_stamp_rebase"])
        self.assertEqual(g.counts["rebase"], 1)
        rebase_at = g.counts["stamp_older"]          # older arrivals refused before the rebase
        self.assertGreaterEqual(rebase_at * self.P, 2 * 10 ** 9 - self.P)
        self.assertGreaterEqual(g.counts["released"], 7 + 1 + 45)   # 7 before, the rebase, every 3rd of 138

    def test_stuck_stamp_never_rebases(self):
        g = RateGate(10.0, rebase_s=2.0)
        _s, t = self.base(g)
        s = g.last_stamp_ns                          # the stamp stuck at the last released one
        rel = replay(g, [(s, t + (i + 1) * self.P) for i in range(300)])
        self.assertEqual(rel, [])
        self.assertEqual(g.counts["stamp_repeat"], 300)


class WiringTests(unittest.TestCase):
    def test_on_depth_order_and_frame_stats(self):
        src = (ROOT / "host" / "ros_geo_producer.py").read_text()
        body = src[src.index("        def on_depth(self, msg):"):src.index("        def lookup(self, target")]
        feed, gate, put = (body.index("self.input_gaps.feed("), body.index("self.gate.decide("),
                           body.index("self.slot.put("))
        self.assertLess(feed, gate)
        self.assertLess(gate, put)
        self.assertNotIn("last_put_ns", src)
        self.assertIn('fs["gate"] = self.gate.stats()', src)
        self.assertIn('"rate_gate": dict(node.gate.stats()', src)
        # frames.jsonl rows (--stats-out) carry the same gate body as the topic (board 10-15: 0 rows had it)
        i_row = src.index('row = {"frame_id"')
        row = src[i_row:src.index("write_row(row)", i_row)]
        self.assertIn('"gate": fs["gate"]', row)
        self.assertLess(src.index('fs["gate"] = self.gate.stats()'), i_row)

    def test_default_is_auto_gate(self):
        from host.ros_geo_producer import parse_args
        a, _ = parse_args(["--data-source", "replay", "--backend", "cpu", "--ground-z-m", "0"])
        self.assertEqual((a.rate_gate, a.rate_min_step_ms, a.rate_nominal_fps, a.rate_rebase_s),
                         ("auto", 0.0, 30.0, 2.0))

    def test_node_builds_gate_from_resolved_mode(self):
        src = (ROOT / "host" / "ros_geo_producer.py").read_text()
        self.assertIn("gate_mode, gate_basis = rate_gate_mode(a)", src)
        self.assertIn("RateGate(a.rate_hz, mode=gate_mode,", src)
        self.assertIn("switch=a.rate_gate, basis=gate_basis)", src)
        self.assertNotIn("mode=a.rate_gate", src)


def gate_for(src, *extra):
    """The producer's gate for these flags, built the way the node builds it."""
    from host.ros_geo_producer import parse_args, rate_gate_mode
    a, _ = parse_args(["--data-source", src, "--backend", "cpu", "--ground-z-m", "0"] + list(extra))
    mode, basis = rate_gate_mode(a)
    return RateGate(a.rate_hz, mode=mode, min_step_ms=a.rate_min_step_ms, nominal_fps=a.rate_nominal_fps,
                    rebase_s=a.rate_rebase_s, switch=a.rate_gate, basis=basis)


def sim_rtf(n, rtf, fps=30.0):
    """Gazebo camera at fps of SIM time, sim running at real-time factor rtf: stamps step 1/fps,
    arrivals step 1/(fps*rtf) of wall time."""
    p_sim, p_wall = int(1e9 / fps), int(1e9 / (fps * rtf))
    return [(10 ** 18 + i * p_sim, 5 * 10 ** 12 + i * p_wall + 20 * MS) for i in range(n)]


class AutoModeTests(unittest.TestCase):
    """--rate-gate auto (controller 2026-10-15): stamp for --data-source live, arrival for replay / sim."""

    def test_resolution(self):
        from host.ros_geo_producer import parse_args, rate_gate_mode
        base = ["--backend", "cpu", "--ground-z-m", "0"]
        want = {("live",): ("stamp", "auto: --data-source live"),
                ("replay",): ("arrival", "auto: --data-source replay"),
                ("sim",): ("arrival", "auto: --data-source sim"),
                ("sim", "--rate-gate", "stamp"): ("stamp", "--rate-gate stamp"),
                ("live", "--rate-gate", "arrival"): ("arrival", "--rate-gate arrival"),
                ("replay", "--rate-gate", "auto"): ("arrival", "auto: --data-source replay")}
        for k, v in want.items():
            a, _ = parse_args(["--data-source", k[0]] + base + list(k[1:]))
            self.assertEqual(rate_gate_mode(a), v, k)

    def test_stats_record_mode_switch_basis(self):
        g = gate_for("live")
        g.decide(10 ** 18, 10 ** 12, None)
        st = g.stats()
        self.assertEqual((st["mode"], st["switch"], st["basis"]), ("stamp", "auto", "auto: --data-source live"))
        st = gate_for("sim").stats()
        self.assertEqual((st["mode"], st["switch"], st["basis"]), ("arrival", "auto", "auto: --data-source sim"))
        st = gate_for("sim", "--rate-gate", "stamp").stats()
        self.assertEqual((st["mode"], st["switch"]), ("stamp", "stamp"))
        st = RateGate(10.0).stats()                       # bare gate: switch = mode, no basis
        self.assertEqual((st["mode"], st["switch"], st["basis"]), ("stamp", "stamp", None))

    def test_live_auto_equals_stamp_daylight(self):
        rows, _m = load("day_1015_0336_ns")
        auto, stamp, old = (replay(gate_for("live"), rows), replay(RateGate(10.0, mode="stamp"), rows),
                            replay(RateGate(10.0, mode="arrival"), rows))
        self.assertEqual(auto, stamp)
        self.assertNotEqual(auto, old)                    # positive control: the two modes differ here
        self.assertGreater(len(auto), len(old))

    def test_replay_and_sim_auto_equal_arrival_daylight(self):
        rows, _m = load("day_1015_0336_ns")
        old, stamp = replay(RateGate(10.0, mode="arrival"), rows), replay(RateGate(10.0, mode="stamp"), rows)
        for src in ("replay", "sim"):
            self.assertEqual(replay(gate_for(src), rows), old, src)
            self.assertNotEqual(replay(gate_for(src), rows), stamp, src)

    def test_sim_rtf_half_keeps_arrival_rate(self):
        """Handover sec. 3: RTF 0.5, 30 fps sim camera = 15 fps wall. arrival gate: every 2nd (7.5 Hz
        wall); stamp gate: every 3rd (5 Hz). auto under sim must give the 7.5 Hz, under live the 5 Hz."""
        seq = sim_rtf(301, 0.5)                            # 20 s wall
        sim, live = replay(gate_for("sim"), seq), replay(gate_for("live"), seq)
        self.assertEqual(len(sim), 151)                    # every 2nd of 301: 7.5 Hz wall
        self.assertEqual(len(live), 101)                   # every 3rd: 5 Hz wall (control: modes differ)
        self.assertEqual(sim, replay(RateGate(10.0, mode="arrival"), seq))
        self.assertEqual(replay(gate_for("sim", "--rate-gate", "stamp"), seq), live)

    def test_mailbox_newest_wins(self):
        from host.kv_link import LatestSlot
        slot, g = LatestSlot(), RateGate(10.0)
        for s, t in regular(30, 32300000):
            if g.decide(s, t, 32300000)[0]:
                slot.put(s)
        self.assertEqual(slot.puts, 10)
        self.assertEqual(slot.superseded, 9)
        self.assertEqual(slot.take(timeout=0.01), 10 ** 18 + 27 * 32300000)


if __name__ == "__main__":
    unittest.main()
