"""KV260 <-> Nano clock mapping (host/kv_clock_sync.py, B2 2026-10-11).

Known answers:
  1. simulated clocks with a set offset and drift (+3.2 s, +50 ppm; also -0.7 s,
     -80 ppm), asymmetric delays, delay spikes and loss in both directions: every
     query inside the window (and 2 s past it) is within the stated bound, and the
     bound is not vacuous;
  2. zero offset / zero drift, simulated and over a real UDP loopback (both ends
     read the same CLOCK_MONOTONIC_RAW, so the truth is exactly 0);
  3. timebase (spec section 9): fake clocks where NTP slews CLOCK_MONOTONIC by
     +/-500 ppm mid-run and CLOCK_MONOTONIC_RAW is untouched; stamping with RAW
     keeps every estimate inside its bound, and the same check stamped with the
     slewed MONOTONIC must fail (positive control). Plus wiring: the shipped
     defaults (client, responder, PL sampler, server t2) all read RAW.
PL timebase: simulated PL counter (+30 ppm vs KV, latch anywhere inside the
bracket, preempted brackets) mapped PL -> KV -> Nano within bound; the UIO/MAGIC
gate and the explicit degraded report.
Positive controls (the known-answer check itself must FAIL): t1<->t2, t2<->t3,
t1<->t4 swapped, offset sign flipped, PL count paired with a stale KV stamp.
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_kv_clock_sync.py
(also with /usr/bin/python3.8).
"""
import os
from pathlib import Path
import random
import struct
import tempfile
import threading
import time
import unittest
from unittest import mock

from host import kv_clock_sync as kcs
from host.kv_clock_sync import (PL_OK, PL_UNAVAILABLE, ClockSync, ClockSyncClient, ClockSyncError,
                                MockPlTimebase, PlTimebase, PlTimebaseUnavailable, SyncResponder,
                                exchange_sample)
from host.kv_link import KvLinkServer, NullCore
from host import fpga_clock
from host.fpga_clock import now_ns as raw_ns

MS = 1000000
US = 1000
S = 1000000000


class SimClocks:
    """Nano clock = true time T (ns). KV = T + offset + drift * (T - T0).
    PL count = floor((KV_latch - reset) * clk * (1 + pl_ppm) / 1e9)."""

    def __init__(self, offset_ns, drift_ppm, pl_ppm=30.0, clk_hz=100000000, t0=50000 * S):
        self.offset, self.rho, self.t0 = int(offset_ns), drift_ppm * 1e-6, t0
        self.pl_ppm, self.clk = pl_ppm, clk_hz
        self.pl_reset_kv = t0 - 1234 * S

    def kv(self, t):
        return int(t + self.offset + self.rho * (t - self.t0))

    def kv_inverse(self, kv):
        """True Nano time at which the KV clock reads ``kv`` (to < 1 ns)."""
        return (kv - self.offset + self.rho * self.t0) / (1.0 + self.rho)

    def pl_count_at_kv(self, kv):
        return int((kv - self.pl_reset_kv) * self.clk * (1 + self.pl_ppm * 1e-6) // S)

    def kv_at_pl_count(self, count):
        return self.pl_reset_kv + count * S / (self.clk * (1 + self.pl_ppm * 1e-6))


def run_sim(clocks, *, seed=1, duration_s=120.0, hz=4.0, fwd=(2 * MS, 1 * MS), back=(100 * US, 50 * US),
            loss=0.1, spike_p=0.05, spike=20 * MS, hold=(20 * US, 300 * US), mutate=None,
            pl=True, pl_mutate=None, window_s=60.0, extra_query_s=2.0):
    """Feeds ClockSync through the four timestamps the wire would carry; returns
    (sync, checks) where each check = (kind, error_ns, bound_ns) or ("no_estimate", ...)."""
    rng = random.Random(seed)
    sync = ClockSync(window_s=window_s)
    checks = []
    t = clocks.t0
    end = clocks.t0 + int(duration_s * S)
    prev_pair = None
    n = 0
    while t < end:
        t += int(S / hz * (0.8 + 0.4 * rng.random()))
        n += 1
        a = fwd[0] + rng.expovariate(1.0 / fwd[1]) + (spike if rng.random() < spike_p else 0)
        b = back[0] + rng.expovariate(1.0 / back[1])
        hd = hold[0] + rng.random() * (hold[1] - hold[0])
        t1 = t
        arrive = t1 + a
        t2 = clocks.kv(arrive)
        depart = arrive + hd
        # PL pair sampled between t2 and t3 on the KV, bracket 1..5 us, 3% preempted
        width = 1 * US + rng.random() * 4 * US + (200 * US if rng.random() < 0.03 else 0)
        ka = clocks.kv(arrive + 10 * US)
        kb = ka + int(width)
        latch = ka + rng.random() * (kb - ka)
        pair = (clocks.pl_count_at_kv(latch), (ka + kb) // 2, kb - ka)
        t3 = clocks.kv(depart)
        t4 = int(depart + b)
        lost = rng.random() < loss or rng.random() < loss
        if lost:
            continue
        stamps = [t1, t2, t3, t4]
        if mutate is not None:
            stamps = mutate(stamps)
        sync.add_exchange(*stamps)
        if pl:
            use = pair
            if pl_mutate == "stale_stamp" and prev_pair is not None:
                use = (pair[0], prev_pair[1], pair[2])
            sync.add_pl_pair(use[0], use[1], use[2], clocks.clk)
            prev_pair = pair
        if n < 12:
            continue
        # queries: now, a random instant inside the window, and 2 s into the future
        qs = [t4, t4 - int(rng.random() * window_s * 0.9 * S), t4 + int(extra_query_s * S)]
        for q in qs:
            try:
                m = sync.kv_map()
                y, bound = m.offset_at_nano(q)
                checks.append(("offset", y - (clocks.kv(q) - q), bound))
                kv_ev = clocks.kv(q)
                nano, bound2 = m.kv_to_nano(kv_ev)
                checks.append(("kv_to_nano", nano - clocks.kv_inverse(kv_ev), bound2))
                if pl:
                    cnt = clocks.pl_count_at_kv(kv_ev)
                    nano_pl, bound3 = sync.pl_to_nano(cnt)
                    truth = clocks.kv_inverse(clocks.kv_at_pl_count(cnt))
                    checks.append(("pl_to_nano", nano_pl - truth, bound3))
            except ClockSyncError as exc:
                checks.append(("no_estimate", str(exc), None))
    return sync, checks


def known_answer_ok(checks, min_checks=100):
    """The check the tests assert on (and the positive controls must break)."""
    bad = [c for c in checks if c[0] == "no_estimate" or abs(c[1]) > c[2]]
    good = [c for c in checks if c[0] != "no_estimate"]
    return len(good) >= min_checks and not bad, bad[:3], len(good)


class SimulatedKnownAnswerTests(unittest.TestCase):
    def assert_known(self, checks, max_median_bound_ns):
        ok, bad, n = known_answer_ok(checks)
        self.assertTrue(ok, "violations %s of %d checks" % (bad, n))
        bounds = sorted(c[2] for c in checks if c[0] == "offset")
        errs = [abs(c[1]) for c in checks if c[0] == "offset"]
        # not vacuous: the bound is of the size of the injected delays, and the
        # error actually uses a fair part of it (asymmetry is real)
        self.assertLess(bounds[len(bounds) // 2], max_median_bound_ns)
        return max(errs), bounds[len(bounds) // 2]

    def test_offset_3p2s_drift_50ppm_asymmetric_lossy(self):
        clocks = SimClocks(3200 * MS, 50.0)
        sync, checks = run_sim(clocks, seed=11)
        worst, med = self.assert_known(checks, 3 * MS)
        self.assertGreater(worst, 0.5 * MS)         # 2 ms vs 0.1 ms one-way: bias ~ -0.95 ms
        d = sync.kv_map().describe()
        self.assertLess(abs(d["drift_ppm"] - 50.0), 20.0, d)
        p = sync.pl_map().describe()
        # PL 30 ppm fast -> each tick is 1/(1+30e-6) of nominal in KV ns: about -30 ppm
        self.assertLess(abs(p["rate_dev_ppm"] - (1.0 / (1 + 30e-6) - 1.0) * 1e6), 2.0, p)
        self.assertGreater(sync.counts["exchanges_valid"], 300)

    def test_offset_negative_drift_negative(self):
        clocks = SimClocks(-700 * MS, -80.0, pl_ppm=-15.0)
        _, checks = run_sim(clocks, seed=5, fwd=(200 * US, 100 * US), back=(1 * MS, 2 * MS), loss=0.2)
        self.assert_known(checks, 3 * MS)

    def test_zero_offset_zero_drift_symmetric(self):
        clocks = SimClocks(0, 0.0, pl_ppm=0.0)
        sync, checks = run_sim(clocks, seed=3, fwd=(100 * US, 30 * US), back=(100 * US, 30 * US),
                               spike_p=0.0, loss=0.0)
        worst, med = self.assert_known(checks, 300 * US)
        d = sync.kv_map().describe()
        self.assertLess(abs(d["offset_ns_at_anchor"]), 50 * US)
        self.assertLess(abs(d["drift_ppm"]), 5.0)

    def test_extrapolated_bound_grows(self):
        clocks = SimClocks(3200 * MS, 50.0)
        sync, _ = run_sim(clocks, seed=11)
        m = sync.kv_map()
        x_a = m.fit["x_a"]
        _, b0 = m.offset_at_nano(x_a)
        _, b1 = m.offset_at_nano(x_a + 30 * S)
        self.assertAlmostEqual(b0, m.fit["h_a"])
        self.assertGreater(b1, b0)

    def test_clock_step_is_reported_not_absorbed(self):
        """A 10 ms step in the KV clock contradicts a constant-rate line: no bound is stated."""
        clocks = SimClocks(0, 0.0)
        sync = ClockSync(window_s=60.0)
        for i in range(40):
            t1 = clocks.t0 + i * S // 2
            step = 10 * MS if i >= 20 else 0
            sync.add_exchange(t1, t1 + 100 * US + step, t1 + 150 * US + step, t1 + 250 * US)
        self.assertFalse(sync.kv.fit()["consistent"])
        with self.assertRaises(ClockSyncError):
            sync.kv_map().offset_at_nano(clocks.t0)


def swap(i, j):
    def f(s):
        s = list(s)
        s[i], s[j] = s[j], s[i]
        return s
    return f


def flipped_sign_sample(t1, t2, t3, t4, rho_max=kcs.DEFAULT_RHO_MAX):
    s, why = exchange_sample(t1, t2, t3, t4, rho_max)
    if s is None:
        return s, why
    return (s[0], -s[1], s[2]), None


class PositiveControlTests(unittest.TestCase):
    """Each mutant must make the SAME known-answer check fail."""

    def sim(self, **kw):
        return run_sim(SimClocks(3200 * MS, 50.0), seed=11, **kw)[1]

    def test_unmutated_passes(self):
        self.assertTrue(known_answer_ok(self.sim())[0])

    def test_swap_t1_t2_fails(self):
        ok, bad, _ = known_answer_ok(self.sim(mutate=swap(0, 1)))
        self.assertFalse(ok)

    def test_swap_t2_t3_fails(self):
        # theta is unchanged by this swap; the causality check (t3 >= t2) rejects every
        # sample, so no estimate exists -> the check fails
        checks = self.sim(mutate=swap(1, 2))
        ok, bad, n = known_answer_ok(checks)
        self.assertFalse(ok)
        self.assertEqual(n, 0)

    def test_swap_t1_t4_fails(self):
        self.assertFalse(known_answer_ok(self.sim(mutate=swap(0, 3)))[0])

    def test_offset_sign_flip_fails(self):
        with mock.patch.object(kcs, "exchange_sample", flipped_sign_sample):
            checks = self.sim()
        ok, bad, n = known_answer_ok(checks)
        self.assertFalse(ok)
        self.assertGreater(n, 0)

    def test_pl_stale_kv_stamp_fails(self):
        self.assertFalse(known_answer_ok(self.sim(pl_mutate="stale_stamp"))[0])


class PlTimebaseGateTests(unittest.TestCase):
    def make_sysfs(self, root, name="m7_permit", addr=0xA0060000, size=0x10000):
        d = Path(root) / "sys" / "uio3"
        (d / "maps" / "map0").mkdir(parents=True)
        (d / "name").write_text(name + "\n")
        (d / "maps" / "map0" / "addr").write_text("0x%x\n" % addr)
        (d / "maps" / "map0" / "size").write_text("0x%x\n" % size)
        return str(Path(root) / "sys")

    def make_dev(self, root, words):
        dev = Path(root) / "dev"
        dev.mkdir()
        buf = bytearray(0x10000)
        for off, val in words.items():
            struct.pack_into("<I", buf, off, val)
        (dev / "uio3").write_bytes(bytes(buf))
        return str(dev)

    def test_no_uio_node_degrades_without_touching_any_device(self):
        with tempfile.TemporaryDirectory() as tmp:
            sysfs = os.path.join(tmp, "nosys")
            pl = PlTimebase.open("permit", sysfs_root=sysfs, dev_root=os.path.join(tmp, "absent"))
            self.assertFalse(pl.available)
            self.assertIn("no UIO node named 'm7_permit'", pl.reason)
            with self.assertRaises(PlTimebaseUnavailable):
                pl.sample()

    def test_wrong_magic_degrades(self):
        with tempfile.TemporaryDirectory() as tmp:
            sysfs = self.make_sysfs(tmp)
            dev = self.make_dev(tmp, {0x00: 0x4D37454E})            # encoder magic at permit node
            pl = PlTimebase.open("permit", sysfs_root=sysfs, dev_root=dev)
            self.assertFalse(pl.available)
            self.assertIn("MAGIC", pl.reason)

    def test_right_magic_reads_lo_then_hi(self):
        with tempfile.TemporaryDirectory() as tmp:
            sysfs = self.make_sysfs(tmp)
            dev = self.make_dev(tmp, {0x00: 0x4D37504D, 0x4C: 0x89ABCDEF, 0x50: 0x12})
            pl = PlTimebase.open("permit", sysfs_root=sysfs, dev_root=dev, clk_hz=99999000)
            self.assertTrue(pl.available, pl.reason)
            count, mid, width = pl.sample()
            self.assertEqual(count, (0x12 << 32) | 0x89ABCDEF)
            self.assertGreaterEqual(width, 0)
            self.assertEqual(pl.describe()["clk_hz"], 99999000)
            # a frozen counter is caught as non-monotonic rather than paired twice
            with self.assertRaises(ClockSyncError):
                pl.sample_best(2)
            pl.regs.close()

    def test_lo_read_is_bracketed_and_hi_after(self):
        order = []
        clock_v = [1000]

        def clock():
            order.append("clock")
            clock_v[0] += 7
            return clock_v[0]

        class Regs:
            def rd(self, off):
                order.append(off)
                return {0x4C: 5, 0x50: 1}[off]
        pl = PlTimebase(Regs(), block="permit", clock=clock)
        count, mid, width = pl.sample()
        self.assertEqual(order, ["clock", 0x4C, "clock", 0x50])
        self.assertEqual((count, mid, width), ((1 << 32) | 5, (1007 + 1014) // 2, 7))


def start_server(responder):
    srv = KvLinkServer(NullCore(), "null", bind=("127.0.0.1", 0), sync_responder=responder)
    return srv.start()


class LoopbackTests(unittest.TestCase):
    """Real UDP over 127.0.0.1; both ends read the same CLOCK_MONOTONIC_RAW: truth = 0."""

    def test_zero_known_answer_and_pl_map(self):
        mock_pl = MockPlTimebase(ppm=30.0, reset_kv_ns=raw_ns() - 5 * S)
        srv = start_server(SyncResponder(pl=mock_pl))
        client = ClockSyncClient(("127.0.0.1", srv.port), sync=ClockSync(window_s=30.0))
        try:
            checks = []
            for i in range(150):
                ack = client.exchange()
                if ack is None or i < 10:
                    continue
                t = (ack["t1"] + ack["t4"]) // 2
                y, b = client.sync.kv_map().offset_at_nano(t)
                checks.append(("offset", y, b))
                ev = raw_ns()
                cnt = mock_pl.true_count(ev)
                nano, b2 = client.sync.pl_to_nano(cnt)
                checks.append(("pl_to_nano", nano - ev, b2 + 10))   # count floor: <= 1 tick
                time.sleep(0.002)
            ok, bad, n = known_answer_ok(checks, min_checks=150)
            self.assertTrue(ok, (bad, n))
            rep = client.report()
            self.assertEqual(rep["pl_state"]["status"], "ok")
            self.assertLess(abs(rep["pl_map"]["rate_dev_ppm"] + 30.0), 30.0, rep["pl_map"])
            self.assertGreater(srv.sync_responder.counts["sync_answered"], 100)
            self.assertEqual(srv.counts["slot_superseded"], 0)
        finally:
            client.close()
            srv.stop()

    def test_degraded_pl_is_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            pl = PlTimebase.open("permit", sysfs_root=os.path.join(tmp, "nosys"))
        srv = start_server(SyncResponder(pl=pl))
        client = ClockSyncClient(("127.0.0.1", srv.port))
        try:
            for _ in range(20):
                ack = client.exchange()
            self.assertEqual(ack["pl_status"], PL_UNAVAILABLE)
            self.assertIn("m7_permit", ack["pl_reason"])
            client.sync.kv_to_nano(raw_ns())          # the KV map still works
            with self.assertRaises(PlTimebaseUnavailable) as cm:
                client.sync.pl_to_nano(12345)
            self.assertIn("m7_permit", str(cm.exception))
            rep = client.report()
            self.assertIn("degraded", rep["pl_map"])
            self.assertEqual(rep["counts"]["pl_status:unavailable"], rep["counts"]["exchanges_valid"])
        finally:
            client.close()
            srv.stop()

    def test_sync_answered_while_worker_busy(self):
        """SYNC is served on the reader thread: a busy core does not delay it."""
        busy = threading.Event()

        class SlowCore(NullCore):
            def __call__(self, *a, **k):
                busy.set()
                time.sleep(0.5)
                NullCore.__call__(self, *a, **k)
        from host.kv_link import KvLinkClient, REQ, split
        srv = KvLinkServer(SlowCore(), "slow", bind=("127.0.0.1", 0)).start()
        client = ClockSyncClient(("127.0.0.1", srv.port))
        try:
            # a minimal valid request to occupy the worker
            from host.kv_link import encode_request, codes_to_m, crc32
            import numpy as np
            H, W = 64, 96
            codes = np.full((H, W), 700, dtype=np.uint16)
            req = encode_request(frame_id=1, capture_ns=1, pose_ns=1,
                                 identity={"stream_instance": "a", "clock_id": "b",
                                           "calibration_id": "c", "pose_epoch": "d"},
                                 width=W, height=H, nx=4, ny=4, nz=4, unit_f32=np.float32(0.001),
                                 voxel=0.05, origin=np.zeros(3), inverse_k=np.eye(3),
                                 world_camera=np.zeros((3, 4)), codes=codes,
                                 depth_crc=crc32(codes_to_m(codes, np.float32(0.001))))
            fc = KvLinkClient(("127.0.0.1", srv.port))
            for d in split(REQ, fc.session, 1, req, 60000):
                fc.sock.sendto(d, fc.server)
            self.assertTrue(busy.wait(2))
            ack = client.exchange()
            self.assertIsNotNone(ack)
            self.assertLess(ack["t4"] - ack["t1"], 100 * MS)
            fc.close()
        finally:
            client.close()
            srv.stop()


class ResponderTests(unittest.TestCase):
    def test_t3_taken_after_pl_sample(self):
        seq = []

        def clock():
            seq.append("clock")
            return 10 ** 12 + len(seq)

        class PL(MockPlTimebase):
            def sample_best(self, n=3):
                seq.append("pl")
                return 1, 2, 3
        r = SyncResponder(pl=PL(clock=clock), clock=clock)
        body = r.answer(kcs.SYNC_BODY.pack(7, 99), t2=5)
        ack = kcs.decode_sync_ack(body)
        self.assertEqual(seq, ["pl", "clock"])
        self.assertEqual((ack["seq"], ack["t1"], ack["t2"], ack["pl_status"]), (7, 99, 5, PL_OK))
        self.assertGreater(ack["t3"], ack["t2"])



# ----------------------------------------------------------------- timebase (spec section 9)
class SlewClocks:
    """True time T (ns). RAW: Nano = T, KV = T + offset + rho*(T - T0) (crystal only).
    MONOTONIC = RAW + NTP slew: each board's MONOTONIC runs at RAW rate + extra ppm
    inside its slew windows [(start_s, end_s, ppm)], as adjtimex frequency/offset
    correction does. ``arm`` picks which pair of clocks the stamps are read from."""

    def __init__(self, arm, *, offset_ns=-61146 * S, rho_ppm=40.0, t0=60000 * S,
                 nano_slews=((40.0, 52.0, +500.0),), kv_slews=((75.0, 87.0, -500.0),)):
        assert arm in ("raw", "mono")
        self.arm, self.offset, self.rho, self.t0 = arm, int(offset_ns), rho_ppm * 1e-6, t0
        self.nano_slews, self.kv_slews = nano_slews, kv_slews

    def _slew(self, t, slews):
        el = (t - self.t0) / S
        return sum(ppm * 1e-6 * S * max(0.0, min(el, b) - a) for a, b, ppm in slews if el > a)

    def nano_raw(self, t):
        return int(t)

    def kv_raw(self, t):
        return int(t + self.offset + self.rho * (t - self.t0))

    def nano(self, t):
        return self.nano_raw(t) + (int(self._slew(t, self.nano_slews)) if self.arm == "mono" else 0)

    def kv(self, t):
        return self.kv_raw(t) + (int(self._slew(t, self.kv_slews)) if self.arm == "mono" else 0)


def run_slew(arm, *, seed=21, duration_s=150.0, hz=10.0, window_s=60.0, settle=20):
    """Board-like exchanges (10 Hz, ~0.4 ms each way, jitter and rare 3 ms spikes).
    Queries the running estimate at each exchange's t4, the way clock_sync_probe does;
    truth = KV(T4) - Nano(T4) on the clocks the stamps came from."""
    clocks = SlewClocks(arm)
    rng = random.Random(seed)
    sync = ClockSync(window_s=window_s)
    checks = []
    t, end, i = clocks.t0, clocks.t0 + int(duration_s * S), 0
    while t < end:
        t += int(S / hz)
        i += 1
        up = 380 * US + rng.expovariate(1.0 / (40 * US)) + (3 * MS if rng.random() < 0.01 else 0)
        down = 380 * US + rng.expovariate(1.0 / (40 * US))
        hold = 30 * US + rng.random() * 50 * US
        T1, T2 = t, t + up
        T3 = T2 + hold
        T4 = T3 + down
        sync.add_exchange(clocks.nano(T1), clocks.kv(T2), clocks.kv(T3), clocks.nano(T4))
        if i < settle:
            continue
        q = clocks.nano(T4)
        try:
            y, b = sync.kv_map().offset_at_nano(q)
            checks.append(("offset", y - (clocks.kv(T4) - q), b))
        except ClockSyncError as exc:
            checks.append(("no_estimate", str(exc), None))
    return clocks, checks


class TimebaseSlewTests(unittest.TestCase):
    def test_fake_slew_is_real(self):
        """The fake must actually slew MONOTONIC (else the RAW pass proves nothing)."""
        c = SlewClocks("mono")
        t = c.t0 + 100 * S
        self.assertEqual(c.nano(t) - c.nano_raw(t), 6 * MS)      # +500 ppm x 12 s
        self.assertEqual(c.kv(t) - c.kv_raw(t), -6 * MS)         # -500 ppm x 12 s

    def test_raw_timebase_holds_bound_through_ntp_slew(self):
        _, checks = run_slew("raw")
        ok, bad, n = known_answer_ok(checks, min_checks=1000)
        self.assertTrue(ok, "violations %s of %d checks" % (bad, n))

    def test_positive_control_slewed_monotonic_breaks_bound(self):
        """Same exchanges and the same check, stamps read from the slewed clock: must fail,
        with real out-of-bound estimates (the board's failure), not only refusals."""
        _, checks = run_slew("mono")
        ok, _, _ = known_answer_ok(checks, min_checks=1000)
        self.assertFalse(ok)
        over = [c for c in checks if c[0] == "offset" and abs(c[1]) > c[2]]
        self.assertGreater(len(over), 0)


class TimebaseWiringTests(unittest.TestCase):
    def test_fpga_clock_reads_monotonic_raw(self):
        a = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
        b = raw_ns()
        c = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
        self.assertTrue(a <= b <= c)
        self.assertEqual(fpga_clock.CLOCK_NAME, "CLOCK_MONOTONIC_RAW")

    def test_defaults_use_raw(self):
        self.assertIs(SyncResponder().clock, raw_ns)
        self.assertIs(PlTimebase().clock, raw_ns)
        self.assertIs(MockPlTimebase().clock, raw_ns)
        with tempfile.TemporaryDirectory() as tmp:
            self.assertIs(PlTimebase.open("permit", sysfs_root=os.path.join(tmp, "x")).clock, raw_ns)
        c = ClockSyncClient(("127.0.0.1", 9))
        try:
            self.assertIs(c.clock, raw_ns)
        finally:
            c.close()

    def test_server_t2_comes_from_the_responder_clock(self):
        """t2 (stamped in KvLinkServer._reader) and t3 must share one clock."""
        fake = iter(range(7 * 10 ** 15, 8 * 10 ** 15, 1000))
        srv = start_server(SyncResponder(clock=lambda: next(fake)))
        client = ClockSyncClient(("127.0.0.1", srv.port))
        try:
            ack = None
            for _ in range(5):
                ack = client.exchange() or ack
            self.assertIsNotNone(ack)
            self.assertTrue(7 * 10 ** 15 <= ack["t2"] < ack["t3"] < 8 * 10 ** 15, ack)
        finally:
            client.close()
            srv.stop()

if __name__ == "__main__":
    unittest.main()
