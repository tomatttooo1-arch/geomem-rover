"""kv_server.py --permit-model realtime (host only, kv-permit 2026-10-14).

The permit block on the host is host.kv_regs.RealtimePermitModel behind PlRegBlock("permit");
kv_server wraps the geometry core so each returned call raises the completion edge.

  M1 heartbeat pin (STATUS b1): PermitPlModel against a cycle-by-cycle transcription of the RTL
     (hw/m7/pl/rtl/m7_permit_axil.v: hb_cnt / permit_hb_o / armed / age / FORCE_STOP) on a random
     event schedule -- permit bit equal on every clock, pin bit equal on every clock except the
     revoking one (documented: the model's pin drops on that clock, the RTL's one clock later).
     Positive control: a mutant with pin = permit fails the same comparison.
  M2 square wave: while permitted, full HB_HALF pulses on the 2*HB_HALF grid from reset, none
     started mid-period after a grant; low after a revoke; HB_HALF_CYCLES (0x48) readable.
  M3 off = unchanged: attach_permit_model returns the very same core object and leaves the
     register dict untouched; the CLI without the option keeps the permit block unavailable,
     no feed, no new info / summary keys (subprocess, --core null).
  M4 wiring: realtime replaces the permit block (refused over a real block and with
     --device devmem); the wrapper raises the edge only after a core call that returned.
  M5 rules end to end over UDP (KvLinkServer + attach_permit_model, host clock replaced by a
     controlled clock so 200 ms is exact): live TELEM (permit, accepted, last_frame_id rising);
     no frame -> TIMEOUT (cause 1) at exactly last accepted + TIMEOUT + 1 cycle; e-stop ->
     revoked at once (cause 2), frames during it not granted, release alone does not grant,
     the next new frame does; replayed frame id (fresh link, same Nano session) -> revoked at
     once (cause 3). Positive control: the kv-counter feed renews on the same replay.
  M6 the CLI end to end (subprocess, --core null --permit-model realtime, real clock):
     TELEM live, then TIMEOUT with REVOKE_TS - DONE_TS = TIMEOUT + 1 cycle; summary has the
     model's end snapshot.
Run: PYTHONPATH=. python3 -m unittest tests.test_kv_permit_model  (also /usr/bin/python3.8).
"""
import argparse
import json
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

from host import kv_regs
from host.kv_link import (KvEstopClient, KvLinkClient, KvLinkServer, KvTelemetryClient,
                          LinkTimeout, NullCore, blocks_mask, crc32, encode_request)
from host.kv_regs import (KvCounterFrameFeed, NanoFrameFeed, PermitPlModel, PlRegBlock,
                          RealtimePermitModel)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "kv_link"))
import kv_server  # noqa: E402


# ------------------------------------------------------------------ RTL transcription (M1)
class RtlPermit:
    """Per-clock transcription of m7_permit_axil.v (permit / heartbeat part). ``clock(ev)``
    applies the registers' update of one clock with the inputs of that clock; reads between
    clocks return the register state."""

    def __init__(self, timeout, half):
        self.T, self.HALF, self.PERIOD = timeout, half, 2 * half
        self.armed, self.age, self.force = False, timeout, False
        self.last_id, self.last_valid = 0, False
        self.hb_cnt, self.hb_o = 0, False

    @property
    def permit(self):
        return self.armed and self.age < self.T and not self.force

    def clock(self, ev=None):
        permit = self.permit
        hb_o = permit and self.hb_cnt < self.HALF and (self.hb_o or self.hb_cnt == 0)
        hb_cnt = 0 if self.hb_cnt == self.PERIOD - 1 else self.hb_cnt + 1
        armed, age, force = self.armed, self.age, self.force
        if self.armed and self.age < self.T:
            age = self.age + 1
        if ev is not None and ev[0] == "done":
            fid = ev[1]
            newer = (not self.last_valid) or fid > self.last_id
            if newer:
                self.last_id, self.last_valid = fid, True
            if newer and not self.force:
                armed, age = True, 0
            elif not newer:
                armed = False
        if ev is not None and ev[0] == "ctrl":
            force = bool(ev[1])
            if force:
                armed = False
        self.armed, self.age, self.force = armed, age, force
        self.hb_cnt, self.hb_o = hb_cnt, hb_o


class PinIsPermitMutant(PermitPlModel):
    """Positive control for M1: the pin simply follows permit (no divider)."""

    @property
    def pin(self):
        return self.permit


def drive(model, rtl, schedule, cycles):
    """Same schedule into both; returns per-clock rows (n, rtl_permit_before, rtl_permit, rtl_pin,
    model_permit, model_pin) after each clock."""
    rows = []
    for n in range(cycles):
        ev = schedule.get(n)
        before = rtl.permit
        rtl.clock(ev)
        if ev is not None and ev[0] == "done":
            model.next_frame_id = ev[1]            # what the feed writes before the core start
            model.complete()                       # the edge: one clock
        elif ev is not None and ev[0] == "ctrl":
            model.wr(kv_regs.PERMIT_CTRL, ev[1])
            model.advance(1)
        else:
            model.advance(1)
        st = model.rd(kv_regs.PERMIT_STATUS)
        rows.append((n, before, rtl.permit, rtl.hb_o, bool(st & 1), bool(st >> 1 & 1)))
    return rows


def random_schedule(seed, cycles, T):
    rng = random.Random(seed)
    sched, fid, n = {}, 0, 0
    while n < cycles:
        n += rng.choice((1, 2, 3, 5, 8, T // 2, T - 1, T, T + 3, 2 * T))
        r = rng.random()
        if r < 0.70:
            fid += rng.choice((1, 1, 1, 2, 5))
            sched[n] = ("done", fid)
        elif r < 0.80:
            sched[n] = ("done", max(0, fid - rng.choice((0, 0, 1, 3))))   # duplicate / regress
        else:
            sched[n] = ("ctrl", rng.random() < 0.5)
    return sched


class HeartbeatPinTests(unittest.TestCase):
    T, HALF, N = 40, 3, 30000

    def _compare(self, cls, seed):
        model = cls(timeout=self.T, tb_start=0, hb_half=self.HALF)
        rows = drive(model, RtlPermit(self.T, self.HALF), random_schedule(seed, self.N, self.T),
                     self.N)
        permit_bad = [r for r in rows if r[2] != r[4]]
        compared = [r for r in rows if not (r[1] and not r[2])]      # skip the revoking clock
        pin_bad = [r for r in compared if r[3] != r[5]]
        return rows, permit_bad, compared, pin_bad

    def test_m1_pin_matches_rtl_transcription(self):
        for seed in (1, 2, 3):
            rows, permit_bad, compared, pin_bad = self._compare(PermitPlModel, seed)
            self.assertEqual(permit_bad, [], "seed %d" % seed)
            self.assertEqual(pin_bad[:5], [], "seed %d" % seed)
            self.assertGreater(len(compared), self.N - 2000)
            highs = sum(1 for r in rows if r[3])
            self.assertGreater(highs, 1000, "schedule must exercise the pin")

    def test_m1_positive_control_pin_equals_permit_fails(self):
        rows, permit_bad, compared, pin_bad = self._compare(PinIsPermitMutant, 1)
        self.assertEqual(permit_bad, [])
        self.assertGreater(len(pin_bad), 1000)

    def test_m2_square_wave_grid_and_revoke(self):
        half, T = 5, 1000
        m = PermitPlModel(timeout=T, tb_start=0, hb_half=half)
        self.assertEqual(m.rd(0x48), half)
        m.advance(13)                       # grant mid-period (period = 10)
        m.next_frame_id = 1
        m.complete()                        # tb 14, permit from clock 14 on
        trace = []
        for _ in range(200):
            trace.append((m.tb, m.pin))
            m.advance(1)
        rises = [trace[i + 1][0] for i in range(len(trace) - 1)
                 if not trace[i][1] and trace[i + 1][1]]
        # first rise: the pin register goes high on the clock after hb_cnt == 0 at n = 20
        self.assertEqual(rises[0], 21)
        self.assertTrue(all((r - 1) % (2 * half) == 0 for r in rises))
        self.assertTrue(all(not p for tb, p in trace if tb <= 20))
        # every pulse exactly half long
        run, runs = 0, []
        for _, p in trace:
            if p:
                run += 1
            elif run:
                runs.append(run)
                run = 0
        self.assertTrue(runs and all(r == half for r in runs), runs)
        m.wr(kv_regs.PERMIT_CTRL, 1)       # revoke: low from then on
        for _ in range(50):
            self.assertFalse(m.pin)
            m.advance(1)
        self.assertEqual(m.rd(kv_regs.PERMIT_STATUS) & 0b11, 0)


# ------------------------------------------------------------------ helpers (M3-M6)
def ns(permit_model="realtime", device=None, pl_clk_hz=None):
    return argparse.Namespace(permit_model=permit_model, device=device, pl_clk_hz=pl_clk_hz)


def degraded_regs():
    return {n: PlRegBlock(n, None, reason="no UIO node named %r (test)" % n)
            for n in ("sysinfo", "permit", "encoder")}


H, W = 24, 32
K_INV = np.linalg.inv(np.array([[30.0, 0, 15.5], [0, 30.0, 11.5], [0, 0, 1]])).astype(np.float32)


def request(stream, fid):
    codes = np.full((H, W), 1000, dtype=np.uint16)
    ident = {"stream_instance": stream, "clock_id": "c", "calibration_id": "cal",
             "pose_epoch": "e1"}
    body = encode_request(frame_id=fid, capture_ns=fid * 1000, pose_ns=fid * 1000,
                          identity=ident, width=W, height=H, nx=8, ny=8, nz=8, unit_f32=0.001,
                          voxel=0.1, origin=np.zeros(3), inverse_k=K_INV,
                          world_camera=np.eye(4, dtype=np.float32)[:3],
                          codes=codes, depth_crc=crc32(codes.astype(np.float32) * np.float32(0.001)))
    return body, {"frame_id": fid, "capture_ns": fid * 1000, "pose_ns": fid * 1000,
                  "identity": ident}


def send(port, stream, fid, client=None, timeout=1.0):
    c = client or KvLinkClient(("127.0.0.1", port), timeout_s=timeout)
    try:
        body, expect = request(stream, fid)
        try:
            c.call(body, expect)
            return True
        except LinkTimeout:
            return False
    finally:
        if client is None:
            c.close()


def telem(port):
    tel = KvTelemetryClient(("127.0.0.1", port), timeout_s=0.5, hello_timeout_s=1.0)
    try:
        return tel.poll(blocks_mask(("permit", "board"))).blocks["permit"]
    finally:
        tel.close()


class OffAndWiringTests(unittest.TestCase):
    def test_m3_off_returns_same_objects(self):
        regs = degraded_regs()
        before = dict(regs)
        core = NullCore()
        out, model = kv_server.attach_permit_model(ns("off"), regs, core)
        self.assertIs(out, core)
        self.assertIsNone(model)
        self.assertEqual(set(regs), set(before))
        for k in regs:
            self.assertIs(regs[k], before[k])

    def test_m4_realtime_replaces_block_and_wraps(self):
        regs = degraded_regs()
        calls = []

        def core(*args):
            calls.append(args)
            return 7
        out, model = kv_server.attach_permit_model(ns(), regs, core)
        self.assertIsInstance(model, RealtimePermitModel)
        self.assertTrue(regs["permit"].available)
        self.assertIs(regs["permit"].window, model)
        self.assertEqual((model.timeout, model.hb_half), (100000000, 500000))   # 1000 ms (v3fc2)
        self.assertEqual((model.window_min, model.window_max), (10000000, 500000000))
        self.assertEqual(regs["permit"].detail["replaced"], "no UIO node named 'permit' (test)")
        self.assertEqual(out(1, 2), 7)
        self.assertEqual(calls, [(1, 2)])
        self.assertEqual(len(model.events), 1)          # one completion edge

        def failing(*args):
            raise RuntimeError("device error")
        out2, model2 = kv_server.attach_permit_model(ns(), degraded_regs(), failing)
        with self.assertRaises(RuntimeError):
            out2()
        self.assertEqual(model2.events, [])             # a failed core raises no edge

    def test_m4_clock_option_scales_timeout(self):
        _, model = kv_server.attach_permit_model(ns(pl_clk_hz=250000000), degraded_regs(), NullCore())
        self.assertEqual((model.timeout, model.hb_half), (250000000, 1250000))

    def test_m4_refused_over_real_block(self):
        regs = degraded_regs()
        regs["permit"] = kv_regs.mock_permit()[0]       # an available (MAGIC-ok) block
        with self.assertRaises(SystemExit):
            kv_server.attach_permit_model(ns(), regs, NullCore())

    def test_m4_cli_refuses_devmem(self):
        r = subprocess.run([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--core", "pl",
                            "--device", "devmem", "--wait", "irq", "--permit-model", "realtime"],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                           timeout=60, env=dict(os.environ, PYTHONPATH=str(ROOT)))
        self.assertEqual(r.returncode, 2)
        self.assertIn("--permit-model is host only", r.stderr)


class FakeClock:
    def __init__(self):
        self.t = 10 ** 12

    def __call__(self):
        return self.t

    def ms(self, v):
        self.t += int(v * 1e6)


class RulesEndToEndTests(unittest.TestCase):
    """KvLinkServer with the kv_server wiring (attach_permit_model) and a controlled clock."""

    def _server(self, feed_cls=NanoFrameFeed):
        regs = degraded_regs()
        core, model = kv_server.attach_permit_model(ns(), regs, NullCore())
        clock = FakeClock()
        model._mono = clock
        model._t_ns = clock()
        srv = KvLinkServer(core, "null", bind=("127.0.0.1", 0), regs=regs,
                           permit_feed=feed_cls(regs["permit"]))
        srv.start()
        self.addCleanup(srv.stop)
        return srv, model, clock

    def test_m5_live_and_timeout(self):
        srv, model, clock = self._server()
        c = KvLinkClient(("127.0.0.1", srv.port), timeout_s=1.0)
        self.addCleanup(c.close)
        seen = []
        for f in range(1, 6):
            self.assertTrue(send(srv.port, "geoprod-1-a", f, c))
            clock.ms(100)                      # 10 Hz
            p = telem(srv.port)
            seen.append((p["status_bits"]["permit"], p["frames_accepted"], p["last_frame_id"],
                         p["age_cycles"]))
        self.assertTrue(all(s[0] for s in seen))
        self.assertEqual([s[1] for s in seen], [1, 2, 3, 4, 5])
        ids = [s[2] for s in seen]
        self.assertEqual(ids, sorted(set(ids)))
        self.assertEqual(len({s[3] for s in seen}), 1)   # 100 ms each (fake clock) ...
        clock.ms(899.99)                                 # 999.99 ms since the last edge (v3fc2 1 s)
        self.assertTrue(telem(srv.port)["status_bits"]["permit"])
        clock.ms(0.02)
        p = telem(srv.port)
        self.assertFalse(p["status_bits"]["permit"])
        self.assertEqual(p["last_cause"], PermitPlModel.C_TIMEOUT)
        self.assertEqual(p["last_revoke_ts"] - p["last_done_ts"], model.timeout + 1)
        self.assertEqual(p["revoke_count"], 1)

    def test_m5_estop_immediate_and_needs_new_frame(self):
        srv, model, clock = self._server()
        c = KvLinkClient(("127.0.0.1", srv.port), timeout_s=1.0)
        self.addCleanup(c.close)
        self.assertTrue(send(srv.port, "geoprod-2-b", 1, c))
        clock.ms(10)
        e = KvEstopClient(("127.0.0.1", srv.port))
        self.addCleanup(e.close)
        model.rd(kv_regs.PERMIT_STATUS)                  # bring the PL clock to "now"
        tb_assert = model.tb
        ack = e.send("assert", 1, timeout_s=1.0)
        self.assertTrue(ack.latched and ack.force_stop_written)
        p = telem(srv.port)
        self.assertFalse(p["status_bits"]["permit"])
        self.assertTrue(p["status_bits"]["estop"])
        self.assertEqual(p["last_cause"], PermitPlModel.C_FORCE)
        self.assertEqual(p["last_revoke_ts"], tb_assert)  # at the CTRL write, no clock passed
        for f in (2, 3):                                  # frames while latched: not granted
            self.assertTrue(send(srv.port, "geoprod-2-b", f, c))
            clock.ms(20)
            self.assertFalse(telem(srv.port)["status_bits"]["permit"])
        acc = model.accepted
        self.assertEqual(acc, 1)
        e.send("release", 2, epoch=ack.epoch, timeout_s=1.0)
        clock.ms(50)
        p = telem(srv.port)
        self.assertFalse(p["status_bits"]["permit"])      # release alone never grants
        self.assertFalse(p["status_bits"]["estop"])
        self.assertTrue(send(srv.port, "geoprod-2-b", 4, c))
        p = telem(srv.port)
        self.assertTrue(p["status_bits"]["permit"])
        self.assertEqual(p["frames_accepted"], acc + 1)

    def _replay(self, feed_cls):
        srv, model, clock = self._server(feed_cls)
        c = KvLinkClient(("127.0.0.1", srv.port), timeout_s=1.0)
        self.addCleanup(c.close)
        for f in (1, 2, 3):
            self.assertTrue(send(srv.port, "geoprod-3-c", f, c))
            clock.ms(30)
        self.assertTrue(telem(srv.port)["status_bits"]["permit"])
        tb0 = model.tb
        self.assertTrue(send(srv.port, "geoprod-3-c", 3))   # same session, same id, fresh link
        return telem(srv.port), model, tb0

    def test_m5_replay_revokes_at_once(self):
        p, model, tb0 = self._replay(NanoFrameFeed)
        self.assertFalse(p["status_bits"]["permit"])
        self.assertEqual(p["last_cause"], PermitPlModel.C_REPLAY)
        self.assertEqual(p["frames_rejected"], 1)
        self.assertEqual(p["last_revoke_ts"], tb0 + 1)     # the reject edge, no wait
        self.assertLess(p["last_revoke_ts"] - p["last_done_ts"], model.timeout)

    def test_m5_positive_control_kv_counter_renews_on_replay(self):
        p, model, tb0 = self._replay(KvCounterFrameFeed)
        self.assertTrue(p["status_bits"]["permit"])
        self.assertEqual(p["frames_rejected"], 0)
        self.assertEqual(p["frames_accepted"], 4)


class CliEndToEndTests(unittest.TestCase):
    def _run(self, extra):
        tmp = tempfile.mkdtemp(prefix="kvpm_")
        p = subprocess.Popen([sys.executable, str(ROOT / "tools/kv_link/kv_server.py"), "--core", "null",
                              "--bind", "127.0.0.1", "--port", "0", "--out", tmp,
                              "--uio-sysfs", os.path.join(tmp, "no_uio")] + extra,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                             env=dict(os.environ, PYTHONPATH=str(ROOT)))
        line = p.stdout.readline()
        port = json.loads(line)["port"]
        return p, port, tmp

    def _stop(self, p, tmp):
        p.terminate()
        p.communicate(timeout=30)
        with open(os.path.join(tmp, "summary.json")) as fp:
            return json.load(fp)

    def _info(self, port):
        c = KvLinkClient(("127.0.0.1", port), timeout_s=2.0)
        try:
            return c.stats()["info"]
        finally:
            c.close()

    def test_m3_cli_off_unchanged(self):
        p, port, tmp = self._run([])
        try:
            self.assertTrue(send(port, "geoprod-9-z", 1))
            blk = telem(port)
            info = self._info(port)
        finally:
            summary = self._stop(p, tmp)
        self.assertFalse(blk["available"])
        self.assertIn("no UIO node named 'm7_permit'", blk["reason"])
        self.assertNotIn("permit_model", info)
        self.assertIsNone(info["permit_feed"])
        self.assertFalse(info["registers"]["permit"]["available"])
        self.assertNotIn("permit_model_end", summary)
        self.assertIsNone(summary["permit_feed"])

    def test_m6_cli_realtime_live_then_timeout(self):
        # v3fc2: the window is written by --permit-window-ms (200 ms here, so the test stays short)
        p, port, tmp = self._run(["--permit-model", "realtime", "--permit-window-ms", "200"])
        try:
            c = KvLinkClient(("127.0.0.1", port), timeout_s=2.0)
            try:
                for f in (1, 2, 3):
                    self.assertTrue(send(port, "geoprod-8-y", f, c))
            finally:
                c.close()
            first = telem(port)
            time.sleep(0.5)
            later = telem(port)
            info = self._info(port)
        finally:
            summary = self._stop(p, tmp)
        self.assertTrue(first["available"])
        self.assertEqual(first["frames_accepted"], 3)
        self.assertEqual(first["feed"]["session"], "geoprod-8-y")
        self.assertEqual(first["last_frame_id"] - first["feed"]["pl_minus_nano"], 3)
        self.assertFalse(later["status_bits"]["permit"])
        self.assertEqual(later["last_cause"], PermitPlModel.C_TIMEOUT)
        self.assertEqual(later["last_revoke_ts"] - later["last_done_ts"], 20000001)
        self.assertEqual(info["permit_model"], "realtime")
        self.assertEqual(info["permit_window"]["readback_cycles"], 20000000)
        self.assertEqual(info["permit_window"]["before_cycles"], 100000000)
        self.assertEqual(info["permit_feed"]["mode"], "nano_frame_id")
        self.assertEqual(info["registers"]["permit"]["detail"]["model"], "RealtimePermitModel")
        self.assertEqual(summary["permit_model_end"]["frames_accepted"], 3)


class PermitWindowTests(unittest.TestCase):
    """v3fc2 (2026-10-26): runtime window register 0x44 -- model rules (as the RTL bench S10) and
    kv_server --permit-window-ms."""

    def test_w1_model_clamp_disarm_no_regrant(self):
        m = PermitPlModel(timeout=1000, tb_start=0, hb_half=10)
        self.assertEqual((m.rd(0x44), m.rd(0x58), m.rd(0x5C)), (1000, 100, 5000))
        for v, want in ((1, 100), (0, 100), (99, 100), (0xFFFFFFFF, 5000), (5001, 5000), (300, 300)):
            m.wr(0x44, v)
            self.assertEqual(m.rd(0x44), want)
        m.next_frame_id = 1
        m.complete()
        self.assertTrue(m.permit)
        m.advance(299)
        self.assertTrue(m.permit)
        m.advance(1)
        self.assertFalse(m.permit)
        self.assertEqual(m.last_cause, m.C_TIMEOUT)
        m.wr(0x44, 5000)                                # widening an expired window: no re-grant
        self.assertFalse(m.permit)
        m.next_frame_id = 2
        m.complete()
        self.assertTrue(m.permit)
        m.wr(0x44, 5000)                                # any write while permitted revokes (cause 5)
        self.assertFalse(m.permit)
        self.assertEqual(m.last_cause, m.C_WINDOW)
        m.next_frame_id = 3
        m.complete()
        m.wr(kv_regs.PERMIT_CTRL, 1)                    # FORCE_STOP still immediate
        self.assertFalse(m.permit)
        self.assertEqual(m.last_cause, m.C_FORCE)

    def test_w1_positive_control_write_without_disarm_regrants(self):
        class NoDisarm(PermitPlModel):                  # the RTL mutant permit_window_write_keeps_armed
            def wr(self, off, value):
                if off == 0x44:
                    self.timeout = min(max(int(value), self.window_min), self.window_max)
                    return
                PermitPlModel.wr(self, off, value)
        m = NoDisarm(timeout=1000, tb_start=0, hb_half=10)
        m.wr(0x44, 300)
        m.next_frame_id = 1
        m.complete()
        m.advance(400)
        self.assertFalse(m.permit)
        m.wr(0x44, 5000)
        self.assertTrue(m.permit)                       # what the real rule forbids

    def _args(self, ms):
        return argparse.Namespace(permit_window_ms=ms)

    def test_w0_permit_map_covers_window_min_max(self):
        """v3fc2 reads 0x58 WINDOW_MIN / 0x5C WINDOW_MAX: the permit map must reach 0x60"""
        self.assertEqual(kv_regs.LAYOUTS["permit"]["min_size"], 0x60)
        self.assertGreaterEqual(kv_regs.LAYOUTS["permit"]["min_size"], kv_regs.PERMIT_WINDOW_MIN + 8)

    def test_w2_apply_none_writes_nothing(self):
        blk, win = kv_regs.mock_permit()
        self.assertIsNone(kv_server.apply_permit_window(self._args(None), {"permit": blk}, 100000000))
        self.assertEqual(win.writes, [])

    def test_w2_apply_v3fc2_model(self):
        model = PermitPlModel(timeout=100000000, tb_start=0)
        regs = {"permit": PlRegBlock("permit", model)}
        out = kv_server.apply_permit_window(self._args(300.0), regs, 100000000)
        self.assertEqual((out["before_cycles"], out["readback_cycles"]), (100000000, 30000000))
        self.assertEqual((out["min_cycles"], out["max_cycles"]), (10000000, 500000000))
        self.assertAlmostEqual(out["readback_ms"], 300.0)

    def test_w2_apply_read_only_window_warns_and_continues(self):
        """owner 10-26: a readback mismatch (pre-v3fc2, 0x44 read-only) warns, the server goes on"""
        blk, _ = kv_regs.mock_permit(ignore_writes=True)    # pre-v3fc2: 0x44 read-only
        out = kv_server.apply_permit_window(self._args(300.0), {"permit": blk}, 100000000)
        self.assertTrue(out["written"])
        self.assertNotEqual(out["readback_cycles"], out["written_cycles"])
        self.assertEqual(len(out["warnings"]), 1)
        self.assertIn("older than v3fc2", out["warnings"][0])
        self.assertIn("continuing", out["warnings"][0])
        # no permit block / unknown PL clock: a warning, nothing written, no SystemExit
        out = kv_server.apply_permit_window(self._args(300.0), degraded_regs(), 100000000)
        self.assertFalse(out["written"])
        self.assertIn("unavailable", out["warnings"][0])
        blk, win = kv_regs.mock_permit()
        out = kv_server.apply_permit_window(self._args(300.0), {"permit": blk}, None)
        self.assertFalse(out["written"])
        self.assertIn("PL clock unknown", out["warnings"][0])
        self.assertEqual(win.writes, [])

    def test_w2_apply_v3fc2_model_exact_has_no_warning(self):
        model = PermitPlModel(timeout=100000000, tb_start=0)
        out = kv_server.apply_permit_window(self._args(300.0), {"permit": PlRegBlock("permit", model)}, 100000000)
        self.assertNotIn("warnings", out)

    def test_w2_out_of_range_is_clamped_and_warned(self):
        for ms, want_ms in ((99.0, 100.0), (5001.0, 5000.0), (0.0, 100.0), (1e9, 5000.0)):
            self.assertEqual(kv_server.clamp_permit_window_ms(ms)[0], want_ms)
            self.assertIn("clamped", kv_server.clamp_permit_window_ms(ms)[1])
            model = PermitPlModel(timeout=100000000, tb_start=0)
            out = kv_server.apply_permit_window(self._args(ms), {"permit": PlRegBlock("permit", model)}, 100000000)
            self.assertEqual((out["applied_ms"], out["written_cycles"]), (want_ms, int(want_ms * 100000)))
            self.assertAlmostEqual(out["readback_ms"], want_ms)
            self.assertEqual(len(out["warnings"]), 1)          # the clamp only; readback agrees
        self.assertEqual(kv_server.clamp_permit_window_ms(300.0), (300.0, None))

    def test_w2_cli_out_of_range_starts_clamped(self):
        """owner 10-26: 5001 / 99 start the server (clamped, warned) instead of an argparse error;
        no permit block (no UIO) also starts, with a warning"""
        cli = CliEndToEndTests()
        for extra, want in ((["--permit-model", "realtime", "--permit-window-ms", "5001"], 5000.0),
                            (["--permit-model", "realtime", "--permit-window-ms", "99"], 100.0),
                            (["--permit-window-ms", "300"], None)):
            p, port, tmp = cli._run(extra)
            try:
                info = cli._info(port)
            finally:
                cli._stop(p, tmp)
            w = info["permit_window"]
            if want is None:
                self.assertFalse(w["written"])
                self.assertIn("unavailable", w["warnings"][0])
            else:
                self.assertEqual(w["applied_ms"], want)
                self.assertAlmostEqual(w["readback_ms"], want)
                self.assertIn("clamped", w["warnings"][0])

if __name__ == "__main__":
    unittest.main()
