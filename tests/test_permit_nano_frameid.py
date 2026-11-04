"""Permit NEXT_FRAME_ID from the Nano's frame id (owner 10-11), host side.

docs/design/PERMIT_NANO_FRAMEID_20261011.md. The PL is replaced by kv_regs.PermitPlModel, a
transcription of the id / freshness rule of hw/m7/pl/rtl/m7_permit_axil.v (accept iff the
written id is strictly greater than LAST_FRAME_ID; not greater -> reject + revoke at once;
FORCE_STOP disarms; 200 ms = TIMEOUT cycles after the accepted edge).

Assertions (each numbered as in the design doc, section 3):
  A1 normal renewal: every new Nano frame is accepted; LAST_FRAME_ID - pl_minus_nano = Nano id.
  A2 gaps (skipped Nano frames) are accepted and kept.
  A3 duplicated Nano frame id: the PL rejects it (LAST_CAUSE 3), never a renewal.
  A4 older Nano frame id (regress): same.
  A5 Nano restart (new stream_instance, ids from 1 again): first frame of the new session granted.
  A6 late packet of the ended session: dropped, no core, no register write, no renewal.
  A7 expiry: last accepted edge + TIMEOUT cycles -> revoked, cause 1 (REVOKE_TS-DONE_TS = T+1).
  A8 a huge Nano jump advances the PL id by the cap only.
  A9 PL id space exhausted: dropped (safe), no write.
  A10 seed from LAST_FRAME_ID (KV server restart keeps the PL id increasing).
  A11 duplicate before anything was ever accepted: dropped.
  A12 e-stop: frames during FORCE_STOP are not granted; release alone does not grant; the next
      new Nano frame does. The feed writes NEXT_FRAME_ID only.
  A13 only the feeds call write_permit_next_frame_id; KV loss never selects the CPU
      (ros_runtime --geometry-fallback default none).
Positive controls: the old KV counter (KvCounterFrameFeed) renews on a duplicated id (A3) and
on the late packet of an ended session (A6); writing the raw Nano id gets stuck after a Nano
restart (A5). End to end over UDP with KvLinkServer: same scenarios, both feeds.
Run: PYTHONPATH=. python3 -m unittest discover -s tests -p test_permit_nano_frameid.py
(also /usr/bin/python3.8).
"""
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

from host import kv_regs
from host.kv_link import (KvLinkClient, KvLinkServer, KvTelemetryClient, LinkTimeout, NullCore,
                          blocks_mask, crc32, encode_request)
from host.kv_regs import (PERMIT_CTRL, PERMIT_NEXT_FRAME_ID, PL_ID_MAX, KvCounterFrameFeed,
                          NanoFrameFeed, PermitPlModel, mock_permit_pl)

ROOT = Path(__file__).resolve().parents[1]
T = PermitPlModel().timeout


class RawNanoFeed:
    """Variant that writes the Nano frame id itself (no session handling): positive control
    for A5 -- after a Nano restart the PL's old LAST_FRAME_ID blocks every new frame."""

    def __init__(self, permit):
        self.permit = permit

    def admit(self, session, nano_frame_id):
        self.permit.write_permit_next_frame_id(int(nano_frame_id) & 0xFFFFFFFF)
        return kv_regs.AdmitResult(True, "raw", int(nano_frame_id))


def run_frame(feed, model, session, fid, gap=1000):
    """One request: admit, then (if run) the core's completion edge; then ``gap`` cycles."""
    res = feed.admit(session, fid)
    if res.run:
        model.complete()
    model.advance(gap)
    return res


class FeedUnitTests(unittest.TestCase):
    def setUp(self):
        self.blk, self.m = mock_permit_pl()
        self.feed = NanoFrameFeed(self.blk)

    def test_a1_normal_renewal_and_mapping(self):
        for f in range(1, 6):
            r = run_frame(self.feed, self.m, "geoprod-1-aa", f)
            self.assertTrue(r.run)
            self.assertTrue(self.m.permit)
        self.assertEqual(self.m.accepted, 5)
        self.assertEqual(self.m.rejected, 0)
        d = self.feed.describe()
        self.assertEqual(self.m.last_id - d["pl_minus_nano"], 5)
        self.assertEqual([w for w in self.m.writes], [(PERMIT_NEXT_FRAME_ID, f) for f in range(1, 6)])
        self.assertEqual(self.feed.counts["verdict:new_session"], 1)
        self.assertEqual(self.feed.counts["verdict:newer"], 4)

    def test_a2_gaps_are_kept(self):
        for f in (10, 11, 15, 40):
            self.assertTrue(run_frame(self.feed, self.m, "s", f).run)
        self.assertEqual(self.m.accepted, 4)
        ids = [v for off, v in self.m.writes]
        self.assertEqual([b - a for a, b in zip(ids, ids[1:])], [1, 4, 25])

    def _dup_or_regress(self, feed, last_fid):
        for f in range(1, 6):
            run_frame(feed, self.m, "s", f)
        acc, rev = self.m.accepted, self.m.revoke_count
        self.assertTrue(self.m.permit)
        r = run_frame(feed, self.m, "s", last_fid)
        return r, acc, rev

    def test_a3_duplicate_is_rejected_by_pl(self):
        r, acc, rev = self._dup_or_regress(self.feed, 5)
        self.assertEqual(r.verdict, "duplicate")
        self.assertFalse(self.m.permit)
        self.assertEqual(self.m.accepted, acc)
        self.assertEqual(self.m.rejected, 1)
        self.assertEqual(self.m.last_cause, PermitPlModel.C_REPLAY)
        self.assertEqual(self.m.revoke_count, rev + 1)

    def test_a3_positive_control_kv_counter_renews_duplicate(self):
        legacy = KvCounterFrameFeed(self.blk)
        r, acc, rev = self._dup_or_regress(legacy, 5)
        self.assertTrue(r.run)
        self.assertTrue(self.m.permit)                  # misjudged: renewed
        self.assertEqual(self.m.accepted, acc + 1)
        self.assertEqual(self.m.rejected, 0)

    def test_a4_regress_is_rejected_by_pl(self):
        r, acc, _ = self._dup_or_regress(self.feed, 2)
        self.assertEqual(r.verdict, "regress")
        self.assertFalse(self.m.permit)
        self.assertEqual(self.m.accepted, acc)
        self.assertEqual(self.m.last_cause, PermitPlModel.C_REPLAY)
        # and the session goes on: the next newer frame is granted again
        self.assertTrue(run_frame(self.feed, self.m, "s", 6).run)
        self.assertTrue(self.m.permit)

    def test_a5_nano_restart_new_session_granted_first_frame(self):
        for f in range(1, 101):
            run_frame(self.feed, self.m, "geoprod-100-aaaaaa", f)
        old_last = self.m.last_id
        self.m.advance(T)                                # Nano process dead > 200 ms
        self.assertFalse(self.m.permit)
        r = run_frame(self.feed, self.m, "geoprod-200-bbbbbb", 1)
        self.assertEqual(r.verdict, "new_session")
        self.assertTrue(self.m.permit)
        self.assertGreater(self.m.last_id, old_last)
        for f in range(2, 5):
            self.assertTrue(run_frame(self.feed, self.m, "geoprod-200-bbbbbb", f).run)
            self.assertTrue(self.m.permit)
        d = self.feed.describe()
        self.assertEqual(d["sessions_seen"], 2)
        self.assertEqual(self.m.last_id - d["pl_minus_nano"], 4)

    def test_a5_positive_control_raw_nano_id_stuck_after_restart(self):
        raw = RawNanoFeed(self.blk)
        for f in range(1, 101):
            run_frame(raw, self.m, "a", f)
        self.m.advance(T)
        for f in range(1, 101):                          # restarted Nano: 1..100 all rejected
            run_frame(raw, self.m, "b", f)
            self.assertFalse(self.m.permit)
        self.assertEqual(self.m.rejected, 100)

    def _late_old_session(self, feed):
        for f in range(1, 11):
            run_frame(feed, self.m, "A", f)
        for f in range(1, 4):
            run_frame(feed, self.m, "B", f)
        writes, acc = len(self.m.writes), self.m.accepted
        self.m.advance(T - 5000)                         # B's permit about to expire
        r = run_frame(feed, self.m, "A", 11, gap=10000)  # late packet of the ended session A
        return r, writes, acc

    def test_a6_late_packet_of_ended_session_dropped(self):
        r, writes, acc = self._late_old_session(self.feed)
        self.assertEqual((r.run, r.verdict), (False, "retired_session"))
        self.assertEqual(len(self.m.writes), writes)    # no register write at all
        self.assertEqual(self.m.accepted, acc)
        self.assertFalse(self.m.permit)                  # B expired, A could not renew
        self.assertEqual(self.m.last_cause, PermitPlModel.C_TIMEOUT)
        self.assertTrue(run_frame(self.feed, self.m, "B", 4).run)   # live session unaffected
        self.assertTrue(self.m.permit)

    def test_a6_positive_control_kv_counter_renews_late_packet(self):
        r, writes, acc = self._late_old_session(KvCounterFrameFeed(self.blk))
        self.assertTrue(r.run)
        self.assertEqual(self.m.accepted, acc + 1)
        self.assertTrue(self.m.permit)                   # misjudged: dead session renewed

    def test_a7_expiry_after_timeout_cycles(self):
        run_frame(self.feed, self.m, "s", 1, gap=0)
        self.m.advance(T - 1)
        self.assertTrue(self.m.permit)
        self.m.advance(1)
        self.assertFalse(self.m.permit)
        self.assertEqual(self.m.last_cause, PermitPlModel.C_TIMEOUT)
        self.assertEqual(self.m.last_revoke_ts - self.m.last_done_ts, T + 1)

    def test_a8_jump_is_capped(self):
        feed = NanoFrameFeed(self.blk, jump_cap=1000)
        run_frame(feed, self.m, "s", 1)
        r = run_frame(feed, self.m, "s", 1 + 10 ** 9)
        self.assertEqual(r.verdict, "jump_capped")
        self.assertEqual(r.pl_id, 1 + 1000)
        self.assertTrue(self.m.permit)
        self.assertEqual(run_frame(feed, self.m, "s", 2 + 10 ** 9).pl_id, 1002)

    def test_a9_pl_id_exhausted_is_dropped(self):
        self.m.last_id, self.m.last_valid = PL_ID_MAX, True
        feed = NanoFrameFeed(self.blk)
        r = run_frame(feed, self.m, "s", 1)
        self.assertEqual((r.run, r.verdict), (False, "pl_id_exhausted"))
        self.assertEqual(self.m.writes, [])
        self.m.last_id = PL_ID_MAX - 3
        feed = NanoFrameFeed(self.blk)
        self.assertTrue(run_frame(feed, self.m, "s", 1).run)          # PL id MAX-2
        self.assertTrue(run_frame(feed, self.m, "s", 3).run)          # MAX
        r = run_frame(feed, self.m, "s", 4)
        self.assertEqual((r.run, r.verdict), (False, "pl_id_exhausted"))

    def test_a10_seed_and_kv_server_restart(self):
        self.m.last_id, self.m.last_valid = 41, True
        feed = NanoFrameFeed(self.blk)
        self.assertEqual(run_frame(feed, self.m, "s", 7).pl_id, 42)
        run_frame(feed, self.m, "s", 8)
        restarted = NanoFrameFeed(self.blk)              # KV server restart, same Nano session
        r = run_frame(restarted, self.m, "s", 9)
        self.assertEqual(r.verdict, "new_session")
        self.assertEqual(r.pl_id, 44)
        self.assertTrue(self.m.permit)

    def test_a11_duplicate_before_any_accept_dropped(self):
        r = self.feed.admit("s", 1)                      # written, but the core never completed
        self.assertTrue(r.run)
        r = run_frame(self.feed, self.m, "s", 1)
        self.assertEqual((r.run, r.verdict), (False, "duplicate"))
        self.assertEqual(self.m.accepted, 0)
        self.assertFalse(self.m.permit)

    def test_a12_estop_release_needs_new_frame(self):
        run_frame(self.feed, self.m, "s", 1)
        self.blk.write_permit_ctrl(True)                 # the e-stop module's write
        self.assertFalse(self.m.permit)
        self.assertEqual(self.m.last_cause, PermitPlModel.C_FORCE)
        for f in (2, 3):
            run_frame(self.feed, self.m, "s", f)
            self.assertFalse(self.m.permit)
        self.blk.write_permit_ctrl(False)
        self.assertFalse(self.m.permit)                  # release alone never grants
        run_frame(self.feed, self.m, "s", 4)
        self.assertTrue(self.m.permit)
        feed_writes = {off for off, v in self.m.writes} - {PERMIT_CTRL}
        self.assertEqual(feed_writes, {PERMIT_NEXT_FRAME_ID})

    def test_retired_sessions_bounded(self):
        feed = NanoFrameFeed(self.blk, retired_max=3)
        for i in range(6):
            run_frame(feed, self.m, "s%d" % i, 1)
        self.assertEqual(list(feed.retired), ["s2", "s3", "s4"])
        self.assertEqual(run_frame(feed, self.m, "s4", 2).verdict, "retired_session")

    def test_unavailable_block_refused(self):
        broken = kv_regs.PlRegBlock("permit", None, reason="absent")
        with self.assertRaises(kv_regs.RegisterReadError):
            NanoFrameFeed(broken)

    def test_a13_only_feeds_write_next_frame_id_and_no_cpu_fallback(self):
        callers = sorted(str(p.relative_to(ROOT)) for p in list((ROOT / "host").glob("*.py")) +
                         list((ROOT / "tools/kv_link").glob("*.py"))
                         if "write_permit_next_frame_id(" in p.read_text())
        self.assertEqual(callers, ["host/kv_regs.py"])
        src = (ROOT / "host/ros_runtime.py").read_text()
        m = re.search(r'"--geometry-fallback", choices=\(([^)]*)\), default="(\w+)"', src)
        self.assertIsNotNone(m)
        self.assertEqual(m.group(2), "none")


# ------------------------------------------------------------------ end to end over UDP
H, W = 24, 32
K_INV = np.linalg.inv(np.array([[30.0, 0, 15.5], [0, 30.0, 11.5], [0, 0, 1]])).astype(np.float32)


class EdgeCore:
    """NullCore answer + the completion edge of the permit model (what the PL core does)."""

    def __init__(self, model):
        self.model, self.null, self.calls = model, NullCore(), 0

    def __call__(self, *args):
        self.calls += 1
        self.null(*args)
        self.model.complete()


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


class ServerEndToEndTests(unittest.TestCase):
    def _server(self, feed_cls):
        blk, model = mock_permit_pl()
        core = EdgeCore(model)
        srv = KvLinkServer(core, "edge", bind=("127.0.0.1", 0), regs={"permit": blk},
                           permit_feed=feed_cls(blk))
        srv.start()
        self.addCleanup(srv.stop)
        return srv, model, core

    def _send(self, srv, stream, fid, client=None, timeout=1.0):
        own = client is None
        c = client or KvLinkClient(("127.0.0.1", srv.port), timeout_s=timeout)
        try:
            body, expect = request(stream, fid)
            try:
                c.call(body, expect)
                return True
            except LinkTimeout:
                return False
        finally:
            if own:
                c.close()

    def _scenario(self, feed_cls):
        srv, model, core = self._server(feed_cls)
        a = KvLinkClient(("127.0.0.1", srv.port), timeout_s=1.0)
        self.addCleanup(a.close)
        for f in (1, 2, 3):
            self.assertTrue(self._send(srv, "geoprod-11-aaaaaa", f, a))
        self.assertTrue(model.permit)
        b = KvLinkClient(("127.0.0.1", srv.port), timeout_s=1.0)   # "restarted Nano process"
        self.addCleanup(b.close)
        for f in (1, 2):
            self.assertTrue(self._send(srv, "geoprod-22-bbbbbb", f, b))
        self.assertTrue(model.permit)
        acc0 = model.accepted
        late = self._send(srv, "geoprod-11-aaaaaa", 4, a, timeout=0.3)
        acc_late = model.accepted - acc0
        # duplicate of B's frame 2 through a new link session (same stream, same id)
        dup = self._send(srv, "geoprod-22-bbbbbb", 2)
        return srv, model, core, late, acc_late, dup

    def test_nano_feed_end_to_end(self):
        srv, model, core, late, acc_late, dup = self._scenario(NanoFrameFeed)
        self.assertFalse(late)                           # dropped: no core, no answer
        self.assertEqual(acc_late, 0)
        self.assertEqual(srv.counts["permit_drop:retired_session"], 1)
        self.assertTrue(dup)                             # core ran, PL judged it a replay
        self.assertFalse(model.permit)
        self.assertEqual(model.last_cause, PermitPlModel.C_REPLAY)
        self.assertEqual(core.calls, 6)
        tel = KvTelemetryClient(("127.0.0.1", srv.port), timeout_s=0.5, hello_timeout_s=1.0)
        try:
            p = tel.poll(blocks_mask(("permit",))).blocks["permit"]
        finally:
            tel.close()
        self.assertEqual(p["feed"]["session"], "geoprod-22-bbbbbb")
        self.assertEqual(p["feed"]["sessions_seen"], 2)
        self.assertEqual(p["feed"]["dropped"], 1)
        self.assertEqual(p["last_frame_id"] - p["feed"]["pl_minus_nano"], 2)

    def test_positive_control_kv_counter_end_to_end(self):
        srv, model, core, late, acc_late, dup = self._scenario(KvCounterFrameFeed)
        self.assertTrue(late)                            # renewed by a dead session's packet
        self.assertEqual(acc_late, 1)
        self.assertTrue(dup)
        self.assertTrue(model.permit)                    # renewed by a duplicate
        self.assertEqual(model.rejected, 0)


class ProbeDryRunTests(unittest.TestCase):
    """tools/kv_link/permit_session_probe.py end to end on the host: KvLinkServer + a permit
    model whose PL clock follows the host clock (100 MHz) + a core that raises the completion
    edge. The probe's own positive control: --expect nano against the kv-counter server fails."""

    def _probe(self, feed_cls, expect):
        blk = kv_regs.PlRegBlock("permit", kv_regs.RealtimePermitModel())
        srv = KvLinkServer(EdgeCore(blk.window), "edge", bind=("127.0.0.1", 0),
                           regs={"permit": blk}, permit_feed=feed_cls(blk))
        srv.start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                out = os.path.join(tmp, "rep.json")
                r = subprocess.run([sys.executable, str(ROOT / "tools/kv_link/permit_session_probe.py"),
                                    "run", "--server", "127.0.0.1:%d" % srv.port, "--n", "12",
                                    "--m", "8", "--k", "6", "--expect", expect, "--out", out],
                                   capture_output=True, text=True, timeout=180,
                                   env=dict(os.environ, PYTHONPATH=str(ROOT)))
                rep = json.loads(Path(out).read_text()) if os.path.exists(out) else None
        finally:
            srv.stop()
        return r, rep

    def test_probe_nano_passes(self):
        r, rep = self._probe(NanoFrameFeed, "nano")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr[-3000:])
        self.assertTrue(rep["pass"])
        self.assertAlmostEqual(rep["phases"]["P1_P2"]["revoke_after_done_ms_pl"], 1000.0, places=3)  # v3fc2 default

    def test_probe_kv_counter_control_passes_its_own_expectation(self):
        r, rep = self._probe(KvCounterFrameFeed, "kv-counter")
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr[-3000:])

    def test_probe_positive_control_nano_expectation_fails_on_kv_counter(self):
        r, rep = self._probe(KvCounterFrameFeed, "nano")
        self.assertEqual(r.returncode, 1)
        failed = sorted(k for k, v in rep["checks"].items() if not v)
        self.assertIn("P4_late_not_accepted", failed)
        self.assertIn("P5b_dup_and_regress_rejected", failed)


if __name__ == "__main__":
    unittest.main()
