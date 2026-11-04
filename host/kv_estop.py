"""KV260 e-stop latch: the single writer of permit CTRL.FORCE_STOP (2026-10-11).

docs/design/PC_DATAPLANE_SPEC_20261011.md A.4. The PC e-stop only ADDS a stop
source; it does not change the PL 200 ms heartbeat rule, the MCU 30 ms stop-and-
lock rule or "after release wait for the next accepted geometry completion".

* Latched: ASSERT adds source ``pc``; a new epoch (previous + 1); FORCE_STOP is
  (source set non-empty); after writing CTRL.b0 the STATUS.b3 read-back must be 1.
* Released only explicitly: RELEASE with the matching epoch removes ``pc``;
  CTRL.b0 is cleared only when the set is empty. Nothing else releases it.
* Idempotent delivery: answers are cached by ``cmd_id`` (last 64); a repeated
  command gets the same answer bytes and is not executed again.
* Permit block unavailable (bitstream 0x0117bc0a): the latch is still kept on the
  KV, result 5 (latched, FORCE_STOP not written).
* Write not verified (STATUS.b3 != 1): result 6, rewritten every 100 ms while latched.

The KV server dispatches ESTOP_CMD here on its reader thread (``handle``).
Malformed bodies raise host.kv_link.BadExtension (void, not answered).
"""

from collections import Counter, OrderedDict
import threading
import time

from host.fpga_clock import now_ns

from host.kv_link import (ESTOP_ALREADY, ESTOP_ASSERT, ESTOP_EPOCH_MISMATCH, ESTOP_INVALID,
                          ESTOP_LATCHED_NO_PERMIT, ESTOP_NOT_LATCHED, ESTOP_OK, ESTOP_QUERY,
                          ESTOP_RELEASE, ESTOP_WRITE_UNVERIFIED, decode_estop_cmd,
                          encode_estop_ack)
from host.kv_regs import PERMIT_STATUS_ESTOP

DEDUP = 64
REWRITE_S = 0.1


class EstopLatch:
    def __init__(self, permit=None, *, boot_id=0, clock=now_ns):
        self.permit = permit if (permit is not None and permit.available) else None
        self.permit_reason = (permit.reason if permit is not None else "no permit block opened")
        self.boot_id = int(boot_id)
        self.clock = clock
        self.lock = threading.Lock()
        self.sources = set()
        self.epoch = 0
        self.latched_kv_ns = None
        self.force_stop_written = False
        self.verified = None
        self.permit_status = 0
        self.last_result = None
        self.cache = OrderedDict()
        self.counts = Counter()
        self._stop = threading.Event()
        self._thread = None

    # -- the only CTRL writes in the repository
    def _write(self, value):
        status = self.permit.write_permit_ctrl(value)
        self.counts["ctrl_writes"] += 1
        self.permit_status = status
        return status

    def _apply_assert(self):
        if self.permit is None:
            self.force_stop_written = False
            return ESTOP_LATCHED_NO_PERMIT
        status = self._write(True)
        self.force_stop_written = True
        self.verified = bool(status & PERMIT_STATUS_ESTOP)
        return ESTOP_OK if self.verified else ESTOP_WRITE_UNVERIFIED

    def handle(self, payload):
        cmd = decode_estop_cmd(payload)
        with self.lock:
            cached = self.cache.get(cmd["cmd_id"])
            if cached is not None:
                self.counts["duplicate_cmd"] += 1
                return cached
            op = cmd["op"]
            if op == ESTOP_ASSERT:
                if cmd["epoch"]:
                    result = ESTOP_INVALID
                elif "pc" in self.sources:
                    result = ESTOP_ALREADY
                else:
                    self.sources.add("pc")
                    self.epoch += 1
                    self.latched_kv_ns = self.clock()
                    result = self._apply_assert()
            elif op == ESTOP_RELEASE:
                if "pc" not in self.sources:
                    result = ESTOP_NOT_LATCHED
                elif cmd["epoch"] != self.epoch:
                    result = ESTOP_EPOCH_MISMATCH
                else:
                    self.sources.discard("pc")
                    if not self.sources and self.permit is not None:
                        self._write(False)
                    self.force_stop_written = bool(self.sources) and self.force_stop_written
                    self.verified = None
                    result = ESTOP_OK
            elif op == ESTOP_QUERY:
                result = ESTOP_INVALID if cmd["epoch"] else self._state_result()
            else:                                   # decode_estop_cmd already refuses it
                result = ESTOP_INVALID
            self.last_result = result
            self.counts["result:%d" % result] += 1
            body = self._ack(op, result, cmd["cmd_id"])
            self.cache[cmd["cmd_id"]] = body
            while len(self.cache) > DEDUP:
                self.cache.popitem(last=False)
            return body

    def _state_result(self):
        if not self.sources:
            return ESTOP_NOT_LATCHED
        if self.permit is None:
            return ESTOP_LATCHED_NO_PERMIT
        return ESTOP_OK if self.verified else ESTOP_WRITE_UNVERIFIED

    def _ack(self, op, result, cmd_id):
        return encode_estop_ack(op=op, result=result, cmd_id=cmd_id, latched=bool(self.sources),
                                force_stop_written=self.force_stop_written,
                                permit_avail=self.permit is not None, epoch=self.epoch,
                                permit_status=self.permit_status if self.permit is not None else 0,
                                kv_mono_ns=self.clock(), kv_boot_id=self.boot_id)

    # -- result 6: rewrite every 100 ms while latched and unverified
    def tick(self):
        with self.lock:
            if self.sources and self.permit is not None and not self.verified:
                self.counts["rewrites"] += 1
                self.verified = bool(self._write(True) & PERMIT_STATUS_ESTOP)
                self.last_result = ESTOP_OK if self.verified else ESTOP_WRITE_UNVERIFIED

    def start(self):
        if self._thread is None:
            def loop():
                while not self._stop.wait(REWRITE_S):
                    self.tick()
            self._thread = threading.Thread(target=loop, name="kv-estop", daemon=True)
            self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def snapshot(self):
        """TELEM estop block (raw)."""
        with self.lock:
            return {"latched": bool(self.sources), "epoch": self.epoch,
                    "latched_kv_ns": self.latched_kv_ns,
                    "force_stop_written": self.force_stop_written,
                    "pl_applied": (bool(self.permit_status & PERMIT_STATUS_ESTOP)
                                   if self.permit is not None else None),
                    "last_result": self.last_result, "sources": sorted(self.sources),
                    "permit_available": self.permit is not None,
                    "permit_reason": None if self.permit is not None else self.permit_reason}
