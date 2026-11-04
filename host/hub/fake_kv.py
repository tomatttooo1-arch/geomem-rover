"""FakeKvLink: same-signature stand-ins for package 1's KV clients (spec F contract table).

    KvTelemetryClient(server, *, timeout_s=0.2).poll(want=ALL) -> Telemetry
    KvEstopClient(server).send(op, cmd_id, epoch=0, timeout_s=0.2) -> EstopAck

``server`` here is a FakeKv object (the real clients take (host, port)). Package 1's
contract additions (da4e4b3, PKG1 handover "契约补充") are mirrored: ``poll(want)`` takes the
TELEM bit mask (``host.kv_link.TELEM_ALL``) or block names, both clients accept the optional
keywords ``hello_timeout_s``/``session``/``sock``/``chunk_bytes`` and have ``close()``, and
the answer is built as a raw KV-side TELEM document (raw integers, ``times_ns``,
``kv_done_ns``, permit cycles) decoded by the real ``host.kv_status.decode_telemetry`` --
so the hub sees exactly the Telemetry shape and unit conversion package 1 produces. The fake follows
A.4's e-stop state machine (results 0/1/2/3/5/6, cmd_id dedup of 64, epoch +1 per new
latch) and A.3's "unavailable block = {available: false, reason}" rule, so the hub can be
developed and tested before package 1 lands. Timeouts raise host.kv_link.LinkTimeout,
the exception type package 1 already defines.
"""

import collections
import os
import threading
import time

from host.kv_link import CHUNK_FRAGMENTED, LinkTimeout, TELEM_ALL, TELEM_BLOCKS, EstopAck, mask_blocks
from host.kv_status import Telemetry, decode_telemetry

ALL = TELEM_ALL
FAKE_PL_CLK_HZ = 100000000
_ = Telemetry                                 # re-exported: the real type, not a look-alike


def _want_names(want):
    if isinstance(want, int):
        return set(mask_blocks(want))
    return set(want)


class FakeKv(object):
    """Shared state of one fake kv_server (today's bitstream by default: no permit block)."""

    def __init__(self, permit_avail=False, ctrl_readback_ok=True, sleep_on_timeout=False):
        self.lock = threading.Lock()
        self.boot_id = os.urandom(8).hex()
        self.latched, self.epoch = False, 0
        self.permit_avail, self.ctrl_readback_ok = permit_avail, ctrl_readback_ok
        self.force_stop = False
        self.ctrl_writes = 0
        self.offline = False
        self.drop = 0                 # drop the next N requests of any kind
        self.cache = collections.OrderedDict()
        self.executed = collections.Counter()
        self.sleep_on_timeout = sleep_on_timeout
        self.frame_id = 0
        self.geometry = {"core": "null", "wait": "irq", "served": 0, "slot_superseded": 0,
                         "rx_incomplete_superseded": 0}

    def restart(self):
        with self.lock:
            self.boot_id = os.urandom(8).hex()
            self.latched, self.force_stop = False, False
            self.cache.clear()

    def _lost(self, timeout_s):
        if self.offline or self.drop > 0:
            if self.drop > 0:
                self.drop -= 1
            if self.sleep_on_timeout:
                time.sleep(timeout_s)
            return True
        return False

    def serve_frame(self):
        with self.lock:
            self.frame_id += 1
            self.geometry["served"] += 1

    def telemetry(self, want, timeout_s):
        """Raw KV-side document (as host.kv_telem would send) -> real decode_telemetry."""
        want = _want_names(want)
        with self.lock:
            if self._lost(timeout_s):
                raise LinkTimeout("fake kv: telemetry not answered")
            t_req = 10 ** 12
            kv_mono = 5 * 10 ** 12
            unavailable = {"available": False, "reason": "no_uio_node"}
            times = {"recv": 1.0, "slot_wait": 0.1, "decode": 2.0, "convert": 3.0, "crc": 0.5,
                     "core_prep": 0.2, "core_device": 604.0, "core_readback": 1.0, "core_total": 606.0}
            doc = {
                "geometry": dict(self.geometry, available=True, last={
                    "frame_id": self.frame_id, "capture_ns": 0, "status": 0, "counts": [921600, 1200, 35],
                    "times_ns": {k: int(round(v * 1e6)) for k, v in times.items()}, "pl_cycles": None,
                    "kv_done_ns": kv_mono - 3500000}),           # KV age 3.5 ms (+1.5 ms RTT = 5.0)
                "roi": dict(unavailable), "permit": dict(unavailable), "encoder": dict(unavailable),
                "ar": dict(unavailable),
                "estop": {"available": True, "latched": self.latched, "epoch": self.epoch,
                          "latched_kv_ns": kv_mono - 1000000 if self.latched else None,
                          "force_stop_written": self.force_stop, "pl_applied": False,
                          "last_result": None, "sources": ["pc"] if self.latched else [],
                          "permit_available": self.permit_avail,
                          "permit_reason": None if self.permit_avail else "no_uio_node"},
                "proc": {"available": True, "cpu_per_wall_1s": 0.12, "worker_error": None,
                         "snmp": [0, 0, 0, 0],
                         "snmp_names": ["Udp.InErrors", "Udp.RcvbufErrors", "Udp.InCsumErrors",
                                        "Ip.ReasmFails"]},
                "board": {"available": True, "kv_boot_id": self.boot_id, "build_id": None,
                          "pl_clk_hz": None, "build_id_reason": "no_uio_node"},
            }
            if self.permit_avail:
                clk = FAKE_PL_CLK_HZ
                doc["board"].update(pl_clk_hz=clk, build_id=0x20261011, build_id_reason=None)
                doc["permit"] = {"available": True,
                                 "status": 0b101101 | (8 if self.force_stop else 0), "last_cause": 0,
                                 "revoke_count": 0, "frames_accepted": self.frame_id, "frames_rejected": 0,
                                 "last_frame_id": self.frame_id, "age_cycles": clk * 12 // 1000,
                                 "timeout_cycles": clk // 2, "tb_now": 10 ** 9,
                                 "last_done_ts": 10 ** 9 - clk // 2000,      # 0.5 ms (+1.5 RTT = 2.0)
                                 "last_revoke_ts": 0}
            doc = {k: v for k, v in doc.items() if k in want}
            avail = 0
            for i, name in enumerate(TELEM_BLOCKS):
                if doc.get(name, {}).get("available"):
                    avail |= 1 << i
            ack = {"avail": avail, "kv_mono_ns": kv_mono, "t_req_nano_ns": t_req, "doc": doc}
            return decode_telemetry(ack, t_req, t_req + 1500000)

    def estop(self, op, cmd_id, epoch, timeout_s):
        with self.lock:
            if self._lost(timeout_s):
                raise LinkTimeout("fake kv: estop not answered")
            if cmd_id in self.cache:
                return self.cache[cmd_id]
            self.executed[op] += 1
            if op == "assert":
                if epoch:                                # pkg 1: ASSERT/QUERY with an epoch -> 4
                    result = 4
                elif self.latched:
                    result = 1
                else:
                    self.latched, self.epoch = True, self.epoch + 1
                    result = self._write(True)
            elif op == "release":
                if not self.latched:
                    result = 3
                elif epoch != self.epoch:
                    result = 2
                else:
                    self.latched = False
                    result = self._write(False)
            elif op == "query":                          # pkg 1: result by current state
                if epoch:
                    result = 4
                elif not self.latched:
                    result = 3
                elif not self.permit_avail:
                    result = 5
                else:
                    result = 0 if self.ctrl_readback_ok else 6
            else:
                result = 4
            ack = EstopAck(result, self.latched, self.epoch, self.force_stop and self.permit_avail,
                           self.permit_avail, (8 if self.force_stop else 0) if self.permit_avail else 0,
                           self.boot_id)
            self.cache[cmd_id] = ack
            while len(self.cache) > 64:
                self.cache.popitem(last=False)
            return ack

    def _write(self, value):
        if not self.permit_avail:
            self.force_stop = value
            return 5 if value else 0
        self.ctrl_writes += 1
        self.force_stop = value
        if value and not self.ctrl_readback_ok:
            return 6
        return 0


class _FakeClient(object):
    def __init__(self, server, *, hello_timeout_s=1.0, session=None, sock=None,
                 chunk_bytes=CHUNK_FRAGMENTED):
        self.server = server
        self.hello_timeout_s, self.session, self.sock, self.chunk_bytes = hello_timeout_s, session, sock, chunk_bytes
        self.counts = collections.Counter()

    def close(self):
        pass


class KvTelemetryClient(_FakeClient):
    def __init__(self, server, *, timeout_s=0.2, resend_s=0.05, **kw):
        _FakeClient.__init__(self, server, **kw)
        self.timeout_s = timeout_s
        self.resend_s = resend_s            # accepted for signature parity (no network here)

    def poll(self, want=TELEM_ALL):
        return self.server.telemetry(want, self.timeout_s)


class KvEstopClient(_FakeClient):
    def __init__(self, server, **kw):
        _FakeClient.__init__(self, server, **kw)

    def send(self, op, cmd_id, epoch=0, timeout_s=0.2, _frame_id=None):
        if op not in ("assert", "release", "query"):
            raise ValueError(op)
        return self.server.estop(op, int(cmd_id), int(epoch), timeout_s)
