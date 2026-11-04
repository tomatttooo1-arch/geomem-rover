"""KV260 side of TELEM (type 10/11): one snapshot of every block, raw integers only.

docs/design/PC_DATAPLANE_SPEC_20261011.md A.3. Answered inline on the server's
reader thread. Register blocks come from host.kv_regs (UIO + MAGIC gated, shared
lock); a block that is not there is reported ``{"available": false, "reason": ...}``
with its ``avail`` bit 0 -- never zeros standing in for readings. Unit conversion
and ages are the Nano's job (host/kv_status.py). Reads only: nothing here writes
any register.
"""

import resource
import threading
import time

from host.fpga_clock import now_ns

from host.kv_link import (TELEM_BLOCKS, decode_telem_req, encode_telem_ack, mask_blocks,
                          read_snmp)

AR_REASON = "AR display driver does not run inside kv_server (no shared record)"
# --ar drm: host.kv_ar.ArPresenter.telem() (mode, source, last flipped frame id, AR core ns,
# flip interval, fail-safe state); --ar off keeps AR_REASON byte for byte.


def _unavailable(reason):
    return {"available": False, "reason": str(reason)}


class TelemetryResponder:
    def __init__(self, server, *, clock=now_ns):
        self.server = server
        self.clock = clock
        self._cpu_lock = threading.Lock()
        self._cpu_mark = None           # (wall, cpu) of the last >= 1 s window start
        self._cpu_rate = None

    def _proc(self):
        ru = resource.getrusage(resource.RUSAGE_SELF)
        cpu, wall = ru.ru_utime + ru.ru_stime, time.monotonic()
        with self._cpu_lock:
            if self._cpu_mark is None:
                self._cpu_mark = (wall, cpu)
            elif wall - self._cpu_mark[0] >= 1.0:
                self._cpu_rate = (cpu - self._cpu_mark[1]) / (wall - self._cpu_mark[0])
                self._cpu_mark = (wall, cpu)
            rate = self._cpu_rate
        snmp = read_snmp()
        udp, ip = snmp.get("Udp", {}), snmp.get("Ip", {})
        return {"cpu_per_wall_1s": rate, "worker_error": self.server.worker_error,
                "snmp": [udp.get("InErrors"), udp.get("RcvbufErrors"), udp.get("InCsumErrors"),
                         ip.get("ReasmFails")],
                "snmp_names": ["Udp.InErrors", "Udp.RcvbufErrors", "Udp.InCsumErrors",
                               "Ip.ReasmFails"]}

    def _geometry(self):
        s = self.server
        with s._rec_lock:
            last = dict(s.last_geometry) if s.last_geometry else None
        return {"core": s.geometry_desc.get("core"), "wait": s.geometry_desc.get("wait"),
                "served": s.counts["served"], "last": last,
                "slot_superseded": s.counts["slot_superseded"],
                "rx_incomplete_superseded": s.counts["rx_incomplete_superseded"]}

    def _roi(self):
        s = self.server
        if s.roi_core is None:
            return _unavailable("--roi-core none (no ROI core configured)")
        with s._rec_lock:
            last = dict(s.last_roi) if s.last_roi else None
        return {"core": s.roi_core.name, "served": s.roi_served, "last": last}

    def _block(self, name, fn):
        blk = self.server.regs.get(name)
        if blk is None:
            return _unavailable("%s block not opened by kv_server" % name)
        if not blk.available:
            return _unavailable(blk.reason)
        try:
            out = fn(blk)
        except Exception as exc:
            return _unavailable("%s read failed: %s" % (name, exc))
        out["available"] = True
        return out

    def _board(self):
        s = self.server
        out = {"available": True, "kv_boot_id": "%016x" % s.boot_id}
        blk = s.regs.get("sysinfo")
        if blk is not None and blk.available:
            try:
                info = blk.sysinfo()
                out.update(build_id=info["build_id"], pl_clk_hz=info["clk_hz"],
                           sysinfo_version=info["version"])
            except Exception as exc:
                out.update(build_id=None, pl_clk_hz=None, build_id_reason="sysinfo read failed: %s" % exc)
        else:
            out.update(build_id=None, pl_clk_hz=None,
                       build_id_reason=blk.reason if blk is not None else "sysinfo block not opened")
        return out

    def snapshot(self, want):
        names = mask_blocks(want)
        doc = {}
        for name in TELEM_BLOCKS:
            if name not in names:
                continue
            if name == "geometry":
                b = self._geometry()
                b["available"] = True
            elif name == "roi":
                b = self._roi()
                b.setdefault("available", True)
            elif name == "permit":
                b = self._block("permit", lambda blk: blk.permit_snapshot())
                feed = getattr(self.server, "permit_feed", None)
                if b.get("available") and feed is not None:
                    b["feed"] = feed.brief()       # Nano session / id mapping (full: STATS)
            elif name == "encoder":
                b = self._block("encoder", lambda blk: blk.encoder_snapshot())
            elif name == "ar":
                ar = getattr(self.server, "ar", None)
                if ar is None:          # --ar off: the block exactly as before
                    b = _unavailable(AR_REASON)
                else:
                    try:
                        b = ar.telem()
                    except Exception as exc:
                        b = _unavailable("AR presenter state unreadable: %s" % exc)
            elif name == "estop":
                b = self.server.estop.snapshot()
                b["available"] = True
            elif name == "proc":
                b = self._proc()
                b["available"] = True
            else:
                b = self._board()
            doc[name] = b
        avail = 0
        for i, name in enumerate(TELEM_BLOCKS):
            if doc.get(name, {}).get("available"):
                avail |= 1 << i
        return avail, doc

    def answer(self, payload):
        req = decode_telem_req(payload)
        avail, doc = self.snapshot(req["want"])
        return encode_telem_ack(avail, self.clock(), req["t_req_nano_ns"], doc)
