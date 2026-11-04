"""PL register blocks the KV260 server reads (sysinfo / permit / encoder), 2026-10-11.

See docs/design/PC_DATAPLANE_SPEC_20261011.md A.3 / A.4. Rules:

* Gate before every block: a generic-uio node of the block's name exists, its
  ``maps/map0`` is readable and large enough, the mmap of ``/dev/uioN`` works and
  the MAGIC register matches. Anything else -> the block is unavailable with a
  reason and is never touched again. There is no fallback to a raw physical
  address: bitstream 0x0117bc0a has no slave behind the placeholder addresses
  of permit/encoder/AR and an unassigned AXI read can hang the bus.
* One ``PlRegBlock`` per block per process, shared by every user (B2 SYNC
  responder, TELEM, the e-stop module, pl_cycles). Every multi-register read
  (TIMEBASE LO then HI -- reading LO latches HI for the whole block --, the
  encoder's UPDATE_COUNT bracketed read) holds the block's lock.
* Reads only, except three permit writes: ``write_permit_window`` (WINDOW_CYCLES 0x44,
  v3fc2 2026-10-26; only caller kv_server --permit-window-ms, once at start, before any
  frame), ``write_permit_ctrl`` (permit CTRL.FORCE_STOP),
  whose only caller is the e-stop module (host/kv_estop.py), and
  ``write_permit_next_frame_id`` (NEXT_FRAME_ID 0x08, PU §3.4 item 2), whose only callers
  are the geometry-frame feeds below (NanoFrameFeed: id from the Nano's frame id per Nano
  session, the default; KvCounterFrameFeed: the old KV counter, positive control only),
  called by kv_link.KvLinkServer right before each frame-core start. Nothing here ever writes
  the encoder CTRL (write-only CLEAR_ODOM / CLEAR_COUNTERS).

Python 3.8 compatible. The mock window at the end is for host tests.
"""

from collections import Counter, OrderedDict, namedtuple
import mmap
import os
import threading

LAYOUTS = {
    # hw/m7/pl/overlay/m7_pl.dtso + hw/m7/pl/sw/m7pl_dev.cpp (sysinfo, present in 0x0117bc0a);
    # PL_PERMIT_UART_SPEC 3.3 / 4.4 (permit, encoder: placeholder blocks, not in 0x0117bc0a).
    "sysinfo": {"uio": "m7_sysinfo", "magic": 0x4D37504C, "min_size": 0x24},
    "permit": {"uio": "m7_permit", "magic": 0x4D37504D, "min_size": 0x60},
    "encoder": {"uio": "m7_encoder", "magic": 0x4D37454E, "min_size": 0xC8},
}

# sysinfo
SYS_MAGIC, SYS_VERSION, SYS_BUILD_ID, SYS_BUILD_DATE, SYS_CLK_HZ = 0x00, 0x04, 0x08, 0x0C, 0x10
SYS_SLOT_MAP = 0x20     # bit k = control slot 0xA00k_0000 populated (0x1F on 0x0117bc0a)

# permit (PU 3.3)
PERMIT_CTRL = 0x04
PERMIT_NEXT_FRAME_ID = 0x08
PERMIT_LAST_FRAME_ID = 0x18
PERMIT_STATUS = 0x10
PERMIT_STATUS_ESTOP = 1 << 3
PERMIT_STATUS_ID_VALID = 1 << 4
PERMIT_WINDOW = 0x44           # RW from v3fc2 (power-on 1000 ms, writes clamped, a write disarms)
PERMIT_WINDOW_MIN = 0x58       # RO from v3fc2 (0 on older bitstreams: 0x44 read-only there)
PERMIT_WINDOW_MAX = 0x5C
PERMIT_WORDS = (("status", 0x10), ("age_cycles", 0x14), ("last_frame_id", 0x18),
                ("frames_accepted", 0x1C), ("frames_rejected", 0x20), ("revoke_count", 0x24),
                ("last_cause", 0x28), ("timeout_cycles", 0x44))
PERMIT_PAIRS = (("last_done_ts", 0x2C, 0x30), ("last_revoke_ts", 0x34, 0x38))
PERMIT_TIMEBASE = (0x4C, 0x50)

# encoder (PU 4.4)
ENC_UPDATE_COUNT = 0x24
ENC_WORDS = (("status", 0x20), ("seq_word", 0x30), ("left_count", 0x34), ("right_count", 0x38),
             ("odom_flags", 0x3C), ("v_left_q16", 0x40), ("v_right_q16", 0x44),
             ("v_center_q16", 0x48), ("heading_q16", 0x64), ("dt_cycles", 0x68))
ENC_SIGNED = ("left_count", "right_count", "v_left_q16", "v_right_q16", "v_center_q16",
              "heading_q16")
ENC_PAIRS = (("frame_ts", 0x28, 0x2C), ("dist_left_um", 0x4C, 0x50),
             ("dist_right_um", 0x54, 0x58), ("dist_center_um", 0x5C, 0x60))
ENC_SIGNED64 = ("dist_left_um", "dist_right_um", "dist_center_um")
ENC_COUNTER_NAMES = ("good", "bad_checksum", "seq_jumps", "lost", "duplicate", "partial",
                     "framing", "sync_skipped_bytes", "link_timeouts", "start_glitches",
                     "count_jumps")
ENC_COUNTERS = 0x6C                    # 11 words, 0x6C..0x94
ENC_TIMEBASE = (0xB0, 0xB4)
ENC_MAX_TRIES = 5


class RegisterReadError(RuntimeError):
    pass


def _s32(v):
    return v - (1 << 32) if v & 0x80000000 else v


def _s64(v):
    return v - (1 << 64) if v & (1 << 63) else v


def _read_sysfs_int(path):
    with open(path) as fp:
        return int(fp.read().strip(), 0)


class UioWindow:
    """32-bit register window of one generic-uio node (map0 of /dev/uioN).

    Each access is one 32-bit load/store through a uint32 memoryview (a wider or
    byte-wise copy of TIMEBASE_LO/HI would break the latch-on-LO rule).
    """

    def __init__(self, path, size, opener=os.open):
        self.path = path
        self.fd = opener(path, os.O_RDWR | os.O_SYNC)
        try:
            self.mm = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, 0)
        except Exception:
            os.close(self.fd)
            raise
        self.words = memoryview(self.mm).cast("I")

    def rd(self, off):
        return int(self.words[off >> 2])

    def wr(self, off, value):
        self.words[off >> 2] = int(value) & 0xFFFFFFFF

    def close(self):
        self.words.release()
        self.mm.close()
        os.close(self.fd)


class PlRegBlock:
    """One PL register block; ``available`` False => degraded, see ``reason``."""

    def __init__(self, name, window=None, *, reason=None, detail=None):
        if name not in LAYOUTS:
            raise ValueError("unknown register block %r" % name)
        self.name = name
        self.layout = LAYOUTS[name]
        self.window = window
        self.available = window is not None and not reason
        self.reason = reason or ("" if window is not None else "no registers")
        self.detail = dict(detail or {})
        self.lock = threading.Lock()
        self.counts = Counter()

    @classmethod
    def open(cls, name, *, sysfs_root="/sys/class/uio", dev_root="/dev", opener=os.open):
        """UIO node of the block's name -> map0 -> mmap -> MAGIC; any failure -> degraded."""
        from host.m7_pl_device import find_uio
        lay = LAYOUTS[name]

        def degraded(reason, **detail):
            return cls(name, None, reason=reason, detail=detail)
        try:
            n = find_uio(lay["uio"], sysfs_root)
        except Exception as exc:
            return degraded("uio lookup failed: %s" % exc)
        if n is None:
            return degraded("no UIO node named %r under %s (bitstream without the %s block)"
                            % (lay["uio"], sysfs_root, name))
        mdir = os.path.join(sysfs_root, "uio%d" % n, "maps", "map0")
        try:
            addr = _read_sysfs_int(os.path.join(mdir, "addr"))
            size = _read_sysfs_int(os.path.join(mdir, "size"))
        except (OSError, ValueError) as exc:
            return degraded("uio%d map0 unreadable: %s" % (n, exc))
        detail = {"uio": "uio%d" % n, "map_addr": hex(addr), "map_size": hex(size)}
        if size < lay["min_size"]:
            return degraded("uio%d map0 size %#x < %#x" % (n, size, lay["min_size"]), **detail)
        try:
            window = UioWindow(os.path.join(dev_root, "uio%d" % n), size, opener=opener)
        except (OSError, ValueError) as exc:
            return degraded("mmap %s/uio%d failed: %s" % (dev_root, n, exc), **detail)
        magic = window.rd(0)
        detail["magic"] = hex(magic)
        if magic != lay["magic"]:
            window.close()
            return degraded("MAGIC %#010x != %#010x at uio%d" % (magic, lay["magic"], n), **detail)
        return cls(name, window, detail=detail)

    def _need(self):
        if not self.available:
            raise RegisterReadError("%s block unavailable: %s" % (self.name, self.reason))

    def _pair(self, lo, hi):
        """64-bit value, LO first (caller holds the lock)."""
        w = self.window
        low = w.rd(lo)
        return (w.rd(hi) << 32) | low

    # -- reads (each one snapshot under the block lock)
    def sysinfo(self):
        self._need()
        with self.lock:
            w = self.window
            return {"magic": w.rd(SYS_MAGIC), "version": w.rd(SYS_VERSION),
                    "build_id": w.rd(SYS_BUILD_ID), "build_date": w.rd(SYS_BUILD_DATE),
                    "clk_hz": w.rd(SYS_CLK_HZ), "slot_map": w.rd(SYS_SLOT_MAP)}

    def timebase(self):
        """Shared PL timebase (permit 0x4C/0x50, encoder 0xB0/0xB4): LO latches HI."""
        self._need()
        lo, hi = {"permit": PERMIT_TIMEBASE, "encoder": ENC_TIMEBASE}[self.name]
        with self.lock:
            return self._pair(lo, hi)

    def permit_snapshot(self):
        self._need()
        if self.name != "permit":
            raise ValueError("not the permit block")
        with self.lock:
            w = self.window
            out = {k: w.rd(off) for k, off in PERMIT_WORDS}
            for k, lo, hi in PERMIT_PAIRS:
                out[k] = self._pair(lo, hi)
            out["tb_now"] = self._pair(*PERMIT_TIMEBASE)
        self.counts["permit_snapshots"] += 1
        return out

    def encoder_snapshot(self, max_tries=ENC_MAX_TRIES):
        """All published odometry from one PL update: UPDATE_COUNT read before and
        after the data; if it moved, read again (counted in ``counts['reread']``)."""
        self._need()
        if self.name != "encoder":
            raise ValueError("not the encoder block")
        with self.lock:
            w = self.window
            for _ in range(int(max_tries)):
                u1 = w.rd(ENC_UPDATE_COUNT)
                out = {k: w.rd(off) for k, off in ENC_WORDS}
                for k, lo, hi in ENC_PAIRS:
                    out[k] = self._pair(lo, hi)
                out["counters"] = [w.rd(ENC_COUNTERS + 4 * i) for i in range(len(ENC_COUNTER_NAMES))]
                out["tb_now"] = self._pair(*ENC_TIMEBASE)
                u2 = w.rd(ENC_UPDATE_COUNT)
                if u1 == u2:
                    break
                self.counts["reread"] += 1
            else:
                self.counts["inconsistent"] += 1
                raise RegisterReadError("encoder UPDATE_COUNT moved on %d consecutive reads" % max_tries)
        for k in ENC_SIGNED:
            out[k] = _s32(out[k])
        for k in ENC_SIGNED64:
            out[k] = _s64(out[k])
        out["update_count"] = u2
        self.counts["encoder_snapshots"] += 1
        return out

    # -- the one write (e-stop module only)
    def write_permit_ctrl(self, force_stop):
        """CTRL.FORCE_STOP <- force_stop, then read STATUS back; returns STATUS."""
        self._need()
        if self.name != "permit":
            raise ValueError("FORCE_STOP lives in the permit block")
        with self.lock:
            self.window.wr(PERMIT_CTRL, 1 if force_stop else 0)
            status = self.window.rd(PERMIT_STATUS)
        self.counts["ctrl_writes"] += 1
        return status

    def write_permit_window(self, cycles):
        """WINDOW_CYCLES (0x44) <- cycles, read back under the block lock; returns
        (readback, min, max). The PL clamps to [min, max] and the write disarms (permission
        resumes at the next accepted completion edge). On a bitstream older than v3fc2 the
        register is read-only: readback stays the build parameter and min/max read 0."""
        self._need()
        if self.name != "permit":
            raise ValueError("WINDOW_CYCLES lives in the permit block")
        with self.lock:
            self.window.wr(PERMIT_WINDOW, int(cycles) & 0xFFFFFFFF)
            back = self.window.rd(PERMIT_WINDOW)
            lo = self.window.rd(PERMIT_WINDOW_MIN)
            hi = self.window.rd(PERMIT_WINDOW_MAX)
        self.counts["window_writes"] += 1
        return back, lo, hi

    def write_permit_next_frame_id(self, frame_id):
        """NEXT_FRAME_ID <- frame_id (the id the next geometry completion edge is judged
        with, PU §3.1/§3.4). Only callers: NanoFrameFeed / KvCounterFrameFeed. Not CTRL."""
        self._need()
        if self.name != "permit":
            raise ValueError("NEXT_FRAME_ID lives in the permit block")
        with self.lock:
            self.window.wr(PERMIT_NEXT_FRAME_ID, frame_id)
        self.counts["next_frame_id_writes"] += 1

    def permit_last_frame_id(self):
        self._need()
        with self.lock:
            return self.window.rd(PERMIT_LAST_FRAME_ID)

    def permit_id_state(self):
        """(LAST_FRAME_ID, STATUS b4 last_id_valid) read under the block lock."""
        self._need()
        if self.name != "permit":
            raise ValueError("not the permit block")
        with self.lock:
            last = self.window.rd(PERMIT_LAST_FRAME_ID)
            status = self.window.rd(PERMIT_STATUS)
        return last, bool(status & PERMIT_STATUS_ID_VALID)

    def describe(self):
        return {"block": self.name, "available": self.available, "reason": self.reason,
                "detail": self.detail, "counts": dict(self.counts)}

    def close(self):
        if self.window is not None and hasattr(self.window, "close"):
            self.window.close()


PL_ID_MAX = 0xFFFFFFFF          # NEXT_FRAME_ID / LAST_FRAME_ID are plain 32-bit unsigned
PERMIT_JUMP_CAP = 1 << 20       # one Nano frame-id jump advances the PL id by at most this
RETIRED_SESSIONS = 256          # how many ended Nano sessions are remembered (LRU)

AdmitResult = namedtuple("AdmitResult", "run verdict pl_id")


class NanoFrameFeed:
    """NEXT_FRAME_ID from the Nano's frame id, per Nano session (owner 10-11).

    The permit block (hw/m7/pl/rtl/m7_permit_axil.v) accepts a completion edge only if the
    id written to NEXT_FRAME_ID is strictly greater than LAST_FRAME_ID (32-bit unsigned,
    no wrap rule, no software reset: only a PL reset / bitstream load clears it); an id that
    is not greater revokes at once (LAST_CAUSE 3). The Nano's frame id restarts when the
    Nano process restarts, so it cannot be written raw. This feed writes

        pl_id = base + (nano_frame_id - first_nano_id)       within one Nano session

    where ``base`` = (largest id written or read back from LAST_FRAME_ID) + 1 at the session's
    first frame. Order and gaps of the Nano frame ids are kept exactly; the PL still judges
    every renewal. Session = the request's ``identity.stream_instance`` (one per Nano frame
    producer process: geoprod-<pid>-<rand>, kvgeo-<tag>-<pid>-<rand>, kvlink-<tag>-<pid>;
    kept across KV reconnects, so the frame ids of one session are one sequence).

    ``admit(session, nano_frame_id)`` runs in the server worker right before the frame
    core starts and returns AdmitResult(run, verdict, pl_id):

    * new_session   unseen session: the current one is retired, base re-read; run, write base.
    * newer         current session, id > last: run, write base + (id - first).
    * jump_capped   id - last > PERMIT_JUMP_CAP: run, PL id advances by the cap only (session
                    re-anchored; keeps the 32-bit space from being burnt by one bad jump).
    * duplicate / regress   current session, id == / < last: run with NEXT_FRAME_ID =
                    LAST_FRAME_ID as read now, so the PL rejects it as a replay and revokes
                    (spec 3.1) -- never a renewal. If the PL has never seen an id (STATUS b4
                    = 0, nothing granted yet) the frame is dropped instead (run False).
    * retired_session   a session that has ended (a newer one took over): dropped, no core,
                    no write -- a late packet of a dead Nano process can neither renew nor
                    revoke the live session.
    * pl_id_exhausted   the PL id would pass 0xFFFFFFFF: dropped (no renewal; the car stops
                    within 200 ms); only a bitstream reload clears LAST_FRAME_ID. At 30 frames/s
                    that is 4.5 years of frames.
    A NEXT_FRAME_ID write error is counted and the frame still runs (the stale id makes the
    PL reject it: safe). The feed never touches CTRL, CAPTURE_AGE or any other register.
    """

    mode = "nano_frame_id"

    def __init__(self, permit, *, jump_cap=PERMIT_JUMP_CAP, retired_max=RETIRED_SESSIONS):
        self.permit = permit
        self.jump_cap, self.retired_max = int(jump_cap), int(retired_max)
        last, _valid = permit.permit_id_state()     # raises if the block is unavailable
        self.seed_last_frame_id = last
        self.last_written = last
        self.session = None
        self.base = self.first = self.last_nano = self.last_pl = None
        self.retired = OrderedDict()
        self.sessions_seen = 0
        self.last = None
        self.counts = Counter()
        self._lock = threading.Lock()

    def _id_state(self):
        try:
            return self.permit.permit_id_state()
        except Exception as exc:
            self.counts["read_error:%s" % type(exc).__name__] += 1
            return None, None

    def _write(self, pl_id):
        try:
            self.permit.write_permit_next_frame_id(pl_id)
            self.counts["written"] += 1
        except Exception as exc:                 # geometry still runs; the PL rejects the stale id
            self.counts["write_error:%s" % type(exc).__name__] += 1
        if pl_id > self.last_written:
            self.last_written = pl_id

    def admit(self, session, nano_frame_id):
        f = int(nano_frame_id)
        if f < 0:
            raise ValueError("nano frame id must be >= 0")
        session = str(session)
        with self._lock:
            res = self._admit(session, f)
            self.counts["verdict:%s" % res.verdict] += 1
            if not res.run:
                self.counts["dropped"] += 1
            self.last = {"session": session, "nano_frame_id": f, "pl_id": res.pl_id,
                         "verdict": res.verdict, "run": res.run}
            return res

    def _admit(self, session, f):
        if session in self.retired:
            self.retired.move_to_end(session)
            return AdmitResult(False, "retired_session", None)
        if session != self.session:
            last, _valid = self._id_state()
            floor = self.last_written if last is None else max(self.last_written, last)
            pl = floor + 1
            if pl > PL_ID_MAX:
                return AdmitResult(False, "pl_id_exhausted", None)
            if self.session is not None:
                self.retired[self.session] = True
                while len(self.retired) > self.retired_max:
                    self.retired.popitem(last=False)
            self.session, self.base, self.first = session, pl, f
            self.last_nano, self.last_pl = f, pl
            self.sessions_seen += 1
            self._write(pl)
            return AdmitResult(True, "new_session", pl)
        if f > self.last_nano:
            verdict = "newer"
            if f - self.last_nano > self.jump_cap:
                verdict = "jump_capped"
                self.base, self.first = self.last_pl + self.jump_cap, f
            pl = self.base + (f - self.first)
            if pl > PL_ID_MAX:
                return AdmitResult(False, "pl_id_exhausted", None)
            self.last_nano, self.last_pl = f, pl
            self._write(pl)
            return AdmitResult(True, verdict, pl)
        verdict = "duplicate" if f == self.last_nano else "regress"
        last, valid = self._id_state()
        if last is None or not valid:
            return AdmitResult(False, verdict, None)    # nothing granted that could be renewed
        self._write(last)                                # not newer -> PL replay reject + revoke
        return AdmitResult(True, verdict, last)

    def describe(self):
        with self._lock:
            return {"mode": self.mode, "session": self.session, "base": self.base,
                    "first_nano_id": self.first, "last_nano_id": self.last_nano,
                    "last_pl_id": self.last_pl,
                    "pl_minus_nano": (None if self.base is None else self.base - self.first),
                    "last_written": self.last_written,
                    "seed_last_frame_id": self.seed_last_frame_id,
                    "sessions_seen": self.sessions_seen, "retired": len(self.retired),
                    "jump_cap": self.jump_cap, "last": dict(self.last) if self.last else None,
                    "counts": dict(self.counts)}

    def brief(self):
        """Compact form for every TELEM permit block."""
        with self._lock:
            last = self.last or {}
            return {"mode": self.mode, "session": self.session, "last_nano_id": self.last_nano,
                    "last_pl_id": self.last_pl,
                    "pl_minus_nano": (None if self.base is None else self.base - self.first),
                    "sessions_seen": self.sessions_seen, "last_verdict": last.get("verdict"),
                    "dropped": self.counts["dropped"]}


class KvCounterFrameFeed:
    """The 10-11 KV-side counter (superseded by NanoFrameFeed, owner 10-11): every frame-core
    start gets LAST_FRAME_ID + 1, + 2, ... whatever Nano frame or session it came from.
    Kept only as the positive control (``kv_server.py --permit-id-source kv-counter``):
    it renews on a duplicated Nano frame id and on a late packet of an ended Nano session."""

    mode = "kv_counter"

    def __init__(self, permit):
        self.permit = permit
        self.next_id = (permit.permit_last_frame_id() + 1) & 0xFFFFFFFF or 1
        self.seed = self.next_id
        self.counts = Counter()
        self.last = None

    def admit(self, session, nano_frame_id):
        pl = self.next_id
        try:
            self.permit.write_permit_next_frame_id(pl)
            self.counts["written"] += 1
        except Exception as exc:
            self.counts["write_error:%s" % type(exc).__name__] += 1
        self.next_id = (self.next_id + 1) & 0xFFFFFFFF or 1
        self.counts["verdict:kv_counter"] += 1
        self.last = {"session": str(session), "nano_frame_id": int(nano_frame_id), "pl_id": pl,
                     "verdict": "kv_counter", "run": True}
        return AdmitResult(True, "kv_counter", pl)

    def describe(self):
        return {"mode": self.mode, "seed": self.seed, "next_id": self.next_id,
                "last": dict(self.last) if self.last else None, "counts": dict(self.counts)}

    def brief(self):
        last = self.last or {}
        return {"mode": self.mode, "session": last.get("session"),
                "last_nano_id": last.get("nano_frame_id"), "last_pl_id": last.get("pl_id"),
                "next_id": self.next_id}


PERMIT_FEEDS = {"nano": NanoFrameFeed, "kv-counter": KvCounterFrameFeed}


# Control slot of each optional block (0xA00k_0000 <-> sysinfo SLOT_MAP bit k).
BLOCK_SLOTS = {"permit": 6, "encoder": 7}


def slot_gate(sysinfo_block, name):
    """None if ``name`` may be opened, else the reason it is skipped.

    A readable sysinfo whose SLOT_MAP is plausible (bit 0, sysinfo itself, set) and
    lacks the block's slot means the loaded bitstream has no such block (v3fc
    removed the encoder UART, slot 7): the block is skipped without touching its
    UIO node. Without a usable sysinfo the per-block UIO + MAGIC gate decides alone.
    """
    slot = BLOCK_SLOTS.get(name)
    if slot is None or sysinfo_block is None or not sysinfo_block.available:
        return None
    try:
        slot_map = sysinfo_block.sysinfo()["slot_map"]
    except Exception:
        return None
    if not slot_map & 1 or slot_map & (1 << slot):
        return None
    return ("control slot %d not in sysinfo SLOT_MAP %#x (bitstream without the %s block); not opened"
            % (slot, slot_map, name))


def open_blocks(names=("sysinfo", "permit", "encoder"), **kw):
    """{name: PlRegBlock} -- each block opened once for the whole process.

    sysinfo is opened first; a block whose control slot is absent from its SLOT_MAP
    is returned degraded without being opened (``slot_gate``)."""
    out = {}
    sysblk = PlRegBlock.open("sysinfo", **kw) if "sysinfo" in names else None
    if sysblk is not None:
        out["sysinfo"] = sysblk
    for n in names:
        if n == "sysinfo":
            continue
        why = slot_gate(sysblk, n)
        out[n] = PlRegBlock(n, None, reason=why, detail={"slot_map_gate": True}) if why \
            else PlRegBlock.open(n, **kw)
    return {n: out[n] for n in names}


# ------------------------------------------------------------------ host mock
class MockWindow:
    """Host stand-in for a UioWindow: a dict of 32-bit words.

    ``timebase=(lo, hi)`` models the latch: every read of ``lo`` advances the
    counter by ``tb_step`` and latches its high word, ``hi`` returns the latch
    (so two unlocked readers interleaving LO/HI read torn values).
    ``on_read(off)`` runs before each read (to model a PL publish between reads);
    ``ignore_writes`` drops writes (a block that does not take CTRL);
    ``write_effects`` maps (off, value) -> {off: value} applied after a write.
    Every write is recorded in ``writes``.
    """

    def __init__(self, words=None, *, timebase=None, tb_start=0, tb_step=1, on_read=None,
                 ignore_writes=False, write_effects=None, yield_on_lo=False):
        self.words = dict(words or {})
        self.timebase = timebase
        self.tb = int(tb_start)
        self.tb_step = int(tb_step)
        self.latched_hi = 0
        self.issued = set()
        self.on_read = on_read
        self.ignore_writes = ignore_writes
        self.write_effects = write_effects
        self.yield_on_lo = yield_on_lo
        self.writes = []
        self.reads = Counter()

    def rd(self, off):
        self.reads[off] += 1
        if self.on_read is not None:
            self.on_read(off)
        if self.timebase is not None and off == self.timebase[0]:
            self.tb += self.tb_step
            self.issued.add(self.tb)
            self.latched_hi = self.tb >> 32
            low = self.tb & 0xFFFFFFFF
            if self.yield_on_lo:
                import time
                time.sleep(0)          # give another thread the GIL between LO and HI
            return low
        if self.timebase is not None and off == self.timebase[1]:
            return self.latched_hi
        return int(self.words.get(off, 0)) & 0xFFFFFFFF

    def wr(self, off, value):
        self.writes.append((off, int(value)))
        if self.ignore_writes:
            return
        self.words[off] = int(value) & 0xFFFFFFFF
        if self.write_effects:
            for k, v in (self.write_effects(off, int(value)) or {}).items():
                self.words[k] = v

    def close(self):
        pass


def mock_permit(*, status=0, tb_start=1 << 32, tb_step=7, **kw):
    """A permit block whose CTRL.b0 drives STATUS.b3 (unless ignore_writes)."""
    words = {0x00: LAYOUTS["permit"]["magic"], PERMIT_STATUS: status, PERMIT_WINDOW: 100000000}

    def effects(off, value):
        if off == PERMIT_CTRL:
            st = win.words.get(PERMIT_STATUS, 0)
            st = (st | PERMIT_STATUS_ESTOP) if value & 1 else (st & ~PERMIT_STATUS_ESTOP)
            return {PERMIT_STATUS: st}
        return None
    kw.setdefault("write_effects", effects)
    win = MockWindow(words, timebase=PERMIT_TIMEBASE, tb_start=tb_start, tb_step=tb_step, **kw)
    return PlRegBlock("permit", win), win


def mock_encoder(fields=None, *, tb_start=1 << 32, tb_step=7, **kw):
    words = {0x00: LAYOUTS["encoder"]["magic"]}
    words.update(fields or {})
    win = MockWindow(words, timebase=ENC_TIMEBASE, tb_start=tb_start, tb_step=tb_step, **kw)
    return PlRegBlock("encoder", win), win


def mock_sysinfo(build_id=0x0117BC0A, clk_hz=100000000, slot_map=None):
    if slot_map is None:
        slot_map = 0x3D7 if build_id == 0x07784F75 else 0x1F
    win = MockWindow({SYS_MAGIC: LAYOUTS["sysinfo"]["magic"], SYS_VERSION: 0x00010000,
                      SYS_BUILD_ID: build_id, SYS_CLK_HZ: clk_hz, SYS_SLOT_MAP: slot_map})
    return PlRegBlock("sysinfo", win), win


def encoder_words(*, update_count=1, status=1, frame_ts=0, seq_word=0, left=0, right=0,
                  odom_flags=0, v_left_q16=0, v_right_q16=0, v_center_q16=0, dist_left_um=0,
                  dist_right_um=0, dist_center_um=0, heading_q16=0, dt_cycles=0, counters=None):
    """Register words of one encoder publication (signed values two's-complement)."""
    m32 = 0xFFFFFFFF
    w = {ENC_UPDATE_COUNT: update_count, 0x20: status, 0x30: seq_word, 0x34: left & m32,
         0x38: right & m32, 0x3C: odom_flags, 0x40: v_left_q16 & m32, 0x44: v_right_q16 & m32,
         0x48: v_center_q16 & m32, 0x64: heading_q16 & m32, 0x68: dt_cycles}
    for (name, lo, hi), val in zip(ENC_PAIRS, (frame_ts, dist_left_um, dist_right_um, dist_center_um)):
        v = val & ((1 << 64) - 1)
        w[lo], w[hi] = v & m32, v >> 32
    for i, c in enumerate(counters or [0] * len(ENC_COUNTER_NAMES)):
        w[ENC_COUNTERS + 4 * i] = c
    return w


class PermitPlModel:
    """Host stand-in for the permit block's id / freshness rule: a line-by-line transcription
    of hw/m7/pl/rtl/m7_permit_axil.v (pl-merge 7784f75) for one completion edge at a time.
    Duck-types a UioWindow (rd / wr / close) so PlRegBlock("permit", model) works; the frame
    core calls ``complete()`` (the ap_done rising edge) and tests move time with ``advance``.

    Heartbeat pin (STATUS b1, kv-permit 10-14): the RTL's free-running divider gated by permit,
        hb_cnt(n)  = (n - reset) mod 2*HB_HALF          (reset = ``tb_start`` here)
        pin(n + 1) = permit(n) and hb_cnt(n) < HB_HALF and (pin(n) or hb_cnt(n) == 0)
    i.e. 1 iff permit has held since the start of the current 2*HB_HALF period and the period is
    in its first half: a 100 Hz square wave while permitted (5 ms high / 5 ms low at 100 MHz), a
    high phase only starting on the period grid, low otherwise. One simplification: the pin reads
    low from the revoking clock itself (RTL: one PL clock later). HB_HALF_CYCLES (0x48) is read.
    Not modelled: IRQ registers, CAPTURE_AGE (kept 0 by every feed).

    Window (v3fc2, 2026-10-26): ``timeout`` is the power-on value of the runtime window
    register (0x44, RTL default 1000 ms); a write there is clamped to [window_min, window_max]
    (default timeout // 10 and timeout * 5, i.e. 100 ms / 5000 ms) and disarms (cause 5 if
    it revokes), exactly as the RTL."""

    C_NONE, C_TIMEOUT, C_FORCE, C_REPLAY, C_OLD, C_WINDOW = 0, 1, 2, 3, 4, 5

    def __init__(self, timeout=100000000, tb_start=1 << 32, hb_half=500000, window_min=None,
                 window_max=None):
        self.timeout = int(timeout)
        self.hb_half = int(hb_half)
        self.window_min = self.timeout // 10 if window_min is None else int(window_min)
        self.window_max = self.timeout * 5 if window_max is None else int(window_max)
        if (self.window_min < 2 or self.hb_half < 1 or
                not self.window_min <= self.timeout <= self.window_max <= 0xFFFFFFFF):
            raise ValueError("2 <= MIN <= TIMEOUT <= MAX < 2**32 and HB_HALF >= 1 cycles "
                             "(RTL parameter check)")
        self.tb = int(tb_start)
        self.tb_reset = self.tb          # hb_cnt == 0 here (the divider's reset)
        self.permit_since = None         # first clock of the current permit-true run
        self.force_stop = False
        self.next_frame_id = 0
        self.armed = False
        self.age = self.timeout
        self.last_id, self.last_valid = 0, False
        self.accepted = self.rejected = self.revoke_count = 0
        self.last_cause = self.disarm_reason = self.C_NONE
        self.last_done_ts = self.last_revoke_ts = 0
        self.writes = []
        self.events = []            # (kind, id_now, accepted?) per completion edge
        self._hi = 0

    @property
    def permit(self):
        return self.armed and self.age < self.timeout and not self.force_stop

    @property
    def pin(self):
        """The heartbeat output register now (STATUS b1), see the class docstring."""
        if not self.permit or self.permit_since is None:
            return False
        n = self.tb - 1                                   # the last clock edge evaluated
        phase = (n - self.tb_reset) % (2 * self.hb_half)
        return phase < self.hb_half and n - phase >= self.permit_since

    def _after(self, was):
        if was and not self.permit:
            self.revoke_count += 1
            self.last_cause = (self.C_FORCE if self.force_stop else
                               self.disarm_reason if not self.armed else self.C_TIMEOUT)
            self.last_revoke_ts = self.tb

    def advance(self, cycles):
        """``cycles`` PL clocks with no completion edge (age +1 per clock, saturating). A
        timeout inside the span is stamped at its exact cycle (REVOKE_TS = DONE_TS + T + 1)."""
        cycles = int(cycles)
        was = self.permit
        t_before = self.tb
        remaining = self.timeout - self.age
        self.tb += cycles
        if self.armed:
            self.age = min(self.timeout, self.age + cycles)
        self._after(was)
        if was and not self.permit and self.last_cause == self.C_TIMEOUT:
            self.last_revoke_ts = t_before + remaining

    def complete(self):
        """One rising edge of the frame core's completion line."""
        was = self.permit
        id_now = self.next_frame_id
        id_newer = (not self.last_valid) or id_now > self.last_id
        accept = id_newer and not self.force_stop          # capture age 0 < TIMEOUT
        if id_newer:
            self.last_id, self.last_valid = id_now, True
        if accept:
            self.armed, self.age = True, 0
            self.accepted += 1
            self.last_done_ts = self.tb
        elif not id_newer:
            self.armed = False
            self.disarm_reason = self.C_REPLAY
            self.rejected += 1
        self.events.append(("accept" if accept else "reject" if not id_newer else "force",
                            id_now))
        self.tb += 1
        if self.permit and not was:
            self.permit_since = self.tb                  # armed from the next clock on
        self._after(was)

    # -- UioWindow duck type
    def rd(self, off):
        if off == 0x00:
            return LAYOUTS["permit"]["magic"]
        if off == PERMIT_CTRL:
            return int(self.force_stop)
        if off == PERMIT_NEXT_FRAME_ID:
            return self.next_frame_id
        if off == PERMIT_STATUS:
            fresh = self.armed and self.age < self.timeout
            return (int(self.permit) | int(self.pin) << 1 | int(self.armed) << 2 |
                    int(self.force_stop) << 3 |
                    int(self.last_valid) << 4 | int(fresh) << 5 | self.last_cause << 8)
        regs = {0x14: self.age, PERMIT_LAST_FRAME_ID: self.last_id, 0x1C: self.accepted,
                0x20: self.rejected, 0x24: self.revoke_count, 0x28: self.last_cause,
                0x2C: self.last_done_ts & 0xFFFFFFFF, 0x30: self.last_done_ts >> 32,
                0x34: self.last_revoke_ts & 0xFFFFFFFF, 0x38: self.last_revoke_ts >> 32,
                0x44: self.timeout, 0x48: self.hb_half, 0x54: 0,
                PERMIT_WINDOW_MIN: self.window_min, PERMIT_WINDOW_MAX: self.window_max}
        if off == PERMIT_TIMEBASE[0]:
            self._hi = self.tb >> 32
            return self.tb & 0xFFFFFFFF
        if off == PERMIT_TIMEBASE[1]:
            return self._hi
        return regs.get(off, 0) & 0xFFFFFFFF

    def wr(self, off, value):
        value = int(value) & 0xFFFFFFFF
        self.writes.append((off, value))
        was = self.permit
        if off == PERMIT_NEXT_FRAME_ID:
            self.next_frame_id = value
        elif off == PERMIT_CTRL:
            self.force_stop = bool(value & 1)
            if self.force_stop:
                self.armed = False
                self.disarm_reason = self.C_FORCE
        elif off == PERMIT_WINDOW:
            self.timeout = min(max(value, self.window_min), self.window_max)
            self.armed = False
            if not self.force_stop:
                self.disarm_reason = self.C_WINDOW
        self._after(was)

    def close(self):
        pass


class RealtimePermitModel(PermitPlModel):
    """PermitPlModel whose PL clock follows the host clock (``clk_hz``, default 100 MHz):
    every access first advances it by the elapsed time. Thread safe (reader, worker and
    TELEM threads all touch it). For host dry runs of the board probe."""

    def __init__(self, clk_hz=100000000, **kw):
        super().__init__(**kw)
        import time as _time
        self._mono, self._clk = _time.monotonic_ns, int(clk_hz)
        self._t_ns = self._mono()
        self._mlock = threading.RLock()

    def _sync(self):
        now = self._mono()
        cyc = (now - self._t_ns) * self._clk // 1000000000
        if cyc > 0:
            self._t_ns += cyc * 1000000000 // self._clk
            PermitPlModel.advance(self, cyc)

    def complete(self):
        with self._mlock:
            self._sync()
            PermitPlModel.complete(self)

    def rd(self, off):
        with self._mlock:
            self._sync()
            return PermitPlModel.rd(self, off)

    def wr(self, off, value):
        with self._mlock:
            self._sync()
            PermitPlModel.wr(self, off, value)


def mock_permit_pl(**kw):
    """(PlRegBlock("permit") over a PermitPlModel, the model)."""
    model = PermitPlModel(**kw)
    return PlRegBlock("permit", model), model
