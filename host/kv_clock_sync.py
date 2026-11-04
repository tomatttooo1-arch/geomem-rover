"""KV260 <-> Nano clock mapping (B2, 2026-10-11). See docs/design/KV_CLOCK_SYNC_SPEC_20261011.md.

What it is for: put KV260-side events (KV ``CLOCK_MONOTONIC_RAW`` times, PL timebase
counts such as the encoder frame stamps and LAST_DONE_TS) on the Nano's
``CLOCK_MONOTONIC_RAW`` axis, with an error bound, so encoder odometry, KV depth
processing instants and Nano capture instants can be aligned.

What it is NOT for: frame age. Frame age stays a round-trip difference on the
Nano's single clock (host/kv_link.py). Nothing here feeds it.

Timebase: every stamp here comes from host.fpga_clock.now_ns() (CLOCK_MONOTONIC_RAW,
never slewed by NTP); see that module and spec section 9.

Two linear maps, both fitted with the same ``IntervalLineFit``:

* KV mono -> Nano mono, from NTP-style four-timestamp exchanges on the existing
  UDP channel (message types SYNC / SYNC_ACK, same chunk framing and server
  port as the frame requests). Per exchange:
      d     = (t4 - t1) - (t3 - t2)            (network delay, both directions)
      theta = ((t2 - t1) + (t3 - t4)) / 2      (estimate of KV - Nano)
  With causal timestamps (t2 taken after the request arrived, t3 before the
  answer left) the true offset lies in theta +/- d/2 whatever the asymmetry.
  The offset is anchored on the minimum-delay sample of a sliding window, the
  drift is a least-squares slope over the window (per-bin minimum-delay
  samples), clipped into the slope interval that every pair of samples allows.
  bound(t) = d_anchor/2 + slack + slope_uncertainty * |t - t_anchor|.

* PL timebase count -> KV mono, from paired samples taken on the KV260: KV mono
  read right before and right after the 32-bit read of TIMEBASE_LO (the read
  that latches the count; HI is read after the second KV stamp). The count was
  latched inside [t_a, t_b], so kv = mid +/- width/2. Fitted as an offset
  relative to the nominal period (sysinfo CLK_HZ) plus a fitted slope, so the
  real PL clock rate (ppm from nominal) is measured, not assumed.

The PL timebase register is only touched when a UIO node of the expected name
exists AND its MAGIC register matches: bitstream 0x0117bc0a has nothing at the
placeholder address and an unmapped AXI read can hang the board. Otherwise the
PL map is reported unavailable (with the reason) in every SYNC_ACK and every
report, and only the KV-mono map is produced.

No system clock is changed and no NTP/chrony is used. Python 3.8 compatible.
"""

from collections import Counter, deque
import mmap
import os
import socket
import struct
import threading
import time

from host.fpga_clock import now_ns
from host.kv_link import CHUNK, CHUNK_FRAGMENTED, MAGIC, NUDGE, Reassembler, split

SYNC, SYNC_ACK = 8, 9          # registered in host.kv_link.TYPE_NAMES
SYNC_BODY = struct.Struct("<QQ")                 # seq, t1 (Nano mono ns)
# seq, t1 echo, t2, t3 (KV mono ns), pl_status, pl_width_ns, pl_count, pl_kv_mid_ns,
# pl_clk_hz; then the PL reason string (u16 length + utf-8).
SYNC_ACK_BODY = struct.Struct("<QQQQIIQQI")
PL_OK, PL_UNAVAILABLE, PL_READ_ERROR = 0, 1, 2
PL_STATUS_NAMES = {PL_OK: "ok", PL_UNAVAILABLE: "unavailable", PL_READ_ERROR: "read_error"}

DEFAULT_RHO_MAX = 500e-6       # assumed |rate error| bound of either crystal (prior)
# PL clock prior around the nominal CLK_HZ: pl_clk0 comes from a PS PLL divider and may sit
# well off the nominal value; only used while the window cannot bound the slope itself.
PL_RATE_PRIOR = 0.02
NS = 1000000000


class ClockSyncError(RuntimeError):
    pass


class PlTimebaseUnavailable(ClockSyncError):
    """The PL map was requested but the PL timebase could not be read."""


# ----------------------------------------------------------------- estimator
class IntervalLineFit:
    """y(x) = y_a + s * (x - x_a), from samples (x, y, h) with |y_true(x) - y| <= h.

    ``x`` are Python ints (ns or ticks; kept exact), ``y`` floats (offsets in ns,
    small after the caller removes the nominal part), ``h`` >= 0 half-widths.
    ``slope_prior`` bounds |s| when the data cannot (fewer than two usable bins).
    """

    def __init__(self, *, window_x, max_samples=256, bins=16, slope_prior=DEFAULT_RHO_MAX,
                 slope_center=0.0):
        self.window_x = int(window_x)
        self.samples = deque(maxlen=int(max_samples))
        self.bins = int(bins)
        self.slope_prior = float(slope_prior)
        self.slope_center = float(slope_center)

    def add(self, x, y, h):
        x = int(x)
        if self.samples and x < self.samples[-1][0]:
            raise ValueError("samples must arrive in x order")
        self.samples.append((x, float(y), float(h)))
        lo = x - self.window_x
        while self.samples and self.samples[0][0] < lo:
            self.samples.popleft()

    def __len__(self):
        return len(self.samples)

    def _bin_minima(self):
        s = list(self.samples)
        x0, x1 = s[0][0], s[-1][0]
        if x1 == x0:
            return [min(s, key=lambda r: r[2])]
        best = {}
        span = x1 - x0
        for r in s:
            k = min(self.bins - 1, (r[0] - x0) * self.bins // span)
            if k not in best or r[2] < best[k][2]:
                best[k] = r
        return [best[k] for k in sorted(best)]

    def fit(self):
        """dict(x_a, y_a, h_a, slope, slope_lo, slope_hi, n, n_bins, consistent, ls_slope)."""
        if not self.samples:
            raise ClockSyncError("no valid samples in the window")
        anchor = min(self.samples, key=lambda r: r[2])
        x_a, y_a, h_a = anchor
        pts = self._bin_minima()
        lo_b = self.slope_center - self.slope_prior
        hi_b = self.slope_center + self.slope_prior
        ls = None
        if len(pts) >= 2:
            xs = [float(p[0] - x_a) for p in pts]          # centred: exact int subtraction first
            ys = [p[1] - y_a for p in pts]
            mx = sum(xs) / len(xs)
            my = sum(ys) / len(ys)
            sxx = sum((u - mx) ** 2 for u in xs)
            if sxx > 0:
                ls = sum((u - mx) * (v - my) for u, v in zip(xs, ys)) / sxx
            for i in range(len(pts)):
                for j in range(i + 1, len(pts)):
                    dx = float(pts[j][0] - pts[i][0])
                    if dx <= 0:
                        continue
                    lo_b = max(lo_b, ((pts[j][1] - pts[j][2]) - (pts[i][1] + pts[i][2])) / dx)
                    hi_b = min(hi_b, ((pts[j][1] + pts[j][2]) - (pts[i][1] - pts[i][2])) / dx)
        consistent = lo_b <= hi_b
        slope = self.slope_center if ls is None else ls
        if consistent:
            slope = min(max(slope, lo_b), hi_b)
        return {"x_a": x_a, "y_a": y_a, "h_a": h_a, "slope": slope, "slope_lo": lo_b,
                "slope_hi": hi_b, "ls_slope": ls, "n": len(self.samples), "n_bins": len(pts),
                "consistent": consistent, "x_first": self.samples[0][0],
                "x_last": self.samples[-1][0]}


def _eval(fit, x):
    """(y, bound) at x; raises when the window's samples contradict a constant rate."""
    if not fit["consistent"]:
        raise ClockSyncError("samples contradict a constant-rate model "
                             "(slope interval empty): bound cannot be stated")
    dx = float(int(x) - fit["x_a"])
    ds = max(fit["slope"] - fit["slope_lo"], fit["slope_hi"] - fit["slope"])
    return fit["y_a"] + fit["slope"] * dx, fit["h_a"] + ds * abs(dx)


class KvNanoMap:
    """offset(t_nano) = kv - nano, fitted from four-timestamp exchanges."""

    def __init__(self, fit):
        self.fit = fit

    def offset_at_nano(self, nano_ns):
        return _eval(self.fit, nano_ns)

    def kv_to_nano(self, kv_ns):
        """(nano_ns as float, bound_ns). Solves kv = t + y_a + s (t - x_a)."""
        f = self.fit
        s = f["slope"]
        u = float(int(kv_ns) - f["x_a"]) - f["y_a"]
        dt = u / (1.0 + s)
        t = f["x_a"] + dt
        _, b = _eval(f, int(round(t)))
        # an error e in the kv-side offset moves t by e / (1 + s)
        return t, b / (1.0 - abs(s))

    def nano_to_kv(self, nano_ns):
        y, b = _eval(self.fit, nano_ns)
        return int(nano_ns) + y, b

    def describe(self):
        f = self.fit
        return {"offset_ns_at_anchor": f["y_a"], "anchor_nano_ns": f["x_a"],
                "anchor_rtt_half_ns": f["h_a"], "drift_ppm": f["slope"] * 1e6,
                "drift_ppm_interval": [f["slope_lo"] * 1e6, f["slope_hi"] * 1e6],
                "ls_drift_ppm": None if f["ls_slope"] is None else f["ls_slope"] * 1e6,
                "samples": f["n"], "bins": f["n_bins"], "consistent": f["consistent"],
                "window_nano_ns": [f["x_first"], f["x_last"]]}


class PlKvMap:
    """kv = count * 1e9 / clk_hz + y(count); y fitted (offset + rate deviation)."""

    def __init__(self, fit, clk_hz):
        self.fit, self.clk_hz = fit, int(clk_hz)

    def nominal_kv(self, count):
        return count * NS / self.clk_hz          # float, ~1e14 ns, 0.02 ns resolution

    def pl_to_kv(self, count):
        y, b = _eval(self.fit, count)
        return self.nominal_kv(int(count)) + y, b

    def describe(self):
        f = self.fit
        per_tick = NS / self.clk_hz
        return {"clk_hz_nominal": self.clk_hz, "anchor_count": f["x_a"],
                "anchor_kv_minus_nominal_ns": f["y_a"], "anchor_halfwidth_ns": f["h_a"],
                "rate_dev_ppm": f["slope"] / per_tick * 1e6,
                "rate_dev_ppm_interval": [f["slope_lo"] / per_tick * 1e6,
                                          f["slope_hi"] / per_tick * 1e6],
                "samples": f["n"], "bins": f["n_bins"], "consistent": f["consistent"]}


def exchange_sample(t1, t2, t3, t4, rho_max=DEFAULT_RHO_MAX):
    """(x_nano, theta, half) of one exchange, or (None, reason) if not causal."""
    t1, t2, t3, t4 = int(t1), int(t2), int(t3), int(t4)
    hold = t3 - t2
    span = t4 - t1
    if hold < 0:
        return None, "kv_hold_negative"
    if span < 0:
        return None, "nano_span_negative"
    d = span - hold
    if d < 0:
        return None, "delay_negative"
    theta = ((t2 - t1) + (t3 - t4)) / 2.0
    # within-exchange rate error: hold is measured on the KV clock, span on the Nano's
    half = d / 2.0 + rho_max * (span + hold)
    return ((t1 + t4) // 2, theta, half), None


def pl_pair_sample(count, kv_mid_ns, width_ns, clk_hz):
    """(x=count, y=kv_mid - nominal(count), half) for IntervalLineFit."""
    count, kv_mid_ns, clk_hz = int(count), int(kv_mid_ns), int(clk_hz)
    y = (kv_mid_ns * clk_hz - count * NS) / float(clk_hz)   # exact ints until the last division
    # the latched count c means the latch instant lies in [time(c), time(c + 1)): one
    # tick of quantization on top of the bracket; +1 ns for the integer mid rounding
    return count, y, width_ns / 2.0 + NS / float(clk_hz) + 1.0


# ----------------------------------------------------------------- PL timebase (KV260)
PL_TIMEBASE_BLOCKS = {
    # B3/B4 placeholder address table (hw/m7/pl/bd/drafts/permit_uart_integration_20261011.patch,
    # docs/design/PL_PERMIT_UART_SPEC_20261011.md 3.3 / 4.4). LO read latches HI.
    "permit": {"uio": "m7_permit", "base": 0xA0060000, "magic": 0x4D37504D, "lo": 0x4C, "hi": 0x50},
    "encoder": {"uio": "m7_encoder", "base": 0xA0070000, "magic": 0x4D37454E, "lo": 0xB0, "hi": 0xB4},
}
DEFAULT_PL_CLK_HZ = 100000000


class UioRegisters:
    """32-bit register window of a generic-uio node (map0), mmap of /dev/uioN.

    Reads go through a uint32 memoryview so each is one 32-bit load (a wider
    or byte-wise copy of TIMEBASE_LO/HI would break the latch-on-LO rule).
    """

    def __init__(self, path, size):
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_SYNC)
        try:
            self.mm = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, 0)
        except Exception:
            os.close(self.fd)
            raise
        self.words = memoryview(self.mm).cast("I")

    def rd(self, off):
        return int(self.words[off >> 2])

    def close(self):
        self.words.release()
        self.mm.close()
        os.close(self.fd)


def _read_sysfs_int(path):
    with open(path) as fp:
        return int(fp.read().strip(), 0)


class PlTimebase:
    """Paired (PL count, KV mono) sampler. ``available`` False => degraded, see ``reason``."""

    def __init__(self, regs=None, *, block="permit", clk_hz=DEFAULT_PL_CLK_HZ,
                 clk_source="default", reason=None, clock=now_ns, detail=None):
        self.block = block
        self.layout = PL_TIMEBASE_BLOCKS[block]
        self.regs = regs
        self.clk_hz = int(clk_hz)
        self.clk_source = clk_source
        self.clock = clock
        self.available = regs is not None and reason is None
        self.reason = reason if reason is not None else ("" if regs is not None else "no registers")
        self.detail = dict(detail or {})
        self.counts = Counter()
        self._last_count = None
        # LO latches HI for the whole PL block: every reader of the block in this
        # process must hold the same lock across its LO..HI pair (PC_DATAPLANE A.3).
        self.lock = threading.Lock()

    @classmethod
    def from_block(cls, reg_block, *, clk_hz=None, clock=time.monotonic_ns):
        """Pair over an already-open host.kv_regs.PlRegBlock (permit/encoder), sharing
        its window and its lock with every other user of that block."""
        clk_source = "argument" if clk_hz else "default %d Hz (pass the sysinfo CLK_HZ)" % DEFAULT_PL_CLK_HZ
        clk_hz = int(clk_hz or DEFAULT_PL_CLK_HZ)
        if not reg_block.available:
            return cls(None, block=reg_block.name, clk_hz=clk_hz, clk_source=clk_source,
                       reason=reg_block.reason, clock=clock, detail=reg_block.detail)
        tb = cls(reg_block.window, block=reg_block.name, clk_hz=clk_hz, clk_source=clk_source,
                 clock=clock, detail=reg_block.detail)
        tb.lock = reg_block.lock
        return tb

    @classmethod
    def open(cls, block="permit", *, sysfs_root="/sys/class/uio", dev_root="/dev",
             clk_hz=None, clock=now_ns):
        """Gate: UIO node of the expected name -> its map0 address -> mmap -> MAGIC.

        Never reads a raw /dev/mem address: an unassigned PL address can hang the AXI bus.
        Any failure returns a degraded instance (``available`` False) with the reason.
        """
        from host.m7_pl_device import find_uio
        lay = PL_TIMEBASE_BLOCKS[block]
        clk_source = "argument" if clk_hz else "default %d Hz (pass the sysinfo CLK_HZ)" % DEFAULT_PL_CLK_HZ
        clk_hz = int(clk_hz or DEFAULT_PL_CLK_HZ)

        def degraded(reason, **detail):
            return cls(None, block=block, clk_hz=clk_hz, clk_source=clk_source,
                       reason=reason, clock=clock, detail=detail)
        try:
            n = find_uio(lay["uio"], sysfs_root)
        except Exception as exc:
            return degraded("uio lookup failed: %s" % exc)
        if n is None:
            return degraded("no UIO node named %r under %s (bitstream without the %s block?)"
                            % (lay["uio"], sysfs_root, block))
        mdir = os.path.join(sysfs_root, "uio%d" % n, "maps", "map0")
        try:
            addr = _read_sysfs_int(os.path.join(mdir, "addr"))
            size = _read_sysfs_int(os.path.join(mdir, "size"))
        except (OSError, ValueError) as exc:
            return degraded("uio%d map0 unreadable: %s" % (n, exc))
        detail = {"uio": "uio%d" % n, "map_addr": hex(addr), "map_size": hex(size),
                  "placeholder_base": hex(lay["base"])}
        if size < lay["hi"] + 4:
            return degraded("uio%d map0 size %#x too small" % (n, size), **detail)
        try:
            regs = UioRegisters(os.path.join(dev_root, "uio%d" % n), size)
        except OSError as exc:
            return degraded("mmap %s/uio%d failed: %s" % (dev_root, n, exc), **detail)
        magic = regs.rd(0)
        detail["magic"] = hex(magic)
        if magic != lay["magic"]:
            regs.close()
            return degraded("MAGIC %#010x != %#010x at uio%d" % (magic, lay["magic"], n), **detail)
        if addr != lay["base"]:
            detail["note"] = "map0 address differs from the placeholder table; used the UIO map"
        return cls(regs, block=block, clk_hz=clk_hz, clk_source=clk_source, clock=clock,
                   detail=detail)

    def sample(self):
        """One paired sample: (count, kv_mid_ns, width_ns). LO latches HI, so only LO
        is bracketed; HI is read after the second stamp."""
        if not self.available:
            raise PlTimebaseUnavailable(self.reason)
        lay = self.layout
        with self.lock:
            t_a = self.clock()
            lo = self.regs.rd(lay["lo"])
            t_b = self.clock()
            hi = self.regs.rd(lay["hi"])
        return (hi << 32) | lo, (t_a + t_b) // 2, t_b - t_a

    def sample_best(self, n=3):
        """Narrowest bracket of n samples (a preempted read is discarded, not averaged).
        Raises ClockSyncError when counts go backwards (latch raced by another reader?)."""
        best = None
        prev = self._last_count
        for _ in range(int(n)):
            s = self.sample()
            if prev is not None and s[0] <= prev:
                self.counts["non_monotonic"] += 1
                raise ClockSyncError("PL timebase went from %d to %d (another reader of "
                                     "TIMEBASE_LO between our LO and HI reads?)" % (prev, s[0]))
            prev = s[0]
            if best is None or s[2] < best[2]:
                best = s
        self._last_count = prev
        self.counts["samples"] += 1
        return best

    def describe(self):
        return {"block": self.block, "available": self.available, "reason": self.reason,
                "clk_hz": self.clk_hz, "clk_source": self.clk_source, "detail": self.detail,
                "counts": dict(self.counts)}


class MockPlTimebase(PlTimebase):
    """Host stand-in: count = (kv_true - reset) * clk * (1 + ppm); latch inside the bracket."""

    def __init__(self, *, clk_hz=DEFAULT_PL_CLK_HZ, ppm=0.0, reset_kv_ns=0, clock=now_ns,
                 latch_frac=None, block="permit"):
        super().__init__(regs=None, block=block, clk_hz=clk_hz, clk_source="mock", clock=clock,
                         reason="")
        self.available = True
        self.reason = ""
        self.ppm, self.reset_kv_ns = float(ppm), int(reset_kv_ns)
        self.latch_frac = latch_frac

    def true_count(self, kv_ns):
        return int((kv_ns - self.reset_kv_ns) * self.clk_hz * (1 + self.ppm * 1e-6) // NS)

    def sample(self):
        t_a = self.clock()
        t_b = self.clock()
        f = 0.5 if self.latch_frac is None else self.latch_frac()
        return self.true_count(t_a + f * (t_b - t_a)), (t_a + t_b) // 2, t_b - t_a


# ----------------------------------------------------------------- KV260 responder
class SyncResponder:
    """Answers SYNC inline on the KV server's reader thread.

    ``t2`` is the KV mono time taken right after ``recvfrom`` returned; ``t3`` is
    taken last, right before the answer is packed and sent. The PL pair is
    sampled between them (that time is excluded from the delay ``d``).
    """

    def __init__(self, pl=None, clock=now_ns, pl_samples=3):
        self.pl = pl
        self.clock = clock
        self.pl_samples = int(pl_samples)
        self.counts = Counter()

    def pl_reason(self):
        if self.pl is None:
            return "PL timebase not configured on the server (--pl-timebase none)"
        return self.pl.reason

    def answer(self, payload, t2):
        try:
            seq, t1 = SYNC_BODY.unpack_from(payload)
        except struct.error:
            self.counts["sync_malformed"] += 1
            return None
        status, count, mid, width, reason = PL_UNAVAILABLE, 0, 0, 0, self.pl_reason()
        clk = self.pl.clk_hz if self.pl is not None else 0
        if self.pl is not None and self.pl.available:
            try:
                count, mid, width = self.pl.sample_best(self.pl_samples)
                status, reason = PL_OK, ""
            except ClockSyncError as exc:
                status, reason = PL_READ_ERROR, str(exc)
                self.counts["pl_read_error"] += 1
        self.counts["sync_answered"] += 1
        self.counts["pl_%s" % PL_STATUS_NAMES[status]] += 1
        raw = reason.encode("utf-8")[:4000]
        t3 = self.clock()
        return SYNC_ACK_BODY.pack(seq, t1, int(t2), int(t3), status, min(int(width), 0xFFFFFFFF),
                                  count, mid, clk) + struct.pack("<H", len(raw)) + raw


def decode_sync_ack(buf):
    (seq, t1, t2, t3, status, width, count, mid, clk) = SYNC_ACK_BODY.unpack_from(buf)
    (n,) = struct.unpack_from("<H", buf, SYNC_ACK_BODY.size)
    off = SYNC_ACK_BODY.size + 2
    reason = bytes(buf[off:off + n]).decode("utf-8", "replace")
    return {"seq": seq, "t1": t1, "t2": t2, "t3": t3, "pl_status": status, "pl_width_ns": width,
            "pl_count": count, "pl_kv_mid_ns": mid, "pl_clk_hz": clk, "pl_reason": reason}


# ----------------------------------------------------------------- Nano side
class ClockSync:
    """Accumulates exchanges and PL pairs; produces the maps. Transport-free (tests feed it)."""

    def __init__(self, *, window_s=60.0, bins=16, rho_max=DEFAULT_RHO_MAX, max_samples=2048):
        self.rho_max = float(rho_max)
        self.kv = IntervalLineFit(window_x=int(window_s * NS), bins=bins, slope_prior=rho_max,
                                  max_samples=max_samples)
        self.pl = None             # IntervalLineFit over PL counts, created at the first PL pair
        self.pl_clk_hz = None
        self.window_s, self.bins, self.max_samples = float(window_s), int(bins), int(max_samples)
        self.counts = Counter()
        self.pl_state = {"status": None, "reason": "no SYNC_ACK yet"}
        self.thetas = deque(maxlen=4096)     # (x, theta, half) of every valid exchange, for jitter

    def add_exchange(self, t1, t2, t3, t4):
        s, why = exchange_sample(t1, t2, t3, t4, self.rho_max)
        if s is None:
            self.counts["rejected:%s" % why] += 1
            return False
        self.kv.add(*s)
        self.thetas.append(s)
        self.counts["exchanges_valid"] += 1
        return True

    def add_pl_pair(self, count, kv_mid_ns, width_ns, clk_hz):
        clk_hz = int(clk_hz)
        if self.pl is None or self.pl_clk_hz != clk_hz:
            if self.pl is not None:
                self.counts["pl_clk_changed"] += 1
            self.pl_clk_hz = clk_hz
            per_tick = NS / float(clk_hz)
            # window in counts ~ window_s; slope in ns/tick, prior around nominal
            self.pl = IntervalLineFit(window_x=int(self.window_s * clk_hz), bins=self.bins,
                                      slope_prior=max(self.rho_max, PL_RATE_PRIOR) * per_tick,
                                      max_samples=self.max_samples)
        x, y, h = pl_pair_sample(count, kv_mid_ns, width_ns, clk_hz)
        if self.pl.samples and x <= self.pl.samples[-1][0]:
            self.counts["pl_rejected:non_monotonic"] += 1
            return False
        self.pl.add(x, y, h)
        self.counts["pl_pairs_valid"] += 1
        return True

    def add_ack(self, ack, t4):
        ok = self.add_exchange(ack["t1"], ack["t2"], ack["t3"], t4)
        st = ack["pl_status"]
        self.pl_state = {"status": PL_STATUS_NAMES.get(st, st), "reason": ack["pl_reason"]}
        self.counts["pl_status:%s" % PL_STATUS_NAMES.get(st, st)] += 1
        if st == PL_OK:
            self.add_pl_pair(ack["pl_count"], ack["pl_kv_mid_ns"], ack["pl_width_ns"],
                             ack["pl_clk_hz"])
        return ok

    def kv_map(self):
        return KvNanoMap(self.kv.fit())

    def pl_map(self):
        if self.pl is None or not len(self.pl):
            raise PlTimebaseUnavailable("PL timebase map unavailable: %s (%s)"
                                        % (self.pl_state["status"], self.pl_state["reason"]))
        return PlKvMap(self.pl.fit(), self.pl_clk_hz)

    def kv_to_nano(self, kv_ns):
        return self.kv_map().kv_to_nano(kv_ns)

    def pl_to_nano(self, count):
        """(nano_ns, bound_ns) of a PL timebase count, via KV mono. Raises
        PlTimebaseUnavailable (explicitly) when the PL map is missing."""
        kv, b1 = self.pl_map().pl_to_kv(count)
        nano, b2 = self.kv_map().kv_to_nano(int(round(kv)))
        return nano, b1 + b2 + 1.0

    def report(self):
        out = {"counts": dict(self.counts), "pl_state": dict(self.pl_state), "rho_max": self.rho_max,
               "window_s": self.window_s}
        try:
            out["kv_map"] = self.kv_map().describe()
        except ClockSyncError as exc:
            out["kv_map"] = {"error": str(exc)}
        try:
            out["pl_map"] = self.pl_map().describe()
        except ClockSyncError as exc:
            out["pl_map"] = {"error": str(exc), "degraded": "only KV mono -> Nano mono is available"}
        if self.thetas:
            th = sorted(t[1] for t in self.thetas)
            ds = sorted(2 * t[2] for t in self.thetas)
            out["exchange_theta_ns"] = {"n": len(th), "min": th[0], "median": th[len(th) // 2],
                                        "max": th[-1]}
            out["exchange_bound_2h_ns"] = {"min": ds[0], "median": ds[len(ds) // 2], "max": ds[-1]}
        return out


class ClockSyncClient:
    """Nano end: own socket and session on the KV server's port (frame calls untouched)."""

    def __init__(self, server, *, sync=None, timeout_s=0.5, session=None, sock=None,
                 clock=now_ns):
        self.server = (server[0], int(server[1]))
        self.sync = sync or ClockSync()
        self.timeout_s = float(timeout_s)
        self.session = int.from_bytes(os.urandom(8), "little") if session is None else int(session)
        self.clock = clock
        self.sock = sock or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if sock is None:
            self.sock.bind(("0.0.0.0", 0))
        self.rx = Reassembler(self.sync.counts, prefix="rx_")
        self.seq = 0
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def exchange(self):
        """One four-timestamp exchange. Returns the ack dict (with t4) or None if lost."""
        self.seq += 1
        seq = self.seq
        t1 = self.clock()
        for d in split(SYNC, self.session, seq, SYNC_BODY.pack(seq, t1), CHUNK_FRAGMENTED):
            self.sock.sendto(d, self.server)
        self.sync.counts["sent"] += 1
        deadline = time.monotonic() + self.timeout_s
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                self.sync.counts["lost"] += 1
                return None
            self.sock.settimeout(left)
            try:
                dgram, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                self.sync.counts["lost"] += 1
                return None
            t4 = self.clock()
            if len(dgram) >= CHUNK.size and dgram[:4] == MAGIC and dgram[4] == NUDGE:
                self.sync.counts["nudge"] += 1      # KV260 tail release, not an answer
                continue
            got = self.rx.feed(dgram)
            if got is None:
                continue
            mtype, _, session, rid, payload, _ = got
            if session != self.session or mtype != SYNC_ACK:
                self.sync.counts["unexpected"] += 1
                continue
            try:
                ack = decode_sync_ack(payload)
            except (struct.error, ValueError):
                self.sync.counts["malformed"] += 1
                continue
            if rid != seq or ack["seq"] != seq or ack["t1"] != t1:
                self.sync.counts["stale_ack"] += 1
                self.rx.forget_done(SYNC_ACK)
                continue
            ack["t4"] = t4
            with self._lock:
                self.sync.add_ack(ack, t4)
            return ack

    def run(self, n, interval_s):
        acks = []
        for i in range(int(n)):
            acks.append(self.exchange())
            if i + 1 < n:
                time.sleep(interval_s)
        return acks

    def start(self, hz):
        """Background exchanges at ``hz`` (for use beside the frame client)."""
        def loop():
            period = 1.0 / float(hz)
            while not self._stop.wait(period):
                try:
                    self.exchange()
                except Exception as exc:     # never die silently; counted and visible in report()
                    self.sync.counts["loop_error:%s" % type(exc).__name__] += 1
        self._thread = threading.Thread(target=loop, name="clock-sync", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def report(self):
        with self._lock:
            return self.sync.report()

    def close(self):
        self.stop()
        self.sock.close()


__all__ = ["SYNC", "SYNC_ACK", "ClockSync", "ClockSyncClient", "ClockSyncError", "IntervalLineFit",
           "KvNanoMap", "MockPlTimebase", "PlKvMap", "PlTimebase", "PlTimebaseUnavailable",
           "SyncResponder", "decode_sync_ack", "exchange_sample", "pl_pair_sample"]
