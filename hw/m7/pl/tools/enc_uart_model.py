#!/usr/bin/env python3
"""Encoder-UART frame format, normative odometry model and XSim stimulus/checker.

This file is the reference definition for the PL block m7_enc_uart_axil:
  * encode_frame(): the 15-byte MCU frame (also used by mcu_frame_sim.py);
  * odom_k_q16() / Odom: the fixed-point wheel speed / odometry arithmetic,
    written independently of the RTL, bit-exact by construction;
  * ParserModel: the receive-side counters predicted from the stimulus;
  * `gen`: writes the stimulus for tb_m7_enc_uart.sv and the expected results;
  * `check`: compares the bench log with the expectation (exit 0 = PASS).

Usage:
  enc_uart_model.py gen   <dir>
  enc_uart_model.py check <dir>
  enc_uart_model.py selftest
"""
import json
import math
import random
import sys
from pathlib import Path

# ---------------------------------------------------------------- frame format
SYNC = (0xAA, 0x55)
FRAME_LEN = 15
PI_Q30 = 3373259426           # round(pi * 2^30)
assert PI_Q30 == round(math.pi * (1 << 30))
U32 = (1 << 32) - 1
S31_MAX = (1 << 31) - 1


def encode_frame(seq, mcu_ms, left, right, flags, corrupt_checksum=False):
    body = bytes([seq & 0xFF]) + (mcu_ms & 0xFFFF).to_bytes(2, "little") \
        + (left & U32).to_bytes(4, "little") + (right & U32).to_bytes(4, "little") \
        + bytes([flags & 0xFF])
    ck = sum(body) & 0xFF
    if corrupt_checksum:
        ck ^= 0x5A
    return bytes(SYNC) + body + bytes([ck])


def s32(v):
    v &= U32
    return v - (1 << 32) if v & 0x80000000 else v


def s48(v):
    v &= (1 << 48) - 1
    return v - (1 << 48) if v & (1 << 47) else v


def trunc_half(v):
    """(v / 2) truncated toward zero."""
    return -((-v) >> 1) if v < 0 else v >> 1


# ---------------------------------------------------------------- odometry
def odom_k_q16(diam_um, cpr):
    """um per count in Q16.16, rounded; returns (k, param_err)."""
    if cpr == 0:
        return 0, True
    q = (PI_Q30 * diam_um + (cpr << 13)) // (cpr << 14)
    if q > U32:
        return U32, True
    return q, diam_um == 0


class Odom:
    def __init__(self, clk_khz, max_dt_default):
        self.clk_khz = clk_khz
        self.diam = self.cpr = self.track = 0
        self.max_delta = 32767
        self.max_dt = max_dt_default
        self.k = 0
        self.param_valid = False
        self.param_err = False
        self.clear()

    def clear(self):
        self.have_prev = False
        self.prev_l = self.prev_r = 0
        self.prev_ts = 0
        self.cum_l = self.cum_r = 0

    def apply(self):
        self.k, self.param_err = odom_k_q16(self.diam, self.cpr)
        self.param_valid = True

    def write(self, reg, val):
        if reg == 0x08: self.diam = val
        elif reg == 0x0C: self.cpr = val
        elif reg == 0x10: self.track = val
        elif reg == 0x14: self.max_delta = val
        elif reg == 0x18: self.max_dt = val
        elif reg == 0x04:
            if val & 2: self.clear()
            if val & 1: self.apply()

    def frame(self, left, right, ts):
        first = not self.have_prev
        dl = s32(left - self.prev_l)
        dr = s32(right - self.prev_r)
        dt = (ts - self.prev_ts) & ((1 << 64) - 1)
        jump = (not first) and (abs(dl) > self.max_delta or abs(dr) > self.max_delta)
        dt_long = (not first) and dt > self.max_dt
        speed_ok = not first and not jump and not dt_long
        if not first and not jump:
            self.cum_l = s48(self.cum_l + dl)
            self.cum_r = s48(self.cum_r + dr)
        self.have_prev = True
        self.prev_l, self.prev_r, self.prev_ts = left & U32, right & U32, ts

        def dist(c):
            m = (abs(c) * self.k) >> 16
            return -m if c < 0 else m

        def speed(d):
            if not speed_ok:
                return 0
            m = min(S31_MAX, (abs(d) * self.k * self.clk_khz) // dt)
            return -m if d < 0 else m

        dist_l, dist_r = dist(self.cum_l), dist(self.cum_r)
        v_l, v_r = speed(dl), speed(dr)
        if self.track == 0:
            head, head_bad = 0, True
        else:
            diff = dist_r - dist_l
            m = min(S31_MAX, (abs(diff) << 16) // self.track)
            head, head_bad = (-m if diff < 0 else m), False
        oflags = (int(dt_long) << 5) | (int(self.param_valid) << 4) | (int(head_bad) << 3) \
            | (int(jump) << 2) | (int(first) << 1) | int(speed_ok)
        return dict(v_l=v_l, v_r=v_r, v_c=trunc_half(v_l + v_r),
                    dist_l=dist_l, dist_r=dist_r, dist_c=trunc_half(dist_l + dist_r),
                    heading=head, oflags=oflags,
                    dt=0 if first else min(dt, U32),
                    cum_l=self.cum_l, cum_r=self.cum_r, jump=jump)


# ---------------------------------------------------------------- simulation config
SIM = dict(
    CLK_HZ=11_520_000,          # 100 cycles per bit at 115200 baud
    BAUD=115200,
    BYTE_GAP=11_520,            # 1 ms
    LINK_TO=576_000,            # 50 ms
    MAX_DT=1_152_000,           # 100 ms
)
SIM["DIV"] = (SIM["CLK_HZ"] + SIM["BAUD"] // 2) // SIM["BAUD"]
SIM["CLK_KHZ"] = SIM["CLK_HZ"] // 1000
PERIOD = SIM["CLK_HZ"] // 100   # 10 ms frame period


class ParserModel:
    """Byte-level model of the receive FSM, fed with the scheduled byte stream."""

    def __init__(self):
        self.c = dict(good=0, bad=0, gapev=0, lost=0, dup=0, partial=0, framing=0,
                      skip=0, linkto=0, glitch=0, jump=0)
        self.state = "H1"
        self.buf = []
        self.last_seq = None
        self.frames = []        # non-duplicate good frames, in order
        self.last_good_t = None

    def gap(self, idle_cycles):
        # inter-byte idle; generator keeps gaps either tiny or >> BYTE_GAP
        if self.state != "H1" and idle_cycles > SIM["BYTE_GAP"]:
            self.state = "H1"
            self.c["partial"] += 1

    def framing_error(self):
        self.c["framing"] += 1
        self.state = "H1"

    def byte(self, b, t_end):
        st = self.state
        if st == "H1":
            if b == 0xAA: self.state = "H2"
            else: self.c["skip"] += 1
        elif st == "H2":
            if b == 0x55:
                self.state, self.buf = "BODY", []
            elif b == 0xAA:
                self.c["skip"] += 1
            else:
                self.state = "H1"; self.c["skip"] += 2
        elif st == "BODY":
            self.buf.append(b)
            if len(self.buf) == 12: self.state = "CK"
        else:
            self.state = "H1"
            if b == (sum(self.buf) & 0xFF):
                self.c["good"] += 1
                if self.last_good_t is not None and t_end - self.last_good_t > SIM["LINK_TO"]:
                    self.c["linkto"] += 1
                self.last_good_t = t_end
                seq = self.buf[0]
                if self.last_seq is not None and seq == self.last_seq:
                    self.c["dup"] += 1
                else:
                    if self.last_seq is not None and ((seq - self.last_seq) & 0xFF) != 1:
                        self.c["gapev"] += 1
                        self.c["lost"] += ((seq - self.last_seq) & 0xFF) - 1
                    bb = bytes(self.buf)
                    self.frames.append(dict(
                        seq=seq, mcu_ms=int.from_bytes(bb[1:3], "little"),
                        left=int.from_bytes(bb[3:7], "little"),
                        right=int.from_bytes(bb[7:11], "little"), flags=bb[11]))
                self.last_seq = seq
            else:
                self.c["bad"] += 1

    def end(self, t_end):
        if self.last_good_t is not None and t_end - self.last_good_t > SIM["LINK_TO"]:
            self.c["linkto"] += 1


# ---------------------------------------------------------------- stimulus
def build_stimulus(seed=20261011):
    """Returns (events, model). Event tuples:
    ('B', idle, bit_cycles, byte, stop_ok) | ('G', idle, low_cycles) | ('W', idle, addr, data) | ('E', idle)
    """
    rnd = random.Random(seed)
    DIV = SIM["DIV"]
    ev = []
    t = 0                       # schedule time (cycles) for link-timeout prediction
    pm = ParserModel()
    order = []                  # ('F', idx) frames and ('W', reg, val) writes, in order
    frame_end_byte = []         # byte index (in B-event order) that completed each odometry frame
    nbytes = 0

    def put_bytes(data, first_idle, bit=DIV, stop_bad_at=None):
        nonlocal t, nbytes
        for i, b in enumerate(data):
            bidx = nbytes
            nbytes += 1
            idle = first_idle if i == 0 else 0
            stop_ok = stop_bad_at != i
            ev.append(("B", idle, bit, b, int(stop_ok)))
            pm.gap(idle)
            t += idle + 10 * bit
            if stop_ok:
                nf = len(pm.frames)
                pm.byte(b, t)
                if len(pm.frames) > nf:
                    order.append(("F", nf))
                    frame_end_byte.append(bidx)
            else:
                pm.framing_error()
                ev.append(("B_IDLE_AFTER_BREAK",))
                t += bit

    def write(reg, val, idle=200):
        nonlocal t
        ev.append(("W", idle, reg, val))
        t += idle
        order.append(("W", reg, val))

    # wheel state (cumulative int32 counts, wrap naturally)
    st = dict(seq=0, ms=65400, left=(1 << 31) - 3000, right=-(1 << 31) + 2000, flags=3)

    def advance(vl, vr, n=1):
        st["left"] = (st["left"] + vl * n) & U32
        st["right"] = (st["right"] + vr * n) & U32
        st["ms"] = (st["ms"] + 10 * n) & 0xFFFF

    def send(idle=PERIOD - 15 * DIV, bit=DIV, corrupt=False, cut=None, stop_bad_at=None,
             prefix=b"", seq=None, flags=None):
        f = encode_frame(st["seq"] if seq is None else seq, st["ms"], s32(st["left"]),
                         s32(st["right"]), st["flags"] if flags is None else flags, corrupt)
        data = prefix + (f if cut is None else f[:cut])
        put_bytes(data, idle, bit, stop_bad_at)
        return f

    last = {}

    def step(vl, vr, **kw):
        f = send(**kw)
        last["frame"] = f
        st["seq"] = (st["seq"] + 1) & 0xFF
        advance(vl, vr)
        return f

    write(0x08, 65000); write(0x0C, 1320); write(0x10, 160000); write(0x04, 1)
    # 1. normal: accelerate forward with a turn (left wraps past +2^31)
    for i in range(40):
        step(10 + i, 8 + i, flags=rnd.choice([1, 3]))
    # 2. bad checksum, then good (seq gap 1)
    step(50, 48, corrupt=True)
    step(50, 48)
    # 3. half frame then idle, next frame
    step(50, 48, cut=7)
    step(50, 48)
    # 4. junk bytes back-to-back before a frame (byte resync)
    step(50, 48, prefix=bytes([0x12, 0xAA, 0xAA, 0x37, 0xAA]))
    # 5. seq jump: MCU frames 45..49 never arrive
    for _ in range(5):
        st["seq"] = (st["seq"] + 1) & 0xFF
        advance(50, 48)
    for i in range(10):
        step(50 - 5 * i, 48 - 6 * i)
    # 6. duplicate (same frame re-sent)
    put_bytes(last["frame"], PERIOD - 15 * DIV)
    # 7. framing error inside a frame, then normal
    step(0, -2, stop_bad_at=5)
    step(-5, -9)
    # 8. glitch on idle line (shorter than half a bit)
    ev.append(("G", 3000, DIV // 5)); t += 3000; pm.c["glitch"] += 1
    # 9. reverse with a turn; seq wraps 255 -> 0
    for i in range(140):
        step(-20 - (i % 7), -35 + (i % 11))
    # 10. baud mismatch +-2 %
    for i in range(4):
        step(15, 15, bit=DIV + 2)
    for i in range(4):
        step(15, 15, bit=DIV - 2)
    # 11. parameter change mid-stream (diameter 66 mm)
    write(0x08, 66000); write(0x04, 1)
    for i in range(10):
        step(30, 10)
    # 12. long silence: > MAX_DT (and > LINK_TO) -> dt_long + link timeout
    st_idle = 13 * PERIOD
    for _ in range(12):
        st["seq"] = (st["seq"] + 1) & 0xFF
        advance(20, 20)
    step(20, 20, idle=st_idle)
    for i in range(5):
        step(20, 20)
    # 13. MCU reset: seq restarts at 0, counts jump to 0 -> jump, no accumulate
    st["seq"], st["left"], st["right"] = 0, 0, 0
    for i in range(6):
        step(25, 25)
    # 14. heading with TRACK = 0 (flag), then restore
    write(0x10, 0)
    for i in range(3):
        step(25, 30)
    write(0x10, 160000)
    # 15. clear odometry: next frame is a first frame again
    write(0x04, 2)
    for i in range(8):
        step(-12, 12)
    # end: stay idle beyond LINK_TO
    end_idle = 2 * SIM["LINK_TO"]
    ev.append(("E", end_idle))
    t += end_idle
    pm.end(t)
    return ev, pm, order, frame_end_byte


def write_stim(path, ev):
    with open(path, "w") as fh:
        for e in ev:
            if e[0] == "B":
                fh.write("B %d %d %02x %d\n" % e[1:])
            elif e[0] == "B_IDLE_AFTER_BREAK":
                pass            # the bench always releases the line for one bit after a bad stop bit
            elif e[0] == "G":
                fh.write("G %d %d 00 0\n" % e[1:])
            elif e[0] == "W":
                fh.write("W %d %d %08x 0\n" % e[1:])
            elif e[0] == "E":
                fh.write("E %d 0 00 0\n" % e[1])


def cmd_gen(d):
    d = Path(d); d.mkdir(parents=True, exist_ok=True)
    ev, pm, order, feb = build_stimulus()
    write_stim(d / "stim.txt", ev)
    exp = dict(sim=SIM, counters=pm.c, frames=pm.frames, order=order, frame_end_byte=feb)
    (d / "expect.json").write_text(json.dumps(exp, indent=1))
    print("gen: %d events, %d odometry frames, counters %s" % (len(ev), len(pm.frames), pm.c))


# ---------------------------------------------------------------- checker
ODOM_FIELDS = ["seq", "mcu_ms", "flags", "left", "right", "oflags", "v_l", "v_r", "v_c",
               "dist_l", "dist_r", "dist_c", "heading", "dt", "cum_l", "cum_r"]
CNT_NAMES = ["good", "bad", "gapev", "lost", "dup", "partial", "framing", "skip",
             "linkto", "glitch", "jump"]


def parse_log(path):
    odo, cnt, frames_tx, misc = [], None, [], {}
    for line in Path(path).read_text().splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "ODOM":
            # ODOM upd ts seq mcu_ms flags left right oflags v_l v_r v_c dl dr dc head dt cl cr
            v = [int(x) for x in p[1:]]
            rec = dict(update=v[0], ts=v[1])
            for k, x in zip(ODOM_FIELDS, v[2:]):
                rec[k] = x
            odo.append(rec)
        elif p[0] == "CNT":
            cnt = dict(zip(CNT_NAMES, [int(x) for x in p[1:]]))
        elif p[0] == "TXB":     # TXB <byte_index> <start_cycle> <bit_cycles>
            frames_tx.append((int(p[1]), int(p[2]), int(p[3])))
        elif p[0] == "TBOFF":
            misc["tboff"] = int(p[1])
        elif p[0] == "BENCH_ERRORS":
            misc["bench_errors"] = int(p[1])
        elif p[0] == "CFG":
            misc["cfg"] = [int(x) for x in p[1:]]
    return odo, cnt, frames_tx, misc


def to_signed(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


def cmd_check(d):
    d = Path(d)
    exp = json.loads((d / "expect.json").read_text())
    odo, cnt, txb, misc = parse_log(d / "tb_log.txt")
    errs = []
    # --- counters predicted purely from the stimulus ---
    if cnt is None:
        errs.append("no CNT line in bench log")
    else:
        for k in CNT_NAMES:
            if k == "jump":
                continue        # predicted by the odometry model below
            if cnt[k] != exp["counters"][k]:
                errs.append("counter %s: rtl %d expected %d" % (k, cnt[k], exp["counters"][k]))
    # --- per-frame comparison ---
    frames = exp["frames"]
    if len(odo) != len(frames):
        errs.append("odometry updates: rtl %d expected %d" % (len(odo), len(frames)))
    om = Odom(SIM["CLK_KHZ"], SIM["MAX_DT"])
    fi = 0
    jumps = 0
    prev_lr = (0, 0)
    ferr = dict(v=0.0, dist=0.0, head=0.0)   # vs floating-point physics
    for item in exp["order"]:
        if item[0] == "W":
            om.write(item[1], item[2])
            continue
        if fi >= len(odo):
            break
        f, r = frames[item[1]], odo[fi]
        want = om.frame(f["left"], f["right"], r["ts"])
        jumps += int(want["jump"])
        want.update(seq=f["seq"], mcu_ms=f["mcu_ms"], flags=f["flags"],
                    left=f["left"], right=f["right"])
        got = dict(r)
        for k, bits in [("left", 32), ("right", 32), ("v_l", 32), ("v_r", 32), ("v_c", 32),
                        ("heading", 32), ("dist_l", 64), ("dist_r", 64), ("dist_c", 64),
                        ("cum_l", 64), ("cum_r", 64)]:
            got[k] = to_signed(got[k], bits)
        want["left"], want["right"] = s32(want["left"]), s32(want["right"])
        for k in ODOM_FIELDS:
            if got[k] != want[k]:
                errs.append("frame %d (seq %d) field %s: rtl %d expected %d"
                            % (fi, f["seq"], k, got[k], want[k]))
        # independent float sanity (catches a model that is wrong in the same way as the RTL)
        if om.cpr and om.diam and om.track:
            k_exact = math.pi * om.diam / om.cpr                       # um / count
            if want["oflags"] & 1:
                dt_s = r["dt"] / SIM["CLK_HZ"]
                for key, dcnt in (("v_l", s32(f["left"] - prev_lr[0])), ("v_r", s32(f["right"] - prev_lr[1]))):
                    exact = dcnt * k_exact / 1000.0 / dt_s              # mm/s
                    ferr["v"] = max(ferr["v"], abs(got[key] / 65536.0 - exact))
            for key, c in (("dist_l", want["cum_l"]), ("dist_r", want["cum_r"])):
                ferr["dist"] = max(ferr["dist"], abs(got[key] - c * k_exact))
            exact_h = (want["cum_r"] - want["cum_l"]) * k_exact / om.track
            ferr["head"] = max(ferr["head"], abs(got["heading"] / 65536.0 - exact_h))
        prev_lr = (f["left"], f["right"])
        if got["update"] != fi + 1:
            errs.append("frame %d: UPDATE_COUNT %d" % (fi, got["update"]))
        fi += 1
    print("max |rtl - float physics|: speed %.6f mm/s, distance %.3f um, heading %.3e rad"
          % (ferr["v"], ferr["dist"], ferr["head"]))
    if ferr["v"] > 1e-3 or ferr["dist"] > 2.0 or ferr["head"] > 1e-4:
        errs.append("fixed-point result deviates from float physics beyond the documented bound")
    if cnt is not None and jumps != cnt["jump"]:
        errs.append("jump counter: rtl %d, model over frames %d" % (cnt["jump"], jumps))
    # --- timestamps: strictly increasing ---
    for a, b in zip(odo, odo[1:]):
        if b["ts"] <= a["ts"]:
            errs.append("timestamp not increasing at update %d" % b["update"])
    # --- timestamps vs bench drive times (independent of the RTL). The stamp
    # of a frame is the timebase when the RTL samples the last stop bit, i.e.
    # start of the frame's last byte + 9.5 RTL bit periods (DIV, whatever the
    # actual line bit width) + synchronizer/register latency (constant).
    starts = {idx: start for idx, start, _bit in txb}
    tbo = misc.get("tboff")
    feb = exp["frame_end_byte"]
    if tbo is None or len(feb) != len(odo) or any(i not in starts for i in feb):
        errs.append("timestamp drive-time check impossible (TBOFF %s, %d ends, %d updates)"
                    % (tbo, len(feb), len(odo)))
    else:
        lat = [r["ts"] + tbo - (starts[i] + 9 * SIM["DIV"] + SIM["DIV"] // 2)
               for i, r in zip(feb, odo)]
        lo, hi = min(lat), max(lat)
        if hi - lo > 1 or lo < 0 or hi > 8:
            errs.append("timestamp latency after mid-stop-bit not constant/in [0,8]: %d..%d" % (lo, hi))
        else:
            print("timestamp latency after mid-stop-bit sample: %d..%d cycles over %d frames"
                  % (lo, hi, len(lat)))
    if misc.get("cfg") and misc["cfg"] != [SIM["CLK_HZ"], SIM["BAUD"], SIM["BYTE_GAP"], SIM["LINK_TO"], SIM["MAX_DT"]]:
        errs.append("bench parameters %s differ from model %s" % (misc["cfg"], SIM))
    if misc.get("bench_errors") != 0:
        errs.append("bench reported errors: %s" % misc.get("bench_errors"))
    # --- summary ---
    if errs:
        for e in errs[:40]:
            print("ERROR", e)
        print("ENC_UART_CHECK FAIL (%d errors)" % len(errs))
        return 1
    print("counters:", cnt)
    print("ENC_UART_CHECK PASS (%d odometry frames bit-exact, counters match)" % len(odo))
    return 0


def cmd_selftest():
    f = encode_frame(7, 0x1234, -1, 2, 3)
    assert len(f) == FRAME_LEN and f[:2] == bytes(SYNC)
    assert f[-1] == sum(f[2:14]) & 0xFF
    k, err = odom_k_q16(65000, 1320)
    exact = math.pi * 65000 / 1320
    assert abs(k / 65536 - exact) < 1 / 65536 and not err
    print("selftest ok: K_Q16(65 mm, 1320) = %d (%.6f um/count, exact %.6f)" % (k, k / 65536, exact))
    return 0


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "selftest":
        sys.exit(cmd_selftest())
    if len(sys.argv) != 3 or sys.argv[1] not in ("gen", "check"):
        print(__doc__); sys.exit(2)
    sys.exit(cmd_gen(sys.argv[2]) or 0 if sys.argv[1] == "gen" else cmd_check(sys.argv[2]))
