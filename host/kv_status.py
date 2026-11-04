"""Nano-side decoding of TELEM answers: unit conversion and ages (2026-10-11).

docs/design/PC_DATAPLANE_SPEC_20261011.md A.3. The KV260 sends raw integers;
every conversion formula lives here, with known-answer tests
(tests/test_kv_link_ext.py):

  Q16.16 mm/s  -> mm/s   v / 65536            (encoder v_left/right/center)
  um (int64)   -> m      d / 1e6              (encoder dist_left/right/center)
  Q16.16 rad   -> rad    h / 65536            (encoder heading)
  PL cycles    -> ms     c / pl_clk_hz * 1e3  (permit age, encoder dt, PL ages)
  ns           -> ms     t / 1e6              (KV stage durations)

Ages are differences on ONE clock each: KV blocks ``kv_mono_ns - kv_done_ns``
(KV clock), PL blocks ``(tb_now - ts) / clk`` (PL clock); the Nano then adds the
whole round trip (its own clock) as an upper bound: ``age_upper_ms``. No clock
mapping (B2 offset) is ever used here -- it must not enter any age (CS 1).
A block whose clock is unknown (no sysinfo CLK_HZ) gets None, never a guess.
"""

from collections import namedtuple

from host.kv_link import TELEM_BLOCKS, mask_blocks

Q16 = 65536.0
ENC_COUNTER_NAMES = ("good", "bad_checksum", "seq_jumps", "lost", "duplicate", "partial",
                     "framing", "sync_skipped_bytes", "link_timeouts", "start_glitches",
                     "count_jumps")

Telemetry = namedtuple("Telemetry", "avail blocks age_upper_ms rtt_ms kv_boot_id")


def q16_to_float(v):
    return int(v) / Q16


def um_to_m(v):
    return int(v) / 1e6


def cycles_to_ms(c, clk_hz):
    if c is None or not clk_hz:
        return None
    return int(c) / float(clk_hz) * 1e3


def ns_to_ms(t):
    return None if t is None else int(t) / 1e6


def pl_age_ms(tb_now, ts, clk_hz):
    """Age on the PL clock alone; None when the stamp is absent (0) or the clock unknown."""
    if not ts or tb_now is None or not clk_hz or tb_now < ts:
        return None
    return (int(tb_now) - int(ts)) / float(clk_hz) * 1e3


def kv_age_ms(kv_mono_ns, kv_done_ns):
    if kv_done_ns is None or kv_mono_ns < kv_done_ns:
        return None
    return (int(kv_mono_ns) - int(kv_done_ns)) / 1e6


def decode_encoder(raw, clk_hz):
    out = dict(raw)
    for side in ("left", "right", "center"):
        out["v_%s_mm_s" % side] = q16_to_float(raw["v_%s_q16" % side])
        out["dist_%s_m" % side] = um_to_m(raw["dist_%s_um" % side])
    out["heading_rad"] = q16_to_float(raw["heading_q16"])
    out["dt_ms"] = cycles_to_ms(raw["dt_cycles"], clk_hz)
    w = int(raw["seq_word"])
    out["seq"] = {"mcu_ms": w >> 16, "flags": (w >> 8) & 0xFF, "seq": w & 0xFF,
                  "mcu_sees_heartbeat": bool(w >> 8 & 1), "motor_enabled": bool(w >> 9 & 1)}
    out["counters_named"] = dict(zip(ENC_COUNTER_NAMES, raw.get("counters") or []))
    out["link_ok"] = bool(raw["status"] & 1)
    out["frame_age_ms"] = pl_age_ms(raw.get("tb_now"), raw.get("frame_ts"), clk_hz)
    return out


def decode_permit(raw, clk_hz):
    out = dict(raw)
    st = int(raw["status"])
    out["status_bits"] = {"permit": bool(st & 1), "pin": bool(st >> 1 & 1),
                          "accepted": bool(st >> 2 & 1), "estop": bool(st >> 3 & 1),
                          "frame_id_valid": bool(st >> 4 & 1), "fresh": bool(st >> 5 & 1),
                          "revoke_cause": (st >> 8) & 0xF}
    out["age_ms"] = cycles_to_ms(raw["age_cycles"], clk_hz)
    out["timeout_ms"] = cycles_to_ms(raw["timeout_cycles"], clk_hz)
    out["since_done_ms"] = pl_age_ms(raw.get("tb_now"), raw.get("last_done_ts"), clk_hz)
    return out


def _with_times(last, kv_mono_ns, rtt_ms):
    if not last:
        return last, None
    last = dict(last)
    last["times_ms"] = {k: ns_to_ms(v) for k, v in (last.get("times_ns") or {}).items()}
    age = kv_age_ms(kv_mono_ns, last.get("kv_done_ns"))
    last["age_kv_ms"] = age
    return last, (None if age is None else age + rtt_ms)


def decode_telemetry(ack, t_req_nano_ns, t_recv_nano_ns):
    """host.kv_link.decode_telem_ack output + the Nano's send/receive stamps -> Telemetry."""
    rtt_ms = (int(t_recv_nano_ns) - int(t_req_nano_ns)) / 1e6
    doc, kv_mono = ack["doc"], int(ack["kv_mono_ns"])
    avail = mask_blocks(ack["avail"])
    for name in TELEM_BLOCKS:                       # avail bit and the block must agree
        if name in doc and bool(doc[name].get("available")) != (name in avail):
            raise ValueError("TELEM block %r: avail bit and 'available' disagree" % name)
    board = doc.get("board") or {}
    clk = board.get("pl_clk_hz")
    blocks, ages = {}, {}
    for name, raw in doc.items():
        b = dict(raw)
        if not raw.get("available"):
            blocks[name] = b
            continue
        if name in ("geometry", "roi"):
            b["last"], age = _with_times(raw.get("last"), kv_mono, rtt_ms)
            if age is not None:
                ages[name] = age
        elif name == "permit":
            b = decode_permit(raw, clk)
            if b["since_done_ms"] is not None:
                ages[name] = b["since_done_ms"] + rtt_ms
        elif name == "encoder":
            b = decode_encoder(raw, clk)
            if b["frame_age_ms"] is not None:
                ages[name] = b["frame_age_ms"] + rtt_ms
        elif name == "estop":
            age = kv_age_ms(kv_mono, raw.get("latched_kv_ns"))
            b["latched_age_kv_ms"] = age
        blocks[name] = b
    return Telemetry(avail, blocks, ages, rtt_ms, board.get("kv_boot_id", ""))
