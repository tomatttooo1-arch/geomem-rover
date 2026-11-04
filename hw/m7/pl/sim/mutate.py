#!/usr/bin/env python3
"""Positive controls: apply one named fault to a COPY of an RTL file.

usage: mutate.py <name> <src.v> <dst.v>
Each mutation is an exact text replacement that must match exactly once;
otherwise the script exits 3 (an unapplied mutation would make the control
meaningless). The bench run on the mutated copy is expected to FAIL.
"""
import sys

MUTATIONS = {
    # permit: a PS write to NEXT_FRAME_ID also counts as a refresh
    "permit_ps_write_refreshes": (
        "wire refresh_evt = frame_done_i && !done_d;",
        "wire refresh_evt = (frame_done_i && !done_d) || (do_write && waddr == 14'h0002);"),
    # permit: frame id only has to be >= (replay accepted)
    "permit_replay_accepted": (
        "(id_now > last_id)",
        "(id_now >= last_id)"),
    # permit: FORCE_STOP does not disarm, so releasing it re-grants without a frame
    "permit_force_keeps_armed": (
        "if (s_axi_wdata[0]) begin\n                    armed <= 1'b0;",
        "if (s_axi_wdata[0]) begin\n                    armed <= armed;"),
    # permit (v3fc2): a window write does not disarm (widening re-grants / extends)
    "permit_window_write_keeps_armed": (
        "                armed <= 1'b0;\n                if (!force_stop) disarm_reason <= C_WINDOW;",
        "                armed <= armed;\n                if (!force_stop) disarm_reason <= C_WINDOW;"),
    # permit (v3fc2): written window is not clamped
    "permit_window_no_clamp": (
        "window <= (s_axi_wdata < WMIN) ? WMIN :\n                          (s_axi_wdata > WMAX) ? WMAX : s_axi_wdata;",
        "window <= s_axi_wdata;"),
    # uart: checksum leaves out the flags byte
    "uart_checksum_skips_flags": (
        "psum <= psum + byte_data;",
        "psum <= (pidx == 4'd11) ? psum : psum + byte_data;"),
    # uart: a repeated 0xAA while hunting for 0x55 drops the sync instead of keeping it
    "uart_no_aa_resync": (
        "end else if (byte_data == 8'hAA) begin\n                                 c_skip <= sat_inc(c_skip);               // earlier 0xAA was junk",
        "end else if (1'b0) begin\n                                 c_skip <= sat_inc(c_skip);"),
    # uart: lost-frame count off by one
    "uart_lost_off_by_one": (
        "c_lost <= sat_add(c_lost, {24'd0, seq_d - 8'd1});",
        "c_lost <= sat_add(c_lost, {24'd0, seq_d});"),
    # odometry: left speed loses its sign
    "odom_speed_sign": (
        "w_v_l <= neg32(sat31(ar_quo), w_dl[31]);",
        "w_v_l <= neg32(sat31(ar_quo), 1'b0);"),
}


def main():
    if len(sys.argv) != 4 or sys.argv[1] not in MUTATIONS:
        print(__doc__, "\nnames:", " ".join(MUTATIONS)); return 2
    old, new = MUTATIONS[sys.argv[1]]
    src = open(sys.argv[2]).read()
    n = src.count(old)
    if n != 1:
        print("MUTATION NOT APPLIED: %s matched %d times" % (sys.argv[1], n)); return 3
    open(sys.argv[3], "w").write(src.replace(old, new))
    print("mutation %s applied (1 site)" % sys.argv[1])
    return 0


if __name__ == "__main__":
    sys.exit(main())
