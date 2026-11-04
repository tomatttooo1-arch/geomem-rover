// One-bit level synchronizer between the two PL clock domains (v3b, 2026-10-26).
// Thin wrapper over the Xilinx XPM macro xpm_cdc_single so the block design
// can instantiate it as a module reference. Only for LEVEL signals whose
// consumer samples them (the receiving side sees the level DEST_SYNC_FF
// destination clocks later; a pulse shorter than ~2 dest cycles may be lost):
//   frame/interrupt (fast compute clock) -> permit/frame_done_i (100 MHz):
//       ap_done level interrupt, held until the PS clears ISR (>> 2 cycles)
//   permit/permit_o (100 MHz) -> ar/permit_i (fast): drive-permit level,
//       sampled by the AR core once at ap_start
// SRC_INPUT_REG=1 registers the source in its own clock domain (glitch-free
// launch, and report_cdc then sees a properly registered CDC-1 crossing).
`timescale 1ns / 1ps
module m7_cdc_single #(
    parameter integer DEST_SYNC_FF = 3
) (
    input  wire src_clk,
    input  wire src_in,
    input  wire dest_clk,
    output wire dest_out
);
    xpm_cdc_single #(
        .DEST_SYNC_FF(DEST_SYNC_FF),
        .INIT_SYNC_FF(0),
        .SIM_ASSERT_CHK(0),
        .SRC_INPUT_REG(1)
    ) u_sync (
        .src_clk(src_clk),
        .src_in(src_in),
        .dest_clk(dest_clk),
        .dest_out(dest_out)
    );
endmodule
