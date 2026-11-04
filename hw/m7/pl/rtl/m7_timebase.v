// Shared PL timebase (2026-10-11).
//
// One free-running 64-bit cycle counter on the PL clock. Every block that
// time-stamps something (permit: depth-frame completion and revocations;
// encoder UART: good-frame arrival) takes this SAME counter as an input, so
// stamps from different blocks are directly comparable (same clock, same
// origin). Origin = release of the PL peripheral reset. It never wraps in
// practice (2^64 cycles at 100 MHz = 5,800 years).
module m7_timebase (
    input  wire        clk,
    input  wire        rst_n,
    output reg  [63:0] timebase
);
    always @(posedge clk) begin
        if (!rst_n) timebase <= 64'd0;
        else        timebase <= timebase + 64'd1;
    end
endmodule
