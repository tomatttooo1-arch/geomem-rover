// Free-running display-output freshness gate for M7.
//
// This module neither generates video timing nor owns the framebuffer RAM. A
// continuously running raster/timing block must supply display_scan_clk,
// frame_start, pixel coordinates and the one-cycle framebuffer read result.
// Control/identity inputs must already be synchronized into display_scan_clk.
// The future integration controller is responsible for converting the real
// capture age to commit_age_cycles by ceiling and saturation; processing
// completion is never age zero. Values too large for the port must saturate,
// never truncate or wrap. ready signals express only local bank availability;
// they do not prove provenance, complete writes, status, age or CDC correctness.
module m7_display_watchdog #(
    parameter integer AGE_WIDTH = 32,
    parameter integer TIMEOUT_CYCLES = 20_000_000
) (
    input  logic                   display_scan_clk,
    input  logic                   rst_n,

    input  logic                   context_load,
    input  logic [63:0]            context_id,
    input  logic                   invalidate,

    input  logic                   frame_commit,
    input  logic                   commit_ok,
    input  logic [63:0]            commit_context_id,
    input  logic [63:0]            commit_frame_id,
    input  logic                   commit_bank,
    input  logic [AGE_WIDTH-1:0]   commit_age_cycles,

    // Must occur at a framebuffer bank-selection boundary, at least one RAM
    // read-latency before the first active pixel that may use the new bank.
    input  logic                   frame_start,

    // pixel_in/x/y/valid are aligned to the same completed framebuffer read.
    input  logic [23:0]            pixel_in,
    input  logic                   pixel_valid_in,
    input  logic [11:0]            pixel_x,
    input  logic [11:0]            pixel_y,

    output logic [23:0]            pixel_out,
    output logic                   pixel_valid_out,

    output logic                   active_bank,
    output logic                   inactive_bank,
    output logic                   write_ready,
    output logic                   commit_ready,
    output logic                   fresh,
    output logic                   stale,
    output logic                   switch_blank,
    output logic                   context_valid,
    output logic                   pending_valid,
    output logic                   protocol_fault,
    output logic                   deadline_miss,
    output logic [AGE_WIDTH-1:0]   active_age_cycles,
    output logic [AGE_WIDTH-1:0]   pending_age_cycles,
    output logic [63:0]            active_context_id,
    output logic [63:0]            active_frame_id,
    output logic [63:0]            pending_frame_id,
    output logic [63:0]            last_seen_frame_id,
    output logic                   last_seen_valid
);
    localparam logic [AGE_WIDTH-1:0] TIMEOUT_VALUE = TIMEOUT_CYCLES;

    // Elaboration-time guard, deliberately NOT inside `ifndef SYNTHESIS and
    // NOT a run-time $fatal. The previous form failed twice over: synthesis
    // excluded the block outright, and the local XSim was measured this round
    // to continue past $fatal, so an over-wide TIMEOUT_CYCLES would have left
    // only an unread message while TIMEOUT_VALUE silently truncated. Vivado
    // evaluates $error in a generate block during elaboration, for synthesis
    // and simulation alike, and nothing at run time can skip it.
    generate
        if (AGE_WIDTH < 1 || TIMEOUT_CYCLES < 1 ||
            TIMEOUT_VALUE != TIMEOUT_CYCLES) begin : gen_timeout_width_error
            $error("TIMEOUT_CYCLES must be positive and fit AGE_WIDTH");
        end
    endgenerate

    logic pending_bank;
    logic armed;

    function automatic logic [AGE_WIDTH-1:0] saturated_increment(
        input logic [AGE_WIDTH-1:0] value
    );
        if (value >= TIMEOUT_VALUE)
            saturated_increment = TIMEOUT_VALUE;
        else
            saturated_increment = value + {{(AGE_WIDTH-1){1'b0}}, 1'b1};
    endfunction

    always_comb begin
        inactive_bank = ~active_bank;
        // A controller may write only the inactive bank while no completed
        // buffer is waiting for the next frame boundary.
        write_ready = context_valid && !pending_valid && !frame_start;
        commit_ready = write_ready;
        fresh = armed && (active_age_cycles < TIMEOUT_VALUE);
        stale = !fresh;
        pixel_valid_out = pixel_valid_in;
        if (fresh && !switch_blank)
            pixel_out = pixel_in;
        else if (pixel_x[3] ^ pixel_y[3])
            pixel_out = 24'hA000A0;
        else
            pixel_out = 24'h180018;
    end

    always_ff @(posedge display_scan_clk) begin
        if (!rst_n) begin
            active_bank <= 1'b0;
            armed <= 1'b0;
            switch_blank <= 1'b0;
            context_valid <= 1'b0;
            pending_valid <= 1'b0;
            protocol_fault <= 1'b0;
            deadline_miss <= 1'b0;
            active_age_cycles <= TIMEOUT_VALUE;
            pending_age_cycles <= TIMEOUT_VALUE;
            active_context_id <= 64'b0;
            active_frame_id <= 64'b0;
            pending_frame_id <= 64'b0;
            pending_bank <= 1'b0;
            last_seen_frame_id <= 64'b0;
            last_seen_valid <= 1'b0;
        end else begin
            switch_blank <= 1'b0;

            // Age advances on the independent scan clock even when no frame,
            // HLS call or CPU transaction occurs. It saturates, never wraps.
            if (armed)
                active_age_cycles <= saturated_increment(active_age_cycles);

            if (context_load) begin
                if (!context_valid || context_id != active_context_id) begin
                    active_context_id <= context_id;
                    context_valid <= 1'b1;
                    armed <= 1'b0;
                    pending_valid <= 1'b0;
                    active_age_cycles <= TIMEOUT_VALUE;
                    pending_age_cycles <= TIMEOUT_VALUE;
                    last_seen_valid <= 1'b0;
                    last_seen_frame_id <= 64'b0;
                    protocol_fault <= 1'b0;
                    deadline_miss <= 1'b0;
                end else begin
                    // Re-loading one context must not erase last_seen and let
                    // an old frame replay. A later strictly larger frame may recover.
                    armed <= 1'b0;
                    pending_valid <= 1'b0;
                    active_age_cycles <= TIMEOUT_VALUE;
                    pending_age_cycles <= TIMEOUT_VALUE;
                    protocol_fault <= 1'b1;
                end
            end else if (invalidate) begin
                armed <= 1'b0;
                pending_valid <= 1'b0;
                active_age_cycles <= TIMEOUT_VALUE;
                pending_age_cycles <= TIMEOUT_VALUE;
            end else if (frame_commit && frame_start) begin
                // The controller must submit outside the bank-swap edge. This
                // removes any ambiguity about old/new inactive-bank ownership.
                armed <= 1'b0;
                pending_valid <= 1'b0;
                active_age_cycles <= TIMEOUT_VALUE;
                pending_age_cycles <= TIMEOUT_VALUE;
                protocol_fault <= 1'b1;
                if (context_valid && commit_context_id == active_context_id &&
                    (!last_seen_valid || commit_frame_id > last_seen_frame_id)) begin
                    last_seen_frame_id <= commit_frame_id;
                    last_seen_valid <= 1'b1;
                end
            end else if (frame_commit) begin
                // Record every newer same-context commit identity even when
                // malformed, so correcting controls cannot replay that frame.
                if (context_valid && commit_context_id == active_context_id &&
                    (!last_seen_valid || commit_frame_id > last_seen_frame_id)) begin
                    last_seen_frame_id <= commit_frame_id;
                    last_seen_valid <= 1'b1;
                end

                if (context_valid && !pending_valid && commit_ok &&
                    commit_context_id == active_context_id &&
                    (!last_seen_valid || commit_frame_id > last_seen_frame_id) &&
                    commit_bank == ~active_bank &&
                    commit_age_cycles < TIMEOUT_VALUE) begin
                    pending_valid <= 1'b1;
                    pending_bank <= commit_bank;
                    pending_frame_id <= commit_frame_id;
                    // This is capture-to-commit age supplied by the trusted
                    // controller, never zeroed merely because work completed.
                    pending_age_cycles <= commit_age_cycles;
                end else begin
                    // In particular, a second commit while pending is rejected
                    // and cannot overwrite or replace the waiting ownership.
                    armed <= 1'b0;
                    pending_valid <= 1'b0;
                    active_age_cycles <= TIMEOUT_VALUE;
                    pending_age_cycles <= TIMEOUT_VALUE;
                    protocol_fault <= 1'b1;
                end
            end else if (frame_start && pending_valid) begin
                // One scan-clock interval has elapsed since the last sampled
                // pending age. Waiting for SOF consumes the same deadline.
                if (saturated_increment(pending_age_cycles) < TIMEOUT_VALUE) begin
                    active_bank <= pending_bank;
                    active_frame_id <= pending_frame_id;
                    active_age_cycles <= saturated_increment(pending_age_cycles);
                    armed <= 1'b1;
                    pending_valid <= 1'b0;
                    pending_age_cycles <= TIMEOUT_VALUE;
                    // The synchronous framebuffer still returns the previous
                    // bank on this edge. Mask exactly that read-latency cycle.
                    switch_blank <= 1'b1;
                    protocol_fault <= 1'b0;
                    deadline_miss <= 1'b0;
                end else begin
                    pending_valid <= 1'b0;
                    pending_age_cycles <= TIMEOUT_VALUE;
                    deadline_miss <= 1'b1;
                end
            end else if (pending_valid) begin
                if (saturated_increment(pending_age_cycles) >= TIMEOUT_VALUE) begin
                    pending_valid <= 1'b0;
                    pending_age_cycles <= TIMEOUT_VALUE;
                    deadline_miss <= 1'b1;
                end else begin
                    pending_age_cycles <= saturated_increment(pending_age_cycles);
                end
            end
        end
    end
endmodule
