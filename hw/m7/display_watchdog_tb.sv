`timescale 1ns/1ps

// BYPASS is only a validation-instrument positive control. The production DUT
// has no bypass. With BYPASS=1 the first stopped-stream pixel assertion must
// fail, proving the checker can distinguish an unguarded old framebuffer.
module m7_display_watchdog_tb #(
    parameter integer BYPASS = 0
);
    localparam integer AGE_WIDTH = 3;
    localparam integer TIMEOUT = 4;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    logic rst_n = 1'b0;
    logic context_load = 1'b0;
    logic [63:0] context_id = 0;
    logic invalidate = 1'b0;
    logic frame_commit = 1'b0;
    logic commit_ok = 1'b0;
    logic [63:0] commit_context_id = 0;
    logic [63:0] commit_frame_id = 0;
    logic commit_bank = 0;
    logic [AGE_WIDTH-1:0] commit_age_cycles = 0;
    logic frame_start = 0;
    logic [23:0] pixel_in = 0;
    logic pixel_valid_in = 1;
    logic [11:0] pixel_x = 0;
    logic [11:0] pixel_y = 0;
    logic [23:0] pixel_out;
    logic pixel_valid_out;
    logic active_bank, inactive_bank, write_ready, commit_ready;
    logic fresh, stale, switch_blank, context_valid, pending_valid;
    logic protocol_fault, deadline_miss;
    logic [AGE_WIDTH-1:0] active_age_cycles, pending_age_cycles;
    logic [63:0] active_context_id, active_frame_id, pending_frame_id;
    logic [63:0] last_seen_frame_id;
    logic last_seen_valid;

    m7_display_watchdog #(
        .AGE_WIDTH(AGE_WIDTH), .TIMEOUT_CYCLES(TIMEOUT)
    ) dut (
        .display_scan_clk(clk), .rst_n(rst_n),
        .context_load(context_load), .context_id(context_id),
        .invalidate(invalidate), .frame_commit(frame_commit),
        .commit_ok(commit_ok), .commit_context_id(commit_context_id),
        .commit_frame_id(commit_frame_id), .commit_bank(commit_bank),
        .commit_age_cycles(commit_age_cycles), .frame_start(frame_start),
        .pixel_in(pixel_in), .pixel_valid_in(pixel_valid_in),
        .pixel_x(pixel_x), .pixel_y(pixel_y),
        .pixel_out(pixel_out), .pixel_valid_out(pixel_valid_out),
        .active_bank(active_bank), .inactive_bank(inactive_bank),
        .write_ready(write_ready), .commit_ready(commit_ready),
        .fresh(fresh), .stale(stale), .switch_blank(switch_blank),
        .context_valid(context_valid), .pending_valid(pending_valid),
        .protocol_fault(protocol_fault), .deadline_miss(deadline_miss),
        .active_age_cycles(active_age_cycles),
        .pending_age_cycles(pending_age_cycles),
        .active_context_id(active_context_id),
        .active_frame_id(active_frame_id),
        .pending_frame_id(pending_frame_id),
        .last_seen_frame_id(last_seen_frame_id),
        .last_seen_valid(last_seen_valid)
    );

    logic [23:0] bank0 [0:15];
    logic [23:0] bank1 [0:15];
    logic [3:0] scan_address = 0;
    logic [23:0] expected_read_pixel = 0;

    // Actual one-cycle synchronous framebuffer-read model. active_bank is
    // sampled here; on a swap edge this still returns the old bank. The DUT's
    // switch_blank must hide it, then the next cycle must match the new bank.
    always_ff @(posedge clk) begin
        if (active_bank) begin
            pixel_in <= bank1[scan_address];
            expected_read_pixel <= bank1[scan_address];
        end else begin
            pixel_in <= bank0[scan_address];
            expected_read_pixel <= bank0[scan_address];
        end
        pixel_x <= {8'b0, scan_address};
        pixel_y <= 12'd16;
        scan_address <= scan_address + 1'b1;
    end

    wire [23:0] observed_pixel = BYPASS ? pixel_in : pixel_out;

    function automatic [23:0] expected_checker_pixel(input [11:0] x, input [11:0] y);
        expected_checker_pixel = (x[3] ^ y[3]) ? 24'hA000A0 : 24'h180018;
    endfunction

    task automatic step;
        begin
            @(posedge clk);
            #1;
            // Every visible pixel is checked, not only the final cycle of a
            // timeout loop. This verifies the pixel mapping selected by DUT
            // state; separate directed assertions below verify state boundaries.
            if (pixel_valid_out) begin
                if (stale || switch_blank)
                    check_condition(pixel_out == expected_checker_pixel(pixel_x,pixel_y),
                                    "all unavailable pixels must be checker");
                else
                    check_condition(pixel_out == pixel_in,
                                    "all fresh nonblank pixels must pass RAM data");
            end
        end
    endtask

    task automatic check_condition(input logic condition, input string message);
        begin
            if (!condition) begin
                $display("WATCHDOG TB FAIL: %s at %0t", message, $time);
                $fatal(1);
            end
        end
    endtask

    task automatic load_context(input [63:0] id);
        begin
            context_id = id;
            context_load = 1'b1;
            step();
            context_load = 1'b0;
        end
    endtask

    task automatic submit(input [63:0] ctx, input [63:0] fid,
                          input logic bank, input [AGE_WIDTH-1:0] age,
                          input logic ok);
        begin
            commit_context_id = ctx;
            commit_frame_id = fid;
            commit_bank = bank;
            commit_age_cycles = age;
            commit_ok = ok;
            frame_commit = 1'b1;
            step();
            frame_commit = 1'b0;
        end
    endtask

    task automatic start_frame;
        begin
            frame_start = 1'b1;
            step();
            frame_start = 1'b0;
        end
    endtask

    integer i;
    initial begin
        for (i=0; i<16; i=i+1) begin
            // Interleaved scan addresses have visibly distinct bank prefixes.
            bank0[i] = 24'h100000 + i;
            bank1[i] = 24'hA00000 + i;
        end

        step(); step();
        check_condition(stale && !context_valid && !pending_valid,
                "reset must start disarmed and stale");
        rst_n = 1'b1;
        step();
        check_condition(stale && !write_ready, "reset release cannot revive old RAM");

        // context_load has priority over a simultaneous commit: the new
        // context is established, but the ambiguous frame is never pending.
        context_id=64'h1000; context_load=1;
        commit_context_id=64'h1000; commit_frame_id=64'd1;
        commit_bank=1; commit_age_cycles=0; commit_ok=1; frame_commit=1;
        step(); context_load=0; frame_commit=0;
        check_condition(active_context_id == 64'h1000 && context_valid &&
                        !pending_valid && !last_seen_valid && stale,
                        "context load must dominate same-cycle commit");

        load_context(64'h1001);
        check_condition(context_valid && write_ready && inactive_bank == 1'b1,
                "new unique context exposes only inactive bank");
        submit(64'h1001, 64'd1, 1'b1, 0, 1'b1);
        check_condition(pending_valid && !commit_ready && !fresh && active_bank == 1'b0,
                "complete commit waits for SOF and locks pending bank");
        start_frame();
        check_condition(active_bank == 1'b1 && switch_blank && fresh &&
                active_age_cycles == 1,
                "SOF swaps bank, includes wait age and blanks old RAM read");
        check_condition(pixel_in[23:20] == 4'h1 &&
                        pixel_out == expected_checker_pixel(pixel_x,pixel_y),
                "swap edge old-bank read must be identified and masked");
        step();
        check_condition(!switch_blank && fresh && pixel_in == expected_read_pixel &&
                        pixel_in[23:20] == 4'hA && pixel_out == pixel_in,
                "one cycle later pixel must come from selected new bank");

        // No HLS/CPU/frame event occurs. Independent scan-clock age reaches
        // T exactly and starts masking in the middle of this raster.
        step();
        check_condition(fresh && active_age_cycles == 3,
                "T-1 active age remains fresh");
        step();
        check_condition(stale && active_age_cycles == TIMEOUT,
                "exact T boundary must become stale without a new event");
        // This is the first BYPASS-sensitive assertion. BYPASS=1 must fail.
        check_condition(observed_pixel == expected_checker_pixel(pixel_x,pixel_y) &&
                observed_pixel != pixel_in,
                "stopped stream must hide old framebuffer pixels");
        for (i=0; i<12; i=i+1) step();
        check_condition(stale && active_age_cycles == TIMEOUT,
                "saturating age must not wrap and revive pixels");

        // A new complete frame recovers only at SOF. The inactive bank is now 0.
        submit(64'h1001, 64'd2, 1'b0, 0, 1'b1);
        check_condition(pending_valid && stale, "recovery commit remains pending");
        start_frame();
        check_condition(active_bank == 1'b0 && fresh && switch_blank,
                "recovery switches only at SOF");
        step();
        check_condition(pixel_out == pixel_in && pixel_in == expected_read_pixel &&
                        pixel_in[23:20] == 4'h1,
                "recovery uses the interleaved new active-bank read");

        // Duplicate and reverse commits while active-fresh must mask at once,
        // rather than relying only on the pending-state rejection case.
        submit(64'h1001, 64'd2, inactive_bank, 0, 1'b1);
        check_condition(stale && protocol_fault,
                        "duplicate active frame must immediately mask");
        submit(64'h1001, 64'd3, inactive_bank, 0, 1'b1);
        start_frame(); step();
        check_condition(fresh && active_frame_id == 3,
                        "higher frame id can recover the same context");
        submit(64'h1001, 64'd1, inactive_bank, 0, 1'b1);
        check_condition(stale && protocol_fault,
                        "reverse active frame must immediately mask");

        // invalidate dominates a simultaneous otherwise-new valid commit and
        // cannot let that commit alter replay identity or create pending work.
        commit_context_id=64'h1001; commit_frame_id=64'd4;
        commit_bank=inactive_bank; commit_age_cycles=0; commit_ok=1;
        invalidate=1; frame_commit=1; step(); invalidate=0; frame_commit=0;
        check_condition(stale && !pending_valid && last_seen_frame_id == 3,
                        "invalidate must dominate same-cycle commit");

        // commit_age is capture-to-commit age. SOF waiting consumes one more.
        load_context(64'h1002);
        submit(64'h1002, 64'd10, inactive_bank, TIMEOUT-2, 1'b1);
        check_condition(pending_valid && pending_age_cycles == TIMEOUT-2,
                "T-2 commit age must be retained, not reset");
        start_frame();
        check_condition(fresh && active_age_cycles == TIMEOUT-1,
                "one SOF wait leaves exactly one fresh cycle");
        step();
        check_condition(stale, "T boundary after late commit must mask");

        load_context(64'h1003);
        submit(64'h1003, 64'd20, inactive_bank, TIMEOUT-1, 1'b1);
        check_condition(pending_valid, "T-1 commit may wait but is not yet displayed");
        start_frame();
        check_condition(!pending_valid && stale && deadline_miss,
                "T-1 plus SOF wait reaches T and must be dropped");

        load_context(64'h1004);
        submit(64'h1004, 64'd30, inactive_bank, TIMEOUT, 1'b1);
        check_condition(stale && !pending_valid && protocol_fault,
                "age at T is rejected immediately");

        // Same-context load is not an epoch and cannot clear replay history.
        load_context(64'h1005);
        submit(64'h1005, 64'd7, inactive_bank, 0, 1'b1);
        check_condition(last_seen_valid && last_seen_frame_id == 7 && pending_valid,
                "accepted pending frame records strict identity");
        load_context(64'h1005);
        check_condition(protocol_fault && !pending_valid && last_seen_frame_id == 7,
                "same context reload must fault without clearing last id");
        submit(64'h1005, 64'd7, inactive_bank, 0, 1'b1);
        check_condition(protocol_fault && !pending_valid,
                "same frame cannot replay after rejected same-context load");
        submit(64'h1005, 64'd8, inactive_bank, 0, 1'b1);
        start_frame(); step();
        check_condition(fresh && active_frame_id == 8,
                "larger frame id may recover after same-context reload fault");

        // A second commit while pending is refused, never replaces ownership.
        load_context(64'h1006);
        submit(64'h1006, 64'd1, inactive_bank, 0, 1'b1);
        check_condition(pending_valid && !commit_ready,
                "pending buffer must deassert commit ready");
        submit(64'h1006, 64'd2, inactive_bank, 0, 1'b1);
        check_condition(protocol_fault && !pending_valid && stale &&
                last_seen_frame_id == 2,
                "second pending commit faults and cannot overwrite display");
        submit(64'h1006, 64'd2, inactive_bank, 0, 1'b1);
        check_condition(protocol_fault && !pending_valid,
                "rejected newer identity cannot be retried to extend life");

        // Late old-context commit, active-bank write, bad status and commit/SOF
        // collision are all fail-closed.
        load_context(64'h1007);
        submit(64'h1006, 64'd3, inactive_bank, 0, 1'b1);
        check_condition(protocol_fault && stale, "old context commit rejected");
        submit(64'h1007, 64'd1, active_bank, 0, 1'b1);
        check_condition(protocol_fault && stale, "active bank cannot be committed");
        submit(64'h1007, 64'd2, inactive_bank, 0, 1'b0);
        check_condition(protocol_fault && stale, "bad completed frame cannot display");
        commit_context_id=64'h1007; commit_frame_id=64'd3;
        commit_bank=inactive_bank; commit_age_cycles=0; commit_ok=1;
        frame_commit=1; frame_start=1; step();
        frame_commit=0; frame_start=0;
        check_condition(protocol_fault && stale && !pending_valid,
                "commit and SOF collision is rejected");

        // Frame-id wrap within an epoch is replay; a new unique context is
        // required before zero can be accepted.
        load_context(64'h1008);
        submit(64'h1008, 64'hFFFF_FFFF_FFFF_FFFF, inactive_bank, 0, 1'b1);
        start_frame(); step();
        check_condition(fresh && active_frame_id == 64'hFFFF_FFFF_FFFF_FFFF,
                "maximum frame id can be displayed once");
        submit(64'h1008, 64'd0, inactive_bank, 0, 1'b1);
        check_condition(stale && protocol_fault, "frame id wrap cannot replay epoch");
        load_context(64'h1009);
        submit(64'h1009, 64'd0, inactive_bank, 0, 1'b1);
        start_frame(); step();
        check_condition(fresh && active_frame_id == 0,
                "zero frame id is legal only in a new unique context");

        invalidate=1; step(); invalidate=0;
        check_condition(stale && !pending_valid,
                "explicit invalidation immediately hides all old pixels");

        // Hard reset forgets IDs by design but never revives RAM. External
        // reset integration must clear/own buffers and allocate a never-reused
        // context before the next complete commit.
        rst_n=0; step(); rst_n=1; step();
        check_condition(stale && !context_valid && !write_ready,
                "hard reset requires external context/buffer handshake");

        $display("M7 display watchdog RTL testbench PASS");
        $finish;
    end
endmodule
