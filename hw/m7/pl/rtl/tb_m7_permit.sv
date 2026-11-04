// Self-checking XSim bench for m7_permit_axil (2026-10-11).
//
// FULL=1: scaled timing (TIMEOUT/HB_HALF small) and every scene.
// FULL=0: real timing (200 ms / 5 ms at 100 MHz): first heartbeat edge,
//         exact expiry at the power-on window and revocation latency only.
// S10 (FULL=1, v3fc2 2026-10-26): runtime window register 0x44 (clamp to
//         [TMIN, TMAX], a write disarms, new window governs expiry, reset value).
// Expected times are computed by this bench from the cycles at which IT drove
// frame_done and the ids/ages IT wrote, never read back from the DUT.
module tb_m7_permit;
    parameter integer TIMEOUT = 2000;
    parameter integer HALF    = 50;
    parameter integer FULL    = 1;
    parameter integer TMIN    = TIMEOUT / 10;    // ratios of m7_bd.tcl: 100 / 1000 / 5000 ms
    parameter integer TMAX    = TIMEOUT * 5;
    localparam integer W3     = (TIMEOUT * 3) / 10;   // "300 ms" at the scaled default
    localparam integer PERIOD = 2 * HALF;

    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;
    logic [15:0] awaddr = 0, araddr = 0;
    logic        awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    logic [31:0] wdata = 0;
    wire         awready, wready, bvalid, arready, rvalid;
    wire [31:0]  rdata;
    wire [1:0]   bresp, rresp;
    logic        frame_done = 0;
    wire [63:0]  timebase;
    wire         hb, permit, irq;
    int errors = 0;

    m7_timebase u_tb (.clk(clk), .rst_n(rstn), .timebase(timebase));
    m7_permit_axil #(.TIMEOUT_CYCLES(TIMEOUT), .TIMEOUT_MIN_CYCLES(TMIN), .TIMEOUT_MAX_CYCLES(TMAX),
                     .HB_HALF_CYCLES(HALF)) dut (
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'b0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'b0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .frame_done_i(frame_done), .frame_id_i(32'd0), .timebase_i(timebase),
        .permit_hb_o(hb), .permit_o(permit), .irq(irq));

    // ---------------- cycle counter and heartbeat monitor ----------------
    longint cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;
    int     epoch_phase = -1;        // rise phase (mod PERIOD) in this reset epoch
    longint last_rise = -1, rise_at = -1;
    int     rises = 0;
    int     full_pulses = 0;
    longint prev_full_rise = -1;
    logic   hb_d = 0;
    logic rstn_prev = 0;   // reset already sampled on the previous edge
    always @(posedge clk) begin
        rstn_prev <= rstn;
        // From the second reset edge on the (synchronously reset) pin must be low.
        if (!rstn && !rstn_prev && hb === 1'b1 && cyc > 2) begin
            $display("ERROR hb high during reset at cyc %0d", cyc); errors++;
        end
        hb_d <= hb;
        if (hb && !hb_d) begin
            rises++;
            // A high phase may only start at a divider period boundary: every
            // rise in one reset epoch has the same phase modulo PERIOD.
            if (epoch_phase < 0) epoch_phase = cyc % PERIOD;
            else if ((cyc % PERIOD) != epoch_phase) begin
                $display("ERROR rising edge off the period grid at cyc %0d (phase %0d, epoch %0d)",
                         cyc, cyc % PERIOD, epoch_phase); errors++;
            end
            last_rise = cyc; rise_at = cyc;
        end
        if (!hb && hb_d) begin
            if (cyc - rise_at > HALF) begin
                $display("ERROR high pulse %0d cycles > HALF %0d", cyc - rise_at, HALF); errors++;
            end
            // A pulse not cut short by revocation must be exactly HALF wide.
            if (permit === 1'b1) begin
                if (cyc - rise_at != HALF) begin
                    $display("ERROR full pulse %0d cycles != HALF %0d", cyc - rise_at, HALF); errors++;
                end else full_pulses++;
                if (prev_full_rise >= 0 && rise_at - prev_full_rise != PERIOD && rise_at - prev_full_rise < 2 * PERIOD) begin
                    $display("ERROR heartbeat period %0d != %0d", rise_at - prev_full_rise, PERIOD); errors++;
                end
                prev_full_rise = rise_at;
            end
        end
        if (!rstn) epoch_phase = -1;
        if (hb && rise_at >= 0 && cyc - rise_at > HALF) begin
            $display("ERROR heartbeat stuck high at cyc %0d", cyc); errors++;
            rise_at = cyc;   // report once per stuck period
        end
    end

    // permit 1->0 and pin state, sampled 1 time unit after each edge; `cyc`
    // then already holds that edge's count, so all stamps share one convention.
    longint last_fall = -1, pin_low_after_fall = -1;
    logic   permit_prev = 0;
    always @(posedge clk) begin
        #1;
        if (permit_prev && !permit) begin last_fall = cyc; pin_low_after_fall = -1; end
        if (last_fall >= 0 && pin_low_after_fall < 0 && hb === 1'b0) pin_low_after_fall = cyc;
        permit_prev = permit;
    end

    // ---------------- AXI tasks (as tb_m7_axil_shells.sv) ----------------
    longint wr_done_cyc;
    task automatic wr(input [15:0] a, input [31:0] v);
        @(negedge clk); awaddr = a; wdata = v; awvalid = 1; wvalid = 1; bready = 1;
        do @(posedge clk); while (!(awready && wready));
        #1 wr_done_cyc = cyc;        // the write takes effect on this edge
        @(negedge clk); awvalid = 0; wvalid = 0;
        while (!bvalid) @(posedge clk);
        if (bresp != 0) begin $display("ERROR bresp @%h", a); errors++; end
        @(negedge clk); bready = 0;
    endtask
    task automatic rd(input [15:0] a, output [31:0] v);
        @(negedge clk); araddr = a; arvalid = 1; rready = 1;
        do @(posedge clk); while (!arready);
        @(negedge clk); arvalid = 0;
        while (!rvalid) @(posedge clk);
        v = rdata;
        if (rresp != 0) begin $display("ERROR rresp @%h", a); errors++; end
        @(negedge clk); rready = 0;
    endtask
    task automatic expect_eq(input string what, input [63:0] got, input [63:0] exp);
        if (got !== exp) begin $display("ERROR %s got %0d (0x%h) expected %0d", what, got, got, exp); errors++; end
        else $display("ok    %s = %0d", what, got);
    endtask
    // One-clock completion pulse; returns the cycle on which the DUT samples it.
    longint done_cyc;
    task automatic pulse_done();
        @(negedge clk); frame_done = 1;
        @(posedge clk); #1 done_cyc = cyc;
        @(negedge clk); frame_done = 0;
    endtask
    task automatic frame(input [31:0] id);
        wr(16'h0008, id);
        pulse_done();
    endtask
    task automatic idle(input int n);
        repeat (n) @(posedge clk);
    endtask
    // No rising edge may occur during n cycles.
    task automatic expect_no_rise(input string what, input int n);
        int r0;
        r0 = rises; idle(n);
        if (rises != r0) begin $display("ERROR %s: %0d heartbeat edges while not permitted", what, rises - r0); errors++; end
        else $display("ok    %s: no heartbeat edge in %0d cycles", what, n);
    endtask
    // Waits for the permit to fall; checks it falls exactly at `exp_fall`
    // (first cycle with permit low, sampled after that edge) and that the pin
    // is low no later than 1 cycle after.
    task automatic expect_fall_at(input string what, input longint exp_fall);
        // Wait (bounded) for a fall at or after exp_fall-1, then compare exactly.
        longint guard;
        guard = 0;
        while (!(last_fall >= exp_fall - 1 && last_fall >= 0) && guard < 4 * TIMEOUT + 100) begin
            @(posedge clk); guard++;
        end
        repeat (3) @(posedge clk);
        if (last_fall != exp_fall) begin
            $display("ERROR %s: permit fell at %0d expected %0d (diff %0d)", what, last_fall, exp_fall, last_fall - exp_fall); errors++;
        end else $display("ok    %s: permit fell exactly at the expected cycle", what);
        if (pin_low_after_fall < 0 || pin_low_after_fall - last_fall > 1) begin
            $display("ERROR %s: pin low %0d cycles after permit fell (limit 1)", what, pin_low_after_fall - last_fall); errors++;
        end else $display("ok    %s: heartbeat pin low %0d cycle(s) after permit fell", what, pin_low_after_fall - last_fall);
    endtask

    logic [31:0] v, v2, st;
    longint t0, lo, hi;
    int r0, r1;
    initial begin
        // ======== S0 reset: pin low in and after reset, no frames -> no edge ========
        idle(10); rstn = 1; @(posedge clk);
        rd(16'h0000, v); expect_eq("MAGIC", v, 32'h4D37504D);
        rd(16'h0044, v); expect_eq("TIMEOUT_CYCLES", v, TIMEOUT);
        rd(16'h0048, v); expect_eq("HB_HALF_CYCLES", v, HALF);
        expect_no_rise("S0 after reset, no frame", 3 * PERIOD);

        if (FULL) begin
            // ======== S1 PS writes alone must never grant ========
            for (int i = 1; i <= 20; i++) begin
                wr(16'h0008, i); wr(16'h000C, 0); wr(16'h0004, 0);
                wr(16'h0040, 7); wr(16'h003C, 7); wr(16'h0044, TIMEOUT);
            end
            expect_no_rise("S1 PS register writes without completion edge", 3 * PERIOD);
            rd(16'h0010, st); expect_eq("S1 STATUS.permit", st[0], 0);
            rd(16'h001C, v);  expect_eq("S1 FRAMES_ACCEPTED", v, 0);
        end

        // ======== S2 completion edges grant; heartbeat is a clean 100 Hz ========
        frame(32'd100);
        t0 = done_cyc;
        r0 = rises;
        while (rises == r0 && cyc - t0 < PERIOD + 4) @(posedge clk);
        if (rises == r0) begin $display("ERROR S2 no heartbeat edge within PERIOD+4 of grant"); errors++; end
        else $display("ok    S2 first heartbeat edge %0d cycles after grant (limit %0d)", last_rise - t0, PERIOD + 2);
        if (FULL) begin
            r0 = rises; lo = cyc;
            for (int i = 1; i <= 6; i++) begin idle(TIMEOUT / 2); frame(32'd100 + i); end
            r1 = rises - r0;
            hi = cyc;
            // continuous permit: one edge per PERIOD, allow +-1 for window edges
            if (r1 < (hi - lo) / PERIOD - 1 || r1 > (hi - lo) / PERIOD + 1) begin
                $display("ERROR S2 %0d edges in %0d cycles, expected ~%0d", r1, hi - lo, (hi - lo) / PERIOD); errors++;
            end else $display("ok    S2 continuous heartbeat: %0d edges in %0d cycles", r1, hi - lo);
            rd(16'h0018, v); expect_eq("S2 LAST_FRAME_ID", v, 106);
        end

        // ======== S3 frames stop: expiry exactly TIMEOUT after the last edge ========
        t0 = done_cyc;
        expect_fall_at("S3 stale expiry", t0 + TIMEOUT);
        rd(16'h0024, v); expect_eq("S3 REVOKE_COUNT", v, 1);
        rd(16'h0028, v); expect_eq("S3 LAST_CAUSE (1=timeout)", v, 1);
        rd(16'h002C, v); rd(16'h0034, v2);
        // done_ts = timebase in the cycle the edge was sampled; revoke_ts = timebase
        // in the first cycle with permit low: permitted cycles = TIMEOUT -> diff TIMEOUT+1.
        expect_eq("S3 revoke_ts - done_ts", v2 - v, TIMEOUT + 1);
        expect_no_rise("S3 after expiry", 3 * PERIOD);

        if (!FULL) begin
            $display("info  %0d full heartbeat pulses checked (width HALF, period 2*HALF)", full_pulses);
            if (errors == 0) $display("TB_M7_PERMIT PASS (real timing)");
            else $display("TB_M7_PERMIT FAIL errors=%0d", errors);
            $finish;
        end

        // ======== S4 PS writes after expiry do not regrant ========
        for (int i = 0; i < 10; i++) begin wr(16'h0008, 32'd500 + i); wr(16'h0004, 0); end
        expect_no_rise("S4 PS writes while stale", 2 * PERIOD);

        // ======== S5 frame-id monotonicity ========
        frame(32'd200); idle(2 * PERIOD);
        rd(16'h0010, st); expect_eq("S5 permit after id 200", st[0], 1);
        frame(32'd200);                                   // replay
        expect_fall_at("S5 replayed id revokes", done_cyc);
        rd(16'h0028, v); expect_eq("S5 LAST_CAUSE (3=replay)", v, 3);
        frame(32'd199);                                   // older
        expect_no_rise("S5 older id", 2 * PERIOD);
        rd(16'h0020, v); expect_eq("S5 FRAMES_REJECTED", v, 2);
        frame(32'd201); idle(PERIOD + 4);
        rd(16'h0010, st); expect_eq("S5 newer id regrants", st[0], 1);

        // ======== S6 software emergency stop ========
        wr(16'h0040, 32'h1);                              // irq on revoke
        wr(16'h003C, 32'h7);                              // clear stale bits
        wr(16'h0004, 32'h1);
        t0 = wr_done_cyc;
        expect_fall_at("S6 FORCE_STOP revokes", t0);
        rd(16'h0028, v); expect_eq("S6 LAST_CAUSE (2=force)", v, 2);
        if (irq !== 1'b1) begin $display("ERROR S6 revoke irq missing"); errors++; end
        else $display("ok    S6 revoke irq asserted");
        wr(16'h003C, 32'h1);
        if (irq !== 1'b0) begin $display("ERROR S6 irq not cleared by W1C"); errors++; end
        frame(32'd202);                                   // frame while forced
        expect_no_rise("S6 frame during FORCE_STOP", 2 * PERIOD);
        wr(16'h0004, 32'h0);                              // release
        expect_no_rise("S6 FORCE_STOP released, no new frame", 2 * PERIOD);
        frame(32'd203); idle(PERIOD + 4);
        rd(16'h0010, st); expect_eq("S6 new frame after release grants", st[0], 1);

        // ======== S7 capture age shortens the window / rejects too-old ========
        wr(16'h000C, TIMEOUT - 300);
        frame(32'd204);
        expect_fall_at("S7 capture age TIMEOUT-300", done_cyc + 300);
        wr(16'h000C, TIMEOUT);
        frame(32'd205);
        idle(4);
        rd(16'h0028, v); rd(16'h0010, st);
        expect_eq("S7 too-old frame not granted", st[0], 0);
        rd(16'h0020, v2); expect_eq("S7 FRAMES_REJECTED", v2, 3);
        wr(16'h000C, 0);

        // ======== S8 completion line stuck high: only one grant ========
        rd(16'h001C, v);
        wr(16'h0008, 32'd300);
        @(negedge clk); frame_done = 1; @(posedge clk); #1 t0 = cyc;
        for (int i = 1; i <= 30; i++) begin wr(16'h0008, 32'd300 + i); idle(TIMEOUT / 20); end
        expect_fall_at("S8 stuck-high completion line expires", t0 + TIMEOUT);
        rd(16'h001C, v2); expect_eq("S8 only one acceptance", v2 - v, 1);
        @(negedge clk); frame_done = 0;

        // ======== S10 runtime window register (v3fc2) ========
        rd(16'h0058, v); expect_eq("S10 WINDOW_MIN", v, TMIN);
        rd(16'h005C, v); expect_eq("S10 WINDOW_MAX", v, TMAX);
        rd(16'h0044, v); expect_eq("S10 window still the power-on default", v, TIMEOUT);
        frame(32'd350); idle(PERIOD + 4);
        rd(16'h0010, st); expect_eq("S10 permitted before the window write", st[0], 1);
        wr(16'h0044, W3);
        expect_fall_at("S10 window write while permitted revokes at once", wr_done_cyc);
        rd(16'h0028, v); expect_eq("S10 LAST_CAUSE (5=window written)", v, 5);
        rd(16'h0044, v); expect_eq("S10 window reads back the written value", v, W3);
        expect_no_rise("S10 window write alone never grants", 2 * PERIOD);
        frame(32'd351);
        expect_fall_at("S10 expiry exactly at the written window", done_cyc + W3);
        rd(16'h0028, v); expect_eq("S10 LAST_CAUSE after expiry (1=timeout)", v, 1);
        wr(16'h0044, TIMEOUT);                            // widen after expiry
        expect_no_rise("S10 widening an expired window does not re-grant", 2 * PERIOD);
        rd(16'h0010, st); expect_eq("S10 STATUS.permit after widening", st[0], 0);
        wr(16'h0044, 32'd1);          rd(16'h0044, v); expect_eq("S10 clamp 1 -> MIN", v, TMIN);
        wr(16'h0044, 32'd0);          rd(16'h0044, v); expect_eq("S10 clamp 0 -> MIN", v, TMIN);
        wr(16'h0044, TMIN - 1);       rd(16'h0044, v); expect_eq("S10 clamp MIN-1 -> MIN", v, TMIN);
        wr(16'h0044, 32'hFFFFFFFF);   rd(16'h0044, v); expect_eq("S10 clamp 0xFFFFFFFF -> MAX", v, TMAX);
        wr(16'h0044, TMAX + 1);       rd(16'h0044, v); expect_eq("S10 clamp MAX+1 -> MAX", v, TMAX);
        wr(16'h0044, TMIN);           rd(16'h0044, v); expect_eq("S10 MIN exact", v, TMIN);
        frame(32'd352);
        expect_fall_at("S10 expiry exactly at the MIN window", done_cyc + TMIN);
        wr(16'h0044, TMAX);
        frame(32'd353); idle(PERIOD + 4);
        rd(16'h0010, st); expect_eq("S10 permitted at the MAX window", st[0], 1);
        wr(16'h0004, 32'h1);
        expect_fall_at("S10 FORCE_STOP still revokes at once (MAX window)", wr_done_cyc);
        rd(16'h0028, v); expect_eq("S10 LAST_CAUSE (2=force)", v, 2);
        wr(16'h0004, 32'h0);
        expect_no_rise("S10 FORCE_STOP released, no new frame", 2 * PERIOD);

        // ======== S9 reset while permitted ========
        frame(32'd400); idle(PERIOD + HALF / 2);
        if (hb !== 1'b1 && permit !== 1'b1) begin $display("ERROR S9 setup not permitted"); errors++; end
        @(negedge clk); rstn = 0; @(posedge clk); #1;
        if (hb !== 1'b0) begin $display("ERROR S9 pin not low on the first reset edge"); errors++; end
        else $display("ok    S9 pin low on the first clock edge of reset");
        idle(5); @(negedge clk); rstn = 1; @(posedge clk);
        expect_no_rise("S9 after reset, no frame", 2 * PERIOD);
        rd(16'h0028, v); expect_eq("S9 LAST_CAUSE after reset (0)", v, 0);
        rd(16'h0044, v); expect_eq("S9 window back to the power-on default after reset", v, TIMEOUT);

        $display("info  %0d full heartbeat pulses checked (width HALF, period 2*HALF)", full_pulses);
        if (errors == 0) $display("TB_M7_PERMIT PASS");
        else $display("TB_M7_PERMIT FAIL errors=%0d", errors);
        $finish;
    end
endmodule
