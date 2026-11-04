// Self-checking XSim bench for the two AXI-lite RTL shells (2026-09-26).
// Checks register maps, pulse semantics and the watchdog's stale interrupt
// timing with a short TIMEOUT so the exact threshold cycle is observable.
module tb_m7_axil_shells;
    localparam integer TIMEOUT = 50;
    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;

    // One AXI-lite master bus per DUT.
    logic [15:0] awaddr[2], araddr[2];
    logic        awvalid[2], wvalid[2], bready[2], arvalid[2], rready[2];
    logic [31:0] wdata[2];
    wire         awready[2], wready[2], bvalid[2], arready[2], rvalid[2];
    wire [31:0]  rdata[2];
    wire [1:0]   bresp[2], rresp[2];
    wire         irq, stale_out;
    int errors = 0;

    m7_sysinfo_axil #(.BUILD_ID(32'h054B2860), .BUILD_DATE(32'h20260926),
                      .CLK_HZ(32'd100000000), .SLOT_MAP(32'h1F)) u_info (
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr[0]), .s_axi_awprot(3'b0), .s_axi_awvalid(awvalid[0]), .s_axi_awready(awready[0]),
        .s_axi_wdata(wdata[0]), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid[0]), .s_axi_wready(wready[0]),
        .s_axi_bresp(bresp[0]), .s_axi_bvalid(bvalid[0]), .s_axi_bready(bready[0]),
        .s_axi_araddr(araddr[0]), .s_axi_arprot(3'b0), .s_axi_arvalid(arvalid[0]), .s_axi_arready(arready[0]),
        .s_axi_rdata(rdata[0]), .s_axi_rresp(rresp[0]), .s_axi_rvalid(rvalid[0]), .s_axi_rready(rready[0]));
    m7_watchdog_axil #(.TIMEOUT_CYCLES(TIMEOUT)) u_wd (
        .s_axi_aclk(clk), .s_axi_aresetn(rstn),
        .s_axi_awaddr(awaddr[1]), .s_axi_awprot(3'b0), .s_axi_awvalid(awvalid[1]), .s_axi_awready(awready[1]),
        .s_axi_wdata(wdata[1]), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid[1]), .s_axi_wready(wready[1]),
        .s_axi_bresp(bresp[1]), .s_axi_bvalid(bvalid[1]), .s_axi_bready(bready[1]),
        .s_axi_araddr(araddr[1]), .s_axi_arprot(3'b0), .s_axi_arvalid(arvalid[1]), .s_axi_arready(arready[1]),
        .s_axi_rdata(rdata[1]), .s_axi_rresp(rresp[1]), .s_axi_rvalid(rvalid[1]), .s_axi_rready(rready[1]),
        .irq(irq), .stale_out(stale_out));

    task automatic wr(input int d, input [15:0] a, input [31:0] v);
        @(negedge clk); awaddr[d] = a; wdata[d] = v; awvalid[d] = 1; wvalid[d] = 1; bready[d] = 1;
        do @(posedge clk); while (!(awready[d] && wready[d]));
        @(negedge clk); awvalid[d] = 0; wvalid[d] = 0;
        while (!bvalid[d]) @(posedge clk);
        if (bresp[d] != 0) begin $display("ERROR bresp dut%0d @%h", d, a); errors++; end
        @(negedge clk); bready[d] = 0;
    endtask
    task automatic rd(input int d, input [15:0] a, output [31:0] v);
        @(negedge clk); araddr[d] = a; arvalid[d] = 1; rready[d] = 1;
        do @(posedge clk); while (!arready[d]);
        @(negedge clk); arvalid[d] = 0;
        while (!rvalid[d]) @(posedge clk);
        v = rdata[d];
        if (rresp[d] != 0) begin $display("ERROR rresp dut%0d @%h", d, a); errors++; end
        @(negedge clk); rready[d] = 0;
    endtask
    task automatic expect_eq(input string what, input [31:0] got, input [31:0] exp);
        if (got !== exp) begin $display("ERROR %s got %h expected %h", what, got, exp); errors++; end
        else $display("ok    %s = %h", what, got);
    endtask

    logic [31:0] v, c0, c1, st;
    int waited;
    initial begin
        for (int d = 0; d < 2; d++) begin
            awvalid[d] = 0; wvalid[d] = 0; bready[d] = 0; arvalid[d] = 0; rready[d] = 0;
            awaddr[d] = 0; araddr[d] = 0; wdata[d] = 0;
        end
        repeat (5) @(posedge clk); rstn = 1; repeat (2) @(posedge clk);

        // ---------------- sysinfo ----------------
        rd(0, 16'h0000, v); expect_eq("sysinfo MAGIC", v, 32'h4D37504C);
        rd(0, 16'h0008, v); expect_eq("sysinfo BUILD_ID", v, 32'h054B2860);
        rd(0, 16'h0010, v); expect_eq("sysinfo CLK_HZ", v, 32'd100000000);
        rd(0, 16'h0020, v); expect_eq("sysinfo SLOT_MAP", v, 32'h1F);
        wr(0, 16'h001C, 32'hA5A55A5A); rd(0, 16'h001C, v); expect_eq("sysinfo SCRATCH", v, 32'hA5A55A5A);
        wr(0, 16'h0000, 32'h0); rd(0, 16'h0000, v); expect_eq("sysinfo MAGIC after RO write", v, 32'h4D37504C);
        rd(0, 16'h0100, v); expect_eq("sysinfo unmapped", v, 32'h0);
        rd(0, 16'h0014, c0); repeat (100) @(posedge clk); rd(0, 16'h0014, c1);
        if (c1 - c0 < 100 || c1 - c0 > 120) begin $display("ERROR cycle delta %0d", c1 - c0); errors++; end
        else $display("ok    cycle counter delta %0d over >=100 clocks", c1 - c0);

        // ---------------- watchdog ----------------
        rd(1, 16'h0000, v); expect_eq("wd MAGIC", v, 32'h4D375744);
        rd(1, 16'h0048, v); expect_eq("wd TIMEOUT", v, TIMEOUT);
        rd(1, 16'h0024, st); expect_eq("wd reset stale/fresh bits", st[5:4], 2'b10);
        wr(1, 16'h0044, 32'h7);                       // enable all irq sources
        wr(1, 16'h0008, 32'h00001234); wr(1, 16'h000C, 32'h0);
        wr(1, 16'h0004, 32'h1);                       // context_load
        rd(1, 16'h0024, st); expect_eq("wd context_valid,write_ready", {st[7], st[2]}, 2'b11);
        // Commit frame 1 into the inactive bank with capture age 5 cycles.
        wr(1, 16'h0010, 32'h00001234); wr(1, 16'h0014, 32'h0);
        wr(1, 16'h0018, 32'd1); wr(1, 16'h001C, 32'h0); wr(1, 16'h0020, 32'd5);
        wr(1, 16'h0004, 32'h304);                     // commit, ok, bank=1
        rd(1, 16'h0024, st); expect_eq("wd pending after commit", st[8], 1'b1);
        wr(1, 16'h0004, 32'h8);                       // frame_start (page flip done)
        rd(1, 16'h0024, st); expect_eq("wd fresh,active_bank after SOF", {st[4], st[0]}, 2'b11);
        rd(1, 16'h0030, v); expect_eq("wd ACTIVE_FRAME_ID", v, 32'd1);
        if (irq !== 1'b0) begin $display("ERROR irq asserted while fresh"); errors++; end
        // Wait for stale without any further bus activity; bound the cycle.
        waited = 0;
        while (!stale_out && waited < 4 * TIMEOUT) begin @(posedge clk); waited++; end
        if (!stale_out) begin $display("ERROR never went stale"); errors++; end
        else $display("ok    stale after %0d idle clocks (timeout %0d incl. age 5 + bus)", waited, TIMEOUT);
        repeat (3) @(posedge clk);
        if (irq !== 1'b1) begin $display("ERROR stale irq missing"); errors++; end
        rd(1, 16'h0040, v); expect_eq("wd IRQ_STATUS stale", v[0], 1'b1);
        wr(1, 16'h0040, 32'h1);
        rd(1, 16'h0040, v); expect_eq("wd IRQ_STATUS after W1C", v[0], 1'b0);
        if (irq !== 1'b0) begin $display("ERROR irq not cleared"); errors++; end
        // Replaying frame id 1 must be rejected as a protocol fault.
        wr(1, 16'h0004, 32'h104);                     // commit ok, bank 0 = inactive, same id 1
        rd(1, 16'h0024, st); expect_eq("wd replay -> protocol_fault", st[9], 1'b1);
        rd(1, 16'h0040, v); expect_eq("wd IRQ_STATUS fault", v[1], 1'b1);
        // Too-old commit (age >= timeout) must be rejected.
        wr(1, 16'h0018, 32'd2); wr(1, 16'h0020, TIMEOUT);
        wr(1, 16'h0004, 32'h104);                     // commit ok bank=0 (inactive now)
        rd(1, 16'h0024, st); expect_eq("wd old commit not pending", st[8], 1'b0);
        // Fresh newer frame recovers.
        wr(1, 16'h0018, 32'd3); wr(1, 16'h0020, 32'd2);
        rd(1, 16'h0024, st);
        wr(1, 16'h0004, 32'h104 | (st[1] << 9));      // bank = current inactive
        wr(1, 16'h0004, 32'h8);
        rd(1, 16'h0024, st); expect_eq("wd recovered fresh", st[4], 1'b1);

        if (errors == 0) $display("TB_M7_AXIL_SHELLS PASS");
        else $display("TB_M7_AXIL_SHELLS FAIL errors=%0d", errors);
        $finish;
    end
endmodule
