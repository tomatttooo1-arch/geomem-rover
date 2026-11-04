// AXI4-Lite control shell for the unchanged m7_display_watchdog (2026-09-26).
//
// Display scheme 1 (PS DisplayPort scans a DDR framebuffer) has no PL pixel
// stream, so the watchdog's pixel gate is tied off here and the module is used
// for what still applies: a free-running, PS-independent freshness clock
// (display_scan_clk = the PL clock), double-bank ownership (bank = which of two
// DRM framebuffers is on screen) and an interrupt when the shown frame goes
// stale. The PS drives the control pulses:
//   write IDs/age registers, then CTRL.frame_commit (+commit_ok/commit_bank)
//   after the display core finished writing the inactive framebuffer;
//   CTRL.frame_start after the DRM page flip to that framebuffer completed.
// Pixel masking on the scanout itself only exists in scheme 2 (PS DP live
// video from PL), where this same module sits in the pixel path.
// Ages are PL-clock cycles; the PS converts capture age by ceiling and
// saturation (see M7_DISPLAY_WATCHDOG_INTERFACE_20260921.md section 3).
//
// 0x00 MAGIC        RO 0x4D375744 ("M7WD")
// 0x04 CTRL         WO pulses: b0 context_load, b1 invalidate, b2 frame_commit,
//                      b3 frame_start; qualifiers b8 commit_ok, b9 commit_bank
// 0x08/0x0C CONTEXT_ID lo/hi            RW
// 0x10/0x14 COMMIT_CONTEXT_ID lo/hi     RW
// 0x18/0x1C COMMIT_FRAME_ID lo/hi       RW
// 0x20 COMMIT_AGE_CYCLES                RW
// 0x24 STATUS  RO b0 active_bank b1 inactive_bank b2 write_ready b3 commit_ready
//                 b4 fresh b5 stale b6 switch_blank b7 context_valid
//                 b8 pending_valid b9 protocol_fault b10 deadline_miss
//                 b11 last_seen_valid
// 0x28 ACTIVE_AGE_CYCLES RO, 0x2C PENDING_AGE_CYCLES RO
// 0x30/0x34 ACTIVE_FRAME_ID lo/hi RO, 0x38/0x3C LAST_SEEN_FRAME_ID lo/hi RO
// 0x40 IRQ_STATUS  RW1C b0 fresh->stale edge, b1 protocol_fault rise,
//                       b2 deadline_miss rise
// 0x44 IRQ_ENABLE  RW (same bits); irq = |(IRQ_STATUS & IRQ_ENABLE), level
// 0x48 TIMEOUT_CYCLES RO (build parameter)
// 0x4C ACTIVE_CONTEXT_ID lo RO
module m7_watchdog_axil #(
    parameter integer TIMEOUT_CYCLES = 20000000
) (
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    input  wire [15:0] s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [15:0] s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,
    output wire        irq,
    output wire        stale_out
);
    localparam [31:0] MAGIC = 32'h4D375744;

    reg [63:0] context_id, commit_context_id, commit_frame_id;
    reg [31:0] commit_age;
    reg        p_context_load, p_invalidate, p_frame_commit, p_frame_start;
    reg        q_commit_ok, q_commit_bank;
    reg [2:0]  irq_status, irq_enable;

    wire active_bank, inactive_bank, write_ready, commit_ready, fresh, stale,
         switch_blank, context_valid, pending_valid, protocol_fault,
         deadline_miss, last_seen_valid;
    wire [31:0] active_age, pending_age;
    wire [63:0] active_context_id, active_frame_id, pending_frame_id,
                last_seen_frame_id;
    wire [23:0] unused_pixel;
    wire        unused_pixel_valid;

    m7_display_watchdog #(
        .AGE_WIDTH(32),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) u_watchdog (
        .display_scan_clk(s_axi_aclk),
        .rst_n(s_axi_aresetn),
        .context_load(p_context_load),
        .context_id(context_id),
        .invalidate(p_invalidate),
        .frame_commit(p_frame_commit),
        .commit_ok(q_commit_ok),
        .commit_context_id(commit_context_id),
        .commit_frame_id(commit_frame_id),
        .commit_bank(q_commit_bank),
        .commit_age_cycles(commit_age),
        .frame_start(p_frame_start),
        .pixel_in(24'd0),
        .pixel_valid_in(1'b0),
        .pixel_x(12'd0),
        .pixel_y(12'd0),
        .pixel_out(unused_pixel),
        .pixel_valid_out(unused_pixel_valid),
        .active_bank(active_bank),
        .inactive_bank(inactive_bank),
        .write_ready(write_ready),
        .commit_ready(commit_ready),
        .fresh(fresh),
        .stale(stale),
        .switch_blank(switch_blank),
        .context_valid(context_valid),
        .pending_valid(pending_valid),
        .protocol_fault(protocol_fault),
        .deadline_miss(deadline_miss),
        .active_age_cycles(active_age),
        .pending_age_cycles(pending_age),
        .active_context_id(active_context_id),
        .active_frame_id(active_frame_id),
        .pending_frame_id(pending_frame_id),
        .last_seen_frame_id(last_seen_frame_id),
        .last_seen_valid(last_seen_valid)
    );

    assign stale_out = stale;
    assign irq = |(irq_status & irq_enable);

    reg fresh_d, fault_d, miss_d;

    wire do_write = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
    assign s_axi_awready = do_write;
    assign s_axi_wready  = do_write;
    assign s_axi_bresp   = 2'b00;
    wire do_read = s_axi_arvalid && !s_axi_rvalid;
    assign s_axi_arready = do_read;
    assign s_axi_rresp   = 2'b00;
    wire [13:0] waddr = s_axi_awaddr[15:2];
    wire [13:0] raddr = s_axi_araddr[15:2];
    wire [2:0]  irq_clear = (do_write && waddr == 14'h0010) ? s_axi_wdata[2:0] : 3'd0;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            context_id <= 64'd0;
            commit_context_id <= 64'd0;
            commit_frame_id <= 64'd0;
            commit_age <= 32'hFFFFFFFF;
            p_context_load <= 1'b0;
            p_invalidate <= 1'b0;
            p_frame_commit <= 1'b0;
            p_frame_start <= 1'b0;
            q_commit_ok <= 1'b0;
            q_commit_bank <= 1'b0;
            irq_status <= 3'd0;
            irq_enable <= 3'd0;
            fresh_d <= 1'b0;
            fault_d <= 1'b0;
            miss_d <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata <= 32'd0;
        end else begin
            // Control pulses last exactly one PL clock.
            p_context_load <= 1'b0;
            p_invalidate <= 1'b0;
            p_frame_commit <= 1'b0;
            p_frame_start <= 1'b0;

            fresh_d <= fresh;
            fault_d <= protocol_fault;
            miss_d <= deadline_miss;
            // New edges are OR-ed in after a same-cycle W1C clear, so an event
            // arriving while software acknowledges an older one is not lost.
            irq_status <= (irq_status & ~irq_clear) |
                          {!miss_d && deadline_miss,
                           !fault_d && protocol_fault,
                           fresh_d && !fresh};

            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
            if (do_write) begin
                s_axi_bvalid <= 1'b1;
                case (waddr)
                    14'h0001: begin
                        p_context_load <= s_axi_wdata[0];
                        p_invalidate   <= s_axi_wdata[1];
                        p_frame_commit <= s_axi_wdata[2];
                        p_frame_start  <= s_axi_wdata[3];
                        q_commit_ok    <= s_axi_wdata[8];
                        q_commit_bank  <= s_axi_wdata[9];
                    end
                    14'h0002: context_id[31:0] <= s_axi_wdata;
                    14'h0003: context_id[63:32] <= s_axi_wdata;
                    14'h0004: commit_context_id[31:0] <= s_axi_wdata;
                    14'h0005: commit_context_id[63:32] <= s_axi_wdata;
                    14'h0006: commit_frame_id[31:0] <= s_axi_wdata;
                    14'h0007: commit_frame_id[63:32] <= s_axi_wdata;
                    14'h0008: commit_age <= s_axi_wdata;
                    14'h0011: irq_enable <= s_axi_wdata[2:0];
                    default: ;
                endcase
            end
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
            if (do_read) begin
                s_axi_rvalid <= 1'b1;
                case (raddr)
                    14'h0000: s_axi_rdata <= MAGIC;
                    14'h0002: s_axi_rdata <= context_id[31:0];
                    14'h0003: s_axi_rdata <= context_id[63:32];
                    14'h0004: s_axi_rdata <= commit_context_id[31:0];
                    14'h0005: s_axi_rdata <= commit_context_id[63:32];
                    14'h0006: s_axi_rdata <= commit_frame_id[31:0];
                    14'h0007: s_axi_rdata <= commit_frame_id[63:32];
                    14'h0008: s_axi_rdata <= commit_age;
                    14'h0009: s_axi_rdata <= {20'd0, last_seen_valid,
                                              deadline_miss, protocol_fault,
                                              pending_valid, context_valid,
                                              switch_blank, stale, fresh,
                                              commit_ready, write_ready,
                                              inactive_bank, active_bank};
                    14'h000A: s_axi_rdata <= active_age;
                    14'h000B: s_axi_rdata <= pending_age;
                    14'h000C: s_axi_rdata <= active_frame_id[31:0];
                    14'h000D: s_axi_rdata <= active_frame_id[63:32];
                    14'h000E: s_axi_rdata <= last_seen_frame_id[31:0];
                    14'h000F: s_axi_rdata <= last_seen_frame_id[63:32];
                    14'h0010: s_axi_rdata <= {29'd0, irq_status};
                    14'h0011: s_axi_rdata <= {29'd0, irq_enable};
                    14'h0012: s_axi_rdata <= TIMEOUT_CYCLES;
                    14'h0013: s_axi_rdata <= active_context_id[31:0];
                    default:  s_axi_rdata <= 32'd0;
                endcase
            end
        end
    end
endmodule
