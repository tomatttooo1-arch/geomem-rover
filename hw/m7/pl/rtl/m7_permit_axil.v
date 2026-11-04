// Drive-permit heartbeat ("允许行驶心跳") with AXI4-Lite status/control, 2026-10-11.
//
// Output contract (agreed with the chassis MCU firmware, do not change here):
//   permit_hb_o = 100 Hz square wave (HB_HALF_CYCLES high / HB_HALF_CYCLES low)
//   while the depth path is FRESH and software has not forced a stop;
//   constant LOW otherwise. The MCU stops and latches its motors when it sees
//   no rising edge for 30 ms, so ANY constant level (low, high, floating,
//   clock stopped) means "stop".
//
// Freshness (window WINDOW, frame id strictly increasing; same rule as the old
// stream watchdog, which used a fixed 200 ms), with one deliberate difference:
// freshness is granted
// or extended ONLY by a hardware frame-completion edge (frame_done_i, wired to
// the geometry core's completion interrupt). No PS register write can grant
// or extend permission. PS writes can only (a) supply the id/age that the next
// completion edge is judged with, (b) revoke (FORCE_STOP), and (c) set the
// window WINDOW (0x44, v3fc2 2026-10-26), which also disarms (see below). The existing
// m7_watchdog_axil does NOT satisfy this: its refresh is the PS writing
// CTRL.frame_commit + frame_start (see PL_PERMIT_UART_SPEC_20261011.md §2).
//
//   refresh_evt = rising edge of frame_done_i
//   accept      = refresh_evt && (no id seen yet || id > last id)
//                 && capture_age < WINDOW && !FORCE_STOP && !(window write now)
//   on accept:  armed = 1, age = capture_age (then +1 per clock, saturating)
//   on reject (replayed/older id, or capture age >= WINDOW): armed = 0
//   fresh       = armed && age < WINDOW
//   permit      = fresh && !FORCE_STOP
// Writing FORCE_STOP=1 also disarms, so clearing it never re-grants by itself:
// permission resumes only at the next accepted completion edge.
// WINDOW (v3fc2, 2026-10-26): runtime register, reset value TIMEOUT_CYCLES
// (the power-on default, 1000 ms in m7_bd.tcl), writable at 0x44; a written
// value is clamped to [TIMEOUT_MIN_CYCLES, TIMEOUT_MAX_CYCLES] (100 ms / 5000 ms
// in m7_bd.tcl). ANY write to 0x44 disarms (cause 5 if it revokes), exactly like
// FORCE_STOP=1: widening the window can therefore neither re-grant an expired
// permit nor extend a live one; permission resumes only at the next accepted
// completion edge. Intended use: written once at server start, before frames.
//
// The heartbeat is a free-running divider gated by `permit`: a high phase
// only STARTS at a period boundary (so every high pulse is a full
// HB_HALF_CYCLES unless cut short by revocation), and the output register
// goes low on the clock edge right after permit falls (revocation latency
// 1 PL clock). Reset value 0.
//
// 0x00 MAGIC            RO 0x4D37504D ("M7PM")
// 0x04 CTRL             RW b0 FORCE_STOP (software emergency stop, level)
// 0x08 NEXT_FRAME_ID    RW id the next completion edge is judged with
//                          (ignored when EXT_FRAME_ID=1: id comes from frame_id_i)
// 0x0C CAPTURE_AGE      RW capture-to-completion age in PL cycles added at the
//                          next accepted edge (0 = count from completion only)
// 0x10 STATUS           RO b0 permit b1 heartbeat pin b2 armed b3 force_stop
//                          b4 last_id_valid b5 fresh b11:8 last_cause
// 0x14 AGE_CYCLES       RO current age (saturates at WINDOW)
// 0x18 LAST_FRAME_ID    RO newest id seen at a completion edge
// 0x1C FRAMES_ACCEPTED  RO saturating
// 0x20 FRAMES_REJECTED  RO saturating
// 0x24 REVOKE_COUNT     RO saturating count of permit 1->0 transitions
// 0x28 LAST_CAUSE       RO 0 none-since-reset 1 timeout 2 force 3 replay 4 too-old
//                          5 window written (v3fc2)
// 0x2C/0x30 LAST_DONE_TS lo/hi    RO timebase at the last accepted edge
// 0x34/0x38 LAST_REVOKE_TS lo/hi  RO timebase at the last revocation
// 0x3C IRQ_STATUS       RW1C b0 revoked b1 frame rejected b2 frame accepted
// 0x40 IRQ_ENABLE       RW same bits; irq = |(IRQ_STATUS & IRQ_ENABLE), level
// 0x44 WINDOW_CYCLES    RW freshness window in PL cycles; reset TIMEOUT_CYCLES;
//                          write clamped to [TIMEOUT_MIN_CYCLES, TIMEOUT_MAX_CYCLES]
//                          and disarms (v3fc2; RO parameter before)
// 0x48 HB_HALF_CYCLES   RO parameter
// 0x4C TIMEBASE_LO      RO shared timebase [31:0]; reading it latches [63:32]
// 0x50 TIMEBASE_HI      RO latched [63:32]
// 0x54 CONFIG           RO b0 EXT_FRAME_ID
// 0x58 WINDOW_MIN       RO TIMEOUT_MIN_CYCLES (v3fc2)
// 0x5C WINDOW_MAX       RO TIMEOUT_MAX_CYCLES (v3fc2)
module m7_permit_axil #(
    parameter integer TIMEOUT_CYCLES     = 100000000,   // power-on window (1 s at 100 MHz)
    parameter integer TIMEOUT_MIN_CYCLES = 10000000,    // write clamp low  (100 ms)
    parameter integer TIMEOUT_MAX_CYCLES = 500000000,   // write clamp high (5 s)
    parameter integer HB_HALF_CYCLES = 500000,
    parameter integer EXT_FRAME_ID   = 0
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
    input  wire        frame_done_i,
    input  wire [31:0] frame_id_i,
    input  wire [63:0] timebase_i,
    output reg         permit_hb_o,
    output wire        permit_o,
    output wire        irq
);
    localparam [31:0] MAGIC   = 32'h4D37504D;
    localparam [31:0] TIMEOUT = TIMEOUT_CYCLES;          // reset value of `window`
    localparam [31:0] WMIN    = TIMEOUT_MIN_CYCLES;
    localparam [31:0] WMAX    = TIMEOUT_MAX_CYCLES;
    localparam [3:0] C_WINDOW = 4'd5;
    localparam [31:0] HALF    = HB_HALF_CYCLES;
    localparam [31:0] PERIOD  = 2 * HB_HALF_CYCLES;
    // The heartbeat divider only ever holds 0..PERIOD-1 (reset 0, wraps at PERIOD-1),
    // so $clog2(PERIOD) bits suffice (20 at 100 MHz) instead of 32. Port behaviour is
    // unchanged (PERMIT0 2026-11-03: lockstep-equivalent to 8880589).
    localparam integer HBW = (PERIOD > 1) ? $clog2(PERIOD) : 1;
    localparam [3:0] C_NONE = 4'd0, C_TIMEOUT = 4'd1, C_FORCE = 4'd2,
                     C_REPLAY = 4'd3, C_OLD = 4'd4;

    generate
        if (TIMEOUT_MIN_CYCLES < 2 || HB_HALF_CYCLES < 1 ||
            TIMEOUT_MIN_CYCLES > TIMEOUT_CYCLES || TIMEOUT_CYCLES > TIMEOUT_MAX_CYCLES ||
            TIMEOUT != TIMEOUT_CYCLES || WMIN != TIMEOUT_MIN_CYCLES || WMAX != TIMEOUT_MAX_CYCLES ||
            PERIOD != 2 * HB_HALF_CYCLES) begin : gen_param_error
            // Verilog-2001 elaboration guard: an undefined module stops elaboration
            // (synthesis and simulation alike) instead of a message nobody reads.
            m7_permit_axil_bad_TIMEOUT_or_HB_HALF_parameter u_param_error ();
        end
    endgenerate

    // ---------------- AXI-lite handshake (same scheme as m7_watchdog_axil) ----------------
    wire do_write = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
    assign s_axi_awready = do_write;
    assign s_axi_wready  = do_write;
    assign s_axi_bresp   = 2'b00;
    wire do_read = s_axi_arvalid && !s_axi_rvalid;
    assign s_axi_arready = do_read;
    assign s_axi_rresp   = 2'b00;
    wire [13:0] waddr = s_axi_awaddr[15:2];
    wire [13:0] raddr = s_axi_araddr[15:2];
    wire wr_ctrl  = do_write && waddr == 14'h0001;
    wire wr_win   = do_write && waddr == 14'h0011;
    wire [2:0] irq_clear = (do_write && waddr == 14'h000F) ? s_axi_wdata[2:0] : 3'd0;

    // ---------------- state ----------------
    reg        force_stop;
    reg [31:0] window;          // freshness window, cycles (0x44)
    reg [31:0] next_frame_id, capture_age;
    reg        armed;
    reg [31:0] age;
    reg [31:0] last_id;
    reg        last_valid;
    reg [31:0] accepted_count, rejected_count, revoke_count;
    reg [3:0]  last_cause, disarm_reason;
    reg [63:0] last_done_ts, last_revoke_ts;
    reg [31:0] tb_hi_latch;
    reg [2:0]  irq_status, irq_enable;
    reg        done_d, permit_d;
    reg [HBW-1:0] hb_cnt;

    // The ONLY freshness source: a rising edge of the frame-completion line.
    wire refresh_evt = frame_done_i && !done_d;

    wire [31:0] id_now   = (EXT_FRAME_ID != 0) ? frame_id_i : next_frame_id;
    wire        id_newer = !last_valid || (id_now > last_id);
    wire        age_ok   = capture_age < window;
    wire        force_next = wr_ctrl ? s_axi_wdata[0] : force_stop;
    wire        accept   = refresh_evt && id_newer && age_ok && !force_next && !wr_win;
    wire        reject   = refresh_evt && !(id_newer && age_ok);

    wire fresh  = armed && (age < window);
    wire permit = fresh && !force_stop;
    assign permit_o = permit;
    assign irq = |(irq_status & irq_enable);

    function [31:0] sat_inc(input [31:0] v);
        sat_inc = (v == 32'hFFFFFFFF) ? v : v + 32'd1;
    endfunction

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            force_stop <= 1'b0;
            window <= TIMEOUT;
            next_frame_id <= 32'd0;
            capture_age <= 32'd0;
            armed <= 1'b0;
            age <= TIMEOUT;
            last_id <= 32'd0;
            last_valid <= 1'b0;
            accepted_count <= 32'd0;
            rejected_count <= 32'd0;
            revoke_count <= 32'd0;
            last_cause <= C_NONE;
            disarm_reason <= C_NONE;
            last_done_ts <= 64'd0;
            last_revoke_ts <= 64'd0;
            tb_hi_latch <= 32'd0;
            irq_status <= 3'd0;
            irq_enable <= 3'd0;
            done_d <= 1'b0;
            permit_d <= 1'b0;
            hb_cnt <= {HBW{1'b0}};
            permit_hb_o <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata <= 32'd0;
        end else begin
            done_d <= frame_done_i;
            permit_d <= permit;

            // ---- heartbeat: free-running divider, gated, full pulses only ----
            hb_cnt <= (hb_cnt == PERIOD - 1) ? {HBW{1'b0}} : hb_cnt + 1'b1;
            permit_hb_o <= permit && (hb_cnt < HALF) && (permit_hb_o || hb_cnt == {HBW{1'b0}});

            // ---- age ----
            if (armed && age < window) age <= age + 32'd1;

            // ---- completion edge ----
            if (refresh_evt) begin
                if (id_newer) begin
                    last_id <= id_now;
                    last_valid <= 1'b1;
                end
                if (accept) begin
                    armed <= 1'b1;
                    age <= capture_age;
                    accepted_count <= sat_inc(accepted_count);
                    last_done_ts <= timebase_i;
                end else if (reject) begin
                    armed <= 1'b0;
                    disarm_reason <= !id_newer ? C_REPLAY : C_OLD;
                    rejected_count <= sat_inc(rejected_count);
                end
                // refresh while FORCE_STOP: id recorded, not armed, not a reject.
            end

            // ---- software emergency stop (writes may only revoke) ----
            if (wr_ctrl) begin
                force_stop <= s_axi_wdata[0];
                if (s_axi_wdata[0]) begin
                    armed <= 1'b0;
                    disarm_reason <= C_FORCE;
                end
            end

            // ---- freshness window (clamped; a write disarms like FORCE_STOP) ----
            if (wr_win) begin
                window <= (s_axi_wdata < WMIN) ? WMIN :
                          (s_axi_wdata > WMAX) ? WMAX : s_axi_wdata;
                armed <= 1'b0;
                if (!force_stop) disarm_reason <= C_WINDOW;
            end

            // ---- revocation bookkeeping (permit 1 -> 0) ----
            if (permit_d && !permit) begin
                revoke_count <= sat_inc(revoke_count);
                last_cause <= force_stop ? C_FORCE : (!armed ? disarm_reason : C_TIMEOUT);
                last_revoke_ts <= timebase_i;
            end

            irq_status <= (irq_status & ~irq_clear) |
                          {accept, reject, permit_d && !permit};

            // ---- AXI writes ----
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
            if (do_write) begin
                s_axi_bvalid <= 1'b1;
                case (waddr)
                    14'h0002: next_frame_id <= s_axi_wdata;
                    14'h0003: capture_age <= s_axi_wdata;
                    14'h0010: irq_enable <= s_axi_wdata[2:0];
                    default: ;
                endcase
            end

            // ---- AXI reads ----
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
            if (do_read) begin
                s_axi_rvalid <= 1'b1;
                case (raddr)
                    14'h0000: s_axi_rdata <= MAGIC;
                    14'h0001: s_axi_rdata <= {31'd0, force_stop};
                    14'h0002: s_axi_rdata <= next_frame_id;
                    14'h0003: s_axi_rdata <= capture_age;
                    14'h0004: s_axi_rdata <= {20'd0, last_cause, 2'd0, fresh, last_valid,
                                              force_stop, armed, permit_hb_o, permit};
                    14'h0005: s_axi_rdata <= age;
                    14'h0006: s_axi_rdata <= last_id;
                    14'h0007: s_axi_rdata <= accepted_count;
                    14'h0008: s_axi_rdata <= rejected_count;
                    14'h0009: s_axi_rdata <= revoke_count;
                    14'h000A: s_axi_rdata <= {28'd0, last_cause};
                    14'h000B: s_axi_rdata <= last_done_ts[31:0];
                    14'h000C: s_axi_rdata <= last_done_ts[63:32];
                    14'h000D: s_axi_rdata <= last_revoke_ts[31:0];
                    14'h000E: s_axi_rdata <= last_revoke_ts[63:32];
                    14'h000F: s_axi_rdata <= {29'd0, irq_status};
                    14'h0010: s_axi_rdata <= {29'd0, irq_enable};
                    14'h0011: s_axi_rdata <= window;
                    14'h0012: s_axi_rdata <= HALF;
                    14'h0013: begin
                        s_axi_rdata <= timebase_i[31:0];
                        tb_hi_latch <= timebase_i[63:32];
                    end
                    14'h0014: s_axi_rdata <= tb_hi_latch;
                    14'h0015: s_axi_rdata <= (EXT_FRAME_ID != 0) ? 32'd1 : 32'd0;
                    14'h0016: s_axi_rdata <= WMIN;
                    14'h0017: s_axi_rdata <= WMAX;
                    default:  s_axi_rdata <= 32'd0;
                endcase
            end
        end
    end
endmodule
