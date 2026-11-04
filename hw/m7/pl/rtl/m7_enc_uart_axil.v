// Wheel-encoder UART receiver + wheel speed / odometry, AXI4-Lite, 2026-10-11.
//
// Link contract (agreed with the chassis MCU firmware, do not change here):
//   115200 8N1, MCU -> KV260 only. One 15-byte frame every 10 ms:
//   0xAA 0x55 | seq u8 | mcu_ms u16 LE | left i32 LE | right i32 LE | flags u8 | cksum u8
//   left/right = cumulative signed encoder counts; flags b0 = MCU sees heartbeat,
//   b1 = motor enabled; cksum = (sum of the 12 bytes seq..flags) mod 256.
//
// Receive path: 2-FF synchronizer -> mid-bit sampling UART (start bit re-checked
// at half a bit, stop bit checked) -> frame FSM. Resync rules: hunting for
// 0xAA 0x55 (a repeated 0xAA keeps the second-byte hunt), any receive error or
// an inter-byte gap of BYTE_GAP_CYCLES inside a frame aborts it, a bad checksum
// drops it; after any of these the FSM hunts again from the next byte. MCU
// frames are separated by ~8.7 ms of idle, so the gap rule guarantees a resync
// at every frame boundary at the latest.
//
// Every checksum-valid frame is stamped with the shared PL timebase at the
// moment its last stop bit is sampled (same counter as the permit block's
// depth-frame completion stamp). seq handling: +1 normal; +0 duplicate (counted,
// ignored); +k>1 gap (k-1 frames counted lost).
//
// Odometry (fixed point; the Python model tools/enc_uart_model.py is the
// normative definition, the spec gives widths and quantization errors):
//   K_Q16   = round(PI_Q30 * D_um / (CPR * 2^14))         um per count, Q16.16
//   dX      = (X - X_prev) mod 2^32 as int32               counts since last frame
//   jump    = |dL| > MAX_DELTA or |dR| > MAX_DELTA         (MCU reset etc.: no accumulate)
//   cumX   += dX  (48-bit)                                 total counts since clear
//   distX   = sgn(cumX) * ((|cumX| * K_Q16) >> 16)         um, int64
//   vX      = sgn(dX) * min(2^31-1, floor(|dX|*K_Q16*CLK_KHZ / dt))   mm/s Q16.16
//             (0 and speed_valid=0 on first frame, jump, or dt > MAX_DT)
//   vC, distC = (L + R) / 2 truncated toward zero
//   heading = sgn(d) * min(2^31-1, floor(|d| * 2^16 / TRACK_um)), d = distR - distL
//             rad Q16.16 (0 and flag when TRACK_um = 0)
// All multiplies/divides run on one shared shift-add multiplier / restoring
// divider (no DSP), ~500 cycles per frame against ~1e6 cycles between frames.
//
// Register map (32-bit, byte offsets). Multi-register values are made
// consistent by UPDATE_COUNT (seqlock): read it, read the values, read it again,
// retry if it changed.
// 0x00 MAGIC 0x4D37454E ("M7EN")
// 0x04 CTRL  WO pulses b0 PARAM_APPLY b1 CLEAR_ODOM b2 CLEAR_COUNTERS
// 0x08 WHEEL_DIAM_UM  RW   0x0C COUNTS_PER_REV RW   0x10 TRACK_UM RW
// 0x14 MAX_DELTA_COUNTS RW (reset 32767)   0x18 MAX_DT_CYCLES RW (reset MAX_DT_DEFAULT)
// 0x1C K_Q16 RO
// 0x20 STATUS RO b0 link_ok b1 param_valid b2 param_err b3 busy b4 link_seen
// 0x24 UPDATE_COUNT RO
// 0x28/0x2C FRAME_TS lo/hi RO        0x30 SEQ RO {mcu_ms[31:16], flags[15:8], seq[7:0]}
// 0x34 LEFT_COUNT RO  0x38 RIGHT_COUNT RO
// 0x3C ODOM_FLAGS RO b0 speed_valid b1 first b2 jump b3 heading_invalid
//                    b4 param_valid b5 dt_too_long
// 0x40 V_LEFT 0x44 V_RIGHT 0x48 V_CENTER (mm/s Q16.16, int32)
// 0x4C/0x50 DIST_LEFT_UM 0x54/0x58 DIST_RIGHT_UM 0x5C/0x60 DIST_CENTER_UM (int64 lo/hi)
// 0x64 HEADING_Q16 (rad Q16.16)  0x68 DT_CYCLES (saturated to 32 bits)
// 0x6C GOOD_FRAMES 0x70 BAD_CHECKSUM 0x74 SEQ_GAP_EVENTS 0x78 LOST_FRAMES
// 0x7C DUP_FRAMES 0x80 PARTIAL_ABORTS 0x84 FRAMING_ERRORS 0x88 SYNC_SKIPPED_BYTES
// 0x8C LINK_TIMEOUTS 0x90 START_GLITCHES 0x94 COUNT_JUMPS   (all RO, saturating)
// 0x98 IRQ_STATUS RW1C b0 odometry updated b1 link timeout b2 receive error
// 0x9C IRQ_ENABLE RW
// 0xA0 CLK_HZ RO 0xA4 BAUD_DIV RO 0xA8 LINK_TIMEOUT_CYCLES RO 0xAC BYTE_GAP_CYCLES RO
// 0xB0 TIMEBASE_LO RO (latches hi) 0xB4 TIMEBASE_HI RO
// 0xB8/0xBC CUM_LEFT_COUNTS lo/hi(sign-extended) 0xC0/0xC4 CUM_RIGHT_COUNTS
module m7_enc_uart_axil #(
    parameter integer CLK_HZ              = 100000000,
    parameter integer BAUD                = 115200,
    parameter integer BYTE_GAP_CYCLES     = 100000,    // 1 ms
    parameter integer LINK_TIMEOUT_CYCLES = 5000000,   // 50 ms = 5 frame periods
    parameter integer MAX_DT_DEFAULT      = 10000000   // 100 ms
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
    input  wire        uart_rx_i,
    input  wire [63:0] timebase_i,
    output wire        irq
);
    localparam [31:0] MAGIC   = 32'h4D37454E;
    localparam integer DIV    = (CLK_HZ + BAUD / 2) / BAUD;
    localparam [31:0] CLK_KHZ = CLK_HZ / 1000;
    localparam [31:0] PI_Q30  = 32'd3373259426;   // round(pi * 2^30)
    localparam [31:0] GAP     = BYTE_GAP_CYCLES;
    localparam [31:0] LINK_TO = LINK_TIMEOUT_CYCLES;

    generate
        if (DIV < 8 || DIV > 65535 || (CLK_HZ % 1000) != 0 || BYTE_GAP_CYCLES < 12 * DIV ||
            LINK_TIMEOUT_CYCLES < 2) begin : gen_param_error
            // Verilog-2001 elaboration guard (undefined module stops elaboration).
            m7_enc_uart_axil_bad_CLK_BAUD_or_GAP_parameter u_param_error ();
        end
    endgenerate

    function [31:0] sat_inc(input [31:0] v);
        sat_inc = (v == 32'hFFFFFFFF) ? v : v + 32'd1;
    endfunction
    function [31:0] sat_add(input [31:0] v, input [31:0] a);
        reg [32:0] s;
        begin
            s = {1'b0, v} + {1'b0, a};
            sat_add = s[32] ? 32'hFFFFFFFF : s[31:0];
        end
    endfunction

    // ================= AXI-lite handshake (same scheme as m7_watchdog_axil) =================
    wire do_write = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
    assign s_axi_awready = do_write;
    assign s_axi_wready  = do_write;
    assign s_axi_bresp   = 2'b00;
    wire do_read = s_axi_arvalid && !s_axi_rvalid;
    assign s_axi_arready = do_read;
    assign s_axi_rresp   = 2'b00;
    wire [13:0] waddr = s_axi_awaddr[15:2];
    wire [13:0] raddr = s_axi_araddr[15:2];
    wire wr_ctrl = do_write && waddr == 14'h0001;
    wire ctrl_param_apply = wr_ctrl && s_axi_wdata[0];
    wire ctrl_clear_odom  = wr_ctrl && s_axi_wdata[1];
    wire ctrl_clear_cnt   = wr_ctrl && s_axi_wdata[2];

    // ================= UART receiver =================
    reg rx_s1, rx_s2;
    localparam [2:0] R_IDLE = 3'd0, R_START = 3'd1, R_DATA = 3'd2, R_STOP = 3'd3, R_WAITHI = 3'd4;
    reg [2:0]  rstate;
    reg [15:0] rcnt;
    reg [2:0]  bitn;
    reg [7:0]  rsh;
    reg        byte_valid, framing_err, start_glitch;
    reg [7:0]  byte_data;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            rx_s1 <= 1'b1; rx_s2 <= 1'b1;
            rstate <= R_IDLE; rcnt <= 16'd0; bitn <= 3'd0; rsh <= 8'd0;
            byte_valid <= 1'b0; framing_err <= 1'b0; start_glitch <= 1'b0; byte_data <= 8'd0;
        end else begin
            rx_s1 <= uart_rx_i;
            rx_s2 <= rx_s1;
            byte_valid <= 1'b0; framing_err <= 1'b0; start_glitch <= 1'b0;
            case (rstate)
                R_IDLE: if (!rx_s2) begin rstate <= R_START; rcnt <= DIV / 2 - 1; end
                R_START: if (rcnt == 0) begin
                        if (!rx_s2) begin rstate <= R_DATA; rcnt <= DIV - 1; bitn <= 3'd0; end
                        else begin rstate <= R_IDLE; start_glitch <= 1'b1; end
                    end else rcnt <= rcnt - 16'd1;
                R_DATA: if (rcnt == 0) begin
                        rsh <= {rx_s2, rsh[7:1]};
                        rcnt <= DIV - 1;
                        if (bitn == 3'd7) rstate <= R_STOP;
                        bitn <= bitn + 3'd1;
                    end else rcnt <= rcnt - 16'd1;
                R_STOP: if (rcnt == 0) begin
                        if (rx_s2) begin byte_valid <= 1'b1; byte_data <= rsh; rstate <= R_IDLE; end
                        else begin framing_err <= 1'b1; rstate <= R_WAITHI; end
                    end else rcnt <= rcnt - 16'd1;
                default: if (rx_s2) rstate <= R_IDLE;   // R_WAITHI: wait out a break
            endcase
        end
    end

    // ================= frame FSM =================
    localparam [1:0] P_HUNT1 = 2'd0, P_HUNT2 = 2'd1, P_BODY = 2'd2, P_CK = 2'd3;
    reg [1:0]  pstate;
    reg [3:0]  pidx;
    reg [95:0] body;          // bytes seq..flags, byte 0 in [7:0]
    reg [7:0]  psum;
    reg [31:0] gap_cnt;
    wire [7:0]  cksum_calc = psum;
    wire [95:0] body_full  = {byte_data, body[95:8]};

    // counters
    reg [31:0] c_good, c_bad, c_gapev, c_lost, c_dup, c_partial, c_framing, c_skip,
               c_linkto, c_glitch, c_jump;

    // link state
    reg        link_seen;
    reg [31:0] link_cnt;
    wire       link_ok = link_seen && (link_cnt < LINK_TO);
    reg        link_ok_d;

    // seq tracking
    reg        have_seq;
    reg [7:0]  last_seq;

    // job handed to the odometry sequencer
    reg        job_pending;
    reg [63:0] job_ts;
    reg [7:0]  job_seq, job_flags;
    reg [15:0] job_ms;
    reg [31:0] job_left, job_right;

    reg        good_evt, rx_err_evt;

    wire        frame_good = byte_valid && pstate == P_CK && byte_data == cksum_calc;
    wire [7:0]  fseq = body[7:0];
    wire [7:0]  seq_d = fseq - last_seq;
    wire        seq_dup = have_seq && seq_d == 8'd0;

    reg job_take;   // from sequencer

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            pstate <= P_HUNT1; pidx <= 4'd0; body <= 96'd0; psum <= 8'd0; gap_cnt <= 32'd0;
            c_good <= 0; c_bad <= 0; c_gapev <= 0; c_lost <= 0; c_dup <= 0; c_partial <= 0;
            c_framing <= 0; c_skip <= 0; c_linkto <= 0; c_glitch <= 0;
            link_seen <= 1'b0; link_cnt <= 32'd0; link_ok_d <= 1'b0;
            have_seq <= 1'b0; last_seq <= 8'd0;
            job_pending <= 1'b0; job_ts <= 64'd0; job_seq <= 8'd0; job_flags <= 8'd0;
            job_ms <= 16'd0; job_left <= 32'd0; job_right <= 32'd0;
            good_evt <= 1'b0; rx_err_evt <= 1'b0;
        end else begin
            good_evt <= 1'b0; rx_err_evt <= 1'b0;
            if (job_take) job_pending <= 1'b0;

            // link timer
            if (link_seen && link_cnt < LINK_TO) link_cnt <= link_cnt + 32'd1;
            link_ok_d <= link_ok;
            if (link_ok_d && !link_ok) c_linkto <= sat_inc(c_linkto);

            if (start_glitch) c_glitch <= sat_inc(c_glitch);

            // inter-byte gap inside a frame
            if (pstate != P_HUNT1 && !byte_valid && !framing_err) begin
                if (gap_cnt >= GAP - 1) begin
                    pstate <= P_HUNT1;
                    c_partial <= sat_inc(c_partial);
                    rx_err_evt <= 1'b1;
                end else gap_cnt <= gap_cnt + 32'd1;
            end

            if (framing_err) begin
                c_framing <= sat_inc(c_framing);
                rx_err_evt <= 1'b1;
                pstate <= P_HUNT1;
            end else if (byte_valid) begin
                gap_cnt <= 32'd0;
                case (pstate)
                    P_HUNT1: if (byte_data == 8'hAA) pstate <= P_HUNT2;
                             else c_skip <= sat_inc(c_skip);
                    P_HUNT2: if (byte_data == 8'h55) begin
                                 pstate <= P_BODY; pidx <= 4'd0; psum <= 8'd0;
                             end else if (byte_data == 8'hAA) begin
                                 c_skip <= sat_inc(c_skip);               // earlier 0xAA was junk
                             end else begin
                                 pstate <= P_HUNT1; c_skip <= sat_add(c_skip, 32'd2);
                             end
                    P_BODY: begin
                        body <= body_full;
                        psum <= psum + byte_data;
                        pidx <= pidx + 4'd1;
                        if (pidx == 4'd11) pstate <= P_CK;
                    end
                    default: begin   // P_CK
                        pstate <= P_HUNT1;
                        if (byte_data == cksum_calc) begin
                            c_good <= sat_inc(c_good);
                            link_seen <= 1'b1;
                            link_cnt <= 32'd0;
                            have_seq <= 1'b1;
                            last_seq <= fseq;
                            if (seq_dup) begin
                                c_dup <= sat_inc(c_dup);
                            end else begin
                                if (have_seq && seq_d != 8'd1) begin
                                    c_gapev <= sat_inc(c_gapev);
                                    c_lost <= sat_add(c_lost, {24'd0, seq_d - 8'd1});
                                end
                                job_pending <= 1'b1;
                                job_ts <= timebase_i;
                                job_seq <= fseq;
                                job_ms <= body[23:8];
                                job_left <= body[55:24];
                                job_right <= body[87:56];
                                job_flags <= body[95:88];
                                good_evt <= 1'b1;
                            end
                        end else begin
                            c_bad <= sat_inc(c_bad);
                            rx_err_evt <= 1'b1;
                        end
                    end
                endcase
            end

            if (ctrl_clear_cnt) begin
                c_good <= 0; c_bad <= 0; c_gapev <= 0; c_lost <= 0; c_dup <= 0; c_partial <= 0;
                c_framing <= 0; c_skip <= 0; c_linkto <= 0; c_glitch <= 0;
            end
        end
    end

    // ================= odometry sequencer with shared shift-add MUL / restoring DIV =================
    reg [31:0] p_diam, p_cpr, p_track, p_max_delta, p_max_dt;
    reg [31:0] k_q16;
    reg        param_valid, param_err;
    reg        param_pending, clear_pending;

    // arithmetic engine
    reg        ar_run, ar_div;
    reg [6:0]  ar_it;
    reg [95:0] ar_acc, ar_mcand;
    reg [31:0] ar_mplier;
    reg [95:0] ar_dvd, ar_quo;
    reg [47:0] ar_dvs;
    reg [48:0] ar_rem;
    wire [48:0] ar_rem_next = {ar_rem[47:0], ar_dvd[95]};
    wire        ar_ge = ar_rem_next >= {1'b0, ar_dvs};

    // odometry state
    reg        have_prev;
    reg [31:0] prev_l, prev_r;
    reg [63:0] prev_ts;
    reg [47:0] cum_l, cum_r;
    // working
    reg [31:0] w_dl, w_dr;
    reg [63:0] w_dt;
    reg        w_first, w_jump, w_speed_ok, w_dt_long;
    reg [63:0] w_dist_l, w_dist_r;
    reg [31:0] w_v_l, w_v_r, w_head;
    reg        w_head_bad;
    reg [31:0] w_track, w_k;     // parameters frozen when the frame is taken
    reg [63:0] w_ts;
    reg [7:0]  w_seq, w_flags;
    reg [15:0] w_ms;
    reg [31:0] w_left, w_right;
    // published (seqlock by o_update)
    reg [31:0] o_update;
    reg [63:0] o_ts;
    reg [7:0]  o_seq, o_flags;
    reg [15:0] o_ms;
    reg [31:0] o_left, o_right, o_v_l, o_v_r, o_v_c, o_head, o_dt;
    reg [63:0] o_dist_l, o_dist_r, o_dist_c;
    reg [5:0]  o_oflags;
    reg [47:0] o_cum_l, o_cum_r;
    reg        odom_evt;

    localparam [4:0] S_IDLE = 5'd0, S_K_MUL = 5'd1, S_K_DIV = 5'd2, S_K_END = 5'd3,
                     S_F0 = 5'd4, S_F1 = 5'd5, S_DL = 5'd6, S_DR = 5'd7, S_VL1 = 5'd8,
                     S_VL2 = 5'd9, S_VL3 = 5'd10, S_VR1 = 5'd11, S_VR2 = 5'd12,
                     S_VR3 = 5'd13, S_HD = 5'd14, S_COMMIT = 5'd15, S_WAIT = 5'd16,
                     S_VL_END = 5'd17, S_VR_END = 5'd18, S_HD_END = 5'd19;
    reg [4:0] sstate, sret;

    // helpers
    function [31:0] abs32(input [31:0] v);
        abs32 = v[31] ? (~v + 32'd1) : v;
    endfunction
    function [47:0] abs48(input [47:0] v);
        abs48 = v[47] ? (~v + 48'd1) : v;
    endfunction
    function [31:0] sat31(input [95:0] q);
        sat31 = (|q[95:31]) ? 32'h7FFFFFFF : q[31:0];
    endfunction
    function [31:0] neg32(input [31:0] v, input n);
        neg32 = n ? (~v + 32'd1) : v;
    endfunction
    function [63:0] neg64(input [63:0] v, input n);
        neg64 = n ? (~v + 64'd1) : v;
    endfunction

    wire [32:0] vsum   = {w_v_l[31], w_v_l} + {w_v_r[31], w_v_r};
    wire [32:0] vsum_m = vsum[32] ? (~vsum + 33'd1) : vsum;
    wire [31:0] vc     = neg32(vsum_m[32:1], vsum[32]);
    wire [64:0] dsum   = {w_dist_l[63], w_dist_l} + {w_dist_r[63], w_dist_r};
    wire [64:0] dsum_m = dsum[64] ? (~dsum + 65'd1) : dsum;
    wire [63:0] dc     = neg64(dsum_m[64:1], dsum[64]);
    wire [64:0] ddiff  = {w_dist_r[63], w_dist_r} - {w_dist_l[63], w_dist_l};
    wire [64:0] ddiff_m = ddiff[64] ? (~ddiff + 65'd1) : ddiff;

    wire [31:0] dl_now = job_left - prev_l;
    wire [31:0] dr_now = job_right - prev_r;
    wire [63:0] dt_now = job_ts - prev_ts;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            p_diam <= 0; p_cpr <= 0; p_track <= 0; p_max_delta <= 32'd32767;
            p_max_dt <= MAX_DT_DEFAULT; k_q16 <= 0; param_valid <= 0; param_err <= 0;
            param_pending <= 0; clear_pending <= 0;
            ar_run <= 0; ar_div <= 0; ar_it <= 0; ar_acc <= 0; ar_mcand <= 0; ar_mplier <= 0;
            ar_dvd <= 0; ar_quo <= 0; ar_dvs <= 0; ar_rem <= 0;
            have_prev <= 0; prev_l <= 0; prev_r <= 0; prev_ts <= 0; cum_l <= 0; cum_r <= 0;
            w_dl <= 0; w_dr <= 0; w_dt <= 0; w_first <= 0; w_jump <= 0; w_speed_ok <= 0; w_dt_long <= 0;
            w_dist_l <= 0; w_dist_r <= 0; w_v_l <= 0; w_v_r <= 0; w_head <= 0; w_head_bad <= 0;
            w_track <= 0; w_k <= 0;
            w_ts <= 0; w_seq <= 0; w_flags <= 0; w_ms <= 0; w_left <= 0; w_right <= 0;
            o_update <= 0; o_ts <= 0; o_seq <= 0; o_flags <= 0; o_ms <= 0; o_left <= 0; o_right <= 0;
            o_v_l <= 0; o_v_r <= 0; o_v_c <= 0; o_head <= 0; o_dt <= 0;
            o_dist_l <= 0; o_dist_r <= 0; o_dist_c <= 0; o_oflags <= 0; o_cum_l <= 0; o_cum_r <= 0;
            odom_evt <= 0; job_take <= 0;
            sstate <= S_IDLE; sret <= S_IDLE;
        end else begin
            odom_evt <= 1'b0;
            job_take <= 1'b0;
            if (ctrl_param_apply) param_pending <= 1'b1;
            if (ctrl_clear_odom)  clear_pending <= 1'b1;
            if (do_write) begin
                case (waddr)
                    14'h0002: p_diam <= s_axi_wdata;
                    14'h0003: p_cpr <= s_axi_wdata;
                    14'h0004: p_track <= s_axi_wdata;
                    14'h0005: p_max_delta <= s_axi_wdata;
                    14'h0006: p_max_dt <= s_axi_wdata;
                    default: ;
                endcase
            end

            // ---- arithmetic engine ----
            if (ar_run) begin
                if (ar_div) begin
                    if (ar_ge) begin
                        ar_rem <= ar_rem_next - {1'b0, ar_dvs};
                        ar_quo <= {ar_quo[94:0], 1'b1};
                    end else begin
                        ar_rem <= ar_rem_next;
                        ar_quo <= {ar_quo[94:0], 1'b0};
                    end
                    ar_dvd <= {ar_dvd[94:0], 1'b0};
                end else begin
                    if (ar_mplier[0]) ar_acc <= ar_acc + ar_mcand;
                    ar_mcand <= {ar_mcand[94:0], 1'b0};
                    ar_mplier <= {1'b0, ar_mplier[31:1]};
                end
                ar_it <= ar_it - 7'd1;
                if (ar_it == 7'd1) ar_run <= 1'b0;
            end

            case (sstate)
                S_IDLE: begin
                    if (clear_pending && !ctrl_clear_odom) begin   // same-cycle new request is kept
                        clear_pending <= 1'b0;
                        have_prev <= 1'b0; cum_l <= 0; cum_r <= 0;
                    end else if (param_pending) begin
                        param_pending <= 1'b0;
                        // MUL PI_Q30 * D_um
                        ar_div <= 1'b0; ar_run <= 1'b1; ar_it <= 7'd32;
                        ar_acc <= 0; ar_mcand <= {64'd0, PI_Q30}; ar_mplier <= p_diam;
                        sstate <= S_WAIT; sret <= S_K_DIV;
                    end else if (job_pending) begin
                        job_take <= 1'b1;
                        sstate <= S_F0;
                    end
                end
                S_WAIT: if (!ar_run) sstate <= sret;
                S_K_DIV: begin
                    if (p_cpr == 0) begin
                        k_q16 <= 0; param_err <= 1'b1; param_valid <= 1'b1; sstate <= S_IDLE;
                    end else begin
                        ar_div <= 1'b1; ar_run <= 1'b1; ar_it <= 7'd96;
                        ar_dvd <= ar_acc + ({64'd0, p_cpr} << 13);
                        ar_dvs <= {16'd0, p_cpr} << 14;
                        ar_rem <= 0; ar_quo <= 0;
                        sstate <= S_WAIT; sret <= S_K_END;
                    end
                end
                S_K_END: begin
                    param_valid <= 1'b1;
                    if (|ar_quo[95:32]) begin k_q16 <= 32'hFFFFFFFF; param_err <= 1'b1; end
                    else begin k_q16 <= ar_quo[31:0]; param_err <= (p_diam == 0); end
                    sstate <= S_IDLE;
                end
                S_F0: begin
                    w_ts <= job_ts; w_seq <= job_seq; w_flags <= job_flags; w_ms <= job_ms;
                    w_left <= job_left; w_right <= job_right;
                    w_dl <= dl_now; w_dr <= dr_now; w_dt <= dt_now;
                    w_first <= !have_prev;
                    // A frame is computed with the parameters valid when it is
                    // taken (S_F0); a register write landing while the ~500-cycle
                    // computation runs only affects later frames.
                    w_track <= p_track; w_k <= k_q16;
                    w_jump <= have_prev && (abs32(dl_now) > p_max_delta || abs32(dr_now) > p_max_delta);
                    w_dt_long <= have_prev && (dt_now > {32'd0, p_max_dt});
                    sstate <= S_F1;
                end
                S_F1: begin
                    w_speed_ok <= !w_first && !w_jump && !w_dt_long;
                    if (!w_first && !w_jump) begin
                        cum_l <= cum_l + {{16{w_dl[31]}}, w_dl};
                        cum_r <= cum_r + {{16{w_dr[31]}}, w_dr};
                    end
                    have_prev <= 1'b1; prev_l <= w_left; prev_r <= w_right; prev_ts <= w_ts;
                    sstate <= S_DL;
                end
                S_DL: begin   // |cum_l| * K
                    ar_div <= 1'b0; ar_run <= 1'b1; ar_it <= 7'd32;
                    ar_acc <= 0; ar_mcand <= {48'd0, abs48(cum_l)}; ar_mplier <= w_k;
                    sstate <= S_WAIT; sret <= S_DR;
                end
                S_DR: begin
                    w_dist_l <= neg64(ar_acc[79:16], cum_l[47]);
                    ar_div <= 1'b0; ar_run <= 1'b1; ar_it <= 7'd32;
                    ar_acc <= 0; ar_mcand <= {48'd0, abs48(cum_r)}; ar_mplier <= w_k;
                    sstate <= S_WAIT; sret <= S_VL1;
                end
                S_VL1: begin
                    w_dist_r <= neg64(ar_acc[79:16], cum_r[47]);
                    if (!w_speed_ok) begin
                        w_v_l <= 0; w_v_r <= 0; sstate <= S_HD;
                    end else begin
                        ar_div <= 1'b0; ar_run <= 1'b1; ar_it <= 7'd32;
                        ar_acc <= 0; ar_mcand <= {64'd0, abs32(w_dl)}; ar_mplier <= w_k;
                        sstate <= S_WAIT; sret <= S_VL2;
                    end
                end
                S_VL2: begin
                    ar_div <= 1'b0; ar_run <= 1'b1; ar_it <= 7'd32;
                    ar_acc <= 0; ar_mcand <= {32'd0, ar_acc[63:0]}; ar_mplier <= CLK_KHZ;
                    sstate <= S_WAIT; sret <= S_VL3;
                end
                S_VL3: begin
                    ar_div <= 1'b1; ar_run <= 1'b1; ar_it <= 7'd96;
                    ar_dvd <= ar_acc; ar_dvs <= w_dt[47:0]; ar_rem <= 0; ar_quo <= 0;
                    sstate <= S_WAIT; sret <= S_VL_END;
                end
                S_VL_END: begin
                    w_v_l <= neg32(sat31(ar_quo), w_dl[31]);
                    ar_div <= 1'b0; ar_run <= 1'b1; ar_it <= 7'd32;
                    ar_acc <= 0; ar_mcand <= {64'd0, abs32(w_dr)}; ar_mplier <= w_k;
                    sstate <= S_WAIT; sret <= S_VR2;
                end
                S_VR2: begin
                    ar_div <= 1'b0; ar_run <= 1'b1; ar_it <= 7'd32;
                    ar_acc <= 0; ar_mcand <= {32'd0, ar_acc[63:0]}; ar_mplier <= CLK_KHZ;
                    sstate <= S_WAIT; sret <= S_VR3;
                end
                S_VR3: begin
                    ar_div <= 1'b1; ar_run <= 1'b1; ar_it <= 7'd96;
                    ar_dvd <= ar_acc; ar_dvs <= w_dt[47:0]; ar_rem <= 0; ar_quo <= 0;
                    sstate <= S_WAIT; sret <= S_VR_END;
                end
                S_VR_END: begin
                    w_v_r <= neg32(sat31(ar_quo), w_dr[31]);
                    sstate <= S_HD;
                end
                S_HD: begin
                    if (w_track == 0) begin
                        w_head <= 0; w_head_bad <= 1'b1; sstate <= S_COMMIT;
                    end else begin
                        w_head_bad <= 1'b0;
                        ar_div <= 1'b1; ar_run <= 1'b1; ar_it <= 7'd96;
                        ar_dvd <= {15'd0, ddiff_m, 16'd0}; ar_dvs <= {16'd0, w_track};
                        ar_rem <= 0; ar_quo <= 0;
                        sstate <= S_WAIT; sret <= S_HD_END;
                    end
                end
                S_HD_END: begin
                    w_head <= neg32(sat31(ar_quo), ddiff[64]);
                    sstate <= S_COMMIT;
                end
                default: begin  // S_COMMIT: publish everything in one clock
                    o_ts <= w_ts; o_seq <= w_seq; o_flags <= w_flags; o_ms <= w_ms;
                    o_left <= w_left; o_right <= w_right;
                    o_v_l <= w_v_l; o_v_r <= w_v_r; o_v_c <= vc;
                    o_dist_l <= w_dist_l; o_dist_r <= w_dist_r; o_dist_c <= dc;
                    o_head <= w_head;
                    o_dt <= w_first ? 32'd0 : ((|w_dt[63:32]) ? 32'hFFFFFFFF : w_dt[31:0]);
                    o_oflags <= {w_dt_long, param_valid, w_head_bad, w_jump, w_first, w_speed_ok};
                    o_cum_l <= cum_l; o_cum_r <= cum_r;
                    o_update <= o_update + 32'd1;
                    odom_evt <= 1'b1;
                    sstate <= S_IDLE;
                end
            endcase
        end
    end

    // jump counter lives with the sequencer's decision
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) c_jump <= 0;
        else if (ctrl_clear_cnt) c_jump <= 0;
        else if (sstate == S_F1 && w_jump) c_jump <= sat_inc(c_jump);
    end

    // ================= IRQ + AXI read/write =================
    reg [2:0]  irq_status, irq_enable;
    reg [31:0] tb_hi_latch;
    wire [2:0] irq_clear = (do_write && waddr == 14'h0026) ? s_axi_wdata[2:0] : 3'd0;
    assign irq = |(irq_status & irq_enable);
    wire busy = (sstate != S_IDLE) || job_pending || param_pending;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            irq_status <= 0; irq_enable <= 0; tb_hi_latch <= 0;
            s_axi_bvalid <= 1'b0; s_axi_rvalid <= 1'b0; s_axi_rdata <= 32'd0;
        end else begin
            irq_status <= (irq_status & ~irq_clear) |
                          {rx_err_evt, link_ok_d && !link_ok, odom_evt};
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
            if (do_write) begin
                s_axi_bvalid <= 1'b1;
                if (waddr == 14'h0027) irq_enable <= s_axi_wdata[2:0];
            end
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
            if (do_read) begin
                s_axi_rvalid <= 1'b1;
                case (raddr)
                    14'h0000: s_axi_rdata <= MAGIC;
                    14'h0002: s_axi_rdata <= p_diam;
                    14'h0003: s_axi_rdata <= p_cpr;
                    14'h0004: s_axi_rdata <= p_track;
                    14'h0005: s_axi_rdata <= p_max_delta;
                    14'h0006: s_axi_rdata <= p_max_dt;
                    14'h0007: s_axi_rdata <= k_q16;
                    14'h0008: s_axi_rdata <= {27'd0, link_seen, busy, param_err, param_valid, link_ok};
                    14'h0009: s_axi_rdata <= o_update;
                    14'h000A: s_axi_rdata <= o_ts[31:0];
                    14'h000B: s_axi_rdata <= o_ts[63:32];
                    14'h000C: s_axi_rdata <= {o_ms, o_flags, o_seq};
                    14'h000D: s_axi_rdata <= o_left;
                    14'h000E: s_axi_rdata <= o_right;
                    14'h000F: s_axi_rdata <= {26'd0, o_oflags};
                    14'h0010: s_axi_rdata <= o_v_l;
                    14'h0011: s_axi_rdata <= o_v_r;
                    14'h0012: s_axi_rdata <= o_v_c;
                    14'h0013: s_axi_rdata <= o_dist_l[31:0];
                    14'h0014: s_axi_rdata <= o_dist_l[63:32];
                    14'h0015: s_axi_rdata <= o_dist_r[31:0];
                    14'h0016: s_axi_rdata <= o_dist_r[63:32];
                    14'h0017: s_axi_rdata <= o_dist_c[31:0];
                    14'h0018: s_axi_rdata <= o_dist_c[63:32];
                    14'h0019: s_axi_rdata <= o_head;
                    14'h001A: s_axi_rdata <= o_dt;
                    14'h001B: s_axi_rdata <= c_good;
                    14'h001C: s_axi_rdata <= c_bad;
                    14'h001D: s_axi_rdata <= c_gapev;
                    14'h001E: s_axi_rdata <= c_lost;
                    14'h001F: s_axi_rdata <= c_dup;
                    14'h0020: s_axi_rdata <= c_partial;
                    14'h0021: s_axi_rdata <= c_framing;
                    14'h0022: s_axi_rdata <= c_skip;
                    14'h0023: s_axi_rdata <= c_linkto;
                    14'h0024: s_axi_rdata <= c_glitch;
                    14'h0025: s_axi_rdata <= c_jump;
                    14'h0026: s_axi_rdata <= {29'd0, irq_status};
                    14'h0027: s_axi_rdata <= {29'd0, irq_enable};
                    14'h0028: s_axi_rdata <= CLK_HZ;
                    14'h0029: s_axi_rdata <= DIV;
                    14'h002A: s_axi_rdata <= LINK_TO;
                    14'h002B: s_axi_rdata <= GAP;
                    14'h002C: begin
                        s_axi_rdata <= timebase_i[31:0];
                        tb_hi_latch <= timebase_i[63:32];
                    end
                    14'h002D: s_axi_rdata <= tb_hi_latch;
                    14'h002E: s_axi_rdata <= o_cum_l[31:0];
                    14'h002F: s_axi_rdata <= {{16{o_cum_l[47]}}, o_cum_l[47:32]};
                    14'h0030: s_axi_rdata <= o_cum_r[31:0];
                    14'h0031: s_axi_rdata <= {{16{o_cum_r[47]}}, o_cum_r[47:32]};
                    default:  s_axi_rdata <= 32'd0;
                endcase
            end
        end
    end
endmodule
