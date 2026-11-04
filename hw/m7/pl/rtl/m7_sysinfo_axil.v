// M7 PL system-information block (2026-09-26), AXI4-Lite slave, 32-bit.
//
// Purpose: the first read after `xmutil loadapp` must be a known answer that
// proves (a) the bitstream that is running is the one we packaged (BUILD_ID),
// (b) the PS->PL control path works (MAGIC, SCRATCH write/readback), and
// (c) the PL clock really runs at the rate the overlay asked for (CYCLE
// counter compared against the PS clock over a timed interval).
//
// 0x00 MAGIC      RO 0x4D37504C ("M7PL")
// 0x04 VERSION    RO register-layout version (major<<16 | minor)
// 0x08 BUILD_ID   RO parameter (git short hash of the build sources)
// 0x0C BUILD_DATE RO parameter (0xYYYYMMDD)
// 0x10 CLK_HZ     RO parameter: nominal PL clock the build was timed for
// 0x14 CYCLE_LO   RO free-running cycle counter [31:0]; reading it latches [63:32]
// 0x18 CYCLE_HI   RO latched counter [63:32]
// 0x1C SCRATCH    RW
// 0x20 SLOT_MAP   RO parameter: bit i set = address slot i populated
// other offsets read 0; writes to RO/unmapped offsets are ignored (OKAY).
module m7_sysinfo_axil #(
    parameter [31:0] BUILD_ID   = 32'h0,
    parameter [31:0] BUILD_DATE = 32'h0,
    parameter [31:0] CLK_HZ     = 32'd100000000,
    parameter [31:0] SLOT_MAP   = 32'h0
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
    input  wire        s_axi_rready
);
    localparam [31:0] MAGIC   = 32'h4D37504C;
    localparam [31:0] VERSION = 32'h00010000;

    reg [63:0] cycles;
    reg [31:0] cycles_hi_latched;
    reg [31:0] scratch;

    // Write channel: take AW and W together, one outstanding response.
    wire do_write = s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid;
    assign s_axi_awready = do_write;
    assign s_axi_wready  = do_write;
    assign s_axi_bresp   = 2'b00;
    // Read channel: one outstanding response.
    wire do_read = s_axi_arvalid && !s_axi_rvalid;
    assign s_axi_arready = do_read;
    assign s_axi_rresp   = 2'b00;

    integer i;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            cycles <= 64'd0;
            cycles_hi_latched <= 32'd0;
            scratch <= 32'd0;
            s_axi_bvalid <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata <= 32'd0;
        end else begin
            cycles <= cycles + 64'd1;
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
            if (do_write) begin
                s_axi_bvalid <= 1'b1;
                if (s_axi_awaddr[15:2] == 14'h0007)
                    for (i = 0; i < 4; i = i + 1)
                        if (s_axi_wstrb[i]) scratch[8*i +: 8] <= s_axi_wdata[8*i +: 8];
            end
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
            if (do_read) begin
                s_axi_rvalid <= 1'b1;
                case (s_axi_araddr[15:2])
                    14'h0000: s_axi_rdata <= MAGIC;
                    14'h0001: s_axi_rdata <= VERSION;
                    14'h0002: s_axi_rdata <= BUILD_ID;
                    14'h0003: s_axi_rdata <= BUILD_DATE;
                    14'h0004: s_axi_rdata <= CLK_HZ;
                    14'h0005: begin
                        s_axi_rdata <= cycles[31:0];
                        cycles_hi_latched <= cycles[63:32];
                    end
                    14'h0006: s_axi_rdata <= cycles_hi_latched;
                    14'h0007: s_axi_rdata <= scratch;
                    14'h0008: s_axi_rdata <= SLOT_MAP;
                    default: s_axi_rdata <= 32'd0;
                endcase
            end
        end
    end
endmodule
