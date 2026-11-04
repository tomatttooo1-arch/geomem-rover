// Unified legacy-compatible ROI control shell, architecture baseline only.
// clk=100 MHz. One 1024x32 byte-write TDP BRAM: host A, kernel B.
// Reset does not clear arrays/results. The 22 used scalar RAM words are
// scrubbed after rst_n rises; AXI ready/core_start stay low for 22 clocks.
// Busy parameter writes remain accepted, matching the legacy caller contract:
// software MUST keep configuration stable between start and done.
// Kernel must validate the whole batch before any legal result write.
module roi_csr_shell (
    input  logic clk, input logic rst_n,
    input  logic [11:0] s_axi_awaddr, input logic s_axi_awvalid,
    output logic s_axi_awready,
    input  logic [31:0] s_axi_wdata, input logic [3:0] s_axi_wstrb,
    input  logic s_axi_wvalid, output logic s_axi_wready,
    output logic [1:0] s_axi_bresp, output logic s_axi_bvalid, input logic s_axi_bready,
    input  logic [11:0] s_axi_araddr, input logic s_axi_arvalid, output logic s_axi_arready,
    output logic [31:0] s_axi_rdata, output logic [1:0] s_axi_rresp,
    output logic s_axi_rvalid, input logic s_axi_rready,
    input logic kernel_en, input logic [3:0] kernel_we,
    input logic [9:0] kernel_addr, input logic [31:0] kernel_wdata,
    output logic [31:0] kernel_rdata,
    output logic core_start, input logic core_done, input logic core_idle, input logic core_ready,
    input logic [31:0] evidence_status, input logic [31:0] obs_status,
    output logic interrupt
);
    localparam logic [9:0] CTRL=10'h000, GIE=10'h001, IER=10'h002, ISR=10'h003;
    localparam logic [9:0] EV_RETURN=10'h004, OBS_RETURN=10'h35b;
    // Constant address ROM; first entry is packed in its low ten bits.
    localparam logic [219:0] RESET_ADDRS={
        10'h359,10'h357,10'h356,10'h354,10'h353,10'h351,10'h350,
        10'h34e,10'h34c,10'h34b,10'h349,10'h348,10'h346,10'h344,
        10'h342,10'h340,10'h012,10'h00d,10'h00b,10'h009,10'h007,10'h006};
    (* ram_style="block" *) logic [31:0] csr [0:1023];
    logic [4:0] reset_index;
    logic initialized;
    logic aw_held,w_held;
    logic [9:0] held_awaddr;
    logic [31:0] held_wdata;
    logic [3:0] held_wstrb;
    logic pending_read, pending_read_ram;
    logic [31:0] pending_read_special;
    logic ap_start, ap_done, ap_ready, auto_restart, gie;
    logic [1:0] ier,isr;
    logic [31:0] a_ram_q,b_ram_q,b_special_q;
    logic b_ram_selected;
    logic a_en; logic [3:0] a_we; logic [9:0] a_addr; logic [31:0] a_wdata;
    logic b_en; logic [3:0] b_we;
    logic write_issue,read_issue,write_scalar_or_array;

    function automatic logic scalar_address(input logic [9:0] a);
        scalar_address = (a==10'h006 || a==10'h007 || a==10'h009 ||
            a==10'h00b || a==10'h00d || a==10'h012 ||
            a==10'h340 || a==10'h342 || a==10'h344 || a==10'h346 ||
            a==10'h348 || a==10'h349 || a==10'h34b || a==10'h34c || a==10'h34e ||
            a==10'h350 || a==10'h351 || a==10'h353 || a==10'h354 ||
            a==10'h356 || a==10'h357 || a==10'h359);
    endfunction
    function automatic logic parameter_address(input logic [9:0] a);
        parameter_address = scalar_address(a) ||
            (a>=10'h020 && a<=10'h03f) || // enabled[32]
            (a>=10'h080 && a<=10'h0ff) || // xyxy[128]
            (a>=10'h140 && a<=10'h15f) || // id[32]
            (a>=10'h300 && a<=10'h33f);   // front/back cuts
    endfunction
    function automatic logic result_address(input logic [9:0] a);
        result_address = (a>=10'h200 && a<=10'h2df); // 224 words, 800..b7c
    endfunction
    function automatic logic memory_address(input logic [9:0] a);
        memory_address = parameter_address(a) || result_address(a);
    endfunction
    function automatic logic [31:0] special_data(input logic [9:0] a);
        logic [31:0] d;
        begin
            d=32'b0;
            if (a==CTRL) begin
                d[0]=ap_start; d[1]=ap_done; d[2]=core_idle; d[3]=ap_ready;
                d[7]=auto_restart; d[9]=interrupt;
            end
            if (a==GIE) d[0]=gie;
            if (a==IER) d[1:0]=ier;
            if (a==ISR) d[1:0]=isr;
            if (a==EV_RETURN) d=evidence_status;
            if (a==OBS_RETURN) d=obs_status;
            special_data=d;
        end
    endfunction

    assign core_start = initialized && ap_start;
    assign interrupt = gie && (|isr);
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;
    // AW and W may arrive in either order. One outstanding B and R are
    // retained independently until accepted. Buffered writes take priority
    // over a new AR, but a pending RAM read owns A until captured.
    assign s_axi_awready = initialized && !aw_held && !s_axi_bvalid;
    assign s_axi_wready = initialized && !w_held && !s_axi_bvalid;
    assign write_issue = initialized && aw_held && w_held && !s_axi_bvalid && !pending_read;
    assign s_axi_arready = initialized && !pending_read && !s_axi_rvalid && !write_issue;
    assign read_issue = s_axi_arvalid && s_axi_arready;
    assign write_scalar_or_array = parameter_address(held_awaddr);
    // A is a single physical RAM port. Initialization and normal writes
    // share the same byte-write statement, not separate RAM write processes.
    always_comb begin
        a_en=1'b0; a_we=4'b0; a_addr=10'b0; a_wdata=32'b0;
        if (!initialized) begin
            a_en=rst_n; a_we=4'hf; a_addr=RESET_ADDRS[10*reset_index +: 10];
        end else if (write_issue && write_scalar_or_array) begin
            a_en=1'b1; a_we=held_wstrb; a_addr=held_awaddr; a_wdata=held_wdata;
        end else if (read_issue && memory_address(s_axi_araddr[11:2])) begin
            a_en=1'b1; a_addr=s_axi_araddr[11:2];
        end
    end
    // Host cannot write the result window; kernel cannot write parameters.
    // Simultaneous host read/kernel write of one result word is unspecified,
    // just as legacy RAM collision behavior; caller reads only after done.
    assign b_en = initialized && kernel_en;
    assign b_we = b_en && result_address(kernel_addr) ? kernel_we : 4'b0000;
    // Plain sequential blocks follow the inferred TDP RAM template: both
    // ports intentionally update disjoint byte enables of this one memory.
    always @(posedge clk) begin
        if (a_en) begin
            a_ram_q <= csr[a_addr]; // Sole host RAM read site; synchronous 1 cycle.
            for (integer lane=0;lane<4;lane=lane+1)
                if (a_we[lane]) csr[a_addr][8*lane +: 8] <= a_wdata[8*lane +: 8];
        end
    end
    always @(posedge clk) begin
        if (b_en) begin
            b_ram_q <= csr[kernel_addr]; // Sole kernel RAM read site.
            b_ram_selected <= memory_address(kernel_addr);
            b_special_q <= special_data(kernel_addr);
            for (integer lane=0;lane<4;lane=lane+1)
                if (b_we[lane]) csr[kernel_addr][8*lane +: 8] <= kernel_wdata[8*lane +: 8];
        end
    end
    assign kernel_rdata = !initialized ? 32'b0 : (b_ram_selected ? b_ram_q : b_special_q);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            initialized<=1'b0; reset_index<=5'b0;
            aw_held<=1'b0; w_held<=1'b0; held_awaddr<=10'b0;
            held_wdata<=32'b0; held_wstrb<=4'b0;
            s_axi_bvalid<=1'b0; s_axi_rvalid<=1'b0; s_axi_rdata<=32'b0;
            pending_read<=1'b0; pending_read_ram<=1'b0; pending_read_special<=32'b0;
            ap_start<=1'b0; ap_done<=1'b0; ap_ready<=1'b0; auto_restart<=1'b0;
            gie<=1'b0; ier<=2'b0; isr<=2'b0;
        end else begin
            if (!initialized) begin
                if (reset_index==5'd21) initialized<=1'b1;
                else reset_index<=reset_index+1'b1;
            end
            if (s_axi_awvalid && s_axi_awready) begin
                aw_held<=1'b1; held_awaddr<=s_axi_awaddr[11:2];
            end
            if (s_axi_wvalid && s_axi_wready) begin
                w_held<=1'b1; held_wdata<=s_axi_wdata; held_wstrb<=s_axi_wstrb;
            end
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid<=1'b0;
            if (write_issue) begin
                aw_held<=1'b0; w_held<=1'b0; s_axi_bvalid<=1'b1;
            end
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid<=1'b0;
            if (read_issue) begin
                pending_read<=1'b1;
                pending_read_ram<=memory_address(s_axi_araddr[11:2]);
                pending_read_special<=special_data(s_axi_araddr[11:2]);
            end else if (pending_read) begin
                pending_read<=1'b0; s_axi_rvalid<=1'b1;
                s_axi_rdata<=pending_read_ram ? a_ram_q : pending_read_special;
            end

            // ap_ctrl_hs: start is held until ready, then follows restart.
            if (write_issue && held_awaddr==CTRL && held_wstrb[0] && held_wdata[0]) ap_start<=1'b1;
            else if (core_ready) ap_start<=auto_restart;
            if (write_issue && held_awaddr==CTRL && held_wstrb[0]) auto_restart<=held_wdata[7];
            // Event set wins over clear-on-read in the same cycle.
            if (core_done) ap_done<=1'b1;
            else if (read_issue && s_axi_araddr[11:2]==CTRL) ap_done<=1'b0;
            if (auto_restart) ap_ready<=1'b0;
            else if (core_ready) ap_ready<=1'b1;
            else if (read_issue && s_axi_araddr[11:2]==CTRL) ap_ready<=1'b0;
            if (write_issue && held_wstrb[0]) begin
                if (held_awaddr==GIE) gie<=held_wdata[0];
                if (held_awaddr==IER) ier<=held_wdata[1:0];
            end
            if (ier[0] && core_done) isr[0]<=1'b1;
            else if (write_issue && held_awaddr==ISR && held_wstrb[0]) isr[0]<=isr[0]^held_wdata[0];
            if (ier[1] && core_ready) isr[1]<=1'b1;
            else if (write_issue && held_awaddr==ISR && held_wstrb[0]) isr[1]<=isr[1]^held_wdata[1];
        end
    end
endmodule
