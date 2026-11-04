// 校准用阴性对照：干净的同步计数器（单时钟沿、完整赋值、单驱动、位宽匹配）
module lint_neg_top (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       en,
    output reg  [7:0] cnt
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)  cnt <= 8'd0;
        else if (en) cnt <= cnt + 8'd1;
    end
endmodule
