// 校准用阳性对照：故意埋入 2022.2 linter 支持的若干规则违例（勿用于实际设计）
module lint_pos_top (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       en,
    input  wire       unused_in,     // ASSIGN-4: 未使用的标量输入
    input  wire [1:0] sel,
    input  wire [7:0] a,
    input  wire [7:0] b,
    output reg  [7:0] latch_q,       // INFER-1: 不完整赋值推断锁存器
    output reg  [7:0] case_q,        // INFER-2: 不完整 case
    output reg  [7:0] multi_q,       // ASSIGN-7: 两个 always 块驱动同一信号
    output reg  [7:0] sum_q,         // ASSIGN-1: 8+8 位加法截断到 8 位
    output reg        both_q         // CLOCK-1: 同一模块用双沿
);
    always @(*) begin
        if (en) latch_q = a;
    end
    always @(*) begin
        case (sel)
            2'b00: case_q = a;
            2'b01: case_q = b;
        endcase
    end
    always @(posedge clk) multi_q <= a;
    always @(posedge clk) if (en) multi_q <= b;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sum_q <= 8'd0;
        else        sum_q <= a + b + 8'd1;
    end
    always @(negedge clk) both_q <= en;
endmodule
