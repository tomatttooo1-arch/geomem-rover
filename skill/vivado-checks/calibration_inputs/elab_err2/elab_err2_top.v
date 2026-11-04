// 校准用 elaboration 对照 2：端口位宽不匹配（期望 WARNING）+ 不存在的命名端口（期望 ERROR）
module child2 (input wire [3:0] din, output wire [3:0] dout);
    assign dout = din;
endmodule
module elab_err2_top (input wire [7:0] x, output wire [3:0] z, output wire [3:0] w);
    child2 u_w (.din(x), .dout(w));            // 8 位接 4 位端口：位宽不匹配
    child2 u_t (.din(x[3:0]), .dout_typo(z));  // 不存在的端口名
endmodule
