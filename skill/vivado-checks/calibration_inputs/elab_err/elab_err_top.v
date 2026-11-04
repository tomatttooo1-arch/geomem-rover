// 校准用 elaboration 错误：例化一个不存在的模块 + 端口名写错
module child (input wire [3:0] din, output wire [3:0] dout);
    assign dout = din;
endmodule
module elab_err_top (input wire [7:0] x, output wire [7:0] y, output wire [3:0] z);
    no_such_module u_missing (.a(x), .b(y));          // 未定义模块
    child u_child (.din(x[3:0]), .dout_typo(z));       // 不存在的端口
endmodule
