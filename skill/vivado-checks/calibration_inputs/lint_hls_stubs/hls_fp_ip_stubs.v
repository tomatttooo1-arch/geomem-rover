// 校准用：HLS 生成的 floating_point IP 黑盒占位（只为让 lint/elab 能走完 HLS 顶层；端口取自 HLS 包装器例化）
(* black_box *) module fpga_backproject_fadd_32ns_32ns_32_4_full_dsp_1_ip (
    input aclk, input aclken, input s_axis_a_tvalid, input [31:0] s_axis_a_tdata,
    input s_axis_b_tvalid, input [31:0] s_axis_b_tdata,
    output m_axis_result_tvalid, output [31:0] m_axis_result_tdata);
endmodule
(* black_box *) module fpga_backproject_fmul_32ns_32ns_32_3_max_dsp_1_ip (
    input aclk, input aclken, input s_axis_a_tvalid, input [31:0] s_axis_a_tdata,
    input s_axis_b_tvalid, input [31:0] s_axis_b_tdata,
    output m_axis_result_tvalid, output [31:0] m_axis_result_tdata);
endmodule
(* black_box *) module fpga_backproject_fcmp_32ns_32ns_1_2_no_dsp_1_ip (
    input s_axis_a_tvalid, input [31:0] s_axis_a_tdata,
    input s_axis_b_tvalid, input [31:0] s_axis_b_tdata,
    input s_axis_operation_tvalid, input [7:0] s_axis_operation_tdata,
    output m_axis_result_tvalid, output [7:0] m_axis_result_tdata);
endmodule
