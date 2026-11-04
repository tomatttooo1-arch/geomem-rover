# PREP1 识别前处理核 HLS 工程。由 build.sh 调用：环境变量 PP_SRC=源码快照目录、PP_BUILD=构建根、PP_NAME=工程名、PP_VEC=向量表。
# KV260（xck26），clk_pl_0 100 MHz：HLS 与 OOC 都按 10.0 ns。
cd $env(PP_BUILD)
open_project -reset $env(PP_NAME)
set_top prep_top
set cf "-std=c++14 -I$env(PP_SRC)"
add_files $env(PP_SRC)/prep_k.cpp -cflags $cf
add_files -tb $env(PP_SRC)/tb_prep.cpp -cflags "$cf -ffp-contract=off"
open_solution -reset sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10.0 -name default
config_interface -m_axi_conservative_mode=false
config_export -format ip_catalog -rtl verilog -vivado_clock 10.0
