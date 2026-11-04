# 检测后处理核 HLS 工程（DETPOST1）。由 build.sh 调用：环境变量 DP_SRC=本目录、DP_BUILD=构建目录、DP_NAME=工程名、
# DP_KSRC=额外综合源（空格分隔，可空）、DP_VEC=testbench 向量文件。KV260，6.0 ns（166.67 MHz）。
cd $env(DP_BUILD)
open_project -reset $env(DP_NAME)
set_top detpost_top
set cf "-std=c++14 -DDETPOST_CAND_MAX=256 -I$env(DP_SRC)"
add_files $env(DP_SRC)/detpost_k.cpp -cflags $cf
foreach f $env(DP_KSRC) { add_files $f -cflags $cf }
add_files -tb $env(DP_SRC)/tb_detpost.cpp -cflags $cf
add_files -tb $env(DP_SRC)/../detpost_host.cpp -cflags $cf
open_solution -reset sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 6.0 -name default
config_export -format ip_catalog -rtl verilog -vivado_clock 6.0
csim_design -argv $env(DP_VEC)
cosim_design -rtl verilog -argv $env(DP_VEC)
