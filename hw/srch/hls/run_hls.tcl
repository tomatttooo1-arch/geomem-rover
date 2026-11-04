# SRCHK3 概率搜索核 HLS 工程。由 build.sh 调用：SR_SRC=源码快照目录、SR_BUILD=构建根、SR_NAME=工程名。
# KV260（xck26），clk_pl_0 100 MHz：HLS 与 OOC 都按 10.0 ns。
cd $env(SR_BUILD)
open_project -reset $env(SR_NAME)
set_top srch_top
set cf "-std=c++14 -I$env(SR_SRC)"
add_files $env(SR_SRC)/srch_core.cpp -cflags $cf
add_files -tb $env(SR_SRC)/srch_tb.cpp -cflags "$cf -Wno-sign-compare"
open_solution -reset sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10.0 -name default
config_interface -m_axi_addr64=false -m_axi_buffer_impl lutram
# SRCHK5：额外工具配置（二分 c6 用），SR_CFG 里放 Tcl 行（分号分隔）
if {[info exists env(SR_CFG)]} { eval $env(SR_CFG) }
config_export -format ip_catalog -rtl verilog -vivado_clock 10.0
cosim_design -rtl verilog
