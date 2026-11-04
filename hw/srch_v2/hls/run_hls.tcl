# SRCHK6 微程序搜索核 HLS 工程。由 build.sh 调用：SR_SRC=源码快照目录、SR_BUILD=构建根、SR_NAME=工程名。KV260，10 ns。
cd $env(SR_BUILD)
open_project -reset $env(SR_NAME)
set_top srch_top
set cf "-std=c++14 -I$env(SR_SRC)"
add_files $env(SR_SRC)/srch6_core.cpp -cflags $cf
add_files -tb $env(SR_SRC)/srch_tb.cpp -cflags "$cf -Wno-sign-compare"
open_solution -reset sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10.0 -name default
config_compile -pipeline_loops 0
config_interface -m_axi_addr64=false -m_axi_buffer_impl lutram -m_axi_conservative_mode=false
if {[info exists env(SR_CFG)]} { eval $env(SR_CFG) }
config_export -format ip_catalog -rtl verilog -vivado_clock 10.0
cosim_design -rtl verilog
