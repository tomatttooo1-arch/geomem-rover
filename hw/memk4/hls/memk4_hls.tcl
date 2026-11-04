# MEMK4: Vitis HLS 2022.2 project for the microcoded memory core (KV260, 100 MHz).
# env: MEMK_SRC (hw/memk4 dir), MEMK_PRJ (project dir), MEMK_VEC (record file for csim/cosim), MEMK_CFLAGS (-DMK4_IF=0|1)
set src $::env(MEMK_SRC)
set cfl "-I$src $::env(MEMK_CFLAGS)"
cd [file dirname $::env(MEMK_PRJ)]
open_project [file tail $::env(MEMK_PRJ)]
set_top memk4_top
add_files $src/memk4_core.cpp -cflags $cfl
add_files -tb $src/tb_memk4.cpp -cflags $cfl
open_solution sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10 -name default
config_compile -pipeline_loops 0
config_interface -m_axi_addr64=false
config_export -format ip_catalog -rtl verilog -vivado_clock 10
csim_design -argv $::env(MEMK_VEC)
cosim_design -rtl verilog -argv $::env(MEMK_VEC)
