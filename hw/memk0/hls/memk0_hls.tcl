# MEMK3: Vitis HLS 2022.2 project for the MEMK0 memory core (KV260, 100 MHz).
# env: MEMK_SRC (hw/memk0 dir), MEMK_PRJ (project dir), MEMK_VEC (record file for csim/cosim), MEMK_CFLAGS
set src $::env(MEMK_SRC)
set cfl "-I$src $::env(MEMK_CFLAGS)"
cd [file dirname $::env(MEMK_PRJ)]
open_project [file tail $::env(MEMK_PRJ)]
set_top memk0_top
add_files $src/memk0_core.cpp -cflags $cfl
add_files -tb $src/tb_memk0.cpp -cflags $cfl
open_solution sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10 -name default
config_compile -pipeline_loops 0
config_export -format ip_catalog -rtl verilog -vivado_clock 10
csim_design -argv $::env(MEMK_VEC)
cosim_design -rtl verilog -argv $::env(MEMK_VEC)
