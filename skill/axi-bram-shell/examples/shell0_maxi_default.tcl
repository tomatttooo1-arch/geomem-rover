set src $::env(MEMK_SRC)
cd [file dirname $::env(MEMK_PRJ)]
open_project [file tail $::env(MEMK_PRJ)]
set_top memk4_shell
add_files $src/shell0_maxi_default.cpp
add_files -tb $src/tb_shell.cpp
open_solution sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10 -name default
config_compile -pipeline_loops 0
config_export -format ip_catalog -rtl verilog -vivado_clock 10
