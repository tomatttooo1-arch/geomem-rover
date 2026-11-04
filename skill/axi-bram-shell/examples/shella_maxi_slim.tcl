set src $::env(MEMK_SRC)
cd [file dirname $::env(MEMK_PRJ)]
open_project [file tail $::env(MEMK_PRJ)]
set_top memk4_shella
add_files $src/shella_maxi_slim.cpp
add_files -tb $src/tb_shell_ab.cpp -cflags ""
open_solution sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10 -name default
config_compile -pipeline_loops 0
config_interface -m_axi_addr64=false -m_axi_conservative_mode=false
config_export -format ip_catalog -rtl verilog -vivado_clock 10
