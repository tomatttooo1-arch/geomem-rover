set source_dir [file dirname [file normalize [info script]]]
set project [file join $source_dir .memory/build/hls]
file mkdir [file dirname $project]
cd [file dirname $project]
open_project [file tail $project]
set_top fpga_m7_roi_obs_phase
add_files [file join $source_dir roi_obs_top.cpp] -cflags "-std=c++14"
add_files [file join $source_dir obs_compute.cpp] -cflags "-std=c++14"
add_files [file join $source_dir obs_ccl.cpp] -cflags "-std=c++14"
add_files -tb [file join $source_dir tb_roi_obs.cpp] -cflags "-std=c++14"
add_files -tb [file join $source_dir roi_obs_ps.cpp] -cflags "-std=c++14 -ffp-contract=off"
open_solution solution -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 10 -name default
config_compile -pipeline_loops 0 -pragma_strict_mode=true
config_interface -m_axi_addr64=true

csim_design -argv "${FPGA_RUN}/OBSREF0/vectors/obs_vectors.bin [file join $source_dir .memory quality-cdf-fix representative.bin]"
cosim_design -argv "${FPGA_RUN}/OBSREF0/vectors/obs_vectors.bin [file join $source_dir .memory quality-cdf-fix representative.bin]" -rtl verilog
