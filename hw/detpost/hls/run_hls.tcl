# 检测后处理参考模型 csynth（只拿资源/II 量级；未做 HLS 优化）。KV260，6.0 ns（166.67 MHz，几何核快钟档）。
set here [file normalize [file dirname [info script]]]
cd $here
open_project -reset detpost_prj
set_top detpost_top
add_files $here/detpost_top.cpp
add_files $here/../detpost_ref.cpp
open_solution -reset sol1 -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period 6.0 -name default
csynth_design
exit
