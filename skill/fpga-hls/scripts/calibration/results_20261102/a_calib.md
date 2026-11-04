| 项 | 期望（文本 .rpt） | 实际（hls_report.sh） | 结果 |
|---|---|---|---|
| top 目标时钟 ns | 10.00 | 10.00 | 一致 |
| top 估计时钟 ns | 7.016 | 7.016 | 一致 |
| top 延迟 min (cycles) | ? | undef | 一致 |
| top 延迟 max (cycles) | ? | undef | 一致 |
| top II min | ? | undef | 一致 |
| top II max | ? | undef | 一致 |
| top BRAM_18K | 0 | 0 | 一致 |
| top DSP | 43 | 43 | 一致 |
| top FF | 6196 | 6196 | 一致 |
| top LUT | 4536 | 4536 | 一致 |
| top URAM | 0 | 0 | 一致 |
| 模块 fpga_backproject 延迟(cycles) | - | undef | 一致 |
| 模块 fpga_backproject Interval | - | undef | 一致 |
| 模块 fpga_backproject BRAM_18K | - | 0 | 一致 |
| 模块 fpga_backproject DSP | 43 (3%) | 43 | 一致 |
| 模块 fpga_backproject FF | 6196 (2%) | 6196 | 一致 |
| 模块 fpga_backproject LUT | 4536 (3%) | 4536 | 一致 |
| 模块 fpga_backproject URAM | - | 0 | 一致 |
| 模块 fpga_backproject_Pipeline_VITIS_LOOP_20_1 延迟(cycles) | - | undef | 一致 |
| 模块 fpga_backproject_Pipeline_VITIS_LOOP_20_1 Interval | - | undef | 一致 |
| 模块 fpga_backproject_Pipeline_VITIS_LOOP_20_1 BRAM_18K | - | 0 | 一致 |
| 模块 fpga_backproject_Pipeline_VITIS_LOOP_20_1 DSP | 43 (3%) | 43 | 一致 |
| 模块 fpga_backproject_Pipeline_VITIS_LOOP_20_1 FF | 4957 (2%) | 4957 | 一致 |
| 模块 fpga_backproject_Pipeline_VITIS_LOOP_20_1 LUT | 4334 (3%) | 4334 | 一致 |
| 模块 fpga_backproject_Pipeline_VITIS_LOOP_20_1 URAM | - | 0 | 一致 |
| 循环 fpga_backproject_Pipeline_VITIS_LOOP_20_1/VITIS_LOOP_20_1 II | 2 | 2 | 一致 |
| 循环 fpga_backproject_Pipeline_VITIS_LOOP_20_1/VITIS_LOOP_20_1 迭代延迟 | 36 | 36 | 一致 |
| 循环 fpga_backproject_Pipeline_VITIS_LOOP_20_1/VITIS_LOOP_20_1 延迟max | - | undef | 一致 |
| 循环 fpga_backproject_Pipeline_VITIS_LOOP_20_1/VITIS_LOOP_20_1 trip max | - | undef | 一致 |
| cosim 状态 | Pass | Pass | 一致 |
| cosim(rpt→summary) latency_min | 42 | 42 | 一致 |
| cosim(rpt→summary) latency_avg | 43 | 43 | 一致 |
| cosim(rpt→summary) latency_max | 45 | 45 | 一致 |
| cosim(rpt→summary) interval_min | 46 | 46 | 一致 |
| cosim(rpt→summary) interval_avg | 46 | 46 | 一致 |
| cosim(rpt→summary) interval_max | 46 | 46 | 一致 |
| cosim(rpt→summary) total_execution_cycles | 88 | 88 | 一致 |
| cosim(rpt→upstream process.zip) min_latency | 42 | 42 | 一致 |
| cosim(rpt→upstream process.zip) avg_latency | 43 | 43 | 一致 |
| cosim(rpt→upstream process.zip) max_latency | 45 | 45 | 一致 |
| cosim(rpt→upstream) total_execution_time | 88 | 88 | 一致 |

共 41 项，不一致 0 项 → PASS
