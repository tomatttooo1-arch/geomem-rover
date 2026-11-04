| 项 | 期望（文本 .rpt） | 实际（hls_report.sh） | 结果 |
|---|---|---|---|
| top 目标时钟 ns | 10.00 | 10.00 | 一致 |
| top 估计时钟 ns | 1.882 | 1.882 | 一致 |
| top 延迟 min (cycles) | 69 | 69 | 一致 |
| top 延迟 max (cycles) | 69 | 69 | 一致 |
| top II min | 70 | 70 | 一致 |
| top II max | 70 | 70 | 一致 |
| top BRAM_18K | 0 | 0 | 一致 |
| top DSP | 1 | 1 | 一致 |
| top FF | 167 | 167 | 一致 |
| top LUT | 129 | 129 | 一致 |
| top URAM | 0 | 0 | 一致 |
| 模块 toy 延迟(cycles) | 69 | 69 | 一致 |
| 模块 toy Interval | 70 | 70 | 一致 |
| 模块 toy BRAM_18K | - | 0 | 一致 |
| 模块 toy DSP | 1 (~0%) | 1 | 一致 |
| 模块 toy FF | 167 (~0%) | 167 | 一致 |
| 模块 toy LUT | 129 (~0%) | 129 | 一致 |
| 模块 toy URAM | - | 0 | 一致 |
| 循环 toy/VITIS_LOOP_4_1 II | 1 | 1 | 一致 |
| 循环 toy/VITIS_LOOP_4_1 迭代延迟 | 5 | 5 | 一致 |
| 循环 toy/VITIS_LOOP_4_1 延迟max | 67 | 67 | 一致 |
| 循环 toy/VITIS_LOOP_4_1 trip max | 64 | 64 | 一致 |
| cosim 状态 | Pass | Pass | 一致 |
| cosim(rpt→summary) latency_min | 67 | 67 | 一致 |
| cosim(rpt→summary) latency_avg | 67 | 67 | 一致 |
| cosim(rpt→summary) latency_max | 67 | 67 | 一致 |
| cosim(rpt→summary) interval_min | NA | NA | 一致 |
| cosim(rpt→summary) interval_avg | NA | NA | 一致 |
| cosim(rpt→summary) interval_max | NA | NA | 一致 |
| cosim(rpt→summary) total_execution_cycles | 67 | 67 | 一致 |
| cosim(rpt→upstream process.zip) min_latency | 67 | 67 | 一致 |
| cosim(rpt→upstream process.zip) avg_latency | 67 | 67 | 一致 |
| cosim(rpt→upstream process.zip) max_latency | 67 | 67 | 一致 |
| cosim(rpt→upstream) total_execution_time | 67 | 67 | 一致 |

共 34 项，不一致 0 项 → PASS
