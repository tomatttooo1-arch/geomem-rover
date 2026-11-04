| 项 | 期望（文本 .rpt） | 实际（hls_report.sh） | 结果 |
|---|---|---|---|
| top 目标时钟 ns | 10.00 | 10.00 | 一致 |
| top 估计时钟 ns | 7.016 | 7.016 | 一致 |
| top 延迟 min (cycles) | ? | undef | 一致 |
| top 延迟 max (cycles) | ? | undef | 一致 |
| top II min | ? | undef | 一致 |
| top II max | ? | undef | 一致 |
| top BRAM_18K | 0 | 0 | 一致 |
| top DSP | 8 | 8 | 一致 |
| top FF | 816 | 816 | 一致 |
| top LUT | 856 | 856 | 一致 |
| top URAM | 0 | 0 | 一致 |
| 模块 kernel 延迟(cycles) | - | undef | 一致 |
| 模块 kernel Interval | - | undef | 一致 |
| 模块 kernel BRAM_18K | - | 0 | 一致 |
| 模块 kernel DSP | 8 (~0%) | 8 | 一致 |
| 模块 kernel FF | 816 (~0%) | 816 | 一致 |
| 模块 kernel LUT | 856 (~0%) | 856 | 一致 |
| 模块 kernel URAM | - | 0 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2 延迟(cycles) | - | undef | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2 Interval | - | undef | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2 BRAM_18K | - | 0 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2 DSP | - | 0 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2 FF | 66 (~0%) | 66 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2 LUT | 127 (~0%) | 127 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2 URAM | - | 0 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_19_5 延迟(cycles) | - | undef | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_19_5 Interval | - | undef | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_19_5 BRAM_18K | - | 0 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_19_5 DSP | 2 (~0%) | 2 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_19_5 FF | 296 (~0%) | 296 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_19_5 LUT | 356 (~0%) | 356 | 一致 |
| 模块 kernel_Pipeline_VITIS_LOOP_19_5 URAM | - | 0 | 一致 |
| 循环 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2/VITIS_LOOP_10_1_VITIS_LOOP_11_2 II | 1 | 1 | 一致 |
| 循环 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2/VITIS_LOOP_10_1_VITIS_LOOP_11_2 迭代延迟 | 1 | 1 | 一致 |
| 循环 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2/VITIS_LOOP_10_1_VITIS_LOOP_11_2 延迟max | - | undef | 一致 |
| 循环 kernel_Pipeline_VITIS_LOOP_10_1_VITIS_LOOP_11_2/VITIS_LOOP_10_1_VITIS_LOOP_11_2 trip max | - | undef | 一致 |
| 循环 kernel/VITIS_LOOP_17_3_VITIS_LOOP_18_4 II | - | None | 一致 |
| 循环 kernel/VITIS_LOOP_17_3_VITIS_LOOP_18_4 迭代延迟 | - | undef | 一致 |
| 循环 kernel/VITIS_LOOP_17_3_VITIS_LOOP_18_4 延迟max | - | undef | 一致 |
| 循环 kernel/VITIS_LOOP_17_3_VITIS_LOOP_18_4 trip max | - | undef | 一致 |
| 循环 kernel_Pipeline_VITIS_LOOP_19_5/VITIS_LOOP_19_5 II | 3 | 3 | 一致 |
| 循环 kernel_Pipeline_VITIS_LOOP_19_5/VITIS_LOOP_19_5 迭代延迟 | 4 | 4 | 一致 |
| 循环 kernel_Pipeline_VITIS_LOOP_19_5/VITIS_LOOP_19_5 延迟max | - | undef | 一致 |
| 循环 kernel_Pipeline_VITIS_LOOP_19_5/VITIS_LOOP_19_5 trip max | - | undef | 一致 |

共 44 项，不一致 0 项 → PASS
