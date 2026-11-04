# sim/ —— 各核的测试台与联仿入口

本目录只是索引：测试台和 HLS 工程脚本都放在各核的源码目录旁边（`hw/...`），这里列出路径和运行方法。
工具：Vitis HLS 2022.2，器件 xck26-sfvc784-2LV-c。所有构建目录都放在源码树之外（脚本要求一个尚不存在的新目录）。
脚本里的 `${FPGA_RUN}` 等目录变量含义见顶层 README。

## 在提交比特流（m5）里的核

| 核 | 测试台 | 联仿入口 | 怎么跑 |
|---|---|---|---|
| 几何换算（定点） | `hw/m7/pl/hls/tb_frame_stream_fx.cpp` | `hw/m7/pl/hls/run_export_hls.tcl`（csim → csynth → 导出 IP → 可选 Verilog 联仿） | `FPGA_M7_PL_TOP=… FPGA_M7_PL_PART=xck26-sfvc784-2LV-c FPGA_M7_PL_CLOCK_NS=6 FPGA_M7_PL_BUILD=<新目录> FPGA_M7_PL_IPREPO=<目录> FPGA_M7_PL_COSIM=1 vitis_hls -f hw/m7/pl/hls/run_export_hls.tcl`（变量全集见脚本开头注释） |
| 融合与代价图（滚动栅格） | `hw/m7/pl/hls/tb_rolling_axi.cpp`、`hw/m7/rolling_testbench.cpp` | `hw/m7/run_rolling_hls.tcl`（默认 csim + csynth + Verilog 联仿；`FPGA_M7_ROLL_FLOW=csim` 只跑 C 仿真） | `vitis_hls -f hw/m7/run_rolling_hls.tcl`（构建目录、器件、时钟由环境变量给，见脚本开头注释）；宿主侧对照 `python3 -m pytest tests/test_m7_rolling.py` |
| 目标 ROI 核验 | `hw/m7/pl/hls/tb_object_roi_axi.cpp` | `hw/m7/run_object_roi_hls.tcl`（`FPGA_M7_ROI_TOP=axi` 综合 AXI 外壳，`FPGA_M7_ROI_COSIM=1` 打开联仿） | `vitis_hls -f hw/m7/run_object_roi_hls.tcl` |
| 识别后处理 | `hw/detpost/hls1/tb_detpost.cpp`，向量 `vec_full.txt`、`vec_cosim.txt`，参考实现 `hw/detpost/detpost_ref.cpp` | `hw/detpost/hls1/run_hls.tcl`（由 `build.sh` 调用） | `DP_BUILD_ROOT=<新目录> bash hw/detpost/hls1/build.sh <工程名> hw/detpost/hls1/vec_cosim.txt "" csim csynth cosim`；PS 驱动自检 `make -C hw/detpost/drv check` |
| 路径规划（双尺寸代价场，m5 版） | `hw/planner_dual/m5_68b345d/tb/testbench.cpp` | `hw/planner_dual/m5_68b345d/run_hls.tcl` | `vitis_hls -f hw/planner_dual/m5_68b345d/run_hls.tcl`；PS 驱动自检 `make -C hw/planner/drv check` |
| 图像前处理 | `hw/prep/hls1/tb_prep.cpp`，向量 `vec_csim.txt`、`vec_cosim.txt` | `hw/prep/hls1/run_hls.tcl`（由 `build.sh` 调用） | `PP_BUILD_ROOT=<新目录> bash hw/prep/hls1/build.sh <工程名> hw/prep/hls1/vec_cosim.txt csim csynth cosim`；PS 驱动自检 `make -C hw/prep/drv check` |
| 行驶许可（手写 RTL） | `hw/m7/pl/rtl/tb_m7_permit.sv`、`tb_m7_axil_shells.sv` | `hw/m7/pl/sim/run_permit_uart_sim.sh`（XSim 回归） | `bash hw/m7/pl/sim/run_permit_uart_sim.sh <新目录> [quick]` |
| DPU 识别 | 厂商 IP，无自写测试台 | — | 板上用 `xdputil benchmark` 测 DPU 本体；整链见 `board/results/` |
| 系统信息、时基、跨时钟域、LPD 互连 | — | 整板 validate | 板上寄存器自检程序 `bitstream/m7pl_test`，结果 `bitstream/selfcheck.txt` |

## 不在提交比特流里的扩展核

| 核 | 测试台 | 联仿入口 | 怎么跑 |
|---|---|---|---|
| 记忆核（微程序） | `hw/memk4/tb_memk4.cpp`（参考模型 `hw/memk4/uref.py`） | `hw/memk4/hls/memk4_hls.tcl`（csim + Verilog 联仿，同一份记录文件） | `MEMK_SRC=hw/memk4 MEMK_PRJ=<新目录>/prj MEMK_VEC=<记录文件> MEMK_CFLAGS=-DMK4_IF=1 vitis_hls -f hw/memk4/hls/memk4_hls.tcl`；宿主侧 `python3 -m pytest tests/test_memk0_err_slot.py` |
| 规划核终版（含取路径、全局地图） | `hw/planner_dual/tb/testbench.cpp`（取路径参数 `path_params.bin`） | `hw/planner_dual/run_hls.tcl` | `vitis_hls -f hw/planner_dual/run_hls.tcl`；PS 驱动自检 `make -C hw/planner/drv_gmap3 check` |
| 搜索核 | `hw/srch/srch_tb.cpp` | `hw/srch/hls/run_hls.tcl`（由 `hw/srch/hls/build.sh` 调用） | `SR_BUILD_ROOT=<新目录> SR_VEC=<轨迹向量> bash hw/srch/hls/build.sh <工程名> csim csynth cosim` |
| 观测与核验核 | `hw/m7/pl/hls/roi_obs/tb_roi_obs.cpp`；证据核 `hw/m7/pl/hls/roi_fx/tb_roi_fx.cpp` | `hw/m7/pl/hls/roi_obs/run_hls.tcl`、`hw/m7/pl/hls/roi_fx/run_hls.tcl`（csim + Verilog 联仿） | `vitis_hls -f hw/m7/pl/hls/roi_obs/run_hls.tcl`；C 参考 `hw/obs/obs_ref.{cpp,h}`、`tools/obs_ref/` |

## 宿主侧测试

`tests/` 下是宿主侧单元测试与参考实现，在仓库根目录执行 `python3 -m pytest tests/ -q`（部分用例依赖板上或 Nano 环境，在服务器上可能跳过或失败，按用例说明判断）。

## 正确性是怎么确认的

各 HLS 核都按同一套流程：C 仿真对独立的 C/Python 参考实现逐字节比对 → 综合后 C/RTL 联仿用同一组向量逐字节比对 → 整板构建后在板上用 `bitstream/m7pl_test` 读回寄存器与构建号 → 整机台架运行（`board/results/`）。
各核的联仿结论与比对口径汇总在设计报告"正确性与精度"一节。
