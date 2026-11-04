# 拾忆：基于 FPGA 的语义空间记忆寻物机器人——代码包

## 作品简介

本作品是拾忆：基于 FPGA 的语义空间记忆寻物机器人。小车搭载 KV260 开发板和 RGB-D 相机，边行驶边把看到的物体连同几何位置记录成语义地图，之后可以按一句寻物指令把目标物体找回来。感知、建图、规划与识别的计算在 KV260 上完成：可编程逻辑（PL）中实现了几何换算、融合与代价图、路径规划、图像前处理、DPU 神经网络识别、识别后处理、行驶许可等硬件核，以及语义记忆、概率搜索与到场核验核；三维观测核的 HLS 工程见 `hw/m7/pl/hls/roi_obs/`；Nano 只运行控制台，负责寻物指令输入与结果显示；物体记忆由 PL 语义记忆核维护，概率搜索由 PL 概率搜索核完成，均在 KV260 上运行。

## 目录结构

| 目录 | 内容 |
|---|---|
| `hw/` | 硬件源码：各 PL 核的 HLS/RTL 源码与测试台、块设计与比特流构建脚本（`hw/m7/pl/`）、PS 侧驱动 |
| `host/` | KV260 板上运行的宿主程序（几何生产、DPU 前处理、底盘串口、指标采集等） |
| `perception/` | 感知与记忆的回放、离线记忆和 PL 记忆核的软件后端 |
| `tools/` | 板上服务、识别流水线（`tools/dpu_pipe/`）、搜索、相机与整车启动等工具 |
| `ui/geomem/` | 操作端界面与实车接口 |
| `pc/` | PC 端控制工具 |
| `robot_sim/` | 仿真环境 |
| `tests/` | 宿主侧单元测试与参考实现 |
| `docs/design/` | 观测与核验核的设计说明 |
| `scripts/build/` | 整板比特流一键重建脚本 |
| `bitstream_m7_fullboard/` | 产品默认整板比特流（规划核终版、记忆核、搜索核同时装入）与构建报告，见下文「两份比特流」 |
| `bitstream/` | 另一份可选比特流包 m5（`BIT=m5`，KV260 加载用） |
| `models/dpu/` | DPU 识别模型（编译好的 xmodel） |
| `LICENSE`、`THIRD_PARTY_NOTICES.md` | 本包许可（MIT）与第三方组件许可清单 |
| `board/results/` | 上板实测输出汇总（产品默认配置的整机台架判读 `golive_m7/`、各段时延、端到端时延与功耗、底盘桥 `chassis_bridge/`），见该目录 README |
| `sim/` | 各核测试台与联仿入口的索引（测试台本身在 `hw/` 各核目录旁），见 `sim/README.md` |
| `skill/` | 开发过程中提炼的 AI 技能包，见 `skill/` 内说明 |
| `report/llm_log/` | 大模型协作开发记录，见 `report/llm_log/` 内说明 |
| `data/`、`models/` 下的 README | 数据与模型的说明 |

### 与竞赛推荐目录结构的对照

本包沿用项目自己的目录划分（`hw/`、`host/`、`tools/` 等），与推荐结构的对应关系如下：

| 推荐目录 | 本包中的位置 |
|---|---|
| `README.md` | `README.md`（本文件） |
| `src/` | `hw/`（PL 核 HLS/RTL 源码、块设计、PS 侧驱动）、`host/`、`tools/`、`perception/`、`ui/`、`pc/`、`robot_sim/` |
| `sim/` | `sim/README.md`（索引）；测试台 `hw/**/tb_*`、RTL 仿真 `hw/m7/pl/sim/`；宿主侧测试 `tests/` |
| `build/` | `scripts/build/`（整板一键重建）、各核 HLS 脚本 `hw/**/run_*hls.tcl`、`hw/**/run_hls.tcl`；实现报告 `bitstream/timing_route.rpt`、`bitstream/utilization_route.rpt`；整板版报告 `bitstream_m7_fullboard/` |
| `board/` | `bitstream_m7_fullboard/`（产品默认整板比特流）、`bitstream/`（可选 m5 比特流包与板上加载脚本）、`tools/golive/`（一条命令启动整机）、`board/results/`（实测输出） |
| `data/` | `data/`、`models/`（含 `models/dpu/` 编译好的识别模型） |
| `skill/` | `skill/` |
| `report/` | 设计报告另行提交；大模型协作记录 `report/llm_log/` |

脚本里的 `${FPGA_RUN}`、`${FPGA_SHARED}`、`${FPGA_HOME}`、`${FPGA_SRC}`、`${FPGA_LOCKS}`、`${TRAIN_HOME}` 是本机目录变量，分别指运行记录与运行时依赖目录、共享大文件目录（模型、IP 等）、工作根目录、源码目录、资源锁目录、模型训练目录；使用前按本机实际位置导出（`export FPGA_RUN=...`）。

## 器件与工具链

- 器件：AMD/Xilinx Kria **KV260** Starter Kit（K26 SOM），芯片型号 **XCK26-SFVC784-2LV-C**（Zynq UltraScale+ MPSoC）。板端运行 KV260 官方 Ubuntu 22.04 镜像，用 xmutil / dfx-mgr 加载比特流。
- 工具链版本：**Vivado / Vitis HLS 2022.2**，**Vitis AI 3.0**，DPU IP **DPUCZDX8G v4.1**。
- 为什么用 2022.2：识别所用的 DPU IP（DPUCZDX8G v4.1，随 Vitis AI 3.0 发布）官方只提供与 Vivado 2022.2 配套的版本；DPU 与自研 PL 核在同一个块设计里构建，所以整板与各核统一用 2022.2。

## 复现步骤

1. **取包**：解压 `fpga_submit_code.zip`，进入 `fpga_submit_code/`。
2. **校验 SHA256**：先校验 zip 本身（`sha256sum -c fpga_submit_code.zip.sha256`，与 zip 同目录），再校验两份比特流包：`cd bitstream_m7_fullboard && sha256sum -c SHA256SUMS && cd ..`、`cd bitstream && sha256sum -c SHA256SUMS && cd ..`，应全部 OK。识别模型目录各带 `md5sum.txt`（`cd models/dpu/y8s640 && md5sum -c md5sum.txt`）。
3. **安装比特流**：把 `bitstream_m7_fullboard/` 拷到 KV260 上，在该目录执行 `sudo M7PL_APP=m7_pl_m7s17 ./board_bringup.sh`。脚本把 `m7_pl_m7s17/` 装到 `/lib/firmware/xilinx/`，用 `xmutil unloadapp / loadapp` 加载，读回 PL 系统信息寄存器里的构建号（应为 0x063ba99b，对照 `bitstream_m7_fullboard/selfcheck.txt`），每步打印 PASS/FAIL。之后一条命令启动脚本默认加载这份整板比特流并核对构建号。可选的 m5 比特流同法安装：`bitstream/` 下 `sudo M7PL_APP=m7_pl_m5_166 ./board_bringup.sh`（构建号 0x0f35f07f），启动时加 `BIT=m5`。
4. **一条命令启动**：`bash tools/golive/start_all.sh`，见下节"上板启动"。
5. **判读**：停止后用 `python3 tools/golive/judge.py <取回的日志目录> 200 gmap3` 按判据 C1–C9 判读（C1–C6 整机链路，C7 搜索核，C8 规划核终版，C9 记忆核），输出各段帧率与 `judge.json`；参考结果见 `board/results/golive_m7/`（深度链约 26 帧/秒，识别 640 模型约 7 帧/秒，连续运行 250.6 s 无崩溃无重启，规划核取路径与 PS 参考全部一致，记忆全部来自 PL 记忆核，搜索全部在核上完成）。

## PL 核与源码目录

两份比特流都包含的核：

| 核 | 文档中的名称 | 功能 | 源码路径 |
|---|---|---|---|
| 几何换算（定点） | 八路行走几何核（创新一） | 深度帧到地面坐标与射线标志的定点换算 | `hw/m7/pl/hls/m7_frame_stream.cpp`（顶层，流式）、`hw/m7/frame_fx.{hpp,cpp}`（定点核与 PS 常数）、`hw/m7/pl/hls/run_export_hls.tcl`；测试台 `hw/m7/pl/hls/tb_frame_stream_fx.cpp` |
| 融合与代价图（滚动栅格） | 深度直通势场（创新二） | 多帧观测融合为滚动栅格地图并生成代价图 | `hw/m7/rolling_map.{hpp,cpp}`、`hw/m7/pl/hls/m7_rolling_axi.cpp`；测试台 `hw/m7/pl/hls/tb_rolling_axi.cpp`、`hw/m7/rolling_testbench.cpp` |
| 目标 ROI 核验 | 寻物决策链硬件化（创新三） | 识别框区域的几何证据核验 | `hw/m7/object_evidence.{hpp,cpp}`、`hw/m7/pl/hls/m7_object_roi_axi.cpp`、`hw/m7/run_object_roi_hls.tcl` |
| 识别后处理 | 寻物决策链硬件化（创新三） | DPU 输出解码、阈值与非极大值抑制 | `hw/detpost/hls1/`（detpost_k.cpp、run_hls.tcl）、参考实现 `hw/detpost/detpost_ref.cpp`；PS 驱动 `hw/detpost/drv/`；调用 `tools/dpu_pipe/pl_post.cpp` |
| 路径规划（双尺寸代价场） | 深度直通势场（创新二） | 在代价图上计算规划势场 | `hw/planner_dual/m5_68b345d/`；PS 驱动 `hw/planner/drv/` |
| 图像前处理 | 寻物决策链硬件化（创新三） | 缩放与量化，输出 DPU 输入张量 | `hw/prep/hls1/`（prep_k.cpp、run_hls.tcl）；PS 驱动 `hw/prep/drv/` |
| DPU 识别 | — | 神经网络推理（DPUCZDX8G B1024，300 MHz） | 厂商 IP（Vitis AI 3.0），配置在 `hw/m7/pl/bd/m7_bd.tcl` |
| 行驶许可 | — | 行驶许可寄存器与超时保护 | `hw/m7/pl/rtl/m7_permit_axil.v` |
| 系统信息、时基、跨时钟域 | — | 构建号、时间戳与跨域同步 | `hw/m7/pl/rtl/m7_sysinfo_axil.v`、`m7_timebase.v`、`m7_cdc_single.v` |
| LPD 互连 | — | 低功耗域 AXI 互连 | 块设计 `hw/m7/pl/bd/m7_bd.tcl` |

扩展核（由 `scripts/build/launch_whole_m7.sh` 接入整板构建；产品默认的整板比特流 `bitstream_m7_fullboard/` 装入了其中的记忆核、规划核终版、搜索核）：

| 核 | 文档中的名称 | 功能 | 源码路径 |
|---|---|---|---|
| 记忆核（微程序） | 寻物决策链硬件化（创新三） | 物体语义记忆的更新与查询 | `hw/memk4/`（memk4_core.cpp、hls/memk4_hls.tcl、uisa/uprog/uref.py）；PS 驱动 `perception/fpga_replay/memory_pl.py` |
| 规划核（含取路径、全局地图） | 深度直通势场（创新二） | 全局地图上的代价场与路径提取 | `hw/planner_dual/src/`、`hw/planner_dual/path/`、`hw/planner_dual/tb/`；驱动 `hw/planner/drv_gmap3/`；寄存器头 `hw/m7/pl/sw/merge10_m7/m7pl_planner_regs.h` |
| 搜索核（微程序，第二版） | 寻物决策链硬件化（创新三） | 寻物目标的候选位置搜索 | `hw/srch_v2/`（srch6_core.cpp、srch6_tb.cpp、uprog6.py、hls/run_hls.tcl；整板中 5,035 LUT / 51 DSP / 13 URAM）；优化前的 c8 版 `hw/srch/` 保留作对照；PS 侧 `tools/srch/`；寄存器头 `hw/m7/pl/sw/merge10_m7/m7pl_srch_regs.h` |
| 到场核验核 | 寻物决策链硬件化（创新三） | 到场证据核验 | `hw/m7/pl/hls/roi_fx/` |
| 三维观测核 HLS 工程 | 寻物决策链硬件化（创新三） | 目标三维观测 | `hw/m7/pl/hls/roi_obs/`（含 rtl/、run_hls.tcl、tb_roi_obs.cpp）；C 参考 `hw/obs/obs_ref.{cpp,h}`、`tools/obs_ref/`；设计说明 `docs/design/` |

块设计与构建工具：块设计 `hw/m7/pl/bd/m7_bd.tcl`、`build_bitstream.tcl`；地址与寄存器头 `hw/m7/pl/sw/`；生成与检查工具 `hw/m7/pl/tools/m7pl_gen.py`。

## 重建比特流

工具：Vivado / Vitis HLS 2022.2，Vitis AI 3.0 的 DPUCZDX8G v4.1，器件 XCK26-SFVC784-2LV-C（KV260）。

1. 构建脚本用源码提交号生成构建号（BUILD_ID），因此先把本目录初始化为 git 仓库：
   `git init && git add -A && git commit -m src`
2. 用各核的 HLS 脚本导出 IP（见上表），把几何、融合、ROI、后处理四个导出放进同一个 IP 仓库目录。
3. 设好环境变量后在仓库根目录执行：

```bash
export FPGA_IP_REPO=<HLS 导出所在的 IP 仓库目录>
export FPGA_DPU_IPREPO=<Vitis AI 3.0 的 DPUCZDX8G_VAI_v3.0/dpu_ip>
export FPGA_PREP_IP=<前处理核 export.zip>
export FPGA_PLANNER_IP=<规划核 export.zip>     # m5 配置所用的规划核导出 sha256 前缀 92b35457
bash scripts/build/launch_whole_m5.sh <标签>   # m5 配置（bitstream/）
# 整板配置（bitstream_m7_fullboard/，参数见其 README）：bash scripts/build/launch_whole_m7.sh <标签> [planner_ip] [srch_ip] [memk_ip] [obs_ip]
# 只打印配置不构建：DRY=1 bash scripts/build/launch_whole_m7.sh <标签>
```

构建输出在 `builds/impl-<配置>-<标签>/`（可用 `FPGA_BUILD_OUT` 改）。

## 两份比特流

本包带两份比特流：

| 目录 | 是什么 | 用法 |
|---|---|---|
| `bitstream_m7_fullboard/` | **产品默认**：整板比特流 `m7_pl_m7s17`（构建号 0x063ba99b）。在 m5 配置的各核与 DPU 之上，同时装入规划核终版、记忆核、搜索核第二版（`hw/srch_v2/`）；建立/保持裕量 +0.016 / +0.009 ns，LUT 83,750（71.5%）、寄存器 106,984、BRAM 129.5、URAM 50、DSP 366 | 一条命令启动默认加载它，记忆、搜索、规划终版都在 PL 上运行；板上整机台架连续运行 250.6 s（识别 7.11 帧/秒，深度 26.3 帧/秒，整机功耗 6.27 W），判据 C1–C9 判读，结果见 `board/results/golive_m7/` |
| `bitstream/` | 另一份可选比特流：m5 配置（构建号 0x0f35f07f），几何、融合、ROI、后处理、前处理、规划（m5 版）各核与 DPU | `BIT=m5 bash tools/golive/start_all.sh` |

`bitstream_m7_fullboard/` 附构建报告（资源总表与分层表、路由后时序汇总、地址映射）和从源码重建的步骤，
见 `bitstream_m7_fullboard/README.md`。重建它时先把 `hw/m7/pl/sw/merge10_m7/` 下的寄存器头换回 `hw/m7/pl/sw/`。

## 上板启动

在服务器上一条命令启动整机（比特流加载、相机、几何—融合—规划、识别流水线、记忆、搜索、到 Nano 的上行与 Nano 任务控制台）：

```bash
bash tools/golive/start_all.sh            # 启动（默认整板比特流 bitstream_m7_fullboard/，记忆、搜索、规划终版走 PL）
bash tools/golive/start_all.sh status     # 查看两端状态
bash tools/golive/stop_all.sh             # 停止并取回日志
python3 tools/golive/judge.py <取回的日志目录> 200 gmap3   # 判读，判据 C1–C9
```

默认配置：`BIT=m7pre`（整板比特流 `m7_pl_m7s17`，构建号 0x063ba99b，用 xmutil 加载并读回构建号核对），识别前处理、后处理与导航走 PL，`MEM=pl`（记忆核）、`SRCH=pl`（搜索核）、`PLN=gmap3`（规划核终版：核内取路径 + 全局地图）。首次在一块 KV260 上使用时，先在板上 `bitstream_m7_fullboard/` 目录执行一次 `sudo M7PL_APP=m7_pl_m7s17 ./board_bringup.sh`，把 app 包装到 `/lib/firmware/xilinx/`。

另一份可选比特流：`BIT=m5 bash tools/golive/start_all.sh` 加载 `bitstream/`（m5 配置，构建号 0x0f35f07f）。

运行前先导出上面的目录变量：`start_all.sh` 从 `${FPGA_RUN}` 下取相机程序包、交叉编译的板上库和功耗采样脚本，从 `${FPGA_SHARED}/models/` 取识别模型（同 `models/dpu/`）。

可选环境变量：`RECOG_MODEL=y8s640|y8s512`（识别模型），`POST_ARGS`（默认 `--post pl`，识别后处理走 PL 核），`MEM`、`SRCH`、`PLN`（记忆、搜索、规划的实现选择），`XBIN`（交叉编译的板上程序与库所在目录，默认 `${FPGA_RUN}/GOLIVE1/xbin`：识别接收程序、导航库、规划核驱动、搜索核驱动、记忆核库与搜索场景），`RUN_S`（运行秒数，0 为直到停止），`FIND`（寻物类别）。KV260 端脚本为 `tools/golive/kv_golive.sh`（记忆服务 `kv_memsvc.py`、搜索服务 `srch_chain.py`），Nano 端控制台为 `tools/golive/nano_console.py`。

## 模型与比特流的位置

- 产品默认整板比特流：`bitstream_m7_fullboard/`。`m7_pl_m7s17/` 下是 `m7_pl_m7s17.bit.bin`、`m7_pl_m7s17.dtbo`、`shell.json`；构建号 0x063ba99b；附构建报告，见该目录 README。文件校验：`cd bitstream_m7_fullboard && sha256sum -c SHA256SUMS`。
- 可选比特流包：`bitstream/`。`m7_pl_m5_166/` 下是 `m7_pl_m5_166.bit.bin`、`m7_pl_m5_166.dtbo`、`shell.json`；`m7_pl.hwh` 为硬件描述，`board_bringup.sh` 为板上加载脚本，`m7pl_test` 为板上寄存器自检程序；构建号 0x0f35f07f。文件校验：`cd bitstream && sha256sum -c SHA256SUMS`。
- DPU 识别模型：`models/dpu/y8s640/q4_y8s_640_b1024.xmodel`（默认，640 输入，阈值文件 `y8s640_threshold-selection.json`），`models/dpu/y8s512_backup/y8s_six_512_b1024.xmodel`（512 输入，`RECOG_MODEL=y8s512` 时使用）；各目录附编译记录与 `md5sum.txt`。

## 说明

### 文件改名对照

为满足"文件名只用小写英文字母、数字、下划线、连字符"的要求，下列文件在本包中改了名，包内对它们的引用已同步修改（`Makefile`、`CMakeLists.txt`、技能包格式规定的 `SKILL.md`、`Dockerfile`、`README.md`、`*LICENSE*`、`SHA256SUMS` 这类惯用名保留原样）：

| 原文件名 | 本包中的文件名 |
|---|---|
| `ui/geomem/先看这里-使用说明.txt` | `ui/geomem/readme_first_zh.txt` |
| `ui/geomem/实车接口协议.md` | `ui/geomem/vehicle_interface_protocol.md` |
| `ui/geomem/启动演示.cmd` | `ui/geomem/start_demo.cmd` |
| `ui/geomem/operator/portable-team/关闭本机控制台.cmd` | `ui/geomem/operator/portable-team/stop_local_console.cmd` |
| `ui/geomem/operator/portable-team/启动服务器控制台.cmd` | `ui/geomem/operator/portable-team/start_server_console.cmd` |
| `hw/detpost/DESIGN_DETPOST0.md` | `hw/detpost/design_detpost0.md` |
| `bitstream/CLOCKS.txt` | `bitstream/clocks.txt` |
| `bitstream/BUILD_SUMMARY.txt` | `bitstream/build_summary.txt` |
| `bitstream/SELFCHECK.txt` | `bitstream/selfcheck.txt` |
| `ui/geomem/EXTERNAL_FILES.md` | `ui/geomem/external_files.md` |
| `ui/geomem/app/vendor/OrbitControls.js` | `ui/geomem/app/vendor/orbitcontrols.js` |
| `ui/geomem/operator/PRODUCT.md` | `ui/geomem/operator/product.md` |
| `ui/geomem/operator/portable-team/Start-DirectConsole.cmd` | `ui/geomem/operator/portable-team/start-directconsole.cmd` |
| `ui/geomem/operator/portable-team/Stop-CarStack.ps1` | `ui/geomem/operator/portable-team/stop-carstack.ps1` |
| `ui/geomem/operator/portable-team/Start-TeamConsole.ps1` | `ui/geomem/operator/portable-team/start-teamconsole.ps1` |
| `ui/geomem/operator/portable-team/Stop-TeamConsole.ps1` | `ui/geomem/operator/portable-team/stop-teamconsole.ps1` |
| `ui/geomem/operator/portable-team/Stop-CarStack.cmd` | `ui/geomem/operator/portable-team/stop-carstack.cmd` |
| `ui/geomem/operator/portable-team/Start-DirectConsole.ps1` | `ui/geomem/operator/portable-team/start-directconsole.ps1` |
| `ui/geomem/operator/robot/Start-KV260-Interactive.cmd` | `ui/geomem/operator/robot/start-kv260-interactive.cmd` |
| `ui/geomem/operator/robot/Start-Competition.ps1` | `ui/geomem/operator/robot/start-competition.ps1` |
| `ui/geomem/operator/robot/Start-LabConsole.ps1` | `ui/geomem/operator/robot/start-labconsole.ps1` |
| `ui/geomem/operator/vendor/OrbitControls.js` | `ui/geomem/operator/vendor/orbitcontrols.js` |
| `ui/geomem/tools/basemap/LAPTOP.md` | `ui/geomem/tools/basemap/laptop.md` |
| `docs/design/OBS_CORE_PRODUCT_DESIGN.md` | `docs/design/obs_core_product_design.md` |
| `docs/evidence/camera/kv_cam_P1_cores.txt` | `docs/evidence/camera/kv_cam_p1_cores.txt` |
| `tools/kv_cam/PROTOCOL.md` | `tools/kv_cam/protocol.md` |
| `tools/kv_nav/sim/PREREG_CTRLPS1.md` | `tools/kv_nav/sim/prereg_ctrlps1.md` |
| `tools/planner_model/V2_README.md` | `tools/planner_model/v2_readme.md` |
| `tools/planner_model/V3_README.md` | `tools/planner_model/v3_readme.md` |
| `tools/planner_model/AUDIT.md` | `tools/planner_model/audit.md` |
| `tools/planner_model/realpose/RECORDING.md` | `tools/planner_model/realpose/recording.md` |
| `tools/planner_model/vendor/UPSTREAM.txt` | `tools/planner_model/vendor/upstream.txt` |
| `tools/nano_loop/OWNER_SCRIPT.md` | `tools/nano_loop/owner_script.md` |

比特流包里的 `selfcheck.txt`、`clocks.txt`、`build_summary.txt` 由构建与打包脚本生成，脚本在自己的构建目录里仍按大写原名输出；只是提交副本改了名。

### 许可证

本包自研代码以 MIT 许可发布，见 `LICENSE`；第三方组件（DPU IP、Vitis AI、YOLOv8、three.js 等）保留其原许可，清单见 `THIRD_PARTY_NOTICES.md`。
