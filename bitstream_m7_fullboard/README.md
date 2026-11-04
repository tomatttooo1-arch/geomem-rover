# bitstream_m7_fullboard —— 整板比特流（产品默认；规划终版、记忆、搜索三个核同时装入）

这是本作品产品默认的比特流（app 名 `m7_pl_m7s17`）：在 m5 配置（`bitstream/`）各核与 DPU 的基础上，把三个扩展核一起装进同一块 KV260：
规划核终版（GMAP3 v9）、记忆核（MEMK4）、搜索核第二版（微程序引擎，源码 `hw/srch_v2/`）；ROI 核沿用 m5 的版本。
一条命令 `bash tools/golive/start_all.sh` 默认加载它，记忆、搜索、规划终版都在 PL 上运行。

## 构建结果

| 项目 | 数值 | 出处 |
|---|---|---|
| 构建号 BUILD_ID | 0x063ba99b（源码提交 63ba99b） | `selfcheck.txt` |
| 时钟 | PL 100 MHz / 166.67 MHz，DPU 300 MHz（B1024） | `clocks.txt` |
| 实现策略 | 默认策略 c1 之上，布局 ExtraTimingOpt、物理优化与布线 AggressiveExplore | `build_summary.txt`（只记基础策略 c1）与本节 |
| 建立时间最差裕量 WNS | +0.016 ns（全部时序约束满足，TNS 0） | `timing_route_summary.rpt` |
| 保持时间最差裕量 WHS | +0.009 ns | `timing_route_summary.rpt` |
| CLB LUT | 83,750 / 117,120（71.51%） | `utilization_route.rpt` |
| CLB 寄存器 | 106,984 / 234,240（45.67%） | `utilization_route.rpt` |
| BRAM | 129.5 / 144（89.93%） | `utilization_route.rpt` |
| URAM | 50 / 64（78.13%） | `utilization_route.rpt` |
| DSP | 366 / 1,248（29.33%） | `utilization_route.rpt` |

三个扩展核在整板中的资源（`utilization_route_hier.rpt`）：

| 核 | LUT | FF | RAMB36 / RAMB18 | URAM | DSP |
|---|---|---|---|---|---|
| 规划核终版 GMAP3 v9（planner） | 5,631 | 6,395 | 45 / 8 | 0 | 18 |
| 记忆核 MEMK4（memk） | 1,454 | 907 | 4 / 0 | 2 | 4 |
| 搜索核第二版（srch，`hw/srch_v2/`） | 5,035 | 6,083 | 3 / 2 | 13 | 51 |

搜索核换成第二版后，整板 LUT 从上一版整板（构建号 0x06661def，搜索核为 `hw/srch/` 的 c8 版，整板中 9,754 LUT）的 89,426 降到 83,750。

地址映射见 `address_map.csv`（记忆核控制口 0xA00E_0000、BRAM 口 0xA00E_8000，搜索核 0xA00D_0000，规划核 0xA00C_0000）。
`timing_route_summary.rpt` 是路由后时序报告截到逐路径明细之前的部分（总表、时钟表、时钟域内与跨时钟域汇总）。

## 板上核级冒烟（2026-11-04，KV260 台架）

加载这份比特流后（构建号读回 0x063ba99b），三个扩展核各跑一组关键向量，与宿主上的 C 模型 / 黄金输出逐位比较：

| 核 | 结果 | 比较范围 | 单次调用时延（板上实测） |
|---|---|---|---|
| 记忆核 MEMK4（0xA00E_0000，BRAM 口 0xA00E_8000） | PASS | 录制调用序列的前 500 次，每次比较结果区 1,024 个字，共 512,000 字，0 次不一致 | 驱动整次调用中位数 2.50 ms（含 PS 侧 Python 逐字读写） |
| 规划核终版 GMAP3 v9（0xA00C_0000） | PASS | 两组势场输入（80×80、200×200）的势场与标志共 186,904 字节、带取路径 187,732 字节，0 字节不同；73 个地图操作每步比较整块地图区 1,173,088 字节，0 步不一致 | — |
| 搜索核第二版（0xA00D_0000） | PASS | 测试轨迹全部 32 条记录按序执行，每条之后比较 0x60000 字节的 DDR 镜像（黄金输出为 c8 版定点 C 模型录制），共 12,582,912 字节，0 条不一致 | 观测平均 0.81 ms（15 次），选点平均 1.26 ms（3 次） |

摘录见 `docs/evidence/golive_m7/smoke_table.md`。

## 板上整机运行（2026-11-04，KV260 台架）

一条命令启动整机（`MEM=pl`、`SRCH=pl`、`PLN=gmap3`，640 识别模型，前处理与后处理走 PL），构建号读回 0x063ba99b，
连续运行 250.6 s，用 `python3 tools/golive/judge.py <日志目录> 200 gmap3` 按判据 C1–C9 判读（结果见 `board/results/golive_m7/`）：

| 项目 | 结果 |
|---|---|
| 深度链（几何→融合→代价图） | 稳态 26.3 帧/秒，全程帧数 7,038 帧 |
| 规划势场 | 509 次，1.90 次/秒；相机→势场中位 / p95 46.4 / 65.1 ms |
| 识别（640 模型，PL 后处理） | 稳态 7.11 帧/秒，全程帧数 1,901 帧；相机→物体框中位 / p95 159.2 / 177.1 ms |
| 规划核终版（核内取路径） | 510 次取路径全部与 PS 参考一致 |
| 记忆核 | 507 条上行的记忆全部来自 PL 记忆核 |
| 搜索核第二版 | 26 轮搜索全部在核上完成，一轮中位 2.31 s；首轮与 A53 软件结果一致（软件 9.36 s，核路径 2.53 s） |
| 整机功耗（SOM 5 V 轨） | 均值 6.27 W；同一比特流已加载、应用进程全停时空闲 3.96 W |

表中帧数为整次运行的全程帧数；`board/results/golive_m7/chain_report.txt` 是去掉启动段后的稳态统计，其中深度 6,774 帧、彩色 1,829 帧。

## 文件

- `m7_pl_m7s17/`：`m7_pl_m7s17.bit.bin`、`m7_pl_m7s17.dtbo`、`shell.json`（KV260 app 包）
- `m7_pl.hwh`：硬件描述；`m7_pl_m7s17.dtso.txt`：设备树覆盖源文件
- `board_bringup.sh`：板上加载与寄存器自检脚本；`m7pl_test`：板上寄存器自检程序（aarch64）
- `selfcheck.txt`：打包时的自检（地址、中断、头文件一致性）；`clocks.txt`、`build_summary.txt`：构建配置
- 校验：`cd bitstream_m7_fullboard && sha256sum -c SHA256SUMS`

加载：在 KV260 上该目录执行 `sudo M7PL_APP=m7_pl_m7s17 ./board_bringup.sh`，把 app 包装到 `/lib/firmware/xilinx/` 并加载，构建号应读回 0x063ba99b；
之后一条命令启动脚本用 xmutil 加载它。另一份可选比特流见 `bitstream/`（`BIT=m5`，构建号 0x0f35f07f）。

## 从源码重建这一版

块设计、构建脚本与 `bitstream/` 是同一份源码，区别只在三个扩展核的 IP 与两个软件寄存器头。

1. 换寄存器头。本包 `hw/m7/pl/sw/` 下的规划核寄存器头保持 m5 版本，这样 m5 配置的头文件检查能通过；
   整板版本所需的头放在 `hw/m7/pl/sw/merge10_m7/`。重建整板前先换回去：
   `cp hw/m7/pl/sw/merge10_m7/m7pl_planner_regs.h hw/m7/pl/sw/merge10_m7/m7pl_srch_regs.h hw/m7/pl/sw/`
   （头文件只影响驱动与打包检查，不影响比特流本身。）
2. 按顶层 README「重建比特流」一节设好 `FPGA_IP_REPO`、`FPGA_DPU_IPREPO`、`FPGA_PREP_IP`、`FPGA_PLANNER_IP`，
   并像那里一样先把目录初始化为 git 仓库（构建号取自提交号）。
3. 三个扩展核的 HLS 导出（sha256 前缀）：规划核终版 GMAP3 v9 `480886f6`，记忆核 MEMK4 `8e9daf92`，
   搜索核第二版 `10670556`（由 `hw/srch_v2/hls/` 导出）。在仓库根目录执行（观测核参数留空；三个实现指令与本版构建一致）：

```bash
FPGA_M7_PL_PLACE_DIRECTIVE=ExtraTimingOpt FPGA_M7_PL_PHYSOPT_DIRECTIVE=AggressiveExplore FPGA_M7_PL_ROUTE_DIRECTIVE=AggressiveExplore \
LPD_FIFO_NEW=1 PLANNER_SHA256=480886f6 SRCH_SHA256=10670556 MEMK_SHA256=8e9daf92 \
bash scripts/build/launch_whole_m7.sh <标签> <规划核 export.zip> <搜索核 export.zip> <记忆核 export.zip> ""
# 先用 DRY=1 打印配置核对：应看到 FPGA_M7_PL_MEMK_IF=bram、FPGA_M7_PL_LPD_FIFO_NEW=1、三个哈希前缀
```

4. 产物目录 `builds/impl-m6a-mk4-<标签>/`；打包：
   `bash hw/m7/pl/tools/package_app.sh <产物目录> <IP 仓库目录> <输出目录> m7_pl_m7s17`。
   Vivado 实现结果不保证逐字节复现，判据是时序满足、资源同量级、自检 PASS。
