# FPGA 各硬件模块优化前后对比（docs/evidence/optimization/）

口径：每个 PL 核只和自己比（早期版本 → 最终版本），或和同一块芯片上的 Cortex-A53 单核跑同一算法、同一输入比。
“优化后”的资源一律取上一版 m7 整板比特流（BUILD_ID 0x06661def）布线后分层报告，
核心行摘录在 `fullboard/fullboard_core_rows.txt`（该报告本身不随包；本包提交的整板 0x063ba99b 只把搜索核换成第二版，
其分层报告是 `../../../bitstream_m7_fullboard/utilization_route_hier.rpt`，搜索核第二版在其中为 5,035 LUT / 51 DSP / 13 URAM）（列：Total LUT / Logic LUT / LUTRAM / SRL / FF / RAMB36 / RAMB18 / URAM / DSP）。
整板时钟：几何核与核验核 166.67 MHz，其余 PL 核 100 MHz，DPU 300 MHz（B1024）。
出处路径均相对于本目录；`../xxx` 指上级 `docs/evidence/` 里已有的文件，`../../../xxx` 指代码包根目录。

## 总表

| 核 | 优化前（版本 / 口径） | 优化后（m7 整板版本） | 指标 | 倍数 / 降幅 | 关键优化手段 | 出处文件 |
|---|---|---|---|---|---|---|
| 几何（帧核 frame） | 上一版车上浮点几何核（v3fc2 整板布线后分层；板上 142.86 MHz rec30 每帧中位）；同芯片 A53 单核跑定点算法 | 定点几何核 X3（整板分层；rec30 一帧联合仿真 263.3k 拍 @166.67 MHz） | LUT 38,936 → 10,423；DSP 145 → 20；每帧 3.82 ms → 1.58 ms；A53 56.1 ms → 1.58 ms | LUT −73.2%，DSP −86.2%；时延 2.4×；对 A53 35.5× | 浮点射线遍历改为可证位宽的定点算法（与精确有理数逐格相同），常数由 PS 每帧下发、12 个 64 位寄存器收成一个数组 | `../geometry/geo_board_and_cosim.txt`（BOARD5 L98，GEO_FX1 L105–108，GEO_FX3 L11，MERGE1 L137）；`../a53_baseline/a53_bench_results.md` geo_fx 行；`fullboard/fullboard_core_rows.txt` L34 |
| 融合与代价图（rolling） | 改前 b0_base（同一导出流程 OOC 综合，含代价图 GRID）；板上旧版融合核；同芯片 A53 单核 | 融合核 c3（整板分层） | LUT 11,646 → 9,075（同口径 OOC 综合），整板 8,597；Block RAM 19.5 → 6.5 块；DSP 12 → 6；每帧融合（INTEGRATE，cosim_r10 轨迹）板上中位 0.539 → 0.067 ms；A53 0.414 → 0.067 ms | LUT −22.1%，BRAM −66.7%，DSP −50%；时延 8.1×；对 A53 6.2× | 融合循环重写为只走被碰到的字、行装载与逐字更新合并成一条 II=1 流水；清除引擎改预算掩码；核顶层状态寄存器去副本 | `fusion/fusion_excerpts.txt`（SLIM_FUSE2 L27/L32，FUSE_SLIM3 L50/L53/L55，BOARD4 L32/L53）；`../a53_baseline/a53_bench_results.md` 分组表 fuse_r10 integrate 行；`fullboard/fullboard_core_rows.txt` L44 |
| 规划（planner） | 势场部分改前版（OOC 布线后，100 MHz）；同芯片 A53 单核同一高代价 16 轮 80×80 请求 | 规划核终版 v9（势场＋核内取路径＋全局地图合并，整板分层） | 势场部分 LUT 4,411 → 3,509，DSP 2 → 1，BRAM 9 → 7.5 块；整板 5,708 LUT / 18 DSP；80×80 建场 A53 12.90 ms → PL 2.22 ms（222,442 拍 @100 MHz） | 势场部分 LUT −20.4%；对 A53 5.8× | 单一可换向扫描器＋缓存、预计算 9 位入口/行走图、地址与计数收窄到可证位宽；取路径直接读片上势场 | `planner/planner_excerpts.txt`（planner_v3_slim/SUMMARY.txt L9，PLNPATH0 L16–L22，GMAP2 L94）；`../a53_baseline/a53_bench_results.md` 分组表 plan_key2 r16 行；`fullboard/fullboard_core_rows.txt` L40 |
| 识别前处理（prep） | 首版 HLS 实现 s2（csynth）；同芯片 A53 单核软件前处理（640×480 → 640×640，30 帧中位） | 交付版 s18（整板分层） | LUT 18,712 → 8,329（同口径 csynth），OOC 3,551，整板 3,523；DSP 26 → 22；每帧 A53 18.3 ms → PL 5.1 ms（含读环） | LUT −55.5%（csynth）；对 A53 3.6× | 1 像素/拍全 II=1 流水；配置挪进 DDR 参数块；读写 64 拍突发＋BRAM FIFO 攒块；齿轮箱改移入窗；量化改每通道 1 个 DSP 的仿射式 | `prep/prep_excerpts.txt`（qor_steps.csv L2/L24/L25）；`../a53_baseline/a53_prep_post_note.txt` L59；`fullboard/fullboard_core_rows.txt` L41 |
| 识别后处理（detpost） | 参考模型原样包顶层的第 0 步（csynth）；同芯片 A53 CPU 后处理（30 帧中位） | 后处理核终版（整板分层） | LUT 20,485 → 8,659（同口径 csynth），整板 3,732；DSP 54 → 6；每帧 A53 6.70 ms → PL 0.443 ms | LUT −57.7%（csynth），DSP −88.9%；对 A53 15.1× | 结构重写：128 位扫描＋dataflow 候选管理＋计数排序＋按 NMS 序回读解码；64 位除法改 14 步恢复余数除法；NMS 乘法收窄 | `post/post_excerpts.txt`（DETPOST1 L19/L23，qor_steps.csv L3/L10）；`../a53_baseline/a53_prep_post_note.txt` L60；`fullboard/fullboard_core_rows.txt` L32 |
| 核验（深度证据 roi） | 上一版证据核（v3fc2 整板布线后分层）；同芯片 A53 单核同一 19 框批次 | 瘦身证据核（整板分层；166.67 MHz） | LUT 10,009 → 6,151；FF 11,745 → 7,634；每帧（19 框）A53 70.3 ms → PL 1.41 ms | LUT −38.5%，FF −35.0%；对 A53 49.9× | 判定里三次双精度除法改精确整数比较；双精度加减共用一个运算器；常数比较改位模式比较；每框像素数改用框面积 | `roi/roi_excerpts.txt`（MERGE1 L139，SLIM_ROI1 L64–L71，BOARD3 L132–L133）；`../a53_baseline/a53_bench_results.md` roi19 行；`fullboard/fullboard_core_rows.txt` L43 |
| 记忆（memk） | 上一版记忆核 MEMK3（OOC） | MEMK4（整板分层） | LUT 9,180 → 1,452；FF 9,041 → 907；DSP 55 → 4；实车流每帧中位 0.42 ms | LUT −84.2%，FF −90.0%，DSP −92.7% | 改为一个共用运算单元＋微程序顺序器；命令/结果区放到核外 BRAM 口 | `memory/memory_excerpts.txt`（MEMK4 L12/L15/L28–L32）；`../ooc_cores/memory_planner_ooc.txt`；`fullboard/fullboard_core_rows.txt` L38 |
| 搜索（srch，整板版） | 首个布线版本 c2（OOC 布线后，关键路径 20.65 ns）；同芯片 A53 软件选点（首轮） | 搜索核 c8（整板分层，100 MHz 收敛） | LUT 11,612 → 9,754；DSP 142 → 76；首轮选点 A53 9.6 s → 核路径 2.7 s | LUT −16.0%，DSP −46.5%，周期 20.65 → 9.67 ns；对 A53 3.6× | 乘法器按变量绑定、m_axi 缓冲改 LUTRAM、各模块按 100 MHz 重排流水 | `search/search_excerpts.txt`（SRCHK3 L29，SRCHK6 L14）；`fullboard/fullboard_core_rows.txt` L50 与其中 bitstream_m7_fullboard/README.md L48 摘录 |
| 搜索（第二版 hw/srch_v2） | 同上 c8（OOC 9,683 LUT / 79 DSP；整板中 9,754 LUT / 76 DSP） | 微程序版 m17（OOC 布线后；提交的整板 0x063ba99b 中 5,035 LUT / 51 DSP） | LUT 9,683 → 5,049；DSP 79 → 52；FF 13,012 → 6,101；联仿 71/71 逐字节相同；整板中 LUT 9,754 → 5,035，整板合计 89,426 → 83,750 | LUT −47.9%，FF −53.1%，DSP −34.2% | 改写为一个共用运算单元＋微程序＋两个专用单元 | `search/search_excerpts.txt`（SRCHK6 L14/L16） |
| DPU 识别链整体 | 前处理与后处理都在 A53 上的整条识别链（640 模型） | 前处理核＋DPU＋后处理核全在 PL（整机运行） | 输出 6.20 → 7.13 帧/秒；到达→出框中位 180.4 → 157.9 ms；整机 7.12 帧/秒，为 DPU 单路上界 7.24 帧/秒的 98.3%；DPU B1024 整板 33,830 LUT / 230 DSP / 26 URAM | 帧率 +15%，时延 −12.5% | 前后处理下放 PL，与 DPU 流水重叠，A53 只做调度 | `../a53_baseline/a53_soft_prep_chain_report.txt`、`../a53_baseline/pl_post_chain_report.txt`、`../a53_baseline/a53_prep_post_note.txt` L59–L60；`../golive_m7/chain_report.txt`；`../dpu_bound/note.txt`；`fullboard/fullboard_core_rows.txt` L33 |

## 备注（口径）

1. **几何时延**：3.82 ms 是上一版浮点核在车上（142.86 MHz）rec30 每帧设备时间中位；1.58 ms 是定点版 rec30 一帧联合仿真拍数按 166.67 MHz 换算（不含上电首帧的 32,768 拍位平面清零）。几何核演进的中间档（166.67 MHz 浮点 2.10 ms、射线抽稀 1.62 ms）见 `../geometry/geo_board_and_cosim.txt`。整板 10,423 LUT 与单核 OOC 10,304 LUT 是同一核（跨边界优化差 119）。
2. **融合**：整板装的是 c3（FUSE_SLIM3 L50），INTEGRATE 拍数与 b7_final 在三条联仿轨迹上差 < 0.1%（FUSE_SLIM3 L55），所以板上 0.067 ms（BOARD4，b7_final 那版）代表整板版本。11,646 → 9,075 是同一导出流程的 OOC 综合口径，两端都含 GRID；整板 8,597 是布线后口径。A53 对照取 a53_bench_results.md 分组表里 fuse_r10 的 integrate 组中位 0.414 ms（同一 cosim_r10 轨迹，与 PL 同一 C++ 源码）。
3. **规划**：4,411 → 3,509 只比势场部分（同口径 OOC 布线后）；整板 5,708 LUT 多出的部分是核内取路径（PLNPATH0，+630 LUT）与全局地图合并（GMAP2/GMAP3 v9）。A53 12.90 ms 是 plan_key2 的 r16 组（高代价 16 轮，80×80）单核中位，PL 2.22 ms 是同一请求的整次调用拍数 222,442 @100 MHz。整板板上冒烟中 80×80 势场驱动整次调用 2.28 ms（`fullboard/fullboard_core_rows.txt` 中 bitstream_m7_fullboard/README.md L33 摘录）。
4. **前处理**：18,712 → 8,329 是两版 csynth 报告同口径；交付版 OOC 布线后 3,551、整板 3,523。PL 5.1 ms 是整机链路中从读环到前处理完成的每帧时间，与 A53 18.3 ms 同口径。
5. **后处理**：20,485 → 8,659 是两版 csynth 报告同口径；整板布线后 3,732 LUT。A53 6.70 ms 与 PL 0.443 ms 都是整机链路中 DPU 完成 → 后处理完成的每帧中位。
6. **核验**：10,009 与 6,151 都是整板布线后分层口径（前者 v3fc2，后者 m7）。同一流程 OOC 综合口径是 10,398 → 6,450（−38.0%，SLIM_ROI1 L64/L68）。A53 roi19 与 PL 1.41 ms 都是 rec30 帧上同一 19 框批次（BOARD3 L132：290 帧 5,510 行，即每帧 19 框）。补充：技能包 fpga-hls 的一次自动优化案例在同一核的 HLS 工程上把联仿最大时延从 271,534 拍降到 143,560 拍（−47.1%），见 `../../../skill/fpga-hls/README.md` L61；该数是技能包工作流的效果，不列入本表主行。
7. **记忆**：9,180 是 MEMK3 的 OOC 布线后数，1,452 是整板布线后数（同一核单独 OOC 为 1,487）。
8. **搜索**：整板版 c8 单独 OOC 为 9,683 LUT / 79 DSP，整板中为 9,754 LUT / 76 DSP。11,612 → 9,754 是 OOC → 整板口径。A53 9.6 s 与核路径 2.7 s 是整机运行首轮选点，两者结果相同（`fullboard/fullboard_core_rows.txt` 中 bitstream_m7_fullboard/README.md L48 摘录）。第二版 m17 与 c8 同为 OOC 布线后口径；相对首个布线版本 c2 的 11,612 LUT 为 −56.5%（doc_table.md 用这个口径）。
9. **DPU 识别链**：6.20 帧/秒与 180.4 ms 是前处理 soft、后处理 CPU 那次 30 帧左右的整链报告（`../a53_baseline/a53_soft_prep_chain_report.txt`，同次后处理为 CPU 版，见 note L60）；7.13 帧/秒与 157.9 ms 是前后处理都在 PL 的同口径报告。98.3% = 整机 7.12 ÷ DPU 上界 7.24。
10. 整板其余部分（报告同一文件）：许可核 permit 389 LUT。

## 未找到

- **观测核（OBSK1）**：只有 C 架构模型与微程序解释器的拍数，没有 RTL 综合 / OOC 资源与联仿结果，也未装入 m7 整板比特流，表中不列行。
- 几何核的最早版本（603.7 ms 第一版）在运行目录里没有一手结果文件，未列入。
- 记忆核 MEMK3 → MEMK4 的同口径时延对比、DPU 本体（B1024）自身的优化前后资源对比：未找到对应产物。
