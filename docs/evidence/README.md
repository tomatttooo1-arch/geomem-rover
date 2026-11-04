# 文档数字出处目录（docs/evidence/）

本目录收录作品文档中各项数字对应的原始结果文件或其关键片段。片段文件逐行注明原文件名与行号，数字与原文件一致。
路径均相对于本目录。

## 1. 整机运行（m7 整板比特流 m7_pl_m7s17，BUILD_ID 0x063ba99b，640 模型，记忆核、搜索核第二版、规划核终版全开；2026-11-04）

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 250.6 s | 整机连续运行时长（running 到 stop 两个时间戳之差） | `golive_m7/golive_m7_counts.txt` §Run duration |
| 510 次，0 处不一致 | 规划核取路径 510 次，每次与 PS 参考逐点比对一致（path_match=510，path_mismatch=0） | `golive_m7/golive_m7_counts.txt` §Planner |
| 507 条 | 记忆上行 507 条，全部来自记忆核（mem.src=pl，fallbacks=0） | `golive_m7/golive_m7_counts.txt` §Memory uplink |
| 26 轮 | 搜索核选点 26 轮，全部在核上执行（impl=pl），核调用 2,414 次，首轮核与软件选点相同（软件 9.36 s，核路径 2.53 s） | `golive_m7/golive_m7_counts.txt` §Search core |
| 7.11 帧/秒、26.3 帧/秒 | 识别链与深度链输出帧率 | `golive_m7/chain_report.txt` |
| 18.0 / 24.0 ms、46.4 / 65.1 ms、159.2 / 177.1 ms | 相机→代价图、相机→势场、相机→物体框（中位 / p95） | `golive_m7/chain_report.txt` |
| 1.98 核 | 整机运行时 A53 四核合计占用（各进程分项同文件） | `golive_m7/golive_m7_counts.txt` §CPU |
| 3 个核 PASS | 同一比特流加载后，三个扩展核（记忆、规划终版、搜索第二版）关键向量与 C 模型逐位比较，0 处不一致 | `golive_m7/smoke_table.md` |

## 2. 功耗（INA260，SOM 5 V 轨，10 Hz）

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 6.27 W（p95 6.75 W） | 上述整机运行全程均值，2,300 个样本 | `golive_m7/pwr_run.csv`，汇总 `golive_m7/power_summary.txt` |
| 3.96 W | 同一比特流已加载、应用进程全停时的空闲均值，15 s，150 个样本 | `golive_m7/pwr_idle.csv`，汇总 `golive_m7/power_summary.txt` |
| 2.31 W | 6.27 − 3.96，两者同一块板、同一比特流 | 同上 |

## 3. 几何核演进

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 3.82 ms | 上一版车上几何核（142.86 MHz）rec30 每帧设备时间中位 3.8211 ms | `geometry/geo_board_and_cosim.txt`（L98）；逐帧 `geometry/board_v3fc2_rec30_frames.csv` |
| 2.10 ms | 166.67 MHz 版 rec30 每帧中位 2.0986 / 2.0978 ms | 同上（L98）；逐帧 `geometry/board_m2_166_rec30_frames.csv` |
| 1.62 ms | 射线抽稀开启后 rec30 每帧中位 1.6160 ms | 同上（L99）；逐帧 `geometry/board_m4_rec30_switch1_frames.csv` |
| 2.32 ms | TUM 640×480 抽稀开启每帧中位 2.318 ms | 同上（L101）；逐帧 `geometry/board_m4_tum_switch1_frames.csv` |
| 1.58 ms | 定点版几何核 rec30 一帧联合仿真约 263.3k 拍 @166.67 MHz | `geometry/geo_board_and_cosim.txt`（GEO_FX1 L105–108） |
| 2.19 ms | 定点版几何核 car570 第 100 帧（640×480）联合仿真 365,332 拍 @166.67 MHz | 同上 |
| 38,936 LUT / 145 DSP | 上一版几何核在整板布线后分层报告中的面积 | `geometry/geo_board_and_cosim.txt`（MERGE1 L137） |
| 10,304 LUT | 定点版几何核单核布线后面积（整板中为 10,423 LUT，见包内整板分层报告） | `geometry/geo_board_and_cosim.txt`（GEO_FX3 L11、L177） |
| 1.611 → 0.200 ms | 写回改突发后空帧开销 | `geometry/geo_board_and_cosim.txt`（L104） |

## 4. A53 单核对照（表 3-7）

同一块芯片的 Cortex-A53 单核运行与 PL 核同一算法、同一输入，输出逐位等于 PL 核黄金。

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 56.1 ms | 几何（定点算法）每帧单核中位 56.140 ms | `a53_baseline/a53_bench_results.md`，geo_fx 行 |
| 0.362 ms | 融合每条记录 | 同上，fuse_r10 行 |
| 0.470 ms | 代价图（GRID）每条记录 | 同上，grid_formal 行 |
| 9.86 ms | 规划势场关键两例每次请求 9.857 ms | 同上，plan_key2 行 |
| 70.3 ms | 深度核验 19 框一帧 70.289 ms | 同上，roi19 行 |
| 18.3 ms | 识别前处理软件版每帧（30 帧中位） | `a53_baseline/a53_prep_post_note.txt`；链路报告 `a53_baseline/a53_soft_prep_chain_report.txt` |
| 6.70 ms | 识别后处理 CPU 版每帧（30 帧中位） | `a53_baseline/a53_prep_post_note.txt`；链路报告 `a53_baseline/a53_cpu_post_chain_report.txt`，PL 版对照 `a53_baseline/pl_post_chain_report.txt` |

## 5. 模块级时延（表 3-5）

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 0.067 ms | 融合核每次积分设备时间中位 0.0670 ms（板上） | `module_latency/module_latency_excerpts.txt`（BOARD5 L106） |
| 0.22 ms | 代价图 GRID 每次设备时间中位 0.2245 ms（板上，750 条） | 同上（BOARD5 L108） |
| 2.22 / 5.73 ms | 规划核 80×80（16 轮）/ 200×200（6 轮）整次调用 222,442 / 573,332 拍 @100 MHz | 同上（PLNPATH0 L21–22） |
| 0.42 / 0.81 ms | 记忆核实车流每帧中位 / 最坏 | 同上（MEMK4 L15、L75） |
| 0.28 ms | 搜索核每次选点 28,194 拍 | 同上（SRCHK5 L50） |
| 联仿拍数 | 前处理核 512 档一帧 322,900 拍 | 同上（PREP1 L16） |

## 6. DPU 单路上界

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 7.24 帧/秒、138 ms | 640 模型只跑 DPU，xdputil benchmark 1 线程 60 s：FPS 7.23671，即每帧 138.2 ms | `dpu_bound/xdputil_bench_640.txt`，`dpu_bound/note.txt` |
| 98.2% | 整机识别 7.11 帧/秒 ÷ DPU 上界 7.24 帧/秒 | `golive_m7/chain_report.txt` 与上一行 |

## 7. 正确性计数（表 3-9）

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 201,840 条射线 | 定点几何与精确有理数在 rec30 全部分歧射线上同格 | `correctness/correctness_counts_excerpts.txt`（GEO_FX1 L42–43） |
| 1,239 条 0 失配 | 融合核 grid_formal 回放逐位一致 | 同上（BOARD5 L21） |
| 1,639 帧 | 全局地图操作两趟回放，每帧 160,000 格逐位相同 | 同上（GMAP0 L33） |

## 8. 识别精度

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| mAP50 84.77% | 640 档 INT8 模型，车载主集 391 张，总 mAP50 | `recognition/recognition_map50.txt` |

## 9. 定位标定

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 0.301 → 0.140 m | 长趟（35.3 m）位置差 p95：原样里程 0.301 m，标定后 0.140 m；航向 14.1° → 1.13° | `localization/odom_calibration.txt`（SUMMARY L23、L61） |

## 10. 取帧占用

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 0.47 核 | 产品相机配置（彩色 YUYV＋硬件对齐＋同步，30 帧/秒，视差→深度换算交回相机）下取帧进程 30 s 窗口占用 0.468 核 | `camera/kv_cam_p1_cores.txt`，`camera/camera_cores_note.txt` |

## 11. 记忆核与规划核单核数

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 1,487 LUT / 909 FF，余量 2.0 ns | 记忆核单核布线后（100 MHz）；整板中为 1,452 LUT / 907 FF，见包内整板分层报告 | `ooc_cores/memory_planner_ooc.txt`（MEMK4 L11–12） |
| 3,747 / 3,777 | 规划核势场部分单核 LUT / FF | `ooc_cores/memory_planner_ooc.txt`（PLNPATH0 L16–17） |

## 文档正文其他数字

| 数字 | 含义与口径 | 包内出处 |
|---|---|---|
| 约 2.4 个处理器核 | PL 承担的几何、融合、代价图、前处理、后处理、深度核验若改由 A53 软件完成所需核数：56.1/33.3 ＋ 0.414/33.3 ＋ 0.470/33.3 ＋ 18.3/140.6 ＋ 6.70/140.6 ＋ 70.3/140.6 ≈ 2.39（深度链按 30 帧/秒，识别链按 7.11 帧/秒） | `extra/cpu_cores/README.md`（逐项指向 `a53_baseline/` 的对应行） |
| 整板 LUT 85,747（73.2%）→ 40,895（34.9%） | 同一组 PL 功能（几何、融合与代价图、深度核验、行驶许可）的两版整板：v3fc2 → m4；减少的 44,852 LUT 中，删除车屏叠加核及其互连 23,659，三个计算核瘦身 19,180 | `extra/lut_evolution/README.md`，两份布线后分层报告摘录 `util_hier_v3fc2_142.txt`、`util_hier_m4_166.txt` |
| 代价图与 Nav2 代价层 376 帧逐格一致 | 实车录制轨迹回放 376 帧：对软件桥节点 0 格差异，对 Nav2 膨胀公式精确参照 0 格差异；阳性对照可抓到 | `extra/costmap_nav2/README.md`，`extra/costmap_nav2/costmap_grid_bench.json` |

## 优化前后对比

各硬件模块优化前后的资源与时延对比及出处见 [optimization/README.md](optimization/README.md)。
