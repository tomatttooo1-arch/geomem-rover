【DPU640OPT】分析：六类 YOLOv8s 在输入 640 下，能否在 KV260 上达到整条识别流水 ≥10 帧/秒？有哪些优化空间？中文写结论。

## 背景（你没有我们会话的上下文，以下是全部要点）
FPGA 国赛"智能寻物小车"：KV260（xck26，LUT 117,120、DSP 1,248、URAM 64）＋ Jetson Orin Nano。完整版要把识别搬到 KV260 的 DPU（DPUCZDX8G）上。owner 定 DPU 方案按序过三道闸：①帧率 ≥10 帧/秒，按整条识别流水吞吐（Nano 发帧→KV 出框，传输与计算可重叠；单帧时延另报不判门）→ ②精度 → ③DPU 运行时几何核 p99 恶化 ≤10%。

板上实测（DPUB1，今天）：当前比特流 m7_pl_m3 的 DPU 为 B1024、1 核、300 MHz，指纹 0x101000056010402。DPU 本体每帧时间（内核计时中位，周期计数器互证差 0.06%）：
- y8s_six 640：103.04 ms → 上界 9.71 帧/秒；QAT640：104.26 → 9.59
- y8s_six 512：63.77 → 15.7；416：49.05 → 20.4；y8n_33 640：34.11 → 29.3
- 640 的理论下界 96.5 ms（编译器运算量 ÷ 1024 运算/拍 ÷ 300 MHz），算力利用率已 92–94%。
- 4 线程吞吐 ≈ 1000/DPU 时间（单核串行）；VART 每任务固有开销约 7 ms，多线程时与计算重叠。
- CPU 输出处理（先筛后解码）640 约 5.2 ms，不是瓶颈；整板 DPU 跑满 +1.57 W。
主控初判"640 无论怎么重叠都到不了 10 帧/秒"——owner 要求下结论前再分析一轮，认为可能还有优化空间。**你的任务就是认真找这个空间，不预设结论。**

## 必读（都在 <run>/）
- DPUB1/README.md（实测与口径）；DPU1/README.md（DPU 集成、arch、端口、时钟）；DPURT0/README.md
- SLIM_AUDIT2/README.md（§3.2–3.3 DPU 配置项 OOC 实测：关 argmax −445 LUT、ALU 并行 4→2 −2,799、关通道增强 −2,670、时钟门控；§0–§5 全板资源与可瘦身清单；§4 新实现策略：拥塞清零）
- MERGE3/README.md（DPU 版整板：80,796 LUT、切片 96.8%、WNS +0.005；dsp 时钟 600 MHz 余量）
- DPUPIPE0/README.md（彩色帧通路：Nano 预处理到 DPU 输入 int8；相机 640×480 letterbox 到 640×640）
- <repo>/docs/competition/nationals/FULL_VERSION_ANALYSIS_20261103.md
- member-c 模型交接：<shared>/models/20261103-member-c-recog-quant2/member-c_RECOG_QUANT2_HANDOFF_20261103.md（模型、精度、编译脚本、arch_b1024.json）
- 官方资料：AMD PG338（DPUCZDX8G 资源表、各架构峰值、时钟上限）、Vitis AI 3.0 文档（编译选项、优化器/剪枝）、KV260 官方 smartcam/DPU TRD 用的配置。可联网查一手资料。

## 建议考察的方向（建议，不是死清单；发现更好的思路直接加）
1. 输入形状：相机 640×480，letterbox 到 640×640 有 25% 是灰边。改编矩形输入（640×480 或 640×448 等，32 对齐）可省多少运算？精度影响（同分辨率不缩放，理论上不掉）？
2. 更大 DPU 架构：B1152/B1600/B2304/B3136/B4096 各自峰值、PG338 资源（LUT/FF/DSP/BRAM/URAM）、能否和现有 PL 核（几何约 1.x 万、融合 9k、ROI、后处理核约 8–10k）一起放进 KV260；放不下时配合 SLIM_AUDIT2 的可瘦身项。
3. DPU 时钟：300 MHz 能否提高（器件速度等级、PG338 上限、现有 dsp 时钟 WNS），收益与风险。
4. 双核 DPU、或者 DPU 配置项（低 RAM 用法、URAM 代 BRAM 等）。
5. 模型侧：Vitis AI 剪枝/通道缩减、YOLOv8s 宽度系数、只保留六类对检测头的影响；每项的精度代价（member-c 的两个测试集口径：主测试 391 张、旧场景 803 张）。
6. 流水层面：DPU 与传输/CPU 已假设完全重叠；还有没有被忽略的点（例如批处理、VART 开销、多线程）。
7. 组合：哪几项叠加能让 640 稳定 ≥10 帧/秒（建议留 ≥15% 余量，因为整条流水还有传输和 Nano 端开销）。

对每个方向给：预计 DPU 每帧时间与帧率（写清是算术推断还是有实测依据）、资源代价、精度风险、工作量与谁来做（我们 FPGA 侧 / member-c 模型侧）、**坏处**（降级类必须写坏处）、最便宜的验证办法。最后给推荐路线与反向检验（推荐错了最早在哪里看到信号）。

## 可以做的便宜验证（可选）
- 宿主上用一个小构建槽跑更大 DPU 架构的 OOC 综合拿真实资源（可参照 SLIM_AUDIT2/tcl/dpu_ooc_runs.sh 的做法；输出写到 DPU640OPT/ 下）。构建槽：mkdir <locks>/nat_build_slot_{1,2,3} 原子取锁，写 owner 文件「DPU640OPT <用途> pid <pid> <时间>」，用完 rm owner＋rmdir；只占 1 个；起前确认 MemAvailable ≥12 GB；≤8 线程；禁 pkill -f。
- 若宿主上有可用的 Vitis AI 编译环境（查 member-c 交接里的编译脚本），可以对矩形输入或更大架构的 arch.json 重编 xmodel 只读编译器给出的运算量/子图数；没有就算术估计并写明。不要动 GPU（宿主有守护进程会杀碰 GPU 的进程，必要时 CUDA_VISIBLE_DEVICES=）。

## 底线
- 不改任何仓库代码、不推 git；只在 DPU640OPT/ 下写文件。不写 shared/ 和队友目录（member-c、member-b）。
- 数字带口径与来源；推断与实测分开写。
- 结论写进 <run>/DPU640OPT/README.md；最终回答 ≤40 行：每个方向一行（预计帧率/资源/精度风险/坏处）、推荐路线、需要 owner 决定的事。
