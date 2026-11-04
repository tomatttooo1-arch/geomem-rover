# SRCHK4 brief：概率搜索核 HLS 第二棒——压 LUT 到门内并交 export（FPGA 国赛线主控 11-04 派，worker-opus-high）

你是 FPGA 国赛线（全国 FPGA 创新设计竞赛，KV260 单板寻物机器人）主控派出的执行代理。RUN = <run>。HLS 工作调用 skill fpga-hls，时序与方法学看 vivado-checks。

## 先读
`RUN/SRCHK3/HANDOVER.md`（全文，**§5 就是你的工作顺序**）、`RUN/SRCHK0/DESIGN.md`、`RUN/HWDESIGN_RULES_20261104.md`（末尾"补充"五条写码规矩）。代码：worktree `<worktree>/nat-srchk3-20261104`，分支 feat/member-a/nat-srchk3-20261104  at b448e45（上一棒代理已结束，你直接在这个 worktree 接着做；先把分支推上去）。

## 现状
HLS 估计 c0 37,703 LUT → c2 25,863 LUT／124 DSP／BRAM18 5／URAM 13，门是 OOC ≤4,000。c2 的 cosim＋OOC 正在跑（pid 1584055，持 nat_build_slot_1，输出 `RUN/SRCHK3/build/c2_cosim_impl.out`，结束自己放锁）。**第一步：前台阻塞循环等它结束（每次 Bash ≤10 分钟，循环直到进程退出），读 cosim 是否逐字节过、OOC 实测 LUT/FF/DSP/BRAM/URAM/WNS。** 有了 OOC 实数再按它排第二批改写的优先级（HLS 估计对控制逻辑常偏大 2.5–3 倍，以 OOC 为准）。

## 主控已定
1. 核内 DDR 地址按 32 位算、DMA 缓冲必须整块在 4 GB 以下（驱动断言）：接受（S_AXI_LPD 只映射 LPD_DDR_LOW，本来就在 2 GB 以下）。
2. STATUS 位布局以实现为准（[7:0] 错误、[15:8] 版本、[23:16] 操作码），README 与驱动照实现写。
3. **预先批准更深的结构改动**（不必再回来问），只要功能与 SW TB／两组 cosim 向量逐字节一致：例如把锥体判定、射线步进、收益乘加做成**一套时分复用的共享数据通路**由一个小微码/表驱动的序列器调度（替代 294 状态的大 FSM 和多份并行算子）；用延迟换面积可以，上限是每帧观测 ≤1 ms、一次选点 ≤2 ms（搜索 5 Hz 更新，帧间隔 200 ms）。
4. 门：OOC **≤4,000 LUT** 为目标；做完第二批与上面的结构改动后若在 4,000–5,000，交回主控定；>5,000 继续改。DESIGN §7.3 第 2、3 项（栅格初始化或先验移 PS）仍不准用。RAMB36 ≤4、URAM ≤14、WNS ≥0  at 100 MHz 不变。

## 要做
按 HANDOVER §5 第 2–6 条做完：第二批改写（每批 SW TB → HLS csim → csynth）→ OOC 过门 → export.zip 与 sha256 → A53 计时包补阴性对照并放 `RUN/SRCHK3/board/` → README（改前改后资源表、cosim、拍数、主卖点）→ post 标签、只推显式 ref。

## 纪律
构建槽只用 `<locks>/nat_build_slot_{1,2,3}`：**一条 mkdir 原子取槽并立刻写 owner**，用完删 owner 再 rmdir；开前 MemAvailable ≥12 GB、线程 ≤8；`nat_build_whole` 不碰。不 `--tags`、不合 main。禁 `pkill -f`，只按 pid 停。不动板子。输出节流。长任务用前台阻塞循环等，不要后台起了就结束回合。**约 20 万 token 时写 `RUN/SRCHK4/HANDOVER.md` 收尾**（上一棒超到 30 万，别再犯）。最后回答 10 行内中文摘要。
