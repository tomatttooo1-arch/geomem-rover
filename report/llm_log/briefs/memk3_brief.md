# MEMK3 brief：语义记忆核 HLS 实现（FPGA 国赛线主控 11-04 派，worker-opus-high）

你是 FPGA 国赛线（全国 FPGA 创新设计竞赛，KV260 单板寻物机器人）主控派出的执行代理。RUN = <run>。HLS 工作调用 skill fpga-hls，时序与方法学看 vivado-checks。

## 背景与并行关系
记忆核的设计已审定、C 模型已在实车 5,276 帧上和产品全一致。另一个代理（MEMK2）正在同一核的旧 worktree 里收尾判据①（跑 D3/D4、修 ERR_SLOT、两个阳性对照）。**为压缩时间，你与它并行**：从 f70faa5 开新分支做 HLS 改写，ERR_SLOT 的修法你自己按交接文件做进去；改写完你自己重跑判据①全部（D1prod、D2、D3、D4 和两个阳性对照），这样不依赖 MEMK2。不要在 MEMK2 的 worktree `<worktree>/nat-memk0-20261104` 里写任何东西。

## 先读
`RUN/MEMK0/DESIGN.md`（结构、位宽、存储、§5 手算预算、§10 判据）、`RUN/MEMK0/HANDOVER.md`（**"下一步"第 2–4 条就是你的工作说明，逐条照做**，含 HLS 改写要点、宽度修正、ERR_SLOT bug、"已知坑"）、`RUN/HWDESIGN_RULES_20261104.md`（**末尾"补充"：HLS 生成的选择逻辑常超手算 1.7 倍，五条写码规矩照做**）。

## 要做
1. 新 worktree `<worktree>/nat-memk3-20261104`，分支 `feat/member-a/nat-memk3-20261104` 从 f70faa5 开出；pre 标签。
2. 按 HANDOVER 第 2 条改 `memk0_core.cpp`（同一份源码 g++ 也能编，C 模型库就用它）；改完重跑判据①全部，结论写进 README（判据原文不改，不过的照 DESIGN §10 第 5 条报主控）。
3. csynth：逐函数对手算（去掉 m_axi 900、加 LUTRAM 约 400），任何一项超手算 1.5 倍先改再往下。
4. 联仿 2 组逐字节（D3 指令最多一帧、D2 观测最多一帧；testbench 从 PLMemory 录 cmd/res 与调用前后槽表）。
5. 一次 OOC  at 100 MHz：**LUT ≤2,500、RAMB36 当量 ≤6、URAM ≤2**、WNS ≥0。压不到先回报，不擅自把逐槽计算挪回 PS。
6. 交出 export.zip（路径与 sha256 写进 README）给 m6；A53 计时程序照 `RUN/A53BENCH0/` 格式（一条命令出中位数，板子不碰）。
7. `RUN/MEMK3/README.md`：改前改后资源表、判据结果、拍数、固定常数清单与坏处。post 标签、只推显式 ref。

## 纪律
构建槽只用 `<locks>/nat_build_slot_{1,2,3}`：**一条 mkdir 原子取槽并立刻写 owner**，用完删 owner 再 rmdir；开前 MemAvailable ≥12 GB、线程 ≤8；`nat_build_whole` 不碰。不 `--tags`、不合 main。禁 `pkill -f`，只按 pid 停。不动板子。输出节流（tail/grep，长内容落文件）。**长任务（cosim/OOC/判据回放）用前台阻塞循环等，不要后台起了就结束回合。** 约 20 万 token 时写 `RUN/MEMK3/HANDOVER.md` 收尾。最后回答 10 行内中文摘要。
