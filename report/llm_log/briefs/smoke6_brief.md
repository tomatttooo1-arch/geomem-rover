# SMOKE6 brief：m5 上板一次带全（FPGA 国赛线主控 11-04 派，worker-opus-high）

你是 FPGA 国赛线（全国 FPGA 创新设计竞赛，KV260 单板寻物机器人）主控派出的板上执行代理。RUN = <run>。

## 先读
`RUN/SMOKE5/brief.md`（"第二段"五项就是你的任务）、`RUN/SMOKE5/README.md`（一键冒烟用法、前提、每项"不通过时看到什么"）、`RUN/SMOKE5/HANDOVER.md`。m5 运行包：`RUN/MERGE7/builds/pkg-m5_166/`（SELFCHECK.txt 里有期望 BUILD_ID，SHA256SUMS 要能过）。

## 要做（顺序）
1. `smoke/host/stage.sh <m5 包目录>`（**必须重跑**，现在的 stage/ 是 m4 试拼）→ push.sh → start.sh → status.sh → pull.sh。结果表 `out/results.md`、`out/verdicts.tsv` 拷回 `RUN/SMOKE6/`。
2. 相机端换算的几何对比（SMOKE5 brief 第二段第 2 项）。
3. A53 计时：`RUN/GMAP0/a53/board/run_a53gmap.sh`、`RUN/OBSREF0/board/run_obsbench.sh`、`RUN/PLNPATH0/board/` 的计时包；`RUN/SRCHK3/board/`、`RUN/MEMK3/` 下若届时已有就顺手跑，没有就跳过。
4. 后处理核取舍要的数（第 4 项）以及"各核板上 PL 时间 vs A53 优化版"一张表。
5. 两个默认开关建议（第 5 项）。
冒烟里任何一项不过：先按 README 的"不通过时看到什么"定位、记录现象与日志路径，不在板上改硬件；软件侧小问题（脚本路径、权限）可以修了重跑，写进 README。

## 纪律（硬）
- /dev/shm 里 ubuntu 的文件会被删，转储用 root 写到 home 下。禁 `pkill -f`，只按 pid 停。git 只推显式 ref。被系统拒绝的操作停下写进报告，不绕过。
- 异常只告警继续，不加任何拒起/停车类防御。长任务用前台阻塞循环等。输出节流。约 20 万 token 写 `RUN/SMOKE6/HANDOVER.md` 收尾。
- 交付 `RUN/SMOKE6/README.md`（冒烟结果表、相机换算结论、A53 对照表、后处理取舍数、开关建议、收尾状态：锁已释放、车未动、板上留哪个比特流）。最后回答 10 行内中文摘要。
