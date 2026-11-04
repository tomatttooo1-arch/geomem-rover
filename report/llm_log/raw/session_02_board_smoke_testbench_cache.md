# 原始交互记录 2：三核整板上板冒烟（规划核三次运行，定位到测试台缓存问题）

**这是什么**：一次完整的执行代理会话，任务是把主线中间版整板（记忆核、规划核终版、搜索核三个 PL 核）在 KV260 板上做冒烟：先在宿主上准备板上程序和黄金向量，再取板锁上板，逐位比对板上输出与宿主黄金。第一节是主控写的任务书全文，第三节是代理交回的报告。本文件由 `convert_session.py` 自动转换并脱敏。

**自我纠错出现在哪里（按第二节的步号找）**：
1. **阳性对照**（第 47–48 步）：宿主 mock 三项都 PASS 后，代理把三份黄金各改 1 字节，确认三项都报 FAIL，证明比对能抓错。第 47 步的 ⚠ 是带删除命令的写法被权限拦下，代理改成新建目录复制的写法（第 48 步）完成同一对照。
2. **规划核第一次板上失败 → 三次运行定位到测试台的缓存问题**（第 88–94 步，报告第三节"需要你知道的两件事"第 1 条）：
   - 第 1 次：地图操作部分失败，差异全在全局图。代理读驱动的缓存刷新代码（第 90–91 步），判断是测试台开头整区拷贝留下的脏缓存行被写回、盖掉了核的输出；
   - 第 2 次：测试台开头先整区刷一次缓存，前 30 个操作全对，剩下差异只在驱动不负责刷新的"扫过时间图"，且两次统计数不同——是 CPU 缓存里的旧数据；
   - 第 3 次：每个操作后由测试台整区作废缓存，73 个操作全对。
   结论落在"测试台有错、核和驱动没错"，驱动和核都没改；并说明只重跑规划核的理由（它不跨调用保存状态，另两个核会，不能重跑）。
3. **发现上板包缺一项检查**（第 75–76 步）：新包的上板脚本少了 m5 包里有的低功耗域总线复位检查，代理手动补跑并把"是否并进终版包"交主控定。

**陌生队伍可以复用的做法**：PS 与 PL 共享内存时，测试台自己写过的区域必须刷缓存、读之前必须作废缓存；一次失败后先看差异"落在哪块、两次是否稳定"，再区分是被测件错还是仪器错。

- 会话规模：106 轮，耗时约 42 分钟，结束状态 completed。

---

## 一、提示词全文（主控写给执行代理的任务书，原样）

````markdown
# M7BOARD0：主线中间版整板在 KV260 上做三核冒烟（记忆核、规划核终版、搜索核）

你是 FPGA 国赛线主控派的有界工作包。中文报告，effort medium。RUN=<run> 。产物放 RUN/M7BOARD0/。
今晚 18:00 提交。目标是让记忆核、规划核终版、搜索核这三个核在板上真跑一次，并且输出和宿主黄金一致。这样提交文档就能把它们写成“已在板上运行”。测试从简：每个核只跑一组关键向量，比一次就够。

## 输入
- 比特流：M7PRE0 正在远端构建 m5 的全部核＋GMAP3＋MEMK4＋搜索 c8，有 c1 和 etp 两种策略。看 RUN/OFFLOAD0/REMOTE_BUILDS.txt 里这两条：impl-m6a-mk4-m7pre0-6661def、impl-m6a-mk4-setp-m7pre0-6661def。看 PULLED 行写的是 timing_met 还是 NOT_met。
  - 选过时序的那个；两个都过，选 WNS 大的；都没过，选 WNS 大的，并在 README 里注明。
  - 打包：用 MERGE10/tools 或 E2E0/stage/host/pkg_bit.sh 的做法，对拉回的 bit/hwh 打 app 包。M7PRE0（另一个会话）也可能已经打好，看 RUN/M7PRE0/README.md，有就直接用。
- 驱动和测试向量：
  - 记忆核：perception/fpga_replay/memory_pl.py 的 MMIOBackend，版式 bram，slot 14：控制口 0xA00E_0000，BRAM 口 0xA00E_8000。录制调用和宿主模拟校验见 tools/memk0/mmio_sim_check.py 和 RUN/MERGE10/README.md 第 4 节。板上只重放前约 500 次调用，和 C 模型逐位比较。
  - 规划核终版：驱动在 hw/planner/drv_gmap3/（发布分支），测试向量和黄金在 RUN/GMAP3/（evidence、results），寄存器头在 hw/m7/pl/sw/gmap3_v9/。跑 1–2 组向量，比较势场或路径输出。
  - 搜索核：hw/srch 和 tools/srch，向量见 RUN/SRCHK5/README。跑 1 条记录，和 c8 的 C 模型比较。
  - 地址以这次构建的 address_map.csv 为准。
- 代码基线：发布分支 release/member-a/nat-submit-20261104（worktree <worktree>，rc2 = 8bf7399）。可以另开 worktree：<worktree>，分支 feat/member-a/nat-m7board0-20261104。

## 两阶段
**第 1 阶段（现在开始，只在宿主上做）：**准备好一个上板套件。
- 所有 C 程序都在宿主上交叉编译成 aarch64，参考 E2E0/stage/host/xbuild_dpu_rx.sh 和 POSTPL0 的做法。**KV260 上禁止原生编译**，今天上午因为这个整机卡死过。
- Python 脚本可以直接在板上跑。
- 写一个板上脚本 board_smoke.sh，一次跑完三个核，并输出 PASS/FAIL 表。
- 在宿主上用 mock 或 sim 把这个脚本跑通一次。

**第 2 阶段（上板）：**
- 上板条件：GOLIVE0 已经释放 kv260 锁（`bash <project>/tools/lock.sh status kv260`），并且第 1 阶段已经完成。GOLIVE0 预计 14:45 释放。
- 上板之前，先查一下 RUN/GOLIVE0/README.md 是否写了交回。
- 流程：
  1. 用 `tools/lock.sh acquire kv260 "M7BOARD0 三核冒烟" -u member-a` 取锁。
  2. 停 gdm。
  3. 加载新 app，读回 BUILD_ID 确认。
  4. 跑 board_smoke.sh。
  5. **跑完必须把 m5 app（m7_pl_m5_166）加载回去**，读回 BUILD_ID 0x0f35f07f，再恢复 gdm，释放锁。15:30 以后队友要用 m5 拍视频，所以最晚 15:40 必须恢复 m5 并释放锁。
- 卡死时：立即停，写进 README，不重试，并告诉主控。
- 可以顺便做的一项（不超过 2 分钟）：用 INA260 测一次加载新 app 后的空闲功耗。

## 产出
RUN/M7BOARD0/README.md，写明：
- 用了哪个构建，以及它的 WNS；
- 三个核各自的 PASS/FAIL、比较了多少字节或多少项、单次调用时延；
- 板子的最终状态：m5 已恢复、锁已释放。
改动只推这个分支和 pre/post 标签，用显式 ref。

## 规矩
- 车绝对不动，只在台架上做。sudo 只走 stdin：`sudo -S -p "" ... < <secret-file>
- 进程只按 pid 停；禁止 git push --tags；不合 main；不读 member-a_archive/。
- 小构建槽只用 nat_build_slot_{1,2,3,5}。
- 输出节流。上下文到约 250k 时写 HANDOVER 后结束。
- 期限：15:45 交回。
````

---

## 二、执行过程（按时间顺序：模型文字回答原文 ＋ 工具调用一行摘要）

图例：▶ 模型对外说的话（原文）；· 工具调用（只留工具名和命令/文件名）；⚠ 工具返回报错（只留首行）。

· [1] `Bash` date; cd <run>; ls; grep -n "m7pre0" OFFLOAD0/REMOTE_BUILDS.txt | tail -20; ls M7PRE0 2>/dev/null; bash <path>…
· [2] `Bash` cd <run>; ls M7BOARD0; cat M7PRE0/brief.md | head -60; ls M7PRE0/tools M7PRE0/logs; cat MERGE10/README.md | sed -n '1,2…
· [3] `Bash` cd <run>; sed -n 82,125p MERGE10/README.md; ls MERGE10/tools; ls E2E0/stage/host; tail -5 OFFLOAD0/REMOTE_BUILDS.txt
· [4] `Bash` cd <worktree>; git log --oneline -3; find . -name "address_map*.csv" | head; ls hw/m7/pl/sw/ ; ls hw/planner/ 2>/dev/null; ls hw/src…
· [5] `Bash` cd <worktree>; git log --oneline -2; ls hw/planner/drv_gmap3 hw/m7/pl/sw/gmap3_v9 hw/srch tools/srch tools/memk0 2>&1 | head -60; ca…

▶ 正在整理三核的驱动和测试向量，准备上板套件（第 1 阶段，只在宿主上做）。

· [6] `Bash` cd <worktree>/hw/planner/drv_gmap3; cat SNAPSHOT.md | head -50; cat Makefile; wc -l *.cpp *.hpp; ls build
· [7] `Bash` cd <worktree>/hw/planner/drv_gmap3; sed -n 1,124p planner_core.hpp; grep -n "" ../../m7/pl/sw/pl_io.hpp 2>/dev/null | head -60; ls .…
· [8] `Bash` cd <run>; grep -n "^#" GMAP3/README.md | head -30; ls GMAP3 GMAP3/evidence GMAP3/evidence/vectors GMAP3/results 2>&1 | …
· [9] `Bash` cd <run>; head -60 GMAP3/HANDOVER.md; ls GMAP3/evidence/drv GMAP3/tools; grep -n "板\|board\|m7pl_test\|上板" DRV5/README.…
· [10] `Bash` cd <worktree>/hw/planner/drv_gmap3; sed -n 1,292p test_planner_drv.cpp | cut -c1-220
· [11] `Bash` cd <worktree>/hw; sed -n 60,140p m7/pl/sw/pl_io.hpp; grep -n "planner_drv_open\b\|extern \"C\"\|void\* planner_drv_open" -A25 planne…

▶ 在看搜索核与记忆核的驱动和向量，然后写三个板上程序。

· [12] `Bash` cd <run>; wc -l SRCHK5/README.md; grep -n "^#\|向量\|vector\|a53\|板\|C 模型\|golden\|黄金" SRCHK5/README.md | head -40; ls SR…
· [13] `Bash` cd <run>; sed -n 74,91p SRCHK5/README.md; ls SRCHK5/build SRCHK5/build/c8i 2>&1|head; grep -rn "srch_ps\|SRCH_SW\|V1\|V…
· [14] `Bash` cd <run>; sed -n 1,40p SRCHK5/HANDOVER.md; cd <worktree>; head -50 tools…
· [15] `Bash` cd <worktree>; cat hw/srch/srch_tb.cpp | cut -c1-200; grep -n "srch_top\|void srch\|#pragma HLS INTERFACE" hw/srch/srch_core.cpp | h…
· [16] `Bash` cd <worktree>; sed -n 901,918p hw/srch/srch_core.cpp; grep -rn "srch" <repo>/runs/nationals-prelim-20261…
· [17] `Bash` cd <worktree>; grep -n "AD \|typedef.*AD\|prm\b\|base" hw/srch/srch_core.cpp | head -30; grep -n "srch" <project>/wo…

▶ 正在确认搜索核的地址口径（核用 32 位绝对地址，录制镜像是相对 0 的，上板要重定位）。

· [18] `Bash` cd <worktree>; grep -n "base\|P_RES\|A_RES\|A_JOB\|def run\|def _call\|struct.pack\|prm\[" tools/srch/srch_ps.py | head -40; sed -n …
· [19] `Bash` cd <worktree>; sed -n 50,135p tools/srch/srch_ps.py
· [20] `Bash` cd <worktree>; grep -n "class \|def \|LAYOUT\|0x8000\|0x9000\|import" perception/fpga_replay/memory_pl.py | head -60; wc -l perceptio…
· [21] `Bash` cd <worktree>; sed -n 118,188p perception/fpga_replay/memory_pl.py; cat tools/memk0/mmio_sim_check.py | sed -n 1,131p | cut -c1-200
· [22] `Bash` cd <worktree>; grep -n "^import\|^from" perception/fpga_replay/core.py perception/fpga_replay/memory_gates.py; ls perception/__init__.…
· [23] `Bash` R=<run>; grep -rn "xmutil\|BUILD_ID\|gdm\|devmem\|sudo -S" $R/E2E0/stage/*.sh $R/E2E0/stage/host/e2e0.sh 2>/dev/null | …
· [24] `Bash` R=<run>; sed -n 1,80p $R/E2E0/stage/host/e2e0.sh | cut -c1-250; ls $R/E2E0/stage/board
· [25] `Bash` R=<run>; sed -n 1,60p $R/E2E0/stage/board/e2e0_board.sh | cut -c1-250
· [26] `Bash` R=<run>; ls $R/MERGE7/builds/pkg-m5_166; sed -n 1,40p $R/MERGE7/builds/pkg-m5_166/board_bringup.sh | cut -c1-200; file …
· [27] `Bash` R=<run>; grep -n "^step\|xmutil\|lib/firmware\|^main\|for s in\|case" $R/MERGE7/builds/pkg-m5_166/board_bringup.sh | he…
· [28] `Bash` date +%T; R=<run>; tail -3 $R/OFFLOAD0/REMOTE_BUILDS.txt | cut -c1-200; tail -5 $R/M7PRE0/logs/launch.out 2>/dev/null |…
· [29] `Bash` cd <repo> && git worktree add -q -b feat/member-a/nat-m7board0-20261104 <worktree> 8bf7399 && cd <path>…

▶ 先写三个板上程序（规划核、搜索核用 C++ 交叉编译，记忆核用 Python），并核对向量格式。

· [30] `Bash` cd /tmp; python3 - <<'EOF' ⏎ import numpy as np, struct ⏎ r=np.memmap('<agent-run>/work/rec_old/d3s1.bin',…
· [31] `Bash` cd <worktree>; grep -n "start_and_wait\|bool .*alloc\|open_hw" -A12 hw/m7/pl/sw/pl_io.cpp | grep -v "^\s*$" | sed -n 1,80p | cut -…
· [32] `Bash` mkdir -p <worktree>/tools/m7board0 && cd $_ && cat > srch_board.cpp <<'EOF' ⏎ // M7BOARD0 (2026-11-04): board smoke of the probabi…
· [33] `Bash` cd <worktree>; grep -n "#include\|#define GMAP_OFF\|^[a-z].*gmap_run" tools/gmap/gmap_model.c | head -20; ls tools/gmap/; ls hw/pl…
· [34] `Bash` cd <worktree>; grep -n "define" tools/gmap/gmap_model.h | head -30; cd hw/planner/drv_gmap3 && (make check > /tmp/m7b0_plncheck.lo…
· [35] `Bash` cd <worktree>/tools/m7board0 && cat > pln_board.cpp <<'EOF' ⏎ // M7BOARD0 (2026-11-04): board smoke of the GMAP3 planner core (v9,…

▶ 规划核、搜索核两个 C++ 板上程序已写好，现在写 Makefile 并在宿主上编译（aarch64 静态 + 宿主 mock 版）。

· [36] `Bash` cat > Makefile <<'EOF' ⏎ # M7BOARD0: host cross build of the board smoke binaries (NO native build on the KV260), plus host mock builds. ⏎ #   make            -> build/a64/{pln_bo…

▶ Now run the mocks on host.

· [37] `Bash` cd <worktree>/tools/m7board0; R=<run>; FZ=<project>/sh…

▶ 两个 mock 都 PASS。现在写记忆核的板上脚本。

· [38] `Bash` cd <worktree>; sed -n 24,40p perception/fpga_replay/memory_pl.py; grep -n "^[A-Z_]* = \|^_\w* = " perception/fpga_replay/memory_pl.p…
· [39] `Bash` cd <worktree>/tools/m7board0 && cat > memk_board.py <<'EOF' ⏎ #!/usr/bin/env python3 ⏎ """M7BOARD0 (2026-11-04): board smoke of th…
· [40] `Bash` date +%T; R=<run>; grep -n "m7pre0" $R/OFFLOAD0/REMOTE_BUILDS.txt | grep -v LAUNCH | cut -c1-250; for b in impl-m6a-mk4…
· [41] `Bash` R=<run>; head -20 $R/MERGE7/builds/pkg-m5_166/address_map.csv; ls $R/OFFLOAD0/builds/ | tail; ls $R/MERGE10/builds 2>/d…
· [42] `Bash` R=<run>; tail -3 $R/OFFLOAD0/builds/impl-m6a-mk4-m7pre0-6661def.poll.log $R/OFFLOAD0/builds/impl-m6a-mk4-setp-m7pre0-66…

▶ 构建还在布线阶段（c1 已进入备用布线指令）。先写 board_smoke.sh。

· [43] `Bash` cd <worktree>/tools/m7board0 && cat > board_smoke.sh <<'EOF' ⏎ #!/bin/bash ⏎ # M7BOARD0 (2026-11-04): three-core smoke on the KV26…
· [44] `Bash` cat > stage.sh <<'EOF' ⏎ #!/bin/bash ⏎ # M7BOARD0 host side: stage the board kit.   stage.sh <kit_dir> <app_pkg_dir> ⏎ # kit = bin (aarch64 static) + bin_host (mock) + repo (git a…
· [45] `Bash` W=<agent-run>/work; mkdir -p $W/fakepkg; printf 'master_space,segment,offset,range\nData,SEG_planner_Re…
· [46] `Bash` cd <worktree>; sed -i 's|^K=\${K:-\$(cd "\$(dirname "\$0")/.." \&\& pwd)}|K=${K:-$(cd "$(dirname "$0")" \&\& pwd)}|; s|(default: t…

▶ 宿主 mock 三核全 PASS。再做一次阳性对照：每个黄金各改 1 字节，三项都必须报 FAIL。

· [47] `Bash` W=<agent-run>/work; rm -rf $W/kit_neg; cp -r $W/kit_mock $W/kit_neg; rm -rf $W/kit_neg/out; cd $W/kit_n…
  ⚠ [47] 报错：Permission to use Bash with command W=<agent-run>/work; rm -rf $W/kit_neg; cp -r $W…
· [48] `Bash` W=<agent-run>/work; mkdir $W/kit_neg2 && cp -r $W/kit_mock/bin_host $W/kit_mock/repo $W/kit_mock/vec $W…
· [49] `Bash` W=<agent-run>/work; MOCK=1 MEMK_SIM_LIB=<repo>/runs/nationals-prelim-202611…

▶ 三个阳性对照都抓到。再用 qemu 确认 aarch64 二进制能启动。

· [50] `Bash` W=<agent-run>/work; V=$W/kit_mock/vec; qemu-aarch64-static $W/kit_mock/bin/srch_board $V/trace_v1.bin 1…
· [51] `Bash` R=<run>; sed -n 1,25p $R/BOARD5/stage/scripts/pwr_sample.py | cut -c1-200; sed -n 57,92p $R/MERGE7/builds/pkg-m5_166/bo…
· [52] `Bash` cd <worktree>/tools/m7board0 && cat > board_run.sh <<'EOF' ⏎ #!/bin/bash ⏎ # M7BOARD0 board-side phases (root, from the kit dir; s…

▶ 第 1 阶段套件已完成并提交。现在后台等两个远端构建拉回。

· [53] `Bash` R=<run>; until [ $(grep -c "PULLED impl-m6a-mk4-.*m7pre0-6661def" $R/OFFLOAD0/REMOTE_BUILDS.txt) -ge 1 ]; do sleep 60; …
· [54] `Bash` R=<run>; ls $R/GOLIVE0; head -30 $R/GOLIVE0/README.md 2>/dev/null | cut -c1-200
· [55] `Bash` bash <project>/tools/lock.sh status kv260; R=<run>; cat $R/GOLIVE0/lock_release.txt | tail -3; grep -n …

▶ GOLIVE0 已交回（锁已释放、m5 仍加载、gdm 已恢复）。等构建拉回期间先起草 README。

· [56] `Bash` sleep 240; date +%T; R=<run>; for b in impl-m6a-mk4-m7pre0-6661def impl-m6a-mk4-setp-m7pre0-6661def; do bash $R/OFFLOAD…
  ⚠ [56] 报错：<tool_use_error>Blocked: sleep 240 followed by: date +%T R=<run> for b in impl-m6a-mk4-m7pre0-6661d…
· [57] `Bash` cd <worktree>; sed -n 1,30p hw/m7/pl/tools/package_app.sh | grep "^#" | cut -c1-200; cat <repo>/runs/nat…
· [58] `Bash` R=<run>; grep -n "package_app" $R/MERGE10/README.md $R/OFFLOAD0/README.md | cut -c1-300 | head
· [59] `Bash` cd <worktree>; grep -n "IP_REPO\|ip_repo\|\$2" hw/m7/pl/tools/package_app.sh | head -12 | cut -c1-200
· [60] `Bash` cd <worktree>; sed -n 30,80p hw/m7/pl/tools/package_app.sh | grep -v "^#" | head -40 | cut -c1-200

▶ 等待构建完成通知中。

· [61] `Bash` cat <tmp> date +%T
· [62] `Bash` R=<run>; t0=$(date +%s); until grep -q "PULLED impl-m6a-mk4-.*m7pre0-6661def" $R/OFFLOAD0/REMOTE_BUILDS.txt || [ $(( $(…

▶ c1 已拉回。看它的时序；etp 版也快写完 routed.dcp。

· [63] `Bash` B=<run>/OFFLOAD0/builds/impl-m6a-mk4-m7pre0-6661def; ls $B | head -30; grep -m3 -A3 "WNS(ns)" $B/timing_route.rpt 2>/de…

▶ c1 时序收口：WNS +0.022 ns、WHS +0.007 ns。先给 c1 打包（同时等 etp）。

· [64] `Bash` R=<run>; ls $R/M7PRE0 $R/M7PRE0/logs; ls $R/OFFLOAD0/builds/impl-m6a-mk4-m7pre0-6661def/post; ps -eo pid,etime,args | g…
· [65] `Bash` B=<run>/OFFLOAD0/builds/impl-m6a-mk4-m7pre0-6661def; grep -E "^Data,SEG_(planner|srch|memk|bc_memk|sysinfo)" $B/address…
· [66] `Bash` R=<run>; grep -n -i "package\|打包\|CLOCKS" $R/OFFLOAD0/README.md | head -10 | cut -c1-250; ls $R/OFFLOAD0
· [67] `Bash` B=<run>/OFFLOAD0/builds/impl-m6a-mk4-m7pre0-6661def; grep -i "prep" $B/impl-m6a-mk4-m7pre0-6661def.env | head -5 | cut …
· [68] `Bash` R=<run>; grep -n "PREP_IP\|prep_ip\|94b41446" $R/OFFLOAD0/tools/remote_build.sh $R/MERGE10/tools/launch_whole_m7.sh 2>/…

▶ Now make a local package copy with CLOCKS paths rewritten to local exports, then package.

· [69] `Bash` R=<run>; M=$R/M7BOARD0; B=$R/OFFLOAD0/builds/impl-m6a-mk4-m7pre0-6661def; L=$M/pkgsrc/impl-m6a-mk4-m7pre0-6661def; mkdi…
· [70] `Bash` cd <repo> && git worktree add -q --detach <worktree> 6661def && R=<repo>…
· [71] `Bash` tail -15 <run>/M7BOARD0/pkg-m7pre0c1.log | cut -c1-250
· [72] `Bash` cd <worktree>; grep -n "LEDGER" -B2 -A8 hw/m7/pl/tools/package_app.sh | cut -c1-220
· [73] `Bash` R=<run>; M=$R/M7BOARD0; T=<worktree>; L=$M/pkgsrc/impl-m6a-mk4-m7pr…
· [74] `Bash` M=<run>/M7BOARD0; grep -n "BUILD_ID\|driver make check\|RESULT\|PASS\|FAIL" $M/pkg-m7pre0c1/SELFCHECK.txt | cut -c1-200…

▶ c1 打包完成（BUILD_ID 应为 0x06661def，自检 CHECK PASS）。发现新包的 board_bringup 不含 m5 的 LPD 复位检查，正在核对这一点，避免规划核/搜索核访存挂死。

· [75] `Bash` sed -n 1,40p <run>/MERGE7/builds/pkg-m5_166/m5_lpd_afi.py | cut -c1-200
· [76] `Bash` cd <worktree>; python3 - <<'EOF' ⏎ p='tools/m7board0/board_run.sh' ⏎ s=open(p).read() ⏎ old='''  W=$(want_of "$K/pkg"); G=$(bid "$…
· [77] `Bash` R=<run>; t0=$(date +%s); until grep -q "PULLED impl-m6a-mk4-setp-m7pre0-6661def" $R/OFFLOAD0/REMOTE_BUILDS.txt || [ $((…
· [78] `Bash` R=<run>; for b in impl-m6a-mk4-m7pre0-6661def impl-m6a-mk4-setp-m7pre0-6661def; do echo "$b: $(grep -m1 -A3 'WNS(ns)' $…
· [79] `Bash` R=<run>; for b in impl-m6a-mk4-m7pre0-6661def impl-m6a-mk4-setp-m7pre0-6661def; do echo "$b"; grep -m1 -A2 'WNS(ns)' $R…
  ⚠ [79] 报错：Exit code 1 ⏎ impl-m6a-mk4-m7pre0-6661def ⏎ WNS 0.022 TNS 0.000 WHS 0.007 ⏎ 0 ⏎ impl-m6a-mk4-setp-m7pre0-6661def ⏎ WNS 0.020 TNS 0.000 WHS 0.010 ⏎ 0

▶ 两个都收口：c1 WNS +0.022 / etp +0.020，按规则选 WNS 大的 c1（已打包）。用真包重新打套件并在宿主 mock 跑一遍。

· [80] `Bash` R=<run>; M=$R/M7BOARD0; cd <worktree>; bash tools/m7board0/stage.sh $M…

▶ 第 1 阶段完成。GOLIVE0 已交回、锁空闲，开始第 2 阶段：取锁。

· [81] `Bash` M=<run>/M7BOARD0; bash <project>/tools/lock.sh status kv260; bash <project>/tools/lock.…
· [82] `Bash` M=<run>/M7BOARD0; timeout 30 ssh -o ConnectTimeout=20 <host-alias> 'uptime; systemctl is-active gdm; xmutil listapps 2>&1 | gr…
· [83] `Bash` M=<run>/M7BOARD0; T=<worktree>/tools/m7board0; time bash $T/host.sh pu…
· [84] `Bash` timeout 30 ssh <host-alias> "cd <board-home>/m7board0_member-a && (cd vec && sha256sum -c --quiet SHA256SUMS) && (cd pkg && sha256sum -c --quiet SHA256SUMS) && (cd pkg_m5 && sha256sum -c --qu…

▶ Kit on board, all sha OK. Load the new app (stops gdm, loads, LPD check, BUILD_ID).

· [85] `Bash` T=<worktree>/tools/m7board0; date +%T; bash $T/host.sh run load_new m7_pl_m7pre0c1 && bash $T/host.sh wait load_new 240; date +%T
· [86] `Bash` timeout 30 ssh <host-alias> "cd <board-home>/m7board0_member-a/out && grep -E 'PASS|FAIL|WARN|gdm|BUILD_ID|RST_LPD|bit 19|LPD' load_new.log | cut -c1-200 | head -20"

▶ 加载成功（第 1 次 xmutil 报已知的 "load Error: -1"，第 2 次成功），BUILD_ID 读回 0x06661def。看 LPD 检查结果，然后跑冒烟。

· [87] `Bash` timeout 30 ssh <host-alias> "cat <board-home>/m7board0_member-a/out/lpd_afi_new.json | head -c 900; echo; grep -i 'm5_lpd\|lpd\|afi' <board-home>/m7board0_member-a/out/load_new.log | grep -v dmesg…

▶ LPD AFI 不在复位、无告警。跑三核冒烟。

· [88] `Bash` T=<worktree>/tools/m7board0; date +%T; bash $T/host.sh run smoke && bash $T/host.sh wait smoke 420; date +%T; timeout 30 ssh <host-alias>…

▶ 记忆核、搜索核板上 PASS；规划核有 1 项失败，看细节。

· [89] `Bash` timeout 30 ssh <host-alias> "cat <board-home>/m7board0_member-a/out/pln.txt" | cut -c1-250
· [90] `Bash` cd <worktree>; grep -n "bool Core::gmap" -A40 hw/planner/drv_gmap3/planner_core.cpp | cut -c1-200
· [91] `Bash` cd <worktree>; grep -n "void to_device\|void from_device\|cvac\|civac\|DMA_BUF_IOCTL_SYNC" -A6 hw/m7/pl/sw/pl_io.cpp | head -40 | …
· [92] `Bash` cd <worktree>/tools/m7board0; python3 - <<'EOF' ⏎ p='pln_board.cpp' ⏎ s=open(p).read() ⏎ old='''  std::memcpy(area, ref.data(), GM…
· [93] `Bash` K=<board-home>/m7board0_member-a; timeout 150 ssh <host-alias> "sudo -S -p '' bash -c 'cd $K && V=vec; PLN_NO_INIT_FLUSH=1 timeout 60 bin/pln_board_fix \$V/input.bin \$V/expected.bin \$V/path…
· [94] `Bash` cd <worktree>/tools/m7board0; python3 - <<'EOF' ⏎ p='pln_board.cpp' ⏎ s=open(p).read() ⏎ old='''  if (!getenv("PLN_NO_INIT_FLUSH")…

▶ 规划核在板上三部分全部逐位一致（PASS）。顺手测一次新 app 的空闲功耗（30 秒），然后恢复 m5。

· [95] `Bash` T=<worktree>/tools/m7board0; date +%T; bash $T/host.sh run pwr && bash $T/host.sh wait pwr 90; timeout 30 ssh <host-alias> "uptime; ps -e…

▶ Idle 3.953 W (300 samples, gdm stopped). Now restore m5.

· [96] `Bash` T=<worktree>/tools/m7board0; date +%T; bash $T/host.sh run restore_m5 && bash $T/host.sh wait restore_m5 240; timeout 30 ssh <host-alias>…

▶ m5 已恢复（BUILD_ID 0x0f35f07f，gdm active）。独立再读一次、拉回板上输出、释放锁。

· [97] `Bash` M=<run>/M7BOARD0; T=<worktree>/tools/m7board0; timeout 40 ssh <host-alias> "s…
· [98] `Bash` bash <project>/tools/lock.sh 2>&1 | head -15; grep -n "release" <project>/tools/lock.sh | head -5
· [99] `Bash` M=<run>/M7BOARD0; bash <project>/tools/lock.sh release kv260 --token <redacted> -…

▶ 锁已释放（14:02）。提交测试台修正、打 post 标签、按显式 ref 推送，然后写 README。

· [100] `Bash` cd <worktree>; git add tools/m7board0/pln_board.cpp; git commit -qm "M7BOARD0: pln_board map-op test cleans the whole area once af…
· [101] `Bash` M=<run>/M7BOARD0; ls $M/board_out; grep -E "^MEMK base" $M/board_out/memk.txt | cut -c1-200; grep -E "^SRCH bus|SRCH re…
· [102] `Bash` B=<run>/M7BOARD0/pkgsrc/impl-m6a-mk4-m7pre0-6661def; awk '/^\| Intra Clock Table/{f=1;next} /^\| Inter Clock Table/{f=0…

▶ 全部完成，写 README。

· [103] `Write` <run>/M7BOARD0/README.md（写入 7489 字符）
· [104] `Bash` sha256sum <worktree>/tools/m7board0/build/a64/pln_board | cut -c1-16; timeout 20 ssh <host-alias> "sha256sum <board-home>/m7board0_member-a/bi…
· [105] `Edit` <run>/M7BOARD0/README.md（替换 169→155 字符）

▶ （最后一条文字回答就是下方第三节的最终回答，这里不重复。）


---

## 三、最终回答（执行代理交回主控的报告，原文）

## M7BOARD0 交回：三个核在 KV260 上都跑通了，输出和宿主黄金逐位一致

板子已经恢复成 m5：BUILD_ID 读回 0x0f35f07f，gdm 是 active，我这边没有进程留在板上。kv260 锁 14:02:46 用原令牌释放，查锁状态是空的。

**用的构建是 c1 版 impl-m6a-mk4-m7pre0-6661def。** 两个构建时序都收口了，按规则选 WNS 大的：
- c1：WNS +0.022 ns，WHS +0.007 ns，最差的时钟是 clk_dsp；
- etp：WNS +0.020 ns，WHS +0.010 ns。

c1 的 PULLED 行里没写 timing_met，WNS 是我从 timing_route.rpt 读的。M7PRE0 在我要用包时还没打好，我在独立的 detached worktree 里自己打了包，app 名 `m7_pl_m7pre0c1`。自检 CHECK PASS，上板后 BUILD_ID 读回 0x06661def，确认板上跑的就是这次构建。四个基址也按这次构建的 address_map.csv 核对过，全部对上。

| 核 | 结果 | 比较了什么 | 单次时延 |
|---|---|---|---|
| 记忆核 MEMK4（bram 版式） | **PASS** | 录制的前 500 次调用，共 512,000 字，0 次不一致 | Python 驱动整次调用中位数 2.50 ms（主要是逐字读写的开销，核本身没单独测） |
| 规划核 GMAP3 v9 | **PASS**（第 3 次运行） | 两组势场 186,904 字节；同两组带取路径 187,732 字节；地图操作向量 vecA 共 73 个操作，每个操作后整块地图区 1,173,088 字节和 C 模型比。全部 0 差 | 势场 2.28 ms（80×80）/ 6.08 ms（200×200）；地图操作平均 0.88 ms，最长 7.42 ms |
| 搜索核 SRCHK5 c8 | **PASS** | trace_v1 全部 32 条记录（任务只要求 1 条），每条比整块 0x60000 字节，共 12,582,912 字节，0 条不一致 | 观测平均 245 µs，选点 261 µs，最长的初始化 8.38 ms |

**顺手测的功耗：** 新 app 加载、冒烟跑完后空闲 3.953 W（INA260，30 秒，gdm 已停）。这次没有在同样条件下测 m5，所以没有对照。

**需要你知道的两件事：**
1. **规划核第一次跑时地图操作（C 部分）失败了，原因在我的测试台，不是核或驱动。** 测试开头把整块地图区直接拷进缓存、没有刷到 DDR，而驱动只刷新、作废产品里 PS 真正会读写的那几块（不碰 T 图，也就是"扫过时间图"）。
   - 第 1 次：差异全在全局图，是开头拷贝留下的脏缓存行被写回、盖掉了核的输出。
   - 第 2 次：开头先整区刷一次，前 30 个操作全对，剩下的差异只在 T 图，而且两次统计的数还不一样，是 CPU 缓存里的 T 图旧数据。
   - 第 3 次：每个操作后再由测试整区作废缓存，73 个全对，T 图也真的和 C 模型比到了。

   驱动和核都没改。三次都只重跑规划核：它不跨调用保存状态，记忆核和搜索核会，不能重跑。对驱动使用方的提醒：产品里 PS 如果往全局图或粗图写东西，必须先自己刷缓存。目前只有代码注释说产品只写参数块、帧和 aux，没有检查兜底。
2. **6661def 这版的上板脚本少了 LPD 总线复位检查**（m5 包里有，这版没有），而规划核和搜索核都挂在这条总线上。所以我加载后手动跑了 m5 包里的 `m5_lpd_afi.py`：复位位本来就是 0，没改，也没有告警。以后终版包里如果有挂在这条总线上的核，建议把这一步并进包里，请主控定。

**做了什么：**
- 第 1 阶段：三个板上程序加一键脚本 `board_smoke.sh`。C 程序全部在宿主上交叉编译成 aarch64 静态二进制，板上没有任何编译。宿主 mock 三项都 PASS；三份黄金各改 1 字节的对照，三项都报 FAIL。
- 第 2 阶段 13:55–14:02：取锁 → 停 gdm → 加载。第 1 次加载报已知的 "load Error: -1"，脚本自动重试，第 2 次成功。之后跑冒烟、测功耗、恢复 m5、恢复 gdm、释放锁。全程没有卡死，车没动。

**产物与代码：**
- 报告 `RUN/M7BOARD0/README.md`，里面有完整过程、竞争解释和反向检验；
- 板上原始输出在 `board_out/`，包在 `pkg-m7pre0c1/`，宿主 mock 输出在 `hostmock_out/`；
- 分支 `feat/member-a/nat-m7board0-20261104`，标签 pre = 8bf7399、post = 3325c75，都已用显式 ref 推送。

**未完成和遗留：**
- c1 构建的功耗和 CDC 守卫我没做，交给 M7PRE0（13:47 时它的后处理已经在跑）。
- 板上留了套件目录 `<board-home>/m7board0_member-a`（约 63 MB）和已安装但没在用的 app `/lib/firmware/xilinx/m7_pl_m7pre0c1`，都不影响 m5，我没删。
- 宿主上留了一个打包用的 worktree `member-a-wt/nat-m7board0-pkg-6661def`。
- 套件里 `board_smoke.sh` 调用的还是修正前的规划核程序。复跑前要在 post 标签上重新打套件，而且必须是重新加载 bitstream 之后第一次用这些核。
