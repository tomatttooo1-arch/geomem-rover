# GMAP3 brief：规划核第二段第二棒——OOC 实测、WINDOW 进核、联仿、驱动、export（FPGA 国赛线主控 11-04 派，worker-opus-high）

你是 FPGA 国赛线（全国 FPGA 创新设计竞赛，KV260 单板寻物机器人）主控派出的执行代理。RUN = <run>。HLS 工作调用 skill fpga-hls，时序看 vivado-checks。

## 先读
`RUN/GMAP2/brief.md`（原任务书，期望结果不变）、`RUN/GMAP2/HANDOVER.md`（做到哪、"没做完"的接法、已知坑）、`RUN/GMAP2/README.md`、`RUN/GMAP0/DESIGN.md`（§6.1 降法）、`RUN/GMAP0/DESIGN_ADD_PLGRID.md`、`RUN/HWDESIGN_RULES_20261104.md`（末尾五条写码规矩）。代码：worktree `<worktree>/nat-gmap2-20261104`，分支 feat/member-a/nat-gmap2-20261104  at 5e464b9（上一棒代理已结束，你直接接着做；先推分支）。

## 主控已定
1. **先跑一次 OOC 拿实测**（MERGE/SCAN/CLEAR 已进核、取路径已降），后面的取舍都按 OOC 实数，不按 csynth 估计。
2. **WINDOW 改 1/65536 格精度后进核**（诊断探针已显示两项 0 差）：测前判据不变（car1015 差异格 ≤0.01%、清除计数差 ≤0.1%），先在 C 模型上按判据跑过，再写 HLS。WINDOW 预算放宽到 **≤400 LUT**（换来与产品逐格一致）。
3. 整核门：**OOC ≤5,500 LUT**（= 4,347 ＋ 740 ＋ 400）、WNS ≥0  at 100 MHz、BRAM36 新增 ≤2、不新增 URAM。超了先照写码规矩改结构（单读点单写点、常量表、相位计数、位宽截到值域；gmap_core csynth 估 3,284 对手算 740，偏离手算 4 倍，先找是哪几处）。**GMAP0 §6.1 降法 1–3（把外接框、清图、粗图重建交给 PS）不准用**——都是把逐格计算退回 PS；降法 4（复用原 load/store 级）可以用，条件是 gmap_op=0 时原生 tb 两个 sha（0036fda9…、86b2d252…）不变。5,500–6,000 交回主控；>6,000 继续改。
4. 联仿两组（各约 1 小时）放在结构定型之后跑一次，别每改一次跑一次。

## 要做
OOC 实测 → WINDOW（C 模型过判据 → HLS）→ 结构压缩到门内 → 联仿两组逐位（gmap_op 0–4 一起回归，含合成向量 1）→ 规划驱动（DRV5 分支 d2c5086 的 `hw/planner/drv`）补 0x90 与 gmap_op 寄存器和参数块，宿主与 qemu 自检 → export.zip 与 sha256 写进 `RUN/GMAP2/README.md` → post 标签、只推显式 ref。

## 纪律
构建槽只用 `<locks>/nat_build_slot_{1,2,3}`：**一条 mkdir 原子取槽并立刻写 owner**，用完删 owner 再 rmdir；开前 MemAvailable ≥12 GB、线程 ≤8；`nat_build_whole` 不碰。不 `--tags`、不合 main。禁 `pkill -f`。不动板子。输出节流。长任务用前台阻塞循环等，不要后台起了就结束回合。约 20 万 token 写 `RUN/GMAP3/HANDOVER.md` 收尾。最后回答 10 行内中文摘要。
