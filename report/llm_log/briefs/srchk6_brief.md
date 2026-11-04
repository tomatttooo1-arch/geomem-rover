# SRCHK6：概率搜索核改成微程序引擎（路线 c），目标 ≤4,000 LUT（2026-11-04）

你是 FPGA 国赛线执行代理，交回后主控验收。调用全局 skill `fpga-hls`（必要时 `vivado-checks`）。输出节流（tail/grep，长内容落文件）。上下文约 200k 停手写 HANDOVER。

## 先读
RUN=<run>
- RUN/SRCHK5/HANDOVER.md（下一棒起点：C 解释器先行、必须留硬件的信念引擎与"射线一步/锥体一式"两条专用指令、宽乘法位宽清单与多精度、ROM 与 BRAM 门、联仿 OOC 并行、hier.sh）、ARCH.md §4（路线 c 设计起点）、README.md。
- 参照已成功的记忆核微程序化：RUN/MEMK4/ARCH.md、README.md，源码 <worktree>/nat-memk4-20261104/hw/memk4/（运算单元、顺序器、Python 参考＋ROM 生成器、校验框架尽量复用）。
- 写码规矩：RUN/HWDESIGN_RULES_20261104.md 末节五条。
- 当前最好功能版 c8（9d53224，OOC 9,683 LUT／2.5 RAMB36／13 URAM／79 DSP，联仿 71/71）。worktree <worktree>/nat-srchk5-20261104（HEAD ef2e8b4）。

## 门与上限
LUT ≤4,000 收；4,000–5,000 交主控；>5,000 继续。BRAM36 ≤4、URAM 不增。拍数：观测 ≤2 ms/帧、选点 ≤5 ms/次（100 MHz），初始化 ≤0.5 s。接口（AXI-Lite 寄存器、参数块、结果块、m_axi）尽量不变；要变就写清，主控改块设计与驱动。

## 步骤
1. 先在 SW 测试台验证微程序 C 解释器（与 C 模型逐位同，V1 32 条＋V2 39 条），再上 HLS；ARCH 手算先 ≤4,000 再写 HLS。
2. 新 worktree <worktree>/nat-srchk6-20261104，从 ef2e8b4 开；pre/post 标签，只推显式 ref。
3. csynth 后逐项对手算，超 1.5 倍先改再 OOC。
4. 校验只做关键的：SW 测试台逐字节同、Python/C 参考同＋一个阳性对照（改一条微指令必须被抓到）、联仿 71/71（可先跑子集再全量）、OOC 实测。
5. 过门出 export.zip（记 sha），交 RUN/SRCHK6/README.md。

## 时间
今晚 18:00 的提交快照不等你（快照里搜索核源码用 c8）；你的版本进终版整板。质量优先，但一个阶段一交接，别憋到最后。

## 规矩
