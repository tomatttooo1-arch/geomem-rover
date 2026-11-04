# MEMK4：记忆核改成共用运算单元＋微程序顺序器（2026-11-04）

你是 FPGA 国赛线执行代理，交回后主控验收。调用全局 skill `fpga-hls`（必要时 `vivado-checks`）。输出节流（tail/grep，长内容落文件）。上下文到约 200k 就停手写交接（HANDOVER），不要硬撑。

## 背景（先读）
- RUN=<run>
- 上一棒：RUN/MEMK3/README.md（尤其 §1 资源对照、§2 超支原因、§3 路线、§6 固定常数、§7 拍数）、RUN/MEMK3/HANDOVER.md；设计：RUN/MEMK0/DESIGN.md（§5 手算、§10 判据）。
- 写码规矩：RUN/HWDESIGN_RULES_20261104.md 末节五条（小映射用常量 ROM；同一存储单读调用点；输出多路选择用相位计数器；最小位宽；csynth 后逐项对手算，超 1.5 倍先改）。
- 现状：MEMK3 的 export（sha 90a86fc9）功能已验（判据①、531 次联仿逐字节同），OOC 9,180 LUT，预算 2,500。超支是结构性的：HLS 给每个 C 运算单独造硬件（186 加法器、179 比较器、154 选择器），AXI-Lite 数组外壳 1,418、LUTRAM 槽表写口 1,427。

## 主控裁定（已定，不再讨论）
- 走路线 A：一个共用运算单元（加减、比较、乘，宽运算放 DSP）＋小寄存器堆（LUTRAM 或 BRAM）＋微程序 ROM（常量数组即可，不要求独立汇编器——用 Python 生成器从一份可读的操作表生成 ROM 初值，并把同一操作表在 Python 里解释执行作为参考，与 C 模型逐位对照）＋顺序器。同时并入路线 C 里便宜的部分：cmd/res 从 s_axilite 数组改到核内 BRAM（或走已有 m_axi 口），只留少量控制寄存器；槽表进 BRAM/URAM，单读单写调用点。
- 存储上限：BRAM36 当量 ≤6（现 5）、URAM ≤2；整板 BRAM 是头号风险，不要多占。
- 拍数放宽：典型帧 ≤1 ms（100 MHz）即可，不追设计的 40 µs。
- LUT 门：≤2,500 收；2,500–4,000 交主控；>4,000 继续压。FF、DSP 写明对手算。
- 本核不阻塞 m6b：m6b 先用 MEMK3 的 export；你这版进下一次整板（与 member-b 观测核同批）。所以质量优先，不赶工；但别做低价值测试。

## 步骤
1. 先出架构设计 `RUN/MEMK4/ARCH.md`：数据通路框图（文字即可）、指令格式与位宽、寄存器堆大小、ROM 深度、各引擎（配对门、存在度结算、扫描、去重、除法、开方）映射成多少条微指令与拍数、逐部件手算 LUT/FF/DSP/BRAM。手算合计必须 ≤2,500 再往下写码。
2. 实现（新 worktree：`<worktree>/nat-memk4-20261104`，从 MEMK3 的 post 提交 d3cf7fe 开；顺手 cherry-pick e7e44a1 那个单测文件 tests/test_memk0_err_slot.py；pre/post 标签，只推显式 ref）。
3. 校验（只做关键的）：①csim 用 MEMK3 录下的 14,886 次调用对 C 模型逐位同；②微程序 Python 参考与 C 模型同（保证生成器没错），一个阳性对照（故意改一条微指令必须被抓到）；③判据①按 MEMK3 的跑法重跑（D4 那 1 次落在内点阈值上的边界翻转已获放行，同现象不算失败）；④联仿取 MEMK3 联仿向量的子集逐字节同；⑤OOC 实测。
4. csynth 后逐项对手算，超 1.5 倍的先改再 OOC。
5. 交 `RUN/MEMK4/README.md`：资源对照表（手算/csynth/OOC）、拍数、校验结果、export 路径与 sha、未完成项。

## 规矩
构建槽按 RUN/SLOT_RULES_20261104.md（小槽 1/2/3/5，原子取槽、立即写 owner、用完释放，线程 ≤8，MemAvailable ≥12 GB）。整板槽不碰。不碰板子。只按 pid 停进程，禁 pkill -f。
