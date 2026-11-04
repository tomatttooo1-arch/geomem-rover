# SRCHK6 第二棒：搜索核微码版压 LUT、修 BRAM、联仿、OOC

你是 FPGA 国赛线主控派的有界工作包，接上一棒继续。中文写报告，effort medium。
RUN=<run> 。

## 先读
1. RUN/SRCHK6/brief.md：原任务书，门和规矩以它为准。
2. RUN/SRCHK6/HANDOVER.md：上一棒交接。§4 是逐项压 LUT 的方案。

## 现状
- worktree：<worktree>/nat-srchk6-20261104，分支 feat/member-a/nat-srchk6-20261104，当前 2ecf92c。
- m1 的功能已经和 c8 逐字节一致：V1 32/32、V2 39/39，阳性对照有效。
- m1 csynth：LUT 10,485，手算约 4,000。BRAM 24 RAMB18，门是 ≤4 RAMB36 当量。其中 16 个 RAMB18 是 m_axi 适配器带来的，原因可能是加了 `-m_axi_conservative_mode=false`，并把突发上限改成了 2。

## 要做，按顺序
1. 先把 m_axi 的写法退回 c8，看 BRAM 是否回到门内。
2. 按交接 §4 逐项压 LUT。每改一轮跑一次 csynth，测试台必须保持逐字节一致，记一行 LUT 分项。
3. LUT 降到 ≤6,000（csynth）之后，跑联仿：先用 SRCH_TB_MAXREC=3 冒烟，再跑全量 71/71。OOC 和联仿并行。
4. 把 OBSERVE、SELECT、INIT 的实测拍数换算成 ms，和门对比：初始化 0.5 s，观测 2 ms。
5. 过门：出 export.zip 记 sha，打 post 标签，用显式 ref 推送。
   不过门：写清最好一版的 LUT 和拍数，交主控定。上一棒判断实际结果很可能在 4,000–5,000。
6. 写 RUN/SRCHK6/README.md 的结论节。Python 参考解释器这次可以不做，写进未完成项。

## 规矩
- 小构建槽只用 <locks>/nat_build_slot_{1,2,3,5}，规则见 RUN/SLOT_RULES_20261104.md。开构建前确认 MemAvailable ≥12 GB，线程 ≤8。
- 输出节流：日志用 tail 或 grep 看。上下文吃紧（约 250k）就写 HANDOVER_r2.md 交接后结束。
- 期限：16:30 前交回。
