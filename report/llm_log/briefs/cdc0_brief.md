# CDC0：m5 整板跨时钟域与方法学严重项逐条定性（2026-11-04，约 30 分钟包）

你是 FPGA 国赛线执行代理，交回后主控验收。输出节流（tail/grep，长内容落文件），上下文约 200k 收尾。调用全局 skill `vivado-checks`。

## 为什么
文档正文写"跨时钟域报告 0 个严重警告""方法学 0 个新增严重项"。核数发现 m5 整板（RUN/MERGE7/builds/impl-m5-166-f35f07f/）cdc.rpt 有 48 条 Critical、methodology.rpt 有 2 条 TIMING-54 Critical Warning；m4 时 cdc 是 0 条。要弄清这些是什么：是 AMD 自带 IP（DPU、互连、时钟转换器、复位）内部已带同步器/约束的路径，还是我们自己的核/拼接引入的真问题。m6 很快要整板构建，若是真问题要在 m6 前修。

## 要做
1. 按 RUN/SLOT_RULES_20261104.md 取一个小构建槽（1/2/3/5 任一），vivado batch 打开 routed.dcp（只读，不改不写回），`report_cdc -details -file`、`report_methodology`、必要时 `report_clock_interaction`。线程 ≤8。
2. 48 条 Critical 按"源时钟→目的时钟、源/目的所在层级（哪个 IP/哪个核）、CDC 类型（如 CDC-1/CDC-4/CDC-10 …）"分组。每组判断：是否有 ASYNC_REG 同步器、是否 AMD IP 自带 XDC 已有 set_max_delay -datapath_only / set_false_path 覆盖、是否属于静态配置寄存器（只在启动时写一次）。TIMING-54 两条同样定性（哪两个时钟、来自哪条约束）。与 m4 比，指出 0→48 是哪次拼接引入的（看层级名）。
3. 结论三选一，每组给出：①AMD IP 内部已处理的、可以安全豁免，写清依据（IP 文档或 XDC 行）；②我方逻辑但实际安全（如准静态寄存器），给出加 ASYNC_REG/set_false_path 的最小 XDC 改法，供 m6 带上；③真问题，给出修法与改动位置。
4. 交 `RUN/CDC0/README.md`：分组表、结论、给 m6 的 XDC 建议（若有，放 `RUN/CDC0/m6_cdc.xdc`），以及文档该怎么写（例："跨时钟域路径全部经同步器或约束覆盖"能否成立）。

## 规矩
不改 m5 产物、不改正文、不碰板子。用完释放槽（删 owner、rmdir、HISTORY.log 写 RELEASE）。只按 pid 停进程。
