# vivado-checks 校准记录（2026-11-02，本机 Vivado 2022.2 Build 3671981，器件 xck26-sfvc784-2LV-c）

目的：先在已知答案上确认这套批处理脚本报出来的东西是对的，再拿去用。每项写输入、期望（跑之前定的）、实际、结论。
原始输出都在私有草稿 `.scratch_vivado/<项>/`（不提交）：`summary.txt`、`vivado.log`、`linter.rpt/.csv`、`messages.csv`、`timing_summary.rpt`、`methodology.*`。
所有 Vivado 均串行运行（同一时刻只有本线一个），单次最长约 3 分钟，没有超过 15 分钟的。

## 1. 时序摘要（已知答案 = 构建时留下的 `timing_route.rpt`）

已知答案来源：`<artifacts>/20260927-m7-pl-bitstream/build-{100,200}mhz/timing_route.rpt`，
是 2026-09-27 构建时同版本 Vivado 用 `report_timing_summary -max_paths 20 -report_unconstrained` 生成的，与本脚本命令相同，但是由另一次独立运行产生（构建流程内，而本次是重新打开 dcp）。

| 量 | 100 MHz 已知 | 100 MHz 本脚本 | 200 MHz 已知 | 200 MHz 本脚本 |
|---|---|---|---|---|
| WNS (ns) | 0.567 | 0.567 | 0.081 | 0.081 |
| TNS (ns) | 0.000 | 0.000 | 0.000 | 0.000 |
| TNS 失败端点 | 0 | 0 | 0 | 0 |
| TNS 总端点 | 105732 | 105732 | 131820 | 131820 |
| WHS (ns) | 0.010 | 0.010 | 0.010 | 0.010 |
| THS 失败端点 | 0 | 0 | 0 | 0 |
| WPWS (ns) | 3.500 | 3.500 | 1.000 | 1.000 |
| Tcl `get_timing_paths` 最差 setup / hold | — | 0.567 / 0.010 | — | 0.081 / 0.010 |

结论：**通过**。两档逐项一致，两档差异（WNS 0.567 vs 0.081、端点数、WPWS）被正确区分；rpt 解析与 Tcl 独立读数一致。
局限：两档都没有违例（原报告即如此），所以这一项只证明“满足时序时读数正确”。“有违例时读数正确”由第 2 项的示例设计补：
那里 hold 违例 WHS −0.031、THS −0.496、16 个失败端点，rpt 解析与 Tcl `get_timing_paths -hold` 读数（−0.031）一致——但这只是脚本自洽，不是独立已知答案。

只读确认：两次运行前后用 `find <artifacts 目录> -newer <标记>` 检查，**未向 artifacts 写入任何文件**；`.Xil/` 与日志都在草稿输出目录。

## 2. 时序方法学检查

| 输入 | 期望（跑前写） | 实际 | 结论 |
|---|---|---|---|
| 阳性：上游示例 `reference/examples/timing-methodology-checks/multi-violation`（双时钟，clk_b 无 create_clock，重复 create_clock，false_path 指向不存在的时钟），在 KV260 上 `synth_design` 出 dcp 后跑 | 至少报 TIMING-17（clk_b 域寄存器无时钟）与 TIMING-18（IO 无 input/output delay）；不预测其他 ID | 共 36 条：TIMING-17 ×18（Critical Warning）、TIMING-18 ×17（Warning）、XDCC-8 ×1；`methodology.json` 36 条记录，规则字段为 `CHECK` | **通过** |
| 100 MHz routed dcp | 无独立已知答案（构建时没跑 report_methodology） | 0 条（`methodology.rpt` “Violations found: 0”） | 只能说脚本没有失明（阳性对照证明能看到违例），0 条本身无独立答案 |
| 200 MHz routed dcp | 同上 | 0 条 | 同上 |

上游 README 写的是 TIMING-18 对应“重复 create_clock”、TIMING-17 对应“false_path 用了不存在的时钟”，这与 2022.2 实测的触发对象不同（实测 TIMING-17 是 clk_b 无时钟、TIMING-18 是 IO 延迟；重复时钟和坏 false_path 在 2022.2 以 `Constraints 18-1056`、`Vivado 12-4739` 的 CRITICAL WARNING 出现在读约束阶段，不进方法学报告）。按上游 references/TIMING-17.md、TIMING-18.md 的定义，实测归类是对的。
`id-info TIMING-17` 返回 GROUP=1 PRIORITY=1 HAS_REFERENCE_FILE=1，查询脚本可用。

## 3. RTL lint

| 输入 | 期望（跑前写） | 实际 | 结论 |
|---|---|---|---|
| 阳性 A：自写 `calibration_inputs/lint_pos/lint_pos_top.v` | INFER-1（锁存器）、INFER-2（不完整 case）、ASSIGN-7（多驱动）、CLOCK-1（双沿）、ASSIGN-1（加法溢出）、ASSIGN-4（未用输入） | Vivado 总数 6 = CSV 6 行：ASSIGN-7 ×1、INFER-1 ×2（case_q 也被推断为锁存器）、INFER-2 ×1、CLOCK-1 ×1、ASSIGN-10 ×1（未用输入，2022.2 用 ASSIGN-10 而非 ASSIGN-4）。**ASSIGN-1 未报** | 核心四类（多驱动/锁存器/不完整 case/双沿）**通过**；ASSIGN-4 期望错了规则号（同一问题以 ASSIGN-10 报出）；ASSIGN-1 未命中——`a + b + 8'd1` 赋给 8 位寄存器在 2022.2 不触发，原因未查清，如实记为未命中 |
| 阳性 B（已知答案）：上游示例 `reference/examples/rtl-lint/multi-violation`，上游 README 给出 2026.1 下 11 条期望 | 2022.2 应复现除 2022.2 不存在的规则外的全部条目，行号一致 | 10 条，Vivado 总数 10 = CSV 10 行：ASSIGN-1 @137、ASSIGN-2 @45、ASSIGN-3 @53、ASSIGN-5 phantom_sig @59、ASSIGN-6 ×2（dead_signal、mixed_result）、INFER-1 ×2 @90/101、INFER-2 @100、CLOCK-1 @127——与上游表逐条同规则同行号；缺 ASSIGN-14（重复 case 分支，2022.2 无此规则） | **通过** |
| 阴性 A：自写干净计数器 `calibration_inputs/lint_neg/` | 0 条 | Vivado 总数 0，CSV 0 行 | **通过**（修复解析器后；见下） |
| 阴性 B：HLS 生成 RTL `<runs>/m7-hls-bringup-20260916/out/fpga_backproject-10ns/solution/syn/verilog/`（只读） + 浮点 IP 黑盒壳 `calibration_inputs/lint_hls_stubs/` | 不出现 INFER-1/INFER-2/ASSIGN-7/CLOCK-1 | 63 条 = CSV 63 行，全部是 ASSIGN-5 ×7、ASSIGN-6 ×53、ASSIGN-9 ×2、ASSIGN-10 ×1（未用/未赋值位，HLS 接口典型现象）；**无** INFER/ASSIGN-7/CLOCK-1 | **通过** |

过程中发现并修复的问题（仪器校准的价值所在）：
1. 第一次跑阳性 A，Vivado 报 6 条但 CSV 0 行（`lint_count_check=MISMATCH`）：上游解析器只认 2026.1 的 7 列表，2022.2 是 6 列。已改解析器。
2. 第一次跑阴性 A，Vivado 报 0 条但 CSV 2 行假违例：空表时表尾说明文字被切成行。已改解析器。
修复后对阳性 A、阴性 A 的原始 `linter.rpt` 重新解析，结果如上表；阴性 B 与阳性 B 是修复后才跑的。两次问题都是被 “Vivado 总数 vs CSV 行数” 对账抓到的，这道对账保留在 `vv_post.py` 中。
HLS RTL 不加黑盒壳时 lint 中止于 `[Synth 8-439] module '..._ip' not found`（真实使用会遇到，已写进 SKILL.md）。
只读确认：HLS 目录在运行前后 `find -newer` 无新文件。

## 4. elaboration 分析

| 输入 | 期望（跑前写） | 实际 | 结论 |
|---|---|---|---|
| `calibration_inputs/elab_err/`：例化不存在的模块 + 不存在的端口名 | 捕获未定义模块错误，并指到知识库条目 | `elab_rc=1`；messages.csv：ERROR 8-439 “module 'no_such_module' not found” @6 → `resolution/8-439.md`（标题 “Module Not Found”，模式 P1 拼写错误等）；ERROR 8-6156 → `resolution/8-6156.md` | **通过**。端口名错误没报——elaboration 在第一个致命错误处停止 |
| `calibration_inputs/elab_err2/`：8 位信号接 4 位端口 + 不存在的命名端口 | 捕获位宽不匹配与端口名错误 | WARNING 8-689 “width (8) of port connection 'din' does not match port width (4)” @6 → `resolution/8-689.md`（正是端口位宽不匹配的解读）；ERROR 8-448 “named port connection 'dout_typo' does not exist” @7 → **知识库无条目**（`NO_FILE`）；另有 WARNING 8-4446、ERROR 8-6156，均有条目 | **通过**（捕获全部；8-448 知识库缺口按上游流程走 `uncovered-synth-messages.md`） |

## 汇总

| 项 | 结论 |
|---|---|
| 时序摘要 100/200 MHz 对已知报告 | 通过（逐项一致） |
| 方法学阳性对照 | 通过 |
| 方法学 0 违例读数（m7 两档） | 无独立已知答案；仪器经阳性对照证明不失明 |
| lint 上游已知答案示例 | 通过（10/11，缺的是 2022.2 不存在的规则） |
| lint 自写阳性 | 核心四类通过；ASSIGN-1 未命中（如实记录，未改期望） |
| lint 阴性（干净模块 / HLS RTL） | 通过 |
| elaboration 两个错误对照 | 通过 |
