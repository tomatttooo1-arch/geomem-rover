# vivado-checks：Vivado 2022.2 批处理检查（RTL lint、elaboration 报错、时序与方法学）（基于上游 AMD Ross 改进）

类型：校验脚本 + 提示词工作流。智能体入口是 `SKILL.md`。

**来源声明**：基于上游 AMD Xilinx/ross-ai-assistant（MIT，plugin.json 2026.10.1，2026-11-02 下载，归档 sha256 `a20046c7…ab9d23`）的
`vivado-rtl-lint`、`vivado-rtl-elaboration-analysis`、`vivado-timing-methodology-checks` 三个 skill。上游依赖 Vivado MCP 和 2026.1；
我们改成**不需要 MCP 的批处理 Tcl**，在 Vivado 2022.2 上跑。来源与改动表见 `upstream.md`，许可原文 `license_upstream.txt`。
上游 630 个参考文件（主要是 560 条 `8-xxx.md` 报错知识库）没有随包分发，用 `reference/fetch_upstream_reference.sh` 一条命令取回（见 `reference/README.md`）。

## 目录

| 路径 | 是什么 |
|---|---|
| `SKILL.md` | 三种模式（lint / elab / timing）的工作流与 2022.2 差异 |
| `scripts/vv_run.sh` | 统一入口：`vv_run.sh lint [--top T] SRC...`、`vv_run.sh elab ...`、`vv_run.sh elab-log <runme.log>`、`vv_run.sh timing --dcp routed.dcp` |
| `scripts/vv_lint.tcl`、`vv_elab.tcl`、`vv_timing.tcl`、`vv_common.tcl` | 批处理 Tcl（只读打开 dcp，不写回） |
| `scripts/vv_post.py` | 后处理：lint 报告转 CSV 并做"Vivado 总数 = CSV 行数"对账；elab 报错按 ID 计数并指向知识库条目 |
| `reference/vivado-rtl-lint/parse_lint_report.py` | 上游解析器，改成兼容 2022.2 的 6 列表 |
| `calibration_20261102.md`、`calibration_inputs/` | 已知答案上的校准记录和校准用的小设计（阳性、阴性各一组） |
| `porting_notes.md` | 移植时逐条核实的 2022.2 差异 |

## 适用场景

- 整板或单核构建之后，让智能体读时序（WNS/TNS/WHS）和方法学违例（TIMING-xx），给出定性和修复建议；
- 手写 RTL 或 HLS 生成 RTL 的 lint；
- 读 `[Synth 8-xxx]` elaboration 报错并查对应的修复条目。

## 用法

1. 装进 Claude Code：复制成 `~/.claude/skills/vivado-checks/`；需要报错知识库时先跑 `bash reference/fetch_upstream_reference.sh`。
2. 环境：`export VV_SETTINGS=<你的 Vivado 2022.2>/settings64.sh`；可选 `VV_TIMEOUT`（默认 900 秒）、`VV_THREADS`（默认 4）。
   我们机器上 2022.2 的许可库加载会段错误，脚本默认预加载 libudev 规避；你的机器没这问题可设 `VV_NO_UDEV_PRELOAD=1`。
3. 例：`bash scripts/vv_run.sh timing --dcp <build>/routed.dcp`，终端只打一行 `VV_SUMMARY`，完整日志在输出目录的 `vivado.log`。
4. 第一次在你的工程上用时，拿一个你已经知道答案的 dcp（构建时留下的 `timing_route.rpt`）对一遍 WNS/TNS。

## 失效条件

- Vivado 版本不是 2022.2：lint 表的列数、方法学规则号的触发对象都会变（见下文"失败"第 1、3 条），要重新校准。
- 方法学检查报 0 条时**没有独立已知答案**：我们的构建流程不跑 `report_methodology`，0 条只说明脚本"能看到违例"（阳性对照），不证明设计干净。
- elab 只报它能报的：校准里"端口名写错"那一项 2022.2 没有报（见校准第 4 节），不要把"没报错"当成"没问题"。
- HLS 生成的 RTL 里有浮点 IP 黑盒时，不加黑盒壳 lint 会中止于 `[Synth 8-439] module not found`，校准输入里有现成的黑盒壳写法。
- lint 不保证抓到所有加法溢出：校准里 `a + b + 8'd1` 赋给 8 位寄存器在 2022.2 没有触发 ASSIGN-1。
- 不做 CDC 结论：跨时钟域请用 `../cdc-guard/`。

## 已验证的效果（`calibration_20261102.md`）

| 项 | 输入 | 结果 |
|---|---|---|
| 时序摘要 | 两个已构建的整板 routed.dcp（100 / 200 MHz），已知答案是构建时生成的 `timing_route.rpt` | WNS 0.567 / 0.081、TNS、失败端点、总端点（105,732 / 131,820）、WHS、WPWS **逐项一致** |
| 方法学 | 阳性：上游示例（一个时钟缺 `create_clock` 等） | 报出 36 条，含 TIMING-17、TIMING-18 |
| lint 阳性 | 自写违例设计；上游示例（上游列 2026.1 下 11 条期望） | 自写设计：多驱动、锁存器、不完整 case、双沿四类核心违例全部报出，未用输入以 ASSIGN-10 报出，**加法溢出（ASSIGN-1）未命中、原因未查清**；上游示例 11 条期望报出 10 条，同规则同行号（缺的 ASSIGN-14 是 2022.2 没有的规则） |
| lint 阴性 | 干净计数器；HLS 生成的反投影核 RTL | 0 条；63 条全是未用位类（HLS 接口常见），无锁存器、多驱动、双沿 |
| elab | 例化不存在的模块 | 报 8-439 并指到知识库条目 |

## 从哪些失败里总结出来

1. 第一次跑 lint 阳性，Vivado 报 6 条，CSV 却是 0 行：上游解析器只认 2026.1 的 7 列表。被"Vivado 总数 vs CSV 行数"对账抓到，改了解析器，这道对账保留在 `vv_post.py` 里。
2. 第一次跑 lint 阴性，Vivado 报 0 条，CSV 却有 2 行假违例：空表时表尾说明被切成了数据行。同一道对账抓到。
3. 上游文档写的 TIMING-17/18 对应关系在 2022.2 上不成立（2022.2 里"重复时钟"和"坏 false_path"在读约束阶段以 CRITICAL WARNING 出现，不进方法学报告）。教训：上游规则号的含义要在本版本上用阳性对照重新确认。
4. 上游依赖 MCP 服务，我们的构建机上没有，所以全部改成 batch Tcl，只读打开 dcp。
