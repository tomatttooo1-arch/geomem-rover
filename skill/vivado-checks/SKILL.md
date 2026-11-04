---
name: vivado-checks
description: Vivado 2022.2 批处理检查（KV260，无需 MCP）：RTL lint（synth_design -lint）、elaboration 报错/[Synth 8-xxx] 解读、时序摘要 WNS/TNS 与方法学检查 report_methodology（TIMING-xx 违例诊断与修复建议）。Use for RTL lint, timing methodology, timing violations, elaboration errors on KV260 dcp/RTL/HLS IP.
license: MIT
metadata:
  version: "0.1.0-fpga-20261102"
  upstream: "Xilinx/ross-ai-assistant (MIT) — see upstream.md"
---

<!--
Portions Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
Modified 2026-11-02 (fpga): 三个上游 Vivado skill 合并为一个批处理入口；MCP 调用改为 scripts/vv_run.sh。
-->

# vivado-checks —— Vivado 2022.2 批处理检查（lint / elaboration / 时序方法学）

这是 AMD 开源 Ross 包里三个 Vivado skill 的批处理移植（来源、改动、许可见 `upstream.md`）。
上游原本经 Vivado MCP Server 的 `vivado_execute` 与常驻 Vivado 会话交互；这里**全部改为
一次性 `vivado -mode batch`**，由 `scripts/vv_run.sh` 统一包装：自动 source 2022.2 环境、
把工作目录放在输出目录（`.Xil/`、日志都落在那里，不污染输入目录）、终端只打 `VV_SUMMARY` 摘要行。

**与上游文档冲突时以本文件为准**。上游文档里 “Never create .tcl / Never run `vivado -mode batch`”
“用 vivadoExecute / vivado_doc_search” 等指令在本移植里不适用；其余技术内容（检查项、解读方法、
修复建议、resolution 知识库）原样保留在 `reference/` 下，按下文指针去读。

默认器件 `xck26-sfvc784-2LV-c`（KV260），Vivado `/tools/Xilinx/Vivado/2022.2/settings64.sh`。
机器多人共享：**一次只跑一个 Vivado**；默认 `VV_THREADS=4`、`VV_TIMEOUT=900`（15 分钟）超时即停并记录，不盲目重跑。
日志只用 `tail`/`grep` 看，不要 `cat` 整个 `vivado.log` 或 `timing_summary.rpt`。

## 模式总览

| 模式 | 命令 | 产物（在 `--out` 目录） | 详细方法 |
|---|---|---|---|
| RTL lint | `vv_run.sh lint [--top T] SRC...` | `linter.rpt` → `linter.csv` | `reference/vivado-rtl-lint/` |
| elaboration 分析 | `vv_run.sh elab [--top T] SRC...` 或 `vv_run.sh elab-log <runme.log>` | `vivado.log` → `messages.csv` | `reference/vivado-rtl-elaboration-analysis/` |
| 时序摘要 + 方法学 | `vv_run.sh timing --dcp routed.dcp` 或 `--xpr p.xpr --run impl_1` | `timing_summary.rpt`、`methodology.rpt/.json`、`methodology_counts.csv`、`constraints.xdc` | `reference/vivado-timing-methodology-checks/` |
| 方法学 ID 查询 | `vv_run.sh id-info TIMING-16` | 终端 | 同上 `references/SUPPORTED_IDS.md` |

`SRC` 可以是文件、目录（递归找 `.v/.sv/.vhd/.vhdl`）或 `.f` 文件列表；可重复 `--inc DIR`、`--define X=1`；
`--part` 覆盖器件；`--out DIR` 指定输出目录（默认 `./vivado_checks_out/<mode>-<时间戳>`）。
脚本路径：`tools/agent_skills/vivado-checks/scripts/vv_run.sh`（仓内相对路径）。

## 模式 1：RTL lint

1. 跑：`vv_run.sh lint --top <top> --out <dir> <rtl 目录或文件>`。
2. 读摘要：`lint_vivado_total`（Vivado 日志里的 “Total of X linter message(s)”）与 `lint_csv_rows`
   必须相等（`lint_count_check=OK`）；不等时按上游回退规则直接读 `linter.rpt` 的 “2. Expanded” 表。
3. 按上游流程写报告：`reference/vivado-rtl-lint/upstream_skill.md` 的 Step 4–6
   （CSV 7 列含义、T1/T2 行、按规则/文件分组、分级处置、每条违例读真实源码 ±5 行、diff 格式修复）。
   规则 → 处置见 `violation-handlers.md`；有 `resolution/<RULE-ID>.md` 的必须先读再写修复；
   报告版式见 `report-format.md` 与 `examples/`。
4. **只报 CSV 里有的违例；0 就是 0**（上游反例：把 `// 注释` 当代码误报）。

2022.2 差异（已在本机核实，见 `porting_notes.md`）：
- `synth_design -lint -file` 可用；输出只有 `.rpt`（2026.1 才直接出 CSV），已由 `parse_lint_report.py`（已改，兼容 2022.2 的 6 列表）转 CSV，`info` 列填 `N/A`。
- 2022.2 实测会报的规则：ASSIGN-1/2/3/5/6/7/9/10、INFER-1/2、CLOCK-1（`help synth_design` 列的是 ASSIGN-1..7、QOR-1、INFER-1..3、CLOCK-1、RESET-1/2，但实测还会报 ASSIGN-9/10，帮助文本不全）。
  上游的 ASSIGN-12/14、RESET-3 等新规则在 2022.2 **不会出现**——没报不代表没有这类问题（例：重复 case 分支 ASSIGN-14 在 2022.2 漏报，已实测）。
- 豁免：`create_waiver -type LINT -id <RULE> ...`（2022.2 支持）。

**HLS 生成 RTL 的注意点**：HLS 的浮点等运算会例化 `*_ip` 模块（由 `*_ip.tcl` 生成 IP），只读 `.v` 会在
elaboration 报 `[Synth 8-439] module '..._ip' not found` 而 lint 中止。做法：给这些 `_ip` 写 `(* black_box *)`
空壳模块（端口照 HLS 包装器里的例化抄，示例 `calibration_inputs/lint_hls_stubs/hls_fp_ip_stubs.v`）一起传入。
HLS RTL 典型会报大量 ASSIGN-5/6/9/10（AXI/握手的未用位），这类多为 “Accept/waive” 档；
INFER-1（锁存器）、ASSIGN-7（多驱动）、CLOCK-1 若出现在 HLS RTL 里才值得警惕。

## 模式 2：elaboration 分析

1. 已有日志（`runme.log`/`vivado.log`）优先：`vv_run.sh elab-log <log>`，不起 Vivado。
   没有日志：`vv_run.sh elab --top <top> SRC...`（`synth_design -rtl`，可加 `--skip-ip`）。
2. 摘要 `elab_ids=` 给出每个非 INFO 的 `[Synth 8-xxx]` 及是否有知识库文件
   `reference/vivado-rtl-elaboration-analysis/resolution/8-xxx.md`（`NO_FILE` 表示没有）。
3. 按上游流程写报告：`reference/vivado-rtl-elaboration-analysis/upstream_skill.md` Step 3.5–6
   （resolution checklist → 分级 Tier1/2/3 → 读真实 RTL ±10 行 → diff + 可粘贴修复 → `REPORT.md`；
   每条非 INFO 消息都要有一节，不许丢）。ID → 处置速查 `message-handlers.md`；无文件的 ID 用
   `resolution/uncovered-synth-messages.md` 与自己的分析（上游的 `vivado_doc_search` 不可用，可查 UG901 本地/网页版）。
   超过 50 条消息时按上游 Step 5A 用 `csv_to_per_file_parts.py` 分片。
4. 注意：elaboration 在第一个致命错误处停止（例：缺模块 8-439 会挡住后面的端口名错误），修完一轮要再跑一轮。

知识库（553 个 `8-xxx.md` + 若干模式文件）按上游原样保留，是 Oasys 消息 ID 级的解读与修复模板。

## 模式 3：时序摘要 + 时序方法学检查

1. 跑：`vv_run.sh timing --dcp <routed 或 synth dcp> --out <dir>`（只读打开，不写回检查点；
   大设计 routed dcp 约 3 分钟）。`--skip-methodology` 只出时序摘要。
2. 摘要：`wns_ns/tns_ns/tns_failing_endpoints/whs_ns/ths_ns/ths_failing_endpoints/wpws_ns`
   取自 `timing_summary.rpt` 的 Design Timing Summary 行（命令与构建脚本一致：
   `report_timing_summary -max_paths 20 -report_unconstrained`）；`tcl_worst_setup/hold_slack`
   是 `get_timing_paths` 的独立读数，两者应一致。`timing_constraints=met` 表示报告里有
   “All user specified timing constraints are met”。
3. 时序违例诊断：在 `timing_summary.rpt` 里 `grep -n "VIOLATED\|Slack (VIOLATED)"` 找最差路径，
   按 UG906 看逻辑级数/布线延迟占比/跨时钟域；不要把整个 rpt 读进上下文。
4. 方法学：`methodology_counts.csv` 给每条规则的数量；`methodology.json` 每条违例的全部属性
   （由上游 `scripts/write_json_report.tcl` 生成；**2022.2 的规则字段叫 `CHECK`，上游 2026.1 JSON 叫 `RULE`**）。
   然后按上游流程：`reference/vivado-timing-methodology-checks/upstream_skill.md` Step 3–10
   ——`vv_run.sh id-info <ID>` 查 GROUP/PRIORITY → 先 GROUP 1 再 GROUP 2 → 读
   `references/GENERAL_GUIDELINES.md` 与 `references/<ID>.md` 的 Generate Data / 解决流程 →
   AUTO 修复可直接改约束，USER 修复必须问 owner 不猜设计意图 → 改约束后重跑本模式验证 →
   写 `METHODOLOGY_RESOLUTION_REPORT.md`。
5. 修约束的批处理验证（替代上游 Step 8 的会话内 `reset_timing; read_xdc`）：写一个小 Tcl
   `open_checkpoint X.dcp; reset_timing; read_xdc new.xdc; report_methodology -file m2.rpt`，
   或在我们的构建脚本里改 XDC 重新出 dcp 再跑本模式。
6. 豁免：`create_waiver -of_objects [get_methodology_violations TIMING-x#n] -user <名> -description <理由>`，必须有理由。

本项目的 BD 设计时钟来自 PS（`pl_clk0`），常见方法学项目是 TIMING-18（IO 无约束，通常不适用于纯 PS-PL 设计）、
TIMING-17（未被时钟到达的寄存器）、跨时钟域相关。0 违例并不罕见（实测 m7 100/200 MHz routed dcp 都是 0），
但 0 要配阳性对照才可信——见 `calibration_20261102.md`。

## 可信度（校准）

本移植在本机 2022.2 上做过已知答案校准，记录在 `calibration_20261102.md`：
时序摘要对已有 `timing_route.rpt` 逐项一致；lint 在上游示例上复现 10/11 条期望违例（缺的是 2022.2 没有的 ASSIGN-14）、
阳性/阴性对照符合；elaboration 能捕获缺模块/端口名错误并指到知识库条目。

## 文件

- `scripts/vv_run.sh`（入口）、`vv_common.tcl`、`vv_lint.tcl`、`vv_elab.tcl`、`vv_timing.tcl`、`vv_post.py`
- `reference/vivado-rtl-lint/`、`reference/vivado-rtl-elaboration-analysis/`、`reference/vivado-timing-methodology-checks/`（上游原样，`SKILL.md` 改名 `upstream_skill.md`）
- `reference/examples/`（上游示例工程）
- `calibration_inputs/`（校准用小 RTL）、`calibration_20261102.md`、`porting_notes.md`、`upstream.md`、`license_upstream.txt`
