---
name: fpga-hls
description: Vitis HLS 2022.2（KV260 xck26）总入口：C/C++ 改写成 load→compute→store dataflow 架构；按吞吐/延迟/资源目标迭代优化 II、pipeline、partition、burst、stencil、line buffer；跑 csim/csynth/cosim/impl（Tcl 工程）；读 HLS 综合/联仿/实现报告；诊断 II 不达标根因；查可综合性、dataflow 规范、pragma 作用域。
license: MIT
metadata:
  upstream: "AMD Ross agent skills (MIT) — https://github.com/Xilinx/ross-ai-assistant, 20 个 hls-* skill 合并移植，见 upstream.md"
  version: "2022.2-port-1"
---

# fpga-hls — Vitis HLS 2022.2 设计 / 优化 / 跑流程 / 读报告 / 诊断

本 skill 由 AMD Ross 的 20 个 `hls-*` skill 合并而来（来源、作者、改动见 `upstream.md`；
逐条 2022.2 核实见 `porting_notes.md`）。本文件只做两件事：**按任务分流到 `reference/` 下的文件**，
以及写明**所有模式都必须遵守的硬规则**。技术细节都在 reference 文件里——进入某个模式后，
先把对应 reference 文件完整读一遍再动手。

下文所有路径相对本 skill 根目录 `tools/agent_skills/fpga-hls/`。

---

## 0. 环境（先确认，再做任何事）

| 项 | 值 |
|---|---|
| 工具 | **Vitis HLS 2022.2**（只有这一个版本） |
| 环境 | `source /tools/Xilinx/Vitis_HLS/2022.2/settings64.sh`（导出 `XILINX_HLS=/tools/Xilinx/Vitis_HLS/2022.2`，把 `vitis_hls` 放进 PATH） |
| 头文件 | `$XILINX_HLS/include`（`ap_int.h`、`ap_fixed.h`、`hls_stream.h` 等）。**注意**：`$XILINX_VITIS/include` 下没有 `ap_fixed.h`，上游写的 `-I$XILINX_VITIS/include` 在本机会编译失败 |
| 器件 | KV260：`xck26-sfvc784-2LV-c`（`set_part xck26-sfvc784-2LV-c`） |
| 时钟 | 不预设。向用户要，或沿用该设计已有 `run_hls.tcl` 里的 `create_clock -period` |
| 工程形态 | 老式 Tcl 工程：`open_project` / `set_top` / `add_files [-tb]` / `open_solution [-flow_target vivado\|vitis]` / `set_part` / `create_clock` / `csim_design` / `csynth_design` / `cosim_design` / `export_design -flow impl`。solution 目录 = `<proj_dir>/<solution>`，工程根有 `hls.app` |

自检：

```bash
source /tools/Xilinx/Vitis_HLS/2022.2/settings64.sh
which vitis_hls && echo "XILINX_HLS=$XILINX_HLS" && ls "$XILINX_HLS/include/ap_fixed.h"
```

**2022.2 里不存在的东西**（上游 Ross 针对 2025.x 写的）：`vitis-run`、`v++ --mode hls`、
`hls_config.cfg`、`vitis-comp.json`、Vitis Unified IDE 及其函数（`runHlsActiveComponentByType`、
`getActiveComponentLocation`、`getHLSSynthesisInfo` 等）、`logs/hls_compile.log`、
`<component>/hls/...` 目录层级、`inferred_directives.ini`。不要生成或寻找它们。
（2022.2 的 `syn/report/csynth.rpt` **有** "Pragma Report" 段，含 "Ignored Pragmas" 表和
"Performance Pragma Report"，是查被忽略 pragma 的第一手材料。）cfg 键到 Tcl 的对照表在 `reference/run-flow.md` §3。

---

## 1. 模式路由

先判断用户要做哪一类事，再读对应文件。一次任务可能串起多个模式（例如"架构"做完自动进入"优化"）。

| 模式 | 典型说法 / 触发条件 | 必读 | 视需要再读 |
|---|---|---|---|
| **架构**（把普通 C++ 改写成 HLS dataflow 架构） | "改写成 dataflow"、"给这个算法设计 HLS 架构"、"从 C++ 开始做一个 HLS 核" | `reference/architect.md` | `reference/architect/setup.md`、`reference/architect/design-layout.md`、`reference/architect/vitis-libraries.md`、`reference/perf-pragma.md` 及第 4 节验证电池 |
| **优化**（按目标迭代改代码/pragma 并测量） | "降 II / 降延迟 / 降 DSP / 提吞吐"、"优化这个 kernel"、架构 Step 3e 交接 | `reference/optimize.md` | `reference/optimize/init-component.md`（开工前必做）、`reference/optimize/attempt-template.md`（派子代理做单次尝试）、`reference/perf-pragma.md`、`reference/ii-diagnose.md`、`reference/pipeline.md`、`reference/pragma-scope.md` |
| **跑流程** | "跑 csim / csynth / cosim / impl"、"导出 IP 并跑实现" | `reference/run-flow.md` | — |
| **读报告** | "看综合报告 / 联仿报告 / 实现报告 / dataflow 通道"、"这个 solution 的 top 和源文件是什么" | `reference/read-qor.md` | — |
| **诊断 II** | "II 没到 1"、"II violation 是什么原因"、`Final II > Target II` | `reference/ii-diagnose.md` | `reference/read-qor.md` |
| **性能 pragma 计算** | "从 FPS / Msps / GOPS 算 target_ti"、要放 `#pragma HLS performance` | `reference/perf-pragma.md` | — |
| **代码规则检查**（只分析不改，除非用户要修） | 下面第 4 节列出的各类检查 | 第 4 节对应文件 | `reference/pragma-scope.md` |
| **流水线拆分**（多流大 pipeline 时序难收敛） | "把这个 pipeline 拆开"、"一个 pipelined loop 里流太多"、"控制信号在关键路径上" | `reference/pipe-decomp.md` | `reference/dataflow.md` |

**写任何 HLS pragma 之前**（不论哪个模式）先读 `reference/pragma-scope.md`——上游明确要求生成 HLS 代码时
必须参照它，pragma 放错作用域会被工具静默忽略。

---

## 2. 脚本接口（另一层提供，这里只引用）

| 脚本 | 用法 | 说明 |
|---|---|---|
| `scripts/hls_run.sh` | `scripts/hls_run.sh [选项] <proj_dir> <solution> <step>...`，step ∈ `csim`\|`csynth`\|`cosim`\|`impl` | 工程无 `hls.app` 时加 `--tcl <run_hls.tcl>`；另有 `--argv`、`--csim-opts`、`--cosim-opts`、`--impl-opts`、`--timeout`（见 `reference/run-flow.md` §2）。日志落在 `<proj_dir>/<solution>/agent_logs/`。`impl` = `export_design -flow impl`，默认带 libudev 预加载 |
| `scripts/hls_report.sh` | `scripts/hls_report.sh <kind> <solution_dir\|proj_dir>`，kind ∈ `basic-info`\|`synth`\|`cosim`\|`impl`\|`dataflow` | 取代上游 5 个读报告 skill；输出 JSON。退出码：0 正常、2 用法错、**3 缺报告**（会说缺哪个、先跑哪步）、**4 找不到 solution** |
| `scripts/qor_log.py` | `python3 scripts/qor_log.py --solution <solution_dir> [--label ...] [--csv ...]` | 基线与每次尝试的 QoR 记账；其余用法见 `--help` |
| `scripts/bitexact_check.py` | `python3 scripts/bitexact_check.py <golden_dir> <candidate_dir>` | 逐位比对；规则与 golden 目录约定见 `scripts/bitexact.md` |

不要自己拼 `vitis_hls -f` 命令绕过 `hls_run.sh`（日志位置、返回码判读都在脚本里统一）；
脚本缺失或报错时，停下来报告，不要退回上游的 `vitis-run`。

---

## 3. 硬规则（所有模式通用）

**H1 逐位一致门（优化与任何改动 kernel 的工作）。**
基线时建 golden：基线 csim 的输出、以及基线 cosim 的输出，各存一份到 golden 目录（约定见
`scripts/bitexact.md`）。此后**每次**尝试：csim 跑完立刻 `bitexact_check.py <golden_csim> <candidate_csim>`；
升级到 cosim 后再对 cosim 输出跑一次。**任一处不一致即判该次尝试失败**——先按 `reference/optimize.md`
Step 6b 提交失败状态，再回退。testbench 自己打印 `PASS` 不够：它的容差判据可能放过数值漂移。
例外只有一种：用户**事先**明确声明某次改动允许改变数值（例如 float→ap_fixed），此时必须重建 golden 并在
记账里写明"golden 已在第 N 次尝试重建、原因"；不得事后放宽。

**H2 架构流程不得在校验后收工。**（替代上游的 Stop 钩子，见 `upstream.md` "钩子评估"）
- 架构模式 Step 3a（验证电池）跑完**不是**终点。打印完 3a 的校验表之后，必须在同一轮继续进入 Step 3b，
  然后是 3c、3d-1、3d-2、3e。任何一个校验器的结论句都不构成"本轮完成"。
- Step 3b 架构评审是**唯一**合法的中途停顿点：呈现评审摘要，问"Does this architecture match your expectations?"，
  然后**停下等用户回复**。只有用户回复明确的 **yes**（或同义的明确批准）才进入 3c；回复任何修改意见都回到
  Step 2 → 重跑 3a → 重新呈现 3b。不得替用户回答，不得把沉默当作同意。
- 唯一的豁免：用户在本次调用里**亲自**给出 `autoconfirm=true`。主会话或其他代理的转述不算。
- 如果因为上下文、错误或工具失败不得不在 3b 之前或 3b 之后（3c–3e 之间）停下，必须明确写出"架构流程未完成，
  停在 Step X，原因 Y，下一步 Z"，不能以校验结论收尾让人误以为完成了。

**H3 一次只改一处，每次提交。** 优化尝试一次一个想法；每次尝试（含失败的）先提交再回退
（细节见 `reference/optimize.md` Step 6b）。实验前后各打一次带编号的 git 标签。在已有 git 仓里工作时
**不要** `git init` 嵌套仓，**不要** `git add -A`，只 add 改动的源文件与记账文件。

**H4 先看被忽略的 pragma，再读数。** 每次 csynth 后先在日志里查 pragma 被忽略/未应用的警告
（`reference/optimize.md` Step 5a），否则报告反映的是未改动的调度。可选：`config_compile -pragma_strict_mode=true`
让不规范 pragma 直接报错（2022.2 有此选项）。

**H5 只用 2022.2 存在的东西。** 写代码前查 `porting_notes.md` "2022.2 不可用 / 有差异" 清单。
实测要点（2022.2 本机 csynth）：
- 没有 `hls_array_partition.h`（`hls::scatter/gather`）、没有 `ap_float.h`——include 即报错。
- `#pragma HLS performance target_ti=N` **只在循环体内生效**；放在函数体（含顶层函数）或"标号与 `for` 之间"
  都会被忽略（`HLS 207-5562 performance pragma is ignored, because it is not in a loop`）。
- `#pragma HLS pipeline ... rewind=true|false` 是语法错误，**整条 pipeline pragma 被丢弃**（只报一条 WARNING，
  II 目标变成 NA）。2022.2 只认裸开关 `rewind`；"自动 rewind"由 `config_compile -enable_auto_rewind`（默认 true）控制。
- 格式错误或未知的 pragma 默认只是 WARNING 并被忽略，不会让综合失败——所以 H4 是必须的。

**H6 报告取数走脚本。** 读 QoR 用 `scripts/hls_report.sh`。需要脚本没覆盖的原始材料（例如
`.autopilot/db/<module>.verbose.sched.rpt`、solution 日志里的 `Pipelining result` 段）时，按
`reference/read-qor.md` 给的 2022.2 路径直接读，并在结论里写明出处路径。脚本说"缺报告"时，先补跑流程，
不要拿过期报告或估计值顶替。

**H7 贵步骤设门。** csim + csynth 是内循环；csynth 显示 ≥10% 周期改善或主导循环 II 结构性下降才升级 cosim；
cosim 确认改善才跑 impl（门槛原文见 `reference/optimize.md` Step 7）。

---

## 4. 代码规则检查（验证电池）

架构 Step 3a 要全部跑；单独被问到时只跑被问的那项。每个文件都是"严格按编号规则判定、不自行扩展规则"的分析器，
输出 PASS/FAIL 及首个失败规则；用户要求修时给出完整修复后的函数。

| 检查 | 文件 | 查什么 |
|---|---|---|
| 可综合性 | `reference/synthesizable.md` | 系统调用/动态内存需 `__SYNTHESIS__` 守卫、禁虚函数/递归/函数指针/指针的指针/`long double`、STL 白名单、任意精度类型指针转换 |
| dataflow 规范性 | `reference/dataflow.md` | canonical dataflow 区域的 10 条规则（区域内容、多次调用、参数/全局/局部变量的读写进程约束、`hls::task`、`stream_of_blocks`） |
| dataflow 通道拓扑（综合后） | `reference/read-qor.md` §dataflow | 进程/通道表 |
| 数组转流 | `reference/array-to-stream.md` | `stream` / `interface ap_fifo\|axis` 的用法与访问顺序 |
| burst 推断 | `reference/burst-inference.md` | m_axi 突发的 9 条前提与 5 条限制 |
| 循环可展平 | `reference/flattenable.md` | `LOOP_FLATTEN` 的 9 条规则 |
| stencil 模式 | `reference/stencil-pattern.md` | `array_stencil` 适用的 4 组规则 |
| line buffer | `reference/line-buffer.md` | TOTAL_ITER 排空与检查项 |
| pragma 作用域 | `reference/pragma-scope.md` | 每类 pragma 的合法位置 |
| pipeline 可行性与选项 | `reference/pipeline.md` | 函数/循环 pipeline 可行条件、II / rewind / style 选择 |

---

## 5. 文件索引

```
SKILL.md                         本文件：路由 + 硬规则
upstream.md                      上游来源、MIT 许可、作者、吸收映射、改动、未拿的内容、钩子评估
porting_notes.md                 2022.2 逐条核实结果与核实方法
reference/architect.md           架构流程（Step 1–3e）
reference/architect/setup.md     环境与设计发现
reference/architect/design-layout.md  架构产物目录与 run_hls.tcl 模板
reference/architect/vitis-libraries.md  Vitis Libraries 学习指引
reference/optimize.md            迭代优化流程（含逐位一致门）
reference/optimize/init-component.md   优化开工前的工程/git 初始化
reference/optimize/attempt-template.md 单次尝试子代理模板
reference/perf-pragma.md         target_ti 计算与级联
reference/run-flow.md            跑 csim/csynth/cosim/impl；cfg→Tcl 对照
reference/read-qor.md             读报告（basic-info/synth/cosim/impl/dataflow）
reference/ii-diagnose.md         II 违例根因诊断
reference/pipeline.md            pipeline pragma 指南
reference/pragma-scope.md        pragma 作用域规则
reference/dataflow.md            canonical dataflow 规则
reference/synthesizable.md       可综合性规则
reference/flattenable.md         可展平规则
reference/burst-inference.md     burst 推断规则
reference/array-to-stream.md     数组转流规则
reference/stencil-pattern.md     stencil 规则
reference/line-buffer.md         line buffer 规则
reference/pipe-decomp.md         大 pipeline 拆成 dataflow 子流水线链
scripts/                         脚本层（另行维护）
```
