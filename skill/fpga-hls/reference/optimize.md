> 来源：AMD Ross `hls-optimize`（作者：Alex Li, Sai Akhil Ayyagari），MIT 许可，见 `../upstream.md`。已按 Vitis HLS 2022.2 移植。
> 上游描述：Iteratively optimize the HLS kernel against a given criteria (e.g. minimize latency, reduce DSP usage, maximize throughput) by applying and measuring pragma and algorithmic changes.
<!--
Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
-->

# Skill: hls-optimize

Iteratively modify the kernel to improve performance against an optimization criteria, using the other skills to measure progress and committing each attempt.

**Optimization criteria**: $ARGUMENTS

Examples (criteria strings the user may give):
- `minimize latency`
- `minimize DSP usage while meeting timing`
- `maximize throughput (minimize II)`
- `reduce LUT count below 5000`

## Init component and git repository

**MANDATORY**: Before any optimization work, you MUST read and execute `reference/optimize/init-component.md`.

> **团队仓库改造（owner 2026-11-02）**：上游这里要求执行 `git init` / `git config` / 改 `.gitignore`。
> 在本仓库**一律禁止**：`git init`、`git config`（任何作用域的写入）、修改 `.gitignore`、`git push`、
> 实验区以外的 `git reset --hard`、`git clean`。身份沿用仓库已有配置。所有逐次尝试只在
> `session/hlsopt_session.sh start` 建出的本地实验分支 `exp/<本人>/hls-opt-<模块>-<YYYYMMDD>`
> （副本外的独立 worktree）里提交；收尾时补丁+账本存档到 `shared/artifacts/`，再向用户功能分支做**一次**提交。
> git 操作一律经 `session/hlsopt_session.sh` 的子命令（start / baseline / attempt / good / revert / finish），
> 它们自带前置检查；不要手写等价的 git 命令绕过检查。

This ensures every optimization session has an isolated, reversible, traceable history without touching the team's shared history or the copy's identity.

## Philosophy

**One idea at a time.** Each attempt changes exactly one thing — one pragma, one algorithmic transformation, one loop restructuring. Never combine independent changes in a single attempt. You cannot attribute cause if multiple things changed simultaneously.

**Investigate before moving on.** After each synthesis, read the reports thoroughly. Do not just check whether latency went up or down — understand *why*. Read the loop detail table. Check II and achieved vs. target. Look for warnings in the log. If something unexpected happened, dig into it before proceeding.

**Use all available tools.** The synthesis outputs give access to BC file inspection (`<solution_dir>/.autopilot/db/a.g*.bc`), QoR XML (`syn/report/csynth.xml`), the Pragma Report, and schedule details (`.autopilot/db/<module>.verbose.sched.rpt`). Use `scripts/hls_report.sh basic-info` and `scripts/hls_report.sh synth` (see `reference/read-qor.md`) for a quick overview. In synthesis log to inspect how the pragma used in the code. Use `scripts/qor_log.py` to track the history of changes. Verify that the change had the expected effect — not just in the report, but in the IR and schedule.

**Experimental changes are valid.** Sometimes the right move is to make a change that you don't expect to improve the metric, but that tests a hypothesis about the tool's behavior. Document what you expected and what actually happened. Understanding the compiler's behavior is itself valuable.

**Do not rush to run cosim or impl.** Use csim+csynth as the inner loop. Only escalate to cosim when csynth shows meaningful improvement (≥ 10% in cycles or a structural change like II reduction). Only run impl when cosim confirms improvement. The expensive steps are a gate, not a rubber stamp.

**Verify Vitis HLS 2022.2 environment.**

```bash
source /tools/Xilinx/Vitis_HLS/2022.2/settings64.sh
which vitis_hls && echo "XILINX_HLS=$XILINX_HLS"
```

Do not proceed until `vitis_hls` (2022.2) is on PATH. There is no `vitis-run` in 2022.2.

## Step 0: Establish baseline and get HLS design data

### 0a. Establish a baseline

Before making any change, ensure you have a measured baseline to compare against. The baseline consists of:

1. **cosim latency** (cycles) from cosimulation report
2. **post-route clock period** (ns) from `scripts/hls_report.sh impl` (`postRoute`; must come from `export_design -flow impl`, never from a `-flow syn` post-synthesis number)
3. **true latency** = cycles × period
4. **resource utilization** (LUT, FF, DSP, BRAM, URAM) from `scripts/hls_report.sh impl` (`placeRouteResourceSummary`)
5. **golden outputs**（新增，逐位一致门的参照）: the baseline's csim outputs and cosim outputs, saved as the golden set (see §0e)

When you establish the baseline, you must make sure the csimulation, synthesis, cosimulation and implementation flows have all been run successfully. If any of the reports is missing or stale, run the corresponding flow with `scripts/hls_run.sh` (see `reference/run-flow.md`) before recording the baseline.

### 0b. Get HLS design reports and data

**MANDATORY**: You MUST use `scripts/hls_report.sh` to get ALL report-related data. **DO NOT** read `.rpt` files piecemeal with file read tools — always use this entry point so every reading uses the same extraction (details: `reference/read-qor.md`).

- `scripts/hls_report.sh synth <solution_dir>`: synthesis report (`synthesisReportContent`), Pragma Report incl. *Ignored Pragmas* (`synthesisPragmaReportContent`), filtered synthesis log (`synthesisLogContent`)
- `scripts/hls_report.sh cosim <solution_dir>`: cosimulation report (`coSimulationReportContent`) and dataflow process/channel content (`dataflowProcessAndChannelContent`)
- `scripts/hls_report.sh impl <solution_dir>`: `postRoute`, `placeRouteResourceSummary`, `placeRouteFailFast`

Raw material the script does not cover (`verbose.sched.rpt`, raw `Pipelining` blocks in `<solution_dir>/<solution>.log`) may be read directly at the 2022.2 paths in `reference/read-qor.md`; cite the path.

### 0c. Run HLS flows if reports are missing or stale

If these reports do not exist or are stale, run the flow with `scripts/hls_run.sh <proj_dir> <solution> <step>...` (`reference/run-flow.md`).
上游此处要求检查 cfg 里的 `csim.profile_tripcount=1`（用 csim 剖析出可变循环的 trip count）。2022.2 没有这个 cfg 项；
最接近的是 `csim_design -profile`。对 trip count 依赖运行时参数的循环，确保报告里的 trip count 不是 `?`：
用 `-profile` 或在源码加 `#pragma HLS loop_tripcount min=.. max=.. avg=..`。When this changes the trip counts, you must run all flows (csim, csynth, cosim, impl) again before recording the baseline.

### 0d. Log the baseline QoR data for future comparison
On the unmodified kernel first.

The script is `scripts/qor_log.py` (path relative to the skill root; run `python3 scripts/qor_log.py --help` for its exact arguments — the script layer is maintained separately).
Upstream usage was `python3 qor_log.py --report "$synthesisReportContent" [--label ...] [--csv ...]` to print a concise baseline summary.
Record the baseline before starting optimization iterations. The baseline's git anchor is the `hlsopt/<模块>/base` tag created by
`session/hlsopt_session.sh start` (architect flow: `hlsopt/<模块>/arch` from `baseline`); ledger updates are committed with the next attempt.

### 0e. Build the golden set（新增：逐位一致门）

上游以 csim 的 PASS 作为正确性门、cosim 只比较周期。这里加强为**逐位一致**：

1. 基线 csim 跑完后，把 testbench 写出的全部输出复制到 golden 目录的 csim 份；基线 cosim 跑完后，同样存 cosim 份。
   golden 目录与输出文件约定以 `scripts/bitexact.md` 为准（建议放在账本目录下 `<ledger>/golden/{csim,cosim}/`；
   golden 是构建产物，**不提交**进 git，收尾时随账本目录一起存档）。
2. 基线自检：`python3 scripts/bitexact_check.py <golden_csim> <golden_cosim>`——基线自己的 csim 与 cosim 输出若已不一致，
   说明 testbench / 设计里有 C 与 RTL 语义不一致（未初始化、浮点收缩、越界等），**先停下报告**，不得在这种基线上做优化。
3. 仪器对照：至少做一次阳性对照，证明 `bitexact_check.py` 会报失败（例如拿 golden 的一个副本改一个字节去比），
   否则"一致"没有判别力。
4. testbench 的输出必须是确定的（固定随机种子、不含时间戳/地址）；否则先改 testbench 并重建基线。

## Step 1: Analyze the kernel

You must run `scripts/hls_report.sh basic-info <solution_dir>` to get the basic component info, including `component_name`, `source_files`, `testbench_files`, `include_paths`, `top_function`. Read this information along with the `synthesisReportContent` and `synthesisLogContent` to analyze the kernel and identify bottlenecks.

- **High II on inner loops** → memory port bottleneck or long recurrence chain. Check loop table for `II achieved` vs `II target`. Read the warning messages in `synthesisLogContent` for the root cause.
- **Large iteration latency on unpipelined loops** → missing pipeline pragma, or loop not eligible for pipelining (e.g. variable trip count, non-trivial control flow, contains sub-loop).
- **Low resource utilization** → opportunity to trade area for performance (unrolling, partitioning). DSP at 2% means 98% of compute is still sequential.
- **High resource utilization** → risk of fit; verify after every change.
- **Sequential outer loop containing pipelined inner loop** → inner pipeline restarts on every iteration, dominant latency term.

Also read the Violation Report in the `synthesisLogContent` carefully — it identifies exactly which dependency causes each II violation, including the variable name and memory port.
For any loop whose achieved II misses its target, run the diagnosis in `reference/ii-diagnose.md` before choosing a technique (2022.2 messages: `HLS 200-880` carried dependence, `HLS 200-885` limited memory ports, `HLS 200-887` timing).

## Step 2: Form a hypothesis

Before touching any code, write down:

1. **What you will change** (exact pragma or code transformation)
2. **What you expect to happen** in the `synthesisReportContent` (loop II, latency, resources)
3. **What would confirm the hypothesis** (e.g. "II drops from 5 to 2 because we have 2 memory ports now")
4. **What would falsify it** (e.g. "II stays at 5 despite partition → the bottleneck is the FMA recurrence, not memory")

This forces you to understand the compiler before running it, and makes failed experiments informative.

## Step 3: Choose one optimization technique

Pick one technique per attempt. See the full technique catalogue below.

### Pragma-only changes (lowest risk, try first)

| Technique | Pragma | Effect | When to use |
|---|---|---|---|
| Acknowledge II | `#pragma HLS PIPELINE II=<N>` | Prioritize II over satisfying timing pressure | II violation in csynth report (to acknowledge the real achievable II) |
| Loop unroll (partial) | `#pragma HLS UNROLL factor=<N>` | Replicates loop body N times, reduces trip count by N | Inner loop with known trip count divisible by N |
| Loop unroll (full) | `#pragma HLS UNROLL` | Fully unrolls loop; all iterations in parallel | Short loops (≤ 16 iterations), sufficient resources, constant loop trip count |
| Array partition (cyclic) | `#pragma HLS ARRAY_PARTITION variable=<x> cyclic factor=<N> dim=<D>` | Creates N memory banks; N parallel accesses per cycle | Memory port II bottleneck |
| Array partition (complete) | `#pragma HLS ARRAY_PARTITION variable=<x> complete dim=<D>` | Fully registers the array (no BRAM) | Small arrays only |
| Array partition (block) | `#pragma HLS ARRAY_PARTITION variable=<x> block factor=<N> dim=<D>` | Splits into N contiguous blocks | When access pattern is block-sequential |
| Loop flatten | `#pragma HLS LOOP_FLATTEN` | Merges nested loops into a single loop counter | Nested loops with loop-invariant bounds |
| Dataflow | `#pragma HLS DATAFLOW` | Task-level pipelining between functions or loops | Sequential stages that can overlap, non-innerloop pipelining |
| Expression balance | `#pragma HLS EXPRESSION_BALANCE` | Allow expression tree balancing on float | Floating point reductions in critical path |

### Algorithmic changes (higher risk, higher reward)

| Technique | Effect | When to use |
|---|---|---|
| Loop reordering | Changes memory access pattern; may improve spatial locality or expose better reuse | Poor memory reuse in inner loop; can turn random access into sequential |
| Tiling / blocking | Reuses on-chip data, reduces off-chip bandwidth, make buffer size constant | Memory bandwidth bound, variable size data to chunk into fixed size buffer |
| Local buffer | Stage data into local arrays to decouple memory access from compute | Repeated accesses to same memory region |
| Scalarization | Store data into individual scalars to allow arbitrary accesses and additional simplifications | Repeated accesses to the same memory element; can allow infinite access per cycle |
| Loop fusion | Merge two loop body into the same loop to reduce storage and allow for pipelining | Two consecutive loops with identical loop trip count working on the same buffers |
| Loop fission | Split the loop body into two separate loop to create independent processes in dataflow context | Loop body working on independent arrays/streams causing stall or complex control |
| Replace `float` with `ap_fixed<W,I>` | Shorter pipeline, fewer DSPs, faster clock | Fixed-point arithmetic acceptable for the application. **会改变数值，逐位一致门必然失败**：只有用户事先明确允许数值变化时才可尝试，并按 SKILL.md H1 重建 golden、在账本写明 |
| Replace division with reciprocal multiply | Eliminates expensive divider | Division inside a loop by constant divisor |

### HLS library usage

```cpp
#include <ap_int.h>              // ap_int<N>, ap_uint<N> — arbitrary-width integers
#include <ap_fixed.h>            // ap_fixed<W,I> — fixed-point arithmetic
// #include <ap_float.h>         // ap_float<W,E> — 2022.2 不可用（头文件不存在，include 即报 HLS 207-812）
#include <hls_math.h>            // hls::sqrt, hls::exp, etc. — HLS-optimized math
#include <hls_vector.h>          // hls::vector<T,N> — SIMD-style vectorized types
// #include <hls_array_partition.h> // hls::scatter / hls::gather — 2022.2 不可用（头文件不存在）；
//                                  // 替代：#pragma HLS ARRAY_PARTITION + 手写 unroll 循环做分散/聚集
#include <hls_stream.h>          // hls::stream<T> — FIFO communication between tasks
#include <hls_burst_maxi.h>      // hls::burst_maxi — high-performance M-AXI burst（2022.2 可用，已实测综合通过）
#include <hls_task.h>            // hls::task + hls_thread_local（2022.2 可用）
#include <hls_streamofblocks.h>  // hls::stream_of_blocks（2022.2 可用）
```

2022.2 pragma 差异（实测，详见 `../porting_notes.md`）：`#pragma HLS performance target_ti=` 只在循环体内生效；
`#pragma HLS pipeline rewind=true|false` 是语法错误，整条 pragma 被丢弃（只认裸 `rewind`）；
`#pragma HLS array_stencil` 虽是隐藏选项但可用；`#pragma HLS inline region` 已弃用。

## Step 4: Implement the change

Use `reference/optimize/attempt-template.md` to launch a sub-agent for each attempt. Fill in all bracketed fields before launching.

Key rules for the sub-agent:
- Make **only** the described change. Do not opportunistically fix other things.
- After each tool run, **read the output carefully** before deciding to proceed.
- If csynth warnings or loop table do not match the hypothesis, investigate before escalating.

## Step 5: Investigate the result thoroughly

For `run csynth`, do not just check the summary. Investigate systematically:

### 5a. Check for ignored-pragma warnings in the HLS log

Do this first — before reading any report. HLS frequently silently ignores pragmas that cannot be applied and emits a warning explaining why. If a pragma was ignored, the change you intended was **not applied** and the csynth result reflects the unmodified schedule. Reading the reports without knowing this leads to wrong conclusions.

First read the **Ignored Pragmas** table in the Pragma Report (`synthesisPragmaReportContent`; 2022.2 `csynth.rpt` has it). Then check `pragma\|ignored\|cannot\|not applied\|warning` in `$synthesisLogContent` for any ignored pragma warnings.

2022.2 actual message forms seen on this machine:
- `WARNING: [HLS 207-5554] unexpected pragma parameter 'bogus_opt'` — unknown option
- `WARNING: [HLS 207-5555] unexpected pragma argument '=', expects identifier` — malformed (e.g. `rewind=true`); **the whole pragma is dropped**
- `WARNING: [HLS 207-5572] unknown HLS pragma ignored`
- `WARNING: [HLS 207-5562] performance pragma is ignored, because it is not in a loop`
- `WARNING: [HLS 207-5528] 'region' in '#pragma HLS Inline' is deprecated`

Optionally add `config_compile -pragma_strict_mode=true` to `run_hls.tcl` so malformed pragmas become errors (verified: turns 207-5555 into an ERROR).

Common ignored-pragma patterns to look for:

| Warning text (approximate) | Meaning |
|---|---|
| `pragma HLS PIPELINE ignored` | Loop is not eligible for pipelining (e.g. contains a sub-loop, variable trip count, or non-trivial control flow). |
| `pragma HLS UNROLL factor=N ignored` | Loop body cannot be fully unrolled (e.g. trip count not statically known, or factor does not divide trip count). |
| `pragma HLS ARRAY_PARTITION … ignored` | Variable is not a local array, or partition dim exceeds array rank. |
| `pragma HLS LOOP_FLATTEN ignored` | Loop bounds are not loop-invariant — flattening would change semantics. |
| `pipeline II target … cannot be achieved` | The target II was set but the scheduler could only achieve a higher value. Check the Violation Report for root cause. |

If a pragma was ignored, fix the pragma or restructure the code and re-run before drawing any conclusions from the reports.

### 5b. Read the synthesis report

Use the `synthesisReportContent` to check the loop table for the loops of interest. Look at II achieved vs. target, iteration latency, and resource usage.

Check:
- Did II change on the expected loop?
- Did II change on any other loop (unexpected side effect)?
- Did total latency change by the expected factor?
- Did resource usage change? By how much?

Also check for general synthesis warnings:

check `warning\|critical\|fail\|error` in `synthesisReportContent` and `synthesisLogContent` for any synthesis warnings.

### 5c. Read the Violation

Check violation in `synthesisLogContent` for any II or Timing violations.

### 5d. Read the inferred directives

2022.2 has **no** `inferred_directives.ini`. Equivalent evidence of auto-applied directives:

- `INFO` lines in the raw solution log, e.g. `[XFORM 203-510] Pipelining loop '...' automatically.`,
  `[HLS 214-268] Inferring pragma 'unroll factor=2' ... due to performance pragma`,
  `[HLS 214-269] Inferring pragma 'pipeline II=1' ...`, `[HLS 214-291] Loop '...' is marked as complete unroll implied by the pipeline pragma`
- `<solution_dir>/.autopilot/db/all.directive.json` and `fe_pragma_dump.reflow.0.xml` (what the front end saw)
- `config_compile -pipeline_loops` (default 64: loops with trip count above this are auto-pipelined)

Check whether HLS auto-applied any directives you didn't expect. Array partitions, loop flattening, pipeline pragmas can be inferred automatically and can interact with your pragma.

### 5e. Verify transformation in synthesis log (when hypothesis is about compiler behavior)

When you want to verify that HLS actually applied the transformation you intended (e.g., that a partition was respected, or that a loop was actually unrolled), search `synthesisLogContent` for key indicators.

**Search patterns by pragma type:**

| Pragma | Search pattern in `synthesisLogContent` | Confirmation |
|--------|----------------------------------------|--------------|
| `ARRAY_PARTITION` | `partitioned\|partition.*factor\|partition.*complete` | Look for "partitioned into N banks" or similar messages |
| `UNROLL factor=N` | `unroll\|unrolled\|trip count` | Look for "loop completely unrolled" or "loop unrolled by factor N" |
| `LOOP_FLATTEN` | `flatten\|flattened\|merged` | Look for "loop flattened" or "loops merged" messages |
| `PIPELINE` | `pipeline\|pipelined\|II=` | Look for "loop pipelined with II=N" |
| `DATAFLOW` | `dataflow\|FIFO\|channel` | Look for dataflow region and channel creation messages |

**Example grep commands:**

```bash
# Check if partition was applied
echo "$synthesisLogContent" | grep -i "partition"

# Check if unroll was applied
echo "$synthesisLogContent" | grep -i "unroll"

# Check if pipeline achieved target II
echo "$synthesisLogContent" | grep -i "pipeline.*II"

# General transformation summary
echo "$synthesisLogContent" | grep -iE "applied|transformed|optimized|inferred"
```

**Key things to look for:**
- After `ARRAY_PARTITION`: messages confirming partition into N banks, or warnings if partition was not applied
- After `UNROLL factor=N`: messages showing "unrolled by factor N" or "completely unrolled"
- After `LOOP_FLATTEN`: messages showing nested loops were merged into a single loop
- After `PIPELINE`: messages showing achieved II value

**If the expected transformation message is not found**, the pragma may have been ignored — cross-reference with Step 5a for ignored-pragma warnings.

### 5f. Log the result

```bash
python3 scripts/qor_log.py ...   # upstream form: --report "$synthesisReportContent" --label "<technique-applied>" --csv <ledger>/qor_history.csv
```

This appends a row to `<ledger>/qor_history.csv` for tracking across attempts (`<ledger>` = the ledger dir given to `hlsopt_session.sh start`, default `<first source dir>/.memory`).

### 5g. Bit-exact gate after csim（新增，硬门）

csim 跑完（且 testbench 自己 PASS）后**立刻**：

```bash
python3 scripts/bitexact_check.py <ledger>/golden/csim <this attempt's csim output dir>
```

不一致 ⇒ 本次尝试**判失败**：不进入 csynth 结论、不升级 cosim；按 Step 6 记账、`attempt --status fail` 提交、`revert`。
升级到 cosim 后，同样对 cosim 输出跑一次 `bitexact_check.py <ledger>/golden/cosim <cosim output dir>`，不一致同样判失败
（即使 cosim 报 Pass、周期变好）。比较口径以 `scripts/bitexact.md` 为准。

## Step 6: Record the result — MANDATORY after every attempt

**Do this before deciding whether to escalate or start the next attempt. Never skip this step.**

### 6a. Update `<ledger>/optimize_outcomes.md`

Append a row to the outcomes table (`<ledger>/optimize_outcomes.md`) immediately after reading the csynth report. Use the present attempt's data — do not wait until the end of the session. Include:
- Technique name
- Observed effect (cycles before → after, II change)
- Whether the hypothesis was confirmed
- Bit-exact result: csim yes/no, cosim yes/no/not-run
- Any unexpected behavior, side effects, or lessons

This file is read at the start of every session. If it is not updated after each attempt, the next session will repeat experiments that have already been run.

### 6b. Commit the state — MANDATORY, including for failed attempts

**Every attempt gets its own commit before anything else happens — including attempts that did not improve the metric.** The commit must happen *before* any revert, file edit, or next-attempt work. Skipping the commit on failure loses the experiment from history and is the most common mistake in this workflow.

> 团队仓库改造：提交只发生在 `hlsopt_session.sh start` 建的实验分支上（脚本拒绝在非 `exp/` 分支、`main`、他人分支、
> 或实验区外执行）。回退不再用上游的 `git checkout HEAD~1 -- <file>`（在共享副本里会覆盖他人未提交修改；连续失败时
> HEAD~1 也未必是好版本），改为回到**最近的 good 标签**（没有则 `arch` / `base` 标签）。

Order of operations after every attempt (no exceptions):
1. Update `<ledger>/optimize_outcomes.md` (Step 6a).
2. Commit via the session script (this step) — even if the build broke, the bit-exact gate failed, or the result was worse.
3. *Only then* may you revert, adapt, or move on.

`S=tools/agent_skills/fpga-hls/session/hlsopt_session.sh`（在实验区里，相对仓库根）。

**If the attempt passed the bit-exact gate AND improved the target metric (or confirmed a useful hypothesis and you keep the code):**

```bash
MSG=$(mktemp)
cat > "$MSG" <<'MSG_END'
<technique>: <what changed> → <observed effect>

Hypothesis: <what you expected>
Observed: <what actually happened in csynth/cosim/impl>
Bit-exact: csim <yes>, cosim <yes|not run>
Baseline: cosim=<X> cycles, post-route=<Y> ns, true latency=<Z> ms
After:    cosim=<X'> cycles, post-route=<Y'> ns, true latency=<Z'> ms
Resources: LUT <A>→<A'>, DSP <B>→<B'>
MSG_END
bash "$S" attempt --status pass --label "<technique>" --message-file "$MSG"
bash "$S" good --bitexact pass        # tags hlsopt/<模块>/good-<N>
```

Only tag `good` when **both** hold: bit-exact (csim, and cosim if it was run) and the metric improved / the change is kept.

**If the attempt did not help, got worse, or failed the bit-exact gate — commit first, then revert:**

```bash
MSG=$(mktemp)
cat > "$MSG" <<'MSG_END'
experiment(<technique>): <what changed> — did not help

Hypothesis: <what you expected>
Observed: <what actually happened — bit-exact mismatch / QoR worse / build broke, and why>
MSG_END
# 1. Commit the failed state as-is (code + ledger); empty commits allowed, as upstream
bash "$S" attempt --status fail --label "<technique>" --message-file "$MSG"
# 2. The script verifies the commit landed (sources + ledger clean afterwards) and refuses otherwise.
#    If it refuses, STOP and investigate — do not revert.
# 3. Only after the commit is confirmed, revert sources to the latest good tag (or arch/base):
bash "$S" revert
```

`--allow-empty` is kept (inside the script) so the commit still lands even if the only thing that changed was a file that has since been auto-regenerated to its prior content. `revert` refuses while the sources still have uncommitted changes — that is the guard that the failed attempt was preserved first; it then checks the sources out of the latest good tag and commits that revert, so every attempt commit on the experiment branch is a clean single-change diff.

Committing before reverting preserves the exact failing code in history, making it possible to inspect, diff, or re-apply it later. A failed experiment that teaches you something is not wasted work.

## Step 7: Decide whether to escalate

| csynth result | Action |
|---|---|
| Latency improved ≥ 10% OR II reduced on a dominant loop | Escalate: run cosimulation flow |
| Latency unchanged, but hypothesis was confirmed (e.g. II matched expected value) | Valid data. Move on to the next hypothesis. |
| Latency got worse | Investigate why, then **commit (Step 6b) before reverting**. The result may reveal a real constraint. |
| csim FAIL | Fix logic error or revert immediately. |
| csim PASS but bit-exact vs golden FAIL | Attempt failed — commit (6b) then revert. Do not escalate. |

For `run cosim`: run the bit-exact check on the cosim outputs first (§5g); any mismatch fails the attempt. Then run `scripts/hls_report.sh cosim <solution_dir>` to get `coSimulationReportContent` and `dataflowProcessAndChannelContent`. Compare cosim cycles to csynth estimate. For designs with memory backpressure, cosim can be significantly worse than csynth estimates; for compute-bound kernels, they should be close.

For `run impl`: only run when cosim confirms latency improvement **and** cosim is bit-exact. Run `scripts/hls_report.sh impl <solution_dir>` and use `postRoute` for the actual post-route clock period and compute true latency = cosim_cycles × period_ns.

**DO NOT** read `.rpt` files directly using file read tools to extract QoR numbers. Always use `scripts/hls_report.sh` to ensure consistency and correctness.

## Step 8: Update skills with lessons learned

After each attempt — successful or not — note findings that are general enough to apply to future kernels. In this repo the skill text lives in
`tools/agent_skills/fpga-hls/` (SKILL.md / reference/*.md) and is version-controlled: write the lesson into the ledger first; changing skill files is a
separate, user-approved commit on a normal feature branch (never on the hls-opt experiment branch):

- If a technique had an unexpected interaction with HLS (e.g., auto-inferred flatten conflicted with your pipeline pragma), note it.
- If the Violation Report pointed to a root cause that wasn't obvious from the loop table, note the pattern.
- If the IR inspection revealed something unexpected (e.g., the partition was not reflected in the IR), note it.

## Cost model for the inner loop

| Step | Typical runtime | When to run |
|---|---|---|
| `run csim` + bit-exact check | 5–30 s | Always — correctness gate (bit-exact vs golden) |
| `run csynth` | 10–60 s | Always — the main investigation tool |
| `run cosim` + bit-exact check | 30 s – 5 min | When csynth shows ≥ 10% improvement in latency cycles, or a structural improvement (II reduction on dominant loop) |
| `run impl` | 1 – 30 min | When cosim confirms improvement |

## Stopping criteria

Stop iterating when any of the following is true:
- The optimization criteria is met.
- Three consecutive attempts produced no improvement, and no new hypothesis remains to test.
- Resource utilization is approaching device limits (> 60% of any constrained resource) and further improvement requires more resources.
- The improvement per attempt has diminished to < 5% of baseline.

When stopping criteria are met, summarize the lessons learned, then close the session.

### Finish（团队仓库改造）

```bash
SUM=$(mktemp)
cat > "$SUM" <<'SUM_END'
Latency: <baseline> -> <final> (cosim cycles; true latency ms)
II: <baseline> -> <final> (dominant loop)
Resources: LUT <a>-><a'>, FF <b>-><b'>, DSP <c>-><c'>, BRAM <d>-><d'>
Bit-exact: csim yes, cosim yes (golden = baseline at hlsopt/<模块>/base)
SUM_END
bash "$S" finish --summary-file "$SUM" [--best hlsopt/<模块>/good-<N>] [--remove-worktree]
```

`finish` 依次：
1. `git format-patch hlsopt/<模块>/base..HEAD` + 账本目录（含 golden）+ SUMMARY + MANIFEST + SHA256SUMS 存档到
   `<artifacts>/<YYYYMMDD>-<本人>-hls-opt-<模块>/`（不覆盖已有目录；补丁数须等于提交数；sha256 自校验）；
2. 回到用户功能分支（原副本必须仍在开工时的分支上、被优化文件无未提交修改、开工后该分支没改过这些文件，否则拒绝并提示人工
   `merge --squash`），只取最佳 good 标签的**源码**做**一次**提交，提交信息含尝试次数、QoR 三元组、逐位一致结论、存档路径；
3. 实验分支与 `hlsopt/<模块>/*` 标签保留在本地，**永不推送**，删不删由用户定；
4. 只有加 `--remove-worktree`、存档已成功、实验区无未跟踪内容时才移除 worktree（有 HLS 构建产物时不强删，交用户决定）。

没有任何 good 标签时 `finish` 拒绝提交；把结论告诉用户，实验区保留。

**In interactive mode**: ask the user if they want to update the relevant `SKILL.md` with the new insights. The update role is described in step 8 above.

**In non-interactive mode** (invoked via `claude --print`): skip the prompt — do NOT update any skill file. Print the summary and exit.

## Known outcomes

Outcomes are recorded in `<ledger>/optimize_outcomes.md` on the experiment branch, and archived under `shared/artifacts/<YYYYMMDD>-<本人>-hls-opt-<模块>/ledger/` at finish.
Read the latest one (and earlier archives for the same module) at the start of each session to avoid repeating experiments.
