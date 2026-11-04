> 来源：AMD Ross `hls-perf-pragma`（作者：Sai Akhil Ayyagari），MIT 许可，见 `../upstream.md`。已按 Vitis HLS 2022.2 移植。
> 上游描述：Vitis HLS Performance Pragma — calculate target_ti from a throughput target, cascade through architecture and loops, and present the pragma placement table for user confirmation. Run this before placing any #pragma HLS performance.
<!--
Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
-->
# Skill: perf-pragma

> **2022.2 关键差异（本机 csynth 实测，证据见 `../porting_notes.md` §2）——本文件已按此改写：**
> 1. `#pragma HLS performance` **只在循环体内生效**。放在函数体里（含顶层函数）、或放在"循环标号与 `for` 之间"，
>    都会被忽略：`WARNING [HLS 207-5562] performance pragma is ignored, because it is not in a loop`，综合照常成功。
>    上游的"Position 1（顶层函数体）"与"Position 2（标号与 for 之间）"两种放法在 2022.2 **都无效**。
> 2. 唯一合法位置是**循环体的第一行**（`for (...) {` 之后）。嵌套循环里外层、内层都可放：实测外层 `target_ti=4096`、
>    内层 `target_ti=128`（trip count 64）时，工具把内层转成 `pipeline II=2`（`HLS 214-269`）、对外层推断 `loop_flatten`
>    （`HLS 214-339`），并报 `HLS 200-1957 Successfully applied performance pragma with Target TI='4096' on loop 'Row_Loop_Col_Loop'`。
> 3. 因此"函数级预算"（顶层或 dataflow 各阶段）一律落到**该函数最外层循环**的循环体里；没有循环的函数无法用本 pragma 约束。
> 4. 公开选项只有 `target_ti`、`target_tl`（无 `unit=`；HLSPragma.json 另有标 Hidden 的 `assume_ti`/`assume_tl`/`scope`，不要用）；`target_ti` 单位是时钟周期。
> 5. 时钟与顶层函数从 `run_hls.tcl`（`create_clock -period`、`set_top`）读，2022.2 没有 HLS cfg 文件。
> 6. 生效判据：日志里每条 pragma 有 `HLS 200-1957 Successfully applied` 或被转成的 `HLS 214-269`，且
>    `syn/report/csynth.rpt` 的 "Performance Pragma Report" 列出该循环、"Ignored Pragmas" 表里没有它。综合成功本身不证明生效。

Calculate `target_ti` from the user's throughput target and produce a cascade table for review. This skill is a pure calculator — it does not run synthesis or interpret reports.

**Throughput target:** $ARGUMENTS

Print at the start:
```
─────────────────────────────────────────────────────
[perf-pragma]  flow overview
  Steps 0 → 5
  Step 0  — Gather inputs (clock, architecture, loop structure)
  Step 1  — Calculate top-level target_ti
  Step 2  — Cascade through architecture
  Step 3  — Cascade through loops → build table
  Step 4  — Present table for review
  Step 5  — Apply pragmas to source code
─────────────────────────────────────────────────────
```

---

---

## Step 0: Gather inputs

The throughput requirement comes from $ARGUMENTS.

**Read the configuration and source code to determine:**

1. **Clock frequency** — Read from the design's `run_hls.tcl` (2022.2 Tcl project; there is no HLS cfg file):
   ```bash
   TCL_FILE=run_hls.tcl      # or the .tcl found by reference/architect/setup.md Step 2
   CLOCK_NS=$(grep -E "^\s*create_clock" "$TCL_FILE" | grep -oE -- "-period +[0-9.]+" | awk '{print $2}')
   clock_hz = 1e9 / CLOCK_NS
   ```
   If the project only has `hls.app` (no Tcl), use `scripts/hls_report.sh basic-info <solution_dir>` or ask the user.
   Note: `create_clock -period` may also be given in MHz form (`-period 200MHz`) — convert to ns.

2. **Architecture type** — Inspect the top-level function:
   - Look for `#pragma HLS dataflow` or `hls::task` → dataflow pipeline
   - Multiple function calls in sequence → sequential functions
   - Single pipelined function otherwise

3. **Loop structure** — Identify loop nesting levels and trip counts:
   - Extract all labeled loops (e.g., `Row_Loop:`, `Col_Loop:`)
   - Determine nesting depth
   - **Trip counts:** Read loop bounds from source code   
   - **If trip counts are runtime parameters (not compile-time constants):**: **warn the user** : "Performance pragma requires loop trip counts. Please specify the trip count values for: [list loops with variable bounds]"

4. **Throughput conversion** — If throughput is given as data rate (MB/s, GB/s):

   **Read source code to determine input data size:**
   
   ```bash
   # Find top function (from run_hls.tcl set_top)
   TOP_FUNCTION=$(grep -E "^\s*set_top" "$TCL_FILE" | awk '{print $2}')
   
   # Find source file containing top function
   SRC_FILE=$(grep -l "void $TOP_FUNCTION" src/*.cpp)
   ```
   
   Read the function signature. The function will have multiple inputs/outputs. **Identify the input arrays** and calculate:
   
   ```
   bytes_per_invocation = total_input_array_size_in_bytes
   ```
   
   **Convert data rate to invocations per second:**
   
   ```
   data_rate_bytes_per_sec = throughput_value × (1,000,000 for MB/s, 1,000,000,000 for GB/s)
   invocations_per_sec = data_rate_bytes_per_sec / bytes_per_invocation
   ```
   
   Use this `invocations_per_sec` value in Step 1.

5. **Computational throughput** — If throughput is given as GOPS or GFLOPS:

   ```bash
   # Parse value (e.g., "100 GOPS" → 100e9 ops/sec)
   THROUGHPUT_VALUE=$(echo "$ARGUMENTS" | grep -oE '[0-9.]+')
   OPS_PER_SEC=$(echo "$THROUGHPUT_VALUE * 1e9" | bc)
   
   # Read source code to determine operations per invocation
   # Look for dimensions in defines or comments
   
   # Convert: invocations/sec = ops/sec / ops_per_invocation
   # Note: 1 OP = 1 MAC = 2 FLOPs
   invocations_per_sec=$(echo "$OPS_PER_SEC / $OPS_PER_INVOCATION" | bc)
   ```
   
   If operation count cannot be determined, warn the user to specify it or use FPS/MB/s instead.
   
   Use the calculated `invocations_per_sec` in Step 1.

Print after Step 0:
```
─────────────────────────────────────────────────────
[perf-pragma]  Step 0 — done
  ✓ Step 0  clock=<CLOCK_NS> ns  arch=<single/sequential/dataflow>  loops=<N levels>
  ← NEXT    Step 1 — Calculate top-level target_ti
─────────────────────────────────────────────────────
```

---

## Step 1: Calculate top-level target_ti

**Formula:**
```
target_ti = clock_hz / throughput_rate
```

Where `throughput_rate` is the target invocations per second (e.g., FPS, invocations/s, samples/s).

Print after Step 1:
```
─────────────────────────────────────────────────────
[perf-pragma]  Step 1 — done
  ✓ Step 0  Inputs gathered
  ✓ Step 1  top-level target_ti = <N> cycles  (<throughput> @ <CLOCK_NS> ns)
  ← NEXT    Step 2 — Cascade through architecture
─────────────────────────────────────────────────────
```

---

## Step 2: Cascade through architecture

### Case A — Single function

The full `target_ti` applies to the one function. In 2022.2 it is placed on that function's **outermost loop** (loop body),
not on the function body. Go to Step 3.

### Case B — Sequential functions

Functions execute one after the other — total latency is the **sum**:
```
target_ti_total = target_ti_fn1 + target_ti_fn2 + ...
```
Allocate per-function budget by trip-count ratio. Apply the top-level budget first — in 2022.2 that means a pragma in the body of
each function's outermost loop, carrying that function's share; add deeper loop-scope pragmas only after synthesis confirms the bottleneck.

### Case C — Dataflow pipeline (including `hls::task`)

Throughput = **max** across all stages — the slowest stage sets the rate:
```
Design Throughput = max(TI_stage1, TI_stage2, ...)
```

`target_ti` is a **ceiling** that applies equally to every stage. A stage whose budgeter-allocated Target TI exceeds the top-level target_ti will dominate throughput and cause a miss — even if that stage shows "yes" for its own allocated target.

**Cascading strategy:**
1. A pragma on the top (dataflow) function body is ignored in 2022.2. Start instead by placing `target_ti` = top-level value
   in the body of **each stage's outermost loop** (same value for every stage, not a fraction).
2. Then cascade into that stage's inner loops where needed:
   ```
   stage_loop_II = ceil(target_ti / stage_loop_trip_count)
   ```

Print after Step 2:
```
─────────────────────────────────────────────────────
[perf-pragma]  Step 2 — done
  ✓ Step 0  Inputs gathered
  ✓ Step 1  top-level target_ti = <N> cycles
  ✓ Step 2  Architecture: <single/sequential/dataflow>  budget allocated
  ← NEXT    Step 3 — Cascade through loops
─────────────────────────────────────────────────────
```

---

## Step 3: Cascade through loops — build the table

### 3a. Classify trip counts before cascading

Inspect the source for each loop. If any trip count is variable (runtime argument or input-dependent), **stop here** and return to the caller (`reference/optimize.md` Step 1d) to run csim first. Do not guess or estimate variable trip counts.

| Type | Example | Status |
|---|---|---|
| Compile-time constant | `for (int i = 0; i < 64; i++)` | ✓ Proceed |
| Fixed template parameter | `for (int i = 0; i < N; i++)` — N known at instantiation | ✓ Proceed |
| Runtime parameter | `for (int i = 0; i < height; i++)` — height is a function arg | ✗ Run csim first |
| Input-dependent | count driven by data | ✗ Run csim first |

### 3b. Propagate the budget

Propagate the budget downward through each loop level. The formula depends on loop nesting depth:

**General pattern:** Each nested loop divides its parent's target_ti by the parent's trip count.

**Case 1: Single loop in function**
```
Loop_target_ti = Function_target_ti      (assuming negligible function overhead)
```

**Case 2: Two-level nested loops**
```
Outer_loop_target_ti = Function_target_ti                    (overhead usually negligible)
Inner_loop_target_ti = Outer_loop_target_ti / outer_trip_count
```

**Case 3: Three-level nested loops**
```
Outer_loop_target_ti  = Function_target_ti
Middle_loop_target_ti = Outer_loop_target_ti / outer_trip_count
Inner_loop_target_ti  = Middle_loop_target_ti / middle_trip_count
```


Fill this table using the known or profiled trip counts (example shows 2-level nesting):

| Level | Formula | Value |
|---|---|---|
| Function target_ti | clock_hz / throughput_rate | ? cycles |
| Outer loop target_ti | function_ti (overhead ≈ 0) | ? cycles |
| Inner loop target_ti | outer_ti / outer_trip_count | ? cycles |

### 3c. Detect scope types (function vs loop)

**Rule (2022.2):** every pragma targets a loop. The "function" row of the table is a budget row only: its `target_ti`
is placed in the body of the function's outermost loop(s) (in a dataflow region: each stage's outermost loop).
A function with no loop cannot carry a performance pragma — say so in the table and use `pipeline II` instead.

**Detection logic:**

1. **Extract TOP function** from `run_hls.tcl`:
   ```bash
   TOP_FUNCTION=$(grep -E "^\s*set_top" "$TCL_FILE" | awk '{print $2}')
   ```

2. **Find loop labels** in source files:
   ```bash
   grep -Pn "^\s*\w+:\s*for\s*\(" <source_files>
   ```
   This captures patterns like:
   - `Row_Loop: for (int r = 0; r < ROWS; r++)`
   - `Col_Loop: for (int c = 0; c < COLS; c++)`

3. **Build enhanced table** with Type column:

   | Target          | Type            | target_ti | Pragma Position                                  |
   |-----------------|-----------------|-----------|--------------------------------------------------|
   | <TOP_FUNCTION>  | function budget | <value>   | → carried by its outermost loop (Row_Loop) below |
   | Row_Loop        | loop            | <value>   | first line of loop body                          |
   | Col_Loop        | loop            | <value>   | first line of loop body                          |

   Unlabeled loops can also carry the pragma (it goes in the loop body), but label them so the reports name them.
   (Upstream's `detect_loops.sh` helper is not ported; the grep above is equivalent.)

Print after Step 3:
```
─────────────────────────────────────────────────────
[perf-pragma]  Step 3 — done
  ✓ Step 0  Inputs gathered
  ✓ Step 1  top-level target_ti = <N> cycles
  ✓ Step 2  Architecture cascaded
  ✓ Step 3  Loop table built
  ← NEXT    Step 4 — Present table and wait for confirmation
─────────────────────────────────────────────────────
```

---

## Step 4: Present cascade table and wait for confirmation

Show the completed table with **Type** and **Pragma Location** columns to clearly distinguish function-level vs loop-level pragmas:

**Enhanced Cascade Table Format:**

| Target          | Type            | target_ti | Pragma Location                                 |
|-----------------|-----------------|-----------|-------------------------------------------------|
| <TOP_FUNCTION>  | function budget | <value>   | carried by outermost loop Row_Loop (no pragma in function body) |
| Row_Loop        | loop            | <value>   | first line of loop body                         |
| Col_Loop        | loop            | <value>   | first line of loop body                         |

**Placement guidance (2022.2 — only one valid position):**
- **Every pragma goes on the first line inside the loop body:**
  ```cpp
  Row_Loop: for (int r = 0; r < ROWS; r++) {
      #pragma HLS performance target_ti=<value>
      // loop body
  }
  ```
- **Never** in a function body (top-level or stage function) and **never** between the label and `for` — both are ignored with
  `WARNING [HLS 207-5562] ... not in a loop` while synthesis still succeeds.

Do not place any pragma until the gate above resolves (auto in demo, user-confirmed in interactive).

Print after presenting the table:
```
─────────────────────────────────────────────────────
[perf-pragma]  Step 4 Complete — Cascade Table Presented
  ✓ Step 0  Inputs gathered
  ✓ Step 1  top-level target_ti = <N> cycles
  ✓ Step 2  Architecture cascaded
  ✓ Step 3  Loop table built with Type detection
  ✓ Step 4  Cascade table presented
  ← NEXT    Step 5 — Apply pragmas to source code
─────────────────────────────────────────────────────
```

---

## Step 5: Apply performance pragmas to source code

**This step applies ALL pragmas from the cascade table built in Steps 1-3.**

### Step 5a: Locate source file

Find the file containing the top-level function:

```bash
TOP_FUNCTION=$(grep -E "^\s*set_top" "$TCL_FILE" | awk '{print $2}')
SRC_FILE=$(grep -l "void $TOP_FUNCTION" src/*.cpp)
echo "Top function: $TOP_FUNCTION"
echo "Source file: $SRC_FILE"
```

Read the file to understand its current structure before editing.

### Step 5b: Function budgets → outermost loops

**Target**: every row of Type="function budget" (top-level function; in a dataflow region, each stage).

**Location**: first line inside the body of that function's **outermost loop**. Do **not** put it in the function body —
2022.2 ignores it there (`HLS 207-5562`). If the function has several top-level loops in sequence, split the budget between
them by trip-count ratio (Case B); if it has none, report "no loop — cannot carry performance pragma" and skip.

**Example** (TOP_FUNCTION=colordetect_accel with outermost loop `Row_Loop`, TOP_LEVEL_TARGET_TI=68863):

**BEFORE**:
```cpp
void colordetect_accel(...) {
    Row_Loop: for (int r = 0; r < rows; r++) {
        // loop body
```

**AFTER**:
```cpp
void colordetect_accel(...) {
    Row_Loop: for (int r = 0; r < rows; r++) {
        #pragma HLS performance target_ti=68863
        // loop body
```

(Upstream's example put the pragma in the function body before an `xf::cv::Mat` declaration; in 2022.2 that pragma is ignored.
For a top built only from `xf::cv` library calls without loops of its own, there is no loop in our code to carry the pragma;
set throughput through the library's own parameters (e.g. NPPC) instead, and say so in the cascade table.)

### Step 5c: Apply loop-level pragmas (inner loops)

**Target**: All remaining loops from the cascade table where Type="loop"

**Location**: first line inside the loop body (after `{`). **Not** between the loop label and `for` (ignored in 2022.2).

**Pragma to add**:
```cpp
        #pragma HLS performance target_ti=<LOOP_TARGET_TI>
```

**Example** (Col_Loop has target_ti=63):

**BEFORE**:
```cpp
    Col_Loop: for (int c = 0; c < cols; c++) {
        // loop body
    }
```

**AFTER**:
```cpp
    Col_Loop: for (int c = 0; c < cols; c++) {
        #pragma HLS performance target_ti=63
        // loop body
    }
```

**Repeat for ALL loops** in the cascade table.

### Step 5d: Verify pragmas applied correctly

Read the modified source file and confirm:
- ✓ No `#pragma HLS performance` directly in any function body (top-level or stage)
- ✓ Every performance pragma is the first line inside a loop body, none between a label and `for`
- ✓ Syntax: `#pragma HLS performance target_ti=<number>`
- ✓ No typos (target_ti not target_ii); no `unit=` option (does not exist in 2022.2)
- ✓ After the caller's next csynth: each pragma shows `HLS 200-1957 Successfully applied` (or its converted
  `HLS 214-269 Inferring pragma 'pipeline II=...'`), appears in csynth.rpt "Performance Pragma Report", and is absent from
  "Ignored Pragmas". Any `HLS 207-5562` warning = misplaced pragma → fix before reading QoR (SKILL.md H4).
- ✓ Indentation matches surrounding code

### Step 5e: Return to caller

**DO NOT commit here.** The calling flow (`reference/architect.md` / `reference/optimize.md`) commits via
`session/hlsopt_session.sh attempt` (or `baseline` for the first architect commit).

Return the following information to the caller:
- TOP_FUNCTION_NAME
- TOP_LEVEL_TARGET_TI
- List of modified files (src/*.cpp)
- Cascade table (for commit message)

Print completion:
```
─────────────────────────────────────────────────────
[perf-pragma]  Step 5 Complete — Pragmas Applied to Source
  Function       : $TOP_FUNCTION
    Pragma       : #pragma HLS performance target_ti=$TOP_LEVEL_TARGET_TI
    Location     : outermost loop body (function-body pragmas are ignored in 2022.2)
  
  Loops          : <N> loops
    Pragma       : target_ti values from cascade table
    Location     : first line of each loop body
  
  Modified files : $SRC_FILE
  Status         : Pragmas applied — ready for commit by caller
─────────────────────────────────────────────────────
```

**The calling flow is responsible for**:
- Committing through `session/hlsopt_session.sh attempt --status ... --label ...` (never `git add -A`)
- Putting the cascade table in the commit message / ledger
- Proceeding with next steps
