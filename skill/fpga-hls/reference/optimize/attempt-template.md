> 来源：AMD Ross `hls-optimize/template.md`（作者：Alex Li, Sai Akhil Ayyagari），MIT 许可，见 `../../upstream.md`。
> 2022.2 移植：`vitis-run` / `v++ --mode hls` 命令换成 `scripts/hls_run.sh`；报告路径换成 2022.2 solution 布局；
> 上游引用的 `.claude/skills/synth/scripts/report_summary.py` 在上游包里不存在，换成 `scripts/hls_report.sh`；
> 新增逐位一致门；git 操作改由 `session/hlsopt_session.sh` 执行（见 `../optimize.md` Step 6b）。

# Sub-agent prompt template for a single optimization attempt

Use this template when launching a sub-agent for one optimization attempt.
Fill in all `[bracketed]` fields before sending. Paths: `SKILL_ROOT=tools/agent_skills/fpga-hls`（相对实验区仓库根）。

---

Task: Apply [specific technique] to `[source files]`.

Goal: [state the optimization criteria, e.g. "minimize true latency in seconds"].

Work area: experiment worktree `[实验区路径]` on branch `[exp/<本人>/hls-opt-<模块>-<日期>]`. Do all edits and runs there.
Project: `<proj_dir>=[...]`, `<solution>=[...]`, `<solution_dir>=<proj_dir>/<solution>`. Ledger: `[<ledger>]`.
Golden: `[<ledger>/golden/csim]`, `[<ledger>/golden/cosim]`.

Environment: `source /tools/Xilinx/Vitis_HLS/2022.2/settings64.sh` (Vitis HLS 2022.2; part `xck26-sfvc784-2LV-c`).

Forbidden: `git init`, `git config` (any write), editing `.gitignore`, `git push`, `git reset --hard`, `git clean`,
any `--force`, `git add -A`. Do not commit — the caller commits via `session/hlsopt_session.sh` (unless the caller
explicitly delegates the `attempt` call to you; then use exactly that script).

Baseline:
- cosim latency: [X] cycles
- post-route clock: [Y] ns
- true latency: [Z] ms  (= X × Y)
- LUT: [A], FF: [B], DSP: [C], BRAM: [D]

Hypothesis:
- What you will change: [describe exactly what pragma or algorithmic change to apply and where]
- What you expect to happen: [e.g. "II on the k-loop should drop from 5 to 2 because we now have 2 independent memory ports for C"]
- What would confirm the hypothesis: [e.g. "csynth loop table shows the hot loop's II reduced from N to M"]
- What would falsify it: [e.g. "II unchanged, meaning the bottleneck is a recurrence rather than a memory dependency"]

Output throttling: never `cat` whole logs; use `tail` / `grep`; put long excerpts in a file and cite the path.

Steps:

### 1. Read the current kernel

Read the source files. Understand the current implementation before touching anything.
Also run:
```bash
$SKILL_ROOT/scripts/hls_report.sh synth <solution_dir>
```
to confirm you understand the baseline loop structure and bottlenecks.
Before writing any pragma, check placement rules in `$SKILL_ROOT/reference/pragma-scope.md` and the 2022.2
differences in `$SKILL_ROOT/porting_notes.md` (e.g. `performance` only inside loop bodies; `rewind=true` drops the whole pragma).

### 2. Apply the change

Make **only** the change described above. Do not fix, refactor, or optimize anything else
while you are in here — even if you notice an opportunity. One change per attempt.

### 3. Run csim + bit-exact check

```bash
$SKILL_ROOT/scripts/hls_run.sh <proj_dir> <solution> csim
```

Read the tail of the csim log in `<solution_dir>/agent_logs/` (2022.2 also writes `<solution_dir>/csim/report/<top>_csim.log`).
If it ends with `FAIL` or a non-zero error count, fix the logic error (if trivial) or abort and report why.
Then:

```bash
python3 $SKILL_ROOT/scripts/bitexact_check.py <ledger>/golden/csim <this run's csim outputs>
```

Any mismatch ⇒ the attempt **fails**; do not proceed to csynth; report back (the caller commits it as failed and reverts).

### 4. Run csynth

```bash
$SKILL_ROOT/scripts/hls_run.sh <proj_dir> <solution> csynth
```

Then investigate **thoroughly**:

#### 4a. Quick summary
```bash
$SKILL_ROOT/scripts/hls_report.sh synth <solution_dir>
```

Check:
- Did II change on the loop you targeted?
- Did II change on any other loop (side effect)?
- Did total latency change? By the expected factor?
- Did resource usage change? By how much?

#### 4b. Ignored pragmas and violations
First read `== Pragma Report` → *Ignored Pragmas* at the end of `<solution_dir>/syn/report/csynth.rpt` (2022.2 has it).
If your pragma is listed there, the change was **not applied** — stop and report.
Then read the II-violation messages in the raw log `<solution_dir>/<solution>.log`
(`grep -n "HLS 200-88[0-9]\|Pipelining result" <log> | tail -40`).
Does the remaining violation match your hypothesis about what is limiting II?
If the violation is different from what you expected, note it — the tool is telling you
the actual bottleneck. For a real diagnosis use `$SKILL_ROOT/reference/ii-diagnose.md`.

#### 4c. Auto-inferred directives
2022.2 has no `inferred_directives.ini`. Check the raw log for `XFORM 203-510 ... automatically`,
`HLS 214-268/269 Inferring pragma ...`, `HLS 214-291 ... complete unroll implied by the pipeline pragma`,
and `<solution_dir>/.autopilot/db/all.directive.json`. Did HLS auto-apply any directive that interacts
with your change (e.g. auto-partition, auto-flatten, auto-pipeline)?

#### 4d. Warnings in the synthesis log
```bash
grep -i "warning\|violation" <solution_dir>/<solution>.log | head -40
```

#### 4e. IR inspection (when verifying compiler behavior)
When you want to confirm the compiler actually applied the transformation you intended
(e.g. that an array was partitioned, or a loop was unrolled in the IR):

```bash
export LD_LIBRARY_PATH="$XILINX_HLS/lib/lnx64.o:$XILINX_HLS/lnx64/lib/lnx64.o${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$XILINX_HLS/lnx64/tools/clang-3.9-csynth/bin/llvm-dis" <solution_dir>/.autopilot/db/a.g.bc -o - 2>/dev/null | grep -A30 "<pattern of interest>"
```
(2022.2: `llvm-dis` exists at that path and `a.g.bc` is in `.autopilot/db/`; the `LD_LIBRARY_PATH` line is carried
over from upstream and not separately verified here.)

Things to look for:
- **After `ARRAY_PARTITION`**: multiple GEP chains or multiple `alloca`s for the partitioned array.
- **After `UNROLL factor=N`**: N repetitions of the loop body in the IR.
- **After `LOOP_FLATTEN`**: single induction variable instead of nested loop counters.

#### 4f. Log the QoR
```bash
python3 $SKILL_ROOT/scripts/qor_log.py ...   # see --help; label "[technique-applied]", csv <ledger>/qor_history.csv
```

### 5. Assess vs. hypothesis

Write down explicitly:
- Did the result match the hypothesis?
- If yes: what does this confirm about the design?
- If no: what does the actual result reveal? Is there a secondary bottleneck?

This assessment determines the next step.

### 6. Decide whether to escalate

| csynth result | Action |
|---|---|
| Latency improved ≥ 10% OR II reduced on dominant loop | Run cosim |
| Latency unchanged, but hypothesis confirmed (e.g. "acknowledge II" change) | Report; caller commits; move to next idea |
| Latency got worse | Investigate. Do not immediately revert — understand why first. Then decide. |
| csim FAIL or csim bit-exact FAIL | Attempt failed. Report; caller commits it as failed and reverts. |

#### If running cosim:
```bash
$SKILL_ROOT/scripts/hls_run.sh <proj_dir> <solution> cosim
python3 $SKILL_ROOT/scripts/bitexact_check.py <ledger>/golden/cosim <this run's cosim outputs>
$SKILL_ROOT/scripts/hls_report.sh cosim <solution_dir>
```
Bit-exact mismatch ⇒ attempt failed regardless of cycles. Otherwise read `<solution_dir>/sim/report/<top>_cosim.rpt`
(via the report script). Compare cosim cycles to csynth estimate. Note any discrepancy (cosim includes stall cycles; csynth does not).

#### If running impl:
```bash
$SKILL_ROOT/scripts/hls_run.sh <proj_dir> <solution> impl      # export_design -flow impl
$SKILL_ROOT/scripts/hls_report.sh impl <solution_dir>
```
Use the post-route clock period (never the `-flow syn` post-synthesis number).
Compute: `true latency = cosim_cycles × period_ns`.

### 7. Report back

Provide a concise summary:

```
Technique applied: [exact pragma or code change]
File changed: [file] lines [N-M]

Hypothesis: [what you expected]
Confirmed: [yes/no/partial]
Reason: [why the result matched or didn't match the hypothesis]

Bit-exact: csim [yes/no], cosim [yes/no/not run]

csynth result:
  Latency: [baseline] → [new] cycles (csynth estimate)
  II (targeted loop): [baseline] → [new]
  II (other loops): [any unexpected changes]
  Resources: LUT [A]→[A'], FF [B]→[B'], DSP [C]→[C'], BRAM [D]→[D']
  Timing: [estimated] ns (target: [X] ns)
  Violations: [list any new or resolved violations]
  Ignored pragmas / inferred directives: [any surprises?]
  IR observation: [what you saw in a.g.bc, if inspected]

cosim result (if run):
  Latency: [X'] cycles (vs csynth estimate [Y'] — delta: [Z%])

impl result (if run):
  Post-route clock: [Y'] ns
  True latency: [Z'] ms (vs baseline [Z] ms)

Recommendation: [keep (good) / revert / investigate further]
```
