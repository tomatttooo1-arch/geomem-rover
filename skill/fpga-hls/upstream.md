# UPSTREAM — fpga-hls 的上游来源、许可、作者、吸收映射与改造说明

## 1. 来源

| 项 | 值 |
|---|---|
| 上游仓库 | AMD Ross AI Assistant — https://github.com/Xilinx/ross-ai-assistant |
| 分支 / 下载日期 | `main`，2026-11-02 下载（tarball） |
| 本地只读副本 | `<upstream_copy>/ross-ai-assistant-main/` |
| tarball | `<upstream_copy>/ross.tgz` |
| tarball sha256 | `a20046c7253b38dce9b4df8a3e9f93eedf1e6c62dc1686feff492692faab9d23`（同目录 `SHA256SUMS`） |
| 上游目标工具版本 | Vitis 2025.x（Unified IDE、`vitis-run`、`hls_config.cfg`） |
| 本移植目标 | Vitis HLS 2022.2 老式 Tcl 工程，KV260 `xck26-sfvc784-2LV-c` |

本仓库首个导入提交 `9bb5866` 是上游文字的原样导入，其后提交才是 2022.2 适配，可用 `git diff 9bb5866 -- tools/agent_skills/fpga-hls`
逐行看改动。

## 2. 许可（MIT，上游 `LICENSE` 全文）

```
MIT License

Copyright (c) 2026 Advanced Micro Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

本 skill 的改写部分沿用 MIT 许可。

## 3. 作者表与吸收映射（20 个 skill）

作者取自上游各 `skills/<name>/SKILL.md` frontmatter 的 `metadata.author`，上游版本均为 `1.0.0`。

| 上游 skill | 作者 | 本 skill 中的位置 | 处理 |
|---|---|---|---|
| hls-architect | Sai Akhil Ayyagari | `reference/architect.md`；`reference/architect/setup.md`、`design-layout.md`、`vitis-libraries.md`（来自其 `reference/`） | 改写为 2022.2 Tcl 工程 + hlsopt 实验区；Stop 钩子改为 SKILL.md 硬规则 H2 |
| hls-optimize | Alex Li, Sai Akhil Ayyagari | `reference/optimize.md`；`reference/optimize/init-component.md`（来自其 `reference/`）；`reference/optimize/attempt-template.md`（来自其 `template.md`） | 加逐位一致门（H1）；git 纪律改为 `session/hlsopt_session.sh`；`qor_log.py` 由脚本层另做 |
| hls-run-flow | Alex Li | `reference/run-flow.md` | 重写为 Tcl 流程 + cfg→Tcl 对照表 |
| hls-component-basic-info | Alex Li | `reference/read-qor.md` §basic-info | 5 个读报告 skill 合并；取数走 `scripts/hls_report.sh` |
| hls-synth-report | Alex Li | `reference/read-qor.md` §synth | 同上 |
| hls-cosim-report | Alex Li | `reference/read-qor.md` §cosim | 同上 |
| hls-impl-report | Alex Li | `reference/read-qor.md` §impl | 同上 |
| hls-dataflow-infos | Alex Li | `reference/read-qor.md` §dataflow | 同上 |
| hls-ii-diagnose | Zayn He | `reference/ii-diagnose.md` | 报告路径改为 2022.2 |
| hls-perf-pragma | Sai Akhil Ayyagari | `reference/perf-pragma.md` | pragma 位置改为"只在循环体内"；clock/top 从 `run_hls.tcl` 读 |
| hls-pipeline | Sheng Wang | `reference/pipeline.md` | 加 2022.2 注记（rewind、默认 style） |
| hls-pragma-scope | Yi Gao | `reference/pragma-scope.md` | 加 2022.2 注记 |
| hls-pipe-decomp | Zayn He | `reference/pipe-decomp.md` | 加 2022.2 注记 |
| hls-dataflow | Alexandre Isoard, Alain Darte | `reference/dataflow.md` | 加 2022.2 注记 |
| hls-synthesizable | Alex Li | `reference/synthesizable.md` | 加 2022.2 注记 |
| hls-flattenable | Alex Li, Yi Gao | `reference/flattenable.md` | 规则原样 |
| hls-burst-inference | Alex Li | `reference/burst-inference.md` | 加 2022.2 注记 |
| hls-array-to-stream | Huina Li, Alex Li | `reference/array-to-stream.md` | 加 2022.2 注记 |
| hls-stencil-pattern | Alex Li, Yuxin Wang | `reference/stencil-pattern.md` | 加 2022.2 注记 |
| hls-line-buffer | Sai Akhil Ayyagari | `reference/line-buffer.md` | 规则原样 |

### 未拿的内容

- **hls-matlab-to-cpp**（Sai Akhil Ayyagari）：MATLAB → C++ 转换流程。本团队的 HLS 输入都是 C/C++（或 Python 参考实现），
  无 MATLAB 源；该 skill 依赖 MATLAB 运行环境与 Unified IDE 流程，本机均无。需要时再单独移植。
- **hls-scripts** 及各 skill 自带的 `scripts/`、`hls-scripts/`（`hls_adb`、`hls_synth_report` 等 Python，基于 2025.x 报告格式与
  Unified IDE 目录层级）：不复制，由本仓 `scripts/` 层（`hls_run.sh`、`hls_report.sh`、`qor_log.py`、`bitexact_check.py`）
  按 2022.2 报告格式另写。
- 各 skill 的 `skill-card.md`（面向 Ross 插件市场的展示卡）：无功能内容，未带入。
- **hls-architect/hooks/architect-hooks.py**：评估后不装，见第 5 节。

## 4. 2022.2 适配要点（摘要；逐条证据见 `porting_notes.md`）

- 工程形态：`vitis-run` / `v++ --mode hls` / `hls_config.cfg` / `vitis-comp.json` / Unified IDE 函数在 2022.2 均不存在，
  全部换成 `run_hls.tcl`（`open_project`/`set_top`/`add_files [-tb]`/`open_solution`/`set_part`/`create_clock`/`csim_design`/
  `csynth_design`/`cosim_design`/`export_design`），经 `scripts/hls_run.sh` 调用。cfg 键→Tcl 对照见 `reference/run-flow.md` §3。
- 头文件：`hls_array_partition.h`、`ap_float.h` 不存在；`hls_burst_maxi.h`、`hls_task.h`、`hls_streamofblocks.h`、`hls_np_channel.h` 可用。
- `#pragma HLS performance` 只在循环体内生效（函数体/顶层/"标号与 for 之间"被忽略）。
- `pipeline rewind=true|false` 是语法错误、整条 pragma 被丢弃；只写裸 `rewind`。默认 pipeline style 为 frp（上游写 stp）。
- 格式错误的 pragma 默认只是 WARNING；`config_compile -pragma_strict_mode=true` 可让它报错。
- 报告路径：原始日志 `<solution>/<solution>.log`（无 `logs/hls_compile.log`）；无 `inferred_directives.ini`；csynth.rpt 有 Pragma Report。

## 5. 改造：git 纪律与钩子

### 5.1 git 纪律改造与覆盖变化（规则：改验证机制要写覆盖变化）

**旧（上游 hls-optimize / init-component）**：在当前目录 `git init`（若无仓）、`git config user.*`、改 `.gitignore`；
每次尝试（含失败）一个提交直接落在当前分支；失败后 `git checkout HEAD~1` 之类回退。
它覆盖了：每次尝试可回退、可追溯；失败尝试也留痕；"哪次改动引入问题"在历史里可查。
它在团队共享仓里的问题：嵌套仓、改团队 git 配置与 .gitignore、几十个尝试提交污染共享历史并可能被推送、`HEAD~1` 在有别人提交时退错位置。

**新（`session/hlsopt_session.sh`）**：
- `start` 在用户功能分支上为本次优化建专用 git worktree + 本地分支 `exp/<本人>/hls-opt-<模块>-<日期>`（永不推送；脚本从不 push、
  从不 `git init`/`git config`/改 `.gitignore`/`reset --hard`/`--force`）。
- 每次尝试 `attempt --status pass|fail` 照旧一个提交（落在实验分支），逐位一致通过的打 `good` 标签；`revert` 回到最近 good 标签
  （架构流程首次为 `arch` 标签），不用 `HEAD~1`。
- `finish` 把实验分支全部提交 `git format-patch` 成补丁并连同 QoR/尝试账本存到 `shared/artifacts/`（带 sha256 自校验），
  再向用户功能分支做**一次** squash 提交（源码 = 最佳 good，提交信息含尝试次数与存档路径；账本不进功能分支）。

**覆盖差集**：远程（共享）历史里看不到逐次尝试的细节，只有一次汇总提交。补偿：实验分支在本地保留，补丁与账本存档在
`shared/artifacts/`，提交信息写明存档路径，可完整重放每一次尝试。owner 于 2026-11-02 接受此差集。
其余覆盖（逐次可回退、失败留痕、可追溯）不变；回退目标从"上一个提交"改为"最近 good 标签"更严格。
脚本自测 `session/selftest.sh`：44 passed / 0 failed（覆盖项见 `porting_notes.md` §3）。

### 5.2 Stop 钩子（hls-architect/hooks/architect-hooks.py）：不装

上游钩子在 Stop / UserPromptSubmit 事件上检查架构流程是否在校验后提前收工，必要时阻止收工。评估结论：**不装**。理由：
1. 只能装成全局钩子：本机启动链 `--setting-sources user`，项目级 settings 不生效；全局 Stop/UserPromptSubmit 钩子作用于所有会话
   （每轮起一次 Python、可能向下扫 3 层目录，在 fpga 大仓里有开销）。
2. 误伤：其 fallback 逻辑会把 cwd 附近任何含 C++ 的 `rearchitect/` 目录自动登记为进行中的架构流程，并最多阻止收工 5 次。
3. 版本不配套：钩子读 `.architect_state`，而随包 SKILL.md 并不写这个文件，实际只能走 fallback。

它要保证的东西（3a 校验后不得收工、3b 必须停下等用户明确批准）已改写为 `SKILL.md` 硬规则 H2 与 `reference/architect.md`
Step 3a/3b 的硬规则。覆盖差集：从"工具强制"降为"文本约束"，失去对模型违规的机械拦截；接受理由同上 1–3。
