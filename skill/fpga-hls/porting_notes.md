# porting_notes — Ross HLS skills → Vitis HLS 2022.2（本机逐条核实）

核实环境：本机 `/tools/Xilinx/Vitis_HLS/2022.2`，器件 `xck26-sfvc784-2LV-c`，`-flow_target vivado`，clock 10 ns。
探针工程位于会话 scratchpad（临时，未入库）；每个探针一个最小 kernel、单独 `csynth_design`，判读看
日志 WARNING/ERROR 与 `syn/report/csynth.rpt` 的 Pragma Report。

## 1. 核实方法（三层）

1. **头文件存在性**：`ls $XILINX_HLS/include/<hdr>`，再用 include 该头的最小 kernel 跑 csynth 确认（不存在时报 `HLS 207-812 file not found`）。
2. **pragma/选项表**：`$XILINX_HLS/data/hls/HLSPragma.json`（2022.2 前端的 pragma 定义：每个 pragma 的 Objects、选项、Hidden/Deprecated 标记）；Tcl 命令用 `vitis_hls -f help.tcl`（`help csim_design` 等）导出。
3. **实测 csynth**（决定性）：对每个存疑写法跑最小设计，看是否被接受、是否真正生效。
   **阴性对照**：`#pragma HLS pipeline II=1 bogus_opt=3` 与 `#pragma HLS foobar_nonexistent` —— 2022.2 只报
   `HLS 207-5554 unexpected pragma parameter` / `HLS 207-5572 unknown HLS pragma ignored` 两条 WARNING，综合照常成功。
   ⇒ "综合成功"不能证明 pragma 生效；判据必须是日志 WARNING + Pragma Report 的 *Ignored Pragmas* 表 + 生效消息。

## 2. 2022.2 不可用 / 有差异（结论）

| 项 | 2022.2 结论 | 证据 |
|---|---|---|
| `hls_array_partition.h`（`hls::scatter/gather`） | **不可用** | 头不存在；include → `ERROR [HLS 207-812] 'hls_array_partition.h' file not found` |
| `ap_float.h`（`ap_float<W,E>`） | **不可用** | 头不存在；include → `ERROR [HLS 207-812]` |
| `hls_burst_maxi.h`（`hls::burst_maxi`） | 可用 | csynth 通过，日志 `HLS 214-306 Performing burst_maxi read of length 64` |
| `hls_task.h`、`hls_thread_local`、`hls_streamofblocks.h`、`hls_np_channel.h` | 可用 | 头存在；`hls_thread_local hls::task` 的 dataflow 探针 csynth 通过 |
| `#pragma HLS performance target_ti=` | **只在循环体内生效**；嵌套循环外层、内层都可放 | 循环体内：`HLS 200-1957 Successfully applied performance pragma with Target TI='32'`，Performance Pragma Report "TI met yes"。函数体（含顶层）与"标号和 for 之间"：`WARNING [HLS 207-5562] performance pragma is ignored, because it is not in a loop`。公开选项只有 `target_ti`、`target_tl`（无 `unit=`；`assume_ti`/`assume_tl`/`scope` 在 HLSPragma.json 标 Hidden）。嵌套探针（外层 TI=4096、内层 TI=128，trip 32×64，工作目录 `<tmp>/rht_pp/`）：`HLS 214-269 Inferring pragma 'pipeline II=2' for loop 'Col_Loop' ... due to performance pragma`、`HLS 214-339 Inferring pragma 'loop_flatten'`、`HLS 200-1957 Successfully applied performance pragma with Target TI='4096' on loop 'Row_Loop_Col_Loop'`，Performance Pragma Report "TI met yes" |
| `#pragma HLS pipeline ... rewind=true / rewind=false` | **语法错误，整条 pragma 丢弃** | `WARNING [HLS 207-5555] unexpected pragma argument '='`，Ignored Pragmas 表列出，`Target II = NA`（II=1 也一并丢失）。只认裸 `rewind` |
| 自动 rewind | 存在，由 `config_compile -enable_auto_rewind`（默认 true）控制 | `help config_compile` |
| `pipeline style=frp` | 可用 | csynth.rpt Pipelined 列 `yes(frp)` |
| 默认 pipeline style | `config_compile -pipeline_style` 默认 **frp**（偏好而非硬约束）；上游写"默认 stp" | `help config_compile`。实际选用以 csynth.rpt 的 Pipelined 列为准 |
| `#pragma HLS array_stencil` | 可用（HLSPragma.json 标 Hidden） | `HLS 214-330 Applying 3x3 stencil optimization to variable 'in'` |
| `#pragma HLS inline region` | 弃用但仍接受 | `WARNING [HLS 207-5528] 'region' in '#pragma HLS Inline' is deprecated` |
| `config_compile -pragma_strict_mode=true` | 可用 | 打开后 207-5555 变 ERROR，综合失败（rc=1） |
| `csim.profile_tripcount` | 无同名项；最接近 `csim_design -profile` | `help csim_design` |
| `cosim.argv`/`csim.argv` | `cosim_design -argv` / `csim_design -argv` | `help` |
| `vitis-run`、`v++ --mode hls`、`hls_config.cfg`、`vitis-comp.json`、Unified IDE 函数 | 不存在 | 2022.2 安装目录无 `vitis-run`；`v++` 属 Vitis 2022.2 但无 `--mode hls` |
| `$XILINX_VITIS/include/ap_fixed.h` | 不存在；用 `$XILINX_HLS/include` | `ls` |
| csynth.rpt "Pragma Report" | **存在**（Ignored Pragmas、Performance Pragma Report） | 探针 csynth.rpt |
| `inferred_directives.ini` | 不存在 | 本机多个已综合工程 `find` 为空；替代：日志 214-268/269/291、XFORM 203-510，`.autopilot/db/all.directive.json` |
| `logs/hls_compile.log` | 不存在；原始日志 = `<solution_dir>/<solution>.log` | 已有工程 |
| `.autopilot/db/process_info.csv`、`loop_info.csv` | **存在（cosim 后）**：`<solution_dir>/.autopilot/db/` 下有 `process_info.csv`、`loop_info.csv`、`process_stalling_info/process.zip`、`loop_performance_info/loop.zip`，另有 `channel_info.csv`、`channel_depth_info/`；是否生成可能因设计与 cosim 选项而异。`hls_report.sh cosim` 的进程/通道解析依赖它们 | 脚本层实测：`<runs>/m7-hls-bringup-20260916/out/fpga_backproject-10ns/solution/.autopilot/db/`（本代理复核 4 个文件均在）。**更正 2026-11-02**：前一版此行写"不存在"，有误 |
| `verbose.sched.rpt`、`Pipelining result`（HLS 200-1470）、`HLS 200-885` | 存在，格式同上游 | 已有工程日志 |
| `export_design -flow syn` 的 export.rpt | 只有 post-synthesis 段，无 post-route | 已有工程 `*_export.rpt` |
| `export_design -flow impl` 产物格式 | **未核实**：本机小探针跑 impl 时 Vivado 段错误（`Abnormal program termination (11)`，路径很长的 /tmp scratchpad 下；疑与 unix socket 路径长度有关，未定位），`vitis_hls` 只报 `ERROR [HLS 200-478] vivado returned an error` | 交接项 |

## 3. hlsopt 会话脚本自测（session/selftest.sh）

`bash session/selftest.sh`（临时 git 仓库，不碰真实仓库）：**44 passed, 0 failed**。覆盖：
start → attempt(fail) → revert → attempt(pass) → good → finish（存档补丁数=提交数、sha256 自校验、功能分支只多一个提交、
源码=最佳 good、账本不进功能分支、提交信息含尝试次数与存档路径、实验分支保留、无 remote、`--remove-worktree`）；
架构路径：start --allow-new → baseline(arch 标签) → attempt(fail) → revert 回 arch → attempt(pass) → good → finish。
阳性对照（均因脚本的指定原因被拒）：①原副本源文件有未提交修改 ⇒ start 拒绝；②非 exp/ 分支上 attempt ⇒ 拒绝；
③原副本在 main ⇒ start 拒绝；④未提交失败尝试就 revert ⇒ 拒绝；⑤失败尝试后 good ⇒ 拒绝；⑥新文件在原副本已是未跟踪文件 ⇒ 拒绝；
⑦新文件缺 --allow-new ⇒ 拒绝。自测前后本地 `user.name/email` 不变（alice|alice@example.org），未创建 `.gitignore`。

## 4. 交接（2026-11-02 文字层收尾：完成）

本节原为上一位代理的未完成清单，已全部处理：

1. `upstream.md` 已写：来源、tarball sha256、MIT 全文、20 个 skill 的作者表与吸收映射、未拿的内容（hls-matlab-to-cpp、hls-scripts、
   skill-card、钩子）、2022.2 适配摘要、git 纪律改造的新旧覆盖与差集、Stop 钩子不装的评估。
2. `reference/architect/setup.md`、`design-layout.md` 改为 2022.2：`source settings64.sh` + `$XILINX_HLS`、从 `run_hls.tcl`/`hls.app`
   取设置、`run_hls.tcl` 模板、流程走 `scripts/hls_run.sh`、产物建在 hlsopt 实验区。`vitis-libraries.md` 加 2022.2 分支注记（未联网核实）。
3. `reference/perf-pragma.md` 实质改写：所有 performance pragma 只放循环体第一行；函数级预算落到最外层循环；clock/top 从 `run_hls.tcl` 读
   （`create_clock -period` 也可写 MHz，`help create_clock` 核实）；去掉 `detect_loops.sh`；加生效判据（200-1957 / 214-269 /
   Performance Pragma Report / Ignored Pragmas）。嵌套探针见 §2。
4. `pipeline.md`、`pragma-scope.md` 加 2022.2 注记（rewind 只认裸开关、默认 style frp〔`help config_compile` 复核〕、auto rewind、
   pragma_strict_mode、`hls-loop-unrollable` 上游不存在、PERFORMANCE 仅循环、INLINE region 弃用、8.4 指向 dataflow.md）。
   `dataflow`、`synthesizable`（`ap_float` 不存在）、`burst-inference`（引本机 2022.2 日志的 214-115 / 200-1603 实例）、`array-to-stream`、
   `stencil-pattern`、`pipe-decomp` 末尾加注记；`flattenable`、`line-buffer` 无版本相关内容，原样。
5. 按脚本层（分支 `feat/<user>/ross-hls-port-20261102`，提交 `54d4910`）最终接口对齐 `SKILL.md` §2、`run-flow.md` §2（`--argv`/`--csim-opts`/
   `--cosim-opts`/`--impl-opts`/`--timeout`、`--tcl` 桩语义、impl 默认 libudev 预加载）、`read-qor.md` §3/§5（process_info.csv 更正、退出码 3/4）。

### 自查结果

- `grep -rn "vitis-run\|vitis-comp.json\|hls_config.cfg\|runHlsActiveComponentByType\|getSkillResourcePath\|v++ --compile --mode hls" --include=*.md`
  除 UPSTREAM/porting_notes 外剩余各处全部是"2022.2 不存在 / 被替换 / 禁止生成"的说明句：
  SKILL.md:39-40（不存在清单）、SKILL.md:78（不要退回）、architect.md:183/415（run_hls.tcl 取代）、architect.md:441（FORBIDDEN）、
  architect/design-layout.md 规则节（禁止生成）、run-flow.md:3-4（移植说明）、run-flow.md cfg→Tcl 对照表两处、read-qor.md:14（不存在）、
  optimize/init-component.md:16/18（取代 / 没有）、optimize.md:53（没有）、optimize/attempt-template.md:2（移植说明）。
- 路径存在性（`grep -rhoP '(?<![/\w.-])(reference|scripts|session)/[\w./-]+\.(md|sh|py)' --include=*.md`，排除带上游前缀的路径）：
  28 个引用中 23 个存在；缺 5 个，全在 `scripts/`，均已在 `54d4910` 中：`scripts/hls_run.sh`、`scripts/hls_report.sh`、
  `scripts/qor_log.py`、`scripts/bitexact_check.py`、`scripts/bitexact.md`。**两分支合并后重跑此检查。**
  各文件 `../` 相对链接已全部可解析（修了 architect/ 下三处 `../upstream.md` → `../../upstream.md`）。
- `bash session/selftest.sh`：44 passed, 0 failed（未改脚本）。

### 遗留

- `export_design -flow impl` 产物格式仍未在本机小探针上核实（§2 末行）；脚本层称 impl 段错误首查 libudev 预加载，再查路径长度。
- 合并后：脚本层另有 `scripts/porting_notes.md`（不同文件，不冲突），两边关于报告路径的结论需交叉核对一次。
- Vitis_Libraries `2022.2` 分支未联网核实。
- 探针目录 `<tmp>/rht_pp/` 未删（删除命令被权限拦截），可由主会话清理。
