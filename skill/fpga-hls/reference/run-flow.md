> 来源：AMD Ross `hls-run-flow`（作者：Alex Li），MIT 许可，见 `../upstream.md`。
> 上游描述：Run HLS Flow including C Simulation (csim), C Synthesis (csynth), C/RTL Co-Simulation (cosim), Implementation (impl).
> 2022.2 移植：上游的执行方式（Vitis Unified IDE 的 `runHlsActiveComponentByType`，或 2025.x 命令行
> `vitis-run --mode hls` / `v++ --compile --mode hls` + `hls_config.cfg`）在 2022.2 都不存在，整段改写为
> `scripts/hls_run.sh` + Tcl 工程。关键词表、前置检查、cosim argv 预检的意图原样保留。

# 跑 HLS 流程（csim / csynth / cosim / impl）

## 1. 关键词 → 流程

| 用户说 | 流程 | 2022.2 Tcl 命令 | `hls_run.sh` step |
|---|---|---|---|
| `run csim` | C Simulation | `csim_design [-argv ...] [-ldflags ...] [-profile]` | `csim` |
| `run csynth` | C Synthesis | `csynth_design` | `csynth` |
| `run cosim` | C/RTL Co-Simulation | `cosim_design [-argv ...] [-ldflags ...] [-rtl verilog] [-trace_level ...]` | `cosim` |
| `run impl` | Implementation | `export_design -flow impl -format ip_catalog` | `impl` |

用户说的不是这四个之一时，问清楚要跑哪个。

## 2. 怎么跑

```bash
source /tools/Xilinx/Vitis_HLS/2022.2/settings64.sh
scripts/hls_run.sh <proj_dir> <solution> <step> [<step> ...]          # 工程已有 hls.app
scripts/hls_run.sh <proj_dir> <solution> <step> ... --tcl <run_hls.tcl>  # 工程还没建（没有 hls.app）
```

可选项（可写在任意位置；以 `scripts/hls_run.sh` 头部注释为准）：

| 选项 | 作用 |
|---|---|
| `--tcl <file>` | 工程建立脚本。没有 `hls.app` 时必须给；已有 `hls.app` 时忽略并提示。脚本被 `source` 时 `csim_design`/`csynth_design`/`cosim_design`/`export_design`/`close_*`/`exit` 被替换成桩，只有建工程部分（`open_project`、`add_files`、`set_top`、`open_solution`、`set_part`、`create_clock`、`config_*`、`set_directive_*`）生效；脚本里写给这些流程命令的参数（如 `cosim_design -rtl verilog`、`csim_design -argv {8 8 8}`）作为**本次**调用的默认值 |
| `--argv "<args>"` | csim 与 cosim 的 testbench 参数。默认：csim 用 `hls.app` 的 Simulation argv；cosim 用 `<solution>.aps` 的 argv，没有则回落到 csim 的 argv（即上游 `cosim.argv ← csim.argv` 语义） |
| `--csim-opts "..."` | 追加给 `csim_design` 的选项（如 `-ldflags ...`、`-profile`） |
| `--cosim-opts "..."` | `cosim_design` 选项（默认取 `--tcl` 桩里的参数，否则 `-rtl verilog`） |
| `--impl-opts "..."` | `export_design` 选项（默认 `-flow impl -rtl verilog -format ip_catalog`；总会保证 `-flow impl`） |
| `--timeout <sec>` | 每个 step 的墙钟上限（默认 0 = 不限） |

- `impl` 默认带 libudev 预加载。本机 Vivado 段错误（`Abnormal program termination (11)`）时先查 libudev 预加载是否生效，
  再查工作路径是否过长（Unix socket 路径上限约 108 字节，工程放短路径下）。

- 日志在 `<proj_dir>/<solution>/agent_logs/`；报告位置与读法见 `reference/read-qor.md`。
- **解析 `<proj_dir>` 与 `<solution>`**（取代上游的 `$CONFIG` / `$WORK_DIR` 解析）：
  1. 用户给了就用用户的。
  2. 否则在工作区找 `hls.app`（工程根）或 `run_hls.tcl`（尚未建工程）；`<solution>` 取 `hls.app` 里
     `<solution name=...>`，或 `run_hls.tcl` 里 `open_solution <name>`。找到多个时问用户，不要挑一个猜。
  3. 不要把工作区根、`.`、或 solution 目录本身当 `<proj_dir>`。
- 不要绕过脚本直接拼 `vitis_hls -f`；脚本失败就停下报告，不要回退到上游命令。
- 需要大段日志时用 `tail`/`grep` 看 `agent_logs/` 里的文件，不要整段 `cat`。
- **返回码不可全信**：2022.2 里 pragma 写错只是 WARNING；Vivado 在 impl 中崩溃时 `vitis_hls` 也只打一行
  `ERROR: [HLS 200-478] vivado returned an error`。跑完一律 grep 日志里的 `^ERROR` 与 `Ignored Pragmas`。

## 3. 前置检查（原样保留，路径改为 2022.2）

- **cosim 之前**：确认 csynth 已跑过——`<proj_dir>/<solution>/syn/` 存在（上游是 `$WORK_DIR/hls/syn/`）。
  不存在就停下告诉用户：
  > "C Synthesis (csynth) must be completed before running Co-Simulation. Please run csynth first."
- **cosim 之前的 argv 预检**（上游的 `cosim.argv` 预检）：
  1. 看 `run_hls.tcl` 里 `cosim_design` 是否带 `-argv`。有且非空 → 继续。
  2. 没有 → 看 `csim_design -argv`（或 `hls.app` 的 `<Simulation argv="...">`）。有且非空 → 把同一值加到
     cosim 上。`hls_run.sh` 默认已做这个回落；需要显式指定时用 `--argv "..."`。
  3. 两者都没有 → 继续（testbench 可能不需要参数）。
  argv 必须一致，否则 cosim 跑的数据和 csim 不同，逐位比对没有意义。
- **impl 之前**：确认 cosim 已跑过——`<proj_dir>/<solution>/sim/` 存在。不存在就停下：
  > "C/RTL Co-Simulation (cosim) must be completed before running Implementation. Please run cosim first."
- `impl` 必须是 `-flow impl`。`-flow syn` 只给 post-synthesis 数，不能当 post-route 用（见 `reference/read-qor.md` §4）。

## 4. 上游 cfg 键 → 2022.2 Tcl 对照

上游文本里出现的 `hls_config.cfg` 键，在 2022.2 用下表的 Tcl 等价物（写在 `run_hls.tcl` 里）：

| 上游 cfg 键 | 2022.2 Tcl |
|---|---|
| `part=<XPART>`（顶层） | `set_part xck26-sfvc784-2LV-c` |
| `clock=<ns>` | `create_clock -period <ns> -name default` |
| `syn.top=<fn>` | `set_top <fn>` |
| `syn.file=<f>` | `add_files <f> [-cflags "..."]` |
| `tb.file=<f>` | `add_files -tb <f> [-cflags "..."]` |
| `tb.cflags=...` | `add_files -tb <f> -cflags "..."`（逐文件） |
| `csim.argv=...` | `csim_design -argv "..."` |
| `cosim.argv=...` | `cosim_design -argv "..."` |
| `csim.ldflags=...` | `csim_design -ldflags "..."` |
| `cosim.ldflags=...` | `cosim_design -ldflags "..."` |
| `csim.profile_tripcount=1` | 2022.2 没有同名项；最接近的是 `csim_design -profile`（"Enable profiling"）。是否给出 trip count 以实际产物为准；拿不到时在源码里加 `#pragma HLS loop_tripcount` |
| `syn.directive.*` | `set_directive_*`（写进 solution 的 `directives.tcl`，或直接写 `#pragma HLS`） |
| `syn.compile.*` 等 | `config_compile ...`（例：`-pipeline_style`、`-enable_auto_rewind`、`-pragma_strict_mode`） |
| `syn.dataflow.*` | `config_dataflow ...` |
| `syn.interface.*` | `config_interface ...` |
| `package.output.format=ip_catalog` | `export_design -format ip_catalog` |
| 组件类型 / flow target | `open_solution <sol> -flow_target vivado`（Vivado IP 流，KV260 PL 默认用这个）或 `vitis` |
| `vitis-comp.json` | 无对应物；工程身份由 `hls.app` 承担，由 `open_project` 自动生成 |

最小 `run_hls.tcl` 模板（架构流程也用它，见 `reference/architect/design-layout.md`）：

```tcl
open_project -reset <proj_dir_name>
set_top <TOP_FUNCTION>
add_files src/kernel_hls.cpp
add_files -tb tb/testbench.cpp
open_solution -reset <solution> -flow_target vivado
set_part xck26-sfvc784-2LV-c
create_clock -period <CLOCK_NS> -name default
# 流程命令（csim_design / csynth_design / cosim_design / export_design）由 scripts/hls_run.sh 按 step 执行。
# run_hls.tcl 里也可以写流程命令：经 --tcl 载入时它们被替换成桩，参数作为本次调用的默认值（见 §2）
```
