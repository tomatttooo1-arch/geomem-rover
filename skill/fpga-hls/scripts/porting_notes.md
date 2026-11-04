# Ross HLS 脚本层 → Vitis HLS 2022.2 经典 Tcl 流程：移植说明（2026-11-02）

## 0. 来源

- 上游：AMD Ross agent skill 包（MIT），本地快照 `<upstream_copy>/`
- 上游包 `ross.tgz` sha256 = `a20046c7253b38dce9b4df8a3e9f93eedf1e6c62dc1686feff492692faab9d23`（与该目录 `SHA256SUMS` 一致，2026-11-02 复算）
- 所有照搬文件保留上游 `Copyright (C) 2026, Advanced Micro Devices, Inc.` 与 `SPDX-License-Identifier: MIT` 两行；改过的文件在其下加一行
  `# Modified for Vitis HLS 2022.2 classic flow by the contest team, 2026-11-02` 并列出改动点。新写的文件标 `Copyright (C) 2026 the contest team`，由上游语义派生的同时注明来源。

## 1. 为什么要改

上游的"跑工具 + 读报告"层按新版 Vitis Unified 写：`vitis-run --mode hls` / `v++ --compile --mode hls` 跑流程，
靠 `vitis-comp.json` + `hls_config.cfg` 找工程，solution 目录 = `<work_dir>/hls`。我们只有 Vitis HLS 2022.2
（bin 里没有 `vitis-run`），工程是 `open_project / open_solution / csim_design ...` 的 Tcl 写法，solution 目录 = `<proj>/<solution>`，工程目录里有 `hls.app`。

上游原样用在 2022.2 工程上的实际表现（2026-11-02 在工程 B 上实测，未改动的上游代码）：
- `hls_synth_report`：输出 `"Synthesis report file does not exist"`，退出码 0（找不到 solution 目录）。即使给对目录，2022.2 的 csynth.rpt 也会得到 `"Synthesis report content not found"`（分节标记不同，见 §3）。
- `hls_cosim_report`：`rows: []`，退出码 0 —— 空表冒充结果。
- `qor_log.py --report <2022.2 csynth.rpt>`：全部字段 -1 / `?`（列序与顶层行判据都不符）。

## 2. 文件清单（目录 `tools/agent_skills/fpga-hls/scripts/`）

| 文件 | 来源 | 说明 |
|---|---|---|
| `pylib/hls_common/*`、`hls_adb/*`、`hls_component_basic_info/*`、`hls_cosim_report/*`、`hls_dataflow/*`、`hls_design/*`、`hls_impl_report/*`、`hls_process_channel_cosim/*`、`hls_synth_report/*` | 上游 `skills/hls-scripts/` 全量复制 | 40 个 .py 逐字节照搬，2 个改动（下两行） |
| `pylib/hls_synth_report/report_info_getter.py` | 上游，**改** | ① 2022.2 的 csynth.rpt 把图例 `PS: '+' for module; 'o' for loop; '*' for dataflow` 放在汇总表**上方**（新版的 `Name Prefix: ...` 在下方），按上游标记会截不到表 → 加 2022.2 标记，截到下一个 `== ` 小节为止；② 综合日志：2022.2 没有 `<work_dir>/logs/hls_compile.log`，改为依次找 `agent_logs/csynth_*.log`（hls_run.sh 写的）→ `<sol>/<sol>.log` → `<proj>/vitis_hls.log`，上游路径存在时仍优先 |
| `pylib/hls_common/solution_folder_getter.py` | 上游，**改** | 上游解析失败（无 vitis-comp.json / cfg）时回退到经典布局：`<proj>/hls.app` 只有一个 solution → `<proj>/<sol>`；路径本身是 solution 目录（父目录有 hls.app）→ 原样。上游解析路径不变。这样上游自带的 `python3 -m hls_*.…_print` 入口在 2022.2 工程上也能用 |
| `pylib/hls_classic/locate.py` | 新 | 定位 solution 目录（顺序：工程目录 → solution 目录 → Vitis Unified 组件 → 裸 solution 目录）；多 solution 报错并列出；解析 hls.app（top、源文件/TB 文件与 cflags、csim argv）与 `<sol>.aps`（器件、时钟、cosim argv） |
| `pylib/hls_classic/reports.py` | 新 | 2022.2 报告的结构化摘要：综合读 `csynth.xml` + 各 `<module>_csynth.xml`（顶层/模块的估计时钟、延迟 best/avg/worst、Interval、资源、嵌套循环的 II / 深度 / trip / 延迟）；cosim 读 `<top>_cosim.rpt`；impl 读 `impl/report/verilog/export_impl.xml`(+.rpt)。值原样保留（`undef`/`?` 不转 0），缺文件抛 `MissingReport` |
| `pylib/hls_classic/cli.py` | 新 | `hls_report.sh` 的入口；输出 JSON = `summary`（上面的结构化数字）+ `upstream`（上游解析器原输出） |
| `hls_report.sh` | 新（替代上游各 `hls-*/scripts/run.sh`） | `hls_report.sh <basic-info|synth|cosim|impl|dataflow> <solution_dir|project_dir|component_dir>`；退出码 0 / 2 用法 / 3 缺报告（打印"缺 X 报告，先跑 Y"）/ 4 找不到 solution |
| `hls_run.sh` | 新（替代上游 hls-run-flow 的 `vitis-run`/`v++` 命令） | 见 §4 |
| `qor_log.py` | 上游 `skills/hls-optimize/scripts/qor_log.py`，**大改** | 新主模式 `--solution`：综合（csynth.xml + csynth.rpt 顶层行 slack/pipelined）、cosim 周期、impl 布线后周期与资源，真实延迟 = cosim 周期数 × 布线后周期（无 impl 时退到综合估计周期，并在 `real_period_basis` 列写明"非布线后实测"）；缺的数据留空 + 打印"缺 X"，不写 0/-1；没有综合结果时不写 CSV（退出码 3）；CSV 表头与本版不一致时拒写（退出码 5）。保留上游 `--report` 模式，但改为按表头定位列（2022.2 列序：Issue Type / Slack / Latency(cycles) / Latency(ns) / Iteration Latency / Interval / Trip / Pipelined / BRAM / DSP / FF / LUT / URAM，与上游硬编码列序不同；顶层行只有 dataflow 顶层才带 `*`） |
| `bitexact_check.py`、`bitexact.md` | 新（上游没有） | 逐字节输出门，见 `bitexact.md` |
| `calibration/calib_reports.py` | 新 | 校准仪器：用独立的正则解析文本 .rpt，与 hls_report.sh 的 XML 摘要逐项对表 |
| `calibration_20261102.md` | 新 | 校准记录 |

未复制：上游 `hls-matlab-to-cpp/scripts`（与报告/流程无关）、各 skill 的 SKILL.md / reference（文字层由另一代理负责）。

## 3. 2022.2 下的报告位置（实测）

| 内容 | 2022.2 经典流程位置 | 上游（Vitis Unified）位置 |
|---|---|---|
| solution 目录 | `<proj>/<solution>` | `<work_dir>/hls` |
| 综合汇总 | `<sol>/syn/report/csynth.rpt`、`csynth.xml`、`<module>_csynth.{rpt,xml}` | 同名（`<work_dir>/hls/syn/report/`） |
| 综合日志 | `<sol>/<sol>.log`（会话日志）；hls_run.sh 另写 `<sol>/agent_logs/csynth_*.log` | `<work_dir>/logs/hls_compile.log` |
| 设计层次/逐模块数据库 | `<sol>/.autopilot/db/<top>.design.xml`、`*.adb`、`process_info.csv`、`loop_info.csv`、`process_stalling_info/process.zip`、`loop_performance_info/loop.zip` | 同 |
| cosim | `<sol>/sim/report/<top>_cosim.rpt`（列：RTL / Status / 延迟 min avg max / Interval min avg max / Total Execution Time） | 同 |
| impl | `<sol>/impl/report/verilog/export_impl.rpt`、`export_impl.xml` | 同 |
| testbench argv | csim：`hls.app` 的 `<Simulation argv>`；cosim：`<sol>.aps` 的 `<Simulation argv>`；运行时 `csim_design -argv` / `cosim_design -argv` | `hls_config.cfg` 的 `csim.argv` / `cosim.argv` |

## 4. hls_run.sh 的做法

- 每个 step 起一个独立的 `vitis_hls -nolog -f <临时 Tcl>` 会话：`cd <proj 的父目录>; open_project <proj 名>; open_solution <sol>; <命令>`，命令包在 `catch` 里，成功/失败写 `AGENT_STEP_RESULT`。退出码以 vitis_hls 返回码为准，另外 csim 出现 `CSim done with N errors`(N>0)、cosim 出现 `co-simulation finished: FAIL`、impl 出现 `vivado returned an error` 都判失败（impl 那条是因为 2022.2 的 export_design 在 Vivado 崩溃后仍 `catch` 不到具体信息，见下）。
- 没有 `hls.app` 时必须给 `--tcl`：把用户的 run_*.tcl 在一个会话里 `source`，但事先把 `csim_design / csynth_design / cosim_design / export_design / close_project / close_solution / exit` 改名并换成"只记录参数"的桩，于是只有建工程那部分（open_project、add_files、set_top、open_solution、set_part、create_clock、config_*、set_directive_*）生效；之后还原真命令、`close_project` 落盘。桩记下的参数（如 `cosim_design -rtl verilog`）作为相应 step 的默认参数。脚本需要的环境变量由调用方设置。`hls.app` 已存在时 `--tcl` 被忽略（打印提示）。
- 前置检查照上游语义：cosim 前要 `<sol>/syn`，impl 前要 `<sol>/sim`；`open_solution` 一个不存在的名字会静默新建空 solution，所以先检查 `<proj>/<sol>` 存在。
- argv：`--argv` 优先；否则 csim 用 hls.app 的 argv，cosim 用 `<sol>.aps` 的 argv、为空时回退到 csim argv（对应上游 `cosim.argv ← csim.argv`，但我们不回写工程文件，只在命令行传 `-argv`）。
- impl：本机 Vivado 2022.2 在许可证库枚举 udev 设备时段错误（两套内存分配器混用，根因与修法见 `docs/design/M7_PL_BITSTREAM_20260927.md`）。hls_run.sh 对 impl step 默认 `LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1`（`HLS_RUN_NO_UDEV_PRELOAD=1` 关闭）。未加预加载时玩具核 impl 实测崩溃（`Abnormal program termination (11)`，栈在 `libudev ... malloc_usable_size`），hls_run.sh 正确报 `EXIT_CODE=1`。
- 日志：`<proj>/<sol>/agent_logs/<step>_<YYYYmmdd_HHMMSS>.log`（建工程那次是 `setup_*.log`）；终端只打每个 step 日志最后 30 行 + `STEP <step> EXIT_CODE=<rc>`。`--timeout <秒>` 给每个 step 设墙钟上限（超时退出码 124）。

## 5. 上游功能在 2022.2 下的可用性

| 上游功能 | 2022.2 下 | 说明 |
|---|---|---|
| `vitis-run` / `v++ --mode hls` 跑流程 | 不可用 | 由 hls_run.sh 代替 |
| IDE 内 `runHlsActiveComponentByType` | 不可用 | 没有 Vitis Unified IDE |
| `cosim.argv` 预检时回写 `hls_config.cfg` | 改为不回写 | 只在命令行传 `-argv` |
| init-component（建 `vitis-comp.json`） | 不需要 | 经典工程用 `hls.app`；hls_report.sh 两种布局都认 |
| 综合报告正文/日志/Pragma 小节 | 可用（已改） | 见 §2 |
| cosim 逐模块/逐循环 II 与延迟（process.zip / loop.zip） | 可用，未改 | 2022.2 在 `.autopilot/db` 里同样产出；顶层行与 cosim.rpt 对表一致（CALIBRATION §1） |
| dataflow 进程/通道信息 | 解析器可用，未改；**本次没有 dataflow 设计做阳性对照** | A/B/C 与玩具核都没有 dataflow 区域，输出为空列表并附说明"设计中没有 dataflow 区域"。对有 dataflow 的设计是否正确，未验证 |
| impl 报告（上游按 `== Place & Route Timing Summary` / `Resource Summary` / `Fail Fast` 切） | 可用，未改 | 2022.2 的 `impl/report/verilog/export_impl.rpt` 有同名小节；toy 上 postRoute=1.524 与 XML 一致（CALIBRATION §3） |
| `hls_component_basic_info` 的默认 include（`$XILINX_VITIS/include` 等） | 经典布局改为 `$XILINX_HLS/include` | 在 hls_classic/cli.py 里给出；上游函数对 Unified 布局不变 |

## 6. 2022.2 特有的坑（实测）

- **XML 里的 -1**：可变 trip 循环的未知上界在 `<module>_csynth.xml` 里写成 `-1`（文本报告印 "-"）。summary 把所有负的延迟/trip/迭代延迟报成 `"undef"`，不当数字用（mm 例子发现，CALIBRATION §1）。
- **Ross 例子用了 2022.2 没有的头文件**：`examples/hls-intro-matmul/main.cpp` 包含 `ap_float.h`，2022.2 csim 编译失败。上游其他 skill 文字里若出现 `ap_float`、`hls::vector` 新接口等，需按 2022.2 核实。
- **Vivado impl 段错误**：根因是许可证库里 libudev 的内存分配器混用（`docs/design/M7_PL_BITSTREAM_20260927.md`），hls_run.sh 对 impl 默认预加载 libudev。toy impl 在 121 字节长的 `impl/verilog` 路径下预加载后跑通，未预加载时崩溃栈在 libudev，说明本机这类段错误首先该查 libudev。另一个已知坑是路径过长超过 Unix socket 路径上限（约 108 字节）会让 Vivado/hw_server 崩：hls_run.sh 的 vitis_hls 工作目录固定用短路径 `/tmp/rh_XXXXXX`（忽略 TMPDIR）；但 Vivado 自己在 `<proj>/<sol>/impl/verilog` 下运行，这个路径由工程位置决定，脚本缩不短——impl 若仍崩且栈不在 libudev，先把工程复制到短路径（如 `<tmp>/<短名>/`）再跑。
- **2022.2 有/没有的文件**（实测 A 的 solution）：有 `.autopilot/db/process_info.csv`、`loop_info.csv`、`channel_info.csv`、`process_stalling_info/process.zip`、`loop_performance_info/loop.zip`（csynth 后即有，cosim 后 zip 有内容）；没有 `<work_dir>/logs/hls_compile.log`（已换来源，见 §2）；本脚本层不读 `inferred_directives.ini`。csynth.rpt 末尾有 `== Pragma Report`（含被忽略的 pragma），synth 的 `upstream.synthesisPragmaReportContent` 原样给出。
- **运行中不要改 hls_run.sh**：bash 按偏移增量读脚本，运行中改文件会让后半段解析错乱（实测一次）。

## 7. 交接

全部完成，无未交接项。遗留：dataflow 解析无阳性对照；impl 只在 toy 上验证过一次；`sim/wrapc_pc` 来自 RTL 只有源码级证据。
