> 来源：AMD Ross `hls-component-basic-info`、`hls-synth-report`、`hls-cosim-report`、`hls-impl-report`、
> `hls-dataflow-infos`（作者：Alex Li），MIT 许可，见 `../upstream.md`。五个"读报告"skill 在上游都是同一个
> Python 脚本包（`hls-scripts/`）的薄壳，这里合并为一个文件，调用改为 `scripts/hls_report.sh`。已按 Vitis HLS 2022.2 移植。

# 读 HLS 报告（basic-info / synth / cosim / impl / dataflow）

所有报告取数走一个入口：

```bash
scripts/hls_report.sh <kind> <solution_dir>     # kind ∈ basic-info | synth | cosim | impl | dataflow
```

`<solution_dir>` = `<proj_dir>/<solution>`（2022.2 Tcl 工程；`<proj_dir>` 下有 `hls.app`）。
上游的 `component_location`（含 `vitis-comp.json` 的目录）和 Vitis Unified IDE 的
`getActiveComponentLocation` / `getHLSSynthesisInfo` / `getHLSCoSimulationInfo` / `getHLSImplementationInfo`
在 2022.2 都不存在：用户没给 solution 目录就问用户，不要猜。

**缺报告时**：`hls_report.sh` 会明说缺哪个文件。此时先按 `reference/run-flow.md` 补跑对应流程，
不要拿别的 solution、旧时间戳的报告或估计值顶替。

**上游的硬规则保留**：读 QoR 一律通过这个入口，不要用文件读取工具零散地抠 `.rpt`——保证每次取数口径一致。
脚本没覆盖、但诊断需要的原始材料（`verbose.sched.rpt`、原始 solution 日志里的 `Pipelining` 段）
按下文路径直接读，并在结论里写出处路径。

---

## 1. basic-info（上游 hls-component-basic-info）

用途：拿到设计的基本信息——组件（工程）名、源文件、testbench 文件、include 路径、顶层函数。
上游输出字段（`hls_report.sh basic-info` 尽量保持同名）：

- `component_name` — 2022.2 下对应工程名（`hls.app` 里 `<AutoPilot:project name=...>`）
- `source_files` — `hls.app` 中 `tb="false"` 的 `<file name=...>`
- `testbench_files` — `hls.app` 中 `tb="1"` 的文件
- `include_paths` — 各文件 `cflags` 里的 `-I...`
- `top_function` — `hls.app` 的 `top=` 属性（= Tcl 的 `set_top`）

2022.2 的 `hls.app` 是 XML，形如（本机已有工程）：

```xml
<AutoPilot:project ... name="hls" top="fpga_m7_frame_axi">
  <files>
    <file name=".../tb_frame_stream.cpp" tb="1" cflags=" -std=c++11 ..." csimflags="..."/>
    <file name=".../m7_frame_stream.cpp" tb="false" cflags="-std=c++11 ..." csimflags=""/>
  </files>
  <solutions><solution name="solution" status=""/></solutions>
  <Simulation argv=""> ... </Simulation>
</AutoPilot:project>
```

工程如果只有 `run_hls.tcl`、还没跑过（没有 `hls.app`），basic-info 从 `run_hls.tcl` 的
`set_top` / `add_files [-tb] [-cflags]` 读。

## 2. synth（上游 hls-synth-report）

前提：csynth 已完成，`<solution_dir>/syn/report/csynth.rpt` 存在（上游写的 `hls/syn/report/csynth.rpt` 与
`../logs/hls_compile.log` 是 2025.x 组件布局，2022.2 没有）。

上游输出三段，对应 2022.2 的数据源：

| 上游字段 | 内容 | 2022.2 来源 |
|---|---|---|
| `synthesisReportContent` | 性能/资源估计、循环表（II、延迟、trip count、Pipelined 列） | `<solution_dir>/syn/report/csynth.rpt`（截到模块/循环细节段之前）；每个模块另有 `<module>_csynth.rpt` |
| `synthesisPragmaReportContent` | Pragma Report：从 `== Pragma Report` 到文件尾 | 同上 `csynth.rpt` 末段。2022.2 **有**这一段，含 **Ignored Pragmas** 表（类型、选项、位置、原因）；用了 `performance` pragma 时，性能段里还有 **Performance Pragma Report**（Target TI / TI / TI met） |
| `synthesisLogContent` | 综合日志，去掉噪声 `INFO:` 行 | `<solution_dir>/<solution>.log`（另有工作目录下的 `vitis_hls.log`；经 `scripts/hls_run.sh` 运行时在 `<solution_dir>/agent_logs/`） |

注意过滤掉 `INFO:` 会同时去掉 `Pipelining result`（`HLS 200-1470`）、
`Successfully applied performance pragma`（`HLS 200-1957`）、`Inferring pragma ... due to performance pragma`
（`HLS 214-268/269`）这类信息行；需要它们时读原始日志（见 `reference/ii-diagnose.md`）。

2022.2 Ignored Pragmas 表的样子（本机实测）：

```
== Pragma Report
* Ignored Pragmas
| Type     | Options      | Location                    | Messages                                           |
| pipeline | II=1 rewind= | t3_rewind_true.cpp:4 in top | unexpected pragma argument '=', expects identifier |
```

## 3. cosim（上游 hls-cosim-report）

前提：cosim 已完成。上游的前提文件 `process_info.csv` 在 2022.2 位于 `<solution_dir>/.autopilot/db/process_info.csv`
（上游写的 `hls/.autopilot/...` 是 2025.x 组件布局）。它和 `loop_info.csv`、`process_stalling_info/process.zip`、
`loop_performance_info/loop.zip` 在 **cosim 后**出现在 solution 下（本机实测见
`<runs>/m7-hls-bringup-20260916/out/fpga_backproject-10ns/solution/.autopilot/db/`；
是否生成可能因设计与 cosim 选项而异），`hls_report.sh cosim` 的进程/通道解析依赖它们，缺失时脚本以退出码 3 明说。
2022.2 相关文件是：

- `<solution_dir>/sim/report/<top>_cosim.rpt` — 联仿结果（Pass/Fail、latency 与 interval，单位 cycle）
- `<solution_dir>/.autopilot/db/channel_info.csv`、`channel_depth_info/` — dataflow 通道信息
- 用 `cosim_design -enable_dataflow_profiling` 跑时，会额外产生通道占用的剖析数据

上游输出字段：`coSimulationReportContent`（cosim.rpt 内容）、`dataflowProcessAndChannelContent`
（dataflow 进程/通道表）。读 cosim 周期时与 csynth 估计对比：有内存反压的设计 cosim 可能显著差于 csynth 估计；
计算受限的 kernel 两者应接近。

**逐位一致不在这里判**：cosim 的 Pass 只说明 testbench 判据通过；逐位一致门见 `reference/optimize.md`
（`scripts/bitexact_check.py`）。

## 4. impl（上游 hls-impl-report）

前提：`export_design -flow impl` 已完成（`scripts/hls_run.sh ... impl`）。上游前提文件
`hls/impl/report/verilog|vhdl/export_impl.rpt` 是 2025.x 布局。2022.2 下：

- `<solution_dir>/impl/report/verilog/<top>_export.rpt` — 导出摘要（资源、`CP required` / `CP achieved ...`、Timing met）
- `<solution_dir>/impl/verilog/report/` — Vivado 详细报告（`*_utilization_*`、`*_timing_*`、`*_failfast_*`）

上游输出字段：

```json
{
  "postRoute": "...",                  // Post-Route 时钟周期（ns）
  "placeRouteResourceSummary": "...",  // 布局布线后 LUT, FF, DSP, BRAM, URAM, SRL
  "placeRouteFailFast": "..."          // Fail Fast 表：LUT / FD / DSP 百分比
}
```

**陷阱（本机核实）**：`export_design -flow syn` 产出的 `<top>_export.rpt` 只有
`#=== Post-Synthesis Resource usage ===` 和 `CP achieved post-synthesis`，**没有** post-route 数——
不能当成 impl 结果用。基线里的 post-route clock period 必须来自 `-flow impl`；
`hls_report.sh impl` 报"缺 post-route"时去补跑 impl，不要拿 post-synthesis 数顶替。
（`-flow impl` 产物的段名本机尚未见到实例，以 `hls_report.sh impl` 的解析为准，见 `../porting_notes.md`。）

true latency = cosim cycles × post-route period (ns)。

## 5. dataflow（上游 hls-dataflow-infos）

用途：列出每个 dataflow 模块的进程与通道（名称、类型、深度、读写进程等）。
上游输出是 JSON 数组，每个元素对应一个 dataflow 模块。

2022.2 数据源：综合后 `<solution_dir>/.autopilot/db/` 下的 `*.adb.xml` / `*.sched.adb.xml` 与
`channel_info.csv`、`channel_depth_info/`；跑过 cosim 后还有 `process_info.csv`、`loop_info.csv`（见 §3）。
进程/通道表由 `hls_report.sh dataflow` 解析；缺材料时它以退出码 3 明说，此时用 `csynth.rpt` 的模块层级 + 源码中的
dataflow 区域人工核对。

架构流程 Step 3a 的"dataflow 拓扑"检查读的就是这一项（综合之后才有）；综合之前的 canonical
规则检查见 `reference/dataflow.md`。
