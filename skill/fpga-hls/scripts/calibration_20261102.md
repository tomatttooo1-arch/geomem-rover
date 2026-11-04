# 脚本层校准记录（2026-11-02，Vitis HLS 2022.2，xck26-sfvc784-2LV-c）

目的：在已知答案上确认 `hls_report.sh` / `hls_run.sh` / `qor_log.py` / `bitexact_check.py` 报的数是对的、该报错时会报错。
试跑全部在草稿目录 `.scratch_ross_scripts/`（不提交）里新建或复制工程；原 runs 目录只读。逐项明细表在 `calibration/results_20261102/`。

工程：
- A = `<runs>/m7-hls-bringup-20260916/out/fpga_backproject-10ns`（反投影核，09-16）
- B = `<runs>/m7-core-20260921-worktree/runs/m7-frame-20260921/v1b/hls_k26_10ns`（深度帧核）
- C = `<runs>/m7-core-20260921-worktree/runs/m7-display-20260921/hls-v2/hls_k26_10ns`（显示叠加核）
- toy = `calibration/toy_kernel/`（64 元素 `ap_uint<8>` 平方加偏置，II=1，可用宏注入两种错误）
- mm = Ross 自带 `examples/hls-intro-matmul`（经典 Tcl 改写，见第 3 节）

## 1. 报告解析对表（synth / cosim）

仪器：`calibration/calib_reports.py`。期望值用独立正则从**文本**报告读：`<top>_csynth.rpt`（目标/估计时钟、延迟 min/max、Interval min/max、Total 资源）、`csynth.rpt` 汇总表（每个模块行的延迟/Interval/五类资源，每个循环行的 II、迭代延迟、延迟、trip）、`<top>_cosim.rpt`（状态、延迟与 Interval 的 min/avg/max、Total Execution Time）。实际值是 `hls_report.sh` 的输出：`summary` 来自 **XML**（csynth.xml、各 `<module>_csynth.xml`），cosim 的 `upstream` 顶层行来自 `.autopilot/db` 的 process.zip（上游解析器）。两边来源不同，所以对得上才有意义。"?"/"-"/"undef" 视为同一个"未知"；资源列 "-" 视为 0。

| 工程 | 期望关键数（文本报告） | 对表项数 | 不一致 | 结果 |
|---|---|---|---|---|
| A | 估计时钟 7.016 ns；顶层延迟 ?/?（可变 trip）；DSP 43 / FF 6196 / LUT 4536 / BRAM 0 / URAM 0；循环 VITIS_LOOP_20_1 II=2、迭代延迟 36；cosim Pass，延迟 42/43/45，Interval 46，总 88 周期 | 41（2 模块、1 循环） | 0 | 通过 |
| B | 估计时钟 7.458 ns；顶层延迟 2 / 583745810，Interval 3 / 583745811；BRAM 6 / DSP 59 / FF 6724 / LUT 10601；12 个循环（含嵌套 VITIS_LOOP_114_6 下 3 个、add_ray 内 VITIS_LOOP_38_2 II=3）；cosim Pass，延迟 2/47226/99134，总 330588 周期 | 134（9 模块、12 循环） | 0 | 通过 |
| C | 估计时钟 7.068 ns；顶层延迟 ?/?；DSP 40 / FF 10582 / LUT 7453；2 个循环；cosim Pass，最大延迟 30765，总 68981 周期 | 66（5 模块、2 循环） | 0 | 通过 |
| toy | 估计时钟 1.882 ns，延迟 69，Interval 70，DSP 1 / FF 167 / LUT 129；cosim Pass 67 周期 | 34 | 0 | 通过 |
| mm | 估计时钟 7.016 ns；3 个循环（含可变 trip 的 VITIS_LOOP_19_5，II=3） | 44 | 0（修正后，见下） | 通过 |

仪器自身的判别力（阴性对照，必须报不一致）：
- `--perturb`（把 B 顶层 DSP 实际值 +1）→ 报 `top DSP 59 vs 60 不一致`，退出码 1。
- 手改 B 的 JSON：循环 VITIS_LOOP_38_2 II 3→4、cosim 上游顶层 max_latency 99134→1 → 两项都报不一致，退出码 1。
- 仪器第一版有个 bug：汇总表行过滤写错，模块/循环行一行没读进来，却因为只剩顶层项而"看似可用"；是"XML 多出的循环在 rpt 里没有"这一检查把它暴露出来的。修正后才得到上表。

mm 暴露的真问题（已修）：2022.2 的 XML 对可变 trip 循环把未知上界写成 `-1`（`<Latency><range><min>3</min><max>-1</max>`），文本报告印 "-"。第一版 summary 原样输出 `-1`，下游会把它当成一个数。现在负值一律报 `"undef"`（`reports.py::_unknown_if_negative`），修后 A/B/C/toy/mm 全部重跑仍 0 不一致。

同一工具重跑的已知答案：用 `hls_run.sh --tcl hw/geometry/run_hls.tcl` 从源码重建 A（源码自 09-16 后未改）跑 csim+csynth：csim `CSim done with 0 errors`；新 `csynth.rpt` 去掉前 12 行头（日期）后与原 A **逐字节相同**；两边 `hls_report.sh synth` 的 summary（去掉路径字段）完全相等。

## 2. 阴性对照：只有 csim 的 solution

输入：`<runs>/nationals-prelim-20261102/B1/hls/csim2`（无 syn/sim/impl）。期望：明确报缺，不输出任何数。

| 命令 | 实际 | 结果 |
|---|---|---|
| `hls_report.sh synth <工程目录>` 与 `<solution 目录>` | 退出码 3，`缺 综合(csynth) 报告（…/syn/report/csynth.rpt），先跑 csynth`；JSON 只有 error/message/run_first/path | 通过 |
| `hls_report.sh cosim` | 退出码 3，`缺 协同仿真(cosim) 报告 …，先跑 cosim` | 通过 |
| `hls_report.sh impl` | 退出码 3，`缺 实现(impl …) 报告 …，先跑 impl` | 通过 |
| `hls_report.sh dataflow` | 退出码 3，`缺 设计层次(.autopilot/db/*.design.xml) …，先跑 csynth` | 通过 |
| `hls_report.sh basic-info` | 退出码 0，top=fpga_b1_costmap、器件 xck26…、`steps_done: csim 真，其余假`（基本信息不依赖报告，正确） | 通过 |
| `qor_log.py --solution` | 退出码 3，三条"缺"，**不写 CSV** | 通过 |
| `hls_run.sh <副本> solution cosim` / `impl` | 退出码 2，打印"先跑 csynth / 先跑 cosim"，未启动 vitis_hls | 通过 |

对照：同一类输入下**未改动的上游**行为——`hls_synth_report` 在 B 上输出 `"Synthesis report file does not exist"` 退出码 0；`hls_cosim_report` 输出 `rows: []` 退出码 0；上游 `qor_log.py --report <B 的 csynth.rpt>` 全部 -1。

## 3. 跑流程

| 运行 | 期望 | 实际 | 结果 |
|---|---|---|---|
| A 从 Tcl 重建：`hls_run.sh --tcl hw/geometry/run_hls.tcl … solution csim csynth` | csim 过；csynth 与原 A 一致 | 两步 EXIT_CODE=0，约 50 s；结果见第 1 节（逐字节一致） | 通过 |
| A 在已有 hls.app 上再跑 `csynth`（带 --tcl） | 打印"--tcl ignored"，重开工程综合 | EXIT_CODE=0 | 通过 |
| toy `--tcl … csim csynth cosim`（**本次唯一的 cosim**） | 三步过，cosim PASS | 三步 EXIT_CODE=0，`C/RTL co-simulation finished: PASS`，约 110 s；cosim 选项 `-rtl verilog` 取自用户 Tcl 的桩记录 | 通过 |
| toy `impl`（未加 libudev 预加载） | — | Vivado 段错误（栈 `libudev … malloc_usable_size`，与 `docs/design/M7_PL_BITSTREAM_20260927.md` 记录一致），hls_run.sh 报 `EXIT_CODE=1` | 失败被正确报出 |
| toy `impl`（hls_run.sh 默认 `LD_PRELOAD=libudev`） | 跑完，有 export_impl 报告 | step EXIT_CODE=0，约 8 min；布线后 CP 1.524 ns、时序满足、LUT 21 / FF 49 / DSP 1 / BRAM 0 / URAM 0（XML 与文本 `export_impl.rpt` 的 Post-Route 行和 Resource Summary 逐项一致；上游 impl 解析器 postRoute=1.524 也一致） | 通过 |
| Ross 例子 mm 原样（`csim_design -argv {8 8 8}`） | — | csim 编译失败：`ap_float.h: No such file or directory`（2022.2 没有这个头文件，例子是按 2026.1 写的），hls_run.sh 报 `EXIT_CODE=1` | 失败被正确报出 |
| mm（testbench 把 ap_float 的 ULP 比较换成 memcpy 位转换，核本身未改） | csim 过、出综合报告 | csim `PASS`（main 要求 argc==4，所以 argv 确实从 Tcl 桩传到了 `-argv`），csynth 过，约 65 s；估计时钟 7.016 ns，DSP 8 / FF 816 / LUT 856 | 通过（只证明流程能跑通，数字无原报告可对） |

附带一次失效：toy impl 第二次运行结束后 hls_run.sh 自身报 `syntax error near unexpected token do`、总退出码 2——原因是运行期间我改写了 hls_run.sh 文件本身（bash 按偏移增量读脚本）。impl step 本身已 `EXIT_CODE=0`。脚本 `bash -n` 通过，之后的所有运行正常。**运行期间不要改脚本**。

## 4. 逐位比对的阳性/阴性对照（toy）

golden = toy 基线 csim 输出 `csim/build/tb_out/`（out.bin 128 字节 = 64 个 uint16；out.txt 534 字节）。完整输出 `calibration/results_20261102/bx_result.txt`。

| 对照 | 期望 | 实际 | 结果 |
|---|---|---|---|
| 基线 cosim 输出 `sim/wrapc_pc/tb_out` vs golden | 一致 | 两个文件 SAME，`RESULT BITEXACT`，退出码 0 | 通过 |
| 无改动重建重跑 csim vs golden | 一致 | SAME ×2，退出码 0 | 通过 |
| 注入错误 1：第 37 个输出 +1 | 不一致，偏移 74（=37×2） | `DIFF out.bin first difference at byte offset 74 (golden 0x0b vs candidate 0x0c) element 37`；out.txt 第 38 行；退出码 1 | 通过 |
| 注入错误 2：中间量 ap_uint<16> 截成 ap_uint<15> | 不一致，第一个 ≥32768 的元素：in[5]=196，196²−5=38411=0x960B → 0x160B | `DIFF out.bin … offset 11 (golden 0x96 vs candidate 0x16) element 5`；out.txt 第 6 行、长度 534→528；退出码 1 | 通过 |
| golden 目录为空 | 不得判一致 | 退出码 2，"golden 目录为空…不能判一致" | 通过 |
| candidate 目录不存在 | 不一致 | 退出码 1 | 通过 |

`sim/wrapc_pc/tb_out` 来自 RTL 的依据：`sim/wrapc_pc/apatb_toy.cpp` 以 `POST_CHECK` 编译，定义 `AUTOTB_TVOUT_PC_out_r "../tv/rtldatafile/rtl.toy.autotvout_out_r.dat"`，即从 RTL 仿真输出向量读回结果再交给 testbench。这是源码级证据；**没有做"篡改 RTL 输出向量后看 wrapc_pc 输出是否跟着变"的直接实验**（该 .dat 是 2022.2 的二进制格式，未解开）。

## 5. 未覆盖

- dataflow 解析：A/B/C/toy/mm 都没有 dataflow 区域，只验证了"无 dataflow 时明确说明、不出空表冒充"，有 dataflow 的设计未做阳性对照。
- impl 只在 toy 上跑过一次（很小的设计）；A/B/C 盘上没有 impl 报告。
- qor_log 的真实延迟：toy 上 67 周期 × 1.524 ns = 102.1 ns，手算一致；其余工程无 impl，按综合估计周期计并在 `real_period_basis` 列写明"非布线后实测"。
