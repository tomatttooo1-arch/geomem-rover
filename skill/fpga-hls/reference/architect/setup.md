> 来源：AMD Ross `hls-architect/reference/setup.md`，MIT 许可，见 `../../upstream.md`。已按 Vitis HLS 2022.2 移植：
> 上游 Step 1 查 2025.x 的命令行工具、Step 2 从 HLS cfg 文件取 top/clock/part、Step 3 用 2025.x 的单命令流程；
> 2022.2 改为 `source settings64.sh` + `vitis_hls`、从 `run_hls.tcl`（或已有工程的 `hls.app`）取设置、流程走 `scripts/hls_run.sh`。
> MATLAB 相关内容（上游 hls-matlab-to-cpp）未移植。
> 上游描述：Set up the HLS design environment — verify Vitis, OpenCV, discover design config, detect build system. Run once at the start of any hls-architect, csim, csynth, cosim, or optimize session.
<!--
Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
-->

# Setup（2022.2 版）

为一次 HLS 设计会话准备环境。运行时机：
- **架构 / 跑流程 / 优化模式开工时**（不带参数）：完整 setup，含设计发现。
- **其他场合**带 `mode=tooling-only`：只查工具与 OpenCV，跳过设计发现。

带 `mode=tooling-only` 时**跳过 Step 2（设计发现）与 Step 3（流程命令）**，只跑 Step 1 与 Step 4。

开头打印：
```
─────────────────────────────────────────────────────
[fpga-hls/setup]  required tooling
  Step 1  — Verify Vitis HLS 2022.2   (source settings64.sh, exports $XILINX_HLS)
  Step 4  — Verify OpenCV             (only if testbench uses cv::imread)
[fpga-hls/setup]  design configuration  (skipped in tooling-only mode)
  Step 2  — Discover the design
  Step 3  — Build commands
─────────────────────────────────────────────────────
```

---

## Step 1：确认 Vitis HLS 2022.2 环境

```bash
source /tools/Xilinx/Vitis_HLS/2022.2/settings64.sh
which vitis_hls && echo "XILINX_HLS=$XILINX_HLS"
ls "$XILINX_HLS/include/ap_fixed.h" "$XILINX_HLS/include/hls_stream.h"
```

- 期望：`XILINX_HLS=/tools/Xilinx/Vitis_HLS/2022.2`，`vitis_hls` 在 PATH 上，头文件存在。
- 头文件目录是 `$XILINX_HLS/include`。不要用 `$XILINX_VITIS/include`（本机那里没有 `ap_fixed.h`）。
- 本机只有 2022.2 这一个版本；2025.x 的命令行工具与 Unified IDE 不存在，找不到不是故障，不要去装或去找。

`vitis_hls` 不在 PATH 上就停下报告，不要继续。

Step 1 后打印：
```
─────────────────────────────────────────────────────
[fpga-hls/setup]  Step 1 — done
  ✓ Step 1  Vitis HLS 2022.2 on PATH  (<which vitis_hls>)
  ← NEXT    Step 2 — Discover the design
─────────────────────────────────────────────────────
```

---

## Step 2：发现设计

向用户要设计目录（用户已给就用）。在其中找工程定义，按下面顺序：

1. **已有工程**：找 `hls.app`（工程根标志）。
   ```bash
   find <design_dir> -maxdepth 3 -name hls.app 2>/dev/null
   ```
   top 与源文件可用 `scripts/hls_report.sh basic-info <solution_dir>` 取（见 `reference/read-qor.md` §basic-info）。
2. **Tcl 脚本**：找 `run_hls.tcl`（或其他含 `open_project` 的 `.tcl`）。
   ```bash
   grep -l "open_project" <design_dir>/*.tcl <design_dir>/*/*.tcl 2>/dev/null
   grep -E "^\s*(open_project|set_top|add_files|open_solution|set_part|create_clock)" <run_hls.tcl>
   ```
3. 两者都没有（全新设计，例如架构流程从 `frame_based/` 开始）：设置从用户处拿，由 `reference/architect/design-layout.md`
   生成 `run_hls.tcl` 模板。

找到多个工程或脚本时问用户，不要挑一个猜。

提取并在会话中保持：
- `TCL_FILE` — 工程脚本路径（如 `run_hls.tcl`；只有 `hls.app` 时为空）
- `PROJ_DIR` / `SOLUTION` — `open_project` 的目录名 / `open_solution` 的名字（或 `hls.app` 里的 `<solution name=...>`）
- `TOP_FUNCTION` — `set_top` 的参数；缺失就问用户
- `CLOCK_NS` — `create_clock -period` 的值（ns）；缺失就问用户，不预设
- `XPART` — `set_part` 的参数；缺失时用 KV260 `xck26-sfvc784-2LV-c` 并明说
- `SRC_FILES` — 不带 `-tb` 的 `add_files` 条目；`TB_FILES` — 带 `-tb` 的条目

Step 2 后打印：
```
─────────────────────────────────────────────────────
[fpga-hls/setup]  Step 2 — done
  ✓ Step 1  Vitis HLS 2022.2 on PATH
  ✓ Step 2  top=<TOP_FUNCTION>  tcl=<TCL_FILE>  proj=<PROJ_DIR>/<SOLUTION>  part=<XPART>  clock=<CLOCK_NS> ns
  ← NEXT    Step 3 — Build commands
─────────────────────────────────────────────────────
```

---

## Step 3：流程命令

所有流程统一走 `scripts/hls_run.sh`（脚本层提供，用法见 `reference/run-flow.md` §2）：

| 流程 | 命令 | 对应 Tcl |
|---|---|---|
| C 仿真 | `scripts/hls_run.sh <PROJ_DIR> <SOLUTION> csim [--tcl <TCL_FILE>]` | `csim_design` |
| C 综合 | `scripts/hls_run.sh <PROJ_DIR> <SOLUTION> csynth [--tcl <TCL_FILE>]` | `csynth_design` |
| 联合仿真 | `scripts/hls_run.sh <PROJ_DIR> <SOLUTION> cosim [--tcl <TCL_FILE>]` | `cosim_design` |
| 实现 | `scripts/hls_run.sh <PROJ_DIR> <SOLUTION> impl [--tcl <TCL_FILE>]` | `export_design -flow impl` |

工程还没建（没有 `hls.app`）时带 `--tcl`。不要自己拼 `vitis_hls -f`，不要用 `make csim` 之类（不是每个设计都有）；
脚本缺失或报错就停下报告。

**优化 / 架构会话的实验区**：改源码、跑流程都在 `session/hlsopt_session.sh start` 建的 worktree 里进行
（见 `reference/optimize/init-component.md`、`reference/architect.md`），不要在用户的工作副本里直接改。

报告位置（2022.2）：csynth 在 `<PROJ_DIR>/<SOLUTION>/syn/report/`，csim 在 `<PROJ_DIR>/<SOLUTION>/csim/report/`，
原始日志 `<PROJ_DIR>/<SOLUTION>/<SOLUTION>.log`，脚本日志 `<PROJ_DIR>/<SOLUTION>/agent_logs/`。读法见 `reference/read-qor.md`。

Step 3 后打印：
```
─────────────────────────────────────────────────────
[fpga-hls/setup]  Step 3 — done
  ✓ Step 1  Vitis HLS 2022.2 on PATH
  ✓ Step 2  Design discovered
  ✓ Step 3  Build commands ready (scripts/hls_run.sh)
  ← NEXT    Step 4 — OpenCV dependency check
─────────────────────────────────────────────────────
```

---

## Step 4：OpenCV 依赖检查

先看是否需要 OpenCV：

```bash
grep -rl "opencv\|cv\.h\|highgui\|imgproc\|imgcodecs" . --include="*.cpp" --include="*.hpp" --include="*.h" 2>/dev/null | head -5
```

**没有匹配 → 本步记 `N/A`，跳过其余部分。**

**有匹配**则需要 OpenCV。按顺序解析路径：

1. **环境里已有？** `$OPENCV_INCLUDE`、`$OPENCV_LIB` 已设且头文件存在就复用。
2. **否则问用户**两个问题：
   - "Path to OpenCV include directory (containing `opencv2/` subfolder):"
   - "Path to OpenCV lib directory (containing `libopencv_core.so`):"

   然后 `export OPENCV_INCLUDE=<answer>`、`export OPENCV_LIB=<answer>`。

最后：

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$OPENCV_LIB
ls "$OPENCV_INCLUDE/opencv2/core.hpp"
ls "$OPENCV_LIB"/libopencv_core.so*
```

这些环境变量由下游（优化、csim、csynth）继承，下游不要重复 export。

还要确认 `TCL_FILE` 把 OpenCV 路径传给了编译与链接：2022.2 写法是
`add_files -tb tb/testbench.cpp -cflags "-I<OPENCV_INCLUDE>"`，以及 `csim_design` / `cosim_design` 的
`-ldflags "-L<OPENCV_LIB> -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -Wl,-rpath,<OPENCV_LIB>"`
（见 `reference/architect.md` Step 2d、`reference/run-flow.md` §4）。没传会在编译时报找不到头文件。

---

## 最终汇总打印

```
─────────────────────────────────────────────────────
[fpga-hls/setup]  done
  ✓ Step 1  Vitis HLS  : 2022.2  (<which vitis_hls>)
  ✓ Step 4  OpenCV     : <OPENCV_INCLUDE>   (or "N/A — testbench has no OpenCV")
  ✓ Step 2  Design     : top=<TOP_FUNCTION>  tcl=<TCL_FILE>  part=<XPART>  clock=<CLOCK_NS> ns   (skipped if tooling-only)
  ✓ Step 3  Build      : scripts/hls_run.sh ready                                               (skipped if tooling-only)
─────────────────────────────────────────────────────
```
