# srch_v2：概率搜索核第二版（微程序引擎结构）

本目录是概率搜索核的第二版实现（面积优化版）。功能、外部接口（AXI-Lite 寄存器、参数块、结果块、128 位 m_axi）与
`hw/srch/`（c8 版）完全相同，回放向量逐字节一致；PS 侧驱动 `tools/srch/` 与块设计无需改动即可换用。
**本作品提交的整板比特流（`bitstream_m7_fullboard/`，app `m7_pl_m7s17`，构建号 0x063ba99b）使用本版搜索核：整板中 5,035 LUT / 51 DSP / 13 URAM**（布线后分层报告 `bitstream_m7_fullboard/utilization_route_hier.rpt`）。
`hw/srch/` 的 c8 版源码保留，作为优化前版本（上一版整板 0x06661def 使用它，整板中 9,754 LUT）。

## 1. 结构

c8 版把每种操作（初始化、观测、选点、导出……）各自展开成一套专用数据通路。第二版改为
“一个共用运算单元 ＋ 微程序 ＋ 两个专用单元”：

- **共用标量运算单元 ＋ 微程序顺序器**：48 位微指令（op / 立即数标志 / rd / ra / rb / 27 位立即数），64 位补码寄存器堆；
  初始化、先验、复位、导出、选点调度等控制密集的步骤全部写成微程序，由一套 ALU 分时执行。
  微程序由 `uprog6.py`（指令集 ＋ 两遍汇编器 ＋ 微程序本体）生成 `srch6_urom.h`（ROM 初值）与 `srch6_urom.lst`（反汇编清单）。
- **SCAN 单元**（窗口扫描）：几何前端、锥体 5 式、遮挡射线、动作，内层流水。
- **BEL 单元**（信念）：质量、视点增益、观测写回，流水执行。
- 片上状态（GRID / VISW / ENTRY / PST / MST）沿用 c8 的存储布局，深表放 URAM。

| 文件 | 内容 |
|---|---|
| `srch6_core.cpp` | 核顶层 `srch_top`、顺序器、SCAN / BEL 单元；同一份源码也是 C 模型 |
| `uprog6.py` | 微程序指令集、汇编器与微程序本体 |
| `srch6_urom.h` / `srch6_urom.lst` | 生成的微程序 ROM 与清单 |
| `srch6_tb.cpp` | 向量回放测试台（与 `hw/srch/srch_tb.cpp` 同格式） |
| `hls/run_hls.tcl`、`hls/build.sh`、`hls/run_slot.sh` | HLS 工程与构建入口 |
| `reports/` | 本版的 csynth、OOC 布局布线、联仿报告与拍数 |

## 2. 资源与时序（KV260 xck26，目标 10 ns，HLS `export_design -flow impl` 布局布线后）

| 版本 | LUT | FF | DSP | BRAM | URAM | 布线后周期 |
|---|---|---|---|---|---|---|
| c8（`hw/srch/`） | 9,683 | 13,012 | 79 | 2.5 RAMB36 | 13 | 9.667 ns |
| **第二版（本目录）** | **5,049** | **6,101** | **52** | 8 RAMB18（4.0 RAMB36 当量） | 13 | **8.331 ns** |
| 变化 | −47.9% | −53.1% | −34.2% | | 相同 | 时序余量 1.67 ns |

报告：`reports/ooc_export_impl.rpt`（汇总）、`ooc_utilization_routed.rpt`、`ooc_utilization_hier_routed.rpt`（分层）、
`ooc_timing_routed.rpt`；HLS 综合估计 `reports/csynth.rpt`。

## 3. 功能验证与拍数

- **C/RTL 联仿**：两组回放向量 V1 32 条、V2 39 条，共 **71/71 条记录与 c8 C 模型逐字节相同**（`reports/cosim.rpt`、`reports/cosim_tb_log.txt`）。
  每条记录是一次完整的核调用：调用前后整块 DDR 镜像逐字节比对，片上状态跨记录保留并经 DUMP 与 SELECT 结果间接逐位核对。
- **拍数**（联仿逐笔事务实测，100 MHz，`reports/cosim_cycles.txt`）：

| 操作 | 最长 | 时间 |
|---|---|---|
| 观测 OBSERVE | 81,895 拍 | 0.82 ms |
| 选点 SELECT | 122,039 拍 | 1.22 ms |
| 初始化（INIT_MAP / UNIV / FINAL / VIEWS 合计） | — | 223 ms |

## 4. 复现

向量为 `tools/srch/srch_harness.py` 录制的 trace（`trace_v1.bin`、`trace_v2.bin`），经 `SR_VEC` 传入（空格分隔）；
构建目录由 `SR_BUILD_ROOT` 指定。源码引用同级 `hw/srch/srch_core.h` 与 `hw/srch/srch_tb.cpp`，`build.sh` 会把它们与本目录源码一起快照进构建目录。

```bash
# 1) 重新生成微程序 ROM（可选，仓库内已带生成结果）
cd hw/srch_v2 && python3 uprog6.py

# 2) 软件自检（C 模型回放，g++）
g++ -O2 -std=c++14 -DSRCH6_STEPS -Wno-sign-compare -I$XILINX_HLS/include -Ihw/srch \
    -o tb_v2 hw/srch_v2/srch6_tb.cpp hw/srch_v2/srch6_core.cpp && ./tb_v2 trace_v1.bin trace_v2.bin
#    输出末行 "TB PASS"：V1 32 条、V2 39 条记录全部 0 mismatching

# 3) HLS：csim → csynth → cosim → OOC 布局布线（Vitis HLS 2022.2）
export SR_BUILD_ROOT=<构建目录> SR_VEC="<路径>/trace_v1.bin <路径>/trace_v2.bin"
bash hw/srch_v2/hls/build.sh srch_v2 csim csynth cosim impl
#   impl 一步即 export_design -flow impl -rtl verilog -format ip_catalog（KV260，10 ns），产出 IP 与布线后报告
#   需要排队取构建槽时用 hls/run_slot.sh，参数相同
```

`hls/run_hls.tcl` 是工程定义（顶层 `srch_top`、`xck26-sfvc784-2LV-c`、10 ns、
`config_interface -m_axi_addr64=false -m_axi_buffer_impl lutram -m_axi_conservative_mode=false`）；
直接用 `vitis_hls` 时，在其后依次执行 `csim_design`、`csynth_design`、`cosim_design -rtl verilog`、
`export_design -flow impl -rtl verilog -format ip_catalog` 即可得到同样的四步结果。
