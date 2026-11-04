# M7 有界射线帧证据核（软件与 HLS 验证阶段）

这里实现的是**单帧**射线穿越证据：`ray_flags.cpp` 给网格内的终点标记命中，把确实穿越的中间体素标记为空闲。它不保存跨帧地图、不计算占据概率，也不等同于完整的 OctoMap、板上障碍更新或显示链。原 `hw/geometry` 保留浮点反投影参照；主机包装器 `host/m7_native.py` 把深度经原参照转换为定点射线，再调用此核。

## C ABI 与边界

`ray_flags.hpp` 声明 `fpga_m7_ray_flags(origins, ends, ray_valid, count, nx, ny, nz, flags, status)`。`origins`、`ends` 是各容纳 4096 个 `[x,y,z]` 三元组的 `int32` 非别名缓冲；`ray_valid` 是容纳 4096 个 `uint32` 的缓冲，只允许 0/1。`count <= 4096`，三个维度各在 1～32；这是本阶段的验证容量，**不是产品地图尺寸决定**。每个坐标必须在 `[0, 维度×256)` 内，一个体素等于 256 个整数单位。输出为最多 `32³` 个 `uint8`，按 `(z×ny+y)×nx+x` 排列：0 未知、1 穿越空闲、2 射线命中。

每条有效射线的原点与终点必须都在网格内；无效射线跳过，其坐标不读取。`status=0` 才能消费输出；`1` 是维度/数量非法，`2` 是非法有效性标志，`3` 是有效射线的端点或原点越界。错误时先拒绝整批输入，不写 `flags`，上层必须丢弃该帧，不能复用缓冲里的旧数据。成功调用会先清空当前网格，然后给每个体素每帧最多一个证据值；命中总是压过空闲，与射线顺序和重复输入无关。起点所在体素保守不清空。多个主机分块的证据用 0 < 1 < 2 逐体素合并；主机对网格外的终点明确丢弃、计数，绝不把它们转换为空闲。

**整帧核 `frame_depth.cpp` 自 2026-10-11 起不同**：终点在网格外的射线不再丢弃，而是从相机格出发按同一穿越规则走到出格为止，
格内经过的格子标空闲、不标命中，计为「被裁剪」（`counts[2]`，名字仍叫 outside）。见 `docs/design/M7_CLIP_SPEC_20261011.md`。
上面一段仍只描述分块射线核 `ray_flags.cpp` 与主机 `NativeM7`（冻结的旧规则）。

整数 DDA 用交叉积比较穿越边界的时间。恰好同时穿越两条或三条边界时一起跨轴，不能将只擦过角/边的体素标为空闲；恰好位于体素边界的静止坐标按半开区间归到正向一侧。**未知不是已清空**：从未穿越、无效深度、遮挡、越界或缺少观测时必须保留未知；此核不能直接将物体从长期记忆中判定为消失。

`NativeM7.frame_from_depth(...)` 的输入使用米制深度、相机内参、刚体位姿、网格原点和体素米制边长。调用方必须给帧号、采集与位姿时间戳、标定版本，并先判定位姿可信且时间对齐；包装器另拒绝非物理内参/位姿和网格外相机原点。原参照用 `float32` 反投影，包装器再向下取整到 Q8；仅最后一步的**每轴量化**误差小于 `voxel_size_m/256`。此界**不包含**前面的 `float32` 投影、标定或位姿误差。靠近体素边界时，量化后所属体素可能和连续空间理想线不同；测试的独立解析线段/体素盒答案以实际送入核的整数射线为准，不宣称未量化连续线的精度。

## 复跑已验证的范围

已验证源码标签 `m7-core-20260921-native-v2`，提交 `17d0405`。该版的主机测试及 HLS 原始产物在 `runs/m7-start-20260921/native-v2/`：`pytest.log`、`vitis_hls.log`、`hls_k26_10ns/solution/syn/report/fpga_m7_ray_flags_csynth.rpt`、`hls_k26_10ns/solution/sim/report/fpga_m7_ray_flags_cosim.rpt`；源码与产物校验文件为 `source.sha256`、`MANIFEST.sha256`。原几何单测与新增解析盒交对照共 18 项通过；Vitis HLS 2022.2 针对 `xck26-sfvc784-2LV-c`、10 ns 目标，C 仿真无错误、出综合报告、Verilog C/RTL 协同仿真 PASS。C++ testbench 覆盖最高索引、4096 数量上界和空帧重用；其有限事务不能代表整帧典型负载。

在本人 clone 的固定源码版本中，从**新的**产物目录复跑（占位的运行名请换成尚不存在的名字）：

```bash
cd ${FPGA_SRC}/runs/m7-core-20260921-worktree
RUN="$PWD/runs/m7-start-20260921/native-recheck-unique-name"
mkdir -p "$RUN/tmp"
env TMPDIR="$RUN/tmp" PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 -m pytest -p no:cacheprovider -q \
  tests/test_m7_native.py tests/test_geometry_cpp.py > "$RUN/pytest.log" 2>&1
(
  cd "$RUN" || exit 1
  source /data1/Xilinx/Vitis_HLS/2022.2/settings64.sh
  FPGA_M7_PART=xck26-sfvc784-2LV-c \
  FPGA_M7_BUILD="$RUN/hls_k26_10ns" FPGA_M7_CLOCK_NS=10 \
    vitis_hls -f ${FPGA_SRC}/runs/m7-core-20260921-worktree/hw/m7/run_hls.tcl \
    > "$RUN/vitis_hls.log" 2>&1
)
```

这里显式使用 `/usr/bin/python3`：默认 `python3` 可能指向未安装 pytest 的 Conda 解释器。HLS 脚本要求构建目录事先不存在，且必须在源码目录外；失败时保留原始日志，不能把工具环境错误当成算法失败。

当前综合报告的 `origins`、`ends`、`ray_valid` 和 `flags` 都是外部 **`ap_memory` 端口**，综合统计 BRAM/URAM 均为 0。综合中的资源占用和估计时序仅属此核、此器件与此目标时钟；没有实现片上驻留网格、DDR/DMA 连线、布局布线、实板运行或导航/记忆/显示的完整 PL 业务链。协同仿真的事务周期数不代表实机吞吐、端到端时延或相对 Orin Nano 的收益。

## 整帧、物体与显示入口（2026-09-21续轮）

新 `frame_depth` 顶层把深度投影/Q8/逐帧网格累积放到一次调用内；`object_evidence` 做原帧掩码投影与ROI计数；
`display_overlay` 实际生成伪彩和状态文字RGB像素。三个新顶层分别通过CPU/Csim/综合/RTL，完整系统互连仍未做。
[完整ABI与责任边界](../../docs/design/M7_FRAME_CHAIN_INTERFACE_20260921.md)，
[冻结版本、实验报告和剩余工作](../../docs/design/M7_FRAME_CHAIN_RESULTS_20260921.md)。
对应HLS入口为 `run_frame_hls.tcl`、`run_object_hls.tcl`、`run_display_hls.tcl`（无掩码多框区域核对另有 `run_object_roi_hls.tcl` + `object_roi_testbench.cpp`，见 `docs/design/OBJECT_ROI_SPEC_20261011.md`），需按脚本显式指定器件、周期和全新构建目录；
启动vitis_hls前先cd到独立run目录，避免工具自动日志落源码根。最终组合代码标签m7-chain-20260921-v2。

## 独立显示保护与成本诊断（2026-09-21第三轮）

`display_watchdog.sv` 在持续扫描时钟下按采集年龄遮蔽旧像素，支持完整帧pending、SOF换bank和一拍RAM旧读遮蔽。
它不生成视频时序，也不实现buffer/DMA/CDC；接口前提与复跑命令见[显示保护接口](../../docs/design/M7_DISPLAY_WATCHDOG_INTERFACE_20260921.md)。
最终运行器标签m7-display-watchdog-20260921-v1c，经RTL正/故障对照和OOC综合，未做布局布线或实板。
`run_frame_dense_hls.tcl`的13案CPU黄金通过但RTL提前终止；`run_object_port_probe.tcl`两臂RTL通过，三bank更慢且不交付。
[本轮结果与原件](../../docs/design/M7_DENSE_WATCHDOG_RESULTS_20260921.md)记录运行器失败、资源/周期范围和下一步；原生产计算核保持原样。

## 跨帧滚动 log-odds 地图（2026-09-26）

`rolling_map.cpp` 是第一个**片上驻留**的 M7 地图核：80×80×32 格（5 cm 时 4.0×4.0×1.6 m，验证配置）的 `int8` log-odds，
8 格一字绑到 URAM；窗口随车平移只改环形偏移、清出界切片，不搬数据。输入就是逐帧核的三态输出块，输出紧凑增量、三态查询和整窗快照。
`rolling_map_ref.cpp` 是写法不同的朴素参考（每格一字节、平移真搬数据），`rolling_testbench.cpp` 用现有 `ray_flags` 生成观测，
逐调用对照两者并核手算已知答案。`run_rolling_hls.tcl` 跑 csim/csynth/cosim（`FPGA_M7_ROLL_FLOW=csim` 只跑 C 仿真，
`FPGA_M7_ROLL_MUTANT=<名>` 编入故障变体，测试台必须失败；csim 失败时 vitis_hls 返回 1，仍以日志中的 FAIL/PASS 行为准；
Vitis 会把设计文件的 -cflags 同时加到测试台与参考上，所以 HLS 流程里的变体是「核与参考同带故障」，靠手算已知答案判出）。
主机编译 HLS 源码时需加 `-Wno-unknown-pragmas -Wno-unused-label`（见 `host/m7_rolling.py`）。
接口见 [滚动融合接口](../../docs/design/M7_ROLLING_FUSION_INTERFACE_20260926.md)，数字与对照见 [结果](../../docs/design/M7_ROLLING_FUSION_RESULTS_20260926.md)。
