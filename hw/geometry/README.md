# 几何计算的浮点参考实现（定点几何核的对照基准）

这一层先验证两个已有 Python 运算的 C++ 表达：像素深度反投影到世界坐标，以及历史物体投影区域的
有效、前方遮挡、后方空闲深度计数。公式与严格比较来自 `perception/fpga_replay/core.py`。
这是本队新增源码，作为 PL 定点几何核（`hw/m7/pl/hls/m7_frame_*`）的浮点对照基准，公式与 `perception/fpga_replay/core.py` 一致。

`geometry.hpp` 定义接口。每次最多处理 4096 个样本，较大输入由主机分块；该上限是接口容量，
不是画面大小或实测吞吐。数组、标定矩阵均按行展开。深度是米制光轴 Z，不是射线距离。
主机提供完整逆内参，因此支持非零 skew；世界变换为 `[R|t]`。无效点清零并返回独立有效位，
消费者必须看有效位，不能把世界原点当作无效。超长请求返回错误并且不写点输出，不能消费旧输出。
计数接口发生错误时清零全部计数。空批次合法。指针容量、不别名、标定和可信位姿由主机验证。

计数输入是 Python 已经投影、裁剪并检查过的区域，front/back 已包含深度裕量。
有效率、对象状态、遮挡优先顺序、独立视角累计与时间戳逻辑仍归 Python；不能凭这些计数绕过它们。
分块时先对计数求和再计算比例，不平均各块比例。帧号及对象编号应随主机请求保存，本层不产生身份。

## 目前可运行的主机验证

在仓库根、装有回放依赖和 g++ 的环境运行：

```bash
python -m unittest discover -s tests -p test_geometry_cpp.py -v
```

测试临时编译共享库与独立 testbench，不把构建产物放进源码目录。包含恒等变换、旋转和平移的已知答案、
带 skew 的整块 Python 对照、无效深度与严格阈值边界、相反几何证据、与现有 `compute_geometry` 的
状态判读比较、空输入与超容量错误。原有 Python 测试不变；新增覆盖主要抓坐标矩阵布局、浮点精度转换、
有效位和原生接口边界错误。未撤掉或缩小任何已有检查。

当前只用 IEEE float32，无定点化；g++ 禁止浮点收缩，不使用 fast-math。Python 几何中还存在 float64
矩阵计算，测试的坐标容差只适用于覆盖的室内输入范围。接近阈值的 float64 到 float32 转换、其他标定范围、
HLS 运算重排，以及最终记忆轨迹仍需后续同输入验证，不能宣称逐位相同或完整替换已通过。

## 有工具链后执行

Tcl 流程参考 AMD/Xilinx 的 [HLS 入门示例](https://github.com/Xilinx/Vitis-HLS-Introductory-Examples)
和 [2022.2 用户指南](https://www.xilinx.com/support/documents/sw_manuals/xilinx2022_2/ug1399-vitis-hls.pdf)。
面向 Vitis HLS 2022.2。运行时填写实际 part、时钟周期和全新的构建目录：

```bash
export FPGA_HLS_PART='<实际器件完整 part>'
export FPGA_HLS_CLOCK_NS='<本次要求的正数周期，单位 ns>'
export FPGA_HLS_BUILD='/tmp/fpga-geometry-backproject-本次运行标识'
export FPGA_HLS_TOP='fpga_backproject'
vitis_hls -f hw/geometry/run_hls.tcl
```

脚本依次运行 C 仿真、综合、Verilog 协同仿真，拒绝复用已有目录。计数核需要将 top 改成
`fpga_visibility_counts`，并使用另一个新目录单独运行。testbench 可先用 g++ 跑，但这不等于 HLS C 仿真，
更不等于 RTL 协同仿真。当前采用基础数组接口，未定义板上 AXI/DMA 连接、缓存同步或超时恢复。

后续按实际报告决定存储端口、流水线和定点方案；没有预先承诺每拍一个样本。工具版本、器件、时钟、命令及
报告随运行保存；综合估计不能写成板端性能。真实板到位后还要验证传输、复位、阻塞以及任务级影响。
