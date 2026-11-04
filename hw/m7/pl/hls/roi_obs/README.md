# 三维观测核 HLS 工程

三维观测核 HLS 工程：四个相位复用直方图、连通域、颜色与精细分位工作区；在两组工程关键数据上，records 1,024 字节＋status 104 字节与 C 参考逐位一致；在 236 个真实框上，是否出观测、颜色类别与参考相同，最大位置差 0.725 mm。

## 验证结果

- C 参考（冻结版本 `75ad840e20e8e26ee98f5da5cbf1622ea18b7a33`）对照，固定 236 框：是否出观测、颜色、截断、status、有效数、最大域面积全部相同，最大位置差 0.725495 mm。
- 工程入口 `fpga_m7_roi_obs_phase`：slot 0 / 33 两组共 8 框、26 次相位调用，records 1,024 字节＋status 104 字节逐位一致。
- HLS 综合估计：17,168 LUT / 9,006 FF / 28 DSP / 17 BRAM18 / 9 URAM，估计时钟 7.300 ns；连通域与直方图/投影内层 II=1。
- 结构要点：窄位宽读图单元、连通域请求响应、带保护的直方图转发、框级单个 4-DSP 乘法器、分位排序一次求商余。

## 源码与接口

`obs_compute.cpp` 四个相位复用深度直方图 / 连通域 / 粗分位＋HSV / 精细分位工作区；`obs_ccl.cpp` 是完整 32,768 标签 / 65,536 像素的快速连通域工作区（打包父表读改写）。`obs_math.hpp` 是精确分箱位图、整数 HSV 与有效率中点。`roi_obs_ps.cpp` 生成参数 v3（914 字），PS 只负责几何与配置编译。

`roi_obs_top.cpp` 是工程入口：调用方对同一个框依次调用相位 0..3，状态与工作 RAM 留在核内；32 位 AXI 数据、64 位地址、914 字独立参数 RAM。`rtl/` 下是 CSR 外壳与封装的设计稿。

## 复现

工具为 Vitis HLS 2022.2，器件 xck26-sfvc784-2LV-c，时钟 10 ns，工程脚本 `run_hls.tcl`，测试台 `tb_roi_obs.cpp`；通过 `--argv` 指定 C 参考的向量文件 `vectors/obs_vectors.bin` 与期望记录 `representative.bin`。csim 后对 `csim/build/tb_out` 与期望相位输出逐字节核对。`snapshot-v0.1.json` 固定了核心源码的 SHA。
