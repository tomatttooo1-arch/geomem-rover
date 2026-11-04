# Third-party notices / 第三方组件与许可

本包自研代码以 MIT 许可发布（见 `LICENSE`）。下列第三方组件不属于本项目，各自保留其原许可；表中"位置"一列说明它们在本包中的形态。

| 组件 | 用途 | 许可 | 在本包中的位置 |
|---|---|---|---|
| AMD/Xilinx DPUCZDX8G v4.1（DPU IP） | PL 中的神经网络推理核 | AMD/Xilinx 的 IP 许可（随 Vitis AI 3.0 发布，以 AMD 官方条款为准） | IP 本身**不在本包中**；块设计 `hw/m7/pl/bd/m7_bd.tcl` 只引用它，构建时需自行从 Vitis AI 3.0 取得（见 README"重建比特流"）。提交比特流 `bitstream/` 中含其综合结果 |
| Vitis AI 3.0（量化、编译工具与运行库） | 模型量化编译（xmodel）、板上 DPU 运行时 | Apache-2.0 | 工具与运行库不在本包中；`models/dpu/` 下的 xmodel 由其编译生成 |
| Ultralytics YOLOv8 | 识别模型的网络结构与训练框架 | AGPL-3.0 | `models/dpu/` 下的识别模型（y8s640、y8s512_backup）基于 YOLOv8s 训练后量化编译；训练代码不在本包中 |
| AMD Ross AI Assistant（`github.com/Xilinx/ross-ai-assistant`） | 技能包 `skill/vivado-checks/`、`skill/fpga-hls/` 的上游 | MIT，Copyright Advanced Micro Devices, Inc. | 上游许可全文 `skill/vivado-checks/license_upstream.txt`；来源与改动记录 `skill/vivado-checks/upstream.md`、`skill/fpga-hls/upstream.md` |
| three.js（含 OrbitControls） | 操作端三维界面 | MIT，Copyright 2010-2025 three.js authors | `ui/geomem/app/vendor/`、`ui/geomem/operator/vendor/`，许可全文 `THREE-LICENSE.txt`（同目录） |
| Node.js | 操作端界面的本地运行时 | Node.js 许可（MIT 及其依赖的各自许可） | 运行时二进制不在本包中；许可全文 `ui/geomem/runtime/NODE-LICENSE.txt` |
| ROS 2 Navigation2（Humble）navfn | 规划参考模型 | `navfn.cpp`、`navfn.hpp` 为 BSD；包装参考 `navfn_planner_reference.cpp.txt` 为 Apache-2.0 | `tools/planner_model/vendor/`，来源说明 `upstream.txt`（未修改的上游文件） |

如本表与各组件自带的许可文件不一致，以组件自带的许可文件为准。
