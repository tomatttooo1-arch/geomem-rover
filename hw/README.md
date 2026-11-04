# hw/ —— 板上逻辑

放在 PL（可编程逻辑）里跑的东西：HLS 源码与 testbench、手写 RTL、平台与约束文件（.xdc 等）、
以及把它们变成比特流的构建脚本。

只提交源码、约束和脚本。**构建产物不进 git**（.xclbin / .bit / .xsa / .hwh / .dcp / _x/ 等已在 .gitignore 里），
发布件放 `${FPGA_SHARED}/artifacts/` 并附 MANIFEST。

构建脚本里不要写死某台机器的绝对路径；工具版本要写进脚本注释或 `docs/team/ENV.md`。
