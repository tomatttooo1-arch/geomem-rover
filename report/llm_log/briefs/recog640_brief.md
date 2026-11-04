# RECOG640：识别链换成 member-c 第四轮 640 模型（零板端准备，2026-11-04）

你是 FPGA 国赛线执行代理，交回后主控验收。输出节流（tail/grep，长内容落文件）。上下文约 200k 停手写 HANDOVER。

## 背景
owner 11-04 定：六类检测换成 member-c 第四轮 640 档 INT8 模型（车载主集 mAP50 84.77%，现役 512 档 75.85%）。
- 模型包：<shared>/models/20261103-member-c-recog-quant4/full/（先 `sha256sum --quiet -c MANIFEST.sha256`）。交接 member-c_RECOG_QUANT4_HANDOFF_20261103.md（"模型与阈值""实际编译接口"两节必读）。编译模型 models/640/compiled-selected/q4_y8s_640_b1024.xmodel（sha256 b04fd7c1…），阈值 models/640/threshold-selection.json（不得混用其他尺寸的阈值）。输入 [1,640,640,3] xint8 fix_point=6；输出 80/40/20 三尺度、70 通道（64 DFL＋6 类 logit），fix_point=2。前处理口径：保持比例缩放、居中 letterbox（填 114）、RGB、/255。
- 现役链路：RUN 下 PREP1（前处理核，已支持方 640 配置，见 README）、DETPOST1（后处理核，接口前提输入 ≤640、网格 ≤80/40/20、nc 1..40、缓冲 ≤1 MiB）、DPU 运行时（DPUB1/DPURT0 等）、产品启动链里加载模型与阈值的地方（grep 现役 512 模型文件名/sha 定位）。
- RUN=<run>

## 要做
1. 找出产品链里所有和 512 模型绑定的地方（模型路径、输入尺寸、前处理参数块、后处理网格/通道/fix_point、类别表、每类阈值、NMS 参数、DPU 缓冲大小），列表落 RECOG640/BINDINGS.md。
2. 新 worktree <worktree>/nat-recog640-20261104（从当前产品分支的 post 标签开，写明从哪开；pre/post 标签，只推显式 ref）：改成 640 模型，做成**一个开关**（如 RECOG_MODEL=y8s640|y8s512，默认 y8s640），旧路径保留。每类阈值读 640 的 threshold-selection.json；**药盒类阈值单独可配**（默认先取 json 里的值，另留覆盖项，后续重新标定后改）。
3. 宿主侧校验（只做关键的）：①前处理：用 PREP1 的 C 模型/驱动检查对 640×480 源生成方 640 张量的参数块合法、与 member-c 评测口径（letterbox 114、RGB、/255 再按 fix_point=6 量化）逐像素一致——取几张真实帧比对；②后处理：DETPOST1 的 C 模型按 640/nc=6/fix_point=2 解码，与 member-c 源码里的 CPU 解码（tools/nat_recog_quant4/ 下，NMS IoU 0.7、max_det 100、底阈 0.01）在同一组输入张量上结果一致——没有真实 DPU 输出就用随机张量＋若干人工构造的强响应格子，至少一个阳性对照（故意改错一个 fix_point 必须被抓到）。
4. 板上运行包准备好但不上板（KV260 目前卡死待重启）：写好上板一条命令跑"DPU 640 推理帧率（只 DPU）＋整条识别链帧率＋xdputil 查 DPU 子图 workload 与时钟"的脚本，带期望值（帧率预估约 10 帧/秒，按 DPUOPS0 的运算量外推为准）。
5. 交 RECOG640/README.md：绑定清单、改动、校验结果、上板脚本用法、文档里受影响的数字清单（帧率、时延、精度、类别阈值）。

## 规矩
