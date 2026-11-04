# RECOG641：后处理核接进产品识别链 dpu_rx＋矩形 640×480 档打通（2026-11-04）

你是 FPGA 国赛线执行代理，交回后主控验收。输出节流（tail/grep，长内容落文件）。上下文约 200k 停手写 HANDOVER。

## 背景（先读）
RUN=<run>
- 上一棒：RUN/RECOG640/README.md、BINDINGS.md、HANDOVER.md。代码 worktree <worktree>/nat-recog640-20261104（分支 feat/member-a/nat-recog640-20261104  at d1f4063）。
- 现状：产品识别链 dpu_rx 用 CPU 后处理（det_post.cpp）；PL 后处理核（DETPOST1，已在 m5 比特流里）没接进来；后处理核侧的 recog_config()/score_floor_to_q() 已写好。文档按"前处理核＋DPU＋后处理核"写，PL 核必须真用上（做完即上线）。
- member-c 正在出 640×480 矩形输入版模型（同 QAT 权重重量化编译），到货时间未知。

## 要做
1. **后处理核接进 dpu_rx**：新增 `--post pl|cpu`（默认 pl；pl 初始化失败或核报错时告警并回落 cpu，不拒起）。DPU 输出头 DMA 缓冲布局按 BINDINGS（20×20 at 0、40×40 at 28000、80×80 at 140000，共 588,000 B；VART 输出顺序 80/20/40 要按网格映射），按 DETPOST1 驱动接口配置（nc=6、每类阈值换成定点、IoU、max_det、exp 表）；结果框格式与 CPU 路径一致。宿主校验：后处理核 C 模型路径与 CPU 路径在 RECOG640 的 22 例上逐框一致（已知同分格例外照旧说明）＋一个阳性对照。驱动/接口若与 DETPOST1 README 的前提不符，列出来交主控。
2. **矩形档 y8s640r 打通到代码层**：Geometry 加 tw/th、帧环与 SoftPrep 支持矩形、前处理核配置支持矩形旁路（640×480 源直出 640×480 张量、无灰边）、VART 输入形状核对改为读模型；宿主用合成 640×480 张量跑通前处理（对 member-c 口径"只缩放不补灰边"逐字节同）与后处理（网格 80×60/40×30/20×15）。profiles.json 里模型路径与阈值保持占位 ready=false。
3. 上板脚本 RUN/RECOG640/board/run_recog640.sh 增加 POST=pl|cpu 参数，两种各测一次识别链帧率与后处理耗时（不上板，留给主控安排）。
4. 交 RUN/RECOG641/README.md：改动、校验、上板用法、未完成。
在原 worktree 上续做（新提交，post 标签 nat-recog641-post-20261104，只推显式 ref）。

## 规矩
