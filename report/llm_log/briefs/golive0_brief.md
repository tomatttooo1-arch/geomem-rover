# GOLIVE0：m5 接进产品启动链，台架整机跑通（今晚提交用，2026-11-04）

你是 FPGA 国赛线执行代理，交回后主控验收。输出节流（tail/grep，长内容落文件）。上下文约 200k 停手写 HANDOVER。

## 为什么
今晚交"m5 比特流＋现在能跑通的软件链路"。必须证明：一条命令起整机（KV260 加载 m5 跑核心各核，相机，识别；Nano 只承担现有的上层任务逻辑/控制台，弱化），台架上连续跑通。**车绝对不动**（不发任何行驶指令，底盘断开或只读）。

## 第一阶段：零板端（现在就做）
1. 找到现役产品启动链（10-26 上线时的 runbook / kv_start / board_bringup.sh / Nano 侧启动脚本 / tools/nn_cam2/kv2_chain.sh 等；RUN 下 SMOKE5/stage、SMOKE6/stage_s6 有 m5 版的拼装），列出它在 m5 下要改的点（加载 m7_pl_m5_166、BUILD_ID 0x0f35f07f 核对、各核驱动换 m5 版、识别用 RECOG_MODEL=y8s640、后处理：RECOG641 若交付则 --post pl）。
2. 在 REL0 的发布 worktree（<worktree>/nat-submit-20261104，REL0 会建；没建好前先在你自己的 worktree <worktree>/nat-golive0-20261104 从 1ff2bf9 开，做完让 REL0 合）写一条命令的整机启动脚本 `start_all.sh`＋停止脚本，**全部用宿主预编译或已在板上的二进制，板上不原生编译**（上午卡死发生在板上链接时）。
3. 台架判据写在测量前（RUN/GOLIVE0/CRITERIA.md）：连续 10 分钟；两条链帧都在走（深度路势场更新、识别路出框）；Nano 侧任务逻辑收到框与地图；无崩溃无重启；记录各段帧率。

## 第二阶段：上板（等主控放行）

## 交付
RUN/GOLIVE0/README.md：启动方式（一条命令）、判据与结果、各段帧率、日志路径、Nano 侧承担了什么（一句话，便于文档弱化表述）。

## 规矩
