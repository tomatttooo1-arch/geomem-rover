# REL0：今晚提交快照——发布分支＋发布包（2026-11-04）

你是 FPGA 国赛线执行代理，交回后主控验收。输出节流（tail/grep，长内容落文件）。上下文约 200k 停手写 HANDOVER。

## owner 定（11-04 12:0x）
今晚交一版"m5 比特流＋现在能跑通的软件链路"，冻结成快照，**要提交完整代码**；FPGA 为核心，Nano 可以在但要弱化；主线（搜索核、member-b 观测核、终版整板）后台继续，不进今晚快照的比特流。

## 要做
1. **盘点**（RUN=<run>；读 RUN/CONTROL.md 近两天的验收行）：列出今晚快照应含的所有分支/标签及其提交，至少：m5 产品代码 nat-merge8-post（1ff2bf9，m5 包 pkg-m5_166 所用）；m5 硬件源 f35f07f（MERGE7）；识别 640 开关 feat/member-a/nat-recog640-20261104（d1f4063，基于 1ff2bf9）；后处理核接入 RECOG641（在做，post 标签 nat-recog641-post-20261104 出来再合；没出就不合）；规划核终版 nat-gmap2-post（e267cfe）；记忆核重写 feat/member-a/nat-memk4-20261104（a87972e）；m6/终版一键脚本 MERGE9P（8da98df）与 MERGE10（90795a3）；搜索核在做的版本（SRCHK5 分支，取它最新已推送的提交）；DRV5 驱动；换数工具等。teammate（member-b、member-c）的分支只读引用，按其最新已推送标签/提交记录，不改。
2. **发布分支** `release/member-a/nat-submit-20261104`（新 worktree <worktree>/nat-submit-20261104，从 1ff2bf9 开）：按上面合并；**冲突一律以 m5 产品代码为准**，新核与新脚本只新增目录不改 m5 文件；合不进来的（冲突大的）以子目录快照方式放入并在说明里注明来源提交。不合 main，只推这个分支和标签 `nat-submit-rc1-20261104`（显式 ref，禁 --tags）。
3. **快速检查（只做这些）**：宿主单测（各包已有的 make check / pytest 里几分钟内能跑完的）；"新功能全关时块设计与 m5 逐字节同"那项 validate（MERGE10 工具，小构建槽按 RUN/SLOT_RULES_20261104.md，用完释放）；驱动自检。失败的列出来，不硬修大问题。
4. **发布包** RUN/REL0/pkg/（不进 git 的大文件放这里）：m5 比特流包（MERGE7/builds/pkg-m5_166 原样拷贝＋SHA256SUMS 复核）、识别模型（member-c 第四轮 640 编译模型＋阈值 json；现役 512 作备份；sha 核对）、板上一条命令启动脚本（等 GOLIVE0 交，先占位）、`SOURCE.txt`（发布分支提交号＋各子分支来源表）、`BUILD.md`（从源码重建比特流的命令，引用 MERGE7/MERGE10 一键脚本）、`MANIFEST.sha256`。
5. **代码—比特流对应表** RUN/REL0/MAP.md：每个 PL 核一行——源码路径、HLS export sha、是否在 m5 比特流里（m5 里有：几何、融合、规划双尺寸旧版、前处理、DPU、后处理、许可、ROI 等，以 MERGE7 的 CLOCKS.txt/hwh 为准）、单独验证记录路径（OOC/联仿/判据）。这张表给文档会话对齐用。
6. 交 RUN/REL0/README.md。

## 时间
目标 16:30 前出 rc1（分支＋包＋MAP.md）；之后只接 GOLIVE0 的启动脚本与 RECOG641 若已过验收。

## 规矩
