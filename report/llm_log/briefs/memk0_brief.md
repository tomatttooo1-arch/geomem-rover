# MEMK0 brief：语义记忆核（记忆关联与融合进 PL）——第一阶段：硬件设计

先读 `RUN/HWDESIGN_RULES_20261104.md`（共用规矩，必须守）。

## 要搬的东西
产品源 `perception/fpga_replay/core.py` 的 `class Memory`（第 313 行起：`update_memory`、`_associate_v2`、`_gate`、`_fuse`、`_promote`、`_merge_pass`、候选生命周期、`query_memory` 等）和 `perception/fpga_replay/memory_gates.py`。以 `work/member-a` 当前 main 上车用默认版本为准（找车上默认提交：CONTROL.md 里"车上默认主机代码"一条；读代码用 git show / git archive，不要改 main）。
输入是每帧的若干条观测（类别、得分、三维位置、范围、颜色、可信度、帧号时间、框等，见 `Observation`），以及几何证据；输出是轨迹表的更新。

## 期望结果（第一阶段）
`RUN/MEMK0/DESIGN.md`，按共用规矩第 1 条。特别回答：
- 轨迹表与每条轨迹的样本历史放在片上的存储结构（容量上限按产品实际用到的量定，写依据）；
- 一条新观测与全部轨迹的门限判定、打分怎么并行（目标：一次比完全部轨迹的固定拍数）；
- 融合里的中位数、MAD、逆方差加权怎么做成定点硬件（排序网络 / 直方图 / 增量维护，选哪个、为什么）；
- 合并扫描（两两轨迹比较）怎么做；
- 哪些分支裁决留 PS、PS 和核之间每帧交换什么、多少字节；
- 资源目标：**LUT ≤2,500**（BRAM/DSP 可多用）；
- 关键对照①的判据：建议用 `tools/m5_software_replay.py` 的 9 个场景（以及能找到的实车回放），定点模型与 Python 的轨迹事件流（新建/关联/合并/状态）完全一致、位置差 ≤5 mm——你可按实际情况调整，但要测前写死。
写完设计就停，交回主控审。交回时 10 行内摘要：切口、存储结构、并行方式、手算 LUT/BRAM/DSP、每帧拍数、PS 剩什么。
