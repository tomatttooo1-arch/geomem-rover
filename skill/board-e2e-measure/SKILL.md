---
name: board-e2e-measure
description: 嵌入式板卡上板安全（不在板上编译、读回构建号、失联预案）与端到端时延测量（单调时钟分段打点、预注册判读、汇总脚本先自测、多推理线程时延翻倍）。
---

# board-e2e-measure

完整说明（适用场景、用法、失效条件、已验证的效果、来自哪些失败）见同目录 `README.md`，先读它再动手。
本目录下的模板和脚本：`criteria_template.md`（预注册判读模板）、`seg_latency.py`（分段汇总，`--selftest` 自测）。
