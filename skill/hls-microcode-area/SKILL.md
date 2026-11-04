---
name: hls-microcode-area
description: HLS 核超面积时使用：先按 arch_template.md 做硬件架构与逐部件手算，再写码；长串行控制逻辑改成一个共用运算单元加微程序；csynth 后按 coding_rules.md 逐项对手算。
---

# hls-microcode-area

完整说明（适用场景、用法、失效条件、已验证的效果、来自哪些失败）见同目录 `README.md`，先读它再动手。
本目录下的模板和脚本：`arch_template.md`（架构设计模板）、`coding_rules.md`（五条写码规矩）。
