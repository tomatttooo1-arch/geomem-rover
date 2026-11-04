---
name: axi-bram-shell
description: 小 HLS 核的数据口选型：先用只有搬运的空壳桩核 OOC 量 m_axi / 瘦身 m_axi / BRAM 端口三种外壳的 LUT，命令结果块固定且不大时用 BRAM 端口加 AXI BRAM 控制器代替 m_axi。
---

# axi-bram-shell

完整说明（适用场景、用法、失效条件、已验证的效果、来自哪些失败）见同目录 `README.md`，先读它再动手。
本目录下的模板和脚本：`examples/`（三种外壳桩核、测试台与 HLS Tcl）。
