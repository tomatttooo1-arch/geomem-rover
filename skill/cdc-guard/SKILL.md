---
name: cdc-guard
description: Vivado 整板构建后，IP 自带时钟对级 false path（TIMING-54）时，检查该时钟对之间的所有路径都在 IP 层级内；附阳性对照 ECO 脚本。
---

# cdc-guard

完整说明（适用场景、用法、失效条件、已验证的效果、来自哪些失败）见同目录 `README.md`，先读它再动手。
本目录下的模板和脚本：`cdc_guard.tcl`、`poscontrol_eco.tcl`、`validated/`（原版脚本与两次运行结果）。
