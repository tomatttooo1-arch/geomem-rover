---
name: remote-build-offload
description: 整板 Vivado 构建推到远端空闲机器、两种实现策略并行、拉回后判时序，并用已知答案构建做一次校准对比。
---

# remote-build-offload

完整说明（适用场景、用法、失效条件、已验证的效果、来自哪些失败）见同目录 `README.md`，先读它再动手。
本目录下的模板和脚本：`compare_builds.sh`（构建对比）、`remote_run_template.sh`（远端脱钩启动器模板）。
