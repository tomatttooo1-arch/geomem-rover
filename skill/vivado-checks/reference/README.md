# vivado-checks/reference：上游参考资料（大部分未随包分发）

`../SKILL.md` 里指向的 `reference/vivado-rtl-lint/`、`reference/vivado-rtl-elaboration-analysis/`、`reference/vivado-timing-methodology-checks/`、
`reference/examples/` 共 630 个文件，几乎全部是上游 AMD Xilinx/ross-ai-assistant（MIT，版本 plugin.json 2026.10.1，2026-11-02 下载）原样内容，
其中 560 个是 `8-xxx.md` 报错知识库条目。为了不把上游资料原样搬进提交包，这里只保留**我们改过的唯一一个文件**：

- `vivado-rtl-lint/parse_lint_report.py`：兼容 Vivado 2022.2 的 6 列 lint 表（无 info 列），并修正空表时把表尾文字当数据行的问题（改动见文件头 `Modified 2026-11-02` 注释）。

取回其余资料：`bash fetch_upstream_reference.sh`（联网 clone）或 `bash fetch_upstream_reference.sh <已解压的上游目录>`。
脚本会把上游 `SKILL.md` 改名为 `upstream_skill.md`（防止被当成独立 skill 加载），最后用我们的 `parse_lint_report.py` 覆盖上游版本。
不取回也能用 `../scripts/vv_run.sh` 跑 lint / elab / timing 并得到 CSV 和摘要；缺的只是"按报错号查修复建议"的知识库。
