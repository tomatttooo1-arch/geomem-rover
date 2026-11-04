# 上游来源与改动记录

## 来源

| 项 | 值 |
|---|---|
| 仓库 | https://github.com/Xilinx/ross-ai-assistant （AMD "Ross" agent skill 包，`plugin.json` version 2026.10.1） |
| 下载日期 | 2026-11-02 |
| 本地快照 | `<upstream_copy>/`（`ross.tgz` 及解压目录，只读） |
| 归档 sha256 | `a20046c7253b38dce9b4df8a3e9f93eedf1e6c62dc1686feff492692faab9d23  ross.tgz`（同目录 `SHA256SUMS`） |
| 许可 | MIT（全文见下，副本 `license_upstream.txt`） |
| 作者 / 版权 | Advanced Micro Devices, Inc.（`plugin.json` author；各文件头 `Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved. SPDX-License-Identifier: MIT`）。各 skill 未署个人作者，skill-card 只记测试客户端/模型 |

各 skill 上游版本（`manifest.yaml`）与 skill-card 测试记录：

| 上游 skill | 版本 | skill-card 测试 | 本移植 |
|---|---|---|---|
| vivado-rtl-lint | 1.0.0（SKILL 内版本史到 2.1.0） | Claude Code / Opus 5.5 / 2026-10-21 | **拿** |
| vivado-timing-methodology-checks | 2.0.0（标注 Requires Vivado 2026.1+） | 2026-10-21 | **拿** |
| vivado-rtl-elaboration-analysis | 1.0.0 | 2026-10-21 | **拿** |
| vivado-simulate-rtl | 1.0.1（需 MCP，验证于 2025.2/2026.1） | 2026-10-18 | 不拿 |
| vivado-ip-configurator | （需 2026.1 + Vivado MCP + AMD 文档检索 MCP） | 2026-10-17 | 不拿 |
| vivado-revision-control | 1.0.1 | 2026-10-18 | 不拿 |

## 拿了什么

- `reference/vivado-rtl-lint/`：上游 `skills/vivado-rtl-lint/` 全部 29 个文件（SKILL.md、tcl-reference、violation-handlers、report-format、resolution-guide、20 个 `resolution/*.md`、examples、`parse_lint_report.py`、skill-card）。
- `reference/vivado-timing-methodology-checks/`：上游全部 22 个文件（SKILL.md、16 个 references、3 个 scripts、examples 模板、skill-card）。
- `reference/vivado-rtl-elaboration-analysis/`：上游全部 569 个文件（含 560 个 `resolution/` 知识库条目：553 个 `8-xxx.md` + VLOG-689 + 5 个模式文件 + README；两个 Python 解析器）。
- `reference/examples/`：上游 `examples/rtl-lint/multi-violation` 与 `examples/timing-methodology-checks/multi-violation`（用作校准的已知答案）。
- `license_upstream.txt`：上游 LICENSE 原文。
- 未复制但已阅读：`docs/getting-started/vivado-mcp.md`（MCP 安装，我们不装）、`docs/faq.md`（声称支持 2020.2+，2026.1 为测试基线）。

## 改了什么（每个改动文件内都有一行 `Modified 2026-11-02` 说明）

| 文件 | 改动 |
|---|---|
| 三个 `SKILL.md` → `upstream_skill.md` | 改名；frontmatter `name` 加 `upstream-` 前缀（防止被当成独立 skill 加载）；文首加一行修改说明，声明 MCP 专属指令由顶层 `SKILL.md` 取代。正文未改 |
| `reference/vivado-rtl-lint/parse_lint_report.py` | ① 兼容 2022.2 “2. Expanded” 的 6 列表（无 `info` 列，输出 `N/A`）；② 修正空表时把表尾说明文字切成假行的问题（2022.2 空表头后无数据行，原逻辑把表尾 `* create_waiver...` 当数据）。7 列路径不变 |
| 新增 `SKILL.md`（顶层） | 合并入口，三种模式 + 2022.2 差异 + 我们工程（HLS IP、BD、KV260）的注意点 |
| 新增 `scripts/vv_run.sh`、`vv_common.tcl`、`vv_lint.tcl`、`vv_elab.tcl`、`vv_timing.tcl`、`vv_post.py` | 批处理包装。`vv_lint/elab/timing.tcl` 的 Tcl 主体移植自上游 tcl-reference / SKILL 步骤（文件头注明出处与改动）：默认器件 xc7k70t → xck26-sfvc784-2LV-c；非工程模式 include_dirs 改为 `synth_design -include_dirs`；`report_methodology -json`（2022.2 无）改为 `-file` + 上游 `write_json_report.tcl` |

## 没拿什么及理由

- **vivado-simulate-rtl**：价值中等但不适合现在。我们的 HLS IP 用 Vitis HLS 的 csim/cosim 验证，手写 RTL 很少；该 skill 核心是经 MCP 驱动多仿真器（Questa/VCS/Xcelium…）、审批式交互修复、UVM/覆盖率闭环，验证基线 2025.2/2026.1；移植成批处理等于重写，且会让 `vivado-checks` 越界。若以后需要 XSim 批处理，可单独参考其 `references/xsim*.md`。
- **vivado-ip-configurator**：硬依赖 Vivado 2026.1 + Vivado MCP + AMD 文档检索 MCP，学习缓存 `learned_params.json` 来自 2026.1 IP 目录，参数名/版本与 2022.2 的 IP 不保证一致；我们的 IP 配置已在 BD Tcl 脚本里固化，收益小、错配风险大。
- **vivado-revision-control**：面向 GUI 工程（.xpr）转可复现脚本、DFX/Segmented Config 等；我们的构建已是非工程模式脚本化（`hw/m7/pl/bd/build_bitstream.tcl`）并在 git 中，基本无增量价值。

## MIT 许可全文（上游 LICENSE 原文）

```
MIT License

Copyright (c) 2026 Advanced Micro Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
