# skill/：我们在大模型协作开发 FPGA 系统时沉淀的技能包

本目录收集了我们在用大模型智能体（主要是 Claude Code：一个主会话负责分析和验收，多个子代理负责执行）开发一台 KV260 + RGB-D 相机的寻物小车时，
从**真实失败**里总结、并且**用对照或已知答案验证过**的做法。每一项都可以单独拿走用。

## 适用对象

- 用 Vitis HLS / Vivado 2022.2 做 Zynq UltraScale+（KV260、ZCU 系列等）设计的队伍；
- 想让大模型智能体参与 HLS 优化、整板构建、上板测量，又担心它"报错数字、改乱代码、上下文撑爆"的队伍。
- 不要求用 Claude Code：每一项的 `README.md` 都是普通文档，模板和脚本是普通 Markdown / Bash / Tcl / Python，其他智能体或人都能直接用。

## 技能项清单

| 名称 | 类型 | 适用场景 | 已验证效果 | 出处 |
|---|---|---|---|---|
| [`hls-microcode-area/`](hls-microcode-area/README.md) | 提示词工作流 + 架构模板 + 写码规矩 | HLS 核面积远超预算，C 代码是长串行控制逻辑 | 记忆核 OOC 9,180 → **1,487 LUT**（−84%），14,886 次录制调用逐字相同；反例：搜索核照搬后 csynth 估计 10,485 LUT，未见改善 | 我们的工作包（记忆核、搜索核微程序化） |
| [`axi-bram-shell/`](axi-bram-shell/README.md) | 案例模板 + 测量用桩核 | 小核与 PS 交换固定大小的命令 / 结果块 | 桩核 OOC：m_axi 默认外壳 1,505 LUT（适配器 **1,157**）、瘦身 926、BRAM 端口 **30** | 我们的工作包（记忆核外壳、整板合并） |
| [`cdc-guard/`](cdc-guard/README.md) | 校验脚本 + 阳性对照 | IP 自带时钟对级 false path（TIMING-54）的整板 | 原设计 209 + 27 条路径全在 IP 内 → PASS；ECO 加一个越界触发器 → **FAIL outside=1** | 我们的工作包（CDC 定性、整板合并） |
| [`remote-build-offload/`](remote-build-offload/README.md) | 工作流 + 构建对比脚本 + 远端启动模板 | 本机忙，整板构建慢，时序在边缘 | 远端 **29.5 分钟** vs 本机 111 分钟；默认策略差 6 ps 未过、加强时序策略 +22 ps 过；地址表逐字节相同 | 我们的工作包（构建分流校准） |
| [`board-e2e-measure/`](board-e2e-measure/README.md) | 踩坑清单 + 预注册判读模板 + 汇总脚本（带自测） | 板上端到端时延 / 帧率 / 功耗测量与上板安全 | 双推理线程单帧 147.7 ms、单线程 **99.4 ms**（双缓冲使时延近翻倍）；一次整机卡死用只读日志定位到"板上链接期间" | 我们的工作包（端到端测量、卡死定位） |
| [`subagent-brief/`](subagent-brief/README.md) | 提示词工作流 + 任务书 / 交接单 / 验收清单模板 | 主控 + 子代理的多小时工程 | 长代理 94 万 token 半截终止、五次仪器错误复发后总结；采用后国赛约 110 个工作包按任务书过门或按交接线轮换 | 我们的工作方式 |
| [`fpga-hls/`](fpga-hls/README.md) | 提示词工作流 + 校验脚本（**基于上游 AMD Ross 改进**） | Vitis HLS 2022.2 写核、优化、跑流程、读报告 | 脚本层 5 个工程 319 项对表 0 处不一致；在用核一次优化 cosim 最大延迟 **−47.1%**，15 个逐次补丁可回放 | 上游 Xilinx/ross-ai-assistant（MIT）+ 我们的 2022.2 移植与校准 |
| [`vivado-checks/`](vivado-checks/README.md) | 校验脚本（**基于上游 AMD Ross 改进**） | Vivado 2022.2 批处理 lint / elaboration / 时序与方法学 | 两个整板 dcp 的 WNS/TNS 等与构建时报告逐项一致；lint 解析器的两个 2022.2 bug 被对账抓到并修复 | 上游 Xilinx/ross-ai-assistant（MIT）+ 我们的批处理移植与校准 |

每个子目录的 `README.md` 都按同样五栏写：**适用场景、用法、失效条件、已验证的效果、从哪些失败里总结出来**。

## 技能包的定位

上游移植的两项技能（`fpga-hls`、`vivado-checks`）负责**让智能体报出的数字可信、改动可追溯**：校准过的报告解析、逐字节比对、逐次补丁与账本。
大幅面积收益来自我们自己总结的架构做法（`hls-microcode-area/`、`axi-bram-shell/`），效果都是同口径的 OOC 实测。
实践中发现的缺口（m_axi 加宽需要对齐）已补进 `fpga-hls/reference/maxi-align-2022.md`。

## 怎么装

**Claude Code**：每个子目录都带 `SKILL.md`（frontmatter 里有 `name` 和 `description`），整个目录复制过去即可：
```bash
mkdir -p ~/.claude/skills            # 或者项目内 .claude/skills/
cp -r skill/* ~/.claude/skills/      # 会多拷一个本 README，无害
```
对话里描述任务（例如"这个 HLS 核 LUT 超了，用 hls-microcode-area 的流程做"），或直接点名技能名即可触发。

**其他智能体**：把对应子目录的 `README.md`（和 `SKILL.md`）作为系统提示词或任务书附件给它；脚本按 README 的命令调用。

**环境前提**：Vitis HLS / Vivado 2022.2（脚本用环境变量指定安装位置：`HLS_SETTINGS`、`VV_SETTINGS`、`VIVADO_SETTINGS`；默认值是 `/tools/Xilinx/...`），Python 3，Bash。
`vivado-checks` 的 560 条报错知识库需要联网取回：`bash vivado-checks/reference/fetch_upstream_reference.sh`。

## 命名、路径与来源说明

- 文件和目录名全部是小写英文、数字、下划线、连字符；例外只有 `README.md` 和 `SKILL.md` 两个约定名（后者是 Claude Code 识别技能的固定文件名）。
- 文中的 `<repo>/`、`<run>/`、`<runs>/`、`<worktree>`、`<artifacts>`、`<upstream_copy>/` 是占位符，指我们自己机器上的仓库、运行目录、构建产物目录和上游副本，换成你的路径即可。
- 上游来源：AMD Xilinx/ross-ai-assistant，https://github.com/Xilinx/ross-ai-assistant ，MIT 许可，2026-11-02 下载（归档 sha256 `a20046c7253b38dce9b4df8a3e9f93eedf1e6c62dc1686feff492692faab9d23`）。
  `fpga-hls/upstream.md`、`vivado-checks/upstream.md` 里有许可全文、逐文件吸收映射和改动说明；上游 630 个参考文件未原样随包分发。
- 标注"为提交包改写、未在 Vivado 上重跑"的脚本（`cdc-guard/cdc_guard.tcl`、`cdc-guard/poscontrol_eco.tcl`、`remote-build-offload/remote_run_template.sh`）都在各自 README 的"失效条件"里写明了，第一次使用请先跑对应的阳性对照。
