# fpga-hls：Vitis HLS 2022.2 的设计 / 优化 / 跑流程 / 读报告技能（基于上游 AMD Ross 改进）

类型：提示词工作流 + 校验脚本。智能体入口是同目录的 `SKILL.md`（Claude Code skill 格式，带 frontmatter）。

**来源声明**：基于上游 AMD Xilinx/ross-ai-assistant（MIT，https://github.com/Xilinx/ross-ai-assistant ，`main` 分支 2026-11-02 下载，
归档 sha256 `a20046c7253b38dce9b4df8a3e9f93eedf1e6c62dc1686feff492692faab9d23`）的 20 个 `hls-*` skill 合并移植。上游面向 Vitis 2025.x（Unified IDE、`vitis-run`），
我们把它改到 Vitis HLS 2022.2 的老式 Tcl 工程，并补了脚本层和校准。来源、许可全文、逐文件吸收映射见 `upstream.md`。

## 目录

| 路径 | 是什么 | 是我们改的还是新加的 |
|---|---|---|
| `SKILL.md` | 总入口：架构 → 写码 → 跑流程 → 读报告 → 诊断的工作流 | 由上游 20 个 skill 合并改写 |
| `reference/*.md`、`reference/architect/`、`reference/optimize/` | 各主题参考（dataflow、pipeline、burst、II 诊断、pragma 作用域等） | 上游文字按 2022.2 逐条核实后改写（20 个文件有改动） |
| `reference/maxi-align-2022.md` | m_axi 加宽需要对齐的经验 | 新加，来自上面那次优化 |
| `scripts/hls_run.sh` | 一条命令跑 csim / csynth / cosim / export（带超时，工程目录放仓库外） | 新加 |
| `scripts/hls_report.sh`、`scripts/pylib/` | 把 csynth / cosim / impl 报告解析成 JSON 摘要 | `pylib` 来自上游解析器，改成读 2022.2 目录布局 |
| `scripts/bitexact_check.py`（说明 `scripts/bitexact.md`） | 两个输出目录逐字节比对，报第一处不同的偏移 | 新加 |
| `scripts/qor_log.py` | 每次尝试记一行 QoR 账本 | 新加 |
| `scripts/calibration_20261102.md`、`scripts/calibration/` | 脚本在已知答案上的校准记录、玩具核、逐项结果 | 新加 |
| `session/hlsopt_session.sh`、`session/selftest.sh` | 优化会话：每次尝试一个补丁 + 账本，防止覆盖用户源码 | 新加 |
| `porting_notes.md`、`scripts/porting_notes.md` | 上游写法在 2022.2 下哪些能用、哪些不行，逐条证据 | 新加 |

## 适用场景

- 用 Vitis HLS 2022.2 给 Zynq UltraScale+（我们是 KV260，器件 `xck26-sfvc784-2LV-c`）写或优化一个核；
- 要按吞吐、延迟或资源目标迭代优化（II、pipeline、partition、burst、dataflow）；
- 要让智能体稳定地跑 csim / csynth / cosim 并读出数字，而不是让它自己去 grep 报告。

## 用法

1. 装进 Claude Code：把本目录复制成 `~/.claude/skills/fpga-hls/`（或项目内 `.claude/skills/fpga-hls/`），对话里说"用 fpga-hls 优化这个核"即可触发。
   其他智能体：把 `SKILL.md` 正文当系统提示词或任务书附件，脚本照常调用。
2. 设环境：先 `source <你的安装>/Vitis_HLS/2022.2/settings64.sh`（脚本看到 `XILINX_HLS` 已设就不再 source），或者 `export HLS_SETTINGS=<你的 settings64.sh>`。
3. 典型一轮（工程目录必须在仓库外、事先不存在）：
   ```bash
   H=skill/fpga-hls/scripts
   bash $H/hls_run.sh --tcl run_hls.tcl --timeout 900 <proj> solution csim csynth cosim
   python3 $H/bitexact_check.py <golden_dir> <proj>/solution/csim/build/tb_out      # 必须 RESULT BITEXACT
   python3 $H/bitexact_check.py <golden_dir> <proj>/solution/sim/wrapc_pc/tb_out    # cosim 的 RTL 输出
   bash $H/hls_report.sh synth <proj>/solution     # 资源 / 估计时钟 / 每个循环的 II
   bash $H/hls_report.sh cosim <proj>/solution     # 实测延迟
   ```
4. 用之前先在你自己的一个已知工程上跑一次 `hls_report.sh`，与文本报告对一遍（做法见 `scripts/calibration_20261102.md`）。

## 失效条件

- 工具版本不是 2022.2：报告目录布局、XML 字段、pragma 行为都可能不同，`porting_notes.md` 里的结论不能直接沿用，需要重做校准。
- 上游 Vitis 2025.x 的 `hls_config.cfg` / `vitis-run` 流程在这里不适用。
- 只有 csim 的工程：`hls_report.sh` 会按设计报"缺综合报告"并以退出码 3 退出，不给数字。
- 可变循环次数的延迟在报告里是"未定义"，摘要输出 `"undef"` 而不是数字，下游脚本要处理。
- **它不替你做硬件架构**。我们的经验是：直接把软件代码交给 HLS、再用这个技能调 pragma，面积往往仍然远超预算；先做架构见 `../hls-microcode-area/`。

## 已验证的效果

**1. 脚本层校准（已知答案，`scripts/calibration_20261102.md`）**：对 5 个工程（3 个真实核、1 个玩具核、1 个上游示例）把 JSON 摘要和文本报告逐项对表，
共 319 项（41 + 134 + 66 + 34 + 44），修正后 0 处不一致。阴性对照：人为把 DSP 实际值 +1、把某循环 II 3→4，仪器都报"不一致"并以退出码 1 退出。
校准过程中抓到两个真 bug 并修掉：解析器把可变循环上界 `-1` 当成数字；仪器第一版汇总表过滤写错，模块和循环行一行没读进来。

**2. 在用核上的一次完整优化（2026-11-02）**：对一个在用的 HLS 核（物体证据区域统计核），智能体按本技能包的流程在 8 次尝试、约 37 分钟内，
把 cosim 最大延迟从 271,534 周期降到 143,560 周期（**−47.1%**），资源 LUT / FF / BRAM_18K / DSP = 17,444 / 9,246 / 22 / 8。
csim、cosim 与 golden 逐字节一致，另加一套事先不公开的隐藏场景也全部通过；主会话从最终标签用同一套工具重跑复核，不采信自报数字。
主手段是"像素计数改成框面积以删掉 32 路计数器"和"主循环每拍 2 像素、端口 32→64 位"。整个过程留下 15 个逐次补丁和账本，每次尝试一处改动，可以精确回放。

## 从哪些失败里总结出来

- 上游解析器读 2022.2 报告时把可变循环上界 `-1` 原样当数输出（校准第 1 节）；仪器第一版漏读模块和循环行却"看起来能用"——所以我们坚持"新仪器先在已知答案上对表，并且要有一个必然报错的阴性对照"。
- 智能体直接改用户源码、多次尝试互相覆盖，事后说不清哪一处改动带来了收益——所以加了 `session/` 的逐次补丁与账本，且拒绝在未登记的新文件上工作（`porting_notes.md` 第 3 节自测）。
- 优化中有 4 次尝试卡在"让 HLS 认出对齐以加宽 m_axi"上——补成 `reference/maxi-align-2022.md`。
