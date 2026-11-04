# 移植笔记：上游（MCP + Vivado 2026.1）→ 本仓（批处理 + Vivado 2022.2）

所有 “2022.2 是否支持” 的结论都在本机 `vivado -mode batch`（2022.2 Build 3671981）里用 `help <cmd>` 或实跑核实，
核实脚本与输出在私有草稿 `.scratch_vivado/probe/`（不提交）。

## 命令 / 选项逐条核实

| 上游用法 | 2022.2 | 处理 |
|---|---|---|
| `vivadoExecute` / `vivado_execute`（MCP 工具） | 不存在（不装 MCP） | 全部改为 `scripts/vv_run.sh` → `vivado -mode batch -source scripts/vv_*.tcl` |
| `vivado_doc_search`（AMD 文档检索 MCP） | 不可用 | 查本地/网页版 UG901/UG906/UG949；SKILL.md 中已注明 |
| `synth_design -lint -file <f>` | **可用**（`help synth_design` 有 `-lint` `-file`；实跑产出 `.rpt`） | 直接用 |
| lint 直接出 CSV（2026.1+，`linter.csv`） | 不支持，只有 ASCII `.rpt` | 用上游 `parse_lint_report.py` 转 CSV（已改，见下） |
| `set_param synth.elaboration.rodinMoreOptions "rt::set_parameter linterCsvFile true"` | 参数存在（`get_param` 返回空串），但未见 CSV 产出 | 不设置；不依赖 |
| lint “2. Expanded” 表 7 列（含 `info`） | **6 列**（无 `info`） | 解析器补 `N/A` |
| lint 规则集 | `help` 列 ASSIGN-1..7、QOR-1、INFER-1..3、CLOCK-1、RESET-1/2；**实测还会报 ASSIGN-9、ASSIGN-10** | 上游 ASSIGN-12/14、RESET-3 等新规则在 2022.2 不出现（ASSIGN-14 漏报已实测） |
| `create_waiver -type LINT` | 可用（help 有示例） | 保留 |
| `synth_design -rtl -name rtl_1` / `-rtl_skip_ip` / `-rtl_skip_constraints` | 可用 | `vv_run.sh elab [--skip-ip]` |
| `[Synth 8-xxx]` 消息格式 | 与上游解析器正则一致（实测 8-439/8-448/8-689/8-4446/8-6156 均被解析） | 上游 `parse_elab_messages.py` 原样使用 |
| `report_methodology -json` | **不存在**（2022.2 选项：`-name -cells -checks -file -rpx -append -waived -no_waivers -slack_lesser_than -return_string -merge_exceptions`） | 改为 `-file methodology.rpt` + `get_methodology_violations` 计数 + 上游 `scripts/write_json_report.tcl` 出 JSON |
| 方法学违例的规则字段 `RULE` | 2022.2 对象属性叫 **`CHECK`** | `vv_timing.tcl` 自动判别 CHECK/RULE；读 JSON 时用 `CHECK` |
| `write_xdc -force -exclude_physical` | 可用 | 保留 |
| `find_top`（非工程模式） | 可用 | `--top` 不给时自动猜，多候选时打印 |
| `set_property include_dirs ... [current_fileset]`（非工程模式） | 非工程模式无工程 fileset，不适用 | 改为 `synth_design -include_dirs`；`--inc`、`--define` 透传 |
| 默认器件 `xc7k70tfbg676-2` / 示例 `xcvc1902` | — | 改为 KV260 `xck26-sfvc784-2LV-c`，`--part` 可改 |
| 上游 Step 8 会话内 `reset_timing; read_xdc` 迭代 | 批处理无常驻会话 | 写小 Tcl 一次跑完（SKILL.md 模式 3 第 5 步） |

## 本机环境注意

- `vv_run.sh` 默认 `LD_PRELOAD=/lib/x86_64-linux-gnu/libudev.so.1`：本机 Ubuntu 20.04 上 2022.2 的许可库 dlopen libudev
  会混用 glibc/tcmalloc 分配器导致段错误（构建脚本里已记录，见 `docs/design/M7_PL_BITSTREAM_20260927.md`）。
  只读分析通常碰不到，但预加载无害；`VV_NO_UDEV_PRELOAD=1` 关闭。
- 工作目录固定在输出目录，`.Xil/`、`vivado.log` 都落在那里。校准时用 `find <输入目录> -newer <标记>` 确认未往
  artifacts / runs 原目录写任何文件（结果见 CALIBRATION）。
- 大设计 routed dcp（约 120 MB）：打开 + 时序摘要 + 方法学约 3 分钟（4 线程，机器负载 ~22）。

## 上游已知问题（顺带发现）

- `parse_lint_report.py` 空表问题：表头分隔线后若无数据行，原逻辑会把表尾的 `* create_waiver ...` 说明文字按列位切成
  假违例行。2026.1 走 CSV 不经过这条路，所以上游测试未必覆盖；本副本已修，且 `vv_post.py lint` 会对账
  “Total of X linter message(s)” 与 CSV 行数，任何解析失明都会以 `lint_count_check=MISMATCH` 暴露。
- 上游 elaboration 知识库没有 `8-448`（命名端口不存在）条目，按上游流程落到 `uncovered-synth-messages.md`。

## 交接

（本轮完成，无未尽交接。）
