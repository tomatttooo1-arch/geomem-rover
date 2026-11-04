# cdc-guard：IP 自带"时钟对"级 false path 时，用一个会失败的脚本守住"只有 IP 内部才允许跨域"

类型：校验脚本（Vivado Tcl）+ 阳性对照脚本 + 判读清单。

## 适用场景

- 整板里有一个第三方 IP（我们是 AMD DPU，DPUCZDX8G v4.1，S_AXI 独立时钟模式），它的 XDC 里写了**时钟对级**的 `set_false_path -from <clkA> -to <clkB>`；
- 方法学报告里出现 TIMING-54（"IP 作用域约束作用到了全设计"），CDC 报告里出现一批 Critical（我们是 48 条）；
- 你要回答两个问题：这些 Critical 是不是真问题？以后我方新加的跨这对时钟的逻辑会不会被这条约束静默屏蔽？

## 用法

1. **先定性现有的 Critical**（一次性，人工 + 脚本）：用 `get_timing_paths -from clkA -to clkB` 枚举两个方向的全部路径，看起点和终点是不是都在 IP 层级里；
   对照 IP 的 XDC 原文和厂商参考设计的配置（我们核对的是参考设计 Tcl 里 `DPU_SAXICLK_INDPD {1}`，IP 默认值也是 1）。
   全在 IP 内部、约束是厂商原样生成 ⇒ 判为"IP 自己处理，可豁免"；有一条在外面 ⇒ 真问题。
2. **每次整板构建后跑护栏**：
   ```bash
   vivado -mode batch -nojournal -source cdc_guard.tcl -tclargs <routed.dcp> <out_dir> [clkA] [clkB_glob] [inside_glob]
   grep CDC_GUARD <out_dir>/cdc_guard.txt      # 期望 PASS
   ```
   默认参数是我们的设计（`clk_pl_0`、`clk_dpu_*`、`m7_pl_i/dpu/*`），换成你的时钟名和 IP 层级。
3. **第一次用之前跑阳性对照**：`poscontrol_eco.tcl` 在 dcp 副本里加一个越界触发器（时钟接 clkB，D 接 clkA 域的一个我方寄存器），
   对副本原样跑护栏，必须 FAIL 并报出这一条，路径计数应 +1。跑完删掉临时 dcp。
4. 护栏 FAIL 时：那条逻辑必须用 XPM_CDC 同步；而且要注意它自己的 `set_max_delay -datapath_only` 会被 IP 的时钟对 false path 覆盖（false path 优先级更高），
   格雷码偏斜约束会失效——要么换一对时钟，要么改用握手（XPM_CDC_HANDSHAKE）。
5. 文档措辞：不能写"CDC 0 个严重警告"。要拆成两半写：我方部分"全部经同步器"，IP 部分"经 IP 自带约束覆盖，内部同步方式不可见（加密网表）"。

## 失效条件

- 护栏只看你指定的那一对时钟；别的时钟对之间的跨域照样要看 `report_cdc`。
- 只检查路径起点和终点的层级名，不检查同步器结构。IP 内部是加密网表时，"IP 内部是否同步正确"本脚本无法回答，只能依据厂商约束与参考设计。
- `-max_paths 5000` 是上限，路径更多时会漏查（脚本会打 WARN）。
- clkB 的通配必须只匹配一个时钟，否则脚本报 ERROR 退出。
- **参数化版本 `cdc_guard.tcl` 与 `poscontrol_eco.tcl` 是为提交包改写的，没有在 Vivado 上重跑**；默认参数下的判据逻辑与 `validated/` 里跑过的原版相同。第一次用请先跑第 3 步的阳性对照。

## 已验证的效果（Vivado 2022.2，原版脚本，在一个已布线的整板 dcp 上）

| 对照 | 结果（原始输出在 `validated/`） |
|---|---|
| 阴性（原设计） | clk_pl_0 → clk_dpu 209 条、反向 27 条，全部在 DPU 内，**PASS**；计数与 `report_clock_interaction` 一致（209 / 27） |
| 阳性（ECO 加一个越界触发器） | **FAIL outside=1**，报出的正是 `sysinfo/.../cycles_hi_latched_reg[0]/C → ...negctl_ff/D`，路径计数 209 → 210 |
| 现有 48 条 Critical 的定性 | 41 条 CDC-1、4 条 CDC-4（DDR 基地址等准静态配置寄存器）、2 条 CDC-11、1 条 CDC-10，全部在 DPU 内；凡是有 DPU 的构建都固定出现这 48 条（纯 DPU 探针也是），没有 DPU 的构建是 0 |

## 从哪些失败里总结出来

- 我们的设计文档原来写着"跨时钟域报告 0 个严重警告""方法学 0 个新增严重项"，而产物里实际是 48 + 2。是写文档时按"应该如此"填的，没有回到报告核对。
- 加了 DPU 以后 Critical 从 0 跳到 48，第一反应是"拼接逻辑出问题了"。逐条枚举后发现全是 IP 内部、而且只要有 DPU 就固定出现——判读前先找"同一现象在别的构建里出不出现"。
- 第一版护栏只有阴性结果（PASS），没有阳性对照，等于不知道它能不能抓到越界。后来用 ECO 造了一条越界路径，才证明它会 FAIL。
- 用 `report_cdc -cells <IP 层级>` 想只看 IP 内部，返回空表——这种用法在层级单元上没有判别力，不能拿来下结论。
