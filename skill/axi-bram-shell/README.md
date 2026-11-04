# axi-bram-shell：小核的数据口先用"空壳"量成本，命令/结果块不大时用 BRAM 端口代替 m_axi

类型：案例模板 + 测量脚本（HLS 源码与 Tcl）。

## 适用场景

- 一个 HLS 核的 LUT 预算很紧（几千 LUT 以内），而它和 PS 之间要交换的是一块**固定大小、不大**的命令/结果数据（我们的例子是 cmd 1024 字 + res 1024 字）；
- 你默认会写 `#pragma HLS interface mode=m_axi`，但没量过这个适配器本身要吃多少 LUT；
- Zynq UltraScale+ / Vivado block design 流程（PS 能通过 AXI BRAM 控制器访问 PL 里的块存储）。

## 用法

1. **先量外壳，再定接口。** 写一个"只有搬运、没有计算"的桩核（例子在 `examples/`），每种接口一个版本，各跑一次 OOC 实现
   （`export_design -flow impl`，HLS 的 csynth 估计会偏离实数，要以 OOC 为准）：
   - `shell0_maxi_default.cpp`：m_axi 32 位、2 个 outstanding、突发 16，HLS 默认配置；
   - `shella_maxi_slim.cpp` + `shella_maxi_slim.tcl`：m_axi 瘦身（32 位地址 `-m_axi_addr64=false`、单 outstanding、`-m_axi_conservative_mode=false`）；
   - `shellb_bram_port.cpp`：`#pragma HLS interface mode=bram port=blk storage_type=ram_1p`，控制寄存器只留 `s_axilite port=return`。
   Tcl 里用环境变量 `MEMK_SRC`（源码目录）、`MEMK_PRJ`（工程目录，放仓库外）。跑法：
   `MEMK_SRC=$PWD MEMK_PRJ=/tmp/shellb vitis_hls -f shellb_bram_port.tcl`，然后在 Tcl 末尾或交互里加 `csynth_design; export_design -flow impl`。
2. 外壳占预算比例 > 20–30% 时，考虑 BRAM 端口。块设计里加两样：一个 AXI BRAM 控制器（AXI4-Lite、32 位、单口、无 ECC）和一块真双口块存储，
   核的 BRAM 口接块存储的 B 口，控制器接 A 口，控制器挂在 PS 的控制总线上。PS 驱动从"写 s_axilite 数组"改成"写控制器映射的地址"，块内版式不变。
3. **加构建后检查**，防止静默接错：核带 BRAM 口但块设计开关没开、控制器不在 hwh 里、位宽或深度不对，都要直接报错（我们的整板检查器里这一项叫 N6，见下）。

## 失效条件

- 数据量大或是流式（图像、点云）：BRAM 端口只能访问核外那一块固定存储，搬大数据还得 m_axi，这时应该做的是共享一个宽 m_axi，而不是去掉它。
- 块存储 BRAM 很紧时：BRAM 端口把存储挪到了核外，**总 BRAM 不一定省**（我们的例子里核外块存储 2 个 RAMB36，记忆核总量 6 个，比旧版多 1 个）。
- 统一外壳的工程约定会被打破：本工程原本约定"一个 128 位 m_axi + AXI-Lite"，改 BRAM 端口后块设计和驱动都要跟着改，属于跨模块改动，要负责整板的人同意。
- PS 和核同时访问同一块存储没有硬件互斥，靠调用协议保证（核运行期间 PS 不写 res 半块；我们的测试台专门检查了这一条，见下）。

## 已验证的效果

桩核 OOC 实测（Vivado 默认策略，10 ns，器件 xck26），出处：记忆核微程序化工作包的 README 第 2 节与 ARCH 第 6 节：

| 外壳 | 外壳 LUT | 其中 m_axi 适配器 | BRAM |
|---|---|---|---|
| m_axi 32 位默认 | 1,505 | **1,157**（地址 FIFO 各约 185、写节流 135、寄存器片各约 100） | 核内 2 + 适配器 2 个 RAMB18 |
| m_axi 瘦身 | 926 | 650 | 同上 |
| **BRAM 端口** | **30**（只剩控制寄存器） | — | 块存储放核外 |

核外 AXI BRAM 控制器 OOC 综合 27 LUT、12 FF。记忆核按此改完（同时做了架构改写，见 `../hls-microcode-area/`）整核 OOC 1,487 LUT；
这 1,157 LUT 的适配器单独就会吃掉该核 2,500 LUT 预算的 46%。

整板接入（终版整板合并包）：块设计新增 `memk_if=bram` 开关；构建后检查器的 N6 项有阳性对照——从 hwh 删掉 BRAM 控制器、不带开关去查，都报 FAILED。
PS 驱动在宿主上用内存模拟的 BRAM 窗口回放实车与压力录制共 14,886 次调用（9,383 + 5,503），结果与直接调用 C 模型、与录制值都 0 处不同；模拟窗口还检查"两次调用之间 PS 有没有改过 res 半块"，阳性对照（PS 每次往 res 写 1 字）600 次调用中 599 次被这一项抓到（第一次调用之前没有"上一次"可比）。cmd、res 偏移各错 1 字的两个阳性对照 600 次全部报不一致。没做：板上实测。

## 从哪些失败里总结出来

- 记忆核上一版超预算（OOC 9,180 LUT，预算 2,500）时，分项里 AXI-Lite 数组外壳就占 1,418 LUT。我们原来以为接口是"固定小开销"，从没单独量过。
- 搜索核微程序化的第一版沿用了 m_axi，csynth 报 m_axi 适配器 869 LUT、**16 个 RAMB18**，把 BRAM 门直接顶穿——又一次是接口而不是计算出了问题。
- 教训写成规矩：**接口成本先用空壳测出来（价值闸），再决定计算部分的预算**。
