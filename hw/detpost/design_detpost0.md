<!-- 本文件是 runs/nationals-prelim-20261102/DETPOST0/README.md（主文档，runs/ 不入库）在提交时的副本；结果 JSON 在该目录 results/。 -->
# 检测后处理核：调研、DPU 输出实情、价值闸方案、规格草案、C 参考模型（DETPOST0）

用了 1 个小构建槽（2 号）跑一次 csynth，用完已释放。

## 0. 一页结论

- **为什么做这一包**：完整版硬件顺序是 DPU → 检测后处理核 → 16 位深度直进 PL → ORB。DPU 只算到三个卷积输出头，
  之后的「反量化、DFL 框解码、类别 sigmoid、阈值、NMS」默认在 A53 上跑。本包回答：要不要、怎么把它做成 PL 核。
  33 个类别 logit 在后），**fix_point = 2（步长 0.25）**，三个头在一块连续缓冲里：20×20 在偏移 0、40×40 在 38,800、
  80×80 在 194,000，共 814,800 字节。后处理（fix2float 之后的全部）由 CPU 做——xmodel 里明确是 3 个 CPU 子图。
  「仿真 DPU 输出」＋ 60 帧合成场景）**框集合与顺序 120/120 帧完全相同，类别 100% 一致**；坐标最大误差 0.055 像素，
  分数最大误差 8×10⁻⁶。对照：DPU 输出头本身的 int8 量化就让同一批图的框坐标中位偏 0.35 像素、约 13% 的框换了锚点——
  定点后处理的误差比上游量化小一个数量级以上。
- **价值闸（测量前冻结，§4）**：时延理由三条——L1：最好的软件实现在 A53 上的中位时延 ≥ DPU 中位时延的 10%；
  L2：与几何核并发时 p99 ≥ 5 ms；L3：按识别帧率并发时几何核每帧 call_ms 中位 +3% 或 p99 +10%（且大于基线漂移 3 倍）。
  任一成立即「时延上值得做」。**「核心链路全在 FPGA」是另一条独立理由，与时延闸分开写（§4.4），交主控决定。**
  板测脚本已写好未上板（`hw/detpost/bench/`）。
- **规格（§6）**：四级流水（扫描判阈值 → 只对过阈值的格做 DFL 解码 → 按 int8 分数档计数排序 → 与已保留框并行比 IoU 的贪心 NMS），
  全整数，不需要 exp/sigmoid 运算器（两张主机写入的查表）。**LUT 预算上限 6,000，目标 ≤4,500；DSP ≤12；RAMB36 ≤6；URAM 0。**
- **csynth（未优化的参考模型直接综合，只取量级）**：20,420 LUT / 14,561 FF / 49 DSP / 22 BRAM18，估计时钟 4.38 ns（目标 6 ns）。
  这是 HLS 直出、未做任何优化的数，超预算 3 倍多；大头可指认（64 位除法器、64 位 IoU 乘法、8 位宽数据口），§8 给出逐项压缩路径。

## 1. 调研：已发表的 FPGA 检测后处理 / NMS 硬件

检索方式：OpenAlex 与网页检索（关键词：FPGA NMS accelerator、YOLO post-processing FPGA、YOLOv8 DFL FPGA）、AMD Vitis AI 源码。
数字取自摘要或可读到的正文；没读到全文的标「摘要」。

| 工作 | 对象 | 平台 | 结构要点 | 吞吐/时延 | 资源 | 精度口径 |
|---|---|---|---|---|---|---|
| AMD Vitis AI Library `yolov8_post_process`（v3.5 源码，[yolov8.cpp](https://github.com/Xilinx/Vitis-AI/blob/v3.5/src/vai_library/xnnpp/src/yolov8.cpp)、[apply_nms.cpp](https://github.com/Xilinx/Vitis-AI/blob/v3.5/src/vai_library/xnnpp/src/apply_nms.cpp)） | YOLOv8 | **A53 软件** | 对全部 8400 格先做 DFL softmax（逐格 vector 分配、expf）再判阈值；类别阈值换成 int8 比较；多标签（每个过阈值类别出一个框）；按类 NMS，IoU ≥ 阈值抑制 | 本包移植计时，宿主机参考 7–12 ms/帧（§4.3） | — | 与 ultralytics 不同：多标签、阈值换算向零截断（负阈值时高一档：conf 0.25 时漏掉 q=−4 那一档 0.269 分）、`output_dim` 写死 80 类 |
| Guo 等，FPT 2023，[10.1109/icfpt59805.2023.00019](https://doi.org/10.1109/icfpt59805.2023.00019)（摘要） | YOLOv3-tiny | Virtex-7 690T，150 MHz | 扫描/缓存、解码、类别判定、NMS 融合成一条流水；并行 IoU 与阈值比较 | 后处理 4.46 µs/帧，NMS 0.07 µs | 摘要未给 | 摘要未给 |
| Wang、Xu、Chen，ICCTIT 2024，[10.1109/icctit64404.2024.10928440](https://doi.org/10.1109/icctit64404.2024.10928440)（摘要） | 通用 NMS | VCK190 | 预处理单元先定每框最大类别；新框与已存框并行比分数与 IoU 决定去留 | 比 O3NMS 快约 20% | LUT +21.4%、FF −81.5%（相对 O3NMS） | — |
| Chen 等 O3NMS，ISCAS 2023，[10.1109/iscas46773.2023.10181731](https://doi.org/10.1109/iscas46773.2023.10181731)（摘要） | 通用 NMS | Xilinx FPGA＋40 nm | 去掉预排序，乱序 NMS | 2.51× 于此前 SOTA | 资源 −37% | 软硬件协同 |
| Shi 等，TCAS-II 2019，[10.1109/tcsii.2019.2893527](https://doi.org/10.1109/tcsii.2019.2893527)（摘要＋DATE'22 引述） | 人脸检测 NMS | TSMC 28 nm | 多线程＋滑窗并行、按位置位表复用 | 1000 框 12.79 µs@400 MHz | 6.142 mW | — |
| Chen 等 ShapoolNMS，DATE 2022（[PDF](https://past.date-conference.com/proceedings-archive/2022/pdf/0239.pdf)，已读正文） | PSRR-MaxpoolNMS | Genesys 2（Kintex-7） | 分数图上做池化代替两两 IoU，O(n) | 比硬件 GreedyNMS 快 305× | 2 路 14,188 LUT、2,924 FF、7,504 LUTRAM | mAP 比 PSRR 软件低 0.94% |
| Anupreetham 等，FPL 2021 / TRETS 2023，[10.1109/fpl53798.2021.00021](https://doi.org/10.1109/fpl53798.2021.00021)（摘要） | SSD-MobileNet | Intel FPGA | 新 NMS 算法让 NMS 与卷积流水重叠，免去等全部框 | 端到端低时延 | — | — |
| Pestana 等，IEEE Access 2021，[10.1109/access.2021.3081818](https://doi.org/10.1109/access.2021.3081818)（摘要） | YOLOv3/v4-tiny | XCKU040 | 预处理、推理、后处理全进核；16 位定点、分段线性激活 | 32 / 31 fps | — | — |
| MaxpoolNMS（CVPR 2019）/ PSRR-MaxpoolNMS（CVPR 2021）/ ++（TPAMI 2024） | 算法 | — | 用池化近似 GreedyNMS 以便并行 | — | — | 近似：单阶段检测器末级直接替换会掉点 |

本次检索没找到公开的、在 FPGA 上实现 **YOLOv8 DFL 解码**的后处理核（有 YOLOv8 主干上 FPGA 的工作，后处理仍在 CPU）——
这只说明检索范围内没有，不等于不存在。

**能直接借来的**：①「先判阈值、只解码少数候选」与「新框与已保留框并行比 IoU」——Guo 2023、Wang 2024 都是这个结构，我们照用并引用；
②Vitis AI 把类别阈值换成 int8 比较的做法（但修正其截断错误）。**不借的**：MaxpoolNMS 一类近似算法（与 ultralytics 结果不再逐框一致，
会让精度口径变成「近似 NMS」，要另测 mAP）；Shapool 的分数图存储（640 输入下按分数图存要几十 KB，我们候选只有几十个，不划算）。

**我们相对已有工作多做的（待硬件实现后实测才算数）**：
1. 利用 DPU 输出就是 int8 这一点：sigmoid 单调，所以阈值、排序都直接在 int8 logit 上做，**整核不需要 exp/sigmoid 运算器**；
   排序变成 256 档的计数排序（O(N)、精确、稳定），不需要比较排序网络。
2. DFL 期望 = Σi·e^{q_i}/Σe^{q_i}：减去本边最大值后查 256 项指数表（主机按 fix_point 生成），整数累加，每边一次除法（Q.10）。
3. IoU 判定写成 `inter × den > num × union`（阈值 0.7 = 7/10），无除法、整数精确；和浮点只差坐标量化。
4. 与 ultralytics 逐框一致作为验收口径（不是近似 NMS）：120 帧框集合、顺序、类别全同（§7）。

## 2. 车上现行后处理参数（核实过的，与本核默认值一致）

- 车上识别：Nano 上 TensorRT FP16 的 YOLOv8s 六类（赛用1，`console-direct-4988645` 的 `runtime_detector.env` 指向
  `saiyong1-fp16.engine`，PROFILE combined33）。TensorRT 引擎里已含 DFL 解码，所以车上 Python 只做阈值与 NMS。
- `perception/fpga_replay/yolo_detector.py`（4988645）与 `tools/nano_detector/nano_detector.py`：**conf 0.25、NMS IoU 0.7（IoU > 阈值抑制）、
  max_det 100、按类 NMS**；NMS 和截 max_det 之后再做类别白名单与按类下限（`host/inference_service.py`：33 类模型 `HOUSEHOLD33_CLASS_MIN_SCORES`
  书包 0.35，COCO 模型书包 0.45）。
- 本核默认值按此设：conf 0.25、IoU 7/10、max_det 100、NMS 后白名单＋按类下限（寄存器可配）。


用 protobuf 原始解码直接读 op / tensor 属性（脚本在 scratchpad，方法：按字段号解出 `op_node` 的输出张量 `tensor_dim`、`tensor_attr`）。
与 `quant_info.json`（量化器记录）交叉一致。

| 张量（concat-fix 输出，进 download） | dims | 类型 | fix_point | ddr_addr | 每格步长 |
|---|---|---|---|---|---|
| `…ModuleList_22__11640`（P5） | 1×20×20×97 | int8 有符号 | 2 | 0 | 97 字节 |
| `…ModuleList_22__11491`（P4） | 1×40×40×97 | int8 | 2 | 38,800 | 97 |
| `…ModuleList_22__11342`（P3） | 1×80×80×97 | int8 | 2 | 194,000 | 97 |
| 输入 `RawYOLO__input_0` | 1×640×640×3 | int8 | 6 | 1,936（reg 2） | — |

- 三个输出都在 reg_id 3（输出区），按上表偏移首尾相接，共 814,800 字节；每格 97 字节紧排，无对齐填充。
- 通道顺序：concat 的两个输入依次是 `cv2`（框，64 通道）与 `cv3`（类别，33 通道）。
- **框 logit 和类别 logit 共用一个 fix_point=2**（concat 强制同尺度）：类别 logit 可表示范围 [−32, 31.75]，步长 0.25。
  conf 0.25 对应 logit −1.0986，整数阈值 q ≥ −4（sigmoid(−1.0)=0.269 留，sigmoid(−1.25)=0.223 不留）。
- 子图：1 个 USER（输入）、1 个 DPU（244 个 op）、3 个 CPU（fix2float）。DFL、sigmoid、解码、NMS 都不在 DPU 里。
- **没核到的**：板上 VART 实际分给输出张量的缓冲是否保持这个排列（同一 reg 区一般是）——`bench/dump_dpu_out.py` 第一次上板会打印并核对。

## 4. 价值闸方案（2026-11-03 测量前冻结；改动只能另开带日期的节）

### 4.1 量什么、用哪段代码

| 读数 | 代码 | 口径 |
|---|---|---|
| T_sw（主读数） | `hw/detpost/bench/detpost_fast.cpp`：本包定点参考模型，扫描一步用 NEON | 「最好的软件实现」。KV260 A53、单核钉核、**冷缓存**（每帧前 DC CIVAC 清掉输入，模拟 DPU 刚写完 DDR）、2000 次 |
| T_sw_ref | `detpost_ref.cpp` 原样 | 参考：未向量化的同一算法 |
| T_sw_vai | `bench/vitis_yolov8_port.cpp`：Vitis AI 官方 YOLOv8 后处理原样移植 | 参考：开箱即用的官方做法 |
| T_sw_numpy | `bench/bench_detpost_np.py`（浮点参考，NumPy 全量向量化） | 参考：Python 产品驱动若用 NumPy 做的代价 |
| T_dpu | `bench/dump_dpu_out.py`：VART execute_async+wait | 同一 xmodel，第一张图重复 50 次的中位 |
| 几何核干扰 | `bench/run_board_bench.sh`：rec30 310 帧产品驱动回放（上板二口径），A 基线 → 并发（按识别帧率 / 背靠背）→ B 基线 | 几何核 `frames.csv` 的 call_ms（CPU 侧）与 device_ms（PL 计数）中位与 p99 |

输入两种：**真实 DPU 输出**（`dump_dpu_out.py` 板上抓）为准；没有时用 `gen_inputs.py model` 的仿真帧，结论标「线索」。
用最好的软件实现（fast）作分子，是为了不和稻草人比——拿官方移植版（多半慢一个数量级）去过闸会把硬件的收益夸大。

### 4.2 时延闸阈值与理由（`bench/gate_eval.py` 逐字实现）

- **L1：T_sw 中位 ≥ 0.10 × T_dpu 中位。** 识别链是 DPU → 后处理串行，后处理每帧直接加在识别时延上；PL 核预计 0.3–0.5 ms，
  省下的≈T_sw。10% 是「把 DPU 时钟再提一档（250→275 MHz）」量级的收益，低于它，做核换来的时延改善不如别的手段。
  0.05–0.10 为灰区，交主控。
- **L2：与几何核并发、按识别帧率调用时 T_sw 的 p99 ≥ 5.0 ms。** 5 ms 是相机帧周期 33 ms 的 15%，也约等于 1.5 个几何帧（rec30 3.16 ms）。
  尾时延超过它，后处理会成为整链抖动的来源。
- **L3：按识别帧率并发时，几何核每帧 call_ms 中位 +3% 或 p99 +10%，且增量 ≥ 两次基线漂移的 3 倍。**
  上板二基线复现差 0.03%，3% 约是噪声的百倍；p99 天然更抖，放宽到 10%。只在背靠背压力下成立 → 灰区。
- **任一条成立 = 时延上值得做硬件。** 全不成立 = 时延理由不支持；此时是否做，只看 §4.4。

### 4.3 测前预期（写在测量前，测后对照）

宿主机（x86，负载 45/48 核，仅作量级参考，不进闸）：fast 0.22 ms、ref 0.76 ms、vai 7–12 ms、NumPy 25 ms（中位，冷缓存）。
A53 单核大约慢 3–8 倍 → fast 预计 0.7–2 ms、vai 20–60 ms、NumPy 100 ms 以上；YOLOv8n 在 B1024@250 MHz 上 DPU 预计数十毫秒。
**所以 L1 预计不过（fast/T_dpu 约 2–5%）**，L2 预计不过；L3 不确定（vai/NumPy 路径大概率触发，fast 路径大概率不触发）。
这个预期本身说明：如果识别链将来用 Python + NumPy 做后处理，CPU 代价会很大——那时该优先上 fast 这一类 C 实现，而不是因此就做硬件。

### 4.4 独立理由：「核心链路全在 FPGA」（与时延闸分开，交主控决定）

- 国赛方针：核心链路全在 FPGA，FPGA 只和自己（优化前后）及已发表方法比。识别链里后处理若留在 A53，数据路径是
  PL（DPU）→ DDR → **A53** → DDR → PL（ROI 证据核），「识别全在 FPGA」这句话就不成立。做成 PL 核后，CPU 只剩下发起与收尾（写一个启动寄存器、读一个框数）。
- 代价：LUT 预算 4.5–6k（§6.5）、HLS 优化与联仿约 2–3 天工作量、多一个上板验证项；它与 16 位深度直进 PL、ORB 抢同一块 LUT 余量。
- 这条理由成立与否不需要板测；需要主控在「方针完整性」与「ORB 等后续项的余量」之间取舍。

## 5. 语义（定点模型与浮点参考共同遵守）

按 ultralytics 8.3.40 predict：`Detect` 头 DFL softmax 期望 → `dist2bbox`（×stride）→ 类别 sigmoid → `non_max_suppression`
（每格取最大类别、分数 > conf、按分数降序、按类 NMS IoU > iou、前 max_det）→ 车上附加：白名单与按类下限（score ≥ 下限）。

几处 ultralytics 自身含糊的地方，本包的取法：
- 同分排序：取锚点序（稳定排序；ultralytics 的 torch argsort 不保证稳定）。
- float32 sigmoid 在 logit ≥ 约 16.6 时饱和成 1.0：ultralytics 在这些框之间按锚点序排，多个类别都饱和时取下标小的类别；
  定点模型按 int8 logit 原值排、取 logit 最大的类别。真实帧（logit 上限 31.75）上没触发差异；随机压力帧上触发（§7）。
- ultralytics 用「类别 × 7680」平移坐标做按类 NMS，float32 下大类别号损失约 1/64 像素；浮点参考可切成精确逐类 NMS 对照，结果相同。

## 6. 规格草案（硬件核）

### 6.1 接口
- AXI-Lite（挂现有 m7 槽位窗口，占一个 SLOT_MAP 位）：控制/状态、输入缓冲物理地址、三个头的偏移、网格与 stride、nc、cls_thr_q、
  IoU num/den、max_det、按类下限与白名单（各 nc 字节）、指数表与 sigmoid 表（各 256×16 位，主机初始化时写一次）；
  状态：候选数、溢出标志与抬高后的阈值、NMS 后框数、输出框数、周期计数。
- AXI 主口（128 位）：读 DPU 输出缓冲（814,800 字节/帧），写框表（每框 16 字节：x1,y1,x2,y2 为 int16 Q4 像素；类别、int8 分数档、
  UQ0.16 分数、锚点号）。走 HP3（若空闲），否则经 SmartConnect 与 ROI 核共用 HP1；带宽每帧 0.8 MB，5–10 帧/秒时 ≤8 MB/s，可忽略。
- 完成中断：pl_ps_irq 一位。

### 6.2 数据流
1. **扫描**：128 位突发读整块缓冲，按 97 字节一格重排成字节流；每格对 nc 个类别字节求最大与下标（16 路比较树），与 cls_thr_q 比。
   约 50,925 拍（814,800/16）；166.67 MHz 下 0.31 ms。
2. **候选 DFL 解码**：过阈值的格把锚点号送入小 FIFO；解码单元按锚点号**回读**该格 64 字节（一次 64 字节突发），每边：求最大 →
   查指数表 → 累加 S、W → 一次 W·1024/S（Q.10，迭代除法器）→ 坐标 Q4。候选稀疏（真实图片中位 12 个/帧、最多 86 个），回读开销 ≤约 0.1 ms。
3. **候选存储＋计数排序**：CAND_MAX 行（每行约 94 位）＋ 256 档直方图；扫描完后前缀和、按档散列出排好序的下标表。
4. **NMS**：保留表最多 max_det 个框（LUTRAM，P 路并行），新框与同类已保留框并行算交集，判 `inter·den > num·union`。
5. **写出**：白名单/下限过滤，写框表，置完成。

### 6.3 定点位宽与上限
- 类别/框 logit int8；指数表 UQ1.15（d=m−q 0..255）；S ≤ 2^19，W < 2^23；距离 Q.10（12+10 位）；坐标 int16 Q4（±2047 像素）；
  面积 30 位；IoU 判定积 ≤ 37 位（den ≤ 127 时）。
- **为什么距离取 Q.10**：Q.8 时 60 帧合成场景有 2 帧在 IoU 0.7 边界上翻转（例：浮点 IoU 0.70069、定点算出 ≤0.7）；Q.10 后 0 帧。代价是除法器宽 2 位。
- CAND_MAX：参考模型用 1024；真实帧最多 86 → **硬件建议 256**（4 KB 级存储）。溢出时的规则（精确、可解释）：存储满且不止一个分数档时，
  整档丢掉最低档并把阈值抬一档；全在同一档时同分留先来的、更高分的新候选顶掉最后一个。坏处：溢出帧上与 ultralytics（上限 30000）
  不再逐框一致；溢出标志与抬高后的阈值写进状态寄存器，主机可见。
- MAX_DET 128（车上 100）。

### 6.4 与 DPU 的同步
- **默认（软件发起）**：A53 上 VART `wait()` 返回后，写本核的输入物理地址并启动；完成中断后读框表。DPU 输出由 DPU 经 HP 口直接写 DDR，
  本核经 HP 口读，不经 CPU 缓存，无一致性问题；框表缓冲沿用几何核驱动的 dmaheap＋缓存维护做法。
- 可选（硬件串联）：把 DPU 完成中断线在 PL 内引一份作为本核启动沿，省一次 CPU 唤醒（几十微秒）。前提是 VART 每帧复用同一输出缓冲
  （物理地址不变）——未核实，且 CPU 仍需为每帧启动 DPU，所以控制路径上的 CPU 去不掉。不建议首版做。

### 6.5 资源预算与理由
- 现状：下一版产品比特流（MERGE1，无 DPU）50,826 LUT；DPU B1024 本身 33,759 LUT（D0 第二轮）；合起来约 86–87k（74%）。
  D0 第一轮 91.9% LUT 时布线拥塞、时序失败；第二轮 83.1%、切片 99.6% 时靠降频才收住。
- **上限 6,000 LUT、目标 ≤4,500；DSP ≤12；RAMB36 ≤6（候选存储＋两张表＋框表缓冲）；URAM 0。** 理由：加本核后整板约 91k（78%），
  给 16 位深度直进 PL 留 1–2k，仍低于 80%；ORB（公开 FPGA 实现一般上万 LUT）放不进剩余余量，需要另行腾挪——这一点要主控提前知道。
- 手算构成（目标）：AXI 主口读写约 1.4k、AXI-Lite 约 0.7k、扫描重排＋16 路比较约 1.0k、DFL（迭代除法器＋表）约 0.5k、
  候选/排序约 0.4k、NMS（P=2）约 0.6k＋2–4 DSP、写出约 0.2k。

## 7. C 参考模型与浮点参考的比较

代码：`hw/detpost/`——`detpost.h`、`detpost_ref.cpp`（可综合风格：无动态内存、循环上界固定）、`detpost_host.cpp`（阈值换算、查表）、
`detpost_cli.cpp`；浮点参考 `detpost_float.py`；输入生成 `gen_inputs.py`；比较 `compare.py`；头量化影响 `tools/quant_effect.py`。

结果（`results/cmp_*.json`，车上参数 conf 0.25、IoU 7/10、max_det 100）：

| 输入 | 帧 | 框集合相同 | 顺序相同 | 框数（定点/浮点） | 类别一致 | 坐标最大误差 | 分数最大误差 |
|---|---|---|---|---|---|---|---|
| 同上，加书包式下限（类 24 ≥0.35，仅测机制） | 60 | 60 | 60 | 326 / 326 | 全一致 | 0.055 px | 8×10⁻⁶ |
| 合成场景（1–24 个物体，跨尺度重复框） | 60 | 60 | 60 | 1000 / 1000 | 全一致 | 0.048 px | 8×10⁻⁶ |
| 随机 int8（每帧 8400 格全过阈值，压溢出），浮点按 int8 排序、上限 1024 | 3 | 3 | 3 | 300 / 300 | 全一致 | 0.036 px | 0 |
| 随机 int8，浮点按纯 ultralytics 语义 | 3 | 0 | 0 | 300 / 300 | 5/35 | — | — |

- 最后一行的差别是预期的：随机数据里大量类别 logit 远超 16.6，float32 sigmoid 全饱和成 1.0，ultralytics 的排序与取类变成按下标，
  且候选上限 30000 对 1024。用同一排序键、同一上限比时（倒数第二行）完全一致——差别全来自这两条语义，不是定点误差。
- 仿真帧候选数：中位 12、最多 86；真实 DPU 输出可能不同（DPU 内部逐层量化）。
- 头量化背景量（`results/quant_effect.json`）：同 60 张图，浮点头 vs 量化头（都用浮点后处理）：310 vs 327 框、269 个同锚点，
  同类 IoU≥0.5 配对的坐标差中位 0.35 px、p90 1.51 px。定点后处理误差（≤0.055 px、0 个集合差）远小于此。
- 仪器检查：比较器阳性对照（某框挪 1 像素 / 删一框 / 改类别）每次都报出；快速版（NEON）与参考模型逐字节相同：
  宿主 SSE4.1 路径 123 帧 0 差、标量路径 63 帧 0 差、**aarch64 NEON 路径（交叉编译 -mcpu=cortex-a53，qemu 运行）123 帧 0 差**。
- **未做的校准（遗留）**：浮点参考只做到「按 ultralytics 源码逐步写、与定点模型互证」，还没有在同一输入上对 ultralytics 库本身或
  不能排除同一处理解错误；补法见 HANDOVER。

## 8. csynth 量级（2 号小槽，`build/hls`，`results/csynth_summary.json`）

参考模型原样包顶层、只加接口 pragma、6.0 ns（166.67 MHz）、xck26：**20,420 LUT / 14,561 FF / 49 DSP / 22 BRAM18 / 0 URAM**，估计 4.38 ns；
无被忽略的 pragma（Pragma 报告空）。变长循环，总时延未给出。

| 部分 | LUT | 说明与压缩路径 |
|---|---|---|
| DFL 四边循环（含内联除法） | 5,215，II=32 | `(uint64)w·1024/s` 被综合成 64 位除法器。改 33/20 位迭代除法器、四边共用 → 约 0.5k |
| NMS 内层 | 4,621，II=9 | int64 乘法（inter×den、num×union）全在 LUT 里。改 30×7 位、P=2 → 约 0.6k＋2–4 DSP |
| detpost_ref 其余（扫描、溢出压缩、候选存储、排序） | 约 4.6k＋44 DSP | 8 位数据口未加宽（214-115 只推出 8 位突发）；溢出压缩循环；改 128 位口＋字节重排、CAND_MAX 256 |
| 顶层外壳与接口 | 约 4.5k | 两张表在 AXI-Lite 里各 256 项；改成 BRAM 表、主机初始化写 |

结论：直出数超预算约 3.4 倍，但大头都是可指认的位宽/结构浪费，按上表逐项改后落到 4.5–6k 有依据；没做之前这只是手算估计。

## 9. 降级选项及其坏处（如果不做或少做）

| 选项 | 好处 | 坏处 |
|---|---|---|
| 不做核，A53 上用 fast 版 | 零 LUT、零开发 | 「核心链路全在 FPGA」不成立；后处理占一个 A53 核的 2–5%（预估）；尾时延受 Linux 调度影响 |
| 只把扫描判阈值放 PL，其余留 CPU | LUT 约减半（~2.5k） | CPU 仍在数据路径上，方针理由只兑现一半；CPU 需读候选表，多一次交互 |
| CAND_MAX 降到 64 | 存储更小 | 仿真帧最多已 86 候选，会触发溢出、与 ultralytics 不再逐框一致 |
| 距离 Q.8 | 除法器窄 2 位 | 60 帧合成场景 2 帧在 IoU 边界翻转 |
| 用 MaxpoolNMS 类近似 NMS | 无两两 IoU | 与车上 NMS 结果不再一致，需重新测 mAP |

## 10. 产物与复现

- 代码：`hw/detpost/`（见 §7）；`hw/detpost/bench/`（`bench_detpost.cpp`、`detpost_fast.cpp`、`vitis_yolov8_port.cpp`、
  `bench_detpost_np.py`、`dump_dpu_out.py`、`run_board_bench.sh`、`gate_eval.py`）；`hw/detpost/hls/`（csynth 外壳与 tcl）；
- 结果：`results/`（比较、csynth 摘要、输入清单）。输入帧本身（每帧 815 KB）未入库，按 `results/manifest_*.json` 与固定种子 20261103 可重生。
- 复现：
  ```bash
  cd hw/detpost && g++ -O2 -std=c++14 -o /tmp/dp_cli detpost_cli.cpp detpost_ref.cpp detpost_host.cpp
  python3 gen_inputs.py scene --n 60 --out F_scene && python3 compare.py --cli /tmp/dp_cli --frames F_scene
  python3 gen_inputs.py model --images <图片…> --out F_model && python3 compare.py --cli /tmp/dp_cli --frames F_model
  ```
- 板测（主控择机）：`dump_dpu_out.py`（抓真实输出＋T_dpu）→ `run_board_bench.sh <帧目录> <输出目录>`（设 GEO_CMD）→ `gate_eval.py`。
