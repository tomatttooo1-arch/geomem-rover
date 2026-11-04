# 逐位一致门（bitexact_check.py）

Ross 的 hls-optimize 只把 csim 通过当功能门、cosim 只读周期数。我们的要求更严：**每次优化后，csim 输出和 cosim 输出都必须与基线逐字节一致**。
这份说明讲 testbench 要怎么写、golden 怎么建、怎么判。校准记录见 `calibration_20261102.md` 第 4 节。

## 1. testbench 约定

1. 把顶层函数的**全部输出**（每个输出数组、标量返回值、状态字）写成文件，放在**相对当前工作目录**的 `tb_out/` 目录下（先 `mkdir("tb_out")`）。
   - 二进制输出按固定字节序写原始值（如 `fwrite` 每个元素的整型位模式；`ap_uint<W>` 用 `.to_uint()`/`.to_uint64()` 后按固定宽度写；浮点按 IEEE 位模式写，不要 `printf("%f")`，打印会丢位）。
   - 可以额外写一份文本（`%d`/`%u`/十六进制位模式），方便看差异落在哪一行。
   - 文件名、文件个数、写出顺序在优化前后必须固定；不要写时间戳、地址、随机种子未固定的内容。
2. 输入必须确定：固定种子或固定数据文件。
3. testbench 正常结束返回 0（自检失败另说），否则 cosim 不会进入后处理阶段。
4. 可选：环境变量 `HLS_TB_OUT_DIR` 覆盖输出目录。**不建议在 cosim 时设它**：cosim 会跑两遍 testbench（见下），共用一个目录会让后一遍覆盖前一遍。

为什么用相对目录：2022.2 每个阶段在不同工作目录里运行 testbench，相对路径自然把各阶段的输出分开（2026-11-02 在玩具核上实测）：

| 阶段 | testbench 的工作目录 | 输出位置 | 内容来自 |
|---|---|---|---|
| csim | `<proj>/<sol>/csim/build` | `csim/build/tb_out/` | C 模型 |
| cosim 第 1 遍（生成激励） | `<proj>/<sol>/sim/wrapc` | `sim/wrapc/tb_out/` | C 模型 |
| cosim 第 2 遍（post-check） | `<proj>/<sol>/sim/wrapc_pc` | `sim/wrapc_pc/tb_out/` | **RTL 仿真输出**（`apatb_<top>.cpp` 以 `POST_CHECK` 编译，从 `sim/tv/rtldatafile/rtl.<top>.autotvout_*.dat` 读回 RTL 结果再交给 testbench） |

所以 cosim 要比的是 `sim/wrapc_pc/tb_out/`，不是 `sim/wrapc/tb_out/`。

## 2. 建 golden

在基线版本上跑 `hls_run.sh <proj> <sol> csim`（或 `csim csynth cosim`），然后把 csim 输出复制到**工程目录之外**（重跑 csim 会清掉 `csim/build`）：

```bash
cp -a <proj>/<sol>/csim/build/tb_out  <优化记录目录>/golden_tb_out
```

基线自己也要过一遍：如果基线跑了 cosim，先比一次
`bitexact_check.py golden_tb_out <proj>/<sol>/sim/wrapc_pc/tb_out`，确认基线 RTL 与基线 C 模型一致，golden 才算立住。

## 3. 每次优化尝试后的判定

```bash
bitexact_check.py <golden> <proj>/<sol>/csim/build/tb_out        # 必做
bitexact_check.py <golden> <proj>/<sol>/sim/wrapc_pc/tb_out      # 跑了 cosim 就必做
```

- 每个 golden 文件打印 `SAME` / `DIFF`（第一处差异的字节偏移、两边字节值；文本文件附行号；`--elem-size N` 附元素下标 = 偏移 // N）/ `MISSING`；candidate 多出的文件记 `EXTRA`，默认也算失败（`--allow-extra` 可放行）。
- 退出码：0 = 全部一致（`RESULT BITEXACT`）；1 = 有任何不一致/缺失/多出；2 = 用法错误或 golden 目录为空（空 golden 不能当"一致"）。
- **任一不一致即判该次优化无效**，回退，不得以"误差很小"放行。

## 4. 它管不到的地方

- 只覆盖 testbench 实际喂进去的输入；输入覆盖面不够，逐位一致也只是在这些输入上一致。
- testbench 没写出的输出（漏写的端口、副作用）不在比对范围内，所以第 1 条"全部输出"是硬要求。
- cosim 只证明 RTL 仿真与 C 一致，不证明上板一致（时序收敛、接口时序另有门）。
