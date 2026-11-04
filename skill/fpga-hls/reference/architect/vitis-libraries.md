> 来源：AMD Ross `hls-architect/reference/vitis-libraries.md`，MIT 许可，见 `../../upstream.md`。已按 Vitis HLS 2022.2 移植。
> 上游描述：Vitis Libraries study guide — browse L1 source code to learn HLS coding patterns. DO NOT include library headers in user designs. Study, then write your own.
<!--
Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
-->

> **2022.2 注记**：上游指向 Vitis_Libraries 的 `2025.1` 分支。本机工具是 2022.2，学习时应改看与之配套的 `2022.2` 分支
> （把下文 URL 里的 `2025.1` 换成 `2022.2`）。2025.1 分支的代码可能用到 2022.2 没有的头文件或写法（例如 `hls_array_partition.h`、
> `ap_float.h`，见 `../../porting_notes.md` §2），照抄前先对照该清单。`2022.2` 分支是否存在、目录结构是否相同**未联网核实**
> （本机无 Vitis_Libraries 副本）；打不开时退回 2025.1 分支并按上面的清单逐项核对。本文件的原则（只学模式、不 include 库头文件）不变。

# Pattern References — Study, Then Write Your Own Code

**Rule for all references**: DO NOT include or link any headers or source files in user designs. Use them as **pattern references only** — read the implementation, understand the HLS coding style, then write equivalent code from scratch in the user's own kernel.

---

## Vitis Libraries (Production AMD/Xilinx HLS Code)

**Repository**: https://github.com/Xilinx/Vitis_Libraries/tree/2025.1

Production-quality HLS implementations covering vision, DSP, linear algebra, data compression, and more. Study L1 source code to learn streaming interfaces, pragma usage, AXI/stream bridging, and memory optimization patterns.

---

## How to Look Up Library Code

1. **Browse the L1 include directory** for your domain (see Library Domains table below):
   `https://github.com/Xilinx/Vitis_Libraries/tree/2025.1/<domain>/L1/include/`

2. **Read the raw source** to study the implementation:
   `https://raw.githubusercontent.com/Xilinx/Vitis_Libraries/2025.1/<domain>/L1/include/hw/<file>.hpp`

3. **Extract the pattern** and write your own kernel from scratch following that style

---

**Focus on L1** (individual HLS kernels for C-sim/csynth/cosim). L2/L3 are for complete XCLBIN/multi-kernel apps.

---

## Library Domains

| Domain | Path | Key Primitives |
|---|---|---|
| **DSP** | `dsp/L1/include/hw/` | FFT (1D, 2D), FIR filters, DDS, matrix multiply (GeMM) |
| **Vision** | `vision/L1/include/` | filter2D, resize, threshold, morphology, Harris, FAST, line buffer, window buffer |
| **Utils / Streams** | `utils/L1/include/xf_utils_hw/` | AXI↔stream bridges, stream split/combine/dup/reorder/sync |
| **Data Mover** | `data_mover/L1/include/xf_data_mover/` | Load master→stream, store stream→master, 4D data movers |
| **BLAS** | `blas/L1/include/` | GEMM, GEMV, DOT, AXPY — matrix/vector ops |
| **Solver** | `solver/L1/include/` | QR, SVD, Cholesky, linear system solver |
| **Data Compression** | `data_compression/L1/include/` | LZ4, Snappy, Zlib, Zstd |
| **Security** | `security/L1/include/` | AES, SHA, RSA, ECC |
| **HPC** | `hpc/L1/include/` | GEMM variants, systolic arrays |


