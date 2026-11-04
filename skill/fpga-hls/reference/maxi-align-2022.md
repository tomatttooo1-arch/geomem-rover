# m_axi 自动加宽需要编译器可见的对齐（Vitis HLS 2022.2 实测经验）

来源：2026-11-02 一次在用核的优化（见 `../README.md`"已验证的效果"第 2 条）。智能体想把一个 `float*` 深度图端口从 32 位加宽到 64 位，
以便每拍读 2 个像素。8 次尝试里有 4 次（第 3–6 次）卡在这里，原技能包 reference 里没有这一条，所以补上。

## 现象

- 端口声明成 `float *depth`，`#pragma HLS INTERFACE m_axi ... max_widen_bitwidth=64`，循环每拍读两个相邻 float，csynth 仍然报端口 32 位、每拍 1 次读。
- 以下写法在 2022.2 下**没有**让 HLS 认出 8 字节对齐（csynth 报告里端口宽度不变）：
  - 在子函数里写 `__builtin_assume_aligned(depth, 8)`；
  - 用 union 把 float 对拼成 64 位；
  - 只在顶层参数上写 `align_value`，而读循环在没有内联的子函数里（对齐信息不跨进子函数）。

## 两种成功写法

1. 写法一：先做一个小实验，直接把端口改成 `uint64_t*`（或 `ap_uint<64>*`），在核内用位操作拆成两个 float。
2. 写法二：顶层参数 `align_value` + 对读循环所在子函数加 `#pragma HLS INLINE`，第 7 次尝试成功。

## 坏处与新前提（采用前写进接口说明）

- 驱动必须保证基地址 8 字节对齐；
- 像素数为奇数时末拍会多读 1 个 float，缓冲区要留出这 4 字节；
- 已在 csynth 和 cosim 上验证。
