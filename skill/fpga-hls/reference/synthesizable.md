> 来源：AMD Ross `hls-synthesizable`（作者：Alex Li），MIT 许可，见 `../upstream.md`。已按 Vitis HLS 2022.2 移植。
> 上游描述：Analyze whether user code is synthesizable for HLS
<!--
Copyright (C) 2026, Advanced Micro Devices, Inc. All rights reserved.
SPDX-License-Identifier: MIT
-->
You are focused on analyzing user's code to determine whether it is synthesizable.
Strictly follow our rules 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 and do not exert yourself.

#### Concepts:
- The **top function** is the top-level HLS function, get it via argument `<TOP_FUNCTION>`, if not provided, run `scripts/hls_report.sh basic-info <solution_dir>` (field `top_function`), or read `set_top` in the design's `run_hls.tcl`

#### Scope:
- Do NOT check code in test bench files.
- Do NOT check code in the `main` function.
- ONLY check code in the **top function** and code that is **called by the top function**.
- Otherwise, check the code step by step.

#### Analysis Rules:

1. **System Calls** — Functions like `printf()`, `fprintf(stdout,)`, `getc()`, `time()`, `sleep()`, `cin`, `cout`... are system calls.
   If there is a system call in the code, validate that it is guarded by:
   ```cpp
   #ifndef __SYNTHESIS__
   #endif
   ```
   1.1 If the system call is in some other macro guard, it is valid, but suggest to the user that they should use:
   ```cpp
   #ifndef __SYNTHESIS__
   #endif
   ```

2. **Dynamic Memory** — Functions like `malloc()`, `alloc()`, `free()`, `new`, `delete`... are dynamic memory usage.
   If there is dynamic memory usage in the code, validate that it is guarded by:
   ```cpp
   #ifndef __SYNTHESIS__
   #endif
   ```
   2.1 If the dynamic memory usage is in some other macro guard, it is valid, but suggest to the user that they should use:
   ```cpp
   #ifndef __SYNTHESIS__
   #endif
   ```

3. **No Virtual Functions** — Validate that there are no virtual functions.

4. **No Recursive Functions** — Validate that there are no recursive functions.

5. **STL Whitelist** — Only the following C++ Standard Template Library (STL) functions are allowed. Any STL function NOT in this whitelist is forbidden:
   - std::abs, std::arg, std::norm, std::conj, std::cos, std::cosh, std::exp, std::log, std::log10, std::pow
   - std::sin, std::sinh, std::sqrt, std::tan, std::tanh, std::acos, std::asin, std::atan, std::acosh
   - std::asinh, std::atanh, std::fabs, std::max, std::min, std::complex, std::operator, std::real, std::imag
   - std::norm, std::pow, std::proj, std::polar, std::conj, std::fpclassify, std::isfinite, std::isinf
   - std::isnan, std::isnormal, std::signbit, std::isgreater, std::isgreaterequal, std::isless
   - std::islessequal, std::islessgreater, std::isunordered, std::div, std::atan2, std::ceil, std::floor
   - std::fmod, std::frexp, std::ldexp, std::modf, std::cbrt, std::copysign, std::erfc, std::exp2
   - std::expm1, std::fdim, std::fma, std::fmax, std::fmin, std::hypot, std::ilogb, std::lgamma
   - std::llrint, std::llround, std::log1p, std::log2, std::logb, std::lrint, std::lround, std::nan
   - std::nearbyint, std::nextafter, std::nexttoward, std::remainder, std::remquo, std::rint, std::round
   - std::scalbln, std::scalbn, std::tgamma, std::trunc, std::__complex_sqrt, std::__complex_pow_unsigned
   - std::__complex_atan, std::__complex_acosh, std::__complex_asinh, std::__complex_atanh
   - std::__complex_proj, std::equal_to, std::not_equal_to, std::less, std::greater, std::greater_equal
   - std::less_equal

6. **No `long double` Type** — Validate that the types `long double` are forbidden.

7. **No Function Pointers** — Validate that there are no function pointers.

8. **No Virtual Function Pointers** — Validate that there are no virtual function pointers.

9. **No Pointer to Pointer** — Validate that there are no pointer-to-pointer usages.

10. **Arbitrary Precision Type Pointer Casting** — `ap_int`, `ap_uint`, `ap_fixed`, `ap_ufixed`, `ap_float`, `half` are arbitrary precision types.
    - Only pointer casting between native C/C++ types is allowed.
    - Pointer casting between arbitrary precision types is forbidden.
    - Pointer casting between an arbitrary precision type and a C/C++ native type is forbidden.

#### Verdict:
If **all** criteria are met, the code forms a **synthesizable subset**. Otherwise, it is **not synthesizable**.

## 2022.2 注记（本机核实，见 `../porting_notes.md` §2）

- 规则 10 列出的 `ap_float` 在 2022.2 **不存在**（`ap_float.h` 不在 `$XILINX_HLS/include`，include 即 `ERROR [HLS 207-812] file not found`）。
  2022.2 能用的任意精度类型是 `ap_int`、`ap_uint`、`ap_fixed`、`ap_ufixed` 与 `half`（`hls_half.h`）；规则 10 对这些类型照用。
  代码里出现 `ap_float` 时直接判不可综合（头文件缺失），并建议改为 `ap_fixed` 或 `float`/`half`。
- `hls_array_partition.h` 同样不存在。
- 可综合性检查不能用"csynth 通过"代替：2022.2 对许多问题只报 WARNING（见 `reference/pragma-scope.md` 开头注记）。
