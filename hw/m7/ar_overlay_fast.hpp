// Fast CPU twin of the AR overlay core (AR2NANO2, nationals 2026-11-02).
//
// fpga_m7_ar_overlay_fast() paints the SAME picture as fpga_m7_ar_overlay() (hw/m7/ar_overlay.cpp, the
// golden reference, unchanged) byte for byte, for the Jetson Orin Nano CPU: plain arrays instead of
// 128-bit beats, row-wise painter's algorithm instead of a per-pixel 32-box compare:
//   rows y < 40   : panel only (the reference's panel branch overrides every other source there);
//   rows y >= 40  : background / depth colour (LUT) -> box lines (index 31..0, so the lowest index
//                   ends on top) -> box labels (31..0, above all lines) -> mini map (inner, ring, label),
//                   each only on the x span it can touch.
// Equivalence is proven by tests, not by construction: tools/ar_display/nano_ar_check.py
// (--impl fast; 71 fixed cases, >= 200 seeded random cases, mutant positive controls).
//
// Inputs: depth = depth_width*depth_height u16 (row-major), grid = 32768 u8, desc = 196 u32,
// regs = the 24 registers in spec order (index 15 = permit_i). fb = width*height u32 XRGB8888,
// written only when the return value is not 1 (status 1 = dimensions refused, nothing touched).
// threads <= 1: one thread; > 1: horizontal bands on std::thread (only when built with
// FPGA_AR_FAST_THREADS).
#ifndef FPGA_M7_AR_OVERLAY_FAST_HPP
#define FPGA_M7_AR_OVERLAY_FAST_HPP
#include <cstdint>

std::uint32_t fpga_m7_ar_overlay_fast(const std::uint16_t* depth, const std::uint8_t* grid,
                                     const std::uint32_t* desc, std::uint32_t* fb,
                                     const std::uint32_t regs[24], int threads);

#endif
