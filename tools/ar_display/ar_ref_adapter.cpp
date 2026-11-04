// Test-only adapter: plain arrays <-> 128-bit beats around fpga_m7_ar_overlay.
// Packs with ap_uint::range() (never relies on ap_uint memory layout).
// fb_px_cap pixels are pre-filled with sentinel 0xDEADBEEF; the caller makes
// fb_px_cap = width*height + 16 so 4 extra beats detect overruns.
#include <cstdint>
#include <vector>
#include "ar_overlay.hpp"

extern "C" std::uint32_t ar_ref_run(
    const std::uint16_t* depth, std::uint32_t depth_px,
    const std::uint8_t* grid,          // 32768 bytes
    const std::uint32_t* desc,         // 196 u32
    std::uint32_t* fb, std::uint32_t fb_px_cap,
    const std::uint32_t* regs) {       // 24 registers, spec order
    std::vector<fpga_m7_ar_word> d((depth_px + 7) / 8 + 1, 0);
    for (std::uint32_t i = 0; i < depth_px; ++i)
        d[i / 8].range(16 * (i % 8) + 15, 16 * (i % 8)) = depth[i];
    std::vector<fpga_m7_ar_word> g(FPGA_M7_AR_GRID_WORDS, 0);
    for (std::uint32_t i = 0; i < 32768u; ++i)
        g[i / 16].range(8 * (i % 16) + 7, 8 * (i % 16)) = grid[i];
    std::vector<fpga_m7_ar_word> b(FPGA_M7_AR_DESC_WORDS, 0);
    for (std::uint32_t i = 0; i < FPGA_M7_AR_DESC_U32; ++i)
        b[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)) = desc[i];
    std::vector<fpga_m7_ar_word> f((fb_px_cap + 3) / 4, 0);
    for (std::uint32_t i = 0; i < f.size() * 4; ++i)
        f[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)) = 0xDEADBEEFu;
    const std::uint32_t* r = regs;
    const std::uint32_t st = fpga_m7_ar_overlay(
        d.data(), g.data(), b.data(), f.data(),
        r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10],
        r[11], r[12], r[13], r[14], r[15], r[16], r[17],
        static_cast<std::int32_t>(r[18]), static_cast<std::int32_t>(r[19]),
        r[20], r[21], r[22], r[23]);
    for (std::uint32_t i = 0; i < fb_px_cap; ++i)
        fb[i] = f[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)).to_uint();
    return st;
}
