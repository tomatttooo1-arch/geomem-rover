// Test-only adapter for hw/m7/ar_overlay_fast.cpp with the SAME C signature as ar_ref_run
// (ar_ref_adapter.cpp): fb_px_cap pixels pre-filled with 0xDEADBEEF so a write past width*height
// shows up; depth padded with zeros to depth_width*depth_height like the reference's beat vector.
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "ar_overlay_fast.hpp"

extern "C" std::uint32_t ar_fast_run(
    const std::uint16_t* depth, std::uint32_t depth_px,
    const std::uint8_t* grid, const std::uint32_t* desc,
    std::uint32_t* fb, std::uint32_t fb_px_cap, const std::uint32_t* regs) {
    const std::size_t need = static_cast<std::size_t>(regs[2] <= 1280 ? regs[2] : 0) * (regs[3] <= 800 ? regs[3] : 0);
    std::vector<std::uint16_t> d(need > depth_px ? need : depth_px, 0);
    if (depth_px) std::memcpy(d.data(), depth, depth_px * 2u);
    for (std::uint32_t i = 0; i < fb_px_cap; ++i) fb[i] = 0xDEADBEEFu;
    const char* t = std::getenv("AR_FAST_THREADS");
    return fpga_m7_ar_overlay_fast(d.data(), grid, desc, fb, regs, t ? std::atoi(t) : 1);
}
