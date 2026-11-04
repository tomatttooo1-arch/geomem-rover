// Wrapper-vs-core equivalence bench for fpga_m7_display_axi (see tb_frame_axi).
#include <cstdio>
#include <cstring>
#include "../../display_overlay.hpp"

extern "C" std::uint32_t fpga_m7_display_axi(
    const std::uint16_t*, std::uint32_t*, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t);

static std::uint16_t depth[FPGA_M7_DISPLAY_MAX_PIXELS];
static std::uint32_t rgb_ref[FPGA_M7_DISPLAY_MAX_PIXELS];
static std::uint32_t rgb_dut[FPGA_M7_DISPLAY_MAX_PIXELS];

static int run_case(const char* name, std::uint32_t w, std::uint32_t h,
                    std::uint32_t fv, std::uint32_t pv, std::uint32_t age,
                    std::uint32_t maxage, std::uint32_t ev,
                    std::uint32_t expect_status) {
    const std::uint32_t n = w * h > 0 ? w * h : 64;
    for (std::uint32_t i = 0; i < n + 16; ++i) rgb_ref[i] = rgb_dut[i] = 0xDEADBEEFu;
    std::uint32_t status = 99;
    fpga_m7_display_overlay(depth, w, h, fv, pv, age, maxage, ev, rgb_ref, &status);
    const std::uint32_t got = fpga_m7_display_axi(depth, rgb_dut, w, h, fv, pv,
                                                 age, maxage, ev);
    const bool same = std::memcmp(rgb_ref, rgb_dut, sizeof(std::uint32_t) * (n + 16)) == 0;
    std::uint32_t coloured = 0;
    for (std::uint32_t i = 0; i < n; ++i) coloured += rgb_ref[i] != 0 && rgb_ref[i] != 0xDEADBEEFu;
    std::printf("[%s] ref status=%u nonblack=%u px0=%06x | dut status=%u equal=%d\n",
                name, status, coloured, rgb_ref[0], got, same);
    if (status != expect_status) return 1;
    if (got != status || !same) return 2;
    if (status != 1 && coloured == 0) return 3;
    return 0;
}

int main() {
    for (std::uint32_t y = 0; y < 60; ++y)
        for (std::uint32_t x = 0; x < 640; ++x)
            depth[y * 640 + x] = static_cast<std::uint16_t>(
                (x * 37 + y * 101) % 11000);  // covers invalid 0..99 and >9999
    int rc = 0;
    rc |= run_case("fresh64x40", 64, 40, 1, 1, 30, 200, 1, 0) << 0;
    rc |= run_case("stale640x24", 640, 24, 1, 1, 250, 200, 2, 0) << 4;
    rc |= run_case("fresh640x30", 640, 30, 1, 1, 12, 200, 2, 0) << 8;
    rc |= run_case("malformed", 64, 40, 1, 1, 30, 200, 3, 2) << 12;
    rc |= run_case("badsize", 0, 40, 1, 1, 30, 200, 1, 1) << 16;
    std::printf(rc ? "TB_DISPLAY_AXI FAIL rc=0x%x\n" : "TB_DISPLAY_AXI PASS\n", rc);
    return rc ? 1 : 0;
}
