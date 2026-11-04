// Multi-box co-simulation gate for fpga_m7_ar_overlay_axi (D, 2026-10-11).
// The default testbench (tb_ar_overlay_axi.cpp) co-simulates one 64x48 frame
// with a single box, which cannot see a box-index or priority error that only
// exists in the RTL. This testbench reads ar_mb_cases.bin (written by
// tools/ar_display/multibox_cosim.py from the numpy golden, never from the C
// core) and compares every pixel of every case.
//
// ar_mb_cases.bin, little endian u32 unless noted:
//   magic 0x424D5241 "ARMB", n_cases
//   per case: regs[24], desc[196], expect_status, depth_n, depth[depth_n] (u16,
//   padded to even count), grid[32768] (u8), fb[width*height] (expected).
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "../../ar_overlay.hpp"

extern "C" std::uint32_t fpga_m7_ar_overlay_axi(
    const fpga_m7_ar_word*, const fpga_m7_ar_word*, const fpga_m7_ar_word*,
    fpga_m7_ar_word*, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, bool, std::uint32_t, std::uint32_t,
    std::int32_t, std::int32_t, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t);

static fpga_m7_ar_word g_depth[FPGA_M7_AR_DEPTH_WORDS_MAX];
static fpga_m7_ar_word g_grid[FPGA_M7_AR_GRID_WORDS];
static fpga_m7_ar_word g_boxes[FPGA_M7_AR_DESC_WORDS];
static fpga_m7_ar_word g_fb[FPGA_M7_AR_FB_WORDS_MAX];

static FILE* g_in = nullptr;
static bool g_eof = false;

static std::uint32_t rd32() {
    unsigned char b[4];
    if (std::fread(b, 1, 4, g_in) != 4) { g_eof = true; return 0; }
    return b[0] | (b[1] << 8) | (b[2] << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
}

static std::uint32_t px(std::uint32_t i) {
    return g_fb[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)).to_uint();
}

static int run_case(std::uint32_t idx) {
    std::uint32_t r[24], desc[FPGA_M7_AR_DESC_U32];
    for (int k = 0; k < 24; ++k) r[k] = rd32();
    for (std::uint32_t k = 0; k < FPGA_M7_AR_DESC_U32; ++k) desc[k] = rd32();
    const std::uint32_t expect_status = rd32();
    const std::uint32_t depth_n = rd32();
    if (g_eof || depth_n > 8 * FPGA_M7_AR_DEPTH_WORDS_MAX) return -1;
    for (std::uint32_t i = 0; i < FPGA_M7_AR_DEPTH_WORDS_MAX; ++i) g_depth[i] = 0;
    for (std::uint32_t i = 0; i < depth_n; i += 2) {
        const std::uint32_t v = rd32();
        g_depth[i / 8].range(16 * (i % 8) + 15, 16 * (i % 8)) = v & 0xFFFF;
        if (i + 1 < depth_n)
            g_depth[(i + 1) / 8].range(16 * ((i + 1) % 8) + 15, 16 * ((i + 1) % 8)) = v >> 16;
    }
    for (std::uint32_t i = 0; i < FPGA_M7_AR_GRID_WORDS; ++i)
        for (int j = 0; j < 4; ++j) g_grid[i].range(32 * j + 31, 32 * j) = rd32();
    for (std::uint32_t i = 0; i < FPGA_M7_AR_DESC_U32; ++i)
        g_boxes[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)) = desc[i];
    for (std::uint32_t i = 0; i < FPGA_M7_AR_FB_WORDS_MAX; ++i)
        for (int j = 0; j < 4; ++j) g_fb[i].range(32 * j + 31, 32 * j) = 0xDEADBEEFu;
    const std::uint32_t W = r[0], H = r[1], npx = W * H;
    std::vector<std::uint32_t> exp(npx);
    for (std::uint32_t i = 0; i < npx; ++i) exp[i] = rd32();
    if (g_eof || npx > 4 * FPGA_M7_AR_FB_WORDS_MAX - 16) return -1;

    const std::uint32_t st = fpga_m7_ar_overlay_axi(
        g_depth, g_grid, g_boxes, g_fb, r[0], r[1], r[2], r[3], r[4], r[5],
        r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14], r[15] != 0,
        r[16], r[17], static_cast<std::int32_t>(r[18]),
        static_cast<std::int32_t>(r[19]), r[20], r[21], r[22], r[23]);

    std::uint32_t ndiff = 0, overrun = 0;
    for (std::uint32_t i = 0; i < npx; ++i) {
        const std::uint32_t got = px(i);
        if (got != exp[i]) {
            if (ndiff < 6)
                std::printf("  case %u diff at (x=%u,y=%u): got %06x exp %06x\n",
                            idx, i % W, i / W, got, exp[i]);
            ++ndiff;
        }
    }
    for (std::uint32_t i = npx; i < npx + 16; ++i) overrun += px(i) != 0xDEADBEEFu;
    const bool ok = st == expect_status && ndiff == 0 && overrun == 0;
    std::printf("[mb%u %ux%u] status=%u (exp %u) diff_px=%u/%u overrun=%u %s\n",
                idx, W, H, st, expect_status, ndiff, npx, overrun, ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

int main() {
    g_in = std::fopen("ar_mb_cases.bin", "rb");
    if (!g_in) { std::printf("AR_MB FAIL cannot open ar_mb_cases.bin\n"); return 2; }
    const std::uint32_t magic = rd32(), n = rd32();
    if (magic != 0x424D5241u || n == 0 || n > 16) {
        std::printf("AR_MB FAIL bad header\n");
        return 2;
    }
    int fails = 0;
    for (std::uint32_t i = 0; i < n; ++i) {
        const int rc = run_case(i);
        if (rc < 0) { std::printf("AR_MB FAIL truncated input at case %u\n", i); return 2; }
        fails += rc;
    }
    std::fclose(g_in);
    std::printf("AR_MB %s cases=%u fails=%d\n", fails ? "FAIL" : "PASS", n, fails);
    return fails ? 1 : 0;
}
