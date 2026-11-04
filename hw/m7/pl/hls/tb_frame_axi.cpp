// Wrapper-vs-core equivalence bench for fpga_m7_frame_axi. In C simulation both
// sides are the same C code (a wiring check only); in C/RTL co-simulation the
// wrapper side is the generated RTL with real m_axi/s_axilite adapters, so a
// match there checks the bus marshaling (base-address offsets, float64
// register halves, count/return registers) against the unchanged core.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include "../../frame_depth.hpp"

extern "C" std::uint32_t fpga_m7_frame_axi(
    const float*, std::uint8_t*, const float[9], const float[12], double,
    double, double, double, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t*, std::uint32_t*,
    std::uint32_t*);

static float depth[FPGA_M7_FRAME_MAX_PIXELS];
static std::uint8_t flags_ref[FPGA_M7_FRAME_GRID_CAPACITY];
static std::uint8_t flags_dut[FPGA_M7_FRAME_GRID_CAPACITY];

static int run_case(const char* name, std::uint32_t w, std::uint32_t h,
                    std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
                    double voxel, std::uint32_t expect_status) {
    const float ik[9] = {1.f/40, 0, -20.f/40, 0, 1.f/40, -15.f/40, 0, 0, 1};
    // Camera looks along +z from (1.6, 1.6, 0.25) inside a 3.2 m cube.
    const float pose[12] = {1,0,0,1.6f, 0,1,0,1.6f, 0,0,1,.25f};
    const double origin[3] = {0., 0., 0.};
    std::memset(flags_ref, 0x5A, sizeof flags_ref);
    std::memset(flags_dut, 0x5A, sizeof flags_dut);
    std::uint32_t counts[3] = {7, 7, 7}, status = 99;
    fpga_m7_depth_frame(depth, ik, pose, origin, voxel, w, h, nx, ny, nz,
                       flags_ref, counts, &status);
    std::uint32_t a = 7, b = 7, c = 7;
    const std::uint32_t got = fpga_m7_frame_axi(depth, flags_dut, ik, pose,
        origin[0], origin[1], origin[2], voxel, w, h, nx, ny, nz, &a, &b, &c);
    const bool same_flags = std::memcmp(flags_ref, flags_dut, sizeof flags_ref) == 0;
    std::uint32_t marked = 0;
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        marked += flags_ref[i] == 1 || flags_ref[i] == 2;
    std::printf("[%s] ref status=%u counts=%u/%u/%u marked=%u | dut status=%u "
                "counts=%u/%u/%u flags_equal=%d\n", name, status, counts[0],
                counts[1], counts[2], marked, got, a, b, c, same_flags);
    if (status != expect_status) return 1;  // known-answer on the core itself
    if (got != status || !same_flags) return 2;
    if (status == 0 && (a != counts[0] || b != counts[1] || c != counts[2])) return 3;
    if (status == 0 && marked == 0) return 4;  // bench must actually mark cells
    return 0;
}

int main() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (std::uint32_t y = 0; y < 30; ++y)
        for (std::uint32_t x = 0; x < 40; ++x) {
            float z = 1.0f + 0.05f * static_cast<float>((x * 7 + y * 3) % 23);
            if ((x + y) % 11 == 0) z = 0.f;          // missing
            if ((x * y) % 17 == 5) z = nan;          // invalid
            if (x == 39 && y % 5 == 0) z = 12.f;     // beyond range
            if (x == 0 && y % 3 == 0) z = 9.5f;      // leaves the grid
            depth[y * 40 + x] = z;
        }
    int rc = 0;
    rc |= run_case("dense40x30", 40, 30, 32, 32, 32, 0.1, 0) << 0;
    rc |= run_case("smallgrid", 40, 30, 20, 24, 28, 0.125, 0) << 4;
    rc |= run_case("baddims", 40, 30, 0, 32, 32, 0.1, 1) << 8;
    rc |= run_case("badvoxel", 40, 30, 32, 32, 32, -1.0, 2) << 12;
    std::printf(rc ? "TB_FRAME_AXI FAIL rc=0x%x\n" : "TB_FRAME_AXI PASS\n", rc);
    return rc ? 1 : 0;
}
