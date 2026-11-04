// Wrapper-vs-core equivalence bench for fpga_m7_object_axi (see tb_frame_axi).
#include <cstdio>
#include <cstring>
#include "../../object_evidence.hpp"

extern "C" std::uint32_t fpga_m7_object_axi(
    const float*, const std::uint8_t*, double*, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, double, double, double, const double[9], const double[12],
    std::uint32_t, double, double, std::uint32_t*, std::uint32_t*,
    std::uint32_t*, std::uint32_t*, std::uint32_t*, std::uint32_t*,
    std::uint32_t*, std::uint32_t*);

static float depth[FPGA_M7_OBJECT_MAX_PIXELS];
static std::uint8_t mask[FPGA_M7_OBJECT_MAX_PIXELS];
static double xyz_ref[3 * FPGA_M7_OBJECT_MAX_PIXELS];
static double xyz_dut[3 * FPGA_M7_OBJECT_MAX_PIXELS];

static const double ik[9] = {1.0/12.0, 0., -1., 0., 1.0/12.0, -1., 0., 0., 1.};
static const double pose[12] = {0.8, -0.6, 0., 0.3, 0.6, 0.8, 0., -0.2, 0., 0., 1., 0.05};

static int run_case(const char* name, std::uint32_t present, std::uint32_t roi_on,
                    const std::uint32_t roi[4], std::uint32_t expect_status,
                    std::uint32_t expect_candidate) {
    const std::uint32_t w = 25, h = 25;
    std::memset(xyz_ref, 0, sizeof(double) * 3 * 625);
    std::memset(xyz_dut, 0, sizeof(double) * 3 * 625);
    std::uint32_t counts[8] = {0}, status = 99;
    fpga_m7_object_evidence(depth, mask, w, h, present, roi_on, roi, 1.5, 2.5,
                           .04, ik, pose, 9, .7, .9, xyz_ref, counts, &status);
    std::uint32_t d[8] = {0};
    const std::uint32_t got = fpga_m7_object_axi(depth, mask, xyz_dut, w, h,
        present, roi_on, roi[0], roi[1], roi[2], roi[3], 1.5, 2.5, .04, ik,
        pose, 9, .7, .9, &d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7]);
    const bool same_counts = status != 0 || std::memcmp(counts, d, sizeof d) == 0;
    const bool same_xyz = std::memcmp(xyz_ref, xyz_dut, sizeof(double) * 3 * 625) == 0;
    std::printf("[%s] ref status=%u cand=%u mask=%u valid=%u roi=%u/%u/%u/%u | "
                "dut status=%u cand=%u counts_equal=%d xyz_equal=%d\n", name,
                status, counts[7], counts[1], counts[2], counts[3], counts[4],
                counts[5], counts[6], got, d[7], same_counts, same_xyz);
    if (status != expect_status) return 1;
    if (status == 0 && counts[7] != expect_candidate) return 1;
    if (got != status || !same_counts || !same_xyz) return 2;
    return 0;
}

int main() {
    const std::uint32_t roi[4] = {10, 10, 14, 14};
    const std::uint32_t bad_roi[4] = {10, 10, 12, 14};
    for (int i = 0; i < 625; ++i) { depth[i] = 3.f; mask[i] = 0; }
    for (int y = 9; y <= 15; ++y)
        for (int x = 9; x <= 15; ++x) {
            depth[y * 25 + x] = 2.f + 0.01f * static_cast<float>(x - y);
            mask[y * 25 + x] = (x + y) % 5 != 0;  // not a solid block
        }
    int rc = 0;
    rc |= run_case("support", 1, 1, roi, 0, M7_SUPPORT_PRESENT) << 0;
    for (int i = 0; i < 625; ++i) mask[i] = 0;
    for (int y = 10; y <= 14; ++y)
        for (int x = 10; x <= 14; ++x) depth[y * 25 + x] = 3.f;
    rc |= run_case("freeconflict", 1, 1, roi, 0, M7_FREE_SPACE_CONFLICT) << 4;
    rc |= run_case("badroi", 1, 1, bad_roi, 3, 0) << 8;
    std::printf(rc ? "TB_OBJECT_AXI FAIL rc=0x%x\n" : "TB_OBJECT_AXI PASS\n", rc);
    return rc ? 1 : 0;
}
