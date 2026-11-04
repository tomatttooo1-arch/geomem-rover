#include "object_evidence.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

static float depth[FPGA_M7_OBJECT_MAX_PIXELS];
static std::uint8_t mask[FPGA_M7_OBJECT_MAX_PIXELS];
static double xyz[3 * FPGA_M7_OBJECT_MAX_PIXELS];
static std::uint32_t counts[M7_OBJECT_COUNT_FIELDS];
static std::uint32_t status;

static const double inverse_k[9] = {
    1.0 / 12.0, 0., -1.,
    0., 1.0 / 12.0, -1.,
    0., 0., 1.};
static const double local_camera[12] = {
    1., 0., 0., 0.,
    0., 1., 0., 0.,
    0., 0., 1., 0.};
static const std::uint32_t roi[4] = {10, 10, 14, 14};
static const std::uint32_t small_roi[4] = {10, 10, 12, 14};

static void call(std::uint32_t present = 1, const std::uint32_t* rectangle = roi,
                 std::uint32_t width = 25, std::uint32_t height = 25,
                 double margin = .04) {
    fpga_m7_object_evidence(depth, mask, width, height, present, 1,
                            rectangle, 1.5, 2.5, margin, inverse_k,
                            local_camera, 9, .7, .9, xyz, counts, &status);
}

int main() {
    for (int i = 0; i < 625; ++i) {
        depth[i] = 3.f;
        mask[i] = 0;
    }
    for (int y = 10; y <= 14; ++y)
        for (int x = 10; x <= 14; ++x) {
            depth[y * 25 + x] = 2.f;
            mask[y * 25 + x] = 1;
        }
    call();
    assert(status == 0 && counts[M7_CANDIDATE] == M7_SUPPORT_PRESENT);
    assert(counts[M7_MASK_PIXELS] == 25 && counts[M7_PROJECTED_VALID] == 25);
    assert(counts[M7_ROI_PIXELS] == 25 && counts[M7_ROI_VALID] == 25);
    assert(std::fabs(xyz[3 * 12]) < 1e-12 &&
           std::fabs(xyz[3 * 12 + 1]) < 1e-12 &&
           std::fabs(xyz[3 * 12 + 2] - 2.) < 1e-12);

    for (int i = 0; i < 625; ++i) mask[i] = 0;
    for (int y = 10; y <= 14; ++y)
        for (int x = 10; x <= 14; ++x) depth[y * 25 + x] = 3.f;
    call();
    assert(status == 0 && counts[M7_CANDIDATE] == M7_FREE_SPACE_CONFLICT);
    assert(counts[M7_ROI_BEHIND] == 25 && counts[M7_PROJECTED_VALID] == 0);

    for (int y = 10; y <= 14; ++y)
        for (int x = 10; x <= 14; ++x) depth[y * 25 + x] = 1.f;
    call();
    assert(status == 0 && counts[M7_CANDIDATE] == M7_INDETERMINATE);
    assert(counts[M7_ROI_FRONT] == 25);

    for (int i = 0; i < 625; ++i) depth[i] = 3.f;
    call(0);
    assert(status == 0 && counts[M7_CANDIDATE] == M7_INDETERMINATE);
    assert(counts[M7_ROI_BEHIND] == 25);

    counts[0] = 777;
    xyz[0] = 777.;
    mask[624] = 2;
    call();
    assert(status == 2 && counts[0] == 777 && xyz[0] == 777.);
    mask[624] = 0;
    call(1, roi, 1281, 25);
    assert(status == 1 && counts[0] == 777 && xyz[0] == 777.);
    call(1, small_roi);
    assert(status == 3 && counts[0] == 777 && xyz[0] == 777.);
    call(1, roi, 25, 25, .01);
    assert(status == 4 && counts[0] == 777 && xyz[0] == 777.);
    std::puts("M7 object evidence C++ testbench PASS");
    return 0;
}
