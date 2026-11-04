#include "object_evidence.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

// This testbench is diagnostic only. It drives the unchanged production ABI.
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

// Frozen copy of the eight transactions in object_testbench.cpp at tag
// m7-object-20260921-v1. This probe does not inherit future testbench edits.
static void original_eight_transactions() {
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
}

static void check_point(std::uint32_t output_index, std::uint32_t x,
                        std::uint32_t y) {
    const double expected_x =
        (inverse_k[0] * static_cast<double>(x) +
         inverse_k[1] * static_cast<double>(y) + inverse_k[2]) * 2.;
    const double expected_y =
        (inverse_k[3] * static_cast<double>(x) +
         inverse_k[4] * static_cast<double>(y) + inverse_k[5]) * 2.;
    const double expected_z =
        (inverse_k[6] * static_cast<double>(x) +
         inverse_k[7] * static_cast<double>(y) + inverse_k[8]) * 2.;
    assert(std::fabs(xyz[3 * output_index] - expected_x) <= 1e-12);
    assert(std::fabs(xyz[3 * output_index + 1] - expected_y) <= 1e-12);
    assert(std::fabs(xyz[3 * output_index + 2] - expected_z) <= 1e-12);
}

static void dense_ninth_transaction() {
    const std::uint32_t width = 24;
    const std::uint32_t height = 24;
    const std::uint32_t pixels = width * height;
    for (std::uint32_t i = 0; i < pixels; ++i) {
        depth[i] = 2.f;
        mask[i] = 1;
    }
    call(1, roi, width, height);
    assert(status == 0);
    assert(counts[M7_INPUT_PIXELS] == pixels);
    assert(counts[M7_MASK_PIXELS] == pixels);
    assert(counts[M7_PROJECTED_VALID] == pixels);
    assert(counts[M7_CANDIDATE] == M7_SUPPORT_PRESENT);
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const std::uint32_t point = y * width + x;
            check_point(point, x, y);
        }
    }
    std::puts("M7 object port dense transaction 8 points=576 PASS");
}

static void sparse_tenth_transaction() {
    const std::uint32_t width = 24;
    const std::uint32_t height = 24;
    const std::uint32_t pixels = width * height;
    std::uint32_t selected = 0;
    for (std::uint32_t i = 0; i < pixels; ++i) {
        depth[i] = 2.f;
        mask[i] = (i % 8 == 0) ? 1 : 0;
        if (mask[i]) ++selected;
    }
    call(1, roi, width, height);
    assert(status == 0);
    assert(counts[M7_INPUT_PIXELS] == pixels);
    assert(counts[M7_MASK_PIXELS] == selected);
    assert(counts[M7_PROJECTED_VALID] == selected);
    assert(counts[M7_CANDIDATE] == M7_SUPPORT_PRESENT);
    std::uint32_t output_index = 0;
    for (std::uint32_t i = 0; i < pixels; ++i) {
        if (!mask[i]) continue;
        check_point(output_index, i % width, i / width);
        ++output_index;
    }
    assert(output_index == selected);
    std::puts("M7 object port sparse transaction 9 points=72 PASS");
}

int main() {
    original_eight_transactions();
    dense_ninth_transaction();
    sparse_tenth_transaction();
    std::puts("M7 object port probe C++ testbench PASS transactions=10");
    return 0;
}
