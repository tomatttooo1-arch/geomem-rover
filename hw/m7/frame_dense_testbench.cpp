#include "frame_depth.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

namespace {

constexpr std::uint32_t WIDTH = 64;
constexpr std::uint32_t HEIGHT = 64;
constexpr std::uint32_t PIXELS = WIDTH * HEIGHT;
constexpr std::uint32_t AXIS = 32;
constexpr double VOXEL_M = 0.25;
constexpr std::int32_t Q8_HALF = 128;

float depth[FPGA_M7_FRAME_MAX_PIXELS] = {};
std::uint8_t flags[FPGA_M7_FRAME_GRID_CAPACITY] = {};
std::uint32_t counts[3] = {};

enum Mode { ALL_INVALID, SAME_SHORT, VARY_SHORT, SAME_LONG, VARY_LONG };

struct ProbeCase {
    const char* name;
    Mode mode;
    std::uint32_t valid;
};

const ProbeCase CASES[] = {
    {"invalid", ALL_INVALID, 0},
    {"same_short_64", SAME_SHORT, 64},
    {"same_short_512", SAME_SHORT, 512},
    {"same_short_4096", SAME_SHORT, 4096},
    {"vary_short_64", VARY_SHORT, 64},
    {"vary_short_512", VARY_SHORT, 512},
    {"vary_short_4096", VARY_SHORT, 4096},
    {"same_long_64", SAME_LONG, 64},
    {"same_long_512", SAME_LONG, 512},
    {"same_long_4096", SAME_LONG, 4096},
    {"vary_long_64", VARY_LONG, 64},
    {"vary_long_512", VARY_LONG, 512},
    {"vary_long_4096", VARY_LONG, 4096},
};

std::uint32_t offset(int x, int y, int z) {
    return static_cast<std::uint32_t>((z * AXIS + y) * AXIS + x);
}

bool varying(Mode mode) { return mode == VARY_SHORT || mode == VARY_LONG; }
bool long_ray(Mode mode) { return mode == SAME_LONG || mode == VARY_LONG; }

void expected_end(Mode mode, std::uint32_t pixel, std::int32_t end[3]) {
    const std::uint32_t x = pixel % WIDTH;
    if (mode == VARY_SHORT)
        end[0] = 64 + static_cast<std::int32_t>(128 * x);
    else if (mode == VARY_LONG)
        end[0] = 84 + static_cast<std::int32_t>(120 * x);
    else if (mode == SAME_LONG)
        end[0] = 6264;
    else
        end[0] = 24 * 256 + Q8_HALF;
    end[1] = 16 * 256 + Q8_HALF;
    end[2] = (long_ray(mode) ? 30 : 2) * 256 + Q8_HALF;
}

bool intersects_open_voxel(const std::int32_t start[3],
                           const std::int32_t end[3],
                           int x, int y, int z) {
    const int cell[3] = {x, y, z};
    double low_t = 0.0, high_t = 1.0;
    for (int axis = 0; axis < 3; ++axis) {
        const std::int32_t delta = end[axis] - start[axis];
        const std::int32_t low = cell[axis] * 256;
        const std::int32_t high = low + 256;
        if (delta == 0) {
            // CONVENTION DIFFERENCE, deliberate and currently unobservable:
            // this demands the strict interior, whereas the DUT and the
            // oracle in tests/test_m7_native.py:29-31 are half-open and give
            // a constant axis lying exactly on a voxel plane to the positive
            // side. Every coordinate this probe generates is ...*256+128, a
            // voxel centre, so no case can tell the two apart. Feeding this
            // testbench plane-aligned coordinates would make these golden
            // answers wrong rather than the DUT: widen the criterion here
            // first, never relax the DUT to match it.
            if (!(start[axis] > low && start[axis] < high)) return false;
            continue;
        }
        const double a = static_cast<double>(low - start[axis]) / delta;
        const double b = static_cast<double>(high - start[axis]) / delta;
        low_t = std::max(low_t, std::min(a, b));
        high_t = std::min(high_t, std::max(a, b));
    }
    return low_t < high_t;
}

std::uint32_t traversal_steps(const std::int32_t origin[3],
                              const std::int32_t end[3]) {
    // Diagnostic cost covariate only: this intentionally follows the DUT's
    // integer boundary-event rule. Correctness comes from the separate
    // double slab/interior oracle in intersects_open_voxel, not this counter.
    std::int32_t cell[3], target[3], direction[3], next[3], delta[3];
    for (int axis = 0; axis < 3; ++axis) {
        cell[axis] = origin[axis] / 256;
        target[axis] = end[axis] / 256;
        direction[axis] = end[axis] > origin[axis] ? 1 :
                          end[axis] < origin[axis] ? -1 : 0;
        delta[axis] = std::abs(end[axis] - origin[axis]);
        next[axis] = direction[axis] > 0
            ? (cell[axis] + 1) * 256 - origin[axis]
            : origin[axis] - cell[axis] * 256;
    }
    std::uint32_t steps = 0;
    while (cell[0] != target[0] || cell[1] != target[1] || cell[2] != target[2]) {
        int first = -1;
        for (int axis = 0; axis < 3; ++axis) {
            if (direction[axis] == 0 || cell[axis] == target[axis]) continue;
            if (first < 0 || static_cast<std::int64_t>(next[axis]) * delta[first] <
                             static_cast<std::int64_t>(next[first]) * delta[axis])
                first = axis;
        }
        if (first < 0) return 999999; // Testbench instrumentation failure.
        const std::int32_t numerator = next[first], denominator = delta[first];
        for (int axis = 0; axis < 3; ++axis) {
            if (direction[axis] == 0 || cell[axis] == target[axis]) continue;
            if (static_cast<std::int64_t>(next[axis]) * denominator ==
                static_cast<std::int64_t>(numerator) * delta[axis]) {
                cell[axis] += direction[axis];
                next[axis] += 256;
            }
        }
        ++steps;
    }
    return steps;
}

void prepare_depth(Mode mode, std::uint32_t valid, bool selected[PIXELS]) {
    const float missing = std::numeric_limits<float>::quiet_NaN();
    for (std::uint32_t i = 0; i < PIXELS; ++i) {
        depth[i] = missing;
        selected[i] = false;
    }
    if (mode == ALL_INVALID) return;
    const std::uint32_t groups = valid / WIDTH;
    for (std::uint32_t group = 0; group < groups; ++group) {
        for (std::uint32_t x = 0; x < WIDTH; ++x) {
            const std::uint32_t y = (17 * x + 7 * group) % HEIGHT;
            const std::uint32_t index = y * WIDTH + x;
            selected[index] = true;
            depth[index] = long_ray(mode) ? 7.5f : 0.5f;
        }
    }
}

void build_expected(Mode mode, const bool selected[PIXELS],
                    std::uint8_t expected[FPGA_M7_FRAME_GRID_CAPACITY],
                    std::uint32_t* unique, std::uint64_t* step_sum) {
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i) expected[i] = 0;
    *unique = 0;
    *step_sum = 0;
    if (mode == ALL_INVALID) return;
    const std::int32_t origin[3] = {16*256+Q8_HALF, 16*256+Q8_HALF, Q8_HALF};
    bool used_x[WIDTH] = {};
    for (std::uint32_t pixel = 0; pixel < PIXELS; ++pixel) {
        if (!selected[pixel]) continue;
        std::int32_t end[3];
        expected_end(mode, pixel, end);
        *step_sum += traversal_steps(origin, end);
        const std::uint32_t key = varying(mode) ? pixel % WIDTH : 0;
        used_x[key] = true;
    }
    for (std::uint32_t key = 0; key < WIDTH; ++key) {
        if (!used_x[key]) continue;
        ++*unique;
        std::int32_t end[3];
        expected_end(mode, key, end);
        const int origin_cell[3] = {16,16,0};
        const int target[3] = {end[0]/256, end[1]/256, end[2]/256};
        for (int z = 0; z < static_cast<int>(AXIS); ++z)
            for (int y = 0; y < static_cast<int>(AXIS); ++y)
                for (int x = 0; x < static_cast<int>(AXIS); ++x) {
                    if ((x == origin_cell[0] && y == origin_cell[1] && z == origin_cell[2]) ||
                        (x == target[0] && y == target[1] && z == target[2])) continue;
                    if (intersects_open_voxel(origin, end, x, y, z) &&
                        expected[offset(x,y,z)] == 0)
                        expected[offset(x,y,z)] = 1;
                }
        expected[offset(target[0],target[1],target[2])] = 2;
    }
}

int run_case(std::uint32_t index, const ProbeCase& probe) {
    bool selected[PIXELS];
    prepare_depth(probe.mode, probe.valid, selected);
    const double z = long_ray(probe.mode) ? 7.5 : 0.5;
    float inverse_k[9] = {};
    inverse_k[8] = 1.0f;
    // A tiny positive diagonal keeps every probe a nonsingular pinhole model.
    // For same-endpoint cases it is a synthetic narrow FOV, not a proposed
    // camera calibration; its sub-Q8 variation leaves the golden cells exact.
    inverse_k[0] = 0.00000095367431640625f; // 2^-20
    inverse_k[4] = 0.00000095367431640625f;
    if (probe.mode == VARY_SHORT) {
        inverse_k[0] = static_cast<float>(0.125 / z);
        inverse_k[2] = static_cast<float>(-4.0625 / z);
    } else if (probe.mode == VARY_LONG) {
        // Exact binary coefficients keep endpoints away from voxel planes:
        // q8 x = 84 + 120*u after z=7.5 and the 4.125 m translation.
        inverse_k[0] = 0.015625f;
        inverse_k[2] = -0.5390625f;
    } else if (probe.mode == SAME_LONG) {
        inverse_k[2] = 0.265625f;
    } else {
        inverse_k[2] = static_cast<float>(2.0 / z);
    }
    const float world_camera[12] = {
        1,0,0,4.125f, 0,1,0,4.125f, 0,0,1,.125f};
    const double grid_origin[3] = {0,0,0};
    std::uint32_t status = 99;
    fpga_m7_depth_frame(depth, inverse_k, world_camera, grid_origin, VOXEL_M,
                       WIDTH, HEIGHT, AXIS, AXIS, AXIS, flags, counts, &status);
    if (status || counts[0] != probe.valid || counts[1] != PIXELS-probe.valid ||
        counts[2] != 0) return 10 + index;
    std::uint8_t expected[FPGA_M7_FRAME_GRID_CAPACITY];
    std::uint32_t unique = 0;
    std::uint64_t step_sum = 0;
    build_expected(probe.mode, selected, expected, &unique, &step_sum);
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        if (flags[i] != expected[i]) return 40 + index;
    std::cout << "DENSE_CASE index=" << index << " name=" << probe.name
              << " pixels=" << PIXELS << " valid=" << probe.valid
              << " unique=" << unique << " dda_steps=" << step_sum
              << " accepted=" << counts[0] << " invalid=" << counts[1]
              << " outside=" << counts[2] << "\n";
    return 0;
}

} // namespace

int main() {
    for (std::uint32_t i = 0; i < sizeof(CASES)/sizeof(CASES[0]); ++i) {
        const int result = run_case(i, CASES[i]);
        if (result) return result;
    }
    std::cout << "M7 dense frame testbench PASS\n";
    return 0;
}
