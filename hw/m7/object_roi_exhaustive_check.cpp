// Exhaustive check of the E2 bit-pattern comparisons in fpga_m7_object_roi_batch
// (docs/design/OBJECT_ROI_SPEC_20261011.md section 11). Every one of the 2^32
// float32 bit patterns is fed as a depth pixel, in 1280x800 frames, to 32
// full-frame boxes with different near/far extents (two depth margins, so the
// front threshold is also zero or negative). The kernel's valid/front/behind
// counts must equal counts taken with the reference float32 comparisons
// (valid_depth: 0.1f < z < 10.0f; z < float(near-margin); z > float(far+margin)).
//
// Host only, not an HLS testbench. Build (OpenMP optional, ~minutes with it):
//   g++ -std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off -fopenmp
//       hw/m7/object_evidence.cpp hw/m7/object_roi_exhaustive_check.cpp -o roi_exhaustive
#include "object_evidence.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

namespace {
const std::uint32_t W = FPGA_M7_OBJECT_MAX_WIDTH, H = FPGA_M7_OBJECT_MAX_HEIGHT;
const std::uint64_t FRAME = static_cast<std::uint64_t>(W) * H;
const std::uint32_t N = FPGA_M7_ROI_MAX_BOXES;

struct Config {
    double margin;
    double near_z[FPGA_M7_ROI_MAX_BOXES];
    double far_z[FPGA_M7_ROI_MAX_BOXES];
};

void make_config(Config& c, double margin, const double* nears, const double* fars) {
    c.margin = margin;
    for (std::uint32_t b = 0; b < N; ++b) {
        c.near_z[b] = nears[b];
        c.far_z[b] = fars[b] < nears[b] ? nears[b] : fars[b];
    }
}
}

int main() {
    // Near/far extents chosen so that float(near-margin) and float(far+margin)
    // land on and around the validity edges 0.1f and 10.0f, inside the window,
    // above it, and (margin .5) at +0 and below zero.
    static const double nears_a[32] = {
        0.1000000001, 0.10000001, 0.14, 0.14000001, 0.1400000001, 0.2, 0.5, 0.54,
        1.0, 1.86, 1.9, 2.0, 3.3, 5.0, 7.77, 9.99, 9.999999, 10.03, 10.04,
        10.0400001, 10.05, 11.0, 100.0, 1e10, 1e30, 0.33, 0.1000000000001, 4.04,
        6.0, 8.5, 0.7, 2.5};
    static const double fars_a[32] = {
        0.1000000001, 0.2, 0.14, 9.96, 9.9599999, 9.96000001, 0.5, 2.1, 2.14, 10.0,
        1.9, 2.0, 3.3, 5.0, 7.77, 9.99, 9.999999, 10.03, 10.04, 10.0400001, 10.05,
        11.0, 100.0, 1e10, 1e30, 0.33, 0.06, 4.04, 6.0, 8.5, 0.7, 2.5};
    static const double nears_b[32] = {
        0.11, 0.3, 0.5, 0.5000000001, 0.4999999999, 0.6, 0.6000000001, 0.59999999,
        0.75, 1.0, 2.0, 5.0, 9.5, 10.5, 10.6, 12.0, 0.12, 0.25, 0.45, 0.55,
        0.65, 0.85, 1.5, 3.0, 4.0, 6.0, 7.0, 8.0, 9.0, 9.4999999, 0.105, 0.2};
    static const double fars_b[32] = {
        0.11, 9.5, 0.5, 9.4999999, 9.5000001, 0.6, 9.6, 0.59999999, 0.75, 1.0,
        2.0, 5.0, 9.5, 10.5, 10.6, 12.0, 0.12, 0.25, 0.45, 0.55, 0.65, 0.85, 1.5,
        3.0, 4.0, 6.0, 7.0, 8.0, 9.0, 9.4999999, 0.105, 0.2};
    Config configs[2];
    make_config(configs[0], .04, nears_a, fars_a);
    make_config(configs[1], .5, nears_b, fars_b);

    std::uint32_t enabled[N], xyxy[4 * N], ids[N];
    for (std::uint32_t b = 0; b < N; ++b) {
        enabled[b] = 1;
        xyxy[4 * b] = 0; xyxy[4 * b + 1] = 0;
        xyxy[4 * b + 2] = W - 1; xyxy[4 * b + 3] = H - 1;
        ids[b] = b;
    }
    const std::uint64_t chunks = ((1ull << 32) + FRAME - 1) / FRAME;
    long long failures = 0;
    for (int ci = 0; ci < 2; ++ci) {
        const Config& c = configs[ci];
        float front[N], back[N];
        for (std::uint32_t b = 0; b < N; ++b) {
            front[b] = static_cast<float>(c.near_z[b] - c.margin);
            back[b] = static_cast<float>(c.far_z[b] + c.margin);
        }
        unsigned long long total_valid = 0, total_ahead = 0, total_behind = 0;
        long long bad = 0;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 4) reduction(+ : bad, total_valid, total_ahead, total_behind)
#endif
        for (long long k = 0; k < static_cast<long long>(chunks); ++k) {
            std::vector<float> depth(FRAME);
            for (std::uint64_t i = 0; i < FRAME; ++i) {
                const std::uint64_t pattern = static_cast<std::uint64_t>(k) * FRAME + i;
                // Past 2^32 the last frame repeats pattern 0; both sides see it.
                const std::uint32_t bits = pattern < (1ull << 32) ? static_cast<std::uint32_t>(pattern) : 0u;
                std::memcpy(&depth[i], &bits, sizeof bits);
            }
            std::uint32_t result[M7_RB_FIELDS * N];
            std::uint32_t status = 99;
            fpga_m7_object_roi_batch(depth.data(), W, H, N, enabled, xyxy, c.near_z, c.far_z,
                                    ids, c.margin, 9, .7, .9, result, &status);
            if (status != 0) { ++bad; continue; }
            std::uint32_t valid = 0, ahead[N] = {0}, behind[N] = {0};
            for (std::uint64_t i = 0; i < FRAME; ++i) {
                const float z = depth[i];
                if (!(z > 0.1f && z < 10.0f)) continue;
                ++valid;
                for (std::uint32_t b = 0; b < N; ++b) {
                    if (z < front[b]) ++ahead[b];
                    if (z > back[b]) ++behind[b];
                }
            }
            for (std::uint32_t b = 0; b < N; ++b) {
                const std::uint32_t* r = result + M7_RB_FIELDS * b;
                if (r[M7_RB_PIXELS] != FRAME || r[M7_RB_VALID] != valid ||
                    r[M7_RB_FRONT] != ahead[b] || r[M7_RB_BEHIND] != behind[b]) ++bad;
                total_ahead += ahead[b];
                total_behind += behind[b];
            }
            total_valid += valid;
        }
        std::printf("margin %.2f: %llu frames, valid patterns %llu, sum front %llu, "
                    "sum behind %llu, mismatching box-frames %lld\n", c.margin,
                    static_cast<unsigned long long>(chunks), total_valid, total_ahead,
                    total_behind, bad);
        failures += bad;
    }
    // Known answer: the number of float32 patterns strictly inside (0.1f, 10.0f).
    std::uint32_t lo, hi;
    const float lo_f = 0.1f, hi_f = 10.0f;
    std::memcpy(&lo, &lo_f, 4);
    std::memcpy(&hi, &hi_f, 4);
    std::printf("expected valid patterns per margin: %u\n", hi - lo - 1);
    std::printf(failures ? "object ROI exhaustive check FAILED\n"
                         : "object ROI exhaustive check passed\n");
    return failures ? 1 : 0;
}
