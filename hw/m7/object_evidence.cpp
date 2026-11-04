#include "object_evidence.hpp"

#include <cfloat>

namespace {
bool finite_double(double value) {
    return value >= -DBL_MAX && value <= DBL_MAX;
}

bool finite_float(float value) {
    return value >= -FLT_MAX && value <= FLT_MAX;
}

bool valid_depth(float z) {
    // NumPy compares a float32 depth array against these Python scalar
    // constants in float32 for the supported image dtype (NumPy 1.24).
    return z > 0.1f && z < 10.0f;
}
}

extern "C" void fpga_m7_object_evidence(
    const float depth[FPGA_M7_OBJECT_MAX_PIXELS],
    const std::uint8_t mask[FPGA_M7_OBJECT_MAX_PIXELS],
    std::uint32_t width, std::uint32_t height,
    std::uint32_t mask_present, std::uint32_t roi_enabled,
    const std::uint32_t roi_xyxy[4],
    double near_z, double far_z, double depth_margin_m,
    const double inverse_k[9], const double local_camera[12],
    std::uint32_t min_pixels, double min_valid_fraction,
    double free_fraction,
    double xyz_local[3 * FPGA_M7_OBJECT_MAX_PIXELS],
    std::uint32_t counts[M7_OBJECT_COUNT_FIELDS],
    std::uint32_t* status) {
    // All input validation precedes any output writes. In particular, a
    // malformed mask near the end of the image cannot leave partial points.
    *status = 1;
    if (width == 0 || height == 0 || width > FPGA_M7_OBJECT_MAX_WIDTH ||
        height > FPGA_M7_OBJECT_MAX_HEIGHT) return;
    const std::uint32_t pixels = width * height;
    *status = 2;
    if (mask_present > 1 || roi_enabled > 1) return;
    *status = 3;
    if (roi_enabled) {
        if (roi_xyxy[0] > roi_xyxy[2] || roi_xyxy[1] > roi_xyxy[3] ||
            roi_xyxy[2] >= width || roi_xyxy[3] >= height ||
            roi_xyxy[2] - roi_xyxy[0] < 3 ||
            roi_xyxy[3] - roi_xyxy[1] < 3 ||
            !finite_double(near_z) || !finite_double(far_z) ||
            near_z <= 0.1 || far_z < near_z) return;
    }
    *status = 4;
    if (min_pixels < 9 || min_pixels > FPGA_M7_OBJECT_MAX_PIXELS ||
        !finite_double(min_valid_fraction) ||
        min_valid_fraction < 0.7 || min_valid_fraction > 1.0 ||
        !finite_double(free_fraction) ||
        free_fraction < 0.9 || free_fraction > 1.0 ||
        !finite_double(depth_margin_m) || depth_margin_m < 0.04) return;
    for (std::uint32_t i = 0; i < 9; ++i)
        if (!finite_double(inverse_k[i])) return;
    for (std::uint32_t i = 0; i < 12; ++i)
        if (!finite_double(local_camera[i])) return;
    const float front = static_cast<float>(near_z - depth_margin_m);
    const float back = static_cast<float>(far_z + depth_margin_m);
    if (roi_enabled && (!finite_float(front) || !finite_float(back) || front > back)) return;

    if (mask_present) {
        *status = 2;
        for (std::uint32_t i = 0; i < pixels; ++i)
            if (mask[i] > 1) return;
    }

    std::uint32_t mask_pixels = 0;
    std::uint32_t projected_valid = 0;
    std::uint32_t roi_pixels = 0;
    std::uint32_t roi_valid = 0;
    std::uint32_t roi_front = 0;
    std::uint32_t roi_behind = 0;
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const std::uint32_t index = y * width + x;
            const float z = depth[index];
            const bool depth_valid = valid_depth(z);
            if (roi_enabled && x >= roi_xyxy[0] && x <= roi_xyxy[2] &&
                y >= roi_xyxy[1] && y <= roi_xyxy[3]) {
                ++roi_pixels;
                if (depth_valid) {
                    ++roi_valid;
                    if (z < front) ++roi_front;
                    if (z > back) ++roi_behind;
                }
            }
            if (!mask_present || mask[index] == 0) continue;
            ++mask_pixels;
            if (!depth_valid) continue;
            const double fx = static_cast<double>(x);
            const double fy = static_cast<double>(y);
            const double fz = static_cast<double>(z);
            double camera[3];
            double point[3];
            for (std::uint32_t row = 0; row < 3; ++row) {
                camera[row] = (inverse_k[3 * row] * fx +
                               inverse_k[3 * row + 1] * fy +
                               inverse_k[3 * row + 2]) * fz;
            }
            bool point_valid = true;
            for (std::uint32_t row = 0; row < 3; ++row) {
                point[row] = local_camera[4 * row] * camera[0] +
                             local_camera[4 * row + 1] * camera[1] +
                             local_camera[4 * row + 2] * camera[2] +
                             local_camera[4 * row + 3];
                point_valid = point_valid && finite_double(point[row]);
            }
            if (!point_valid) continue;
            for (std::uint32_t row = 0; row < 3; ++row)
                xyz_local[3 * projected_valid + row] = point[row];
            ++projected_valid;
        }
    }

    std::uint32_t candidate = M7_INDETERMINATE;
    if (mask_present) {
        if (mask_pixels > 0) {
            if (projected_valid >= min_pixels &&
                static_cast<double>(projected_valid) / mask_pixels >= min_valid_fraction)
                candidate = M7_SUPPORT_PRESENT;
        } else if (roi_enabled && roi_valid >= min_pixels &&
                   static_cast<double>(roi_valid) / roi_pixels >= min_valid_fraction) {
            // Front occlusion wins over a free-space claim.
            if (static_cast<double>(roi_front) / roi_valid >= 0.5) {
                candidate = M7_INDETERMINATE;
            } else if (static_cast<double>(roi_behind) / roi_valid >= free_fraction) {
                candidate = M7_FREE_SPACE_CONFLICT;
            }
        }
    }
    counts[M7_INPUT_PIXELS] = pixels;
    counts[M7_MASK_PIXELS] = mask_pixels;
    counts[M7_PROJECTED_VALID] = projected_valid;
    counts[M7_ROI_PIXELS] = roi_pixels;
    counts[M7_ROI_VALID] = roi_valid;
    counts[M7_ROI_FRONT] = roi_front;
    counts[M7_ROI_BEHIND] = roi_behind;
    counts[M7_CANDIDATE] = candidate;
    *status = 0;
}

// ---------------------------------------------------------------------------
// Mask-free multi-box region check. Appended entry; the function above is
// unchanged. Test-only fault macros FPGA_M7_ROI_MUTANT_* touch only this one.
//
// E2 structure (docs/design/OBJECT_ROI_SPEC_20261011.md section 11). Results
// are bit-identical to E1 (23cbbff); the per-pixel predicates are the same,
// only re-expressed for hardware:
//  - Box membership: bit b of col_cover[x] is set iff enabled box b has
//    x0<=x<=x1, likewise row_cover[y]; a pixel is inside box b iff bit b is
//    set in col_cover[x] & row_cover[y]. Both tables are built once per call
//    as XOR difference arrays (toggle at x0 and x1+1) and a prefix XOR, so the
//    pixel loop needs no coordinate comparators.
//  - valid_depth(z) and z<front / z>back are evaluated on IEEE-754 bit
//    patterns. For non-negative floats (sign bit clear, NaN excluded) the
//    unsigned order of the bit pattern equals the numeric order. NaN or
//    negative z has a bit pattern above bits(10.0f), so it fails the window
//    LO < bits(z) < HI (LO = bits(0.1f), HI = bits(10.0f)) exactly like
//    valid_depth. A valid z therefore has offset o(z) = bits(z) - LO in
//    [1, HI-LO-1] (26 bits). Each threshold t becomes key(t) = clamp(bits(t),
//    LO, HI) - LO, with negative t (sign bit set) mapped to 0. Then for valid z:
//    z<t <=> o(z) < key(t) and z>t <=> o(z) > key(t): if t <= 0.1f (or is
//    negative) z<t is never true and z>t always true, matching key 0 against
//    o(z) >= 1; if t >= 10.0f the reverse, matching key HI-LO > o(z);
//    otherwise the offset is exact. hw/m7/object_roi_exhaustive_check.cpp
//    checks this on all 2^32 depth bit patterns.
//  - One flat pipelined pass over the frame (II=1), depth read strictly in
//    address order; every box is a lane with private counters, kept to 20 bits
//    (a box holds at most 1280*800 < 2^20 pixels).
//  - The per-box validation and the float32 thresholds are the E1
//    expressions. The float64 verdict divisions are exact integer tests
//    (fpga_roi_ratio_ge, 2026-11-02), bit-identical to the E1 divisions.
// Pragmas are guarded so the host g++ -Wall -Werror build never sees them.
#include <cstring>

namespace {
std::uint32_t fpga_roi_float_bits(float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof bits);
    return bits;
}

const std::uint32_t FPGA_ROI_SIGN = 0x80000000u;
const std::uint32_t FPGA_ROI_VALID_LO = 0x3DCCCCCDu;  // bit pattern of 0.1f
const std::uint32_t FPGA_ROI_VALID_HI = 0x41200000u;  // bit pattern of 10.0f
const std::uint32_t FPGA_ROI_COUNT_MASK = 0xFFFFFu;   // 20 bits >= 1280*800
const std::uint32_t FPGA_ROI_KEY_MASK = 0x3FFFFFFu;   // 26 bits >= HI - LO

std::uint32_t fpga_roi_key(float threshold) {
    const std::uint32_t bits = fpga_roi_float_bits(threshold);
    if ((bits & FPGA_ROI_SIGN) || bits <= FPGA_ROI_VALID_LO) return 0u;
    if (bits >= FPGA_ROI_VALID_HI) return FPGA_ROI_VALID_HI - FPGA_ROI_VALID_LO;
    return bits - FPGA_ROI_VALID_LO;
}

std::uint64_t fpga_roi_double_bits(double value) {
    std::uint64_t bits;
    std::memcpy(&bits, &value, sizeof bits);
    return bits;
}

// Range checks on IEEE-754 bit patterns (2026-11-02, SLIM_ROI1 step 3).
// For sign-clear, non-NaN values the unsigned order of the bit pattern is the
// numeric order. A set sign bit (negative or -0) puts the pattern at >= 2^63
// and a NaN or infinity at >= bits(+inf), so "lo <= bits(x) <= hi" with
// finite positive lo, hi is exactly "finite(x) && lo <= x <= hi", and
// "lo < bits(x) < bits(+inf)" is exactly "finite(x) && x > lo".
const std::uint64_t FPGA_ROI_D_0_04 = 0x3FA47AE147AE147Bull;  // 0.04
const std::uint64_t FPGA_ROI_D_0_1 = 0x3FB999999999999Aull;   // 0.1
#ifdef FPGA_M7_ROI_MUTANT_BOUND_ULP
const std::uint64_t FPGA_ROI_D_0_7 = 0x3FE6666666666665ull;   // Fault: one ulp below 0.7.
#else
const std::uint64_t FPGA_ROI_D_0_7 = 0x3FE6666666666666ull;   // 0.7
#endif
const std::uint64_t FPGA_ROI_D_0_9 = 0x3FECCCCCCCCCCCCDull;   // 0.9
const std::uint64_t FPGA_ROI_D_1_0 = 0x3FF0000000000000ull;   // 1.0
const std::uint64_t FPGA_ROI_D_INF = 0x7FF0000000000000ull;   // +inf
const std::uint32_t FPGA_ROI_F_EXP = 0x7F800000u;             // float exponent field

// Verdict ratios without division (2026-11-02, SLIM_ROI1 step 1). The E1
// test static_cast<double>(a) / b >= f uses IEEE round-to-nearest-even
// division. For f in [0.5, 1] (all three thresholds once validated) and
// 0 <= a, b < 2^20 it equals an exact integer test against the midpoint
// between f and the next double below it, mid = M * 2^-55:
//   a/b rounds to >= f  <=>  a*2^55 > b*M, or a*2^55 == b*M and f's
//   significand is even (a tie rounds to the even neighbour).
// A plain a >= f*b (no rounding) is wrong: ratios within half an ulp below f
// round up to f (SLIM_ROI audit, 314,572 mismatches on 1.2e8 boundary pairs).
// f = (2^52 + m) * 2^-53 for f in [0.5, 1) gives M = 2^54 + 4m - 2; the
// binade edges are f = 0.5 (M = 2^54 - 1) and f = 1.0 (M = 2^55 - 2).
// b*M (< 2^76) is formed as two 48-bit partial products, M = mh*2^28 + ml,
// by a serial shift-add over the 20 bits of b: one adder pair, no multiplier.
const std::uint32_t FPGA_ROI_RATIO_BITS = 20;     // a, b < 2^20 (counts)
const std::uint64_t FPGA_ROI_LO28 = (1ull << 28) - 1;

struct FpgaRoiRatio {
    std::uint32_t mh, ml;  // M = mh * 2^28 + ml
    bool tie;              // result when a*2^55 == b*M exactly
};

FpgaRoiRatio fpga_roi_ratio_bound(double f) {
    const std::uint64_t bits = fpga_roi_double_bits(f);
    const std::uint64_t m = bits & ((1ull << 52) - 1);
    std::uint64_t mid;
#ifdef FPGA_M7_ROI_MUTANT_RATIO_NAIVE
    // Fault: compare against f itself (exact rational a/b >= f, no rounding).
    mid = (bits >> 52) == 0x3FFu ? (1ull << 55) : (1ull << 54) + 4 * m;
    FpgaRoiRatio t;
    t.mh = static_cast<std::uint32_t>(mid >> 28);
    t.ml = static_cast<std::uint32_t>(mid & FPGA_ROI_LO28);
    t.tie = true;
    return t;
#else
    if ((bits >> 52) == 0x3FFu) mid = (1ull << 55) - 2;  // f == 1.0
    else if (m == 0) mid = (1ull << 54) - 1;            // f == 0.5
    else mid = (1ull << 54) + 4 * m - 2;
#ifdef FPGA_M7_ROI_MUTANT_RATIO_MID_PLUS1
    mid += 1;  // Fault: boundary one unit (2^-55) too high.
#endif
    FpgaRoiRatio t;
    t.mh = static_cast<std::uint32_t>(mid >> 28);
    t.ml = static_cast<std::uint32_t>(mid & FPGA_ROI_LO28);
    t.tie = (m & 1u) == 0;
    return t;
#endif
}

// == (static_cast<double>(a) / b >= f) for f in [0.5, 1], a, b < 2^20.
bool fpga_roi_ratio_ge(std::uint32_t a, std::uint32_t b, const FpgaRoiRatio& t) {
    std::uint64_t p = 0, q = 0;  // b*mh, b*ml (< 2^48 each)
    for (std::uint32_t i = 0; i < FPGA_ROI_RATIO_BITS; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II=1
#endif
        const bool bit = (b >> (FPGA_ROI_RATIO_BITS - 1 - i)) & 1u;
        p = (p << 1) + (bit ? t.mh : 0u);
        q = (q << 1) + (bit ? t.ml : 0u);
    }
    // b*M = h*2^28 + (q mod 2^28); a*2^55 = (a*2^27)*2^28.
    const std::uint64_t h = p + (q >> 28);
    const std::uint64_t lhs = static_cast<std::uint64_t>(a) << 27;
    return lhs > h || (lhs == h && (q & FPGA_ROI_LO28) == 0 && t.tie);
}
}

extern "C" void fpga_m7_object_roi_batch(
    const float depth[FPGA_M7_OBJECT_MAX_PIXELS],
    std::uint32_t width, std::uint32_t height,
    std::uint32_t box_count,
    const std::uint32_t box_enabled[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t box_xyxy[4 * FPGA_M7_ROI_MAX_BOXES],
    const double near_z[FPGA_M7_ROI_MAX_BOXES],
    const double far_z[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t object_id[FPGA_M7_ROI_MAX_BOXES],
    double depth_margin_m, std::uint32_t min_pixels,
    double min_valid_fraction, double free_fraction,
    std::uint32_t result[M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES],
    std::uint32_t* status) {
    *status = 1;
    if (width == 0 || height == 0 || width > FPGA_M7_OBJECT_MAX_WIDTH ||
        height > FPGA_M7_OBJECT_MAX_HEIGHT) return;
    *status = 4;
    // == !finite || out of range, on bit patterns (see FPGA_ROI_D_*).
    const std::uint64_t valid_fraction_bits = fpga_roi_double_bits(min_valid_fraction);
    const std::uint64_t free_fraction_bits = fpga_roi_double_bits(free_fraction);
    const std::uint64_t margin_bits = fpga_roi_double_bits(depth_margin_m);
    if (min_pixels < 9 || min_pixels > FPGA_M7_OBJECT_MAX_PIXELS ||
        valid_fraction_bits < FPGA_ROI_D_0_7 || valid_fraction_bits > FPGA_ROI_D_1_0 ||
        free_fraction_bits < FPGA_ROI_D_0_9 || free_fraction_bits > FPGA_ROI_D_1_0 ||
        margin_bits < FPGA_ROI_D_0_04 || margin_bits >= FPGA_ROI_D_INF) return;
    *status = 3;
    if (box_count == 0 || box_count > FPGA_M7_ROI_MAX_BOXES) return;

    std::uint32_t col_cover[FPGA_M7_OBJECT_MAX_WIDTH + 1];
    std::uint32_t row_cover[FPGA_M7_OBJECT_MAX_HEIGHT + 1];
    const std::uint32_t span = width > height ? width : height;
    for (std::uint32_t i = 0; i <= span; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=2 avg=641 max=1281
#endif
        if (i <= width) col_cover[i] = 0;
        if (i <= height) row_cover[i] = 0;
    }

    // Lane thresholds. The validation loop (early exits, unchanged order)
    // only stores box b's two keys at index b of a small unpartitioned table;
    // a separate loop without early exits then copies the table into the
    // per-lane registers, one constant index per lane. (The earlier shift
    // chain inside the validation loop simulated correctly in C but in RTL
    // left lane b holding box b-1's keys; see
    // docs/design/OBJECT_ROI_E2_HANDOVER_20261011.md.)
    // One double adder serves both lane thresholds (2026-11-02, SLIM_ROI1
    // step 2): near - margin is computed as near + (-margin), which IEEE-754
    // defines as the same operation (same rounding, same signed zero), so
    // the E1 front/back values are unchanged bit for bit.
#ifdef __SYNTHESIS__
#pragma HLS ALLOCATION operation instances=dadd limit=1
#endif
#ifdef FPGA_M7_ROI_MUTANT_MARGIN_SIGN
    const double neg_margin = depth_margin_m;  // Fault: front uses near + margin.
#else
    // -x is a sign-bit flip; written on the bit pattern so HLS cannot turn
    // it into a second (0 - x) subtractor.
    double neg_margin;
    {
        const std::uint64_t flipped = fpga_roi_double_bits(depth_margin_m) ^ (1ull << 63);
        std::memcpy(&neg_margin, &flipped, sizeof neg_margin);
    }
#endif
    std::uint32_t active_mask = 0;
    std::uint32_t key_table_front[FPGA_M7_ROI_MAX_BOXES];
    std::uint32_t key_table_back[FPGA_M7_ROI_MAX_BOXES];
    // Box pixel count from the area (2026-11-02, SLIM_ROI1 step 4). A
    // validated box lies inside the frame and every one of its
    // (x1-x0+1)*(y1-y0+1) pixels is visited once by the pass, so the area
    // equals the former per-pixel count (<= 1280*800 < 2^20, no wrap).
    std::uint32_t area_table[FPGA_M7_ROI_MAX_BOXES];
    for (std::uint32_t b = 0; b < FPGA_M7_ROI_MAX_BOXES; ++b) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        std::uint32_t lane_front = 0, lane_back = 0, lane_area = 0;
        if (b < box_count) {
            if (box_enabled[b] > 1) return;
            if (box_enabled[b]) {
                const std::uint32_t* r = box_xyxy + 4 * b;
                // == !finite(near) || !finite(far) || near <= 0.1 || far < near:
                // near passes only as a finite double > 0.1; then a sign-set,
                // NaN or infinite far is >= bits(+inf), and a finite
                // non-negative far compares with near by bit pattern.
                const std::uint64_t near_bits = fpga_roi_double_bits(near_z[b]);
                const std::uint64_t far_bits = fpga_roi_double_bits(far_z[b]);
                if (r[0] > r[2] || r[1] > r[3] || r[2] >= width || r[3] >= height ||
                    r[2] - r[0] < 3 || r[3] - r[1] < 3 ||
                    near_bits <= FPGA_ROI_D_0_1 || near_bits >= FPGA_ROI_D_INF ||
                    far_bits >= FPGA_ROI_D_INF || far_bits < near_bits) return;
                const float front = static_cast<float>(near_z[b] + neg_margin);
                const float back = static_cast<float>(far_z[b] + depth_margin_m);
                // == !finite(front) || !finite(back) || front > back. Here
                // back = float(far + margin) with far > 0.1, margin >= 0.04,
                // so back is +inf or a positive float (sign clear); a front
                // with its sign set is never > back, otherwise the order is
                // the bit-pattern order.
                const std::uint32_t front_bits = fpga_roi_float_bits(front);
                const std::uint32_t back_bits = fpga_roi_float_bits(back);
                if ((front_bits & FPGA_ROI_F_EXP) == FPGA_ROI_F_EXP ||
                    (back_bits & FPGA_ROI_F_EXP) == FPGA_ROI_F_EXP ||
                    (!(front_bits & FPGA_ROI_SIGN) && front_bits > back_bits)) return;
                const std::uint32_t bit = 1u << b;
                active_mask |= bit;
                lane_front = fpga_roi_key(front);
                lane_back = fpga_roi_key(back);
                lane_area = (r[2] - r[0] + 1) * (r[3] - r[1] + 1);
#ifdef FPGA_M7_ROI_MUTANT_AREA_MINUS_ONE
                lane_area -= 1;  // Fault: area one short.
#endif
                col_cover[r[0]] ^= bit;
#ifdef FPGA_M7_ROI_MUTANT_OFF_BY_ONE
                col_cover[r[2]] ^= bit;  // Fault: right edge treated as exclusive.
#else
                col_cover[r[2] + 1] ^= bit;
#endif
                row_cover[r[1]] ^= bit;
                row_cover[r[3] + 1] ^= bit;
            }
        }
        // Every index is written (0 for disabled rows and rows past box_count).
        key_table_front[b] = lane_front;
        key_table_back[b] = lane_back;
        area_table[b] = lane_area;
    }
    std::uint32_t front_key[FPGA_M7_ROI_MAX_BOXES], back_key[FPGA_M7_ROI_MAX_BOXES];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=front_key type=complete
#pragma HLS ARRAY_PARTITION variable=back_key type=complete
#endif
    for (std::uint32_t k = 0; k < FPGA_M7_ROI_MAX_BOXES; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS UNROLL
#endif
#if defined(FPGA_M7_ROI_MUTANT_LANE_SHIFT_RTL) && defined(__SYNTHESIS__)
        // Fault, RTL only (C simulation unaffected): lane k gets box k-1's
        // keys. Positive control: C/RTL co-simulation must fail.
        const std::uint32_t src = (k + FPGA_M7_ROI_MAX_BOXES - 1) % FPGA_M7_ROI_MAX_BOXES;
#else
        const std::uint32_t src = k;
#endif
        // Keys are <= HI-LO already; the mask only tells HLS they fit 26 bits.
        front_key[k] = key_table_front[src] & FPGA_ROI_KEY_MASK;
        back_key[k] = key_table_back[src] & FPGA_ROI_KEY_MASK;
    }

    // Prefix XOR turns the toggles into coverage masks, in place.
    std::uint32_t col_run = 0, row_run = 0;
    for (std::uint32_t i = 0; i < span; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=1 avg=640 max=1280
#pragma HLS DEPENDENCE variable=col_cover inter false
#pragma HLS DEPENDENCE variable=row_cover inter false
#endif
        if (i < width) {
            col_run ^= col_cover[i];
            col_cover[i] = col_run;
        }
        if (i < height) {
            row_run ^= row_cover[i];
            row_cover[i] = row_run;
        }
    }

    std::uint32_t valid[FPGA_M7_ROI_MAX_BOXES];
    std::uint32_t ahead[FPGA_M7_ROI_MAX_BOXES], behind[FPGA_M7_ROI_MAX_BOXES];
#ifdef __SYNTHESIS__
#pragma HLS ARRAY_PARTITION variable=valid type=complete
#pragma HLS ARRAY_PARTITION variable=ahead type=complete
#pragma HLS ARRAY_PARTITION variable=behind type=complete
#endif
    for (std::uint32_t b = 0; b < FPGA_M7_ROI_MAX_BOXES; ++b) {
#ifdef __SYNTHESIS__
#pragma HLS UNROLL
#endif
        valid[b] = ahead[b] = behind[b] = 0;
    }

    // One flat pass, row-major, depth read strictly in address order.
    const std::uint32_t frame_pixels = width * height;
    std::uint32_t x = 0, y = 0;
    for (std::uint32_t i = 0; i < frame_pixels; ++i) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=16 avg=256000 max=1024000
#endif
        const std::uint32_t zbits = fpga_roi_float_bits(depth[i]);
        // == valid_depth(z): 0.1f < z < 10.0f, on the bit pattern.
        const bool depth_valid = zbits > FPGA_ROI_VALID_LO && zbits < FPGA_ROI_VALID_HI;
        const std::uint32_t offset = (zbits - FPGA_ROI_VALID_LO) & FPGA_ROI_KEY_MASK;
        std::uint32_t inside = col_cover[x] & row_cover[y];
#ifdef FPGA_M7_ROI_MUTANT_FIRST_ONLY
        inside &= 1u;  // Fault: only the first lane is wired.
#endif
#ifdef FPGA_M7_ROI_MUTANT_OVERLAP_FIRST_MATCH
        // Fault: a valid pixel feeds only the lowest-index box holding it.
        if (depth_valid) inside &= ~inside + 1u;
#endif
        for (std::uint32_t b = 0; b < FPGA_M7_ROI_MAX_BOXES; ++b) {
#ifdef __SYNTHESIS__
#pragma HLS UNROLL
#endif
            const bool in_box = (inside >> b) & 1u;
            const bool hit = in_box && depth_valid;
            valid[b] = (valid[b] + (hit ? 1u : 0u)) & FPGA_ROI_COUNT_MASK;
            ahead[b] = (ahead[b] + (hit && offset < front_key[b] ? 1u : 0u)) &
                       FPGA_ROI_COUNT_MASK;
            behind[b] = (behind[b] + (hit && offset > back_key[b] ? 1u : 0u)) &
                        FPGA_ROI_COUNT_MASK;
        }
        if (x + 1u == width) {
            x = 0;
            ++y;
        } else {
            ++x;
        }
    }

    // Verdict thresholds as exact integer ratio bounds (see fpga_roi_ratio_ge).
    FpgaRoiRatio ratio_bound[3];
    ratio_bound[0] = fpga_roi_ratio_bound(min_valid_fraction);
    ratio_bound[1] = fpga_roi_ratio_bound(0.5);
    ratio_bound[2] = fpga_roi_ratio_bound(free_fraction);
    for (std::uint32_t b = 0; b < FPGA_M7_ROI_MAX_BOXES; ++b) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
        if (b >= box_count) continue;
        const bool active = (active_mask >> b) & 1u;
        // The three ratio tests run one after another on one shift-add unit:
        // valid/pixels, ahead/valid, behind/valid. All three are evaluated;
        // the verdict below uses them in the E1 order and priority.
        bool ratio_ok[3];
        for (std::uint32_t k = 0; k < 3; ++k) {
#ifdef __SYNTHESIS__
#pragma HLS PIPELINE off
#endif
            const std::uint32_t num = k == 0 ? valid[b] : (k == 1 ? ahead[b] : behind[b]);
            const std::uint32_t den = k == 0 ? area_table[b] : valid[b];
            ratio_ok[k] = fpga_roi_ratio_ge(num, den, ratio_bound[k]);
        }
        std::uint32_t verdict = M7_RB_INDETERMINATE;
        if (active && valid[b] >= min_pixels && ratio_ok[0]) {
            // Occlusion has fixed priority over a free-space claim.
            if (ratio_ok[1]) {
#ifdef FPGA_M7_ROI_MUTANT_OCC_AS_FREE
                verdict = M7_RB_FREE_SPACE_CONFLICT;
#else
                verdict = M7_RB_OCCLUDED;
#endif
            } else if (ratio_ok[2]) {
                verdict = M7_RB_FREE_SPACE_CONFLICT;
            }
        }
        std::uint32_t* row = result + M7_RB_FIELDS * b;
        row[M7_RB_OBJECT_ID] = object_id[b];
        row[M7_RB_ENABLED] = active ? 1u : 0u;
        row[M7_RB_PIXELS] = area_table[b];
        row[M7_RB_VALID] = valid[b];
#ifdef FPGA_M7_ROI_DEBUG_KEYS
        // Test-only readout: the lane's threshold keys instead of the counts.
        row[M7_RB_FRONT] = front_key[b];
        row[M7_RB_BEHIND] = back_key[b];
#else
        row[M7_RB_FRONT] = ahead[b];
        row[M7_RB_BEHIND] = behind[b];
#endif
        row[M7_RB_VERDICT] = verdict;
    }
    *status = 0;
}
