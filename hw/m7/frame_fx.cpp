// GEO_FX3 (2026-11-04): see frame_fx.hpp. Compile with -ffp-contract=off (host and aarch64).
//
// fpga_m7_fx_prepare is the arithmetic of GEO_FX0 tools/fx_golden.cpp fx_prepare (DESIGN.md
// section 2.2) written again here; fpga_m7_fx_frame is its FX arm (fx_E at 22 fractional bits),
// back_run (Dlat2 thinning as the HLS convert stage) and add_ray (hw/m7/frame_depth.cpp DDA),
// with the core's register-level decoding in front (slot bit fields, 13-bit origin, 16-bit
// thresholds) and the statistics / crc registers added.
//
// Positive controls (GEO_FX3 ps_check only; the bit-exact check must report both):
//   -DFPGA_M7_FX_CONTROL_SWAP=1  adds the three products of A[a][j] in the order (2 + 1) + 0
//   -DFPGA_M7_FX_CONTROL_SWAP=2  O = (t - org) / voxel * 256 instead of (t - org) * 256 / voxel
//                               (bitwise the same: * 256 is exact; kept as a recorded negative)
//   -DFPGA_M7_FX_CONTROL_SWAP=3  s one ulp up (sensitivity floor of the check)
#include "frame_fx.hpp"

#include <cfloat>
#include <cmath>
#include <cstring>
#include <vector>

namespace {

const std::int32_t kQ8 = 256;
const std::uint32_t kCap = 32u * 32u * 32u;
const std::uint32_t kAxis = 32;
const std::uint32_t kMaxW = 1280, kMaxH = 800;
const int kFacc = 36;  // accumulator fractional bits (Gu, Gv, G0)
const int kFp = 22;    // fractional bits of P (27-bit DSP A operand) and of O, E

bool finite_f(float v) { return v >= -FLT_MAX && v <= FLT_MAX; }
bool finite_d(double v) { return v >= -DBL_MAX && v <= DBL_MAX; }
double rne_scale(double x, int bits) { return std::nearbyint(std::ldexp(x, bits)); }

// thresholds depend on the unit only (f32(code) * unit is monotone in code): cache the last one
struct ThrCache {
    bool ok = false;
    std::uint32_t unit_bits = 0, c_lo = 0, c_hi = 0, c20 = 0;
};
thread_local ThrCache g_thr;

void thresholds(float unit, std::uint32_t& c_lo, std::uint32_t& c_hi, std::uint32_t& c20) {
    std::uint32_t ub;
    std::memcpy(&ub, &unit, 4);
    if (!g_thr.ok || g_thr.unit_bits != ub) {
        g_thr.c_lo = 65536; g_thr.c_hi = 0; g_thr.c20 = 0;
        for (std::uint32_t c = 1; c < 65536; ++c) {
            const float z = static_cast<float>(c) * unit;
            if (z > 0.1f && z < 10.0f) { if (c < g_thr.c_lo) g_thr.c_lo = c; g_thr.c_hi = c; }
            if (z <= 20.0f) g_thr.c20 = c;
        }
        g_thr.unit_bits = ub;
        g_thr.ok = true;
    }
    c_lo = g_thr.c_lo; c_hi = g_thr.c_hi; c20 = g_thr.c20;
}

std::int32_t floor_q8(std::int32_t value) {
    return value >= 0 ? value / kQ8 : -((-value + kQ8 - 1) / kQ8);
}
bool cell_in_grid(const std::int32_t c[3], const std::uint32_t d[3]) {
    return c[0] >= 0 && c[0] < (std::int32_t)d[0] && c[1] >= 0 && c[1] < (std::int32_t)d[1] &&
           c[2] >= 0 && c[2] < (std::int32_t)d[2];
}
std::uint32_t grid_offset(const std::int32_t c[3], std::uint32_t nx, std::uint32_t ny) {
    return ((std::uint32_t)c[2] * ny + (std::uint32_t)c[1]) * nx + (std::uint32_t)c[0];
}
// hw/m7/frame_depth.cpp add_ray, verbatim
void add_ray(const std::int32_t origin[3], const std::int32_t end[3], const std::uint32_t dims[3],
             std::uint8_t* grid) {
    std::int32_t cell[3], target[3], direction[3], next[3], delta[3];
    for (int a = 0; a < 3; ++a) {
        const std::int32_t from = origin[a], to = end[a];
        cell[a] = from / kQ8;
        target[a] = floor_q8(to);
        direction[a] = to > from ? 1 : (to < from ? -1 : 0);
        delta[a] = to > from ? to - from : from - to;
        next[a] = direction[a] > 0 ? (cell[a] + 1) * kQ8 - from : from - cell[a] * kQ8;
    }
    for (std::uint32_t step = 0; step < 3 * kAxis; ++step) {
        if (cell[0] == target[0] && cell[1] == target[1] && cell[2] == target[2]) break;
        int first = -1;
        for (int a = 0; a < 3; ++a) {
            if (direction[a] == 0 || cell[a] == target[a]) continue;
            if (first < 0 || (std::int64_t)next[a] * delta[first] < (std::int64_t)next[first] * delta[a]) first = a;
        }
        if (first < 0) break;
        const std::int32_t num = next[first], den = delta[first];
        for (int a = 0; a < 3; ++a) {
            if (direction[a] == 0 || cell[a] == target[a]) continue;
            if ((std::int64_t)next[a] * den == (std::int64_t)num * delta[a]) {
                cell[a] += direction[a];
                next[a] += kQ8;
            }
        }
        if (!cell_in_grid(cell, dims)) break;
        if (cell[0] != target[0] || cell[1] != target[1] || cell[2] != target[2]) {
            const std::uint32_t idx = grid_offset(cell, dims[0], dims[1]);
            if (grid[idx] == 0) grid[idx] = 1;
        }
    }
    if (cell_in_grid(target, dims)) grid[grid_offset(target, dims[0], dims[1])] = 2;
}

std::int64_t sext(std::uint64_t v, int bits) {
    const std::uint64_t m = (bits == 64) ? ~0ull : ((1ull << bits) - 1);
    v &= m;
    if (bits < 64 && (v >> (bits - 1)) & 1) v |= ~m;
    return static_cast<std::int64_t>(v);
}

std::uint32_t crc_table[256];
bool crc_init = false;
std::uint32_t crc32_update(std::uint32_t crc, const std::uint8_t* p, std::size_t n) {
    if (!crc_init) {
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            crc_table[i] = c;
        }
        crc_init = true;
    }
    crc = ~crc;
    for (std::size_t i = 0; i < n; ++i) crc = crc_table[(crc ^ p[i]) & 0xffu] ^ (crc >> 8);
    return ~crc;
}

}  // namespace

extern "C" int fpga_m7_fx_prepare(std::uint32_t w, std::uint32_t h, std::uint32_t nx, std::uint32_t ny,
                                 std::uint32_t nz, float unit, const float ik[9], const float wc[12],
                                 const double org[3], double voxel, double reach_limit,
                                 std::int64_t gk[12], std::int32_t origin[3], std::uint32_t thr[3],
                                 double env[2]) {
    for (int i = 0; i < 12; ++i) gk[i] = 0;
    origin[0] = origin[1] = origin[2] = 0;
    thr[0] = thr[1] = thr[2] = 0;
    env[0] = env[1] = 0;
    // status 2 (reference)
    if (!finite_d(voxel) || voxel <= 0) return 2;
    for (int a = 0; a < 3; ++a) if (!finite_d(org[a])) return 2;
    for (int i = 0; i < 9; ++i) if (!finite_f(ik[i])) return 2;
    for (int i = 0; i < 12; ++i) if (!finite_f(wc[i])) return 2;
    // status 3: reference origin, verbatim
    const std::uint32_t dims[3] = {nx, ny, nz};
    for (int a = 0; a < 3; ++a) {
        const double c = std::floor((static_cast<double>(wc[4 * a + 3]) - org[a]) / voxel * kQ8);
        if (!finite_d(c) || c < 0 || c >= dims[a] * kQ8) return 3;
        origin[a] = static_cast<std::int32_t>(c);
    }
    std::uint32_t c_lo, c_hi, c20;
    thresholds(unit, c_lo, c_hi, c20);
    thr[0] = c_lo; thr[1] = c_hi; thr[2] = c20;
    // per-frame coefficients, double, fixed order (no FMA)
    const double s = static_cast<double>(unit) * 256.0 / voxel;
#if defined(FPGA_M7_FX_CONTROL_SWAP) && FPGA_M7_FX_CONTROL_SWAP == 3
    const double s_ctl = std::nextafter(s, 1e300);
#define s s_ctl
#endif
    double pmax = 0, reach = 0;
    for (int a = 0; a < 3; ++a) {
        double A[3];
        for (int j = 0; j < 3; ++j) {
#if defined(FPGA_M7_FX_CONTROL_SWAP) && FPGA_M7_FX_CONTROL_SWAP == 1
            double acc = static_cast<double>(wc[4 * a + 2]) * static_cast<double>(ik[6 + j]);
            acc = acc + static_cast<double>(wc[4 * a + 1]) * static_cast<double>(ik[3 + j]);
            acc = acc + static_cast<double>(wc[4 * a]) * static_cast<double>(ik[j]);
#else
            double acc = static_cast<double>(wc[4 * a]) * static_cast<double>(ik[j]);
            acc = acc + static_cast<double>(wc[4 * a + 1]) * static_cast<double>(ik[3 + j]);
            acc = acc + static_cast<double>(wc[4 * a + 2]) * static_cast<double>(ik[6 + j]);
#endif
            A[j] = acc * s;
        }
        gk[a] = static_cast<std::int64_t>(rne_scale(A[0], kFacc));
        gk[3 + a] = static_cast<std::int64_t>(rne_scale(A[1], kFacc));
        gk[6 + a] = static_cast<std::int64_t>(rne_scale(A[2], kFacc));
#if defined(FPGA_M7_FX_CONTROL_SWAP) && FPGA_M7_FX_CONTROL_SWAP == 2
        const double Oa = (static_cast<double>(wc[4 * a + 3]) - org[a]) / voxel * 256.0;
#else
        const double Oa = (static_cast<double>(wc[4 * a + 3]) - org[a]) * 256.0 / voxel;
#endif
        gk[9 + a] = static_cast<std::int64_t>(rne_scale(Oa, kFp));
        double pm = 0;
        for (int cu = 0; cu < 2; ++cu)
            for (int cv = 0; cv < 2; ++cv) {
                const double u = cu ? w - 1.0 : 0.0, v = cv ? h - 1.0 : 0.0;
                pm = std::fmax(pm, std::fabs(A[0] * u + A[1] * v + A[2]));
            }
        pmax = std::fmax(pmax, pm);
        reach = std::fmax(reach, pm * c_hi + std::fabs(Oa));
    }
#ifdef s
#undef s
#endif
    env[0] = pmax; env[1] = reach;
    if (c_lo > c_hi) return 5;
    if (!(pmax < 15.5)) return 5;
    if (!(reach < reach_limit)) return 5;
    return 0;
}

extern "C" void fpga_m7_fx_pack(const std::int64_t gk[12], std::uint32_t words[FPGA_M7_FX_K_WORDS]) {
    for (int s = 0; s < 12; ++s) {
        const std::uint64_t v = static_cast<std::uint64_t>(gk[s]);
        words[2 * s] = static_cast<std::uint32_t>(v);
        words[2 * s + 1] = static_cast<std::uint32_t>(v >> 32);
    }
}

extern "C" std::uint32_t fpga_m7_fx_frame(const std::uint16_t* codes, std::uint32_t w, std::uint32_t h,
                                         std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
                                         const std::uint32_t words[FPGA_M7_FX_K_WORDS],
                                         const std::uint32_t origin_reg[3], std::uint32_t codes_reg,
                                         std::uint32_t c20_reg, std::uint32_t dlat2_en,
                                         std::uint32_t unit_bits, std::uint8_t* flags,
                                         std::uint32_t counts[3], std::uint32_t stats[4],
                                         std::uint32_t crc[1]) {
    counts[0] = counts[1] = counts[2] = 0;
    stats[0] = stats[1] = stats[2] = stats[3] = 0;
    crc[0] = 0;
    // core status 1 (m7_frame_stream.cpp top)
    if ((w == 0) != (h == 0) || w > kMaxW || h > kMaxH || nx == 0 || ny == 0 || nz == 0 ||
        nx > kAxis || ny > kAxis || nz > kAxis) return 1;
    // register decoding as the core
    std::int64_t G[12];
    for (int s = 0; s < 12; ++s) {
        const std::uint64_t v = words[2 * s] | (static_cast<std::uint64_t>(words[2 * s + 1]) << 32);
        G[s] = sext(v, s < 9 ? 41 : 36);
    }
    const std::int32_t origin[3] = {(std::int32_t)(origin_reg[0] & 0x1fffu),
                                    (std::int32_t)(origin_reg[1] & 0x1fffu),
                                    (std::int32_t)(origin_reg[2] & 0x1fffu)};
    const std::uint32_t c_lo = codes_reg & 0xffffu, c_hi = codes_reg >> 16, c20 = c20_reg & 0xffffu;
    float unit;
    std::memcpy(&unit, &unit_bits, 4);
    const std::uint32_t dims[3] = {nx, ny, nz};
    const bool dlat = (dlat2_en & 1u) != 0;
    const std::uint32_t np = w * h;
    std::vector<std::uint8_t> grid(kCap, 0), ev(kCap, 0), blk(kMaxW / 2, 0);
    std::uint32_t accepted = 0, invalid = 0, clipped = 0;
    std::uint32_t npos = 0, le20 = 0, maxle20 = 0, mingt20 = 65535, nvalid = 0, minvalid = 65535, maxcode = 0;
    bool any_gt20 = false, prev_in = false, prev_cl = false;
    std::uint32_t c = 0;
    const int sh = kFacc - kFp;
    for (std::uint32_t p = 0; p < np; ++p) {
        const std::uint32_t u = p % w, v = p / w, code = codes[p];
        // depth crc (float32 depth of the A53 rule; code 0 -> quiet NaN)
        const std::uint32_t zbits = code ? 0u : 0x7fc00000u;
        float z = static_cast<float>(code) * unit;
        std::uint32_t zb;
        std::memcpy(&zb, &z, 4);
        if (!code) zb = zbits;
        std::uint8_t zbytes[4] = {(std::uint8_t)zb, (std::uint8_t)(zb >> 8), (std::uint8_t)(zb >> 16),
                                  (std::uint8_t)(zb >> 24)};
        c = crc32_update(c, zbytes, 4);
        // statistics (DESIGN.md section 3.3)
        if (code >= 1) {
            ++npos;
            if (code > maxcode) maxcode = code;
            if (code <= c20) { ++le20; if (code > maxle20) maxle20 = code; }
            else { any_gt20 = true; if (code < mingt20) mingt20 = code; }
        }
        const bool valid = code >= c_lo && code <= c_hi;
        if (valid) { ++nvalid; if (code < minvalid) minvalid = code; }
        // FX front end: E = P27 * code + O, P27 = (G0 + 2^13 + v Gv + u Gu) >> 14, end = E >> 22
        std::int32_t end[3] = {0, 0, 0};
        if (valid)
            for (int a = 0; a < 3; ++a) {
                const std::int64_t pacc = G[6 + a] + (std::int64_t(1) << (sh - 1)) + std::int64_t(v) * G[3 + a] +
                                          std::int64_t(u) * G[a];
                const std::int64_t pp = pacc >> sh;
                const std::int64_t E = pp * std::int64_t(code) + G[9 + a];
                end[a] = static_cast<std::int32_t>(E >> kFp);
            }
        // Dlat2 back end (golden back_run)
        bool inside = true;
        for (int a = 0; a < 3; ++a)
            if (end[a] < 0 || end[a] >= (std::int32_t)(dims[a] * kQ8)) inside = false;
        const bool is_in = valid && inside, is_cl = valid && !inside;
        const bool u_odd = u & 1, v_odd = v & 1;
        const std::uint32_t bcol = u >> 1;
        const std::uint8_t above = v_odd ? blk[bcol] : 0;
        const bool in_seen = (above & 1) || (u_odd && prev_in);
        const bool cl_seen = (above & 2) || (u_odd && prev_cl);
        const std::uint8_t mark = (in_seen || is_in ? 1 : 0) | (cl_seen || is_cl ? 2 : 0);
        if (!v_odd) blk[bcol] = mark;
        bool ev_seen = false;
        if (is_in) {
            const std::uint32_t e = ((std::uint32_t)(end[2] >> 8) * ny + (std::uint32_t)(end[1] >> 8)) * nx +
                                    (std::uint32_t)(end[0] >> 8);
            ev_seen = ev[e] != 0;
            ev[e] = 1;
        }
        prev_in = is_in;
        prev_cl = is_cl;
        const bool keep_in = !in_seen || !ev_seen;
        const bool keep = !dlat || (inside ? keep_in : !cl_seen);
        if (!valid) ++invalid; else if (inside) ++accepted; else ++clipped;
        if (valid && keep) add_ray(origin, end, dims, grid.data());
    }
    std::memcpy(flags, grid.data(), kCap);
    counts[0] = accepted; counts[1] = invalid; counts[2] = clipped;
    stats[0] = npos;
    stats[1] = le20;
    stats[2] = maxle20 | ((any_gt20 ? mingt20 : 0u) << 16);
    stats[3] = (nvalid ? minvalid : 0u) | (maxcode << 16);
    crc[0] = c;
    return 0;
}
