// B1 navigation-grid core: C model in HLS-synthesizable style (2026-11-02).
// Contract in costmap.hpp; design and verification in
// docs/design/B1_COSTMAP_DESIGN_20261102.md.
//
// Four passes, each a pipelined loop at II=1:
//   1 project  (MY*WX*nb iterations, one URAM word per cycle): per physical
//              word column, OR "occupied" / AND "free" / OR "tall" over the
//              band layers -> 2-bit tri-state x 8 lanes, stored in LOCAL y and
//              PHYSICAL x order (the 8 lanes of a word stay together, so no
//              8-way scattered write is needed).
//   2 cells    (MX*MY): local (ly, lx) -> physical lane, body seed, writes
//              grid/aux, keeps the final tri-state for inflation.
//   3 vertical (2*MX*MY): per column, distance in cells to the nearest lethal
//              cell in that column, capped at B1_GCAP (forward + backward scan).
//   4 horizontal (MY*(MX+RMAX)), sliding window: per row, cost = max over |dx| <= B1_RMAX of
//              lut[|dx|][g(x+dx)]. Exact: for a fixed dx the nearest lethal
//              cell of that column gives the largest cost (the table falls with
//              |dy|), so this equals "max over every lethal cell of the cached
//              Nav2 cost" -- the exact form of what Nav2's wavefront
//              approximates (see the bench for how often they differ).
#include "costmap.hpp"

namespace {

inline std::uint32_t wrap_add(std::uint32_t a, std::uint32_t off, std::uint32_t m) {
    std::uint32_t v = a + off;
    return v >= m ? v - m : v;
}

inline std::uint32_t wrap_sub(std::uint32_t a, std::uint32_t off, std::uint32_t m) {
    return a >= off ? a - off : a + m - off;
}

}  // namespace

extern "C" std::uint32_t fpga_b1_costmap(
    const std::uint64_t map[B1_WORDS],
    std::uint32_t off_x, std::uint32_t off_y, std::uint32_t off_z,
    std::uint32_t band_lo, std::uint32_t band_hi, std::uint32_t tall_lo,
    std::uint32_t body_enable, std::int32_t body_cx, std::int32_t body_cy,
    std::uint32_t body_r,
    std::uint32_t infl_enable, const std::uint8_t lut[B1_LUT_SIZE],
    std::int8_t grid[B1_CELLS], std::uint8_t aux[B1_CELLS], std::uint8_t cost[B1_CELLS]) {
    // ---------------------------------------------------------------- checks
    if (off_x >= B1_MX || off_y >= B1_MY || off_z >= B1_MZ) return B1_BAD_OFFSETS;
    if (band_lo > band_hi || band_hi >= B1_MZ || tall_lo < band_lo || tall_lo > band_hi + 1)
        return B1_BAD_BAND;
    const std::int32_t lim = 1 << 28;
    if (body_enable > 1 || body_cx <= -lim || body_cx >= lim || body_cy <= -lim ||
        body_cy >= lim || body_r > 40u * static_cast<std::uint32_t>(B1_Q))
        return B1_BAD_BODY;
    if (infl_enable > 1) return B1_BAD_LUT;
    if (infl_enable) {
        if (lut[0] != B1_COST_LETHAL) return B1_BAD_LUT;
        for (std::uint32_t k = 0; k < B1_LUT_DIM; ++k) {
#pragma HLS PIPELINE II=1
            if (lut[(B1_LUT_DIM - 1) * B1_LUT_DIM + k] != 0 || lut[k * B1_LUT_DIM + B1_LUT_DIM - 1] != 0)
                return B1_BAD_LUT;
        }
    }

    static std::uint16_t tri_w[B1_MY][B1_WX];
    static std::uint8_t tall_w[B1_MY][B1_WX];
    static std::uint8_t leth[B1_MY][B1_MX];
    static std::uint8_t gf[B1_MY][B1_MX];
    static std::uint8_t gv[B1_MY][B1_MX];
    static std::uint8_t fin[B1_MY][B1_MX];

    // ------------------------------------------------------------- 1 project
    const std::uint32_t nb = band_hi - band_lo + 1;
    const std::uint32_t total = B1_MY * B1_WX * nb;
    std::uint32_t py = 0, w = 0, zi = 0;
    std::uint8_t acc_occ = 0, acc_free = 0xFF, acc_any_free = 0, acc_tall = 0;
project:
    for (std::uint32_t i = 0; i < total; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=800 max=25600 avg=4800
        const std::uint32_t lz = band_lo + zi;
        const std::uint32_t pz = wrap_add(lz, off_z, B1_MZ);
        const std::uint64_t word = map[(pz * B1_MY + py) * B1_WX + w];
        std::uint8_t occ = 0, fr = 0;
        for (std::uint32_t j = 0; j < 8; ++j) {
#pragma HLS UNROLL
            const std::int32_t l = static_cast<std::int8_t>((word >> (8 * j)) & 0xFF);
            const std::uint32_t st = fpga_m7_roll_state(l);
            if (st == FPGA_M7_ROLL_OCCUPIED) occ |= static_cast<std::uint8_t>(1u << j);
            if (st == FPGA_M7_ROLL_FREE) fr |= static_cast<std::uint8_t>(1u << j);
        }
        acc_occ |= occ;
        acc_free &= fr;
        acc_any_free |= fr;
        if (lz >= tall_lo) acc_tall |= occ;
        if (zi == nb - 1) {
#ifdef FPGA_B1_MUTANT_UNK_FREE
            const std::uint8_t free_mask = acc_any_free;
#else
            const std::uint8_t free_mask = acc_free;
#endif
            std::uint16_t t = 0;
            for (std::uint32_t j = 0; j < 8; ++j) {
#pragma HLS UNROLL
                std::uint16_t s = FPGA_M7_ROLL_UNKNOWN;
                if ((acc_occ >> j) & 1u) s = FPGA_M7_ROLL_OCCUPIED;
                else if ((free_mask >> j) & 1u) s = FPGA_M7_ROLL_FREE;
                t |= static_cast<std::uint16_t>(s << (2 * j));
            }
#ifdef FPGA_B1_MUTANT_RING_Y
            const std::uint32_t ly = py;
#else
            const std::uint32_t ly = wrap_sub(py, off_y, B1_MY);
#endif
            tri_w[ly][w] = t;
            tall_w[ly][w] = acc_tall;
            acc_occ = 0; acc_free = 0xFF; acc_any_free = 0; acc_tall = 0;
            zi = 0;
            if (w == B1_WX - 1) { w = 0; ++py; } else { ++w; }
        } else {
            ++zi;
        }
    }

    // --------------------------------------------------------------- 2 cells
    const std::int64_t r2 = static_cast<std::int64_t>(body_r) * static_cast<std::int64_t>(body_r);
cells_y:
    for (std::uint32_t ly = 0; ly < B1_MY; ++ly) {
        // closest-point distance along y of this row to the body centre (Q16)
        const std::int32_t y0 = static_cast<std::int32_t>(ly) * B1_Q;
        std::int64_t dy = 0;
        if (y0 - body_cy > 0) dy = y0 - body_cy;
        else if (body_cy - (y0 + B1_Q) > 0) dy = body_cy - (y0 + B1_Q);
    cells_x:
        for (std::uint32_t lx = 0; lx < B1_MX; ++lx) {
#pragma HLS PIPELINE II=1
            const std::uint32_t px = wrap_add(lx, off_x, B1_MX);
            const std::uint32_t pw = px >> 3, pj = px & 7u;
            const std::uint32_t strict = (tri_w[ly][pw] >> (2 * pj)) & 3u;
            const std::uint32_t tall = (tall_w[ly][pw] >> pj) & 1u;
            const std::int32_t x0 = static_cast<std::int32_t>(lx) * B1_Q;
            std::int64_t dx = 0;
            if (x0 - body_cx > 0) dx = x0 - body_cx;
            else if (body_cx - (x0 + B1_Q) > 0) dx = body_cx - (x0 + B1_Q);
            const bool in_body = body_enable && (dx * dx + dy * dy <= r2);
            std::uint32_t f = strict;
            if (in_body && f == FPGA_M7_ROLL_UNKNOWN) f = FPGA_M7_ROLL_FREE;
            const std::uint32_t idx = ly * B1_MX + lx;
            grid[idx] = f == FPGA_M7_ROLL_OCCUPIED ? static_cast<std::int8_t>(100)
                      : (f == FPGA_M7_ROLL_FREE ? static_cast<std::int8_t>(0)
                                               : static_cast<std::int8_t>(-1));
            aux[idx] = static_cast<std::uint8_t>(strict | (tall << 2));
            fin[ly][lx] = static_cast<std::uint8_t>(f);
            leth[ly][lx] = f == FPGA_M7_ROLL_OCCUPIED ? 1 : 0;
        }
    }

    if (!infl_enable) {
    zero_cost:
        for (std::uint32_t i = 0; i < B1_CELLS; ++i) {
#pragma HLS PIPELINE II=1
            cost[i] = 0;
        }
        return B1_OK;
    }

    // ------------------------------------------------------------ 3 vertical
    std::uint8_t gcol[B1_MX];
#pragma HLS ARRAY_PARTITION variable=gcol type=complete
init_fwd:
    for (std::uint32_t x = 0; x < B1_MX; ++x) {
#pragma HLS UNROLL
        gcol[x] = B1_GCAP;
    }
fwd_y:
    for (std::uint32_t ly = 0; ly < B1_MY; ++ly) {
    fwd_x:
        for (std::uint32_t lx = 0; lx < B1_MX; ++lx) {
#pragma HLS PIPELINE II=1
            const std::uint8_t up = static_cast<std::uint32_t>(gcol[lx]) + 1u > B1_GCAP ? static_cast<std::uint8_t>(B1_GCAP) : static_cast<std::uint8_t>(gcol[lx] + 1);
            const std::uint8_t g = leth[ly][lx] ? 0 : up;
            gcol[lx] = g;
            gf[ly][lx] = g;
        }
    }
init_bwd:
    for (std::uint32_t x = 0; x < B1_MX; ++x) {
#pragma HLS UNROLL
        gcol[x] = B1_GCAP;
    }
bwd_y:
    for (std::uint32_t k = 0; k < B1_MY; ++k) {
        const std::uint32_t ly = B1_MY - 1 - k;
    bwd_x:
        for (std::uint32_t lx = 0; lx < B1_MX; ++lx) {
#pragma HLS PIPELINE II=1
            const std::uint8_t dn = static_cast<std::uint32_t>(gcol[lx]) + 1u > B1_GCAP ? static_cast<std::uint8_t>(B1_GCAP) : static_cast<std::uint8_t>(gcol[lx] + 1);
            const std::uint8_t g = leth[ly][lx] ? 0 : dn;
            gcol[lx] = g;
            gv[ly][lx] = g < gf[ly][lx] ? g : gf[ly][lx];
        }
    }

    // ---------------------------------------------------------- 4 horizontal
    std::uint8_t tab[B1_LUT_DIM][B1_LUT_DIM];
#pragma HLS ARRAY_PARTITION variable=tab type=complete dim=0
load_lut:
    for (std::uint32_t i = 0; i < B1_LUT_SIZE; ++i) {
#pragma HLS PIPELINE II=1
        tab[i / B1_LUT_DIM][i % B1_LUT_DIM] = lut[i];
    }
    // sliding window of 2*RMAX+1 vertical distances: win[t] = g(column c - RMAX + t) for output
    // column c = x - RMAX; columns outside the window read B1_GCAP (table column GCAP is 0).
    const std::uint32_t TAPS = 2 * B1_RMAX + 1;
    std::uint8_t win[2 * B1_RMAX + 1];
#pragma HLS ARRAY_PARTITION variable=win type=complete
hor_y:
    for (std::uint32_t ly = 0; ly < B1_MY; ++ly) {
    hor_init:
        for (std::uint32_t t = 0; t < TAPS; ++t) {
#pragma HLS UNROLL
            win[t] = B1_GCAP;
        }
    hor_x:
        for (std::uint32_t x = 0; x < B1_MX + B1_RMAX; ++x) {
#pragma HLS PIPELINE II=1
            for (std::uint32_t t = 0; t + 1 < TAPS; ++t) {
#pragma HLS UNROLL
                win[t] = win[t + 1];
            }
            win[TAPS - 1] = x < B1_MX ? gv[ly][x] : static_cast<std::uint8_t>(B1_GCAP);
            if (x >= B1_RMAX) {
                const std::uint32_t lx = x - B1_RMAX;
                std::uint8_t c = 0;
                for (std::uint32_t t = 0; t < TAPS; ++t) {
#pragma HLS UNROLL
                    const std::uint32_t ak = t >= B1_RMAX ? t - B1_RMAX : B1_RMAX - t;
                    const std::uint8_t v = tab[ak][win[t]];
                    if (v > c) c = v;
                }
                const std::uint32_t f = fin[ly][lx];
                std::uint8_t out;
                if (f == FPGA_M7_ROLL_OCCUPIED) out = B1_COST_LETHAL;
                else if (f == FPGA_M7_ROLL_UNKNOWN)
                    out = c >= B1_COST_INSCRIBED ? c : static_cast<std::uint8_t>(B1_COST_UNKNOWN);
                else out = c;
                cost[ly * B1_MX + lx] = out;
            }
        }
    }
    return B1_OK;
}
