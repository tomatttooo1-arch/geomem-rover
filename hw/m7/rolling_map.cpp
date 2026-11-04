// M7 rolling log-odds voxel map: on-chip resident HLS top.
//
// Storage: 8 int8 log-odds cells along x per 64-bit word, bound to URAM.
// Rolling uses per-axis ring offsets: physical = (local + offset) mod M.
// A window move only changes the offsets and clears the physical slices
// that the leaving and the entering cells share; no cell is copied.
// See docs/design/M7_ROLLING_FUSION_INTERFACE_20260926.md for the contract.
#include "rolling_map.hpp"

namespace {

typedef std::uint64_t word_t;

const std::uint32_t MX = FPGA_M7_ROLL_DIM_X;
const std::uint32_t MY = FPGA_M7_ROLL_DIM_Y;
const std::uint32_t MZ = FPGA_M7_ROLL_DIM_Z;
const std::uint32_t WX = FPGA_M7_ROLL_WORDS_X;
const std::uint32_t WORDS = FPGA_M7_ROLL_WORDS;

// SLIM_FUSE2 item 2: first word of physical row (pz, py) = (pz*MY + py)*WX with
// narrow operands and, for the delivered 80x80 window, shifts and adds only
// (pz*800 + py*10; pz < 32, py < 80 -> < 25,600, 15 bits). The 2026-10 product
// had two 32x32 multipliers here and for onx*ony*onz (6 DSP).
// SLIM_FUSE2 item 7: HLS folds the shift-add form back into pz*800 + py*10 and
// binds it to a DSP multiply-add (mac_muladd_8ns_10ns_*) at every call site (one
// DSP each in clear_z/y/x, fuse_loop and two in query_loop); the products are
// written as multiplies bound to fabric instead (same value, no DSP).
inline std::uint16_t row_word(std::uint8_t pz, std::uint8_t py) {
#if FPGA_M7_ROLL_MX == 80 && FPGA_M7_ROLL_MY == 80
    const std::uint16_t zb = static_cast<std::uint16_t>(static_cast<std::uint16_t>(pz) * 800u);
#pragma HLS BIND_OP variable=zb op=mul impl=fabric
    const std::uint16_t yb = static_cast<std::uint16_t>(static_cast<std::uint16_t>(py) * 10u);
#pragma HLS BIND_OP variable=yb op=mul impl=fabric
    return static_cast<std::uint16_t>(zb + yb);
#else
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(pz) * MY + py) * WX);
#endif
}

// fusion word list: a <= 32-cell block row touches at most 5 words
const std::uint32_t FUSE_MAX_WORDS = 5;
const std::uint32_t FUSE_ENTRIES = FPGA_M7_ROLL_OBS_CAPACITY / FPGA_M7_ROLL_LANES;   // 16-bit entries
static_assert((FUSE_ENTRIES & (FUSE_ENTRIES - 1)) == 0, "entry count power of two");

// cell i of the 2-bit block (item 6 layout)
inline std::uint32_t obs2_cell(const std::uint16_t obs2[2][FPGA_M7_ROLL_OBS2_BANK], std::uint32_t i) {
    const std::uint32_t k = i >> 3;
    return (obs2[k & 1u][k >> 1] >> (2 * (i & 7u))) & 3u;
}
static_assert(FPGA_M7_ROLL_OBS_AXIS == 32, "fusion path selection (#if MX > 32) assumes 32-cell rows");
static_assert(FPGA_M7_ROLL_MX <= 80 && FPGA_M7_ROLL_MY <= 80 && FPGA_M7_ROLL_MZ <= 80,
              "clamp_off() and the 8-bit fusion set-up assume windows <= 80 cells");

bool coord_ok(std::int32_t v) {
    return v > -FPGA_M7_ROLL_COORD_LIMIT && v < FPGA_M7_ROLL_COORD_LIMIT;
}

// New ring offset and the evicted physical range for a move of d (|d| < m).
// SLIM_FUSE2 item 5: d is the window move after the full-clear test, so
// |d| < m <= 248 (int16); offsets / starts / lengths are < m (8 bits).
void ring_move(std::int16_t d, std::uint8_t m, std::uint8_t* offset,
               std::uint8_t* start, std::uint8_t* len) {
#ifdef FPGA_M7_ROLL_MUTANT_SHIFT_SIGN
    d = static_cast<std::int16_t>(-d);
#endif
    const std::int16_t mm = m;
    const std::int16_t off = *offset;
    if (d > 0) {
        // Leaving locals [0, d) sit at physical off..off+d-1.
        *start = *offset;
        *len = static_cast<std::uint8_t>(d);
    } else if (d < 0) {
        // Leaving locals [m+d, m) sit at physical off+m+d .. off+m-1.
        std::int16_t st = static_cast<std::int16_t>(off + mm + d);
        if (st >= mm) st = static_cast<std::int16_t>(st - mm);
        *start = static_cast<std::uint8_t>(st);
        *len = static_cast<std::uint8_t>(-d);
    } else {
        *start = 0;
        *len = 0;
    }
    std::int16_t next = static_cast<std::int16_t>(off + d);
    if (next < 0) next = static_cast<std::int16_t>(next + mm);
    if (next >= mm) next = static_cast<std::int16_t>(next - mm);
    *offset = static_cast<std::uint8_t>(next);
}

// SLIM_FUSE2 item 5: a block offset (obs base - window base, |.| < 2^29) only
// matters through "where does [e, e+32) meet [0, M)"; clamping it to [-64, 127]
// keeps every such answer for M <= 80 (e < -64: e+32 < 0, nothing inside;
// e > 127: e >= M, nothing inside) and lets the fusion set-up run on 8 bits.
inline std::int8_t clamp_off(std::int32_t e) {
    return static_cast<std::int8_t>(e < -64 ? -64 : (e > 127 ? 127 : e));
}

// SLIM_FUSE2 item 3: one clear engine for RESET, the full clear and the window
// move. The cells to clear are the union of three slices: physical layers
// [sz, sz+lz), rows [sy, sy+ly) and columns [sx, sx+lx) (ring ranges; lz = MZ
// clears everything). The old engine scanned all 25,600 words for any move;
// this one visits each word at most once and only words with a cell to clear:
//   Z pass  every word of the evicted layers                (write 0)
//   Y pass  evicted rows of the kept layers (ring complement [sz+lz, sz+MZ))
//                                                           (write 0)
//   X pass  kept rows of kept layers, only the words holding evicted columns
//           (read, clear those lanes, write)
// The three sets are disjoint, so the result (each cell zero iff it lies in a
// slice, every other cell unchanged) equals the old engine's and the visit count
// never exceeds WORDS. Clearing writes no delta, so only the final map matters.
inline std::uint8_t ring_add(std::uint8_t a, std::uint8_t b, std::uint8_t m) {
    const std::uint8_t v = static_cast<std::uint8_t>(a + b);   // a, b < m <= 248: no wrap
    return v >= m ? static_cast<std::uint8_t>(v - m) : v;
}

void clear_slices(word_t map[FPGA_M7_ROLL_WORDS],
                  std::uint8_t sx, std::uint8_t lx,
                  std::uint8_t sy, std::uint8_t ly,
                  std::uint8_t sz, std::uint8_t lz) {
    // Z pass: lz*MX*MY/8 words, contiguous per layer.
    // FUSE_SLIM3 cut 2: set-up at real widths (lz <= MZ, ly <= MY, lx < MX,
    // all <= 80: 7 bits; kept counts likewise; nw <= WX = 10: 4 bits), the three
    // "% M" of values in [0, M] (were sequential dividers, 12-13 cycles each)
    // as one compare, and the X pass keep mask from two precomputed 8-bit lane
    // masks instead of 8 ring compares per word.
    const std::uint16_t z_words = static_cast<std::uint16_t>((lz & 127u) * (MY * WX));
#pragma HLS BIND_OP variable=z_words op=mul impl=fabric
    std::uint8_t pz = sz;
    std::uint16_t j = 0;
clear_z_loop:
    for (std::uint16_t i = 0; i < z_words; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=25600
        map[row_word(pz, 0) + j] = 0;
        if (j == MY * WX - 1) { j = 0; pz = ring_add(pz, 1, MZ); } else { ++j; }
    }
    const std::uint8_t kz_n = static_cast<std::uint8_t>((MZ - lz) & 127u);   // kept layers
    const std::uint8_t pz0 = ring_add(sz, static_cast<std::uint8_t>(lz == MZ ? 0 : lz), MZ);
    // Y pass: kept layers x evicted rows x all WX words.
    const std::uint16_t y_rows = static_cast<std::uint16_t>(kz_n * (ly & 127u));
#pragma HLS BIND_OP variable=y_rows op=mul impl=fabric
    const std::uint16_t y_words = static_cast<std::uint16_t>(y_rows * WX);
#pragma HLS BIND_OP variable=y_words op=mul impl=fabric
    pz = pz0;
    std::uint8_t ky = 0, w = 0;
clear_y_loop:
    for (std::uint16_t i = 0; i < y_words; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=25600
        map[row_word(pz, ring_add(sy, ky, MY)) + w] = 0;
        if (w == WX - 1) {
            w = 0;
            if (ky == ly - 1) { ky = 0; pz = ring_add(pz, 1, MZ); } else { ++ky; }
        } else {
            ++w;
        }
    }
    // X pass: words holding evicted columns, a ring range of words [ws, ws+nw).
    // c1x = last evicted column (unused when lx = 0: no X pass); x_wrap = the
    // range comes back into its first word.
    std::uint8_t nw = 0, ws = static_cast<std::uint8_t>(sx >> 3);
    const std::uint8_t c1x = ring_add(sx, static_cast<std::uint8_t>((lx - 1) & 127u), MX);
    const bool x_wrap = (sx & 7u) + lx > MX;
    if (lx != 0) {
        const std::uint8_t we = static_cast<std::uint8_t>(c1x >> 3);
        nw = x_wrap ? static_cast<std::uint8_t>(WX)
           : static_cast<std::uint8_t>((we >= ws ? we - ws : we + WX - ws) + 1);
    }
    const std::uint8_t ky_n = static_cast<std::uint8_t>((MY - ly) & 127u);   // kept rows
    // ly < MY always (a move has |d| < M; RESET and the full clear pass ly = 0),
    // so the old ly % MY is ly and ky_n >= 1 (the old (ky_n + MY - 1) % MY is
    // ky_n - 1). lz = MZ (every layer) leaves no kept layer, so pz0 is unused then.
    const std::uint8_t py0 = ring_add(sy, ly, MY);
    const std::uint16_t x_rows = static_cast<std::uint16_t>(kz_n * ky_n);
#pragma HLS BIND_OP variable=x_rows op=mul impl=fabric
    std::uint16_t x_words = static_cast<std::uint16_t>((x_rows & 8191u) * (nw & 15u));
#pragma HLS BIND_OP variable=x_words op=mul impl=fabric
    const std::uint8_t py_last = ring_add(py0, static_cast<std::uint8_t>(ky_n - 1), MY);
    // Lane masks of the evicted column ring range [sx, sx+lx) (lx >= 1 when the
    // pass runs). The k-th visited word is ws + k (ring). Word ws holds lanes
    // >= sx%8 of the range (head), the last word we lanes <= c1%8 (tail),
    // words strictly between are wholly evicted. When the range comes back into
    // its first word (nw = WX, then we = ws) that word is evicted on head | tail
    // and every other word wholly; when it is one word (nw = 1) on head & tail.
    const std::uint8_t ev_head = static_cast<std::uint8_t>(0xFFu << (sx & 7u));
    const std::uint8_t ev_tail = static_cast<std::uint8_t>(0xFFu >> (7u - (c1x & 7u)));
    pz = pz0;
    std::uint8_t py = py0, k = 0;
    w = ws;
clear_x_loop:
    for (std::uint16_t i = 0; i < x_words; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=25600
#pragma HLS DEPENDENCE variable=map inter false
        const std::uint16_t a = static_cast<std::uint16_t>(row_word(pz, py) + w);
        const bool first = k == 0, lastw = k == nw - 1;
        const std::uint8_t ev = x_wrap
            ? (first ? static_cast<std::uint8_t>(ev_head | ev_tail) : static_cast<std::uint8_t>(0xFFu))
            : static_cast<std::uint8_t>((first ? ev_head : 0xFFu) & (lastw ? ev_tail : 0xFFu));
        word_t keep = 0;
        for (std::uint32_t l = 0; l < FPGA_M7_ROLL_LANES; ++l) {
#pragma HLS UNROLL
            if (((ev >> l) & 1u) == 0) keep |= static_cast<word_t>(0xFFu) << (8 * l);
        }
        map[a] = map[a] & keep;
        if (k == nw - 1) {
            k = 0;
            w = ws;
            if (py == py_last) {
                py = py0;
                pz = ring_add(pz, 1, MZ);
            } else {
                py = ring_add(py, 1, MY);
            }
        } else {
            ++k;
            w = ring_add(w, 1, static_cast<std::uint8_t>(WX));
        }
    }
}

#ifdef FPGA_M7_ROLL_GRID
// ---------------------------------------------------------------------------
// B1h op GRID (2026-11-02): the resident map -> 2-D navigation planes, read
// straight from URAM. Same semantics as hw/b1_costmap/costmap.cpp (the C model
// checked cell by cell against the Nano bridge and a Nav2 replica); written
// separately so the integrated op and the model check each other. Differences
// in form only: outputs are packed 8 cells per word into the delta buffer
// (so the AXI shell's existing delta burst carries them), "lethal" is read
// from the final tri-state instead of a separate array, and the cost plane is
// not written when inflation is off.
const std::uint32_t G_RMAX = FPGA_M7_ROLL_GRID_RMAX;
const std::uint32_t G_DIM = FPGA_M7_ROLL_GRID_LUT_DIM;
const std::uint32_t G_GCAP = G_RMAX + 1;
const std::uint32_t G_PW = FPGA_M7_ROLL_GRID_PLANE_WORDS;
const std::uint32_t G_CELLS = MX * MY;
const std::int32_t G_Q = 65536;

inline std::uint32_t g_wrap_add(std::uint32_t a, std::uint32_t off, std::uint32_t m) {
    const std::uint32_t v = a + off;
    return v >= m ? v - m : v;
}

// 0 = accepted, else the reason written to header CHANGED_CELLS.
std::uint32_t grid_check(std::uint32_t band_lo, std::uint32_t band_hi, std::uint32_t tall_lo,
                         bool centre_ok, std::uint32_t r,
                         std::uint32_t flags,
                         const std::uint8_t lut[FPGA_M7_ROLL_LANES][FPGA_M7_ROLL_GTAB_BANK]) {
    if (flags > 3u) return 5;
    if (band_lo > band_hi || band_hi >= MZ || tall_lo < band_lo || tall_lo > band_hi + 1) return 2;
    if (!centre_ok || r > 40u * static_cast<std::uint32_t>(G_Q)) return 3;
    if (flags & 2u) {
        // [0][0] = 254, last row and column 0, and (B1h-v2) every row non-increasing
        // along |dy| -- true for any Nav2 table (cost falls with distance); the tap
        // pass relies on it to fold taps +-dx into one. One pass, row/column counters.
        bool bad = false;
        std::uint8_t row = 0, col = 0, prev = 0;
grid_lut_check:
        for (std::uint16_t i = 0; i < G_DIM * G_DIM; ++i) {
#pragma HLS PIPELINE II=1
            const std::uint8_t v = lut[i & 7u][i >> 3];
            if (i == 0 && v != 254) bad = true;
            if ((row == G_DIM - 1 || col == G_DIM - 1) && v != 0) bad = true;
            if (col != 0 && v > prev) bad = true;
            prev = v;
            if (col == G_DIM - 1) { col = 0; ++row; } else { ++col; }
        }
        if (bad) return 4;
    }
    return 0;
}

// Returns the number of words written to out[].
std::uint32_t grid_run(const word_t map[FPGA_M7_ROLL_WORDS],
                       std::uint32_t off_x, std::uint32_t off_y, std::uint32_t off_z,
                       std::uint32_t band_lo, std::uint32_t band_hi, std::uint32_t tall_lo,
                       bool body_en, std::int32_t cx, std::int32_t cy, std::uint32_t r,
                       bool infl,
                       const std::uint8_t lut[FPGA_M7_ROLL_LANES][FPGA_M7_ROLL_GTAB_BANK],
                       word_t out[FPGA_M7_ROLL_DELTA_CAPACITY]) {
    static std::uint16_t tri_w[MY][WX];   // 2-bit strict state x 8 lanes, [local y][physical word]
    static std::uint8_t tall_w[MY][WX];   // tall bit x 8 lanes
    // B1h-v1: one byte per cell, bits 0-1 final state, bits 2-5 forward vertical distance
    // (replaces fin / gf / gv; the backward distance is consumed on the fly)
    static std::uint8_t cg[MY][MX];

    // 1 project: one URAM word per cycle, band layers innermost.
    const std::uint8_t nb = static_cast<std::uint8_t>(band_hi - band_lo + 1);   // 1..MZ
    std::uint16_t total = static_cast<std::uint16_t>(nb * (MY * WX));
#pragma HLS BIND_OP variable=total op=mul impl=fabric
    std::uint8_t py = 0, w = 0, zi = 0;
    std::uint8_t acc_occ = 0, acc_free = 0xFF, acc_tall = 0;
    const std::uint8_t blo = static_cast<std::uint8_t>(band_lo), tlo = static_cast<std::uint8_t>(tall_lo);
    const std::uint8_t oz = static_cast<std::uint8_t>(off_z), nb1 = static_cast<std::uint8_t>(nb - 1);
grid_project:
    for (std::uint16_t i = 0; i < total; ++i) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT min=800 max=25600 avg=4800
        const std::uint8_t lz = static_cast<std::uint8_t>(blo + zi);
        const std::uint8_t pzs = static_cast<std::uint8_t>(lz + oz);
        const std::uint8_t pz = static_cast<std::uint8_t>(pzs >= MZ ? pzs - MZ : pzs);
        const word_t word = map[(pz * MY + py) * WX + w];
        std::uint8_t occ = 0, fr = 0;
        for (std::uint32_t j = 0; j < FPGA_M7_ROLL_LANES; ++j) {
#pragma HLS UNROLL
            const std::int32_t l = static_cast<std::int8_t>(
                static_cast<std::uint8_t>((word >> (8 * j)) & 0xFFu));
            const std::uint32_t st = fpga_m7_roll_state(l);
            if (st == FPGA_M7_ROLL_OCCUPIED) occ |= static_cast<std::uint8_t>(1u << j);
            if (st == FPGA_M7_ROLL_FREE) fr |= static_cast<std::uint8_t>(1u << j);
        }
        const std::uint8_t a_occ = acc_occ | occ;
        const std::uint8_t a_free = acc_free & fr;
        const std::uint8_t a_tall = lz >= tlo ? static_cast<std::uint8_t>(acc_tall | occ) : acc_tall;
        if (zi == nb1) {
            std::uint16_t t = 0;
            for (std::uint32_t j = 0; j < FPGA_M7_ROLL_LANES; ++j) {
#pragma HLS UNROLL
                std::uint16_t s = FPGA_M7_ROLL_UNKNOWN;
                if ((a_occ >> j) & 1u) s = FPGA_M7_ROLL_OCCUPIED;
                else if ((a_free >> j) & 1u) s = FPGA_M7_ROLL_FREE;
                t |= static_cast<std::uint16_t>(s << (2 * j));
            }
#ifdef FPGA_M7_ROLL_MUTANT_GRID_RING_Y
            const std::uint8_t ly = static_cast<std::uint8_t>(py + 0 * off_y);  // fault: ring offset y ignored
#else
            const std::uint8_t ly = static_cast<std::uint8_t>(py >= off_y ? py - off_y : py + MY - off_y);
#endif
            tri_w[ly][w] = t;
            tall_w[ly][w] = a_tall;
            acc_occ = 0; acc_free = 0xFF; acc_tall = 0;
            zi = 0;
            if (w == WX - 1) { w = 0; ++py; } else { ++w; }
        } else {
            acc_occ = a_occ; acc_free = a_free; acc_tall = a_tall;
            ++zi;
        }
    }

    // 2 cells (+ forward vertical scan): local order; grid word k written when its 8th
    // cell is done, the aux word one cycle later (one buffer write per cycle).
    // Body test: a cell is inside only if both closest-point distances are <= r, so
    // they are clamped before squaring (r < 2^22: exact, 22x22-bit products). B1h-v4:
    // the squares depend on the column or the row only, so they are tabulated once
    // (80 steps); "beyond r" is stored as r^2+1 so the sum test stays exact. The cells
    // loop then only adds and compares.
    const std::uint32_t rc = r + 1;
    const std::uint64_t r2 = static_cast<std::uint64_t>(r & 0x3FFFFFu) * (r & 0x3FFFFFu);
    std::uint64_t sqx[MX], sqy[MY];
#pragma HLS BIND_STORAGE variable=sqx type=ram_2p impl=lutram
#pragma HLS BIND_STORAGE variable=sqy type=ram_2p impl=lutram
    static_assert(MX == MY, "square tables share one loop");
grid_squares:
    for (std::uint8_t i = 0; i < MX; ++i) {
#pragma HLS PIPELINE II=1
        const std::int32_t v0 = static_cast<std::int32_t>(i) * G_Q;
        std::int32_t ex = 0, ey = 0;
        if (v0 - cx > 0) ex = v0 - cx;
        else if (cx - (v0 + G_Q) > 0) ex = cx - (v0 + G_Q);
        if (v0 - cy > 0) ey = v0 - cy;
        else if (cy - (v0 + G_Q) > 0) ey = cy - (v0 + G_Q);
        const std::uint32_t exc = static_cast<std::uint32_t>(ex) & 0x3FFFFFu;
        const std::uint32_t eyc = static_cast<std::uint32_t>(ey) & 0x3FFFFFu;
        sqx[i] = static_cast<std::uint32_t>(ex) < rc ? static_cast<std::uint64_t>(exc) * exc : r2 + 1;
        sqy[i] = static_cast<std::uint32_t>(ey) < rc ? static_cast<std::uint64_t>(eyc) * eyc : r2 + 1;
    }
    std::uint8_t gcol[MX];   // per-column line buffer (LUTRAM), forward then backward scan
#pragma HLS BIND_STORAGE variable=gcol type=ram_2p impl=lutram
    std::uint8_t lx = 0, ly = 0;
    word_t gacc = 0, aacc = 0;
grid_cells:
    for (std::uint16_t idx = 0; idx < G_CELLS; ++idx) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=gcol inter false
#pragma HLS DEPENDENCE variable=cg inter false
        const std::uint32_t px = g_wrap_add(lx, off_x, MX);
        const std::uint32_t pw = px >> 3, pj = px & 7u;
        const std::uint32_t strict = (tri_w[ly][pw] >> (2 * pj)) & 3u;
        const std::uint32_t tall = (tall_w[ly][pw] >> pj) & 1u;
        const bool in_body = body_en && sqx[lx] + sqy[ly] <= r2;
        std::uint32_t f = strict;
        if (in_body && f == FPGA_M7_ROLL_UNKNOWN) f = FPGA_M7_ROLL_FREE;
        const std::uint32_t prev = ly == 0 ? G_GCAP : gcol[lx];
        const std::uint32_t g = f == FPGA_M7_ROLL_OCCUPIED ? 0u : (prev + 1 > G_GCAP ? G_GCAP : prev + 1);
        gcol[lx] = static_cast<std::uint8_t>(g);
        cg[ly][lx] = static_cast<std::uint8_t>(f | (g << 2));
        const word_t gbyte = f == FPGA_M7_ROLL_OCCUPIED ? 100u : (f == FPGA_M7_ROLL_FREE ? 0u : 0xFFu);
        const word_t abyte = strict | (tall << 2);
        // shift registers: after 8 cells lane 0 holds the first one (no barrel shifter)
        const std::uint32_t j = idx & 7u;
        const word_t gnew = (gacc >> 8) | (gbyte << 56);
        const word_t anew = (aacc >> 8) | (abyte << 56);
        if (j == 7) {
            out[idx >> 3] = gnew;
        } else if (j == 0 && idx != 0) {
            out[G_PW + (idx >> 3) - 1] = aacc;   // previous aux word, complete, not yet shifted
        }
        gacc = gnew;
        aacc = anew;
        if (lx == MX - 1) { lx = 0; ++ly; } else { ++lx; }
    }
#ifndef FPGA_M7_ROLL_MUTANT_GRID_LAST_AUX
    out[2 * G_PW - 1] = aacc;
#endif
    if (!infl) return 2 * G_PW;

    // 3+4 backward vertical scan fused with the horizontal pass, rows bottom-up:
    // column x's two-sided distance enters a 25-tap window as soon as it is known;
    // output column x-RMAX = window centre (its final state rides in a 13-deep shift).
    // one 14-entry LUTRAM per table row (each row is read once per cycle by its tap)
    // instead of 196 registers (v4: 1.5k FF plus 13 register-file muxes).
    std::uint8_t tab[G_DIM][G_DIM];
#pragma HLS ARRAY_PARTITION variable=tab type=complete dim=1
#pragma HLS BIND_STORAGE variable=tab type=ram_1p impl=lutram
grid_load_lut:
    for (std::uint32_t i = 0; i < G_DIM * G_DIM; ++i) {
#pragma HLS PIPELINE II=1
        tab[i / G_DIM][i % G_DIM] = lut[i & 7u][i >> 3];
    }
    const std::uint32_t TAPS = 2 * G_RMAX + 1;
    std::uint8_t win[2 * G_RMAX + 1];
#pragma HLS ARRAY_PARTITION variable=win type=complete
    std::uint8_t fwin[G_RMAX + 1];
#pragma HLS ARRAY_PARTITION variable=fwin type=complete
    // the window is filled with G_GCAP once: each row ends with RMAX pushes of G_GCAP,
    // exactly the left padding the next row needs (no per-row reset); the cost
    // accumulator is a full 8-cell shift register, rewritten before every store.
    for (std::uint32_t t = 0; t < TAPS; ++t) {
#pragma HLS UNROLL
        win[t] = G_GCAP;
    }
    word_t cacc = 0;
grid_hor_y:
    for (std::uint8_t k = 0; k < MY; ++k) {
        const std::uint8_t y = static_cast<std::uint8_t>(MY - 1 - k);
grid_hor_x:
        for (std::uint8_t x = 0; x < MX + G_RMAX; ++x) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=gcol inter false
            std::uint8_t gin = G_GCAP, fnew = 0;
            if (x < MX) {
                const std::uint32_t c = cg[y][x];
                const std::uint32_t fx = c & 3u, gfx = c >> 2;
                const std::uint32_t prev = k == 0 ? G_GCAP : gcol[x];
                const std::uint32_t g = fx == FPGA_M7_ROLL_OCCUPIED ? 0u : (prev + 1 > G_GCAP ? G_GCAP : prev + 1);
                gcol[x] = static_cast<std::uint8_t>(g);
                gin = static_cast<std::uint8_t>((g < gfx ? g : gfx) & 15u);   // <= G_GCAP
                fnew = static_cast<std::uint8_t>(fx);
            }
            for (std::uint32_t t = 0; t + 1 < TAPS; ++t) {
#pragma HLS UNROLL
                win[t] = win[t + 1];
            }
            win[TAPS - 1] = gin;
            for (std::uint32_t t = 0; t < G_RMAX; ++t) {
#pragma HLS UNROLL
                fwin[t] = fwin[t + 1];
            }
            fwin[G_RMAX] = fnew;
            if (x >= G_RMAX) {
                const std::uint8_t cxl = static_cast<std::uint8_t>(x - G_RMAX);
                // taps +-a share table row a, which falls along g: the nearer of
                // the two columns gives the larger cost, so look up min(g-a, g+a) only.
                std::uint8_t c = 0;
                for (std::uint32_t a = 0; a <= G_RMAX; ++a) {
#pragma HLS UNROLL
                    const std::uint8_t gl = win[G_RMAX - a] & 15u, gr = win[G_RMAX + a] & 15u;
                    const std::uint8_t v = tab[a][(gl < gr ? gl : gr) & 15u];
                    if (v > c) c = v;
                }
                const std::uint32_t f = fwin[0];
                std::uint8_t o;
                if (f == FPGA_M7_ROLL_OCCUPIED) o = 254;
                else if (f == FPGA_M7_ROLL_UNKNOWN) o = c >= 253 ? c : static_cast<std::uint8_t>(255);
                else o = c;
                const std::uint32_t j = cxl & 7u;
                const word_t cnew = (cacc >> 8) | (static_cast<word_t>(o) << 56);
                if (j == 7) out[2 * G_PW + ((static_cast<std::uint32_t>(y) * MX + cxl) >> 3)] = cnew;
                cacc = cnew;
            }
        }
    }
    return 3 * G_PW;
}
#endif

}  // namespace

extern "C" void fpga_m7_rolling_core(
    std::uint32_t op, std::uint32_t epoch, std::uint32_t frame_seq,
    const std::int32_t window_base[3], const std::int32_t obs_base[3],
    std::uint32_t onx, std::uint32_t ony, std::uint32_t onz,
    bool obs_bad, std::uint16_t obs_nonzero,
    const std::uint16_t obs2[2][FPGA_M7_ROLL_OBS2_BANK],
    const std::uint8_t gtab[FPGA_M7_ROLL_LANES][FPGA_M7_ROLL_GTAB_BANK],
    std::uint32_t query_count,
    const std::uint64_t query_xyz[3 * FPGA_M7_ROLL_QUERY_CAPACITY / 2],
    std::uint64_t query_out[FPGA_M7_ROLL_QUERY_CAPACITY / 2],
    std::uint64_t delta[FPGA_M7_ROLL_DELTA_CAPACITY],
    std::uint64_t dump[FPGA_M7_ROLL_WORDS],
    std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]) {
    // Resident state. The map is on-chip (URAM), never an external port.
    static word_t map[FPGA_M7_ROLL_WORDS];
#pragma HLS BIND_STORAGE variable=map type=ram_2p impl=uram
    static bool initialized = false;
    static bool has_frame = false;
    static std::uint32_t cur_epoch = 0, delta_seq = 0, last_frame = 0;
    static std::int32_t base_x = 0, base_y = 0, base_z = 0;
    static std::uint8_t off_x = 0, off_y = 0, off_z = 0;   // ring offsets < M
    static std::uint32_t accepted = 0, rej_epoch = 0, rej_frame = 0;
    static std::uint32_t rej_malformed = 0, delta_overflow = 0;
    // FUSE_SLIM3 cut 3: the state is read once into these locals, the
    // decision chain works on them, and the statics are written back once,
    // right after it (one write site per register); the op loops then read the
    // statics. Delta overflow events of this call are counted in ovf and added
    // once before the header.
    bool st_init = initialized, st_has = has_frame;
    std::uint32_t st_epoch = cur_epoch, st_dseq = delta_seq, st_last = last_frame;
    std::int32_t st_bx = base_x, st_by = base_y, st_bz = base_z;
    std::uint8_t st_ox = off_x, st_oy = off_y, st_oz = off_z;
    std::uint32_t st_acc = accepted, st_rej_e = rej_epoch, st_rej_f = rej_frame;
    std::uint32_t st_rej_m = rej_malformed;
    std::uint16_t ovf = 0;   // <= 5,120 visits per call

#ifndef FPGA_M7_ROLL_GRID
    (void)gtab;
#endif
    std::uint32_t status = FPGA_M7_ROLL_OK;
    // Not named inside/outside: "inside" is a SystemVerilog keyword and the
    // generated cosim wrapper failed to parse it (run hls-v1, kept).
    // item 5: n_delta <= 5,120, changed <= 8 x 5,120, counts <= 32,768 (16 bits;
    // written zero-extended into the 32-bit header words as before)
    std::uint16_t n_delta = 0, changed = 0, obs_out_count = 0, obs_in_count = 0;
    std::uint8_t shift_mask = 0;
    // item 5: every coordinate range check once, shared by the ops
    const bool wb_ok = coord_ok(window_base[0]) && coord_ok(window_base[1]) &&
                       coord_ok(window_base[2]);
    const bool ob_ok01 = coord_ok(obs_base[0]) && coord_ok(obs_base[1]);
    const bool ob_ok = ob_ok01 && coord_ok(obs_base[2]);
    // item 5: one increment site for the malformed counter (RESET bad base,
    // INTEGRATE bad args, malformed block)
    bool bump_malformed = false;

    // Clear request for the one clear engine (item 3): RESET and the full clear
    // pass lz = MZ (every layer), a window move passes the evicted slices.
    bool do_clear = false;
    std::uint8_t c_sx = 0, c_lx = 0, c_sy = 0, c_ly = 0, c_sz = 0, c_lz = 0;
    bool do_fuse = false;

    if (op == FPGA_M7_ROLL_OP_RESET) {
        if (!wb_ok) {
            status = FPGA_M7_ROLL_BAD_ARGS;
            bump_malformed = true;
        } else {
            do_clear = true;
            c_lz = MZ;
            st_bx = window_base[0];
            st_by = window_base[1];
            st_bz = window_base[2];
            st_ox = st_oy = st_oz = 0;
            st_epoch = epoch;
            st_dseq = 0;
            st_last = 0;
            st_has = false;
            st_init = true;
        }
    } else if (op == FPGA_M7_ROLL_OP_INTEGRATE) {
        if (!st_init) {
            status = FPGA_M7_ROLL_UNINITIALIZED;
        } else if (onx == 0 || ony == 0 || onz == 0 ||
                   onx > FPGA_M7_ROLL_OBS_AXIS || ony > FPGA_M7_ROLL_OBS_AXIS ||
                   onz > FPGA_M7_ROLL_OBS_AXIS || !wb_ok || !ob_ok) {
            status = FPGA_M7_ROLL_BAD_ARGS;
            bump_malformed = true;
#ifndef FPGA_M7_ROLL_MUTANT_IGNORE_EPOCH
        } else if (epoch != st_epoch) {
            status = FPGA_M7_ROLL_STALE_EPOCH;
            ++st_rej_e;
#endif
        } else if (st_has && frame_seq <= st_last) {
            status = FPGA_M7_ROLL_STALE_FRAME;
            ++st_rej_f;
        } else if (obs_bad) {
            // The whole block was checked by the caller before any state
            // changes, so a malformed value anywhere rejects the frame atomically.
            status = FPGA_M7_ROLL_MALFORMED_OBS;
            bump_malformed = true;
        }

        if (status == FPGA_M7_ROLL_OK) {
            // 1. Roll the window. Both bases are inside +-2^28, so the 32-bit
            // differences are exact; after the full-clear test |d| < M (int16).
            const std::int32_t dx = window_base[0] - st_bx;
            const std::int32_t dy = window_base[1] - st_by;
            const std::int32_t dz = window_base[2] - st_bz;
            const bool full =
                dx >= static_cast<std::int32_t>(MX) || -dx >= static_cast<std::int32_t>(MX) ||
                dy >= static_cast<std::int32_t>(MY) || -dy >= static_cast<std::int32_t>(MY) ||
                dz >= static_cast<std::int32_t>(MZ) || -dz >= static_cast<std::int32_t>(MZ);
            shift_mask = (dx != 0 ? 1u : 0u) | (dy != 0 ? 2u : 0u) |
                         (dz != 0 ? 4u : 0u);
            if (full) {
                shift_mask |= 8u;
#ifndef FPGA_M7_ROLL_MUTANT_NO_EVICT
                do_clear = true;
                c_lz = MZ;
#endif
                st_ox = st_oy = st_oz = 0;
            } else if (shift_mask != 0) {
                std::uint8_t sx, lx, sy, ly, sz, lz;
                ring_move(static_cast<std::int16_t>(dx), MX, &st_ox, &sx, &lx);
                ring_move(static_cast<std::int16_t>(dy), MY, &st_oy, &sy, &ly);
                ring_move(static_cast<std::int16_t>(dz), MZ, &st_oz, &sz, &lz);
#ifndef FPGA_M7_ROLL_MUTANT_NO_EVICT
                do_clear = true;
                c_sx = sx; c_lx = lx; c_sy = sy; c_ly = ly; c_sz = sz; c_lz = lz;
#endif
            }
            st_bx = window_base[0];
            st_by = window_base[1];
            st_bz = window_base[2];

            do_fuse = true;
            ++st_dseq;
            st_last = frame_seq;
            st_has = true;
            ++st_acc;
        }
    } else if (op == FPGA_M7_ROLL_OP_QUERY) {
        if (!st_init) {
            status = FPGA_M7_ROLL_UNINITIALIZED;
        } else if (query_count > FPGA_M7_ROLL_QUERY_CAPACITY) {
            status = FPGA_M7_ROLL_BAD_ARGS;
        }
    } else if (op == FPGA_M7_ROLL_OP_DUMP) {
        if (!st_init) {
            status = FPGA_M7_ROLL_UNINITIALIZED;
        }
#ifdef FPGA_M7_ROLL_GRID
    } else if (op == FPGA_M7_ROLL_OP_GRID) {
        // Read-only: no state, counter or window change (see rolling_map.hpp).
        // The table check and the run are after the decision chain (cut 3).
        if (!st_init) status = FPGA_M7_ROLL_UNINITIALIZED;
#endif
    } else {
        status = FPGA_M7_ROLL_BAD_ARGS;
    }

    if (bump_malformed) ++st_rej_m;
    // cut 3: the one write-back of the decided state, before any loop; the
    // ops below read the statics (= the decided values).
    initialized = st_init;
    has_frame = st_has;
    cur_epoch = st_epoch;
    delta_seq = st_dseq;
    last_frame = st_last;
    base_x = st_bx;
    base_y = st_by;
    base_z = st_bz;
    off_x = st_ox;
    off_y = st_oy;
    off_z = st_oz;
    accepted = st_acc;
    rej_epoch = st_rej_e;
    rej_frame = st_rej_f;
    rej_malformed = st_rej_m;

    // FUSE_SLIM3 cut 3: the loops of the read-only ops run after the decision
    // chain (they were inside it, so HLS kept 4-6 copies of every 32-bit state
    // register across their FSM states).
    if (op == FPGA_M7_ROLL_OP_QUERY && status == FPGA_M7_ROLL_OK) {
        // item 6: read -> look up -> write in one pipeline straight on the
        // DDR views (no local query buffers): one pair of queries = three
        // u64 words in, one u64 out; the map is read twice per pair. An odd
        // count reads the next three words of the pair (inside the 3,072-word
        // buffer) and leaves the upper half of the last word unspecified.
query_loop:
        for (std::uint16_t qp = 0; qp < static_cast<std::uint16_t>((query_count + 1) / 2); ++qp) {
#pragma HLS PIPELINE II=3
#pragma HLS LOOP_TRIPCOUNT max=512
            const std::uint64_t w0 = query_xyz[3 * qp], w1 = query_xyz[3 * qp + 1];
            const std::uint64_t w2 = query_xyz[3 * qp + 2];
            const std::int32_t q[2][3] = {
                {static_cast<std::int32_t>(static_cast<std::uint32_t>(w0)),
                 static_cast<std::int32_t>(static_cast<std::uint32_t>(w0 >> 32)),
                 static_cast<std::int32_t>(static_cast<std::uint32_t>(w1))},
                {static_cast<std::int32_t>(static_cast<std::uint32_t>(w1 >> 32)),
                 static_cast<std::int32_t>(static_cast<std::uint32_t>(w2)),
                 static_cast<std::int32_t>(static_cast<std::uint32_t>(w2 >> 32))}};
            std::uint32_t res[2];
            for (std::uint32_t h = 0; h < 2; ++h) {
#pragma HLS UNROLL
                const std::int32_t qx = q[h][0], qy = q[h][1], qz = q[h][2];
                // item 5: q - base in [0, M)  <=>  base <= q < base + M (base
                // within +-2^28, so base + M cannot overflow): two 32-bit
                // compares instead of a 64-bit subtract; the local index is
                // the low 8 bits of the wrapping 32-bit difference.
                std::uint32_t result = 1u << 16;
                if (qx >= base_x && qx < base_x + static_cast<std::int32_t>(MX) &&
                    qy >= base_y && qy < base_y + static_cast<std::int32_t>(MY) &&
                    qz >= base_z && qz < base_z + static_cast<std::int32_t>(MZ)) {
                    const std::uint8_t lxq = static_cast<std::uint8_t>(
                        static_cast<std::uint32_t>(qx) - static_cast<std::uint32_t>(base_x));
                    const std::uint8_t lyq = static_cast<std::uint8_t>(
                        static_cast<std::uint32_t>(qy) - static_cast<std::uint32_t>(base_y));
                    const std::uint8_t lzq = static_cast<std::uint8_t>(
                        static_cast<std::uint32_t>(qz) - static_cast<std::uint32_t>(base_z));
                    const std::uint8_t px = ring_add(lxq, off_x, MX);
                    const std::uint8_t py = ring_add(lyq, off_y, MY);
                    const std::uint8_t pz = ring_add(lzq, off_z, MZ);
                    const word_t word = map[row_word(pz, py) + px / FPGA_M7_ROLL_LANES];
                    const std::uint32_t raw = static_cast<std::uint32_t>(
                        (word >> (8 * (px % FPGA_M7_ROLL_LANES))) & 0xFFu);
                    const std::int32_t l = static_cast<std::int8_t>(static_cast<std::uint8_t>(raw));
                    result = fpga_m7_roll_state(l) | (raw << 8);
                }
                res[h] = result;
            }
            query_out[qp] = static_cast<std::uint64_t>(res[0]) | (static_cast<std::uint64_t>(res[1]) << 32);
        }
    }
    if (op == FPGA_M7_ROLL_OP_DUMP && status == FPGA_M7_ROLL_OK) {
dump_loop:
        for (std::uint32_t i = 0; i < WORDS; ++i) {
#pragma HLS PIPELINE II=1
            dump[i] = map[i];
        }
    }
#ifdef FPGA_M7_ROLL_GRID
    if (op == FPGA_M7_ROLL_OP_GRID && status == FPGA_M7_ROLL_OK) {
        const std::uint32_t body_r = static_cast<std::uint32_t>(obs_base[2]);
        const std::uint32_t why = grid_check(onx, ony, onz, ob_ok01, body_r,
                                             query_count, gtab);
        if (why != 0) {
            status = FPGA_M7_ROLL_BAD_ARGS;
            changed = static_cast<std::uint16_t>(why);
        } else {
            n_delta = static_cast<std::uint16_t>(grid_run(map, off_x, off_y, off_z, onx, ony, onz,
                               (query_count & 1u) != 0, obs_base[0], obs_base[1], body_r,
                               (query_count & 2u) != 0, gtab, delta));
        }
    }
#endif

    // Window clear (one engine, item 3), then the fusion of an accepted block.
    if (do_clear) clear_slices(map, c_sx, c_lx, c_sy, c_ly, c_sz, c_lz);
    if (do_fuse) {
#if FPGA_M7_ROLL_MX > 32
        // 2. Fuse the block (SLIM_FUSE2 item 4). local = observation index + e.
        // Only rows inside the window and, per row, only the physical words that
        // hold a cell of the block are visited: one (row, word) pair per cycle in
        // one II=1 pipeline (the old code spent 32 cycles loading every row and
        // then walked all WX words). Order = old order: rows by (oz, oy), words by
        // ascending physical index; a skipped word has no observed cell, so the
        // old code produced no change and no delta record for it.
        // item 5: offsets clamped to [-64, 127] (clamp_off), set-up in 16 bits
        const std::int16_t ex = clamp_off(obs_base[0] - base_x);
        const std::int16_t ey = clamp_off(obs_base[1] - base_y);
        const std::int16_t ez = clamp_off(obs_base[2] - base_z);
        // block rows inside the window: oy in [ya, yb), oz in [za, zb)
        const std::int16_t ny = static_cast<std::int16_t>(ony), nz = static_cast<std::int16_t>(onz);
        const std::int16_t ya = ey < 0 ? (-ey < ny ? static_cast<std::int16_t>(-ey) : ny) : 0;
        const std::int16_t ym = static_cast<std::int16_t>(static_cast<std::int16_t>(MY) - ey);
        const std::int16_t yb = ym < ny ? (ym > ya ? ym : ya) : ny;
        const std::int16_t za = ez < 0 ? (-ez < nz ? static_cast<std::int16_t>(-ez) : nz) : 0;
        const std::int16_t zm = static_cast<std::int16_t>(static_cast<std::int16_t>(MZ) - ez);
        const std::int16_t zb = zm < nz ? (zm > za ? zm : za) : nz;
        // block cells inside the window along x: local [xa, xb)
        const std::int16_t xa = ex > 0 ? ex : 0;
        const std::int16_t xe = static_cast<std::int16_t>(ex + static_cast<std::int16_t>(onx));
        const std::int16_t xb = xe < static_cast<std::int16_t>(MX) ? xe : static_cast<std::int16_t>(MX);
        // Word list, ascending physical index: word w, lane mask (lanes whose
        // local x is in [xa, xb)), and cell offset of lane 0 relative to the row
        // start (lane j reads cell row_start + off + j). A word that straddles the
        // ring seam (local MX-1 | 0) has its observed lanes on one side only (the
        // block is <= 32 < MX cells wide), so one offset serves the word.
        std::uint8_t wl_w[FUSE_MAX_WORDS], wl_m[FUSE_MAX_WORDS];
        std::int8_t wl_o[FUSE_MAX_WORDS];
#pragma HLS ARRAY_PARTITION variable=wl_w complete
#pragma HLS ARRAY_PARTITION variable=wl_m complete
#pragma HLS ARRAY_PARTITION variable=wl_o complete
        std::uint8_t n_w = 0;
fuse_words:
        for (std::uint8_t w = 0; w < WX; ++w) {
#pragma HLS PIPELINE II=1
            std::uint8_t m = 0;
            bool wrap_side = false;
            for (std::uint32_t j = 0; j < FPGA_M7_ROLL_LANES; ++j) {
#pragma HLS UNROLL
                const std::int16_t p = static_cast<std::int16_t>(w * FPGA_M7_ROLL_LANES + j);
                const std::int16_t o = off_x;
                const std::int16_t lxv = static_cast<std::int16_t>(
                    p >= o ? p - o : p + static_cast<std::int16_t>(MX) - o);
                if (xa < xb && lxv >= xa && lxv < xb) {
                    m |= static_cast<std::uint8_t>(1u << j);
                    if (p < o) wrap_side = true;
                }
            }
            if (m != 0 && n_w < FUSE_MAX_WORDS) {
                const std::int16_t o0 = static_cast<std::int16_t>(
                    static_cast<std::int16_t>(w * FPGA_M7_ROLL_LANES) - static_cast<std::int16_t>(off_x) +
                    (wrap_side ? static_cast<std::int16_t>(MX) : 0) - ex);
                wl_w[n_w] = w;
                wl_m[n_w] = m;
                wl_o[n_w] = static_cast<std::int8_t>(o0);
                ++n_w;
            }
        }
        // FUSE_SLIM3 cut 1: the set-up products at their real widths. Here
        // 1 <= onx, ony, onz <= 32 (checked above), 0 <= ya <= yb <= ony and
        // 0 <= za <= zb <= onz, so every factor fits 6 bits (n_w <= 5: 3 bits);
        // the masks only tell HLS the ranges (each masked value is unchanged).
        // Products: n_rows <= 1,024, n_it <= 5,120, rs0 <= 33,792 (za <= 32
        // only when no row is visited), rs_skip <= 33 x 32: all exact in 16 bits,
        // as before (were 16 x 32-bit fabric multipliers, ~390 LUT).
        const std::uint8_t onx6 = static_cast<std::uint8_t>(onx & 63u);
        const std::uint8_t ony6 = static_cast<std::uint8_t>(ony & 63u);
        const std::uint8_t ya6 = static_cast<std::uint8_t>(ya & 63);
        const std::uint8_t za6 = static_cast<std::uint8_t>(za & 63);
        const std::uint8_t nrow_y = static_cast<std::uint8_t>((yb - ya) & 63);
        const std::uint8_t nrow_z = static_cast<std::uint8_t>((zb - za) & 63);
        std::uint16_t n_rows = static_cast<std::uint16_t>(nrow_y * nrow_z);
#pragma HLS BIND_OP variable=n_rows op=mul impl=fabric
        std::uint16_t n_it = static_cast<std::uint16_t>(n_rows * (n_w & 7u));
#pragma HLS BIND_OP variable=n_it op=mul impl=fabric
        // row start in the block (cells): (oz*ony + oy)*onx, stepped by counters
        const std::uint16_t zrow = static_cast<std::uint16_t>(za6 * ony6);
#pragma HLS BIND_OP variable=zrow op=mul impl=fabric
        std::uint16_t rs0 = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>((zrow + ya6) & 2047u) * onx6);
#pragma HLS BIND_OP variable=rs0 op=mul impl=fabric
        std::uint16_t rs_skip = static_cast<std::uint16_t>(
            static_cast<std::uint8_t>((ony6 - nrow_y + 1) & 63u) * onx6);
#pragma HLS BIND_OP variable=rs_skip op=mul impl=fabric
        std::uint16_t rs = rs0;
        std::uint8_t oy = static_cast<std::uint8_t>(ya), oz = static_cast<std::uint8_t>(za);
        std::uint8_t ly = static_cast<std::uint8_t>(ya + ey), lz = static_cast<std::uint8_t>(za + ez);
        const std::uint8_t py0 = ring_add(static_cast<std::uint8_t>(ya + ey), static_cast<std::uint8_t>(off_y), MY);
        std::uint8_t py = py0;
        std::uint8_t pz = ring_add(static_cast<std::uint8_t>(za + ez), static_cast<std::uint8_t>(off_z), MZ);
        std::uint8_t k = 0;
        std::uint16_t in_cnt = 0;
fuse_loop:
        for (std::uint16_t it = 0; it < n_it; ++it) {
#pragma HLS PIPELINE II=1
#pragma HLS LOOP_TRIPCOUNT max=5120
#pragma HLS DEPENDENCE variable=map inter false
            const std::uint8_t w = wl_w[k], lm = wl_m[k];
            // 8 consecutive block cells from cell c (item 6: 2-bit cells): entries
            // c/8 and c/8+1 sit in different banks (entry k in bank k % 2); the two
            // 16-bit entries are joined and shifted by c % 8 cells. c < 0 or past the
            // block only feeds lanes that the lane mask drops.
            const std::int16_t c = static_cast<std::int16_t>(static_cast<std::int16_t>(rs) + wl_o[k]);
            const std::uint16_t k0 = static_cast<std::uint16_t>(c >> 3) & (FUSE_ENTRIES - 1);
            const std::uint16_t k1 = static_cast<std::uint16_t>(k0 + 1) & (FUSE_ENTRIES - 1);
            const bool k0_odd = (k0 & 1u) != 0;
            const std::uint16_t ve = obs2[0][(k0_odd ? k1 : k0) >> 1];
            const std::uint16_t vo = obs2[1][(k0_odd ? k0 : k1) >> 1];
            const std::uint32_t pair = k0_odd ? (static_cast<std::uint32_t>(ve) << 16) | vo
                                              : (static_cast<std::uint32_t>(vo) << 16) | ve;
            const std::uint16_t cells = static_cast<std::uint16_t>(pair >> (2 * (c & 7)));
            const word_t old = map[row_word(pz, py) + w];
            word_t next = 0;
            std::uint32_t cmask = 0, states = 0, ncount = 0, nin = 0;
            for (std::uint32_t j = 0; j < FPGA_M7_ROLL_LANES; ++j) {
#pragma HLS UNROLL
                const std::uint32_t v = ((lm >> j) & 1u) ? ((cells >> (2 * j)) & 3u) : 0u;
                if (v != 0) ++nin;
                const std::int32_t l = static_cast<std::int8_t>(
                    static_cast<std::uint8_t>((old >> (8 * j)) & 0xFFu));
                const std::int32_t nl = fpga_m7_roll_update(l, v);
                const std::uint32_t os = fpga_m7_roll_state(l);
                const std::uint32_t ns = fpga_m7_roll_state(nl);
                next |= static_cast<word_t>(static_cast<std::uint8_t>(nl)) << (8 * j);
                states |= ns << (2 * j);
                if (os != ns) { cmask |= 1u << j; ++ncount; }
            }
            in_cnt = static_cast<std::uint16_t>(in_cnt + nin);
            if (next != old) map[row_word(pz, py) + w] = next;
            if (cmask != 0) {
                if (n_delta < FPGA_M7_ROLL_DELTA_CAPACITY) {
                    delta[n_delta] = static_cast<word_t>(states) |
                        (static_cast<word_t>(cmask) << 16) |
                        (static_cast<word_t>(w) << 24) |
                        (static_cast<word_t>(ly) << 32) |
                        (static_cast<word_t>(lz) << 40);
                    ++n_delta;
                } else {
                    ++ovf;
                }
                changed += ncount;
            }
            // next (row, word)
            if (k == n_w - 1) {
                k = 0;
                if (oy == yb - 1) {
                    oy = static_cast<std::uint8_t>(ya);
                    ly = static_cast<std::uint8_t>(ya + ey);
                    py = py0;
                    ++oz; ++lz;
                    pz = ring_add(pz, 1, MZ);
                    rs = static_cast<std::uint16_t>(rs + rs_skip);
                } else {
                    ++oy; ++ly;
                    py = ring_add(py, 1, MY);
                    rs = static_cast<std::uint16_t>(rs + onx);
                }
            } else {
                ++k;
            }
        }
        // every observed (non-zero) cell of the block is either inside the
        // window (counted on its visited lane) or outside
        obs_in_count = in_cnt;
        obs_out_count = obs_nonzero - in_cnt;
#else
        // Windows not wider than a block row (test builds only, e.g. MX = 16):
        // a word at the ring seam can hold observed cells from both sides, so the
        // one-offset word fetch above does not apply; the pre-SLIM_FUSE2 row loop
        // (32-cycle row load + all WX words) is kept for them.
        (void)obs_nonzero;
        // 2. Fuse the block. local = observation index + e.
        const std::int32_t ex = obs_base[0] - base_x;
        const std::int32_t ey = obs_base[1] - base_y;
        const std::int32_t ez = obs_base[2] - base_z;
        std::uint32_t row_start = 0;
row_z_loop:
        for (std::uint32_t oz = 0; oz < onz; ++oz) {
#pragma HLS LOOP_TRIPCOUNT max=32
row_y_loop:
            for (std::uint32_t oy = 0; oy < ony; ++oy) {
#pragma HLS LOOP_TRIPCOUNT max=32
                std::uint8_t row[FPGA_M7_ROLL_OBS_AXIS];
#pragma HLS ARRAY_PARTITION variable=row complete
                const std::int32_t ly = static_cast<std::int32_t>(oy) + ey;
                const std::int32_t lz = static_cast<std::int32_t>(oz) + ez;
                const bool row_in = ly >= 0 && ly < static_cast<std::int32_t>(MY) &&
                                    lz >= 0 && lz < static_cast<std::int32_t>(MZ);
row_load_loop:
                for (std::uint32_t ox = 0; ox < FPGA_M7_ROLL_OBS_AXIS; ++ox) {
#pragma HLS PIPELINE II=1
                    const std::uint8_t v = ox < onx ? static_cast<std::uint8_t>(obs2_cell(obs2, row_start + ox)) : 0;
                    row[ox] = v;
                    if (v != 0) {
                        const std::int32_t lxv = static_cast<std::int32_t>(ox) + ex;
                        if (row_in && lxv >= 0 && lxv < static_cast<std::int32_t>(MX))
                            ++obs_in_count;
                        else
                            ++obs_out_count;
                    }
                }
                row_start += onx;
                if (!row_in) continue;
                std::uint32_t py = static_cast<std::uint32_t>(ly) + off_y;
                if (py >= MY) py -= MY;
                std::uint32_t pz = static_cast<std::uint32_t>(lz) + off_z;
                if (pz >= MZ) pz -= MZ;
                const std::uint32_t base = row_word(static_cast<std::uint8_t>(pz),
                                                    static_cast<std::uint8_t>(py));
row_word_loop:
                for (std::uint32_t w = 0; w < WX; ++w) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=map inter false
                    const word_t old = map[base + w];
                    word_t next = 0;
                    std::uint32_t cmask = 0, states = 0, ncount = 0;
                    for (std::uint32_t j = 0; j < FPGA_M7_ROLL_LANES; ++j) {
#pragma HLS UNROLL
                        std::int32_t lxv = static_cast<std::int32_t>(w * FPGA_M7_ROLL_LANES + j) -
                                           static_cast<std::int32_t>(off_x);
                        if (lxv < 0) lxv += static_cast<std::int32_t>(MX);
                        const std::int32_t ox = lxv - ex;
                        const std::uint32_t v =
                            (ox >= 0 && ox < static_cast<std::int32_t>(onx))
                            ? row[static_cast<std::uint32_t>(ox) & (FPGA_M7_ROLL_OBS_AXIS - 1)]
                            : 0u;
                        const std::int32_t l = static_cast<std::int8_t>(
                            static_cast<std::uint8_t>((old >> (8 * j)) & 0xFFu));
                        const std::int32_t nl = fpga_m7_roll_update(l, v);
                        const std::uint32_t os = fpga_m7_roll_state(l);
                        const std::uint32_t ns = fpga_m7_roll_state(nl);
                        next |= static_cast<word_t>(static_cast<std::uint8_t>(nl)) << (8 * j);
                        states |= ns << (2 * j);
                        if (os != ns) { cmask |= 1u << j; ++ncount; }
                    }
                    if (next != old) map[base + w] = next;
                    if (cmask != 0) {
                        if (n_delta < FPGA_M7_ROLL_DELTA_CAPACITY) {
                            delta[n_delta] = static_cast<word_t>(states) |
                                (static_cast<word_t>(cmask) << 16) |
                                (static_cast<word_t>(w) << 24) |
                                (static_cast<word_t>(ly) << 32) |
                                (static_cast<word_t>(lz) << 40);
                            ++n_delta;
                        } else {
                            ++ovf;
                        }
                        changed += ncount;
                    }
                }
            }
        }
#endif
    }

    delta_overflow = delta_overflow + ovf;

    header[FPGA_M7_ROLL_H_STATUS] = status;
    header[FPGA_M7_ROLL_H_OP] = op;
    header[FPGA_M7_ROLL_H_EPOCH] = cur_epoch;
    header[FPGA_M7_ROLL_H_DELTA_SEQ] = delta_seq;
    header[FPGA_M7_ROLL_H_LAST_FRAME] = last_frame;
    header[FPGA_M7_ROLL_H_BASE + 0] = static_cast<std::uint32_t>(base_x);
    header[FPGA_M7_ROLL_H_BASE + 1] = static_cast<std::uint32_t>(base_y);
    header[FPGA_M7_ROLL_H_BASE + 2] = static_cast<std::uint32_t>(base_z);
    header[FPGA_M7_ROLL_H_OFFSET + 0] = off_x;
    header[FPGA_M7_ROLL_H_OFFSET + 1] = off_y;
    header[FPGA_M7_ROLL_H_OFFSET + 2] = off_z;
    header[FPGA_M7_ROLL_H_N_DELTA] = n_delta;
    header[FPGA_M7_ROLL_H_CHANGED_CELLS] = changed;
    header[FPGA_M7_ROLL_H_OBS_OUTSIDE] = obs_out_count;
    header[FPGA_M7_ROLL_H_OBS_INSIDE] = obs_in_count;
    header[FPGA_M7_ROLL_H_SHIFT] = shift_mask;
    header[FPGA_M7_ROLL_H_ACCEPTED] = accepted;
    header[FPGA_M7_ROLL_H_REJ_EPOCH] = rej_epoch;
    header[FPGA_M7_ROLL_H_REJ_FRAME] = rej_frame;
    header[FPGA_M7_ROLL_H_REJ_MALFORMED] = rej_malformed;
    header[FPGA_M7_ROLL_H_MAGIC] = FPGA_M7_ROLL_MAGIC;
    header[FPGA_M7_ROLL_H_DIMS] = MX | (MY << 8) | (MZ << 16);
    header[FPGA_M7_ROLL_H_HAS_FRAME] = (initialized ? 2u : 0u) | (has_frame ? 1u : 0u);
    header[FPGA_M7_ROLL_H_RESERVED] = delta_overflow;
}

#ifndef __SYNTHESIS__
// C entry (host builds, testbenches): the 0/1/2 check of the block, then the core.
extern "C" void fpga_m7_rolling_map(
    std::uint32_t op, std::uint32_t epoch, std::uint32_t frame_seq,
    const std::int32_t window_base[3], const std::int32_t obs_base[3],
    std::uint32_t onx, std::uint32_t ony, std::uint32_t onz,
    const std::uint8_t obs[FPGA_M7_ROLL_OBS_CAPACITY],
    std::uint32_t query_count,
    const std::int32_t query_xyz[3 * FPGA_M7_ROLL_QUERY_CAPACITY],
    std::uint32_t query_out[FPGA_M7_ROLL_QUERY_CAPACITY],
    std::uint64_t delta[FPGA_M7_ROLL_DELTA_CAPACITY],
    std::uint64_t dump[FPGA_M7_ROLL_WORDS],
    std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]) {
    bool bad = false;
    std::uint32_t nonzero = 0;
    if (op == FPGA_M7_ROLL_OP_INTEGRATE && onx >= 1 && onx <= FPGA_M7_ROLL_OBS_AXIS &&
        ony >= 1 && ony <= FPGA_M7_ROLL_OBS_AXIS && onz >= 1 && onz <= FPGA_M7_ROLL_OBS_AXIS) {
        const std::uint32_t total = onx * ony * onz;
        for (std::uint32_t i = 0; i < total; ++i) {
            if (obs[i] > FPGA_M7_ROLL_OCCUPIED) bad = true;
            if (obs[i] != 0) ++nonzero;
        }
    }
    // the core's input layouts (as the AXI shell builds them): 2-bit cells
    // (entry k = cells 8k..8k+7 in bank k % 2), the GRID table bytes by lane,
    // query coordinates / results as u64 pairs
    static std::uint16_t obs2[2][FPGA_M7_ROLL_OBS2_BANK];
    static std::uint8_t gtab[FPGA_M7_ROLL_LANES][FPGA_M7_ROLL_GTAB_BANK];
    static std::uint64_t qin[3 * FPGA_M7_ROLL_QUERY_CAPACITY / 2], qout[FPGA_M7_ROLL_QUERY_CAPACITY / 2];
    for (std::uint32_t k = 0; k < FPGA_M7_ROLL_OBS_CAPACITY / 8; ++k) {
        std::uint16_t e = 0;
        for (std::uint32_t j = 0; j < 8; ++j) e = static_cast<std::uint16_t>(e | ((obs[8 * k + j] & 3u) << (2 * j)));
        obs2[k & 1u][k >> 1] = e;
    }
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_LANES * FPGA_M7_ROLL_GTAB_BANK; ++i) gtab[i & 7u][i >> 3] = obs[i];
    for (std::uint32_t i = 0; i < 3 * FPGA_M7_ROLL_QUERY_CAPACITY / 2; ++i)
        qin[i] = static_cast<std::uint64_t>(static_cast<std::uint32_t>(query_xyz[2 * i])) |
                 (static_cast<std::uint64_t>(static_cast<std::uint32_t>(query_xyz[2 * i + 1])) << 32);
    fpga_m7_rolling_core(op, epoch, frame_seq, window_base, obs_base, onx, ony, onz, bad,
                        static_cast<std::uint16_t>(nonzero), obs2, gtab,
                        query_count, qin, qout, delta, dump, header);
    if (op == FPGA_M7_ROLL_OP_QUERY && header[FPGA_M7_ROLL_H_STATUS] == FPGA_M7_ROLL_OK)
        for (std::uint32_t i = 0; i < query_count; ++i)
            query_out[i] = static_cast<std::uint32_t>(qout[i / 2] >> (32 * (i & 1u)));
}

extern "C" void fpga_m7_rolling_config(std::uint32_t out[16]) {
    out[0] = FPGA_M7_ROLL_DIM_X;
    out[1] = FPGA_M7_ROLL_DIM_Y;
    out[2] = FPGA_M7_ROLL_DIM_Z;
    out[3] = FPGA_M7_ROLL_LANES;
    out[4] = FPGA_M7_ROLL_WORDS;
    out[5] = FPGA_M7_ROLL_OBS_AXIS;
    out[6] = FPGA_M7_ROLL_DELTA_CAPACITY;
    out[7] = FPGA_M7_ROLL_QUERY_CAPACITY;
    out[8] = FPGA_M7_ROLL_HEADER_WORDS;
    out[9] = static_cast<std::uint32_t>(FPGA_M7_ROLL_HIT);
    out[10] = static_cast<std::uint32_t>(FPGA_M7_ROLL_MISS);
    out[11] = static_cast<std::uint32_t>(FPGA_M7_ROLL_LMAX);
    out[12] = static_cast<std::uint32_t>(FPGA_M7_ROLL_LMIN);
    out[13] = static_cast<std::uint32_t>(FPGA_M7_ROLL_T_OCC);
    out[14] = static_cast<std::uint32_t>(FPGA_M7_ROLL_T_FREE);
    out[15] = fpga_m7_roll_mutant_mask();
}
#endif
