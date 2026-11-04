// A2 resource / II probe (csynth only, host; not a product source, not
// bit-checked here -- the bit-identity evidence is tools/m7_lanes/bitexact.py).
//
// Structure proposed for A2 (docs/design/M7_RAY_LANES_A2_DESIGN_20261011.md
// @5bf7115) -> round-robin dispatcher -> NL lanes, each an explicitly
// instantiated walk process with its own two 32Kx1 bit planes -> after the
// DATAFLOW region an OR merge writing one flag byte per cycle.
// The walk loop body follows the A1 design (one crossing per iteration,
// incremental pairwise differences); the linear index is updated by stride
// instead of the A1 multiply (both are listed as options in the design doc).
//
// Build: vitis_hls -f run_probe.tcl with PROBE_LANES=<1|2|4|8|16>.
#include <ap_int.h>
#include <hls_stream.h>

#ifndef PROBE_LANES
#define PROBE_LANES 4
#endif
#define NL PROBE_LANES

const int kCap = 32768;
const unsigned kMaxSteps = 96;
typedef ap_int<44> cross_t;
typedef ap_uint<33> d256_t;
typedef ap_int<7> cell_t;

struct Ray {
    cross_t pxy, pxz, pyz;
    d256_t dx, dy, dz;
    cell_t tx, ty, tz;
    ap_int<2> sx, sy, sz;
    ap_uint<15> tidx;
    bool tin;
    bool last;
};

static void dispatch(hls::stream<Ray>& in, hls::stream<Ray> lane_s[NL]) {
    ap_uint<5> k = 0;
dispatch_loop:
    for (;;) {
#pragma HLS PIPELINE II=1
        const Ray r = in.read();
        if (r.last) break;
        lane_s[k].write(r);
        k = (k == NL - 1) ? ap_uint<5>(0) : ap_uint<5>(k + 1);
    }
    Ray s;
    s.pxy = 0; s.pxz = 0; s.pyz = 0; s.dx = 0; s.dy = 0; s.dz = 0;
    s.tx = 0; s.ty = 0; s.tz = 0; s.sx = 0; s.sy = 0; s.sz = 0;
    s.tidx = 0; s.tin = false; s.last = true;
sentinel_loop:
    for (int i = 0; i < NL; ++i) {
#pragma HLS UNROLL
        lane_s[i].write(s);  // every lane gets its own end-of-frame marker
    }
}

template <int ID>
static void walk_lane(hls::stream<Ray>& in, ap_uint<1> free_pl[kCap], ap_uint<1> hit_pl[kCap],
                      cell_t ox, cell_t oy, cell_t oz, cell_t dimx, cell_t dimy, cell_t dimz,
                      ap_uint<15> o_idx, ap_uint<15> sty_, ap_uint<15> stz_) {
#pragma HLS INLINE off
    cell_t cx = 0, cy = 0, cz = 0, tx = 0, ty = 0, tz = 0;
    ap_int<2> sx = 0, sy = 0, sz = 0;
    cross_t pxy = 0, pxz = 0, pyz = 0;
    d256_t dx = 0, dy = 0, dz = 0;
    ap_uint<15> tidx = 0, idx = 0;
    bool tin = false, busy = false, done = false;
    ap_uint<7> steps = 0;
walk_loop:
    while (!done) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=free_pl type=inter false
#pragma HLS DEPENDENCE variable=hit_pl type=inter false
        if (!busy) {
            const Ray r = in.read();
            if (r.last) {
                done = true;
            } else {
                cx = ox; cy = oy; cz = oz; idx = o_idx;
                tx = r.tx; ty = r.ty; tz = r.tz;
                sx = r.sx; sy = r.sy; sz = r.sz;
                pxy = r.pxy; pxz = r.pxz; pyz = r.pyz;
                dx = r.dx; dy = r.dy; dz = r.dz;
                tidx = r.tidx; tin = r.tin;
                steps = 0;
                busy = true;
            }
        }
        if (busy) {
            const bool ax = sx != 0 && cx != tx;
            const bool ay = sy != 0 && cy != ty;
            const bool az = sz != 0 && cz != tz;
            bool end = false;
            if (!ax && !ay && !az) {
                end = true;
            } else {
                const bool stx = ax && (!ay || pxy <= 0) && (!az || pxz <= 0);
                const bool sty = ay && (!ax || pxy >= 0) && (!az || pyz <= 0);
                const bool stz = az && (!ax || pxz >= 0) && (!ay || pyz >= 0);
                cross_t nxy = pxy, nxz = pxz, nyz = pyz;
                if (stx) { nxy += dy; nxz += dz; }
                if (sty) { nxy -= dx; nyz += dz; }
                if (stz) { nxz -= dx; nyz -= dy; }
                pxy = nxy; pxz = nxz; pyz = nyz;
                ap_uint<15> ni = idx;
                if (stx) { cx += sx; ni += (sx > 0) ? ap_uint<15>(1) : ap_uint<15>(-1); }
                if (sty) { cy += sy; ni += (sy > 0) ? sty_ : ap_uint<15>(-sty_); }
                if (stz) { cz += sz; ni += (sz > 0) ? stz_ : ap_uint<15>(-stz_); }
                idx = ni;
                ++steps;
                const bool in_grid = cx >= 0 && cx < dimx && cy >= 0 && cy < dimy && cz >= 0 && cz < dimz;
                if (!in_grid) {
                    end = true;
                } else if (cx == tx && cy == ty && cz == tz) {
                    end = true;
                } else {
                    free_pl[ni] = 1;
                    if (steps == kMaxSteps) end = true;
                }
            }
            if (end) {
                if (tin) hit_pl[tidx] = 1;
                busy = false;
            }
        }
    }
}

// Same walk, written so that every state register gets exactly one value per
// iteration through explicit "loaded ? record : register" selects (no
// conditionally loaded state). Probe run 3 (variant A, A1-style body above)
// showed Vivado inferring 175 latches (LDCE) per lane from the HLS phi muxes
// of the "load in one branch, step in the next" structure; this form is the
// candidate fix. Selected with -DPROBE_WALK_MUX.
template <int ID>
static void walk_lane_mux(hls::stream<Ray>& in, ap_uint<1> free_pl[kCap], ap_uint<1> hit_pl[kCap],
                          cell_t ox, cell_t oy, cell_t oz, cell_t dimx, cell_t dimy, cell_t dimz,
                          ap_uint<15> o_idx, ap_uint<15> sty_, ap_uint<15> stz_) {
#pragma HLS INLINE off
    cell_t cx = 0, cy = 0, cz = 0, tx = 0, ty = 0, tz = 0;
    ap_int<2> sx = 0, sy = 0, sz = 0;
    cross_t pxy = 0, pxz = 0, pyz = 0;
    d256_t dx = 0, dy = 0, dz = 0;
    ap_uint<15> tidx = 0, idx = 0;
    bool tin = false, busy = false, done = false;
    ap_uint<7> steps = 0;
walk_loop:
    while (!done) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=free_pl type=inter false
#pragma HLS DEPENDENCE variable=hit_pl type=inter false
        Ray r;
        r.pxy = 0; r.pxz = 0; r.pyz = 0; r.dx = 0; r.dy = 0; r.dz = 0;
        r.tx = 0; r.ty = 0; r.tz = 0; r.sx = 0; r.sy = 0; r.sz = 0;
        r.tidx = 0; r.tin = false; r.last = false;
        if (!busy) r = in.read();
        const bool ld = !busy && !r.last;
        const bool fin = !busy && r.last;
        // current state: one select per register
        const cell_t ccx = ld ? ox : cx, ccy = ld ? oy : cy, ccz = ld ? oz : cz;
        const cell_t ttx = ld ? r.tx : tx, tty = ld ? r.ty : ty, ttz = ld ? r.tz : tz;
        const ap_int<2> ssx = ld ? r.sx : sx, ssy = ld ? r.sy : sy, ssz = ld ? r.sz : sz;
        const cross_t qxy = ld ? r.pxy : pxy, qxz = ld ? r.pxz : pxz, qyz = ld ? r.pyz : pyz;
        const d256_t ddx = ld ? r.dx : dx, ddy = ld ? r.dy : dy, ddz = ld ? r.dz : dz;
        const ap_uint<15> ttidx = ld ? r.tidx : tidx, cidx = ld ? o_idx : idx;
        const bool ttin = ld ? r.tin : tin;
        const ap_uint<7> cst = ld ? ap_uint<7>(0) : steps;
        const bool act = busy || ld;
        const bool ax = ssx != 0 && ccx != ttx;
        const bool ay = ssy != 0 && ccy != tty;
        const bool az = ssz != 0 && ccz != ttz;
        const bool none = !ax && !ay && !az;
        const bool stx = ax && (!ay || qxy <= 0) && (!az || qxz <= 0);
        const bool sty = ay && (!ax || qxy >= 0) && (!az || qyz <= 0);
        const bool stz = az && (!ax || qxz >= 0) && (!ay || qyz >= 0);
        cross_t nxy = qxy, nxz = qxz, nyz = qyz;
        if (stx) { nxy += ddy; nxz += ddz; }
        if (sty) { nxy -= ddx; nyz += ddz; }
        if (stz) { nxz -= ddx; nyz -= ddy; }
        cell_t ncx = ccx, dev3 = ccy, ncz = ccz;
        ap_uint<15> ni = cidx;
        if (stx) { ncx += ssx; ni += (ssx > 0) ? ap_uint<15>(1) : ap_uint<15>(-1); }
        if (sty) { dev3 += ssy; ni += (ssy > 0) ? sty_ : ap_uint<15>(-sty_); }
        if (stz) { ncz += ssz; ni += (ssz > 0) ? stz_ : ap_uint<15>(-stz_); }
        const ap_uint<7> nst = cst + 1;
        const bool in_grid = ncx >= 0 && ncx < dimx && dev3 >= 0 && dev3 < dimy && ncz >= 0 && ncz < dimz;
        const bool at_t = ncx == ttx && dev3 == tty && ncz == ttz;
        const bool mark = act && !none && in_grid && !at_t;
        const bool end = act && (none || !in_grid || at_t || nst == kMaxSteps);
        if (mark) free_pl[ni] = 1;
        if (end && ttin) hit_pl[ttidx] = 1;
        cx = ncx; cy = dev3; cz = ncz; tx = ttx; ty = tty; tz = ttz;
        sx = ssx; sy = ssy; sz = ssz; pxy = nxy; pxz = nxz; pyz = nyz;
        dx = ddx; dy = ddy; dz = ddz; tidx = ttidx; idx = ni; tin = ttin; steps = nst;
        busy = act && !end;
        done = fin;
    }
}

static void lanes_region(hls::stream<Ray>& in, ap_uint<1> free_pl[NL][kCap], ap_uint<1> hit_pl[NL][kCap],
                         cell_t ox, cell_t oy, cell_t oz, cell_t dimx, cell_t dimy, cell_t dimz,
                         ap_uint<15> o_idx, ap_uint<15> sty_, ap_uint<15> stz_) {
#pragma HLS DATAFLOW
    hls::stream<Ray> lane_s[NL];
#pragma HLS STREAM variable=lane_s depth=4
#ifndef PROBE_FIFO_BRAM
    // Without this HLS puts each 275-bit x 4 FIFO in 16 BRAM18 (probe run 1).
#pragma HLS BIND_STORAGE variable=lane_s type=fifo impl=srl
#endif
    dispatch(in, lane_s);
#ifdef PROBE_WALK_MUX
#define LANE(i) walk_lane_mux<i>(lane_s[i], free_pl[i], hit_pl[i], ox, oy, oz, dimx, dimy, dimz, o_idx, sty_, stz_);
#else
#define LANE(i) walk_lane<i>(lane_s[i], free_pl[i], hit_pl[i], ox, oy, oz, dimx, dimy, dimz, o_idx, sty_, stz_);
#endif
    LANE(0)
#if NL > 1
    LANE(1)
#endif
#if NL > 2
    LANE(2) LANE(3)
#endif
#if NL > 4
    LANE(4) LANE(5) LANE(6) LANE(7)
#endif
#if NL > 8
    LANE(8) LANE(9) LANE(10) LANE(11) LANE(12) LANE(13) LANE(14) LANE(15)
#endif
#undef LANE
}

void lanes_probe(hls::stream<Ray>& in, ap_uint<8> flags[kCap], cell_t ox, cell_t oy, cell_t oz,
                 cell_t dimx, cell_t dimy, cell_t dimz, ap_uint<15> o_idx, ap_uint<15> sty_,
                 ap_uint<15> stz_) {
#pragma HLS INTERFACE mode=ap_fifo port=in
#pragma HLS INTERFACE mode=bram port=flags
    static ap_uint<1> free_pl[NL][kCap];
    static ap_uint<1> hit_pl[NL][kCap];
#pragma HLS ARRAY_PARTITION variable=free_pl type=complete dim=1
#pragma HLS ARRAY_PARTITION variable=hit_pl type=complete dim=1
#pragma HLS BIND_STORAGE variable=free_pl type=ram_t2p impl=bram
#pragma HLS BIND_STORAGE variable=hit_pl type=ram_t2p impl=bram
clear_loop:
    for (int i = 0; i < kCap; ++i) {
#pragma HLS PIPELINE II=1
        for (int k = 0; k < NL; ++k) {
#pragma HLS UNROLL
            free_pl[k][i] = 0;
            hit_pl[k][i] = 0;
        }
    }
    lanes_region(in, free_pl, hit_pl, ox, oy, oz, dimx, dimy, dimz, o_idx, sty_, stz_);
merge_loop:
    for (int i = 0; i < kCap; ++i) {
#pragma HLS PIPELINE II=1
        bool f = false, h = false;
        for (int k = 0; k < NL; ++k) {
#pragma HLS UNROLL
            f = f || free_pl[k][i];
            h = h || hit_pl[k][i];
        }
        flags[i] = h ? 2 : (f ? 1 : 0);
    }
}
