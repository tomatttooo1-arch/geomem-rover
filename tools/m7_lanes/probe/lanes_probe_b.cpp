// A2 probe, variant B (csynth only; not bit-checked here): ONE pipelined walk
// process with NL lanes unrolled inside its II=1 loop. Each cycle at most one
// ray is read (non-blocking) into the first idle lane ("idle lane first"),
// and every busy lane takes one crossing. Lane state and planes are
// partitioned per lane. No dispatcher process, no per-lane FIFOs.
// Compare with variant A (lanes_probe.cpp: one dataflow process per lane).
// Build: same run_probe.tcl with PROBE_SRC=lanes_probe_b.cpp, top lanes_probe.
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
    cell_t cx[NL], cy[NL], cz[NL], tx[NL], ty[NL], tz[NL];
    ap_int<2> sx[NL], sy[NL], sz[NL];
    cross_t pxy[NL], pxz[NL], pyz[NL];
    d256_t dx[NL], dy[NL], dz[NL];
    ap_uint<15> tidx[NL], idx[NL];
    bool tin[NL], busy[NL];
    ap_uint<7> steps[NL];
#pragma HLS ARRAY_PARTITION variable=cx type=complete
#pragma HLS ARRAY_PARTITION variable=cy type=complete
#pragma HLS ARRAY_PARTITION variable=cz type=complete
#pragma HLS ARRAY_PARTITION variable=tx type=complete
#pragma HLS ARRAY_PARTITION variable=ty type=complete
#pragma HLS ARRAY_PARTITION variable=tz type=complete
#pragma HLS ARRAY_PARTITION variable=sx type=complete
#pragma HLS ARRAY_PARTITION variable=sy type=complete
#pragma HLS ARRAY_PARTITION variable=sz type=complete
#pragma HLS ARRAY_PARTITION variable=pxy type=complete
#pragma HLS ARRAY_PARTITION variable=pxz type=complete
#pragma HLS ARRAY_PARTITION variable=pyz type=complete
#pragma HLS ARRAY_PARTITION variable=dx type=complete
#pragma HLS ARRAY_PARTITION variable=dy type=complete
#pragma HLS ARRAY_PARTITION variable=dz type=complete
#pragma HLS ARRAY_PARTITION variable=tidx type=complete
#pragma HLS ARRAY_PARTITION variable=idx type=complete
#pragma HLS ARRAY_PARTITION variable=tin type=complete
#pragma HLS ARRAY_PARTITION variable=busy type=complete
#pragma HLS ARRAY_PARTITION variable=steps type=complete
init_loop:
    for (int k = 0; k < NL; ++k) {
#pragma HLS UNROLL
        busy[k] = false;
        cx[k] = 0; cy[k] = 0; cz[k] = 0; tx[k] = 0; ty[k] = 0; tz[k] = 0;
        sx[k] = 0; sy[k] = 0; sz[k] = 0; pxy[k] = 0; pxz[k] = 0; pyz[k] = 0;
        dx[k] = 0; dy[k] = 0; dz[k] = 0; tidx[k] = 0; idx[k] = 0; tin[k] = false; steps[k] = 0;
    }
    bool in_done = false, any_busy = false;
walk_loop:
    while (!in_done || any_busy) {
#pragma HLS PIPELINE II=1
#pragma HLS DEPENDENCE variable=free_pl type=inter false
#pragma HLS DEPENDENCE variable=hit_pl type=inter false
        int sel = -1;
        for (int k = NL - 1; k >= 0; --k) {
#pragma HLS UNROLL
            if (!busy[k]) sel = k;
        }
        Ray r;
        bool got = false;
        if (!in_done && sel >= 0) {
            if (in.read_nb(r)) {
                if (r.last) in_done = true;
                else got = true;
            }
        }
        bool nb = false;
        for (int k = 0; k < NL; ++k) {
#pragma HLS UNROLL
            if (got && k == sel) {
                cx[k] = ox; cy[k] = oy; cz[k] = oz; idx[k] = o_idx;
                tx[k] = r.tx; ty[k] = r.ty; tz[k] = r.tz;
                sx[k] = r.sx; sy[k] = r.sy; sz[k] = r.sz;
                pxy[k] = r.pxy; pxz[k] = r.pxz; pyz[k] = r.pyz;
                dx[k] = r.dx; dy[k] = r.dy; dz[k] = r.dz;
                tidx[k] = r.tidx; tin[k] = r.tin; steps[k] = 0; busy[k] = true;
            }
            if (busy[k]) {
                const bool ax = sx[k] != 0 && cx[k] != tx[k];
                const bool ay = sy[k] != 0 && cy[k] != ty[k];
                const bool az = sz[k] != 0 && cz[k] != tz[k];
                bool end = false;
                if (!ax && !ay && !az) {
                    end = true;
                } else {
                    const bool stx = ax && (!ay || pxy[k] <= 0) && (!az || pxz[k] <= 0);
                    const bool sty = ay && (!ax || pxy[k] >= 0) && (!az || pyz[k] <= 0);
                    const bool stz = az && (!ax || pxz[k] >= 0) && (!ay || pyz[k] >= 0);
                    cross_t nxy = pxy[k], nxz = pxz[k], nyz = pyz[k];
                    if (stx) { nxy += dy[k]; nxz += dz[k]; }
                    if (sty) { nxy -= dx[k]; nyz += dz[k]; }
                    if (stz) { nxz -= dx[k]; nyz -= dy[k]; }
                    pxy[k] = nxy; pxz[k] = nxz; pyz[k] = nyz;
                    ap_uint<15> ni = idx[k];
                    if (stx) { cx[k] += sx[k]; ni += (sx[k] > 0) ? ap_uint<15>(1) : ap_uint<15>(-1); }
                    if (sty) { cy[k] += sy[k]; ni += (sy[k] > 0) ? sty_ : ap_uint<15>(-sty_); }
                    if (stz) { cz[k] += sz[k]; ni += (sz[k] > 0) ? stz_ : ap_uint<15>(-stz_); }
                    idx[k] = ni;
                    steps[k] = steps[k] + 1;
                    const bool in_grid = cx[k] >= 0 && cx[k] < dimx && cy[k] >= 0 && cy[k] < dimy &&
                                         cz[k] >= 0 && cz[k] < dimz;
                    if (!in_grid) {
                        end = true;
                    } else if (cx[k] == tx[k] && cy[k] == ty[k] && cz[k] == tz[k]) {
                        end = true;
                    } else {
                        free_pl[k][ni] = 1;
                        if (steps[k] == kMaxSteps) end = true;
                    }
                }
                if (end) {
                    if (tin[k]) hit_pl[k][tidx[k]] = 1;
                    busy[k] = false;
                }
            }
            nb = nb || busy[k];
        }
        any_busy = nb;
    }
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
