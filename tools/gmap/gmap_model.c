/* GMAP1 fixed-point C model of the planner core's global-map operations (see gmap_model.h).
 *
 * MERGE (DESIGN 4.2): one 80 x 80 frame at integer offset (ix, iy) into the 400 x 400 int16 global map.
 *   Region  rows R0 = iy - (iy & 1) .. R1 = iy + 80 + ((iy + 80) & 1)   (even-aligned, clamped to [0, 400))
 *           cols C0 = floor(ix / 8) * 8 .. C1 = ceil((ix + 80) / 8) * 8  (8-aligned, clamped to [0, 400))
 *   Every region cell is read and written back whole (cells outside the window unchanged), and every coarse cell
 *   under the region is recomputed from its four fine cells (2 x 2 most optimistic, then the code table), so each
 *   written word -- 2 fine cells or 4 coarse cells -- is complete: no read of the coarse map, no partial writes.
 *   Dropped = known frame cells whose global cell is outside the map (same count as kv_mapper.dropped_cells).
 * SCAN  (DESIGN 4.3): whole map -> image (code table per rank, optional y flip), coarse map rebuilt, bbox of seen.
 * CLEAR (DESIGN 4.4): global map all unseen, coarse map all "unknown" code; GMAP3: param byte 13 bit 0 also sets
 *        the swept time map T to "never" (kv_nav start).
 * WINDOW (GMAP3, DESIGN_ADD_PLGRID sections 1-3, positions Q16): body seed + swept clearing of PlGridLayer on the
 *        80 x 80 aux plane; output codes into the frame slot (MERGE reads it next), T updated in place.
 *        Segment 1: per capsule point a 16 x 16 box (cells floor(P) - 8 .. floor(P) + 7 on each axis, clamped to the
 *        window) -> "swept this frame" mask; exact for r_b <= 7 cells (car: 6). Segment 2: per window cell.
 * Rank: free 0 < unknown 1 < occupied 2; unknown = the -32768 sentinel; occupied = L > 0.
 */
#include "gmap_model.h"

static uint32_t rd8(const uint8_t *m, uint32_t a) { return m[a]; }

static void wr32(uint8_t *m, uint32_t a, uint32_t w) {
    m[a] = (uint8_t)w;
    m[a + 1] = (uint8_t)(w >> 8);
    m[a + 2] = (uint8_t)(w >> 16);
    m[a + 3] = (uint8_t)(w >> 24);
}

static int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

static int32_t floor8(int32_t v) { return v >= 0 ? v / 8 : -((-v + 7) / 8); }   /* floor(v / 8) */

static uint32_t rank_of(int32_t L) { return L == GMAP_UNSEEN ? 1u : (L > 0 ? 2u : 0u); }

static uint32_t min4(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    uint32_t x = a < b ? a : b, y = c < d ? c : d;
    return x < y ? x : y;
}

typedef struct {
    int32_t ix, iy;
    uint32_t ctab[3], itab[3], flip, clear_t;
} gmap_params;

static int read_params(const uint8_t *m, gmap_params *p) {
    const uint32_t P = GMAP_OFF_PARAM;
    const uint32_t magic = rd8(m, P) | (rd8(m, P + 1) << 8);
    p->ix = (int16_t)(rd8(m, P + 2) | (rd8(m, P + 3) << 8));
    p->iy = (int16_t)(rd8(m, P + 4) | (rd8(m, P + 5) << 8));
    for (int k = 0; k < 3; ++k) {
        p->ctab[k] = rd8(m, P + 6 + k);
        p->itab[k] = rd8(m, P + 9 + k);
    }
    p->flip = rd8(m, P + 12) & 1u;
    p->clear_t = rd8(m, P + 13) & 1u;
    return magic == GMAP_MAGIC;
}

static void write_result(uint8_t *m, uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3) {
    wr32(m, GMAP_OFF_RESULT, w0);
    wr32(m, GMAP_OFF_RESULT + 4, w1);
    wr32(m, GMAP_OFF_RESULT + 8, w2);
    wr32(m, GMAP_OFF_RESULT + 12, w3);
}

static int32_t rd_cell(const uint8_t *m, int32_t r, int32_t c) {
    const uint32_t a = GMAP_OFF_GLOBAL + ((uint32_t)r * GMAP_N + (uint32_t)c) * 2u;
    return (int16_t)(rd8(m, a) | (rd8(m, a + 1) << 8));
}

/* ---------------------------------------------------------------------------------------------- MERGE */
static uint32_t merge(uint8_t *m, const gmap_params *p) {
    const int32_t ix = p->ix, iy = p->iy;
    const int32_t R0 = clampi(iy - (iy & 1), 0, GMAP_N);
    const int32_t R1 = clampi(iy + GMAP_W + ((iy + GMAP_W) & 1), 0, GMAP_N);
    const int32_t C0 = clampi(floor8(ix) * 8, 0, GMAP_N);
    const int32_t C1 = clampi(-floor8(-(ix + GMAP_W)) * 8, 0, GMAP_N);
    uint32_t dropped = 0;
    uint8_t frow[GMAP_W];
    uint8_t lb[GMAP_N];                       /* rank line buffer of the even row, index c - C0 (<= 88 used) */

    /* frame rows outside the map: read only to count their known cells */
    for (int32_t ly = 0; ly < GMAP_W; ++ly) {
        const int32_t r = iy + ly;
        if (r >= 0 && r < GMAP_N) continue;
        for (int32_t lx = 0; lx < GMAP_W; ++lx)
            if (rd8(m, GMAP_OFF_FRAME + (uint32_t)(ly * GMAP_W + lx)) != 0xFFu) ++dropped;
    }
    for (int32_t r = R0; r < R1; ++r) {
        const int inwin = r >= iy && r < iy + GMAP_W;
        if (inwin) {
            const int32_t ly = r - iy;
            for (int32_t lx = 0; lx < GMAP_W; ++lx) {
                frow[lx] = (uint8_t)rd8(m, GMAP_OFF_FRAME + (uint32_t)(ly * GMAP_W + lx));
                const int32_t c = ix + lx;
                if ((c < 0 || c >= GMAP_N) && frow[lx] != 0xFFu) ++dropped;
            }
        }
        uint32_t word = 0, cword = 0, rk_prev = 0;
        for (int32_t c = C0; c < C1; ++c) {
            int32_t L = rd_cell(m, r, c);
            const int32_t lx = c - ix;
            if (inwin && lx >= 0 && lx < GMAP_W && frow[lx] != 0xFFu) {
                const int32_t d = frow[lx] == 0x64u ? GMAP_HIT : GMAP_MISS;
                const int32_t v = L == GMAP_UNSEEN ? 0 : L;
                L = clampi(v + d, GMAP_LMIN, GMAP_LMAX);           /* v + d fits 17 bits */
            }
            const uint32_t u = (uint16_t)L;
            if ((c & 1) == 0) {
                word = u;
            } else {
                word |= u << 16;
                wr32(m, GMAP_OFF_GLOBAL + ((uint32_t)r * GMAP_N + (uint32_t)(c - 1)) * 2u, word);
            }
            const uint32_t rk = rank_of(L);
            if ((r & 1) == 0) {
                lb[c - C0] = (uint8_t)rk;
            } else if (c & 1) {
                const uint32_t mn = min4(lb[c - 1 - C0], lb[c - C0], rk_prev, rk);
                const uint32_t k = ((uint32_t)c >> 1) & 3u;         /* C0 % 8 == 0: words start at k == 0 */
                cword |= p->ctab[mn] << (8u * k);
                if (k == 3u) {
                    wr32(m, GMAP_OFF_COARSE + (uint32_t)(r >> 1) * GMAP_CN + ((uint32_t)c >> 1) - 3u, cword);
                    cword = 0;
                }
            }
            rk_prev = rk;
        }
    }
    return dropped;
}

/* ----------------------------------------------------------------------------------------------- SCAN */
static uint32_t scan(uint8_t *m, const gmap_params *p, uint32_t *w2, uint32_t *w3) {
    uint32_t rmin = GMAP_N, rmax = 0, cmin = GMAP_N, cmax = 0, any = 0;
    uint8_t lb[GMAP_N];
    for (int32_t r = 0; r < GMAP_N; ++r) {
        const uint32_t rr = p->flip ? (uint32_t)(GMAP_N - 1 - r) : (uint32_t)r;
        uint32_t iword = 0, cword = 0, rk_prev = 0;
        for (int32_t c = 0; c < GMAP_N; ++c) {
            const int32_t L = rd_cell(m, r, c);
            const uint32_t rk = rank_of(L);
            if (L != GMAP_UNSEEN) {
                any = 1;
                if ((uint32_t)r < rmin) rmin = (uint32_t)r;
                if ((uint32_t)r > rmax) rmax = (uint32_t)r;
                if ((uint32_t)c < cmin) cmin = (uint32_t)c;
                if ((uint32_t)c > cmax) cmax = (uint32_t)c;
            }
            iword |= p->itab[rk] << (8u * ((uint32_t)c & 3u));
            if ((c & 3) == 3) {
                wr32(m, GMAP_OFF_IMAGE + rr * GMAP_N + (uint32_t)c - 3u, iword);
                iword = 0;
            }
            if ((r & 1) == 0) {
                lb[c] = (uint8_t)rk;
            } else if (c & 1) {
                const uint32_t mn = min4(lb[c - 1], lb[c], rk_prev, rk);
                const uint32_t k = ((uint32_t)c >> 1) & 3u;
                cword |= p->ctab[mn] << (8u * k);
                if (k == 3u) {
                    wr32(m, GMAP_OFF_COARSE + (uint32_t)(r >> 1) * GMAP_CN + ((uint32_t)c >> 1) - 3u, cword);
                    cword = 0;
                }
            }
            rk_prev = rk;
        }
    }
    if (!any) {
        *w2 = 0;
        *w3 = 0;
        return GMAP_ST_BBOX_EMPTY;
    }
    *w2 = rmin | (rmax << 16);
    *w3 = cmin | (cmax << 16);
    return 0;
}

/* ---------------------------------------------------------------------------------------------- CLEAR */
static void clear(uint8_t *m, const gmap_params *p) {
    for (uint32_t k = 0; k < GMAP_N * GMAP_N / 2; ++k) wr32(m, GMAP_OFF_GLOBAL + 4u * k, 0x80008000u);
    const uint32_t u = p->ctab[1];
    const uint32_t w = u | (u << 8) | (u << 16) | (u << 24);
    for (uint32_t k = 0; k < GMAP_CN * GMAP_CN / 4; ++k) wr32(m, GMAP_OFF_COARSE + 4u * k, w);
    if (p->clear_t)
        for (uint32_t k = 0; k < GMAP_N * GMAP_N; ++k) wr32(m, GMAP_OFF_T + 4u * k, (uint32_t)GMAP_NEVER);
}

/* --------------------------------------------------------------------------------------------- WINDOW */
static uint32_t rd32(const uint8_t *m, uint32_t a) {
    return rd8(m, a) | (rd8(m, a + 1) << 8) | (rd8(m, a + 2) << 16) | (rd8(m, a + 3) << 24);
}

static int32_t sx24(uint32_t v) { return (int32_t)(v << 8) >> 8; }        /* the core keeps 24 bits */

static const uint64_t M48 = (1ull << 48) - 1u;                              /* the core keeps 48 bits of r^2 */

/* closest-point distance along one axis from cell k (Q16 edges k*Q, (k+1)*Q) to P, signed (squared next) */
static int64_t axis_d(int32_t k, int32_t P) {
    const int64_t a = (int64_t)k * GMAP_QS - P, b = a + GMAP_QS;
    return a > 0 ? a : (b < 0 ? b : 0);
}

typedef struct {
    int32_t ix, iy, cx, cy, now, thr, npts;
    uint32_t seed_on, swept_on;
    uint64_t rs2, re2, rb2;
    int32_t px[GMAP_PTS_MAX], py[GMAP_PTS_MAX];
} gmap_wparams;

static int read_wparams(const uint8_t *m, gmap_wparams *w) {
    const uint32_t P = GMAP_OFF_WPARAM;
    const uint32_t magic = rd8(m, P) | (rd8(m, P + 1) << 8);
    w->ix = (int16_t)(rd8(m, P + 2) | (rd8(m, P + 3) << 8));
    w->iy = (int16_t)(rd8(m, P + 4) | (rd8(m, P + 5) << 8));
    w->seed_on = rd8(m, P + 6) & 1u;
    w->swept_on = (rd8(m, P + 6) >> 1) & 1u;
    w->npts = (int32_t)rd8(m, P + 7);
    w->now = (int32_t)rd32(m, P + 8);
    w->thr = (int32_t)rd32(m, P + 12);
    w->rs2 = (rd32(m, P + 16) | ((uint64_t)rd32(m, P + 20) << 32)) & M48;
    w->re2 = (rd32(m, P + 24) | ((uint64_t)rd32(m, P + 28) << 32)) & M48;
    w->rb2 = (rd32(m, P + 32) | ((uint64_t)rd32(m, P + 36) << 32)) & M48;
    w->cx = sx24(rd32(m, P + 40));
    w->cy = sx24(rd32(m, P + 44));
    if (magic != GMAP_WMAGIC || w->npts > GMAP_PTS_MAX) return 0;
    for (int32_t k = 0; k < w->npts; ++k) {
        w->px[k] = sx24(rd32(m, P + 48 + 8 * (uint32_t)k));
        w->py[k] = sx24(rd32(m, P + 52 + 8 * (uint32_t)k));
    }
    return 1;
}

static void window(uint8_t *m, const gmap_wparams *w, uint32_t *cleared, uint32_t *evidence) {
    uint8_t mask[GMAP_W * GMAP_W];
    for (int32_t i = 0; i < GMAP_W * GMAP_W; ++i) mask[i] = 0;
    /* segment 1: swept-this-frame mask */
    if (w->swept_on)
        for (int32_t k = 0; k < w->npts; ++k) {
            const int32_t fx = w->px[k] >> 16, fy = w->py[k] >> 16;            /* floor (arithmetic shift) */
            for (int32_t v = 0; v < 16; ++v)
                for (int32_t u = 0; u < 16; ++u) {
                    const int32_t lx = fx - 8 + u, ly = fy - 8 + v;
                    if (lx < 0 || lx >= GMAP_W || ly < 0 || ly >= GMAP_W) continue;
                    const int64_t ex = axis_d(lx, w->px[k]), ey = axis_d(ly, w->py[k]);
                    if ((uint64_t)(ex * ex + ey * ey) <= w->rb2) mask[ly * GMAP_W + lx] = 1;
                }
        }
    /* segment 2: per cell */
    uint32_t ncl = 0, nev = 0;
    for (int32_t ly = 0; ly < GMAP_W; ++ly) {
        const int32_t r = w->iy + ly;
        const int64_t dy = axis_d(ly, w->cy);
        uint32_t word = 0;
        for (int32_t lx = 0; lx < GMAP_W; ++lx) {
            const int32_t c = w->ix + lx;
            const int inmap = r >= 0 && r < GMAP_N && c >= 0 && c < GMAP_N;
            const uint32_t ta = GMAP_OFF_T + 4u * ((uint32_t)r * GMAP_N + (uint32_t)c);
            const int32_t T = inmap ? (int32_t)rd32(m, ta) : GMAP_NEVER;
            const uint32_t aux = rd8(m, GMAP_OFF_AUX + (uint32_t)(ly * GMAP_W + lx));
            const uint32_t tall = (aux >> 2) & 1u;
            const int64_t dx = axis_d(lx, w->cx);
            const uint64_t d2 = (uint64_t)(dx * dx + dy * dy);
            const uint32_t seed = w->seed_on && d2 <= w->rs2, near = d2 <= w->re2;
            uint32_t f = aux & 3u, r1 = 0;
            int32_t Tn = T;
            if (w->swept_on) {
                const uint32_t r0 = T >= w->thr, mk = mask[ly * GMAP_W + lx];
                const uint32_t strong = r0 && tall && near;
                Tn = mk ? w->now : (strong ? GMAP_NEVER : T);
                r1 = mk || (r0 && !strong);
                nev += strong;
            }
            if (seed && f == 0) f = 1;
            if (r1 && f != 1) ++ncl;
            if (r1) f = 1;
            if (inmap) wr32(m, ta, (uint32_t)Tn);
            const uint32_t code = f == 2 ? 0x64u : (f == 1 ? 0x00u : 0xFFu);
            word |= code << (8u * ((uint32_t)lx & 3u));
            if ((lx & 3) == 3) {
                wr32(m, GMAP_OFF_FRAME + (uint32_t)(ly * GMAP_W + lx - 3), word);
                word = 0;
            }
        }
    }
    *cleared = ncl;
    *evidence = nev;
}

uint32_t gmap_run(uint8_t *mem, uint32_t op) {
    gmap_params p;
    const uint32_t ver = GMAP_VERSION << 8;
    if (op < GMAP_OP_MERGE || op > GMAP_OP_WINDOW) {
        write_result(mem, ver | GMAP_ST_BAD_OP, 0, 0, 0);
        return ver | GMAP_ST_BAD_OP;
    }
    if (op == GMAP_OP_WINDOW) {
        gmap_wparams w;
        uint32_t cl = 0, ev = 0;
        if (!read_wparams(mem, &w)) {
            write_result(mem, ver | GMAP_ST_BAD_MAGIC, 0, 0, 0);
            return ver | GMAP_ST_BAD_MAGIC;
        }
        window(mem, &w, &cl, &ev);
        write_result(mem, ver, cl, ev, 0);
        return ver;
    }
    if (!read_params(mem, &p)) {
        write_result(mem, ver | GMAP_ST_BAD_MAGIC, 0, 0, 0);
        return ver | GMAP_ST_BAD_MAGIC;
    }
    if (op == GMAP_OP_MERGE) {
        const uint32_t dropped = merge(mem, &p);
        write_result(mem, ver, dropped, 0, 0);
        return ver;
    }
    if (op == GMAP_OP_SCAN) {
        uint32_t w2, w3;
        const uint32_t st = scan(mem, &p, &w2, &w3);
        write_result(mem, ver | st, 0, w2, w3);
        return ver | st;
    }
    clear(mem, &p);
    write_result(mem, ver, 0, 0, 0);
    return ver;
}
