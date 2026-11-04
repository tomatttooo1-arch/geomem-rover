// A2 multi-lane ray walk: functional C model + cycle-level dispatch model.
// docs/design/M7_RAY_LANES_A2_DESIGN_20261011.md
//
// Host-only (g++ -std=c++11 -O2 -ffp-contract=off -fPIC -shared). Not an HLS
// source. Three parts:
//
// 1. Convert (m7l_prepare): the per-pixel float32 projection, float64 Q8 floor,
//    range gate and classification of hw/m7/frame_depth.cpp, copied verbatim;
//    instead of calling add_ray it records one ray per walked pixel (accepted
//    or clipped) in pixel (raster) order, and the same three counts.
// 2. Lanes (m7l_run): N lanes, each with PRIVATE planes. Two walk cores:
//      core 0 "ref"   : a verbatim copy of the reference add_ray writing a
//                       private uint8 grid per lane; per lane free = (g == 1),
//                       hit = (g == 2). Zero new geometry: a mismatch against
//                       the reference can only come from dispatch or merge.
//      core 1 "pstep" : the A1 hardware form (one crossing per iteration,
//                       incremental pairwise differences P_ab, target clamped
//                       to [-1,32]); writes a free bit plane and a hit bit
//                       plane per lane. Re-implemented here from the A1
//                       A1's files are not used.
//    Merge: flags = OR(hit) ? 2 : (OR(free) ? 1 : 0).
// 3. Dispatch: which lane walks which ray and in which order comes from the
//    cycle model (policies below) or from a seeded random shuffle (functional
//    only). The cycle model is a per-cycle simulation of
//      read+convert (1 pixel per conv_ii cycles, invalid pixels included)
//        -> s_ray FIFO (depth sray_depth)
//        -> dispatcher (1 record per cycle, blocking unless fault DROP;
//           policy DIRECT: no dispatcher and no lane FIFOs, see enum)
//        -> per-lane FIFO (depth fifo_depth)
//        -> lane (a ray costs max(S,1) * step_cycles + ray_overhead cycles;
//                 S = crossings; load is merged with the first step as in A1)
//    A record pushed in cycle t is visible to its consumer in cycle t+1. A
//    FIFO slot freed in cycle t can be refilled in cycle t+1 (full flag is
//    taken at the start of the cycle). Fixed costs (clear, merge/write-out,
//    pipeline fill) are added by the caller; they are not simulated.
//
// All cycle counts produced here are MODEL numbers, not measurements.
#include "../../hw/m7/frame_depth.hpp"

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>
#include <algorithm>

namespace ref {
// ---- verbatim copies from hw/m7/frame_depth.cpp @ 8e21077 ----
static bool finite_float(float value) {
    return value >= -FLT_MAX && value <= FLT_MAX;
}

static bool finite_double(double value) {
    return value >= -DBL_MAX && value <= DBL_MAX;
}

static std::uint32_t grid_offset(const std::int32_t cell[3],
                                 std::uint32_t nx, std::uint32_t ny) {
    return (static_cast<std::uint32_t>(cell[2]) * ny +
            static_cast<std::uint32_t>(cell[1])) * nx +
            static_cast<std::uint32_t>(cell[0]);
}

static std::int32_t floor_q8(std::int32_t value) {
    return value >= 0 ? value / FPGA_M7_FRAME_Q8
                      : -((-value + FPGA_M7_FRAME_Q8 - 1) / FPGA_M7_FRAME_Q8);
}

static bool cell_in_grid(const std::int32_t cell[3], const std::uint32_t dims[3]) {
    return cell[0] >= 0 && cell[0] < static_cast<std::int32_t>(dims[0]) &&
           cell[1] >= 0 && cell[1] < static_cast<std::int32_t>(dims[1]) &&
           cell[2] >= 0 && cell[2] < static_cast<std::int32_t>(dims[2]);
}

static void add_ray(const std::int32_t origin[3], const std::int32_t end[3],
                    const std::uint32_t dims[3],
                    std::uint8_t grid[FPGA_M7_FRAME_GRID_CAPACITY]) {
    std::int32_t cell[3], target[3], direction[3], next[3], delta[3];
    for (int axis = 0; axis < 3; ++axis) {
        const std::int32_t from = origin[axis];
        const std::int32_t to = end[axis];
        cell[axis] = from / FPGA_M7_FRAME_Q8;  // from >= 0: truncation == floor.
        target[axis] = floor_q8(to);
        direction[axis] = to > from ? 1 : (to < from ? -1 : 0);
        delta[axis] = to > from ? to - from : from - to;
        next[axis] = direction[axis] > 0
            ? (cell[axis] + 1) * FPGA_M7_FRAME_Q8 - from
            : from - cell[axis] * FPGA_M7_FRAME_Q8;
    }
    for (std::uint32_t step = 0; step < 3 * FPGA_M7_FRAME_GRID_AXIS; ++step) {
        if (cell[0] == target[0] && cell[1] == target[1] &&
            cell[2] == target[2]) break;
        int first = -1;
        for (int axis = 0; axis < 3; ++axis) {
            if (direction[axis] == 0 || cell[axis] == target[axis]) continue;
            if (first < 0 ||
                static_cast<std::int64_t>(next[axis]) * delta[first] <
                static_cast<std::int64_t>(next[first]) * delta[axis])
                first = axis;
        }
        if (first < 0) break;  // Bounded valid endpoints guarantee progress.
        const std::int32_t numerator = next[first];
        const std::int32_t denominator = delta[first];
        for (int axis = 0; axis < 3; ++axis) {
            if (direction[axis] == 0 || cell[axis] == target[axis]) continue;
            if (static_cast<std::int64_t>(next[axis]) * denominator ==
                static_cast<std::int64_t>(numerator) * delta[axis]) {
                cell[axis] += direction[axis];
                next[axis] += FPGA_M7_FRAME_Q8;
            }
        }
        if (!cell_in_grid(cell, dims)) break;
        if (cell[0] != target[0] || cell[1] != target[1] ||
            cell[2] != target[2]) {
            const std::uint32_t index = grid_offset(cell, dims[0], dims[1]);
            if (grid[index] == 0) grid[index] = 1;
        }
    }
    if (cell_in_grid(target, dims))
        grid[grid_offset(target, dims[0], dims[1])] = 2;
}
// ---- end of verbatim copies ----
}  // namespace ref

namespace {

const std::uint32_t kCap = FPGA_M7_FRAME_GRID_CAPACITY;
const int kMaxLanes = 32;

struct Ray {
    std::uint32_t pixel;
    std::uint16_t u, v;
    std::int32_t end[3];
    bool inside;
    std::uint8_t steps;  // S = crossings (pstep walk), filled in prepare
};

struct Frame {
    std::uint32_t status;
    std::uint32_t counts[3];
    std::int32_t origin[3];
    std::uint32_t dims[3];
    std::uint32_t width, height, pixels;
    std::vector<Ray> rays;
    std::uint64_t step_core_mismatch;  // pstep S != instrumented-reference S
};

// Instrumented copy of ref::add_ray that only counts crossings (no writes);
// same as tools/m7_clip/ray_step_stats.cpp walk(). Used to cross-check the
// pstep core's step count ray by ray.
unsigned ref_steps(const std::int32_t origin[3], const std::int32_t end[3],
                   const std::uint32_t dims[3]) {
    std::int32_t cell[3], target[3], direction[3], next[3], delta[3];
    for (int axis = 0; axis < 3; ++axis) {
        const std::int32_t from = origin[axis], to = end[axis];
        cell[axis] = from / FPGA_M7_FRAME_Q8;
        target[axis] = ref::floor_q8(to);
        direction[axis] = to > from ? 1 : (to < from ? -1 : 0);
        delta[axis] = to > from ? to - from : from - to;
        next[axis] = direction[axis] > 0 ? (cell[axis] + 1) * FPGA_M7_FRAME_Q8 - from
                                         : from - cell[axis] * FPGA_M7_FRAME_Q8;
    }
    unsigned steps = 0;
    for (std::uint32_t step = 0; step < 3 * FPGA_M7_FRAME_GRID_AXIS; ++step) {
        if (cell[0] == target[0] && cell[1] == target[1] && cell[2] == target[2]) break;
        int first = -1;
        for (int a = 0; a < 3; ++a) {
            if (direction[a] == 0 || cell[a] == target[a]) continue;
            if (first < 0 || std::int64_t(next[a]) * delta[first] < std::int64_t(next[first]) * delta[a])
                first = a;
        }
        if (first < 0) break;
        const std::int32_t num = next[first], den = delta[first];
        for (int a = 0; a < 3; ++a) {
            if (direction[a] == 0 || cell[a] == target[a]) continue;
            if (std::int64_t(next[a]) * den == std::int64_t(num) * delta[a]) {
                cell[a] += direction[a];
                next[a] += FPGA_M7_FRAME_Q8;
            }
        }
        ++steps;
        if (!ref::cell_in_grid(cell, dims)) break;
    }
    return steps;
}

// A1 hardware form of the walk (see header). free/hit: one byte per bit.
// Returns S (crossings); the lane spends max(S,1) iterations on the ray.
unsigned walk_pstep(const std::int32_t origin[3], const std::int32_t end[3], bool inside,
                    const std::uint32_t dims[3], std::uint8_t* free_pl, std::uint8_t* hit_pl) {
    std::int32_t c[3], t[3], s[3];
    std::int64_t delta[3], next[3];
    for (int a = 0; a < 3; ++a) {
        const std::int32_t fr = origin[a], to = end[a];
        c[a] = fr / FPGA_M7_FRAME_Q8;
        const std::int32_t tg = ref::floor_q8(to);
        t[a] = tg < -1 ? -1 : (tg > 32 ? 32 : tg);
        s[a] = to > fr ? 1 : (to < fr ? -1 : 0);
        delta[a] = to > fr ? std::int64_t(to) - fr : std::int64_t(fr) - to;
        next[a] = s[a] > 0 ? (c[a] + 1) * FPGA_M7_FRAME_Q8 - fr : fr - c[a] * FPGA_M7_FRAME_Q8;
    }
    std::int64_t pxy = next[0] * delta[1] - next[1] * delta[0];
    std::int64_t pxz = next[0] * delta[2] - next[2] * delta[0];
    std::int64_t pyz = next[1] * delta[2] - next[2] * delta[1];
    const std::int64_t dx = delta[0] << 8, dy = delta[1] << 8, dz = delta[2] << 8;
    unsigned steps = 0;
    for (;;) {
        const bool ax = s[0] != 0 && c[0] != t[0];
        const bool ay = s[1] != 0 && c[1] != t[1];
        const bool az = s[2] != 0 && c[2] != t[2];
        if (!ax && !ay && !az) break;
        const bool stx = ax && (!ay || pxy <= 0) && (!az || pxz <= 0);
        const bool sty = ay && (!ax || pxy >= 0) && (!az || pyz <= 0);
        const bool stz = az && (!ax || pxz >= 0) && (!ay || pyz >= 0);
        if (stx) { pxy += dy; pxz += dz; c[0] += s[0]; }
        if (sty) { pxy -= dx; pyz += dz; c[1] += s[1]; }
        if (stz) { pxz -= dx; pyz -= dy; c[2] += s[2]; }
        ++steps;
        if (!ref::cell_in_grid(c, dims)) break;
        if (c[0] == t[0] && c[1] == t[1] && c[2] == t[2]) break;
        free_pl[ref::grid_offset(c, dims[0], dims[1])] = 1;
        if (steps == 3 * FPGA_M7_FRAME_GRID_AXIS) break;
    }
    if (inside) {
        std::int32_t tt[3];
        for (int a = 0; a < 3; ++a) tt[a] = ref::floor_q8(end[a]);
        hit_pl[ref::grid_offset(tt, dims[0], dims[1])] = 1;
    }
    return steps;
}

// Persistent lane storage (so that fault NOCLEAR can leak across frames).
std::uint8_t g_grid[kMaxLanes][kCap];   // core ref
std::uint8_t g_free[kMaxLanes][kCap];   // core pstep
std::uint8_t g_hit[kMaxLanes][kCap];
std::uint8_t g_scratch_free[kCap], g_scratch_hit[kCap];

enum Policy {
    P_RR = 0,          // round robin, blocking on the target lane's FIFO
    P_FNF = 1,         // first non-full FIFO, rotating priority
    P_IDLE = 2,        // empty FIFO first (rotating), else first non-full
    P_LWL_ORACLE = 3,  // least work left (needs ray costs: not buildable, bound)
    P_TILE_COL = 4,    // contiguous column stripes u*N/W, blocking
    P_INTERLEAVE = 5,  // static u mod N, blocking
    P_TILE_ROW_MC = 6, // contiguous row bands, one converter per lane
    P_SHUFFLE = 7,     // functional only: random order + random lane
    P_DIRECT = 8       // no dispatcher/lane FIFOs: the first idle lane takes the
                       // s_ray head and steps in the same cycle (probe variant B)
};

enum Fault {
    F_NONE = 0,
    F_MERGE_FREE_OVER_HIT = 1,
    F_DROP_ON_FULL = 2,
    F_SKIP_LAST_LANE = 3,
    F_NO_CLEAR = 4
};

struct Ring {
    std::vector<std::uint32_t> item;
    std::vector<std::uint64_t> when;
    std::uint32_t head = 0, size = 0, cap = 0;
    std::uint64_t work = 0;  // queued cost (oracle only)
    void init(std::uint32_t c) {
        cap = c; head = 0; size = 0; work = 0;
        item.assign(c, 0); when.assign(c, 0);
    }
    void push(std::uint32_t x, std::uint64_t t) {
        const std::uint32_t i = (head + size) % cap;
        item[i] = x; when[i] = t; ++size;
    }
    bool ready(std::uint64_t t) const { return size > 0 && when[head] < t; }
    std::uint32_t pop() {
        const std::uint32_t x = item[head];
        head = (head + 1) % cap; --size;
        return x;
    }
};

// stats layout (uint64[128]):
//  0 walk_cycles (first pixel cycle 0 .. last lane idle, exclusive end)
//  1 conv_done   (cycle after the converter consumed the last pixel)
//  2 conv_stall  (cycles the converter waited on a full s_ray)
//  3 disp_stall  (cycles the dispatcher held a ready record it could not place)
//  4 work_total  (sum of ray costs)   5 steps_total   6 rays
//  7 max_ray_cost  8 lower_bound = max(conv_done_unstalled, ceil(work/N), max_ray_cost)
//  9 dropped records (fault DROP)  10 max lane FIFO occupancy  11 max s_ray occupancy
// 12 tail = walk_cycles - conv_done   13 lanes
// 14 step_core_mismatch (prepare)     15 max lane busy  16 min lane busy
// 32.. lane_busy[32]   64.. lane_rays[32]
void simulate(const Frame& f, const std::int32_t* cfg, std::vector<std::vector<std::uint32_t> >& order,
              std::uint64_t* st) {
    const int N = cfg[0], policy = cfg[1];
    const std::uint32_t D = static_cast<std::uint32_t>(cfg[2]);
    const std::uint32_t Qs = static_cast<std::uint32_t>(cfg[3]);
    const std::uint64_t ii = static_cast<std::uint64_t>(cfg[4]);
    const std::uint64_t step_cycles = static_cast<std::uint64_t>(cfg[5]);
    const std::uint64_t ray_overhead = static_cast<std::uint64_t>(cfg[6]);
    const int fault = cfg[8];
    const std::uint32_t R = static_cast<std::uint32_t>(f.rays.size());
    std::vector<std::uint64_t> cost(R);
    std::uint64_t work = 0, steps = 0, maxc = 0;
    for (std::uint32_t i = 0; i < R; ++i) {
        const std::uint64_t s = f.rays[i].steps;
        cost[i] = (s > 0 ? s : 1) * step_cycles + ray_overhead;
        work += cost[i]; steps += s; maxc = std::max(maxc, cost[i]);
    }
    order.assign(N, std::vector<std::uint32_t>());
    st[4] = work; st[5] = steps; st[6] = R; st[7] = maxc; st[13] = N;

    std::vector<Ring> lf(N);
    std::vector<std::uint64_t> rem(N, 0), busy(N, 0), nrays(N, 0);
    std::vector<std::uint32_t> popped(N, 0);
    std::uint64_t t = 0, conv_done = 0, conv_stall = 0, disp_stall = 0, dropped = 0;
    std::uint32_t maxocc = 0, maxsray = 0;
    int ptr = 0;

    if (policy == P_TILE_ROW_MC) {
        // One converter per lane over its own row band; each writes its own
        // s_ray (depth Qs) read directly by its lane.
        std::vector<std::uint32_t> p(N), pend(N), ri(N), riend(N);
        std::vector<std::uint64_t> ready_at(N, 0), done_at(N, 0);
        for (int k = 0; k < N; ++k) {
            const std::uint32_t r0 = f.height * k / N, r1 = f.height * (k + 1) / N;
            p[k] = r0 * f.width; pend[k] = r1 * f.width;
            lf[k].init(Qs);
        }
        // rays are in pixel order: find each band's first ray
        for (int k = 0; k < N; ++k) {
            std::uint32_t lo = 0, hi = R;
            while (lo < hi) { std::uint32_t m = (lo + hi) / 2; if (f.rays[m].pixel < p[k]) lo = m + 1; else hi = m; }
            ri[k] = lo;
        }
        for (int k = 0; k < N; ++k) riend[k] = (k + 1 < N) ? ri[k + 1] : R;
        for (;;) {
            bool all_done = true;
            for (int k = 0; k < N; ++k) {
                popped[k] = 0;
                if (rem[k] == 0 && lf[k].ready(t)) {
                    const std::uint32_t r = lf[k].pop();
                    popped[k] = 1;
                    rem[k] = cost[r]; order[k].push_back(r); ++nrays[k];
                }
                if (rem[k] > 0) { --rem[k]; ++busy[k]; }
                if (p[k] < pend[k] && t >= ready_at[k]) {
                    if (ri[k] < riend[k] && f.rays[ri[k]].pixel == p[k]) {
                        if (lf[k].size + popped[k] < Qs) {
                            lf[k].push(ri[k], t); ++ri[k]; ++p[k]; ready_at[k] = t + ii;
                        } else {
                            ++conv_stall;
                        }
                    } else {
                        ++p[k]; ready_at[k] = t + ii;
                    }
                    if (p[k] == pend[k]) done_at[k] = t + 1;
                }
                maxsray = std::max(maxsray, lf[k].size);
                if (p[k] < pend[k] || lf[k].size > 0 || rem[k] > 0) all_done = false;
            }
            ++t;
            if (all_done) break;
        }
        for (int k = 0; k < N; ++k) conv_done = std::max(conv_done, done_at[k]);
        std::uint64_t band_max = 0;
        for (int k = 0; k < N; ++k) {
            const std::uint32_t r0 = f.height * k / N, r1 = f.height * (k + 1) / N;
            band_max = std::max<std::uint64_t>(band_max, std::uint64_t(r1 - r0) * f.width);
        }
        st[8] = std::max(std::max(band_max * ii, (work + N - 1) / N), maxc);
    } else {
        Ring sray;
        sray.init(Qs);
        for (int k = 0; k < N; ++k) lf[k].init(D);
        std::uint32_t p = 0, ri = 0;
        std::uint64_t ready_at = 0;
        for (;;) {
            // 1. lanes
            bool direct_taken = false;
            for (int k = 0; k < N; ++k) {
                popped[k] = 0;
                if (policy == P_DIRECT) {
                    if (rem[k] == 0 && !direct_taken && sray.ready(t)) {
                        const std::uint32_t r = sray.pop();
                        direct_taken = true;
                        rem[k] = cost[r]; order[k].push_back(r); ++nrays[k];
                    }
                } else if (rem[k] == 0 && lf[k].ready(t)) {
                    const std::uint32_t r = lf[k].pop();
                    lf[k].work -= cost[r];
                    popped[k] = 1;
                    rem[k] = cost[r]; order[k].push_back(r); ++nrays[k];
                }
                if (rem[k] > 0) { --rem[k]; ++busy[k]; }
            }
            // 2. dispatcher
            bool sray_popped = direct_taken;
            if (policy != P_DIRECT && sray.ready(t)) {
                const std::uint32_t r = sray.item[sray.head];
                int target = -1;
                auto has_space = [&](int k) { return lf[k].size + popped[k] < D; };
                switch (policy) {
                case P_RR:
                    if (has_space(ptr)) target = ptr;
                    break;
                case P_FNF:
                    for (int i = 0; i < N; ++i) { int k = (ptr + i) % N; if (has_space(k)) { target = k; break; } }
                    break;
                case P_IDLE:
                    for (int i = 0; i < N; ++i) { int k = (ptr + i) % N; if (lf[k].size + popped[k] == 0) { target = k; break; } }  // empty at start of cycle
                    if (target < 0)
                        for (int i = 0; i < N; ++i) { int k = (ptr + i) % N; if (has_space(k)) { target = k; break; } }
                    break;
                case P_LWL_ORACLE: {
                    std::uint64_t best = ~0ull;
                    for (int k = 0; k < N; ++k)
                        if (has_space(k) && lf[k].work + rem[k] < best) { best = lf[k].work + rem[k]; target = k; }
                    break;
                }
                case P_TILE_COL: {
                    const int k = static_cast<int>(std::uint64_t(f.rays[r].u) * N / f.width);
                    if (has_space(k)) target = k;
                    break;
                }
                case P_INTERLEAVE: {
                    const int k = f.rays[r].u % N;
                    if (has_space(k)) target = k;
                    break;
                }
                default: break;
                }
                if (target >= 0) {
                    sray.pop(); sray_popped = true;
                    lf[target].push(r, t);
                    lf[target].work += cost[r];
                    maxocc = std::max(maxocc, lf[target].size);
                    if (policy == P_RR || policy == P_FNF || policy == P_IDLE) ptr = (target + 1) % N;
                } else if (fault == F_DROP_ON_FULL) {
                    sray.pop(); sray_popped = true; ++dropped;
                    if (policy == P_RR) ptr = (ptr + 1) % N;
                } else {
                    ++disp_stall;
                }
            }
            // 3. converter
            if (p < f.pixels && t >= ready_at) {
                if (ri < R && f.rays[ri].pixel == p) {
                    if (sray.size + (sray_popped ? 1u : 0u) < Qs) {
                        sray.push(ri, t); ++ri; ++p; ready_at = t + ii;
                        maxsray = std::max(maxsray, sray.size);
                    } else {
                        ++conv_stall;
                    }
                } else {
                    ++p; ready_at = t + ii;
                }
                if (p == f.pixels) conv_done = t + 1;
            }
            ++t;
            bool idle = p >= f.pixels && sray.size == 0;
            for (int k = 0; k < N && idle; ++k) if (lf[k].size > 0 || rem[k] > 0) idle = false;
            if (idle) break;
        }
        if (f.pixels == 0) conv_done = 0;
        st[8] = std::max(std::max(std::uint64_t(f.pixels) * ii, (work + N - 1) / N), maxc);
    }
    st[0] = t; st[1] = conv_done; st[2] = conv_stall; st[3] = disp_stall;
    st[9] = dropped; st[10] = maxocc; st[11] = maxsray;
    st[12] = t > conv_done ? t - conv_done : 0;
    std::uint64_t bmax = 0, bmin = ~0ull;
    for (int k = 0; k < N; ++k) {
        st[32 + k] = busy[k]; st[64 + k] = nrays[k];
        bmax = std::max(bmax, busy[k]); bmin = std::min(bmin, busy[k]);
    }
    st[15] = bmax; st[16] = N ? bmin : 0;
}

}  // namespace

extern "C" {

// Convert one frame (reference checks, origin, per-pixel projection verbatim).
// Returns a handle (always; check *status). counts as the reference.
void* m7l_prepare(const float depth_m[FPGA_M7_FRAME_MAX_PIXELS], const float inverse_k[9],
                  const float world_camera[12], const double grid_origin_m[3],
                  double voxel_size_m, std::uint32_t width, std::uint32_t height,
                  std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
                  std::uint32_t counts[3], std::uint32_t* status) {
    using namespace ref;
    Frame* f = new Frame();
    f->step_core_mismatch = 0;
    f->width = width; f->height = height; f->pixels = 0;
    f->dims[0] = nx; f->dims[1] = ny; f->dims[2] = nz;
    counts[0] = counts[1] = counts[2] = 0;
    f->counts[0] = f->counts[1] = f->counts[2] = 0;
    // ---- reference checks and origin, verbatim ----
    *status = 1;
    if ((width == 0) != (height == 0) || width > FPGA_M7_FRAME_MAX_WIDTH ||
        height > FPGA_M7_FRAME_MAX_HEIGHT || nx == 0 || ny == 0 || nz == 0 ||
        nx > FPGA_M7_FRAME_GRID_AXIS || ny > FPGA_M7_FRAME_GRID_AXIS ||
        nz > FPGA_M7_FRAME_GRID_AXIS) { f->status = *status; return f; }
    *status = 2;
    if (!finite_double(voxel_size_m) || voxel_size_m <= 0) { f->status = *status; return f; }
    for (int axis = 0; axis < 3; ++axis)
        if (!finite_double(grid_origin_m[axis])) { f->status = *status; return f; }
    for (int i = 0; i < 9; ++i)
        if (!finite_float(inverse_k[i])) { f->status = *status; return f; }
    for (int i = 0; i < 12; ++i)
        if (!finite_float(world_camera[i])) { f->status = *status; return f; }

    const std::uint32_t dims[3] = {nx, ny, nz};
    std::int32_t origin[3];
    *status = 3;
    for (int axis = 0; axis < 3; ++axis) {
        const double coordinate = std::floor(
            (static_cast<double>(world_camera[4*axis+3]) - grid_origin_m[axis]) /
            voxel_size_m * FPGA_M7_FRAME_Q8);
        if (!finite_double(coordinate) || coordinate < 0 ||
            coordinate >= dims[axis] * FPGA_M7_FRAME_Q8) { f->status = *status; return f; }
        origin[axis] = static_cast<std::int32_t>(coordinate);
    }
    for (int a = 0; a < 3; ++a) f->origin[a] = origin[a];

    const std::uint32_t pixels = width * height;
    f->pixels = pixels;
    f->rays.reserve(pixels);
    std::uint32_t accepted = 0, invalid = 0, clipped = 0;
    for (std::uint32_t pixel = 0; pixel < FPGA_M7_FRAME_MAX_PIXELS; ++pixel) {
        if (pixel >= pixels) break;
        const float z = depth_m[pixel];
        if (!(z > 0.1f && z < 10.0f)) {
            ++invalid;
            continue;
        }
        const float u = static_cast<float>(pixel % width);
        const float v = static_cast<float>(pixel / width);
        float camera[3], world[3];
        for (int row = 0; row < 3; ++row)
            camera[row] = (inverse_k[3*row]*u + inverse_k[3*row+1]*v +
                           inverse_k[3*row+2]) * z;
        bool good = true;
        for (int row = 0; row < 3; ++row) {
            world[row] = world_camera[4*row]*camera[0] +
                         world_camera[4*row+1]*camera[1] +
                         world_camera[4*row+2]*camera[2] +
                         world_camera[4*row+3];
            good = good && finite_float(world[row]);
        }
        if (!good) {
            ++invalid;
            continue;
        }
        std::int32_t end[3];
        bool inside = true, representable = true;
        for (int axis = 0; axis < 3; ++axis) {
            const double coordinate = std::floor(
                (static_cast<double>(world[axis]) - grid_origin_m[axis]) /
                voxel_size_m * FPGA_M7_FRAME_Q8);
            if (!(coordinate >= -FPGA_M7_FRAME_Q8_LIMIT &&
                  coordinate < FPGA_M7_FRAME_Q8_LIMIT)) {
                representable = false;
                end[axis] = 0;
                continue;
            }
            end[axis] = static_cast<std::int32_t>(coordinate);
            if (coordinate < 0 || coordinate >= dims[axis] * FPGA_M7_FRAME_Q8)
                inside = false;
        }
        if (!representable) {
            ++invalid;
            continue;
        }
        // ---- end of verbatim part; the reference calls add_ray here ----
        Ray r;
        r.pixel = pixel;
        r.u = static_cast<std::uint16_t>(pixel % width);
        r.v = static_cast<std::uint16_t>(pixel / width);
        for (int a = 0; a < 3; ++a) r.end[a] = end[a];
        r.inside = inside;
        const unsigned s = walk_pstep(origin, end, inside, dims, g_scratch_free, g_scratch_hit);
        if (s != ref_steps(origin, end, dims)) ++f->step_core_mismatch;
        r.steps = static_cast<std::uint8_t>(s);
        f->rays.push_back(r);
        if (inside)
            ++accepted;
        else
            ++clipped;
    }
    f->counts[0] = counts[0] = accepted;
    f->counts[1] = counts[1] = invalid;
    f->counts[2] = counts[2] = clipped;
    *status = 0;
    f->status = 0;
    return f;
}

void m7l_free(void* h) { delete static_cast<Frame*>(h); }

std::uint32_t m7l_num_rays(void* h) { return static_cast<std::uint32_t>(static_cast<Frame*>(h)->rays.size()); }

std::uint64_t m7l_step_core_mismatch(void* h) { return static_cast<Frame*>(h)->step_core_mismatch; }

// Per pixel: steps (S) and class (0 invalid, 1 accepted, 2 clipped).
void m7l_profile(void* h, std::uint8_t* steps, std::uint8_t* cls) {
    Frame* f = static_cast<Frame*>(h);
    std::memset(steps, 0, f->pixels);
    std::memset(cls, 0, f->pixels);
    for (const Ray& r : f->rays) {
        steps[r.pixel] = r.steps;
        cls[r.pixel] = r.inside ? 1 : 2;
    }
}

// cfg (int32[16]): 0 lanes, 1 policy, 2 lane fifo depth, 3 s_ray depth,
// 4 conv_ii, 5 step_cycles, 6 ray_overhead, 7 core (0 ref, 1 pstep),
// 8 fault, 9 seed (shuffle), 10 functional (1 = build flags), 11 timing (1 = run cycle model)
// Returns 0 on success, nonzero on bad config or bad frame status.
int m7l_run(void* h, const std::int32_t* cfg, std::uint8_t flags[FPGA_M7_FRAME_GRID_CAPACITY],
            std::uint32_t counts[3], std::uint64_t* st) {
    Frame* f = static_cast<Frame*>(h);
    const int N = cfg[0], policy = cfg[1], core = cfg[7], fault = cfg[8];
    if (N < 1 || N > kMaxLanes || cfg[2] < 1 || cfg[3] < 1 || cfg[4] < 1 || cfg[5] < 1 || cfg[6] < 0)
        return 10;
    if (f->status != 0) return 20 + static_cast<int>(f->status);
    std::memset(st, 0, 128 * sizeof(std::uint64_t));
    st[14] = f->step_core_mismatch;
    std::vector<std::vector<std::uint32_t> > order;
    const std::uint32_t R = static_cast<std::uint32_t>(f->rays.size());
    if (policy == P_SHUFFLE) {
        std::mt19937_64 rng(static_cast<std::uint64_t>(cfg[9]));
        std::vector<std::uint32_t> perm(R);
        for (std::uint32_t i = 0; i < R; ++i) perm[i] = i;
        std::shuffle(perm.begin(), perm.end(), rng);
        order.assign(N, std::vector<std::uint32_t>());
        std::uniform_int_distribution<int> pick(0, N - 1);
        for (std::uint32_t i = 0; i < R; ++i) order[pick(rng)].push_back(perm[i]);
        st[6] = R; st[13] = N;
    } else if ((policy >= P_RR && policy <= P_TILE_ROW_MC) || policy == P_DIRECT) {
        if (cfg[11]) simulate(*f, cfg, order, st);
        else return 11;
    } else {
        return 12;
    }
    if (!cfg[10]) return 0;
    // Functional pass: every lane walks its rays, in its order, into its
    // private planes.
    const std::uint32_t n = kCap;  // planes beyond nx*ny*nz stay 0, like the reference grid
    for (int k = 0; k < N; ++k) {
        if (fault != F_NO_CLEAR) {
            std::memset(g_grid[k], 0, kCap);
            std::memset(g_free[k], 0, kCap);
            std::memset(g_hit[k], 0, kCap);
        }
        for (std::uint32_t r : order[k]) {
            const Ray& ray = f->rays[r];
            if (core == 0) ref::add_ray(f->origin, ray.end, f->dims, g_grid[k]);
            else walk_pstep(f->origin, ray.end, ray.inside, f->dims, g_free[k], g_hit[k]);
        }
    }
    const int merged = (fault == F_SKIP_LAST_LANE && N > 1) ? N - 1 : N;
    for (std::uint32_t i = 0; i < n; ++i) {
        bool any_free = false, any_hit = false;
        for (int k = 0; k < merged; ++k) {
            if (core == 0) { any_free |= g_grid[k][i] == 1; any_hit |= g_grid[k][i] == 2; }
            else { any_free |= g_free[k][i] != 0; any_hit |= g_hit[k][i] != 0; }
        }
        if (fault == F_MERGE_FREE_OVER_HIT)
            flags[i] = any_free ? 1 : (any_hit ? 2 : 0);
        else
            flags[i] = any_hit ? 2 : (any_free ? 1 : 0);
    }
    counts[0] = f->counts[0]; counts[1] = f->counts[1]; counts[2] = f->counts[2];
    return 0;
}

}  // extern "C"
