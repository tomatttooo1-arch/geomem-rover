// M7 same-function CPU baseline on the target host (Jetson Orin Nano).
//
// Chain per depth frame (same semantics as hw/m7 reference kernels):
//   S0 decode : raw uint16 depth units -> float32 optical-Z metres, 0 -> NaN
//   S1 project: validity (0.1 < z < 10) + float32 back-projection + camera->
//               local transform + Q8 grid coordinate (float64 floor), in/out
//               (|Q8| >= 2^24 counts as invalid, M7_CLIP_SPEC_20261011)
//   S2 rays   : integer DDA per accepted AND clipped ray (2026-10-11 clip
//               semantics: an out-of-grid endpoint is walked until the ray
//               leaves the grid, no HIT), UNKNOWN(0)/FREE(1)/HIT(2) grid
//   S3 object : one masked object (projection of mask pixels + median of the
//               local points) and one ROI-only object (front/behind counts)
//
// Arms
//   ref   : the unchanged repo kernels (hw/m7/frame_depth.cpp fuses S1+S2;
//           hw/m7/object_evidence.cpp is S3 without the median), built by the
//           caller with the repo flags; S0 and the median are added here.
//   opt   : this file's staged implementation: OpenMP rows for S1, per-thread
//           grids merged by max for S2 (exact: the per-voxel result is the max
//           over rays of {traversed=1, end=2}), OpenMP row chunks with
//           in-order concatenation for S3. -ffp-contract=off keeps float32
//           rounding identical to the reference.
// Every opt frame is compared voxel-by-voxel and count-by-count with ref; any
// difference is a hard failure (exit 7). A comparator self-test flips one
// voxel and must be detected.
//
// Usage: m7_cpu_bench --depth <u16 file> --width W --height H --frames N
//          --fx --fy --cx --cy --unit-m <metres per depth unit>
//          --grid g1|g2 --arm ref|opt --passes P --warmup K --csv out.csv
//          [--mask x0,y0,x1,y1] [--roi x0,y0,x1,y1]
//          [--pace-hz F]  sustained mode: frames are issued on a fixed F Hz clock
//                         (cycling through the N input frames); the process sleeps
//                         between frames, so whole-board CPU/power at camera rate
//                         can be read. Late frames are counted, never skipped.
//          [--grid p32]   product validation grid of tools/nano_loop/chain.py
//                         default_grid(): origin (-0.8,-0.8,-0.05) m, 5 cm, 32^3
//          [--ik a,b,...] the exact 9 float32 inverse-K values the host passes
//                         (FrameNativeM7 uses float32(numpy.linalg.inv(K)));
//                         strtof accepts hex floats, so the host can hand them
//                         over bit-exactly. Default: float32 of 1/fx, -cx/fx, ...
//          [--no-object]  geometry only (S0-S2): skip S3 in both arms
//          [--dump f]     write the opt arm's per-frame output of the first
//                         measured pass: per frame 3 x uint32 counts (accepted,
//                         invalid, clipped) + nx*ny*nz uint8 flags, frames in
//                         input order (the host comparator reads this)
// The final JSON line also reports getrusage CPU time over the measured passes
// only (the untimed reference pre-pass is excluded).
#include <algorithm>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <vector>
#ifdef _OPENMP
#include <omp.h>
#endif
#include "frame_depth.hpp"
#include "object_evidence.hpp"

using clk = std::chrono::steady_clock;
static double ms_since(clk::time_point t) { return std::chrono::duration<double, std::milli>(clk::now() - t).count(); }

struct Grid { double origin[3]; double voxel; uint32_t nx, ny, nz; };

// ---------------- opt arm ----------------
static inline uint32_t goff(const int32_t c[3], uint32_t nx, uint32_t ny) {
    return (uint32_t(c[2]) * ny + uint32_t(c[1])) * nx + uint32_t(c[0]);
}

static inline int32_t floor_q8(int32_t v) { return v >= 0 ? v / 256 : -((-v + 255) / 256); }
static inline bool in_grid(const int32_t c[3], const uint32_t d[3]) {
    return c[0] >= 0 && c[0] < int32_t(d[0]) && c[1] >= 0 && c[1] < int32_t(d[1]) && c[2] >= 0 && c[2] < int32_t(d[2]);
}

// identical traversal rule to hw/m7/frame_depth.cpp add_ray (clip semantics)
static inline void add_ray(const int32_t origin[3], const int32_t end[3], const uint32_t dims[3], uint8_t *grid) {
    int32_t cell[3], target[3], direction[3], next[3], delta[3];
    for(int a = 0; a < 3; ++a) {
        const int32_t from = origin[a], to = end[a];
        cell[a] = from / 256; target[a] = floor_q8(to);
        direction[a] = to > from ? 1 : (to < from ? -1 : 0);
        delta[a] = to > from ? to - from : from - to;
        next[a] = direction[a] > 0 ? (cell[a] + 1) * 256 - from : from - cell[a] * 256;
    }
    for(uint32_t step = 0; step < 3 * 32; ++step) {
        if(cell[0] == target[0] && cell[1] == target[1] && cell[2] == target[2]) break;
        int first = -1;
        for(int a = 0; a < 3; ++a) {
            if(direction[a] == 0 || cell[a] == target[a]) continue;
            if(first < 0 || int64_t(next[a]) * delta[first] < int64_t(next[first]) * delta[a]) first = a;
        }
        if(first < 0) break;
        const int32_t num = next[first], den = delta[first];
        for(int a = 0; a < 3; ++a) {
            if(direction[a] == 0 || cell[a] == target[a]) continue;
            if(int64_t(next[a]) * den == int64_t(num) * delta[a]) { cell[a] += direction[a]; next[a] += 256; }
        }
        if(!in_grid(cell, dims)) break;
        if(cell[0] != target[0] || cell[1] != target[1] || cell[2] != target[2]) {
            uint8_t &g = grid[goff(cell, dims[0], dims[1])];
            if(g == 0) g = 1;
        }
    }
    if(in_grid(target, dims)) grid[goff(target, dims[0], dims[1])] = 2;
}

struct OptState {
    int threads = 1;
    std::vector<std::vector<int32_t>> ends;   // per thread, xyz triples, endpoint inside the grid
    std::vector<std::vector<int32_t>> clip;   // per thread, xyz triples, endpoint outside (walked, no HIT)
    std::vector<std::vector<uint8_t>> grids;  // per thread
    std::vector<uint32_t> inval, outside;
    std::vector<std::vector<double>> pts;     // per chunk object points
    std::vector<uint32_t> cnt6;               // per chunk counters
};

static bool opt_origin(const float wc[12], const Grid &g, int32_t origin[3]) {
    const uint32_t dims[3] = {g.nx, g.ny, g.nz};
    for(int a = 0; a < 3; ++a) {
        double c = std::floor((double(wc[4 * a + 3]) - g.origin[a]) / g.voxel * 256);
        if(!(c >= 0 && c < dims[a] * 256.0)) return false;
        origin[a] = int32_t(c);
    }
    return true;
}

// S1: validity + projection + Q8, rows split over threads
static void opt_project(const float *z, uint32_t w, uint32_t h, const float ik[9], const float wc[12], const Grid &g, OptState &st) {
    const uint32_t dims[3] = {g.nx, g.ny, g.nz};
#pragma omp parallel num_threads(st.threads)
    {
        int t = 0;
#ifdef _OPENMP
        t = omp_get_thread_num();
#endif
        auto &out = st.ends[t]; out.clear();
        auto &clp = st.clip[t]; clp.clear();
        uint32_t inv = 0, outs = 0;
#pragma omp for schedule(static)
        for(uint32_t y = 0; y < h; ++y) {
            const float v = float(y);
            const float *row = z + size_t(y) * w;
            for(uint32_t x = 0; x < w; ++x) {
                const float d = row[x];
                if(!(d > 0.1f && d < 10.0f)) { ++inv; continue; }
                const float u = float(x);
                float cam[3], wo[3];
                for(int r = 0; r < 3; ++r) cam[r] = (ik[3 * r] * u + ik[3 * r + 1] * v + ik[3 * r + 2]) * d;
                bool good = true;
                for(int r = 0; r < 3; ++r) {
                    wo[r] = wc[4 * r] * cam[0] + wc[4 * r + 1] * cam[1] + wc[4 * r + 2] * cam[2] + wc[4 * r + 3];
                    good = good && (wo[r] >= -FLT_MAX && wo[r] <= FLT_MAX);
                }
                if(!good) { ++inv; continue; }
                int32_t e[3] = {0, 0, 0}; bool inside = true, repr = true;
                for(int a = 0; a < 3; ++a) {
                    const double c = std::floor((double(wo[a]) - g.origin[a]) / g.voxel * 256);
                    if(!(c >= -16777216.0 && c < 16777216.0)) { repr = false; continue; }
                    e[a] = int32_t(c);
                    if(!(c >= 0 && c < dims[a] * 256.0)) inside = false;
                }
                if(!repr) { ++inv; continue; }
                auto &dst = inside ? out : clp;
                if(!inside) ++outs;
                dst.push_back(e[0]); dst.push_back(e[1]); dst.push_back(e[2]);
            }
        }
        st.inval[t] = inv; st.outside[t] = outs;
    }
}

// S2: per-thread grids, then max-merge
static void opt_rays(const int32_t origin[3], const Grid &g, OptState &st, uint8_t *flags) {
    const uint32_t n = g.nx * g.ny * g.nz;
#pragma omp parallel num_threads(st.threads)
    {
        int t = 0;
#ifdef _OPENMP
        t = omp_get_thread_num();
#endif
        uint8_t *grid = st.grids[t].data();
        std::memset(grid, 0, n);
        const uint32_t dims[3] = {g.nx, g.ny, g.nz};
        const auto &e = st.ends[t];
        for(size_t i = 0; i + 2 < e.size(); i += 3) add_ray(origin, &e[i], dims, grid);
        const auto &c = st.clip[t];
        for(size_t i = 0; i + 2 < c.size(); i += 3) add_ray(origin, &c[i], dims, grid);
    }
    std::memcpy(flags, st.grids[0].data(), n);
    for(int t = 1; t < st.threads; ++t) {
        const uint8_t *s = st.grids[t].data();
        for(uint32_t i = 0; i < n; ++i) flags[i] = std::max(flags[i], s[i]);
    }
}

// S3: object evidence, row chunks concatenated in order (keeps source-pixel order)
static void opt_object(const float *z, const uint8_t *mask, uint32_t w, uint32_t h, bool mask_present, bool roi_on, const uint32_t roi[4],
                       float front, float back, const double ik[9], const double lc[12], OptState &st, std::vector<double> &xyz,
                       uint32_t c[6]) {
    const int T = st.threads;
#pragma omp parallel num_threads(T)
    {
        int t = 0;
#ifdef _OPENMP
        t = omp_get_thread_num();
#endif
        const uint32_t y0 = uint32_t(uint64_t(h) * t / T), y1 = uint32_t(uint64_t(h) * (t + 1) / T);
        auto &p = st.pts[t]; p.clear();
        uint32_t mp = 0, rp = 0, rv = 0, rf = 0, rb = 0;
        for(uint32_t y = y0; y < y1; ++y) {
            const bool yroi = roi_on && y >= roi[1] && y <= roi[3];
            for(uint32_t x = 0; x < w; ++x) {
                const uint32_t i = y * w + x; const float d = z[i];
                const bool valid = d > 0.1f && d < 10.0f;
                if(yroi && x >= roi[0] && x <= roi[2]) { ++rp; if(valid) { ++rv; if(d < front) ++rf; if(d > back) ++rb; } }
                if(!mask_present || mask[i] == 0) continue;
                ++mp;
                if(!valid) continue;
                const double fx = x, fy = y, fz = d; double cam[3], pt[3];
                for(int r = 0; r < 3; ++r) cam[r] = (ik[3 * r] * fx + ik[3 * r + 1] * fy + ik[3 * r + 2]) * fz;
                bool ok = true;
                for(int r = 0; r < 3; ++r) { pt[r] = lc[4 * r] * cam[0] + lc[4 * r + 1] * cam[1] + lc[4 * r + 2] * cam[2] + lc[4 * r + 3];
                                             ok = ok && (pt[r] >= -DBL_MAX && pt[r] <= DBL_MAX); }
                if(!ok) continue;
                p.push_back(pt[0]); p.push_back(pt[1]); p.push_back(pt[2]);
            }
        }
        uint32_t *o = &st.cnt6[6 * t]; o[0] = mp; o[1] = rp; o[2] = rv; o[3] = rf; o[4] = rb; o[5] = uint32_t(p.size() / 3);
    }
    xyz.clear(); for(int k = 0; k < 6; ++k) c[k] = 0;
    for(int t = 0; t < T; ++t) { xyz.insert(xyz.end(), st.pts[t].begin(), st.pts[t].end()); for(int k = 0; k < 6; ++k) c[k] += st.cnt6[6 * t + k]; }
}

// numpy.median semantics per axis (mean of the two middle values for even n)
static void median3(const std::vector<double> &xyz, size_t n, double out[3], std::vector<double> &tmp) {
    for(int a = 0; a < 3; ++a) {
        tmp.resize(n); for(size_t i = 0; i < n; ++i) tmp[i] = xyz[3 * i + a];
        if(n == 0) { out[a] = NAN; continue; }
        size_t m = n / 2; std::nth_element(tmp.begin(), tmp.begin() + m, tmp.end()); double hi = tmp[m];
        if(n % 2) out[a] = hi; else { double lo = *std::max_element(tmp.begin(), tmp.begin() + m); out[a] = lo + (hi - lo) * 0.5; }
    }
}

static void parse4(const char *s, uint32_t r[4]) { if(sscanf(s, "%u,%u,%u,%u", &r[0], &r[1], &r[2], &r[3]) != 4) { fprintf(stderr, "bad rect\n"); exit(2); } }

int main(int argc, char **argv) {
    std::string depth_path, csv_path, arm = "opt", grid_name = "g1";
    uint32_t W = 0, H = 0, N = 0, passes = 3, warmup = 1; double pace_hz = 0; double fx = 0, fy = 0, cx = 0, cy = 0, unit = 0.001;
    uint32_t mask_r[4] = {0, 0, 0, 0}, roi_r[4] = {0, 0, 0, 0}; bool mask_set = false, roi_set = false;
    bool object_on = true, ik_set = false; float ik_arg[9] = {0}; std::string dump_path;
    for(int i = 1; i < argc; ++i) {
        std::string a = argv[i]; auto nx = [&]() { if(i + 1 >= argc) { fprintf(stderr, "missing value %s\n", a.c_str()); exit(2); } return argv[++i]; };
        if(a == "--depth") depth_path = nx(); else if(a == "--csv") csv_path = nx(); else if(a == "--arm") arm = nx();
        else if(a == "--grid") grid_name = nx(); else if(a == "--width") W = atoi(nx()); else if(a == "--height") H = atoi(nx());
        else if(a == "--frames") N = atoi(nx()); else if(a == "--passes") passes = atoi(nx()); else if(a == "--warmup") warmup = atoi(nx());
        else if(a == "--fx") fx = atof(nx()); else if(a == "--fy") fy = atof(nx()); else if(a == "--cx") cx = atof(nx()); else if(a == "--cy") cy = atof(nx());
        else if(a == "--unit-m") unit = atof(nx()); else if(a == "--mask") { parse4(nx(), mask_r); mask_set = true; }
        else if(a == "--roi") { parse4(nx(), roi_r); roi_set = true; }
        else if(a == "--pace-hz") pace_hz = atof(nx());
        else if(a == "--no-object") object_on = false;
        else if(a == "--dump") dump_path = nx();
        else if(a == "--ik") {
            const char *s = nx(); char *e = nullptr;
            for(int k = 0; k < 9; ++k) {
                ik_arg[k] = strtof(s, &e);
                if(e == s || (k < 8 && *e != ',') || (k == 8 && *e != 0)) { fprintf(stderr, "bad --ik\n"); return 2; }
                s = e + 1;
            }
            ik_set = true;
        }
        else { fprintf(stderr, "unknown arg %s\n", a.c_str()); return 2; }
    }
    if(!W || !H || !N || W > FPGA_M7_FRAME_MAX_WIDTH || H > FPGA_M7_FRAME_MAX_HEIGHT || fx <= 0) { fprintf(stderr, "bad geometry args\n"); return 2; }
    Grid g;
    if(grid_name == "g1") g = {{-2., -2., -1.}, .25, 16, 16, 20};          // tools/m7_chain_replay.py GRID
    else if(grid_name == "g2") g = {{-2.4, -2.4, -0.4}, .15, 32, 32, 32};  // full 32^3 capacity, 4.8 m cube
    else if(grid_name == "p32") g = {{-0.8, -0.8, -0.05}, 0.05, 32, 32, 32};  // tools/nano_loop/chain.py default_grid()
    else { fprintf(stderr, "grid g1|g2|p32\n"); return 2; }
    const size_t P = size_t(W) * H;
    std::vector<uint16_t> raw(P * N);
    FILE *fp = fopen(depth_path.c_str(), "rb");
    if(!fp || fread(raw.data(), 2, raw.size(), fp) != raw.size()) { fprintf(stderr, "cannot read %u frames from %s\n", N, depth_path.c_str()); return 3; }
    fclose(fp);
    if(!mask_set) { mask_r[0] = W / 2 - 24; mask_r[1] = H / 2 - 24; mask_r[2] = W / 2 + 23; mask_r[3] = H / 2 + 23; }
    if(!roi_set) { roi_r[0] = W / 4 - 32; roi_r[1] = H / 2 - 32; roi_r[2] = W / 4 + 31; roi_r[3] = H / 2 + 31; }
    std::vector<uint8_t> mask(FPGA_M7_OBJECT_MAX_PIXELS, 0), nomask(FPGA_M7_OBJECT_MAX_PIXELS, 0);
    for(uint32_t y = mask_r[1]; y <= mask_r[3]; ++y) for(uint32_t x = mask_r[0]; x <= mask_r[2]; ++x) mask[y * W + x] = 1;
    // K, inverse K (float32 as FrameNativeM7 passes it), identity camera->local pose
    const double K[9] = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    const double ikd[9] = {1 / fx, 0, -cx / fx, 0, 1 / fy, -cy / fy, 0, 0, 1};
    float ik[9]; for(int i = 0; i < 9; ++i) ik[i] = ik_set ? ik_arg[i] : float(ikd[i]);
    const float wc[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    const double lc[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    (void)K;
    int threads = 1;
#ifdef _OPENMP
    threads = omp_get_max_threads();
#endif
    OptState st; st.threads = threads; st.ends.resize(threads); st.clip.resize(threads); st.grids.assign(threads, std::vector<uint8_t>(FPGA_M7_FRAME_GRID_CAPACITY));
    st.inval.assign(threads, 0); st.outside.assign(threads, 0); st.pts.resize(threads); st.cnt6.assign(6 * threads, 0);
    for(auto &e : st.ends) e.reserve(3 * P / threads + 3 * W);

    std::vector<float> zbuf(FPGA_M7_FRAME_MAX_PIXELS, 0.f);  // reference ABI needs full-capacity buffer
    std::vector<uint8_t> flags(FPGA_M7_FRAME_GRID_CAPACITY), rflags(FPGA_M7_FRAME_GRID_CAPACITY);
    std::vector<double> xyz_ref(3 * size_t(FPGA_M7_OBJECT_MAX_PIXELS)), xyz, tmp;
    const double org[3] = {g.origin[0], g.origin[1], g.origin[2]};
    const uint32_t gn = g.nx * g.ny * g.nz;
    FILE *csv = csv_path.empty() ? nullptr : fopen(csv_path.c_str(), "w");
    if(csv) setvbuf(csv, nullptr, _IOLBF, 0);  // line-buffered: a stopped run keeps its rows (N4)
    if(csv) fprintf(csv, "arm,grid,threads,pass,frame,s0_decode_ms,s1_project_ms,s2_rays_ms,s1s2_fused_ms,s3_object_ms,e2e_ms,accepted,invalid,outside,free_voxels,hit_voxels,mask_valid,roi_valid,candidate_mask,candidate_roi\n");
    uint64_t mismatches = 0, compared = 0;
    // reference results cached per frame for equivalence check of the opt arm
    std::vector<std::vector<uint8_t>> ref_flags(N); std::vector<std::array<uint32_t, 3>> ref_counts(N);
    std::vector<std::array<uint32_t, 8>> ref_obj_m(N), ref_obj_r(N); std::vector<std::vector<double>> ref_pts(N);
    auto run_ref_frame = [&](uint32_t f, double &s0, double &s12, double &s3, uint32_t cnt[3], uint32_t om[8], uint32_t orr[8], size_t &npts, double med[3]) {
        const uint16_t *r = &raw[P * f];
        auto t = clk::now();
        for(size_t i = 0; i < P; ++i) zbuf[i] = r[i] ? float(r[i]) * float(unit) : NAN;
        s0 = ms_since(t);
        uint32_t status = 99; t = clk::now();
        fpga_m7_depth_frame(zbuf.data(), ik, wc, org, g.voxel, W, H, g.nx, g.ny, g.nz, rflags.data(), cnt, &status);
        s12 = ms_since(t);
        if(status) { fprintf(stderr, "ref status %u\n", status); exit(4); }
        t = clk::now();
        if(!object_on) { s3 = 0; npts = 0; for(int k = 0; k < 8; ++k) om[k] = orr[k] = 0; med[0] = med[1] = med[2] = NAN; return; }
        uint32_t st1 = 99, st2 = 99; const uint32_t noroi[4] = {0, 0, 0, 0};
        fpga_m7_object_evidence(zbuf.data(), mask.data(), W, H, 1, 0, noroi, 0, 0, 0.04, ikd, lc, 9, 0.7, 0.9, xyz_ref.data(), om, &st1);
        npts = om[M7_PROJECTED_VALID];
        median3(xyz_ref, npts, med, tmp);
        // ROI object: fixed historical box with camera-Z extent 1.0-1.2 m over the ROI rectangle (host box projection not timed)
        fpga_m7_object_evidence(zbuf.data(), nomask.data(), W, H, 1, 1, roi_r, 1.0, 1.2, 0.04, ikd, lc, 9, 0.7, 0.9, xyz_ref.data(), orr, &st2);
        s3 = ms_since(t);
        if(st1 || st2) { fprintf(stderr, "ref object status %u %u\n", st1, st2); exit(4); }
    };
    // Always compute reference outputs once (untimed) for the equivalence check.
    for(uint32_t f = 0; f < N; ++f) {
        double a, b, c, med[3]; uint32_t cnt[3], om[8], orr[8]; size_t np;
        run_ref_frame(f, a, b, c, cnt, om, orr, np, med);
        ref_flags[f].assign(rflags.begin(), rflags.begin() + gn);
        ref_counts[f] = {cnt[0], cnt[1], cnt[2]};
        std::copy(om, om + 8, ref_obj_m[f].begin()); std::copy(orr, orr + 8, ref_obj_r[f].begin());
        ref_pts[f].assign(xyz_ref.begin(), xyz_ref.begin() + 3 * np);
    }
    // comparator self-test: a one-voxel flip and a different frame must both be detected
    {
        auto c = ref_flags[0]; c[gn / 2] = uint8_t((c[gn / 2] + 1) % 3);
        if(c == ref_flags[0]) { fprintf(stderr, "COMPARATOR_SELFTEST_FAILED\n"); return 8; }
        printf("{\"comparator_selftest\": \"flip_detected\", \"frame0_vs_frame%u_flags_differ\": %s}\n", N - 1,
               (N > 1 && ref_flags[0] != ref_flags[N - 1]) ? "true" : "false");
    }
    uint64_t late = 0, issued = 0;
    FILE *dump = dump_path.empty() ? nullptr : fopen(dump_path.c_str(), "wb");
    if(!dump_path.empty() && !dump) { fprintf(stderr, "cannot open dump\n"); return 3; }
    struct rusage ru0, ru1; getrusage(RUSAGE_SELF, &ru0); auto wall0 = clk::now();
    const auto period = std::chrono::nanoseconds(pace_hz > 0 ? int64_t(1e9 / pace_hz) : 0);
    auto tick = clk::now();
    for(uint32_t pass = 0; pass < warmup + passes; ++pass) {
        if(pass == warmup) { getrusage(RUSAGE_SELF, &ru0); wall0 = clk::now(); }
        for(uint32_t f = 0; f < N; ++f) {
            if(pace_hz > 0) {
                tick += period;
                if(clk::now() > tick) late++; else std::this_thread::sleep_until(tick);
                issued++;
            }
            double s0 = 0, s1 = 0, s2 = 0, s12 = 0, s3 = 0, e2e = 0; uint32_t cnt[3] = {0, 0, 0}; uint32_t mvalid = 0, rvalid = 0, cm = 0, cr = 0;
            size_t freev = 0, hitv = 0;
            if(arm == "ref") {
                auto t = clk::now(); uint32_t om[8], orr[8]; size_t np; double med[3];
                run_ref_frame(f, s0, s12, s3, cnt, om, orr, np, med);
                e2e = ms_since(t);
                for(uint32_t i = 0; i < gn; ++i) { freev += rflags[i] == 1; hitv += rflags[i] == 2; }
                mvalid = om[M7_PROJECTED_VALID]; rvalid = orr[M7_ROI_VALID]; cm = om[M7_CANDIDATE]; cr = orr[M7_CANDIDATE];
            } else {
                const uint16_t *r = &raw[P * f];
                auto t0 = clk::now(); auto t = t0;
                float *z = zbuf.data();
#pragma omp parallel for num_threads(threads) schedule(static)
                for(size_t i = 0; i < P; ++i) z[i] = r[i] ? float(r[i]) * float(unit) : NAN;
                s0 = ms_since(t); t = clk::now();
                int32_t origin[3];
                if(!opt_origin(wc, g, origin)) { fprintf(stderr, "origin outside grid\n"); return 4; }
                opt_project(z, W, H, ik, wc, g, st);
                s1 = ms_since(t); t = clk::now();
                opt_rays(origin, g, st, flags.data());
                s2 = ms_since(t); t = clk::now();
                uint32_t c6[6] = {0, 0, 0, 0, 0, 0}, r6[6] = {0, 0, 0, 0, 0, 0}; const float front = float(1.0 - 0.04), back = float(1.2 + 0.04);
                size_t np = 0; uint32_t mask_px = 0;
                if(object_on) {
                opt_object(z, mask.data(), W, H, true, false, roi_r, 0, 0, ikd, lc, st, xyz, c6);
                np = c6[5]; double med[3]; median3(xyz, np, med, tmp);
                mask_px = c6[0];
                cm = (np >= 9 && double(np) / mask_px >= 0.7) ? 1 : 0;
                mvalid = uint32_t(np);
                std::vector<double> dummy;
                opt_object(z, nomask.data(), W, H, true, true, roi_r, front, back, ikd, lc, st, dummy, r6);
                }
                const uint32_t rp = r6[1], rv = r6[2], rf = r6[3], rb = r6[4];
                cr = 0;
                if(rv >= 9 && double(rv) / rp >= 0.7) { if(double(rf) / rv >= 0.5) cr = 0; else if(double(rb) / rv >= 0.9) cr = 2; }
                rvalid = rv;
                s3 = ms_since(t);
                e2e = ms_since(t0);
                uint32_t acc = 0, inv = 0, outs = 0;
                for(int k = 0; k < threads; ++k) { acc += uint32_t(st.ends[k].size() / 3); inv += st.inval[k]; outs += st.outside[k]; }
                cnt[0] = acc; cnt[1] = inv; cnt[2] = outs;
                for(uint32_t i = 0; i < gn; ++i) { freev += flags[i] == 1; hitv += flags[i] == 2; }
                // equivalence against the unchanged reference kernels
                compared++;
                bool same = std::equal(flags.begin(), flags.begin() + gn, ref_flags[f].begin()) && cnt[0] == ref_counts[f][0] &&
                            cnt[1] == ref_counts[f][1] && cnt[2] == ref_counts[f][2] && mask_px == ref_obj_m[f][M7_MASK_PIXELS] &&
                            np == ref_obj_m[f][M7_PROJECTED_VALID] && cm == ref_obj_m[f][M7_CANDIDATE] && xyz == ref_pts[f] &&
                            rp == ref_obj_r[f][M7_ROI_PIXELS] && rv == ref_obj_r[f][M7_ROI_VALID] && rf == ref_obj_r[f][M7_ROI_FRONT] &&
                            rb == ref_obj_r[f][M7_ROI_BEHIND] && cr == ref_obj_r[f][M7_CANDIDATE];
                if(!same) mismatches++;
                if(dump && pass == warmup) { fwrite(cnt, 4, 3, dump); fwrite(flags.data(), 1, gn, dump); }
            }
            if(csv && pass >= warmup)
                fprintf(csv, "%s,%s,%d,%u,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u,%u,%zu,%zu,%u,%u,%u,%u\n", arm.c_str(), grid_name.c_str(), threads,
                        pass - warmup, f, s0, s1, s2, arm == "ref" ? s12 : s1 + s2, s3, e2e, cnt[0], cnt[1], cnt[2], freev, hitv, mvalid, rvalid, cm, cr);
        }
    }
    getrusage(RUSAGE_SELF, &ru1);
    const double wall_s = std::chrono::duration<double>(clk::now() - wall0).count();
    auto tv = [](const timeval &a, const timeval &b) { return double(b.tv_sec - a.tv_sec) + 1e-6 * double(b.tv_usec - a.tv_usec); };
    if(csv) fclose(csv);
    if(dump) fclose(dump);
    printf("{\"measured_wall_s\": %.4f, \"measured_cpu_user_s\": %.4f, \"measured_cpu_sys_s\": %.4f, \"object\": %s, \"ik\": [%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]}\n",
           wall_s, tv(ru0.ru_utime, ru1.ru_utime), tv(ru0.ru_stime, ru1.ru_stime), object_on ? "true" : "false",
           ik[0], ik[1], ik[2], ik[3], ik[4], ik[5], ik[6], ik[7], ik[8]);
    printf("{\"arm\": \"%s\", \"grid\": \"%s\", \"threads\": %d, \"frames\": %u, \"passes\": %u, \"warmup\": %u, \"opt_frames_compared\": %llu, \"opt_mismatches\": %llu, "
           "\"pace_hz\": %.2f, \"paced_issued\": %llu, \"paced_late_starts\": %llu, \"mask\": [%u,%u,%u,%u], \"roi\": [%u,%u,%u,%u]}\n", arm.c_str(), grid_name.c_str(), threads, N, passes, warmup,
           (unsigned long long)compared, (unsigned long long)mismatches, pace_hz, (unsigned long long)issued, (unsigned long long)late, mask_r[0], mask_r[1], mask_r[2], mask_r[3], roi_r[0], roi_r[1], roi_r[2], roi_r[3]);
    return mismatches ? 7 : 0;
}
