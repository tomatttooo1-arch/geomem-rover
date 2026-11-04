// Offline DDA step statistics for the M7 frame kernel (M7_CLIP_SPEC_20261011 §7).
//
// Per frame: the same float32 projection / float64 Q8 floor / range gate as
// hw/m7/frame_depth.cpp, then the same integer traversal, instrumented to
// count steps (one step = one boundary-crossing event, i.e. one iteration of
// add_ray's loop that moves the cell; same definition as traversal_steps() in
// hw/m7/frame_dense_testbench.cpp). Accepted rays count steps until the end
// cell; clipped rays count steps up to and including the step that leaves
// the grid. The tool also rebuilds the flags from its own walk and writes
// them with the counts, so the caller can check them against the reference
// kernel byte for byte (the instrumented copy must not drift from it).
//
// Usage: ray_step_stats --depth f.bin --width W --height H --frames N
//          --unit-m U --ik a,b,...(9 float32, hex ok) --out-prefix P
// Writes P.frames.csv (per frame) P.hist.csv (steps 0..96 histogram,
// accepted and clipped, whole run) and P.dump (3 x uint32 counts + 32^3 flags
// per frame, the tools/geo_same_input dump format).
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_set>
#include <vector>

static const double ORIGIN[3] = {-0.8, -0.8, -0.05};
static const double VOXEL = 0.05;
static const uint32_t DIM = 32;
static const int32_t Q8 = 256;
static const double Q8_LIMIT = 16777216.0;

static int32_t floor_q8(int32_t v) { return v >= 0 ? v / Q8 : -((-v + Q8 - 1) / Q8); }
static bool in_grid(const int32_t c[3]) {
    for(int a = 0; a < 3; ++a) if(c[a] < 0 || c[a] >= int32_t(DIM)) return false;
    return true;
}
static uint32_t off(const int32_t c[3]) { return (uint32_t(c[2]) * DIM + uint32_t(c[1])) * DIM + uint32_t(c[0]); }

static uint32_t walk(const int32_t origin[3], const int32_t end[3], uint8_t *grid) {
    int32_t cell[3], target[3], direction[3], next[3], delta[3];
    for(int a = 0; a < 3; ++a) {
        const int32_t from = origin[a], to = end[a];
        cell[a] = from / Q8; target[a] = floor_q8(to);
        direction[a] = to > from ? 1 : (to < from ? -1 : 0);
        delta[a] = to > from ? to - from : from - to;
        next[a] = direction[a] > 0 ? (cell[a] + 1) * Q8 - from : from - cell[a] * Q8;
    }
    uint32_t steps = 0;
    for(uint32_t step = 0; step < 3 * DIM; ++step) {
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
            if(int64_t(next[a]) * den == int64_t(num) * delta[a]) { cell[a] += direction[a]; next[a] += Q8; }
        }
        ++steps;
        if(!in_grid(cell)) break;
        if(cell[0] != target[0] || cell[1] != target[1] || cell[2] != target[2]) {
            uint8_t &g = grid[off(cell)];
            if(g == 0) g = 1;
        }
    }
    if(in_grid(target)) grid[off(target)] = 2;
    return steps;
}

// Endpoint dedupe key: low 21 bits per axis (two's complement) are unique for
// |Q8| < 2^20 (4096 voxels, 205 m at 5 cm); depth < 10 m stays far below.
struct KeyHash { size_t operator()(uint64_t k) const { return std::hash<uint64_t>()(k * 0x9E3779B97F4A7C15ull); } };
static uint64_t key(const int32_t e[3]) {
    return (uint64_t(uint32_t(e[0]) & 0x1FFFFF) << 42) | (uint64_t(uint32_t(e[1]) & 0x1FFFFF) << 21) | uint64_t(uint32_t(e[2]) & 0x1FFFFF);
}

int main(int argc, char **argv) {
    std::string depth_path, prefix; uint32_t W = 0, H = 0, N = 0; double unit = 0; float ik[9]; bool ik_set = false;
    for(int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto nx = [&]() { if(i + 1 >= argc) { fprintf(stderr, "missing %s\n", a.c_str()); exit(2); } return argv[++i]; };
        if(a == "--depth") depth_path = nx(); else if(a == "--width") W = atoi(nx()); else if(a == "--height") H = atoi(nx());
        else if(a == "--frames") N = atoi(nx()); else if(a == "--unit-m") unit = atof(nx()); else if(a == "--out-prefix") prefix = nx();
        else if(a == "--ik") {
            const char *s = nx(); char *e = nullptr;
            for(int k = 0; k < 9; ++k) { ik[k] = strtof(s, &e); if(e == s || (k < 8 && *e != ',')) { fprintf(stderr, "bad --ik\n"); return 2; } s = e + 1; }
            ik_set = true;
        } else { fprintf(stderr, "unknown %s\n", a.c_str()); return 2; }
    }
    if(!W || !H || !N || unit <= 0 || !ik_set || prefix.empty()) { fprintf(stderr, "args\n"); return 2; }
    const size_t P = size_t(W) * H;
    std::vector<uint16_t> raw(P);
    FILE *fp = fopen(depth_path.c_str(), "rb");
    if(!fp) { fprintf(stderr, "open depth\n"); return 3; }
    const float wc[12] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    int32_t origin[3];
    for(int a = 0; a < 3; ++a) origin[a] = int32_t(std::floor((double(wc[4 * a + 3]) - ORIGIN[a]) / VOXEL * Q8));
    FILE *fcsv = fopen((prefix + ".frames.csv").c_str(), "w");
    FILE *fdump = fopen((prefix + ".dump").c_str(), "wb");
    fprintf(fcsv, "frame,accepted,invalid,clipped,steps_accepted,steps_clipped,max_steps_accepted,max_steps_clipped,"
                  "unique_end_accepted,unique_end_clipped,unique_end_walked,free_voxels,hit_voxels\n");
    std::vector<uint64_t> hist_acc(3 * DIM + 1, 0), hist_clip(3 * DIM + 1, 0);
    std::vector<uint8_t> grid(DIM * DIM * DIM);
    for(uint32_t f = 0; f < N; ++f) {
        if(fread(raw.data(), 2, P, fp) != P) { fprintf(stderr, "short read frame %u\n", f); return 3; }
        std::memset(grid.data(), 0, grid.size());
        uint32_t acc = 0, inv = 0, clip = 0, mxa = 0, mxc = 0; uint64_t sa = 0, sc = 0;
        std::unordered_set<uint64_t, KeyHash> ua, uc, uw;
        for(size_t p = 0; p < P; ++p) {
            const float z = raw[p] ? float(raw[p]) * float(unit) : NAN;
            if(!(z > 0.1f && z < 10.0f)) { ++inv; continue; }
            const float u = float(p % W), v = float(p / W);
            float cam[3], wo[3]; bool good = true;
            for(int r = 0; r < 3; ++r) cam[r] = (ik[3 * r] * u + ik[3 * r + 1] * v + ik[3 * r + 2]) * z;
            for(int r = 0; r < 3; ++r) {
                wo[r] = wc[4 * r] * cam[0] + wc[4 * r + 1] * cam[1] + wc[4 * r + 2] * cam[2] + wc[4 * r + 3];
                good = good && wo[r] >= -FLT_MAX && wo[r] <= FLT_MAX;
            }
            if(!good) { ++inv; continue; }
            int32_t e[3] = {0, 0, 0}; bool inside = true, repr = true;
            for(int a = 0; a < 3; ++a) {
                const double c = std::floor((double(wo[a]) - ORIGIN[a]) / VOXEL * Q8);
                if(!(c >= -Q8_LIMIT && c < Q8_LIMIT)) { repr = false; continue; }
                e[a] = int32_t(c);
                if(c < 0 || c >= double(DIM) * Q8) inside = false;
            }
            if(!repr) { ++inv; continue; }
            const uint32_t s = walk(origin, e, grid.data());
            const uint64_t k = key(e);
            uw.insert(k);
            if(inside) { ++acc; sa += s; if(s > mxa) mxa = s; hist_acc[s]++; ua.insert(k); }
            else { ++clip; sc += s; if(s > mxc) mxc = s; hist_clip[s]++; uc.insert(k); }
        }
        uint32_t fr = 0, ht = 0;
        for(uint8_t g : grid) { fr += g == 1; ht += g == 2; }
        fprintf(fcsv, "%u,%u,%u,%u,%llu,%llu,%u,%u,%zu,%zu,%zu,%u,%u\n", f, acc, inv, clip, (unsigned long long)sa,
                (unsigned long long)sc, mxa, mxc, ua.size(), uc.size(), uw.size(), fr, ht);
        const uint32_t cnt[3] = {acc, inv, clip};
        fwrite(cnt, 4, 3, fdump); fwrite(grid.data(), 1, grid.size(), fdump);
    }
    fclose(fp); fclose(fcsv); fclose(fdump);
    FILE *fh = fopen((prefix + ".hist.csv").c_str(), "w");
    fprintf(fh, "steps,accepted_rays,clipped_rays\n");
    for(uint32_t s = 0; s <= 3 * DIM; ++s) fprintf(fh, "%u,%llu,%llu\n", s, (unsigned long long)hist_acc[s], (unsigned long long)hist_clip[s]);
    fclose(fh);
    return 0;
}
