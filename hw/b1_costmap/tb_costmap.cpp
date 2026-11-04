// B1 costmap C-sim testbench (2026-11-02).
//  1 known answers: an all-zero map (everything UNKNOWN) with the body seed on;
//    one occupied band voxel -> exactly that cell lethal, its inscribed ring 253;
//    bad arguments rejected.
//  2 replay vectors written by tools/b1_costmap/bench.py --vectors (expected grid/aux
//    from the Nano's software bridge, expected cost from the exact Nav2-cost reference):
//    env B1_VECTORS=<dir>/vectors.bin. Prints TB_SUMMARY ... PASS/FAIL.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "costmap.hpp"

static std::uint64_t mapw[B1_WORDS];
static std::int8_t grid[B1_CELLS];
static std::uint8_t aux[B1_CELLS], cost[B1_CELLS], lut[B1_LUT_SIZE];

static int known_answers() {
    int bad = 0;
    std::memset(mapw, 0, sizeof(mapw));
    std::memset(lut, 0, sizeof(lut));
    // Nav2 row 0 for inflation 0.45 / scaling 3 / robot radius 0.30 (bench lut_row0); symmetric fill
    const std::uint8_t row0[10] = {254, 253, 253, 253, 253, 253, 247, 213, 183, 157};
    for (int i = 0; i < 10; ++i) { lut[i] = row0[i]; lut[i * B1_LUT_DIM] = row0[i]; }
    // body at the window centre, radius 7 voxels
    std::uint32_t st = fpga_b1_costmap(mapw, 0, 0, 0, 9, 14, 11, 1, 40 * B1_Q + 100, 40 * B1_Q + 200,
                                      7 * B1_Q, 1, lut, grid, aux, cost);
    if (st != B1_OK) { std::printf("KA empty: status %u\n", st); ++bad; }
    int free_n = 0;
    for (std::uint32_t i = 0; i < B1_CELLS; ++i) {
        if (grid[i] == 0) ++free_n;
        else if (grid[i] != -1) ++bad;
        if (aux[i] != 0) ++bad;
        if (cost[i] != (grid[i] == 0 ? 0 : 255)) ++bad;
    }
    if (free_n < 140 || free_n > 230) { std::printf("KA empty: %d free cells\n", free_n); ++bad; }
    // one occupied voxel at local (lx 10, ly 20, lz 12), offsets (3, 5, 7)
    const std::uint32_t ox = 3, oy = 5, oz = 7, lx = 10, ly = 20, lz = 12;
    const std::uint32_t px = (lx + ox) % B1_MX, py = (ly + oy) % B1_MY, pz = (lz + oz) % B1_MZ;
    mapw[(pz * B1_MY + py) * B1_WX + px / 8] = static_cast<std::uint64_t>(40) << (8 * (px % 8));
    st = fpga_b1_costmap(mapw, ox, oy, oz, 9, 14, 11, 0, 0, 0, 0, 1, lut, grid, aux, cost);
    if (st != B1_OK) { std::printf("KA one: status %u\n", st); ++bad; }
    for (std::uint32_t y = 0; y < B1_MY; ++y)
        for (std::uint32_t x = 0; x < B1_MX; ++x) {
            const std::uint32_t i = y * B1_MX + x;
            const bool hit = (x == lx && y == ly);
            if (grid[i] != (hit ? 100 : -1)) ++bad;
            if (aux[i] != (hit ? (2u | 4u) : 0u)) ++bad;   // tall: lz 12 >= tall_lo 11
            const int dx = static_cast<int>(x) - static_cast<int>(lx), dy = static_cast<int>(y) - static_cast<int>(ly);
            const int d2 = dx * dx + dy * dy;
            // UNKNOWN cells take only costs >= 253: lethal at 0, inscribed within 5 cells (Nav2 0.294 m)
            const std::uint8_t want = hit ? 254 : (d2 <= 25 && dx * dx <= 25 && dy * dy <= 25 &&
                                                   lut[(dx < 0 ? -dx : dx) * B1_LUT_DIM + (dy < 0 ? -dy : dy)] >= 253
                                                   ? 253 : 255);
            if (cost[i] != want) { if (bad < 20) std::printf("KA one: cost (%u,%u)=%u want %u\n", x, y, cost[i], want); ++bad; }
        }
    // rejected arguments
    if (fpga_b1_costmap(mapw, 80, 0, 0, 9, 14, 11, 0, 0, 0, 0, 0, lut, grid, aux, cost) != B1_BAD_OFFSETS) ++bad;
    if (fpga_b1_costmap(mapw, 0, 0, 0, 14, 9, 11, 0, 0, 0, 0, 0, lut, grid, aux, cost) != B1_BAD_BAND) ++bad;
    lut[(B1_LUT_DIM - 1) * B1_LUT_DIM + 1] = 7;
    if (fpga_b1_costmap(mapw, 0, 0, 0, 9, 14, 11, 0, 0, 0, 0, 1, lut, grid, aux, cost) != B1_BAD_LUT) ++bad;
    std::printf("KA bad=%d\n", bad);
    return bad;
}

static int vectors(const char* path, int* frames) {
    FILE* f = std::fopen(path, "rb");
    if (!f) { std::printf("no vectors at %s\n", path); return 1; }
    std::uint32_t n = 0;
    int bad = 0;
    if (std::fread(&n, 4, 1, f) != 1 || std::fread(lut, 1, B1_LUT_SIZE, f) != B1_LUT_SIZE) { std::fclose(f); return 1; }
    std::vector<std::int8_t> eg(B1_CELLS);
    std::vector<std::uint8_t> ea(B1_CELLS), ec(B1_CELLS);
    for (std::uint32_t k = 0; k < n; ++k) {
        std::uint32_t u[7]; std::int32_t c[2]; std::uint32_t r;
        if (std::fread(u, 4, 7, f) != 7 || std::fread(c, 4, 2, f) != 2 || std::fread(&r, 4, 1, f) != 1 ||
            std::fread(mapw, 8, B1_WORDS, f) != B1_WORDS || std::fread(eg.data(), 1, B1_CELLS, f) != B1_CELLS ||
            std::fread(ea.data(), 1, B1_CELLS, f) != B1_CELLS || std::fread(ec.data(), 1, B1_CELLS, f) != B1_CELLS) {
            std::fclose(f); return bad + 1;
        }
        const std::uint32_t st = fpga_b1_costmap(mapw, u[0], u[1], u[2], u[3], u[4], u[5], u[6], c[0], c[1], r, 1,
                                                lut, grid, aux, cost);
        int d = st != B1_OK;
        for (std::uint32_t i = 0; i < B1_CELLS; ++i)
            d += (grid[i] != eg[i]) + (aux[i] != ea[i]) + (cost[i] != ec[i]);
        std::printf("vector %u: status %u, %d differing values\n", k, st, d);
        bad += d;
        ++*frames;
    }
    std::fclose(f);
    return bad;
}

int main() {
    int bad = known_answers();
    int frames = 0;
    const char* p = std::getenv("B1_VECTORS");
    if (p && *p) bad += vectors(p, &frames);
    else { std::printf("B1_VECTORS not set\n"); ++bad; }
    std::printf("TB_SUMMARY frames=%d bad=%d %s\n", frames, bad, bad ? "FAIL" : "PASS");
    return bad ? 1 : 0;
}
