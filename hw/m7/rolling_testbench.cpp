// C simulation / C-RTL co-simulation testbench for fpga_m7_rolling_map.
//
// Observation blocks come from the EXISTING per-frame kernel
// (fpga_m7_ray_flags), so the fused input has exactly the layout and the
// hit-over-free semantics of the per-frame output. Every call is compared
// against the independent plain reference (rolling_map_ref.cpp): header
// fields, the set of changed cells, query answers and, at checkpoints, the
// complete window cell by cell. Hand-computed known answers from the update
// rule are checked on cells whose per-frame evidence is exact by geometry.
//
// Exit code 0 and the final PASS line only when every check passed. A build
// of the kernel with one FPGA_M7_ROLL_MUTANT_* macro must exit non-zero.
#include "rolling_map.hpp"
#include "rolling_map_ref.hpp"
#include "ray_flags.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

std::uint8_t obs[FPGA_M7_ROLL_OBS_CAPACITY];
std::int32_t qxyz[3 * FPGA_M7_ROLL_QUERY_CAPACITY];
std::uint32_t qout[FPGA_M7_ROLL_QUERY_CAPACITY];
std::uint32_t ref_qout[FPGA_M7_ROLL_QUERY_CAPACITY];
std::uint64_t delta_buf[FPGA_M7_ROLL_DELTA_CAPACITY];
std::uint64_t dump_buf[FPGA_M7_ROLL_WORDS];
std::uint32_t hk[FPGA_M7_ROLL_HEADER_WORDS];
std::uint32_t hr[FPGA_M7_ROLL_HEADER_WORDS];
std::int32_t ref_changes[4 * FPGA_M7_ROLL_CELLS];
std::int8_t ref_cells[FPGA_M7_ROLL_CELLS];
std::int8_t kernel_cells[FPGA_M7_ROLL_CELLS];
void* ref = nullptr;

std::int32_t origins[3 * FPGA_M7_RAY_CAPACITY];
std::int32_t ends[3 * FPGA_M7_RAY_CAPACITY];
std::uint32_t ray_valid[FPGA_M7_RAY_CAPACITY];

const std::int32_t MX = FPGA_M7_ROLL_DIM_X, MY = FPGA_M7_ROLL_DIM_Y, MZ = FPGA_M7_ROLL_DIM_Z;
const std::int32_t NO_BASE[3] = {0, 0, 0};

struct Change {
    std::int32_t x, y, z, s;
    bool operator<(const Change& o) const {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        if (z != o.z) return z < o.z;
        return s < o.s;
    }
    bool operator==(const Change& o) const {
        return x == o.x && y == o.y && z == o.z && s == o.s;
    }
};

int fail(const char* tag, int line) {
    std::printf("FAIL %s (testbench line %d)\n", tag, line);
    return 1;
}
#define CHECK(cond, tag) do { if (!(cond)) return fail(tag, __LINE__); } while (0)

void kernel_call(std::uint32_t op, std::uint32_t epoch, std::uint32_t seq,
                 const std::int32_t wb[3], const std::int32_t ob[3],
                 std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
                 std::uint32_t query_count) {
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_HEADER_WORDS; ++i) hk[i] = 0xDEADBEEFu;
    fpga_m7_rolling_map(op, epoch, seq, wb, ob, nx, ny, nz, obs, query_count, qxyz,
                       qout, delta_buf, dump_buf, hk);
}

// Header words that both implementations define identically (the ring
// offsets and the record count are implementation-specific).
int compare_headers() {
    static const std::uint32_t same[] = {
        FPGA_M7_ROLL_H_STATUS, FPGA_M7_ROLL_H_OP, FPGA_M7_ROLL_H_EPOCH,
        FPGA_M7_ROLL_H_DELTA_SEQ, FPGA_M7_ROLL_H_LAST_FRAME, FPGA_M7_ROLL_H_BASE,
        FPGA_M7_ROLL_H_BASE + 1, FPGA_M7_ROLL_H_BASE + 2,
        FPGA_M7_ROLL_H_CHANGED_CELLS, FPGA_M7_ROLL_H_OBS_OUTSIDE,
        FPGA_M7_ROLL_H_OBS_INSIDE, FPGA_M7_ROLL_H_SHIFT, FPGA_M7_ROLL_H_ACCEPTED,
        FPGA_M7_ROLL_H_REJ_EPOCH, FPGA_M7_ROLL_H_REJ_FRAME,
        FPGA_M7_ROLL_H_REJ_MALFORMED, FPGA_M7_ROLL_H_MAGIC, FPGA_M7_ROLL_H_DIMS,
        FPGA_M7_ROLL_H_HAS_FRAME};
    for (std::uint32_t i = 0; i < sizeof(same) / sizeof(same[0]); ++i) {
        if (hk[same[i]] != hr[same[i]]) {
            std::printf("header word %u kernel=%u reference=%u\n", same[i],
                        hk[same[i]], hr[same[i]]);
            return 1;
        }
    }
    return hk[FPGA_M7_ROLL_H_RESERVED] == 0 ? 0 : 1;  // no delta overflow
}

std::vector<Change> kernel_changes() {
    std::vector<Change> out;
    const std::int32_t base[3] = {static_cast<std::int32_t>(hk[FPGA_M7_ROLL_H_BASE]),
                                  static_cast<std::int32_t>(hk[FPGA_M7_ROLL_H_BASE + 1]),
                                  static_cast<std::int32_t>(hk[FPGA_M7_ROLL_H_BASE + 2])};
    const std::int32_t off_x = static_cast<std::int32_t>(hk[FPGA_M7_ROLL_H_OFFSET]);
    for (std::uint32_t i = 0; i < hk[FPGA_M7_ROLL_H_N_DELTA] &&
                              i < FPGA_M7_ROLL_DELTA_CAPACITY; ++i) {
        const std::uint64_t r = delta_buf[i];
        const std::uint32_t states = static_cast<std::uint32_t>(r & 0xFFFFu);
        const std::uint32_t mask = static_cast<std::uint32_t>((r >> 16) & 0xFFu);
        const std::int32_t w = static_cast<std::int32_t>((r >> 24) & 0xFFu);
        const std::int32_t ly = static_cast<std::int32_t>((r >> 32) & 0xFFu);
        const std::int32_t lz = static_cast<std::int32_t>((r >> 40) & 0xFFu);
        for (std::int32_t j = 0; j < 8; ++j) {
            if (!(mask & (1u << j))) continue;
            std::int32_t lx = (8 * w + j - off_x) % MX;
            if (lx < 0) lx += MX;
            Change c = {base[0] + lx, base[1] + ly, base[2] + lz,
                        static_cast<std::int32_t>((states >> (2 * j)) & 3u)};
            out.push_back(c);
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

// Kernel and reference integrate the SAME block; everything must agree.
int integrate_both(std::uint32_t epoch, std::uint32_t seq, const std::int32_t wb[3],
                   const std::int32_t ob[3], std::uint32_t nx, std::uint32_t ny,
                   std::uint32_t nz, std::uint32_t expect_status) {
    kernel_call(FPGA_M7_ROLL_OP_INTEGRATE, epoch, seq, wb, ob, nx, ny, nz, 0);
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_HEADER_WORDS; ++i) hr[i] = 0xDEADBEEFu;
    fpga_m7_roll_ref_integrate(ref, epoch, seq, wb, ob, nx, ny, nz, obs, ref_changes, hr);
    if (hk[FPGA_M7_ROLL_H_STATUS] != expect_status) {
        std::printf("integrate seq %u: kernel status %u, expected %u\n", seq,
                    hk[FPGA_M7_ROLL_H_STATUS], expect_status);
        return 1;
    }
    if (compare_headers()) return 1;
    std::vector<Change> rc;
    for (std::uint32_t i = 0; i < hr[FPGA_M7_ROLL_H_N_DELTA]; ++i) {
        Change c = {ref_changes[4 * i], ref_changes[4 * i + 1], ref_changes[4 * i + 2],
                    ref_changes[4 * i + 3]};
        rc.push_back(c);
    }
    std::sort(rc.begin(), rc.end());
    const std::vector<Change> kc = kernel_changes();
    if (!(kc == rc) || kc.size() != hk[FPGA_M7_ROLL_H_CHANGED_CELLS]) {
        std::printf("delta mismatch at seq %u: kernel %u cells, reference %u\n", seq,
                    static_cast<unsigned>(kc.size()), static_cast<unsigned>(rc.size()));
        return 1;
    }
    return 0;
}

// Full window, cell by cell (int8 log-odds), in local order.
int compare_dump() {
    kernel_call(FPGA_M7_ROLL_OP_DUMP, 0, 0, NO_BASE, NO_BASE, 0, 0, 0, 0);
    fpga_m7_roll_ref_dump(ref, ref_cells, hr);
    if (hk[FPGA_M7_ROLL_H_STATUS] != FPGA_M7_ROLL_OK || hr[FPGA_M7_ROLL_H_STATUS] != FPGA_M7_ROLL_OK)
        return 1;
    const std::int32_t off[3] = {static_cast<std::int32_t>(hk[FPGA_M7_ROLL_H_OFFSET]),
                                 static_cast<std::int32_t>(hk[FPGA_M7_ROLL_H_OFFSET + 1]),
                                 static_cast<std::int32_t>(hk[FPGA_M7_ROLL_H_OFFSET + 2])};
    std::uint32_t mismatches = 0;
    for (std::int32_t z = 0; z < MZ; ++z)
        for (std::int32_t y = 0; y < MY; ++y)
            for (std::int32_t x = 0; x < MX; ++x) {
                const std::int32_t px = (x + off[0]) % MX, py = (y + off[1]) % MY,
                                   pz = (z + off[2]) % MZ;
                const std::uint64_t word = dump_buf[(pz * MY + py) * (MX / 8) + px / 8];
                const std::int8_t v = static_cast<std::int8_t>(
                    static_cast<std::uint8_t>((word >> (8 * (px % 8))) & 0xFFu));
                kernel_cells[(z * MY + y) * MX + x] = v;
                if (v != ref_cells[(z * MY + y) * MX + x]) ++mismatches;
            }
    if (mismatches) std::printf("dump: %u of %u cells differ\n", mismatches,
                                static_cast<unsigned>(FPGA_M7_ROLL_CELLS));
    return mismatches ? 1 : 0;
}

// Query the same world cells from both; returns the kernel answer of cell 0.
int query_both(std::uint32_t count) {
    kernel_call(FPGA_M7_ROLL_OP_QUERY, 0, 0, NO_BASE, NO_BASE, 0, 0, 0, count);
    fpga_m7_roll_ref_query(ref, count, qxyz, ref_qout, hr);
    if (hk[FPGA_M7_ROLL_H_STATUS] != hr[FPGA_M7_ROLL_H_STATUS]) return 1;
    for (std::uint32_t i = 0; i < count; ++i)
        if (qout[i] != ref_qout[i]) {
            std::printf("query %u: kernel 0x%x reference 0x%x\n", i, qout[i], ref_qout[i]);
            return 1;
        }
    return 0;
}

// Query one world cell; out: state, log-odds, outside flag.
int cell(std::int32_t x, std::int32_t y, std::int32_t z, std::uint32_t* state,
         std::int32_t* logodds, bool* outside) {
    qxyz[0] = x; qxyz[1] = y; qxyz[2] = z;
    if (query_both(1)) return 1;
    *state = qout[0] & 3u;
    *logodds = static_cast<std::int8_t>(static_cast<std::uint8_t>((qout[0] >> 8) & 0xFFu));
    *outside = (qout[0] >> 16) & 1u;
    return 0;
}

// Scene in the observation grid (32 x 32 x 8 voxels): camera at the centre
// of voxel (2,16,4), a wall at x = 24, an optional box slab at x = 12
// (y 13..19, z 2..6). Rays aim at every wall voxel centre; with the box
// present, rays whose line crosses the slab end there.
const std::uint32_t ONX = 32, ONY = 32, ONZ = 8;
int make_obs(bool box, bool invalid_centre) {
    std::uint32_t n = 0;
    for (std::int32_t z = 0; z < static_cast<std::int32_t>(ONZ); ++z)
        for (std::int32_t y = 0; y < static_cast<std::int32_t>(ONY); ++y) {
            origins[3 * n] = 2 * 256 + 128;
            origins[3 * n + 1] = 16 * 256 + 128;
            origins[3 * n + 2] = 4 * 256 + 128;
            std::int32_t ex = 24, ey = y, ez = z;
            if (box) {
                const double t = 10.0 / 22.0;
                const int yb = static_cast<int>(std::floor(16.5 + t * (y + 0.5 - 16.5)));
                const int zb = static_cast<int>(std::floor(4.5 + t * (z + 0.5 - 4.5)));
                if (yb >= 13 && yb <= 19 && zb >= 2 && zb <= 6) { ex = 12; ey = yb; ez = zb; }
            }
            ends[3 * n] = ex * 256 + 128;
            ends[3 * n + 1] = ey * 256 + 128;
            ends[3 * n + 2] = ez * 256 + 128;
            ray_valid[n] = (invalid_centre && y == 16 && z == 4) ? 0u : 1u;
            ++n;
        }
    std::uint32_t st = 99;
    std::memset(obs, 0, sizeof(obs));
    fpga_m7_ray_flags(origins, ends, ray_valid, n, ONX, ONY, ONZ, obs, &st);
    return st == 0 ? 0 : 1;
}

std::uint32_t lcg_state = 12345u;
std::uint32_t lcg() {
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return lcg_state >> 8;
}

}  // namespace

int main() {
    ref = std::malloc(fpga_m7_roll_ref_state_bytes());
    CHECK(ref != nullptr, "alloc");
    fpga_m7_roll_ref_init(ref);

    // --- 0. Power-up: the resident map refuses work before RESET.
    const std::int32_t base0[3] = {-40, -40, -8};
    const std::int32_t obs_base[3] = {-2, -16, -4};  // camera at world voxel (0,0,0)
    CHECK(make_obs(false, false) == 0, "ray kernel");
    CHECK(integrate_both(7, 1, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_UNINITIALIZED) == 0,
          "uninitialised integrate rejected");
    kernel_call(FPGA_M7_ROLL_OP_DUMP, 0, 0, NO_BASE, NO_BASE, 0, 0, 0, 0);
    CHECK(hk[FPGA_M7_ROLL_H_STATUS] == FPGA_M7_ROLL_UNINITIALIZED, "uninitialised dump rejected");

    kernel_call(FPGA_M7_ROLL_OP_RESET, 7, 0, base0, NO_BASE, 0, 0, 0, 0);
    fpga_m7_roll_ref_reset(ref, 7, base0, hr);
    CHECK(hk[FPGA_M7_ROLL_H_STATUS] == 0 && compare_headers() == 0, "reset");
    CHECK(compare_dump() == 0, "reset clears every cell");
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_CELLS; ++i)
        CHECK(kernel_cells[i] == 0, "reset leaves only unknown");

    // World coordinates of the analytic cells (camera voxel = world 0,0,0).
    const std::int32_t WALL[3] = {22, 0, 0}, BOX[3] = {10, 0, 0}, PATH[3] = {3, 0, 0};
    std::uint32_t s; std::int32_t l; bool out;
    std::uint32_t seq = 0;

    // --- 1. Wall: three frames. Wall cell hit 3x: min(96, 3*32) = 96.
    for (int f = 0; f < 3; ++f) {
        CHECK(make_obs(false, false) == 0, "ray kernel");
        CHECK(integrate_both(7, ++seq, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0,
              "wall frame");
    }
    CHECK(cell(WALL[0], WALL[1], WALL[2], &s, &l, &out) == 0, "query");
    CHECK(s == FPGA_M7_ROLL_OCCUPIED && l == 96 && !out, "known answer: wall 3 hits = 96");
    CHECK(cell(PATH[0], PATH[1], PATH[2], &s, &l, &out) == 0, "query");
    CHECK(s == FPGA_M7_ROLL_FREE && l == -32, "known answer: path 3 frees = -32 (clamped)");
    CHECK(cell(BOX[0], BOX[1], BOX[2], &s, &l, &out) == 0, "query");
    CHECK(s == FPGA_M7_ROLL_FREE && l == -32, "known answer: future box cell free");
    CHECK(compare_dump() == 0, "dump after wall");

    // --- 2. Moving obstacle appears (3 frames) then leaves (7 frames).
    const std::int32_t appear[3] = {0, 32, 64};
    for (int f = 0; f < 3; ++f) {
        CHECK(make_obs(true, false) == 0, "ray kernel");
        CHECK(integrate_both(7, ++seq, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0,
              "box frame");
        CHECK(cell(BOX[0], BOX[1], BOX[2], &s, &l, &out) == 0, "query");
        CHECK(l == appear[f], "known answer: box log-odds while present");
        CHECK(s == (f == 0 ? FPGA_M7_ROLL_UNKNOWN : FPGA_M7_ROLL_OCCUPIED),
              "known answer: saturated-free cell leaves FREE on 1st hit, OCC on 2nd");
        CHECK(cell(WALL[0], WALL[1], WALL[2], &s, &l, &out) == 0, "query");
        CHECK(s == FPGA_M7_ROLL_OCCUPIED && l == 96, "occluded wall unchanged");
    }
    // After leaving: 64 - 12k -> k=4: 16 OCC, 5: 4 UNK, 6: -8 UNK, 7: -20 FREE.
    const std::int32_t leave_l[7] = {52, 40, 28, 16, 4, -8, -20};
    const std::uint32_t leave_s[7] = {2, 2, 2, 2, 0, 0, 1};
    for (int k = 0; k < 7; ++k) {
        CHECK(make_obs(false, false) == 0, "ray kernel");
        CHECK(integrate_both(7, ++seq, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0,
              "box gone frame");
        CHECK(cell(BOX[0], BOX[1], BOX[2], &s, &l, &out) == 0, "query");
        CHECK(l == leave_l[k] && s == leave_s[k],
              "known answer: only valid free observations clear the old obstacle");
    }
    CHECK(compare_dump() == 0, "dump after moving obstacle");

    // --- 3. Occlusion for 12 frames and invalid centre depth for 12 frames:
    // the wall cell behind must stay OCCUPIED at 96 (never cleared).
    for (int k = 0; k < 24; ++k) {
        CHECK(make_obs(k < 12, k >= 12) == 0, "ray kernel");
        CHECK(integrate_both(7, ++seq, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0,
              "occlusion/invalid frame");
        CHECK(cell(WALL[0], WALL[1], WALL[2], &s, &l, &out) == 0, "query");
        CHECK(s == FPGA_M7_ROLL_OCCUPIED && l == 96,
              "safety: occlusion / invalid depth never frees the wall");
    }
    CHECK(compare_dump() == 0, "dump after occlusion");

    // --- 4. Stale epoch, stale/duplicate frame, malformed value: rejected,
    // counted, map untouched.
    const std::uint32_t acc_before = hk[FPGA_M7_ROLL_H_ACCEPTED];
    CHECK(make_obs(false, false) == 0, "ray kernel");
    CHECK(integrate_both(6, seq + 1, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_STALE_EPOCH) == 0,
          "stale epoch rejected");
    CHECK(hk[FPGA_M7_ROLL_H_REJ_EPOCH] == 1, "stale epoch counted");
    CHECK(integrate_both(7, seq, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_STALE_FRAME) == 0,
          "duplicate frame rejected");
    CHECK(integrate_both(7, seq - 5, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_STALE_FRAME) == 0,
          "old frame rejected");
    CHECK(hk[FPGA_M7_ROLL_H_REJ_FRAME] == 2, "stale frames counted");
    obs[ONX * ONY * ONZ - 1] = 3;
    CHECK(integrate_both(7, seq + 1, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_MALFORMED_OBS) == 0,
          "malformed value rejected");
    const std::int32_t far_base[3] = {FPGA_M7_ROLL_COORD_LIMIT, 0, 0};
    CHECK(integrate_both(7, seq + 1, far_base, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_BAD_ARGS) == 0,
          "coordinate limit rejected");
    CHECK(integrate_both(7, seq + 1, base0, obs_base, 33, ONY, ONZ, FPGA_M7_ROLL_BAD_ARGS) == 0,
          "oversize block rejected");
    CHECK(hk[FPGA_M7_ROLL_H_ACCEPTED] == acc_before, "rejections do not count as accepted");
    CHECK(compare_dump() == 0, "rejections leave map unchanged");
    CHECK(cell(WALL[0], WALL[1], WALL[2], &s, &l, &out) == 0 && l == 96, "wall still 96");

    // --- 5. Rolling: move the window in +x (4 voxels per frame, with y/z
    // moves too) while NOTHING is observed, so only correct ring addressing
    // keeps the wall at world (22,0,0) = 96 and the path at -32. The wall
    // leaves the window at the last step. Then move back: the re-entered
    // wall must be UNKNOWN (no ghost of the evicted value).
    std::int32_t wb[3] = {base0[0], base0[1], base0[2]};
    for (int k = 0; k < 16; ++k) {
        wb[0] += 4;
        if (k == 3) { wb[1] -= 2; wb[2] += 1; }
        if (k == 7) { wb[1] += 3; wb[2] -= 2; }
        std::memset(obs, 0, sizeof(obs));
        CHECK(integrate_both(7, ++seq, wb, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0,
              "rolling forward");
        CHECK(cell(WALL[0], WALL[1], WALL[2], &s, &l, &out) == 0, "query");
        const bool inside = WALL[0] - wb[0] >= 0 && WALL[0] - wb[0] < MX;
        CHECK(out == !inside, "outside flag follows the window");
        if (inside) CHECK(s == FPGA_M7_ROLL_OCCUPIED && l == 96, "world-consistent wall");
        CHECK(cell(PATH[0], PATH[1], PATH[2], &s, &l, &out) == 0, "query");
        if (!out) CHECK(s == FPGA_M7_ROLL_FREE && l == -32, "world-consistent path");
        if (k == 15) CHECK(out, "path evicted at the end");
    }
    CHECK(compare_dump() == 0, "dump after rolling forward");
    // Back to base0 in one move of -64 voxels (< MX, ring path, not full clear).
    for (int a = 0; a < 3; ++a) wb[a] = base0[a];
    std::memset(obs, 0, sizeof(obs));  // an all-unknown block: nothing observed
    CHECK(integrate_both(7, ++seq, wb, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0,
          "rolling back");
    CHECK(hk[FPGA_M7_ROLL_H_SHIFT] == 7u, "three-axis ring move, not a full clear");
    CHECK(cell(WALL[0], WALL[1], WALL[2], &s, &l, &out) == 0, "query");
    CHECK(!out && s == FPGA_M7_ROLL_UNKNOWN && l == 0,
          "re-entered cell is UNKNOWN, not a ghost of the old wall");
    CHECK(compare_dump() == 0, "dump after rolling back");
    // Window x now starts at world 10: block x -2..9 is outside, the rest in.
    wb[0] = 10;
    CHECK(make_obs(false, false) == 0, "ray kernel");
    CHECK(integrate_both(7, ++seq, wb, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0,
          "observe with part of the block outside");
    CHECK(hk[FPGA_M7_ROLL_H_OBS_OUTSIDE] > 0 && hk[FPGA_M7_ROLL_H_OBS_INSIDE] > 0,
          "observations outside the window counted, inside fused");
    CHECK(cell(WALL[0], WALL[1], WALL[2], &s, &l, &out) == 0, "query");
    CHECK(!out && s == FPGA_M7_ROLL_OCCUPIED && l == 32, "known answer: fresh wall one hit = 32");
    // A jump of >= MX voxels clears everything.
    wb[1] += MY;
    std::memset(obs, 0, sizeof(obs));
    CHECK(integrate_both(7, ++seq, wb, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0, "jump");
    CHECK((hk[FPGA_M7_ROLL_H_SHIFT] & 8u) != 0, "full clear flagged");
    CHECK(compare_dump() == 0, "dump after jump");
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_CELLS; ++i)
        CHECK(kernel_cells[i] == 0, "jump clears to unknown");

    // --- 6. Randomised differential: random blocks/bases/moves incl. wraps.
    for (int k = 0; k < 14; ++k) {
        const std::uint32_t nx = 1 + lcg() % 32, ny = 1 + lcg() % 32, nz = 1 + lcg() % 32;
        for (std::uint32_t i = 0; i < nx * ny * nz; ++i) obs[i] = static_cast<std::uint8_t>(lcg() % 3);
        const std::int32_t ob[3] = {wb[0] + static_cast<std::int32_t>(lcg() % 100) - 20,
                                    wb[1] + static_cast<std::int32_t>(lcg() % 100) - 20,
                                    wb[2] + static_cast<std::int32_t>(lcg() % 50) - 10};
        for (int a = 0; a < 3; ++a) wb[a] += static_cast<std::int32_t>(lcg() % 21) - 10;
        if (k == 9) wb[0] -= 300;
        CHECK(integrate_both(7, ++seq, wb, ob, nx, ny, nz, FPGA_M7_ROLL_OK) == 0, "random frame");
        if (k % 5 == 4) CHECK(compare_dump() == 0, "dump after random frames");
    }
    CHECK(compare_dump() == 0, "dump after random frames");
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_QUERY_CAPACITY; ++i) {
        qxyz[3 * i] = wb[0] + static_cast<std::int32_t>(lcg() % 100) - 10;
        qxyz[3 * i + 1] = wb[1] + static_cast<std::int32_t>(lcg() % 100) - 10;
        qxyz[3 * i + 2] = wb[2] + static_cast<std::int32_t>(lcg() % 40) - 4;
    }
    CHECK(query_both(FPGA_M7_ROLL_QUERY_CAPACITY) == 0, "full-capacity query");
    kernel_call(FPGA_M7_ROLL_OP_QUERY, 0, 0, NO_BASE, NO_BASE, 0, 0, 0,
                FPGA_M7_ROLL_QUERY_CAPACITY + 1);
    CHECK(hk[FPGA_M7_ROLL_H_STATUS] == FPGA_M7_ROLL_BAD_ARGS, "oversize query rejected");
    kernel_call(9, 0, 0, NO_BASE, NO_BASE, 0, 0, 0, 0);
    CHECK(hk[FPGA_M7_ROLL_H_STATUS] == FPGA_M7_ROLL_BAD_ARGS, "unknown op rejected");

    // --- 7. New epoch (pose/version change): everything unknown, the old
    // epoch is rejected and counted, the new epoch accepts frame 1 again.
    kernel_call(FPGA_M7_ROLL_OP_RESET, 8, 0, base0, NO_BASE, 0, 0, 0, 0);
    fpga_m7_roll_ref_reset(ref, 8, base0, hr);
    CHECK(compare_headers() == 0, "second reset");
    CHECK(compare_dump() == 0, "second reset dump");
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_CELLS; ++i)
        CHECK(kernel_cells[i] == 0, "epoch change clears to unknown");
    CHECK(make_obs(false, false) == 0, "ray kernel");
    CHECK(integrate_both(7, ++seq, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_STALE_EPOCH) == 0,
          "old epoch rejected after reset");
    CHECK(hk[FPGA_M7_ROLL_H_REJ_EPOCH] == 2, "old epoch counted");
    CHECK(integrate_both(8, 1, base0, obs_base, ONX, ONY, ONZ, FPGA_M7_ROLL_OK) == 0,
          "new epoch restarts frame sequence");
    CHECK(hk[FPGA_M7_ROLL_H_DELTA_SEQ] == 1, "delta sequence restarts at 1");
    CHECK(cell(WALL[0], WALL[1], WALL[2], &s, &l, &out) == 0, "query");
    CHECK(s == FPGA_M7_ROLL_OCCUPIED && l == 32, "known answer: one hit after reset = 32");
    CHECK(compare_dump() == 0, "final dump");

    std::free(ref);
    std::printf("M7 rolling map testbench PASS (accepted=%u rej_epoch=%u rej_frame=%u "
                "rej_malformed=%u)\n", hk[FPGA_M7_ROLL_H_ACCEPTED], hk[FPGA_M7_ROLL_H_REJ_EPOCH],
                hk[FPGA_M7_ROLL_H_REJ_FRAME], hk[FPGA_M7_ROLL_H_REJ_MALFORMED]);
    return 0;
}
