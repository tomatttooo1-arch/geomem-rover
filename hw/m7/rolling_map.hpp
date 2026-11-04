#ifndef FPGA_M7_ROLLING_MAP_HPP
#define FPGA_M7_ROLLING_MAP_HPP

// M7 cross-frame rolling log-odds voxel map (odom-anchored, on-chip).
//
// VERIFICATION CONFIGURATION, NOT A PRODUCT DECISION: window size, the
// log-odds constants and the observation/query capacities below were chosen
// to exercise the design against the K26 on-chip budget. Product values are
//
// Macros allow a test build with other dimensions; MX must stay a multiple of
// FPGA_M7_ROLL_LANES so the x-direction ring wrap falls on a word boundary.

#include <cstdint>

#ifndef FPGA_M7_ROLL_MX
#define FPGA_M7_ROLL_MX 80
#endif
#ifndef FPGA_M7_ROLL_MY
#define FPGA_M7_ROLL_MY 80
#endif
#ifndef FPGA_M7_ROLL_MZ
#define FPGA_M7_ROLL_MZ 32
#endif

constexpr std::uint32_t FPGA_M7_ROLL_LANES = 8;  // int8 cells per 64-bit word
constexpr std::uint32_t FPGA_M7_ROLL_DIM_X = FPGA_M7_ROLL_MX;
constexpr std::uint32_t FPGA_M7_ROLL_DIM_Y = FPGA_M7_ROLL_MY;
constexpr std::uint32_t FPGA_M7_ROLL_DIM_Z = FPGA_M7_ROLL_MZ;
constexpr std::uint32_t FPGA_M7_ROLL_WORDS_X = FPGA_M7_ROLL_MX / FPGA_M7_ROLL_LANES;
constexpr std::uint32_t FPGA_M7_ROLL_WORDS =
    FPGA_M7_ROLL_WORDS_X * FPGA_M7_ROLL_MY * FPGA_M7_ROLL_MZ;
constexpr std::uint32_t FPGA_M7_ROLL_CELLS =
    FPGA_M7_ROLL_MX * FPGA_M7_ROLL_MY * FPGA_M7_ROLL_MZ;
static_assert(FPGA_M7_ROLL_MX % 8 == 0 && FPGA_M7_ROLL_MX >= 8 &&
              FPGA_M7_ROLL_MX <= 248, "MX must be a multiple of 8 in [8,248]");
static_assert(FPGA_M7_ROLL_MY >= 1 && FPGA_M7_ROLL_MY <= 255 &&
              FPGA_M7_ROLL_MZ >= 1 && FPGA_M7_ROLL_MZ <= 255,
              "MY/MZ must fit the 8-bit delta record fields");

// One observation block = the existing per-frame kernel output layout:
// uint8 flags, (z*ny+y)*nx+x, each axis 1..32, 0 unknown/1 free/2 hit.
constexpr std::uint32_t FPGA_M7_ROLL_OBS_AXIS = 32;
constexpr std::uint32_t FPGA_M7_ROLL_OBS_CAPACITY = 32 * 32 * 32;
// A 32-cell observation row touches at most 5 packed words (worst lane
// alignment), so this bound can never overflow.
constexpr std::uint32_t FPGA_M7_ROLL_DELTA_CAPACITY = 32 * 32 * 5;
constexpr std::uint32_t FPGA_M7_ROLL_QUERY_CAPACITY = 1024;
constexpr std::uint32_t FPGA_M7_ROLL_HEADER_WORDS = 24;
constexpr std::uint32_t FPGA_M7_ROLL_MAGIC = 0x4D375246u;  // "M7RF"
// |window/observation/query base| must stay below this so int32 differences
// never overflow.
constexpr std::int32_t FPGA_M7_ROLL_COORD_LIMIT = 1 << 28;

// Log-odds in units of 1/32 nat, stored as int8 (see design doc 2.2).
constexpr std::int32_t FPGA_M7_ROLL_HIT = 32;
constexpr std::int32_t FPGA_M7_ROLL_MISS = 12;
constexpr std::int32_t FPGA_M7_ROLL_LMAX = 96;
constexpr std::int32_t FPGA_M7_ROLL_LMIN = -32;
constexpr std::int32_t FPGA_M7_ROLL_T_OCC = 16;
constexpr std::int32_t FPGA_M7_ROLL_T_FREE = -16;

enum : std::uint32_t {
    FPGA_M7_ROLL_UNKNOWN = 0, FPGA_M7_ROLL_FREE = 1, FPGA_M7_ROLL_OCCUPIED = 2
};
enum : std::uint32_t {
    FPGA_M7_ROLL_OP_RESET = 0, FPGA_M7_ROLL_OP_INTEGRATE = 1,
    FPGA_M7_ROLL_OP_QUERY = 2, FPGA_M7_ROLL_OP_DUMP = 3
};
#ifdef FPGA_M7_ROLL_GRID
// B1 navigation grid (nationals 2026-11-02, B1h). Compile-time feature bit:
// without -DFPGA_M7_ROLL_GRID this block and every GRID code path are absent,
// the core preprocesses to the pre-B1h source and op 4 stays BAD_ARGS.
// GRID is read-only (map, window, counters untouched) and reuses existing
// ports, no new interface:
//   onx/ony/onz      band_lo / band_hi / tall_lo (local z layers, D-035)
//   obs_base[0..1]   body circle centre, window-local Q16 voxels (int32)
//   obs_base[2]      body radius, Q16 voxels (uint32 bits, <= 40 voxels)
//   query_count      flags: bit0 body seed, bit1 inflation; others must be 0
//   obs[0..195]      (B1_RMAX+2)^2 Nav2 cost table when inflation is on; [0][0]
//                    = 254, last row and column 0, every row non-increasing
//                    along |dy| (any Nav2 table; else reason 4)
//   delta[]          OUTPUT, packed 8 cells per word, little-endian lanes,
//                    local order (cell ly*MX+lx -> word idx/8, lane idx%8):
//                    [0, C) grid int8 100/0/-1, [C, 2C) aux (strict|tall<<2),
//                    [2C, 3C) Nav2 cost (only with inflation), C = MX*MY/8
//   header           N_DELTA = words written (2C or 3C); on BAD_ARGS
//                    CHANGED_CELLS = reason (1 offsets 2 band 3 body 4 table
//                    5 flags); the other words as for QUERY/DUMP.
// Semantics = hw/b1_costmap/costmap.cpp (independent implementation, the
// bench tools/b1_costmap/bench_int.py checks the two word for word).
enum : std::uint32_t { FPGA_M7_ROLL_OP_GRID = 4 };
constexpr std::uint32_t FPGA_M7_ROLL_GRID_RMAX = 12;
constexpr std::uint32_t FPGA_M7_ROLL_GRID_LUT_DIM = FPGA_M7_ROLL_GRID_RMAX + 2;
constexpr std::uint32_t FPGA_M7_ROLL_GRID_LUT_SIZE =
    FPGA_M7_ROLL_GRID_LUT_DIM * FPGA_M7_ROLL_GRID_LUT_DIM;
constexpr std::uint32_t FPGA_M7_ROLL_GRID_PLANE_WORDS =
    FPGA_M7_ROLL_MX * FPGA_M7_ROLL_MY / FPGA_M7_ROLL_LANES;
static_assert(3 * FPGA_M7_ROLL_GRID_PLANE_WORDS <= FPGA_M7_ROLL_DELTA_CAPACITY,
              "grid planes must fit the delta buffer");
#endif
enum : std::uint32_t {
    FPGA_M7_ROLL_OK = 0, FPGA_M7_ROLL_UNINITIALIZED = 1, FPGA_M7_ROLL_BAD_ARGS = 2,
    FPGA_M7_ROLL_MALFORMED_OBS = 3, FPGA_M7_ROLL_STALE_EPOCH = 4,
    FPGA_M7_ROLL_STALE_FRAME = 5
};
// header[] word indices. Written on EVERY call, including rejected ones.
enum : std::uint32_t {
    FPGA_M7_ROLL_H_STATUS = 0, FPGA_M7_ROLL_H_OP = 1, FPGA_M7_ROLL_H_EPOCH = 2,
    FPGA_M7_ROLL_H_DELTA_SEQ = 3, FPGA_M7_ROLL_H_LAST_FRAME = 4,
    FPGA_M7_ROLL_H_BASE = 5,      // 5..7 window base, int32 bit patterns
    FPGA_M7_ROLL_H_OFFSET = 8,    // 8..10 ring offsets
    FPGA_M7_ROLL_H_N_DELTA = 11, FPGA_M7_ROLL_H_CHANGED_CELLS = 12,
    FPGA_M7_ROLL_H_OBS_OUTSIDE = 13, FPGA_M7_ROLL_H_OBS_INSIDE = 14,
    FPGA_M7_ROLL_H_SHIFT = 15,    // bit a: axis a shifted; bit 3: full clear
    FPGA_M7_ROLL_H_ACCEPTED = 16, FPGA_M7_ROLL_H_REJ_EPOCH = 17,
    FPGA_M7_ROLL_H_REJ_FRAME = 18, FPGA_M7_ROLL_H_REJ_MALFORMED = 19,
    FPGA_M7_ROLL_H_MAGIC = 20, FPGA_M7_ROLL_H_DIMS = 21,
    FPGA_M7_ROLL_H_HAS_FRAME = 22, FPGA_M7_ROLL_H_RESERVED = 23
};

// Delta record (uint64): bits 0..15 new tri-state of the 8 lanes (2 bits
// each), 16..23 changed-lane mask, 24..31 physical word index in the row,
// 32..39 local y, 40..47 local z. Lane j's local x is
// (8*word + j - offset_x) mod MX; world = window base + local.
// Query result (uint32): bits 0..1 state, 8..15 log-odds (int8 bits),
// bit 16 outside window (state then UNKNOWN, log-odds 0).

extern "C" {
// Resident-state HLS top. The map is a static on-chip array that persists
// between calls; URAM has no power-up content, so op RESET must run first.
//   RESET:     clear map, set epoch and window base, ring offsets 0.
//   INTEGRATE: validate everything first (epoch, strictly increasing
//              frame_seq, dims, 0/1/2 values), then roll the window to
//              window_base (clearing evicted slices), then fuse one block.
//              Rejection leaves the map, window and sequence untouched.
//   QUERY:     query_count odom voxel coordinates -> query_out.
//   DUMP:      physical-order snapshot of all words -> dump.
void fpga_m7_rolling_map(std::uint32_t op, std::uint32_t epoch,
                        std::uint32_t frame_seq,
                        const std::int32_t window_base[3],
                        const std::int32_t obs_base[3],
                        std::uint32_t onx, std::uint32_t ony, std::uint32_t onz,
                        const std::uint8_t obs[FPGA_M7_ROLL_OBS_CAPACITY],
                        std::uint32_t query_count,
                        const std::int32_t query_xyz[3 * FPGA_M7_ROLL_QUERY_CAPACITY],
                        std::uint32_t query_out[FPGA_M7_ROLL_QUERY_CAPACITY],
                        std::uint64_t delta[FPGA_M7_ROLL_DELTA_CAPACITY],
                        std::uint64_t dump[FPGA_M7_ROLL_WORDS],
                        std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]);

constexpr std::uint32_t FPGA_M7_ROLL_OBS2_BANK = FPGA_M7_ROLL_OBS_CAPACITY / 16;   // 2-bit cells, 2 banks
constexpr std::uint32_t FPGA_M7_ROLL_GTAB_BANK = 32;   // >= ceil(196 / 8) GRID table bytes per lane
// SLIM_FUSE2 (2026-11-03): the synthesized core. Same contract as
// fpga_m7_rolling_map except that the 0/1/2 check of the observation block is
// done by the caller (the AXI shell checks every byte while it reads the block
// from DDR) and handed in as obs_bad, together with the number of non-zero
// block cells (obs_nonzero, item 4). Item 6: the block arrives as 2-bit cells
// (8 per 16-bit entry, entry k = cells 8k..8k+7 = obs2[k % 2][k / 2], so any 8
// consecutive cells are two entries from the two banks); op GRID's cost table
// arrives separately (gtab, byte i at gtab[i % 8][i / 8]); query coordinates
// and results are the DDR u64 views (query_xyz: 3 int32 per query, little
// endian pairs; query_out: 2 results per word, an odd count leaves the upper
// half of the last word unspecified, as the 10-26 shell did); the core still rejects the frame
// (MALFORMED_OBS) at the same point of the status order, before any state
// change. fpga_m7_rolling_map is the C entry for host builds and testbenches:
// it runs the check itself and calls this function.
void fpga_m7_rolling_core(std::uint32_t op, std::uint32_t epoch,
                         std::uint32_t frame_seq,
                         const std::int32_t window_base[3],
                         const std::int32_t obs_base[3],
                         std::uint32_t onx, std::uint32_t ony, std::uint32_t onz,
                         bool obs_bad, std::uint16_t obs_nonzero,
                         const std::uint16_t obs2[2][FPGA_M7_ROLL_OBS2_BANK],
                         const std::uint8_t gtab[FPGA_M7_ROLL_LANES][FPGA_M7_ROLL_GTAB_BANK],
                         std::uint32_t query_count,
                         const std::uint64_t query_xyz[3 * FPGA_M7_ROLL_QUERY_CAPACITY / 2],
                         std::uint64_t query_out[FPGA_M7_ROLL_QUERY_CAPACITY / 2],
                         std::uint64_t delta[FPGA_M7_ROLL_DELTA_CAPACITY],
                         std::uint64_t dump[FPGA_M7_ROLL_WORDS],
                         std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]);

// Host-only helper: the compiled-in configuration, so wrappers never guess.
// out[0..12] = MX, MY, MZ, LANES, WORDS, OBS_AXIS, DELTA_CAP, QUERY_CAP,
// HEADER_WORDS, HIT, MISS, LMAX, LMIN; out[13..14] = T_OCC, T_FREE (as int32
// bit patterns); out[15] = mutant bitmask compiled into this build.
void fpga_m7_rolling_config(std::uint32_t out[16]);
}

// Test-only fault variants (never defined in the delivered build). Each one
// breaks exactly one rule so tests can prove they detect that failure.
//   FPGA_M7_ROLL_MUTANT_UNKNOWN_CLEARS  UNKNOWN observation applies MISS
//   FPGA_M7_ROLL_MUTANT_NO_FREE_CLEAR   FREE observation ignored
//   FPGA_M7_ROLL_MUTANT_DECAY           every accepted frame decays all cells
//   FPGA_M7_ROLL_MUTANT_SHIFT_SIGN      ring offset moved the wrong way
//   FPGA_M7_ROLL_MUTANT_NO_EVICT        evicted slices not cleared
//   FPGA_M7_ROLL_MUTANT_IGNORE_EPOCH    stale epoch accepted
static inline std::uint32_t fpga_m7_roll_mutant_mask() {
    std::uint32_t m = 0;
#ifdef FPGA_M7_ROLL_MUTANT_UNKNOWN_CLEARS
    m |= 1u;
#endif
#ifdef FPGA_M7_ROLL_MUTANT_NO_FREE_CLEAR
    m |= 2u;
#endif
#ifdef FPGA_M7_ROLL_MUTANT_DECAY
    m |= 4u;
#endif
#ifdef FPGA_M7_ROLL_MUTANT_SHIFT_SIGN
    m |= 8u;
#endif
#ifdef FPGA_M7_ROLL_MUTANT_NO_EVICT
    m |= 16u;
#endif
#ifdef FPGA_M7_ROLL_MUTANT_IGNORE_EPOCH
    m |= 32u;
#endif
    return m;
}

// Shared per-cell rule (used by the kernel AND the reference; the rule text
// is identical by design, the storage/rolling code paths are independent;
// the rule itself is checked against hand-computed known answers).
// static: each translation unit keeps its own copy, so a test build may
// compile the kernel with a mutant macro and the reference without it.
// FPGA_M7_ROLL_MUTANT_DECAY is implemented in the reference only.
static inline std::int32_t fpga_m7_roll_update(std::int32_t l, std::uint32_t obs) {
#ifdef FPGA_M7_ROLL_MUTANT_UNKNOWN_CLEARS
    if (obs == FPGA_M7_ROLL_UNKNOWN) obs = FPGA_M7_ROLL_FREE;
#endif
    if (obs == FPGA_M7_ROLL_OCCUPIED) {
        const std::int32_t v = l + FPGA_M7_ROLL_HIT;
        return v > FPGA_M7_ROLL_LMAX ? FPGA_M7_ROLL_LMAX : v;
    }
#ifndef FPGA_M7_ROLL_MUTANT_NO_FREE_CLEAR
    if (obs == FPGA_M7_ROLL_FREE) {
        const std::int32_t v = l - FPGA_M7_ROLL_MISS;
        return v < FPGA_M7_ROLL_LMIN ? FPGA_M7_ROLL_LMIN : v;
    }
#endif
    return l;
}

static inline std::uint32_t fpga_m7_roll_state(std::int32_t l) {
    return l >= FPGA_M7_ROLL_T_OCC ? FPGA_M7_ROLL_OCCUPIED
         : (l <= FPGA_M7_ROLL_T_FREE ? FPGA_M7_ROLL_FREE : FPGA_M7_ROLL_UNKNOWN);
}

#endif
