#ifndef FPGA_B1_COSTMAP_HPP
#define FPGA_B1_COSTMAP_HPP

// B1 (nationals, 2026-11-02): rolling log-odds map -> 2-D navigation grid on the PL.
//
// Reads the M7 rolling map exactly as it sits in URAM (hw/m7/rolling_map.cpp:
// 8 int8 log-odds cells along x per 64-bit word, word index (pz*MY+py)*WX+w,
// physical = (local + ring offset) mod M) and produces, in LOCAL order
// (row ly, column lx, index ly*MX+lx -- the order of nav_msgs/OccupancyGrid
// data that host/rolling_nav_layer.py publishes):
//
//   grid[]  int8  100 / 0 / -1  = the OccupancyGrid Nav2's rolling_layer reads
//                 (D-035 height-band projection, then -- when body_enable --
//                 UNKNOWN cells overlapped by the body circle sent FREE)
//   aux[]   uint8 bits 0-1 strict tri-state (0 unknown, 1 free, 2 occupied,
//                 before the body seed), bit 2 "tall": an OCCUPIED band voxel
//                 at local z >= tall_lo (the swept-clearing evidence of
//                 host/rolling_nav_layer.tall_occupied), bits 3-7 zero
//   cost[]  uint8 Nav2 master costs after StaticLayer(trinary) + InflationLayer
//                 (254 lethal, 253 inscribed, 255 no information, 0 free),
//                 only when infl_enable (else all 0); see costmap.cpp.
//
// Nothing here decides policy: the Nano (or the PS) passes the band, the body
// circle and the inflation cost table, computed from the same parameters the
// Python bridge / Nav2 use. The table is cost[i][j] = Nav2 computeCost(hypot(i,j))
// when hypot(i,j) <= cell_inflation_radius, else 0 (built in double on the CPU,
// so Nav2's float quirks -- ceil(0.45/0.05)=10, 6*0.05>0.30 -- are copied, not
// re-derived in hardware).
//
// This is a C model in HLS-synthesizable style (verification stage, not on any
// board). It includes hw/m7/rolling_map.hpp read-only for the map geometry and
// the per-cell state rule.

#include <cstdint>

#include "../m7/rolling_map.hpp"

constexpr std::uint32_t B1_MX = FPGA_M7_ROLL_DIM_X;
constexpr std::uint32_t B1_MY = FPGA_M7_ROLL_DIM_Y;
constexpr std::uint32_t B1_MZ = FPGA_M7_ROLL_DIM_Z;
constexpr std::uint32_t B1_WX = FPGA_M7_ROLL_WORDS_X;
constexpr std::uint32_t B1_WORDS = FPGA_M7_ROLL_WORDS;
constexpr std::uint32_t B1_CELLS = B1_MX * B1_MY;
// Largest supported inflation radius in cells (0.60 m at 5 cm). The cost table
// is (B1_RMAX+2) x (B1_RMAX+2); row/column B1_RMAX+1 must be 0 (beyond radius).
constexpr std::uint32_t B1_RMAX = 12;
constexpr std::uint32_t B1_LUT_DIM = B1_RMAX + 2;
constexpr std::uint32_t B1_LUT_SIZE = B1_LUT_DIM * B1_LUT_DIM;
constexpr std::uint32_t B1_GCAP = B1_RMAX + 1;   // vertical distance cap
// Body circle in window-local fixed point: 1 voxel = 65536 units.
constexpr std::int32_t B1_Q = 65536;

enum : std::uint32_t {
    B1_OK = 0, B1_BAD_OFFSETS = 1, B1_BAD_BAND = 2, B1_BAD_BODY = 3, B1_BAD_LUT = 4
};
enum : std::uint8_t { B1_COST_FREE = 0, B1_COST_INSCRIBED = 253, B1_COST_LETHAL = 254,
                      B1_COST_UNKNOWN = 255 };

// Test-only fault variants (positive controls; never in a delivered build).
//   FPGA_B1_MUTANT_RING_Y   ring offset y ignored (reads the wrong rows)
//   FPGA_B1_MUTANT_UNK_FREE "not fully seen" counted as free (any FREE voxel)
// The two controls asked for in the brief (one tampered obstacle cell, a wrong
// inflation radius) are input-side and need no build variant.

extern "C" {
// status = return value. Rejection writes nothing to grid/aux/cost.
//   map        physical URAM words (in the integrated core: the resident array)
//   off_x/y/z  ring offsets (rolling core header words 8..10)
//   band_lo/hi local z layers of the obstacle band, inclusive (D-035)
//   tall_lo    first local z counted as "tall" (band_hi+1 = none)
//   body_*     circle centre in window-local Q16 voxels, radius in Q16 voxels
//   lut        (B1_RMAX+2)^2 Nav2 costs, row = |dx|, column = |dy|
std::uint32_t fpga_b1_costmap(const std::uint64_t map[B1_WORDS],
                             std::uint32_t off_x, std::uint32_t off_y, std::uint32_t off_z,
                             std::uint32_t band_lo, std::uint32_t band_hi, std::uint32_t tall_lo,
                             std::uint32_t body_enable, std::int32_t body_cx, std::int32_t body_cy,
                             std::uint32_t body_r,
                             std::uint32_t infl_enable, const std::uint8_t lut[B1_LUT_SIZE],
                             std::int8_t grid[B1_CELLS], std::uint8_t aux[B1_CELLS],
                             std::uint8_t cost[B1_CELLS]);
}

#endif
