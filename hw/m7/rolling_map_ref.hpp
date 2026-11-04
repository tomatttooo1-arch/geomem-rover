#ifndef FPGA_M7_ROLLING_MAP_REF_HPP
#define FPGA_M7_ROLLING_MAP_REF_HPP

// Plain CPU reference for the rolling map. Deliberately written a different
// way from the HLS top: one byte per cell in LOCAL window order, the window
// move physically copies cells (no ring offsets), and every observation cell
// is addressed through its world coordinate. Only the per-cell update rule
// (rolling_map.hpp) is shared; that rule is checked against hand-computed
// known answers in tests/test_m7_rolling.py.
//
// Multiple independent maps: all state lives in a caller-owned buffer of
// fpga_m7_roll_ref_state_bytes() bytes (no statics).

#include "rolling_map.hpp"

extern "C" {
std::uint32_t fpga_m7_roll_ref_state_bytes();
// Power-up state: uninitialised map, counters zero.
void fpga_m7_roll_ref_init(void* state);
void fpga_m7_roll_ref_reset(void* state, std::uint32_t epoch,
                           const std::int32_t window_base[3],
                           std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]);
// changes: up to FPGA_M7_ROLL_CELLS records of (gx, gy, gz, new_state) for
// every cell whose tri-state changed, in observation order (mutant decay
// records follow). header[N_DELTA] = number of records.
void fpga_m7_roll_ref_integrate(void* state, std::uint32_t epoch,
                               std::uint32_t frame_seq,
                               const std::int32_t window_base[3],
                               const std::int32_t obs_base[3],
                               std::uint32_t onx, std::uint32_t ony,
                               std::uint32_t onz,
                               const std::uint8_t obs[FPGA_M7_ROLL_OBS_CAPACITY],
                               std::int32_t changes[4 * FPGA_M7_ROLL_CELLS],
                               std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]);
void fpga_m7_roll_ref_query(void* state, std::uint32_t query_count,
                           const std::int32_t query_xyz[3 * FPGA_M7_ROLL_QUERY_CAPACITY],
                           std::uint32_t query_out[FPGA_M7_ROLL_QUERY_CAPACITY],
                           std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]);
// Local-order snapshot: cells[(lz*MY+ly)*MX+lx].
void fpga_m7_roll_ref_dump(void* state, std::int8_t cells[FPGA_M7_ROLL_CELLS],
                          std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]);
void fpga_m7_roll_ref_config(std::uint32_t out[16]);
}

#endif
