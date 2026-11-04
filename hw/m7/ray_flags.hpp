#ifndef FPGA_M7_RAY_FLAGS_HPP
#define FPGA_M7_RAY_FLAGS_HPP

#include <cstdint>

// These are verification capacities, not chosen product map dimensions.
constexpr std::uint32_t FPGA_M7_AXIS_CAPACITY = 32;
constexpr std::uint32_t FPGA_M7_GRID_CAPACITY = 32 * 32 * 32;
constexpr std::uint32_t FPGA_M7_RAY_CAPACITY = 4096;
constexpr std::int32_t FPGA_M7_UNITS_PER_VOXEL = 256;

extern "C" {
// Each Q8 coordinate is in [0, dimension * 256); x changes fastest.
// Input buffers must be non-aliased and hold their declared full capacity.
// ray_valid is strictly 0/1; invalid rays are ignored, including their coords.
// flags: 0 unknown, 1 traversed free, 2 hit. One vote per voxel per frame;
// hit wins regardless of input ordering. Origin's cell is not cleared.
// status: 0 success, 1 invalid dimensions/count, 2 malformed validity flag,
// 3 out-of-range origin/endpoint on an enabled ray. On failure flags is untouched.
void fpga_m7_ray_flags(const std::int32_t origins[3 * FPGA_M7_RAY_CAPACITY],
                      const std::int32_t ends[3 * FPGA_M7_RAY_CAPACITY],
                      const std::uint32_t ray_valid[FPGA_M7_RAY_CAPACITY],
                      std::uint32_t count, std::uint32_t nx, std::uint32_t ny,
                      std::uint32_t nz,
                      std::uint8_t flags[FPGA_M7_GRID_CAPACITY],
                      std::uint32_t* status);
}
#endif
