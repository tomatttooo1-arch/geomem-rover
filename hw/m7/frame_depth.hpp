#ifndef FPGA_M7_FRAME_DEPTH_HPP
#define FPGA_M7_FRAME_DEPTH_HPP

#include <cstdint>

// This is an input capacity, not a throughput or camera-mode claim.
constexpr std::uint32_t FPGA_M7_FRAME_MAX_WIDTH = 1280;
constexpr std::uint32_t FPGA_M7_FRAME_MAX_HEIGHT = 800;
constexpr std::uint32_t FPGA_M7_FRAME_MAX_PIXELS =
    FPGA_M7_FRAME_MAX_WIDTH * FPGA_M7_FRAME_MAX_HEIGHT;
constexpr std::uint32_t FPGA_M7_FRAME_GRID_AXIS = 32;
constexpr std::uint32_t FPGA_M7_FRAME_GRID_CAPACITY = 32 * 32 * 32;
constexpr std::int32_t FPGA_M7_FRAME_Q8 = 256;
// Range gate for Q8 endpoint coordinates (see M7_CLIP_SPEC_20261011.md).
constexpr std::int32_t FPGA_M7_FRAME_Q8_LIMIT = 1 << 24;

extern "C" {
// Full optical-Z depth image in row-major order. The caller supplies the same
// float32 inverse K and camera-to-world [R|t] consumed by fpga_backproject.
// The grid origin and voxel size remain float64 to match NativeM7's final
// floor((float32_world - float64_origin) / float64_voxel_size * 256).
// flags uses (z*ny+y)*nx+x and values 0 unknown, 1 free, 2 hit.
// counts: [0] accepted (endpoint inside the grid: FREE path + HIT end),
// [1] invalid (depth not in (0.1,10) m, non-finite projection, or a Q8
// endpoint coordinate outside [-2^24, 2^24)), [2] clipped (endpoint outside
// the grid: the in-grid part of the ray is walked and marked FREE, no HIT).
// Since docs/design/M7_CLIP_SPEC_20261011.md counts[2] no longer means
// "discarded"; the ABI/register name "outside" is kept for compatibility.
// status: 0 success, 1 bad dimensions, 2 non-finite grid/calibration or
// non-positive voxel size, 3 camera origin outside grid. On error counts are
// zero and flags is UNTOUCHED. The (0,0) image is a valid empty reset.
// Inputs and outputs must be non-aliased buffers with the declared capacity.
void fpga_m7_depth_frame(
    const float depth_m[FPGA_M7_FRAME_MAX_PIXELS], const float inverse_k[9],
    const float world_camera[12], const double grid_origin_m[3],
    double voxel_size_m, std::uint32_t width, std::uint32_t height,
    std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
    std::uint8_t flags[FPGA_M7_FRAME_GRID_CAPACITY], std::uint32_t counts[3],
    std::uint32_t* status);
}
#endif
