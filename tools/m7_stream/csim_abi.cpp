// C ABI adapter for the C model of the streaming frame top (A1): exposes the
// reference entry point fpga_m7_depth_frame (hw/m7/frame_depth.hpp) on top of
// fpga_m7_frame_axi from hw/m7/pl/hls/m7_frame_stream.cpp, so the same ctypes
// fixture (tools/m7_clip/kernels.Kernel) drives reference and stream alike.
#include "frame_depth.hpp"

extern "C" std::uint32_t fpga_m7_frame_axi(
    const float*, std::uint8_t*, const float[9], const float[12], double,
    double, double, double, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t*, std::uint32_t*,
    std::uint32_t*, std::uint32_t, std::uint32_t, float, std::uint32_t*);

extern "C" void fpga_m7_depth_frame(
    const float depth_m[FPGA_M7_FRAME_MAX_PIXELS], const float inverse_k[9],
    const float world_camera[12], const double grid_origin_m[3],
    double voxel_size_m, std::uint32_t width, std::uint32_t height,
    std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
    std::uint8_t flags[FPGA_M7_FRAME_GRID_CAPACITY], std::uint32_t counts[3],
    std::uint32_t* status) {
    std::uint32_t depth_crc = 0;
    *status = fpga_m7_frame_axi(depth_m, flags, inverse_k, world_camera,
                               grid_origin_m[0], grid_origin_m[1],
                               grid_origin_m[2], voxel_size_m, width, height,
                               nx, ny, nz, &counts[0], &counts[1], &counts[2], 0u,
                               0u, 0.0f, &depth_crc);  // D16: float32 input (mode 0)
}
