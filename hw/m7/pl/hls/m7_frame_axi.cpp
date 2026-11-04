// Thin AXI shell around the unchanged fpga_m7_depth_frame core (2026-09-26).
//
// Why a wrapper: the core's arrays synthesize to external ap_memory ports,
// which a Zynq MPSoC block design cannot attach to DDR. This file only moves
// the same arguments onto standard buses; it adds no computation:
//   * depth_m (read) and flags (write) become one m_axi master (bundle gmem)
//     whose base addresses the PS programs through s_axilite;
//   * the small calibration/pose arrays and all scalars become s_axilite
//     registers and are copied into local arrays before the call;
//   * status becomes ap_return, counts become three s_axilite outputs.
//     count_outside keeps its register name but, since the 2026-10-11 clip
//     semantics (docs/design/M7_CLIP_SPEC_20261011.md), counts CLIPPED rays:
//     endpoint outside the grid, in-grid part walked as FREE, no HIT.
// Buffer contracts are the core's: depth_m holds width*height float32 values
// in row-major order, flags receives all FPGA_M7_FRAME_GRID_CAPACITY bytes and
// is written ONLY when status==0 (on error the core leaves it untouched).
#include "../../frame_depth.hpp"

extern "C" std::uint32_t fpga_m7_frame_axi(
    const float* depth_m, std::uint8_t* flags,
    const float inverse_k[9], const float world_camera[12],
    double grid_origin_x, double grid_origin_y, double grid_origin_z,
    double voxel_size_m, std::uint32_t width, std::uint32_t height,
    std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
    std::uint32_t* count_accepted, std::uint32_t* count_invalid,
    std::uint32_t* count_outside) {
#pragma HLS INTERFACE mode=m_axi port=depth_m offset=slave bundle=gmem depth=1024000 max_widen_bitwidth=32
#pragma HLS INTERFACE mode=m_axi port=flags offset=slave bundle=gmem depth=32768 max_widen_bitwidth=32
#pragma HLS INTERFACE mode=s_axilite port=depth_m bundle=control
#pragma HLS INTERFACE mode=s_axilite port=flags bundle=control
#pragma HLS INTERFACE mode=s_axilite port=inverse_k bundle=control
#pragma HLS INTERFACE mode=s_axilite port=world_camera bundle=control
#pragma HLS INTERFACE mode=s_axilite port=grid_origin_x bundle=control
#pragma HLS INTERFACE mode=s_axilite port=grid_origin_y bundle=control
#pragma HLS INTERFACE mode=s_axilite port=grid_origin_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=voxel_size_m bundle=control
#pragma HLS INTERFACE mode=s_axilite port=width bundle=control
#pragma HLS INTERFACE mode=s_axilite port=height bundle=control
#pragma HLS INTERFACE mode=s_axilite port=nx bundle=control
#pragma HLS INTERFACE mode=s_axilite port=ny bundle=control
#pragma HLS INTERFACE mode=s_axilite port=nz bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_accepted bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_invalid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_outside bundle=control
#pragma HLS INTERFACE mode=s_axilite port=return bundle=control
    float local_k[9];
    float local_pose[12];
#pragma HLS ARRAY_PARTITION variable=local_k type=complete
#pragma HLS ARRAY_PARTITION variable=local_pose type=complete
    for (int i = 0; i < 9; ++i) local_k[i] = inverse_k[i];
    for (int i = 0; i < 12; ++i) local_pose[i] = world_camera[i];
    const double origin[3] = {grid_origin_x, grid_origin_y, grid_origin_z};
    std::uint32_t counts[3] = {0, 0, 0};
    std::uint32_t status = 0xFFFFFFFFu;
    fpga_m7_depth_frame(depth_m, local_k, local_pose, origin, voxel_size_m,
                       width, height, nx, ny, nz, flags, counts, &status);
    *count_accepted = counts[0];
    *count_invalid = counts[1];
    *count_outside = counts[2];
    return status;
}
