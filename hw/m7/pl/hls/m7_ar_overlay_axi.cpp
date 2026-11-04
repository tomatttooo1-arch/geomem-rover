// AXI shell for the AR overlay core (D, 2026-10-11). Replaces
// fpga_m7_display_axi in the block design (owner-approved); the legacy shell
// m7_display_axi.cpp is kept untouched for rollback.
//
// gmem0 (read only): depth (u16 mm), grid (32^3 u8 flags), boxes (49 beats).
// gmem1 (write only): fb, tightly packed XRGB8888 (pitch = width*4), i.e. the
// little-endian DRM_FORMAT_XRGB8888 layout the PS DisplayPort scans out.
// permit_i is a 1-bit wire from m7_permit_axil.permit_o (ap_none, sampled
// once per frame at start); every other scalar and the return status are s_axilite.
#include "../../ar_overlay.hpp"

extern "C" std::uint32_t fpga_m7_ar_overlay_axi(
    const fpga_m7_ar_word* depth, const fpga_m7_ar_word* grid,
    const fpga_m7_ar_word* boxes, fpga_m7_ar_word* fb,
    std::uint32_t width, std::uint32_t height,
    std::uint32_t depth_width, std::uint32_t depth_height,
    std::uint32_t depth_dx, std::uint32_t depth_dy,
    std::uint32_t frame_valid, std::uint32_t pose_valid,
    std::uint32_t age_ms, std::uint32_t age_valid, std::uint32_t max_age_ms,
    std::uint32_t replay, std::uint32_t object_evidence,
    std::uint32_t frame_id, std::uint32_t permit_present,
    bool permit_i, std::uint32_t grid_valid, std::uint32_t slice_y,
    std::int32_t slice_lo_mm, std::int32_t slice_hi_mm,
    std::uint32_t cam_cell_x, std::uint32_t cam_cell_z,
    std::uint32_t mini_x, std::uint32_t mini_y) {
#pragma HLS INTERFACE mode=m_axi port=depth offset=slave bundle=gmem0 depth=128000 max_read_burst_length=256 num_write_outstanding=1
#pragma HLS INTERFACE mode=m_axi port=grid offset=slave bundle=gmem0 depth=2048 max_read_burst_length=256 num_write_outstanding=1
#pragma HLS INTERFACE mode=m_axi port=boxes offset=slave bundle=gmem0 depth=49 max_read_burst_length=256 num_write_outstanding=1
#pragma HLS INTERFACE mode=m_axi port=fb offset=slave bundle=gmem1 depth=256000 max_write_burst_length=256 num_read_outstanding=1
#pragma HLS INTERFACE mode=s_axilite port=depth bundle=control
#pragma HLS INTERFACE mode=s_axilite port=grid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=boxes bundle=control
#pragma HLS INTERFACE mode=s_axilite port=fb bundle=control
#pragma HLS INTERFACE mode=s_axilite port=width bundle=control
#pragma HLS INTERFACE mode=s_axilite port=height bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_width bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_height bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_dx bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_dy bundle=control
#pragma HLS INTERFACE mode=s_axilite port=frame_valid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=pose_valid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=age_ms bundle=control
#pragma HLS INTERFACE mode=s_axilite port=age_valid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=max_age_ms bundle=control
#pragma HLS INTERFACE mode=s_axilite port=replay bundle=control
#pragma HLS INTERFACE mode=s_axilite port=object_evidence bundle=control
#pragma HLS INTERFACE mode=s_axilite port=frame_id bundle=control
#pragma HLS INTERFACE mode=s_axilite port=permit_present bundle=control
#pragma HLS INTERFACE mode=ap_none port=permit_i
#pragma HLS INTERFACE mode=s_axilite port=grid_valid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=slice_y bundle=control
#pragma HLS INTERFACE mode=s_axilite port=slice_lo_mm bundle=control
#pragma HLS INTERFACE mode=s_axilite port=slice_hi_mm bundle=control
#pragma HLS INTERFACE mode=s_axilite port=cam_cell_x bundle=control
#pragma HLS INTERFACE mode=s_axilite port=cam_cell_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=mini_x bundle=control
#pragma HLS INTERFACE mode=s_axilite port=mini_y bundle=control
#pragma HLS INTERFACE mode=s_axilite port=return bundle=control
    return fpga_m7_ar_overlay(depth, grid, boxes, fb, width, height,
                             depth_width, depth_height, depth_dx, depth_dy,
                             frame_valid, pose_valid, age_ms, age_valid,
                             max_age_ms, replay, object_evidence, frame_id,
                             permit_present, permit_i, grid_valid, slice_y,
                             slice_lo_mm, slice_hi_mm, cam_cell_x, cam_cell_z,
                             mini_x, mini_y);
}
