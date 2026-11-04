// Thin AXI shell around fpga_m7_object_roi_batch (E2, 2026-10-11; proposal for
// the 10-16 merge build, not yet in the block design).
//
// Same rule as m7_object_axi.cpp: move the core's arguments onto buses, add no
// computation. depth (float32, read only) is the only m_axi master (bundle
// gmem); the core reads it strictly in address order, one pixel per clock, so
// HLS turns it into long read bursts. Everything else is on s_axilite: the
// scalars, the five 32-entry box tables (input register blocks) and the
// 7x32 result table (output register block, FpgaM7RoiField order per row).
// status becomes ap_return. On nonzero status the core writes no result row,
// so the result block keeps whatever it held before the call.
#include "../../object_evidence.hpp"

extern "C" std::uint32_t fpga_m7_object_roi_axi(
    const float* depth, std::uint32_t width, std::uint32_t height,
    std::uint32_t box_count,
    const std::uint32_t box_enabled[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t box_xyxy[4 * FPGA_M7_ROI_MAX_BOXES],
    const double near_z[FPGA_M7_ROI_MAX_BOXES],
    const double far_z[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t object_id[FPGA_M7_ROI_MAX_BOXES],
    double depth_margin_m, std::uint32_t min_pixels,
    double min_valid_fraction, double free_fraction,
    std::uint32_t result[M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES]) {
#pragma HLS INTERFACE mode=m_axi port=depth offset=slave bundle=gmem depth=1024000 max_widen_bitwidth=32 max_read_burst_length=256 num_read_outstanding=8 max_write_burst_length=2 num_write_outstanding=1
#pragma HLS INTERFACE mode=s_axilite port=depth bundle=control
#pragma HLS INTERFACE mode=s_axilite port=width bundle=control
#pragma HLS INTERFACE mode=s_axilite port=height bundle=control
#pragma HLS INTERFACE mode=s_axilite port=box_count bundle=control
#pragma HLS INTERFACE mode=s_axilite port=box_enabled bundle=control
#pragma HLS INTERFACE mode=s_axilite port=box_xyxy bundle=control
#pragma HLS INTERFACE mode=s_axilite port=near_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=far_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=object_id bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_margin_m bundle=control
#pragma HLS INTERFACE mode=s_axilite port=min_pixels bundle=control
#pragma HLS INTERFACE mode=s_axilite port=min_valid_fraction bundle=control
#pragma HLS INTERFACE mode=s_axilite port=free_fraction bundle=control
#pragma HLS INTERFACE mode=s_axilite port=result bundle=control
#pragma HLS INTERFACE mode=s_axilite port=return bundle=control
    std::uint32_t status = 0xFFFFFFFFu;
    fpga_m7_object_roi_batch(depth, width, height, box_count, box_enabled, box_xyxy,
                            near_z, far_z, object_id, depth_margin_m, min_pixels,
                            min_valid_fraction, free_fraction, result, &status);
    return status;
}
