// Thin AXI shell around the unchanged fpga_m7_object_evidence core (2026-09-26).
//
// Same rule as m7_frame_axi.cpp: move the core's arguments onto buses, add no
// computation. depth (float32 read), mask (uint8 read) and xyz_local (float64
// write) share one m_axi master (bundle gmem); ROI endpoints, thresholds and
// the float64 calibration/pose arrays are s_axilite registers copied locally.
// status becomes ap_return and the eight counts become s_axilite outputs, in
// the core's FpgaM7ObjectCount order. Only the first count_projected_valid
// xyz triples are defined; on nonzero status xyz and counts are untouched by
// the core (the shell then reports zeros for counts).
#include "../../object_evidence.hpp"

extern "C" std::uint32_t fpga_m7_object_axi(
    const float* depth, const std::uint8_t* mask, double* xyz_local,
    std::uint32_t width, std::uint32_t height,
    std::uint32_t mask_present, std::uint32_t roi_enabled,
    std::uint32_t roi_x0, std::uint32_t roi_y0,
    std::uint32_t roi_x1, std::uint32_t roi_y1,
    double near_z, double far_z, double depth_margin_m,
    const double inverse_k[9], const double local_camera[12],
    std::uint32_t min_pixels, double min_valid_fraction, double free_fraction,
    std::uint32_t* count_input, std::uint32_t* count_mask,
    std::uint32_t* count_projected_valid, std::uint32_t* count_roi,
    std::uint32_t* count_roi_valid, std::uint32_t* count_roi_front,
    std::uint32_t* count_roi_behind, std::uint32_t* candidate) {
#pragma HLS INTERFACE mode=m_axi port=depth offset=slave bundle=gmem depth=1024000 max_widen_bitwidth=32
#pragma HLS INTERFACE mode=m_axi port=mask offset=slave bundle=gmem depth=1024000 max_widen_bitwidth=32
#pragma HLS INTERFACE mode=m_axi port=xyz_local offset=slave bundle=gmem depth=3072000 max_widen_bitwidth=64
#pragma HLS INTERFACE mode=s_axilite port=depth bundle=control
#pragma HLS INTERFACE mode=s_axilite port=mask bundle=control
#pragma HLS INTERFACE mode=s_axilite port=xyz_local bundle=control
#pragma HLS INTERFACE mode=s_axilite port=width bundle=control
#pragma HLS INTERFACE mode=s_axilite port=height bundle=control
#pragma HLS INTERFACE mode=s_axilite port=mask_present bundle=control
#pragma HLS INTERFACE mode=s_axilite port=roi_enabled bundle=control
#pragma HLS INTERFACE mode=s_axilite port=roi_x0 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=roi_y0 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=roi_x1 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=roi_y1 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=near_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=far_z bundle=control
#pragma HLS INTERFACE mode=s_axilite port=depth_margin_m bundle=control
#pragma HLS INTERFACE mode=s_axilite port=inverse_k bundle=control
#pragma HLS INTERFACE mode=s_axilite port=local_camera bundle=control
#pragma HLS INTERFACE mode=s_axilite port=min_pixels bundle=control
#pragma HLS INTERFACE mode=s_axilite port=min_valid_fraction bundle=control
#pragma HLS INTERFACE mode=s_axilite port=free_fraction bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_input bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_mask bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_projected_valid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_roi bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_roi_valid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_roi_front bundle=control
#pragma HLS INTERFACE mode=s_axilite port=count_roi_behind bundle=control
#pragma HLS INTERFACE mode=s_axilite port=candidate bundle=control
#pragma HLS INTERFACE mode=s_axilite port=return bundle=control
    double local_k[9];
    double local_pose[12];
#pragma HLS ARRAY_PARTITION variable=local_k type=complete
#pragma HLS ARRAY_PARTITION variable=local_pose type=complete
    for (int i = 0; i < 9; ++i) local_k[i] = inverse_k[i];
    for (int i = 0; i < 12; ++i) local_pose[i] = local_camera[i];
    const std::uint32_t roi[4] = {roi_x0, roi_y0, roi_x1, roi_y1};
    std::uint32_t counts[M7_OBJECT_COUNT_FIELDS] = {0, 0, 0, 0, 0, 0, 0, 0};
    std::uint32_t status = 0xFFFFFFFFu;
    fpga_m7_object_evidence(depth, mask, width, height, mask_present,
                           roi_enabled, roi, near_z, far_z, depth_margin_m,
                           local_k, local_pose, min_pixels, min_valid_fraction,
                           free_fraction, xyz_local, counts, &status);
    *count_input = counts[M7_INPUT_PIXELS];
    *count_mask = counts[M7_MASK_PIXELS];
    *count_projected_valid = counts[M7_PROJECTED_VALID];
    *count_roi = counts[M7_ROI_PIXELS];
    *count_roi_valid = counts[M7_ROI_VALID];
    *count_roi_front = counts[M7_ROI_FRONT];
    *count_roi_behind = counts[M7_ROI_BEHIND];
    *candidate = counts[M7_CANDIDATE];
    return status;
}
