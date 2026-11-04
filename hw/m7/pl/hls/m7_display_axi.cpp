// Thin AXI shell around the unchanged fpga_m7_display_overlay core (2026-09-26).
//
// depth_mm (uint16 read) and rgb888 (uint32 write) share one m_axi master.
// rgb888 pixels are (R<<16)|(G<<8)|B with a zero top byte, i.e. exactly the
// little-endian DRM_FORMAT_XRGB8888 memory layout, so the PS DisplayPort
// controller can scan this buffer out without conversion. The core writes a
// tightly packed image (stride = width*4 bytes); the framebuffer handed to
// DRM must therefore use pitch = width*4 and the same width/height.
// status becomes ap_return (0 painted, 1 bad size/output untouched,
// 2 malformed controls/fail-closed pattern painted).
#include "../../display_overlay.hpp"

extern "C" std::uint32_t fpga_m7_display_axi(
    const std::uint16_t* depth_mm, std::uint32_t* rgb888,
    std::uint32_t width, std::uint32_t height,
    std::uint32_t frame_valid, std::uint32_t pose_valid,
    std::uint32_t age_ms, std::uint32_t max_age_ms,
    std::uint32_t object_evidence) {
#pragma HLS INTERFACE mode=m_axi port=depth_mm offset=slave bundle=gmem depth=1024000 max_widen_bitwidth=32
#pragma HLS INTERFACE mode=m_axi port=rgb888 offset=slave bundle=gmem depth=1024000 max_widen_bitwidth=32
#pragma HLS INTERFACE mode=s_axilite port=depth_mm bundle=control
#pragma HLS INTERFACE mode=s_axilite port=rgb888 bundle=control
#pragma HLS INTERFACE mode=s_axilite port=width bundle=control
#pragma HLS INTERFACE mode=s_axilite port=height bundle=control
#pragma HLS INTERFACE mode=s_axilite port=frame_valid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=pose_valid bundle=control
#pragma HLS INTERFACE mode=s_axilite port=age_ms bundle=control
#pragma HLS INTERFACE mode=s_axilite port=max_age_ms bundle=control
#pragma HLS INTERFACE mode=s_axilite port=object_evidence bundle=control
#pragma HLS INTERFACE mode=s_axilite port=return bundle=control
    std::uint32_t status = 0xFFFFFFFFu;
    fpga_m7_display_overlay(depth_mm, width, height, frame_valid, pose_valid,
                           age_ms, max_age_ms, object_evidence, rgb888,
                           &status);
    return status;
}
