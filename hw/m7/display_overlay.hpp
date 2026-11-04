#ifndef FPGA_M7_DISPLAY_OVERLAY_HPP
#define FPGA_M7_DISPLAY_OVERLAY_HPP

#include <cstdint>

constexpr std::uint32_t FPGA_M7_DISPLAY_MAX_WIDTH = 1280;
constexpr std::uint32_t FPGA_M7_DISPLAY_MAX_HEIGHT = 800;
constexpr std::uint32_t FPGA_M7_DISPLAY_MAX_PIXELS =
    FPGA_M7_DISPLAY_MAX_WIDTH * FPGA_M7_DISPLAY_MAX_HEIGHT;

// A single invocation paints every pixel in [0,width*height), including a
// fail-closed pattern for invalid/stale control. Buffers must not alias.
// depth_mm is calibrated optical-axis Z, NOT Euclidean obstacle distance:
// 100..9999 is valid, 0..99 and 10000..65535 are missing/invalid (black).
// The host decides validity before mm quantization, so legal 0.1001 m can
// quantize to 100 mm without being mistaken for exactly 0.1 m.
// object_evidence: 0 unknown, 1 support-present, 2 one free-space conflict,
// 3 region occluded (mask-free region check only, OBJECT_ROI_SPEC s.8).
// These are observations, never a safety or object-removal decision.
// status: 0 painted; 1 invalid dimensions (output untouched); 2 malformed
// controls. Status 2 still paints every in-range pixel, but NOT as one solid
// checker: the top panel_height rows always carry the information panel,
// which reads BADFRAME for this case, and the fail-closed checker occupies
// only the rows below it. Any other non-fresh control (frame_valid 0, age
// past max_age_ms, pose_valid 0) paints the same way while keeping status 0,
// with the panel naming the reason. This note was corrected on 2026-09-21:
// display_overlay.cpp has always painted the panel here, and keeping the
// reason legible is the intended behaviour.
extern "C" void fpga_m7_display_overlay(
    const std::uint16_t depth_mm[FPGA_M7_DISPLAY_MAX_PIXELS],
    std::uint32_t width, std::uint32_t height,
    std::uint32_t frame_valid, std::uint32_t pose_valid,
    std::uint32_t age_ms, std::uint32_t max_age_ms,
    std::uint32_t object_evidence,
    std::uint32_t rgb888[FPGA_M7_DISPLAY_MAX_PIXELS],
    std::uint32_t* status);

#endif
