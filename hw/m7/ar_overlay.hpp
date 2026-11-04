// AR overlay display core (D, 2026-10-11). Replaces fpga_m7_display_overlay in
// the block design; the legacy display_overlay.cpp/.hpp stay untouched.
// Spec: docs/design/AR_DISPLAY_SPEC_20261011.md (sections 3-5) and the pinned
// design in docs/design/AR_DISPLAY_HANDOVER_20261011.md.
//
// Memory layout (all buffers 16-byte aligned, 128-bit beats, little endian):
//   depth : depth_width*depth_height u16 mm, row-major; pixel j of a beat is
//           bits [16j+15:16j]. Valid optical-axis Z is 100..9999 mm.
//   grid  : 32x32x32 u8 flags, byte (z*32+y)*32+x (0 unknown,1 free,2 hit).
//           Read only when grid_valid=1: 2 beats per z for layer slice_y.
//   boxes : 196 u32 = 49 beats, always read. [0] magic 0x4D374152 "M7AR",
//           [1] count <= 32, box i at [2+6i..7+6i] = x0,y0,x1,y1 (depth-image
//           coordinates, CLOSED interval), verdict (0 undecided, 2 emptied,
//           3 occluded), label (4 ASCII, byte 0 first).
//   fb    : width*height XRGB8888 (pitch width*4); pixel j of a beat is
//           bits [32j+31:32j] (DRM_FORMAT_XRGB8888).
// Return: 0 painted; 1 bad dimensions (nothing read or written); 2 malformed
// controls; 3 malformed descriptor (2 wins over 3). 2/3 still paint the full
// frame in mode 1 (BADFRAME).
#ifndef FPGA_M7_AR_OVERLAY_HPP
#define FPGA_M7_AR_OVERLAY_HPP

#include <cstdint>
#include <ap_int.h>

typedef ap_uint<128> fpga_m7_ar_word;

constexpr std::uint32_t FPGA_M7_AR_MAX_WIDTH = 1280;
constexpr std::uint32_t FPGA_M7_AR_MAX_HEIGHT = 800;
constexpr std::uint32_t FPGA_M7_AR_MAX_BOXES = 32;
constexpr std::uint32_t FPGA_M7_AR_DESC_U32 = 196;
constexpr std::uint32_t FPGA_M7_AR_DESC_WORDS = 49;
constexpr std::uint32_t FPGA_M7_AR_GRID_N = 32;
constexpr std::uint32_t FPGA_M7_AR_GRID_WORDS = 32 * 32 * 32 / 16;
constexpr std::uint32_t FPGA_M7_AR_MAGIC = 0x4D374152u;
constexpr std::uint32_t FPGA_M7_AR_DEPTH_WORDS_MAX =
    FPGA_M7_AR_MAX_WIDTH * FPGA_M7_AR_MAX_HEIGHT / 8;
constexpr std::uint32_t FPGA_M7_AR_FB_WORDS_MAX =
    FPGA_M7_AR_MAX_WIDTH * FPGA_M7_AR_MAX_HEIGHT / 4;

// Register order (shared with tools/ar_display, 24 entries) is the scalar
// argument order below: width .. mini_y. slice_lo_mm/slice_hi_mm are int32.
std::uint32_t fpga_m7_ar_overlay(
    const fpga_m7_ar_word* depth, const fpga_m7_ar_word* grid,
    const fpga_m7_ar_word* boxes, fpga_m7_ar_word* fb,
    std::uint32_t width, std::uint32_t height,
    std::uint32_t depth_width, std::uint32_t depth_height,
    std::uint32_t depth_dx, std::uint32_t depth_dy,
    std::uint32_t frame_valid, std::uint32_t pose_valid,
    std::uint32_t age_ms, std::uint32_t age_valid, std::uint32_t max_age_ms,
    std::uint32_t replay, std::uint32_t object_evidence,
    std::uint32_t frame_id, std::uint32_t permit_present,
    std::uint32_t permit_i, std::uint32_t grid_valid, std::uint32_t slice_y,
    std::int32_t slice_lo_mm, std::int32_t slice_hi_mm,
    std::uint32_t cam_cell_x, std::uint32_t cam_cell_z,
    std::uint32_t mini_x, std::uint32_t mini_y);

#endif
