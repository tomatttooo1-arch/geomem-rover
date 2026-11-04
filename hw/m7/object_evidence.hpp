#ifndef FPGA_M7_OBJECT_EVIDENCE_HPP
#define FPGA_M7_OBJECT_EVIDENCE_HPP

#include <cstdint>

// External-buffer capacity for one Gemini 2 depth frame. A single object is
// processed per call; this is not a claim about on-chip storage or throughput.
constexpr std::uint32_t FPGA_M7_OBJECT_MAX_WIDTH = 1280;
constexpr std::uint32_t FPGA_M7_OBJECT_MAX_HEIGHT = 800;
constexpr std::uint32_t FPGA_M7_OBJECT_MAX_PIXELS =
    FPGA_M7_OBJECT_MAX_WIDTH * FPGA_M7_OBJECT_MAX_HEIGHT;

enum FpgaM7ObjectCount : std::uint32_t {
    M7_INPUT_PIXELS = 0,
    M7_MASK_PIXELS = 1,
    M7_PROJECTED_VALID = 2,
    M7_ROI_PIXELS = 3,
    M7_ROI_VALID = 4,
    M7_ROI_FRONT = 5,
    M7_ROI_BEHIND = 6,
    M7_CANDIDATE = 7,
    M7_OBJECT_COUNT_FIELDS = 8
};

enum FpgaM7ObjectCandidate : std::uint32_t {
    M7_INDETERMINATE = 0,
    M7_SUPPORT_PRESENT = 1,
    M7_FREE_SPACE_CONFLICT = 2
};

extern "C" {
// depth is aligned optical-axis Z in metres, float32; NaN/invalid returns are
// skipped. mask is binary 0/1 in original image coordinates. If mask_present=0
// it is not read, and the result is always indeterminate. roi_xyxy is an
// inclusive [x0,y0,x1,y1] rectangle projected by the host from historical
// 3D bounds; if roi_enabled=0 it is not read. near/far are the historical box
// camera-Z extents before margin. The host validates frame identity, pose,
// calibration, full visibility of the ROI, and object association. Enabled
// ROI endpoints must be in-bounds with x1-x0>=3 and y1-y0>=3.
//
// inverse_k is row-major 3x3 float64, local_camera is row-major 3x4 float64.
// xyz_local contains a row-major compact [N,3] list for only the first
// counts[M7_PROJECTED_VALID] positions, ordered by source pixel. The host
// computes the existing numpy median; the kernel does not change the position
// summary algorithm. Array tails are undefined and must never be consumed.
//
// Threshold parameters may tighten but cannot relax the existing Config:
// min_pixels>=9, min_valid_fraction>=.7, free_fraction>=.9,
// depth_margin_m>=.04. Occlusion has fixed priority at >=.5 of valid ROI.
//
// status: 0 success; 1 dimensions/capacity; 2 mask flag/value; 3 ROI;
// 4 threshold or nonfinite calibration. Output aliasing is caller error.
// On any nonzero status, xyz_local and counts are untouched. Input and output
// buffers must be non-aliased with declared capacity. status is always written.
void fpga_m7_object_evidence(
    const float depth[FPGA_M7_OBJECT_MAX_PIXELS],
    const std::uint8_t mask[FPGA_M7_OBJECT_MAX_PIXELS],
    std::uint32_t width, std::uint32_t height,
    std::uint32_t mask_present, std::uint32_t roi_enabled,
    const std::uint32_t roi_xyxy[4],
    double near_z, double far_z, double depth_margin_m,
    const double inverse_k[9], const double local_camera[12],
    std::uint32_t min_pixels, double min_valid_fraction,
    double free_fraction,
    double xyz_local[3 * FPGA_M7_OBJECT_MAX_PIXELS],
    std::uint32_t counts[M7_OBJECT_COUNT_FIELDS],
    std::uint32_t* status);
}

// ---------------------------------------------------------------------------
// Mask-free multi-box region check (E1, docs/design/OBJECT_ROI_SPEC_20261011.md).
// A separate entry; fpga_m7_object_evidence above never writes code 3.
constexpr std::uint32_t FPGA_M7_ROI_MAX_BOXES = 32;

enum FpgaM7RoiField : std::uint32_t {
    M7_RB_OBJECT_ID = 0,
    M7_RB_ENABLED = 1,
    M7_RB_PIXELS = 2,
    M7_RB_VALID = 3,
    M7_RB_FRONT = 4,
    M7_RB_BEHIND = 5,
    M7_RB_VERDICT = 6,
    M7_RB_FIELDS = 7
};

enum FpgaM7RoiVerdict : std::uint32_t {
    M7_RB_INDETERMINATE = 0,
    // 1 (support present) is never produced: presence needs a detector mask.
    M7_RB_FREE_SPACE_CONFLICT = 2,
    M7_RB_OCCLUDED = 3
};

extern "C" {
// One pass over depth counts every enabled box at once. Rows 0..box_count-1
// of result are written (M7_RB_FIELDS per row); later rows are untouched.
// The host has already disabled boxes that are out of view, too small or have
// a corner at z<=0.1 (core.py compute_geometry); enabled boxes must satisfy
// the same ROI rules as fpga_m7_object_evidence. Verdict per box: occluded
// (front/valid>=.5) wins over free (behind/valid>=free_fraction); invalid
// depth or neither -> indeterminate. Disabled rows: id echoed, rest zero.
// status: 0 success; 1 dimensions; 3 box_count/flags/box geometry;
// 4 thresholds. On nonzero status result is untouched.
void fpga_m7_object_roi_batch(
    const float depth[FPGA_M7_OBJECT_MAX_PIXELS],
    std::uint32_t width, std::uint32_t height,
    std::uint32_t box_count,
    const std::uint32_t box_enabled[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t box_xyxy[4 * FPGA_M7_ROI_MAX_BOXES],
    const double near_z[FPGA_M7_ROI_MAX_BOXES],
    const double far_z[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t object_id[FPGA_M7_ROI_MAX_BOXES],
    double depth_margin_m, std::uint32_t min_pixels,
    double min_valid_fraction, double free_fraction,
    std::uint32_t result[M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES],
    std::uint32_t* status);
}

#endif
