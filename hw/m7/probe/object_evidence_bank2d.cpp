#include "object_evidence_bank2d.hpp"

#include <cfloat>

namespace {
bool finite_double(double value) {
    return value >= -DBL_MAX && value <= DBL_MAX;
}

bool finite_float(float value) {
    return value >= -FLT_MAX && value <= FLT_MAX;
}

bool valid_depth(float z) {
    // NumPy compares a float32 depth array against these Python scalar
    // constants in float32 for the supported image dtype (NumPy 1.24).
    return z > 0.1f && z < 10.0f;
}
}

extern "C" void fpga_m7_object_evidence(
    const float depth[FPGA_M7_OBJECT_MAX_PIXELS],
    const std::uint8_t mask[FPGA_M7_OBJECT_MAX_PIXELS],
    std::uint32_t width, std::uint32_t height,
    std::uint32_t mask_present, std::uint32_t roi_enabled,
    const std::uint32_t roi_xyxy[4],
    double near_z, double far_z, double depth_margin_m,
    const double inverse_k[9], const double local_camera[12],
    std::uint32_t min_pixels, double min_valid_fraction,
    double free_fraction,
    double xyz_local[FPGA_M7_OBJECT_MAX_PIXELS][3],
    std::uint32_t counts[M7_OBJECT_COUNT_FIELDS],
    std::uint32_t* status) {
    // All input validation precedes any output writes. In particular, a
    // malformed mask near the end of the image cannot leave partial points.
    *status = 1;
    if (width == 0 || height == 0 || width > FPGA_M7_OBJECT_MAX_WIDTH ||
        height > FPGA_M7_OBJECT_MAX_HEIGHT) return;
    const std::uint32_t pixels = width * height;
    *status = 2;
    if (mask_present > 1 || roi_enabled > 1) return;
    *status = 3;
    if (roi_enabled) {
        if (roi_xyxy[0] > roi_xyxy[2] || roi_xyxy[1] > roi_xyxy[3] ||
            roi_xyxy[2] >= width || roi_xyxy[3] >= height ||
            roi_xyxy[2] - roi_xyxy[0] < 3 ||
            roi_xyxy[3] - roi_xyxy[1] < 3 ||
            !finite_double(near_z) || !finite_double(far_z) ||
            near_z <= 0.1 || far_z < near_z) return;
    }
    *status = 4;
    if (min_pixels < 9 || min_pixels > FPGA_M7_OBJECT_MAX_PIXELS ||
        !finite_double(min_valid_fraction) ||
        min_valid_fraction < 0.7 || min_valid_fraction > 1.0 ||
        !finite_double(free_fraction) ||
        free_fraction < 0.9 || free_fraction > 1.0 ||
        !finite_double(depth_margin_m) || depth_margin_m < 0.04) return;
    for (std::uint32_t i = 0; i < 9; ++i)
        if (!finite_double(inverse_k[i])) return;
    for (std::uint32_t i = 0; i < 12; ++i)
        if (!finite_double(local_camera[i])) return;
    const float front = static_cast<float>(near_z - depth_margin_m);
    const float back = static_cast<float>(far_z + depth_margin_m);
    if (roi_enabled && (!finite_float(front) || !finite_float(back) || front > back)) return;

    if (mask_present) {
        *status = 2;
        for (std::uint32_t i = 0; i < pixels; ++i)
            if (mask[i] > 1) return;
    }

    std::uint32_t mask_pixels = 0;
    std::uint32_t projected_valid = 0;
    std::uint32_t roi_pixels = 0;
    std::uint32_t roi_valid = 0;
    std::uint32_t roi_front = 0;
    std::uint32_t roi_behind = 0;
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const std::uint32_t index = y * width + x;
            const float z = depth[index];
            const bool depth_valid = valid_depth(z);
            if (roi_enabled && x >= roi_xyxy[0] && x <= roi_xyxy[2] &&
                y >= roi_xyxy[1] && y <= roi_xyxy[3]) {
                ++roi_pixels;
                if (depth_valid) {
                    ++roi_valid;
                    if (z < front) ++roi_front;
                    if (z > back) ++roi_behind;
                }
            }
            if (!mask_present || mask[index] == 0) continue;
            ++mask_pixels;
            if (!depth_valid) continue;
            const double fx = static_cast<double>(x);
            const double fy = static_cast<double>(y);
            const double fz = static_cast<double>(z);
            double camera[3];
            double point[3];
            for (std::uint32_t row = 0; row < 3; ++row) {
                camera[row] = (inverse_k[3 * row] * fx +
                               inverse_k[3 * row + 1] * fy +
                               inverse_k[3 * row + 2]) * fz;
            }
            bool point_valid = true;
            for (std::uint32_t row = 0; row < 3; ++row) {
                point[row] = local_camera[4 * row] * camera[0] +
                             local_camera[4 * row + 1] * camera[1] +
                             local_camera[4 * row + 2] * camera[2] +
                             local_camera[4 * row + 3];
                point_valid = point_valid && finite_double(point[row]);
            }
            if (!point_valid) continue;
            for (std::uint32_t row = 0; row < 3; ++row)
                xyz_local[projected_valid][row] = point[row];
            ++projected_valid;
        }
    }

    std::uint32_t candidate = M7_INDETERMINATE;
    if (mask_present) {
        if (mask_pixels > 0) {
            if (projected_valid >= min_pixels &&
                static_cast<double>(projected_valid) / mask_pixels >= min_valid_fraction)
                candidate = M7_SUPPORT_PRESENT;
        } else if (roi_enabled && roi_valid >= min_pixels &&
                   static_cast<double>(roi_valid) / roi_pixels >= min_valid_fraction) {
            // Front occlusion wins over a free-space claim.
            if (static_cast<double>(roi_front) / roi_valid >= 0.5) {
                candidate = M7_INDETERMINATE;
            } else if (static_cast<double>(roi_behind) / roi_valid >= free_fraction) {
                candidate = M7_FREE_SPACE_CONFLICT;
            }
        }
    }
    counts[M7_INPUT_PIXELS] = pixels;
    counts[M7_MASK_PIXELS] = mask_pixels;
    counts[M7_PROJECTED_VALID] = projected_valid;
    counts[M7_ROI_PIXELS] = roi_pixels;
    counts[M7_ROI_VALID] = roi_valid;
    counts[M7_ROI_FRONT] = roi_front;
    counts[M7_ROI_BEHIND] = roi_behind;
    counts[M7_CANDIDATE] = candidate;
    *status = 0;
}
